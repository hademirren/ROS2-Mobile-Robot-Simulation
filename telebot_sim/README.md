# telebot_sim

Differential-drive mobile robot simulated in Gazebo with ROS2. Keyboard teleoperation over `cmd_vel`, LiDAR + odometry monitoring, RViz visualization, and a LiDAR-based safety layer that slows/blocks forward motion near obstacles and stops the robot if teleop goes quiet.

Built and tested against **ROS2 Humble on Ubuntu 22.04** with Gazebo Classic 11.

## What's in here

```
telebot_sim/
├── urdf/telebot.urdf.xacro      # robot model: chassis, 2 drive wheels, caster, lidar
│                                # + gazebo diff_drive and ray_sensor plugins
├── worlds/simple.world          # ground plane + a few obstacles for the lidar
├── launch/sim.launch.py         # gazebo + robot_state_publisher + spawn + monitor (+ safety_controller)
├── telebot_sim/
│   ├── teleop_keyboard.py       # WASD teleop, publishes Twist on /cmd_vel at 10 Hz
│   ├── state_monitor.py         # subscribes /scan + /odom, logs pose/speed/nearest obstacle
│   ├── safety_controller.py     # ROS node: /cmd_vel_raw + /scan -> /cmd_vel
│   └── safety_logic.py          # pure decision logic, no ROS imports
└── test/test_safety_logic.py    # pytest, runs without ROS
```

## Setup

```bash
# ROS2 Humble desktop (includes RViz) + gazebo integration
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-xacro

# workspace
mkdir -p ~/ros2_ws/src
cp -r telebot_sim ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

Terminal 1 (Gazebo + monitor + safety controller):
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch telebot_sim sim.launch.py
```

Terminal 2 (teleop, needs its own tty). With the safety layer on (the default), teleop has to publish on `/cmd_vel_raw`:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run telebot_sim teleop_keyboard --ros-args -r cmd_vel:=cmd_vel_raw
```

Without the safety layer (original behaviour, teleop drives `/cmd_vel` directly):
```bash
ros2 launch telebot_sim sim.launch.py enable_safety:=false
ros2 run telebot_sim teleop_keyboard
```

Terminal 3 (RViz, optional):
```bash
rviz2
```
In RViz set Fixed Frame to `odom`, then add displays: RobotModel (topic `/robot_description`), LaserScan (topic `/scan`), Odometry (topic `/odom`).

## Sanity checks

```bash
ros2 topic list                  # expect /cmd_vel /cmd_vel_raw /scan /odom /robot_description
ros2 topic echo /scan --once
ros2 topic hz /odom
ros2 run tf2_tools view_frames   # generates frames.pdf of the TF tree
```

Expected TF tree: `odom -> base_link -> {left_wheel_link, right_wheel_link, caster_link, lidar_link}`

## Safety controller

`safety_controller` sits between teleop and the diff drive plugin. It only ever limits forward motion, so the operator can always turn or reverse out of trouble.

```
enable_safety:=true (default)

teleop_keyboard --/cmd_vel_raw--> safety_controller --/cmd_vel--> gazebo diff_drive --/odom, TF-->
                                          ^
gazebo lidar ----------/scan--------------+--------------------> state_monitor, RViz

enable_safety:=false (original setup)

teleop_keyboard -----------------/cmd_vel-----------------> gazebo diff_drive
```

Teleop is not started by the launch file because it needs its own tty, so the remap to `/cmd_vel_raw` goes on the teleop command line. If something else is still publishing on `/cmd_vel` (for example teleop started without the remap), the node logs a warning saying it is being bypassed.

What it does, every 1/`publish_rate` seconds:

| State | When | Output |
|---|---|---|
| `TIMEOUT` | no `/cmd_vel_raw` message for `cmd_timeout` | zero velocity |
| `BLOCKED` | closest valid range in the front sector `< stop_distance`, or no scan for `scan_timeout` | forward speed forced to 0, turning and reversing pass through |
| `SLOWING` | closest valid range between `stop_distance` and `slow_distance` | forward speed scaled linearly toward 0 at `stop_distance` (never below `creep_speed`) |
| `CLEAR` | nothing within `slow_distance` | command passes through unchanged |

- Ranges that are inf, nan or outside `range_min`/`range_max` are ignored. No valid reading in the sector counts as clear.
- State changes are logged once per change, not per message (`BLOCKED`/`TIMEOUT` as warnings).
- Distances are measured from the lidar, which sits 0.1 m behind the front of the chassis. With the defaults the chassis stops about 0.4 m short of `box_1`.
- Timeouts run on a monotonic wall clock, not sim time. Teleop publishes on wall time, and Gazebo's `/clock` only ticks at 10 Hz, which is coarse enough to make a 0.3 s watchdog trip falsely.
- The decision logic lives in `safety_logic.py` with no ROS imports, so it is unit tested with plain pytest.

Parameters:

| Parameter | Default | Meaning |
|---|---|---|
| `stop_distance` | 0.5 | m. Block forward motion below this |
| `slow_distance` | 1.0 | m. Start scaling forward speed below this |
| `front_angle_deg` | 30.0 | Half-width of the front sector (+/-30 deg around straight ahead) |
| `cmd_timeout` | 0.3 | s. Teleop watchdog |
| `scan_timeout` | 0.5 | s. No scan for this long blocks forward motion (can't see, don't drive) |
| `creep_speed` | 0.05 | m/s. Minimum forward speed while slowing. Pure linear scaling only approaches the stop line asymptotically, so the robot would never reach `BLOCKED` |
| `hysteresis` | 0.05 | m. Extra clearance needed to leave `BLOCKED`/`SLOWING`, so readings sitting on a threshold don't make the state flicker |
| `publish_rate` | 20.0 | Hz. Output rate on `/cmd_vel` |

To try other values, launch with the safety layer off and run the node yourself:
```bash
ros2 launch telebot_sim sim.launch.py enable_safety:=false
ros2 run telebot_sim safety_controller --ros-args -p stop_distance:=0.7 -p slow_distance:=1.5
```

Limitations: it checks a front sector of a single 2D scan plane, not the robot footprint, so a corner can still clip an obstacle while turning close to it, reversing is not checked, and anything above or below the scan plane is invisible.

Unit tests (no ROS install needed):
```bash
cd telebot_sim
python3 -m pytest test/ -v
```

## Test scenarios

Start the sim with the safety layer on and teleop remapped (see Run). The robot spawns facing `box_1`, about 1.65 m ahead of the lidar. Watch the launch terminal for the `safety_controller` state logs.

| # | Scenario | Expected behaviour | How to reproduce in the sim |
|---|---|---|---|
| 1 | Clear path | `CLEAR`, command passes through unchanged | Press `w` twice (0.2 m/s). `ros2 topic echo /cmd_vel` matches `/cmd_vel_raw` until `box_1` is 1 m away |
| 2 | Slowing | `SLOWING` at about 1.0 m, speed drops as the box gets closer | Press `w` five times (0.5 m/s) from spawn, straight at `box_1` |
| 3 | Blocked forward | `BLOCKED` at 0.5 m, robot stops about 0.4 m short of the box and stays there with forward still held | Keep going from scenario 2 |
| 4 | Turning allowed while blocked | Robot rotates in place. Once the box leaves the +/-30 deg sector it drives off again, since teleop still holds forward speed | While blocked, press `a` a few times |
| 5 | Reverse allowed while blocked | Robot backs away, log goes `SLOWING` then `CLEAR` | While blocked, press `space` then `s` |
| 6 | Teleop watchdog | `TIMEOUT` within 0.3 s, robot stops even though the last command was "forward" | Quit teleop (`q`), then `ros2 topic pub -r 10 /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.2}}"` and Ctrl-C it while the robot is moving |
| 7 | inf/nan ranges | Nothing in range reads as `CLEAR`, not as an obstacle at 0 m | Turn so the front sector faces open space (most rays return inf). Edge cases are covered by `test_front_min_*` unit tests |
| 8 | Lidar lost | `BLOCKED: no scan received yet`, turning and reversing still work | `ros2 launch telebot_sim sim.launch.py enable_safety:=false`, then `ros2 run telebot_sim safety_controller --ros-args -r scan:=scan_dead` |
| 9 | Teleop not remapped | Warning that another publisher on `/cmd_vel` is bypassing the safety layer | Run `ros2 run telebot_sim teleop_keyboard` without the remap while safety is on |
| 10 | Safety disabled | Original behaviour, robot drives into `box_1` | `enable_safety:=false` and teleop without the remap |

## Gotchas that will bite you (worth understanding, they come up in interviews)

- **LaserScan in the wrong frame in RViz**: the ray sensor plugin's `<frame_name>` must exactly match the URDF link name (`lidar_link`). If they differ, RViz drops or misplaces the scan.
- **`/scan` doesn't exist**: the plugin publishes on a private `~/out` topic by default. The `<remapping>~/out:=scan</remapping>` line in the xacro is what gives you `/scan`.
- **Robot stutters while driving**: the diff drive plugin has a command timeout. That's why the teleop node republishes the last command at 10 Hz instead of only on keypress.
- **No odom->base_link transform**: `robot_state_publisher` only publishes the static tree from the URDF. The odom transform comes from `<publish_odom_tf>true</publish_odom_tf>` in the diff drive plugin.
- **use_sim_time**: nodes that consume TF or timestamps must run with `use_sim_time: true` or lookups fail with extrapolation errors once Gazebo clock and wall clock drift apart.
- **Robot tips forward on spawn**: check the caster z-offset vs wheel radius; the chassis should be level. Caster friction is zeroed (`mu1/mu2 = 0`) so it slides rather than fighting the drive wheels.

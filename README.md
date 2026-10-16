# ROS 2 Mobile Robot Simulation

A differential-drive robot in Gazebo, driven from the keyboard, with a LiDAR safety layer between teleop and the wheels. The robot slows down when something is ahead, refuses to drive forward when it gets close, and stops if teleop goes quiet.

Tested on Ubuntu 22.04 with ROS 2 Humble and Gazebo Classic 11.

```
teleop_keyboard --/cmd_vel_raw--> safety_controller --/cmd_vel--> Gazebo diff drive
                                          ^
Gazebo lidar ----------/scan--------------+--> state_monitor, RViz
```

## Files

```
telebot_sim/
├── launch/sim.launch.py         Gazebo, robot_state_publisher, spawn, state_monitor, safety_controller
├── urdf/telebot.urdf.xacro      chassis, 2 wheels, caster, lidar, diff drive plugin
├── worlds/simple.world          ground plane and three obstacles
├── telebot_sim/
│   ├── teleop_keyboard.py       WASD teleop, publishes at 10 Hz
│   ├── safety_controller.py     ROS node: /cmd_vel_raw + /scan -> /cmd_vel
│   ├── safety_logic.py          decision logic, no ROS imports
│   └── state_monitor.py         logs pose, speed and nearest obstacle
└── test/test_safety_logic.py    33 pytest tests, no ROS needed
```

## Setup

```bash
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-xacro python3-pytest

mkdir -p ~/safety_ws/src && cd ~/safety_ws/src
git clone https://github.com/hademirren/ROS2-Mobile-Robot-Simulation.git
cd ~/safety_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

In every new terminal, first run `source /opt/ros/humble/setup.bash && source ~/safety_ws/install/setup.bash`.

```bash
# terminal 1: Gazebo, state_monitor and safety_controller
ros2 launch telebot_sim sim.launch.py

# terminal 2: teleop (w/s speed, a/d turn, space stop, q quit)
ros2 run telebot_sim teleop_keyboard --ros-args -r cmd_vel:=cmd_vel_raw

# terminal 3 (optional): set Fixed Frame to odom, add RobotModel, LaserScan /scan, Odometry /odom
rviz2
```

Teleop reads raw keys, so it needs its own terminal. That's why the launch file doesn't start it and the remap goes on the teleop command line.

Without the safety layer, launch with `enable_safety:=false` and run teleop without the remap.

Unit tests are plain Python and also run on Windows. From the `telebot_sim/` folder:

```bash
python3 -m pytest test -v
```

## Safety controller

Twenty times a second it takes the latest teleop command and the closest valid LiDAR reading within ±30° of straight ahead, then picks a state:

| State | When | Output |
|---|---|---|
| TIMEOUT | no `/cmd_vel_raw` for 0.3 s | zero velocity |
| BLOCKED | obstacle under 0.5 m, or no scan for 0.5 s | forward speed 0, turning and reversing still allowed |
| SLOWING | obstacle 0.5 to 1.0 m away | forward speed scaled toward 0 at 0.5 m, never below 0.05 m/s |
| CLEAR | nothing within 1.0 m | command passes through |

Only forward motion is limited, so you can always turn or back out. Readings that are inf, nan or out of range are ignored. Leaving BLOCKED or SLOWING takes 0.05 m of extra clearance, which stops the state flickering when a reading sits on a threshold.

The watchdog matters because the Gazebo diff drive plugin keeps executing the last command forever. It runs on the wall clock: Gazebo's `/clock` only ticks at 10 Hz, and that made a 0.3 s timeout trip while teleop was still sending.

The node logs each state change once. If teleop wasn't remapped and is also publishing `/cmd_vel`, it warns that the safety layer is being bypassed.

Parameters, all settable with `-p name:=value`:

| Parameter | Default | Meaning |
|---|---|---|
| `stop_distance` | 0.5 m | block forward motion below this |
| `slow_distance` | 1.0 m | start slowing below this |
| `front_angle_deg` | 30 | half-width of the front sector |
| `cmd_timeout` | 0.3 s | teleop watchdog |
| `scan_timeout` | 0.5 s | no scan for this long blocks forward motion |
| `creep_speed` | 0.05 m/s | slowest speed while slowing, so the robot actually reaches the stop line |
| `hysteresis` | 0.05 m | extra clearance needed to leave a state |
| `publish_rate` | 20 Hz | output rate on `/cmd_vel` |

```bash
ros2 launch telebot_sim sim.launch.py enable_safety:=false
ros2 run telebot_sim safety_controller --ros-args -p stop_distance:=0.7 -p slow_distance:=1.5
```

Limitations: it checks a front sector of one 2D scan plane, not the robot's footprint. A corner can clip an obstacle while turning, reversing isn't checked, and anything above or below the scan plane is invisible.

## Test scenarios

The robot spawns facing `box_1`, about 1.65 m from the lidar. Watch terminal 1 for the state changes.

| # | Scenario | Expected | How |
|---|---|---|---|
| 1 | Clear path | CLEAR, command unchanged | press `w` twice; `/cmd_vel` matches `/cmd_vel_raw` until the box is 1 m away |
| 2 | Slowing | SLOWING at about 1.0 m, speed drops as the box gets closer | press `w` five times from spawn |
| 3 | Blocked | BLOCKED at 0.5 m; the chassis stops about 0.4 m short of the box with forward still held | keep going from 2 |
| 4 | Turn while blocked | turns in place, drives off once the box leaves the ±30° sector | while blocked, press `a` a few times |
| 5 | Reverse while blocked | backs away, SLOWING then CLEAR | while blocked, press `space` then `s` |
| 6 | Teleop watchdog | TIMEOUT within 0.3 s, robot stops | quit teleop, run `ros2 topic pub -r 10 /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.2}}"` and Ctrl-C it mid-drive |
| 7 | inf/nan ranges | open space reads CLEAR, not an obstacle at 0 m | face open space; the `test_front_min_*` unit tests cover the edge cases |
| 8 | Lidar lost | BLOCKED with "no recent lidar scan"; turning and reversing still work | `enable_safety:=false`, then `ros2 run telebot_sim safety_controller --ros-args -r scan:=scan_dead` |
| 9 | Teleop not remapped | bypass warning | run teleop without the remap while safety is on |
| 10 | Safety off | robot drives into `box_1` | `enable_safety:=false`, teleop without the remap |

## Debugging notes

- No `/scan` topic: the ray sensor plugin publishes on `~/out` unless remapped. The xacro remaps it to `scan`.
- Scan drawn in the wrong place in RViz: the plugin's `<frame_name>` has to match the URDF link, `lidar_link`.
- No `odom -> base_link` transform: it comes from `publish_odom_tf` in the diff drive plugin, not from robot_state_publisher.
- Nodes that use TF or message stamps need `use_sim_time: true`.
- Quick checks: `ros2 topic list`, `ros2 topic echo /scan --once`, `ros2 topic hz /odom`, `ros2 run tf2_tools view_frames`. The TF tree should be `odom -> base_link -> {left_wheel_link, right_wheel_link, caster_link, lidar_link}`.
- PyCharm: start it from a sourced terminal so `rclpy` resolves, open `telebot_sim/`, use `/usr/bin/python3`, and run teleop in a real terminal.

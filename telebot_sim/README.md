# telebot_sim

Differential-drive mobile robot simulated in Gazebo with ROS2. Keyboard teleoperation over `cmd_vel`, LiDAR + odometry monitoring, RViz visualization.

Built and tested against **ROS2 Humble on Ubuntu 22.04** with Gazebo Classic 11.

## What's in here

```
telebot_sim/
├── urdf/telebot.urdf.xacro      # robot model: chassis, 2 drive wheels, caster, lidar
│                                # + gazebo diff_drive and ray_sensor plugins
├── worlds/simple.world          # ground plane + a few obstacles for the lidar
├── launch/sim.launch.py         # gazebo + robot_state_publisher + spawn + monitor
└── telebot_sim/
    ├── teleop_keyboard.py       # WASD teleop, publishes Twist on /cmd_vel at 10 Hz
    └── state_monitor.py         # subscribes /scan + /odom, logs pose/speed/nearest obstacle
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

Terminal 1 (Gazebo + monitor):
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch telebot_sim sim.launch.py
```

Terminal 2 (teleop, needs its own tty):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run telebot_sim teleop_keyboard
```

Terminal 3 (RViz, optional):
```bash
rviz2
```
In RViz set Fixed Frame to `odom`, then add displays: RobotModel (topic `/robot_description`), LaserScan (topic `/scan`), Odometry (topic `/odom`).

## Sanity checks

```bash
ros2 topic list                  # expect /cmd_vel /scan /odom /robot_description
ros2 topic echo /scan --once
ros2 topic hz /odom
ros2 run tf2_tools view_frames   # generates frames.pdf of the TF tree
```

Expected TF tree: `odom -> base_link -> {left_wheel_link, right_wheel_link, caster_link, lidar_link}`

## Gotchas that will bite you (worth understanding, they come up in interviews)

- **LaserScan in the wrong frame in RViz**: the ray sensor plugin's `<frame_name>` must exactly match the URDF link name (`lidar_link`). If they differ, RViz drops or misplaces the scan.
- **`/scan` doesn't exist**: the plugin publishes on a private `~/out` topic by default. The `<remapping>~/out:=scan</remapping>` line in the xacro is what gives you `/scan`.
- **Robot stutters while driving**: the diff drive plugin has a command timeout. That's why the teleop node republishes the last command at 10 Hz instead of only on keypress.
- **No odom->base_link transform**: `robot_state_publisher` only publishes the static tree from the URDF. The odom transform comes from `<publish_odom_tf>true</publish_odom_tf>` in the diff drive plugin.
- **use_sim_time**: nodes that consume TF or timestamps must run with `use_sim_time: true` or lookups fail with extrapolation errors once Gazebo clock and wall clock drift apart.
- **Robot tips forward on spawn**: check the caster z-offset vs wheel radius; the chassis should be level. Caster friction is zeroed (`mu1/mu2 = 0`) so it slides rather than fighting the drive wheels.

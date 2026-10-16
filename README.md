# ROS2 Mobile Robot Simulation

Differential-drive robot simulated in Gazebo with ROS 2 Humble: keyboard teleop, LiDAR and odometry monitoring, RViz, and a LiDAR-based safety controller that slows and then blocks forward motion near obstacles and stops the robot if teleop goes quiet.

The ROS 2 package is in [`telebot_sim/`](telebot_sim/). Full docs, safety controller details and test scenarios: [`telebot_sim/README.md`](telebot_sim/README.md).

```
teleop_keyboard --/cmd_vel_raw--> safety_controller --/cmd_vel--> gazebo diff_drive
                                          ^
gazebo lidar ----------/scan--------------+
```

## Quick start (Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11)

```bash
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-xacro python3-pytest

mkdir -p ~/safety_ws/src && cd ~/safety_ws/src
git clone https://github.com/hademirren/ROS2-Mobile-Robot-Simulation.git
# or: unzip the GitHub "Download ZIP" here
cd ~/safety_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install && source install/setup.bash
```

Run (each terminal: `source /opt/ros/humble/setup.bash && source ~/safety_ws/install/setup.bash`):

```bash
ros2 launch telebot_sim sim.launch.py                                    # Gazebo + monitor + safety controller
ros2 run telebot_sim teleop_keyboard --ros-args -r cmd_vel:=cmd_vel_raw  # teleop, own terminal
```

Without the safety layer (teleop drives `/cmd_vel` directly):

```bash
ros2 launch telebot_sim sim.launch.py enable_safety:=false
ros2 run telebot_sim teleop_keyboard
```

Unit tests for the safety logic (no ROS needed):

```bash
cd telebot_sim && python3 -m pytest test/ -v
```

## Opening in PyCharm

Launch PyCharm from a terminal with ROS sourced so `rclpy` resolves, open the `telebot_sim/` folder (not the repo root), and pick the system interpreter `/usr/bin/python3`. Run the sim from a real terminal or PyCharm's Terminal tab, not the Run button: teleop needs a tty for raw key input.

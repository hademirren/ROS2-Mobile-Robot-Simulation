import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('telebot_sim')
    gazebo_share = get_package_share_directory('gazebo_ros')

    # xacro -> urdf at launch time so edits to the .xacro don't need a rebuild step
    xacro_path = os.path.join(pkg_share, 'urdf', 'telebot.urdf.xacro')
    robot_description = xacro.process_file(xacro_path).toxml()

    world_path = os.path.join(pkg_share, 'worlds', 'simple.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_path}.items(),
    )

    # publishes the static TF tree from the URDF; the diff drive plugin adds odom->base_link
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    # spawn from the robot_description topic rather than a file so both stay in sync
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'telebot'],
    )

    monitor = Node(
        package='telebot_sim',
        executable='state_monitor',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([gazebo, rsp, spawn, monitor])

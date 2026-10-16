#!/usr/bin/env python3
"""Safety layer between teleop and the robot.

    teleop_keyboard --/cmd_vel_raw--> safety_controller --/cmd_vel--> diff drive
                              /scan --^

Teleop must be remapped so it doesn't bypass this node:
    ros2 run telebot_sim teleop_keyboard --ros-args -r cmd_vel:=cmd_vel_raw
"""
import math
import time
from dataclasses import asdict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from telebot_sim.safety_logic import (
    BLOCKED, TIMEOUT, SafetyParams, decide, front_min_range)


class SafetyController(Node):
    def __init__(self):
        super().__init__('safety_controller')

        # every SafetyParams field is a ROS parameter, e.g. -p stop_distance:=0.7
        values = {name: self.get_float(name, default)
                  for name, default in asdict(SafetyParams()).items()}
        self.params = SafetyParams(**values)
        rate = self.get_float('publish_rate', 20.0)
        self.half_angle = math.radians(self.params.front_angle_deg)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel_raw', self.on_cmd, 10)
        # sensor QoS (best effort) works with any lidar publisher
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

        self.cmd = Twist()
        self.last_cmd_time = None
        self.front_range = None
        self.last_scan_time = None
        self.state = None
        self.warned_bypass = False

        # timer, not per message, so we still stop when teleop goes silent
        self.create_timer(1.0 / rate, self.step)
        self.create_timer(2.0, self.check_bypass)

        p = self.params
        self.get_logger().info(
            f'stop={p.stop_distance:.2f} m  slow={p.slow_distance:.2f} m  '
            f'sector=+/-{p.front_angle_deg:.0f} deg  timeout={p.cmd_timeout:.2f} s')

    def get_float(self, name, default):
        # dynamic typing so "-p stop_distance:=1" works as well as 1.0
        desc = ParameterDescriptor(dynamic_typing=True)
        return float(self.declare_parameter(name, default, desc).value)

    def on_cmd(self, msg):
        self.cmd = msg
        self.last_cmd_time = time.monotonic()

    def on_scan(self, msg):
        self.front_range = front_min_range(
            msg.ranges, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max, self.half_angle)
        self.last_scan_time = time.monotonic()

    def step(self):
        # wall clock, not sim time: Gazebo's /clock is too coarse for a 0.3 s watchdog
        now = time.monotonic()
        cmd_age = None if self.last_cmd_time is None else now - self.last_cmd_time
        scan_age = None if self.last_scan_time is None else now - self.last_scan_time

        d = decide(self.cmd.linear.x, self.cmd.angular.z, self.front_range,
                   cmd_age, scan_age, self.state, self.params)

        # log state changes only, not every cycle
        if d.state != self.state:
            # two separate calls: rclpy errors if one log line changes severity
            if d.state in (BLOCKED, TIMEOUT):
                self.get_logger().warn(f'{d.state}: {d.reason}')
            else:
                self.get_logger().info(f'{d.state}: {d.reason}')
            self.state = d.state

        out = Twist()
        out.linear.x = d.linear
        out.angular.z = d.angular
        self.pub.publish(out)

    def check_bypass(self):
        # a teleop that wasn't remapped also publishes cmd_vel and overrides us
        others = self.count_publishers(self.pub.topic_name) - 1
        if others > 0 and not self.warned_bypass:
            self.get_logger().warn(
                f'{others} other publisher(s) on {self.pub.topic_name} bypass the '
                'safety layer. Run teleop with: '
                'ros2 run telebot_sim teleop_keyboard --ros-args -r cmd_vel:=cmd_vel_raw')
        self.warned_bypass = others > 0


def main(args=None):
    rclpy.init(args=args)
    node = SafetyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # Ctrl-C may have shut down already
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Safety layer between teleop and the diff drive plugin.

    teleop_keyboard --/cmd_vel_raw--> safety_controller --/cmd_vel--> diff drive
                              /scan --^

Blocks or slows forward motion when something is close in front, and holds
zero velocity if teleop goes quiet. Decision logic is in safety_logic.py.

Teleop must be remapped so it doesn't bypass this node:
    ros2 run telebot_sim teleop_keyboard --ros-args -r cmd_vel:=cmd_vel_raw
"""
import math
import time

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
        defaults = SafetyParams()
        self.params = SafetyParams(
            stop_distance=self._param('stop_distance', defaults.stop_distance),
            slow_distance=self._param('slow_distance', defaults.slow_distance),
            front_angle_deg=self._param('front_angle_deg', defaults.front_angle_deg),
            cmd_timeout=self._param('cmd_timeout', defaults.cmd_timeout),
            scan_timeout=self._param('scan_timeout', defaults.scan_timeout),
            creep_speed=self._param('creep_speed', defaults.creep_speed),
            hysteresis=self._param('hysteresis', defaults.hysteresis),
        )
        rate = self._param('publish_rate', 20.0)
        self.half_angle = math.radians(self.params.front_angle_deg)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel_raw', self.on_cmd, 10)
        # best effort matches both reliable and best effort lidar publishers
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

        self.cmd = Twist()
        self.last_cmd_time = None
        self.front_range = None
        self.last_scan_time = None
        self.state = None
        self.warned_bypass = False

        # publish on a timer, not per message, so the watchdog still fires
        # when teleop stops sending
        self.create_timer(1.0 / rate, self.step)
        self.create_timer(2.0, self.check_bypass)

        p = self.params
        self.get_logger().info(
            f'{self.cmd_sub.topic_name} -> {self.pub.topic_name}  '
            f'stop={p.stop_distance:.2f} m slow={p.slow_distance:.2f} m '
            f'sector=+/-{p.front_angle_deg:.0f} deg timeout={p.cmd_timeout:.2f} s')

    def _param(self, name, default):
        # dynamic typing so "-p stop_distance:=1" works as well as 1.0
        desc = ParameterDescriptor(dynamic_typing=True)
        return float(self.declare_parameter(name, default, desc).value)

    # Ages use a monotonic wall clock, not sim time: teleop publishes on wall
    # time, and /clock only ticks at 10 Hz, which made the 0.3 s watchdog trip
    # falsely while teleop was still sending.
    def _age(self, stamp, now):
        return None if stamp is None else now - stamp

    def on_cmd(self, msg: Twist):
        self.cmd = msg
        self.last_cmd_time = time.monotonic()

    def on_scan(self, msg: LaserScan):
        self.front_range = front_min_range(
            msg.ranges, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max, self.half_angle)
        self.last_scan_time = time.monotonic()

    def step(self):
        now = time.monotonic()
        d = decide(self.cmd.linear.x, self.cmd.angular.z, self.front_range,
                   self._age(self.last_cmd_time, now),
                   self._age(self.last_scan_time, now),
                   self.state, self.params)

        # log transitions only; at 20 Hz per-cycle logs would bury everything
        if d.state != self.state:
            # separate call sites: rclpy refuses one call site at two severities
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
        # an unremapped teleop also publishes cmd_vel and fights this node
        others = self.count_publishers(self.pub.topic_name) - 1
        if others > 0 and not self.warned_bypass:
            self.get_logger().warn(
                f'{others} other publisher(s) on {self.pub.topic_name} are bypassing '
                'the safety layer. Run teleop with: '
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
        # Ctrl-C may already have shut the context down
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

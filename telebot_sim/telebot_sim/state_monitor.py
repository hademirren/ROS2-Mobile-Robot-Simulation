#!/usr/bin/env python3
"""Subscribes to /scan and /odom, logs robot state at 1 Hz and warns
when an obstacle enters the safety radius. This is the 'monitoring'
half of the teleop setup: drive in one terminal, watch state here.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

SAFETY_RADIUS = 0.5  # m


class StateMonitor(Node):
    def __init__(self):
        super().__init__('state_monitor')
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.min_range = None
        self.min_angle = None
        self.pose = None
        self.speed = 0.0
        # log on a timer instead of per-message; /scan at 10 Hz would flood the console
        self.create_timer(1.0, self.report)

    def on_scan(self, msg: LaserScan):
        # inf/0.0 readings mean "nothing in range", not "something at 0 m"
        best_r, best_i = None, None
        for i, r in enumerate(msg.ranges):
            if msg.range_min <= r <= msg.range_max:
                if best_r is None or r < best_r:
                    best_r, best_i = r, i
        self.min_range = best_r
        if best_i is not None:
            self.min_angle = msg.angle_min + best_i * msg.angle_increment

    def on_odom(self, msg: Odometry):
        self.pose = msg.pose.pose
        self.speed = msg.twist.twist.linear.x

    def report(self):
        if self.pose is None:
            self.get_logger().info('waiting for /odom...')
            return

        # yaw from quaternion; planar robot so roll/pitch are ignorable
        q = self.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))

        line = (f'pose: x={self.pose.position.x:+.2f} '
                f'y={self.pose.position.y:+.2f} '
                f'yaw={math.degrees(yaw):+.1f}deg  '
                f'speed={self.speed:+.2f} m/s')

        if self.min_range is not None:
            line += f'  nearest obstacle: {self.min_range:.2f} m @ {math.degrees(self.min_angle):+.0f}deg'
            if self.min_range < SAFETY_RADIUS:
                self.get_logger().warn(line + '  << INSIDE SAFETY RADIUS')
                return

        self.get_logger().info(line)


def main(args=None):
    rclpy.init(args=args)
    node = StateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Keyboard teleop: WASD to drive, space to stop, q to quit.

Run in its own terminal (needs a real tty for raw key reads):
    ros2 run telebot_sim teleop_keyboard
"""
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LIN_STEP = 0.1   # m/s per keypress
ANG_STEP = 0.2   # rad/s per keypress
LIN_MAX = 0.8
ANG_MAX = 1.5

HELP = """
Controls:
  w/s : forward / backward
  a/d : turn left / right
  space : stop
  q : quit
"""


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lin = 0.0
        self.ang = 0.0
        # republish at 10 Hz even without keypresses; the diff drive plugin
        # stops the robot if cmd_vel goes quiet, which reads as stuttering
        self.timer = self.create_timer(0.1, self.publish_cmd)

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.lin
        msg.angular.z = self.ang
        self.pub.publish(msg)

    def handle_key(self, key):
        if key == 'w':
            self.lin = min(self.lin + LIN_STEP, LIN_MAX)
        elif key == 's':
            self.lin = max(self.lin - LIN_STEP, -LIN_MAX)
        elif key == 'a':
            self.ang = min(self.ang + ANG_STEP, ANG_MAX)
        elif key == 'd':
            self.ang = max(self.ang - ANG_STEP, -ANG_MAX)
        elif key == ' ':
            self.lin = 0.0
            self.ang = 0.0
        self.get_logger().info(f'lin={self.lin:+.2f} m/s  ang={self.ang:+.2f} rad/s')


def get_key(settings, timeout=0.1):
    # raw mode so single keys arrive without Enter
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if ready else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = TeleopKeyboard()
    print(HELP)
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == 'q' or key == '\x03':  # q or Ctrl-C
                break
            if key:
                node.handle_key(key)
            # spin_once keeps the 10 Hz publish timer firing between key reads
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        # send a zero command on exit so the robot doesn't coast forever
        node.lin = 0.0
        node.ang = 0.0
        node.publish_cmd()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

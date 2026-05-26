#!/usr/bin/env python3
"""
SEN771 Robot Monitor
Watches odometry + cmd_vel every 2 seconds.
Flags when the robot is commanded to move but position/velocity shows it isn't.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class MonitorNode(Node):

    def __init__(self):
        super().__init__('sen771_monitor')

        self._px, self._py = 0.0, 0.0       # current position
        self._vx           = 0.0             # actual forward velocity (from odom)
        self._cmd_vx       = 0.0             # commanded forward velocity
        self._prev_px, self._prev_py = None, None  # position 2s ago

        self.create_subscription(Odometry, '/sen771_robot/odometry', self._odom_cb, 10)
        self.create_subscription(Twist,    '/sen771_robot/cmd_vel',   self._cmd_cb,  10)
        self.create_timer(2.0, self._report)

        self.get_logger().info('Monitor started — checking every 2 s')

    def _odom_cb(self, msg):
        self._px = msg.pose.pose.position.x
        self._py = msg.pose.pose.position.y
        self._vx = msg.twist.twist.linear.x

    def _cmd_cb(self, msg):
        self._cmd_vx = msg.linear.x

    def _report(self):
        if self._prev_px is None:
            self._prev_px, self._prev_py = self._px, self._py
            return

        moved = math.hypot(self._px - self._prev_px, self._py - self._prev_py)
        self._prev_px, self._prev_py = self._px, self._py

        # classify — only flag when controller is commanding real forward speed (> 1.5)
        # Low cmd (< 1.5) means the robot is decelerating near a waypoint — expected
        if self._cmd_vx < 0.3:
            label = '-- idle/turning'
        elif self._cmd_vx < 1.5:
            label = '~  decelerating (near waypoint)'
        elif moved < 0.10:
            label = '!! WALL/STUCK  — full speed commanded, zero movement'
        elif moved < 0.50:
            label = '!! SLOW        — full speed commanded, likely wall contact'
        elif moved < 1.0:
            label = '~  below speed  — obstacle repulsion active'
        else:
            label = 'OK'

        self.get_logger().info(
            f'pos=({self._px:6.1f},{self._py:6.1f})  '
            f'moved={moved:5.2f}m/2s  '
            f'cmd={self._cmd_vx:4.1f}  '
            f'vel={self._vx:4.2f}  '
            f'{label}')


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

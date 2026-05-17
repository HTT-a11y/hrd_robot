#!/usr/bin/env python3
"""
Print current left_tcp_link / right_tcp_link pose in base_link frame.
Refresh with Enter, or run with --stream for continuous output.

Usage:
  ros2 run hrd_moveit_config print_tcp_pose.py
  ros2 run hrd_moveit_config print_tcp_pose.py --stream   # 1 Hz continuous
"""

import sys

import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TcpPosePrinter(Node):

    def __init__(self, continuous=False):
        super().__init__("print_tcp_pose")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        if continuous:
            self._timer = self.create_timer(1.0, self._print_poses)
        else:
            self.get_logger().info(
                "Press Enter to print current TCP poses "
                "(left_tcp_link / right_tcp_link in base_link). "
                "Ctrl-C to exit."
            )
            self._timer = self.create_timer(0.1, self._check_stdin)

    def _check_stdin(self):
        """Non-blocking stdin poll — print on Enter."""
        import select
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            self._print_poses()

    def _print_poses(self):
        for link in ("left_tcp_link", "right_tcp_link"):
            try:
                t = self._tf_buffer.lookup_transform(
                    "base_link", link, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
                )
                p = t.transform.translation
                q = t.transform.rotation
                self.get_logger().info(
                    f"\n  {link}:\n"
                    f"    xyz=({p.x:.4f}, {p.y:.4f}, {p.z:.4f})\n"
                    f"    xyzw=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})"
                )
            except Exception:
                self.get_logger().info(f"\n  {link}: <no tf available>")


def main():
    rclpy.init()
    continuous = "--stream" in sys.argv
    node = TcpPosePrinter(continuous=continuous)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

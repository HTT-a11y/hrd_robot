#!/usr/bin/env python3
"""
Publish a red cylinder collision object for MoveIt / RViz.
The same cylinder is spawned in Gazebo by the launch file.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


class TargetObjectPublisher(Node):

    def __init__(self):
        super().__init__("spawn_target_object")

        # wait until move_group is up so the object isn't dropped
        self._timer = self.create_timer(5.0, self._publish)

    def _publish(self):
        self._timer.cancel()

        pub = self.create_publisher(CollisionObject, "/collision_object", 10)

        obj = CollisionObject()
        obj.id = "target_cylinder"
        obj.header.frame_id = "base_link"
        obj.operation = CollisionObject.ADD

        # cylinder: height 0.1 m, radius 0.02 m
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [0.25, 0.02]

        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = -0.35
        pose.position.z = 0.95
        pose.orientation.w = 1.0

        obj.primitives = [cylinder]
        obj.primitive_poses = [pose]

        pub.publish(obj)
        self.get_logger().info("Published target cylinder to /collision_object")


def main():
    rclpy.init()
    node = TargetObjectPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

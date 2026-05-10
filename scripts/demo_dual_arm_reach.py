#!/usr/bin/env python3
"""
Dual-arm demo: Both end effectors reach toward the same target point (a cylinder).

Uses position-constrained planning via /move_action — the same path RViz uses
when you drag the interactive marker and hit "Plan & Execute".

Usage:
  ros2 run hrd_moveit_config demo_dual_arm_reach.py
"""

import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
    PositionConstraint,
    WorkspaceParameters,
)
from shape_msgs.msg import SolidPrimitive


ARM_JOINTS = {
    "left_arm": [
        "left_link_1_joint", "left_link_2_joint", "left_link_3_joint",
        "left_link_4_joint", "left_link_5_joint", "left_link_6_joint",
        "left_link_7_joint",
    ],
    "right_arm": [
        "right_link_1_joint", "right_link_2_joint", "right_link_3_joint",
        "right_link_4_joint", "right_link_5_joint", "right_link_6_joint",
        "right_link_7_joint",
    ],
}

HOME = {
    "left_link_1_joint": -0.0172,  "left_link_2_joint": -0.0627,
    "left_link_3_joint": 0.0,      "left_link_4_joint": 0.0,
    "left_link_5_joint": 0.0,      "left_link_6_joint": 0.0,
    "left_link_7_joint": -1.8389,
    "right_link_1_joint": 0.0858,  "right_link_2_joint": -0.0877,
    "right_link_3_joint": -0.2231, "right_link_4_joint": -0.1121,
    "right_link_5_joint": 0.0515,  "right_link_6_joint": -0.1111,
    "right_link_7_joint": 1.6307,
}


class DualArmReachDemo(Node):

    def __init__(self):
        super().__init__("demo_dual_arm_reach")
        self._action = ActionClient(self, MoveGroup, "/move_action")
        if not self._action.wait_for_server(timeout_sec=15.0):
            self.get_logger().fatal("move_group not ready")
            sys.exit(1)
        self.get_logger().info("Connected to /move_action")

    # ── helpers ──────────────────────────────────────────────────────────

    @staticmethod
    def _pos_constraint(link_name, x, y, z, tol=0.03):
        """End-effector <link_name> must be within a <tol>-radius sphere at (x,y,z)."""
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[tol])
        sphere_pose = Pose(position=Point(x=x, y=y, z=z))
        sphere_pose.orientation.w = 1.0
        bv = BoundingVolume(primitives=[sphere], primitive_poses=[sphere_pose])
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = link_name
        pc.constraint_region = bv
        pc.weight = 1.0
        return pc

    # ── position-constrained planning (same path as RViz) ───────────────

    def plan_position_goal(self, group, pos_constraints, vel=0.5, acc=0.5):
        req = MotionPlanRequest()
        req.group_name = group
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = acc
        req.num_planning_attempts = 20
        req.allowed_planning_time = 10.0

        goal = Constraints(position_constraints=pos_constraints)
        req.goal_constraints.append(goal)

        ws = WorkspaceParameters()
        ws.header.frame_id = "base_link"
        ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -2.0
        ws.max_corner.x = ws.max_corner.y = ws.max_corner.z = 2.0
        req.workspace_parameters = ws

        return self._send_and_wait(req)

    # ── joint-space planning ────────────────────────────────────────────

    def plan_joint_goal(self, group, joint_values, vel=0.5, acc=0.5):
        joints = ARM_JOINTS[group]
        req = MotionPlanRequest()
        req.group_name = group
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = acc
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0

        goal = Constraints()
        for jn, jv in zip(joints, joint_values):
            c = JointConstraint(joint_name=jn, position=jv,
                                tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            goal.joint_constraints.append(c)
        req.goal_constraints.append(goal)

        ws = WorkspaceParameters()
        ws.header.frame_id = "base_link"
        ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -2.0
        ws.max_corner.x = ws.max_corner.y = ws.max_corner.z = 2.0
        req.workspace_parameters = ws

        return self._send_and_wait(req)

    # ── send + wait ─────────────────────────────────────────────────────

    def _send_and_wait(self, request):
        goal = MoveGroup.Goal(request=request, planning_options=PlanningOptions())

        send_fut = self._action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=10.0)
        if not send_fut.done():
            self.get_logger().error("Goal send timed out")
            return False

        gh = send_fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error("Goal rejected")
            return False

        self.get_logger().info(f"Planning '{request.group_name}' …")
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=30.0)
        if not res_fut.done():
            self.get_logger().error("Planning timed out")
            return False

        result = res_fut.result().result
        traj = getattr(result, "planned_trajectory", None)
        if traj and len(traj.joint_trajectory.points) > 0:
            pts = len(traj.joint_trajectory.points)
            dur = traj.joint_trajectory.points[-1].time_from_start
            self.get_logger().info(
                f"OK  {request.group_name} — {pts} pts, "
                f"{dur.sec + dur.nanosec * 1e-9:.2f}s"
            )
            return True

        ec = getattr(result, "error_code", None)
        self.get_logger().error(f"FAIL  {request.group_name} — error {ec.val if ec else '?'}")
        return False


# ═══════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    demo = DualArmReachDemo()

    # Cylinder is at (0, -0.65, 0.75).
    # Each arm approaches from its own side to avoid colliding with the cylinder.
    LEFT_TARGET  = (0.0, -0.58, 0.75)   #  7 cm +Y from cylinder (left arm approaches from left)
    RIGHT_TARGET = (-0.1, -0.72, 0.75)  # right arm approaches from right-front

    # ── Step 1 — Home ────────────────────────────────────────────────────
    demo.get_logger().info("=" * 60)
    demo.get_logger().info("Step 1/3: HOME positions")
    demo.get_logger().info("=" * 60)
    demo.plan_joint_goal("left_arm",  [HOME[j] for j in ARM_JOINTS["left_arm"]])
    time.sleep(0.5)
    demo.plan_joint_goal("right_arm", [HOME[j] for j in ARM_JOINTS["right_arm"]])
    time.sleep(2.0)

    # ── Step 2 — Left arm reach ──────────────────────────────────────────
    demo.get_logger().info("=" * 60)
    demo.get_logger().info(f"Step 2/3: Left arm -> {LEFT_TARGET}")
    demo.get_logger().info("=" * 60)
    left_ok = demo.plan_position_goal(
        "left_arm", [demo._pos_constraint("left_link_7", *LEFT_TARGET)]
    )
    time.sleep(1.0)

    # ── Step 3 — Right arm reach ─────────────────────────────────────────
    demo.get_logger().info("=" * 60)
    demo.get_logger().info(f"Step 3/3: Right arm -> {RIGHT_TARGET}")
    demo.get_logger().info("=" * 60)
    right_ok = demo.plan_position_goal(
        "right_arm", [demo._pos_constraint("right_link_7", *RIGHT_TARGET)]
    )

    # ── Summary ──────────────────────────────────────────────────────────
    demo.get_logger().info("=" * 60)
    if left_ok and right_ok:
        demo.get_logger().info("SUCCESS — both arms reached the target")
    elif left_ok:
        demo.get_logger().warn("Left OK, right FAILED")
    elif right_ok:
        demo.get_logger().warn("Right OK, left FAILED")
    else:
        demo.get_logger().error("Both FAILED")
    demo.get_logger().info("=" * 60)

    rclpy.shutdown()
    return 0 if (left_ok and right_ok) else 1


if __name__ == "__main__":
    sys.exit(main())

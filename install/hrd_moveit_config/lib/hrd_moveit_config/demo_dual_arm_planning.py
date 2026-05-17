#!/usr/bin/env python3
"""
Dual-arm path planning demo for HRD robot.
Launched after: ros2 launch hrd_moveit_config demo.launch.py

Uses MoveIt's /move_action [moveit_msgs/action/MoveGroup].
"""

import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
    WorkspaceParameters,
)


# ── Planning groups and their joints ──────────────────────────────────────────
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

HOME_POSITIONS = {
    "left_link_1_joint": -0.0172,  "left_link_2_joint": -0.0627,
    "left_link_3_joint": 0.0,      "left_link_4_joint": 0.0,
    "left_link_5_joint": 0.0,      "left_link_6_joint": 0.0,
    "left_link_7_joint": -1.8389,
    "right_link_1_joint": 0.0858,  "right_link_2_joint": -0.0877,
    "right_link_3_joint": -0.2231, "right_link_4_joint": -0.1121,
    "right_link_5_joint": 0.0515,  "right_link_6_joint": -0.1111,
    "right_link_7_joint": 1.6307,
}


def _list_actions():
    """Get available action servers using ros2 CLI."""
    try:
        out = subprocess.check_output(
            ["ros2", "action", "list", "-t"],
            stderr=subprocess.DEVNULL, timeout=5, text=True,
        )
        return out.strip().splitlines()
    except Exception:
        return []


class MoveGroupClient(Node):
    """Planning client via MoveGroup action — synchronous style with spin."""

    def __init__(self):
        super().__init__("demo_dual_arm_planning")

        lines = _list_actions()
        self.get_logger().info("Action servers found:")
        found = None
        for line in lines:
            self.get_logger().info(f"  {line}")
            if "/move_action" in line:
                found = "/move_action"
        if not found:
            self.get_logger().error(
                "/move_action not found. "
                "Is move_group fully started? (look for 'You can start planning now!')"
            )
            sys.exit(1)

        self._action = ActionClient(self, MoveGroup, found)
        self.get_logger().info("Waiting for /move_action to become ready...")
        if not self._action.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/move_action not ready.")
            sys.exit(1)
        self.get_logger().info("Connected. Ready to plan.")

    # ── helpers ──────────────────────────────────────────────────────────────

    @staticmethod
    def _make_request(group_name, joint_names, joint_values,
                      velocity_scaling=0.1, acceleration_scaling=0.1):
        req = MotionPlanRequest()
        req.group_name = group_name
        req.max_velocity_scaling_factor = velocity_scaling
        req.max_acceleration_scaling_factor = acceleration_scaling
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0

        goal = Constraints()
        for jn, jv in zip(joint_names, joint_values):
            c = JointConstraint()
            c.joint_name = jn
            c.position = jv
            c.tolerance_above = 0.001
            c.tolerance_below = 0.001
            c.weight = 1.0
            goal.joint_constraints.append(c)
        req.goal_constraints.append(goal)

        ws = WorkspaceParameters()
        ws.header.frame_id = "base_link"
        ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -2.0
        ws.max_corner.x = ws.max_corner.y = ws.max_corner.z = 2.0
        req.workspace_parameters = ws
        return req

    def plan(self, group, joint_values,
             velocity_scaling=0.1, acceleration_scaling=0.1):
        """Plan a single arm group. Returns True on success."""
        req = self._make_request(
            group, ARM_JOINTS[group], joint_values,
            velocity_scaling, acceleration_scaling,
        )
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()

        # Step 1: send goal
        send_future = self._action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            self.get_logger().error("Goal send timed out!")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by move_group.")
            return False

        self.get_logger().info("Goal accepted, planning...")

        # Step 2: wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        if not result_future.done():
            self.get_logger().error("Planning timed out!")
            return False

        result = result_future.result().result
        traj = getattr(result, 'planned_trajectory', None)
        if traj and len(traj.joint_trajectory.points) > 0:
            pts = len(traj.joint_trajectory.points)
            dur = traj.joint_trajectory.points[-1].time_from_start
            self.get_logger().info(
                f"Planning SUCCESS — {pts} pts, "
                f"duration={dur.sec}.{dur.nanosec:09d}s"
            )
            return True
        else:
            ec = getattr(result, 'error_code', None)
            code = ec.val if ec else -1
            self.get_logger().error(f"Planning FAILED — error code: {code}")
            return False


# ── Demo routines ────────────────────────────────────────────────────────────

def demo_home_position(client):
    client.get_logger().info("=== [1/3] Left arm to HOME ===")
    ok = client.plan("left_arm",
                     [HOME_POSITIONS[j] for j in ARM_JOINTS["left_arm"]])
    time.sleep(0.5)
    client.get_logger().info("=== [1/3] Right arm to HOME ===")
    ok &= client.plan("right_arm",
                      [HOME_POSITIONS[j] for j in ARM_JOINTS["right_arm"]])
    return ok

def demo_left_arm_reach(client):
    client.get_logger().info("=== [2/3] Left arm forward reach ===")
    return client.plan("left_arm", [0.3, -0.5, 0.0, -1.2, 0.0, 0.8, 0.5])

def demo_right_arm_reach(client):
    client.get_logger().info("=== [3/3] Right arm side reach ===")
    return client.plan("right_arm", [-0.5, -0.4, 0.2, -0.9, -0.2, 0.6, 1.0])


def main():
    rclpy.init()
    client = MoveGroupClient()

    ok = True
    if not demo_home_position(client):
        ok = False
    time.sleep(2.0)
    if not demo_left_arm_reach(client):
        ok = False
    time.sleep(2.0)
    if not demo_right_arm_reach(client):
        ok = False

    if ok:
        client.get_logger().info("All planning demos completed successfully!")
    else:
        client.get_logger().warn("Some demos failed — check logs above.")

    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
Dual-arm demo: Both end effectors move to pre-grasp poses near a cylinder,
with the palm oriented to face the cylinder for grasping.

Strategy:
  1.  Call /compute_ik with many orientations to check reachability.
  2.  Use the best IK solution's orientation as a guide for the planning
      constraint (loose tolerance so KDL has room).
  3.  Detailed logging so every failure has a clear cause.

Usage:
  ros2 run hrd_moveit_config demo_dual_arm_reach.py
"""

import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    WorkspaceParameters,
)
from moveit_msgs.srv import GetPositionIK
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

# Orientations to try in /compute_ik diagnostic (xyzw quaternions)
ORIENTATIONS = [
    (0.0, 0.0, 0.0, 1.0),           # identity
    (0.0, 0.707, 0.0, 0.707),       # +90° Y
    (0.0, -0.707, 0.0, 0.707),      # -90° Y
    (0.707, 0.0, 0.0, 0.707),       # +90° X
    (-0.707, 0.0, 0.0, 0.707),      # -90° X
    (0.0, 0.0, 0.707, 0.707),       # +90° Z
    (0.0, 0.0, -0.707, 0.707),      # -90° Z
    (0.5, 0.5, 0.5, 0.5),           # 120° around (1,1,1)
    (0.0, 1.0, 0.0, 0.0),           # 180° Y
    (1.0, 0.0, 0.0, 0.0),           # 180° X
]


def _approach_quat(dx, dy, dz):
    """Quaternion (xyzw) mapping local +Z → world (dx, dy, dz)."""
    m = math.hypot(dx, dy, dz)
    if m < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    vx, vy, vz = dx / m, dy / m, dz / m
    c = vz
    if c < -0.9999:
        return (1.0, 0.0, 0.0, 0.0)
    ax, ay = -vy, vx
    an = math.hypot(ax, ay)
    if an < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sqrt((1.0 - c) / 2.0)
    return (ax / an * s, ay / an * s, 0.0, math.sqrt((1.0 + c) / 2.0))


def _ik_error_name(code):
    names = {v: k for k, v in vars(MoveItErrorCodes).items() if isinstance(v, int) and not k.startswith("_")}
    return names.get(code, f"UNKNOWN({code})")


class DualArmReachDemo(Node):

    def __init__(self):
        super().__init__("demo_dual_arm_reach")
        self._action = ActionClient(self, MoveGroup, "/move_action")
        if not self._action.wait_for_server(timeout_sec=15.0):
            self.get_logger().fatal("move_group not ready")
            sys.exit(1)
        self.get_logger().info("Connected to /move_action")

        self._ik = self.create_client(GetPositionIK, "/compute_ik")
        if not self._ik.wait_for_service(timeout_sec=5.0):
            self.get_logger().fatal("/compute_ik unavailable")
            sys.exit(1)
        self.get_logger().info("Connected to /compute_ik")

    # ── diagnostic: /compute_ik with multiple orientations ────────────────

    def diagnose_ik(self, group, link_name, x, y, z, avoid_collisions=True):
        """Try /compute_ik with a set of orientations; log results."""
        self.get_logger().info(
            f"--- IK diagnosis for {group}: link={link_name} target=({x:.2f}, {y:.2f}, {z:.2f}) "
            f"collisions={'ON' if avoid_collisions else 'OFF'} ---"
        )
        for i, (qx, qy, qz, qw) in enumerate(ORIENTATIONS):
            req = GetPositionIK.Request()
            req.ik_request.group_name = group
            req.ik_request.avoid_collisions = avoid_collisions
            req.ik_request.timeout.sec = 5
            req.ik_request.ik_link_name = link_name

            ps = PoseStamped()
            ps.header.frame_id = "base_link"
            ps.pose = Pose(position=Point(x=x, y=y, z=z),
                           orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))

            req.ik_request.pose_stamped = ps

            fut = self._ik.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            if not fut.done():
                self.get_logger().warn(f"  orient #{i}: IK service timeout")
                continue
            resp = fut.result()
            if resp.error_code.val == MoveItErrorCodes.SUCCESS:
                names = ARM_JOINTS[group]
                jn = resp.solution.joint_state.name
                jv = resp.solution.joint_state.position
                lookup = dict(zip(jn, jv))
                vals = [lookup.get(n, 0.0) for n in names]
                self.get_logger().info(
                    f"  orient #{i}: SUCCESS  joints=" +
                    " ".join(f"{v:+.2f}" for v in vals)
                )
                return True  # at least one solution exists
            else:
                self.get_logger().info(
                    f"  orient #{i}: {_ik_error_name(resp.error_code.val)}"
                )
        self.get_logger().error(
            f"  ALL {len(ORIENTATIONS)} orientations FAILED — "
            f"target ({x:.2f}, {y:.2f}, {z:.2f}) is likely OUT OF REACH"
        )
        return False

    # ── constraint builders ───────────────────────────────────────────────

    @staticmethod
    def _pos_constraint(link_name, x, y, z, tol=0.05):
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

    @staticmethod
    def _orient_constraint(link_name, qx, qy, qz, qw, z_tol=0.5):
        """Loose orientation: +Z within z_tol rad of desired; X/Y free."""
        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = link_name
        oc.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        oc.absolute_x_axis_tolerance = 3.14
        oc.absolute_y_axis_tolerance = 3.14
        oc.absolute_z_axis_tolerance = z_tol
        oc.weight = 0.5
        return oc

    # ── planning ──────────────────────────────────────────────────────────

    def plan_pose_goal(self, group, pos_constraints, orient_constraints=None,
                       vel=0.5, acc=0.5):
        req = MotionPlanRequest()
        req.group_name = group
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = acc
        req.num_planning_attempts = 20
        req.allowed_planning_time = 10.0

        goal = Constraints(position_constraints=pos_constraints)
        if orient_constraints:
            goal.orientation_constraints = orient_constraints
        req.goal_constraints.append(goal)

        ws = WorkspaceParameters()
        ws.header.frame_id = "base_link"
        ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -2.0
        ws.max_corner.x = ws.max_corner.y = ws.max_corner.z = 2.0
        req.workspace_parameters = ws

        return self._send_and_wait(req)

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

    # ── send + wait ───────────────────────────────────────────────────────

    def _send_and_wait(self, request):
        goal = MoveGroup.Goal(request=request, planning_options=PlanningOptions())
        send_fut = self._action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=10.0)
        if not send_fut.done():
            self.get_logger().error("Goal send timed out")
            return False
        gh = send_fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error("Goal rejected by move_group")
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
        self.get_logger().error(
            f"FAIL  {request.group_name} — {_ik_error_name(ec.val if ec else -1)}"
        )
        return False


# ═══════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    demo = DualArmReachDemo()

    CYL = (0.0, -0.65, 0.75)

    LEFT_TCP  = (0.3, -0.58, 0.82)   # front-left, upper half
    # Right arm workspace probe: try wrist IK to narrow down reachable Y range
    RIGHT_TCP = None  # will be determined by probing below

    # ── Step 1 — Home ────────────────────────────────────────────────────
    demo.get_logger().info("=" * 60)
    demo.get_logger().info("Step 1/3: HOME positions")
    demo.get_logger().info("=" * 60)
    demo.plan_joint_goal("left_arm",  [HOME[j] for j in ARM_JOINTS["left_arm"]])
    time.sleep(0.5)
    demo.plan_joint_goal("right_arm", [HOME[j] for j in ARM_JOINTS["right_arm"]])
    time.sleep(2.0)

    # ── Step 2 — Left arm ────────────────────────────────────────────────
    demo.get_logger().info("=" * 60)
    demo.get_logger().info(f"Step 2/3: Left  arm -> {LEFT_TCP}")
    demo.get_logger().info("=" * 60)

    # Diagnostic: can IK even find a solution?
    left_reachable = demo.diagnose_ik("left_arm", "left_tcp_link", *LEFT_TCP)

    l_dx = CYL[0] - LEFT_TCP[0]
    l_dy = CYL[1] - LEFT_TCP[1]
    l_dz = CYL[2] - LEFT_TCP[2]
    left_orient = _approach_quat(l_dx, l_dy, l_dz)

    left_ok = demo.plan_pose_goal(
        "left_arm",
        [demo._pos_constraint("left_tcp_link", *LEFT_TCP, tol=0.05)],
        [demo._orient_constraint("left_tcp_link", *left_orient)],
    )
    time.sleep(1.0)

    # ── Step 3 — Right arm: test seed hypothesis ──────────────────────────
    demo.get_logger().info("=" * 60)
    demo.get_logger().info("Step 3/3: Right arm — move to neutral seed, then IK")
    demo.get_logger().info("=" * 60)

    # Move right arm to a "neutral" configuration first (like left arm home but mirrored)
    # This gives KDL a better seed than the default right-arm home pose
    NEUTRAL_RIGHT = [0.0, -0.06, 0.0, 0.0, 0.0, 0.0, -1.57]
    demo.get_logger().info("Moving right arm to neutral seed...")
    demo.plan_joint_goal("right_arm", NEUTRAL_RIGHT)
    time.sleep(2.0)

    # Now try IK from this neutral seed
    RIGHT_TCP = (0.3, -0.58, 0.82)  # same target as left arm
    demo.get_logger().info(f"Testing IK from neutral seed -> {RIGHT_TCP}")
    right_reachable = demo.diagnose_ik("right_arm", "right_tcp_link", *RIGHT_TCP)

    r_dx = CYL[0] - RIGHT_TCP[0]
    r_dy = CYL[1] - RIGHT_TCP[1]
    r_dz = CYL[2] - RIGHT_TCP[2]
    right_orient = _approach_quat(r_dx, r_dy, r_dz)

    right_ok = demo.plan_pose_goal(
        "right_arm",
        [demo._pos_constraint("right_tcp_link", *RIGHT_TCP, tol=0.05)],
        [demo._orient_constraint("right_tcp_link", *right_orient)],
    )

    r_dx = CYL[0] - right_tcp[0]
    r_dy = CYL[1] - right_tcp[1]
    r_dz = CYL[2] - right_tcp[2]
    right_orient = _approach_quat(r_dx, r_dy, r_dz)

    right_ok = demo.plan_pose_goal(
        "right_arm",
        [demo._pos_constraint("right_tcp_link", *right_tcp, tol=0.05)],
        [demo._orient_constraint("right_tcp_link", *right_orient)],
    )

    # ── Summary ──────────────────────────────────────────────────────────
    demo.get_logger().info("=" * 60)
    if left_ok and right_ok:
        demo.get_logger().info("SUCCESS — both arms in pre-grasp poses")
    else:
        demo.get_logger().error(
            f"Left: {'OK' if left_ok else 'FAIL'}  "
            f"Right: {'OK' if right_ok else 'FAIL'}"
        )
    demo.get_logger().info("=" * 60)

    rclpy.shutdown()
    return 0 if (left_ok and right_ok) else 1


if __name__ == "__main__":
    sys.exit(main())

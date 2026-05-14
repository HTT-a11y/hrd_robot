/**
 * Dual-arm demo (C++): reach pre-grasp with palm facing cylinder.
 *
 * Constraints per arm:  position (3 DOF) + palm Y→cylinder (1 DOF) = 4 DOF
 * Leaves 3 DOF slack for a 7-DOF arm → IK stays tractable.
 *
 * Uses raw MoveGroup action client for full constraint control.
 *
 * Build: colcon build --packages-select hrd_moveit_config
 * Run:   ros2 run hrd_moveit_config demo_dual_arm_reach_cpp
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/bounding_volume.hpp>
#include <moveit_msgs/msg/workspace_parameters.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

using namespace std::chrono_literals;
using MoveGroup = moveit_msgs::action::MoveGroup;
using GoalHandle = rclcpp_action::ClientGoalHandle<MoveGroup>;

// ── helpers ──────────────────────────────────────────────────────────────────

// Quaternion (xyzw) from two axes:
//   palm_dir → TCP local +Y  (palm normal, toward object)
//   x_pref  → TCP local +X  (preferred: cylinder axis / up, projected to palm plane)
static void quat_from_axes(double palm_x, double palm_y, double palm_z,
                           double xpref_x, double xpref_y, double xpref_z,
                           double &qx, double &qy, double &qz, double &qw) {
  // Normalize Y = palm direction
  double my = std::sqrt(palm_x*palm_x + palm_y*palm_y + palm_z*palm_z);
  if (my < 1e-9) { qx=qy=qz=0; qw=1; return; }
  double Yx = palm_x/my, Yy = palm_y/my, Yz = palm_z/my;

  // Project X_pref orthogonal to Y → X axis
  double dot = Yx*xpref_x + Yy*xpref_y + Yz*xpref_z;
  double Xx = xpref_x - dot*Yx;
  double Xy = xpref_y - dot*Yy;
  double Xz = xpref_z - dot*Yz;
  double mx = std::sqrt(Xx*Xx + Xy*Xy + Xz*Xz);
  if (mx < 1e-9) { qx=qy=qz=0; qw=1; return; }
  Xx/=mx; Xy/=mx; Xz/=mx;

  // Z = X × Y
  double Zx = Xy*Yz - Xz*Yy;
  double Zy = Xz*Yx - Xx*Yz;
  double Zz = Xx*Yy - Xy*Yx;

  // Rotation matrix [X Y Z] → quaternion (xyzw)
  double tr = Xx + Yy + Zz;
  double s;
  if (tr > 0) {
    s = std::sqrt(tr + 1.0) * 2.0;
    qx = (Zy - Yz) / s;  qy = (Xz - Zx) / s;
    qz = (Yx - Xy) / s;  qw = 0.25 * s;
  } else if (Xx > Yy && Xx > Zz) {
    s = std::sqrt(1.0 + Xx - Yy - Zz) * 2.0;
    qx = 0.25 * s;       qy = (Xy + Yx) / s;
    qz = (Xz + Zx) / s;  qw = (Zy - Yz) / s;
  } else if (Yy > Zz) {
    s = std::sqrt(1.0 + Yy - Xx - Zz) * 2.0;
    qx = (Xy + Yx) / s;  qy = 0.25 * s;
    qz = (Yz + Zy) / s;  qw = (Xz - Zx) / s;
  } else {
    s = std::sqrt(1.0 + Zz - Xx - Yy) * 2.0;
    qx = (Xz + Zx) / s;  qy = (Yz + Zy) / s;
    qz = 0.25 * s;       qw = (Yx - Xy) / s;
  }
}

static moveit_msgs::msg::PositionConstraint
make_pos(const std::string &link, double x, double y, double z, double tol=0.05) {
  moveit_msgs::msg::PositionConstraint pc;
  pc.header.frame_id = "base_link";
  pc.link_name = link;
  pc.weight = 1.0;
  shape_msgs::msg::SolidPrimitive sp;
  sp.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  sp.dimensions = {tol};
  geometry_msgs::msg::Pose ps;
  ps.position.x=x; ps.position.y=y; ps.position.z=z; ps.orientation.w=1;
  pc.constraint_region.primitives.push_back(sp);
  pc.constraint_region.primitive_poses.push_back(ps);
  return pc;
}

static moveit_msgs::msg::OrientationConstraint
make_orient(const std::string &link, double qx, double qy, double qz, double qw,
            double tol = 0.15, double w = 1.0) {
  moveit_msgs::msg::OrientationConstraint oc;
  oc.header.frame_id = "base_link";
  oc.link_name = link;
  oc.orientation.x=qx; oc.orientation.y=qy; oc.orientation.z=qz; oc.orientation.w=qw;
  oc.absolute_x_axis_tolerance = tol;
  oc.absolute_y_axis_tolerance = tol;
  oc.absolute_z_axis_tolerance = tol;
  oc.weight = w;
  return oc;
}

static moveit_msgs::msg::MotionPlanRequest
make_request(const std::string &group,
             const std::vector<moveit_msgs::msg::PositionConstraint> &pc,
             const std::vector<moveit_msgs::msg::OrientationConstraint> &oc) {
  moveit_msgs::msg::MotionPlanRequest req;
  req.group_name = group;
  req.max_velocity_scaling_factor = 0.5;
  req.max_acceleration_scaling_factor = 0.5;
  req.num_planning_attempts = 20;
  req.allowed_planning_time = 30.0;

  moveit_msgs::msg::Constraints goal;
  goal.position_constraints = pc;
  goal.orientation_constraints = oc;
  req.goal_constraints.push_back(goal);

  moveit_msgs::msg::WorkspaceParameters ws;
  ws.header.frame_id = "base_link";
  ws.min_corner.x=ws.min_corner.y=ws.min_corner.z = -2.0;
  ws.max_corner.x=ws.max_corner.y=ws.max_corner.z =  2.0;
  req.workspace_parameters = ws;
  return req;
}

// ── send request & wait ──────────────────────────────────────────────────────

bool plan_and_execute(rclcpp::Node::SharedPtr node,
                      rclcpp_action::Client<MoveGroup>::SharedPtr client,
                      const moveit_msgs::msg::MotionPlanRequest &req) {
  auto goal = MoveGroup::Goal();
  goal.request = req;
  goal.planning_options.plan_only = false;  // plan + execute

  RCLCPP_INFO(node->get_logger(), "Sending goal for '%s' (plan+execute) ...",
              req.group_name.c_str());

  auto send_fut = client->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(node, send_fut, 10s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "FAIL %s — send timeout", req.group_name.c_str());
    return false;
  }
  auto gh = send_fut.get();
  if (!gh) {
    RCLCPP_ERROR(node->get_logger(), "FAIL %s — goal rejected", req.group_name.c_str());
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "Planning '%s' ...", req.group_name.c_str());
  auto res_fut = client->async_get_result(gh);
  if (rclcpp::spin_until_future_complete(node, res_fut, 120s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "FAIL %s — result timeout", req.group_name.c_str());
    return false;
  }
  auto result = res_fut.get();
  auto ec = result.result->error_code.val;
  auto &traj = result.result->planned_trajectory;
  if (ec == moveit_msgs::msg::MoveItErrorCodes::SUCCESS &&
      !traj.joint_trajectory.points.empty()) {
    auto &pts = traj.joint_trajectory.points;
    auto dur = rclcpp::Duration(pts.back().time_from_start);
    // Log first → last joint values to confirm it's a real motion
    RCLCPP_INFO(node->get_logger(),
        "OK  %s — %zu pts, %.2fs  start[0]=%.3f end[0]=%.3f",
        req.group_name.c_str(), pts.size(), dur.seconds(),
        pts.front().positions.empty() ? -1.0 : pts.front().positions[0],
        pts.back().positions.empty() ? -1.0 : pts.back().positions[0]);
    return true;
  }
  RCLCPP_ERROR(node->get_logger(), "FAIL %s — error %d",
               req.group_name.c_str(), result.result->error_code.val);
  return false;
}

// ══════════════════════════════════════════════════════════════════════════════

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("demo_dual_arm_reach_cpp");
  auto log = node->get_logger();

  auto client = rclcpp_action::create_client<MoveGroup>(node, "/move_action");
  if (!client->wait_for_action_server(15s)) {
    RCLCPP_FATAL(log, "/move_action not available"); return 1;
  }
  RCLCPP_INFO(log, "Connected to /move_action");

  // ── joint-space helper ────────────────────────────────────────────────────
  auto plan_joint = [&](const std::string &group,
                         const std::vector<std::string> &names,
                         const std::vector<double> &vals) {
    moveit_msgs::msg::Constraints goal;
    for (size_t i=0; i<names.size(); i++) {
      moveit_msgs::msg::JointConstraint jc;
      jc.joint_name=names[i]; jc.position=vals[i];
      jc.tolerance_above=jc.tolerance_below=0.01; jc.weight=1.0;
      goal.joint_constraints.push_back(jc);
    }
    moveit_msgs::msg::MotionPlanRequest req;
    req.group_name=group;
    req.max_velocity_scaling_factor=req.max_acceleration_scaling_factor=0.5;
    req.num_planning_attempts=10; req.allowed_planning_time=5.0;
    req.goal_constraints.push_back(goal);
    moveit_msgs::msg::WorkspaceParameters ws;
    ws.header.frame_id="base_link";
    ws.min_corner.x=ws.min_corner.y=ws.min_corner.z=-2.0;
    ws.max_corner.x=ws.max_corner.y=ws.max_corner.z=2.0;
    req.workspace_parameters=ws;
    return plan_and_execute(node, client, req);
  };

  // ── Step 1 — Home ────────────────────────────────────────────────────────
  RCLCPP_INFO(log, "============================================================");
  RCLCPP_INFO(log, "Step 1/3: HOME");
  RCLCPP_INFO(log, "============================================================");

  plan_joint("left_arm",
    {"left_link_1_joint","left_link_2_joint","left_link_3_joint",
     "left_link_4_joint","left_link_5_joint","left_link_6_joint","left_link_7_joint"},
    {-0.0172,-0.0627,0.0,0.0,0.0,0.0,-1.8389});
  std::this_thread::sleep_for(500ms);

  plan_joint("right_arm",
    {"right_link_1_joint","right_link_2_joint","right_link_3_joint",
     "right_link_4_joint","right_link_5_joint","right_link_6_joint","right_link_7_joint"},
    {0.0858,-0.0877,-0.2231,-0.1121,0.0515,-0.1111,1.6307});
  std::this_thread::sleep_for(2s);

  // ── Target & approach ────────────────────────────────────────────────────
  double cyl_x=0.0, cyl_y=-0.65, cyl_z=0.85;

  // Left  arm: TCP X -> cylinder normal (approach dir),  TCP Y -> cylinder axis (up)
  double lx=0.0907, ly=-0.6390, lz=0.7414;
  double lqx=0.4723, lqy=-0.5790, lqz=0.3633, lqw=0.5566;

  // Right arm: TCP X -> cylinder normal (approach dir),  TCP Y -> cylinder axis (up)
  double rx=-0.0556, ry=-0.6286, rz=0.9247;
  double rqx=0.5345, rqy=0.5149, rqz=-0.4849, rqw=0.4627;

  // ── Step 2 — Right arm FIRST ─────────────────────────────────────────────
  RCLCPP_INFO(log, "============================================================");
  RCLCPP_INFO(log, "Step 2/3: RIGHT palm→cylinder [FIRST]");
  RCLCPP_INFO(log, "============================================================");

  auto right_req = make_request("right_arm",
      {make_pos("right_tcp_link", rx, ry, rz)},
      {make_orient("right_tcp_link", rqx, rqy, rqz, rqw, 0.25, 1.0)});  // ~14°
  bool right_ok = plan_and_execute(node, client, right_req);
  std::this_thread::sleep_for(1s);

  // ── Step 3 — Left arm SECOND ─────────────────────────────────────────────
  RCLCPP_INFO(log, "============================================================");
  RCLCPP_INFO(log, "Step 3/3: LEFT palm→cylinder [SECOND]");
  RCLCPP_INFO(log, "============================================================");

  auto left_req = make_request("left_arm",
      {make_pos("left_tcp_link", lx, ly, lz)},
      {make_orient("left_tcp_link", lqx, lqy, lqz, lqw, 0.15, 1.0)});  // ~8.5°
  bool left_ok = plan_and_execute(node, client, left_req);

  // ── Summary ──────────────────────────────────────────────────────────────
  RCLCPP_INFO(log, "============================================================");
  RCLCPP_INFO(log, "Left: %s  Right: %s",
              left_ok ? "OK" : "FAIL", right_ok ? "OK" : "FAIL");
  RCLCPP_INFO(log, "============================================================");

  rclcpp::shutdown();
  return (left_ok && right_ok) ? 0 : 1;
}

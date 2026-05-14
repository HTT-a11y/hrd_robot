/**
 * Dual-arm demo (C++): dynamic grasp pose + two-stage approach
 *   Stage 1 — free-space motion to pre-grasp (MoveGroupInterface::plan)
 *   Stage 2 — Cartesian straight-line approach to grasp (computeCartesianPath)
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

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/workspace_parameters.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>

using namespace std::chrono_literals;

// ═══════════════════════════════════════════════════════════════════════════
//  Dynamic grasp-pose solver (robot faces -Y, arms approach from +/-X)
// ═══════════════════════════════════════════════════════════════════════════

void calculate_dynamic_grasp_poses(
    const geometry_msgs::msg::Pose &cyl_pose,
    double safe_dist, double grasp_dist, double z_offset,
    geometry_msgs::msg::Pose &left_pre,  geometry_msgs::msg::Pose &left_grasp,
    geometry_msgs::msg::Pose &right_pre, geometry_msgs::msg::Pose &right_grasp)
{
    tf2::Transform T_world_to_cyl;
    tf2::fromMsg(cyl_pose, T_world_to_cyl);

    // ── Left arm (from -X side, palm → +X, fingers ↓) ─────────────────────
    //   Local Y(palm) → world +X (toward cylinder)
    //   Local Z(fingers) → world -Z (down)
    //   Local X(thumb) → Y×Z → world +Y (backward)
    tf2::Matrix3x3 R_left(0,1,0, 1,0,0, 0,0,-1);
    tf2::Vector3 t_left_pre  (-safe_dist,  0.0,  z_offset);
    tf2::Vector3 t_left_grasp(-grasp_dist,  0.0,  z_offset);

    tf2::toMsg(T_world_to_cyl * tf2::Transform(R_left, t_left_pre),  left_pre);
    tf2::toMsg(T_world_to_cyl * tf2::Transform(R_left, t_left_grasp), left_grasp);

    // ── Right arm (from +X side, palm → -X, fingers ↓) ────────────────────
    //   Local Y(palm) → world -X (toward cylinder)
    //   Local Z(fingers) → world -Z (down)
    //   Local X(thumb) → Y×Z → world -Y (forward)
    tf2::Matrix3x3 R_right(0,-1,0, -1,0,0, 0,0,-1);
    tf2::Vector3 t_right_pre  (safe_dist,  0.0, -z_offset);
    tf2::Vector3 t_right_grasp(grasp_dist,  0.0, -z_offset);

    tf2::toMsg(T_world_to_cyl * tf2::Transform(R_right, t_right_pre),  right_pre);
    tf2::toMsg(T_world_to_cyl * tf2::Transform(R_right, t_right_grasp), right_grasp);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Joint-space planner (HOME)
// ═══════════════════════════════════════════════════════════════════════════

bool plan_joint_home(rclcpp::Node::SharedPtr node,
                     moveit::planning_interface::MoveGroupInterface &mg,
                     const std::vector<double> &values) {
    mg.setMaxVelocityScalingFactor(0.5);
    mg.setMaxAccelerationScalingFactor(0.5);
    mg.setJointValueTarget(values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(node->get_logger(), "HOME '%s' ...", mg.getName().c_str());

    if (mg.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "FAIL %s HOME", mg.getName().c_str());
        return false;
    }
    mg.execute(plan);
    auto &pts = plan.trajectory_.joint_trajectory.points;
    auto dur = rclcpp::Duration(pts.back().time_from_start);
    RCLCPP_INFO(node->get_logger(), "OK  %s HOME — %zu pts, %.2fs",
                mg.getName().c_str(), pts.size(), dur.seconds());
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Two-stage grasp executor
// ═══════════════════════════════════════════════════════════════════════════

bool execute_two_stage_grasp(
    rclcpp::Logger log,
    moveit::planning_interface::MoveGroupInterface &group,
    const geometry_msgs::msg::Pose &pre_pose,
    const geometry_msgs::msg::Pose &grasp_pose,
    const std::string &name)
{
    // ── Stage 1: free-space motion to pre-grasp ──────────────────────────
    RCLCPP_INFO(log, "[%s] Stage 1 — moving to pre-grasp...", name.c_str());
    group.setPoseTarget(pre_pose);
    group.setGoalPositionTolerance(0.01);
    group.setGoalOrientationTolerance(0.5);  // ~28 deg — loose for multi-finger hand
    group.setPlanningTime(10.0);
    group.setNumPlanningAttempts(20);
    group.setMaxVelocityScalingFactor(0.5);
    group.setMaxAccelerationScalingFactor(0.5);

    moveit::planning_interface::MoveGroupInterface::Plan pre_plan;
    if (group.plan(pre_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(log, "FAIL [%s] pre-grasp plan", name.c_str());
        return false;
    }
    group.execute(pre_plan);
    auto &pts = pre_plan.trajectory_.joint_trajectory.points;
    auto dur = rclcpp::Duration(pts.back().time_from_start);
    RCLCPP_INFO(log, "OK   [%s] pre-grasp — %zu pts, %.2fs",
                name.c_str(), pts.size(), dur.seconds());

    std::this_thread::sleep_for(1s);

    // ── Stage 2: Cartesian straight-line approach to final grasp ─────────
    RCLCPP_INFO(log, "[%s] Stage 2 — Cartesian approach...", name.c_str());
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(grasp_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;       // 1 cm step
    const double jump_threshold = 0.0;   // no jumps allowed

    moveit_msgs::msg::Constraints path_constraints;
    bool avoid_collisions = false;
    moveit::core::MoveItErrorCode ec;

    double fraction = group.computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory,
        path_constraints, avoid_collisions, &ec);

    if (fraction >= 0.9) {
        group.execute(trajectory);
        RCLCPP_INFO(log, "OK   [%s] grasp — Cartesian %.0f%%",
                    name.c_str(), fraction * 100);
        return true;
    } else {
        RCLCPP_ERROR(log, "FAIL [%s] Cartesian only %.0f%%",
                     name.c_str(), fraction * 100);
        return false;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  main
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("demo_dual_arm_reach_cpp", opts);
    auto log = node->get_logger();

    // MoveGroupInterface needs a spinning node
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&]() { executor.spin(); });

    using MoveGroup = moveit::planning_interface::MoveGroupInterface;

    // ── HOME ─────────────────────────────────────────────────────────────
    RCLCPP_INFO(log, "=== Step 1: HOME ===");
    {
        MoveGroup left_arm(node, "left_arm");
        MoveGroup right_arm(node, "right_arm");
        left_arm.setEndEffectorLink("left_tcp_link");
        right_arm.setEndEffectorLink("right_tcp_link");

        plan_joint_home(node, left_arm,
            {-0.0172,-0.0627,0.0,0.0,0.0,0.0,-1.8389});
        std::this_thread::sleep_for(500ms);
        plan_joint_home(node, right_arm,
            {0.0858,-0.0877,-0.2231,-0.1121,0.0515,-0.1111,1.6307});
    }
    std::this_thread::sleep_for(2s);

    // ── Compute grasp poses dynamically ──────────────────────────────────
    geometry_msgs::msg::Pose cyl_pose;
    cyl_pose.position.x = 0.0;
    cyl_pose.position.y = -0.65;
    cyl_pose.position.z = 0.85;
    cyl_pose.orientation.w = 1.0;

    geometry_msgs::msg::Pose l_pre, l_grasp, r_pre, r_grasp;
    calculate_dynamic_grasp_poses(cyl_pose, 0.15, 0.02, 0.08,
                                  l_pre, l_grasp, r_pre, r_grasp);

    RCLCPP_INFO(log, "Left  pre:   (%.3f, %.3f, %.3f)",
        l_pre.position.x, l_pre.position.y, l_pre.position.z);
    RCLCPP_INFO(log, "Left  grasp: (%.3f, %.3f, %.3f)",
        l_grasp.position.x, l_grasp.position.y, l_grasp.position.z);
    RCLCPP_INFO(log, "Right pre:   (%.3f, %.3f, %.3f)",
        r_pre.position.x, r_pre.position.y, r_pre.position.z);
    RCLCPP_INFO(log, "Right grasp: (%.3f, %.3f, %.3f)",
        r_grasp.position.x, r_grasp.position.y, r_grasp.position.z);

    // ── Execute two-stage grasps ─────────────────────────────────────────
    RCLCPP_INFO(log, "============================================================");
    MoveGroup right_arm(node, "right_arm");
    MoveGroup left_arm(node, "left_arm");
    right_arm.setPoseReferenceFrame("base_link");
    left_arm.setPoseReferenceFrame("base_link");
    right_arm.setEndEffectorLink("right_tcp_link");
    left_arm.setEndEffectorLink("left_tcp_link");

    RCLCPP_INFO(log, "Step 2: RIGHT arm two-stage grasp");
    RCLCPP_INFO(log, "============================================================");
    bool right_ok = execute_two_stage_grasp(log, right_arm, r_pre, r_grasp, "right_arm");

    RCLCPP_INFO(log, "============================================================");
    RCLCPP_INFO(log, "Step 3: LEFT arm two-stage grasp");
    RCLCPP_INFO(log, "============================================================");
    bool left_ok = execute_two_stage_grasp(log, left_arm, l_pre, l_grasp, "left_arm");

    // ── Summary ──────────────────────────────────────────────────────────
    RCLCPP_INFO(log, "============================================================");
    RCLCPP_INFO(log, "Left: %s  Right: %s",
                left_ok ? "OK" : "FAIL", right_ok ? "OK" : "FAIL");
    RCLCPP_INFO(log, "============================================================");

    rclcpp::shutdown();
    spin_thread.join();
    return (left_ok && right_ok) ? 0 : 1;
}

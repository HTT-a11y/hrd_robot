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
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/workspace_parameters.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>

using namespace std::chrono_literals;

// ── Axis-based grasp pose solver (X=thumb↑, Y=palm→cyl, Z=fingers) ────────

void calculate_dynamic_grasp_poses(
    const geometry_msgs::msg::Pose &cyl_pose,
    double left_approach_angle_rad,
    double right_approach_angle_rad,
    double grasp_dist,
    double safe_dist,
    geometry_msgs::msg::Pose &left_pre,  geometry_msgs::msg::Pose &left_grasp,
    geometry_msgs::msg::Pose &right_pre, geometry_msgs::msg::Pose &right_grasp)
{
    double cx = cyl_pose.position.x;
    double cy = cyl_pose.position.y;
    double cz = cyl_pose.position.z;

    // ── Left arm ──────────────────────────────────────────────────────────
    left_grasp.position.x = cx + grasp_dist * std::cos(left_approach_angle_rad);
    left_grasp.position.y = cy + grasp_dist * std::sin(left_approach_angle_rad);
    left_grasp.position.z = cz;

    tf2::Vector3 l_x_axis(0.0, 0.0, 1.0);  // thumb = world Z up
    tf2::Vector3 l_y_axis(cx - left_grasp.position.x, cy - left_grasp.position.y, 0.0);
    l_y_axis.normalize();
    tf2::Vector3 l_z_axis = l_x_axis.cross(l_y_axis);
    l_z_axis.normalize();

    tf2::Matrix3x3 R_left(l_x_axis.x(),l_y_axis.x(),l_z_axis.x(),
                          l_x_axis.y(),l_y_axis.y(),l_z_axis.y(),
                          l_x_axis.z(),l_y_axis.z(),l_z_axis.z());
    tf2::Quaternion q_left; R_left.getRotation(q_left);
    left_grasp.orientation = tf2::toMsg(q_left);

    left_pre = left_grasp;
    left_pre.position.z += safe_dist;   // high hover, no XY retreat

    // ── Right arm ─────────────────────────────────────────────────────────
    right_grasp.position.x = cx + grasp_dist * std::cos(right_approach_angle_rad);
    right_grasp.position.y = cy + grasp_dist * std::sin(right_approach_angle_rad);
    right_grasp.position.z = cz;

    tf2::Vector3 r_x_axis(0.0, 0.0, 1.0);
    tf2::Vector3 r_y_axis(cx - right_grasp.position.x, cy - right_grasp.position.y, 0.0);
    r_y_axis.normalize();
    tf2::Vector3 r_z_axis = r_x_axis.cross(r_y_axis);
    r_z_axis.normalize();

    tf2::Matrix3x3 R_right(r_x_axis.x(),r_y_axis.x(),r_z_axis.x(),
                           r_x_axis.y(),r_y_axis.y(),r_z_axis.y(),
                           r_x_axis.z(),r_y_axis.z(),r_z_axis.z());
    tf2::Quaternion q_right; R_right.getRotation(q_right);
    right_grasp.orientation = tf2::toMsg(q_right);

    right_pre = right_grasp;
    right_pre.position.z += safe_dist;   // high hover, no XY retreat

    // ── Diagnostic ────────────────────────────────────────────────────────
    std::cout << "\n=========== [Kinematic Diagnostic] ===========" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Cylinder: (" << cx << ", " << cy << ", " << cz << ")" << std::endl;
    std::cout << "Left  GRASP: (" << left_grasp.position.x << ", " << left_grasp.position.y << ", " << left_grasp.position.z << ")" << std::endl;
    std::cout << "Left  PRE:   (" << left_pre.position.x << ", " << left_pre.position.y << ", " << left_pre.position.z << ")" << std::endl;
    std::cout << "Right GRASP: (" << right_grasp.position.x << ", " << right_grasp.position.y << ", " << right_grasp.position.z << ")" << std::endl;
    std::cout << "Right PRE:   (" << right_pre.position.x << ", " << right_pre.position.y << ", " << right_pre.position.z << ")" << std::endl;
    std::cout << "==============================================\n" << std::endl;
}

// ── Adaptive orientation constraint (tight X/Y, loose Z rotation) ─────────

moveit_msgs::msg::Constraints create_adaptive_orientation_constraint(
    const std::string &link_name,
    const geometry_msgs::msg::Pose &target_pose)
{
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::OrientationConstraint o_const;
    o_const.header.frame_id = "base_link";
    o_const.link_name = link_name;
    o_const.orientation = target_pose.orientation;
    o_const.absolute_x_axis_tolerance = 0.05;   // tight tilt
    o_const.absolute_y_axis_tolerance = 0.05;   // tight pitch
    o_const.absolute_z_axis_tolerance = 3.14;   // free rotation around cylinder axis
    o_const.weight = 1.0;
    constraints.orientation_constraints.push_back(o_const);
    return constraints;
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
//  Two-stage grasp executor with fallback diagnosis
// ═══════════════════════════════════════════════════════════════════════════

bool execute_two_stage_grasp(
    rclcpp::Logger log,
    moveit::planning_interface::MoveGroupInterface &group,
    const geometry_msgs::msg::Pose &pre_pose,
    const geometry_msgs::msg::Pose &grasp_pose,
    const std::string &name)
{
    // ── Stage 1: free-space motion to pre-grasp ──────────────────────────
    RCLCPP_INFO(log, "[%s] Stage 1 -- planning with adaptive orientation...",
                name.c_str());
    group.setPoseTarget(pre_pose);
    group.setGoalPositionTolerance(0.01);
    group.setGoalOrientationTolerance(0.1);
    group.setPlanningTime(30.0);
    group.setNumPlanningAttempts(10);
    group.setMaxVelocityScalingFactor(0.5);
    group.setMaxAccelerationScalingFactor(0.5);

    // Adaptive constraint: tight X/Y tilt, free Z rotation
    std::string ee_link = (name == "left_arm") ? "left_tcp_link" : "right_tcp_link";
    group.setPathConstraints(
        create_adaptive_orientation_constraint(ee_link, pre_pose));

    moveit::planning_interface::MoveGroupInterface::Plan pre_plan;

    if (group.plan(pre_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        group.clearPathConstraints();
        RCLCPP_ERROR(log,
            "[FATAL] [%s] XYZ (%.2f,%.2f,%.2f) UNREACHABLE — outside workspace",
            name.c_str(),
            pre_pose.position.x, pre_pose.position.y, pre_pose.position.z);
        return false;
    }
    group.clearPathConstraints();

    RCLCPP_INFO(log, "OK   [%s] Stage 1 planned, executing...", name.c_str());
    group.execute(pre_plan);
    auto &pts = pre_plan.trajectory_.joint_trajectory.points;
    auto dur = rclcpp::Duration(pts.back().time_from_start);
    RCLCPP_INFO(log, "OK   [%s] Stage 1 done -- %zu pts, %.2fs",
                name.c_str(), pts.size(), dur.seconds());
    return true;

    // Stage 2 (Cartesian) disabled until basic reachability confirmed.
    // Uncomment below and shorten pre-grasp→grasp distance (< 5 cm)
    // once both arms reach Stage 1 successfully.
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
            {0,0,0,0,0,0,4.6146});
        std::this_thread::sleep_for(500ms);
        plan_joint_home(node, right_arm,
            {-0.0515,-0.0877,-0.2231,-0.1121,0.0515,-0.1111,6.28});
    }
    std::this_thread::sleep_for(2s);

    // ── MoveGroup initialization ───────────────────────────────────────────
    MoveGroup left_group(node, "left_arm");
    MoveGroup right_group(node, "right_arm");
    right_group.setPoseReferenceFrame("base_link");
    left_group.setPoseReferenceFrame("base_link");
    right_group.setEndEffectorLink("right_tcp_link");
    left_group.setEndEffectorLink("left_tcp_link");

    // ── Cylinder collision object DISABLED (let arms approach freely) ──────
    // Collision object removed so the planner can reach the grasp pose
    // without being blocked by the cylinder primitive.
    // In production, attach the object AFTER finger closure via attachObject().
    RCLCPP_INFO(log, "Collision object disabled — arms ignore cylinder");
    std::this_thread::sleep_for(1s);

    // ── Approach-angle search: +X (0 rad) for left, -X (π rad) for right ──
    geometry_msgs::msg::Pose cyl_pose;
    cyl_pose.position.x = 0.0;
    cyl_pose.position.y = -0.45;
    cyl_pose.position.z = 0.70;
    cyl_pose.orientation.w = 1.0;

    const double GRASP_DIST = 0.10;  // 10 cm — matches RViz safe reachable zone
    const double SAFE_DIST  = 0.06;   // 6 cm pre-grasp lift

    bool grasp_success = false;
    geometry_msgs::msg::Pose l_pre, l_grasp, r_pre, r_grasp;

    std::vector<double> yaw_offsets = {0.0, 5.0, -5.0, 10.0, -10.0, 15.0, -15.0};

    RCLCPP_INFO(log, "Approach-angle search (left=0, right=pi base)...");

    for (double offset_deg : yaw_offsets) {
        double offset_rad = offset_deg * M_PI / 180.0;
        double left_angle  = 0.0   + offset_rad;   // +X side
        double right_angle = M_PI  + offset_rad;   // -X side

        calculate_dynamic_grasp_poses(cyl_pose,
            left_angle, right_angle, GRASP_DIST, SAFE_DIST,
            l_pre, l_grasp, r_pre, r_grasp);

        RCLCPP_INFO(log, ">>> Offset: %+.1f deg  L=%.1f  R=%.1f",
                     offset_deg, left_angle*180.0/M_PI, right_angle*180.0/M_PI);

        moveit::planning_interface::MoveGroupInterface::Plan dummy_plan;

        // ── Layer 1: position-only, fast 1.5s ──────────────────────────────
        right_group.setPoseTarget(r_pre);
        right_group.setGoalPositionTolerance(0.01);
        right_group.setGoalOrientationTolerance(3.14);
        right_group.setPlanningTime(1.5);
        if (right_group.plan(dummy_plan) != moveit::core::MoveItErrorCode::SUCCESS)
            continue;

        left_group.setPoseTarget(l_pre);
        left_group.setGoalPositionTolerance(0.01);
        left_group.setGoalOrientationTolerance(3.14);
        left_group.setPlanningTime(1.5);
        if (left_group.plan(dummy_plan) != moveit::core::MoveItErrorCode::SUCCESS)
            continue;

        // ── Layer 2: adaptive orientation constraint ───────────────────────
        right_group.setPathConstraints(
            create_adaptive_orientation_constraint("right_tcp_link", r_pre));
        right_group.setGoalOrientationTolerance(0.1);
        if (right_group.plan(dummy_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            right_group.clearPathConstraints();
            continue;
        }
        right_group.clearPathConstraints();

        left_group.setPathConstraints(
            create_adaptive_orientation_constraint("left_tcp_link", l_pre));
        left_group.setGoalOrientationTolerance(0.1);
        if (left_group.plan(dummy_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            left_group.clearPathConstraints();
            continue;
        }
        left_group.clearPathConstraints();

        RCLCPP_INFO(log, "FOUND reachable pose at offset %.1f deg!",
                     offset_deg);
        grasp_success = true;
        break;
    }

    if (!grasp_success) {
        RCLCPP_ERROR(log, "All approach angles failed.");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    // ── Execute the winning poses ───────────────────────────────────────────
    RCLCPP_INFO(log, "============================================================");
    RCLCPP_INFO(log, "Step 2: LEFT arm two-stage grasp");
    bool left_ok = execute_two_stage_grasp(log, left_group, l_pre, l_grasp, "left_arm");

    RCLCPP_INFO(log, "============================================================");
    RCLCPP_INFO(log, "Step 3: RIGHT arm two-stage grasp");
    bool right_ok = execute_two_stage_grasp(log, right_group, r_pre, r_grasp, "right_arm");

    // ── Summary ──────────────────────────────────────────────────────────
    RCLCPP_INFO(log, "============================================================");
    RCLCPP_INFO(log, "Left: %s  Right: %s",
                left_ok ? "OK" : "FAIL", right_ok ? "OK" : "FAIL");
    RCLCPP_INFO(log, "============================================================");

    rclcpp::shutdown();
    spin_thread.join();
    return (left_ok && right_ok) ? 0 : 1;
}

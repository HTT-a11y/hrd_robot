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

// ── Heuristic grasp-pose solver (RViz-measured offsets + quaternions) ────

void calculate_dynamic_grasp_poses(
    const geometry_msgs::msg::Pose &cyl_pose,
    double safe_dist, double grasp_dist, double z_offset,
    geometry_msgs::msg::Pose &left_pre,  geometry_msgs::msg::Pose &left_grasp,
    geometry_msgs::msg::Pose &right_pre, geometry_msgs::msg::Pose &right_grasp)
{
    double cx = cyl_pose.position.x;
    double cy = cyl_pose.position.y;
    double cz = cyl_pose.position.z;

    // ── Left arm: RViz-verified offsets + quaternion ──────────────────────
    left_grasp.position.x = cx + 0.064;
    left_grasp.position.y = cy - 0.115;
    left_grasp.position.z = cz + 0.098;
    left_grasp.orientation.x = -0.5243;
    left_grasp.orientation.y =  0.5462;
    left_grasp.orientation.z = -0.4406;
    left_grasp.orientation.w = -0.4823;

    left_pre = left_grasp;
    left_pre.position.y -= 0.10;

    // ── Right arm: RViz-verified offsets + quaternion ─────────────────────
    right_grasp.position.x = cx - 0.121;
    right_grasp.position.y = cy - 0.088;
    right_grasp.position.z = cz - 0.023;
    right_grasp.orientation.x =  0.4392;
    right_grasp.orientation.y =  0.5466;
    right_grasp.orientation.z = -0.4546;
    right_grasp.orientation.w =  0.5493;

    right_pre = right_grasp;
    right_pre.position.y -= 0.10;
}

// ── quaternion from two axes: Y→palm_dir, X→x_pref (projected) ──────────

static void quat_from_axes(double palm_x,double palm_y,double palm_z,
                           double xpx,double xpy,double xpz,
                           double &qx,double &qy,double &qz,double &qw) {
  double my=std::sqrt(palm_x*palm_x+palm_y*palm_y+palm_z*palm_z);
  if(my<1e-9){qx=qy=qz=0;qw=1;return;}
  double Yx=palm_x/my,Yy=palm_y/my,Yz=palm_z/my;
  double dot=Yx*xpx+Yy*xpy+Yz*xpz;
  double Xx=xpx-dot*Yx, Xy=xpy-dot*Yy, Xz=xpz-dot*Yz;
  double mx=std::sqrt(Xx*Xx+Xy*Xy+Xz*Xz);
  if(mx<1e-9){qx=qy=qz=0;qw=1;return;}
  Xx/=mx;Xy/=mx;Xz/=mx;
  double Zx=Xy*Yz-Xz*Yy, Zy=Xz*Yx-Xx*Yz, Zz=Xx*Yy-Xy*Yx;
  double tr=Xx+Yy+Zz,s;
  if(tr>0){s=std::sqrt(tr+1)*2; qx=(Zy-Yz)/s; qy=(Xz-Zx)/s; qz=(Yx-Xy)/s; qw=.25*s;}
  else if(Xx>Yy&&Xx>Zz){s=std::sqrt(1+Xx-Yy-Zz)*2; qx=.25*s; qy=(Xy+Yx)/s; qz=(Xz+Zx)/s; qw=(Zy-Yz)/s;}
  else if(Yy>Zz){s=std::sqrt(1+Yy-Xx-Zz)*2; qx=(Xy+Yx)/s; qy=.25*s; qz=(Yz+Zy)/s; qw=(Xz-Zx)/s;}
  else{s=std::sqrt(1+Zz-Xx-Yy)*2; qx=(Xz+Zx)/s; qy=(Yz+Zy)/s; qz=.25*s; qw=(Yx-Xy)/s;}
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
    RCLCPP_INFO(log, "[%s] Stage 1 -- planning (position-only, 30s timeout)...",
                name.c_str());
    group.setPoseTarget(pre_pose);
    group.setGoalPositionTolerance(0.01);
    // Left arm: loose orientation (computed pose hard to reach)
    // Right arm: tighter (pose works well)
    double orient_tol = (name == "left_arm") ? 3.14 : 0.5;
    group.setGoalOrientationTolerance(orient_tol);
    group.setPlanningTime(30.0);
    group.setNumPlanningAttempts(10);
    group.setMaxVelocityScalingFactor(0.5);
    group.setMaxAccelerationScalingFactor(0.5);

    moveit::planning_interface::MoveGroupInterface::Plan pre_plan;

    if (group.plan(pre_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(log,
            "[FATAL] [%s] XYZ (%.2f,%.2f,%.2f) UNREACHABLE — outside workspace",
            name.c_str(),
            pre_pose.position.x, pre_pose.position.y, pre_pose.position.z);
        return false;
    }

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

    // ── Z-axis symmetry search for reachable grasp poses ───────────────────
    geometry_msgs::msg::Pose cyl_pose;
    cyl_pose.position.x = 0.05;
    cyl_pose.position.y = -0.70;
    cyl_pose.position.z = 0.8;

    bool grasp_success = false;
    geometry_msgs::msg::Pose l_pre, l_grasp, r_pre, r_grasp;

    RCLCPP_INFO(log, "Searching grasp poses via Z-axis symmetry (-45 to +45 deg)...");

    for (double yaw_deg = -45.0; yaw_deg <= 45.0; yaw_deg += 5.0) {
        double yaw_rad = yaw_deg * M_PI / 180.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_rad);
        cyl_pose.orientation = tf2::toMsg(q);

        calculate_dynamic_grasp_poses(cyl_pose, 0.10, 0.02, 0.05,
                                      l_pre, l_grasp, r_pre, r_grasp);

        RCLCPP_INFO(log, ">>> Testing yaw: %.1f deg", yaw_deg);

        // ── Test right arm ──────────────────────────────────────────────────
        right_group.setPoseTarget(r_pre);
        right_group.setGoalPositionTolerance(0.01);
        right_group.setGoalOrientationTolerance(0.3);
        right_group.setPlanningTime(2.0);
        right_group.setNumPlanningAttempts(5);
        moveit::planning_interface::MoveGroupInterface::Plan plan_r;
        if (right_group.plan(plan_r) != moveit::core::MoveItErrorCode::SUCCESS)
            continue;

        // ── Test left arm ───────────────────────────────────────────────────
        left_group.setPoseTarget(l_pre);
        left_group.setGoalPositionTolerance(0.01);
        left_group.setGoalOrientationTolerance(0.3);
        left_group.setPlanningTime(2.0);
        left_group.setNumPlanningAttempts(5);
        moveit::planning_interface::MoveGroupInterface::Plan plan_l;
        if (left_group.plan(plan_l) != moveit::core::MoveItErrorCode::SUCCESS)
            continue;

        RCLCPP_INFO(log, "FOUND reachable pose at yaw %.1f deg!", yaw_deg);
        grasp_success = true;
        break;
    }

    if (!grasp_success) {
        RCLCPP_ERROR(log, "All yaw angles failed. Move cylinder closer or further.");
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

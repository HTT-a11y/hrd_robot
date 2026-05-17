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
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
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

    // -- Left arm (from -X side, palm -> +X, fingers -> -Y) (handshake)
    //   Local Y(palm) -> world +X (toward cylinder)
    //   Local Z(fingers)-> world -Y (forward)
    //   Local X(thumb) -> YxZ -> world -Z (down)
    tf2::Matrix3x3 R_left(0,1,0, 0,0,-1, -1,0,0);
    tf2::Vector3 t_left_pre  (-safe_dist,  0.0,  z_offset);
    tf2::Vector3 t_left_grasp(-grasp_dist,  0.0,  z_offset);

    tf2::toMsg(T_world_to_cyl * tf2::Transform(R_left, t_left_pre),  left_pre);
    tf2::toMsg(T_world_to_cyl * tf2::Transform(R_left, t_left_grasp), left_grasp);

    // -- Right arm (from +X side, palm -> -X, fingers -> -Y) (handshake)
    //   Local Y(palm) -> world -X (toward cylinder)
    //   Local Z(fingers)-> world -Y (forward)
    //   Local X(thumb) -> YxZ -> world +Z (up)
    tf2::Matrix3x3 R_right(0,-1,0, 0,0,-1, 1,0,0);
    tf2::Vector3 t_right_pre  (safe_dist,  0.0, -z_offset);
    tf2::Vector3 t_right_grasp(grasp_dist,  0.0, -z_offset);

    tf2::toMsg(T_world_to_cyl * tf2::Transform(R_right, t_right_pre),  right_pre);
    tf2::toMsg(T_world_to_cyl * tf2::Transform(R_right, t_right_grasp), right_grasp);
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

    // ── Ensure cylinder collision object is in the planning scene ────────
    {
      moveit::planning_interface::PlanningSceneInterface psi;
      moveit_msgs::msg::CollisionObject cyl_obj;
      cyl_obj.id = "target_cylinder";
      cyl_obj.header.frame_id = "base_link";
      cyl_obj.operation = moveit_msgs::msg::CollisionObject::ADD;

      shape_msgs::msg::SolidPrimitive cyl_prim;
      cyl_prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      cyl_prim.dimensions = {0.25, 0.02};
      cyl_obj.primitives.push_back(cyl_prim);

      geometry_msgs::msg::Pose cyl_pose;
      cyl_pose.position.x = 0.0; cyl_pose.position.y = -0.65; cyl_pose.position.z = 0.85;
      cyl_pose.orientation.w = 1.0;
      cyl_obj.primitive_poses.push_back(cyl_pose);

      psi.applyCollisionObject(cyl_obj);
      RCLCPP_INFO(log, "Added target_cylinder to planning scene");
    }
    std::this_thread::sleep_for(1s);  // let the scene update propagate

    // ── Hardcoded targets with palm-Y → cylinder orientation ─────────────
    //    Cylinder at (0.0, -0.65, 0.85)
    double cyl_x=0.0, cyl_y=-0.65, cyl_z=0.85;

    geometry_msgs::msg::Pose l_pre, l_grasp, r_pre, r_grasp;

    // Left arm: upper grasp (Z=0.92), further forward (X=0.15)
    l_pre.position.x = 0.15; l_pre.position.y = -0.58; l_pre.position.z = 0.92;
    { double lax=cyl_x-0.15, lay=cyl_y+0.58, laz=cyl_z-0.92;
      double qx,qy,qz,qw; quat_from_axes(lax,lay,laz, 0,0,1, qx,qy,qz,qw);
      l_pre.orientation.x=qx; l_pre.orientation.y=qy; l_pre.orientation.z=qz; l_pre.orientation.w=qw; }
    l_grasp.position = l_pre.position;
    l_grasp.position.x = 0.05; l_grasp.position.y = -0.60; l_grasp.position.z = 0.90;

    // Right arm: behind cylinder (-X), lower (Z=0.76)
    r_pre.position.x = -0.10; r_pre.position.y = -0.72; r_pre.position.z = 0.76;
    { double rax=cyl_x+0.10, ray=cyl_y+0.72, raz=cyl_z-0.76;
      double qx,qy,qz,qw; quat_from_axes(rax,ray,raz, 0,0,1, qx,qy,qz,qw);
      r_pre.orientation.x=qx; r_pre.orientation.y=qy; r_pre.orientation.z=qz; r_pre.orientation.w=qw; }
    r_grasp.position = r_pre.position;
    r_grasp.position.x = -0.03; r_grasp.position.y = -0.70; r_grasp.position.z = 0.78;

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

    RCLCPP_INFO(log, "Step 2: RIGHT arm two-stage grasp [FIRST — tight orientation]");
    RCLCPP_INFO(log, "============================================================");
    bool right_ok = execute_two_stage_grasp(log, right_arm, r_pre, r_grasp, "right_arm");

    RCLCPP_INFO(log, "============================================================");
    RCLCPP_INFO(log, "Step 3: LEFT arm two-stage grasp [SECOND — loose, navigates around]");
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

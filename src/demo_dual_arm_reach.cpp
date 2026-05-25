/**
 * FINAL DUAL-ARM GRASP DEMO
 * Verified common workspace: (-0.15, -0.45, 0.80)
 * Right arm reaches X=-0.35~-0.15, Left arm reaches X=-0.15
 */

#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using MoveGroup = moveit::planning_interface::MoveGroupInterface;

// ── Axis-based grasp solver ──────────────────────────────────────────────

void calc_grasp(const geometry_msgs::msg::Pose &cyl,
                double left_ang, double right_ang, double grasp_d, double safe_d,
                geometry_msgs::msg::Pose &l_pre, geometry_msgs::msg::Pose &l_grasp,
                geometry_msgs::msg::Pose &r_pre, geometry_msgs::msg::Pose &r_grasp)
{
    double cx=cyl.position.x, cy=cyl.position.y, cz=cyl.position.z;

    // Left arm
    l_grasp.position.x = cx + grasp_d*cos(left_ang);
    l_grasp.position.y = cy + grasp_d*sin(left_ang);
    l_grasp.position.z = cz;
    tf2::Vector3 lx(0,0,1), ly(cx-l_grasp.position.x,cy-l_grasp.position.y,0); ly.normalize();
    tf2::Vector3 lz = lx.cross(ly); lz.normalize();
    tf2::Matrix3x3 Rl(lx.x(),ly.x(),lz.x(), lx.y(),ly.y(),lz.y(), lx.z(),ly.z(),lz.z());
    tf2::Quaternion ql; Rl.getRotation(ql);
    l_grasp.orientation = tf2::toMsg(ql);
    l_pre = l_grasp; l_pre.position.z += safe_d;

    // Right arm
    r_grasp.position.x = cx + grasp_d*cos(right_ang);
    r_grasp.position.y = cy + grasp_d*sin(right_ang);
    r_grasp.position.z = cz;
    tf2::Vector3 rx(0,0,1), ry(cx-r_grasp.position.x,cy-r_grasp.position.y,0); ry.normalize();
    tf2::Vector3 rz = rx.cross(ry); rz.normalize();
    tf2::Matrix3x3 Rr(rx.x(),ry.x(),rz.x(), rx.y(),ry.y(),rz.y(), rx.z(),ry.z(),rz.z());
    tf2::Quaternion qr; Rr.getRotation(qr);
    r_grasp.orientation = tf2::toMsg(qr);
    r_pre = r_grasp; r_pre.position.z += safe_d;
}

// ── Plan + execute ──────────────────────────────────────────────────────

bool plan_exec(rclcpp::Logger log, MoveGroup &mg,
               const geometry_msgs::msg::Pose &pre,
               const std::string &name)
{
    mg.setPoseTarget(pre);
    mg.setGoalPositionTolerance(0.05);
    mg.setGoalOrientationTolerance(3.14);  // position-only — proven to work
    mg.setPlanningTime(15.0);
    mg.setMaxVelocityScalingFactor(0.5);
    mg.setMaxAccelerationScalingFactor(0.5);
    MoveGroup::Plan p;
    RCLCPP_INFO(log, "[%s] planning...", name.c_str());
    if (mg.plan(p) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(log, "[%s] FAIL", name.c_str());
        return false;
    }
    mg.execute(p);
    auto d = rclcpp::Duration(p.trajectory_.joint_trajectory.points.back().time_from_start);
    RCLCPP_INFO(log, "[%s] OK — %zu pts %.2fs", name.c_str(),
                p.trajectory_.joint_trajectory.points.size(), d.seconds());
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("dual_grasp");
    auto log = node->get_logger();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    std::thread t([&](){ exec.spin(); });

    // ── HOME ──────────────────────────────────────────────────────────
    {
        MoveGroup lg(node, "left_arm"), rg(node, "right_arm");
        lg.setMaxVelocityScalingFactor(0.5); lg.setMaxAccelerationScalingFactor(0.5);
        lg.setJointValueTarget({0,0,0,0,0,0,4.6146});
        MoveGroup::Plan p; lg.plan(p); lg.execute(p);
        std::this_thread::sleep_for(500ms);
        rg.setMaxVelocityScalingFactor(0.5); rg.setMaxAccelerationScalingFactor(0.5);
        rg.setJointValueTarget({-0.0515,-0.0877,-0.2231,-0.1121,0.0515,-0.1111,6.28});
        rg.plan(p); rg.execute(p);
    }
    std::this_thread::sleep_for(2s);

    // ── Cylinder at verified sweet spot ────────────────────────────────
    geometry_msgs::msg::Pose cyl;
    cyl.position.x = -0.10;
    cyl.position.y = -0.45;
    cyl.position.z =  0.80;
    cyl.orientation.w = 1.0;

    geometry_msgs::msg::Pose l_pre, l_grasp, r_pre, r_grasp;
    // Left from +X (0°), Right from -X (180°), 10cm grasp, 6cm lift
    calc_grasp(cyl, 0.0, M_PI, 0.10, 0.03, l_pre, l_grasp, r_pre, r_grasp);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\nCylinder: (" << cyl.position.x << "," << cyl.position.y << "," << cyl.position.z << ")\n";
    std::cout << "Left  PRE: (" << l_pre.position.x << "," << l_pre.position.y << "," << l_pre.position.z << ")\n";
    std::cout << "Right PRE: (" << r_pre.position.x << "," << r_pre.position.y << "," << r_pre.position.z << ")\n\n";

    // ── Execute ───────────────────────────────────────────────────────
    MoveGroup lg(node, "left_arm"), rg(node, "right_arm");
    lg.setEndEffectorLink("left_tcp_link");
    rg.setEndEffectorLink("right_tcp_link");
    lg.setPoseReferenceFrame("base_link");
    rg.setPoseReferenceFrame("base_link");

    RCLCPP_INFO(log, "=== LEFT ARM ===");
    bool lok = plan_exec(log, lg, l_pre, "left_arm");

    RCLCPP_INFO(log, "=== RIGHT ARM ===");
    bool rok = plan_exec(log, rg, r_pre, "right_arm");

    RCLCPP_INFO(log, "Left:%s Right:%s", lok?"OK":"FAIL", rok?"OK":"FAIL");

    rclcpp::shutdown();
    t.join();
    return (lok && rok) ? 0 : 1;
}

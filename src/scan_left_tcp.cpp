/**
 * LEFT ARM TCP WORKSPACE SCAN
 *
 * Build: colcon build --packages-select hrd_moveit_config
 * Run:   ros2 run hrd_moveit_config scan_left_tcp
 */

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std::chrono_literals;
using MoveGroup = moveit::planning_interface::MoveGroupInterface;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("scan_left_tcp");
    auto log = node->get_logger();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    std::thread t([&]() { exec.spin(); });

    // ── HOME ──────────────────────────────────────────────────────────
    {
        MoveGroup lg(node, "left_arm"), rg(node, "right_arm");
        lg.setMaxVelocityScalingFactor(0.5);
        lg.setMaxAccelerationScalingFactor(0.5);
        lg.setJointValueTarget({0, 0, 0, 0, 0, 0, 4.6146});
        MoveGroup::Plan p;
        lg.plan(p);
        lg.execute(p);
        std::this_thread::sleep_for(500ms);
        rg.setMaxVelocityScalingFactor(0.5);
        rg.setMaxAccelerationScalingFactor(0.5);
        rg.setJointValueTarget(
            {-0.0515, -0.0877, -0.2231, -0.1121, 0.0515, -0.1111, 6.28});
        rg.plan(p);
        rg.execute(p);
    }
    std::this_thread::sleep_for(2s);

    // ── Scan setup ────────────────────────────────────────────────────
    MoveGroup lg(node, "left_arm");
    lg.setEndEffectorLink("left_tcp_link");
    lg.setGoalPositionTolerance(0.05);
    lg.setGoalOrientationTolerance(3.14);  // position-only
    lg.setPlanningTime(5.0);
    lg.setNumPlanningAttempts(1);
    lg.setMaxVelocityScalingFactor(0.5);
    lg.setMaxAccelerationScalingFactor(0.5);

    RCLCPP_INFO(log, "=== LEFT ARM TCP WORKSPACE SCAN ===");

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "\n        ";
    for (double yy = -0.15; yy >= -0.55; yy -= 0.10)
        std::cout << "  Y=" << std::setw(5) << yy;
    std::cout << "\n";

    for (double zz : {0.80, 0.85, 0.90}) {
        std::cout << "Z=" << zz << "  ";
        for (double xx = -0.35; xx <= 0.15; xx += 0.10) {
            std::cout << "\n  X=" << std::setw(5) << xx << " ";
            for (double yy = -0.15; yy >= -0.55; yy -= 0.10) {
                geometry_msgs::msg::Pose tp;
                tp.position.x = xx;
                tp.position.y = yy;
                tp.position.z = zz;
                tp.orientation.w = 1.0;

                lg.setPoseTarget(tp);
                MoveGroup::Plan dummy;
                bool ok = (lg.plan(dummy) ==
                           moveit::core::MoveItErrorCode::SUCCESS);
                std::cout << (ok ? "  OK " : "  -- ");
            }
        }
        std::cout << "\n";
    }

    RCLCPP_INFO(log, "Scan complete.");
    rclcpp::shutdown();
    t.join();
    return 0;
}

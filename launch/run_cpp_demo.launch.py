"""
Launch the C++ dual-arm reach demo with full MoveIt configuration.
Passes kinematics.yaml (pick_ik) etc. to the node so MoveGroupInterface
can find the IK solver.

Usage:
  ros2 launch hrd_moveit_config run_cpp_demo.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("HRD", package_name="hrd_moveit_config").to_moveit_configs()

    demo_node = Node(
        package="hrd_moveit_config",
        executable="demo_dual_arm_reach_cpp",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([demo_node])

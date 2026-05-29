from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "HRD", package_name="hrd_moveit_config"
    ).to_moveit_configs()

    node = Node(
        package="hrd_moveit_config",
        executable="scan_left_tcp",
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

    return LaunchDescription([node])

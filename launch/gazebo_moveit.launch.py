"""
MoveIt + Gazebo co-simulation for HRD dual-arm robot.

The gazebo_ros2_control plugin inside Gazebo handles:
  - Hardware interfaces (GazeboSystem)
  - Internal controller_manager
  - Loading/spawning controllers from ros2_controllers.yaml

No standalone ros2_control_node is needed.
"""

import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
)


def _gazebo_ready_urdf(urdf_str, pkg_share):
    urdf_str = re.sub(r'^\s*<\?xml[^>]*\?>\s*', '', urdf_str, count=1)
    mesh_dir = os.path.join(pkg_share, 'HRD_description', 'meshes')
    if os.path.isdir(mesh_dir):
        urdf_str = urdf_str.replace(
            'package://turn_on_hrd_robot/HRD_description/meshes/',
            'file://' + mesh_dir + '/',
        )
    urdf_str = urdf_str.replace('<material name="">', '<material name="gz_mat">')
    urdf_str = urdf_str.replace('<material>', '<material name="gz_mat">')
    return urdf_str


def generate_launch_description():
    pkg_share = get_package_share_directory('turn_on_hrd_robot')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_z = LaunchConfiguration('spawn_z')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('spawn_z', default_value='0.35'),
    ]

    # ── URDF with gazebo_ros2_control ───────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder('HRD ', package_name='hrd_moveit_config')
        .robot_description(
            mappings={
                'use_fake_hardware': 'false',
                'ros_hardware_interface': 'position',
                'initial_positions_file': os.path.join(
                    get_package_share_directory('hrd_moveit_config'),
                    'config', 'initial_positions.yaml',
                ),
            }
        )
        .robot_description_semantic()
        .robot_description_kinematics()
        .trajectory_execution()
        .planning_pipelines()
        .joint_limits()
        .to_moveit_configs()
    )

    raw_urdf = moveit_config.robot_description['robot_description']
    gazebo_urdf_str = _gazebo_ready_urdf(raw_urdf, pkg_share)

    fd, urdf_tmp = tempfile.mkstemp(
        prefix='hrd_gz_moveit_', suffix='.urdf', text=True,
    )
    with os.fdopen(fd, 'w', encoding='utf-8') as f:
        f.write(gazebo_urdf_str)

    robot_desc_param = {'robot_description': gazebo_urdf_str}

    # ── Gazebo ──────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={'verbose': 'false'}.items(),
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_desc_param, {'use_sim_time': use_sim_time}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'HRD',
            '-file', urdf_tmp,
            '-x', '0.0', '-y', '0.0', '-z', spawn_z,
            '-timeout', '120.0', '-unpause',
        ],
    )

    # ── MoveGroup + RViz ────────────────────────────────────────────────────
    mg_ld = generate_move_group_launch(moveit_config)
    rviz_ld = generate_moveit_rviz_launch(moveit_config)

    # ── Timing ──────────────────────────────────────────────────────────────
    delayed_spawn = TimerAction(period=10.0, actions=[spawn_entity])

    # ── Model path ──────────────────────────────────────────────────────────
    parent = os.path.abspath(os.path.join(pkg_share, '..'))
    prev = os.environ.get('GAZEBO_MODEL_PATH', '')
    model_path = f'{parent}{os.pathsep}{prev}' if prev else parent

    ld = LaunchDescription(args)
    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path))
    ld.add_action(gazebo)
    ld.add_action(robot_state_pub)
    ld.add_action(delayed_spawn)
    for entity in mg_ld.entities:
        ld.add_entity(entity)
    for entity in rviz_ld.entities:
        ld.add_entity(entity)
    return ld

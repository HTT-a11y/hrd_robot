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

    # inject use_sim_time into robot_description so move_group (via to_dict()) and
    # robot_state_publisher both receive it
    moveit_config.robot_description['use_sim_time'] = True

    # ── MoveGroup + RViz ────────────────────────────────────────────────────
    mg_ld = generate_move_group_launch(moveit_config)

    args.append(
        DeclareLaunchArgument(
            'rviz_config',
            default_value=str(moveit_config.package_path / 'config/moveit.rviz'),
        )
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── Target cylinder (Gazebo) ────────────────────────────────────────────
    cylinder_sdf = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="target_cylinder">
    <static>true</static>
    <link name="cylinder_link">
      <collision name="collision">
        <geometry><cylinder><radius>0.02</radius><length>0.25</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>0.02</radius><length>0.25</length></cylinder></geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

    fd_cyl, sdf_tmp = tempfile.mkstemp(
        prefix='target_cylinder_', suffix='.sdf', text=True,
    )
    with os.fdopen(fd_cyl, 'w', encoding='utf-8') as f:
        f.write(cylinder_sdf)

    spawn_cylinder = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'target_cylinder',
            '-file', sdf_tmp,
            '-x', '0.05', '-y', '-0.70', '-z', '0.8',
            '-timeout', '60.0',
        ],
    )

    # ── Target cylinder (MoveIt / RViz) ────────────────────────────────────
    publish_collision_object = Node(
        package='hrd_moveit_config',
        executable='spawn_target_object.py',
        output='screen',
    )

    # ── Controllers ──────────────────────────────────────────────────────────
    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hrd_moveit_config'),
                'launch', 'spawn_controllers.launch.py',
            )
        )
    )

    # ── Timing ──────────────────────────────────────────────────────────────
    delayed_spawn = TimerAction(period=10.0, actions=[spawn_entity])
    delayed_spawn_cylinder = TimerAction(
        period=12.0, actions=[spawn_cylinder],
    )
    delayed_spawn_controllers = TimerAction(
        period=15.0, actions=[spawn_controllers_launch],
    )
    delayed_target_rviz = TimerAction(
        period=20.0, actions=[publish_collision_object],
    )

    # ── Model path ──────────────────────────────────────────────────────────
    parent = os.path.abspath(os.path.join(pkg_share, '..'))
    prev = os.environ.get('GAZEBO_MODEL_PATH', '')
    model_path = f'{parent}{os.pathsep}{prev}' if prev else parent

    ld = LaunchDescription(args)
    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path))
    ld.add_action(gazebo)
    ld.add_action(robot_state_pub)
    ld.add_action(delayed_spawn)
    ld.add_action(delayed_spawn_cylinder)
    ld.add_action(delayed_spawn_controllers)
    ld.add_action(delayed_target_rviz)
    for entity in mg_ld.entities:
        ld.add_entity(entity)
    ld.add_action(rviz_node)
    return ld

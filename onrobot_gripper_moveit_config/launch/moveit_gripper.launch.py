"""Run MoveIt move_group against the standard OnRobot gripper action."""

from pathlib import Path
import subprocess
import yaml

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


_MODELS = {
    '2fg7': (
        'onrobot_2fg7', 'onrobot_2fg7',
        'realmesh_onrobot_2fg7.urdf.xacro'),
    '2fg14': (
        'onrobot_2fg14', 'onrobot_2fg14',
        'realmesh_onrobot_2fg14.urdf.xacro'),
    'rg2': ('onrobot_rg2', 'onrobot_rg2', 'realmesh_onrobot_rg2.urdf.xacro'),
    'rg6': ('onrobot_rg6', 'onrobot_rg6', 'realmesh_onrobot_rg6.urdf.xacro'),
}


def _launch_model(context):
    model = LaunchConfiguration('model').perform(context)
    package, robot_name, xacro_name = _MODELS[model]
    backend = LaunchConfiguration('backend').perform(context)
    host = LaunchConfiguration('host').perform(context)
    port = LaunchConfiguration('port').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    physical_joint = 'finger_stroke' if model.startswith('2fg') else 'finger_joint'
    profile_path = Path(get_package_share_directory('onrobot_gripper_description')) / 'config/planning_profiles' / f'{model}.yaml'
    resolver = Path(get_package_prefix('onrobot_gripper_description')) / 'bin/resolve_gripper_profile'
    resolved = subprocess.run([str(resolver), '--profile', str(profile_path)],
                              check=True, capture_output=True, text=True, timeout=5)
    profile = yaml.safe_load(resolved.stdout)
    safe_joint = profile['safe_q_domain']

    package_share = Path(get_package_share_directory(package))
    xacro_path = package_share / 'urdf' / xacro_name
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name,
            package_name='onrobot_gripper_moveit_config')
        .robot_description(
            file_path=xacro_path,
            mappings={'backend': backend, 'host': host, 'port': port})
        .robot_description_semantic(
            file_path='config/onrobot_gripper.srdf.xacro',
            mappings={'robot_name': robot_name, 'model': model, 'physical_joint': physical_joint,
                      'closed_position': str(safe_joint['minimum'])})
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(
            default_planning_pipeline='ompl', pipelines=['ompl'])
        .trajectory_execution(
            file_path='config/moveit_controllers.yaml',
            moveit_manage_controllers=False)
        .to_moveit_configs()
    )
    # These are physical-joint planning limits, not device task-aperture limits.
    # Excluding overlap and unreachable targets never rescales the linkage.
    moveit_config.joint_limits = {'robot_description_planning': {'joint_limits': {
        physical_joint: {
            'has_position_limits': True,
            'min_position': safe_joint['minimum'],
            'max_position': safe_joint['maximum'],
            'has_velocity_limits': True,
            'max_velocity': 0.05 if model.startswith('2fg') else 0.5,
            'has_acceleration_limits': True,
            'max_acceleration': 0.5 if model.startswith('2fg') else 1.0,
        }}}}
    controller = moveit_config.trajectory_execution[
        'moveit_simple_controller_manager']['physical_gripper_controller']
    controller['joints'] = [physical_joint]
    controller['command_joint'] = physical_joint

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('onrobot_gripper_bringup'),
            '/launch/gripper.launch.py',
        ]),
        launch_arguments={
            'model': model,
            'backend': backend,
            'host': host,
            'port': port,
            'namespace': namespace,
            'start_realtime_controller': 'false',
        }.items())

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        namespace=namespace,
        parameters=[
            moveit_config.to_dict(),
            {
                'publish_robot_description_semantic': True,
                'allow_trajectory_execution': True,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
                'monitor_dynamics': False,
            },
        ],
    )
    adapter = Node(
        package='onrobot_gripper_moveit_config', executable='physical_gripper_adapter',
        namespace=namespace, output='screen', parameters=[{
            'profile': str(profile_path), 'profile_hash': profile['profile_hash'],
        }])
    return [bringup, adapter, move_group]


def generate_launch_description():
    """Declare the standalone MoveIt gripper example interface."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value='2fg7', choices=list(_MODELS)),
        DeclareLaunchArgument(
            'backend', default_value='fake', choices=['real', 'fake']),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='502'),
        DeclareLaunchArgument('namespace', default_value=''),
        OpaqueFunction(function=_launch_model),
    ])

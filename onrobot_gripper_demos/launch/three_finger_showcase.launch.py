"""Shared native-diameter control showcase for 3FG15 and 3FG25."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _fingertip_position(context):
    position = LaunchConfiguration('fingertip_position').perform(context)
    if position == 'auto':
        model = LaunchConfiguration('model').perform(context)
        # Same reference mountings as the unified showcase. The hardware
        # still verifies this against the device before accepting commands.
        return {'3fg15': '2', '3fg25': '3'}[model]
    return position


def _nodes(context):
    model = LaunchConfiguration('model').perform(context)
    package = f'onrobot_{model}'
    share = FindPackageShare(package)
    description = ParameterValue(Command([
        'xacro ',
        PathJoinSubstitution([
            share, 'urdf', f'realmesh_onrobot_{model}.urdf.xacro']),
        ' host:=', LaunchConfiguration('host'),
        ' port:=', LaunchConfiguration('port'),
        ' fingertip_position:=', _fingertip_position(context),
    ]), value_type=str)
    controller_config = PathJoinSubstitution([
        FindPackageShare('onrobot_gripper_demos'), 'launch',
        'three_finger_controllers.yaml'])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('onrobot_3fg25'), 'rviz',
        '3fg25_control_showcase.rviz'])

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen', parameters=[{'robot_description': description,
                                         'frame_prefix': LaunchConfiguration('frame_prefix')}],
            remappings=[('/robot_description', 'robot_description')]),
        Node(
            package='controller_manager', executable='ros2_control_node',
            output='screen', parameters=[{'robot_description': description},
                                         controller_config],
            remappings=[('/robot_description', 'robot_description')]),
        Node(
            package='controller_manager', executable='spawner',
            output='screen',
            arguments=['joint_state_broadcaster', '--controller-manager',
                       'controller_manager']),
        Node(
            package='controller_manager', executable='spawner',
            output='screen',
            arguments=[
                'gripper_state_broadcaster', '--controller-manager',
                'controller_manager', '--controller-ros-args',
                '--ros-args -p model:=' + model]),
        Node(
            package='controller_manager', executable='spawner',
            output='screen',
            arguments=['three_finger_limit_broadcaster',
                       '--controller-manager',
                       'controller_manager']),
        Node(
            package='controller_manager', executable='spawner',
            output='screen',
            arguments=['diameter_controller', '--controller-manager',
                       'controller_manager']),
        Node(
            package='onrobot_gripper_hardware',
            executable='joint_state_effort_filter', output='screen'),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            output='screen', arguments=[
                '--x', '0', '--y', '0', '--z', '0', '--frame-id', 'world',
                '--child-frame-id',
                [LaunchConfiguration('frame_prefix'), 'base_link']]),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', rviz_config, '-f',
                       [LaunchConfiguration('frame_prefix'), 'base_link']],
            condition=IfCondition(LaunchConfiguration('start_rviz'))),
    ]
    return [GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')), *nodes])]


def generate_launch_description():
    """Declare the internal three-finger showcase composition."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value='3fg25', choices=['3fg15', '3fg25']),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='502'),
        DeclareLaunchArgument(
            'fingertip_position', default_value='auto',
            choices=['auto', '1', '2', '3'],
            description='Reference mounting: 2 for 3FG15, 3 for 3FG25. '
                        'Set 1, 2 or 3 to match a different physical mounting.'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        OpaqueFunction(function=_nodes),
    ])

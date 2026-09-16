"""One RViz showcase entry point for the six available gripper models."""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_PARALLEL_MODELS = {'2fg7', '2fg14', 'rg2', 'rg6'}
_THREE_FINGER_MODELS = {'3fg15', '3fg25'}
_REFERENCE_FINGERTIP_POSITIONS = {'3fg15': '2', '3fg25': '3'}
_RVIZ_CONFIGS = {
    ('2fg7', 'conventional'): '2fg7_ros2_control_showcase.rviz',
    ('2fg7', 'realtime'): '2fg7_realtime_control_showcase.rviz',
    ('2fg14', 'conventional'): '2fg14_ros2_control_showcase.rviz',
    ('2fg14', 'realtime'): '2fg14_realtime_control_showcase.rviz',
    ('rg2', 'conventional'): 'rg2_ros2_control_showcase.rviz',
    ('rg2', 'realtime'): 'rg_realtime_control_panel.rviz',
    ('rg6', 'conventional'): 'rg6_ros2_control_showcase.rviz',
    ('rg6', 'realtime'): 'rg_realtime_control_panel.rviz',
}
_RVIZ_PARAMETERS = {
    '2fg7': {
        'minimum_effort_n': 20.0,
        'maximum_effort_n': 140.0,
        'default_effort_n': 20.0,
    },
    '2fg14': {
        'minimum_effort_n': 40.0,
        'maximum_effort_n': 280.0,
        'default_effort_n': 40.0,
    },
    'rg2': {
        'maximum_effort_n': 40.0,
        'default_effort_n': 10.0,
        'realtime_coordinate_profile': 'rg',
        'realtime_maximum_angular_velocity_rad_s': 1.175,
        'realtime_maximum_position_force_n': 40.0,
        'realtime_default_position_force_n': 10.0,
    },
    'rg6': {
        'maximum_effort_n': 120.0,
        'default_effort_n': 20.0,
        'realtime_coordinate_profile': 'rg',
        'realtime_maximum_angular_velocity_rad_s': 0.849,
        'realtime_maximum_position_force_n': 120.0,
        'realtime_default_position_force_n': 20.0,
    },
}


def _include_selected_model(context):
    model = LaunchConfiguration('model').perform(context)
    control = LaunchConfiguration('control').perform(context)
    backend = LaunchConfiguration('backend').perform(context)
    port = LaunchConfiguration('port')

    if model in _PARALLEL_MODELS:
        model_package = f'onrobot_{model}'
        rviz_config = '/'.join([
            get_package_share_directory(model_package),
            'rviz',
            _RVIZ_CONFIGS[(model, control)],
        ])
        source = PythonLaunchDescriptionSource([
            get_package_share_directory('onrobot_gripper_bringup'),
            '/launch/gripper.launch.py',
        ])
        arguments = {
            'model': model,
            'backend': LaunchConfiguration('backend'),
            'transport': LaunchConfiguration('transport'),
            'host': LaunchConfiguration('host'),
            'port': port,
            'serial_device': LaunchConfiguration('serial_device'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'rtu_parity': LaunchConfiguration('rtu_parity'),
            'slave_id': LaunchConfiguration('slave_id'),
            'connect_timeout_ms': LaunchConfiguration('connect_timeout_ms'),
            'read_timeout_ms': LaunchConfiguration('read_timeout_ms'),
            'write_timeout_ms': LaunchConfiguration('write_timeout_ms'),
            'namespace': LaunchConfiguration('namespace'),
            'frame_prefix': LaunchConfiguration('frame_prefix'),
            'fake_motion_speed_m_s': LaunchConfiguration(
                'fake_motion_speed_m_s'),
            'fake_stall': LaunchConfiguration('fake_stall'),
            'isaac_joint_commands_topic': LaunchConfiguration(
                'isaac_joint_commands_topic'),
            'isaac_joint_states_topic': LaunchConfiguration(
                'isaac_joint_states_topic'),
            'isaac_state_timeout_ms': LaunchConfiguration(
                'isaac_state_timeout_ms'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'realtime_update_rate_hz': LaunchConfiguration(
                'realtime_update_rate_hz'),
            'controller_manager_update_rate_hz': LaunchConfiguration(
                'controller_manager_update_rate_hz'),
            'state_publish_rate_hz': LaunchConfiguration(
                'state_publish_rate_hz'),
            'supply_power_w': LaunchConfiguration('supply_power_w'),
            'conventional_speed_percent': LaunchConfiguration(
                'conventional_speed_percent'),
            'visual_detail': LaunchConfiguration('visual_detail'),
            'gripper_stall_timeout_s': LaunchConfiguration(
                'gripper_stall_timeout_s'),
            'start_realtime_controller': (
                'true' if control == 'realtime' else 'false'),
        }
        rviz_parameters = dict(_RVIZ_PARAMETERS.get(model, {}))
        rviz_parameters['use_sim_time'] = backend == 'isaac'
        return [
            IncludeLaunchDescription(
                source, launch_arguments=arguments.items()),
            Node(
                package='rviz2', executable='rviz2', name='rviz2',
                namespace=LaunchConfiguration('namespace'), output='screen',
                arguments=['-d', rviz_config],
                parameters=[rviz_parameters],
                condition=IfCondition(LaunchConfiguration('start_rviz'))),
        ]

    if control == 'realtime':
        raise RuntimeError(
            f"model '{model}' does not implement realtime control; "
            'use control:=conventional')
    if backend != 'real':
        raise RuntimeError(
            f"model '{model}' currently supports backend:=real only")

    configured_fingertip_position = LaunchConfiguration(
        'fingertip_position').perform(context)
    fingertip_position = (
        _REFERENCE_FINGERTIP_POSITIONS[model]
        if configured_fingertip_position == 'auto'
        else configured_fingertip_position)
    source = PythonLaunchDescriptionSource([
        get_package_share_directory('onrobot_gripper_demos'),
        '/launch/three_finger_showcase.launch.py',
    ])
    arguments = {
        'model': model,
        'host': LaunchConfiguration('host'),
        'port': port,
        'fingertip_position': fingertip_position,
        'start_rviz': LaunchConfiguration('start_rviz'),
        'namespace': LaunchConfiguration('namespace'),
        'frame_prefix': LaunchConfiguration('frame_prefix'),
    }
    return [IncludeLaunchDescription(
        source, launch_arguments=arguments.items())]


def generate_launch_description():
    """Declare the public six-model showcase interface."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value='2fg7',
            choices=['2fg7', '2fg14', 'rg2', 'rg6', '3fg15', '3fg25']),
        DeclareLaunchArgument(
            'control', default_value='conventional',
            choices=['conventional', 'realtime']),
        DeclareLaunchArgument(
            'backend', default_value='real', choices=['real', 'fake', 'isaac']),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument(
            'port', default_value='502',
            description=(
                'Modbus TCP port reported by the gripper configuration')),
        DeclareLaunchArgument(
            'transport', default_value='tcp', choices=['tcp', 'rtu'],
            description='Modbus transport used by real hardware'),
        DeclareLaunchArgument(
            'serial_device', default_value='/dev/ttyUSB0',
            description='Serial device used when transport:=rtu'),
        DeclareLaunchArgument(
            'baud_rate', default_value='1000000',
            choices=['115200', '1000000'],
            description='RTU baud rate used when transport:=rtu'),
        DeclareLaunchArgument(
            'rtu_parity', default_value='even', choices=['even'],
            description='RTU parity; OnRobot framing is 8E1'),
        DeclareLaunchArgument(
            'slave_id', default_value='65', choices=['65', '66', '67'],
            description='Modbus device address for the mounting position'),
        DeclareLaunchArgument(
            'connect_timeout_ms', default_value='1000',
            description='Modbus connection timeout in milliseconds'),
        DeclareLaunchArgument(
            'read_timeout_ms', default_value='1000',
            description='Modbus read response timeout in milliseconds'),
        DeclareLaunchArgument(
            'write_timeout_ms', default_value='1000',
            description='Modbus write response timeout in milliseconds'),
        DeclareLaunchArgument(
            'fake_motion_speed_m_s', default_value='0.0',
            description='Fake-only deterministic conventional motion rate'),
        DeclareLaunchArgument(
            'fake_stall', default_value='0', choices=['0', '1'],
            description='Fake-only conventional stall injection'),
        DeclareLaunchArgument(
            'isaac_joint_commands_topic', default_value='isaac_joint_commands',
            description='Isaac physical-joint command topic'),
        DeclareLaunchArgument(
            'isaac_joint_states_topic', default_value='isaac_joint_states',
            description='Isaac physical-joint feedback topic'),
        DeclareLaunchArgument(
            'isaac_state_timeout_ms', default_value='250',
            description='Maximum age of Isaac joint feedback'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock for non-Isaac backends'),
        DeclareLaunchArgument(
            'realtime_update_rate_hz', default_value='auto',
            description='Realtime hardware exchange rate'),
        DeclareLaunchArgument(
            'controller_manager_update_rate_hz', default_value='100',
            description='ros2_control controller-manager update rate in hertz'),
        DeclareLaunchArgument(
            'state_publish_rate_hz', default_value='100',
            description=(
                'Typed and realtime state publication rate in hertz')),
        DeclareLaunchArgument(
            'gripper_stall_timeout_s', default_value='2.0',
            description='Time below the stall threshold before an action stalls'),
        DeclareLaunchArgument(
            'supply_power_w', default_value='48',
            description='2FG supply-power budget reapplied after connection'),
        DeclareLaunchArgument(
            'conventional_speed_percent', default_value='50',
            description=(
                '2FG conventional motion speed as a device percentage; '
                'valid values are 1..100 and ignored by RG')),
        DeclareLaunchArgument(
            'visual_detail', default_value='detailed',
            choices=['detailed', 'low'],
            description=(
                '2FG7 body visual detail; low preserves all collision meshes '
                'and kinematic geometry')),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        DeclareLaunchArgument(
            'fingertip_position', default_value='auto',
            choices=['auto', '1', '2', '3'],
            description=(
                'Physical 3FG fingertip position; auto selects the model '
                'reference configuration')),
        OpaqueFunction(function=_include_selected_model),
    ])

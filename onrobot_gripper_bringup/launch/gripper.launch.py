"""Model-neutral entry point for supported OnRobot parallel grippers."""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import PushRosNamespace


_MODEL_PACKAGES = {
    '2fg7': 'onrobot_2fg7',
    '2fg14': 'onrobot_2fg14',
    'rg2': 'onrobot_rg2',
    'rg6': 'onrobot_rg6',
}
_MODEL_REALTIME_UPDATE_RATES_HZ = {
    # Conservative common rate. Firmware transport ceilings are intentionally
    # not used as launch defaults: the host, network and device must be
    # measured together before selecting a higher explicit value.
    '2fg7': '50',
    '2fg14': '50',
    'rg2': '50',
    'rg6': '50',
}
_ISAAC_MODELS = tuple(_MODEL_PACKAGES)


def _resolved_controller_manager(namespace, requested):
    """Resolve the default manager without hiding explicit overrides."""
    requested = requested.strip()
    if requested:
        return requested
    namespace = namespace.strip('/')
    return (
        f'/{namespace}/controller_manager'
        if namespace else '/controller_manager')


def _resolved_realtime_update_rate(model, requested_rate):
    """Replace the model-neutral auto value before including model launch."""
    if requested_rate == 'auto':
        return _MODEL_REALTIME_UPDATE_RATES_HZ[model]
    return requested_rate


def _include_model(context):
    model = LaunchConfiguration('model').perform(context)
    backend = LaunchConfiguration('backend').perform(context)
    if backend == 'isaac' and model not in _ISAAC_MODELS:
        raise RuntimeError(
            'backend:=isaac requires model:=2fg7, model:=2fg14, '
            'model:=rg2, or model:=rg6')
    package = _MODEL_PACKAGES[model]
    namespace = LaunchConfiguration('namespace').perform(context)
    requested_manager = LaunchConfiguration('controller_manager').perform(
        context)
    manager = _resolved_controller_manager(namespace, requested_manager)
    arguments = {
        'backend': LaunchConfiguration('backend'),
        'transport': LaunchConfiguration('transport'),
        'host': LaunchConfiguration('host'),
        'port': LaunchConfiguration('port'),
        'serial_device': LaunchConfiguration('serial_device'),
        'baud_rate': LaunchConfiguration('baud_rate'),
        'rtu_parity': LaunchConfiguration('rtu_parity'),
        'slave_id': LaunchConfiguration('slave_id'),
        'connect_timeout_ms': LaunchConfiguration('connect_timeout_ms'),
        'read_timeout_ms': LaunchConfiguration('read_timeout_ms'),
        'write_timeout_ms': LaunchConfiguration('write_timeout_ms'),
        'frame_prefix': LaunchConfiguration('frame_prefix'),
        'namespace': LaunchConfiguration('namespace'),
        'controller_manager': manager,
        'publish_base_frame': LaunchConfiguration('publish_base_frame'),
        'base_parent_frame': LaunchConfiguration('base_parent_frame'),
        'use_sim_time': (
            'true' if backend == 'isaac'
            else LaunchConfiguration('use_sim_time')),
        'controller_manager_update_rate_hz': LaunchConfiguration(
            'controller_manager_update_rate_hz'),
        'state_publish_rate_hz': LaunchConfiguration(
            'state_publish_rate_hz'),
    }
    if model == '2fg7':
        arguments['fake_motion_speed_m_s'] = LaunchConfiguration(
            'fake_motion_speed_m_s')
        arguments['fake_stall'] = LaunchConfiguration('fake_stall')
        arguments['visual_detail'] = LaunchConfiguration('visual_detail')
    if model in ('2fg7', '2fg14'):
        arguments['supply_power_w'] = LaunchConfiguration('supply_power_w')
        arguments['conventional_speed_percent'] = LaunchConfiguration(
            'conventional_speed_percent')
    if model in _MODEL_REALTIME_UPDATE_RATES_HZ:
        arguments['realtime_update_rate_hz'] = (
            _resolved_realtime_update_rate(
                model,
                LaunchConfiguration('realtime_update_rate_hz').perform(
                    context)))
    if model in _ISAAC_MODELS:
        arguments['isaac_joint_commands_topic'] = LaunchConfiguration(
            'isaac_joint_commands_topic')
        arguments['isaac_joint_states_topic'] = LaunchConfiguration(
            'isaac_joint_states_topic')
        arguments['isaac_state_timeout_ms'] = LaunchConfiguration(
            'isaac_state_timeout_ms')
    arguments['start_realtime_controller'] = LaunchConfiguration(
        'start_realtime_controller')
    arguments['gripper_stall_timeout_s'] = LaunchConfiguration(
        'gripper_stall_timeout_s')
    source = PythonLaunchDescriptionSource(
        [get_package_share_directory(package), '/launch/control.launch.py'])
    return [GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        IncludeLaunchDescription(source, launch_arguments=arguments.items()),
    ])]


def generate_launch_description():
    """Select one model without requiring package-specific launch files."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value='2fg7', choices=list(_MODEL_PACKAGES)),
        DeclareLaunchArgument(
            'backend', default_value='real',
            choices=['real', 'fake', 'isaac']),
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
            'namespace', default_value='',
            description='ROS namespace for this gripper instance'),
        DeclareLaunchArgument(
            'controller_manager', default_value='',
            description=(
                'Controller-manager service name passed to spawners; empty '
                'selects the namespace-specific manager')),
        DeclareLaunchArgument(
            'frame_prefix', default_value='',
            description='Prefix applied to every published TF frame'),
        DeclareLaunchArgument(
            'publish_base_frame', default_value='true',
            choices=['true', 'false'],
            description=(
                'Publish the fixed parent transform used by a standalone or '
                'robot-mounted gripper description')),
        DeclareLaunchArgument(
            'base_parent_frame', default_value='world',
            description=(
                'Parent TF frame for the gripper base when '
                'publish_base_frame is true')),
        DeclareLaunchArgument(
            'fake_motion_speed_m_s', default_value='0.0',
            description='Fake-only deterministic conventional motion rate'),
        DeclareLaunchArgument(
            'fake_stall', default_value='0', choices=['0', '1'],
            description='Fake-only conventional stall injection'),
        DeclareLaunchArgument(
            'isaac_joint_commands_topic',
            default_value='isaac_joint_commands',
            description='Isaac physical-joint command topic'),
        DeclareLaunchArgument(
            'isaac_joint_states_topic', default_value='isaac_joint_states',
            description='Isaac physical-joint feedback topic'),
        DeclareLaunchArgument(
            'isaac_state_timeout_ms', default_value='250',
            description='Maximum age of Isaac joint feedback'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description=(
                'Use simulation clock; automatically enabled for Isaac')),
        DeclareLaunchArgument(
            'start_realtime_controller', default_value='false',
            description='Activate the model-approved realtime controller'),
        DeclareLaunchArgument(
            'gripper_stall_timeout_s', default_value='2.0',
            description=(
                'Time below the stall-velocity threshold before a standard '
                'action reports a stall')),
        DeclareLaunchArgument(
            'realtime_update_rate_hz', default_value='auto',
            description=(
                'Realtime hardware exchange rate; auto uses the conservative '
                '50 Hz default. Set a higher explicit value only after '
                'measuring the target host, network and gripper.')),
        DeclareLaunchArgument(
            'controller_manager_update_rate_hz', default_value='100',
            description='ros2_control controller-manager update rate in hertz'),
        DeclareLaunchArgument(
            'state_publish_rate_hz', default_value='100',
            description=(
                'Typed and realtime state publication rate in hertz')),
        DeclareLaunchArgument(
            'supply_power_w', default_value='48',
            description=(
                '2FG supply-power budget reapplied after connection; zero '
                'preserves the current device setting')),
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
        OpaqueFunction(function=_include_model),
    ])

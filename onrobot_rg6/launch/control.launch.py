"""Bring up an RG6 with real, fake, or Isaac ros2_control hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.descriptions import Parameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _ros_double_literal(value):
    """Keep numeric controller overrides compatible with double parameters."""
    value = value.strip()
    if '.' not in value and 'e' not in value.lower():
        return value + '.0'
    return value


def _chain_on_success(next_action, controller_name):
    """Continue the ordered spawner chain only after a clean exit."""
    def on_exit(event, _context):
        if event.returncode == 0:
            return [next_action]
        return [EmitEvent(event=Shutdown(reason=(
            f'controller spawner {controller_name} exited with '
            f'code {event.returncode}')))]
    return on_exit


def generate_launch_description():
    """Bring up a real RG6 through the standard ros2_control stack."""
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    transport = LaunchConfiguration('transport')
    serial_device = LaunchConfiguration('serial_device')
    baud_rate = LaunchConfiguration('baud_rate')
    rtu_parity = LaunchConfiguration('rtu_parity')
    slave_id = LaunchConfiguration('slave_id')
    connect_timeout = LaunchConfiguration('connect_timeout_ms')
    read_timeout = LaunchConfiguration('read_timeout_ms')
    write_timeout = LaunchConfiguration('write_timeout_ms')
    backend = LaunchConfiguration('backend')
    frame_prefix = LaunchConfiguration('frame_prefix')
    start_realtime_controller = LaunchConfiguration(
        'start_realtime_controller')
    isaac_commands = LaunchConfiguration('isaac_joint_commands_topic')
    isaac_states = LaunchConfiguration('isaac_joint_states_topic')
    isaac_state_timeout = LaunchConfiguration('isaac_state_timeout_ms')
    realtime_update_rate = LaunchConfiguration('realtime_update_rate_hz')
    controller_manager_update_rate = LaunchConfiguration(
        'controller_manager_update_rate_hz')
    state_publish_rate = LaunchConfiguration('state_publish_rate_hz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    controller_manager = LaunchConfiguration('controller_manager')
    gripper_stall_timeout = LaunchConfiguration('gripper_stall_timeout_s')
    share = FindPackageShare('onrobot_rg6')
    description = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([
            share, 'urdf', 'realmesh_onrobot_rg6.urdf.xacro'
        ]),
        ' host:=', host,
        ' port:=', port,
        ' transport:=', transport,
        ' serial_device:=', serial_device,
        ' baud_rate:=', baud_rate,
        ' rtu_parity:=', rtu_parity,
        ' slave_id:=', slave_id,
        ' connect_timeout_ms:=', connect_timeout,
        ' read_timeout_ms:=', read_timeout,
        ' write_timeout_ms:=', write_timeout,
        ' backend:=', backend,
        ' safe_width_min_mm:=0.0',
        ' safe_width_max_mm:=160.0',
        ' default_force_n:=20.0',
        ' isaac_joint_commands_topic:=', isaac_commands,
        ' isaac_joint_states_topic:=', isaac_states,
        ' isaac_state_timeout_ms:=', isaac_state_timeout,
        ' realtime_update_rate_hz:=', realtime_update_rate,
    ]), value_type=str)
    controllers = PathJoinSubstitution([
        FindPackageShare('onrobot_gripper_controllers'), 'config', 'rg.yaml'
    ])
    state_parameters = PathJoinSubstitution([
        share, 'config', 'gripper_state_broadcaster.yaml'])

    def build_controller_spawner_chain(context):
        """Resolve the namespaced manager before registering exit handlers."""
        manager = controller_manager.perform(context)
        realtime = start_realtime_controller.perform(context) == 'true'
        state_rate = _ros_double_literal(state_publish_rate.perform(context))

        def spawner(name, inactive=False):
            arguments = [name]
            if inactive:
                arguments.append('--inactive')
            if name == 'gripper_controller':
                arguments.extend([
                    '--controller-ros-args',
                    '--ros-args -p stall_timeout:=' +
                    gripper_stall_timeout.perform(context),
                ])
            if name == 'gripper_state_broadcaster':
                arguments.extend([
                    '--controller-ros-args',
                    '--ros-args -p publish_rate:=' +
                    state_rate,
                ])
            if name == 'realtime_controller':
                arguments.extend([
                    '--controller-ros-args',
                    '--ros-args -p state_publish_rate_hz:=' +
                    state_rate,
                ])
            arguments.extend(['--controller-manager', manager])
            return Node(
                package='controller_manager', executable='spawner',
                arguments=arguments, output='screen')

        joint_state = spawner('joint_state_broadcaster')
        limits = spawner('parallel_gripper_limit_broadcaster')
        state = spawner('gripper_state_broadcaster')
        recovery = spawner('recovery_controller')
        gripper = spawner('gripper_controller', inactive=realtime)
        realtime_controller = spawner(
            'realtime_controller', inactive=not realtime)
        return [
            joint_state,
            RegisterEventHandler(OnProcessExit(
                target_action=joint_state,
                on_exit=_chain_on_success(
                    limits, 'joint_state_broadcaster'))),
            RegisterEventHandler(OnProcessExit(
                target_action=limits,
                on_exit=_chain_on_success(
                    state, 'parallel_gripper_limit_broadcaster'))),
            RegisterEventHandler(OnProcessExit(
                target_action=state,
                on_exit=_chain_on_success(
                    recovery, 'gripper_state_broadcaster'))),
            RegisterEventHandler(OnProcessExit(
                target_action=recovery,
                on_exit=_chain_on_success(
                    gripper, 'recovery_controller'))),
            RegisterEventHandler(OnProcessExit(
                target_action=gripper,
                on_exit=_chain_on_success(
                    realtime_controller, 'gripper_controller'))),
        ]
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='502'),
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
        DeclareLaunchArgument('backend', default_value='real',
                              choices=['real', 'fake', 'isaac']),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        DeclareLaunchArgument(
            'start_realtime_controller', default_value='false'),
        DeclareLaunchArgument(
            'gripper_stall_timeout_s', default_value='2.0'),
        DeclareLaunchArgument(
            'isaac_joint_commands_topic',
            default_value='isaac_joint_commands',
            description='JointState commands sent to the Isaac articulation'),
        DeclareLaunchArgument(
            'isaac_joint_states_topic',
            default_value='isaac_joint_states',
            description='JointState feedback received from Isaac Sim'),
        DeclareLaunchArgument(
            'isaac_state_timeout_ms', default_value='250',
            description='Maximum age of Isaac joint feedback'),
        DeclareLaunchArgument(
            'realtime_update_rate_hz', default_value='50',
            description='Realtime Modbus exchange rate in Hz'),
        DeclareLaunchArgument(
            'controller_manager_update_rate_hz', default_value='100',
            description=(
                'ros2_control controller-manager update rate in hertz')),
        DeclareLaunchArgument(
            'state_publish_rate_hz', default_value='100',
            description=(
                'Typed and realtime state publication rate in hertz')),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the clock published by a simulation backend'),
        DeclareLaunchArgument(
            'controller_manager', default_value=PathJoinSubstitution([
                '/', LaunchConfiguration('namespace'), 'controller_manager']),
            description='Controller-manager service name for spawners'),
        DeclareLaunchArgument('namespace', default_value=''),
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             output='screen', parameters=[{
                 'robot_description': description,
                 'frame_prefix': frame_prefix,
                 'use_sim_time': use_sim_time,
             }]),
        Node(package='controller_manager', executable='ros2_control_node',
             name='controller_manager',
             output='screen', parameters=[
                 {'robot_description': description}, controllers,
                 state_parameters,
                 {
                     'hardware_components_initial_state.'
                     'shutdown_on_initial_state_failure': False,
                     'use_sim_time': use_sim_time,
                 },
                 Parameter(
                     'update_rate', controller_manager_update_rate,
                     value_type=int),
             ]),
        OpaqueFunction(function=build_controller_spawner_chain),
        Node(package='onrobot_gripper_hardware',
             executable='joint_state_effort_filter', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             output='screen',
             arguments=['--x', '0', '--y', '0', '--z', '0',
                        '--frame-id', 'world',
                        '--child-frame-id', [frame_prefix, 'base_link']],
             parameters=[{'use_sim_time': use_sim_time}]),
    ])

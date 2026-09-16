"""Launch the 2FG7 ros2_control stack with real, fake, or Isaac hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
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


def _validate_conventional_speed(context):
    """Reject an invalid 2FG conventional speed before starting the stack."""
    value = LaunchConfiguration('conventional_speed_percent').perform(context).strip()
    backend = LaunchConfiguration('backend').perform(context)
    try:
        speed = int(value)
    except ValueError as error:
        raise RuntimeError(
            'conventional_speed_percent must be an integer from 1 to 100') from error
    if str(speed) != value or speed < 1 or speed > 100:
        raise RuntimeError('conventional_speed_percent must be an integer from 1 to 100')
    if backend == 'isaac' and speed != 50:
        raise RuntimeError(
            'conventional_speed_percent is unavailable for backend:=isaac; '
            'Isaac has no verified percentage-to-motion mapping')
    return []


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
    """Build the standalone 2FG7 control launch description."""
    description_xacro = LaunchConfiguration('description_xacro')
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
    fake_motion_speed = LaunchConfiguration('fake_motion_speed_m_s')
    fake_stall = LaunchConfiguration('fake_stall')
    isaac_commands = LaunchConfiguration('isaac_joint_commands_topic')
    isaac_states = LaunchConfiguration('isaac_joint_states_topic')
    isaac_state_timeout = LaunchConfiguration('isaac_state_timeout_ms')
    realtime_update_rate = LaunchConfiguration('realtime_update_rate_hz')
    controller_manager_update_rate = LaunchConfiguration(
        'controller_manager_update_rate_hz')
    state_publish_rate = LaunchConfiguration('state_publish_rate_hz')
    supply_power = LaunchConfiguration('supply_power_w')
    conventional_speed = LaunchConfiguration('conventional_speed_percent')
    visual_detail = LaunchConfiguration('visual_detail')
    use_sim_time = LaunchConfiguration('use_sim_time')
    xacro_path = PathJoinSubstitution([
        FindPackageShare('onrobot_2fg7'),
        'urdf',
        description_xacro,
    ])
    controllers_path = PathJoinSubstitution([
        FindPackageShare('onrobot_2fg7'),
        'config',
        'ros2_control_controllers.yaml',
    ])
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_path,
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
            ' fake_motion_speed_m_s:=', fake_motion_speed,
            ' fake_stall:=', fake_stall,
            ' isaac_joint_commands_topic:=', isaac_commands,
            ' isaac_joint_states_topic:=', isaac_states,
            ' isaac_state_timeout_ms:=', isaac_state_timeout,
            ' realtime_update_rate_hz:=', realtime_update_rate,
            ' supply_power_w:=', supply_power,
            ' conventional_speed_percent:=', conventional_speed,
            ' visual_detail:=', visual_detail,
        ]), value_type=str
    )
    publish_base_frame = LaunchConfiguration('publish_base_frame')
    base_parent_frame = LaunchConfiguration('base_parent_frame')
    start_realtime_controller = LaunchConfiguration(
        'start_realtime_controller')
    controller_manager = LaunchConfiguration('controller_manager')
    gripper_stall_timeout = LaunchConfiguration('gripper_stall_timeout_s')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'frame_prefix': frame_prefix,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[
            {'robot_description': robot_description},
            controllers_path,
            # A disconnected physical gripper should leave controller_manager
            # alive so RViz can explain the fault and the operator can fix the
            # endpoint, rather than losing the whole demonstration process.
            {
                'hardware_components_initial_state.'
                'shutdown_on_initial_state_failure': False,
                'use_sim_time': use_sim_time,
            },
            Parameter(
                'update_rate', controller_manager_update_rate, value_type=int),
        ],
        output='screen',
    )

    def build_controller_spawner_chain(context):
        """Resolve the namespaced manager before registering exit handlers."""
        manager = controller_manager.perform(context)
        realtime = start_realtime_controller.perform(context) == 'true'
        state_rate = _ros_double_literal(state_publish_rate.perform(context))

        def spawner(name, inactive=False):
            arguments = [name]
            if name == 'gripper_controller':
                controller_arguments = (
                    '--ros-args -p stall_timeout:=' +
                    gripper_stall_timeout.perform(context))
                if backend.perform(context) == 'isaac':
                    controller_arguments += ' -p conventional_speed_control:=false'
                else:
                    controller_arguments += (
                        ' -p conventional_speed_percent:=' +
                        conventional_speed.perform(context))
                arguments.extend([
                    '--controller-ros-args',
                    controller_arguments,
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
            if inactive:
                arguments.append('--inactive')
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

    joint_state_filter = Node(
        package='onrobot_gripper_hardware',
        executable='joint_state_effort_filter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Keep the selected RViz fixed frame available during device discovery.
    # The hardware and joint-state broadcaster will add the moving transforms
    # as soon as their first state sample arrives.
    base_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gripper_base_frame',
        output='screen',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--frame-id', base_parent_frame, '--child-frame-id',
            [frame_prefix, 'base_link'],
        ],
        condition=IfCondition(publish_base_frame),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'backend',
            default_value='real',
            choices=['real', 'fake', 'isaac'],
            description='Execution backend',
        ),
        DeclareLaunchArgument(
            'frame_prefix',
            default_value='',
            description='Prefix applied by robot_state_publisher to TF frames',
        ),
        DeclareLaunchArgument(
            'description_xacro',
            default_value='realmesh_onrobot_2fg7.urdf.xacro',
            description=(
                'Description Xacro filename. The standalone showcase uses '
                'the detailed real-mesh description by default.'
            ),
        ),
        DeclareLaunchArgument(
            'fake_motion_speed_m_s',
            default_value='0.0',
            description=(
                'Deterministic fake-only motion rate; zero preserves '
                'instantaneous fake motion'
            ),
        ),
        DeclareLaunchArgument(
            'fake_stall',
            default_value='0',
            choices=['0', '1'],
            description='Fake-only switch that holds position under a goal',
        ),
        DeclareLaunchArgument(
            'isaac_joint_commands_topic',
            default_value='isaac_joint_commands',
            description='JointState commands sent to the Isaac articulation',
        ),
        DeclareLaunchArgument(
            'isaac_joint_states_topic',
            default_value='isaac_joint_states',
            description='JointState feedback received from Isaac Sim',
        ),
        DeclareLaunchArgument(
            'isaac_state_timeout_ms',
            default_value='250',
            description='Maximum age of Isaac joint feedback',
        ),
        DeclareLaunchArgument(
            'realtime_update_rate_hz',
            default_value='50',
            description='Realtime Modbus exchange rate in hertz',
        ),
        DeclareLaunchArgument(
            'controller_manager_update_rate_hz',
            default_value='100',
            description='ros2_control controller-manager update rate in hertz',
        ),
        DeclareLaunchArgument(
            'state_publish_rate_hz',
            default_value='100',
            description=(
                'Typed and realtime state publication rate in hertz'),
        ),
        DeclareLaunchArgument(
            'supply_power_w',
            default_value='48',
            description=(
                '2FG supply-power budget reapplied after connection; zero '
                'preserves the current device setting'),
        ),
        DeclareLaunchArgument(
            'conventional_speed_percent',
            default_value='50',
            description=(
                'Conventional 2FG motion speed percentage; valid values are '
                '1..100'),
        ),
        DeclareLaunchArgument(
            'visual_detail',
            default_value='detailed',
            choices=['detailed', 'low'],
            description=(
                '2FG7 body visual detail. Low keeps the same geometry '
                'origins and all collision meshes unchanged'),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the clock published by a simulation backend',
        ),
        DeclareLaunchArgument(
            'host',
            default_value='127.0.0.1',
            description='Gripper IP address or hostname',
        ),
        DeclareLaunchArgument(
            'port',
            default_value='502',
            description=(
                'Modbus TCP port reported by the gripper configuration'),
        ),
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
            'publish_base_frame',
            default_value='true',
            description='Publish a startup parent frame for standalone RViz',
        ),
        DeclareLaunchArgument(
            'base_parent_frame',
            default_value='world',
            description='Parent of base_link when publish_base_frame is true',
        ),
        DeclareLaunchArgument(
            'start_realtime_controller',
            default_value='false',
            description=(
                'Activate realtime ownership and load conventional control '
                'inactive'),
        ),
        DeclareLaunchArgument(
            'gripper_stall_timeout_s', default_value='2.0'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'controller_manager', default_value=PathJoinSubstitution([
                '/', LaunchConfiguration('namespace'), 'controller_manager']),
            description='Controller-manager service name for spawners'),
        OpaqueFunction(function=_validate_conventional_speed),
        robot_state_publisher,
        ros2_control_node,
        OpaqueFunction(function=build_controller_spawner_chain),
        joint_state_filter,
        base_frame,
    ])

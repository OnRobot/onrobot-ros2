"""Check the unified showcase launch contract."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions

import pytest


PACKAGE = Path(__file__).parents[1]


def _load_showcase_module():
    path = PACKAGE / 'launch' / 'showcase.launch.py'
    spec = importlib.util.spec_from_file_location('onrobot_showcase', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(**overrides):
    values = {
        'model': '2fg7',
        'control': 'conventional',
        'backend': 'real',
        'host': '192.0.2.42',
        'port': '1502',
        'transport': 'tcp',
        'serial_device': '/dev/ttyUSB0',
        'baud_rate': '1000000',
        'rtu_parity': 'even',
        'slave_id': '65',
        'connect_timeout_ms': '1000',
        'read_timeout_ms': '1000',
        'write_timeout_ms': '1000',
        'start_rviz': 'false',
        'namespace': 'fixture',
        'frame_prefix': 'fixture/',
        'fingertip_position': 'auto',
        'fake_motion_speed_m_s': '0.0',
        'fake_stall': '0',
        'isaac_joint_commands_topic': 'isaac_joint_commands',
        'isaac_joint_states_topic': 'isaac_joint_states',
        'isaac_state_timeout_ms': '250',
        'use_sim_time': 'false',
        'realtime_update_rate_hz': 'auto',
        'controller_manager_update_rate_hz': '100',
        'state_publish_rate_hz': '100',
        'supply_power_w': '48',
        'conventional_speed_percent': '50',
        'visual_detail': 'detailed',
        'gripper_stall_timeout_s': '2.0',
    }
    values.update(overrides)
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


def _resolved_arguments(include, context):
    return {
        name: perform_substitutions(
            context, normalize_to_list_of_substitutions(value))
        for name, value in include.launch_arguments
    }


@pytest.mark.parametrize('model', ('2fg7', '2fg14', 'rg2', 'rg6'))
@pytest.mark.parametrize(
    ('control', 'starts_realtime'),
    (('conventional', 'false'), ('realtime', 'true')),
)
def test_parallel_models_forward_inputs_to_canonical_bringup(
        model, control, starts_realtime):
    """Forward each parallel model and input without reinterpretation."""
    module = _load_showcase_module()
    context = _context(model=model, control=control, backend='fake')

    include = module._include_selected_model(context)[0]
    include.launch_description_source.get_launch_description(context)
    arguments = _resolved_arguments(include, context)

    assert include.launch_description_source.location.endswith(
        '/onrobot_gripper_bringup/launch/gripper.launch.py')
    assert arguments == {
        'model': model,
        'backend': 'fake',
        'transport': 'tcp',
        'host': '192.0.2.42',
        'port': '1502',
        'serial_device': '/dev/ttyUSB0',
        'baud_rate': '1000000',
        'rtu_parity': 'even',
        'slave_id': '65',
        'connect_timeout_ms': '1000',
        'read_timeout_ms': '1000',
        'write_timeout_ms': '1000',
        'namespace': 'fixture',
        'frame_prefix': 'fixture/',
        'fake_motion_speed_m_s': '0.0',
        'fake_stall': '0',
        'isaac_joint_commands_topic': 'isaac_joint_commands',
        'isaac_joint_states_topic': 'isaac_joint_states',
        'isaac_state_timeout_ms': '250',
        'use_sim_time': 'false',
        'realtime_update_rate_hz': 'auto',
        'controller_manager_update_rate_hz': '100',
        'state_publish_rate_hz': '100',
        'supply_power_w': '48',
        'conventional_speed_percent': '50',
        'visual_detail': 'detailed',
        'gripper_stall_timeout_s': '2.0',
        'start_realtime_controller': starts_realtime,
    }
    assert "package='rviz2'" in (
        PACKAGE / 'launch' / 'showcase.launch.py').read_text()


@pytest.mark.parametrize(
    ('model', 'control', 'expected_panel'),
    (
        ('2fg7', 'conventional', 'GripperControlPanel'),
        ('2fg7', 'realtime', 'RealtimeControlPanel'),
        ('2fg14', 'conventional', 'GripperControlPanel'),
        ('2fg14', 'realtime', 'RealtimeControlPanel'),
        ('rg2', 'conventional', 'GripperControlPanel'),
        ('rg2', 'realtime', 'RealtimeControlPanel'),
        ('rg6', 'conventional', 'GripperControlPanel'),
        ('rg6', 'realtime', 'RealtimeControlPanel'),
    ),
)
def test_parallel_showcase_layout_contains_model_and_selected_panel(
        model, control, expected_panel):
    """Every parallel showcase opens one model and one matching panel."""
    module = _load_showcase_module()
    config = (
        PACKAGE.parent / f'onrobot_{model}' / 'rviz' /
        module._RVIZ_CONFIGS[(model, control)])
    source = config.read_text()

    assert 'rviz_default_plugins/RobotModel' in source
    assert source.count(
        'onrobot_gripper_rviz_plugins/GripperControlPanel') == (
            expected_panel == 'GripperControlPanel')
    assert source.count(
        'onrobot_gripper_rviz_plugins/RealtimeControlPanel') == (
            expected_panel == 'RealtimeControlPanel')
    assert source.count(
        'onrobot_gripper_rviz_plugins/ForceHistoryPanel') == 1
    if control == 'realtime':
        assert 'Name: Realtime Position and Velocity' in source


def test_parallel_showcase_presets_keep_camera_navigation_tools():
    """Keep orbit, pan, zoom and selection available in every parallel view."""
    module = _load_showcase_module()
    for (model, _control), filename in module._RVIZ_CONFIGS.items():
        if model not in ('2fg7', '2fg14', 'rg2', 'rg6'):
            continue
        source = (PACKAGE.parent / f'onrobot_{model}' / 'rviz' / filename).read_text(
            encoding='utf-8')
        assert '  Tools:' in source
        for tool in ('Interact', 'MoveCamera', 'Select', 'FocusCamera'):
            assert f'    - Class: rviz_default_plugins/{tool}' in source


def test_three_finger_showcase_preset_keeps_camera_navigation_tools():
    """The shared 3FG preset must retain normal RViz camera navigation."""
    source = (PACKAGE.parent / 'onrobot_3fg25' / 'rviz' /
              '3fg25_control_showcase.rviz').read_text(encoding='utf-8')
    assert '  Tools:' in source
    for tool in ('Interact', 'MoveCamera', 'Select', 'FocusCamera'):
        assert f'    - Class: rviz_default_plugins/{tool}' in source


@pytest.mark.parametrize(
    ('model', 'reference_position'),
    (('3fg15', '2'), ('3fg25', '3')),
)
def test_three_finger_models_forward_inputs_and_reference_mounting(
        model, reference_position):
    """Use the model reference fingertip mounting when auto is selected."""
    module = _load_showcase_module()
    context = _context(model=model)

    include = module._include_selected_model(context)[0]
    include.launch_description_source.get_launch_description(context)
    arguments = _resolved_arguments(include, context)

    assert include.launch_description_source.location.endswith(
        '/onrobot_gripper_demos/launch/three_finger_showcase.launch.py')
    assert arguments == {
        'model': model,
        'host': '192.0.2.42',
        'port': '1502',
        'fingertip_position': reference_position,
        'start_rviz': 'false',
        'namespace': 'fixture',
        'frame_prefix': 'fixture/',
    }


def test_three_finger_explicit_fingertip_mounting_is_preserved():
    """Forward an operator-supplied physical fingertip mounting unchanged."""
    module = _load_showcase_module()
    context = _context(model='3fg25', fingertip_position='1')

    include = module._include_selected_model(context)[0]

    assert _resolved_arguments(include, context)['fingertip_position'] == '1'


@pytest.mark.parametrize('model,expected', [('3fg15', '2'), ('3fg25', '3')])
def test_direct_three_finger_launch_uses_the_same_reference_mounting(model, expected):
    path = PACKAGE / 'launch' / 'three_finger_showcase.launch.py'
    spec = importlib.util.spec_from_file_location('three_finger_showcase', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    declarations = module.generate_launch_description().entities
    argument = next(item for item in declarations
                    if getattr(item, 'name', None) == 'fingertip_position')
    context = _context(model=model)
    default = perform_substitutions(context, argument.default_value)
    context.launch_configurations['fingertip_position'] = default
    assert module._fingertip_position(context) == expected
    for explicit in ('1', '2', '3'):
        context.launch_configurations['fingertip_position'] = explicit
        assert module._fingertip_position(context) == explicit


@pytest.mark.parametrize(
    ('overrides', 'message'),
    (
        ({'model': '3fg15', 'control': 'realtime'},
         'does not implement realtime control'),
        ({'model': '3fg25', 'backend': 'fake'},
         'currently supports backend:=real only'),
    ),
)
def test_unsupported_three_finger_combinations_fail_closed(overrides, message):
    """Reject control combinations which cannot honor the public contract."""
    module = _load_showcase_module()

    with pytest.raises(RuntimeError, match=message):
        module._include_selected_model(_context(**overrides))


def test_three_finger_native_diameter_controller_contract():
    """Keep three-finger tools on their native diameter controller."""
    source = (
        PACKAGE / 'launch' / 'three_finger_showcase.launch.py').read_text()
    controllers = (
        PACKAGE / 'launch' / 'three_finger_controllers.yaml').read_text()
    assert "choices=['3fg15', '3fg25']" in source
    assert "f'realmesh_onrobot_{model}.urdf.xacro'" in source
    assert 'diameter_controller' in controllers
    assert 'gripper_state_broadcaster' in controllers
    assert "'gripper_state_broadcaster'" in source
    assert "'--ros-args -p model:=' + model" in source
    assert 'diagnostic_rate: 1.0' in controllers
    assert 'grip_diameter/minimum_external_diameter' in controllers
    assert 'grip_diameter/maximum_external_diameter' in controllers
    assert 'interfaces: [position, velocity]' in controllers
    assert 'use_local_topics: true' in controllers
    assert "package='onrobot_gripper_hardware'" in source
    assert "executable='joint_state_effort_filter'" in source


def test_showcase_defaults_to_standard_modbus_tcp_port():
    """Use the normal device port without environment-specific remapping."""
    source = (PACKAGE / 'launch' / 'showcase.launch.py').read_text()
    assert "'port', default_value='502'" in source
    assert '_DEFAULT_PORTS' not in source

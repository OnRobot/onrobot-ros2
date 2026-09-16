"""Unit checks for model-neutral launch value resolution."""

import importlib.util
from pathlib import Path
import subprocess
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from launch.actions import EmitEvent
from launch.events import Shutdown


LAUNCH_FILE = (
    Path(__file__).resolve().parents[1] / 'launch' / 'gripper.launch.py')
SPEC = importlib.util.spec_from_file_location(
    'onrobot_gripper_bringup_launch', LAUNCH_FILE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_auto_realtime_rate_resolves_before_model_launch_is_included():
    """Do not let the outer literal 'auto' leak into numeric hardware data."""
    assert MODULE._resolved_realtime_update_rate('2fg7', 'auto') == '50'
    assert MODULE._resolved_realtime_update_rate('2fg14', 'auto') == '50'
    assert MODULE._resolved_realtime_update_rate('rg2', 'auto') == '50'
    assert MODULE._resolved_realtime_update_rate('rg6', 'auto') == '50'


def test_direct_model_launches_use_the_same_conservative_rate():
    """Keep direct model launches aligned with model-neutral ``auto``."""
    root = Path(__file__).resolve().parents[2]
    for package in ('onrobot_2fg7', 'onrobot_2fg14', 'onrobot_rg2', 'onrobot_rg6'):
        launch = (root / package / 'launch/control.launch.py').read_text(
            encoding='utf-8')
        assert "'realtime_update_rate_hz', default_value='50'" in launch or (
            "'realtime_update_rate_hz',\n            default_value='50'" in launch)


def test_description_macros_use_the_same_conservative_rate():
    """Robot-composition users must get the launch default when omitting it."""
    root = Path(__file__).resolve().parents[2]
    for package in ('onrobot_2fg7', 'onrobot_2fg14', 'onrobot_rg2', 'onrobot_rg6'):
        for xacro in (root / package / 'urdf').glob('*.xacro'):
            text = xacro.read_text(encoding='utf-8')
            assert 'name="realtime_update_rate_hz" default="500"' not in text
            assert 'name="realtime_update_rate_hz" default="200"' not in text
            assert "realtime_update_rate_hz:='500'" not in text
            assert "realtime_update_rate_hz:='200'" not in text


def test_all_parallel_models_are_available_through_the_isaac_backend():
    """Keep the model-neutral Isaac launch contract aligned with its assets."""
    assert MODULE._ISAAC_MODELS == ('2fg7', '2fg14', 'rg2', 'rg6')


def test_explicit_realtime_rate_is_preserved():
    """Keep an operator-selected supported rate unchanged."""
    assert MODULE._resolved_realtime_update_rate('2fg7', '50') == '50'
    assert MODULE._resolved_realtime_update_rate('rg2', '100') == '100'


def test_controller_manager_default_is_namespaced_and_override_is_preserved():
    """Keep included managers isolated while honoring explicit launch input."""
    assert MODULE._resolved_controller_manager('left_gripper', '') == (
        '/left_gripper/controller_manager')
    assert MODULE._resolved_controller_manager('', '') == '/controller_manager'
    assert MODULE._resolved_controller_manager(
        'left_gripper', '/custom_manager') == '/custom_manager'


def test_model_launches_resolve_ordered_spawners_inside_launch_context():
    """Keep all supported models on the namespace-safe startup path."""
    root = Path(__file__).resolve().parents[2]
    for package in ('onrobot_2fg7', 'onrobot_2fg14', 'onrobot_rg2', 'onrobot_rg6'):
        text = (root / package / 'launch/control.launch.py').read_text(
            encoding='utf-8')
        assert 'OpaqueFunction(function=build_controller_spawner_chain)' in text
        assert 'RegisterEventHandler(OnProcessExit' in text
        assert "'--controller-manager', manager" in text


def test_qualification_stall_timeout_is_forwarded_to_every_model():
    """Allow a qualifier override without changing the normal 2 s default."""
    root = Path(__file__).resolve().parents[2]
    bringup = LAUNCH_FILE.read_text(encoding='utf-8')
    assert "'gripper_stall_timeout_s', default_value='2.0'" in bringup
    assert (
        "arguments['gripper_stall_timeout_s'] = LaunchConfiguration(" in
        bringup)
    for package in ('onrobot_2fg7', 'onrobot_2fg14', 'onrobot_rg2', 'onrobot_rg6'):
        text = (root / package / 'launch/control.launch.py').read_text(
            encoding='utf-8')
        assert "'gripper_stall_timeout_s', default_value='2.0'" in text
        assert "'--controller-ros-args'" in text
        assert "'--ros-args -p stall_timeout:='" in text


def test_model_spawner_chain_aborts_after_a_failed_spawner():
    """A failed controller load must not activate later controllers."""
    root = Path(__file__).resolve().parents[2]
    for package in ('onrobot_2fg7', 'onrobot_2fg14', 'onrobot_rg2', 'onrobot_rg6'):
        path = root / package / 'launch/control.launch.py'
        spec = importlib.util.spec_from_file_location(
            f'{package}_control_launch', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        next_action = object()
        handler = module._chain_on_success(next_action, 'failed_controller')

        assert handler(SimpleNamespace(returncode=0), None) == [next_action]
        failure = handler(SimpleNamespace(returncode=17), None)
        assert len(failure) == 1
        assert isinstance(failure[0], EmitEvent)
        assert isinstance(failure[0].event, Shutdown)
        assert failure[0].event.reason == (
            'controller spawner failed_controller exited with code 17')


def test_rate_controls_are_explicit_and_reach_every_parallel_launch():
    """Keep the device, manager, and fresh-state rates independently visible."""
    root = Path(__file__).resolve().parents[2]
    bringup = LAUNCH_FILE.read_text(encoding='utf-8')
    assert "'controller_manager_update_rate_hz', default_value='100'" in bringup
    assert "'state_publish_rate_hz', default_value='100'" in bringup
    assert "'controller_manager_update_rate_hz': LaunchConfiguration(" in bringup
    assert "'state_publish_rate_hz': LaunchConfiguration(" in bringup
    for package in ('onrobot_2fg7', 'onrobot_2fg14', 'onrobot_rg2', 'onrobot_rg6'):
        launch = (root / package / 'launch/control.launch.py').read_text(
            encoding='utf-8')
        normalized = ' '.join(launch.split())
        assert "'controller_manager_update_rate_hz', default_value='100'" in normalized
        assert "'state_publish_rate_hz', default_value='100'" in normalized
        assert "name='controller_manager'" in launch
        assert "Parameter( 'update_rate', controller_manager_update_rate" in normalized
        assert 'def _ros_double_literal(value):' in launch
        assert "'--ros-args -p publish_rate:='" in launch
        assert "'--ros-args -p state_publish_rate_hz:='" in launch


def test_conventional_speed_is_exposed_only_by_the_two_finger_path():
    """Expose native conventional percentage speed without inventing RG units."""
    root = Path(__file__).resolve().parents[2]
    bringup = LAUNCH_FILE.read_text(encoding='utf-8')
    assert "'conventional_speed_percent', default_value='50'" in bringup
    assert "arguments['conventional_speed_percent'] = LaunchConfiguration(" in bringup
    showcase = (root / 'onrobot_gripper_demos' / 'launch' /
                'showcase.launch.py').read_text(encoding='utf-8')
    assert "'conventional_speed_percent'" in showcase
    assert "default_value='50'" in showcase
    assert "'conventional_speed_percent': LaunchConfiguration(" in showcase
    for package in ('onrobot_2fg7', 'onrobot_2fg14'):
        launch = (root / package / 'launch/control.launch.py').read_text(
            encoding='utf-8')
        assert "'conventional_speed_percent'" in launch
        assert "default_value='50'" in launch
        assert 'conventional_speed_percent:=' in launch
        xacros = '\n'.join(
            path.read_text(encoding='utf-8')
            for path in (root / package / 'urdf').glob('*.xacro'))
        assert 'conventional_speed_percent' in xacros
        assert 'speed_percent' in xacros
    for package in ('onrobot_rg2', 'onrobot_rg6'):
        launch = (root / package / 'launch/control.launch.py').read_text(
            encoding='utf-8')
        assert 'conventional_speed_percent' not in launch


def _expand_2fg7_description(description, visual_detail):
    """Expand one installed 2FG7 entry point for a visual-only comparison."""
    root = Path(__file__).resolve().parents[2]
    result = subprocess.run(
        ['xacro', str(root / 'onrobot_2fg7' / 'urdf' / description),
         f'visual_detail:={visual_detail}'],
        check=True, capture_output=True, text=True)
    return ET.fromstring(result.stdout)


def _link_meshes(description):
    link = next(item for item in description.findall('link')
                if item.attrib['name'] == 'base_link')
    visual = link.find('./visual/geometry/mesh')
    collision = link.find('./collision')
    origin = link.find('./visual/origin')
    assert visual is not None
    assert collision is not None
    assert origin is not None
    return visual.attrib['filename'], ET.tostring(collision), origin.attrib


def test_2fg7_low_visual_detail_changes_only_the_body_visual_mesh():
    """Retain the physical description when choosing the lighter body DAE."""
    for entry_point in (
            'realmesh_onrobot_2fg7.urdf.xacro',
            'simplemesh_onrobot_2fg7.urdf.xacro'):
        detailed = _expand_2fg7_description(entry_point, 'detailed')
        low = _expand_2fg7_description(entry_point, 'low')
        detailed_visual, detailed_collision, detailed_origin = _link_meshes(
            detailed)
        low_visual, low_collision, low_origin = _link_meshes(low)

        assert detailed_visual.endswith('/base_link.dae')
        assert low_visual.endswith('/base_link_low.dae')
        assert low_collision == detailed_collision
        assert low_origin == detailed_origin
        assert [ET.tostring(item) for item in low.findall('joint')] == [
            ET.tostring(item) for item in detailed.findall('joint')]


def test_rtu_transport_contract_reaches_every_parallel_launch():
    """Keep serial settings visible from the common and showcase entry points."""
    transport_args = (
        'transport', 'serial_device', 'baud_rate', 'rtu_parity', 'slave_id',
        'connect_timeout_ms', 'read_timeout_ms', 'write_timeout_ms')
    root = Path(__file__).resolve().parents[2]
    common = LAUNCH_FILE.read_text(encoding='utf-8')
    showcase = (
        root / 'onrobot_gripper_demos' / 'launch' / 'showcase.launch.py'
    ).read_text(encoding='utf-8')
    for argument in transport_args:
        assert argument in common
        assert argument in showcase

    for package in ('onrobot_2fg7', 'onrobot_2fg14', 'onrobot_rg2', 'onrobot_rg6'):
        launch = (root / package / 'launch/control.launch.py').read_text(
            encoding='utf-8')
        xacros = '\n'.join(
            path.read_text(encoding='utf-8')
            for path in (root / package / 'urdf').glob('*.xacro'))
        for argument in transport_args:
            assert argument in launch
            assert argument in xacros

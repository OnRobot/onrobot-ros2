"""Unit checks for the realtime ROS-path measurement utility."""

import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'measure_realtime_ros_path.py'


def load_module():
    """Load the utility directly from its source path."""
    spec = importlib.util.spec_from_file_location('measure_ros_path', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def description(model='2fg7', transport='rtu', device='/dev/ttyUSB9'):
    """Return a compact ros2_control fixture description."""
    plugin = (
        'OnRobotRgSystem' if model.startswith('rg') else
        'OnRobotGripperSystem')
    return f"""<robot name="fixture">
      <ros2_control name="OnRobot{model.upper()}System" type="system">
        <hardware>
          <plugin>onrobot_gripper_hardware/{plugin}</plugin>
          <param name="transport">{transport}</param>
          <param name="serial_device">{device}</param>
          <param name="baud_rate">1000000</param>
          <param name="rtu_parity">even</param>
          <param name="slave_id">65</param>
        </hardware>
      </ros2_control>
    </robot>"""


@pytest.mark.parametrize('model', ('2fg7', '2fg14', 'rg2', 'rg6'))
def test_matches_exact_rtu_model_and_device(model):
    """Accept matching real RTU hardware settings."""
    module = load_module()
    match = module.matching_rtu_system(
        description(model=model), model, '/dev/ttyUSB9', 1000000, 65)
    assert match['parameters']['transport'] == 'rtu'


@pytest.mark.parametrize(
    ('model', 'transport', 'device'),
    (('rg2', 'rtu', '/dev/ttyUSB9'),
     ('2fg7', 'tcp', '/dev/ttyUSB9'),
     ('2fg7', 'rtu', '/dev/ttyUSB8')),
)
def test_rejects_wrong_model_transport_or_device(model, transport, device):
    """Reject model, transport, and device mismatches."""
    module = load_module()
    assert module.matching_rtu_system(
        description(transport=transport), model, device) is None


def test_rejects_fake_plugin_and_wrong_serial_settings():
    """Reject fake hardware and wrong serial settings."""
    module = load_module()
    source = description().replace(
        'OnRobotGripperSystem', 'OnRobotParallelGripperFakeSystem')
    assert module.matching_rtu_system(
        source, '2fg7', '/dev/ttyUSB9', 1000000, 65) is None
    assert module.matching_rtu_system(
        description(), '2fg7', '/dev/ttyUSB9', 115200, 65) is None
    assert module.matching_rtu_system(
        description(), '2fg7', '/dev/ttyUSB9', 1000000, 66) is None


def test_percentiles_are_interpolated_and_empty_is_explicit():
    """Keep timing summaries deterministic and missing data explicit."""
    module = load_module()
    assert module.percentile([1.0, 2.0, 3.0], 0.5) == 2.0
    assert module.percentile([], 0.99) is None
    assert module.timing_summary([])['max'] is None


def test_rate_identity_requires_matching_hardware_and_ros_rates():
    """Reject evidence collected against a differently configured live path."""
    module = load_module()
    system = {
        'parameters': {'realtime_update_rate_hz': '200'},
    }
    controllers = {
        'realtime_controller': {'update_rate': 200},
        'gripper_state_broadcaster': {'update_rate': 200},
    }
    assert module.rate_mismatches(system, controllers, 200) == []

    controllers['realtime_controller']['update_rate'] = 500
    controllers['gripper_state_broadcaster']['update_rate'] = 500
    assert module.rate_mismatches(system, controllers, 200) == []

    system['parameters']['realtime_update_rate_hz'] = '500'
    assert module.rate_mismatches(system, controllers, 200) == [
        'hardware realtime_update_rate_hz=500',
    ]

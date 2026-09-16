"""Response metrics and diagnostic inputs; runtime evidence is separate."""
import importlib.util
import math
from pathlib import Path
import sys

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
try:
    spec = importlib.util.spec_from_file_location('drive_response', SCRIPTS / 'run_drive_step_response.py')
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
finally:
    sys.path.remove(str(SCRIPTS))


@pytest.mark.parametrize('direction', [1, -1])
def test_response_metrics_preserve_direction_and_time(direction):
    samples = [dict(time_s=i / 100, measured=direction * min(i / 10, 1),
                    velocity=direction * (10 if i < 10 else 0)) for i in range(40)]
    result = runner._measure_step(samples, direction)
    assert result['rise_time_10_to_90_s'] == pytest.approx(.08)
    assert result['settling_time_5_percent_s'] == pytest.approx(.10)
    assert result['command_endpoint_error'] == 0
    assert result['overshoot'] == 0


@pytest.mark.parametrize('field', ['time_s', 'measured', 'velocity'])
def test_nonfinite_trace_cannot_pass(field):
    sample = dict(time_s=0, measured=0, velocity=0)
    sample[field] = math.nan
    with pytest.raises(ValueError):
        runner._measure_step([sample], 1)


@pytest.mark.parametrize('flag,value,model', [
    ('--drive-armature-kg-m2', '.1', '2fg7'),
    ('--drive-armature-kg-m2', '-1', 'rg2'),
    ('--drive-armature-kg', '1', 'rg2'),
    ('--drive-armature-kg', '-1', '2fg14'),
    ('--drive-armature-kg', 'nan', '2fg7'),
    ('--drive-damping-si', 'nan', 'rg6'),
    ('--drive-damping-si', '-1', '2fg14')])
def test_invalid_diagnostic_inputs_fail_before_isaac(flag, value, model):
    with pytest.raises(SystemExit):
        runner.arguments(['--model', model, '--output', 'test.json', flag, value])


def test_default_does_not_override_product_drive():
    args = runner.arguments(['--model', 'rg6', '--output', 'test.json'])
    assert args.drive_armature_kg_m2 is None
    assert args.drive_armature_kg is None
    assert args.drive_damping_si is None


def test_native_joint_endpoints_must_be_paired_and_finite():
    base = ['--model', 'rg2', '--output', 'test.json']
    for options in (['--start-joint', '.3'], ['--start-joint', '.3', '--target-joint', 'nan']):
        with pytest.raises(SystemExit):
            runner.arguments(base + options)
    args = runner.arguments(base + ['--start-joint', '.3', '--target-joint', '1.1'])
    assert (args.start_joint, args.target_joint) == (.3, 1.1)


def test_no_motion_is_not_an_instantaneous_successful_response():
    result = runner._measure_step([
        dict(time_s=i / 100, measured=.3, velocity=0) for i in range(40)], 1.1)
    assert result['response_detected'] is False
    assert result['rise_time_10_to_90_s'] is None
    assert result['settling_time_5_percent_s'] is None
    assert result['command_endpoint_error'] == pytest.approx(-.8)


@pytest.mark.parametrize('times', [[0, 0], [0, -.1], [-.1, .1]])
def test_invalid_sample_clock_is_rejected(times):
    with pytest.raises(ValueError, match='increase strictly'):
        runner._measure_step([dict(time_s=t, measured=i, velocity=1)
                              for i, t in enumerate(times)], 1)


def test_existing_evidence_is_not_overwritten(tmp_path):
    output = tmp_path / 'response.json'
    output.write_text('preserve')
    with pytest.raises(SystemExit):
        runner.arguments(['--model', 'rg2', '--output', str(output)])
    assert output.read_text() == 'preserve'


@pytest.mark.parametrize('model,value', [('rg2', '.05'), ('2fg7', 'nan'), ('2fg14', '0')])
def test_linear_speed_override_requires_linear_model_and_finite_positive_speed(model, value):
    with pytest.raises(SystemExit):
        runner.arguments(['--model', model, '--output', 'test.json',
                          '--maximum-joint-velocity-m-s', value])


def test_linear_speed_override_is_one_finger_speed():
    args = runner.arguments(['--model', '2fg7', '--output', 'test.json',
                             '--maximum-joint-velocity-m-s', '.05'])
    assert args.maximum_joint_velocity_m_s == .05

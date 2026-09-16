"""Offline acceptance/argument checks; physical outcomes require Isaac runs."""
import importlib.util
import math
from pathlib import Path
import sys

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
try:
    spec = importlib.util.spec_from_file_location('workpiece', SCRIPTS / 'run_workpiece_showcase.py')
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
finally:
    sys.path.remove(str(SCRIPTS))


def good_cycle():
    return dict(grasp_settled=True, maximum_relative_slip_m=.0001,
                maximum_relative_rotation_rad=.01,
                placement_xy_error_m=.0001, placement_height_error_m=.0001,
                peak_table_load_during_approach_n=6.985,
                peak_table_load_during_closure_n=70.,
                peak_nonpad_contact_during_swing_n=0.,
                maximum_mimic_error=.00001,
                maximum_lift_m=.12, maximum_swing_rad=math.radians(20))


def test_accepts_measured_cycle_and_reports_closure_preload_separately():
    assert runner.cycle_passes(good_cycle(), .712, 20)


def test_contact_relative_preload_uses_generalized_force_without_moving_pose():
    # 280 N per opposed pad requires 560 N in the one-jaw generalized coordinate.
    q = .0035
    target = runner.linear_preload_target(q, 40000., 560.)
    assert target == pytest.approx(-.0105)
    assert 40000. * (q - target) == pytest.approx(560.)
    assert q == .0035


@pytest.mark.parametrize('q,k,f', [
    (math.nan, 40000., 560.), (.0035, 0., 560.), (.0035, -1., 560.),
    (.0035, math.inf, 560.), (.0035, 40000., 0.), (.0035, 40000., math.nan)])
def test_preload_rejects_invalid_inputs(q, k, f):
    with pytest.raises(ValueError):
        runner.linear_preload_target(q, k, f)


def test_preload_is_not_silently_applied_to_angular_linkages():
    with pytest.raises(SystemExit):
        runner.arguments(['--model', 'rg2', '--output', 'out.json',
                          '--hold-control', 'preload-position'])
    with pytest.raises(SystemExit):
        runner.arguments(['--model', 'rg6', '--output', 'out.json',
                          '--drive-armature-kg', '1'])


def test_contact_impulse_decomposition_preserves_units_and_normal_scale():
    assert runner.contact_force_components([.3, .4, 2.], [0., 0., 1.], .01) == (200., 50.)
    assert runner.contact_force_components([.3, .4, -2.], [0., 0., 2.], .01) == (200., 50.)


@pytest.mark.parametrize('impulse,normal,dt', [
    ([0., 0., 1.], [0., 0., 0.], .01), ([0., 0., math.nan], [0., 0., 1.], .01),
    ([0., 1.], [0., 0., 1.], .01), ([0., 0., 1.], [0., 0., 1.], 0.)])
def test_invalid_contact_observation_is_not_zero(impulse, normal, dt):
    with pytest.raises(ValueError):
        runner.contact_force_components(impulse, normal, dt)


def test_supported_weight_checks_scale_and_sign():
    samples = [{'cube_contact_force_vectors_n': {'table': [0., 0., 7 * 9.81]}}] * 12
    assert runner.supported_weight_check(samples, 7.)['passed']
    wrong = [{'cube_contact_force_vectors_n': {'table': [0., 0., -7 * 9.81]}}] * 12
    assert not runner.supported_weight_check(wrong, 7.)['passed']
    wrong = [{'cube_contact_force_vectors_n': {'table': [0., 0., 7 * 9.81 * 240]}}] * 12
    assert not runner.supported_weight_check(wrong, 7.)['passed']


def test_contact_averaging_preserves_complete_step_force_and_instantaneous_data():
    window = runner.ContactForceAverage(.001)
    samples = []
    for i in range(48):
        instantaneous = {'pad': [0., 0., 19.62 + (1. if i % 2 else -1.)]}
        window.add(instantaneous)
        if (i + 1) % 4 == 0:
            samples.append({'cube_contact_force_vectors_n': instantaneous,
                            'cube_contact_force_average': window.take()})
    assert all(s['cube_contact_force_vectors_n']['pad'][2] == 20.62 for s in samples)
    assert all(s['cube_contact_force_average']['step_count'] == 4 for s in samples)
    assert runner.supported_weight_check(samples, 2.)['passed']
    for s in samples:
        s['cube_contact_force_average']['force_vectors_n']['pad'][2] += 2.
    assert not runner.supported_weight_check(samples, 2.)['passed']


def test_contact_average_includes_missing_contact_and_resets_after_take():
    window = runner.ContactForceAverage(.01)
    window.add({'left': [4., -8., 12.]})
    window.add({})
    window.add({'right': [-4., 8., 12.]})
    window.add({})
    result = window.take()
    assert result == {'step_count': 4, 'duration_s': .04,
                      'force_vectors_n': {'left': [1., -2., 3.], 'right': [-1., 2., 3.]}}
    with pytest.raises(ValueError):
        window.take()
    window.add({})
    assert window.take()['force_vectors_n'] == {}


def test_contact_average_uses_duration_not_observation_count():
    samples = [{'cube_contact_force_average': {'step_count': steps, 'duration_s': steps * .001,
                'force_vectors_n': {'table': [0., 0., force]}}}
               for steps, force in [(1, 0.), (3, 4 * 19.62 / 3)] * 4]
    assert runner.supported_weight_check(samples, 2.)['passed']


@pytest.mark.parametrize('dt', [0., -1., math.inf, math.nan])
def test_contact_average_rejects_invalid_timestep(dt):
    with pytest.raises(ValueError):
        runner.ContactForceAverage(dt)


@pytest.mark.parametrize('field,value', [('step_count', 0), ('step_count', True),
                                      ('duration_s', 0.), ('duration_s', math.nan), ('duration_s', True)])
def test_invalid_average_does_not_fall_back_to_a_passing_instantaneous_sample(field, value):
    sample = {'cube_contact_force_vectors_n': {'table': [0., 0., 19.62]},
              'cube_contact_force_average': {'step_count': 4, 'duration_s': .004,
                                            'force_vectors_n': {'table': [0., 0., 19.62]}}}
    sample['cube_contact_force_average'][field] = value
    with pytest.raises(ValueError):
        runner.supported_weight_check([sample] * 8, 2.)


def test_missing_average_mapping_is_rejected():
    with pytest.raises(ValueError):
        runner.sampled_cube_forces({'cube_contact_force_average': None})


@pytest.mark.parametrize('vector', [[0., math.nan, 1.], [1., 2.]])
def test_invalid_force_cannot_enter_average(vector):
    with pytest.raises(ValueError):
        runner.ContactForceAverage(.01).add({'pad': vector})


@pytest.mark.parametrize('sample', [{}, {'cube_contact_force_vectors_n': {}},
    {'cube_contact_force_vectors_n': {'table': [0., 0., math.nan]}},
    {'cube_contact_force_vectors_n': {'table': [1.]}}])
def test_missing_weight_observation_does_not_pass(sample):
    with pytest.raises(ValueError):
        runner.supported_weight_check([sample] * 12, .712)


@pytest.mark.parametrize('field,value', [
    ('maximum_relative_slip_m', .004), ('maximum_relative_rotation_rad', .2),
    ('placement_xy_error_m', .006), ('placement_height_error_m', .004),
    ('peak_table_load_during_approach_n', 5000),
    ('peak_nonpad_contact_during_swing_n', 7),
    ('maximum_mimic_error', .001),
    ('maximum_lift_m', 0.), ('maximum_swing_rad', 0.)])
def test_rejects_drop_crushing_support_or_missing_motion(field, value):
    result = good_cycle()
    result[field] = value
    assert not runner.cycle_passes(result, .712, 20)


@pytest.mark.parametrize('field', list(good_cycle()))
def test_nonfinite_required_measurements_fail(field):
    result = good_cycle()
    result[field] = math.nan
    assert not runner.cycle_passes(result, .712, 20)
    del result[field]
    assert not runner.cycle_passes(result, .712, 20)


def settled_window():
    return [dict(time_s=i / 100, jaw=.4, cube=[0, 0, .38], pad_normals=[40, 40])
            for i in range(21)]


def test_grasp_readiness_requires_quiet_bilateral_contact():
    window = settled_window()
    assert runner.grasp_is_settled(window, angular=True)
    assert not runner.grasp_is_settled(window[:10], angular=True)
    for row in window:
        row['pad_normals'][1] = 0
    assert not runner.grasp_is_settled(window, angular=True)


@pytest.mark.parametrize('field,value', [('jaw', .41), ('cube', [0, 0, .381]),
                                        ('jaw', math.nan), ('pad_normals', [40, math.nan])])
def test_unsettled_or_invalid_grasp_cannot_start_lift(field, value):
    window = settled_window()
    window[-1][field] = value
    assert not runner.grasp_is_settled(window, angular=True)


def test_rg_mimic_error_uses_radians():
    result = good_cycle()
    result['maximum_mimic_error'] = .001
    assert runner.cycle_passes(result, .712, 20, 'rg2')
    result['maximum_mimic_error'] = .03
    assert not runner.cycle_passes(result, .712, 20, 'rg6')


@pytest.mark.parametrize('model', ['2fg7', '2fg14', 'rg2', 'rg6'])
def test_family_defaults(model):
    args = runner.arguments(['--model', model, '--output', 'out.json'])
    assert (args.cube_size_m, args.payload_kg, args.cycles) == (.062, .712, 3)
    assert not args.contact_stiffness_n_m
    assert args.hold_control == 'position'
    assert args.solver_type == 'TGS'
    assert args.physics_hz == 480
    assert args.external_forces_every_iteration is True
    assert not args.override_pad_material
    assert args.drive_stiffness_si is None
    assert args.drive_damping_si is None
    assert args.drive_armature_kg_m2 is None
    assert args.drive_armature_kg is None
    assert args.mimic_damping_ratio is None
    assert args.solver_velocity_iterations is None


def test_tgs_force_iteration_can_be_disabled_for_an_explicit_comparison():
    args = runner.arguments(['--model', '2fg7', '--output', 'out.json',
                             '--no-external-forces-every-iteration'])
    assert args.external_forces_every_iteration is False


@pytest.mark.parametrize('model', ['2fg7', '2fg14', 'rg2', 'rg6'])
def test_finer_loaded_timestep_is_accepted(model):
    args = runner.arguments(['--model', model, '--output', 'out.json', '--physics-hz', '960'])
    assert args.physics_hz == 960


@pytest.mark.parametrize('flag,value', [
    ('--swing-cycles', '1'), ('--payload-kg', 'nan'), ('--cube-size-m', '-1'),
    ('--physics-hz', '60'), ('--effort-fraction', '1.1'),
    ('--static-friction', '2'), ('--dynamic-friction', '.9'),
    ('--mimic-frequency-hz', 'nan'), ('--mimic-damping-ratio', '-1'),
    ('--drive-stiffness-si', 'inf'), ('--drive-damping-si', '-1'),
    ('--force-ramp-seconds', '0'), ('--force-ramp-seconds', 'nan'),
    ('--drive-armature-kg', '-1'), ('--drive-armature-kg', 'nan'),
    ('--drive-armature-kg-m2', '0.01')])
def test_rejects_invalid_physics_inputs(flag, value):
    with pytest.raises(SystemExit):
        runner.arguments(['--model', '2fg7', '--output', 'out.json', flag, value])

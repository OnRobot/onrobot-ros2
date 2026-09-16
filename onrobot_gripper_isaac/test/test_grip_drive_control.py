"""Guarded preload tests without requiring Isaac imports."""
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from grip_drive_control import (PadContactObserver, contact_preload_reference,
                                hold_control, linear_preload_target)


def filled_observer():
    observer = PadContactObserver('/block', ['/left', '/right'], 1 / 240)
    for _ in range(observer.history.maxlen):
        observer.begin_step()
        observer.observe('/block', '/left', (0, .01, 0), (0, 1, 0))
        observer.observe('/right', '/block', (0, -.02, 0), (0, -1, 0))
        observer.end_step()
    return observer


def test_bilateral_presence_reversed_actors_and_reference():
    observer = filled_observer()
    assert observer.require_bilateral_contact()['last_pad_normal_magnitudes_n'] == pytest.approx([2.4, 4.8])

    class Articulation:
        def get_dof_positions(self): return [[.009]]
        def get_dof_gains(self): return [[20000]], [[282.8427]]
        def get_dof_max_efforts(self): return [[280]]

    report = contact_preload_reference(Articulation(), 0, observer)
    assert report['drive_reference_m'] == pytest.approx(-.005)
    assert report['nominal_load_per_pad_n'] == 140


def test_absent_reports_are_not_reused():
    observer = filled_observer()
    observer.begin_step()
    observer.end_step()
    with pytest.raises(RuntimeError, match='bilateral'):
        observer.require_bilateral_contact()


def test_other_contact_and_one_sided_contact_do_not_allow_preload():
    observer = PadContactObserver('/block', ['/left', '/right'], .01)
    for _ in range(observer.history.maxlen):
        observer.begin_step()
        observer.observe('/block', '/left', (0, .01, 0), (0, 1, 0))
        observer.observe('/table', '/block', (0, 0, 100), (0, 0, 1))
        observer.end_step()
    with pytest.raises(RuntimeError, match='bilateral'):
        observer.require_bilateral_contact()


def test_error_is_sticky():
    observer = filled_observer()
    observer.error = 'injected callback error'
    for operation in (observer.begin_step, observer.end_step, observer.require_bilateral_contact):
        with pytest.raises(RuntimeError, match='injected callback'):
            operation()


def test_full_vectors_separate_friction_and_reverse_actor1():
    observer = PadContactObserver('/block', ['/left', '/right'], .01, capture_vectors=True)
    observer.begin_step()
    observer.observe('/block', '/left', (1, 0, 0), (1, 0, 0), (0, 1, 0))
    observer.observe_friction('/block', '/left', (0, 0, .2))
    observer.observe('/right', '/block', (1, 0, 0), (1, 0, 0), (1, 0, 0))
    observer.observe_friction('/right', '/block', (0, 0, -.2))
    # Support is relevant to force balance, but is not bilateral pad contact.
    observer.observe('/table', '/block', (0, 0, -.3), (0, 0, -1))
    observer.end_step()
    sample = observer.vector_sample()
    assert observer.normals == [100, 100]
    assert sample['normal_vectors_on_block_n'] == {
        '/left': [100, 0, 0], '/right': [-100, 0, 0], '/table': [0, 0, 30]}
    assert sample['friction_vectors_on_block_n'] == {
        '/left': [0, 0, 20], '/right': [0, 0, 20]}
    assert sample['contact_points'][1]['cube_is_actor0'] is False
    observer.begin_step()
    assert observer.vector_sample() == {'normal_vectors_on_block_n': {},
                                         'friction_vectors_on_block_n': {}, 'contact_points': []}
    assert sample['normal_vectors_on_block_n']['/table'] == [0, 0, 30]


def test_vectors_are_explicitly_opt_in():
    observer = filled_observer()
    with pytest.raises(RuntimeError, match='not enabled'):
        observer.vector_sample()
    observer.observe_friction('/block', '/left', (math.nan, 0, 0))
    assert observer.friction_vectors == {}


def test_full_vector_malformed_friction_fails():
    observer = PadContactObserver('/block', ['/left', '/right'], .01, capture_vectors=True)
    with pytest.raises(ValueError, match='contact impulse'):
        observer.observe_friction('/block', '/left', (0, math.nan, 0))


@pytest.mark.parametrize('impulse,normal', [((math.nan, 0, 0), (1, 0, 0)),
                                         ((1, 0, 0), (0, 0, 0)),
                                         ((1, 0), (1, 0, 0))])
def test_invalid_contact_fails(impulse, normal):
    with pytest.raises(ValueError):
        filled_observer().observe('/block', '/left', impulse, normal)


@pytest.mark.parametrize('position,stiffness,force', [(math.nan, 20, 1), (1, 0, 1),
                                                     (1, 20, math.inf), (1, 20, -1),
                                                     (1, 1e-300, 1e300)])
def test_bad_preload_fails(position, stiffness, force):
    with pytest.raises(ValueError):
        linear_preload_target(position, stiffness, force)


def test_policy_is_explicit_and_does_not_apply_linear_rule_to_rg():
    assert hold_control('2fg7', {}) == 'position'
    assert hold_control('2fg7', {'hold_control': 'preload-position'}) == 'preload-position'
    assert hold_control('2fg7', {'hold_control': 'preload-position'}, 'position') == 'position'
    with pytest.raises(ValueError):
        hold_control('rg6', {}, 'preload-position')
    with pytest.raises(ValueError):
        hold_control('2fg7', {'hold_control': 'unknown'})

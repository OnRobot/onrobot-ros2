#!/usr/bin/env python3
"""Unit tests for the dependency-free synthetic-data learner."""

import json
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE_ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from synthetic_grip_learning import (  # noqa: E402
    CONTROLLED_EFFORT_DESIGN,
    FEATURE_NAMES,
    load_jsonl,
    predict_probability,
    select_candidate,
    train_selector,
    train_controlled_selector,
)


def _episodes():
    episodes = []
    for index in range(25):
        payload = (0.5, 0.75, 1.0)[index % 3]
        friction = (0.3, 0.5, 0.75, 1.0)[(index // 3) % 4]
        drive = (0.5, 0.75, 1.0)[(index // 2) % 3]
        success = int((friction >= 0.75 and drive >= 0.75) or
                      (payload <= 0.5 and drive >= 1.0))
        episodes.append({
            'schema_version': 1,
            'seed': index + 1,
            'valid': True,
            'label': success,
            'scenario_group': f'payload-{payload:.2f}',
            'candidate': {
                'model': '2fg7',
                'payload_fraction': payload,
                'fingertip_static_friction': friction,
                'fingertip_dynamic_friction': max(0.2, friction * 0.8),
                'drive_force_fraction': drive,
            },
        })
    return episodes


def test_train_selector_is_deterministic_and_has_a_holdout():
    """Training is repeatable and reports a deterministic holdout split."""
    first = train_selector(_episodes())
    second = train_selector(_episodes())
    assert first == second
    assert first['status'] == 'trained'
    assert first['feature_names'] == list(FEATURE_NAMES)
    assert first['training']['train_episodes'] > 0
    assert first['training']['holdout_episodes'] > 0
    assert first['training']['holdout_accuracy'] is not None
    selection = first['training']['holdout_selection']
    assert selection['groups_evaluated'] > 0
    assert selection['learned_success_rate'] is not None
    assert selection['baseline_success_rate'] is not None


def test_saved_selector_scores_and_selects_candidates():
    """A saved-style model selects the lowest effort meeting its policy."""
    model = train_selector(_episodes())
    selected = select_candidate(model, _episodes())
    assert selected['status'] == 'selected'
    assert not selected['abstained']
    assert selected['seed'] in {9, 10}
    assert selected['candidate']['drive_force_fraction'] == 0.75
    assert 0.0 <= selected['probability'] <= 1.0
    assert 0.0 <= predict_probability(model, _episodes()[0]) <= 1.0


def test_selector_abstains_when_no_candidate_meets_threshold():
    """The selector never recommends a below-threshold candidate."""
    model = train_selector(_episodes())
    result = select_candidate(model, _episodes()[:4])
    assert result['status'] == 'abstained'
    assert result['abstained']
    assert result['seed'] is None
    assert result['candidate'] is None
    assert result['best_probability'] < result['threshold']


def test_loader_and_invalid_episode_handling(tmp_path):
    """The JSONL loader works and incomplete data is not trained."""
    dataset = tmp_path / 'dataset.jsonl'
    dataset.write_text(
        '\n'.join(json.dumps(item) for item in _episodes()) + '\n',
        encoding='utf-8')
    assert len(load_jsonl(dataset)) == 25
    incomplete = _episodes()[:5]
    incomplete.append({'valid': False, 'error': 'runner failed'})
    report = train_selector(incomplete)
    assert report['status'] == 'insufficient-data'


def test_learning_script_keeps_collection_shell_free():
    """Collection uses an argument vector rather than a shell command."""
    source = (SCRIPTS / 'run_synthetic_grip_learning.py').read_text(
        encoding='utf-8')
    assert 'shell=True' not in source
    assert 'subprocess.run(' in source
    assert "'--apply-profile-physics-settings'" in source


def test_pose_feature_is_backward_compatible_and_bounded():
    """Old episodes default to zero offset; unsafe offsets are rejected."""
    from synthetic_grip_learning import episode_features

    episode = _episodes()[0]
    assert episode_features(episode)[4] == 0.0
    episode['candidate']['pose_offset_m'] = 0.002
    assert episode_features(episode)[4] == 0.002
    episode['candidate']['pose_offset_m'] = 0.051
    try:
        episode_features(episode)
    except ValueError as error:
        assert 'pose offset' in str(error)
    else:
        raise AssertionError('unbounded pose offset was accepted')


def test_controlled_collection_design_keeps_environment_fixed_per_effort_sweep():
    """Each training choice compares efforts in one fixed environment."""
    import importlib.util

    script = SCRIPTS / 'run_synthetic_grip_learning.py'
    spec = importlib.util.spec_from_file_location('synthetic_collection', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    trials = module.controlled_effort_trials(
        '2fg7', 7.0, 10, (0.15, 0.35, 0.55, 0.75, 1.0), 77)
    environments = {}
    for trial in trials:
        environments.setdefault(trial['environment_id'], []).append(trial)
    assert len(environments) == 10
    assert {trial['environment_split'] for trial in trials} == {
        'train', 'validation', 'test'}
    for values in environments.values():
        assert len(values) == 5
        assert {value['candidate']['drive_force_fraction'] for value in values} == {
            0.15, 0.35, 0.55, 0.75, 1.0}
        fixed = {key: values[0]['candidate'][key] for key in (
            'payload_fraction', 'fingertip_static_friction',
            'fingertip_dynamic_friction', 'block_static_friction',
            'block_dynamic_friction', 'pose_offset_m')}
        assert all({key: value['candidate'][key] for key in fixed} == fixed
                   for value in values)
    expanded = module.controlled_effort_trials(
        '2fg7', 7.0, 14, (0.15, 1.0), 78)
    assert len({trial['environment_id'] for trial in expanded}) == 14
    assert module._default_paths('2fg7')[2].name == (
        'run_physical_motion_showcase.py')
    assert 'episode[\'trajectory\']' in (
        (SCRIPTS / 'run_synthetic_grip_learning.py').read_text(
            encoding='utf-8'))
    assert "'environment_id': trial['environment_id']" in (
        (SCRIPTS / 'run_synthetic_grip_learning.py').read_text(
            encoding='utf-8'))
    assert "'trajectory'" in (
        (SCRIPTS / 'run_payload_retention_test.py').read_text(
            encoding='utf-8'))


def test_installed_learning_runner_uses_sibling_physics_runner(tmp_path):
    """Released packages resolve executable runners from lib, not share."""
    import importlib.util

    script = SCRIPTS / 'run_synthetic_grip_learning.py'
    spec = importlib.util.spec_from_file_location('installed_learning', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    installed = tmp_path / 'lib' / 'onrobot_gripper_isaac'
    installed.mkdir(parents=True)
    (tmp_path / 'share/onrobot_gripper_isaac/assets').mkdir(parents=True)
    module.__file__ = str(installed / 'run_synthetic_grip_learning.py')
    expected = installed / 'run_physical_motion_showcase.py'
    expected.write_text('#!/usr/bin/env python3\n', encoding='utf-8')
    assert module._default_paths('2fg7')[2] == expected


def test_presentation_plan_rebases_synchronized_artifacts(tmp_path):
    """A plan remains usable after its producer workspace is relocated."""
    import importlib.util

    script = SCRIPTS / 'run_synthetic_grip_learning.py'
    spec = importlib.util.spec_from_file_location('portable_plan', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    plan = tmp_path / 'presentation_plan.json'
    plan.write_text('{}\n', encoding='utf-8')
    selector = tmp_path / 'selector.json'
    selector.write_text('{}\n', encoding='utf-8')
    assert module._resolve_plan_artifact(
        '/sandbox/work/ws/old/selector.json', plan, 'selector model') == selector


def test_controlled_training_has_disjoint_environment_evaluation():
    """Testing uses whole unseen environments and equal comparison denominators."""
    import importlib.util

    script = SCRIPTS / 'run_synthetic_grip_learning.py'
    spec = importlib.util.spec_from_file_location('controlled_collection', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    trials = module.controlled_effort_trials(
        '2fg7', 7.0, 10, (0.15, 0.35, 0.55, 0.75, 1.0), 9)
    episodes = []
    for trial in trials:
        candidate = trial['candidate']
        # A deterministic simulated physics outcome: weak effort cannot lift
        # a full payload, while adequate effort can.  This exercises the split
        # and fair selection mechanics without claiming a real PhysX result.
        threshold = 0.55 if candidate['payload_fraction'] >= 0.9 else 0.35
        episodes.append({
            'schema_version': 2,
            'experiment_design': CONTROLLED_EFFORT_DESIGN,
            'valid': True,
            'seed': candidate['seed'],
            'environment_id': trial['environment_id'],
            'environment_split': trial['environment_split'],
            'candidate': candidate,
            'label': int(candidate['drive_force_fraction'] >= threshold),
        })
    trained = train_controlled_selector(episodes)
    assert trained['status'] == 'trained'
    assert trained['model_type'] == 'monotonic-demand-effort-threshold-v1'
    summary = trained['training']['test_selection_summary']
    assert summary['environments_evaluated'] == 2
    assert summary['selected_success_rate_over_all_test_environments'] is not None
    for entry in trained['training']['test_selection']:
        assert entry['baseline_drive_force_fraction'] == 0.15
        assert not entry['learned_abstained']
        assert entry['learned_drive_force_fraction'] >= 0.35
    assert 0.0 <= predict_probability(trained, episodes[0]) <= 1.0
    leaked = [dict(item) for item in episodes]
    next(item for item in leaked if item['environment_split'] == 'train')[
        'environment_split'] = 'test'
    try:
        train_controlled_selector(leaked)
    except ValueError as error:
        assert 'multiple data splits' in str(error)
    else:
        raise AssertionError('cross-split environment was accepted')

#!/usr/bin/env python3
"""Small dependency-free learner for Isaac synthetic grip experiments.

The collector supplies labels from actual PhysX reports.  This module only
handles the bounded, deterministic feature extraction and binary classifier;
it never turns a failed or incomplete simulation into a success label.
"""

import hashlib
import json
import math
from pathlib import Path


SCHEMA_VERSION = 2
CONTROLLED_EFFORT_DESIGN = 'controlled-effort-v1'
DEFAULT_PREDICTED_SUCCESS_THRESHOLD = 0.75
FEATURE_NAMES = (
    'payload_fraction',
    'fingertip_static_friction',
    'fingertip_dynamic_friction',
    'drive_force_fraction',
    'pose_offset_m',
    'friction_force_demand_fraction',
)


def _finite(value) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def episode_features(episode: dict) -> list[float]:
    """Extract physically meaningful candidate-setting features."""
    candidate = episode.get('candidate', {})
    payload = float(candidate['payload_fraction'])
    static = float(candidate['fingertip_static_friction'])
    dynamic = float(candidate['fingertip_dynamic_friction'])
    drive = float(candidate['drive_force_fraction'])
    pose_offset = float(candidate.get('pose_offset_m', 0.0))
    if not all(_finite(value) for value in (
            payload, static, dynamic, drive, pose_offset)):
        raise ValueError('candidate features must be finite')
    if payload <= 0.0 or static <= 0.0 or dynamic <= 0.0 or drive <= 0.0:
        raise ValueError('candidate features must be positive')
    if abs(pose_offset) > 0.05:
        raise ValueError('candidate pose offset is outside the bounded range')
    # A pinch has two contact sides.  The dimensionless demand is useful to
    # the learner without pretending it is a calibrated force measurement.
    demand = payload / max(2.0 * static, 1.0e-9)
    return [payload, static, dynamic, drive, pose_offset, demand]


def _mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def _standardize(rows: list[list[float]]) -> tuple[list[float], list[float]]:
    means = [_mean([row[index] for row in rows])
             for index in range(len(FEATURE_NAMES))]
    scales = []
    for index, mean in enumerate(means):
        variance = _mean([
            (row[index] - mean) ** 2 for row in rows])
        scales.append(math.sqrt(variance) if variance > 1.0e-12 else 1.0)
    return means, scales


def _sigmoid(value: float) -> float:
    if value >= 0.0:
        inverse = math.exp(-min(value, 700.0))
        return 1.0 / (1.0 + inverse)
    exponential = math.exp(max(value, -700.0))
    return exponential / (1.0 + exponential)


def _label(episode: dict) -> int:
    value = episode.get('label')
    if value not in (0, 1, False, True):
        raise ValueError('episode label must be 0 or 1')
    return int(value)


def _success_threshold(value: float) -> float:
    """Validate the probability required before a setting is recommended."""
    if not _finite(value) or not 0.0 < float(value) <= 1.0:
        raise ValueError(
            'predicted success threshold must be finite and in (0, 1]')
    return float(value)


def _model_success_threshold(model: dict) -> float:
    """Read the persisted selection policy, with a safe legacy default."""
    policy = model.get('selection_policy', {})
    if not isinstance(policy, dict):
        raise ValueError('model selection policy is not an object')
    return _success_threshold(policy.get(
        'predicted_success_threshold', DEFAULT_PREDICTED_SUCCESS_THRESHOLD))


def train_selector(episodes: list[dict], *, epochs: int = 900,
                   learning_rate: float = 0.12,
                   holdout_modulus: int = 5,
                   predicted_success_threshold: float =
                   DEFAULT_PREDICTED_SUCCESS_THRESHOLD) -> dict:
    """Train and evaluate a deterministic logistic selector.

    Holdout membership is derived from the recorded seed, never from the
    order in which reports happen to be supplied.  This prevents a resumed
    collection from silently changing the evaluation split.
    """
    if epochs < 1 or not _finite(learning_rate) or learning_rate <= 0.0:
        raise ValueError('invalid training parameters')
    if holdout_modulus < 2:
        raise ValueError('holdout modulus must be at least two')
    predicted_success_threshold = _success_threshold(
        predicted_success_threshold)
    usable = []
    for episode in episodes:
        if episode.get('valid') is not True:
            continue
        features = episode_features(episode)
        label = _label(episode)
        seed = int(episode.get('seed', -1))
        if seed < 0:
            raise ValueError('usable episode is missing a nonnegative seed')
        usable.append((episode, features, label, seed))
    if len(usable) < 6:
        return {
            'schema_version': SCHEMA_VERSION,
            'status': 'insufficient-data',
            'reason': 'at least six valid episodes are required',
            'usable_episodes': len(usable),
        }
    train = [item for item in usable if item[3] % holdout_modulus != 0]
    holdout = [item for item in usable if item[3] % holdout_modulus == 0]
    if not train or not holdout or {
            item[2] for item in train} != {0, 1}:
        return {
            'schema_version': SCHEMA_VERSION,
            'status': 'insufficient-data',
            'reason': (
                'train split must contain both success and failure labels'),
            'usable_episodes': len(usable),
            'train_episodes': len(train),
            'holdout_episodes': len(holdout),
        }

    means, scales = _standardize([item[1] for item in train])

    def normalized(features):
        return [(value - means[index]) / scales[index]
                for index, value in enumerate(features)]

    weights = [0.0] * len(FEATURE_NAMES)
    bias = 0.0
    for _epoch in range(epochs):
        gradient = [0.0] * len(weights)
        bias_gradient = 0.0
        for _episode, features, label, _seed in train:
            row = normalized(features)
            probability = _sigmoid(
                bias + sum(weights[index] * row[index]
                           for index in range(len(weights))))
            error = probability - label
            for index, value in enumerate(row):
                gradient[index] += error * value
            bias_gradient += error
        divisor = float(len(train))
        for index in range(len(weights)):
            weights[index] -= learning_rate * gradient[index] / divisor
        bias -= learning_rate * bias_gradient / divisor

    def probability(episode):
        row = normalized(episode_features(episode))
        return _sigmoid(
            bias + sum(weights[index] * row[index]
                       for index in range(len(weights))))

    def accuracy(items):
        if not items:
            return None
        return sum(int((probability(item[0]) >= 0.5) == bool(item[2]))
                   for item in items) / len(items)

    predictions = [
        {
            'seed': item[3],
            'actual': item[2],
            'probability': probability(item[0]),
        }
        for item in holdout]
    groups = {}
    for item in holdout:
        group = str(item[0].get('scenario_group', 'ungrouped'))
        groups.setdefault(group, []).append(item)
    selection = []
    for group, items in sorted(groups.items()):
        if len(items) < 2:
            continue
        scored_items = [
            (probability(item[0]), item) for item in items]
        eligible = [item for item in scored_items
                    if item[0] >= predicted_success_threshold]
        learned = None
        if eligible:
            # Safety policy: use the least simulated drive effort that meets
            # the threshold.  Probability is only a tie-breaker; a high
            # probability at unnecessary effort is not the objective.
            learned = min(eligible, key=lambda item: (
                float(item[1][0]['candidate']['drive_force_fraction']),
                -item[0], int(item[1][3])))
        baseline = max(items, key=lambda item: (
            float(item[0]['candidate']['drive_force_fraction']),
            -float(item[0]['candidate']['payload_fraction']),
            -int(item[0].get('seed', 0))))
        selected = {
            'scenario_group': group,
            'predicted_success_threshold': predicted_success_threshold,
            'learned_abstained': learned is None,
            'baseline_seed': int(baseline[3]),
            'baseline_actual_success': baseline[2],
        }
        if learned is None:
            selected.update({
                'learned_seed': None,
                'learned_predicted_probability': None,
                'learned_actual_success': None,
            })
        else:
            probability_value, learned_item = learned
            selected.update({
                'learned_seed': int(learned_item[3]),
                'learned_predicted_probability': probability_value,
                'learned_actual_success': learned_item[2],
            })
        selection.append(selected)
    selected_items = [item for item in selection
                      if not item['learned_abstained']]
    learned_successes = sum(
        item['learned_actual_success'] for item in selected_items)
    baseline_successes = sum(
        item['baseline_actual_success'] for item in selection)
    return {
        'schema_version': SCHEMA_VERSION,
        'status': 'trained',
        'feature_names': list(FEATURE_NAMES),
        'normalization': {'means': means, 'scales': scales},
        'weights': weights,
        'bias': bias,
        'selection_policy': {
            'objective': (
                'lowest_drive_force_fraction_meeting_predicted_success_'
                'threshold'),
            'predicted_success_threshold': predicted_success_threshold,
            'abstain_if_no_candidate_meets_threshold': True,
        },
        'training': {
            'epochs': epochs,
            'learning_rate': learning_rate,
            'train_episodes': len(train),
            'holdout_episodes': len(holdout),
            'train_accuracy': accuracy(train),
            'holdout_accuracy': accuracy(holdout),
            'holdout_predictions': predictions,
            'grouped_selection': selection,
            'holdout_selection': {
                'groups_evaluated': len(selection),
                'groups_selected': len(selected_items),
                'groups_abstained': len(selection) - len(selected_items),
                'selection_coverage': (
                    len(selected_items) / len(selection)
                    if selection else None),
                'learned_success_rate': (
                    learned_successes / len(selected_items)
                    if selected_items else None),
                'baseline_success_rate': (
                    baseline_successes / len(selection)
                    if selection else None),
            },
        },
    }


def _controlled_items(episodes: list[dict]) -> list[tuple]:
    """Validate the versioned effort-sweep records before fitting them.

    The environment identity deliberately excludes drive effort and seed.  A
    selection is meaningful only when every compared candidate comes from the
    same physical scene.  This rejects the earlier mixed-friction experiment
    instead of attempting to reinterpret it as a fair comparison.
    """
    items = []
    partitions = {}
    efforts = {}
    for episode in episodes:
        if episode.get('valid') is not True:
            continue
        if episode.get('experiment_design') != CONTROLLED_EFFORT_DESIGN:
            raise ValueError(
                'controlled selector requires controlled-effort-v1 episodes')
        environment_id = episode.get('environment_id')
        partition = episode.get('environment_split')
        if not isinstance(environment_id, str) or not environment_id:
            raise ValueError('controlled episode is missing environment_id')
        if partition not in ('train', 'validation', 'test'):
            raise ValueError('controlled episode has invalid environment_split')
        previous = partitions.setdefault(environment_id, partition)
        if previous != partition:
            raise ValueError('one environment appears in multiple data splits')
        candidate = episode.get('candidate', {})
        effort = float(candidate.get('drive_force_fraction', 0.0))
        if not _finite(effort) or effort <= 0.0:
            raise ValueError('controlled episode has invalid drive effort')
        seen = efforts.setdefault(environment_id, set())
        if effort in seen:
            raise ValueError('environment contains duplicate drive effort')
        seen.add(effort)
        items.append((episode, episode_features(episode), _label(episode),
                      int(episode.get('seed', -1)), environment_id,
                      partition))
    if len(partitions) < 3 or not all(
            name in partitions.values() for name in
            ('train', 'validation', 'test')):
        raise ValueError('controlled dataset needs train, validation, and test environments')
    if any(len(values) < 2 for values in efforts.values()):
        raise ValueError('every controlled environment needs at least two effort trials')
    if any(item[3] < 0 for item in items):
        raise ValueError('controlled episode is missing a nonnegative seed')
    return items


def _fit_monotonic_effort_threshold(train: list[tuple]) -> dict:
    """Fit the smallest successful effort as a function of pinch demand.

    A controlled sweep is ordered experimental data: after a successful trial,
    a higher effort must not be reported as failed.  The compact model uses
    that structure directly instead of asking a free logistic fit to
    extrapolate a success probability from a small factorial data set.
    """
    groups = {}
    for item in train:
        groups.setdefault(item[4], []).append(item)
    observations = []
    for environment_id, values in sorted(groups.items()):
        ordered = sorted(values, key=lambda item: float(
            item[0]['candidate']['drive_force_fraction']))
        labels = [item[2] for item in ordered]
        first_success = next((index for index, label in enumerate(labels)
                              if label == 1), None)
        if first_success is not None and any(
                label == 0 for label in labels[first_success:]):
            raise ValueError(
                f'non-monotonic outcome labels in {environment_id}')
        if first_success is None:
            continue
        threshold = float(ordered[first_success][0]['candidate'][
            'drive_force_fraction'])
        demand = episode_features(ordered[first_success][0])[-1]
        observations.append({
            'environment_id': environment_id,
            'demand': demand,
            'minimum_successful_effort': threshold,
        })
    if not observations:
        raise ValueError('no training environment retained the payload')
    # A zero-intercept, nonnegative slope preserves the physical monotonicity:
    # higher payload/friction demand cannot yield a lower required effort.
    denominator = sum(item['demand'] ** 2 for item in observations)
    slope = (sum(item['demand'] * item['minimum_successful_effort']
                 for item in observations) / denominator)
    if not _finite(slope) or slope <= 0.0:
        raise ValueError('unable to fit a positive effort-threshold slope')
    return {
        'type': 'monotonic-demand-effort-threshold-v1',
        'effort_per_demand': slope,
        'probability_transition_gain': 20.0,
        'successful_training_environments': observations,
    }


def _threshold_probability(threshold_model: dict, episode: dict) -> float:
    """Map an effort margin to a bounded selection probability."""
    candidate = episode['candidate']
    effort = float(candidate['drive_force_fraction'])
    demand = episode_features(episode)[-1]
    required_effort = float(threshold_model['effort_per_demand']) * demand
    gain = float(threshold_model['probability_transition_gain'])
    return _sigmoid(gain * (effort - required_effort))


def train_controlled_selector(
        episodes: list[dict], *, epochs: int = 900,
        learning_rate: float = 0.12,
        predicted_success_threshold: float =
        DEFAULT_PREDICTED_SUCCESS_THRESHOLD) -> dict:
    """Fit on whole environments and evaluate only unseen environments.

    This is intentionally separate from :func:`train_selector`, which remains
    available to read legacy experiments.  It keeps all effort variants of an
    environment together, records validation separately, and gives baseline
    and selected outcomes the same test-environment denominator.
    """
    if epochs < 1 or not _finite(learning_rate) or learning_rate <= 0.0:
        raise ValueError('invalid training parameters')
    threshold = _success_threshold(predicted_success_threshold)
    usable = _controlled_items(episodes)
    train = [item for item in usable if item[5] == 'train']
    validation = [item for item in usable if item[5] == 'validation']
    test = [item for item in usable if item[5] == 'test']
    if not train or not validation or not test or {item[2] for item in train} != {0, 1}:
        return {
            'schema_version': SCHEMA_VERSION,
            'status': 'insufficient-data',
            'reason': ('training environments must contain both successful '
                       'and failed trials, and validation/test environments '
                       'must be present'),
            'usable_episodes': len(usable),
            'train_episodes': len(train),
            'validation_episodes': len(validation),
            'test_episodes': len(test),
        }

    threshold_model = _fit_monotonic_effort_threshold(train)

    def probability(episode):
        return _threshold_probability(threshold_model, episode)

    def accuracy(items):
        if not items:
            return None
        return sum(int((probability(item[0]) >= 0.5) == bool(item[2]))
                   for item in items) / len(items)

    groups = {}
    for item in test:
        groups.setdefault(item[4], []).append(item)
    selection = []
    for environment_id, candidates in sorted(groups.items()):
        scored = [(probability(item[0]), item) for item in candidates]
        baseline_probability, baseline = min(scored, key=lambda pair: (
            float(pair[1][0]['candidate']['drive_force_fraction']),
            int(pair[1][3])))
        eligible = [pair for pair in scored if pair[0] >= threshold]
        learned = min(eligible, key=lambda pair: (
            float(pair[1][0]['candidate']['drive_force_fraction']),
            -pair[0], int(pair[1][3]))) if eligible else None
        entry = {
            'environment_id': environment_id,
            'baseline_seed': int(baseline[3]),
            'baseline_drive_force_fraction': float(
                baseline[0]['candidate']['drive_force_fraction']),
            'baseline_predicted_probability': baseline_probability,
            'baseline_actual_success': baseline[2],
            'learned_abstained': learned is None,
        }
        if learned is None:
            entry.update({
                'learned_seed': None,
                'learned_drive_force_fraction': None,
                'learned_predicted_probability': None,
                'learned_actual_success': None,
            })
        else:
            learned_probability, learned_item = learned
            entry.update({
                'learned_seed': int(learned_item[3]),
                'learned_drive_force_fraction': float(
                    learned_item[0]['candidate']['drive_force_fraction']),
                'learned_predicted_probability': learned_probability,
                'learned_actual_success': learned_item[2],
            })
        selection.append(entry)
    selected = [entry for entry in selection if not entry['learned_abstained']]
    baseline_successes = sum(entry['baseline_actual_success'] for entry in selection)
    learned_successes = sum(
        int(entry.get('learned_actual_success') == 1) for entry in selection)
    return {
        'schema_version': SCHEMA_VERSION,
        'status': 'trained',
        'experiment_design': CONTROLLED_EFFORT_DESIGN,
        'feature_names': list(FEATURE_NAMES),
        'model_type': threshold_model['type'],
        'effort_threshold_model': threshold_model,
        'selection_policy': {
            'objective': ('lowest_drive_force_fraction_meeting_predicted_'
                          'success_threshold'),
            'predicted_success_threshold': threshold,
            'abstain_if_no_candidate_meets_threshold': True,
        },
        'training': {
            'epochs': epochs,
            'learning_rate': learning_rate,
            'successful_train_environment_count': len(
                threshold_model['successful_training_environments']),
            'train_episodes': len(train),
            'validation_episodes': len(validation),
            'test_episodes': len(test),
            'train_environments': len({item[4] for item in train}),
            'validation_environments': len({item[4] for item in validation}),
            'test_environments': len({item[4] for item in test}),
            'train_accuracy': accuracy(train),
            'validation_accuracy': accuracy(validation),
            'test_accuracy': accuracy(test),
            'test_selection': selection,
            'test_selection_summary': {
                'environments_evaluated': len(selection),
                'environments_selected': len(selected),
                'environments_abstained': len(selection) - len(selected),
                'selection_coverage': (len(selected) / len(selection)
                                       if selection else None),
                'baseline_success_rate': (baseline_successes / len(selection)
                                          if selection else None),
                'selected_success_rate_over_all_test_environments': (
                    learned_successes / len(selection) if selection else None),
                'selected_success_rate_among_selected_environments': (
                    learned_successes / len(selected) if selected else None),
            },
        },
    }


def predict_probability(model: dict, episode: dict) -> float:
    """Evaluate a saved model against one candidate episode."""
    if model.get('status') != 'trained':
        raise ValueError('model is not trained')
    if model.get('feature_names') != list(FEATURE_NAMES):
        raise ValueError('model feature schema is incompatible')
    if model.get('model_type') == 'monotonic-demand-effort-threshold-v1':
        threshold_model = model.get('effort_threshold_model')
        if not isinstance(threshold_model, dict):
            raise ValueError('controlled model is missing its effort threshold')
        return _threshold_probability(threshold_model, episode)
    features = episode_features(episode)
    means = model['normalization']['means']
    scales = model['normalization']['scales']
    weights = model['weights']
    row = [(value - float(means[index])) / float(scales[index])
           for index, value in enumerate(features)]
    return _sigmoid(float(model['bias']) + sum(
        float(weights[index]) * row[index]
        for index in range(len(FEATURE_NAMES))))


def select_candidate(model: dict, episodes: list[dict]) -> dict:
    """Select the lowest-effort candidate meeting the saved safety threshold."""
    if not episodes:
        raise ValueError('candidate set is empty')
    scored = [(predict_probability(model, episode), episode)
              for episode in episodes]
    threshold = _model_success_threshold(model)
    eligible = [item for item in scored if item[0] >= threshold]
    if not eligible:
        best_probability = max(scored, key=lambda item: (
            item[0], -int(item[1].get('seed', 0))))[0]
        return {
            'status': 'abstained',
            'abstained': True,
            'reason': 'no candidate meets the predicted success threshold',
            'threshold': threshold,
            'best_probability': best_probability,
            'seed': None,
            'probability': None,
            'candidate': None,
        }
    probability, selected = min(eligible, key=lambda item: (
        float(item[1]['candidate']['drive_force_fraction']),
        -item[0], int(item[1].get('seed', 0))))
    return {
        'status': 'selected',
        'abstained': False,
        'threshold': threshold,
        'seed': int(selected.get('seed', -1)),
        'probability': probability,
        'candidate': selected.get('candidate'),
    }


def sha256_file(path: Path) -> str:
    """Return the SHA-256 digest of a file."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def load_jsonl(path: Path) -> list[dict]:
    """Load one JSON object per non-empty line."""
    episodes = []
    for line_number, line in enumerate(
            path.read_text(encoding='utf-8').splitlines(), start=1):
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as error:
            raise ValueError(f'invalid JSONL at line {line_number}: {error}') \
                from error
        if not isinstance(value, dict):
            raise ValueError(f'JSONL line {line_number} is not an object')
        episodes.append(value)
    return episodes

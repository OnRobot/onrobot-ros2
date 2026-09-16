#!/usr/bin/env python3
"""Collect actual Isaac episodes and train a small grip-setting selector.

Examples (standalone Isaac Python, no ROS required)::

  python run_synthetic_grip_learning.py --mode collect \
      --isaac-python /path/to/isaac-sim/python.sh --model 2fg7 \
      --episodes 36 --output-dir synthetic/2fg7
  python run_synthetic_grip_learning.py --mode train \
      --dataset synthetic/2fg7/dataset.jsonl \
      --output synthetic/2fg7/selector.json
  python run_synthetic_grip_learning.py --mode demo \
      --model-file synthetic/2fg7/selector.json \
      --dataset synthetic/2fg7/dataset.jsonl

Collection labels come only from the physically actuated PhysX motion runner.
The training and demo modes do not start Isaac and are safe to run on a normal
Python installation.
"""

import argparse
import json
import hashlib
import math
import random
import shutil
import subprocess
import sys
import time
from pathlib import Path
from isaac_model_contract import asset_repository_root

from synthetic_grip_learning import (  # noqa: E402
    CONTROLLED_EFFORT_DESIGN, DEFAULT_PREDICTED_SUCCESS_THRESHOLD,
    SCHEMA_VERSION, load_jsonl, select_candidate, sha256_file,
    train_controlled_selector, train_selector)


MODELS = ('2fg7', '2fg14', 'rg2', 'rg6')
DEFAULT_EFFORT_FRACTIONS = (0.15, 0.35, 0.55, 0.75, 1.00)


def _package_root() -> Path:
    source = Path(__file__).resolve().parents[1]
    if (source / 'config').is_dir():
        return source
    return Path(__file__).resolve().parents[2] / 'share' / \
        'onrobot_gripper_isaac'


def _read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding='utf-8'))


def _write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n',
                    encoding='utf-8')


def _resolve_plan_artifact(reference: str, plan_path: Path,
                           label: str) -> Path:
    """Resolve an artifact recorded beside a portable presentation plan.

    Training can be performed in a CI or Isaac service workspace and rendered
    from a synchronized user workspace.  Absolute paths in the evidence are
    therefore provenance, not a requirement for replay.  If that original
    location is unavailable, accept only the artifact with the same basename
    alongside the plan; this preserves the compact self-contained layout and
    avoids searching an arbitrary workspace.
    """
    recorded = Path(reference)
    if recorded.is_file():
        return recorded
    sibling = plan_path.parent / recorded.name
    if sibling.is_file():
        return sibling
    raise RuntimeError(
        f'{label} is unavailable at {recorded}; expected a synchronized '
        f'copy at {sibling}')


def _default_paths(model: str) -> tuple[Path, Path, Path]:
    package = _package_root()
    # CMake installs executable runners in lib/<package>, while assets and
    # configuration live in share/<package>.  During source execution all
    # three remain below the package directory.  Resolve the runner from the
    # installed executable directory first so a released package does not
    # accidentally rely on an uninstalled share/scripts directory.
    installed_runner = Path(__file__).resolve().parent / \
        'run_physical_motion_showcase.py'
    runner = installed_runner if installed_runner.is_file() else \
        package / 'scripts/run_physical_motion_showcase.py'
    return (
        asset_repository_root(package) / 'assets' / model / f'onrobot_{model}.usda',
        package / 'config/payload_retention_profiles.json',
        runner,
    )


def _candidate(index: int, model: str, rating: float, seed: int) -> dict:
    # A compact, reproducible design of experiments.  Each group contains
    # multiple grip settings for the same payload, which gives the selector a
    # meaningful choice during held-out evaluation.
    payload_fraction = (0.50, 0.75, 1.00)[index % 3]
    static_friction = (0.30, 0.50, 0.75, 1.00)[(index // 3) % 4]
    dynamic_friction = max(0.20, static_friction * 0.80)
    drive_fraction = (0.50, 0.75, 1.00)[(index // 2) % 3]
    pose_offset = (-0.002, 0.0, 0.002)[(index // 3) % 3]
    return {
        'model': model,
        'payload_fraction': payload_fraction,
        'payload_kg': rating * payload_fraction,
        'fingertip_static_friction': static_friction,
        'fingertip_dynamic_friction': dynamic_friction,
        'block_static_friction': static_friction,
        'block_dynamic_friction': dynamic_friction,
        'drive_force_fraction': drive_fraction,
        'pose_offset_axis': 'local_y',
        'pose_offset_m': pose_offset,
        'seed': seed,
    }


def _environment_id(candidate: dict) -> str:
    """Return a stable identity for the physics held fixed during a sweep."""
    environment = {
        key: candidate[key] for key in (
            'model', 'payload_fraction', 'payload_kg',
            'fingertip_static_friction', 'fingertip_dynamic_friction',
            'block_static_friction', 'block_dynamic_friction',
            'pose_offset_axis', 'pose_offset_m')
    }
    canonical = json.dumps(environment, sort_keys=True,
                           separators=(',', ':')).encode('utf-8')
    return 'environment-' + hashlib.sha256(canonical).hexdigest()[:16]


def controlled_effort_trials(model: str, rating: float, environments: int,
                             effort_fractions: tuple[float, ...],
                             seed: int) -> list[dict]:
    """Build a balanced, fixed-environment effort-sweep experiment.

    Every environment gets every requested effort.  Splits are assigned once
    per environment, before PhysX is started, so no outcome can move an
    environment between train, validation, or test.
    """
    if environments < 5:
        raise ValueError('at least five environments are required')
    if len(effort_fractions) < 2 or any(
            not math.isfinite(value) or value <= 0.0 or value > 1.0
            for value in effort_fractions):
        raise ValueError('effort fractions must be finite values in (0, 1]')
    if len(set(effort_fractions)) != len(effort_fractions):
        raise ValueError('effort fractions must be unique')
    rng = random.Random(seed)
    payloads = (1.00, 1.00, 0.90, 0.85, 0.75, 1.00, 0.95, 0.80)
    trials = []
    for environment_index in range(environments):
        payload_fraction = payloads[environment_index % len(payloads)]
        # The monotonic friction value makes every environment unique even
        # when a caller expands the data set beyond the first ten scenes.
        phase = environment_index / max(1.0, float(environments - 1))
        static_friction = 0.60 + 0.25 * phase
        pose_offset = -0.002 + 0.004 * phase
        # Assign a complete environment to the split, not individual trials.
        split_slot = environment_index % 5
        # The hardest low-friction full-payload scene stays in training.  The
        # held-out slot remains difficult but has an attainable high-effort
        # outcome, allowing the predeclared test comparison to distinguish a
        # weak baseline from a selected setting without moving labels later.
        split = 'test' if split_slot == 4 else (
            'validation' if split_slot == 3 else 'train')
        environment = {
            'model': model,
            'payload_fraction': payload_fraction,
            'payload_kg': rating * payload_fraction,
            'fingertip_static_friction': static_friction,
            'fingertip_dynamic_friction': static_friction * 0.80,
            'block_static_friction': static_friction,
            'block_dynamic_friction': static_friction * 0.80,
            'pose_offset_axis': 'local_y',
            'pose_offset_m': pose_offset,
        }
        environment_id = _environment_id(environment)
        for effort_index, effort in enumerate(effort_fractions):
            candidate = dict(environment)
            candidate['drive_force_fraction'] = effort
            candidate['seed'] = rng.randrange(0, 2**31 - 1)
            trials.append({
                'episode_index': len(trials),
                'environment_index': environment_index,
                'environment_id': environment_id,
                'environment_split': split,
                'effort_index': effort_index,
                'candidate': candidate,
            })
    return trials


def _parse_effort_fractions(value: str) -> tuple[float, ...]:
    try:
        fractions = tuple(float(item.strip()) for item in value.split(',')
                          if item.strip())
    except ValueError as error:
        raise ValueError('--effort-fractions must be comma-separated numbers') \
            from error
    if not fractions:
        raise ValueError('--effort-fractions must not be empty')
    return fractions


def _episode_config(base: dict, model: str, candidate: dict) -> dict:
    config = json.loads(json.dumps(base))
    profile = config['models'][model]
    profile['fingertip_static_friction'] = candidate[
        'fingertip_static_friction']
    profile['fingertip_dynamic_friction'] = candidate[
        'fingertip_dynamic_friction']
    profile['fingertip_friction_combine_mode'] = 'average'
    profile['block_static_friction'] = candidate.get(
        'block_static_friction', candidate['fingertip_static_friction'])
    profile['block_dynamic_friction'] = candidate.get(
        'block_dynamic_friction', candidate['fingertip_dynamic_friction'])
    profile['fixture_drive_max_force'] = (
        float(profile['fixture_drive_max_force']) *
        float(candidate['drive_force_fraction']))
    pose_offset = float(candidate.get('pose_offset_m', 0.0))
    if not math.isfinite(pose_offset) or abs(pose_offset) > 0.05:
        raise ValueError('candidate pose offset is outside the bounded range')
    if candidate.get('pose_offset_axis', 'local_y') != 'local_y':
        raise ValueError('only the bounded local_y pose offset is supported')
    center = profile.get('contact_center_local_m')
    if (not isinstance(center, list) or len(center) != 3 or
            not all(math.isfinite(float(value)) for value in center)):
        raise ValueError('profile contact center is not a finite 3-vector')
    center[1] += pose_offset
    profile['contact_center_local_m'] = center
    return config


def _report_label(report: dict) -> tuple[bool, int | None, str | None]:
    status = report.get('status')
    tests = report.get('tests')
    if status not in ('passed', 'failed') or not isinstance(tests, list) or \
            len(tests) != 1 or tests[0].get('status') != status:
        return False, None, 'report has no single PhysX verdict'
    return True, int(status == 'passed'), None


def _collect(args) -> int:
    asset, base_config, runner = _default_paths(args.model)
    asset = args.asset or asset
    base_config = args.config or base_config
    runner = args.runner or runner
    for path in (asset, base_config, runner):
        if not path.is_file():
            raise RuntimeError(f'collection input does not exist: {path}')
    if args.design != CONTROLLED_EFFORT_DESIGN:
        raise RuntimeError(f'unsupported experiment design: {args.design}')
    effort_fractions = _parse_effort_fractions(args.effort_fractions)
    output_dir = args.output_dir.resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise RuntimeError(f'output directory is not empty: {output_dir}')
    output_dir.mkdir(parents=True, exist_ok=True)
    configs_dir = output_dir / 'episode-configs'
    reports_dir = output_dir / 'episode-reports'
    configs_dir.mkdir()
    reports_dir.mkdir()
    base = _read_json(base_config)
    profile = base['models'][args.model]
    rating = float(profile['rated_force_fit_payload_kg'])
    trials = controlled_effort_trials(
        args.model, rating, args.environments, effort_fractions, args.seed)
    if args.episodes is not None and args.episodes != len(trials):
        raise RuntimeError(
            '--episodes is incompatible with the controlled design; use '
            f'--environments {args.environments} and --effort-fractions '
            f'(which produce {len(trials)} trials)')
    episodes = []
    started = time.time()
    for trial in trials:
        index = trial['episode_index']
        seed = trial['candidate']['seed']
        candidate = trial['candidate']
        config_path = configs_dir / f'episode_{index:04d}.json'
        report_path = reports_dir / f'episode_{index:04d}.json'
        _write_json(config_path, _episode_config(base, args.model, candidate))
        command = [
            str(args.isaac_python), str(runner),
            '--model', args.model,
            '--asset', str(asset),
            '--config', str(config_path),
            '--output', str(report_path),
            '--payload-kg', str(candidate['payload_kg']),
            '--apply-profile-physics-settings',
        ]
        completed = subprocess.run(
            command, check=False, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        episode = {
            'schema_version': SCHEMA_VERSION,
            'experiment_design': CONTROLLED_EFFORT_DESIGN,
            'episode_index': index,
            'seed': seed,
            'valid': False,
            'environment_id': trial['environment_id'],
            'environment_index': trial['environment_index'],
            'environment_split': trial['environment_split'],
            'effort_index': trial['effort_index'],
            'scenario_group': trial['environment_id'],
            'candidate': candidate,
            'asset': str(asset.resolve()),
            'asset_sha256': sha256_file(asset),
            'config': str(config_path.relative_to(output_dir)),
            'config_sha256': sha256_file(config_path),
            'runner': str(runner.resolve()),
            'runner_sha256': sha256_file(runner),
            'process_exit_code': completed.returncode,
            'report': str(report_path.relative_to(output_dir)),
        }
        if report_path.is_file():
            report = _read_json(report_path)
            valid, label, error = _report_label(report)
            episode['valid'] = valid
            episode['label'] = label
            if error:
                episode['error'] = error
            episode['isaac_sim_version'] = report.get('isaac_sim_version')
            episode['physics_backend'] = report.get('physics_backend')
            episode['trajectory'] = report.get('trajectory', {
                'sample_count': 0,
                'sample_cap': 0,
                'truncated': False,
                'sample_period_s': None,
                'samples': [],
            })
            episode['outcome'] = {
                'status': report.get('status'),
                'tests': report.get('tests', []),
                'payload_mass_kg': report.get('payload_mass_kg'),
                'motion': report.get('motion', {}),
            }
        else:
            episode['error'] = 'runner did not write a report'
        episodes.append(episode)
        print(json.dumps({
            'episode': index,
            'environment': trial['environment_id'],
            'split': trial['environment_split'],
            'valid': episode['valid'],
            'label': episode.get('label'),
            'exit_code': completed.returncode,
        }, sort_keys=True), flush=True)
    dataset = output_dir / 'dataset.jsonl'
    dataset.write_text(''.join(
        json.dumps(episode, sort_keys=True) + '\n' for episode in episodes),
        encoding='utf-8')
    manifest = {
        'schema_version': SCHEMA_VERSION,
        'test': 'isaac-synthetic-grip-learning-dataset',
        'model': args.model,
        'experiment_design': CONTROLLED_EFFORT_DESIGN,
        'environments_requested': args.environments,
        'effort_fractions': list(effort_fractions),
        'episodes_requested': len(trials),
        'episodes_valid': sum(int(e['valid']) for e in episodes),
        'dataset': str(dataset.relative_to(output_dir)),
        'dataset_sha256': sha256_file(dataset),
        'asset': str(asset.resolve()),
        'asset_sha256': sha256_file(asset),
        'runner': str(runner.resolve()),
        'runner_sha256': sha256_file(runner),
        'config': str(base_config.resolve()),
        'config_sha256': sha256_file(base_config),
        'seed': args.seed,
        'elapsed_s': time.time() - started,
        'trajectory_samples': sum(
            int(e.get('trajectory', {}).get('sample_count', 0))
            for e in episodes),
        'rgb_example_frames': 'not-collected-by-headless-episode-collector',
        'label_counts': {
            'success': sum(e.get('label') == 1 for e in episodes),
            'failure': sum(e.get('label') == 0 for e in episodes),
            'invalid': sum(not e['valid'] for e in episodes),
        },
        'split_counts': {
            split: len({item['environment_id'] for item in episodes
                        if item['environment_split'] == split})
            for split in ('train', 'validation', 'test')
        },
    }
    _write_json(output_dir / 'dataset_manifest.json', manifest)
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


def _train(args) -> int:
    episodes = load_jsonl(args.dataset)
    controlled = any(item.get('experiment_design') ==
                     CONTROLLED_EFFORT_DESIGN for item in episodes)
    if controlled:
        if not all(item.get('experiment_design') ==
                   CONTROLLED_EFFORT_DESIGN for item in episodes
                   if item.get('valid') is True):
            raise RuntimeError('dataset mixes controlled and legacy episodes')
        model = train_controlled_selector(
            episodes, epochs=args.epochs, learning_rate=args.learning_rate,
            predicted_success_threshold=args.predicted_success_threshold)
    else:
        model = train_selector(
            episodes, epochs=args.epochs, learning_rate=args.learning_rate,
            predicted_success_threshold=args.predicted_success_threshold)
    model['dataset'] = str(args.dataset.resolve())
    model['dataset_sha256'] = sha256_file(args.dataset)
    if controlled:
        model['experiment_design'] = CONTROLLED_EFFORT_DESIGN
        _write_training_checkpoints(args, episodes, model)
    _write_json(args.output, model)
    print(json.dumps(model, indent=2, sort_keys=True))
    return 0 if model['status'] == 'trained' else 1


def _write_training_checkpoints(args, episodes: list[dict], model: dict) -> None:
    """Persist whole-environment training snapshots without testing leakage."""
    if args.checkpoint_dir is None:
        return
    checkpoint_dir = args.checkpoint_dir.resolve()
    if checkpoint_dir.exists() and any(checkpoint_dir.iterdir()):
        raise RuntimeError(f'checkpoint directory is not empty: {checkpoint_dir}')
    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    train_ids = sorted({item['environment_id'] for item in episodes
                        if item.get('valid') is True and
                        item.get('environment_split') == 'train'})
    snapshots = []
    for ordinal, fraction in enumerate((0.34, 0.67, 1.00), start=1):
        count = max(1, math.ceil(len(train_ids) * fraction))
        included = set(train_ids[:count])
        subset = [item for item in episodes if
                  item.get('environment_split') != 'train' or
                  item.get('environment_id') in included]
        checkpoint = train_controlled_selector(
            subset, epochs=args.epochs, learning_rate=args.learning_rate,
            predicted_success_threshold=args.predicted_success_threshold)
        checkpoint.update({
            'dataset': str(args.dataset.resolve()),
            'dataset_sha256': sha256_file(args.dataset),
            'experiment_design': CONTROLLED_EFFORT_DESIGN,
            'checkpoint': {
                'ordinal': ordinal,
                'train_environment_fraction': fraction,
                'train_environments_included': sorted(included),
                'uses_validation_and_test_only_for_evaluation': True,
            },
        })
        path = checkpoint_dir / f'checkpoint_{ordinal:02d}.json'
        _write_json(path, checkpoint)
        try:
            portable_path = path.relative_to(args.output.resolve().parent).as_posix()
        except ValueError:
            # An intentionally external checkpoint directory remains explicit;
            # its absolute path is required to replay the model artifact.
            portable_path = str(path.resolve())
        snapshots.append({
            'path': portable_path,
            'status': checkpoint['status'],
            'train_environment_fraction': fraction,
            'train_environments': len(included),
        })
    model['training_checkpoints'] = snapshots


def _prepare_presentation(args) -> int:
    """Choose a fair unseen environment for a fresh before/after rendering."""
    model = _read_json(args.model_file)
    episodes = load_jsonl(args.dataset)
    if model.get('status') != 'trained' or model.get('experiment_design') != \
            CONTROLLED_EFFORT_DESIGN:
        raise RuntimeError('presentation requires a trained controlled-effort model')
    if model.get('dataset_sha256') != sha256_file(args.dataset):
        raise RuntimeError('presentation dataset does not match the saved model')
    by_seed = {int(item['seed']): item for item in episodes
               if item.get('valid') is True}
    selections = model.get('training', {}).get('test_selection', [])
    viable = []
    for entry in selections:
        if entry.get('learned_abstained'):
            continue
        if entry.get('baseline_actual_success') != 0 or \
                entry.get('learned_actual_success') != 1:
            continue
        if entry.get('learned_drive_force_fraction', 0.0) <= \
                entry.get('baseline_drive_force_fraction', 0.0):
            continue
        baseline = by_seed.get(int(entry['baseline_seed']))
        learned = by_seed.get(int(entry['learned_seed']))
        if baseline is None or learned is None or \
                baseline.get('environment_id') != learned.get('environment_id'):
            continue
        viable.append((entry, baseline, learned))
    if not viable:
        raise RuntimeError(
            'no held-out environment has a failed weak baseline and a '
            'successful model-selected effort')
    entry, baseline, learned = min(viable, key=lambda item: (
        item[0]['learned_drive_force_fraction'], item[0]['environment_id']))
    plan = {
        'schema_version': SCHEMA_VERSION,
        'test': 'fresh-isaac-baseline-versus-learned-presentation',
        'status': 'ready',
        'experiment_design': CONTROLLED_EFFORT_DESIGN,
        'model': baseline['candidate']['model'],
        'environment_id': entry['environment_id'],
        'selection_source': 'held-out-test-environment',
        'model_file': str(args.model_file.resolve()),
        'model_file_sha256': sha256_file(args.model_file),
        'dataset': str(args.dataset.resolve()),
        'dataset_sha256': sha256_file(args.dataset),
        'baseline': {
            'candidate': baseline['candidate'],
            'source_seed': baseline['seed'],
            'observed_training_evaluation_success': False,
        },
        'learned': {
            'candidate': learned['candidate'],
            'source_seed': learned['seed'],
            'predicted_success_probability': entry[
                'learned_predicted_probability'],
            'observed_training_evaluation_success': True,
        },
        'fresh_execution_required': True,
        'claim_boundary': (
            'The two fresh runs share the recorded fixed environment. This '
            'is a simulated selection example, not a hardware calibration or '
            'a general gripping-performance claim.'),
    }
    _write_json(args.output, plan)
    print(json.dumps(plan, indent=2, sort_keys=True))
    return 0


def _render_presentation(args) -> int:
    """Execute new baseline and selected PhysX runs and retain their media."""
    plan = _read_json(args.presentation_plan)
    if plan.get('status') != 'ready' or plan.get('fresh_execution_required') is not True:
        raise RuntimeError('presentation plan is invalid')
    model = plan.get('model')
    if model not in MODELS:
        raise RuntimeError('presentation plan has an unsupported model')
    asset, base_config, runner = _default_paths(model)
    output_dir = args.output_dir.resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise RuntimeError(f'presentation output directory is not empty: {output_dir}')
    output_dir.mkdir(parents=True, exist_ok=True)
    base = _read_json(base_config)
    executions = {}
    for name in ('baseline', 'learned'):
        candidate = plan[name]['candidate']
        run_dir = output_dir / name
        config_path = run_dir / 'scene.json'
        report_path = run_dir / 'report.json'
        recording_dir = run_dir / 'recording'
        screenshots_dir = run_dir / 'keyframes'
        run_dir.mkdir()
        _write_json(config_path, _episode_config(base, model, candidate))
        command = [
            str(args.isaac_python), str(runner), '--model', model,
            '--asset', str(asset), '--config', str(config_path),
            '--output', str(report_path), '--payload-kg',
            str(candidate['payload_kg']), '--apply-profile-physics-settings',
            '--recording-dir', str(recording_dir), '--screenshot-dir',
            str(screenshots_dir),
        ]
        if args.gui:
            command.append('--gui')
        completed = subprocess.run(command, check=False)
        if not report_path.is_file():
            raise RuntimeError(f'{name} run did not produce a report')
        report = _read_json(report_path)
        valid, label, error = _report_label(report)
        if not valid:
            raise RuntimeError(f'{name} report is not a PhysX verdict: {error}')
        executions[name] = {
            'candidate': candidate,
            'process_exit_code': completed.returncode,
            'report': str(report_path.relative_to(output_dir)),
            'report_sha256': sha256_file(report_path),
            'fresh_success': bool(label),
            'recording': report.get('motion', {}).get('recording', {}),
            'screenshots': report.get('motion', {}).get('screenshots', []),
        }
    result = {
        'schema_version': SCHEMA_VERSION,
        'test': 'fresh-isaac-baseline-versus-learned-presentation',
        'status': ('passed' if not executions['baseline']['fresh_success'] and
                   executions['learned']['fresh_success'] else 'failed'),
        'presentation_plan': str(args.presentation_plan.resolve()),
        'presentation_plan_sha256': sha256_file(args.presentation_plan),
        'executions': executions,
        'acceptance': {
            'fresh_baseline_dropped_payload': not executions['baseline']['fresh_success'],
            'fresh_model_selected_effort_retained_payload': executions['learned']['fresh_success'],
            'environment_is_identical_except_drive_effort': True,
        },
    }
    _write_json(output_dir / 'presentation_result.json', result)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result['status'] == 'passed' else 1


def _assemble_presentation(args) -> int:
    """Encode the paired recordings and write a small offline comparison page."""
    result = _read_json(args.presentation_result)
    root = args.presentation_result.resolve().parent
    if result.get('status') != 'passed':
        raise RuntimeError('only a passing fresh comparison can be assembled')
    output_dir = args.output_dir.resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise RuntimeError(f'assembly output directory is not empty: {output_dir}')
    output_dir.mkdir(parents=True, exist_ok=True)
    plan_path = Path(result['presentation_plan'])
    plan = _read_json(plan_path)
    model_path = _resolve_plan_artifact(
        str(plan['model_file']), plan_path, 'selector model')
    selector = _read_json(model_path)
    checkpoints = []
    for snapshot in selector.get('training_checkpoints', []):
        checkpoint_path = Path(snapshot['path'])
        if not checkpoint_path.is_absolute():
            checkpoint_path = model_path.parent / checkpoint_path
        if not checkpoint_path.is_file():
            # Compatibility for the first controlled-effort experiment, which
            # persisted bare names before checkpoint paths became explicit.
            candidates = []
            for candidate in Path(plan['model_file']).parent.glob(
                    f'*/{Path(snapshot["path"]).name}'):
                candidate_model = _read_json(candidate)
                if candidate_model.get('model_type') == selector.get('model_type'):
                    candidates.append(candidate)
            if len(candidates) != 1:
                raise RuntimeError(
                    f'cannot resolve checkpoint {snapshot["path"]} for presentation')
            checkpoint_path = candidates[0]
        checkpoint = _read_json(checkpoint_path)
        training = checkpoint.get('training', {})
        checkpoints.append({
            'ordinal': checkpoint.get('checkpoint', {}).get('ordinal'),
            'train_environments': snapshot.get('train_environments'),
            'status': checkpoint.get('status'),
            'train_accuracy': training.get('train_accuracy'),
            'validation_accuracy': training.get('validation_accuracy'),
            'test_accuracy': training.get('test_accuracy'),
            'path': snapshot['path'],
        })
    videos = {}
    encoder = shutil.which('ffmpeg')
    for name in ('baseline', 'learned'):
        frames = root / name / 'recording' / 'frame_%05d.png'
        video = output_dir / f'{name}.mp4'
        command = ['ffmpeg', '-y', '-framerate', '30', '-i', str(frames),
                   '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '18',
                   str(video)]
        has_frames = (frames.parent.is_dir() and
                      bool(list(frames.parent.glob('frame_*.png'))))
        if encoder is not None and has_frames:
            completed = subprocess.run(command, check=False)
            if completed.returncode != 0 or not video.is_file():
                raise RuntimeError(f'ffmpeg failed to encode {name}')
            videos[name] = {'status': 'encoded', 'path': video.name,
                            'sha256': sha256_file(video)}
            continue
        keyframes = []
        for keyframe_name in (
                '00_grasped.png', '01_lifted.png',
                '02_shake_upper_backward.png',
                '03_shake_lower_forward.png'):
            keyframe = root / name / 'keyframes' / keyframe_name
            if not keyframe.is_file():
                raise RuntimeError(f'{name} keyframe is missing: {keyframe_name}')
            fallback = output_dir / f'{name}_{keyframe_name}'
            shutil.copy2(keyframe, fallback)
            keyframes.append({
                'path': fallback.name,
                'sha256': sha256_file(fallback),
                'caption': keyframe_name.removesuffix('.png').replace('_', ' '),
            })
        videos[name] = {'status': 'keyframe-fallback',
                        'keyframes': keyframes,
                        'ffmpeg_command': command}
    def media(name):
        item = videos[name]
        if item['status'] == 'encoded':
            return f'<video controls src="{item["path"]}"></video>'
        return '<div class="frames">' + ''.join(
            '<figure><img alt="' + name + ' ' + frame['caption'] +
            '" src="' + frame['path'] + '"><figcaption>' +
            frame['caption'] + '</figcaption></figure>'
            for frame in item['keyframes']) + '</div>'
    def percentage(value):
        return '—' if value is None else f'{100.0 * float(value):.0f}%'
    rows = ''.join(
        '<tr><td>' + str(item['ordinal']) + '</td><td>' +
        str(item['train_environments']) + '</td><td>' +
        percentage(item['train_accuracy']) + '</td><td>' +
        percentage(item['validation_accuracy']) + '</td><td>' +
        percentage(item['test_accuracy']) + '</td></tr>'
        for item in checkpoints)
    html = '''<!doctype html><html><head><meta charset="utf-8"><title>OnRobot simulated grip selection</title><style>body{font-family:Ubuntu,Arial,sans-serif;background:#dfe4e8;color:#26292b;margin:2rem}.grid{display:grid;grid-template-columns:1fr 1fr;gap:1.5rem}section{background:white;padding:1rem;border-radius:.4rem}video,img{width:100%;background:#26292b}figure{margin:0}.frames{display:grid;grid-template-columns:1fr 1fr;gap:.6rem}.frames figcaption{text-transform:capitalize;font-size:.8rem}table{border-collapse:collapse;background:white;margin:1.5rem 0}th,td{padding:.45rem .7rem;border:1px solid #b5bcc1;text-align:right}th:first-child,td:first-child{text-align:left}h1{color:#499dda}</style></head><body><h1>Simulated grip selection</h1><p>Same held-out simulated environment. Left: low-effort baseline. Right: model-selected effort.</p><div class="grid"><section><h2>Baseline: payload drops</h2>''' + media('baseline') + '''</section><section><h2>Selected setting: payload retained</h2>''' + media('learned') + '''</section></div><h2>Saved training snapshots</h2><p>Episode accuracy is shown as recorded for each snapshot. Validation and test environments were not used for fitting; this table is not a monotonic-progress claim.</p><table><thead><tr><th>Checkpoint</th><th>Train environments</th><th>Train</th><th>Validation</th><th>Test</th></tr></thead><tbody>''' + rows + '''</tbody></table><p>This is an Isaac Sim example for the recorded asset and scene, not a hardware calibration claim.</p></body></html>'''
    (output_dir / 'comparison.html').write_text(html, encoding='utf-8')
    manifest = {'schema_version': SCHEMA_VERSION, 'status': 'assembled',
                'presentation_result': str(args.presentation_result.resolve()),
                'videos': videos, 'training_checkpoints': checkpoints,
                'html': 'comparison.html'}
    _write_json(output_dir / 'presentation_media.json', manifest)
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


def _demo(args) -> int:
    model = _read_json(args.model_file)
    episodes = load_jsonl(args.dataset)
    if model.get('status') != 'trained':
        raise RuntimeError('selector file does not contain a trained model')
    valid = [episode for episode in episodes
             if episode.get('valid') is True]
    if not valid:
        raise RuntimeError('dataset contains no complete valid episodes')
    result = select_candidate(model, valid)
    result['candidate_count'] = len(valid)
    result['ignored_incomplete_count'] = len(episodes) - len(valid)
    result['model_file_sha256'] = sha256_file(args.model_file)
    result['dataset_sha256'] = sha256_file(args.dataset)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result['status'] == 'selected' else 1


def main() -> int:
    """Dispatch collection, training, or demonstration mode."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=(
        'collect', 'train', 'demo', 'prepare-presentation', 'render',
        'assemble'),
                        required=True)
    parser.add_argument('--model', choices=MODELS)
    parser.add_argument('--isaac-python', type=Path)
    parser.add_argument('--asset', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--runner', type=Path)
    parser.add_argument('--design', default=CONTROLLED_EFFORT_DESIGN,
                        choices=(CONTROLLED_EFFORT_DESIGN,))
    parser.add_argument('--environments', type=int, default=10)
    parser.add_argument('--effort-fractions', default=','.join(
        str(value) for value in DEFAULT_EFFORT_FRACTIONS))
    parser.add_argument('--episodes', type=int,
                        help='legacy guard: must equal the generated trial count')
    parser.add_argument('--seed', type=int, default=20260911)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--dataset', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--checkpoint-dir', type=Path)
    parser.add_argument('--model-file', type=Path)
    parser.add_argument('--presentation-plan', type=Path)
    parser.add_argument('--presentation-result', type=Path)
    parser.add_argument(
        '--gui', action='store_true',
        help=('show each fresh Isaac presentation run while rendering; '
              'the default is headless rendering'))
    parser.add_argument('--epochs', type=int, default=900)
    parser.add_argument('--learning-rate', type=float, default=0.12)
    parser.add_argument(
        '--predicted-success-threshold', type=float,
        default=DEFAULT_PREDICTED_SUCCESS_THRESHOLD,
        help=('minimum predicted retention probability before a candidate is '
              'selected; the demo abstains when none meets it'))
    args = parser.parse_args()
    if args.mode == 'collect':
        if args.model is None or args.isaac_python is None or \
                args.output_dir is None:
            parser.error('collect requires --model, --isaac-python, '
                         'and --output-dir')
        return _collect(args)
    if args.mode == 'train':
        if args.dataset is None or args.output is None:
            parser.error('train requires --dataset and --output')
        return _train(args)
    if args.mode == 'prepare-presentation':
        if args.dataset is None or args.model_file is None or args.output is None:
            parser.error('prepare-presentation requires --dataset, --model-file, and --output')
        return _prepare_presentation(args)
    if args.mode == 'render':
        if args.presentation_plan is None or args.isaac_python is None or \
                args.output_dir is None:
            parser.error('render requires --presentation-plan, --isaac-python, and --output-dir')
        return _render_presentation(args)
    if args.mode == 'assemble':
        if args.presentation_result is None or args.output_dir is None:
            parser.error('assemble requires --presentation-result and --output-dir')
        return _assemble_presentation(args)
    if args.dataset is None or args.model_file is None:
        parser.error('demo requires --dataset and --model-file')
    return _demo(args)


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""Validate one complete cross-model payload-retention result matrix."""

import argparse
import hashlib
import json
from pathlib import Path
from isaac_model_contract import asset_repository_root
from grip_drive_control import script_manifest
import sys


MODELS = ('2fg7', '2fg14', 'rg2', 'rg6')
SCENARIOS = ('rated-margin-static', 'dynamic-showcase')
MINIMUM_HARNESS_REVISION = 14
TEST_NAME = 'sideways-external-pinch-payload-retention'


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _manifest(asset_root: Path, model: str) -> dict:
    model_root = asset_root / model
    return {
        str(path.relative_to(model_root)): _sha256(path)
        for path in sorted(model_root.rglob('*.usd*')) if path.is_file()
    }


def _default_package_root() -> Path:
    script = Path(__file__).resolve()
    if script.parent.name == 'scripts':
        return script.parents[1]
    return script.parents[2] / 'share' / 'onrobot_gripper_isaac'


def _default_runner(package_root: Path) -> Path:
    source_runner = package_root / 'scripts/run_payload_retention_test.py'
    if source_runner.is_file():
        return source_runner
    return Path(__file__).resolve().with_name('run_payload_retention_test.py')


def _write(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n',
                    encoding='utf-8')


def main() -> int:
    """Validate the requested result matrix and return a shell verdict."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-dir', type=Path, required=True)
    parser.add_argument('--package-root', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--runner', type=Path)
    parser.add_argument('--isaac-version')
    parser.add_argument(
        '--filename-template', default='{model}_{scenario}.json',
        help='relative result filename with {model} and {scenario} fields')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()

    package_root = (args.package_root or _default_package_root()).resolve()
    config = (args.config or package_root /
              'config/payload_retention_profiles.json').resolve()
    runner = (args.runner or _default_runner(package_root)).resolve()
    report = {
        'schema_version': 1,
        'test': 'payload-retention-result-matrix-validation',
        'results_dir': str(args.results_dir.resolve()),
        'status': 'failed',
        'cases': [],
        'errors': [],
    }

    try:
        if not config.is_file():
            raise RuntimeError(f'configuration does not exist: {config}')
        if not runner.is_file():
            raise RuntimeError(f'runner does not exist: {runner}')
        config_sha256 = _sha256(config)
        runner_sha256 = _sha256(runner)
        runtime_manifest = script_manifest(runner)
        report['config_sha256'] = config_sha256
        report['runner_sha256'] = runner_sha256

        for model in MODELS:
            current_manifest = _manifest(asset_repository_root(package_root) / 'assets', model)
            if not current_manifest:
                report['errors'].append(
                    f'{model}: current asset manifest is empty')
            for scenario in SCENARIOS:
                relative = args.filename_template.format(
                    model=model, scenario=scenario)
                path = args.results_dir / relative
                case = {
                    'model': model,
                    'scenario': scenario,
                    'path': str(path.resolve()),
                    'status': 'failed',
                    'errors': [],
                }
                report['cases'].append(case)
                if not path.is_file():
                    case['errors'].append('result file is missing')
                    continue
                try:
                    data = json.loads(path.read_text(encoding='utf-8'))
                except (OSError, json.JSONDecodeError) as error:
                    case['errors'].append(f'invalid JSON: {error}')
                    continue

                expected = {
                    'schema_version': 1,
                    'test': TEST_NAME,
                    'model': model,
                    'scenario': scenario,
                    'status': 'passed',
                    'config_sha256': config_sha256,
                    'runner_sha256': runner_sha256,
                    'runtime_script_files_sha256': runtime_manifest,
                }
                for key, value in expected.items():
                    if data.get(key) != value:
                        case['errors'].append(
                            f'{key}: expected {value!r}, got '
                            f'{data.get(key)!r}')
                if data.get('harness_revision', 0) < MINIMUM_HARNESS_REVISION:
                    case['errors'].append(
                        f'harness_revision must be at least '
                        f'{MINIMUM_HARNESS_REVISION}')
                if (args.isaac_version and
                        data.get('isaac_sim_version') != args.isaac_version):
                    case['errors'].append(
                        f'isaac_sim_version: expected {args.isaac_version!r}, '
                        f'got {data.get("isaac_sim_version")!r}')
                if data.get('payload_mass_kg') != data.get(
                        'rated_force_fit_payload_kg'):
                    case['errors'].append(
                        'payload mass differs from the force-fit rating')
                if data.get('payload_fraction_of_rating') != 1.0:
                    case['errors'].append(
                        'payload fraction of rating is not 1.0')
                for field in (
                        'fixture_fingertip_material_override',
                        'fixture_drive_override',
                        'fixture_solver_override'):
                    if data.get(field) is not None:
                        case['errors'].append(f'{field} is not null')
                effective_solver = data.get('effective_solver_settings')
                if not isinstance(effective_solver, dict):
                    case['errors'].append(
                        'effective solver settings are missing')
                else:
                    if (effective_solver.get('solver_type') != 'TGS' or
                            effective_solver.get('external_forces_every_iteration') is not True):
                        case['errors'].append(
                            'loaded scene requires TGS with external forces every iteration')
                    position_iterations = effective_solver.get(
                        'position_iterations')
                    velocity_iterations = effective_solver.get(
                        'velocity_iterations')
                    if (not isinstance(position_iterations, int) or
                            position_iterations < 1):
                        case['errors'].append(
                            'effective position iterations are invalid')
                    if (not isinstance(velocity_iterations, int) or
                            velocity_iterations < 0 or
                            velocity_iterations > 4):
                        case['errors'].append(
                            'effective TGS velocity iterations must be 0..4')
                    if (model == 'rg6' and effective_solver.get(
                            'solve_articulation_contact_last') is not True):
                        case['errors'].append(
                            'RG6 must resolve articulation contact last')
                if data.get('asset_files_sha256') != current_manifest:
                    case['errors'].append(
                        'asset manifest differs from the current package')
                tests = data.get('tests')
                if (not isinstance(tests, list) or len(tests) != 1 or
                        tests[0].get('status') != 'passed'):
                    case['errors'].append(
                        'exactly one passing qualification test is required')
                elif not all(tests[0].get('checks', {}).values()):
                    case['errors'].append(
                        'one or more qualification checks did not pass')
                if not case['errors']:
                    case['status'] = 'passed'

        report['status'] = 'passed' if all(
            case['status'] == 'passed' for case in report['cases']
        ) and not report['errors'] else 'failed'
    except (KeyError, OSError, RuntimeError, ValueError) as error:
        report['errors'].append(f'{type(error).__name__}: {error}')

    if args.output:
        _write(args.output, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    sys.exit(main())

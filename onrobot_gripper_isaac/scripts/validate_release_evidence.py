#!/usr/bin/env python3
"""Generate a release-support verdict from bound structural and runtime evidence."""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
from isaac_model_contract import asset_repository_root
import sys


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _package_root() -> Path:
    script = Path(__file__).resolve()
    if script.parent.name == 'scripts':
        return script.parents[1]
    return script.parents[2] / 'share' / 'onrobot_gripper_isaac'


def _asset_manifest(asset_root: Path, model: str) -> dict:
    model_root = asset_root / model
    return {
        str(path.relative_to(model_root)): _sha256(path)
        for path in sorted(model_root.rglob('*.usd*')) if path.is_file()
    }


def _load_asset_validator(path: Path):
    spec = importlib.util.spec_from_file_location(
        'onrobot_release_asset_validator', path)
    module = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(path.parent))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(str(path.parent))
    return module


def _case_tests(report: dict) -> dict:
    """Normalize runner case lists and older mapping-form reports."""
    tests = report.get('tests')
    if isinstance(tests, list):
        return {
            item.get('name'): item.get('status')
            for item in tests if isinstance(item, dict) and item.get('name')
        }
    if isinstance(tests, dict):
        return tests
    return {}


def _loaded_control_identity(report: dict) -> bool:
    """Verify mapped plugin identities against the candidate install tree."""
    cases = report.get('tests')
    if not isinstance(cases, list):
        return False
    for case in cases:
        if not isinstance(case, dict) or case.get('name') != (
                'clock_namespace_and_live_state'):
            continue
        prefixes = case.get('package_prefixes')
        if not isinstance(prefixes, dict):
            return False
        expected = (
            ('controller_plugin_libraries',
             'onrobot_gripper_controllers',
             'libonrobot_gripper_controller.so'),
            ('hardware_plugin_libraries',
             'onrobot_gripper_hardware',
             'libonrobot_gripper_hardware.so'),
        )
        for field, package, basename in expected:
            libraries = case.get(field)
            prefix = prefixes.get(package)
            if not isinstance(libraries, list) or not libraries or not prefix:
                return False
            prefix_path = Path(prefix).resolve()
            verified = False
            for item in libraries:
                if not isinstance(item, dict):
                    continue
                path = Path(str(item.get('path', ''))).resolve()
                digest = item.get('sha256')
                try:
                    in_prefix = path.is_relative_to(prefix_path)
                    verified = (
                        in_prefix and path.name == basename and
                        isinstance(digest, str) and _sha256(path) == digest)
                except OSError:
                    verified = False
                if verified:
                    break
            if not verified:
                return False
        return True
    return False


def _runtime_matches(report: dict, expected: dict) -> bool:
    """Match both current flat and retained nested runtime report formats."""
    runtime = report.get('runtime')
    if not isinstance(runtime, dict):
        runtime = report
    return all(runtime.get(key) == value for key, value in expected.items())


def _validate_candidate(report: dict, specification: dict,
                        expected_manifest: dict, runner_hash: str) -> list[str]:
    """Return every failed binding for one candidate runtime report."""
    errors = []
    if report.get('status') != 'passed':
        errors.append('candidate status is not passed')
    if report.get('model') != specification['model']:
        errors.append('candidate model does not match the support matrix')
    if not _runtime_matches(report, specification['runtime']):
        errors.append('candidate runtime does not match the support matrix')
    if report.get('runner_sha256') != runner_hash:
        errors.append('candidate runner hash does not match the current runner')
    if report.get('asset_files_sha256') != expected_manifest:
        errors.append('candidate asset manifest does not match the current asset')
    tests = _case_tests(report)
    missing = [
        name for name in specification['required_tests']
        if tests.get(name) != 'passed'
    ]
    if missing:
        errors.append('candidate required cases are absent or not passed: ' +
                      ', '.join(missing))
    if (specification.get('require_loaded_control_identity') and
            not _loaded_control_identity(report)):
        errors.append('candidate loaded control identity is absent or incomplete')
    return errors


def _read_json(path: Path) -> dict:
    data = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError(f'JSON object required: {path}')
    return data


def validate(package_root: Path, ros_root: Path, description_root: Path,
             evidence_dir: Path, matrix_path: Path) -> dict:
    """Validate one declared release profile without promoting old reports."""
    matrix = _read_json(matrix_path)
    if matrix.get('schema_version') != 1 or not matrix.get('profile'):
        raise ValueError('invalid release support matrix identity')
    models = matrix.get('models')
    if not isinstance(models, dict) or not models:
        raise ValueError('release support matrix has no models')
    asset_validator = _load_asset_validator(
        package_root / 'scripts/validate_asset_contract.py')
    report = {
        'schema_version': 1,
        'profile': matrix['profile'],
        'matrix_sha256': _sha256(matrix_path),
        'package_root': str(package_root),
        'evidence_dir': str(evidence_dir),
        'status': 'blocked',
        'models': [],
    }
    ready = True
    for model, model_specification in sorted(models.items()):
        model_report = {
            'model': model,
            'structural': {'status': 'failed'},
            'modes': [],
            'status': 'blocked',
        }
        report['models'].append(model_report)
        try:
            model_report['structural']['result'] = asset_validator.validate(
                package_root, ros_root / f'onrobot_{model}', description_root,
                model)
            model_report['structural']['status'] = 'passed'
        except Exception as error:  # validator errors are release evidence.
            model_report['structural']['error'] = (
                f'{type(error).__name__}: {error}')
            ready = False
            continue
        modes = model_specification.get('modes')
        if not isinstance(modes, list) or not modes:
            model_report['structural']['error'] = 'model declares no modes'
            ready = False
            continue
        for configured_mode in modes:
            mode = dict(configured_mode)
            mode['model'] = model
            mode_report = {
                'name': mode.get('name'),
                'historical': {'status': 'unavailable'},
                'candidate': {'status': 'missing'},
                'status': 'blocked',
            }
            model_report['modes'].append(mode_report)
            historical = package_root / mode['historical_report']
            if historical.is_file():
                try:
                    historical_data = _read_json(historical)
                    if historical_data.get('model') == model:
                        mode_report['historical'] = {
                            'status': 'retained',
                            'path': mode['historical_report'],
                            'sha256': _sha256(historical),
                        }
                    else:
                        mode_report['historical']['error'] = (
                            'historical report model does not match')
                except (OSError, ValueError, json.JSONDecodeError) as error:
                    mode_report['historical']['error'] = (
                        f'{type(error).__name__}: {error}')
            else:
                mode_report['historical']['error'] = 'historical report missing'

            candidate = evidence_dir / mode['candidate_report']
            if candidate.is_file():
                try:
                    candidate_data = _read_json(candidate)
                    runner = package_root / mode['runner']
                    errors = _validate_candidate(
                        candidate_data, mode, _asset_manifest(
                            asset_repository_root(package_root) / 'assets', model), _sha256(runner))
                    mode_report['candidate'] = {
                        'status': 'passed' if not errors else 'rejected',
                        'path': str(candidate),
                        'errors': errors,
                    }
                    if not errors:
                        mode_report['status'] = 'passed'
                    else:
                        ready = False
                except (OSError, ValueError, KeyError,
                        json.JSONDecodeError) as error:
                    mode_report['candidate'] = {
                        'status': 'rejected',
                        'path': str(candidate),
                        'errors': [f'{type(error).__name__}: {error}'],
                    }
                    ready = False
            else:
                ready = False
        if model_report['structural']['status'] == 'passed' and all(
                item['status'] == 'passed' for item in model_report['modes']):
            model_report['status'] = 'passed'
    report['status'] = 'passed' if ready and all(
        item['status'] == 'passed' for item in report['models']) else 'blocked'
    return report


def main() -> int:
    """Write a generated release-support result and return its shell verdict."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package-root', type=Path, default=_package_root())
    parser.add_argument('--ros-root', type=Path)
    parser.add_argument('--description-root', type=Path)
    parser.add_argument('--evidence-dir', type=Path, required=True)
    parser.add_argument('--matrix', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    package_root = args.package_root.resolve()
    ros_root = (args.ros_root or package_root.parent).resolve()
    description_root = (args.description_root or ros_root /
                        'onrobot_gripper_description').resolve()
    matrix = (args.matrix or package_root /
              'config/release_support_matrix.json').resolve()
    try:
        report = validate(package_root, ros_root, description_root,
                          args.evidence_dir.resolve(), matrix)
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as error:
        report = {'schema_version': 1, 'status': 'blocked',
                  'error': f'{type(error).__name__}: {error}'}
    rendered = json.dumps(report, indent=2, sort_keys=True) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding='utf-8')
    print(rendered, end='')
    return 0 if report.get('status') == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

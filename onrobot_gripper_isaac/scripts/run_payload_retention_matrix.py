#!/usr/bin/env python3
"""Run and validate the complete payload-retention qualification matrix."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from isaac_model_contract import asset_repository_root
import subprocess
import sys


PACKAGE_NAME = 'onrobot_gripper_isaac'
MODELS = ('2fg7', '2fg14', 'rg2', 'rg6')
SCENARIOS = ('rated-margin-static', 'dynamic-showcase')


def _default_package_root() -> Path:
    """Locate either the installed package share or the source package."""
    script = Path(__file__).absolute()
    if (script.parent.name == PACKAGE_NAME and
            script.parent.parent.name == 'lib'):
        return script.parents[2] / 'share' / PACKAGE_NAME
    return script.resolve().parents[1]


def _default_program(package_root: Path, name: str) -> Path:
    """Locate a companion program in source or installed layouts."""
    source_program = package_root / 'scripts' / name
    if source_program.is_file():
        return source_program
    return Path(__file__).absolute().with_name(name)


def _case_command(isaac_python: Path, runner: Path, model: str,
                  scenario: str, asset: Path, config: Path,
                  output: Path) -> list[str]:
    """Build one fixed qualification command without invoking a shell."""
    return [
        str(isaac_python), str(runner),
        '--model', model,
        '--scenario', scenario,
        '--asset', str(asset),
        '--config', str(config),
        '--output', str(output),
    ]


def _run_logged(command: list[str], log_path: Path) -> int:
    """Run one case, streaming output to the terminal and its log."""
    with log_path.open('w', encoding='utf-8') as log:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        try:
            assert process.stdout is not None
            for line in process.stdout:
                sys.stdout.write(line)
                sys.stdout.flush()
                log.write(line)
                log.flush()
            return process.wait()
        except KeyboardInterrupt:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            raise


def _require_file(path: Path, description: str) -> Path:
    path = path.expanduser().absolute()
    if not path.is_file():
        raise RuntimeError(f'{description} does not exist: {path}')
    return path


def _read_case_status(path: Path) -> str:
    try:
        return str(json.loads(path.read_text(encoding='utf-8')).get(
            'status', 'missing-status'))
    except (OSError, json.JSONDecodeError):
        return 'invalid-result'


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _write_summary(path: Path, summary: dict) -> None:
    """Atomically publish progress for synchronized-workspace observers."""
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    temporary.replace(path)


def main() -> int:
    """Run all eight cases and the strict result validator."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--isaac-python', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--package-root', type=Path)
    parser.add_argument('--asset-root', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--runner', type=Path)
    parser.add_argument('--validator', type=Path)
    parser.add_argument('--isaac-version', default='6.0.1')
    parser.add_argument(
        '--continue-on-failure', action='store_true',
        help='run later cases after a failed process (validation still fails)')
    args = parser.parse_args()

    try:
        package_root = (
            args.package_root or _default_package_root()).absolute()
        isaac_python = _require_file(
            args.isaac_python, 'Isaac Python launcher')
        asset_root = (args.asset_root or asset_repository_root(package_root) / 'assets').absolute()
        default_config = (
            package_root / 'config/payload_retention_profiles.json')
        config = _require_file(
            args.config or default_config, 'profile configuration')
        runner = _require_file(
            args.runner or _default_program(
                package_root, 'run_payload_retention_test.py'),
            'payload-retention runner')
        validator = _require_file(
            args.validator or _default_program(
                package_root, 'validate_payload_retention_results.py'),
            'matrix validator')
        runner_sha256 = hashlib.sha256(runner.read_bytes()).hexdigest()
        print(
            f'Payload-retention runner: {runner}\n'
            f'Runner SHA-256: {runner_sha256}\n'
            f'Asset root: {asset_root}',
            flush=True,
        )
        output_dir = args.output_dir.expanduser().absolute()
        output_dir.mkdir(parents=True, exist_ok=True)

        summary_path = output_dir / 'matrix_run.json'
        case_summary = []
        summary = {
            'schema_version': 1,
            'test': 'payload-retention-matrix-run',
            'status': 'running',
            'isaac_version': args.isaac_version,
            'runner': str(runner),
            'runner_sha256': runner_sha256,
            'asset_root': str(asset_root),
            'started_at': _utc_now(),
            'finished_at': None,
            'current_case': None,
            'cases': case_summary,
            'validation': None,
            'validation_returncode': None,
        }
        _write_summary(summary_path, summary)
        process_failed = False
        for model in MODELS:
            asset = _require_file(
                asset_root / model / f'onrobot_{model}.usda',
                f'{model} asset')
            for scenario in SCENARIOS:
                stem = f'{model}_{scenario}'
                result_path = output_dir / f'{stem}.json'
                log_path = output_dir / f'{stem}.log'
                print(f'\n=== {model}: {scenario} ===', flush=True)
                summary['current_case'] = {
                    'model': model,
                    'scenario': scenario,
                }
                _write_summary(summary_path, summary)
                command = _case_command(
                    isaac_python, runner, model, scenario, asset, config,
                    result_path)
                returncode = _run_logged(command, log_path)
                status = _read_case_status(result_path)
                case_summary.append({
                    'model': model,
                    'scenario': scenario,
                    'returncode': returncode,
                    'status': status,
                    'result': str(result_path),
                    'log': str(log_path),
                })
                _write_summary(summary_path, summary)
                if returncode != 0 or status != 'passed':
                    process_failed = True
                    if not args.continue_on_failure:
                        break
            if process_failed and not args.continue_on_failure:
                break

        validation_path = output_dir / 'matrix_validation.json'
        summary['current_case'] = {
            'model': None,
            'scenario': 'strict-matrix-validation',
        }
        summary['validation'] = str(validation_path)
        _write_summary(summary_path, summary)
        validation_command = [
            sys.executable, str(validator),
            '--results-dir', str(output_dir),
            '--package-root', str(package_root),
            '--config', str(config),
            '--runner', str(runner),
            '--isaac-version', args.isaac_version,
            '--output', str(validation_path),
        ]
        print('\n=== strict matrix validation ===', flush=True)
        validation_returncode = subprocess.run(
            validation_command, check=False).returncode
        summary = {
            'schema_version': 1,
            'test': 'payload-retention-matrix-run',
            'status': ('passed' if not process_failed and
                       validation_returncode == 0 else 'failed'),
            'isaac_version': args.isaac_version,
            'runner': str(runner),
            'runner_sha256': runner_sha256,
            'asset_root': str(asset_root),
            'started_at': summary['started_at'],
            'finished_at': _utc_now(),
            'current_case': None,
            'cases': case_summary,
            'validation': str(validation_path),
            'validation_returncode': validation_returncode,
        }
        _write_summary(summary_path, summary)
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0 if summary['status'] == 'passed' else 1
    except (OSError, RuntimeError, ValueError) as error:
        print(f'error: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())

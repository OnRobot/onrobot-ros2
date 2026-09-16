#!/usr/bin/env python3
"""Validate one complete four-model physical-motion showcase result set."""

import argparse
import hashlib
import json
import math
from pathlib import Path
from isaac_model_contract import asset_repository_root
from grip_drive_control import script_manifest
import sys


MODELS = ('2fg7', '2fg14', 'rg2', 'rg6')
MINIMUM_HARNESS_REVISION = 14
TEST_NAME = 'physically-actuated-full-payload-lift-and-shake'
EXPECTED_SCREENSHOTS = {
    '00_grasped.png',
    '01_lifted.png',
    '02_shake_upper_backward.png',
    '03_shake_lower_forward.png',
}
EXPECTED_KEYFRAMES = (
    '00_grasped',
    '01_lifted',
    '02_shake_upper_backward',
    '03_shake_lower_forward',
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _contained_path(path: Path, root: Path) -> Path:
    """Return *path* after proving its real path stays below *root*.

    ``Path.is_file`` follows symlinks, so checking it alone would allow a
    report to make the validator read media outside the result bundle.  The
    real-path check is deliberately performed before every media read.
    """
    resolved_root = root.resolve()
    try:
        resolved_path = path.resolve(strict=False)
    except (OSError, RuntimeError) as error:
        raise ValueError(f'cannot resolve path safely: {path}') from error
    try:
        resolved_path.relative_to(resolved_root)
    except ValueError as error:
        raise ValueError(
            f'path escapes report root {resolved_root}: {path}') from error
    return resolved_path


def _paths_alias(left: Path, right: Path) -> bool:
    """Return whether two paths name the same existing file or path."""
    try:
        return left.samefile(right)
    except (FileNotFoundError, OSError, RuntimeError):
        try:
            return left.resolve(strict=False) == right.resolve(strict=False)
        except (OSError, RuntimeError):
            return False


def _resolve_media_path(value: str, results_root: Path,
                        report_root: Path | None = None) -> Path:
    """Resolve a report media path without permitting path or symlink escape.

    New reports use paths relative to ``results_root``.  An archived report
    may retain an absolute path only when the caller explicitly supplies the
    original archive root; that root is mapped to the local results root.
    The original root is never treated as a filesystem fallback.
    """
    if not isinstance(value, str) or not value:
        raise ValueError('media path must be a non-empty string')
    local_root = results_root.resolve()
    raw_path = Path(value)
    if raw_path.is_absolute():
        if report_root is None:
            raise ValueError(
                'absolute media path requires an explicit --report-root '
                'mapping')
        archive_root = Path(report_root)
        if not archive_root.is_absolute():
            raise ValueError('--report-root must be an absolute path')
        try:
            relative = raw_path.relative_to(archive_root)
        except ValueError as error:
            raise ValueError(
                f'absolute media path is outside mapped report root '
                f'{archive_root}: {raw_path}') from error
        raw_path = local_root / relative
    else:
        raw_path = local_root / raw_path
    return _contained_path(raw_path, local_root)


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
    source_runner = package_root / 'scripts/run_physical_motion_showcase.py'
    if source_runner.is_file():
        return source_runner
    return Path(__file__).resolve().with_name(
        'run_physical_motion_showcase.py')


def _write(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n',
                    encoding='utf-8')


def _finite_at_least(value, minimum: float) -> bool:
    return (isinstance(value, (int, float)) and math.isfinite(value) and
            value >= minimum)


def main() -> int:
    """Validate all four reports against the exact current inputs."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-dir', type=Path, required=True)
    parser.add_argument('--package-root', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--runner', type=Path)
    parser.add_argument('--isaac-version')
    parser.add_argument(
        '--filename-template', default='{model}.json',
        help='relative result filename with a {model} field')
    parser.add_argument(
        '--require-screenshots', action='store_true',
        help='require all four live-viewport keyframes for every model')
    parser.add_argument(
        '--require-recording', action='store_true',
        help='require a complete live-physics frame sequence for every model')
    parser.add_argument(
        '--report-root', type=Path,
        help=('original absolute root for archived media paths; it is mapped '
              'to --results-dir after containment checks'))
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()

    package_root = (args.package_root or _default_package_root()).resolve()
    results_root = args.results_dir.resolve()
    report_root = args.report_root
    config = (args.config or package_root /
              'config/payload_retention_profiles.json').resolve()
    runner = (args.runner or _default_runner(package_root)).resolve()
    report = {
        'schema_version': 1,
        'test': 'physical-motion-showcase-result-set-validation',
        'results_dir': str(args.results_dir.resolve()),
        'report_root': str(report_root) if report_root is not None else None,
        'status': 'failed',
        'require_screenshots': args.require_screenshots,
        'require_recording': args.require_recording,
        'cases': [],
        'errors': [],
    }

    # Never let a verdict overwrite an input report, source, or configuration.
    # Check aliases (including symlinks and existing hard links) before any
    # validation work; a protected output must fail without writing anywhere.
    if args.output is not None:
        protected_paths = [config, runner]
        for model in MODELS:
            protected_paths.append(
                args.results_dir / args.filename_template.format(model=model))
        try:
            output_path = args.output
            output_path.resolve(strict=False)
            if any(_paths_alias(output_path, protected)
                   for protected in protected_paths):
                report['errors'].append(
                    'output path aliases a protected input report, config, '
                    'or runner')
                print(json.dumps(report, indent=2, sort_keys=True))
                return 1
        except (OSError, RuntimeError, ValueError) as error:
            report['errors'].append(f'output path cannot be resolved: {error}')
            print(json.dumps(report, indent=2, sort_keys=True))
            return 1

    try:
        if not config.is_file():
            raise RuntimeError(f'configuration does not exist: {config}')
        if not runner.is_file():
            raise RuntimeError(f'runner does not exist: {runner}')
        if report_root is not None and not report_root.is_absolute():
            raise RuntimeError('--report-root must be an absolute path')
        config_sha256 = _sha256(config)
        runner_sha256 = _sha256(runner)
        report['config_sha256'] = config_sha256
        report['runner_sha256'] = runner_sha256

        for model in MODELS:
            current_manifest = _manifest(asset_repository_root(package_root) / 'assets', model)
            case = {
                'model': model,
                'path': None,
                'status': 'failed',
                'errors': [],
            }
            report['cases'].append(case)
            if not current_manifest:
                case['errors'].append('current asset manifest is empty')
            result_candidate = args.results_dir / args.filename_template.format(
                model=model)
            try:
                path = _contained_path(result_candidate, results_root)
                case['path'] = str(path)
            except ValueError as error:
                case['errors'].append(f'result path: {error}')
                continue
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
                'status': 'passed',
                'config_sha256': config_sha256,
                'runner_sha256': runner_sha256,
                'runtime_script_files_sha256': script_manifest(runner),
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
            if data.get('asset_files_sha256') != current_manifest:
                case['errors'].append(
                    'asset manifest differs from the current package')

            scene_settings = data.get('physics_scene_settings', {})
            if (not isinstance(scene_settings, dict) or
                    scene_settings.get('solver_type') != 'TGS' or
                    scene_settings.get('external_forces_every_iteration') is not True):
                case['errors'].append(
                    'loaded scene requires TGS with external forces every iteration')
            motion = data.get('motion')
            if not isinstance(motion, dict):
                case['errors'].append('motion evidence is missing')
                motion = {}
            if motion.get('replay') is not False:
                case['errors'].append('motion must be live PhysX, not replay')
            if motion.get('source') != 'physx-two-axis-prismatic-mount-drive':
                case['errors'].append('unexpected motion source')
            if not _finite_at_least(
                    motion.get('actual_physics_dt_s'), 1e-6):
                case['errors'].append('actual physics step is missing')
            actual_physics_dt = motion.get('actual_physics_dt_s')
            timing_error = motion.get('manual_step_timing_error_s')
            if (not isinstance(timing_error, (int, float)) or
                    not math.isfinite(timing_error) or
                    not isinstance(actual_physics_dt, (int, float)) or
                    timing_error > max(actual_physics_dt * 0.01, 1e-6)):
                case['errors'].append(
                    'rendering advanced unmeasured physics time')
            keyframes = motion.get('measured_keyframes')
            if (not isinstance(keyframes, list) or
                    [value.get('name') for value in keyframes
                     if isinstance(value, dict)] != list(EXPECTED_KEYFRAMES)):
                case['errors'].append(
                    'measured keyframes are missing or out of order')
            else:
                lifted = keyframes[1].get('carrier_delta_from_grasp_m')
                upper = keyframes[2].get('carrier_delta_from_grasp_m')
                lower = keyframes[3].get('carrier_delta_from_grasp_m')
                if (not isinstance(lifted, list) or len(lifted) != 3 or
                        not _finite_at_least(lifted[1], 0.030) or
                        not _finite_at_least(lifted[2], 0.055)):
                    case['errors'].append(
                        'lifted keyframe does not show backward and upward '
                        'travel')
                if (not isinstance(upper, list) or len(upper) != 3 or
                        not isinstance(lower, list) or len(lower) != 3 or
                        not _finite_at_least(upper[1] - lower[1], 0.035) or
                        not _finite_at_least(upper[2] - lower[2], 0.014)):
                    case['errors'].append(
                        'shake keyframes do not show independent horizontal '
                        'and vertical travel')
            camera = motion.get('camera')
            if not isinstance(camera, dict):
                case['errors'].append('camera evidence is missing')
            elif (camera.get('fixed_world_camera') is not True or
                  camera.get('level_view') is not True):
                case['errors'].append(
                    'camera must be fixed in the world and level')

            tests = data.get('tests')
            if (not isinstance(tests, list) or len(tests) != 1 or
                    tests[0].get('status') != 'passed'):
                case['errors'].append(
                    'exactly one passing showcase test is required')
            else:
                checks = tests[0].get('checks')
                measurements = tests[0].get('measurements')
                if not isinstance(checks, dict) or not checks or not all(
                        value is True for value in checks.values()):
                    case['errors'].append(
                        'one or more showcase checks did not pass')
                if not isinstance(measurements, dict):
                    case['errors'].append('motion measurements are missing')
                else:
                    for field, minimum in (
                            ('horizontal_shake_span_m', 0.035),
                            ('vertical_shake_span_m', 0.014),
                            ('diagonal_shake_span_m', 0.040),
                            ('maximum_lift_backward_travel_m', 0.030),
                            ('maximum_carrier_vertical_lift_m', 0.055)):
                        if not _finite_at_least(
                                measurements.get(field), minimum):
                            case['errors'].append(
                                f'{field} is below {minimum}')

            screenshots = motion.get('screenshots')
            if args.require_screenshots:
                if not isinstance(screenshots, list):
                    case['errors'].append('screenshot list is missing')
                else:
                    screenshot_paths = []
                    for value in screenshots:
                        try:
                            screenshot_paths.append(_resolve_media_path(
                                value, results_root, report_root))
                        except ValueError as error:
                            case['errors'].append(
                                f'screenshot path: {error}')
                    actual_names = {value.name for value in screenshot_paths}
                    if actual_names != EXPECTED_SCREENSHOTS:
                        case['errors'].append(
                            'screenshot set does not contain the four '
                            'required keyframes')
                    missing = [str(value) for value in screenshot_paths
                               if not value.is_file()]
                    if missing:
                        case['errors'].append(
                            'screenshot files are missing: ' + ', '.join(
                                missing))
            if args.require_recording:
                recording = motion.get('recording')
                if not isinstance(recording, dict):
                    case['errors'].append('recording evidence is missing')
                else:
                    if recording.get('enabled') is not True:
                        case['errors'].append('recording was not enabled')
                    if recording.get(
                            'source') != 'same-live-physx-evidence-run':
                        case['errors'].append(
                            'recording is not from the live evidence run')
                    if recording.get('replay') is not False:
                        case['errors'].append(
                            'recording must not be a replay')
                    frame_count = recording.get('frame_count')
                    if (not isinstance(frame_count, int) or
                            isinstance(frame_count, bool) or
                            frame_count < 30):
                        case['errors'].append(
                            'recording must contain at least 30 frames')
                    directory_value = recording.get('directory')
                    if not isinstance(directory_value, str):
                        case['errors'].append(
                            'recording directory is missing')
                    elif isinstance(frame_count, int):
                        try:
                            directory = _resolve_media_path(
                                directory_value, results_root, report_root)
                        except ValueError as error:
                            case['errors'].append(
                                f'recording directory path: {error}')
                            directory = None
                        if directory is None:
                            continue
                        expected_frames = [
                            directory / f'frame_{index:05d}.png'
                            for index in range(frame_count)]
                        contained_frames = []
                        for value in expected_frames:
                            try:
                                contained_frames.append(_contained_path(
                                    value, results_root))
                            except ValueError as error:
                                case['errors'].append(
                                    f'recording frame path: {error}')
                        missing_frames = [
                            str(value) for value in contained_frames
                            if not value.is_file()]
                        extra_frames = []
                        if directory.is_dir():
                            for value in directory.glob('frame_*.png'):
                                try:
                                    _contained_path(value, results_root)
                                except ValueError as error:
                                    case['errors'].append(
                                        f'recording frame path: {error}')
                                    continue
                                if value not in expected_frames:
                                    extra_frames.append(value)
                        if missing_frames:
                            case['errors'].append(
                                'recording frames are missing: ' + ', '.join(
                                    missing_frames[:5]))
                        if extra_frames:
                            case['errors'].append(
                                'recording contains non-sequential frames')
                    if not _finite_at_least(
                            recording.get('effective_frame_rate_hz'), 1.0):
                        case['errors'].append(
                            'effective recording frame rate is missing')
                    first_time = recording.get('first_simulation_time_s')
                    last_time = recording.get('last_simulation_time_s')
                    if (not isinstance(first_time, (int, float)) or
                            not isinstance(last_time, (int, float)) or
                            not math.isfinite(first_time) or
                            not math.isfinite(last_time) or
                            last_time <= first_time):
                        case['errors'].append(
                            'recording simulation-time bounds are invalid')
                    if not isinstance(
                            recording.get('ffmpeg_command'), str):
                        case['errors'].append(
                            'recording ffmpeg command is missing')
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

#!/usr/bin/env python3
"""Record a safe, state-only OnRobot ROS session for later replay."""

import argparse
import hashlib
import json
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


MODELS = ('2fg7', '2fg14', 'rg2', 'rg6')
DESCRIPTION_FILES = {
    model: f'realmesh_onrobot_{model}.urdf.xacro' for model in MODELS
}
TOPIC_SUFFIXES = (
    'joint_states',
    'gripper_state_broadcaster/state',
    'realtime_controller/state',
    'parallel_gripper_limit_broadcaster/values',
    'diagnostics',
)
QOS_OVERRIDE = {
    'reliability': 'best_effort',
    'durability': 'volatile',
    'history': 'keep_last',
    'depth': 100,
}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _namespace(value: str) -> str:
    value = value.strip()
    if not value or value == '/':
        return ''
    return '/' + value.strip('/')


def _topic(namespace: str, suffix: str) -> str:
    return f'{namespace}/{suffix}' if namespace else f'/{suffix}'


def _write_qos_profile_overrides(path: Path, topics: list[str]) -> None:
    """Write compatible read-only subscriptions for every state stream.

    The filtered joint-state publisher uses sensor-data QoS on supported
    bringups.  A rosbag recorder otherwise creates a default reliable
    subscription and can silently receive no samples when the offered QoS is
    best effort.  Best-effort subscribers remain compatible with reliable
    publishers, so this profile covers both the filtered joint state and the
    typed state streams without changing the publishers or replay behavior.
    """
    lines = []
    for topic in topics:
        lines.extend([
            f'{json.dumps(topic)}:',
            f"  reliability: {QOS_OVERRIDE['reliability']}",
            f"  durability: {QOS_OVERRIDE['durability']}",
            f"  history: {QOS_OVERRIDE['history']}",
            f"  depth: {QOS_OVERRIDE['depth']}",
        ])
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def _validate_namespace(value: str) -> str:
    """Accept only a canonical ROS namespace, never arbitrary YAML/text."""
    if not isinstance(value, str) or _namespace(value) != value:
        raise RuntimeError('namespace is not canonical')
    if value and not all(re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', part)
                         for part in value.strip('/').split('/')):
        raise RuntimeError('namespace contains an invalid ROS name segment')
    return value


def _validate_frame(value: str, field: str) -> str:
    """Validate a TF frame without rejecting a normal hierarchical prefix."""
    if not isinstance(value, str) or not value or value.startswith('/'):
        raise RuntimeError(f'{field} is invalid')
    if not all(re.fullmatch(r'[A-Za-z0-9_-]+', part)
               for part in value.split('/')):
        raise RuntimeError(f'{field} is invalid')
    return value


def _package_prefix(package: str) -> Path | None:
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'prefix', package], check=True,
            capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return None
    value = result.stdout.strip()
    return Path(value) if value else None


def _default_xacro(model: str) -> Path:
    source_root = Path(__file__).resolve().parents[2]
    source = (source_root / f'onrobot_{model}' / 'urdf' /
              DESCRIPTION_FILES[model])
    if source.is_file():
        return source
    prefix = _package_prefix(f'onrobot_{model}')
    if prefix is not None:
        installed = (prefix / 'share' / f'onrobot_{model}' / 'urdf' /
                     DESCRIPTION_FILES[model])
        if installed.is_file():
            return installed
    raise RuntimeError(
        f'cannot find the installed or source URDF Xacro for {model}; '
        'pass --description-file explicitly')


def _git_identity(root: Path, label: str) -> dict[str, object]:
    """Capture a compact, read-only identity for a source checkout."""
    if not root.is_dir():
        return {'status': 'unknown', 'reason': f'{label} checkout not found'}
    try:
        revision = subprocess.run(
            ['git', '-C', str(root), 'rev-parse', 'HEAD'], check=True,
            capture_output=True, text=True, timeout=5).stdout.strip()
        state = subprocess.run(
            ['git', '-C', str(root), 'status', '--porcelain=v1',
             '--untracked-files=all'],
            check=True, capture_output=True, text=True, timeout=5).stdout
        diff = subprocess.run(
            ['git', '-C', str(root), 'diff', '--binary', 'HEAD'],
            check=True, capture_output=True, timeout=15).stdout
        untracked = subprocess.run(
            ['git', '-C', str(root), 'ls-files', '--others',
             '--exclude-standard', '-z'], check=True, capture_output=True,
            timeout=10).stdout.split(b'\0')
    except (OSError, subprocess.SubprocessError) as error:
        return {'status': 'unknown', 'reason': f'{label} git identity unavailable: '
                f'{error}'}
    if not revision:
        return {'status': 'unknown', 'reason': f'{label} has no git revision'}
    untracked_hashes: dict[str, str] = {}
    for raw_path in untracked:
        if not raw_path:
            continue
        relative = raw_path.decode('utf-8', errors='surrogateescape')
        path = root / relative
        if path.is_file():
            untracked_hashes[relative] = _sha256(path)
    return {
        'status': 'available',
        'revision': revision,
        'tree_state': 'dirty' if state.strip() else 'clean',
        'tracked_diff_sha256': hashlib.sha256(diff).hexdigest(),
        'untracked_file_sha256': untracked_hashes,
    }


def _asset_identity(model: str) -> dict[str, object]:
    """Read the model contract without making a release claim."""
    source_root = Path(__file__).resolve().parents[2]
    candidates = [
        source_root / 'onrobot_gripper_isaac' / 'config' /
        f'{model}_asset_contract.json',
    ]
    prefix = _package_prefix('onrobot_gripper_isaac')
    if prefix is not None:
        candidates.append(
            prefix / 'share' / 'onrobot_gripper_isaac' / 'config' /
            f'{model}_asset_contract.json')
    contract_path = next((path for path in candidates if path.is_file()), None)
    if contract_path is None:
        return {'status': 'unknown', 'reason': 'asset contract not found'}
    try:
        contract = json.loads(contract_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        return {'status': 'unknown', 'reason': f'asset contract unreadable: {error}'}
    if not isinstance(contract, dict):
        return {'status': 'unknown', 'reason': 'asset contract is not an object'}
    asset_root = contract_path.parent.parent
    identity: dict[str, object] = {
        'status': 'available',
        'asset_revision': contract.get('asset_revision', 'unknown'),
        'asset_entrypoint': contract.get('asset_entrypoint', 'unknown'),
        'contract_sha256': _sha256(contract_path),
    }
    entrypoint = contract.get('asset_entrypoint')
    if isinstance(entrypoint, str):
        entrypoint_path = asset_root / entrypoint
        identity['entrypoint_sha256'] = (
            _sha256(entrypoint_path) if entrypoint_path.is_file() else 'missing')
    layers = contract.get('layers')
    if isinstance(layers, dict):
        identity['declared_layer_sha256'] = {
            str(path): str(digest) for path, digest in layers.items()}
        identity['layer_sha256'] = {
            str(path): (_sha256(asset_root / str(path))
                        if (asset_root / str(path)).is_file() else 'missing')
            for path in layers}
    return identity


def _tool_api_identity() -> dict[str, object]:
    """Capture available Tool API checkout and package identities."""
    workspace_root = Path(__file__).resolve().parents[3]
    checkout = workspace_root / 'onrobot-tool-api'
    identity: dict[str, object] = {
        'source': _git_identity(checkout, 'Tool API'),
        'debian_packages': {},
    }
    for package in ('libonrobot-tool-api0', 'libonrobot-tool-api-dev'):
        try:
            result = subprocess.run(
                ['dpkg-query', '-W', '-f=${Version}', package], check=True,
                capture_output=True, text=True, timeout=5)
        except (OSError, subprocess.SubprocessError):
            continue
        version = result.stdout.strip()
        if version:
            identity['debian_packages'][package] = version
    if not identity['debian_packages']:
        identity['debian_packages'] = {'status': 'unknown'}
    hardware_prefix = _package_prefix('onrobot_gripper_hardware')
    if hardware_prefix is not None:
        hardware_library = hardware_prefix / 'lib' / 'libonrobot_gripper_hardware.so'
        if hardware_library.is_file():
            try:
                result = subprocess.run(
                    ['ldd', str(hardware_library)], check=True,
                    capture_output=True, text=True, timeout=10)
                match = re.search(
                    r'libonrobot_tool_api\.so[^\s]*\s+=>\s+(\S+)',
                    result.stdout)
                if match:
                    library = Path(match.group(1))
                    if library.is_file():
                        identity['resolved_by_hardware_plugin'] = {
                            'path': str(library), 'sha256': _sha256(library)}
            except (OSError, subprocess.SubprocessError):
                pass
    identity.setdefault('resolved_by_hardware_plugin', {'status': 'unknown'})
    return identity


def _expand_description(model: str, description_file: Path | None,
                        backend: str) -> tuple[str, Path]:
    source = description_file or _default_xacro(model)
    source = source.expanduser().resolve()
    if not source.is_file():
        raise RuntimeError(f'description file does not exist: {source}')
    if source.suffix == '.xacro' or source.name.endswith('.urdf.xacro'):
        try:
            result = subprocess.run(
                ['xacro', str(source), f'backend:={backend}'], check=True,
                capture_output=True, text=True, timeout=30)
        except FileNotFoundError as error:
            raise RuntimeError(
                'xacro is not installed or not on PATH') from error
        except subprocess.CalledProcessError as error:
            detail = (error.stderr or error.stdout or '').strip()
            raise RuntimeError(f'xacro expansion failed: {detail}') from error
        return result.stdout, source
    return source.read_text(encoding='utf-8'), source


def _live_robot_description(node: str) -> str:
    """Read the exact description used by the currently running publisher."""
    try:
        result = subprocess.run(
            ['ros2', 'param', 'get', '--hide-type', '--timeout', '3', node,
             'robot_description'], check=True, capture_output=True, text=True,
            timeout=8)
    except (OSError, subprocess.SubprocessError) as error:
        raise RuntimeError(
            f'cannot read live robot_description from {node}; start the showcase '
            'first or pass --description-file explicitly') from error
    # Fast DDS may write colored diagnostics to the child process' stdout in
    # restricted environments.  The parameter value is still the exact URDF,
    # but it may no longer be the first bytes in stdout.  Extract the complete
    # XML document and reject anything that does not contain a closed robot
    # element rather than persisting transport diagnostics as URDF.
    description = result.stdout
    for prefix in ('String value is:', 'value:'):
        marker = description.find(prefix)
        if marker >= 0:
            description = description[marker + len(prefix):]
            break
    start = description.find('<robot')
    end_marker = '</robot>'
    end = description.rfind(end_marker)
    closed_robot = start >= 0 and end >= start
    if start >= 0 and not closed_robot:
        self_closing_end = description.find('/>', start)
        if self_closing_end >= 0:
            end = self_closing_end + 2
    if start < 0 or end < start:
        raise RuntimeError(
            f'live robot_description from {node} is not a URDF string')
    if closed_robot:
        end += len(end_marker)
    return description[start:end].strip()


def _capture_description(model: str, description_file: Path | None,
                         backend: str, publisher_node: str) -> tuple[str, str, str]:
    """Return a saved URDF and a precise statement of how it was captured."""
    if description_file is not None:
        xml, source = _expand_description(model, description_file, backend)
        return xml, str(source), 'explicit-description-file'
    return (_live_robot_description(publisher_node),
            f'ros-parameter:{publisher_node}:robot_description',
            'live-robot-state-publisher')


def _state_metadata(topic: str) -> dict[str, str]:
    """Best-effort identity probe; the bag remains authoritative if absent."""
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '--once',
             '--qos-reliability', 'best_effort', topic],
            capture_output=True, text=True, timeout=3)
    except (OSError, subprocess.SubprocessError):
        return {}
    found: dict[str, str] = {}
    for key in ('model', 'firmware', 'device_profile_revision',
                'finger_profile_name', 'finger_profile_revision'):
        match = re.search(rf'^\s*{re.escape(key)}:\s*(.*?)\s*$',
                          result.stdout, flags=re.MULTILINE)
        if match:
            found[key] = match.group(1).strip().strip("'")
    return found


def _bag_counts(bag: Path) -> tuple[dict[str, int], str]:
    try:
        result = subprocess.run(
            ['ros2', 'bag', 'info', '-s', 'mcap', str(bag)],
            capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.SubprocessError) as error:
        return {}, f'ros2 bag info unavailable: {error}'
    output = (result.stdout or '') + (result.stderr or '')
    counts: dict[str, int] = {}
    # Jazzy prints one topic per line with ``Count: N``.  Keep the raw output
    # as well because format changes must not discard the evidence.
    for line in output.splitlines():
        match = re.search(r'Topic:\s*(\S+).*?Count:\s*(\d+)', line)
        if match:
            counts[match.group(1)] = int(match.group(2))
    return counts, output


def _bag_state_stream_identity(bag: Path, topics: list[str]) -> dict[str, object]:
    """Hash the actual recorded state streams without decoding their messages.

    MCAP serialized bytes plus recorded timestamps are the exact state samples
    replayed by rosbag2.  Keeping one compact digest per allowlisted topic lets
    replay reject a bag that no longer matches its session manifest without
    inventing a controller, hardware owner, or command stream.
    """
    try:
        import rosbag2_py  # pylint: disable=import-outside-toplevel
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
            rosbag2_py.ConverterOptions('', ''))
    except (ImportError, RuntimeError, OSError) as error:
        return {'status': 'unknown', 'reason': f'bag stream identity unavailable: {error}'}

    digests = {topic: hashlib.sha256() for topic in topics}
    counts = {topic: 0 for topic in topics}
    try:
        while reader.has_next():
            topic, serialized, stamp_ns = reader.read_next()
            if topic not in digests:
                continue
            digests[topic].update(int(stamp_ns).to_bytes(8, 'little', signed=True))
            digests[topic].update(bytes(serialized))
            counts[topic] += 1
    except RuntimeError as error:
        return {'status': 'unknown', 'reason': f'bag stream identity unreadable: {error}'}
    return {
        'status': 'available',
        'sample_counts': counts,
        'sample_sha256': {topic: digest.hexdigest()
                          for topic, digest in digests.items()},
    }


def _recording_error(bag: Path, counts: dict[str, int],
                     namespace: str) -> str | None:
    """Return a failure reason when rosbag did not produce usable state."""
    if not (bag / 'metadata.yaml').is_file():
        return 'rosbag2 did not write metadata.yaml'
    if not counts:
        return 'rosbag2 metadata contains no topic counts'
    required = (
        _topic(namespace, 'joint_states'),
        _topic(namespace, 'gripper_state_broadcaster/state'),
    )
    missing = [topic for topic in required if counts.get(topic, 0) <= 0]
    if missing:
        return 'required state topics have no samples: ' + ', '.join(missing)
    return None


def _signal_recording(process: subprocess.Popen[bytes], signum: int) -> None:
    """Stop ros2 bag through its process group, tolerating an early exit."""
    if process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signum)
    except ProcessLookupError:
        # The recorder can finish between poll() and killpg().
        return


def main() -> int:
    """Record the configured state topics and write session metadata."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=MODELS, required=True)
    parser.add_argument('--namespace', default='')
    parser.add_argument('--backend', choices=('real', 'fake', 'isaac'),
                        default='real')
    parser.add_argument('--description-file', type=Path)
    parser.add_argument(
        '--robot-state-publisher-node',
        help='Node whose live robot_description is saved; defaults to the '
             'namespaced robot_state_publisher')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--duration', type=float,
                        help='stop automatically after this many seconds')
    parser.add_argument('--base-parent-frame', default='world')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--frame-prefix', default='')
    parser.add_argument('--base-xyz', nargs=3, type=float,
                        default=(0.0, 0.0, 0.0), metavar=('X', 'Y', 'Z'))
    parser.add_argument('--base-rpy', nargs=3, type=float,
                        default=(0.0, 0.0, 0.0), metavar=('R', 'P', 'Y'))
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0.0:
        parser.error('--duration must be positive')

    output = args.output_dir.expanduser().resolve()
    bag = output / 'bag'
    description = output / 'robot_description.urdf'
    qos_overrides = output / 'qos_profile_overrides.yaml'
    metadata_file = output / 'session.json'
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f'output directory is not empty: {output}')
    output.mkdir(parents=True, exist_ok=True)

    namespace = _validate_namespace(_namespace(args.namespace))
    raw_base_frame = _validate_frame(args.base_frame, 'base frame')
    frame_prefix = args.frame_prefix.strip()
    if frame_prefix.startswith('/'):
        raise RuntimeError('frame prefix must not start with a slash')
    base_frame = _validate_frame(frame_prefix + raw_base_frame, 'prefixed base frame')
    base_parent_frame = _validate_frame(args.base_parent_frame,
                                        'base parent frame')
    publisher_node = args.robot_state_publisher_node or _topic(
        namespace, 'robot_state_publisher')
    topics = [_topic(namespace, suffix) for suffix in TOPIC_SUFFIXES]
    _write_qos_profile_overrides(qos_overrides, topics)
    xml, source, description_capture = _capture_description(
        args.model, args.description_file, args.backend, publisher_node)
    description.write_text(xml, encoding='utf-8')
    state_identity = _state_metadata(
        _topic(namespace, 'gripper_state_broadcaster/state'))
    started = _utc_now()
    metadata = {
        'schema_version': 1,
        'status': 'recording',
        'model': args.model,
        'backend': args.backend,
        'namespace': namespace,
        'topics': topics,
        'description_file': description.name,
        'description_source': str(source),
        'description_sha256': _sha256(description),
        'base_parent_frame': base_parent_frame,
        'base_frame': base_frame,
        'frame_mapping': {
            'namespace': namespace,
            'frame_prefix': frame_prefix,
            'parent_frame': base_parent_frame,
            'unprefixed_base_frame': raw_base_frame,
            'base_frame': base_frame,
        },
        'base_xyz_m': list(args.base_xyz),
        'base_rpy_rad': list(args.base_rpy),
        'started_at': started,
        'firmware_identity_at_start': state_identity,
        'description_capture': description_capture,
        'source_identity': _git_identity(
            Path(__file__).resolve().parents[2], 'ROS source'),
        'asset_identity': _asset_identity(args.model),
        'tool_api_identity': _tool_api_identity(),
        'bag_directory': bag.name,
        'qos_profile_overrides_file': qos_overrides.name,
        'command': ['ros2', 'bag', 'record', '-o', str(bag),
                    '--storage', 'mcap',
                    '--qos-profile-overrides-path', str(qos_overrides),
                    '--topics', *topics],
    }
    metadata_file.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')

    command = ['ros2', 'bag', 'record', '-o', str(bag), '--storage', 'mcap',
               '--qos-profile-overrides-path', str(qos_overrides),
               '--topics', *topics]
    process = subprocess.Popen(command, start_new_session=True)
    exit_code = 0
    intentionally_stopped = False
    try:
        if args.duration is None:
            process.wait()
        else:
            deadline = time.monotonic() + args.duration
            while process.poll() is None and time.monotonic() < deadline:
                time.sleep(min(0.25, max(0.01, deadline - time.monotonic())))
            if process.poll() is None:
                intentionally_stopped = True
                # ros2 bag's Jazzy recorder installs a SIGTERM handler that
                # finalizes storage. SIGINT is only a KeyboardInterrupt when
                # a terminal is attached and can leave an empty MCAP here.
                _signal_recording(process, signal.SIGTERM)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                _signal_recording(process, signal.SIGKILL)
                process.wait()
    except KeyboardInterrupt:
        intentionally_stopped = True
        _signal_recording(process, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            _signal_recording(process, signal.SIGKILL)
            process.wait()
    finally:
        exit_code = process.returncode or 0
        # ros2 bag receives SIGINT/SIGTERM when this wrapper stops it.  That is
        # a normal completed recording, not a failed bag.  Preserve nonzero
        # codes from an unexpected recorder failure.
        if intentionally_stopped and exit_code < 0:
            exit_code = 0

    counts, info = _bag_counts(bag)
    (output / 'bag_info.txt').write_text(info, encoding='utf-8')
    recording_error = _recording_error(bag, counts, namespace)
    if recording_error is not None and exit_code == 0:
        exit_code = 2
    metadata.update({
        'status': 'complete' if exit_code == 0 else 'failed',
        'finished_at': _utc_now(),
        'record_exit_code': exit_code,
        'topic_counts': counts,
        'bag_info_file': 'bag_info.txt',
        'recorded_state_streams': _bag_state_stream_identity(bag, topics),
    })
    if recording_error is not None:
        metadata['recording_error'] = recording_error
    metadata_file.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    print(json.dumps(metadata, indent=2, sort_keys=True))
    return exit_code


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, OSError, ValueError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(2)

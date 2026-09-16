#!/usr/bin/env python3
"""Replay a state-only OnRobot session in RViz without hardware access."""

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path


MODELS = ('2fg7', '2fg14', 'rg2', 'rg6')
TOPIC_SUFFIXES = (
    'joint_states',
    'gripper_state_broadcaster/state',
    'realtime_controller/state',
    'parallel_gripper_limit_broadcaster/values',
    'diagnostics',
)


def _namespace(value: str) -> str:
    """Return the canonical ROS namespace representation."""
    value = value.strip()
    if not value or value == '/':
        return ''
    return '/' + value.strip('/')


def _topic(namespace: str, suffix: str) -> str:
    return f'{namespace}/{suffix}' if namespace else f'/{suffix}'


def _validate_namespace(value: object) -> str:
    """Accept a canonical ROS namespace and reject config-injection text."""
    if not isinstance(value, str) or _namespace(value) != value:
        raise RuntimeError('session namespace is not canonical')
    if value and not all(re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', part)
                         for part in value.strip('/').split('/')):
        raise RuntimeError('session namespace is invalid')
    return value


def _validate_session_paths(metadata: dict) -> None:
    """Keep schema-1 replay confined to recorder-owned files."""
    if metadata['description_file'] != (
            'robot_description.urdf'):
        raise RuntimeError('session description path is not supported')
    if metadata['bag_directory'] != 'bag':
        raise RuntimeError('session bag path is not supported')


def _validate_topics(metadata: dict) -> list[str]:
    """Allow only the state streams written by the recorder."""
    namespace = _validate_namespace(metadata['namespace'])
    expected = {_topic(namespace, suffix) for suffix in TOPIC_SUFFIXES}
    topics = metadata['topics']
    if (not isinstance(topics, list) or not topics or
            not all(isinstance(topic, str) for topic in topics)):
        raise RuntimeError('session topics are not a non-empty list')
    if len(set(topics)) != len(topics) or not set(topics) <= expected:
        raise RuntimeError(
            'session topic allowlist contains a non-state or duplicate topic')
    state_topic = _topic(namespace, 'gripper_state_broadcaster/state')
    if state_topic not in topics:
        raise RuntimeError('session does not contain typed gripper state')
    return topics


def _validate_frame(value: object, field: str) -> str:
    """Validate a frame before placing it in a generated RViz config."""
    if not isinstance(value, str) or not value or value.startswith('/'):
        raise RuntimeError(f'session {field} is invalid')
    if not all(re.fullmatch(r'[A-Za-z0-9_-]+', segment)
               for segment in value.split('/')):
        raise RuntimeError(f'session {field} is invalid')
    return value


def _validate_number_list(value: object, field: str) -> None:
    """Validate recorded transform components before process launch."""
    if (not isinstance(value, list) or len(value) != 3 or
            not all(isinstance(item, (int, float)) and math.isfinite(item)
                    for item in value)):
        raise RuntimeError(f'session {field} is invalid')


def _validate_optional_provenance(metadata: dict) -> None:
    """Validate optional recorder provenance without requiring it for replay.

    Provenance is descriptive rather than an installation dependency.  Older
    schema-1 sessions therefore remain replayable, while newer sessions must
    not carry a frame mapping that contradicts the fields used to launch RViz.
    Identity records are deliberately only shape-checked: an unavailable
    checkout or package must be recorded as unknown, not make a valid bag
    unusable.
    """
    mapping = metadata.get('frame_mapping')
    if mapping is not None:
        if not isinstance(mapping, dict):
            raise RuntimeError('session frame mapping is invalid')
        if mapping.get('namespace') != metadata['namespace']:
            raise RuntimeError('session frame mapping namespace disagrees')
        if mapping.get('base_frame') != metadata['base_frame']:
            raise RuntimeError('session frame mapping base frame disagrees')
        for field in ('frame_prefix', 'parent_frame'):
            if field in mapping and not isinstance(mapping[field], str):
                raise RuntimeError(f'session frame mapping {field} is invalid')
        raw_base = mapping.get('unprefixed_base_frame')
        prefix = mapping.get('frame_prefix', '')
        if raw_base is not None:
            _validate_frame(raw_base, 'unprefixed base frame')
            if not isinstance(prefix, str) or prefix.startswith('/'):
                raise RuntimeError('session frame prefix is invalid')
            if prefix + raw_base != metadata['base_frame']:
                raise RuntimeError('session frame prefix disagrees with base frame')
        if ('parent_frame' in mapping and
                mapping['parent_frame'] != metadata.get('base_parent_frame',
                                                        'world')):
            raise RuntimeError('session frame mapping parent frame disagrees')

    for field in ('source_identity', 'asset_identity', 'tool_api_identity'):
        value = metadata.get(field)
        if value is not None and not isinstance(value, dict):
            raise RuntimeError(f'session {field} is invalid')


def _bag_state_stream_identity(bag: Path, topics: list[str]) -> dict[str, object]:
    """Return compact identities for the serialized state samples in an MCAP."""
    try:
        import rosbag2_py  # pylint: disable=import-outside-toplevel
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
            rosbag2_py.ConverterOptions('', ''))
    except (ImportError, RuntimeError, OSError) as error:
        raise RuntimeError(f'cannot verify recorded state streams: {error}') from error
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
        raise RuntimeError(f'cannot read recorded state streams: {error}') from error
    return {
        'sample_counts': counts,
        'sample_sha256': {topic: digest.hexdigest()
                          for topic, digest in digests.items()},
    }


def _validate_recorded_state_streams(metadata: dict, bag: Path,
                                    topics: list[str]) -> None:
    """Reject a completed recording whose serialized replay input changed."""
    expected = metadata.get('recorded_state_streams')
    if expected is None:
        # Schema-1 recordings created before the compact MCAP digest remain
        # replayable, but newer recordings always carry this field.
        return
    if not isinstance(expected, dict):
        raise RuntimeError('session recorded state streams are invalid')
    if expected.get('status') == 'unknown':
        return
    expected_counts = expected.get('sample_counts')
    expected_digests = expected.get('sample_sha256')
    if (not isinstance(expected_counts, dict) or
            not isinstance(expected_digests, dict) or
            set(expected_counts) != set(topics) or
            set(expected_digests) != set(topics)):
        raise RuntimeError('session recorded state streams are invalid')
    observed = _bag_state_stream_identity(bag, topics)
    if (observed['sample_counts'] != expected_counts or
            observed['sample_sha256'] != expected_digests):
        raise RuntimeError('recorded state streams no longer match session metadata')


def _load_session(path: Path) -> tuple[Path, dict]:
    root = path.expanduser().resolve()
    session_file = root / 'session.json' if root.is_dir() else root
    if session_file.name != 'session.json':
        raise RuntimeError('pass a session directory or its session.json file')
    metadata = json.loads(session_file.read_text(encoding='utf-8'))
    if (metadata.get('schema_version') != 1 or
            metadata.get('status') != 'complete'):
        raise RuntimeError(
            'session is incomplete or uses an unsupported schema')
    required = ('model', 'namespace', 'topics', 'description_file',
                'description_sha256', 'bag_directory', 'base_frame')
    missing = [key for key in required if key not in metadata]
    if missing:
        raise RuntimeError('session is missing: ' + ', '.join(missing))
    if metadata['model'] not in MODELS:
        raise RuntimeError('session model is unsupported')
    _validate_session_paths(metadata)
    topics = _validate_topics(metadata)
    _validate_frame(metadata['base_frame'], 'base frame')
    if 'base_parent_frame' in metadata:
        if metadata['base_parent_frame']:
            _validate_frame(metadata['base_parent_frame'], 'base parent frame')
    if 'base_xyz_m' in metadata:
        _validate_number_list(metadata['base_xyz_m'], 'base translation')
    if 'base_rpy_rad' in metadata:
        _validate_number_list(metadata['base_rpy_rad'], 'base rotation')
    _validate_optional_provenance(metadata)
    description = session_file.parent / metadata['description_file']
    bag = session_file.parent / metadata['bag_directory']
    if (not description.is_file() or not bag.is_dir() or
            not (bag / 'metadata.yaml').is_file()):
        raise RuntimeError('session description or bag directory is missing')
    digest = hashlib.sha256(description.read_bytes()).hexdigest()
    if digest != metadata['description_sha256']:
        raise RuntimeError(
            'saved robot description hash does not match metadata')
    _validate_recorded_state_streams(metadata, bag, topics)
    return session_file.parent, metadata


@dataclass
class _ReplayChild:
    process: subprocess.Popen
    process_group: int


def _start_child(command: list[str]) -> _ReplayChild:
    """Start each replay component in a private process group."""
    process = subprocess.Popen(command, start_new_session=True)
    return _ReplayChild(process, os.getpgid(process.pid))


def _signal_child(child: _ReplayChild, signum: int) -> None:
    """Signal a replay component and any process it launched."""
    try:
        os.killpg(child.process_group, signum)
    except ProcessLookupError:
        return


def _write_rviz_config(path: Path, metadata: dict, state_topic: str) -> Path:
    frame = metadata['base_frame']
    # JSON string syntax is also valid YAML.  Quote all session-derived
    # values before writing a temporary RViz configuration.
    state_topic_yaml = json.dumps(state_topic)
    model_yaml = json.dumps(f"{metadata['model']} replay")
    frame_yaml = json.dumps(frame)
    config = f"""Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: onrobot_gripper_rviz_plugins/ForceHistoryPanel
    Name: Force & State History
    OnRobot panel settings:
      gripper_state_topic: {state_topic_yaml}
      max_sample_age_s: 0.75
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description Source: Topic
      Description Topic: /robot_description
      Enabled: true
      Name: {model_yaml}
      Update Interval: 0.1
      Visual Enabled: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: {frame_yaml}
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 0.45
      Focal Point: {{X: 0, Y: 0, Z: 0.05}}
      Name: Replay View
      Near Clip Distance: 0.01
      Pitch: 0.55
      Target Frame: {frame_yaml}
      Yaw: 0.75
Window Geometry:
  Height: 900
  Width: 1400
  X: 40
  Y: 40
"""
    path.write_text(config, encoding='utf-8')
    return path


def _robot_state_publisher_command(description: Path, joint_topic: str,
                                   frame_prefix: str) -> list[str]:
    """Build the read-only robot-state publisher command for one replay."""
    command = [
        'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
        '--ros-args', '-p',
        f'robot_description:={description.read_text(encoding="utf-8")}',
        '-r', f'joint_states:={joint_topic}',
    ]
    if frame_prefix:
        command.extend(['-p', f'frame_prefix:={frame_prefix}'])
    return command


def _bag_play_command(bag: Path, rate: float, topics: list[str], loop: bool,
                      start_paused: bool) -> list[str]:
    """Build a state-only rosbag command; no command/action topic is admitted."""
    command = ['ros2', 'bag', 'play', str(bag), '--clock', '-r', str(rate),
               '--topics', *topics]
    if loop:
        command.append('--loop')
    if start_paused:
        command.append('--start-paused')
    return command


def main() -> int:
    """Replay a validated state-only session and optionally open RViz."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--loop', action='store_true')
    parser.add_argument('--start-paused', action='store_true')
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--rviz-delay-s', type=float, default=0.0)
    args = parser.parse_args()
    if args.rate <= 0.0 or args.rviz_delay_s < 0.0:
        parser.error('--rate must be positive and --rviz-delay-s nonnegative')

    root, metadata = _load_session(args.session)
    namespace = metadata['namespace']
    state_topic = _topic(namespace, 'gripper_state_broadcaster/state')
    joint_topic = _topic(namespace, 'joint_states')
    description = root / metadata['description_file']
    bag = root / metadata['bag_directory']
    topics = _validate_topics(metadata)

    children: list[subprocess.Popen] = []
    temporary = tempfile.TemporaryDirectory(prefix='onrobot-replay-')
    try:
        rviz_config = _write_rviz_config(
            Path(temporary.name) / 'replay.rviz', metadata, state_topic)
        mapping = metadata.get('frame_mapping', {})
        frame_prefix = mapping.get('frame_prefix', '')
        robot_command = _robot_state_publisher_command(
            description, joint_topic, frame_prefix)
        children.append(_start_child(robot_command))

        base_parent = metadata.get('base_parent_frame', 'world')
        base_frame = metadata['base_frame']
        if base_parent and base_parent != base_frame:
            xyz = metadata.get('base_xyz_m', [0.0, 0.0, 0.0])
            rpy = metadata.get('base_rpy_rad', [0.0, 0.0, 0.0])
            children.append(_start_child([
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                '--roll', str(rpy[0]), '--pitch', str(rpy[1]),
                '--yaw', str(rpy[2]), '--frame-id', base_parent,
                '--child-frame-id', base_frame,
            ]))

        bag_command = _bag_play_command(
            bag, args.rate, topics, args.loop, args.start_paused)
        children.append(_start_child(bag_command))
        if not args.no_rviz:
            if args.rviz_delay_s:
                time.sleep(args.rviz_delay_s)
            children.append(_start_child(['rviz2', '-d', str(rviz_config)]))

        while children and all(child.process.poll() is None for child in children):
            time.sleep(0.25)
        finished = [child for child in children
                    if child.process.poll() is not None]
        return (0 if all(child.process.returncode == 0 for child in finished)
                else 1)
    except KeyboardInterrupt:
        return 0
    finally:
        for child in reversed(children):
            _signal_child(child, signal.SIGTERM)
        deadline = time.monotonic() + 5.0
        for child in reversed(children):
            remaining = max(0.1, deadline - time.monotonic())
            try:
                child.process.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                _signal_child(child, signal.SIGKILL)
                child.process.wait()
            else:
                # ros2 run may have already exited while its launched ROS
                # executable is still in the private process group. Do not
                # leave that grandchild publishing after replay returns.
                _signal_child(child, signal.SIGKILL)
        temporary.cleanup()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, OSError, ValueError, json.JSONDecodeError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(2)

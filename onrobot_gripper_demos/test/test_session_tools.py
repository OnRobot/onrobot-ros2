"""Unit checks for safe ROS session recording and replay contracts."""

import hashlib
import importlib.util
import json
import subprocess
from pathlib import Path
from tempfile import TemporaryDirectory


PACKAGE = Path(__file__).parents[1]
RECORD_SCRIPT = PACKAGE / 'scripts' / 'record_gripper_session.py'
REPLAY_SCRIPT = PACKAGE / 'scripts' / 'replay_gripper_session.py'


def _load_module(name, path):
    """Load a script without executing its command-line entry point."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_record_topics_are_state_only_and_namespace_aware():
    """Record/replay topics never include an action or command stream."""
    record = _load_module('record_gripper_session', RECORD_SCRIPT)
    namespace = record._namespace('fixture/')
    topics = [record._topic(namespace, suffix)
              for suffix in record.TOPIC_SUFFIXES]

    assert namespace == '/fixture'
    assert topics == [
        '/fixture/joint_states',
        '/fixture/gripper_state_broadcaster/state',
        '/fixture/realtime_controller/state',
        '/fixture/parallel_gripper_limit_broadcaster/values',
        '/fixture/diagnostics',
    ]
    assert all(not any(word in topic for word in ('command', 'action'))
               for topic in topics)


def test_record_qos_overrides_cover_every_state_topic():
    """The recorder opts into a compatible subscription for filtered states."""
    record = _load_module('record_gripper_session_qos', RECORD_SCRIPT)
    topics = [record._topic('/fixture', suffix)
              for suffix in record.TOPIC_SUFFIXES]
    with TemporaryDirectory() as directory:
        path = Path(directory) / 'qos_profile_overrides.yaml'
        record._write_qos_profile_overrides(path, topics)
        source = path.read_text(encoding='utf-8')

    for topic in topics:
        assert f'"{topic}":' in source
    assert source.count('reliability: best_effort') == len(topics)
    assert source.count('durability: volatile') == len(topics)
    assert source.count('history: keep_last') == len(topics)
    assert source.count('depth: 100') == len(topics)


def test_replay_config_contains_history_panel_but_no_actionable_panel():
    """Generated replay RViz contains only read-only state visualization."""
    replay = _load_module('replay_gripper_session', REPLAY_SCRIPT)
    metadata = {'model': '2fg7', 'base_frame': 'base_link'}
    with TemporaryDirectory() as directory:
        config = replay._write_rviz_config(
            Path(directory) / 'replay.rviz', metadata,
            '/fixture/gripper_state_broadcaster/state')
        source = config.read_text(encoding='utf-8')

    assert 'ForceHistoryPanel' in source
    assert 'RobotModel' in source
    assert 'GripperControlPanel' not in source
    assert 'RealtimeControlPanel' not in source
    assert '/fixture/gripper_state_broadcaster/state' in source


def test_replay_validates_description_hash_and_recording_state():
    """Replay accepts an intact completed session and rejects tampering."""
    replay = _load_module('replay_gripper_session', REPLAY_SCRIPT)
    with TemporaryDirectory() as directory:
        root = Path(directory)
        description = root / 'robot_description.urdf'
        description.write_text('<robot name="fixture"/>\n', encoding='utf-8')
        (root / 'bag').mkdir()
        metadata = {
            'schema_version': 1,
            'status': 'complete',
            'model': '2fg7',
            'namespace': '/fixture',
            'topics': [
                '/fixture/joint_states',
                '/fixture/gripper_state_broadcaster/state',
                '/fixture/realtime_controller/state',
                '/fixture/parallel_gripper_limit_broadcaster/values',
                '/fixture/diagnostics',
            ],
            'description_file': description.name,
            'description_sha256': hashlib.sha256(
                description.read_bytes()).hexdigest(),
            'bag_directory': 'bag',
            'base_frame': 'base_link',
        }
        session_file = root / 'session.json'
        session_file.write_text(json.dumps(metadata), encoding='utf-8')
        (root / 'bag' / 'metadata.yaml').write_text(
            'rosbag2_bagfile_information: {}\n', encoding='utf-8')

        loaded_root, loaded = replay._load_session(session_file)
        assert loaded_root == root
        assert loaded['model'] == '2fg7'

        description.write_text('<robot name="tampered"/>\n', encoding='utf-8')
        try:
            replay._load_session(session_file)
        except RuntimeError as error:
            assert 'hash' in str(error)
        else:
            raise AssertionError('tampered session was accepted')


def test_record_provenance_is_compact_and_unknown_is_explicit():
    """Recorder provenance identifies current source/assets without guessing."""
    record = _load_module('record_gripper_session_provenance', RECORD_SCRIPT)

    asset = record._asset_identity('2fg7')
    assert asset['status'] == 'available'
    assert asset['asset_revision'] == '3.0'
    assert asset['asset_entrypoint'].endswith('onrobot_2fg7.usda')
    assert len(asset['contract_sha256']) == 64
    assert isinstance(asset['layer_sha256'], dict)

    source = record._git_identity(Path('/definitely/not-a-checkout'), 'test')
    assert source['status'] == 'unknown'
    assert 'reason' in source

    tool_api = record._tool_api_identity()
    assert isinstance(tool_api, dict)
    assert isinstance(tool_api['source'], dict)
    assert isinstance(tool_api['debian_packages'], dict)


def test_recording_validation_rejects_empty_or_incomplete_bags():
    """A zero-exit rosbag process is not a valid session by itself."""
    record = _load_module('record_gripper_session_validation', RECORD_SCRIPT)
    with TemporaryDirectory() as directory:
        bag = Path(directory) / 'bag'
        bag.mkdir()
        assert 'metadata.yaml' in (record._recording_error(
            bag, {}, '/fixture') or '')
        (bag / 'metadata.yaml').write_text(
            'rosbag2_bagfile_information: {}\n', encoding='utf-8')
        assert 'no topic counts' in (record._recording_error(
            bag, {}, '/fixture') or '')
        counts = {'/fixture/joint_states': 2}
        assert 'gripper_state_broadcaster/state' in (record._recording_error(
            bag, counts, '/fixture') or '')
        counts['/fixture/gripper_state_broadcaster/state'] = 1
        assert record._recording_error(bag, counts, '/fixture') is None


def test_replay_rejects_inconsistent_optional_frame_mapping():
    """Provenance cannot silently describe a different replay frame graph."""
    replay = _load_module('replay_gripper_session_mapping', REPLAY_SCRIPT)
    metadata = {
        'model': '2fg7',
        'namespace': '/fixture',
        'base_frame': 'base_link',
        'base_parent_frame': 'world',
        'frame_mapping': {
            'namespace': '/other',
            'frame_prefix': '',
            'parent_frame': 'world',
            'base_frame': 'base_link',
        },
    }
    try:
        replay._validate_optional_provenance(metadata)
    except RuntimeError as error:
        assert 'namespace' in str(error)
    else:
        raise AssertionError('inconsistent frame mapping was accepted')


def test_recorder_captures_live_description_instead_of_reexpanding_defaults(
        monkeypatch):
    """The normal recording path saves the description the running node uses."""
    record = _load_module('record_gripper_session_live_description', RECORD_SCRIPT)

    class Result:
        stdout = 'String value is: <robot name="live_fixture"/>\n'

    def fake_run(command, **kwargs):
        assert command[-2:] == ['/fixture/robot_state_publisher',
                                'robot_description']
        return Result()

    monkeypatch.setattr(record.subprocess, 'run', fake_run)
    assert record._live_robot_description('/fixture/robot_state_publisher') == (
        '<robot name="live_fixture"/>')


def test_recorder_discards_transport_diagnostics_around_live_description(
        monkeypatch):
    """DDS diagnostics must not corrupt the saved URDF parameter value."""
    record = _load_module('record_gripper_session_noisy_description',
                          RECORD_SCRIPT)

    class Result:
        stdout = ('\x1b[31mtransport warning\x1b[0m\n'
                  'String value is: <robot name="live_fixture">'
                  '<link name="base_link"/></robot>\n'
                  'trailing diagnostic\n')

    monkeypatch.setattr(record.subprocess, 'run', lambda *args, **kwargs: Result())
    assert record._live_robot_description('/fixture/robot_state_publisher') == (
        '<robot name="live_fixture"><link name="base_link"/></robot>')


def test_all_four_replays_restore_prefixed_tf_and_keep_state_only_topics():
    """Every model replays saved state with the same prefixed visual frames."""
    replay = _load_module('replay_gripper_session_all_models', REPLAY_SCRIPT)
    with TemporaryDirectory() as directory:
        description = Path(directory) / 'robot_description.urdf'
        description.write_text('<robot name="fixture"/>\n', encoding='utf-8')
        for model in ('2fg7', '2fg14', 'rg2', 'rg6'):
            namespace = f'/fixture_{model}'
            topics = [replay._topic(namespace, suffix)
                      for suffix in replay.TOPIC_SUFFIXES]
            metadata = {
                'model': model,
                'namespace': namespace,
                'base_frame': 'fixture/base_link',
                'base_parent_frame': 'world',
                'frame_mapping': {
                    'namespace': namespace,
                    'frame_prefix': 'fixture/',
                    'unprefixed_base_frame': 'base_link',
                    'parent_frame': 'world',
                    'base_frame': 'fixture/base_link',
                },
                'topics': topics,
            }
            replay._validate_topics(metadata)
            replay._validate_optional_provenance(metadata)
            robot_command = replay._robot_state_publisher_command(
                description, replay._topic(namespace, 'joint_states'),
                metadata['frame_mapping']['frame_prefix'])
            assert 'frame_prefix:=fixture/' in robot_command
            assert f'joint_states:={namespace}/joint_states' in robot_command
            bag_command = replay._bag_play_command(
                Path(directory) / 'bag', 0.5, topics, True, True)
            assert '--loop' in bag_command and '--start-paused' in bag_command
            assert all('command' not in value and 'action' not in value
                       for value in bag_command)


def test_replay_rejects_namespace_and_frame_mapping_injection():
    """Session metadata cannot alter the generated RViz configuration."""
    replay = _load_module('replay_gripper_session_validation', REPLAY_SCRIPT)
    try:
        replay._validate_namespace('/fixture\nPanels')
    except RuntimeError:
        pass
    else:
        raise AssertionError('newline namespace was accepted')
    try:
        replay._validate_frame('fixture/../base_link', 'base frame')
    except RuntimeError:
        pass
    else:
        raise AssertionError('unsafe frame was accepted')


def test_provenance_records_dirty_diff_and_untracked_content():
    """A dirty checkout cannot be represented as an indistinguishable HEAD."""
    record = _load_module('record_gripper_session_git_identity', RECORD_SCRIPT)
    with TemporaryDirectory() as directory:
        root = Path(directory)
        subprocess.run(['git', 'init', '-q', str(root)], check=True)
        subprocess.run(['git', '-C', str(root), 'config', 'user.email',
                        'test@example.invalid'], check=True)
        subprocess.run(['git', '-C', str(root), 'config', 'user.name', 'test'],
                       check=True)
        tracked = root / 'tracked.txt'
        tracked.write_text('base\n', encoding='utf-8')
        subprocess.run(['git', '-C', str(root), 'add', 'tracked.txt'], check=True)
        subprocess.run(['git', '-C', str(root), 'commit', '-qm', 'base'], check=True)
        tracked.write_text('changed\n', encoding='utf-8')
        untracked = root / 'new.txt'
        untracked.write_text('new\n', encoding='utf-8')

        identity = record._git_identity(root, 'fixture')
        assert identity['tree_state'] == 'dirty'
        assert identity['tracked_diff_sha256'] != hashlib.sha256(b'').hexdigest()
        assert identity['untracked_file_sha256']['new.txt'] == hashlib.sha256(
            b'new\n').hexdigest()


def test_replay_rejects_changed_serialized_state_streams(monkeypatch):
    """A session manifest binds replay to the state bytes that were recorded."""
    replay = _load_module('replay_gripper_session_stream_identity', REPLAY_SCRIPT)
    topics = ['/fixture/joint_states',
              '/fixture/gripper_state_broadcaster/state']
    metadata = {
        'recorded_state_streams': {
            'status': 'available',
            'sample_counts': {topic: 1 for topic in topics},
            'sample_sha256': {topic: 'a' * 64 for topic in topics},
        },
    }
    monkeypatch.setattr(
        replay, '_bag_state_stream_identity',
        lambda bag, selected_topics: {
            'sample_counts': {topic: 1 for topic in selected_topics},
            'sample_sha256': {topic: 'b' * 64 for topic in selected_topics},
        })
    try:
        replay._validate_recorded_state_streams(metadata, Path('/bag'), topics)
    except RuntimeError as error:
        assert 'no longer match' in str(error)
    else:
        raise AssertionError('changed serialized state was accepted')

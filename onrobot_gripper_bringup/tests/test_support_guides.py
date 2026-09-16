"""Execute guide argument parsing without touching a physical endpoint."""
import argparse
import re
import shlex
import subprocess
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from ros2bag.verb.record import RecordVerb

from test_usb_rtu_guide import load_launch

ROOT = Path(__file__).resolve().parents[2]
START = ROOT / 'GETTING_STARTED.md'
REPORT = ROOT / 'onrobot_gripper_bringup/doc/REPORTING_ISSUES.md'


@pytest.mark.parametrize('path', [START, REPORT])
def test_shell_examples_parse(path):
    for block in re.findall(r'```bash\n(.*?)```', path.read_text(), re.S):
        subprocess.run(['bash', '-n'], input=block, text=True, check=True)


@pytest.mark.parametrize('model', ['2fg7', '2fg14', 'rg2', 'rg6'])
def test_getting_started_200hz_is_forwarded_to_bringup(monkeypatch, model):
    launch = load_launch('onrobot_gripper_demos', 'showcase.launch.py')
    monkeypatch.setattr(launch, 'get_package_share_directory', lambda n: str(ROOT / n))
    examples = re.findall(r'```bash\n(.*?)```', START.read_text(), re.S)
    command = next(b for b in examples if 'control:=realtime' in b and 'transport:=rtu' in b)
    argv = shlex.split(command.replace('\\\n', ' '))
    values = dict(v.split(':=', 1) for v in argv[4:])
    values['model'] = model
    context = LaunchContext()
    context.launch_configurations.update(values)
    for d in launch.generate_launch_description().entities:
        if isinstance(d, DeclareLaunchArgument):
            d.execute(context)
    included = launch._include_selected_model(context)[0]
    actual = {k: perform_substitutions(context, normalize_to_list_of_substitutions(v))
              for k, v in included.launch_arguments}
    assert actual['model'] == model
    assert actual['serial_device'] == '/dev/ttyUSB0'
    assert actual['transport'] == 'rtu'
    assert actual['start_realtime_controller'] == 'true'
    for key in ['realtime_update_rate_hz', 'controller_manager_update_rate_hz',
                'state_publish_rate_hz']:
        assert actual[key] == '200'


@pytest.mark.parametrize('namespace', ['', '/gripper_a'])
def test_diagnostic_bag_command_with_installed_jazzy_parser(namespace):
    example = next(b for b in re.findall(r'```bash\n(.*?)```', REPORT.read_text(), re.S)
                   if 'ros2 bag record' in b)
    command = example[example.index('ros2 bag record'):].replace('\\\n', ' ')
    command = command.replace('$ONROBOT_NS', namespace).replace('$ONROBOT_REPORT', '/report')
    parser = argparse.ArgumentParser()
    RecordVerb().add_arguments(parser, cli_name='ros2 bag record')
    args = parser.parse_args(shlex.split(command)[3:])
    assert args.storage == 'mcap'
    assert args.output == '/report/bag'
    assert args.include_hidden_topics
    assert not args.all and not args.all_topics
    for topic in ['joint_state_broadcaster/joint_states', 'joint_states',
                  'gripper_state_broadcaster/state', 'diagnostics',
                  'realtime_controller/command', 'realtime_controller/state']:
        assert f'{namespace}/{topic}' in args.topics
    assert set(['/tf', '/tf_static', '/rosout', '/clock']) <= set(args.topics)


def test_public_build_is_consistent_with_test_free_archive():
    for path in [START, ROOT / 'README.md']:
        text = re.sub(r'\\\s*\n\s*', ' ', path.read_text())
        commands = re.findall(r'^colcon build.*$', text, re.M)
        assert commands
        assert all('-DBUILD_TESTING=OFF' in c for c in commands)


def test_ros_tests_do_not_include_private_api_headers():
    for path in (ROOT / 'onrobot_gripper_hardware/tests').rglob('*.cpp'):
        assert 'onrobot_tool_api/testing/' not in path.read_text()

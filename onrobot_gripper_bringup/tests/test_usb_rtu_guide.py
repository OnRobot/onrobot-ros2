"""Check USB guide shell syntax and documented launch arguments without I/O."""

import importlib.util
import re
import shlex
import subprocess
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions

import pytest


ROOT = Path(__file__).resolve().parents[2]
GUIDE = ROOT / 'onrobot_gripper_bringup/doc/USB_RTU.md'


def blocks():
    """Return the literal shell examples in the public guide."""
    return re.findall(r'```bash\n(.*?)```', GUIDE.read_text(), re.DOTALL)


def load_launch(package, name):
    """Import a launch description without starting any process."""
    spec = importlib.util.spec_from_file_location(
        package, ROOT / package / 'launch' / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_guide_shell_examples_parse_without_executing():
    """Catch copy/paste syntax errors; never execute serial or sudo examples."""
    examples = blocks()
    assert len(examples) == 6
    for example in examples:
        subprocess.run(['bash', '-n'], input=example, text=True, check=True)


@pytest.mark.parametrize('model', ['2fg7', '2fg14', 'rg2', 'rg6'])
def test_documented_usb_launches_forward_settings(monkeypatch, model):
    """Resolve both documented modes through showcase and common bringup."""
    showcase = load_launch('onrobot_gripper_demos', 'showcase.launch.py')
    bringup = load_launch('onrobot_gripper_bringup', 'gripper.launch.py')
    monkeypatch.setattr(showcase, 'get_package_share_directory',
                        lambda name: str(ROOT / name))
    monkeypatch.setattr(bringup, 'get_package_share_directory',
                        lambda name: str(ROOT / name))
    examples = [item for item in blocks() if item.startswith('ros2 launch')]
    assert len(examples) == 2
    for example in examples:
        example = example.replace('$ONROBOT_MODEL', model).replace(
            '$ONROBOT_SERIAL_DEVICE', '/dev/serial/by-id/test-adapter')
        args = shlex.split(example.replace('\\\n', ' '))
        assert args[:4] == [
            'ros2', 'launch', 'onrobot_gripper_demos', 'showcase.launch.py']
        given = dict(item.split(':=', 1) for item in args[4:])
        context = LaunchContext()
        declarations = {
            item.name: item
            for item in showcase.generate_launch_description().entities
            if isinstance(item, DeclareLaunchArgument)}
        assert not set(given) - declarations.keys()
        context.launch_configurations.update(given)
        for declaration in declarations.values():
            declaration.execute(context)
        included = showcase._include_selected_model(context)[0]
        assert isinstance(included, IncludeLaunchDescription)
        forwarded = {
            key: perform_substitutions(
                context, normalize_to_list_of_substitutions(value))
            for key, value in included.launch_arguments}
        assert forwarded['model'] == model
        assert forwarded['transport'] == 'rtu'
        assert forwarded['serial_device'] == '/dev/serial/by-id/test-adapter'
        assert forwarded['baud_rate'] == '1000000'
        assert forwarded['rtu_parity'] == 'even'
        assert forwarded['slave_id'] == '65'
        realtime = given['control'] == 'realtime'
        assert forwarded['start_realtime_controller'] == str(realtime).lower()
        if realtime:
            assert forwarded['realtime_update_rate_hz'] == '200'
            assert forwarded['controller_manager_update_rate_hz'] == '200'
            assert forwarded['state_publish_rate_hz'] == '200'
        # Apply the common launch's argument validation too, without launching
        # the selected hardware plugin or touching the serial path.
        common_context = LaunchContext()
        common_context.launch_configurations.update(forwarded)
        for declaration in bringup.generate_launch_description().entities:
            if isinstance(declaration, DeclareLaunchArgument):
                declaration.execute(common_context)

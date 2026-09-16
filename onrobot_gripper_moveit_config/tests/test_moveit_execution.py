"""Exercise planning and execution through MoveIt for every parallel model."""

import os
import json
import signal
import subprocess
import time
import uuid
from pathlib import Path

import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from onrobot_gripper_msgs.msg import GripperState
from controller_manager_msgs.srv import ListControllers
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import yaml


MODELS = ('2fg7', '2fg14', 'rg2', 'rg6')


def _stop_launch(process):
    """Stop a launch process group without leaving controller managers."""
    if process.poll() is not None:
        return
    # Let launch supervise child shutdown. Broadcasting to both launch and its
    # children at once can deliver a second SIGINT during middleware teardown.
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=15)
        return
    except subprocess.TimeoutExpired:
        pass
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=5)
        return
    except subprocess.TimeoutExpired:
        pass
    os.killpg(process.pid, signal.SIGKILL)
    process.wait(timeout=5)


@pytest.mark.parametrize('model', MODELS)
def test_moveit_plans_and_executes_standard_gripper_action(model, tmp_path):
    """Require a successful OMPL-to-standard-action execution per model."""
    log_path = tmp_path / f'{model}_moveit.log'
    namespace = 'moveit_test_' + uuid.uuid4().hex[:10]
    context = rclpy.Context()
    rclpy.init(context=context)
    node = Node('observer', namespace=namespace, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    joints, typed = [], []
    node.create_subscription(JointState, 'joint_states', joints.append, qos_profile_sensor_data)
    node.create_subscription(GripperState, 'gripper_state_broadcaster/state', typed.append, qos_profile_sensor_data)
    controllers = node.create_client(ListControllers, 'controller_manager/list_controllers')
    joint = 'finger_stroke' if model.startswith('2fg') else 'finger_joint'
    target = 0.01 if model.startswith('2fg') else 0.3
    motion_passed = False

    def wait_for(predicate, timeout=30.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)
            if predicate():
                return
        pytest.fail(f'{model} state/controller readiness failed:\n{log_path.read_text()}')

    with log_path.open('w', encoding='utf-8') as log:
        launch = subprocess.Popen(
            [
                'ros2', 'launch', 'onrobot_gripper_moveit_config',
                'moveit_gripper.launch.py', f'model:={model}',
                'backend:=fake',
                f'namespace:={namespace}',
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )
        try:
            wait_for(controllers.service_is_ready)
            # Bringup deliberately starts controllers sequentially. DDS service
            # discovery can consume several spawner retries on a shared runner;
            # readiness, not repeated motion requests, is the prerequisite.
            deadline = time.monotonic() + 90.0
            while True:
                listing = controllers.call_async(ListControllers.Request())
                wait_for(listing.done, 5.0)
                states = {c.name: c.state for c in listing.result().controller}
                if all(states.get(name) == 'active' for name in (
                        'gripper_controller', 'joint_state_broadcaster',
                        'gripper_state_broadcaster', 'parallel_gripper_limit_broadcaster')):
                    break
                assert time.monotonic() < deadline, states
                time.sleep(0.1)
            wait_for(lambda: len(joints) >= 10 and len(typed) >= 10 and
                     typed[-1].sample_sequence > typed[-5].sample_sequence)
            last_smoke = subprocess.run(
                    [
                        'ros2', 'run', 'onrobot_gripper_moveit_config',
                        'moveit_gripper_smoke', '--ros-args',
                        '-p', ('target:=0.01' if model.startswith('2fg') else 'target:=0.3'),
                        '-p', ('joint:=finger_stroke' if model.startswith('2fg') else 'joint:=finger_joint'),
                        '-r', '__ns:=/' + namespace,
                    ],
                    capture_output=True,
                    text=True,
                    timeout=35,
                    check=False,
                )
            assert last_smoke.returncode == 0, (last_smoke.stdout + last_smoke.stderr + log_path.read_text())

            smoke_output = last_smoke.stdout + last_smoke.stderr
            assert f'MoveIt executed {joint} target' in smoke_output
            def physical_target_received():
                if not joints or joint not in joints[-1].name:
                    return False
                return abs(joints[-1].position[joints[-1].name.index(joint)] - target) < 1e-4
            wait_for(physical_target_received, 5.0)
            sample = joints[-1]
            measured_q = sample.position[sample.name.index(joint)]
            profile = Path(get_package_share_directory('onrobot_gripper_description')) / 'config/planning_profiles' / (model + '.yaml')
            resolver = Path(get_package_prefix('onrobot_gripper_description')) / 'bin/resolve_gripper_profile'
            mapped = yaml.safe_load(subprocess.check_output(
                [str(resolver), '--profile', str(profile), '--joint', str(measured_q)], text=True))
            assert sample.position[sample.name.index('grip_stroke')] == pytest.approx(mapped['aperture_m'], abs=1e-6)
            motion_passed = True
        finally:
            _stop_launch(launch)
            executor.shutdown()
            node.destroy_node()
            context.shutdown()
            # Keep motion and teardown evidence separate: a successful action
            # cannot hide a crashed child during launch shutdown.
            text = log_path.read_text()
            shutdown_clean = 'Segmentation fault' not in text and 'exit code -11' not in text
            (tmp_path / 'result.json').write_text(json.dumps({
                'model': model, 'motion_passed': motion_passed,
                'shutdown_clean': shutdown_clean, 'launch_exit_code': launch.returncode,
            }, indent=2) + '\n')
    assert shutdown_clean, f'{model} MoveIt teardown failed:\n{text[-9000:]}'

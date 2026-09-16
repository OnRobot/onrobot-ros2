"""Execute the installed adapter against a deterministic ROS action peer.

No device connection or real command endpoint is used. Events control delayed
acceptance, cancellation and results so these are behavior tests, not source
text assertions. Each test uses a private ROS namespace.
"""

import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from control_msgs.action import ParallelGripperCommand as Action
from control_msgs.msg import Float64Values
from onrobot_gripper_msgs.msg import GripperState
from sensor_msgs.msg import JointState
import yaml


def wait_until(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError('condition did not become true within timeout')


def completed(future, timeout=5.0):
    wait_until(future.done, timeout)
    return future.result()


class Peer:
    def __init__(self, model, tmp_path, prefix=''):
        self.model, self.prefix = model, prefix
        self.namespace = '/adapter_test_' + uuid.uuid4().hex[:12]
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.node = Node('peer', namespace=self.namespace, context=self.context)
        self.group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        self.executor.add_node(self.node)
        self.stop = threading.Event()
        self.accept = threading.Event()
        self.accept.set()
        self.release = threading.Event()
        self.release.set()
        self.cancel_seen = threading.Event()
        self.goal_seen = threading.Event()
        self.cancel_terminal = threading.Event()
        self.cancel_terminal.set()
        self.commands, self.feedback = [], []
        self.running = 0
        self.max_running = 0
        self.publish = True
        self.freeze = False
        self.sequence = 0
        self.fault = False
        self.bad_feedback = False
        self.bad_result = False
        self.false_success = False
        self.reject_goal = False
        self.joint_bias = 0.0
        self.physical = prefix + ('finger_stroke' if model.startswith('2fg') else 'finger_joint')
        self.task = prefix + 'grip_stroke'
        self.profile = Path(get_package_share_directory('onrobot_gripper_description')) / 'config/planning_profiles' / (model + '.yaml')
        self.resolver = Path(get_package_prefix('onrobot_gripper_description')) / 'bin/resolve_gripper_profile'
        resolved = yaml.safe_load(subprocess.check_output(
            [str(self.resolver), '--profile', str(self.profile)], text=True))
        self.joint_min = resolved['safe_q_domain']['minimum']
        self.joint_max = resolved['safe_q_domain']['maximum']
        self.q = (self.joint_min + self.joint_max) / 2.0
        self.aperture = self.convert(joint=self.q)['aperture_m']
        self.limits = [resolved['safe_aperture']['minimum'], resolved['safe_aperture']['maximum']]
        self.publishers = [
            self.node.create_publisher(GripperState, 'gripper_state_broadcaster/state', qos_profile_sensor_data),
            self.node.create_publisher(JointState, 'joint_states', qos_profile_sensor_data),
            self.node.create_publisher(Float64Values, 'parallel_gripper_limit_broadcaster/values', qos_profile_sensor_data),
        ]
        self.timer = self.node.create_timer(0.02, self.emit, callback_group=self.group)
        self.server = ActionServer(self.node, Action, 'gripper_controller/gripper_cmd',
                                   execute_callback=self.execute, goal_callback=self.goal,
                                   cancel_callback=self.cancel, callback_group=self.group)
        self.client = ActionClient(self.node, Action, 'physical_gripper_controller/gripper_cmd',
                                   callback_group=self.group)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.log = (tmp_path / (model + '_adapter.log')).open('w+')
        executable = Path(get_package_prefix('onrobot_gripper_moveit_config')) / 'lib/onrobot_gripper_moveit_config/physical_gripper_adapter'
        self.process = subprocess.Popen([
            str(executable), '--ros-args', '-r', '__ns:=' + self.namespace,
            '-p', 'profile:=' + str(self.profile), '-p', 'profile_hash:=' + resolved['profile_hash'],
            '-p', 'joint_prefix:=' + prefix, '-p', 'state_timeout_s:=0.3',
            '-p', 'goal_timeout_s:=3.0'], stdout=self.log, stderr=subprocess.STDOUT)
        assert self.client.wait_for_server(timeout_sec=10.0)
        wait_until(lambda: all(p.get_subscription_count() for p in self.publishers))
        time.sleep(0.15)

    def convert(self, *, joint=None, aperture=None):
        value = ['--joint', str(joint)] if joint is not None else ['--aperture', str(aperture)]
        return yaml.safe_load(subprocess.check_output(
            [str(self.resolver), '--profile', str(self.profile), *value], text=True))

    def emit(self):
        if not self.publish:
            return
        self.sequence += not self.freeze
        state = GripperState(model=self.model, sample_sequence=self.sequence,
                             task_aperture_valid=True, task_aperture=self.aperture,
                             mapping_validity=GripperState.MAPPING_VALID,
                             connection_state=GripperState.CONNECTION_IDLE,
                             fault_code=1 if self.fault else 0)
        state.header.stamp = self.node.get_clock().now().to_msg()
        self.publishers[0].publish(state)
        self.publishers[1].publish(JointState(name=[self.physical, self.task],
                                            position=[self.q + self.joint_bias, self.aperture]))
        self.publishers[2].publish(Float64Values(values=self.limits))

    def goal(self, goal):
        self.commands.append(goal.command)
        self.goal_seen.set()
        self.accept.wait(5.0)
        return GoalResponse.REJECT if self.reject_goal else GoalResponse.ACCEPT

    def cancel(self, _goal):
        self.cancel_seen.set()
        return CancelResponse.ACCEPT

    def execute(self, goal):
        self.running += 1
        self.max_running = max(self.max_running, self.running)
        try:
            while not self.stop.is_set():
                if goal.is_cancel_requested and self.cancel_terminal.is_set():
                    goal.canceled()
                    return Action.Result(state=JointState(name=[self.task], position=[self.aperture]))
                state = JointState(name=[self.task], position=[float('nan') if self.bad_feedback else self.aperture])
                goal.publish_feedback(Action.Feedback(state=state))
                if self.release.wait(0.01):
                    break
            if self.stop.is_set():
                goal.abort()
                return Action.Result()
            aperture = goal.request.command.position[0]
            joint = self.convert(aperture=aperture)['joint_position']
            # Never publish a task value from the new sample while the slower
            # CLI conversion is still producing its physical coordinate.
            self.q, self.aperture = joint, aperture
            state = JointState(name=[self.task], position=[float('nan') if self.bad_result else self.aperture])
            goal.succeed()
            return Action.Result(state=state, reached_goal=not self.false_success)
        finally:
            self.running -= 1

    def send(self, q=None, *, name=None, velocity=None, effort=None):
        goal = Action.Goal(command=JointState(
            name=[self.physical if name is None else name],
            position=[self.q if q is None else q],
            velocity=[] if velocity is None else velocity, effort=[] if effort is None else effort))
        return completed(self.client.send_goal_async(goal, feedback_callback=self.feedback.append))

    def close(self):
        self.accept.set()
        self.cancel_terminal.set()
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.stop.set()
        self.release.set()
        self.executor.shutdown(timeout_sec=5.0)
        self.thread.join(timeout=5.0)
        self.server.destroy()
        self.client.destroy()
        self.node.destroy_node()
        self.context.shutdown()
        self.log.close()


@pytest.fixture
def peer(request, tmp_path):
    model = getattr(request, 'param', '2fg7')
    instance = Peer(model, tmp_path, prefix='fixture_')
    try:
        yield instance
    finally:
        instance.close()


@pytest.mark.parametrize('peer', ['2fg7', '2fg14', 'rg2', 'rg6'], indirect=True)
def test_round_trip_names_units_feedback_and_no_optional_effort(peer):
    for q in [peer.joint_min, peer.joint_max]:
        handle = peer.send(q)
        assert handle.accepted
        result = completed(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_SUCCEEDED
        assert result.result.reached_goal
        assert result.result.state.name == [peer.physical]
        assert result.result.state.position == pytest.approx([q], abs=1e-12)
        assert len(result.result.state.velocity) == len(result.result.state.effort) == 0
        command = peer.commands[-1]
        assert command.name == [peer.task]
        assert command.position == pytest.approx([peer.convert(joint=q)['aperture_m']])
        assert len(command.velocity) == len(command.effort) == 0
    assert peer.feedback and all(f.feedback.state.name == [peer.physical] for f in peer.feedback)


@pytest.mark.parametrize('options', [dict(q=float('nan')), dict(q=-1.), dict(q=10.),
                                  dict(name='grip_stroke'), dict(velocity=[0.01]), dict(effort=[0.0])])
def test_invalid_physical_request_never_reaches_task_action(peer, options):
    assert not peer.send(**options).accepted
    assert peer.commands == []


@pytest.mark.parametrize('failure', ['frozen', 'fault', 'mismatch', 'stale'])
def test_invalid_state_rejects_new_motion(peer, failure):
    if failure == 'frozen':
        peer.freeze = True
    elif failure == 'fault':
        peer.fault = True
    elif failure == 'mismatch':
        peer.joint_bias = 0.01
    else:
        peer.publish = False
    time.sleep(0.4)
    assert not peer.send().accepted
    assert peer.commands == []


def test_cancellation_before_delayed_downstream_acceptance(peer):
    peer.release.clear()
    peer.accept.clear()
    handle = peer.send()
    assert handle.accepted
    assert peer.goal_seen.wait(3.0)
    assert completed(handle.cancel_goal_async()).goals_canceling
    peer.accept.set()
    assert peer.cancel_seen.wait(3.0)
    assert completed(handle.get_result_async()).status == GoalStatus.STATUS_CANCELED


def test_preemption_waits_for_previous_terminal_result(peer):
    peer.release.clear()
    peer.cancel_terminal.clear()
    first = peer.send()
    assert first.accepted and peer.goal_seen.wait(3.0)
    second = peer.send(peer.joint_min)
    assert second.accepted and peer.cancel_seen.wait(3.0)
    time.sleep(0.1)
    assert len(peer.commands) == 1
    peer.cancel_terminal.set()
    assert completed(first.get_result_async()).status == GoalStatus.STATUS_ABORTED
    wait_until(lambda: len(peer.commands) == 2)
    peer.release.set()
    assert completed(second.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED
    assert peer.max_running == 1


@pytest.mark.parametrize('failure', ['fault', 'stale', 'limits', 'feedback'])
def test_running_motion_cancels_on_invalid_state(peer, failure):
    peer.release.clear()
    handle = peer.send()
    assert handle.accepted and peer.goal_seen.wait(3.0)
    if failure == 'fault':
        peer.fault = True
    elif failure == 'stale':
        peer.publish = False
    elif failure == 'limits':
        peer.limits = [peer.limits[0], peer.aperture - 0.001]
    else:
        peer.bad_feedback = True
    assert peer.cancel_seen.wait(3.0)
    assert completed(handle.get_result_async()).status == GoalStatus.STATUS_ABORTED


@pytest.mark.parametrize('failure', ['bad_result', 'false_success', 'reject_goal'])
def test_invalid_downstream_terminal_result_is_not_success(peer, failure):
    setattr(peer, failure, True)
    handle = peer.send()
    assert handle.accepted
    assert completed(handle.get_result_async()).status == GoalStatus.STATUS_ABORTED


def test_unacknowledged_cancel_inhibits_following_motion(peer):
    peer.release.clear()
    peer.cancel_terminal.clear()
    handle = peer.send()
    assert handle.accepted and peer.goal_seen.wait(3.0)
    completed(handle.cancel_goal_async())
    assert completed(handle.get_result_async()).status == GoalStatus.STATUS_CANCELED
    assert not peer.send().accepted
    assert len(peer.commands) == 1


def test_sigint_cancels_downstream_before_shutdown(peer):
    peer.release.clear()
    handle = peer.send()
    assert handle.accepted and peer.goal_seen.wait(3.0)
    peer.process.send_signal(signal.SIGINT)
    assert peer.cancel_seen.wait(3.0)
    assert completed(handle.get_result_async()).status == GoalStatus.STATUS_ABORTED
    assert peer.process.wait(timeout=4.0) == 0


def test_shutdown_reports_unconfirmed_stop_as_failure(peer):
    peer.release.clear()
    peer.cancel_terminal.clear()
    assert peer.send().accepted and peer.goal_seen.wait(3.0)
    peer.process.send_signal(signal.SIGINT)
    assert peer.cancel_seen.wait(3.0)
    assert peer.process.wait(timeout=4.0) == 2


@pytest.mark.parametrize('peer', ['2fg7', '2fg14', 'rg2', 'rg6'], indirect=True)
def test_small_measured_contact_compression_does_not_change_command_limits(peer):
    peer.q = peer.joint_min
    peer.aperture = peer.limits[0] - 0.0001
    time.sleep(0.1)
    handle = peer.send((peer.joint_min + peer.joint_max) / 2)
    assert handle.accepted
    assert completed(handle.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED
    assert not peer.send(peer.joint_min - 0.000001).accepted


def test_binary_roundoff_of_live_endpoint_does_not_reject_legal_physical_target(peer):
    peer.limits[-1] = 0.071
    time.sleep(0.1)
    handle = peer.send(peer.joint_max)
    assert handle.accepted
    assert completed(handle.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED
    assert peer.commands[-1].position[0] == 0.071

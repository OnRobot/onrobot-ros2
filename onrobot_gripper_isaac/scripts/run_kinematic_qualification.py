#!/usr/bin/python3
"""Run the repeatable OnRobot gripper kinematic qualification workflow."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from action_msgs.msg import GoalStatus

from ament_index_python.packages import PackageNotFoundError, get_package_prefix

from control_msgs.action import ParallelGripperCommand

from controller_manager_msgs.srv import ListControllers, SwitchController

from isaac_model_contract import coordinate_metadata
from isaac_model_contract import default_asset
from isaac_model_contract import driven_joint
from isaac_model_contract import joint_limits
from isaac_model_contract import load_contract
from isaac_model_contract import package_root
from isaac_model_contract import SUPPORTED_MODELS

from onrobot_gripper_msgs.msg import GripperState, RealtimeCommand
from onrobot_gripper_msgs.msg import RealtimeState

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy

from rosgraph_msgs.msg import Clock

from rcl_interfaces.srv import GetParameters

from sensor_msgs.msg import JointState

from std_srvs.srv import Trigger


RG_MODELS = frozenset(('rg2', 'rg6'))


def _default_bridge() -> Path:
    root = package_root()
    source = root / 'scripts/run_ros_bridge.py'
    if source.is_file():
        return source
    prefix = Path(__file__).resolve().parents[2]
    return prefix / 'lib/onrobot_gripper_isaac/run_ros_bridge.py'


def _fq(namespace: str, name: str) -> str:
    namespace = namespace.strip('/')
    return f'/{namespace}/{name}' if namespace else f'/{name}'


def _sha256(path: Path) -> str:
    """Return a stable digest for an executable qualification input."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _asset_manifest(entrypoint: Path) -> dict:
    """Hash every file in the layered asset beside the entry point."""
    root = entrypoint.resolve().parent
    return {
        str(path.relative_to(root)): _sha256(path)
        for path in sorted(root.rglob('*'))
        if path.is_file()
    }


def _terminate(process: subprocess.Popen | None, timeout: float = 15.0):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=timeout)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.0)


def _descendant_process_ids(root_pid: int,
                            proc_root: Path = Path('/proc')) -> set[int]:
    """Return a best-effort snapshot of one process tree, including its root."""
    discovered = {root_pid}
    pending = [root_pid]
    while pending:
        pid = pending.pop()
        children = proc_root / str(pid) / 'task' / str(pid) / 'children'
        try:
            child_ids = {
                int(value) for value in children.read_text(
                    encoding='utf-8').split()
            }
        except (OSError, ValueError):
            continue
        for child_pid in child_ids - discovered:
            discovered.add(child_pid)
            pending.append(child_pid)
    return discovered


def _mapped_library_identity(process: subprocess.Popen | None,
                             library_name: str,
                             proc_root: Path = Path('/proc')) -> list[dict]:
    """Return hashes of mapped plugins in the spawned bringup process tree.

    This is intentionally limited to the bringup process that this runner starts
    and its descendants.  ROS launch itself does not map hardware or controller
    plugins; the descendant ros2_control_node does.
    A package prefix only proves what was on the environment path; the mapped
    file is the relevant identity for a controller behaviour claim.
    """
    if process is None or process.poll() is not None:
        return []
    mapped_paths = set()
    for pid in _descendant_process_ids(process.pid, proc_root):
        try:
            lines = (proc_root / str(pid) / 'maps').read_text(
                encoding='utf-8').splitlines()
        except OSError:
            continue
        mapped_paths.update(
            Path(line.rsplit(maxsplit=1)[-1])
            for line in lines if line.rstrip().endswith(library_name)
        )
    result = []
    for path in sorted(mapped_paths):
        try:
            result.append({
                'path': str(path),
                'sha256': _sha256(path),
            })
        except OSError:
            # A process may unload while it is shutting down.  Preserve the
            # path rather than replacing its identity with a guessed hash.
            result.append({'path': str(path), 'sha256': None})
    return result


def _package_prefixes(packages: tuple[str, ...]) -> dict:
    """Resolve the package prefixes actually visible to this runner."""
    result = {}
    for package in packages:
        try:
            result[package] = get_package_prefix(package)
        except PackageNotFoundError:
            result[package] = None
    return result


def _required_controller_states():
    """Return the controller graph required by every qualification phase."""
    return {
        'joint_state_broadcaster': 'active',
        'parallel_gripper_limit_broadcaster': 'active',
        'gripper_state_broadcaster': 'active',
        'recovery_controller': 'active',
        'gripper_controller': 'active',
        'realtime_controller': 'inactive',
    }


class QualificationNode:
    """ROS clients and observations used by the kinematic workflow."""

    def __init__(self, namespace: str, model: str,
                 task_limits: tuple[float, float],
                 physical_limits: tuple[float, float],
                 physical_joint: str):
        """Create namespaced clients and state observers."""
        # The controller-manager and Isaac bridge use /clock.  Stamp the
        # streamed realtime commands in that same domain: wall-epoch stamps
        # are in the controller's future and are deliberately rejected.
        self.node = rclpy.create_node(
            f'onrobot_{model}_kinematic_qualification',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True),
            ])
        self.namespace = namespace.strip('/')
        self.task_min_m, self.task_max_m = task_limits
        self.physical_min_m, self.physical_max_m = physical_limits
        self.physical_joint = physical_joint
        # RG models expose a linear task aperture to ROS but stream angular
        # mechanism velocity to the realtime controller and Isaac bridge.
        # Keep this family boundary explicit: treating RG6 as a 2FG would send
        # a task-linear velocity to an unclaimed command interface and compare
        # its angular state with a linear mapping.
        self.rg_model = model in RG_MODELS
        self.joint_state = None
        self.gripper_state = None
        self.realtime_state = None
        self.clock_samples = []
        self.task_samples = []
        self.gripper_state_rx_count = 0
        self.realtime_state_rx_count = 0
        self.clock_rx_count = 0
        self.last_realtime_publication = None
        self.realtime_activation_observation = None
        self.action_observations = []
        self._action_observations_by_handle = {}
        self.node.create_subscription(
            JointState, _fq(namespace, 'joint_states'),
            self._joint_state, qos_profile_sensor_data)
        self.node.create_subscription(
            GripperState,
            _fq(namespace, 'gripper_state_broadcaster/state'),
            self._gripper_state, qos_profile_sensor_data)
        self.node.create_subscription(
            RealtimeState, _fq(namespace, 'realtime_controller/state'),
            self._realtime_state, qos_profile_sensor_data)
        self.node.create_subscription(
            Clock, '/clock', self._clock, qos_profile_sensor_data)
        self.action = ActionClient(
            self.node, ParallelGripperCommand,
            _fq(namespace, 'gripper_controller/gripper_cmd'))
        self.realtime_publisher = self.node.create_publisher(
            RealtimeCommand,
            _fq(namespace, 'realtime_controller/command'),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.stop_client = self.node.create_client(
            Trigger, _fq(namespace, 'realtime_controller/stop'))
        self.switch_client = self.node.create_client(
            SwitchController,
            _fq(namespace, 'controller_manager/switch_controller'))
        self.controller_list_client = self.node.create_client(
            ListControllers,
            _fq(namespace, 'controller_manager/list_controllers'))
        self.realtime_parameter_client = self.node.create_client(
            GetParameters,
            _fq(namespace, 'realtime_controller/get_parameters'))

    def destroy(self):
        """Release the ROS entities owned by the qualification node."""
        self.action.destroy()
        self.node.destroy_node()

    def _joint_state(self, message):
        self.joint_state = message

    def _gripper_state(self, message):
        self.gripper_state = message
        self.gripper_state_rx_count += 1
        if message.task_aperture_valid:
            self.task_samples.append((time.monotonic(), message.task_aperture))
            self.task_samples = self.task_samples[-1000:]

    def _realtime_state(self, message):
        self.realtime_state = message
        self.realtime_state_rx_count += 1

    def _clock(self, message):
        value = message.clock.sec + message.clock.nanosec * 1e-9
        self.clock_rx_count += 1
        self.clock_samples.append(value)
        self.clock_samples = self.clock_samples[-100:]

    def spin_until(self, predicate, timeout: float, message: str):
        """Spin until a condition is true or raise with useful context."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(message)

    def spin_for(self, duration: float):
        """Spin callbacks for a monotonic duration."""
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def task_position(self) -> float:
        """Return the latest valid semantic aperture."""
        if (self.gripper_state is None or
                not self.gripper_state.task_aperture_valid):
            return math.nan
        return self.gripper_state.task_aperture

    def physical_position(self) -> float:
        """Return the latest physical articulation coordinate."""
        if (self.joint_state is None or
                self.physical_joint not in self.joint_state.name):
            return math.nan
        index = self.joint_state.name.index(self.physical_joint)
        if index >= len(self.joint_state.position):
            return math.nan
        return self.joint_state.position[index]

    def wait_for_live_state(self, timeout: float):
        """Wait for valid semantic and physical position observations."""
        self.spin_until(
            lambda: math.isfinite(self.task_position()) and
            math.isfinite(self.physical_position()), timeout,
            'live task and physical state were not available')

    def controller_states(self, timeout: float = 0.5):
        """Return the current controller-manager lifecycle states."""
        if not self.controller_list_client.service_is_ready():
            return None
        future = self.controller_list_client.call_async(
            ListControllers.Request())
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            future.cancel()
            return None
        return {
            controller.name: controller.state
            for controller in future.result().controller
        }

    def wait_for_controller_states(self, expected, timeout: float):
        """Wait for an exact controller lifecycle boundary with evidence."""
        deadline = time.monotonic() + timeout
        if not self.controller_list_client.wait_for_service(
                timeout_sec=min(timeout, 10.0)):
            raise RuntimeError(
                'controller readiness service is unavailable: '
                f'{_fq(self.namespace, "controller_manager/list_controllers")}')
        observed = None
        while time.monotonic() < deadline:
            observed = self.controller_states(
                timeout=max(0.01, min(0.5, deadline - time.monotonic())))
            if observed is not None and all(
                    observed.get(name) == state
                    for name, state in expected.items()):
                return observed
            rclpy.spin_once(self.node, timeout_sec=0.05)
        expected_text = ', '.join(
            f'{name}={state}' for name, state in expected.items())
        observed_text = ', '.join(
            f'{name}={state}' for name, state in sorted((observed or {}).items()))
        raise RuntimeError(
            f'controller readiness timed out after {timeout:.1f}s; '
            f'expected [{expected_text}]; observed [{observed_text}]')

    def send_goal(self, position: float, timeout: float = 10.0):
        """Send one standard parallel-gripper action goal."""
        observation = {
            'target_task_aperture_m': position,
            'requested_effort': 10.0,
            'accepted': False,
        }
        self.action_observations.append(observation)
        if not self.action.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('ParallelGripperCommand action is unavailable')
        goal = ParallelGripperCommand.Goal()
        goal.command.name = ['grip_stroke']
        goal.command.position = [position]
        goal.command.effort = [10.0]
        future = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError('action goal request timed out')
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('action goal was rejected')
        observation['accepted'] = True
        self._action_observations_by_handle[id(handle)] = observation
        return handle

    def wait_goal(self, handle, timeout: float = 10.0):
        """Wait for and return a wrapped action result."""
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError('action result timed out')
        wrapped = future.result()
        observation = self._action_observations_by_handle.pop(
            id(handle), None)
        if observation is not None:
            observation.update({
                'status': wrapped.status,
                'reached_goal': wrapped.result.reached_goal,
                'stalled': wrapped.result.stalled,
                'result_position': list(wrapped.result.state.position),
                'result_effort': list(wrapped.result.state.effort),
            })
        return wrapped

    def move_and_check(self, target: float, timeout: float = 10.0):
        """Move conventionally and verify task-to-physical mapping."""
        wrapped = self.wait_goal(self.send_goal(target, timeout), timeout)
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'action finished with status {wrapped.status}')
        if not wrapped.result.reached_goal or wrapped.result.stalled:
            raise RuntimeError(
                'action did not report an unobstructed reached goal')
        ratio = ((target - self.task_min_m) /
                 (self.task_max_m - self.task_min_m))
        if self.rg_model:
            expected_physical = math.asin(
                ratio * math.sin(self.physical_max_m))
            physical_tolerance = 0.002
        else:
            expected_physical = (
                self.physical_min_m +
                ratio * (self.physical_max_m - self.physical_min_m))
            physical_tolerance = 0.0005
        self.spin_until(
            lambda: math.isfinite(self.task_position()) and
            math.isfinite(self.physical_position()) and
            abs(self.task_position() - target) <= 0.001 and
            abs(self.physical_position() - expected_physical) <=
            physical_tolerance,
            5.0, f'state did not converge to task target {target}')
        task = self.task_position()
        physical = self.physical_position()
        if abs(task - target) > 0.001:
            raise RuntimeError(f'task target {target} measured as {task}')
        if abs(physical - expected_physical) > physical_tolerance:
            raise RuntimeError(
                f'physical target {expected_physical} measured as {physical}')
        return {
            'target_task_aperture_m': target,
            'measured_task_aperture_m': task,
            'measured_physical_position': physical,
        }

    def cancel_and_check(self):
        """Cancel a conventional goal and require a stable position hold."""
        handle = self.send_goal(self.task_max_m)
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, cancel, timeout_sec=3.0)
        if not cancel.done() or len(cancel.result().goals_canceling) != 1:
            raise RuntimeError('action cancellation was not accepted')
        wrapped = self.wait_goal(handle)
        if wrapped.status != GoalStatus.STATUS_CANCELED:
            raise RuntimeError(
                f'canceled action finished with status {wrapped.status}')
        self.spin_for(0.2)
        before = self.task_position()
        self.spin_for(0.35)
        after = self.task_position()
        if not math.isfinite(before) or abs(after - before) > 0.001:
            raise RuntimeError('canceled action did not hold position')
        return {'held_task_m': after, 'drift_m': abs(after - before)}

    def switch_to_realtime(self):
        """Switch resource ownership from conventional to realtime control."""
        self.wait_for_controller_states({
            'gripper_controller': 'active',
            'realtime_controller': 'inactive',
        }, timeout=30.0)
        if not self.switch_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('controller switch service is unavailable')
        request = SwitchController.Request()
        request.activate_controllers = ['realtime_controller']
        request.deactivate_controllers = ['gripper_controller']
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 5
        future = self.switch_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=8.0)
        if (not future.done() or future.result() is None or
                not future.result().ok):
            message = (
                future.result().message
                if future.done() and future.result() else '')
            states = self.controller_states() or {}
            raise RuntimeError(
                f'controller switch failed: {message}; states={states}')
        self.wait_for_controller_states({
            'gripper_controller': 'inactive',
            'realtime_controller': 'active',
        }, timeout=30.0)
        state_count_before_activation = self.realtime_state_rx_count
        clock_count_before_activation = self.clock_rx_count
        self.spin_until(
            lambda: (
                self.realtime_state is not None and
                self.realtime_state_rx_count > state_count_before_activation and
                self.clock_rx_count >= clock_count_before_activation + 2 and
                self.realtime_publisher.get_subscription_count() > 0),
            5.0,
            'realtime controller did not reach a discovered post-activation '
            'clock boundary')
        self.realtime_activation_observation = {
            'realtime_state_rx_count_before': state_count_before_activation,
            'realtime_state_rx_count_after': self.realtime_state_rx_count,
            'clock_rx_count_before': clock_count_before_activation,
            'clock_rx_count_after': self.clock_rx_count,
            'command_subscription_count': (
                self.realtime_publisher.get_subscription_count()),
            'runner_clock_s': self.node.get_clock().now().nanoseconds * 1e-9,
            'controller_parameters': self.realtime_parameter_snapshot(),
        }
        return {'realtime_activation': self.realtime_activation_observation}

    def realtime_parameter_snapshot(self):
        """Capture the small parameter set relevant to command admission."""
        names = ('joint', 'mechanism_joint', 'coordinate_profile',
                 'command_timeout_ms')
        result = {
            'service': _fq(self.namespace,
                           'realtime_controller/get_parameters'),
            'available': self.realtime_parameter_client.wait_for_service(
                timeout_sec=2.0),
        }
        if not result['available']:
            return result
        request = GetParameters.Request()
        request.names = list(names)
        future = self.realtime_parameter_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            future.cancel()
            result['response'] = 'unavailable'
            return result
        result['values'] = {
            name: parameter_value_to_python(value)
            for name, value in zip(names, future.result().values)
        }
        return result

    def publish_realtime(self, mode: int, duration: float, *,
                         position: float = 0.0, velocity: float = 0.0):
        """Refresh one realtime command for a monotonic duration."""
        if self.realtime_publisher.get_subscription_count() <= 0:
            raise RuntimeError(
                'realtime command publisher has no discovered controller '
                'subscription')
        published = 0
        first_stamp_s = None
        last_stamp_s = None
        active_observed = False
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            command = RealtimeCommand()
            command.header.stamp = self.node.get_clock().now().to_msg()
            stamp_s = (command.header.stamp.sec +
                       command.header.stamp.nanosec * 1e-9)
            if first_stamp_s is None:
                first_stamp_s = stamp_s
            last_stamp_s = stamp_s
            command.mode = mode
            command.task_position = position
            if self.rg_model:
                command.mechanism_angular_velocity = velocity
            else:
                command.task_velocity = velocity
            self.realtime_publisher.publish(command)
            published += 1
            rclpy.spin_once(self.node, timeout_sec=0.02)
            active_observed = active_observed or (
                self.realtime_state is not None and
                self.realtime_state.realtime_active)
        self.last_realtime_publication = {
            'mode': mode,
            'published_command_count': published,
            'first_source_stamp_s': first_stamp_s,
            'last_source_stamp_s': last_stamp_s,
            'command_subscription_count': (
                self.realtime_publisher.get_subscription_count()),
            'active_observed': active_observed,
        }
        return self.last_realtime_publication

    def realtime_diagnostic_snapshot(self):
        """Capture bounded sequence and clock evidence for one RT case."""
        state = self.realtime_state
        state_stamp_s = None
        if state is not None:
            state_stamp_s = (state.header.stamp.sec +
                             state.header.stamp.nanosec * 1e-9)
        return {
            'runner_clock_s': self.node.get_clock().now().nanoseconds * 1e-9,
            'steady_time_s': time.monotonic(),
            'last_received_clock_s': (
                self.clock_samples[-1] if self.clock_samples else None),
            'realtime_state_stamp_s': state_stamp_s,
            'realtime_state_rx_count': self.realtime_state_rx_count,
            'requested_command_sequence': (
                state.requested_command_sequence if state else None),
            'applied_command_sequence': (
                state.applied_command_sequence if state else None),
            'watchdog_stops': state.watchdog_stops if state else None,
            'realtime_active': state.realtime_active if state else None,
            'active_mode': state.active_mode if state else None,
            'task_position_m': self.task_position(),
            'command_subscription_count': (
                self.realtime_publisher.get_subscription_count()),
            'activation_observation': self.realtime_activation_observation,
        }

    def stop_realtime(self):
        """Request an explicit realtime stop and observe it take effect."""
        if not self.stop_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('realtime stop service is unavailable')
        future = self.stop_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        if (not future.done() or future.result() is None or
                not future.result().success):
            raise RuntimeError('realtime stop service failed')
        self.spin_until(
            lambda: self.realtime_state is not None and
            not self.realtime_state.realtime_active,
            2.0, 'realtime session did not stop')


class KinematicQualificationWorkflow:
    """Process orchestration and qualification cases."""

    def __init__(self, args):
        """Capture paths and initialize the machine-readable report."""
        self.args = args
        self.contract = load_contract(args.model)
        self.task_min_m = float(
            self.contract['ros']['task_minimum_m'])
        self.task_max_m = float(
            self.contract['ros']['task_maximum_m'])
        self.physical_min_m, self.physical_max_m = joint_limits(self.contract)
        self.physical_joint = driven_joint(self.contract)
        physical_dimension, physical_unit = coordinate_metadata(self.contract)
        self.output = args.output_dir.resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.bridge_process = None
        self.bringup_process = None
        self.bridge_starts = 0
        self.bridge_log = None
        self.bringup_log = None
        self.node = None
        self.report = {
            'schema_version': 1,
            'model': args.model,
            'fidelity': 'kinematic',
            'ros_distro': os.environ.get('ROS_DISTRO'),
            'rmw_implementation': rclpy.get_rmw_implementation_identifier(),
            'runner_sha256': _sha256(Path(__file__).resolve()),
            'startup_timeout_s': args.startup_timeout,
            'controller_readiness_timeout_s': min(args.startup_timeout, 60.0),
            'isaac_sim_version': None,
            'physics_backend': 'physx',
            'physical_coordinate': {
                'joint': self.physical_joint,
                'dimension': physical_dimension,
                'unit': physical_unit,
            },
            'namespace': args.namespace.strip('/'),
            'started_at': datetime.now(timezone.utc).isoformat(),
            'status': 'failed',
            'tests': [],
        }

    def case(self, name, function):
        """Execute one evidence-producing case without losing later cleanup."""
        started = time.monotonic()
        realtime_case = name.startswith('realtime_')
        action_case = name.startswith('conventional_')
        before = (self.node.realtime_diagnostic_snapshot()
                  if realtime_case and self.node is not None else None)
        action_start = (len(self.node.action_observations)
                        if action_case and self.node is not None else 0)
        try:
            details = function() or {}
            result = {'name': name, 'status': 'passed', **details}
        except Exception as error:
            result = {
                'name': name,
                'status': 'failed',
                'error': f'{type(error).__name__}: {error}',
            }
        if realtime_case and self.node is not None:
            after = self.node.realtime_diagnostic_snapshot()
            requested_advanced = (
                before['requested_command_sequence'] is not None and
                after['requested_command_sequence'] is not None and
                after['requested_command_sequence'] >
                before['requested_command_sequence'])
            if result['status'] == 'passed':
                rejection_reason = 'none_observed'
            elif not requested_advanced:
                rejection_reason = 'no_new_hardware_sequence_observed'
            elif (after['applied_command_sequence'] is not None and
                  after['requested_command_sequence'] is not None and
                  after['applied_command_sequence'] <
                  after['requested_command_sequence']):
                rejection_reason = 'requested_sequence_not_applied'
            elif not after['realtime_active']:
                rejection_reason = 'latest_applied_sequence_not_motion'
            else:
                rejection_reason = 'motion_applied_but_target_not_reached'
            result['realtime_diagnostics'] = {
                'before': before,
                'publication': self.node.last_realtime_publication,
                'after': after,
                'rejection_reason': rejection_reason,
            }
        if action_case and self.node is not None:
            result['action_observations'] = (
                self.node.action_observations[action_start:])
        result['elapsed_s'] = time.monotonic() - started
        self.report['tests'].append(result)
        return result['status'] == 'passed'

    def _start_bridge(self):
        ready = self.output / 'bridge_ready.json'
        ready.unlink(missing_ok=True)
        if self.bridge_log is not None:
            self.bridge_log.close()
        log_mode = 'w' if self.bridge_starts == 0 else 'a'
        self.bridge_log = (self.output / 'bridge.log').open(
            log_mode, encoding='utf-8')
        self.bridge_starts += 1
        command = [
            str(self.args.isaac_python), str(self.args.bridge),
            '--model', self.args.model,
            '--asset', str(self.args.asset),
            '--namespace', self.args.namespace,
            '--ready-output', str(ready),
        ]
        self.bridge_process = subprocess.Popen(
            command, stdout=self.bridge_log, stderr=subprocess.STDOUT,
            start_new_session=True)
        deadline = time.monotonic() + self.args.startup_timeout
        while time.monotonic() < deadline:
            if self.bridge_process.poll() is not None:
                raise RuntimeError(
                    'Isaac bridge exited with '
                    f'{self.bridge_process.returncode}')
            if ready.is_file():
                payload = json.loads(ready.read_text(encoding='utf-8'))
                if payload.get('status') == 'ready':
                    version = payload.get('isaac_sim_version')
                    if not isinstance(version, str) or not version:
                        raise RuntimeError(
                            'Isaac bridge did not report its runtime version')
                    expected = self.report['isaac_sim_version']
                    if expected is not None and version != expected:
                        raise RuntimeError(
                            f'Isaac runtime changed during qualification: '
                            f'{expected} -> {version}')
                    return payload
                if payload.get('status') == 'failed':
                    raise RuntimeError(payload.get('error', 'bridge failed'))
            time.sleep(0.1)
        raise RuntimeError('Isaac bridge startup timed out')

    def _start_bringup(self):
        self.bringup_log = (self.output / 'bringup.log').open(
            'w', encoding='utf-8')
        command = [
            self.args.ros2, 'launch', 'onrobot_gripper_bringup',
            'gripper.launch.py', f'model:={self.args.model}',
            'backend:=isaac',
            'start_realtime_controller:=false',
            # Isaac may advance simulated time rapidly while an articulation
            # drive is still converging. This is qualification-only; normal
            # hardware bringup retains the two-second default.
            'gripper_stall_timeout_s:=8.0',
            f'namespace:={self.args.namespace}',
        ]
        self.bringup_process = subprocess.Popen(
            command, stdout=self.bringup_log, stderr=subprocess.STDOUT,
            start_new_session=True)

    def _clock_and_namespace(self):
        states = self.node.wait_for_controller_states(
            _required_controller_states(),
            self.report['controller_readiness_timeout_s'])
        self.report['initial_controller_states'] = states
        self.node.spin_until(
            lambda: len(self.node.clock_samples) >= 2 and
            self.node.clock_samples[-1] > self.node.clock_samples[0],
            10.0, '/clock did not advance')
        self.node.wait_for_live_state(30.0)
        return {
            'clock_start_s': self.node.clock_samples[0],
            'clock_end_s': self.node.clock_samples[-1],
            'task_state_topic': _fq(self.args.namespace, 'joint_states'),
            'controller_plugin_libraries': _mapped_library_identity(
                self.bringup_process, 'libonrobot_gripper_controller.so'),
            'hardware_plugin_libraries': _mapped_library_identity(
                self.bringup_process, 'libonrobot_gripper_hardware.so'),
            'package_prefixes': _package_prefixes((
                'onrobot_gripper_isaac',
                'onrobot_gripper_bringup',
                'onrobot_gripper_controllers',
                'onrobot_gripper_hardware',
            )),
        }

    def _conventional(self):
        return {'motions': [
            self.node.move_and_check(self.task_max_m),
            self.node.move_and_check(
                (self.task_min_m + self.task_max_m) / 2.0),
            self.node.move_and_check(self.task_min_m),
        ]}

    def _dropout_recovery(self):
        # A controller may disappear after the initial startup gate.  Do not
        # deliberately interrupt the bridge while the ROS graph is partial:
        # the outage test must begin from a complete, known ownership state.
        states = self.node.wait_for_controller_states(
            _required_controller_states(), timeout=5.0)
        self.report['controller_states_before_dropout'] = states
        initial_reconnects = self.node.gripper_state.reconnects
        _terminate(self.bridge_process)
        self.node.spin_for(0.3)
        state_count_during_outage = self.node.gripper_state_rx_count
        clock_count_during_outage = self.node.clock_rx_count
        outage_started = time.monotonic()
        self.node.spin_for(0.7)
        outage_duration = time.monotonic() - outage_started
        if self.node.gripper_state_rx_count != state_count_during_outage:
            raise RuntimeError(
                'gripper state continued after the bridge exited')
        if self.node.clock_rx_count != clock_count_during_outage:
            raise RuntimeError('/clock continued after the bridge exited')
        bridge_status = self._start_bridge()
        self.node.wait_for_controller_states({
            **_required_controller_states(),
        }, timeout=15.0)
        self.node.spin_until(
            lambda: self.node.gripper_state is not None and
            self.node.gripper_state.task_aperture_valid and
            self.node.gripper_state_rx_count > state_count_during_outage and
            self.node.clock_rx_count > clock_count_during_outage,
            20.0, 'Isaac state and clock did not recover after bridge restart')
        return {
            'observed_state_stream_outage_s': outage_duration,
            'clock_stalled_with_bridge': True,
            'reconnects_before': initial_reconnects,
            'reconnects_after': self.node.gripper_state.reconnects,
            'bridge_articulation_path': bridge_status['articulation_path'],
        }

    def _realtime_position(self):
        self.node.publish_realtime(
            RealtimeCommand.POSITION, 0.6,
            position=(self.task_min_m + self.task_max_m) / 2.0)
        self.node.spin_until(
            lambda: abs(
                self.node.task_position() -
                (self.task_min_m + self.task_max_m) / 2.0) < 0.001,
            5.0, 'realtime position did not reach midpoint')
        return {'task_m': self.node.task_position()}

    def _realtime_endpoint(self):
        self.node.task_samples.clear()
        velocity = 0.7 if self.node.rg_model else self.task_max_m
        duration = 2.4 if self.node.rg_model else 1.3
        self.node.publish_realtime(
            RealtimeCommand.VELOCITY, duration, velocity=velocity)
        end = self.node.task_position()
        recent = [value for stamp, value in self.node.task_samples
                  if stamp >= time.monotonic() - 0.5]
        if end < self.task_max_m - 0.001:
            raise RuntimeError(f'open endpoint was not reached: {end}')
        if recent and min(recent) < self.task_max_m - 0.0015:
            raise RuntimeError('held opening command reversed at the endpoint')
        self.node.stop_realtime()
        return {'endpoint_m': end, 'minimum_recent_m': min(recent or [end])}

    def _realtime_watchdog(self):
        self.node.task_samples.clear()
        self.node.publish_realtime(
            RealtimeCommand.POSITION, 0.4,
            position=(self.task_min_m + self.task_max_m) / 2.0)
        motion_start = self.node.task_position()
        self.node.task_samples.clear()
        self.node.publish_realtime(
            RealtimeCommand.VELOCITY, 0.25,
            velocity=-0.4 if self.node.rg_model else -0.04)
        motion_samples = [value for _, value in self.node.task_samples]
        excursion = max(
            (abs(value - motion_start) for value in motion_samples),
            default=0.0)
        if (not self.node.last_realtime_publication['active_observed'] or
                excursion <= 0.001):
            raise RuntimeError(
                'watchdog precondition failed: realtime motion was not '
                'observed before publisher loss')
        self.node.spin_until(
            lambda: self.node.realtime_state is not None and
            not self.node.realtime_state.realtime_active,
            2.0, 'stale realtime command did not stop')
        self.node.spin_for(0.25)
        before = self.node.task_position()
        self.node.spin_for(0.35)
        after = self.node.task_position()
        if abs(after - before) > 0.001:
            raise RuntimeError('position drifted after realtime watchdog stop')
        return {
            'held_task_m': after,
            'drift_m': abs(after - before),
            'prior_motion_excursion_m': excursion,
            'prior_realtime_active_observed': True,
        }

    def _force_unavailable(self):
        if self.node.gripper_state.force_valid:
            raise RuntimeError(
                'unqualified simulated force was reported valid')
        if (self.node.realtime_state is not None and
                self.node.realtime_state.force_valid):
            raise RuntimeError('realtime state reported simulated force valid')
        return {'force_valid': False}

    def run(self):
        """Start both stacks and execute the kinematic case sequence."""
        if not self.args.isaac_python.is_file():
            raise RuntimeError(
                f'Isaac python does not exist: {self.args.isaac_python}')
        if not self.args.asset.is_file() or not self.args.bridge.is_file():
            raise RuntimeError('asset or bridge script does not exist')
        self.report['asset_files_sha256'] = _asset_manifest(self.args.asset)
        self.report['bridge_sha256'] = _sha256(self.args.bridge)
        bridge_status = self._start_bridge()
        self.report['bridge'] = bridge_status
        self.report['isaac_sim_version'] = bridge_status.get(
            'isaac_sim_version')
        self._start_bringup()
        rclpy.init()
        self.node = QualificationNode(
            self.args.namespace, self.args.model,
            (self.task_min_m, self.task_max_m),
            (self.physical_min_m, self.physical_max_m),
            self.physical_joint)

        if not self.case(
                'clock_namespace_and_live_state', self._clock_and_namespace):
            return
        if not self.case(
                'conventional_open_midpoint_close', self._conventional):
            return
        if not self.case(
                'conventional_cancel_holds', self.node.cancel_and_check):
            return
        if not self.case(
                'state_stream_outage_and_bridge_recovery',
                self._dropout_recovery):
            return
        if self.case(
                'switch_to_realtime_controller',
                self.node.switch_to_realtime):
            self.case('realtime_position', self._realtime_position)
            self.case(
                'realtime_velocity_endpoint_hold', self._realtime_endpoint)
            self.case(
                'realtime_stale_command_watchdog', self._realtime_watchdog)
            self.case('force_remains_unavailable', self._force_unavailable)

    def finish(self):
        """Stop only spawned processes and finish the report."""
        if self.node is not None:
            self.node.destroy()
        if rclpy.ok():
            rclpy.shutdown()
        _terminate(self.bringup_process)
        _terminate(self.bridge_process)
        if self.bringup_log is not None:
            self.bringup_log.close()
        if self.bridge_log is not None:
            self.bridge_log.close()
        self.report['finished_at'] = datetime.now(timezone.utc).isoformat()
        self.report['status'] = (
            'passed' if self.report['tests'] and
            all(item['status'] == 'passed' for item in self.report['tests'])
            else 'failed')
        report_path = self.output / (
            f'{self.args.model}_kinematic_qualification.json')
        report_path.write_text(
            json.dumps(self.report, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')
        print(json.dumps(self.report, indent=2, sort_keys=True))
        return 0 if self.report['status'] == 'passed' else 1


def main() -> int:
    """Parse paths, execute the qualification, and preserve all evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=SUPPORTED_MODELS, required=True)
    parser.add_argument('--isaac-python', type=Path, required=True)
    parser.add_argument('--asset', type=Path)
    parser.add_argument('--bridge', type=Path, default=_default_bridge())
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--namespace', default='onrobot_kinematic')
    parser.add_argument('--ros2', default='ros2')
    parser.add_argument('--startup-timeout', type=float, default=180.0)
    args = parser.parse_args()
    if args.asset is None:
        args.asset = default_asset(args.model)

    workflow = KinematicQualificationWorkflow(args)
    try:
        workflow.run()
    except Exception as error:
        workflow.report['error'] = f'{type(error).__name__}: {error}'
    finally:
        exit_code = workflow.finish()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())

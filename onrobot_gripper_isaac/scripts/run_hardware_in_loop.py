#!/usr/bin/python3
"""Qualify gripper ROS hardware and optionally mirror supported USD assets."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
import traceback

from action_msgs.msg import GoalStatus

from control_msgs.action import ParallelGripperCommand
from control_msgs.msg import Float64Values
from control_msgs.msg import Keys

from isaac_model_contract import articulation_path
from isaac_model_contract import coordinate_metadata
from isaac_model_contract import default_asset
from isaac_model_contract import driven_joint
from isaac_model_contract import joint_limits
from isaac_model_contract import load_contract
from isaac_model_contract import SUPPORTED_MODELS

from isaac_runtime_compat import is_stage_loading
from isaac_runtime_compat import play
from isaac_runtime_compat import setup_simulation
from isaac_runtime_compat import stop

from onrobot_gripper_msgs.msg import GripperState
from onrobot_gripper_msgs.msg import RealtimeCommand
from onrobot_gripper_msgs.msg import RealtimeState

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from sensor_msgs.msg import JointState

from std_srvs.srv import Trigger


SUPPORTED_HIL_MODELS = ('2fg7', '2fg14', 'rg2', 'rg6')
ISAAC_HIL_MODELS = SUPPORTED_MODELS
DEFAULT_PHYSICAL_JOINTS = {
    '2fg7': 'finger_stroke',
    '2fg14': 'finger_stroke',
    'rg2': 'finger_joint',
    'rg6': 'finger_joint',
}
# Every supported model has bounded physical position, direction/reversal,
# watchdog, Stop, and restoration evidence for its current host protocol.
# This gate permits qualification motion; passing HIL evidence remains a
# separate result that must not be inferred from membership here.
REALTIME_POSITION_MOTION_QUALIFIED_MODELS = SUPPORTED_HIL_MODELS
ROS_CALLBACK_DRAIN_LIMIT = 16
REALTIME_COMMAND_REFRESH_PERIOD_S = 0.02
REALTIME_POSITION_MAXIMUM_VELOCITY_M_S = 0.015
REALTIME_TARGET_HOLD_S = 0.12
REALTIME_QUALIFICATION_MAX_TRAVEL_M = 0.004
REALTIME_QUALIFICATION_START_MARGIN_M = 0.002
REALTIME_OPPOSED_MOTION_TOLERANCE_M = 0.0005
REALTIME_TASK_AGREEMENT_TOLERANCE_M = 0.0005
REALTIME_MAPPING_TOLERANCE_M = 0.002
# The ROS ``finger_joint`` is the CAD linkage angle, while the live task
# aperture includes the product's fingertip/zero compensation.  Keep the
# same geometry used by RgCadKinematics in the qualification runner so the
# physical-travel check validates the actual published coordinate contract.
# Values are (link length [m], lateral offset [m], signed gap offset [m]).
RG_CAD_GEOMETRY = {
    'rg2': (0.055, 0.0025, -0.0039),
    'rg6': (0.080, 0.0042, -0.0016),
}
RG6_MAPPING_TOLERANCE_RAD = 0.005
TWO_FG_REALTIME_POSITION_QUALIFICATION_REASON = (
    'this 2FG model has not yet qualified command-6 position and feedback as '
    'external task aperture against current integration guide, so physical '
    'position motion remains disabled until the documented semantics are '
    'proven')
RG_REALTIME_POSITION_QUALIFICATION_REASON = (
    'firmware identity has not been matched to a release-qualified RG '
    'realtime revision, so physical position motion remains disabled until '
    'the installed firmware is within an approved range')


def _realtime_position_qualification_reason(model: str):
    """Return the evidence gate that applies to the selected model family."""
    if model in REALTIME_POSITION_MOTION_QUALIFIED_MODELS:
        return None
    if model.startswith('2fg'):
        return TWO_FG_REALTIME_POSITION_QUALIFICATION_REASON
    return RG_REALTIME_POSITION_QUALIFICATION_REASON


def _runtime_realtime_position_reason(model: str, state):
    """Return a model/protocol or live firmware gate for physical motion."""
    reason = _realtime_position_qualification_reason(model)
    if reason is not None:
        return reason
    if (model.startswith('rg') and
            state.firmware_qualification != GripperState.FIRMWARE_QUALIFIED):
        return RG_REALTIME_POSITION_QUALIFICATION_REASON
    return None


def _fq(namespace: str, name: str) -> str:
    namespace = namespace.strip('/')
    return f'/{namespace}/{name}' if namespace else f'/{name}'


def _value(array, row: int, column: int) -> float:
    values = array.numpy() if hasattr(array, 'numpy') else array
    return float(values[row, column])


def _shadow_dof_positions(contract: dict, dof_names: list[str],
                          reference_positions: list[float],
                          driven_index: int,
                          position: float) -> dict[int, float]:
    """Map one measured leader position to a constraint-consistent pose."""
    mirrored = {driven_index: position}
    follower_name = contract['usd'].get('follower_joint')
    if follower_name in dof_names:
        mirrored[dof_names.index(follower_name)] = position

    leader_delta = position - reference_positions[driven_index]
    for mimic in contract['usd'].get('mimic_joints', []):
        name = mimic['name']
        if name not in dof_names:
            raise RuntimeError(
                f'contract mimic DOF missing from runtime articulation: '
                f'{name}')
        index = dof_names.index(name)
        # PhysX mimic gearing appears in the constraint equation; the
        # follower displacement therefore has the opposite sign.
        mirrored[index] = (
            reference_positions[index] -
            float(mimic['gearing']) * leader_delta)
    return mirrored


def _mirror_measured_position(articulation, driven_index: int,
                              positions: dict[int, float]) -> None:
    """Mirror one complete, constraint-consistent measured snapshot."""
    indices = list(positions)
    values = [positions[index] for index in indices]
    articulation.set_dof_position_targets(
        positions[driven_index], dof_indices=[driven_index])
    velocities = [0.0] * len(indices)
    articulation.set_dof_positions(values, dof_indices=indices)
    articulation.set_dof_velocities(velocities, dof_indices=indices)


def _drain_ros_callbacks(node, limit: int = ROS_CALLBACK_DRAIN_LIMIT) -> None:
    """Process queued ROS work after a potentially slow Isaac frame."""
    for _ in range(limit):
        rclpy.spin_once(node, timeout_sec=0.0)


class RealtimePositionCommandStream:
    """Refresh one bounded position command independently of rendering."""

    def __init__(self, publisher, stamp, position: float,
                 period: float = REALTIME_COMMAND_REFRESH_PERIOD_S):
        """Store an immutable target and the ROS publishing dependencies."""
        self._publisher = publisher
        self._stamp = stamp
        self._position = position
        self._period = period
        self._stop = threading.Event()
        self._thread = None
        self._error = None
        self.publish_count = 0

    def start(self) -> None:
        """Start the refresh worker after all inputs are immutable."""
        if self._thread is not None:
            raise RuntimeError('realtime command stream already started')
        self._thread = threading.Thread(
            target=self._run,
            name='onrobot-realtime-position-refresh',
            daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop refreshing before the caller requests explicit device Stop."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, self._period * 4.0))
            if self._thread.is_alive():
                raise RuntimeError('realtime command stream did not stop')
        self.raise_if_failed()

    def raise_if_failed(self) -> None:
        """Surface publisher failures on the qualification thread."""
        if self._error is not None:
            raise RuntimeError(
                'realtime command stream failed: '
                f'{type(self._error).__name__}: {self._error}')

    def _run(self) -> None:
        next_publish = time.monotonic()
        try:
            while not self._stop.is_set():
                command = RealtimeCommand()
                command.header.stamp = self._stamp()
                command.mode = RealtimeCommand.POSITION
                command.task_position = self._position
                command.task_velocity = REALTIME_POSITION_MAXIMUM_VELOCITY_M_S
                command.force = 0.0
                self._publisher.publish(command)
                self.publish_count += 1
                next_publish += self._period
                delay = next_publish - time.monotonic()
                if delay <= 0.0:
                    next_publish = time.monotonic()
                    continue
                self._stop.wait(delay)
        except Exception as error:  # pragma: no cover - middleware failure
            self._error = error
            self._stop.set()


def _reported_sample_age(state: GripperState) -> float:
    """Convert the device sample age duration to seconds."""
    return (
        float(state.sample_age.sec) +
        float(state.sample_age.nanosec) * 1e-9)


def _hardware_cycle_duration(state: GripperState) -> float:
    """Return the latest hardware worker cycle duration in seconds."""
    return float(state.last_cycle_duration)


def _sha256(path: Path) -> str:
    """Return a stable digest for a qualification input."""
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


def _acceptance_limits(args) -> dict:
    """Record the exact thresholds applied to a HIL qualification run."""
    limits = {
        'startup_timeout_s': args.startup_timeout,
        'state_freshness_timeout_s': args.state_timeout,
        'monitoring_duration_s': args.duration,
        'adverse_counter_change_allowed': False,
    }
    if not (args.ros_preflight_only or args.verify_conventional_completion or
            args.verify_conventional_cancel or
            args.verify_conventional_preemption or
            args.verify_realtime_position):
        limits.update({
            'physics_step_s': args.physics_dt,
            'physical_coordinate_shadow_tolerance': args.shadow_tolerance,
        })
    if (args.enable_hardware_motion or args.verify_conventional_completion or
            args.verify_conventional_cancel or
            args.verify_conventional_preemption or
            args.verify_realtime_position):
        limits.update({
            'maximum_external_aperture_excursion_m': args.travel,
            'motion_timeout_s': args.motion_timeout,
        })
        if args.hardware_control == 'conventional':
            limits['maximum_conventional_effort_n'] = args.effort
    if args.verify_conventional_cancel:
        limits['minimum_motion_before_cancel_m'] = args.cancel_after_motion
    if args.verify_conventional_preemption:
        limits['minimum_motion_before_preemption_m'] = (
            args.cancel_after_motion)
    if args.verify_realtime_stop:
        limits['realtime_stop_timeout_s'] = args.stop_timeout
    if args.verify_realtime_position:
        limits.update({
            'realtime_stop_timeout_s': args.stop_timeout,
            'realtime_command_refresh_period_s': (
                REALTIME_COMMAND_REFRESH_PERIOD_S),
            'maximum_opposed_motion_m': (
                REALTIME_OPPOSED_MOTION_TOLERANCE_M),
            'task_feedback_agreement_tolerance_m': (
                REALTIME_TASK_AGREEMENT_TOLERANCE_M),
            'task_to_physical_mapping_tolerance': (
                0.02 if args.model == 'rg2'
                else REALTIME_MAPPING_TOLERANCE_M),
            'maximum_start_distance_from_open_endpoint_m': (
                args.travel + REALTIME_QUALIFICATION_START_MARGIN_M),
        })
    return limits


class HardwareObservation:
    """Receive the real gripper state and own the standard action client."""

    def __init__(self, namespace: str, task_joint: str,
                 physical_joint: str, expected_model: str = '2fg7'):
        """Create state subscriptions and the conventional action client."""
        suffix = namespace.strip('/').replace('/', '_') or 'root'
        self.node = rclpy.create_node(f'onrobot_gripper_hil_{suffix}')
        self.task_joint = task_joint
        self.physical_joint = physical_joint
        self.expected_model = expected_model
        self.joint_position = math.nan
        self.joint_velocity = 0.0
        self.joint_received_at = 0.0
        self.joint_samples = 0
        self.state = None
        self.state_received_at = 0.0
        self.timing_metrics_enabled = False
        self.timing_samples = 0
        self.maximum_reported_sample_age = 0.0
        self.maximum_hardware_cycle_duration = 0.0
        self.maximum_typed_state_interval = 0.0
        self.reported_sample_ages = []
        self.hardware_cycle_durations = []
        self.typed_state_intervals = []
        self.previous_timing_state_received_at = 0.0
        self.model_mismatch = None
        self.active_goal = None
        self.realtime_state = None
        self.realtime_state_received_at = 0.0
        self.limit_keys = []
        self.limit_values = []
        self.task_limits = None
        limit_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(
            JointState, _fq(namespace, 'joint_states'),
            self._receive_joint_state, qos_profile_sensor_data)
        self.node.create_subscription(
            GripperState,
            _fq(namespace, 'gripper_state_broadcaster/state'),
            self._receive_gripper_state, qos_profile_sensor_data)
        self.node.create_subscription(
            Keys, _fq(namespace, 'parallel_gripper_limit_broadcaster/names'),
            self._receive_limit_keys, limit_qos)
        self.node.create_subscription(
            Float64Values,
            _fq(namespace, 'parallel_gripper_limit_broadcaster/values'),
            self._receive_limit_values, limit_qos)
        self.node.create_subscription(
            RealtimeState, _fq(namespace, 'realtime_controller/state'),
            self._receive_realtime_state, qos_profile_sensor_data)
        self.action = ActionClient(
            self.node, ParallelGripperCommand,
            _fq(namespace, 'gripper_controller/gripper_cmd'))
        self.realtime_publisher = self.node.create_publisher(
            RealtimeCommand,
            _fq(namespace, 'realtime_controller/command'),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.realtime_stop = self.node.create_client(
            Trigger, _fq(namespace, 'realtime_controller/stop'))

    def _receive_joint_state(self, message):
        try:
            index = message.name.index(self.physical_joint)
        except ValueError:
            return
        if index >= len(message.position):
            return
        position = float(message.position[index])
        if not math.isfinite(position):
            return
        velocity = (
            float(message.velocity[index])
            if index < len(message.velocity) and
            math.isfinite(message.velocity[index]) else 0.0)
        self.joint_position = position
        self.joint_velocity = velocity
        self.joint_received_at = time.monotonic()
        self.joint_samples += 1

    def _receive_gripper_state(self, message):
        if message.model and message.model != self.expected_model:
            self.model_mismatch = message.model
            return
        received_at = time.monotonic()
        self.state = message
        self.state_received_at = received_at
        if self.timing_metrics_enabled:
            if self.previous_timing_state_received_at > 0.0:
                interval = (
                    received_at - self.previous_timing_state_received_at)
                self.maximum_typed_state_interval = max(
                    self.maximum_typed_state_interval, interval)
                self.typed_state_intervals.append(interval)
            self.previous_timing_state_received_at = received_at
            sample_age = _reported_sample_age(message)
            cycle_duration = _hardware_cycle_duration(message)
            if math.isfinite(sample_age) and sample_age >= 0.0:
                self.maximum_reported_sample_age = max(
                    self.maximum_reported_sample_age, sample_age)
                self.reported_sample_ages.append(sample_age)
            if math.isfinite(cycle_duration) and cycle_duration >= 0.0:
                self.maximum_hardware_cycle_duration = max(
                    self.maximum_hardware_cycle_duration, cycle_duration)
                self.hardware_cycle_durations.append(cycle_duration)
            self.timing_samples += 1

    def reset_timing_metrics(self):
        """Start a fresh post-readiness hardware timing observation window."""
        self.timing_metrics_enabled = True
        self.timing_samples = 0
        self.maximum_reported_sample_age = 0.0
        self.maximum_hardware_cycle_duration = 0.0
        self.maximum_typed_state_interval = 0.0
        self.reported_sample_ages = []
        self.hardware_cycle_durations = []
        self.typed_state_intervals = []
        self.previous_timing_state_received_at = 0.0

    @staticmethod
    def _percentiles(values: list[float]) -> dict:
        """Return nearest-rank percentiles without a statistics dependency."""
        if not values:
            return {}
        ordered = sorted(values)
        result = {}
        for label, fraction in (
                ('median', 0.50), ('percentile_95', 0.95),
                ('percentile_99', 0.99)):
            index = max(0, math.ceil(fraction * len(ordered)) - 1)
            result[f'{label}_s'] = ordered[index]
        result['maximum_s'] = ordered[-1]
        return result

    def timing_metrics(self) -> dict:
        """Return bounded timing evidence collected after readiness."""
        return {
            'typed_state_samples': self.timing_samples,
            'maximum_reported_sample_age_s': (
                self.maximum_reported_sample_age),
            'maximum_hardware_cycle_duration_s': (
                self.maximum_hardware_cycle_duration),
            'maximum_typed_state_interval_s': (
                self.maximum_typed_state_interval),
            'reported_sample_age': self._percentiles(
                self.reported_sample_ages),
            'hardware_cycle_duration': self._percentiles(
                self.hardware_cycle_durations),
            'typed_state_interval': self._percentiles(
                self.typed_state_intervals),
        }

    def _receive_limit_keys(self, message):
        self.limit_keys = list(message.keys)
        self._update_task_limits()

    def _receive_limit_values(self, message):
        self.limit_values = [float(value) for value in message.values]
        self._update_task_limits()

    def _receive_realtime_state(self, message):
        self.realtime_state = message
        self.realtime_state_received_at = time.monotonic()

    def _update_task_limits(self):
        if len(self.limit_keys) != len(self.limit_values):
            return
        minimum_key = f'{self.task_joint}/minimum_task_aperture'
        maximum_key = f'{self.task_joint}/maximum_task_aperture'
        try:
            minimum = self.limit_values[self.limit_keys.index(minimum_key)]
            maximum = self.limit_values[self.limit_keys.index(maximum_key)]
        except ValueError:
            return
        if (math.isfinite(minimum) and math.isfinite(maximum) and
                minimum >= 0.0 and maximum > minimum):
            self.task_limits = (minimum, maximum)

    def destroy(self):
        """Release ROS resources without touching the hardware session."""
        self.action.destroy()
        self.node.destroy_node()

    def send_goal(self, position: float, effort: float, pump, timeout: float):
        """Send a goal and require fresh typed state at the reached target."""
        state_received_before_goal = self.state_received_at
        goal_handle, result_future = self._start_goal(
            position, effort, pump, timeout)
        deadline = time.monotonic() + timeout
        while not result_future.done() and time.monotonic() < deadline:
            pump()
        if not result_future.done() or result_future.result() is None:
            raise RuntimeError('action result timed out')
        wrapped = result_future.result()
        self.active_goal = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f'action finished with status {wrapped.status}')
        if not wrapped.result.reached_goal or wrapped.result.stalled:
            raise RuntimeError(
                'action did not report an unobstructed reached goal')
        while time.monotonic() < deadline:
            state = self.state
            if (state is not None and
                    self.state_received_at > state_received_before_goal and
                    state.task_aperture_valid and
                    math.isfinite(state.task_aperture) and
                    abs(float(state.task_aperture) - position) <= 0.0015):
                return wrapped.result
            pump()
        raise RuntimeError(
            'action succeeded but fresh typed hardware state did not confirm '
            f'target aperture {position}')

    def _start_goal(self, position: float, effort: float,
                    pump, timeout: float):
        """Start an action and return its handle and result future."""
        deadline = time.monotonic() + timeout
        while (not self.action.server_is_ready() and
               time.monotonic() < deadline):
            pump()
        if not self.action.server_is_ready():
            raise RuntimeError('ParallelGripperCommand action is unavailable')

        goal = ParallelGripperCommand.Goal()
        goal.command.name = [self.task_joint]
        goal.command.position = [position]
        goal.command.effort = [effort]
        future = self.action.send_goal_async(goal)
        while not future.done() and time.monotonic() < deadline:
            pump()
        if not future.done() or future.result() is None:
            raise RuntimeError('action goal request timed out')
        self.active_goal = future.result()
        if not self.active_goal.accepted:
            self.active_goal = None
            raise RuntimeError('action goal was rejected')
        return self.active_goal, self.active_goal.get_result_async()

    def cancel_goal_after_motion(self, position: float, effort: float,
                                 minimum_motion: float, pump,
                                 timeout: float) -> dict:
        """Cancel a goal after fresh measured motion is visible."""
        if self.state is None or not self.state.task_aperture_valid:
            raise RuntimeError('live task aperture is unavailable before goal')
        initial_position = float(self.state.task_aperture)
        initial_sequence = int(self.state.sample_sequence)
        goal_handle, result_future = self._start_goal(
            position, effort, pump, timeout)
        deadline = time.monotonic() + timeout
        measured_before_cancel = initial_position
        maximum_excursion = 0.0
        sequence_before_cancel = initial_sequence
        while time.monotonic() < deadline:
            pump()
            if result_future.done():
                self.active_goal = None
                raise RuntimeError(
                    'conventional goal completed before cancellation evidence')
            state = self.state
            if state is None or not state.task_aperture_valid:
                continue
            measured_before_cancel = float(state.task_aperture)
            maximum_excursion = max(
                maximum_excursion,
                abs(measured_before_cancel - initial_position))
            sequence_before_cancel = int(state.sample_sequence)
            if (sequence_before_cancel > initial_sequence and state.busy and
                    abs(measured_before_cancel - initial_position) >=
                    minimum_motion):
                break
        else:
            self.cancel_active_goal(pump)
            raise RuntimeError(
                'measured motion was not observed before cancel timeout')

        cancel_requested_at = time.monotonic()
        cancel_future = goal_handle.cancel_goal_async()
        while not cancel_future.done() and time.monotonic() < deadline:
            pump()
        if (not cancel_future.done() or cancel_future.result() is None or
                len(cancel_future.result().goals_canceling) != 1):
            self.active_goal = None
            raise RuntimeError('action cancellation was not accepted')
        while not result_future.done() and time.monotonic() < deadline:
            pump()
        if not result_future.done() or result_future.result() is None:
            self.active_goal = None
            raise RuntimeError('cancelled action result timed out')
        wrapped = result_future.result()
        self.active_goal = None
        if wrapped.status != GoalStatus.STATUS_CANCELED:
            raise RuntimeError(
                f'cancelled action finished with status {wrapped.status}')

        stopped_samples = 0
        previous_position = math.nan
        stopped_position = math.nan
        stopped_sequence = sequence_before_cancel
        while time.monotonic() < deadline:
            pump()
            state = self.state
            if (state is None or not state.task_aperture_valid or
                    self.state_received_at <= cancel_requested_at or
                    int(state.sample_sequence) <= stopped_sequence or
                    state.busy):
                continue
            current_position = float(state.task_aperture)
            maximum_excursion = max(
                maximum_excursion,
                abs(current_position - initial_position))
            stopped_sequence = int(state.sample_sequence)
            if (math.isfinite(previous_position) and
                    abs(current_position - previous_position) <= 0.0005):
                stopped_samples += 1
            else:
                stopped_samples = 0
            previous_position = current_position
            stopped_position = current_position
            if stopped_samples >= 2:
                return {
                    'initial_task_aperture_m': initial_position,
                    'target_task_aperture_m': position,
                    'measured_before_cancel_m': measured_before_cancel,
                    'stopped_task_aperture_m': stopped_position,
                    'peak_observed_excursion_m': maximum_excursion,
                    'cancelled_result_observed': True,
                    'newer_idle_state_observed': True,
                }
        raise RuntimeError(
            'hardware did not publish stable idle state after cancellation')

    def cancel_active_goal(self, pump, timeout: float = 3.0):
        """Cancel an action so the controller performs its Stop path."""
        if self.active_goal is None:
            return
        future = self.active_goal.cancel_goal_async()
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            pump()
        self.active_goal = None

    def preempt_goal_after_motion(
            self, position: float, replacement: float, effort: float,
            minimum_motion: float, pump, timeout: float) -> dict:
        """Replace a moving goal and verify both action results."""
        if self.state is None or not self.state.task_aperture_valid:
            raise RuntimeError('live task aperture is unavailable before goal')
        initial_position = float(self.state.task_aperture)
        initial_sequence = int(self.state.sample_sequence)
        direction = 1.0 if position > initial_position else -1.0
        _, result_future = self._start_goal(
            position, effort, pump, timeout)
        deadline = time.monotonic() + timeout
        measured_before_preemption = initial_position
        peak_progress = 0.0
        while time.monotonic() < deadline:
            pump()
            if result_future.done():
                self.active_goal = None
                raise RuntimeError(
                    'conventional goal completed before preemption evidence')
            state = self.state
            if state is None or not state.task_aperture_valid:
                continue
            measured_before_preemption = float(state.task_aperture)
            peak_progress = max(
                peak_progress,
                direction * (measured_before_preemption - initial_position))
            if (int(state.sample_sequence) > initial_sequence and
                    state.busy and
                    abs(measured_before_preemption - initial_position) >=
                    minimum_motion):
                break
        else:
            self.cancel_active_goal(pump)
            raise RuntimeError(
                'measured motion was not observed before preemption timeout')

        preemption_requested_at = time.monotonic()
        replacement_handle, replacement_result_future = self._start_goal(
            replacement, effort, pump, timeout)
        while time.monotonic() < deadline:
            pump()
            state = self.state
            if state is not None and state.task_aperture_valid:
                peak_progress = max(
                    peak_progress,
                    direction * (
                        float(state.task_aperture) - initial_position))
            if result_future.done() and replacement_result_future.done():
                break
        if not result_future.done() or result_future.result() is None:
            self.active_goal = replacement_handle
            raise RuntimeError('superseded action result timed out')
        superseded = result_future.result()
        if superseded.status != GoalStatus.STATUS_CANCELED:
            self.active_goal = replacement_handle
            raise RuntimeError(
                'superseded action finished with status '
                f'{superseded.status}')
        if (not replacement_result_future.done() or
                replacement_result_future.result() is None):
            self.active_goal = replacement_handle
            raise RuntimeError('replacement action result timed out')
        replacement_result = replacement_result_future.result()
        self.active_goal = None
        if replacement_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                'replacement action finished with status '
                f'{replacement_result.status}')
        if (not replacement_result.result.reached_goal or
                replacement_result.result.stalled):
            raise RuntimeError(
                'replacement action did not report an unobstructed goal')

        while time.monotonic() < deadline:
            pump()
            state = self.state
            if state is None or not state.task_aperture_valid:
                continue
            current_position = float(state.task_aperture)
            peak_progress = max(
                peak_progress,
                direction * (current_position - initial_position))
            if (self.state_received_at > preemption_requested_at and
                    not state.busy and
                    abs(current_position - replacement) <= 0.0015):
                return {
                    'initial_task_aperture_m': initial_position,
                    'superseded_target_task_aperture_m': position,
                    'measured_before_preemption_m': (
                        measured_before_preemption),
                    'replacement_target_task_aperture_m': replacement,
                    'replacement_measured_task_aperture_m': current_position,
                    'peak_progress_toward_superseded_target_m': peak_progress,
                    'superseded_result_cancelled': True,
                    'replacement_result_succeeded': True,
                    'newer_idle_state_observed': True,
                }
        raise RuntimeError(
            'hardware did not publish replacement target in idle state')

    def realtime_ready(self) -> bool:
        """Return whether the active realtime controller path is observable."""
        return bool(
            self.realtime_state is not None and
            self.realtime_publisher.get_subscription_count() > 0 and
            self.realtime_stop.service_is_ready())

    def stop_realtime(self, pump, timeout: float = 3.0):
        """Request Stop and wait for realtime motion to become idle."""
        def advance():
            try:
                pump()
            except Exception:
                rclpy.spin_once(self.node, timeout_sec=0.02)

        deadline = time.monotonic() + timeout
        while (not self.realtime_stop.service_is_ready() and
               time.monotonic() < deadline):
            advance()
        if not self.realtime_stop.service_is_ready():
            raise RuntimeError('realtime Stop service is unavailable')
        stop_requested_at = time.monotonic()
        future = self.realtime_stop.call_async(Trigger.Request())
        while not future.done() and time.monotonic() < deadline:
            advance()
        if (not future.done() or future.result() is None or
                not future.result().success):
            raise RuntimeError('realtime Stop service failed')
        while time.monotonic() < deadline:
            advance()
            if (self.realtime_state is not None and
                    self.realtime_state_received_at > stop_requested_at and
                    not self.realtime_state.realtime_active):
                return {
                    'elapsed_s': time.monotonic() - stop_requested_at,
                    'newer_idle_state_observed': True,
                    'applied_command_sequence': int(
                        self.realtime_state.applied_command_sequence),
                }
        raise RuntimeError('realtime session did not become idle after Stop')

    def send_realtime_position(
            self, position: float, pump, timeout: float,
            maximum_opposed_motion: float =
            REALTIME_OPPOSED_MOTION_TOLERANCE_M) -> dict:
        """Stream one bounded position target and return measured evidence."""
        deadline = time.monotonic() + timeout
        while not self.realtime_ready() and time.monotonic() < deadline:
            pump()
        if not self.realtime_ready():
            raise RuntimeError('realtime controller path is unavailable')

        if (self.realtime_state is None or
                not self.realtime_state.task_position_valid or
                self.state is None or not self.state.task_aperture_valid or
                not math.isfinite(self.joint_position)):
            raise RuntimeError(
                'realtime task and physical position are unavailable')

        initial_state_received_at = self.realtime_state_received_at
        initial_applied_sequence = (
            int(self.realtime_state.applied_command_sequence)
            if self.realtime_state is not None else 0)
        initial_task = float(self.realtime_state.task_position)
        initial_typed_task = float(self.state.task_aperture)
        initial_physical = float(self.joint_position)
        initial_task_disagreement = abs(initial_task - initial_typed_task)
        if initial_task_disagreement > REALTIME_TASK_AGREEMENT_TOLERANCE_M:
            raise RuntimeError(
                'realtime feedback does not use the typed external-aperture '
                'coordinate before motion: disagreement '
                f'{initial_task_disagreement:.6f} m')
        direction = 1.0 if position > initial_task else -1.0
        if abs(position - initial_task) < 0.0005:
            raise RuntimeError(
                'realtime target is too close to the measured position')
        minimum_task = initial_task
        maximum_task = initial_task
        maximum_opposed = 0.0
        peak_progress = 0.0
        maximum_hold_error = 0.0
        measured_samples = 0
        last_state_received_at = initial_state_received_at
        reached = False
        reached_at = None
        stream = RealtimePositionCommandStream(
            self.realtime_publisher,
            lambda: self.node.get_clock().now().to_msg(),
            position)
        stop_result = None
        try:
            stream.start()
            while time.monotonic() < deadline:
                pump()
                stream.raise_if_failed()
                state = self.realtime_state
                if state is not None and state.faulted:
                    raise RuntimeError('realtime controller reported a fault')
                if (state is not None and state.task_position_valid and
                        self.realtime_state_received_at >
                        last_state_received_at):
                    measured_task = float(state.task_position)
                    last_state_received_at = self.realtime_state_received_at
                    measured_samples += 1
                    minimum_task = min(minimum_task, measured_task)
                    maximum_task = max(maximum_task, measured_task)
                    progress = direction * (measured_task - initial_task)
                    peak_progress = max(peak_progress, progress)
                    maximum_opposed = max(maximum_opposed, -progress)
                    if maximum_opposed > maximum_opposed_motion:
                        raise RuntimeError(
                            'realtime feedback moved opposite the commanded '
                            f'direction by {maximum_opposed:.6f} m')
                    if reached_at is not None:
                        maximum_hold_error = max(
                            maximum_hold_error,
                            abs(measured_task - position))
                        if maximum_hold_error > 0.0015:
                            raise RuntimeError(
                                'realtime position left the target tolerance '
                                f'during hold by {maximum_hold_error:.6f} m')
                if (state is not None and
                        self.realtime_state_received_at >
                        initial_state_received_at and
                        int(state.applied_command_sequence) >
                        initial_applied_sequence and
                        state.task_position_valid and
                        abs(state.task_position - position) <= 0.0015 and
                        self.state is not None and
                        self.state.task_aperture_valid and
                        abs(float(self.state.task_aperture) -
                            float(state.task_position)) <=
                        REALTIME_TASK_AGREEMENT_TOLERANCE_M):
                    if reached_at is None:
                        reached = True
                        reached_at = time.monotonic()
                if (reached_at is not None and
                        time.monotonic() - reached_at >=
                        REALTIME_TARGET_HOLD_S):
                    break
        finally:
            try:
                stream.stop()
            finally:
                stop_result = self.stop_realtime(pump)
        if not reached:
            raise RuntimeError('realtime position target timed out')
        return {
            'initial_task_aperture_m': initial_task,
            'initial_typed_task_aperture_m': initial_typed_task,
            'initial_task_feedback_disagreement_m': (
                initial_task_disagreement),
            'target_task_aperture_m': position,
            'measured_task_aperture_m': float(
                self.realtime_state.task_position),
            'measured_typed_task_aperture_m': float(
                self.state.task_aperture),
            'initial_physical_position': initial_physical,
            'measured_physical_position': float(self.joint_position),
            'minimum_observed_task_aperture_m': minimum_task,
            'maximum_observed_task_aperture_m': maximum_task,
            'peak_progress_m': peak_progress,
            'maximum_opposed_motion_m': maximum_opposed,
            'maximum_target_hold_error_m': maximum_hold_error,
            'target_hold_duration_s': REALTIME_TARGET_HOLD_S,
            'measured_feedback_samples': measured_samples,
            'published_command_count': stream.publish_count,
            'command_refresh_period_s': REALTIME_COMMAND_REFRESH_PERIOD_S,
            'initial_applied_command_sequence': initial_applied_sequence,
            'final_applied_command_sequence': int(
                self.realtime_state.applied_command_sequence),
            'stop': stop_result,
        }


def _excursion_target(initial: float, lower: float, upper: float,
                      travel: float) -> float:
    """Choose a bounded target with enough room for a restoring move."""
    opening_room = upper - initial
    closing_room = initial - lower
    if opening_room >= closing_room:
        target = initial + min(travel, opening_room)
    else:
        target = initial - min(travel, closing_room)
    if abs(target - initial) < min(0.002, travel * 0.5):
        raise RuntimeError(
            'the live position has insufficient room for the requested '
            'hardware excursion')
    return target


def _realtime_qualification_target(initial: float, lower: float,
                                   upper: float, travel: float) -> float:
    """Choose a fail-closed 2FG qualification target near full opening."""
    values = (initial, lower, upper, travel)
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError('realtime qualification limits are invalid')
    if lower >= upper or initial < lower or initial > upper:
        raise RuntimeError(
            'initial aperture is outside the reported live limits')
    if travel <= 0.0 or travel > REALTIME_QUALIFICATION_MAX_TRAVEL_M:
        raise RuntimeError(
            'realtime qualification travel must be greater than zero and no '
            f'more than {REALTIME_QUALIFICATION_MAX_TRAVEL_M:.3f} m')
    distance_from_open = upper - initial
    maximum_start_distance = (
        travel + REALTIME_QUALIFICATION_START_MARGIN_M)
    if distance_from_open > maximum_start_distance:
        raise RuntimeError(
            'realtime position qualification must start near the open '
            f'endpoint: distance {distance_from_open:.6f} m exceeds '
            f'{maximum_start_distance:.6f} m')
    target = initial - travel
    if target < lower:
        raise RuntimeError(
            'reported live range cannot accommodate the requested bounded '
            'closing excursion')
    return target


def _task_to_physical_position(model: str, task_position: float,
                               task_limits: tuple[float, float],
                               physical_limits: tuple[float, float]) -> float:
    """Map semantic aperture to the model contract's physical coordinate."""
    task_lower, task_upper = task_limits
    physical_lower, physical_upper = physical_limits
    ratio = min(1.0, max(
        0.0, (task_position - task_lower) / (task_upper - task_lower)))
    if model == 'rg2':
        return math.asin(ratio * math.sin(physical_upper))
    return physical_lower + ratio * (physical_upper - physical_lower)


def _expected_physical_delta(model: str, initial_task: float,
                             moved_task: float,
                             initial_physical: float,
                             task_limits: tuple[float, float],
                             physical_limits: tuple[float, float]) -> float:
    """Predict physical travel using the model's published CAD geometry."""
    if model in RG_CAD_GEOMETRY:
        link_length, lateral_offset, gap_offset = RG_CAD_GEOMETRY[model]
        task_lower, task_upper = task_limits
        initial_width = min(task_upper, max(task_lower, initial_task))
        moved_width = min(task_upper, max(task_lower, moved_task))
        cad_zero_phase = math.asin(lateral_offset / link_length)

        def cad_angle(width: float) -> float:
            sine = (max(0.0, width) - gap_offset) / (2.0 * link_length)
            return cad_zero_phase + math.asin(min(1.0, max(-1.0, sine)))

        return cad_angle(moved_width) - cad_angle(initial_width)
    return (_task_to_physical_position(
        model, moved_task, task_limits, physical_limits) -
        _task_to_physical_position(
        model, initial_task, task_limits, physical_limits))


def _validate_realtime_position_leg(
        result: dict, maximum_travel: float, model: str = '2fg7',
        task_limits: tuple[float, float] = (0.0, 0.107),
        physical_limits: tuple[float, float] = (0.0, 0.019),
        target_tolerance: float = 0.0015) -> dict:
    """Validate bounded task feedback and physical-coordinate agreement."""
    required = (
        'initial_task_aperture_m',
        'target_task_aperture_m',
        'measured_task_aperture_m',
        'initial_physical_position',
        'measured_physical_position',
        'minimum_observed_task_aperture_m',
        'maximum_observed_task_aperture_m',
        'maximum_opposed_motion_m',
        'measured_feedback_samples',
        'published_command_count',
        'initial_applied_command_sequence',
        'final_applied_command_sequence',
    )
    if any(key not in result for key in required):
        raise RuntimeError('realtime position evidence is incomplete')
    initial_task = float(result['initial_task_aperture_m'])
    target_task = float(result['target_task_aperture_m'])
    measured_task = float(result['measured_task_aperture_m'])
    initial_physical = float(result['initial_physical_position'])
    measured_physical = float(result['measured_physical_position'])
    values = (
        initial_task, target_task, measured_task,
        initial_physical, measured_physical,
        float(result['minimum_observed_task_aperture_m']),
        float(result['maximum_observed_task_aperture_m']),
        float(result['maximum_opposed_motion_m']),
    )
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError(
            'realtime position evidence contains non-finite data')

    maximum_excursion = max(
        abs(float(result['minimum_observed_task_aperture_m']) - initial_task),
        abs(float(result['maximum_observed_task_aperture_m']) - initial_task))
    if maximum_excursion > maximum_travel + target_tolerance:
        raise RuntimeError(
            'realtime motion exceeded the declared aperture bound: '
            f'{maximum_excursion:.6f} m')
    target_error = abs(measured_task - target_task)
    if target_error > target_tolerance:
        raise RuntimeError(
            'realtime measured aperture missed the target by '
            f'{target_error:.6f} m')
    if (float(result['maximum_opposed_motion_m']) >
            REALTIME_OPPOSED_MOTION_TOLERANCE_M):
        raise RuntimeError('realtime motion opposed the bounded command')
    if int(result['measured_feedback_samples']) < 1:
        raise RuntimeError('realtime command produced no measured feedback')
    if int(result['published_command_count']) < 2:
        raise RuntimeError('realtime target was not refreshed before Stop')
    if (int(result['final_applied_command_sequence']) <=
            int(result['initial_applied_command_sequence'])):
        raise RuntimeError('realtime controller did not apply a newer command')

    task_delta = measured_task - initial_task
    physical_delta = measured_physical - initial_physical
    expected_physical_delta = _expected_physical_delta(
        model, initial_task, measured_task, initial_physical,
        task_limits, physical_limits)
    mapping_error = abs(physical_delta - expected_physical_delta)
    physical_tolerance = (
        0.02 if model == 'rg2' else
        RG6_MAPPING_TOLERANCE_RAD if model == 'rg6' else
        REALTIME_MAPPING_TOLERANCE_M)
    if mapping_error > physical_tolerance:
        raise RuntimeError(
            'measured physical travel does not match the model coordinate '
            f'mapping: error {mapping_error:.6f}')
    result.update({
        'maximum_observed_excursion_m': maximum_excursion,
        'target_error_m': target_error,
        'task_delta_m': task_delta,
        'physical_delta': physical_delta,
        'expected_physical_delta': expected_physical_delta,
        'task_to_physical_delta_error': mapping_error,
    })
    return result


def _validate_shadow_fidelity(shadow_error: float, mimic_error: float,
                              tolerance: float) -> None:
    """Reject a run whose USD state did not follow measured hardware."""
    if shadow_error > tolerance:
        raise RuntimeError(
            'Isaac physical joint did not follow measured hardware: '
            f'error {shadow_error:.6f} exceeds {tolerance:.6f}')
    if mimic_error > tolerance:
        raise RuntimeError(
            'Isaac mimic joint did not follow the driven joint: '
            f'error {mimic_error:.6f} exceeds {tolerance:.6f}')


def _validate_hardware_liveness(observation: HardwareObservation,
                                maximum_age: float) -> None:
    """Require fresh, connected joint and typed state after readiness."""
    now = time.monotonic()
    if observation.model_mismatch is not None:
        raise RuntimeError(
            'connected state identifies model '
            f'{observation.model_mismatch}, '
            f'expected {observation.expected_model}')
    if not observation.joint_samples:
        raise RuntimeError('real joint state is unavailable')
    joint_age = now - observation.joint_received_at
    if joint_age > maximum_age:
        raise RuntimeError(f'real joint state is stale by {joint_age:.3f} s')
    if observation.state is None:
        raise RuntimeError('typed gripper state is unavailable')
    state_age = now - observation.state_received_at
    if state_age > maximum_age:
        raise RuntimeError(
            f'typed gripper state is stale by {state_age:.3f} s')
    sample_age = _reported_sample_age(observation.state)
    if (not math.isfinite(sample_age) or sample_age < 0.0 or
            sample_age > maximum_age):
        raise RuntimeError(
            f'device sample is stale by {sample_age:.3f} s')
    cycle_duration = _hardware_cycle_duration(observation.state)
    if not math.isfinite(cycle_duration) or cycle_duration < 0.0:
        raise RuntimeError('hardware cycle duration is invalid')
    if not observation.state.task_aperture_valid:
        raise RuntimeError('typed task aperture became invalid')
    if observation.state.mapping_validity != GripperState.MAPPING_VALID:
        raise RuntimeError('task-to-mechanism mapping became invalid')
    if observation.state.connection_state not in (
            GripperState.CONNECTION_IDLE,
            GripperState.CONNECTION_ACTIVE):
        raise RuntimeError(
            'hardware connection left the idle/active state')


def _validate_controller_liveness(observation: HardwareObservation,
                                  control: str,
                                  maximum_age: float) -> None:
    """Require the selected controller path to remain usable after startup."""
    if control == 'conventional':
        if not observation.action.server_is_ready():
            raise RuntimeError(
                'ParallelGripperCommand action became unavailable')
        return
    if control != 'realtime-position':
        raise RuntimeError(f'unsupported HIL control selection: {control}')
    if observation.realtime_publisher.get_subscription_count() == 0:
        raise RuntimeError(
            'realtime command subscriber became unavailable')
    if not observation.realtime_stop.service_is_ready():
        raise RuntimeError('realtime Stop service became unavailable')
    if observation.realtime_state is None:
        raise RuntimeError('realtime controller state is unavailable')
    if observation.realtime_state.faulted:
        raise RuntimeError('realtime controller reports a fault')
    state_age = time.monotonic() - observation.realtime_state_received_at
    if state_age > maximum_age:
        raise RuntimeError(
            f'realtime controller state is stale by {state_age:.3f} s')


def _hardware_health(state: GripperState) -> dict:
    """Return the monotonic hardware evidence used across a HIL run."""
    return {
        'sample_sequence': int(state.sample_sequence),
        'successful_hardware_cycles': int(state.successful_cycles),
        'failed_hardware_cycles': int(state.failed_cycles),
        'missed_deadlines': int(state.missed_deadlines),
        'watchdog_stops': int(state.watchdog_stops),
        'reconnects': int(state.reconnects),
    }


def _hardware_health_delta(initial: dict, final: dict) -> dict:
    """Calculate counter movement without hiding resets or wraparound."""
    return {
        key: final[key] - initial[key]
        for key in initial
    }


def _validate_hardware_health_delta(
        delta: dict, require_progress: bool = False) -> None:
    """Reject an ordinary qualification run with adverse device events."""
    resets = {
        key: value for key, value in delta.items() if value < 0
    }
    if resets:
        details = ', '.join(
            f'{key}={value}' for key, value in resets.items())
        raise RuntimeError(
            f'hardware health counters reset during run: {details}')
    adverse = {
        key: delta[key]
        for key in (
            'failed_hardware_cycles', 'missed_deadlines',
            'watchdog_stops', 'reconnects')
        if delta[key] != 0
    }
    if adverse:
        details = ', '.join(
            f'{key}={value}' for key, value in adverse.items())
        raise RuntimeError(
            f'adverse hardware health counters changed during run: {details}')
    if require_progress:
        stalled = [
            key for key in ('sample_sequence', 'successful_hardware_cycles')
            if delta[key] == 0
        ]
        if stalled:
            raise RuntimeError(
                'hardware health counters did not advance during timed run: ' +
                ', '.join(stalled))


def _hardware_ready(observation: HardwareObservation,
                    require_action: bool = False,
                    require_realtime: bool = False,
                    maximum_age: float | None = None) -> bool:
    """Return whether all read-only ROS hardware evidence is live."""
    now = time.monotonic()
    locally_fresh = bool(
        maximum_age is None or
        (observation.joint_received_at > 0.0 and
         observation.state_received_at > 0.0 and
         now - observation.joint_received_at <= maximum_age and
         now - observation.state_received_at <= maximum_age))
    return bool(
        observation.joint_samples and observation.state is not None and
        observation.task_limits is not None and
        locally_fresh and
        observation.state.task_aperture_valid and
        observation.state.connection_state in (
            GripperState.CONNECTION_IDLE,
            GripperState.CONNECTION_ACTIVE) and
        (not require_action or observation.action.server_is_ready()) and
        (not require_realtime or observation.realtime_ready()))


def _ready_payload(args, report, observation: HardwareObservation) -> dict:
    """Describe the verified ROS interfaces and current bounded state."""
    initial_task = float(observation.state.task_aperture)
    live_task_lower, live_task_upper = observation.task_limits
    restore_task = min(
        live_task_upper, max(live_task_lower, initial_task))
    firmware_qualification = {
        GripperState.FIRMWARE_UNKNOWN: 'unknown',
        GripperState.FIRMWARE_QUALIFIED: 'qualified',
        GripperState.FIRMWARE_UNSUPPORTED: 'unsupported',
    }.get(observation.state.firmware_qualification, 'invalid')
    realtime_reason = _runtime_realtime_position_reason(
        args.model, observation.state)
    payload = {
        'status': 'ready',
        'model': args.model,
        'mode': report['mode'],
        'joint_state_topic': _fq(args.namespace, 'joint_states'),
        'typed_state_topic': _fq(
            args.namespace, 'gripper_state_broadcaster/state'),
        'limit_names_topic': _fq(
            args.namespace, 'parallel_gripper_limit_broadcaster/names'),
        'limit_values_topic': _fq(
            args.namespace, 'parallel_gripper_limit_broadcaster/values'),
        'action': _fq(
            args.namespace, 'gripper_controller/gripper_cmd'),
        'standard_action_ready': observation.action.server_is_ready(),
        'realtime_command_topic': _fq(
            args.namespace, 'realtime_controller/command'),
        'realtime_stop_service': _fq(
            args.namespace, 'realtime_controller/stop'),
        'realtime_controller_ready': observation.realtime_ready(),
        'realtime_position_motion_qualified': (
            realtime_reason is None),
        'realtime_position_qualification_reason': realtime_reason,
        'reported_firmware': observation.state.firmware or None,
        'firmware_qualification': firmware_qualification,
        'device_profile_revision': (
            observation.state.device_profile_revision or None),
        'finger_profile_name': (
            observation.state.finger_profile_name or None),
        'finger_profile_revision': (
            observation.state.finger_profile_revision or None),
        'mapping_validity': int(observation.state.mapping_validity),
        'mapping_source': int(observation.state.mapping_source),
        'initial_task_aperture_m': initial_task,
        'initial_physical_position': observation.joint_position,
        'live_task_minimum_m': live_task_lower,
        'live_task_maximum_m': live_task_upper,
        'restore_task_aperture_m': restore_task,
        'reported_sample_age_s': _reported_sample_age(observation.state),
        'last_hardware_cycle_duration_s': _hardware_cycle_duration(
            observation.state),
    }
    payload.update(_hardware_health(observation.state))
    return payload


def _write_json(path: Path | None, payload: dict) -> None:
    """Write a report when requested."""
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')


def _run_ros_qualification(args, report) -> int:
    """Qualify the hardware-facing ROS half without starting Isaac Sim."""
    observation = None
    initialized = False
    initial_hardware_health = None
    restore_task = math.nan
    motion_started = False
    restored = False
    try:
        rclpy.init()
        initialized = True
        observation = HardwareObservation(
            args.namespace, args.task_joint, args.physical_joint, args.model)
        deadline = time.monotonic() + args.startup_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(observation.node, timeout_sec=0.02)
            if observation.model_mismatch is not None:
                raise RuntimeError(
                    'connected state identifies model '
                    f'{observation.model_mismatch}, expected {args.model}')
            if _hardware_ready(
                    observation,
                    require_action=(
                        args.hardware_control == 'conventional'),
                    require_realtime=(
                        args.hardware_control == 'realtime-position'),
                    maximum_age=args.state_timeout):
                break
        else:
            raise RuntimeError(
                'live gripper joint, typed state, task limits, and selected '
                'controller path were not available')

        _validate_hardware_liveness(observation, args.state_timeout)
        _validate_controller_liveness(
            observation, args.hardware_control, args.state_timeout)

        ready = _ready_payload(args, report, observation)
        if ((args.verify_realtime_position or
             (args.enable_hardware_motion and
              args.hardware_control == 'realtime-position')) and
                ready['realtime_position_qualification_reason'] is not None):
            raise RuntimeError(
                'realtime-position hardware motion is disabled: ' +
                ready['realtime_position_qualification_reason'])
        report['readiness'] = ready
        initial_hardware_health = _hardware_health(observation.state)
        observation.reset_timing_metrics()
        restore_task = ready['restore_task_aperture_m']
        _write_json(args.ready_output, ready)
        print(json.dumps(ready, indent=2, sort_keys=True), flush=True)
        report['tests'].append({
            'name': 'ros_hardware_preflight',
            'status': 'passed',
            'joint_state_samples': observation.joint_samples,
            'task_aperture_m': ready['initial_task_aperture_m'],
            'physical_position': ready['initial_physical_position'],
            'live_task_minimum_m': ready['live_task_minimum_m'],
            'live_task_maximum_m': ready['live_task_maximum_m'],
        })
        if args.verify_realtime_stop:
            stop_started = time.monotonic()
            observation.stop_realtime(
                lambda: rclpy.spin_once(
                    observation.node, timeout_sec=0.02),
                timeout=args.stop_timeout)
            report['tests'].append({
                'name': 'realtime_stop',
                'status': 'passed',
                'elapsed_s': time.monotonic() - stop_started,
                'newer_idle_state_observed': True,
            })
        if args.verify_realtime_position:
            def pump():
                rclpy.spin_once(observation.node, timeout_sec=0.02)
                _validate_hardware_liveness(
                    observation, args.state_timeout)
                _validate_controller_liveness(
                    observation, 'realtime-position', args.state_timeout)

            pre_motion_stop = observation.stop_realtime(
                pump, timeout=args.stop_timeout)
            pre_motion_stop.update({
                'name': 'realtime_stop_precondition',
                'status': 'passed',
            })
            report['tests'].append(pre_motion_stop)

            target = _realtime_qualification_target(
                restore_task,
                ready['live_task_minimum_m'],
                ready['live_task_maximum_m'],
                args.travel)
            motion_started = True
            excursion_result = observation.send_realtime_position(
                target, pump, args.motion_timeout)
            _validate_realtime_position_leg(
                excursion_result, args.travel, args.model,
                (ready['live_task_minimum_m'],
                 ready['live_task_maximum_m']),
                joint_limits(load_contract(args.model)))
            excursion_result.update({
                'name': 'realtime_position_bounded_excursion',
                'status': 'passed',
                'maximum_external_aperture_excursion_m': args.travel,
            })
            report['tests'].append(excursion_result)

            restore_result = observation.send_realtime_position(
                restore_task, pump, args.motion_timeout)
            _validate_realtime_position_leg(
                restore_result, args.travel, args.model,
                (ready['live_task_minimum_m'],
                 ready['live_task_maximum_m']),
                joint_limits(load_contract(args.model)))
            restored_position = float(observation.state.task_aperture)
            if abs(restored_position - restore_task) > 0.0015:
                raise RuntimeError(
                    f'hardware restored to {restored_position}, expected '
                    f'{restore_task}')
            restored = True
            restore_result.update({
                'name': 'restore_after_realtime_position',
                'status': 'passed',
                'startup_task_aperture_m': restore_task,
            })
            report['tests'].append(restore_result)
        elif args.verify_conventional_completion:
            def pump():
                rclpy.spin_once(observation.node, timeout_sec=0.02)
                _validate_hardware_liveness(
                    observation, args.state_timeout)
                _validate_controller_liveness(
                    observation, 'conventional', args.state_timeout)

            target = _excursion_target(
                restore_task,
                ready['live_task_minimum_m'],
                ready['live_task_maximum_m'],
                args.travel)
            motion_started = True
            started = time.monotonic()
            # _start_goal intentionally leaves command.velocity empty.  This
            # is the standards-compliant conventional path when a model has
            # no authoritative task-aperture velocity conversion.
            observation.send_goal(
                target, args.effort, pump, args.motion_timeout)
            completed_position = float(observation.state.task_aperture)
            if abs(completed_position - target) > 0.0015:
                raise RuntimeError(
                    f'conventional action reached {completed_position}, '
                    f'expected {target}')
            report['tests'].append({
                'name': 'conventional_empty_velocity_goal',
                'status': 'passed',
                'command_velocity': [],
                'target_task_aperture_m': target,
                'measured_task_aperture_m': completed_position,
                'elapsed_s': time.monotonic() - started,
            })
            restore_started = time.monotonic()
            observation.send_goal(
                restore_task, args.effort, pump, args.motion_timeout)
            restored_position = float(observation.state.task_aperture)
            if abs(restored_position - restore_task) > 0.0015:
                raise RuntimeError(
                    f'hardware restored to {restored_position}, expected '
                    f'{restore_task}')
            restored = True
            report['tests'].append({
                'name': 'restore_after_conventional_empty_velocity_goal',
                'status': 'passed',
                'command_velocity': [],
                'target_task_aperture_m': restore_task,
                'measured_task_aperture_m': restored_position,
                'elapsed_s': time.monotonic() - restore_started,
            })
        elif args.verify_conventional_cancel:
            def pump():
                rclpy.spin_once(observation.node, timeout_sec=0.02)
                _validate_hardware_liveness(
                    observation, args.state_timeout)
                _validate_controller_liveness(
                    observation, 'conventional', args.state_timeout)

            target = _excursion_target(
                restore_task,
                ready['live_task_minimum_m'],
                ready['live_task_maximum_m'],
                args.travel)
            motion_started = True
            cancel_result = observation.cancel_goal_after_motion(
                target, args.effort, args.cancel_after_motion,
                pump, args.motion_timeout)
            excursion = abs(
                cancel_result['stopped_task_aperture_m'] - restore_task)
            peak_excursion = cancel_result['peak_observed_excursion_m']
            if peak_excursion > args.travel + 0.001:
                raise RuntimeError(
                    'cancelled motion exceeded the declared aperture bound: '
                    f'{peak_excursion:.6f} m')
            direction = 1.0 if target > restore_task else -1.0
            remaining_to_target = direction * (
                target - cancel_result['stopped_task_aperture_m'])
            required_stop_margin = min(
                0.001, abs(target - restore_task) * 0.25)
            if remaining_to_target < required_stop_margin:
                raise RuntimeError(
                    'action cancellation did not interrupt conventional '
                    'motion before its target: remaining distance '
                    f'{remaining_to_target:.6f} m, required '
                    f'{required_stop_margin:.6f} m')
            cancel_result.update({
                'name': 'conventional_cancel_stop',
                'status': 'passed',
                'maximum_excursion_m': args.travel,
                'observed_excursion_m': excursion,
                'remaining_distance_to_target_m': remaining_to_target,
                'required_stop_margin_m': required_stop_margin,
            })
            report['tests'].append(cancel_result)
            observation.send_goal(
                restore_task, args.effort, pump, args.motion_timeout)
            restored_position = float(observation.state.task_aperture)
            if abs(restored_position - restore_task) > 0.0015:
                raise RuntimeError(
                    f'hardware restored to {restored_position}, expected '
                    f'{restore_task}')
            restored = True
            report['tests'].append({
                'name': 'restore_after_conventional_cancel',
                'status': 'passed',
                'target_task_aperture_m': restore_task,
                'measured_task_aperture_m': restored_position,
            })
        elif args.verify_conventional_preemption:
            def pump():
                rclpy.spin_once(observation.node, timeout_sec=0.02)
                _validate_hardware_liveness(
                    observation, args.state_timeout)
                _validate_controller_liveness(
                    observation, 'conventional', args.state_timeout)

            target = _excursion_target(
                restore_task,
                ready['live_task_minimum_m'],
                ready['live_task_maximum_m'],
                args.travel)
            motion_started = True
            preemption_result = observation.preempt_goal_after_motion(
                target, restore_task, args.effort,
                args.cancel_after_motion, pump, args.motion_timeout)
            peak_progress = preemption_result[
                'peak_progress_toward_superseded_target_m']
            target_distance = abs(target - preemption_result[
                'initial_task_aperture_m'])
            remaining_to_superseded_target = target_distance - peak_progress
            required_stop_margin = min(0.001, target_distance * 0.25)
            if peak_progress > args.travel + 0.001:
                raise RuntimeError(
                    'preempted motion exceeded the declared aperture bound: '
                    f'{peak_progress:.6f} m')
            if remaining_to_superseded_target < required_stop_margin:
                raise RuntimeError(
                    'preemption did not interrupt conventional motion before '
                    'its superseded target: remaining distance '
                    f'{remaining_to_superseded_target:.6f} m, required '
                    f'{required_stop_margin:.6f} m')
            restored = True
            preemption_result.update({
                'name': 'conventional_preemption_stop',
                'status': 'passed',
                'maximum_excursion_m': args.travel,
                'remaining_distance_to_superseded_target_m': (
                    remaining_to_superseded_target),
                'required_stop_margin_m': required_stop_margin,
            })
            report['tests'].append(preemption_result)
        stability_started = time.monotonic()
        while time.monotonic() - stability_started < args.duration:
            rclpy.spin_once(observation.node, timeout_sec=0.02)
            _validate_hardware_liveness(
                observation, args.state_timeout)
            _validate_controller_liveness(
                observation, args.hardware_control, args.state_timeout)
        final_hardware_health = _hardware_health(observation.state)
        hardware_health_delta = _hardware_health_delta(
            initial_hardware_health, final_hardware_health)
        _validate_hardware_health_delta(
            hardware_health_delta,
            require_progress=(
                args.verify_realtime_position or args.duration > 0.0))
        report['final_hardware_health'] = final_hardware_health
        report['hardware_health_delta'] = hardware_health_delta
        report['timing_observation'] = observation.timing_metrics()
        report['tests'].append({
            'name': 'ros_hardware_stability',
            'status': 'passed',
            'duration_s': time.monotonic() - stability_started,
            'counter_delta': hardware_health_delta,
        })
        report['status'] = 'passed'
        return 0
    except KeyboardInterrupt:
        report['status'] = 'stopped'
        return 0
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
        report['traceback'] = traceback.format_exc().splitlines()
        return 1
    finally:
        if (motion_started and not restored and observation is not None and
                math.isfinite(restore_task) and
                observation.model_mismatch is None and
                time.monotonic() - observation.joint_received_at <=
                args.state_timeout):
            report['cleanup_restore_attempted'] = True
            try:
                def cleanup_pump():
                    rclpy.spin_once(
                        observation.node, timeout_sec=0.02)

                if args.verify_realtime_position:
                    if (observation.realtime_state is not None and
                            observation.realtime_state.realtime_active):
                        observation.stop_realtime(cleanup_pump)
                    observation.send_realtime_position(
                        restore_task, cleanup_pump, args.motion_timeout)
                else:
                    observation.cancel_active_goal(cleanup_pump)
                    observation.send_goal(
                        restore_task, args.effort, cleanup_pump,
                        args.motion_timeout)
                report['cleanup_restore_succeeded'] = True
            except Exception as cleanup_error:
                report['cleanup_restore_succeeded'] = False
                report['cleanup_restore_error'] = (
                    f'{type(cleanup_error).__name__}: {cleanup_error}')
        report['joint_state_samples'] = (
            observation.joint_samples if observation is not None else 0)
        if (observation is not None and
                observation.timing_metrics_enabled and
                'timing_observation' not in report):
            report['timing_observation'] = observation.timing_metrics()
        if (observation is not None and observation.state is not None and
                'final_hardware_health' not in report):
            final_hardware_health = _hardware_health(observation.state)
            report['final_hardware_health'] = final_hardware_health
            if initial_hardware_health is not None:
                report['hardware_health_delta'] = _hardware_health_delta(
                    initial_hardware_health, final_hardware_health)
        report['finished_at'] = datetime.now(timezone.utc).isoformat()
        _write_json(args.output, report)
        print(json.dumps(report, indent=2, sort_keys=True), flush=True)
        if observation is not None:
            observation.destroy()
        if initialized and rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    """Run a read-only shadow or an explicitly enabled HIL motion test."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--model', choices=SUPPORTED_HIL_MODELS, required=True,
        help='real gripper model; Isaac modes also require a model asset')
    parser.add_argument(
        '--asset', type=Path,
        help='USD entry point; defaults to the installed model asset')
    parser.add_argument(
        '--namespace', default='', help='ROS namespace of real bringup')
    parser.add_argument(
        '--task-joint', default='grip_stroke',
        help='standard task-aperture joint name')
    parser.add_argument(
        '--physical-joint',
        help='measured mechanism joint; defaults from the selected model')
    parser.add_argument(
        '--physics-dt', type=float, default=1.0 / 120.0,
        help='Isaac physics step in seconds')
    parser.add_argument(
        '--state-timeout', type=float, default=0.5,
        help='maximum age of real joint state in seconds')
    parser.add_argument(
        '--shadow-tolerance', type=float, default=0.001,
        help=(
            'maximum hardware-to-USD and mimic error in the physical joint '
            'SI unit (meters for linear joints, radians for angular joints)'))
    parser.add_argument(
        '--startup-timeout', type=float, default=30.0,
        help='seconds to wait for the selected ROS interfaces')
    parser.add_argument(
        '--duration', type=float, default=0.0,
        help=(
            'read-only preflight monitoring duration; for an Isaac shadow, '
            'zero runs until interrupted'))
    parser.add_argument(
        '--output', type=Path, help='machine-readable qualification report')
    parser.add_argument(
        '--ready-output', type=Path,
        help='optional path for the initial readiness record')
    parser.add_argument(
        '--gui', action='store_true', help='show the Isaac Sim window')
    parser.add_argument(
        '--ros-preflight-only', action='store_true',
        help=(
            'verify ROS hardware interfaces without importing Isaac or '
            'moving'))
    parser.add_argument(
        '--verify-realtime-stop', action='store_true',
        help=(
            'during realtime ROS preflight, request Stop and require a newer '
            'idle controller state; sends no motion command'))
    parser.add_argument(
        '--verify-conventional-completion', action='store_true',
        help=(
            'without Isaac, send one bounded conventional action with the '
            'standard empty velocity array, require completion, and restore'))
    parser.add_argument(
        '--verify-conventional-cancel', action='store_true',
        help=(
            'without Isaac, start one bounded conventional action, cancel '
            'after measured motion, require stable idle state, and restore'))
    parser.add_argument(
        '--verify-conventional-preemption', action='store_true',
        help=(
            'without Isaac, start one bounded conventional action, replace '
            'it after measured motion with the restore target, and verify '
            'both action results'))
    parser.add_argument(
        '--verify-realtime-position', action='store_true',
        help=(
            'without Isaac, qualify 2FG command-6 external-aperture '
            'semantics using one bounded closing excursion, independently '
            'refreshed commands, explicit Stop, and restoration'))
    parser.add_argument(
        '--enable-hardware-motion', action='store_true',
        help='permit one bounded excursion and restoration')
    parser.add_argument(
        '--hardware-control',
        choices=('conventional', 'realtime-position'),
        default='conventional',
        help='controller path to preflight or use for bounded motion')
    parser.add_argument(
        '--confirm-model', choices=SUPPORTED_HIL_MODELS,
        help='second explicit model confirmation required for motion')
    parser.add_argument(
        '--travel', type=float, default=0.010,
        help='maximum external-aperture excursion in meters (max 0.020)')
    parser.add_argument(
        '--effort', type=float, default=10.0,
        help='conventional action effort in newtons (max 40)')
    parser.add_argument(
        '--motion-timeout', type=float, default=20.0,
        help='timeout per motion or restoration in seconds')
    parser.add_argument(
        '--stop-timeout', type=float, default=3.0,
        help='timeout for explicit realtime Stop qualification in seconds')
    parser.add_argument(
        '--cancel-after-motion', type=float, default=0.001,
        help='measured aperture change required before cancellation in meters')
    args, _ = parser.parse_known_args()

    if args.enable_hardware_motion and args.confirm_model != args.model:
        parser.error(
            '--enable-hardware-motion requires --confirm-model matching '
            '--model')
    if args.enable_hardware_motion and args.output is None:
        parser.error('--enable-hardware-motion requires --output')
    if (args.enable_hardware_motion and
            args.hardware_control == 'realtime-position' and
            args.model not in REALTIME_POSITION_MOTION_QUALIFIED_MODELS):
        parser.error(
            'realtime-position hardware motion is disabled for this model '
            'because ' + _realtime_position_qualification_reason(args.model) +
            '; use read-only preflight/shadow or conventional bounded motion')
    if args.ros_preflight_only and args.enable_hardware_motion:
        parser.error('--ros-preflight-only does not command hardware')
    if (args.verify_realtime_stop and
            (not args.ros_preflight_only or
             args.hardware_control != 'realtime-position')):
        parser.error(
            '--verify-realtime-stop requires --ros-preflight-only and '
            '--hardware-control realtime-position')
    if (args.verify_realtime_stop and args.confirm_model != args.model):
        parser.error(
            '--verify-realtime-stop requires --confirm-model matching '
            '--model')
    if args.verify_realtime_stop and args.output is None:
        parser.error('--verify-realtime-stop requires --output')
    if args.verify_realtime_position:
        if (args.ros_preflight_only or args.enable_hardware_motion or
                args.verify_realtime_stop or
                args.verify_conventional_cancel or
                args.verify_conventional_preemption):
            parser.error(
                '--verify-realtime-position is a standalone ROS-hardware '
                'qualification')
        if args.hardware_control != 'realtime-position':
            parser.error(
                '--verify-realtime-position requires --hardware-control '
                'realtime-position')
        if args.confirm_model != args.model:
            parser.error(
                '--verify-realtime-position requires --confirm-model '
                'matching --model')
        if args.output is None:
            parser.error('--verify-realtime-position requires --output')
        if args.travel > REALTIME_QUALIFICATION_MAX_TRAVEL_M:
            parser.error(
                '--verify-realtime-position permits at most 0.004 m travel')
    conventional_verifications = (
        args.verify_conventional_completion,
        args.verify_conventional_cancel,
        args.verify_conventional_preemption)
    if sum(conventional_verifications) > 1:
        parser.error(
            'conventional completion, cancellation, and preemption '
            'verifications are mutually exclusive')
    if (any(conventional_verifications) and
            (args.ros_preflight_only or args.enable_hardware_motion or
             args.verify_realtime_stop or
             args.hardware_control != 'conventional')):
        parser.error(
            'conventional action verification is a standalone conventional '
            'ROS-hardware qualification')
    if (any(conventional_verifications) and
            args.confirm_model != args.model):
        parser.error(
            'conventional action verification requires --confirm-model '
            'matching --model')
    if any(conventional_verifications) and args.output is None:
        parser.error('conventional action verification requires --output')
    if (not args.enable_hardware_motion and
            not args.ros_preflight_only and
            not args.verify_realtime_position and
            args.hardware_control != 'conventional'):
        parser.error(
            '--hardware-control requires hardware motion or ROS preflight')
    if (args.physics_dt <= 0.0 or args.state_timeout <= 0.0 or
            args.shadow_tolerance <= 0.0 or
            args.startup_timeout <= 0.0 or args.duration < 0.0 or
            args.travel <= 0.0 or args.travel > 0.020 or
            args.effort <= 0.0 or args.effort > 40.0 or
            args.motion_timeout <= 0.0 or args.stop_timeout <= 0.0 or
            args.cancel_after_motion <= 0.0 or
            args.cancel_after_motion >= args.travel):
        parser.error('invalid timing, travel, or effort limit')

    ros_only = bool(
        args.ros_preflight_only or args.verify_conventional_completion or
        args.verify_conventional_cancel or
        args.verify_conventional_preemption or
        args.verify_realtime_position)
    contract = None
    root_path = None
    lower = None
    upper = None
    if ros_only:
        if args.physical_joint is None:
            args.physical_joint = DEFAULT_PHYSICAL_JOINTS[args.model]
        joint_name = args.physical_joint
    else:
        if args.model not in ISAAC_HIL_MODELS:
            parser.error(
                f'Isaac shadow/motion is not available for {args.model}; '
                'use a ROS-only qualification mode')
        contract = load_contract(args.model)
        root_path = articulation_path(contract)
        joint_name = driven_joint(contract)
        if args.physical_joint is None:
            args.physical_joint = joint_name
        lower, upper = joint_limits(contract)
    if not args.task_joint or not args.physical_joint:
        parser.error('task and physical joint names must not be empty')
    report = {
        'schema_version': 1,
        'model': args.model,
        'ros_distro': os.environ.get('ROS_DISTRO'),
        'rmw_implementation': rclpy.get_rmw_implementation_identifier(),
        'runner_sha256': _sha256(Path(__file__).resolve()),
        'acceptance_limits': _acceptance_limits(args),
        'namespace': args.namespace.strip('/'),
        'mode': (
            'ros-realtime-position-qualification'
            if args.verify_realtime_position else
            'ros-preflight' if args.ros_preflight_only else
            'ros-conventional-completion'
            if args.verify_conventional_completion else
            'ros-conventional-cancel'
            if args.verify_conventional_cancel else
            'ros-conventional-preemption'
            if args.verify_conventional_preemption else
            'bounded-hardware-motion' if args.enable_hardware_motion else
            'read-only-shadow'),
        'command_authority': (
            'ros2-control-typed-realtime-bounded-position'
            if args.verify_realtime_position else
            'ros2-control-stop-only' if args.verify_realtime_stop else
            'ros2-control-standard-action-empty-velocity'
            if args.verify_conventional_completion else
            'ros2-control-standard-action-bounded-cancel'
            if args.verify_conventional_cancel else
            'ros2-control-standard-action-bounded-preemption'
            if args.verify_conventional_preemption else
            'none-preflight' if args.ros_preflight_only else
            'none-read-only' if not args.enable_hardware_motion else
            'ros2-control-typed-realtime-topic'
            if args.hardware_control == 'realtime-position' else
            'ros2-control-standard-action'),
        'hardware_motion_enabled': (
            args.enable_hardware_motion or args.verify_conventional_completion or
            args.verify_conventional_cancel or
            args.verify_conventional_preemption or
            args.verify_realtime_position),
        'hardware_control': args.hardware_control,
        'hardware_owner': 'external-real-backend-bringup',
        'shadow_coordinate': joint_name,
        'task_joint': args.task_joint,
        'physical_joint': args.physical_joint,
        'started_at': datetime.now(timezone.utc).isoformat(),
        'status': 'failed',
        'tests': [],
    }
    if args.model in ISAAC_HIL_MODELS:
        coordinate_dimension, coordinate_unit = coordinate_metadata(
            load_contract(args.model))
        report['physical_coordinate'] = {
            'joint': args.physical_joint,
            'dimension': coordinate_dimension,
            'unit': coordinate_unit,
        }
    if (args.ros_preflight_only or args.verify_conventional_completion or
            args.verify_conventional_cancel or
            args.verify_conventional_preemption or
            args.verify_realtime_position):
        return _run_ros_qualification(args, report)

    if args.asset is None:
        args.asset = default_asset(args.model)
    report['asset'] = str(args.asset.resolve())
    report['physics_backend'] = 'physx'

    from isaacsim import SimulationApp
    app = None
    observation = None
    rclpy_initialized = False
    articulation = None
    driven_index = None
    initial_task = math.nan
    restore_task = math.nan
    maximum_shadow_error = 0.0
    maximum_mimic_error = 0.0
    last_mirrored_sample = 0
    pump_callback = None
    motion_started = False
    restored_in_main = False
    readiness_established = False
    initial_hardware_health = None
    exit_code = 1

    try:
        launch_config = {
            'headless': not args.gui,
            'width': 1280,
            'height': 720,
        }
        if args.gui:
            launch_config['renderer'] = 'RaytracedLighting'
        app = SimulationApp(launch_config)

        import omni.usd
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.core.version import get_version

        report['isaac_sim_version'] = get_version()[0]
        if not args.asset.is_file():
            raise RuntimeError(f'asset does not exist: {args.asset}')
        report['asset_files_sha256'] = _asset_manifest(args.asset)
        if not omni.usd.get_context().open_stage(str(args.asset.resolve())):
            raise RuntimeError(f'Isaac Sim could not open {args.asset}')
        app.update()
        app.update()
        while is_stage_loading():
            app.update()

        setup_simulation(SimulationManager, dt=args.physics_dt, device='cpu')
        play()
        # The first PhysX frame can block for seconds while Isaac initializes
        # physics and renderer resources.  Complete that work before creating
        # ROS subscriptions so it cannot be mistaken for stale hardware.
        for _ in range(4):
            SimulationManager.step()
            app.update()
        articulation = Articulation(root_path)
        if joint_name not in articulation.dof_names:
            raise RuntimeError(
                f'{joint_name} missing from runtime DOFs: '
                f'{articulation.dof_names}')
        driven_index = articulation.dof_names.index(joint_name)
        articulation.switch_dof_control_mode(
            'position', dof_indices=[driven_index])
        dof_names = list(articulation.dof_names)
        expected_mimics = [
            item['name'] for item in
            contract['usd'].get('mimic_joints', [])
        ]
        missing_mimics = [
            name for name in expected_mimics if name not in dof_names
        ]
        if missing_mimics:
            raise RuntimeError(
                'contract mimic DOFs missing from runtime articulation: '
                f'{missing_mimics}')
        reference_array = articulation.get_dof_positions()
        reference_positions = [
            _value(reference_array, 0, index)
            for index in range(len(dof_names))
        ]
        report['articulation_path'] = root_path
        report['dof_names'] = list(articulation.dof_names)
        report['shadow_mimic_dofs'] = expected_mimics
        report['shadow_mimic_dof_count'] = len(expected_mimics)
        report['shadow_simulation_mode'] = 'live-measured-state-rendering'

        rclpy.init()
        rclpy_initialized = True
        observation = HardwareObservation(
            args.namespace, args.task_joint, args.physical_joint, args.model)

        def pump():
            nonlocal maximum_shadow_error, maximum_mimic_error
            nonlocal last_mirrored_sample
            mirrored = None
            target = math.nan
            _drain_ros_callbacks(observation.node)
            if observation.model_mismatch is not None:
                raise RuntimeError(
                    'connected state identifies model '
                    f'{observation.model_mismatch}, expected {args.model}')
            if observation.joint_samples:
                target = min(upper, max(lower, observation.joint_position))
                mirrored = _shadow_dof_positions(
                    contract, dof_names, reference_positions,
                    driven_index, target)
                _mirror_measured_position(
                    articulation, driven_index, mirrored)
                last_mirrored_sample = observation.joint_samples
            SimulationManager.step()
            app.update()
            if mirrored is not None:
                # Measure after PhysX advances: reading immediately after the
                # setter would only prove that teleportation accepted values,
                # not that the rendered linkage remained coherent.
                positions = articulation.get_dof_positions()
                simulated = _value(positions, 0, driven_index)
                maximum_shadow_error = max(
                    maximum_shadow_error,
                    abs(simulated - target))
                for index, expected in mirrored.items():
                    if index == driven_index:
                        continue
                    follower = _value(positions, 0, index)
                    maximum_mimic_error = max(
                        maximum_mimic_error, abs(follower - expected))
            # Isaac may briefly monopolize the CPU during rendering or shader
            # work.  Consume the newest queued hardware samples before using
            # local receipt time as a communication-health signal.
            _drain_ros_callbacks(observation.node)
            if observation.model_mismatch is not None:
                raise RuntimeError(
                    'connected state identifies model '
                    f'{observation.model_mismatch}, expected {args.model}')
            if readiness_established:
                _validate_hardware_liveness(
                    observation, args.state_timeout)
                if args.enable_hardware_motion:
                    _validate_controller_liveness(
                        observation, args.hardware_control,
                        args.state_timeout)

        pump_callback = pump

        startup_deadline = time.monotonic() + args.startup_timeout
        while time.monotonic() < startup_deadline:
            pump()
            if _hardware_ready(
                    observation,
                    require_action=(
                        args.enable_hardware_motion and
                        args.hardware_control == 'conventional'),
                    require_realtime=(
                        args.enable_hardware_motion and
                        args.hardware_control == 'realtime-position'),
                    maximum_age=args.state_timeout):
                break
        else:
            raise RuntimeError(
                f'live {args.model} joint, typed state, and task limits were '
                'not available for the selected mode')

        _validate_hardware_liveness(observation, args.state_timeout)
        if args.enable_hardware_motion:
            _validate_controller_liveness(
                observation, args.hardware_control, args.state_timeout)

        ready = _ready_payload(args, report, observation)
        if (args.enable_hardware_motion and
                args.hardware_control == 'realtime-position' and
                ready['realtime_position_qualification_reason'] is not None):
            raise RuntimeError(
                'realtime-position hardware motion is disabled: ' +
                ready['realtime_position_qualification_reason'])
        report['readiness'] = ready
        initial_hardware_health = _hardware_health(observation.state)
        report['startup_shadow_alignment'] = {
            'maximum_physical_coordinate_shadow_error': maximum_shadow_error,
            'maximum_mimic_coordinate_error': maximum_mimic_error,
        }
        # Asset initialization starts from its authored pose.  Retain that
        # convergence transient for diagnosis, but qualify fidelity only
        # after live hardware readiness establishes the shadow coordinate.
        maximum_shadow_error = 0.0
        maximum_mimic_error = 0.0
        observation.reset_timing_metrics()
        readiness_established = True
        initial_task = ready['initial_task_aperture_m']
        initial_physical = observation.joint_position
        live_task_lower = ready['live_task_minimum_m']
        live_task_upper = ready['live_task_maximum_m']
        restore_task = ready['restore_task_aperture_m']
        _write_json(args.ready_output, ready)
        print(json.dumps(ready, indent=2, sort_keys=True), flush=True)

        if args.enable_hardware_motion:
            def command_target(target):
                if args.hardware_control == 'realtime-position':
                    observation.send_realtime_position(
                        target, pump, args.motion_timeout)
                else:
                    observation.send_goal(
                        target, args.effort, pump, args.motion_timeout)

            target = _excursion_target(
                restore_task, live_task_lower, live_task_upper, args.travel)
            motion_started = True
            command_target(target)
            moved_task = float(observation.state.task_aperture)
            moved_physical = observation.joint_position
            if abs(moved_task - target) > 0.0015:
                raise RuntimeError(
                    f'hardware reached {moved_task}, expected {target}')
            report['tests'].append({
                'name': 'bounded_hardware_motion',
                'status': 'passed',
                'control': args.hardware_control,
                'target_task_aperture_m': target,
                'measured_task_aperture_m': moved_task,
                'measured_physical_position': moved_physical,
            })
            task_delta = moved_task - initial_task
            physical_delta = moved_physical - initial_physical
            expected_physical_delta = _expected_physical_delta(
                args.model, initial_task, moved_task, initial_physical,
                (live_task_lower, live_task_upper), (lower, upper))
            mechanism_delta_error = abs(
                physical_delta - expected_physical_delta)
            physical_tolerance = (
                0.02 if args.model == 'rg2' else
                RG6_MAPPING_TOLERANCE_RAD if args.model == 'rg6' else 0.002)
            if mechanism_delta_error > physical_tolerance:
                raise RuntimeError(
                    'measured physical travel does not match the model '
                    'coordinate mapping: error '
                    f'{mechanism_delta_error:.6f}')
            report['tests'].append({
                'name': 'task_to_mechanism_delta',
                'status': 'passed',
                'task_delta_m': task_delta,
                'physical_delta': physical_delta,
                'expected_physical_delta': expected_physical_delta,
                'delta_error': mechanism_delta_error,
            })
            command_target(restore_task)
            restored_task = float(observation.state.task_aperture)
            if abs(restored_task - restore_task) > 0.0015:
                raise RuntimeError(
                    f'hardware restored to {restored_task}, expected valid '
                    f'aperture {restore_task}')
            restored_in_main = True
            report['tests'].append({
                'name': 'restore_valid_initial_aperture',
                'status': 'passed',
                'target_task_aperture_m': restore_task,
                'startup_task_aperture_m': initial_task,
                'measured_task_aperture_m': restored_task,
                'measured_physical_position': observation.joint_position,
            })
        else:
            started = time.monotonic()
            while app.is_running() and (
                    args.duration == 0.0 or
                    time.monotonic() - started < args.duration):
                pump()
            report['tests'].append({
                'name': 'measured_state_shadow',
                'status': 'passed',
                'duration_s': time.monotonic() - started,
            })

        if last_mirrored_sample == 0:
            raise RuntimeError('no real joint state was mirrored into Isaac')
        _validate_shadow_fidelity(
            maximum_shadow_error, maximum_mimic_error,
            args.shadow_tolerance)
        report['tests'].append({
            'name': 'hardware_to_usd_shadow_fidelity',
            'status': 'passed',
            'maximum_physical_coordinate_shadow_error': maximum_shadow_error,
            'maximum_mimic_coordinate_error': maximum_mimic_error,
            'physical_coordinate_tolerance': args.shadow_tolerance,
        })
        final_hardware_health = _hardware_health(observation.state)
        hardware_health_delta = _hardware_health_delta(
            initial_hardware_health, final_hardware_health)
        report['final_hardware_health'] = final_hardware_health
        report['hardware_health_delta'] = hardware_health_delta
        report['timing_observation'] = observation.timing_metrics()
        _validate_hardware_health_delta(
            hardware_health_delta,
            require_progress=(
                args.enable_hardware_motion or args.duration > 0.0))
        report['tests'].append({
            'name': 'hardware_health_stable',
            'status': 'passed',
            'counter_delta': hardware_health_delta,
        })
        report['status'] = 'passed'
        exit_code = 0
    except KeyboardInterrupt:
        if not args.enable_hardware_motion and last_mirrored_sample:
            report['status'] = 'stopped'
            exit_code = 0
        else:
            report['error'] = 'KeyboardInterrupt: interrupted'
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
        report['traceback'] = traceback.format_exc().splitlines()
    finally:
        if observation is not None and articulation is not None:
            try:
                if args.hardware_control == 'realtime-position':
                    if (observation.realtime_state is not None and
                            observation.realtime_state.realtime_active):
                        observation.stop_realtime(
                            lambda: rclpy.spin_once(
                                observation.node, timeout_sec=0.02))
                else:
                    observation.cancel_active_goal(
                        lambda: rclpy.spin_once(
                            observation.node, timeout_sec=0.02))
            except Exception:
                pass
        if (args.enable_hardware_motion and motion_started and
                not restored_in_main and observation is not None and
                pump_callback is not None and math.isfinite(restore_task) and
                observation.model_mismatch is None and
                time.monotonic() - observation.joint_received_at <=
                args.state_timeout):
            report['cleanup_restore_attempted'] = True
            try:
                if args.hardware_control == 'realtime-position':
                    observation.send_realtime_position(
                        restore_task, pump_callback, args.motion_timeout)
                else:
                    observation.send_goal(
                        restore_task, args.effort, pump_callback,
                        args.motion_timeout)
                report['cleanup_restore_succeeded'] = True
            except Exception as cleanup_error:
                report['cleanup_restore_succeeded'] = False
                report['cleanup_restore_error'] = (
                    f'{type(cleanup_error).__name__}: {cleanup_error}')
        report['maximum_physical_coordinate_shadow_error'] = (
            maximum_shadow_error)
        report['maximum_mimic_coordinate_error'] = maximum_mimic_error
        report['joint_state_samples'] = (
            observation.joint_samples if observation is not None else 0)
        if (observation is not None and
                observation.timing_metrics_enabled and
                'timing_observation' not in report):
            report['timing_observation'] = observation.timing_metrics()
        if (observation is not None and observation.state is not None and
                'final_hardware_health' not in report):
            final_hardware_health = _hardware_health(observation.state)
            report['final_hardware_health'] = final_hardware_health
            if initial_hardware_health is not None:
                report['hardware_health_delta'] = _hardware_health_delta(
                    initial_hardware_health, final_hardware_health)
        report['finished_at'] = datetime.now(timezone.utc).isoformat()
        _write_json(args.output, report)
        print(json.dumps(report, indent=2, sort_keys=True), flush=True)
        if observation is not None:
            observation.destroy()
        if rclpy_initialized and rclpy.ok():
            rclpy.shutdown()
        if app is not None:
            try:
                stop()
            except Exception:
                pass
            app.close(exit_code=exit_code)

    return exit_code


if __name__ == '__main__':
    sys.exit(main())

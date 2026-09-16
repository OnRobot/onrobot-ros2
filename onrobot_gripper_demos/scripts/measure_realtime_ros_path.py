#!/usr/bin/python3
"""Exercise and measure a running realtime ros2_control gripper path."""

import argparse
import json
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from control_msgs.msg import Float64Values, Keys

from controller_manager_msgs.srv import ListControllers, SwitchController

from onrobot_gripper_msgs.msg import GripperState
from onrobot_gripper_msgs.msg import RealtimeCommand
from onrobot_gripper_msgs.msg import RealtimeState

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState

from std_msgs.msg import String

from std_srvs.srv import Trigger


MODELS = ('2fg7', '2fg14', 'rg2', 'rg6')


def fq(namespace, name):
    """Return an absolute name below an optional namespace."""
    namespace = namespace.strip('/')
    return f'/{namespace}/{name}' if namespace else f'/{name}'


def percentile(values, fraction):
    """Return a linearly interpolated percentile, or None for no samples."""
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def timing_summary(values):
    """Summarize one timing distribution."""
    return {
        'count': len(values),
        'percentile_50': percentile(values, 0.50),
        'percentile_95': percentile(values, 0.95),
        'percentile_99': percentile(values, 0.99),
        'max': max(values) if values else None,
    }


def hardware_parameters(description):
    """Extract ros2_control hardware parameters from a URDF document."""
    root = ET.fromstring(description)
    systems = []
    for control in root.findall('.//ros2_control'):
        hardware = control.find('hardware')
        if hardware is None:
            continue
        systems.append({
            'name': control.get('name', ''),
            'plugin': (hardware.findtext('plugin') or '').strip(),
            'parameters': {
                item.get('name', ''): (item.text or '').strip()
                for item in hardware.findall('param')
            },
        })
    return systems


def matching_rtu_system(description, model, serial_device=None,
                        baud_rate=None, slave_id=None):
    """Find the selected real RTU hardware system or return None."""
    model_token = model.replace('fg', 'FG').upper()
    expected_plugin = (
        'onrobot_gripper_hardware/OnRobotRgSystem'
        if model.startswith('rg') else
        'onrobot_gripper_hardware/OnRobotGripperSystem')
    for system in hardware_parameters(description):
        parameters = system['parameters']
        identity = f"{system['name']} {system['plugin']}".upper()
        if (system['plugin'] == expected_plugin and
                parameters.get('transport') == 'rtu' and
                model_token in identity and
                (serial_device is None or
                 parameters.get('serial_device') == serial_device) and
                (baud_rate is None or
                 parameters.get('baud_rate') == str(baud_rate)) and
                (slave_id is None or
                 parameters.get('slave_id') == str(slave_id)) and
                parameters.get('rtu_parity') == 'even'):
            return system
    return None


def rate_mismatches(system, controllers, target_rate_hz):
    """Describe a wrong device rate or controller rates below the target."""
    mismatches = []
    configured_rate = system['parameters'].get('realtime_update_rate_hz')
    try:
        hardware_rate = int(configured_rate)
    except (TypeError, ValueError):
        mismatches.append(
            'hardware realtime_update_rate_hz is missing or invalid')
    else:
        if hardware_rate != target_rate_hz:
            mismatches.append(
                f'hardware realtime_update_rate_hz={hardware_rate}')

    for name in ('realtime_controller', 'gripper_state_broadcaster'):
        controller = controllers.get(name)
        if controller is None:
            mismatches.append(f'{name} is missing')
        elif controller.get('update_rate', 0) < target_rate_hz:
            mismatches.append(
                f"{name} update_rate={controller.get('update_rate')}")
    return mismatches


class RosPathObserver(Node):
    """Collect live state and own the bounded test commands."""

    def __init__(self, namespace, model):
        """Create the state observers and control clients."""
        super().__init__('onrobot_realtime_ros_path_measurement')
        self.namespace = namespace
        self.model = model
        self.description = None
        self.joint_state = None
        self.gripper_state = None
        self.realtime_state = None
        self.limit_names = []
        self.limit_values = []
        self.state_times = []
        self.state_sample_sequences = []
        self.realtime_times = []
        self.realtime_cycles = []
        self.realtime_missed = []
        self.sample_ages = []
        self.task_samples = []

        sensor_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        reliable_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            String, fq(namespace, 'robot_description'), self._description,
            latched_qos)
        self.create_subscription(
            JointState, fq(namespace, 'joint_states'), self._joint, sensor_qos)
        self.create_subscription(
            GripperState, fq(namespace, 'gripper_state_broadcaster/state'),
            self._gripper, sensor_qos)
        self.create_subscription(
            RealtimeState, fq(namespace, 'realtime_controller/state'),
            self._realtime, sensor_qos)
        self.create_subscription(
            Keys, fq(namespace, 'parallel_gripper_limit_broadcaster/names'),
            self._limit_names, latched_qos)
        self.create_subscription(
            Float64Values,
            fq(namespace, 'parallel_gripper_limit_broadcaster/values'),
            self._limit_values, latched_qos)
        self.publisher = self.create_publisher(
            RealtimeCommand, fq(namespace, 'realtime_controller/command'),
            reliable_qos)
        self.stop_client = self.create_client(
            Trigger, fq(namespace, 'realtime_controller/stop'))
        self.list_client = self.create_client(
            ListControllers,
            fq(namespace, 'controller_manager/list_controllers'))
        self.switch_client = self.create_client(
            SwitchController,
            fq(namespace, 'controller_manager/switch_controller'))

    def _description(self, message):
        self.description = message.data

    def _joint(self, message):
        self.joint_state = message

    def _gripper(self, message):
        now = time.monotonic()
        self.gripper_state = message
        if message.task_aperture_valid:
            self.task_samples.append(float(message.task_aperture))
        if (not self.state_sample_sequences or
                message.sample_sequence != self.state_sample_sequences[-1]):
            self.state_times.append(now)
            self.state_sample_sequences.append(int(message.sample_sequence))
            age = message.sample_age.sec + message.sample_age.nanosec * 1e-9
            if math.isfinite(age) and age >= 0.0:
                self.sample_ages.append(age)

    def _realtime(self, message):
        now = time.monotonic()
        self.realtime_state = message
        if (not self.realtime_cycles or
                message.successful_cycles != self.realtime_cycles[-1]):
            self.realtime_times.append(now)
            self.realtime_cycles.append(int(message.successful_cycles))
            self.realtime_missed.append(int(message.missed_deadlines))

    def _limit_names(self, message):
        self.limit_names = list(message.keys)

    def _limit_values(self, message):
        self.limit_values = [float(value) for value in message.values]

    def spin_until(self, predicate, timeout, error):
        """Process callbacks until a predicate succeeds or time expires."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        raise RuntimeError(error)

    def spin_for(self, duration):
        """Process callbacks for a monotonic duration."""
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)

    def task_limits(self):
        """Return valid task-aperture limits when both are available."""
        if len(self.limit_names) != len(self.limit_values):
            return None
        keys = ('grip_stroke/minimum_task_aperture',
                'grip_stroke/maximum_task_aperture')
        try:
            values = tuple(
                self.limit_values[self.limit_names.index(key)] for key in keys)
        except ValueError:
            return None
        return values if values[1] > values[0] else None

    def controller_states(self):
        """Return controller state and update-rate observations."""
        if not self.list_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                'controller-manager list service is unavailable')
        future = self.list_client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('controller-manager list request timed out')
        return {
            controller.name: {
                'state': controller.state,
                'update_rate': int(controller.update_rate),
            }
            for controller in future.result().controller
        }

    def wait_controller_states(self, expected, timeout=10.0):
        """Wait until named controllers reach their expected states."""
        observed = {}
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            observed = self.controller_states()
            if all(observed.get(name, {}).get('state') == state
                   for name, state in expected.items()):
                return observed
            self.spin_for(0.05)
        raise RuntimeError(
            f'controller states did not converge: expected={expected}, '
            f'observed={observed}')

    def switch(self, activate, deactivate):
        """Perform one strict controller ownership switch."""
        if not self.switch_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                'controller-manager switch service is unavailable')
        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 5
        future = self.switch_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)
        if (not future.done() or future.result() is None or
                not future.result().ok):
            message = future.result().message if future.done() else 'timeout'
            raise RuntimeError(f'controller switch failed: {message}')

    def stop(self):
        """Request Stop and require inactive realtime state."""
        if not self.stop_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('realtime Stop service is unavailable')
        future = self.stop_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if (not future.done() or future.result() is None or
                not future.result().success):
            message = future.result().message if future.done() else 'timeout'
            raise RuntimeError(f'realtime Stop failed: {message}')
        self.spin_until(
            lambda: self.realtime_state is not None and
            not self.realtime_state.realtime_active,
            3.0, 'realtime Stop was not reflected in state')
        return future.result().message

    def publish_velocity(self, direction):
        """Publish one bounded model-aware velocity command."""
        message = RealtimeCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.mode = RealtimeCommand.VELOCITY
        if self.model.startswith('rg'):
            message.mechanism_angular_velocity = direction * 0.2
        else:
            message.task_velocity = direction * 0.010
        self.publisher.publish(message)


def run(args):
    """Run one bounded RTU ROS-path measurement."""
    rclpy.init()
    node = RosPathObserver(args.namespace, args.model)
    report = {
        'schema_version': 1,
        'model': args.model,
        'required_transport': 'rtu',
        'target_rate_hz': args.rate_hz,
        'status': 'failed',
        'tests': [],
    }
    try:
        node.spin_until(
            lambda: node.description is not None and
            node.gripper_state is not None and
            node.realtime_state is not None and
            node.joint_state is not None and
            node.task_limits() is not None,
            args.startup_timeout, 'live ROS gripper state is incomplete')
        system = matching_rtu_system(
            node.description, args.model, args.serial_device,
            args.baud_rate, args.slave_id)
        if system is None:
            raise RuntimeError(
                'live robot_description does not contain the selected RTU '
                'hardware system and serial device')
        if node.gripper_state.model != args.model:
            raise RuntimeError(
                f'live model is {node.gripper_state.model}, not {args.model}')
        if (node.gripper_state.firmware_qualification !=
                GripperState.FIRMWARE_QUALIFIED):
            raise RuntimeError(
                'live firmware is not qualified for realtime control')
        if (node.gripper_state.mapping_validity !=
                GripperState.MAPPING_VALID):
            raise RuntimeError('live task mapping is not valid')
        if (node.gripper_state.fault_code != 0 or
                node.gripper_state.safety_dc_error):
            raise RuntimeError(
                'live gripper has a device or safety fault')
        controllers = node.wait_controller_states({
            'realtime_controller': 'active',
            'gripper_controller': 'inactive',
        })
        mismatches = rate_mismatches(system, controllers, args.rate_hz)
        if mismatches:
            raise RuntimeError(
                f'live rates do not match --rate-hz {args.rate_hz}: ' +
                ', '.join(mismatches))
        report['hardware'] = system
        report['controllers_before'] = controllers
        report['tests'].append({'name': 'rtu_identity_and_readiness',
                                'status': 'passed'})

        minimum, maximum = node.task_limits()
        lower = minimum + 0.40 * (maximum - minimum)
        upper = minimum + 0.60 * (maximum - minimum)
        initial = float(node.gripper_state.task_aperture)
        direction = -1.0 if initial >= 0.5 * (lower + upper) else 1.0
        baseline = node.realtime_state
        baseline_time = time.monotonic()
        node.task_samples = [initial]
        node.state_times = []
        node.state_sample_sequences = []
        node.realtime_times = []
        node.realtime_cycles = []
        node.realtime_missed = []
        node.sample_ages = []
        directions = set()
        next_command = baseline_time
        end = baseline_time + args.duration
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_command:
                current = float(node.gripper_state.task_aperture)
                if direction > 0.0 and current >= upper:
                    direction = -1.0
                elif direction < 0.0 and current <= lower:
                    direction = 1.0
                node.publish_velocity(direction)
                directions.add(int(direction))
                next_command += 0.02
            rclpy.spin_once(node, timeout_sec=0.005)

        elapsed = time.monotonic() - baseline_time
        final = node.realtime_state
        completed = int(final.successful_cycles - baseline.successful_cycles)
        failures = int(final.failed_cycles - baseline.failed_cycles)
        missed = int(final.missed_deadlines - baseline.missed_deadlines)
        watchdog_stops = int(
            final.watchdog_stops - baseline.watchdog_stops)
        reconnects = int(final.reconnects - baseline.reconnects)
        completed_rate = completed / elapsed
        missed_rate = missed / max(1, completed + failures)
        travel = max(node.task_samples) - min(node.task_samples)
        state_intervals = [
            right - left for left, right in
            zip(node.state_times, node.state_times[1:])]
        realtime_intervals = [
            right - left for left, right in
            zip(node.realtime_times, node.realtime_times[1:])]
        typed_state_rate = (
            len(state_intervals) / sum(state_intervals)
            if state_intervals else 0.0)
        realtime_state_rate = (
            len(realtime_intervals) / sum(realtime_intervals)
            if realtime_intervals else 0.0)
        maximum_sample_age = max(node.sample_ages, default=math.inf)
        period = 1.0 / args.rate_hz
        acceptance = {
            'device_exchange_rate':
                completed_rate >= 0.99 * args.rate_hz,
            'no_failed_exchanges': failures == 0,
            'no_reconnects': reconnects == 0,
            'no_watchdog_stops': watchdog_stops == 0,
            'deadline_budget': missed_rate <= 0.001,
            'fresh_typed_state_rate':
                typed_state_rate >= 0.99 * args.rate_hz,
            'realtime_state_rate':
                realtime_state_rate >= 0.99 * args.rate_hz,
            'sample_freshness': maximum_sample_age <= 4.0 * period,
            'bidirectional_motion': len(directions) == 2,
            'minimum_travel': travel >= 0.002,
            'command_applied': final.applied_command_sequence >
                baseline.applied_command_sequence,
            'fresh_typed_samples': bool(
                node.state_sample_sequences and
                node.state_sample_sequences[-1] >
                node.state_sample_sequences[0]),
        }
        failed_acceptance_checks = [
            name for name, passed in acceptance.items() if not passed]
        health_passed = not failed_acceptance_checks
        motion = {
            'name': 'bounded_realtime_motion_and_rate',
            'status': 'passed' if health_passed else 'failed',
            'acceptance': acceptance,
            'failed_acceptance_checks': failed_acceptance_checks,
            'duration_s': elapsed,
            'completed_cycles': completed,
            'completed_rate_hz': completed_rate,
            'failed_cycles': failures,
            'missed_deadlines': missed,
            'missed_deadline_rate': missed_rate,
            'watchdog_stops': watchdog_stops,
            'reconnects': reconnects,
            'task_minimum_m': min(node.task_samples),
            'task_maximum_m': max(node.task_samples),
            'task_travel_m': travel,
            'both_directions_commanded': len(directions) == 2,
            'fresh_typed_state_rate_hz': typed_state_rate,
            'realtime_state_rate_hz': realtime_state_rate,
            'fresh_typed_state_interval_s': timing_summary(state_intervals),
            'realtime_state_interval_s': timing_summary(realtime_intervals),
            'sample_age_s': timing_summary(node.sample_ages),
            'requested_command_sequence': int(
                final.requested_command_sequence),
            'applied_command_sequence': int(final.applied_command_sequence),
        }
        report['tests'].append(motion)
        if not health_passed:
            raise RuntimeError('bounded realtime ROS-path acceptance failed')

        sequence_before_stale_stop = int(final.applied_command_sequence)
        watchdogs_before_stale_stop = int(final.watchdog_stops)
        node.spin_until(
            lambda: node.realtime_state is not None and
            not node.realtime_state.realtime_active and
            node.realtime_state.applied_command_sequence >
            sequence_before_stale_stop,
            3.0, 'publisher loss did not produce an ordered Stop')
        stopped_position = float(node.gripper_state.task_aperture)
        node.spin_for(0.30)
        drift = abs(float(node.gripper_state.task_aperture) - stopped_position)
        stale_watchdog_delta = int(
            node.realtime_state.watchdog_stops -
            watchdogs_before_stale_stop)
        if drift > 0.001 or stale_watchdog_delta != 0:
            raise RuntimeError(
                'ROS stale-input Stop was late or allowed gripper drift')
        report['tests'].append({
            'name': 'stale_ros_command_stop', 'status': 'passed',
            'applied_sequence_before': sequence_before_stale_stop,
            'applied_sequence_after': int(
                node.realtime_state.applied_command_sequence),
            'hardware_watchdog_stop_delta': stale_watchdog_delta,
            'post_stop_drift_m': drift,
        })

        stop_message = node.stop()
        before_switch = float(node.gripper_state.task_aperture)
        node.switch(['gripper_controller'], ['realtime_controller'])
        node.wait_controller_states({
            'realtime_controller': 'inactive',
            'gripper_controller': 'active',
        })
        node.switch(['realtime_controller'], ['gripper_controller'])
        after_states = node.wait_controller_states({
            'realtime_controller': 'active',
            'gripper_controller': 'inactive',
        })
        node.spin_for(0.30)
        after_switch = float(node.gripper_state.task_aperture)
        switch_drift = abs(after_switch - before_switch)
        if switch_drift > 0.001 or node.realtime_state.realtime_active:
            raise RuntimeError(
                'controller switch replayed motion or changed aperture')
        report['tests'].append({
            'name': 'stop_and_controller_round_trip', 'status': 'passed',
            'stop_message': stop_message,
            'task_drift_m': switch_drift,
            'controllers_after': after_states,
        })
        report['status'] = 'passed'
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        try:
            if node.realtime_state is not None:
                node.stop()
        except Exception as error:
            report['cleanup_error'] = f'{type(error).__name__}: {error}'
            report['status'] = 'failed'
        node.destroy_node()
        rclpy.shutdown()
    return report


def main():
    """Run the requested measurement and preserve the JSON result."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=MODELS, required=True)
    parser.add_argument('--namespace', default='')
    parser.add_argument('--serial-device', required=True)
    parser.add_argument('--baud-rate', type=int, choices=(115200, 1000000),
                        default=1000000)
    parser.add_argument('--slave-id', type=int, choices=(65, 66, 67),
                        default=65)
    parser.add_argument('--rate-hz', type=int, choices=range(1, 501),
                        metavar='1..500', required=True)
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--startup-timeout', type=float, default=30.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if (not math.isfinite(args.duration) or
            args.duration < 5.0 or args.duration > 3600.0):
        parser.error('--duration must be between 5 and 3600 seconds')
    if (not math.isfinite(args.startup_timeout) or
            args.startup_timeout <= 0.0 or args.startup_timeout > 300.0):
        parser.error('--startup-timeout must be between 0 and 300 seconds')

    report = run(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    sys.exit(main())

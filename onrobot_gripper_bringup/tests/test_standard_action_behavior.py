"""End-to-end standard action cancellation and preemption qualification."""

import time
import unittest

from action_msgs.msg import GoalStatus

from ament_index_python.packages import get_package_share_directory

from control_msgs.action import ParallelGripperCommand

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_testing
from launch_testing.actions import ReadyToTest

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState


TEST_NAMESPACE = 'standard_action_behavior'
ACTION_NAME = f'/{TEST_NAMESPACE}/gripper_controller/gripper_cmd'
JOINT_STATES_TOPIC = f'/{TEST_NAMESPACE}/joint_states'


def generate_test_description():
    """Launch slow deterministic fake motion so goals remain interruptible."""
    source = PythonLaunchDescriptionSource([
        get_package_share_directory('onrobot_gripper_bringup'),
        '/launch/gripper.launch.py',
    ])
    bringup = IncludeLaunchDescription(source, launch_arguments={
        'model': '2fg7',
        'backend': 'fake',
        'namespace': TEST_NAMESPACE,
        'frame_prefix': f'{TEST_NAMESPACE}_',
        'fake_motion_speed_m_s': '0.02',
        'fake_stall': '0',
    }.items())
    return LaunchDescription([bringup, ReadyToTest()])


class StandardActionBehavior(unittest.TestCase):
    """Exercise the same ParallelGripperCommand endpoint used by MoveIt."""

    @classmethod
    def setUpClass(cls):
        """Create the action client and measured-state observer."""
        rclpy.init()
        cls.node = rclpy.create_node('standard_action_behavior_test')
        cls.client = ActionClient(
            cls.node, ParallelGripperCommand, ACTION_NAME)
        cls.position = None
        cls.finger_position = None
        cls.node.create_subscription(
            JointState, JOINT_STATES_TOPIC, cls._on_joint_state,
            qos_profile_sensor_data)

    @classmethod
    def tearDownClass(cls):
        """Release ROS entities after the launch test."""
        cls.client.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_joint_state(cls, message):
        if 'grip_stroke' in message.name:
            cls.position = message.position[
                message.name.index('grip_stroke')]
        if 'finger_stroke' in message.name:
            cls.finger_position = message.position[
                message.name.index('finger_stroke')]

    def _spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail('condition was not reached before timeout')

    def _send_goal(self, position):
        goal = ParallelGripperCommand.Goal()
        goal.command.name = ['grip_stroke']
        goal.command.position = [position]
        goal.command.effort = [20.0]
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertTrue(future.done())
        handle = future.result()
        self.assertTrue(handle.accepted)
        return handle

    def _result(self, handle, timeout=8.0):
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done())
        return future.result()

    def test_cancel_preempt_and_success_results(self):
        """Require canonical terminal states and held position on cancel."""
        self.assertTrue(self.client.wait_for_server(timeout_sec=30.0))
        self._spin_until(lambda: self.position is not None, 30.0)

        cancel_goal = self._send_goal(0.0)
        self._spin_until(lambda: self.position < 0.068, 3.0)
        cancel_future = cancel_goal.cancel_goal_async()
        rclpy.spin_until_future_complete(
            self.node, cancel_future, timeout_sec=3.0)
        self.assertTrue(cancel_future.done())
        self.assertEqual(len(cancel_future.result().goals_canceling), 1)
        canceled = self._result(cancel_goal)
        self.assertEqual(canceled.status, GoalStatus.STATUS_CANCELED)
        position_after_cancel = self.position
        self._spin_until(
            lambda: abs(self.position - position_after_cancel) < 0.0005, 0.5)
        time.sleep(0.3)
        rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertAlmostEqual(
            self.position, position_after_cancel, delta=0.0005)

        immediate_start = self.position
        immediate_target = 0.0 if immediate_start > 0.0365 else 0.073
        immediate_goal = self._send_goal(immediate_target)
        immediate_cancel = immediate_goal.cancel_goal_async()
        rclpy.spin_until_future_complete(
            self.node, immediate_cancel, timeout_sec=3.0)
        self.assertTrue(immediate_cancel.done())
        self.assertEqual(len(immediate_cancel.result().goals_canceling), 1)
        immediate_result = self._result(immediate_goal)
        self.assertEqual(
            immediate_result.status, GoalStatus.STATUS_CANCELED)
        immediate_stopped_position = self.position
        time.sleep(0.3)
        rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertAlmostEqual(
            self.position, immediate_stopped_position, delta=0.0005)
        self.assertGreater(
            abs(self.position - immediate_target), 0.005,
            'an immediately cancelled target must not be replayed')

        position_after_cancel = self.position
        superseded_goal = self._send_goal(0.0)
        self._spin_until(
            lambda: self.position < position_after_cancel - 0.002, 2.0)
        replacement_goal = self._send_goal(0.060)
        superseded = self._result(superseded_goal)
        replacement = self._result(replacement_goal)
        self.assertEqual(superseded.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(replacement.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(replacement.result.reached_goal)
        self.assertFalse(replacement.result.stalled)
        self.assertAlmostEqual(
            replacement.result.state.position[0], 0.060, delta=0.001)
        self.assertEqual(list(replacement.result.state.effort), [20.0])
        self._spin_until(lambda: self.finger_position is not None, 2.0)
        # Stock outward silicone pads have a 33 mm gap at physical joint zero;
        # both jaws move equally. Do not rescale stroke by a legacy 0..73 mm
        # task range: the public aperture and one jaw's travel are distinct.
        expected_finger_position = (0.060 - 0.033) / 2.0
        self.assertAlmostEqual(
            self.finger_position, expected_finger_position, delta=0.0005)


@launch_testing.post_shutdown_test()
class ProcessesExitCleanly(unittest.TestCase):
    """Check that launch-managed processes accept normal test shutdown."""

    def test_exit_codes(self, proc_info):
        """Require clean or launch-approved signal exit codes."""
        # The ordered inactive realtime spawner can still be waiting on its
        # controller-manager service when launch_testing tears the graph down.
        # SIGINT is the expected launch-managed shutdown path in that case.
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2])

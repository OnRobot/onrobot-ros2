"""End-to-end standard grasp-stall result qualification."""

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


TEST_NAMESPACE = 'standard_action_stall'
ACTION_NAME = f'/{TEST_NAMESPACE}/gripper_controller/gripper_cmd'


def generate_test_description():
    """Launch a fake mechanism that reports no motion under command."""
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
        'fake_stall': '1',
    }.items())
    return LaunchDescription([bringup, ReadyToTest()])


class StandardActionStall(unittest.TestCase):
    """Verify the configured contact-stall interpretation."""

    @classmethod
    def setUpClass(cls):
        """Create the standard action client."""
        rclpy.init()
        cls.node = rclpy.create_node('standard_action_stall_test')
        cls.client = ActionClient(
            cls.node, ParallelGripperCommand, ACTION_NAME)

    @classmethod
    def tearDownClass(cls):
        """Release ROS entities after the launch test."""
        cls.client.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_contact_stall_is_a_successful_grasp(self):
        """Return succeeded/stalled rather than reached-goal on contact."""
        self.assertTrue(self.client.wait_for_server(timeout_sec=30.0))
        goal = ParallelGripperCommand.Goal()
        goal.command.name = ['grip_stroke']
        goal.command.position = [0.0]
        goal.command.effort = [20.0]
        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=5.0)
        self.assertTrue(send.done())
        handle = send.result()
        self.assertTrue(handle.accepted)

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node, result_future, timeout_sec=8.0)
        self.assertTrue(result_future.done())
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(wrapped.result.stalled)
        self.assertFalse(wrapped.result.reached_goal)
        self.assertEqual(list(wrapped.result.state.effort), [20.0])


@launch_testing.post_shutdown_test()
class ProcessesExitCleanly(unittest.TestCase):
    """Check that launch-managed processes accept normal test shutdown."""

    def test_exit_codes(self, proc_info):
        """Require clean or launch-approved signal exit codes."""
        # The final inactive realtime spawner is dynamically added by the
        # ordered launch chain.  launch_testing may interrupt that short-lived
        # helper while tearing down the test after lifecycle readiness and the
        # action result have already been proven.
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2])

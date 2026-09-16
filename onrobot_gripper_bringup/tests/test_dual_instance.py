"""Prove two independently namespaced grippers can run together."""

import time
import unittest

from action_msgs.msg import GoalStatus

from ament_index_python.packages import get_package_share_directory

from control_msgs.action import ParallelGripperCommand
from controller_manager_msgs.srv import ListControllers

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_testing
from launch_testing.actions import ReadyToTest

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from tf2_msgs.msg import TFMessage


def _instance(namespace, frame_prefix):
    source = PythonLaunchDescriptionSource([
        get_package_share_directory('onrobot_gripper_bringup'),
        '/launch/gripper.launch.py',
    ])
    return IncludeLaunchDescription(source, launch_arguments={
        'model': '2fg7',
        'backend': 'fake',
        'namespace': namespace,
        'frame_prefix': frame_prefix,
    }.items())


def generate_test_description():
    """Launch two fake grippers with separate ROS and TF identities."""
    return LaunchDescription([
        _instance('left_gripper', 'left_gripper_'),
        _instance('right_gripper', 'right_gripper_'),
        ReadyToTest(),
    ])


class DualInstance(unittest.TestCase):
    """Exercise isolated actions and frame graphs."""

    @classmethod
    def setUpClass(cls):
        """Create both action clients and the global static-TF observer."""
        rclpy.init()
        cls.node = rclpy.create_node('dual_gripper_instance_test')
        cls.left = ActionClient(
            cls.node, ParallelGripperCommand,
            '/left_gripper/gripper_controller/gripper_cmd')
        cls.right = ActionClient(
            cls.node, ParallelGripperCommand,
            '/right_gripper/gripper_controller/gripper_cmd')
        cls.controller_lists = {
            namespace: cls.node.create_client(
                ListControllers,
                f'/{namespace}/controller_manager/list_controllers',
            )
            for namespace in ('left_gripper', 'right_gripper')
        }
        cls.frames = set()
        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.node.create_subscription(
            TFMessage, '/tf_static', cls._on_tf, qos)

    @classmethod
    def tearDownClass(cls):
        """Release ROS entities after the launch test."""
        cls.left.destroy()
        cls.right.destroy()
        for client in cls.controller_lists.values():
            client.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_tf(cls, message):
        for transform in message.transforms:
            cls.frames.add(transform.child_frame_id)

    def _goal_result(self, client, position):
        goal = ParallelGripperCommand.Goal()
        goal.command.name = ['grip_stroke']
        goal.command.position = [position]
        # Zero selects the model's documented conventional minimum and keeps
        # this multi-instance test independent of model-specific minima.
        goal.command.effort = [0.0]
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=5.0)
        self.assertTrue(send.done())
        self.assertTrue(send.result().accepted)
        result = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=5.0)
        self.assertTrue(result.done())
        return result.result()

    def _wait_for_realtime_controller_readiness(self, timeout=30.0):
        """Wait until both ordered chains have configured realtime control."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ready = True
            for client in self.controller_lists.values():
                if not client.service_is_ready():
                    client.wait_for_service(timeout_sec=0.05)
                    ready = False
                    continue
                future = client.call_async(ListControllers.Request())
                rclpy.spin_until_future_complete(
                    self.node, future, timeout_sec=0.5)
                if not future.done() or future.result() is None:
                    ready = False
                    continue
                states = {
                    item.name: item.state
                    for item in future.result().controller
                }
                if states.get('realtime_controller') != 'inactive':
                    ready = False
            if ready:
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.fail('realtime controllers did not reach inactive readiness')

    def test_actions_and_tf_are_isolated(self):
        """Command distinct apertures and require distinct base frames."""
        self.assertTrue(self.left.wait_for_server(timeout_sec=30.0))
        self.assertTrue(self.right.wait_for_server(timeout_sec=30.0))
        left_result = self._goal_result(self.left, 0.040)
        right_result = self._goal_result(self.right, 0.060)
        self.assertEqual(left_result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(right_result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertAlmostEqual(
            left_result.result.state.position[0], 0.040, delta=0.001)
        self.assertAlmostEqual(
            right_result.result.state.position[0], 0.060, delta=0.001)
        self._wait_for_realtime_controller_readiness()

        deadline = self.node.get_clock().now().nanoseconds + 5_000_000_000
        expected = {'left_gripper_base_link', 'right_gripper_base_link'}
        while not expected <= self.frames:
            self.assertLess(self.node.get_clock().now().nanoseconds, deadline)
            rclpy.spin_once(self.node, timeout_sec=0.1)


@launch_testing.post_shutdown_test()
class ProcessesExitCleanly(unittest.TestCase):
    """Check that launch-managed processes accept normal test shutdown."""

    def test_exit_codes(self, proc_info):
        """Require clean or launch-approved signal exit codes."""
        # The final inactive realtime spawner is dynamically added by the
        # ordered launch chain.  launch_testing may interrupt that short-lived
        # helper while tearing down the test after lifecycle readiness has
        # already been proven.
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2])

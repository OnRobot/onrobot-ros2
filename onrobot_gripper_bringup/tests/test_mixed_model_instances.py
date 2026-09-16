"""Prove unlike parallel-gripper models can share one ROS graph."""

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


INSTANCES = {
    'wide_gripper': {
        'model': '2fg14',
        'frame_prefix': 'wide_gripper_',
        'target_m': 0.100,
    },
    'rg_gripper': {
        'model': 'rg2',
        'frame_prefix': 'rg_gripper_',
        'target_m': 0.080,
    },
}


def _instance(namespace, configuration):
    source = PythonLaunchDescriptionSource([
        get_package_share_directory('onrobot_gripper_bringup'),
        '/launch/gripper.launch.py',
    ])
    return IncludeLaunchDescription(source, launch_arguments={
        'model': configuration['model'],
        'backend': 'fake',
        'namespace': namespace,
        'frame_prefix': configuration['frame_prefix'],
    }.items())


def generate_test_description():
    """Launch a 2FG14 and RG2 with separate ROS and TF identities."""
    return LaunchDescription([
        *[
            _instance(namespace, configuration)
            for namespace, configuration in INSTANCES.items()
        ],
        ReadyToTest(),
    ])


class MixedModelInstances(unittest.TestCase):
    """Exercise isolated actions and frame graphs for unlike models."""

    @classmethod
    def setUpClass(cls):
        """Create model-specific action clients and a static-TF observer."""
        rclpy.init()
        cls.node = rclpy.create_node('mixed_gripper_instance_test')
        cls.clients = {
            namespace: ActionClient(
                cls.node,
                ParallelGripperCommand,
                f'/{namespace}/gripper_controller/gripper_cmd',
            )
            for namespace in INSTANCES
        }
        cls.controller_lists = {
            namespace: cls.node.create_client(
                ListControllers,
                f'/{namespace}/controller_manager/list_controllers',
            )
            for namespace in INSTANCES
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
        for client in cls.clients.values():
            client.destroy()
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
        # Zero selects the model's documented conventional minimum.  This
        # keeps the mixed-model test valid for both the 2FG14 and the RG2.
        goal.command.effort = [0.0]
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=5.0)
        self.assertTrue(send.done())
        self.assertTrue(send.result().accepted)
        result = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=5.0)
        self.assertTrue(result.done())
        return result.result()

    def _wait_for_controller_readiness(self, timeout=120.0):
        """Require all ordered controller chains to reach their final state."""
        expected = {
            'joint_state_broadcaster': 'active',
            'parallel_gripper_limit_broadcaster': 'active',
            'gripper_state_broadcaster': 'active',
            'recovery_controller': 'active',
            'gripper_controller': 'active',
            'realtime_controller': 'inactive',
        }
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
                if any(states.get(name) != state
                       for name, state in expected.items()):
                    ready = False
            if ready:
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.fail('controller chains did not reach readiness')

    def test_actions_and_tf_are_isolated_across_models(self):
        """Command each model and require its own result and TF base."""
        self._wait_for_controller_readiness()
        for client in self.clients.values():
            self.assertTrue(client.wait_for_server(timeout_sec=30.0))

        results = {
            namespace: self._goal_result(
                self.clients[namespace], configuration['target_m'])
            for namespace, configuration in INSTANCES.items()
        }
        for namespace, configuration in INSTANCES.items():
            result = results[namespace]
            self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
            self.assertAlmostEqual(
                result.result.state.position[0],
                configuration['target_m'],
                delta=0.001,
            )

        deadline = self.node.get_clock().now().nanoseconds + 5_000_000_000
        expected = {
            f"{configuration['frame_prefix']}base_link"
            for configuration in INSTANCES.values()
        }
        while not expected <= self.frames:
            self.assertLess(self.node.get_clock().now().nanoseconds, deadline)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self._wait_for_controller_readiness()


@launch_testing.post_shutdown_test()
class ProcessesExitCleanly(unittest.TestCase):
    """Check that launch-managed processes accept normal test shutdown."""

    def test_exit_codes(self, proc_info):
        """Require clean or launch-approved signal exit codes."""
        # The last dynamically spawned helper may be interrupted by launch
        # teardown after manager readiness and action behavior have passed.
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2])

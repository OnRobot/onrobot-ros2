"""Require every supported model to expose a complete controller graph."""

import time
import unittest

from ament_index_python.packages import get_package_share_directory

from controller_manager_msgs.srv import ListControllers

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_testing.actions import ReadyToTest

import rclpy


INSTANCES = {
    'startup_2fg7': '2fg7',
    'startup_2fg14': '2fg14',
    'startup_rg2': 'rg2',
    'startup_rg6': 'rg6',
}

EXPECTED_STATES = {
    'joint_state_broadcaster': 'active',
    'parallel_gripper_limit_broadcaster': 'active',
    'gripper_state_broadcaster': 'active',
    'recovery_controller': 'active',
    'gripper_controller': 'active',
    'realtime_controller': 'inactive',
}


def generate_test_description():
    """Launch every supported model with conventional ownership."""
    source = PythonLaunchDescriptionSource([
        get_package_share_directory('onrobot_gripper_bringup'),
        '/launch/gripper.launch.py',
    ])
    launches = [
        IncludeLaunchDescription(source, launch_arguments={
            'model': model,
            'backend': 'fake',
            'namespace': namespace,
            'frame_prefix': f'{namespace}_',
        }.items())
        for namespace, model in INSTANCES.items()
    ]
    return LaunchDescription([*launches, ReadyToTest()])


class ControllerStartupReadiness(unittest.TestCase):
    """Check lifecycle readiness after the ordered spawner chain settles."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('controller_startup_readiness_test')
        cls.clients = {
            namespace: cls.node.create_client(
                ListControllers,
                f'/{namespace}/controller_manager/list_controllers',
            )
            for namespace in INSTANCES
        }

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _states(self, client):
        request = ListControllers.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=1.0)
        if not future.done() or future.result() is None:
            return None
        return {
            item.name: item.state
            for item in future.result().controller
        }

    def test_all_models_reach_complete_startup(self):
        """A delayed controller must not leave a partial ROS graph."""
        deadline = time.monotonic() + 120.0
        observed = {}
        while time.monotonic() < deadline:
            for namespace, client in self.clients.items():
                if not client.service_is_ready():
                    client.wait_for_service(timeout_sec=0.05)
                    continue
                states = self._states(client)
                if states is not None:
                    observed[namespace] = states
            if all(
                    all(states.get(name) == state
                        for name, state in EXPECTED_STATES.items())
                    for states in observed.values()) and \
                    len(observed) == len(INSTANCES):
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.fail(f'controller readiness timed out: {observed}')

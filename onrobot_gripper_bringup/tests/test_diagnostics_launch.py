"""Verify standard diagnostics publication and namespace isolation."""

import time
import unittest

from ament_index_python.packages import get_package_share_directory

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_testing
from launch_testing.actions import ReadyToTest

import rclpy
from rclpy.qos import qos_profile_system_default


NAMESPACES = ('diagnostics_2fg7', 'diagnostics_rg2')


def generate_test_description():
    """Launch two fake grippers with the ordinary public bringup."""
    source = PythonLaunchDescriptionSource([
        get_package_share_directory('onrobot_gripper_bringup'),
        '/launch/gripper.launch.py',
    ])
    launches = []
    for model, namespace in zip(('2fg7', 'rg2'), NAMESPACES):
        launches.append(IncludeLaunchDescription(source, launch_arguments={
            'model': model,
            'backend': 'fake',
            'namespace': namespace,
            'frame_prefix': f'{namespace}_',
            'fake_motion_speed_m_s': '0.02',
        }.items()))
    return LaunchDescription([*launches, ReadyToTest()])


class DiagnosticsLaunch(unittest.TestCase):
    """Check the user-visible DiagnosticArray contract."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('diagnostics_launch_test')
        cls.messages = {}
        cls.subscriptions = []
        for namespace in NAMESPACES:
            topic = f'/{namespace}/diagnostics'
            cls.subscriptions.append(cls.node.create_subscription(
                DiagnosticArray,
                topic,
                lambda message, name=namespace: cls.messages.__setitem__(
                    name, message),
                qos_profile_system_default))

    @classmethod
    def tearDownClass(cls):
        for subscription in cls.subscriptions:
            cls.node.destroy_subscription(subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout=45.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return
        self.fail('diagnostic messages were not received before timeout')

    @staticmethod
    def _values(message):
        assert len(message.status) == 1
        return message.status[0], {
            value.key: value.value for value in message.status[0].values}

    def test_each_instance_publishes_a_namespaced_standard_status(self):
        """Require one status per instance and no merged global stream."""
        self._spin_until(lambda: set(self.messages) == set(NAMESPACES))
        for namespace in NAMESPACES:
            status, values = self._values(self.messages[namespace])
            self.assertTrue(status.name.endswith('/gripper'))
            self.assertIn(namespace[-3:], status.hardware_id)
            self.assertIn('model', values)
            self.assertIn('connection_state', values)
            self.assertIn('sample_age_s', values)
            self.assertIn('diagnostic_status_valid', values)
            self.assertIn('diagnostic_raw_status', values)
            self.assertIn('diagnostic_busy', values)
            self.assertIn('diagnostic_grip_detected', values)
            self.assertEqual(status.level, DiagnosticStatus.OK)
            self.assertEqual(values['diagnostic_status_valid'], '1.000000')


@launch_testing.post_shutdown_test()
class ProcessesExitCleanly(unittest.TestCase):
    """Check that launch-managed processes accept normal test shutdown."""

    def test_exit_codes(self, proc_info):
        """Require clean or launch-approved signal exit codes."""
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2])

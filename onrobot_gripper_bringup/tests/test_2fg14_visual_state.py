"""Verify that 2FG14 task commands drive its physical URDF articulation."""

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
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState


TEST_NAMESPACE = 'two_fg14_visual_state'
ACTION_NAME = f'/{TEST_NAMESPACE}/gripper_controller/gripper_cmd'
JOINT_STATES_TOPIC = f'/{TEST_NAMESPACE}/joint_states'
LIST_CONTROLLERS_SERVICE = (
    f'/{TEST_NAMESPACE}/controller_manager/list_controllers')


def generate_test_description():
    """Launch the model-neutral bringup with a fake 2FG14."""
    source = PythonLaunchDescriptionSource([
        get_package_share_directory('onrobot_gripper_bringup'),
        '/launch/gripper.launch.py',
    ])
    bringup = IncludeLaunchDescription(source, launch_arguments={
        'model': '2fg14',
        'backend': 'fake',
        'namespace': TEST_NAMESPACE,
        'frame_prefix': f'{TEST_NAMESPACE}_',
    }.items())
    return LaunchDescription([bringup, ReadyToTest()])


class TwoFG14VisualState(unittest.TestCase):
    """Observe the physical one-jaw coordinate after a standard action."""

    @classmethod
    def setUpClass(cls):
        """Create the action client and joint-state observer."""
        rclpy.init()
        cls.node = rclpy.create_node('two_fg14_visual_state_test')
        cls.client = ActionClient(
            cls.node, ParallelGripperCommand, ACTION_NAME)
        cls.list_controllers = cls.node.create_client(
            ListControllers, LIST_CONTROLLERS_SERVICE)
        cls.positions = {}
        cls.node.create_subscription(
            JointState, JOINT_STATES_TOPIC, cls._on_joint_state,
            qos_profile_sensor_data)

    @classmethod
    def tearDownClass(cls):
        """Release ROS entities after the launch test."""
        cls.client.destroy()
        cls.list_controllers.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_joint_state(cls, message):
        cls.positions.update(zip(message.name, message.position))

    def _spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail('condition was not reached before timeout')

    def _wait_for_inactive_realtime_controller(self):
        self.assertTrue(
            self.list_controllers.wait_for_service(timeout_sec=30.0))
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            future = self.list_controllers.call_async(
                ListControllers.Request())
            rclpy.spin_until_future_complete(
                self.node, future, timeout_sec=2.0)
            if future.done() and any(
                    controller.name == 'realtime_controller' and
                    controller.state == 'inactive'
                    for controller in future.result().controller):
                time.sleep(0.2)
                return
        self.fail('realtime controller did not finish configuring')

    def test_task_aperture_drives_one_jaw_travel(self):
        """Map a 70 mm aperture through the 2FG14 geometry zero."""
        self.assertTrue(self.client.wait_for_server(timeout_sec=30.0))
        self._spin_until(
            lambda: 'grip_stroke' in self.positions and
            'finger_stroke' in self.positions,
            30.0)
        goal = ParallelGripperCommand.Goal()
        goal.command.name = ['grip_stroke']
        goal.command.position = [0.070]
        goal.command.effort = [40.0]
        goal_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self.node, goal_future, timeout_sec=5.0)
        self.assertTrue(goal_future.done())
        handle = goal_future.result()
        self.assertTrue(handle.accepted)

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node, result_future, timeout_sec=5.0)
        self.assertTrue(result_future.done())
        self.assertEqual(result_future.result().status,
                         GoalStatus.STATUS_SUCCEEDED)

        self._spin_until(
            lambda: abs(self.positions['grip_stroke'] - 0.070) < 0.001,
            3.0)
        self.assertAlmostEqual(
            self.positions['grip_stroke'], 0.070, delta=0.001)
        self.assertAlmostEqual(
            # The 2FG14 visual joint is zeroed at its 55 mm closed-geometry
            # aperture: (70 - 55) / 2 = 7.5 mm per jaw.
            self.positions['finger_stroke'], 0.0075, delta=0.0005)
        self._wait_for_inactive_realtime_controller()


@launch_testing.post_shutdown_test()
class ProcessesExitCleanly(unittest.TestCase):
    """Check that launch-managed processes accept normal test shutdown."""

    def test_exit_codes(self, proc_info):
        """Require clean or launch-approved signal exit codes."""
        launch_testing.asserts.assertExitCodes(proc_info)

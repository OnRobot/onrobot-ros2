"""Exercise the headless joint-state validity filter and its ROS contract."""

import math
import time
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node

import launch_testing
from launch_testing.actions import ReadyToTest

import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


TEST_NAMESPACE = 'joint_state_effort_filter_test'
INPUT_TOPIC = f'/{TEST_NAMESPACE}/joint_state_broadcaster/joint_states'
OUTPUT_TOPIC = f'/{TEST_NAMESPACE}/joint_states'
ROBOT = '''<robot name="finite_feedback">
  <link name="base_link"/><link name="right_finger_base_link"/>
  <link name="left_finger_base_link"/><link name="task_aperture_link"/>
  <joint name="finger_stroke" type="prismatic">
    <parent link="base_link"/><child link="right_finger_base_link"/>
    <axis xyz="0 1 0"/><limit lower="0" upper="0.02" effort="140" velocity="0.1"/>
  </joint>
  <joint name="left_finger" type="prismatic">
    <parent link="base_link"/><child link="left_finger_base_link"/>
    <axis xyz="0 -1 0"/><limit lower="0" upper="0.02" effort="140" velocity="0.1"/>
    <mimic joint="finger_stroke" multiplier="1"/>
  </joint>
  <joint name="grip_stroke" type="prismatic">
    <parent link="base_link"/><child link="task_aperture_link"/>
    <axis xyz="1 0 0"/><limit lower="0" upper="0.1" effort="140" velocity="0.1"/>
  </joint>
</robot>'''


def generate_test_description():
    """Run one namespaced cleaner without starting any GUI process."""
    return LaunchDescription([
        Node(
            package='onrobot_gripper_hardware',
            executable='joint_state_effort_filter',
            namespace=TEST_NAMESPACE,
            output='screen',
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=TEST_NAMESPACE,
            parameters=[{'robot_description': ROBOT, 'ignore_timestamp': True}],
            remappings=[('/tf', f'/{TEST_NAMESPACE}/tf')], output='screen'),
        ReadyToTest(),
    ])


class JointStateEffortFilter(unittest.TestCase):
    """Verify filtering, optional arrays, QoS, and namespace resolution."""

    sample_sequence = 0

    @classmethod
    def setUpClass(cls):
        """Create an independent publisher and output observer."""
        rclpy.init()
        cls.node = rclpy.create_node('joint_state_effort_filter_test')
        cls.messages = []
        cls.transforms = []
        cls.publisher = cls.node.create_publisher(
            JointState, INPUT_TOPIC, qos_profile_sensor_data)
        cls.node.create_subscription(
            JointState, OUTPUT_TOPIC, cls.messages.append,
            qos_profile_sensor_data)
        cls.node.create_subscription(
            TFMessage, f'/{TEST_NAMESPACE}/tf',
            lambda msg: cls.transforms.extend(msg.transforms), 100)

    @classmethod
    def tearDownClass(cls):
        """Release ROS entities after the launch test."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail('condition was not reached before timeout')

    def _publish_and_wait(self, message):
        self._spin_until(lambda: self.publisher.get_subscription_count() > 0)
        type(self).sample_sequence += 1
        message.header.stamp.sec = 1_234_567_890
        message.header.stamp.nanosec = type(self).sample_sequence
        expected_stamp = (
            message.header.stamp.sec,
            message.header.stamp.nanosec,
        )

        # Both sides intentionally use best-effort sensor-data QoS. Endpoint
        # discovery can therefore be visible to this publisher before the
        # filter's output publisher is ready to deliver its first sample. Keep
        # the existing bound, but repeat this exact identified sample until its
        # matching output arrives instead of treating one discovery-time loss
        # as a filter failure.
        deadline = time.monotonic() + 5.0
        next_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self.publisher.publish(message)
                next_publish = now + 0.05
            rclpy.spin_once(self.node, timeout_sec=0.05)
            for candidate in reversed(self.messages):
                candidate_stamp = (
                    candidate.header.stamp.sec,
                    candidate.header.stamp.nanosec,
                )
                if candidate_stamp == expected_stamp:
                    return candidate
        self.fail(
            f'sample {expected_stamp} was not forwarded before timeout')

    def test_filter_preserves_valid_optional_arrays_and_clears_invalid_fields(
            self):
        """Keep finite values while removing unavailable velocity and effort."""
        self._spin_until(lambda: bool(
            self.node.get_publishers_info_by_topic(OUTPUT_TOPIC)))
        publisher_info = self.node.get_publishers_info_by_topic(OUTPUT_TOPIC)
        self.assertEqual(len(publisher_info), 1)
        self.assertEqual(
            publisher_info[0].qos_profile.reliability,
            ReliabilityPolicy.BEST_EFFORT,
        )
        self.assertEqual(
            publisher_info[0].qos_profile.durability,
            DurabilityPolicy.VOLATILE,
        )

        valid = JointState()
        valid.name = ['joint_a', 'joint_b']
        valid.position = [0.1, 0.2]
        valid.velocity = []
        valid.effort = [3.0, 4.0]
        result = self._publish_and_wait(valid)
        self.assertEqual(list(result.name), valid.name)
        self.assertEqual(list(result.position), list(valid.position))
        self.assertEqual(list(result.velocity), [])
        self.assertEqual(list(result.effort), list(valid.effort))

        unavailable = JointState()
        unavailable.name = valid.name
        unavailable.position = valid.position
        unavailable.velocity = [math.nan, 0.1]
        unavailable.effort = [0.2, math.nan]
        result = self._publish_and_wait(unavailable)
        self.assertEqual(list(result.name), valid.name)
        self.assertEqual(list(result.position), list(valid.position))
        self.assertEqual(list(result.velocity), [])
        self.assertEqual(list(result.effort), [])

    def test_missing_position_is_omitted_without_replacing_it_or_other_joints(self):
        message = JointState()
        message.header.frame_id = 'base_link'
        message.name = ['grip_stroke', 'finger_stroke', 'missing']
        message.position = [math.nan, .012, math.inf]
        message.velocity = [.2, .3, .4]
        message.effort = [1., 2., 3.]
        result = self._publish_and_wait(message)
        self.assertEqual(result.name, ['finger_stroke'])
        self.assertEqual(list(result.position), [.012])
        self.assertEqual(list(result.velocity), [.3])
        self.assertEqual(list(result.effort), [2.])
        self.assertEqual(result.header.frame_id, 'base_link')

    def test_fault_does_not_publish_an_empty_or_nan_joint_state(self):
        """All-invalid/fault data must not reach the TF consumer at all."""
        self._spin_until(lambda: self.publisher.get_subscription_count() > 0)
        bad = JointState()
        bad.header.stamp.sec = 7654321
        bad.name = ['grip_stroke', 'finger_stroke']
        bad.position = [math.nan, math.inf]
        self.messages.clear()
        for _ in range(10):
            self.publisher.publish(bad)
            rclpy.spin_once(self.node, timeout_sec=.05)
        self.assertFalse(any(m.header.stamp.sec == 7654321 for m in self.messages))

    def test_malformed_arrays_are_rejected_before_optional_field_cleanup(self):
        """A NaN in a short optional array must not hide the size mismatch."""
        self._spin_until(lambda: self.publisher.get_subscription_count() > 0)
        for field in ['position', 'velocity', 'effort']:
            with self.subTest(field=field):
                bad = JointState()
                bad.header.stamp.sec = 7654322
                bad.name = ['grip_stroke', 'finger_stroke']
                bad.position = [.05, .0085]
                setattr(bad, field, [math.nan])
                self.messages.clear()
                for _ in range(10):
                    self.publisher.publish(bad)
                    rclpy.spin_once(self.node, timeout_sec=.05)
                self.assertFalse(any(m.header.stamp.sec == 7654322
                                     for m in self.messages))

    def test_actual_tf_consumer_survives_invalid_then_recovers(self):
        """Exercise physical, mimic and task frames, not just filter output."""
        self.transforms.clear()
        for q in [.008, .012]:
            good = JointState()
            good.name = ['grip_stroke', 'finger_stroke']
            good.position = [.033 + 2*q, q]
            self._publish_and_wait(good)
            self._spin_until(lambda: any(
                t.child_frame_id == 'right_finger_base_link' and
                abs(t.transform.translation.y - q) < 1e-9 for t in self.transforms))
            invalid = JointState()
            invalid.name = good.name
            invalid.position = [math.nan, q]  # physical visualization only
            self._publish_and_wait(invalid)
        self.assertTrue(any(t.child_frame_id == 'left_finger_base_link'
                            for t in self.transforms))
        self.assertTrue(any(t.child_frame_id == 'task_aperture_link'
                            for t in self.transforms))
        for t in self.transforms:
            v, r = t.transform.translation, t.transform.rotation
            self.assertTrue(all(math.isfinite(x) for x in
                                [v.x, v.y, v.z, r.x, r.y, r.z, r.w]))


@launch_testing.post_shutdown_test()
class ProcessesExitCleanly(unittest.TestCase):
    """Check that launch-managed processes accept normal test shutdown."""

    def test_exit_codes(self, proc_info):
        """Require clean or launch-approved signal exit codes."""
        launch_testing.asserts.assertExitCodes(proc_info)

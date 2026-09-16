"""Exercise the installed HIL preflight against normal fake bringup."""

import json
import os
from pathlib import Path
import unittest

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_testing
from launch_testing.actions import ReadyToTest


STOP_REPORT = Path('/tmp') / (
    f'onrobot_2fg7_fake_stop_preflight_{os.getpid()}.json')
CANCEL_REPORT = Path('/tmp') / (
    f'onrobot_2fg7_fake_cancel_qualification_{os.getpid()}.json')
PREEMPT_REPORT = Path('/tmp') / (
    f'onrobot_2fg7_fake_preemption_qualification_{os.getpid()}.json')


def generate_test_description():
    """Start isolated conventional and realtime 2FG7 preflights."""
    source = PythonLaunchDescriptionSource([
        get_package_share_directory('onrobot_gripper_bringup'),
        '/launch/gripper.launch.py',
    ])
    conventional_bringup = IncludeLaunchDescription(
        source, launch_arguments={
            'model': '2fg7',
            'backend': 'fake',
            'start_realtime_controller': 'false',
            'namespace': 'hil_conventional',
            'frame_prefix': 'hil_conventional/',
            'fake_motion_speed_m_s': '0.01',
        }.items())
    realtime_bringup = IncludeLaunchDescription(
        source, launch_arguments={
            'model': '2fg7',
            'backend': 'fake',
            'start_realtime_controller': 'true',
            'namespace': 'hil_realtime',
            'frame_prefix': 'hil_realtime/',
        }.items())
    conventional_preflight = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'onrobot_gripper_isaac',
            'run_hardware_in_loop.py', '--model', '2fg7',
            '--namespace', 'hil_conventional',
            '--ros-preflight-only', '--startup-timeout', '45',
        ],
        name='hil_conventional_preflight',
        output='screen',
    )
    realtime_preflight = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'onrobot_gripper_isaac',
            'run_hardware_in_loop.py', '--model', '2fg7',
            '--namespace', 'hil_realtime',
            '--ros-preflight-only', '--hardware-control',
            'realtime-position', '--startup-timeout', '45',
            '--verify-realtime-stop', '--confirm-model', '2fg7',
            '--output', str(STOP_REPORT),
        ],
        name='hil_realtime_preflight',
        output='screen',
    )
    cancel_qualification = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'onrobot_gripper_isaac',
            'run_hardware_in_loop.py', '--model', '2fg7',
            '--namespace', 'hil_conventional',
            '--verify-conventional-cancel', '--confirm-model', '2fg7',
            '--travel', '0.010', '--cancel-after-motion', '0.001',
            '--startup-timeout', '45', '--output', str(CANCEL_REPORT),
        ],
        name='hil_conventional_cancel_qualification',
        output='screen',
    )
    preemption_qualification = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'onrobot_gripper_isaac',
            'run_hardware_in_loop.py', '--model', '2fg7',
            '--namespace', 'hil_conventional',
            '--verify-conventional-preemption', '--confirm-model', '2fg7',
            '--travel', '0.010', '--cancel-after-motion', '0.001',
            '--startup-timeout', '45', '--output', str(PREEMPT_REPORT),
        ],
        name='hil_conventional_preemption_qualification',
        output='screen',
    )
    start_preemption_after_cancel = RegisterEventHandler(OnProcessExit(
        target_action=cancel_qualification,
        on_exit=[preemption_qualification],
    ))
    return (
        LaunchDescription([
            conventional_bringup,
            realtime_bringup,
            conventional_preflight,
            realtime_preflight,
            cancel_qualification,
            start_preemption_after_cancel,
            ReadyToTest(),
        ]),
        {
            'conventional_preflight': conventional_preflight,
            'realtime_preflight': realtime_preflight,
            'cancel_qualification': cancel_qualification,
            'preemption_qualification': preemption_qualification,
        },
    )


class HilPreflightIntegration(unittest.TestCase):
    """Verify the installed command observes the standard ROS surface."""

    def _assert_preflight(
            self, proc_info, proc_output, preflight, command_authority):
        """Require a passing no-authority report and zero exit status."""
        proc_info.assertWaitForShutdown(preflight, timeout=60)
        proc_output.assertWaitFor(
            f'"command_authority": "{command_authority}"',
            process=preflight,
            stream='stdout',
            timeout=5,
        )
        proc_output.assertWaitFor(
            '"status": "passed"',
            process=preflight,
            stream='stdout',
            timeout=5,
        )
        launch_testing.asserts.assertExitCodes(
            proc_info, process=preflight)

    def test_conventional_preflight_passes_without_motion(
            self, proc_info, proc_output, conventional_preflight):
        """Require the standard action path without sending a goal."""
        self._assert_preflight(
            proc_info, proc_output, conventional_preflight,
            'none-preflight')

    def test_realtime_preflight_passes_without_motion(
            self, proc_info, proc_output, realtime_preflight):
        """Require realtime command, state, and Stop without a command."""
        self._assert_preflight(
            proc_info, proc_output, realtime_preflight,
            'ros2-control-stop-only')
        proc_output.assertWaitFor(
            '"name": "realtime_stop"',
            process=realtime_preflight,
            stream='stdout',
            timeout=5,
        )
        report = json.loads(STOP_REPORT.read_text(encoding='utf-8'))
        assert report['status'] == 'passed'
        assert report['command_authority'] == 'ros2-control-stop-only'
        assert report['hardware_motion_enabled'] is False
        assert report['acceptance_limits']['realtime_stop_timeout_s'] == 3.0
        stop_test = next(
            test for test in report['tests']
            if test['name'] == 'realtime_stop')
        assert stop_test['newer_idle_state_observed'] is True

    def test_conventional_cancel_stops_and_restores(
            self, proc_info, proc_output, cancel_qualification):
        """Require standard-action cancellation and bounded restoration."""
        self._assert_preflight(
            proc_info, proc_output, cancel_qualification,
            'ros2-control-standard-action-bounded-cancel')
        report = json.loads(CANCEL_REPORT.read_text(encoding='utf-8'))
        assert report['status'] == 'passed'
        assert report['hardware_motion_enabled'] is True
        cancel_test = next(
            test for test in report['tests']
            if test['name'] == 'conventional_cancel_stop')
        assert cancel_test['cancelled_result_observed'] is True
        assert cancel_test['newer_idle_state_observed'] is True
        assert cancel_test['observed_excursion_m'] <= 0.011
        assert cancel_test['remaining_distance_to_target_m'] >= 0.001
        assert any(
            test['name'] == 'restore_after_conventional_cancel'
            for test in report['tests'])

    def test_conventional_preemption_stops_and_replaces(
            self, proc_info, proc_output, preemption_qualification):
        """Require bounded replacement-goal preemption and restoration."""
        self._assert_preflight(
            proc_info, proc_output, preemption_qualification,
            'ros2-control-standard-action-bounded-preemption')
        report = json.loads(PREEMPT_REPORT.read_text(encoding='utf-8'))
        assert report['status'] == 'passed'
        assert report['hardware_motion_enabled'] is True
        preemption_test = next(
            test for test in report['tests']
            if test['name'] == 'conventional_preemption_stop')
        assert preemption_test['superseded_result_cancelled'] is True
        assert preemption_test['replacement_result_succeeded'] is True
        assert preemption_test['newer_idle_state_observed'] is True
        assert preemption_test[
            'peak_progress_toward_superseded_target_m'] <= 0.011
        assert preemption_test[
            'remaining_distance_to_superseded_target_m'] >= 0.001


@launch_testing.post_shutdown_test()
class ProcessesExitCleanly(unittest.TestCase):
    """Check that fake bringup accepts normal test shutdown."""

    def test_exit_codes(self, proc_info):
        """Require clean or launch-approved signal exit codes."""
        launch_testing.asserts.assertExitCodes(proc_info)

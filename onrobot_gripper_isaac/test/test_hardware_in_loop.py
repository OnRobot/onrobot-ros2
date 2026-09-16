"""Pure ROS-side tests for the 2FG hardware-in-the-loop runner."""

import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace

from control_msgs.msg import Float64Values
from control_msgs.msg import Keys

from onrobot_gripper_msgs.msg import GripperState
from onrobot_gripper_msgs.msg import RealtimeState

import pytest

from sensor_msgs.msg import JointState

from std_srvs.srv import Trigger


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _load_hil_module():
    scripts = PACKAGE_ROOT / 'scripts'
    sys.path.insert(0, str(scripts))
    try:
        spec = importlib.util.spec_from_file_location(
            'onrobot_hardware_in_loop', scripts / 'run_hardware_in_loop.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(scripts))


def _observation(module):
    observation = module.HardwareObservation.__new__(
        module.HardwareObservation)
    observation.task_joint = 'grip_stroke'
    observation.physical_joint = 'finger_stroke'
    observation.expected_model = '2fg7'
    observation.joint_position = math.nan
    observation.joint_velocity = 0.0
    observation.joint_received_at = 0.0
    observation.joint_samples = 0
    observation.state = None
    observation.state_received_at = 0.0
    observation.timing_metrics_enabled = False
    observation.timing_samples = 0
    observation.maximum_reported_sample_age = 0.0
    observation.maximum_hardware_cycle_duration = 0.0
    observation.maximum_typed_state_interval = 0.0
    observation.previous_timing_state_received_at = 0.0
    observation.model_mismatch = None
    observation.realtime_state = None
    observation.realtime_state_received_at = 0.0
    observation.limit_keys = []
    observation.limit_values = []
    observation.task_limits = None
    return observation


def test_module_loads_without_importing_isaac_sim():
    """The hardware preflight path must work in a normal ROS environment."""
    prior_modules = set(sys.modules)
    _load_hil_module()
    newly_loaded = set(sys.modules) - prior_modules
    assert not any(name == 'isaacsim' or name.startswith('isaacsim.')
                   for name in newly_loaded)


def test_rg_ros_preflight_uses_physical_joint_without_asset_contract(
        monkeypatch):
    """ROS-only RG qualification must not require an unavailable USD asset."""
    module = _load_hil_module()
    captured = {}

    def run_ros(args, report):
        captured['physical_joint'] = args.physical_joint
        captured['model'] = report['model']
        return 0

    monkeypatch.setattr(module, '_run_ros_qualification', run_ros)
    monkeypatch.setattr(
        sys, 'argv', [
            'run_hardware_in_loop.py', '--model', 'rg2',
            '--ros-preflight-only'])

    assert module.main() == 0
    assert captured == {
        'physical_joint': 'finger_joint',
        'model': 'rg2',
    }


def test_2fg_realtime_position_qualification_is_ros_only_and_explicit(
        monkeypatch, tmp_path):
    """The command-6 proof must bypass Isaac and retain bounded authority."""
    module = _load_hil_module()
    captured = {}

    def run_ros(args, report):
        captured['args'] = args
        captured['report'] = report
        return 0

    monkeypatch.setattr(module, '_run_ros_qualification', run_ros)
    monkeypatch.setattr(
        sys, 'argv', [
            'run_hardware_in_loop.py',
            '--model', '2fg14',
            '--verify-realtime-position',
            '--hardware-control', 'realtime-position',
            '--confirm-model', '2fg14',
            '--travel', '0.004',
            '--output', str(tmp_path / 'result.json'),
        ])

    assert module.main() == 0
    assert captured['args'].physical_joint == 'finger_stroke'
    assert captured['report']['mode'] == (
        'ros-realtime-position-qualification')
    assert captured['report']['command_authority'] == (
        'ros2-control-typed-realtime-bounded-position')
    assert captured['report']['hardware_motion_enabled']


def test_ros_entry_point_uses_the_jazzy_system_python():
    """The documented ros2 run preflight must not select workspace Python."""
    script = PACKAGE_ROOT / 'scripts' / 'run_hardware_in_loop.py'
    assert script.read_text(encoding='utf-8').splitlines()[0] == (
        '#!/usr/bin/python3')


def test_qualification_input_digest_is_stable(tmp_path):
    """Retained HIL reports must identify exact runner and asset inputs."""
    module = _load_hil_module()
    input_file = tmp_path / 'input.usda'
    input_file.write_bytes(b'#usda 1.0\n')
    assert module._sha256(input_file) == (
        '28f84f705dab5dc6d5baaee8ee94e59b93b71bcd708c048c71066e7a8956e2d1')
    payload = tmp_path / 'payloads' / 'physics.usda'
    payload.parent.mkdir()
    payload.write_bytes(b'#usda 1.0\n')
    assert module._asset_manifest(input_file) == {
        'input.usda': module._sha256(input_file),
        'payloads/physics.usda': module._sha256(payload),
    }


def test_acceptance_limits_capture_only_the_runtime_gates_that_apply():
    """A report must retain its gates without implying unused thresholds."""
    module = _load_hil_module()
    args = SimpleNamespace(
        startup_timeout=30.0,
        model='2fg7',
        state_timeout=0.5,
        duration=60.0,
        physics_dt=1.0 / 120.0,
        shadow_tolerance=0.001,
        travel=0.010,
        effort=10.0,
        motion_timeout=20.0,
        ros_preflight_only=True,
        enable_hardware_motion=False,
        hardware_control='realtime-position',
        verify_realtime_stop=False,
        verify_realtime_position=False,
        verify_conventional_completion=False,
        verify_conventional_cancel=False,
        verify_conventional_preemption=False,
        stop_timeout=3.0,
        cancel_after_motion=0.001,
    )

    assert module._acceptance_limits(args) == {
        'startup_timeout_s': 30.0,
        'state_freshness_timeout_s': 0.5,
        'monitoring_duration_s': 60.0,
        'adverse_counter_change_allowed': False,
    }

    args.ros_preflight_only = False
    args.enable_hardware_motion = True
    args.hardware_control = 'conventional'
    assert module._acceptance_limits(args) == {
        'startup_timeout_s': 30.0,
        'state_freshness_timeout_s': 0.5,
        'monitoring_duration_s': 60.0,
        'physics_step_s': 1.0 / 120.0,
        'physical_coordinate_shadow_tolerance': 0.001,
        'maximum_external_aperture_excursion_m': 0.010,
        'maximum_conventional_effort_n': 10.0,
        'motion_timeout_s': 20.0,
        'adverse_counter_change_allowed': False,
    }

    args.enable_hardware_motion = False
    args.verify_realtime_stop = True
    args.ros_preflight_only = True
    assert module._acceptance_limits(args) == {
        'startup_timeout_s': 30.0,
        'state_freshness_timeout_s': 0.5,
        'monitoring_duration_s': 60.0,
        'realtime_stop_timeout_s': 3.0,
        'adverse_counter_change_allowed': False,
    }

    args.verify_realtime_stop = False
    args.verify_conventional_completion = True
    args.hardware_control = 'conventional'
    assert module._acceptance_limits(args) == {
        'startup_timeout_s': 30.0,
        'state_freshness_timeout_s': 0.5,
        'monitoring_duration_s': 60.0,
        'maximum_external_aperture_excursion_m': 0.010,
        'maximum_conventional_effort_n': 10.0,
        'motion_timeout_s': 20.0,
        'adverse_counter_change_allowed': False,
    }

    args.verify_conventional_completion = False
    args.verify_conventional_cancel = True
    args.hardware_control = 'conventional'
    assert module._acceptance_limits(args) == {
        'startup_timeout_s': 30.0,
        'state_freshness_timeout_s': 0.5,
        'monitoring_duration_s': 60.0,
        'maximum_external_aperture_excursion_m': 0.010,
        'maximum_conventional_effort_n': 10.0,
        'motion_timeout_s': 20.0,
        'minimum_motion_before_cancel_m': 0.001,
        'adverse_counter_change_allowed': False,
    }

    args.verify_conventional_cancel = False
    args.verify_conventional_preemption = True
    assert module._acceptance_limits(args) == {
        'startup_timeout_s': 30.0,
        'state_freshness_timeout_s': 0.5,
        'monitoring_duration_s': 60.0,
        'maximum_external_aperture_excursion_m': 0.010,
        'maximum_conventional_effort_n': 10.0,
        'motion_timeout_s': 20.0,
        'minimum_motion_before_preemption_m': 0.001,
        'adverse_counter_change_allowed': False,
    }

    args.verify_conventional_preemption = False
    args.verify_realtime_position = True
    args.hardware_control = 'realtime-position'
    args.travel = 0.004
    assert module._acceptance_limits(args) == {
        'startup_timeout_s': 30.0,
        'state_freshness_timeout_s': 0.5,
        'monitoring_duration_s': 60.0,
        'maximum_external_aperture_excursion_m': 0.004,
        'motion_timeout_s': 20.0,
        'realtime_stop_timeout_s': 3.0,
        'realtime_command_refresh_period_s': 0.02,
        'maximum_opposed_motion_m': 0.0005,
        'task_feedback_agreement_tolerance_m': 0.0005,
        'task_to_physical_mapping_tolerance': 0.002,
        'maximum_start_distance_from_open_endpoint_m': 0.006,
        'adverse_counter_change_allowed': False,
    }


def test_realtime_hardware_motion_is_enabled_only_for_qualified_firmware():
    """Only models with physical protocol evidence may issue HIL motion."""
    module = _load_hil_module()
    assert module.SUPPORTED_HIL_MODELS == (
        '2fg7', '2fg14', 'rg2', 'rg6')
    assert module.ISAAC_HIL_MODELS == ('2fg7', '2fg14', 'rg2', 'rg6')
    assert module.DEFAULT_PHYSICAL_JOINTS['rg2'] == 'finger_joint'
    assert module.REALTIME_POSITION_MOTION_QUALIFIED_MODELS == (
        '2fg7', '2fg14', 'rg2', 'rg6')
    assert module._realtime_position_qualification_reason('2fg7') is None
    assert module._realtime_position_qualification_reason('2fg14') is None
    assert module._realtime_position_qualification_reason('rg2') is None
    assert module._realtime_position_qualification_reason('rg6') is None


def test_2fg14_hil_uses_its_physical_joint_and_checked_in_asset_contract():
    """2FG14 HIL must not silently inherit an RG or 2FG7 asset selection."""
    module = _load_hil_module()

    assert module.DEFAULT_PHYSICAL_JOINTS['2fg14'] == 'finger_stroke'
    assert '2fg14' in module.ISAAC_HIL_MODELS
    assert module.default_asset('2fg14').as_posix().endswith(
        '/assets/2fg14/onrobot_2fg14.usda')
    contract = module.load_contract('2fg14')
    assert module.driven_joint(contract) == 'finger_stroke'
    assert float(contract['usd']['lower_limit_m']) == pytest.approx(0.0)
    assert float(contract['usd']['upper_limit_m']) > 0.0


def test_ready_record_exposes_realtime_motion_qualification():
    """A ready controller must not be confused with qualified firmware."""
    module = _load_hil_module()
    observation = _observation(module)
    observation.joint_position = 0.004
    observation.task_limits = (0.033, 0.071)
    observation.state = GripperState(
        model='2fg7',
        firmware='1.2.3',
        device_profile_revision='2fg-guide-current',
        finger_profile_name='2fg7_standard',
        finger_profile_revision='1',
        task_aperture=0.041,
        task_aperture_valid=True,
        connection_state=GripperState.CONNECTION_IDLE,
        firmware_qualification=GripperState.FIRMWARE_UNKNOWN,
        mapping_validity=GripperState.MAPPING_VALID,
        mapping_source=GripperState.MAPPING_SOURCE_DEVICE_MEASURED,
        sample_sequence=42,
        successful_cycles=100,
        failed_cycles=2,
        missed_deadlines=4,
        watchdog_stops=3,
        reconnects=1,
        last_cycle_duration=0.0015,
    )
    observation.action = SimpleNamespace(server_is_ready=lambda: True)
    observation.realtime_ready = lambda: True
    args = SimpleNamespace(model='2fg7', namespace='hil')

    ready = module._ready_payload(
        args, {'mode': 'ros-preflight'}, observation)

    assert ready['realtime_controller_ready']
    assert ready['realtime_position_motion_qualified']
    assert ready['realtime_position_qualification_reason'] is None
    assert ready['reported_firmware'] == '1.2.3'
    assert ready['firmware_qualification'] == 'unknown'
    assert ready['device_profile_revision'] == '2fg-guide-current'
    assert ready['finger_profile_name'] == '2fg7_standard'
    assert ready['finger_profile_revision'] == '1'
    assert ready['mapping_validity'] == GripperState.MAPPING_VALID
    assert ready['mapping_source'] == (
        GripperState.MAPPING_SOURCE_DEVICE_MEASURED)
    assert ready['sample_sequence'] == 42
    assert ready['successful_hardware_cycles'] == 100
    assert ready['failed_hardware_cycles'] == 2
    assert ready['missed_deadlines'] == 4
    assert ready['watchdog_stops'] == 3
    assert ready['reconnects'] == 1
    assert ready['reported_sample_age_s'] == 0.0
    assert ready['last_hardware_cycle_duration_s'] == 0.0015


def test_ready_record_uses_rg_firmware_evidence_gate():
    """RG reports must not repeat the 2FG command-feedback observation."""
    module = _load_hil_module()
    observation = _observation(module)
    observation.joint_position = 1.0
    observation.task_limits = (0.0, 0.160)
    observation.state = GripperState(
        model='rg6',
        task_aperture=0.100,
        task_aperture_valid=True,
        firmware_qualification=GripperState.FIRMWARE_UNKNOWN,
    )
    observation.action = SimpleNamespace(server_is_ready=lambda: True)
    observation.realtime_ready = lambda: True
    args = SimpleNamespace(model='rg6', namespace='hil')

    ready = module._ready_payload(
        args, {'mode': 'ros-preflight'}, observation)

    reason = ready['realtime_position_qualification_reason']
    assert 'approved range' in reason
    assert 'command-6' not in reason
    assert 'raw mechanism' not in reason

    observation.state.firmware_qualification = GripperState.FIRMWARE_QUALIFIED
    ready = module._ready_payload(
        args, {'mode': 'ros-preflight'}, observation)
    assert ready['realtime_position_motion_qualified'] is True
    assert ready['realtime_position_qualification_reason'] is None


def test_joint_callback_selects_only_finite_physical_state():
    """Task aperture and invalid samples must not become USD joint state."""
    module = _load_hil_module()
    observation = _observation(module)

    observation._receive_joint_state(JointState(
        name=['grip_stroke'], position=[0.05]))
    assert observation.joint_samples == 0

    observation._receive_joint_state(JointState(
        name=['finger_stroke'], position=[math.nan]))
    assert observation.joint_samples == 0

    observation._receive_joint_state(JointState(
        name=['grip_stroke', 'finger_stroke'],
        position=[0.05, 0.004], velocity=[0.01, math.nan]))
    assert observation.joint_samples == 1
    assert observation.joint_position == 0.004
    assert observation.joint_velocity == 0.0
    assert observation.joint_received_at > 0.0


def test_shadow_updates_drive_target_before_teleporting_measured_state():
    """A stale PhysX drive target must not pull a mirrored joint away."""
    module = _load_hil_module()
    calls = []

    class Articulation:
        def set_dof_position_targets(self, position, dof_indices):
            calls.append(('target', position, dof_indices))

        def set_dof_positions(self, position, dof_indices):
            calls.append(('position', position, dof_indices))

        def set_dof_velocities(self, velocity, dof_indices):
            calls.append(('velocity', velocity, dof_indices))

    module._mirror_measured_position(
        Articulation(), 3, {3: 0.012, 4: 0.012})

    assert calls == [
        ('target', 0.012, [3]),
        ('position', [0.012, 0.012], [3, 4]),
        ('velocity', [0.0, 0.0], [3, 4]),
    ]


def test_shadow_without_mimic_updates_only_the_driven_joint():
    """Assets without a follower still use the measured-state helper."""
    module = _load_hil_module()
    calls = []

    class Articulation:
        def set_dof_position_targets(self, position, dof_indices):
            calls.append(('target', position, dof_indices))

        def set_dof_positions(self, position, dof_indices):
            calls.append(('position', position, dof_indices))

        def set_dof_velocities(self, velocity, dof_indices):
            calls.append(('velocity', velocity, dof_indices))

    module._mirror_measured_position(Articulation(), 3, {3: 0.012})

    assert calls == [
        ('target', 0.012, [3]),
        ('position', [0.012], [3]),
        ('velocity', [0.0], [3]),
    ]


def test_2fg_shadow_preserves_equal_follower_coordinate():
    """The existing 2FG follower remains equal to its physical leader."""
    module = _load_hil_module()
    contract = {
        'usd': {
            'follower_joint': 'left_finger_stroke',
        },
    }

    mirrored = module._shadow_dof_positions(
        contract,
        ['finger_stroke', 'left_finger_stroke'],
        [0.002, 0.002],
        0,
        0.012,
    )

    assert mirrored == {0: 0.012, 1: 0.012}


def test_rg_shadow_maps_every_mimic_from_one_reference_pose():
    """RG shadow teleportation must preserve all PhysX mimic constraints."""
    module = _load_hil_module()
    contract = {
        'usd': {
            'mimic_joints': [
                {'name': 'same', 'gearing': -1.0},
                {'name': 'opposite', 'gearing': 1.0},
            ],
        },
    }
    mirrored = module._shadow_dof_positions(
        contract,
        ['leader', 'same', 'opposite'],
        [0.5, 0.25, -0.25],
        0,
        0.7,
    )

    assert mirrored == pytest.approx({
        0: 0.7,
        1: 0.45,
        2: -0.45,
    })


def test_rg_shadow_rejects_missing_contract_mimic():
    """A partial RG linkage must fail instead of producing a false pass."""
    module = _load_hil_module()
    contract = {
        'usd': {
            'mimic_joints': [
                {'name': 'missing', 'gearing': -1.0},
            ],
        },
    }

    with pytest.raises(RuntimeError, match='missing.*missing'):
        module._shadow_dof_positions(
            contract, ['leader'], [0.5], 0, 0.7)


def test_successful_action_waits_for_fresh_typed_target_state():
    """Action success must not expose the previous typed-state sample."""
    module = _load_hil_module()
    observation = _observation(module)
    observation.state_received_at = 1.0
    observation.state = GripperState(
        model='2fg7', task_aperture=0.061,
        task_aperture_valid=True)
    result = SimpleNamespace(reached_goal=True, stalled=False)
    wrapped = SimpleNamespace(
        status=module.GoalStatus.STATUS_SUCCEEDED, result=result)
    result_future = SimpleNamespace(
        done=lambda: True, result=lambda: wrapped)
    observation._start_goal = lambda *args: (object(), result_future)
    pump_calls = []

    def pump():
        pump_calls.append(True)
        observation.state_received_at = 2.0
        observation.state.task_aperture = 0.071

    returned = observation.send_goal(0.071, 10.0, pump, 0.5)

    assert returned is result
    assert len(pump_calls) == 1


def test_limit_callbacks_require_matching_valid_task_bounds():
    """Latched names and values may arrive in either order but must align."""
    module = _load_hil_module()
    observation = _observation(module)

    observation._receive_limit_values(Float64Values(values=[0.033, 0.071]))
    assert observation.task_limits is None
    observation._receive_limit_keys(Keys(keys=[
        'grip_stroke/minimum_task_aperture',
        'grip_stroke/maximum_task_aperture',
    ]))
    assert observation.task_limits == (0.033, 0.071)

    observation._receive_limit_values(Float64Values(values=[0.08, 0.07]))
    assert observation.task_limits == (0.033, 0.071)


def test_model_mismatch_is_latched_without_replacing_valid_state():
    """A state from another model must stop readiness deterministically."""
    module = _load_hil_module()
    observation = _observation(module)
    valid = GripperState(model='2fg7', task_aperture_valid=True)
    observation._receive_gripper_state(valid)
    assert observation.state is valid

    observation._receive_gripper_state(GripperState(model='2fg14'))
    assert observation.model_mismatch == '2fg14'
    assert observation.state is valid


def test_post_readiness_timing_metrics_capture_worst_observed_values():
    """Reports must retain worst timing, not only the readiness snapshot."""
    module = _load_hil_module()
    observation = _observation(module)
    observation.reset_timing_metrics()

    first = GripperState(model='2fg7', last_cycle_duration=0.004)
    first.sample_age.nanosec = 12_000_000
    observation._receive_gripper_state(first)
    second = GripperState(model='2fg7', last_cycle_duration=0.009)
    second.sample_age.nanosec = 28_000_000
    observation._receive_gripper_state(second)

    metrics = observation.timing_metrics()
    assert metrics['typed_state_samples'] == 2
    assert metrics['maximum_reported_sample_age_s'] == 0.028
    assert metrics['maximum_hardware_cycle_duration_s'] == 0.009
    assert metrics['maximum_typed_state_interval_s'] >= 0.0
    assert metrics['reported_sample_age'] == {
        'median_s': 0.012, 'percentile_95_s': 0.028,
        'percentile_99_s': 0.028,
        'maximum_s': 0.028,
    }
    assert metrics['hardware_cycle_duration'] == {
        'median_s': 0.004, 'percentile_95_s': 0.009,
        'percentile_99_s': 0.009,
        'maximum_s': 0.009,
    }


def test_preflight_ready_requires_standard_action_and_all_live_state():
    """Readiness must include command-path discovery without sending a goal."""
    module = _load_hil_module()
    observation = _observation(module)
    observation.joint_samples = 1
    observation.task_limits = (0.033, 0.071)
    observation.state = GripperState(
        model='2fg7',
        task_aperture=0.04,
        task_aperture_valid=True,
        connection_state=GripperState.CONNECTION_IDLE,
    )
    observation.action = SimpleNamespace(server_is_ready=lambda: False)
    observation.realtime_ready = lambda: False
    assert module._hardware_ready(observation)
    assert not module._hardware_ready(observation, require_action=True)
    assert not module._hardware_ready(observation, require_realtime=True)
    observation.action = SimpleNamespace(server_is_ready=lambda: True)
    observation.realtime_ready = lambda: True
    assert module._hardware_ready(observation, require_action=True)
    assert module._hardware_ready(observation, require_realtime=True)


def test_readiness_rejects_state_stale_after_a_slow_isaac_frame():
    """Discovery alone must not end startup with old queued observations."""
    module = _load_hil_module()
    observation = _observation(module)
    observation.joint_samples = 1
    observation.joint_received_at = module.time.monotonic() - 3.0
    observation.state_received_at = module.time.monotonic() - 3.0
    observation.task_limits = (0.055, 0.105)
    observation.state = GripperState(
        model='2fg14',
        task_aperture_valid=True,
        connection_state=GripperState.CONNECTION_IDLE,
    )
    observation.action = SimpleNamespace(server_is_ready=lambda: True)

    assert module._hardware_ready(observation, require_action=True)
    assert not module._hardware_ready(
        observation, require_action=True, maximum_age=0.5)

    observation.joint_received_at = module.time.monotonic()
    observation.state_received_at = module.time.monotonic()
    assert module._hardware_ready(
        observation, require_action=True, maximum_age=0.5)


def test_excursion_and_restore_are_bounded_by_live_profile():
    """Target selection must use live device limits and retain return room."""
    module = _load_hil_module()
    assert math.isclose(
        module._excursion_target(0.033, 0.033, 0.071, 0.010), 0.043)
    assert math.isclose(
        module._excursion_target(0.069, 0.033, 0.071, 0.010), 0.059)

    try:
        module._excursion_target(0.033, 0.033, 0.0335, 0.010)
    except RuntimeError as error:
        assert 'insufficient room' in str(error)
    else:
        raise AssertionError('insufficient excursion room was accepted')


def test_shadow_fidelity_must_be_within_the_declared_tolerance():
    """A HIL run must not pass when Isaac diverges from measured state."""
    module = _load_hil_module()
    module._validate_shadow_fidelity(0.0008, 0.0009, 0.001)

    with pytest.raises(RuntimeError, match='physical joint did not follow'):
        module._validate_shadow_fidelity(0.0011, 0.0, 0.001)
    with pytest.raises(RuntimeError, match='mimic joint did not follow'):
        module._validate_shadow_fidelity(0.0, 0.0011, 0.001)


def test_liveness_requires_fresh_joint_and_typed_state():
    """A discovered ROS interface must not mask stale hardware feedback."""
    module = _load_hil_module()
    observation = _observation(module)
    observation.joint_samples = 1
    observation.joint_received_at = module.time.monotonic()
    observation.state_received_at = module.time.monotonic()
    observation.state = GripperState(
        model='2fg7',
        task_aperture_valid=True,
        mapping_validity=GripperState.MAPPING_VALID,
        connection_state=GripperState.CONNECTION_IDLE,
    )
    module._validate_hardware_liveness(observation, 0.5)

    observation.state_received_at -= 1.0
    with pytest.raises(RuntimeError, match='typed gripper state is stale'):
        module._validate_hardware_liveness(observation, 0.5)

    observation.state_received_at = module.time.monotonic()
    observation.state.sample_age.sec = 1
    with pytest.raises(RuntimeError, match='device sample is stale'):
        module._validate_hardware_liveness(observation, 0.5)

    observation.state.sample_age.sec = 0
    observation.state.last_cycle_duration = math.nan
    with pytest.raises(RuntimeError, match='cycle duration is invalid'):
        module._validate_hardware_liveness(observation, 0.5)


def test_liveness_rejects_invalid_mapping_and_connection_state():
    """A finite aperture is unusable without its live semantic mapping."""
    module = _load_hil_module()
    observation = _observation(module)
    observation.joint_samples = 1
    observation.joint_received_at = module.time.monotonic()
    observation.state_received_at = module.time.monotonic()
    observation.state = GripperState(
        model='2fg7',
        task_aperture_valid=True,
        mapping_validity=GripperState.MAPPING_INVALID,
        connection_state=GripperState.CONNECTION_IDLE,
    )

    with pytest.raises(RuntimeError, match='mapping became invalid'):
        module._validate_hardware_liveness(observation, 0.5)

    observation.state.mapping_validity = GripperState.MAPPING_VALID
    observation.state.connection_state = GripperState.CONNECTION_DISCONNECTED
    with pytest.raises(RuntimeError, match='left the idle/active state'):
        module._validate_hardware_liveness(observation, 0.5)


def test_hardware_health_rejects_adverse_counter_changes():
    """Normal shadow qualification must retain communication integrity."""
    module = _load_hil_module()
    initial = {
        'sample_sequence': 100,
        'successful_hardware_cycles': 90,
        'failed_hardware_cycles': 2,
        'missed_deadlines': 0,
        'watchdog_stops': 1,
        'reconnects': 0,
    }
    final = {
        'sample_sequence': 150,
        'successful_hardware_cycles': 140,
        'failed_hardware_cycles': 2,
        'missed_deadlines': 0,
        'watchdog_stops': 1,
        'reconnects': 0,
    }
    delta = module._hardware_health_delta(initial, final)
    module._validate_hardware_health_delta(delta)
    assert delta['sample_sequence'] == 50
    assert delta['successful_hardware_cycles'] == 50

    final['reconnects'] = 1
    delta = module._hardware_health_delta(initial, final)
    with pytest.raises(RuntimeError, match='reconnects=1'):
        module._validate_hardware_health_delta(delta)

    final['reconnects'] = 0
    final['missed_deadlines'] = 1
    delta = module._hardware_health_delta(initial, final)
    with pytest.raises(RuntimeError, match='missed_deadlines=1'):
        module._validate_hardware_health_delta(delta)

    final['missed_deadlines'] = 0
    final['sample_sequence'] = 10
    delta = module._hardware_health_delta(initial, final)
    with pytest.raises(RuntimeError, match='counters reset'):
        module._validate_hardware_health_delta(delta)

    no_progress = {key: 0 for key in initial}
    with pytest.raises(RuntimeError, match='did not advance'):
        module._validate_hardware_health_delta(
            no_progress, require_progress=True)


def test_selected_controller_liveness_is_checked_after_readiness():
    """A timed preflight must fail if its selected control path disappears."""
    module = _load_hil_module()
    observation = _observation(module)
    observation.action = SimpleNamespace(server_is_ready=lambda: True)
    module._validate_controller_liveness(
        observation, 'conventional', 0.5)

    observation.action = SimpleNamespace(server_is_ready=lambda: False)
    with pytest.raises(RuntimeError, match='action became unavailable'):
        module._validate_controller_liveness(
            observation, 'conventional', 0.5)

    observation.realtime_state = RealtimeState()
    observation.realtime_state_received_at = module.time.monotonic()
    observation.realtime_publisher = SimpleNamespace(
        get_subscription_count=lambda: 1)
    observation.realtime_stop = SimpleNamespace(
        service_is_ready=lambda: True)
    module._validate_controller_liveness(
        observation, 'realtime-position', 0.5)

    observation.realtime_state_received_at -= 1.0
    with pytest.raises(RuntimeError, match='controller state is stale'):
        module._validate_controller_liveness(
            observation, 'realtime-position', 0.5)

    observation.realtime_state_received_at = module.time.monotonic()
    observation.realtime_state.faulted = True
    with pytest.raises(RuntimeError, match='reports a fault'):
        module._validate_controller_liveness(
            observation, 'realtime-position', 0.5)


def test_realtime_position_is_refreshed_until_measured_target_then_stopped(
        monkeypatch):
    """Bounded realtime motion must refresh, observe feedback, and Stop."""
    module = _load_hil_module()
    observation = _observation(module)
    published = []
    stopped = []
    pumps = 0
    observation.realtime_ready = lambda: True
    observation.realtime_publisher = SimpleNamespace(
        publish=lambda message: published.append(message))
    observation.node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: None)))
    observation.realtime_state = RealtimeState(
        realtime_active=True,
        task_position_valid=True,
        task_position=0.033,
        applied_command_sequence=7,
    )
    observation.realtime_state_received_at = module.time.monotonic()
    observation.state = GripperState(
        task_aperture_valid=True,
        task_aperture=0.033,
    )
    observation.joint_position = 0.0165
    observation.stop_realtime = (
        lambda pump, timeout=3.0: (
            stopped.append(True) or {
                'elapsed_s': 0.01,
                'newer_idle_state_observed': True,
                'applied_command_sequence': 8,
            }))

    class DeterministicStream:
        def __init__(self, publisher, stamp, position, period=0.02):
            self.publisher = publisher
            self.stamp = stamp
            self.position = position
            self.publish_count = 0

        def start(self):
            for _ in range(3):
                command = module.RealtimeCommand()
                command.mode = module.RealtimeCommand.POSITION
                command.task_position = self.position
                self.publisher.publish(command)
                self.publish_count += 1

        def raise_if_failed(self):
            pass

        def stop(self):
            pass

    monkeypatch.setattr(
        module, 'RealtimePositionCommandStream', DeterministicStream)
    monkeypatch.setattr(module, 'REALTIME_TARGET_HOLD_S', 0.0)

    def pump():
        nonlocal pumps
        pumps += 1
        if pumps == 3:
            observation.realtime_state.task_position = 0.043
            observation.realtime_state.applied_command_sequence = 8
            observation.realtime_state_received_at = module.time.monotonic()
            observation.state.task_aperture = 0.043
            observation.joint_position = 0.0215

    result = observation.send_realtime_position(0.043, pump, timeout=1.0)
    assert len(published) == 3
    assert all(message.mode == module.RealtimeCommand.POSITION
               for message in published)
    assert all(message.task_position == 0.043 for message in published)
    assert stopped == [True]
    assert result['published_command_count'] == 3
    assert result['stop']['newer_idle_state_observed']


def test_realtime_fault_still_requests_stop(monkeypatch):
    """A reported realtime fault must leave through the explicit Stop path."""
    module = _load_hil_module()
    observation = _observation(module)
    stopped = []
    observation.realtime_ready = lambda: True
    observation.realtime_publisher = SimpleNamespace(publish=lambda _: None)
    observation.node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: None)))
    observation.realtime_state = RealtimeState(
        realtime_active=True,
        task_position_valid=True,
        task_position=0.033,
    )
    observation.realtime_state_received_at = module.time.monotonic()
    observation.state = GripperState(
        task_aperture_valid=True,
        task_aperture=0.033,
    )
    observation.joint_position = 0.0165
    observation.stop_realtime = (
        lambda pump, timeout=3.0: (
            stopped.append(True) or {
                'elapsed_s': 0.01,
                'newer_idle_state_observed': True,
                'applied_command_sequence': 0,
            }))

    class DeterministicStream:
        def __init__(self, publisher, stamp, position, period=0.02):
            self.publish_count = 1

        def start(self):
            pass

        def raise_if_failed(self):
            pass

        def stop(self):
            pass

    monkeypatch.setattr(
        module, 'RealtimePositionCommandStream', DeterministicStream)

    def pump():
        observation.realtime_state.faulted = True

    with pytest.raises(RuntimeError, match='reported a fault'):
        observation.send_realtime_position(0.043, pump, timeout=1.0)
    assert stopped == [True]


def test_realtime_position_refuses_mismatched_coordinate_before_publishing():
    """Raw-mechanism feedback must be rejected before command-6 motion."""
    module = _load_hil_module()
    observation = _observation(module)
    published = []
    observation.realtime_ready = lambda: True
    observation.realtime_publisher = SimpleNamespace(
        publish=lambda message: published.append(message))
    observation.realtime_state = RealtimeState(
        task_position_valid=True,
        task_position=0.050,
    )
    observation.realtime_state_received_at = module.time.monotonic()
    observation.state = GripperState(
        task_aperture_valid=True,
        task_aperture=0.100,
    )
    observation.joint_position = 0.050

    with pytest.raises(RuntimeError, match='external-aperture coordinate'):
        observation.send_realtime_position(
            0.096, lambda: None, timeout=0.1)
    assert published == []


def test_realtime_qualification_target_requires_open_endpoint_and_4_mm_bound():
    """The first semantics proof must be a small closing move near open."""
    module = _load_hil_module()

    assert module._realtime_qualification_target(
        0.101, 0.000, 0.105, 0.004) == pytest.approx(0.097)
    with pytest.raises(RuntimeError, match='start near the open endpoint'):
        module._realtime_qualification_target(
            0.080, 0.000, 0.105, 0.004)
    with pytest.raises(RuntimeError, match='no more than'):
        module._realtime_qualification_target(
            0.101, 0.000, 0.105, 0.005)


def test_realtime_position_leg_proves_bound_refresh_and_one_jaw_mapping():
    """Qualification evidence must establish task and mechanism semantics."""
    module = _load_hil_module()
    initial_physical = module._task_to_physical_position(
        '2fg7', 0.101, (0.0, 0.107), (0.0, 0.019))
    measured_physical = module._task_to_physical_position(
        '2fg7', 0.0971, (0.0, 0.107), (0.0, 0.019))
    result = {
        'initial_task_aperture_m': 0.101,
        'target_task_aperture_m': 0.097,
        'measured_task_aperture_m': 0.0971,
        'initial_physical_position': initial_physical,
        'measured_physical_position': measured_physical,
        'minimum_observed_task_aperture_m': 0.0971,
        'maximum_observed_task_aperture_m': 0.101,
        'maximum_opposed_motion_m': 0.0,
        'measured_feedback_samples': 4,
        'published_command_count': 7,
        'initial_applied_command_sequence': 10,
        'final_applied_command_sequence': 17,
    }

    checked = module._validate_realtime_position_leg(result, 0.004)
    assert checked['maximum_observed_excursion_m'] == pytest.approx(0.0039)
    assert checked['task_to_physical_delta_error'] == pytest.approx(0.0)

    invalid = dict(result)
    invalid['measured_physical_position'] = initial_physical + 0.01
    with pytest.raises(RuntimeError, match='physical travel'):
        module._validate_realtime_position_leg(invalid, 0.004)


def test_rg2_task_to_physical_mapping_is_angular_and_nonlinear():
    """RG2 HIL comparisons must use the visual linkage angle in radians."""
    module = _load_hil_module()
    upper = 1.22277767395
    assert module._task_to_physical_position(
        'rg2', 0.0, (0.0, 0.110), (0.0, upper)) == pytest.approx(0.0)
    assert module._task_to_physical_position(
        'rg2', 0.110, (0.0, 0.110), (0.0, upper)) == pytest.approx(upper)
    midpoint = module._task_to_physical_position(
        'rg2', 0.055, (0.0, 0.110), (0.0, upper))
    assert midpoint != pytest.approx(upper / 2.0)


def test_rg6_delta_mapping_uses_measured_angle_and_finger_geometry():
    """RG6 HIL must not approximate its rotating fingers as a linear joint."""
    module = _load_hil_module()
    expected = module._expected_physical_delta(
        'rg6', 0.041, 0.0509, 0.32202464728299995,
        (0.0, 0.160), (0.0, 1.1928))
    measured = 0.38684214118588556 - 0.32202464728299995
    assert abs(expected - measured) < module.RG6_MAPPING_TOLERANCE_RAD


def test_realtime_command_stream_refreshes_until_stopped():
    """Command refresh must run independently of the observation loop."""
    module = _load_hil_module()
    published = []
    stream = module.RealtimePositionCommandStream(
        SimpleNamespace(publish=lambda message: published.append(message)),
        lambda: module.RealtimeCommand().header.stamp,
        0.097,
        period=0.002)
    stream.start()
    deadline = module.time.monotonic() + 0.2
    while len(published) < 3 and module.time.monotonic() < deadline:
        module.time.sleep(0.001)
    stream.stop()

    assert len(published) >= 3
    assert stream.publish_count == len(published)
    assert all(message.mode == module.RealtimeCommand.POSITION
               for message in published)
    assert all(
        message.task_velocity == pytest.approx(
            module.REALTIME_POSITION_MAXIMUM_VELOCITY_M_S)
        for message in published)


def test_realtime_stop_requires_state_newer_than_the_request():
    """A successful service reply alone must not qualify physical Stop."""
    module = _load_hil_module()
    observation = _observation(module)

    class CompletedFuture:
        def done(self):
            return True

        def result(self):
            return Trigger.Response(success=True, message='stopped')

    observation.realtime_stop = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda _: CompletedFuture(),
    )
    observation.realtime_state = RealtimeState(realtime_active=False)
    observation.realtime_state_received_at = module.time.monotonic()

    with pytest.raises(
            RuntimeError, match='did not become idle after Stop'):
        observation.stop_realtime(lambda: None, timeout=0.01)

    def publish_new_idle_state():
        observation.realtime_state_received_at = module.time.monotonic()

    observation.stop_realtime(publish_new_idle_state, timeout=0.1)


def test_conventional_cancel_requires_motion_cancel_result_and_idle_state():
    """Cancellation evidence must span command, result, and fresh state."""
    module = _load_hil_module()
    observation = _observation(module)
    observation.state = GripperState(
        model='2fg7',
        task_aperture=0.040,
        task_aperture_valid=True,
        sample_sequence=10,
    )
    observation.state_received_at = module.time.monotonic()

    class Future:
        def __init__(self, result=None):
            self.payload = result
            self.complete = result is not None

        def done(self):
            return self.complete

        def result(self):
            return self.payload

    result_future = Future()
    cancel_requested = False

    class GoalHandle:
        def cancel_goal_async(self):
            nonlocal cancel_requested
            cancel_requested = True
            return Future(SimpleNamespace(goals_canceling=[object()]))

    observation._start_goal = (
        lambda position, effort, pump, timeout:
        (GoalHandle(), result_future))

    def pump():
        state = observation.state
        state.sample_sequence += 1
        observation.state_received_at = module.time.monotonic()
        if not cancel_requested:
            state.busy = True
            state.task_aperture = 0.0415
            return
        state.busy = False
        result_future.payload = SimpleNamespace(
            status=module.GoalStatus.STATUS_CANCELED)
        result_future.complete = True

    result = observation.cancel_goal_after_motion(
        0.050, 10.0, 0.001, pump, 0.5)

    assert result['cancelled_result_observed'] is True
    assert result['newer_idle_state_observed'] is True
    assert result['measured_before_cancel_m'] == 0.0415
    assert result['stopped_task_aperture_m'] == 0.0415
    assert result['peak_observed_excursion_m'] == pytest.approx(0.0015)

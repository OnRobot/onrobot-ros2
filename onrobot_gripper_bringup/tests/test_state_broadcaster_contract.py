"""Check the model-neutral typed-state and diagnostics contract."""

import re
from pathlib import Path

import yaml


ROS_ROOT = Path(__file__).resolve().parents[2]
TYPE = 'onrobot_gripper_controllers/GripperStateBroadcaster'
RECOVERY_TYPE = 'onrobot_gripper_controllers/GripperRecoveryController'
REQUIRED_TASK_INTERFACES = {
    'busy', 'grip_detected', 'fault_code', 'sample_sequence', 'sample_age',
    'requested_command_sequence', 'applied_command_sequence',
    'connection_state', 'reconnects', 'firmware_qualification',
}

COMMON_DIAGNOSTIC_INTERFACES = {
    'diagnostic_status_valid', 'diagnostic_raw_status', 'diagnostic_busy',
    'diagnostic_grip_detected', 'diagnostic_sample_sequence',
    'diagnostic_age',
}

IDENTITY_DIAGNOSTIC_INTERFACES = {
    'diagnostic_identity_valid', 'diagnostic_product_code',
    'diagnostic_firmware_major', 'diagnostic_firmware_minor',
    'diagnostic_firmware_build', 'diagnostic_board_revisions_valid',
    'diagnostic_product_revision', 'diagnostic_hardware_revision',
    'diagnostic_pcb_revision', 'diagnostic_firmware_crc_valid',
    'diagnostic_firmware_crc', 'diagnostic_firmware_source_valid',
    *{f'diagnostic_firmware_git_word_{i}' for i in range(5)},
}

TWO_FINGER_DIAGNOSTIC_INTERFACES = COMMON_DIAGNOSTIC_INTERFACES | IDENTITY_DIAGNOSTIC_INTERFACES | {
    'diagnostic_command_force', 'diagnostic_command_force_valid',
    'diagnostic_measured_force', 'diagnostic_measured_force_valid',
    'diagnostic_force_provenance', 'diagnostic_motor_voltage',
    'diagnostic_motor_current', 'diagnostic_temperature',
    'diagnostic_realtime_linear_velocity',
    'diagnostic_realtime_angular_velocity', 'diagnostic_realtime_force',
    'diagnostic_supply_power', 'diagnostic_maximum_force',
    'diagnostic_maximum_realtime_force', 'diagnostic_statistics_valid',
    'diagnostic_conventional_grip_on_time', 'diagnostic_power_cycles',
    'diagnostic_conventional_grip_cycles', 'diagnostic_grip_detected_count',
    'diagnostic_realtime_grip_on_time', 'diagnostic_not_calibrated',
    'diagnostic_linear_sensor_error', 'diagnostic_external_width_mm',
    'diagnostic_internal_width_mm', 'diagnostic_minimum_external_width_mm',
    'diagnostic_maximum_external_width_mm', 'diagnostic_minimum_internal_width_mm',
    'diagnostic_maximum_internal_width_mm', 'diagnostic_additional_results_raw',
    'diagnostic_linear_mechanism_position_mm', 'diagnostic_raw_motor_width_mm',
    'diagnostic_voltage_5v_v', 'diagnostic_linear_count',
    'diagnostic_motor_speed_rpm', 'diagnostic_angle_sensor_count',
    'diagnostic_linear_error_count', 'diagnostic_realtime_external_width_mm',
}

RG_DIAGNOSTIC_INTERFACES = COMMON_DIAGNOSTIC_INTERFACES | IDENTITY_DIAGNOSTIC_INTERFACES | {
    'diagnostic_command_force', 'diagnostic_command_force_valid',
    'diagnostic_measured_force', 'diagnostic_measured_force_valid',
    'diagnostic_force_provenance', 'diagnostic_motor_voltage',
    'diagnostic_motor_current', 'diagnostic_temperature',
    'diagnostic_realtime_linear_velocity',
    'diagnostic_realtime_angular_velocity', 'diagnostic_realtime_force',
    'diagnostic_supply_power', 'diagnostic_maximum_force',
    'diagnostic_maximum_realtime_force', 'diagnostic_statistics_valid',
    'diagnostic_conventional_grip_on_time', 'diagnostic_power_cycles',
    'diagnostic_conventional_grip_cycles', 'diagnostic_grip_detected_count',
    'diagnostic_realtime_grip_on_time', 'diagnostic_rg_fingertip_offset_mm',
    'diagnostic_rg_depth_acceleration_mm_s2', 'diagnostic_rg_depth_speed_mm_s',
    'diagnostic_rg_actual_depth_mm', 'diagnostic_rg_actual_relative_depth_mm',
    'diagnostic_rg_mechanism_angle_rad',
    'diagnostic_rg_legacy_angular_velocity_rad_s',
    'diagnostic_rg_actual_width_mm', 'diagnostic_rg_voltage_5v_v',
    'diagnostic_rg_safety_24v_v', 'diagnostic_rg_plug_24v_v',
    'diagnostic_rg_width_with_fingertip_mm', 'diagnostic_rg_error_code',
}

THREE_FINGER_DIAGNOSTIC_INTERFACES = COMMON_DIAGNOSTIC_INTERFACES | {
    'diagnostic_force_percent', 'diagnostic_finger_angle_rad',
    'diagnostic_diameter_mm', 'diagnostic_diameter_with_tip_offset_mm',
    'diagnostic_voltage_24v_v', 'diagnostic_motor_current',
    'diagnostic_temperature', 'diagnostic_minimum_external_aperture_mm',
    'diagnostic_maximum_external_aperture_mm',
    'diagnostic_minimum_internal_aperture_mm',
    'diagnostic_maximum_internal_aperture_mm',
    'diagnostic_current_external_aperture_mm',
    'diagnostic_current_internal_aperture_mm',
    'diagnostic_fingertip_offset_mm', 'diagnostic_boost_power_limit_w',
    'diagnostic_three_finger_force_grip_detected',
    'diagnostic_three_finger_calibration_valid',
    'diagnostic_three_finger_grip_lost',
    'diagnostic_three_finger_position',
}


def load(path):
    """Load a YAML document relative to the ROS repository root."""
    return yaml.safe_load((ROS_ROOT / path).read_text(encoding='utf-8'))


def node_parameters(document, node_name):
    """Return parameters from a namespace-wildcard node selector."""
    return document[f'/**/{node_name}']['ros__parameters']


def test_all_controller_sets_declare_the_typed_state_broadcaster():
    """Require the broadcaster type in every supported controller set."""
    controller_files = (
        'onrobot_2fg7/config/ros2_control_controllers.yaml',
        'onrobot_2fg14/config/ros2_control_controllers.yaml',
        'onrobot_gripper_controllers/config/rg.yaml',
    )
    for filename in controller_files:
        manager = node_parameters(load(filename), 'controller_manager')
        assert manager['gripper_state_broadcaster']['type'] == TYPE, filename
        recovery_type = manager['recovery_controller']['type']
        assert recovery_type == RECOVERY_TYPE, filename


def test_three_finger_showcase_controller_contract_is_namespace_safe():
    """Keep the native 3FG controller set loadable under a namespace."""
    manager = node_parameters(
        load('onrobot_gripper_demos/launch/three_finger_controllers.yaml'),
        'controller_manager')
    assert manager['joint_state_broadcaster']['type'] == \
        'joint_state_broadcaster/JointStateBroadcaster'
    assert manager['diameter_controller']['type'] == \
        'forward_command_controller/ForwardCommandController'
    assert manager['gripper_state_broadcaster']['type'] == TYPE
    assert manager['three_finger_limit_broadcaster']['type'] == \
        'state_interfaces_broadcaster/StateInterfacesBroadcaster'


def test_profiles_identify_each_model_and_coordinate_dimension():
    """Keep model/profile provenance explicit and dimensionally correct."""
    profiles = {
        '2fg7': (
            'onrobot_2fg7/config/ros2_control_controllers.yaml', 'linear'),
        '2fg14': (
            'onrobot_2fg14/config/ros2_control_controllers.yaml', 'linear'),
        'rg2': (
            'onrobot_rg2/config/gripper_state_broadcaster.yaml', 'angular'),
        'rg6': (
            'onrobot_rg6/config/gripper_state_broadcaster.yaml', 'angular'),
    }
    for model, (filename, dimension) in profiles.items():
        params = node_parameters(load(filename), 'gripper_state_broadcaster')
        assert params['model'] == model, filename
        assert params['mechanism_dimension'] == dimension, filename
        assert params['finger_profile_name'] == f'{model}_standard', filename


def test_hardware_descriptions_expose_state_provenance_interfaces():
    """Require real/fake hardware to share typed-state source interfaces."""
    xacros = (
        'onrobot_2fg7/urdf/onrobot_2fg7_ros2_control.xacro',
        'onrobot_2fg14/urdf/onrobot_2fg14_realmesh_macro.xacro',
        'onrobot_rg2/urdf/onrobot_rg2_realmesh_macro.xacro',
        'onrobot_rg6/urdf/onrobot_rg6_realmesh_macro.xacro',
    )
    for filename in xacros:
        text = (ROS_ROOT / filename).read_text(encoding='utf-8')
        for interface in REQUIRED_TASK_INTERFACES:
            assert f'name="{interface}"' in text, (filename, interface)
        assert 'name="fault_recovery_command_sequence"' in text, filename


def test_all_model_families_expose_the_documented_diagnostic_surface():
    """Keep register-derived diagnostics available to the broadcaster."""
    profiles = {
        '2fg7/urdf/onrobot_2fg7_ros2_control.xacro':
            TWO_FINGER_DIAGNOSTIC_INTERFACES,
        '2fg14/urdf/onrobot_2fg14_realmesh_macro.xacro':
            TWO_FINGER_DIAGNOSTIC_INTERFACES,
        'rg2/urdf/onrobot_rg2_realmesh_macro.xacro':
            RG_DIAGNOSTIC_INTERFACES,
        'rg6/urdf/onrobot_rg6_realmesh_macro.xacro':
            RG_DIAGNOSTIC_INTERFACES,
        '3fg15/urdf/realmesh_onrobot_3fg15.urdf.xacro':
            THREE_FINGER_DIAGNOSTIC_INTERFACES,
        '3fg25/urdf/realmesh_onrobot_3fg25.urdf.xacro':
            THREE_FINGER_DIAGNOSTIC_INTERFACES,
    }
    for filename, interfaces in profiles.items():
        package, relative = filename.split('/', 1)
        text = (ROS_ROOT / f'onrobot_{package}' / relative).read_text(
            encoding='utf-8')
        for interface in interfaces:
            assert f'name="{interface}"' in text, (filename, interface)


def test_diagnostic_interface_table_has_unique_stable_names():
    """Prevent a state-interface index from silently changing meaning."""
    header = (ROS_ROOT / 'onrobot_gripper_msgs/include/onrobot_gripper_msgs/'
              'diagnostic_interfaces.hpp').read_text(encoding='utf-8')
    names = re.findall(r'"(diagnostic_[^"]+)"', header)
    assert len(names) == 90
    assert len(names) == len(set(names))
    assert set(names[-17:]) == IDENTITY_DIAGNOSTIC_INTERFACES


def test_launches_activate_the_broadcaster_in_all_control_modes():
    """Keep typed state independent of conventional/realtime ownership."""
    launches = (
        'onrobot_2fg7/launch/control.launch.py',
        'onrobot_2fg14/launch/control.launch.py',
        'onrobot_rg2/launch/control.launch.py',
        'onrobot_rg6/launch/control.launch.py',
    )
    for filename in launches:
        text = (ROS_ROOT / filename).read_text(encoding='utf-8')
        assert 'gripper_state_broadcaster' in text, filename
        assert 'recovery_controller' in text, filename


def test_all_supported_control_launches_are_namespace_and_tf_prefix_safe():
    """Reject absolute manager paths and unprefixed published base frames."""
    launches = (
        'onrobot_2fg7/launch/control.launch.py',
        'onrobot_2fg14/launch/control.launch.py',
        'onrobot_rg2/launch/control.launch.py',
        'onrobot_rg6/launch/control.launch.py',
    )
    for filename in launches:
        text = (ROS_ROOT / filename).read_text(encoding='utf-8')
        assert '/controller_manager' not in text, filename
        assert 'LaunchConfiguration("frame_prefix")' in text or \
            "LaunchConfiguration('frame_prefix')" in text, filename
        assert '"frame_prefix": frame_prefix' in text or \
            "'frame_prefix': frame_prefix" in text, filename

    controller_files = (
        'onrobot_2fg7/config/ros2_control_controllers.yaml',
        'onrobot_2fg14/config/ros2_control_controllers.yaml',
        'onrobot_gripper_controllers/config/rg.yaml',
    )
    for filename in controller_files:
        document = load(filename)
        assert '/**/controller_manager' in document, filename

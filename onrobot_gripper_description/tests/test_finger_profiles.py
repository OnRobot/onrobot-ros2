"""Validate the declarative custom-finger profile contract."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml


PROFILE_DIR = Path(__file__).parents[1] / 'config' / 'finger_profiles'
EXAMPLE_DIR = Path(__file__).parents[1] / 'examples' / 'custom_fingers'
ROS_ROOT = Path(__file__).parents[2]
SCHEMA = yaml.safe_load(
    (PROFILE_DIR / 'schema.yaml').read_text(encoding='utf-8'))
REQUIRED = {
    'schema_version', 'name', 'revision', 'compatible_models',
    'task_coordinate', 'mechanism_coordinate', 'attachment_frames',
}


def profiles():
    """Return all concrete finger-profile files."""
    return sorted(path for path in PROFILE_DIR.glob('*.yaml')
                  if path.name != 'schema.yaml')


def test_profiles_are_declarative_and_dimensionally_explicit():
    """Require task and mechanism coordinates to state dimensions and units."""
    seen_names = set()
    assert profiles()
    for path in profiles():
        profile = yaml.safe_load(path.read_text(encoding='utf-8'))
        assert REQUIRED <= profile.keys(), path
        assert profile['schema_version'] == 1, path
        assert profile['name'] not in seen_names, path
        seen_names.add(profile['name'])
        assert profile['compatible_models'], path
        assert profile['attachment_frames'] == \
            SCHEMA['attachment_frame_contract'], path

        task = profile['task_coordinate']
        mechanism = profile['mechanism_coordinate']
        assert task['dimension'] == 'linear', path
        assert task['unit'] == 'm', path
        assert task['mapping_source'] in SCHEMA['mapping_sources'], path
        assert mechanism['dimension'] in SCHEMA['coordinate_dimensions'], path
        expected_unit = SCHEMA['coordinate_units'][mechanism['dimension']]
        assert mechanism['unit'] == expected_unit, path
        assert task['minimum'] <= task['maximum'], path
        assert mechanism['minimum'] <= mechanism['maximum'], path


def test_profile_filenames_match_profile_names():
    """Keep the profile identity stable and discoverable from its filename."""
    for path in profiles():
        profile = yaml.safe_load(path.read_text(encoding='utf-8'))
        assert path.stem == profile['name']


def test_rg_task_limits_match_the_authoritative_model_ranges():
    """Keep RG task limits aligned with the realtime integration guide."""
    expected_maximum = {'rg2': 0.110, 'rg6': 0.160}
    for model, maximum in expected_maximum.items():
        profile = yaml.safe_load(
            (PROFILE_DIR / f'{model}_standard.yaml').read_text(
                encoding='utf-8'))
        assert profile['task_coordinate']['minimum'] == 0.0
        assert profile['task_coordinate']['maximum'] == maximum


def test_supported_xacros_expose_stable_finger_attachment_frames():
    """Keep standard tips downstream of the public custom-finger frames."""
    for profile_path in profiles():
        profile = yaml.safe_load(profile_path.read_text(encoding='utf-8'))
        for model in profile['compatible_models']:
            urdf_dir = ROS_ROOT / f'onrobot_{model}' / 'urdf'
            for variant in ('realmesh', 'simplemesh'):
                xacro_path = urdf_dir / (
                    f'{variant}_onrobot_{model}.urdf.xacro')
                result = subprocess.run(
                    ['xacro', str(xacro_path)],
                    check=True,
                    capture_output=True,
                    text=True,
                )
                robot = ET.fromstring(result.stdout)
                links = {link.attrib['name'] for link in robot.findall('link')}
                joints = {
                    joint.attrib['name']: joint
                    for joint in robot.findall('joint')
                }

                for side in ('left', 'right'):
                    mount = f'{side}_finger_mount'
                    mount_joint = joints[f'{mount}_joint']
                    tip_joint_name = (
                        f'{side}_fingertip_joint' if model.startswith('2fg')
                        else f'{side}_finger_tip_joint')
                    tip_joint = joints[tip_joint_name]

                    assert mount in links, xacro_path
                    assert mount_joint.attrib['type'] == 'fixed', xacro_path
                    assert mount_joint.find('parent').attrib['link'] == \
                        f'{side}_finger_base_link', xacro_path
                    assert mount_joint.find('child').attrib['link'] == mount, \
                        xacro_path
                    assert mount_joint.find('origin').attrib['rpy'] == \
                        '0 0 0', xacro_path
                    assert tip_joint.attrib['type'] == 'fixed', xacro_path
                    assert tip_joint.find('parent').attrib['link'] == mount, \
                        xacro_path
                    origin = tip_joint.find('origin')
                    assert origin.attrib['xyz'] == '0 0 0', xacro_path

                if model.startswith('rg'):
                    outwards = subprocess.run(
                        [
                            'xacro', str(xacro_path),
                            'fingertip_orientation:=outwards',
                        ],
                        check=True,
                        capture_output=True,
                        text=True,
                    )
                    outwards_robot = ET.fromstring(outwards.stdout)
                    outwards_joints = {
                        joint.attrib['name']: joint
                        for joint in outwards_robot.findall('joint')
                    }
                    for side in ('left', 'right'):
                        joint_name = f'{side}_finger_mount_joint'
                        default_origin = joints[joint_name].find('origin')
                        outwards_origin = outwards_joints[joint_name].find(
                            'origin')
                        assert default_origin.attrib == \
                            outwards_origin.attrib, xacro_path


def test_standard_fingertips_can_be_replaced_without_mechanism_changes():
    """Omit vendor tips while retaining mounts and all mechanism joints."""
    for profile_path in profiles():
        profile = yaml.safe_load(profile_path.read_text(encoding='utf-8'))
        for model in profile['compatible_models']:
            urdf_dir = ROS_ROOT / f'onrobot_{model}' / 'urdf'
            for variant in ('realmesh', 'simplemesh'):
                xacro_path = urdf_dir / (
                    f'{variant}_onrobot_{model}.urdf.xacro')
                default = ET.fromstring(subprocess.run(
                    ['xacro', str(xacro_path)],
                    check=True, capture_output=True, text=True,
                ).stdout)
                custom = ET.fromstring(subprocess.run(
                    [
                        'xacro', str(xacro_path),
                        'use_standard_fingertips:=false',
                    ],
                    check=True, capture_output=True, text=True,
                ).stdout)

                default_links = {
                    link.attrib['name'] for link in default.findall('link')}
                custom_links = {
                    link.attrib['name'] for link in custom.findall('link')}
                default_joints = {
                    joint.attrib['name'] for joint in default.findall('joint')}
                custom_joints = {
                    joint.attrib['name'] for joint in custom.findall('joint')}
                tip_links = {
                    f'{side}_fingertip_link' if model.startswith('2fg')
                    else f'{side}_finger_tip_link'
                    for side in ('left', 'right')
                }
                tip_joints = {
                    f'{side}_fingertip_joint' if model.startswith('2fg')
                    else f'{side}_finger_tip_joint'
                    for side in ('left', 'right')
                }

                assert tip_links <= default_links, xacro_path
                assert tip_joints <= default_joints, xacro_path
                assert not tip_links & custom_links, xacro_path
                assert not tip_joints & custom_joints, xacro_path
                assert default_links - tip_links == custom_links, xacro_path
                assert default_joints - tip_joints == custom_joints, xacro_path


def test_2fg_uses_physical_one_jaw_articulation_without_virtual_mechanism():
    """Keep ROS kinematics aligned with the USD/PhysX articulation."""
    raw_maximum_by_model = {'2fg7': '39.0', '2fg14': '51.0'}
    finger_maximum_by_model = {'2fg7': '0.019', '2fg14': '0.025'}
    for model in ('2fg7', '2fg14'):
        profile = yaml.safe_load(
            (PROFILE_DIR / f'{model}_standard.yaml').read_text(
                encoding='utf-8'))
        mechanism = profile['mechanism_coordinate']
        assert profile['revision'] == 2
        assert mechanism['minimum'] == 0.001
        assert mechanism['maximum'] == \
            float(raw_maximum_by_model[model]) / 1000.0
        urdf_dir = ROS_ROOT / f'onrobot_{model}' / 'urdf'
        for variant in ('realmesh', 'simplemesh'):
            xacro_path = urdf_dir / (
                f'{variant}_onrobot_{model}.urdf.xacro')
            result = subprocess.run(
                ['xacro', str(xacro_path)],
                check=True,
                capture_output=True,
                text=True,
            )
            robot = ET.fromstring(result.stdout)
            links = {link.attrib['name'] for link in robot.findall('link')}
            joints = {
                joint.attrib['name']: joint
                for joint in robot.findall('joint')
            }

            assert 'virtual_stroke_link' not in links, xacro_path
            assert 'mechanism_stroke' not in joints, xacro_path
            finger = joints['finger_stroke']
            assert finger.attrib['type'] == 'prismatic', xacro_path
            assert finger.find('parent').attrib['link'] == 'base_link', \
                xacro_path
            assert finger.find('child').attrib['link'] == \
                'right_finger_base_link', xacro_path
            assert finger.find('mimic') is None, xacro_path

            follower = joints['left_finger_base_joint']
            mimic = follower.find('mimic')
            assert mimic is not None, xacro_path
            assert mimic.attrib == {
                'joint': 'finger_stroke',
                'multiplier': '1.0',
                'offset': '0',
            }, xacro_path

            if variant == 'realmesh':
                hardware = robot.find('ros2_control/hardware')
                parameters = {
                    parameter.attrib['name']: parameter.text
                    for parameter in hardware.findall('param')
                }
                assert parameters['finger_joint_upper_m'] == \
                    finger_maximum_by_model[model], xacro_path
                assert parameters['raw_linear_min_mm'] == '1.0', xacro_path
                assert parameters['raw_linear_max_mm'] == \
                    raw_maximum_by_model[model], xacro_path


def test_external_custom_finger_fixture_is_a_valid_robot():
    """Prove that a consumer-owned Xacro can attach replacement fingers."""
    fixture = EXAMPLE_DIR / '2fg7_custom_fingers.urdf.xacro'
    result = subprocess.run(
        ['xacro', str(fixture)],
        check=True,
        capture_output=True,
        text=True,
    )
    robot = ET.fromstring(result.stdout)
    links = {link.attrib['name'] for link in robot.findall('link')}
    joints = {joint.attrib['name']: joint for joint in robot.findall('joint')}

    assert 'left_fingertip_link' not in links
    assert 'right_fingertip_link' not in links
    for side in ('left', 'right'):
        user_link = f'user_{side}_finger'
        user_joint = joints[f'{user_link}_joint']
        assert user_link in links
        assert user_joint.find('parent').attrib['link'] == \
            f'{side}_finger_mount'
        assert user_joint.find('child').attrib['link'] == user_link


def test_dual_gripper_arm_composition_has_unique_prefixed_names():
    """Expand both 2FG7 mesh variants under caller-owned tool links."""
    fixture_dir = Path(__file__).parents[1] / 'examples' / 'composition'
    for fixture_name in (
            'dual_2fg7_arm.urdf.xacro',
            'dual_2fg7_realmesh_arm.urdf.xacro',
            'dual_2fg14_arm.urdf.xacro',
            'dual_2fg14_realmesh_arm.urdf.xacro',
            'dual_rg2_arm.urdf.xacro',
            'dual_rg2_realmesh_arm.urdf.xacro',
            'dual_rg6_arm.urdf.xacro',
            'dual_rg6_realmesh_arm.urdf.xacro'):
        fixture = fixture_dir / fixture_name
        result = subprocess.run(
            ['xacro', str(fixture)],
            check=True,
            capture_output=True,
            text=True,
        )
        robot = ET.fromstring(result.stdout)
        links = [link.attrib['name'] for link in robot.findall('link')]
        joints = [joint.attrib['name'] for joint in robot.findall('joint')]
        assert len(links) == len(set(links)), fixture
        assert len(joints) == len(set(joints)), fixture
        assert robot.find('ros2_control') is None, fixture

        by_name = {
            joint.attrib['name']: joint for joint in robot.findall('joint')}
        tip_stem = 'finger_tip' if 'rg' in fixture_name else 'fingertip'
        for side in ('left', 'right'):
            prefix = f'{side}_gripper_'
            mount = by_name[f'{prefix}mount_joint']
            assert mount.find('parent').attrib['link'] == \
                f'{side}_tool0', fixture
            assert mount.find('child').attrib['link'] == \
                f'{prefix}base_link', fixture
            assert mount.find('origin').attrib['xyz'] == \
                '0 0 0.08', fixture
            assert f'{prefix}left_finger_mount' in links, fixture
            assert f'{prefix}right_finger_mount' in links, fixture
            assert f'{prefix}left_{tip_stem}_link' not in links, fixture
            assert f'{prefix}right_{tip_stem}_link' not in links, fixture


def test_2fg_composed_control_resources_are_prefixed():
    """Keep embedded control resources aligned with prefixed URDF joints."""
    fixture_dir = Path(__file__).parents[1] / 'examples' / 'composition'
    for model in ('2fg7', '2fg14'):
        fixture = fixture_dir / f'dual_{model}_realmesh_arm.urdf.xacro'
        result = subprocess.run(
            ['xacro', str(fixture), 'include_ros2_control:=true'],
            check=True,
            capture_output=True,
            text=True,
        )
        robot = ET.fromstring(result.stdout)
        controls = robot.findall('ros2_control')
        system_name = model.upper()
        assert {control.attrib['name'] for control in controls} == {
            f'left_gripper_OnRobot{system_name}System',
            f'right_gripper_OnRobot{system_name}System',
        }
        for control in controls:
            prefix = control.attrib['name'].split('OnRobot', maxsplit=1)[0]
            control_joints = {
                joint.attrib['name'] for joint in control.findall('joint')}
            assert control_joints == {
                f'{prefix}grip_stroke',
                f'{prefix}finger_stroke',
            }


def test_rg_composed_control_resources_are_prefixed():
    """Keep RG embedded control resources aligned with prefixed joints."""
    fixture_dir = Path(__file__).parents[1] / 'examples' / 'composition'
    for model in ('rg2', 'rg6'):
        fixture = fixture_dir / f'dual_{model}_realmesh_arm.urdf.xacro'
        result = subprocess.run(
            ['xacro', str(fixture), 'include_ros2_control:=true'],
            check=True,
            capture_output=True,
            text=True,
        )
        robot = ET.fromstring(result.stdout)
        controls = robot.findall('ros2_control')
        system_name = model.upper()
        assert {control.attrib['name'] for control in controls} == {
            f'left_gripper_OnRobot{system_name}System',
            f'right_gripper_OnRobot{system_name}System',
        }
        for control in controls:
            prefix = control.attrib['name'].split('OnRobot', maxsplit=1)[0]
            control_joints = {
                joint.attrib['name'] for joint in control.findall('joint')}
            assert control_joints == {
                f'{prefix}grip_stroke',
                f'{prefix}finger_joint',
            }

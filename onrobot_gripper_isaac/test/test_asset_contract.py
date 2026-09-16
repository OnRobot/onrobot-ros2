"""Offline checks for the versioned 2FG7 Isaac asset contract."""

import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parent
sys.path.insert(0, str(PACKAGE_ROOT / 'scripts'))
from isaac_model_contract import asset_repository_root
ASSET_REPOSITORY = asset_repository_root(PACKAGE_ROOT)


def _copy_package(destination):
    shutil.copytree(PACKAGE_ROOT, destination)
    shutil.copytree(ASSET_REPOSITORY / 'assets', destination / 'assets')


def _load_validator():
    path = PACKAGE_ROOT / 'scripts/validate_asset_contract.py'
    spec = importlib.util.spec_from_file_location(
        'onrobot_2fg7_asset_contract_validator', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_offline_asset_contract_passes():
    """The checked-in USD and ROS descriptions must agree."""
    result = subprocess.run(
        [
            sys.executable,
            str(PACKAGE_ROOT / 'scripts/validate_asset_contract.py'),
            '--package-root', str(PACKAGE_ROOT),
            '--model-root', str(REPOSITORY_ROOT / 'onrobot_2fg7'),
            '--description-root',
            str(REPOSITORY_ROOT / 'onrobot_gripper_description'),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    report = json.loads(result.stdout)
    assert report['model'] == '2fg7'
    assert report['offline_contract'] == 'passed'
    assert report['runtime_validation'] == 'kinematic-automated-passed'
    assert report['fidelity'] == 'kinematic'
    assert report['dynamic_contact_status'] == 'runtime-pending'


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
@pytest.mark.parametrize('side', ['actual', 'expected'])
def test_numeric_contract_checks_reject_nonfinite_values(value, side):
    validator = _load_validator()
    actual, expected = (value, 1.) if side == 'actual' else (1., value)
    with pytest.raises(AssertionError):
        validator._assert_close(actual, expected, 'test')
    with pytest.raises(AssertionError):
        validator._assert_near(actual, expected, .1, 'test')


def test_public_contract_validation_does_not_need_internal_reports(tmp_path):
    """The installed structural checker must work without private reports."""
    package_root = tmp_path / 'onrobot_gripper_isaac'
    _copy_package(package_root)
    shutil.rmtree(package_root / 'evidence')
    report = _load_validator().validate(
        package_root,
        REPOSITORY_ROOT / 'onrobot_2fg7',
        REPOSITORY_ROOT / 'onrobot_gripper_description')
    assert report['offline_contract'] == 'passed'
    assert report['runtime_validation'] == 'not-published'
    assert report['dynamic_contact_status'] == 'not-published'
    assert report['hardware_in_loop_status'] == 'not-published'


def test_2fg14_ros_contract_uses_kinematic_joint_definitions():
    """ROS validation must ignore same-name ros2_control declarations."""
    validator = _load_validator()
    contract = json.loads(
        (PACKAGE_ROOT / 'config/2fg14_asset_contract.json').read_text(
            encoding='utf-8'))
    validator._validate_ros(
        contract,
        REPOSITORY_ROOT / 'onrobot_2fg14',
        REPOSITORY_ROOT / 'onrobot_gripper_description')


def test_rg2_offline_report_retains_runtime_and_contact_qualification():
    """Expose the two independently retained RG2 runtime gates."""
    result = subprocess.run(
        [
            sys.executable,
            str(PACKAGE_ROOT / 'scripts/validate_asset_contract.py'),
            '--model', 'rg2',
            '--package-root', str(PACKAGE_ROOT),
            '--model-root', str(REPOSITORY_ROOT / 'onrobot_rg2'),
            '--description-root',
            str(REPOSITORY_ROOT / 'onrobot_gripper_description'),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    report = json.loads(result.stdout)
    assert report['model'] == 'rg2'
    assert report['runtime_validation'] == 'kinematic-automated-passed'
    assert report['dynamic_contact_status'] == (
        'passed-isaac-sim-6.0.1')


@pytest.mark.parametrize('model', ['rg2', 'rg6'])
def test_rg_fingertips_use_schema_backed_physics_material_bindings(model):
    """Fingertip friction must be a composed physics material binding."""
    physics = (ASSET_REPOSITORY / f'assets/{model}/payloads/Physics/'
               'physics.usda').read_text(encoding='utf-8')
    validator = _load_validator()
    for side in ('left', 'right'):
        block = validator._joint_block(
            physics, f'over "{side}_finger_tip_link"')
        header = validator._declaration_header(
            physics, f'over "{side}_finger_tip_link"')
        assert 'MaterialBindingAPI' in header
        assert 'custom rel material:binding:physics' not in block
        assert ('rel material:binding:physics = '
                f'</onrobot_{model}/Physics/fingertip_physics_material>'
                in block)


@pytest.mark.parametrize('model', ['rg2', 'rg6'])
@pytest.mark.parametrize('field', ['armature_kg_m2', 'damping_si'])
def test_rg_drive_parameter_drift_is_detected(model, field):
    validator = _load_validator()
    contract = json.loads((PACKAGE_ROOT / f'config/{model}_asset_contract.json').read_text())
    root = ASSET_REPOSITORY / 'assets' / model
    def read(relative):
        return (root / relative).read_text()
    inputs = dict(entry=read(f'onrobot_{model}.usda'), physics=read('payloads/Physics/physics.usda'),
                  physx=read('payloads/Physics/physx_parallel_grip_tip_contact.usda'),
                  default_physx=read('payloads/Physics/physx_parallel_grip.usda'),
                  robot=read('payloads/robot.usda'), instances=read('payloads/instances.usda'),
                  usd=contract['usd'], contract=contract)
    validator._validate_rg_usd(**inputs)
    contract['drive_tuning'][field] *= 2
    with pytest.raises(AssertionError, match='armature|angular unit conversion'):
        validator._validate_rg_usd(**inputs)


@pytest.mark.parametrize('model', ['2fg7', '2fg14'])
@pytest.mark.parametrize('field', ['armature_kg', 'damping'])
def test_2fg_drive_parameter_drift_is_detected(model, field):
    validator = _load_validator()
    contract = json.loads((PACKAGE_ROOT / f'config/{model}_asset_contract.json').read_text())
    validator._validate_usd(PACKAGE_ROOT, contract)
    contract['drive_tuning'][field] *= 2
    with pytest.raises(AssertionError, match='armature|damping'):
        validator._validate_usd(PACKAGE_ROOT, contract)


@pytest.mark.parametrize('model', ['2fg7', '2fg14'])
def test_2fg_collision_meshes_preserve_concave_finger_geometry(model):
    """L-shaped fingers must not be replaced by corner-filling hulls."""
    instances = (
        ASSET_REPOSITORY / f'assets/{model}/payloads/instances.usda').read_text(
            encoding='utf-8')
    assert 'physics:approximation = "convexDecomposition"' in instances
    assert 'physics:approximation = "convexHull"' not in instances


@pytest.mark.parametrize('model', ['rg2', 'rg6'])
def test_rg_collision_meshes_follow_family_decomposition_policy(model):
    """RG contact meshes must preserve non-convex finger geometry."""
    instances = (
        ASSET_REPOSITORY / f'assets/{model}/payloads/instances.usda').read_text(
            encoding='utf-8')
    assert 'physics:approximation = "convexDecomposition"' in instances
    assert 'physics:approximation = "convexHull"' not in instances


def test_rg6_uses_closed_standard_pad_contact_proxies():
    """RG6 must not feed its non-manifold imported tip mesh to PhysX."""
    physics = (
        ASSET_REPOSITORY / 'assets/rg6/payloads/Physics/physics.usda').read_text(
            encoding='utf-8')
    contract = json.loads(
        (PACKAGE_ROOT / 'config/rg6_asset_contract.json').read_text(
            encoding='utf-8'))
    proxy = contract['collision_model']['fingertip_contact_proxy']
    assert proxy['source'] == (
        'visible-pad-contact-face-with-rigid-fingertip-backing')
    assert proxy['contact_face_local_z_m'] == pytest.approx(0.005)
    assert (proxy['center_m'][2] + proxy['size_m'][2] / 2.0 ==
            pytest.approx(proxy['contact_face_local_z_m']))
    assert physics.count('def Cube "payload_contact_collision"') == 2
    assert physics.count('over "fingertip_standard_link_1"') == 2
    assert physics.count('active = false') >= 2
    assert physics.count(
        'float3 xformOp:scale = (0.037, 0.025, 0.0114)') == 2
    assert physics.count(
        'double3 xformOp:translate = (0, 0.0001, -0.0007)') == 2


def test_2fg14_collision_meshes_are_not_rendered_as_visual_geometry():
    """Collision-only meshes must remain hidden from normal rendering."""
    instances = (
        ASSET_REPOSITORY / 'assets/2fg14/payloads/instances.usda').read_text(
            encoding='utf-8')
    collision_mesh_count = instances.count('PhysicsMeshCollisionAPI')
    assert collision_mesh_count == 3
    assert instances.count('token purpose = "guide"') == (
        collision_mesh_count)


def test_2fg14_standard_fingertips_use_reviewed_primitive_collisions():
    """The imported tip mesh must not own contact after decomposition drift."""
    contract = json.loads(
        (PACKAGE_ROOT / 'config/2fg14_asset_contract.json').read_text(
            encoding='utf-8'))
    base = (
        ASSET_REPOSITORY / 'assets/2fg14/payloads/base.usda').read_text(
            encoding='utf-8')
    instances = (
        ASSET_REPOSITORY / 'assets/2fg14/payloads/instances.usda').read_text(
            encoding='utf-8')
    _load_validator()._validate_reference_finger_collisions(
        contract, base, instances)
    assert base.count('def Cube "upper_contact"') == 2
    assert base.count('def Cube "lower_finger"') == 2
    assert instances.count('bool physics:collisionEnabled = 0') == 1


@pytest.mark.parametrize('model', ['2fg7', '2fg14'])
def test_2fg_release_layers_do_not_embed_generator_paths(model):
    """Release assets must not retain local authoring workspace paths."""
    asset_root = ASSET_REPOSITORY / f'assets/{model}'
    for layer in asset_root.rglob('*.usda'):
        text = layer.read_text(encoding='utf-8')
        assert '/tmp/' not in text
        assert '/home/' not in text
        assert '/Users/' not in text
        assert 'C:\\' not in text


def test_2fg14_offline_asset_contract_exposes_pending_runtime_reruns():
    """The current 2FG14 asset must not inherit stale runtime claims."""
    result = subprocess.run(
        [
            sys.executable,
            str(PACKAGE_ROOT / 'scripts/validate_asset_contract.py'),
            '--model', '2fg14',
            '--package-root', str(PACKAGE_ROOT),
            '--model-root', str(REPOSITORY_ROOT / 'onrobot_2fg14'),
            '--description-root',
            str(REPOSITORY_ROOT / 'onrobot_gripper_description'),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    report = json.loads(result.stdout)
    assert report == {
        'asset_revision': '3.0',
        'dynamic_contact_status': 'runtime-pending',
        'fidelity': 'kinematic',
        'hardware_in_loop_status': 'runtime-pending-current-asset',
        'model': '2fg14',
        'offline_contract': 'passed',
        'physics_backend': 'physx',
        'runtime_validation': 'kinematic-automated-passed',
        'tested_isaac_sim_versions': ['6.0.1'],
    }


def test_runtime_claims_remain_evidence_gated():
    """Runtime claims must match retained evidence and pending boundaries."""
    contract = json.loads(
        (PACKAGE_ROOT / 'config/2fg7_asset_contract.json').read_text(
            encoding='utf-8'))
    assert contract['runtime'] == {
        'tested_isaac_sim_versions': ['6.0.1'],
        'version_policy': 'feature-gated',
        'physics_backend': 'physx',
        'ros_distribution': 'jazzy',
        'runtime_validation': 'kinematic-automated-passed',
        'fidelity': 'kinematic',
    }
    assert contract['drive_tuning']['qualified'] is False
    assert set(contract['realtime'].values()) == {
        'runtime-passed'}
    assert contract['dynamic_contact'] == {
        'status': 'runtime-pending',
        'reference_object_config': (
            'config/2fg7_dynamic_contact_reference_objects.json'),
        'reference_object_config_sha256': (
            '6bc270ce3c670cb29da238911ed1f307'
            '8ce48209753852bf18f15cb61a728678'),
        'test_harness': 'run_2fg_dynamic_contact_test.py',
        'reference_fingers': 'standard-silicone-outwards',
        'object_set': ['external_pinch_block_v1'],
        'reason': (
            'The articulation drive was retuned after the retained Isaac '
            'Sim 6.0.1 contact run.'),
        'required_rerun': 'dynamic-contact-current-asset',
    }
    qualification = dict(contract['qualification'])
    assert qualification.pop('robot_free_workpiece')['status'] == 'passed'
    assert qualification.pop('physical_motion_proof')['replay'] is False
    assert qualification == {
        'qualified_at': '2026-09-01',
        'articulation_endpoints': 'automated-passed',
        'ros_conventional_control': 'operator-passed',
        'ros_realtime_control': 'operator-passed',
        'rviz_isaac_motion_agreement': 'operator-passed',
        'automated_kinematic_regression': 'automated-passed',
        'kinematic_evidence': (
            'evidence/2fg7_kinematic_isaac_sim_6_0_1_2026-08-31.json'),
        'kinematic_source_evidence_sha256': (
            'a5e287c3f400ce3a5a2f3c631055d737'
            'd2bcb3f275b7f709c2ba341e8e7811df'),
        'physical_motion_showcase': 'automated-passed',
        'contact_and_grasp': 'runtime-pending-current-asset',
        'drive_tuning': 'pending',
    }


@pytest.mark.parametrize('model', ('2fg7', '2fg14', 'rg2', 'rg6'))
def test_workpiece_claim_is_bound_to_the_current_asset_bytes(model):
    import hashlib
    from grip_drive_control import script_manifest

    contract = json.loads((PACKAGE_ROOT / 'config' / f'{model}_asset_contract.json').read_text())
    qualification = contract['qualification']
    proof = qualification['robot_free_workpiece']
    root = ASSET_REPOSITORY / 'assets' / model
    files = {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
             for path in sorted(root.rglob('*')) if path.is_file()}
    digest = hashlib.sha256(json.dumps(files, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    assert proof['asset_manifest_sha256'] == digest
    runner = PACKAGE_ROOT / 'scripts/run_workpiece_showcase.py'
    assert proof['runner_sha256'] == hashlib.sha256(runner.read_bytes()).hexdigest()
    assert proof['runtime_script_files_sha256'] == script_manifest(runner)
    assert len(proof['report_sha256']) == 64
    assert int(proof['report_sha256'], 16) >= 0
    assert proof['physics_hz'] == 480
    assert proof['external_forces_every_iteration'] is True
    assert proof['hold_control'] == 'position'
    assert proof['isaac_sim_version'] == '6.0.1'
    assert proof['cube_size_m'] == 0.062
    assert proof['payload_kg'] == 0.712
    assert proof['pickup_setdown_cycles'] == 3
    assert proof['pad_material_override'] is False
    assert 'not rated-payload or force calibration' in proof['scope']


@pytest.mark.parametrize('model', ('2fg7', '2fg14', 'rg2', 'rg6'))
def test_physical_motion_claim_is_bound_to_current_sources_and_profile(model):
    import hashlib
    from grip_drive_control import script_manifest

    contract = json.loads((PACKAGE_ROOT / 'config' / f'{model}_asset_contract.json').read_text())
    qualification = contract['qualification']
    assert qualification['physical_motion_showcase'] == 'automated-passed'
    proof = qualification['physical_motion_proof']
    runner = PACKAGE_ROOT / 'scripts/run_physical_motion_showcase.py'
    profile = PACKAGE_ROOT / 'config/payload_retention_profiles.json'
    assert proof['runner_sha256'] == hashlib.sha256(runner.read_bytes()).hexdigest()
    digest = hashlib.sha256(json.dumps(script_manifest(runner), sort_keys=True,
                                      separators=(',', ':')).encode()).hexdigest()
    assert proof['runtime_manifest_sha256'] == digest
    assert proof['config_sha256'] == hashlib.sha256(profile.read_bytes()).hexdigest()
    assert proof['asset_manifest_sha256'] == qualification['robot_free_workpiece']['asset_manifest_sha256']
    settings = json.loads(profile.read_text())
    assert proof['payload_kg'] == settings['models'][model]['dynamic_qualification_payload_kg']
    assert proof['physics_hz'] == round(1 / settings['physics']['physics_dt_s'])
    assert proof['hold_control'] == ('preload-position' if model.startswith('2fg') else 'position')
    assert proof['external_forces_every_iteration'] is True
    assert proof['isaac_sim_version'] == '6.0.1'
    assert proof['replay'] is False
    assert len(proof['report_sha256']) == 64 and int(proof['report_sha256'], 16) >= 0
    assert 'not table pickup/setdown or hardware-force calibration' in proof['scope']


def test_2fg14_candidate_contract_keeps_force_claims_unavailable():
    """2FG14 geometry must not imply an uncalibrated public force state."""
    contract = json.loads(
        (PACKAGE_ROOT / 'config/2fg14_asset_contract.json').read_text(
            encoding='utf-8'))
    assert contract['model'] == '2fg14'
    assert contract['usd']['upper_limit_m'] == 0.025
    assert contract['ros']['task_maximum_m'] == 0.140
    assert contract['ros']['mechanism_maximum_m'] == 0.051
    assert contract['usd']['maximum_drive_force_n'] == 280.0
    assert contract['drive_tuning']['physx_single_drive_force_n'] == 560.0
    assert contract['drive_tuning']['nominal_grip_force_reference_n'] == 280.0
    assert 'physx_mimic_compensation' not in contract['drive_tuning']
    assert contract['force_model']['public_state'] == 'unavailable'
    assert contract['force_model'][
        'maximum_customer_realtime_grip_force_n'] == 196.0
    assert contract['force_model'][
        'realtime_force_limit_fraction_of_conventional'] == 0.7
    assert contract['force_model'][
        'maximum_customer_realtime_grip_force_n'] == pytest.approx(
            contract['usd']['maximum_drive_force_n'] *
            contract['force_model'][
                'realtime_force_limit_fraction_of_conventional'])
    assert contract['qualification']['force_mapping'] == 'pending'
    assert contract['drive_tuning']['qualified'] is False


def test_2fg7_contract_separates_grip_force_from_physx_drive_force():
    """The authored ceiling is distinct from a nominal grip-force rating."""
    contract = json.loads(
        (PACKAGE_ROOT / 'config/2fg7_asset_contract.json').read_text(
            encoding='utf-8'))
    tuning = contract['drive_tuning']
    assert contract['usd']['maximum_drive_force_n'] == 140.0
    assert tuning['nominal_grip_force_reference_n'] == 140.0
    assert 'physx_mimic_compensation' not in tuning
    assert tuning['physx_single_drive_force_n'] == 280.0
    assert 'uncalibrated' in tuning['force_mapping_basis']


def test_2fg_family_custom_finger_contract_uses_matching_anchors():
    """Both 2FG assets must expose the same model-relative USD hooks."""
    contracts = {
        model: json.loads(
            (PACKAGE_ROOT / f'config/{model}_asset_contract.json').read_text(
                encoding='utf-8'))
        for model in ('2fg7', '2fg14')
    }
    normalized = []
    for model, contract in contracts.items():
        hooks = dict(contract['custom_fingers'])
        normalized.append({
            key: value.replace(f'/onrobot_{model}', '/onrobot_2fg')
            if isinstance(value, str) else value
            for key, value in hooks.items()
        })
    assert normalized[0] == normalized[1]
    assert normalized[0]['geometry_and_physics_owner'] == 'user'


@pytest.mark.parametrize('model', ['2fg7', '2fg14'])
def test_dynamic_contact_reference_object_is_external_and_scope_bounded(model):
    """Each contact scene must model the supported external grip."""
    contract = json.loads(
        (PACKAGE_ROOT / f'config/{model}_asset_contract.json').read_text(
            encoding='utf-8'))
    config = json.loads(
        (PACKAGE_ROOT / contract['dynamic_contact'][
            'reference_object_config']).read_text(encoding='utf-8'))
    assert contract['runtime']['fidelity'] == 'kinematic'
    assert contract['dynamic_contact']['status'] == 'runtime-pending'
    assert config['fidelity_target'] == 'dynamic-contact'
    assert config['asset_revision'] == contract['asset_revision']
    assert config['reference_fingers']['qualification_scope'] == (
        'OnRobot-supplied reference fingers only')
    assert config['physics']['trials'] == 5
    assert [item['id'] for item in config['objects']] == (
        contract['dynamic_contact']['object_set'])
    reference_object = config['objects'][0]
    assert reference_object['grasp_kind'] == 'external-pinch'
    assert reference_object['grasp_target_m'] == 0.0
    assert reference_object['release_target_m'] == (
        contract['usd']['upper_limit_m'])
    assert reference_object['mass_kg'] > 0.0
    assert reference_object['approach_closing_step_m'] <= 0.0005
    assert reference_object['contact_closing_step_m'] <= 0.000025
    assert reference_object['preload_closing_step_m'] <= 0.00005
    assert reference_object['minimum_bilateral_contact_fraction'] >= 0.9
    placement = _load_validator()._validate_dynamic_contact_object_placement(
        config, reference_object)
    assert placement['base_clearance_m'] >= 0.005
    assert placement['contact_face_z_overlap_m'] >= 0.025
    assert placement['release_clearance_m'] >= 0.005
    assert placement['grasp_interference_m'] >= 0.005
    assert placement['lateral_overlap_m'] >= 0.01
    assert abs(placement['support_top_gap_m']) <= 0.0005
    assert placement['support_base_clearance_m'] >= 0.005
    assert placement['support_finger_clearance_m'] >= 0.005


def test_dynamic_contact_reference_object_rejects_base_overlap():
    """The scene must fail closed if its object intersects the base."""
    config = json.loads(
        (PACKAGE_ROOT / 'config' /
         '2fg7_dynamic_contact_reference_objects.json').read_text(
             encoding='utf-8'))
    reference_object = dict(config['objects'][0])
    reference_object['center_m'] = [0.0, 0.0, 0.115]
    with pytest.raises(AssertionError, match='gripper base'):
        _load_validator()._validate_dynamic_contact_object_placement(
            config, reference_object)


def test_2fg14_reference_fixture_does_not_reduce_geometric_preload():
    """The wider 2FG geometry must not receive a weaker contact fixture."""
    interference = {}
    for model in ('2fg7', '2fg14'):
        config = json.loads(
            (PACKAGE_ROOT / 'config' /
             f'{model}_dynamic_contact_reference_objects.json').read_text(
                 encoding='utf-8'))
        reference_object = config['objects'][0]
        interference[model] = (
            float(reference_object['size_m'][0]) / 2.0 -
            float(config['reference_geometry_m'][
                'contact_face_inner_x_at_zero']) -
            float(reference_object['grasp_target_m']))
    assert interference['2fg14'] >= interference['2fg7']


def test_2fg14_dynamic_contact_rejects_sloped_hold_contacts():
    """The standard vertical pads require predominantly horizontal normals."""
    config = json.loads(
        (PACKAGE_ROOT / 'config' /
         '2fg14_dynamic_contact_reference_objects.json').read_text(
             encoding='utf-8'))
    assert config['objects'][0][
        'maximum_mean_hold_contact_normal_vertical_component'] <= 0.15


def test_dynamic_contact_reference_object_rejects_bad_support():
    """The temporary support must meet the object without blocking fingers."""
    config = json.loads(
        (PACKAGE_ROOT / 'config' /
         '2fg14_dynamic_contact_reference_objects.json').read_text(
             encoding='utf-8'))
    reference_object = dict(config['objects'][0])
    reference_object['support_center_m'] = [0.0, 0.0, 0.12]
    with pytest.raises(AssertionError, match='does not meet'):
        _load_validator()._validate_dynamic_contact_object_placement(
            config, reference_object)


def test_retained_kinematic_evidence_matches_the_runtime_claim():
    """The kinematic claim must name complete passing evidence."""
    contract = json.loads(
        (PACKAGE_ROOT / 'config/2fg7_asset_contract.json').read_text(
            encoding='utf-8'))
    evidence_path = (
        PACKAGE_ROOT / contract['qualification']['kinematic_evidence'])
    evidence = json.loads(evidence_path.read_text(encoding='utf-8'))
    expected_cases = {
        'clock_namespace_and_live_state',
        'conventional_open_midpoint_close',
        'conventional_cancel_holds',
        'state_stream_outage_and_bridge_recovery',
        'switch_to_realtime_controller',
        'realtime_position',
        'realtime_velocity_endpoint_hold',
        'realtime_stale_command_watchdog',
        'force_remains_unavailable',
    }
    assert evidence['status'] == 'passed'
    assert evidence['fidelity'] == 'kinematic'
    assert evidence['asset_revision'] == contract['asset_revision']
    assert set(evidence['tests']) == expected_cases
    assert set(evidence['tests'].values()) == {'passed'}
    assert evidence['source_artifact']['sha256'] == (
        contract['qualification']['kinematic_source_evidence_sha256'])
    assert evidence['measurements']['force_valid'] is False


@pytest.mark.parametrize('model', ['2fg7', '2fg14'])
def test_dynamic_contact_rerun_is_required_for_current_drive(model):
    """Retuned 2FG drives must not promote the older contact evidence."""
    contract = json.loads(
        (PACKAGE_ROOT / f'config/{model}_asset_contract.json').read_text(
            encoding='utf-8'))
    assert contract['runtime']['fidelity'] == 'kinematic'
    assert contract['dynamic_contact']['status'] == 'runtime-pending'
    assert contract['qualification']['contact_and_grasp'] == (
        'runtime-pending-current-asset')
    assert contract['dynamic_contact']['required_rerun'] == (
        'dynamic-contact-current-asset')
    assert 'evidence' not in contract['dynamic_contact']
    assert 'source_evidence_sha256' not in contract['dynamic_contact']


def test_2fg14_hardware_in_loop_requires_current_asset_rerun():
    """2FG14 must not promote HIL evidence from older physics layers."""
    contract = json.loads(
        (PACKAGE_ROOT / 'config/2fg14_asset_contract.json').read_text(
            encoding='utf-8'))
    hil = contract['hardware_in_loop']
    assert hil['status'] == 'runtime-pending-current-asset'
    assert hil['realtime_position_motion'] == (
        'pending-firmware-identity-qualification')
    assert hil['required_rerun'] == ['conventional']
    assert 'evidence' not in hil
    assert 'source_evidence_sha256' not in hil
    assert contract['qualification'][
        'conventional_hardware_in_loop'] == (
            'runtime-pending-current-asset')
    assert contract['qualification'][
        'rviz_isaac_motion_agreement'] == 'runtime-pending-current-asset'


def test_rg2_hardware_in_loop_requires_current_asset_rerun():
    """RG2 must not promote HIL evidence from older physics layers."""
    contract = json.loads(
        (PACKAGE_ROOT / 'config/rg2_asset_contract.json').read_text(
            encoding='utf-8'))
    hil = contract['hardware_in_loop']
    assert hil['status'] == 'runtime-pending-current-asset'
    assert hil['realtime_position_motion'] == (
        'runtime-pending-current-asset')
    assert hil['required_rerun'] == ['conventional', 'realtime-position']
    assert 'evidence' not in hil
    assert 'realtime_position_evidence' not in hil
    assert contract['qualification'][
        'conventional_hardware_in_loop'] == (
            'runtime-pending-current-asset')
    assert contract['qualification']['realtime_hardware_in_loop'] == (
        'runtime-pending-current-asset')
    assert contract['qualification'][
        'rviz_isaac_motion_agreement'] == 'runtime-pending-current-asset'


def test_2fg7_xacro_selects_the_isaac_adapter_and_topics():
    """The release Xacro must wire the physical-joint transport exactly."""
    xacro = shutil.which('xacro')
    assert xacro is not None
    result = subprocess.run(
        [
            xacro,
            str(REPOSITORY_ROOT / 'onrobot_2fg7/urdf/'
                'realmesh_onrobot_2fg7.urdf.xacro'),
            'backend:=isaac',
            'isaac_joint_commands_topic:=sim/test_commands',
            'isaac_joint_states_topic:=sim/test_states',
            'isaac_state_timeout_ms:=321',
            'realtime_update_rate_hz:=100',
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    robot = result.stdout
    assert robot.count(
        '<plugin>onrobot_gripper_hardware/OnRobotIsaacSystem</plugin>') == 1
    assert 'OnRobotParallelGripperFakeSystem' not in robot
    assert 'OnRobotGripperSystem</plugin>' not in robot
    assert (
        '<param name="isaac_joint">finger_stroke</param>' in robot)
    assert (
        '<param name="isaac_joint_commands_topic">sim/test_commands</param>'
        in robot)
    assert (
        '<param name="isaac_joint_states_topic">sim/test_states</param>'
        in robot)
    assert '<param name="isaac_state_timeout_ms">321</param>' in robot
    assert '<param name="realtime_update_rate_hz">100</param>' in robot


def test_2fg14_xacro_separates_ros_task_and_isaac_articulation():
    """2FG14 Isaac import must contain only physical articulation joints."""
    xacro = shutil.which('xacro')
    assert xacro is not None
    result = subprocess.run(
        [
            xacro,
            str(REPOSITORY_ROOT / 'onrobot_2fg14/urdf/'
                'realmesh_onrobot_2fg14.urdf.xacro'),
            'backend:=isaac',
            'include_task_coordinate:=false',
            'include_ros2_control:=false',
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    robot = result.stdout
    assert '<joint name="finger_stroke" type="prismatic">' in robot
    assert '<joint name="left_finger_base_joint" type="prismatic">' in robot
    assert (
        '<mimic joint="finger_stroke" multiplier="1.0" offset="0"/>'
        in robot)
    assert 'grip_stroke' not in robot
    assert 'task_aperture_link' not in robot
    assert '<ros2_control' not in robot

    control_result = subprocess.run(
        [
            xacro,
            str(REPOSITORY_ROOT / 'onrobot_2fg14/urdf/'
                'realmesh_onrobot_2fg14.urdf.xacro'),
            'backend:=isaac',
            'isaac_joint_commands_topic:=sim/2fg14_commands',
            'isaac_joint_states_topic:=sim/2fg14_states',
            'isaac_state_timeout_ms:=432',
            'realtime_update_rate_hz:=100',
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    control_robot = control_result.stdout
    assert control_robot.count(
        '<plugin>onrobot_gripper_hardware/OnRobotIsaacSystem</plugin>') == 1
    assert '<param name="model">2fg14</param>' in control_robot
    assert '<param name="task_max_m">0.140</param>' in control_robot
    assert '<param name="physical_max_m">0.025</param>' in control_robot
    assert (
        '<param name="isaac_joint_commands_topic">'
        'sim/2fg14_commands</param>' in control_robot)
    assert (
        '<param name="isaac_joint_states_topic">'
        'sim/2fg14_states</param>' in control_robot)
    assert '<param name="isaac_state_timeout_ms">432</param>' in control_robot
    assert (
        '<param name="realtime_update_rate_hz">100</param>' in control_robot)


def test_rg2_xacro_selects_angular_isaac_adapter():
    """RG2 must retain a linear task API over its angular Isaac DOF."""
    xacro = shutil.which('xacro')
    assert xacro is not None
    result = subprocess.run(
        [
            xacro,
            str(REPOSITORY_ROOT / 'onrobot_rg2/urdf/'
                'realmesh_onrobot_rg2.urdf.xacro'),
            'backend:=isaac',
            'isaac_joint_commands_topic:=sim/rg2_commands',
            'isaac_joint_states_topic:=sim/rg2_states',
            'isaac_state_timeout_ms:=543',
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    robot = result.stdout
    assert robot.count(
        '<plugin>onrobot_gripper_hardware/OnRobotIsaacSystem</plugin>') == 1
    assert '<param name="model">rg2</param>' in robot
    assert '<param name="isaac_joint">finger_joint</param>' in robot
    assert '<param name="physical_max_m">1.22277767395</param>' in robot
    assert (
        '<param name="isaac_joint_commands_topic">sim/rg2_commands</param>'
        in robot)
    assert (
        '<param name="isaac_joint_states_topic">sim/rg2_states</param>'
        in robot)
    assert '<param name="isaac_state_timeout_ms">543</param>' in robot
    assert (
        '<command_interface name="realtime_mechanism_angular_velocity"/>'
        in robot)


def test_rg6_xacro_selects_angular_isaac_adapter():
    """RG6 must expose the same ROS/Isaac boundary with RG6 limits."""
    xacro = shutil.which('xacro')
    assert xacro is not None
    result = subprocess.run(
        [
            xacro,
            str(REPOSITORY_ROOT / 'onrobot_rg6/urdf/'
                'realmesh_onrobot_rg6.urdf.xacro'),
            'backend:=isaac',
            'isaac_joint_commands_topic:=sim/rg6_commands',
            'isaac_joint_states_topic:=sim/rg6_states',
            'isaac_state_timeout_ms:=654',
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    robot = result.stdout
    assert robot.count(
        '<plugin>onrobot_gripper_hardware/OnRobotIsaacSystem</plugin>') == 1
    assert '<param name="model">rg6</param>' in robot
    assert '<param name="isaac_joint">finger_joint</param>' in robot
    assert '<param name="physical_max_m">1.1928</param>' in robot
    assert '<param name="task_max_m">0.16</param>' in robot
    assert (
        '<param name="isaac_joint_commands_topic">sim/rg6_commands</param>'
        in robot)
    assert (
        '<param name="isaac_joint_states_topic">sim/rg6_states</param>'
        in robot)
    assert '<param name="isaac_state_timeout_ms">654</param>' in robot
    assert (
        '<command_interface name="realtime_mechanism_angular_velocity"/>'
        in robot)


def test_rg2_contract_rejects_filtered_tip_to_tip_contact(tmp_path):
    """The selected contact variant must not suppress fingertip collision."""
    package_root = tmp_path / 'onrobot_gripper_isaac'
    _copy_package(package_root)
    contract_path = package_root / 'config/rg2_asset_contract.json'
    contract = json.loads(contract_path.read_text(encoding='utf-8'))
    physics_relative = 'assets/rg2/payloads/Physics/physics.usda'
    physics_path = package_root / physics_relative
    physics = physics_path.read_text(encoding='utf-8')
    marker = 'over "right_finger_tip_link"'
    block_start = physics.index(marker)
    filtered_pairs = physics.index(
        'rel physics:filteredPairs = [', block_start)
    insertion = physics.index(']', filtered_pairs)
    physics = (
        physics[:insertion] +
        '                </onrobot_rg2/Geometry/left_finger_tip_link>,\n' +
        physics[insertion:])
    physics_path.write_text(physics, encoding='utf-8')
    validator = _load_validator()
    contract['layers'][physics_relative] = validator._sha256(physics_path)
    contract_path.write_text(
        json.dumps(contract, indent=2) + '\n', encoding='utf-8')

    result = subprocess.run(
        [
            sys.executable,
            str(package_root / 'scripts/validate_asset_contract.py'),
            '--package-root', str(package_root), '--model', 'rg2',
            '--model-root', str(REPOSITORY_ROOT / 'onrobot_rg2'),
            '--description-root',
            str(REPOSITORY_ROOT / 'onrobot_gripper_description'),
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert 'filters the required left-tip contact' in result.stderr

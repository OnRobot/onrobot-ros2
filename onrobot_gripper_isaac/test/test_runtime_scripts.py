"""Source-level contracts for scripts that require Isaac Sim at runtime."""

import ast
import importlib.util
import json
from pathlib import Path
import re
import stat
import sys
import xml.etree.ElementTree as ET

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / 'scripts'))
from isaac_model_contract import asset_repository_root
ASSET_REPOSITORY = asset_repository_root(PACKAGE_ROOT)


def _source(name):
    path = PACKAGE_ROOT / 'scripts' / name
    text = path.read_text(encoding='utf-8')
    ast.parse(text, filename=str(path))
    return text


def test_payload_initialization_commands_a_smooth_path_without_mimic_teleport():
    tree = ast.parse(_source('run_payload_retention_test.py'))
    helper = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                  and node.name == '_smooth_joint_targets')
    import math
    namespace = {'math': math}
    exec(compile(ast.Module(body=[helper], type_ignores=[]), '<joint-path>', 'exec'), namespace)
    generate = namespace['_smooth_joint_targets']
    for start, end in ((0., .85), (.85, .2), (.019, 0.)):
        targets = list(generate(start, end, 480))
        assert len(targets) == 480
        assert targets[-1] == pytest.approx(end)
        assert all(min(start, end) <= value <= max(start, end) for value in targets)
        assert all((b - a) * (end - start) >= 0 for a, b in zip(targets, targets[1:]))
        assert abs(targets[0] - start) < abs(end - start) / 480
        assert abs(targets[-1] - targets[-2]) < abs(end - start) / 480
    for endpoints, steps in (((math.nan, 1.), 10), ((0., 1.), 0)):
        with pytest.raises(ValueError):
            list(generate(*endpoints, steps))
    for name in ('run_payload_retention_test.py', 'run_physical_motion_showcase.py'):
        methods = [node.func.attr for node in ast.walk(ast.parse(_source(name)))
                   if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)]
        assert 'set_dof_positions' not in methods


def test_contact_observation_does_not_select_material_override():
    # Execute the actual material-selection conditions with only observation on.
    from types import SimpleNamespace
    tree = ast.parse(_source('run_payload_retention_test.py'))
    condition = next(node.test for node in ast.walk(tree) if isinstance(node, ast.If)
                     and isinstance(node.test, ast.BoolOp)
                     and ast.unparse(node.test) == 'args.apply_profile_physics_settings or args.reference_friction')
    args = SimpleNamespace(contact_observability=True, apply_profile_physics_settings=False,
                           reference_friction=False)
    code = compile(ast.Expression(condition), '<material-choice>', 'eval')
    assert not eval(code, {'args': args})
    args.reference_friction = True
    assert eval(code, {'args': args})


def test_contact_observation_accepts_runtime_vectors_and_rejects_other_bodies(monkeypatch):
    from array import array
    import math
    from types import SimpleNamespace
    tree = ast.parse(_source('run_payload_retention_test.py'))
    names = {'_finite_vector', '_vector_norm', '_contact_body_path', '_contact_observability_sample'}
    nodes = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
    namespace = {'math': math}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), '<contact-observer>', 'exec'), namespace)
    monkeypatch.setitem(sys.modules, 'pxr', SimpleNamespace(
        PhysicsSchemaTools=SimpleNamespace(intToSdfPath=lambda n: {17: '/cube', 18: '/tip'}[n])))
    contact = dict(body0=17, body1=18, normal=array('d', [1, 0, 0]),
                   impulse=array('d', [.5, .1, 0]))
    sensor = SimpleNamespace(get_data=lambda: dict(force=120., in_contact=True, contacts=[contact]))
    sample = namespace['_contact_observability_sample'](sensor, '/cube', .01)
    assert sample['normal_force_from_impulse_n'] == pytest.approx(50.)
    assert sample['object_contact_points'] == 1
    contact['body0'] = '/cube_other'
    sample = namespace['_contact_observability_sample'](sensor, '/cube', .01)
    assert sample['object_contact_points'] == 0
    for vector in ('123', {'a': 1, 'b': 2, 'c': 3}, [0, math.nan, 1], [0, 1]):
        assert not namespace['_finite_vector'](vector, 3)


@pytest.mark.parametrize('filename', [
    'run_payload_retention_test.py', 'run_physical_motion_showcase.py'])
def test_articulation_reacquisition_is_bounded(filename):
    """Execute the actual nested helper with a controlled tensor-view lifetime."""
    helper = next(node for node in ast.walk(ast.parse(_source(filename)))
                  if isinstance(node, ast.FunctionDef)
                  and node.name == 'current_articulation_view')
    factory = ast.parse('''
def make_view(articulation, Articulation):
    articulation_view_reacquires = 0
    mount_root_joint_path = '/root'
    runtime_articulation_path = '/root'
    return None
''').body[0]
    factory.body[-1:] = [helper, ast.Return(value=ast.Name(id=helper.name, ctx=ast.Load()))]
    namespace = {}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[factory], type_ignores=[])),
                 '<actual-view-reacquisition>', 'exec'), namespace)

    class View:
        def __init__(self, valid=True):
            self.valid = valid

        def is_physics_tensor_entity_valid(self):
            return self.valid

    calls = []

    def construct(path):
        calls.append(path)
        return View()

    initial = View()
    current = namespace['make_view'](initial, construct)
    assert current() is initial
    assert calls == []
    view = initial
    for _ in range(4):
        view.valid = False
        view = current()
        assert view.valid
    view.valid = False
    with pytest.raises(RuntimeError, match='repeatedly invalidated'):
        current()
    assert calls == ['/root'] * 5
    bad = namespace['make_view'](View(False), lambda path: View(False))
    with pytest.raises(RuntimeError, match='remained invalid'):
        bad()


def _load_runtime_compat():
    path = PACKAGE_ROOT / 'scripts/isaac_runtime_compat.py'
    spec = importlib.util.spec_from_file_location(
        'onrobot_isaac_runtime_compat', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_kinematic_qualification():
    path = PACKAGE_ROOT / 'scripts/run_kinematic_qualification.py'
    spec = importlib.util.spec_from_file_location(
        'onrobot_kinematic_qualification', path)
    module = importlib.util.module_from_spec(spec)
    scripts_path = str(path.parent)
    sys.path.insert(0, scripts_path)
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(scripts_path)
    return module


def _load_2fg14_authoring():
    path = PACKAGE_ROOT / 'scripts/author_2fg14_asset.py'
    spec = importlib.util.spec_from_file_location(
        'onrobot_2fg14_asset_authoring', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_rg6_authoring():
    path = PACKAGE_ROOT / 'scripts/author_rg6_asset.py'
    spec = importlib.util.spec_from_file_location(
        'onrobot_rg6_asset_authoring', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_payload_result_validator():
    path = PACKAGE_ROOT / 'scripts/validate_payload_retention_results.py'
    spec = importlib.util.spec_from_file_location(
        'onrobot_payload_result_validator', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_payload_matrix_runner():
    path = PACKAGE_ROOT / 'scripts/run_payload_retention_matrix.py'
    spec = importlib.util.spec_from_file_location(
        'onrobot_payload_matrix_runner', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_physical_motion_result_validator():
    path = PACKAGE_ROOT / 'scripts/validate_physical_motion_results.py'
    spec = importlib.util.spec_from_file_location(
        'onrobot_physical_motion_result_validator', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_physical_motion_matrix_runner():
    path = PACKAGE_ROOT / 'scripts/run_physical_motion_showcase_matrix.py'
    spec = importlib.util.spec_from_file_location(
        'onrobot_physical_motion_matrix_runner', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_showcase_branding():
    path = PACKAGE_ROOT / 'scripts/showcase_branding.py'
    spec = importlib.util.spec_from_file_location(
        'onrobot_showcase_branding', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_release_evidence_validator():
    path = PACKAGE_ROOT / 'scripts/validate_release_evidence.py'
    spec = importlib.util.spec_from_file_location(
        'onrobot_release_evidence_validator', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_showcase_sign_uses_approved_logo_and_payload_label(tmp_path):
    """The presentation sign must keep product and payload readable."""
    branding = _load_showcase_branding()
    logo = PACKAGE_ROOT / 'resource/branding/logo_onrobot_rgb.png'
    sign = branding.create_showcase_sign('2fg7', 7.0, logo)
    try:
        from PIL import Image

        image = Image.open(sign)
        assert image.size == (1200, 600)
        assert image.mode == 'RGB'
        assert image.getpixel((100, 590)) == (73, 157, 218)
        # The payload text must occupy useful space, not a tiny bitmap fallback.
        crop = image.crop((70, 455, 850, 570))
        assert sum(1 for pixel in crop.getdata() if max(pixel) < 200) > 500
    finally:
        sign.unlink(missing_ok=True)


def test_workpiece_mass_label_is_readable_and_mass_specific():
    branding = _load_showcase_branding()
    from PIL import Image

    labels = [branding.create_payload_label(value) for value in (.712, 2.)]
    try:
        assert labels[0] != labels[1]  # Avoid stale RTX textures after changing mass.
        for path in labels:
            with Image.open(path) as image:
                assert image.size == (640, 220)
                assert sum(1 for pixel in image.getdata() if max(pixel) < 100) > 1000
    finally:
        for path in labels:
            path.unlink()


def test_installed_python_entrypoints_are_executable_in_source():
    """PROGRAMS installed through symlinks must remain directly runnable."""
    entrypoints = (
        'author_2fg14_asset.py',
        'author_rg6_asset.py',
        'run_articulation_test.py',
        'run_hardware_in_loop.py',
        'run_ros_bridge.py',
        'run_rg2_tip_contact_test.py',
        'run_kinematic_qualification.py',
        'run_drive_step_response.py',
        'run_2fg_dynamic_contact_test.py',
        'run_physical_motion_showcase.py',
        'run_workpiece_showcase.py',
        'run_physical_motion_showcase_matrix.py',
        'run_payload_retention_matrix.py',
        'run_payload_retention_test.py',
        'validate_asset_contract.py',
        'validate_physical_motion_results.py',
        'validate_payload_retention_results.py',
    )
    for name in entrypoints:
        mode = (PACKAGE_ROOT / 'scripts' / name).stat().st_mode
        assert mode & stat.S_IXUSR, f'{name} is not executable'


def test_release_preflight_binds_candidate_inputs_and_generates_status(tmp_path):
    """Current candidate evidence cannot be substituted or edited silently."""
    validator = _load_release_evidence_validator()
    matrix = json.loads((PACKAGE_ROOT / 'config/release_support_matrix.json').read_text(
        encoding='utf-8'))
    assert matrix['profile'] == 'september-17-four-model-release'
    assert set(matrix['models']) == {'2fg7', '2fg14', 'rg2', 'rg6'}
    for specification in matrix['models'].values():
        assert (PACKAGE_ROOT / specification['modes'][0][
            'historical_report']).is_file()
    evidence = tmp_path / 'evidence'
    evidence.mkdir()
    candidate_prefix = tmp_path / 'candidate_install'
    candidate_lib = candidate_prefix / 'lib'
    candidate_lib.mkdir(parents=True)
    controller_plugin = candidate_lib / 'libonrobot_gripper_controller.so'
    hardware_plugin = candidate_lib / 'libonrobot_gripper_hardware.so'
    controller_plugin.write_bytes(b'candidate controller')
    hardware_plugin.write_bytes(b'candidate hardware')
    runner_hash = validator._sha256(
        PACKAGE_ROOT / 'scripts/run_kinematic_qualification.py')
    for model, specification in matrix['models'].items():
        mode = specification['modes'][0]
        manifest = validator._asset_manifest(ASSET_REPOSITORY / 'assets', model)
        cases = [
            {'name': name, 'status': 'passed'}
            for name in mode['required_tests']
        ]
        cases[0]['controller_plugin_libraries'] = [
            {'path': str(controller_plugin),
             'sha256': validator._sha256(controller_plugin)}]
        cases[0]['hardware_plugin_libraries'] = [
            {'path': str(hardware_plugin),
             'sha256': validator._sha256(hardware_plugin)}]
        cases[0]['package_prefixes'] = {
            'onrobot_gripper_controllers': str(candidate_prefix),
            'onrobot_gripper_hardware': str(candidate_prefix),
        }
        report = {
            'status': 'passed', 'model': model,
            'isaac_sim_version': mode['runtime']['isaac_sim_version'],
            'physics_backend': mode['runtime']['physics_backend'],
            'runner_sha256': runner_hash,
            'asset_files_sha256': manifest,
            'tests': cases,
        }
        (evidence / mode['candidate_report']).write_text(
            json.dumps(report), encoding='utf-8')

    report = validator.validate(
        PACKAGE_ROOT, PACKAGE_ROOT.parent,
        PACKAGE_ROOT.parent / 'onrobot_gripper_description', evidence,
        PACKAGE_ROOT / 'config/release_support_matrix.json')
    assert report['status'] == 'passed'
    assert all(item['status'] == 'passed' for item in report['models'])

    tampered = evidence / matrix['models']['rg2']['modes'][0]['candidate_report']
    data = json.loads(tampered.read_text(encoding='utf-8'))
    data['runner_sha256'] = '0' * 64
    tampered.write_text(json.dumps(data), encoding='utf-8')
    rejected = validator.validate(
        PACKAGE_ROOT, PACKAGE_ROOT.parent,
        PACKAGE_ROOT.parent / 'onrobot_gripper_description', evidence,
        PACKAGE_ROOT / 'config/release_support_matrix.json')
    assert rejected['status'] == 'blocked'
    rg2 = next(item for item in rejected['models'] if item['model'] == 'rg2')
    assert 'runner hash does not match' in rg2['modes'][0]['candidate']['errors'][0]

    data['runner_sha256'] = runner_hash
    data['tests'][0]['controller_plugin_libraries'][0]['sha256'] = 'a' * 64
    tampered.write_text(json.dumps(data), encoding='utf-8')
    rejected = validator.validate(
        PACKAGE_ROOT, PACKAGE_ROOT.parent,
        PACKAGE_ROOT.parent / 'onrobot_gripper_description', evidence,
        PACKAGE_ROOT / 'config/release_support_matrix.json')
    rg2 = next(item for item in rejected['models'] if item['model'] == 'rg2')
    assert 'loaded control identity' in rg2['modes'][0]['candidate']['errors'][0]


@pytest.mark.parametrize(
    ('mutation', 'expected'),
    (
        ('status', 'status is not passed'),
        ('model', 'model does not match'),
        ('runtime', 'runtime does not match'),
        ('asset', 'asset manifest does not match'),
        ('required_case', 'required cases are absent'),
        ('identity', 'loaded control identity is absent'),
    ),
)
def test_release_candidate_rejects_each_claim_binding(
        tmp_path, mutation, expected):
    """Every declared release binding needs an executable negative control."""
    validator = _load_release_evidence_validator()
    prefix = tmp_path / 'install'
    library = prefix / 'lib'
    library.mkdir(parents=True)
    controller = library / 'libonrobot_gripper_controller.so'
    hardware = library / 'libonrobot_gripper_hardware.so'
    controller.write_bytes(b'controller')
    hardware.write_bytes(b'hardware')
    case = {
        'name': 'required', 'status': 'passed',
        'controller_plugin_libraries': [{
            'path': str(controller), 'sha256': validator._sha256(controller)}],
        'hardware_plugin_libraries': [{
            'path': str(hardware), 'sha256': validator._sha256(hardware)}],
        'package_prefixes': {
            'onrobot_gripper_controllers': str(prefix),
            'onrobot_gripper_hardware': str(prefix),
        },
    }
    report = {
        'status': 'passed', 'model': '2fg7',
        'isaac_sim_version': '6.0.1', 'physics_backend': 'physx',
        'runner_sha256': 'a' * 64, 'asset_files_sha256': {'asset': 'b' * 64},
        'tests': [case],
    }
    specification = {
        'model': '2fg7',
        'runtime': {'isaac_sim_version': '6.0.1',
                    'physics_backend': 'physx'},
        'required_tests': ['required'],
        'require_loaded_control_identity': True,
    }
    if mutation == 'status':
        report['status'] = 'failed'
    elif mutation == 'model':
        report['model'] = 'rg2'
    elif mutation == 'runtime':
        report['isaac_sim_version'] = '5.0.0'
    elif mutation == 'asset':
        report['asset_files_sha256'] = {}
    elif mutation == 'required_case':
        case['status'] = 'failed'
    else:
        case['controller_plugin_libraries'][0]['path'] = str(hardware)
    errors = validator._validate_candidate(
        report, specification, {'asset': 'b' * 64}, 'a' * 64)
    assert any(expected in error for error in errors)


def test_conventional_completion_qualification_keeps_velocity_empty():
    """The normal action qualification must not fabricate a velocity limit."""
    text = _source('run_hardware_in_loop.py')
    assert "'--verify-conventional-completion'" in text
    assert "'ros-conventional-completion'" in text
    assert "'ros2-control-standard-action-empty-velocity'" in text
    assert "'conventional_empty_velocity_goal'" in text
    assert "'restore_after_conventional_empty_velocity_goal'" in text
    assert "'command_velocity': []" in text
    start_goal = text[text.index('    def _start_goal('):
                      text.index('    def cancel_goal_after_motion(')]
    assert 'goal.command.position = [position]' in start_goal
    assert 'goal.command.effort = [effort]' in start_goal
    assert 'goal.command.velocity' not in start_goal


def test_articulation_test_uses_current_isaac_api_and_all_endpoints():
    """The runtime harness must cover the full physical stroke."""
    text = _source('run_articulation_test.py')
    assert 'from isaacsim import SimulationApp' in text
    assert 'isaacsim.core.experimental.prims import Articulation' in text
    assert 'isaacsim.core.simulation_manager import SimulationManager' in text
    assert "('closed', lower)" in text
    assert "('midpoint', (lower + upper) / 2.0)" in text
    assert "('open', upper)" in text
    assert "report['isaac_sim_version'] != '6.0.1'" not in text
    assert 'setup_simulation(' in text
    assert 'play()' in text
    assert "'width': 1280" in text
    assert "'height': 720" in text
    assert "'renderer': None" not in text
    assert 'async def run(args):' in text
    assert 'root_path = articulation_path(contract)' in text
    assert 'Articulation(root_path)' in text
    assert "report['articulation_probe']" in text
    assert 'standard_articulation_roots' in text
    assert text.index('        timeline.play()') < text.index(
        'dof_names = articulation.dof_names')
    assert 'if not dof_names:' in text
    assert 'timeline.pause()' in text
    assert 'physics time differs from requested steps' in text
    assert 'close_app(app, exit_code)' in text
    assert 'args.output.write_text(' in text


def test_ros_bridge_matches_isaac_sim_6_graph_and_watchdog_contract():
    """The bridge must publish state and stop stale velocity targets."""
    text = _source('run_ros_bridge.py')
    required_nodes = (
        'isaacsim.ros2.bridge.ROS2Context',
        'isaacsim.ros2.bridge.ROS2PublishJointState',
        'isaacsim.sensors.physics.IsaacReadJointState',
        'isaacsim.ros2.bridge.ROS2PublishClock',
    )
    for node in required_nodes:
        assert node in text
    assert 'root_path = articulation_path(contract)' in text
    assert 'usdrt.Sdf.Path(root_path)' in text
    assert 'Articulation(root_path)' in text
    assert text.index('        play()') < text.index(
        'if joint_name not in articulation.dof_names')
    assert "clock_topic = 'clock'" in text
    assert "'physical_joint': joint_name" in text
    assert "parser.add_argument('--command-timeout'" in text
    assert 'time.monotonic() - last_velocity_command' in text
    assert "switch_dof_control_mode(\n                    'position'" in text
    assert 'set_dof_velocity_targets' in text
    assert 'rclpy.spin_once(command_node, timeout_sec=0.0)' in text
    assert "parser.add_argument(\n        '--gui', action='store_true'" in text
    assert "launch_config['renderer'] = 'RaytracedLighting'" in text
    assert "'headless': not args.gui" in text
    assert "'renderer': None" not in text
    assert 'ROS2SubscribeJointState' not in text
    assert 'IsaacArticulationController' not in text
    assert 'app.close(exit_code=exit_code)' in text


def test_hardware_in_loop_has_one_owner_and_explicit_motion_gate():
    """Isaac HIL must use ROS state/action without owning device transport."""
    text = _source('run_hardware_in_loop.py')
    assert "SUPPORTED_HIL_MODELS = ('2fg7', '2fg14', 'rg2', 'rg6')" in text
    assert 'ISAAC_HIL_MODELS = SUPPORTED_MODELS' in text
    assert "'rg2': 'finger_joint'" in text
    assert 'ParallelGripperCommand' in text
    assert "_fq(namespace, 'joint_states')" in text
    assert "_fq(namespace, 'gripper_state_broadcaster/state')" in text
    assert "_fq(namespace, 'parallel_gripper_limit_broadcaster/names')" in text
    assert (
        "_fq(namespace, 'parallel_gripper_limit_broadcaster/values')" in text)
    assert "_fq(namespace, 'gripper_controller/gripper_cmd')" in text
    assert "'--enable-hardware-motion'" in text
    assert "'--confirm-model'" in text
    assert "'--ros-preflight-only'" in text
    assert "'--verify-realtime-stop'" in text
    assert "'--verify-conventional-completion'" in text
    assert "'--verify-conventional-cancel'" in text
    assert "'--verify-conventional-preemption'" in text
    assert "'--verify-realtime-position'" in text
    assert "choices=('conventional', 'realtime-position')" in text
    assert 'args.confirm_model != args.model' in text
    assert '--ros-preflight-only does not command hardware' in text
    assert '--verify-realtime-stop requires --confirm-model matching' in text
    assert (
        'conventional action verification requires --confirm-model'
        in text)
    assert "'name': 'ros_hardware_preflight'" in text
    assert "args.hardware_control == 'conventional'" in text
    assert "args.hardware_control == 'realtime-position'" in text
    assert 'observation.action.server_is_ready()' in text
    assert text.index(
        'if (args.ros_preflight_only or args.verify_conventional_completion or'
    ) < text.index(
        'from isaacsim import SimulationApp')
    assert "'hardware_owner': 'external-real-backend-bringup'" in text
    assert "'command_authority': (" in text
    assert "'none-read-only'" in text
    assert "'ros2-control-stop-only'" in text
    assert "'ros2-control-standard-action-empty-velocity'" in text
    assert "'ros2-control-standard-action-bounded-cancel'" in text
    assert "'ros2-control-standard-action-bounded-preemption'" in text
    assert "'ros2-control-standard-action'" in text
    assert "'ros2-control-typed-realtime-topic'" in text
    assert "'ros2-control-typed-realtime-bounded-position'" in text
    assert 'articulation.set_dof_positions(' in text
    pump_end = text.index('        pump_callback = pump')
    pump_start = text.rindex('        def pump():', 0, pump_end)
    pump = text[pump_start:pump_end]
    assert pump.index('_mirror_measured_position(') < pump.index(
        'SimulationManager.step()')
    assert pump.index('SimulationManager.step()') < pump.index(
        'positions = articulation.get_dof_positions()')
    assert "report['shadow_mimic_dof_count']" in text
    assert '_excursion_target(' in text
    assert 'observation.task_limits is not None' in text
    assert "'name': 'bounded_hardware_motion'" in text
    assert "'name': 'task_to_mechanism_delta'" in text
    assert "'name': 'restore_valid_initial_aperture'" in text
    assert 'cancel_active_goal(' in text
    assert 'send_realtime_position(' in text
    assert 'RealtimePositionCommandStream' in text
    assert '_realtime_qualification_target(' in text
    assert '_validate_realtime_position_leg(' in text
    assert 'self.stop_realtime(pump)' in text
    assert "'name': 'realtime_stop'" in text
    assert "'newer_idle_state_observed': True" in text
    assert "'name': 'conventional_cancel_stop'" in text
    assert "'name': 'restore_after_conventional_cancel'" in text
    assert "'name': 'conventional_preemption_stop'" in text
    assert "'name': 'realtime_position_bounded_excursion'" in text
    assert "'name': 'restore_after_realtime_position'" in text
    assert "'name': 'realtime_stop_precondition'" in text
    assert 'command.mode = RealtimeCommand.POSITION' in text
    assert 'OnRobotGripperSystem' not in text
    assert 'modbus' not in text.lower()
    assert 'PublishClock' not in text


def test_2fg_realtime_exchange_rate_is_configurable_through_bringup():
    """Deployment must tune the Modbus cycle without editing descriptions."""
    bringup = (
        PACKAGE_ROOT.parent / 'onrobot_gripper_bringup/launch/'
        'gripper.launch.py').read_text(encoding='utf-8')
    two_fg7_launch = (
        PACKAGE_ROOT.parent / 'onrobot_2fg7/launch/control.launch.py'
        ).read_text(encoding='utf-8')
    two_fg14_launch = (
        PACKAGE_ROOT.parent / 'onrobot_2fg14/launch/control.launch.py'
        ).read_text(encoding='utf-8')

    assert "'realtime_update_rate_hz', default_value='auto'" in bringup
    assert "arguments['realtime_update_rate_hz']" in bringup
    assert "'realtime_update_rate_hz', default_value='50'" in (
        two_fg14_launch)
    assert "'realtime_update_rate_hz'," in two_fg7_launch
    assert "default_value='50'" in two_fg7_launch


def test_kinematic_qualification_is_repeatable_and_evidence_producing():
    """The kinematic runner must cover normal and failure-path behavior."""
    text = _source('run_kinematic_qualification.py')
    required_cases = (
        'clock_namespace_and_live_state',
        'conventional_open_midpoint_close',
        'conventional_cancel_holds',
        'state_stream_outage_and_bridge_recovery',
        'switch_to_realtime_controller',
        'realtime_position',
        'realtime_velocity_endpoint_hold',
        'realtime_stale_command_watchdog',
        'force_remains_unavailable',
    )
    for case in required_cases:
        assert case in text
    assert 'ParallelGripperCommand' in text
    assert 'ListControllers' in text
    assert 'SwitchController' in text
    assert 'wait_for_controller_states' in text
    assert 'controller readiness timed out' in text
    assert "'realtime_controller': 'inactive'" in text
    assert "'realtime_controller': 'active'" in text
    assert 'RealtimeCommand' in text
    assert 'self.physical_joint not in self.joint_state.name' in text
    assert 'expected_physical = math.asin(' in text
    assert 'command.mechanism_angular_velocity = velocity' in text
    assert (
        "_fq(namespace, 'gripper_state_broadcaster/state'),\n"
        '            self._gripper_state, qos_profile_sensor_data)' in text)
    assert 'os.killpg(process.pid, signal.SIGINT)' in text
    assert "f'{self.args.model}_kinematic_qualification.json'" in text
    assert "'fidelity': 'kinematic'" in text
    assert "'runner_sha256': _sha256(Path(__file__).resolve())" in text
    assert "self.report['asset_files_sha256'] = _asset_manifest" in text
    assert "self.report['bridge_sha256'] = _sha256" in text
    assert (
        "'rmw_implementation': rclpy.get_rmw_implementation_identifier()"
        in text)
    assert 'start_new_session=True' in text
    assert 'shell=True' not in text
    assert "'/clock continued after the bridge exited'" in text


def test_kinematic_qualification_hashes_plugins_mapped_by_descendants(
        tmp_path):
    """Plugin evidence must reach ros2_control_node below ros2 launch."""
    qualification = _load_kinematic_qualification()
    plugin = tmp_path / 'libonrobot_gripper_controller.so'
    plugin.write_bytes(b'current controller plugin')
    proc_root = tmp_path / 'proc'
    for pid in (100, 101, 102):
        (proc_root / str(pid) / 'task' / str(pid)).mkdir(parents=True)
        (proc_root / str(pid) / 'maps').write_text('', encoding='utf-8')
    (proc_root / '100/task/100/children').write_text(
        '101', encoding='utf-8')
    (proc_root / '101/task/101/children').write_text(
        '102', encoding='utf-8')
    (proc_root / '102/task/102/children').write_text('', encoding='utf-8')
    (proc_root / '102/maps').write_text(
        f'7f000000-7f001000 r-xp 00000000 00:00 0 {plugin}\n',
        encoding='utf-8')

    class FakeProcess:
        pid = 100

        @staticmethod
        def poll():
            return None

    identities = qualification._mapped_library_identity(
        FakeProcess(), plugin.name, proc_root)
    assert identities == [{
        'path': str(plugin),
        'sha256': qualification._sha256(plugin),
    }]


@pytest.mark.parametrize(
    ('model', 'physical_joint', 'velocity_field'),
    (('2fg7', 'finger_stroke', 'task_velocity'),
     ('2fg14', 'finger_stroke', 'task_velocity'),
     ('rg2', 'finger_joint', 'mechanism_angular_velocity'),
     ('rg6', 'finger_joint', 'mechanism_angular_velocity')))
def test_kinematic_qualification_stamps_realtime_commands_in_sim_time(
        monkeypatch, model, physical_joint, velocity_field):
    """The runner must not send wall-epoch stamps to a sim-time controller."""
    qualification = _load_kinematic_qualification()
    simulated_now_s = 12.25
    wall_now_s = 1788772952.25
    created_with_sim_time = False

    class FakeNow:
        def __init__(self, seconds):
            self.seconds = seconds

        def to_msg(self):
            message = qualification.RealtimeCommand().header.stamp
            message.sec = int(self.seconds)
            message.nanosec = int((self.seconds % 1.0) * 1e9)
            return message

    class FakePublisher:
        def __init__(self):
            self.accepted = []

        def get_subscription_count(self):
            return 1

        def publish(self, message):
            stamp = message.header.stamp
            source_s = stamp.sec + stamp.nanosec * 1e-9
            # This models the realtime controller's apply-time clock-domain
            # gate: a command from the future is never applied as motion.
            if 0.0 < source_s <= simulated_now_s:
                self.accepted.append(message)

    class FakeNode:
        def __init__(self, use_sim_time):
            self.use_sim_time = use_sim_time
            self.publisher = FakePublisher()

        def create_subscription(self, *_args, **_kwargs):
            return object()

        def create_publisher(self, *_args, **_kwargs):
            return self.publisher

        def create_client(self, *_args, **_kwargs):
            return object()

        def get_clock(self):
            seconds = simulated_now_s if self.use_sim_time else wall_now_s
            return type('FakeClock', (), {
                'now': lambda _self: FakeNow(seconds),
            })()

    def create_node(_name, *, parameter_overrides=None):
        nonlocal created_with_sim_time
        overrides = parameter_overrides or []
        created_with_sim_time = any(
            parameter.name == 'use_sim_time' and parameter.value is True
            for parameter in overrides)
        return FakeNode(created_with_sim_time)

    monkeypatch.setattr(qualification.rclpy, 'create_node', create_node)
    monkeypatch.setattr(
        qualification.rclpy, 'spin_once', lambda *_args, **_kwargs: None)
    monkeypatch.setattr(
        qualification, 'ActionClient', lambda *_args, **_kwargs: object())
    node = qualification.QualificationNode(
        'onrobot_kinematic', model, (0.0, 0.073), (0.0, 0.019),
        physical_joint)

    node.publish_realtime(
        qualification.RealtimeCommand.VELOCITY, 0.001, velocity=0.25)

    assert created_with_sim_time
    assert node.node.publisher.accepted
    assert getattr(node.node.publisher.accepted[-1], velocity_field) == 0.25


@pytest.mark.parametrize('model', ('2fg7', '2fg14', 'rg2', 'rg6'))
def test_kinematic_watchdog_case_requires_observed_motion(monkeypatch, model):
    """A stationary rejected stream must not count as a watchdog pass."""
    qualification = _load_kinematic_qualification()

    class FakeNode:
        def __init__(self):
            self.task_samples = []
            self.last_realtime_publication = None
            self.rg_model = model in ('rg2', 'rg6')

        def publish_realtime(self, *_args, **_kwargs):
            self.last_realtime_publication = {
                'active_observed': False,
            }

        def task_position(self):
            return 0.03

    workflow = object.__new__(qualification.KinematicQualificationWorkflow)
    workflow.args = type('Args', (), {'model': model})()
    workflow.task_min_m = 0.0
    workflow.task_max_m = {
        '2fg7': 0.073,
        '2fg14': 0.140,
        'rg2': 0.110,
        'rg6': 0.160,
    }[model]
    workflow.node = FakeNode()

    with pytest.raises(RuntimeError, match='motion was not observed'):
        workflow._realtime_watchdog()


def test_kinematic_failed_rt_case_keeps_clock_and_sequence_diagnostics():
    """A failed native rerun must retain evidence that narrows rejection."""
    qualification = _load_kinematic_qualification()

    class FakeNode:
        last_realtime_publication = {
            'published_command_count': 30,
            'first_source_stamp_s': 4.0,
            'last_source_stamp_s': 4.5,
        }

        def __init__(self):
            self.calls = 0

        def realtime_diagnostic_snapshot(self):
            self.calls += 1
            return {
            'runner_clock_s': 4.5,
            'steady_time_s': 9.0,
            'last_received_clock_s': 4.5,
                'realtime_state_stamp_s': 4.5,
                'realtime_state_rx_count': self.calls,
                'requested_command_sequence': 7,
                'applied_command_sequence': 7,
                'watchdog_stops': 0,
                'realtime_active': False,
                'active_mode': 0,
            'task_position_m': 0.03,
            'command_subscription_count': 1,
            'activation_observation': {
                'command_subscription_count': 1,
            },
            }

    workflow = object.__new__(qualification.KinematicQualificationWorkflow)
    workflow.node = FakeNode()
    workflow.report = {'tests': []}

    def fail():
        raise RuntimeError('no motion')

    assert not workflow.case('realtime_position', fail)
    diagnostics = workflow.report['tests'][0]['realtime_diagnostics']
    assert diagnostics['publication']['published_command_count'] == 30
    assert diagnostics['before']['runner_clock_s'] == 4.5
    assert diagnostics['after']['requested_command_sequence'] == 7
    assert diagnostics['after']['applied_command_sequence'] == 7
    assert diagnostics['after']['command_subscription_count'] == 1
    assert diagnostics['rejection_reason'] == (
        'no_new_hardware_sequence_observed')


def test_kinematic_dropout_requires_controller_readiness():
    """Never start the deliberate bridge outage from a partial ROS graph."""
    qualification = _load_kinematic_qualification()

    class FakeNode:
        def wait_for_controller_states(self, expected, timeout):
            assert expected == qualification._required_controller_states()
            assert timeout == 5.0
            raise RuntimeError('controller readiness disappeared')

    workflow = object.__new__(qualification.KinematicQualificationWorkflow)
    workflow.node = FakeNode()
    workflow.report = {}
    workflow.bridge_process = object()

    with pytest.raises(RuntimeError, match='controller readiness disappeared'):
        workflow._dropout_recovery()

    assert workflow.report == {}


def test_kinematic_run_stops_before_outage_after_required_case_failure():
    """Do not continue into recovery when conventional qualification fails."""
    text = _source('run_kinematic_qualification.py')
    assert (
        "if not self.case(\n"
        "                'conventional_open_midpoint_close', self._conventional):\n"
        "            return" in text)
    assert (
        "if not self.case(\n"
        "                'conventional_cancel_holds', self.node.cancel_and_check):\n"
        "            return" in text)
    assert "controller_states_before_dropout" in text


def test_dynamic_contact_test_is_repeated_bilateral_and_evidence_gated():
    """The dynamic contact harness must prove contact, hold, and release."""
    text = _source('run_2fg_dynamic_contact_test.py')
    required_checks = (
        'bilateral_object_contact_acquired',
        'bilateral_contact_maintained_during_preload',
        'object_blocks_target_before_endpoint',
        'bilateral_contact_maintained',
        'contact_force_finite_and_bounded',
        'hold_drop_bounded',
        'hold_lateral_drift_bounded',
        'hold_rotation_bounded',
        'hold_contact_normals_match_vertical_pinch_surfaces',
        'object_state_finite',
        'object_speed_bounded',
        'object_angular_speed_bounded',
        'object_displacement_bounded',
        'release_loses_contact',
        'release_target_reached',
        'inter_trial_reset_ready',
    )
    for check in required_checks:
        assert check in text
    assert 'contract = load_contract(args.model)' in text
    assert "'--model', choices=SUPPORTED_MODELS, required=True" in text
    assert 'root_path = articulation_path(contract)' in text
    assert "actuated_joint = contract['usd']['actuated_joint']" in text
    assert 'from isaacsim.sensors.experimental.physics import (' in text
    assert 'Contact, ContactSensor)' in text
    assert 'ContactSensor' in text
    assert 'CreateCollisionEnabledAttr().Set(False)' in text
    assert 'articulation.set_dof_positions(' in text
    assert 'collision_api.GetCollisionEnabledAttr().Set(True)' in text
    assert 'SUPPORT_PATH' in text
    assert 'support_collision_prim, False' in text
    assert 'approach_closing_step_m' in text
    assert 'contact_closing_step_m' in text
    assert 'preload_closing_step_m' in text
    assert 'preload_contact_force' in text
    assert 'hold_contact_force' in text
    assert 'estimated_minimum_normal_force_per_finger_n' in text
    assert 'DEFAULT_RESET_LIFT_STEPS = 120' in text
    assert 'RESET_MAXIMUM_OBJECT_STEP_M = 0.001' in text
    assert 'CreateKinematicEnabledAttr().Set(False)' in text
    assert 'PhysxSchema.PhysxMaterialAPI.Apply' in text
    assert "'friction_combine_mode', 'average'" in text
    assert 'def _add_box_mesh_collider(' in text
    assert 'mesh.CreateFaceVertexCountsAttr([4, 4, 4, 4, 4, 4])' in text
    assert "mesh_collision.CreateApproximationAttr().Set('convexHull')" in text
    assert 'support_collision_prim = _add_box_mesh_collider(' in text
    assert 'support_body = RigidPrim(SUPPORT_PATH)' in text
    assert 'SUPPORT_ANCHOR_PATH' in text
    assert 'SUPPORT_JOINT_PATH' in text
    assert 'UsdPhysics.PrismaticJoint.Define(' in text
    assert "support_joint.CreateAxisAttr('Z')" in text
    assert "support_drive.CreateTypeAttr('force')" in text
    assert 'support_drive.GetTargetPositionAttr().Set(offset)' in text
    assert '_lift_reset_fixture(' in text
    assert 'if trial_index == 1:' in text
    assert "'inter_trial_reset': pre_trial_reset" in text
    assert "'mode': 'feedback-bounded-prismatic-carrier-drive-lift'" in text
    assert "'maximum_object_step_m': maximum_object_step" in text
    assert 'RESET_MAXIMUM_CARRIER_OVERRUN_M = 0.01' in text
    assert 'support_nominal_target_position_m' in text
    assert 'support_end_position_m' in text
    assert 'while extension_steps < maximum_extension_steps:' in text
    assert "'failure_phase': 'pre-trial-reset'" in text
    assert "trial.get('failure_phase') == 'pre-trial-reset'" in text
    assert 'SUPPORT_DRIVE_STIFFNESS_N_M = 10000.0' in text
    assert 'SUPPORT_DRIVE_DAMPING_N_S_M = 100.0' in text
    assert 'SUPPORT_DRIVE_MAXIMUM_FORCE_N = 100.0' in text
    assert 'def _position_support(' in text
    assert 'set_camera_view = ViewportManager.set_camera_view' in text
    assert 'from isaacsim.core.utils.viewports import (' in text
    assert 'object_prim.set_enabled_gravities([True])' in text
    assert 'object_prim.set_enabled_gravities([False])' in text
    assert 'sensor.add_raw_contact_data_to_frame()' in text
    assert 'def _bind_collision_materials(' in text
    assert 'Usd.TraverseInstanceProxies()' in text
    assert 'prim.HasAPI(UsdPhysics.CollisionAPI)' in text
    assert 'if enabled is False:' in text
    assert 'while binding_prim.IsInstanceProxy():' in text
    assert "'binding_paths': sorted(bound_paths)" in text
    assert "report['fingertip_collision_material_bindings']" in text
    assert "'hold_contact_normal':" in text
    assert "'hold_contact_impulse':" in text
    assert 'def _contact_impulse_report(' in text
    assert "object_config['minimum_bilateral_contact_fraction']" in text
    assert "config['reference_geometry_m']" in text
    assert 'observed_contact_pairs' in text
    assert 'MAX_RECORDED_CONTACT_PAIRS = 16' in text
    assert 'HARNESS_REVISION = 9' in text
    assert "'harness_revision': HARNESS_REVISION" in text
    assert "report['status'] = 'running'" in text
    assert "report['completed_trials'] = len(report['tests'])" in text
    assert 'except KeyboardInterrupt:' in text
    assert "config['physics']['trials']" in text
    assert "'fidelity_target': 'dynamic-contact'" in text
    assert "report['status'] = (" in text
    assert 'close_app(app, exit_code)' in text
    assert 'from isaacsim.sensors.physics import ContactSensor' in text
    assert 'sensor.get_current_frame()' in text


def test_rg2_tip_contact_test_measures_linkage_displacement_during_contact():
    """The RG2 regression must prove contact and recovered link poses."""
    text = _source('run_rg2_tip_contact_test.py')
    assert "contract['usd']['mimic_joints']" in text
    assert "observation['maximum_linkage_error_rad']" in text
    assert 'args.maximum_contact_linkage_error_rad' in text
    assert "'minimum_leader_position_rad'" in text
    assert 'ContactSensor(Contact.create(' in text
    assert "'tip_contact_test_variant'" in text
    assert 'variant_set.SetVariantSelection(contact_variant)' in text
    assert 'contact_sensor.add_raw_contact_data_to_frame()' in text
    assert 'contact_frame.get' in text
    assert 'right_tip_path not in bodies' in text
    assert "'maximum_tip_contact_impulse_ns'" in text
    assert "'tip_to_tip_contact_observed': contact_observed" in text
    assert "'maximum_recovered_tip_position_error_m'" in text
    assert "'maximum_recovered_tip_orientation_error_rad'" in text
    assert "'recovered_dof_errors_rad'" in text
    assert "'recovered_root_position_error_m'" in text
    assert "'recovered_root_orientation_error_rad'" in text


def test_payload_retention_test_is_cross_model_and_evidence_gated():
    """The transport fixture must prove acquisition and retained payload."""
    text = _source('run_payload_retention_test.py')
    for required in (
            'choices=SUPPORTED_MODELS',
            "'sideways-external-pinch-payload-retention'",
            "choices=('rated-margin-static', 'dynamic-showcase')",
            "'joint_blocked_before_commanded_endpoint'",
            "'payload_relative_translation_bounded'",
            "'payload_relative_rotation_bounded'",
            "'payload_speed_bounded'",
            "'gripper_joint_stable'",
            "'gripper_linkage_stable'",
            "'articulation_and_payload_state_finite'",
            "report['asset_files_sha256'] = _asset_manifest(asset)",
            "'runner_sha256': hashlib.sha256(",
            "fixture['diagonal_hold_acceleration_m_s2']",
            "fixture['shake_acceleration_amplitude_m_s2']",
            "fixture['dynamic_lift_displacement_m']",
            "fixture['dynamic_shake_displacement_amplitude_m']",
            "if args.gui and args.scenario == 'dynamic-showcase'",
            "'--payload-kg'",
            "'--workpiece-size-m'",
            "'payload_size_source'",
            "'workpiece_definition'",
            "'--apply-profile-physics-settings'",
                "'--diagnostic-drive-max-force'",
                "'--contact-observability'",
                "'--recording-dir'",
            'renderer_capture.capture_next_frame_swapchain',
            "'evidence_role': 'visualization-only'",
            "'ffmpeg_command'",
            'block_transform = UsdGeom.XformCommonAPI(block_prim)',
            'stop()',
            "'configured-fraction-of-datasheet-maximum-force-fit-payload'",
            "'fingertip_friction_combine_mode'",
            "'fixture_fingertip_material_override'",
            "'fixture_solver_override'",
            "'effective_solver_settings'",
            'def current_articulation_view():',
            'is_physics_tensor_entity_valid',
            "'runtime_articulation_view_reacquires'",
            "'solve_articulation_contact_last'",
                "'maximum_absolute_projected_joint_force'",
                "'normal_force_from_impulse_n'",
                "'public_tool_api_force'",
            "'fingertip_orientations_wxyz'",
            'get_dof_projected_joint_forces',
            "'scope': 'reference-friction-observability'"):
        assert required in text
    assert 'args.use_asset_drive_settings' not in text
    assert text.count('args.apply_profile_physics_settings') == 4
    assert text.count('args.diagnostic_drive_max_force') >= 5

    config = json.loads((PACKAGE_ROOT / 'config' /
                         'payload_retention_profiles.json').read_text(
                             encoding='utf-8'))
    assert config['schema_version'] == 1
    assert set(config['models']) == {'2fg7', '2fg14', 'rg2', 'rg6'}
    assert config['fixture']['shake_cycles'] >= 1
    assert any(abs(value) > 0.0 for value in
               config['fixture']['diagonal_hold_acceleration_m_s2'])
    assert any(abs(value) > 0.0 for value in
               config['fixture']['shake_acceleration_amplitude_m_s2'])
    assert any(abs(value) > 0.0 for value in
               config['fixture']['dynamic_lift_displacement_m'])
    assert any(abs(value) > 0.0 for value in
               config['fixture']['dynamic_shake_displacement_amplitude_m'])
    for model, profile in config['models'].items():
        contract = json.loads((PACKAGE_ROOT / 'config' /
                               f'{model}_asset_contract.json').read_text(
                                   encoding='utf-8'))
        usd = contract['usd']
        lower = usd.get('lower_limit', usd.get('lower_limit_m'))
        upper = usd.get('upper_limit', usd.get('upper_limit_m'))
        assert (lower <= profile['grasp_target'] <
                profile['open_target'] <= upper)
        assert profile['minimum_blocking_error'] > 0.0
        assert profile['maximum_linkage_drift'] > 0.0
        assert profile['qualification_payload_kg'] == pytest.approx(
            profile['rated_force_fit_payload_kg'])
        assert profile['dynamic_qualification_payload_kg'] == pytest.approx(
            profile['rated_force_fit_payload_kg'])
        assert profile['payload_rating_source'].startswith('OnRobot ')
        assert profile['fixture_drive_stiffness'] > 0.0
        assert profile['fixture_drive_damping'] >= 0.0
        tuning = contract['drive_tuning']
        assert profile['fixture_drive_stiffness'] == pytest.approx(
            tuning['stiffness'])
        assert profile['fixture_drive_damping'] == pytest.approx(
            tuning['damping'])
        if 'fixture_drive_max_force' in profile:
            expected_drive_limit = tuning.get(
                'physx_single_drive_force_n',
                usd.get('maximum_drive_force_n',
                        usd.get('maximum_drive_torque_nm')))
            assert profile['fixture_drive_max_force'] == pytest.approx(
                expected_drive_limit)
        if 'fixture_solver_position_iterations' in profile:
            assert profile['fixture_solver_position_iterations'] >= 4
            assert profile['fixture_solver_velocity_iterations'] >= 1
        if 'collision_model' in contract:
            collision = contract['collision_model']
            assert profile['fingertip_friction_combine_mode'] == collision.get(
                'fingertip_friction_combine_mode', 'max')
            for kind in ('static', 'dynamic'):
                assert profile[f'fingertip_{kind}_friction'] == pytest.approx(
                    collision[f'fingertip_{kind}_friction'])
        assert len(profile['fingertip_links']) == 2


def test_physical_showcase_moves_every_supported_model_under_physx():
    """The showcase must use two-axis physical motion for all models."""
    text = _source('run_physical_motion_showcase.py')
    required = (
        "'physically-actuated-full-payload-lift-and-shake'",
        'choices=SUPPORTED_MODELS',
        'UsdPhysics.FixedJoint.Define(',
        'UsdPhysics.PrismaticJoint.Define(',
        'UsdPhysics.ArticulationRootAPI.Apply(mount_root_joint.GetPrim())',
        'articulation = Articulation(mount_root_joint_path)',
        'def current_articulation_view():',
        'is_physics_tensor_entity_valid',
        "carrier = RigidPrim(f'{root_prim_path}/Geometry/base_link')",
        "'source': 'physx-two-axis-prismatic-mount-drive'",
        "'replay': False",
        "'mount_physically_lifted'",
        "'payload_retained_during_physical_motion'",
        "'maximum_carrier_vertical_lift_m'",
        'LIFT_DURATION_S = 1.0',
        'BACKWARD_LIFT_DISTANCE_M = 0.040',
        'CAMERA_EYE_OFFSET_M = [0.50, -0.03, 0.045]',
        'CAMERA_TARGET_OFFSET_M = [0.0, 0.115, 0.045]',
        'CAMERA_FOCAL_LENGTH_MM = 22.0',
        "camera_path = f'{scene_root}/camera'",
        'camera = UsdGeom.Camera.Define(stage, camera_path)',
        'ViewportManager.set_camera(camera.GetPrim())',
        "'fixed_world_camera': True",
        "'motion_reference_marker_world_positions_m'",
        "'measured_keyframes'",
        'timeline.pause()',
        "'rendering_did_not_advance_physics'",
        "'manual_step_timing_error_s'",
        "'--screenshot-dir'",
        "'--recording-dir'",
        'renderer_capture.capture_next_frame_swapchain',
        "'source': 'same-live-physx-evidence-run'",
        "'runtime_articulation_view_reacquires'",
        "'ffmpeg_command'",
        'RECORDING_RATE_HZ = 30.0',
        "capture_keyframe('00_grasped')",
        "capture_keyframe('01_lifted')",
        "capture_keyframe('02_shake_upper_backward')",
        "capture_keyframe('03_shake_lower_forward')",
        'BACKWARD_SHAKE_AMPLITUDE_M = 0.025',
        'VERTICAL_SHAKE_AMPLITUDE_M = 0.010',
        'SHAKE_FREQUENCY_HZ = 2.0',
        'SHAKE_CYCLES = 5',
        'GUI_RENDER_RATE_HZ = 60.0',
        "'peak_commanded_shake_acceleration_m_s2'",
        'MAX_TRAJECTORY_SAMPLES = 8000',
        "'trajectory'",
        "'physics_overrides'",
        "'--apply-profile-physics-settings'",
        "'synthetic-data-experiment-only'",
        "'gui_render_stride'",
        "'lift_motion_axis_world_xyz'",
        "'shake_motion_axis_world_xyz': shake_motion_axis_world",
        "'mount_moved_diagonally_backward'",
        "'mount_shook_diagonally'",
        'maximum_lift_backward_travel >= 0.030',
        "'camera_exposes_diagonal_shake'",
        "'expected_horizontal_shake_span_m'",
        "'expected_vertical_shake_span_m'",
        "'maximum_carrier_backward_travel_m'",
        "'maximum_lift_backward_travel_m'",
        "'maximum_carrier_off_axis_error_m'",
        "'horizontal_shake_span_m'",
        "'vertical_shake_span_m'",
        "'diagonal_shake_span_m'",
        "'showcase_branding'",
        '_define_textured_sign(',
        'ONROBOT_BLUE',
    )
    for token in required:
        assert token in text
    assert text.count('block.set_world_poses(') == 1
    assert text.index('play()') < text.index(
        'for index in range(lift_steps):')
    assert text.index('for index in range(lift_steps):') < text.index(
        'for index in range(shake_steps):')
    assert ') if render_output else None)' in text
    assert 'physics_step_count % render_stride == 0' in text
    transition = text.index('block_kinematic.Set(False)')
    transition_step = text.index('SimulationManager.step()', transition)
    velocity_reset = text.index('block.set_velocities(', transition)
    assert transition < transition_step < velocity_reset


def test_drive_step_response_uses_exact_asset_drive_and_all_models():
    """The observational harness must step each product articulation."""
    text = _source('run_drive_step_response.py')
    config = json.loads(
        (PACKAGE_ROOT / 'config/drive_step_response_profiles.json').read_text(
            encoding='utf-8'))

    assert config['test'] == 'unloaded-drive-step-response'
    assert set(config['models']) == {'2fg7', '2fg14', 'rg2', 'rg6'}
    assert 'choices=SUPPORTED_MODELS' in text
    assert 'Articulation(articulation_path(contract))' in text
    assert 'set_dof_position_targets(' in text
    assert (repr('fidelity_claim') + ': ' + repr('observational-only')) in text
    assert "'asset_files_sha256'" in text
    assert "stage.GetPrimAtPath('/PhysicsScene')" in text
    assert 'UsdPhysics.Scene.Define(' in text
    assert 'fixture_drive' not in text
    assert 'UsdPhysics.PrismaticJoint.Define' not in text
    response_loop = text.split(
        'for _ in range(response_steps):', maxsplit=1)[1].split(
            'result = _measure_step', maxsplit=1)[0]
    assert 'SimulationManager.step()' in response_loop
    assert 'app.update()' not in response_loop


def test_physical_motion_matrix_runner_is_fixed_scope_and_shell_free(
        tmp_path):
    """The workstation helper may run only the four supported showcases."""
    runner = _load_physical_motion_matrix_runner()
    assert runner.MODELS == ('2fg7', '2fg14', 'rg2', 'rg6')
    command = runner._case_command(
        tmp_path / 'python.sh', tmp_path / 'runner.py', 'rg6',
        tmp_path / 'rg6.usda', tmp_path / 'config.json',
        tmp_path / 'rg6.json', True, tmp_path / 'screenshots',
        tmp_path / 'recording')
    assert command == [
        str(tmp_path / 'python.sh'), str(tmp_path / 'runner.py'),
        '--model', 'rg6',
        '--asset', str(tmp_path / 'rg6.usda'),
        '--config', str(tmp_path / 'config.json'),
        '--output', str(tmp_path / 'rg6.json'),
        '--gui',
        '--screenshot-dir', str(tmp_path / 'screenshots'),
        '--recording-dir', str(tmp_path / 'recording'),
    ]
    source = _source('run_physical_motion_showcase_matrix.py')
    assert 'shell=True' not in source
    assert 'subprocess.Popen' in source
    assert 'validate_physical_motion_results.py' in source
    assert 'Physical-motion runner:' in source
    assert "'runner_sha256': runner_sha256" in source
    assert "'asset_root': str(asset_root)" in source
    assert "'--record-frames'" in source
    assert "validation_command.append('--require-recording')" in source

    progress = tmp_path / 'motion_matrix_run.json'
    runner._write_summary(progress, {'status': 'running'})
    assert json.loads(progress.read_text(encoding='utf-8')) == {
        'status': 'running'}
    assert not progress.with_suffix('.json.tmp').exists()


def test_physical_motion_runner_serializes_media_relative_to_report(tmp_path):
    """Runner media serialization is portable and refuses outside paths."""
    tree = ast.parse(_source('run_physical_motion_showcase.py'))
    helpers = [node for node in tree.body if isinstance(node, ast.FunctionDef)
               and node.name in ('_portable_media_path',
                                 '_recording_ffmpeg_command')]
    namespace = {'Path': Path, 'RuntimeError': RuntimeError}
    import shlex
    namespace['shlex'] = shlex
    module = ast.Module(body=helpers, type_ignores=[])
    exec(compile(module, '<runner-serialization>', 'exec'), namespace)

    report = tmp_path / 'results/rg6.json'
    recording = report.parent / 'rg6_recording'
    recording.mkdir(parents=True)
    portable = namespace['_portable_media_path']
    assert portable(recording / 'frame_00000.png', report) == (
        'rg6_recording/frame_00000.png')
    assert namespace['_recording_ffmpeg_command'](recording, report) == (
        'ffmpeg -y -framerate 30 -i '
        'rg6_recording/frame_%05d.png -c:v libx264 -pix_fmt yuv420p -crf 18 '
        'rg6_recording/showcase.mp4')
    with pytest.raises(RuntimeError, match='must be below report directory'):
        portable(tmp_path / 'outside/frame.png', report)


def _write_runtime_fixture_dependencies(runner):
    for name in ('grip_drive_control.py', 'run_payload_retention_test.py',
                 'isaac_model_contract.py', 'isaac_runtime_compat.py', 'showcase_branding.py'):
        path = runner.parent / name
        if not path.exists():
            path.write_text('# fixture dependency\n')


def test_physical_motion_validator_accepts_exact_matrix_and_rejects_replay(
        tmp_path, monkeypatch, capsys):
    """Accept four current live-motion reports and reject replay evidence."""
    validator = _load_physical_motion_result_validator()
    package_root = tmp_path / 'package'
    results = tmp_path / 'results'
    config = package_root / 'config/payload_retention_profiles.json'
    runner = package_root / 'scripts/run_physical_motion_showcase.py'
    config.parent.mkdir(parents=True)
    runner.parent.mkdir(parents=True)
    results.mkdir()
    config.write_text('{"schema_version": 1}\n', encoding='utf-8')
    runner.write_text('# runner\n', encoding='utf-8')
    _write_runtime_fixture_dependencies(runner)
    config_digest = validator._sha256(config)
    runner_digest = validator._sha256(runner)

    for model in validator.MODELS:
        asset = package_root / 'assets' / model / f'onrobot_{model}.usda'
        asset.parent.mkdir(parents=True)
        asset.write_text(f'#usda 1.0\n# {model}\n', encoding='utf-8')
        manifest = validator._manifest(package_root / 'assets', model)
        result = {
            'schema_version': 1,
            'harness_revision': validator.MINIMUM_HARNESS_REVISION,
            'test': validator.TEST_NAME,
            'model': model,
            'status': 'passed',
            'config_sha256': config_digest,
            'runner_sha256': runner_digest,
            'runtime_script_files_sha256': validator.script_manifest(runner),
            'isaac_sim_version': '6.0.1',
            'payload_mass_kg': 1.0,
            'rated_force_fit_payload_kg': 1.0,
            'payload_fraction_of_rating': 1.0,
            'asset_files_sha256': manifest,
            'physics_scene_settings': {'solver_type': 'TGS', 'external_forces_every_iteration': True},
            'motion': {
                'source': 'physx-two-axis-prismatic-mount-drive',
                'replay': False,
                'actual_physics_dt_s': 1.0 / 240.0,
                'manual_step_timing_error_s': 0.0,
                'measured_keyframes': [
                    {
                        'name': '00_grasped',
                        'carrier_delta_from_grasp_m': None,
                    },
                    {
                        'name': '01_lifted',
                        'carrier_delta_from_grasp_m': [0.0, 0.040, 0.075],
                    },
                    {
                        'name': '02_shake_upper_backward',
                        'carrier_delta_from_grasp_m': [0.0, 0.065, 0.085],
                    },
                    {
                        'name': '03_shake_lower_forward',
                        'carrier_delta_from_grasp_m': [0.0, 0.015, 0.065],
                    },
                ],
                'camera': {
                    'fixed_world_camera': True,
                    'level_view': True,
                },
                'screenshots': [],
            },
            'tests': [{
                'status': 'passed',
                'checks': {'held': True, 'diagonal': True},
                'measurements': {
                    'horizontal_shake_span_m': 0.050,
                    'vertical_shake_span_m': 0.020,
                    'diagonal_shake_span_m': 0.054,
                    'maximum_lift_backward_travel_m': 0.040,
                    'maximum_carrier_vertical_lift_m': 0.075,
                },
            }],
        }
        (results / f'{model}.json').write_text(
            json.dumps(result), encoding='utf-8')

    arguments = [
        'validate_physical_motion_results.py',
        '--results-dir', str(results),
        '--package-root', str(package_root),
        '--isaac-version', '6.0.1',
    ]
    monkeypatch.setattr(sys, 'argv', arguments)
    assert validator.main() == 0
    assert json.loads(capsys.readouterr().out)['status'] == 'passed'

    dependency = runner.parent / 'grip_drive_control.py'
    saved_dependency = dependency.read_bytes()
    dependency.write_text('# changed hold implementation\n')
    monkeypatch.setattr(sys, 'argv', arguments)
    assert validator.main() == 1
    rejected_dependencies = json.loads(capsys.readouterr().out)
    assert all(any('runtime_script_files_sha256:' in e for e in case['errors'])
               for case in rejected_dependencies['cases'])
    dependency.write_bytes(saved_dependency)

    failed_path = results / 'rg2.json'
    failed = json.loads(failed_path.read_text(encoding='utf-8'))
    failed['motion']['replay'] = True
    failed_path.write_text(json.dumps(failed), encoding='utf-8')
    monkeypatch.setattr(sys, 'argv', arguments)
    assert validator.main() == 1
    rejected = json.loads(capsys.readouterr().out)
    rg2 = next(case for case in rejected['cases']
               if case['model'] == 'rg2')
    assert 'motion must be live PhysX, not replay' in rg2['errors']


def test_physical_motion_validator_requires_gapless_live_recording(
        tmp_path, monkeypatch, capsys):
    """Requested video evidence must be live, complete, and sequential."""
    validator = _load_physical_motion_result_validator()
    package_root = tmp_path / 'package'
    results = tmp_path / 'results'
    config = package_root / 'config/payload_retention_profiles.json'
    runner = package_root / 'scripts/run_physical_motion_showcase.py'
    config.parent.mkdir(parents=True)
    runner.parent.mkdir(parents=True)
    results.mkdir()
    config.write_text('{"schema_version": 1}\n', encoding='utf-8')
    runner.write_text('# runner\n', encoding='utf-8')
    _write_runtime_fixture_dependencies(runner)

    for model in validator.MODELS:
        asset = package_root / 'assets' / model / f'onrobot_{model}.usda'
        asset.parent.mkdir(parents=True)
        asset.write_text('#usda 1.0\n', encoding='utf-8')
        recording_dir = results / f'{model}_recording'
        recording_dir.mkdir()
        for index in range(30):
            (recording_dir / f'frame_{index:05d}.png').write_bytes(b'png')
        screenshot_dir = results / f'{model}_screenshots'
        screenshot_dir.mkdir()
        for name in validator.EXPECTED_SCREENSHOTS:
            (screenshot_dir / name).write_bytes(b'png')
        data = {
            'schema_version': 1,
            'harness_revision': validator.MINIMUM_HARNESS_REVISION,
            'test': validator.TEST_NAME,
            'model': model,
            'status': 'passed',
            'config_sha256': validator._sha256(config),
            'runner_sha256': validator._sha256(runner),
            'runtime_script_files_sha256': validator.script_manifest(runner),
            'isaac_sim_version': '6.0.1',
            'payload_mass_kg': 1.0,
            'rated_force_fit_payload_kg': 1.0,
            'payload_fraction_of_rating': 1.0,
            'asset_files_sha256': validator._manifest(
                package_root / 'assets', model),
            'physics_scene_settings': {'solver_type': 'TGS', 'external_forces_every_iteration': True},
            'motion': {
                'source': 'physx-two-axis-prismatic-mount-drive',
                'replay': False,
                'actual_physics_dt_s': 1.0 / 240.0,
                'manual_step_timing_error_s': 0.0,
                'measured_keyframes': [
                    {'name': '00_grasped',
                     'carrier_delta_from_grasp_m': None},
                    {'name': '01_lifted',
                     'carrier_delta_from_grasp_m': [0.0, 0.04, 0.075]},
                    {'name': '02_shake_upper_backward',
                     'carrier_delta_from_grasp_m': [0.0, 0.065, 0.085]},
                    {'name': '03_shake_lower_forward',
                     'carrier_delta_from_grasp_m': [0.0, 0.015, 0.065]},
                ],
                'camera': {
                    'fixed_world_camera': True,
                    'level_view': True,
                },
                'screenshots': [
                    f'{model}_screenshots/{name}'
                    for name in sorted(validator.EXPECTED_SCREENSHOTS)],
                'recording': {
                    'enabled': True,
                    'source': 'same-live-physx-evidence-run',
                    'replay': False,
                    'effective_frame_rate_hz': 30.0,
                    'frame_count': 30,
                    'first_simulation_time_s': 1.0,
                    'last_simulation_time_s': 2.0,
                    # New reports keep media portable by resolving relative
                    # to the result directory at validation time.
                    'directory': f'{model}_recording',
                    'ffmpeg_command': 'ffmpeg example',
                },
            },
            'tests': [{
                'status': 'passed',
                'checks': {'held': True},
                'measurements': {
                    'horizontal_shake_span_m': 0.050,
                    'vertical_shake_span_m': 0.020,
                    'diagonal_shake_span_m': 0.054,
                    'maximum_lift_backward_travel_m': 0.040,
                    'maximum_carrier_vertical_lift_m': 0.075,
                },
            }],
        }
        (results / f'{model}.json').write_text(
            json.dumps(data), encoding='utf-8')

    arguments = [
        'validate_physical_motion_results.py',
        '--results-dir', str(results),
        '--package-root', str(package_root),
        '--isaac-version', '6.0.1',
        '--require-screenshots',
        '--require-recording',
    ]
    monkeypatch.setattr(sys, 'argv', arguments)
    assert validator.main() == 0
    assert json.loads(capsys.readouterr().out)['status'] == 'passed'

    # A verdict destination must not alias an input, including through a
    # symlink. The protected bytes remain unchanged after refusal.
    config_bytes = config.read_bytes()
    monkeypatch.setattr(sys, 'argv', arguments + ['--output', str(config)])
    assert validator.main() == 1
    rejected = json.loads(capsys.readouterr().out)
    assert 'output path aliases a protected input' in rejected['errors'][0]
    assert config.read_bytes() == config_bytes
    output_link = tmp_path / 'validation-output-link.json'
    output_link.symlink_to(runner)
    runner_bytes = runner.read_bytes()
    monkeypatch.setattr(sys, 'argv', arguments + [
        '--output', str(output_link)])
    assert validator.main() == 1
    rejected = json.loads(capsys.readouterr().out)
    assert 'output path aliases a protected input' in rejected['errors'][0]
    assert runner.read_bytes() == runner_bytes
    result_input = results / '2fg7.json'
    result_bytes = result_input.read_bytes()
    monkeypatch.setattr(sys, 'argv', arguments + [
        '--output', str(result_input)])
    assert validator.main() == 1
    rejected = json.loads(capsys.readouterr().out)
    assert 'output path aliases a protected input' in rejected['errors'][0]
    assert result_input.read_bytes() == result_bytes

    # Digest mismatches remain promotion failures even when every media file
    # is present. Exercise each current-input dependency independently.
    for changed_path, expected_error in (
            (config, 'config_sha256:'),
            (runner, 'runner_sha256:')):
        original_bytes = changed_path.read_bytes()
        changed_path.write_bytes(original_bytes + b'changed\n')
        monkeypatch.setattr(sys, 'argv', arguments)
        assert validator.main() == 1
        rejected = json.loads(capsys.readouterr().out)
        rg6 = next(case for case in rejected['cases']
                   if case['model'] == 'rg6')
        assert any(expected_error in value for value in rg6['errors'])
        changed_path.write_bytes(original_bytes)

    asset = package_root / 'assets/rg6/onrobot_rg6.usda'
    original_bytes = asset.read_bytes()
    asset.write_bytes(original_bytes + b'changed\n')
    monkeypatch.setattr(sys, 'argv', arguments)
    assert validator.main() == 1
    rejected = json.loads(capsys.readouterr().out)
    rg6 = next(case for case in rejected['cases'] if case['model'] == 'rg6')
    assert 'asset manifest differs from the current package' in rg6['errors']
    asset.write_bytes(original_bytes)

    # Archived reports may retain their producer's absolute paths only when
    # an explicit original-root mapping is provided. The source report is
    # restored after each mutation: validation never rewrites it.
    archived = results / 'rg6.json'
    original = json.loads(archived.read_text(encoding='utf-8'))
    archive_root = Path('/archived/work/isaac-results/branded-showcase')
    archived_data = json.loads(json.dumps(original))
    archived_data['motion']['screenshots'] = [
        str(archive_root / value)
        for value in original['motion']['screenshots']]
    archived_data['motion']['recording']['directory'] = str(
        archive_root / 'rg6_recording')
    archived.write_text(json.dumps(archived_data), encoding='utf-8')
    monkeypatch.setattr(sys, 'argv', arguments)
    assert validator.main() == 1
    rejected = json.loads(capsys.readouterr().out)
    rg6 = next(case for case in rejected['cases'] if case['model'] == 'rg6')
    assert any('requires an explicit --report-root mapping' in value
               for value in rg6['errors'])
    monkeypatch.setattr(sys, 'argv', arguments + [
        '--report-root', str(archive_root)])
    assert validator.main() == 0
    assert json.loads(capsys.readouterr().out)['status'] == 'passed'
    archived.write_text(json.dumps(original), encoding='utf-8')

    # Relative traversal and a symlink to an outside directory are rejected
    # before media existence checks can read anything outside the bundle.
    traversal = json.loads(json.dumps(original))
    traversal['motion']['screenshots'][0] = '../outside.png'
    archived.write_text(json.dumps(traversal), encoding='utf-8')
    monkeypatch.setattr(sys, 'argv', arguments)
    assert validator.main() == 1
    rejected = json.loads(capsys.readouterr().out)
    rg6 = next(case for case in rejected['cases'] if case['model'] == 'rg6')
    assert any('escapes report root' in value for value in rg6['errors'])
    archived.write_text(json.dumps(original), encoding='utf-8')

    outside = tmp_path / 'outside-recording'
    outside.mkdir()
    link = results / 'rg6_recording_escape'
    link.symlink_to(outside, target_is_directory=True)
    symlink_escape = json.loads(json.dumps(original))
    symlink_escape['motion']['recording']['directory'] = (
        'rg6_recording_escape')
    archived.write_text(json.dumps(symlink_escape), encoding='utf-8')
    monkeypatch.setattr(sys, 'argv', arguments)
    assert validator.main() == 1
    rejected = json.loads(capsys.readouterr().out)
    rg6 = next(case for case in rejected['cases'] if case['model'] == 'rg6')
    assert any('escapes report root' in value for value in rg6['errors'])
    archived.write_text(json.dumps(original), encoding='utf-8')

    loop = results / 'rg6_recording_loop'
    loop.symlink_to(loop.name)
    looped = json.loads(json.dumps(original))
    looped['motion']['recording']['directory'] = 'rg6_recording_loop'
    archived.write_text(json.dumps(looped), encoding='utf-8')
    monkeypatch.setattr(sys, 'argv', arguments)
    assert validator.main() == 1
    rejected = json.loads(capsys.readouterr().out)
    rg6 = next(case for case in rejected['cases'] if case['model'] == 'rg6')
    assert any('cannot resolve path safely' in value for value in rg6['errors'])
    archived.write_text(json.dumps(original), encoding='utf-8')

    missing = results / 'rg6_recording/frame_00017.png'
    missing.unlink()
    monkeypatch.setattr(sys, 'argv', arguments)
    assert validator.main() == 1
    rejected = json.loads(capsys.readouterr().out)
    rg6 = next(case for case in rejected['cases']
               if case['model'] == 'rg6')
    assert any('recording frames are missing' in value
               for value in rg6['errors'])


def test_payload_result_validator_requires_exact_current_asset_evidence():
    """Matrix promotion must reject stale, overridden, or partial results."""
    text = _source('validate_payload_retention_results.py')
    for required in (
            "MODELS = ('2fg7', '2fg14', 'rg2', 'rg6')",
            "SCENARIOS = ('rated-margin-static', 'dynamic-showcase')",
            'MINIMUM_HARNESS_REVISION = 14',
            "'fixture_fingertip_material_override'",
            "'fixture_drive_override'",
            "'fixture_solver_override'",
            "'asset manifest differs from the current package'",
            "'payload fraction of rating is not 1.0'",
            "'runner_sha256': runner_sha256",
            "'config_sha256': config_sha256"):
        assert required in text


def test_payload_runner_has_an_embedded_server_mode_without_closing_kit():
    """Remote execution must reuse the server's Kit application."""
    text = _source('run_payload_retention_test.py')
    assert 'class _EmbeddedSimulationApp' in text
    assert "omni.kit.app.get_app().update()" in text
    assert "parser.add_argument(\n        '--embedded'" in text
    assert "args.embedded and args.gui" in text


def test_payload_matrix_runner_is_fixed_scope_and_shell_free(tmp_path):
    """The laptop-side helper may run only the declared qualification cases."""
    runner = _load_payload_matrix_runner()
    assert runner.MODELS == ('2fg7', '2fg14', 'rg2', 'rg6')
    assert runner.SCENARIOS == (
        'rated-margin-static', 'dynamic-showcase')
    command = runner._case_command(
        tmp_path / 'python.sh', tmp_path / 'runner.py', 'rg2',
        'dynamic-showcase', tmp_path / 'rg2.usda', tmp_path / 'config.json',
        tmp_path / 'result.json')
    assert command == [
        str(tmp_path / 'python.sh'), str(tmp_path / 'runner.py'),
        '--model', 'rg2',
        '--scenario', 'dynamic-showcase',
        '--asset', str(tmp_path / 'rg2.usda'),
        '--config', str(tmp_path / 'config.json'),
        '--output', str(tmp_path / 'result.json'),
    ]
    source = _source('run_payload_retention_matrix.py')
    assert 'shell=True' not in source
    assert 'subprocess.Popen' in source
    assert 'validate_payload_retention_results.py' in source
    assert 'Payload-retention runner:' in source
    assert "'runner_sha256': runner_sha256" in source
    assert "'asset_root': str(asset_root)" in source

    progress = tmp_path / 'matrix_run.json'
    runner._write_summary(progress, {'status': 'running'})
    assert json.loads(progress.read_text(encoding='utf-8')) == {
        'status': 'running'}
    assert not progress.with_suffix('.json.tmp').exists()


def test_payload_result_validator_accepts_complete_matrix_and_rejects_override(
        tmp_path, monkeypatch, capsys):
    """Accept a complete matrix and reject a stage-local override."""
    validator = _load_payload_result_validator()
    package_root = tmp_path / 'package'
    results = tmp_path / 'results'
    config = package_root / 'config/payload_retention_profiles.json'
    runner = package_root / 'scripts/run_payload_retention_test.py'
    config.parent.mkdir(parents=True)
    runner.parent.mkdir(parents=True)
    results.mkdir()
    config.write_text('{"schema_version": 1}\n', encoding='utf-8')
    runner.write_text('# runner\n', encoding='utf-8')
    _write_runtime_fixture_dependencies(runner)
    config_digest = validator._sha256(config)
    runner_digest = validator._sha256(runner)

    for model in validator.MODELS:
        asset = package_root / 'assets' / model / f'onrobot_{model}.usda'
        asset.parent.mkdir(parents=True)
        asset.write_text(f'#usda 1.0\n# {model}\n', encoding='utf-8')
        manifest = validator._manifest(package_root / 'assets', model)
        for scenario in validator.SCENARIOS:
            result = {
                'schema_version': 1,
                'harness_revision': validator.MINIMUM_HARNESS_REVISION,
                'test': validator.TEST_NAME,
                'model': model,
                'scenario': scenario,
                'status': 'passed',
                'config_sha256': config_digest,
                'runner_sha256': runner_digest,
                'runtime_script_files_sha256': validator.script_manifest(runner),
                'isaac_sim_version': '6.0.1',
                'payload_mass_kg': 1.0,
                'rated_force_fit_payload_kg': 1.0,
                'payload_fraction_of_rating': 1.0,
                'fixture_fingertip_material_override': None,
                'fixture_drive_override': None,
                'fixture_solver_override': None,
                'effective_solver_settings': {
                    'solver_type': 'TGS',
                    'external_forces_every_iteration': True,
                    'position_iterations': 64,
                    'velocity_iterations': 4,
                    'solve_articulation_contact_last': model == 'rg6',
                },
                'asset_files_sha256': manifest,
                'tests': [{'status': 'passed', 'checks': {'held': True}}],
            }
            (results / f'{model}_{scenario}.json').write_text(
                json.dumps(result), encoding='utf-8')

    arguments = [
        'validate_payload_retention_results.py',
        '--results-dir', str(results),
        '--package-root', str(package_root),
        '--isaac-version', '6.0.1',
    ]
    monkeypatch.setattr(sys, 'argv', arguments)
    assert validator.main() == 0
    assert json.loads(capsys.readouterr().out)['status'] == 'passed'

    dependency = runner.parent / 'grip_drive_control.py'
    saved_dependency = dependency.read_bytes()
    dependency.write_text('# changed hold implementation\n')
    monkeypatch.setattr(sys, 'argv', arguments)
    assert validator.main() == 1
    rejected_dependencies = json.loads(capsys.readouterr().out)
    assert all(any('runtime_script_files_sha256:' in e for e in case['errors'])
               for case in rejected_dependencies['cases'])
    dependency.write_bytes(saved_dependency)

    failed_path = results / 'rg6_dynamic-showcase.json'
    failed = json.loads(failed_path.read_text(encoding='utf-8'))
    failed['fixture_drive_override'] = {'maximum_force': 999.0}
    failed_path.write_text(json.dumps(failed), encoding='utf-8')
    monkeypatch.setattr(sys, 'argv', arguments)
    assert validator.main() == 1
    rejected = json.loads(capsys.readouterr().out)
    rg6_dynamic = next(
        case for case in rejected['cases']
        if case['model'] == 'rg6' and case['scenario'] == 'dynamic-showcase')
    assert 'fixture_drive_override is not null' in rg6_dynamic['errors']

    failed['fixture_drive_override'] = None
    failed['effective_solver_settings']['velocity_iterations'] = 8
    failed_path.write_text(json.dumps(failed), encoding='utf-8')
    monkeypatch.setattr(sys, 'argv', arguments)
    assert validator.main() == 1
    rejected = json.loads(capsys.readouterr().out)
    rg6_dynamic = next(
        case for case in rejected['cases']
        if case['model'] == 'rg6' and case['scenario'] == 'dynamic-showcase')
    assert ('effective TGS velocity iterations must be 0..4' in
            rg6_dynamic['errors'])

    failed['effective_solver_settings']['velocity_iterations'] = 4
    failed['effective_solver_settings']['external_forces_every_iteration'] = False
    failed_path.write_text(json.dumps(failed), encoding='utf-8')
    monkeypatch.setattr(sys, 'argv', arguments)
    assert validator.main() == 1
    rejected = json.loads(capsys.readouterr().out)
    assert any('loaded scene requires TGS with external forces every iteration' in case['errors']
               for case in rejected['cases'])


def test_asset_authoring_defaults_match_retention_contracts():
    """Regenerating tuned assets must not restore older physics values."""
    two_fg = _load_2fg14_authoring()
    rg = _load_rg6_authoring()
    two_fg_contract = json.loads(
        (PACKAGE_ROOT / 'config/2fg14_asset_contract.json').read_text(
            encoding='utf-8'))
    rg_contract = json.loads(
        (PACKAGE_ROOT / 'config/rg6_asset_contract.json').read_text(
            encoding='utf-8'))

    assert two_fg.PROVISIONAL_STIFFNESS == pytest.approx(
        two_fg_contract['drive_tuning']['stiffness'])
    assert two_fg.PROVISIONAL_DAMPING == pytest.approx(
        two_fg_contract['drive_tuning']['damping'])
    assert two_fg.PROVISIONAL_ARMATURE_KG == pytest.approx(
        two_fg_contract['drive_tuning']['armature_kg'])
    assert two_fg.PHYSX_SINGLE_DRIVE_FORCE_N == pytest.approx(
        two_fg_contract['drive_tuning']['physx_single_drive_force_n'])
    assert two_fg.FINGERTIP_FRICTION == pytest.approx(
        two_fg_contract['reference_fingers']['friction_coefficient'])

    assert rg.PROVISIONAL_MAX_TORQUE_NM == pytest.approx(
        rg_contract['drive_tuning']['maximum_drive_torque_nm'])
    assert rg.PROVISIONAL_STIFFNESS == pytest.approx(
        rg_contract['drive_tuning']['stiffness'])
    assert rg.PROVISIONAL_DAMPING == pytest.approx(
        rg_contract['drive_tuning']['damping'])
    assert rg.FINGERTIP_FRICTION == pytest.approx(
        rg_contract['collision_model']['fingertip_static_friction'])


def test_runtime_scripts_are_portable_and_do_not_use_deprecated_imports():
    """Runtime scripts must remain portable release artifacts."""
    for name in ('isaac_model_contract.py',
                 'isaac_runtime_compat.py',
                 'run_articulation_test.py',
                 'run_hardware_in_loop.py',
                 'run_ros_bridge.py',
                 'run_rg2_tip_contact_test.py',
                 'run_kinematic_qualification.py',
                 'run_drive_step_response.py',
                 'run_2fg_dynamic_contact_test.py',
                 'run_payload_retention_matrix.py',
                 'run_payload_retention_test.py'):
        text = _source(name)
        assert 'omni.isaac.' not in text
        assert not re.search(r'(?<![0-9])10\.[0-9]+\.[0-9]+\.[0-9]+', text)
        assert '/home/' not in text
        assert 'C:\\Users\\' not in text


def test_runtime_compatibility_is_feature_based():
    """Runtime support must follow available APIs rather than patch pins."""
    text = _source('isaac_runtime_compat.py')
    assert "getattr(manager, 'setup_simulation', None)" in text
    assert 'manager.initialize_physics()' in text
    assert 'get_stage_loading_status()[2] > 0' in text
    assert 'isaacsim.core.utils.extensions' in text
    for script in (
            'run_articulation_test.py', 'run_ros_bridge.py',
            'run_hardware_in_loop.py',
            'run_2fg_dynamic_contact_test.py'):
        source = _source(script)
        assert "!= '6.0.1'" not in source


def test_runtime_compatibility_selects_current_and_legacy_physics_setup():
    """Both known SimulationManager shapes must receive the same contract."""
    compat = _load_runtime_compat()

    class CurrentManager:
        calls = []
        physics_dt = None

        @classmethod
        def setup_simulation(cls, **kwargs):
            cls.calls.append(('setup', kwargs))
            cls.physics_dt = kwargs['dt']

        @classmethod
        def get_physics_dt(cls):
            return cls.physics_dt

    class LegacyManager:
        calls = []
        physics_dt = None

        @classmethod
        def set_physics_dt(cls, value):
            cls.calls.append(('dt', value))
            cls.physics_dt = value

        @classmethod
        def set_physics_sim_device(cls, value):
            cls.calls.append(('device', value))

        @classmethod
        def initialize_physics(cls):
            cls.calls.append(('initialize', None))

        @classmethod
        def get_physics_dt(cls):
            return cls.physics_dt

    assert compat.setup_simulation(CurrentManager, 0.01, 'cpu') == 0.01
    assert compat.setup_simulation(LegacyManager, 0.02, 'cpu') == 0.02
    assert CurrentManager.calls == [
        ('setup', {'dt': 0.01, 'device': 'cpu'})]
    assert LegacyManager.calls == [
        ('device', 'cpu'), ('dt', 0.02), ('initialize', None)]


def test_runtime_compatibility_reports_effective_physics_step():
    """Callers must receive the runtime step even if it differs by version."""
    compat = _load_runtime_compat()

    class BrokenManager:
        @classmethod
        def setup_simulation(cls, **kwargs):
            pass

        @classmethod
        def get_physics_dt(cls):
            return 1.0 / 60.0

    assert compat.setup_simulation(
        BrokenManager, 1.0 / 240.0, 'cpu') == pytest.approx(1.0 / 60.0)


def test_runtime_compatibility_closes_current_and_legacy_apps():
    """The test verdict must survive both SimulationApp close signatures."""
    compat = _load_runtime_compat()

    class CurrentApp:
        def __init__(self):
            self.exit_code = None

        def close(self, *, exit_code):
            self.exit_code = exit_code

    class LegacyApp:
        def __init__(self):
            self.closed = False

        def close(self):
            self.closed = True

    current = CurrentApp()
    legacy = LegacyApp()
    compat.close_app(current, 1)
    compat.close_app(legacy, 1)
    assert current.exit_code == 1
    assert legacy.closed


def test_2fg14_authoring_is_physical_only_and_release_bounded():
    """The authoring script must preserve the 2FG architecture boundary."""
    text = _source('author_2fg14_asset.py')
    assert 'include_task_coordinate:=false' in text
    assert 'include_ros2_control:=false' in text
    assert '_remove_frame_only_mount_links(urdf_path)' in text
    assert "fingertip_parent.set('link', jaw_name)" in text
    assert 'End Effector' in text
    assert 'fix_base=True' in text
    assert 'merge_fixed_joints=False' in text
    assert 'PROVISIONAL_STIFFNESS = 40000.0' in text
    assert 'PROVISIONAL_DAMPING = 900.0' in text
    assert 'PROVISIONAL_ARMATURE_KG = 5.0625' in text
    assert 'PHYSX_SINGLE_DRIVE_FORCE_N = 560.0' in text
    assert 'FINGERTIP_FRICTION = 0.6' in text
    assert 'FINGERTIP_DYNAMIC_FRICTION = 0.5' in text
    assert 'SOLVER_VELOCITY_ITERATIONS = 4' in text
    assert "CreateFrictionCombineModeAttr().Set('average')" in text
    assert 'forbidden = (' in text
    assert 'GetBody1Rel().GetTargets()' in text
    assert '_normalize_import_hierarchy(raw_stage)' in text
    assert "layout': '2fg-family-flat-rigid-links'" in text
    assert 'Usd.NamespaceEditor(stage)' in text
    assert 'candidate.HasAPI(UsdPhysics.RigidBodyAPI)' in text
    assert 'debug_mode=True' in text
    assert 'run_asset_transformer=False' in text
    assert 'run_asset_transformer_profile(' in text
    assert '_normalize_collision_approximations(generated)' in text
    assert '_author_reference_fingertip_collisions(generated)' in text
    assert 'REFERENCE_FINGERTIP_COLLISION_BOXES' in text
    assert "COLLISION_APPROXIMATION = 'convexDecomposition'" in text
    assert '_validate_semantic_layer_routing(generated)' in text
    assert "semantic_stage('payloads/robot.usda')" in text
    assert "semantic_stage('payloads/Physics/physics.usda')" in text
    assert "semantic_stage('payloads/Physics/physx.usda')" in text
    assert 'stage.SetEditTarget' not in text
    assert 'stage.Reload()' in text
    assert 'root_joint.GetBody0Rel().ClearTargets(removeSpec=False)' in text
    assert "Sdf.Path(f'{ROBOT_PRIM}/Geometry/base_link')" in text
    assert 'UsdPhysics.ArticulationRootAPI.Apply(physics_root_joint)' in text
    assert "root.ApplyAPI('NewtonArticulationRootAPI')" in text
    assert 'physics_geometry.RemoveAPI(' in text
    assert "physics_geometry.RemoveAPI('NewtonArticulationRootAPI')" in text
    assert (
        "physics_geometry.RemoveProperty('newton:selfCollisionEnabled')"
        in text)
    assert 'FAMILY_FILTERED_PAIR_TARGETS.items()' in text
    assert 'UsdPhysics.FilteredPairsAPI.Apply(owner)' in text
    assert 'allow_self_collision=True' in text
    assert 'SOLVER_POSITION_ITERATIONS = 68' in text
    assert "f'{ROBOT_PRIM}/Geometry/{side}_finger_base_link'" in text
    assert 'FINGERTIP_JOINTS' in text
    assert 'not prim.HasAPI(UsdPhysics.CollisionAPI)' not in text
    assert 'NBRPhysicsMaterial' not in text
    assert "AppendChild(f'{side}_finger_mount')" not in text
    assert "'custom_finger_anchors':" in text
    assert '_validate_family_topology(' in text
    assert "'reference_model': '2fg7'" in text
    assert "profile_name') != 'Isaac Sim Structure'" in text
    assert "ASSET_STRUCTURE_VERSION = '3.0'" in text
    assert "'asset_structure_version': ASSET_STRUCTURE_VERSION" in text
    assert "'@./payloads/base.usda@'" in text
    assert "geometry_format': 'usdc-crate'" in text
    assert 'ASSET_STRUCTURE_REQUIRED_LAYERS' in text
    assert "'follower_linear_drive': follower.HasAPI(" in text
    assert "topology['filtered_pairs'] = {}" in text
    assert "'physics_articulation_roots': api_paths(" in text
    assert "'newton_articulation_roots': api_paths(" in text
    assert "report['traceback'] = traceback.format_exc().splitlines()" in (
        _source('run_articulation_test.py'))
    assert "report['asset_files_sha256'] = _asset_manifest(" in (
        _source('run_articulation_test.py'))


def test_rg6_authoring_uses_rg2_structure_and_physical_only_urdf(tmp_path):
    """RG6 authoring must reuse policy, not RG2 model-specific physics."""
    authoring = _load_rg6_authoring()
    source = _source('author_rg6_asset.py')
    xacro = PACKAGE_ROOT.parent / (
        'onrobot_rg6/urdf/realmesh_onrobot_rg6.urdf.xacro')
    urdf = tmp_path / 'onrobot_rg6.urdf'

    authoring._expand_xacro(xacro, urdf)
    authoring._make_physical_only_urdf(urdf)
    import_contract = authoring._validate_physical_urdf(urdf)
    robot = ET.parse(urdf).getroot()
    links = {link.get('name') for link in robot.findall('link')}
    joints = {joint.get('name'): joint for joint in robot.findall('joint')}

    assert 'grip_stroke_control_link' not in links
    assert 'grip_stroke' not in joints
    assert 'left_finger_mount' not in links
    assert 'right_finger_mount' not in links
    assert joints['left_finger_tip_joint'].find('parent').get('link') == (
        'left_finger_base_link')
    assert joints['right_finger_tip_joint'].find('parent').get('link') == (
        'right_finger_base_link')
    assert robot.find('ros2_control') is None
    assert import_contract['massless_links'] == []
    assert set(import_contract['links']) == set(authoring.LINKS)
    assert set(import_contract['mimics']) == set(authoring.MIMICS)
    for name, definition in import_contract['mimics'].items():
        assert authoring.MIMICS[name] == definition['multiplier']
        assert authoring.PHYSX_MIMICS[name] == -definition['multiplier']
    assert '_normalize_import_hierarchy(raw_stage)' in source
    assert "'layout': 'rg-family-flat-rigid-links'" in source
    assert 'Usd.NamespaceEditor(stage)' in source
    assert '_validate_authored_stage(entrypoint)' in source
    assert '_clear_imported_articulation_roots(transformed)' in source
    assert '_remove_composed_geometry_articulation_roots(entrypoint)' in source
    assert 'binding = UsdShade.MaterialBindingAPI.Apply(body)' in source
    assert 'UsdShade.Material(material.GetPrim())' in source
    assert 'UsdShade.Tokens.weakerThanDescendants' in source
    assert "'fingertip-material-binding-api'" in source
    assert "'canonical_layer': 'payloads/Physics/physics.usda'" in source
    assert "base_path = package / 'payloads/base.usda'" in source
    assert 'Sdf.CreatePrimInLayer(' in source
    assert 'new_op.deletedItems' in source
    entrypoint = tmp_path / 'onrobot_rg6.usda'
    authoring._canonical_entrypoint(entrypoint)
    entrypoint_text = entrypoint.read_text(encoding='utf-8')
    assert ('prepend payload = '
            '@./payloads/Physics/physx_parallel_grip_tip_contact.usda@'
            in entrypoint_text)
    assert 'prepend payload =\n' not in entrypoint_text
    assert 'physx_parallel_grip_tip_contact' in source
    assert authoring.FINGERTIP_FRICTION == .6
    assert authoring.FINGERTIP_DYNAMIC_FRICTION == .5
    assert 'FINGERTIP_PAD_SIZE_M = (0.037, 0.025, 0.0114)' in source
    assert 'FINGERTIP_PAD_CENTER_M = (0.0, 0.0001, -0.0007)' in source
    assert 'imported_collision.SetActive(False)' in source
    assert "f'{body_path}/payload_contact_collision'" in source
    assert "CreateFrictionCombineModeAttr().Set('average')" in source
    assert 'PROVISIONAL_MAX_TORQUE_NM = 16.17' in source
    assert authoring.PROVISIONAL_STIFFNESS == pytest.approx(1.74532925199)
    assert authoring.PROVISIONAL_DAMPING == pytest.approx(.45712968617)
    assert authoring.PRIMARY_ARMATURE_KG_M2 == pytest.approx(1.715)
    assert authoring.MIMIC_DAMPING_RATIO == 1.
    assert 'SOLVER_POSITION_ITERATIONS = 68' in source
    assert 'SOLVER_VELOCITY_ITERATIONS = 4' in source
    assert 'physxScene:solveArticulationContactLast = 1' in source
    assert "'kind': 'leader-drive'" in source
    assert "'kind': 'articulation-solver'" in source
    assert 'MAX_VELOCITY_RAD_S = 0.8492845886' in source
    assert 'args.family_reference.resolve()' in source


def test_2fg14_layer_routing_rejects_duplicate_articulation_roots(tmp_path):
    """Authoring must not accept importer and root-joint articulation roots."""
    module = _load_2fg14_authoring()
    entrypoint = tmp_path / 'onrobot_2fg14.usda'
    robot = tmp_path / 'payloads' / 'robot.usda'
    physics = tmp_path / 'payloads' / 'Physics' / 'physics.usda'
    physx = tmp_path / 'payloads' / 'Physics' / 'physx.usda'
    instances = tmp_path / 'payloads' / 'instances.usda'
    robot.parent.mkdir(parents=True)
    physics.parent.mkdir(parents=True)
    entrypoint.write_text('#usda 1.0\n', encoding='utf-8')
    robot.write_text(
        'prepend rel isaac:physics:robotLinks = '
        '[</onrobot_2fg14/Geometry/base_link>]\n'
        'prepend rel isaac:physics:robotJoints = '
        '[</onrobot_2fg14/Physics/root_joint>]\n',
        encoding='utf-8')
    physics.write_text(
        'over "Geometry" (prepend apiSchemas = '
        '["PhysicsArticulationRootAPI"]) {}\n'
        'def PhysicsFixedJoint "root_joint" '
        '(prepend apiSchemas = ["PhysicsArticulationRootAPI"]) {}\n'
        'float drive:linear:physics:maxForce = 280\n',
        encoding='utf-8')
    physx.write_text('', encoding='utf-8')
    instances.write_text(
        'token physics:approximation = "convexDecomposition"\n',
        encoding='utf-8')

    with pytest.raises(RuntimeError, match='one root_joint articulation'):
        module._validate_semantic_layer_routing(entrypoint)


def test_2fg14_import_urdf_removes_only_frame_mount_links(tmp_path):
    """The Isaac import input must retain the physical finger bodies."""
    authoring = _load_2fg14_authoring()
    xacro = PACKAGE_ROOT.parent / 'onrobot_2fg14/urdf/' \
        'realmesh_onrobot_2fg14.urdf.xacro'
    urdf = tmp_path / 'onrobot_2fg14.urdf'

    authoring._expand_xacro(xacro, urdf)
    authoring._remove_frame_only_mount_links(urdf)
    authoring._validate_import_urdf(urdf)

    robot = ET.parse(urdf).getroot()
    links = {link.get('name') for link in robot.findall('link')}
    joints = {joint.get('name'): joint for joint in robot.findall('joint')}
    assert 'right_finger_mount' not in links
    assert 'left_finger_mount' not in links
    assert 'right_finger_mount_joint' not in joints
    assert 'left_finger_mount_joint' not in joints
    assert joints['right_fingertip_joint'].find('parent').get('link') == (
        'right_finger_base_link')
    assert joints['left_fingertip_joint'].find('parent').get('link') == (
        'left_finger_base_link')
    assert {'finger_stroke', 'left_finger_base_joint'} <= set(joints)


def test_2fg14_authoring_uses_the_release_2fg7_family_reference():
    """The authoring gate must compare against the shipped 2FG7 asset."""
    authoring = _load_2fg14_authoring()
    reference = authoring._default_family_reference()
    assert reference == ASSET_REPOSITORY / 'assets/2fg7/onrobot_2fg7.usda'
    assert reference.is_file()


def test_2fg14_authoring_restores_family_collision_policy():
    """Asset transformation must not turn concave fingers into solid hulls."""
    text = _source('author_2fg14_asset.py')
    assert "COLLISION_APPROXIMATION = 'convexDecomposition'" in text
    assert '_normalize_collision_approximations(generated)' in text
    assert "'family_reference': '2fg7'" in text


def test_2fg14_authoring_declares_the_2fg7_filtered_pair_graph():
    """The candidate must preserve the released family's collision policy."""
    authoring = _load_2fg14_authoring()
    assert authoring.FAMILY_FILTERED_PAIR_TARGETS == {
        'base_link': (
            'right_finger_base_link', 'right_fingertip_link',
            'left_finger_base_link', 'left_fingertip_link'),
        'right_finger_base_link': (
            'right_fingertip_link', 'left_finger_base_link',
            'left_fingertip_link'),
        'right_fingertip_link': ('left_finger_base_link',),
        'left_finger_base_link': ('left_fingertip_link',),
    }


def test_2fg14_asset_structure_gate_accepts_transformer_layout(tmp_path):
    """The authoring gate must recognize the required layered interface."""
    authoring = _load_2fg14_authoring()
    entrypoint = tmp_path / 'onrobot_2fg14.usda'
    entrypoint.write_text(
        '#usda 1.0\n( references = @./payloads/base.usda@ )\n'
        'def Xform "onrobot_2fg14" (\n'
        '    variantSets = "Physics"\n'
        '    variants = { string Physics = "physx" }\n) {}\n',
        encoding='utf-8')
    for relative in authoring.ASSET_STRUCTURE_REQUIRED_LAYERS:
        layer = tmp_path / relative
        layer.parent.mkdir(parents=True, exist_ok=True)
        if relative == 'payloads/geometries.usd':
            layer.write_bytes(b'PXR-USDCfixture')
        else:
            layer.write_text('#usda 1.0\n', encoding='utf-8')
    profile = tmp_path / 'profile.json'
    profile.write_text(json.dumps({
        'profile_name': 'Isaac Sim Structure',
        'version': '1.0',
    }), encoding='utf-8')

    report = authoring._validate_asset_structure(entrypoint, profile)
    assert report['asset_structure_version'] == '3.0'
    assert report['profile'] == 'Isaac Sim Structure'
    assert report['geometry_format'] == 'usdc-crate'


def test_2fg14_semantic_layer_gate_accepts_2fg7_layer_ownership(tmp_path):
    """Family physics and robot opinions must stay out of the interface."""
    authoring = _load_2fg14_authoring()
    entrypoint = tmp_path / 'onrobot_2fg14.usda'
    robot = tmp_path / 'payloads/robot.usda'
    physics = tmp_path / 'payloads/Physics/physics.usda'
    physx = tmp_path / 'payloads/Physics/physx.usda'
    instances = tmp_path / 'payloads/instances.usda'
    for path in (robot, physics, physx, instances):
        path.parent.mkdir(parents=True, exist_ok=True)
    entrypoint.write_text('#usda 1.0\n', encoding='utf-8')
    robot.write_text(
        'prepend rel isaac:physics:robotLinks = '
        '[</onrobot_2fg14/Geometry/base_link>]\n'
        'prepend rel isaac:physics:robotJoints = '
        '[</onrobot_2fg14/Physics/root_joint>]\n',
        encoding='utf-8')
    physics.write_text(
        'def PhysicsFixedJoint "root_joint" '
        '(prepend apiSchemas = ["PhysicsArticulationRootAPI"]) {}\n'
        'float drive:linear:physics:maxForce = 280\n'
        'float physics:staticFriction = 0.6\n'
        'float physics:dynamicFriction = 0.5\n'
        'token physxMaterial:frictionCombineMode = "average"\n',
        encoding='utf-8')
    physx.write_text(
        'PhysxArticulationAPI\n'
        'bool physxArticulation:enabledSelfCollisions = 1\n'
        'int physxArticulation:solverPositionIterationCount = 68\n'
        'int physxArticulation:solverVelocityIterationCount = 4\n'
        'float drive:linear:physics:maxForce = 560\n'
        'PhysxMimicJointAPI:rotX\n'
        'delete apiSchemas = ["PhysicsDriveAPI:linear"]\n',
        encoding='utf-8')
    instances.write_text(
        'token physics:approximation = "convexDecomposition"\n',
        encoding='utf-8')

    report = authoring._validate_semantic_layer_routing(entrypoint)
    assert report['status'] == 'matched-2fg7-layer-ownership'
    assert report['collision_approximation'] == 'convexDecomposition'

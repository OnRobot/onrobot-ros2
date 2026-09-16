#!/usr/bin/env python3
"""Qualify sideways external-pinch payload retention."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import shlex
import sys
import time
import traceback

from isaac_model_contract import articulation_path, default_asset
from isaac_model_contract import driven_joint, joint_limits, load_contract
from isaac_model_contract import SUPPORTED_MODELS

from isaac_runtime_compat import is_stage_loading, play, setup_simulation, stop
from grip_drive_control import PadContactObserver, contact_preload_reference, hold_control, script_manifest

SCENE_ROOT = '/onrobot_payload_retention'
CARRIER_PATH = f'{SCENE_ROOT}/carrier'
BLOCK_PATH = f'{SCENE_ROOT}/block'
SUPPORT_PATH = f'{SCENE_ROOT}/support'
HARNESS_REVISION = 14
REFERENCE_STATIC_FRICTION = 0.5
REFERENCE_DYNAMIC_FRICTION = 0.4
MAX_TRAJECTORY_SAMPLES = 8000


class _EmbeddedSimulationApp:
    """Small SimulationApp-compatible adapter for a running Isaac server.

    The remote Python endpoint already owns a Kit application. Creating a
    second SimulationApp from a command launched by that endpoint can stall
    Kit startup and consumes the server's renderer/physics resources. The
    embedded path therefore advances the existing application in-place and
    deliberately leaves its lifetime to the server.
    """

    def update(self):
        import omni.kit.app
        omni.kit.app.get_app().update()

    def close(self, *args, **kwargs):
        del args, kwargs


def _package_path(relative: str) -> Path:
    source = Path(__file__).resolve().parents[1] / relative
    if source.is_file():
        return source
    return (Path(__file__).resolve().parents[2] / 'share' /
            'onrobot_gripper_isaac' / relative)


def _write(path: Path, report: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n',
                    encoding='utf-8')


def _array(value):
    return value.numpy() if hasattr(value, 'numpy') else value


def _row(value):
    return [float(item) for item in _array(value)[0]]


def _linear_velocity(value):
    # Isaac Sim 5 returns (linear, angular) arrays while Isaac Sim 6 returns
    # one combined velocity array.
    if isinstance(value, tuple):
        return _row(value[0])
    return _row(value)[:3]


def _norm(values) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in values))


def _smooth_joint_targets(initial, final, steps):
    """Command a bounded rest-to-rest path; never teleport one mimic member."""
    if steps < 1 or not all(math.isfinite(v) for v in (initial, final)):
        raise ValueError('joint path requires finite endpoints and positive steps')
    for index in range(steps):
        u = (index + 1) / steps
        blend = u ** 3 * (10 + u * (-15 + 6 * u))
        yield initial + (final - initial) * blend


def _projected_joint_force(articulation, joint_index):
    """Return the active-axis joint reaction when the runtime exposes it."""
    getter = getattr(articulation, 'get_dof_projected_joint_forces', None)
    if not callable(getter):
        return None
    try:
        value = float(_array(getter(
            dof_indices=[joint_index]))[0, 0])
    except (AssertionError, AttributeError, IndexError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _finite_vector(values, size: int) -> bool:
    # Isaac may return NumPy or Gf vectors rather than Python lists.
    if isinstance(values, (str, bytes, dict)):
        return False
    try:
        return len(values) == size and all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError, OverflowError):
        return False


def _contact_body_path(value):
    """Decode the PhysX integer path form without confusing it with text."""
    try:
        import operator
        encoded = operator.index(value)
    except TypeError:
        return str(value)
    from pxr import PhysicsSchemaTools
    return str(PhysicsSchemaTools.intToSdfPath(encoded))


def _vector_norm(values) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in values))


def _contact_observability_sample(sensor, object_path: str, physics_dt: float):
    """Keep independent contact observables separate in the report."""
    getter = getattr(sensor, 'get_data', None)
    data = getter() if getter is not None else sensor.get_current_frame()
    sensor_force = float(data.get('force', 0.0))
    valid = ('in_contact' in data and math.isfinite(sensor_force))
    raw_contact_data_available = 'contacts' in data
    normal_force = 0.0
    impulse = [0.0, 0.0, 0.0]
    object_contacts = 0
    for contact in data.get('contacts', []):
        body0 = _contact_body_path(contact.get('body0', ''))
        body1 = _contact_body_path(contact.get('body1', ''))
        normal = contact.get('normal', [])
        contact_impulse = contact.get('impulse', [])
        if object_path not in (body0, body1) or not (
                _finite_vector(normal, 3) and
                _finite_vector(contact_impulse, 3)):
            continue
        magnitude = _vector_norm(normal)
        if magnitude <= 1.0e-12:
            continue
        unit_normal = [float(value) / magnitude for value in normal]
        impulse_vector = [float(value) for value in contact_impulse]
        # Absolute projection removes PhysX body-order sign ambiguity; the full
        # vector remains available to distinguish tangential impulse.
        normal_force += abs(sum(
            impulse_vector[index] * unit_normal[index]
            for index in range(3))) / physics_dt
        for index in range(3):
            impulse[index] += impulse_vector[index]
        object_contacts += 1
    return {
        'sensor_valid': valid,
        'raw_contact_data_available': raw_contact_data_available,
        'sensor_force_magnitude_n': sensor_force,
        'object_contact_points': object_contacts,
        'normal_force_from_impulse_n': normal_force,
        'impulse_vector_ns': impulse,
        'impulse_vector_over_dt_n': [value / physics_dt for value in impulse],
    }


def _series_report(values):
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return {'samples': 0, 'mean': None, 'maximum': None, 'final': None}
    return {'samples': len(finite), 'mean': sum(finite) / len(finite),
            'maximum': max(finite), 'final': finite[-1]}


def _vector_series_report(values):
    finite = [value for value in values if _finite_vector(value, 3)]
    if not finite:
        return {'samples': 0, 'mean_xyz': None, 'maximum_norm': None,
                'final_xyz': None}
    return {
        'samples': len(finite),
        'mean_xyz': [sum(float(value[index]) for value in finite) /
                     len(finite) for index in range(3)],
        'maximum_norm': max(_vector_norm(value) for value in finite),
        'final_xyz': [float(value) for value in finite[-1]],
    }


def _quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]


def _quat_inverse(q):
    magnitude = sum(value * value for value in q)
    return [q[0] / magnitude, -q[1] / magnitude,
            -q[2] / magnitude, -q[3] / magnitude]


def _rotate(q, vector):
    value = _quat_multiply(
        _quat_multiply(q, [0.0, *vector]), _quat_inverse(q))
    return value[1:]


def _relative_pose(carrier_position, carrier_orientation,
                   object_position, object_orientation):
    inverse = _quat_inverse(carrier_orientation)
    relative_position = _rotate(inverse, [
        object_position[index] - carrier_position[index]
        for index in range(3)])
    relative_orientation = _quat_multiply(inverse, object_orientation)
    return relative_position, relative_orientation


def _rotation_error(a, b) -> float:
    dot = abs(sum(a[index] * b[index] for index in range(4)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def _define_cube(stage, path, position, size, color, Gf, UsdGeom,
                 UsdPhysics):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(
        ).Set(True)
    return cube.GetPrim()


def _asset_manifest(asset: Path) -> dict:
    root = asset.parent
    return {
        str(path.relative_to(root)): hashlib.sha256(
            path.read_bytes()).hexdigest()
        for path in sorted(root.rglob('*.usd*')) if path.is_file()
    }


def _collision_material_scope(root_prim, material, Usd, UsdPhysics,
                              UsdShade):
    """Inventory fingertip colliders and optionally bind a fixture material."""
    paths = []
    bound = set()
    for prim in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        collision_enabled = UsdPhysics.CollisionAPI(
            prim).GetCollisionEnabledAttr().Get()
        if collision_enabled is False:
            continue
        paths.append(str(prim.GetPath()))
        binding_prim = prim
        while binding_prim.IsInstanceProxy():
            binding_prim = binding_prim.GetParent()
        path = str(binding_prim.GetPath())
        if path in bound:
            continue
        if material is not None:
            UsdShade.MaterialBindingAPI.Apply(binding_prim).Bind(
                material,
                bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                materialPurpose='physics')
            bound.add(path)
    if not paths:
        raise RuntimeError(
            f'no enabled fingertip collision geometry below '
            f'{root_prim.GetPath()}')
    return {'collision_paths': paths, 'binding_paths': sorted(bound)}


def main() -> int:
    """Run one configured payload-retention qualification."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=SUPPORTED_MODELS, required=True)
    parser.add_argument('--asset', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument(
        '--scenario', choices=('rated-margin-static', 'dynamic-showcase'),
        default='rated-margin-static')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--hold-control', choices=('position', 'preload-position'),
                        help='override the model profile holding command (no physics changes)')
    parser.add_argument('--diagnostic-contact-last', action=argparse.BooleanOptionalAction,
                        default=None, help='diagnostic: override articulation contact solver ordering')
    parser.add_argument('--diagnostic-velocity-iterations', type=int, choices=(0, 1, 2, 4),
                        help='diagnostic: set matching articulation and workpiece velocity iterations')
    parser.add_argument('--diagnostic-contact-vectors', action='store_true',
                        help='diagnostic: record separate normal/friction impulses on the workpiece')
    parser.add_argument('--diagnostic-stabilization', action=argparse.BooleanOptionalAction,
                        default=None, help='diagnostic: override PhysX scene stabilization')
    parser.add_argument('--diagnostic-external-forces-every-iteration', action=argparse.BooleanOptionalAction,
                        default=None, help='diagnostic: apply external forces at each TGS position iteration')
    parser.add_argument(
        '--embedded', action='store_true',
        help=argparse.SUPPRESS)
    parser.add_argument(
        '--payload-kg', type=float,
        help='override the configured payload mass for qualification sweeps')
    parser.add_argument(
        '--workpiece-size-m', '--block-size-m', dest='workpiece_size_m',
        type=float, nargs=3, metavar=('X', 'Y', 'Z'),
        help=('override the configured workpiece dimensions in metres; the '
              'default remains the model profile fixture'))
    parser.add_argument(
        '--apply-profile-physics-settings', action='store_true',
        help=('diagnostic only: override the asset fingertip material, drive, '
              'and articulation solver settings from the test profile'))
    parser.add_argument(
        '--diagnostic-drive-max-force', type=float,
        help=('diagnostic only: override just the driven-joint maximum force '
              'or torque in the qualification stage'))
    parser.add_argument(
        '--contact-observability', action='store_true',
        help=('record per-tip contact-sensor magnitudes, impulse-derived '
              'normal forces and the driven-joint reaction without changing '
              'the contact materials; this is not a '
              'public Tool API force measurement'))
    parser.add_argument(
        '--reference-friction', action='store_true',
        help='diagnostic only: replace fingertip friction with 0.5/0.4, average combining')
    parser.add_argument(
        '--recording-dir', type=Path,
        help='capture the GUI presentation replay as a 30 fps PNG sequence')
    args, _ = parser.parse_known_args()
    if (args.diagnostic_drive_max_force is not None and
            (not math.isfinite(args.diagnostic_drive_max_force) or
             args.diagnostic_drive_max_force <= 0.0)):
        parser.error(
            '--diagnostic-drive-max-force must be finite and positive')
    if args.reference_friction and args.apply_profile_physics_settings:
        parser.error(
            '--reference-friction and --apply-profile-physics-settings '
            'would apply competing fixture material settings')

    contract = load_contract(args.model)
    asset = args.asset or default_asset(args.model)
    config_path = args.config or _package_path(
        'config/payload_retention_profiles.json')
    config_bytes = config_path.read_bytes() if config_path.is_file() else b''
    report = {
        'schema_version': 1,
        'harness_revision': HARNESS_REVISION,
        'test': 'sideways-external-pinch-payload-retention',
        'model': args.model,
        'scenario': args.scenario,
        'asset': str(asset.resolve()),
        'config': str(config_path.resolve()),
        'config_sha256': hashlib.sha256(config_bytes).hexdigest(),
        'runner_sha256': hashlib.sha256(
            Path(__file__).resolve().read_bytes()).hexdigest(),
        'runtime_script_files_sha256': script_manifest(__file__),
        'status': 'starting',
        'tests': [],
    }
    _write(args.output, report)
    app = None
    pad_contacts = None
    try:
        config = json.loads(config_bytes.decode('utf-8'))
        if (config.get('schema_version') != 1 or
                config.get('test') != report['test'] or
                args.model not in config.get('models', {})):
            raise RuntimeError('payload-retention configuration is invalid')
        fixture = config['fixture']
        profile = config['models'][args.model]
        control = hold_control(args.model, profile, args.hold_control)
        report['hold_control'] = control
        report['hold_control_source'] = 'cli' if args.hold_control is not None else 'model-profile'
        report['closure_target'] = float(profile['grasp_target'])
        fingertip_paths = [
            f'/onrobot_{args.model}/Geometry/{name}'
            for name in profile['fingertip_links']]
        rated_payload_mass = float(profile['rated_force_fit_payload_kg'])
        configured_payload = (
            profile['qualification_payload_kg']
            if args.scenario == 'rated-margin-static'
            else profile['dynamic_qualification_payload_kg'])
        payload_mass = float(
            args.payload_kg if args.payload_kg is not None
            else configured_payload)
        if not math.isfinite(payload_mass) or payload_mass <= 0.0:
            raise RuntimeError('qualification payload must be positive')
        if (args.payload_kg is None and
                payload_mass - rated_payload_mass > 1.0e-9):
            raise RuntimeError(
                'qualification payload must not exceed the rated force-fit '
                'payload')
        lower, upper = joint_limits(contract)
        for name in ('open_target', 'grasp_target'):
            if not lower <= float(profile[name]) <= upper:
                raise RuntimeError(f'{name} is outside the asset joint limits')
        if float(profile['open_target']) <= float(profile['grasp_target']):
            raise RuntimeError(
                'external-pinch targets do not close the gripper')

        if args.embedded and args.gui:
            raise RuntimeError('--embedded cannot be combined with --gui')
        if args.embedded:
            app = _EmbeddedSimulationApp()
        else:
            import isaacsim
            from isaacsim.simulation_app import SimulationApp
            app = SimulationApp({
                'headless': not args.gui,
                'renderer': ('RayTracedLighting' if args.gui
                             else 'MinimalRendering'),
                'disable_viewport_updates': not args.gui,
                'multi_gpu': False,
                'width': 1280,
                'height': 720,
            })
        import omni.usd
        import omni.timeline
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.core.version import get_version
        from pxr import (Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics,
                         UsdShade)

        report['isaac_sim_version'] = get_version()[0]
        report['asset_revision'] = contract['asset_revision']
        if not asset.is_file():
            raise RuntimeError(f'asset does not exist: {asset}')
        if not omni.usd.get_context().open_stage(str(asset.resolve())):
            raise RuntimeError(f'Isaac Sim could not open {asset}')
        app.update()
        while is_stage_loading():
            app.update()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Xform.Define(stage, SCENE_ROOT)
        from showcase_branding import create_studio_lighting
        report['lighting'] = create_studio_lighting(stage, f'{SCENE_ROOT}/studio_light')

        carrier_position = [float(v) for v in fixture['carrier_position_m']]
        carrier_orientation = [
            float(v) for v in fixture['carrier_orientation_wxyz']]
        carrier = UsdGeom.Xform.Define(stage, CARRIER_PATH)
        carrier_xform = UsdGeom.Xformable(carrier.GetPrim())
        carrier_translate = carrier_xform.AddTranslateOp()
        carrier_translate.Set(Gf.Vec3d(*carrier_position))
        carrier_xform.AddOrientOp().Set(Gf.Quatf(
            carrier_orientation[0], *carrier_orientation[1:]))
        root_prim = f'/onrobot_{args.model}'
        asset_root_prim = stage.GetPrimAtPath(root_prim)
        asset_root_xform = UsdGeom.Xformable(asset_root_prim)
        asset_root_xform.ClearXformOpOrder()
        asset_translate = asset_root_xform.AddTranslateOp()
        asset_translate.Set(Gf.Vec3d(*carrier_position))
        asset_root_xform.AddOrientOp().Set(Gf.Quatf(
            carrier_orientation[0], *carrier_orientation[1:]))
        root_joint_path = f'{root_prim}/Physics/root_joint'
        root_joint = UsdPhysics.FixedJoint.Get(stage, root_joint_path)
        if not root_joint.GetPrim().IsValid():
            raise RuntimeError(
                f'asset root joint is missing: {root_joint_path}')
        runtime_articulation_path = articulation_path(contract)

        local_center = [float(v) for v in profile['contact_center_local_m']]
        rotated_center = _rotate(carrier_orientation, local_center)
        block_position = [carrier_position[i] + rotated_center[i]
                          for i in range(3)]
        configured_block_size = profile.get(
            'block_size_m', fixture['block_size_m'])
        block_size_local = [
            float(v) for v in (
                args.workpiece_size_m
                if args.workpiece_size_m is not None
                else configured_block_size)]
        if (len(block_size_local) != 3 or
                any(not math.isfinite(value) or value <= 0.0
                    for value in block_size_local)):
            raise RuntimeError(
                'workpiece dimensions must contain three finite positive '
                'metre values')
        workpiece_size_source = (
            'cli' if args.workpiece_size_m is not None else 'model-profile')
        # A 90-degree X rotation maps local Y to world Z and local Z to
        # world Y.
        block_size_world = [block_size_local[0], block_size_local[2],
                            block_size_local[1]]
        block_prim = _define_cube(
            stage, BLOCK_PATH, block_position, block_size_world,
            (0.12, 0.42, 0.8), Gf, UsdGeom, UsdPhysics)
        UsdPhysics.RigidBodyAPI.Apply(
            block_prim).CreateRigidBodyEnabledAttr().Set(True)
        block_kinematic = UsdPhysics.RigidBodyAPI(
            block_prim).CreateKinematicEnabledAttr()
        block_kinematic.Set(True)
        block_collision = UsdPhysics.CollisionAPI(
            block_prim).GetCollisionEnabledAttr()
        block_collision.Set(False)
        UsdPhysics.MassAPI.Apply(block_prim).CreateMassAttr().Set(payload_mass)
        block_ccd = PhysxSchema.PhysxRigidBodyAPI.Apply(
            block_prim).CreateEnableCCDAttr()
        # CCD is enabled only after the placement body becomes dynamic. PhysX
        # rejects CCD on a kinematic body during scene construction.
        block_ccd.Set(False)

        material = UsdShade.Material.Define(
            stage, f'{SCENE_ROOT}/block_material')
        material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        material_api.CreateStaticFrictionAttr().Set(
            float(fixture['block_static_friction']))
        material_api.CreateDynamicFrictionAttr().Set(
            float(fixture['block_dynamic_friction']))
        material_api.CreateRestitutionAttr().Set(0.0)
        binding = UsdShade.MaterialBindingAPI.Apply(block_prim)
        binding.Bind(
            material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose='physics')

        asset_fingertip_combine_mode = str(profile.get(
            'fingertip_friction_combine_mode', 'average'))
        if asset_fingertip_combine_mode not in {
                'average', 'min', 'multiply', 'max'}:
            raise RuntimeError('invalid fingertip friction combine mode')
        fingertip_combine_mode = asset_fingertip_combine_mode
        fixture_fingertip_material = None
        fingertip_material = None
        if args.apply_profile_physics_settings or args.reference_friction:
            fingertip_material = UsdShade.Material.Define(
                stage, f'{SCENE_ROOT}/fingertip_material')
            if args.reference_friction:
                fingertip_static_friction = REFERENCE_STATIC_FRICTION
                fingertip_dynamic_friction = REFERENCE_DYNAMIC_FRICTION
                fingertip_combine_mode = 'average'
            else:
                fingertip_static_friction = float(profile.get(
                    'fingertip_static_friction',
                    fixture['fingertip_static_friction']))
                fingertip_dynamic_friction = float(profile.get(
                    'fingertip_dynamic_friction',
                    fixture['fingertip_dynamic_friction']))
            fingertip_api = UsdPhysics.MaterialAPI.Apply(
                fingertip_material.GetPrim())
            fingertip_api.CreateStaticFrictionAttr().Set(
                fingertip_static_friction)
            fingertip_api.CreateDynamicFrictionAttr().Set(
                fingertip_dynamic_friction)
            fingertip_api.CreateRestitutionAttr().Set(0.0)
            fingertip_physx_api = PhysxSchema.PhysxMaterialAPI.Apply(
                fingertip_material.GetPrim())
            fingertip_physx_api.CreateFrictionCombineModeAttr().Set(
                fingertip_combine_mode)
            fixture_fingertip_material = {
                'static_friction': fingertip_static_friction,
                'dynamic_friction': fingertip_dynamic_friction,
                'friction_combine_mode': fingertip_combine_mode,
                'scope': 'reference-friction-observability',
            }
        fingertip_bindings = []
        for link in profile['fingertip_links']:
            fingertip_bindings.append(_collision_material_scope(
                stage.GetPrimAtPath(f'{root_prim}/Geometry/{link}'),
                fingertip_material, Usd, UsdPhysics, UsdShade))

        fixture_drive = None
        if ((args.apply_profile_physics_settings and
             ('fixture_drive_stiffness' in profile or
              'fixture_drive_max_force' in profile)) or
                args.diagnostic_drive_max_force is not None):
            joint_prim = stage.GetPrimAtPath(
                f'{root_prim}/Physics/{driven_joint(contract)}')
            if not joint_prim.IsValid():
                raise RuntimeError('fixture drive joint is missing')
            drive_instance = 'linear' if args.model.startswith('2fg') \
                else 'angular'
            drive = UsdPhysics.DriveAPI(joint_prim, drive_instance)
            stiffness = float(profile.get(
                'fixture_drive_stiffness', drive.GetStiffnessAttr().Get()))
            damping = float(profile.get(
                'fixture_drive_damping', drive.GetDampingAttr().Get()))
            maximum_force = (
                args.diagnostic_drive_max_force
                if args.diagnostic_drive_max_force is not None
                else float(profile.get(
                    'fixture_drive_max_force',
                    drive.GetMaxForceAttr().Get())))
            drive.CreateStiffnessAttr(stiffness)
            drive.CreateDampingAttr(damping)
            drive.CreateMaxForceAttr(maximum_force)
            fixture_drive = {
                'instance': drive_instance,
                'stiffness': stiffness,
                'damping': damping,
                'maximum_force': maximum_force,
                'scope': 'reference-friction-observability',
                'source': (
                    'diagnostic-command-line'
                    if args.diagnostic_drive_max_force is not None
                    else 'test-profile'),
            }

        fixture_solver = None
        if (args.apply_profile_physics_settings and
                ('fixture_solver_position_iterations' in profile or
                 'fixture_solver_velocity_iterations' in profile)):
            articulation_prim = stage.GetPrimAtPath(
                runtime_articulation_path)
            articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(
                articulation_prim)
            position_iterations = int(profile.get(
                'fixture_solver_position_iterations', 64))
            velocity_iterations = int(profile.get(
                'fixture_solver_velocity_iterations', 2))
            articulation_api.CreateSolverPositionIterationCountAttr().Set(
                position_iterations)
            articulation_api.CreateSolverVelocityIterationCountAttr().Set(
                velocity_iterations)
            fixture_solver = {
                'position_iterations': position_iterations,
                'velocity_iterations': velocity_iterations,
                'scope': 'reference-friction-observability',
            }

        support_height = 0.01
        support_position = list(block_position)
        support_position[2] -= (
            block_size_world[2] / 2.0 + support_height / 2.0 +
            float(fixture['support_clearance_m']))
        support_prim = _define_cube(
            stage, SUPPORT_PATH, support_position,
            [min(0.015, block_size_world[0] * 0.3),
             block_size_world[1] * 0.6,
             support_height], (0.25, 0.27, 0.30), Gf, UsdGeom, UsdPhysics)

        floor_prim = _define_cube(
            stage, f'{SCENE_ROOT}/floor',
            [carrier_position[0], carrier_position[1], 0.0],
            [0.65, 0.65, 0.02], (0.12, 0.14, 0.18),
            Gf, UsdGeom, UsdPhysics)
        UsdPhysics.CollisionAPI(
            floor_prim).GetCollisionEnabledAttr().Set(False)

        scene_prim = stage.GetPrimAtPath('/PhysicsScene')
        if not scene_prim.IsValid():
            scene_prim = UsdPhysics.Scene.Define(
                stage, '/PhysicsScene').GetPrim()
        scene = UsdPhysics.Scene(scene_prim)
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        PhysxSchema.PhysxSceneAPI.Apply(
            scene_prim).CreateEnableCCDAttr().Set(True)
        if args.diagnostic_stabilization is not None:
            PhysxSchema.PhysxSceneAPI(scene_prim).CreateEnableStabilizationAttr().Set(
                args.diagnostic_stabilization)
            fixture_solver = dict(fixture_solver or {},
                                  stabilization=args.diagnostic_stabilization,
                                  source='diagnostic-command-line')
        scene_external_forces = (args.diagnostic_external_forces_every_iteration
                                 if args.diagnostic_external_forces_every_iteration is not None
                                 else bool(config['physics']['external_forces_every_iteration']))
        scene_api = PhysxSchema.PhysxSceneAPI(scene_prim)
        scene_api.CreateSolverTypeAttr().Set('TGS')
        scene_api.CreateEnableExternalForcesEveryIterationAttr().Set(scene_external_forces)
        if args.diagnostic_external_forces_every_iteration is not None:
            fixture_solver = dict(fixture_solver or {},
                                  external_forces_every_iteration=args.diagnostic_external_forces_every_iteration,
                                  source='diagnostic-command-line')
        if args.diagnostic_contact_last is not None:
            PhysxSchema.PhysxSceneAPI(scene_prim).CreateSolveArticulationContactLastAttr().Set(
                args.diagnostic_contact_last)
            fixture_solver = dict(fixture_solver or {},
                                  solve_articulation_contact_last=args.diagnostic_contact_last,
                                  source='diagnostic-command-line')

        articulation_settings_prim = stage.GetPrimAtPath(
            runtime_articulation_path)
        if args.diagnostic_velocity_iterations is not None:
            PhysxSchema.PhysxArticulationAPI(articulation_settings_prim).CreateSolverVelocityIterationCountAttr().Set(
                args.diagnostic_velocity_iterations)
            PhysxSchema.PhysxRigidBodyAPI(block_prim).CreateSolverVelocityIterationCountAttr().Set(
                args.diagnostic_velocity_iterations)
            fixture_solver = dict(fixture_solver or {},
                                  velocity_iterations=args.diagnostic_velocity_iterations,
                                  workpiece_velocity_iterations=args.diagnostic_velocity_iterations,
                                  source='diagnostic-command-line')
        effective_solver_settings = {
            'solver_type': scene_api.GetSolverTypeAttr().Get(),
            'external_forces_every_iteration': PhysxSchema.PhysxSceneAPI(scene_prim).GetEnableExternalForcesEveryIterationAttr().Get(),
            'stabilization': PhysxSchema.PhysxSceneAPI(scene_prim).GetEnableStabilizationAttr().Get(),
            'position_iterations': articulation_settings_prim.GetAttribute(
                'physxArticulation:solverPositionIterationCount').Get(),
            'velocity_iterations': articulation_settings_prim.GetAttribute(
                'physxArticulation:solverVelocityIterationCount').Get(),
            'solve_articulation_contact_last': scene_prim.GetAttribute(
                'physxScene:solveArticulationContactLast').Get(),
            'workpiece_velocity_iterations': PhysxSchema.PhysxRigidBodyAPI(
                block_prim).GetSolverVelocityIterationCountAttr().Get(),
        }

        contact_sensors = None
        legacy_contact_sensor = False
        if args.contact_observability:
            try:
                from isaacsim.sensors.experimental.physics import (
                    Contact, ContactSensor)

                def create_contact_sensor(path):
                    return ContactSensor(Contact.create(
                        path, min_threshold=0.0,
                        max_threshold=100000.0, radius=-1.0))
            except ModuleNotFoundError:
                from isaacsim.sensors.physics import ContactSensor
                legacy_contact_sensor = True

                def create_contact_sensor(path):
                    return ContactSensor(
                        prim_path=path, min_threshold=0.0,
                        max_threshold=100000.0, radius=-1.0)
            contact_sensors = [create_contact_sensor(
                f'{path}/onrobot_payload_contact_sensor')
                for path in fingertip_paths]
            for sensor in contact_sensors:
                sensor.add_raw_contact_data_to_frame()

        # Flush the test-layer relationship and transform edits before PhysX
        # parses the stage. This is required by Isaac Sim 5's Fabric bridge.
        app.update()
        app.update()

        if args.gui:
            from isaacsim.core.rendering_manager import ViewportManager
            ViewportManager.set_camera_view(
                '/OmniverseKit_Persp', eye=[0.45, -0.35, 0.32],
                target=block_position)
        physics = config['physics']
        dt = float(physics['physics_dt_s'])
        simulation_step_index = 0
        last_commanded_joint_target = None
        trajectory_samples = []
        trajectory_truncated = False
        actual_dt = setup_simulation(SimulationManager, dt=dt, device='cpu')
        if not math.isclose(actual_dt, dt, rel_tol=1e-9):
            raise RuntimeError('effective physics timestep differs from request')
        # Finish schema edits before creating the articulation tensor view.
        if control == 'preload-position' or args.diagnostic_contact_vectors:
            pad_contacts = PadContactObserver(BLOCK_PATH, fingertip_paths, dt,
                                              capture_vectors=args.diagnostic_contact_vectors)
            pad_contacts.subscribe(stage)
        block = RigidPrim(BLOCK_PATH)
        fingertips = RigidPrim(fingertip_paths)
        articulation = Articulation(runtime_articulation_path)
        play()
        for _ in range(4):
            SimulationManager.step()
            app.update()
        # Only step() owns measured physics time. Kit updates may render or
        # rebuild views, but must not introduce an extra gravity/load step.
        omni.timeline.get_timeline_interface().pause()
        app.update()
        manual_step_started = float(SimulationManager.get_simulation_time())
        fabric_enabled = SimulationManager.is_fabric_enabled()
        render_stride = max(1, round(1 / (60 * dt))) if args.gui else None
        report['gui_render_stride'] = render_stride
        if legacy_contact_sensor:
            for sensor in contact_sensors or []:
                sensor.initialize()
        joint_name = driven_joint(contract)
        dof_names = articulation.dof_names
        if not dof_names:
            raise RuntimeError(
                f'PhysX did not create an articulation at '
                f'{runtime_articulation_path}')
        if joint_name not in dof_names:
            raise RuntimeError(f'{joint_name} is absent from the articulation')
        joint_index = dof_names.index(joint_name)
        articulation_view_reacquires = 0

        def current_block_view():
            """Reacquire the RigidPrim view after a body-state edit."""
            nonlocal block
            if not block.valid:
                block = RigidPrim(BLOCK_PATH)
            if not block.valid:
                raise RuntimeError(
                    f'workpiece prim became unavailable: {BLOCK_PATH}')
            return block

        def current_articulation_view():
            """Reacquire the PhysX view after an Isaac 6 stage edit.

            Toggling support collision after acquisition can invalidate the
            experimental tensor entity while leaving the USD articulation
            intact. Treat that wrapper lifecycle event like the dynamic-body
            view transition below; do not mistake it for an asset failure.
            """
            nonlocal articulation, articulation_view_reacquires
            is_valid = getattr(
                articulation, 'is_physics_tensor_entity_valid', None)
            if callable(is_valid):
                try:
                    valid = bool(is_valid())
                except Exception:
                    valid = False
                if not valid:
                    articulation = Articulation(runtime_articulation_path)
                    articulation_view_reacquires += 1
                    if articulation_view_reacquires > 4:
                        raise RuntimeError('payload articulation repeatedly invalidated')
                    is_valid = getattr(
                        articulation, 'is_physics_tensor_entity_valid', None)
                    if callable(is_valid) and not is_valid():
                        raise RuntimeError(
                            'PhysX articulation tensor view remained invalid '
                            f'after reacquiring {runtime_articulation_path}')
            return articulation

        def step(target, force=None):
            nonlocal simulation_step_index, last_commanded_joint_target
            if pad_contacts is not None:
                pad_contacts.begin_step()
            last_commanded_joint_target = float(target)
            active_block = current_block_view()
            if force is not None and any(value != 0.0 for value in force):
                active_block.apply_forces([force])
            active_articulation = current_articulation_view()
            active_articulation.set_dof_position_targets(
                [float(target)], dof_indices=[joint_index])
            SimulationManager.step(update_fabric=fabric_enabled)
            if render_stride is not None and simulation_step_index % render_stride == 0:
                app.update()
            if pad_contacts is not None:
                pad_contacts.end_step()
            simulation_step_index += 1
            expected_time = manual_step_started + simulation_step_index * dt
            if abs(float(SimulationManager.get_simulation_time()) - expected_time) > 1e-5:
                raise RuntimeError('payload physics time differs from explicit steps')

        open_target = float(profile['open_target'])
        grasp_target = float(profile['grasp_target'])
        initial_joint = _row(current_articulation_view().get_dof_positions())[joint_index]
        for target in _smooth_joint_targets(initial_joint, open_target, math.ceil(2.0 / dt)):
            step(target)
        for _ in range(int(physics['settle_steps'])):
            step(open_target)
        block.set_world_poses(
            positions=[block_position],
            orientations=[[1.0, 0.0, 0.0, 0.0]])
        block_kinematic.Set(False)
        block_ccd.Set(True)
        app.update()
        block.set_velocities(
            linear_velocities=[[0.0, 0.0, 0.0]],
            angular_velocities=[[0.0, 0.0, 0.0]])
        block_collision.Set(True)
        app.update()
        for _ in range(4):
            step(open_target)
        if args.diagnostic_contact_vectors:
            for _ in range(math.ceil(.25 / dt)):
                step(open_target)
            report['supported_contact_vectors'] = pad_contacts.vector_sample()
        report['native_payload_inertia'] = {
            'mass_kg': _row(current_block_view().get_masses())[0],
            'inertia_kg_m2': _row(current_block_view().get_inertias()),
            'size_world_m': block_size_world,
            'basis': 'native-dynamic-body-after-placement',
        }
        for target in _smooth_joint_targets(open_target, grasp_target, int(physics['close_steps'])):
            step(target)
        if control == 'preload-position':
            preload = contact_preload_reference(current_articulation_view(), joint_index, pad_contacts)
            report['impedance_preload'] = preload
            preload_target = preload['drive_reference_m']
            for target in _smooth_joint_targets(grasp_target, preload_target, math.ceil(.25 / dt)):
                step(target)
            grasp_target = preload_target
        for _ in range(int(physics['preload_steps'])):
            step(grasp_target)
        if args.diagnostic_contact_vectors:
            report['acquired_contact_vectors'] = pad_contacts.vector_sample()

        active_articulation = current_articulation_view()
        acquired_joint = float(_array(
            active_articulation.get_dof_positions())[0, joint_index])
        acquired_projected_joint_force = _projected_joint_force(
            active_articulation, joint_index)
        acquired_dofs = _row(active_articulation.get_dof_positions())
        blocking_error = abs(acquired_joint - grasp_target)
        initial_block_view = current_block_view()
        initial_block_position = _row(initial_block_view.get_world_poses()[0])
        initial_block_orientation = _row(initial_block_view.get_world_poses()[1])
        tip_poses = fingertips.get_world_poses()
        tip_positions = _array(tip_poses[0])
        tip_orientations = _array(tip_poses[1])
        report['acquisition_geometry'] = {
            'block_position_m': initial_block_position,
            'fingertip_positions_m': [
                [float(value) for value in row] for row in tip_positions],
            'fingertip_orientations_wxyz': [
                [float(value) for value in row] for row in tip_orientations],
        }
        initial_relative = _relative_pose(
            carrier_position, carrier_orientation,
            initial_block_position, initial_block_orientation)
        UsdPhysics.CollisionAPI(
            support_prim).GetCollisionEnabledAttr().Set(False)
        UsdGeom.Imageable(support_prim).MakeInvisible()
        maximum_relative_translation = 0.0
        maximum_relative_rotation = 0.0
        maximum_speed = 0.0
        maximum_joint_drift = 0.0
        maximum_linkage_drift = 0.0
        maximum_absolute_projected_joint_force = abs(
            acquired_projected_joint_force
            if acquired_projected_joint_force is not None else 0.0)
        state_finite = True
        contact_samples = [[], []]
        summed_normal_force_samples = []

        def observe():
            nonlocal maximum_relative_translation, maximum_relative_rotation
            nonlocal maximum_speed, maximum_joint_drift, state_finite
            nonlocal maximum_linkage_drift
            nonlocal maximum_absolute_projected_joint_force
            nonlocal trajectory_truncated
            block_view = current_block_view()
            object_position = _row(block_view.get_world_poses()[0])
            object_orientation = _row(block_view.get_world_poses()[1])
            relative = _relative_pose(
                carrier_position, carrier_orientation,
                object_position, object_orientation)
            maximum_relative_translation = max(
                maximum_relative_translation,
                _norm([relative[0][i] - initial_relative[0][i]
                       for i in range(3)]))
            maximum_relative_rotation = max(
                maximum_relative_rotation,
                _rotation_error(relative[1], initial_relative[1]))
            linear_velocity = _linear_velocity(block_view.get_velocities())
            maximum_speed = max(maximum_speed, _norm(linear_velocity))
            active_articulation = current_articulation_view()
            joint_position = float(_array(
                active_articulation.get_dof_positions())[0, joint_index])
            maximum_joint_drift = max(
                maximum_joint_drift, abs(joint_position - acquired_joint))
            dofs = _row(active_articulation.get_dof_positions())
            maximum_linkage_drift = max(
                maximum_linkage_drift,
                max((abs(dofs[i] - acquired_dofs[i])
                     for i in range(len(dofs)) if i != joint_index),
                    default=0.0))
            projected_force = _projected_joint_force(
                active_articulation, joint_index)
            if projected_force is not None:
                maximum_absolute_projected_joint_force = max(
                    maximum_absolute_projected_joint_force,
                    abs(projected_force))
            if contact_sensors is not None:
                samples = [_contact_observability_sample(
                    sensor, BLOCK_PATH, dt) for sensor in contact_sensors]
                for collected, sample in zip(contact_samples, samples):
                    collected.append(sample)
                summed_normal_force_samples.append(sum(
                    sample['normal_force_from_impulse_n']
                    for sample in samples))
            values = object_position + object_orientation + dofs
            state_finite = state_finite and all(
                math.isfinite(v) for v in values)
            if len(trajectory_samples) < MAX_TRAJECTORY_SAMPLES:
                trajectory_samples.append({
                    'simulation_step': simulation_step_index,
                    'time_s': simulation_step_index * dt,
                    'commanded_joint_position': (
                        last_commanded_joint_target),
                    'actual_joint_position': joint_position,
                    'dof_positions': dofs,
                    'actual_joint_velocity': _row(active_articulation.get_dof_velocities())[joint_index],
                    'pad_normal_magnitudes_n': list(pad_contacts.normals) if pad_contacts else None,
                    **({'contact_vectors': pad_contacts.vector_sample()}
                       if args.diagnostic_contact_vectors else {}),
                    'object_position_m': object_position,
                    'object_orientation_wxyz': object_orientation,
                    'object_linear_velocity_m_s': linear_velocity,
                    'relative_position_m': relative[0],
                    'relative_orientation_wxyz': relative[1],
                })
            else:
                trajectory_truncated = True

        dynamic_mass = (
            payload_mass if args.scenario == 'dynamic-showcase' else 0.0)
        diagonal_force = [
            dynamic_mass * float(v)
            for v in fixture['diagonal_hold_acceleration_m_s2']]
        motion_origin = list(carrier_position)
        lift_displacement = (
            [float(v) for v in fixture['dynamic_lift_displacement_m']]
            if args.scenario == 'dynamic-showcase' else [0.0, 0.0, 0.0])
        for index in range(int(physics['lift_steps'])):
            blend = (index + 1) / float(physics['lift_steps'])
            force = [blend * value for value in diagonal_force]
            step(grasp_target, force)
            observe()

        shake_steps = int(physics['shake_steps'])
        cycles = int(fixture['shake_cycles'])
        amplitude = [
            dynamic_mass * float(v)
            for v in fixture['shake_acceleration_amplitude_m_s2']]
        shake_displacement = (
            [float(v) for v in
             fixture['dynamic_shake_displacement_amplitude_m']]
            if args.scenario == 'dynamic-showcase' else [0.0, 0.0, 0.0])
        lifted_position = [
            motion_origin[i] + lift_displacement[i] for i in range(3)]
        for index in range(shake_steps):
            phase = 2.0 * math.pi * cycles * index / max(1, shake_steps - 1)
            force = [
                diagonal_force[0] + amplitude[0] * math.sin(phase),
                diagonal_force[1] + amplitude[1] * math.sin(phase * 1.5),
                diagonal_force[2] + amplitude[2] * math.sin(phase * 2.0),
            ]
            step(grasp_target, force)
            observe()

        if args.recording_dir is not None and not args.gui:
            raise RuntimeError('--recording-dir requires --gui')
        if args.recording_dir is not None and \
                args.scenario != 'dynamic-showcase':
            raise RuntimeError(
                '--recording-dir requires --scenario dynamic-showcase')

        presentation = {'enabled': False}
        if args.gui and args.scenario == 'dynamic-showcase':
            # Replay the declared carrier path with physics stopped. The
            # qualification above remains a fixed-base force/load test; this
            # second phase exists only to make the same path easy to inspect
            # and record without changing the evidence measurements.
            stop()
            block_transform = UsdGeom.XformCommonAPI(block_prim)
            relative_offset = _rotate(
                carrier_orientation, initial_relative[0])

            renderer_capture = None
            recording_dir = None
            recorded_frames = []
            if args.recording_dir is not None:
                import omni.kit.renderer.capture
                recording_dir = args.recording_dir.resolve()
                recording_dir.mkdir(parents=True, exist_ok=True)
                if any(recording_dir.glob('frame_*.png')):
                    raise RuntimeError(
                        'recording directory already contains frame_*.png')
                renderer_capture = (
                    omni.kit.renderer.capture
                    .acquire_renderer_capture_interface())

            def present(position, frame_index):
                asset_translate.Set(Gf.Vec3d(*position))
                object_position = [
                    position[i] + relative_offset[i] for i in range(3)]
                block_transform.SetTranslate(Gf.Vec3d(*object_position))
                if renderer_capture is not None and frame_index % 2 == 0:
                    frame_path = recording_dir / (
                        f'frame_{len(recorded_frames):04d}.png')
                    renderer_capture.capture_next_frame_swapchain(
                        str(frame_path))
                app.update()
                if renderer_capture is not None and frame_index % 2 == 0:
                    renderer_capture.wait_async_capture()
                    recorded_frames.append(str(frame_path))
                time.sleep(1.0 / 60.0)

            presentation_lift_frames = 120
            for index in range(presentation_lift_frames):
                blend = (index + 1) / presentation_lift_frames
                smooth_blend = blend * blend * (3.0 - 2.0 * blend)
                present([
                    motion_origin[i] + smooth_blend * lift_displacement[i]
                    for i in range(3)], index)
            presentation_shake_frames = 240
            for index in range(presentation_shake_frames):
                phase = (2.0 * math.pi * cycles * index /
                         max(1, presentation_shake_frames - 1))
                present([
                    lifted_position[0] +
                    shake_displacement[0] * math.sin(phase),
                    lifted_position[1] +
                    shake_displacement[1] * math.sin(phase * 1.5),
                    lifted_position[2] +
                    shake_displacement[2] * math.sin(phase * 2.0),
                ], presentation_lift_frames + index)
            presentation = {
                'enabled': True,
                'evidence_role': 'visualization-only',
                'frames_rendered': (
                    presentation_lift_frames + presentation_shake_frames),
                'duration_s': (
                    presentation_lift_frames + presentation_shake_frames) /
                60.0,
                'recording': {
                    'frame_rate_hz': 30,
                    'frame_count': len(recorded_frames),
                    'directory': (str(recording_dir)
                                  if recording_dir is not None else None),
                    'ffmpeg_command': (
                        'ffmpeg -framerate 30 -i ' + shlex.quote(str(
                            recording_dir / 'frame_%04d.png')) +
                        ' -c:v libx264 -pix_fmt yuv420p ' + shlex.quote(str(
                            recording_dir / 'showcase.mp4'))
                        if recording_dir is not None else None),
                },
            }

        checks = {
            'joint_blocked_before_commanded_endpoint': blocking_error >= float(
                profile['minimum_blocking_error']),
            'payload_relative_translation_bounded': (
                maximum_relative_translation <= float(
                    fixture['maximum_relative_translation_error_m'])),
            'payload_relative_rotation_bounded': (
                maximum_relative_rotation <= float(
                    fixture['maximum_relative_rotation_error_rad'])),
            'payload_speed_bounded': maximum_speed <= float(
                fixture['maximum_object_speed_m_s']),
            'gripper_joint_stable': maximum_joint_drift <= float(
                fixture['maximum_joint_drift']),
            'gripper_linkage_stable': maximum_linkage_drift <= float(
                profile['maximum_linkage_drift']),
            'articulation_and_payload_state_finite': state_finite,
        }
        report['asset_files_sha256'] = _asset_manifest(asset)
        report['physics_backend'] = 'physx'
        report['payload_mass_kg'] = payload_mass
        report['rated_force_fit_payload_kg'] = rated_payload_mass
        report['payload_rating_source'] = profile['payload_rating_source']
        report['payload_basis'] = (
            'configured-fraction-of-datasheet-maximum-force-fit-payload')
        report['payload_fraction_of_rating'] = (
            payload_mass / rated_payload_mass)
        report['payload_size_m'] = block_size_local
        report['payload_size_source'] = workpiece_size_source
        report['workpiece_definition'] = {
            'size_m': block_size_local,
            'mass_kg': payload_mass,
            'mass_source': (
                'cli' if args.payload_kg is not None else 'model-profile'),
        }
        report['fingertip_material_bindings'] = fingertip_bindings
        report['expected_asset_fingertip_material'] = {
            'static_friction': float(profile.get(
                'fingertip_static_friction',
                fixture['fingertip_static_friction'])),
            'dynamic_friction': float(profile.get(
                'fingertip_dynamic_friction',
                fixture['fingertip_dynamic_friction'])),
            'friction_combine_mode': asset_fingertip_combine_mode,
        }
        report['fixture_fingertip_material_override'] = (
            fixture_fingertip_material)
        report['fixture_drive_override'] = fixture_drive
        report['fixture_solver_override'] = fixture_solver
        report['runtime_articulation_view_reacquires'] = (
            articulation_view_reacquires)
        report['effective_solver_settings'] = effective_solver_settings
        report['motion'] = {
            'mount': 'sideways-horizontal-external-pinch',
            'diagonal_hold_force_n': diagonal_force,
            'shake_force_amplitude_n': amplitude,
            'lift_displacement_m': lift_displacement,
            'shake_displacement_amplitude_m': shake_displacement,
            'shake_cycles': cycles,
        }
        report['trajectory'] = {
            'manual_step_count': simulation_step_index,
            'manual_step_timing_error_s': (
                float(SimulationManager.get_simulation_time()) - manual_step_started
                - simulation_step_index * dt) if not presentation['enabled'] else None,
            'sample_count': len(trajectory_samples),
            'sample_cap': MAX_TRAJECTORY_SAMPLES,
            'truncated': trajectory_truncated,
            'sample_period_s': dt,
            'samples': trajectory_samples,
        }
        if args.contact_observability:
            per_tip = []
            for samples in contact_samples:
                per_tip.append({
                    'sensor_force_magnitude_n': _series_report([
                        sample['sensor_force_magnitude_n']
                        for sample in samples]),
                    'normal_force_from_impulse_n': _series_report([
                        sample['normal_force_from_impulse_n']
                        for sample in samples]),
                    'impulse_vector_ns': _vector_series_report([
                        sample['impulse_vector_ns'] for sample in samples]),
                    'impulse_vector_over_dt_n': _vector_series_report([
                        sample['impulse_vector_over_dt_n']
                        for sample in samples]),
                    'object_contact_points': _series_report([
                        sample['object_contact_points'] for sample in samples]),
                    'all_sensor_samples_valid': all(
                        sample['sensor_valid'] for sample in samples),
                    'raw_contact_data_available': all(
                        sample['raw_contact_data_available']
                        for sample in samples),
                })
            report['contact_observability'] = {
                'enabled': True,
                'reference_condition': {
                    'material_override': fixture_fingertip_material,
                    'scope': 'effective-composed-materials',
                    'stock_rubber_geometry_retained': True,
                },
                'per_tip': per_tip,
                'summed_normal_force_from_impulse_n': _series_report(
                    summed_normal_force_samples),
                'driven_joint_reaction': {
                    'unit': ('N' if args.model.startswith('2fg') else 'Nm'),
                    'acquired': acquired_projected_joint_force,
                    'maximum_absolute': (
                        maximum_absolute_projected_joint_force),
                },
                'public_tool_api_force': {
                    'available': False,
                    'reason': ('pure Isaac execution has no ROS/Tool API '
                               'hardware session'),
                },
                'interpretation': (
                    'Sensor magnitude, normal impulse divided by dt, full '
                    'impulse vector and generalized drive reaction are '
                    'reported separately and must not be equated.'),
            }
        report['presentation'] = presentation
        report['tests'] = [{
            'name': (
                'external_pinch_full_rated_static_retention'
                if args.scenario == 'rated-margin-static'
                else 'external_pinch_full_rated_diagonal_load_and_shake'),
            'status': 'passed' if all(checks.values()) else 'failed',
            'checks': checks,
            'measurements': {
                'acquired_joint_position': acquired_joint,
                'commanded_grasp_target': grasp_target,
                'blocking_error': blocking_error,
                'maximum_relative_translation_error_m': (
                    maximum_relative_translation),
                'maximum_relative_rotation_error_rad': (
                    maximum_relative_rotation),
                'maximum_payload_speed_m_s': maximum_speed,
                'maximum_joint_drift': maximum_joint_drift,
                'maximum_linkage_drift': maximum_linkage_drift,
                'acquired_projected_joint_force': (
                    acquired_projected_joint_force),
                'maximum_absolute_projected_joint_force': (
                    maximum_absolute_projected_joint_force),
            },
        }]
        report['status'] = report['tests'][0]['status']
    except KeyboardInterrupt:
        report['status'] = 'canceled'
        report['error'] = 'KeyboardInterrupt: canceled by operator'
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = f'{type(error).__name__}: {error}'
        report['traceback'] = traceback.format_exc().splitlines()
    finally:
        if pad_contacts is not None:
            pad_contacts.close()
        _write(args.output, report)
        print(json.dumps(report, indent=2, sort_keys=True), flush=True)
        exit_code = 0 if report['status'] == 'passed' else 1
        if app is not None:
            try:
                stop()
            except Exception:
                pass
            try:
                app.close(exit_code=exit_code)
            except TypeError:
                # Isaac Sim 5 has no exit_code argument.
                app.close()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())

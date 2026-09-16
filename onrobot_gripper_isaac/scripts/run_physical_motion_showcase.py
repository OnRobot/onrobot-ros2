#!/usr/bin/env python3
"""Run a physically actuated full-payload showcase for an OnRobot gripper."""

import argparse
import asyncio
import hashlib
import json
import math
from pathlib import Path
import shlex
import sys
import traceback

from isaac_model_contract import (
    articulation_path, default_asset, driven_joint, load_contract,
    SUPPORTED_MODELS,
)

from isaac_runtime_compat import close_app, is_stage_loading
from isaac_runtime_compat import play, setup_simulation, stop
from grip_drive_control import PadContactObserver, contact_preload_reference, hold_control, script_manifest

from run_payload_retention_test import _asset_manifest, _collision_material_scope
from run_payload_retention_test import _define_cube
from run_payload_retention_test import _norm, _relative_pose, _rotation_error, _smooth_joint_targets
from run_payload_retention_test import _rotate, _row

from showcase_branding import create_showcase_sign, create_studio_lighting, define_textured_sign, ONROBOT_BLUE


HARNESS_REVISION = 14
MOUNT_DRIVE_STIFFNESS = 80000.0
MOUNT_DRIVE_DAMPING = 1500.0
MOUNT_DRIVE_MAXIMUM_FORCE = 5000.0
LIFT_DISTANCE_M = 0.075
BACKWARD_LIFT_DISTANCE_M = 0.040
LIFT_DURATION_S = 1.0
BACKWARD_SHAKE_AMPLITUDE_M = 0.025
VERTICAL_SHAKE_AMPLITUDE_M = 0.010
SHAKE_FREQUENCY_HZ = 2.0
SHAKE_CYCLES = 5
GUI_RENDER_RATE_HZ = 60.0
RECORDING_RATE_HZ = 30.0
MAX_TRAJECTORY_SAMPLES = 8000
CAMERA_SELECTION_SETTLE_FRAMES = 30
CAMERA_EYE_OFFSET_M = [0.50, -0.03, 0.045]
CAMERA_TARGET_OFFSET_M = [0.0, 0.115, 0.045]
# Keep the full tool and branded sign inside the frame at both shake extrema.
CAMERA_FOCAL_LENGTH_MM = 22.0


def _define_textured_sign(stage, path, texture_path, center, size,
                          Gf, Sdf, UsdGeom, UsdShade, Vt):
    """Define a non-colliding YZ sign facing the showcase camera."""
    return define_textured_sign(stage, path, texture_path, center, size)


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


def _portable_media_path(path: Path, report_path: Path) -> str:
    """Serialize an evidence path relative to the JSON report directory."""
    report_root = report_path.resolve().parent
    resolved = path.resolve()
    try:
        return resolved.relative_to(report_root).as_posix()
    except ValueError as error:
        raise RuntimeError(
            f'media path must be below report directory {report_root}: '
            f'{path}') from error


def _recording_ffmpeg_command(recording_dir: Path, report_path: Path) -> str:
    """Build a portable encoding command for the recorded frame sequence."""
    input_path = _portable_media_path(
        recording_dir / 'frame_%05d.png', report_path)
    output_path = _portable_media_path(
        recording_dir / 'showcase.mp4', report_path)
    return ('ffmpeg -y -framerate 30 -i ' + shlex.quote(input_path) +
            ' -c:v libx264 -pix_fmt yuv420p -crf 18 ' +
            shlex.quote(output_path))


def _linear_velocity(value):
    if isinstance(value, tuple):
        return _row(value[0])
    return _row(value)[:3]


def main(app_factory=None) -> int:
    """Run a selected gripper showcase and write a machine-readable result."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=SUPPORTED_MODELS, required=True)
    parser.add_argument('--asset', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--payload-kg', type=float)
    parser.add_argument('--hold-control', choices=('position', 'preload-position'),
                        help='override the model profile holding command (no physics changes)')
    parser.add_argument(
        '--screenshot-dir', type=Path,
        help='capture grasp, lift, and shake-extrema viewport frames')
    parser.add_argument(
        '--recording-dir', type=Path,
        help='capture the live PhysX run as a 30 fps PNG sequence')
    parser.add_argument(
        '--apply-profile-physics-settings', action='store_true',
        help=(
            'opt in to profile physics overrides for bounded synthetic '
            'data collection; ordinary showcases use composed asset values'))
    args, _ = parser.parse_known_args()

    model = args.model
    scene_root = f'/onrobot_{model}_physical_showcase'
    block_path = f'{scene_root}/block'
    support_path = f'{scene_root}/support'
    camera_path = f'{scene_root}/camera'
    root_prim_path = f'/onrobot_{model}'
    lift_joint_path = f'{scene_root}/Physics/showcase_lift_joint'
    backward_joint_path = f'{scene_root}/Physics/showcase_backward_joint'
    mount_root_joint_path = f'{scene_root}/Physics/showcase_mount_root_joint'
    mount_link_path = f'{scene_root}/ShowcaseMount'
    backward_carriage_path = f'{scene_root}/ShowcaseBackwardCarriage'

    contract = load_contract(model)
    asset = args.asset or default_asset(model)
    config_path = args.config or _package_path(
        'config/payload_retention_profiles.json')
    report = {
        'schema_version': 1,
        'harness_revision': HARNESS_REVISION,
        'test': 'physically-actuated-full-payload-lift-and-shake',
        'model': model,
        'asset': str(asset.resolve()),
        'config': str(config_path.resolve()),
        'status': 'starting',
        'tests': [],
    }
    _write(args.output, report)
    app = None
    pad_contacts = None
    try:
        config_bytes = config_path.read_bytes()
        config = json.loads(config_bytes.decode('utf-8'))
        report['config_sha256'] = hashlib.sha256(config_bytes).hexdigest()
        report['runner_sha256'] = hashlib.sha256(
            Path(__file__).resolve().read_bytes()).hexdigest()
        report['runtime_script_files_sha256'] = script_manifest(__file__)
        print(
            'OnRobot physical-motion runner: '
            f'{Path(__file__).resolve()}\n'
            f"Runner SHA-256: {report['runner_sha256']}",
            flush=True,
        )
        fixture = config['fixture']
        profile = config['models'][model]
        control = hold_control(model, profile, args.hold_control)
        report['hold_control'] = control
        report['hold_control_source'] = 'cli' if args.hold_control is not None else 'model-profile'
        report['closure_target'] = float(profile['grasp_target'])
        rated_payload = float(profile['rated_force_fit_payload_kg'])
        payload_mass = float(
            rated_payload if args.payload_kg is None else args.payload_kg)
        if (not math.isfinite(payload_mass) or payload_mass <= 0.0 or
                payload_mass > rated_payload):
            raise RuntimeError(
                'payload must be finite, positive, and no greater than the '
                'published force-fit rating')
        render_output = (args.gui or args.screenshot_dir is not None or
                         args.recording_dir is not None)
        if app_factory is None:
            from isaacsim import SimulationApp
            app_factory = SimulationApp
        app = app_factory({
            'headless': not args.gui,
            'renderer': ('RayTracedLighting' if render_output
                         else 'MinimalRendering'),
            'disable_viewport_updates': not render_output,
            'multi_gpu': False,
            'width': 1280,
            'height': 720,
        })
        import omni.timeline
        import omni.usd
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.core.version import get_version
        from pxr import (Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux,
                         UsdPhysics, UsdShade, Vt)

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
        UsdGeom.Xform.Define(stage, scene_root)
        report['lighting'] = create_studio_lighting(stage, f'{scene_root}/studio_light')

        carrier_position = [float(v) for v in fixture['carrier_position_m']]
        carrier_orientation = [
            float(v) for v in fixture['carrier_orientation_wxyz']]
        asset_root = UsdGeom.Xformable(stage.GetPrimAtPath(root_prim_path))
        asset_root.ClearXformOpOrder()
        asset_root.AddTranslateOp().Set(Gf.Vec3d(*carrier_position))
        asset_root.AddOrientOp().Set(Gf.Quatf(
            carrier_orientation[0], *carrier_orientation[1:]))

        original_root_path = articulation_path(contract)
        original_root_prim = stage.GetPrimAtPath(original_root_path)
        if not original_root_prim.IsValid():
            raise RuntimeError(
                f'asset root joint is missing: {original_root_path}')
        if original_root_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            original_root_prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if original_root_prim.IsA(UsdPhysics.Joint):
            UsdPhysics.Joint(
                original_root_prim).CreateJointEnabledAttr().Set(False)
            original_root_prim.SetActive(False)
        else:
            # RG assets place ArticulationRootAPI on the model root and use a
            # separate root_joint as the fixed-base mount. Disable that joint
            # while keeping the referenced model root active.
            fixed_root_prim = stage.GetPrimAtPath(
                f'{root_prim_path}/Physics/root_joint')
            if not fixed_root_prim.IsValid() or not fixed_root_prim.IsA(
                    UsdPhysics.Joint):
                raise RuntimeError(
                    'model-root articulation has no fixed root_joint')
            UsdPhysics.Joint(
                fixed_root_prim).CreateJointEnabledAttr().Set(False)
            fixed_root_prim.SetActive(False)

        mount_link = UsdGeom.Xform.Define(stage, mount_link_path).GetPrim()
        mount_xform = UsdGeom.Xformable(mount_link)
        mount_xform.AddTranslateOp().Set(Gf.Vec3d(*carrier_position))
        mount_xform.AddOrientOp().Set(Gf.Quatf(
            carrier_orientation[0], *carrier_orientation[1:]))
        UsdPhysics.RigidBodyAPI.Apply(
            mount_link).CreateRigidBodyEnabledAttr().Set(True)
        UsdPhysics.MassAPI.Apply(mount_link).CreateMassAttr().Set(1.0)

        backward_carriage = UsdGeom.Xform.Define(
            stage, backward_carriage_path).GetPrim()
        backward_carriage_xform = UsdGeom.Xformable(backward_carriage)
        backward_carriage_xform.AddTranslateOp().Set(
            Gf.Vec3d(*carrier_position))
        backward_carriage_xform.AddOrientOp().Set(Gf.Quatf(
            carrier_orientation[0], *carrier_orientation[1:]))
        UsdPhysics.RigidBodyAPI.Apply(
            backward_carriage).CreateRigidBodyEnabledAttr().Set(True)
        UsdPhysics.MassAPI.Apply(
            backward_carriage).CreateMassAttr().Set(1.0)

        mount_root_joint = UsdPhysics.FixedJoint.Define(
            stage, mount_root_joint_path)
        mount_root_joint.CreateBody1Rel().SetTargets([mount_link_path])
        mount_root_joint.CreateLocalPos0Attr().Set(
            Gf.Vec3f(*carrier_position))
        mount_root_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        mount_root_joint.CreateLocalRot0Attr().Set(Gf.Quatf(
            carrier_orientation[0], *carrier_orientation[1:]))
        mount_root_joint.CreateLocalRot1Attr().Set(
            Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        UsdPhysics.ArticulationRootAPI.Apply(mount_root_joint.GetPrim())
        # The mounting articulation owns solver settings after composition.
        # Retain the supplied product settings instead of silently using Kit
        # defaults on the new root.
        original_api = PhysxSchema.PhysxArticulationAPI(original_root_prim)
        mount_api = PhysxSchema.PhysxArticulationAPI.Apply(mount_root_joint.GetPrim())
        inherited_solver = {}
        for name in ('SolverPositionIterationCount', 'SolverVelocityIterationCount',
                     'EnabledSelfCollisions'):
            value = getattr(original_api, 'Get' + name + 'Attr')().Get()
            if value is not None:
                getattr(mount_api, 'Create' + name + 'Attr')().Set(value)
                inherited_solver[name] = value
        report['mount_inherited_articulation_settings'] = inherited_solver

        # In this fixture the gripper's local -Z direction is world +Y,
        # which is backward toward the robot-side base. A dedicated joint
        # allows the showcase to combine backward and vertical motion.
        backward_joint = UsdPhysics.PrismaticJoint.Define(
            stage, backward_joint_path)
        backward_joint.CreateBody0Rel().SetTargets([mount_link_path])
        backward_joint.CreateBody1Rel().SetTargets(
            [backward_carriage_path])
        backward_joint.CreateAxisAttr().Set('Z')
        backward_joint.CreateLowerLimitAttr().Set(-0.08)
        backward_joint.CreateUpperLimitAttr().Set(0.01)
        backward_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        backward_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        backward_joint.CreateLocalRot0Attr().Set(
            Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        backward_joint.CreateLocalRot1Attr().Set(
            Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        backward_drive = UsdPhysics.DriveAPI.Apply(
            backward_joint.GetPrim(), 'linear')
        backward_drive.CreateTypeAttr().Set('force')
        backward_drive.CreateStiffnessAttr().Set(MOUNT_DRIVE_STIFFNESS)
        backward_drive.CreateDampingAttr().Set(MOUNT_DRIVE_DAMPING)
        backward_drive.CreateMaxForceAttr().Set(MOUNT_DRIVE_MAXIMUM_FORCE)
        backward_drive.CreateTargetPositionAttr().Set(0.0)
        backward_drive.CreateTargetVelocityAttr().Set(0.0)

        lift_joint = UsdPhysics.PrismaticJoint.Define(
            stage, lift_joint_path)
        lift_joint.CreateBody0Rel().SetTargets([backward_carriage_path])
        lift_joint.CreateBody1Rel().SetTargets(
            [f'{root_prim_path}/Geometry/base_link'])
        lift_joint.CreateAxisAttr().Set('Y')
        lift_joint.CreateLowerLimitAttr().Set(-0.01)
        lift_joint.CreateUpperLimitAttr().Set(0.12)
        lift_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        lift_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        lift_joint.CreateLocalRot0Attr().Set(
            Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        lift_joint.CreateLocalRot1Attr().Set(
            Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        lift_drive = UsdPhysics.DriveAPI.Apply(
            lift_joint.GetPrim(), 'linear')
        lift_drive.CreateTypeAttr().Set('force')
        lift_drive.CreateStiffnessAttr().Set(MOUNT_DRIVE_STIFFNESS)
        lift_drive.CreateDampingAttr().Set(MOUNT_DRIVE_DAMPING)
        lift_drive.CreateMaxForceAttr().Set(MOUNT_DRIVE_MAXIMUM_FORCE)
        lift_drive.CreateTargetPositionAttr().Set(0.0)
        lift_drive.CreateTargetVelocityAttr().Set(0.0)
        lift_path_length = math.hypot(
            BACKWARD_LIFT_DISTANCE_M, LIFT_DISTANCE_M)
        lift_motion_axis_world = [
            0.0,
            BACKWARD_LIFT_DISTANCE_M / lift_path_length,
            LIFT_DISTANCE_M / lift_path_length,
        ]

        local_center = [float(v) for v in profile['contact_center_local_m']]
        rotated_center = _rotate(carrier_orientation, local_center)
        block_position = [carrier_position[i] + rotated_center[i]
                          for i in range(3)]
        block_size_local = [
            float(v) for v in profile.get(
                'block_size_m', fixture['block_size_m'])]
        block_size_world = [block_size_local[0], block_size_local[2],
                            block_size_local[1]]
        block_prim = _define_cube(
            stage, block_path, block_position, block_size_world,
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
        block_ccd.Set(False)

        material = UsdShade.Material.Define(
            stage, f'{scene_root}/block_material')
        material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        block_static_friction = float(profile.get(
            'block_static_friction', fixture['block_static_friction']))
        block_dynamic_friction = float(profile.get(
            'block_dynamic_friction', fixture['block_dynamic_friction']))
        if (not math.isfinite(block_static_friction) or
                not math.isfinite(block_dynamic_friction) or
                block_static_friction < 0.0 or block_dynamic_friction < 0.0):
            raise RuntimeError('block friction values must be finite and nonnegative')
        material_api.CreateStaticFrictionAttr().Set(
            block_static_friction)
        material_api.CreateDynamicFrictionAttr().Set(
            block_dynamic_friction)
        material_api.CreateRestitutionAttr().Set(0.0)
        UsdShade.MaterialBindingAPI.Apply(block_prim).Bind(
            material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose='physics')

        physics_overrides = None
        if args.apply_profile_physics_settings:
            fingertip_static_friction = float(profile.get(
                'fingertip_static_friction',
                fixture['fingertip_static_friction']))
            fingertip_dynamic_friction = float(profile.get(
                'fingertip_dynamic_friction',
                fixture['fingertip_dynamic_friction']))
            if (not math.isfinite(fingertip_static_friction) or
                    not math.isfinite(fingertip_dynamic_friction) or
                    fingertip_static_friction < 0.0 or
                    fingertip_dynamic_friction < 0.0):
                raise RuntimeError(
                    'fingertip friction values must be finite and nonnegative')
            combine_mode = str(profile.get(
                'fingertip_friction_combine_mode', 'average'))
            if combine_mode not in {'average', 'min', 'multiply', 'max'}:
                raise RuntimeError(
                    f'invalid fingertip friction combine mode: {combine_mode}')
            fingertip_material = UsdShade.Material.Define(
                stage, f'{scene_root}/fingertip_material')
            fingertip_api = UsdPhysics.MaterialAPI.Apply(
                fingertip_material.GetPrim())
            fingertip_api.CreateStaticFrictionAttr().Set(
                fingertip_static_friction)
            fingertip_api.CreateDynamicFrictionAttr().Set(
                fingertip_dynamic_friction)
            fingertip_api.CreateRestitutionAttr().Set(0.0)
            PhysxSchema.PhysxMaterialAPI.Apply(
                fingertip_material.GetPrim()).CreateFrictionCombineModeAttr().Set(
                    combine_mode)
            fingertip_bindings = []
            for link in profile.get('fingertip_links', []):
                link_prim = stage.GetPrimAtPath(
                    f'{root_prim_path}/Geometry/{link}')
                if not link_prim.IsValid():
                    raise RuntimeError(
                        f'fingertip link is missing for experiment: {link}')
                fingertip_bindings.append(_collision_material_scope(
                    link_prim, fingertip_material, Usd, UsdPhysics, UsdShade))

            joint_prim = stage.GetPrimAtPath(
                f'{root_prim_path}/Physics/{driven_joint(contract)}')
            if not joint_prim.IsValid():
                raise RuntimeError('driven joint is missing for experiment')
            drive_instance = 'linear' if model.startswith('2fg') else 'angular'
            drive = UsdPhysics.DriveAPI(joint_prim, drive_instance)
            drive_stiffness = float(profile.get(
                'fixture_drive_stiffness', drive.GetStiffnessAttr().Get()))
            drive_damping = float(profile.get(
                'fixture_drive_damping', drive.GetDampingAttr().Get()))
            drive_max_force = float(profile.get(
                'fixture_drive_max_force', drive.GetMaxForceAttr().Get()))
            if (not all(math.isfinite(value) for value in (
                    drive_stiffness, drive_damping, drive_max_force)) or
                    drive_stiffness < 0.0 or drive_damping < 0.0 or
                    drive_max_force < 0.0):
                raise RuntimeError(
                    'driven-joint experiment values must be finite and nonnegative')
            drive.CreateStiffnessAttr().Set(drive_stiffness)
            drive.CreateDampingAttr().Set(drive_damping)
            drive.CreateMaxForceAttr().Set(drive_max_force)
            physics_overrides = {
                'scope': 'synthetic-data-experiment-only',
                'block_static_friction': block_static_friction,
                'block_dynamic_friction': block_dynamic_friction,
                'fingertip_static_friction': fingertip_static_friction,
                'fingertip_dynamic_friction': fingertip_dynamic_friction,
                'fingertip_friction_combine_mode': combine_mode,
                'fingertip_bindings': fingertip_bindings,
                'driven_joint': str(joint_prim.GetPath()),
                'drive_instance': drive_instance,
                'drive_stiffness': drive_stiffness,
                'drive_damping': drive_damping,
                'drive_max_force': drive_max_force,
            }

        support_height = 0.01
        support_position = list(block_position)
        support_position[2] -= (
            block_size_world[2] / 2.0 + support_height / 2.0 +
            float(fixture['support_clearance_m']))
        support_prim = _define_cube(
            stage, support_path, support_position,
            [min(0.015, block_size_world[0] * 0.3),
             block_size_world[1] * 0.6, support_height],
            (0.25, 0.27, 0.30), Gf, UsdGeom, UsdPhysics)
        support_body_api = UsdPhysics.RigidBodyAPI.Apply(support_prim)
        support_body_api.CreateRigidBodyEnabledAttr().Set(True)
        support_body_api.CreateKinematicEnabledAttr().Set(True)
        support_collision = UsdPhysics.CollisionAPI(
            support_prim).GetCollisionEnabledAttr()
        support_collision.Set(False)

        floor_prim = _define_cube(
            stage, f'{scene_root}/floor',
            [carrier_position[0], carrier_position[1], 0.0],
            [0.65, 0.65, 0.02], (0.12, 0.14, 0.18),
            Gf, UsdGeom, UsdPhysics)
        UsdPhysics.CollisionAPI(
            floor_prim).GetCollisionEnabledAttr().Set(False)

        # A fixed, non-colliding backdrop and a diagonal row of markers make
        # both components of the world-YZ shake visible independently of the
        # floor texture or a user's viewport layout. The camera looks mainly
        # along world -X, so world Y and Z map to image horizontal and
        # vertical.
        reference_x = block_position[0] - 0.10
        reference_center_y = (
            block_position[1] + BACKWARD_LIFT_DISTANCE_M)
        reference_center_z = block_position[2] + LIFT_DISTANCE_M
        backdrop_prim = _define_cube(
            stage, f'{scene_root}/motion_backdrop',
            [reference_x, reference_center_y, reference_center_z],
            [0.008, 0.20, 0.20], (0.08, 0.09, 0.12),
            Gf, UsdGeom, UsdPhysics)
        UsdPhysics.CollisionAPI(
            backdrop_prim).GetCollisionEnabledAttr().Set(False)
        sign_texture_path = create_showcase_sign(
            model, payload_mass,
            _package_path('resource/branding/logo_onrobot_rgb.png'))
        sign_path = f'{scene_root}/brand_sign'
        _define_textured_sign(
            stage, sign_path, sign_texture_path,
            [reference_x + 0.005,
             reference_center_y,
             reference_center_z + 0.050],
            [0.155, 0.078], Gf, Sdf, UsdGeom, UsdShade, Vt)
        marker_positions = []
        marker_prims = []
        for marker_index, fraction in enumerate(
                (-1.0, -0.5, 0.0, 0.5, 1.0)):
            marker_position = [
                reference_x + 0.005,
                block_position[1] + BACKWARD_LIFT_DISTANCE_M +
                fraction * BACKWARD_SHAKE_AMPLITUDE_M,
                block_position[2] + LIFT_DISTANCE_M +
                fraction * VERTICAL_SHAKE_AMPLITUDE_M,
            ]
            marker_positions.append(marker_position)
            marker_prim = _define_cube(
                stage, f'{scene_root}/motion_marker_{marker_index}',
                marker_position, [0.010, 0.006, 0.006],
                ONROBOT_BLUE,
                Gf, UsdGeom, UsdPhysics)
            marker_prims.append(marker_prim)
            UsdPhysics.CollisionAPI(
                marker_prim).GetCollisionEnabledAttr().Set(False)

        scene_prim = stage.GetPrimAtPath('/PhysicsScene')
        if not scene_prim.IsValid():
            scene_prim = UsdPhysics.Scene.Define(
                stage, '/PhysicsScene').GetPrim()
        scene = UsdPhysics.Scene(scene_prim)
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        PhysxSchema.PhysxSceneAPI.Apply(
            scene_prim).CreateEnableCCDAttr().Set(True)
        scene_api = PhysxSchema.PhysxSceneAPI(scene_prim)
        scene_api.CreateSolverTypeAttr().Set('TGS')
        scene_api.CreateEnableExternalForcesEveryIterationAttr().Set(
            bool(config['physics']['external_forces_every_iteration']))
        report['physics_scene_settings'] = {
            'solver_type': scene_api.GetSolverTypeAttr().Get(),
            'external_forces_every_iteration': scene_api.GetEnableExternalForcesEveryIterationAttr().Get(),
        }

        showcase_viewport = None
        active_camera_path = None
        app.update()
        app.update()
        if render_output:
            from isaacsim.core.rendering_manager import ViewportManager
            camera_eye = [
                block_position[index] + CAMERA_EYE_OFFSET_M[index]
                for index in range(3)]
            camera_target = [
                block_position[index] + CAMERA_TARGET_OFFSET_M[index]
                for index in range(3)]
            camera = UsdGeom.Camera.Define(stage, camera_path)
            camera.CreateFocalLengthAttr().Set(CAMERA_FOCAL_LENGTH_MM)
            camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.01, 10.0))
            camera_xform = UsdGeom.Xformable(camera.GetPrim())
            camera_xform.ClearXformOpOrder()
            camera_xform.AddTransformOp().Set(
                Gf.Matrix4d().SetLookAt(
                    Gf.Vec3d(*camera_eye),
                    Gf.Vec3d(*camera_target),
                    Gf.Vec3d(0.0, 0.0, 1.0)).GetInverse())
            ViewportManager.set_camera(camera.GetPrim())
            showcase_viewport = ViewportManager.get_viewport_api()
            app.update()

        screenshot_viewport = None
        screenshot_dir = None
        captured_screenshots = []
        keyframe_states = []
        if args.screenshot_dir is not None:
            screenshot_dir = args.screenshot_dir.resolve()
            screenshot_dir.mkdir(parents=True, exist_ok=True)
            screenshot_viewport = showcase_viewport
            if screenshot_viewport is None:
                raise RuntimeError(
                    'no active viewport is available for capture')

        renderer_capture = None
        recording_dir = None
        recorded_frames = []
        recording_active = False
        recording_first_simulation_time = None
        recording_last_simulation_time = None
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

        physics = config['physics']
        requested_physics_dt = float(physics['physics_dt_s'])
        actual_physics_dt = setup_simulation(
            SimulationManager, dt=requested_physics_dt, device='cpu')
        dt = actual_physics_dt
        # These constructors can author rigid-body schemas. Complete them
        # before creating the articulation view and starting physics.
        if control == 'preload-position':
            pad_contacts = PadContactObserver(block_path, [
                f'{root_prim_path}/Geometry/{name}' for name in profile['fingertip_links']], dt)
            pad_contacts.subscribe(stage)
        block = RigidPrim(block_path)
        carrier = RigidPrim(f'{root_prim_path}/Geometry/base_link')
        support = RigidPrim(support_path)
        articulation = Articulation(mount_root_joint_path)
        play()
        for _ in range(8):
            SimulationManager.step()
        # The harness owns physics stepping. Keep the Kit timeline paused so
        # viewport refresh and screenshot completion cannot advance an extra
        # unmeasured physics frame.
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        manual_step_started = float(
            SimulationManager.get_simulation_time())
        if render_output:
            # Timeline start and viewport restoration can replace a camera
            # selected during stage construction. Select it again after play,
            # verify the live viewport path, and render enough frames for the
            # operator to see the intended view before motion begins.
            ViewportManager.set_camera(camera.GetPrim())
            showcase_viewport = ViewportManager.get_viewport_api()
            active_camera_path = str(showcase_viewport.camera_path)
            if active_camera_path != camera_path:
                raise RuntimeError(
                    'active viewport did not select the showcase camera: '
                    f'{active_camera_path}')
            for _ in range(CAMERA_SELECTION_SETTLE_FRAMES):
                app.update()
        if not articulation.dof_names:
            raise RuntimeError(
                'PhysX did not create the moving showcase articulation')
        lift_name = lift_joint.GetPrim().GetName()
        backward_name = backward_joint.GetPrim().GetName()
        finger_name = driven_joint(contract)
        if lift_name not in articulation.dof_names:
            raise RuntimeError(
                f'{lift_name} is absent from the moving articulation: '
                f'{articulation.dof_names}')
        if backward_name not in articulation.dof_names:
            raise RuntimeError(
                f'{backward_name} is absent from the moving articulation: '
                f'{articulation.dof_names}')
        if finger_name not in articulation.dof_names:
            raise RuntimeError(
                f'{finger_name} is absent from the moving articulation')
        lift_index = articulation.dof_names.index(lift_name)
        backward_index = articulation.dof_names.index(backward_name)
        finger_index = articulation.dof_names.index(finger_name)
        articulation_view_reacquires = 0

        def current_articulation_view():
            """Reacquire the PhysX view after a support/body stage edit."""
            nonlocal articulation, articulation_view_reacquires
            is_valid = getattr(
                articulation, 'is_physics_tensor_entity_valid', None)
            if callable(is_valid):
                try:
                    valid = bool(is_valid())
                except Exception:
                    valid = False
                if not valid:
                    articulation = Articulation(mount_root_joint_path)
                    articulation_view_reacquires += 1
                    if articulation_view_reacquires > 4:
                        raise RuntimeError('showcase articulation repeatedly invalidated')
                    is_valid = getattr(
                        articulation, 'is_physics_tensor_entity_valid', None)
                    if callable(is_valid) and not is_valid():
                        raise RuntimeError(
                            'PhysX showcase articulation tensor view remained '
                            f'invalid after reacquiring {mount_root_joint_path}')
            return articulation

        render_stride = (max(
            1, round(1.0 / (dt * GUI_RENDER_RATE_HZ))
        ) if render_output else None)
        recording_stride = max(
            1, round(1.0 / (dt * RECORDING_RATE_HZ))
        ) if renderer_capture is not None else 1
        effective_recording_rate_hz = (
            1.0 / (dt * recording_stride)
            if renderer_capture is not None else None)
        physics_step_count = 0
        initial_carrier_position = None

        def step(backward_target, lift_target, finger_target):
            nonlocal physics_step_count
            nonlocal recording_first_simulation_time
            nonlocal recording_last_simulation_time
            if pad_contacts is not None:
                pad_contacts.begin_step()
            active_articulation = current_articulation_view()
            active_articulation.set_dof_position_targets(
                [-float(backward_target)], dof_indices=[backward_index])
            active_articulation.set_dof_position_targets(
                [float(lift_target)], dof_indices=[lift_index])
            active_articulation.set_dof_position_targets(
                [float(finger_target)], dof_indices=[finger_index])
            SimulationManager.step()
            if pad_contacts is not None:
                pad_contacts.end_step()
            physics_step_count += 1
            capture_frame = (
                recording_active and renderer_capture is not None and
                physics_step_count % recording_stride == 0)
            if capture_frame:
                frame_path = recording_dir / (
                    f'frame_{len(recorded_frames):05d}.png')
                renderer_capture.capture_next_frame_swapchain(
                    str(frame_path))
            if ((render_stride is not None and
                 physics_step_count % render_stride == 0) or capture_frame):
                app.update()
            if capture_frame:
                renderer_capture.wait_async_capture()
                if not frame_path.is_file():
                    raise RuntimeError(
                        'renderer did not create recording frame '
                        f'{frame_path}')
                recorded_frames.append(str(frame_path))
                simulation_time = float(
                    SimulationManager.get_simulation_time())
                if recording_first_simulation_time is None:
                    recording_first_simulation_time = simulation_time
                recording_last_simulation_time = simulation_time

        def capture_keyframe(name):
            carrier_position_now = _row(carrier.get_world_poses()[0])
            block_position_now = _row(block.get_world_poses()[0])
            keyframe_states.append({
                'name': name,
                'simulation_time_s': float(
                    SimulationManager.get_simulation_time()),
                'carrier_world_position_m': carrier_position_now,
                'payload_world_position_m': block_position_now,
                'carrier_delta_from_grasp_m': (
                    [carrier_position_now[index] -
                     initial_carrier_position[index]
                     for index in range(3)]
                    if initial_carrier_position is not None else None),
            })
            if screenshot_viewport is None:
                return
            from omni.kit.viewport.utility import capture_viewport_to_file
            frame_path = screenshot_dir / f'{name}.png'
            capture = capture_viewport_to_file(
                screenshot_viewport, file_path=str(frame_path), is_hdr=False)
            task = asyncio.ensure_future(capture.wait_for_result())
            while not task.done():
                app.update()
            if task.exception() is not None:
                raise RuntimeError(
                    f'failed to capture {frame_path}: {task.exception()}')
            captured_screenshots.append(str(frame_path))

        open_target = float(profile['open_target'])
        grasp_target = float(profile['grasp_target'])
        initial_joint = _row(current_articulation_view().get_dof_positions())[finger_index]
        for target in _smooth_joint_targets(initial_joint, open_target, math.ceil(2.0 / dt)):
            step(0.0, 0.0, target)
        for _ in range(int(physics['settle_steps'])):
            step(0.0, 0.0, open_target)

        placement_carrier_position = _row(carrier.get_world_poses()[0])
        placement_carrier_orientation = _row(carrier.get_world_poses()[1])
        aligned_center = _rotate(
            placement_carrier_orientation, local_center)
        aligned_block_position = [
            placement_carrier_position[index] + aligned_center[index]
            for index in range(3)]
        placement_delta = [
            aligned_block_position[index] - block_position[index]
            for index in range(3)]
        block_position = aligned_block_position
        support_position = [
            support_position[index] + placement_delta[index]
            for index in range(3)]
        block.set_world_poses(
            positions=[block_position],
            orientations=[[1.0, 0.0, 0.0, 0.0]])
        support.set_world_poses(
            positions=[support_position],
            orientations=[[1.0, 0.0, 0.0, 0.0]])
        support_collision.Set(True)
        if render_output:
            backdrop_position = [
                reference_x + placement_delta[0],
                reference_center_y + placement_delta[1],
                reference_center_z + placement_delta[2],
            ]
            UsdGeom.Xformable(backdrop_prim).GetOrderedXformOps()[0].Set(
                Gf.Vec3d(*backdrop_position))
            for marker_prim, marker_position in zip(
                    marker_prims, marker_positions):
                for index in range(3):
                    marker_position[index] += placement_delta[index]
                UsdGeom.Xformable(
                    marker_prim).GetOrderedXformOps()[0].Set(
                        Gf.Vec3d(*marker_position))
            camera_eye = [
                block_position[index] + CAMERA_EYE_OFFSET_M[index]
                for index in range(3)]
            camera_target = [
                block_position[index] + CAMERA_TARGET_OFFSET_M[index]
                for index in range(3)]
            camera_xform.GetOrderedXformOps()[0].Set(
                Gf.Matrix4d().SetLookAt(
                    Gf.Vec3d(*camera_eye),
                    Gf.Vec3d(*camera_target),
                    Gf.Vec3d(0.0, 0.0, 1.0)).GetInverse())
        block_kinematic.Set(False)
        block_ccd.Set(True)
        block_collision.Set(True)
        # Start the optional recording only after scene/camera settling and
        # payload placement. Every captured frame below belongs to the same
        # live, manually stepped PhysX run used for the verdict.
        recording_active = True
        # Let PhysX consume the kinematic-to-dynamic transition before using
        # the dynamic-body velocity API. Calling set_velocities in the same
        # stage update produces a PhysX error even though the following frame
        # already treats the body as dynamic.
        app.update()
        SimulationManager.step()
        physics_step_count += 1
        if (render_stride is not None and
                physics_step_count % render_stride == 0):
            app.update()
        block.set_velocities(
            linear_velocities=[[0.0, 0.0, 0.0]],
            angular_velocities=[[0.0, 0.0, 0.0]])
        for _ in range(4):
            step(0.0, 0.0, open_target)
        for target in _smooth_joint_targets(open_target, grasp_target, int(physics['close_steps'])):
            step(0.0, 0.0, target)
        if pad_contacts is not None:
            preload = contact_preload_reference(current_articulation_view(), finger_index, pad_contacts)
            report['impedance_preload'] = preload
            preload_target = preload['drive_reference_m']
            for target in _smooth_joint_targets(grasp_target, preload_target, math.ceil(.25 / dt)):
                step(0.0, 0.0, target)
            grasp_target = preload_target
        for _ in range(int(physics['preload_steps'])):
            step(0.0, 0.0, grasp_target)

        capture_keyframe('00_grasped')

        active_articulation = current_articulation_view()
        acquired_dofs = _row(active_articulation.get_dof_positions())
        acquired_finger = acquired_dofs[finger_index]
        acquired_lift = acquired_dofs[lift_index]
        acquired_backward = acquired_dofs[backward_index]
        blocking_error = abs(acquired_finger - grasp_target)
        initial_carrier_position = _row(carrier.get_world_poses()[0])
        initial_carrier_orientation = _row(carrier.get_world_poses()[1])
        initial_block_position = _row(block.get_world_poses()[0])
        initial_block_orientation = _row(block.get_world_poses()[1])
        initial_relative = _relative_pose(
            initial_carrier_position, initial_carrier_orientation,
            initial_block_position, initial_block_orientation)

        support_collision.Set(False)
        UsdGeom.Imageable(support_prim).MakeInvisible()
        maximum_relative_translation = 0.0
        maximum_relative_rotation = 0.0
        maximum_payload_speed = 0.0
        maximum_finger_drift = 0.0
        maximum_linkage_drift = 0.0
        maximum_root_tracking_error = 0.0
        maximum_lift = 0.0
        maximum_carrier_displacement = 0.0
        maximum_carrier_vertical_lift = 0.0
        maximum_carrier_backward_travel = 0.0
        maximum_lift_backward_travel = 0.0
        maximum_carrier_axis_travel = 0.0
        maximum_carrier_off_axis_error = 0.0
        minimum_shake_backward_travel = math.inf
        maximum_shake_backward_travel = -math.inf
        minimum_shake_vertical_travel = math.inf
        maximum_shake_vertical_travel = -math.inf
        trajectory_samples = []
        trajectory_truncated = False
        state_finite = True

        def observe(commanded_backward, commanded_lift, measure_lift_axis):
            nonlocal maximum_relative_translation, maximum_relative_rotation
            nonlocal maximum_payload_speed, maximum_finger_drift
            nonlocal maximum_linkage_drift, maximum_root_tracking_error
            nonlocal maximum_lift, maximum_carrier_displacement
            nonlocal maximum_carrier_vertical_lift
            nonlocal maximum_carrier_backward_travel
            nonlocal maximum_lift_backward_travel
            nonlocal maximum_carrier_axis_travel
            nonlocal maximum_carrier_off_axis_error, state_finite
            carrier_position_now = _row(carrier.get_world_poses()[0])
            carrier_orientation_now = _row(carrier.get_world_poses()[1])
            object_position = _row(block.get_world_poses()[0])
            object_orientation = _row(block.get_world_poses()[1])
            relative = _relative_pose(
                carrier_position_now, carrier_orientation_now,
                object_position, object_orientation)
            maximum_relative_translation = max(
                maximum_relative_translation,
                _norm([relative[0][i] - initial_relative[0][i]
                       for i in range(3)]))
            maximum_relative_rotation = max(
                maximum_relative_rotation,
                _rotation_error(relative[1], initial_relative[1]))
            maximum_payload_speed = max(
                maximum_payload_speed,
                _norm(_linear_velocity(block.get_velocities())))
            active_articulation = current_articulation_view()
            dofs = _row(active_articulation.get_dof_positions())
            maximum_finger_drift = max(
                maximum_finger_drift,
                abs(dofs[finger_index] - acquired_finger))
            maximum_linkage_drift = max(
                maximum_linkage_drift,
                max((abs(dofs[i] - acquired_dofs[i])
                     for i in range(len(dofs))
                     if i not in (finger_index, lift_index, backward_index)),
                    default=0.0))
            actual_lift = dofs[lift_index] - acquired_lift
            actual_backward = -(
                dofs[backward_index] - acquired_backward)
            maximum_lift = max(maximum_lift, actual_lift)
            maximum_root_tracking_error = max(
                maximum_root_tracking_error,
                abs(actual_lift - commanded_lift),
                abs(actual_backward - commanded_backward))
            carrier_delta = [
                carrier_position_now[i] - initial_carrier_position[i]
                for i in range(3)]
            maximum_carrier_displacement = max(
                maximum_carrier_displacement, _norm(carrier_delta))
            maximum_carrier_vertical_lift = max(
                maximum_carrier_vertical_lift, carrier_delta[2])
            maximum_carrier_backward_travel = max(
                maximum_carrier_backward_travel, carrier_delta[1])
            if measure_lift_axis:
                maximum_lift_backward_travel = max(
                    maximum_lift_backward_travel, carrier_delta[1])
                axis_travel = sum(
                    carrier_delta[i] * lift_motion_axis_world[i]
                    for i in range(3))
                maximum_carrier_axis_travel = max(
                    maximum_carrier_axis_travel, axis_travel)
                off_axis = [
                    carrier_delta[i] -
                    axis_travel * lift_motion_axis_world[i]
                    for i in range(3)]
                maximum_carrier_off_axis_error = max(
                    maximum_carrier_off_axis_error, _norm(off_axis))
            state_finite = state_finite and all(math.isfinite(value) for value
                                                in (carrier_position_now +
                                                    carrier_orientation_now +
                                                    object_position +
                                                    object_orientation +
                                                    dofs))
            if len(trajectory_samples) < MAX_TRAJECTORY_SAMPLES:
                trajectory_samples.append({
                    'simulation_time_s': float(
                        SimulationManager.get_simulation_time()),
                    'commanded_backward_m': float(commanded_backward),
                    'commanded_lift_m': float(commanded_lift),
                    'commanded_finger_position': float(grasp_target),
                    'carrier_world_position_m': carrier_position_now,
                    'payload_world_position_m': object_position,
                    'payload_relative_translation_m': [
                        float(relative[0][i]) for i in range(3)],
                    'payload_speed_m_s': _norm(
                        _linear_velocity(block.get_velocities())),
                    'finger_position': float(dofs[finger_index]),
                })
            else:
                trajectory_truncated = True
            return carrier_delta

        # Verify gravity retention before the mount starts moving.
        for _ in range(int(physics['lift_steps']) // 3):
            step(0.0, 0.0, grasp_target)
            observe(0.0, 0.0, False)

        lift_steps = max(1, round(LIFT_DURATION_S / dt))
        for index in range(lift_steps):
            blend = (index + 1) / float(lift_steps)
            smooth = blend * blend * (3.0 - 2.0 * blend)
            backward_target = BACKWARD_LIFT_DISTANCE_M * smooth
            lift_target = LIFT_DISTANCE_M * smooth
            step(backward_target, lift_target, grasp_target)
            observe(backward_target, lift_target, True)
        capture_keyframe('01_lifted')

        shake_duration = SHAKE_CYCLES / SHAKE_FREQUENCY_HZ
        shake_steps = max(1, round(shake_duration / dt))
        first_peak_index = round(
            (shake_steps - 1) / (4.0 * SHAKE_CYCLES))
        first_trough_index = round(
            3.0 * (shake_steps - 1) / (4.0 * SHAKE_CYCLES))
        for index in range(shake_steps):
            phase = 2.0 * math.pi * SHAKE_CYCLES * index / max(
                1, shake_steps - 1)
            phase_position = math.sin(phase)
            backward_target = (
                BACKWARD_LIFT_DISTANCE_M +
                BACKWARD_SHAKE_AMPLITUDE_M * phase_position)
            lift_target = (
                LIFT_DISTANCE_M +
                VERTICAL_SHAKE_AMPLITUDE_M * phase_position)
            step(backward_target, lift_target, grasp_target)
            carrier_delta = observe(
                backward_target, lift_target, False)
            backward_travel = carrier_delta[1]
            vertical_travel = carrier_delta[2]
            minimum_shake_backward_travel = min(
                minimum_shake_backward_travel, backward_travel)
            maximum_shake_backward_travel = max(
                maximum_shake_backward_travel, backward_travel)
            minimum_shake_vertical_travel = min(
                minimum_shake_vertical_travel, vertical_travel)
            maximum_shake_vertical_travel = max(
                maximum_shake_vertical_travel, vertical_travel)
            if index == first_peak_index:
                capture_keyframe('02_shake_upper_backward')
            elif index == first_trough_index:
                capture_keyframe('03_shake_lower_forward')

        shake_path_amplitude = math.hypot(
            BACKWARD_SHAKE_AMPLITUDE_M, VERTICAL_SHAKE_AMPLITUDE_M)
        shake_motion_axis_world = [
            0.0,
            BACKWARD_SHAKE_AMPLITUDE_M / shake_path_amplitude,
            VERTICAL_SHAKE_AMPLITUDE_M / shake_path_amplitude,
        ]
        measured_backward_span = (
            maximum_shake_backward_travel -
            minimum_shake_backward_travel)
        measured_vertical_span = (
            maximum_shake_vertical_travel -
            minimum_shake_vertical_travel)
        measured_diagonal_span = math.hypot(
            measured_backward_span, measured_vertical_span)
        measured_manual_step_duration = float(
            SimulationManager.get_simulation_time()) - manual_step_started
        expected_manual_step_duration = physics_step_count * actual_physics_dt
        manual_step_timing_error = abs(
            measured_manual_step_duration - expected_manual_step_duration)

        checks = {
            'joint_blocked_before_commanded_endpoint': (
                blocking_error >= float(profile['minimum_blocking_error'])),
            'mount_physically_lifted': (
                maximum_lift >= 0.055 and
                maximum_carrier_vertical_lift >= 0.055),
            'mount_moved_diagonally_backward': (
                maximum_lift_backward_travel >= 0.030 and
                maximum_carrier_off_axis_error <= 0.002),
            'mount_shook_diagonally': (
                measured_backward_span >= 0.035 and
                measured_vertical_span >= 0.014 and
                measured_diagonal_span >= 0.040),
            'mount_followed_actuated_trajectory': (
                maximum_root_tracking_error <= 0.025),
            'payload_retained_during_physical_motion': (
                maximum_relative_translation <= float(
                    fixture['maximum_relative_translation_error_m'])),
            'payload_rotation_bounded': (
                maximum_relative_rotation <= float(
                    fixture['maximum_relative_rotation_error_rad'])),
            'payload_speed_bounded': (
                maximum_payload_speed <= float(
                    fixture['maximum_object_speed_m_s'])),
            'gripper_joint_stable': (
                maximum_finger_drift <= float(
                    fixture['maximum_joint_drift'])),
            'gripper_linkage_stable': (
                maximum_linkage_drift <= float(
                    profile['maximum_linkage_drift'])),
            'articulation_and_payload_state_finite': state_finite,
            'rendering_did_not_advance_physics': (
                manual_step_timing_error <= max(
                    actual_physics_dt * 0.01, 1e-6)),
        }
        report['asset_files_sha256'] = _asset_manifest(asset)
        report['physics_backend'] = 'physx'
        report['payload_mass_kg'] = payload_mass
        report['showcase_branding'] = {
            'logo': str(_package_path(
                'resource/branding/logo_onrobot_rgb.png')),
            'sign_path': sign_path,
            'payload_label': f'{payload_mass:g} kg',
            'motion_marker_color_rgb': list(ONROBOT_BLUE),
        }
        report['physics_overrides'] = physics_overrides
        report['runtime_articulation_view_reacquires'] = (
            articulation_view_reacquires)
        report['rated_force_fit_payload_kg'] = rated_payload
        report['payload_fraction_of_rating'] = payload_mass / rated_payload
        report['placement_alignment'] = {
            'expected_carrier_position_m': carrier_position,
            'measured_carrier_position_m': placement_carrier_position,
            'measured_carrier_orientation_wxyz': (
                placement_carrier_orientation),
            'correction_m': placement_delta,
            'block_position_m': block_position,
        }
        camera_forward_xy = [
            CAMERA_TARGET_OFFSET_M[index] - CAMERA_EYE_OFFSET_M[index]
            for index in range(2)]
        camera_forward_xy_norm = math.hypot(*camera_forward_xy)
        camera_world_y_horizontal_visibility = abs(
            camera_forward_xy[0] / camera_forward_xy_norm)
        expected_camera_horizontal_shake_span = (
            2.0 * BACKWARD_SHAKE_AMPLITUDE_M *
            camera_world_y_horizontal_visibility)
        expected_camera_vertical_shake_span = (
            2.0 * VERTICAL_SHAKE_AMPLITUDE_M)
        checks['camera_exposes_diagonal_shake'] = (
            expected_camera_horizontal_shake_span >= 0.040 and
            expected_camera_vertical_shake_span >= 0.015)
        report['motion'] = {
            'source': 'physx-two-axis-prismatic-mount-drive',
            'replay': False,
            'vertical_lift_distance_m': LIFT_DISTANCE_M,
            'backward_lift_distance_m': BACKWARD_LIFT_DISTANCE_M,
            'lift_duration_s': LIFT_DURATION_S,
            'lift_motion_axis_world_xyz': lift_motion_axis_world,
            'camera': {
                'path': camera_path,
                'verified_active_path': active_camera_path,
                'eye_offset_from_initial_payload_m': CAMERA_EYE_OFFSET_M,
                'target_offset_from_initial_payload_m': (
                    CAMERA_TARGET_OFFSET_M),
                'fixed_world_camera': True,
                'focal_length_mm': CAMERA_FOCAL_LENGTH_MM,
                'level_view': True,
                'world_y_horizontal_visibility': (
                    camera_world_y_horizontal_visibility),
                'expected_horizontal_shake_span_m': (
                    expected_camera_horizontal_shake_span),
                'expected_vertical_shake_span_m': (
                    expected_camera_vertical_shake_span),
            },
            'motion_reference_marker_world_positions_m': marker_positions,
            'screenshots': [
                _portable_media_path(Path(path), args.output)
                for path in captured_screenshots],
            'measured_keyframes': keyframe_states,
            'recording': {
                'enabled': recording_dir is not None,
                'source': 'same-live-physx-evidence-run',
                'replay': False,
                'requested_frame_rate_hz': RECORDING_RATE_HZ,
                'effective_frame_rate_hz': effective_recording_rate_hz,
                'frame_count': len(recorded_frames),
                'first_simulation_time_s': (
                    recording_first_simulation_time),
                'last_simulation_time_s': recording_last_simulation_time,
                'directory': (_portable_media_path(recording_dir, args.output)
                              if recording_dir is not None else None),
                'ffmpeg_command': (
                    _recording_ffmpeg_command(recording_dir, args.output)
                    if recording_dir is not None else None),
            },
            'backward_shake_amplitude_m': (
                BACKWARD_SHAKE_AMPLITUDE_M),
            'vertical_shake_amplitude_m': VERTICAL_SHAKE_AMPLITUDE_M,
            'shake_motion_axis_world_xyz': shake_motion_axis_world,
            'shake_frequency_hz': SHAKE_FREQUENCY_HZ,
            'shake_cycles': SHAKE_CYCLES,
            'shake_duration_s': shake_duration,
            'requested_physics_dt_s': requested_physics_dt,
            'actual_physics_dt_s': actual_physics_dt,
            'manual_physics_steps': physics_step_count,
            'expected_manual_step_duration_s': (
                expected_manual_step_duration),
            'measured_manual_step_duration_s': (
                measured_manual_step_duration),
            'manual_step_timing_error_s': manual_step_timing_error,
            'peak_commanded_shake_acceleration_m_s2': (
                shake_path_amplitude *
                (2.0 * math.pi * SHAKE_FREQUENCY_HZ) ** 2),
            'gui_render_stride': render_stride,
            'recording_stride': recording_stride,
            'mount_drives': {
                'stiffness': MOUNT_DRIVE_STIFFNESS,
                'damping': MOUNT_DRIVE_DAMPING,
                'maximum_force_n': MOUNT_DRIVE_MAXIMUM_FORCE,
            },
        }
        report['trajectory'] = {
            'sample_count': len(trajectory_samples),
            'sample_cap': MAX_TRAJECTORY_SAMPLES,
            'truncated': trajectory_truncated,
            'sample_period_s': actual_physics_dt,
            'samples': trajectory_samples,
        }
        report['tests'] = [{
            'name': 'full_rated_payload_physical_lift_and_shake',
            'status': 'passed' if all(checks.values()) else 'failed',
            'checks': checks,
            'measurements': {
                'acquired_finger_position': acquired_finger,
                'blocking_error': blocking_error,
                'maximum_mount_lift_m': maximum_lift,
                'maximum_mount_tracking_error_m': (
                    maximum_root_tracking_error),
                'maximum_carrier_world_displacement_m': (
                    maximum_carrier_displacement),
                'maximum_carrier_vertical_lift_m': (
                    maximum_carrier_vertical_lift),
                'maximum_carrier_backward_travel_m': (
                    maximum_carrier_backward_travel),
                'maximum_lift_backward_travel_m': (
                    maximum_lift_backward_travel),
                'maximum_carrier_axis_travel_m': (
                    maximum_carrier_axis_travel),
                'maximum_carrier_off_axis_error_m': (
                    maximum_carrier_off_axis_error),
                'minimum_shake_backward_travel_m': (
                    minimum_shake_backward_travel),
                'maximum_shake_backward_travel_m': (
                    maximum_shake_backward_travel),
                'minimum_shake_vertical_travel_m': (
                    minimum_shake_vertical_travel),
                'maximum_shake_vertical_travel_m': (
                    maximum_shake_vertical_travel),
                'horizontal_shake_span_m': (
                    measured_backward_span),
                'vertical_shake_span_m': measured_vertical_span,
                'diagonal_shake_span_m': measured_diagonal_span,
                'maximum_relative_translation_error_m': (
                    maximum_relative_translation),
                'maximum_relative_rotation_error_rad': (
                    maximum_relative_rotation),
                'maximum_payload_speed_m_s': maximum_payload_speed,
                'maximum_finger_joint_drift': maximum_finger_drift,
                'maximum_linkage_drift': maximum_linkage_drift,
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
            close_app(app, exit_code)
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    sys.exit(main())

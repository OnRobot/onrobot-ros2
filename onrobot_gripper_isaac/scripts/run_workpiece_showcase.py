#!/usr/bin/env python3
"""Pick up a rigid workpiece, swing it like a pendulum, and put it down.

No ROS is required. The gripper is driven through a vertical carriage and a
rotary wrist. The cube remains dynamic, with gravity and table collision, for
the entire test. Contact forces are simulation observations, not a calibrated
hardware force measurement.
"""

import argparse
import asyncio
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback

from isaac_model_contract import default_asset, load_contract, package_root, SUPPORTED_MODELS
from grip_drive_control import linear_preload_target, script_manifest


def contact_force_components(impulse, normal, dt):
    """Normal and tangential impulse magnitudes in newtons, without sign claims."""
    if (len(impulse) != 3 or len(normal) != 3 or not math.isfinite(dt) or dt <= 0
            or not all(math.isfinite(v) for v in (*impulse, *normal))):
        raise ValueError('invalid contact impulse, normal or timestep')
    length = math.sqrt(sum(v * v for v in normal))
    if length < 1e-10:
        raise ValueError('contact normal has zero length')
    n = [v / length for v in normal]
    projected = sum(v * axis for v, axis in zip(impulse, n))
    tangent = math.sqrt(sum((v - projected * axis) ** 2 for v, axis in zip(impulse, n)))
    return float(abs(projected) / dt), float(tangent / dt)


class ContactForceAverage:
    """Integrate every contact step between saved samples, including no-contact steps.

    This recorder does not filter physics or replace instantaneous safety checks.
    Decimating instantaneous impulses can alias the constraint-solver oscillations.
    """

    def __init__(self, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('contact averaging requires a positive finite timestep')
        self.dt = dt
        self.steps = 0
        self.sums = {}

    def add(self, force_vectors):
        for body, vector in force_vectors.items():
            if len(vector) != 3 or not all(math.isfinite(v) for v in vector):
                raise ValueError('invalid contact vector for averaging')
            total = self.sums.setdefault(body, [0., 0., 0.])
            for i, value in enumerate(vector):
                total[i] += float(value)
        self.steps += 1

    def take(self):
        if not self.steps:
            raise ValueError('no physics steps in contact average')
        result = {'step_count': self.steps, 'duration_s': self.steps * self.dt,
                  'force_vectors_n': {body: [v / self.steps for v in total]
                                      for body, total in self.sums.items()}}
        self.steps = 0
        self.sums.clear()
        return result


def sampled_cube_forces(sample):
    """Return interval-average forces and duration, or a legacy snapshot weight."""
    if 'cube_contact_force_average' not in sample:
        return sample.get('cube_contact_force_vectors_n'), 1.
    window = sample['cube_contact_force_average']
    if not isinstance(window, dict):
        raise ValueError('invalid contact averaging interval')
    steps, duration = window.get('step_count'), window.get('duration_s')
    if (type(steps) is not int or steps <= 0 or type(duration) not in (int, float)
            or not math.isfinite(duration) or duration <= 0):
        raise ValueError('invalid contact averaging interval')
    return window.get('force_vectors_n'), duration


def supported_weight_check(samples, mass_kg):
    """Validate the observer's force sign/scale against a quiet supported cube."""
    if len(samples) < 8 or not math.isfinite(mass_kg) or mass_kg <= 0:
        raise ValueError('supported-weight check needs mass and at least eight observations')
    net, durations = [], []
    for sample in samples:
        forces, duration = sampled_cube_forces(sample)
        if not isinstance(forces, dict) or not forces:
            raise ValueError('supported cube has no reported contact force')
        if any(len(v) != 3 or not all(math.isfinite(x) for x in v) for v in forces.values()):
            raise ValueError('invalid supported contact force')
        net.append([sum(v[i] for v in forces.values()) for i in range(3)])
        durations.append(duration)
    mean = [sum(v[i] * dt for v, dt in zip(net, durations)) / sum(durations) for i in range(3)]
    weight = mass_kg * 9.81
    error = math.sqrt(mean[0] ** 2 + mean[1] ** 2 + (mean[2] - weight) ** 2)
    return {'passed': error < max(.1, .02 * weight), 'sample_count': len(samples),
            'expected_weight_n': weight, 'mean_contact_force_n': mean, 'vector_error_n': error}


def grasp_is_settled(window, angular):
    """Require sustained bilateral contact and quiet measured poses for 200 ms.

    Use pose changes rather than solver-reported joint velocity: implicit
    constraint velocity can differ from the derivative of the final poses.
    This is a simulation readiness check, not a calibrated grip-force signal.
    """
    if len(window) < 2 or window[-1]['time_s'] - window[0]['time_s'] < .2 - 1e-9:
        return False
    values = [[r['time_s'], r['jaw'], *r['cube'], *r['pad_normals']] for r in window]
    if not all(math.isfinite(v) for row in values for v in row):
        return False
    if any(b['time_s'] <= a['time_s'] for a, b in zip(window, window[1:])):
        return False
    jaw_range = max(r['jaw'] for r in window) - min(r['jaw'] for r in window)
    cube_span = math.sqrt(sum((max(r['cube'][i] for r in window) -
                               min(r['cube'][i] for r in window)) ** 2 for i in range(3)))
    return (jaw_range < (.0005 if angular else .000025) and cube_span < .0002
            and all(sum(r['pad_normals'][i] > .1 for r in window) / len(window) >= .75
                    for i in (0, 1)))


def cycle_passes(result, payload_kg, swing_deg, model='2fg7'):
    """Evaluate measured outcomes; absent/non-finite observations cannot pass."""
    closure_load = result.get('peak_table_load_during_closure_n', math.nan)
    if result.get('grasp_settled') is not True or not math.isfinite(closure_load) or closure_load < 0:
        return False
    bounds = {
        'maximum_relative_slip_m': .003,
        'maximum_relative_rotation_rad': .15,
        'placement_xy_error_m': .005,
        'placement_height_error_m': .003,
        'peak_table_load_during_approach_n': payload_kg * 9.81 + 20,
        'peak_nonpad_contact_during_swing_n': .5,
        'maximum_mimic_error': .02 if model.startswith('rg') else .0002,
    }
    if any(not math.isfinite(result.get(key, math.nan)) or
           not 0 <= result[key] < limit for key, limit in bounds.items()):
        return False
    lift, swing = result.get('maximum_lift_m', math.nan), result.get('maximum_swing_rad', math.nan)
    return (math.isfinite(lift) and math.isfinite(swing) and
            lift > .10 and swing > math.radians(swing_deg) * .85)


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=SUPPORTED_MODELS, required=True)
    parser.add_argument('--asset', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--screenshots', action='store_true')
    parser.add_argument('--cube-size-m', type=float, default=0.062)
    parser.add_argument('--payload-kg', type=float, default=0.712)
    parser.add_argument('--cycles', type=int, default=3)
    parser.add_argument('--physics-hz', type=int, default=480)
    parser.add_argument('--solver-type', choices=['TGS', 'PGS'], default='TGS',
                        help='explicit numerical comparison; default matches the asset tests')
    parser.add_argument('--external-forces-every-iteration', action=argparse.BooleanOptionalAction,
                        default=True,
                        help='apply external forces during TGS position iterations; disable only for comparison')
    parser.add_argument('--swing-deg', type=float, default=20.0)
    parser.add_argument('--swing-hz', type=float, default=1.5)
    parser.add_argument('--swing-cycles', type=int, default=5)
    parser.add_argument('--swing-axis', choices=['X', 'Y'], default='Y')
    parser.add_argument('--static-friction', type=float, default=0.6)
    parser.add_argument('--dynamic-friction', type=float, default=0.5)
    parser.add_argument('--override-pad-material', action='store_true',
                        help='diagnostic only: apply the object friction pair to the pads as well')
    parser.add_argument('--effort-fraction', type=float, default=1.0,
                        help='fraction of the authored drive ceiling; low values exercise failure')
    parser.add_argument('--hold-control', choices=['position', 'effort', 'preload-position'], default='position',
                        help='diagnostic hold: original position, direct effort, or 2FG contact-relative impedance preload')
    parser.add_argument('--close-seconds', type=float, default=1.5)
    parser.add_argument('--preload-seconds', type=float, default=.5,
                        help='Minimum contact settling duration before lifting')
    parser.add_argument('--force-ramp-seconds', type=float, default=.25,
                        help='Smooth loading time for contact-relative position preload')
    parser.add_argument('--grasp-settle-timeout', type=float, default=3.,
                        help='Maximum wait for stable jaw/cube poses and bilateral fingertip contact')
    parser.add_argument('--contact-datum-m', type=float,
                        help='mount-to-cube-center distance for a custom grasp placement')
    parser.add_argument('--contact-stiffness-n-m', type=float, default=0.0)
    parser.add_argument('--contact-damping-ns-m', type=float, default=0.0)
    parser.add_argument('--drive-stiffness-si', type=float,
                        help='diagnostic drive gain in N/m or N m/rad')
    parser.add_argument('--drive-damping-si', type=float,
                        help='diagnostic drive damping in N s/m or N m s/rad')
    parser.add_argument('--solver-velocity-iterations', type=int, choices=[0, 1, 2, 4])
    parser.add_argument('--mimic-frequency-hz', type=float,
                        help='diagnostic compliant mimic frequency; asset default if omitted')
    parser.add_argument('--mimic-damping-ratio', type=float,
                        help='diagnostic mimic damping ratio; asset default if omitted')
    parser.add_argument('--pad-approximation', choices=['convexHull', 'convexDecomposition'],
                        help='diagnostic rubber collision simplification; visual mesh unchanged')
    parser.add_argument('--drive-armature-kg-m2', type=float,
                        help='RG diagnostic reflected motor inertia on the actuated joint only')
    parser.add_argument('--drive-armature-kg', type=float,
                        help='2FG diagnostic reflected inertia in the one-jaw translation coordinate')
    parser.add_argument('--maximum-joint-velocity-rad-s', type=float,
                        help='Diagnostic reduced RG primary-joint speed ceiling')
    args = parser.parse_args(argv)
    if args.hold_control == 'preload-position' and not args.model.startswith('2fg'):
        parser.error('contact-relative preload currently supports symmetric 2FG translation only')
    if args.output.exists():
        parser.error('output already exists; choose a new report path')
    for field in ('cube_size_m', 'payload_kg', 'swing_hz', 'close_seconds', 'preload_seconds', 'force_ramp_seconds'):
        if not math.isfinite(getattr(args, field)) or getattr(args, field) <= 0:
            parser.error(f'{field} must be positive and finite')
    if not args.preload_seconds <= args.grasp_settle_timeout <= 10:
        parser.error('grasp settling timeout must be between preload duration and 10 seconds')
    if not 0 <= args.dynamic_friction <= args.static_friction <= 1.0:
        parser.error('require 0 <= dynamic friction <= static friction <= 1')
    if not 0 < args.effort_fraction <= 1 or not 0 < args.swing_deg <= 35:
        parser.error('effort fraction must be (0, 1], swing angle (0, 35] degrees')
    if args.cycles < 1 or args.swing_cycles < 3 or args.physics_hz not in (240, 480, 960):
        parser.error('positive trial count, at least three swing cycles, and 240, 480 or 960 Hz are required')
    if any(not math.isfinite(v) or v < 0 for v in (args.contact_stiffness_n_m, args.contact_damping_ns_m)):
        parser.error('contact stiffness/damping must be finite and nonnegative')
    if args.contact_datum_m is not None and not .1 <= args.contact_datum_m <= .4:
        parser.error('contact datum must be in [0.1, 0.4] metres')
    if any(v is not None and (not math.isfinite(v) or v < 0)
           for v in (args.drive_stiffness_si, args.drive_damping_si,
                     args.mimic_frequency_hz, args.mimic_damping_ratio,
                     args.drive_armature_kg_m2, args.drive_armature_kg)):
        parser.error('drive gains must be finite and nonnegative')
    if args.hold_control == 'preload-position' and args.drive_stiffness_si == 0:
        parser.error('contact-relative preload requires nonzero drive stiffness')
    if args.drive_armature_kg_m2 is not None and not args.model.startswith('rg'):
        parser.error('rotary motor armature comparison is only available for RG')
    if args.drive_armature_kg is not None and not args.model.startswith('2fg'):
        parser.error('linear motor armature comparison is only available for 2FG')
    if args.maximum_joint_velocity_rad_s is not None and (
            not args.model.startswith('rg') or not math.isfinite(args.maximum_joint_velocity_rad_s)
            or args.maximum_joint_velocity_rad_s <= 0):
        parser.error('RG velocity ceiling must be positive and finite')
    return args


async def run(args):
    import numpy as np
    import omni.kit.app
    import omni.physx
    import omni.timeline
    import omni.usd
    from isaacsim.core.experimental.prims import Articulation, RigidPrim
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.version import get_version
    from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    from isaac_runtime_compat import setup_simulation
    from run_payload_retention_test import _asset_manifest, _collision_material_scope, _row, _relative_pose, _rotation_error
    from showcase_branding import create_payload_label, create_studio_lighting, create_showcase_sign, define_textured_sign

    app = omni.kit.app.get_app()
    context = omni.usd.get_context()
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    await app.next_update_async()
    asset = (args.asset or default_asset(args.model)).resolve()
    contract = load_contract(args.model)
    report = {'schema_version': 1, 'test': 'table-pick-pendulum', 'status': 'running',
              'model': args.model, 'isaac_sim_version': get_version()[0],
              'asset': str(asset), 'asset_files_sha256': _asset_manifest(asset),
              'runner_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'runtime_script_files_sha256': script_manifest(__file__),
              'workpiece_bulk_density_kg_m3': args.payload_kg / args.cube_size_m ** 3,
              'settings': {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
              'cycles': [], 'frames': [], 'samples': []}
    args.output.parent.mkdir(parents=True, exist_ok=True)

    def save():
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')

    save()
    contact_subscription = None
    try:
        if not asset.is_file():
            raise RuntimeError(f'asset not found: {asset}')
        await context.new_stage_async()
        stage = context.get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        root = f'/onrobot_{args.model}'
        robot = UsdGeom.Xform.Define(stage, root)
        robot.GetPrim().GetReferences().AddReference(str(asset))
        scene_root = '/WorkpieceShowcase'
        UsdGeom.Xform.Define(stage, scene_root)
        # Local Z runs from mounting flange towards the fingertips.
        contact_z = {'2fg7': .160, '2fg14': .168, 'rg2': .233, 'rg6': .263}[args.model]
        if args.contact_datum_m is not None:
            contact_z = args.contact_datum_m
        report['contact_datum_m'] = contact_z
        table_top, lift_distance = .35, .12
        base_height = table_top + args.cube_size_m / 2 + contact_z + lift_distance
        origin = Gf.Vec3d(0, 0, base_height)
        down = Gf.Quatf(0, 1, 0, 0)
        robot.ClearXformOpOrder()
        robot.AddTranslateOp().Set(origin)
        robot.AddOrientOp().Set(down)
        original_root = stage.GetPrimAtPath(contract['usd']['articulation_root'])
        original_root.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        fixed = stage.GetPrimAtPath(f'{root}/Physics/root_joint')
        if not fixed.IsA(UsdPhysics.FixedJoint):
            raise RuntimeError('asset has no supported fixed mounting joint')
        fixed.SetActive(False)
        scene = UsdPhysics.Scene.Define(stage, f'{scene_root}/physics')
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        scene_api.CreateTimeStepsPerSecondAttr().Set(args.physics_hz)
        scene_api.CreateSolverTypeAttr().Set(args.solver_type)
        scene_api.CreateEnableExternalForcesEveryIterationAttr().Set(args.external_forces_every_iteration)
        scene_api.CreateEnableGPUDynamicsAttr().Set(False)
        scene_api.CreateBroadphaseTypeAttr().Set('MBP')

        def body(path):
            prim = UsdGeom.Xform.Define(stage, path)
            prim.AddTranslateOp().Set(origin)
            UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim())
            mass = UsdPhysics.MassAPI.Apply(prim.GetPrim())
            mass.CreateMassAttr().Set(1.0)
            mass.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(.01))
            return prim.GetPrim()

        anchor_path, lift_path = f'{scene_root}/anchor', f'{scene_root}/carriage'
        body(anchor_path)
        body(lift_path)
        base_path = f'{root}/Geometry/base_link'
        mount_root = UsdPhysics.FixedJoint.Define(stage, f'{scene_root}/joints/root')
        mount_root.CreateBody1Rel().SetTargets([anchor_path])
        mount_root.CreateLocalPos0Attr().Set(Gf.Vec3f(*origin))
        UsdPhysics.ArticulationRootAPI.Apply(mount_root.GetPrim())
        art_api = PhysxSchema.PhysxArticulationAPI.Apply(mount_root.GetPrim())
        # Carry the asset's solver policy onto the new articulation owner.
        original_api = PhysxSchema.PhysxArticulationAPI(original_root)
        solver_position = original_api.GetSolverPositionIterationCountAttr().Get() or 64
        solver_velocity = (args.solver_velocity_iterations if args.solver_velocity_iterations is not None
                           else original_api.GetSolverVelocityIterationCountAttr().Get())
        if solver_velocity is None:
            solver_velocity = 4
        art_api.CreateSolverPositionIterationCountAttr().Set(solver_position)
        art_api.CreateSolverVelocityIterationCountAttr().Set(solver_velocity)
        art_api.CreateEnabledSelfCollisionsAttr().Set(
            bool(original_api.GetEnabledSelfCollisionsAttr().Get()))
        report['solver'] = {'type': args.solver_type,
                            'external_forces_every_iteration': scene_api.GetEnableExternalForcesEveryIterationAttr().Get(),
                            'position_iterations': solver_position,
                            'velocity_iterations': solver_velocity}

        def joint(kind, name, body0, body1, axis, low, high, stiffness, damping, ceiling):
            value = kind.Define(stage, f'{scene_root}/joints/{name}')
            value.CreateBody0Rel().SetTargets([body0])
            value.CreateBody1Rel().SetTargets([body1])
            value.CreateAxisAttr().Set(axis)
            value.CreateLowerLimitAttr().Set(low)
            value.CreateUpperLimitAttr().Set(high)
            value.CreateLocalPos0Attr().Set(Gf.Vec3f(0))
            value.CreateLocalPos1Attr().Set(Gf.Vec3f(0))
            drive = UsdPhysics.DriveAPI.Apply(value.GetPrim(), 'linear' if kind == UsdPhysics.PrismaticJoint else 'angular')
            drive.CreateTypeAttr().Set('force')
            drive.CreateStiffnessAttr().Set(stiffness)
            drive.CreateDampingAttr().Set(damping)
            drive.CreateMaxForceAttr().Set(ceiling)
            drive.CreateTargetPositionAttr().Set(0)
            return value

        joint(UsdPhysics.PrismaticJoint, 'lift', anchor_path, lift_path, 'Z', -.15, .05, 1e6, 3000, 5000)
        # USD angular drive gains are per degree; PhysX tensors use radians.
        wrist = joint(UsdPhysics.RevoluteJoint, 'swing', lift_path, base_path, args.swing_axis,
                      -40, 40, 4000 * math.pi / 180, 40 * math.pi / 180, 500)
        wrist.CreateLocalRot1Attr().Set(down)
        finger_name = contract['usd']['actuated_joint']
        finger = stage.GetPrimAtPath(f'{root}/Physics/{finger_name}')
        drive_kind = 'angular' if args.model.startswith('rg') else 'linear'
        drive = UsdPhysics.DriveAPI(finger, drive_kind)
        finger_api = PhysxSchema.PhysxJointAPI.Apply(finger)
        if args.drive_armature_kg_m2 is not None:
            finger_api.CreateArmatureAttr().Set(args.drive_armature_kg_m2)
        if args.drive_armature_kg is not None:
            finger_api.CreateArmatureAttr().Set(args.drive_armature_kg)
        if args.maximum_joint_velocity_rad_s is not None:
            if args.maximum_joint_velocity_rad_s > math.radians(finger_api.GetMaxJointVelocityAttr().Get()) + 1e-6:
                raise ValueError('diagnostic velocity ceiling may only reduce the asset limit')
            finger_api.GetMaxJointVelocityAttr().Set(math.degrees(args.maximum_joint_velocity_rad_s))
        stock_ceiling = drive.GetMaxForceAttr().Get()
        stock_stiffness = drive.GetStiffnessAttr().Get()
        stock_damping = drive.GetDampingAttr().Get()
        drive.CreateMaxForceAttr().Set(stock_ceiling * args.effort_fraction)
        angular_scale = math.pi / 180 if drive_kind == 'angular' else 1.0
        if args.drive_stiffness_si is not None:
            drive.CreateStiffnessAttr().Set(args.drive_stiffness_si * angular_scale)
        if args.drive_damping_si is not None:
            drive.CreateDampingAttr().Set(args.drive_damping_si * angular_scale)
        report['gripper_drive'] = {'kind': drive_kind, 'stock_ceiling': stock_ceiling,
                                    'used_ceiling': stock_ceiling * args.effort_fraction,
                                    'stock_stiffness_usd': stock_stiffness,
                                    'stock_damping_usd': stock_damping,
                                    'stiffness': drive.GetStiffnessAttr().Get(),
                                    'damping': drive.GetDampingAttr().Get()}
        report['gripper_drive']['armature'] = finger_api.GetArmatureAttr().Get()
        report['gripper_drive']['armature_unit'] = 'kg m^2' if drive_kind == 'angular' else 'kg'
        report['mimic_constraints'] = {}
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root + '/Physics')):
            for schema in prim.GetAppliedSchemas():
                if not schema.startswith('PhysxMimicJointAPI:'):
                    continue
                axis = schema.split(':', 1)[1]
                mimic = PhysxSchema.PhysxMimicJointAPI(prim, axis)
                if mimic.GetReferenceJointRel().GetTargets() != [finger.GetPath()]:
                    raise RuntimeError('unsupported mimic reference: ' + str(prim.GetPath()))
                if args.mimic_frequency_hz is not None:
                    mimic.CreateNaturalFrequencyAttr().Set(args.mimic_frequency_hz)
                if args.mimic_damping_ratio is not None:
                    mimic.CreateDampingRatioAttr().Set(args.mimic_damping_ratio)
                report['mimic_constraints'][prim.GetName()] = {
                    'frequency_hz': mimic.GetNaturalFrequencyAttr().Get(),
                    'damping_ratio': mimic.GetDampingRatioAttr().Get(),
                    'gearing': mimic.GetGearingAttr().Get(),
                    'offset': mimic.GetOffsetAttr().Get() or 0.0}

        def cube(path, size, position, color):
            shape = UsdGeom.Cube.Define(stage, path)
            shape.CreateSizeAttr().Set(1.0)
            shape.AddTranslateOp().Set(Gf.Vec3d(*position))
            shape.AddScaleOp().Set(Gf.Vec3f(*size))
            shape.CreateDisplayColorAttr().Set([Gf.Vec3f(*color)])
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
            return shape.GetPrim()

        table_path, block_path = f'{scene_root}/table', f'{scene_root}/cube'
        cube(table_path, [.7, .5, .06], [0, 0, table_top - .03], [.25, .28, .30])
        block_prim = cube(block_path, [args.cube_size_m] * 3,
                          [0, 0, table_top + args.cube_size_m / 2 + .001], [.45, .48, .51])
        UsdPhysics.RigidBodyAPI.Apply(block_prim)
        # Keep contact reporting active for the support-weight sanity check.
        block_api = PhysxSchema.PhysxRigidBodyAPI.Apply(block_prim)
        block_api.CreateSleepThresholdAttr().Set(0.0)
        # Contact islands use the maximum requested count. A zero-iteration
        # diagnostic must include the workpiece, whose default is nonzero.
        if args.solver_velocity_iterations is not None:
            block_api.CreateSolverVelocityIterationCountAttr().Set(args.solver_velocity_iterations)
        report['cube_solver_velocity_iterations'] = block_api.GetSolverVelocityIterationCountAttr().Get()
        mass = UsdPhysics.MassAPI.Apply(block_prim)
        mass.CreateMassAttr().Set(args.payload_kg)
        mass.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(args.payload_kg * args.cube_size_m ** 2 / 6))
        # Coordinates are local to the unit cube before its inherited scale.
        # No collision, mass or extra rigid body is added by this label.
        define_textured_sign(stage, block_path + '/mass_label', create_payload_label(args.payload_kg),
                             [0, -.501, 0], [.85, .29], facing='-Y')
        material = UsdShade.Material.Define(stage, f'{scene_root}/rubber_metal_pair')
        physmat = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physmat.CreateStaticFrictionAttr().Set(args.static_friction)
        physmat.CreateDynamicFrictionAttr().Set(args.dynamic_friction)
        physmat.CreateRestitutionAttr().Set(0)
        PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr().Set('average')
        UsdShade.MaterialBindingAPI.Apply(block_prim).Bind(material, materialPurpose='physics')
        pad_material = material
        if args.contact_stiffness_n_m:
            pad_material = UsdShade.Material.Define(stage, f'{scene_root}/compliant_pad_pair')
            pad = UsdPhysics.MaterialAPI.Apply(pad_material.GetPrim())
            pad.CreateStaticFrictionAttr().Set(args.static_friction)
            pad.CreateDynamicFrictionAttr().Set(args.dynamic_friction)
            pad.CreateRestitutionAttr().Set(0)
            compliance = PhysxSchema.PhysxMaterialAPI.Apply(pad_material.GetPrim())
            compliance.CreateFrictionCombineModeAttr().Set('average')
            compliance.CreateCompliantContactAccelerationSpringAttr().Set(False)
            compliance.CreateCompliantContactStiffnessAttr().Set(args.contact_stiffness_n_m)
            compliance.CreateCompliantContactDampingAttr().Set(args.contact_damping_ns_m)
        report['pad_compliance'] = {'stiffness_n_m': args.contact_stiffness_n_m,
                                    'damping_ns_m': args.contact_damping_ns_m,
                                    'scope': 'pad material only; rigid cube and table unchanged'}
        report['fingertip_material_bindings'] = {}
        report['pad_collision_approximations'] = {}
        report['effective_pad_materials'] = {}
        for side in ('left', 'right'):
            tip = f'{root}/Geometry/{side}_' + ('finger_tip_link' if args.model.startswith('rg') else 'fingertip_link')
            tip_prim = stage.GetPrimAtPath(tip)
            if args.pad_approximation is not None:
                # Only a diagnostic local stage opinion. De-instance this pad
                # before authoring child collider attributes, never its source.
                for prim in list(Usd.PrimRange(tip_prim)):
                    if prim.IsInstance():
                        prim.SetInstanceable(False)
            for prim in Usd.PrimRange(tip_prim, Usd.TraverseInstanceProxies()):
                if prim.HasAPI(UsdPhysics.CollisionAPI) and prim.IsA(UsdGeom.Mesh):
                    mesh_collision = UsdPhysics.MeshCollisionAPI(prim)
                    if args.pad_approximation is not None:
                        mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
                        mesh_collision.CreateApproximationAttr().Set(args.pad_approximation)
                    report['pad_collision_approximations'][str(prim.GetPath())] = mesh_collision.GetApproximationAttr().Get()
            # Bind to each collider so inherited imported materials cannot win.
            report['fingertip_material_bindings'][side] = _collision_material_scope(
                tip_prim, pad_material if args.override_pad_material or args.contact_stiffness_n_m else None,
                Usd, UsdPhysics, UsdShade)
            for path in report['fingertip_material_bindings'][side]['collision_paths']:
                bound, _ = UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(path)).ComputeBoundMaterial('physics')
                if not bound:
                    raise RuntimeError('pad collider has no physics material: ' + path)
                material_api = UsdPhysics.MaterialAPI(bound.GetPrim())
                report['effective_pad_materials'][path] = {
                    'path': str(bound.GetPath()),
                    'static': material_api.GetStaticFrictionAttr().Get(),
                    'dynamic': material_api.GetDynamicFrictionAttr().Get(),
                    'combine': PhysxSchema.PhysxMaterialAPI(bound.GetPrim()).GetFrictionCombineModeAttr().Get()}
        report['material_pair'] = {'static': args.static_friction, 'dynamic': args.dynamic_friction,
                                  'combine': 'average', 'scope': 'object; pad values reported separately',
                                  'pad_override': bool(args.override_pad_material or args.contact_stiffness_n_m),
                                  'basis': 'declared dry rubber/metal assumption; not a measured coefficient'}
        report['lighting'] = create_studio_lighting(stage, f'{scene_root}/lights')
        backdrop = cube(f'{scene_root}/backdrop', [1.8, .01, 1.2], [0, .30, .54], [.12, .14, .16])
        UsdPhysics.CollisionAPI(backdrop).CreateCollisionEnabledAttr().Set(False)
        texture = create_showcase_sign(args.model, args.payload_kg,
                                      package_root() / 'resource/branding/logo_onrobot_rgb.png')
        define_textured_sign(stage, f'{scene_root}/sign', texture,
                             [-.33, .29, base_height - .04], [.22, .11], facing='-Y')
        camera_center_z = (base_height + table_top) / 2
        camera = UsdGeom.Camera.Define(stage, f'{scene_root}/camera')
        camera.CreateFocalLengthAttr().Set(26)
        camera.CreateClippingRangeAttr().Set(Gf.Vec2f(.01, 20))
        camera_distance = 1.15 if args.model == 'rg6' else 1.0
        camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(.24 * camera_distance, -camera_distance,
                     camera_center_z + .12 * camera_distance), Gf.Vec3d(0, 0, camera_center_z),
            Gf.Vec3d(0, 0, 1)).GetInverse())
        from isaacsim.core.rendering_manager import ViewportManager
        ViewportManager.set_camera(camera.GetPrim())
        viewport = ViewportManager.get_viewport_api()
        # Appearance-only diagnostic frame is captured before physics starts.
        async def capture(name):
            if not args.screenshots:
                return
            # Complete the deferred Hydra/RTX frames at this fixed physics
            # instant before saving; a single update may still show old poses.
            for _ in range(8):
                await app.next_update_async()
            from omni.kit.viewport.utility import capture_viewport_to_file
            path = args.output.parent / (args.output.stem + '-frames') / f'{name}.png'
            path.parent.mkdir(parents=True, exist_ok=True)
            await capture_viewport_to_file(viewport, file_path=str(path)).wait_for_result()
            report['frames'].append(str(path.relative_to(args.output.parent)))
            if 'block' in observed_bodies:
                frame_block, frame_base = observed_bodies['block'], observed_bodies['base']
                report.setdefault('frame_states', []).append({
                    'name': name, 'simulation_time_s': float(SimulationManager.get_simulation_time()),
                    'camera_path': str(viewport.camera_path),
                    'cube_position_m': _row(frame_block.get_world_poses()[0]),
                    'base_orientation_wxyz': _row(frame_base.get_world_poses()[1]),
                    'usd_cube_position_m': list(UsdGeom.Xformable(block_prim).
                        ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation())})

        observed_bodies = {}
        for _ in range(12):
            await app.next_update_async()
        await capture('00_scene')
        PhysxSchema.PhysxContactReportAPI.Apply(block_prim).CreateThresholdAttr().Set(0)
        contacts = {}
        friction_forces = {}
        cube_contact_forces = {}
        contact_details = []
        contact_errors = []

        def read_contacts(headers, data, friction_anchors):
            for header in headers:
                a = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
                b = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
                if block_path not in (a, b):
                    continue
                other = b if a == block_path else a
                cube_sign = 1.0 if a == block_path else -1.0
                for entry in data[header.contact_data_offset:header.contact_data_offset + header.num_contact_data]:
                    impulse = np.asarray(entry.impulse, dtype=float)
                    normal = np.asarray(entry.normal, dtype=float)
                    normal_force, _ = contact_force_components(impulse, normal, 1 / args.physics_hz)
                    contacts[other] = contacts.get(other, 0.0) + normal_force
                    cube_contact_forces[other] = (cube_contact_forces.get(other, np.zeros(3))
                                                   + cube_sign * impulse * args.physics_hz)
                    if normal_force > 1e-4 and len(contact_details) < 64:
                        contact_details.append({'body': other, 'position_m': list(entry.position),
                                                'cube_is_actor0': a == block_path,
                                                'normal': normal.tolist(), 'normal_force_n': normal_force})
                # Ordinary contact impulses contain the normal contribution.
                # Friction is reported separately at the patch anchors; do not
                # mislabel a zero tangent projection as zero physical friction.
                for entry in friction_anchors[header.friction_anchors_offset:
                                               header.friction_anchors_offset + header.num_friction_anchors_data]:
                    force = np.asarray(entry.impulse, dtype=float) * args.physics_hz
                    if not np.isfinite(force).all():
                        raise RuntimeError('non-finite friction impulse')
                    friction_forces[other] = friction_forces.get(other, np.zeros(3)) + force
                    cube_contact_forces[other] = cube_contact_forces.get(other, np.zeros(3)) + cube_sign * force

        def contact_event(headers, data, friction_anchors):
            try:
                read_contacts(headers, data, friction_anchors)
            except Exception as error:
                # Exceptions inside PhysX callbacks may otherwise only be logged.
                contact_errors.append(f'{type(error).__name__}: {error}')

        contact_subscription = omni.physx.get_physx_simulation_interface().subscribe_full_contact_report_events(contact_event)
        report['contact_observation'] = {
            'normal': 'ordinary contact impulse projected on its normal / dt',
            'friction': 'separately reported patch-anchor impulses / dt',
            'friction_vector_sign': 'PhysX actor-pair convention; magnitudes do not assume a body sign',
            'cube_force_sign': 'actor0 impulse, reversed where cube is actor1; compare supported weight before use',
            'sampled_cube_force': 'cube_contact_force_average integrates every physics step since the previous sample; instantaneous vectors remain separate'}
        # Constructors may apply PhysxRigidBodyAPI. Finish those schema edits
        # before the articulation tensor view is created.
        block, base = RigidPrim(block_path), RigidPrim(base_path)
        observed_bodies.update(block=block, base=base)
        dt = setup_simulation(SimulationManager, 1 / args.physics_hz, device='cpu')
        if not math.isclose(dt, 1 / args.physics_hz):
            raise RuntimeError('effective physics timestep differs from request')
        fabric_enabled = SimulationManager.is_fabric_enabled()
        report['render_transform_path'] = 'fabric' if fabric_enabled else 'usd'
        art = Articulation(str(mount_root.GetPath()))
        timeline.play()
        for _ in range(2):
            await app.next_update_async()
        timeline.pause()
        await app.next_update_async()
        ViewportManager.set_camera(camera.GetPrim())
        names = art.dof_names
        report['dof_names'] = names
        indices = [names.index(n) for n in ['lift', 'swing', finger_name]]
        report['effective_dof_gains_si'] = [value.numpy().tolist() for value in art.get_dof_gains()]
        hold_gains = tuple(_row(value)[indices[2]] for value in art.get_dof_gains())
        report['effective_dof_max_efforts_si'] = art.get_dof_max_efforts().numpy().tolist()
        report['articulation_solver_counts'] = [value.numpy().tolist() for value in art.get_solver_iteration_counts()]
        upper = float(UsdPhysics.Joint(finger).GetPrim().GetAttribute('physics:upperLimit').Get())
        open_target = math.radians(upper) if drive_kind == 'angular' else upper
        grasp_target = 0.0
        step_count = 0
        contact_average = ContactForceAverage(dt)
        phase = 'settle'
        cycle_index = 0
        reference = None
        slip_values, rotation_values = [], []
        peak_swing = 0.0
        maximum_lift = 0.0
        last_mount_targets = (0.0, 0.0)
        effort_hold = False
        peak_table_load = 0.0
        peak_closure_table_load = 0.0
        peak_nonpad_contact = 0.0
        peak_mimic_error = 0.0
        allowed_pad_bodies = {
            f'{root}/Geometry/{side}_' + ('finger_tip_link' if args.model.startswith('rg') else 'fingertip_link')
            for side in ('left', 'right')}
        ordered_pad_bodies = sorted(allowed_pad_bodies)
        grasp_window = deque()
        time_origin = float(SimulationManager.get_simulation_time())
        report['tensor_view_reacquires'] = 0

        def current_articulation():
            nonlocal art
            if not art.is_physics_tensor_entity_valid():
                art = Articulation(str(mount_root.GetPath()))
                report['tensor_view_reacquires'] += 1
                if report['tensor_view_reacquires'] > 4 or not art.is_physics_tensor_entity_valid():
                    raise RuntimeError('articulation tensor view is repeatedly invalidated')
            return art

        async def step(lift, swing, jaw):
            nonlocal step_count, peak_swing, maximum_lift, peak_table_load, peak_closure_table_load, last_mount_targets, peak_nonpad_contact, peak_mimic_error
            contacts.clear()
            friction_forces.clear()
            cube_contact_forces.clear()
            contact_details.clear()
            current_articulation().set_dof_position_targets([lift, swing, jaw], dof_indices=indices)
            current_articulation().set_dof_velocity_targets(
                [(lift - last_mount_targets[0]) / dt, (swing - last_mount_targets[1]) / dt],
                dof_indices=indices[:2])
            if args.hold_control == 'effort':
                current_articulation().set_dof_efforts(
                    -stock_ceiling * args.effort_fraction if effort_hold else 0.0,
                    dof_indices=[indices[2]])
            last_mount_targets = (lift, swing)
            # A paused timeline does not refresh the render fabric on its own.
            # Publish actual simulation transforms; never pose-drive the cube.
            SimulationManager.step(update_fabric=fabric_enabled)
            if not fabric_enabled:
                omni.physx.get_physx_interface().update_transformations(False, True, False)
            if contact_errors:
                raise RuntimeError('contact observer failed: ' + contact_errors[0])
            contact_average.add(cube_contact_forces)
            step_count += 1
            expected_time = time_origin + step_count * dt
            if abs(SimulationManager.get_simulation_time() - expected_time) > 1e-5:
                raise RuntimeError('physics time differs from explicitly requested steps')
            bp, bq = _row(block.get_world_poses()[0]), _row(block.get_world_poses()[1])
            gp, gq = _row(base.get_world_poses()[0]), _row(base.get_world_poses()[1])
            joints = _row(current_articulation().get_dof_positions())
            velocities = _row(current_articulation().get_dof_velocities())
            if not np.isfinite([*bp, *bq, *gp, *gq, *joints, *velocities]).all():
                raise RuntimeError('non-finite PhysX state')
            if phase == 'grasp_settle':
                grasp_window.append({'time_s': step_count * dt, 'jaw': joints[indices[2]],
                                     'cube': bp,
                                     'pad_normals': [contacts.get(p, 0.) for p in ordered_pad_bodies]})
                while grasp_window[-1]['time_s'] - grasp_window[0]['time_s'] > .2 + dt:
                    grasp_window.popleft()
            if phase != 'settle':
                for name, mimic in report['mimic_constraints'].items():
                    peak_mimic_error = max(peak_mimic_error, abs(
                        joints[names.index(name)] + mimic['gearing'] * joints[indices[2]] + mimic['offset']))
            relative = _relative_pose(gp, gq, bp, bq)
            if reference is not None and phase in ('lift', 'swing', 'lower'):
                slip_values.append(float(np.linalg.norm(np.array(relative[0]) - reference[0])))
                rotation_values.append(_rotation_error(relative[1], reference[1]))
            if phase == 'swing':
                peak_swing = max(peak_swing, abs(joints[indices[1]]))
                maximum_lift = max(maximum_lift, bp[2] - table_top - args.cube_size_m / 2)
                peak_nonpad_contact = max(peak_nonpad_contact,
                    sum(force for body, force in contacts.items() if body not in allowed_pad_bodies))
            if phase == 'approach':
                load = contacts.get(table_path, 0.0)
                if load > peak_table_load:
                    peak_table_load = load
                    report['peak_table_contact_details'] = list(contact_details)
            if phase in ('close', 'preload', 'grasp_settle'):
                peak_closure_table_load = max(peak_closure_table_load, contacts.get(table_path, 0.0))
            if step_count % 4 == 0:
                report['samples'].append({'time_s': step_count * dt, 'cycle': cycle_index, 'phase': phase,
                                          'jaw_position_target': jaw,
                                          'direct_jaw_effort': (-stock_ceiling * args.effort_fraction
                                                                if effort_hold else 0.0),
                                          'cube_position_m': bp, 'cube_orientation_wxyz': bq,
                                          'base_position_m': gp, 'base_orientation_wxyz': gq,
                                          'joint_positions': [joints[i] for i in indices],
                                          'joint_velocities': [velocities[i] for i in indices],
                                          'normal_forces_n': dict(contacts),
                                          'friction_force_vectors_n': {body: value.tolist()
                                                                       for body, value in friction_forces.items()},
                                          'cube_contact_force_vectors_n': {body: value.tolist()
                                                                         for body, value in cube_contact_forces.items()},
                                          'cube_contact_force_average': contact_average.take()})
            if step_count % max(1, round(args.physics_hz / (60 if args.gui else 15))) == 0:
                await app.next_update_async()
            return relative, bp

        async def segment(duration, start, end):
            count = max(1, round(duration / dt))
            result = None
            for i in range(count):
                u = (i + 1) / count
                s = u ** 3 * (10 + u * (-15 + 6 * u))
                result = await step(*(a + (b - a) * s for a, b in zip(start, end)))
            return result

        await segment(1, (0, 0, open_target), (0, 0, open_target))
        report['supported_weight_check'] = supported_weight_check(
            [sample for sample in report['samples'] if sample['time_s'] > .8], args.payload_kg)
        if not report['supported_weight_check']['passed']:
            raise RuntimeError('contact observer does not recover the supported workpiece weight')
        await capture('01_open')
        for cycle_index in range(args.cycles):
            grasp_target = 0.0
            slip_values, rotation_values = [], []
            peak_swing, maximum_lift, peak_table_load = 0.0, 0.0, 0.0
            peak_closure_table_load = 0.0
            peak_nonpad_contact = 0.0
            peak_mimic_error = 0.0
            reference = None
            phase = 'approach'
            await segment(1, (0, 0, open_target), (-lift_distance, 0, open_target))
            phase = 'close'
            await segment(args.close_seconds, (-lift_distance, 0, open_target), (-lift_distance, 0, grasp_target))
            if args.hold_control == 'effort':
                current_articulation().set_dof_gains(0., 0., dof_indices=[indices[2]], update_default_gains=False)
                effort_hold = True
            elif args.hold_control == 'preload-position':
                # No cube or joint pose reset. Shift only the drive reference
                # after both pads touch; leave gains, limits and friction intact.
                if not all(contacts.get(path, 0.0) > .1 for path in ordered_pad_bodies):
                    raise RuntimeError('impedance preload requires observed bilateral pad contact')
                contact_q = _row(current_articulation().get_dof_positions())[indices[2]]
                grasp_target = linear_preload_target(
                    contact_q, hold_gains[0], stock_ceiling * args.effort_fraction)
                report.setdefault('impedance_preloads', []).append({
                    'cycle': cycle_index, 'contact_joint_position_m': contact_q,
                    'hold_reference_m': grasp_target,
                    'generalized_force_reference_n': stock_ceiling * args.effort_fraction,
                    'per_pad_reference_n': stock_ceiling * args.effort_fraction / 2,
                    'stiffness_n_m': hold_gains[0],
                    'basis': 'contact-relative impedance reference; not closed-loop force regulation'})
                phase = 'preload'
                await segment(args.force_ramp_seconds,
                              (-lift_distance, 0, 0.), (-lift_distance, 0, grasp_target))
            phase = 'grasp_settle'
            grasp_window.clear()
            grasp_settled = False
            for settling_step in range(math.ceil(args.grasp_settle_timeout / dt)):
                reference, _ = await step(-lift_distance, 0, grasp_target)
                settling_duration = (settling_step + 1) * dt
                if (settling_duration >= args.preload_seconds and
                        grasp_is_settled(list(grasp_window), drive_kind == 'angular')):
                    grasp_settled = True
                    break
            if not grasp_settled:
                report['cycles'].append({'cycle': cycle_index, 'passed': False,
                                         'reason': 'grasp did not settle before the bounded timeout',
                                         'grasp_settled': False, 'grasp_settling_duration_s': settling_duration,
                                         'maximum_lift_m': 0.0})
                break
            report.setdefault('grasp_contacts', []).append(list(contact_details))
            report.setdefault('grasp_joint_states', []).append({
                'positions': current_articulation().get_dof_positions().numpy().tolist(),
                'velocities': current_articulation().get_dof_velocities().numpy().tolist(),
                'projected_joint_forces': current_articulation().get_dof_projected_joint_forces().numpy().tolist()})
            if cycle_index == 0:
                report['grasp_jacobians'] = {
                    'link_names': current_articulation().link_names,
                    'matrices': current_articulation().get_jacobian_matrices().numpy().tolist()}
                report['grasp_mass_matrix'] = current_articulation().get_mass_matrices().numpy().tolist()
            await capture(f'{cycle_index:02d}_grasp')
            phase = 'lift'
            await segment(1, (-lift_distance, 0, grasp_target), (0, 0, grasp_target))
            await capture(f'{cycle_index:02d}_lift')
            phase = 'swing'
            swing_steps = round(args.swing_cycles / args.swing_hz / dt)
            for i in range(swing_steps):
                t = (i + 1) * dt
                envelope = min(1.0, t * args.swing_hz, (swing_steps * dt - t) * args.swing_hz)
                angle = math.radians(args.swing_deg) * max(0, envelope) * math.sin(2 * math.pi * args.swing_hz * t)
                await step(0, angle, grasp_target)
                if cycle_index == 0 and i in (swing_steps // 3, swing_steps // 3 + round(.5 / args.swing_hz / dt)):
                    await capture(f'02_swing_{i}')
            await segment(.4, (0, 0, grasp_target), (0, 0, grasp_target))
            phase = 'lower'
            await segment(1, (0, 0, grasp_target), (-lift_distance, 0, grasp_target))
            phase = 'release'
            if args.hold_control == 'effort':
                effort_hold = False
                current_articulation().set_dof_efforts(0., dof_indices=[indices[2]])
                current_articulation().set_dof_gains(*hold_gains, dof_indices=[indices[2]], update_default_gains=False)
            await segment(.7, (-lift_distance, 0, grasp_target), (-lift_distance, 0, open_target))
            await segment(.5, (-lift_distance, 0, open_target), (-lift_distance, 0, open_target))
            phase = 'withdraw'
            _, placed = await segment(1, (-lift_distance, 0, open_target), (0, 0, open_target))
            await capture(f'{cycle_index:02d}_released')
            result = {'cycle': cycle_index, 'maximum_relative_slip_m': max(slip_values, default=0),
                      'grasp_settled': grasp_settled, 'grasp_settling_duration_s': settling_duration,
                      'maximum_relative_rotation_rad': max(rotation_values, default=0),
                      'placement_xy_error_m': math.hypot(placed[0], placed[1]),
                      'placement_height_error_m': abs(placed[2] - table_top - args.cube_size_m / 2),
                      'maximum_lift_m': maximum_lift, 'maximum_swing_rad': peak_swing,
                      'peak_table_load_during_approach_n': peak_table_load,
                      'peak_table_load_during_closure_n': peak_closure_table_load,
                      'peak_nonpad_contact_during_swing_n': peak_nonpad_contact,
                      'maximum_mimic_error': peak_mimic_error,
                      'mimic_error_unit': 'rad' if drive_kind == 'angular' else 'm'}
            result['passed'] = cycle_passes(result, args.payload_kg, args.swing_deg, args.model)
            report['cycles'].append(result)
            save()
            if not result['passed']:
                break
        report['status'] = 'passed' if len(report['cycles']) == args.cycles and all(c['passed'] for c in report['cycles']) else 'failed'
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = f'{type(error).__name__}: {error}'
        report['traceback'] = traceback.format_exc().splitlines()
    finally:
        timeline.stop()
        contact_subscription = None
        save()
    return report


def main():
    args = arguments()
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': not args.gui, 'renderer': 'RayTracedLighting',
                         'multi_gpu': False, 'width': 1280, 'height': 720})
    task = asyncio.ensure_future(run(args))
    while not task.done():
        app.update()
    report = task.result()
    print(json.dumps({k: v for k, v in report.items() if k != 'samples'}, indent=2))
    code = 0 if report['status'] == 'passed' else 1
    from isaac_runtime_compat import close_app
    close_app(app, code)
    return code


if __name__ == '__main__':
    sys.exit(main())

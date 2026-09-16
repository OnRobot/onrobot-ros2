#!/usr/bin/env python3
"""Run repeated 2FG-family dynamic contact/grasp trials in Isaac Sim."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

from isaac_model_contract import articulation_path
from isaac_model_contract import default_asset
from isaac_model_contract import load_contract
from isaac_model_contract import SUPPORTED_MODELS

from isaac_runtime_compat import close_app
from isaac_runtime_compat import is_stage_loading
from isaac_runtime_compat import play
from isaac_runtime_compat import setup_simulation
from isaac_runtime_compat import stop

from isaacsim import SimulationApp


SCENARIO_ROOT = '/onrobot_dynamic_contact'
OBJECT_PATH = f'{SCENARIO_ROOT}/reference_object'
SUPPORT_PATH = f'{SCENARIO_ROOT}/acquisition_support'
SUPPORT_ANCHOR_PATH = f'{SCENARIO_ROOT}/acquisition_support_anchor'
SUPPORT_JOINT_PATH = f'{SCENARIO_ROOT}/acquisition_support_joint'
MAX_RECORDED_CONTACT_PAIRS = 16
HARNESS_REVISION = 9
DEFAULT_RESET_LIFT_STEPS = 120
RESET_POSITION_TOLERANCE_M = 0.001
RESET_ROTATION_TOLERANCE_RAD = 0.05
RESET_MAXIMUM_OBJECT_STEP_M = 0.001
RESET_MAXIMUM_CARRIER_OVERRUN_M = 0.01
RESET_CARRIER_TARGET_TOLERANCE_M = 0.0005
SUPPORT_DRIVE_STIFFNESS_N_M = 10000.0
SUPPORT_DRIVE_DAMPING_N_S_M = 100.0
SUPPORT_DRIVE_MAXIMUM_FORCE_N = 100.0
SUPPORT_DRIVE_LIMIT_M = 0.05
SUPPORT_POSITIONING_MAXIMUM_STEPS = 240
GUI_INSPECTION_DELAY_S = 2.0


def _package_path(relative: str) -> Path:
    source = Path(__file__).resolve().parents[1] / relative
    if source.is_file():
        return source
    return (Path(__file__).resolve().parents[2] / 'share' /
            'onrobot_gripper_isaac' / relative)


def _write_report(path: Path, report) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')


def _default_config(model: str) -> Path:
    return _package_path(
        f'config/{model}_dynamic_contact_reference_objects.json')


def _as_numpy(value):
    return value.numpy() if hasattr(value, 'numpy') else value


def _row(value, index: int = 0):
    return [float(item) for item in _as_numpy(value)[index]]


def _scalar(value, row: int, column: int) -> float:
    return float(_as_numpy(value)[row, column])


def _finite(values) -> bool:
    return all(math.isfinite(value) for value in values)


def _rotation_delta(a, b) -> float:
    dot = abs(sum(left * right for left, right in zip(a, b)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def _bind_physics_material(prim, material, UsdShade) -> None:
    binding = UsdShade.MaterialBindingAPI.Apply(prim)
    binding.Bind(
        material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        materialPurpose='physics')


def _create_material(stage, path, values, PhysxSchema, UsdPhysics,
                     UsdShade):
    material = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr().Set(float(values['static_friction']))
    api.CreateDynamicFrictionAttr().Set(float(values['dynamic_friction']))
    api.CreateRestitutionAttr().Set(float(values['restitution']))
    combine_mode = str(values.get('friction_combine_mode', 'average'))
    if combine_mode not in {'average', 'min', 'multiply', 'max'}:
        raise RuntimeError(
            f'unsupported friction combine mode: {combine_mode}')
    physx_api = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx_api.CreateFrictionCombineModeAttr().Set(combine_mode)
    return material


def _combined_friction(left: float, right: float, mode: str) -> float:
    if mode == 'average':
        return (left + right) / 2.0
    if mode == 'min':
        return min(left, right)
    if mode == 'multiply':
        return left * right
    if mode == 'max':
        return max(left, right)
    raise RuntimeError(f'unsupported friction combine mode: {mode}')


def _validate_object_placement(config, object_config):
    geometry = config['reference_geometry_m']
    center_x, center_y, center_z = (
        float(value) for value in object_config['center_m'])
    size_x, size_y, size_z = (
        float(value) for value in object_config['size_m'])
    center_limit = float(geometry['maximum_center_offset_xy'])
    if abs(center_x) > center_limit or abs(center_y) > center_limit:
        raise RuntimeError('reference object is not centered on the fingers')

    object_min_z = center_z - size_z / 2.0
    object_max_z = center_z + size_z / 2.0
    base_clearance = (
        object_min_z - float(geometry['base_collision_max_z']))
    if base_clearance < float(geometry['minimum_base_clearance']):
        raise RuntimeError('reference object overlaps or is too close to the '
                           'gripper base')

    contact_z_min, contact_z_max = (
        float(value) for value in geometry['contact_face_z_range'])
    contact_face_z_overlap = max(
        0.0,
        min(object_max_z, contact_z_max) -
        max(object_min_z, contact_z_min))
    if contact_face_z_overlap < float(
            geometry['minimum_contact_face_z_overlap']):
        raise RuntimeError(
            'reference object misses the vertical contact faces')

    object_half_x = size_x / 2.0
    contact_face_inner_x = float(
        geometry['contact_face_inner_x_at_zero'])
    release_clearance = (
        contact_face_inner_x + float(object_config['release_target_m']) -
        object_half_x)
    if release_clearance < float(geometry['minimum_release_clearance']):
        raise RuntimeError('reference object does not clear the released '
                           'fingertips')
    grasp_interference = (
        object_half_x - contact_face_inner_x -
        float(object_config['grasp_target_m']))
    if grasp_interference < float(geometry['minimum_grasp_interference']):
        raise RuntimeError('grasp target cannot reach the reference object')

    lateral_overlap = min(
        size_y / 2.0, float(geometry['fingertip_half_width_y']))
    if lateral_overlap < float(geometry['minimum_lateral_overlap']):
        raise RuntimeError('reference object has insufficient fingertip '
                           'lateral overlap')

    support_center = [
        float(value) for value in object_config['support_center_m']]
    support_size = [
        float(value) for value in object_config['support_size_m']]
    if min(support_size) <= 0.0:
        raise RuntimeError('acquisition support dimensions are invalid')
    support_top_z = support_center[2] + support_size[2] / 2.0
    support_bottom_z = support_center[2] - support_size[2] / 2.0
    if (abs(support_center[0] - center_x) > center_limit or
            abs(support_center[1] - center_y) > center_limit):
        raise RuntimeError('acquisition support is not centered under the '
                           'reference object')
    if abs(support_top_z - object_min_z) > float(
            geometry['maximum_support_top_gap']):
        raise RuntimeError('acquisition support does not meet the bottom of '
                           'the reference object')
    support_base_clearance = (
        support_bottom_z - float(geometry['base_collision_max_z']))
    if support_base_clearance < float(
            geometry['minimum_support_base_clearance']):
        raise RuntimeError('acquisition support overlaps or is too close to '
                           'the gripper base')
    support_half_x = support_size[0] / 2.0
    support_finger_clearance = contact_face_inner_x - support_half_x
    if support_finger_clearance < float(
            geometry['minimum_support_finger_clearance']):
        raise RuntimeError('acquisition support obstructs the closing fingers')

    return {
        'object_z_range_m': [object_min_z, object_max_z],
        'base_clearance_m': base_clearance,
        'contact_face_z_overlap_m': contact_face_z_overlap,
        'release_clearance_m': release_clearance,
        'grasp_interference_m': grasp_interference,
        'lateral_overlap_m': lateral_overlap,
        'support_top_gap_m': support_top_z - object_min_z,
        'support_base_clearance_m': support_base_clearance,
        'support_finger_clearance_m': support_finger_clearance,
    }


def _add_cube_collider(stage, path, translation, size, material, color,
                       Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateExtentAttr([
        Gf.Vec3f(-0.5, -0.5, -0.5),
        Gf.Vec3f(0.5, 0.5, 0.5),
    ])
    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
    xform.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    collision_api = UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    collision_api.CreateCollisionEnabledAttr().Set(False)
    collision = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
    collision.CreateContactOffsetAttr().Set(0.001)
    collision.CreateRestOffsetAttr().Set(0.0)
    _bind_physics_material(cube.GetPrim(), material, UsdShade)
    return cube.GetPrim()


def _add_box_mesh_collider(stage, path, translation, size, material, color,
                           Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade):
    """Create an exact unscaled box collider suitable for runtime motion."""
    half_x, half_y, half_z = (float(value) / 2.0 for value in size)
    points = [
        (-half_x, -half_y, -half_z),
        (half_x, -half_y, -half_z),
        (half_x, half_y, -half_z),
        (-half_x, half_y, -half_z),
        (-half_x, -half_y, half_z),
        (half_x, -half_y, half_z),
        (half_x, half_y, half_z),
        (-half_x, half_y, half_z),
    ]
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in points])
    mesh.CreateFaceVertexCountsAttr([4, 4, 4, 4, 4, 4])
    mesh.CreateFaceVertexIndicesAttr([
        0, 3, 2, 1,
        4, 5, 6, 7,
        0, 1, 5, 4,
        1, 2, 6, 5,
        2, 3, 7, 6,
        3, 0, 4, 7,
    ])
    mesh.CreateExtentAttr([
        Gf.Vec3f(-half_x, -half_y, -half_z),
        Gf.Vec3f(half_x, half_y, half_z),
    ])
    xform = UsdGeom.Xformable(mesh.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    collision_api = UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    collision_api.CreateCollisionEnabledAttr().Set(False)
    mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
    mesh_collision.CreateApproximationAttr().Set('convexHull')
    collision = PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim())
    collision.CreateContactOffsetAttr().Set(0.001)
    collision.CreateRestOffsetAttr().Set(0.0)
    _bind_physics_material(mesh.GetPrim(), material, UsdShade)
    return mesh.GetPrim()


def _bind_collision_materials(root_prim, material, Usd, UsdPhysics,
                              UsdShade):
    _bind_physics_material(root_prim, material, UsdShade)
    paths = []
    bound_paths = {str(root_prim.GetPath())}
    for prim in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies()):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            enabled = UsdPhysics.CollisionAPI(
                prim).GetCollisionEnabledAttr().Get()
            if enabled is False:
                continue
            paths.append(str(prim.GetPath()))
            binding_prim = prim
            while binding_prim.IsInstanceProxy():
                binding_prim = binding_prim.GetParent()
            binding_path = str(binding_prim.GetPath())
            if binding_path not in bound_paths:
                _bind_physics_material(binding_prim, material, UsdShade)
                bound_paths.add(binding_path)
    if not paths:
        raise RuntimeError(
            f'no collision geometry found below {root_prim.GetPath()}')
    return {
        'collision_paths': paths,
        'binding_paths': sorted(bound_paths),
    }


def _author_scenario(stage, config, object_config, fingertip_paths,
                     Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade):
    UsdGeom.Xform.Define(stage, SCENARIO_ROOT)
    root = UsdGeom.Xform.Define(stage, OBJECT_PATH)
    root_xform = UsdGeom.Xformable(root.GetPrim())
    root_xform.AddTranslateOp().Set(
        Gf.Vec3d(*[float(item) for item in object_config['center_m']]))

    body = UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    body.CreateRigidBodyEnabledAttr().Set(True)
    mass = UsdPhysics.MassAPI.Apply(root.GetPrim())
    mass.CreateMassAttr().Set(float(object_config['mass_kg']))
    physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(root.GetPrim())
    physx_body.CreateDisableGravityAttr().Set(True)
    physx_body.CreateEnableCCDAttr().Set(True)

    object_material = _create_material(
        stage, f'{SCENARIO_ROOT}/Materials/reference_object',
        object_config['material'], PhysxSchema, UsdPhysics, UsdShade)
    finger_material = _create_material(
        stage, f'{SCENARIO_ROOT}/Materials/reference_fingers',
        config['reference_fingers'], PhysxSchema, UsdPhysics, UsdShade)

    fingertip_collision_paths = []
    for path in fingertip_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f'missing reference fingertip prim: {path}')
        fingertip_collision_paths.append(_bind_collision_materials(
            prim, finger_material, Usd, UsdPhysics, UsdShade))

    size = tuple(float(value) for value in object_config['size_m'])
    if min(size) <= 0.0:
        raise RuntimeError('reference object dimensions are invalid')
    object_collision_prim = _add_cube_collider(
        stage, f'{OBJECT_PATH}/block', (0.0, 0.0, 0.0), size,
        object_material, (0.12, 0.42, 0.8), Gf, PhysxSchema, UsdGeom,
        UsdPhysics, UsdShade)
    support_collision_prim = _add_box_mesh_collider(
        stage, SUPPORT_PATH,
        tuple(float(value) for value in object_config['support_center_m']),
        tuple(float(value) for value in object_config['support_size_m']),
        object_material, (0.32, 0.34, 0.38), Gf, PhysxSchema, UsdGeom,
        UsdPhysics, UsdShade)
    support_body = UsdPhysics.RigidBodyAPI.Apply(support_collision_prim)
    support_body.CreateRigidBodyEnabledAttr().Set(True)
    support_body.CreateKinematicEnabledAttr().Set(False)
    support_mass = UsdPhysics.MassAPI.Apply(support_collision_prim)
    support_mass.CreateMassAttr().Set(float(
        object_config.get('support_mass_kg', 0.1)))
    support_physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(
        support_collision_prim)
    support_physx_body.CreateDisableGravityAttr().Set(True)

    support_center = tuple(
        float(value) for value in object_config['support_center_m'])
    support_anchor = UsdGeom.Xform.Define(stage, SUPPORT_ANCHOR_PATH)
    support_anchor_xform = UsdGeom.Xformable(support_anchor.GetPrim())
    support_anchor_xform.AddTranslateOp().Set(Gf.Vec3d(*support_center))
    anchor_body = UsdPhysics.RigidBodyAPI.Apply(support_anchor.GetPrim())
    anchor_body.CreateRigidBodyEnabledAttr().Set(True)
    anchor_body.CreateKinematicEnabledAttr().Set(True)
    anchor_physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(
        support_anchor.GetPrim())
    anchor_physx_body.CreateDisableGravityAttr().Set(True)

    # A prismatic joint gives the acquisition support one physical degree of
    # freedom. The drive moves it vertically while the joint prevents the
    # lateral drift and rotation that a free rigid-body velocity servo permits.
    support_joint = UsdPhysics.PrismaticJoint.Define(
        stage, SUPPORT_JOINT_PATH)
    support_joint.CreateBody0Rel().SetTargets([SUPPORT_ANCHOR_PATH])
    support_joint.CreateBody1Rel().SetTargets([SUPPORT_PATH])
    support_joint.CreateAxisAttr('Z')
    support_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    support_joint.CreateLocalRot0Attr().Set(
        Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    support_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    support_joint.CreateLocalRot1Attr().Set(
        Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    support_joint.CreateLowerLimitAttr(-SUPPORT_DRIVE_LIMIT_M)
    support_joint.CreateUpperLimitAttr(SUPPORT_DRIVE_LIMIT_M)
    support_drive = UsdPhysics.DriveAPI.Apply(
        support_joint.GetPrim(), 'linear')
    support_drive.CreateTypeAttr('force')
    support_drive.CreateStiffnessAttr(SUPPORT_DRIVE_STIFFNESS_N_M)
    support_drive.CreateDampingAttr(SUPPORT_DRIVE_DAMPING_N_S_M)
    support_drive.CreateMaxForceAttr(SUPPORT_DRIVE_MAXIMUM_FORCE_N)
    support_drive.CreateTargetPositionAttr(0.0)

    scene_prim = stage.GetPrimAtPath('/PhysicsScene')
    if not scene_prim.IsValid():
        scene_prim = UsdPhysics.Scene.Define(
            stage, '/PhysicsScene').GetPrim()
    scene = UsdPhysics.Scene(scene_prim)
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
    physics = config['physics']
    physx_scene.CreateEnableCCDAttr().Set(bool(physics['enable_ccd']))
    physx_scene.CreateEnableStabilizationAttr().Set(
        bool(physics['enable_stabilization']))
    physx_scene.CreateSolverTypeAttr().Set(str(physics['solver_type']))
    physx_scene.CreateTimeStepsPerSecondAttr().Set(
        int(round(1.0 / float(physics['physics_dt_s']))))
    return (object_collision_prim, support_collision_prim, support_drive,
            fingertip_collision_paths)


def _set_support_enabled(support_prim, enabled, UsdGeom, UsdPhysics):
    collision = UsdPhysics.CollisionAPI(support_prim)
    collision.GetCollisionEnabledAttr().Set(enabled)
    imageable = UsdGeom.Imageable(support_prim)
    imageable.GetVisibilityAttr().Set(
        UsdGeom.Tokens.inherited if enabled else UsdGeom.Tokens.invisible)


def _set_support_drive_target(support_drive, support_center, target):
    """Set the prismatic carrier target relative to its fixed anchor."""
    if any(abs(float(target[index]) - float(support_center[index])) >
           RESET_CARRIER_TARGET_TOLERANCE_M for index in (0, 1)):
        raise RuntimeError('acquisition support drive cannot move laterally')
    offset = float(target[2]) - float(support_center[2])
    if abs(offset) > SUPPORT_DRIVE_LIMIT_M:
        raise RuntimeError('acquisition support target exceeds drive limits')
    support_drive.GetTargetPositionAttr().Set(offset)


def _position_support(step, support_body, target, set_support_target):
    """Move the disabled carrier to a target through its physical drive."""
    set_support_target(target)
    steps = 0
    error = math.inf
    while steps < SUPPORT_POSITIONING_MAXIMUM_STEPS:
        step()
        steps += 1
        actual = _object_state(support_body)['position_m']
        error = math.sqrt(sum(
            (actual[index] - float(target[index])) ** 2
            for index in range(3)))
        if error <= RESET_CARRIER_TARGET_TOLERANCE_M:
            break
    if error > RESET_CARRIER_TARGET_TOLERANCE_M:
        raise RuntimeError(
            'acquisition support drive did not reach its target: '
            f'error={error:.6f} m')
    return steps


def _support_catch_position(object_state, object_config):
    position = object_state['position_m']
    w, x, y, z = object_state['orientation_wxyz']
    half_x, half_y, half_z = (
        float(value) / 2.0 for value in object_config['size_m'])
    vertical_half_extent = (
        abs(2.0 * (x * z - w * y)) * half_x +
        abs(2.0 * (y * z + w * x)) * half_y +
        abs(1.0 - 2.0 * (x * x + y * y)) * half_z)
    support_half_z = float(object_config['support_size_m'][2]) / 2.0
    return [
        position[0],
        position[1],
        position[2] - vertical_half_extent - support_half_z -
        float(object_config['support_catch_gap_m']),
    ]


def _contact_sample(sensor, object_path: str):
    getter = getattr(sensor, 'get_data', None)
    data = getter() if getter is not None else sensor.get_current_frame()
    valid = 'in_contact' in data and math.isfinite(
        float(data.get('force', 0.0)))
    object_contact = False
    contact_pairs = set()
    object_normals = []
    object_impulses = []
    for contact in data.get('contacts', []):
        body0 = str(contact.get('body0', ''))
        body1 = str(contact.get('body1', ''))
        contact_pairs.add((body0, body1))
        if object_path in body0 or object_path in body1:
            object_contact = True
            object_normals.append([
                float(value) for value in contact.get('normal', [])])
            object_impulses.append([
                float(value) for value in contact.get('impulse', [])])
    return {
        'valid': valid,
        'in_contact': bool(data.get('in_contact', False)),
        'object_contact': object_contact,
        'force_n': float(data.get('force', 0.0)),
        'contact_count': int(data.get('number_of_contacts', 0)),
        'contact_pairs': contact_pairs,
        'object_normals': object_normals,
        'object_impulses': object_impulses,
    }


def _contact_pair_report(contact_pairs):
    ordered = sorted(contact_pairs)
    return {
        'unique_count': len(ordered),
        'pairs': [
            {'body0': body0, 'body1': body1}
            for body0, body1 in ordered[:MAX_RECORDED_CONTACT_PAIRS]
        ],
        'truncated': len(ordered) > MAX_RECORDED_CONTACT_PAIRS,
    }


def _force_report(samples):
    finite = [float(value) for value in samples if math.isfinite(value)]
    if not finite:
        return {'samples': 0, 'minimum_n': None, 'mean_n': None,
                'maximum_n': None, 'final_n': None}
    return {
        'samples': len(finite),
        'minimum_n': min(finite),
        'mean_n': sum(finite) / len(finite),
        'maximum_n': max(finite),
        'final_n': finite[-1],
    }


def _contact_normal_report(samples):
    valid = [sample for sample in samples
             if len(sample) == 3 and _finite(sample)]
    if not valid:
        return {
            'samples': 0,
            'mean_xyz': None,
            'mean_absolute_z': None,
            'maximum_absolute_z': None,
        }
    return {
        'samples': len(valid),
        'mean_xyz': [
            sum(sample[axis] for sample in valid) / len(valid)
            for axis in range(3)
        ],
        'mean_absolute_z': (
            sum(abs(sample[2]) for sample in valid) / len(valid)),
        'maximum_absolute_z': max(abs(sample[2]) for sample in valid),
    }


def _contact_impulse_report(samples, physics_dt: float):
    """Report per-point normal contact impulses in world coordinates."""
    valid = [sample for sample in samples
             if len(sample) == 3 and _finite(sample)]
    if not valid:
        return {
            'samples': 0,
            'mean_force_xyz_n': None,
            'mean_absolute_normal_axis_force_n': None,
            'mean_absolute_vertical_normal_force_component_n': None,
            'mean_vertical_to_normal_component_ratio': None,
        }
    forces = [
        [component / physics_dt for component in sample]
        for sample in valid
    ]
    ratios = [
        abs(force[2]) / abs(force[0])
        for force in forces if abs(force[0]) > 1.0e-9
    ]
    return {
        'samples': len(forces),
        'mean_force_xyz_n': [
            sum(force[axis] for force in forces) / len(forces)
            for axis in range(3)
        ],
        'mean_absolute_normal_axis_force_n': (
            sum(abs(force[0]) for force in forces) / len(forces)),
        'mean_absolute_vertical_normal_force_component_n': (
            sum(abs(force[2]) for force in forces) / len(forces)),
        'mean_vertical_to_normal_component_ratio': (
            sum(ratios) / len(ratios) if ratios else None),
    }


def _object_state(object_prim):
    positions, orientations = object_prim.get_world_poses()
    linear, angular = object_prim.get_velocities()
    return {
        'position_m': _row(positions),
        'orientation_wxyz': _row(orientations),
        'linear_velocity_m_s': _row(linear),
        'angular_velocity_rad_s': _row(angular),
    }


def _lift_reset_fixture(step, object_prim, support_body,
                        support_start, support_nominal_end,
                        object_position_end, object_orientation_end,
                        reset_steps, set_support_target):
    start = _object_state(object_prim)
    previous_position = start['position_m']
    maximum_object_step = 0.0

    # Preserve the actual released object-to-carrier offset. Using the
    # configured acquisition-support height here assumes a perfect release
    # pose and can stop the carrier short after a real gravity hold.
    required_object_lift = max(
        0.0, object_position_end[2] - start['position_m'][2])
    support_target = [
        float(support_nominal_end[0]),
        float(support_nominal_end[1]),
        float(support_start[2]) + required_object_lift,
    ]

    def move_and_measure(support_position):
        nonlocal maximum_object_step, previous_position
        set_support_target(support_position)
        step()
        current_position = _object_state(object_prim)['position_m']
        maximum_object_step = max(
            maximum_object_step,
            math.sqrt(sum(
                (current_position[axis] - previous_position[axis]) ** 2
                for axis in range(3))))
        previous_position = current_position

    nominal_step = max(
        required_object_lift / reset_steps,
        RESET_CARRIER_TARGET_TOLERANCE_M / reset_steps)
    for index in range(1, reset_steps + 1):
        fraction = index / reset_steps
        support_position = [
            (1.0 - fraction) * a + fraction * b
            for a, b in zip(support_start, support_target)
        ]
        move_and_measure(support_position)

    # A dynamic object can settle into the carrier or lose part of its lift
    # through compliant contacts. Continue the physical lift under feedback;
    # never teleport the object and never begin another grasp if the bounded
    # carrier overrun cannot restore it.
    maximum_extension_steps = math.ceil(
        RESET_MAXIMUM_CARRIER_OVERRUN_M / nominal_step)
    extension_steps = 0
    maximum_support_z = support_target[2] + RESET_MAXIMUM_CARRIER_OVERRUN_M
    while extension_steps < maximum_extension_steps:
        current = _object_state(object_prim)
        if (object_position_end[2] - current['position_m'][2] <=
                RESET_CARRIER_TARGET_TOLERANCE_M):
            break
        current_support = _object_state(support_body)['position_m']
        if current_support[2] >= maximum_support_z:
            break
        support_position = list(current_support)
        support_position[2] = min(
            maximum_support_z, support_position[2] + nominal_step)
        move_and_measure(support_position)
        extension_steps += 1

    end = _object_state(object_prim)
    support_end = _object_state(support_body)
    return {
        'mode': 'feedback-bounded-prismatic-carrier-drive-lift',
        'steps': reset_steps + extension_steps,
        'nominal_steps': reset_steps,
        'extension_steps': extension_steps,
        'start_position_m': start['position_m'],
        'end_position_m': end['position_m'],
        'end_orientation_wxyz': end['orientation_wxyz'],
        'end_linear_velocity_m_s': end['linear_velocity_m_s'],
        'maximum_object_step_m': maximum_object_step,
        'target_position_m': object_position_end,
        'target_orientation_wxyz': object_orientation_end,
        'support_start_position_m': support_start,
        'support_nominal_target_position_m': support_target,
        'support_end_position_m': support_end['position_m'],
        'maximum_carrier_overrun_m': RESET_MAXIMUM_CARRIER_OVERRUN_M,
    }


def _run_trial(app, SimulationManager, articulation, driven_index,
               object_prim, support_collision_prim, support_body,
               support_drive, sensors, object_config, trial_index,
               settle_steps: int, grasp_steps: int, hold_steps: int,
               contact_debounce_steps: int, expected_contact_target: float,
               reference_finger_static_friction: float,
               friction_combine_mode: str, physics_dt: float,
               reset_steps: int,
               UsdGeom, UsdPhysics):
    grasp_target = float(object_config['grasp_target_m'])
    release_target = float(object_config['release_target_m'])
    initial_position = [float(value) for value in object_config['center_m']]
    initial_orientation = [1.0, 0.0, 0.0, 0.0]

    support_center = [
        float(value) for value in object_config['support_center_m']]
    support_target = list(support_center)

    def set_support_target(position):
        support_target[:] = [float(value) for value in position]
        _set_support_drive_target(
            support_drive, support_center, support_target)

    def step(target=release_target):
        articulation.set_dof_position_targets(
            target, dof_indices=[driven_index])
        SimulationManager.step()
        app.update()

    if trial_index == 1:
        _set_support_enabled(
            support_collision_prim, False, UsdGeom, UsdPhysics)
        set_support_target(support_center)
        object_prim.set_enabled_gravities([False])
        object_prim.set_world_poses(
            positions=initial_position, orientations=initial_orientation)
        object_prim.set_velocities(
            linear_velocities=[0.0, 0.0, 0.0],
            angular_velocities=[0.0, 0.0, 0.0])
        _position_support(
            step, support_body, support_center, set_support_target)
        _set_support_enabled(
            support_collision_prim, True, UsdGeom, UsdPhysics)
        object_prim.set_enabled_gravities([True])
        pre_trial_reset = {
            'mode': 'initial-placement',
            'steps': 0,
            'start_position_m': initial_position,
            'end_position_m': initial_position,
            'end_orientation_wxyz': initial_orientation,
            'end_linear_velocity_m_s': [0.0, 0.0, 0.0],
            'maximum_object_step_m': 0.0,
            'target_position_m': initial_position,
            'target_orientation_wxyz': initial_orientation,
        }
    else:
        object_prim.set_enabled_gravities([True])
        _set_support_enabled(
            support_collision_prim, True, UsdGeom, UsdPhysics)
        support_start = _object_state(support_body)['position_m']
        set_support_target(support_start)
        pre_trial_reset = _lift_reset_fixture(
            step, object_prim, support_body,
            support_start, support_center,
            initial_position, initial_orientation, reset_steps,
            set_support_target)
    for _ in range(settle_steps):
        step(release_target)

    reset_end = _object_state(object_prim)
    pre_trial_reset['settled_object_position_m'] = reset_end['position_m']
    pre_trial_reset['settled_support_position_m'] = _object_state(
        support_body)['position_m']
    reset_position_error = math.sqrt(sum(
        (actual - expected) ** 2
        for actual, expected in zip(
            reset_end['position_m'], initial_position)))
    reset_rotation_error = _rotation_delta(
        reset_end['orientation_wxyz'], initial_orientation)
    reset_ready = (
        reset_position_error <= RESET_POSITION_TOLERANCE_M and
        reset_rotation_error <= RESET_ROTATION_TOLERANCE_RAD and
        pre_trial_reset['maximum_object_step_m'] <=
        RESET_MAXIMUM_OBJECT_STEP_M)
    if not reset_ready:
        return {
            'trial': trial_index,
            'status': 'failed',
            'failure_phase': 'pre-trial-reset',
            'checks': {
                'inter_trial_reset_ready': False,
            },
            'measurements': {
                'inter_trial_reset': pre_trial_reset,
                'inter_trial_reset_position_error_m': reset_position_error,
                'inter_trial_reset_rotation_error_rad': reset_rotation_error,
            },
        }

    consecutive_bilateral = 0
    contact_step = None
    hold_target = release_target
    contact_target = None
    peak_force = 0.0
    all_sensors_valid = True
    observed_contact_pairs = [set(), set()]
    preload_force_samples = [[], []]
    hold_force_samples = [[], []]
    hold_normal_samples = [[], []]
    hold_impulse_samples = [[], []]

    def sample_contacts():
        samples = [_contact_sample(sensor, OBJECT_PATH)
                   for sensor in sensors]
        for observed, sample in zip(observed_contact_pairs, samples):
            observed.update(sample['contact_pairs'])
        return samples

    for index in range(grasp_steps):
        slow_approach_start = (
            expected_contact_target +
            float(object_config['contact_approach_margin_m']))
        closing_step = (
            float(object_config['approach_closing_step_m'])
            if hold_target > slow_approach_start
            else float(object_config['contact_closing_step_m']))
        hold_target = max(grasp_target, hold_target - closing_step)
        step(hold_target)
        samples = sample_contacts()
        all_sensors_valid = all_sensors_valid and all(
            sample['valid'] for sample in samples)
        peak_force = max(
            peak_force, *(sample['force_n'] for sample in samples))
        bilateral = all(
            sample['in_contact'] and sample['object_contact']
            for sample in samples)
        consecutive_bilateral = (
            consecutive_bilateral + 1 if bilateral else 0)
        if consecutive_bilateral >= contact_debounce_steps:
            contact_step = index + 1
            contact_target = hold_target
            break

    preload_steps = 0
    preload_bilateral_steps = 0
    if contact_step is not None:
        preload_step = float(object_config['preload_closing_step_m'])
        while hold_target > grasp_target:
            hold_target = max(grasp_target, hold_target - preload_step)
            step(hold_target)
            preload_steps += 1
            samples = sample_contacts()
            all_sensors_valid = all_sensors_valid and all(
                sample['valid'] for sample in samples)
            peak_force = max(
                peak_force, *(sample['force_n'] for sample in samples))
            for force_samples, sample in zip(
                    preload_force_samples, samples):
                force_samples.append(sample['force_n'])
            if all(sample['in_contact'] and sample['object_contact']
                   for sample in samples):
                preload_bilateral_steps += 1

    grasp_position = _scalar(
        articulation.get_dof_positions(), 0, driven_index)
    hold_start = _object_state(object_prim)
    if contact_step is not None:
        _set_support_enabled(
            support_collision_prim, False, UsdGeom, UsdPhysics)
    bilateral_steps = 0
    maximum_drop = 0.0
    maximum_lateral_drift = 0.0
    maximum_rotation = 0.0
    maximum_speed = 0.0
    maximum_angular_speed = 0.0
    maximum_displacement = 0.0
    state_finite = True

    for _ in range(hold_steps):
        step(hold_target)
        samples = sample_contacts()
        all_sensors_valid = all_sensors_valid and all(
            sample['valid'] for sample in samples)
        peak_force = max(
            peak_force, *(sample['force_n'] for sample in samples))
        for force_samples, sample in zip(hold_force_samples, samples):
            force_samples.append(sample['force_n'])
        for normal_samples, sample in zip(hold_normal_samples, samples):
            normal_samples.extend(sample['object_normals'])
        for impulse_samples, sample in zip(hold_impulse_samples, samples):
            impulse_samples.extend(sample['object_impulses'])
        if all(sample['in_contact'] and sample['object_contact']
               for sample in samples):
            bilateral_steps += 1

        state = _object_state(object_prim)
        position = state['position_m']
        linear = state['linear_velocity_m_s']
        angular = state['angular_velocity_rad_s']
        orientation = state['orientation_wxyz']
        state_finite = state_finite and _finite(
            position + linear + angular + orientation)
        maximum_drop = max(
            maximum_drop, hold_start['position_m'][2] - position[2])
        maximum_lateral_drift = max(
            maximum_lateral_drift,
            math.hypot(
                position[0] - hold_start['position_m'][0],
                position[1] - hold_start['position_m'][1]))
        maximum_rotation = max(
            maximum_rotation,
            _rotation_delta(hold_start['orientation_wxyz'], orientation))
        maximum_speed = max(
            maximum_speed, math.sqrt(sum(value * value for value in linear)))
        maximum_angular_speed = max(
            maximum_angular_speed,
            math.sqrt(sum(value * value for value in angular)))
        maximum_displacement = max(
            maximum_displacement,
            math.sqrt(sum(
                (position[i] - initial_position[i]) ** 2
                for i in range(3))))

    hold_end = _object_state(object_prim)
    support_catch_position = _support_catch_position(
        hold_end, object_config)
    support_catch_position[0:2] = support_center[0:2]
    support_positioning_steps = _position_support(
        step, support_body, support_catch_position, set_support_target)
    _set_support_enabled(
        support_collision_prim, True, UsdGeom, UsdPhysics)
    release_contact_step = None
    release_timeout = int(object_config['release_contact_timeout_steps'])
    for index in range(release_timeout):
        step(release_target)
        samples = sample_contacts()
        if not any(sample['in_contact'] and sample['object_contact']
                   for sample in samples):
            release_contact_step = index + 1
            break
    for _ in range(max(0, settle_steps - (release_contact_step or 0))):
        step(release_target)
    release_position = _scalar(
        articulation.get_dof_positions(), 0, driven_index)
    contact_fraction = bilateral_steps / max(1, hold_steps)
    preload_contact_fraction = (
        preload_bilateral_steps / max(1, preload_steps))
    blocking_error = abs(grasp_position - grasp_target)
    effective_static_friction = _combined_friction(
        float(object_config['material']['static_friction']),
        reference_finger_static_friction, friction_combine_mode)
    estimated_minimum_normal_force = (
        float(object_config['mass_kg']) * 9.81 /
        (2.0 * effective_static_friction))
    hold_normal_reports = [
        _contact_normal_report(samples) for samples in hold_normal_samples]
    hold_impulse_reports = [
        _contact_impulse_report(samples, physics_dt)
        for samples in hold_impulse_samples
    ]
    maximum_mean_normal_vertical = float(object_config.get(
        'maximum_mean_hold_contact_normal_vertical_component', 1.0))
    checks = {
        'bilateral_object_contact_acquired': contact_step is not None,
        'bilateral_contact_maintained_during_preload': (
            contact_step is not None and
            preload_contact_fraction >= float(
                object_config['minimum_preload_contact_fraction'])),
        'contact_sensors_valid': all_sensors_valid,
        'object_blocks_target_before_endpoint': blocking_error >= float(
            object_config['minimum_grasp_target_error_m']),
        'bilateral_contact_maintained': contact_fraction >= float(
            object_config['minimum_bilateral_contact_fraction']),
        'contact_force_finite_and_bounded': (
            math.isfinite(peak_force) and
            peak_force >= float(object_config['contact_force_min_n']) and
            peak_force <= float(object_config['contact_force_max_n'])),
        'hold_drop_bounded': maximum_drop <= float(
            object_config['maximum_hold_drop_m']),
        'hold_lateral_drift_bounded': maximum_lateral_drift <= float(
            object_config['maximum_hold_lateral_drift_m']),
        'hold_rotation_bounded': maximum_rotation <= float(
            object_config['maximum_hold_rotation_rad']),
        'hold_contact_normals_match_vertical_pinch_surfaces': all(
            report['samples'] > 0 and
            report['mean_absolute_z'] <= maximum_mean_normal_vertical
            for report in hold_normal_reports),
        'object_state_finite': state_finite,
        'object_speed_bounded': maximum_speed <= float(
            object_config['maximum_object_speed_m_s']),
        'object_angular_speed_bounded': maximum_angular_speed <= float(
            object_config['maximum_object_angular_speed_rad_s']),
        'object_displacement_bounded': maximum_displacement <= float(
            object_config['maximum_object_displacement_m']),
        'release_loses_contact': release_contact_step is not None,
        'release_target_reached': (
            abs(release_position - release_target) <= 0.001),
        'inter_trial_reset_ready': reset_ready,
    }
    return {
        'trial': trial_index,
        'status': 'passed' if all(checks.values()) else 'failed',
        'checks': checks,
        'measurements': {
            'contact_acquired_step': contact_step,
            'contact_target_m': contact_target,
            'preload_steps': preload_steps,
            'preload_bilateral_contact_fraction': preload_contact_fraction,
            'hold_target_m': hold_target,
            'grasp_position_m': grasp_position,
            'grasp_target_error_m': blocking_error,
            'peak_contact_force_n': peak_force,
            'preload_contact_force': {
                'left_fingertip': _force_report(preload_force_samples[0]),
                'right_fingertip': _force_report(preload_force_samples[1]),
            },
            'hold_contact_force': {
                'left_fingertip': _force_report(hold_force_samples[0]),
                'right_fingertip': _force_report(hold_force_samples[1]),
            },
            'hold_contact_normal': {
                'left_fingertip': hold_normal_reports[0],
                'right_fingertip': hold_normal_reports[1],
            },
            'hold_contact_impulse': {
                'left_fingertip': hold_impulse_reports[0],
                'right_fingertip': hold_impulse_reports[1],
            },
            'maximum_mean_hold_contact_normal_vertical_component': (
                maximum_mean_normal_vertical),
            'estimated_effective_static_friction': (
                effective_static_friction),
            'friction_combine_mode': friction_combine_mode,
            'estimated_minimum_normal_force_per_finger_n': (
                estimated_minimum_normal_force),
            'bilateral_contact_fraction': contact_fraction,
            'maximum_hold_drop_m': maximum_drop,
            'maximum_hold_lateral_drift_m': maximum_lateral_drift,
            'maximum_hold_rotation_rad': maximum_rotation,
            'maximum_object_speed_m_s': maximum_speed,
            'maximum_object_angular_speed_rad_s': maximum_angular_speed,
            'maximum_object_displacement_m': maximum_displacement,
            'release_contact_step': release_contact_step,
            'support_positioning_steps': support_positioning_steps,
            'release_position_m': release_position,
            'inter_trial_reset': pre_trial_reset,
            'inter_trial_reset_position_error_m': reset_position_error,
            'inter_trial_reset_rotation_error_rad': reset_rotation_error,
            'observed_contact_pairs': {
                'left_fingertip': _contact_pair_report(
                    observed_contact_pairs[0]),
                'right_fingertip': _contact_pair_report(
                    observed_contact_pairs[1]),
            },
            'hold_start': hold_start,
            'hold_end': hold_end,
        },
    }


def main() -> int:
    """Load an asset, author its scene, and run dynamic contact trials."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--model', choices=SUPPORTED_MODELS, required=True,
        help='Gripper model; must match the selected asset and config')
    parser.add_argument('--asset', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--object-id', default='external_pinch_block_v1')
    parser.add_argument('--trials', type=int)
    parser.add_argument('--settle-steps', type=int, default=240)
    parser.add_argument('--grasp-steps', type=int, default=480)
    parser.add_argument('--hold-steps', type=int, default=240)
    parser.add_argument('--contact-debounce-steps', type=int, default=12)
    parser.add_argument('--reset-lift-steps', type=int,
                        default=DEFAULT_RESET_LIFT_STEPS)
    parser.add_argument('--gui', action='store_true')
    args, _ = parser.parse_known_args()

    contract = load_contract(args.model)
    args.asset = args.asset or default_asset(args.model)
    args.config = args.config or _default_config(args.model)
    root_prim = f'/onrobot_{args.model}'
    root_path = articulation_path(contract)
    actuated_joint = contract['usd']['actuated_joint']
    fingertip_paths = (
        f'{root_prim}/Geometry/left_fingertip_link',
        f'{root_prim}/Geometry/right_fingertip_link',
    )

    config_bytes = args.config.read_bytes() if args.config.is_file() else b''
    report = {
        'schema_version': 1,
        'harness_revision': HARNESS_REVISION,
        'model': args.model,
        'asset': str(args.asset.resolve()),
        'config': str(args.config.resolve()),
        'config_sha256': hashlib.sha256(config_bytes).hexdigest(),
        'physics_backend': 'physx',
        'fidelity_target': 'dynamic-contact',
        'status': 'starting',
        'completed_trials': 0,
        'tests': [],
    }
    _write_report(args.output, report)
    print(
        'OnRobot dynamic-contact harness '
        f'revision={HARNESS_REVISION} model={args.model} '
        f'config_sha256={report["config_sha256"]}',
        flush=True)
    app = None
    try:
        launch_config = {
            'headless': not args.gui,
            'width': 1280,
            'height': 720,
        }
        if args.gui:
            launch_config['renderer'] = 'RaytracedLighting'
        app = SimulationApp(launch_config)

        import omni.usd
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.core.version import get_version
        from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

        legacy_contact_sensor = False
        try:
            from isaacsim.sensors.experimental.physics import (
                Contact, ContactSensor)

            def create_contact_sensor(path):
                return ContactSensor(Contact.create(
                    path, min_threshold=0.0,
                    max_threshold=1000.0, radius=-1.0))
        except ModuleNotFoundError:
            from isaacsim.sensors.physics import ContactSensor
            legacy_contact_sensor = True

            def create_contact_sensor(path):
                return ContactSensor(
                    prim_path=path, min_threshold=0.0,
                    max_threshold=1000.0, radius=-1.0)

        set_camera_view = None
        if args.gui:
            try:
                from isaacsim.core.rendering_manager import ViewportManager
                set_camera_view = ViewportManager.set_camera_view
            except ModuleNotFoundError:
                from isaacsim.core.utils.viewports import (
                    set_camera_view as set_legacy_camera_view)

                def set_camera_view(camera_prim_path, eye, target):
                    set_legacy_camera_view(
                        eye=eye, target=target,
                        camera_prim_path=camera_prim_path)

        report['isaac_sim_version'] = get_version()[0]
        if not args.asset.is_file():
            raise RuntimeError(f'asset does not exist: {args.asset}')
        if not args.config.is_file():
            raise RuntimeError(f'config does not exist: {args.config}')
        config = json.loads(config_bytes.decode('utf-8'))
        if (config.get('model') != args.model or
                config.get('asset_revision') !=
                contract.get('asset_revision') or
                config.get('fidelity_target') != 'dynamic-contact'):
            raise RuntimeError(
                'dynamic contact configuration identity is invalid')
        object_config = next(
            (item for item in config['objects']
             if item['id'] == args.object_id), None)
        if object_config is None:
            raise RuntimeError(
                f'object {args.object_id!r} is absent from the dynamic '
                'contact config')
        if object_config.get('grasp_kind') != 'external-pinch':
            raise RuntimeError(
                'outward reference fingers require an external-pinch object')
        finger_combine_mode = str(config['reference_fingers'].get(
            'friction_combine_mode', 'average'))
        object_combine_mode = str(object_config['material'].get(
            'friction_combine_mode', 'average'))
        if finger_combine_mode != object_combine_mode:
            raise RuntimeError(
                'reference finger and object friction combine modes differ')
        report['scene_geometry'] = _validate_object_placement(
            config, object_config)
        trials = int(args.trials or config['physics']['trials'])
        if (trials < 1 or args.settle_steps < 1 or args.grasp_steps < 1 or
                args.hold_steps < 1 or args.contact_debounce_steps < 1 or
                args.reset_lift_steps < 1):
            raise RuntimeError('trial and step counts must be positive')
        approach_step = float(object_config['approach_closing_step_m'])
        contact_step = float(object_config['contact_closing_step_m'])
        preload_step = float(object_config['preload_closing_step_m'])
        if not (0.0 < contact_step <= approach_step <= 0.001):
            raise RuntimeError('closing approach steps are invalid')
        if not (0.0 < preload_step <= 0.001):
            raise RuntimeError('preload closing step is invalid')
        expected_contact_target = (
            float(object_config['size_m'][0]) / 2.0 -
            float(config['reference_geometry_m'][
                'contact_face_inner_x_at_zero']))
        approach_margin = float(object_config['contact_approach_margin_m'])
        if approach_margin <= 0.0:
            raise RuntimeError('contact approach margin is invalid')
        fast_distance = max(
            0.0,
            float(object_config['release_target_m']) -
            expected_contact_target - approach_margin)
        estimated_approach_steps = (
            math.ceil(fast_distance / approach_step) +
            math.ceil(approach_margin / contact_step) +
            args.contact_debounce_steps)
        if estimated_approach_steps > args.grasp_steps:
            raise RuntimeError('grasp step budget cannot reach the reference '
                               'object')

        if not omni.usd.get_context().open_stage(str(args.asset.resolve())):
            raise RuntimeError(f'Isaac Sim could not open {args.asset}')
        app.update()
        app.update()
        while is_stage_loading():
            app.update()
        stage = omni.usd.get_context().get_stage()
        (object_collision_prim, support_collision_prim, support_drive,
         fingertip_collision_paths) = _author_scenario(
            stage, config, object_config, fingertip_paths,
            Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade)

        if args.gui:
            set_camera_view(
                '/OmniverseKit_Persp',
                eye=[0.28, -0.30, 0.23],
                target=[0.0, 0.0, 0.115])
            for _ in range(5):
                app.update()
            time.sleep(GUI_INSPECTION_DELAY_S)

        left_sensor = create_contact_sensor(
            f'{fingertip_paths[0]}/onrobot_dynamic_contact_sensor')
        right_sensor = create_contact_sensor(
            f'{fingertip_paths[1]}/onrobot_dynamic_contact_sensor')
        for sensor in (left_sensor, right_sensor):
            sensor.add_raw_contact_data_to_frame()

        physics_dt = float(config['physics']['physics_dt_s'])
        setup_simulation(
            SimulationManager, dt=physics_dt, device='cpu')
        articulation = Articulation(root_path)
        object_prim = RigidPrim(OBJECT_PATH)
        support_body = RigidPrim(SUPPORT_PATH)
        play()
        SimulationManager.step()
        app.update()
        if legacy_contact_sensor:
            for sensor in (left_sensor, right_sensor):
                sensor.initialize()
        if actuated_joint not in articulation.dof_names:
            raise RuntimeError(
                f'{actuated_joint} missing from runtime DOFs: '
                f'{articulation.dof_names}')
        driven_index = articulation.dof_names.index(actuated_joint)
        release_target = float(object_config['release_target_m'])
        articulation.set_dof_positions(
            [release_target], dof_indices=[driven_index])
        articulation.set_dof_velocities(
            [0.0], dof_indices=[driven_index])
        articulation.set_dof_position_targets(
            release_target, dof_indices=[driven_index])
        for _ in range(7):
            SimulationManager.step()
            app.update()
        collision_api = UsdPhysics.CollisionAPI(object_collision_prim)
        collision_api.GetCollisionEnabledAttr().Set(True)
        report['articulation_path'] = root_path
        report['dof_names'] = list(articulation.dof_names)
        report['object_id'] = object_config['id']
        report['reference_fingers'] = config['reference_fingers']
        report['fingertip_collision_material_bindings'] = {
            'left': fingertip_collision_paths[0],
            'right': fingertip_collision_paths[1],
        }
        report['physics'] = config['physics']
        report['acquisition_support_drive'] = {
            'type': 'prismatic-position-drive',
            'stiffness_n_m': SUPPORT_DRIVE_STIFFNESS_N_M,
            'damping_n_s_m': SUPPORT_DRIVE_DAMPING_N_S_M,
            'maximum_force_n': SUPPORT_DRIVE_MAXIMUM_FORCE_N,
            'travel_limit_m': SUPPORT_DRIVE_LIMIT_M,
        }
        report['trial_count'] = trials
        report['status'] = 'running'
        _write_report(args.output, report)

        joint = UsdPhysics.PrismaticJoint.Get(
            stage, f'{root_prim}/Physics/{actuated_joint}')
        drive = UsdPhysics.DriveAPI.Get(joint.GetPrim(), 'linear')
        report['drive'] = {
            'stiffness': float(drive.GetStiffnessAttr().Get()),
            'damping': float(drive.GetDampingAttr().Get()),
            'maximum_force_n': float(drive.GetMaxForceAttr().Get()),
        }

        started = time.monotonic()
        for trial_index in range(1, trials + 1):
            trial = _run_trial(
                app, SimulationManager, articulation, driven_index,
                object_prim, support_collision_prim, support_body,
                support_drive, (left_sensor, right_sensor), object_config,
                trial_index, args.settle_steps, args.grasp_steps,
                args.hold_steps, args.contact_debounce_steps,
                expected_contact_target,
                float(config['reference_fingers']['static_friction']),
                finger_combine_mode, physics_dt, args.reset_lift_steps,
                UsdGeom, UsdPhysics)
            report['tests'].append(trial)
            report['completed_trials'] = len(report['tests'])
            _write_report(args.output, report)
            if trial.get('failure_phase') == 'pre-trial-reset':
                break
        report['elapsed_s'] = time.monotonic() - started
        report['status'] = (
            'passed' if all(item['status'] == 'passed'
                            for item in report['tests']) else 'failed')
    except KeyboardInterrupt:
        report['status'] = 'canceled'
        report['error'] = 'KeyboardInterrupt: canceled by operator'
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        _write_report(args.output, report)
        print(json.dumps(report, indent=2, sort_keys=True), flush=True)
        exit_code = 0 if report['status'] == 'passed' else 1
        if app is not None:
            try:
                stop()
            except Exception:
                pass
            close_app(app, exit_code)

    return exit_code


if __name__ == '__main__':
    sys.exit(main())

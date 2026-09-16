#!/usr/bin/env python3
"""Author the release-candidate 2FG14 Asset Structure 3.0 USD."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

MODEL = '2fg14'
ASSET_STRUCTURE_VERSION = '3.0'
ROBOT_PRIM = '/onrobot_2fg14'
ARTICULATION_ROOT = f'{ROBOT_PRIM}/Physics/root_joint'
ACTUATED_JOINT = f'{ROBOT_PRIM}/Physics/finger_stroke'
FOLLOWER_JOINT = f'{ROBOT_PRIM}/Physics/left_finger_base_joint'
FINGERTIP_JOINTS = {
    'right': f'{ROBOT_PRIM}/Physics/right_fingertip_joint',
    'left': f'{ROBOT_PRIM}/Physics/left_fingertip_joint',
}
LOWER_LIMIT_M = 0.0
UPPER_LIMIT_M = 0.025
MAXIMUM_DRIVE_FORCE_N = 280.0
PHYSX_SINGLE_DRIVE_FORCE_N = 560.0
MAXIMUM_VELOCITY_M_S = 0.45
PROVISIONAL_STIFFNESS = 40000.0
# Joint-space conditioning, not measured rotor inertia or firmware loop gains.
# The 2FG7 1 kg assumption is scaled by (16 / (64 / 9))**2; damping is
# 2 * sqrt(stiffness * reflected inertia), neglecting the smaller link mass.
PROVISIONAL_ARMATURE_KG = 5.0625
PROVISIONAL_DAMPING = 900.0
FINGERTIP_FRICTION = 0.6
FINGERTIP_DYNAMIC_FRICTION = 0.5
ASSET_STRUCTURE_REQUIRED_LAYERS = (
    'payloads/base.usda',
    'payloads/geometries.usd',
    'payloads/instances.usda',
    'payloads/materials.usda',
    'payloads/robot.usda',
    'payloads/Physics/physics.usda',
    'payloads/Physics/physx.usda',
    'payloads/Physics/mujoco.usda',
)
FAMILY_JOINTS = (
    'root_joint',
    'finger_stroke',
    'left_finger_base_joint',
    'right_fingertip_joint',
    'left_fingertip_joint',
)
FAMILY_LINKS = (
    'base_link',
    'right_finger_base_link',
    'right_fingertip_link',
    'left_finger_base_link',
    'left_fingertip_link',
)
FAMILY_FILTERED_PAIR_TARGETS = {
    'base_link': (
        'right_finger_base_link',
        'right_fingertip_link',
        'left_finger_base_link',
        'left_fingertip_link',
    ),
    'right_finger_base_link': (
        'right_fingertip_link',
        'left_finger_base_link',
        'left_fingertip_link',
    ),
    'right_fingertip_link': ('left_finger_base_link',),
    'left_finger_base_link': ('left_fingertip_link',),
}
# TGS supports at most four velocity iterations without changing their
# interpretation. Preserve the previous aggregate budget in position solves.
SOLVER_POSITION_ITERATIONS = 68
SOLVER_VELOCITY_ITERATIONS = 4
COLLISION_APPROXIMATION = 'convexDecomposition'
REFERENCE_FINGERTIP_COLLISION_BOXES = {
    'upper_contact': {
        'center_m': (0.029435, 0.0, 0.0125),
        'size_m': (0.0453, 0.045, 0.006),
    },
    'lower_finger': {
        'center_m': (0.00335, 0.0, 0.002403),
        'size_m': (0.0067, 0.045, 0.0258),
    },
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _expand_xacro(xacro: Path, output: Path) -> None:
    command = [
        shutil.which('xacro') or 'xacro',
        str(xacro),
        'include_task_coordinate:=false',
        'include_ros2_control:=false',
        'fingertip_orientation:=outwards',
        'use_standard_fingertips:=true',
    ]
    result = subprocess.run(
        command, check=True, capture_output=True, text=True)
    output.write_text(result.stdout, encoding='utf-8')


def _remove_frame_only_mount_links(path: Path) -> None:
    """Keep ROS attachment frames out of the physical import articulation."""
    tree = ET.parse(path)
    robot = tree.getroot()
    removed = []
    for side in ('right', 'left'):
        jaw_name = f'{side}_finger_base_link'
        mount_name = f'{side}_finger_mount'
        mount_joint_name = f'{side}_finger_mount_joint'
        fingertip_joint_name = f'{side}_fingertip_joint'

        mount_link = next(
            (link for link in robot.findall('link')
             if link.get('name') == mount_name), None)
        mount_joint = next(
            (joint for joint in robot.findall('joint')
             if joint.get('name') == mount_joint_name), None)
        fingertip_joint = next(
            (joint for joint in robot.findall('joint')
             if joint.get('name') == fingertip_joint_name), None)
        if (mount_link is None or mount_joint is None or
                fingertip_joint is None):
            raise RuntimeError(
                f'cannot normalize {side} attachment frame: required URDF '
                'link or joint is missing')
        parent = mount_joint.find('parent')
        child = mount_joint.find('child')
        fingertip_parent = fingertip_joint.find('parent')
        if (mount_joint.get('type') != 'fixed' or parent is None or
                parent.get('link') != jaw_name or child is None or
                child.get('link') != mount_name or
                fingertip_parent is None or
                fingertip_parent.get('link') != mount_name):
            raise RuntimeError(
                f'{side} attachment frame no longer matches the expected '
                'fixed, identity parent chain')
        origin = mount_joint.find('origin')
        if origin is not None:
            xyz = [float(value) for value in
                   origin.get('xyz', '0 0 0').split()]
            rpy = [float(value) for value in
                   origin.get('rpy', '0 0 0').split()]
            if any(abs(value) > 1e-12 for value in xyz + rpy):
                raise RuntimeError(
                    f'{side} attachment frame has a non-identity transform; '
                    'authoring must preserve it explicitly')
        fingertip_parent.set('link', jaw_name)
        robot.remove(mount_joint)
        robot.remove(mount_link)
        removed.extend((mount_name, mount_joint_name))

    tree.write(path, encoding='unicode', xml_declaration=True)
    if len(removed) != 4:
        raise RuntimeError('failed to normalize both attachment frames')


def _validate_import_urdf(path: Path) -> None:
    robot = ET.parse(path).getroot()
    joint_names = {joint.attrib['name'] for joint in robot.findall('joint')}
    link_names = {link.attrib['name'] for link in robot.findall('link')}
    required_joints = {'finger_stroke', 'left_finger_base_joint'}
    required_links = {
        'base_link', 'right_finger_base_link', 'left_finger_base_link',
        'right_fingertip_link', 'left_fingertip_link',
    }
    if not required_joints.issubset(joint_names):
        raise RuntimeError(
            f'import URDF is missing joints: {required_joints - joint_names}')
    if not required_links.issubset(link_names):
        raise RuntimeError(
            f'import URDF is missing links: {required_links - link_names}')
    forbidden = {
        'grip_stroke', 'task_aperture_link',
        'right_finger_mount', 'left_finger_mount',
        'right_finger_mount_joint', 'left_finger_mount_joint',
    }
    if forbidden & (joint_names | link_names):
        raise RuntimeError(
            'simulation-only URDF contains the ROS task-coordinate joint')
    if robot.find('ros2_control') is not None:
        raise RuntimeError('simulation-only URDF contains ros2_control')


def _validate_asset_structure(entrypoint: Path, profile_path: Path) -> dict:
    profile = json.loads(profile_path.read_text(encoding='utf-8'))
    if profile.get('profile_name') != 'Isaac Sim Structure':
        raise RuntimeError(
            f'unexpected Asset Transformer profile: '
            f'{profile.get("profile_name")!r}')
    expected_entrypoint = f'onrobot_{MODEL}.usda'
    if entrypoint.name != expected_entrypoint:
        raise RuntimeError(
            f'Asset Transformer entrypoint must be {expected_entrypoint}, '
            f'got {entrypoint.name}')
    package_root = entrypoint.parent
    missing = [
        relative for relative in ASSET_STRUCTURE_REQUIRED_LAYERS
        if not (package_root / relative).is_file()
    ]
    if missing:
        raise RuntimeError(
            f'Asset Transformer output is missing layers: {missing}')
    entrypoint_bytes = entrypoint.read_bytes()
    if entrypoint_bytes[:5] != b'#usda':
        raise RuntimeError('Asset Transformer entrypoint is not ASCII USDA')
    entrypoint_text = entrypoint_bytes.decode('utf-8')
    required_interface_opinions = (
        '@./payloads/base.usda@',
        'variantSets = "Physics"',
        'string Physics = "physx"',
    )
    missing_opinions = [
        opinion for opinion in required_interface_opinions
        if opinion not in entrypoint_text
    ]
    if missing_opinions:
        raise RuntimeError(
            f'Asset Transformer interface is missing composition opinions: '
            f'{missing_opinions}')
    geometry_layer = package_root / 'payloads/geometries.usd'
    if geometry_layer.read_bytes()[:8] != b'PXR-USDC':
        raise RuntimeError(
            'Asset Transformer geometry layer is not a binary USD crate')
    return {
        'asset_structure_version': ASSET_STRUCTURE_VERSION,
        'profile': profile['profile_name'],
        'profile_version': str(profile.get('version', 'unknown')),
        'entrypoint': entrypoint.name,
        'layers': list(ASSET_STRUCTURE_REQUIRED_LAYERS),
        'geometry_format': 'usdc-crate',
    }


def _validate_semantic_layer_routing(entrypoint: Path) -> dict:
    """Require the same semantic-layer ownership used by the 2FG7 asset."""
    package_root = entrypoint.parent
    layers = {
        'entry': entrypoint.read_text(encoding='utf-8'),
        'robot': (package_root / 'payloads/robot.usda').read_text(
            encoding='utf-8'),
        'physics': (
            package_root / 'payloads/Physics/physics.usda').read_text(
                encoding='utf-8'),
        'physx': (
            package_root / 'payloads/Physics/physx.usda').read_text(
                encoding='utf-8'),
        'instances': (
            package_root / 'payloads/instances.usda').read_text(
                encoding='utf-8'),
    }
    forbidden_entry_opinions = (
        'isaac:physics:robotLinks',
        'isaac:physics:robotJoints',
        'physxArticulation:enabledSelfCollisions',
        'physxArticulation:solverPositionIterationCount',
        'physxMimicJoint:rotX:',
    )
    leaked = [
        opinion for opinion in forbidden_entry_opinions
        if opinion in layers['entry']]
    if leaked:
        raise RuntimeError(
            f'2FG semantic opinions leaked into the interface layer: '
            f'{leaked}')

    required_robot = (
        'prepend rel isaac:physics:robotLinks',
        'prepend rel isaac:physics:robotJoints',
        f'<{ROBOT_PRIM}/Geometry/base_link>',
        f'<{ROBOT_PRIM}/Physics/root_joint>',
    )
    missing_robot = [
        item for item in required_robot if item not in layers['robot']]
    if missing_robot or f'<{ROBOT_PRIM}>' in layers['robot']:
        raise RuntimeError(
            '2FG robot schema layer has an invalid relationship catalog: '
            f'missing={missing_robot}')

    root_at_joint = re.search(
        r'def\s+PhysicsFixedJoint\s+"root_joint"\s*\([^)]*'
        r'PhysicsArticulationRootAPI[^)]*\)',
        layers['physics'], re.DOTALL)
    articulation_root_count = layers['physics'].count(
        'PhysicsArticulationRootAPI')
    if (not root_at_joint or articulation_root_count != 1 or
            not re.search(
                r'drive:linear:physics:maxForce\s*=\s*280(?:\.0+)?\b',
                layers['physics'])):
        raise RuntimeError(
            '2FG physics layer must have one root_joint articulation root '
            'and the 280 N drive limit')
    required_physx = (
        'PhysxArticulationAPI',
        'physxArticulation:enabledSelfCollisions',
        'physxArticulation:solverPositionIterationCount = 68',
        'physxArticulation:solverVelocityIterationCount = 4',
        'drive:linear:physics:maxForce = 560',
        'PhysxMimicJointAPI:rotX',
        'delete apiSchemas = ["PhysicsDriveAPI:linear"]',
    )
    missing_physx = [
        item for item in required_physx if item not in layers['physx']]
    if missing_physx:
        raise RuntimeError(
            f'2FG PhysX layer is missing family policy: {missing_physx}')
    required_material = (
        'physics:staticFriction = 0.6',
        'physics:dynamicFriction = 0.5',
        'physxMaterial:frictionCombineMode = "average"',
    )
    missing_material = [
        item for item in required_material if item not in layers['physics']]
    if missing_material:
        raise RuntimeError(
            f'2FG14 fingertip material is incomplete: {missing_material}')
    if ('physics:approximation = "convexHull"' in layers['instances'] or
            'physics:approximation = "convexDecomposition"' not in
            layers['instances']):
        raise RuntimeError(
            '2FG14 collision meshes must use the decomposed-collision policy '
            'of the 2FG7 family reference')
    return {
        'status': 'matched-2fg7-layer-ownership',
        'robot_schema': 'payloads/robot.usda',
        'physics': 'payloads/Physics/physics.usda',
        'physx': 'payloads/Physics/physx.usda',
        'collision_approximation': COLLISION_APPROXIMATION,
    }


def _normalize_collision_approximations(entrypoint: Path) -> dict:
    """Restore the 2FG7 collision policy after Asset Transformer output."""
    from pxr import Usd, UsdPhysics

    instances_path = entrypoint.parent / 'payloads/instances.usda'
    instances_stage = Usd.Stage.Open(str(instances_path))
    if instances_stage is None:
        raise RuntimeError(
            f'cannot open collision instance layer: {instances_path}')
    collision_paths = []
    fingertip_collision_found = False
    for prim in instances_stage.Traverse():
        if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            continue
        approximation = UsdPhysics.MeshCollisionAPI(
            prim).CreateApproximationAttr()
        approximation.Set(COLLISION_APPROXIMATION)
        path = str(prim.GetPath())
        collision_paths.append(path)
        fingertip_collision_found = (
            fingertip_collision_found or 'fingertip_link' in path)
    if not collision_paths or not fingertip_collision_found:
        raise RuntimeError(
            'Asset Transformer output is missing the expected fingertip '
            'collision mesh')
    instances_stage.GetRootLayer().Save()
    return {
        'approximation': COLLISION_APPROXIMATION,
        'mesh_collision_prims': collision_paths,
        'family_reference': '2fg7',
    }


def _author_reference_fingertip_collisions(entrypoint: Path) -> dict:
    """Use the validated primitive contact shape for standard 2FG14 tips."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    package_root = entrypoint.parent
    instances_path = package_root / 'payloads/instances.usda'
    instances_stage = Usd.Stage.Open(str(instances_path))
    if instances_stage is None:
        raise RuntimeError(
            f'cannot open collision instance layer: {instances_path}')
    disabled_meshes = []
    for prim in instances_stage.Traverse():
        if (prim.HasAPI(UsdPhysics.MeshCollisionAPI) and
                'fingertip_link' in str(prim.GetPath())):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(
                False)
            disabled_meshes.append(str(prim.GetPath()))
    if len(disabled_meshes) != 1:
        raise RuntimeError(
            'expected one shared 2FG14 fingertip collision mesh, got '
            f'{disabled_meshes}')
    instances_stage.GetRootLayer().Save()
    instances_stage = None

    base_path = package_root / 'payloads/base.usda'
    base_stage = Usd.Stage.Open(str(base_path))
    if base_stage is None:
        raise RuntimeError(f'cannot open base layer: {base_path}')
    authored = []
    for side in ('left', 'right'):
        link_path = f'{ROBOT_PRIM}/Geometry/{side}_fingertip_link'
        if not base_stage.GetPrimAtPath(link_path).IsValid():
            raise RuntimeError(
                f'cannot author collision shapes below missing {link_path}')
        for name, shape in REFERENCE_FINGERTIP_COLLISION_BOXES.items():
            path = f'{link_path}/reference_collisions/{name}'
            cube = UsdGeom.Cube.Define(base_stage, path)
            cube.CreateSizeAttr(1.0)
            cube.CreateExtentAttr([
                Gf.Vec3f(-0.5, -0.5, -0.5),
                Gf.Vec3f(0.5, 0.5, 0.5),
            ])
            cube.CreatePurposeAttr().Set(UsdGeom.Tokens.guide)
            xform = UsdGeom.Xformable(cube.GetPrim())
            xform.AddTranslateOp().Set(Gf.Vec3d(*shape['center_m']))
            xform.AddScaleOp().Set(Gf.Vec3d(*shape['size_m']))
            collision = UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            collision.CreateCollisionEnabledAttr().Set(True)
            authored.append(path)
    base_stage.GetRootLayer().Save()
    base_stage = None
    return {
        'source': '2fg14-standard-fingertip-primitive-collision',
        'disabled_mesh_collision_prims': disabled_meshes,
        'authored_collision_prims': authored,
        'shapes': REFERENCE_FINGERTIP_COLLISION_BOXES,
    }


def _strip_generator_documentation(package_root: Path) -> list[str]:
    """Remove transient Asset Transformer provenance from release layers."""
    cleaned = []
    documentation = re.compile(r'\n\s+doc = """.*?"""', re.DOTALL)
    for layer_path in sorted(package_root.rglob('*.usda')):
        text = layer_path.read_text(encoding='utf-8')
        metadata_end = text.find('\n)')
        if metadata_end < 0:
            continue
        metadata, body = text[:metadata_end], text[metadata_end:]
        metadata, substitutions = documentation.subn(
            '', metadata, count=1)
        if substitutions:
            layer_path.write_text(metadata + body, encoding='utf-8')
            cleaned.append(str(layer_path.relative_to(package_root)))
    return cleaned


def _default_family_reference() -> Path:
    from isaac_model_contract import default_asset
    return default_asset('2fg7')


def _single_body_target(stage, joint_path: str, relationship: str):
    from pxr import UsdPhysics

    joint_prim = stage.GetPrimAtPath(joint_path)
    if not joint_prim.IsValid():
        raise RuntimeError(f'imported USD is missing joint {joint_path}')
    joint = UsdPhysics.Joint(joint_prim)
    targets = (
        joint.GetBody0Rel().GetTargets()
        if relationship == 'body0'
        else joint.GetBody1Rel().GetTargets())
    if len(targets) != 1:
        raise RuntimeError(
            f'{joint_path} must have exactly one {relationship} target, '
            f'got {list(targets)}')
    target = targets[0]
    if not stage.GetPrimAtPath(target).IsValid():
        raise RuntimeError(
            f'{joint_path} targets missing {relationship} body {target}')
    return target


def _normalize_import_hierarchy(stage) -> dict:
    """Flatten imported rigid links to the released 2FG family layout."""
    from pxr import Usd, UsdGeom, UsdPhysics

    root = stage.GetPrimAtPath(ROBOT_PRIM)
    geometry = stage.GetPrimAtPath(f'{ROBOT_PRIM}/Geometry')
    if not root.IsValid() or not geometry.IsValid():
        raise RuntimeError('raw import is missing the 2FG14 Geometry scope')

    sources = {
        'base_link': _single_body_target(
            stage, ACTUATED_JOINT, 'body0'),
        'right_finger_base_link': _single_body_target(
            stage, ACTUATED_JOINT, 'body1'),
        'left_finger_base_link': _single_body_target(
            stage, FOLLOWER_JOINT, 'body1'),
        'right_fingertip_link': _single_body_target(
            stage, FINGERTIP_JOINTS['right'], 'body1'),
        'left_fingertip_link': _single_body_target(
            stage, FINGERTIP_JOINTS['left'], 'body1'),
    }
    expected = {
        name: f'{ROBOT_PRIM}/Geometry/{name}' for name in FAMILY_LINKS}
    if str(sources['base_link']) != expected['base_link']:
        raise RuntimeError(
            'raw import has an unexpected base-link location: '
            f'{sources["base_link"]}')

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    world_transforms = {
        name: cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))
        for name, path in sources.items()
    }

    # Move fingertips first because the importer nests each tip below its jaw.
    # NamespaceEditor updates joint relationships as prim paths change.
    moved = []
    for name in (
            'right_fingertip_link', 'left_fingertip_link',
            'right_finger_base_link', 'left_finger_base_link'):
        source_path = str(sources[name])
        destination_path = expected[name]
        if source_path == destination_path:
            continue
        if stage.GetPrimAtPath(destination_path).IsValid():
            raise RuntimeError(
                f'cannot flatten {name}: destination already exists at '
                f'{destination_path}')
        source_prim = stage.GetPrimAtPath(source_path)
        editor = Usd.NamespaceEditor(stage)
        editor.ReparentPrim(source_prim, geometry)
        editor.ApplyEdits()

        moved_prim = stage.GetPrimAtPath(destination_path)
        if not moved_prim.IsValid():
            raise RuntimeError(
                f'failed to flatten {name} to {destination_path}')
        parent_world = UsdGeom.XformCache(
            Usd.TimeCode.Default()).GetLocalToWorldTransform(geometry)
        local_transform = world_transforms[name] * parent_world.GetInverse()
        xformable = UsdGeom.Xformable(moved_prim)
        xformable.ClearXformOpOrder()
        for attribute in list(moved_prim.GetAttributes()):
            if attribute.GetName().startswith('xformOp:'):
                moved_prim.RemoveProperty(attribute.GetName())
        xformable = UsdGeom.Xformable(moved_prim)
        xformable.AddTransformOp().Set(local_transform)
        moved.append({'name': name, 'from': source_path,
                      'to': destination_path})

    for name, path in expected.items():
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f'normalized USD is missing {name}: {path}')
        if name != 'base_link' and str(sources[name]) == path:
            continue
        nested_suffix = f'/{name}'
        nested = [
            str(candidate.GetPath()) for candidate in stage.Traverse()
            if str(candidate.GetPath()).endswith(nested_suffix)
            and str(candidate.GetPath()) != path
            and candidate.HasAPI(UsdPhysics.RigidBodyAPI)
        ]
        if nested:
            raise RuntimeError(
                f'normalized USD retains nested copies of {name}: {nested}')

    stage.GetRootLayer().Save()
    return {
        'layout': '2fg-family-flat-rigid-links',
        'links': [expected[name] for name in FAMILY_LINKS],
        'moved': moved,
    }


def _family_topology(stage, root_path: str) -> dict:
    from pxr import UsdPhysics

    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        raise RuntimeError(f'2FG family root is missing: {root_path}')

    def local_targets(relationship):
        result = []
        for target in relationship.GetTargets():
            target_text = str(target)
            if not target_text.startswith(f'{root_path}/'):
                raise RuntimeError(
                    f'2FG family relationship {relationship.GetPath()} '
                    f'escapes {root_path}: {target_text}')
            result.append(target_text[len(root_path):])
        return sorted(result)

    def value(prim, name):
        attribute = prim.GetAttribute(name)
        return attribute.Get() if attribute.IsValid() else None

    def api_paths(api):
        result = []
        for prim in stage.Traverse():
            if not prim.HasAPI(api):
                continue
            path = str(prim.GetPath())
            if path == root_path:
                result.append('/')
            elif path.startswith(f'{root_path}/'):
                result.append(path[len(root_path):])
            else:
                raise RuntimeError(
                    f'2FG family API root escapes {root_path}: {path}')
        return sorted(result)

    topology = {
        'robot_links': local_targets(
            root.GetRelationship('isaac:physics:robotLinks')),
        'robot_joints': local_targets(
            root.GetRelationship('isaac:physics:robotJoints')),
        'joints': {},
    }
    for name in FAMILY_JOINTS:
        prim = stage.GetPrimAtPath(f'{root_path}/Physics/{name}')
        if not prim.IsValid():
            raise RuntimeError(f'2FG family joint is missing: {name}')
        joint = UsdPhysics.Joint(prim)
        topology['joints'][name] = {
            'type': prim.GetTypeName(),
            'body0': local_targets(joint.GetBody0Rel()),
            'body1': local_targets(joint.GetBody1Rel()),
            'axis': (
                value(prim, 'physics:axis')),
        }

    driven = stage.GetPrimAtPath(f'{root_path}/Physics/finger_stroke')
    follower = stage.GetPrimAtPath(
        f'{root_path}/Physics/left_finger_base_joint')
    articulation = stage.GetPrimAtPath(f'{root_path}/Physics/root_joint')
    topology['interfaces'] = {
        'articulation_root': articulation.HasAPI(
            UsdPhysics.ArticulationRootAPI),
        'driven_linear_drive': driven.HasAPI(
            UsdPhysics.DriveAPI, 'linear'),
        'follower_linear_drive': follower.HasAPI(
            UsdPhysics.DriveAPI, 'linear'),
        'mimic_schema': 'PhysxMimicJointAPI:rotX' in set(
            follower.GetAppliedSchemas()),
        'mimic_reference': local_targets(follower.GetRelationship(
            'physxMimicJoint:rotX:referenceJoint')),
        'mimic_gearing': value(
            follower, 'physxMimicJoint:rotX:gearing'),
        'mimic_offset': value(
            follower, 'physxMimicJoint:rotX:offset'),
        'self_collision': value(
            articulation, 'physxArticulation:enabledSelfCollisions'),
        'solver_position_iterations': value(
            articulation,
            'physxArticulation:solverPositionIterationCount'),
        'physics_articulation_roots': api_paths(
            UsdPhysics.ArticulationRootAPI),
        'newton_articulation_roots': api_paths(
            'NewtonArticulationRootAPI'),
    }
    topology['rigid_links'] = sorted(
        path for path in topology['robot_links']
        if stage.GetPrimAtPath(f'{root_path}{path}').HasAPI(
            UsdPhysics.RigidBodyAPI))
    topology['filtered_pairs'] = {}
    for link in FAMILY_LINKS:
        link_prim = stage.GetPrimAtPath(
            f'{root_path}/Geometry/{link}')
        if not link_prim.IsValid():
            raise RuntimeError(f'2FG family link is missing: {link}')
        topology['filtered_pairs'][link] = local_targets(
            link_prim.GetRelationship('physics:filteredPairs'))
    return topology


def _validate_family_topology(stage, reference_stage) -> dict:
    reference_root = str(reference_stage.GetDefaultPrim().GetPath())
    reference = _family_topology(reference_stage, reference_root)
    candidate = _family_topology(stage, ROBOT_PRIM)
    if candidate != reference:
        raise RuntimeError(
            '2FG14 physical topology differs from the 2FG7 family reference: '
            f'candidate={candidate}, reference={reference}')
    return {
        'reference_model': '2fg7',
        'status': 'matched',
        'topology': candidate,
    }


def _author_release_overrides(stage) -> dict:
    from pxr import PhysxSchema
    from pxr import Sdf
    from pxr import Usd
    from pxr import UsdGeom
    from pxr import UsdPhysics

    root = stage.GetPrimAtPath(ROBOT_PRIM)
    articulation = stage.GetPrimAtPath(ARTICULATION_ROOT)
    driven = stage.GetPrimAtPath(ACTUATED_JOINT)
    follower = stage.GetPrimAtPath(FOLLOWER_JOINT)
    for label, prim in (
            ('robot', root), ('articulation root', articulation),
            ('actuated joint', driven), ('follower joint', follower)):
        if not prim.IsValid():
            raise RuntimeError(f'imported USD is missing {label}')

    stage.SetDefaultPrim(root)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    if not root.HasAPI('NewtonArticulationRootAPI'):
        root.ApplyAPI('NewtonArticulationRootAPI')
    layer_data = dict(stage.GetRootLayer().customLayerData)
    layer_data['creator'] = (
        f'OnRobot 2FG14 PhysX Asset Structure '
        f'{ASSET_STRUCTURE_VERSION}')
    stage.GetRootLayer().customLayerData = layer_data

    exact_links = [
        Sdf.Path(f'{ROBOT_PRIM}/Geometry/{name}')
        for name in FAMILY_LINKS]
    exact_joints = [
        Sdf.Path(f'{ROBOT_PRIM}/Physics/{name}')
        for name in FAMILY_JOINTS]

    package_root = Path(stage.GetRootLayer().realPath).parent
    stage.GetRootLayer().Save()

    def semantic_stage(relative):
        layer_stage = Usd.Stage.Open(str(package_root / relative))
        if layer_stage is None:
            raise RuntimeError(
                f'Asset Structure layer cannot be opened: {relative}')
        return layer_stage

    robot_stage = semantic_stage('payloads/robot.usda')
    robot_root = robot_stage.GetPrimAtPath(ROBOT_PRIM)
    if not robot_root.IsValid():
        raise RuntimeError('robot schema layer is missing the model root')
    for name, targets in (
            ('isaac:physics:robotLinks', exact_links),
            ('isaac:physics:robotJoints', exact_joints)):
        relationship = robot_root.CreateRelationship(name, custom=True)
        relationship.ClearTargets(removeSpec=False)
        for target in reversed(targets):
            relationship.AddTarget(
                target, position=Usd.ListPositionFrontOfPrependList)
    robot_stage.GetRootLayer().Save()
    robot_stage = None

    physics_stage = semantic_stage('payloads/Physics/physics.usda')
    physics_geometry = physics_stage.GetPrimAtPath(
        f'{ROBOT_PRIM}/Geometry')
    physics_root_joint = physics_stage.GetPrimAtPath(ARTICULATION_ROOT)
    physics_driven = physics_stage.GetPrimAtPath(ACTUATED_JOINT)
    physics_follower = physics_stage.GetPrimAtPath(FOLLOWER_JOINT)
    if (not physics_geometry.IsValid() or
            not physics_root_joint.IsValid() or
            not physics_driven.IsValid() or not physics_follower.IsValid()):
        raise RuntimeError('physics layer is missing the 2FG joints')

    # Asset Transformer may preserve the importer's articulation schemas on
    # Geometry. The released 2FG family has exactly one standard articulation
    # root at root_joint and only a Newton marker on the entry prim. A second
    # standard root prevents PhysX from returning articulation metadata.
    if physics_geometry.HasAPI(UsdPhysics.ArticulationRootAPI):
        physics_geometry.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    if physics_geometry.HasAPI('NewtonArticulationRootAPI'):
        physics_geometry.RemoveAPI('NewtonArticulationRootAPI')
    physics_geometry.RemoveProperty('newton:selfCollisionEnabled')

    # The importer may fix the base by connecting root_joint.body0 to the
    # model Xform. The released 2FG7 asset represents the same world anchor
    # with an empty body0 relationship and base_link as body1. Normalize the
    # 2FG14 to that family contract before comparing the composed topology.
    root_joint = UsdPhysics.Joint(physics_root_joint)
    root_joint.GetBody0Rel().ClearTargets(removeSpec=False)
    root_joint.GetBody1Rel().SetTargets([
        Sdf.Path(f'{ROBOT_PRIM}/Geometry/base_link')])
    if not physics_root_joint.HasAPI(
            UsdPhysics.ArticulationRootAPI):
        UsdPhysics.ArticulationRootAPI.Apply(physics_root_joint)

    for owner_name, target_names in FAMILY_FILTERED_PAIR_TARGETS.items():
        owner = physics_stage.GetPrimAtPath(
            f'{ROBOT_PRIM}/Geometry/{owner_name}')
        if not owner.IsValid():
            raise RuntimeError(
                f'physics layer is missing filtered-pair owner {owner_name}')
        filtered_pairs = UsdPhysics.FilteredPairsAPI.Apply(owner)
        filtered_pairs.CreateFilteredPairsRel().SetTargets([
            Sdf.Path(f'{ROBOT_PRIM}/Geometry/{target_name}')
            for target_name in target_names
        ])

    lower = physics_driven.GetAttribute('physics:lowerLimit')
    upper = physics_driven.GetAttribute('physics:upperLimit')
    if not lower.IsValid() or not upper.IsValid():
        raise RuntimeError('actuated USD joint has no limits')
    lower.Set(LOWER_LIMIT_M)
    upper.Set(UPPER_LIMIT_M)
    physics_follower.GetAttribute(
        'physics:lowerLimit').Set(LOWER_LIMIT_M)
    physics_follower.GetAttribute(
        'physics:upperLimit').Set(UPPER_LIMIT_M)

    drive = UsdPhysics.DriveAPI.Apply(physics_driven, 'linear')
    drive.CreateMaxForceAttr(MAXIMUM_DRIVE_FORCE_N)
    drive.CreateTargetPositionAttr(0.0)
    drive.CreateTargetVelocityAttr(0.0)

    material_prim = physics_stage.GetPrimAtPath(
        f'{ROBOT_PRIM}/Physics/fingertip_physics_material')
    if not material_prim.IsValid():
        material_prim = physics_stage.DefinePrim(
            f'{ROBOT_PRIM}/Physics/fingertip_physics_material', 'Material')
    material = UsdPhysics.MaterialAPI.Apply(material_prim)
    material.CreateStaticFrictionAttr(FINGERTIP_FRICTION)
    material.CreateDynamicFrictionAttr(FINGERTIP_DYNAMIC_FRICTION)
    material.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(
        material_prim).CreateFrictionCombineModeAttr().Set('average')
    physics_stage.GetRootLayer().Save()
    physics_stage = None

    physx_stage = semantic_stage('payloads/Physics/physx.usda')
    physx_articulation = physx_stage.GetPrimAtPath(ARTICULATION_ROOT)
    physx_driven = physx_stage.GetPrimAtPath(ACTUATED_JOINT)
    physx_follower = physx_stage.GetPrimAtPath(FOLLOWER_JOINT)
    if (not physx_articulation.IsValid() or
            not physx_driven.IsValid() or not physx_follower.IsValid()):
        raise RuntimeError('PhysX layer is missing 2FG articulation prims')

    for prim in (physx_driven, physx_follower):
        velocity = prim.GetAttribute('physxJoint:maxJointVelocity')
        if not velocity.IsValid():
            velocity = prim.CreateAttribute(
                'physxJoint:maxJointVelocity', Sdf.ValueTypeNames.Float)
        velocity.Set(MAXIMUM_VELOCITY_M_S)

    UsdPhysics.DriveAPI(
        physx_driven, 'linear').CreateMaxForceAttr(
            PHYSX_SINGLE_DRIVE_FORCE_N)
    PhysxSchema.PhysxJointAPI.Apply(physx_driven).CreateArmatureAttr(
        PROVISIONAL_ARMATURE_KG)

    if not physx_articulation.HasAPI(PhysxSchema.PhysxArticulationAPI):
        PhysxSchema.PhysxArticulationAPI.Apply(physx_articulation)
    self_collision = physx_articulation.GetAttribute(
        'physxArticulation:enabledSelfCollisions')
    if not self_collision.IsValid():
        self_collision = physx_articulation.CreateAttribute(
            'physxArticulation:enabledSelfCollisions',
            Sdf.ValueTypeNames.Bool)
    self_collision.Set(True)
    solver_iterations = physx_articulation.GetAttribute(
        'physxArticulation:solverPositionIterationCount')
    if not solver_iterations.IsValid():
        solver_iterations = physx_articulation.CreateAttribute(
            'physxArticulation:solverPositionIterationCount',
            Sdf.ValueTypeNames.Int)
    solver_iterations.Set(SOLVER_POSITION_ITERATIONS)
    solver_velocity_iterations = physx_articulation.GetAttribute(
        'physxArticulation:solverVelocityIterationCount')
    if not solver_velocity_iterations.IsValid():
        solver_velocity_iterations = physx_articulation.CreateAttribute(
            'physxArticulation:solverVelocityIterationCount',
            Sdf.ValueTypeNames.Int)
    solver_velocity_iterations.Set(SOLVER_VELOCITY_ITERATIONS)

    if physx_follower.HasAPI(UsdPhysics.DriveAPI, 'linear'):
        physx_follower.RemoveAPI(UsdPhysics.DriveAPI, 'linear')
    if not physx_follower.HasAPI(
            PhysxSchema.PhysxMimicJointAPI, 'rotX'):
        PhysxSchema.PhysxMimicJointAPI.Apply(physx_follower, 'rotX')
    mimic_values = {
        'physxMimicJoint:rotX:dampingRatio': 0.0,
        'physxMimicJoint:rotX:gearing': -1.0,
        'physxMimicJoint:rotX:naturalFrequency': 0.0,
        'physxMimicJoint:rotX:offset': 0.0,
    }
    for name, value in mimic_values.items():
        attribute = physx_follower.GetAttribute(name)
        if not attribute.IsValid():
            attribute = physx_follower.CreateAttribute(
                name, Sdf.ValueTypeNames.Float)
        attribute.Set(value)
    physx_follower.CreateRelationship(
        'physxMimicJoint:rotX:referenceJoint').SetTargets([
            Sdf.Path(ACTUATED_JOINT)])
    physx_stage.GetRootLayer().Save()
    physx_stage = None

    # Match the working 2FG7 entry layer: only the selected driven-joint
    # position and provisional gains override the routed semantic layers.
    stage.Reload()
    root = stage.GetPrimAtPath(ROBOT_PRIM)
    articulation = stage.GetPrimAtPath(ARTICULATION_ROOT)
    driven = stage.GetPrimAtPath(ACTUATED_JOINT)
    follower = stage.GetPrimAtPath(FOLLOWER_JOINT)
    for label, prim in (
            ('robot', root), ('articulation root', articulation),
            ('actuated joint', driven), ('follower joint', follower)):
        if not prim.IsValid():
            raise RuntimeError(
                f'composed USD lost {label} after semantic-layer edits')
    driven.GetAttribute('physics:lowerLimit').Set(LOWER_LIMIT_M)
    driven.GetAttribute('physics:upperLimit').Set(UPPER_LIMIT_M)
    drive = UsdPhysics.DriveAPI(driven, 'linear')
    drive.CreateTargetPositionAttr(0.0)
    drive.CreateStiffnessAttr(PROVISIONAL_STIFFNESS)
    drive.CreateDampingAttr(PROVISIONAL_DAMPING)

    def child_body(joint_prim, label):
        body_targets = (
            UsdPhysics.Joint(joint_prim).GetBody1Rel().GetTargets())
        if len(body_targets) != 1:
            raise RuntimeError(
                f'{label} joint must target exactly one child body, got '
                f'{list(body_targets)}')
        body_path = body_targets[0]
        body_prim = stage.GetPrimAtPath(body_path)
        if not body_prim.IsValid():
            raise RuntimeError(
                f'{label} joint targets missing body {body_path}')
        if not body_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError(
                f'{label} joint target is not a rigid body: {body_path}')
        return body_path, body_prim

    # The raw import is normalized before Asset Transformer, so the released
    # family paths are stable customization anchors and part of the gate.
    finger_bodies = {}
    for side, joint_prim in (
            ('right', driven), ('left', follower)):
        body_path, _ = child_body(joint_prim, f'{side} jaw')
        expected_path = Sdf.Path(
            f'{ROBOT_PRIM}/Geometry/{side}_finger_base_link')
        if body_path != expected_path:
            raise RuntimeError(
                f'{side} jaw body differs from the 2FG family layout: '
                f'{body_path} != {expected_path}')
        finger_bodies[side] = body_path

    fingertip_bodies = {}
    for side, joint_path in FINGERTIP_JOINTS.items():
        joint_prim = stage.GetPrimAtPath(joint_path)
        if not joint_prim.IsValid():
            raise RuntimeError(
                f'imported USD is missing {side} fingertip joint')
        body_path, body_prim = child_body(
            joint_prim, f'{side} fingertip')
        expected_path = Sdf.Path(
            f'{ROBOT_PRIM}/Geometry/{side}_fingertip_link')
        if body_path != expected_path:
            raise RuntimeError(
                f'{side} fingertip body differs from the 2FG family layout: '
                f'{body_path} != {expected_path}')
        fingertip_bodies[side] = (body_path, body_prim)

    forbidden = ('grip_stroke', 'task_aperture_link',
                 'virtual_stroke_link', 'mechanism_stroke')
    usd_paths = [str(prim.GetPath()) for prim in stage.Traverse()]
    for name in forbidden:
        if any(name in path for path in usd_paths):
            raise RuntimeError(
                f'imported articulation contains forbidden ROS coordinate '
                f'{name}')
    stage.GetRootLayer().Save()
    return {
        'finger_body_paths': {
            side: str(path) for side, path in finger_bodies.items()},
        'fingertip_body_paths': {
            side: str(body[0]) for side, body in fingertip_bodies.items()},
        'custom_finger_anchors': {
            side: str(path) for side, path in finger_bodies.items()},
    }


def _author_housing_appearance(entrypoint: Path) -> dict:
    """Scope the aluminum finish to the housing, not shared white labels."""
    from asset_visual_materials import author_housing_appearance

    return author_housing_appearance(entrypoint, '2fg14')


def main() -> int:
    """Expand, import, normalize, and inspect the 2FG14 asset."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xacro', type=Path, required=True)
    parser.add_argument('--package-root', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--family-reference', type=Path)
    args, _ = parser.parse_known_args()

    report = {
        'schema_version': 1,
        'model': MODEL,
        'isaac_sim_version': None,
        'status': 'failed',
        'output_dir': str(args.output_dir.resolve()),
        'authoring_program': str(Path(__file__).resolve()),
        'authoring_program_sha256': _sha256(Path(__file__).resolve()),
    }
    app = None
    try:
        if not args.xacro.is_file():
            raise RuntimeError(f'Xacro does not exist: {args.xacro}')
        if not args.package_root.is_dir():
            raise RuntimeError(
                f'2FG14 package root does not exist: {args.package_root}')
        report['entry_xacro'] = str(args.xacro.resolve())
        report['entry_xacro_sha256'] = _sha256(args.xacro.resolve())
        if args.output_dir.exists() and any(args.output_dir.iterdir()):
            raise RuntimeError(
                f'output directory must be absent or empty: '
                f'{args.output_dir}')

        with tempfile.TemporaryDirectory(
                prefix='onrobot_2fg14_authoring_') as temporary:
            temporary_root = Path(temporary)
            urdf_path = temporary_root / 'onrobot_2fg14.urdf'
            import_root = temporary_root / 'imported'
            transformed_root = temporary_root / 'transformed' / (
                'onrobot_2fg14')
            _expand_xacro(args.xacro.resolve(), urdf_path)
            _remove_frame_only_mount_links(urdf_path)
            _validate_import_urdf(urdf_path)

            from isaacsim import SimulationApp
            app = SimulationApp({
                'headless': True,
                'width': 1280,
                'height': 720,
            })
            from isaacsim.asset.importer.urdf import URDFImporter
            from isaacsim.asset.importer.urdf import URDFImporterConfig
            from isaacsim.asset.transformer.rules import DEFAULT_PROFILE_PATH
            from isaacsim.asset.importer.utils import (
                run_asset_transformer_profile)
            from isaacsim.core.version import get_version
            from pxr import Usd

            report['isaac_sim_version'] = get_version()[0]

            config = URDFImporterConfig(
                urdf_path=str(urdf_path),
                usd_path=str(import_root),
                merge_fixed_joints=False,
                merge_mesh=False,
                debug_mode=True,
                collision_from_visuals=False,
                collision_type='Convex Decomposition',
                allow_self_collision=True,
                ros_package_paths=[{
                    'name': 'onrobot_2fg14',
                    'path': str(args.package_root.resolve()),
                }],
                robot_type='End Effector',
                fix_base=True,
                joint_drive_type='force',
                joint_target_type='position',
                override_joint_stiffness=PROVISIONAL_STIFFNESS,
                override_joint_damping=PROVISIONAL_DAMPING,
                run_asset_transformer=False,
                run_multi_physics_conversion=True,
            )
            raw_generated = Path(URDFImporter(config).import_urdf())
            raw_stage = Usd.Stage.Open(str(raw_generated))
            if raw_stage is None:
                raise RuntimeError(
                    f'could not open raw imported USD: {raw_generated}')
            report['hierarchy_normalization'] = (
                _normalize_import_hierarchy(raw_stage))
            raw_stage = None

            run_asset_transformer_profile(
                input_stage_path=str(raw_generated),
                output_package_root=str(transformed_root),
                profile_json_path=DEFAULT_PROFILE_PATH,
            )
            generated = transformed_root / f'onrobot_{MODEL}.usda'
            if not generated.is_file():
                raise RuntimeError(
                    f'Asset Transformer did not create {generated}')
            report['asset_structure'] = _validate_asset_structure(
                generated, Path(DEFAULT_PROFILE_PATH))
            report['collision_model'] = (
                _normalize_collision_approximations(generated))
            report['reference_fingertip_collisions'] = (
                _author_reference_fingertip_collisions(generated))
            stage = Usd.Stage.Open(str(generated))
            if stage is None:
                raise RuntimeError(
                    f'could not open generated USD: {generated}')
            report.update(_author_release_overrides(stage))
            report['appearance'] = _author_housing_appearance(generated)
            report['release_hygiene'] = {
                'stripped_generator_documentation':
                    _strip_generator_documentation(generated.parent),
            }
            stage.Reload()
            report['semantic_layer_routing'] = (
                _validate_semantic_layer_routing(generated))
            reference_path = (
                args.family_reference or _default_family_reference())
            report['family_reference_sha256'] = _sha256(
                reference_path.resolve())
            reference_stage = Usd.Stage.Open(str(reference_path.resolve()))
            if reference_stage is None:
                raise RuntimeError(
                    f'could not open 2FG7 family reference: {reference_path}')
            report['family_contract'] = _validate_family_topology(
                stage, reference_stage)
            report['family_contract']['reference_asset'] = str(
                reference_path.resolve())
            reference_stage = None
            stage = None

            args.output_dir.parent.mkdir(parents=True, exist_ok=True)
            if args.output_dir.exists():
                args.output_dir.rmdir()
            shutil.copytree(generated.parent, args.output_dir)
            report['asset'] = str(
                (args.output_dir / generated.name).resolve())
            report['status'] = 'authored-unqualified'
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(
            json.dumps(report, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')
        print(json.dumps(report, indent=2, sort_keys=True), flush=True)
        exit_code = 0 if report['status'] == 'authored-unqualified' else 1
        if app is not None:
            app.close(exit_code=exit_code)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())

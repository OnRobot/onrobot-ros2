#!/usr/bin/env python3
"""Author an RG6 Asset Structure 3.0 package from the RG family contract."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET


MODEL = 'rg6'
ROOT = '/onrobot_rg6'
LINKS = (
    'base_link', 'body_link', 'left_outer_proximal_finger_link',
    'left_finger_base_link', 'left_finger_cover_link',
    'left_finger_tip_link', 'left_inner_proximal_link',
    'right_outer_proximal_finger_link', 'right_finger_base_link',
    'right_finger_cover_link', 'right_finger_tip_link',
    'right_inner_proximal_finger_link')
JOINTS = (
    'root_joint', 'body_joint', 'finger_joint', 'left_finger_base_joint',
    'left_finger_cover_joint', 'left_finger_tip_joint',
    'left_inner_proximal_joint',
    'left_outer_proximal_finger_joint', 'right_finger_base_link',
    'right_finger_cover_joint', 'right_finger_tip_joint',
    'right_inner_proximal_finger_joint')
MIMICS = {
    'left_finger_base_joint': -1.0,
    'left_finger_cover_joint': 1.0,
    'left_inner_proximal_joint': 1.0,
    'left_outer_proximal_finger_joint': 1.0,
    'right_finger_base_link': -1.0,
    'right_finger_cover_joint': 1.0,
    'right_inner_proximal_finger_joint': 1.0,
}
# PhysX mimic gearing uses the opposite sign convention from URDF mimic.
PHYSX_MIMICS = {name: -multiplier for name, multiplier in MIMICS.items()}
LOWER_RAD = 0.0
UPPER_RAD = 1.1928
MAX_VELOCITY_RAD_S = 0.8492845886  # RG6 firmware: 48.66 deg/s.
PROVISIONAL_MAX_TORQUE_NM = 16.17
# USD angular gains are per degree; preserve the declared SI control gains.
PROVISIONAL_STIFFNESS = 100.0 * math.pi / 180.0
PRIMARY_ARMATURE_KG_M2 = 3.5e-6 * (50.0 * 14.0) ** 2  # Firmware motor/gear model, not unit identification.
PROVISIONAL_DAMPING = 2.0 * math.sqrt(PRIMARY_ARMATURE_KG_M2 * 100.0) * math.pi / 180.0
MIMIC_DAMPING_RATIO = 1.0
FINGERTIP_FRICTION = 0.6
FINGERTIP_DYNAMIC_FRICTION = 0.5
# Keep the visible pad's +Z contact face at 5 mm while backing the thin
# rubber envelope with an 11.4 mm closed collision depth inside the rigid tip.
FINGERTIP_PAD_SIZE_M = (0.037, 0.025, 0.0114)
FINGERTIP_PAD_CENTER_M = (0.0, 0.0001, -0.0007)
# TGS no longer converts velocity iterations above four into position
# iterations. Preserve the previous aggregate iteration budget explicitly and
# stay within the supported TGS velocity-iteration range.
SOLVER_POSITION_ITERATIONS = 68
SOLVER_VELOCITY_ITERATIONS = 4


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _expand_xacro(xacro: Path, output: Path) -> None:
    subprocess.run([
        'xacro', str(xacro), 'backend:=fake',
        'use_standard_fingertips:=true', 'fingertip_orientation:=inwards',
    ], check=True, stdout=output.open('wb'))


def _make_physical_only_urdf(path: Path) -> None:
    """Remove ROS task coordinates and massless custom-finger frame links."""
    tree = ET.parse(path)
    robot = tree.getroot()
    # Keep the imported RG6 input aligned with the already released RG-family
    # USD namespace. The CAD exporter uses ``*_finger_*`` for the left inner
    # pair and calls the right base joint ``*_base_joint``; the normalized
    # Asset Structure 3.0 layers use the shorter historical names below.
    renames = {
        'left_inner_proximal_finger_link': 'left_inner_proximal_link',
        'left_inner_proximal_finger_joint': 'left_inner_proximal_joint',
        'right_finger_base_joint': 'right_finger_base_link',
    }
    for element in robot.iter():
        for attribute, value in list(element.attrib.items()):
            element.set(attribute, renames.get(value, value))
    links = {link.attrib['name']: link for link in robot.findall('link')}
    joints = {joint.attrib['name']: joint for joint in robot.findall('joint')}

    for name in ('grip_stroke',):
        joint = joints.get(name)
        if joint is not None:
            robot.remove(joint)
    task_link = links.get('grip_stroke_control_link')
    if task_link is not None:
        robot.remove(task_link)

    for side in ('left', 'right'):
        mount_name = f'{side}_finger_mount'
        mount_joint_name = f'{mount_name}_joint'
        tip_joint = joints[f'{side}_finger_tip_joint']
        mount_joint = joints[mount_joint_name]
        mount_origin = mount_joint.find('origin')
        tip_origin = tip_joint.find('origin')
        tip_joint.find('parent').attrib['link'] = f'{side}_finger_base_link'
        # The mount has translation only and the tip joint has rotation only.
        tip_origin.attrib['xyz'] = mount_origin.attrib['xyz']
        robot.remove(mount_joint)
        robot.remove(links[mount_name])

    for control in robot.findall('ros2_control'):
        robot.remove(control)
    tree.write(path, encoding='utf-8', xml_declaration=True)


def _validate_physical_urdf(path: Path) -> dict:
    """Prove the import input contains exactly the intended RG6 mechanism."""
    robot = ET.parse(path).getroot()
    links = {link.get('name') for link in robot.findall('link')}
    joints = {joint.get('name'): joint for joint in robot.findall('joint')}
    expected_joints = set(JOINTS) - {'root_joint'}
    errors = []
    if links != set(LINKS):
        errors.append({
            'kind': 'links',
            'missing': sorted(set(LINKS) - links),
            'unexpected': sorted(links - set(LINKS)),
        })
    if set(joints) != expected_joints:
        errors.append({
            'kind': 'joints',
            'missing': sorted(expected_joints - set(joints)),
            'unexpected': sorted(set(joints) - expected_joints),
        })
    actual_mimics = {}
    for name, joint in joints.items():
        mimic = joint.find('mimic')
        if mimic is not None:
            actual_mimics[name] = {
                'joint': mimic.get('joint'),
                'multiplier': float(mimic.get('multiplier', '1')),
                'offset': float(mimic.get('offset', '0')),
            }
        parent = joint.find('parent')
        child = joint.find('child')
        if (parent is None or child is None or
                parent.get('link') not in links or
                child.get('link') not in links):
            errors.append({'kind': 'joint-bodies', 'joint': name})
    expected_mimics = {
        name: {'joint': 'finger_joint', 'multiplier': multiplier,
               'offset': 0.0}
        for name, multiplier in MIMICS.items()}
    if actual_mimics != expected_mimics:
        errors.append({
            'kind': 'mimics', 'expected': expected_mimics,
            'actual': actual_mimics})
    missing_inertials = sorted(
        link.get('name') for link in robot.findall('link')
        if link.find('inertial') is None)
    if missing_inertials:
        errors.append({
            'kind': 'massless-links', 'links': missing_inertials})
    if errors:
        raise RuntimeError(
            'physical-only RG6 URDF differs from its declared topology: '
            f'{json.dumps(errors, sort_keys=True)}')
    return {
        'links': sorted(links),
        'joints': sorted(joints),
        'mimics': actual_mimics,
        'massless_links': [],
    }


def _normalize_import_hierarchy(stage) -> dict:
    """Flatten RG6 rigid bodies to the same layout convention as RG2."""
    from pxr import Usd, UsdGeom, UsdPhysics

    geometry_path = f'{ROOT}/Geometry'
    geometry = stage.GetPrimAtPath(geometry_path)
    if not geometry.IsValid():
        raise RuntimeError('raw RG6 import is missing its Geometry scope')

    sources = {}
    rigid_paths = [
        prim.GetPath() for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
    for name in LINKS:
        matches = [path for path in rigid_paths if path.name == name]
        if len(matches) != 1:
            raise RuntimeError(
                f'raw RG6 import must contain one rigid body named {name}; '
                f'found {[str(path) for path in matches]} among '
                f'{[str(path) for path in rigid_paths]}')
        sources[name] = matches[0]

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    world_transforms = {
        name: cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))
        for name, path in sources.items()}
    moved = []
    # Children must move before their ancestors. NamespaceEditor keeps the
    # joint body relationships synchronized with every move.
    ordered = sorted(
        LINKS, key=lambda name: str(sources[name]).count('/'), reverse=True)
    for name in ordered:
        source_path = sources[name]
        destination_path = f'{geometry_path}/{name}'
        if str(source_path) == destination_path:
            continue
        if stage.GetPrimAtPath(destination_path).IsValid():
            raise RuntimeError(
                f'cannot flatten {name}: destination already exists at '
                f'{destination_path}')
        editor = Usd.NamespaceEditor(stage)
        editor.ReparentPrim(stage.GetPrimAtPath(source_path), geometry)
        editor.ApplyEdits()
        moved_prim = stage.GetPrimAtPath(destination_path)
        if not moved_prim.IsValid():
            raise RuntimeError(
                f'failed to flatten {name} to {destination_path}')
        parent_world = UsdGeom.XformCache(
            Usd.TimeCode.Default()).GetLocalToWorldTransform(geometry)
        local = world_transforms[name] * parent_world.GetInverse()
        xformable = UsdGeom.Xformable(moved_prim)
        xformable.ClearXformOpOrder()
        for attribute in list(moved_prim.GetAttributes()):
            if attribute.GetName().startswith('xformOp:'):
                moved_prim.RemoveProperty(attribute.GetName())
        UsdGeom.Xformable(moved_prim).AddTransformOp().Set(local)
        moved.append({'name': name, 'from': str(source_path),
                      'to': destination_path})

    for name in LINKS:
        expected = f'{geometry_path}/{name}'
        if not stage.GetPrimAtPath(expected).IsValid():
            raise RuntimeError(
                f'normalized RG6 import is missing {expected}')
    stage.GetRootLayer().Save()
    return {
        'layout': 'rg-family-flat-rigid-links',
        'links': [f'{geometry_path}/{name}' for name in LINKS],
        'moved': moved,
    }


def _canonical_entrypoint(path: Path) -> None:
    text = """#usda 1.0
(
    customLayerData = {
        string creator = "OnRobot RG6 PhysX Asset Structure 3.0"
    }
    defaultPrim = "onrobot_rg6"
    kilogramsPerUnit = 1
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "onrobot_rg6" (
    prepend references = @./payloads/base.usda@
    variants = { string Physics = "physx_parallel_grip" }
    append variantSets = "Physics"
)
{
    variantSet "Physics" = {
        "none" {}
        "physics" (prepend payload = @./payloads/Physics/physics.usda@) {}
        "physx" (prepend payload = @./payloads/Physics/physx.usda@) {}
        "physx_parallel_grip" (
            prepend payload = @./payloads/Physics/physx_parallel_grip.usda@
        ) {}
        "physx_parallel_grip_tip_contact" (
            prepend payload = @TIP_CONTACT_PHYSX_LAYER@
        ) {}
    }
}

def PhysicsScene "PhysicsScene" (prepend apiSchemas = ["PhysxSceneAPI"])
{
    bool physxScene:enableStabilization = 1
    uint physxScene:gpuFoundLostAggregatePairsCapacity = 8192
    bool physxScene:solveArticulationContactLast = 1
}
"""
    text = text.replace(
        '@TIP_CONTACT_PHYSX_LAYER@',
        '@./payloads/Physics/physx_parallel_grip_tip_contact.usda@')
    path.write_text(text, encoding='utf-8')


def _strip_generator_documentation(package: Path) -> list[str]:
    """Remove temporary Asset Transformer paths from layer metadata."""
    cleaned = []
    documentation = re.compile(r'\n\s+doc = """.*?"""', re.DOTALL)
    for layer_path in sorted(package.rglob('*.usda')):
        text = layer_path.read_text(encoding='utf-8')
        metadata_end = text.find('\n)')
        if metadata_end < 0:
            continue
        metadata, body = text[:metadata_end], text[metadata_end:]
        metadata, substitutions = documentation.subn(
            '', metadata, count=1)
        if substitutions:
            layer_path.write_text(metadata + body, encoding='utf-8')
            cleaned.append(str(layer_path.relative_to(package)))
    return cleaned


def _clear_imported_articulation_roots(package: Path) -> dict:
    """Remove importer roots before physics.usda receives sole ownership."""
    from pxr import Sdf, Usd, UsdPhysics

    removed = []
    # Process the semantic leaves before base.usda, which composes robot.usda.
    layers = sorted(
        package.rglob('*.usda'),
        key=lambda item: (len(item.relative_to(package).parts), str(item)),
        reverse=True)
    for layer_path in layers:
        stage = Usd.Stage.Open(str(layer_path))
        if stage is None:
            raise RuntimeError(
                f'could not inspect articulation ownership in {layer_path}')
        changed = False
        for prim in list(stage.Traverse()):
            path = str(prim.GetPath())
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                removed.append({
                    'layer': str(layer_path.relative_to(package)),
                    'path': path,
                    'schema': 'PhysicsArticulationRootAPI'})
                changed = True
            if prim.HasAPI('NewtonArticulationRootAPI'):
                prim.RemoveAPI('NewtonArticulationRootAPI')
                removed.append({
                    'layer': str(layer_path.relative_to(package)),
                    'path': path,
                    'schema': 'NewtonArticulationRootAPI'})
                changed = True
            if prim.HasProperty('newton:selfCollisionEnabled'):
                prim.RemoveProperty('newton:selfCollisionEnabled')
                changed = True
        if changed:
            stage.GetRootLayer().Save()

    # The transformer can author the Geometry root in its interface layer.
    # That layer is replaced below, so a delete opinion written there would
    # disappear. Place the durable suppression in base.usda, alongside the
    # composed geometry hierarchy and above its robot.usda sublayer.
    base_path = package / 'payloads/base.usda'
    base_layer = Sdf.Layer.FindOrOpen(str(base_path))
    if base_layer is None:
        raise RuntimeError(f'could not open RG6 base layer {base_path}')
    geometry_spec = Sdf.CreatePrimInLayer(
        base_layer, f'{ROOT}/Geometry')
    existing = geometry_spec.GetInfo('apiSchemas')
    forbidden = {
        'PhysicsArticulationRootAPI', 'NewtonArticulationRootAPI'}
    new_op = Sdf.TokenListOp()
    if isinstance(existing, Sdf.TokenListOp) and existing.isExplicit:
        new_op.explicitItems = [
            item for item in existing.explicitItems if item not in forbidden]
    else:
        if isinstance(existing, Sdf.TokenListOp):
            new_op.prependedItems = [
                item for item in existing.prependedItems
                if item not in forbidden]
            new_op.appendedItems = [
                item for item in existing.appendedItems
                if item not in forbidden]
        deleted = (
            list(existing.deletedItems)
            if isinstance(existing, Sdf.TokenListOp) else [])
        new_op.deletedItems = list(dict.fromkeys(deleted + sorted(forbidden)))
    geometry_spec.SetInfo('apiSchemas', new_op)
    base_layer.Save()
    return {
        'canonical_layer': 'payloads/Physics/physics.usda',
        'canonical_path': ROOT,
        'durable_geometry_suppression_layer': 'payloads/base.usda',
        'removed_importer_opinions': removed,
    }


def _remove_composed_geometry_articulation_roots(entrypoint: Path) -> dict:
    """Remove articulation APIs from their actual composed source specs.

    Asset Transformer layers cannot be reasoned about reliably by opening each
    layer as an independent stage: references, sublayers, and list operations
    can produce a composed API that is absent from that standalone view.  Use
    the final stage's prim stack to find and edit the contributing specs.
    """
    from pxr import Sdf, Usd, UsdPhysics

    forbidden = {
        'PhysicsArticulationRootAPI', 'NewtonArticulationRootAPI'}
    stage = Usd.Stage.Open(str(entrypoint))
    if stage is None:
        raise RuntimeError(f'could not open final RG6 stage {entrypoint}')
    geometry = stage.GetPrimAtPath(f'{ROOT}/Geometry')
    if not geometry:
        raise RuntimeError(f'final RG6 stage is missing {ROOT}/Geometry')

    inspected = []
    changed_layers = set()
    for spec in geometry.GetPrimStack():
        operation = spec.GetInfo('apiSchemas')
        if not isinstance(operation, Sdf.TokenListOp):
            continue
        positive = set(operation.explicitItems)
        positive.update(operation.prependedItems)
        positive.update(operation.appendedItems)
        present = sorted(positive & forbidden)
        inspected.append({
            'layer': spec.layer.realPath or spec.layer.identifier,
            'positive_articulation_schemas': present,
        })
        if not present:
            continue

        replacement = Sdf.TokenListOp()
        if operation.isExplicit:
            replacement.explicitItems = [
                item for item in operation.explicitItems
                if item not in forbidden]
        else:
            replacement.prependedItems = [
                item for item in operation.prependedItems
                if item not in forbidden]
            replacement.appendedItems = [
                item for item in operation.appendedItems
                if item not in forbidden]
            replacement.deletedItems = list(dict.fromkeys(
                list(operation.deletedItems) + sorted(forbidden)))
        spec.SetInfo('apiSchemas', replacement)
        spec.layer.Save()
        changed_layers.add(spec.layer.realPath or spec.layer.identifier)

    # Reopen to discard composition caches and prove the correction happened.
    stage = Usd.Stage.Open(str(entrypoint))
    roots = [
        str(prim.GetPath()) for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if roots != [ROOT]:
        raise RuntimeError(
            'could not establish sole RG6 articulation ownership after '
            f'composed-layer cleanup: roots={roots}, prim_stack={inspected}')
    return {
        'geometry_prim_stack': inspected,
        'changed_layers': sorted(changed_layers),
        'articulation_roots_after_cleanup': roots,
    }


def _author_physics(path: Path) -> None:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    from pxr import PhysxSchema

    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise RuntimeError(f'could not open physics layer {path}')
    root = stage.GetPrimAtPath(ROOT)
    if not root:
        raise RuntimeError(f'imported USD is missing {ROOT}')
    for prim in stage.Traverse():
        if (prim != root and
                prim.HasAPI(UsdPhysics.ArticulationRootAPI)):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    UsdPhysics.ArticulationRootAPI.Apply(root)

    material = UsdPhysics.MaterialAPI.Apply(
        stage.DefinePrim(f'{ROOT}/Physics/fingertip_physics_material',
                         'Material'))
    material.CreateStaticFrictionAttr(FINGERTIP_FRICTION)
    material.CreateDynamicFrictionAttr(FINGERTIP_DYNAMIC_FRICTION)
    material.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(
        material.GetPrim()).CreateFrictionCombineModeAttr().Set('average')

    paths = [Sdf.Path(f'{ROOT}/Geometry/{name}') for name in LINKS]
    tips = {
        Sdf.Path(f'{ROOT}/Geometry/left_finger_tip_link'),
        Sdf.Path(f'{ROOT}/Geometry/right_finger_tip_link')}
    for body_path in paths:
        body = stage.GetPrimAtPath(body_path)
        if not body:
            raise RuntimeError(
                f'imported USD is missing rigid link {body_path}')
        filtered = [other for other in paths
                    if other != body_path and not ({body_path, other} == tips)]
        filtered_pairs = UsdPhysics.FilteredPairsAPI.Apply(body)
        filtered_pairs.CreateFilteredPairsRel().SetTargets(filtered)
        if body_path in tips:
            binding = UsdShade.MaterialBindingAPI.Apply(body)
            binding.Bind(
                UsdShade.Material(material.GetPrim()),
                UsdShade.Tokens.weakerThanDescendants,
                'physics')
            # The imported RG6 fingertip collision mesh is not manifold. Its
            # decomposition is therefore PhysX-version dependent. Replace it
            # with a closed box that preserves the visible EPDM contact face
            # and adds a rigidly backed collision depth.
            imported_collision = stage.OverridePrim(
                f'{body_path}/fingertip_standard_link_1')
            imported_collision.SetActive(False)
            pad = UsdGeom.Cube.Define(
                stage, f'{body_path}/payload_contact_collision')
            pad.CreateSizeAttr(1.0)
            pad_xform = UsdGeom.Xformable(pad.GetPrim())
            pad_xform.AddTranslateOp().Set(
                Gf.Vec3d(*FINGERTIP_PAD_CENTER_M))
            pad_xform.AddScaleOp().Set(Gf.Vec3f(*FINGERTIP_PAD_SIZE_M))
            UsdGeom.Imageable(pad.GetPrim()).CreatePurposeAttr().Set(
                UsdGeom.Tokens.guide)
            UsdPhysics.CollisionAPI.Apply(
                pad.GetPrim()).CreateCollisionEnabledAttr().Set(True)
            UsdShade.MaterialBindingAPI.Apply(pad.GetPrim()).Bind(
                UsdShade.Material(material.GetPrim()),
                UsdShade.Tokens.strongerThanDescendants,
                'physics')

    leader = stage.GetPrimAtPath(f'{ROOT}/Physics/finger_joint')
    if not leader:
        raise RuntimeError('imported USD is missing finger_joint')
    leader.CreateAttribute('physics:lowerLimit', Sdf.ValueTypeNames.Float).Set(
        0.0)
    leader.CreateAttribute('physics:upperLimit', Sdf.ValueTypeNames.Float).Set(
        float(UPPER_RAD * 180.0 / 3.141592653589793))
    leader.CreateAttribute(
        'drive:angular:physics:maxForce', Sdf.ValueTypeNames.Float).Set(
            PROVISIONAL_MAX_TORQUE_NM)
    leader.CreateAttribute(
        'drive:angular:physics:stiffness', Sdf.ValueTypeNames.Float).Set(
            PROVISIONAL_STIFFNESS)
    leader.CreateAttribute(
        'drive:angular:physics:damping', Sdf.ValueTypeNames.Float).Set(
            PROVISIONAL_DAMPING)
    for name in MIMICS:
        follower = stage.GetPrimAtPath(f'{ROOT}/Physics/{name}')
        if not follower:
            raise RuntimeError(f'imported USD is missing mimic joint {name}')
        if follower.HasAPI(UsdPhysics.DriveAPI, 'angular'):
            follower.RemoveAPI(UsdPhysics.DriveAPI, 'angular')
        # PhysX mimic joints require finite limits. Match the working RG
        # family policy instead of retaining continuous or zero-width URDF
        # import artifacts.
        if name == 'left_outer_proximal_finger_joint':
            lower_deg = 0.0
            upper_deg = float(
                UPPER_RAD * 180.0 / 3.141592653589793)
        else:
            lower_deg = -180.0
            upper_deg = 180.0
        follower.CreateAttribute(
            'physics:lowerLimit', Sdf.ValueTypeNames.Float).Set(lower_deg)
        follower.CreateAttribute(
            'physics:upperLimit', Sdf.ValueTypeNames.Float).Set(upper_deg)
    stage.GetRootLayer().Save()


def _author_physx(path: Path, physics_relative: str,
                  self_collision: bool) -> None:
    from pxr import Sdf, Usd
    from pxr import PhysxSchema

    path.unlink(missing_ok=True)
    layer = Sdf.Layer.CreateNew(str(path))
    layer.defaultPrim = 'onrobot_rg6'
    layer.subLayerPaths = [physics_relative]
    stage = Usd.Stage.Open(layer)
    root = stage.OverridePrim(ROOT)
    PhysxSchema.PhysxArticulationAPI.Apply(root)
    root.CreateAttribute(
        'physxArticulation:enabledSelfCollisions',
        Sdf.ValueTypeNames.Bool).Set(self_collision)
    root.CreateAttribute(
        'physxArticulation:solverPositionIterationCount',
        Sdf.ValueTypeNames.Int).Set(SOLVER_POSITION_ITERATIONS)
    root.CreateAttribute(
        'physxArticulation:solverVelocityIterationCount',
        Sdf.ValueTypeNames.Int).Set(SOLVER_VELOCITY_ITERATIONS)

    for name in ('finger_joint', *MIMICS):
        joint = stage.OverridePrim(f'{ROOT}/Physics/{name}')
        PhysxSchema.PhysxJointAPI.Apply(joint)
        joint.CreateAttribute(
            'physxJoint:armature', Sdf.ValueTypeNames.Float).Set(
                PRIMARY_ARMATURE_KG_M2 if name == 'finger_joint' else 0.0001)
        joint.CreateAttribute(
            'physxJoint:maxJointVelocity', Sdf.ValueTypeNames.Float).Set(
                float(MAX_VELOCITY_RAD_S * 180.0 / 3.141592653589793))
        if name == 'finger_joint':
            continue
        PhysxSchema.PhysxMimicJointAPI.Apply(joint, 'rotZ')
        values = {
            'dampingRatio': MIMIC_DAMPING_RATIO,
            'gearing': PHYSX_MIMICS[name],
            'naturalFrequency': 5000.0,
            'offset': 0.0,
        }
        for attribute, value in values.items():
            joint.CreateAttribute(
                f'physxMimicJoint:rotZ:{attribute}',
                Sdf.ValueTypeNames.Float).Set(value)
        joint.CreateRelationship(
            'physxMimicJoint:rotZ:referenceJoint').SetTargets([
                Sdf.Path(f'{ROOT}/Physics/finger_joint')])
    layer.Save()


def _validate_rg_family(package: Path, reference: Path) -> dict:
    required = {
        'onrobot_rg6.usda', 'payloads/base.usda', 'payloads/geometries.usd',
        'payloads/instances.usda', 'payloads/materials.usda',
        'payloads/robot.usda', 'payloads/Physics/physics.usda',
        'payloads/Physics/physx.usda',
        'payloads/Physics/physx_parallel_grip.usda',
        'payloads/Physics/physx_parallel_grip_tip_contact.usda'}
    missing = sorted(str(item) for item in required
                     if not (package / item).is_file())
    if missing:
        raise RuntimeError(f'RG6 package lacks RG-family layers: {missing}')
    reference_entry = reference.read_text(encoding='utf-8')
    candidate_entry = (package / 'onrobot_rg6.usda').read_text(
        encoding='utf-8')
    variants = ('none', 'physics', 'physx', 'physx_parallel_grip',
                'physx_parallel_grip_tip_contact')
    for variant in variants:
        if (f'"{variant}"' not in reference_entry or
                f'"{variant}"' not in candidate_entry):
            raise RuntimeError(
                f'RG-family Physics variant is missing: {variant}')
    physics_text = (package / 'payloads/Physics/physics.usda').read_text(
        encoding='utf-8')
    for value in ('physics:staticFriction = 0.6',
                  'physics:dynamicFriction = 0.5',
                  'physxMaterial:frictionCombineMode = "average"',
                  'drive:angular:physics:maxForce = 16.17'):
        if value not in physics_text:
            raise RuntimeError(f'RG6 physics policy mismatch: {value}')
    for name, expected in (('stiffness', PROVISIONAL_STIFFNESS),
                           ('damping', PROVISIONAL_DAMPING)):
        match = re.search(r'drive:angular:physics:' + name + r'\s*=\s*([0-9.eE+-]+)', physics_text)
        if not match or not math.isclose(float(match.group(1)), expected, rel_tol=1e-6):
            raise RuntimeError(f'RG6 angular drive units/value mismatch: {name}')
    physx_text = (package / 'payloads/Physics/physx.usda').read_text(
        encoding='utf-8')
    for value in (
            'physxArticulation:solverPositionIterationCount = 68',
            'physxArticulation:solverVelocityIterationCount = 4'):
        if value not in physx_text:
            raise RuntimeError(f'RG6 PhysX policy mismatch: {value}')
    instances_text = (package / 'payloads/instances.usda').read_text(
        encoding='utf-8')
    if ('physics:approximation = "convexHull"' in instances_text or
            'physics:approximation = "convexDecomposition"' not in
            instances_text):
        raise RuntimeError(
            'RG6 collision meshes must use convex decomposition')
    forbidden = ('grip_stroke', 'mechanism_stroke', 'virtual_stroke_link')
    text_layers = '\n'.join(
        path.read_text(encoding='utf-8') for path in package.rglob('*.usda'))
    for name in forbidden:
        if name in text_layers:
            raise RuntimeError(f'RG6 USD contains ROS-only coordinate {name}')
    return {
        'reference': str(reference.resolve()),
        'reference_sha256': _sha256(reference),
        'layer_layout': 'matched',
        'physics_variants': list(variants),
        'fingertip_material': 'matched-full-rated-retention-profile',
        'topology': 'model-specific-validated',
    }


def _validate_authored_stage(entrypoint: Path) -> dict:
    """Validate the RG6 articulation and collision-policy inventory."""
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    from pxr import PhysxSchema

    stage = Usd.Stage.Open(str(entrypoint))
    if stage is None:
        raise RuntimeError(f'could not open authored RG6 asset {entrypoint}')
    root = stage.GetPrimAtPath(ROOT)
    errors = []
    variants = root.GetVariantSet('Physics')
    variant_names = set(variants.GetVariantNames())
    expected_variants = {
        'none', 'physics', 'physx', 'physx_parallel_grip',
        'physx_parallel_grip_tip_contact'}
    if variant_names != expected_variants:
        errors.append({
            'kind': 'physics-variants',
            'missing': sorted(expected_variants - variant_names),
            'unexpected': sorted(variant_names - expected_variants),
        })
    if variants.GetVariantSelection() != 'physx_parallel_grip':
        errors.append({
            'kind': 'default-physics-variant',
            'actual': variants.GetVariantSelection()})

    rigid = {
        str(prim.GetPath()) for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.RigidBodyAPI)}
    expected_rigid = {f'{ROOT}/Geometry/{name}' for name in LINKS}
    if rigid != expected_rigid:
        errors.append({
            'kind': 'rigid-links',
            'missing': sorted(expected_rigid - rigid),
            'unexpected': sorted(rigid - expected_rigid),
        })
    joints = {
        prim.GetName() for prim in stage.Traverse()
        if prim.IsA(UsdPhysics.Joint)}
    if joints != set(JOINTS):
        errors.append({
            'kind': 'physics-joints',
            'missing': sorted(set(JOINTS) - joints),
            'unexpected': sorted(joints - set(JOINTS)),
        })

    if not root.HasAPI(UsdPhysics.ArticulationRootAPI):
        errors.append({'kind': 'missing-articulation-root'})
    if not root.HasAPI(PhysxSchema.PhysxArticulationAPI):
        errors.append({'kind': 'missing-physx-articulation'})
    roots = [
        str(prim.GetPath()) for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if roots != [ROOT]:
        errors.append({'kind': 'articulation-roots', 'actual': roots})

    collision_approximations = {}
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            continue
        approximation = UsdPhysics.MeshCollisionAPI(
            prim).GetApproximationAttr().Get()
        collision_approximations[str(prim.GetPath())] = approximation
        if approximation != 'convexDecomposition':
            errors.append({
                'kind': 'collision-mesh-approximation',
                'prim': str(prim.GetPath()),
                'actual': approximation,
            })
    if not collision_approximations:
        errors.append({'kind': 'missing-collision-meshes'})

    tips = {
        Sdf.Path(f'{ROOT}/Geometry/left_finger_tip_link'),
        Sdf.Path(f'{ROOT}/Geometry/right_finger_tip_link')}
    expected_paths = {Sdf.Path(path) for path in expected_rigid}
    for body_path in expected_paths:
        body = stage.GetPrimAtPath(body_path)
        filtered_api = UsdPhysics.FilteredPairsAPI(body)
        actual = set(filtered_api.GetFilteredPairsRel().GetTargets())
        expected = {
            other for other in expected_paths
            if other != body_path and {body_path, other} != tips}
        if actual != expected:
            errors.append({
                'kind': 'filtered-pairs', 'body': str(body_path),
                'missing': sorted(str(item) for item in expected - actual),
                'unexpected': sorted(str(item) for item in actual - expected),
            })
    material_path = Sdf.Path(
        f'{ROOT}/Physics/fingertip_physics_material')
    material = UsdPhysics.MaterialAPI(stage.GetPrimAtPath(material_path))
    friction = {
        'static': material.GetStaticFrictionAttr().Get(),
        'dynamic': material.GetDynamicFrictionAttr().Get(),
        'restitution': material.GetRestitutionAttr().Get(),
        'combine_mode': PhysxSchema.PhysxMaterialAPI(
            material.GetPrim()).GetFrictionCombineModeAttr().Get(),
    }
    if (friction['static'] is None or friction['dynamic'] is None or
            friction['restitution'] is None or
            abs(friction['static'] - FINGERTIP_FRICTION) > 1e-6 or
            abs(friction['dynamic'] - FINGERTIP_DYNAMIC_FRICTION) > 1e-6 or
            abs(friction['restitution']) > 1e-6 or
            friction['combine_mode'] != 'average'):
        errors.append({'kind': 'fingertip-material', 'actual': friction})
    for tip in tips:
        tip_prim = stage.GetPrimAtPath(tip)
        if not tip_prim.HasAPI(UsdShade.MaterialBindingAPI):
            errors.append({
                'kind': 'fingertip-material-binding-api', 'tip': str(tip)})
        targets = tip_prim.GetRelationship(
            'material:binding:physics').GetTargets()
        if targets != [material_path]:
            errors.append({
                'kind': 'fingertip-material-binding', 'tip': str(tip),
                'actual': [str(target) for target in targets]})
        imported_collision = stage.GetPrimAtPath(
            f'{tip}/fingertip_standard_link_1')
        if (not imported_collision.IsValid() or
                imported_collision.IsActive()):
            errors.append({
                'kind': 'non-manifold-imported-fingertip-collision',
                'tip': str(tip),
                'active': (imported_collision.IsActive()
                           if imported_collision.IsValid() else None),
            })
        pad_path = Sdf.Path(f'{tip}/payload_contact_collision')
        pad_prim = stage.GetPrimAtPath(pad_path)
        if (not pad_prim.IsValid() or
                not pad_prim.IsA(UsdGeom.Cube) or
                not pad_prim.HasAPI(UsdPhysics.CollisionAPI)):
            errors.append({
                'kind': 'missing-closed-fingertip-contact-proxy',
                'tip': str(tip),
            })
        else:
            size = pad_prim.GetAttribute('xformOp:scale').Get()
            center = pad_prim.GetAttribute('xformOp:translate').Get()
            if (size is None or any(
                    abs(float(size[index]) - FINGERTIP_PAD_SIZE_M[index]) >
                    1.0e-6 for index in range(3))):
                errors.append({
                    'kind': 'fingertip-contact-proxy-size',
                    'tip': str(tip), 'actual': list(size or [])})
            if (center is None or any(
                    abs(float(center[index]) -
                        FINGERTIP_PAD_CENTER_M[index]) > 1.0e-6
                    for index in range(3))):
                errors.append({
                    'kind': 'fingertip-contact-proxy-center',
                    'tip': str(tip), 'actual': list(center or [])})
            pad_targets = pad_prim.GetRelationship(
                'material:binding:physics').GetTargets()
            if pad_targets != [material_path]:
                errors.append({
                    'kind': 'fingertip-contact-proxy-material',
                    'tip': str(tip),
                    'actual': [str(target) for target in pad_targets]})

    leader_path = Sdf.Path(f'{ROOT}/Physics/finger_joint')
    leader = stage.GetPrimAtPath(leader_path)
    leader_drive = UsdPhysics.DriveAPI(leader, 'angular')
    drive = {
        'maximum_torque_nm': leader_drive.GetMaxForceAttr().Get(),
        'stiffness': leader_drive.GetStiffnessAttr().Get(),
        'damping': leader_drive.GetDampingAttr().Get(),
    }
    expected_drive = {
        'maximum_torque_nm': PROVISIONAL_MAX_TORQUE_NM,
        'stiffness': PROVISIONAL_STIFFNESS,
        'damping': PROVISIONAL_DAMPING,
    }
    for name, expected in expected_drive.items():
        actual = drive[name]
        if actual is None or abs(float(actual) - expected) > 1.0e-6:
            errors.append({
                'kind': 'leader-drive', 'property': name,
                'expected': expected, 'actual': actual})
    armature = PhysxSchema.PhysxJointAPI(leader).GetArmatureAttr().Get()
    if armature is None or not math.isclose(armature, PRIMARY_ARMATURE_KG_M2, rel_tol=1e-6):
        errors.append({'kind': 'leader-armature', 'actual': armature,
                       'expected': PRIMARY_ARMATURE_KG_M2})

    solver = {
        'position_iterations': root.GetAttribute(
            'physxArticulation:solverPositionIterationCount').Get(),
        'velocity_iterations': root.GetAttribute(
            'physxArticulation:solverVelocityIterationCount').Get(),
    }
    if solver != {
            'position_iterations': SOLVER_POSITION_ITERATIONS,
            'velocity_iterations': SOLVER_VELOCITY_ITERATIONS}:
        errors.append({'kind': 'articulation-solver', 'actual': solver})

    for name, gearing in PHYSX_MIMICS.items():
        follower = stage.GetPrimAtPath(f'{ROOT}/Physics/{name}')
        lower = follower.GetAttribute('physics:lowerLimit').Get()
        upper = follower.GetAttribute('physics:upperLimit').Get()
        if (lower is None or upper is None or
                not float(lower) < float(upper)):
            errors.append({
                'kind': 'mimic-finite-limits', 'joint': name,
                'lower': lower, 'upper': upper})
        if follower.HasAPI(UsdPhysics.DriveAPI, 'angular'):
            errors.append({'kind': 'independent-follower-drive',
                           'joint': name})
        if not follower.HasAPI(PhysxSchema.PhysxMimicJointAPI, 'rotZ'):
            errors.append({'kind': 'missing-mimic-schema', 'joint': name})
            continue
        reference = follower.GetRelationship(
            'physxMimicJoint:rotZ:referenceJoint').GetTargets()
        actual_gearing = follower.GetAttribute(
            'physxMimicJoint:rotZ:gearing').Get()
        if reference != [leader_path] or actual_gearing != gearing:
            errors.append({
                'kind': 'mimic-definition', 'joint': name,
                'reference': [str(target) for target in reference],
                'gearing': actual_gearing})
        damping = follower.GetAttribute('physxMimicJoint:rotZ:dampingRatio').Get()
        if damping is None or not math.isclose(damping, MIMIC_DAMPING_RATIO, rel_tol=1e-6):
            errors.append({'kind': 'mimic-damping', 'joint': name, 'actual': damping})

    if errors:
        raise RuntimeError(
            'authored RG6 semantic contract differs from the RG family '
            f'policy: {json.dumps(errors, sort_keys=True)}')
    return {
        'rigid_links': sorted(rigid),
        'physics_joints': sorted(joints),
        'articulation_root': ROOT,
        'collision_mesh_approximations': collision_approximations,
        'physics_variants': sorted(variant_names),
        'mimic_joints': sorted(MIMICS),
        'filtered_pair_policy': 'all-self-pairs-except-tip-to-tip',
        'fingertip_material': friction,
        'leader_drive': drive,
        'articulation_solver': solver,
        'custom_finger_anchors': {
            'left': f'{ROOT}/Geometry/left_finger_base_link',
            'right': f'{ROOT}/Geometry/right_finger_base_link',
        },
    }


def main() -> int:
    """Import, normalize, and family-check an RG6 asset package."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xacro', type=Path, required=True)
    parser.add_argument('--package-root', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--family-reference', type=Path, required=True)
    args, _ = parser.parse_known_args()
    report = {'schema_version': 1, 'model': MODEL, 'status': 'failed'}
    app = None
    try:
        if args.output_dir.exists() and any(args.output_dir.iterdir()):
            raise RuntimeError(
                f'output directory must be absent or empty: '
                f'{args.output_dir}')
        with tempfile.TemporaryDirectory(
                prefix='onrobot_rg6_authoring_') as temp:
            temporary = Path(temp)
            urdf = temporary / 'onrobot_rg6.urdf'
            imported = temporary / 'imported'
            transformed = temporary / 'transformed' / 'onrobot_rg6'
            _expand_xacro(args.xacro.resolve(), urdf)
            _make_physical_only_urdf(urdf)
            report['import_urdf_contract'] = _validate_physical_urdf(urdf)

            from isaacsim import SimulationApp
            app = SimulationApp({'headless': True, 'width': 1280,
                                 'height': 720})
            from isaacsim.asset.importer.urdf import (
                URDFImporter, URDFImporterConfig)
            from isaacsim.asset.importer.utils import (
                run_asset_transformer_profile)
            from isaacsim.asset.transformer.rules import DEFAULT_PROFILE_PATH
            from isaacsim.core.version import get_version
            from pxr import Usd

            report['isaac_sim_version'] = get_version()[0]
            config = URDFImporterConfig(
                urdf_path=str(urdf), usd_path=str(imported),
                merge_fixed_joints=False, merge_mesh=False, debug_mode=True,
                collision_from_visuals=False,
                collision_type='Convex Decomposition',
                allow_self_collision=True,
                ros_package_paths=[{'name': 'onrobot_rg6',
                                    'path': str(args.package_root.resolve())}],
                robot_type='End Effector', fix_base=True,
                joint_drive_type='force', joint_target_type='position',
                override_joint_stiffness=PROVISIONAL_STIFFNESS,
                override_joint_damping=PROVISIONAL_DAMPING,
                run_asset_transformer=False,
                run_multi_physics_conversion=True)
            generated_raw = Path(URDFImporter(config).import_urdf())
            raw_stage = Usd.Stage.Open(str(generated_raw))
            if raw_stage is None:
                raise RuntimeError(
                    f'could not open raw imported USD: {generated_raw}')
            report['hierarchy_normalization'] = (
                _normalize_import_hierarchy(raw_stage))
            raw_stage = None
            run_asset_transformer_profile(
                input_stage_path=str(generated_raw),
                output_package_root=str(transformed),
                profile_json_path=DEFAULT_PROFILE_PATH)
            entrypoint = transformed / 'onrobot_rg6.usda'
            physics_dir = transformed / 'payloads/Physics'
            report['articulation_layer_routing'] = (
                _clear_imported_articulation_roots(transformed))
            _author_physics(physics_dir / 'physics.usda')
            _author_physx(
                physics_dir / 'physx.usda', './physics.usda', False)
            _author_physx(physics_dir / 'physx_parallel_grip.usda',
                          './physics.usda', False)
            _author_physx(
                physics_dir / 'physx_parallel_grip_tip_contact.usda',
                './physics.usda', True)
            _canonical_entrypoint(entrypoint)
            report['stripped_generator_documentation'] = (
                _strip_generator_documentation(transformed))
            report['composed_articulation_cleanup'] = (
                _remove_composed_geometry_articulation_roots(entrypoint))
            from asset_visual_materials import author_housing_appearance
            report['housing_appearance'] = author_housing_appearance(entrypoint, 'rg6')
            report['semantic_contract'] = _validate_authored_stage(entrypoint)
            report['rg_family_contract'] = _validate_rg_family(
                transformed, args.family_reference.resolve())
            if args.output_dir.exists():
                args.output_dir.rmdir()
            args.output_dir.parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(transformed, args.output_dir)
            report['asset'] = str(
                (args.output_dir / 'onrobot_rg6.usda').resolve())
            report['status'] = 'authored-unqualified'
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(
            json.dumps(report, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')
        print(json.dumps(report, indent=2, sort_keys=True), flush=True)
        code = 0 if report['status'] == 'authored-unqualified' else 1
        if app is not None:
            app.close(exit_code=code)
    return code


if __name__ == '__main__':
    sys.exit(main())

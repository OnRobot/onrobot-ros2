#!/usr/bin/env python3
"""Validate a checked-in gripper USD against its ROS coordinate contract."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml
from isaac_model_contract import asset_repository_root


SUPPORTED_MODELS = ('2fg7', '2fg14', 'rg2', 'rg6')


def _package_root() -> Path:
    source_root = Path(__file__).resolve().parents[1]
    if (source_root / 'config' / '2fg7_asset_contract.json').is_file():
        return source_root
    install_root = Path(__file__).resolve().parents[2]
    return install_root / 'share' / 'onrobot_gripper_isaac'


def _share_path(package: str, source_sibling: Path) -> Path:
    if source_sibling.is_dir():
        return source_sibling
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(package))
    except (ImportError, LookupError):
        raise RuntimeError(
            f'cannot locate {package}; build/source the workspace or pass '
            'its path')


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _joint_block(text: str, declaration: str) -> str:
    start = text.find(declaration)
    if start < 0:
        raise AssertionError(f'missing USD declaration: {declaration}')
    brace = text.find('{', start)
    if brace < 0:
        raise AssertionError(
            f'missing block for USD declaration: {declaration}')
    depth = 0
    for index in range(brace, len(text)):
        if text[index] == '{':
            depth += 1
        elif text[index] == '}':
            depth -= 1
            if depth == 0:
                return text[brace + 1:index]
    raise AssertionError(f'unclosed USD block: {declaration}')


def _declaration_header(text: str, declaration: str) -> str:
    """Return a USD prim declaration including its metadata, before `{`."""
    start = text.find(declaration)
    if start < 0:
        raise AssertionError(f'missing USD declaration: {declaration}')
    brace = text.find('{', start)
    if brace < 0:
        raise AssertionError(
            f'missing block for USD declaration: {declaration}')
    return text[start:brace]


def _number(text: str, attribute: str) -> float:
    match = re.search(
        rf'\b{re.escape(attribute)}\s*=\s*([-+0-9.eE]+)', text)
    if match is None:
        raise AssertionError(f'missing numeric USD attribute: {attribute}')
    return float(match.group(1))


def _vector3(text: str, attribute: str) -> tuple[float, float, float]:
    number = r'([-+0-9.eE]+)'
    match = re.search(
        rf'\b{re.escape(attribute)}\s*=\s*\(\s*{number}\s*,\s*'
        rf'{number}\s*,\s*{number}\s*\)', text)
    if match is None:
        raise AssertionError(f'missing vector USD attribute: {attribute}')
    return tuple(float(value) for value in match.groups())


def _assert_close(actual: float, expected: float, label: str) -> None:
    if not (math.isfinite(actual) and math.isfinite(expected)) or abs(actual - expected) > 1e-9:
        raise AssertionError(f'{label}: expected {expected}, got {actual}')


def _assert_near(actual: float, expected: float, tolerance: float,
                 label: str) -> None:
    if (not all(math.isfinite(value) for value in (actual, expected, tolerance))
            or tolerance < 0 or abs(actual - expected) > tolerance):
        raise AssertionError(
            f'{label}: expected {expected} +/- {tolerance}, got {actual}')


def _assert_vector_close(actual, expected, label: str) -> None:
    if len(actual) != 3 or len(expected) != 3:
        raise AssertionError(f'{label}: expected three components')
    for axis, (actual_value, expected_value) in enumerate(
            zip(actual, expected)):
        _assert_close(actual_value, float(expected_value),
                      f'{label}[{axis}]')


def _validate_reference_finger_collisions(
        contract: dict, base: str, instances: str) -> None:
    collision_model = contract.get(
        'reference_fingers', {}).get('collision_model')
    if collision_model is None:
        return
    if contract['model'] != '2fg14':
        raise AssertionError(
            'explicit standard-fingertip collision contract is currently '
            'defined only for 2FG14')
    if collision_model['source'] != (
            'standard-fingertip-primitive-collision'):
        raise AssertionError('unknown reference-finger collision source')

    imported_mesh = _joint_block(
        instances,
        f'def Xform "{collision_model["disabled_imported_mesh"]}"')
    if (_number(imported_mesh, 'physics:collisionEnabled') != 0.0 or
            'PhysicsMeshCollisionAPI' not in imported_mesh):
        raise AssertionError(
            'the imported 2FG14 fingertip mesh collision must be disabled')

    for side in ('left', 'right'):
        link = _joint_block(
            base, f'def Xform "{side}_fingertip_link"')
        for shape_name in ('upper_contact', 'lower_finger'):
            shape = _joint_block(link, f'def Cube "{shape_name}"')
            expected = collision_model[shape_name]
            if ('token purpose = "guide"' not in shape or
                    _number(shape, 'physics:collisionEnabled') != 1.0):
                raise AssertionError(
                    f'{side} {shape_name} is not an enabled hidden collider')
            _assert_vector_close(
                _vector3(shape, 'xformOp:translate'),
                expected['center_m'], f'{side} {shape_name} center')
            _assert_vector_close(
                _vector3(shape, 'xformOp:scale'),
                expected['size_m'], f'{side} {shape_name} size')


def _validate_references(asset_root: Path, layers: dict[str, str]) -> None:
    known = {Path(relative) for relative in layers}
    for relative in layers:
        path = asset_root / relative
        if path.suffix != '.usda':
            continue
        text = path.read_text(encoding='utf-8')
        for reference in re.findall(r'@([^@]+)@', text):
            if '://' in reference:
                raise AssertionError(
                    f'{relative}: external URL is not release-portable: '
                    f'{reference}')
            target = (path.parent / reference).resolve()
            if not target.is_file():
                raise AssertionError(
                    f'{relative}: unresolved layer reference {reference}')
            target_relative = target.relative_to(asset_root.resolve())
            if target_relative not in known:
                raise AssertionError(
                    f'{relative}: referenced layer is absent from manifest: '
                    f'{target_relative}')


def _validate_release_portability(asset_root: Path,
                                  layers: dict[str, str]) -> None:
    forbidden_paths = ('C:\\', '/home/', '/Users/', '/tmp/')
    for relative in layers:
        path = asset_root / relative
        if path.suffix != '.usda':
            continue
        text = path.read_text(encoding='utf-8')
        for private_path in forbidden_paths:
            if private_path in text:
                raise AssertionError(
                    f'{relative}: USD contains a generator-local absolute '
                    f'path: {private_path}')


def _validate_rg_usd(entry: str, physics: str, physx: str,
                     default_physx: str, robot: str, instances: str, usd: dict,
                     contract: dict) -> None:
    """Validate the RG angular linkage and intentional tip-contact policy."""
    driven = _joint_block(
        physics, f'def PhysicsRevoluteJoint "{usd["actuated_joint"]}"')
    _assert_near(math.radians(_number(driven, 'physics:lowerLimit')),
                 usd['lower_limit'], 1e-7, 'USD driven lower limit')
    _assert_near(math.radians(_number(driven, 'physics:upperLimit')),
                 usd['upper_limit'], 1e-7, 'USD driven upper limit')
    _assert_close(_number(driven, 'drive:angular:physics:maxForce'),
                  usd['maximum_drive_torque_nm'],
                  'USD drive maximum torque')
    _assert_close(_number(driven, 'drive:angular:physics:stiffness'),
                  contract['drive_tuning']['stiffness'], 'USD stiffness')
    _assert_close(_number(driven, 'drive:angular:physics:damping'),
                  contract['drive_tuning']['damping'], 'USD damping')

    driven_physx = _joint_block(
        physx, f'over "{usd["actuated_joint"]}"')
    _assert_near(math.radians(
        _number(driven_physx, 'physxJoint:maxJointVelocity')),
        usd['maximum_velocity'], 1e-6, 'USD driven maximum velocity')
    tuning = contract['drive_tuning']
    _assert_close(_number(driven_physx, 'physxJoint:armature'),
                  tuning['armature_kg_m2'], 'USD driven armature')
    default_driven = _joint_block(default_physx, f'over "{usd["actuated_joint"]}"')
    _assert_close(_number(default_driven, 'physxJoint:armature'),
                  tuning['armature_kg_m2'], 'USD default driven armature')
    _assert_near(math.radians(_number(default_driven, 'physxJoint:maxJointVelocity')),
                 usd['maximum_velocity'], 1e-6, 'USD default driven maximum velocity')
    for name in ('stiffness', 'damping'):
        _assert_near(tuning[name] * 180 / math.pi, tuning[name + '_si'],
                     1e-5, f'RG {name} angular unit conversion')
    expected_reference = (
        f'<{usd["articulation_root"]}/Physics/{usd["actuated_joint"]}>')
    mimic_tuning = contract['mimic_constraint_tuning']
    for follower in usd['mimic_joints']:
        block = _joint_block(physx, f'over "{follower["name"]}"')
        if expected_reference not in block:
            raise AssertionError(
                f'USD mimic {follower["name"]} has the wrong leader')
        _assert_close(
            _number(block,
                    f'physxMimicJoint:{follower["axis"]}:gearing'),
            follower['gearing'], f'USD {follower["name"]} gearing')
        _assert_close(
            _number(
                block,
                f'physxMimicJoint:{follower["axis"]}:naturalFrequency'),
            mimic_tuning['natural_frequency_hz'],
            f'USD {follower["name"]} mimic natural frequency')
        _assert_close(
            _number(
                block,
                f'physxMimicJoint:{follower["axis"]}:dampingRatio'),
            mimic_tuning['damping_ratio'],
            f'USD {follower["name"]} mimic damping ratio')
        if 'PhysicsDriveAPI' in block:
            raise AssertionError(
                f'USD mimic {follower["name"]} has an independent drive')

    authored_roots = re.findall(
        r'prepend apiSchemas\s*=\s*\[[^\]]*'
        r'"PhysicsArticulationRootAPI"', physics)
    if len(authored_roots) != 1:
        raise AssertionError(
            'RG physics layer must define exactly one articulation root')
    if usd['articulation_root'] != f'/{usd["default_prim"]}':
        raise AssertionError('RG articulation root must be the asset root')
    collision = contract['collision_model']
    if f'string Physics = "{collision["default_variant"]}"' not in entry:
        raise AssertionError('RG entry point selects the wrong collision mode')
    self_collision = _number(physx, 'physxArticulation:enabledSelfCollisions')
    _assert_close(self_collision, 1.0 if collision['self_collision'] else 0.0,
                  'RG self-collision policy')
    default_self_collision = _number(
        default_physx, 'physxArticulation:enabledSelfCollisions')
    _assert_close(
        default_self_collision,
        1.0 if collision['default_self_collision'] else 0.0,
        'RG default self-collision policy')
    if ('physics:approximation = "convexHull"' in instances or
            'physics:approximation = "convexDecomposition"' not in
            instances):
        raise AssertionError(
            'RG collision meshes must use convex decomposition')
    material = _joint_block(
        physics, 'def Material "fingertip_physics_material"')
    _assert_close(_number(material, 'physics:staticFriction'),
                  collision['fingertip_static_friction'],
                  'RG fingertip static friction')
    _assert_close(_number(material, 'physics:dynamicFriction'),
                  collision['fingertip_dynamic_friction'],
                  'RG fingertip dynamic friction')
    _assert_close(_number(material, 'physics:restitution'),
                  collision['fingertip_restitution'],
                  'RG fingertip restitution')
    right_tip_path = f'/{usd["default_prim"]}/Geometry/right_finger_tip_link'
    left_tip_path = f'/{usd["default_prim"]}/Geometry/left_finger_tip_link'
    right_tip = _joint_block(physics, 'over "right_finger_tip_link"')
    left_tip = _joint_block(physics, 'over "left_finger_tip_link"')
    material_path = (
        f'</{usd["default_prim"]}/Physics/fingertip_physics_material>')
    for side, tip in (('right', right_tip), ('left', left_tip)):
        declaration = f'over "{side}_finger_tip_link"'
        header = _declaration_header(physics, declaration)
        if 'MaterialBindingAPI' not in header:
            raise AssertionError(
                f'RG {side} fingertip omits MaterialBindingAPI')
        if 'custom rel material:binding:physics' in tip:
            raise AssertionError(
                f'RG {side} fingertip uses a custom physics binding')
        if f'rel material:binding:physics = {material_path}' not in tip:
            raise AssertionError(
                f'RG {side} fingertip omits its physics material binding')
    contact_proxy = collision.get('fingertip_contact_proxy')
    if contact_proxy is not None:
        if (contact_proxy.get('type') != 'closed-box' or
                contact_proxy.get('source') !=
                'visible-pad-contact-face-with-rigid-fingertip-backing'):
            raise AssertionError('unknown RG fingertip contact proxy')
        if physics.count('def Cube "payload_contact_collision"') != 2:
            raise AssertionError(
                'RG contact proxy must define one closed pad per fingertip')
        if physics.count(
                f'over "{contact_proxy["disabled_imported_collision"]}"') != 2:
            raise AssertionError(
                'RG imported fingertip collisions are not explicitly owned')
        if physics.count('active = false') < 2:
            raise AssertionError(
                'RG non-manifold imported fingertip collisions remain active')
        size = tuple(float(value) for value in contact_proxy['size_m'])
        center = tuple(float(value) for value in contact_proxy['center_m'])
        contact_face = float(contact_proxy['contact_face_local_z_m'])
        if abs(center[2] + size[2] / 2.0 - contact_face) > 1.0e-9:
            raise AssertionError(
                'RG contact proxy backing moved the visible contact face')
        size_text = ', '.join(f'{value:g}' for value in size)
        center_text = ', '.join(f'{value:g}' for value in center)
        if physics.count(f'float3 xformOp:scale = ({size_text})') != 2:
            raise AssertionError('RG fingertip contact proxy size differs')
        if physics.count(
                f'double3 xformOp:translate = ({center_text})') != 2:
            raise AssertionError('RG fingertip contact proxy center differs')
        for side, tip in (('right', right_tip), ('left', left_tip)):
            pad = _joint_block(tip, 'def Cube "payload_contact_collision"')
            pad_header = _declaration_header(
                tip, 'def Cube "payload_contact_collision"')
            if ('PhysicsCollisionAPI' not in pad_header or
                    f'rel material:binding:physics = {material_path}'
                    not in pad):
                raise AssertionError(
                    f'RG {side} contact proxy lacks collision/material API')
    if collision['tip_to_tip_contact']:
        if f'<{left_tip_path}>' in right_tip:
            raise AssertionError(
                'RG right fingertip filters the required left-tip contact')
        if f'<{right_tip_path}>' in left_tip:
            raise AssertionError(
                'RG left fingertip filters the required right-tip contact')
    for side in ('right', 'left'):
        fixed_joint_path = contract['custom_fingers'][f'{side}_fixed_joint']
        anchor_path = contract['custom_fingers'][f'{side}_anchor']
        fixed_joint_name = fixed_joint_path.rsplit('/', 1)[-1]
        fixed_joint = _joint_block(
            physics, f'def PhysicsFixedJoint "{fixed_joint_name}"')
        expected_tip = (
            f'/{usd["default_prim"]}/Geometry/{side}_finger_tip_link')
        if f'<{anchor_path}>' not in fixed_joint:
            raise AssertionError(
                f'RG {side} fingertip joint does not use its finger anchor')
        if f'<{expected_tip}>' not in fixed_joint:
            raise AssertionError(
                f'RG {side} fingertip joint does not own its fingertip body')
    for name in (usd['actuated_joint'], 'right_finger_tip_joint',
                 'left_finger_tip_joint', 'right_finger_base_link',
                 'left_finger_base_link'):
        if name not in robot:
            raise AssertionError(f'RG robot schema omits {name}')


def _validate_usd(package_root: Path, contract: dict) -> None:
    package_root = asset_repository_root(package_root)
    layers = contract['layers']
    for relative, expected_hash in layers.items():
        path = package_root / relative
        if not path.is_file():
            raise AssertionError(f'missing asset layer: {relative}')
        actual_hash = _sha256(path)
        if actual_hash != expected_hash:
            raise AssertionError(
                f'asset layer checksum changed: {relative}; update the asset '
                'contract intentionally after review')

    _validate_references(package_root, layers)
    _validate_release_portability(package_root, layers)
    entry = (package_root / contract['asset_entrypoint']).read_text(
        encoding='utf-8')
    asset_directory = Path(contract['asset_entrypoint']).parent
    physics_path = package_root / asset_directory / (
        'payloads/Physics/physics.usda')
    physx_layer = (
        'payloads/Physics/physx_parallel_grip_tip_contact.usda'
        if contract['model'] in ('rg2', 'rg6')
        else 'payloads/Physics/physx.usda')
    physx_path = package_root / asset_directory / physx_layer
    physics = physics_path.read_text(encoding='utf-8')
    physx = physx_path.read_text(encoding='utf-8')
    default_physx = (
        package_root / asset_directory /
        'payloads/Physics/physx_parallel_grip.usda').read_text(
            encoding='utf-8') if contract['model'] in ('rg2', 'rg6') else physx
    robot = (package_root / asset_directory / 'payloads/robot.usda').read_text(
        encoding='utf-8')
    instances = (
        package_root / asset_directory / 'payloads/instances.usda').read_text(
            encoding='utf-8')
    base = (
        package_root / asset_directory / 'payloads/base.usda').read_text(
            encoding='utf-8')
    usd = contract['usd']

    if f'defaultPrim = "{usd["default_prim"]}"' not in entry:
        raise AssertionError('USD defaultPrim does not match the manifest')
    _assert_close(_number(entry, 'metersPerUnit'), 1.0, 'USD metersPerUnit')
    if 'string Physics = "physx"' not in entry:
        if contract['model'] not in ('rg2', 'rg6'):
            raise AssertionError(
                'the checked-in USD must select the PhysX variant')
    if contract['model'] in ('rg2', 'rg6'):
        _validate_rg_usd(
            entry, physics, physx, default_physx, robot, instances, usd,
            contract)
        return
    if ('physics:approximation = "convexHull"' in instances or
            'physics:approximation = "convexDecomposition"' not in
            instances):
        raise AssertionError(
            '2FG collision meshes must use convex decomposition so concave '
            'finger geometry is not filled by an invisible convex hull')
    if contract['model'] == '2fg14':
        collision_mesh_count = instances.count('PhysicsMeshCollisionAPI')
        guide_purpose_count = instances.count('token purpose = "guide"')
        if (collision_mesh_count == 0 or
                guide_purpose_count != collision_mesh_count):
            raise AssertionError(
                '2FG14 collision-only meshes must use guide purpose so they '
                'cannot appear as rendered geometry')
    _validate_reference_finger_collisions(contract, base, instances)

    driven = _joint_block(
        physics, f'def PhysicsPrismaticJoint "{usd["actuated_joint"]}"')
    follower = _joint_block(
        physics, f'def PhysicsPrismaticJoint "{usd["follower_joint"]}"')
    mimic = _joint_block(
        physx, f'over "{usd["follower_joint"]}"')
    driven_physx = _joint_block(
        physx, f'over "{usd["actuated_joint"]}"')
    drive_override = _joint_block(
        entry, f'over "{usd["actuated_joint"]}"')

    _assert_close(_number(driven, 'physics:lowerLimit'),
                  usd['lower_limit_m'], 'USD driven lower limit')
    _assert_close(_number(driven, 'physics:upperLimit'),
                  usd['upper_limit_m'], 'USD driven upper limit')
    _assert_close(_number(driven, 'drive:linear:physics:maxForce'),
                  usd['maximum_drive_force_n'],
                  'USD drive maximum force')
    physx_drive_force = contract['drive_tuning'].get(
        'physx_single_drive_force_n')
    if physx_drive_force is not None:
        nominal_force = contract['drive_tuning'][
            'nominal_grip_force_reference_n']
        _assert_close(
            nominal_force, usd['maximum_drive_force_n'],
            'nominal grip-force reference')
        # Validate the authored ceiling, not an unproven universal factor
        # between a datasheet grip rating and a generalized actuator force.
        _assert_close(
            _number(driven_physx, 'drive:linear:physics:maxForce'),
            physx_drive_force, 'PhysX single-drive maximum force')
    _assert_close(_number(drive_override, 'drive:linear:physics:stiffness'),
                  contract['drive_tuning']['stiffness'], 'USD stiffness')
    _assert_close(_number(drive_override, 'drive:linear:physics:damping'),
                  contract['drive_tuning']['damping'], 'USD damping')
    _assert_close(_number(driven_physx, 'physxJoint:armature'),
                  contract['drive_tuning']['armature_kg'],
                  'USD driven linear armature')
    _assert_close(_number(driven_physx, 'physxJoint:maxJointVelocity'),
                  usd['maximum_velocity_m_s'],
                  'USD driven maximum velocity')
    _assert_close(_number(mimic, 'physxJoint:maxJointVelocity'),
                  usd['maximum_velocity_m_s'],
                  'USD follower maximum velocity')
    _assert_close(_number(follower, 'physics:lowerLimit'),
                  usd['lower_limit_m'], 'USD follower lower limit')
    _assert_close(_number(follower, 'physics:upperLimit'),
                  usd['upper_limit_m'], 'USD follower upper limit')

    reference = (
        f'rel physxMimicJoint:rotX:referenceJoint = '
        f'<{usd["articulation_root"].rsplit("/", 1)[0]}/'
        f'{usd["actuated_joint"]}>')
    if reference not in mimic:
        raise AssertionError('USD mimic does not reference the driven joint')
    # PhysX expresses a positive multiplier using a negative gearing value.
    _assert_close(_number(mimic, 'physxMimicJoint:rotX:gearing'),
                  -usd['mimic_multiplier'], 'USD mimic gearing')
    if 'PhysicsDriveAPI:linear' in mimic:
        raise AssertionError(
            'the mimic follower must not have an independent drive')
    if not re.search(
            r'def\s+PhysicsFixedJoint\s+"root_joint"\s*\([^)]*'
            r'PhysicsArticulationRootAPI[^)]*\)', physics, re.DOTALL):
        raise AssertionError('USD root_joint is not an articulation root')
    if len(re.findall(r'PhysicsArticulationRootAPI', physics)) != 1:
        raise AssertionError(
            'USD physics layer must define exactly one Physics articulation '
            'root')
    expected_root = f'/{usd["default_prim"]}/Physics/root_joint'
    if usd['articulation_root'] != expected_root:
        raise AssertionError(
            f'USD articulation root must be {expected_root}')

    required_robot_prims = (
        usd['actuated_joint'], usd['follower_joint'], 'base_link',
        'right_finger_base_link', 'left_finger_base_link')
    for name in required_robot_prims:
        if name not in robot:
            raise AssertionError(f'robot schema omits {name}')

    combined = '\n'.join((entry, physics, physx, robot, base))
    forbidden_names = (
        'virtual_stroke_link', 'mechanism_stroke', 'grip_stroke')
    for forbidden in forbidden_names:
        if forbidden in combined:
            raise AssertionError(
                f'USD contains non-articulation ROS coordinate {forbidden}')


def _validate_ros(
        contract: dict, ros_root: Path, description_root: Path) -> None:
    model = contract['model']
    xacro_path = ros_root / f'urdf/onrobot_{model}_realmesh_macro.xacro'
    xacro_text = xacro_path.read_text(encoding='utf-8')
    robot = ET.fromstring(xacro_text)
    namespace = {'xacro': 'http://www.ros.org/wiki/xacro'}
    arguments = {
        item.attrib['name']: item.attrib['default']
        for item in robot.findall('xacro:arg', namespace)
    }
    # ros2_control declarations reuse the joint names but are not kinematic
    # URDF joints. Select only joints that own parent/child relationships.
    joints = {
        joint.attrib['name'].removeprefix('${prefix}'): joint
        for joint in robot.findall('.//joint')
        if ('name' in joint.attrib and joint.find('parent') is not None and
            joint.find('child') is not None)
    }
    ros = contract['ros']
    task = joints[ros['task_joint']]
    mechanism = joints[ros['mechanism_joint']]
    if model in ('rg2', 'rg6'):
        # RG macros deliberately select a CAD linkage range by backend.  The
        # raw macro therefore contains a symbolic xacro property and cannot
        # prove the Isaac URDF contract by XML parsing alone.  Expand the
        # actual model entry point with the same backend selection used by the
        # Isaac bridge, then inspect the resulting physical joints.
        xacro = shutil.which('xacro')
        entry = ros_root / f'urdf/realmesh_onrobot_{model}.urdf.xacro'
        if xacro is None or not entry.is_file():
            raise AssertionError(
                'cannot expand RG Isaac URDF: xacro or model entry point is '
                'unavailable')
        expanded = subprocess.run(
            [xacro, str(entry), 'backend:=isaac'],
            capture_output=True, text=True, check=False)
        if expanded.returncode != 0:
            raise AssertionError(
                f'RG Isaac URDF expansion failed: {expanded.stderr.strip()}')
        expanded_robot = ET.fromstring(expanded.stdout)
        joints = {
            joint.attrib['name']: joint
            for joint in expanded_robot.findall('.//joint')
            if (joint.find('parent') is not None and
                joint.find('child') is not None)
        }
        task = joints[ros['task_joint']]
        mechanism = joints[ros['mechanism_joint']]

        def rg_limit(joint, attribute):
            return float(joint.find('limit').attrib[attribute])

        _assert_close(rg_limit(task, 'lower'), ros['task_minimum_m'],
                      'ROS task lower limit')
        _assert_close(rg_limit(task, 'upper'), ros['task_maximum_m'],
                      'ROS task upper limit')
        _assert_close(rg_limit(mechanism, 'lower'),
                      ros['mechanism_minimum_rad'],
                      'ROS mechanism lower limit')
        _assert_close(rg_limit(mechanism, 'upper'),
                      ros['mechanism_maximum_rad'],
                      'ROS mechanism upper limit')
        profile_path = description_root / (
            f'config/finger_profiles/{model}_standard.yaml')
        profile = yaml.safe_load(profile_path.read_text(encoding='utf-8'))
        task_profile = profile['task_coordinate']
        mechanism_profile = profile['mechanism_coordinate']
        _assert_close(task_profile['minimum'], ros['task_minimum_m'],
                      'profile task minimum')
        _assert_close(task_profile['maximum'], ros['task_maximum_m'],
                      'profile task maximum')
        _assert_close(mechanism_profile['minimum'],
                      ros['mechanism_minimum_rad'],
                      'profile mechanism minimum')
        _assert_close(mechanism_profile['maximum'],
                      ros['mechanism_maximum_rad'],
                      'profile mechanism maximum')
        if (task_profile['mapping_source'] != ros['mapping_source'] or
                mechanism_profile['dimension'] != 'angular' or
                mechanism_profile['unit'] != 'rad'):
            raise AssertionError(
                'RG finger profile coordinate contract differs')
        return
    follower = joints[contract['usd']['follower_joint']]

    def limit(joint, attribute):
        value = joint.find('limit').attrib[attribute]
        match = re.fullmatch(r'\$\(arg ([^)]+)\)', value)
        return float(arguments[match.group(1)] if match else value)

    _assert_close(limit(task, 'lower'),
                  ros['task_minimum_m'], 'ROS task lower limit')
    _assert_close(limit(task, 'upper'),
                  ros['task_maximum_m'], 'ROS task upper limit')
    _assert_close(limit(mechanism, 'lower'),
                  contract['usd']['lower_limit_m'],
                  'ROS physical lower limit')
    _assert_close(limit(mechanism, 'upper'),
                  contract['usd']['upper_limit_m'],
                  'ROS physical upper limit')
    mimic = follower.find('mimic')
    if (mimic is None or
            mimic.attrib['joint'] != '${prefix}' + ros['mechanism_joint']):
        raise AssertionError('ROS follower does not mimic finger_stroke')
    _assert_close(float(mimic.attrib['multiplier']),
                  contract['usd']['mimic_multiplier'],
                  'ROS mimic multiplier')
    if ('virtual_stroke_link' in xacro_text or
            'mechanism_stroke' in xacro_text):
        raise AssertionError(
            'obsolete 2FG virtual mechanism remains in ROS URDF')

    profile_path = description_root / (
        f'config/finger_profiles/{model}_standard.yaml')
    profile = yaml.safe_load(profile_path.read_text(encoding='utf-8'))
    if (profile['name'] != f'{model}_standard' or
            profile['compatible_models'] != [model]):
        raise AssertionError(f'wrong {model} finger profile')
    task_profile = profile['task_coordinate']
    mechanism_profile = profile['mechanism_coordinate']
    _assert_close(task_profile['minimum'], ros['task_minimum_m'],
                  'profile task minimum')
    _assert_close(task_profile['maximum'], ros['task_maximum_m'],
                  'profile task maximum')
    _assert_close(mechanism_profile['minimum'], ros['mechanism_minimum_m'],
                  'profile mechanism minimum')
    _assert_close(mechanism_profile['maximum'], ros['mechanism_maximum_m'],
                  'profile mechanism maximum')
    if task_profile['mapping_source'] != ros['mapping_source']:
        raise AssertionError(
            'profile mapping source differs from asset contract')


def _validate_dynamic_contact_object_placement(
        config: dict, item: dict) -> dict:
    geometry = config['reference_geometry_m']
    center_x, center_y, center_z = (
        float(value) for value in item['center_m'])
    size_x, size_y, size_z = (
        float(value) for value in item['size_m'])
    center_limit = float(geometry['maximum_center_offset_xy'])
    if abs(center_x) > center_limit or abs(center_y) > center_limit:
        raise AssertionError(
            'dynamic contact object is not centered on the fingers')

    object_min_z = center_z - size_z / 2.0
    object_max_z = center_z + size_z / 2.0
    base_clearance = (
        object_min_z - float(geometry['base_collision_max_z']))
    if base_clearance < float(geometry['minimum_base_clearance']):
        raise AssertionError(
            'dynamic contact object overlaps or is too close to the gripper '
            'base')

    contact_z_min, contact_z_max = (
        float(value) for value in geometry['contact_face_z_range'])
    contact_face_z_overlap = max(
        0.0,
        min(object_max_z, contact_z_max) -
        max(object_min_z, contact_z_min))
    if contact_face_z_overlap < float(
            geometry['minimum_contact_face_z_overlap']):
        raise AssertionError(
            'dynamic contact object misses the vertical contact faces')

    object_half_x = size_x / 2.0
    contact_face_inner_x = float(
        geometry['contact_face_inner_x_at_zero'])
    release_clearance = (
        contact_face_inner_x + float(item['release_target_m']) -
        object_half_x)
    if release_clearance < float(geometry['minimum_release_clearance']):
        raise AssertionError(
            'dynamic contact object does not clear the released fingertips')
    grasp_interference = (
        object_half_x - contact_face_inner_x -
        float(item['grasp_target_m']))
    if grasp_interference < float(geometry['minimum_grasp_interference']):
        raise AssertionError(
            'dynamic contact grasp target cannot reach the reference object')

    lateral_overlap = min(
        size_y / 2.0, float(geometry['fingertip_half_width_y']))
    if lateral_overlap < float(geometry['minimum_lateral_overlap']):
        raise AssertionError(
            'dynamic contact object has insufficient lateral overlap')

    support_center = [float(value) for value in item['support_center_m']]
    support_size = [float(value) for value in item['support_size_m']]
    if min(support_size) <= 0.0:
        raise AssertionError(
            'dynamic contact acquisition support dimensions are invalid')
    support_top_z = support_center[2] + support_size[2] / 2.0
    support_bottom_z = support_center[2] - support_size[2] / 2.0
    if (abs(support_center[0] - center_x) > center_limit or
            abs(support_center[1] - center_y) > center_limit):
        raise AssertionError(
            'dynamic contact acquisition support is not centered')
    support_top_gap = support_top_z - object_min_z
    if abs(support_top_gap) > float(
            geometry['maximum_support_top_gap']):
        raise AssertionError(
            'dynamic contact acquisition support does not meet the object')
    support_base_clearance = (
        support_bottom_z - float(geometry['base_collision_max_z']))
    if support_base_clearance < float(
            geometry['minimum_support_base_clearance']):
        raise AssertionError(
            'dynamic contact acquisition support overlaps the gripper base')
    support_finger_clearance = (
        contact_face_inner_x - support_size[0] / 2.0)
    if support_finger_clearance < float(
            geometry['minimum_support_finger_clearance']):
        raise AssertionError(
            'dynamic contact acquisition support obstructs the fingers')

    return {
        'object_z_range_m': [object_min_z, object_max_z],
        'base_clearance_m': base_clearance,
        'contact_face_z_overlap_m': contact_face_z_overlap,
        'release_clearance_m': release_clearance,
        'grasp_interference_m': grasp_interference,
        'lateral_overlap_m': lateral_overlap,
        'support_top_gap_m': support_top_gap,
        'support_base_clearance_m': support_base_clearance,
        'support_finger_clearance_m': support_finger_clearance,
    }


def _validate_dynamic_contact(package_root: Path, contract: dict,
                              include_internal_evidence: bool = True) -> None:
    dynamic_contact = contract['dynamic_contact']
    config_path = package_root / dynamic_contact['reference_object_config']
    if not config_path.is_file():
        raise AssertionError(
            'missing dynamic contact reference-object configuration')
    if (_sha256(config_path) !=
            dynamic_contact['reference_object_config_sha256']):
        raise AssertionError(
            'dynamic contact reference-object configuration checksum '
            'changed without a manifest update')
    config = json.loads(config_path.read_text(encoding='utf-8'))
    if (config['schema_version'] != 1 or
            config['model'] != contract['model'] or
            config['asset_revision'] != contract['asset_revision'] or
            config['fidelity_target'] != 'dynamic-contact'):
        raise AssertionError(
            'dynamic contact reference-object identity is invalid')
    if config['reference_fingers']['configuration'] != (
            dynamic_contact['reference_fingers']):
        raise AssertionError(
            'dynamic contact finger configuration differs from manifest')
    if config['reference_fingers']['qualification_scope'] != (
            'OnRobot-supplied reference fingers only'):
        raise AssertionError(
            'dynamic contact custom-finger claim boundary is missing')
    physics = config['physics']
    if (physics['backend'] != 'physx' or
            abs(physics['physics_dt_s'] - 1.0 / 240.0) > 1e-12 or
            physics['solver_type'] != 'TGS' or
            not physics['enable_ccd'] or
            not physics['enable_stabilization'] or
            int(physics['trials']) < 3):
        raise AssertionError(
            'dynamic contact physics configuration is not release-bounded')
    objects = config['objects']
    object_set = dynamic_contact['object_set']
    if ([item['id'] for item in objects] != object_set or
            len(set(object_set)) != len(object_set)):
        raise AssertionError(
            'dynamic contact object set differs from manifest')
    for item in objects:
        if item['grasp_kind'] != 'external-pinch':
            raise AssertionError('outward reference fingers require an '
                                 'external-pinch dynamic contact object')
        if (item['mass_kg'] <= 0.0 or
                min(item['size_m']) <= 0.0):
            raise AssertionError(
                'dynamic contact reference-object dimensions are invalid')
        if not (contract['usd']['lower_limit_m'] <=
                item['grasp_target_m'] < item['release_target_m'] <=
                contract['usd']['upper_limit_m']):
            raise AssertionError(
                'dynamic contact grasp targets exceed USD joint limits')
        if not (0.0 < item['minimum_bilateral_contact_fraction'] <= 1.0 and
                0.0 < item['minimum_preload_contact_fraction'] <= 1.0):
            raise AssertionError('dynamic contact fraction is invalid')
        if not (0.0 < item['contact_closing_step_m'] <=
                item['approach_closing_step_m'] <= 0.001):
            raise AssertionError(
                'dynamic contact approach increments are invalid')
        if not (0.0 < item['preload_closing_step_m'] <= 0.001):
            raise AssertionError(
                'dynamic contact preload increment is invalid')
        if not (0.0 < item['contact_approach_margin_m'] <= 0.01):
            raise AssertionError(
                'dynamic contact approach margin is invalid')
        if not (0.0 <= item['support_catch_gap_m'] <= 0.003):
            raise AssertionError(
                'dynamic contact support catch gap is invalid')
        normal_limit = item.get(
            'maximum_mean_hold_contact_normal_vertical_component')
        if normal_limit is not None and not (0.0 < normal_limit <= 0.25):
            raise AssertionError(
                'dynamic contact vertical-normal limit is invalid')
        _validate_dynamic_contact_object_placement(config, item)

    # The installed package deliberately contains the public asset contract and
    # fixture configuration, but not internal runtime reports.  Keep checking
    # the public configuration while leaving evidence comparison to a source
    # checkout that still contains the private reports.
    if not include_internal_evidence:
        return

    runtime_fidelity = contract['runtime']['fidelity']
    if dynamic_contact['status'] == 'runtime-pending':
        if runtime_fidelity != 'kinematic':
            raise AssertionError(
                'pending dynamic contact evidence requires kinematic fidelity')
        if (not dynamic_contact.get('reason') or
                dynamic_contact.get('required_rerun') !=
                'dynamic-contact-current-asset'):
            raise AssertionError(
                'pending dynamic contact evidence must name its rerun')
        if any(key in dynamic_contact for key in (
                'evidence', 'source_evidence_sha256')):
            raise AssertionError(
                'pending dynamic contact evidence must not promote a stale '
                'artifact')
        if contract['qualification']['contact_and_grasp'] != (
                'runtime-pending-current-asset'):
            raise AssertionError(
                'pending dynamic contact qualification is inconsistent')
        return
    if (dynamic_contact['status'] != 'automated-passed' or
            runtime_fidelity != 'dynamic-contact'):
        raise AssertionError(
            'dynamic contact status and runtime fidelity disagree')

    evidence_path = package_root / dynamic_contact['evidence']
    if not evidence_path.is_file():
        raise AssertionError('retained dynamic contact evidence is missing')
    evidence = json.loads(evidence_path.read_text(encoding='utf-8'))
    if (evidence['status'] != 'passed' or
            evidence['model'] != contract['model'] or
            evidence['asset_revision'] != contract['asset_revision'] or
            evidence['fidelity'] != 'dynamic-contact'):
        raise AssertionError(
            'retained dynamic contact evidence identity is invalid')
    source = evidence['source_artifact']
    if (source['sha256'] != dynamic_contact['source_evidence_sha256'] or
            source['config_sha256'] !=
            dynamic_contact['reference_object_config_sha256']):
        raise AssertionError(
            'retained dynamic contact evidence digest is inconsistent')
    scope = evidence['scope']
    if (scope['reference_fingers'] != dynamic_contact['reference_fingers'] or
            [scope['object_id']] != dynamic_contact['object_set'] or
            int(scope['trial_count']) != int(physics['trials'])):
        raise AssertionError(
            'retained dynamic contact evidence scope is inconsistent')
    expected_result = f"passed-{physics['trials']}-of-{physics['trials']}"
    if (not evidence['checks'] or
            set(evidence['checks'].values()) != {expected_result}):
        raise AssertionError(
            'retained dynamic contact acceptance checks are incomplete')
    _assert_close(float(evidence['drive']['stiffness']),
                  float(contract['drive_tuning']['stiffness']),
                  'retained dynamic contact drive stiffness')
    _assert_close(float(evidence['drive']['damping']),
                  float(contract['drive_tuning']['damping']),
                  'retained dynamic contact drive damping')
    expected_drive_force = contract['drive_tuning'].get(
        'physx_single_drive_force_n',
        contract['usd']['maximum_drive_force_n'])
    _assert_close(float(evidence['drive']['maximum_force_n']),
                  float(expected_drive_force),
                  'retained dynamic contact drive maximum force')
    if (evidence['drive']['qualified'] or
            contract['drive_tuning']['qualified'] or
            contract['qualification']['drive_tuning'] != 'pending'):
        raise AssertionError(
            'dynamic contact evidence overclaims drive-gain qualification')
    if contract['qualification']['contact_and_grasp'] != (
            'automated-passed-reference-object'):
        raise AssertionError('dynamic contact/grasp claim is inconsistent')


def _validate_kinematic_evidence(package_root: Path, contract: dict) -> None:
    qualification = contract['qualification']
    status = qualification['automated_kinematic_regression']
    if status == 'pending':
        return
    if status != 'automated-passed':
        raise AssertionError('unsupported kinematic qualification status')
    evidence_path = package_root / qualification['kinematic_evidence']
    if not evidence_path.is_file():
        raise AssertionError('retained kinematic evidence is missing')
    evidence = json.loads(evidence_path.read_text(encoding='utf-8'))
    runtime = evidence['runtime']
    if (evidence['status'] != 'passed' or
            evidence['model'] != contract['model'] or
            evidence['asset_revision'] != contract['asset_revision'] or
            evidence['fidelity'] != 'kinematic' or
            runtime['physics_backend'] !=
            contract['runtime']['physics_backend'] or
            runtime['ros_distribution'] !=
            contract['runtime']['ros_distribution'] or
            runtime['isaac_sim_version'] not in
            contract['runtime']['tested_isaac_sim_versions']):
        raise AssertionError(
            'retained kinematic evidence identity is invalid')
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
    if (set(evidence['tests']) != expected_cases or
            set(evidence['tests'].values()) != {'passed'}):
        raise AssertionError(
            'retained kinematic acceptance checks are incomplete')
    if (evidence['source_artifact']['sha256'] !=
            qualification['kinematic_source_evidence_sha256']):
        raise AssertionError(
            'retained kinematic source digest is inconsistent')
    if evidence['measurements']['force_valid'] is not False:
        raise AssertionError('kinematic evidence overclaims simulated force')
    if qualification['articulation_endpoints'] != 'automated-passed':
        raise AssertionError(
            'kinematic promotion requires articulation endpoints')
    articulation = evidence.get('articulation_source_artifact')
    articulation_digest = qualification.get(
        'articulation_source_evidence_sha256')
    if articulation_digest is not None:
        if (not articulation or articulation['status'] != 'passed' or
                articulation['sha256'] != articulation_digest or
                set(articulation['tests'].values()) != {'passed'}):
            raise AssertionError(
                'retained articulation evidence is incomplete or '
                'inconsistent')


def _validate_hardware_in_loop_evidence(
        package_root: Path, contract: dict) -> None:
    """Validate an optional retained conventional hardware qualification."""
    hil = contract.get('hardware_in_loop')
    if hil is None:
        return
    if hil['status'] == 'runtime-pending-current-asset':
        required = hil.get('required_rerun')
        if (not hil.get('reason') or
                not isinstance(required, list) or not required or
                any(mode not in {'conventional', 'realtime-position'}
                    for mode in required)):
            raise AssertionError(
                'pending hardware-in-the-loop evidence must name its rerun')
        if any(key in hil for key in (
                'evidence', 'source_evidence_sha256',
                'realtime_position_evidence',
                'realtime_position_source_evidence_sha256')):
            raise AssertionError(
                'pending hardware-in-the-loop evidence must not promote a '
                'stale artifact')
        qualification = contract['qualification']
        if ('conventional' in required and
                qualification['conventional_hardware_in_loop'] !=
                'runtime-pending-current-asset'):
            raise AssertionError(
                'pending conventional HIL qualification is inconsistent')
        if ('realtime-position' in required and
                qualification['realtime_hardware_in_loop'] !=
                'runtime-pending-current-asset'):
            raise AssertionError(
                'pending realtime HIL qualification is inconsistent')
        return
    realtime_pending_states = {
        'pending-firmware-identity-qualification',
        'pending-clean-timing-qualification',
    }
    realtime_qualified = (
        hil['status'] ==
        'automated-passed-conventional-and-realtime-position')
    conventional_claim_valid = (
        hil['status'] == 'automated-passed-conventional' and
        hil['control'] == 'conventional' and
        hil['realtime_position_motion'] in realtime_pending_states)
    realtime_claim_valid = (
        realtime_qualified and
        hil['control'] == 'conventional-and-realtime-position' and
        hil['realtime_position_motion'] == 'automated-passed-at-50-hz')
    if not (conventional_claim_valid or realtime_claim_valid):
        raise AssertionError(
            'hardware-in-the-loop claim has an unsupported scope')
    evidence_path = package_root / hil['evidence']
    if not evidence_path.is_file():
        raise AssertionError(
            'retained hardware-in-the-loop evidence is missing')
    evidence = json.loads(evidence_path.read_text(encoding='utf-8'))
    runtime = evidence['runtime']
    if (evidence['status'] != 'passed' or
            evidence['model'] != contract['model'] or
            evidence['asset_revision'] != contract['asset_revision'] or
            evidence['fidelity'] != 'hardware-in-the-loop' or
            runtime['physics_backend'] !=
            contract['runtime']['physics_backend'] or
            runtime['ros_distribution'] !=
            contract['runtime']['ros_distribution'] or
            runtime['isaac_sim_version'] not in
            contract['runtime']['tested_isaac_sim_versions'] or
            evidence['source_artifact']['sha256'] !=
            hil['source_evidence_sha256']):
        raise AssertionError(
            'retained hardware-in-the-loop evidence identity is invalid')
    expected_tests = {
        'bounded_hardware_motion',
        'task_to_mechanism_delta',
        'restore_valid_initial_aperture',
        'hardware_to_usd_shadow_fidelity',
        'hardware_health_stable',
    }
    if (set(evidence['tests']) != expected_tests or
            set(evidence['tests'].values()) != {'passed'}):
        raise AssertionError(
            'retained hardware-in-the-loop checks are incomplete')
    if (evidence['control']['mode'] != 'conventional' or
            evidence['scope']['task_joint'] !=
            contract['ros']['task_joint'] or
            evidence['scope']['physical_joint'] !=
            contract['ros']['mechanism_joint']):
        raise AssertionError(
            'hardware-in-the-loop control or coordinate scope differs')
    measurements = evidence['measurements']
    coordinate_unit = evidence['scope'].get(
        'physical_coordinate_unit', 'm')
    if coordinate_unit not in {'m', 'rad'}:
        raise AssertionError(
            'hardware-in-the-loop physical coordinate unit is invalid')
    delta_error = measurements[
        f'task_to_mechanism_delta_error_{coordinate_unit}']
    shadow_error = measurements[
        f'maximum_hardware_to_usd_shadow_error_{coordinate_unit}']
    mimic_error = measurements[
        f'maximum_usd_mimic_error_{coordinate_unit}']
    shadow_tolerance = evidence['scope'][
        f'shadow_tolerance_{coordinate_unit}']
    if (abs(delta_error) > 0.002 or
            abs(measurements['restoration_error_m']) > 0.0015 or
            shadow_error > shadow_tolerance or
            mimic_error > shadow_tolerance):
        raise AssertionError(
            'hardware-in-the-loop measurements exceed acceptance limits')
    health = evidence['hardware_health_delta']
    if (health['sample_sequence'] <= 0 or
            health['successful_hardware_cycles'] <= 0 or
            any(health[key] != 0 for key in (
                'failed_hardware_cycles', 'missed_deadlines',
                'watchdog_stops', 'reconnects'))):
        raise AssertionError(
            'hardware-in-the-loop health evidence is incomplete')
    expected_asset_files = {
        str(Path(relative).relative_to(f"assets/{contract['model']}")):
        digest
        for relative, digest in contract['layers'].items()
    }
    if evidence['asset_files_sha256'] != expected_asset_files:
        raise AssertionError(
            'hardware-in-the-loop asset digests differ from the contract')
    qualification = contract['qualification']
    if (qualification['conventional_hardware_in_loop'] !=
            'automated-passed' or
            qualification['rviz_isaac_motion_agreement'] !=
            'operator-passed' or
            evidence['operator_observation'][
                'rviz_isaac_hardware_motion_agreement'] != 'passed'):
        raise AssertionError(
            'hardware-in-the-loop promotion status is inconsistent')
    if not realtime_qualified:
        return

    realtime_path = package_root / hil['realtime_position_evidence']
    if not realtime_path.is_file():
        raise AssertionError(
            'retained realtime-position HIL evidence is missing')
    realtime = json.loads(realtime_path.read_text(encoding='utf-8'))
    realtime_runtime = realtime['runtime']
    if (realtime['status'] != 'passed' or
            realtime['model'] != contract['model'] or
            realtime['asset_revision'] != contract['asset_revision'] or
            realtime['fidelity'] != 'hardware-in-the-loop' or
            realtime_runtime['physics_backend'] !=
            contract['runtime']['physics_backend'] or
            realtime_runtime['ros_distribution'] !=
            contract['runtime']['ros_distribution'] or
            realtime_runtime['isaac_sim_version'] not in
            contract['runtime']['tested_isaac_sim_versions'] or
            realtime['source_artifact']['sha256'] !=
            hil['realtime_position_source_evidence_sha256']):
        raise AssertionError(
            'retained realtime-position HIL evidence identity is invalid')
    if (set(realtime['tests']) != expected_tests or
            set(realtime['tests'].values()) != {'passed'} or
            realtime['control']['mode'] != 'realtime-position' or
            realtime['control']['hardware_exchange_rate_hz'] != 50 or
            realtime['scope']['task_joint'] !=
            contract['ros']['task_joint'] or
            realtime['scope']['physical_joint'] !=
            contract['ros']['mechanism_joint']):
        raise AssertionError(
            'retained realtime-position HIL scope is inconsistent')
    realtime_measurements = realtime['measurements']
    if (abs(realtime_measurements[
            'task_to_mechanism_delta_error_rad']) > 0.002 or
            abs(realtime_measurements['restoration_error_m']) > 0.0015 or
            realtime_measurements[
                'maximum_hardware_to_usd_shadow_error_rad'] >
            realtime['scope']['shadow_tolerance_rad'] or
            realtime_measurements['maximum_usd_mimic_error_rad'] >
            realtime['scope']['shadow_tolerance_rad']):
        raise AssertionError(
            'realtime-position HIL measurements exceed acceptance limits')
    realtime_health = realtime['hardware_health_delta']
    if (realtime_health['sample_sequence'] <= 0 or
            realtime_health['successful_hardware_cycles'] <= 0 or
            any(realtime_health[key] != 0 for key in (
                'failed_hardware_cycles', 'missed_deadlines',
                'watchdog_stops', 'reconnects'))):
        raise AssertionError(
            'realtime-position HIL health evidence is incomplete')
    if (realtime['asset_files_sha256'] != expected_asset_files or
            qualification['realtime_hardware_in_loop'] !=
            'automated-passed-at-50-hz'):
        raise AssertionError(
            'realtime-position HIL promotion is inconsistent')


def validate(package_root: Path, ros_root: Path,
             description_root: Path, model: str = '2fg7',
             include_internal_evidence: bool | None = None) -> dict:
    """Validate asset files and their agreement with the ROS description."""
    if include_internal_evidence is None:
        include_internal_evidence = (package_root / 'evidence').is_dir()
    contract_path = package_root / 'config' / f'{model}_asset_contract.json'
    contract = json.loads(contract_path.read_text(encoding='utf-8'))
    if contract['model'] != model:
        raise AssertionError(
            f'contract identifies {contract["model"]}, expected {model}')
    tested_versions = contract['runtime']['tested_isaac_sim_versions']
    if (not isinstance(tested_versions, list) or
            any(not isinstance(version, str) or not version
                for version in tested_versions)):
        raise AssertionError(
            'tested Isaac Sim versions must be a list of version strings')
    if contract['runtime']['version_policy'] != 'feature-gated':
        raise AssertionError(
            'Isaac Sim runtime compatibility must remain feature-gated')
    if (contract['runtime']['runtime_validation'] != 'pending' and
            not tested_versions):
        raise AssertionError(
            'a qualified runtime claim requires at least one tested version')
    if contract['runtime']['physics_backend'] != 'physx':
        raise AssertionError(
            f'the supplied {model} asset is qualified only for PhysX')
    _validate_usd(package_root, contract)
    _validate_ros(contract, ros_root, description_root)
    if include_internal_evidence:
        _validate_kinematic_evidence(package_root, contract)
    if 'dynamic_contact' in contract:
        _validate_dynamic_contact(
            package_root, contract, include_internal_evidence)
    if include_internal_evidence:
        _validate_hardware_in_loop_evidence(package_root, contract)
    contact_status = contract.get('dynamic_contact', {}).get('status')
    if contact_status is None:
        contact_status = contract['qualification'].get(
            'hard_fingertip_contact', 'not-run')
    report = {
        'model': contract['model'],
        'asset_revision': contract['asset_revision'],
        'tested_isaac_sim_versions': tested_versions,
        'physics_backend': contract['runtime']['physics_backend'],
        'offline_contract': 'passed',
        'runtime_validation': contract['runtime']['runtime_validation'],
        'fidelity': contract['runtime']['fidelity'],
        'dynamic_contact_status': contact_status,
        'hardware_in_loop_status': contract.get(
            'hardware_in_loop', {}).get('status', 'not-run'),
    }
    if not include_internal_evidence:
        report.update({
            'runtime_validation': 'not-published',
            'dynamic_contact_status': 'not-published',
            'hardware_in_loop_status': 'not-published',
        })
    return report


def main() -> int:
    """Run the command-line validator."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=SUPPORTED_MODELS,
                        default='2fg7')
    parser.add_argument('--package-root', type=Path, default=_package_root())
    parser.add_argument('--model-root', type=Path)
    parser.add_argument('--description-root', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument(
        '--with-internal-evidence', action='store_true',
        help=('also verify private runtime reports in a source checkout; '
              'these reports are not part of the installed package'))
    args = parser.parse_args()

    repository = args.package_root.parent
    package_name = f'onrobot_{args.model}'
    ros_root = args.model_root or _share_path(
        package_name, repository / package_name)
    description_root = args.description_root or _share_path(
        'onrobot_gripper_description',
        repository / 'onrobot_gripper_description')
    try:
        report = validate(
            args.package_root, ros_root, description_root, args.model,
            include_internal_evidence=(True if args.with_internal_evidence
                                       else None))
    except (AssertionError, KeyError, OSError, RuntimeError,
            ET.ParseError, ValueError) as error:
        print(
            f'{args.model} Isaac asset contract: FAIL: {error}',
            file=sys.stderr)
        return 1
    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

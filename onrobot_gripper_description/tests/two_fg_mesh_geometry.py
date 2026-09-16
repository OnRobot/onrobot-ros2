"""Read-only source mesh bounds at physical prismatic coordinates."""
from pathlib import Path
import json
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
import xacro

ROS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROS / 'onrobot_gripper_description/tests'))
from rg_mesh_geometry import dae_vertices, origin  # noqa: E402


def inspect(model, q, kind):
    root = ROS / ('onrobot_' + model)
    document = xacro.parse((root / f'urdf/realmesh_onrobot_{model}.urdf.xacro').read_text())
    for include in document.getElementsByTagName('xacro:include'):
        include.setAttribute('filename', include.getAttribute('filename').replace(
            f'$(find onrobot_{model})', str(root)))
    for call in document.getElementsByTagName(f'xacro:onrobot_{model}_realmesh'):
        call.setAttribute('include_ros2_control', 'false')
    xacro.process_doc(document)
    tree = ET.fromstring(document.toxml())
    transforms = {'base_link': np.eye(4)}
    joints = list(tree.findall('joint'))
    while joints:
        before = len(joints)
        for joint in joints[:]:
            parent = joint.find('parent').get('link')
            if parent not in transforms:
                continue
            motion = np.eye(4)
            mimic = joint.find('mimic')
            position = q if joint.get('name') == 'finger_stroke' else (
                q * float(mimic.get('multiplier', 1)) + float(mimic.get('offset', 0))
                if mimic is not None else 0.)
            if joint.get('type') == 'prismatic':
                motion[:3, 3] = position * np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            transforms[joint.find('child').get('link')] = (
                transforms[parent] @ origin(joint.find('origin')) @ motion)
            joints.remove(joint)
        if before == len(joints):
            raise RuntimeError('unresolved tree')
    result = {'model': model, 'joint_m': q, 'kind': kind, 'tips': {}}
    for side in ('left', 'right'):
        name = f'{side}_fingertip_link'
        geom = tree.find(f"link[@name='{name}']/{kind}")
        mesh = geom.find('geometry/mesh')
        path = Path(mesh.get('filename').replace('package://', str(ROS) + '/'))
        if path.suffix.lower() == '.dae':
            vertices = dae_vertices(path)
        else:
            data = path.read_bytes()
            count = struct.unpack_from('<I', data, 80)[0]
            vertices = np.array([struct.unpack_from('<9f', data, 96 + 50*i)
                                 for i in range(count)]).reshape(-1, 3)
        vertices *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
        tf = transforms[name] @ origin(geom.find('origin'))
        vertices = vertices @ tf[:3, :3].T + tf[:3, 3]
        face = vertices[vertices[:, 2] > vertices[:, 2].max() - .010]
        result['tips'][side] = {
            'bounds': [vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()],
            'top_10mm_bounds': [face.min(axis=0).tolist(), face.max(axis=0).tolist()]}
    return result


if __name__ == '__main__':
    print(json.dumps([inspect(model, q, kind)
                      for model, end in [('2fg7', .019), ('2fg14', .025)]
                      for q in (0., end/2, end) for kind in ('collision', 'visual')], indent=2))

"""Measure assembled stock RG visual/collision gaps using source-only Xacro."""
import math
import pathlib
import struct
import xml.etree.ElementTree as ET

import numpy as np
import xacro

ROOT = pathlib.Path(__file__).resolve().parents[2]


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def origin(element):
    matrix = np.eye(4)
    if element is not None:
        matrix[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        matrix[:3, :3] = rotation(np.fromstring(
            element.get('rpy', '0 0 0'), sep=' '))
    return matrix


def dae_vertices(path):
    """Apply the authored Collada node transforms to position vertices."""
    doc = ET.parse(path).getroot()
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    geometries = {}
    for geometry in doc.findall('.//c:geometry', ns):
        position = geometry.find('c:mesh/c:vertices/c:input[@semantic="POSITION"]', ns)
        source = geometry.find(
            f'c:mesh/c:source[@id="{position.get("source")[1:]}"]/c:float_array', ns)
        geometries[geometry.get('id')] = np.fromstring(source.text, sep=' ').reshape(-1, 3)
    result = []

    def visit(node, transform):
        matrix = node.find('c:matrix', ns)
        if matrix is not None:
            transform = transform @ np.fromstring(matrix.text, sep=' ').reshape(4, 4)
        for instance in node.findall('c:instance_geometry', ns):
            vertices = geometries[instance.get('url')[1:]]
            result.append(vertices @ transform[:3, :3].T + transform[:3, 3])
        for child in node.findall('c:node', ns):
            visit(child, transform)

    for node in doc.findall('.//c:visual_scene/c:node', ns):
        visit(node, np.eye(4))
    return np.concatenate(result) * float(doc.find('c:asset/c:unit', ns).get('meter'))


def expanded_model(model, orientation='inwards'):
    """Pin includes to source; a sourced older install must not change this test."""
    if model not in ('rg2', 'rg6') or orientation not in ('inwards', 'outwards'):
        raise ValueError('unsupported RG model or fingertip orientation')
    model_root = ROOT / f'onrobot_{model}'
    entry = model_root / f'urdf/realmesh_onrobot_{model}.urdf.xacro'
    document = xacro.parse(entry.read_text())
    prefix = f'$(find onrobot_{model})/'
    for include in document.getElementsByTagName('xacro:include'):
        filename = include.getAttribute('filename')
        if not filename.startswith(prefix):
            raise ValueError(f'review unexpected source include: {filename}')
        include.setAttribute('filename', str(model_root / filename[len(prefix):]))
    xacro.process_doc(document, mappings={'fingertip_orientation': orientation})
    return ET.fromstring(document.toxml())


def gap(model, angle, orientation='inwards', kind='collision'):
    tree = expanded_model(model, orientation)
    transforms = {'base_link': np.eye(4)}
    joints = list(tree.findall('joint'))
    while joints:
        count = len(joints)
        for joint in joints[:]:
            parent = joint.find('parent').get('link')
            if parent not in transforms:
                continue
            mimic = joint.find('mimic')
            q = angle if joint.get('name') == 'finger_joint' else (
                angle * float(mimic.get('multiplier', '1')) +
                float(mimic.get('offset', '0')) if mimic is not None else 0)
            axis = joint.find('axis')
            a = np.array([0., 0., 1.]) if axis is None else np.fromstring(
                axis.get('xyz'), sep=' ')
            cross = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]],
                              [-a[1], a[0], 0]])
            motion = np.eye(4)
            motion[:3, :3] += math.sin(q)*cross + (1-math.cos(q))*(cross@cross)
            transforms[joint.find('child').get('link')] = (
                transforms[parent] @ origin(joint.find('origin')) @ motion)
            joints.remove(joint)
        if len(joints) == count:
            raise RuntimeError('Unresolved URDF tree')
    bounds = []
    for side in ['left', 'right']:
        link = tree.find(f"link[@name='{side}_finger_tip_link']")
        collision = link.find(kind)
        filename = collision.find('geometry/mesh').get('filename')
        path = pathlib.Path(filename.replace('package://', str(ROOT)+'/'))
        if path.suffix.lower() == '.dae':
            vertices = dae_vertices(path)
        else:
            data = path.read_bytes()
            count = struct.unpack_from('<I', data, 80)[0]
            vertices = np.array([struct.unpack_from('<9f', data, 96+i*50)
                                 for i in range(count)]).reshape(-1, 3)
        transform = transforms[link.get('name')] @ origin(collision.find('origin'))
        vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
        bounds.append((vertices[:, 0].min(), vertices[:, 0].max()))
    return bounds[1][0] - bounds[0][1]

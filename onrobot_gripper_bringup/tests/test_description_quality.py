"""Regression checks for the installed model-description contract.

These checks intentionally exercise the canonical Xacro inputs rather than
copying generated URDF files into the test.  The model packages remain the
owners of their descriptions; bringup only verifies that the public launch
surface can expand them and that the SRDF references the resulting links.
"""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


ROS_ROOT = Path(__file__).resolve().parents[2]
MODELS = ('2fg7', '2fg14', 'rg2', 'rg6', '3fg15', '3fg25')


def _expand(path):
    result = subprocess.run(
        ['xacro', str(path)],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def _urdf_tree_errors(root, path):
    """Return actionable description defects for a parsed URDF root."""
    errors = []
    links = {link.attrib['name'] for link in root.findall('link')}
    if not links:
        errors.append(f'{path}: no links')
    for joint in root.findall('joint'):
        name = joint.attrib.get('name', '<unnamed>')
        parent = joint.find('parent')
        child = joint.find('child')
        if parent is None or parent.attrib.get('link') not in links:
            errors.append(f'{path}: {name} has an invalid parent')
        if child is None or child.attrib.get('link') not in links:
            errors.append(f'{path}: {name} has an invalid child')
        if joint.attrib.get('type') != 'fixed' and len(joint.findall('limit')) > 1:
            errors.append(f'{path}: {name} has duplicate limits')
    return errors


def test_all_supported_model_xacros_expand_and_have_valid_urdf_trees():
    """Catch missing defaults, package paths, links, and joint references."""
    for model in MODELS:
        package = ROS_ROOT / f'onrobot_{model}'
        for variant in ('simplemesh', 'realmesh'):
            path = package / 'urdf' / f'{variant}_onrobot_{model}.urdf.xacro'
            root = _expand(path)
            assert not _urdf_tree_errors(root, path)


def test_description_quality_checker_rejects_broken_fixture():
    """Keep the regression meaningful by exercising its failure paths."""
    root = ET.fromstring(
        '<robot name="broken">'
        '<link name="base"/>'
        '<joint name="bad_joint" type="prismatic">'
        '<parent link="missing_parent"/><child link="base"/>'
        '<limit effort="1" velocity="1" lower="0" upper="1"/>'
        '<limit effort="2" velocity="2" lower="0" upper="1"/>'
        '</joint></robot>')
    errors = _urdf_tree_errors(root, 'broken_fixture')
    assert any('invalid parent' in error for error in errors)
    assert any('duplicate limits' in error for error in errors)


def test_rg6_srdf_references_the_canonical_realmesh_links():
    """Keep RG6 MoveIt collision exclusions aligned with both Xacro variants."""
    root = _expand(
        ROS_ROOT / 'onrobot_rg6' / 'urdf' /
        'realmesh_onrobot_rg6.urdf.xacro')
    links = {link.attrib['name'] for link in root.findall('link')}
    srdf = ET.parse(
        ROS_ROOT / 'onrobot_rg6' / 'srdf' / 'onrobot_rg6.srdf').getroot()
    referenced = {
        value
        for element in srdf.iter()
        for attribute in ('link1', 'link2', 'link')
        if (value := element.attrib.get(attribute)) is not None
    }
    assert referenced <= links, sorted(referenced - links)
    assert 'left_inner_proximal_finger_link' in links
    assert 'right_inner_proximal_finger_link' in links


def test_legacy_static_urdfs_reference_existing_package_meshes():
    """Keep the directly consumable URDF files usable, not only their Xacros."""
    for model in MODELS:
        package = ROS_ROOT / f'onrobot_{model}'
        path = package / 'urdf' / f'onrobot_{model}.urdf'
        if not path.exists():
            continue
        root = ET.parse(path).getroot()
        for mesh in root.findall('.//mesh'):
            filename = mesh.attrib.get('filename', '')
            prefix = f'package://onrobot_{model}/meshes/'
            if filename.startswith(prefix):
                assert (package / 'meshes' / filename[len(prefix):]).is_file(), (
                    f'{path}: missing mesh {filename}')


def test_3fg15_moving_finger_limits_are_not_zero_shadow_limits():
    """Prevent urdfdom from selecting the old zero-limit duplicate."""
    path = ROS_ROOT / 'onrobot_3fg15' / 'urdf' / 'simplemesh_onrobot_3fg15.urdf.xacro'
    root = ET.parse(path).getroot()
    for name in ('right_finger_base_joint', 'middle_finger_base_joint'):
        joint = root.find(f"joint[@name='{name}']")
        assert joint is not None
        limits = joint.findall('limit')
        assert len(limits) == 1
        assert limits[0].attrib['effort'] != '0'
        assert limits[0].attrib['velocity'] != '0'

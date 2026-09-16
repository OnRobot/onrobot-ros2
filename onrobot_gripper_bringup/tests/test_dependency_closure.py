"""Keep model-neutral bringup installable without optional GUI packages."""

from pathlib import Path
import ast
import xml.etree.ElementTree as ET


ROS_ROOT = Path(__file__).resolve().parents[2]
GUI_DEPENDENCIES = {
    'joint_state_publisher_gui',
    'onrobot_gripper_rviz_plugins',
    'rviz2',
}
GUI_BLACKLIST = GUI_DEPENDENCIES | {
    'python_qt_binding',
    'qt_gui',
    'rviz_common',
    'rviz_default_plugins',
    'rqt_gui',
    'rqt_gui_cpp',
}
GUI_FRONTDOOR = 'onrobot_gripper_demos'


def _manifest_dependencies(manifest):
    root = ET.parse(manifest).getroot()
    tags = {'depend', 'build_depend', 'build_export_depend', 'exec_depend'}
    return {
        element.text.strip()
        for element in root
        if element.tag in tags and element.text
    }


def _source_package(name):
    for candidate in ROS_ROOT.glob('*/package.xml'):
        root = ET.parse(candidate).getroot()
        if root.findtext('name') == name:
            return candidate
    return None


def _transitive_source_dependencies(root_name):
    pending = [root_name]
    seen = set()
    while pending:
        name = pending.pop()
        if name in seen:
            continue
        seen.add(name)
        manifest = _source_package(name)
        if manifest is None:
            continue
        pending.extend(_manifest_dependencies(manifest))
    return seen


def test_bringup_transitive_source_dependencies_are_headless():
    """Reject GUI dependencies anywhere below the bringup package."""
    closure = _transitive_source_dependencies('onrobot_gripper_bringup')
    assert not (closure & GUI_BLACKLIST), closure & GUI_BLACKLIST


def test_demo_frontdoor_owns_optional_gui_dependencies():
    """Require the GUI application package to declare every visual extra."""
    demo_manifest = _source_package(GUI_FRONTDOOR)
    assert GUI_DEPENDENCIES <= _manifest_dependencies(demo_manifest)
    closure = _transitive_source_dependencies(GUI_FRONTDOOR)
    assert GUI_DEPENDENCIES <= closure


def _literal_launch_package_references(launch_file):
    references = set()
    tree = ast.parse(launch_file.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            for keyword in node.keywords:
                if (keyword.arg == 'package' and
                        isinstance(keyword.value, ast.Constant) and
                        isinstance(keyword.value.value, str)):
                    references.add(keyword.value.value)
            if (isinstance(node.func, ast.Name) and
                    node.func.id in {
                        'FindPackageShare', 'get_package_share_directory'} and
                    node.args and isinstance(node.args[0], ast.Constant) and
                    isinstance(node.args[0].value, str)):
                references.add(node.args[0].value)
    return references


def test_installed_launch_references_have_direct_manifest_owners():
    """A reverse dependency must not conceal an entrypoint's requirements."""
    missing = {}
    for manifest in ROS_ROOT.glob('*/package.xml'):
        package_name = ET.parse(manifest).getroot().findtext('name')
        declared = _manifest_dependencies(manifest)
        for launch_file in manifest.parent.glob('launch/*.launch.py'):
            references = (
                _literal_launch_package_references(launch_file) -
                {package_name})
            undeclared = references - declared
            if undeclared:
                missing[str(launch_file.relative_to(ROS_ROOT))] = sorted(
                    undeclared)
    assert not missing, missing


def test_model_packages_install_no_gui_launch_entrypoints():
    """Keep visualization composition in the demos package."""
    for model in ('2fg7', '2fg14', 'rg2', 'rg6', '3fg15', '3fg25'):
        package = ROS_ROOT / f'onrobot_{model}'
        for launch_file in package.glob('launch/*.launch.py'):
            assert not (
                _literal_launch_package_references(launch_file) &
                GUI_DEPENDENCIES), launch_file

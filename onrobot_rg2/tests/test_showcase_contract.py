#!/usr/bin/env python3

"""Validate the RG2 description and software-rendered showcase contract."""

import pathlib
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET


PACKAGE_DIR = pathlib.Path(sys.argv[1]).resolve()
XACRO_FILE = PACKAGE_DIR / 'urdf' / 'realmesh_onrobot_rg2.urdf.xacro'
RVIZ_FILE = PACKAGE_DIR / 'rviz' / 'rg2_ros2_control_showcase.rviz'
CONTROL_LAUNCH = PACKAGE_DIR / 'launch' / 'control.launch.py'


class Rg2ShowcaseContract(unittest.TestCase):
    """Protect the measured linkage and ready-to-use RViz layout."""

    @classmethod
    def setUpClass(cls):
        """Expand the production Xacro once for the contract tests."""
        result = subprocess.run(
            ['xacro', str(XACRO_FILE)],
            check=True,
            capture_output=True,
            text=True,
        )
        cls.robot = ET.fromstring(result.stdout)

    def test_visual_joint_is_state_only_and_drives_the_linkage(self):
        """Require fixed structure and a measured state-only drive joint."""
        body_joint = self.robot.find("./joint[@name='body_joint']")
        self.assertIsNotNone(body_joint)
        self.assertEqual(body_joint.attrib['type'], 'fixed')

        finger_joint = self.robot.find("./joint[@name='finger_joint']")
        self.assertIsNotNone(finger_joint)
        self.assertEqual(finger_joint.attrib['type'], 'revolute')
        self.assertEqual(finger_joint.find('limit').attrib['lower'], '0')
        self.assertAlmostEqual(
            float(finger_joint.find('limit').attrib['upper']),
            1.3136012652574625,
        )

        control = self.robot.find(
            "./ros2_control/joint[@name='finger_joint']"
        )
        self.assertIsNotNone(control)
        self.assertEqual(control.findall('command_interface'), [])
        self.assertEqual(
            [
                item.attrib['name']
                for item in control.findall('state_interface')
            ],
            [
                'position', 'velocity', 'measured_angular_position',
                'measured_angular_velocity', 'position_valid',
                'velocity_valid',
            ],
        )

        task_control = self.robot.find(
            "./ros2_control/joint[@name='grip_stroke']"
        )
        self.assertIn(
            'realtime_mechanism_angular_velocity',
            [item.attrib['name'] for item in
             task_control.findall('command_interface')],
        )

        mimics = self.robot.findall("./joint/mimic[@joint='finger_joint']")
        self.assertGreaterEqual(len(mimics), 6)

    def test_every_visual_mesh_is_an_installed_rg2_resource(self):
        """Require every visual resource to exist in the package."""
        visuals = self.robot.findall('./link/visual/geometry/mesh')
        self.assertGreater(len(visuals), 0)
        for visual in visuals:
            resource = visual.attrib['filename']
            self.assertTrue(
                resource.startswith('package://onrobot_rg2/meshes/')
            )
            mesh_name = resource.rsplit('/', 1)[1]
            self.assertTrue((PACKAGE_DIR / 'meshes' / mesh_name).is_file())

    def test_showcase_opens_with_the_conventional_control_panel(self):
        """Make the conventional showcase operable without manual setup."""
        layout = RVIZ_FILE.read_text(encoding='utf-8')
        self.assertIn('Class: rviz_default_plugins/RobotModel', layout)
        self.assertIn('Collision Enabled: false', layout)
        self.assertIn(
            'Class: onrobot_gripper_rviz_plugins/GripperControlPanel',
            layout,
        )

    def test_control_launch_removes_unavailable_effort_from_joint_states(self):
        """Route RG state through the effort cleanup node."""
        launch = CONTROL_LAUNCH.read_text(encoding='utf-8')
        self.assertIn("package='onrobot_gripper_hardware'", launch)
        self.assertIn("executable='joint_state_effort_filter'", launch)
        self.assertNotIn("package='rviz2'", launch)

    def test_control_launch_exposes_the_isaac_backend_contract(self):
        """Keep RG2 available through the shared Isaac ros2_control path."""
        launch = CONTROL_LAUNCH.read_text(encoding='utf-8')
        self.assertIn("choices=['real', 'fake', 'isaac']", launch)
        for argument in (
                'isaac_joint_commands_topic',
                'isaac_joint_states_topic',
                'isaac_state_timeout_ms',
                'use_sim_time'):
            self.assertIn(argument, launch)

        result = subprocess.run(
            ['xacro', str(XACRO_FILE), 'backend:=isaac'],
            check=True,
            capture_output=True,
            text=True,
        )
        robot = ET.fromstring(result.stdout)
        plugin = robot.find('./ros2_control/hardware/plugin')
        self.assertIsNotNone(plugin)
        self.assertEqual(
            plugin.text,
            'onrobot_gripper_hardware/OnRobotIsaacSystem',
        )


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]])

"""Check the standalone MoveIt integration contract."""

from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
ROS_ROOT = PACKAGE.parent


class MoveItContract(unittest.TestCase):
    """Validate standard action, planning-joint, and geometry integration."""

    def test_moveit_uses_native_parallel_gripper_action(self):
        """Configure MoveIt's standard parallel-gripper action handle."""
        config = yaml.safe_load(
            (PACKAGE / 'config' / 'moveit_controllers.yaml').read_text())
        self.assertEqual(
            config['moveit_controller_manager'],
            'moveit_simple_controller_manager/MoveItSimpleControllerManager')
        manager = config['moveit_simple_controller_manager']
        self.assertEqual(manager['controller_names'], ['physical_gripper_controller'])
        controller = manager['physical_gripper_controller']
        self.assertEqual(controller['type'], 'ParallelGripperCommand')
        self.assertEqual(controller['action_ns'], 'gripper_cmd')
        self.assertEqual(controller['joints'], ['finger_stroke'])
        self.assertEqual(controller['command_joint'], 'finger_stroke')
        # Zero omits optional physical velocity/effort fields in MoveIt's
        # handle. This adapter does not silently reinterpret them as task units.
        self.assertEqual(controller['max_effort'], 0.0)
        self.assertEqual(controller['max_velocity'], 0.0)
        self.assertTrue(controller['default'])

    def test_all_controllers_match_moveit_and_allow_grasp_stall(self):
        """Keep each conventional controller compatible with MoveIt."""
        controller_files = [
            ROS_ROOT / 'onrobot_2fg7/config/ros2_control_controllers.yaml',
            ROS_ROOT / 'onrobot_2fg14/config/ros2_control_controllers.yaml',
            ROS_ROOT / 'onrobot_gripper_controllers/config/rg.yaml',
        ]
        for path in controller_files:
            with self.subTest(path=path):
                config = yaml.safe_load(path.read_text())
                declared = config['/**/controller_manager']['ros__parameters'][
                    'gripper_controller']
                self.assertEqual(
                    declared['type'],
                    'onrobot_gripper_controllers/'
                    'ParallelGripperActionController')
                params = config['/**/gripper_controller']['ros__parameters']
                self.assertEqual(params['joint'], 'grip_stroke')
                self.assertTrue(params['allow_stalling'])
                self.assertGreater(params['stall_timeout'], 0.0)
                self.assertGreater(params['stall_velocity_threshold'], 0.0)

    def test_semantic_group_moves_physical_linkage(self):
        """Expand every SRDF; task leaf must never be a planned actuator."""
        for model in ('2fg7', '2fg14', 'rg2', 'rg6'):
            joint = 'finger_stroke' if model.startswith('2fg') else 'finger_joint'
            for prefix in ('', 'tool_'):
                with self.subTest(model=model, prefix=prefix):
                    xml = subprocess.check_output([
                        'xacro', str(PACKAGE / 'config/onrobot_gripper.srdf.xacro'),
                        'robot_name:=onrobot_' + model,
                        'model:=' + model, 'prefix:=' + prefix,
                        'physical_joint:=' + prefix + joint], text=True)
                    srdf = ET.fromstring(xml)
                    group_joints = [j.get('name') for j in srdf.findall('./group/joint')]
                    self.assertEqual(group_joints[0], prefix + joint)
                    self.assertNotIn(prefix + 'grip_stroke', group_joints)
                    self.assertEqual(len(group_joints), len(set(group_joints)))
                    # Dependent joints must be present for planner group-state
                    # copies, but must never become extra actuators.
                    robot = ET.fromstring(subprocess.check_output([
                        'xacro', str(ROS_ROOT / ('onrobot_' + model) / 'urdf' /
                                     ('realmesh_onrobot_' + model + '.urdf.xacro')),
                        'backend:=fake'], text=True))
                    joints = {j.get('name'): j for j in robot.findall('joint')}
                    for name in group_joints[1:]:
                        self.assertIsNotNone(joints[name[len(prefix):]].find('mimic'))
                    for pair in srdf.findall('disable_collisions'):
                        self.assertTrue(pair.get('link1').startswith(prefix))
                        self.assertTrue(pair.get('link2').startswith(prefix))
                        self.assertNotEqual(pair.get('link1'), pair.get('link2'))

    def test_rg_follower_limits_cover_leader_range(self):
        """A 1:1 passive follower cannot be immovable or stop before its leader."""
        for model in ('rg2', 'rg6'):
            with self.subTest(model=model):
                xml = subprocess.check_output([
                    'xacro', str(ROS_ROOT / ('onrobot_' + model) / 'urdf' /
                                 ('realmesh_onrobot_' + model + '.urdf.xacro')),
                    'backend:=fake'], text=True)
                joints = {j.get('name'): j for j in ET.fromstring(xml).findall('joint')}
                follower = joints['left_outer_proximal_finger_joint']
                self.assertEqual(follower.find('mimic').get('joint'), 'finger_joint')
                self.assertEqual(float(follower.find('mimic').get('multiplier')), 1.0)
                leader_limit, follower_limit = joints['finger_joint'].find('limit'), follower.find('limit')
                self.assertLessEqual(float(follower_limit.get('lower')), float(leader_limit.get('lower')))
                self.assertGreaterEqual(float(follower_limit.get('upper')), float(leader_limit.get('upper')))

    def test_2fg7_moving_finger_links_have_collision_geometry(self):
        """Provide collision geometry for each moving 2FG7 finger."""
        xacro = ROS_ROOT / 'onrobot_2fg7/urdf/' / \
            'realmesh_onrobot_2fg7.urdf.xacro'
        result = subprocess.run(
            ['xacro', str(xacro)],
            check=True,
            capture_output=True,
            text=True,
        )
        links = {
            link.attrib['name']: link
            for link in ET.fromstring(result.stdout).findall('link')}
        for link_name in ('right_finger_base_link', 'left_finger_base_link'):
            with self.subTest(link=link_name):
                self.assertIsNotNone(links[link_name].find('visual'))
                self.assertIsNotNone(links[link_name].find('collision'))


if __name__ == '__main__':
    unittest.main()

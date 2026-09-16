"""Load real mesh RobotStates and require FCL obstacle-sensitive planning."""
import json
import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml


@pytest.mark.parametrize('model', ['2fg7', '2fg14', 'rg2', 'rg6'])
def test_min_mid_max_physical_states_and_obstacles(model, tmp_path):
    share = Path(get_package_share_directory('onrobot_' + model))
    moveit = Path(get_package_share_directory('onrobot_gripper_moveit_config'))
    profile = Path(get_package_share_directory('onrobot_gripper_description')) / 'config/planning_profiles' / (model + '.yaml')
    joint = 'finger_stroke' if model.startswith('2fg') else 'finger_joint'
    urdf = tmp_path / 'robot.urdf'
    srdf = tmp_path / 'robot.srdf'
    with urdf.open('w') as stream:
        subprocess.run(['xacro', str(share / 'urdf' / f'realmesh_onrobot_{model}.urdf.xacro'),
                        'backend:=fake'], stdout=stream, check=True, timeout=15)
    with srdf.open('w') as stream:
        subprocess.run(['xacro', str(moveit / 'config/onrobot_gripper.srdf.xacro'),
                        'robot_name:=onrobot_' + model, 'model:=' + model,
                        'physical_joint:=' + joint], stdout=stream, check=True, timeout=15)
    run = subprocess.run([os.environ['PLANNING_GEOMETRY_PROBE'], str(urdf), str(srdf), str(profile)],
                         capture_output=True, text=True, timeout=60)
    assert run.returncode == 0, run.stderr
    result = yaml.safe_load(run.stdout.split('---PROBE---\n')[-1])
    (tmp_path / 'geometry.json').write_text(json.dumps(result, indent=2) + '\n')
    for sample in result['samples']:
        assert result['group_active_variable_count'] == 1, result
        assert sample['mimics_follow_group'], result
        assert not sample['self_collision'], result
        assert not sample['group_self_collision'], result
        assert sample['obstacle_blocks_at_q'], result
        assert sample['task_leaf_keeps_collision'], result
        assert sample['physical_translation_m'] > 0.001, result
        assert sample['obstacle_clear_after_physical_move'], result

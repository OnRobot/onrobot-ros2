"""Compare compiled profile output with independently transformed CAD meshes."""
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix
import pytest
import yaml

from rg_mesh_geometry import gap
from two_fg_mesh_geometry import inspect


@pytest.mark.parametrize('model', ['2fg7', '2fg14', 'rg2', 'rg6'])
@pytest.mark.parametrize('kind', ['visual', 'collision'])
def test_stock_profile_matches_the_actual_moving_mesh(model, kind):
    profile = Path(__file__).resolve().parents[1] / 'config/planning_profiles' / (model + '.yaml')
    resolver = Path(get_package_prefix('onrobot_gripper_description')) / 'bin/resolve_gripper_profile'

    def resolve(*args):
        result = subprocess.run([str(resolver), '--profile', str(profile), *args],
                                capture_output=True, text=True, check=True, timeout=5)
        return yaml.safe_load(result.stdout)

    domain = resolve()['safe_q_domain']
    q_min, q_max = domain['minimum'], domain['maximum']
    for q in (q_min, (q_min + q_max) / 2, q_max):
        expected = resolve('--joint', str(q))['aperture_m']
        if model.startswith('rg'):
            measured = gap(model, q, kind=kind)
            tolerance = 1e-6  # supplied RG DAE/STL agreement
        else:
            tips = inspect(model, q, kind)['tips']
            left = tips['left']['top_10mm_bounds']
            right = tips['right']['top_10mm_bounds']
            measured = right[0][0] - left[1][0]
            # 2FG14's visual export differs from STL by 10 micrometres.
            tolerance = 15e-6 if model == '2fg14' and kind == 'visual' else 1e-6
        assert measured == pytest.approx(expected, abs=tolerance), (model, q, kind, measured, expected)

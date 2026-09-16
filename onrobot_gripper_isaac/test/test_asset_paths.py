"""Exercise asset discovery and content pinning without Isaac or ROS."""

import hashlib
import importlib.util
import json
import os
import subprocess
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'asset_path_contract', PACKAGE / 'scripts/isaac_model_contract.py')
CONTRACT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CONTRACT)
asset_repository_root = CONTRACT.asset_repository_root


def _fixture(tmp_path, *, nested=False):
    package = tmp_path / 'ros' / 'onrobot_gripper_isaac'
    (package / 'config').mkdir(parents=True)
    repo = (package.parent if nested else tmp_path) / 'onrobot-isaac-sim'
    (repo / 'assets/2fg7').mkdir(parents=True)
    asset = repo / 'assets/2fg7/onrobot_2fg7.usda'
    asset.write_text('#usda 1.0\n')
    manifest = repo / 'asset_manifest.json'
    manifest.write_text(json.dumps({'schema_version': 1, 'models': {}, 'files': {
        'assets/2fg7/onrobot_2fg7.usda': hashlib.sha256(asset.read_bytes()).hexdigest()}}))
    (package / 'config/asset_repository.json').write_text(json.dumps({
        'manifest_sha256': hashlib.sha256(manifest.read_bytes()).hexdigest()}))
    return package, repo


@pytest.mark.parametrize('nested', [False, True])
def test_source_asset_discovery_is_content_pinned(tmp_path, monkeypatch, nested):
    """Support both source layouts while rejecting an unpinned manifest."""
    monkeypatch.delenv('ONROBOT_ISAAC_ASSET_REPOSITORY', raising=False)
    package, repo = _fixture(tmp_path, nested=nested)
    assert asset_repository_root(package) == repo
    (repo / 'asset_manifest.json').write_text('{}')
    with pytest.raises(RuntimeError, match='does not match'):
        asset_repository_root(package)


def test_installed_assets_do_not_depend_on_development_environment(tmp_path, monkeypatch):
    """Prefer the installed assets over a stale development override."""
    package = tmp_path / 'installed'
    (package / 'assets').mkdir(parents=True)
    monkeypatch.setenv('ONROBOT_ISAAC_ASSET_REPOSITORY', str(tmp_path / 'missing'))
    assert asset_repository_root(package) == package


def test_missing_asset_dependency_is_actionable(tmp_path, monkeypatch):
    """Report missing checkout content and accept an explicit valid path."""
    package, repo = _fixture(tmp_path)
    monkeypatch.setenv('ONROBOT_ISAAC_ASSET_REPOSITORY', str(tmp_path / 'missing'))
    with pytest.raises(RuntimeError, match='assets are missing'):
        asset_repository_root(package)
    assert asset_repository_root(package, repository=repo) == repo


def _validator():
    path = asset_repository_root(PACKAGE) / 'tools/validate_assets.py'
    spec = importlib.util.spec_from_file_location('asset_integrity_validator', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    'mutation', ['tamper', 'missing', 'extra', 'symlink', 'directory-symlink'])
def test_manifest_rejects_incomplete_or_unsafe_assets(tmp_path, mutation):
    """Reject changed bytes, incomplete inventories and symlink escapes."""
    _, repo = _fixture(tmp_path)
    validator = _validator()
    assert validator.validate(repo, manifest_only=True)['status'] == 'passed'
    asset = repo / 'assets/2fg7/onrobot_2fg7.usda'
    if mutation == 'tamper':
        asset.write_text('#usda 1.0\n# changed\n')
    elif mutation == 'missing':
        asset.unlink()
    elif mutation == 'extra':
        (repo / 'assets/extra.usda').write_text('#usda 1.0\n')
    elif mutation == 'symlink':
        content = asset.read_bytes()
        asset.unlink()
        target = tmp_path / 'unrelated.usda'
        target.write_bytes(content)
        asset.symlink_to(target)
    else:
        (repo / 'assets').rename(repo / 'hidden-assets')
        (repo / 'assets').symlink_to(repo / 'hidden-assets', target_is_directory=True)
    with pytest.raises(ValueError):
        validator.validate(repo, manifest_only=True)


def test_external_assets_and_guides_install_without_runtime_source_dependency(tmp_path):
    """Run the real CMake install and verify its self-contained public layout."""
    repository = asset_repository_root(PACKAGE)
    build, prefix = tmp_path / 'build', tmp_path / 'prefix'
    commands = [
        ['cmake', '-S', str(PACKAGE), '-B', str(build),
         f'-DCMAKE_INSTALL_PREFIX={prefix}', '-DBUILD_TESTING=OFF',
         '-DPython3_EXECUTABLE=/usr/bin/python3',
         f'-DONROBOT_ISAAC_ASSET_REPOSITORY={repository}'],
        ['cmake', '--install', str(build)],
    ]
    for command in commands:
        result = subprocess.run(command, text=True, capture_output=True, timeout=20)
        assert result.returncode == 0, result.stdout + result.stderr
    share = prefix / 'share/onrobot_gripper_isaac'
    manifest = json.loads((repository / 'asset_manifest.json').read_text())
    for name, expected in manifest['files'].items():
        assert hashlib.sha256((share / name).read_bytes()).hexdigest() == expected
    for name in ('RVIZ_ISAAC_2FG7_DEMO.md', 'RG2_HIL_GUIDE.md',
                 'PHYSICAL_MOTION_SHOWCASE.md', 'ISAAC_SIM_VERIFICATION.md'):
        assert (share / 'doc' / name).read_bytes() == (PACKAGE / 'doc' / name).read_bytes()
    for script in ('run_workpiece_showcase.py', 'run_ros_bridge.py', 'run_hardware_in_loop.py'):
        assert os.access(prefix / 'lib/onrobot_gripper_isaac' / script, os.X_OK)
    helper = prefix / 'lib/onrobot_gripper_isaac/asset_visual_materials.py'
    assert helper.read_bytes() == (PACKAGE / 'scripts/asset_visual_materials.py').read_bytes()
    environment = dict(os.environ, ONROBOT_ISAAC_ASSET_REPOSITORY=str(tmp_path / 'absent'),
                       AMENT_PREFIX_PATH='', PYTHONPATH='')
    result = subprocess.run(
        ['/usr/bin/python3', str(prefix / 'lib/onrobot_gripper_isaac/isaac_model_contract.py')],
        env=environment, text=True, capture_output=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert Path(result.stdout.strip()) == share
    assert not (share / 'evidence').exists()
    assert not (share / 'config/release_support_matrix.json').exists()
    assert not list(share.rglob('__pycache__'))

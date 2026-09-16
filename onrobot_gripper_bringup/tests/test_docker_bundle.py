"""Exercise container input validation and entrypoint without a Docker daemon."""

import hashlib
import importlib.util
import json
import os
import re
import stat
import subprocess
from pathlib import Path

import pytest

from test_customer_bundle import _make_deb, _run_bundle

import yaml


DOCKER = Path(__file__).resolve().parents[2] / 'docker'
SPEC = importlib.util.spec_from_file_location(
    'onrobot_bundle_verifier', DOCKER / 'verify_bundle.py')
VERIFIER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VERIFIER)


def _rehash(bundle):
    files = sorted(path for path in bundle.rglob('*')
                   if path.is_file() and path.name != 'SHA256SUMS')
    (bundle / 'SHA256SUMS').write_text(''.join(
        f'{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.relative_to(bundle)}\n'
        for path in files))


def _manifest(bundle, update):
    path = bundle / 'BUNDLE_MANIFEST.json'
    document = json.loads(path.read_text())
    update(document)
    path.write_text(json.dumps(document))
    _rehash(bundle)


def test_generated_customer_bundle_is_accepted(tmp_path):
    """Validate a real bundle containing locally generated binary packages."""
    _, bundle, _ = _run_bundle(tmp_path)
    VERIFIER.verify(bundle)


@pytest.mark.parametrize('change', ['modified', 'extra', 'missing', 'symlink'])
def test_changed_bundle_fails_before_installation(tmp_path, change):
    """Reject a changed file inventory before any package can be installed."""
    _, bundle, _ = _run_bundle(tmp_path)
    readme = bundle / 'README.md'
    if change == 'modified':
        readme.write_text('changed')
    elif change == 'extra':
        (bundle / 'extra.txt').write_text('extra')
    elif change == 'missing':
        readme.unlink()
    else:
        (bundle / 'readme-link').symlink_to('README.md')
    with pytest.raises(ValueError):
        VERIFIER.verify(bundle)


@pytest.mark.parametrize('change', ['platform', 'version', 'asset_digest'])
def test_inconsistent_manifest_is_rejected_even_with_valid_hashes(tmp_path, change):
    """Check metadata consistency independently of the outer checksum list."""
    _, bundle, _ = _run_bundle(tmp_path)

    def update(document):
        if change == 'platform':
            document['target']['architecture'] = 'arm64'
        elif change == 'version':
            for entry in document['tool_api'].values():
                entry['version'] = '0.1.0-999'
        else:
            document['isaac_assets']['manifest_sha256'] = '0' * 64

    _manifest(bundle, update)
    with pytest.raises(ValueError):
        VERIFIER.verify(bundle)


def test_exact_development_runtime_dependency_is_required(tmp_path):
    """Reject a development package that permits a different ABI runtime."""
    _, bundle, _ = _run_bundle(tmp_path)
    development = _make_deb(
        bundle / 'tool-api-debs', 'libonrobot-tool-api-dev', '0.1.0-1',
        'libonrobot-tool-api0 (>= 0.1.0-1)')
    _manifest(bundle, lambda document: document['tool_api']['development'].update(
        sha256=hashlib.sha256(development.read_bytes()).hexdigest()))
    with pytest.raises(ValueError, match='pin the exact runtime'):
        VERIFIER.verify(bundle)


@pytest.mark.parametrize('extra', ['review/private.md', 'tool-api-debs/extra.deb'])
def test_non_public_or_extra_package_is_rejected(tmp_path, extra):
    """Do not accept private content or packages beyond the declared pair."""
    _, bundle, _ = _run_bundle(tmp_path)
    path = bundle / extra
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('not an approved input')
    _rehash(bundle)
    with pytest.raises(ValueError):
        VERIFIER.verify(bundle)


@pytest.mark.parametrize('entry', ['../outside', '/absolute', 'README.md'])
def test_unsafe_or_duplicate_checksum_paths_fail(tmp_path, entry):
    """Never follow checksum paths out of the bundle or hide duplicate names."""
    _, bundle, _ = _run_bundle(tmp_path)
    with (bundle / 'SHA256SUMS').open('a') as stream:
        stream.write(f'{"0" * 64}  {entry}\n')
    with pytest.raises(ValueError, match='unsafe or duplicate'):
        VERIFIER.verify(bundle)


@pytest.mark.parametrize('custom_paths', [False, True])
def test_entrypoint_sources_overlays_preserves_arguments_and_creates_private_paths(
        tmp_path, custom_paths):
    """Execute relocated entrypoint logic and preserve caller directory modes."""
    # Only relocate fixed container paths into the test's local filesystem.
    # Execute the actual entrypoint logic; no host ROS setup or Docker is required.
    base, overlay = tmp_path / 'base.bash', tmp_path / 'overlay.bash'
    base.write_text('export ONROBOT_TEST_BASE=loaded\n')
    overlay.write_text('test "$ONROBOT_TEST_BASE" = loaded\nexport ONROBOT_TEST_OVERLAY=loaded\n')
    entrypoint = tmp_path / 'entrypoint.sh'
    text = (DOCKER / 'entrypoint.sh').read_text()
    text = text.replace('/opt/ros/jazzy/setup.bash', str(base))
    text = text.replace('/opt/onrobot/setup.bash', str(overlay))
    text = text.replace('/tmp/onrobot-session.XXXXXX', str(tmp_path / 'session.XXXXXX'))
    entrypoint.write_text(text)
    environment = dict(os.environ)
    environment.pop('ROS_LOG_DIR', None)
    environment.pop('XDG_RUNTIME_DIR', None)
    if custom_paths:
        for key in ('ROS_LOG_DIR', 'XDG_RUNTIME_DIR'):
            path = tmp_path / key
            path.mkdir(mode=0o750)
            environment[key] = str(path)
    command = ('printf "%s\\n" "$ONROBOT_TEST_OVERLAY" "$ROS_LOG_DIR" '
               '"$XDG_RUNTIME_DIR" "$1"; exit 23')
    result = subprocess.run(
        ['bash', str(entrypoint), 'bash', '-c', command, 'test', 'space preserved'],
        env=environment, text=True, capture_output=True)
    assert result.returncode == 23, result.stderr
    loaded, log_dir, runtime_dir, argument = result.stdout.splitlines()
    assert (loaded, argument) == ('loaded', 'space preserved')
    for directory in (log_dir, runtime_dir):
        mode = stat.S_IMODE(Path(directory).stat().st_mode)
        assert mode == (0o750 if custom_paths else 0o700)


def test_documented_shell_blocks_have_valid_syntax():
    """Parse every public example without accessing a host display or device."""
    examples = re.findall(r'```bash\n(.*?)```', (DOCKER / 'README.md').read_text(), re.S)
    assert len(examples) == 6
    for example in examples:
        subprocess.run(['bash', '-n'], input=example, text=True, check=True)
    subprocess.run(['bash', '-n', str(DOCKER / 'entrypoint.sh')], check=True)


@pytest.mark.parametrize('missing', [False, True])
def test_ci_asset_preflight_runs_before_build(tmp_path, missing):
    """Execute only the offline asset-input portion of the actual CI commands."""
    root = DOCKER.parent
    pipeline = yaml.safe_load((root / 'bitbucket-pipelines.yml').read_text())
    script = pipeline['definitions']['steps'][0]['step']['script']
    commands = [line for line in script if 'ONROBOT_ISAAC_ASSET_REPOSITORY' in line]
    assert len(commands) == 3
    assert max(script.index(line) for line in commands) < script.index('apt-get update')
    environment = dict(os.environ)
    if missing:
        environment['ONROBOT_ISAAC_ASSET_REPOSITORY'] = str(tmp_path / 'missing')
    result = subprocess.run(
        ['bash', '-ec', '\n'.join(commands)], cwd=root, env=environment,
        text=True, capture_output=True)
    if missing:
        assert result.returncode != 0
        assert 'assets are missing' in result.stderr
    else:
        assert result.returncode == 0, result.stderr
        assert json.loads(result.stdout)['status'] == 'passed'

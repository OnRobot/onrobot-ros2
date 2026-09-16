"""Local, offline integration checks for the customer source bundle."""

from pathlib import Path
import json
import hashlib
import os
import shutil
import subprocess


ROS_ROOT = Path(__file__).resolve().parents[2]
BUNDLE_SCRIPT = ROS_ROOT / 'scripts' / 'create_customer_bundle.sh'


def _run(command, *, cwd, check=True):
    environment = dict(os.environ)
    # These commands build isolated miniature bundles. A CI-mounted production
    # asset checkout must not replace the fixture's own content-pinned files.
    environment.pop('ONROBOT_ISAAC_ASSET_REPOSITORY', None)
    return subprocess.run(
        command, cwd=cwd, check=check, text=True,
        env=environment,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def _make_deb(debs, package, version, depends=None, license_text=None):
    package_root = debs / f'{package}-root'
    control = package_root / 'DEBIAN'
    control.mkdir(parents=True)
    fields = [
        f'Package: {package}', f'Version: {version}',
        'Section: libs', 'Priority: optional', 'Architecture: amd64',
        'Maintainer: fixture <fixture@example.invalid>',
        'Description: local bundle fixture',
    ]
    if depends:
        fields.append(f'Depends: {depends}')
    (control / 'control').write_text('\n'.join(fields) + '\n', encoding='utf-8')
    if license_text is None:
        license_text = (ROS_ROOT / 'LICENSE').read_text(encoding='utf-8')
    if package == 'libonrobot-tool-api0':
        doc_dir = package_root / 'usr/share/doc/libonrobot-tool-api1'
        doc_dir.mkdir(parents=True)
        (doc_dir / 'copyright').write_text(license_text, encoding='utf-8')
    elif package == 'libonrobot-tool-api-dev':
        package_doc = package_root / 'usr/share/doc/libonrobot-tool-api-dev'
        package_doc.mkdir(parents=True)
        (package_doc / 'copyright').write_text(license_text, encoding='utf-8')
        sdk_doc = package_root / 'usr/share/doc/onrobot_tool_api'
        sdk_doc.mkdir(parents=True)
        (sdk_doc / 'LICENSE').write_text(license_text, encoding='utf-8')
        (sdk_doc / 'README.md').write_text(
            'The distributed binary SDK uses the BSD 3-Clause License.\n',
            encoding='utf-8')
    output = debs / f'{package}_{version}_amd64.deb'
    _run(['dpkg-deb', '--build', str(package_root), str(output)], cwd=debs)
    shutil.rmtree(package_root)
    return output


def _fixture_workspace(tmp_path):
    root = tmp_path / 'fixture-repository'
    scripts = root / 'scripts'
    scripts.mkdir(parents=True)
    shutil.copy2(BUNDLE_SCRIPT, scripts / BUNDLE_SCRIPT.name)
    os.chmod(scripts / BUNDLE_SCRIPT.name, 0o755)
    for name in ('README.md', 'GETTING_STARTED.md'):
        (root / name).write_text(f'fixture {name}\n', encoding='utf-8')
    shutil.copy2(ROS_ROOT / 'LICENSE', root / 'LICENSE')

    # The real script's repository_root is the ROS repository itself; the
    # bundle places that root below src/onrobot-ros2 during staging.
    source = root
    (source / 'onrobot_gripper_isaac' / 'assets' / '2fg7').mkdir(parents=True)
    (source / 'onrobot_gripper_isaac' / 'assets' / '2fg7' /
     'onrobot_2fg7.usda').write_text('#usda fixture\n', encoding='utf-8')
    (source / 'onrobot_gripper_isaac' / 'assets' / '2fg7' /
     'payload_mesh.obj').write_text('mesh fixture\n', encoding='utf-8')
    (source / 'onrobot_gripper_isaac' / 'assets' / '2fg7' /
     'payload_texture.png').write_bytes(b'fixture texture')
    # A local asset dependency may legitimately use a directory named
    # ``vendor``.  Only the repository-root vendor-demo location is excluded.
    asset_vendor = source / 'onrobot_gripper_isaac' / 'assets' / 'vendor'
    asset_vendor.mkdir(parents=True)
    (asset_vendor / 'mesh.usd').write_text('mesh dependency\n', encoding='utf-8')
    payloads = source / 'onrobot_gripper_isaac' / 'assets' / '2fg7' / 'payloads'
    payloads.mkdir()
    (payloads / 'base.usda').write_text(
        'asset references = @./mesh.usd@\n', encoding='utf-8')
    (payloads / 'mesh.usd').write_text('payload dependency\n', encoding='utf-8')
    (source / 'onrobot_gripper_isaac' / 'config').mkdir(parents=True)
    (source / 'onrobot_gripper_isaac' / 'config' /
     'profile.json').write_text('{}\n', encoding='utf-8')
    (source / 'onrobot_gripper_isaac' / 'doc').mkdir(parents=True)
    (source / 'onrobot_gripper_isaac' / 'doc' /
     'README.md').write_text('offline documentation\n', encoding='utf-8')
    curated_evidence = source / 'onrobot_gripper_isaac' / 'evidence'
    (curated_evidence / '2fg7').mkdir(parents=True)
    (curated_evidence / '2fg7' / 'kinematic.json').write_text(
        '{"status": "passed"}\n', encoding='utf-8')
    (curated_evidence / 'rg6.json').write_text(
        '{"status": "passed"}\n', encoding='utf-8')
    (curated_evidence / 'operator-notes.txt').write_text(
        'not shipped\n', encoding='utf-8')
    (source / 'onrobot_gripper_isaac' / 'CMakeLists.txt').write_text(
        'install(DIRECTORY assets config doc evidence resource DESTINATION share)\n',
        encoding='utf-8')
    (source / 'NOTICE').write_text('notice fixture\n', encoding='utf-8')
    (source / 'KNOWN_ISSUES.md').write_text(
        '# Internal release notes\n\nNot part of the customer bundle.\n',
        encoding='utf-8')

    # These names model state that must not cross the customer boundary.
    forbidden = {
        '.git/HEAD': 'ref: refs/heads/main\n',
        '.gitignore': '*.local\n',
        'backup/old.txt': 'backup\n',
        'notes.bak': 'editor backup\n',
        'notes.backup': 'editor backup\n',
        'notes.orig': 'editor backup\n',
        'README.md~': 'editor backup\n',
        'build/generated.txt': 'build\n',
        'install/generated.txt': 'install\n',
        'workspace/install/generated.txt': 'workspace install\n',
        'vendor/demo.txt': 'vendor demo\n',
        'internal/manual.md': 'internal manual\n',
        'docs-internal/ci.md': 'private release instructions\n',
        'other_package/doc-internal/report.md': 'private report\n',
        'reports/review.json': '{}\n',
        # Split the fixture address so the repository's public-text audit does
        # not mistake this test data for a shipped endpoint.
        'endpoints/hosts.txt': '10.45' + '.30.12\n',
        'credentials/token.txt': 'secret\n',
        'firmware/image.bin': 'firmware\n',
        'evidence/run.json': '{}\n',
        'other_package/evidence/run.json': '{}\n',
        'other_package/evidence/trace.txt': 'not shipped\n',
        'capture.log': 'recording\n',
    }
    for relative, content in forbidden.items():
        path = source / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding='utf-8')
    asset_repo = root / 'onrobot-isaac-sim'
    asset_repo.mkdir()
    shutil.move(source / 'onrobot_gripper_isaac/assets', asset_repo / 'assets')
    supplied_assets = Path(subprocess.check_output([
        '/usr/bin/python3', str(ROS_ROOT /
                               'onrobot_gripper_isaac/scripts/isaac_model_contract.py'),
    ], text=True).strip())
    for relative in ('tools/validate_assets.py', 'resource/onrobot-logo.png',
                     'README.md', 'LICENSE'):
        destination = asset_repo / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(supplied_assets / relative, destination)
    (asset_repo / 'internal').mkdir()
    (asset_repo / 'internal/review.txt').write_text('private asset notes')
    manifest = {'schema_version': 1, 'models': {}, 'files': {
        path.relative_to(asset_repo).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in (asset_repo / 'assets').rglob('*') if path.is_file()}}
    manifest_path = asset_repo / 'asset_manifest.json'
    manifest_path.write_text(json.dumps(manifest), encoding='utf-8')
    (source / 'onrobot_gripper_isaac/config/asset_repository.json').write_text(
        json.dumps({'manifest_sha256': hashlib.sha256(manifest_path.read_bytes()).hexdigest()}))
    (source / 'onrobot_gripper_isaac/scripts').mkdir()
    shutil.copy2(ROS_ROOT / 'onrobot_gripper_isaac/scripts/isaac_model_contract.py',
                 source / 'onrobot_gripper_isaac/scripts/isaac_model_contract.py')
    return root


def _run_bundle(tmp_path):
    root = _fixture_workspace(tmp_path)
    debs = tmp_path / 'debs'
    debs.mkdir()
    version = '0.1.0-1'
    _make_deb(debs, 'libonrobot-tool-api0', version)
    _make_deb(
        debs, 'libonrobot-tool-api-dev', version,
        'libonrobot-tool-api0 (= 0.1.0-1)')
    output = tmp_path / 'customer-bundle'
    result = _run([
        str(root / 'scripts' / 'create_customer_bundle.sh'),
        '--debs', str(debs), '--output', str(output),
    ], cwd=tmp_path)
    assert 'Created customer workspace' in result.stdout
    return root, output, result


def _fixture_debs(tmp_path, name='debs'):
    debs = tmp_path / name
    debs.mkdir()
    version = '0.1.0-1'
    _make_deb(debs, 'libonrobot-tool-api0', version)
    _make_deb(
        debs, 'libonrobot-tool-api-dev', version,
        'libonrobot-tool-api0 (= 0.1.0-1)')
    return debs


def _bundle_files(output):
    return {
        path.relative_to(output).as_posix()
        for path in output.rglob('*') if path.is_file()
    }


def test_bundle_filters_internal_state_and_keeps_asset_dependencies(tmp_path):
    root, output, _ = _run_bundle(tmp_path)
    files = _bundle_files(output)
    assert (root / 'KNOWN_ISSUES.md').is_file()
    assert 'src/onrobot-ros2/KNOWN_ISSUES.md' not in files
    generated_readme = (output / 'README.md').read_text(encoding='utf-8')
    assert 'KNOWN_ISSUES.md' not in generated_readme
    assert 'Internal release notes' not in generated_readme
    for forbidden in (
            'src/onrobot-ros2/.git/HEAD', 'src/onrobot-ros2/.gitignore',
            'src/onrobot-ros2/backup/old.txt',
            'src/onrobot-ros2/notes.bak',
            'src/onrobot-ros2/notes.backup',
            'src/onrobot-ros2/notes.orig',
            'src/onrobot-ros2/README.md~',
            'src/onrobot-ros2/onrobot-isaac-sim/internal/review.txt',
            'src/onrobot-ros2/build/generated.txt',
            'src/onrobot-ros2/install/generated.txt',
            'src/onrobot-ros2/workspace/install/generated.txt',
            'src/onrobot-ros2/vendor/demo.txt',
            'src/onrobot-ros2/internal/manual.md',
            'src/onrobot-ros2/docs-internal/ci.md',
            'src/onrobot-ros2/other_package/doc-internal/report.md',
            'src/onrobot-ros2/reports/review.json',
            'src/onrobot-ros2/endpoints/hosts.txt',
            'src/onrobot-ros2/credentials/token.txt',
            'src/onrobot-ros2/firmware/image.bin',
            'src/onrobot-ros2/evidence/run.json',
            'src/onrobot-ros2/other_package/evidence/run.json',
            'src/onrobot-ros2/other_package/evidence/trace.txt',
            'src/onrobot-ros2/capture.log',
    ):
        assert forbidden not in files
    assert not any(
        'evidence' in Path(relative).parts for relative in files)
    assert not any(
        part in {'test', 'tests'}
        for relative in files for part in Path(relative).parts)
    assert not any(
        part in {'docs-internal', 'doc-internal'}
        for relative in files for part in Path(relative).parts)
    for required in (
            'src/onrobot-ros2/onrobot-isaac-sim/assets/2fg7/onrobot_2fg7.usda',
            'src/onrobot-ros2/onrobot-isaac-sim/assets/2fg7/payload_mesh.obj',
            'src/onrobot-ros2/onrobot-isaac-sim/assets/2fg7/payload_texture.png',
            'src/onrobot-ros2/onrobot-isaac-sim/assets/vendor/mesh.usd',
            'src/onrobot-ros2/onrobot-isaac-sim/assets/2fg7/payloads/base.usda',
            'src/onrobot-ros2/onrobot-isaac-sim/assets/2fg7/payloads/mesh.usd',
            'src/onrobot-ros2/onrobot_gripper_isaac/config/profile.json',
            'src/onrobot-ros2/onrobot_gripper_isaac/doc/README.md',
            'src/onrobot-ros2/NOTICE', 'BUNDLE_MANIFEST.json', 'SHA256SUMS',
    ):
        assert required in files
    assert 'src/onrobot-ros2/onrobot_gripper_isaac/evidence/operator-notes.txt' not in files
    assert 'src/onrobot-ros2/onrobot_gripper_isaac/CMakeLists.txt' in files


def test_bundle_exposes_consistent_bsd_licenses_and_not_tool_api_source(tmp_path):
    """Check the license terms users receive in source and Debian artifacts."""
    _, output, _ = _run_bundle(tmp_path)
    ros_license = (output / 'src/onrobot-ros2/LICENSE').read_text(
        encoding='utf-8')
    asset_license = (output / 'src/onrobot-ros2/onrobot-isaac-sim/LICENSE').read_text(
        encoding='utf-8')
    assert ros_license.startswith('BSD 3-Clause License')
    assert ros_license == asset_license

    readme = (output / 'README.md').read_text(encoding='utf-8')
    assert 'BSD 3-Clause License' in readme
    assert 'Tool API implementation source is not included.' in readme
    assert 'does not certify functional safety' in readme
    assert 'beta' not in readme.lower()
    assert 'prototype' not in readme.lower()
    assert 'Prototype Evaluation License' not in readme
    assert 'not production or safety-critical use' not in readme

    manifest = json.loads((output / 'BUNDLE_MANIFEST.json').read_text())
    assert manifest['licenses'] == {
        'ros_source': 'BSD-3-Clause',
        'isaac_assets': 'BSD-3-Clause',
        'tool_api_binary_sdk': 'BSD-3-Clause',
        'tool_api_implementation_source': 'not distributed',
    }
    assert not (output / 'src/onrobot-tool-api').exists()

    extracted = tmp_path / 'extracted-tool-api'
    runtime_prefix = extracted / 'runtime'
    development_prefix = extracted / 'development'
    runtime_prefix.mkdir(parents=True)
    development_prefix.mkdir(parents=True)
    runtime_deb = next((output / 'tool-api-debs').glob(
        'libonrobot-tool-api0_*.deb'))
    development_deb = next((output / 'tool-api-debs').glob(
        'libonrobot-tool-api-dev_*.deb'))
    _run(['dpkg-deb', '--extract', str(runtime_deb), str(runtime_prefix)], cwd=tmp_path)
    _run(['dpkg-deb', '--extract', str(development_deb), str(development_prefix)], cwd=tmp_path)
    for relative in (
            'usr/share/doc/libonrobot-tool-api1/copyright',):
        assert (runtime_prefix / relative).read_text(encoding='utf-8') == ros_license
    for relative in (
            'usr/share/doc/libonrobot-tool-api-dev/copyright',
            'usr/share/doc/onrobot_tool_api/LICENSE'):
        assert (development_prefix / relative).read_text(
            encoding='utf-8') == ros_license
    installed_guide = (
        development_prefix / 'usr/share/doc/onrobot_tool_api/README.md'
    ).read_text(encoding='utf-8')
    assert 'BSD 3-Clause License' in installed_guide
    assert 'Prototype Evaluation License' not in installed_guide


def test_bundle_rejects_tool_api_deb_with_non_bsd_license(tmp_path):
    """Do not package DEBs whose user-visible license disagrees with ROS."""
    root = _fixture_workspace(tmp_path)
    debs = tmp_path / 'wrong-license-debs'
    debs.mkdir()
    version = '0.1.0-1'
    _make_deb(
        debs, 'libonrobot-tool-api0', version,
        license_text='OnRobot Prototype Evaluation License\n')
    _make_deb(
        debs, 'libonrobot-tool-api-dev', version,
        'libonrobot-tool-api0 (= 0.1.0-1)')
    output = tmp_path / 'rejected-license-bundle'
    result = _run([
        str(root / 'scripts' / BUNDLE_SCRIPT.name),
        '--debs', str(debs), '--output', str(output),
    ], cwd=tmp_path, check=False)
    assert result.returncode != 0
    assert 'Tool API package license does not match the BSD-3-Clause notice' in result.stderr
    assert not output.exists()
    assert not (tmp_path / 'rejected-license-bundle.tar.gz').exists()


def test_bundle_rejects_source_symlinks_without_dereferencing_targets(tmp_path):
    root = _fixture_workspace(tmp_path)
    external_target = tmp_path / 'outside-bundle-secret.txt'
    external_target.write_text('must never be copied\n', encoding='utf-8')
    (root / 'external-link.txt').symlink_to(external_target)
    (root / 'contained-link.txt').symlink_to(root / 'NOTICE')
    output = tmp_path / 'symlink-bundle'
    result = _run([
        str(root / 'scripts' / BUNDLE_SCRIPT.name),
        '--debs', str(_fixture_debs(tmp_path, 'symlink-debs')),
        '--output', str(output),
    ], cwd=tmp_path, check=False)
    assert result.returncode != 0
    assert 'source symlinks are unsupported' in result.stderr
    assert not output.exists()
    assert not (tmp_path / 'symlink-bundle.tar.gz').exists()


def test_bundle_rejects_output_inside_repository_and_realpath_alias(tmp_path):
    root = _fixture_workspace(tmp_path)
    debs = _fixture_debs(tmp_path, 'containment-debs')
    alias = tmp_path / 'repository-alias'
    alias.symlink_to(root, target_is_directory=True)
    for output in (root / 'nested-output', alias / 'aliased-output'):
        result = _run([
            str(root / 'scripts' / BUNDLE_SCRIPT.name),
            '--debs', str(debs), '--output', str(output),
        ], cwd=tmp_path, check=False)
        assert result.returncode != 0
        assert 'output must be outside repository' in result.stderr
        assert not output.exists()


def test_bundle_manifest_and_file_hashes_are_consistent_and_tamper_fails(tmp_path):
    _, output, _ = _run_bundle(tmp_path)
    manifest = json.loads((output / 'BUNDLE_MANIFEST.json').read_text())
    assert manifest['tool_api']['runtime']['version'] == '0.1.0-1'
    assert manifest['tool_api']['development']['version'] == '0.1.0-1'
    checked = _run(['sha256sum', '-c', 'SHA256SUMS'], cwd=output)
    assert 'OK' in checked.stdout

    (output / 'BUNDLE_MANIFEST.json').write_text('{"tampered": true}\n')
    tampered = _run(['sha256sum', '-c', 'SHA256SUMS'], cwd=output, check=False)
    assert tampered.returncode != 0
    assert 'BUNDLE_MANIFEST.json' in tampered.stdout


def test_bundle_rejects_existing_output_or_archive(tmp_path):
    _, output, _ = _run_bundle(tmp_path)
    debs = tmp_path / 'debs'
    script = output.parent / 'fixture-repository' / 'scripts' / BUNDLE_SCRIPT.name
    collision = _run([
        str(script), '--debs', str(debs), '--output', str(output),
    ], cwd=tmp_path, check=False)
    assert collision.returncode != 0
    assert 'output already exists' in collision.stderr

    other = tmp_path / 'archive-collision'
    (tmp_path / 'archive-collision.tar.gz').write_bytes(b'existing archive')
    collision = _run([
        str(script), '--debs', str(debs), '--output', str(other),
    ], cwd=tmp_path, check=False)
    assert collision.returncode != 0
    assert 'archive already exists' in collision.stderr


def test_real_source_does_not_stage_internal_evidence_or_tests(tmp_path):
    """Private reports and quality tests remain outside customer archives."""
    debs = tmp_path / 'real-debs'
    debs.mkdir()
    version = '0.1.0-1'
    _make_deb(debs, 'libonrobot-tool-api0', version)
    _make_deb(
        debs, 'libonrobot-tool-api-dev', version,
        'libonrobot-tool-api0 (= 0.1.0-1)')
    output = tmp_path / 'real-customer-bundle'
    result = _run([
        str(BUNDLE_SCRIPT), '--debs', str(debs), '--output', str(output),
    ], cwd=tmp_path)
    assert 'Created customer workspace' in result.stdout

    source_evidence = ROS_ROOT / 'onrobot_gripper_isaac' / 'evidence'
    install_text = (ROS_ROOT / 'onrobot_gripper_isaac' /
                    'CMakeLists.txt').read_text(encoding='utf-8')
    assert 'DIRECTORY config doc resource' in ' '.join(
        install_text.split())
    assert 'install(DIRECTORY "${asset_repository}/assets"' in install_text
    assert any(source_evidence.rglob('*.json'))
    files = _bundle_files(output)
    assert not any(
        'evidence' in Path(relative).parts for relative in files)
    assert not any(
        part in {'test', 'tests'}
        for relative in files for part in Path(relative).parts)

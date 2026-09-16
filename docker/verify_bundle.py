#!/usr/bin/env python3
"""Validate the supplied bundle before installing its binary Tool API."""

import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path


def verify(root):
    """Require a complete, public-only, matching amd64/Jazzy bundle."""
    root = root.resolve(strict=True)
    for name in ('SHA256SUMS', 'BUNDLE_MANIFEST.json'):
        if (root / name).is_symlink():
            raise ValueError(f'bundle symlink is unsupported: {name}')
    hashes = {}
    for line in (root / 'SHA256SUMS').read_text(encoding='utf-8').splitlines():
        match = re.fullmatch(r'([0-9a-f]{64})  (.+)', line)
        if not match:
            raise ValueError('invalid SHA256SUMS line')
        digest, name = match.groups()
        name = name.removeprefix('./')
        path = Path(name)
        if path.is_absolute() or '..' in path.parts or name in hashes:
            raise ValueError(f'unsafe or duplicate checksum path: {name}')
        hashes[name] = digest
    if not hashes or 'SHA256SUMS' in hashes:
        raise ValueError('invalid bundle checksum inventory')
    files = {}
    forbidden = {
        '.git', 'internal', 'firmware', 'review', 'credentials',
        'demos-with-different-robot-vendors', 'onrobot-tool-api',
    }
    for path in root.rglob('*'):
        if path.is_symlink():
            raise ValueError(f'bundle symlink is unsupported: {path}')
        name = path.relative_to(root)
        if forbidden.intersection(name.parts):
            raise ValueError(f'non-public bundle input: {name}')
        if path.is_file() and name.as_posix() != 'SHA256SUMS':
            files[name.as_posix()] = path
    if files.keys() != hashes.keys():
        raise ValueError('bundle files differ from SHA256SUMS inventory')
    for name, path in files.items():
        if hashlib.sha256(path.read_bytes()).hexdigest() != hashes[name]:
            raise ValueError(f'bundle checksum mismatch: {name}')
    manifest = json.loads((root / 'BUNDLE_MANIFEST.json').read_text())
    if manifest.get('schema_version') != 1 or manifest.get('target') != {
            'operating_system': 'Ubuntu 24.04', 'architecture': 'amd64',
            'ros_distribution': 'Jazzy'}:
        raise ValueError('expected an Ubuntu 24.04 amd64 / Jazzy bundle')
    if manifest['ros_source']['path'] != 'src/onrobot-ros2':
        raise ValueError('unexpected ROS source path')
    packages = manifest['tool_api']
    version = packages['runtime']['version']
    declared = []
    for role, expected_name in (
            ('runtime', 'libonrobot-tool-api0'),
            ('development', 'libonrobot-tool-api-dev')):
        entry = packages[role]
        relative = Path(entry['file'])
        if (relative.parent != Path('tool-api-debs') or
                entry['package'] != expected_name or
                entry['version'] != version or entry['architecture'] != 'amd64' or
                hashes.get(relative.as_posix()) != entry['sha256']):
            raise ValueError(f'inconsistent {role} package manifest')
        package = root / relative
        for field, expected in (
                ('Package', expected_name), ('Version', version),
                ('Architecture', 'amd64')):
            actual = subprocess.check_output(
                ['dpkg-deb', '--field', str(package), field], text=True).strip()
            if actual != expected:
                raise ValueError(f'{role} DEB {field} does not match manifest')
        dependencies = subprocess.check_output(
            ['dpkg-deb', '--field', str(package), 'Depends'], text=True)
        if role == 'development' and not re.search(
                r'(?:^|,)\s*libonrobot-tool-api0\s*\(=\s*' +
                re.escape(version) + r'\s*\)\s*(?:,|$)', dependencies):
            raise ValueError('development DEB must pin the exact runtime version')
        declared.append(package)
    if set((root / 'tool-api-debs').glob('*.deb')) != set(declared):
        raise ValueError('expected only the two declared Tool API DEBs')
    asset_root = root / 'src/onrobot-ros2/onrobot-isaac-sim'
    if (manifest['isaac_assets']['path'] != str(asset_root.relative_to(root)) or
            hashlib.sha256((asset_root / 'asset_manifest.json').read_bytes()).hexdigest()
            != manifest['isaac_assets']['manifest_sha256']):
        raise ValueError('asset repository does not match the bundle manifest')
    print(f'Validated bundle: Tool API {version}, Ubuntu 24.04 amd64, ROS 2 Jazzy')


def main():
    """Validate local files only; never download or install packages."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path)
    args = parser.parse_args()
    try:
        verify(args.bundle)
    except (ValueError, KeyError, TypeError, OSError, subprocess.CalledProcessError) as error:
        parser.exit(1, f'Invalid bundle: {error}\n')


if __name__ == '__main__':
    main()

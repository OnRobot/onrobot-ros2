#!/usr/bin/env python3
"""Load one installed or source-tree OnRobot Isaac model contract."""

import hashlib
import json
import math
import os
from datetime import date
from pathlib import Path, PurePosixPath
import re


SUPPORTED_MODELS = ('2fg7', '2fg14', 'rg2', 'rg6')
QUALIFICATION_MODELS = SUPPORTED_MODELS


def package_root() -> Path:
    """Return the package share/source root containing model contracts."""
    source_root = Path(__file__).resolve().parents[1]
    if (source_root / 'config').is_dir():
        return source_root
    install_prefix = Path(__file__).resolve().parents[2]
    return install_prefix / 'share' / 'onrobot_gripper_isaac'


def contract_path(model: str) -> Path:
    """Return the checked-in contract path for a supported model."""
    if model not in QUALIFICATION_MODELS:
        raise ValueError(
            f'unsupported Isaac model {model!r}; expected one of '
            f'{", ".join(QUALIFICATION_MODELS)}')
    return package_root() / 'config' / f'{model}_asset_contract.json'


def load_contract(model: str) -> dict:
    """Load and minimally identify a supported model contract."""
    path = contract_path(model)
    if not path.is_file():
        raise RuntimeError(f'Isaac model contract does not exist: {path}')
    contract = json.loads(path.read_text(encoding='utf-8'))
    if contract.get('model') != model:
        raise RuntimeError(
            f'Isaac contract {path} identifies model '
            f'{contract.get("model")!r}, expected {model!r}')
    return contract


def default_asset(model: str) -> Path:
    """Return the source or installed USD entry point for a model."""
    contract = load_contract(model)
    return asset_repository_root() / contract['asset_entrypoint']


def package_version(value: str) -> tuple[int, int, int, int]:
    """Compare NVIDIA package date/revision versions numerically."""
    if not isinstance(value, str) or not re.fullmatch(r'\d{4}\.\d{2}\.\d{2}_\d+', value):
        raise ValueError(f'invalid SimReady package version: {value!r}')
    calendar, revision = value.split('_')
    year, month, day = map(int, calendar.split('.'))
    date(year, month, day)
    if int(revision) < 1:
        raise ValueError('SimReady package revision must be positive')
    return year, month, day, int(revision)


def _manifest_path(repository: Path, relative: str) -> Path:
    """Reject paths that escape the asset tree, including symlink ancestors."""
    if (not isinstance(relative, str) or '\\' in relative or ':' in relative or
            not relative.startswith('assets/') or
            any(p in ('', '.', '..') for p in relative.split('/')) or
            PurePosixPath(relative).is_absolute()):
        raise ValueError(f'unsafe asset manifest path: {relative!r}')
    path = repository
    for part in relative.split('/'):
        path = path / part
        if path.is_symlink():
            raise ValueError(f'asset path must not contain symlinks: {relative}')
    if not path.is_file():
        raise ValueError(f'missing asset file: {relative}')
    return path


def compatible_manifest(repository: Path, root: Path) -> dict:
    """Check required model versions/interfaces, allowing compatible additions.

    File hashes are integrity/provenance records, not compatibility versions.
    The build and public asset validator separately verify the file contents.
    """
    try:
        manifest_path = repository / 'asset_manifest.json'
        if manifest_path.is_symlink():
            raise ValueError('asset manifest must not be a symlink')
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        requirements = json.loads((root / 'config/asset_repository.json').read_text(
            encoding='utf-8'))
        if requirements['schema_version'] != 2 or manifest['schema_version'] != 1:
            raise ValueError('unsupported asset dependency or manifest schema')
        if not isinstance(manifest['files'], dict) or not manifest['files']:
            raise ValueError('asset manifest is empty')
        if set(requirements['models']) != set(SUPPORTED_MODELS):
            raise ValueError('dependency requirements must identify all four supported models')
        for model, required in requirements['models'].items():
            info = manifest['models'][model]
            revision = info['interface_revision']
            if (type(revision) is not int or revision < 1 or
                    type(required['interface_revision']) is not int or
                    revision != required['interface_revision']):
                raise ValueError(
                    f'{model}: interface revision {revision!r} is unsupported; '
                    f'ROS requires {required["interface_revision"]}')
            catalog = manifest['catalog_packages'][model]
            identifier = catalog['package_id']
            match = re.fullmatch(
                rf'simready\.hf\.[^.]+\.[^.]+\.onrobot_{re.escape(model)}\.(.+)',
                identifier)
            if not match:
                raise ValueError(f'{model}: invalid SimReady package identity')
            version = match.group(1)
            if package_version(version) < package_version(required['minimum_version']):
                raise ValueError(
                    f'{model}: package {version} is too old; ROS requires '
                    f'at least {required["minimum_version"]}')
            contract = json.loads((root / 'config' / f'{model}_asset_contract.json').read_text())
            usd = contract['usd']
            expected = {
                'entrypoint': contract['asset_entrypoint'],
                'default_prim': '/' + usd['default_prim'],
                'articulation_root': usd['articulation_root'],
                'driven_joint_path': f'/{usd["default_prim"]}/Physics/{usd["actuated_joint"]}',
                'coordinate_dimension': usd.get('coordinate_dimension', 'linear'),
                'coordinate_unit': usd.get('coordinate_unit', 'm'),
            }
            for key, value in expected.items():
                if info.get(key) != value:
                    raise ValueError(f'{model}: incompatible {key}; expected {value!r}, got {info.get(key)!r}')
            if catalog['entrypoint'] != info['entrypoint']:
                raise ValueError(f'{model}: catalog/model entry points differ')
            if not info['entrypoint'].startswith(catalog['directory'] + '/simready_usd/'):
                raise ValueError(f'{model}: entry point is outside its catalog package')
            metadata_name = catalog['directory'] + '/com.nvidia.simready.packaging.json'
            for name in (info['entrypoint'], metadata_name):
                if name not in manifest['files']:
                    raise ValueError(f'{model}: required file absent from manifest: {name}')
                _manifest_path(repository, name)
            metadata = json.loads(_manifest_path(repository, metadata_name).read_text())
            if metadata['package_id'] != identifier:
                raise ValueError(f'{model}: package metadata identity differs from manifest')
        return manifest
    except (OSError, KeyError, TypeError, ValueError, AttributeError) as error:
        raise RuntimeError(
            f'Incompatible Isaac assets at {repository}: {error}. '
            'Use an asset release satisfying config/asset_repository.json and rebuild '
            'onrobot_gripper_isaac.') from error


def verify_asset_integrity(repository: Path, manifest: dict) -> None:
    """Verify current release bytes; no historical content pin is involved."""
    actual = set()
    for path in (repository / 'assets').rglob('*'):
        if path.is_symlink():
            raise ValueError(f'asset symlinks are not portable: {path}')
        if path.is_file():
            actual.add(path.relative_to(repository).as_posix())
    if actual != set(manifest['files']):
        raise ValueError('asset file set differs from the manifest')
    for name, expected in manifest['files'].items():
        path = _manifest_path(repository, name)
        content = path.read_bytes()
        if content.startswith(b'version https://git-lfs.github.com/spec/v1'):
            raise ValueError(f'unresolved Git LFS pointer: {name}')
        if hashlib.sha256(content).hexdigest() != expected:
            raise ValueError(f'asset checksum mismatch: {name}')


def asset_repository_root(root: Path | None = None,
                          repository: Path | None = None) -> Path:
    """Resolve installed assets or a compatible non-ROS repository.

    Installed packages are self-contained. Source checkouts accept the nested
    submodule or a sibling checkout, without scanning unrelated workspaces.
    A present but mismatched repository is an error, never a silent fallback.
    """
    root = Path(root) if root is not None else package_root()
    if (root / 'assets').is_dir():
        compatible_manifest(root, root)
        return root
    configured = repository or os.environ.get('ONROBOT_ISAAC_ASSET_REPOSITORY')
    candidates = ([Path(configured)] if configured else [
        root.parent / 'onrobot-isaac-sim',
        root.parent / 'onrobot-isaacsim',
        root.parent.parent / 'onrobot-isaac-sim',
        root.parent.parent / 'onrobot-isaacsim',
    ])
    for candidate in candidates:
        if not candidate.is_dir():
            continue
        compatible_manifest(candidate, root)
        return candidate.resolve()
    raise RuntimeError('Isaac assets are missing. Provide the onrobot-isaac-sim '
                       'submodule/sibling checkout, or set '
                       'ONROBOT_ISAAC_ASSET_REPOSITORY to its directory.')


def articulation_path(contract: dict) -> str:
    """Return the explicit PhysX articulation-root prim path."""
    return contract['usd']['articulation_root']


def driven_joint(contract: dict) -> str:
    """Return the single commanded physical articulation joint."""
    return contract['usd']['actuated_joint']


def joint_limits(contract: dict) -> tuple[float, float]:
    """Return physical joint limits in the joint's native SI unit."""
    usd = contract['usd']
    if 'lower_limit' in usd and 'upper_limit' in usd:
        return float(usd['lower_limit']), float(usd['upper_limit'])
    return float(usd['lower_limit_m']), float(usd['upper_limit_m'])


def coordinate_metadata(contract: dict) -> tuple[str, str]:
    """Return physical coordinate dimension and unit."""
    usd = contract['usd']
    return (usd.get('coordinate_dimension', 'linear'),
            usd.get('coordinate_unit', 'm'))


def validate_composed_interface(stage, contract: dict) -> None:
    """Check effective USD coordinates before creating the ROS control graph.

    This runs inside Isaac after composition, so an overriding layer cannot
    silently change units, joint limits or the commanded linkage. Drive gains
    and material/contact parameters remain the asset release's choice.
    """
    from pxr import UsdGeom, UsdPhysics
    usd = contract['usd']

    def require(condition, description):
        if not condition:
            raise RuntimeError(f'{contract["model"]}: incompatible composed asset: {description}')

    def close(actual, expected):
        return actual is not None and math.isfinite(float(actual)) and abs(float(actual) - expected) < 1e-6

    def vector_close(actual, expected):
        return (actual is not None and len(actual) == len(expected)
                and all(close(a, b) for a, b in zip(actual, expected)))

    def rotation_close(actual, expected):
        if actual is None:
            return False
        values = [actual.GetReal(), *actual.GetImaginary()]
        # q and -q encode the same orientation. Compare normalized rotations.
        size = math.sqrt(sum(float(v) ** 2 for v in values))
        expected_size = math.sqrt(sum(float(v) ** 2 for v in expected))
        if not math.isfinite(size) or size < 1e-12 or expected_size < 1e-12:
            return False
        values = [v / size for v in values]
        target = [v / expected_size for v in expected]
        return vector_close(values, target) or vector_close(values, [-v for v in target])

    require(stage and not stage.GetCompositionErrors(), 'composition failed')
    require(str(stage.GetDefaultPrim().GetPath()) == '/' + usd['default_prim'], 'default prim')
    require(UsdGeom.GetStageMetersPerUnit(stage) == 1.0, 'metre units required')
    roots = [str(p.GetPath()) for p in stage.Traverse()
             if 'PhysicsArticulationRootAPI' in p.GetPrimTypeInfo().GetAppliedAPISchemas()]
    require(roots == [usd['articulation_root']], 'articulation root')
    prefix = '/' + usd['default_prim'] + '/Physics/'
    expected_joints = usd['joint_interfaces']
    actual_joints = {str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)}
    require(actual_joints == {prefix + name for name in expected_joints}, 'joint inventory')
    for name, expected in expected_joints.items():
        prim = stage.GetPrimAtPath(prefix + name)
        require(prim.GetTypeName() == expected['type'], name + ' joint type')
        require(prim.GetAttribute('physics:jointEnabled').Get() is not False, name + ' disabled joint')
        require(prim.GetAttribute('physics:excludeFromArticulation').Get() is not True,
                name + ' excluded joint')
        for side in ('0', '1'):
            targets = [str(p) for p in prim.GetRelationship('physics:body' + side).GetTargets()]
            bodies = ['/' + usd['default_prim'] + (p if p != '/' else '')
                      for p in expected['body' + side]]
            require(targets == bodies and all(stage.GetPrimAtPath(p) for p in targets),
                    name + ' body' + side)
            for path in targets:
                # The RG world-fixed root targets the model Xform. Link targets
                # must remain dynamic rigid bodies for this joint graph to work.
                if path == '/' + usd['default_prim']:
                    continue
                body = stage.GetPrimAtPath(path)
                require(body.HasAPI(UsdPhysics.RigidBodyAPI)
                        and body.GetAttribute('physics:rigidBodyEnabled').Get() is not False
                        and body.GetAttribute('physics:kinematicEnabled').Get() is not True,
                        name + ' rigid body ' + path)
            require(vector_close(prim.GetAttribute('physics:localPos' + side).Get(),
                                 expected['local_position' + side]), name + ' local position' + side)
            require(rotation_close(prim.GetAttribute('physics:localRot' + side).Get(),
                                   expected['local_rotation' + side]), name + ' local rotation' + side)
        if 'axis' in expected:
            require(prim.GetAttribute('physics:axis').Get() == expected['axis'], name + ' joint axis')
            limits = [prim.GetAttribute('physics:' + limit + 'Limit').Get()
                      for limit in ('lower', 'upper')]
            require(vector_close(limits, expected['limits']), name + ' joint limits')
    angular = coordinate_metadata(contract)[0] == 'angular'
    joint_type = UsdPhysics.RevoluteJoint if angular else UsdPhysics.PrismaticJoint
    joint = stage.GetPrimAtPath(prefix + usd['actuated_joint'])
    require(joint and joint.IsA(joint_type), 'driven joint name/type')
    require(joint.HasAPI(UsdPhysics.DriveAPI, 'angular' if angular else 'linear'),
            'missing driven joint drive')
    require(joint.GetAttribute('physics:axis').Get() == 'Z', 'driven joint axis')
    scale = math.pi / 180.0 if angular else 1.0
    for attribute, value in zip(('physics:lowerLimit', 'physics:upperLimit'), joint_limits(contract)):
        actual = joint.GetAttribute(attribute).Get()
        require(actual is not None and close(actual * scale, value), 'driven ' + attribute)
    followers = (usd['mimic_joints'] if angular else [
        {'name': usd['follower_joint'], 'axis': 'rotX', 'gearing': -usd['mimic_multiplier']}])
    for follower in followers:
        prim = stage.GetPrimAtPath(prefix + follower['name'])
        require(prim and prim.IsA(UsdPhysics.Joint), 'missing follower ' + follower['name'])
        mimic = 'physxMimicJoint:' + follower['axis'] + ':'
        require('PhysxMimicJointAPI:' + follower['axis'] in
                prim.GetPrimTypeInfo().GetAppliedAPISchemas(), 'missing mimic schema')
        targets = [str(p) for p in prim.GetRelationship(mimic + 'referenceJoint').GetTargets()]
        require(targets == [str(joint.GetPath())], 'follower leader ' + follower['name'])
        require(close(prim.GetAttribute(mimic + 'gearing').Get(), follower['gearing']), 'mimic gearing')
        require(close(prim.GetAttribute(mimic + 'offset').Get(), 0.0), 'mimic offset')
        require(not any(str(api).startswith('PhysicsDriveAPI:') for api in prim.GetAppliedSchemas()),
                'independent follower drive')


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Locate compatible Isaac assets')
    parser.add_argument('--package-root', type=Path)
    parser.add_argument('--asset-repository', type=Path)
    args = parser.parse_args()
    print(asset_repository_root(args.package_root, args.asset_repository))

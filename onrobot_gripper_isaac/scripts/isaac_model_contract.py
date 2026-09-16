#!/usr/bin/env python3
"""Load one installed or source-tree OnRobot Isaac model contract."""

import json
import hashlib
import os
from pathlib import Path


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


def asset_repository_root(root: Path | None = None,
                          repository: Path | None = None) -> Path:
    """Resolve installed assets or the content-pinned non-ROS repository.

    Installed packages are self-contained. Source checkouts accept the nested
    submodule or a sibling checkout, without scanning unrelated workspaces.
    A present but mismatched repository is an error, never a silent fallback.
    """
    root = Path(root) if root is not None else package_root()
    if (root / 'assets').is_dir():
        return root
    configured = repository or os.environ.get('ONROBOT_ISAAC_ASSET_REPOSITORY')
    candidates = ([Path(configured)] if configured else [
        root.parent / 'onrobot-isaac-sim',
        root.parent.parent / 'onrobot-isaac-sim',
    ])
    pin = json.loads((root / 'config/asset_repository.json').read_text(
        encoding='utf-8'))
    for candidate in candidates:
        if not candidate.is_dir():
            continue
        manifest = candidate / 'asset_manifest.json'
        if not manifest.is_file() or hashlib.sha256(manifest.read_bytes()).hexdigest() != \
                pin['manifest_sha256']:
            raise RuntimeError(f'Isaac asset repository does not match the ROS '
                               f'content pin: {candidate}')
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


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Locate the pinned Isaac assets')
    parser.add_argument('--package-root', type=Path)
    parser.add_argument('--asset-repository', type=Path)
    args = parser.parse_args()
    print(asset_repository_root(args.package_root, args.asset_repository))

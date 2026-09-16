#!/usr/bin/env python3
"""Run an OnRobot gripper articulation test inside Isaac Sim."""

import argparse
import asyncio
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import traceback

from isaac_model_contract import articulation_path
from isaac_model_contract import coordinate_metadata
from isaac_model_contract import default_asset
from isaac_model_contract import driven_joint
from isaac_model_contract import joint_limits
from isaac_model_contract import load_contract
from isaac_model_contract import QUALIFICATION_MODELS

from isaac_runtime_compat import is_stage_loading
from isaac_runtime_compat import setup_simulation


def _value(array, row: int, column: int) -> float:
    values = array.numpy() if hasattr(array, 'numpy') else array
    return float(values[row, column])


def _sha256(path: Path) -> str:
    """Return a stable digest for the exact USD entry point under test."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _asset_manifest(entrypoint: Path) -> dict:
    """Hash every file in the layered asset beside the entry point."""
    root = entrypoint.resolve().parent
    return {
        str(path.relative_to(root)): _sha256(path)
        for path in sorted(root.rglob('*'))
        if path.is_file()
    }


def _articulation_probe(stage, root_path: str, articulation_api) -> dict:
    """Capture stage facts needed to diagnose tensor-view creation."""
    root = stage.GetPrimAtPath(root_path)
    standard_roots = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.HasAPI(articulation_api)
    ]
    if not root:
        return {
            'root_prim_valid': False,
            'standard_articulation_roots': standard_roots,
        }
    body0 = root.GetRelationship('physics:body0')
    body1 = root.GetRelationship('physics:body1')
    return {
        'root_prim_valid': True,
        'root_prim_type': root.GetTypeName(),
        'root_applied_schemas': list(root.GetAppliedSchemas()),
        'root_body0_targets': (
            [str(target) for target in body0.GetTargets()] if body0 else []),
        'root_body1_targets': (
            [str(target) for target in body1.GetTargets()] if body1 else []),
        'standard_articulation_roots': standard_roots,
    }


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--model', choices=QUALIFICATION_MODELS, required=True)
    parser.add_argument('--asset', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--physics-dt', type=float, default=1.0 / 120.0)
    parser.add_argument('--settle-steps', type=int, default=360)
    parser.add_argument('--tolerance', type=float, default=0.001)
    args = parser.parse_args(argv)
    if (not math.isfinite(args.physics_dt) or not math.isfinite(args.tolerance) or
            args.physics_dt <= 0 or args.tolerance <= 0 or args.settle_steps < 1):
        parser.error('finite positive physics dt, tolerance and settle steps required')
    return args


async def run(args):
    """Load the supplied asset and verify endpoint convergence."""
    import omni.kit.app
    import omni.timeline
    app = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()
    contract = load_contract(args.model)
    if args.asset is None:
        args.asset = default_asset(args.model)
    root_path = articulation_path(contract)
    joint_name = driven_joint(contract)

    report = {
        'model': args.model,
        'asset': str(args.asset.resolve()),
        'physics_backend': 'physx',
        'articulation_path': root_path,
        'physics_dt_s': args.physics_dt,
        'status': 'failed',
        'tests': [],
    }
    try:
        import omni.usd
        from pxr import UsdPhysics
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.core.version import get_version

        report['isaac_sim_version'] = get_version()[0]
        if not args.asset.is_file():
            raise RuntimeError(f'asset does not exist: {args.asset}')
        report['asset_files_sha256'] = _asset_manifest(args.asset)
        if (args.physics_dt <= 0.0 or args.settle_steps < 1 or
                args.tolerance <= 0.0):
            raise RuntimeError(
                'physics dt, settle steps, and tolerance must be positive')

        timeline.stop()
        await app.next_update_async()
        await omni.usd.get_context().open_stage_async(str(args.asset.resolve()))
        while is_stage_loading():
            await app.next_update_async()

        stage = omni.usd.get_context().get_stage()
        articulation_variant = contract.get(
            'collision_model', {}).get(
                'unconstrained_articulation_test_variant')
        if articulation_variant:
            variant_set = stage.GetPrimAtPath(root_path).GetVariantSet(
                'Physics')
            if not variant_set.SetVariantSelection(articulation_variant):
                raise RuntimeError(
                    'could not select unconstrained articulation variant '
                    f'{articulation_variant}')
            await app.next_update_async()
            while is_stage_loading():
                await app.next_update_async()
            report['physics_variant'] = articulation_variant
            report['endpoint_scope'] = (
                'unconstrained-articulation; default contact variant is '
                'qualified separately')
        report['articulation_probe'] = _articulation_probe(
            stage, root_path, UsdPhysics.ArticulationRootAPI)

        setup_simulation(
            SimulationManager, dt=args.physics_dt, device='cpu')
        # The entry-point prim also carries the Newton articulation schema.
        # Resolve the PhysX articulation explicitly so Isaac does not select
        # the entry-point prim when searching the subtree for an API root.
        articulation = Articulation(root_path)

        # Let PhysX initialize the tensor view before querying DOF metadata.
        # The pre-play USD fallback infers type from drive attributes and
        # therefore labels the deliberately undriven mimic DOF as Invalid.
        timeline.play()
        for _ in range(4):
            await app.next_update_async()
        timeline.pause()
        await app.next_update_async()

        try:
            dof_names = articulation.dof_names
        except Exception as error:
            raise RuntimeError(
                f'PhysX could not create an articulation tensor view at '
                f'{root_path}; inspect articulation_probe for root schema '
                f'and body targets ({type(error).__name__}: {error})'
            ) from error
        if not dof_names:
            raise RuntimeError(
                f'PhysX did not create an articulation at {root_path}; '
                'verify the asset has exactly one Physics articulation root')
        if joint_name not in dof_names:
            raise RuntimeError(
                f'{joint_name} missing from runtime DOFs: '
                f'{dof_names}')
        driven_index = dof_names.index(joint_name)
        follower_names = []
        if 'follower_joint' in contract['usd']:
            follower_names.append(contract['usd']['follower_joint'])
        follower_names.extend(
            item['name'] for item in contract['usd'].get('mimic_joints', []))
        follower_indices = {
            name: dof_names.index(name)
            for name in follower_names if name in dof_names
        }
        report['dof_names'] = list(dof_names)
        report['coordinate_dimension'], report['coordinate_unit'] = (
            coordinate_metadata(contract))
        report['runner_sha256'] = _sha256(Path(__file__))
        time_origin = float(SimulationManager.get_simulation_time())
        step_count = 0

        started = time.monotonic()
        lower, upper = joint_limits(contract)
        targets = (
            ('closed', lower),
            ('midpoint', (lower + upper) / 2.0),
            ('open', upper),
        )
        for label, target in targets:
            for _ in range(args.settle_steps):
                articulation.set_dof_position_targets(
                    target, dof_indices=[driven_index])
                SimulationManager.step()
                step_count += 1
                if abs(SimulationManager.get_simulation_time() - time_origin - step_count * args.physics_dt) > 1e-5:
                    raise RuntimeError('physics time differs from requested steps')
                if step_count % 8 == 0:
                    await app.next_update_async()
            positions = articulation.get_dof_positions()
            measured = _value(positions, 0, driven_index)
            followers = {
                name: _value(positions, 0, index)
                for name, index in follower_indices.items()
            }
            target_error = abs(measured - target)
            result = {
                'name': label,
                'target': target,
                'measured': measured,
                'target_error': target_error,
                'followers': followers,
                'passed': (all(math.isfinite(v) for v in [measured, *followers.values()])
                           and target_error <= args.tolerance),
            }
            if contract['model'].startswith('2fg') and followers:
                follower = next(iter(followers.values()))
                mimic_error = abs(follower - measured)
                result['mimic_error'] = mimic_error
                result['passed'] = (
                    result['passed'] and mimic_error <= args.tolerance)
            report['tests'].append(result)
        report['elapsed_s'] = time.monotonic() - started
        report['status'] = (
            'passed' if all(item['passed'] for item in report['tests'])
            else 'failed')
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
        report['traceback'] = traceback.format_exc().splitlines()
    finally:
        timeline.stop()
        # Isaac Sim's fast shutdown can terminate the interpreter from
        # app.close(). Persist the evidence and pass the intended process
        # status to close() before taking that path.
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')
    return report


def main() -> int:
    args = arguments()
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'width': 1280, 'height': 720})
    task = asyncio.ensure_future(run(args))
    while not task.done():
        app.update()
    report = task.result()
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    exit_code = 0 if report['status'] == 'passed' else 1
    from isaac_runtime_compat import close_app
    close_app(app, exit_code)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())

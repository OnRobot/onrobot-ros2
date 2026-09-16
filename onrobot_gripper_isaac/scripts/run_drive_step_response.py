#!/usr/bin/env python3
"""Measure an unloaded product-asset position step inside Isaac Sim."""

import argparse
import asyncio
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
import traceback

from isaac_model_contract import (
    articulation_path, default_asset, driven_joint, joint_limits,
    load_contract, SUPPORTED_MODELS,
)

from isaac_runtime_compat import close_app
from isaac_runtime_compat import play, setup_simulation, stop

HARNESS_REVISION = 4


def _package_path(relative: str) -> Path:
    source = Path(__file__).resolve().parents[1] / relative
    if source.is_file():
        return source
    return (Path(__file__).resolve().parents[2] / 'share' /
            'onrobot_gripper_isaac' / relative)


def _write(path: Path, report: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _asset_manifest(asset: Path) -> dict:
    root = asset.resolve().parent
    return {
        str(path.relative_to(root)): _sha256(path)
        for path in sorted(root.rglob('*.usd*')) if path.is_file()
    }


def _array(value):
    return value.numpy() if hasattr(value, 'numpy') else value


def _value(value, index: int) -> float:
    return float(_array(value)[0, index])


def _coordinate_to_joint(model: str, value: float, contract: dict) -> float:
    if model.startswith('2fg'):
        return value / 2000.0
    _, upper = joint_limits(contract)
    maximum_mm = float(contract['ros']['task_maximum_m']) * 1000.0
    ratio = max(0.0, min(1.0, value / maximum_mm))
    return math.asin(ratio * math.sin(upper))


def _joint_to_coordinate(model: str, value: float, contract: dict) -> float:
    if model.startswith('2fg'):
        return value * 2000.0
    _, upper = joint_limits(contract)
    maximum_mm = float(contract['ros']['task_maximum_m']) * 1000.0
    return maximum_mm * math.sin(value) / math.sin(upper)


def _joint_velocity_to_reported(
        model: str, value: float) -> tuple[float, str]:
    if model.startswith('2fg'):
        return value * 2000.0, 'mm/s'
    return math.degrees(value), 'deg/s'


def _crossing(samples: list[dict], initial: float, final: float,
              fraction: float) -> float | None:
    direction = 1.0 if final > initial else -1.0
    threshold = initial + fraction * (final - initial)
    for sample in samples:
        if direction * (sample['measured'] - threshold) >= 0.0:
            return sample['time_s']
    return None


def _measure_step(samples: list[dict], commanded: float) -> dict:
    if not samples or not math.isfinite(commanded) or not all(
            math.isfinite(sample[field]) for sample in samples
            for field in ('time_s', 'measured', 'velocity')):
        raise ValueError('step response requires finite observations')
    if samples[0]['time_s'] < 0 or any(
            b['time_s'] <= a['time_s'] for a, b in zip(samples, samples[1:])):
        raise ValueError('step response times must increase strictly')
    initial = samples[0]['measured']
    final = statistics.median(
        sample['measured'] for sample in samples[-20:])
    response = final - initial
    response_detected = abs(response) > 1e-9
    time_10_percent = _crossing(samples, initial, final, 0.1) if response_detected else None
    time_50_percent = _crossing(samples, initial, final, 0.5) if response_detected else None
    time_90_percent = _crossing(samples, initial, final, 0.9) if response_detected else None
    direction = 1.0 if response > 0.0 else -1.0
    overshoot = max(
        0.0,
        max(direction * (sample['measured'] - final)
            for sample in samples),
    )
    tolerance = max(abs(response) * 0.05, 1e-6)
    settling = None
    for index, sample in enumerate(samples if response_detected else []):
        if all(abs(later['measured'] - final) <= tolerance
               for later in samples[index:]):
            settling = sample['time_s']
            break
    return {
        'commanded_initial': initial,
        'commanded_target': commanded,
        'measured_steady_state': final,
        'measured_response': response,
        'response_detected': response_detected,
        'command_endpoint_error': final - commanded,
        'time_to_10_percent_s': time_10_percent,
        'time_to_50_percent_s': time_50_percent,
        'time_to_90_percent_s': time_90_percent,
        'rise_time_10_to_90_s': (
            time_90_percent - time_10_percent
            if time_10_percent is not None and time_90_percent is not None
            else None),
        'settling_time_5_percent_s': settling,
        'overshoot': overshoot,
        'peak_absolute_velocity': max(
            abs(sample['velocity']) for sample in samples),
        'samples': samples,
    }


def _summarize(steps: list[dict]) -> dict:
    fields = (
        'time_to_10_percent_s',
        'time_to_50_percent_s',
        'time_to_90_percent_s',
        'rise_time_10_to_90_s',
        'settling_time_5_percent_s',
        'overshoot',
        'peak_absolute_velocity',
        'command_endpoint_error',
    )
    summary = {}
    for direction in ('opening', 'closing'):
        selected = [step for step in steps if step['direction'] == direction]
        values = {'steps': len(selected)}
        for field in fields:
            available = [step[field] for step in selected
                         if step[field] is not None]
            values[f'median_{field}'] = (
                statistics.median(available) if available else None)
        summary[direction] = values
    return summary


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=SUPPORTED_MODELS, required=True)
    parser.add_argument('--asset', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--drive-armature-kg-m2', type=float,
                        help='Diagnostic RG primary-joint reflected inertia override')
    parser.add_argument('--drive-armature-kg', type=float,
                        help='Diagnostic 2FG primary-joint reflected inertia override')
    parser.add_argument('--drive-damping-si', type=float,
                        help='Diagnostic drive damping override (N s/m or N m s/rad)')
    parser.add_argument('--start-joint', type=float, help='Optional physical-joint start (m or rad)')
    parser.add_argument('--target-joint', type=float, help='Optional physical-joint target (m or rad)')
    parser.add_argument('--maximum-joint-velocity-rad-s', type=float,
                        help='Diagnostic reduced RG velocity ceiling')
    parser.add_argument('--maximum-joint-velocity-m-s', type=float,
                        help='Diagnostic reduced 2FG one-finger velocity ceiling')
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error('output already exists; choose a new report path')
    for value in (args.drive_armature_kg_m2, args.drive_armature_kg, args.drive_damping_si):
        if value is not None and (not math.isfinite(value) or value < 0):
            parser.error('diagnostic values must be finite and nonnegative')
    if args.drive_armature_kg_m2 is not None and not args.model.startswith('rg'):
        parser.error('rotational armature applies only to RG models')
    if args.drive_armature_kg is not None and not args.model.startswith('2fg'):
        parser.error('linear armature applies only to 2FG models')
    if (args.start_joint is None) != (args.target_joint is None):
        parser.error('provide both physical-joint endpoints')
    if any(v is not None and not math.isfinite(v) for v in (args.start_joint, args.target_joint)):
        parser.error('physical-joint endpoints must be finite')
    if args.maximum_joint_velocity_rad_s is not None and (
            not args.model.startswith('rg') or not math.isfinite(args.maximum_joint_velocity_rad_s)
            or args.maximum_joint_velocity_rad_s <= 0):
        parser.error('RG velocity ceiling must be positive and finite')
    if args.maximum_joint_velocity_m_s is not None and (
            not args.model.startswith('2fg') or not math.isfinite(args.maximum_joint_velocity_m_s)
            or args.maximum_joint_velocity_m_s <= 0):
        parser.error('2FG velocity ceiling must be positive and finite')
    return args


async def run(args) -> dict:
    """Measure a free response without owning the existing Kit app lifetime."""

    contract = load_contract(args.model)
    asset = args.asset or default_asset(args.model)
    config_path = args.config or _package_path(
        'config/drive_step_response_profiles.json')
    report = {
        'schema_version': 1,
        'harness_revision': HARNESS_REVISION,
        'test': 'unloaded-drive-step-response',
        'model': args.model,
        'asset': str(asset.resolve()),
        'config': str(config_path.resolve()),
        'physics_backend': 'physx',
        'fidelity_claim': 'observational-only',
        'status': 'starting',
        'steps': [],
    }
    _write(args.output, report)
    try:
        config_bytes = config_path.read_bytes()
        config = json.loads(config_bytes.decode('utf-8'))
        if (config.get('schema_version') != 1 or
                config.get('test') != report['test'] or
                args.model not in config.get('models', {})):
            raise RuntimeError('drive-step configuration is invalid')
        profile = config['models'][args.model]
        requested_physics_dt = float(config['physics_dt_s'])
        settle_duration = float(config['settle_duration_s'])
        response_duration = float(config['response_duration_s'])
        repetitions = int(config['repetitions'])
        if (not all(math.isfinite(v) for v in
                    (requested_physics_dt, settle_duration, response_duration)) or
                requested_physics_dt <= 0.0 or settle_duration <= 0.0 or
                response_duration <= 0.0 or repetitions < 1):
            raise RuntimeError('drive-step timing configuration is invalid')

        report['config_sha256'] = hashlib.sha256(config_bytes).hexdigest()
        report['runner_sha256'] = _sha256(Path(__file__).resolve())
        report['asset_files_sha256'] = _asset_manifest(asset)
        report['asset_revision'] = contract['asset_revision']
        report['coordinate'] = profile['coordinate']
        report['coordinate_unit'] = 'mm'
        report['hardware_reference'] = profile['hardware_reference']
        report['requested_physics_dt_s'] = requested_physics_dt
        report['repetitions'] = repetitions

        import omni.kit.app
        import omni.timeline
        import omni.usd
        from pxr import Gf, PhysxSchema, UsdPhysics
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.core.version import get_version

        report['isaac_sim_version'] = get_version()[0]
        if not asset.is_file():
            raise RuntimeError(f'asset does not exist: {asset}')
        opened, _ = await omni.usd.get_context().open_stage_async(str(asset.resolve()))
        if not opened:
            raise RuntimeError(f'Isaac Sim could not open {asset}')
        kit = omni.kit.app.get_app()
        await kit.next_update_async()
        # A reusable robot asset need not own the world PhysicsScene.  Create
        # one in the qualification stage when the asset is deliberately
        # scene-free, as is the case for the composable 2FG14 asset.
        stage = omni.usd.get_context().get_stage()
        scene_prim = stage.GetPrimAtPath('/PhysicsScene')
        if not scene_prim.IsValid():
            scene_prim = UsdPhysics.Scene.Define(
                stage, '/PhysicsScene').GetPrim()
        scene = UsdPhysics.Scene(scene_prim)
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        PhysxSchema.PhysxSceneAPI.Apply(
            scene_prim).CreateEnableCCDAttr().Set(True)
        joint = next((prim for prim in stage.Traverse()
                      if prim.GetName() == driven_joint(contract)
                      and prim.IsA(UsdPhysics.Joint)), None)
        if joint is None:
            raise RuntimeError('driven joint prim missing')
        kind = 'angular' if args.model.startswith('rg') else 'linear'
        drive = UsdPhysics.DriveAPI(joint, kind)
        joint_api = PhysxSchema.PhysxJointAPI.Apply(joint)
        if args.drive_armature_kg_m2 is not None:
            joint_api.CreateArmatureAttr().Set(args.drive_armature_kg_m2)
        if args.drive_armature_kg is not None:
            joint_api.CreateArmatureAttr().Set(args.drive_armature_kg)
        if args.drive_damping_si is not None:
            drive.GetDampingAttr().Set(args.drive_damping_si *
                                      (math.pi / 180 if kind == 'angular' else 1))
        if args.maximum_joint_velocity_rad_s is not None:
            if args.maximum_joint_velocity_rad_s > math.radians(joint_api.GetMaxJointVelocityAttr().Get()) + 1e-6:
                raise ValueError('diagnostic velocity ceiling may only reduce the asset limit')
            joint_api.GetMaxJointVelocityAttr().Set(math.degrees(args.maximum_joint_velocity_rad_s))
        if args.maximum_joint_velocity_m_s is not None:
            if args.maximum_joint_velocity_m_s > joint_api.GetMaxJointVelocityAttr().Get() + 1e-6:
                raise ValueError('diagnostic velocity ceiling may only reduce the asset limit')
            joint_api.GetMaxJointVelocityAttr().Set(args.maximum_joint_velocity_m_s)
        report['drive'] = {
            'stiffness_usd': drive.GetStiffnessAttr().Get(),
            'damping_usd': drive.GetDampingAttr().Get(),
            'maximum_effort': drive.GetMaxForceAttr().Get(),
            'armature': joint_api.GetArmatureAttr().Get(),
            'maximum_joint_velocity_usd': joint_api.GetMaxJointVelocityAttr().Get(),
            'diagnostic_overrides': {
                'armature_kg_m2': args.drive_armature_kg_m2,
                'armature_kg': args.drive_armature_kg,
                'damping_si': args.drive_damping_si,
                'maximum_joint_velocity_rad_s': args.maximum_joint_velocity_rad_s,
                'maximum_joint_velocity_m_s': args.maximum_joint_velocity_m_s},
        }
        report['coordinate_mapping'] = (
            'symmetric stroke: physical joint x2' if kind == 'linear' else
            'legacy normalized aperture approximation; use physical_joint_position for identification')
        await kit.next_update_async()
        physics_dt = setup_simulation(
            SimulationManager, dt=requested_physics_dt, device='cpu')
        settle_steps = round(settle_duration / physics_dt)
        response_steps = round(response_duration / physics_dt)
        if settle_steps < 1 or response_steps < 20:
            raise RuntimeError(
                'runtime physics step is too coarse for the drive-step test')
        report['physics_dt_s'] = physics_dt
        report['settle_duration_s'] = settle_steps * physics_dt
        report['response_duration_s'] = response_steps * physics_dt
        articulation = Articulation(articulation_path(contract))
        play()
        for _ in range(2):
            await kit.next_update_async()
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        await kit.next_update_async()
        manual_started = SimulationManager.get_simulation_time()
        manual_steps = 0

        async def check_clock():
            error = abs(SimulationManager.get_simulation_time() - manual_started
                        - manual_steps * physics_dt)
            if error > 1e-5:
                raise RuntimeError(f'unexpected automatic physics stepping: {error}')
            if manual_steps % 8 == 0:
                await kit.next_update_async()
        if driven_joint(contract) not in articulation.dof_names:
            raise RuntimeError(
                f'{driven_joint(contract)} missing from runtime DOFs')
        joint_index = articulation.dof_names.index(driven_joint(contract))

        start = float(profile['start'])
        target = float(profile['target'])
        start_joint = _coordinate_to_joint(args.model, start, contract)
        target_joint = _coordinate_to_joint(args.model, target, contract)
        if args.start_joint is not None:
            start = start_joint = args.start_joint
            target = target_joint = args.target_joint
            report['coordinate'] = 'physical_joint'
            report['coordinate_unit'] = 'rad' if kind == 'angular' else 'm'
            report['coordinate_mapping'] = 'direct physical joint endpoints; no aperture approximation'

        def reported_position(value):
            return value if args.start_joint is not None else _joint_to_coordinate(args.model, value, contract)

        def reported_velocity(value):
            if args.start_joint is not None:
                return value, 'rad/s' if kind == 'angular' else 'm/s'
            return _joint_velocity_to_reported(args.model, value)
        lower, upper = joint_limits(contract)
        if not (
                lower <= start_joint <= upper and
                lower <= target_joint <= upper and
                start_joint != target_joint):
            raise RuntimeError('drive-step targets are outside asset limits')

        for _ in range(settle_steps):
            articulation.set_dof_position_targets(
                [start_joint], dof_indices=[joint_index])
            SimulationManager.step()
            manual_steps += 1
            await check_clock()

        for repetition in range(repetitions):
            for commanded, commanded_joint, direction in (
                    (target, target_joint, 'opening'),
                    (start, start_joint, 'closing')):
                step_started = SimulationManager.get_simulation_time()
                initial_position = _value(
                    articulation.get_dof_positions(), joint_index)
                initial_velocity = _value(
                    articulation.get_dof_velocities(), joint_index)
                velocity_value, velocity_unit = reported_velocity(initial_velocity)
                samples = [{
                    'time_s': 0.0,
                    'commanded': commanded,
                    'measured': reported_position(initial_position),
                    'physical_joint_position': initial_position,
                    'physical_joint_velocity': initial_velocity,
                    'velocity': velocity_value,
                }]
                for _ in range(response_steps):
                    articulation.set_dof_position_targets(
                        [commanded_joint], dof_indices=[joint_index])
                    SimulationManager.step()
                    manual_steps += 1
                    await check_clock()
                    position = _value(
                        articulation.get_dof_positions(), joint_index)
                    velocity = _value(
                        articulation.get_dof_velocities(), joint_index)
                    velocity_value, velocity_unit = reported_velocity(velocity)
                    samples.append({
                        'time_s': (
                            SimulationManager.get_simulation_time() -
                            step_started),
                        'commanded': commanded,
                        'measured': reported_position(position),
                        'physical_joint_position': position,
                        'physical_joint_velocity': velocity,
                        'velocity': velocity_value,
                    })
                result = _measure_step(samples, commanded)
                native = [dict(time_s=s['time_s'],
                               measured=s['physical_joint_position'],
                               velocity=s['physical_joint_velocity']) for s in samples]
                result['physical_joint_metrics'] = {
                    k: v for k, v in _measure_step(native, commanded_joint).items()
                    if k != 'samples'}
                result['physical_joint_unit'] = 'rad' if kind == 'angular' else 'm'
                result['name'] = f'{direction}_{repetition + 1}'
                result['direction'] = direction
                result['velocity_unit'] = velocity_unit
                report['steps'].append(result)
        report['manual_step_count'] = manual_steps
        report['manual_step_timing_error_s'] = abs(
            SimulationManager.get_simulation_time() - manual_started - manual_steps * physics_dt)
        report['summary'] = _summarize(report['steps'])
        report['status'] = 'passed'
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
        report['traceback'] = traceback.format_exc().splitlines()
        report['status'] = 'failed'
    finally:
        _write(args.output, report)
        try:
            stop()
        except Exception:
            pass
    return report


def main() -> int:
    args = arguments()
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'renderer': 'MinimalRendering',
                         'disable_viewport_updates': True, 'multi_gpu': False})
    task = asyncio.ensure_future(run(args))
    while not task.done():
        app.update()
    report = task.result()
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    exit_code = 0 if report['status'] == 'passed' else 1
    close_app(app, exit_code)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())

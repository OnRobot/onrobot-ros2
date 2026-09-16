#!/usr/bin/env python3
"""Stress RG-family fingertip contact and verify linkage recovery."""

import argparse
import asyncio
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback

from isaac_model_contract import articulation_path, default_asset
from isaac_model_contract import driven_joint, joint_limits, load_contract

from isaac_runtime_compat import is_stage_loading, setup_simulation


def _row(values) -> list[float]:
    array = values.numpy() if hasattr(values, 'numpy') else values
    return [float(value) for value in array[0]]


def _rows(values) -> list[list[float]]:
    array = values.numpy() if hasattr(values, 'numpy') else values
    return [[float(value) for value in row] for row in array]


def _flat(values) -> list[float]:
    array = values.numpy() if hasattr(values, 'numpy') else values
    return [float(value) for value in array.reshape(-1)]


def _pose_error(reference, actual) -> tuple[float, float]:
    reference_position, reference_orientation = reference
    actual_position, actual_orientation = actual
    position_error = math.sqrt(sum(
        (actual_position[index] - reference_position[index]) ** 2
        for index in range(3)))
    quaternion_dot = abs(sum(
        actual_orientation[index] * reference_orientation[index]
        for index in range(4)))
    quaternion_dot = min(1.0, max(-1.0, quaternion_dot))
    return position_error, 2.0 * math.acos(quaternion_dot)


def arguments(argv=None):
    """Parse the same options for standalone and embedded execution."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=('rg2', 'rg6'), default='rg2')
    parser.add_argument('--asset', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cycles', type=int, default=5)
    parser.add_argument('--travel-steps', type=int, default=480)
    parser.add_argument('--contact-steps', type=int, default=480)
    parser.add_argument(
        '--gui', action='store_true', help='show the Isaac Sim window')
    parser.add_argument('--recovery-tolerance-rad', type=float, default=0.01)
    parser.add_argument(
        '--maximum-contact-linkage-error-rad', type=float, default=0.02)
    parser.add_argument('--settled-velocity-rad-s', type=float, default=0.02)
    parser.add_argument(
        '--minimum-contact-impulse-ns', type=float, default=1.0e-7)
    parser.add_argument(
        '--maximum-recovered-pose-error-m', type=float, default=1.0e-4)
    parser.add_argument(
        '--maximum-recovered-orientation-error-rad',
        type=float, default=1.0e-3)
    parser.add_argument('--physics-hz', type=int, choices=[120, 240, 480], default=240)
    return parser.parse_args(argv)


async def run(args):
    """Run repeated hard-contact and recovery cycles with one step owner."""
    import omni.kit.app
    import omni.timeline
    app = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()
    contract = load_contract(args.model)
    asset = args.asset or default_asset(args.model)
    root_path = articulation_path(contract)
    joint_name = driven_joint(contract)
    report = {
        'schema_version': 1,
        'model': args.model,
        'asset': str(asset.resolve()),
        'test': 'repeated-hard-fingertip-contact-and-linkage-recovery',
        'status': 'failed',
        'cycles': [],
        'physics_hz': args.physics_hz,
        'runner_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }
    try:
        if (args.cycles < 1 or args.travel_steps < 1 or
                args.contact_steps < 1 or
                args.recovery_tolerance_rad <= 0.0 or
                args.maximum_contact_linkage_error_rad <= 0.0 or
                args.settled_velocity_rad_s <= 0.0 or
                args.minimum_contact_impulse_ns <= 0.0 or
                args.maximum_recovered_pose_error_m <= 0.0 or
                args.maximum_recovered_orientation_error_rad <= 0.0):
            raise RuntimeError('test counts and tolerances must be positive')
        import omni.usd
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.core.version import get_version
        from isaacsim.sensors.experimental.physics import Contact
        from isaacsim.sensors.experimental.physics import ContactSensor

        report['isaac_sim_version'] = get_version()[0]
        if not asset.is_file():
            raise RuntimeError(f'asset does not exist: {asset}')
        timeline.stop()
        await app.next_update_async()
        await omni.usd.get_context().open_stage_async(str(asset.resolve()))
        while is_stage_loading():
            await app.next_update_async()
        stage = omni.usd.get_context().get_stage()
        contact_variant = contract['collision_model'][
            'tip_contact_test_variant']
        variant_set = stage.GetPrimAtPath(root_path).GetVariantSet('Physics')
        if not variant_set.SetVariantSelection(contact_variant):
            raise RuntimeError(
                f'could not select tip-contact variant {contact_variant}')
        await app.next_update_async()
        while is_stage_loading():
            await app.next_update_async()
        report['physics_variant'] = contact_variant
        left_tip_path = contract['custom_fingers']['left_anchor'].replace(
            'finger_base_link', 'finger_tip_link')
        right_tip_path = contract['custom_fingers']['right_anchor'].replace(
            'finger_base_link', 'finger_tip_link')
        contact_sensor = ContactSensor(Contact.create(
            f'{left_tip_path}/qualification_contact_sensor',
            min_threshold=0.0,
            max_threshold=100000.0,
            radius=-1.0))
        contact_sensor.add_raw_contact_data_to_frame()
        tips = RigidPrim([left_tip_path, right_tip_path])
        setup_simulation(SimulationManager, dt=1.0 / args.physics_hz, device='cpu')
        articulation = Articulation(root_path)
        timeline.play()
        for _ in range(2):
            await app.next_update_async()
        timeline.pause()
        await app.next_update_async()
        names = list(articulation.dof_names)
        if joint_name not in names:
            raise RuntimeError(
                f'{joint_name} missing from runtime DOFs: {names}')
        leader = names.index(joint_name)
        lower, upper = joint_limits(contract)
        report['articulation_path'] = root_path
        report['dof_names'] = names
        mimic_gearing = {
            item['name']: float(item['gearing'])
            for item in contract['usd']['mimic_joints']
            if item['name'] in names
        }
        from run_payload_retention_test import _asset_manifest
        report['asset_files_sha256'] = _asset_manifest(asset)
        report['effective_dof_gains_si'] = [v.numpy().tolist() for v in articulation.get_dof_gains()]
        report['effective_dof_max_efforts_si'] = articulation.get_dof_max_efforts().numpy().tolist()
        time_origin = float(SimulationManager.get_simulation_time())
        step_count = 0

        def poses() -> list[tuple[list[float], list[float]]]:
            positions, orientations = tips.get_world_poses()
            return list(zip(_rows(positions), _rows(orientations)))

        def root_pose() -> tuple[list[float], list[float]]:
            positions, orientations = articulation.get_world_poses()
            return _rows(positions)[0], _rows(orientations)[0]

        async def command(target: float, steps: int, *, observe=False) -> dict:
            nonlocal step_count
            observation = {
                'maximum_linkage_error_rad': 0.0,
                'maximum_velocity_rad_s': 0.0,
                'minimum_leader_position_rad': math.inf,
                'maximum_tip_contact_impulse_ns': 0.0,
            }
            for _ in range(steps):
                articulation.set_dof_position_targets(
                    target, dof_indices=[leader])
                SimulationManager.step(update_fabric=SimulationManager.is_fabric_enabled())
                step_count += 1
                if abs(SimulationManager.get_simulation_time() - time_origin - step_count / args.physics_hz) > 1e-5:
                    raise RuntimeError('physics time differs from requested steps')
                if step_count % 8 == 0:
                    await app.next_update_async()
                if observe:
                    positions = _row(articulation.get_dof_positions())
                    velocities = _row(articulation.get_dof_velocities())
                    leader_delta = positions[leader] - open_reference[leader]
                    for name, gearing in mimic_gearing.items():
                        index = names.index(name)
                        # PhysX defines mimic gearing on the constraint
                        # equation, so follower motion has the opposite sign.
                        expected = (
                            open_reference[index] - gearing * leader_delta)
                        observation['maximum_linkage_error_rad'] = max(
                            observation['maximum_linkage_error_rad'],
                            abs(positions[index] - expected))
                    observation['maximum_velocity_rad_s'] = max(
                        observation['maximum_velocity_rad_s'],
                        max(abs(value) for value in velocities))
                    observation['minimum_leader_position_rad'] = min(
                        observation['minimum_leader_position_rad'],
                        positions[leader])
                    contact_frame = contact_sensor.get_data()
                    for contact in contact_frame.get('contacts', []):
                        bodies = {contact['body0'], contact['body1']}
                        if right_tip_path not in bodies:
                            continue
                        impulse = _flat(contact['impulse'])
                        contact_impulse = math.sqrt(sum(
                            value * value for value in impulse))
                        observation['maximum_tip_contact_impulse_ns'] = max(
                            observation['maximum_tip_contact_impulse_ns'],
                            contact_impulse)
            return observation

        await command(upper, args.travel_steps)
        open_reference = _row(articulation.get_dof_positions())
        open_tip_poses = poses()
        open_root_pose = root_pose()
        if not all(math.isfinite(value) for value in open_reference):
            raise RuntimeError('open reference contains non-finite DOF state')

        for cycle in range(1, args.cycles + 1):
            contact_observation = await command(
                lower, args.travel_steps + args.contact_steps, observe=True)
            contact_position = _row(articulation.get_dof_positions())
            contact_velocity = _row(articulation.get_dof_velocities())
            await command(upper, args.travel_steps)
            recovered = _row(articulation.get_dof_positions())
            recovered_velocity = _row(articulation.get_dof_velocities())
            recovered_tip_poses = poses()
            recovered_root_pose = root_pose()
            recovery_error = max(
                abs(actual - expected)
                for actual, expected in zip(recovered, open_reference))
            recovered_dof_errors = {
                name: abs(recovered[index] - open_reference[index])
                for index, name in enumerate(names)
            }
            maximum_settled_velocity = max(
                abs(value) for value in recovered_velocity)
            recovered_pose_errors = [
                _pose_error(reference, actual)
                for reference, actual in zip(
                    open_tip_poses, recovered_tip_poses)
            ]
            maximum_pose_error = max(
                error[0] for error in recovered_pose_errors)
            maximum_orientation_error = max(
                error[1] for error in recovered_pose_errors)
            root_position_error, root_orientation_error = _pose_error(
                open_root_pose, recovered_root_pose)
            contact_observed = (
                contact_observation['maximum_tip_contact_impulse_ns'] >=
                args.minimum_contact_impulse_ns)
            finite = all(math.isfinite(value) for value in (
                contact_position + contact_velocity + recovered +
                recovered_velocity))
            passed = (
                finite and
                contact_observed and
                contact_observation['maximum_linkage_error_rad'] <=
                args.maximum_contact_linkage_error_rad and
                recovery_error <= args.recovery_tolerance_rad and
                maximum_settled_velocity <= args.settled_velocity_rad_s and
                maximum_pose_error <=
                args.maximum_recovered_pose_error_m and
                maximum_orientation_error <=
                args.maximum_recovered_orientation_error_rad and
                root_position_error <=
                args.maximum_recovered_pose_error_m and
                root_orientation_error <=
                args.maximum_recovered_orientation_error_rad)
            report['cycles'].append({
                'cycle': cycle,
                'contact_leader_position_rad': contact_position[leader],
                'contact_maximum_velocity_rad_s': max(
                    abs(value) for value in contact_velocity),
                **contact_observation,
                'tip_to_tip_contact_observed': contact_observed,
                'maximum_recovery_error_rad': recovery_error,
                'recovered_dof_errors_rad': recovered_dof_errors,
                'maximum_settled_velocity_rad_s': maximum_settled_velocity,
                'maximum_recovered_tip_position_error_m':
                    maximum_pose_error,
                'maximum_recovered_tip_orientation_error_rad':
                    maximum_orientation_error,
                'recovered_root_position_error_m': root_position_error,
                'recovered_root_orientation_error_rad':
                    root_orientation_error,
                'passed': passed,
            })
        report['acceptance'] = {
            'maximum_recovery_error_rad': args.recovery_tolerance_rad,
            'maximum_contact_linkage_error_rad':
                args.maximum_contact_linkage_error_rad,
            'maximum_settled_velocity_rad_s': args.settled_velocity_rad_s,
            'minimum_tip_contact_impulse_ns':
                args.minimum_contact_impulse_ns,
            'maximum_recovered_tip_position_error_m':
                args.maximum_recovered_pose_error_m,
            'maximum_recovered_tip_orientation_error_rad':
                args.maximum_recovered_orientation_error_rad,
            'all_dof_states_finite': True,
        }
        report['status'] = (
            'passed' if all(item['passed'] for item in report['cycles'])
            else 'failed')
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
        report['traceback'] = traceback.format_exc().splitlines()
    finally:
        timeline.stop()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')
    return report


def main() -> int:
    args = arguments()
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': not args.gui, 'width': 1280, 'height': 720})
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

#!/usr/bin/env python3
"""Expose an OnRobot gripper's physical joint through the Isaac ROS bridge."""

import argparse
import json
import math
from pathlib import Path
import sys
import time

from isaac_model_contract import articulation_path
from isaac_model_contract import default_asset
from isaac_model_contract import driven_joint
from isaac_model_contract import load_contract
from isaac_model_contract import SUPPORTED_MODELS

from isaac_runtime_compat import enable_extension
from isaac_runtime_compat import is_stage_loading
from isaac_runtime_compat import play
from isaac_runtime_compat import setup_simulation
from isaac_runtime_compat import stop

from isaacsim import SimulationApp


def _topic(namespace: str, name: str) -> str:
    namespace = namespace.strip('/')
    return f'{namespace}/{name}' if namespace else name


def main() -> int:
    """Load the asset, create its ROS Action Graph, and run simulation."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=SUPPORTED_MODELS, required=True)
    parser.add_argument('--asset', type=Path)
    parser.add_argument('--namespace', default='')
    parser.add_argument('--physics-dt', type=float, default=1.0 / 120.0)
    parser.add_argument('--command-timeout', type=float, default=0.1)
    parser.add_argument('--test-steps', type=int, default=0)
    parser.add_argument('--ready-output', type=Path)
    parser.add_argument(
        '--gui', action='store_true',
        help='Open the Isaac Sim editor window while the bridge runs')
    args, _ = parser.parse_known_args()
    contract = load_contract(args.model)
    if args.asset is None:
        args.asset = default_asset(args.model)
    root_path = articulation_path(contract)
    joint_name = driven_joint(contract)

    status = {
        'model': args.model,
        'asset': str(args.asset.resolve()),
        'status': 'failed',
    }
    app = None
    command_node = None
    command_subscription = None
    rclpy_module = None
    exit_code = 0
    try:
        launch_config = {
            'headless': not args.gui,
            'width': 1280,
            'height': 720,
        }
        if args.gui:
            launch_config['renderer'] = 'RaytracedLighting'
        app = SimulationApp(launch_config)

        import omni.graph.core as og
        import omni.usd
        import rclpy
        import usdrt.Sdf
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.core.version import get_version
        from sensor_msgs.msg import JointState

        rclpy_module = rclpy

        status['isaac_sim_version'] = get_version()[0]
        if not args.asset.is_file():
            raise RuntimeError(f'asset does not exist: {args.asset}')
        if (args.physics_dt <= 0.0 or args.command_timeout <= 0.0 or
                args.test_steps < 0):
            raise RuntimeError(
                'physics dt and command timeout must be positive and test '
                'steps non-negative')

        enable_extension('isaacsim.ros2.bridge')
        app.update()
        if not omni.usd.get_context().open_stage(str(args.asset.resolve())):
            raise RuntimeError(f'Isaac Sim could not open {args.asset}')
        app.update()
        app.update()
        while is_stage_loading():
            app.update()

        state_topic = _topic(args.namespace, 'isaac_joint_states')
        command_topic = _topic(args.namespace, 'isaac_joint_commands')
        # ROS simulation time is a graph-wide clock, not an instance-local
        # gripper topic. Namespacing it would leave use_sim_time nodes waiting
        # on /clock while the bridge publishes elsewhere.
        clock_topic = 'clock'
        og.Controller.edit(
            {'graph_path': '/OnRobotROSBridge',
             'evaluator_name': 'execution'},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ('OnPlaybackTick', 'omni.graph.action.OnPlaybackTick'),
                    (
                        'ReadSimTime',
                        'isaacsim.core.nodes.IsaacReadSimulationTime',
                    ),
                    (
                        'ReadJointState',
                        'isaacsim.sensors.physics.IsaacReadJointState',
                    ),
                    ('Context', 'isaacsim.ros2.bridge.ROS2Context'),
                    (
                        'PublishJointState',
                        'isaacsim.ros2.bridge.ROS2PublishJointState',
                    ),
                    (
                        'PublishClock',
                        'isaacsim.ros2.bridge.ROS2PublishClock',
                    ),
                ],
                og.Controller.Keys.CONNECT: [
                    (
                        'OnPlaybackTick.outputs:tick',
                        'ReadJointState.inputs:execIn',
                    ),
                    (
                        'ReadJointState.outputs:execOut',
                        'PublishJointState.inputs:execIn',
                    ),
                    (
                        'ReadJointState.outputs:jointNames',
                        'PublishJointState.inputs:jointNames',
                    ),
                    (
                        'ReadJointState.outputs:jointPositions',
                        'PublishJointState.inputs:jointPositions',
                    ),
                    (
                        'ReadJointState.outputs:jointVelocities',
                        'PublishJointState.inputs:jointVelocities',
                    ),
                    (
                        'ReadJointState.outputs:jointEfforts',
                        'PublishJointState.inputs:jointEfforts',
                    ),
                    (
                        'ReadJointState.outputs:jointDofTypes',
                        'PublishJointState.inputs:jointDofTypes',
                    ),
                    (
                        'ReadJointState.outputs:stageMetersPerUnit',
                        'PublishJointState.inputs:stageMetersPerUnit',
                    ),
                    (
                        'ReadJointState.outputs:sensorTime',
                        'PublishJointState.inputs:sensorTime',
                    ),
                    (
                        'OnPlaybackTick.outputs:tick',
                        'PublishClock.inputs:execIn',
                    ),
                    (
                        'Context.outputs:context',
                        'PublishJointState.inputs:context',
                    ),
                    ('Context.outputs:context', 'PublishClock.inputs:context'),
                    (
                        'ReadSimTime.outputs:simulationTime',
                        'PublishClock.inputs:timeStamp',
                    ),
                ],
                og.Controller.Keys.SET_VALUES: [
                    (
                        'ReadJointState.inputs:prim',
                        [usdrt.Sdf.Path(root_path)],
                    ),
                    ('PublishJointState.inputs:topicName', state_topic),
                    ('PublishClock.inputs:topicName', clock_topic),
                ],
            })

        setup_simulation(
            SimulationManager, dt=args.physics_dt, device='cpu')
        articulation = Articulation(root_path)

        # Initialize PhysX before reading DOF metadata. Before playback,
        # Isaac's USD fallback cannot infer the type of an undriven mimic DOF.
        play()
        app.update()

        if joint_name not in articulation.dof_names:
            raise RuntimeError(
                f'{joint_name} missing from runtime DOFs: '
                f'{articulation.dof_names}')
        driven_index = articulation.dof_names.index(joint_name)

        # Isaac's standard JointState subscriber retains its last velocity
        # target indefinitely. Handle this single driven DOF explicitly so a
        # stopped ROS process cannot leave the simulated gripper moving.
        rclpy.init()
        command_node = rclpy.create_node(
            f'onrobot_{args.model}_isaac_commands')
        pending_command = None
        last_velocity_command = None
        active_control_mode = None

        def receive_command(message):
            nonlocal pending_command, last_velocity_command
            try:
                index = message.name.index(joint_name)
            except ValueError:
                return
            position = (
                message.position[index]
                if index < len(message.position) else math.nan)
            velocity = (
                message.velocity[index]
                if index < len(message.velocity) else math.nan)
            if math.isfinite(position):
                pending_command = ('position', position)
                last_velocity_command = None
            elif math.isfinite(velocity):
                pending_command = ('velocity', velocity)
                last_velocity_command = time.monotonic()

        command_subscription = command_node.create_subscription(
            JointState, command_topic, receive_command, 1)

        status.update({
            'status': 'ready',
            'command_topic': command_topic,
            'state_topic': state_topic,
            'clock_topic': clock_topic,
            'physical_joint': joint_name,
            'articulation_path': root_path,
            'velocity_watchdog_s': args.command_timeout,
            'gui': args.gui,
        })
        if args.ready_output:
            args.ready_output.parent.mkdir(parents=True, exist_ok=True)
            args.ready_output.write_text(
                json.dumps(status, indent=2, sort_keys=True) + '\n',
                encoding='utf-8')
        print(json.dumps(status, indent=2, sort_keys=True), flush=True)

        steps = 0
        while app.is_running():
            rclpy.spin_once(command_node, timeout_sec=0.0)
            if pending_command is not None:
                mode, target = pending_command
                pending_command = None
                if mode != active_control_mode:
                    articulation.switch_dof_control_mode(
                        mode, dof_indices=[driven_index])
                    active_control_mode = mode
                if mode == 'position':
                    articulation.set_dof_position_targets(
                        target, dof_indices=[driven_index])
                else:
                    articulation.set_dof_velocity_targets(
                        target, dof_indices=[driven_index])
            if (active_control_mode == 'velocity' and
                    last_velocity_command is not None and
                    time.monotonic() - last_velocity_command >
                    args.command_timeout):
                positions = articulation.get_dof_positions(
                    dof_indices=[driven_index])
                hold_position = float(positions.numpy()[0, 0])
                articulation.switch_dof_control_mode(
                    'position', dof_indices=[driven_index])
                articulation.set_dof_position_targets(
                    hold_position, dof_indices=[driven_index])
                active_control_mode = 'position'
                last_velocity_command = None
            app.update()
            steps += 1
            if args.test_steps and steps >= args.test_steps:
                break
    except Exception as error:
        status['status'] = 'failed'
        status['error'] = f'{type(error).__name__}: {error}'
        print(
            json.dumps(status, indent=2, sort_keys=True),
            file=sys.stderr, flush=True)
        exit_code = 1
    finally:
        if args.ready_output:
            args.ready_output.parent.mkdir(parents=True, exist_ok=True)
            args.ready_output.write_text(
                json.dumps(status, indent=2, sort_keys=True) + '\n',
                encoding='utf-8')
        if command_node is not None:
            try:
                if command_subscription is not None:
                    command_node.destroy_subscription(command_subscription)
                command_node.destroy_node()
            except Exception:
                pass
        if rclpy_module is not None:
            try:
                if rclpy_module.ok():
                    rclpy_module.shutdown()
            except Exception:
                pass
        if app is not None:
            try:
                stop()
            except Exception:
                pass
            app.close(exit_code=exit_code)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())

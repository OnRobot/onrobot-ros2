# Verify RG2 with ROS 2 and Isaac Sim

This guide verifies RG2 using the standard ROS action, realtime
velocity control, RViz visualization, and the supplied Isaac Sim asset. The
gripper IP address and Modbus TCP port are the only device-specific connection
inputs.

## Before starting

- Use Ubuntu 24.04 with ROS 2 Jazzy and the repository dependencies installed.
- Install the matching Tool API runtime and development packages.
- Use RG2 realtime firmware compatible with integration guide v1.1. The
  included hardware test result used firmware 1.0.10.
- Secure the gripper, clear its complete travel, and keep an independent means
  of removing power available.
- Run only one real ROS bringup process; it owns the Modbus connection.

Build and source the workspace as described in the repository's top-level
`GETTING_STARTED.md`.

## Conventional RViz control

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=rg2 host:=GRIPPER_IP port:=502 \
  control:=conventional start_rviz:=true
```

RViz opens with the conventional panel. Wait for live state and limits, then
use **Open**, **Close**, or **Send target**. The physical gripper and RViz model
must move in the same direction and settle at the same aperture.

The same control path is available to applications and MoveIt through
`/gripper_controller/gripper_cmd`, using
`control_msgs/action/ParallelGripperCommand`.

## Realtime RViz control

Stop conventional bringup before changing modes, then run:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=rg2 host:=GRIPPER_IP port:=502 \
  control:=realtime start_rviz:=true
```

Set a conservative angular velocity and hold the joystick toward **OPEN** or
**CLOSE**. Releasing it requests Stop. RG realtime velocity is the
finger-independent mechanism angular velocity in rad/s; displayed aperture is
the compensated finger-dependent task coordinate in metres.

## Validate the supplied Isaac asset

After sourcing the built workspace:

```bash
ros2 run onrobot_gripper_isaac validate_asset_contract.py --model rg2
```

The result must report `offline_contract` as `passed` and runtime validation as
`kinematic-automated-passed`. The runtime fields identify the exact asset and
previous test results represented by the installed contract. Run the commands
below when you need results for the currently installed files and hardware.

## Mirror real RG2 motion in Isaac Sim

Keep conventional real-hardware bringup running in one terminal. In a second
terminal, source the same ROS workspace and define the local Isaac installation
and asset paths:

```bash
export ISAAC_SIM_DIR=/path/to/isaacsim
export ISAAC_ASSET=/path/to/install/onrobot_gripper_isaac/share/onrobot_gripper_isaac/assets/rg2/onrobot_rg2.usda

"$ISAAC_SIM_DIR/python.sh" \
  /path/to/install/onrobot_gripper_isaac/lib/onrobot_gripper_isaac/run_hardware_in_loop.py \
  --model rg2 --asset "$ISAAC_ASSET" --gui --duration 15 \
  --output /path/to/results/rg2_read_only_shadow.json
```

The report must pass and record `shadow_mimic_dof_count` as `9`. All finger
links must remain visually connected. This mode sends no hardware command.

With the swept volume clear, the bounded conventional HIL test is:

```bash
"$ISAAC_SIM_DIR/python.sh" \
  /path/to/install/onrobot_gripper_isaac/lib/onrobot_gripper_isaac/run_hardware_in_loop.py \
  --model rg2 --asset "$ISAAC_ASSET" --gui \
  --enable-hardware-motion --confirm-model rg2 \
  --hardware-control conventional --travel 0.010 --effort 10 \
  --output /path/to/results/rg2_conventional_hil.json
```

It commands one bounded aperture excursion, verifies the aperture-to-angle
mapping and all nine mimic joints after each PhysX step, and restores the
initial aperture.

## Check realtime position control

Stop conventional bringup and start realtime bringup without RViz:

```bash
ros2 launch onrobot_gripper_bringup gripper.launch.py \
  model:=rg2 backend:=real host:=GRIPPER_IP port:=502 \
  start_realtime_controller:=true \
  realtime_update_rate_hz:=50
```

With the swept volume clear, run the bounded check headless in a second
terminal:

```bash
"$ISAAC_SIM_DIR/python.sh" \
  /path/to/install/onrobot_gripper_isaac/lib/onrobot_gripper_isaac/run_hardware_in_loop.py \
  --model rg2 --asset "$ISAAC_ASSET" \
  --enable-hardware-motion --confirm-model rg2 \
  --hardware-control realtime-position --travel 0.010 --effort 10 \
  --output /path/to/results/rg2_realtime_position_hil.json
```

For a repeatable timing measurement, do not add `--gui`. A passing result
requires the motion checks to pass without adverse hardware-health counter
changes. A run that completes motion but reports missed deadlines identifies a
timing problem. Use 50 Hz for this check. Select a higher rate only after the
target deployment demonstrates stable zero-miss operation at that rate.

## Capability and simulation scope

- Conventional and realtime control paths are implemented. Use the HIL
  commands above to check the installed asset with the connected gripper.
- Use a 50 Hz hardware exchange rate for the realtime-position HIL rerun.
  Test higher rates on the target host, network, and gripper before relying on
  them.
- The supplied USD drive torque, stiffness, and damping are simulation inputs,
  not a calibrated hardware response model.
- Simulated gripping force is unavailable as a public measurement. Contact
  tests check linkage and contact behavior, not force accuracy or payload
  rating.
- The standard finger attachment anchors are stable, but user-authored finger
  geometry and physics remain the user's responsibility and are outside the
  supplied test scenarios.
- Mounted-robot behavior depends on the robot model, mounting, controller, and
  scene and is not represented by the standalone asset tests.
- This workflow does not certify functional safety or application-specific
  payload retention. Complete a system-level risk assessment, provide
  independent stopping provisions, and validate the setup for your application.

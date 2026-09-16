# Isaac Sim integration

`onrobot_gripper_isaac` integrates Asset Structure 3.0 USD assets and repeatable
Isaac Sim integration checks for 2FG7, 2FG14, RG2, and RG6. ROS applications
continue to use the normal OnRobot `ros2_control` interfaces; the Isaac
adapter converts the task aperture to each asset's physical articulation
coordinate.

The same integration surface is provided for 2FG7, 2FG14, RG2, and RG6.

## Asset dependency

Canonical USD files live in the separate, non-ROS `onrobot-isaac-sim`
repository. Before building from source, provide it at either
`onrobot-ros2/onrobot-isaac-sim` (the submodule location) or as a sibling of
`onrobot-ros2`. Supplied source bundles already include the required files.
The build checks `config/asset_repository.json` against the asset manifest
and verifies every asset checksum. A missing or mismatched dependency stops
the build with an explicit error.

For a different checkout location, set `ONROBOT_ISAAC_ASSET_REPOSITORY` to
that repository directory before building or running source scripts. CMake
also accepts `-DONROBOT_ISAAC_ASSET_REPOSITORY=/path/to/onrobot-isaac-sim`.
Installed commands use their self-contained `share/onrobot_gripper_isaac/assets`
directory and do not depend on the source checkout.

## Start here

Build and source the ROS workspace, then validate the selected asset without
starting Isaac Sim:

```bash
ros2 run onrobot_gripper_isaac validate_asset_contract.py --model 2fg7
```

The installed command performs a public, structural check and reports runtime
status as `not-published`. Runtime reports are created only in the output
directory you choose.

For the shortest path to a moving model, follow
[Run a first gripper simulation](doc/FIRST_ISAAC_SIMULATION.md). For repeatable
checks and machine-readable results, continue with [Verify OnRobot grippers in
Isaac Sim](doc/ISAAC_SIM_VERIFICATION.md). The supported entry checks cover:

- PhysX articulation and kinematic checks;
- conventional and realtime ROS control;
- repeated 2FG and RG contact checks;
- read-only or explicitly bounded hardware-in-the-loop verification.

For an optional Isaac-only data experiment, see
[Simulated grip-setting experiment](doc/SYNTHETIC_GRIP_LEARNING.md). It records
complete PhysX outcomes, saves deterministic training checkpoints, and renders
new baseline-versus-selected trials in a held-out simulated environment. It does
not replace asset validation or claim hardware calibration.

For a real-device shadow or bounded HIL check, use the model-specific guides
for [2FG7](doc/RVIZ_ISAAC_2FG7_DEMO.md) or
[RG2](doc/RG2_HIL_GUIDE.md). These are opt-in checks, not the default first
run.

For a robot-free table pickup and pendulum demonstration, see
[Physical-motion showcases](doc/PHYSICAL_MOTION_SHOWCASE.md#pick-a-workpiece-from-a-table).

## Assets and coordinates

| Model | Driven USD joint | ROS task coordinate | Realtime velocity coordinate |
|---|---|---|---|
| 2FG7 | `finger_stroke` | external aperture in metres | aperture velocity in m/s |
| 2FG14 | `finger_stroke` | external aperture in metres | aperture velocity in m/s |
| RG2 | `finger_joint` | compensated external aperture in metres | mechanism angular velocity in rad/s |
| RG6 | `finger_joint` | compensated external aperture in metres | mechanism angular velocity in rad/s |

`grip_stroke` is a ROS task coordinate, not another USD degree of freedom.
The 2FG assets contain the physical leader and mimic follower. RG assets
contain the angular leader and their complete mimic linkage. Stable finger
attachment prims are part of each asset contract; custom geometry, mass,
collision, contact, limits, calibration, and validation remain the custom
finger author's responsibility.

## Runtime tools

| Executable | Purpose |
|---|---|
| `run_articulation_test.py` | Exercise and verify physical articulation endpoints |
| `run_kinematic_qualification.py` | Verify the ROS/Isaac conventional and realtime paths |
| `run_2fg_dynamic_contact_test.py` | Repeat the declared 2FG external-pinch fixture |
| `run_rg2_tip_contact_test.py` | Exercise the RG linkage and hard-contact recovery |
| `run_drive_step_response.py` | Record an observational unloaded response trace |
| `run_workpiece_showcase.py` | Pick up, swing, replace, and release a rigid cube without ROS |
| `run_synthetic_grip_learning.py` | Collect simulated outcomes and train a grip-setting selector |
| `run_ros_bridge.py` | Connect the standard ROS adapter to one Isaac articulation |
| `run_hardware_in_loop.py` | Mirror measured hardware or run an explicitly bounded check |
| `validate_payload_retention_results.py` | Reject incomplete, stale, reduced, or overridden result matrices |
| `validate_physical_motion_results.py` | Reject incomplete, stale, replayed, or single-axis showcase result sets |

## What the tests establish

Run the documented checks to produce results for the exact installed assets and
runtime. A result from another asset, fixture, runner, or Isaac version does not
validate the files currently installed on your machine. The model contracts under
`config/` define the inputs and expectations used by the local checks.

The installed asset check validates structure, file references, and the ROS
description mapping. Runtime reports are created in the output directory you
choose; they are not part of the installed package.

The included results do not establish:

- calibrated simulated gripping force;
- hardware-equivalent drive gains or timing;
- arbitrary workpieces, materials, or custom fingers;
- internal gripping; or
- a gripper mounted in an arbitrary robot articulation.

Simulated public force stays unavailable until a hardware/PhysX calibration
defines a supported mapping. Test fixtures may observe contact internally
without exposing that value as a calibrated ROS measurement.

## Hardware ownership

In hardware-in-the-loop use, the normal real ROS backend is the only owner of
the gripper connection. Isaac subscribes to measured ROS state and must not
open another Modbus session. Read-only shadowing sends no hardware command.
Hardware motion checks require explicit model confirmation and bounded-motion
arguments; see the verification guide before enabling them.

All drive, material, solver, object, runner, and runtime assumptions that
contribute to a verdict are recorded in its JSON report. Preserve the report
with the exact asset revision it exercised.

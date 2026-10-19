# Isaac Sim integration

`onrobot_gripper_isaac` integrates Asset Structure 3.0 USD assets and repeatable
Isaac Sim 6.1 integration checks for 2FG7, 2FG14, RG2, and RG6. ROS applications
continue to use the normal OnRobot `ros2_control` interfaces; the Isaac
adapter converts the task aperture to each asset's physical articulation
coordinate.

## Asset dependency

The USD files are maintained in the separate, non-ROS `onrobot-isaac-sim`
repository (published as `onrobot-isaacsim` on GitHub). Before building from
source, provide it as a sibling of `onrobot-ros2` or as a nested checkout.
Both `onrobot-isaac-sim` and `onrobot-isaacsim` directory names are accepted.
Supplied source bundles already include the required files.
The build requires SimReady package version **2026.10.07_01 or newer** and
**interface revision 1** for each model, as declared in
`config/asset_repository.json`. It checks the package identities and all file
checksums. Older packages, unsupported interfaces and incomplete asset trees
stop the build with an explicit error.

The package version comes from the existing SimReady `package_id`. Its date
and numeric revision identify the asset release. The separate interface
revision covers entry points, articulation/joint names, SI coordinates and
kinematics used by ROS. Compatible material or drive-tuning releases can be
used without a new ROS release. Changes to joint names or coordinate definitions need a corresponding ROS
update. File hashes identify the exact installed assets
and test inputs; they do not restrict every installation to one release.
Before starting control, the bridge checks the composed joints, rigid bodies,
joint frames and mimic linkage, including overrides in stronger USD layers.

For a different checkout location, set `ONROBOT_ISAAC_ASSET_REPOSITORY` to
that repository directory before building or running source scripts. CMake
also accepts `-DONROBOT_ISAAC_ASSET_REPOSITORY=/path/to/onrobot-isaac-sim`.
Regular installations use their self-contained
`share/onrobot_gripper_isaac/assets` directory. Development builds using
`--symlink-install` require the source checkouts to remain available.

Entry points are at `assets/onrobot_<model>/simready_usd/onrobot_<model>.usda`.
After updating the asset checkout, rebuild `onrobot_gripper_isaac` and source
the installation again. Existing installations retain their installed copy.
Run the asset and application checks again after changing simulation tuning;
previous grasp and timing results apply only to the files tested.

## Start here

Build and source the ROS workspace, then validate the selected asset without
starting Isaac Sim:

```bash
ros2 run onrobot_gripper_isaac validate_asset_contract.py --model 2fg7
```

The command checks asset structure without running the simulation. It reports
runtime status as `not-published` because runtime results are not installed
with the package. Runtime reports are created only in the output
directory you choose.

For the shortest path to a moving model, follow
[Run a first gripper simulation](doc/FIRST_ISAAC_SIMULATION.md). For repeatable
checks and machine-readable results, continue with [Verify OnRobot grippers in
Isaac Sim](doc/ISAAC_SIM_VERIFICATION.md). The checks cover:

- PhysX articulation and kinematic checks;
- conventional and realtime ROS control;
- repeated 2FG and RG contact checks;
- read-only or explicitly bounded hardware-in-the-loop verification.

For an optional Isaac-only data experiment, see
[Simulated grip-setting experiment](doc/SYNTHETIC_GRIP_LEARNING.md). It records
PhysX outcomes and saves the trained model at several stages. It then compares
a baseline setting with the selected setting in a new simulation run. The
comparison uses an environment that was excluded from training. It does
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
contain the angular leader and their complete mimic linkage. Each asset provides stable USD prim paths for finger attachments; custom geometry, mass,
collision, contact, limits, calibration, and validation remain the custom
finger author's responsibility.

## Runtime tools

| Executable | Purpose |
|---|---|
| `run_articulation_test.py` | Exercise and verify physical articulation endpoints |
| `run_kinematic_qualification.py` | Verify the ROS/Isaac conventional and realtime paths |
| `run_2fg_dynamic_contact_test.py` | Repeat the declared 2FG external-pinch fixture |
| `run_rg2_tip_contact_test.py` | Exercise the RG linkage and hard-contact recovery |
| `run_drive_step_response.py` | Record joint motion after a command, with no workpiece |
| `run_workpiece_showcase.py` | Pick up, swing, replace, and release a rigid cube without ROS |
| `run_synthetic_grip_learning.py` | Collect simulated outcomes and train a grip-setting selector |
| `run_ros_bridge.py` | Connect the standard ROS adapter to one Isaac articulation |
| `run_hardware_in_loop.py` | Mirror measured hardware or run an explicitly bounded check |
| `validate_payload_retention_results.py` | Check that all required payload tests passed with the expected assets and settings |
| `validate_physical_motion_results.py` | Check that live showcases passed with the expected assets and motion |

## What the tests establish

Run the documented checks to produce results for the exact installed assets and
runtime. A result from another asset, fixture, runner, or Isaac version does not
validate the files currently installed on your machine. The model configuration
files under `config/` define the test inputs and expected results.

The installed asset check validates structure, file references, and the ROS
description mapping. Runtime reports are created in the output directory you
choose; they are not part of the installed package.

These tests do not establish:

- calibrated simulated gripping force;
- hardware-equivalent drive gains or timing;
- arbitrary workpieces, materials, or custom fingers;
- internal gripping; or
- a gripper mounted in an arbitrary robot articulation.

ROS force measurements remain unavailable in simulation until a calibration
relates PhysX contact values to hardware force measurements. Test fixtures may observe contact internally
without exposing that value as a calibrated ROS measurement.

## Hardware ownership

During hardware-in-the-loop tests, only the real ROS backend connects to the
gripper. Isaac subscribes to measured ROS state and must not
open another Modbus session. Read-only shadowing sends no hardware command.
Hardware motion checks require explicit model confirmation and bounded-motion
arguments; see the verification guide before enabling them.
Conventional motion checks use 20 N for 2FG7, 40 N for 2FG14 and 10 N for
RG2/RG6 by default. `--effort` selects another supported value up to the
runner's 40 N limit.

All drive, material, solver, object, runner, and runtime assumptions that
affect the result are recorded in its JSON report. Preserve the report
with the exact asset revision it exercised.

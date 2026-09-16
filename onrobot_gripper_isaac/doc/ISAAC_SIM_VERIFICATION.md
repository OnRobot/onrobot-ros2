# Verify OnRobot grippers in Isaac Sim

This procedure verifies the supplied 2FG7, 2FG14, RG2, or RG6 asset on one
Ubuntu machine. It checks the USD articulation first, then connects the normal
OnRobot ROS 2 controllers and RViz panels to the simulation. The simulation-only
steps do not require a physical gripper; the later hardware-in-the-loop section
is explicitly opt-in.

The articulation and kinematic procedures establish whether the selected asset
loads and follows the ROS bridge. The full-rated payload-retention and motion
procedures are diagnostic physics scenarios: their reports must be checked for
the exact asset and runtime, and a failed report must not be presented as a
product capability. These tests do not establish calibrated contact force,
hardware-equivalent drive response, or behavior with arbitrary objects and
custom fingers.

The examples use 2FG7. Substitute `2fg14`, `rg2`, or `rg6` consistently in the
model, asset, result-directory, and output-file names for another asset. The
2FG repeated-reference-object test has a model-specific configuration; use the
separate RG contact test where directed below. Never mix files from different
model directories.

## 1. Locate Isaac Sim and the ROS workspace

Open a terminal on the machine where Isaac Sim was built. Set these two paths
to the actual locations on that machine:

```bash
export ONROBOT_WS=/absolute/path/to/ws
export ISAAC_SIM_DIR=/absolute/path/to/IsaacSim/_build/linux-x86_64/release
export ISAAC_RESULTS="$ONROBOT_WS/isaac-results/2fg7"
mkdir -p "$ISAAC_RESULTS"
test -x "$ISAAC_SIM_DIR/python.sh"
test -d "$ONROBOT_WS/src/onrobot-ros2"
```

All three commands after the `export` lines must finish without an error. A
source build places `python.sh` in the `_build/linux-x86_64/release` directory,
not at the Isaac Sim repository root.

## 2. Build and run the offline asset check

Install the supplied Tool API runtime and development packages as described in
the repository getting-started guide. The ROS repository does not build or
include the Tool API.

```bash
cd "$ONROBOT_WS"
source /opt/ros/jazzy/setup.bash
colcon build \
  --base-paths src/onrobot-ros2 \
  --symlink-install --parallel-workers 2 \
  --packages-up-to onrobot_gripper_isaac onrobot_gripper_bringup
source "$ONROBOT_WS/install/setup.bash"
ros2 run onrobot_gripper_isaac validate_asset_contract.py --model 2fg7 \
  2>&1 | tee "$ISAAC_RESULTS/offline_asset_check.log"
```

Continue only if the validator reports `passed`. This is a structure and
checksum check; it does not start PhysX.

## 3. Run the PhysX articulation test

The following paths use colcon's normal isolated install layout:

```bash
export ISAAC_PACKAGE_PREFIX="$ONROBOT_WS/install/onrobot_gripper_isaac"
export ISAAC_ASSET="$ISAAC_PACKAGE_PREFIX/share/onrobot_gripper_isaac/assets/2fg7/onrobot_2fg7.usda"
export ISAAC_ARTICULATION_TEST="$ISAAC_PACKAGE_PREFIX/lib/onrobot_gripper_isaac/run_articulation_test.py"
test -f "$ISAAC_ASSET"
test -x "$ISAAC_ARTICULATION_TEST"

set -o pipefail
"$ISAAC_SIM_DIR/python.sh" "$ISAAC_ARTICULATION_TEST" \
  --model 2fg7 \
  --asset "$ISAAC_ASSET" \
  --output "$ISAAC_RESULTS/2fg7_articulation.json" \
  2>&1 | tee "$ISAAC_RESULTS/2fg7_articulation.log"
```

The first Isaac Sim start can take several minutes while shaders and extensions
are cached. A pass has all of the following:

- process exit code `0`;
- a non-empty `isaac_sim_version` identifying the runtime under test;
- `finger_stroke` for 2FG or `finger_joint` for RG in `dof_names`;
- closed, midpoint, and open tests all marked `passed`; and
- the final report `status` equal to `passed`.

Keep the JSON report even if the test fails. It contains the target, measured
position, mimic error, and failure message needed for diagnosis.

## 4. Start the visible Isaac ROS bridge

Open a new terminal and repeat the path variables from step 1 and the three
`ISAAC_PACKAGE_PREFIX` variables from step 3. Then run:

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISTRO=jazzy

set -o pipefail
"$ISAAC_SIM_DIR/python.sh" \
  "$ISAAC_PACKAGE_PREFIX/lib/onrobot_gripper_isaac/run_ros_bridge.py" \
  --model 2fg7 \
  --asset "$ISAAC_ASSET" \
  --gui \
  --ready-output "$ISAAC_RESULTS/bridge_ready.json" \
  2>&1 | tee "$ISAAC_RESULTS/bridge.log"
```

Leave this terminal running. Isaac Sim should open a window containing the
2FG7, and the terminal should print a JSON object with `status` set to `ready`,
`gui` set to `true`, and these topics:

```text
isaac_joint_commands
isaac_joint_states
clock
```

The script starts the simulation timeline; do not press Play or Stop in the
Isaac window during the test.

## 5. Connect conventional ROS control

Open another terminal on the same machine. Use the same ROS environment and
RMW implementation as the bridge:

```bash
export ONROBOT_WS=/absolute/path/to/ws
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source "$ONROBOT_WS/install/setup.bash"

ros2 topic list | grep -E '^/(clock|isaac_joint_states)$'
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 backend:=isaac start_rviz:=true
```

Both `/clock` and `/isaac_joint_states` must appear before bringup starts. In
RViz, wait until the conventional gripper panel reports live state, then:

1. Click **Close** and wait for motion to finish.
2. Click **Open** and wait for motion to finish.
3. Send one intermediate target.

The fingers must move in the same direction in Isaac Sim and RViz. At the open
endpoint, `grip_stroke` should be close to `0.073 m` and the physical
`finger_stroke` should be close to `0.019 m`. At the closed endpoint, both
should be close to zero. Check the published values from another sourced ROS
terminal with:

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /gripper_state_broadcaster/state --once
ros2 control list_controllers
```

`force_valid: false` is expected. Contact force is intentionally unavailable
for this asset and should not be interpreted as a zero-force measurement.

If the RViz panel is unavailable, the standard ROS action provides a useful
independent check:

```bash
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  '{command: {name: [grip_stroke], position: [0.073]}}'
```

Stop this bringup with `Ctrl+C` before starting the realtime test. Keep the
Isaac bridge running.

## 6. Connect realtime ROS control

Restart bringup in the ROS terminal:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 backend:=isaac start_rviz:=true control:=realtime
```

The realtime RViz panel provides Open, Close, and target-aperture controls. Use
one of them to check the position path, or publish the equivalent semantic
command independently:

```bash
ros2 topic pub --rate 20 --times 40 /realtime_controller/command \
  onrobot_gripper_msgs/msg/RealtimeCommand \
  '{header: auto, mode: 0, task_position: 0.0365, task_velocity: 0.01, force: 0.0}'
```

Confirm that both visualizations reach the same midpoint. Use the panel's
velocity joystick for the remaining checks:

1. Select a conservative velocity, hold the joystick toward **OPEN**, and
   confirm the gripper opens without reversing at the endpoint.
2. Release the joystick and confirm motion stops.
3. Repeat toward **CLOSE**.
4. While moving slowly, stop ROS bringup with `Ctrl+C`. The simulated gripper
   must stop rather than retaining the last velocity command. The bridge's
   monotonic stale-command watchdog changes velocity control to a measured
   position hold after `0.1 s` without a refreshed command.

Force modes are not exposed by this backend because the simulated contact
observations are not calibrated as a gripper-force measurement.

## 7. Run the automated kinematic checks

Stop the manually launched ROS bringup and Isaac bridge first. The verification
runner owns both child processes, uses the isolated `onrobot_kinematic`
namespace, and shuts down only the processes it started:

```bash
source /opt/ros/jazzy/setup.bash
source "$ONROBOT_WS/install/setup.bash"

ros2 run onrobot_gripper_isaac run_kinematic_qualification.py \
  --model 2fg7 \
  --isaac-python "$ISAAC_SIM_DIR/python.sh" \
  --asset "$ISAAC_ASSET" \
  --output-dir "$ISAAC_RESULTS/kinematic_regression"
```

The runner checks:

- namespaced state and an advancing global `/clock`;
- standard open, midpoint, close, and cancellation behavior;
- observable state-stream and `/clock` loss followed by recovery after a
  managed bridge restart;
- switching from conventional to realtime resource ownership;
- realtime position and velocity, including held-command endpoint behavior;
- stale-command stopping and position hold; and
- that uncalibrated simulated force remains unavailable.

A pass exits with code `0` and writes
`kinematic_regression/2fg7_kinematic_qualification.json` with `status` set to `passed`.
Preserve that JSON plus `bridge.log`, `bringup.log`, and `bridge_ready.json`.

## 8. Run the dynamic contact/grasp test

Run this only after the kinematic verification passes. The scene uses the
model's OnRobot-supplied outward-facing reference fingers. Its results do not
apply to user-authored finger geometry or physics.

```bash
export ISAAC_DYNAMIC_CONTACT_TEST="$ISAAC_PACKAGE_PREFIX/lib/onrobot_gripper_isaac/run_2fg_dynamic_contact_test.py"
export ISAAC_DYNAMIC_CONTACT_CONFIG="$ISAAC_PACKAGE_PREFIX/share/onrobot_gripper_isaac/config/2fg7_dynamic_contact_reference_objects.json"
test -x "$ISAAC_DYNAMIC_CONTACT_TEST"
test -f "$ISAAC_DYNAMIC_CONTACT_CONFIG"

set -o pipefail
"$ISAAC_SIM_DIR/python.sh" "$ISAAC_DYNAMIC_CONTACT_TEST" \
  --model 2fg7 \
  --asset "$ISAAC_ASSET" \
  --config "$ISAAC_DYNAMIC_CONTACT_CONFIG" \
  --output "$ISAAC_RESULTS/2fg7_dynamic_contact.json" \
  2>&1 | tee "$ISAAC_RESULTS/2fg7_dynamic_contact.log"
```

The default test performs five trials with the versioned
`external_pinch_block_v1` object. Every trial must:

- start open with the solid block resting under gravity on the narrow scene
  support and acquire contact between the block and both fingertip rigid
  bodies using a fast free-space approach and a slower contact approach;
- retain bilateral contact while the drive target builds preload at a bounded
  rate;
- stop short of the commanded closing endpoint, providing a grasp/stall
  consistency signal;
- retain bilateral contact after the support collision is removed, while
  gravity remains enabled throughout;
- keep translation, rotation, speed, and total displacement within the
  declared bounds;
- report finite, bounded contact-sensor values; and
- lose object contact and reach the release target after reopening.

The scene moves its support beneath the held block before release so the block
is caught instead of striking the gripper base. After the released block has
settled, the next trial keeps the fingers open while a gravity-free dynamic
placement carrier lifts the block physically back to the starting height. The
carrier uses a driven prismatic joint through the physics solver. The joint
prevents lateral drift and rotation while the drive preserves the measured
release offset and can extend its lift under bounded position feedback. The
dynamic block pose is not overwritten during inter-trial reset.
The reset check verifies the final pose, the carrier endpoint, and the maximum
per-step object displacement, so delayed or teleporting resets fail. A failed
reset ends the run before another closing motion can begin. The support is a
test placement fixture, not part of the gripper asset or the gravity-hold
condition.

The reference block width is model-specific so the 2FG7 and 2FG14 finger
geometries build comparable drive preload before the support is removed. Both
fixtures use the same 20 g reference mass so this geometry/contact comparison
does not depend on the still-uncalibrated model drive gains. This fixture
tests repeated contact and gravity hold; it is not a payload or
force-rating test. Both test materials explicitly select the arithmetic-mean
PhysX friction combine rule; the test does not rely on an implicit
engine default.

The 2FG14 standard fingertips use reviewed primitive collision shapes for the
vertical pad and lower L-shaped section. The imported detailed fingertip mesh
remains available as geometry but is disabled as a collider. The test also
checks that the mean vertical component of each hold-contact normal stays
within the model-specific limit; a large vertical component indicates contact
on a sloped or horizontal surface rather than the intended pinch faces.

With `--gui`, the blue solid block must be between the upper contact surfaces
and remain visibly clear of the gripper base. The harness frames this region
in the active viewport and pauses briefly before motion. It validates
the configured base clearance, fingertip overlap, released clearance, and
commanded contact reach before starting the trials. A placement-contract error
means the scene configuration and asset geometry no longer agree; do not tune
physics parameters around it. This test covers external grip only; internal
grip is not supported by the current realtime firmware.

A pass exits with code `0` and writes a report with `status` set to `passed`.
The force values in this report are uncalibrated physics observations used to
validate contact stability. They are not exposed as a valid simulated gripper
force measurement. The report retains per-finger minimum, mean, maximum, and
final force observations for preload and hold diagnosis. Its raw impulse
summary resolves the normal contact impulse into world coordinates; it does
not measure tangential friction impulse. The report also includes an estimated
minimum normal force based on the declared mass and arithmetic-mean static
friction. Preserve the JSON and log when diagnosing contact behavior. The
results apply to the recorded asset, runner, fixture, and simulator version.

## 9. Run the sideways payload-retention test

Build the package, source the resulting overlay, and select the installed
script, profile, and asset:

```bash
export ISAAC_PACKAGE_PREFIX="$(ros2 pkg prefix onrobot_gripper_isaac)"
export ISAAC_PAYLOAD_TEST="$ISAAC_PACKAGE_PREFIX/lib/onrobot_gripper_isaac/run_payload_retention_test.py"
export ISAAC_PAYLOAD_CONFIG="$ISAAC_PACKAGE_PREFIX/share/onrobot_gripper_isaac/config/payload_retention_profiles.json"
export ISAAC_ASSET="$ISAAC_PACKAGE_PREFIX/share/onrobot_gripper_isaac/assets/2fg7/onrobot_2fg7.usda"

"$ISAAC_SIM_DIR/python.sh" "$ISAAC_PAYLOAD_TEST" \
  --model 2fg7 \
  --asset "$ISAAC_ASSET" \
  --config "$ISAAC_PAYLOAD_CONFIG" \
  --output "$ISAAC_RESULTS/2fg7_payload_retention.json" \
  --gui
```

Repeat with the matching model name and asset path for `2fg14`, `rg2`, and
`rg6`. The default scenario uses each model's datasheet maximum force-fit
payload as a static-gravity retention test. The 2FG profiles use guarded
contact-relative preload; see [holding commands and drive settings](PHYSICAL_MOTION_SHOWCASE.md#run-all-four-grippers)
for the distinction from position-only holding. Run the full-rated dynamic
test separately:

```bash
"$ISAAC_SIM_DIR/python.sh" "$ISAAC_PAYLOAD_TEST" \
  --model 2fg7 \
  --scenario dynamic-showcase \
  --asset "$ISAAC_ASSET" \
  --config "$ISAAC_PAYLOAD_CONFIG" \
  --output "$ISAAC_RESULTS/2fg7_payload_dynamic.json" \
  --recording-dir "$ISAAC_RESULTS/2fg7_showcase_frames" \
  --gui
```

The dynamic scenario first runs the explicit payload-force profile and
relative-pose acceptance checks as a fixed-base PhysX test. With
`--gui`, physics then stops and a presentation-only replay visibly moves the
closed gripper and block through a smooth diagonal lift and two multi-axis
shake cycles. Replay motion is not included in the pass/fail measurements. The
static scenario intentionally leaves the gripper stationary after removing
the block support. When `--recording-dir` is supplied, the replay is captured
as a 30 fps PNG sequence and the report provides the exact `ffmpeg` command
for encoding it to MP4. The recording path requires `--gui` and is not part of
the pass/fail measurements.

Both scenarios preserve the exact asset and fixture hashes in their reports.
The fixture uses the product asset's fingertip material, drive, and solver
settings by default. A report representing the supplied asset must record
`null` for
`fixture_fingertip_material_override`, `fixture_drive_override`, and
`fixture_solver_override`. Use `--apply-profile-physics-settings` only for a
diagnostic comparison; a report produced with that option represents the
override rather than the supplied asset. The report records the composed
position/velocity iteration counts and articulation-contact ordering under
`effective_solver_settings`. TGS velocity iteration counts above four are not
supported by this test; migrate the excess budget to position iterations
instead.
RG6 additionally requires `solve_articulation_contact_last=true` for its
full-rated external-pinch contract. A full Isaac Sim 6 asset regression remains
required after changing any of these values.
Inspect a failed report by phase and measurement; do not relax a threshold or
change contact material solely to turn a failure green. The results apply to
the declared reference geometry, stock-fingertip material assumptions, and
load profile, not arbitrary workpieces or custom fingers.

Run the complete full-rated matrix from a built and sourced workspace with:

```bash
mkdir -p "$ISAAC_RESULTS/payload-retention"
for model in 2fg7 2fg14 rg2 rg6; do
  asset="$ISAAC_PACKAGE_PREFIX/share/onrobot_gripper_isaac/assets/$model/onrobot_${model}.usda"
  for scenario in rated-margin-static dynamic-showcase; do
    "$ISAAC_SIM_DIR/python.sh" "$ISAAC_PAYLOAD_TEST" \
      --model "$model" \
      --scenario "$scenario" \
      --asset "$asset" \
      --config "$ISAAC_PAYLOAD_CONFIG" \
      --output "$ISAAC_RESULTS/payload-retention/${model}_${scenario}.json"
  done
done
```

Treat every process exit and report status as a separate result. A complete
candidate matrix is usable only when every required report contains
`"status": "passed"`; otherwise preserve the failed report and investigate
before making a payload claim. Run one model at a time with `--gui` when
visually checking contact placement; use the headless matrix for the repeatable
automated check. The normal run tests the asset without stage-local material,
drive, or solver overrides. The three fixture-override fields must all be
`null` when checking the supplied asset.

Validate the complete matrix against the currently installed assets, profile,
and runner before relying on it:

```bash
ros2 run onrobot_gripper_isaac validate_payload_retention_results.py \
  --results-dir "$ISAAC_RESULTS/payload-retention" \
  --isaac-version 6.0.1 \
  --output "$ISAAC_RESULTS/payload-retention/matrix_validation.json"
```

This fails on a missing case, stale asset/config/runner digest, stage-local
physics override, reduced payload, failed acceptance check, or runtime-version
mismatch.

For a single unattended run on an Isaac Sim 6.0.1 workstation, use the
installed fixed-scope matrix runner instead of copying the loop above:

```bash
ros2 run onrobot_gripper_isaac run_payload_retention_matrix.py \
  --isaac-python "$ISAAC_SIM_DIR/python.sh" \
  --output-dir "$ISAAC_RESULTS/payload-retention-6.0.1"
```

It runs only the four supported models and the two documented scenarios,
captures one log and JSON report per case, performs exact-input matrix
validation, and writes `matrix_run.json`. That manifest is updated atomically
before and after every case, so a synchronized copy shows current progress and
the final verdict. It stops at the first failed case by default; add
`--continue-on-failure` when collecting the entire failure matrix.
The runner is intentionally one-shot. Start it explicitly for each
asset revision; it exits after writing the reports and aggregate verdict.

### Physical-motion showcase

The fixed-base dynamic case and the live moving-gripper demonstration serve
different purposes. Use the dedicated
[four-gripper physical-motion showcase](PHYSICAL_MOTION_SHOWCASE.md) to move
the complete gripper and full-rated reference payload under live PhysX. That
guide contains the installed and source-tree commands, visual acceptance
criteria, exact-input checks, keyframe capture, and optional video-ready frame
recording for 2FG7, 2FG14, RG2, and RG6.

## 10. Record data for a failure

Before reproducing a ROS-side failure, start this in a third sourced ROS
terminal:

```bash
export BAG_DIR="$ISAAC_RESULTS/rosbag_$(date -u +%Y%m%dT%H%M%SZ)"
ros2 bag record -o "$BAG_DIR" \
  /clock \
  /isaac_joint_commands \
  /isaac_joint_states \
  /joint_states \
  /gripper_state_broadcaster/state \
  /realtime_controller/command \
  /realtime_controller/state \
  /diagnostics
```

Reproduce once, wait two seconds, then stop recording with `Ctrl+C`. Preserve:

- `2fg7_articulation.json` and its log;
- `bridge_ready.json` and `bridge.log`;
- the ROS bringup terminal output; and
- the rosbag directory.

## Troubleshooting

**`python.sh` is missing:** `ISAAC_SIM_DIR` points at the source repository
instead of its built release directory. Use
`IsaacSim/_build/linux-x86_64/release`.

**The bridge prints `ready`, but ROS cannot see its topics:** verify that both
terminals use the same ROS environment, `LOCALHOST` discovery, and
`rmw_fastrtps_cpp`, and that both processes are on the same machine. Do not
begin with a remote ROS host; that adds DDS discovery and firewall variables
to the asset test.

**The Isaac window opens but appears empty:** wait for stage loading to finish,
then use the viewport's frame-all command. The asset entry point contains a
saved perspective camera, but a local viewport layout can override it.

**Bringup reports stale Isaac state:** confirm the Isaac timeline was not
stopped and `/isaac_joint_states` is updating. Restart the bridge if the stage
was manually stopped or replaced.

**The articulation test fails an endpoint:** do not tune stiffness or damping
by eye. Keep the JSON and log and use the measurements for a repeatable
drive-gain adjustment.

**The dynamic contact test never acquires bilateral contact:** rerun once with
`--gui` and preserve the report. Confirm that the configured solid block is
centered between the upper contact regions of the supplied outward-facing
fingers and does not intersect the gripper base.
Failed reports include a bounded list of contact-body pairs for each fingertip;
use those paths to distinguish reference-object contact from self-contact.

# Four-gripper physical-motion showcase

Run either of these live PhysX examples with 2FG7, 2FG14, RG2, or RG6:

- [Table pickup](#pick-a-workpiece-from-a-table): grasp a cube with the fingers
  pointing downward, lift it, swing it like a pendulum, and place it back.
- [Sideways grip](#run-all-four-grippers): hold a payload while the gripper
  lifts upward and backward, then shakes along a diagonal path.

Both examples simulate the gripper and payload during motion. Their reports
record the settings and physical outcomes; results apply to the tested
workpiece, materials, and motion, not to every application.

## Pick a workpiece from a table

For a downward-facing grasp followed by pendulum motion, use the table-pickup
example. It needs Isaac Sim 6.0.1, but no ROS nodes, robot, or real gripper.
The default rigid cube is 62 mm on each side and weighs 0.712 kg.

From a built and sourced workspace:

```bash
export ISAAC_SIM_DIR=/path/to/isaac-sim
export ISAAC_PACKAGE_PREFIX="$(ros2 pkg prefix onrobot_gripper_isaac)"
export MODEL=2fg7

"$ISAAC_SIM_DIR/python.sh" \
  "$ISAAC_PACKAGE_PREFIX/lib/onrobot_gripper_isaac/run_workpiece_showcase.py" \
  --model "$MODEL" --cycles 3 --gui --screenshots \
  --output "isaac-results/workpiece-$MODEL.json"
```

The same executable accepts `rg2`, `2fg14`, and `rg6`. To run without a ROS
installation, invoke `scripts/run_workpiece_showcase.py` directly from the
source `onrobot_gripper_isaac` directory with Isaac's `python.sh`.

The gripper lowers, closes, waits for stable contact, lifts 120 mm, swings approximately ±20° at 1.5 Hz,
returns the cube to the table, releases, and withdraws. The cube stays dynamic
throughout; it is not attached, teleported between cycles, or replayed. Use
`--swing-axis X` for motion perpendicular to the default swing across the jaws.

The example records friction, drive ceilings, solver settings, contact forces,
measured motion, slip, placement error, and asset/source hashes. The cube uses
static friction 0.6, dynamic friction 0.5, and average combining. Fingertips
retain their asset materials, whose effective bindings are recorded separately.
These are explicit material assumptions, not measured coefficients or a payload
rating. `--effort-fraction 0.02` provides an intentionally weak-grip comparison;
expect its report to fail. A successful normal run must report `passed` for
every cycle and visibly return the cube to its starting location.
Before approaching, a supported-weight check verifies the contact observer's
sign and scale against the cube's known weight. Normal impulses and separate
friction-anchor impulses are recorded; neither is hardware force feedback.
Each saved sample also includes `cube_contact_force_average`, the mean over
every physics step since the preceding sample, with its step count and duration.
Use these interval averages for force balance: sampling isolated impulses can
give a biased average. Instantaneous forces and per-step motion checks remain
separate.

Before lifting, the example requires 200 ms of stable jaw and cube positions
with contact on both fingertips. It waits at least 0.5 s after closing and
fails the cycle if contact has not settled within 3 s. These bounds are adjustable
with `--preload-seconds` and `--grasp-settle-timeout`. Waiting for contact to settle
does not attach or freeze the cube.

Use a fresh output name for each experiment. `--physics-hz 960` compares a finer
physics timestep. Keep all other settings fixed when comparing results.

### Contact and drive settings

The examples use TGS with external forces applied at every position iteration
(`physxScene:enableExternalForcesEveryIteration = true`). This scene-level
setting reduces loaded-contact drift; it does not increase friction or grip
effort. Configure it on your own `PhysicsScene` when composing an application.
The workpiece runner records its effective value under `solver`; use
`--no-external-forces-every-iteration` only for a controlled comparison. The
default timestep is 1/480 s; repeat demanding contact cases at 1/960 s.

The supplied fingertips use static friction 0.6 and dynamic friction 0.5 with
average combining. This is a dry rubber/metal reference assumption. Surface
finish, contamination and a different workpiece can change the real coefficient.

| Model | Driven coordinate | Stiffness, SI | Damping, SI | Drive ceiling | Primary-joint armature |
|---|---|---:|---:|---:|---:|
| 2FG7 | One-finger travel | 20000 N/m | 282.8427 N s/m | 280 N | 1 kg |
| 2FG14 | One-finger travel | 40000 N/m | 900 N s/m | 560 N | 5.0625 kg |
| RG2 | Finger angle | 100 N m/rad | 8 N m s/rad | 4.5 N m | 0.29435 kg m² |
| RG6 | Finger angle | 100 N m/rad | 26.1916 N m s/rad | 16.17 N m | 1.715 kg m² |

USD stores angular drive gains per degree. The RG values in the files are
therefore approximately 1.745329 for stiffness and 0.139626 / 0.457130 for
RG2 / RG6 damping. RG mimic joints use a damping ratio of 1.0. The primary
armature reflects the firmware motor-inertia reference through the gear ratio:
`J_motor × ratio²`, using 0.0000035 kg m² and ratios 290 / 700. It adds joint-space
inertia, not mass or weight to the housing. These are model references, not
individually measured motor inertias; gearbox friction, compliance and controller
behavior are not fully identified. RG6 damping uses the corresponding critical
damping estimate; RG2 damping also matches bounded unloaded response timing.

The 2FG armatures are provisional joint-space conditioning assumptions, not
measured rotor inertias. The 2FG14 value scales the 2FG7 assumption by the
squared firmware gear-ratio difference. Matched damping approximates
`2 sqrt(stiffness × armature)`; adding armature without damping causes free-motion
oscillation. Neither this servo model nor its fast bare-joint response models
the firmware's command slew, voltage loop or hardware timing.

A drive ceiling is a limit on generalized actuator effort, not a requested or
measured grip force. A position drive below that limit depends on its gains,
position error and velocity. Opposing pad normals, their sum, and the drive
effort are different quantities. These settings must not be interpreted as a
calibrated mapping from a hardware force command to contact force. The report
records the composed settings actually used by the simulation.

Following a real gripper's measured position in a shadow scene does not by
itself reproduce its grip force. Contact geometry and position error can differ
between the real and simulated workpieces. Validate the simulated holding
controller against contact measurements as well as visual position agreement.

For controlled comparisons, `--hold-control effort` switches to direct
generalized effort after closing. `--hold-control preload-position` instead
uses the 2FG's observed contact position to preload the existing position drive
at its unchanged effort ceiling. It changes a drive target, not a body pose,
and requires contact on both pads. These options are explicit diagnostics, not
calibrated hardware force controllers; the default remains `position`. Compare
480 and 960 Hz results before drawing conclusions from contact-force values.
The contact-relative preload ramps smoothly over `--force-ramp-seconds` (0.25 s
by default) before the settling check and releases by opening the drive again.
`--drive-armature-kg` (2FG) and `--drive-armature-kg-m2` (RG) are explicit
reflected-inertia diagnostics, not added body mass or a hardware calibration.
Changing a cube's mass while retaining its dimensions changes its implied
density; a concentrated rated-load test is not automatically a realistic solid
metal workpiece. The report includes the implied bulk density.

## Prerequisites

- A built and sourced OnRobot ROS 2 workspace.
- An Isaac Sim installation whose `python.sh` can start the simulator.
- Enough free GPU memory to run one Isaac Sim process at a time when using the
  GUI.

Set the workspace and output locations:

```bash
export ONROBOT_WS=/path/to/ros2_ws
export ISAAC_SIM_DIR=/path/to/isaac-sim
export ISAAC_RESULTS="$ONROBOT_WS/isaac-results/physical-motion-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$ISAAC_RESULTS"
source "$ONROBOT_WS/install/setup.bash"
```

## Run all four grippers

The sideways static and diagonal-motion profiles use contact-relative
`preload-position` holding for 2FG7 and 2FG14. After closing, the runner checks
recent contact with **both** fingertips and smoothly moves the drive reference
below that contact position by `maximum effort / stiffness`. The physical
joint limits, gains, effort ceiling, friction and body poses are unchanged.
The report records the chosen mode and `impedance_preload`; the nominal load
is not a calibrated force measurement. RG profiles retain position holding.
Use `--hold-control position` on an individual runner for a position-only
comparison; it can drop a heavy block when its position error produces too
little squeezing force. The table-pickup example above keeps its separate,
explicit default of `position`.

```bash
ros2 run onrobot_gripper_isaac run_physical_motion_showcase_matrix.py \
  --isaac-python "$ISAAC_SIM_DIR/python.sh" \
  --output-dir "$ISAAC_RESULTS" \
  --gui \
  --record-frames
```

The launcher runs one simulator process at a time and prints the resolved
showcase executable, its SHA-256 digest, and the asset root before starting.
It writes one report, log, four live-viewport keyframes, and a gapless 30 fps
PNG sequence per model, followed by `motion_matrix_run.json` and
`motion_matrix_validation.json`.

The run is successful only when both aggregate files report `passed`. The
validator checks the full declared payload, both independent motion axes,
diagonal travel, payload retention, product-joint stability, exact runner and
asset digests, and the expected keyframes.

Portable reports store media paths relative to their result directory. When a
retained report still contains absolute paths from the workstation that
produced it, validate a synchronized copy by mapping that original report
root explicitly:

```bash
ros2 run onrobot_gripper_isaac validate_physical_motion_results.py \
  --results-dir "$ISAAC_RESULTS" \
  --isaac-version 6.0.1 \
  --require-screenshots \
  --require-recording \
  --report-root "/path/to/original/physical-motion-run"
```

The mapping resolves only paths below the supplied original root into the
local result directory. Traversal, missing files, and symlink escapes fail the
validation. Validation writes a separate verdict and never changes the
evidence reports.

Use a new output directory for each recorded run. A run stops instead of
mixing new frames with an existing sequence.

## Run one gripper

```bash
export MODEL=2fg7
export ISAAC_PACKAGE_PREFIX="$(ros2 pkg prefix onrobot_gripper_isaac)"

"$ISAAC_SIM_DIR/python.sh" \
  "$ISAAC_PACKAGE_PREFIX/lib/onrobot_gripper_isaac/run_physical_motion_showcase.py" \
  --model "$MODEL" \
  --asset "$ISAAC_PACKAGE_PREFIX/share/onrobot_gripper_isaac/assets/$MODEL/onrobot_$MODEL.usda" \
  --config "$ISAAC_PACKAGE_PREFIX/share/onrobot_gripper_isaac/config/payload_retention_profiles.json" \
  --output "$ISAAC_RESULTS/${MODEL}.json" \
  --screenshot-dir "$ISAAC_RESULTS/${MODEL}-keyframes" \
  --recording-dir "$ISAAC_RESULTS/${MODEL}-recording" \
  --gui
```

Use `2fg7`, `2fg14`, `rg2`, or `rg6` for `MODEL`.

The recording is captured during the same live, manually stepped PhysX run as
the test result. It is not a replay. Convert the frames to MP4 with
the exact `motion.recording.ffmpeg_command` written to the model report. The
equivalent command is:

```bash
ffmpeg -y -framerate 30 \
  -i "$ISAAC_RESULTS/${MODEL}-recording/frame_%05d.png" \
  -c:v libx264 -pix_fmt yuv420p -crf 18 \
  "$ISAAC_RESULTS/${MODEL}-recording/showcase.mp4"
```

## Verify source changes without a stale install

During asset or harness development, run the synchronized package source
directly:

```bash
export ONROBOT_ISAAC_SOURCE="$ONROBOT_WS/src/onrobot-ros2/onrobot_gripper_isaac"

"$ISAAC_SIM_DIR/python.sh" \
  "$ONROBOT_ISAAC_SOURCE/scripts/run_physical_motion_showcase_matrix.py" \
  --isaac-python "$ISAAC_SIM_DIR/python.sh" \
  --package-root "$ONROBOT_ISAAC_SOURCE" \
  --output-dir "$ISAAC_RESULTS" \
  --gui \
  --record-frames
```

Confirm that the printed source path is the one you intended. The runner
digest in every model report must equal the printed digest.

## Expected motion

After acquiring the block and removing its support, the whole gripper moves
approximately 75 mm upward and 40 mm backward. It then completes five 2 Hz
cycles with approximately 50 mm horizontal, 20 mm vertical, and 54 mm diagonal
peak-to-peak travel. Blue markers show the diagonal path, and the backdrop
identifies the OnRobot model and payload used for the run. The gripper and
payload must visibly move together; changing force values without gripper
translation is not a valid run.

The reports show the behavior of the supplied reference fingers, declared
block, material assumptions, load profile, asset revision, and simulator
version. They do not establish performance for arbitrary workpieces, custom
fingers, mounts, or motion profiles. See
[Isaac Sim verification](ISAAC_SIM_VERIFICATION.md) for the complete test and
troubleshooting procedure.

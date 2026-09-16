# Simulated grip-setting experiment

This optional Isaac Sim workflow demonstrates a compact simulate–data–train
loop with a 2FG7. It collects grasp-and-lift outcomes, trains a deterministic
setting selector, then renders new baseline and selected-setting trials.

The result applies only to the recorded asset, workpiece, material settings,
payload and motion. It is not a hardware calibration, safety limit or general
gripping-performance claim.

## How the comparison stays fair

Each environment fixes payload, fingertip and workpiece friction, pose offset,
asset and motion. The collector changes only simulated drive effort within that
environment. Entire environments, including every effort trial, are assigned
to train, validation or test before simulation starts. The selector is trained
only on train environments and reports validation and test outcomes separately.

The final visual comparison is a new Isaac execution in one held-out
environment. It shows a deliberately weak baseline that drops the payload and
a model-selected setting that retains it. The original test results choose the
environment; they are not reused as the rendered verdict.

## Collect a data set

Use a new output directory. The following compact experiment has ten fixed
environments and five effort settings per environment.

```bash
export ONROBOT_WS=/absolute/path/to/ros2_ws
export ISAAC_SIM_DIR=/absolute/path/to/isaac-sim/_build/linux-x86_64/release
export ISAAC_PACKAGE_PREFIX="$(ros2 pkg prefix onrobot_gripper_isaac)"
export SYNTHETIC_RESULTS="$ONROBOT_WS/isaac-results/synthetic-2fg7"

python3 \
  "$ISAAC_PACKAGE_PREFIX/lib/onrobot_gripper_isaac/run_synthetic_grip_learning.py" \
  --mode collect --model 2fg7 \
  --isaac-python "$ISAAC_SIM_DIR/python.sh" \
  --environments 10 \
  --effort-fractions 0.15,0.35,0.55,0.75,1.0 \
  --output-dir "$SYNTHETIC_RESULTS"
```

Every episode retains its PhysX result, bounded trajectory, generated scene
configuration, random seed, asset and runner hashes. A failed episode stays a
failed label. An incomplete episode is not used for training.

## Train and save checkpoints

```bash
python3 "$ISAAC_PACKAGE_PREFIX/lib/onrobot_gripper_isaac/run_synthetic_grip_learning.py" \
  --mode train \
  --dataset "$SYNTHETIC_RESULTS/dataset.jsonl" \
  --output "$SYNTHETIC_RESULTS/selector.json" \
  --checkpoint-dir "$SYNTHETIC_RESULTS/checkpoints" \
  --predicted-success-threshold 0.75
```

The checkpoints are trained from progressively larger sets of complete train
environments. Validation and test environments remain outside every checkpoint
training set. The final selector chooses the lowest simulated effort that meets
its saved probability threshold; it abstains if none does.

## Prepare and render a visual comparison

```bash
python3 "$ISAAC_PACKAGE_PREFIX/lib/onrobot_gripper_isaac/run_synthetic_grip_learning.py" \
  --mode prepare-presentation \
  --dataset "$SYNTHETIC_RESULTS/dataset.jsonl" \
  --model-file "$SYNTHETIC_RESULTS/selector.json" \
  --output "$SYNTHETIC_RESULTS/presentation_plan.json"

python3 \
  "$ISAAC_PACKAGE_PREFIX/lib/onrobot_gripper_isaac/run_synthetic_grip_learning.py" \
  --mode render \
  --isaac-python "$ISAAC_SIM_DIR/python.sh" \
  --presentation-plan "$SYNTHETIC_RESULTS/presentation_plan.json" \
  --output-dir "$SYNTHETIC_RESULTS/fresh-comparison"

python3 "$ISAAC_PACKAGE_PREFIX/lib/onrobot_gripper_isaac/run_synthetic_grip_learning.py" \
  --mode assemble \
  --presentation-result "$SYNTHETIC_RESULTS/fresh-comparison/presentation_result.json" \
  --output-dir "$SYNTHETIC_RESULTS/presentation-media"
```

`render` records new PhysX runs, including frame sequences and keyframes. It
passes only when the weak baseline drops the payload and the selected setting
retains it. `assemble` encodes the two recordings when `ffmpeg` is available
and writes `comparison.html` for an offline side-by-side view.

Use `--gui` with `render` when presenting the live runs. The recorded media is
an explicit fallback, not evidence for a different asset or scene.

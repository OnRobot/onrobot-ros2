# Demonstrate a 2FG7 in RViz, Isaac Sim, and hardware

This procedure puts the real 2FG7, its RViz robot model, and its Isaac Sim
articulation on screen together. A click in the RViz gripper panel commands
the real gripper through `ros2_control`; both visualizations follow measured
hardware state. Isaac Sim is a read-only shadow in this demonstration and
does not open a second connection to the gripper.

The included Isaac Sim test results were produced with Isaac Sim 6.0.1. Use a
2FG7 with a firmware revision qualified by the Tool API and the current
realtime command map.

## Prepare

Complete the repository's top-level `GETTING_STARTED.md`,
then clear the full gripper travel and make an independent means of removing
power available. Only one Modbus client may connect to the gripper.

In every terminal, source ROS and the built workspace:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/ros2_ws/install/setup.bash
```

Set the local paths once in the Isaac Sim terminal:

```bash
export ISAAC_SIM_DIR=/path/to/isaacsim
export ISAAC_PACKAGE_PREFIX="$(ros2 pkg prefix onrobot_gripper_isaac)"
export ISAAC_ASSET="$ISAAC_PACKAGE_PREFIX/share/onrobot_gripper_isaac/assets/2fg7/onrobot_2fg7.usda"
export ISAAC_HIL_RUNNER="$ISAAC_PACKAGE_PREFIX/lib/onrobot_gripper_isaac/run_hardware_in_loop.py"
```

## Start conventional control

In terminal 1, replace the address and port with those of the gripper:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 host:=GRIPPER_IP port:=502 \
  control:=conventional start_rviz:=true
```

Wait until the RViz panel reports live state and enables its controls. In
terminal 2, run the read-only preflight before starting Isaac Sim:

```bash
ros2 run onrobot_gripper_isaac run_hardware_in_loop.py \
  --model 2fg7 --ros-preflight-only --duration 10 \
  --output /tmp/2fg7_demo_preflight.json
```

Do not continue unless the report says `"status": "passed"`.

In terminal 3, start the read-only Isaac shadow:

```bash
"$ISAAC_SIM_DIR/python.sh" "$ISAAC_HIL_RUNNER" \
  --model 2fg7 --asset "$ISAAC_ASSET" --gui --duration 60 \
  --output /tmp/2fg7_demo_shadow.json \
  --ready-output /tmp/2fg7_demo_ready.json
```

Wait for the runner to report readiness, then arrange the Isaac Sim and RViz
windows side by side. Keep the physical gripper visible directly or through a
camera view. Click **Open**, **Close**, and an intermediate **Send target** in
RViz. The physical gripper moves first; RViz and Isaac Sim follow the measured
aperture rather than an assumed command. Let the 60-second run finish and do
not treat the run as successful unless `/tmp/2fg7_demo_shadow.json` reports
`"status": "passed"`.

If the recording must be stopped early, use Ctrl+C before stopping ROS
bringup. That safely ends the read-only shadow, but its report is marked
`"stopped"` rather than `"passed"`.

## Demonstrate realtime velocity

Stop the conventional launch and confirm that no other client owns the
gripper. Restart terminal 1 with the realtime controller and panel:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 host:=GRIPPER_IP port:=502 \
  control:=realtime start_rviz:=true
```

Start the same fixed-duration read-only Isaac shadow again, using distinct
`--output` and `--ready-output` filenames. In RViz, choose a conservative
velocity, hold the joystick toward **OPEN** or **CLOSE**, and release it to
request Stop. The panel continuously refreshes the command while held. Use
**Center / stop** to request Stop explicitly and let the shadow runner finish.

The hardware backend also provides [2FG closing-force controls](../../onrobot_gripper_rviz_plugins/README.md#2fg-closing-force-grip).
This demonstration shadows measured position; it does not forward regulated
force to the Isaac joint bridge.

## Expected result

- Exactly one ROS hardware component owns the gripper connection.
- The RViz panel reports connected state and live limits before motion.
- The real gripper, RViz model, and Isaac articulation move in the same
  direction and settle at corresponding apertures.
- Stopping the Isaac shadow does not stop or disconnect ROS hardware control.
- Conventional control uses the standard
  `control_msgs/action/ParallelGripperCommand` action; realtime control uses
  the typed OnRobot command and explicit Stop interfaces.

This demonstrates control integration and measured kinematic agreement. It
does not claim calibrated simulated force, payload performance, or validated
behavior for arbitrary user-authored fingers.

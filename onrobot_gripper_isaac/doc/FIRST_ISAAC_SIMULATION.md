# Run a first gripper simulation

This guide opens one supplied gripper in Isaac Sim and controls it from the
same RViz panel used for hardware. It is a functional integration preview. This
The package ships the same user-facing simulation surface for `2fg7`, `2fg14`,
`rg2`, and `rg6`. Choose the model that matches the asset you want to inspect.

## Prerequisites

- Ubuntu 24.04 with ROS 2 Jazzy and the supplied Tool API packages installed;
- this repository built and sourced; and
- an Isaac Sim installation with an executable `python.sh`.

Set the paths for your machine and select one of `2fg7`, `2fg14`, `rg2`, or
`rg6`:

```bash
export ONROBOT_WS=/absolute/path/to/ws
export ISAAC_SIM_DIR=/absolute/path/to/isaac-sim
export MODEL=2fg7

source /opt/ros/jazzy/setup.bash
source "$ONROBOT_WS/install/setup.bash"
export ISAAC_PACKAGE_PREFIX="$(ros2 pkg prefix onrobot_gripper_isaac)"
export ISAAC_ASSET="$ISAAC_PACKAGE_PREFIX/share/onrobot_gripper_isaac/assets/$MODEL/onrobot_$MODEL.usda"

test -x "$ISAAC_SIM_DIR/python.sh"
test -f "$ISAAC_ASSET"
ros2 run onrobot_gripper_isaac validate_asset_contract.py --model "$MODEL"
```

Continue only when the contract check reports `passed`.

## Start Isaac Sim

In the first terminal, keep the same environment and run:

```bash
"$ISAAC_SIM_DIR/python.sh" \
  "$ISAAC_PACKAGE_PREFIX/lib/onrobot_gripper_isaac/run_ros_bridge.py" \
  --model "$MODEL" --asset "$ISAAC_ASSET" --gui
```

Wait for the bridge to print `"status": "ready"` and leave it running. The
script starts the simulation timeline; do not stop it in the Isaac window.

## Open the RViz controls

In a second terminal, source the same workspace and start conventional
control:

```bash
source /opt/ros/jazzy/setup.bash
source "$ONROBOT_WS/install/setup.bash"
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:="$MODEL" backend:=isaac control:=conventional start_rviz:=true
```

The matching panel is loaded automatically. Wait for live state, then click
**Open**, **Close**, or select an aperture and click **Send target**. The model
must move in the same direction in RViz and Isaac Sim.

Stop this launch with `Ctrl+C` before trying realtime control:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:="$MODEL" backend:=isaac control:=realtime start_rviz:=true
```

The realtime panel provides aperture **Open**, **Close**, and **Send target**
buttons plus a held velocity joystick. For 2FG, its velocity setting is also
the maximum position-approach speed. For RG, the position command uses the
displayed force limit while the joystick uses mechanism angular velocity.
Release the joystick or click **Center / stop** to request Stop.

## Insert the asset under a scene parent

The supplied 2FG7 and RG2 entry USD files can also be referenced below a
translated and rotated `Xform`. Keep the entry file and its relative payload
layers together, reference the entry file below the parent, and provide one
`PhysicsScene` in the host scene. Set the host stage to metres-per-unit `1.0`
with Z up before starting physics.

Resolve the articulation root and joints below the inserted instance, rather
than assuming the standalone root path. The tested interfaces retain their
model units: 2FG7 drives `finger_stroke` as a prismatic distance in metres;
RG2 drives `finger_joint` as a revolute angle in radians. The insertion check
verified the relative dependency closure, remapped joint targets, both-finger
motion, and position/velocity control for both models on Isaac Sim 5.0.0 under
a non-default parent transform.

Here, `instance` means the referenced root prim below the scene parent:

| Model | Articulation root | Physical joint | Unit |
|---|---|---|---|
| 2FG7 | `<instance>/Physics/root_joint` | `<instance>/Physics/finger_stroke` | metres |
| RG2 | `<instance>` | `<instance>/Physics/finger_joint` | radians |

Position and velocity checks are scoped to the inserted Isaac articulation.

The normal first run above uses each asset as a standalone fixed-root model.
Scene insertion applies a parent transform to that fixed-root asset; it does
not establish behavior for an arbitrary robot articulation, robot controller,
or mounted application.

For repeatable articulation, kinematic, and hardware-in-the-loop checks,
continue with [Isaac Sim verification](ISAAC_SIM_VERIFICATION.md). Contact
behavior depends on the scene and is not a calibrated force or payload claim.

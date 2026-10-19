# Use OnRobot grippers in Isaac Sim

The separate, non-ROS `onrobot-isaac-sim` repository contains the 2FG7, 2FG14, RG2
and RG6 USD assets. `onrobot_gripper_isaac` installs compatible assets and
provides examples and the ROS bridge. A USD can be loaded without ROS.

| Your task | Start here |
|---|---|
| Load and move one gripper | [First simulation](../onrobot_gripper_isaac/doc/FIRST_ISAAC_SIMULATION.md) |
| Grip, lift and shake a workpiece | [Physical motion example](../onrobot_gripper_isaac/doc/PHYSICAL_MOTION_SHOWCASE.md) |
| Check articulation, contact and payload | [Simulation checks](../onrobot_gripper_isaac/doc/ISAAC_SIM_VERIFICATION.md) |
| Collect simulated attempts and fit a selector | [Synthetic grip example](../onrobot_gripper_isaac/doc/SYNTHETIC_GRIP_LEARNING.md) |
| Follow measured ROS/hardware motion | [2FG7 HIL](../onrobot_gripper_isaac/doc/RVIZ_ISAAC_2FG7_DEMO.md), [RG2 HIL](../onrobot_gripper_isaac/doc/RG2_HIL_GUIDE.md) |

After building and sourcing the workspace, locate the installed asset:

```bash
export MODEL=2fg7
export ISAAC_PACKAGE_PREFIX="$(ros2 pkg prefix onrobot_gripper_isaac)"
export ISAAC_ASSET="$ISAAC_PACKAGE_PREFIX/share/onrobot_gripper_isaac/assets/onrobot_${MODEL}/simready_usd/onrobot_${MODEL}.usda"
test -f "$ISAAC_ASSET"
```

Use Isaac Sim 6.1.0 for the documented assets. The
[asset dependency reference](../onrobot_gripper_isaac/README.md#asset-dependency)
explains separate checkouts, minimum versions and interface compatibility. Do not maintain a second
editable USD copy in the ROS package.

For `backend:=isaac`, start the bridge before ROS bringup as shown in the first
simulation guide. A hardware shadow instead subscribes to measured state;
only the existing ROS hardware session connects to the gripper. The shadow
follows position but does not reproduce the real gripper’s force control.

Drive, contact and solver inputs belong to the simulated scene. Use the supplied
scene settings and validate your own workpiece; do not infer measured jaw force
or universal payload retention from a screenshot or a single passing example.

# OnRobot gripper hardware

This package connects OnRobot grippers to `ros2_control`. It exports hardware
plugins for physical devices, deterministic fake systems, and the simulation
adapters available in this repository.

Applications interact with standard command and state interfaces. The plugin
owns the ROS-independent tool API session and translates between:

- `grip_stroke`, the finger-dependent task aperture used by the standard
  parallel-gripper action and MoveIt 2; and
- the model's physical mechanism coordinate used for feedback and
  visualization.

For RG grippers, task aperture includes the device's configured fingertip
offset. That setting must match the installed fingertips; changing the visual
model does not calibrate the device. The standard RG2 visual model follows
measured compensated aperture. Raw mechanism-angle feedback stays available
separately for custom-finger applications.

The fake RG backend uses the supplied inward-facing standard fingertip CAD
geometry. Zero aperture places the rubber surfaces in contact, and the
published aperture limits are bounded by that geometry and the selected joint
range. Custom fingertips require a matching geometry mapping.

Communication runs in a worker-owned session. The controller-manager update
loop exchanges bounded command and state images and does not perform Modbus
I/O. Connection faults remain observable through the semantic state and
diagnostics broadcasters until recovery succeeds.

Each physical read makes one nonblocking state handoff. If the session is
publishing a sample at that instant, the adapter retains the last coherent
measurement and continues to report its increasing `sample_age`; validity and
sequence values change only when a newer coherent sample is received.
During a transient invalid sample, the last finite physical finger pose may
remain visible in TF. The task position used by action controllers is never
replaced by a held visualization value. The joint-state filter omits unavailable
positions, preserving array alignment and the original timestamp. The
corresponding validity flags remain false, unavailable velocity/force values
remain unavailable, and a lifecycle reset or explicit device fault clears the
held pose.

The parallel-gripper plugins also expose a resource-claimed Stop sequence used
by the conventional action controller. Cancelling or preempting a standard
gripper goal therefore interrupts the device command instead of only changing
the action status. A separate one-shot conventional-goal event distinguishes
new action intent from repeated numeric position/effort values. The hardware,
fake and Isaac adapters consume that event once; a canceled or deactivated
goal is cleared before it can be replayed.

Use the showcase package when you need the public launch interface with RViz:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 backend:=fake start_rviz:=true
```

See the workspace Getting Started guide for physical
hardware and the installed Isaac package documentation for
the available simulation backend.

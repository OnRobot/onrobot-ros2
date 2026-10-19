# OnRobot gripper hardware

This package connects OnRobot grippers to `ros2_control`. It exports hardware
plugins for physical devices, deterministic fake systems, and the simulation
adapters available in this repository.

Applications interact with standard command and state interfaces. The plugin
owns the ROS-independent tool API session and translates between:

- `grip_stroke`, the finger-dependent task aperture used by the standard
  parallel-gripper action, including commands from the MoveIt 2 adapter; and
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

Communication runs in a session on a worker thread. The controller-manager
update loop exchanges commands and cached state with that thread, without
performing Modbus I/O. Connection faults remain visible through the gripper state and
diagnostics broadcasters until recovery succeeds.

Recovery controllers use protocol version 2 state interfaces declared in the
supplied Xacros. Update hardware, controllers and robot descriptions together.
Custom descriptions must also export the fields named in
`onrobot_gripper_msgs/fault_recovery_protocol.hpp`, alongside the existing
recovery command, acknowledgement and admission interfaces. Physical RG
backends require valid, clear safety feedback; kinematic simulation does not
invent physical switch state. Use synchronous hardware/controller updates.
See [state and recovery](https://github.com/OnRobot/onrobot-ros2/blob/main/docs/state-diagnostics-and-recovery.md) for usage.

Each physical read tries once to copy the session state without waiting for
its lock. If the session is publishing a sample at that instant, the adapter keeps the last complete
measurement and continues to report its increasing `sample_age`; validity and
sequence values change only when a newer complete sample is received.
During a transient invalid sample, the last finite physical finger pose may
remain visible in TF. The task position used by action controllers is never
replaced by a held visualization value. The joint-state filter omits unavailable
positions, preserving array alignment and the original timestamp. The
corresponding validity flags remain false, unavailable velocity/force values
remain unavailable, and a lifecycle reset or explicit device fault clears the
held pose.

The parallel-gripper plugins also expose a Stop command interface used
exclusively by the active motion controller. Cancelling or replacing a standard
gripper goal therefore interrupts the device command instead of only changing
the action status. A separate event identifies each new conventional goal,
even if its position and effort match the previous goal. The hardware,
fake and Isaac adapters process that event once; a canceled or deactivated
goal is cleared before it can be replayed.

Use the showcase package when you need the public launch interface with RViz:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 backend:=fake start_rviz:=true
```

See the workspace Getting Started guide for physical
hardware and the installed Isaac package documentation for
the available simulation backend.

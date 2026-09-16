# OnRobot gripper messages

This package defines the model-neutral ROS 2 interfaces used when standard
joint state and gripper actions cannot express the complete device state.

- `GripperState` reports task aperture, raw mechanism state, connection and
  firmware status, mapping identity, validity flags, faults, and health
  counters. For RG models it also reports both safety-switch channels and the
  safety DC-error flag; `safety_status_valid=false` means unavailable rather
  than clear.
- `RealtimeCommand` carries semantic position, velocity, and force commands.
- `RealtimeState` reports the active realtime mode, independently valid task
  and mechanism observations, and session counters.

All linear quantities use metres and metres per second. Angular quantities use
radians and radians per second. Consumers must inspect each validity flag; a
numeric value paired with `false` is a placeholder, not a measurement.

`GripperState.realtime_force_control_available` reports whether the selected
backend accepts the two 2FG closing-force modes. Check this flag together with
freshness, mapping and fault state. It does not certify measured force accuracy.
Fake kinematics can accept these commands without a contact model; in that case
`force_valid` is false and `grip_detected` remains false.

Normal open/close and planned aperture commands use
`control_msgs/action/ParallelGripperCommand`. These messages complement that
standard interface rather than replace it.

## Diagnostics

`onrobot_gripper_controllers/GripperStateBroadcaster` also publishes a standard
`diagnostic_msgs/msg/DiagnosticArray`. For a standalone launch the topic is
`/diagnostics`; a namespaced gripper gets its own `/<namespace>/diagnostics`
stream, and the broadcaster's `diagnostics_topic` parameter can select another
remappable name.

The array contains connection, fault, safety, freshness, and health-counter
keys plus the available status-register values for the selected model. Values
are named with their units where the unit is part of the public contract, for
example `diagnostic_external_width_mm`,
`diagnostic_realtime_linear_velocity`, or `diagnostic_temperature`. Missing or
unsupported measurements are omitted from the diagnostic array and are
represented by validity flags in `GripperState`; a numeric placeholder must not
be treated as zero.

For 2FG7, 2FG14, RG2 and RG6, `GripperState.firmware` contains the version read
from the connected gripper, including the source hash when the device supplies
one. Unknown or invalid identity is an empty string, not a configured version.
Diagnostics use `firmware: unknown` in that case. A configured `firmware` label
is reported separately as `configured_firmware`. Connection identity is
refreshed after reconnect; it does not imply the motion sample is fresh.

With bringup running, inspect state and diagnostics without opening another
connection to the gripper:

```bash
ros2 topic echo --once --qos-reliability best_effort /gripper_state_broadcaster/state
ros2 topic echo --once /diagnostics
```

Prefix both topics with the gripper namespace when using multiple instances.
The diagnostic `firmware_source` key distinguishes `device-connection` from
`unavailable`. Board revision, CRC and source-hash fields are provided only by
2FG devices; RG supplies product code and numeric firmware version. 3FG
firmware identity remains unavailable. These fields are not device serial
numbers. Cached diagnostic values may still be present with increasing age;
check `diagnostic_age` and the status level before using them.

The 2FG force values preserve the device's signed measurements in Newtons:
closing into an object is negative and opening into an object is positive.
Positive force targets still request a closing grip. An application that wants
a positive display magnitude must negate a valid closing feedback value; it
must not change the command sign. RG
`diagnostic_command_force` is a command-derived echo because RG devices do not
provide a measured force register, and 3FG `diagnostic_force_percent` is a
percentage rather than Newtons.

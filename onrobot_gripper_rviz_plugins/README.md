# OnRobot gripper RViz plugins

This optional package provides operator panels for commissioning and
demonstrating OnRobot grippers in RViz:

- **Gripper Control** sends the standard parallel-gripper action used by 2FG
  and RG models;
- **Realtime Position and Velocity** sends model-aware aperture targets,
  streams guarded velocity commands, and sends an ordered Stop when a target
  is reached or the joystick is released; and
- **Force & State History** plots read-only typed state from a live session or
  a rosbag replay, including signed measured force where the model provides it,
  validity gaps, grip detection, mode changes, and device faults; and
- **Three-Finger Control** sends the native external-diameter commands used by
  3FG models (evaluation-only ROS path).

The panels use ROS actions, topics, and services only. They do not open a
device connection, so the same controls work with physical, fake, and
supported simulation backends.

Start a configured panel through the showcase package:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 host:=192.168.1.1 port:=502 \
  control:=conventional start_rviz:=true
```

The panels wait for live state, controller availability, and device-provided
limits before enabling motion. The realtime position controls use the same live
aperture limits as the conventional panel. For 2FG, the velocity setting also
limits position-approach speed; for RG, position uses a model-bounded force
limit while velocity remains mechanism angular velocity. The position panel
shows whether the gripper is at the open limit, at the closed limit, or between
the limits; the button for an already-reached endpoint is disabled. On RG
models, both shared panels display the reported safety-switch state and inhibit
their command controls while a switch is pushed, has triggered, or a safety DC
error is present. The safety row stays hidden for 2FG models and other backends
that do not provide RG safety status.

## 2FG closing-force grip

For 2FG7 and 2FG14, the **2FG closing-force grip** group is enabled only when
fresh typed state advertises `realtime_force_control_available`. Select position
or velocity approach, choose a positive closing-force target, then press and
hold **Hold to grip**. Position approach uses the target aperture above, which
must be below the current aperture; velocity approach closes at the selected
velocity limit. Parameters are captured when you press the button. Target ranges
are 30–95 N for 2FG7 and 40–196 N for 2FG14. Closing force feedback is negative,
following the firmware convention; the target is positive.

Releasing the button or losing focus stops force control. **Release / open**
sends a zero-force position command to the live open limit; at least 2 mm of
opening travel must remain. **Center / stop** stops without commanding an opening
position. Support the workpiece before stopping or releasing; these controls
are not safety-rated. An approach without reported contact times out after
30 seconds. The panel displays the device's `grip_detected` flag, not a
prediction based on reaching a target. Validate actual force and retention
for your fingers and workpiece.

The fake backend exercises force-approach motion without a contact model:
force feedback is unavailable and no grip is invented. The Isaac ROS joint
bridge does not advertise regulated-force support. RG controls are unchanged;
these two closing-force modes apply only to 2FG models.

The conventional panel's 2FG force field uses the model's minimum nonzero
force (20 N for 2FG7 and 40 N for 2FG14) through the live conventional ceiling.
The panel selects that minimum by default; the API also accepts an explicit
zero to request the firmware minimum convention. For controllers that expose
runtime speed control, the conventional panel also provides a 1–100 **Speed
(%)** setting and **Apply** button. The display confirms the controller
readback, and a change applies only to future conventional commands. The
launch argument `conventional_speed_percent:=1..100` sets the initial value
(default 50%). This native percentage is not a guaranteed linear speed. See
the [conventional speed panel guide](../docs/realtime-control.md#conventional-2fg-speed-panel).

## Panel routing and state history

For robot-mounted or multi-gripper workcells, the RViz node can remap each
panel through parameters. The conventional panel accepts `action_name`,
`joint_states_topic`, `limits_topic`, `gripper_state_topic`, and
`controller_manager_service`. The realtime panel accepts
`realtime_command_topic`, `realtime_state_topic`, `limits_topic`, and
`gripper_state_topic`. These parameters allow each panel
to control a namespaced gripper from a shared RViz process.
When RViz saves its configuration, these values are stored under that panel's
own `OnRobot panel settings` entry. Saved values take precedence over shared
RViz node defaults, so two panel instances can address different grippers
without parameter collisions.

By default, Gripper Control reads `joint_state_broadcaster/joint_states` in
the RViz node's namespace. The general `joint_states` topic can contain robot
arm joints or other aggregated visualization data. If your broadcaster uses a
different topic, set `joint_states_topic` for the panel. Invalid or stale
aperture feedback disables motion; unavailable effort does not. The panel's
startup log identifies its joint, feedback topic, and action endpoint.

The Three-Finger Control panel uses the same namespace-safe settings pattern.
Its defaults are the relative `diameter_controller/commands`, `joint_states`,
`three_finger_limit_broadcaster/values`, and
`gripper_state_broadcaster/state` topics. It shows the measured external
diameter and, when present, the coupled finger angle. Connection text comes
from the typed gripper state and becomes stale when either the device state or
the measured joint stream stops arriving; it is not inferred from the last
command. Set `three_finger_command_topic`, `joint_states_topic`,
`limits_topic`, or `gripper_state_topic` in that panel's settings when the
topics are remapped.
Invalid or stale measurements are labeled unavailable, and diameter commands
are disabled until valid state and a connected controller are available.
The Force label shows the configured percentage; diameter buttons do not change it.

The history panel is read-only. It subscribes to one typed
`onrobot_gripper_msgs/msg/GripperState` topic and does not subscribe to command,
action, or service interfaces. Its default topic is the relative
`gripper_state_broadcaster/state`, so a panel in a namespaced RViz launch
follows that namespace automatically. Set `gripper_state_topic` in the panel's
`OnRobot panel settings` entry when an absolute topic or a different routing is
needed. The conventional and realtime panels use the same relative-default
rule for their actions and state topics. A supported 2FG force sample is
plotted only when its validity and freshness fields are true. RG force shown
elsewhere in the state contract is command-derived, so the panel labels
measured force as unavailable for RG models.

See the workspace Getting Started guide for the operating procedure.

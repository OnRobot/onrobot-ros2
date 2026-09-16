# Realtime control

Use realtime control on 2FG7, 2FG14, RG2 and RG6 for streaming position or
velocity. First complete [commissioning](hardware-commissioning.md).

The [realtime setup commands](../GETTING_STARTED.md#5-use-realtime-position-and-velocity-control)
include a 200 Hz USB/RTU example with all three rates specified:
`realtime_update_rate_hz`, `controller_manager_update_rate_hz` and
`state_publish_rate_hz`. Raising just the device rate does not raise ROS
publication or controller rates. Check sample age, sequence and timing counters
rather than interpreting repeated publications as fresh hardware samples.

| Control | 2FG7 / 2FG14 | RG2 / RG6 |
|---|---|---|
| Position | Task aperture, m; velocity bounds approach | Compensated task aperture, m; position force limit |
| Velocity | Task-aperture velocity, m/s | Mechanism angular velocity, rad/s |
| Force approach | Positive closing target, N; position or velocity approach | Not a regulated measured-force interface |

The [RViz panel reference](../onrobot_gripper_rviz_plugins/README.md) explains
Open, Close, Send target and the held joystick. Its
[2FG force controls](../onrobot_gripper_rviz_plugins/README.md#2fg-closing-force-grip)
cover contact hold and explicit release. Check
`realtime_force_control_available`; the Isaac joint bridge does not implement
force regulation and fake kinematics do not invent contact or measured force.

Realtime motion requires fresh timestamps and repeated commands, including
position commands. Stale input requests Stop. The panel refreshes a position
target until reached and velocity only while held. Releasing the joystick or
pressing **Center / stop** requests Stop. Activation and recovery never replay
old commands. Only one motion publisher and one active motion controller should
own the gripper.

## Conventional 2FG speed panel

On controllers that advertise runtime conventional-speed support, the
Conventional Control RViz panel shows a **Speed (%)** field and **Apply**
button. Select an integer from 1–100, then Apply. The panel displays the
controller's current value and confirms the change after reading it back.
Changing the setting affects future conventional goals only; it does not
change a motion already in progress. The controller rejects changes while a
goal or Stop handoff is active, and the panel temporarily disables motion
controls while an update is being confirmed. Other panels connected to the
same controller reflect the new value automatically.

This setting is available only when the controller/backend advertises support
(the standard real and fake 2FG configurations). It is hidden for RG/3FG and
unsupported backends. The launch argument `conventional_speed_percent`
initializes the native speed percentage, defaults to 50%, and accepts 1–100;
the panel can change it at runtime. This percentage is sent as the device's
native speed setting, not converted from the standard action's optional
velocity field and not a guaranteed SI velocity.

2FG closing feedback is negative; positive force targets still request closing.
Read [force provenance and validity](../onrobot_gripper_msgs/README.md#diagnostics)
before using force in an application. For 2FG hardware, bringup reapplies the
configured supply-power budget after reconnect; see
[launch parameters](reference/launch-parameters.md).

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
Open, Close, Send target and the held joystick. The realtime-position force
target is positive, captured when each operation starts, and included in every
refreshed 2FG position or velocity command. Its
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

The Conventional Control panel shows **Max aperture speed** in m/s for
supported 2FG7/2FG14 backends. Set it before clicking Open, Close or Send target;
it is sent with that action and does not alter an ongoing motion. Each panel
keeps its own selection. The field is hidden for unsupported controllers.
See the [controller contract](../onrobot_gripper_controllers/README.md) for
the optional velocity field and an action example.

The native `conventional_speed_percent` launch argument remains available for
actions that omit velocity; the panel's explicit m/s selection overrides it.

2FG closing feedback is negative; positive force targets still request closing.
Read [force provenance and validity](../onrobot_gripper_msgs/README.md#diagnostics)
before using force in an application. For 2FG hardware, bringup reapplies the
configured supply-power budget after reconnect; see
[launch parameters](reference/launch-parameters.md).

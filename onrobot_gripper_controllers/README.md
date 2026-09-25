# OnRobot gripper controllers

This package provides the model-neutral `ros2_control` controllers shared by
the supported OnRobot grippers. Applications normally start them through
`onrobot_gripper_bringup` rather than loading the plugins directly.

The package exports:

- `ParallelGripperActionController`, which preserves the standard
  `control_msgs/action/ParallelGripperCommand` interface and issues an explicit
  device Stop when a goal is cancelled or preempted;
- `OnRobotRealtimeController`, which accepts semantic commands on
  `~/command`, publishes `~/state`, and provides `~/stop`;
- `GripperStateBroadcaster`, which publishes typed state on `~/state` and
  standard diagnostics on `/diagnostics` (or the namespace-resolved
  `diagnostics_topic` parameter); and
- `GripperRecoveryController`, which provides an explicit `~/recover` service
  without claiming a motion interface. On RG grippers it refuses recovery
  while either safety switch is pushed or a safety DC error is reported.

The conventional gripper action and the realtime controller claim the same
motion resources, so only one can be active. Realtime commands must be
refreshed before the configured timeout; otherwise the controller requests
Stop. Every motion mode remains a streaming interface, including position
targets. Motion messages need a nonzero `header.stamp` from the controller's
ROS clock and are checked for source freshness both when received and before
first application. A command received while inactive, or before activation,
cannot be reused after activation; the first active update is Stop. A
nonblocking handoff contention is not treated as a timeout, and an explicit
Stop is kept in an independent ordering fence so a coalesced motion cannot
overwrite it. Use one motion publisher; publisher timestamps do not provide
operator arbitration. The RViz target action refreshes its target until
measured completion or its bounded timeout. Each accepted conventional action
goal is treated as new intent even when its position and effort equal the
previous goal; repeated updates of one goal remain coalesced. Cancellation,
deactivation and controller switching use explicit Stop and retire earlier
intent, so an unexecuted goal cannot replay later. A replacement goal stays
buffered until hardware consumes Stop. If cancellation cannot hand off Stop
immediately, the controller rejects cancellation, aborts the action and retries
Stop in its update loop. An accepted cancellation confirms the software handoff,
not that physical motion has already stopped; check device feedback.

The conventional controller requires the configured `max_effort_interface`
on its task joint. It waits for backend command admission before reporting
completion. A rejected force or aperture request aborts the action and requests
Stop; `allow_stalling` does not turn rejection into a successful grasp. Correct
the request and send a new goal. Reported action effort is the requested target,
not a measured contact force.

If position or velocity feedback becomes invalid, the conventional action aborts
and requests Stop. This is not reported as a successful grip or stall, even with
`allow_stalling` enabled. After feedback is valid again, send a new goal to resume;
the interrupted target is not resumed automatically.

Device communication, watchdog handling, and reconnect policy remain in the
hardware session rather than in these controllers.

For 2FG7/2FG14, `FORCE_POSITION` uses a closing-force target and a
nonnegative approach-speed limit; `FORCE_VELOCITY` uses a closing-force
target and a signed approach velocity. The session checks the firmware and
model force range before admission. On 2FG7 firmware 1.0.34 or newer,
headless clients may also request zero or subminimum force: opening ignores
force, while closing uses at least 30 N. Zero force is not Stop; negative
force targets are rejected. Follow the [realtime panel instructions](../onrobot_gripper_rviz_plugins/README.md)
for grip, release and Stop; the typed state advertises backend availability.

The diagnostic status uses the standard `diagnostic_msgs/msg/DiagnosticArray`
shape. It contains one status for this broadcaster, with model/profile,
connection, freshness, safety, health-counter, and available device-telemetry
keys. `status.level` and `status.message` summarize health; consumers should
use the typed state validity flags before using an individual numeric value.
The RG force key is command-derived, not measured contact force, and the 3FG
force key is a percentage.

The conventional action accepts task-aperture position and optional effort.
For 2FG7/2FG14, `command.velocity` is an optional maximum aperture speed, in
m/s. The controller selects the native speed using an estimated full-travel
peak at the requested force.

Supply one finite positive value, or leave the array empty to use
`conventional_speed_percent`. Bounds above the modeled peak select 100%;
bounds below the modeled minimum select 1%, with a warning that the speed may
exceed the request. Conversion uses the faster of opening and closing.
It uses positive `command.effort`, `max_effort` if
omitted, or `default_force_n` for explicit zero effort. Fresh live force and
motor context is required; a context change before dispatch aborts the goal.
RG/3FG and controllers without conventional-speed support reject nonempty
velocity arrays.

For example, the standard action message uses arrays, including for the
optional velocity field:

```bash
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {name: [grip_stroke], position: [0.060], effort: [40.0], velocity: [0.080]}}"
```

The velocity value above is 0.080 m/s, not a native percentage.

RG safety-switch status is published in the typed gripper state and in
`/diagnostics`. The `safety_status_valid` field distinguishes unavailable data
from an all-clear state. Once the physical switch is released and the work area
is clear, `~/recover` requests an explicit Stop/recovery transition. A safety
DC error requires a physical power cycle of the gripper. The released firmware
does not expose an operational Modbus reboot command; its firmware-update reset
mechanism is not a recovery interface and is intentionally not used by this
controller. A successful service response confirms that a request was queued,
not that hardware admitted or completed recovery. See the
[state and recovery guide](../docs/state-diagnostics-and-recovery.md) for the
result, inactive-call, and controller-reload behavior. ROS reporting is
supplementary and is not a safety-rated stopping function.

Use the interactive showcase to exercise the supported control path:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=rg2 host:=192.168.1.1 port:=502 \
  control:=realtime start_rviz:=true
```

See the workspace Getting Started guide for safety,
model support, conventional control, and recovery instructions.

# OnRobot parallel-gripper bringup

This package is the model-neutral entry point for the supported parallel
grippers.
It selects 2FG7, 2FG14, RG2, or RG6 while preserving the same conventional
action name and controller contract.

```bash
ros2 launch onrobot_gripper_bringup gripper.launch.py \
  model:=2fg7 backend:=fake
```

This is a headless launch. Use `onrobot_gripper_demos/showcase.launch.py` when
you need RViz and the optional operator panels.

For multiple independent instances, give each gripper a ROS namespace and a
TF frame prefix:

```bash
ros2 launch onrobot_gripper_bringup gripper.launch.py \
  model:=2fg7 backend:=fake namespace:=left_gripper \
  frame_prefix:=left_gripper_
```

Controller managers, actions, state topics, recovery services, and cleaned
joint states then remain below the instance namespace. Robot state publisher
applies the frame prefix to every link frame. A launch test runs two grippers,
commands distinct apertures through their standard actions, and verifies the
two prefixed TF trees. For inclusion in one combined robot URDF, use the
prefix-safe product Xacro macros described by
`onrobot_gripper_description`; `frame_prefix` does not rename URDF joints.

Use `backend:=real` plus the endpoint `host` and `port` for hardware. The fake
backend is model-aware: conventional commands update the 2FG physical
`finger_stroke` joint or RG linkage, and 2FG/RG realtime commands update the
same state/RViz contract. The typed state also retains the separate raw
mechanism measurement.
The control panel waits for backend-published task-aperture limits instead of
using static model guesses.

The same launch path supports direct Modbus RTU for real hardware. Set
`transport:=rtu`, `serial_device:=...`, `baud_rate:=115200` or `1000000`, and
`slave_id:=65`, `66`, or `67`; `rtu_parity:=even` selects the fixed OnRobot 8E1
framing. The three timeout arguments are positive milliseconds and are passed
to the Tool API session. For example:

```bash
ros2 launch onrobot_gripper_bringup gripper.launch.py \
  model:=2fg7 backend:=real transport:=rtu \
  serial_device:=/dev/ttyUSB0 \
  baud_rate:=1000000 rtu_parity:=even slave_id:=65
```

RTU uses the same hardware and controller contract as TCP. Only one Modbus
master should use a serial bus at a time.
See [USB / Modbus RTU setup](doc/USB_RTU.md) for device permissions,
USB latency settings and a complete realtime launch example.
For feedback, timing or visualization failures, see
[Record a useful bug report](doc/REPORTING_ISSUES.md).

Set `start_realtime_controller:=true` for any supported model. It starts the
resource-claimed controller and leaves the conventional action controller
inactive. The RViz panel sends realtime task-aperture targets and streams
velocity while the joystick is held. For RG2/RG6, velocity is mechanism angular
velocity in rad/s while state retains compensated task aperture in metres and
direct mechanism angle in radians. For 2FG, position and velocity both use the
task-aperture coordinate.

`realtime_update_rate_hz` controls the Modbus exchange period. `auto` selects
the conservative 50 Hz default for every supported model. Use an explicit
higher value only after measuring the target host, network, and gripper under
the intended workload. Confirm that sample age, cycle duration,
missed-deadline, failed-cycle, reconnect, and watchdog counters remain stable
at the selected rate. Firmware protocol limits are not operating-rate
guarantees.

For a measured higher-rate run, set all three loop rates together:

```bash
ros2 launch onrobot_gripper_bringup gripper.launch.py \
  model:=2fg7 backend:=real transport:=rtu serial_device:=/dev/ttyUSB0 \
  baud_rate:=1000000 rtu_parity:=even slave_id:=65 \
  start_realtime_controller:=true \
  realtime_update_rate_hz:=200 \
  controller_manager_update_rate_hz:=200 \
  state_publish_rate_hz:=200
```

`realtime_update_rate_hz` is the device exchange rate,
`controller_manager_update_rate_hz` is the ros2_control update rate, and
`state_publish_rate_hz` is the typed/realtime state publication rate. The
default controller-manager and state-publication rates are 100 Hz, while the
default hardware exchange rate remains 50 Hz. A publication can therefore
repeat the latest device sample; use `sample_sequence` and `sample_age` when
freshness matters.
The explicit values are not performance guarantees; select them only after
checking the observed transaction rate, state freshness, cycle duration, and
health counters on the target host.

The 2FG `supply_power_w` argument defaults to 48 W. The hardware session writes
and confirms this value after every connection and reconnect because the
firmware resets the setting to 14 W at power-up. Select a value from 14 to
48 W to impose a lower installation budget. Set it to zero only to preserve a
value managed outside this driver. The setting limits force holding, not the
realtime travel-speed command.

Every control launch also activates `gripper_state_broadcaster`. It publishes
the same model-neutral `onrobot_gripper_msgs/msg/GripperState` contract on
`/gripper_state_broadcaster/state` in conventional and realtime modes, plus a
standard `diagnostic_msgs/msg/DiagnosticArray` entry on `/diagnostics`.
Task-aperture, mechanism, velocity, and force values have independent validity
flags; an unavailable value is published as zero with its validity flag false.
The message identifies the configured standard finger profile and mapping
source. Firmware is reported as unavailable when the device interface does not
provide firmware identity.

Transient invalid samples and reconnect periods no longer deactivate the
hardware component. The session worker retains ownership of retry/fault policy,
while the state broadcaster remains active so connection state, sample age,
counters, reconnect count, and fault codes stay observable. A latched session
fault never clears merely because Stop was requested: recovery reconnects and
validates product identity on the worker, remains faulted on failure, and
returns active-idle without replaying the previous command on success.
For visualization, the hardware adapter may hold the last finite physical finger pose
while a transient sample is invalid; its validity flags and unavailable values
remain explicit, and a reset or device fault clears that pose. This does not
replace unavailable task-position feedback used by action controllers.

The always-active `recovery_controller` exposes that explicit action without
claiming the position or realtime motion resources:

```bash
ros2 service call /recovery_controller/recover std_srvs/srv/Trigger '{}'
```

The service response means the request was queued, not that reconnection has
already succeeded. Use `/gripper_state_broadcaster/state` or `/diagnostics` to
observe the connection transition, fault code, and reconnect counter.
The typed state reports `CONNECTION_RECOVERING` while the worker is performing
the reconnect and returns to `CONNECTION_FAULTED` if any recovery step fails.

The fake backend supports deterministic test-only motion rate and stall
injection. The standard action reports cancellation, preemption, reached-goal,
and contact-stall outcomes using normal ROS action semantics.

## Isaac Sim backend

The Isaac backend supports the versioned 2FG7, 2FG14, RG2, and RG6 assets
through the same ROS contract. Select the model whose USD bridge is running:

```bash
ros2 launch onrobot_gripper_bringup gripper.launch.py \
  model:=2fg7 backend:=isaac
```

To add RViz, launch the same backend through `onrobot_gripper_demos`.

Start `run_ros_bridge.py --model 2fg7` from the Isaac Sim installation first,
substituting the same model name in both processes. Each passing report records
the asset, runner, configuration, and Isaac Sim version actually exercised.
The model-neutral launch automatically enables simulation time. Conventional
control uses the same standard action; adding
`start_realtime_controller:=true` selects the same realtime position and
velocity controller used by other backends. The adapter maps semantic
`grip_stroke` commands to the selected USD's driven physical joint and maps its
state back to task aperture and raw mechanism state. Use the repeatable checks
in `onrobot_gripper_isaac`; force, contact, and payload results apply only to
the exact asset, fixture, and runtime recorded in a passing report.

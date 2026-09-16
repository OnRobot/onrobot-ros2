# OnRobot gripper showcases

This package provides one launch command for connecting to an OnRobot gripper
and opening or closing it from an RViz panel. See the repository
`GETTING_STARTED.md` for the complete safety, build, and operating procedure.

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 host:=192.168.1.1 port:=502
```

The launcher accepts model values `2fg7`, `2fg14`, `rg2`, `rg6`, `3fg15`, and
`3fg25`. The 2FG and RG models support `control:=conventional` and
`control:=realtime`. Three-finger conventional native external-diameter control
is available for evaluation only and requires the physical
`fingertip_position` setting.
The showcase opens RViz with the matching control panel already configured.
Realtime layouts provide aperture Open, Close, and target controls together
with the model-aware velocity joystick.

For 2FG conventional motion, select the device speed percentage at launch;
the default is 50% and valid values are 1–100. This is a native device
percentage, not an SI velocity guarantee:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 backend:=fake control:=conventional \
  conventional_speed_percent:=25 start_rviz:=true
```

If a lower-specification graphics system needs a lighter 2FG7 body visual, add
`visual_detail:=low`. This changes only the 2FG7 body visual mesh. Origins,
finger and fingertip visuals, collision meshes, joints, and control behavior
are unchanged. The default is `visual_detail:=detailed`.

The package only composes existing drivers, controllers, descriptions, and
RViz panels. It does not access a gripper outside `ros2_control`.

## Record and replay a session

For a bug investigation, use the
[diagnostic recording guide](../onrobot_gripper_bringup/doc/REPORTING_ISSUES.md).
It also captures raw feedback and command topics, so it is separate from the
state-only playback workflow below.

The showcase can record a state-only rosbag while you use either control panel.
The recorder saves joint states, typed gripper state, realtime state, live
limits, and diagnostics. It never records command, action, or service topics,
so the recording cannot command a gripper when it is replayed.
The accompanying `session.json` records the exact `robot_description` from the
running namespaced `robot_state_publisher`, its TF frame prefix and base
transform, firmware/profile identity observed at startup, and hashes for the
ROS source, asset layers, and Tool API library when those identities are
available. An unavailable identity is recorded as unknown; it is never guessed.
It also contains compact per-topic hashes of the serialized recorded state, so
replay refuses a bag whose saved state streams no longer match the session.
The recorder writes a `qos_profile_overrides.yaml` sidecar and passes it to
rosbag2. State topics are recorded with a best-effort, volatile sensor-data
subscription so a filtered `/joint_states` publisher is not silently omitted
when recording starts before or after the showcase. This changes only the
recorder's read-side QoS; it does not change the live publishers or replay
allowlist.
Use `--description-file` only when recording from an existing exact expanded
URDF instead of a running showcase.

Start the showcase in one terminal, then record from another terminal in the
same sourced workspace:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 host:=192.168.1.1 port:=502 \
  control:=conventional start_rviz:=true

ros2 run onrobot_gripper_demos record_gripper_session \
  --model 2fg7 --backend real --duration 30 \
  --output-dir recordings/2fg7-session
```

Use the RViz controls during the recording. A new output directory is required
for every recording. Stop the showcase after the recorder prints its completed
JSON report. For an open-ended recording, omit `--duration` and press
`Ctrl-C` in the recorder terminal; the recorder will finish the bag and write
the same report.

Replay the saved session without a connected gripper:

```bash
ros2 run onrobot_gripper_demos replay_gripper_session \
  recordings/2fg7-session
```

Replay opens a temporary RViz layout containing the robot model and the
read-only **Force & State History** panel. It plays the saved joint and state
topics with the recorded ROS clock; it does not start `ros2_control` and does
not include an actionable control panel. Useful presentation options are:

```bash
# Half speed, with the session repeating until Ctrl-C.
ros2 run onrobot_gripper_demos replay_gripper_session \
  recordings/2fg7-session --rate 0.5 --loop

# Validate and play the bag without opening RViz.
ros2 run onrobot_gripper_demos replay_gripper_session \
  recordings/2fg7-session --no-rviz
```

The history panel plots signed measured force when the selected model provides
that measurement (`2fg7` and `2fg14`). Invalid or stale samples are shown as
gaps, grip-detected periods are shaded, and changes in connection, mode, or
fault state are marked on the timeline. RG models still show their typed state
and events, but their requested/command-derived force is not presented as a
measured-force trace.

## Measure a realtime RTU path

The measurement command checks the complete path from a ROS command through
`ros2_control` and Modbus RTU to fresh measured state. It also checks automatic
Stop on stale input, explicit Stop, and a conventional-to-realtime controller
round trip. The gripper moves within the middle 20% of its live aperture range.
Clear and secure the fixture before running it.

Start the showcase with matching device, controller, and state rates:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 backend:=real control:=realtime start_rviz:=true \
  transport:=rtu serial_device:=/dev/ttyUSB0 \
  baud_rate:=1000000 rtu_parity:=even slave_id:=65 \
  realtime_update_rate_hz:=200 \
  controller_manager_update_rate_hz:=200 \
  state_publish_rate_hz:=200
```

In another sourced terminal, run:

```bash
ros2 run onrobot_gripper_demos measure_realtime_ros_path \
  --model 2fg7 \
  --serial-device /dev/ttyUSB0 \
  --baud-rate 1000000 --slave-id 65 --rate-hz 200 \
  --duration 60 --output realtime-rtu-2fg7-200hz.json
```

The command refuses TCP, fake hardware, a different model, or different serial
settings. A passing JSON report contains the measured device-cycle rate, fresh
state timing, health counters, observed travel, Stop result, and controller
states. Run only one Modbus master on the serial bus.

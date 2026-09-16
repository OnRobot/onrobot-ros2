# Session recording and replay

The demo package can record a state-only rosbag while a conventional or
realtime showcase is running. It records joint state, typed gripper state,
realtime state, live limits, and diagnostics. It never records command, action,
or service topics, so replay cannot command a gripper.

```bash
ros2 run onrobot_gripper_demos record_gripper_session \
  --model 2fg7 --backend real --duration 30 \
  --output-dir recordings/2fg7-session
```

The adjacent `session.json` records the expanded robot description, namespace
and TF configuration, observed device/profile identity, available source and
asset identities, and per-topic hashes of recorded state. Unknown identities
remain explicitly unknown.

The recorder also stores `qos_profile_overrides.yaml` beside the bag. It uses
a best-effort, volatile sensor-data subscription for each state topic, which
keeps filtered `/joint_states` samples when the publisher offers sensor-data
QoS. This is a recorder-side setting; it does not alter the running driver or
the command-free replay contract.

Stop hardware bringup, then replay without a connected gripper:

```bash
ros2 run onrobot_gripper_demos replay_gripper_session \
  recordings/2fg7-session
```

Replay uses the recorded ROS clock and opens a read-only RViz layout. It does
not start `ros2_control` or an actionable control panel. Useful options include
`--rate 0.5`, `--loop`, and `--no-rviz`.

The Force & State History panel plots valid, fresh signed force for 2FG7 and
2FG14, shades grip-detected periods, and marks connection, mode, and fault
changes. RG sessions retain state and event history, but command-derived RG
force is not presented as measured contact force.

For namespace, TF and provenance options, see the
[recording reference](../onrobot_gripper_demos/README.md#record-and-replay-a-session).
For investigating commands or faults, use the separate
[diagnostic recording guide](../onrobot_gripper_bringup/doc/REPORTING_ISSUES.md);
its bag is not suitable for command-free demonstration replay.

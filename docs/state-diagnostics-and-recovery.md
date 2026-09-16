# State, diagnostics, and recovery

Every control launch publishes:

- `/joint_states` for visualization and standard consumers;
- `/gripper_state_broadcaster/state` for semantic task, mechanism, connection,
  validity, safety, and fault state; and
- `/diagnostics`, or the namespace-resolved configured `diagnostics_topic`,
  for standard ROS diagnostics.

| Diagnostic group | Examples |
|---|---|
| Identity | Model, device profile, finger profile, firmware when available |
| Connection and freshness | Connection state, sample age, sample sequence |
| Fault and safety | Fault source/code/message and valid RG safety state |
| Health | Successful/failed cycles, missed deadlines, watchdog stops, reconnects |
| Device registers | Supported model-specific values with units in their keys |

Unsupported values are omitted from diagnostics. Use typed-state validity
fields before consuming numeric observations. 2FG force feedback is signed in
newtons: closing contact is negative and opening contact is positive. RG force
is command-derived, while 3FG force is a percentage rather than newtons.

Measurements have independent validity fields. A numeric value whose validity
field is false is a placeholder, not a measurement. In particular,
`safety_status_valid=false` does not mean an RG safety switch is clear.

Connection failures remain observable while the driver attempts its bounded
policy. The 2FG/RG bringup provides explicit recovery for latched faults:

```bash
ros2 service call /recovery_controller/recover std_srvs/srv/Trigger '{}'
```

The service response means the request was queued. Observe semantic state or
diagnostics for completion. Successful recovery returns to an idle state and
does not replay the previous command.

The 3FG evaluation launch does not provide this recovery service.

RG recovery is rejected while a safety switch remains pushed or a safety DC
error is reported. Clear the work area before releasing a switch. A safety DC
error requires a physical power cycle; the driver does not use a firmware
update reset mechanism as an operational recovery command.

These interfaces supplement the installation's safety system and are not
safety-rated stopping or monitoring functions.

For the exact validity/identity contract and a live read example, see the
[message package](../onrobot_gripper_msgs/README.md#diagnostics). For a bug report,
[record raw diagnostics](../onrobot_gripper_bringup/doc/REPORTING_ISSUES.md)
instead of using the state-only replay recorder.

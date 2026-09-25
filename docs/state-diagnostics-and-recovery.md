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
| Device telemetry | Supported model-specific values with units in their keys |

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

The service accepts a request only while the recovery controller is active,
the device state is available and faulted, and (for RG grippers) the safety
state permits recovery. Only one request can be unresolved at a time. A
successful service response means the controller queued the request; it does
not mean the hardware accepted it or that recovery completed. Observe the
matching request on `/recovery_controller/state`:

```bash
ros2 topic echo /recovery_controller/state
```

`admission_sequence` and `admission` identify backend acceptance or rejection.
`active_sequence` identifies an operation in progress. `result_sequence`,
`result` and `result_code` retain the last completed operation; match the
sequence, not reconnect or health counters. Results are `1` (succeeded), `2`
(failed), or `3` (aborted during shutdown); `0` means no terminal result.
Check `observation_valid` before using these values. Missing observations or a
client timeout do not cancel an operation. Use the semantic state and
`/diagnostics` for the device's current condition. Successful recovery returns the
device to active idle without replaying the previous motion. A backend
rejection is not reported as recovery completion, and the controller does not
retry it automatically. A failed recovery remains faulted. Check the current
device state, address the reported cause, and submit a fresh request if the
device is still faulted.

If RG safety changes before export, `cancelled_sequence` identifies the locally
cancelled request. Make the work area safe and submit a fresh request; an already
exported request is resolved only by the backend.

The controller retains a queued request if its command interface is temporarily
unavailable. Deactivation does not discard an unresolved request, and cleanup
or unload is refused until that request reaches a terminal result. Reactivate
the controller to resume observation. Calls made while it is inactive are
rejected, and a completed request is not replayed after unload/reload.

The 3FG evaluation launch does not provide this recovery service.

Fake and Isaac recovery clears a simulated logical fault and establishes an
idle hold without replaying prior motion. Isaac requires current, usable
feedback when the request is consumed. Simulation does not model physical RG
safety switches or prove that a physical gripper has stopped. Recovery requires
synchronous hardware and controller updates; asynchronous mode is rejected.

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

# State, diagnostics, and recovery

Every control launch publishes:

- `/joint_states` for visualization and other ROS applications;
- `/gripper_state_broadcaster/state` for aperture, mechanism, connection,
  validity, safety, and fault state; and
- `/diagnostics`, or the namespace-resolved configured `diagnostics_topic`,
  for standard ROS diagnostics.

| Diagnostic group | Examples |
|---|---|
| Identity | Model, device profile, finger profile, firmware when available |
| Connection and sample age | Connection state, sample age, sample sequence |
| Fault and safety | Fault source/code/message and valid RG safety state |
| Health | Successful/failed cycles, missed deadlines, watchdog stops, reconnects |
| Device telemetry | Supported model-specific values with units in their keys |

Unsupported values are omitted from diagnostics. Check validity flags in the
typed state message before using its values. 2FG force feedback is signed in
newtons: closing contact is negative and opening contact is positive. RG force
is command-derived, while 3FG force is a percentage rather than newtons.

Measurements have independent validity fields. A numeric value whose validity
field is false is a placeholder, not a measurement. In particular,
`safety_status_valid=false` does not mean an RG safety switch is clear.

Check the reported fault when a connection fails. A latched fault remains
active until recovery succeeds. The 2FG/RG launch provides a recovery service:

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
Check `observation_valid` before using these values. Missing status updates or a
client timeout do not cancel an operation. Use the semantic state and
`/diagnostics` for the device's current condition. After successful recovery,
the device is ready for new commands and does not repeat the previous motion. A backend
rejection is not reported as recovery completion, and the controller does not
retry it automatically. A failed recovery remains faulted. Check the current
device state, address the reported cause, and submit a fresh request if the
device is still faulted.

If RG safety state changes before the controller passes the request to the
backend, `cancelled_sequence` identifies the cancelled request. Make the work
area safe and submit a new request. Once a request has reached the backend,
only the backend can report its outcome.

The controller retains a queued request if its command interface is temporarily
unavailable. Deactivation does not discard an unresolved request, and cleanup
or unload is refused until that request has a final result. Reactivate
the controller to resume observation. Calls made while it is inactive are
rejected, and a completed request is not replayed after unload/reload.

The 3FG evaluation launch does not provide this recovery service.

Fake and Isaac recovery clears the simulated fault and holds position without
repeating earlier motion. Isaac requires current, usable feedback when it
processes the request. Simulation does not model physical RG
safety switches or prove that a physical gripper has stopped. Recovery requires
synchronous hardware and controller updates; asynchronous mode is rejected.

RG recovery is rejected while a safety switch remains pushed or a safety DC
error is reported. Clear the work area before releasing a switch. A safety DC
error requires the gripper to be powered off and on after making the equipment safe.

These interfaces supplement the installation's safety system and are not
safety-rated stopping or monitoring functions.

For the validity and identity fields and a live read example, see the
[message package](../onrobot_gripper_msgs/README.md#diagnostics). For a bug report,
[record raw diagnostics](../onrobot_gripper_bringup/doc/REPORTING_ISSUES.md)
instead of using the state-only replay recorder.

# Commission a physical gripper

Before motion, secure the gripper and workpiece, clear the full travel and
provide an independent stop. Confirm the model, firmware, physical finger
mounting and connection. Software Stop and watchdogs are not safety-rated.

1. Complete [installation](installation.md) and the fake-backend check.
2. Stop other showcases or Modbus clients; one hardware session owns the device.
3. Start the [TCP or RTU showcase](../GETTING_STARTED.md#3-start-the-showcase).
   The Compute Box default is `192.168.1.1:502`. USB examples use `/dev/ttyUSB0`;
   verify the actual adapter and follow the [USB guide](../onrobot_gripper_bringup/doc/USB_RTU.md).
4. Observe state before commanding anything:

```bash
ros2 control list_controllers
ros2 topic echo --once --qos-reliability best_effort /gripper_state_broadcaster/state
ros2 topic echo --once /diagnostics
```

Confirm live, valid measurements and limits. Add the gripper namespace to topic
names if configured. A configured firmware label is not proof of the connected
firmware; use the reported identity and capability state.

For 3FG evaluation setups, `fingertip_position` must match the physical
mounting. The 3FG ROS path is available for evaluation only.
For all models, check that the reported task limits and contact-surface gap
match your fingers. The current generic launch does not configure alternative
finger geometry or overwrite finger settings on the device.

Choose a conservative intermediate target first while observing the tool
directly. Try the endpoints only after confirming the travel is clear. If motion
or feedback differs from expectations, stop and investigate before retrying.
Follow [state and recovery](state-diagnostics-and-recovery.md) for latched faults.

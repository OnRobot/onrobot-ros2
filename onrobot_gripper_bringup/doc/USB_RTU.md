# Connect a gripper over USB / Modbus RTU

Use this guide for 2FG7, 2FG14, RG2 or RG6 on Ubuntu 24.04 with the
Tool API and ROS packages already installed and sourced. Use a supported
USB-to-RS-485 adapter, the correct tool-bus wiring and external gripper power;
a USB-to-RS-232 or TTL cable is not interchangeable.

## 1. Select the device and settings

List the adapters, then replace the example path with the actual entry:

```bash
ls -l /dev/serial/by-id/
export ONROBOT_SERIAL_DEVICE=/dev/ttyUSB0
export ONROBOT_MODEL=2fg7
ls -l "$(readlink -f "$ONROBOT_SERIAL_DEVICE")"
id -nG
```

Use `2fg14`, `rg2` or `rg6` for the other models. `/dev/ttyUSB0` is an example;
verify it belongs to the intended adapter. A stable `/dev/serial/by-id/` path avoids
selecting a different adapter after reconnecting. The ROS user needs read/write
permission on that device. On a standard Ubuntu installation this is usually
granted through the `dialout` group; ask your administrator to add the user if
needed, then log out and back in. Do not run ROS as root or grant every user
access to all serial devices.

| Setting | Value |
|---|---|
| Framing | 8 data bits, even parity, 1 stop bit (`rtu_parity:=even`) |
| Baud rate | `115200` or `1000000`, matching the configured gripper |
| Slave ID | `65` single mount; `66` primary or `67` secondary dual-mount position |

Changing a host launch argument does not change the gripper's baud rate or
address. Close other drivers, serial monitors and direct Tool API utilities
before launching: **one Modbus master owns the serial bus**. An Isaac shadow
uses ROS feedback, not another serial connection.

## 2. Start with conventional control

Secure the gripper and clear its travel before using motion controls:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:="$ONROBOT_MODEL" backend:=real transport:=rtu \
  serial_device:="$ONROBOT_SERIAL_DEVICE" \
  baud_rate:=1000000 rtu_parity:=even slave_id:=65 \
  control:=conventional start_rviz:=true
```

Use `baud_rate:=115200` instead if that is the device setting. Wait for live
state and limits, then use a small intermediate target before testing full
travel. Stop this launch before changing modes or starting another driver.
For 2FG models, the driver normally reapplies a 48 W supply budget on connection;
see the bringup README for `supply_power_w` and lower installation budgets.

## 3. Inspect USB latency before increasing the rate

Some USB-serial drivers buffer short replies. For adapters that expose a
`latency_timer`, inspect it on the **computer connected to the USB adapter**:

```bash
ONROBOT_TTY="$(basename -- "$(readlink -f "$ONROBOT_SERIAL_DEVICE")")"
ONROBOT_LATENCY="/sys/bus/usb-serial/devices/$ONROBOT_TTY/latency_timer"
if [ -r "$ONROBOT_LATENCY" ]; then
  cat "$ONROBOT_LATENCY"
else
  printf 'This driver does not expose a USB-serial latency timer.\n'
fi
```

The timer is in milliseconds. A 16 ms receive delay can limit short
request/reply exchanges to around 62 Hz even at 1 Mbit/s. With the driver
stopped, an administrator can try 1 ms **only on the selected adapter**:

```bash
if [ -f "$ONROBOT_LATENCY" ]; then
  printf '1\n' | sudo tee "$ONROBOT_LATENCY"
  cat "$ONROBOT_LATENCY"
fi
```

Record the original value so it can be restored using the same command.
Do not create a missing timer or apply this setting to unrelated devices.
The setting may reset on unplug or reboot. If persistence is needed, have an
administrator create a udev rule matched to this adapter's actual vendor,
product and unique serial identifiers; verify the value after reconnecting.
Lower latency improves responsiveness but does not guarantee a particular rate.

## 4. Try realtime control and check fresh feedback

Realtime position and velocity require compatible realtime firmware. A firmware
compatibility error is not a baud-rate or latency problem; do not bypass it.
At 1 Mbit/s, this example requests 200 Hz device exchanges, controller updates
and state publication. It does not start automatic movement:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:="$ONROBOT_MODEL" backend:=real transport:=rtu \
  serial_device:="$ONROBOT_SERIAL_DEVICE" \
  baud_rate:=1000000 rtu_parity:=even slave_id:=65 \
  control:=realtime start_rviz:=true \
  realtime_update_rate_hz:=200 \
  controller_manager_update_rate_hz:=200 state_publish_rate_hz:=200
```

In another sourced terminal:

```bash
ros2 control list_controllers
ros2 topic echo --once --qos-reliability best_effort /gripper_state_broadcaster/state
ros2 topic echo --once /diagnostics
```

Expect the realtime controller to be active and the conventional controller
inactive. Observe increasing `successful_cycles` and `sample_sequence`, bounded
`sample_age` and `last_cycle_duration`, and stable failure, reconnect, watchdog
and missed-deadline counters during operation. Publication may repeat a
device sample: `ros2 topic hz` alone does not measure fresh device data.
If ROS scheduling requires headroom, 250 Hz controller and publication rates
can be tried while keeping the device at 200 Hz; repeated samples are expected.
The normal defaults are 50 Hz device exchange and 100 Hz ROS update/publication.
Use those defaults if the higher-rate configuration is not stable under your
workload. Do not interpret a 500 Hz setting as verified hard-realtime behavior.

For a headless application, use `onrobot_gripper_bringup gripper.launch.py`
with the same transport/rate arguments, replace `control:=realtime` with
`start_realtime_controller:=true`, and omit `start_rviz`.

## If connection or motion fails

- **Permission denied:** check the actual device permissions and current group
  membership; a new login may be required.
- **Timeout or CRC errors:** check power, RS-485 wiring/direction handling,
  baud rate, parity, address and whether another master is using the bus.
- **Conventional works, realtime is rejected:** check the installed Tool API
  and the gripper's realtime firmware compatibility.
- **State is slow or stale:** check USB latency and health counters; reduce the
  requested rates before changing timeout settings. Increasing timeouts does
  not make the device faster.

# Launch parameters

## Interactive showcase

`onrobot_gripper_demos/showcase.launch.py` is the recommended user entry point.

| Parameter | Default | Allowed/applicable values | Required when | Description |
|---|---|---|---|---|
| `model` | `2fg7` | `2fg7`, `2fg14`, `rg2`, `rg6`, `3fg15`, `3fg25` | Always | Select product; 3FG path is evaluation only |
| `control` | `conventional` | `conventional`, `realtime` | Always | 3FG accepts conventional only |
| `backend` | `real` | `real`, `fake`, `isaac` | Always | 3FG accepts real only |
| `host` | `127.0.0.1` | Address/hostname | Real backend | Modbus TCP endpoint |
| `port` | `502` | Configured TCP port | Real backend | Modbus TCP port |
| `transport` | `tcp` | `tcp`, `rtu` | Real parallel grippers | Select Modbus TCP or direct serial RTU |
| `serial_device` | `/dev/ttyUSB0` | Device path | RTU | Serial adapter; prefer `/dev/serial/by-id/...` |
| `baud_rate` | `1000000` | `115200`, `1000000` | RTU | Serial baud rate |
| `rtu_parity` | `even` | `even` | RTU | Fixed OnRobot 8E1 framing |
| `slave_id` | `65` | `65`, `66`, `67` | RTU | Modbus address matching the mounting position |
| `connect_timeout_ms` | `1000` | Positive ms | Real backend | Connection timeout |
| `read_timeout_ms` | `1000` | Positive ms | Real backend | Read-response timeout |
| `write_timeout_ms` | `1000` | Positive ms | Real backend | Write-response timeout |
| `realtime_update_rate_hz` | `auto` | Positive rate or `auto` | Parallel grippers | `auto` selects a conservative 50 Hz hardware exchange rate |
| `controller_manager_update_rate_hz` | `100` | Positive Hz | Parallel grippers | `ros2_control` update rate |
| `state_publish_rate_hz` | `100` | Positive Hz | Parallel grippers | Typed and realtime state publication rate |
| `gripper_stall_timeout_s` | `2.0` | Positive seconds | Conventional control | Low-velocity duration before contact stall |
| `start_rviz` | `true` | `true`, `false` | No | Start the configured RViz view |
| `namespace` | empty | ROS namespace | Multiple instances | Scope controller interfaces |
| `frame_prefix` | empty | TF-safe prefix | Multiple instances | Prevent TF name collisions |
| `fingertip_position` | `auto` | `auto`, `1`, `2`, `3` | 3FG hardware | Must match physical mounting; auto uses the model reference |

Both `showcase.launch.py` and `three_finger_showcase.launch.py` use fingertip
position 2 for 3FG15 and position 3 for 3FG25 when `fingertip_position:=auto`.
An explicit position must match the physical mounting and device configuration.
The launch checks that match; it does not write a different mounting to the device.
3FG support is limited to conventional real-TCP evaluation; the supported
release set is 2FG7, 2FG14, RG2, and RG6.

The 3FG showcase currently forwards TCP `host`/`port`, not serial arguments.
For 3FG, do not use the parallel-gripper RTU or rate examples.

The showcase also accepts the parallel-gripper `supply_power_w`,
`conventional_speed_percent`, `fake_motion_speed_m_s`, `fake_stall`,
`visual_detail`,
`isaac_joint_commands_topic`,
`isaac_joint_states_topic`, `isaac_state_timeout_ms` and `use_sim_time`
options described below. Inspect the installed entry point with:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py --show-args
```

For controllers that advertise conventional speed support, the RViz
Conventional Control panel can update `conventional_speed_percent` at runtime.
The launch argument supplies the initial value; runtime changes apply to future
goals only. See the [speed panel guide](../realtime-control.md#conventional-2fg-speed-panel).

## Headless parallel-gripper bringup

`onrobot_gripper_bringup/gripper.launch.py` accepts:

| Parameter | Default | Allowed/applicable values | Description |
|---|---|---|---|
| `model` | `2fg7` | `2fg7`, `2fg14`, `rg2`, `rg6` | Select product |
| `backend` | `real` | `real`, `fake`, `isaac` | Select device backend |
| `host` | `127.0.0.1` | Address/hostname | Real Modbus TCP endpoint |
| `port` | `502` | Configured TCP port | Real Modbus TCP port |
| `transport` | `tcp` | `tcp`, `rtu` | Select real-hardware transport |
| `serial_device` | `/dev/ttyUSB0` | Device path | RTU serial adapter |
| `baud_rate` | `1000000` | `115200`, `1000000` | RTU baud rate |
| `rtu_parity` | `even` | `even` | OnRobot 8E1 framing |
| `slave_id` | `65` | `65`, `66`, `67` | RTU device address |
| `connect_timeout_ms` | `1000` | Positive ms | Connection timeout |
| `read_timeout_ms` | `1000` | Positive ms | Read-response timeout |
| `write_timeout_ms` | `1000` | Positive ms | Write-response timeout |
| `namespace` | empty | ROS namespace | Scope one gripper instance |
| `controller_manager` | empty | Service name | Empty selects the namespace-specific controller manager |
| `frame_prefix` | empty | TF-safe prefix | Prefix every published TF frame |
| `publish_base_frame` | `true` | `true`, `false` | Publish a fixed parent transform |
| `base_parent_frame` | `world` | TF frame | Parent used when base publication is enabled |
| `start_realtime_controller` | `false` | Boolean | Activate realtime instead of conventional motion ownership |
| `realtime_update_rate_hz` | `auto` | Positive rate or `auto` | `auto`: conservative 50 Hz hardware exchange rate |
| `controller_manager_update_rate_hz` | `100` | Positive Hz | `ros2_control` update rate |
| `state_publish_rate_hz` | `100` | Positive Hz | Typed and realtime state publication rate |
| `gripper_stall_timeout_s` | `2.0` | Positive seconds | Conventional action stall interval |
| `supply_power_w` | `48` | `0` or `14`–`48` | 2FG only; zero preserves the device setting |
| `conventional_speed_percent` | `50` | `1`–`100` | 2FG only; initial native conventional speed; the panel can update it at runtime on supported backends |
| `visual_detail` | `detailed` | `detailed`, `low` | 2FG7 only; selects the body visual mesh without changing collisions or kinematics |
| `fake_motion_speed_m_s` | `0.0` | Nonnegative m/s | 2FG7 fake-test motion rate; zero is instantaneous |
| `fake_stall` | `0` | `0`, `1` | 2FG7 deterministic fake-test stall |
| `isaac_joint_commands_topic` | `isaac_joint_commands` | Topic name | Isaac command topic |
| `isaac_joint_states_topic` | `isaac_joint_states` | Topic name | Isaac feedback topic |
| `isaac_state_timeout_ms` | `250` | Positive ms | Maximum accepted Isaac feedback age |
| `use_sim_time` | `false` | Boolean | Automatically enabled for Isaac |

## MoveIt example

| Parameter | Default | Allowed values | Description |
|---|---|---|---|
| `model` | `2fg7` | `2fg7`, `2fg14`, `rg2`, `rg6` | Select product |
| `backend` | `fake` | `fake`, `real` | Select execution backend |
| `host` | `127.0.0.1` | Address/hostname | Real Modbus TCP endpoint |
| `port` | `502` | Configured TCP port | Real Modbus TCP port |

Launch arguments configure composition. RViz panel settings such as
`action_name`, `joint_states_topic`, `limits_topic`, `gripper_state_topic`,
`controller_manager_service`, `realtime_command_topic`, and
`realtime_state_topic` configure where a panel connects; they are saved in the
RViz configuration and are not launch arguments of `showcase.launch.py`.
The Three-Finger Control panel additionally saves
`three_finger_command_topic` and `three_finger_default_force_percent`; its
relative defaults are `diameter_controller/commands`, `joint_states`,
`three_finger_limit_broadcaster/values`, and
`gripper_state_broadcaster/state`. Use the panel's `OnRobot panel settings`
entry to route a panel in a namespaced or remapped 3FG setup.

For a complete 200 Hz USB example, see
[realtime setup](../../GETTING_STARTED.md#5-use-realtime-position-and-velocity-control).
The default TCP host is loopback, not the Compute Box factory address;
use `host:=192.168.1.1` unless its address was changed.

# Message fields and states

## `GripperState`

Identity and timing:

| Field | Type | Unit/values | Meaning |
|---|---|---|---|
| `header` | `std_msgs/Header` | ROS time/frame | Publication metadata |
| `model` | `string` | Model name | Configured product |
| `firmware` | `string` | Version or empty | Reported firmware identity |
| `device_profile_revision` | `string` | Revision | Device mapping revision |
| `finger_profile_name` | `string` | Profile name | Selected finger configuration |
| `finger_profile_revision` | `string` | Revision | Finger-profile revision |
| `sample_sequence` | `uint64` | Counter | Physical I/O sample sequence |
| `received_time` | `builtin_interfaces/Time` | ROS time | Host receive time of the sample |
| `sample_age` | `builtin_interfaces/Duration` | Duration | Monotonic age at publication |

Measurements and validity:

| Field | Type | Unit/values | Meaning |
|---|---|---|---|
| `task_aperture_valid` | `bool` | Boolean | Whether `task_aperture` is a measurement |
| `task_aperture` | `float64` | m | Finger-dependent aperture or external diameter |
| `task_velocity_valid` | `bool` | Boolean | Whether `task_velocity` is a measurement |
| `task_velocity` | `float64` | m/s | Task-coordinate velocity |
| `mechanism_dimension` | `uint8` | Dimension enum | None, linear, or angular |
| `mechanism_position_valid` | `bool` | Boolean | Whether mechanism position is valid |
| `mechanism_position` | `float64` | m or rad | Raw mechanism coordinate |
| `mechanism_velocity_valid` | `bool` | Boolean | Whether mechanism velocity is valid |
| `mechanism_velocity` | `float64` | m/s or rad/s | Raw mechanism velocity |
| `force_valid` | `bool` | Boolean | Whether `force` is a measurement |
| `force` | `float64` | N | Measured gripping force when available |
| `busy` | `bool` | Boolean | Device reports active operation |
| `grip_detected` | `bool` | Boolean | Device reports a detected grip |
| `realtime_force_control_available` | `bool` | Boolean | Backend accepts 2FG closing-force approaches; not an accuracy claim |

RG safety state:

| Field | Meaning |
|---|---|
| `safety_status_valid` | Whether all following safety fields are available |
| `safety_1_pushed` / `safety_2_pushed` | Current physical switch state |
| `safety_1_triggered` / `safety_2_triggered` | Latched switch-trigger state |
| `safety_dc_error` | Safety DC error; physical power cycle required |

Session, fault, and counters:

| Field | Type | Meaning |
|---|---|---|
| `connection_state` | `uint8` | Connection-state enum |
| `active_mode` | `uint8` | Backend control mode |
| `firmware_qualification` | `uint8` | Firmware qualification enum |
| `mapping_validity` | `uint8` | Coordinate-mapping validity enum |
| `mapping_source` | `uint8` | Mapping-source enum |
| `fault_source` | `uint8` | Fault-source enum |
| `fault_code` / `fault_message` | `uint16` / `string` | Current fault identity and description |
| `successful_cycles` / `failed_cycles` | `uint64` | Physical I/O cycle counters |
| `missed_deadlines` | `uint64` | Worker deadline misses |
| `watchdog_stops` | `uint64` | Stops requested by the watchdog |
| `reconnects` | `uint64` | Successful reconnect count |
| `last_cycle_duration` | `float64` | Last physical I/O cycle duration, s |

| Enum | Values |
|---|---|
| Dimension | `DIMENSION_NONE=0`, `DIMENSION_LINEAR=1`, `DIMENSION_ANGULAR=2` |
| Connection | `UNKNOWN=0`, `DISCONNECTED=1`, `CONNECTING=2`, `IDLE=3`, `ACTIVE=4`, `RECOVERING=5`, `FAULTED=6` |
| Firmware | `UNKNOWN=0`, `QUALIFIED=1`, `UNSUPPORTED=2` |
| Mapping validity | `MISSING=0`, `VALID=1`, `INVALID=2` |
| Mapping source | `UNKNOWN=0`, `DEVICE_MEASURED=1`, `DEVICE_COMPENSATED=2`, `DECLARATIVE=3` |
| Fault source | `NONE=0`, `HOST=1`, `TRANSPORT=2`, `PROTOCOL=3`, `DEVICE=4` |

## `RealtimeCommand`

| Field | Type | Unit/values | Requirement |
|---|---|---|---|
| `header` | `std_msgs/Header` | ROS time | Motion needs a nonzero, fresh stamp |
| `mode` | `uint8` | `POSITION=0`, `VELOCITY=1`, `FORCE_POSITION=2`, `FORCE_VELOCITY=3`, `STOP=255` | Selects command semantics |
| `task_position` | `float64` | m | External task aperture for 2FG and RG |
| `task_velocity` | `float64` | m/s | 2FG approach limit or signed velocity |
| `mechanism_angular_velocity` | `float64` | rad/s | RG velocity command |
| `force` | `float64` | N | Positive closing-force target in 2FG force modes; RG position force limit |

All motion modes are streaming interfaces. Refresh commands before the receipt
timeout. Stop is a one-shot ordered command. Force-position and force-velocity
modes are supported on compatible real 2FG devices. Check
`GripperState.realtime_force_control_available`. Fake approach motion does not
produce contact or measured force; the Isaac joint bridge does not implement
force regulation. See the [force controls](../../onrobot_gripper_rviz_plugins/README.md#2fg-closing-force-grip)
for explicit hold, release and Stop behavior.

## `RealtimeState`

| Field group | Fields | Unit/meaning |
|---|---|---|
| Mode | `active_mode`, `realtime_active`, `faulted` | Idle/position/velocity/force mode and controller health |
| Linear mechanism | `mechanism_linear_position_valid`, `mechanism_linear_position`, `mechanism_linear_velocity_valid`, `mechanism_linear_velocity` | m and m/s; used by 2FG |
| Angular mechanism | `mechanism_angular_position_valid`, `mechanism_angular_position`, `mechanism_angular_velocity_valid`, `mechanism_angular_velocity` | rad and rad/s; used by RG |
| Task | `task_position_valid`, `task_position`, `task_velocity_valid`, `task_velocity` | m and m/s |
| Force | `force_valid`, `force` | N when available |
| Health | `successful_cycles`, `failed_cycles`, `missed_deadlines`, `watchdog_stops`, `requested_command_sequence`, `applied_command_sequence`, `reconnects`, `last_cycle_duration` | Worker/session counters and seconds |

`active_mode` values are `IDLE=0`, `POSITION=1`, `VELOCITY=2`,
`FORCE_POSITION=3`, and `FORCE_VELOCITY=4`. Numeric placeholders are not
measurements unless their corresponding validity flag is true.

For measured-force provenance, signed feedback and cached identity, see the
[message package](../../onrobot_gripper_msgs/README.md). The installed definitions
are also available with `ros2 interface show onrobot_gripper_msgs/msg/GripperState`.

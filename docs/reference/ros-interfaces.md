# ROS interface reference

Names below are defaults without a namespace. With `namespace:=left`, relative
controller interfaces and diagnostics resolve below `/left`. The state
broadcaster's `diagnostics_topic` parameter can select another remappable name.

## 2FG and RG interfaces

| Interface | Type | Direction | Available when | Purpose |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | Published | All control modes | Task and visualization joints |
| `/gripper_state_broadcaster/state` | `onrobot_gripper_msgs/msg/GripperState` | Published | All control modes | Semantic device, connection, validity, fault, and health state |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Published | All control modes | Standard ROS diagnostics |
| `/parallel_gripper_limit_broadcaster/values` | `control_msgs/msg/Float64Values` | Published | All control modes | Minimum and maximum task aperture in metres |
| `/gripper_controller/gripper_cmd` | `control_msgs/action/ParallelGripperCommand` | Action server | Conventional controller active | Standard aperture and maximum-effort command |
| `/realtime_controller/command` | `onrobot_gripper_msgs/msg/RealtimeCommand` | Subscribed | Realtime controller active | Streaming position, velocity, supported 2FG force approach, or Stop |
| `/realtime_controller/state` | `onrobot_gripper_msgs/msg/RealtimeState` | Published | Realtime controller active | Realtime mode, measurements, validity, and counters |
| `/realtime_controller/stop` | `std_srvs/srv/Trigger` | Service server | Realtime controller active | Ordered Stop request |
| `/recovery_controller/recover` | `std_srvs/srv/Trigger` | Service server | Bringup active | Queue explicit fault recovery |

`ParallelGripperCommand.command.position` is a one-element array of task
aperture in metres; `command.name` is `[grip_stroke]`.
`command.effort` is an optional one-element array of requested maximum effort in newtons. The backend
supplies valid aperture limits; applications must not infer them from the model
name. A nonempty `command.velocity` is rejected; velocity control belongs to
the realtime controller.

The conventional controller exposes `conventional_speed_control` as a fixed
capability flag and `conventional_speed_percent` as an integer
setting from 1–100. Runtime parameter updates are supported only when the
capability is true, affect future goals, and are rejected while motion or a
Stop handoff is active. The 2FG conventional RViz panel provides the same
setting and verifies the controller readback; see the
[speed panel guide](../realtime-control.md#conventional-2fg-speed-panel).

## Model-specific meaning

| Family | `grip_stroke` | Mechanism state | Realtime velocity | Additional state |
|---|---|---|---|---|
| 2FG | External task aperture, m | Linear position/velocity, m and m/s | Task-aperture velocity, m/s | Force when supplied by the backend |
| RG | Compensated external task aperture, m | Angular position/velocity, rad and rad/s | Mechanism angular velocity, rad/s | Two safety switches and safety DC error on hardware |

## 3FG evaluation interfaces

The 3FG ROS interfaces below are available for evaluation only.

| Interface | Type | Direction | Purpose |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | Published | `grip_diameter` and coupled `finger_angle` state |
| `/three_finger_limit_broadcaster/values` | `control_msgs/msg/Float64Values` | Published | Minimum and maximum external diameter in metres |
| `/diameter_controller/commands` | `std_msgs/msg/Float64MultiArray` | Subscribed | External-diameter position command in metres |

The 3FG launch also starts `/gripper_state_broadcaster/state` and `/diagnostics`.
Use their validity flags: 3FG force is a diagnostic percentage, not a measured
Newton value. The 3FG packages do not expose `ParallelGripperCommand` or
realtime control.

## Isaac bridge interfaces

| Interface | Type | Direction from ROS hardware adapter | Default | Configuration input |
|---|---|---|---|---|
| Isaac joint commands | `sensor_msgs/msg/JointState` | Published | `isaac_joint_commands` | `isaac_joint_commands_topic` |
| Isaac joint state | `sensor_msgs/msg/JointState` | Subscribed | `isaac_joint_states` | `isaac_joint_states_topic` |
| Simulation clock | `rosgraph_msgs/msg/Clock` | Subscribed by ROS nodes | `/clock` | `use_sim_time` |

See [message fields](messages.md) and [launch parameters](launch-parameters.md)
for the complete data and configuration contracts.

MoveIt uses a separate `/physical_gripper_controller/gripper_cmd` action
with physical-joint coordinates, not task aperture. See the
[MoveIt guide](../moveit-integration.md) before choosing an action endpoint.

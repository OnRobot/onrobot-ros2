# Plan and execute with MoveIt 2

Use the [standalone MoveIt example](../onrobot_gripper_moveit_config/README.md#try-a-gripper-without-hardware)
to plan collision-aware motion without hardware, then select the real backend
after commissioning. All four parallel grippers use a physical planning joint:

| Family | Planning joint | Unit |
|---|---|---|
| 2FG7, 2FG14 | `finger_stroke` | m, one-jaw travel |
| RG2, RG6 | `finger_joint` | rad, CAD linkage angle |

The dedicated `/physical_gripper_controller/gripper_cmd` adapter maps that
physical target to `/gripper_controller/gripper_cmd` task aperture. Do not plan
the task-only `grip_stroke` leaf: it does not move collision geometry.

When composing an application, include the supplied physical joint and follower
mimics, the matching profile and action adapter. Preserve the independent
task/physical endpoints and correct prefixes. See the
[integration contract](../onrobot_gripper_moveit_config/README.md#add-to-an-application)
for controller configuration, supported fields and collision exclusions.

The standalone example supports real and fake backends, not the Isaac adapter
path. Custom fingers require matching mapping and collision validation.

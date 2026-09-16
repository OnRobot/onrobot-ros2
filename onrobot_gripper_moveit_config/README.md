# OnRobot gripper MoveIt 2 configuration

Plan and execute gripper opening changes with MoveIt 2. The planning group uses
the physical finger joint, so both fingers and their collision geometry move
with the planned state. A dedicated action adapter converts the physical target
to the task aperture consumed by the existing OnRobot controller.

## Try a gripper without hardware

After building and sourcing the workspace, run:

```bash
ros2 launch onrobot_gripper_moveit_config moveit_gripper.launch.py \
  model:=2fg7 backend:=fake
```

In another sourced terminal, send one planned physical-joint target:

```bash
ros2 run onrobot_gripper_moveit_config moveit_gripper_smoke \
  --ros-args -p joint:=finger_stroke -p target:=0.010
```

For RG2 or RG6, select that model in the launch command and use
`-p joint:=finger_joint -p target:=0.3` in the second command. For 2FG14,
the same 0.010 m physical-joint example applies. `namespace` is optional; when
set, also give the smoke command `-r __ns:=/your_namespace`.

For a real gripper use `backend:=real host:=<gripper-IP> port:=502`. Ensure the
installed fingers match the stock description and the motion area is clear.
Do not run a separate showcase or another device owner at the same time.

## Coordinates and stock limits

The public task-aperture action remains `/gripper_controller/gripper_cmd`, in
metres. MoveIt instead sends the same standard
`control_msgs/action/ParallelGripperCommand` type to
`/physical_gripper_controller/gripper_cmd`, using the physical joint below.
These action endpoints are deliberately distinct: their positions have different
meanings and, for RG grippers, different units.

| Model | Physical planning joint | Unit | Stock planning aperture |
| --- | --- | --- | --- |
| 2FG7 | `finger_stroke` | m, one-jaw travel | 33–71 mm |
| 2FG14 | `finger_stroke` | m, one-jaw travel | 55–105 mm |
| RG2 | `finger_joint` | rad, CAD linkage angle | 0–101.1 mm |
| RG6 | `finger_joint` | rad, CAD linkage angle | 0–150 mm |

These are geometry-specific planning envelopes, not a reduction of the device's
capabilities. Changing fingertip orientation or geometry requires a matching
description and profile. The adapter also checks live device limits; it does
not stretch the geometry to fit a different aperture range. CAD joint zero on
RG grippers is below rubber contact, so the supplied profile has a nonzero
minimum planning angle.

The leader is the group's only active joint. Include the supplied follower
joints in the group as URDF mimics so copying a planned state updates the whole
linkage. Do not add the task-only `grip_stroke` leaf or the raw mechanism
measurement as independently planned actuators.

## Execution and safety behavior

The adapter uses the installed, hashed stock geometry profile and requires
fresh joint, typed gripper-state and task-limit feedback. Inconsistent geometry,
stale state, faults and out-of-range targets prevent execution. Small measured
pad compression or encoder quantization at a boundary may be represented at
the rigid geometry endpoint; this never changes the original measured topics
or permits an out-of-range command.

A new goal cancels the preceding goal and waits for its terminal response before
forwarding another. Lost state cancels an active goal. If cancellation cannot be
confirmed, further motion is inhibited; inspect the device and restart the
adapter only after the condition is resolved. This software mechanism is not a
safety-rated stop.

The adapter currently accepts position-only goals. Leave the optional velocity
and effort arrays empty. It rejects those fields rather than interpreting an
angular effort as a gripping force. Planning velocity/acceleration limits affect
the planned trajectory, but this gripper action executes its final target; they
are not a command to track each timed trajectory point or a hardware speed limit.
Use the documented conventional/realtime interfaces for explicit device control.

The conventional controller can report a grasp stall as success. The adapter
preserves that distinction: a grasp stall does not mean the requested physical
position was reached, and it does not establish force accuracy.

## Add to an application

Include normal OnRobot bringup, this adapter and its matching profile. Configure
MoveIt's native `ParallelGripperCommand` controller handle using
`config/moveit_controllers.yaml`, selecting the correct physical joint. Preserve
the relative action/topic names when using a ROS namespace. For custom prefixed
descriptions, the adapter's `joint_prefix` parameter must match all joint names.

The supplied SRDF excludes contact between assembled mechanical neighbors and
RG mating pads at the closed endpoint. All collision-bearing links, including
the fingertips, still collide with planning-scene objects. These exclusions do
not change the Isaac asset's contact physics. Recheck the collision matrix and
coordinate profile whenever replacing fingers or collision meshes.

The standalone launch supports real and fake backends. It does not currently
provide the physical-planning adapter path for the Isaac backend.

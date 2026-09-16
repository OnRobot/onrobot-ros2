# OnRobot gripper description contract

This package owns the model-neutral description contract used by the supported
parallel grippers. Model packages continue to own their URDF/Xacro and mesh
assets, and application packages own any custom-finger geometry they add.

## Coordinates

Each finger profile declares two independent coordinates:

- `task_coordinate` is the linear gripping aperture in metres. It is the
  public task-action coordinate (`grip_stroke`), not a physical planning joint.
- `mechanism_coordinate` is the raw coordinate of the unchanged gripper
  mechanism: metres for 2FG and radians for RG. It is published in the typed
  gripper state; it is not represented as an extra URDF articulation joint.

Changing fingers may change the useful task aperture and its mapping, but it
does not redefine the raw mechanism coordinate. Consumers must use the
dimension and unit in the profile instead of assuming that every mechanism is
linear.

For 2FG descriptions, `finger_stroke` is the physical right-jaw prismatic
joint and `left_finger_base_joint` mimics it 1:1. The device profile maps the
raw linear mechanism measurement to that one-jaw travel, including any offset
and model-specific scale. Do not infer a current raw range or one-jaw travel
from the model name: use the selected model and finger profile limits.
MoveIt plans the physical leader joint (`finger_stroke` for 2FG or
`finger_joint` for RG) and converts targets through the separate physical-joint
action adapter. See the [MoveIt integration](../onrobot_gripper_moveit_config/README.md).
`grip_stroke` remains the semantic task coordinate used by standard ROS
control; simulation backends must not treat it as a second independently driven
articulation.

## Custom-finger attachment

Every supported real-mesh and simple-mesh Xacro exposes these stable URDF
frames:

- `left_finger_mount`
- `right_finger_mount`

They are massless fixed links located at the physical standard-fingertip
attachment positions, with an orientation independent of selectable standard
finger options. The supplied standard fingertip links retain any installation
rotation below those frames. A custom-finger package can therefore
attach its own visual, collision and inertial links to these frames without
depending on internal linkage names or changing the gripper's mechanism
kinematics.

Set the model Xacro argument `use_standard_fingertips:=false` to omit the two
supplied fingertip links, their fixed joints, and their fingertip-specific
Gazebo contact tags. The mount links and the complete gripper mechanism remain
unchanged. The default is `true`, so existing descriptions retain their
current geometry.

The installed, tested example
`examples/custom_fingers/2fg7_custom_fingers.urdf.xacro` shows the intended ownership
boundary: it includes the OnRobot model with standard fingertips disabled,
then defines consumer-owned links and fixed joints whose parents are
`left_finger_mount` and `right_finger_mount`. It is deliberately a structural
example rather than a production finger design.

The custom-finger package remains responsible for valid mass properties,
collision geometry, contact material, and corresponding USD assets. URDF and
USD variants should preserve the attachment-frame names and the profile's task
and mechanism coordinate semantics so ROS 2, MoveIt 2, and Isaac Sim observe
the same gripper state.

Profiles in `config/finger_profiles` are versioned data. Consumers should
reject an unsupported `schema_version` and identify a profile by both `name`
and `revision` when reproducibility matters. The supplied stock profile files
use `schema_version: 1` and describe device/finger metadata. The separate
`config/planning_profiles` files use schema 2 and are consumed by
`GripperProfile` and the physical-joint action adapter. These profiles bind the
stock mesh's physical coordinates to its useful aperture without rescaling the
mechanism. `resolve_gripper_profile --profile <file>` reports the resolved
domains and content hash; add `--joint <q>` or `--aperture <metres>` for a
checked conversion. Custom runtime profiles require matching geometry and
independent validation, not just edited endpoint numbers.

Both mesh variants for 2FG7, 2FG14, RG2, and RG6 provide reusable macros. Their
macros accept `parent`, `prefix`, `origin_xyz`, and `origin_rpy`; detailed
variants also expose their control inclusion and connection parameters. The
installed fixtures under `examples/composition` demonstrate single-arm and
dual-gripper composition beneath caller-owned tool links, including custom
geometry attached to the prefixed stable mounts.

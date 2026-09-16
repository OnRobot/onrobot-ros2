# Descriptions and custom fingers

Product packages provide reusable simplified and detailed Xacro macros,
meshes, SRDF, and RViz configuration. The shared
`onrobot_gripper_description` package owns finger-profile and coordinate
semantics.

For custom fingers, set:

```text
use_standard_fingertips:=false
```

Attach application-owned geometry below the stable frames:

```text
left_finger_mount
right_finger_mount
```

The model macro retains the gripper mechanism while omitting the supplied
fingertip links. The application owns valid mass and inertia, collision
geometry, material/contact parameters, task-coordinate limits, and equivalent
simulation assets.

Use prefixes when composing more than one gripper, and preserve the stable
mount names below each prefix. The installed examples under
`onrobot_gripper_description/examples` demonstrate composition and a
structural 2FG7 custom-finger example.

Do not reinterpret the mechanism coordinate when fingers change. Task aperture
and raw mechanism feedback are distinct coordinates described by the selected
versioned finger profile.

These are description-macro options, not generic showcase launch arguments.
Changing a mesh alone does not update device settings, task mapping or USD.
See the [description and profile reference](../onrobot_gripper_description/README.md)
for model-specific options and the supported stock profiles.

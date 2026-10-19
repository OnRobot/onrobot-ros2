# OnRobot 2FG7 ROS 2 description

URDF/Xacro, meshes, and model-specific control configuration for the OnRobot 2FG7.
Use the workspace Getting Started guide for installation and control bringup.

## Configure the description

The simplified description uses silicone fingertips and outward
fingertip orientation by default. Available options are:

- `fingertip_material:=silicone|aluminum`
- `fingertip_orientation:=outwards|inwards`

The package includes `simplemesh_onrobot_2fg7.urdf.xacro` and
`realmesh_onrobot_2fg7.urdf.xacro`. Control bringup uses the detailed model by
default and accepts `description_xacro` when another installed variant is
needed.

## Control

Use the shared launch file for fake or physical control:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 backend:=fake start_rviz:=true
```

Set `backend:=real`, `host`, and `port` for a physical gripper. Add
`control:=realtime` to control aperture position and velocity. The realtime RViz layout provides Open, Close, target, and held
velocity controls.

The fingertips contain Gazebo contact parameters. These are simulation inputs,
not guaranteed gripping-performance values; check and adjust them for your payload, surface condition, and simulator.

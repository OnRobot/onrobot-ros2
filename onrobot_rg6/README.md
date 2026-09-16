# OnRobot RG6 ROS 2 description

URDF/Xacro, meshes, and model-specific control configuration for
the OnRobot RG6. Use the workspace Getting Started guide for installation and
model-neutral bringup.

## Configure the description

Available Xacro options are:

- `fingertip_orientation:=inwards|outwards`
- `fingertip_material:=standard|x|50_standard|50_x|100_standard|100_x`

The package contains simplified and detailed mesh Xacros. The installed
control launch uses the detailed model.

## Control

Use the model-neutral launch for fake, physical, or Isaac Sim control:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=rg6 backend:=fake start_rviz:=true
```

Set `backend:=real`, `host`, and `port` for a physical gripper. The conventional
RG state reports measured aperture but not measured gripping force, so
`/joint_states` omits effort and the conventional panel shows aperture only.
The configured force limit is still sent with each motion command.

Set `backend:=isaac` after starting the model-neutral Isaac bridge with
`run_ros_bridge.py --model rg6`. The ROS adapter maps aperture commands to the
asset's driven `finger_joint` angle and maps simulated angle feedback back to
aperture. See the `onrobot_gripper_isaac` package for the complete setup and
verification procedure.

Set `control:=realtime` for compensated task-aperture position
and mechanism-angular-velocity control. The realtime RViz layout provides
Open, Close, target, force-limit, and held-velocity controls.

The fingertips contain Gazebo contact parameters. These are simulation inputs,
not guaranteed gripping-performance values; users should tune them for their
payload, surface condition, and simulator.

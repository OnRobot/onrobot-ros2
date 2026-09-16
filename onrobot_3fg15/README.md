# OnRobot 3FG15 ROS 2 package

URDF/Xacro, meshes, and native external-diameter control for the
OnRobot 3FG15. The ROS control path is available for evaluation only.

To connect to a gripper and use the RViz Open/Close controls, follow the
workspace Getting Started guide or run:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=3fg15 host:=192.168.1.1 port:=502
```

The reference configuration uses fingertip position 2. Provide
`fingertip_position:=1|2|3` if the physical fingertips have been remounted.
Bringup checks the physical setting before accepting commands.

## Configure the description

The simplified description uses 13 mm silicone fingertips and fingertip
position 2 by default. Available options are:

- `fingertip_position:=1|2|3`, where 1 is 9 mm back, 2 is the default, and 3
  is 9 mm forward;
- `fingertip_material:=13_silicone|165_silicone|13_steel|10_steel|knurled`.

The package includes `simplemesh_onrobot_3fg15.urdf.xacro` and
`realmesh_onrobot_3fg15.urdf.xacro`.

The fingertips contain Gazebo contact parameters. These are simulation inputs,
not guaranteed gripping-performance values; users should tune them for their
payload, surface condition, and simulator.

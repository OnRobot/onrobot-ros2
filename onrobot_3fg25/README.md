# OnRobot 3FG25 ROS 2 description

URDF/Xacro, meshes, and native external-diameter control for the
OnRobot 3FG25. The ROS control path is available for evaluation only.

To connect to a gripper and use the RViz Open/Close controls, follow the
workspace Getting Started guide or run:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=3fg25 host:=192.168.1.1 port:=502
```

## Configure the description

Available Xacro options are:

- `fingertip_position:=1|2|3`
- `fingertip_material:=silicone|steel|knurled`

The package includes `simplemesh_onrobot_3fg25.urdf.xacro` and
`realmesh_onrobot_3fg25.urdf.xacro`.

## Diameter control

The shared showcase uses external grip diameter in metres and exports the
measured coupled finger angle. It is not a
`control_msgs/action/ParallelGripperCommand` interface because a three-finger
diameter is not a parallel-jaw aperture.

The reference configuration uses fingertip position 3. The
`fingertip_position` launch value must match the physical tool. Commands
are limited by live device state, and the showcase does not provide adjustable
gripping force.

The fingertips contain Gazebo contact parameters. These are simulation inputs,
not guaranteed gripping-performance values; users should tune them for their
payload, surface condition, and simulator.

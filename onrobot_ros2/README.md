# OnRobot ROS 2 suite

`onrobot_ros2` groups all OnRobot ROS packages: model descriptions, hardware,
controllers, messages, bringup, RViz, MoveIt, Isaac integration and demos.
2FG7, 2FG14, RG2 and RG6 are supported; 3FG15 and 3FG25 are included for
evaluation only.

After installing the Tool API and ROS dependencies and providing the matching
Isaac assets, build this package and its dependencies with
`colcon build --packages-up-to onrobot_ros2`. Follow the repository's
`GETTING_STARTED.md` for the complete installation procedure.

After sourcing the workspace, report the suite version with:

```bash
ros2 pkg xml onrobot_ros2 --tag version
```

This version matches the repository changelog's suite version. Component
packages retain their own versions. The installed `share/onrobot_ros2` directory
contains `package.xml`, this README and the suite's `CHANGELOG.md`.

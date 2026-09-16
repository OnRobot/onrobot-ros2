# OnRobot ROS 2 documentation

Start with [Connect to and move an OnRobot gripper](../GETTING_STARTED.md),
then choose the guide for your application.

| Task | Guide |
|---|---|
| Install and build | [Installation](installation.md) |
| Select a product and backend | [Supported devices](supported-devices.md) |
| Check a physical connection | [Hardware commissioning](hardware-commissioning.md) |
| Stream position, velocity or closing force | [Realtime control](realtime-control.md) |
| Inspect measurements and recover | [State, diagnostics and recovery](state-diagnostics-and-recovery.md) |
| Plan and execute | [MoveIt 2](moveit-integration.md) |
| Compose descriptions or replace fingers | [Descriptions and custom fingers](custom-fingers-and-description.md) |
| Simulate or connect Isaac to ROS | [Isaac Sim](isaac-sim.md) |
| Collect and replay data | [Session recording and replay](session-recording-and-replay.md) |
| Investigate a problem | [Record a useful bug report](../onrobot_gripper_bringup/doc/REPORTING_ISSUES.md) |

## Reference

- [Launch parameters](reference/launch-parameters.md)
- [ROS interfaces](reference/ros-interfaces.md)
- [Message fields](reference/messages.md)
- [Hardware backends](../onrobot_gripper_hardware/README.md)
- [Controllers](../onrobot_gripper_controllers/README.md)
- [RViz panels](../onrobot_gripper_rviz_plugins/README.md)
- [Description profiles](../onrobot_gripper_description/README.md)

Package `doc/` directories also provide rosdoc2/Sphinx documentation. They
retain the detailed procedures linked here; the task guides are navigation
and user workflows, not a second set of interface definitions.

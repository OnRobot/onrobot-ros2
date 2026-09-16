<p align="left">
  <img src="docs/images/onrobot-logo.png" alt="OnRobot" width="260">
</p>

# OnRobot grippers for ROS 2

*One System, Zero Complexity.*

Control and visualize OnRobot grippers with ROS 2 Jazzy and `ros2_control`.
Use conventional actions, realtime control, device diagnostics, RViz panels,
MoveIt 2 and Isaac Sim integration through the same ROS workspace.

| Models | Conventional control | Realtime control | Isaac assets |
|---|---|---|---|
| 2FG7, 2FG14 | Task-aperture action | Position, velocity, closing-force approaches | Yes |
| RG2, RG6 | Task-aperture action | Position, mechanism-angular velocity | Yes |
| 3FG15, 3FG25 | External-diameter commands (evaluation only) | No | No |

See [supported devices and backends](docs/supported-devices.md) for the
capabilities and limits of each path.

## Install and try without hardware

Download tool api debian packages from [latest release](https://github.com/OnRobot/onrobot-tool-api/releases/latest)

Use Ubuntu 24.04 amd64 and ROS 2 Jazzy. Install the matching Tool API runtime
and development packages from the supplied DEB directory:

```bash
sudo apt install \
  ./libonrobot-tool-api1_*_amd64.deb \
  ./libonrobot-tool-api-dev_*_amd64.deb
```

Tool API source is not required. Put this repository in `src/onrobot-ros2`
with the [matching asset dependency](onrobot_gripper_isaac/README.md#asset-dependency);
supplied bundles already contain it. From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/onrobot-ros2 --ignore-src -r -y \
  --skip-keys onrobot_tool_api
colcon build --base-paths src/onrobot-ros2 --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF --packages-skip onrobot_gripper_isaac
source install/setup.bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 backend:=fake control:=conventional start_rviz:=true
```

The panel is loaded automatically. Wait for live state, then try Open, Close
and Send target. Stop this launch before connecting hardware. For dependencies,
offline setup and Docker, see [installation](docs/installation.md).

## Connect hardware

Follow [Connect to and move an OnRobot gripper](GETTING_STARTED.md) for TCP
(Compute Box default `192.168.1.1:502`) or USB/Modbus RTU (`/dev/ttyUSB0`).
Its realtime RTU example sets the device, controller and state-publication
rates explicitly to 200 Hz. Requested rates are not timing guarantees.

Secure the gripper and payload, clear the full travel and provide an independent
stop before commanding motion. ROS Stop, watchdogs and RViz controls are not
safety-rated functions.

## Choose your next task

| Task | Guide |
|---|---|
| First fake or real motion | [Getting started](GETTING_STARTED.md) |
| Setup checks and USB connection | [Commissioning](docs/hardware-commissioning.md) |
| Position, velocity and closing-force control | [Realtime control](docs/realtime-control.md) |
| Read state, diagnose faults and recover | [State and recovery](docs/state-diagnostics-and-recovery.md) |
| Plan collision-aware gripper motion | [MoveIt 2](docs/moveit-integration.md) |
| Mount a gripper or describe custom fingers | [Descriptions](docs/custom-fingers-and-description.md) |
| Load a USD, run a physics example or connect ROS | [Isaac Sim](docs/isaac-sim.md) |
| Record and replay measured state | [Session recording](docs/session-recording-and-replay.md) |
| Capture a problem for investigation | [Report an issue](onrobot_gripper_bringup/doc/REPORTING_ISSUES.md) |
| Look up parameters, topics or messages | [Reference](docs/README.md#reference) |

The detailed [2FG7 HIL](onrobot_gripper_isaac/doc/RVIZ_ISAAC_2FG7_DEMO.md)
and [RG2 HIL](onrobot_gripper_isaac/doc/RG2_HIL_GUIDE.md) guides cover RViz,
Isaac and hardware together. The [documentation index](docs/README.md) also
links the package-level integration references.

## License

Supplied under the [BSD 3-Clause License](LICENSE), without warranty or a support
commitment. Third-party components retain their respective licenses.

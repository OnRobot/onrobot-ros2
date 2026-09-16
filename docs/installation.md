# Install OnRobot ROS 2

Use Ubuntu 24.04 amd64 with ROS 2 Jazzy. The supplied distribution contains ROS
source, the matching Isaac asset repository and a Tool API runtime/development
DEB pair. Tool API source is not needed and is not a ROS source submodule.

Follow [installation and workspace build](../GETTING_STARTED.md#2-install-the-tool-api-and-build-the-workspace)
for the commands. Stop if either DEB fails to install or rosdep reports a missing
dependency. Source `install/setup.bash` in each terminal after a successful build.

For separate checkouts, place `onrobot-ros2` under the workspace `src/` directory
and supply the exact [Isaac asset dependency](../onrobot_gripper_isaac/README.md#asset-dependency).
Do not copy USD files from a different release into the ROS package. Archives
already contain the pinned dependency and do not require Git.

Customer archives omit internal tests: use `-DBUILD_TESTING=OFF` as shown in
the guide. A full developer checkout can build tests using the public SDK.

## Verify installation

```bash
ros2 pkg prefix onrobot_gripper_bringup
ros2 pkg prefix onrobot_gripper_demos
ros2 launch onrobot_gripper_demos showcase.launch.py --show-args
```

The prefixes should point to the workspace you just built. If they point to
an older workspace, start a fresh terminal and source only Jazzy and the
intended installation. Keep the Tool API runtime and development packages at
the same version; mixing libraries and headers can cause missing-symbol errors.

## Other installation environments

- [Docker](../docker/README.md) provides a non-root ROS workspace with the paired
  Tool API packages and instructions for TCP, USB and optional RViz display.
- For offline machines, obtain the required ROS and system dependencies before
  disconnecting. The product bundle alone does not contain every Ubuntu package.
- For USB access and latency, use the [RTU setup guide](../onrobot_gripper_bringup/doc/USB_RTU.md).

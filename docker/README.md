# Run OnRobot ROS 2 in Docker

Build a Linux amd64 image from the supplied customer bundle. It includes the
matching binary Tool API, ROS 2 Jazzy dependencies, RViz panels, Ubuntu fonts
and installed gripper assets. It does **not** include the Isaac Sim application
or an NVIDIA driver. Run Isaac Sim separately when using its ROS bridge.

The image build needs Internet access to the configured Ubuntu and ROS package
repositories. Use a Docker-capable Linux host with access to those repositories;
Tool API source is not required. Container timing depends on the host and does
not provide a hard-realtime guarantee. ARM64 and emulated amd64 hardware control
are not covered by this image.

## Build from a bundle

Extract the bundle and work from its root, containing `BUNDLE_MANIFEST.json`,
`SHA256SUMS`, `tool-api-debs` and `src`. Do not use a full developer workspace
as the Docker build context. The build checks every supplied file and the exact
Tool API package pairing before installing packages. Obtain the bundle from
OnRobot: checksums detect changed files, but do not authenticate their publisher.

```bash
sha256sum --strict --check SHA256SUMS
docker build --platform linux/amd64 \
  -f src/onrobot-ros2/docker/Dockerfile \
  --build-arg BUILD_JOBS=2 -t onrobot/ros2:local .
```

Release automation can pass `--build-arg ROS_BASE_IMAGE=...` with an approved
digest in place of the default `ros:jazzy-ros-base-noble` tag. The image retains
the source bundle and its manifest for development and troubleshooting.
Image identity also depends on the base image and the dependency versions
resolved at build time.

## First launch without hardware

```bash
docker run --rm --name onrobot-demo \
  --cap-drop=ALL --security-opt=no-new-privileges:true \
  onrobot/ros2:local \
  ros2 launch onrobot_gripper_bringup gripper.launch.py \
  model:=2fg7 backend:=fake
```

In another terminal, use the entrypoint when executing commands so both ROS
and the installed OnRobot workspace are sourced:

```bash
docker exec onrobot-demo /onrobot-entrypoint.sh ros2 control list_controllers
docker exec onrobot-demo /onrobot-entrypoint.sh \
  ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  '{command: {name: [grip_stroke], position: [0.05], effort: [10.0]}}'
```

Expect active conventional controllers and a successful action. Select `2fg14`,
`rg2` or `rg6` for the other parallel grippers. Stop the container with Ctrl+C
before starting another driver for the same device.

## Connect real hardware

Secure the gripper and clear its travel. Replace the example IP with the actual
gripper endpoint. Outbound Modbus TCP can use Docker's normal bridge network:

```bash
docker run --rm --name onrobot-demo \
  --cap-drop=ALL --security-opt=no-new-privileges:true \
  onrobot/ros2:local \
  ros2 launch onrobot_gripper_bringup gripper.launch.py \
  model:=2fg7 backend:=real host:=192.168.1.1 port:=502
```

For Linux USB/RS-485, expose only the selected serial device and its group.
Configure host USB latency and serial permissions using the
[USB / Modbus RTU guide](../onrobot_gripper_bringup/doc/USB_RTU.md) first:

```bash
ONROBOT_TTY="$(readlink -f /dev/ttyUSB0)"
test -c "$ONROBOT_TTY"
docker run --rm --name onrobot-demo \
  --cap-drop=ALL --security-opt=no-new-privileges:true \
  --device "$ONROBOT_TTY:$ONROBOT_TTY" \
  --group-add "$(stat -c '%g' "$ONROBOT_TTY")" \
  onrobot/ros2:local \
  ros2 launch onrobot_gripper_bringup gripper.launch.py \
  model:=2fg7 backend:=real transport:=rtu serial_device:="$ONROBOT_TTY" \
  baud_rate:=1000000 rtu_parity:=even slave_id:=65
```

The adapter path is an example; use one that exists on the host. Do not run a
host driver or another Modbus master on that bus at the same time. To activate
realtime control, add `start_realtime_controller:=true`; use the USB guide for
explicit device, controller and publication rates. Do not use `--privileged`,
mount all of `/dev`, or change host serial-driver settings inside the container.

## Show RViz on a Linux desktop

Use the desktop's X11 or XWayland display with an explicit authorization cookie.
The host needs `xauth`, and `DISPLAY` must identify a working local display.
Do not disable X server access control with `xhost +`.

```bash
ONROBOT_XAUTH="$(mktemp)"
xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$ONROBOT_XAUTH" nmerge -
test -s "$ONROBOT_XAUTH" || { rm -f -- "$ONROBOT_XAUTH"; exit 1; }
docker run --rm --name onrobot-demo \
  --user "$(id -u):$(id -g)" \
  --cap-drop=ALL --security-opt=no-new-privileges:true \
  -e DISPLAY -e XAUTHORITY=/tmp/onrobot.xauth \
  -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
  --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly \
  --mount "type=bind,src=$ONROBOT_XAUTH,dst=/tmp/onrobot.xauth,readonly" \
  onrobot/ros2:local \
  ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 backend:=fake control:=conventional start_rviz:=true
rm -f -- "$ONROBOT_XAUTH"
```

This uses software rendering, which is sufficient for a single gripper. An
administrator can configure GPU access separately for heavier scenes. When ROS
nodes run outside this container, Linux `--network host` is a simple way to
share the host's ROS network; it removes Docker network isolation, so use it
only on the intended trusted ROS network. It is not needed when all ROS nodes
run inside the one container.

The default image user is unprivileged UID/GID 10001. The desktop example uses
your host UID to read its temporary authorization file. The entrypoint puts ROS
logs and GUI runtime files in a per-invocation temporary directory; set
`ROS_LOG_DIR` to a writable mounted directory when you need to retain logs.
Use writable output mounts for rosbag recordings. Docker Desktop USB forwarding
and non-Linux display setup depend on the host and are not provided here.

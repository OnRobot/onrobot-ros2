# Record a useful bug report

Use this procedure for unexpected motion, slow feedback, force spikes, missing
TF, or RViz failures. It records ROS traffic only: it does not connect another
Modbus client or command the gripper. Keep an independent stop available. If
the incident could damage equipment, preserve the existing logs and report it
without repeating it. Do not reproduce a loaded failure until the fixture is safe.

For a presentation recording with safe RViz playback, use the showcase's
`record_gripper_session` instead. **The diagnostic bag below contains command
topics. Never run `ros2 bag play` on it alongside a live driver.** It is not a
presentation session and must not be passed to `replay_gripper_session`.

## 1. Keep the launch log and identify the setup

Run your normal showcase from a sourced terminal and save its complete output.
Append the following to your existing launch command, keeping its exact model,
transport, controller and rate arguments:

```bash
set -o pipefail
# Example; replace the connection and rates with the setup being investigated.
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 backend:=real transport:=rtu serial_device:=/dev/ttyUSB0 \
  baud_rate:=1000000 rtu_parity:=even slave_id:=65 \
  control:=realtime start_rviz:=true \
  realtime_update_rate_hz:=200 controller_manager_update_rate_hz:=200 \
  state_publish_rate_hz:=200 2>&1 | tee gripper-launch.log
```

For TCP, replace the RTU connection arguments with
`transport:=tcp host:=192.168.1.1 port:=502`, using the actual Compute Box
address. Preserve the rates at which the problem occurs rather than increasing
them for the recording. Do not restart a running driver merely to capture logs;
its existing launch log is normally in `~/.ros/log/`.

## 2. Capture the context and start a bag

In another terminal, source the **same installed workspace**. Use an empty
namespace for the normal standalone showcase, or the instance namespace with
a leading slash and no trailing slash, such as `/gripper_a`.

```bash
export ONROBOT_NS=""
export ONROBOT_REPORT="$PWD/bug-reports/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$ONROBOT_REPORT"
date -u --iso-8601=seconds > "$ONROBOT_REPORT/start-time.txt"
uname -srmo > "$ONROBOT_REPORT/platform.txt"
ros2 pkg prefix onrobot_gripper_hardware > "$ONROBOT_REPORT/hardware-prefix.txt"
ros2 pkg prefix onrobot_gripper_rviz_plugins > "$ONROBOT_REPORT/panel-prefix.txt"
dpkg-query -W -f='${Package} ${Version} ${Architecture}\n' \
  libonrobot-tool-api0 libonrobot-tool-api-dev \
  > "$ONROBOT_REPORT/api-packages.txt" 2>&1
timeout 10s ros2 control list_controllers -c "$ONROBOT_NS/controller_manager" \
  > "$ONROBOT_REPORT/controllers.txt" 2>&1
ros2 topic list -t > "$ONROBOT_REPORT/topics.txt"
timeout 10s ros2 param dump "$ONROBOT_NS/controller_manager" \
  > "$ONROBOT_REPORT/controller-manager.yaml" 2>&1
timeout 10s ros2 param dump "$ONROBOT_NS/realtime_controller" \
  > "$ONROBOT_REPORT/realtime-controller.yaml" 2>&1
timeout 10s ros2 param dump "$ONROBOT_NS/robot_state_publisher" \
  > "$ONROBOT_REPORT/robot-state-publisher.yaml" 2>&1

ros2 bag record --storage mcap --output "$ONROBOT_REPORT/bag" \
  --include-hidden-topics --topics \
  "$ONROBOT_NS/joint_state_broadcaster/joint_states" \
  "$ONROBOT_NS/joint_state_broadcaster/dynamic_joint_states" \
  "$ONROBOT_NS/joint_states" \
  "$ONROBOT_NS/gripper_state_broadcaster/state" \
  "$ONROBOT_NS/realtime_controller/state" \
  "$ONROBOT_NS/realtime_controller/command" \
  "$ONROBOT_NS/parallel_gripper_limit_broadcaster/names" \
  "$ONROBOT_NS/parallel_gripper_limit_broadcaster/values" \
  "$ONROBOT_NS/diagnostics" "$ONROBOT_NS/robot_description" \
  "$ONROBOT_NS/controller_manager/activity" \
  "$ONROBOT_NS/gripper_controller/gripper_cmd/_action/status" \
  "$ONROBOT_NS/gripper_controller/gripper_cmd/_action/feedback" \
  /tf /tf_static /clock /rosout
```

If the API was installed from source rather than DEBs, `api-packages.txt` may
report missing packages. Include the API installation prefix, build/revision
and matching header/library identity instead. A missing inactive controller's
parameters are also useful context, not a reason to abandon the capture.

Wait until the recorder subscribes to the live state topics. Some listed topics
are optional: `/clock` is usually absent with real hardware, and conventional
action traffic may be absent in realtime mode. Recording uses each publisher's
offered QoS. If a required state topic is missing or a QoS warning appears, save
`ros2 topic info -v TOPIC` and the warning; do not report that topic as captured.

Record 5–10 seconds of normal state, the incident if safe to reproduce, and
5–10 seconds afterward. Note the UTC time and the exact button/target/speed
used. Press **Ctrl+C in the recorder terminal** and wait for it to finish before
closing the driver. This stops recording, not hardware motion. Use the normal
Stop or independent stop for the equipment.

After recording:

```bash
ros2 bag info "$ONROBOT_REPORT/bag" | tee "$ONROBOT_REPORT/bag-info.txt"
cp gripper-launch.log "$ONROBOT_REPORT/"  # if saved in this working directory
```

Keep the complete `bag` directory, including `metadata.yaml` and every `.mcap`
file. Check that both raw and filtered joint states, typed state and diagnostics
have nonzero counts. Include realtime command/state counts for a realtime issue.
If a topic was absent, say so. The capture does not include Modbus wire traffic
or conventional action/service requests; include their exact CLI requests or
panel actions and responses in the report.

## 3. Explain what happened

Use this template in GitHub, Bitbucket or your private support channel:

```text
Title: [model] [TCP/RTU] brief observed symptom

Expected behavior:
Actual behavior:
Frequency: once / intermittent / every attempt (attempt count)
UTC incident time and approximate seconds into bag:

Model and device-reported firmware:
Bundle filename/version or repository revisions:
Tool API runtime and development versions / source install prefix:
Ubuntu, ROS distribution, RViz and Isaac version (if involved):
Backend and transport: real/fake/Isaac; TCP/LAN/VPN or RTU
RTU adapter, serial path, baud rate, parity, slave ID, USB latency:
Requested rates: device / controller manager / state publication:
Observed rates, sample age and health-counter changes:
Namespace, frame prefix, stock/custom fingers and fingertip settings:
Workpiece dimensions, mass, material, grip direction and target force/speed:

Exact launch command (redacted as needed):
Steps/buttons/targets to reproduce safely:
Does it happen headless, without Isaac, or in conventional mode? (if tested)
Did RViz exit, freeze, or only lose the model? Process exit code/log:
Fault/recovery messages; did motion stop? Was a restart/power cycle needed?

Attached: complete bag, bag-info, launch log, context files, screenshot/video
Missing information or tests not performed:
```

For a force spike or NaN fault, retain the unfiltered numeric values. State
validity, fault/status registers and timestamps distinguish an actual device
reading from stale or invalid data. Negative closing force is the current 2FG
convention; do not flip its sign, clip it to the command limit, or replace it
with a held value before sharing. `TF_NAN_INPUT` proves invalid transform input,
but does not by itself prove that RViz crashed or identify the device-side trigger.

## 4. Share privately and inspect without motion

Review the files before uploading. Bags and parameter dumps can contain robot
geometry, application names, serial numbers, IP addresses, file paths and
messages from other nodes on `/tf` or `/rosout`. Do not attach proprietary
scenes, credentials or customer information to a public issue. Use an approved
private channel for the complete evidence; summarize/redact text separately and
identify any removed data. Never edit the only copy of the raw bag.

`ros2 bag info` and offline `rosbag2_py` analysis do not publish commands.
Do not replay this diagnostic bag on the operational ROS graph. Use the normal
state-only recording workflow when you need the supported RViz playback demo.

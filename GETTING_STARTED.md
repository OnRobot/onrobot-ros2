# Connect to and move an OnRobot gripper

This guide starts one gripper, opens RViz, and gives you an Open/Close control.
You need the gripper model, a TCP endpoint or USB/RS-485 device, the supplied
Tool API runtime and development packages, and a ROS 2 Jazzy workspace
containing this repository.

This release supports **2FG7**, **2FG14**, **RG2**, and **RG6** on Ubuntu 24.04
amd64. The same entry point exposes the 3FG descriptions and conventional
diameter-control path for evaluation only. The 3FG path does not include RTU,
fake, Isaac, or realtime control.

## 1. Prepare safely

Mount or place the gripper securely. Clear the complete finger travel and keep
an independent means of removing power available. Only one Modbus client may
control the gripper. A software Stop or communication watchdog is not a
safety-rated emergency stop.

The showcase's `fingertip_position:=auto` setting selects the package reference
configuration: position 2 for 3FG15 and position 3 for 3FG25. If fingertips
have been remounted, provide their physical position
(`1`, `2`, or `3`). Bringup rejects a mismatch instead of moving with incorrect
diameter limits or visualization.

## 2. Install the Tool API and build the workspace

Install both supplied Tool API packages. The runtime package contains the
shared library; the development package contains the public headers and CMake
configuration required to build the ROS hardware plugin. Run this from the
directory containing only the two supplied `.deb` files:

```bash
sudo apt install \
  ./libonrobot-tool-api1_*_amd64.deb \
  ./libonrobot-tool-api-dev_*_amd64.deb
```

The runtime and development packages must have the same version and
architecture. For a fully offline installation, the packages'
declared dependencies must already be installed. Do not continue unless both
local packages install successfully.

The supplied bundle includes the matching `onrobot-isaac-sim` asset dependency.
For separate source checkouts, also provide that repository as described in
the [Isaac asset dependency](onrobot_gripper_isaac/README.md#asset-dependency).

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys onrobot_tool_api
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

The skipped key is the Tool API that was installed in the previous
step; it is not downloaded through rosdep. If dependencies were supplied as an
offline overlay, source that overlay before building. Do not run
`rosdep install` in an offline environment unless its dependency bundle is
already available.

Developers with the full checkout can enable tests with `-DBUILD_TESTING=ON`.

## 3. Start the showcase

First verify the workspace without hardware:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 backend:=fake control:=conventional start_rviz:=true
```

Wait for the fake panel to report live state, then stop that launch with
`Ctrl+C` before opening a real device connection.

The Compute Box default TCP address is `192.168.1.1`, port `502`. Configure
your computer's Ethernet interface on the same subnet, with a different unused
address. If the Compute Box address was changed, use its configured address.
Replace the model below with the connected gripper:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 host:=192.168.1.1 port:=502 \
  control:=conventional start_rviz:=true
```

RViz opens with the control panel selected by `control`. No manual panel setup
is required.

Valid model values are:

| Model | Conventional RViz control | Realtime RViz control |
|---|---|---|
| `2fg7` | Open, Close, target aperture | Position and velocity |
| `2fg14` | Open, Close, target aperture | Position and velocity |
| `rg2` | Open, Close, target aperture | Position and mechanism-angular velocity |
| `rg6` | Open, Close, target aperture | Position and mechanism-angular velocity |
| `3fg15` | Open/Maximum, Close/Minimum, target diameter | Not implemented |
| `3fg25` | Open/Maximum, Close/Minimum, target diameter | Not implemented |

For a three-finger gripper, include its actual fingertip position:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=3fg25 host:=192.168.1.1 port:=502 \
  fingertip_position:=3 start_rviz:=true
```

Provide the gripper's actual TCP port, normally `502`.

For a direct USB/RS-485 connection, select RTU and provide the actual serial
device path. OnRobot RTU framing is fixed at 8E1 and the supported baud rates
are `115200` and `1000000`; `slave_id` is `65` for a single gripper and `66` or
`67` for the primary or secondary mounting position. The TCP `host` and `port`
are ignored in RTU mode.

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 backend:=real transport:=rtu \
  serial_device:=/dev/ttyUSB0 \
  baud_rate:=1000000 rtu_parity:=even slave_id:=65 start_rviz:=true
```

The example uses `/dev/ttyUSB0`; check that it is the intended adapter. For a
persistent selection after reconnects, use its actual `/dev/serial/by-id/` path.
Use a device path that exists on the target computer and ensure the user
running ROS can open it. Only one Modbus master should use the bus at a time.
For permissions, USB latency and realtime setup, follow the
[USB / Modbus RTU guide](onrobot_gripper_bringup/doc/USB_RTU.md).

## 4. Open or close the gripper

Wait until the RViz panel reports a connected/live state and enables its
buttons. Do not press a motion button while it still says that limits or state
are unavailable.

For 2FG7, 2FG14, RG2, and RG6:

1. Click **Open** to command the live maximum aperture.
2. Click **Close** to command the live minimum aperture.
3. Use **Send target** for an intermediate aperture.

For 2FG conventional control, the panel starts with the firmware-supported
minimum nonzero force: 20 N for 2FG7 and 40 N for 2FG14. The live maximum is
device- and power-dependent. When supported by the selected backend, choose a
native speed percentage from 1–100 in the panel and click **Apply**; this
affects subsequent conventional goals, not a motion already in progress. The
launch argument `conventional_speed_percent:=1..100` sets the initial value
(default 50%). It is not a guaranteed SI velocity. See the
[conventional speed panel guide](docs/realtime-control.md#conventional-2fg-speed-panel).

For 3FG15 and 3FG25:

1. Click **Open / Maximum** to command the live maximum external diameter.
2. Click **Close / Minimum** to command the live minimum external diameter.
3. Use **Send target** for an intermediate external diameter.

The controls use limits reported by the connected device. They do not assume a
range from the model name. Custom fingers and fingertip mounting can change the
task-coordinate limits.

## 4a. Optional Isaac path

After the panel check, validate a standalone Isaac asset with [Run a first
gripper simulation](onrobot_gripper_isaac/doc/FIRST_ISAAC_SIMULATION.md). For
optional read-only or bounded HIL, use the [2FG7 HIL guide](onrobot_gripper_isaac/doc/RVIZ_ISAAC_2FG7_DEMO.md)
or [RG2 HIL guide](onrobot_gripper_isaac/doc/RG2_HIL_GUIDE.md). The
shadow keeps the real ROS backend as the sole device owner and sends no
hardware command. Simulator contact behavior is scene-dependent; this does not
establish calibrated force or payload performance.

## 5. Use realtime position and velocity control

Stop the conventional launch first. Realtime control is implemented for all
four parallel grippers. This USB example requests **200 Hz at all three stages**:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 backend:=real transport:=rtu serial_device:=/dev/ttyUSB0 \
  baud_rate:=1000000 rtu_parity:=even slave_id:=65 \
  control:=realtime start_rviz:=true \
  realtime_update_rate_hz:=200 \
  controller_manager_update_rate_hz:=200 \
  state_publish_rate_hz:=200
```

Use `model:=2fg14`, `rg2`, or `rg6` as appropriate. First check serial
permissions and USB latency in the [USB / Modbus RTU guide](onrobot_gripper_bringup/doc/USB_RTU.md).

| Argument | Controls | Default without an override |
|---|---|---|
| `realtime_update_rate_hz` | Device command/feedback exchanges | `auto`: 50 Hz |
| `controller_manager_update_rate_hz` | ros2_control update loop | 100 Hz |
| `state_publish_rate_hz` | Typed and realtime state publication | 100 Hz |

Increasing only one rate does not increase the others. Publication can repeat
the latest device sample; check `sample_sequence`, `sample_age`, cycle duration
and missed-deadline/failure counters, not just `ros2 topic hz`. The requested
rate is not a hard-realtime guarantee. The 200 Hz example assumes a 1 Mbit/s
RTU connection with suitable USB latency; reduce rates if the workload cannot
sustain them.

For TCP, replace the RTU-specific arguments with `transport:=tcp
host:=192.168.1.1 port:=502`. Start with conservative rates on an unmeasured
network or VPN, then set all three explicitly after checking timing. For example:

```bash
ros2 launch onrobot_gripper_demos showcase.launch.py \
  model:=2fg7 backend:=real transport:=tcp host:=192.168.1.1 port:=502 \
  control:=realtime start_rviz:=true \
  realtime_update_rate_hz:=50 \
  controller_manager_update_rate_hz:=100 state_publish_rate_hz:=100
```

In the realtime panel:

1. Set a conservative maximum velocity.
2. Click **Open**, **Close**, or enter an aperture and click **Send target** for
   realtime position control.
3. Press the joystick and drag it toward **OPEN** or **CLOSE** for velocity
   control.
4. Keep holding it to stream the velocity command.
5. Release it to request Stop, or use **Center / stop** explicitly.

For 2FG, velocity is task-aperture velocity in m/s. For RG, velocity is the
finger-independent mechanism angular velocity in rad/s. The panel selects the
correct coordinate and sign for the model. The 2FG velocity setting also limits
the realtime position approach speed. RG position commands use the force limit
shown beside the target.

For RG, **task aperture** is the external distance between the configured
fingertip contact surfaces. The separate mechanism position is the measured
finger angle in radians. If the displayed aperture does not match the physical
contact-surface gap, verify the fingertip offset configured in the gripper; do
not compensate the value again in ROS.

The 2FG7 and 2FG14 realtime position paths use the current realtime command
map and a firmware revision qualified by the Tool API. Older host and firmware
revisions are intentionally not supported because the command selectors and
field order are not compatible. Realtime force feedback follows the device
sign convention: closing is negative and opening is positive, while force
targets remain positive for a closing grip. Conventional control remains the simplest click-to-position
path. Do not compensate task feedback with a finger offset: task aperture
comes from the configured finger profile, while the separate mechanism
coordinate remains finger-independent. For closing-force grips on compatible
2FG devices, use the [force-approach controls](onrobot_gripper_rviz_plugins/README.md#2fg-closing-force-grip).
They provide explicit hold, release and Stop; validate the force and retention
for your fingers and workpiece before use.

For 2FG hardware, bringup writes `supply_power_w:=48` after every connection
and reconnect because the firmware setting is not retained across a power
cycle. This permits the documented force range. Use a lower value from 14 to
48 W when the installation requires it; use `supply_power_w:=0` only when an
external commissioning process owns that setting.

## 6. Stop and recover

Conventional motion can be cancelled through its standard action. Realtime
motion has an explicit Stop service:

```bash
ros2 service call /realtime_controller/stop std_srvs/srv/Trigger '{}'
```

After a latched communication fault, request a reconnect:

```bash
ros2 service call /recovery_controller/recover std_srvs/srv/Trigger '{}'
```

The recovery response only confirms that recovery was queued. Wait for the
state panel or `/diagnostics` to report a live connection before sending a new
command. Recovery does not replay an old motion command.

## 7. Troubleshooting

**The panel never becomes connected:** verify the model, IP, TCP port, power,
and that no other client owns the Modbus connection. Check controller state
with `ros2 control list_controllers` and device state on `/diagnostics`.

**Bringup rejects a 3FG:** confirm `fingertip_position` matches the physical
finger mounting. A mismatch intentionally fails before commands are accepted.

**Buttons remain disabled:** live state or live limits have not arrived. Do not
work around this by guessing endpoints; inspect `/joint_states` and, for 3FG,
`/three_finger_limit_broadcaster/values`.

**The model does not move in RViz:** confirm `/joint_states` changes and that
RViz's fixed frame is `world`. The showcase starts the required robot-state
publisher and fixed transform.

**Realtime stops after commands cease:** this is expected watchdog behavior.
The panel refreshes a position command until the target is reached and a
velocity command only while its joystick is held.

**A connection fault is latched:** clear the workspace, call the recovery
service, and wait for live state. Restart bringup if identity validation or
recovery continues to fail.

**Unexpected force spikes, missing TF, or a frozen/crashed RViz:** Stop motion
and preserve the launch log. Follow [Record a useful bug report](onrobot_gripper_bringup/doc/REPORTING_ISSUES.md)
to capture raw and filtered feedback together. Do not repeat a loaded fault
until the fixture is safe; a screenshot alone cannot identify its trigger.

## Isaac Sim preview

The supplied 2FG7, 2FG14, RG2, and RG6 assets can use the same conventional and
realtime panels through `backend:=isaac`. Start with the concise
[first-simulation guide](onrobot_gripper_isaac/doc/FIRST_ISAAC_SIMULATION.md), then
use the [Isaac Sim verification guide](onrobot_gripper_isaac/doc/ISAAC_SIM_VERIFICATION.md)
for repeatable tests. Each test report records the model, asset files, runner,
configuration, and runtime version actually exercised. A passing report shows
that the documented scenario worked with those exact inputs; it does not make
the preview a force-accurate digital twin.

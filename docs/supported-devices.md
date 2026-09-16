# Supported devices and backends

The main platform is Ubuntu 24.04 amd64, ROS 2 Jazzy and Isaac Sim 6.0.1.

| Models | Conventional coordinate | Realtime | Available showcase backends |
|---|---|---|---|
| 2FG7, 2FG14 | External task aperture, m | Position, linear velocity; closing-force approaches on compatible real devices | Real TCP/RTU, fake, Isaac |
| RG2, RG6 | Compensated external task aperture, m | Position, mechanism-angular velocity | Real TCP/RTU, fake, Isaac |
| 3FG15, 3FG25 | External diameter, m | Not implemented | Real TCP conventional control (evaluation only) |

The four parallel grippers use the standard `ParallelGripperCommand` action.
3FG uses its native diameter command topic instead. Its conventional ROS path
is available for evaluation only; its current showcase does not forward RTU
arguments or provide the parallel-gripper fake/Isaac backends.

Control availability depends on the connected firmware and finger profile.
Bringup reports the device identity and rejects incompatible realtime command
layouts. A version label alone is not an accuracy or timing guarantee.

The supplied planning and USD profiles cover stock outward 2FG and standard
inward RG mounting. Additional mesh choices are not automatically supported
device mappings or USD variants; see [descriptions](custom-fingers-and-description.md).

Fake motion tests software interfaces, not contact. Isaac contact depends on
scene, drive and material settings. A passing example is not a calibrated
digital twin for arbitrary materials, custom fingers or payloads.

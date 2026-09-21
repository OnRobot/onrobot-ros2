# Changelog

## 1.1.0

- Add the `onrobot_ros2` metapackage.
- Require Tool API 1.1.0 or newer.
- Add positive force targets to 2FG realtime position and velocity commands,
  including the RViz position buttons and joystick.
- Fix the 2FG Release / open button. Opening waits for Stop acknowledgement
  and retains the force target selected when clicked.
- Improve explicit recovery admission and result tracking across real, fake
  and Isaac backends without replaying previous motion.
- Clean up and improve documentation.

## 1.0.0

- Initial release of OnRobot ROS 2 Jazzy support for 2FG7, 2FG14, RG2 and RG6,
  with conventional and realtime control, RViz, diagnostics and Isaac integration.
- Conventional 3FG15 and 3FG25 support is available for evaluation only.

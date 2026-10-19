# Changelog

## 1.3.0

- Clarify documentation.
- Fix links to shared guides in installed documentation.
- Use the SimReady asset layout for all four grippers, with minimum package versions and compatible interfaces, allowing compatible asset updates. Rebuild the Isaac package to install compatible asset updates.
- Use Isaac Sim 6.1.
- Supply camera framing, lighting and physics-scene defaults in standalone Isaac bridge and hardware-shadow applications.
- Update simulation examples and result validation for the new asset paths.
- Record the SimReady package identity and current file hashes in recorded ROS sessions.
- Use model-appropriate conventional force defaults in bounded HIL checks and reject invalid force limits before connecting.
- Clear retained velocity targets when the Isaac bridge switches to position control or holds after a command timeout.
- Select PhysX explicitly on runtimes with multiple physics engines and fix the 2FG simulation test runner's model selection.
- Wait for fresh Isaac feedback through startup interruptions without requiring a restart. Feedback loss after a motion command still requires explicit recovery.
- Accept velocity-only Isaac commands without requiring a position-force target.
- Disable conventional panel commands during a fault and show the recovery service in both control panels.

## 1.2.0

- Support Tool API 1.2.0.
- Track whether recovery requests are accepted and how they finish for real, fake and
  Isaac backends. Publish `/recovery_controller/state` and keep tracking pending
  requests when controllers are deactivated and activated again. Old motion
  commands are not replayed.
- Add support for aperture velocity in m/s in the standard conventional
  grip action for 2FG7/2FG14.
- Replace the conventional 2FG RViz percentage control with a speed field in m/s for each command.
  Native percentage settings remain available for actions that omit velocity.
- Fix realtime panel target replacement when a previous move is still active.

## 1.1.0

- Add the `onrobot_ros2` metapackage.
- Require Tool API 1.1.0 or newer.
- Add positive force targets to 2FG realtime position and velocity commands,
  including the RViz position buttons and joystick.
- Fix the 2FG Release / open button. Opening waits for Stop acknowledgement
  and retains the force target selected when clicked.
- Improve recovery request acceptance and result tracking across real, fake
  and Isaac backends without replaying previous motion.
- Clean up and improve documentation.

## 1.0.0

- Initial release of OnRobot ROS 2 Jazzy support for 2FG7, 2FG14, RG2 and RG6,
  with conventional and realtime control, RViz, diagnostics and Isaac integration.
- Conventional 3FG15 and 3FG25 support is available for evaluation only.

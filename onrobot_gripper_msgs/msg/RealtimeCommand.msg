# Semantic control modes. Firmware and transport details remain private.
uint8 POSITION=0
uint8 VELOCITY=1
uint8 FORCE_POSITION=2
uint8 FORCE_VELOCITY=3
# Ordered stop command.  Publishers use this on the command topic when a Stop
# must be sequenced relative to surrounding motion commands.
uint8 STOP=255

# Motion commands require a nonzero stamp from the controller's ROS clock.
# The controller checks source age at receipt and before first application;
# source stamps do not identify or arbitrate command publishers. Use
# `header: auto` with `ros2 topic pub` or stamp every streamed message.
std_msgs/Header header
uint8 mode

# Motion is a streaming interface in every mode: refresh commands before the
# configured receipt timeout. A one-shot Stop is always accepted and is
# sequenced ahead of later motion. Position is external task aperture for both
# 2FG and RG. In 2FG POSITION and FORCE_POSITION modes, task_velocity is the
# nonnegative maximum approach velocity. In 2FG VELOCITY and FORCE_VELOCITY
# modes, it is signed velocity.
float64 task_position
float64 task_velocity

# RG velocity mode uses mechanism angular velocity. The shared controller
# selects this field for an RG coordinate profile and task_velocity for 2FG.
float64 mechanism_angular_velocity
float64 force

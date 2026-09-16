uint8 IDLE=0
uint8 POSITION=1
uint8 VELOCITY=2
uint8 FORCE_POSITION=3
uint8 FORCE_VELOCITY=4

std_msgs/Header header
uint8 active_mode
bool realtime_active
# Set after the configured number of consecutive cycle failures. Call the
# realtime stop service to acknowledge the fault before sending new commands.
bool faulted

# Dimension-specific mechanism observations in SI units. Idle/default numeric
# values are not measurements; consumers must inspect the matching validity bit.
bool mechanism_linear_position_valid
float64 mechanism_linear_position
bool mechanism_linear_velocity_valid
float64 mechanism_linear_velocity
bool mechanism_angular_position_valid
float64 mechanism_angular_position
bool mechanism_angular_velocity_valid
float64 mechanism_angular_velocity

# Finger-profile-dependent task observations, independently valid from the
# mechanism state.
bool task_position_valid
float64 task_position
bool task_velocity_valid
float64 task_velocity
bool force_valid
float64 force

uint64 successful_cycles
uint64 failed_cycles
uint64 missed_deadlines
uint64 watchdog_stops
uint64 requested_command_sequence
uint64 applied_command_sequence
uint64 reconnects
float64 last_cycle_duration

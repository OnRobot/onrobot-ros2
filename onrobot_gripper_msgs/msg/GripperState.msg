uint8 DIMENSION_NONE=0
uint8 DIMENSION_LINEAR=1
uint8 DIMENSION_ANGULAR=2

uint8 CONNECTION_UNKNOWN=0
uint8 CONNECTION_DISCONNECTED=1
uint8 CONNECTION_CONNECTING=2
uint8 CONNECTION_IDLE=3
uint8 CONNECTION_ACTIVE=4
uint8 CONNECTION_RECOVERING=5
uint8 CONNECTION_FAULTED=6

uint8 FIRMWARE_UNKNOWN=0
uint8 FIRMWARE_QUALIFIED=1
uint8 FIRMWARE_UNSUPPORTED=2

uint8 MAPPING_MISSING=0
uint8 MAPPING_VALID=1
uint8 MAPPING_INVALID=2

uint8 MAPPING_SOURCE_UNKNOWN=0
uint8 MAPPING_SOURCE_DEVICE_MEASURED=1
uint8 MAPPING_SOURCE_DEVICE_COMPENSATED=2
uint8 MAPPING_SOURCE_DECLARATIVE=3

uint8 FAULT_SOURCE_NONE=0
uint8 FAULT_SOURCE_HOST=1
uint8 FAULT_SOURCE_TRANSPORT=2
uint8 FAULT_SOURCE_PROTOCOL=3
uint8 FAULT_SOURCE_DEVICE=4

std_msgs/Header header
string model
string firmware
string device_profile_revision
string finger_profile_name
string finger_profile_revision

uint64 sample_sequence
# Host receive time expressed in the node's ROS clock. sample_age is measured
# monotonically at publication; header.stamp is the publication time.
builtin_interfaces/Time received_time
builtin_interfaces/Duration sample_age

bool task_aperture_valid
float64 task_aperture
bool task_velocity_valid
float64 task_velocity

# Linear values are SI metres and m/s; angular values are radians and rad/s.
uint8 mechanism_dimension
bool mechanism_position_valid
float64 mechanism_position
bool mechanism_velocity_valid
float64 mechanism_velocity

bool force_valid
float64 force
bool busy
bool grip_detected
# Backend can execute the two 2FG closing-force command modes. This is not a
# force-measurement/accuracy claim. Fake kinematics have no contact model.
bool realtime_force_control_available

# RG safety-switch status. A false validity flag means that this information is
# unavailable; it must not be interpreted as an all-clear indication.
bool safety_status_valid
bool safety_1_pushed
bool safety_1_triggered
bool safety_2_pushed
bool safety_2_triggered
bool safety_dc_error

uint8 connection_state
uint8 active_mode
uint8 firmware_qualification
uint8 mapping_validity
uint8 mapping_source
uint8 fault_source
uint16 fault_code
string fault_message

# Worker/session health counters. These describe physical I/O samples, not ROS
# publication cycles.
uint64 successful_cycles
uint64 failed_cycles
uint64 missed_deadlines
uint64 watchdog_stops
uint64 reconnects
float64 last_cycle_duration

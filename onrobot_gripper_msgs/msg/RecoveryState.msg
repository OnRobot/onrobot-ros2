# Admission is not completion. Match result_sequence to the ID in the recover response.
# Results: 0 none, 1 succeeded, 2 failed, 3 aborted.
# Succeeded confirms backend recovery, not physical standstill or payload safety.
std_msgs/Header header
uint8 ADMISSION_NONE=0
uint8 ADMISSION_ACCEPTED=1
uint8 ADMISSION_REJECTED=2
uint8 RESULT_NONE=0
uint8 RESULT_SUCCEEDED=1
uint8 RESULT_FAILED=2
uint8 RESULT_ABORTED=3
uint8 CODE_NONE=0
uint8 CODE_NOT_CONNECTED=1
uint8 CODE_TRANSPORT=2
uint8 CODE_TIMEOUT=3
uint8 CODE_PROTOCOL=4
uint8 CODE_DEVICE_FAULT=5
uint8 CODE_INVALID_ARGUMENT=6
uint8 CODE_UNSUPPORTED=7
uint8 CODE_BUSY=8
uint8 CODE_CANCELLED=9
uint8 CODE_UNKNOWN=255
bool observation_valid
bool ready
# Outstanding controller request; zero when none is pending.
uint64 request_sequence
# Last request cancelled locally because safety changed before export.
# This is not a backend terminal result; exported requests cannot be cancelled here.
uint64 cancelled_sequence
uint64 admission_sequence
uint8 admission
uint64 active_sequence
uint64 result_sequence
uint8 result
uint8 result_code

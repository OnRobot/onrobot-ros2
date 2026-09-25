// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <array>
#include <cmath>
#include <cstdint>

namespace onrobot_gripper_msgs::recovery {
inline constexpr double version = 2.0;
inline constexpr uint64_t maximum_sequence = 9007199254740991ULL;
enum class Result : uint8_t { None, Succeeded, Failed, Aborted };
enum class Code : uint8_t {
  None,
  NotConnected,
  Transport,
  Timeout,
  Protocol,
  DeviceFault,
  InvalidArgument,
  Unsupported,
  Busy,
  Cancelled,
  Unknown = 255
};
enum Field : std::size_t {
  Version,
  Ready,
  Active,
  ResultSequence,
  Outcome,
  Reason,
  SafetyRequired,
  Count
};
inline constexpr std::array<const char *, Count> names{
    "fault_recovery_protocol_version", "fault_recovery_ready",
    "fault_recovery_active_sequence",  "fault_recovery_result_sequence",
    "fault_recovery_result",           "fault_recovery_result_code",
    "fault_recovery_safety_required"};
inline bool validSequence(double value, bool allow_zero = true) noexcept {
  return std::isfinite(value) && value >= (allow_zero ? 0.0 : 1.0) &&
         value <= static_cast<double>(maximum_sequence) &&
         std::floor(value) == value;
}
// Owner: synchronous hardware read/update/write loop. Controller lifecycle
// changes must not reset retained outcomes. Counters are not operation results.
struct State {
  // Physical RG backends require safety samples; kinematics-only simulation
  // does not model physical switches. This is backend metadata, not an
  // override.
  std::array<double, Count> values{version, 0, 0, 0, 0, 0, 0};
  void accept(uint64_t sequence) noexcept {
    values[Ready] = 0;
    values[Active] = static_cast<double>(sequence);
  }
  void finish(Result result, Code code = Code::None) noexcept {
    if (values[Active] == 0)
      return;
    values[ResultSequence] = values[Active];
    values[Outcome] = static_cast<double>(result);
    values[Reason] = static_cast<double>(code);
    values[Active] = 0;
    values[Ready] = 0;
  }
};
} // namespace onrobot_gripper_msgs::recovery

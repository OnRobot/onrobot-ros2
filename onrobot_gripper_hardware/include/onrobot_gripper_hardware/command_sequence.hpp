#pragma once

#include <cmath>
#include <cstdint>

namespace onrobot_gripper_hardware {

// Command and state interfaces expose sequence identifiers as doubles. Only
// integers through 2^53-1 survive that representation exactly.
// The manager thread uses conventional_command_sequence to track command
// acceptance. +token means pending, including when a lock is unavailable.
// NaN means accepted. -token means rejected or no longer active and blocks stored
// motion outputs until a new positive token arrives. Stop cancels all earlier
// tokens; a replacement must be written after the backend processes Stop.
// Reading NaN alone is not an acknowledgement unless a goal is pending.
// Stop and recovery sequences only use positive tokens.
constexpr double kMaximumExactCommandSequence = 9007199254740991.0;

inline bool decodeCommandSequence(const double value, std::uint64_t &sequence) {
  if (!std::isfinite(value) || value < 1.0 ||
      value > kMaximumExactCommandSequence || std::floor(value) != value) {
    return false;
  }
  sequence = static_cast<std::uint64_t>(value);
  return true;
}

} // namespace onrobot_gripper_hardware

#pragma once

#include <cmath>
#include <cstdint>

namespace onrobot_gripper_hardware {

// Command and state interfaces expose sequence identifiers as doubles. Only
// integers through 2^53-1 survive that representation exactly.
// conventional_command_sequence is a manager-thread admission handshake:
// +token remains pending (including contention), NaN means admitted, -token
// means rejected/retired and fences retained outputs until a new positive event.
// Stop retires every earlier event; a replacement must be exported after Stop
// is consumed. Reading NaN alone is not an acknowledgement without a pending goal.
// Stop/recovery sequences retain their positive-only event contract.
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

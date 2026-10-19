#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace onrobot_gripper_rviz_plugins {

/// Whether a GripperState sample contains a usable force measurement.
///
/// A device with force feedback can still report invalid or old measurements.
/// These cases are separate from RG and 3FG models, which do not supply a
/// measured force in newtons through this interface.
enum class ForceSampleQuality : std::uint8_t {
  Valid,
  Invalid,
  Stale,
  Unavailable,
};

/// @cond INTERNAL
struct ForceHistorySample {
  std::int64_t stamp_ns{0};
  std::int64_t received_steady_ns{0};
  std::uint64_t sequence{0};
  double force_n{0.0};
  bool force_valid{false};
  bool force_fresh{false};
  bool force_supported{false};
  bool grip_detected{false};
  std::uint8_t active_mode{0};
  std::uint8_t connection_state{0};
  std::uint8_t fault_source{0};
  std::uint16_t fault_code{0};
  bool clock_rollback_before{false};
};
/// @endcond

inline ForceSampleQuality forceSampleQuality(
    const ForceHistorySample &i_sample) noexcept {
  if (!i_sample.force_supported) {
    return ForceSampleQuality::Unavailable;
  }
  if (!i_sample.force_valid || !std::isfinite(i_sample.force_n)) {
    return ForceSampleQuality::Invalid;
  }
  if (!i_sample.force_fresh) {
    return ForceSampleQuality::Stale;
  }
  return ForceSampleQuality::Valid;
}

/// @cond INTERNAL
/// A small buffer for plotting force history without duplicate samples.
///
/// A broadcaster can publish faster than the device supplies new measurements.
/// The buffer uses nonzero sample sequence numbers to detect duplicates. If
/// timestamps go backwards, it marks a break in the trace. This prevents a
/// looped recording or simulation reset from joining unrelated measurements.
class ForceHistoryBuffer {
public:
  explicit ForceHistoryBuffer(std::size_t i_max_samples = 1800)
      : m_max_samples(std::max<std::size_t>(1, i_max_samples)) {}

  void clear() noexcept { m_samples.clear(); }

  bool append(ForceHistorySample i_sample) {
    if (!m_samples.empty()) {
      const auto &last = m_samples.back();
      if (i_sample.sequence != 0 &&
          i_sample.sequence == last.sequence) {
        return false;
      }
      if (i_sample.stamp_ns > 0 && last.stamp_ns > 0 &&
          i_sample.stamp_ns < last.stamp_ns) {
        i_sample.clock_rollback_before = true;
      }
    }
    m_samples.push_back(i_sample);
    while (m_samples.size() > m_max_samples) {
      m_samples.pop_front();
    }
    return true;
  }

  const std::deque<ForceHistorySample> &samples() const noexcept {
    return m_samples;
  }

  std::size_t size() const noexcept { return m_samples.size(); }

  bool empty() const noexcept { return m_samples.empty(); }

  /// Mark the latest supported measurement stale once the state stream stops.
  ///
  /// Uses a monotonic clock to measure time since receipt. The ROS clock may
  /// pause, jump backwards during replay, or be unavailable. When publication
  /// stops, the last value must be marked stale even if no new message arrives.
  bool markLatestStaleIfExpired(std::int64_t i_now_steady_ns,
                                std::int64_t i_max_age_ns) noexcept {
    if (m_samples.empty() || i_now_steady_ns <= 0 || i_max_age_ns <= 0) {
      return false;
    }
    auto &latest = m_samples.back();
    if (!latest.force_supported || latest.received_steady_ns <= 0 ||
        !latest.force_fresh ||
        i_now_steady_ns - latest.received_steady_ns <= i_max_age_ns) {
      return false;
    }
    latest.force_fresh = false;
    return true;
  }

private:
  std::size_t m_max_samples;
  std::deque<ForceHistorySample> m_samples;
};
/// @endcond

inline const char *forceSampleQualityName(ForceSampleQuality i_quality) {
  switch (i_quality) {
  case ForceSampleQuality::Valid:
    return "valid";
  case ForceSampleQuality::Invalid:
    return "invalid";
  case ForceSampleQuality::Stale:
    return "stale";
  case ForceSampleQuality::Unavailable:
    return "unavailable";
  }
  return "unknown";
}

inline const char *realtimeModeName(std::uint8_t i_mode) {
  switch (i_mode) {
  case 0:
    return "Idle";
  case 1:
    return "Conventional";
  case 2:
    return "RT position";
  case 3:
    return "RT velocity";
  case 4:
    return "RT force-position";
  case 5:
    return "RT force-velocity";
  default:
    return "Unknown mode";
  }
}

} // namespace onrobot_gripper_rviz_plugins

#pragma once

#include <onrobot_tool_api/parallel_gripper_session.hpp>
#include <rclcpp/rclcpp.hpp>
#include <optional>

namespace onrobot_gripper_hardware {

// One report per rejection/Stop-admission transition. Successful motion or
// lifecycle changes reset it. This avoids per-cycle formatting and log floods.
class RealtimeRejection {
public:
  void reset() { last_.reset(); stop_queued_ = false; }
  void report(const rclcpp::Logger &logger, onrobot::CommandAdmission admission,
              onrobot::CommandAdmission stop, onrobot::Model model,
              const onrobot::ParallelGripperState &state,
              const onrobot::ParallelGripperIdentity &identity,
              double position, double velocity, double force) {
    const bool firmware = admission == onrobot::CommandAdmission::Unsupported &&
        state.firmware_qualification != onrobot::FirmwareQualification::Qualified;
    const unsigned key = static_cast<unsigned>(admission) * 32 +
        static_cast<unsigned>(stop) * 2 + firmware;
    stop_queued_ = stop == onrobot::CommandAdmission::Accepted;
    if (last_ && *last_ == key) return;
    last_ = key;
    const char *stopText = stop_queued_ ? "Stop queued (awaiting acknowledgement)"
                                      : "Stop pending; admission will be retried";
    const auto *profile = onrobot::findModelCapabilityProfile(model);
    if (firmware) {
      RCLCPP_ERROR(logger,
          "Realtime command rejected: unsupported firmware %u.%u.%u for %s; "
          "minimum supported version is %.*s. %s",
          identity.firmware_major, identity.firmware_minor, identity.firmware_build,
          profile->public_name.data(),
          static_cast<int>(profile->minimum_firmware_revision.size()),
          profile->minimum_firmware_revision.data(), stopText);
      return;
    }
    const char *reason = "invalid target or command";
    switch (admission) {
    case onrobot::CommandAdmission::Unsupported: reason = "unsupported control mode"; break;
    case onrobot::CommandAdmission::NotActive: reason = "session inactive or recovering"; break;
    case onrobot::CommandAdmission::Faulted: reason = "device/session fault; inspect diagnostics and recover"; break;
    case onrobot::CommandAdmission::InvalidArgument:
      reason = "invalid aperture, velocity or force target; check the supported ranges"; break;
    default: break;
    }
    RCLCPP_ERROR(logger,
        "Realtime command rejected: %s (position=%g m, velocity=%g, force=%g N). %s",
        reason, position, velocity, force, stopText);
  }
private:
  std::optional<unsigned> last_;
  bool stop_queued_{false};
};
} // namespace onrobot_gripper_hardware

#pragma once

#include <array>
#include <onrobot_tool_api/parallel_gripper_session.hpp>
#include <onrobot_gripper_msgs/diagnostic_interfaces.hpp>

namespace onrobot_gripper_hardware {

/// Populate freshly reset diagnostic interfaces from a coherent session copy.
inline void exportIdentity(
    const onrobot::ParallelGripperIdentity &identity,
    const onrobot::ParallelGripperState &state,
    std::array<double, onrobot_gripper_msgs::kDiagnosticInterfaceCount> &values) noexcept {
  using I = onrobot_gripper_msgs::DiagnosticInterface;
  const auto put = [&](I field, double value) {
    values[onrobot_gripper_msgs::diagnosticIndex(field)] = value;
  };
  const bool valid = identity.valid &&
      state.session_state != onrobot::SessionState::Faulted &&
      state.session_state != onrobot::SessionState::Recovering;
  put(I::IdentityValid, valid ? 1.0 : 0.0);
  if (!valid) { return; }
  put(I::ProductCode, identity.product_code);
  put(I::FirmwareMajor, identity.firmware_major);
  put(I::FirmwareMinor, identity.firmware_minor);
  put(I::FirmwareBuild, identity.firmware_build);
  put(I::BoardRevisionsValid, identity.board_revisions_valid ? 1.0 : 0.0);
  if (identity.board_revisions_valid) {
    put(I::ProductRevision, identity.product_revision);
    put(I::HardwareRevision, identity.hardware_revision);
    put(I::PcbRevision, identity.pcb_revision);
  }
  put(I::FirmwareCrcValid, identity.firmware_crc_valid ? 1.0 : 0.0);
  if (identity.firmware_crc_valid) { put(I::FirmwareCrc, identity.firmware_crc); }
  put(I::FirmwareSourceValid, identity.firmware_source_valid ? 1.0 : 0.0);
  if (identity.firmware_source_valid) {
    for (std::size_t index = 0; index < identity.firmware_git_hash_words.size(); ++index) {
      put(static_cast<I>(onrobot_gripper_msgs::diagnosticIndex(I::FirmwareGitWord0) + index),
          identity.firmware_git_hash_words[index]);
    }
  }
}

} // namespace onrobot_gripper_hardware

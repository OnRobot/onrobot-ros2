// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <onrobot_gripper_msgs/fault_recovery_protocol.hpp>
#include <onrobot_tool_api/parallel_gripper_session.hpp>

namespace onrobot_gripper_hardware {
inline onrobot_gripper_msgs::recovery::Code
recoveryCode(onrobot::ErrorCode code) {
  using C = onrobot_gripper_msgs::recovery::Code;
  switch (code) {
  case onrobot::ErrorCode::None:
    return C::None;
  case onrobot::ErrorCode::NotConnected:
    return C::NotConnected;
  case onrobot::ErrorCode::Transport:
    return C::Transport;
  case onrobot::ErrorCode::Timeout:
    return C::Timeout;
  case onrobot::ErrorCode::Protocol:
    return C::Protocol;
  case onrobot::ErrorCode::DeviceFault:
    return C::DeviceFault;
  case onrobot::ErrorCode::InvalidArgument:
    return C::InvalidArgument;
  case onrobot::ErrorCode::Unsupported:
    return C::Unsupported;
  case onrobot::ErrorCode::Busy:
    return C::Busy;
  case onrobot::ErrorCode::Cancelled:
    return C::Cancelled;
  }
  return C::Unknown;
}
inline bool finishRecovery(onrobot_gripper_msgs::recovery::State &state,
                           uint64_t sdk_sequence,
                           const onrobot::ParallelGripperRecoveryState &sdk) {
  using R = onrobot_gripper_msgs::recovery::Result;
  if (sdk_sequence == 0 || sdk.result_sequence != sdk_sequence ||
      state.values[onrobot_gripper_msgs::recovery::Active] == 0)
    return false;
  switch (sdk.result) {
  case onrobot::RecoveryResult::Succeeded:
    state.finish(R::Succeeded);
    break;
  case onrobot::RecoveryResult::Failed:
    state.finish(R::Failed, recoveryCode(sdk.result_code));
    break;
  case onrobot::RecoveryResult::Aborted:
    state.finish(R::Aborted, recoveryCode(sdk.result_code));
    break;
  default:
    return false;
  }
  return true;
}
} // namespace onrobot_gripper_hardware

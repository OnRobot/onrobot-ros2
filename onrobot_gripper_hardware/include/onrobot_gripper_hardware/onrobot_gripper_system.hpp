#pragma once

#include "onrobot_gripper_hardware/command_stop_gate.hpp"

#include <chrono>
#include <cstdint>
#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>

#include <onrobot_tool_api/parallel_gripper_session.hpp>
#include <onrobot_gripper_msgs/diagnostic_interfaces.hpp>

#include "onrobot_gripper_hardware/gripper_semantics.hpp"

namespace onrobot_gripper_hardware {

/// @brief Standard ros2_control hardware adapter for one OnRobot 2FG gripper.
///
/// `grip_stroke` is the finger-profile task aperture. The state-only
/// `finger_stroke` is the physical one-jaw articulation used by URDF and USD;
/// its measured-position interfaces retain the invariant total raw mechanism
/// stroke. Device I/O and mode ownership live in ParallelGripperSession.
class OnRobotGripperSystem final : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OnRobotGripperSystem)

  ~OnRobotGripperSystem() override;

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams &i_params)
      override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;

  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &i_previousState) override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &i_previousState) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &i_previousState) override;

  hardware_interface::return_type
  read(const rclcpp::Time &i_time, const rclcpp::Duration &i_period) override;

  hardware_interface::return_type
  write(const rclcpp::Time &i_time, const rclcpp::Duration &i_period) override;

private:
  CommandStopGate command_stop_gate_;
  bool has_command_interface(const std::string &i_name) const;
  bool has_state_interface(const std::string &i_name) const;
  bool has_state_interface(const hardware_interface::ComponentInfo &i_joint,
                           const std::string &i_name) const;
  bool parse_parameters();
  bool apply_snapshot(const onrobot::ParallelGripperState &i_image);
  void reset_read_cache();

  hardware_interface::HardwareInfo m_info;
  std::unique_ptr<onrobot::ParallelGripperSession> m_session;
  std::unique_ptr<GripperKinematics> m_kinematics;
  onrobot::ParallelGripperState m_cachedState{};
  onrobot::ParallelGripperIdentity m_cachedIdentity{};

  onrobot::Model m_model{onrobot::Model::TwoFG7};
  onrobot::ModbusConfig m_connection;
  onrobot::Timeout m_transportTimeout{};
  std::string m_jointName{"grip_stroke"};
  std::string m_fingerJointName{"finger_stroke"};
  double m_fingerJointUpperM{0.019};
  double m_rawLinearMinimumMm{1.0};
  double m_rawLinearMaximumMm{39.0};
  double m_defaultForceN{20.0};
  double m_speedPercent{50.0};
  std::optional<uint16_t> m_supplyPowerW{48U};
  std::chrono::milliseconds m_pollPeriod{20};
  std::chrono::microseconds m_realtimePeriod{2000};
  std::chrono::milliseconds m_realtimeTimeout{100};
  uint32_t m_maximumConsecutiveFailures{3};

  double m_positionCommand{0.0};
  double m_effortCommand{0.0};
  double m_positionState{0.0};
  double m_velocityState{0.0};
  double m_effortState{0.0};
  double m_fingerPositionState{0.0};
  double m_fingerVelocityState{0.0};
  double m_mechanismMeasuredPositionState{0.0};
  double m_mechanismMeasuredVelocityState{0.0};
  double m_mechanismPositionValidState{0.0};
  double m_mechanismVelocityValidState{0.0};
  double m_taskPositionValidState{0.0};
  double m_minimumTaskApertureState{0.0};
  double m_maximumTaskApertureState{0.0};
  double m_forceValidState{0.0};
  double m_busyState{0.0};
  double m_gripDetectedState{0.0};
  double m_sessionActiveModeState{0.0};
  double m_connectionState{1.0};
  double m_sessionFaultedState{0.0};
  double m_faultCodeState{0.0};
  double m_firmwareQualificationState{0.0};
  double m_realtimeForceControlAvailable{0.0};
  double m_sampleSequenceState{0.0};
  double m_sampleAgeState{0.0};
  double m_requestedCommandSequenceState{0.0};
  double m_appliedCommandSequenceState{0.0};
  double m_successfulCyclesState{0.0};
  double m_failedCyclesState{0.0};
  double m_missedDeadlinesState{0.0};
  double m_watchdogStopsState{0.0};
  double m_reconnectsState{0.0};
  double m_lastCycleDurationState{0.0};
  std::array<double, onrobot_gripper_msgs::kDiagnosticInterfaceCount>
      m_diagnosticStates{};
  double m_realtimeModeCommand{0.0};
  double m_realtimeTaskPositionCommand{0.0};
  double m_realtimeTaskVelocityCommand{0.0};
  double m_realtimeForceCommand{0.0};
  double m_realtimeSequenceCommand{0.0};
  double m_faultRecoverySequenceCommand{0.0};
  double m_stopSequenceCommand{0.0};
  double m_conventionalSequenceCommand{0.0};
  double m_conventionalSpeedPercentCommand{50.0};
  bool m_forceConventionalCommand{false};
  bool m_hasEffortCommand{false};
  bool m_hasFingerJoint{false};
  double m_lastTaskPositionM{0.0};
  std::chrono::steady_clock::time_point m_lastReceivedAt{};
  uint64_t m_lastSampleSequence{0};
  double m_lastSentPositionM{0.0};
  double m_lastSentEffortN{0.0};
  bool m_commandSent{false};
  bool m_hasRejectedCommand{false};
  double m_lastRejectedPositionM{0.0};
  uint64_t m_lastRealtimeSequence{0};
  uint64_t m_lastFaultRecoverySequence{0};
  uint64_t m_pendingStopSessionSequence{0};
  uint64_t m_recoveryReconnectsAtRequest{0};
  double m_retiredPositionCommand{0.0};
  bool m_recoveryPending{false};
  bool m_conventionalRecoveryGate{false};
  bool m_stopPending{false};
};

} // namespace onrobot_gripper_hardware

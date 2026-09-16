#pragma once

#include "onrobot_gripper_hardware/command_stop_gate.hpp"

#include <chrono>
#include <cstdint>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <onrobot_tool_api/parallel_gripper_session.hpp>
#include <onrobot_gripper_msgs/diagnostic_interfaces.hpp>
#include <rclcpp/macros.hpp>

#include "onrobot_gripper_hardware/gripper_semantics.hpp"

namespace onrobot_gripper_hardware {

/// @brief ros2_control adapter for conventional and realtime RG2/RG6 commands.
///
/// `grip_stroke` is the measured fingertip-compensated aperture in metres and
/// is the only actuator command coordinate. The optional `finger_joint` is a
/// derived, state-only URDF linkage coordinate for visualization. A dedicated
/// worker remains the sole owner of blocking Modbus operations.
class OnRobotRgSystem final : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OnRobotRgSystem)

  ~OnRobotRgSystem() override;
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams &i_params)
      override;
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &,
                                       const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &,
                                        const rclcpp::Duration &) override;

private:
  CommandStopGate command_stop_gate_;
  bool parseParameters();
  bool hasCommand(const hardware_interface::ComponentInfo &i_joint,
                  const std::string &i_name) const;
  bool hasState(const hardware_interface::ComponentInfo &i_joint,
                const std::string &i_name) const;
  bool applySnapshot(const onrobot::ParallelGripperState &i_image);
  void resetDerivativeHistory();
  void resetReadCache();

  hardware_interface::HardwareInfo m_info;
  std::unique_ptr<onrobot::ParallelGripperSession> m_session;
  std::unique_ptr<RgCadKinematics> m_cadKinematics;
  bool m_visualGapFromAperture{false};
  onrobot::ParallelGripperState m_cachedState{};
  onrobot::ParallelGripperIdentity m_cachedIdentity{};
  onrobot::Model m_model{onrobot::Model::RG2};
  onrobot::ModbusConfig m_connection;
  onrobot::Timeout m_transportTimeout{};
  onrobot::RgMotionLimits m_limits;
  std::string m_apertureJointName{"grip_stroke"};
  std::string m_visualJointName{"finger_joint"};
  double m_defaultForceN{10.0};
  double m_visualFingerJointUpperRad{1.3136012652574625};
  std::chrono::milliseconds m_pollPeriod{20};
  std::chrono::microseconds m_realtimePeriod{2000};
  std::chrono::milliseconds m_realtimeCommandTimeout{100};
  uint32_t m_maximumConsecutiveFailures{3};

  double m_positionCommand{0.0};
  double m_effortCommand{0.0};
  double m_realtimeModeCommand{0.0};
  double m_realtimeTaskPositionCommand{0.0};
  double m_realtimeMechanismAngularVelocityCommand{0.0};
  double m_realtimeForceCommand{0.0};
  double m_realtimeSequenceCommand{0.0};
  double m_faultRecoverySequenceCommand{0.0};
  double m_stopSequenceCommand{0.0};
  double m_conventionalSequenceCommand{0.0};
  bool m_forceConventionalCommand{false};
  double m_positionState{0.0};
  double m_velocityState{0.0};
  double m_effortState{0.0};
  double m_fingerAngleState{0.0};
  double m_fingerVelocityState{0.0};
  double m_measuredAngularPositionState{0.0};
  double m_measuredAngularVelocityState{0.0};
  double m_mechanismAngularPositionValidState{0.0};
  double m_mechanismAngularVelocityValidState{0.0};
  double m_taskPositionValidState{0.0};
  double m_forceValidState{0.0};
  double m_busyState{0.0};
  double m_gripDetectedState{0.0};
  double m_safetyStatusValidState{0.0};
  double m_safety1PushedState{0.0};
  double m_safety1TriggeredState{0.0};
  double m_safety2PushedState{0.0};
  double m_safety2TriggeredState{0.0};
  double m_safetyDcErrorState{0.0};
  double m_activeModeState{0.0};
  double m_connectionState{1.0};
  double m_faultedState{0.0};
  double m_faultCodeState{0.0};
  double m_firmwareQualificationState{0.0};
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
  double m_minimumTaskApertureState{0.0};
  double m_maximumTaskApertureState{0.0};
  bool m_hasEffortCommand{false};
  bool m_hasVisualJoint{false};
  bool m_hasRejectedCommand{false};
  double m_lastRejectedPositionM{0.0};
  double m_lastTaskPositionM{0.0};
  double m_lastFingerAngleRad{0.0};
  std::chrono::steady_clock::time_point m_lastReceivedAt{};
  uint64_t m_lastSampleSequence{0};
  uint64_t m_lastRealtimeSequence{0};
  uint64_t m_lastFaultRecoverySequence{0};
  uint64_t m_pendingStopSessionSequence{0};
  uint64_t m_recoveryReconnectsAtRequest{0};
  double m_retiredPositionCommand{0.0};
  bool m_recoveryPending{false};
  bool m_conventionalRecoveryGate{false};
  bool m_stopPending{false};
  double m_lastSentPositionM{0.0};
  double m_lastSentEffortN{0.0};
  bool m_commandSent{false};
};

} // namespace onrobot_gripper_hardware

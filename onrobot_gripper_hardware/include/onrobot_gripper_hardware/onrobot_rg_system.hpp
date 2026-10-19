#pragma once
#include "onrobot_gripper_hardware/realtime_rejection.hpp"
#include <onrobot_gripper_msgs/fault_recovery_protocol.hpp>

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
/// `grip_stroke` reports the opening at the fingertips in metres, including
/// fingertip offset, and receives motion commands. The optional `finger_joint`
/// reports the calculated URDF linkage angle for visualization and accepts no
/// commands. A dedicated worker handles all blocking device communication.
class OnRobotRgSystem final : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OnRobotRgSystem)

  ~OnRobotRgSystem() override;
  /// @brief Validate the RG model, hardware parameters and joint declarations.
  ///
  /// Reads the supplied hardware description without connecting to the device.
  /// Returns ERROR for an unsupported model or invalid configuration.
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams &i_params)
      override;
  /// @brief Expose aperture, mechanism angle, visual joint and device state.
  ///
  /// Aperture uses metres and angles use radians. Visual joint mapping does not
  /// change the raw mechanism feedback. Consumers must check validity flags.
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;
  /// @brief Expose shared RG motion, Stop and recovery command handles.
  ///
  /// Only configured optional interfaces are exported. Shared handles allow an
  /// action cancellation to pass Stop while the controller owns the joint.
  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;
  /// @brief Connect the Tool API session and check the initial RG state.
  ///
  /// Applies the configured motion limits and communication timing. This
  /// lifecycle call performs device communication. Returns ERROR for pending
  /// recovery, connection failure or invalid state.
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;
  /// @brief Activate the session and initialize targets from measured aperture.
  ///
  /// Requires a configured session and no pending recovery. Returns ERROR if
  /// activation or the initial state check fails. Earlier targets are not sent
  /// again.
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;
  /// @brief Request Stop and shut down the session worker.
  ///
  /// Waits for worker shutdown and retains any recovery result. Returns ERROR
  /// if a pending recovery result cannot be read and finalized.
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;
  /// @brief Copy RG feedback into ROS state and calculate visual joint
  /// positions.
  ///
  /// Reads the cached session snapshot without device I/O. Returns ERROR if no
  /// session exists. Check validity flags before using individual
  /// measurements.
  hardware_interface::return_type read(const rclcpp::Time &,
                                       const rclcpp::Duration &) override;
  /// @brief Queue RG motion, Stop or recovery through the Tool API session.
  ///
  /// Does not wait for the command lock or perform device I/O. A busy lock
  /// defers work. An OK return does not confirm device acceptance or completed
  /// motion; inspect the state interfaces.
  hardware_interface::return_type write(const rclcpp::Time &,
                                        const rclcpp::Duration &) override;

  /// @cond INTERNAL
private:
  onrobot_gripper_msgs::recovery::State recovery_state_;
  bool recovery_lifecycle_active_{false};
  onrobot::ParallelGripperRecoveryState m_cachedRecovery{};
  uint64_t m_recoverySdkSequence{0};
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
  double m_faultRecoverySequenceAckState{0.0};
  double m_faultRecoveryAdmissionState{0.0};
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
  RealtimeRejection m_realtimeRejection;
  uint64_t m_lastFaultRecoverySequence{0};
  uint64_t m_pendingStopSessionSequence{0};
  double m_retiredPositionCommand{0.0};
  bool m_recoveryPending{false};
  bool m_conventionalRecoveryGate{false};
  bool m_stopPending{false};
  double m_lastSentPositionM{0.0};
  double m_lastSentEffortN{0.0};
  bool m_commandSent{false};
  /// @endcond
};

} // namespace onrobot_gripper_hardware

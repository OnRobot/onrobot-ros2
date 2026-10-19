#pragma once

#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <onrobot_tool_api/three_finger_gripper.hpp>
#include <onrobot_gripper_msgs/diagnostic_interfaces.hpp>
#include <rclcpp/macros.hpp>

namespace onrobot_gripper_hardware {

/// ros2_control adapter for controlling 3FG15 and 3FG25 by diameter.
/// The command joint uses diameter in metres. The separate `finger_angle`
/// state converts device feedback to the URDF joint angle for visualization.
/// Custom state interfaces report the current external diameter limits in metres.
class OnRobotThreeFingerSystem final
    : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OnRobotThreeFingerSystem)
  ~OnRobotThreeFingerSystem() override;
  /// @brief Validate the 3FG model, joints and hardware parameters.
  ///
  /// Reads the supplied hardware description without connecting. Returns ERROR
  /// for unsupported models or invalid configuration. 3FG support is for
  /// evaluation.
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
      override;
  /// @brief Expose diameter, finger angle, validity and device diagnostics.
  ///
  /// Diameter uses metres, angle uses radians and the native force value is a
  /// percentage, not newtons.
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;
  /// @brief Expose conventional diameter and force-percentage command handles.
  ///
  /// These interfaces do not provide realtime control.
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;
  /// @brief Connect to the 3FG and check the configured fingertip position.
  ///
  /// Returns ERROR on connection or initial-state failure, including a
  /// fingertip-position mismatch. Unavailable diagnostics produce a warning
  /// without failing configuration.
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;
  /// @brief Stop earlier motion and start the communication worker.
  ///
  /// Reads the current diameter and initializes the target from it. Returns
  /// ERROR without a configured gripper or if activation communication fails.
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;
  /// @brief Request Stop and join the communication worker.
  ///
  /// Waits for worker shutdown before returning.
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;
  /// @brief Copy cached worker feedback and diagnostics into ROS state.
  ///
  /// Marks unavailable state invalid instead of treating old values as current.
  /// Device communication runs in the worker.
  hardware_interface::return_type read(const rclcpp::Time &,
                                       const rclcpp::Duration &) override;
  /// @brief Validate diameter and force-percentage targets and queue new
  /// commands.
  ///
  /// Sends work to the communication worker rather than performing device I/O
  /// here.
  hardware_interface::return_type write(const rclcpp::Time &,
                                        const rclcpp::Duration &) override;

  /// @cond INTERNAL
private:
  struct Command {
    double diameter_m{0};
    double force_percent{0};
    uint64_t seq{0};
    bool available{false};
    bool stop{false};
  };
  struct State {
    double diameter_m{0};
    double finger_angle_rad{0};
    double diameter_velocity_m_s{0};
    double angle_velocity_rad_s{0};
    double minimum_external_diameter_m{0};
    double maximum_external_diameter_m{0};
    uint16_t fingertip_position{0};
    std::chrono::steady_clock::time_point stamp{};
    onrobot::ThreeFingerDiagnostics diagnostics{};
    std::chrono::steady_clock::time_point diagnostics_stamp{};
    uint64_t diagnostics_sequence{0};
    bool diagnostics_valid{false};
    bool valid{false};
    bool velocity_valid{false};
  };
  bool parseParameters();
  bool hasCommand(const hardware_interface::ComponentInfo &,
                  const std::string &) const;
  bool hasState(const hardware_interface::ComponentInfo &,
                const std::string &) const;
  void workerLoop();
  void publishState(const onrobot::ThreeFingerState &);
  void publishDiagnostics(const onrobot::ThreeFingerDiagnostics &);
  void invalidate() noexcept;
  void requestStop() noexcept;
  void stopWorker() noexcept;

  hardware_interface::HardwareInfo m_info;
  std::unique_ptr<onrobot::ThreeFingerGripper> m_gripper;
  onrobot::Model m_model{onrobot::Model::ThreeFG25};
  onrobot::ModbusConfig m_connection;
  std::string m_diameterJoint{"grip_diameter"};
  std::string m_fingerJoint{"finger_angle"};
  double m_defaultForcePercent{20};
  std::chrono::milliseconds m_pollPeriod{20};
  double m_diameterCommand{0};
  double m_forceCommand{0};
  double m_diameterState{0};
  double m_diameterVelocity{0};
  double m_fingerAngleState{0};
  double m_fingerAngleVelocity{0};
  double m_minimumExternalDiameterState{0};
  double m_maximumExternalDiameterState{0};
  double m_taskPositionValidState{0};
  double m_forceValidState{0};
  double m_busyState{0};
  double m_gripDetectedState{0};
  double m_activeModeState{0};
  double m_connectionState{0};
  double m_faultedState{0};
  double m_faultCodeState{0};
  double m_firmwareQualificationState{0};
  double m_sampleSequenceState{0};
  double m_sampleAgeState{0};
  double m_successfulCyclesState{0};
  double m_failedCyclesState{0};
  double m_missedDeadlinesState{0};
  double m_watchdogStopsState{0};
  double m_reconnectsState{0};
  double m_lastCycleDurationState{0};
  double m_measuredFingerAngleState{0};
  double m_measuredFingerVelocityState{0};
  double m_mechanismPositionValidState{0};
  double m_mechanismVelocityValidState{0};
  double m_urdfFingerAngleScale{-1.0};
  double m_urdfFingerAngleOffset{2.70526};
  double m_urdfFingerAngleMinimum{0.0};
  double m_urdfFingerAngleMaximum{2.70526};
  int m_expectedFingertipPosition{3};
  double m_lastRejectedDiameterCommand{0};
  bool m_hasRejectedDiameterCommand{false};
  bool m_hasForceCommand{false};
  std::array<double, onrobot_gripper_msgs::kDiagnosticInterfaceCount>
      m_diagnosticStates{};
  std::mutex m_commandMutex;
  Command m_command;
  mutable std::mutex m_stateMutex;
  State m_state;
  std::atomic_bool m_shutdown{false};
  std::thread m_worker;
  /// @endcond
};

} // namespace onrobot_gripper_hardware

#pragma once
#include <onrobot_gripper_msgs/fault_recovery_protocol.hpp>

#include "onrobot_gripper_hardware/command_stop_gate.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <onrobot_gripper_msgs/diagnostic_interfaces.hpp>
#include <rclcpp/macros.hpp>

#include "onrobot_gripper_hardware/gripper_semantics.hpp"

namespace onrobot_gripper_hardware {

/// Fake hardware for testing parallel-gripper control without a device.
///
/// Converts task aperture to the physical finger or linkage joint shown in the
/// URDF. It accepts 2FG realtime commands through the same interfaces as the
/// real backend. The same controller access rules and visualization therefore
/// apply to both backends, and fake motion is repeatable.
class OnRobotParallelGripperFakeSystem final
    : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OnRobotParallelGripperFakeSystem)

  /// @brief Initialize model coordinates and fake motion
  /// settings.
  ///
  /// Reads the supplied hardware description and returns ERROR for unsupported
  /// models, invalid parameters or incomplete interfaces.
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
      override;
  /// @brief Expose simulated joint positions and model state.
  ///
  /// Uses the same interface names as the real backend. The fake backend does
  /// not measure contact force.
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;
  /// @brief Expose shared motion, Stop and recovery handles for fake hardware.
  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;
  /// @brief Initialize a hold at the current simulated position.
  ///
  /// Clears earlier command sequences and velocity targets.
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;
  /// @brief Disable simulated motion and keep the current position.
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;
  /// @brief Refresh fake-backend recovery readiness.
  ///
  /// Motion advances in write(), not in this call.
  hardware_interface::return_type
  read(const rclcpp::Time &, const rclcpp::Duration &) override;
  /// @brief Process commands and advance the simulated motion.
  ///
  /// Uses period in seconds to update motion. Handles Stop, recovery and
  /// configured stall behavior without physical contact simulation.
  hardware_interface::return_type
  write(const rclcpp::Time &, const rclcpp::Duration &period) override;

  /// @cond INTERNAL
private:
  onrobot_gripper_msgs::recovery::State recovery_state_;
  bool recovery_lifecycle_active_{false};
  CommandStopGate command_stop_gate_;
  bool hasState(const hardware_interface::ComponentInfo &joint,
                const std::string &name) const;
  bool hasCommand(const hardware_interface::ComponentInfo &joint,
                  const std::string &name) const;
  void updateDerivedState(double period_s);
  void updateDiagnosticStates();

  hardware_interface::HardwareInfo info_;
  std::string model_;
  bool is_2fg_{false};
  bool has_effort_{false};
  std::string visual_joint_;
  double task_min_m_{0.0};
  double task_max_m_{0.0};
  double visual_joint_upper_{0.0};
  double raw_min_m_{0.001};
  double raw_max_m_{0.039};
  double geometry_aperture_at_zero_m_{0.0};
  double conventional_min_force_n_{0.0};
  double conventional_max_force_n_{0.0};
  double conventional_default_force_n_{0.0};
  double fake_motion_speed_m_s_{0.0};
  bool fake_stall_{false};
  std::unique_ptr<RgCadKinematics> rg_kinematics_;

  double position_command_{0.0};
  double effort_command_{0.0};
  double realtime_mode_command_{0.0};
  double realtime_task_position_command_{0.0};
  double realtime_task_velocity_command_{0.0};
  double realtime_mechanism_angular_velocity_command_{0.0};
  double realtime_force_command_{0.0};
  double realtime_sequence_command_{0.0};
  double fault_recovery_sequence_command_{0.0};
  double stop_sequence_command_{0.0};
  double conventional_sequence_command_{0.0};
  double conventional_speed_percent_command_{50.0};
  uint64_t last_realtime_sequence_{0};
  uint64_t last_fault_recovery_sequence_{0};
  bool realtime_active_{false};
  int realtime_mode_{-1};
  bool force_conventional_command_{false};

  double position_state_{0.0};
  double velocity_state_{0.0};
  double effort_state_{0.0};
  double visual_position_state_{0.0};
  double visual_velocity_state_{0.0};
  double measured_mechanism_position_state_{0.0};
  double measured_mechanism_velocity_state_{0.0};
  double measured_mechanism_angular_position_state_{0.0};
  double measured_mechanism_angular_velocity_state_{0.0};
  double task_position_valid_state_{1.0};
  double force_valid_state_{1.0};
  double busy_state_{0.0};
  double grip_detected_state_{0.0};
  double safety_status_valid_state_{0.0};
  double safety_1_pushed_state_{0.0};
  double safety_1_triggered_state_{0.0};
  double safety_2_pushed_state_{0.0};
  double safety_2_triggered_state_{0.0};
  double safety_dc_error_state_{0.0};
  double mechanism_position_valid_state_{1.0};
  double mechanism_velocity_valid_state_{1.0};
  double active_mode_state_{0.0};
  double connection_state_{3.0};
  double faulted_state_{0.0};
  double fault_recovery_sequence_ack_state_{0.0};
  double fault_recovery_admission_state_{0.0};
  double fault_code_state_{0.0};
  double firmware_qualification_state_{0.0};
  double realtime_force_control_available_{0.0};
  double sample_sequence_state_{0.0};
  double sample_age_state_{0.0};
  double requested_command_sequence_state_{0.0};
  double applied_command_sequence_state_{0.0};
  double successful_cycles_state_{0.0};
  double failed_cycles_state_{0.0};
  double missed_deadlines_state_{0.0};
  double watchdog_stops_state_{0.0};
  double reconnects_state_{0.0};
  double last_cycle_duration_state_{0.0};
  double minimum_task_aperture_state_{0.0};
  double maximum_task_aperture_state_{0.0};
  std::array<double, onrobot_gripper_msgs::kDiagnosticInterfaceCount>
      diagnostic_states_{};
  /// @endcond
};

} // namespace onrobot_gripper_hardware

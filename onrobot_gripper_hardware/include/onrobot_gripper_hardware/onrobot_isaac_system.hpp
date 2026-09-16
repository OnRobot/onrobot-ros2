#pragma once

#include "onrobot_gripper_hardware/command_stop_gate.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <onrobot_gripper_msgs/diagnostic_interfaces.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "onrobot_gripper_hardware/gripper_semantics.hpp"

namespace onrobot_gripper_hardware {

/// ros2_control adapter between OnRobot task coordinates and an Isaac joint.
///
/// The simulator only contains physical articulation DOFs. This adapter keeps
/// the public OnRobot aperture and raw-mechanism state contract on the ROS side
/// and maps it to the model's single driven physical DOF in Isaac Sim.
/// Feedback samples must carry a positive, advancing source timestamp in their
/// header. Source timestamps order one stream; receipt time separately bounds
/// freshness, so timestamps from different clocks are never compared.
class OnRobotIsaacSystem final : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OnRobotIsaacSystem)

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
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
  bool hasState(const hardware_interface::ComponentInfo &joint,
                const std::string &name) const;
  bool hasCommand(const hardware_interface::ComponentInfo &joint,
                  const std::string &name) const;
  void receiveJointState(const sensor_msgs::msg::JointState &message);
  void publishPosition(double task_position_m);
  void publishVelocity(double task_velocity_m_s);
  void publishPhysicalCommand(double physical_position_m,
                              double physical_velocity_m_s);
  void updateDerivedState(double physical_position_m,
                          double physical_velocity_m_s);
  void updateDiagnosticStates();
  bool currentFeedbackAvailable();

  hardware_interface::HardwareInfo info_;
  std::string task_joint_;
  std::string physical_joint_;
  std::string isaac_joint_{"finger_stroke"};
  std::string command_topic_{"isaac_joint_commands"};
  std::string state_topic_{"isaac_joint_states"};
  double task_min_m_{0.0};
  double task_max_m_{0.073};
  double physical_min_m_{0.0};
  double physical_max_m_{0.019};
  double raw_min_m_{0.001};
  double raw_max_m_{0.039};
  std::chrono::milliseconds state_timeout_{250};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      state_subscription_;
  std::mutex state_mutex_;
  double received_physical_position_{0.0};
  double received_physical_velocity_{std::numeric_limits<double>::quiet_NaN()};
  std::chrono::steady_clock::time_point received_at_{};
  bool received_state_{false};
  bool ever_received_state_{false};
  bool received_velocity_valid_{false};
  uint64_t received_sample_sequence_{0};
  int64_t last_source_stamp_ns_{0};
  bool source_stamp_valid_{false};
  bool feedback_fault_pending_{false};
  bool source_reset_pending_{false};
  std::chrono::steady_clock::time_point applied_feedback_received_at_{};
  bool applied_feedback_available_{false};
  bool active_{false};
  bool feedback_ready_{false};
  bool feedback_seeded_{false};
  bool feedback_loss_stop_pending_{false};
  bool feedback_loss_stop_sent_{false};
  bool conventional_recovery_gate_{false};
  double retired_position_command_{std::numeric_limits<double>::quiet_NaN()};
  bool rg_model_{false};
  std::unique_ptr<RgVisualKinematics> rg_kinematics_;

  double position_command_{0.0};
  double effort_command_{0.0};
  double realtime_mode_command_{0.0};
  double realtime_task_position_command_{0.0};
  double realtime_task_velocity_command_{0.0};
  double realtime_force_command_{0.0};
  double realtime_sequence_command_{0.0};
  double fault_recovery_sequence_command_{0.0};
  double stop_sequence_command_{0.0};
  double conventional_sequence_command_{0.0};
  uint64_t last_realtime_sequence_{0};
  uint64_t last_fault_recovery_sequence_{0};
  int realtime_mode_{-1};
  bool realtime_active_{false};
  bool force_conventional_command_{false};

  double position_state_{std::numeric_limits<double>::quiet_NaN()};
  double velocity_state_{std::numeric_limits<double>::quiet_NaN()};
  double effort_state_{std::numeric_limits<double>::quiet_NaN()};
  double physical_position_state_{std::numeric_limits<double>::quiet_NaN()};
  double physical_velocity_state_{std::numeric_limits<double>::quiet_NaN()};
  double measured_mechanism_position_state_{
      std::numeric_limits<double>::quiet_NaN()};
  double measured_mechanism_velocity_state_{
      std::numeric_limits<double>::quiet_NaN()};
  double task_position_valid_state_{0.0};
  double force_valid_state_{0.0};
  double busy_state_{0.0};
  double grip_detected_state_{0.0};
  double safety_status_valid_state_{0.0};
  double safety_1_pushed_state_{0.0};
  double safety_1_triggered_state_{0.0};
  double safety_2_pushed_state_{0.0};
  double safety_2_triggered_state_{0.0};
  double safety_dc_error_state_{0.0};
  double mechanism_position_valid_state_{0.0};
  double mechanism_velocity_valid_state_{0.0};
  double active_mode_state_{0.0};
  double connection_state_{2.0};
  double faulted_state_{0.0};
  double fault_code_state_{0.0};
  double firmware_qualification_state_{0.0};
  double realtime_force_control_available_{0.0};
  double sample_sequence_state_{0.0};
  double sample_age_state_{std::numeric_limits<double>::quiet_NaN()};
  double requested_command_sequence_state_{0.0};
  double applied_command_sequence_state_{0.0};
  double successful_cycles_state_{0.0};
  double failed_cycles_state_{0.0};
  double missed_deadlines_state_{0.0};
  double watchdog_stops_state_{0.0};
  double reconnects_state_{0.0};
  double last_cycle_duration_state_{0.0};
  double minimum_task_aperture_state_{0.0};
  double maximum_task_aperture_state_{0.073};
  std::array<double, onrobot_gripper_msgs::kDiagnosticInterfaceCount>
      diagnostic_states_{};
};

} // namespace onrobot_gripper_hardware

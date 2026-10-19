#pragma once
#include <onrobot_gripper_msgs/fault_recovery_protocol.hpp>

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
/// ROS uses task aperture and raw mechanism feedback. This adapter converts
/// between those coordinates and the model's single driven joint in Isaac Sim.
/// Each feedback message must have a header timestamp greater than zero and
/// later than the previous one. These timestamps determine message order.
/// Sample age is checked separately using the time when each message arrived,
/// so clocks from different systems are not compared.
class OnRobotIsaacSystem final : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OnRobotIsaacSystem)

  /// @brief Validate model, joints, topics and feedback settings for Isaac.
  ///
  /// Reads the supplied hardware description. Returns ERROR for unsupported
  /// models, invalid settings or incomplete joint interfaces.
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
      override;
  /// @brief Expose task and physical-joint feedback, validity and fault state.
  ///
  /// The joint coordinates depend on the configured model. Unavailable force
  /// feedback remains invalid.
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;
  /// @brief Expose shared motion, Stop and recovery handles for the Isaac
  /// backend.
  ///
  /// Controllers use the same task interfaces as with a physical gripper.
  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;
  /// @brief Create the physical-joint command publisher and feedback
  /// subscription.
  ///
  /// Uses the configured command and state topics with reliable QoS. No device
  /// connection is opened.
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;
  /// @brief Clear exposed feedback and wait for a current state sample.
  ///
  /// The next read() checks sample age before enabling motion. Earlier motion
  /// targets are discarded; activation alone does not establish valid feedback.
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;
  /// @brief Publish a hold using current feedback, or zero velocity without it.
  ///
  /// Disables motion and prevents earlier targets from being reused. Publishing
  /// a command does not confirm that the simulator has applied it.
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;
  /// @brief Check received joint feedback and map it to task aperture.
  ///
  /// Uses a monotonic clock to check time since receipt. Missing startup
  /// feedback keeps motion disabled. Feedback loss after motion requires
  /// explicit recovery; check the reported fault and validity fields.
  hardware_interface::return_type read(const rclcpp::Time &,
                                       const rclcpp::Duration &) override;
  /// @brief Validate commands and publish them to the Isaac joint interface.
  ///
  /// Processes Stop and explicit recovery as well as motion. Commands remain
  /// disabled while required feedback is unavailable. A successful write does
  /// not confirm completed simulated motion.
  hardware_interface::return_type write(const rclcpp::Time &,
                                        const rclcpp::Duration &) override;

  /// @cond INTERNAL
private:
  onrobot_gripper_msgs::recovery::State recovery_state_;
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
  // Initial scene/render startup may interrupt feedback before any operator
  // command. After the first admitted motion, loss requires explicit recovery
  // even if a later Stop or lifecycle transition has made the joint idle.
  bool motion_command_seen_{false};
  double startup_hold_position_{std::numeric_limits<double>::quiet_NaN()};
  bool feedback_loss_stop_pending_{false};
  bool feedback_loss_stop_sent_{false};
  bool conventional_recovery_gate_{false};
  double retired_position_command_{std::numeric_limits<double>::quiet_NaN()};
  bool rg_model_{false};
  double realtime_force_min_{0.0};
  double realtime_force_max_{95.0};
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
  double fault_recovery_sequence_ack_state_{0.0};
  double fault_recovery_admission_state_{0.0};
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
  /// @endcond
};

} // namespace onrobot_gripper_hardware

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>

#include <hardware_interface/loaned_command_interface.hpp>
#include <parallel_gripper_controller/parallel_gripper_action_controller.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include <onrobot_tool_api/model_capabilities.hpp>

namespace onrobot_gripper_controllers {

/// Standard ParallelGripperCommand controller with an explicit device Stop.
///
/// Cancelling or replacing a goal requests Stop. Setting a new position alone
/// does not reliably interrupt an OnRobot conventional movement. The controller
/// keeps the base controller's action, feedback, tolerance and stall behavior.
/// Replacement goals wait until the backend processes Stop.
///
/// During cancellation, the callback uses the shared backend lock to submit
/// Stop. If the lock is busy, the action is aborted and Stop is queued for a
/// retry; the controller does not acknowledge it as submitted. Only the
/// controller manager's update thread writes motion commands.
///
/// Deactivation writes the measured position and Stop before releasing the
/// interfaces, so the backend cannot repeat the old target afterward. The
/// backend clears each Stop marker after processing it. A controller loaded
/// again may therefore restart its marker sequence.
///
/// NaN or infinity in position or velocity feedback aborts the action and
/// queues Stop. It is not reported as object contact. Submit a new goal after
/// valid feedback returns.
class ParallelGripperActionController final
    : public parallel_gripper_action_controller::GripperActionController {
public:
  /// @brief Initialize the base action controller and declare speed settings.
  ///
  /// Adds conventional_speed_control, conventional_speed_percent and model.
  /// Propagates base initialization failures and returns ERROR if parameter
  /// declaration fails.
  controller_interface::CallbackReturn on_init() override;
  /// @brief Configure the action controller and conventional speed selection.
  ///
  /// When speed control is enabled, requires a supported 2FG model and a
  /// default speed in the range 1-100 percent. Returns ERROR for incompatible
  /// settings and propagates base configuration failures.
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  /// @brief Claim Stop and command-sequence interfaces along with motion interfaces.
  ///
  /// Also requests the native speed interface when conventional speed control
  /// is enabled.
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  /// @brief Add the state needed to convert an action speed from m/s.
  ///
  /// When conventional speed control is enabled, requests force limits,
  /// calibration, power validity and sample ages alongside the base feedback
  /// interfaces.
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  /// @brief Check feedback and coordinate motion, replacement goals and Stop.
  ///
  /// Passes the supplied time and period to the base action controller. Defers
  /// work when a callback holds the controller lock. Invalid feedback aborts
  /// the action and requests Stop; it is not reported as a successful grip.
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  /// @brief Bind required interfaces and enable the gripper action server.
  ///
  /// Requires the configured effort interface and the OnRobot command
  /// interfaces. Returns ERROR for incomplete interfaces or failed setup, and
  /// propagates base activation failures.
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  /// @brief Write a measured hold target and Stop before releasing interfaces.
  ///
  /// Cancels the active goal after writing the commands. Returns ERROR if hold or Stop
  /// cannot be written; the return value does not confirm physical standstill.
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  /// @brief Cancel outstanding goals and release the action server and
  /// callbacks.
  ///
  /// Also releases timers and interface bindings so a later load can create a
  /// new server. Returns the base controller cleanup result.
  controller_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  /// @cond INTERNAL
private:
  rclcpp_action::GoalResponse goal_with_lifecycle(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const GripperCommandAction::Goal> goal_handle);
  rclcpp_action::CancelResponse
  cancel_with_stop(const std::shared_ptr<GoalHandle> goal_handle);
  void accept_with_preemption_stop(std::shared_ptr<GoalHandle> goal_handle);
  bool set_hold_position_if_valid();
  bool feedback_is_finite() const;
  struct ConventionalVelocitySnapshot {
    onrobot::ConventionalVelocityCalibration velocity_calibration{
        onrobot::ConventionalVelocityCalibration::A};
    onrobot::ConventionalVelocityDirection direction{
        onrobot::ConventionalVelocityDirection::Opening};
    double requested_velocity_m_s{0.0};
    double effective_force_n{0.0};
    double command_effort_n{0.0};
    double live_maximum_force_n{0.0};
    double default_force_n{0.0};
    int selected_speed_percent{0};
  };
  struct GoalSpeedReservation {
    int speed_percent{50};
    std::optional<ConventionalVelocitySnapshot> si_velocity;
  };
  bool read_conventional_velocity_context(
      double &maximum_force_n, double &default_force_n,
      onrobot::ConventionalVelocityCalibration &velocity_calibration) const;
  bool conventional_velocity_snapshot_is_current(
      const ConventionalVelocitySnapshot &snapshot,
      double target_position_m, double command_effort_n) const;
  bool conventional_motion_busy();
  rcl_interfaces::msg::SetParametersResult
  set_speed_parameters(const std::vector<rclcpp::Parameter> &parameters);
  void
  commit_speed_parameters(const std::vector<rclcpp::Parameter> &parameters);
  void abort_motion();
  void clear_conventional_command_event();
  bool issue_conventional_command_event();
  void cancel_active_goal();
  RealtimeGoalHandlePtr retiring_goal_;
  void request_stop();
  bool issue_stop_command();

  std::optional<
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      stop_command_interface_;
  std::optional<
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      conventional_command_interface_;
  std::optional<
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      conventional_speed_interface_;
  std::optional<onrobot::Model> conventional_velocity_model_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      speed_parameter_callback_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
      speed_parameter_post_callback_;
  std::mutex lifecycle_mutex_;
  bool action_callbacks_enabled_{false};
  bool invalid_feedback_latched_{false};
  bool has_command_{false};
  bool dispatch_pending_{false};
  bool admission_pending_{false};
  bool retire_output_pending_{false};
  bool conventional_speed_control_enabled_{false};
  std::atomic<int> conventional_speed_percent_{50};
  int pending_conventional_speed_percent_{50};
  // The goal response and accepted-goal handler are separate callbacks. Store
  // the selected speed percentage, force and calibration under the goal UUID
  // so a parameter or state change between callbacks cannot change the speed
  // of an accepted goal expressed in m/s.
  std::map<rclcpp_action::GoalUUID, GoalSpeedReservation>
      accepted_goal_speed_percent_;
  std::optional<ConventionalVelocitySnapshot>
      pending_conventional_velocity_snapshot_;
  std::chrono::steady_clock::time_point admission_started_{};
  const GripperCommandAction::Result::SharedPtr invalid_feedback_result_{
      std::make_shared<GripperCommandAction::Result>()};
  std::atomic_bool stop_requested_{false};
  std::uint64_t stop_sequence_{0};
  std::uint64_t conventional_command_sequence_{0};
  /// @endcond
};

#ifdef ONROBOT_GRIPPER_CONTROLLERS_TESTING
// Test-build-only synchronization point used to exercise the distinct action
// goal-response and accepted-goal callbacks. It is deliberately unavailable
// from installed production headers and binaries.
void set_parallel_gripper_action_controller_test_goal_reservation_hook(
    std::function<void()> hook);
#endif

} // namespace onrobot_gripper_controllers

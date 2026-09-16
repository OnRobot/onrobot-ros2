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

namespace onrobot_gripper_controllers {

/// Standard ParallelGripperCommand controller with an explicit device Stop.
///
/// The upstream controller holds the measured position when an action is
/// cancelled. OnRobot conventional commands are atomic device motions, so a
/// new target does not reliably interrupt the active motion. This subclass
/// retains the upstream action, feedback, tolerance, and stall behavior while
/// emitting a one-shot stop event on cancel or preemption. Deactivation writes
/// the measured position and Stop while interfaces are still loaned, so a
/// hardware backend cannot replay the old target after the controller releases
/// its interfaces. The numeric event marker is only a transport-safe marker;
/// the hardware consumes and clears it, so a controller recreated after an
/// unload may start its marker sequence again.
/// Cancellation hands off Stop against the backend's shared admission lock;
/// motion outputs remain owned by the manager update thread. Replacement goals
/// wait for Stop consumption. A busy cancel handoff aborts the action and queues
/// a retry rather than acknowledging a Stop that has not been handed off.
/// Non-finite position/velocity aborts the action without claiming contact,
/// queues Stop, and requires a new goal after valid feedback returns.
class ParallelGripperActionController final
    : public parallel_gripper_action_controller::GripperActionController {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

private:
  rclcpp_action::GoalResponse goal_with_lifecycle(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const GripperCommandAction::Goal> goal_handle);
  rclcpp_action::CancelResponse
  cancel_with_stop(const std::shared_ptr<GoalHandle> goal_handle);
  void accept_with_preemption_stop(std::shared_ptr<GoalHandle> goal_handle);
  bool set_hold_position_if_valid();
  bool feedback_is_finite() const;
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
  // A goal response and its accepted callback are separate action-server
  // callbacks. Reserve the selected speed by UUID at response time so a
  // concurrent parameter request cannot alter an already accepted goal.
  std::map<rclcpp_action::GoalUUID, int> accepted_goal_speed_percent_;
  std::chrono::steady_clock::time_point admission_started_{};
  const GripperCommandAction::Result::SharedPtr invalid_feedback_result_{
      std::make_shared<GripperCommandAction::Result>()};
  std::atomic_bool stop_requested_{false};
  std::uint64_t stop_sequence_{0};
  std::uint64_t conventional_command_sequence_{0};
};

#ifdef ONROBOT_GRIPPER_CONTROLLERS_TESTING
// Test-build-only synchronization point used to exercise the distinct action
// goal-response and accepted-goal callbacks. It is deliberately unavailable
// from installed production headers and binaries.
void set_parallel_gripper_action_controller_test_goal_reservation_hook(
    std::function<void()> hook);
#endif

} // namespace onrobot_gripper_controllers

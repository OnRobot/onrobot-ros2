#include "onrobot_gripper_controllers/parallel_gripper_action_controller.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp/rclcpp.hpp>

namespace onrobot_gripper_controllers {
namespace {

constexpr char kStopCommandSequence[] = "stop_command_sequence";
constexpr char kConventionalCommandSequence[] = "conventional_command_sequence";
constexpr char kConventionalSpeedPercent[] = "conventional_speed_percent";

constexpr std::uint64_t kMaximumExactStopToken = 9007199254740991ULL;

bool is_valid_stop_token(const double value) {
  return std::isfinite(value) && value >= 1.0 &&
         value <= static_cast<double>(kMaximumExactStopToken) &&
         std::floor(value) == value;
}

#ifdef ONROBOT_GRIPPER_CONTROLLERS_TESTING
std::mutex goal_reservation_hook_mutex;
std::function<void()> goal_reservation_hook;

void invoke_goal_reservation_hook() {
  std::function<void()> hook;
  {
    std::lock_guard<std::mutex> lock(goal_reservation_hook_mutex);
    hook = goal_reservation_hook;
  }
  if (hook) hook();
}
#endif

} // namespace

#ifdef ONROBOT_GRIPPER_CONTROLLERS_TESTING
void set_parallel_gripper_action_controller_test_goal_reservation_hook(
    std::function<void()> hook) {
  std::lock_guard<std::mutex> lock(goal_reservation_hook_mutex);
  goal_reservation_hook = std::move(hook);
}
#endif

controller_interface::CallbackReturn ParallelGripperActionController::on_init() {
  const auto result = parallel_gripper_action_controller::GripperActionController::on_init();
  if (result != controller_interface::CallbackReturn::SUCCESS) return result;
  try {
    auto node = get_node();
    node->declare_parameter<bool>("conventional_speed_control", false);
    node->declare_parameter<int>(kConventionalSpeedPercent, 50);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Conventional speed controller initialization failed: %s",
                 error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ParallelGripperActionController::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  const auto result = parallel_gripper_action_controller::GripperActionController::
      on_configure(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) return result;
  auto node = get_node();
  conventional_speed_control_enabled_ =
      node->get_parameter("conventional_speed_control").as_bool();
  const auto configured_speed_percent =
      node->get_parameter(kConventionalSpeedPercent).as_int();
  if (conventional_speed_control_enabled_ &&
      (configured_speed_percent < 1 || configured_speed_percent > 100)) {
    RCLCPP_ERROR(node->get_logger(),
                 "conventional_speed_percent must be an integer from 1 to 100");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!conventional_speed_control_enabled_ && configured_speed_percent != 50) {
    RCLCPP_ERROR(node->get_logger(),
                 "conventional_speed_percent is supported only by enabled 2FG "
                 "conventional-speed controllers");
    return controller_interface::CallbackReturn::ERROR;
  }
  conventional_speed_percent_.store(static_cast<int>(configured_speed_percent),
                                    std::memory_order_release);
  pending_conventional_speed_percent_ =
      conventional_speed_percent_.load(std::memory_order_acquire);
  accepted_goal_speed_percent_.clear();
  speed_parameter_callback_ = node->add_on_set_parameters_callback(
      std::bind(&ParallelGripperActionController::set_speed_parameters, this,
                std::placeholders::_1));
  speed_parameter_post_callback_ = node->add_post_set_parameters_callback(
      std::bind(&ParallelGripperActionController::commit_speed_parameters, this,
                std::placeholders::_1));
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ParallelGripperActionController::command_interface_configuration() const {
  auto configuration = parallel_gripper_action_controller::
      GripperActionController::command_interface_configuration();
  configuration.names.push_back(params_.joint + "/" + kStopCommandSequence);
  configuration.names.push_back(params_.joint + "/" +
                                 kConventionalCommandSequence);
  if (conventional_speed_control_enabled_) {
    configuration.names.push_back(params_.joint + "/" + kConventionalSpeedPercent);
  }
  return configuration;
}

controller_interface::return_type ParallelGripperActionController::update(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
  // Serialize retirement against accept/cancel/lifecycle callbacks without
  // waiting for an executor callback in the controller-manager loop.
  std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_, std::try_to_lock);
  if (!lifecycle_lock.owns_lock()) {
    return controller_interface::return_type::OK;
  }
  if (retire_output_pending_) {
    // Cancel only writes Stop from the executor. Retire motion outputs here,
    // in the same thread as hardware write, without touching a newer goal's
    // buffered command or dispatch flag.
    if (!conventional_command_interface_->get().set_value(
            std::numeric_limits<double>::quiet_NaN())) {
      request_stop();
      return issue_stop_command() ? controller_interface::return_type::OK
                                  : controller_interface::return_type::ERROR;
    }
    if (!has_command_ && set_hold_position_if_valid()) {
      if (!joint_position_command_interface_->get().set_value(
              command_struct_.position_cmd_)) {
        request_stop();
        return issue_stop_command() ? controller_interface::return_type::OK
                                    : controller_interface::return_type::ERROR;
      }
    }
    retire_output_pending_ = false;
  }
  if (!feedback_is_finite() && !invalid_feedback_latched_) {
    abort_motion();
  }
  // A replacement goal is buffered by the executor, but must not be exported
  // until hardware has consumed Stop. Stop retires every earlier event, even
  // if update() is delayed past the worker's acknowledgement.
  if (stop_requested_.exchange(false, std::memory_order_acq_rel)) {
    if (!issue_stop_command()) {
      request_stop();
      return controller_interface::return_type::ERROR;
    }
    return controller_interface::return_type::OK;
  }
  const auto pending_stop = stop_command_interface_->get().get_optional<double>();
  if (!pending_stop || !std::isnan(*pending_stop)) {
    return controller_interface::return_type::OK;
  }
  auto result = controller_interface::return_type::OK;
  bool admission_ready = true;
  if (!invalid_feedback_latched_ && dispatch_pending_) {
    // Publish the target and effort before the event, in the manager update
    // thread. An executor callback must never identify last cycle's values
    // as a new goal if update() yields its lifecycle lock.
    if (!joint_effort_command_interface_->get().set_value(
            command_struct_.effort_cmd_) ||
        !joint_position_command_interface_->get().set_value(
            command_struct_.position_cmd_) ||
        (conventional_speed_control_enabled_ &&
         !conventional_speed_interface_->get().set_value(
             static_cast<double>(pending_conventional_speed_percent_))) ||
        !issue_conventional_command_event()) {
      abort_motion();
    } else {
      dispatch_pending_ = false;
      admission_pending_ = true;
      admission_started_ = std::chrono::steady_clock::now();
    }
    admission_ready = false;
  } else if (!invalid_feedback_latched_ && admission_pending_) {
    const auto event =
        conventional_command_interface_->get().get_optional<double>();
    // Hardware retains +token while busy, clears it on admission, and returns
    // -token on rejection. Do not run upstream reach/stall completion before
    // this handshake, including a goal at the already-measured position.
    if (event && std::isnan(*event)) {
      admission_pending_ = false;
      last_movement_time_ = get_node()->now();
    } else {
      admission_ready = false;
      if ((event && *event < 0.0) ||
          std::chrono::steady_clock::now() - admission_started_ >
              std::chrono::seconds(2)) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Conventional command rejected or admission timed out; "
                     "action aborted");
        abort_motion();
      }
    }
  }
  // Never interpret NaN as a stall, or rewrite the retained pre-fault target
  // when measurements recover. An explicitly accepted new goal clears this.
  if (has_command_ && !invalid_feedback_latched_ && admission_ready) {
    // Terminal results report the requested effort, not a force measurement.
    if (const auto command = command_.try_get(); command.has_value()) {
      computed_command_ = command->effort_cmd_;
    }
    result = parallel_gripper_action_controller::GripperActionController::update(
        time, period);
  }
  if (stop_requested_.exchange(false, std::memory_order_acq_rel)) {
    if (!issue_stop_command()) {
      request_stop();
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Unable to issue explicit gripper Stop command");
      return controller_interface::return_type::ERROR;
    }
  }
  return result;
}

controller_interface::CallbackReturn
ParallelGripperActionController::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  action_callbacks_enabled_ = false;
  joint_effort_command_interface_.reset();
  joint_speed_command_interface_.reset();
  conventional_speed_interface_.reset();
  const auto retained_server = action_server_;
  const auto result = parallel_gripper_action_controller::
      GripperActionController::on_activate(previous_state);
  action_server_ = retained_server;
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result;
  }

  // Jazzy's upstream binder recognizes only set_gripper_max_effort, although
  // its configuration claims the fully qualified name supplied by the user.
  // Bind the configured OnRobot effort resource explicitly and fail closed.
  const auto effort_interface = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&](const auto &interface) {
        return interface.get_name() == params_.max_effort_interface &&
               interface.get_prefix_name() == params_.joint;
      });
  if (effort_interface == command_interfaces_.end() ||
      !std::isfinite(params_.max_effort) || params_.max_effort < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected configured gripper effort interface and finite "
                 "nonnegative default");
    return controller_interface::CallbackReturn::ERROR;
  }
  joint_effort_command_interface_ = *effort_interface;

  const auto stop_interface = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [](const auto &interface) {
        return interface.get_interface_name() == kStopCommandSequence;
      });
  if (stop_interface == command_interfaces_.end() ||
      stop_interface->get_prefix_name() != params_.joint) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected %s/%s command interface", params_.joint.c_str(),
                 kStopCommandSequence);
    return controller_interface::CallbackReturn::ERROR;
  }
  stop_command_interface_ = *stop_interface;
  const auto conventional_interface = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [](const auto &interface) {
        return interface.get_interface_name() == kConventionalCommandSequence;
      });
  if (conventional_interface == command_interfaces_.end() ||
      conventional_interface->get_prefix_name() != params_.joint) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Expected %s/%s command interface", params_.joint.c_str(),
        kConventionalCommandSequence);
    stop_command_interface_ = std::nullopt;
    return controller_interface::CallbackReturn::ERROR;
  }
  conventional_command_interface_ = *conventional_interface;
  if (conventional_speed_control_enabled_) {
    const auto speed_interface = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [](const auto &interface) {
          return interface.get_interface_name() == kConventionalSpeedPercent;
        });
    if (speed_interface == command_interfaces_.end() ||
        speed_interface->get_prefix_name() != params_.joint ||
        !speed_interface->set_value(static_cast<double>(
            conventional_speed_percent_.load(std::memory_order_acquire)))) {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %s/%s command interface",
                   params_.joint.c_str(), kConventionalSpeedPercent);
      conventional_command_interface_ = std::nullopt;
      stop_command_interface_ = std::nullopt;
      return controller_interface::CallbackReturn::ERROR;
    }
    conventional_speed_interface_ = *speed_interface;
  }
  invalid_feedback_latched_ = false;
  has_command_ = false;
  dispatch_pending_ = false;
  admission_pending_ = false;
  stop_requested_.store(false, std::memory_order_release);
  retire_output_pending_ = false;
  accepted_goal_speed_percent_.clear();
  // Preserve a valid event marker left by a previous controller until the next
  // hardware write consumes it. This matters when controller switching
  // deactivates and reactivates controllers in one manager cycle.
  const auto existing = stop_command_interface_->get().get_optional<double>();
  if (!existing.has_value() || !is_valid_stop_token(*existing)) {
    if (!stop_command_interface_->get().set_value(
            std::numeric_limits<double>::quiet_NaN())) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Unable to initialize explicit gripper Stop interface");
      stop_command_interface_ = std::nullopt;
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  clear_conventional_command_event();

  // Replace the upstream action server with the same public action contract
  // and OnRobot-specific cancel/preemption callbacks. All other controller
  // behavior remains inherited from the installed Jazzy controller.
  if (!action_server_) {
    action_server_ = rclcpp_action::create_server<GripperCommandAction>(
      get_node(), "~/gripper_cmd",
      std::bind(&ParallelGripperActionController::goal_with_lifecycle, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&ParallelGripperActionController::cancel_with_stop, this,
                std::placeholders::_1),
      std::bind(
          &ParallelGripperActionController::accept_with_preemption_stop, this,
          std::placeholders::_1));
  }
  action_callbacks_enabled_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ParallelGripperActionController::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  action_callbacks_enabled_ = false;
  // There may be no further update() after this callback. Publish the Stop
  // and replace the old target with the measured hold while the command
  // interfaces are still loaned, then let the base controller release them.
  // Without the hold write, a hardware backend could replay a stale
  // conventional target after it has consumed Stop.
  stop_requested_.store(false, std::memory_order_release);
  // Deactivation is a terminal handoff, not a replacement goal. Retire an
  // accepted-but-not-yet-dispatched identity before the hold/Stop write so
  // the hardware cannot replay it after this controller releases its claims.
  clear_conventional_command_event();
  bool hold_written = false;
  retire_output_pending_ = false;
  if (joint_position_command_interface_.has_value() &&
      joint_position_state_interface_.has_value()) {
    const auto measured_position =
        joint_position_state_interface_->get().get_optional<double>();
    hold_written = measured_position.has_value() &&
                   std::isfinite(*measured_position) &&
                   joint_position_command_interface_->get().set_value(
                       *measured_position);
  }
  const bool stop_written = issue_stop_command();
  if (!hold_written || !stop_written) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Unable to hand off conventional Stop/hold during deactivation");
    stop_requested_.store(true, std::memory_order_release);
    action_callbacks_enabled_ = true;
    return controller_interface::CallbackReturn::ERROR;
  }
  cancel_active_goal();
  // The executor timer delivers the terminal result outside the manager's
  // update thread. Keep the server alive; the lifecycle gate rejects goals.
  stop_command_interface_ = std::nullopt;
  conventional_command_interface_ = std::nullopt;
  conventional_speed_interface_ = std::nullopt;
  accepted_goal_speed_percent_.clear();
  joint_effort_command_interface_.reset();
  joint_speed_command_interface_.reset();
  return parallel_gripper_action_controller::GripperActionController::
      on_deactivate(previous_state);
}

controller_interface::CallbackReturn
ParallelGripperActionController::on_cleanup(
    const rclcpp_lifecycle::State &previous_state) {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  action_callbacks_enabled_ = false;
  // The upstream controller creates its action server during activation but
  // does not tear that ROS entity down in cleanup. Explicitly release it (and
  // its status timer) so controller-manager unload/reload is a real lifecycle
  // operation rather than leaving a plugin-owned callback alive.
  cancel_active_goal();
  if (retiring_goal_) {
    retiring_goal_->runNonRealtime();
    retiring_goal_.reset();
  }
  goal_handle_timer_.reset();
  action_server_.reset();
  stop_requested_.store(false, std::memory_order_release);
  clear_conventional_command_event();
  stop_command_interface_ = std::nullopt;
  conventional_command_interface_ = std::nullopt;
  conventional_speed_interface_ = std::nullopt;
  accepted_goal_speed_percent_.clear();
  speed_parameter_post_callback_.reset();
  speed_parameter_callback_.reset();
  joint_effort_command_interface_.reset();
  joint_speed_command_interface_.reset();
  return parallel_gripper_action_controller::GripperActionController::
      on_cleanup(previous_state);
}

rclcpp_action::GoalResponse ParallelGripperActionController::goal_with_lifecycle(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const GripperCommandAction::Goal> goal_handle) {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (!action_callbacks_enabled_ || !feedback_is_finite()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  const auto &command = goal_handle->command;
  if (command.position.size() != 1 || !std::isfinite(command.position[0]) ||
      command.effort.size() > 1 ||
      (!command.effort.empty() &&
       (!std::isfinite(command.effort[0]) || command.effort[0] < 0.0)) ||
      (!command.name.empty() &&
       (command.name.size() != 1 || command.name[0] != params_.joint))) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Rejecting malformed gripper position/effort goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  // The standard action permits an optional per-goal velocity array. Neither
  // conventional device protocol provides an equivalent SI task-velocity
  // command: 2FG conventional speed is a device percentage and RG's
  // realtime velocity is an angular mechanism coordinate. Accepting the
  // field would therefore acknowledge a limit that the hardware cannot
  // enforce. Realtime control remains the explicit velocity interface.
  if (!goal_handle->command.velocity.empty()) {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "Rejecting ParallelGripperCommand goal with command.velocity: "
        "per-goal conventional velocity is unsupported; use the realtime "
        "controller for velocity control");
    return rclcpp_action::GoalResponse::REJECT;
  }
  const auto response = goal_callback(uuid, std::move(goal_handle));
  if (response == rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE ||
      response == rclcpp_action::GoalResponse::ACCEPT_AND_DEFER) {
    // The action server returns the response before it invokes the accepted
    // callback. Keep the selected percentage with this exact action UUID,
    // rather than sampling the mutable controller parameter later.
    accepted_goal_speed_percent_[uuid] =
        conventional_speed_percent_.load(std::memory_order_acquire);
#ifdef ONROBOT_GRIPPER_CONTROLLERS_TESTING
    invoke_goal_reservation_hook();
#endif
  }
  return response;
}

rclcpp_action::CancelResponse
ParallelGripperActionController::cancel_with_stop(
    const std::shared_ptr<GoalHandle> goal_handle) {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");
  RealtimeGoalHandlePtr active_goal;
  rt_active_goal_.get(
      [&](const RealtimeGoalHandlePtr &goal) { active_goal = goal; });
  if (active_goal && active_goal->gh_ == goal_handle) {
    // The backend holds this shared handle's mutex through admission. Stop
    // therefore linearizes before the next write or after an already-started
    // write, never between a changed position and its retained effort. Motion
    // values/event are retired by update(), not from this executor callback.
    const bool stop_written = issue_stop_command();
    has_command_ = false;
    dispatch_pending_ = false;
    admission_pending_ = false;
    retire_output_pending_ = true;
    if (!stop_written) {
      // Do not acknowledge a cancel whose Stop was not handed off. Fail the
      // action closed and retry Stop in update without waiting in this callback.
      request_stop();
      active_goal->setAborted(invalid_feedback_result_);
      retiring_goal_ = active_goal;
      rt_active_goal_.set([](RealtimeGoalHandlePtr &stored) { stored.reset(); });
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Cancel Stop handoff busy; action aborted and Stop retry queued");
      return rclcpp_action::CancelResponse::REJECT;
    }
    RCLCPP_INFO(get_node()->get_logger(),
                "Canceling active action goal and issuing device Stop.");
    active_goal->setCanceled(std::make_shared<GripperCommandAction::Result>());
    retiring_goal_ = active_goal;
    rt_active_goal_.set([](RealtimeGoalHandlePtr &stored_value) {
      stored_value = RealtimeGoalHandlePtr();
    });
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ParallelGripperActionController::accept_with_preemption_stop(
    std::shared_ptr<GoalHandle> goal_handle) {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  // goal_with_lifecycle() has already accepted this UUID. Consume its
  // reservation before checking mutable feedback or lifecycle state: either
  // condition may have changed between the goal-response and accepted
  // callbacks, and an aborted accepted goal must never leave future parameter
  // writes permanently blocked as "busy".
  const auto speed = accepted_goal_speed_percent_.find(goal_handle->get_goal_id());
  if (speed == accepted_goal_speed_percent_.end()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Accepted conventional goal has no reserved speed snapshot");
    goal_handle->abort(invalid_feedback_result_);
    return;
  }
  const int speed_snapshot = speed->second;
  accepted_goal_speed_percent_.erase(speed);
  if (!action_callbacks_enabled_ || !feedback_is_finite()) {
    auto result = std::make_shared<GripperCommandAction::Result>();
    goal_handle->abort(result);
    return;
  }
  if (retiring_goal_) {
    retiring_goal_->runNonRealtime();
    retiring_goal_.reset();
  }
  RealtimeGoalHandlePtr active_goal;
  rt_active_goal_.get(std::function<void(const RealtimeGoalHandlePtr &)>(
      [&](const RealtimeGoalHandlePtr &goal) { active_goal = goal; }));
  if (active_goal) {
    if (!issue_stop_command()) {
      request_stop();
      has_command_ = false;
      dispatch_pending_ = false;
      admission_pending_ = false;
      retire_output_pending_ = true;
      active_goal->setAborted(invalid_feedback_result_);
      retiring_goal_ = active_goal;
      rt_active_goal_.set([](RealtimeGoalHandlePtr &stored) { stored.reset(); });
      goal_handle->abort(invalid_feedback_result_);
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Replacement Stop handoff busy; actions aborted and Stop retry queued");
      return;
    }
  }
  accepted_callback(std::move(goal_handle));
  has_command_ = true;
  pending_conventional_speed_percent_ = speed_snapshot;
  invalid_feedback_latched_ = false;
  dispatch_pending_ = true;
  admission_pending_ = false;
}

bool ParallelGripperActionController::conventional_motion_busy() {
  if (!accepted_goal_speed_percent_.empty() || dispatch_pending_ ||
      admission_pending_ || retire_output_pending_ ||
      stop_requested_.load(std::memory_order_acquire)) {
    return true;
  }
  if (!stop_command_interface_.has_value()) return true;
  const auto pending_stop = stop_command_interface_->get().get_optional<double>();
  if (!pending_stop.has_value() || !std::isnan(*pending_stop)) return true;
  RealtimeGoalHandlePtr active_goal;
  rt_active_goal_.get(std::function<void(const RealtimeGoalHandlePtr &)>(
      [&](const RealtimeGoalHandlePtr &goal) { active_goal = goal; }));
  return active_goal && active_goal->gh_ && active_goal->gh_->is_active();
}

rcl_interfaces::msg::SetParametersResult
ParallelGripperActionController::set_speed_parameters(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  std::unique_lock<std::mutex> lock(lifecycle_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    result.reason = "controller update is in progress; retry the speed change";
    return result;
  }
  bool change_requested = false;
  for (const auto &parameter : parameters) {
    if (parameter.get_name() == "conventional_speed_control") {
      result.reason =
          "conventional_speed_control is fixed at controller configuration";
      return result;
    }
    if (parameter.get_name() != kConventionalSpeedPercent)
      continue;
    change_requested = true;
    if (!conventional_speed_control_enabled_) {
      result.reason =
          "runtime conventional speed is unavailable for this gripper";
      return result;
    }
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
      result.reason =
          "conventional_speed_percent must be an integer from 1 to 100";
      return result;
    }
    const auto candidate = parameter.as_int();
    if (candidate < 1 || candidate > 100) {
      result.reason =
          "conventional_speed_percent must be an integer from 1 to 100";
      return result;
    }
  }
  if (change_requested && !action_callbacks_enabled_) {
    result.reason = "the conventional controller is inactive";
    return result;
  }
  if (change_requested && conventional_motion_busy()) {
    result.reason = "conventional motion, admission, or Stop is still active";
    return result;
  }
  result.successful = true;
  return result;
}

void ParallelGripperActionController::commit_speed_parameters(
    const std::vector<rclcpp::Parameter> &parameters) {
  // On-set callbacks validate a proposed atomic batch, but another callback
  // can still reject that batch. Mirror the value used for goal snapshots only
  // from this post-set callback, which runs after the complete transaction has
  // been committed by rclcpp.
  std::optional<int> committed_speed;
  for (const auto &parameter : parameters) {
    if (parameter.get_name() == kConventionalSpeedPercent) {
      // rclcpp defines the last occurrence as authoritative when an atomic
      // request repeats a parameter name.
      committed_speed = static_cast<int>(parameter.as_int());
    }
  }
  if (committed_speed.has_value()) {
    conventional_speed_percent_.store(*committed_speed,
                                      std::memory_order_release);
  }
}

void ParallelGripperActionController::clear_conventional_command_event() {
  dispatch_pending_ = false;
  admission_pending_ = false;
  if (conventional_command_interface_.has_value()) {
    (void)conventional_command_interface_->get().set_value(
        std::numeric_limits<double>::quiet_NaN());
  }
}

void ParallelGripperActionController::abort_motion() {
  invalid_feedback_latched_ = true;
  has_command_ = false;
  const auto event =
      conventional_command_interface_
          ? conventional_command_interface_->get().get_optional<double>()
          : std::nullopt;
  if (event && *event < 0.0) {
    // Retain the backend's rejection fence until a fresh goal or lifecycle
    // transition. Clearing it would make the retained outputs look like intent.
    dispatch_pending_ = false;
    admission_pending_ = false;
  } else {
    clear_conventional_command_event();
  }
  // Replace the rejected command both in the buffer and in the loaned output.
  // The next hardware write must not replay it while Stop is being queued.
  if (set_hold_position_if_valid()) {
    (void)joint_position_command_interface_->get().set_value(
        command_struct_.position_cmd_);
  }
  RealtimeGoalHandlePtr active_goal;
  rt_active_goal_.get(
      [&](const RealtimeGoalHandlePtr &goal) { active_goal = goal; });
  if (active_goal) {
    active_goal->setAborted(invalid_feedback_result_);
    retiring_goal_ = active_goal;
    rt_active_goal_.set([](RealtimeGoalHandlePtr &goal) { goal.reset(); });
  }
  request_stop();
}

bool ParallelGripperActionController::issue_conventional_command_event() {
  if (!conventional_command_interface_.has_value()) {
    return false;
  }
  ++conventional_command_sequence_;
  if (conventional_command_sequence_ == 0 ||
      conventional_command_sequence_ > kMaximumExactStopToken) {
    conventional_command_sequence_ = 1;
  }
  return conventional_command_interface_->get().set_value(
      static_cast<double>(conventional_command_sequence_));
}

bool ParallelGripperActionController::feedback_is_finite() const {
  if (!joint_position_state_interface_ || !joint_velocity_state_interface_) {
    return false;
  }
  const auto position = joint_position_state_interface_->get().get_optional<double>();
  const auto velocity = joint_velocity_state_interface_->get().get_optional<double>();
  return position && velocity && std::isfinite(*position) && std::isfinite(*velocity);
}

bool ParallelGripperActionController::set_hold_position_if_valid() {
  if (!joint_position_state_interface_.has_value() ||
      !joint_position_command_interface_.has_value()) {
    return false;
  }
  const auto measured_position =
      joint_position_state_interface_->get().get_optional<double>();
  if (!measured_position.has_value() || !std::isfinite(*measured_position)) {
    return false;
  }
  command_struct_.position_cmd_ = *measured_position;
  command_struct_.effort_cmd_ = params_.max_effort;
  command_struct_.velocity_cmd_ = params_.max_velocity;
  command_.set(command_struct_);
  return true;
}

void ParallelGripperActionController::cancel_active_goal() {
  RealtimeGoalHandlePtr active_goal;
  rt_active_goal_.get(
      [&](const RealtimeGoalHandlePtr &goal) { active_goal = goal; });
  if (active_goal) {
    // Lifecycle interruption is a server abort, not a client cancel request.
    // The latter requires the ROS goal to have entered CANCELING first.
    active_goal->setAborted(std::make_shared<GripperCommandAction::Result>());
    retiring_goal_ = active_goal;
    rt_active_goal_.set([](RealtimeGoalHandlePtr &stored_value) {
      stored_value = RealtimeGoalHandlePtr();
    });
  }
}

void ParallelGripperActionController::request_stop() {
  stop_requested_.store(true, std::memory_order_release);
}

bool ParallelGripperActionController::issue_stop_command() {
  if (!stop_command_interface_.has_value()) {
    return false;
  }
  // This is a one-shot event marker, not a persistent identity. Hardware
  // consumes and clears it in the same write cycle, so a newly loaded
  // controller may safely restart at one even when the previous instance did.
  if (stop_sequence_ >= kMaximumExactStopToken) {
    return false;
  }
  const auto token = ++stop_sequence_;
  return stop_command_interface_->get().set_value(
      static_cast<double>(token));
}

} // namespace onrobot_gripper_controllers

PLUGINLIB_EXPORT_CLASS(
    onrobot_gripper_controllers::ParallelGripperActionController,
    controller_interface::ControllerInterface)

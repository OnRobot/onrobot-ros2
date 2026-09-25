#include "onrobot_gripper_controllers/parallel_gripper_action_controller.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <limits>
#include <string_view>
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
constexpr char kModel[] = "model";
constexpr char kConventionalPowerValid[] = "diagnostic_conventional_power_valid";
constexpr char kMaximumForce[] = "diagnostic_maximum_force";
constexpr char kVelocityCalibration[] = "diagnostic_conventional_velocity_calibration";
constexpr char kVelocityCalibrationValid[] = "diagnostic_conventional_velocity_calibration_valid";
constexpr char kDefaultForce[] = "diagnostic_conventional_default_force_n";
constexpr char kDiagnosticAge[] = "diagnostic_age";
constexpr char kSampleAge[] = "sample_age";
constexpr double kMaximumProcessStateAgeSeconds = 0.5;
constexpr double kMaximumDiagnosticAgeSeconds = 3.0;

constexpr std::uint64_t kMaximumExactStopToken = 9007199254740991ULL;

bool is_valid_stop_token(const double value) {
  return std::isfinite(value) && value >= 1.0 &&
         value <= static_cast<double>(kMaximumExactStopToken) &&
         std::floor(value) == value;
}

std::optional<onrobot::Model> model_from_name(const std::string_view name) {
  if (name == "2fg7") return onrobot::Model::TwoFG7;
  if (name == "2fg14") return onrobot::Model::TwoFG14;
  return std::nullopt;
}

std::optional<double> state_value(
    const std::vector<hardware_interface::LoanedStateInterface> &interfaces,
    const std::string &joint, const std::string_view name) {
  for (const auto &interface : interfaces) {
    if (interface.get_prefix_name() == joint &&
        interface.get_interface_name() == name) {
      return interface.get_optional<double>();
    }
  }
  return std::nullopt;
}

bool has_state_interface(
    const std::vector<hardware_interface::LoanedStateInterface> &interfaces,
    const std::string &joint, const std::string_view name) {
  return std::any_of(interfaces.begin(), interfaces.end(),
                     [&](const auto &interface) {
                       return interface.get_prefix_name() == joint &&
                              interface.get_interface_name() == name;
                     });
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
    node->declare_parameter<std::string>(kModel, "");
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
  conventional_velocity_model_.reset();
  if (conventional_speed_control_enabled_) {
    conventional_velocity_model_ = model_from_name(
        node->get_parameter(kModel).as_string());
    const auto hasOpeningFit = [this](onrobot::ConventionalVelocityCalibration calibration) {
      return onrobot::findConventionalVelocityProfile(
                 conventional_velocity_model_.value(), calibration,
                 onrobot::ConventionalVelocityDirection::Opening) != nullptr;
    };
    if (!conventional_velocity_model_.has_value() ||
        (!hasOpeningFit(onrobot::ConventionalVelocityCalibration::A) &&
         !hasOpeningFit(onrobot::ConventionalVelocityCalibration::B))) {
      RCLCPP_ERROR(
          node->get_logger(),
          "A supported 2FG model (2fg7 or 2fg14) is required when "
          "conventional_speed_control is enabled");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
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

controller_interface::InterfaceConfiguration
ParallelGripperActionController::state_interface_configuration() const {
  auto configuration = parallel_gripper_action_controller::
      GripperActionController::state_interface_configuration();
  if (conventional_speed_control_enabled_) {
    for (const auto name : {kConventionalPowerValid, kMaximumForce,
                            kVelocityCalibration, kVelocityCalibrationValid, kDefaultForce,
                            kDiagnosticAge, kSampleAge}) {
      configuration.names.push_back(params_.joint + "/" + name);
    }
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
    if (pending_conventional_velocity_snapshot_ &&
        !conventional_velocity_snapshot_is_current(
            *pending_conventional_velocity_snapshot_,
            command_struct_.position_cmd_, command_struct_.effort_cmd_)) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "Conventional SI-velocity goal rejected before dispatch: force, "
          "calibration profile, aperture direction, or state freshness changed");
      abort_motion();
    } else if (!joint_effort_command_interface_->get().set_value(
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
      pending_conventional_velocity_snapshot_.reset();
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

  if (conventional_speed_control_enabled_) {
    for (const auto name : {kConventionalPowerValid, kMaximumForce,
                            kVelocityCalibration, kVelocityCalibrationValid, kDefaultForce,
                            kDiagnosticAge, kSampleAge}) {
      if (!has_state_interface(state_interfaces_, params_.joint, name)) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "SI conventional velocity requires state interface %s/%s",
            params_.joint.c_str(), name);
        return controller_interface::CallbackReturn::ERROR;
      }
    }
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
  pending_conventional_velocity_snapshot_.reset();
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
  pending_conventional_velocity_snapshot_.reset();
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
  conventional_velocity_model_.reset();
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
  if (goal_handle->command.velocity.size() > 1 ||
      (!goal_handle->command.velocity.empty() &&
       (!std::isfinite(goal_handle->command.velocity[0]) ||
        goal_handle->command.velocity[0] <= 0.0))) {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "Rejecting ParallelGripperCommand goal with command.velocity: "
        "expected one finite positive value in m/s, or an empty array");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!goal_handle->command.velocity.empty() &&
      !conventional_speed_control_enabled_) {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "Rejecting ParallelGripperCommand goal with command.velocity: "
        "SI velocity conversion is available only for 2FG conventional "
        "controllers");
    return rclcpp_action::GoalResponse::REJECT;
  }
  GoalSpeedReservation reservation;
  std::optional<double> clamped_minimum_speed_m_s;
  reservation.speed_percent =
      conventional_speed_percent_.load(std::memory_order_acquire);
  if (!goal_handle->command.velocity.empty() &&
      goal_handle->command.velocity[0] > 0.0) {
    const auto measured_position =
        joint_position_state_interface_->get().get_optional<double>();
    if (!measured_position.has_value() || !std::isfinite(*measured_position)) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Rejecting velocity goal because measured aperture is unavailable");
      return rclcpp_action::GoalResponse::REJECT;
    }
    // A maximum aperture speed is independent of direction or travel distance.
    const auto direction = onrobot::ConventionalVelocityDirection::Fastest;
    double maximum_force_n = 0.0;
    double default_force_n = 0.0;
    onrobot::ConventionalVelocityCalibration velocity_calibration{};
    if (!read_conventional_velocity_context(
            maximum_force_n, default_force_n, velocity_calibration)) {
      RCLCPP_WARN(
          get_node()->get_logger(),
          "Rejecting SI velocity goal: current 2FG force/calibration context is "
          "missing, stale, unsupported, or invalid");
      return rclcpp_action::GoalResponse::REJECT;
    }
    const double command_effort_n = goal_handle->command.effort.empty()
                                        ? params_.max_effort
                                        : goal_handle->command.effort[0];
    const double effective_force_n = command_effort_n > 0.0
                                         ? command_effort_n
                                         : default_force_n;
    const auto converted = onrobot::conventionalSpeedPercentForVelocity(
        conventional_velocity_model_.value(), velocity_calibration, direction,
        effective_force_n, maximum_force_n,
        std::min(goal_handle->command.velocity[0],
                 std::numeric_limits<double>::max() / 1000.0) * 1000.0);
    if (!converted.has_value()) {
      RCLCPP_WARN(
          get_node()->get_logger(),
          "Rejecting SI velocity goal: effective force, live force ceiling, "
          "or calibrated calibration/direction profile is unsupported");
      return rclcpp_action::GoalResponse::REJECT;
    }
    const auto minimum_speed = onrobot::conventionalVelocityMmS(
        conventional_velocity_model_.value(), velocity_calibration, direction,
        effective_force_n, maximum_force_n, 1.0);
    if (*converted == 1 && minimum_speed &&
        goal_handle->command.velocity[0] * 1000.0 + 1e-9 < *minimum_speed) {
      clamped_minimum_speed_m_s = *minimum_speed / 1000.0;
    }
    reservation.speed_percent = static_cast<int>(converted.value());
    reservation.si_velocity = ConventionalVelocitySnapshot{
        velocity_calibration,
        direction,
        goal_handle->command.velocity[0],
        effective_force_n,
        command_effort_n,
        maximum_force_n,
        default_force_n,
        reservation.speed_percent};
  }
  const auto response = goal_callback(uuid, std::move(goal_handle));
  if (response == rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE ||
      response == rclcpp_action::GoalResponse::ACCEPT_AND_DEFER) {
    // The action server returns the response before it invokes the accepted
    // callback. Keep the selected percentage with this exact action UUID,
    // rather than sampling the mutable controller parameter later.
    accepted_goal_speed_percent_[uuid] = reservation;
    if (clamped_minimum_speed_m_s) {
      RCLCPP_WARN(
          get_node()->get_logger(),
          "Clamping SI velocity %.6f m/s to native 1%% for %.1f N: "
          "minimum estimated speed %.6f m/s may exceed the requested velocity",
          reservation.si_velocity->requested_velocity_m_s,
          reservation.si_velocity->effective_force_n,
          *clamped_minimum_speed_m_s);
    }
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
    pending_conventional_velocity_snapshot_.reset();
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
  const auto reservation = speed->second;
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
      pending_conventional_velocity_snapshot_.reset();
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
  pending_conventional_speed_percent_ = reservation.speed_percent;
  pending_conventional_velocity_snapshot_ = reservation.si_velocity;
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
    if (parameter.get_name() == kModel) {
      result.reason = "model is fixed at controller configuration";
      return result;
    }
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
  pending_conventional_velocity_snapshot_.reset();
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

bool ParallelGripperActionController::read_conventional_velocity_context(
    double &maximum_force_n, double &default_force_n,
    onrobot::ConventionalVelocityCalibration &velocity_calibration) const {
  if (!conventional_speed_control_enabled_ || !conventional_velocity_model_) {
    return false;
  }
  const auto power_valid = state_value(state_interfaces_, params_.joint,
                                       kConventionalPowerValid);
  const auto maximum_force =
      state_value(state_interfaces_, params_.joint, kMaximumForce);
  const auto profile_value =
      state_value(state_interfaces_, params_.joint, kVelocityCalibration);
  const auto profile_valid =
      state_value(state_interfaces_, params_.joint, kVelocityCalibrationValid);
  const auto configured_default =
      state_value(state_interfaces_, params_.joint, kDefaultForce);
  const auto diagnostic_age =
      state_value(state_interfaces_, params_.joint, kDiagnosticAge);
  const auto sample_age =
      state_value(state_interfaces_, params_.joint, kSampleAge);
  if (!power_valid || !maximum_force || !profile_value || !profile_valid ||
      !configured_default || !diagnostic_age || !sample_age ||
      !std::isfinite(*power_valid) || *power_valid < 0.5 ||
      !std::isfinite(*maximum_force) || *maximum_force <= 0.0 ||
      !std::isfinite(*profile_value) ||
      !std::isfinite(*profile_valid) || *profile_valid < 0.5 ||
      !std::isfinite(*configured_default) || *configured_default <= 0.0 ||
      !std::isfinite(*diagnostic_age) || *diagnostic_age < 0.0 ||
      *diagnostic_age > kMaximumDiagnosticAgeSeconds ||
      !std::isfinite(*sample_age) || *sample_age < 0.0 ||
      *sample_age > kMaximumProcessStateAgeSeconds) {
    return false;
  }

  if (*profile_value == static_cast<double>(
                           onrobot::ConventionalVelocityCalibration::A)) {
    velocity_calibration = onrobot::ConventionalVelocityCalibration::A;
  } else if (*profile_value == static_cast<double>(
                                  onrobot::ConventionalVelocityCalibration::B)) {
    velocity_calibration = onrobot::ConventionalVelocityCalibration::B;
  } else {
    return false;
  }
  const auto minimum_force =
      onrobot::minimumConventionalForceN(*conventional_velocity_model_);
  if (*configured_default < minimum_force ||
      *configured_default > *maximum_force) {
    return false;
  }
  maximum_force_n = *maximum_force;
  default_force_n = *configured_default;
  return true;
}

bool ParallelGripperActionController::
conventional_velocity_snapshot_is_current(
    const ConventionalVelocitySnapshot &snapshot,
    double target_position_m, double command_effort_n) const {
  if (!std::isfinite(target_position_m) ||
      !std::isfinite(command_effort_n)) {
    return false;
  }
  constexpr double tolerance = 1.0e-6;
  if (std::abs(command_effort_n - snapshot.command_effort_n) > tolerance) {
    return false;
  }
  double maximum_force_n = 0.0;
  double default_force_n = 0.0;
  onrobot::ConventionalVelocityCalibration velocity_calibration{};
  if (!read_conventional_velocity_context(
          maximum_force_n, default_force_n, velocity_calibration) ||
      velocity_calibration != snapshot.velocity_calibration ||
      std::abs(maximum_force_n - snapshot.live_maximum_force_n) > tolerance ||
      std::abs(default_force_n - snapshot.default_force_n) > tolerance) {
    return false;
  }
  const auto measured_position =
      joint_position_state_interface_->get().get_optional<double>();
  if (!measured_position || !std::isfinite(*measured_position)) {
    return false;
  }
  const auto direction = onrobot::ConventionalVelocityDirection::Fastest;
  if (direction != snapshot.direction) {
    return false;
  }
  const double effective_force_n =
      command_effort_n > 0.0 ? command_effort_n : default_force_n;
  if (std::abs(effective_force_n - snapshot.effective_force_n) > tolerance ||
      effective_force_n > maximum_force_n) {
    return false;
  }
  const auto converted = onrobot::conventionalSpeedPercentForVelocity(
      conventional_velocity_model_.value(), velocity_calibration, direction,
      effective_force_n, maximum_force_n,
      std::min(snapshot.requested_velocity_m_s,
               std::numeric_limits<double>::max() / 1000.0) * 1000.0);
  return converted.has_value() &&
         static_cast<int>(converted.value()) == snapshot.selected_speed_percent;
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

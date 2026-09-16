#include "onrobot_gripper_hardware/onrobot_isaac_system.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

#include <builtin_interfaces/msg/time.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <onrobot_tool_api/result.hpp>

#include "onrobot_gripper_hardware/command_sequence.hpp"

namespace onrobot_gripper_hardware {
namespace {

double nan() { return std::numeric_limits<double>::quiet_NaN(); }

using DiagnosticInterface = onrobot_gripper_msgs::DiagnosticInterface;

void setDiagnostic(
    std::array<double, onrobot_gripper_msgs::kDiagnosticInterfaceCount> &
        io_values,
    DiagnosticInterface i_interface, double i_value) {
  io_values[onrobot_gripper_msgs::diagnosticIndex(i_interface)] = i_value;
}

int64_t sourceStampNanoseconds(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
         static_cast<int64_t>(stamp.nanosec);
}

double faultCode(onrobot::ErrorCode code) { return static_cast<double>(code); }

double finiteParameter(const hardware_interface::HardwareInfo &info,
                       const std::string &name, double fallback) {
  const auto item = info.hardware_parameters.find(name);
  if (item == info.hardware_parameters.end()) {
    return fallback;
  }
  size_t consumed = 0;
  const double value = std::stod(item->second, &consumed);
  if (consumed != item->second.size() || !std::isfinite(value)) {
    throw std::invalid_argument(name + " must be finite");
  }
  return value;
}

std::string stringParameter(const hardware_interface::HardwareInfo &info,
                            const std::string &name,
                            const std::string &fallback) {
  const auto item = info.hardware_parameters.find(name);
  return item == info.hardware_parameters.end() ? fallback : item->second;
}

bool endsWith(const std::string &text, const std::string &suffix) {
  return text.size() >= suffix.size() &&
         text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

} // namespace

hardware_interface::CallbackReturn OnRobotIsaacSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  info_ = params.hardware_info;
  try {
    const auto model = stringParameter(info_, "model", "");
    if (model != "2fg7" && model != "2fg14" && model != "rg2" &&
        model != "rg6") {
      throw std::invalid_argument(
          "Isaac backend supports models 2fg7, 2fg14, rg2, and rg6");
    }
    rg_model_ = model == "rg2" || model == "rg6";
    if (info_.joints.size() != 2 ||
        !endsWith(info_.joints.front().name, "grip_stroke") ||
        (!endsWith(info_.joints.back().name, "finger_stroke") &&
         !endsWith(info_.joints.back().name, "finger_joint"))) {
      throw std::invalid_argument(
          "Isaac backend requires grip_stroke and a supported physical joint");
    }
    task_joint_ = info_.joints.front().name;
    physical_joint_ = info_.joints.back().name;
    isaac_joint_ = stringParameter(info_, "isaac_joint", "finger_stroke");
    command_topic_ = stringParameter(info_, "isaac_joint_commands_topic",
                                     "isaac_joint_commands");
    state_topic_ = stringParameter(info_, "isaac_joint_states_topic",
                                   "isaac_joint_states");
    task_min_m_ = finiteParameter(info_, "task_min_m", 0.0);
    task_max_m_ = finiteParameter(info_, "task_max_m", 0.073);
    physical_min_m_ = finiteParameter(info_, "physical_min_m", 0.0);
    physical_max_m_ = finiteParameter(info_, "physical_max_m", 0.019);
    raw_min_m_ = finiteParameter(info_, "raw_linear_min_mm", 1.0) / 1000.0;
    raw_max_m_ = finiteParameter(info_, "raw_linear_max_mm", 39.0) / 1000.0;
    const double timeout_ms =
        finiteParameter(info_, "isaac_state_timeout_ms", 250.0);
    if (task_max_m_ <= task_min_m_ || physical_max_m_ <= physical_min_m_ ||
        raw_max_m_ <= raw_min_m_ || timeout_ms <= 0.0 || isaac_joint_.empty() ||
        command_topic_.empty() || state_topic_.empty()) {
      throw std::invalid_argument(
          "Isaac coordinate or topic parameters invalid");
    }
    state_timeout_ = std::chrono::milliseconds(
        static_cast<std::chrono::milliseconds::rep>(timeout_ms));
    if (rg_model_) {
      rg_kinematics_ = std::make_unique<RgVisualKinematics>(
          task_min_m_, task_max_m_, physical_max_m_);
    }
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "Invalid Isaac backend parameters: %s",
                 error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // No simulator state is implied before a received sample. In particular,
  // do not seed the command with the maximum aperture: Isaac may start with a
  // closed or unknown articulation and moving it would be unsolicited.
  position_state_ = nan();
  position_command_ = nan();
  physical_position_state_ = nan();
  measured_mechanism_position_state_ = nan();
  minimum_task_aperture_state_ = task_min_m_;
  maximum_task_aperture_state_ = task_max_m_;
  realtime_mode_command_ = nan();
  realtime_task_position_command_ = nan();
  realtime_task_velocity_command_ = nan();
  realtime_force_command_ = nan();
  realtime_sequence_command_ = nan();
  fault_recovery_sequence_command_ = nan();
  stop_sequence_command_ = nan();
  received_physical_velocity_ = nan();
  sample_age_state_ = nan();
  effort_state_ = nan();
  diagnostic_states_.fill(nan());
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OnRobotIsaacSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> result;
  result.emplace_back(task_joint_, hardware_interface::HW_IF_POSITION,
                      &position_state_);
  result.emplace_back(task_joint_, hardware_interface::HW_IF_VELOCITY,
                      &velocity_state_);
  if (hasState(info_.joints.front(), hardware_interface::HW_IF_EFFORT)) {
    result.emplace_back(task_joint_, hardware_interface::HW_IF_EFFORT,
                        &effort_state_);
  }
  if (hasState(info_.joints.front(), "task_position_valid")) {
    result.emplace_back(task_joint_, "task_position_valid",
                        &task_position_valid_state_);
    result.emplace_back(task_joint_, "force_valid", &force_valid_state_);
    result.emplace_back(task_joint_, "busy", &busy_state_);
    result.emplace_back(task_joint_, "grip_detected", &grip_detected_state_);
    if (hasState(info_.joints.front(), "safety_status_valid")) {
      result.emplace_back(task_joint_, "safety_status_valid",
                          &safety_status_valid_state_);
      result.emplace_back(task_joint_, "safety_1_pushed",
                          &safety_1_pushed_state_);
      result.emplace_back(task_joint_, "safety_1_triggered",
                          &safety_1_triggered_state_);
      result.emplace_back(task_joint_, "safety_2_pushed",
                          &safety_2_pushed_state_);
      result.emplace_back(task_joint_, "safety_2_triggered",
                          &safety_2_triggered_state_);
      result.emplace_back(task_joint_, "safety_dc_error",
                          &safety_dc_error_state_);
    }
    result.emplace_back(task_joint_, "active_mode", &active_mode_state_);
    result.emplace_back(task_joint_, "connection_state", &connection_state_);
    result.emplace_back(task_joint_, "faulted", &faulted_state_);
    result.emplace_back(task_joint_, "fault_code", &fault_code_state_);
    if (hasState(info_.joints.front(), "firmware_qualification")) {
      result.emplace_back(task_joint_, "firmware_qualification",
                          &firmware_qualification_state_);
    }
    if (hasState(info_.joints.front(), "realtime_force_control_available")) {
      result.emplace_back(task_joint_, "realtime_force_control_available",
                          &realtime_force_control_available_);
    }
    result.emplace_back(task_joint_, "sample_sequence",
                        &sample_sequence_state_);
    result.emplace_back(task_joint_, "sample_age", &sample_age_state_);
    result.emplace_back(task_joint_, "requested_command_sequence",
                        &requested_command_sequence_state_);
    result.emplace_back(task_joint_, "applied_command_sequence",
                        &applied_command_sequence_state_);
    result.emplace_back(task_joint_, "successful_cycles",
                        &successful_cycles_state_);
    result.emplace_back(task_joint_, "failed_cycles", &failed_cycles_state_);
    result.emplace_back(task_joint_, "missed_deadlines",
                        &missed_deadlines_state_);
    result.emplace_back(task_joint_, "watchdog_stops", &watchdog_stops_state_);
    result.emplace_back(task_joint_, "reconnects", &reconnects_state_);
    result.emplace_back(task_joint_, "last_cycle_duration",
                        &last_cycle_duration_state_);
  }
  result.emplace_back(task_joint_, "minimum_task_aperture",
                      &minimum_task_aperture_state_);
  result.emplace_back(task_joint_, "maximum_task_aperture",
                      &maximum_task_aperture_state_);
  result.emplace_back(physical_joint_, hardware_interface::HW_IF_POSITION,
                      &physical_position_state_);
  result.emplace_back(physical_joint_, hardware_interface::HW_IF_VELOCITY,
                      &physical_velocity_state_);
  result.emplace_back(physical_joint_,
                      rg_model_ ? "measured_angular_position"
                                : "measured_position",
                      &measured_mechanism_position_state_);
  result.emplace_back(physical_joint_,
                      rg_model_ ? "measured_angular_velocity"
                                : "measured_velocity",
                      &measured_mechanism_velocity_state_);
  result.emplace_back(physical_joint_, "position_valid",
                      &mechanism_position_valid_state_);
  result.emplace_back(physical_joint_, "velocity_valid",
                      &mechanism_velocity_valid_state_);
  for (std::size_t index = 0;
       index < onrobot_gripper_msgs::kDiagnosticInterfaceCount; ++index) {
    const auto name = onrobot_gripper_msgs::kDiagnosticInterfaceNames[index];
    if (hasState(info_.joints.front(), name)) {
      result.emplace_back(task_joint_, name, &diagnostic_states_[index]);
    }
  }
  return result;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
OnRobotIsaacSystem::on_export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> result;
  result.emplace_back(task_joint_, hardware_interface::HW_IF_POSITION,
                      &position_command_);
  if (hasCommand(info_.joints.front(), hardware_interface::HW_IF_EFFORT)) {
    result.emplace_back(task_joint_, hardware_interface::HW_IF_EFFORT,
                        &effort_command_);
  }
  if (hasCommand(info_.joints.front(), "realtime_mode")) {
    result.emplace_back(task_joint_, "realtime_mode", &realtime_mode_command_);
    result.emplace_back(task_joint_, "realtime_task_position",
                        &realtime_task_position_command_);
    if (hasCommand(info_.joints.front(), "realtime_task_velocity")) {
      result.emplace_back(task_joint_, "realtime_task_velocity",
                          &realtime_task_velocity_command_);
    }
    if (hasCommand(info_.joints.front(),
                   "realtime_mechanism_angular_velocity")) {
      result.emplace_back(task_joint_, "realtime_mechanism_angular_velocity",
                          &realtime_task_velocity_command_);
    }
    result.emplace_back(task_joint_, "realtime_force",
                        &realtime_force_command_);
    result.emplace_back(task_joint_, "realtime_command_sequence",
                        &realtime_sequence_command_);
  }
  if (hasCommand(info_.joints.front(), "fault_recovery_command_sequence")) {
    result.emplace_back(task_joint_, "fault_recovery_command_sequence",
                        &fault_recovery_sequence_command_);
  }
  if (hasCommand(info_.joints.front(), "stop_command_sequence")) {
    result.emplace_back(task_joint_, "stop_command_sequence",
                        &stop_sequence_command_);
  }
  if (hasCommand(info_.joints.front(), "conventional_command_sequence")) {
    result.emplace_back(task_joint_, "conventional_command_sequence",
                        &conventional_sequence_command_);
  }
  return command_stop_gate_.export_interfaces(std::move(result));
}

hardware_interface::CallbackReturn
OnRobotIsaacSystem::on_configure(const rclcpp_lifecycle::State &) {
  const auto node = get_node();
  command_publisher_ = node->create_publisher<sensor_msgs::msg::JointState>(
      command_topic_, rclcpp::QoS(1).reliable());
  state_subscription_ = node->create_subscription<sensor_msgs::msg::JointState>(
      state_topic_, rclcpp::QoS(1).reliable(),
      [this](const sensor_msgs::msg::JointState &message) {
        receiveJointState(message);
      });
  connection_state_ = 2.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
OnRobotIsaacSystem::on_activate(const rclcpp_lifecycle::State &) {
  active_ = true;
  // Activation always returns through waiting-for-feedback. The next read()
  // evaluates monotonic receipt freshness and seeds the idle hold. This also
  // prevents a cached pre-deactivation validity bit from publishing stale
  // state.
  if (!realtime_active_ && !std::isfinite(retired_position_command_) &&
      std::isfinite(position_command_)) {
    retired_position_command_ = position_command_;
  }
  feedback_ready_ = false;
  feedback_seeded_ = false;
  applied_feedback_available_ = false;
  position_command_ = nan();
  task_position_valid_state_ = 0.0;
  mechanism_position_valid_state_ = 0.0;
  mechanism_velocity_valid_state_ = 0.0;
  position_state_ = nan();
  velocity_state_ = nan();
  effort_state_ = nan();
  physical_position_state_ = nan();
  physical_velocity_state_ = nan();
  measured_mechanism_position_state_ = nan();
  measured_mechanism_velocity_state_ = nan();
  connection_state_ = 2.0;
  realtime_active_ = false;
  realtime_mode_ = -1;
  active_mode_state_ = 0.0;
  realtime_mode_command_ = nan();
  realtime_task_position_command_ = nan();
  realtime_task_velocity_command_ = nan();
  realtime_force_command_ = nan();
  realtime_sequence_command_ = nan();
  fault_recovery_sequence_command_ = nan();
  stop_sequence_command_ = nan();
  conventional_sequence_command_ = nan();
  force_conventional_command_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
OnRobotIsaacSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  if (command_publisher_) {
    if (feedback_ready_ && currentFeedbackAvailable()) {
      publishPosition(position_state_);
    } else {
      // Lifecycle deactivation can occur without a preceding read(). Never
      // refresh an expired cached position merely because its validity bit has
      // not yet been retired by the update loop.
      publishPhysicalCommand(nan(), 0.0);
    }
  }
  if (!realtime_active_ && !std::isfinite(retired_position_command_) &&
      std::isfinite(position_command_)) {
    retired_position_command_ = position_command_;
  }
  active_ = false;
  feedback_ready_ = false;
  feedback_seeded_ = false;
  realtime_active_ = false;
  realtime_mode_ = -1;
  active_mode_state_ = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
OnRobotIsaacSystem::read(const rclcpp::Time &, const rclcpp::Duration &) {
  double position = nan();
  double velocity = nan();
  std::chrono::steady_clock::time_point received_at;
  bool received = false;
  bool velocity_valid = false;
  bool fault_pending = false;
  bool source_reset = false;
  uint64_t sample_sequence = 0;
  bool ever_received = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    position = received_physical_position_;
    velocity = received_physical_velocity_;
    received_at = received_at_;
    received = received_state_;
    velocity_valid = received_velocity_valid_;
    fault_pending = feedback_fault_pending_;
    feedback_fault_pending_ = false;
    source_reset = source_reset_pending_;
    source_reset_pending_ = false;
    sample_sequence = received_sample_sequence_;
    ever_received = ever_received_state_;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto age = received ? now - received_at
                            : std::chrono::steady_clock::duration::zero();
  sample_age_state_ =
      received ? std::chrono::duration<double>(age).count() : nan();
  const bool valid = received && age <= state_timeout_;
  const bool lost = fault_pending || source_reset || (received && !valid);
  if (lost) {
    const bool had_usable_feedback = feedback_ready_;
    if (had_usable_feedback && !realtime_active_ &&
        !conventional_recovery_gate_ && std::isfinite(position_command_)) {
      retired_position_command_ = position_command_;
    }
    feedback_ready_ = false;
    feedback_seeded_ = false;
    applied_feedback_available_ = false;
    conventional_recovery_gate_ = false;
    realtime_active_ = false;
    realtime_mode_ = -1;
    active_mode_state_ = 0.0;
    busy_state_ = 0.0;
    position_command_ = nan();
    realtime_mode_command_ = nan();
    realtime_task_position_command_ = nan();
    realtime_task_velocity_command_ = nan();
    realtime_force_command_ = nan();
    realtime_sequence_command_ = nan();
    conventional_sequence_command_ = nan();
    force_conventional_command_ = false;
    if (had_usable_feedback || fault_pending || source_reset) {
      feedback_loss_stop_pending_ = active_;
      feedback_loss_stop_sent_ = false;
    }
    faulted_state_ = 1.0;
    fault_code_state_ = source_reset || fault_pending
                            ? faultCode(onrobot::ErrorCode::Protocol)
                            : faultCode(onrobot::ErrorCode::Timeout);
    task_position_valid_state_ = 0.0;
    mechanism_position_valid_state_ = 0.0;
    mechanism_velocity_valid_state_ = 0.0;
    position_state_ = nan();
    velocity_state_ = nan();
    effort_state_ = nan();
    physical_position_state_ = nan();
    physical_velocity_state_ = nan();
    measured_mechanism_position_state_ = nan();
    measured_mechanism_velocity_state_ = nan();
    connection_state_ = ever_received ? 1.0 : 2.0;
    updateDiagnosticStates();
    return hardware_interface::return_type::OK;
  }

  if (!valid) {
    // Waiting for the first sample is not itself a fault.
    task_position_valid_state_ = 0.0;
    mechanism_position_valid_state_ = 0.0;
    mechanism_velocity_valid_state_ = 0.0;
    position_state_ = nan();
    velocity_state_ = nan();
    effort_state_ = nan();
    physical_position_state_ = nan();
    physical_velocity_state_ = nan();
    measured_mechanism_position_state_ = nan();
    measured_mechanism_velocity_state_ = nan();
    connection_state_ = 2.0;
    updateDiagnosticStates();
    return hardware_interface::return_type::OK;
  }

  const bool reconnected = connection_state_ == 1.0;
  updateDerivedState(position, velocity);
  applied_feedback_received_at_ = received_at;
  applied_feedback_available_ = true;
  task_position_valid_state_ = 1.0;
  mechanism_position_valid_state_ = 1.0;
  mechanism_velocity_valid_state_ = velocity_valid ? 1.0 : 0.0;
  if (!velocity_valid) {
    velocity_state_ = nan();
    physical_velocity_state_ = nan();
    measured_mechanism_velocity_state_ = nan();
  }
  if (!feedback_seeded_) {
    if (!std::isfinite(conventional_sequence_command_)) {
      position_command_ = position_state_;
    }
    realtime_active_ = false;
    realtime_mode_ = -1;
    realtime_mode_command_ = nan();
    realtime_task_position_command_ = nan();
    realtime_task_velocity_command_ = nan();
    realtime_force_command_ = nan();
    realtime_sequence_command_ = nan();
    conventional_recovery_gate_ = std::isfinite(retired_position_command_);
    feedback_seeded_ = true;
  }
  connection_state_ = active_mode_state_ > 0.0 ? 4.0 : 3.0;
  if (reconnected) {
    reconnects_state_ += 1.0;
  }
  sample_sequence_state_ = static_cast<double>(sample_sequence);
  feedback_ready_ = faulted_state_ == 0.0;
  updateDiagnosticStates();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
OnRobotIsaacSystem::write(const rclcpp::Time &, const rclcpp::Duration &) {
  auto stop_lock = command_stop_gate_.try_lock();
  if (!stop_lock.owns_lock()) return hardware_interface::return_type::OK;
  if (!active_ || !command_publisher_) {
    return hardware_interface::return_type::OK;
  }

  if (std::isfinite(fault_recovery_sequence_command_)) {
    uint64_t sequence = 0;
    const bool valid_sequence =
        decodeCommandSequence(fault_recovery_sequence_command_, sequence);
    const bool fresh_sequence =
        valid_sequence && sequence != last_fault_recovery_sequence_;
    fault_recovery_sequence_command_ = nan();
    if (fresh_sequence && task_position_valid_state_ > 0.0 &&
        currentFeedbackAvailable()) {
      last_fault_recovery_sequence_ = sequence;
      faulted_state_ = 0.0;
      fault_code_state_ = 0.0;
      feedback_ready_ = true;
      feedback_seeded_ = true;
      position_command_ = position_state_;
      realtime_active_ = false;
      realtime_mode_ = -1;
      realtime_mode_command_ = nan();
      realtime_task_position_command_ = nan();
      realtime_task_velocity_command_ = nan();
      realtime_force_command_ = nan();
      realtime_sequence_command_ = nan();
      feedback_loss_stop_pending_ = false;
      feedback_loss_stop_sent_ = false;
      conventional_recovery_gate_ = std::isfinite(retired_position_command_);
      // Recovery itself may only seed the measured idle hold. Return before
      // any retained command interface value can be interpreted as new intent.
      publishPosition(position_state_);
      active_mode_state_ = 0.0;
      busy_state_ = 0.0;
      return hardware_interface::return_type::OK;
    }
  }

  if (!feedback_ready_) {
    if (!realtime_active_ && !std::isfinite(retired_position_command_) &&
        std::isfinite(position_command_)) {
      retired_position_command_ = position_command_;
    }
    if (feedback_loss_stop_pending_ && !feedback_loss_stop_sent_) {
      // Do not send a stale position target after feedback loss. One explicit
      // zero velocity lets the simulator stop, then the watchdog is allowed to
      // expire because no further commands are refreshed.
      publishPhysicalCommand(nan(), 0.0);
      feedback_loss_stop_sent_ = true;
    }
    return hardware_interface::return_type::OK;
  }
  bool stop_issued = false;

  if (!std::isnan(stop_sequence_command_)) {
    uint64_t sequence = 0;
    if (!decodeCommandSequence(stop_sequence_command_, sequence)) {
      stop_sequence_command_ = nan();
      return hardware_interface::return_type::ERROR;
    }
    (void)sequence;
    realtime_active_ = false;
    position_command_ = position_state_;
    conventional_sequence_command_ = nan();
    force_conventional_command_ = false;
    realtime_sequence_command_ = nan();
    publishPosition(position_state_);
    active_mode_state_ = 0.0;
    busy_state_ = 0.0;
    stop_issued = true;
    stop_sequence_command_ = nan();
  }

  if (!stop_issued && std::isfinite(conventional_sequence_command_)) {
    uint64_t sequence = 0;
    if (!decodeCommandSequence(conventional_sequence_command_, sequence)) {
      conventional_sequence_command_ = nan();
      return hardware_interface::return_type::ERROR;
    }
    (void)sequence;
    force_conventional_command_ = true;
    conventional_sequence_command_ = nan();
  }

  if (!stop_issued && std::isfinite(realtime_sequence_command_)) {
    uint64_t sequence = 0;
    const bool valid_sequence =
        decodeCommandSequence(realtime_sequence_command_, sequence);
    if (!valid_sequence) {
      if (!realtime_active_ && !conventional_recovery_gate_ &&
          std::isfinite(position_command_)) {
        retired_position_command_ = position_command_;
      }
      realtime_active_ = false;
      position_command_ = position_state_;
      faulted_state_ = 1.0;
      fault_code_state_ = faultCode(onrobot::ErrorCode::InvalidArgument);
      failed_cycles_state_ += 1.0;
      feedback_ready_ = false;
      feedback_loss_stop_pending_ = active_;
      feedback_loss_stop_sent_ = false;
      realtime_sequence_command_ = nan();
    } else if (sequence != last_realtime_sequence_) {
      const bool mode_numeric = std::isfinite(realtime_mode_command_);
      const bool mode_in_range =
          mode_numeric &&
          realtime_mode_command_ >=
              static_cast<double>(std::numeric_limits<int>::min()) &&
          realtime_mode_command_ <=
              static_cast<double>(std::numeric_limits<int>::max());
      realtime_mode_ =
          mode_in_range ? static_cast<int>(realtime_mode_command_) : -2;
      const bool mode_integral =
          mode_in_range &&
          realtime_mode_command_ == static_cast<double>(realtime_mode_);
      const bool valid_payload =
          std::isfinite(realtime_task_position_command_) &&
          std::isfinite(realtime_task_velocity_command_) &&
          std::isfinite(realtime_force_command_);
      if (mode_integral && valid_payload &&
          (realtime_mode_ == 0 || realtime_mode_ == 1)) {
        requested_command_sequence_state_ = static_cast<double>(sequence);
        realtime_active_ = true;
        applied_command_sequence_state_ = static_cast<double>(sequence);
      } else if (mode_integral && realtime_mode_ == -1) {
        requested_command_sequence_state_ = static_cast<double>(sequence);
        realtime_active_ = false;
        position_command_ = position_state_;
        applied_command_sequence_state_ = static_cast<double>(sequence);
      } else {
        // Reject unsupported modes and incomplete payloads instead of silently
        // falling through to conventional motion.
        if (!realtime_active_ && !conventional_recovery_gate_ &&
            std::isfinite(position_command_)) {
          retired_position_command_ = position_command_;
        }
        realtime_active_ = false;
        position_command_ = position_state_;
        faulted_state_ = 1.0;
        fault_code_state_ =
            mode_integral && valid_payload &&
                    (realtime_mode_ == 2 || realtime_mode_ == 3)
                ? faultCode(onrobot::ErrorCode::Unsupported)
                : faultCode(onrobot::ErrorCode::InvalidArgument);
        failed_cycles_state_ += 1.0;
        feedback_ready_ = false;
        feedback_loss_stop_pending_ = active_;
        feedback_loss_stop_sent_ = false;
      }
      last_realtime_sequence_ = sequence;
      realtime_sequence_command_ = nan();
    }
  }

  if (!feedback_ready_) {
    if (feedback_loss_stop_pending_ && !feedback_loss_stop_sent_) {
      publishPhysicalCommand(nan(), 0.0);
      feedback_loss_stop_sent_ = true;
    }
    return hardware_interface::return_type::OK;
  }

  if (realtime_active_ && realtime_mode_ == 0 &&
      std::isfinite(realtime_task_position_command_)) {
    publishPosition(realtime_task_position_command_);
    active_mode_state_ = 2.0;
    busy_state_ = 1.0;
    successful_cycles_state_ += 1.0;
  } else if (realtime_active_ && realtime_mode_ == 1 &&
             std::isfinite(realtime_task_velocity_command_)) {
    publishVelocity(realtime_task_velocity_command_);
    active_mode_state_ = 3.0;
    busy_state_ = std::abs(realtime_task_velocity_command_) > 1e-9 ? 1.0 : 0.0;
    successful_cycles_state_ += 1.0;
  } else {
    if (!force_conventional_command_ && conventional_recovery_gate_ &&
        (position_command_ == retired_position_command_ ||
         position_command_ == position_state_)) {
      publishPosition(position_state_);
      active_mode_state_ = 0.0;
      busy_state_ = 0.0;
      return hardware_interface::return_type::OK;
    }
    if (conventional_recovery_gate_ && std::isfinite(position_command_)) {
      conventional_recovery_gate_ = false;
      retired_position_command_ = nan();
    }
    publishPosition(stop_issued ? position_state_ : position_command_);
    active_mode_state_ =
        !stop_issued && std::abs(position_command_ - position_state_) > 1e-6
            ? 1.0
            : 0.0;
    busy_state_ = active_mode_state_;
    if (!stop_issued) {
      force_conventional_command_ = false;
    }
  }
  return hardware_interface::return_type::OK;
}

bool OnRobotIsaacSystem::currentFeedbackAvailable() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!applied_feedback_available_ || feedback_fault_pending_ ||
      source_reset_pending_) {
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  return now >= applied_feedback_received_at_ &&
         now - applied_feedback_received_at_ <= state_timeout_;
}

void OnRobotIsaacSystem::receiveJointState(
    const sensor_msgs::msg::JointState &message) {
  const auto item =
      std::find(message.name.begin(), message.name.end(), isaac_joint_);
  if (item == message.name.end() ||
      std::find(std::next(item), message.name.end(), isaac_joint_) !=
          message.name.end()) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    received_state_ = false;
    feedback_fault_pending_ = true;
    return;
  }
  const auto index = static_cast<size_t>(item - message.name.begin());
  if (index >= message.position.size() ||
      !std::isfinite(message.position[index])) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    received_state_ = false;
    feedback_fault_pending_ = true;
    return;
  }
  const double velocity =
      index < message.velocity.size() && std::isfinite(message.velocity[index])
          ? message.velocity[index]
          : nan();
  const bool velocity_valid = std::isfinite(velocity);
  const bool timestamp_well_formed = message.header.stamp.nanosec < 1000000000U;
  const int64_t source_stamp_ns =
      timestamp_well_formed ? sourceStampNanoseconds(message.header.stamp) : 0;
  if (!timestamp_well_formed || source_stamp_ns <= 0) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    received_state_ = false;
    feedback_fault_pending_ = true;
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  // Source timestamps order one simulator stream. Receipt time remains a
  // separate monotonic freshness clock; never compare the two directly.
  if (source_stamp_valid_ && source_stamp_ns == last_source_stamp_ns_) {
    return;
  }
  if (source_stamp_valid_ && source_stamp_ns < last_source_stamp_ns_) {
    last_source_stamp_ns_ = source_stamp_ns;
    received_state_ = false;
    source_reset_pending_ = true;
    feedback_fault_pending_ = true;
    return;
  }
  source_stamp_valid_ = true;
  last_source_stamp_ns_ = source_stamp_ns;
  received_physical_position_ = message.position[index];
  received_physical_velocity_ = velocity;
  received_velocity_valid_ = velocity_valid;
  received_at_ = std::chrono::steady_clock::now();
  received_state_ = true;
  ever_received_state_ = true;
  ++received_sample_sequence_;
}

void OnRobotIsaacSystem::publishPosition(double task_position_m) {
  if (rg_model_) {
    publishPhysicalCommand(rg_kinematics_->widthToFingerAngle(task_position_m),
                           nan());
    return;
  }
  const double ratio = std::clamp(
      (std::clamp(task_position_m, task_min_m_, task_max_m_) - task_min_m_) /
          (task_max_m_ - task_min_m_),
      0.0, 1.0);
  publishPhysicalCommand(
      physical_min_m_ + ratio * (physical_max_m_ - physical_min_m_), nan());
}

void OnRobotIsaacSystem::publishVelocity(double task_velocity_m_s) {
  if (rg_model_) {
    // RG realtime velocity is the physical mechanism's angular velocity.
    publishPhysicalCommand(nan(), task_velocity_m_s);
    return;
  }
  const double scale =
      (physical_max_m_ - physical_min_m_) / (task_max_m_ - task_min_m_);
  publishPhysicalCommand(nan(), task_velocity_m_s * scale);
}

void OnRobotIsaacSystem::publishPhysicalCommand(double physical_position_m,
                                                double physical_velocity_m_s) {
  sensor_msgs::msg::JointState command;
  command.header.stamp = get_clock()->now();
  command.name = {isaac_joint_};
  command.position = {physical_position_m};
  command.velocity = {physical_velocity_m_s};
  command.effort = {nan()};
  command_publisher_->publish(std::move(command));
}

void OnRobotIsaacSystem::updateDerivedState(double physical_position_m,
                                            double physical_velocity_m_s) {
  physical_position_state_ =
      std::clamp(physical_position_m, physical_min_m_, physical_max_m_);
  physical_velocity_state_ = physical_velocity_m_s;
  if (rg_model_) {
    position_state_ =
        rg_kinematics_->fingerAngleToWidth(physical_position_state_);
    constexpr double epsilon = 1e-6;
    const double lower_angle =
        std::max(physical_min_m_, physical_position_state_ - epsilon);
    const double upper_angle =
        std::min(physical_max_m_, physical_position_state_ + epsilon);
    const double width_derivative =
        (rg_kinematics_->fingerAngleToWidth(upper_angle) -
         rg_kinematics_->fingerAngleToWidth(lower_angle)) /
        (upper_angle - lower_angle);
    velocity_state_ = width_derivative * physical_velocity_m_s;
    physical_velocity_state_ = physical_velocity_m_s;
    measured_mechanism_position_state_ = physical_position_state_;
    measured_mechanism_velocity_state_ = physical_velocity_m_s;
    // Isaac does not provide a calibrated gripper-force measurement.
    effort_state_ = nan();
    return;
  }
  const double ratio = std::clamp((physical_position_state_ - physical_min_m_) /
                                      (physical_max_m_ - physical_min_m_),
                                  0.0, 1.0);
  const double velocity_scale =
      (task_max_m_ - task_min_m_) / (physical_max_m_ - physical_min_m_);
  position_state_ = task_min_m_ + ratio * (task_max_m_ - task_min_m_);
  velocity_state_ = physical_velocity_m_s * velocity_scale;
  measured_mechanism_position_state_ =
      raw_min_m_ + ratio * (raw_max_m_ - raw_min_m_);
  measured_mechanism_velocity_state_ = physical_velocity_m_s *
                                       (raw_max_m_ - raw_min_m_) /
                                       (physical_max_m_ - physical_min_m_);
  // Isaac does not provide a calibrated gripper-force measurement.
  effort_state_ = nan();
}

void OnRobotIsaacSystem::updateDiagnosticStates() {
  diagnostic_states_.fill(nan());
  setDiagnostic(diagnostic_states_, DiagnosticInterface::StatusValid,
                task_position_valid_state_ > 0.5 ? 1.0 : 0.0);
  setDiagnostic(diagnostic_states_, DiagnosticInterface::RawStatus, 0.0);
  setDiagnostic(diagnostic_states_, DiagnosticInterface::Busy, busy_state_);
  setDiagnostic(diagnostic_states_, DiagnosticInterface::GripDetected,
                grip_detected_state_);
  setDiagnostic(diagnostic_states_, DiagnosticInterface::SampleSequence,
                sample_sequence_state_);
  setDiagnostic(diagnostic_states_, DiagnosticInterface::Age,
                sample_age_state_);
}

bool OnRobotIsaacSystem::hasState(
    const hardware_interface::ComponentInfo &joint,
    const std::string &name) const {
  return std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
      [&name](const auto &interface) { return interface.name == name; });
}

bool OnRobotIsaacSystem::hasCommand(
    const hardware_interface::ComponentInfo &joint,
    const std::string &name) const {
  return std::any_of(
      joint.command_interfaces.begin(), joint.command_interfaces.end(),
      [&name](const auto &interface) { return interface.name == name; });
}

} // namespace onrobot_gripper_hardware

PLUGINLIB_EXPORT_CLASS(onrobot_gripper_hardware::OnRobotIsaacSystem,
                       hardware_interface::SystemInterface)

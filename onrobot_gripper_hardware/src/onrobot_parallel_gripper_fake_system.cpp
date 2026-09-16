#include "onrobot_gripper_hardware/onrobot_parallel_gripper_fake_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <onrobot_tool_api/diagnostics.hpp>
#include <onrobot_tool_api/model_capabilities.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "onrobot_gripper_hardware/command_sequence.hpp"

namespace onrobot_gripper_hardware {
namespace {

double parameter(const hardware_interface::HardwareInfo &info,
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

double nan() { return std::numeric_limits<double>::quiet_NaN(); }

using DiagnosticInterface = onrobot_gripper_msgs::DiagnosticInterface;

void setDiagnostic(
    std::array<double, onrobot_gripper_msgs::kDiagnosticInterfaceCount> &
        io_values,
    DiagnosticInterface i_interface, double i_value) {
  io_values[onrobot_gripper_msgs::diagnosticIndex(i_interface)] = i_value;
}

} // namespace

hardware_interface::CallbackReturn OnRobotParallelGripperFakeSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  info_ = params.hardware_info;
  const auto model = info_.hardware_parameters.find("model");
  model_ = model == info_.hardware_parameters.end() ? "" : model->second;
  is_2fg_ = model_ == "2fg7" || model_ == "2fg14";
  if (!is_2fg_ && model_ != "rg2" && model_ != "rg6") {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_parallel_gripper_fake_system"),
                 "fake model must be 2fg7, 2fg14, rg2, or rg6");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.joints.size() != 2 || info_.joints.front().name != "grip_stroke") {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_parallel_gripper_fake_system"),
                 "fake parallel gripper requires grip_stroke plus one visual joint");
    return hardware_interface::CallbackReturn::ERROR;
  }
  visual_joint_ = is_2fg_ ? "finger_stroke" : "finger_joint";
  if (is_2fg_) {
    const auto model = model_ == "2fg7" ? onrobot::Model::TwoFG7
                                        : onrobot::Model::TwoFG14;
    conventional_min_force_n_ = onrobot::minimumConventionalForceN(model);
    conventional_max_force_n_ = model == onrobot::Model::TwoFG7 ? 140.0 : 280.0;
  } else {
    conventional_min_force_n_ = 0.0;
    conventional_max_force_n_ = model_ == "rg2" ? 40.0 : 120.0;
  }
  if (info_.joints.back().name != visual_joint_) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    task_min_m_ = parameter(info_, "fake_task_min_m", 0.0);
    task_max_m_ = parameter(info_, "fake_task_max_m", 0.0);
    visual_joint_upper_ = parameter(
        info_, is_2fg_ ? "finger_joint_upper_m"
                       : "visual_finger_joint_upper_rad",
        0.0);
    raw_min_m_ = parameter(info_, "raw_linear_min_mm", 1.0) / 1000.0;
    raw_max_m_ = parameter(
                     info_, "raw_linear_max_mm",
                     model_ == "2fg14" ? 51.0 : 39.0) /
                 1000.0;
    geometry_aperture_at_zero_m_ = parameter(
        info_, "geometry_aperture_at_zero_m", model_ == "2fg14" ? 0.055 : 0.033);
    fake_motion_speed_m_s_ = parameter(info_, "fake_motion_speed_m_s", 0.0);
    fake_stall_ = parameter(info_, "fake_stall", 0.0) != 0.0;
    if (task_min_m_ < 0.0 || task_max_m_ <= task_min_m_ ||
        visual_joint_upper_ <= 0.0 ||
        (is_2fg_ && raw_max_m_ <= raw_min_m_) ||
        fake_motion_speed_m_s_ < 0.0) {
      throw std::invalid_argument("fake coordinate ranges are invalid");
    }
    if (is_2fg_) {
      // Restrict the usable fake envelope; do not rescale the physical stroke
      // when the caller narrows the task aperture. Standard jaws move equally.
      task_min_m_ = std::max(task_min_m_, geometry_aperture_at_zero_m_);
      task_max_m_ = std::min(task_max_m_,
                            geometry_aperture_at_zero_m_ + 2.0 * visual_joint_upper_);
      if (task_max_m_ <= task_min_m_)
        throw std::invalid_argument("fake 2FG task envelope has no physical geometry overlap");
    } else {
      rg_kinematics_ = std::make_unique<RgCadKinematics>(model_ == "rg6");
      if (visual_joint_upper_ >= 1.5707963267948966 + rg_kinematics_->cadZeroPhase()) {
        throw std::invalid_argument("fake RG joint range must remain monotonic");
      }
      // A fake measurement must match the gap the supplied CAD can display.
      // Narrowing the task envelope must not rescale the linkage geometry.
      task_max_m_ = std::min(task_max_m_,
          rg_kinematics_->fingerAngleToWidth(visual_joint_upper_));
      if (task_max_m_ <= task_min_m_) {
        throw std::invalid_argument("fake RG aperture exceeds CAD joint range");
      }
    }
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_parallel_gripper_fake_system"),
                 "invalid fake parameters: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  has_effort_ = hasCommand(info_.joints.front(),
                           hardware_interface::HW_IF_EFFORT);
  position_state_ = task_max_m_;
  position_command_ = position_state_;
  minimum_task_aperture_state_ = task_min_m_;
  maximum_task_aperture_state_ = task_max_m_;
  realtime_mode_command_ = nan();
  realtime_task_position_command_ = nan();
  realtime_task_velocity_command_ = nan();
  realtime_mechanism_angular_velocity_command_ = nan();
  realtime_force_command_ = nan();
  realtime_sequence_command_ = nan();
  fault_recovery_sequence_command_ = nan();
  stop_sequence_command_ = nan();
  conventional_sequence_command_ = nan();
  force_conventional_command_ = false;
  // Kinematics-only backend: requested effort is not a force measurement.
  force_valid_state_ = 0.0;
  effort_state_ = nan();
  realtime_force_control_available_ = is_2fg_ ? 1.0 : 0.0;
  updateDerivedState(0.0);
  updateDiagnosticStates();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OnRobotParallelGripperFakeSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> result;
  if (hasState(info_.joints.front(), "realtime_force_control_available")) {
    result.emplace_back("grip_stroke", "realtime_force_control_available",
                        &realtime_force_control_available_);
  }
  result.emplace_back("grip_stroke", hardware_interface::HW_IF_POSITION,
                      &position_state_);
  result.emplace_back("grip_stroke", hardware_interface::HW_IF_VELOCITY,
                      &velocity_state_);
  if (hasState(info_.joints.front(), hardware_interface::HW_IF_EFFORT)) {
    result.emplace_back("grip_stroke", hardware_interface::HW_IF_EFFORT,
                        &effort_state_);
  }
  if (hasState(info_.joints.front(), "task_position_valid")) {
    result.emplace_back("grip_stroke", "task_position_valid",
                        &task_position_valid_state_);
    result.emplace_back("grip_stroke", "force_valid", &force_valid_state_);
    result.emplace_back("grip_stroke", "busy", &busy_state_);
    result.emplace_back("grip_stroke", "grip_detected",
                        &grip_detected_state_);
    if (hasState(info_.joints.front(), "safety_status_valid")) {
      result.emplace_back("grip_stroke", "safety_status_valid",
                          &safety_status_valid_state_);
      result.emplace_back("grip_stroke", "safety_1_pushed",
                          &safety_1_pushed_state_);
      result.emplace_back("grip_stroke", "safety_1_triggered",
                          &safety_1_triggered_state_);
      result.emplace_back("grip_stroke", "safety_2_pushed",
                          &safety_2_pushed_state_);
      result.emplace_back("grip_stroke", "safety_2_triggered",
                          &safety_2_triggered_state_);
      result.emplace_back("grip_stroke", "safety_dc_error",
                          &safety_dc_error_state_);
    }
    result.emplace_back("grip_stroke", "active_mode", &active_mode_state_);
    result.emplace_back("grip_stroke", "connection_state",
                        &connection_state_);
    result.emplace_back("grip_stroke", "faulted", &faulted_state_);
    result.emplace_back("grip_stroke", "fault_code", &fault_code_state_);
    if (hasState(info_.joints.front(), "firmware_qualification")) {
      result.emplace_back("grip_stroke", "firmware_qualification",
                          &firmware_qualification_state_);
    }
    result.emplace_back("grip_stroke", "sample_sequence",
                        &sample_sequence_state_);
    result.emplace_back("grip_stroke", "sample_age", &sample_age_state_);
    result.emplace_back("grip_stroke", "requested_command_sequence",
                        &requested_command_sequence_state_);
    result.emplace_back("grip_stroke", "applied_command_sequence",
                        &applied_command_sequence_state_);
    result.emplace_back("grip_stroke", "successful_cycles",
                        &successful_cycles_state_);
    result.emplace_back("grip_stroke", "failed_cycles", &failed_cycles_state_);
    result.emplace_back("grip_stroke", "missed_deadlines",
                        &missed_deadlines_state_);
    result.emplace_back("grip_stroke", "watchdog_stops",
                        &watchdog_stops_state_);
    result.emplace_back("grip_stroke", "reconnects", &reconnects_state_);
    result.emplace_back("grip_stroke", "last_cycle_duration",
                        &last_cycle_duration_state_);
  }
  result.emplace_back("grip_stroke", "minimum_task_aperture",
                      &minimum_task_aperture_state_);
  result.emplace_back("grip_stroke", "maximum_task_aperture",
                      &maximum_task_aperture_state_);
  result.emplace_back(visual_joint_, hardware_interface::HW_IF_POSITION,
                      &visual_position_state_);
  result.emplace_back(visual_joint_, hardware_interface::HW_IF_VELOCITY,
                      &visual_velocity_state_);
  if (is_2fg_ && hasState(info_.joints.back(), "measured_position")) {
    result.emplace_back(visual_joint_, "measured_position",
                        &measured_mechanism_position_state_);
    result.emplace_back(visual_joint_, "measured_velocity",
                        &measured_mechanism_velocity_state_);
    result.emplace_back(visual_joint_, "position_valid",
                        &mechanism_position_valid_state_);
    result.emplace_back(visual_joint_, "velocity_valid",
                        &mechanism_velocity_valid_state_);
  } else if (!is_2fg_ &&
             hasState(info_.joints.back(), "measured_angular_position")) {
    result.emplace_back(visual_joint_, "measured_angular_position",
                        &measured_mechanism_angular_position_state_);
    result.emplace_back(visual_joint_, "measured_angular_velocity",
                        &measured_mechanism_angular_velocity_state_);
    result.emplace_back(visual_joint_, "position_valid",
                        &mechanism_position_valid_state_);
    result.emplace_back(visual_joint_, "velocity_valid",
                        &mechanism_velocity_valid_state_);
  }
  for (std::size_t index = 0;
       index < onrobot_gripper_msgs::kDiagnosticInterfaceCount; ++index) {
    const auto name = onrobot_gripper_msgs::kDiagnosticInterfaceNames[index];
    if (hasState(info_.joints.front(), name)) {
      result.emplace_back("grip_stroke", name, &diagnostic_states_[index]);
    }
  }
  return result;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
OnRobotParallelGripperFakeSystem::on_export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> result;
  result.emplace_back("grip_stroke", hardware_interface::HW_IF_POSITION,
                      &position_command_);
  if (has_effort_) {
    result.emplace_back("grip_stroke", hardware_interface::HW_IF_EFFORT,
                        &effort_command_);
  }
  if (hasCommand(info_.joints.front(), "realtime_mode")) {
    result.emplace_back("grip_stroke", "realtime_mode",
                        &realtime_mode_command_);
    result.emplace_back("grip_stroke", "realtime_task_position",
                        &realtime_task_position_command_);
    if (is_2fg_) {
      result.emplace_back("grip_stroke", "realtime_task_velocity",
                          &realtime_task_velocity_command_);
    } else {
      result.emplace_back("grip_stroke",
                          "realtime_mechanism_angular_velocity",
                          &realtime_mechanism_angular_velocity_command_);
    }
    result.emplace_back("grip_stroke", "realtime_force",
                        &realtime_force_command_);
    result.emplace_back("grip_stroke", "realtime_command_sequence",
                        &realtime_sequence_command_);
  }
  if (hasCommand(info_.joints.front(), "fault_recovery_command_sequence")) {
    result.emplace_back("grip_stroke", "fault_recovery_command_sequence",
                        &fault_recovery_sequence_command_);
  }
  if (hasCommand(info_.joints.front(), "stop_command_sequence")) {
    result.emplace_back("grip_stroke", "stop_command_sequence",
                        &stop_sequence_command_);
  }
  if (hasCommand(info_.joints.front(), "conventional_command_sequence")) {
    result.emplace_back("grip_stroke", "conventional_command_sequence",
                        &conventional_sequence_command_);
  }
  if (hasCommand(info_.joints.front(), "conventional_speed_percent")) {
    result.emplace_back("grip_stroke", "conventional_speed_percent",
                        &conventional_speed_percent_command_);
  }
  return command_stop_gate_.export_interfaces(std::move(result));
}

hardware_interface::CallbackReturn OnRobotParallelGripperFakeSystem::on_activate(
    const rclcpp_lifecycle::State &) {
  position_command_ = position_state_;
  velocity_state_ = 0.0;
  visual_velocity_state_ = 0.0;
  measured_mechanism_velocity_state_ = 0.0;
  realtime_active_ = false;
  active_mode_state_ = 0.0;
  conventional_sequence_command_ = nan();
  force_conventional_command_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type OnRobotParallelGripperFakeSystem::read(
    const rclcpp::Time &, const rclcpp::Duration &) {
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OnRobotParallelGripperFakeSystem::write(
    const rclcpp::Time &, const rclcpp::Duration &period) {
  auto stop_lock = command_stop_gate_.try_lock();
  if (!stop_lock.owns_lock()) return hardware_interface::return_type::OK;
  const double seconds = std::max(0.0, period.seconds());
  const double previous_position = position_state_;
  const double previous_visual = visual_position_state_;
  const double previous_mechanism = measured_mechanism_position_state_;
  bool stop_issued = false;

  if (std::isfinite(fault_recovery_sequence_command_)) {
    last_fault_recovery_sequence_ =
        static_cast<uint64_t>(fault_recovery_sequence_command_);
    fault_recovery_sequence_command_ = nan();
    faulted_state_ = 0.0;
    fault_code_state_ = 0.0;
  }

  if (!std::isnan(stop_sequence_command_)) {
    uint64_t sequence = 0;
    if (!decodeCommandSequence(stop_sequence_command_, sequence)) {
      stop_sequence_command_ = nan();
      return hardware_interface::return_type::ERROR;
    }
    (void)sequence;
    realtime_active_ = false;
    active_mode_state_ = 0.0;
    busy_state_ = 0.0;
    // Stop is a one-shot event, not a mode value. Drop every stale command
    // that could otherwise replay motion on the next fake write cycle.
    position_command_ = position_state_;
    conventional_sequence_command_ = nan();
    force_conventional_command_ = false;
    realtime_sequence_command_ = nan();
    stop_issued = true;
    stop_sequence_command_ = nan();
  }

  if (!stop_issued && conventional_sequence_command_ < 0.0 &&
      !std::isfinite(realtime_sequence_command_)) {
    return hardware_interface::return_type::OK;
  }
  if (!stop_issued && conventional_sequence_command_ > 0.0) {
    uint64_t sequence = 0;
    if (!decodeCommandSequence(conventional_sequence_command_, sequence)) {
      conventional_sequence_command_ = nan();
      return hardware_interface::return_type::ERROR;
    }
    if (!std::isfinite(effort_command_) || effort_command_ < 0.0 ||
        !std::isfinite(position_command_) || position_command_ < task_min_m_ ||
        position_command_ > task_max_m_) {
      position_command_ = position_state_;
      conventional_sequence_command_ = -conventional_sequence_command_;
      return hardware_interface::return_type::OK;
    }
    const double force = std::isfinite(effort_command_) && effort_command_ > 0.0
                             ? effort_command_
                             : conventional_min_force_n_;
    if (force < conventional_min_force_n_ || force > conventional_max_force_n_) {
      // The action controller may already have written the target position
      // before the one-shot intent marker reaches this backend.  Retire that
      // target as well as the marker; otherwise a rejected force request
      // would still move on the next fake write cycle.
      position_command_ = position_state_;
      conventional_sequence_command_ = -conventional_sequence_command_;
      return hardware_interface::return_type::OK;
    }
    if (is_2fg_ && (!std::isfinite(conventional_speed_percent_command_) ||
                    conventional_speed_percent_command_ < 1.0 ||
                    conventional_speed_percent_command_ > 100.0 ||
                    std::floor(conventional_speed_percent_command_) !=
                        conventional_speed_percent_command_)) {
      position_command_ = position_state_;
      conventional_sequence_command_ = -conventional_sequence_command_;
      return hardware_interface::return_type::OK;
    }
    (void)sequence;
    force_conventional_command_ = true;
    conventional_sequence_command_ = nan();
  }

  if (!stop_issued && std::isfinite(realtime_sequence_command_)) {
    uint64_t sequence = 0;
    if (!decodeCommandSequence(realtime_sequence_command_, sequence)) {
      realtime_sequence_command_ = nan();
      realtime_active_ = false;
      position_command_ = position_state_;
      return hardware_interface::return_type::ERROR;
    }
    if (sequence != last_realtime_sequence_) {
      realtime_mode_ = std::isfinite(realtime_mode_command_) &&
                              realtime_mode_command_ >= 0 &&
                              realtime_mode_command_ <= 3 &&
                              std::floor(realtime_mode_command_) == realtime_mode_command_
                           ? static_cast<int>(realtime_mode_command_)
                           : -1;
      if (realtime_mode_ == 2 || realtime_mode_ == 3) {
        const double minimumForce = model_ == "2fg14" ? 40.0 : 30.0;
        const double maximumForce = model_ == "2fg14" ? 196.0 : 95.0;
        const bool validForce = std::isfinite(realtime_force_command_) &&
            ((realtime_mode_ == 3 && realtime_force_command_ == 0.0) ||
             (realtime_force_command_ >= minimumForce &&
              realtime_force_command_ <= maximumForce));
        const bool validVelocity = std::isfinite(realtime_task_velocity_command_) &&
            std::abs(realtime_task_velocity_command_) <= 0.3 &&
            (realtime_mode_ == 3 || realtime_task_velocity_command_ >= 0.0);
        const bool validPosition = realtime_mode_ == 3 ||
            (std::isfinite(realtime_task_position_command_) &&
             realtime_task_position_command_ >= task_min_m_ &&
             realtime_task_position_command_ <= task_max_m_);
        if (!is_2fg_ || !validForce || !validVelocity || !validPosition) {
          realtime_sequence_command_ = nan();
          realtime_active_ = false;
          position_command_ = position_state_;
          return hardware_interface::return_type::ERROR;
        }
      }
      realtime_active_ = realtime_mode_ >= 0 &&
                         realtime_mode_ <= (is_2fg_ ? 3 : 1);
      if (!realtime_active_) {
        position_command_ = position_state_;
      }
      last_realtime_sequence_ = sequence;
      requested_command_sequence_state_ = static_cast<double>(sequence);
      applied_command_sequence_state_ = static_cast<double>(sequence);
      realtime_sequence_command_ = nan();
    }
  }

  if (realtime_active_) {
    if ((realtime_mode_ == 0 || realtime_mode_ == 2) &&
        std::isfinite(realtime_task_position_command_)) {
      const double target = std::clamp(realtime_task_position_command_,
                                       task_min_m_, task_max_m_);
      if (is_2fg_) {
        const double speed = std::isfinite(realtime_task_velocity_command_)
                                 ? std::clamp(realtime_task_velocity_command_, 0.0, 0.3)
                                 : 0.0;
        position_state_ += std::clamp(target - position_state_,
                                      -speed * seconds, speed * seconds);
      } else {
        position_state_ = target;
      }
    } else if (realtime_mode_ == 1 || realtime_mode_ == 3) {
      if (is_2fg_ && std::isfinite(realtime_task_velocity_command_)) {
        position_state_ = std::clamp(
            position_state_ + realtime_task_velocity_command_ * seconds,
            task_min_m_, task_max_m_);
      } else if (!is_2fg_ &&
                 std::isfinite(
                     realtime_mechanism_angular_velocity_command_)) {
        const double angle = std::clamp(
            visual_position_state_ +
                realtime_mechanism_angular_velocity_command_ * seconds,
            0.0, visual_joint_upper_);
        position_state_ = std::clamp(rg_kinematics_->fingerAngleToWidth(angle),
                                     task_min_m_, task_max_m_);
      }
    }
    effort_state_ = nan();
    busy_state_ = std::abs(position_state_ - previous_position) > 1e-12 ? 1.0 : 0.0;
    active_mode_state_ = static_cast<double>(realtime_mode_ + 2);
    successful_cycles_state_ += 1.0;
  } else {
    if (std::isfinite(position_command_)) {
      const double target =
          std::clamp(position_command_, task_min_m_, task_max_m_);
      if (!fake_stall_ && !stop_issued) {
        if (fake_motion_speed_m_s_ > 0.0 && seconds > 0.0) {
          const double maximum_step = fake_motion_speed_m_s_ * seconds;
          position_state_ += std::clamp(
              target - position_state_, -maximum_step, maximum_step);
        } else {
          position_state_ = target;
        }
      }
      force_conventional_command_ = false;
    }
    effort_state_ = nan();
    active_mode_state_ = !stop_issued &&
                                 std::abs(position_command_ - position_state_) >
                                     1e-9
                             ? 1.0
                             : 0.0;
    busy_state_ = active_mode_state_;
  }

  updateDerivedState(seconds);
  sample_sequence_state_ += 1.0;
  if (seconds > 0.0) {
    velocity_state_ = (position_state_ - previous_position) / seconds;
    visual_velocity_state_ =
        (visual_position_state_ - previous_visual) / seconds;
    measured_mechanism_velocity_state_ =
        (measured_mechanism_position_state_ - previous_mechanism) / seconds;
    measured_mechanism_angular_velocity_state_ = visual_velocity_state_;
  }
  updateDiagnosticStates();
  connection_state_ = active_mode_state_ > 0.0 ? 4.0 : 3.0;
  return hardware_interface::return_type::OK;
}

void OnRobotParallelGripperFakeSystem::updateDerivedState(double) {
  if (is_2fg_) {
    visual_position_state_ = (position_state_ - geometry_aperture_at_zero_m_) / 2.0;
    measured_mechanism_position_state_ =
        raw_min_m_ + 2.0 * visual_position_state_;
  } else {
    visual_position_state_ = rg_kinematics_->widthToFingerAngle(position_state_);
    measured_mechanism_angular_position_state_ =
        visual_position_state_ - rg_kinematics_->cadZeroPhase();
  }
}

void OnRobotParallelGripperFakeSystem::updateDiagnosticStates() {
  diagnostic_states_.fill(nan());
  setDiagnostic(diagnostic_states_, DiagnosticInterface::StatusValid, 1.0);
  setDiagnostic(diagnostic_states_, DiagnosticInterface::RawStatus, 0.0);
  setDiagnostic(diagnostic_states_, DiagnosticInterface::Busy, busy_state_);
  setDiagnostic(diagnostic_states_, DiagnosticInterface::GripDetected,
                grip_detected_state_);
  setDiagnostic(diagnostic_states_, DiagnosticInterface::SampleSequence,
                sample_sequence_state_);
  setDiagnostic(diagnostic_states_, DiagnosticInterface::Age, 0.0);
  if (std::isfinite(realtime_force_command_)) {
    setDiagnostic(diagnostic_states_, DiagnosticInterface::CommandForce,
                  realtime_force_command_);
    setDiagnostic(diagnostic_states_,
                  DiagnosticInterface::CommandForceValid, 1.0);
    setDiagnostic(diagnostic_states_, DiagnosticInterface::ForceProvenance,
                  static_cast<double>(onrobot::DiagnosticProvenance::CommandDerived));
  }
  if (force_valid_state_ > 0.5 && std::isfinite(effort_state_)) {
    setDiagnostic(diagnostic_states_, DiagnosticInterface::MeasuredForce,
                  effort_state_);
    setDiagnostic(diagnostic_states_,
                  DiagnosticInterface::MeasuredForceValid, 1.0);
    setDiagnostic(diagnostic_states_, DiagnosticInterface::ForceProvenance,
                  static_cast<double>(onrobot::DiagnosticProvenance::Measured));
  }
}

bool OnRobotParallelGripperFakeSystem::hasState(
    const hardware_interface::ComponentInfo &joint,
    const std::string &name) const {
  return std::any_of(joint.state_interfaces.begin(), joint.state_interfaces.end(),
                     [&name](const auto &interface) {
                       return interface.name == name;
                     });
}

bool OnRobotParallelGripperFakeSystem::hasCommand(
    const hardware_interface::ComponentInfo &joint,
    const std::string &name) const {
  return std::any_of(joint.command_interfaces.begin(),
                     joint.command_interfaces.end(),
                     [&name](const auto &interface) {
                       return interface.name == name;
                     });
}

} // namespace onrobot_gripper_hardware

PLUGINLIB_EXPORT_CLASS(
    onrobot_gripper_hardware::OnRobotParallelGripperFakeSystem,
    hardware_interface::SystemInterface)

#include "onrobot_gripper_hardware/onrobot_three_finger_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace onrobot_gripper_hardware {
namespace {
std::string parameter(const hardware_interface::HardwareInfo &info,
                      const std::string &name, const std::string &fallback) {
  const auto it = info.hardware_parameters.find(name);
  return it == info.hardware_parameters.end() ? fallback : it->second;
}
double number(const hardware_interface::HardwareInfo &info,
              const std::string &name, double fallback) {
  const auto text = parameter(info, name, std::to_string(fallback));
  size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (consumed != text.size() || !std::isfinite(value))
    throw std::invalid_argument(name + " must be finite");
  return value;
}
int integer(const hardware_interface::HardwareInfo &info,
            const std::string &name, int fallback) {
  const auto text = parameter(info, name, std::to_string(fallback));
  size_t consumed = 0;
  const long value = std::stol(text, &consumed);
  if (consumed != text.size())
    throw std::invalid_argument(name + " must be an integer");
  return static_cast<int>(value);
}
void set_nan(double &value) {
  value = std::numeric_limits<double>::quiet_NaN();
}

using DiagnosticInterface = onrobot_gripper_msgs::DiagnosticInterface;

void resetDiagnostics(
    std::array<double, onrobot_gripper_msgs::kDiagnosticInterfaceCount> &o_values) {
  for (double &value : o_values) {
    set_nan(value);
  }
  o_values[diagnosticIndex(DiagnosticInterface::StatusValid)] = 0.0;
}
} // namespace

OnRobotThreeFingerSystem::~OnRobotThreeFingerSystem() {
  requestStop();
  stopWorker();
}

hardware_interface::CallbackReturn OnRobotThreeFingerSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;
  m_info = params.hardware_info;
  if (m_info.joints.size() != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_three_finger_system"),
                 "3FG requires grip_diameter and finger_angle joints");
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto diameter = std::find_if(
      m_info.joints.begin(), m_info.joints.end(),
      [](const auto &joint) { return joint.name == "grip_diameter"; });
  const auto angle = std::find_if(
      m_info.joints.begin(), m_info.joints.end(),
      [](const auto &joint) { return joint.name == "finger_angle"; });
  if (diameter == m_info.joints.end() || angle == m_info.joints.end() ||
      !hasCommand(*diameter, hardware_interface::HW_IF_POSITION) ||
      !hasState(*diameter, hardware_interface::HW_IF_POSITION) ||
      !hasState(*angle, hardware_interface::HW_IF_POSITION)) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_three_finger_system"),
                 "3FG interface contract is invalid");
    return hardware_interface::CallbackReturn::ERROR;
  }
  m_hasForceCommand = hasCommand(*diameter, hardware_interface::HW_IF_EFFORT);
  set_nan(m_diameterState);
  set_nan(m_diameterVelocity);
  set_nan(m_fingerAngleState);
  set_nan(m_fingerAngleVelocity);
  set_nan(m_minimumExternalDiameterState);
  set_nan(m_maximumExternalDiameterState);
  resetDiagnostics(m_diagnosticStates);
  try {
    parseParameters();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_three_finger_system"),
                 "Invalid 3FG parameters: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OnRobotThreeFingerSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> out;
  out.emplace_back(m_diameterJoint, hardware_interface::HW_IF_POSITION,
                   &m_diameterState);
  out.emplace_back(m_diameterJoint, hardware_interface::HW_IF_VELOCITY,
                   &m_diameterVelocity);
  out.emplace_back(m_fingerJoint, hardware_interface::HW_IF_POSITION,
                   &m_fingerAngleState);
  out.emplace_back(m_fingerJoint, hardware_interface::HW_IF_VELOCITY,
                   &m_fingerAngleVelocity);
  out.emplace_back(m_diameterJoint, "minimum_external_diameter",
                   &m_minimumExternalDiameterState);
  out.emplace_back(m_diameterJoint, "maximum_external_diameter",
                   &m_maximumExternalDiameterState);
  const auto diameter = std::find_if(
      m_info.joints.begin(), m_info.joints.end(),
      [this](const auto &joint) { return joint.name == m_diameterJoint; });
  if (diameter != m_info.joints.end()) {
    const auto addDiameterState = [&out, this, &diameter](
                                      const char *name, double *value) {
      if (hasState(*diameter, name))
        out.emplace_back(m_diameterJoint, name, value);
    };
    addDiameterState("task_position_valid", &m_taskPositionValidState);
    addDiameterState("force_valid", &m_forceValidState);
    addDiameterState("busy", &m_busyState);
    addDiameterState("grip_detected", &m_gripDetectedState);
    addDiameterState("active_mode", &m_activeModeState);
    addDiameterState("connection_state", &m_connectionState);
    addDiameterState("faulted", &m_faultedState);
    addDiameterState("fault_code", &m_faultCodeState);
    addDiameterState("firmware_qualification",
                     &m_firmwareQualificationState);
    addDiameterState("sample_sequence", &m_sampleSequenceState);
    addDiameterState("sample_age", &m_sampleAgeState);
    addDiameterState("successful_cycles", &m_successfulCyclesState);
    addDiameterState("failed_cycles", &m_failedCyclesState);
    addDiameterState("missed_deadlines", &m_missedDeadlinesState);
    addDiameterState("watchdog_stops", &m_watchdogStopsState);
    addDiameterState("reconnects", &m_reconnectsState);
    addDiameterState("last_cycle_duration", &m_lastCycleDurationState);
    for (std::size_t index = 0;
         index < onrobot_gripper_msgs::kDiagnosticInterfaceCount; ++index) {
      if (hasState(*diameter,
                   onrobot_gripper_msgs::kDiagnosticInterfaceNames[index])) {
        out.emplace_back(m_diameterJoint,
                         onrobot_gripper_msgs::kDiagnosticInterfaceNames[index],
                         &m_diagnosticStates[index]);
      }
    }
  }
  const auto angle = std::find_if(
      m_info.joints.begin(), m_info.joints.end(),
      [this](const auto &joint) { return joint.name == m_fingerJoint; });
  if (angle != m_info.joints.end()) {
    if (hasState(*angle, "measured_angular_position"))
      out.emplace_back(m_fingerJoint, "measured_angular_position",
                       &m_measuredFingerAngleState);
    if (hasState(*angle, "measured_angular_velocity"))
      out.emplace_back(m_fingerJoint, "measured_angular_velocity",
                       &m_measuredFingerVelocityState);
    if (hasState(*angle, "position_valid"))
      out.emplace_back(m_fingerJoint, "position_valid",
                       &m_mechanismPositionValidState);
    if (hasState(*angle, "velocity_valid"))
      out.emplace_back(m_fingerJoint, "velocity_valid",
                       &m_mechanismVelocityValidState);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface>
OnRobotThreeFingerSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> out;
  out.emplace_back(m_diameterJoint, hardware_interface::HW_IF_POSITION,
                   &m_diameterCommand);
  if (m_hasForceCommand)
    out.emplace_back(m_diameterJoint, hardware_interface::HW_IF_EFFORT,
                     &m_forceCommand);
  return out;
}

hardware_interface::CallbackReturn
OnRobotThreeFingerSystem::on_configure(const rclcpp_lifecycle::State &) {
  stopWorker();
  try {
    m_gripper =
        std::make_unique<onrobot::ThreeFingerGripper>(m_model, m_connection);
    const auto initial_state = m_gripper->state();
    if (initial_state.fingertip_position != m_expectedFingertipPosition) {
      throw std::invalid_argument(
          "configured fingertip_position=" +
          std::to_string(m_expectedFingertipPosition) +
          " does not match device fingertip_position=" +
          std::to_string(initial_state.fingertip_position) +
          "; verify the physical mounting and relaunch with fingertip_position:=" +
          std::to_string(initial_state.fingertip_position));
    }
    publishState(initial_state);
    try {
      publishDiagnostics(m_gripper->diagnostics());
    } catch (const std::exception &error) {
      RCLCPP_WARN(rclcpp::get_logger("onrobot_three_finger_system"),
                  "Initial 3FG diagnostics unavailable: %s", error.what());
    }
    m_diameterCommand = m_diameterState;
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_three_finger_system"),
                 "3FG configure failed: %s", error.what());
    m_gripper.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
OnRobotThreeFingerSystem::on_activate(const rclcpp_lifecycle::State &) {
  if (!m_gripper)
    return hardware_interface::CallbackReturn::ERROR;
  stopWorker();
  try {
    m_gripper->stop();
    publishState(m_gripper->state());
    m_diameterCommand = m_diameterState;
    {
      std::lock_guard<std::mutex> lock(m_commandMutex);
      m_command = {m_diameterState, m_defaultForcePercent, 0, false, false};
    }
    m_shutdown.store(false);
    m_worker = std::thread(&OnRobotThreeFingerSystem::workerLoop, this);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_three_finger_system"),
                 "3FG activate failed: %s", error.what());
    invalidate();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
OnRobotThreeFingerSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  requestStop();
  stopWorker();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
OnRobotThreeFingerSystem::read(const rclcpp::Time &, const rclcpp::Duration &) {
  State state;
  {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    state = m_state;
  }
  if (!state.valid) {
    set_nan(m_diameterState);
    set_nan(m_diameterVelocity);
    set_nan(m_fingerAngleState);
    set_nan(m_fingerAngleVelocity);
    set_nan(m_minimumExternalDiameterState);
    set_nan(m_maximumExternalDiameterState);
    set_nan(m_taskPositionValidState);
    set_nan(m_forceValidState);
    set_nan(m_busyState);
    set_nan(m_gripDetectedState);
    set_nan(m_activeModeState);
    set_nan(m_connectionState);
    set_nan(m_faultedState);
    set_nan(m_faultCodeState);
    set_nan(m_firmwareQualificationState);
    set_nan(m_sampleSequenceState);
    set_nan(m_sampleAgeState);
    set_nan(m_successfulCyclesState);
    set_nan(m_failedCyclesState);
    set_nan(m_missedDeadlinesState);
    set_nan(m_watchdogStopsState);
    set_nan(m_reconnectsState);
    set_nan(m_lastCycleDurationState);
    set_nan(m_measuredFingerAngleState);
    set_nan(m_measuredFingerVelocityState);
    set_nan(m_mechanismPositionValidState);
    set_nan(m_mechanismVelocityValidState);
    resetDiagnostics(m_diagnosticStates);
    return hardware_interface::return_type::ERROR;
  }
  m_diameterState = state.diameter_m;
  m_fingerAngleState = state.finger_angle_rad;
  m_minimumExternalDiameterState = state.minimum_external_diameter_m;
  m_maximumExternalDiameterState = state.maximum_external_diameter_m;
  m_taskPositionValidState = 1.0;
  m_forceValidState = 0.0; // 3FG force is a percentage, not Newtons.
  m_activeModeState = 0.0;
  m_connectionState = 3.0; // connected and idle; 3FG has no realtime mode.
  m_faultedState = 0.0;
  m_faultCodeState = 0.0;
  m_firmwareQualificationState = 0.0;
  m_measuredFingerAngleState = state.finger_angle_rad;
  m_mechanismPositionValidState = 1.0;
  if (state.velocity_valid) {
    m_diameterVelocity = state.diameter_velocity_m_s;
    m_fingerAngleVelocity = state.angle_velocity_rad_s;
    m_measuredFingerVelocityState = state.angle_velocity_rad_s;
    m_mechanismVelocityValidState = 1.0;
  } else {
    set_nan(m_diameterVelocity);
    set_nan(m_fingerAngleVelocity);
    set_nan(m_measuredFingerVelocityState);
    m_mechanismVelocityValidState = 0.0;
  }
  resetDiagnostics(m_diagnosticStates);
  if (state.diagnostics_valid) {
    const auto age = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() -
                         state.diagnostics_stamp)
                         .count();
    const auto &diagnostics = state.diagnostics;
    m_busyState = diagnostics.busy ? 1.0 : 0.0;
    m_gripDetectedState = diagnostics.grip_detected ? 1.0 : 0.0;
    m_sampleSequenceState = static_cast<double>(state.diagnostics_sequence);
    m_sampleAgeState = std::isfinite(age) && age >= 0.0 ? age : 0.0;
    m_diagnosticStates[diagnosticIndex(DiagnosticInterface::StatusValid)] =
        diagnostics.status_valid ? 1.0 : 0.0;
    m_diagnosticStates[diagnosticIndex(DiagnosticInterface::RawStatus)] =
        diagnostics.raw_status;
    m_diagnosticStates[diagnosticIndex(DiagnosticInterface::Busy)] =
        diagnostics.busy ? 1.0 : 0.0;
    m_diagnosticStates[diagnosticIndex(DiagnosticInterface::GripDetected)] =
        diagnostics.grip_detected ? 1.0 : 0.0;
    m_diagnosticStates[
        diagnosticIndex(DiagnosticInterface::ThreeFingerForceGripDetected)] =
        diagnostics.force_grip_detected ? 1.0 : 0.0;
    m_diagnosticStates[
        diagnosticIndex(DiagnosticInterface::ThreeFingerCalibrationValid)] =
        diagnostics.calibration_valid ? 1.0 : 0.0;
    m_diagnosticStates[diagnosticIndex(DiagnosticInterface::ThreeFingerGripLost)] =
        diagnostics.grip_lost ? 1.0 : 0.0;
    m_diagnosticStates[diagnosticIndex(DiagnosticInterface::SampleSequence)] =
        static_cast<double>(state.diagnostics_sequence);
    if (std::isfinite(age) && age >= 0.0)
      m_diagnosticStates[diagnosticIndex(DiagnosticInterface::Age)] = age;
    auto put = [this](DiagnosticInterface i_interface,
                      const onrobot::DiagnosticValue &i_value) {
      if (i_value.valid && std::isfinite(i_value.value))
        m_diagnosticStates[diagnosticIndex(i_interface)] = i_value.value;
    };
    put(DiagnosticInterface::ForcePercent, diagnostics.force_percent);
    put(DiagnosticInterface::FingerAngle, diagnostics.finger_angle_rad);
    put(DiagnosticInterface::Diameter, diagnostics.diameter_mm);
    put(DiagnosticInterface::DiameterWithTipOffset,
        diagnostics.diameter_with_tip_offset_mm);
    put(DiagnosticInterface::Voltage24V, diagnostics.voltage_24v_v);
    put(DiagnosticInterface::MotorCurrent, diagnostics.current_24v_a);
    put(DiagnosticInterface::Temperature, diagnostics.temperature_c);
    put(DiagnosticInterface::MinimumExternalAperture,
        diagnostics.minimum_external_diameter_mm);
    put(DiagnosticInterface::MaximumExternalAperture,
        diagnostics.maximum_external_diameter_mm);
    put(DiagnosticInterface::MinimumInternalAperture,
        diagnostics.minimum_internal_diameter_mm);
    put(DiagnosticInterface::MaximumInternalAperture,
        diagnostics.maximum_internal_diameter_mm);
    put(DiagnosticInterface::CurrentExternalAperture,
        diagnostics.current_external_diameter_mm);
    put(DiagnosticInterface::CurrentInternalAperture,
        diagnostics.current_internal_diameter_mm);
    put(DiagnosticInterface::FingertipOffset, diagnostics.fingertip_offset_mm);
    put(DiagnosticInterface::BoostPowerLimit, diagnostics.boost_power_limit_w);
    m_diagnosticStates[diagnosticIndex(DiagnosticInterface::ThreeFingerPosition)] =
        diagnostics.fingertip_position;
    m_diagnosticStates[diagnosticIndex(DiagnosticInterface::StatisticsValid)] =
        0.0;
  } else {
    set_nan(m_busyState);
    set_nan(m_gripDetectedState);
    set_nan(m_sampleSequenceState);
    set_nan(m_sampleAgeState);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
OnRobotThreeFingerSystem::write(const rclcpp::Time &,
                                const rclcpp::Duration &) {
  if (!m_gripper)
    return hardware_interface::return_type::ERROR;
  // Controller-manager may present an uninitialised command before the
  // forward controller claims the interface. Seed it from measured state;
  // do not convert an idle startup into a physical command or hardware fault.
  if (!std::isfinite(m_diameterCommand)) {
    m_diameterCommand = m_diameterState;
    return hardware_interface::return_type::OK;
  }
  State state;
  {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    state = m_state;
  }
  if (!state.valid)
    return hardware_interface::return_type::ERROR;
  if (m_diameterCommand < state.minimum_external_diameter_m ||
      m_diameterCommand > state.maximum_external_diameter_m) {
    if (!m_hasRejectedDiameterCommand ||
        m_lastRejectedDiameterCommand != m_diameterCommand) {
      RCLCPP_ERROR(
          rclcpp::get_logger("onrobot_three_finger_system"),
          "Rejected 3FG external diameter %.1f mm; live range is %.1f..%.1f mm",
          m_diameterCommand * 1000.0,
          state.minimum_external_diameter_m * 1000.0,
          state.maximum_external_diameter_m * 1000.0);
      m_lastRejectedDiameterCommand = m_diameterCommand;
      m_hasRejectedDiameterCommand = true;
    }
    return hardware_interface::return_type::OK;
  }
  m_hasRejectedDiameterCommand = false;
  const double force =
      m_hasForceCommand && std::isfinite(m_forceCommand) && m_forceCommand > 0.0
          ? m_forceCommand
          : m_defaultForcePercent;
  if (!std::isfinite(force) || force < 1.0 || force > 100.0)
    return hardware_interface::return_type::ERROR;
  std::lock_guard<std::mutex> lock(m_commandMutex);
  if (m_command.available && m_command.diameter_m == m_diameterCommand &&
      m_command.force_percent == force)
    return hardware_interface::return_type::OK;
  m_command = {m_diameterCommand, force, m_command.seq + 1, true, false};
  return hardware_interface::return_type::OK;
}

void OnRobotThreeFingerSystem::workerLoop() {
  uint64_t applied = 0;
  auto next = std::chrono::steady_clock::now();
  auto nextDiagnostics = next;
  while (true) {
    Command command;
    {
      std::lock_guard<std::mutex> lock(m_commandMutex);
      command = m_command;
    }
    try {
      if (command.stop) {
        m_gripper->stop();
        applied = command.seq;
      } else if (command.available && command.seq != applied) {
        m_gripper->moveDiameter({onrobot::ThreeFingerGripType::External,
                                 onrobot::ThreeFingerDiameterCommand::Move,
                                 command.diameter_m * 1000.0,
                                 command.force_percent});
        applied = command.seq;
      }
      publishState(m_gripper->state());
      if (std::chrono::steady_clock::now() >= nextDiagnostics) {
        try {
          publishDiagnostics(m_gripper->diagnostics());
        } catch (const std::exception &error) {
          std::lock_guard<std::mutex> lock(m_stateMutex);
          m_state.diagnostics_valid = false;
          RCLCPP_WARN(rclcpp::get_logger("onrobot_three_finger_system"),
                      "3FG diagnostics unavailable: %s", error.what());
        }
        nextDiagnostics = std::chrono::steady_clock::now() +
                          std::chrono::seconds(1);
      }
    } catch (const onrobot::DeviceError &error) {
      applied = command.seq;
      if (error.code() == onrobot::ErrorCode::InvalidArgument) {
        RCLCPP_ERROR(rclcpp::get_logger("onrobot_three_finger_system"),
                     "Rejected 3FG command: %s", error.what());
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("onrobot_three_finger_system"),
                     "3FG worker failed: %s", error.what());
        invalidate();
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR(rclcpp::get_logger("onrobot_three_finger_system"),
                   "3FG worker failed: %s", error.what());
      invalidate();
    }
    if (m_shutdown.load())
      break;
    next += m_pollPeriod;
    const auto now = std::chrono::steady_clock::now();
    if (now < next)
      std::this_thread::sleep_until(next);
    else
      next = now;
  }
}

void OnRobotThreeFingerSystem::publishState(
    const onrobot::ThreeFingerState &source) {
  State state;
  state.diameter_m = source.current_external_diameter_mm / 1000.0;
  state.finger_angle_rad = std::clamp(
      m_urdfFingerAngleScale * source.finger_angle_rad +
          m_urdfFingerAngleOffset,
      m_urdfFingerAngleMinimum, m_urdfFingerAngleMaximum);
  state.minimum_external_diameter_m =
      source.minimum_external_diameter_mm / 1000.0;
  state.maximum_external_diameter_m =
      source.maximum_external_diameter_mm / 1000.0;
  state.fingertip_position = source.fingertip_position;
  state.stamp = std::chrono::steady_clock::now();
  state.valid =
      std::isfinite(state.diameter_m) &&
      std::isfinite(state.finger_angle_rad) &&
      std::isfinite(state.minimum_external_diameter_m) &&
      std::isfinite(state.maximum_external_diameter_m) &&
      state.minimum_external_diameter_m >= 0.0 &&
      state.maximum_external_diameter_m >= state.minimum_external_diameter_m;
  std::lock_guard<std::mutex> lock(m_stateMutex);
  if (m_state.valid) {
    const double seconds =
        std::chrono::duration<double>(state.stamp - m_state.stamp).count();
    if (seconds > 0.0) {
      state.diameter_velocity_m_s =
          (state.diameter_m - m_state.diameter_m) / seconds;
      state.angle_velocity_rad_s =
          (state.finger_angle_rad - m_state.finger_angle_rad) / seconds;
      state.velocity_valid = std::isfinite(state.diameter_velocity_m_s) &&
                             std::isfinite(state.angle_velocity_rad_s);
    }
  }
  // Diagnostics are refreshed at a lower rate than the motion state. Keep
  // the last coherent device snapshot while publishing the next state sample.
  state.diagnostics = m_state.diagnostics;
  state.diagnostics_stamp = m_state.diagnostics_stamp;
  state.diagnostics_sequence = m_state.diagnostics_sequence;
  state.diagnostics_valid = m_state.diagnostics_valid;
  m_state = state;
}

void OnRobotThreeFingerSystem::publishDiagnostics(
    const onrobot::ThreeFingerDiagnostics &source) {
  std::lock_guard<std::mutex> lock(m_stateMutex);
  m_state.diagnostics = source;
  m_state.diagnostics_stamp = std::chrono::steady_clock::now();
  ++m_state.diagnostics_sequence;
  m_state.diagnostics_valid = source.status_valid;
}

void OnRobotThreeFingerSystem::invalidate() noexcept {
  std::lock_guard<std::mutex> lock(m_stateMutex);
  m_state.valid = false;
}
void OnRobotThreeFingerSystem::requestStop() noexcept {
  std::lock_guard<std::mutex> lock(m_commandMutex);
  m_command = {m_command.diameter_m, m_command.force_percent, m_command.seq + 1,
               false, true};
}
void OnRobotThreeFingerSystem::stopWorker() noexcept {
  if (!m_worker.joinable())
    return;
  requestStop();
  m_shutdown.store(true);
  m_worker.join();
}
bool OnRobotThreeFingerSystem::hasCommand(
    const hardware_interface::ComponentInfo &joint,
    const std::string &name) const {
  return std::any_of(
      joint.command_interfaces.begin(), joint.command_interfaces.end(),
      [&name](const auto &interface) { return interface.name == name; });
}
bool OnRobotThreeFingerSystem::hasState(
    const hardware_interface::ComponentInfo &joint,
    const std::string &name) const {
  return std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
      [&name](const auto &interface) { return interface.name == name; });
}

bool OnRobotThreeFingerSystem::parseParameters() {
  const auto model = parameter(m_info, "model", "3fg25");
  if (model == "3fg25")
    m_model = onrobot::Model::ThreeFG25;
  else if (model == "3fg15")
    m_model = onrobot::Model::ThreeFG15;
  else
    throw std::invalid_argument("model must be '3fg15' or '3fg25'");
  m_defaultForcePercent = number(m_info, "default_force_percent", 20.0);
  m_urdfFingerAngleScale = number(m_info, "urdf_finger_angle_scale", -1.0);
  m_urdfFingerAngleOffset = number(m_info, "urdf_finger_angle_offset", 2.70526);
  m_urdfFingerAngleMinimum =
      number(m_info, "urdf_finger_angle_minimum", 0.0);
  m_urdfFingerAngleMaximum =
      number(m_info, "urdf_finger_angle_maximum", 2.70526);
  m_expectedFingertipPosition = integer(m_info, "fingertip_position", 3);
  const int poll_ms = integer(m_info, "poll_period_ms", 20);
  if (m_defaultForcePercent < 1.0 || m_defaultForcePercent > 100.0 ||
      poll_ms <= 0 || !std::isfinite(m_urdfFingerAngleScale) ||
      !std::isfinite(m_urdfFingerAngleOffset) ||
      !std::isfinite(m_urdfFingerAngleMinimum) ||
      !std::isfinite(m_urdfFingerAngleMaximum) ||
      m_urdfFingerAngleScale == 0.0 || m_expectedFingertipPosition < 1 ||
      m_expectedFingertipPosition > 3 ||
      m_urdfFingerAngleMinimum >= m_urdfFingerAngleMaximum)
    throw std::invalid_argument("3FG force, visual-angle mapping, fingertip "
                                "position, or poll period is invalid");
  if (parameter(m_info, "transport", "tcp") != "tcp")
    throw std::invalid_argument(
        "3FG ROS adapter currently supports transport=tcp only");
  const auto host = parameter(m_info, "host", "127.0.0.1");
  const int port = integer(m_info, "port", 502);
  const int slave = integer(m_info, "slave_id", 65);
  if (host.empty() || port < 1 || port > 65535 || slave < 65 || slave > 67)
    throw std::invalid_argument("3FG TCP endpoint or slave id is invalid");
  m_connection = onrobot::tcp(host, static_cast<uint16_t>(port),
                              static_cast<onrobot::ModbusSlaveId>(slave));
  m_pollPeriod = std::chrono::milliseconds(poll_ms);
  return true;
}
} // namespace onrobot_gripper_hardware

PLUGINLIB_EXPORT_CLASS(onrobot_gripper_hardware::OnRobotThreeFingerSystem,
                       hardware_interface::SystemInterface)

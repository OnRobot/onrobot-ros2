#include "onrobot_gripper_hardware/onrobot_rg_system.hpp"
#include "onrobot_gripper_hardware/identity_state.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "onrobot_gripper_hardware/command_sequence.hpp"

namespace onrobot_gripper_hardware {
namespace {

constexpr char kRealtimeMode[] = "realtime_mode";
constexpr char kRealtimeTaskPosition[] = "realtime_task_position";
constexpr char kRealtimeMechanismAngularVelocity[] =
    "realtime_mechanism_angular_velocity";
constexpr char kRealtimeForce[] = "realtime_force";
constexpr char kRealtimeSequence[] = "realtime_command_sequence";
constexpr char kFaultRecoverySequence[] = "fault_recovery_command_sequence";
constexpr char kStopSequence[] = "stop_command_sequence";
constexpr char kConventionalSequence[] = "conventional_command_sequence";

std::string parameter(const hardware_interface::HardwareInfo &i_info,
                      const std::string &i_name,
                      const std::string &i_fallback) {
  const auto item = i_info.hardware_parameters.find(i_name);
  return item == i_info.hardware_parameters.end() ? i_fallback : item->second;
}

double number(const hardware_interface::HardwareInfo &i_info,
              const std::string &i_name, double i_fallback) {
  const auto text = parameter(i_info, i_name, std::to_string(i_fallback));
  size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (consumed != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument(i_name + " must be finite");
  }
  return value;
}

int integer(const hardware_interface::HardwareInfo &i_info,
            const std::string &i_name, int i_fallback) {
  const auto text = parameter(i_info, i_name, std::to_string(i_fallback));
  size_t consumed = 0;
  const long value = std::stol(text, &consumed);
  if (consumed != text.size() || value < std::numeric_limits<int>::min() ||
      value > std::numeric_limits<int>::max()) {
    throw std::invalid_argument(i_name + " must be an integer");
  }
  return static_cast<int>(value);
}

void setNan(double &o_value) {
  o_value = std::numeric_limits<double>::quiet_NaN();
}

using DiagnosticInterface = onrobot_gripper_msgs::DiagnosticInterface;

void resetDiagnostics(
    std::array<double, onrobot_gripper_msgs::kDiagnosticInterfaceCount> &o_values) {
  for (double &value : o_values) {
    setNan(value);
  }
  o_values[diagnosticIndex(DiagnosticInterface::StatusValid)] = 0.0;
  o_values[diagnosticIndex(DiagnosticInterface::IdentityValid)] = 0.0;
}

double connectionState(const onrobot::ParallelGripperState &i_image) {
  switch (i_image.session_state) {
  case onrobot::SessionState::Configured:
    return 1.0; // GripperState::CONNECTION_DISCONNECTED
  case onrobot::SessionState::Active:
    return i_image.active_mode == onrobot::ParallelControlMode::Idle ? 3.0
                                                                     : 4.0;
  case onrobot::SessionState::Recovering:
    return 5.0;
  case onrobot::SessionState::Faulted:
    return 6.0;
  }
  return 0.0;
}

} // namespace

OnRobotRgSystem::~OnRobotRgSystem() {
  if (m_session) {
    m_session->stop();
    m_session->deactivate();
  }
}

hardware_interface::CallbackReturn OnRobotRgSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &i_params) {
  if (hardware_interface::SystemInterface::on_init(i_params) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  m_info = i_params.hardware_info;
  if (m_info.joints.empty() || m_info.joints.size() > 2) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_rg_system"),
                 "RG requires grip_stroke and optional finger_joint");
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto aperture = std::find_if(
      m_info.joints.begin(), m_info.joints.end(),
      [](const auto &i_joint) { return i_joint.name == "grip_stroke"; });
  const auto visual = std::find_if(
      m_info.joints.begin(), m_info.joints.end(),
      [](const auto &i_joint) { return i_joint.name == "finger_joint"; });
  if (aperture == m_info.joints.end() ||
      !hasCommand(*aperture, hardware_interface::HW_IF_POSITION) ||
      !hasState(*aperture, hardware_interface::HW_IF_POSITION)) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_rg_system"),
                 "RG grip_stroke requires position command and state");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (visual != m_info.joints.end() &&
      !hasState(*visual, hardware_interface::HW_IF_POSITION)) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_rg_system"),
                 "RG finger_joint must be state-only with position state");
    return hardware_interface::CallbackReturn::ERROR;
  }
  m_apertureJointName = aperture->name;
  m_hasEffortCommand = hasCommand(*aperture, hardware_interface::HW_IF_EFFORT);
  m_hasVisualJoint = visual != m_info.joints.end();
  setNan(m_positionState);
  setNan(m_velocityState);
  setNan(m_fingerAngleState);
  setNan(m_fingerVelocityState);
  setNan(m_effortState);
  setNan(m_measuredAngularPositionState);
  setNan(m_measuredAngularVelocityState);
  setNan(m_realtimeModeCommand);
  setNan(m_realtimeTaskPositionCommand);
  setNan(m_realtimeMechanismAngularVelocityCommand);
  setNan(m_realtimeForceCommand);
  setNan(m_realtimeSequenceCommand);
  setNan(m_faultRecoverySequenceCommand);
  setNan(m_stopSequenceCommand);
  resetDiagnostics(m_diagnosticStates);
  try {
    parseParameters();
  } catch (const std::exception &i_error) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_rg_system"),
                 "Invalid RG parameters: %s", i_error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OnRobotRgSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(m_apertureJointName,
                          hardware_interface::HW_IF_POSITION, &m_positionState);
  const auto aperture = std::find_if(
      m_info.joints.begin(), m_info.joints.end(),
      [this](const auto &joint) { return joint.name == m_apertureJointName; });
  if (aperture != m_info.joints.end() &&
      hasState(*aperture, hardware_interface::HW_IF_VELOCITY)) {
    interfaces.emplace_back(m_apertureJointName,
                            hardware_interface::HW_IF_VELOCITY,
                            &m_velocityState);
  }
  if (aperture != m_info.joints.end() &&
      hasState(*aperture, hardware_interface::HW_IF_EFFORT)) {
    interfaces.emplace_back(m_apertureJointName,
                            hardware_interface::HW_IF_EFFORT, &m_effortState);
  }
  if (aperture != m_info.joints.end() &&
      hasState(*aperture, "task_position_valid")) {
    interfaces.emplace_back(m_apertureJointName, "task_position_valid",
                            &m_taskPositionValidState);
    interfaces.emplace_back(m_apertureJointName, "force_valid",
                            &m_forceValidState);
    interfaces.emplace_back(m_apertureJointName, "busy", &m_busyState);
    interfaces.emplace_back(m_apertureJointName, "grip_detected",
                            &m_gripDetectedState);
    interfaces.emplace_back(m_apertureJointName, "safety_status_valid",
                            &m_safetyStatusValidState);
    interfaces.emplace_back(m_apertureJointName, "safety_1_pushed",
                            &m_safety1PushedState);
    interfaces.emplace_back(m_apertureJointName, "safety_1_triggered",
                            &m_safety1TriggeredState);
    interfaces.emplace_back(m_apertureJointName, "safety_2_pushed",
                            &m_safety2PushedState);
    interfaces.emplace_back(m_apertureJointName, "safety_2_triggered",
                            &m_safety2TriggeredState);
    interfaces.emplace_back(m_apertureJointName, "safety_dc_error",
                            &m_safetyDcErrorState);
    interfaces.emplace_back(m_apertureJointName, "active_mode",
                            &m_activeModeState);
    interfaces.emplace_back(m_apertureJointName, "connection_state",
                            &m_connectionState);
    interfaces.emplace_back(m_apertureJointName, "faulted", &m_faultedState);
    interfaces.emplace_back(m_apertureJointName, "fault_code",
                            &m_faultCodeState);
    if (hasState(*aperture, "firmware_qualification")) {
      interfaces.emplace_back(m_apertureJointName, "firmware_qualification",
                              &m_firmwareQualificationState);
    }
    interfaces.emplace_back(m_apertureJointName, "sample_sequence",
                            &m_sampleSequenceState);
    interfaces.emplace_back(m_apertureJointName, "sample_age",
                            &m_sampleAgeState);
    interfaces.emplace_back(m_apertureJointName, "requested_command_sequence",
                            &m_requestedCommandSequenceState);
    interfaces.emplace_back(m_apertureJointName, "applied_command_sequence",
                            &m_appliedCommandSequenceState);
    interfaces.emplace_back(m_apertureJointName, "successful_cycles",
                            &m_successfulCyclesState);
    interfaces.emplace_back(m_apertureJointName, "failed_cycles",
                            &m_failedCyclesState);
    interfaces.emplace_back(m_apertureJointName, "missed_deadlines",
                            &m_missedDeadlinesState);
    interfaces.emplace_back(m_apertureJointName, "watchdog_stops",
                            &m_watchdogStopsState);
    interfaces.emplace_back(m_apertureJointName, "reconnects",
                            &m_reconnectsState);
    interfaces.emplace_back(m_apertureJointName, "last_cycle_duration",
                            &m_lastCycleDurationState);
  }
  if (aperture != m_info.joints.end() &&
      hasState(*aperture, "minimum_task_aperture")) {
    interfaces.emplace_back(m_apertureJointName, "minimum_task_aperture",
                            &m_minimumTaskApertureState);
    interfaces.emplace_back(m_apertureJointName, "maximum_task_aperture",
                            &m_maximumTaskApertureState);
  }
  if (aperture != m_info.joints.end()) {
    for (std::size_t index = 0;
         index < onrobot_gripper_msgs::kDiagnosticInterfaceCount; ++index) {
      if (hasState(*aperture,
                   onrobot_gripper_msgs::kDiagnosticInterfaceNames[index])) {
        interfaces.emplace_back(
            m_apertureJointName,
            onrobot_gripper_msgs::kDiagnosticInterfaceNames[index],
            &m_diagnosticStates[index]);
      }
    }
  }
  if (m_hasVisualJoint) {
    interfaces.emplace_back(m_visualJointName,
                            hardware_interface::HW_IF_POSITION,
                            &m_fingerAngleState);
    const auto visual = std::find_if(
        m_info.joints.begin(), m_info.joints.end(),
        [this](const auto &joint) { return joint.name == m_visualJointName; });
    if (visual != m_info.joints.end() &&
        hasState(*visual, hardware_interface::HW_IF_VELOCITY)) {
      interfaces.emplace_back(m_visualJointName,
                              hardware_interface::HW_IF_VELOCITY,
                              &m_fingerVelocityState);
    }
    if (visual != m_info.joints.end() &&
        hasState(*visual, "measured_angular_position")) {
      interfaces.emplace_back(m_visualJointName, "measured_angular_position",
                              &m_measuredAngularPositionState);
      interfaces.emplace_back(m_visualJointName, "measured_angular_velocity",
                              &m_measuredAngularVelocityState);
      interfaces.emplace_back(m_visualJointName, "position_valid",
                              &m_mechanismAngularPositionValidState);
      interfaces.emplace_back(m_visualJointName, "velocity_valid",
                              &m_mechanismAngularVelocityValidState);
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
OnRobotRgSystem::on_export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  const auto aperture = std::find_if(
      m_info.joints.begin(), m_info.joints.end(),
      [this](const auto &joint) { return joint.name == m_apertureJointName; });
  interfaces.emplace_back(m_apertureJointName,
                          hardware_interface::HW_IF_POSITION,
                          &m_positionCommand);
  if (m_hasEffortCommand) {
    interfaces.emplace_back(m_apertureJointName,
                            hardware_interface::HW_IF_EFFORT, &m_effortCommand);
  }
  if (aperture != m_info.joints.end() && hasCommand(*aperture, kRealtimeMode)) {
    interfaces.emplace_back(m_apertureJointName, kRealtimeMode,
                            &m_realtimeModeCommand);
    interfaces.emplace_back(m_apertureJointName, kRealtimeTaskPosition,
                            &m_realtimeTaskPositionCommand);
    interfaces.emplace_back(m_apertureJointName,
                            kRealtimeMechanismAngularVelocity,
                            &m_realtimeMechanismAngularVelocityCommand);
    interfaces.emplace_back(m_apertureJointName, kRealtimeForce,
                            &m_realtimeForceCommand);
    interfaces.emplace_back(m_apertureJointName, kRealtimeSequence,
                            &m_realtimeSequenceCommand);
  }
  if (aperture != m_info.joints.end() &&
      hasCommand(*aperture, kFaultRecoverySequence)) {
    interfaces.emplace_back(m_apertureJointName, kFaultRecoverySequence,
                            &m_faultRecoverySequenceCommand);
  }
  if (aperture != m_info.joints.end() && hasCommand(*aperture, kStopSequence)) {
    interfaces.emplace_back(m_apertureJointName, kStopSequence,
                            &m_stopSequenceCommand);
  }
  if (aperture != m_info.joints.end() &&
      hasCommand(*aperture, kConventionalSequence)) {
    interfaces.emplace_back(m_apertureJointName, kConventionalSequence,
                            &m_conventionalSequenceCommand);
  }
  return command_stop_gate_.export_interfaces(std::move(interfaces));
}

hardware_interface::CallbackReturn
OnRobotRgSystem::on_configure(const rclcpp_lifecycle::State &) {
  resetReadCache();
  if (m_session) {
    m_session->deactivate();
  }
  try {
    onrobot::ParallelGripperSessionConfig config;
    config.model = m_model;
    config.connection = m_connection;
    config.rg_motion_limits = m_limits;
    config.conventional_period = m_pollPeriod;
    config.realtime_period = m_realtimePeriod;
    config.realtime_command_timeout = m_realtimeCommandTimeout;
    config.maximum_consecutive_failures = m_maximumConsecutiveFailures;
    m_session = std::make_unique<onrobot::ParallelGripperSession>(config);
    const auto initial_image = m_session->snapshot();
    m_cachedState = static_cast<const onrobot::ParallelGripperState &>(
        initial_image);
    if (!applySnapshot(m_cachedState)) {
      throw std::runtime_error("initial RG state is invalid");
    }
    m_positionCommand = m_positionState;
    // Configuration is observational. Seed the command cache from the
    // measured state so controller startup cannot resend that measurement as
    // an unsolicited conventional move. This also avoids rejecting a valid
    // endpoint sample that rounds one encoder increment outside the nominal
    // task-aperture limit.
    m_lastSentPositionM = m_positionCommand;
    m_lastSentEffortN = m_defaultForceN;
    m_commandSent = true;
    m_lastFaultRecoverySequence = 0;
    m_recoveryPending = false;
    m_conventionalRecoveryGate = false;
    m_recoveryReconnectsAtRequest = 0;
    m_forceConventionalCommand = false;
    setNan(m_realtimeModeCommand);
    setNan(m_realtimeSequenceCommand);
    setNan(m_faultRecoverySequenceCommand);
    setNan(m_stopSequenceCommand);
    setNan(m_conventionalSequenceCommand);
  } catch (const std::exception &i_error) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_rg_system"),
                 "RG configure failed: %s", i_error.what());
    m_session.reset();
    resetReadCache();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
OnRobotRgSystem::on_activate(const rclcpp_lifecycle::State &) {
  if (!m_session) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    m_session->activate();
    const auto activation_image = m_session->snapshot();
    m_cachedState = static_cast<const onrobot::ParallelGripperState &>(
        activation_image);
    if (!applySnapshot(m_cachedState)) {
      throw std::runtime_error("activation RG state is invalid");
    }
    m_positionCommand = m_positionState;
    m_lastSentPositionM = m_positionCommand;
    m_lastSentEffortN = m_defaultForceN;
    m_commandSent = true;
    m_pendingStopSessionSequence = 0;
    m_stopPending = false;
    m_recoveryPending = false;
    m_conventionalRecoveryGate = false;
    m_recoveryReconnectsAtRequest = 0;
    m_forceConventionalCommand = false;
    setNan(m_realtimeModeCommand);
    setNan(m_realtimeSequenceCommand);
    setNan(m_faultRecoverySequenceCommand);
    setNan(m_stopSequenceCommand);
    setNan(m_conventionalSequenceCommand);
  } catch (const std::exception &i_error) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_rg_system"),
                 "RG activate failed: %s", i_error.what());
    resetReadCache();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
OnRobotRgSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  if (m_session) {
    m_session->stop();
    m_session->deactivate();
  }
  resetReadCache();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
OnRobotRgSystem::read(const rclcpp::Time &, const rclcpp::Duration &) {
  if (!m_session) {
    return hardware_interface::return_type::ERROR;
  }
  const double previousFingerAngle = m_fingerAngleState;
  (void)m_session->trySnapshot(m_cachedState, m_cachedIdentity);
  const bool applied = applySnapshot(m_cachedState);
  if (!applied) {
    if (m_cachedState.session_state == onrobot::SessionState::Faulted) {
      setNan(m_positionState);
      setNan(m_velocityState);
      setNan(m_effortState);
      setNan(m_fingerAngleState);
      setNan(m_fingerVelocityState);
      setNan(m_measuredAngularPositionState);
      setNan(m_measuredAngularVelocityState);
    } else {
      // Retain only the physical visual joint, never the task position used
      // by standard action controllers. The explicit fault path clears both.
      setNan(m_positionState);
      if (std::isfinite(previousFingerAngle)) {
        m_fingerAngleState = previousFingerAngle;
      }
      setNan(m_velocityState);
      setNan(m_effortState);
      setNan(m_fingerVelocityState);
      setNan(m_measuredAngularVelocityState);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
OnRobotRgSystem::write(const rclcpp::Time &, const rclcpp::Duration &) {
  auto stop_lock = command_stop_gate_.try_lock();
  if (!stop_lock.owns_lock()) return hardware_interface::return_type::OK;
  if (!m_session) {
    return hardware_interface::return_type::ERROR;
  }
  const auto tryQueueStop = [this]() {
    uint64_t sessionSequence = 0;
    const auto admission = m_session->tryStop(sessionSequence);
    if (admission == onrobot::CommandAdmission::Accepted) {
      m_pendingStopSessionSequence = sessionSequence;
      m_stopPending = true;
    }
    return admission;
  };
  if (!std::isnan(m_stopSequenceCommand)) {
    uint64_t sequence = 0;
    if (!decodeCommandSequence(m_stopSequenceCommand, sequence)) {
      RCLCPP_ERROR(rclcpp::get_logger("onrobot_rg_system"),
                   "Rejected invalid RG Stop sequence");
      setNan(m_stopSequenceCommand);
      return hardware_interface::return_type::ERROR;
    }
    (void)sequence;
    // The interface is a one-shot event. Retain it until the nonblocking
    // handoff accepts Stop; contention must not turn a requested Stop into a
    // dropped event.
    const auto admission = tryQueueStop();
    if (admission == onrobot::CommandAdmission::Busy) {
      return hardware_interface::return_type::OK;
    }
    if (admission != onrobot::CommandAdmission::Accepted) {
      RCLCPP_ERROR(rclcpp::get_logger("onrobot_rg_system"),
                   "Unable to queue RG Stop because the session is inactive");
      setNan(m_stopSequenceCommand);
      return hardware_interface::return_type::ERROR;
    }
    m_stopPending = true;
    m_conventionalRecoveryGate = true;
    m_retiredPositionCommand = m_positionCommand;
    // No event published before Stop is fresh intent after Stop.
    if (m_conventionalSequenceCommand > 0.0) {
      m_conventionalSequenceCommand = -m_conventionalSequenceCommand;
    }
    m_forceConventionalCommand = false;
    setNan(m_stopSequenceCommand);
    return hardware_interface::return_type::OK;
  }
  if (!std::isnan(m_faultRecoverySequenceCommand)) {
    uint64_t sequence = 0;
    if (!decodeCommandSequence(m_faultRecoverySequenceCommand, sequence)) {
      RCLCPP_ERROR(rclcpp::get_logger("onrobot_rg_system"),
                   "Rejected invalid RG recovery sequence");
      setNan(m_faultRecoverySequenceCommand);
      return hardware_interface::return_type::ERROR;
    }
    if (sequence != m_lastFaultRecoverySequence) {
      uint64_t sessionSequence = 0;
      const auto admission = m_session->tryRequestRecovery(sessionSequence);
      if (admission == onrobot::CommandAdmission::Busy) {
        return hardware_interface::return_type::OK;
      }
      if (admission == onrobot::CommandAdmission::Accepted) {
        m_retiredPositionCommand = m_positionCommand;
        if (m_conventionalSequenceCommand > 0.0) {
          m_conventionalSequenceCommand = -m_conventionalSequenceCommand;
        }
        m_forceConventionalCommand = false;
        m_recoveryReconnectsAtRequest = m_cachedState.reconnects;
        m_recoveryPending = true;
      }
      m_lastFaultRecoverySequence = sequence;
    }
    setNan(m_faultRecoverySequenceCommand);
    setNan(m_realtimeSequenceCommand);
    return hardware_interface::return_type::OK;
  }
  if (m_stopPending) {
    // ParallelGripperSession stores the newest requested command image.  Do
    // not let the controller's hold or preempting target replace Stop before
    // the worker has applied it and reported the session idle.  Explicit
    // recovery is intentionally above this fence: a failed Stop cannot be
    // acknowledged until the worker reconnects, and recovery is the only
    // command allowed to perform that reconnect.
    const auto &stopImage = m_cachedState;
    if (stopImage.applied_command_sequence < m_pendingStopSessionSequence ||
        stopImage.active_mode != onrobot::ParallelControlMode::Idle) {
      return hardware_interface::return_type::OK;
    }
    m_stopPending = false;
  }
  if (m_recoveryPending && m_faultedState < 0.5 &&
      m_reconnectsState >
          static_cast<double>(m_recoveryReconnectsAtRequest) &&
      m_taskPositionValidState > 0.5 && std::isfinite(m_positionState)) {
    m_recoveryPending = false;
    m_conventionalRecoveryGate = true;
    m_lastSentPositionM = m_positionState;
    m_lastSentEffortN = m_defaultForceN;
    m_commandSent = true;
  }
  if (m_recoveryPending) {
    // Values observed while reconnecting are still pre-recovery intent. Keep
    // the latest conventional value retired and consume any realtime event;
    // a caller must submit a new command after recovery becomes Active/Idle.
    m_retiredPositionCommand = m_positionCommand;
    if (m_conventionalSequenceCommand > 0.0) {
      m_conventionalSequenceCommand = -m_conventionalSequenceCommand;
    }
    m_forceConventionalCommand = false;
    setNan(m_realtimeSequenceCommand);
    return hardware_interface::return_type::OK;
  }
  if (std::isfinite(m_realtimeSequenceCommand)) {
    const auto sequence = static_cast<uint64_t>(m_realtimeSequenceCommand);
    if (sequence != m_lastRealtimeSequence) {
      try {
        if (!std::isfinite(m_realtimeModeCommand) ||
            m_realtimeModeCommand < 0.0) {
          // Stop is a command-mode transition only. A latched transport or
          // device fault must remain latched until the caller uses the
          // explicit recovery interface; otherwise releasing a realtime
          // joystick can silently reconnect and resume the session.
          if (tryQueueStop() != onrobot::CommandAdmission::Accepted) {
            return hardware_interface::return_type::OK;
          }
        } else {
          const auto mode = static_cast<int>(m_realtimeModeCommand);
          onrobot::CommandAdmission admission =
              onrobot::CommandAdmission::InvalidArgument;
          uint64_t admittedSequence = 0;
          switch (mode) {
          case 0:
            admission = m_session->tryCommand(
                onrobot::RgRealtimeCommand{onrobot::RgRealtimePositionCommand{
                    m_realtimeTaskPositionCommand * 1000.0,
                    m_realtimeForceCommand}},
                admittedSequence);
            break;
          case 1:
            admission = m_session->tryCommand(
                onrobot::RgRealtimeCommand{onrobot::RgRealtimeVelocityCommand{
                    m_realtimeMechanismAngularVelocityCommand}},
                admittedSequence);
            break;
          default:
            if (tryQueueStop() != onrobot::CommandAdmission::Accepted) {
              return hardware_interface::return_type::OK;
            }
            RCLCPP_ERROR(
                rclcpp::get_logger("onrobot_rg_system"),
                "Rejected unsupported RG realtime mode %d; Stop queued", mode);
            m_lastRealtimeSequence = sequence;
            setNan(m_realtimeSequenceCommand);
            return hardware_interface::return_type::OK;
          }
          if (admission == onrobot::CommandAdmission::Busy) {
            return hardware_interface::return_type::OK;
          }
          if (admission != onrobot::CommandAdmission::Accepted) {
            RCLCPP_ERROR(
                rclcpp::get_logger("onrobot_rg_system"),
                "Rejected RG realtime command (admission=%d); Stop queued",
                static_cast<int>(admission));
            if (tryQueueStop() != onrobot::CommandAdmission::Accepted) {
              return hardware_interface::return_type::OK;
            }
          }
        }
      } catch (const std::exception &i_error) {
        RCLCPP_ERROR(rclcpp::get_logger("onrobot_rg_system"),
                     "Rejected RG realtime command; Stop queued: %s",
                     i_error.what());
        if (tryQueueStop() != onrobot::CommandAdmission::Accepted) {
          return hardware_interface::return_type::OK;
        }
      }
      m_lastRealtimeSequence = sequence;
      setNan(m_realtimeSequenceCommand);
    }
    return hardware_interface::return_type::OK;
  }
  if (m_conventionalSequenceCommand < 0.0) {
    return hardware_interface::return_type::OK;
  }
  if (std::isfinite(m_conventionalSequenceCommand)) {
    uint64_t sequence = 0;
    if (!decodeCommandSequence(m_conventionalSequenceCommand, sequence)) {
      RCLCPP_ERROR(rclcpp::get_logger("onrobot_rg_system"),
                   "Rejected invalid RG conventional command sequence");
      setNan(m_conventionalSequenceCommand);
      return hardware_interface::return_type::ERROR;
    }
    (void)sequence;
    m_forceConventionalCommand = true;
  }
  const auto rejectConventional = [this]() {
    if (std::isfinite(m_conventionalSequenceCommand)) {
      m_conventionalSequenceCommand = -m_conventionalSequenceCommand;
    }
    m_positionCommand = m_positionState;
    m_forceConventionalCommand = false;
  };
  const bool recoveryGate = m_conventionalRecoveryGate;
  if (recoveryGate && !m_forceConventionalCommand &&
      (m_positionCommand == m_retiredPositionCommand ||
       m_positionCommand == m_positionState)) {
    m_forceConventionalCommand = false;
    return hardware_interface::return_type::OK;
  }
  if (!std::isfinite(m_positionCommand)) {
    rejectConventional();
    return hardware_interface::return_type::OK;
  }
  const double minimumM = m_limits.minimum_width_mm / 1000.0;
  const double maximumM = m_limits.maximum_width_mm / 1000.0;
  if (m_positionCommand < minimumM || m_positionCommand > maximumM) {
    if (!m_hasRejectedCommand || m_lastRejectedPositionM != m_positionCommand) {
      RCLCPP_ERROR(rclcpp::get_logger("onrobot_rg_system"),
                   "Rejected RG aperture %.1f mm; range is %.1f..%.1f mm",
                   m_positionCommand * 1000.0, m_limits.minimum_width_mm,
                   m_limits.maximum_width_mm);
      m_hasRejectedCommand = true;
      m_lastRejectedPositionM = m_positionCommand;
    }
    rejectConventional();
    return hardware_interface::return_type::OK;
  }
  m_hasRejectedCommand = false;
  if (recoveryGate) {
    m_conventionalRecoveryGate = false;
    m_retiredPositionCommand = 0.0;
  }
  if (m_hasEffortCommand &&
      (!std::isfinite(m_effortCommand) || m_effortCommand < 0.0)) {
    rejectConventional();
    return hardware_interface::return_type::OK;
  }
  const double force = m_hasEffortCommand && std::isfinite(m_effortCommand) &&
                               m_effortCommand > 0.0
                           ? m_effortCommand
                           : m_defaultForceN;
  if (!std::isfinite(force) || force <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_rg_system"),
                 "Rejected non-finite or non-positive RG force");
    rejectConventional();
    return hardware_interface::return_type::OK;
  }
  if (!m_forceConventionalCommand && m_commandSent &&
      m_lastSentPositionM == m_positionCommand &&
      m_lastSentEffortN == force) {
    return hardware_interface::return_type::OK;
  }
  uint64_t admittedSequence = 0;
  const auto admission = m_session->tryCommand(
      onrobot::ParallelGripCommand{
          m_positionCommand * 1000.0, force, 50.0},
      admittedSequence);
  if (admission == onrobot::CommandAdmission::Busy) {
    return hardware_interface::return_type::OK;
  }
  if (admission == onrobot::CommandAdmission::Accepted) {
    m_lastSentPositionM = m_positionCommand;
    m_lastSentEffortN = force;
    m_commandSent = true;
    m_forceConventionalCommand = false;
    setNan(m_conventionalSequenceCommand);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_rg_system"),
                 "Rejected RG command (admission=%d)",
                 static_cast<int>(admission));
    rejectConventional();
    return hardware_interface::return_type::OK;
  }
  return hardware_interface::return_type::OK;
}

bool OnRobotRgSystem::applySnapshot(
    const onrobot::ParallelGripperState &i_image) {
  resetDiagnostics(m_diagnosticStates);
  exportIdentity(m_cachedIdentity, i_image, m_diagnosticStates);
  const auto putValue = [this](DiagnosticInterface i_interface,
                               const onrobot::DiagnosticValue &i_value) {
    const auto index = diagnosticIndex(i_interface);
    if (i_value.valid && std::isfinite(i_value.value)) {
      m_diagnosticStates[index] = i_value.value;
    }
  };
  const auto putFlag = [this](DiagnosticInterface i_interface, bool i_value) {
    m_diagnosticStates[diagnosticIndex(i_interface)] = i_value ? 1.0 : 0.0;
  };
  if (i_image.diagnostics_valid) {
    const auto age = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() -
                         i_image.diagnostics_received_at)
                         .count();
    const auto &diagnostics = i_image.rg_diagnostics;
    m_diagnosticStates[diagnosticIndex(DiagnosticInterface::StatusValid)] =
        diagnostics.status_valid ? 1.0 : 0.0;
    m_diagnosticStates[diagnosticIndex(DiagnosticInterface::SampleSequence)] =
        static_cast<double>(i_image.diagnostics_sample_sequence);
    if (std::isfinite(age) && age >= 0.0) {
      m_diagnosticStates[diagnosticIndex(DiagnosticInterface::Age)] = age;
    }
    if (diagnostics.status_valid) {
      m_diagnosticStates[diagnosticIndex(DiagnosticInterface::RawStatus)] =
          diagnostics.raw_status;
      putFlag(DiagnosticInterface::Busy, diagnostics.busy);
      putFlag(DiagnosticInterface::GripDetected, diagnostics.grip_detected);
      m_diagnosticStates[diagnosticIndex(DiagnosticInterface::RgErrorCode)] =
          diagnostics.error_code;
    }
    putValue(DiagnosticInterface::CommandForce, diagnostics.command_force_n);
    putFlag(DiagnosticInterface::CommandForceValid,
            diagnostics.command_force_n.valid);
    putFlag(DiagnosticInterface::MeasuredForceValid, false);
    putValue(DiagnosticInterface::MotorVoltage, diagnostics.motor_voltage_v);
    putValue(DiagnosticInterface::MotorCurrent, diagnostics.motor_current_a);
    putValue(DiagnosticInterface::Temperature, diagnostics.temperature_c);
    putValue(DiagnosticInterface::RgFingertipOffset,
             diagnostics.fingertip_offset_mm);
    putValue(DiagnosticInterface::RgDepthAcceleration,
             diagnostics.depth_acceleration_mm_s2);
    putValue(DiagnosticInterface::RgDepthSpeed, diagnostics.depth_speed_mm_s);
    putValue(DiagnosticInterface::RgActualDepth, diagnostics.actual_depth_mm);
    putValue(DiagnosticInterface::RgActualRelativeDepth,
             diagnostics.actual_relative_depth_mm);
    putValue(DiagnosticInterface::RgMechanismAngle,
             diagnostics.mechanism_angle_rad);
    putValue(DiagnosticInterface::RgLegacyAngularVelocity,
             diagnostics.legacy_angular_speed_rad_s);
    putValue(DiagnosticInterface::RgActualWidth, diagnostics.actual_width_mm);
    putValue(DiagnosticInterface::RgVoltage5V, diagnostics.voltage_5v_v);
    putValue(DiagnosticInterface::RgSafety24V, diagnostics.safety_24v_v);
    putValue(DiagnosticInterface::RgPlug24V, diagnostics.plug_24v_v);
    putValue(DiagnosticInterface::RgWidthWithFingertip,
             diagnostics.width_with_fingertip_mm);
    putValue(DiagnosticInterface::RealtimeLinearVelocity,
             diagnostics.realtime_linear_velocity_mm_s);
    putValue(DiagnosticInterface::RealtimeAngularVelocity,
             diagnostics.realtime_angular_velocity_rad_s);
    putValue(DiagnosticInterface::RealtimeForce,
             onrobot::DiagnosticValue{});
    if (diagnostics.command_force_n.valid) {
      m_diagnosticStates[diagnosticIndex(DiagnosticInterface::ForceProvenance)] =
          static_cast<double>(static_cast<uint8_t>(
              diagnostics.command_force_n.provenance));
    }
    putFlag(DiagnosticInterface::StatisticsValid, false);
  }
  m_activeModeState = static_cast<double>(i_image.active_mode);
  m_connectionState = connectionState(i_image);
  m_faultedState =
      i_image.session_state == onrobot::SessionState::Faulted ? 1.0 : 0.0;
  m_faultCodeState = static_cast<double>(i_image.last_error_code);
  m_firmwareQualificationState =
      static_cast<double>(i_image.firmware_qualification);
  m_sampleSequenceState = static_cast<double>(i_image.sample_sequence);
  m_sampleAgeState =
      i_image.sample_sequence > 0
          ? std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          i_image.received_at)
                .count()
          : 0.0;
  m_requestedCommandSequenceState =
      static_cast<double>(i_image.requested_command_sequence);
  m_appliedCommandSequenceState =
      static_cast<double>(i_image.applied_command_sequence);
  m_busyState = i_image.busy ? 1.0 : 0.0;
  m_gripDetectedState = i_image.grip_detected ? 1.0 : 0.0;
  m_safetyStatusValidState = i_image.safety_status_valid ? 1.0 : 0.0;
  m_safety1PushedState = i_image.safety_1_pushed ? 1.0 : 0.0;
  m_safety1TriggeredState = i_image.safety_1_triggered ? 1.0 : 0.0;
  m_safety2PushedState = i_image.safety_2_pushed ? 1.0 : 0.0;
  m_safety2TriggeredState = i_image.safety_2_triggered ? 1.0 : 0.0;
  m_safetyDcErrorState = i_image.safety_dc_error ? 1.0 : 0.0;
  m_taskPositionValidState =
      i_image.task_aperture_valid && std::isfinite(i_image.task_aperture_mm) ? 1.0 : 0.0;
  m_forceValidState = i_image.force_valid && std::isfinite(i_image.force_n) ? 1.0 : 0.0;
  m_mechanismAngularPositionValidState =
      i_image.mechanism_angular_position_valid &&
      std::isfinite(i_image.mechanism_angular_position_rad) ? 1.0 : 0.0;
  m_mechanismAngularVelocityValidState =
      i_image.mechanism_angular_velocity_valid &&
      std::isfinite(i_image.mechanism_angular_velocity_rad_s) ? 1.0 : 0.0;
  m_successfulCyclesState = static_cast<double>(i_image.successful_cycles);
  m_failedCyclesState = static_cast<double>(i_image.failed_cycles);
  m_missedDeadlinesState = static_cast<double>(i_image.missed_deadlines);
  m_watchdogStopsState = static_cast<double>(i_image.watchdog_stops);
  m_reconnectsState = static_cast<double>(i_image.reconnects);
  m_lastCycleDurationState = i_image.last_cycle_duration_s;
  if (i_image.task_aperture_limits_valid) {
    m_minimumTaskApertureState = i_image.minimum_task_aperture_mm / 1000.0;
    m_maximumTaskApertureState = i_image.maximum_task_aperture_mm / 1000.0;
  }
  if (i_image.session_state == onrobot::SessionState::Faulted ||
      m_taskPositionValidState == 0.0) {
    m_taskPositionValidState = 0.0;
    m_forceValidState = 0.0;
    m_mechanismAngularPositionValidState = 0.0;
    m_mechanismAngularVelocityValidState = 0.0;
    setNan(m_measuredAngularPositionState);
    resetDerivativeHistory();
    return false;
  }
  const double taskPositionM = i_image.task_aperture_mm / 1000.0;
  if (!std::isfinite(taskPositionM)) {
    resetDerivativeHistory();
    return false;
  }
  // CAD joint zero and firmware angle zero differ. The CAD linkage has a
  // lateral component of 2.5 mm over 55 mm (RG2), or 4.2 mm over 80 mm
  // (RG6), at joint zero. Rotate by that phase to express measured mechanism
  // angle in the CAD joint frame. Raw mechanism feedback remains unchanged.
  // This is independent of the operator's configured fingertip offset.
  const double cadZeroPhase = m_cadKinematics->cadZeroPhase();
  const double mappedFingerAngle =
      m_cadKinematics->widthToFingerAngle(taskPositionM);
  const double fingerAngle =
      m_visualGapFromAperture
          ? mappedFingerAngle
      : i_image.mechanism_angular_position_valid &&
              std::isfinite(i_image.mechanism_angular_position_rad)
          ? std::clamp(i_image.mechanism_angular_position_rad + cadZeroPhase,
                       0.0, m_visualFingerJointUpperRad)
          : mappedFingerAngle;
  if (!std::isfinite(fingerAngle)) {
    resetDerivativeHistory();
    return false;
  }
  if (i_image.sample_sequence != m_lastSampleSequence) {
    const bool validTaskVelocity = i_image.task_velocity_valid &&
                                   std::isfinite(i_image.task_velocity_mm_s);
    if (validTaskVelocity) {
      m_velocityState = i_image.task_velocity_mm_s / 1000.0;
    } else if (m_lastSampleSequence == 0) {
      setNan(m_velocityState);
    }
    if (m_lastSampleSequence == 0) {
      setNan(m_fingerVelocityState);
    } else {
      const double seconds =
          std::chrono::duration<double>(i_image.received_at - m_lastReceivedAt)
              .count();
      if (seconds > 0.0) {
        if (!validTaskVelocity) {
          m_velocityState = (taskPositionM - m_lastTaskPositionM) / seconds;
        }
        m_fingerVelocityState = (fingerAngle - m_lastFingerAngleRad) / seconds;
      }
    }
  }
  m_positionState = taskPositionM;
  m_effortState = m_forceValidState > 0.0
                      ? i_image.force_n
                      : std::numeric_limits<double>::quiet_NaN();
  m_measuredAngularPositionState = m_mechanismAngularPositionValidState > 0.0
      ? i_image.mechanism_angular_position_rad
      : std::numeric_limits<double>::quiet_NaN();
  m_measuredAngularVelocityState = m_mechanismAngularVelocityValidState > 0.0
      ? i_image.mechanism_angular_velocity_rad_s
      : std::numeric_limits<double>::quiet_NaN();
  m_fingerAngleState = fingerAngle;
  if (i_image.sample_sequence != m_lastSampleSequence) {
    m_lastTaskPositionM = taskPositionM;
    m_lastFingerAngleRad = fingerAngle;
    m_lastReceivedAt = i_image.received_at;
    m_lastSampleSequence = i_image.sample_sequence;
  }
  return true;
}

void OnRobotRgSystem::resetReadCache() {
  m_cachedState = {};
  m_cachedIdentity = {};
  // A lifecycle reset is a real loss of the previously exported measurement.
  // Clear pose and validity-bearing values here so the transient-invalid
  // handoff in read() cannot retain a pose across deactivate/configure.
  setNan(m_positionState);
  setNan(m_velocityState);
  setNan(m_effortState);
  setNan(m_fingerAngleState);
  setNan(m_fingerVelocityState);
  setNan(m_measuredAngularPositionState);
  setNan(m_measuredAngularVelocityState);
  m_taskPositionValidState = 0.0;
  m_forceValidState = 0.0;
  m_mechanismAngularPositionValidState = 0.0;
  m_mechanismAngularVelocityValidState = 0.0;
  resetDiagnostics(m_diagnosticStates);
  resetDerivativeHistory();
  m_recoveryPending = false;
  m_conventionalRecoveryGate = false;
  m_recoveryReconnectsAtRequest = 0;
}

void OnRobotRgSystem::resetDerivativeHistory() {
  m_lastTaskPositionM = 0.0;
  m_lastFingerAngleRad = 0.0;
  m_lastReceivedAt = {};
  m_lastSampleSequence = 0;
}

bool OnRobotRgSystem::hasCommand(
    const hardware_interface::ComponentInfo &i_joint,
    const std::string &i_name) const {
  return std::any_of(
      i_joint.command_interfaces.begin(), i_joint.command_interfaces.end(),
      [&i_name](const auto &interface) { return interface.name == i_name; });
}

bool OnRobotRgSystem::hasState(const hardware_interface::ComponentInfo &i_joint,
                               const std::string &i_name) const {
  return std::any_of(
      i_joint.state_interfaces.begin(), i_joint.state_interfaces.end(),
      [&i_name](const auto &interface) { return interface.name == i_name; });
}

bool OnRobotRgSystem::parseParameters() {
  const auto model = parameter(m_info, "model", "rg2");
  if (model == "rg2") {
    m_model = onrobot::Model::RG2;
    m_visualFingerJointUpperRad =
        number(m_info, "visual_finger_joint_upper_rad", 1.3136012652574625);
  } else if (model == "rg6") {
    m_model = onrobot::Model::RG6;
    m_visualFingerJointUpperRad =
        number(m_info, "visual_finger_joint_upper_rad", 1.2978487644385668);
  } else {
    throw std::invalid_argument("model must be 'rg2' or 'rg6'");
  }
  m_limits.minimum_width_mm = number(m_info, "safe_width_min_mm", 0.0);
  m_limits.maximum_width_mm = number(m_info, "safe_width_max_mm", 0.0);
  m_defaultForceN = number(m_info, "default_force_n", 10.0);
  const int pollPeriodMs = integer(m_info, "poll_period_ms", 20);
  const int realtimeRateHz = integer(m_info, "realtime_update_rate_hz", 500);
  const int realtimeTimeoutMs =
      integer(m_info, "realtime_command_timeout_ms", 100);
  const int maximumFailures =
      integer(m_info, "realtime_maximum_consecutive_failures", 3);
  if (m_limits.minimum_width_mm < 0.0 ||
      m_limits.maximum_width_mm <= m_limits.minimum_width_mm ||
      m_defaultForceN <= 0.0 || pollPeriodMs <= 0 || realtimeRateHz <= 0 ||
      realtimeRateHz > 500 || realtimeTimeoutMs <= 0 || maximumFailures <= 0) {
    throw std::invalid_argument("RG limits, force, or poll period is invalid");
  }
  m_pollPeriod = std::chrono::milliseconds(pollPeriodMs);
  m_realtimePeriod = std::chrono::microseconds(1000000 / realtimeRateHz);
  m_realtimeCommandTimeout = std::chrono::milliseconds(realtimeTimeoutMs);
  m_maximumConsecutiveFailures = static_cast<uint32_t>(maximumFailures);
  m_cadKinematics =
      std::make_unique<RgCadKinematics>(m_model == onrobot::Model::RG6);
  if (m_visualFingerJointUpperRad <= 0.0 || m_visualFingerJointUpperRad >=
      1.5707963267948966 + m_cadKinematics->cadZeroPhase()) {
    throw std::invalid_argument("RG visual joint range must remain monotonic");
  }
  const auto visualGap = parameter(m_info, "visual_gap_from_aperture", "false");
  if (visualGap != "true" && visualGap != "false") {
    throw std::invalid_argument("visual_gap_from_aperture must be true or false");
  }
  m_visualGapFromAperture = visualGap == "true";
  const int slave = integer(m_info, "slave_id", 65);
  const int connectTimeoutMs = integer(m_info, "connect_timeout_ms", 1000);
  const int readTimeoutMs = integer(m_info, "read_timeout_ms", 1000);
  const int writeTimeoutMs = integer(m_info, "write_timeout_ms", 1000);
  if (slave < 65 || slave > 67 || connectTimeoutMs <= 0 ||
      readTimeoutMs <= 0 || writeTimeoutMs <= 0) {
    throw std::invalid_argument("RG slave id or Modbus timeout is invalid");
  }
  m_transportTimeout = {
      connectTimeoutMs, readTimeoutMs, writeTimeoutMs};
  const auto selectedSlave = static_cast<onrobot::ModbusSlaveId>(slave);
  const auto transport = parameter(m_info, "transport", "tcp");
  if (transport == "tcp") {
    const auto host = parameter(m_info, "host", "127.0.0.1");
    const int port = integer(m_info, "port", 502);
    if (host.empty() || port < 1 || port > 65535) {
      throw std::invalid_argument("RG TCP host or port is invalid");
    }
    m_connection = onrobot::tcp(
        host, static_cast<uint16_t>(port), selectedSlave, m_transportTimeout);
  } else if (transport == "rtu") {
    const auto device = parameter(m_info, "serial_device", "/dev/ttyUSB0");
    const int baud = integer(m_info, "baud_rate", 1000000);
    const auto parity = parameter(m_info, "rtu_parity", "even");
    if (device.empty() || (baud != 115200 && baud != 1000000) ||
        parity != "even") {
      throw std::invalid_argument(
          "RG serial device, baud rate, or RTU parity is invalid");
    }
    m_connection = onrobot::rtu(
        device,
        baud == 115200 ? onrobot::ModbusBaudRate::B115200
                       : onrobot::ModbusBaudRate::B1000000,
        selectedSlave, m_transportTimeout, onrobot::ModbusRtuParity::Even);
  } else {
    throw std::invalid_argument("transport must be 'tcp' or 'rtu'");
  }
  return true;
}

} // namespace onrobot_gripper_hardware

PLUGINLIB_EXPORT_CLASS(onrobot_gripper_hardware::OnRobotRgSystem,
                       hardware_interface::SystemInterface)

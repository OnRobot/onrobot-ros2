#include "onrobot_gripper_hardware/onrobot_gripper_system.hpp"
#include "onrobot_gripper_hardware/identity_state.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
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
constexpr char kRealtimeTaskVelocity[] = "realtime_task_velocity";
constexpr char kRealtimeForce[] = "realtime_force";
constexpr char kRealtimeSequence[] = "realtime_command_sequence";
constexpr char kFaultRecoverySequence[] = "fault_recovery_command_sequence";
constexpr char kStopSequence[] = "stop_command_sequence";
constexpr char kConventionalSequence[] = "conventional_command_sequence";
constexpr char kConventionalSpeedPercent[] = "conventional_speed_percent";

std::string parameterOr(const hardware_interface::HardwareInfo &i_info,
                        const std::string &i_name,
                        const std::string &i_default) {
  const auto iterator = i_info.hardware_parameters.find(i_name);
  return iterator == i_info.hardware_parameters.end() ? i_default
                                                      : iterator->second;
}

double parseDouble(const hardware_interface::HardwareInfo &i_info,
                   const std::string &i_name, double i_default) {
  const auto value = parameterOr(i_info, i_name, std::to_string(i_default));
  size_t consumed = 0;
  const double parsed = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(parsed)) {
    throw std::invalid_argument(i_name + " must be a finite number");
  }
  return parsed;
}

int parseInt(const hardware_interface::HardwareInfo &i_info,
             const std::string &i_name, int i_default) {
  const auto value = parameterOr(i_info, i_name, std::to_string(i_default));
  size_t consumed = 0;
  const long parsed = std::stol(value, &consumed);
  if (consumed != value.size() || parsed < std::numeric_limits<int>::min() ||
      parsed > std::numeric_limits<int>::max()) {
    throw std::invalid_argument(i_name + " must be an integer");
  }
  return static_cast<int>(parsed);
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

OnRobotGripperSystem::~OnRobotGripperSystem() {
  if (m_session) {
    m_session->stop();
    m_session->deactivate();
  }
}

hardware_interface::CallbackReturn OnRobotGripperSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &i_params) {
  if (hardware_interface::SystemInterface::on_init(i_params) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  m_info = i_params.hardware_info;
  if (m_info.joints.empty() || m_info.joints.size() > 2) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                 "2FG requires grip_stroke and optional finger_stroke");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto task = std::find_if(
      m_info.joints.begin(), m_info.joints.end(), [](const auto &i_joint) {
        return i_joint.name == "grip_stroke" ||
               (i_joint.command_interfaces.size() > 0 &&
                i_joint.name != "finger_stroke");
      });
  const auto finger = std::find_if(
      m_info.joints.begin(), m_info.joints.end(),
      [](const auto &i_joint) { return i_joint.name == "finger_stroke"; });
  if (m_info.joints.size() == 2 && finger == m_info.joints.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                 "The optional second 2FG joint must be finger_stroke");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (task == m_info.joints.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                 "2FG task joint is missing");
    return hardware_interface::CallbackReturn::ERROR;
  }
  m_jointName = task->name;
  if (!has_command_interface(hardware_interface::HW_IF_POSITION) ||
      !has_state_interface(hardware_interface::HW_IF_POSITION)) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                 "The 2FG task joint requires position command and state");
    return hardware_interface::CallbackReturn::ERROR;
  }
  m_hasEffortCommand =
      std::any_of(task->command_interfaces.begin(),
                  task->command_interfaces.end(), [](const auto &i_interface) {
                    return i_interface.name == hardware_interface::HW_IF_EFFORT;
                  });
  m_hasFingerJoint = finger != m_info.joints.end();
  if (m_hasFingerJoint) {
    m_fingerJointName = finger->name;
    if (!finger->command_interfaces.empty() ||
        !has_state_interface(*finger, hardware_interface::HW_IF_POSITION)) {
      RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                   "finger_stroke must be state-only with position state");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  setNan(m_positionState);
  setNan(m_velocityState);
  setNan(m_effortState);
  setNan(m_fingerPositionState);
  setNan(m_fingerVelocityState);
  setNan(m_mechanismMeasuredPositionState);
  setNan(m_mechanismMeasuredVelocityState);
  setNan(m_realtimeModeCommand);
  setNan(m_realtimeTaskPositionCommand);
  setNan(m_realtimeTaskVelocityCommand);
  setNan(m_realtimeForceCommand);
  setNan(m_realtimeSequenceCommand);
  setNan(m_faultRecoverySequenceCommand);
  setNan(m_stopSequenceCommand);
  resetDiagnostics(m_diagnosticStates);
  try {
    parse_parameters();
    m_kinematics = std::make_unique<GripperKinematics>(
        m_rawLinearMinimumMm, m_rawLinearMaximumMm, m_fingerJointUpperM);
  } catch (const std::exception &i_error) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                 "Invalid 2FG parameters: %s", i_error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OnRobotGripperSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(m_jointName, hardware_interface::HW_IF_POSITION,
                          &m_positionState);
  if (has_state_interface(hardware_interface::HW_IF_VELOCITY)) {
    interfaces.emplace_back(m_jointName, hardware_interface::HW_IF_VELOCITY,
                            &m_velocityState);
  }
  if (has_state_interface(hardware_interface::HW_IF_EFFORT)) {
    interfaces.emplace_back(m_jointName, hardware_interface::HW_IF_EFFORT,
                            &m_effortState);
  }
  if (has_state_interface("task_position_valid")) {
    interfaces.emplace_back(m_jointName, "task_position_valid",
                            &m_taskPositionValidState);
    interfaces.emplace_back(m_jointName, "force_valid", &m_forceValidState);
    interfaces.emplace_back(m_jointName, "busy", &m_busyState);
    interfaces.emplace_back(m_jointName, "grip_detected", &m_gripDetectedState);
    interfaces.emplace_back(m_jointName, "active_mode",
                            &m_sessionActiveModeState);
    interfaces.emplace_back(m_jointName, "connection_state",
                            &m_connectionState);
    interfaces.emplace_back(m_jointName, "faulted", &m_sessionFaultedState);
    interfaces.emplace_back(m_jointName, "fault_code", &m_faultCodeState);
    if (has_state_interface("firmware_qualification")) {
      interfaces.emplace_back(m_jointName, "firmware_qualification",
                              &m_firmwareQualificationState);
    }
    if (has_state_interface("realtime_force_control_available")) {
      interfaces.emplace_back(m_jointName, "realtime_force_control_available",
                              &m_realtimeForceControlAvailable);
    }
    interfaces.emplace_back(m_jointName, "sample_sequence",
                            &m_sampleSequenceState);
    interfaces.emplace_back(m_jointName, "sample_age", &m_sampleAgeState);
    interfaces.emplace_back(m_jointName, "requested_command_sequence",
                            &m_requestedCommandSequenceState);
    interfaces.emplace_back(m_jointName, "applied_command_sequence",
                            &m_appliedCommandSequenceState);
    interfaces.emplace_back(m_jointName, "successful_cycles",
                            &m_successfulCyclesState);
    interfaces.emplace_back(m_jointName, "failed_cycles", &m_failedCyclesState);
    interfaces.emplace_back(m_jointName, "missed_deadlines",
                            &m_missedDeadlinesState);
    interfaces.emplace_back(m_jointName, "watchdog_stops",
                            &m_watchdogStopsState);
    interfaces.emplace_back(m_jointName, "reconnects", &m_reconnectsState);
    interfaces.emplace_back(m_jointName, "last_cycle_duration",
                            &m_lastCycleDurationState);
    interfaces.emplace_back(m_jointName, "minimum_task_aperture",
                            &m_minimumTaskApertureState);
    interfaces.emplace_back(m_jointName, "maximum_task_aperture",
                            &m_maximumTaskApertureState);
    for (std::size_t index = 0;
         index < onrobot_gripper_msgs::kDiagnosticInterfaceCount; ++index) {
      if (has_state_interface(onrobot_gripper_msgs::kDiagnosticInterfaceNames[index])) {
        interfaces.emplace_back(m_jointName,
                                onrobot_gripper_msgs::kDiagnosticInterfaceNames[index],
                                &m_diagnosticStates[index]);
      }
    }
  }
  if (m_hasFingerJoint) {
    interfaces.emplace_back(m_fingerJointName,
                            hardware_interface::HW_IF_POSITION,
                            &m_fingerPositionState);
    const auto finger = std::find_if(
        m_info.joints.begin(), m_info.joints.end(),
        [this](const auto &joint) { return joint.name == m_fingerJointName; });
    if (finger != m_info.joints.end() &&
        has_state_interface(*finger, hardware_interface::HW_IF_VELOCITY)) {
      interfaces.emplace_back(m_fingerJointName,
                              hardware_interface::HW_IF_VELOCITY,
                              &m_fingerVelocityState);
    }
    if (finger != m_info.joints.end() &&
        has_state_interface(*finger, "measured_position")) {
      interfaces.emplace_back(m_fingerJointName, "measured_position",
                              &m_mechanismMeasuredPositionState);
      interfaces.emplace_back(m_fingerJointName, "measured_velocity",
                              &m_mechanismMeasuredVelocityState);
      interfaces.emplace_back(m_fingerJointName, "position_valid",
                              &m_mechanismPositionValidState);
      interfaces.emplace_back(m_fingerJointName, "velocity_valid",
                              &m_mechanismVelocityValidState);
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
OnRobotGripperSystem::on_export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back(m_jointName, hardware_interface::HW_IF_POSITION,
                          &m_positionCommand);
  if (m_hasEffortCommand) {
    interfaces.emplace_back(m_jointName, hardware_interface::HW_IF_EFFORT,
                            &m_effortCommand);
  }
  if (has_command_interface(kRealtimeMode)) {
    interfaces.emplace_back(m_jointName, kRealtimeMode, &m_realtimeModeCommand);
    interfaces.emplace_back(m_jointName, kRealtimeTaskPosition,
                            &m_realtimeTaskPositionCommand);
    interfaces.emplace_back(m_jointName, kRealtimeTaskVelocity,
                            &m_realtimeTaskVelocityCommand);
    interfaces.emplace_back(m_jointName, kRealtimeForce,
                            &m_realtimeForceCommand);
    interfaces.emplace_back(m_jointName, kRealtimeSequence,
                            &m_realtimeSequenceCommand);
  }
  if (has_command_interface(kFaultRecoverySequence)) {
    interfaces.emplace_back(m_jointName, kFaultRecoverySequence,
                            &m_faultRecoverySequenceCommand);
  }
  if (has_command_interface(kStopSequence)) {
    interfaces.emplace_back(m_jointName, kStopSequence, &m_stopSequenceCommand);
  }
  if (has_command_interface(kConventionalSequence)) {
    interfaces.emplace_back(m_jointName, kConventionalSequence,
                            &m_conventionalSequenceCommand);
  }
  if (has_command_interface(kConventionalSpeedPercent)) {
    interfaces.emplace_back(m_jointName, kConventionalSpeedPercent,
                            &m_conventionalSpeedPercentCommand);
  }
  return command_stop_gate_.export_interfaces(std::move(interfaces));
}

hardware_interface::CallbackReturn
OnRobotGripperSystem::on_configure(const rclcpp_lifecycle::State &) {
  reset_read_cache();
  if (m_session) {
    m_session->deactivate();
  }
  try {
    onrobot::ParallelGripperSessionConfig config;
    config.model = m_model;
    config.connection = m_connection;
    config.two_finger_supply_power_w = m_supplyPowerW;
    config.conventional_period = m_pollPeriod;
    config.realtime_period = m_realtimePeriod;
    config.realtime_command_timeout = m_realtimeTimeout;
    config.maximum_consecutive_failures = m_maximumConsecutiveFailures;
    m_session = std::make_unique<onrobot::ParallelGripperSession>(config);
    const auto initial_image = m_session->snapshot();
    const bool realtimeFirmwareCompatible =
        initial_image.firmware_qualification ==
        onrobot::FirmwareQualification::Qualified;
    RCLCPP_INFO(rclcpp::get_logger("onrobot_gripper_system"),
                "Connected %s firmware %s; realtime command map: %s",
                m_model == onrobot::Model::TwoFG7 ? "2FG7" : "2FG14",
                initial_image.firmware_revision.empty()
                    ? "unknown"
                    : initial_image.firmware_revision.c_str(),
                realtimeFirmwareCompatible ? "compatible" : "not enabled");
    if (!realtimeFirmwareCompatible) {
      RCLCPP_WARN(
          rclcpp::get_logger("onrobot_gripper_system"),
          "Realtime commands are disabled because the complete firmware "
          "identity does not match the current command map; conventional "
          "control remains available");
    }
    m_cachedState = static_cast<const onrobot::ParallelGripperState &>(
        initial_image);
    if (!apply_snapshot(m_cachedState)) {
      throw std::runtime_error("initial 2FG state is invalid");
    }
    m_positionCommand = m_positionState;
    // Activation is observational. Mark the measured position as the current
    // command so controller startup cannot resend it as an unintended motion
    // request. The first materially different position or effort still queues
    // a normal device command.
    m_lastSentPositionM = m_positionCommand;
    m_lastSentEffortN = m_defaultForceN;
    m_commandSent = true;
    m_lastRealtimeSequence = 0;
    m_lastFaultRecoverySequence = 0;
    m_recoveryPending = false;
    m_conventionalRecoveryGate = false;
    m_recoveryReconnectsAtRequest = 0;
    m_forceConventionalCommand = false;
    setNan(m_stopSequenceCommand);
    setNan(m_conventionalSequenceCommand);
    m_conventionalSpeedPercentCommand = m_speedPercent;
  } catch (const std::exception &i_error) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                 "Failed to configure 2FG: %s", i_error.what());
    m_session.reset();
    reset_read_cache();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
OnRobotGripperSystem::on_activate(const rclcpp_lifecycle::State &) {
  if (!m_session) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    m_session->activate();
    const auto activation_image = m_session->snapshot();
    m_cachedState = static_cast<const onrobot::ParallelGripperState &>(
        activation_image);
    if (!apply_snapshot(m_cachedState)) {
      throw std::runtime_error("activation state is invalid");
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
    m_conventionalSpeedPercentCommand = m_speedPercent;
  } catch (const std::exception &i_error) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                 "Failed to activate 2FG: %s", i_error.what());
    reset_read_cache();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
OnRobotGripperSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  if (m_session) {
    m_session->stop();
    m_session->deactivate();
  }
  reset_read_cache();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
OnRobotGripperSystem::read(const rclcpp::Time &, const rclcpp::Duration &) {
  if (!m_session) {
    return hardware_interface::return_type::ERROR;
  }
  const double previousFingerPosition = m_fingerPositionState;
  (void)m_session->trySnapshot(m_cachedState, m_cachedIdentity);
  (void)apply_snapshot(m_cachedState);
  if (m_cachedState.session_state != onrobot::SessionState::Faulted) {
    // Only the physical visual joint may retain its pose. grip_stroke is also
    // read by standard action controllers, which do not consume our validity
    // flag; retaining that task value could falsely complete an action.
    // Explicit faults intentionally clear the pose above.
    if (!std::isfinite(m_fingerPositionState) && std::isfinite(previousFingerPosition)) {
      m_fingerPositionState = previousFingerPosition;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
OnRobotGripperSystem::write(const rclcpp::Time &, const rclcpp::Duration &) {
  auto stop_lock = command_stop_gate_.try_lock();
  if (!stop_lock.owns_lock()) return hardware_interface::return_type::OK;
  if (!m_session || !std::isfinite(m_positionCommand)) {
    if (!m_session) {
      return hardware_interface::return_type::ERROR;
    }
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
      RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                   "Rejected invalid 2FG Stop sequence");
      setNan(m_stopSequenceCommand);
      return hardware_interface::return_type::ERROR;
    }
    (void)sequence;
    // The interface is a one-shot event. Do not consume it until the fixed-size
    // session handoff accepts it: a contended callback must retry Stop rather
    // than silently losing the only terminal command.
    const auto admission = tryQueueStop();
    if (admission != onrobot::CommandAdmission::Accepted) {
      // A contended handoff retains the command-interface event for the next
      // write cycle. An inactive session cannot consume it either.
      if (admission == onrobot::CommandAdmission::Busy) {
        return hardware_interface::return_type::OK;
      }
      m_pendingStopSessionSequence = 0;
      RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                   "Unable to queue 2FG Stop because the session is inactive");
      setNan(m_stopSequenceCommand);
      return hardware_interface::return_type::ERROR;
    }
    m_stopPending = true;
    m_conventionalRecoveryGate = true;
    m_retiredPositionCommand = m_positionCommand;
    // Stop retires all intent exported before it. A controller must export a
    // replacement event only after this Stop marker has been consumed.
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
      RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                   "Rejected invalid 2FG recovery sequence");
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
  if (m_recoveryPending && m_sessionFaultedState < 0.5 &&
      m_reconnectsState >
          static_cast<double>(m_recoveryReconnectsAtRequest) &&
      m_taskPositionValidState > 0.5 && std::isfinite(m_positionState)) {
    m_recoveryPending = false;
    m_conventionalRecoveryGate = true;
    // Recovery establishes a measured idle hold. The controller's command
    // interface remains untouched, but its retained pre-fault value is not
    // considered new intent until it changes to a valid target.
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
                onrobot::RealtimeCommand{onrobot::RealtimePositionCommand{
                    m_realtimeTaskPositionCommand * 1000.0,
                    m_realtimeTaskVelocityCommand * 1000.0}},
                admittedSequence);
            break;
          case 1:
            admission = m_session->tryCommand(
                onrobot::RealtimeCommand{onrobot::RealtimeVelocityCommand{
                    m_realtimeTaskVelocityCommand * 1000.0}},
                admittedSequence);
            break;
          case 2:
            admission = m_session->tryCommand(
                onrobot::RealtimeCommand{onrobot::RealtimeForcePositionCommand{
                    m_realtimeTaskPositionCommand * 1000.0,
                    m_realtimeForceCommand,
                    m_realtimeTaskVelocityCommand * 1000.0}},
                admittedSequence);
            break;
          case 3:
            admission = m_session->tryCommand(
                onrobot::RealtimeCommand{onrobot::RealtimeForceVelocityCommand{
                    m_realtimeForceCommand,
                    m_realtimeTaskVelocityCommand * 1000.0}},
                admittedSequence);
            break;
          default:
            if (tryQueueStop() != onrobot::CommandAdmission::Accepted) {
              return hardware_interface::return_type::OK;
            }
            m_lastRealtimeSequence = sequence;
            setNan(m_realtimeSequenceCommand);
            RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                         "Rejected unknown realtime mode %d; Stop queued",
                         mode);
            return hardware_interface::return_type::OK;
          }
          if (admission == onrobot::CommandAdmission::Busy) {
            return hardware_interface::return_type::OK;
          }
          if (admission != onrobot::CommandAdmission::Accepted) {
            RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                         "Rejected realtime command (admission=%d); Stop queued",
                         static_cast<int>(admission));
            if (tryQueueStop() != onrobot::CommandAdmission::Accepted) {
              return hardware_interface::return_type::OK;
            }
          }
        }
        m_lastRealtimeSequence = sequence;
        setNan(m_realtimeSequenceCommand);
      } catch (const std::exception &i_error) {
        RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                     "Rejected realtime command; Stop queued: %s",
                     i_error.what());
        if (tryQueueStop() != onrobot::CommandAdmission::Accepted) {
          return hardware_interface::return_type::OK;
        }
        m_lastRealtimeSequence = sequence;
        setNan(m_realtimeSequenceCommand);
        return hardware_interface::return_type::OK;
      }
    }
    return hardware_interface::return_type::OK;
  }
  if (m_conventionalSequenceCommand < 0.0) {
    // A rejected action remains retired until a fresh positive goal marker.
    return hardware_interface::return_type::OK;
  }
  if (std::isfinite(m_conventionalSequenceCommand)) {
    uint64_t sequence = 0;
    if (!decodeCommandSequence(m_conventionalSequenceCommand, sequence)) {
      RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                   "Rejected invalid 2FG conventional command sequence");
      setNan(m_conventionalSequenceCommand);
      return hardware_interface::return_type::ERROR;
    }
    (void)sequence;
    // This is a one-shot intent marker, not a persistent numeric identity.
    // The next valid conventional target must be sent even when its numeric
    // values equal the preceding goal.
    m_forceConventionalCommand = true;
  }
  if (!std::isfinite(m_positionCommand)) {
    m_forceConventionalCommand = false;
    return hardware_interface::return_type::ERROR;
  }
  const bool recoveryGate = m_conventionalRecoveryGate;
  if (recoveryGate && !m_forceConventionalCommand &&
      (m_positionCommand == m_retiredPositionCommand ||
       m_positionCommand == m_positionState)) {
    m_forceConventionalCommand = false;
    return hardware_interface::return_type::OK;
  }
  const auto &image = m_cachedState;
  const double apertureMm = m_positionCommand * 1000.0;
  // A rejected one-shot goal must not leave its position in the ros2_control
  // command interface.  The controller writes position before the explicit
  // conventional sequence marker; without retiring it here, the next
  // periodic write could reinterpret the rejected target with the default
  // force and move the device anyway.
  const auto retireConventionalIntent = [this]() {
    if (std::isfinite(m_conventionalSequenceCommand)) {
      m_conventionalSequenceCommand = -m_conventionalSequenceCommand;
    }
    if (std::isfinite(m_positionState)) {
      m_positionCommand = m_positionState;
      m_lastSentPositionM = m_positionState;
      m_lastSentEffortN = m_defaultForceN;
      m_commandSent = true;
    }
    m_forceConventionalCommand = false;
  };
  if (m_hasEffortCommand &&
      (!std::isfinite(m_effortCommand) || m_effortCommand < 0.0)) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                 "Rejected negative 2FG force %.3f N", m_effortCommand);
    retireConventionalIntent();
    return hardware_interface::return_type::OK;
  }
  const double force = m_hasEffortCommand && std::isfinite(m_effortCommand) &&
                               m_effortCommand > 0.0
                           ? m_effortCommand
                           : m_defaultForceN;
  if (!std::isfinite(force) || force < 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                 "Rejected non-finite or negative 2FG force");
    retireConventionalIntent();
    return hardware_interface::return_type::OK;
  }
  const double speedPercent = has_command_interface(kConventionalSpeedPercent)
                                  ? m_conventionalSpeedPercentCommand
                                  : m_speedPercent;
  if (!std::isfinite(speedPercent) || speedPercent < 1.0 ||
      speedPercent > 100.0 || std::floor(speedPercent) != speedPercent) {
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                 "Rejected non-integer 2FG conventional speed %.3f%%", speedPercent);
    retireConventionalIntent();
    return hardware_interface::return_type::OK;
  }
  // Check the activation seed before validating active finger-profile limits.
  // A device may report a stationary sample one encoder count outside its
  // configured range; retaining that observation must not be logged as a new
  // rejected motion command.
  if (!m_forceConventionalCommand && m_commandSent &&
      m_lastSentPositionM == m_positionCommand &&
      m_lastSentEffortN == force) {
    return hardware_interface::return_type::OK;
  }
  if (!image.task_aperture_limits_valid ||
      apertureMm < image.minimum_task_aperture_mm ||
      apertureMm > image.maximum_task_aperture_mm) {
    if (!m_hasRejectedCommand || m_lastRejectedPositionM != m_positionCommand) {
      RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                   "Rejected 2FG aperture %.1f mm; active finger-profile range "
                   "is %.1f..%.1f mm",
                   apertureMm, image.minimum_task_aperture_mm,
                   image.maximum_task_aperture_mm);
      m_hasRejectedCommand = true;
      m_lastRejectedPositionM = m_positionCommand;
    }
    retireConventionalIntent();
    return hardware_interface::return_type::OK;
  }
  m_hasRejectedCommand = false;
  if (recoveryGate) {
    m_conventionalRecoveryGate = false;
    m_retiredPositionCommand = 0.0;
  }
  uint64_t admittedSequence = 0;
  const auto admission = m_session->tryCommand(
      onrobot::ParallelGripCommand{apertureMm, force, speedPercent},
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
    RCLCPP_ERROR(rclcpp::get_logger("onrobot_gripper_system"),
                 "Rejected 2FG command (admission=%d)",
                 static_cast<int>(admission));
    retireConventionalIntent();
    return hardware_interface::return_type::OK;
  }
  return hardware_interface::return_type::OK;
}

bool OnRobotGripperSystem::apply_snapshot(
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
  const auto putProvenance = [this](const onrobot::DiagnosticValue &i_value) {
    const auto index = diagnosticIndex(DiagnosticInterface::ForceProvenance);
    if (i_value.valid) {
      m_diagnosticStates[index] =
          static_cast<double>(static_cast<uint8_t>(i_value.provenance));
    }
  };
  if (i_image.diagnostics_valid) {
    const auto age = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() -
                         i_image.diagnostics_received_at)
                         .count();
    m_diagnosticStates[diagnosticIndex(DiagnosticInterface::StatusValid)] =
        i_image.two_finger_diagnostics.status_valid ? 1.0 : 0.0;
    m_diagnosticStates[diagnosticIndex(DiagnosticInterface::SampleSequence)] =
        static_cast<double>(i_image.diagnostics_sample_sequence);
    if (std::isfinite(age) && age >= 0.0) {
      m_diagnosticStates[diagnosticIndex(DiagnosticInterface::Age)] = age;
    }
    const auto &diagnostics = i_image.two_finger_diagnostics;
    if (diagnostics.status_valid) {
      m_diagnosticStates[diagnosticIndex(DiagnosticInterface::RawStatus)] =
          diagnostics.raw_status;
      putFlag(DiagnosticInterface::Busy, diagnostics.busy);
      putFlag(DiagnosticInterface::GripDetected, diagnostics.grip_detected);
      putFlag(DiagnosticInterface::NotCalibrated, diagnostics.not_calibrated);
      putFlag(DiagnosticInterface::LinearSensorError,
              diagnostics.linear_sensor_error);
    }
    putValue(DiagnosticInterface::ExternalWidth,
             diagnostics.external_width_mm);
    putValue(DiagnosticInterface::InternalWidth,
             diagnostics.internal_width_mm);
    putValue(DiagnosticInterface::MinimumExternalWidth,
             diagnostics.minimum_external_width_mm);
    putValue(DiagnosticInterface::MaximumExternalWidth,
             diagnostics.maximum_external_width_mm);
    putValue(DiagnosticInterface::MinimumInternalWidth,
             diagnostics.minimum_internal_width_mm);
    putValue(DiagnosticInterface::MaximumInternalWidth,
             diagnostics.maximum_internal_width_mm);
    putValue(DiagnosticInterface::MeasuredForce, diagnostics.force_n);
    if (diagnostics.force_n.valid) {
      putFlag(DiagnosticInterface::MeasuredForceValid, true);
      putProvenance(diagnostics.force_n);
    } else {
      putFlag(DiagnosticInterface::MeasuredForceValid, false);
    }
    putFlag(DiagnosticInterface::CommandForceValid, false);
    putValue(DiagnosticInterface::MotorVoltage, diagnostics.voltage_24v_v);
    putValue(DiagnosticInterface::MotorCurrent, diagnostics.current_24v_a);
    putValue(DiagnosticInterface::Temperature, diagnostics.temperature_c);
    putValue(DiagnosticInterface::AdditionalResults,
             diagnostics.additional_results);
    putValue(DiagnosticInterface::LinearMechanismPosition,
             diagnostics.linear_mechanism_position_mm);
    putValue(DiagnosticInterface::RawMotorWidth, diagnostics.raw_motor_width_mm);
    putValue(DiagnosticInterface::Voltage5V, diagnostics.voltage_5v_v);
    putValue(DiagnosticInterface::LinearCount, diagnostics.linear_count);
    putValue(DiagnosticInterface::MotorSpeed, diagnostics.motor_speed_rpm);
    putValue(DiagnosticInterface::AngleSensorCount,
             diagnostics.angle_sensor_count);
    putValue(DiagnosticInterface::LinearErrorCount,
             diagnostics.linear_error_count);
    putValue(DiagnosticInterface::RealtimeLinearVelocity,
             diagnostics.realtime_velocity_mm_s);
    putValue(DiagnosticInterface::RealtimeForce, diagnostics.realtime_force_n);
    putValue(DiagnosticInterface::RealtimeExternalWidth,
             diagnostics.realtime_external_width_mm);
    if (diagnostics.power_valid) {
      m_diagnosticStates[diagnosticIndex(DiagnosticInterface::SupplyPower)] =
          diagnostics.supply_power_w;
      m_diagnosticStates[diagnosticIndex(DiagnosticInterface::MaximumForce)] =
          diagnostics.maximum_conventional_force_n;
      m_diagnosticStates[
          diagnosticIndex(DiagnosticInterface::MaximumRealtimeForce)] =
          diagnostics.maximum_realtime_force_n;
    }
    if (diagnostics.statistics.valid) {
      putFlag(DiagnosticInterface::StatisticsValid, true);
      m_diagnosticStates[
          diagnosticIndex(DiagnosticInterface::ConventionalGripOnTime)] =
          diagnostics.statistics.conventional_grip_on_time_s;
      m_diagnosticStates[diagnosticIndex(DiagnosticInterface::PowerCycles)] =
          diagnostics.statistics.power_cycles;
      m_diagnosticStates[
          diagnosticIndex(DiagnosticInterface::ConventionalGripCycles)] =
          diagnostics.statistics.conventional_grip_cycles;
      m_diagnosticStates[
          diagnosticIndex(DiagnosticInterface::GripDetectedCount)] =
          diagnostics.statistics.grip_detected_count;
      m_diagnosticStates[
          diagnosticIndex(DiagnosticInterface::RealtimeGripOnTime)] =
          diagnostics.statistics.realtime_grip_on_time_s;
    } else {
      putFlag(DiagnosticInterface::StatisticsValid, false);
    }
  }
  m_sessionActiveModeState = static_cast<double>(i_image.active_mode);
  m_connectionState = connectionState(i_image);
  m_sessionFaultedState =
      i_image.session_state == onrobot::SessionState::Faulted ? 1.0 : 0.0;
  m_faultCodeState = static_cast<double>(i_image.last_error_code);
  m_firmwareQualificationState =
      static_cast<double>(i_image.firmware_qualification);
  m_realtimeForceControlAvailable =
      i_image.firmware_qualification == onrobot::FirmwareQualification::Qualified &&
      i_image.session_state != onrobot::SessionState::Faulted &&
      onrobot::supportsRealtimeMode(m_model, onrobot::RealtimeMode::ForcePosition) &&
      onrobot::supportsRealtimeMode(m_model, onrobot::RealtimeMode::ForceVelocity)
          ? 1.0 : 0.0;
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
  m_taskPositionValidState =
      i_image.task_aperture_valid && std::isfinite(i_image.task_aperture_mm) ? 1.0 : 0.0;
  m_forceValidState = i_image.force_valid && std::isfinite(i_image.force_n) ? 1.0 : 0.0;
  m_mechanismPositionValidState =
      i_image.mechanism_linear_position_valid &&
      std::isfinite(i_image.mechanism_linear_position_mm) ? 1.0 : 0.0;
  m_mechanismVelocityValidState =
      i_image.mechanism_linear_velocity_valid &&
      std::isfinite(i_image.mechanism_linear_velocity_mm_s) ? 1.0 : 0.0;
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
  if (i_image.session_state == onrobot::SessionState::Faulted) {
    m_taskPositionValidState = 0.0;
    m_forceValidState = 0.0;
    m_mechanismPositionValidState = 0.0;
    m_mechanismVelocityValidState = 0.0;
    // A faulted sample is never allowed to leave a healthy stale joint state
    // behind. Reset the local derivative history as well, so the first valid
    // sample after explicit recovery cannot be compared with pre-fault data.
    setNan(m_positionState);
    setNan(m_velocityState);
    setNan(m_effortState);
    setNan(m_fingerPositionState);
    setNan(m_fingerVelocityState);
    setNan(m_mechanismMeasuredPositionState);
    setNan(m_mechanismMeasuredVelocityState);
    resetDiagnostics(m_diagnosticStates);
    m_lastTaskPositionM = 0.0;
    m_lastReceivedAt = {};
    m_lastSampleSequence = 0;
    return false;
  }

  if (i_image.task_aperture_valid && std::isfinite(i_image.task_aperture_mm)) {
    m_positionState = i_image.task_aperture_mm / 1000.0;
  } else {
    setNan(m_positionState);
  }
  if (i_image.task_velocity_valid &&
      std::isfinite(i_image.task_velocity_mm_s)) {
    m_velocityState = i_image.task_velocity_mm_s / 1000.0;
  } else {
    setNan(m_velocityState);
  }

  if (i_image.mechanism_linear_position_valid &&
      std::isfinite(i_image.mechanism_linear_position_mm)) {
    const double fingerPosition =
        m_kinematics->rawPositionToJoint(i_image.mechanism_linear_position_mm);
    if (std::isfinite(fingerPosition)) {
      m_fingerPositionState = fingerPosition;
      m_mechanismMeasuredPositionState =
          i_image.mechanism_linear_position_mm / 1000.0;
    } else {
      setNan(m_fingerPositionState);
      setNan(m_mechanismMeasuredPositionState);
      m_mechanismPositionValidState = 0.0;
    }
  } else {
    setNan(m_fingerPositionState);
    setNan(m_mechanismMeasuredPositionState);
  }
  if (i_image.mechanism_linear_velocity_valid &&
      std::isfinite(i_image.mechanism_linear_velocity_mm_s)) {
    m_fingerVelocityState = m_kinematics->rawVelocityToJoint(
        i_image.mechanism_linear_velocity_mm_s);
    m_mechanismMeasuredVelocityState =
        i_image.mechanism_linear_velocity_mm_s / 1000.0;
  } else {
    setNan(m_fingerVelocityState);
    setNan(m_mechanismMeasuredVelocityState);
  }
  if (i_image.force_valid && std::isfinite(i_image.force_n)) {
    m_effortState = i_image.force_n;
  } else {
    setNan(m_effortState);
  }
  if (i_image.sample_sequence != m_lastSampleSequence) {
    m_lastTaskPositionM = m_positionState;
    m_lastReceivedAt = i_image.received_at;
    m_lastSampleSequence = i_image.sample_sequence;
  }
  return i_image.task_aperture_valid && std::isfinite(m_positionState);
}

void OnRobotGripperSystem::reset_read_cache() {
  m_cachedState = {};
  m_cachedIdentity = {};
  // A lifecycle reset is a real loss of the previously exported measurement.
  // Clear pose and validity-bearing values here so the transient-invalid
  // handoff in read() cannot retain a pose across deactivate/configure.
  setNan(m_positionState);
  setNan(m_velocityState);
  setNan(m_effortState);
  setNan(m_fingerPositionState);
  setNan(m_fingerVelocityState);
  setNan(m_mechanismMeasuredPositionState);
  setNan(m_mechanismMeasuredVelocityState);
  m_taskPositionValidState = 0.0;
  m_forceValidState = 0.0;
  m_mechanismPositionValidState = 0.0;
  m_mechanismVelocityValidState = 0.0;
  resetDiagnostics(m_diagnosticStates);
  m_lastTaskPositionM = 0.0;
  m_lastReceivedAt = {};
  m_lastSampleSequence = 0;
  m_recoveryPending = false;
  m_conventionalRecoveryGate = false;
  m_recoveryReconnectsAtRequest = 0;
}

bool OnRobotGripperSystem::has_command_interface(
    const std::string &i_name) const {
  const auto task = std::find_if(
      m_info.joints.begin(), m_info.joints.end(),
      [this](const auto &joint) { return joint.name == m_jointName; });
  return task != m_info.joints.end() &&
         std::any_of(task->command_interfaces.begin(),
                     task->command_interfaces.end(),
                     [&i_name](const auto &interface) {
                       return interface.name == i_name;
                     });
}

bool OnRobotGripperSystem::has_state_interface(
    const std::string &i_name) const {
  const auto task = std::find_if(
      m_info.joints.begin(), m_info.joints.end(),
      [this](const auto &joint) { return joint.name == m_jointName; });
  return task != m_info.joints.end() && has_state_interface(*task, i_name);
}

bool OnRobotGripperSystem::has_state_interface(
    const hardware_interface::ComponentInfo &i_joint,
    const std::string &i_name) const {
  return std::any_of(
      i_joint.state_interfaces.begin(), i_joint.state_interfaces.end(),
      [&i_name](const auto &interface) { return interface.name == i_name; });
}

bool OnRobotGripperSystem::parse_parameters() {
  const auto model = parameterOr(m_info, "model", "2fg7");
  if (model == "2fg7") {
    m_model = onrobot::Model::TwoFG7;
  } else if (model == "2fg14") {
    m_model = onrobot::Model::TwoFG14;
  } else {
    throw std::invalid_argument("model must be '2fg7' or '2fg14'");
  }
  m_fingerJointUpperM =
      parseDouble(m_info, "finger_joint_upper_m",
                  m_model == onrobot::Model::TwoFG7 ? 0.019 : 0.025);
  m_rawLinearMinimumMm = parseDouble(m_info, "raw_linear_min_mm", 1.0);
  m_rawLinearMaximumMm =
      parseDouble(m_info, "raw_linear_max_mm",
                  m_model == onrobot::Model::TwoFG7 ? 39.0 : 51.0);
  const double minimumForceN = onrobot::minimumConventionalForceN(m_model);
  m_defaultForceN = parseDouble(m_info, "default_force_n", minimumForceN);
  m_speedPercent = parseDouble(m_info, "speed_percent", 50.0);
  const int supplyPowerW = parseInt(m_info, "supply_power_w", 48);
  if (supplyPowerW == 0) {
    m_supplyPowerW.reset();
  } else if (supplyPowerW >= 14 && supplyPowerW <= 48) {
    m_supplyPowerW = static_cast<uint16_t>(supplyPowerW);
  } else {
    throw std::invalid_argument(
        "supply_power_w must be zero or between 14 and 48 W");
  }
  const int pollPeriodMs = parseInt(m_info, "poll_period_ms", 20);
  const double realtimeRateHz =
      parseDouble(m_info, "realtime_update_rate_hz", 500.0);
  const int realtimeTimeoutMs =
      parseInt(m_info, "realtime_command_timeout_ms", 100);
  const int failureLimit =
      parseInt(m_info, "realtime_maximum_consecutive_failures", 3);
  if (m_defaultForceN < minimumForceN || m_speedPercent < 1.0 ||
      m_speedPercent > 100.0 || pollPeriodMs <= 0 || realtimeRateHz <= 0.0 ||
      realtimeRateHz > 500.0 || realtimeTimeoutMs <= 0 || failureLimit <= 0) {
    throw std::invalid_argument(
        "default_force_n must be at least the model minimum (" +
        std::to_string(minimumForceN) + " N); speed and poll period are also "
        "validated");
  }
  m_pollPeriod = std::chrono::milliseconds(pollPeriodMs);
  m_realtimePeriod = std::chrono::microseconds(
      static_cast<int64_t>(std::llround(1000000.0 / realtimeRateHz)));
  m_realtimeTimeout = std::chrono::milliseconds(realtimeTimeoutMs);
  m_maximumConsecutiveFailures = static_cast<uint32_t>(failureLimit);

  const int slaveId = parseInt(m_info, "slave_id", 65);
  if (slaveId < 65 || slaveId > 67) {
    throw std::invalid_argument("slave_id must be between 65 and 67");
  }
  const auto selectedSlave = static_cast<onrobot::ModbusSlaveId>(slaveId);
  const int connectTimeoutMs =
      parseInt(m_info, "connect_timeout_ms", 1000);
  const int readTimeoutMs = parseInt(m_info, "read_timeout_ms", 1000);
  const int writeTimeoutMs = parseInt(m_info, "write_timeout_ms", 1000);
  if (connectTimeoutMs <= 0 || readTimeoutMs <= 0 || writeTimeoutMs <= 0) {
    throw std::invalid_argument("Modbus timeouts must be positive");
  }
  m_transportTimeout = {
      connectTimeoutMs, readTimeoutMs, writeTimeoutMs};
  const auto transport = parameterOr(m_info, "transport", "tcp");
  if (transport == "tcp") {
    const auto host = parameterOr(m_info, "host", "127.0.0.1");
    const int port = parseInt(m_info, "port", 502);
    if (host.empty() || port < 1 || port > 65535) {
      throw std::invalid_argument("TCP host and port are invalid");
    }
    m_connection = onrobot::tcp(
        host, static_cast<uint16_t>(port), selectedSlave, m_transportTimeout);
  } else if (transport == "rtu") {
    const auto device = parameterOr(m_info, "serial_device", "/dev/ttyUSB0");
    const int baud = parseInt(m_info, "baud_rate", 1000000);
    const auto parity = parameterOr(m_info, "rtu_parity", "even");
    if (device.empty() || (baud != 115200 && baud != 1000000) ||
        parity != "even") {
      throw std::invalid_argument(
          "serial device, baud rate, or RTU parity is invalid");
    }
    m_connection =
        onrobot::rtu(device,
                     baud == 115200 ? onrobot::ModbusBaudRate::B115200
                                    : onrobot::ModbusBaudRate::B1000000,
                     selectedSlave, m_transportTimeout,
                     onrobot::ModbusRtuParity::Even);
  } else {
    throw std::invalid_argument("transport must be 'tcp' or 'rtu'");
  }
  return true;
}

} // namespace onrobot_gripper_hardware

PLUGINLIB_EXPORT_CLASS(onrobot_gripper_hardware::OnRobotGripperSystem,
                       hardware_interface::SystemInterface)

#include "onrobot_gripper_controllers/gripper_state_broadcaster.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <onrobot_gripper_msgs/firmware_identity.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace onrobot_gripper_controllers {
namespace {

using GripperState = onrobot_gripper_msgs::msg::GripperState;

uint8_t mappingSource(const std::string &i_source) {
  if (i_source == "device_measured") {
    return GripperState::MAPPING_SOURCE_DEVICE_MEASURED;
  }
  if (i_source == "device_compensated") {
    return GripperState::MAPPING_SOURCE_DEVICE_COMPENSATED;
  }
  if (i_source == "declarative") {
    return GripperState::MAPPING_SOURCE_DECLARATIVE;
  }
  return GripperState::MAPPING_SOURCE_UNKNOWN;
}

uint8_t faultSource(double i_code) {
  if (!std::isfinite(i_code) || i_code < 0.0) {
    return GripperState::FAULT_SOURCE_NONE;
  }
  switch (static_cast<int>(i_code)) {
  case 1: // NotConnected
  case 2: // Transport
  case 3: // Timeout
    return GripperState::FAULT_SOURCE_TRANSPORT;
  case 4: // Protocol
    return GripperState::FAULT_SOURCE_PROTOCOL;
  case 5: // DeviceFault
    return GripperState::FAULT_SOURCE_DEVICE;
  case 6: // InvalidArgument
  case 7: // Unsupported
  case 8: // Busy
  case 9: // Cancelled
    return GripperState::FAULT_SOURCE_HOST;
  default:
    return GripperState::FAULT_SOURCE_NONE;
  }
}

const char *faultMessage(uint16_t i_code) {
  switch (i_code) {
  case 0:
    return "";
  case 1:
    return "not connected";
  case 2:
    return "transport error";
  case 3:
    return "transport timeout";
  case 4:
    return "protocol error";
  case 5:
    return "device fault";
  case 6:
    return "invalid command";
  case 7:
    return "unsupported operation";
  case 8:
    return "device busy";
  case 9:
    return "operation cancelled";
  default:
    return "unknown fault";
  }
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string &i_key,
                                        const std::string &i_value) {
  diagnostic_msgs::msg::KeyValue result;
  result.key = i_key;
  result.value = i_value;
  return result;
}

double finiteOrZero(double i_value) {
  return std::isfinite(i_value) ? i_value : 0.0;
}

int64_t steadyNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

double advancedAge(double i_age_s, int64_t i_captured_ns, int64_t i_now_ns) {
  if (!std::isfinite(i_age_s) || i_age_s < 0.0) {
    return i_age_s;
  }
  const auto elapsed_ns = std::max<int64_t>(0, i_now_ns - i_captured_ns);
  return i_age_s + static_cast<double>(elapsed_ns) * 1e-9;
}

} // namespace

controller_interface::InterfaceConfiguration
GripperStateBroadcaster::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
GripperStateBroadcaster::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::
              INDIVIDUAL_BEST_EFFORT,
          requestedInterfaces()};
}

controller_interface::CallbackReturn GripperStateBroadcaster::on_init() {
  auto_declare<std::string>("model", "");
  auto_declare<std::string>("task_joint", "grip_stroke");
  auto_declare<std::string>("mechanism_joint", "finger_stroke");
  auto_declare<std::string>("mechanism_dimension", "linear");
  auto_declare<std::string>("firmware", "");
  auto_declare<std::string>("device_profile_revision", "");
  auto_declare<std::string>("finger_profile_name", "");
  auto_declare<std::string>("finger_profile_revision", "");
  auto_declare<std::string>("mapping_source", "unknown");
  auto_declare<double>("publish_rate", 100.0);
  auto_declare<double>("diagnostic_rate", 1.0);
  // Keep the standalone topic at /diagnostics while allowing independent
  // namespaced grippers to publish separate diagnostic streams.
  auto_declare<std::string>("diagnostics_topic", "diagnostics");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GripperStateBroadcaster::on_configure(const rclcpp_lifecycle::State &) {
  const auto node = get_node();
  m_model = node->get_parameter("model").as_string();
  m_taskJoint = node->get_parameter("task_joint").as_string();
  m_mechanismJoint = node->get_parameter("mechanism_joint").as_string();
  m_mechanismDimension = node->get_parameter("mechanism_dimension").as_string();
  m_firmware = node->get_parameter("firmware").as_string();
  for (std::size_t index = 0; index < m_diagnosticInterfaceNames.size(); ++index) {
    m_diagnosticInterfaceNames[index] = m_taskJoint + "/" +
        onrobot_gripper_msgs::kDiagnosticInterfaceNames[index];
  }
  m_deviceProfileRevision =
      node->get_parameter("device_profile_revision").as_string();
  m_fingerProfileName = node->get_parameter("finger_profile_name").as_string();
  m_fingerProfileRevision =
      node->get_parameter("finger_profile_revision").as_string();
  m_mappingSource = node->get_parameter("mapping_source").as_string();
  m_publishRateHz = node->get_parameter("publish_rate").as_double();
  m_diagnosticRateHz = node->get_parameter("diagnostic_rate").as_double();
  m_diagnosticsTopic = node->get_parameter("diagnostics_topic").as_string();
  if (m_model.empty() || m_taskJoint.empty() || m_mechanismJoint.empty() ||
      (m_mechanismDimension != "linear" && m_mechanismDimension != "angular") ||
      !std::isfinite(m_publishRateHz) || m_publishRateHz <= 0.0 ||
      !std::isfinite(m_diagnosticRateHz) || m_diagnosticRateHz <= 0.0 ||
      m_diagnosticsTopic.empty()) {
    RCLCPP_ERROR(node->get_logger(),
                 "Invalid gripper state broadcaster parameters");
    return controller_interface::CallbackReturn::ERROR;
  }

  auto publisher = node->create_publisher<GripperState>(
      "~/state", rclcpp::SensorDataQoS().keep_last(1));
  m_realtimePublisher = std::make_unique<StatePublisher>(publisher);
  auto &message = m_realtimePublisher->msg_;
  message.model = m_model;
  // Configured text is not evidence of the connected device's firmware.
  message.firmware.clear();
  message.firmware.reserve(64);
  message.device_profile_revision = m_deviceProfileRevision;
  message.finger_profile_name = m_fingerProfileName;
  message.finger_profile_revision = m_fingerProfileRevision;
  message.fault_message.reserve(32);
  message.mapping_source = mappingSource(m_mappingSource);
  message.mapping_validity = m_fingerProfileName.empty()
                                 ? GripperState::MAPPING_MISSING
                                 : GripperState::MAPPING_VALID;
  message.firmware_qualification = GripperState::FIRMWARE_UNKNOWN;
  message.mechanism_dimension = m_mechanismDimension == "linear"
                                    ? GripperState::DIMENSION_LINEAR
                                    : GripperState::DIMENSION_ANGULAR;

  m_diagnosticPublisher =
      node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          m_diagnosticsTopic, 10);
  m_diagnosticTimer = node->create_wall_timer(
      std::chrono::duration<double>(1.0 / m_diagnosticRateHz),
      [this]() { publishDiagnostics(); });
  m_lastPublish = rclcpp::Time(0, 0, RCL_ROS_TIME);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GripperStateBroadcaster::on_activate(const rclcpp_lifecycle::State &) {
  m_lastPublish = rclcpp::Time(0, 0, RCL_ROS_TIME);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GripperStateBroadcaster::on_deactivate(const rclcpp_lifecycle::State &) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
GripperStateBroadcaster::update(const rclcpp::Time &i_time,
                                const rclcpp::Duration &) {
  const auto controllerRateHz = get_update_rate();
  const bool publishEveryUpdate =
      controllerRateHz > 0 && m_publishRateHz >= controllerRateHz;
  if (!publishEveryUpdate && m_lastPublish.nanoseconds() != 0 &&
      (i_time - m_lastPublish).seconds() < 1.0 / m_publishRateHz) {
    return controller_interface::return_type::OK;
  }
  if (!m_realtimePublisher || !m_realtimePublisher->trylock()) {
    return controller_interface::return_type::OK;
  }

  auto &state = m_realtimePublisher->msg_;
  state.header.stamp = i_time;
  const double sampleAge = value(m_taskJoint + "/sample_age");
  const double boundedAge =
      std::isfinite(sampleAge) && sampleAge >= 0.0 ? sampleAge : 0.0;
  state.sample_age = rclcpp::Duration::from_seconds(boundedAge);
  state.received_time = i_time - rclcpp::Duration::from_seconds(boundedAge);
  state.sample_sequence = static_cast<uint64_t>(
      std::max(0.0, value(m_taskJoint + "/sample_sequence")));

  state.task_aperture_valid = valid(m_taskJoint + "/task_position_valid");
  const double taskAperture = value(m_taskJoint + "/position");
  const double taskVelocity = value(m_taskJoint + "/velocity");
  state.task_aperture = finiteOrZero(taskAperture);
  state.task_velocity = finiteOrZero(taskVelocity);
  state.task_velocity_valid = std::isfinite(taskVelocity);
  state.force_valid = valid(m_taskJoint + "/force_valid");
  state.force = finiteOrZero(value(m_taskJoint + "/effort"));
  state.busy = valid(m_taskJoint + "/busy");
  state.grip_detected = valid(m_taskJoint + "/grip_detected");
  state.safety_status_valid = valid(m_taskJoint + "/safety_status_valid");
  state.safety_1_pushed = valid(m_taskJoint + "/safety_1_pushed");
  state.safety_1_triggered = valid(m_taskJoint + "/safety_1_triggered");
  state.safety_2_pushed = valid(m_taskJoint + "/safety_2_pushed");
  state.safety_2_triggered = valid(m_taskJoint + "/safety_2_triggered");
  state.safety_dc_error = valid(m_taskJoint + "/safety_dc_error");

  const std::string measuredPosition = m_mechanismDimension == "linear"
                                           ? "measured_position"
                                           : "measured_angular_position";
  const std::string measuredVelocity = m_mechanismDimension == "linear"
                                           ? "measured_velocity"
                                           : "measured_angular_velocity";
  state.mechanism_position_valid = valid(m_mechanismJoint + "/position_valid");
  state.mechanism_velocity_valid = valid(m_mechanismJoint + "/velocity_valid");
  state.mechanism_position =
      finiteOrZero(value(m_mechanismJoint + "/" + measuredPosition));
  state.mechanism_velocity =
      finiteOrZero(value(m_mechanismJoint + "/" + measuredVelocity));

  const double mode = value(m_taskJoint + "/active_mode");
  const bool faulted = valid(m_taskJoint + "/faulted");
  const double connection = value(m_taskJoint + "/connection_state");
  if (std::isfinite(connection) && connection >= 1.0 && connection <= 6.0) {
    state.connection_state = static_cast<uint8_t>(connection);
  } else if (faulted) {
    state.connection_state = GripperState::CONNECTION_FAULTED;
  } else if (state.sample_sequence == 0) {
    state.connection_state = GripperState::CONNECTION_CONNECTING;
  } else if (std::isfinite(mode) && mode > 0.0) {
    state.connection_state = GripperState::CONNECTION_ACTIVE;
  } else {
    state.connection_state = GripperState::CONNECTION_IDLE;
  }
  state.active_mode =
      std::isfinite(mode) && mode >= 0.0 ? static_cast<uint8_t>(mode) : 0;
  const double code = value(m_taskJoint + "/fault_code");
  state.fault_code =
      std::isfinite(code) && code >= 0.0 ? static_cast<uint16_t>(code) : 0;
  const double firmwareQualification =
      value(m_taskJoint + "/firmware_qualification");
  state.firmware_qualification =
      std::isfinite(firmwareQualification) &&
              firmwareQualification >= GripperState::FIRMWARE_UNKNOWN &&
              firmwareQualification <= GripperState::FIRMWARE_UNSUPPORTED
          ? static_cast<uint8_t>(firmwareQualification)
          : GripperState::FIRMWARE_UNKNOWN;
  state.fault_source = faultSource(code);
  state.realtime_force_control_available =
      value(m_taskJoint + "/realtime_force_control_available") > 0.5;
  state.fault_message = faultMessage(state.fault_code);
  state.successful_cycles = static_cast<uint64_t>(
      std::max(0.0, value(m_taskJoint + "/successful_cycles")));
  state.failed_cycles = static_cast<uint64_t>(
      std::max(0.0, value(m_taskJoint + "/failed_cycles")));
  state.missed_deadlines = static_cast<uint64_t>(
      std::max(0.0, value(m_taskJoint + "/missed_deadlines")));
  state.watchdog_stops = static_cast<uint64_t>(
      std::max(0.0, value(m_taskJoint + "/watchdog_stops")));
  state.reconnects =
      static_cast<uint64_t>(std::max(0.0, value(m_taskJoint + "/reconnects")));
  state.last_cycle_duration =
      finiteOrZero(value(m_taskJoint + "/last_cycle_duration"));

  DiagnosticSnapshot diagnostic;
  diagnostic.captured_at_steady_ns = steadyNowNs();
  diagnostic.connection_state = state.connection_state;
  diagnostic.active_mode = state.active_mode;
  diagnostic.mapping_validity = state.mapping_validity;
  diagnostic.firmware_qualification = state.firmware_qualification;
  diagnostic.fault_source = state.fault_source;
  diagnostic.fault_code = state.fault_code;
  diagnostic.sample_sequence = state.sample_sequence;
  diagnostic.sample_age_s = boundedAge;
  diagnostic.successful_cycles = state.successful_cycles;
  diagnostic.failed_cycles = state.failed_cycles;
  diagnostic.missed_deadlines = state.missed_deadlines;
  diagnostic.watchdog_stops = state.watchdog_stops;
  diagnostic.reconnects = state.reconnects;
  diagnostic.last_cycle_duration_s = state.last_cycle_duration;
  diagnostic.task_valid = state.task_aperture_valid;
  diagnostic.mechanism_valid = state.mechanism_position_valid;
  diagnostic.force_valid = state.force_valid;
  diagnostic.safety_status_valid = state.safety_status_valid;
  diagnostic.safety_1_pushed = state.safety_1_pushed;
  diagnostic.safety_1_triggered = state.safety_1_triggered;
  diagnostic.safety_2_pushed = state.safety_2_pushed;
  diagnostic.safety_2_triggered = state.safety_2_triggered;
  diagnostic.safety_dc_error = state.safety_dc_error;
  for (std::size_t index = 0;
       index < onrobot_gripper_msgs::kDiagnosticInterfaceCount; ++index) {
    diagnostic.device_diagnostics[index] =
        value(m_diagnosticInterfaceNames[index]);
  }
  const auto identity = onrobot_gripper_msgs::firmwareIdentityText(diagnostic.device_diagnostics);
  state.firmware.assign(identity.data.data(), identity.size);
  m_diagnosticBox.try_set(diagnostic);

  m_realtimePublisher->unlockAndPublish();
  m_lastPublish = i_time;
  return controller_interface::return_type::OK;
}

double GripperStateBroadcaster::value(const std::string &i_name) const {
  const auto found =
      std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                   [&i_name](const auto &interface) {
                     return interface.get_name() == i_name;
                   });
  if (found == state_interfaces_.end()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return found->template get_optional<double>(1).value_or(
      std::numeric_limits<double>::quiet_NaN());
}

bool GripperStateBroadcaster::valid(const std::string &i_name) const {
  const double candidate = value(i_name);
  return std::isfinite(candidate) && candidate > 0.5;
}

void GripperStateBroadcaster::publishDiagnostics() {
  const auto snapshot = m_diagnosticBox.try_get();
  if (!snapshot || !m_diagnosticPublisher) {
    return;
  }
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = get_node()->now();
  const auto publishSteadyNs = steadyNowNs();
  const double sampleAge = advancedAge(
      snapshot->sample_age_s, snapshot->captured_at_steady_ns, publishSteadyNs);
  const auto diagnosticAgeIndex = onrobot_gripper_msgs::diagnosticIndex(
      onrobot_gripper_msgs::DiagnosticInterface::Age);
  const double diagnosticAge =
      advancedAge(snapshot->device_diagnostics[diagnosticAgeIndex],
                  snapshot->captured_at_steady_ns, publishSteadyNs);
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name =
      get_node()->get_fully_qualified_name() + std::string("/gripper");
  status.hardware_id = m_model;
  if (snapshot->connection_state == GripperState::CONNECTION_FAULTED ||
      (snapshot->safety_status_valid && snapshot->safety_dc_error)) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = snapshot->safety_dc_error ? "RG safety DC error"
                                               : "gripper session faulted";
  } else if (snapshot->safety_status_valid &&
             (snapshot->safety_1_pushed || snapshot->safety_2_pushed ||
              snapshot->safety_1_triggered || snapshot->safety_2_triggered)) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = snapshot->safety_1_pushed || snapshot->safety_2_pushed
                         ? "RG safety switch is pushed"
                         : "RG safety switch has triggered";
  } else if (
      !std::isfinite(
          snapshot->device_diagnostics[onrobot_gripper_msgs::diagnosticIndex(
              onrobot_gripper_msgs::DiagnosticInterface::StatusValid)]) ||
      snapshot->device_diagnostics[onrobot_gripper_msgs::diagnosticIndex(
          onrobot_gripper_msgs::DiagnosticInterface::StatusValid)] < 0.5) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "device diagnostics unavailable";
  } else if (std::isfinite(diagnosticAge) &&
             diagnosticAge > std::max(2.0, 3.0 / m_diagnosticRateHz)) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "device diagnostics are stale";
  } else if (!snapshot->task_valid || !snapshot->mechanism_valid) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "gripper state is incomplete or not yet valid";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "gripper state valid";
  }
  const auto identity = onrobot_gripper_msgs::firmwareIdentityText(
      snapshot->device_diagnostics);
  const std::string observedFirmware = identity.size
      ? std::string(identity.data.data(), identity.size) : "unknown";
  status.values = {
      keyValue("model", m_model),
      keyValue("firmware", observedFirmware),
      keyValue("firmware_source", identity.size ? "device-connection" : "unavailable"),
      keyValue("device_profile_revision", m_deviceProfileRevision),
      keyValue("finger_profile", m_fingerProfileName),
      keyValue("finger_profile_revision", m_fingerProfileRevision),
      keyValue("mapping_source", m_mappingSource),
      keyValue("connection_state", std::to_string(snapshot->connection_state)),
      keyValue("sample_sequence", std::to_string(snapshot->sample_sequence)),
      keyValue("sample_age_s", std::to_string(sampleAge)),
      keyValue("active_mode", std::to_string(snapshot->active_mode)),
      keyValue("successful_cycles",
               std::to_string(snapshot->successful_cycles)),
      keyValue("failed_cycles", std::to_string(snapshot->failed_cycles)),
      keyValue("missed_deadlines", std::to_string(snapshot->missed_deadlines)),
      keyValue("watchdog_stops", std::to_string(snapshot->watchdog_stops)),
      keyValue("reconnects", std::to_string(snapshot->reconnects)),
      keyValue("last_cycle_duration_s",
               std::to_string(snapshot->last_cycle_duration_s)),
      keyValue("task_aperture_valid", snapshot->task_valid ? "true" : "false"),
      keyValue("mechanism_position_valid",
               snapshot->mechanism_valid ? "true" : "false"),
      keyValue("force_valid", snapshot->force_valid ? "true" : "false"),
      keyValue("safety_status_valid",
               snapshot->safety_status_valid ? "true" : "false"),
      keyValue("safety_1_pushed", snapshot->safety_1_pushed ? "true" : "false"),
      keyValue("safety_1_triggered",
               snapshot->safety_1_triggered ? "true" : "false"),
      keyValue("safety_2_pushed", snapshot->safety_2_pushed ? "true" : "false"),
      keyValue("safety_2_triggered",
               snapshot->safety_2_triggered ? "true" : "false"),
      keyValue("safety_dc_error", snapshot->safety_dc_error ? "true" : "false"),
      keyValue("firmware_qualification",
               std::to_string(snapshot->firmware_qualification)),
      keyValue("mapping_validity", std::to_string(snapshot->mapping_validity)),
      keyValue("fault_source", std::to_string(snapshot->fault_source)),
      keyValue("fault_code", std::to_string(snapshot->fault_code)),
  };
  if (!m_firmware.empty()) {
    status.values.push_back(keyValue("configured_firmware", m_firmware));
  }
  for (std::size_t index = 0;
       index < onrobot_gripper_msgs::kDiagnosticInterfaceCount; ++index) {
    const double value = index == diagnosticAgeIndex
                             ? diagnosticAge
                             : snapshot->device_diagnostics[index];
    if (std::isfinite(value)) {
      status.values.push_back(
          keyValue(onrobot_gripper_msgs::kDiagnosticInterfaceNames[index],
                   std::to_string(value)));
    }
  }
  array.status.push_back(std::move(status));
  m_diagnosticPublisher->publish(array);
}

std::vector<std::string> GripperStateBroadcaster::requestedInterfaces() const {
  const std::string &task = m_taskJoint;
  const std::string &mechanism = m_mechanismJoint;
  const std::string measuredPosition = m_mechanismDimension == "linear"
                                           ? "measured_position"
                                           : "measured_angular_position";
  const std::string measuredVelocity = m_mechanismDimension == "linear"
                                           ? "measured_velocity"
                                           : "measured_angular_velocity";
  std::vector<std::string> interfaces{
      task + "/position",
      task + "/velocity",
      task + "/effort",
      task + "/task_position_valid",
      task + "/force_valid",
      task + "/busy",
      task + "/grip_detected",
      task + "/active_mode",
      task + "/connection_state",
      task + "/faulted",
      task + "/fault_code",
      task + "/firmware_qualification",
      task + "/realtime_force_control_available",
      task + "/sample_sequence",
      task + "/sample_age",
      task + "/successful_cycles",
      task + "/failed_cycles",
      task + "/missed_deadlines",
      task + "/watchdog_stops",
      task + "/reconnects",
      task + "/last_cycle_duration",
      mechanism + "/" + measuredPosition,
      mechanism + "/" + measuredVelocity,
      mechanism + "/position_valid",
      mechanism + "/velocity_valid",
  };
  for (const char *name : onrobot_gripper_msgs::kDiagnosticInterfaceNames) {
    interfaces.push_back(task + "/" + name);
  }
  if (m_model == "rg2" || m_model == "rg6") {
    interfaces.insert(interfaces.end(),
                      {task + "/safety_status_valid", task + "/safety_1_pushed",
                       task + "/safety_1_triggered", task + "/safety_2_pushed",
                       task + "/safety_2_triggered",
                       task + "/safety_dc_error"});
  }
  return interfaces;
}

} // namespace onrobot_gripper_controllers

PLUGINLIB_EXPORT_CLASS(onrobot_gripper_controllers::GripperStateBroadcaster,
                       controller_interface::ControllerInterface)

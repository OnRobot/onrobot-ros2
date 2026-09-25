#include "onrobot_gripper_controllers/gripper_recovery_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace onrobot_gripper_controllers {
namespace {
namespace protocol = onrobot_gripper_msgs::recovery;
constexpr std::size_t kExtension = 6;
constexpr std::size_t kCoreCount = kExtension + protocol::Count;
constexpr std::size_t kFaulted = 0, kAck = 4, kAdmission = 5;
constexpr std::size_t field(protocol::Field value) {
  return kExtension + value;
}
bool flag(double value) { return value == 0 || value == 1; }
bool validCode(double value) {
  return (value >= 0 && value <= 9 && std::floor(value) == value) ||
         value == 255;
}
} // namespace

controller_interface::InterfaceConfiguration
GripperRecoveryController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {m_jointName + "/fault_recovery_command_sequence"}};
}

controller_interface::InterfaceConfiguration
GripperRecoveryController::state_interface_configuration() const {
  std::vector<std::string> names{
      m_jointName + "/faulted",
      m_jointName + "/connection_state",
      m_jointName + "/reconnects",
      m_jointName + "/failed_cycles",
      m_jointName + "/fault_recovery_command_sequence_ack",
      m_jointName + "/fault_recovery_command_admission"};
  for (const auto name : protocol::names)
    names.push_back(m_jointName + "/" + name);
  if (m_safetyStatusSupported) {
    for (const auto name : {"safety_status_valid", "safety_1_pushed",
                            "safety_2_pushed", "safety_dc_error"})
      names.push_back(m_jointName + "/" + name);
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          std::move(names)};
}

controller_interface::CallbackReturn GripperRecoveryController::on_init() {
  try {
    get_node()->declare_parameter<std::string>("joint", m_jointName);
    get_node()->declare_parameter<bool>("safety_status_supported", false);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Recovery init failed: %s",
                 error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GripperRecoveryController::on_configure(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(m_requestMutex);
  if (m_outstandingSequence != 0)
    return controller_interface::CallbackReturn::FAILURE;
  if (is_async() || (get_node()->has_parameter("is_async") &&
                     get_node()->get_parameter("is_async").as_bool())) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Recovery requires synchronous controller read/update/write");
    return controller_interface::CallbackReturn::ERROR;
  }
  m_jointName = get_node()->get_parameter("joint").as_string();
  m_safetyStatusSupported =
      get_node()->get_parameter("safety_status_supported").as_bool();
  if (m_jointName.empty())
    return controller_interface::CallbackReturn::ERROR;
  m_active = m_deviceStateSeen = false;
  m_observed.fill(0);
  m_pendingSequence = 0;
  m_statusPublisher =
      std::make_unique<realtime_tools::RealtimePublisher<Status>>(
          get_node()->create_publisher<Status>(
              "~/state", rclcpp::QoS(1).reliable().transient_local()));
  m_recoveryService = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/recover",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::lock_guard<std::mutex> lock(m_requestMutex);
        const auto reject = [&](const char *why) {
          response->success = false;
          response->message = std::string("recovery rejected: ") + why;
        };
        if (!m_active)
          return reject("controller is inactive");
        if (!m_deviceStateSeen)
          return reject(
              "gripper state unavailable or recovery protocol invalid");
        if (m_outstandingSequence)
          return reject("another request is pending");
        if (m_observed[field(protocol::SafetyRequired)] == 1) {
          if (!m_safetyStatusSupported || m_observed[kCoreCount] != 1)
            return reject("RG safety state unavailable");
          if (m_observed[kCoreCount + 1] || m_observed[kCoreCount + 2])
            return reject("release the RG safety switch first");
          if (m_observed[kCoreCount + 3])
            return reject("RG safety DC error requires a physical power cycle");
        }
        if (m_observed[kFaulted] != 1)
          return reject("gripper is not faulted");
        if (m_observed[field(protocol::Ready)] != 1)
          return reject("backend is not ready for recovery");
        const auto ticks =
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();
        if (m_highWater >= protocol::maximum_sequence)
          return reject("command sequence space is exhausted");
        const uint64_t sequence =
            std::max(m_highWater + 1,
                     ticks > 0 ? static_cast<uint64_t>(ticks) : uint64_t{1});
        if (sequence > protocol::maximum_sequence)
          return reject("command sequence space is exhausted");
        m_highWater = m_outstandingSequence = m_pendingSequence = sequence;
        response->success = true;
        response->message = "recovery queued (request " +
                            std::to_string(sequence) +
                            "); hardware admission and completion are pending";
      });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GripperRecoveryController::on_activate(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(m_requestMutex);
  m_deviceStateSeen = false;
  const auto expected = kCoreCount + (m_safetyStatusSupported ? 4 : 0);
  if (m_outstandingSequence == 0 &&
      (state_interfaces_.size() != expected ||
       state_interfaces_[field(protocol::Version)]
               .get_optional<double>(1)
               .value_or(0) != protocol::version)) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Recovery requires complete backend protocol version 2 interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }
  m_active = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GripperRecoveryController::on_deactivate(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(m_requestMutex);
  m_active = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GripperRecoveryController::on_cleanup(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(m_requestMutex);
  m_active = false;
  if (m_outstandingSequence) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Recovery cleanup refused: request unresolved; reactivate to "
                 "observe its result");
    return controller_interface::CallbackReturn::FAILURE;
  }
  m_recoveryService.reset();
  m_statusPublisher.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
GripperRecoveryController::update(const rclcpp::Time &time,
                                  const rclcpp::Duration &) {
  std::unique_lock<std::mutex> lock(m_requestMutex, std::try_to_lock);
  if (!lock.owns_lock())
    return controller_interface::return_type::OK;
  const auto expected = kCoreCount + (m_safetyStatusSupported ? 4 : 0);
  auto next = m_observed;
  bool valid = state_interfaces_.size() == expected;
  if (valid)
    for (std::size_t i = 0; i < expected; ++i) {
      const auto value = state_interfaces_[i].get_optional<double>(1);
      if (!value || !std::isfinite(*value)) {
        valid = false;
        break;
      }
      next[i] = *value;
    }
  if (valid) {
    const double outcome = next[field(protocol::Outcome)],
                 code = next[field(protocol::Reason)];
    valid = next[field(protocol::Version)] == protocol::version &&
            flag(next[kFaulted]) && flag(next[field(protocol::Ready)]) &&
            (next[field(protocol::Ready)] == 0 || next[kFaulted] == 1) &&
            flag(next[field(protocol::SafetyRequired)]) &&
            protocol::validSequence(next[kAck]) &&
            protocol::validSequence(next[field(protocol::Active)]) &&
            protocol::validSequence(next[field(protocol::ResultSequence)]) &&
            (next[kAdmission] == 0 || next[kAdmission] == 1 ||
             next[kAdmission] == 2) &&
            ((next[kAdmission] == 0) == (next[kAck] == 0)) && validCode(code) &&
            ((outcome == 0 && next[field(protocol::ResultSequence)] == 0 &&
              code == 0) ||
             (next[field(protocol::ResultSequence)] > 0 &&
              ((outcome == 1 && code == 0) || (outcome == 2 && code != 0) ||
               (outcome == 3 &&
                code == static_cast<double>(protocol::Code::Cancelled))))) &&
            !(next[field(protocol::Active)] > 0 &&
              (next[field(protocol::Ready)] != 0 || next[kAdmission] != 1 ||
               next[kAck] != next[field(protocol::Active)] ||
               next[field(protocol::Active)] ==
                   next[field(protocol::ResultSequence)]));
    if (valid && next[field(protocol::SafetyRequired)] == 1) {
      valid = m_safetyStatusSupported;
      if (valid)
        for (std::size_t i = kCoreCount; i < expected; ++i)
          valid = valid && flag(next[i]);
    }
  }
  m_deviceStateSeen = valid;
  if (valid) {
    m_observed = next;
    for (const auto i :
         {kAck, field(protocol::Active), field(protocol::ResultSequence)})
      m_highWater = std::max(m_highWater, static_cast<uint64_t>(next[i]));
    // A replacement observer adopts ownership; it never reissues the operation.
    if (m_outstandingSequence == 0 && next[field(protocol::Active)] > 0)
      m_outstandingSequence =
          static_cast<uint64_t>(next[field(protocol::Active)]);
    if (m_outstandingSequence &&
        ((next[kAck] == static_cast<double>(m_outstandingSequence) &&
          next[kAdmission] == 2) ||
         (next[field(protocol::ResultSequence)] ==
              static_cast<double>(m_outstandingSequence) &&
          next[field(protocol::Outcome)] != 0))) {
      if (m_pendingSequence == m_outstandingSequence)
        m_pendingSequence = 0;
      m_outstandingSequence = 0;
    }
  }
  if (m_active && valid && m_pendingSequence) {
    // Cancellation is local only before export; after export only the backend
    // can authoritatively reject or terminate the exact request.
    const bool unsafe = next[field(protocol::SafetyRequired)] == 1 &&
                        (next[kCoreCount] != 1 || next[kCoreCount + 1] ||
                         next[kCoreCount + 2] || next[kCoreCount + 3]);
    if (unsafe) {
      RCLCPP_WARN(
          get_node()->get_logger(),
          "Recovery request %llu cancelled before export: safety state changed",
          static_cast<unsigned long long>(m_pendingSequence));
      m_cancelledSequence = m_pendingSequence;
      m_pendingSequence = m_outstandingSequence = 0;
    } else if (command_interfaces_.size() == 1 &&
               command_interfaces_[0].set_value(
                   static_cast<double>(m_pendingSequence))) {
      m_pendingSequence = 0;
    }
  }
  if (m_statusPublisher && m_statusPublisher->trylock()) {
    auto &message = m_statusPublisher->msg_;
    message.header.stamp = time;
    message.observation_valid = valid;
    message.ready = m_active && valid && !m_outstandingSequence &&
                    m_observed[field(protocol::Ready)] == 1;
    message.request_sequence = m_outstandingSequence;
    message.cancelled_sequence = m_cancelledSequence;
    message.admission_sequence = static_cast<uint64_t>(m_observed[kAck]);
    message.admission = static_cast<uint8_t>(m_observed[kAdmission]);
    message.active_sequence =
        static_cast<uint64_t>(m_observed[field(protocol::Active)]);
    message.result_sequence =
        static_cast<uint64_t>(m_observed[field(protocol::ResultSequence)]);
    message.result = static_cast<uint8_t>(m_observed[field(protocol::Outcome)]);
    message.result_code =
        static_cast<uint8_t>(m_observed[field(protocol::Reason)]);
    m_statusPublisher->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}
} // namespace onrobot_gripper_controllers
PLUGINLIB_EXPORT_CLASS(onrobot_gripper_controllers::GripperRecoveryController,
                       controller_interface::ControllerInterface)

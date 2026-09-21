#include "onrobot_gripper_controllers/gripper_recovery_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <optional>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace onrobot_gripper_controllers {
namespace {

constexpr std::size_t kCoreStateInterfaceCount = 6;
constexpr double kMaximumExactSequence = 9007199254740991.0;
constexpr double kConnectionIdle = 3.0;
constexpr double kConnectionFaulted = 6.0;
constexpr double kAdmissionAccepted = 1.0;
constexpr double kAdmissionRejected = 2.0;

bool nextRecoverySequence(double last_acknowledged,
                          uint64_t &sequence) noexcept {
  // A monotonic-clock token survives controller plugin unload/reload, unlike
  // a counter stored in the plugin's static storage. The hardware's retained
  // acknowledgement provides a second guard if two requests share a clock tick.
  const auto ticks = std::chrono::duration_cast<std::chrono::microseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
  uint64_t candidate = ticks > 0 ? static_cast<uint64_t>(ticks) : 1U;
  if (std::isfinite(last_acknowledged) && last_acknowledged > 0.0) {
    if (last_acknowledged > kMaximumExactSequence ||
        std::floor(last_acknowledged) != last_acknowledged) {
      return false;
    }
    const auto last = static_cast<uint64_t>(last_acknowledged);
    if (last >= static_cast<uint64_t>(kMaximumExactSequence)) {
      return false;
    }
    candidate = std::max(candidate, last + 1U);
  }
  if (candidate == 0 ||
      static_cast<double>(candidate) > kMaximumExactSequence) {
    return false;
  }
  sequence = candidate;
  return true;
}

void clearMatching(std::atomic<uint64_t> &value, uint64_t sequence) noexcept {
  (void)value.compare_exchange_strong(sequence, 0, std::memory_order_acq_rel,
                                      std::memory_order_acquire);
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
  if (m_safetyStatusSupported) {
    names.insert(names.end(),
                 {m_jointName + "/safety_status_valid",
                  m_jointName + "/safety_1_pushed",
                  m_jointName + "/safety_2_pushed",
                  m_jointName + "/safety_dc_error"});
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
  m_jointName = get_node()->get_parameter("joint").as_string();
  m_safetyStatusSupported =
      get_node()->get_parameter("safety_status_supported").as_bool();
  if (m_jointName.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Recovery joint cannot be empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  {
    std::lock_guard<std::mutex> lock(m_requestMutex);
    m_active.store(false, std::memory_order_release);
  }
  m_deviceStateSeen.store(false, std::memory_order_release);
  m_faulted.store(false, std::memory_order_release);
  m_connectionState.store(0.0, std::memory_order_release);
  m_reconnects.store(0.0, std::memory_order_release);
  m_failedCycles.store(0.0, std::memory_order_release);
  m_safetySeen.store(false, std::memory_order_release);
  m_safetyPushed.store(false, std::memory_order_release);
  m_safetyDcError.store(false, std::memory_order_release);
  m_pendingSequence.store(0, std::memory_order_release);
  m_outstandingSequence.store(0, std::memory_order_release);
  m_recoveryAckSequence.store(0.0, std::memory_order_release);
  m_recoveryAdmission.store(0.0, std::memory_order_release);
  m_backendAdmitted.store(false, std::memory_order_release);

  m_recoveryService = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/recover",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::lock_guard<std::mutex> lock(m_requestMutex);
        if (!m_active.load(std::memory_order_acquire)) {
          response->success = false;
          response->message = "recovery rejected: controller is inactive";
          return;
        }
        if (!m_deviceStateSeen.load(std::memory_order_acquire)) {
          response->success = false;
          response->message = "recovery rejected: gripper state unavailable";
          return;
        }
        if (!m_faulted.load(std::memory_order_acquire) ||
            m_connectionState.load(std::memory_order_acquire) !=
                kConnectionFaulted) {
          response->success = false;
          response->message = "recovery rejected: gripper is not faulted";
          return;
        }
        if (m_safetyStatusSupported &&
            !m_safetySeen.load(std::memory_order_acquire)) {
          response->success = false;
          response->message = "recovery rejected: RG safety state unavailable";
          return;
        }
        if (m_safetyPushed.load(std::memory_order_acquire)) {
          response->success = false;
          response->message =
              "recovery rejected: release the RG safety switch first";
          return;
        }
        if (m_safetyDcError.load(std::memory_order_acquire)) {
          response->success = false;
          response->message =
              "recovery rejected: RG safety DC error requires a physical power cycle";
          return;
        }
        if (m_outstandingSequence.load(std::memory_order_acquire) != 0) {
          response->success = false;
          response->message = "recovery rejected: another request is pending";
          return;
        }

        uint64_t sequence = 0;
        if (!nextRecoverySequence(
                m_recoveryAckSequence.load(std::memory_order_acquire),
                sequence)) {
          response->success = false;
          response->message =
              "recovery rejected: command sequence space is exhausted";
          return;
        }
        m_reconnectsAtRequest.store(
            m_reconnects.load(std::memory_order_acquire),
            std::memory_order_release);
        m_failedCyclesAtRequest.store(
            m_failedCycles.load(std::memory_order_acquire),
            std::memory_order_release);
        m_backendAdmitted.store(false, std::memory_order_release);
        m_outstandingSequence.store(sequence, std::memory_order_release);
        m_pendingSequence.store(sequence, std::memory_order_release);
        response->success = true;
        response->message = "recovery queued (request " +
                            std::to_string(sequence) +
                            "); hardware admission and completion are pending";
        RCLCPP_INFO(get_node()->get_logger(),
                    "Queued recovery request %llu; awaiting hardware admission",
                    static_cast<unsigned long long>(sequence));
      });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GripperRecoveryController::on_activate(const rclcpp_lifecycle::State &) {
  m_deviceStateSeen.store(false, std::memory_order_release);
  m_safetySeen.store(false, std::memory_order_release);
  std::lock_guard<std::mutex> lock(m_requestMutex);
  m_active.store(true, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GripperRecoveryController::on_deactivate(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(m_requestMutex);
  m_active.store(false, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GripperRecoveryController::on_cleanup(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(m_requestMutex);
  m_active.store(false, std::memory_order_release);
  if (m_outstandingSequence.load(std::memory_order_acquire) != 0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Recovery cleanup refused while an accepted request is "
                 "unresolved; reactivate the controller to observe or retry it");
    // FAILURE leaves the lifecycle controller inactive so its retained event
    // can be observed after reactivation. ERROR would move the controller into
    // error processing and could strand the accepted request.
    return controller_interface::CallbackReturn::FAILURE;
  }
  m_recoveryService.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
GripperRecoveryController::update(const rclcpp::Time &,
                                  const rclcpp::Duration &) {
  const std::size_t expected_state_count =
      kCoreStateInterfaceCount + (m_safetyStatusSupported ? 4U : 0U);
  if (state_interfaces_.size() == expected_state_count) {
    const auto read_finite = [this](std::size_t index,
                                    double &value) -> bool {
      const auto observed = state_interfaces_[index].get_optional<double>(1);
      if (!observed || !std::isfinite(*observed)) {
        return false;
      }
      value = *observed;
      return true;
    };
    double faulted = 0.0;
    double connection = 0.0;
    double reconnects = 0.0;
    double failed_cycles = 0.0;
    double ack_sequence = 0.0;
    double admission = 0.0;
    const bool valid = read_finite(0, faulted) && read_finite(1, connection) &&
                       read_finite(2, reconnects) &&
                       read_finite(3, failed_cycles) &&
                       read_finite(4, ack_sequence) &&
                       read_finite(5, admission);
    m_deviceStateSeen.store(valid, std::memory_order_release);
    if (valid) {
      m_faulted.store(faulted > 0.5, std::memory_order_release);
      m_connectionState.store(connection, std::memory_order_release);
      m_reconnects.store(reconnects, std::memory_order_release);
      m_failedCycles.store(failed_cycles, std::memory_order_release);
      m_recoveryAckSequence.store(ack_sequence, std::memory_order_release);
      m_recoveryAdmission.store(admission, std::memory_order_release);
    }
    if (m_safetyStatusSupported) {
      const auto read_flag = [this](std::size_t index) {
        const auto value = state_interfaces_[index].get_optional<double>(1);
        return value && std::isfinite(*value) ? std::optional<double>(*value)
                                               : std::nullopt;
      };
      const auto valid_state = read_flag(6);
      const auto safety_one = read_flag(7);
      const auto safety_two = read_flag(8);
      const auto safety_dc_error = read_flag(9);
      const bool safety_valid = valid_state && *valid_state > 0.5 &&
                                safety_one && safety_two && safety_dc_error;
      m_safetySeen.store(safety_valid, std::memory_order_release);
      m_safetyPushed.store(
          safety_valid && (*safety_one > 0.5 || *safety_two > 0.5),
          std::memory_order_release);
      m_safetyDcError.store(safety_valid && *safety_dc_error > 0.5,
                            std::memory_order_release);
    }
  } else {
    m_deviceStateSeen.store(false, std::memory_order_release);
    if (m_safetyStatusSupported) {
      m_safetySeen.store(false, std::memory_order_release);
    }
  }

  const uint64_t outstanding =
      m_outstandingSequence.load(std::memory_order_acquire);
  if (outstanding != 0 && m_deviceStateSeen.load(std::memory_order_acquire)) {
    const double ack = m_recoveryAckSequence.load(std::memory_order_acquire);
    const double admission = m_recoveryAdmission.load(std::memory_order_acquire);
    if (ack == static_cast<double>(outstanding)) {
      if (admission == kAdmissionRejected) {
        uint64_t expected = outstanding;
        if (m_outstandingSequence.compare_exchange_strong(
                expected, 0, std::memory_order_acq_rel,
                std::memory_order_acquire)) {
          clearMatching(m_pendingSequence, outstanding);
          m_backendAdmitted.store(false, std::memory_order_release);
          RCLCPP_ERROR(get_node()->get_logger(),
                       "Recovery request %llu was rejected by the hardware "
                       "backend (admission status %.0f)",
                       static_cast<unsigned long long>(outstanding), admission);
        }
      } else if (admission == kAdmissionAccepted) {
        if (!m_backendAdmitted.exchange(true, std::memory_order_acq_rel)) {
          RCLCPP_INFO(get_node()->get_logger(),
                      "Recovery request %llu admitted by the hardware "
                      "backend; awaiting device recovery",
                      static_cast<unsigned long long>(outstanding));
        }
      }
    }
    if (m_backendAdmitted.load(std::memory_order_acquire)) {
      const double connection =
          m_connectionState.load(std::memory_order_acquire);
      const bool completed =
          !m_faulted.load(std::memory_order_acquire) &&
          connection == kConnectionIdle &&
          m_reconnects.load(std::memory_order_acquire) >
              m_reconnectsAtRequest.load(std::memory_order_acquire);
      const bool failed =
          m_faulted.load(std::memory_order_acquire) &&
          connection == kConnectionFaulted &&
          m_failedCycles.load(std::memory_order_acquire) >
              m_failedCyclesAtRequest.load(std::memory_order_acquire);
      if (completed || failed) {
        uint64_t expected = outstanding;
        if (m_outstandingSequence.compare_exchange_strong(
                expected, 0, std::memory_order_acq_rel,
                std::memory_order_acquire)) {
          m_backendAdmitted.store(false, std::memory_order_release);
          if (completed) {
            RCLCPP_INFO(get_node()->get_logger(),
                        "Recovery request %llu completed; device is active "
                        "and idle",
                        static_cast<unsigned long long>(outstanding));
          } else {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Recovery request %llu failed; device remains faulted",
                         static_cast<unsigned long long>(outstanding));
          }
        }
      }
    }
  }

  const uint64_t sequence =
      m_pendingSequence.load(std::memory_order_acquire);
  if (sequence == 0 || !m_active.load(std::memory_order_acquire)) {
    return controller_interface::return_type::OK;
  }
  if (m_safetyStatusSupported) {
    if (!m_safetySeen.load(std::memory_order_acquire)) {
      return controller_interface::return_type::OK;
    }
    if (m_safetyPushed.load(std::memory_order_acquire) ||
        m_safetyDcError.load(std::memory_order_acquire)) {
      uint64_t expected_pending = sequence;
      if (m_pendingSequence.compare_exchange_strong(
              expected_pending, 0, std::memory_order_acq_rel,
              std::memory_order_acquire)) {
        clearMatching(m_outstandingSequence, sequence);
        m_backendAdmitted.store(false, std::memory_order_release);
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Queued recovery request %llu cancelled because RG "
                     "safety state is no longer clear",
                     static_cast<unsigned long long>(sequence));
      }
      return controller_interface::return_type::OK;
    }
  }
  if (command_interfaces_.size() != 1) {
    // Keep accepted intent until the command interface can accept the handoff.
    return controller_interface::return_type::OK;
  }
  if (!command_interfaces_.front().set_value(static_cast<double>(sequence))) {
    return controller_interface::return_type::OK;
  }
  clearMatching(m_pendingSequence, sequence);
  return controller_interface::return_type::OK;
}

} // namespace onrobot_gripper_controllers

PLUGINLIB_EXPORT_CLASS(onrobot_gripper_controllers::GripperRecoveryController,
                       controller_interface::ControllerInterface)

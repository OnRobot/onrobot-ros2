#include "onrobot_gripper_controllers/gripper_recovery_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace onrobot_gripper_controllers {

controller_interface::InterfaceConfiguration
GripperRecoveryController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {m_jointName + "/fault_recovery_command_sequence"}};
}

controller_interface::InterfaceConfiguration
GripperRecoveryController::state_interface_configuration() const {
  if (!m_safetyStatusSupported) {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {m_jointName + "/safety_status_valid",
           m_jointName + "/safety_1_pushed", m_jointName + "/safety_2_pushed",
           m_jointName + "/safety_dc_error"}};
}

controller_interface::CallbackReturn GripperRecoveryController::on_init() {
  try {
    get_node()->declare_parameter<std::string>("joint", m_jointName);
    get_node()->declare_parameter<bool>("safety_status_supported", false);
  } catch (const std::exception &i_error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Recovery init failed: %s",
                 i_error.what());
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
  m_recoveryService = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/recover",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
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
        m_pendingSequence.store(m_nextSequence.fetch_add(1),
                                std::memory_order_release);
        response->success = true;
        response->message =
            "safe stop/recovery queued; observe GripperState connection_state";
      });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GripperRecoveryController::on_activate(const rclcpp_lifecycle::State &) {
  m_pendingSequence.store(0, std::memory_order_release);
  m_safetySeen.store(false, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GripperRecoveryController::on_deactivate(const rclcpp_lifecycle::State &) {
  m_pendingSequence.store(0, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
GripperRecoveryController::update(const rclcpp::Time &,
                                  const rclcpp::Duration &) {
  if (m_safetyStatusSupported && state_interfaces_.size() == 4) {
    const auto read_flag = [this](std::size_t index) {
      return state_interfaces_[index].get_optional<double>(1).value_or(0.0) >
             0.5;
    };
    const bool valid = read_flag(0);
    m_safetySeen.store(valid, std::memory_order_release);
    m_safetyPushed.store(valid && (read_flag(1) || read_flag(2)),
                         std::memory_order_release);
    m_safetyDcError.store(valid && read_flag(3), std::memory_order_release);
  }
  const uint64_t sequence =
      m_pendingSequence.exchange(0, std::memory_order_acq_rel);
  if (sequence == 0) {
    return controller_interface::return_type::OK;
  }
  if (command_interfaces_.size() != 1 ||
      !command_interfaces_.front().set_value(static_cast<double>(sequence))) {
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

} // namespace onrobot_gripper_controllers

PLUGINLIB_EXPORT_CLASS(onrobot_gripper_controllers::GripperRecoveryController,
                       controller_interface::ControllerInterface)

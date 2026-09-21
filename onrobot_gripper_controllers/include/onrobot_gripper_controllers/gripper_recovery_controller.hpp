#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace onrobot_gripper_controllers {

/// Queues explicit, worker-owned recovery without claiming motion resources.
class GripperRecoveryController final
    : public controller_interface::ControllerInterface {
public:
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time &,
                                           const rclcpp::Duration &) override;

private:
  std::string m_jointName{"grip_stroke"};
  bool m_safetyStatusSupported{false};
  std::mutex m_requestMutex;
  std::atomic<bool> m_active{false};
  std::atomic<bool> m_deviceStateSeen{false};
  std::atomic<bool> m_faulted{false};
  std::atomic<double> m_connectionState{0.0};
  std::atomic<double> m_reconnects{0.0};
  std::atomic<double> m_failedCycles{0.0};
  std::atomic<bool> m_safetySeen{false};
  std::atomic<bool> m_safetyPushed{false};
  std::atomic<bool> m_safetyDcError{false};
  std::atomic<uint64_t> m_pendingSequence{0};
  std::atomic<uint64_t> m_outstandingSequence{0};
  std::atomic<double> m_reconnectsAtRequest{0.0};
  std::atomic<double> m_failedCyclesAtRequest{0.0};
  std::atomic<bool> m_backendAdmitted{false};
  std::atomic<double> m_recoveryAckSequence{0.0};
  std::atomic<double> m_recoveryAdmission{0.0};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_recoveryService;
};

} // namespace onrobot_gripper_controllers

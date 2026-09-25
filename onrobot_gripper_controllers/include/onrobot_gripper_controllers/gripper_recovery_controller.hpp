#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <onrobot_gripper_msgs/fault_recovery_protocol.hpp>
#include <onrobot_gripper_msgs/msg/recovery_state.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
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
  // One coherent observation; update takes the service mutex with try_lock
  // only.
  bool m_active{false};
  bool m_deviceStateSeen{false};
  std::array<double, 6 + onrobot_gripper_msgs::recovery::Count + 4>
      m_observed{};
  uint64_t m_pendingSequence{0};
  uint64_t m_outstandingSequence{0};
  uint64_t m_highWater{0};
  uint64_t m_cancelledSequence{0};
  using Status = onrobot_gripper_msgs::msg::RecoveryState;
  std::unique_ptr<realtime_tools::RealtimePublisher<Status>> m_statusPublisher;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_recoveryService;
};

} // namespace onrobot_gripper_controllers

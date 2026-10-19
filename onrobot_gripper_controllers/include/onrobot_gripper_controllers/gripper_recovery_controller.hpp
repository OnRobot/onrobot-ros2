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

/// Queues explicit recovery requests without claiming motion interfaces.
class GripperRecoveryController final
    : public controller_interface::ControllerInterface {
public:
  /// @brief Claim the recovery-request interface on the configured joint.
  ///
  /// Motion interfaces remain available to the motion controller.
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  /// @brief Request the fault, connection and recovery-result interfaces.
  ///
  /// Also requests RG safety interfaces when safety_status_supported is true.
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  /// @brief Declare the joint and safety_status_supported parameters.
  ///
  /// Returns ERROR if parameter declaration fails.
  controller_interface::CallbackReturn on_init() override;
  /// @brief Create the recover service and recovery-state publisher.
  ///
  /// Requires a nonempty joint name and synchronous controller updates. Returns
  /// FAILURE while a previous recovery is unresolved, or ERROR for invalid
  /// settings. A successful service response means a request was queued; the
  /// state topic reports its outcome.
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;
  /// @brief Enable recovery requests after checking the backend interfaces.
  ///
  /// Preserves an outstanding request so its result can still be observed.
  /// Returns ERROR when starting without the required recovery interfaces.
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;
  /// @brief Reject new service requests while keeping any outstanding recovery.
  ///
  /// Does not cancel recovery already passed to the backend.
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;
  /// @brief Remove the recovery service and state publisher.
  ///
  /// Returns FAILURE if recovery is unresolved. Activate the controller again
  /// to observe that result before cleanup.
  controller_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;
  /// @brief Pass queued recovery requests to hardware and publish their
  /// results.
  ///
  /// Uses the supplied ROS time for the state message stamp. Defers work if the
  /// request lock is busy. An OK return does not mean recovery has finished.
  controller_interface::return_type update(const rclcpp::Time &,
                                           const rclcpp::Duration &) override;

  /// @cond INTERNAL
private:
  std::string m_jointName{"grip_stroke"};
  bool m_safetyStatusSupported{false};
  std::mutex m_requestMutex;
  // Keep related recovery fields together. update uses try_lock on the service
  // mutex so it can skip this cycle instead of waiting for a callback.
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
  /// @endcond
};

} // namespace onrobot_gripper_controllers

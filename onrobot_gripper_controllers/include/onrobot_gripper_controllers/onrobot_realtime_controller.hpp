#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <onrobot_gripper_msgs/msg/realtime_command.hpp>
#include <onrobot_gripper_msgs/msg/realtime_state.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace onrobot_gripper_controllers {

class RealtimeControllerTestAccess;

/// @brief Resource-claimed realtime controller for 2FG and RG grippers.
///
/// The controller claims the conventional position/effort resources together
/// with all realtime command interfaces, so a conventional action controller
/// and realtime stream cannot own the same gripper concurrently. Device I/O
/// remains exclusively in ParallelGripperSession below the hardware plugin.
class OnRobotRealtimeController final
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
  controller_interface::return_type update(const rclcpp::Time &,
                                           const rclcpp::Duration &) override;

private:
  friend class RealtimeControllerTestAccess;
  struct PendingCommand {
    // Keep the RT handoff allocation-free.  In particular, do not carry the
    // ROS header's frame_id string into the control loop.
    uint8_t mode{onrobot_gripper_msgs::msg::RealtimeCommand::STOP};
    double task_position{0.0};
    double task_velocity{0.0};
    double mechanism_angular_velocity{0.0};
    double force{0.0};
    int64_t source_stamp_ns{0};
    int64_t received_at_steady_ns{0};
    uint64_t sequence{0};
    uint64_t epoch{0};
    bool valid{false};
  };

  bool writeInterface(std::size_t i_index, double i_value);
  bool writeStop(uint64_t i_sequence);
  double stateValue(std::size_t i_index) const;
  void queueStop();
  void requestStop();
  bool validMotion(const PendingCommand &i_command) const;
  bool freshSource(const PendingCommand &i_command, int64_t i_ros_now_ns) const;
  uint64_t nextSequence();
  void publishStopFence(uint64_t i_sequence, uint64_t i_epoch);
  controller_interface::return_type publishState(const rclcpp::Time &);

  std::string m_jointName{"grip_stroke"};
  std::string m_mechanismJointName{"finger_stroke"};
  bool m_rgCoordinateProfile{false};
  std::chrono::milliseconds m_timeout{100};
  double m_statePublishRateHz{100.0};
  std::atomic<uint64_t> m_nextSequence{1};
  std::atomic<uint64_t> m_epoch{0};
  std::atomic<bool> m_acceptMotion{false};
  std::atomic<int64_t> m_sourceBoundaryNs{0};
  // A Stop is never coalesced into the motion box.  The sequence is stored
  // last with release semantics, so update observes the complete fence.
  std::atomic<uint64_t> m_stopFenceSequence{0};
  std::atomic<uint64_t> m_stopFenceEpoch{0};
  std::mutex m_inputMutex;
  realtime_tools::RealtimeThreadSafeBox<PendingCommand> m_commandBox;
  PendingCommand m_cachedCommand;
  bool m_cachedValid{false};
  uint64_t m_seenSequence{0};
  uint64_t m_appliedSequence{0};
  uint64_t m_stopSequence{0};
  bool m_stopPending{true};
  bool m_activationBoundaryPending{false};
  int64_t m_lastRosNowNs{0};
  rclcpp::Subscription<onrobot_gripper_msgs::msg::RealtimeCommand>::SharedPtr
      m_subscription;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_stopService;
  std::unique_ptr<realtime_tools::RealtimePublisher<
      onrobot_gripper_msgs::msg::RealtimeState>>
      m_statePublisher;
  rclcpp::Time m_lastStatePublish;
};

} // namespace onrobot_gripper_controllers

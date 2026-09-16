#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <onrobot_gripper_msgs/diagnostic_interfaces.hpp>
#include <onrobot_gripper_msgs/msg/gripper_state.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>

namespace onrobot_gripper_controllers {

class GripperStateBroadcasterTestAccess;

/// Publishes the model-neutral state contract without owning device I/O.
class GripperStateBroadcaster final
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
  friend class GripperStateBroadcasterTestAccess;
  struct DiagnosticSnapshot {
    uint8_t connection_state{0};
    uint8_t active_mode{0};
    uint8_t mapping_validity{0};
    uint8_t firmware_qualification{0};
    uint8_t fault_source{0};
    uint16_t fault_code{0};
    uint64_t sample_sequence{0};
    uint64_t successful_cycles{0};
    uint64_t failed_cycles{0};
    uint64_t missed_deadlines{0};
    uint64_t watchdog_stops{0};
    uint64_t reconnects{0};
    double sample_age_s{0.0};
    double last_cycle_duration_s{0.0};
    int64_t captured_at_steady_ns{0};
    bool task_valid{false};
    bool mechanism_valid{false};
    bool force_valid{false};
    bool safety_status_valid{false};
    bool safety_1_pushed{false};
    bool safety_1_triggered{false};
    bool safety_2_pushed{false};
    bool safety_2_triggered{false};
    bool safety_dc_error{false};
    std::array<double, onrobot_gripper_msgs::kDiagnosticInterfaceCount>
        device_diagnostics{};
  };

  double value(const std::string &i_name) const;
  bool valid(const std::string &i_name) const;
  void publishDiagnostics();
  std::vector<std::string> requestedInterfaces() const;

  std::string m_model;
  std::string m_taskJoint{"grip_stroke"};
  std::string m_mechanismJoint{"finger_stroke"};
  std::string m_mechanismDimension{"linear"};
  std::string m_firmware;
  std::string m_deviceProfileRevision;
  std::string m_fingerProfileName;
  std::string m_fingerProfileRevision;
  std::string m_mappingSource{"unknown"};
  double m_publishRateHz{100.0};
  double m_diagnosticRateHz{1.0};
  std::string m_diagnosticsTopic{"diagnostics"};
  std::array<std::string, onrobot_gripper_msgs::kDiagnosticInterfaceCount>
      m_diagnosticInterfaceNames;
  rclcpp::Time m_lastPublish;

  using StatePublisher = realtime_tools::RealtimePublisher<
      onrobot_gripper_msgs::msg::GripperState>;
  std::unique_ptr<StatePublisher> m_realtimePublisher;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      m_diagnosticPublisher;
  rclcpp::TimerBase::SharedPtr m_diagnosticTimer;
  realtime_tools::RealtimeThreadSafeBox<DiagnosticSnapshot> m_diagnosticBox;
};

} // namespace onrobot_gripper_controllers

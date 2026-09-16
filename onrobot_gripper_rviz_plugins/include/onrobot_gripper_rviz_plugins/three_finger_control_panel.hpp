#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <limits>
#include <string>
#include <vector>

#include <QString>
#include <control_msgs/msg/float64_values.hpp>
#include <onrobot_gripper_msgs/msg/gripper_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "onrobot_gripper_rviz_plugins/panel_settings.hpp"

class QDoubleSpinBox;
class QLabel;
class QTimer;
class QWidget;

namespace onrobot_gripper_rviz_plugins {

/// RViz operator panel for the native three-finger diameter controller.
///
/// The panel intentionally does not use ParallelGripperCommand: 3FG tools
/// are coupled three-finger mechanisms. It publishes the external grip
/// diameter, in metres, to forward_command_controller and only observes the
/// state published by ros2_control.
class ThreeFingerControlPanel final : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit ThreeFingerControlPanel(QWidget *parent = nullptr);
  ~ThreeFingerControlPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config &config) override;
  void save(rviz_common::Config config) const override;

private:
  struct SharedState {
    std::atomic<bool> has_diameter{false};
    std::atomic<bool> has_angle{false};
    std::atomic<bool> has_limits{false};
    std::atomic<bool> has_gripper_state{false};
    std::atomic<double> diameter_m{0.0};
    std::atomic<double> diameter_velocity_m_s{std::numeric_limits<double>::quiet_NaN()};
    std::atomic<double> finger_angle_rad{0.0};
    std::atomic<double> finger_angle_velocity_rad_s{std::numeric_limits<double>::quiet_NaN()};
    std::atomic<double> minimum_diameter_m{0.0};
    std::atomic<double> maximum_diameter_m{0.0};
    std::atomic<std::int64_t> last_measurement_ns{0};
    std::atomic<std::int64_t> last_angle_ns{0};
    std::atomic<std::int64_t> last_gripper_state_ns{0};
    std::atomic<double> gripper_state_age_s{0.0};
    std::atomic<bool> task_position_valid{false};
    std::atomic<unsigned char> connection_state{
        onrobot_gripper_msgs::msg::GripperState::CONNECTION_UNKNOWN};
    std::atomic<unsigned short> fault_code{0};
  };

  void publishDiameter(double diameter_m);
  void updateLiveState();
  void setStatus(const QString &text);
  void readSettings();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscription_;
  rclcpp::Subscription<control_msgs::msg::Float64Values>::SharedPtr
      limit_subscription_;
  rclcpp::Subscription<onrobot_gripper_msgs::msg::GripperState>::SharedPtr
      gripper_state_subscription_;
  std::shared_ptr<SharedState> shared_state_;
  QTimer *state_timer_{nullptr};

  double minimum_diameter_m_{0.0};
  double maximum_diameter_m_{0.0};
  double configured_force_percent_{20.0};
  bool command_ready_{false};

  std::string command_topic_{"diameter_controller/commands"};
  std::string joint_states_topic_{"joint_states"};
  std::string limits_topic_{"three_finger_limit_broadcaster/values"};
  std::string gripper_state_topic_{"gripper_state_broadcaster/state"};
  mutable PanelSettings panel_settings_;

  QDoubleSpinBox *target_spin_box_{nullptr};
  QLabel *measured_label_{nullptr};
  QLabel *force_label_{nullptr};
  QLabel *status_label_{nullptr};
  std::vector<QWidget *> command_widgets_;
};

} // namespace onrobot_gripper_rviz_plugins

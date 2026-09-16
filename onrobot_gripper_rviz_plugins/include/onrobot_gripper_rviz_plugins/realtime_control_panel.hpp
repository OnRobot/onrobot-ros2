#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <QString>
#include <control_msgs/msg/float64_values.hpp>
#include <onrobot_gripper_msgs/msg/gripper_state.hpp>
#include <onrobot_gripper_msgs/msg/realtime_command.hpp>
#include <onrobot_gripper_msgs/msg/realtime_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "onrobot_gripper_rviz_plugins/realtime_panel_semantics.hpp"
#include "onrobot_gripper_rviz_plugins/panel_settings.hpp"

class QLabel;
class QComboBox;
class QDoubleSpinBox;
class QEvent;
class QPushButton;
class QResizeEvent;
class QSlider;
class QTimer;
class QWidget;

namespace onrobot_gripper_rviz_plugins {

/// @brief RViz panel for the product-specific realtime motion interface.
///
/// Position controls stream a target until it is reached or stopped. A signed
/// slider is neutral at center; holding it away from center publishes refreshed
/// VELOCITY commands and releasing it publishes an ordered Stop command.
class RealtimeControlPanel final : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit RealtimeControlPanel(QWidget *i_parent = nullptr);
  ~RealtimeControlPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config &i_config) override;
  void save(rviz_common::Config i_config) const override;

protected:
  bool eventFilter(QObject *i_watched, QEvent *i_event) override;
  void resizeEvent(QResizeEvent *i_event) override;

private:
  using RealtimeCommand = onrobot_gripper_msgs::msg::RealtimeCommand;
  using RealtimeState = onrobot_gripper_msgs::msg::RealtimeState;

  struct SharedState {
    // Typed capability and contact state are one coherent snapshot; never
    // combine a model from one publication with availability from another.
    std::mutex typed_mutex;
    onrobot_gripper_msgs::msg::GripperState typed;
    std::int64_t typed_received_ns{0};
    std::atomic<bool> has_state{false};
    std::atomic<bool> realtime_active{false};
    std::atomic<bool> faulted{false};
    std::atomic<bool> mechanism_linear_position_valid{false};
    std::atomic<double> mechanism_linear_position{0.0};
    std::atomic<bool> mechanism_linear_velocity_valid{false};
    std::atomic<double> mechanism_linear_velocity{0.0};
    std::atomic<bool> mechanism_angular_position_valid{false};
    std::atomic<double> mechanism_angular_position{0.0};
    std::atomic<bool> mechanism_angular_velocity_valid{false};
    std::atomic<double> mechanism_angular_velocity{0.0};
    std::atomic<bool> task_position_valid{false};
    std::atomic<double> task_position{0.0};
    std::atomic<bool> task_velocity_valid{false};
    std::atomic<double> task_velocity{0.0};
    std::atomic<bool> force_valid{false};
    std::atomic<double> force{0.0};
    std::atomic<std::uint64_t> successful_cycles{0};
    std::atomic<std::uint64_t> failed_cycles{0};
    std::atomic<std::uint64_t> missed_deadlines{0};
    std::atomic<std::int64_t> last_state_ns{0};
    std::atomic<double> minimum_aperture_m{0.0};
    std::atomic<double> maximum_aperture_m{0.0};
    std::atomic<bool> has_limits{false};
    std::atomic<std::int64_t> limits_received_ns{0};
    std::atomic<bool> safety_status_valid{false};
    std::atomic<bool> safety_1_pushed{false};
    std::atomic<bool> safety_1_triggered{false};
    std::atomic<bool> safety_2_pushed{false};
    std::atomic<bool> safety_2_triggered{false};
    std::atomic<bool> safety_dc_error{false};
  };

  void beginPositionCommand(double i_targetPositionM);
  void publishPositionCommand();
  void publishJoystickCommand();
  bool forceCommandReady(int *o_model = nullptr) const;
  void beginForceCommand();
  void publishForceCommand();
  void releaseForceGrip();
  void updateForceControls();
  void stopRealtime(const char *i_reason);
  void setCommandWidgetsEnabled(bool i_enabled);
  void updateStatus();
  void setStatus(const QString &i_text);

  rclcpp::Node::SharedPtr m_node;
  rclcpp::Publisher<RealtimeCommand>::SharedPtr m_commandPublisher;
  rclcpp::Subscription<RealtimeState>::SharedPtr m_stateSubscription;
  rclcpp::Subscription<control_msgs::msg::Float64Values>::SharedPtr
      m_limitSubscription;
  rclcpp::Subscription<onrobot_gripper_msgs::msg::GripperState>::SharedPtr
      m_gripperStateSubscription;
  std::shared_ptr<SharedState> m_sharedState;
  QTimer *m_commandTimer{nullptr};
  QTimer *m_stateTimer{nullptr};

  QDoubleSpinBox *m_targetPositionSpinBox{nullptr};
  QDoubleSpinBox *m_positionForceSpinBox{nullptr};
  QLabel *m_positionForceCaption{nullptr};
  QPushButton *m_openButton{nullptr};
  QPushButton *m_closeButton{nullptr};
  QPushButton *m_sendPositionButton{nullptr};
  QLabel *m_endpointLabel{nullptr};
  QWidget *m_forceGroup{nullptr};
  QComboBox *m_forceApproach{nullptr};
  QDoubleSpinBox *m_gripForceSpinBox{nullptr};
  QPushButton *m_holdGripButton{nullptr};
  QPushButton *m_releaseGripButton{nullptr};
  QLabel *m_gripStateLabel{nullptr};
  QLabel *m_forceHint{nullptr};
  bool m_forceCommandActive{false};
  int m_forceModel{0};
  RealtimeCommand m_forceCommand;
  QSlider *m_joystickSlider{nullptr};
  QDoubleSpinBox *m_maxVelocitySpinBox{nullptr};
  QLabel *m_directionLabel{nullptr};
  QLabel *m_measuredLabel{nullptr};
  QLabel *m_taskPositionValue{nullptr};
  QLabel *m_taskVelocityValue{nullptr};
  QLabel *m_mechanismPositionValue{nullptr};
  QLabel *m_mechanismPositionUnit{nullptr};
  QLabel *m_mechanismVelocityValue{nullptr};
  QLabel *m_mechanismVelocityUnit{nullptr};
  QLabel *m_forceCaption{nullptr};
  QLabel *m_forceValue{nullptr};
  QLabel *m_forceUnit{nullptr};
  QLabel *m_statusLabel{nullptr};
  QLabel *m_safetyLabel{nullptr};
  QLabel *m_safetyCaption{nullptr};
  std::vector<QWidget *> m_commandWidgets;
  bool m_dragging{false};
  bool m_positionCommandActive{false};
  bool m_stopRequested{false};
  bool m_rgCoordinateProfile{false};
  bool m_limitsApplied{false};
  bool m_commandWidgetsEnabled{false};
  bool m_directionGuardStopped{false};
  mutable PanelSettings m_panelSettings;
  double m_savedTargetM{std::numeric_limits<double>::quiet_NaN()};
  double m_savedMaximumVelocity{std::numeric_limits<double>::quiet_NaN()};
  double m_savedPositionForce{std::numeric_limits<double>::quiet_NaN()};
  // Relative defaults follow the RViz node namespace.  Absolute names remain
  // available through the panel-local settings for shared RViz processes.
  std::string m_commandTopic{"realtime_controller/command"};
  std::string m_stateTopic{"realtime_controller/state"};
  std::string m_limitsTopic{"parallel_gripper_limit_broadcaster/values"};
  std::string m_gripperStateTopic{"gripper_state_broadcaster/state"};
  std::int64_t m_positionCommandStartedNs{0};
  RealtimeDirectionGuard m_directionGuard;
  RealtimePositionTargetTracker m_positionTracker;
};

} // namespace onrobot_gripper_rviz_plugins

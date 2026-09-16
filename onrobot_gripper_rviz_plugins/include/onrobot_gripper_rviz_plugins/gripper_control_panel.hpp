#pragma once

#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <QString>
#include <QWidget>

#include <control_msgs/action/parallel_gripper_command.hpp>
#include <control_msgs/msg/float64_values.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <onrobot_gripper_msgs/msg/gripper_state.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <rviz_common/panel.hpp>

#include "onrobot_gripper_rviz_plugins/panel_settings.hpp"

class QLabel;
class QDoubleSpinBox;
class QSpinBox;
class QPushButton;
class QGroupBox;
class QTimer;

namespace onrobot_gripper_rviz_plugins {

/// @brief Format measured gripper state without presenting invalid force data.
QString formatMeasuredGripperState(double i_positionM, double i_effortN);

/// @brief Compact RViz panel for showcasing the standard gripper action.
///
/// The panel deliberately speaks only the standard
/// control_msgs/action/ParallelGripperCommand action and subscribes to the
/// joint-state broadcaster. It does not access Modbus or the driver internals,
/// so the same visual demo can be used with physical, fake, or supported
/// simulation backends.
class GripperControlPanel final : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit GripperControlPanel(QWidget *i_parent = nullptr);
  ~GripperControlPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config &i_config) override;
  void save(rviz_common::Config i_config) const override;

private:
  using GripperAction = control_msgs::action::ParallelGripperCommand;
  using GoalHandle = rclcpp_action::ClientGoalHandle<GripperAction>;
  using GetParameters = rcl_interfaces::srv::GetParameters;
  using SetParametersAtomically = rcl_interfaces::srv::SetParametersAtomically;
  using GetParametersClient = rclcpp::Client<GetParameters>;
  using SetParametersClient = rclcpp::Client<SetParametersAtomically>;

  // ROS callbacks execute on RViz's executor thread while the panel widgets
  // belong to the Qt GUI thread.  Keeping the latest sample in this small
  // callback-owned state object lets the GUI poll it safely, and means that a
  // late ROS callback never has to dereference a QWidget during RViz shutdown.
  struct SharedState {
    std::atomic<double> measured_position{0.0};
    std::atomic<double> measured_effort{
        std::numeric_limits<double>::quiet_NaN()};
    std::atomic<bool> has_measurement{false};
    std::atomic<std::int64_t> last_measurement_ns{0};
    std::atomic<bool> standard_action_ready{false};
    std::atomic<double> minimum_aperture_m{0.0};
    std::atomic<double> maximum_aperture_m{0.0};
    std::atomic<bool> has_limits{false};
    std::atomic<bool> safety_status_valid{false};
    std::atomic<bool> safety_1_pushed{false};
    std::atomic<bool> safety_1_triggered{false};
    std::atomic<bool> safety_2_pushed{false};
    std::atomic<bool> safety_2_triggered{false};
    std::atomic<bool> safety_dc_error{false};
    std::atomic<bool> speed_capability_checked{false};
    std::atomic<bool> speed_control_available{false};
    std::atomic<bool> speed_current_valid{false};
    std::atomic<bool> speed_read_in_flight{false};
    std::atomic<bool> speed_write_pending{false};
    std::atomic<bool> speed_set_acknowledged{false};
    std::atomic<bool> speed_input_dirty{false};
    std::atomic<int> speed_percent{50};
    std::atomic<int> speed_expected_percent{-1};
    std::atomic<int> speed_status_code{0};
    std::atomic<std::uint64_t> speed_request_generation{0};
    std::atomic<std::uint64_t> speed_event_generation{0};
    std::atomic<std::int64_t> speed_last_read_ns{0};
    std::atomic<std::int64_t> speed_read_started_ns{0};
    std::atomic<std::int64_t> speed_write_started_ns{0};
    mutable std::mutex speed_status_mutex;
    std::string speed_status_message{"Checking controller speed support"};
    // 0 idle, 1 accepted, 2 feedback/moving, 3 succeeded, 4 canceled,
    // 5 aborted, 6 rejected.  ROS callbacks write this; Qt reads it.
    std::atomic<int> action_status{0};
  };

  void sendGoal(double i_positionM);
  bool hasFreshMeasurement() const;
  void updateMeasuredState(double i_positionM, double i_effortN);
  void setCommandWidgetsEnabled(bool i_enabled);
  void setStatus(const QString &i_text);
  void requestSpeedParameters();
  void applySpeedSetting();
  void setSpeedStatus(int i_code, const std::string &i_message);
  static void setSpeedStatus(const std::shared_ptr<SharedState> &i_state,
                             int i_code, const std::string &i_message);
  void updateSpeedPanel(bool i_command_ready);
  static void requestSpeedReadback(
      const GetParametersClient::SharedPtr &i_client,
      const std::shared_ptr<SharedState> &i_state,
      std::uint64_t i_generation, std::uint64_t i_event_generation,
      int i_expected_percent);

  rclcpp::Node::SharedPtr m_node;
  rclcpp_action::Client<GripperAction>::SharedPtr m_actionClient;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr
      m_controllerListClient;
  GetParametersClient::SharedPtr m_speedGetParametersClient;
  SetParametersClient::SharedPtr m_speedSetParametersClient;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
      m_parameterEventSubscription;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      m_jointStateSubscription;
  rclcpp::Subscription<control_msgs::msg::Float64Values>::SharedPtr
      m_limitSubscription;
  rclcpp::Subscription<onrobot_gripper_msgs::msg::GripperState>::SharedPtr
      m_gripperStateSubscription;
  std::shared_ptr<SharedState> m_sharedState;
  QTimer *m_stateTimer{nullptr};
  double m_jointLowerM{0.0};
  double m_jointUpperM{0.019};
  double m_minimumEffortN{0.0};
  double m_maximumEffortN{95.0};
  double m_defaultEffortN{10.0};
  std::string m_jointName{"grip_stroke"};
  // Relative defaults follow the RViz node namespace.  Absolute names remain
  // available through the panel-local settings for shared RViz processes.
  std::string m_actionName{"gripper_controller/gripper_cmd"};
  // Control observes the gripper broadcaster, not the workcell's aggregated
  // visualization topic, which may contain only another robot's joints.
  std::string m_jointStatesTopic{"joint_state_broadcaster/joint_states"};
  std::string m_limitsTopic{"parallel_gripper_limit_broadcaster/values"};
  std::string m_gripperStateTopic{"gripper_state_broadcaster/state"};
  std::string m_controllerManagerService{
      "controller_manager/list_controllers"};
  std::string m_controllerParameterNode;
  std::vector<QWidget *> m_commandWidgets;
  bool m_commandWidgetsEnabled{false};
  mutable PanelSettings m_panelSettings;
  double m_savedTargetM{std::numeric_limits<double>::quiet_NaN()};
  double m_savedEffortN{std::numeric_limits<double>::quiet_NaN()};

  QDoubleSpinBox *m_targetSpinBox{nullptr};
  QDoubleSpinBox *m_effortSpinBox{nullptr};
  QLabel *m_measuredLabel{nullptr};
  QLabel *m_measuredPositionValue{nullptr};
  QLabel *m_measuredEffortCaption{nullptr};
  QLabel *m_measuredEffortValue{nullptr};
  QLabel *m_measuredEffortUnit{nullptr};
  QLabel *m_statusLabel{nullptr};
  QLabel *m_safetyLabel{nullptr};
  QLabel *m_safetyCaption{nullptr};
  QGroupBox *m_speedGroup{nullptr};
  QSpinBox *m_speedSpinBox{nullptr};
  QPushButton *m_speedApplyButton{nullptr};
  QLabel *m_speedCurrentLabel{nullptr};
  QLabel *m_speedStatusLabel{nullptr};
};

} // namespace onrobot_gripper_rviz_plugins

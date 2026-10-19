#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <QString>
#include <onrobot_gripper_msgs/msg/gripper_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "onrobot_gripper_rviz_plugins/force_history_model.hpp"
#include "onrobot_gripper_rviz_plugins/panel_settings.hpp"

class QLabel;
class QPushButton;
class QTimer;
class QWidget;

namespace onrobot_gripper_rviz_plugins {

/// Read-only force and state history, live or replayed from a rosbag.
///
/// The plot reads only the GripperState topic. It does not send commands, so
/// replaying state in this panel cannot move a gripper. Force targets are not
/// displayed as force measurements.
class ForceHistoryPanel final : public rviz_common::Panel {
  Q_OBJECT

public:
  /// @cond INTERNAL
  explicit ForceHistoryPanel(QWidget *i_parent = nullptr);
  ~ForceHistoryPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config &i_config) override;
  void save(rviz_common::Config i_config) const override;

private:
  class Plot;

  struct SharedState {
    std::mutex mutex;
    std::vector<ForceHistorySample> pending;
    std::int64_t last_received_steady_ns{0};
    std::string model;
  };

  void bindRos();
  void placeInRightDock();
  void updatePlot();
  void setStatus(const QString &i_text);

  rclcpp::Node::SharedPtr m_node;
  rclcpp::Subscription<onrobot_gripper_msgs::msg::GripperState>::SharedPtr
      m_subscription;
  std::shared_ptr<SharedState> m_sharedState;
  std::unique_ptr<ForceHistoryBuffer> m_history;
  QTimer *m_timer{nullptr};
  Plot *m_plot{nullptr};
  QLabel *m_statusLabel{nullptr};
  QLabel *m_legendLabel{nullptr};
  QPushButton *m_clearButton{nullptr};
  // Relative defaults follow the RViz node namespace.  An absolute topic can
  // still be supplied in the panel-local settings for a shared RViz process.
  std::string m_gripperStateTopic{"gripper_state_broadcaster/state"};
  double m_maxSampleAgeS{0.25};
  mutable PanelSettings m_panelSettings;
  /// @endcond
};

} // namespace onrobot_gripper_rviz_plugins

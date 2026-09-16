#include "onrobot_gripper_rviz_plugins/three_finger_control_panel.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QSizePolicy>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include "onrobot_gripper_rviz_plugins/branded_panel.hpp"

namespace onrobot_gripper_rviz_plugins {
namespace {

std::int64_t steadyNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

QString connectionStateText(uint8_t state, uint16_t faultCode) {
  using State = onrobot_gripper_msgs::msg::GripperState;
  switch (state) {
  case State::CONNECTION_DISCONNECTED:
    return QStringLiteral("DISCONNECTED");
  case State::CONNECTION_CONNECTING:
    return QStringLiteral("CONNECTING");
  case State::CONNECTION_IDLE:
    return QStringLiteral("Connected: idle");
  case State::CONNECTION_ACTIVE:
    return QStringLiteral("Connected: active");
  case State::CONNECTION_RECOVERING:
    return QStringLiteral("RECOVERING");
  case State::CONNECTION_FAULTED:
    return faultCode == 0
               ? QStringLiteral("FAULTED")
               : QStringLiteral("FAULTED (code %1)")
                     .arg(static_cast<unsigned int>(faultCode));
  default:
    return QStringLiteral("Connection state unknown");
  }
}

} // namespace

ThreeFingerControlPanel::ThreeFingerControlPanel(QWidget *parent)
    : rviz_common::Panel(parent),
      shared_state_(std::make_shared<SharedState>()) {
  setObjectName("OnRobotThreeFingerControlPanel");
  applyOnRobotPanelStyle(this);

  auto *layout = new QVBoxLayout(this);
  layout->addWidget(createOnRobotPanelHeader(
      this, "Three-finger gripper control",
      "Move to an external diameter through ros2_control"));

  auto *command_group = new QGroupBox("External diameter command", this);
  auto *command_layout = new QFormLayout(command_group);
  command_layout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
  command_layout->setRowWrapPolicy(QFormLayout::WrapLongRows);
  target_spin_box_ = new QDoubleSpinBox(command_group);
  target_spin_box_->setDecimals(1);
  target_spin_box_->setSingleStep(1.0);
  target_spin_box_->setSuffix(" mm");
  target_spin_box_->setToolTip(
      "Requested outside diameter; sent through ros2_control in metres");
  command_layout->addRow("Target:", target_spin_box_);
  force_label_ = new QLabel(command_group);
  force_label_->setObjectName("threeFingerForce");
  force_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  force_label_->setMinimumWidth(180);
  force_label_->setWordWrap(true);
  force_label_->setToolTip("Configured hardware force percentage. This panel sends "
                          "diameter commands; it does not adjust force.");
  command_layout->addRow("Force:", force_label_);
  layout->addWidget(command_group);

  auto *button_layout = new QGridLayout();
  auto *minimum_button = new QPushButton("Close / Minimum", this);
  auto *demo_button = new QPushButton("70 mm", this);
  auto *maximum_button = new QPushButton("Open / Maximum", this);
  auto *send_button = new QPushButton("Send target", this);
  button_layout->addWidget(minimum_button, 0, 0);
  button_layout->addWidget(maximum_button, 0, 1);
  button_layout->addWidget(demo_button, 1, 0);
  button_layout->addWidget(send_button, 1, 1);
  layout->addLayout(button_layout);
  command_widgets_ = {target_spin_box_, minimum_button, demo_button,
                      maximum_button, send_button};
  for (auto *widget : command_widgets_) {
    widget->setEnabled(false);
  }

  auto *state_group = new QGroupBox("Live physical state", this);
  auto *state_layout = new QFormLayout(state_group);
  // Desktop styles differ: FieldsStayAtSizeHint collapses Ignored labels to
  // zero width. Explicit growth and nonzero hints also give wrapping its height.
  state_layout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
  state_layout->setRowWrapPolicy(QFormLayout::WrapLongRows);
  measured_label_ = new QLabel("waiting for /joint_states", state_group);
  status_label_ = new QLabel("RViz panel is starting", state_group);
  measured_label_->setObjectName("threeFingerMeasured");
  status_label_->setObjectName("threeFingerConnection");
  measured_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  status_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  measured_label_->setMinimumWidth(180);
  status_label_->setMinimumWidth(180);
  measured_label_->setWordWrap(true);
  status_label_->setWordWrap(true);
  state_layout->addRow("Measured:", measured_label_);
  state_layout->addRow("Connection:", status_label_);
  layout->addWidget(state_group);
  layout->addStretch(1);

  connect(send_button, &QPushButton::clicked, this,
          [this]() { publishDiameter(target_spin_box_->value() / 1000.0); });
  connect(minimum_button, &QPushButton::clicked, this, [this]() {
    target_spin_box_->setValue(minimum_diameter_m_ * 1000.0);
    publishDiameter(minimum_diameter_m_);
  });
  connect(demo_button, &QPushButton::clicked, this, [this]() {
    const double demo_diameter_m =
        std::clamp(0.070, minimum_diameter_m_, maximum_diameter_m_);
    target_spin_box_->setValue(demo_diameter_m * 1000.0);
    publishDiameter(demo_diameter_m);
  });
  connect(maximum_button, &QPushButton::clicked, this, [this]() {
    target_spin_box_->setValue(maximum_diameter_m_ * 1000.0);
    publishDiameter(maximum_diameter_m_);
  });

  state_timer_ = new QTimer(this);
  state_timer_->setInterval(100);
  connect(state_timer_, &QTimer::timeout, this,
          [this]() { updateLiveState(); });
  state_timer_->start();
}

ThreeFingerControlPanel::~ThreeFingerControlPanel() {
  if (state_timer_) {
    state_timer_->stop();
  }
  joint_state_subscription_.reset();
  limit_subscription_.reset();
  gripper_state_subscription_.reset();
  command_publisher_.reset();
}

void ThreeFingerControlPanel::readSettings() {
  if (!node_) {
    return;
  }
  auto readString = [this](const char *key, std::string &value) {
    if (!panel_settings_.readString(QString::fromUtf8(key), value)) {
      value = readOrDeclare(node_, key, value);
    }
  };
  readString("three_finger_command_topic", command_topic_);
  readString("joint_states_topic", joint_states_topic_);
  readString("limits_topic", limits_topic_);
  readString("gripper_state_topic", gripper_state_topic_);
  if (!panel_settings_.readDouble("three_finger_default_force_percent",
                                  configured_force_percent_)) {
    configured_force_percent_ = readOrDeclare(
        node_, "three_finger_default_force_percent", configured_force_percent_);
  }
  if (!std::isfinite(configured_force_percent_) ||
      configured_force_percent_ < 1.0 || configured_force_percent_ > 100.0) {
    configured_force_percent_ = 20.0;
  }
}

void ThreeFingerControlPanel::onInitialize() {
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    setStatus("DISCONNECTED: RViz ROS node is unavailable");
    return;
  }

  node_ = abstraction->get_raw_node();
  shared_state_ = std::make_shared<SharedState>();
  command_ready_ = false;
  for (auto *widget : command_widgets_) widget->setEnabled(false);
  measured_label_->setText("Waiting for measured diameter");
  readSettings();
  force_label_->setText(
      QString("%1% (hardware default)")
          .arg(configured_force_percent_, 0, 'f', 0));

  command_publisher_ =
      node_->create_publisher<std_msgs::msg::Float64MultiArray>(
          command_topic_, rclcpp::QoS(1).reliable());
  const auto state = shared_state_;
  joint_state_subscription_ =
      node_->create_subscription<sensor_msgs::msg::JointState>(
          joint_states_topic_, rclcpp::SensorDataQoS(),
          [state](const sensor_msgs::msg::JointState::SharedPtr message) {
            for (size_t index = 0; index < message->name.size() &&
                                   index < message->position.size();
                 ++index) {
              const auto &name = message->name[index];
              if (name == "grip_diameter") {
                state->diameter_m.store(message->position[index],
                                        std::memory_order_relaxed);
                state->diameter_velocity_m_s.store(
                    index < message->velocity.size() ? message->velocity[index]
                        : std::numeric_limits<double>::quiet_NaN(),
                    std::memory_order_relaxed);
                state->last_measurement_ns.store(steadyNowNs(),
                                                 std::memory_order_release);
                state->has_diameter.store(std::isfinite(message->position[index]),
                                         std::memory_order_release);
              } else if (name == "finger_angle") {
                state->finger_angle_rad.store(message->position[index],
                                              std::memory_order_relaxed);
                state->finger_angle_velocity_rad_s.store(
                    index < message->velocity.size() ? message->velocity[index]
                        : std::numeric_limits<double>::quiet_NaN(),
                    std::memory_order_relaxed);
                state->last_angle_ns.store(steadyNowNs(), std::memory_order_release);
                state->has_angle.store(std::isfinite(message->position[index]),
                                      std::memory_order_release);
              }
            }
          });
  limit_subscription_ =
      node_->create_subscription<control_msgs::msg::Float64Values>(
          limits_topic_, rclcpp::SensorDataQoS(),
          [state](const control_msgs::msg::Float64Values::SharedPtr message) {
            if (message->values.size() != 2) {
              return;
            }
            const double minimum = message->values[0];
            const double maximum = message->values[1];
            if (!std::isfinite(minimum) || !std::isfinite(maximum) ||
                minimum < 0.0 || maximum <= minimum) {
              return;
            }
            state->minimum_diameter_m.store(minimum, std::memory_order_relaxed);
            state->maximum_diameter_m.store(maximum, std::memory_order_relaxed);
            state->has_limits.store(true, std::memory_order_release);
          });
  gripper_state_subscription_ =
      node_->create_subscription<onrobot_gripper_msgs::msg::GripperState>(
          gripper_state_topic_, rclcpp::SensorDataQoS(),
          [state](const onrobot_gripper_msgs::msg::GripperState::SharedPtr
                      message) {
            const double age =
                static_cast<double>(message->sample_age.sec) +
                static_cast<double>(message->sample_age.nanosec) * 1e-9;
            state->connection_state.store(message->connection_state,
                                          std::memory_order_relaxed);
            state->fault_code.store(message->fault_code,
                                    std::memory_order_relaxed);
            state->task_position_valid.store(message->task_aperture_valid,
                                             std::memory_order_relaxed);
            state->gripper_state_age_s.store(
                age >= 0.0 ? age : std::numeric_limits<double>::quiet_NaN(),
                std::memory_order_relaxed);
            state->last_gripper_state_ns.store(
                steadyNowNs(), std::memory_order_release);
            state->has_gripper_state.store(true, std::memory_order_release);
          });
  setStatus("Waiting for live gripper limits and physical state");
}

void ThreeFingerControlPanel::publishDiameter(double diameter_m) {
  updateLiveState();
  if (!command_ready_ || !std::isfinite(diameter_m)) return;
  const double bounded =
      std::clamp(diameter_m, minimum_diameter_m_, maximum_diameter_m_);
  std_msgs::msg::Float64MultiArray command;
  command.data = {bounded};
  command_publisher_->publish(command);
  setStatus(QString("Sent %1 mm through ros2_control")
                .arg(bounded * 1000.0, 0, 'f', 1));
}

void ThreeFingerControlPanel::updateLiveState() {
  command_ready_ = false;
  // Disabled below whenever feedback is invalid, stale, or disconnected.
  // Only change enabled state when needed, so held clicks keep their focus.
  auto enableCommands = [this](bool enabled) {
    for (auto *widget : command_widgets_) {
      if (widget->isEnabled() != enabled) widget->setEnabled(enabled);
    }
  };
  const bool has_limits = shared_state_ && shared_state_->has_limits.load(
                                               std::memory_order_acquire);
  if (has_limits) {
    const double minimum =
        shared_state_->minimum_diameter_m.load(std::memory_order_relaxed);
    const double maximum =
        shared_state_->maximum_diameter_m.load(std::memory_order_relaxed);
    if (std::isfinite(minimum) && std::isfinite(maximum) && minimum >= 0.0 &&
        maximum > minimum) {
      if (minimum != minimum_diameter_m_ || maximum != maximum_diameter_m_) {
        minimum_diameter_m_ = minimum;
        maximum_diameter_m_ = maximum;
        target_spin_box_->setRange(minimum * 1000.0, maximum * 1000.0);
        target_spin_box_->setValue(std::clamp(
            target_spin_box_->value(), minimum * 1000.0, maximum * 1000.0));
      }
    }
  }
  if (!shared_state_ ||
      !shared_state_->has_diameter.load(std::memory_order_acquire)) {
    enableCommands(false);
    measured_label_->setText("Diameter unavailable");
    setStatus("Waiting for valid measured diameter");
    return;
  }
  if (!has_limits) {
    enableCommands(false);
    setStatus("State online; waiting for live gripper limits");
    return;
  }

  const auto measurement_age_ms =
      (steadyNowNs() - shared_state_->last_measurement_ns.load(
                           std::memory_order_acquire)) /
      1000000;
  if (measurement_age_ms > 1500) {
    enableCommands(false);
    measured_label_->setText("Diameter stale");
    setStatus(QString("CONNECTION LOST: state is %1 ms old")
                  .arg(static_cast<qlonglong>(measurement_age_ms)));
    return;
  }

  const auto velocity = shared_state_->diameter_velocity_m_s.load();
  QString measured =
      QString("diameter %1 mm\nvelocity %2")
          .arg(shared_state_->diameter_m.load(std::memory_order_relaxed) *
                   1000.0,
               0, 'f', 1)
          .arg(std::isfinite(velocity)
                   ? QString("%1 mm/s").arg(velocity * 1000.0, 0, 'f', 1)
                   : QStringLiteral("unavailable"));
  if (shared_state_->has_angle.load(std::memory_order_acquire) &&
      steadyNowNs() - shared_state_->last_angle_ns.load() <= 1500000000LL) {
    measured +=
        QString("\nURDF finger joint %1 rad")
            .arg(shared_state_->finger_angle_rad.load(
                     std::memory_order_relaxed),
                 0, 'f', 3);
  } else {
    measured += QStringLiteral("\nURDF finger joint unavailable");
  }
  measured_label_->setText(measured);

  if (!shared_state_->has_gripper_state.load(std::memory_order_acquire)) {
    enableCommands(false);
    setStatus("State online; waiting for semantic gripper state");
    return;
  }
  const auto state_age_ms =
      (steadyNowNs() - shared_state_->last_gripper_state_ns.load(
                           std::memory_order_acquire)) /
      1000000;
  const double device_age_s =
      shared_state_->gripper_state_age_s.load(std::memory_order_relaxed);
  if (!std::isfinite(device_age_s) || device_age_s < 0.0 ||
      device_age_s + state_age_ms * 0.001 > 1.5) {
    enableCommands(false);
    measured_label_->setText("Device state stale; measurement unavailable");
    const QString age = std::isfinite(device_age_s) && device_age_s >= 0.0
                            ? QString::number(device_age_s * 1000.0 + state_age_ms, 'f', 0)
                            : QStringLiteral("unknown");
    setStatus(QString("CONNECTION LOST: device state age %1 ms").arg(age));
    return;
  }

  QString status = connectionStateText(
      shared_state_->connection_state.load(std::memory_order_relaxed),
      shared_state_->fault_code.load(std::memory_order_relaxed));
  if (!command_publisher_ || command_publisher_->get_subscription_count() == 0) {
    status += QStringLiteral("; waiting for diameter_controller");
  } else {
    using State = onrobot_gripper_msgs::msg::GripperState;
    const auto connection = shared_state_->connection_state.load();
    command_ready_ = shared_state_->task_position_valid.load() &&
        (connection == State::CONNECTION_IDLE || connection == State::CONNECTION_ACTIVE);
  }
  if (!shared_state_->task_position_valid.load()) {
    measured_label_->setText("Device diameter unavailable");
  }
  enableCommands(command_ready_);
  setStatus(status);
}

void ThreeFingerControlPanel::setStatus(const QString &text) {
  if (!status_label_) {
    return;
  }
  status_label_->setText(text);
  if (text.startsWith("DISCONNECTED") || text.startsWith("CONNECTION LOST")) {
    status_label_->setStyleSheet(
        "QLabel { color: #d32f2f; font-weight: bold; }");
  } else if (text.startsWith("Connected")) {
    status_label_->setStyleSheet(
        "QLabel { color: #2e7d32; font-weight: bold; }");
  } else if (text.startsWith("CONNECTING") ||
             text.startsWith("RECOVERING") ||
             text.startsWith("Connection state unknown") ||
             text.startsWith("State online") || text.startsWith("Waiting")) {
    status_label_->setStyleSheet(
        "QLabel { color: #ef6c00; font-weight: bold; }");
  } else if (text.startsWith("FAULTED")) {
    status_label_->setStyleSheet(
        "QLabel { color: #d32f2f; font-weight: bold; }");
  } else {
    status_label_->setStyleSheet(QString());
  }
}

void ThreeFingerControlPanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
  panel_settings_.load(config);
  panel_settings_.readString("three_finger_command_topic", command_topic_);
  panel_settings_.readString("joint_states_topic", joint_states_topic_);
  panel_settings_.readString("limits_topic", limits_topic_);
  panel_settings_.readString("gripper_state_topic", gripper_state_topic_);
  panel_settings_.readDouble("three_finger_default_force_percent",
                             configured_force_percent_);
  if (node_) {
    onInitialize();
  }
}

void ThreeFingerControlPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  panel_settings_.writeString("three_finger_command_topic", command_topic_);
  panel_settings_.writeString("joint_states_topic", joint_states_topic_);
  panel_settings_.writeString("limits_topic", limits_topic_);
  panel_settings_.writeString("gripper_state_topic", gripper_state_topic_);
  panel_settings_.writeDouble("three_finger_default_force_percent",
                              configured_force_percent_);
  panel_settings_.save(config);
}

} // namespace onrobot_gripper_rviz_plugins

PLUGINLIB_EXPORT_CLASS(onrobot_gripper_rviz_plugins::ThreeFingerControlPanel,
                       rviz_common::Panel)

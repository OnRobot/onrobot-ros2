#include "onrobot_gripper_rviz_plugins/gripper_control_panel.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSizePolicy>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rclcpp/expand_topic_or_service_name.hpp>
#include <rclcpp/qos.hpp>

#include "onrobot_gripper_rviz_plugins/branded_panel.hpp"
#include "onrobot_gripper_rviz_plugins/safety_panel_semantics.hpp"

namespace onrobot_gripper_rviz_plugins {
namespace {

void updateWrappedLabelHeight(QLabel *label) {
  const int width = std::max(label->width(), label->minimumWidth());
  const int height = label->heightForWidth(width);
  if (height > 0) {
    label->setMinimumHeight(height);
  }
  label->updateGeometry();
}

constexpr int kActionAccepted = 1;
constexpr int kActionMoving = 2;
constexpr int kActionSucceeded = 3;
constexpr int kActionCanceled = 4;
constexpr int kActionAborted = 5;
constexpr int kActionRejected = 6;
constexpr std::int64_t kSpeedReadTimeoutNs = 3000000000LL;
constexpr std::int64_t kSpeedPollIntervalNs = 2000000000LL;

std::int64_t steadyNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::string controllerNodeFromActionName(const rclcpp::Node::SharedPtr &node,
                                         const std::string &action_name) {
  const auto expanded = rclcpp::expand_topic_or_service_name(
      action_name, node->get_name(), node->get_namespace(), false);
  const auto separator = expanded.rfind('/');
  if (separator == std::string::npos) {
    return {};
  }
  const auto controller = expanded.substr(0, separator);
  return controller.empty() ? "/" : controller;
}

std::string serviceForController(const std::string &controller_node,
                                 const char *service) {
  return (controller_node == "/" ? std::string{} : controller_node) + "/" +
         service;
}

} // namespace

QString formatMeasuredGripperState(double i_positionM, double i_effortN) {
  const QString position = QString("%1 m").arg(i_positionM, 0, 'f', 4);
  if (!std::isfinite(i_effortN)) {
    return position;
  }
  return position + QString(", %1 N").arg(i_effortN, 0, 'f', 1);
}

GripperControlPanel::GripperControlPanel(QWidget *i_parent)
    : rviz_common::Panel(i_parent) {
  setObjectName("OnRobotGripperControlPanel");
  applyOnRobotPanelStyle(this);
  m_sharedState = std::make_shared<SharedState>();

  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(4, 4, 4, 4);
  layout->setSpacing(3);
  layout->addWidget(createOnRobotPanelHeader(
      this, "Parallel gripper control",
      "Open, close, or set the aperture", 95));

  auto *targetGroup = new QGroupBox("Command", this);
  targetGroup->setObjectName("gripperCommandGroup");
  auto *targetLayout = new QFormLayout(targetGroup);
  targetLayout->setVerticalSpacing(4);

  m_targetSpinBox = new QDoubleSpinBox(targetGroup);
  m_targetSpinBox->setObjectName("conventionalTargetAperture");
  m_targetSpinBox->setRange(m_jointLowerM, m_jointUpperM);
  m_targetSpinBox->setDecimals(4);
  m_targetSpinBox->setSingleStep(0.001);
  m_targetSpinBox->setValue(m_jointUpperM);
  m_targetSpinBox->setSuffix(" m");
  m_targetSpinBox->setToolTip("Target task aperture in metres");
  targetLayout->addRow("Target aperture:", m_targetSpinBox);

  m_effortSpinBox = new QDoubleSpinBox(targetGroup);
  m_effortSpinBox->setObjectName("conventionalEffort");
  m_effortSpinBox->setRange(0.0, m_maximumEffortN);
  m_effortSpinBox->setDecimals(1);
  m_effortSpinBox->setValue(10.0);
  m_effortSpinBox->setSuffix(" N");
  m_effortSpinBox->setToolTip(
      "Force limit sent through the standard action effort field");
  targetLayout->addRow("Force limit:", m_effortSpinBox);
  layout->addWidget(targetGroup);

  auto *buttonLayout = new QHBoxLayout();
  auto *openButton = new QPushButton("Open", this);
  auto *closeButton = new QPushButton("Close", this);
  auto *sendButton = new QPushButton("Send target", this);
  buttonLayout->addWidget(openButton);
  buttonLayout->addWidget(closeButton);
  buttonLayout->addWidget(sendButton);
  layout->addLayout(buttonLayout);
  m_commandWidgets = {targetGroup, openButton, closeButton, sendButton};
  for (auto *widget : m_commandWidgets) {
    widget->setEnabled(false);
  }

  m_speedGroup = new QGroupBox("Conventional speed", this);
  m_speedGroup->setObjectName("conventionalSpeedGroup");
  m_speedGroup->setVisible(false);
  auto *speedLayout = new QVBoxLayout(m_speedGroup);
  auto *speedInputLayout = new QHBoxLayout();
  speedInputLayout->addWidget(new QLabel("Max aperture speed:", m_speedGroup));
  m_speedSpinBox = new QDoubleSpinBox(m_speedGroup);
  m_speedSpinBox->setObjectName("conventionalSpeedMps");
  m_speedSpinBox->setDecimals(3);
  m_speedSpinBox->setRange(0.001, 0.400);
  m_speedSpinBox->setSingleStep(0.010);
  m_speedSpinBox->setSuffix(" m/s");
  m_speedSpinBox->setValue(0.100);
  m_speedSpinBox->setToolTip(
      "Optional maximum aperture speed, in m/s. The controller selects the "
      "native speed using an estimated full-travel peak at the requested force.");
  speedInputLayout->addWidget(m_speedSpinBox, 1);
  speedLayout->addLayout(speedInputLayout);
  m_speedStatusLabel = new QLabel("Used by the next Open, Close or Send target.", m_speedGroup);
  m_speedStatusLabel->setObjectName("conventionalSpeedStatus");
  m_speedStatusLabel->setWordWrap(true);
  speedLayout->addWidget(m_speedStatusLabel);
  layout->addWidget(m_speedGroup);

  auto *stateGroup = new QGroupBox("Live state", this);
  auto *stateLayout = new QVBoxLayout(stateGroup);
  stateLayout->setSpacing(2);
  auto *measuredGrid = new QGridLayout();
  measuredGrid->setColumnStretch(0, 1);
  measuredGrid->setColumnMinimumWidth(1, 74);
  m_measuredLabel = new QLabel("Waiting for joint state", stateGroup);
  m_statusLabel = new QLabel("RViz panel is starting", stateGroup);
  m_measuredLabel->setObjectName("conventionalMeasuredState");
  m_statusLabel->setObjectName("conventionalActionStatus");
  m_safetyLabel = new QLabel(stateGroup);
  m_measuredPositionValue = new QLabel(QString::fromUtf8("\u2014"), stateGroup);
  m_measuredEffortCaption = new QLabel("Force", stateGroup);
  m_measuredEffortValue = new QLabel(QString::fromUtf8("\u2014"), stateGroup);
  m_measuredEffortUnit = new QLabel("N", stateGroup);
  for (auto *value : {m_measuredPositionValue, m_measuredEffortValue}) {
    value->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    value->setMinimumWidth(74);
  }
  measuredGrid->addWidget(new QLabel("Aperture", stateGroup), 0, 0);
  measuredGrid->addWidget(m_measuredPositionValue, 0, 1);
  measuredGrid->addWidget(new QLabel("m", stateGroup), 0, 2);
  measuredGrid->addWidget(m_measuredEffortCaption, 1, 0);
  measuredGrid->addWidget(m_measuredEffortValue, 1, 1);
  measuredGrid->addWidget(m_measuredEffortUnit, 1, 2);
  measuredGrid->addWidget(m_measuredLabel, 2, 0, 1, 3);
  m_statusLabel->setSizePolicy(QSizePolicy::Preferred,
                               QSizePolicy::Preferred);
  m_measuredLabel->setMinimumWidth(180);
  m_statusLabel->setMinimumWidth(180);
  m_measuredLabel->setWordWrap(true);
  m_statusLabel->setWordWrap(true);
  m_safetyLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  m_safetyLabel->setMinimumWidth(180);
  m_safetyLabel->setObjectName("conventionalSafetyStatus");
  m_safetyLabel->setWordWrap(true);
  m_safetyLabel->setVisible(false);
  stateLayout->addLayout(measuredGrid);
  stateLayout->addWidget(m_statusLabel);
  m_safetyCaption = new QLabel("Safety:", stateGroup);
  m_safetyCaption->setVisible(false);
  auto *safetyLayout = new QHBoxLayout();
  safetyLayout->addWidget(m_safetyCaption);
  safetyLayout->addWidget(m_safetyLabel, 1);
  stateLayout->addLayout(safetyLayout);
  layout->addWidget(stateGroup);
  layout->addStretch(1);

  // Polling is intentional here: it keeps all widget access on the Qt GUI
  // thread and avoids a ROS callback retaining a QWidget during RViz exit.
  m_stateTimer = new QTimer(this);
  m_stateTimer->setInterval(100);
  connect(m_stateTimer, &QTimer::timeout, this, [this]() {
    if (!m_sharedState) {
      return;
    }
    const bool standard_ready =
        m_actionClient && m_actionClient->action_server_is_ready();
    const bool controller_manager_ready =
        m_controllerListClient && m_controllerListClient->service_is_ready();
    m_sharedState->standard_action_ready.store(standard_ready,
                                               std::memory_order_release);
    const bool has_limits =
        m_sharedState->has_limits.load(std::memory_order_acquire);
    if (has_limits) {
      const double minimum =
          m_sharedState->minimum_aperture_m.load(std::memory_order_relaxed);
      const double maximum =
          m_sharedState->maximum_aperture_m.load(std::memory_order_relaxed);
      if (std::isfinite(minimum) && std::isfinite(maximum) && minimum >= 0.0 &&
          maximum > minimum &&
          (minimum != m_jointLowerM || maximum != m_jointUpperM)) {
        m_jointLowerM = minimum;
        m_jointUpperM = maximum;
        m_targetSpinBox->setRange(minimum, maximum);
      }
    }

    const bool safety_valid =
        m_sharedState->safety_status_valid.load(std::memory_order_acquire);
    m_safetyLabel->setVisible(safety_valid);
    m_safetyCaption->setVisible(safety_valid);
    if (safety_valid) {
      const bool pushed =
          m_sharedState->safety_1_pushed.load(std::memory_order_relaxed) ||
          m_sharedState->safety_2_pushed.load(std::memory_order_relaxed);
      const bool triggered =
          m_sharedState->safety_1_triggered.load(std::memory_order_relaxed) ||
          m_sharedState->safety_2_triggered.load(std::memory_order_relaxed);
      const bool dc_error =
          m_sharedState->safety_dc_error.load(std::memory_order_relaxed);
      m_safetyLabel->setText(formatSafetyState(
          m_sharedState->safety_1_pushed.load(),
          m_sharedState->safety_1_triggered.load(),
          m_sharedState->safety_2_pushed.load(),
          m_sharedState->safety_2_triggered.load(), dc_error));
      updateWrappedLabelHeight(m_safetyLabel);
      m_safetyLabel->setStyleSheet(
          pushed || dc_error ? "QLabel { color: #d32f2f; font-weight: bold; }"
          : triggered        ? "QLabel { color: #ef6c00; font-weight: bold; }"
                             : "QLabel { color: #2e7d32; font-weight: bold; }");
    }

    const bool has_measurement =
        m_sharedState->has_measurement.load(std::memory_order_acquire);
    std::int64_t age_ms = 0;
    if (has_measurement) {
      updateMeasuredState(
          m_sharedState->measured_position.load(std::memory_order_relaxed),
          m_sharedState->measured_effort.load(std::memory_order_relaxed));
      const auto now_ns =
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now().time_since_epoch())
              .count();
      const auto sample_ns =
          m_sharedState->last_measurement_ns.load(std::memory_order_acquire);
      age_ms = sample_ns > 0 && now_ns >= sample_ns
                   ? (now_ns - sample_ns) / 1000000
                   : 0;
    }

    const bool state_fresh = hasFreshMeasurement();
    if (!state_fresh) {
      m_measuredPositionValue->setText(QString::fromUtf8("\u2014"));
      m_measuredEffortValue->setText(QString::fromUtf8("\u2014"));
      m_measuredLabel->setText(has_measurement ? "Joint state is stale"
                                             : "Waiting for valid joint state");
    }
    const bool safety_blocked =
        safety_valid &&
        (m_sharedState->safety_1_pushed.load(std::memory_order_relaxed) ||
         m_sharedState->safety_1_triggered.load(std::memory_order_relaxed) ||
         m_sharedState->safety_2_pushed.load(std::memory_order_relaxed) ||
         m_sharedState->safety_2_triggered.load(std::memory_order_relaxed) ||
         m_sharedState->safety_dc_error.load(std::memory_order_relaxed));
    const auto speed_now_ns = steadyNowNs();
    if (m_sharedState->speed_read_in_flight.load() &&
        speed_now_ns - m_sharedState->speed_read_started_ns.load() >= kSpeedReadTimeoutNs) {
      m_sharedState->speed_request_generation.fetch_add(1);
      m_sharedState->speed_read_in_flight.store(false);
    }
    if (!m_sharedState->speed_read_in_flight.load() && m_speedGetParametersClient &&
        m_speedGetParametersClient->service_is_ready() &&
        speed_now_ns - m_sharedState->speed_last_read_ns.load() >= kSpeedPollIntervalNs) {
      requestSpeedParameters();
    }
    const bool speed_ready = !m_sharedState->speed_control_available.load() ||
        speed_now_ns - m_sharedState->speed_confirmed_ns.load() <
            kSpeedPollIntervalNs + kSpeedReadTimeoutNs;
    const bool command_ready =
        state_fresh && standard_ready && has_limits && !safety_blocked && speed_ready;
    setCommandWidgetsEnabled(command_ready);
    updateSpeedPanel(command_ready);

    if (has_measurement && !state_fresh) {
      setStatus(QString("CONNECTION LOST: last state %1 ms ago")
                    .arg(static_cast<qlonglong>(age_ms)));
      return;
    }

    const int action_status =
        m_sharedState->action_status.load(std::memory_order_acquire);
    switch (action_status) {
    case kActionAccepted:
      setStatus("Goal accepted; gripper is moving");
      break;
    case kActionMoving:
      setStatus("Moving; controller feedback received");
      break;
    case kActionSucceeded:
      setStatus("Goal succeeded");
      break;
    case kActionCanceled:
      setStatus("Goal canceled");
      break;
    case kActionAborted:
      setStatus("Goal aborted");
      break;
    case kActionRejected:
      setStatus("Goal rejected by controller");
      break;
    default: {
      if (has_measurement) {
        if (standard_ready) {
          setStatus("Connected: ros2_control controller and state online");
        } else if (!has_limits) {
          setStatus("State online; waiting for live finger-profile limits");
        }
      } else if (standard_ready) {
        setStatus(QString("Controller online; waiting for %1 on %2")
                      .arg(QString::fromStdString(m_jointName),
                           m_jointStateSubscription
                               ? QString::fromUtf8(m_jointStateSubscription->get_topic_name())
                               : QString::fromStdString(m_jointStatesTopic)));
      } else if (controller_manager_ready) {
        setStatus("Controller manager online; gripper controller unavailable");
      } else {
        setStatus("DISCONNECTED: no gripper controller or state received");
      }
      break;
    }
    }
  });
  m_stateTimer->start();

  connect(sendButton, &QPushButton::clicked, this,
          [this]() { sendGoal(m_targetSpinBox->value()); });
  connect(openButton, &QPushButton::clicked, this, [this]() {
    m_targetSpinBox->setValue(m_jointUpperM);
    sendGoal(m_jointUpperM);
  });
  connect(closeButton, &QPushButton::clicked, this, [this]() {
    m_targetSpinBox->setValue(m_jointLowerM);
    sendGoal(m_jointLowerM);
  });

}

GripperControlPanel::~GripperControlPanel() {
  if (m_stateTimer) {
    m_stateTimer->stop();
  }
  m_jointStateSubscription.reset();
  m_limitSubscription.reset();
  m_gripperStateSubscription.reset();
  m_actionClient.reset();
  m_controllerListClient.reset();
  m_speedGetParametersClient.reset();
}

void GripperControlPanel::onInitialize() {
  // Disable commands synchronously at the start of a rebind.  The polling
  // timer may not run before the new namespace is attached, so an old state
  // sample must never keep these controls operable during that interval.
  setCommandWidgetsEnabled(false);
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    setStatus("RViz ROS node is unavailable");
    return;
  }

  m_node = abstraction->get_raw_node();
  // A remove/re-add or a panel Config reload starts a fresh observation
  // epoch. Do not let a sample from the previous namespace enable controls
  // while the new subscriptions are still waiting for state.
  m_sharedState = std::make_shared<SharedState>();
  auto readDouble = [this](const char *i_key, double &io_value) {
    if (!m_panelSettings.readDouble(QString::fromUtf8(i_key), io_value)) {
      io_value = readOrDeclare(m_node, i_key, io_value);
    }
  };
  auto readString = [this](const char *i_key, std::string &io_value) {
    if (!m_panelSettings.readString(QString::fromUtf8(i_key), io_value)) {
      io_value = readOrDeclare(m_node, i_key, io_value);
    }
  };
  readDouble("joint_lower_m", m_jointLowerM);
  readDouble("joint_upper_m", m_jointUpperM);
  readDouble("minimum_effort_n", m_minimumEffortN);
  readDouble("maximum_effort_n", m_maximumEffortN);
  readDouble("default_effort_n", m_defaultEffortN);
  readString("joint_name", m_jointName);
  readString("action_name", m_actionName);
  readString("joint_states_topic", m_jointStatesTopic);
  readString("limits_topic", m_limitsTopic);
  readString("gripper_state_topic", m_gripperStateTopic);
  readString("controller_manager_service", m_controllerManagerService);
  if (!std::isfinite(m_jointLowerM) || !std::isfinite(m_jointUpperM) ||
      m_jointUpperM <= m_jointLowerM) {
    m_jointLowerM = 0.0;
    m_jointUpperM = 0.019;
  }
  if (!std::isfinite(m_maximumEffortN) || m_maximumEffortN <= 0.0) {
    m_maximumEffortN = 95.0;
  }
  if (!std::isfinite(m_minimumEffortN) || m_minimumEffortN < 0.0 ||
      m_minimumEffortN >= m_maximumEffortN) {
    m_minimumEffortN = 0.0;
  }
  if (!std::isfinite(m_defaultEffortN) ||
      m_defaultEffortN < m_minimumEffortN) {
    m_defaultEffortN = m_minimumEffortN;
  }
  m_effortSpinBox->setRange(m_minimumEffortN, m_maximumEffortN);
  const double effort = std::isfinite(m_savedEffortN)
                            ? m_savedEffortN
                            : m_defaultEffortN;
  m_effortSpinBox->setValue(
      std::clamp(effort, m_minimumEffortN, m_maximumEffortN));
  m_targetSpinBox->setRange(m_jointLowerM, m_jointUpperM);
  if (std::isfinite(m_savedTargetM)) {
    m_targetSpinBox->setValue(
        std::clamp(m_savedTargetM, m_jointLowerM, m_jointUpperM));
  }
  m_jointStateSubscription.reset();
  m_limitSubscription.reset();
  m_gripperStateSubscription.reset();
  m_actionClient.reset();
  m_controllerListClient.reset();
  m_speedGetParametersClient.reset();
  m_actionClient =
      rclcpp_action::create_client<GripperAction>(m_node, m_actionName);
  m_controllerListClient =
      m_node->create_client<controller_manager_msgs::srv::ListControllers>(
          m_controllerManagerService);
  try {
    m_controllerParameterNode =
        controllerNodeFromActionName(m_node, m_actionName);
    if (!m_controllerParameterNode.empty()) {
      m_speedGetParametersClient = m_node->create_client<GetParameters>(
          serviceForController(m_controllerParameterNode, "get_parameters"));
    }
  } catch (const std::exception &error) {
    m_controllerParameterNode.clear();
    RCLCPP_WARN(m_node->get_logger(), "Cannot resolve speed parameters: %s", error.what());
  }
  const auto state = m_sharedState;
  const auto jointName = m_jointName;
  m_jointStateSubscription =
      m_node->create_subscription<sensor_msgs::msg::JointState>(
          m_jointStatesTopic, rclcpp::SensorDataQoS(),
          [state, jointName](
              const sensor_msgs::msg::JointState::SharedPtr i_message) {
            for (size_t index = 0; index < i_message->name.size(); ++index) {
              if (i_message->name[index] != jointName) {
                continue;
              }
              // This is the raw broadcaster stream, not the visualization
              // filter. Unavailable task feedback must immediately revoke
              // readiness; optional unavailable effort does not do so.
              if (index >= i_message->position.size() ||
                  !std::isfinite(i_message->position[index])) {
                state->has_measurement.store(false, std::memory_order_release);
                return;
              }
              const double position = i_message->position[index];
              const double effort =
                  index < i_message->effort.size()
                      ? i_message->effort[index]
                      : std::numeric_limits<double>::quiet_NaN();
              state->measured_position.store(position,
                                             std::memory_order_relaxed);
              state->measured_effort.store(effort, std::memory_order_relaxed);
              state->last_measurement_ns.store(
                  std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::steady_clock::now().time_since_epoch())
                      .count(),
                  std::memory_order_release);
              state->has_measurement.store(true, std::memory_order_release);
              break;
            }
          });
  m_limitSubscription =
      m_node->create_subscription<control_msgs::msg::Float64Values>(
          m_limitsTopic, rclcpp::SensorDataQoS(),
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
            state->minimum_aperture_m.store(minimum, std::memory_order_relaxed);
            state->maximum_aperture_m.store(maximum, std::memory_order_relaxed);
            state->has_limits.store(true, std::memory_order_release);
          });
  m_gripperStateSubscription =
      m_node->create_subscription<onrobot_gripper_msgs::msg::GripperState>(
          m_gripperStateTopic, rclcpp::SensorDataQoS(),
          [state](
              const onrobot_gripper_msgs::msg::GripperState::SharedPtr msg) {
            state->safety_1_pushed.store(msg->safety_1_pushed);
            state->safety_1_triggered.store(msg->safety_1_triggered);
            state->safety_2_pushed.store(msg->safety_2_pushed);
            state->safety_2_triggered.store(msg->safety_2_triggered);
            state->safety_dc_error.store(msg->safety_dc_error);
            state->safety_status_valid.store(msg->safety_status_valid,
                                             std::memory_order_release);
          });
  if (m_speedSpinBox) {
    m_speedSpinBox->setEnabled(false);
  }
  RCLCPP_INFO(m_node->get_logger(),
              "Gripper panel: joint '%s', feedback '%s', action '%s'",
              m_jointName.c_str(), m_jointStateSubscription->get_topic_name(),
              m_actionName.c_str());
  setStatus("Connected to RViz ROS node; waiting for controller action");
}

bool GripperControlPanel::hasFreshMeasurement() const {
  const auto state = m_sharedState;
  if (!state || !state->has_measurement.load(std::memory_order_acquire) ||
      !std::isfinite(state->measured_position.load(std::memory_order_relaxed))) {
    return false;
  }
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  const auto received = state->last_measurement_ns.load(std::memory_order_acquire);
  return received > 0 && now >= received && now - received <= 1500000000LL;
}

void GripperControlPanel::sendGoal(double i_positionM) {
  if (m_sharedState && m_sharedState->speed_control_available.load() &&
      steadyNowNs() - m_sharedState->speed_confirmed_ns.load() >=
          kSpeedPollIntervalNs + kSpeedReadTimeoutNs) {
    setStatus("Waiting for current conventional speed support");
    return;
  }
  auto action_client = m_actionClient;
  if (!action_client ||
      !action_client->wait_for_action_server(std::chrono::milliseconds(100))) {
    setStatus("Standard ros2_control gripper action is unavailable");
    return;
  }

  // Recheck at the click, even if the GUI's next periodic refresh has not
  // disabled the widgets yet. Never command from unavailable raw feedback.
  if (!hasFreshMeasurement()) {
    setCommandWidgetsEnabled(false);
    setStatus("State unavailable or stale; waiting for valid gripper feedback");
    return;
  }

  GripperAction::Goal goal;
  goal.command.name = {m_jointName};
  goal.command.position = {
      std::clamp(i_positionM, m_jointLowerM, m_jointUpperM)};
  goal.command.effort = {m_effortSpinBox->value()};
  if (m_sharedState->speed_control_available.load()) {
    goal.command.velocity = {m_speedSpinBox->value()};
  }

  const auto state = m_sharedState;
  rclcpp_action::Client<GripperAction>::SendGoalOptions options;
  options.goal_response_callback = [state](GoalHandle::SharedPtr i_goal) {
    state->action_status.store(i_goal ? kActionAccepted : kActionRejected,
                               std::memory_order_release);
  };
  options.feedback_callback =
      [state](GoalHandle::SharedPtr,
              const std::shared_ptr<const GripperAction::Feedback> i_feedback) {
        if (!i_feedback || i_feedback->state.position.empty()) {
          return;
        }
        state->action_status.store(kActionMoving, std::memory_order_release);
      };
  options.result_callback = [state](const GoalHandle::WrappedResult &i_result) {
    int status = kActionAborted;
    switch (i_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      status = kActionSucceeded;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      status = kActionCanceled;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      status = kActionAborted;
      break;
    default:
      break;
    }
    state->action_status.store(status, std::memory_order_release);
  };
  action_client->async_send_goal(goal, options);
  setStatus(QString("Sending %1 m goal")
                .arg(goal.command.position.front(), 0, 'f', 4));
}

void GripperControlPanel::updateMeasuredState(double i_positionM,
                                              double i_effortN) {
  m_measuredLabel->clear();
  m_measuredPositionValue->setText(QString::number(i_positionM, 'f', 4));
  const bool effortValid = std::isfinite(i_effortN);
  m_measuredEffortCaption->setVisible(effortValid);
  m_measuredEffortValue->setVisible(effortValid);
  m_measuredEffortUnit->setVisible(effortValid);
  if (effortValid) {
    m_measuredEffortValue->setText(QString::number(i_effortN, 'f', 1));
  }
}

void GripperControlPanel::setCommandWidgetsEnabled(bool i_enabled) {
  if (m_commandWidgetsEnabled == i_enabled) {
    return;
  }
  for (auto *widget : m_commandWidgets) {
    widget->setEnabled(i_enabled);
  }
  m_commandWidgetsEnabled = i_enabled;
}

void GripperControlPanel::setStatus(const QString &i_text) {
  if (m_statusLabel) {
    m_statusLabel->setText(i_text);
    updateWrappedLabelHeight(m_statusLabel);
    if (i_text.startsWith("DISCONNECTED") ||
        i_text.startsWith("CONNECTION LOST")) {
      m_statusLabel->setStyleSheet(
          "QLabel { color: #d32f2f; font-weight: bold; }");
    } else if (i_text.startsWith("Connected")) {
      m_statusLabel->setStyleSheet(
          "QLabel { color: #2e7d32; font-weight: bold; }");
    } else if (i_text.startsWith("Controller online") ||
               i_text.startsWith("State online")) {
      m_statusLabel->setStyleSheet(
          "QLabel { color: #ef6c00; font-weight: bold; }");
    } else {
      m_statusLabel->setStyleSheet(QString());
    }
  }
}

void GripperControlPanel::requestSpeedParameters() {
  const auto state = m_sharedState;
  if (!state || !m_speedGetParametersClient) return;
  const auto generation = state->speed_request_generation.fetch_add(1) + 1;
  state->speed_read_in_flight.store(true);
  state->speed_read_started_ns.store(steadyNowNs());
  state->speed_last_read_ns.store(steadyNowNs());
  auto request = std::make_shared<GetParameters::Request>();
  request->names = {"conventional_speed_control", "model"};
  try {
    m_speedGetParametersClient->async_send_request(
        request, [state, generation](GetParametersClient::SharedFuture future) {
          if (state->speed_request_generation.load() != generation) return;
          state->speed_read_in_flight.store(false);
          try {
            const auto response = future.get();
            using Type = rcl_interfaces::msg::ParameterType;
            if (response->values.size() != 2 ||
                response->values[0].type != Type::PARAMETER_BOOL) return;
            const bool enabled = response->values[0].bool_value;
            const auto &model = response->values[1];
            const int model_id = model.type == Type::PARAMETER_STRING ?
                (model.string_value == "2fg7" ? 7 : model.string_value == "2fg14" ? 14 : 0) : 0;
            // Never silently fall back to an unbounded native command if a
            // speed-capable controller returns incomplete model information.
            if (enabled && model_id == 0) return;
            state->speed_model.store(model_id);
            state->speed_control_available.store(enabled);
            state->speed_confirmed_ns.store(steadyNowNs());
          } catch (const std::exception &) {
            // Existing confirmed state expires; a later poll can recover.
          }
        });
  } catch (const std::exception &) {
    state->speed_read_in_flight.store(false);
  }
}

void GripperControlPanel::updateSpeedPanel(bool command_ready) {
  if (!m_sharedState || !m_speedGroup) return;
  const bool supported = m_sharedState->speed_control_available.load();
  m_speedGroup->setVisible(supported);
  if (!supported) return;
  m_speedSpinBox->setMaximum(m_sharedState->speed_model.load() == 14 ? 0.360 : 0.400);
  m_speedSpinBox->setEnabled(command_ready);
  m_speedStatusLabel->setText(command_ready
      ? "Used by the next Open, Close or Send target."
      : "Waiting for current controller and feedback.");
}

void GripperControlPanel::load(const rviz_common::Config &i_config) {
  rviz_common::Panel::load(i_config);
  m_panelSettings.load(i_config);
  m_panelSettings.readDouble("target_aperture_m", m_savedTargetM);
  m_panelSettings.readDouble("effort_n", m_savedEffortN);
  double saved_speed = 0.100;
  if (m_panelSettings.readDouble("conventional_speed_m_s", saved_speed) &&
      std::isfinite(saved_speed) && saved_speed > 0.0) {
    m_speedSpinBox->setValue(saved_speed);
  }
  m_panelSettings.readDouble("joint_lower_m", m_jointLowerM);
  m_panelSettings.readDouble("joint_upper_m", m_jointUpperM);
  m_panelSettings.readDouble("minimum_effort_n", m_minimumEffortN);
  m_panelSettings.readDouble("maximum_effort_n", m_maximumEffortN);
  m_panelSettings.readDouble("default_effort_n", m_defaultEffortN);
  m_panelSettings.readString("joint_name", m_jointName);
  m_panelSettings.readString("action_name", m_actionName);
  m_panelSettings.readString("joint_states_topic", m_jointStatesTopic);
  m_panelSettings.readString("limits_topic", m_limitsTopic);
  m_panelSettings.readString("gripper_state_topic", m_gripperStateTopic);
  m_panelSettings.readString("controller_manager_service",
                             m_controllerManagerService);
  if (m_targetSpinBox && std::isfinite(m_savedTargetM) &&
      m_jointUpperM > m_jointLowerM) {
    m_targetSpinBox->setRange(m_jointLowerM, m_jointUpperM);
    m_targetSpinBox->setValue(
        std::clamp(m_savedTargetM, m_jointLowerM, m_jointUpperM));
  }
  if (m_effortSpinBox && std::isfinite(m_savedEffortN)) {
    m_effortSpinBox->setValue(
        std::clamp(m_savedEffortN, m_minimumEffortN, m_maximumEffortN));
  }
  // RViz versions differ in whether Panel::load runs before or after
  // onInitialize(). Recreating only this panel's ROS handles makes both
  // orders equivalent and applies saved per-instance routing immediately.
  if (m_node) {
    onInitialize();
  }
}

void GripperControlPanel::save(rviz_common::Config i_config) const {
  rviz_common::Panel::save(i_config);
  m_panelSettings.writeDouble(
      "target_aperture_m",
      m_targetSpinBox ? m_targetSpinBox->value()
                     : m_savedTargetM);
  m_panelSettings.writeDouble("effort_n",
                              m_effortSpinBox ? m_effortSpinBox->value()
                                              : m_savedEffortN);
  m_panelSettings.writeDouble("conventional_speed_m_s", m_speedSpinBox->value());
  m_panelSettings.writeDouble("joint_lower_m", m_jointLowerM);
  m_panelSettings.writeDouble("joint_upper_m", m_jointUpperM);
  m_panelSettings.writeDouble("minimum_effort_n", m_minimumEffortN);
  m_panelSettings.writeDouble("maximum_effort_n", m_maximumEffortN);
  m_panelSettings.writeDouble("default_effort_n", m_defaultEffortN);
  m_panelSettings.writeString("joint_name", m_jointName);
  m_panelSettings.writeString("action_name", m_actionName);
  m_panelSettings.writeString("joint_states_topic", m_jointStatesTopic);
  m_panelSettings.writeString("limits_topic", m_limitsTopic);
  m_panelSettings.writeString("gripper_state_topic", m_gripperStateTopic);
  m_panelSettings.writeString("controller_manager_service",
                              m_controllerManagerService);
  m_panelSettings.save(i_config);
}

} // namespace onrobot_gripper_rviz_plugins

PLUGINLIB_EXPORT_CLASS(onrobot_gripper_rviz_plugins::GripperControlPanel,
                       rviz_common::Panel)

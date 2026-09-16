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
#include <QSignalBlocker>
#include <QSpinBox>
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
constexpr int kSpeedChecking = 0;
constexpr int kSpeedReady = 1;
constexpr int kSpeedApplying = 2;
constexpr int kSpeedVerifying = 3;
constexpr int kSpeedRejected = 4;
constexpr int kSpeedUnavailable = 5;
constexpr int kSpeedMismatch = 6;
constexpr int kSpeedTimeout = 7;
constexpr int kSpeedUnsupported = 8;
constexpr int kSpeedUnapplied = 9;
constexpr std::int64_t kSpeedReadTimeoutNs = 3000000000LL;
constexpr std::int64_t kSpeedWriteNoticeNs = 3000000000LL;
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
  speedLayout->setSpacing(2);
  auto *speedInputLayout = new QHBoxLayout();
  speedInputLayout->addWidget(new QLabel("Speed (%):", m_speedGroup));
  m_speedSpinBox = new QSpinBox(m_speedGroup);
  m_speedSpinBox->setObjectName("conventionalSpeedPercent");
  m_speedSpinBox->setRange(1, 100);
  m_speedSpinBox->setSingleStep(1);
  m_speedSpinBox->setSuffix("%");
  m_speedSpinBox->setValue(50);
  m_speedSpinBox->setToolTip(
      "Native 2FG conventional speed percentage for future goals");
  speedInputLayout->addWidget(m_speedSpinBox, 1);
  m_speedApplyButton = new QPushButton("Apply", m_speedGroup);
  m_speedApplyButton->setObjectName("applyConventionalSpeed");
  m_speedApplyButton->setEnabled(false);
  speedInputLayout->addWidget(m_speedApplyButton);
  speedLayout->addLayout(speedInputLayout);
  m_speedCurrentLabel = new QLabel("Controller setting: —", m_speedGroup);
  m_speedCurrentLabel->setObjectName("conventionalSpeedCurrent");
  m_speedCurrentLabel->setMinimumWidth(180);
  m_speedStatusLabel = new QLabel("Checking controller speed support",
                                  m_speedGroup);
  m_speedStatusLabel->setObjectName("conventionalSpeedStatus");
  m_speedStatusLabel->setMinimumWidth(180);
  m_speedStatusLabel->setWordWrap(true);
  m_speedStatusLabel->setSizePolicy(QSizePolicy::Preferred,
                                    QSizePolicy::Preferred);
  speedLayout->addWidget(m_speedCurrentLabel);
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
    const bool speed_read_in_flight =
        m_sharedState->speed_read_in_flight.load(std::memory_order_acquire);
    if (speed_read_in_flight) {
      const auto read_started_ns =
          m_sharedState->speed_read_started_ns.load(std::memory_order_acquire);
      if (read_started_ns > 0 &&
          speed_now_ns - read_started_ns >= kSpeedReadTimeoutNs) {
        // Invalidate this request before retrying so a late response cannot
        // overwrite a more recent controller value.
        m_sharedState->speed_request_generation.fetch_add(
            1, std::memory_order_acq_rel);
        m_sharedState->speed_read_in_flight.store(false,
                                                  std::memory_order_release);
        setSpeedStatus(
            kSpeedTimeout,
            m_sharedState->speed_write_pending.load(std::memory_order_acquire)
                ? "Speed update is waiting for readback; motion stays disabled."
                : "Controller speed readback timed out; retrying.");
      }
    }
    const bool speed_write_pending =
        m_sharedState->speed_write_pending.load(std::memory_order_acquire);
    const bool speed_set_acknowledged =
        m_sharedState->speed_set_acknowledged.load(std::memory_order_acquire);
    if (speed_write_pending && !speed_set_acknowledged) {
      const auto write_started_ns = m_sharedState->speed_write_started_ns.load(
          std::memory_order_acquire);
      if (write_started_ns > 0 &&
          speed_now_ns - write_started_ns >= kSpeedWriteNoticeNs) {
        // The controller may have committed the parameter even when its
        // service response was lost.  Move into readback reconciliation;
        // never guess whether the write succeeded and never leave the panel
        // permanently waiting for a response that may not arrive.
        m_sharedState->speed_set_acknowledged.store(true,
                                                    std::memory_order_release);
        m_sharedState->speed_last_read_ns.store(0,
                                                std::memory_order_release);
        setSpeedStatus(
            kSpeedTimeout,
            "No response to the speed update; checking the controller value. "
            "Motion stays disabled until readback confirms it.");
      }
    }
    if (!m_sharedState->speed_read_in_flight.load(std::memory_order_acquire) &&
        m_speedGetParametersClient &&
        m_speedGetParametersClient->service_is_ready()) {
      const bool checked = m_sharedState->speed_capability_checked.load(
          std::memory_order_acquire);
      const bool supported = m_sharedState->speed_control_available.load(
          std::memory_order_acquire);
      const auto last_read_ns =
          m_sharedState->speed_last_read_ns.load(std::memory_order_acquire);
      const bool verify_write = speed_write_pending && speed_set_acknowledged;
      const bool refresh_supported =
          !speed_write_pending && checked && supported &&
          (last_read_ns == 0 ||
           speed_now_ns - last_read_ns >= kSpeedPollIntervalNs);
      if (!checked || verify_write || refresh_supported) {
        requestSpeedParameters();
      }
    } else if (!m_sharedState->speed_capability_checked.load(
                   std::memory_order_acquire)) {
      setSpeedStatus(kSpeedChecking,
                     "Waiting for the conventional controller parameters");
    }
    const bool command_ready =
        state_fresh && standard_ready && has_limits && !safety_blocked &&
        !speed_write_pending;
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
  connect(m_speedSpinBox, qOverload<int>(&QSpinBox::valueChanged), this,
          [this](int value) {
            const auto state = m_sharedState;
            if (!state ||
                state->speed_write_pending.load(std::memory_order_acquire)) {
              return;
            }
            state->speed_input_dirty.store(true, std::memory_order_release);
            setSpeedStatus(
                kSpeedUnapplied,
                "Selected " + std::to_string(value) +
                    "%; Apply to use it for the next conventional command.");
          });
  connect(m_speedApplyButton, &QPushButton::clicked, this,
          [this]() { applySpeedSetting(); });
}

GripperControlPanel::~GripperControlPanel() {
  if (m_stateTimer) {
    m_stateTimer->stop();
  }
  m_jointStateSubscription.reset();
  m_limitSubscription.reset();
  m_gripperStateSubscription.reset();
  m_parameterEventSubscription.reset();
  m_actionClient.reset();
  m_controllerListClient.reset();
  m_speedGetParametersClient.reset();
  m_speedSetParametersClient.reset();
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
  m_speedSetParametersClient.reset();
  m_parameterEventSubscription.reset();
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
      m_speedSetParametersClient = m_node->create_client<SetParametersAtomically>(
          serviceForController(m_controllerParameterNode,
                               "set_parameters_atomically"));
    }
  } catch (const std::exception &error) {
    m_controllerParameterNode.clear();
    setSpeedStatus(kSpeedUnavailable,
                   std::string("Cannot resolve controller parameter services: ") +
                       error.what());
  }
  if (!m_controllerParameterNode.empty()) {
    const auto state = m_sharedState;
    const auto controllerNode = m_controllerParameterNode;
    m_parameterEventSubscription =
        m_node->create_subscription<rcl_interfaces::msg::ParameterEvent>(
            "/parameter_events", rclcpp::ParameterEventsQoS(),
            [state, controllerNode](
                const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
              if (event->node != controllerNode) {
                return;
              }
              bool speed_control_seen = false;
              bool speed_control_enabled = false;
              bool speed_value_seen = false;
              int speed_value = 0;
              const auto inspect = [&](
                  const std::vector<rcl_interfaces::msg::Parameter> &parameters) {
                for (const auto &parameter : parameters) {
                  if (parameter.name == "conventional_speed_control" &&
                      parameter.value.type ==
                          rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
                    speed_control_seen = true;
                    speed_control_enabled = parameter.value.bool_value;
                  } else if (parameter.name == "conventional_speed_percent" &&
                             parameter.value.type == rcl_interfaces::msg::
                                 ParameterType::PARAMETER_INTEGER &&
                             parameter.value.integer_value >= 1 &&
                             parameter.value.integer_value <= 100) {
                    speed_value_seen = true;
                    speed_value = static_cast<int>(
                        parameter.value.integer_value);
                  }
                }
              };
              inspect(event->new_parameters);
              inspect(event->changed_parameters);
              if (speed_control_seen) {
                state->speed_capability_checked.store(
                    true, std::memory_order_release);
                state->speed_control_available.store(
                    speed_control_enabled, std::memory_order_release);
              }
              if (speed_value_seen) {
                state->speed_percent.store(speed_value,
                                           std::memory_order_relaxed);
                state->speed_current_valid.store(true,
                                                std::memory_order_release);
                state->speed_event_generation.fetch_add(
                    1, std::memory_order_acq_rel);
              }
              if (speed_control_seen && !speed_control_enabled) {
                GripperControlPanel::setSpeedStatus(
                    state, kSpeedUnsupported,
                    "Runtime speed adjustment is not available for this "
                    "controller or backend.");
              } else if (speed_value_seen &&
                         !state->speed_write_pending.load(
                             std::memory_order_acquire) &&
                         !state->speed_input_dirty.load(
                             std::memory_order_acquire) &&
                         state->speed_status_code.load(
                             std::memory_order_acquire) != kSpeedRejected &&
                         state->speed_status_code.load(
                             std::memory_order_acquire) != kSpeedUnapplied &&
                         state->speed_status_code.load(
                             std::memory_order_acquire) != kSpeedMismatch) {
                GripperControlPanel::setSpeedStatus(
                    state, kSpeedReady,
                    "Controller speed updated to " +
                        std::to_string(speed_value) + "%.");
              }
            });
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
    m_speedApplyButton->setEnabled(false);
    m_speedCurrentLabel->setText("Controller setting: —");
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
  if (m_sharedState &&
      m_sharedState->speed_write_pending.load(std::memory_order_acquire)) {
    setStatus("Waiting for the conventional speed setting to be confirmed");
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

void GripperControlPanel::setSpeedStatus(const std::shared_ptr<SharedState> &state,
                                         int code,
                                         const std::string &message) {
  if (!state) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(state->speed_status_mutex);
    state->speed_status_message = message;
  }
  state->speed_status_code.store(code, std::memory_order_release);
}

void GripperControlPanel::setSpeedStatus(int code,
                                         const std::string &message) {
  setSpeedStatus(m_sharedState, code, message);
}

void GripperControlPanel::requestSpeedReadback(
    const GetParametersClient::SharedPtr &client,
    const std::shared_ptr<SharedState> &state, std::uint64_t generation,
    std::uint64_t event_generation, int expected_percent) {
  if (!client || !state) {
    return;
  }
  auto request = std::make_shared<GetParameters::Request>();
  request->names = {"conventional_speed_control",
                    "conventional_speed_percent"};
  try {
    client->async_send_request(
        request,
        [state, generation, event_generation,
         expected_percent](GetParametersClient::SharedFuture future) {
          if (state->speed_request_generation.load(
                  std::memory_order_acquire) != generation) {
            return;
          }
          state->speed_read_in_flight.store(false, std::memory_order_release);
          try {
            const auto response = future.get();
            if (!response || response->values.size() != 2 ||
                response->values[0].type !=
                    rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
              GripperControlPanel::setSpeedStatus(
                  state, kSpeedUnavailable,
                  "Controller did not return a valid conventional speed "
                  "capability.");
              return;
            }
            const bool speed_supported = response->values[0].bool_value;
            state->speed_capability_checked.store(true,
                                                  std::memory_order_release);
            state->speed_control_available.store(speed_supported,
                                                 std::memory_order_release);
            if (!speed_supported) {
              state->speed_current_valid.store(false,
                                               std::memory_order_release);
              GripperControlPanel::setSpeedStatus(
                  state, kSpeedUnsupported,
                  "Runtime speed adjustment is not available for this "
                  "controller or backend.");
              return;
            }
            if (response->values[1].type !=
                    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER ||
                response->values[1].integer_value < 1 ||
                response->values[1].integer_value > 100) {
              GripperControlPanel::setSpeedStatus(
                  state, kSpeedUnavailable,
                  "Controller returned an invalid speed percentage.");
              return;
            }

            const bool event_is_newer =
                state->speed_event_generation.load(std::memory_order_acquire) !=
                event_generation;
            if (!event_is_newer) {
              state->speed_percent.store(
                  static_cast<int>(response->values[1].integer_value),
                  std::memory_order_relaxed);
              state->speed_current_valid.store(true,
                                               std::memory_order_release);
            }
            const int current_percent = state->speed_percent.load(
                std::memory_order_acquire);
            const bool pending = state->speed_write_pending.load(
                std::memory_order_acquire);
            if (pending && expected_percent >= 1 && expected_percent <= 100) {
              state->speed_write_pending.store(false,
                                               std::memory_order_release);
              state->speed_set_acknowledged.store(false,
                                                  std::memory_order_release);
              state->speed_expected_percent.store(-1,
                                                  std::memory_order_release);
              state->speed_input_dirty.store(false,
                                             std::memory_order_release);
              // Supersede the original SetParameters callback. It may arrive
              // after this readback when its service response was delayed;
              // that stale response must not restart or overwrite the settled
              // transaction.
              state->speed_request_generation.fetch_add(
                  1, std::memory_order_acq_rel);
              if (current_percent == expected_percent) {
                GripperControlPanel::setSpeedStatus(
                    state, kSpeedReady,
                    "Applied; the next conventional command uses " +
                        std::to_string(current_percent) + "%.");
              } else {
                GripperControlPanel::setSpeedStatus(
                    state, kSpeedMismatch,
                    "Requested " + std::to_string(expected_percent) +
                        "%, but the controller reports " +
                        std::to_string(current_percent) +
                        "%. Review other speed changes before moving.");
              }
            } else if (!pending &&
                       !state->speed_input_dirty.load(
                           std::memory_order_acquire) &&
                       state->speed_status_code.load(
                           std::memory_order_acquire) != kSpeedRejected &&
                       state->speed_status_code.load(
                           std::memory_order_acquire) != kSpeedUnapplied &&
                       state->speed_status_code.load(
                           std::memory_order_acquire) != kSpeedMismatch) {
              GripperControlPanel::setSpeedStatus(
                  state, kSpeedReady,
                  "Controller speed: " + std::to_string(current_percent) +
                      "%. Applies to future conventional commands.");
            }
          } catch (const std::exception &error) {
            const bool pending = state->speed_write_pending.load(
                std::memory_order_acquire);
            GripperControlPanel::setSpeedStatus(
                state, pending ? kSpeedTimeout : kSpeedUnavailable,
                std::string("Could not confirm controller speed: ") +
                    error.what() +
                    (pending ? ". Motion stays disabled until readback succeeds."
                             : ". Retrying the controller readback."));
          }
        });
  } catch (const std::exception &error) {
    state->speed_read_in_flight.store(false, std::memory_order_release);
    GripperControlPanel::setSpeedStatus(
        state, kSpeedUnavailable,
        std::string("Could not request controller speed: ") + error.what());
  }
}

void GripperControlPanel::requestSpeedParameters() {
  const auto state = m_sharedState;
  const auto client = m_speedGetParametersClient;
  if (!state || !client || !client->service_is_ready()) {
    return;
  }
  bool expected_in_flight = false;
  if (!state->speed_read_in_flight.compare_exchange_strong(
          expected_in_flight, true, std::memory_order_acq_rel)) {
    return;
  }
  const auto generation = state->speed_request_generation.load(
      std::memory_order_acquire);
  const auto event_generation = state->speed_event_generation.load(
      std::memory_order_acquire);
  const bool verify_write = state->speed_write_pending.load(
                                std::memory_order_acquire) &&
                            state->speed_set_acknowledged.load(
                                std::memory_order_acquire);
  const int expected_percent =
      verify_write
          ? state->speed_expected_percent.load(std::memory_order_acquire)
          : -1;
  const auto now_ns = steadyNowNs();
  state->speed_read_started_ns.store(now_ns, std::memory_order_release);
  state->speed_last_read_ns.store(now_ns, std::memory_order_release);
  requestSpeedReadback(client, state, generation, event_generation,
                       expected_percent);
}

void GripperControlPanel::applySpeedSetting() {
  const auto state = m_sharedState;
  const auto client = m_speedSetParametersClient;
  if (!state || !client || !state->speed_control_available.load(
                              std::memory_order_acquire)) {
    setSpeedStatus(kSpeedUnavailable,
                   "The controller does not currently support speed updates.");
    return;
  }
  if (!client->service_is_ready()) {
    setSpeedStatus(kSpeedUnavailable,
                   "Controller parameter service is unavailable; no update "
                   "was sent.");
    return;
  }
  if (state->speed_write_pending.load(std::memory_order_acquire) ||
      state->speed_read_in_flight.load(std::memory_order_acquire)) {
    return;
  }
  if (!m_commandWidgetsEnabled || !hasFreshMeasurement() ||
      !state->standard_action_ready.load(std::memory_order_acquire) ||
      !state->has_limits.load(std::memory_order_acquire)) {
    setSpeedStatus(kSpeedRejected,
                   "Wait for fresh gripper state and an active conventional "
                   "controller before applying speed.");
    return;
  }
  const int action_status = state->action_status.load(std::memory_order_acquire);
  if (action_status == kActionAccepted || action_status == kActionMoving) {
    setSpeedStatus(kSpeedRejected,
                   "Wait for the current conventional motion to finish.");
    return;
  }

  const int requested_percent = m_speedSpinBox->value();
  const auto generation = state->speed_request_generation.fetch_add(
                              1, std::memory_order_acq_rel) +
                          1;
  state->speed_expected_percent.store(requested_percent,
                                      std::memory_order_release);
  state->speed_write_pending.store(true, std::memory_order_release);
  state->speed_set_acknowledged.store(false, std::memory_order_release);
  state->speed_write_started_ns.store(steadyNowNs(),
                                      std::memory_order_release);
  setSpeedStatus(kSpeedApplying,
                 "Applying " + std::to_string(requested_percent) +
                     "% to the conventional controller");

  auto request = std::make_shared<SetParametersAtomically::Request>();
  request->parameters = {
      rclcpp::Parameter("conventional_speed_percent", requested_percent)
          .to_parameter_msg()};
  try {
    const auto read_client = m_speedGetParametersClient;
    client->async_send_request(
        request,
        [state, generation, requested_percent,
         read_client](SetParametersClient::SharedFuture future) {
          if (state->speed_request_generation.load(
                  std::memory_order_acquire) != generation) {
            return;
          }
          try {
            const auto response = future.get();
            if (!response || !response->result.successful) {
              state->speed_write_pending.store(false,
                                               std::memory_order_release);
              state->speed_set_acknowledged.store(false,
                                                  std::memory_order_release);
              state->speed_expected_percent.store(-1,
                                                  std::memory_order_release);
              const auto reason = response ? response->result.reason
                                           : std::string("empty service response");
              GripperControlPanel::setSpeedStatus(
                  state, kSpeedRejected,
                  "Rejected: " +
                      (reason.empty() ? std::string("controller declined the update")
                                      : reason));
              return;
            }
            state->speed_set_acknowledged.store(true,
                                                std::memory_order_release);
            state->speed_expected_percent.store(requested_percent,
                                                std::memory_order_release);
            state->speed_last_read_ns.store(0, std::memory_order_release);
            GripperControlPanel::setSpeedStatus(
                state, kSpeedVerifying,
                "Controller accepted the setting; checking its current value.");
            if (read_client && read_client->service_is_ready()) {
              bool expected_in_flight = false;
              if (state->speed_read_in_flight.compare_exchange_strong(
                      expected_in_flight, true, std::memory_order_acq_rel)) {
                const auto event_generation = state->speed_event_generation.load(
                    std::memory_order_acquire);
                const auto now_ns = steadyNowNs();
                state->speed_read_started_ns.store(now_ns,
                                                   std::memory_order_release);
                state->speed_last_read_ns.store(now_ns,
                                                std::memory_order_release);
                GripperControlPanel::requestSpeedReadback(
                    read_client, state, generation, event_generation,
                    requested_percent);
              }
            }
          } catch (const std::exception &error) {
            state->speed_set_acknowledged.store(true,
                                                std::memory_order_release);
            state->speed_expected_percent.store(requested_percent,
                                                std::memory_order_release);
            state->speed_last_read_ns.store(0, std::memory_order_release);
            GripperControlPanel::setSpeedStatus(
                state, kSpeedTimeout,
                std::string("No conclusive speed update response: ") +
                    error.what() +
                    ". Motion stays disabled until the controller setting is "
                    "read back.");
          }
        });
  } catch (const std::exception &error) {
    // A send failure does not prove that the remote parameter service did not
    // receive the request. Keep motion disabled and reconcile by readback.
    state->speed_set_acknowledged.store(true, std::memory_order_release);
    setSpeedStatus(kSpeedTimeout,
                   std::string("Speed update could not be confirmed: ") +
                       error.what() +
                       ". Motion stays disabled until readback succeeds.");
  }
}

void GripperControlPanel::updateSpeedPanel(bool command_ready) {
  const auto state = m_sharedState;
  if (!state || !m_speedGroup) {
    return;
  }
  const bool checked =
      state->speed_capability_checked.load(std::memory_order_acquire);
  const bool supported =
      state->speed_control_available.load(std::memory_order_acquire);
  m_speedGroup->setVisible(checked && supported);
  if (!checked || !supported) {
    return;
  }
  const bool current_valid =
      state->speed_current_valid.load(std::memory_order_acquire);
  const bool pending =
      state->speed_write_pending.load(std::memory_order_acquire);
  const bool read_in_flight =
      state->speed_read_in_flight.load(std::memory_order_acquire);
  const bool dirty =
      state->speed_input_dirty.load(std::memory_order_acquire);
  const int current_percent =
      state->speed_percent.load(std::memory_order_acquire);
  m_speedCurrentLabel->setText(
      current_valid
          ? QString("Controller setting: %1%").arg(current_percent)
          : QString("Controller setting: —"));
  if (current_valid && !dirty && !pending &&
      m_speedSpinBox->value() != current_percent) {
    const QSignalBlocker blocker(m_speedSpinBox);
    m_speedSpinBox->setValue(current_percent);
  }

  QString status_message;
  {
    std::lock_guard<std::mutex> lock(state->speed_status_mutex);
    status_message = QString::fromStdString(state->speed_status_message);
  }
  if (m_speedStatusLabel->text() != status_message) {
    m_speedStatusLabel->setText(status_message);
    updateWrappedLabelHeight(m_speedStatusLabel);
  }
  const int status_code =
      state->speed_status_code.load(std::memory_order_acquire);
  if (status_code == kSpeedRejected || status_code == kSpeedUnavailable ||
      status_code == kSpeedMismatch || status_code == kSpeedTimeout) {
    m_speedStatusLabel->setStyleSheet(
        "QLabel { color: #c62828; font-weight: bold; }");
  } else if (status_code == kSpeedReady) {
    m_speedStatusLabel->setStyleSheet(
        "QLabel { color: #2e7d32; font-weight: bold; }");
  } else if (status_code == kSpeedApplying || status_code == kSpeedVerifying) {
    m_speedStatusLabel->setStyleSheet(
        "QLabel { color: #499dda; font-weight: bold; }");
  } else if (status_code == kSpeedUnapplied) {
    m_speedStatusLabel->setStyleSheet(
        "QLabel { color: #ef6c00; font-weight: bold; }");
  } else {
    m_speedStatusLabel->setStyleSheet(QString());
  }
  m_speedSpinBox->setEnabled(supported && !pending);
  const int action_status =
      state->action_status.load(std::memory_order_acquire);
  const bool action_idle = action_status != kActionAccepted &&
                           action_status != kActionMoving;
  m_speedApplyButton->setEnabled(
      supported && current_valid && dirty && !pending && !read_in_flight &&
      command_ready && action_idle && m_speedSetParametersClient &&
      m_speedSetParametersClient->service_is_ready());
}

void GripperControlPanel::load(const rviz_common::Config &i_config) {
  rviz_common::Panel::load(i_config);
  m_panelSettings.load(i_config);
  m_panelSettings.readDouble("target_aperture_m", m_savedTargetM);
  m_panelSettings.readDouble("effort_n", m_savedEffortN);
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

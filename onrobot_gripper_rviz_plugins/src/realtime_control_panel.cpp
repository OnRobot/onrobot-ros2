#include "onrobot_gripper_rviz_plugins/realtime_control_panel.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include <QDoubleSpinBox>
#include <QApplication>
#include <QComboBox>
#include <QEvent>
#include <QFontMetrics>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPushButton>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include "onrobot_gripper_rviz_plugins/branded_panel.hpp"
#include "onrobot_gripper_rviz_plugins/realtime_panel_semantics.hpp"
#include "onrobot_gripper_rviz_plugins/safety_panel_semantics.hpp"

namespace onrobot_gripper_rviz_plugins {
namespace {

constexpr int kSliderMinimum = -100;
constexpr int kSliderMaximum = 100;
constexpr std::int64_t kPositionCommandTimeoutNs = 30000000000LL;
constexpr double kEndpointToleranceM = 0.0005;

void updateWrappedLabelHeight(QLabel *label) {
  const int width = std::max(label->width(), label->minimumWidth());
  const QRect renderRect(0, 0, width, 10000);
  const int renderedHeight = label->fontMetrics().boundingRect(
                                 renderRect, Qt::TextWordWrap, label->text())
                                 .height();
  const int requiredHeight = renderedHeight + 2;
  if (requiredHeight > label->minimumHeight()) {
    label->setMinimumHeight(requiredHeight);
  }
  label->updateGeometry();
}

// A stock QSlider uses relative, auto-repeating page steps for groove presses.
// A velocity joystick instead needs a fixed pointer position to mean a fixed
// command for the complete hold interval.
class RealtimeJoystickSlider final : public QSlider {
public:
  explicit RealtimeJoystickSlider(QWidget *i_parent)
      : QSlider(Qt::Horizontal, i_parent) {}

protected:
  void mousePressEvent(QMouseEvent *i_event) override {
    if (i_event->button() != Qt::LeftButton) {
      QSlider::mousePressEvent(i_event);
      return;
    }
    updateFromPointer(i_event->pos().x());
    setSliderDown(true);
    i_event->accept();
  }

  void mouseMoveEvent(QMouseEvent *i_event) override {
    if (!(i_event->buttons() & Qt::LeftButton)) {
      QSlider::mouseMoveEvent(i_event);
      return;
    }
    updateFromPointer(i_event->pos().x());
    i_event->accept();
  }

  void mouseReleaseEvent(QMouseEvent *i_event) override {
    if (i_event->button() != Qt::LeftButton) {
      QSlider::mouseReleaseEvent(i_event);
      return;
    }
    updateFromPointer(i_event->pos().x());
    setSliderDown(false);
    i_event->accept();
  }

private:
  void updateFromPointer(int i_x) {
    setValue(RealtimeJoystickMapper::pointerToSliderValue(
        i_x, std::max(1, width() - 1), minimum(), maximum(),
        layoutDirection() == Qt::RightToLeft));
  }
};

std::int64_t steady_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

} // namespace

RealtimeControlPanel::RealtimeControlPanel(QWidget *i_parent)
    : rviz_common::Panel(i_parent),
      m_sharedState(std::make_shared<SharedState>()) {
  setObjectName("OnRobotRealtimeControlPanel");
  applyOnRobotPanelStyle(this);
  // A docked panel can lose its window/application without a FocusOut on the
  // button itself. Observe those events without depending on the dock layout.
  qApp->installEventFilter(this);

  auto *layout = new QVBoxLayout(this);
  layout->addWidget(createOnRobotPanelHeader(
      this, "Realtime control",
      "Move to a target or use the realtime controls"));

  auto *positionGroup = new QGroupBox("Realtime position", this);
  positionGroup->setObjectName("realtimePositionGroup");
  auto *positionLayout = new QFormLayout(positionGroup);
  m_targetPositionSpinBox = new QDoubleSpinBox(positionGroup);
  m_targetPositionSpinBox->setObjectName("realtimeTargetAperture");
  m_targetPositionSpinBox->setRange(0.0, 0.2);
  m_targetPositionSpinBox->setDecimals(4);
  m_targetPositionSpinBox->setSingleStep(0.001);
  m_targetPositionSpinBox->setSuffix(" m");
  m_targetPositionSpinBox->setToolTip(
      "External task aperture; live limits come from the connected backend");
  positionLayout->addRow("Target aperture:", m_targetPositionSpinBox);

  m_positionForceCaption = new QLabel("Force limit:", positionGroup);
  m_positionForceCaption->setObjectName("realtimePositionForceCaption");
  m_positionForceSpinBox = new QDoubleSpinBox(positionGroup);
  m_positionForceSpinBox->setObjectName("realtimePositionForce");
  m_positionForceSpinBox->setRange(0.0, 40.0);
  m_positionForceSpinBox->setDecimals(1);
  m_positionForceSpinBox->setSingleStep(1.0);
  m_positionForceSpinBox->setValue(10.0);
  m_positionForceSpinBox->setSuffix(" N");
  m_positionForceSpinBox->setToolTip(
      "RG force limit sent with the realtime position command");
  m_positionForceCaption->setVisible(false);
  m_positionForceSpinBox->setVisible(false);
  positionLayout->addRow(m_positionForceCaption, m_positionForceSpinBox);

  auto *positionButtons = new QHBoxLayout();
  m_openButton = new QPushButton("Open", positionGroup);
  m_closeButton = new QPushButton("Close", positionGroup);
  m_sendPositionButton = new QPushButton("Send target", positionGroup);
  m_openButton->setObjectName("realtimeOpenButton");
  m_closeButton->setObjectName("realtimeCloseButton");
  m_sendPositionButton->setObjectName("realtimeSendTargetButton");
  positionButtons->addWidget(m_openButton);
  positionButtons->addWidget(m_closeButton);
  positionButtons->addWidget(m_sendPositionButton);
  positionLayout->addRow(positionButtons);
  m_endpointLabel = new QLabel("Waiting for live aperture", positionGroup);
  m_endpointLabel->setObjectName("realtimeEndpointLabel");
  m_endpointLabel->setWordWrap(true);
  positionLayout->addRow("Current:", m_endpointLabel);
  layout->addWidget(positionGroup);

  auto *joystickGroup = new QGroupBox("Realtime joystick", this);
  joystickGroup->setObjectName("realtimeJoystickGroup");
  auto *joystickLayout = new QVBoxLayout(joystickGroup);
  auto *directionLayout = new QHBoxLayout();
  directionLayout->addWidget(new QLabel("OPEN", joystickGroup));
  directionLayout->addStretch(1);
  directionLayout->addWidget(new QLabel("NEUTRAL", joystickGroup));
  directionLayout->addStretch(1);
  directionLayout->addWidget(new QLabel("CLOSE", joystickGroup));
  joystickLayout->addLayout(directionLayout);

  m_joystickSlider = new RealtimeJoystickSlider(joystickGroup);
  m_joystickSlider->setObjectName("realtimeVelocityJoystick");
  m_joystickSlider->setRange(kSliderMinimum, kSliderMaximum);
  m_joystickSlider->setValue(0);
  m_joystickSlider->setTickPosition(QSlider::TicksBelow);
  m_joystickSlider->setTickInterval(25);
  m_joystickSlider->setToolTip(
      "Left opens, right closes; distance from center sets velocity");
  m_joystickSlider->installEventFilter(this);
  joystickLayout->addWidget(m_joystickSlider);

  m_directionLabel = new QLabel("Neutral", joystickGroup);
  m_directionLabel->setAlignment(Qt::AlignCenter);
  joystickLayout->addWidget(m_directionLabel);
  layout->addWidget(joystickGroup);

  auto *settingsGroup = new QGroupBox("Velocity limit", this);
  settingsGroup->setObjectName("realtimeSettingsGroup");
  auto *settingsLayout = new QFormLayout(settingsGroup);
  m_maxVelocitySpinBox = new QDoubleSpinBox(settingsGroup);
  m_maxVelocitySpinBox->setObjectName("realtimeMaximumVelocity");
  m_maxVelocitySpinBox->setRange(0.001, 0.3);
  m_maxVelocitySpinBox->setDecimals(3);
  m_maxVelocitySpinBox->setSingleStep(0.005);
  m_maxVelocitySpinBox->setValue(
      RealtimeJoystickMapper::kDefaultMaximumVelocityMps);
  m_maxVelocitySpinBox->setSuffix(" m/s");
  m_maxVelocitySpinBox->setToolTip(
      "Joystick speed and 2FG position-approach limit; maximum is 0.3 m/s");
  settingsLayout->addRow("Maximum velocity:", m_maxVelocitySpinBox);
  layout->addWidget(settingsGroup);

  auto *forceGroup = new QGroupBox("2FG closing-force grip", this);
  m_forceGroup = forceGroup;
  forceGroup->setObjectName("realtimeForceGroup");
  auto *forceLayout = new QFormLayout(forceGroup);
  m_forceApproach = new QComboBox(forceGroup);
  m_forceApproach->setObjectName("realtimeForceApproach");
  m_forceApproach->addItems({"Position approach", "Velocity approach"});
  forceLayout->addRow("Approach:", m_forceApproach);
  m_gripForceSpinBox = new QDoubleSpinBox(forceGroup);
  m_gripForceSpinBox->setObjectName("realtimeGripForce");
  m_gripForceSpinBox->setRange(30.0, 95.0);
  m_gripForceSpinBox->setDecimals(0);
  m_gripForceSpinBox->setSuffix(" N");
  forceLayout->addRow("Closing target:", m_gripForceSpinBox);
  auto *forceHint = new QLabel(
      "Uses the aperture and velocity above. Release the button to stop. "
      "Support the workpiece before stopping or opening. Not safety-rated.", forceGroup);
  forceHint->setWordWrap(true);
  m_forceHint = forceHint;
  forceHint->setMinimumWidth(140);
  forceHint->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  forceHint->setAlignment(Qt::AlignLeft | Qt::AlignTop);
  forceLayout->addRow(forceHint);
  auto *forceButtons = new QHBoxLayout();
  m_holdGripButton = new QPushButton("Hold to grip", forceGroup);
  m_holdGripButton->setObjectName("realtimeHoldGripButton");
  m_holdGripButton->setAutoRepeat(false);
  m_holdGripButton->installEventFilter(this);
  m_releaseGripButton = new QPushButton("Release / open", forceGroup);
  m_releaseGripButton->setObjectName("realtimeReleaseGripButton");
  m_releaseGripButton->setToolTip(
      "Zero-force position command to the live open limit; requires over 1 mm travel");
  forceButtons->addWidget(m_holdGripButton);
  forceButtons->addWidget(m_releaseGripButton);
  forceLayout->addRow(forceButtons);
  m_gripStateLabel = new QLabel("Waiting for force-control capability", forceGroup);
  m_gripStateLabel->setObjectName("realtimeGripState");
  m_gripStateLabel->setWordWrap(true);
  m_gripStateLabel->setMinimumWidth(140);
  m_gripStateLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  forceLayout->addRow(m_gripStateLabel);
  forceGroup->setEnabled(false);
  layout->addWidget(forceGroup);
  connect(m_holdGripButton, &QPushButton::pressed,
          this, &RealtimeControlPanel::beginForceCommand);
  connect(m_holdGripButton, &QPushButton::released, this, [this]() {
    if (m_forceCommandActive) stopRealtime("force grip button released");
  });
  connect(m_releaseGripButton, &QPushButton::clicked,
          this, &RealtimeControlPanel::releaseForceGrip);

  auto *buttonLayout = new QHBoxLayout();
  auto *centerButton = new QPushButton("Center / stop", this);
  centerButton->setObjectName("realtimeStopButton");
  buttonLayout->addWidget(centerButton);
  layout->addLayout(buttonLayout);

  auto *stateGroup = new QGroupBox("Realtime state", this);
  auto *stateLayout = new QVBoxLayout(stateGroup);
  stateLayout->setSpacing(2);
  auto *measuredGrid = new QGridLayout();
  measuredGrid->setColumnStretch(0, 1);
  measuredGrid->setColumnMinimumWidth(1, 74);
  m_measuredLabel = new QLabel("Waiting for realtime state", stateGroup);
  m_measuredLabel->setObjectName("realtimeMeasuredLabel");
  m_statusLabel = new QLabel("RViz realtime panel is starting", stateGroup);
  m_statusLabel->setObjectName("realtimeStatusLabel");
  m_safetyLabel = new QLabel(stateGroup);
  m_safetyLabel->setObjectName("realtimeSafetyLabel");
  // Keep wrapped state fields responsive at narrow dock widths without using
  // a fixed field width that would hide the long safety/status text.
  const auto addMeasurementRow = [&](int row, const QString &caption,
                                     QLabel *&value, QLabel *&unit) {
    value = new QLabel(QString::fromUtf8("\u2014"), stateGroup);
    unit = new QLabel(stateGroup);
    value->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    value->setMinimumWidth(74);
    measuredGrid->addWidget(new QLabel(caption, stateGroup), row, 0);
    measuredGrid->addWidget(value, row, 1);
    measuredGrid->addWidget(unit, row, 2);
  };
  QLabel *taskPositionUnit = nullptr;
  QLabel *taskVelocityUnit = nullptr;
  addMeasurementRow(0, "Task aperture", m_taskPositionValue,
                    taskPositionUnit);
  addMeasurementRow(1, "Task velocity", m_taskVelocityValue,
                    taskVelocityUnit);
  addMeasurementRow(2, "Mechanism position", m_mechanismPositionValue,
                    m_mechanismPositionUnit);
  addMeasurementRow(3, "Mechanism velocity", m_mechanismVelocityValue,
                    m_mechanismVelocityUnit);
  m_forceCaption = new QLabel("Reported force", stateGroup);
  m_forceValue = new QLabel(QString::fromUtf8("\u2014"), stateGroup);
  m_forceValue->setObjectName("realtimeForceValue");
  m_forceUnit = new QLabel("N", stateGroup);
  m_forceValue->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  m_forceValue->setMinimumWidth(74);
  measuredGrid->addWidget(m_forceCaption, 4, 0);
  measuredGrid->addWidget(m_forceValue, 4, 1);
  measuredGrid->addWidget(m_forceUnit, 4, 2);
  taskPositionUnit->setText("m");
  taskVelocityUnit->setText("m/s");
  measuredGrid->addWidget(m_measuredLabel, 5, 0, 1, 3);
  m_statusLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  m_measuredLabel->setMinimumWidth(180);
  m_statusLabel->setMinimumWidth(180);
  m_measuredLabel->setWordWrap(true);
  m_statusLabel->setWordWrap(true);
  m_safetyLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  m_safetyLabel->setMinimumWidth(180);
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

  m_commandWidgets = {positionGroup, joystickGroup, settingsGroup};
  for (auto *widget : m_commandWidgets) {
    widget->setEnabled(false);
  }

  m_commandTimer = new QTimer(this);
  m_commandTimer->setInterval(20);
  connect(m_commandTimer, &QTimer::timeout, this, [this]() {
    if (m_forceCommandActive && !m_stopRequested) {
      publishForceCommand();
    } else if (m_dragging && !m_stopRequested) {
      publishJoystickCommand();
    } else if (m_positionCommandActive && !m_stopRequested) {
      publishPositionCommand();
    }
  });
  m_commandTimer->start();

  m_stateTimer = new QTimer(this);
  m_stateTimer->setInterval(100);
  connect(m_stateTimer, &QTimer::timeout, this, [this]() { updateStatus(); });
  m_stateTimer->start();

  connect(m_joystickSlider, &QSlider::sliderPressed, this, [this]() {
    if (m_forceCommandActive) stopRealtime("switch to joystick");
    m_positionCommandActive = false;
    m_dragging = true;
    m_stopRequested = false;
    m_directionGuardStopped = false;
    m_directionGuard.reset();
    publishJoystickCommand();
  });
  connect(m_joystickSlider, &QSlider::sliderReleased, this, [this]() {
    m_dragging = false;
    m_joystickSlider->setValue(0);
    stopRealtime("joystick released");
  });
  connect(m_joystickSlider, &QSlider::valueChanged, this, [this](int i_value) {
    const double axis = static_cast<double>(i_value) / 100.0;
    const double velocity = RealtimeJoystickMapper::axisToVelocity(
        axis, m_maxVelocitySpinBox->value());
    if (std::abs(velocity) < 1e-9) {
      m_directionLabel->setText("Neutral");
    } else {
      m_directionLabel->setText(
          QString("%1: %2 %3")
              .arg(velocity > 0.0 ? "Opening" : "Closing")
              .arg(std::abs(velocity), 0, 'f', 3)
              .arg(m_rgCoordinateProfile ? "rad/s" : "m/s"));
    }
  });
  connect(centerButton, &QPushButton::clicked, this, [this]() {
    m_dragging = false;
    m_joystickSlider->setValue(0);
    m_directionGuardStopped = false;
    stopRealtime("center button");
  });
  connect(m_openButton, &QPushButton::clicked, this, [this]() {
    if (!m_sharedState->has_limits.load(std::memory_order_acquire)) {
      setStatus("Waiting for live task-aperture limits");
      return;
    }
    const double target =
        m_sharedState->maximum_aperture_m.load(std::memory_order_relaxed);
    m_targetPositionSpinBox->setValue(target);
    beginPositionCommand(target);
  });
  connect(m_closeButton, &QPushButton::clicked, this, [this]() {
    if (!m_sharedState->has_limits.load(std::memory_order_acquire)) {
      setStatus("Waiting for live task-aperture limits");
      return;
    }
    const double target =
        m_sharedState->minimum_aperture_m.load(std::memory_order_relaxed);
    m_targetPositionSpinBox->setValue(target);
    beginPositionCommand(target);
  });
  connect(m_sendPositionButton, &QPushButton::clicked, this,
          [this]() { beginPositionCommand(m_targetPositionSpinBox->value()); });
}

RealtimeControlPanel::~RealtimeControlPanel() {
  if (qApp) qApp->removeEventFilter(this);
  if (m_commandTimer) {
    m_commandTimer->stop();
  }
  if (m_stateTimer) {
    m_stateTimer->stop();
  }
  stopRealtime("panel shutdown");
  m_stateSubscription.reset();
  m_limitSubscription.reset();
  m_gripperStateSubscription.reset();
  m_commandPublisher.reset();
}

bool RealtimeControlPanel::eventFilter(QObject *i_watched, QEvent *i_event) {
  if (m_forceCommandActive &&
      ((i_watched == qApp && i_event->type() == QEvent::ApplicationDeactivate) ||
       (i_watched == window() && i_event->type() == QEvent::WindowDeactivate) ||
       (i_watched == this && i_event->type() == QEvent::Hide))) {
    stopRealtime("force grip panel or application deactivated");
  }
  if (i_watched == m_holdGripButton && m_forceCommandActive &&
      (i_event->type() == QEvent::FocusOut || i_event->type() == QEvent::Hide ||
       i_event->type() == QEvent::WindowDeactivate)) {
    stopRealtime("force grip focus lost");
  }
  if (i_watched == m_joystickSlider && i_event->type() == QEvent::FocusOut &&
      m_dragging && !m_stopRequested) {
    m_dragging = false;
    m_joystickSlider->setValue(0);
    stopRealtime("joystick focus lost");
  }
  return QObject::eventFilter(i_watched, i_event);
}

void RealtimeControlPanel::resizeEvent(QResizeEvent *i_event) {
  QWidget::resizeEvent(i_event);
  updateWrappedLabelHeight(m_measuredLabel);
  updateWrappedLabelHeight(m_statusLabel);
  updateWrappedLabelHeight(m_safetyLabel);
}

void RealtimeControlPanel::onInitialize() {
  // Rebinding a panel is a safety boundary.  Disable the complete command
  // surface before looking up the new RViz context or ROS subscriptions so a
  // timer tick cannot leave the old namespace operable.
  setCommandWidgetsEnabled(false);
  m_dragging = false;
  m_positionCommandActive = false;
  m_forceCommandActive = false;
  m_forceGroup->setEnabled(false);
  m_stopRequested = true;
  m_directionGuardStopped = false;
  m_limitsApplied = false;
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    setStatus("DISCONNECTED: RViz ROS node is unavailable");
    return;
  }

  m_node = abstraction->get_raw_node();
  // A panel reload must not expose measurements or limits from the previous
  // namespace while the newly selected subscriptions are catching up.
  m_sharedState = std::make_shared<SharedState>();
  m_dragging = false;
  m_positionCommandActive = false;
  m_stopRequested = false;
  m_directionGuardStopped = false;
  m_limitsApplied = false;
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
  std::string coordinateProfile{"2fg"};
  readString("realtime_coordinate_profile", coordinateProfile);
  readString("realtime_command_topic", m_commandTopic);
  readString("realtime_state_topic", m_stateTopic);
  readString("limits_topic", m_limitsTopic);
  readString("gripper_state_topic", m_gripperStateTopic);
  m_rgCoordinateProfile = coordinateProfile == "rg";
  m_forceGroup->setVisible(!m_rgCoordinateProfile);
  if (coordinateProfile != "2fg" && !m_rgCoordinateProfile) {
    setStatus("Invalid realtime_coordinate_profile; expected 2fg or rg");
    return;
  }
  // Restore all 2FG presentation defaults before applying the selected
  // profile.  Without this reset, reloading the same panel from RG to 2FG
  // leaves RG units and the force editor visible.
  m_maxVelocitySpinBox->setRange(0.001, 0.3);
  m_maxVelocitySpinBox->setValue(
      RealtimeJoystickMapper::kDefaultMaximumVelocityMps);
  m_maxVelocitySpinBox->setSingleStep(0.005);
  m_maxVelocitySpinBox->setSuffix(" m/s");
  m_maxVelocitySpinBox->setToolTip(
      "Joystick speed and 2FG position-approach limit; maximum is 0.3 m/s");
  m_positionForceSpinBox->setRange(0.0, 40.0);
  m_positionForceSpinBox->setValue(10.0);
  m_positionForceSpinBox->setSuffix(" N");
  m_positionForceSpinBox->setVisible(false);
  m_positionForceCaption->setVisible(false);
  if (m_rgCoordinateProfile) {
    double maximumAngularVelocity = 0.1;
    readDouble("realtime_maximum_angular_velocity_rad_s",
               maximumAngularVelocity);
    if (!std::isfinite(maximumAngularVelocity) ||
        maximumAngularVelocity <= 0.0) {
      setStatus("Invalid RG realtime angular-velocity limit");
      return;
    }
    m_maxVelocitySpinBox->setRange(0.001, maximumAngularVelocity);
    m_maxVelocitySpinBox->setValue(std::min(0.1, maximumAngularVelocity));
    m_maxVelocitySpinBox->setSingleStep(0.01);
    m_maxVelocitySpinBox->setSuffix(" rad/s");
    m_maxVelocitySpinBox->setToolTip(
        "Maximum RG mechanism angular velocity used by the joystick");
    double maximumPositionForce = 40.0;
    double defaultPositionForce = 10.0;
    readDouble("realtime_maximum_position_force_n", maximumPositionForce);
    readDouble("realtime_default_position_force_n", defaultPositionForce);
    if (!std::isfinite(maximumPositionForce) || maximumPositionForce <= 0.0 ||
        !std::isfinite(defaultPositionForce) || defaultPositionForce < 0.0) {
      setStatus("Invalid RG realtime position-force configuration");
      return;
    }
    m_positionForceSpinBox->setRange(0.0, maximumPositionForce);
    m_positionForceSpinBox->setValue(
        std::min(defaultPositionForce, maximumPositionForce));
    m_positionForceCaption->setVisible(true);
    m_positionForceSpinBox->setVisible(true);
  }
  if (std::isfinite(m_savedMaximumVelocity)) {
    m_maxVelocitySpinBox->setValue(std::clamp(
        m_savedMaximumVelocity, m_maxVelocitySpinBox->minimum(),
        m_maxVelocitySpinBox->maximum()));
  }
  if (m_rgCoordinateProfile && std::isfinite(m_savedPositionForce)) {
    m_positionForceSpinBox->setValue(std::clamp(
        m_savedPositionForce, m_positionForceSpinBox->minimum(),
        m_positionForceSpinBox->maximum()));
  }
  m_stateSubscription.reset();
  m_limitSubscription.reset();
  m_gripperStateSubscription.reset();
  m_commandPublisher.reset();
  m_commandPublisher = m_node->create_publisher<RealtimeCommand>(
      m_commandTopic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  const auto state = m_sharedState;
  m_stateSubscription = m_node->create_subscription<RealtimeState>(
      m_stateTopic, rclcpp::SensorDataQoS(),
      [state](const RealtimeState::SharedPtr i_message) {
        state->has_state.store(true, std::memory_order_release);
        state->realtime_active.store(i_message->realtime_active,
                                     std::memory_order_relaxed);
        state->faulted.store(i_message->faulted, std::memory_order_relaxed);
        state->mechanism_linear_position_valid.store(
            i_message->mechanism_linear_position_valid,
            std::memory_order_relaxed);
        state->mechanism_linear_position.store(
            i_message->mechanism_linear_position, std::memory_order_relaxed);
        state->mechanism_linear_velocity_valid.store(
            i_message->mechanism_linear_velocity_valid,
            std::memory_order_relaxed);
        state->mechanism_linear_velocity.store(
            i_message->mechanism_linear_velocity, std::memory_order_relaxed);
        state->mechanism_angular_position_valid.store(
            i_message->mechanism_angular_position_valid,
            std::memory_order_relaxed);
        state->mechanism_angular_position.store(
            i_message->mechanism_angular_position, std::memory_order_relaxed);
        state->mechanism_angular_velocity_valid.store(
            i_message->mechanism_angular_velocity_valid,
            std::memory_order_relaxed);
        state->mechanism_angular_velocity.store(
            i_message->mechanism_angular_velocity, std::memory_order_relaxed);
        state->task_position_valid.store(i_message->task_position_valid,
                                         std::memory_order_relaxed);
        state->task_position.store(i_message->task_position,
                                   std::memory_order_relaxed);
        state->task_velocity_valid.store(i_message->task_velocity_valid,
                                         std::memory_order_relaxed);
        state->task_velocity.store(i_message->task_velocity,
                                   std::memory_order_relaxed);
        state->force_valid.store(i_message->force_valid,
                                 std::memory_order_relaxed);
        state->force.store(i_message->force, std::memory_order_relaxed);
        state->successful_cycles.store(i_message->successful_cycles,
                                       std::memory_order_relaxed);
        state->failed_cycles.store(i_message->failed_cycles,
                                   std::memory_order_relaxed);
        state->missed_deadlines.store(i_message->missed_deadlines,
                                      std::memory_order_relaxed);
        state->last_state_ns.store(steady_now_ns(), std::memory_order_release);
      });
  m_limitSubscription =
      m_node->create_subscription<control_msgs::msg::Float64Values>(
          m_limitsTopic, rclcpp::SensorDataQoS(),
          [state](const control_msgs::msg::Float64Values::SharedPtr i_message) {
            if (i_message->values.size() != 2) {
              state->has_limits.store(false);
              return;
            }
            const double minimum = i_message->values[0];
            const double maximum = i_message->values[1];
            if (!std::isfinite(minimum) || !std::isfinite(maximum) ||
                minimum < 0.0 || maximum <= minimum) {
              state->has_limits.store(false);
              return;
            }
            state->minimum_aperture_m.store(minimum, std::memory_order_relaxed);
            state->maximum_aperture_m.store(maximum, std::memory_order_relaxed);
            state->limits_received_ns.store(steady_now_ns());
            state->has_limits.store(true, std::memory_order_release);
          });
  m_gripperStateSubscription =
      m_node->create_subscription<onrobot_gripper_msgs::msg::GripperState>(
          m_gripperStateTopic, rclcpp::SensorDataQoS(),
          [state](
              const onrobot_gripper_msgs::msg::GripperState::SharedPtr msg) {
            {
              std::lock_guard<std::mutex> lock(state->typed_mutex);
              state->typed = *msg;
              state->typed_received_ns = steady_now_ns();
            }
            state->safety_1_pushed.store(msg->safety_1_pushed);
            state->safety_1_triggered.store(msg->safety_1_triggered);
            state->safety_2_pushed.store(msg->safety_2_pushed);
            state->safety_2_triggered.store(msg->safety_2_triggered);
            state->safety_dc_error.store(msg->safety_dc_error);
            state->safety_status_valid.store(msg->safety_status_valid,
                                             std::memory_order_release);
          });
  setStatus("Checking realtime driver connection...");
}

void RealtimeControlPanel::beginPositionCommand(double i_targetPositionM) {
  if (m_forceCommandActive) stopRealtime("switch to position control");
  if (!m_commandPublisher || !m_sharedState ||
      !m_sharedState->has_limits.load(std::memory_order_acquire)) {
    setStatus("Waiting for realtime driver state and task-aperture limits");
    return;
  }
  const double minimum =
      m_sharedState->minimum_aperture_m.load(std::memory_order_relaxed);
  const double maximum =
      m_sharedState->maximum_aperture_m.load(std::memory_order_relaxed);
  if (!std::isfinite(i_targetPositionM) || !std::isfinite(minimum) ||
      !std::isfinite(maximum) || maximum <= minimum) {
    setStatus("Realtime position target or live limits are invalid");
    return;
  }

  const double target = std::clamp(i_targetPositionM, minimum, maximum);
  const bool taskValid =
      m_sharedState->task_position_valid.load(std::memory_order_acquire);
  const double current =
      m_sharedState->task_position.load(std::memory_order_relaxed);
  if (taskValid && std::isfinite(current) &&
      std::abs(current - target) <= kEndpointToleranceM) {
    setStatus(QString("Already at %1 m").arg(target, 0, 'f', 4));
    return;
  }
  m_dragging = false;
  m_joystickSlider->setValue(0);
  m_directionGuard.reset();
  m_directionGuardStopped = false;
  m_positionTracker.reset(target);
  m_positionCommandStartedNs = steady_now_ns();
  m_positionCommandActive = true;
  m_stopRequested = false;
  publishPositionCommand();
  setStatus(QString("Moving to %1 m with realtime position control")
                .arg(target, 0, 'f', 4));
}

void RealtimeControlPanel::publishPositionCommand() {
  if (!m_commandPublisher || !m_node || !m_positionCommandActive ||
      m_stopRequested) {
    return;
  }
  RealtimeCommand message;
  message.header.stamp = m_node->now();
  message.mode = RealtimeCommand::POSITION;
  message.task_position = m_positionTracker.target();
  if (m_rgCoordinateProfile) {
    message.force = m_positionForceSpinBox->value();
  } else {
    message.task_velocity = m_maxVelocitySpinBox->value();
  }
  m_commandPublisher->publish(message);
}

void RealtimeControlPanel::publishJoystickCommand() {
  if (!m_commandPublisher || !m_joystickSlider || !m_maxVelocitySpinBox) {
    return;
  }
  const double axis = static_cast<double>(m_joystickSlider->value()) / 100.0;
  RealtimeCommand message;
  message.header.stamp = m_node ? m_node->now() : rclcpp::Time(0);
  message.mode = RealtimeCommand::VELOCITY;
  const double velocity = RealtimeJoystickMapper::axisToVelocity(
      axis, m_maxVelocitySpinBox->value());
  if (m_rgCoordinateProfile) {
    message.mechanism_angular_velocity = velocity;
  } else {
    message.task_velocity = velocity;
  }
  m_commandPublisher->publish(message);
}

bool RealtimeControlPanel::forceCommandReady(int *o_model) const {
  using GS = onrobot_gripper_msgs::msg::GripperState;
  if (m_rgCoordinateProfile || !m_node || !m_commandPublisher || !m_sharedState)
    return false;
  const auto now = steady_now_ns();
  const auto fresh = [now](std::int64_t then) {
    return then > 0 && now >= then && now - then <= 500000000LL;
  };
  if (!m_sharedState->has_state.load() || !m_sharedState->has_limits.load() ||
      !fresh(m_sharedState->last_state_ns.load()) ||
      !fresh(m_sharedState->limits_received_ns.load()) ||
      m_sharedState->faulted.load() || !m_sharedState->task_position_valid.load() ||
      !std::isfinite(m_sharedState->task_position.load())) return false;
  std::lock_guard<std::mutex> lock(m_sharedState->typed_mutex);
  const auto &state = m_sharedState->typed;
  const int model = state.model == "2fg7" ? 7 : state.model == "2fg14" ? 14 : 0;
  if (o_model) *o_model = model;
  const double age = state.sample_age.sec + state.sample_age.nanosec * 1e-9;
  return model != 0 && fresh(m_sharedState->typed_received_ns) &&
         age >= 0.0 && age <= 0.5 && state.sample_sequence > 0 &&
         state.realtime_force_control_available &&
         state.mapping_validity == GS::MAPPING_VALID &&
         state.task_aperture_valid && std::isfinite(state.task_aperture) &&
         state.fault_source == GS::FAULT_SOURCE_NONE && state.fault_code == 0 &&
         (state.connection_state == GS::CONNECTION_IDLE ||
          state.connection_state == GS::CONNECTION_ACTIVE);
}

void RealtimeControlPanel::beginForceCommand() {
  int model = 0;
  if (!forceCommandReady(&model)) {
    setStatus("Force control unavailable: check fresh state, backend and firmware");
    return;
  }
  const double target = m_targetPositionSpinBox->value();
  const double force = m_gripForceSpinBox->value();
  const double velocity = m_maxVelocitySpinBox->value();
  const double current = m_sharedState->task_position.load();
  const bool positionApproach = m_forceApproach->currentIndex() == 0;
  if (!std::isfinite(target) || !std::isfinite(force) || !std::isfinite(velocity) ||
      force < (model == 7 ? 30.0 : 40.0) || force > (model == 7 ? 95.0 : 196.0) ||
      velocity <= 0.0 || velocity > 0.3 ||
      target < m_sharedState->minimum_aperture_m.load() ||
      target > m_sharedState->maximum_aperture_m.load() ||
      (positionApproach && target >= current - kEndpointToleranceM)) {
    setStatus("Choose a closing target below the current aperture and a valid force/speed");
    return;
  }
  // Snapshot intent once. Editing another field must not change a held grip.
  m_forceCommand = RealtimeCommand{};
  m_forceCommand.mode = positionApproach ? RealtimeCommand::FORCE_POSITION
                                         : RealtimeCommand::FORCE_VELOCITY;
  m_forceCommand.task_position = positionApproach ? target : 0.0;
  m_forceCommand.task_velocity = positionApproach ? velocity : -velocity;
  m_forceCommand.force = force;
  m_forceModel = model;
  m_dragging = false;
  m_joystickSlider->setValue(0);
  m_positionCommandActive = false;
  m_forceCommandActive = true;
  m_positionCommandStartedNs = steady_now_ns();
  m_stopRequested = false;
  publishForceCommand();
}

void RealtimeControlPanel::publishForceCommand() {
  int model = 0;
  if (!forceCommandReady(&model) || model != m_forceModel ||
      (m_forceCommand.mode == RealtimeCommand::FORCE_POSITION &&
       (m_forceCommand.task_position < m_sharedState->minimum_aperture_m.load() ||
        m_forceCommand.task_position > m_sharedState->maximum_aperture_m.load()))) {
    stopRealtime("force capability, state or limits changed");
    return;
  }
  bool contact = false;
  {
    std::lock_guard<std::mutex> lock(m_sharedState->typed_mutex);
    contact = m_sharedState->typed.grip_detected;
  }
  if (!contact && steady_now_ns() - m_positionCommandStartedNs >
                      kPositionCommandTimeoutNs) {
    stopRealtime("force approach timed out without confirmed contact");
    return;
  }
  m_forceCommand.header.stamp = m_node->now();
  m_commandPublisher->publish(m_forceCommand);
}

void RealtimeControlPanel::releaseForceGrip() {
  if (!forceCommandReady()) {
    setStatus("Release unavailable: fresh 2FG state and limits are required");
    return;
  }
  const double target = m_sharedState->maximum_aperture_m.load();
  const double current = m_sharedState->task_position.load();
  // Margin beyond firmware's strict >1 mm rule, including 0.1 mm encoding.
  if (target - current < 0.002) {
    setStatus("Release needs at least 2 mm of opening travel; support the workpiece and Stop");
    return;
  }
  m_targetPositionSpinBox->setValue(target);
  beginPositionCommand(target); // POSITION maps to selector 6, force = zero.
}

void RealtimeControlPanel::updateForceControls() {
  // This text is static: shrink as well as grow after a narrow initial layout,
  // unlike the height-stable live status fields.
  const int hintHeight = m_forceHint->fontMetrics().boundingRect(
      QRect(0, 0, std::max(140, m_forceHint->width()), 10000),
      Qt::TextWordWrap, m_forceHint->text()).height() + 2;
  m_forceHint->setFixedHeight(hintHeight);
  updateWrappedLabelHeight(m_gripStateLabel);
  int model = 0;
  const bool ready = forceCommandReady(&model);
  if (m_forceCommandActive && (!ready || model != m_forceModel))
    stopRealtime("force state unavailable");
  m_forceGroup->setEnabled(ready);
  if (model == 7 || model == 14) {
    m_gripForceSpinBox->setRange(model == 7 ? 30.0 : 40.0,
                                model == 7 ? 95.0 : 196.0);
  }
  m_forceApproach->setEnabled(!m_forceCommandActive);
  m_gripForceSpinBox->setEnabled(!m_forceCommandActive);
  if (!ready) {
    m_gripStateLabel->setText("Force commands unavailable or state stale");
    return;
  }
  std::lock_guard<std::mutex> lock(m_sharedState->typed_mutex);
  const auto &state = m_sharedState->typed;
  m_gripStateLabel->setText(state.grip_detected ? "Grip detected by device"
      : state.force_valid ? "No grip detected" : "No grip detected; force feedback unavailable");
}

void RealtimeControlPanel::stopRealtime(const char *i_reason) {
  m_forceCommandActive = false;
  if (m_stopRequested) {
    return;
  }
  m_stopRequested = true;
  m_positionCommandActive = false;
  m_directionGuard.reset();
  if (!m_commandPublisher || !m_node) {
    setStatus(QString("Stop requested (%1); command publisher unavailable")
                  .arg(i_reason));
    return;
  }
  RealtimeCommand message;
  message.header.stamp = m_node->now();
  message.mode = RealtimeCommand::STOP;
  m_commandPublisher->publish(message);
  setStatus(QString("Realtime stop sent (%1)").arg(i_reason));
}

void RealtimeControlPanel::setCommandWidgetsEnabled(bool i_enabled) {
  if (m_commandWidgetsEnabled == i_enabled) {
    return;
  }
  for (auto *widget : m_commandWidgets) {
    widget->setEnabled(i_enabled);
  }
  m_commandWidgetsEnabled = i_enabled;
}

void RealtimeControlPanel::updateStatus() {
  if (!m_sharedState) {
    return;
  }
  updateForceControls();
  const bool safety_valid =
      m_sharedState->safety_status_valid.load(std::memory_order_acquire);
  m_safetyLabel->setVisible(safety_valid);
  m_safetyCaption->setVisible(safety_valid);
  bool safety_blocked = false;
  if (safety_valid) {
    const bool pushed = m_sharedState->safety_1_pushed.load() ||
                        m_sharedState->safety_2_pushed.load();
    const bool triggered = m_sharedState->safety_1_triggered.load() ||
                           m_sharedState->safety_2_triggered.load();
    const bool dc_error = m_sharedState->safety_dc_error.load();
    m_safetyLabel->setText(
        formatSafetyState(m_sharedState->safety_1_pushed.load(),
                          m_sharedState->safety_1_triggered.load(),
                          m_sharedState->safety_2_pushed.load(),
                          m_sharedState->safety_2_triggered.load(), dc_error));
    updateWrappedLabelHeight(m_safetyLabel);
    m_safetyLabel->setStyleSheet(
        pushed || dc_error ? "QLabel { color: #d32f2f; font-weight: bold; }"
        : triggered        ? "QLabel { color: #ef6c00; font-weight: bold; }"
                           : "QLabel { color: #2e7d32; font-weight: bold; }");
    safety_blocked = pushed || triggered || dc_error;
    if (safety_blocked && (m_dragging || m_positionCommandActive) &&
        !m_stopRequested) {
      m_dragging = false;
      m_joystickSlider->setValue(0);
      stopRealtime("RG safety input active");
    }
  }

  const bool has_limits =
      m_sharedState->has_limits.load(std::memory_order_acquire);
  if (has_limits) {
    const double minimum =
        m_sharedState->minimum_aperture_m.load(std::memory_order_relaxed);
    const double maximum =
        m_sharedState->maximum_aperture_m.load(std::memory_order_relaxed);
    if (std::isfinite(minimum) && std::isfinite(maximum) && maximum > minimum) {
      const bool firstApplication = !m_limitsApplied;
      m_targetPositionSpinBox->setRange(minimum, maximum);
      if (firstApplication) {
        // A saved operator target is configuration, not a command.  Preserve
        // it across the asynchronous first live-limit sample and clamp it to
        // that fresh range; never publish from this restoration path.
        const double current = m_targetPositionSpinBox->value();
        const double initial = std::isfinite(m_savedTargetM)
                                   ? m_savedTargetM
                                   : current;
        m_targetPositionSpinBox->setValue(
            std::clamp(initial, minimum, maximum));
      }
      m_limitsApplied = true;
    }
  }
  if (!m_sharedState->has_state.load(std::memory_order_acquire)) {
    setCommandWidgetsEnabled(false);
    setStatus("DISCONNECTED: no realtime state received");
    return;
  }

  const auto age_ms = (steady_now_ns() - m_sharedState->last_state_ns.load(
                                             std::memory_order_acquire)) /
                      1000000;
  if (age_ms > 1500) {
    if ((m_dragging || m_positionCommandActive) && !m_stopRequested) {
      m_dragging = false;
      m_joystickSlider->setValue(0);
      stopRealtime("state stream stale");
    }
    setCommandWidgetsEnabled(false);
    setStatus(QString("CONNECTION LOST: realtime state is %1 ms old")
                  .arg(static_cast<qlonglong>(age_ms)));
    return;
  }

  if (m_sharedState->faulted.load(std::memory_order_acquire)) {
    if ((m_dragging || m_positionCommandActive) && !m_stopRequested) {
      m_dragging = false;
      m_joystickSlider->setValue(0);
      stopRealtime("realtime fault");
    }
    setCommandWidgetsEnabled(false);
    setStatus("REALTIME FAULT: call stop/acknowledge before restarting");
    return;
  }

  const bool task_valid =
      m_sharedState->task_position_valid.load(std::memory_order_relaxed);
  const bool command_ready = has_limits && task_valid && !safety_blocked;
  setCommandWidgetsEnabled(command_ready);

  const bool position_valid =
      m_rgCoordinateProfile
          ? m_sharedState->mechanism_angular_position_valid.load(
                std::memory_order_relaxed)
          : m_sharedState->mechanism_linear_position_valid.load(
                std::memory_order_relaxed);
  const bool velocity_valid =
      m_rgCoordinateProfile
          ? m_sharedState->mechanism_angular_velocity_valid.load(
                std::memory_order_relaxed)
          : m_sharedState->mechanism_linear_velocity_valid.load(
                std::memory_order_relaxed);
  const bool force_valid =
      m_sharedState->force_valid.load(std::memory_order_relaxed);
  const double position = m_rgCoordinateProfile
                              ? m_sharedState->mechanism_angular_position.load(
                                    std::memory_order_relaxed)
                              : m_sharedState->mechanism_linear_position.load(
                                    std::memory_order_relaxed);
  const double velocity = m_rgCoordinateProfile
                              ? m_sharedState->mechanism_angular_velocity.load(
                                    std::memory_order_relaxed)
                              : m_sharedState->mechanism_linear_velocity.load(
                                    std::memory_order_relaxed);
  const double force = m_sharedState->force.load(std::memory_order_relaxed);
  const double task_position =
      m_sharedState->task_position.load(std::memory_order_relaxed);
  if (has_limits && task_valid) {
    const double minimum =
        m_sharedState->minimum_aperture_m.load(std::memory_order_relaxed);
    const double maximum =
        m_sharedState->maximum_aperture_m.load(std::memory_order_relaxed);
    const bool atOpen =
        std::abs(task_position - maximum) <= kEndpointToleranceM;
    const bool atClose =
        std::abs(task_position - minimum) <= kEndpointToleranceM;
    m_endpointLabel->setText(atOpen    ? "Open limit reached"
                             : atClose ? "Closed limit reached"
                                       : "Between limits");
    m_openButton->setEnabled(command_ready && !atOpen);
    m_closeButton->setEnabled(command_ready && !atClose);
    m_openButton->setToolTip(atOpen ? "The gripper is already fully open"
                                    : "Move to the live open limit");
    m_closeButton->setToolTip(atClose ? "The gripper is already fully closed"
                                      : "Move to the live closed limit");
  } else {
    m_endpointLabel->setText("Waiting for live aperture");
  }
  const bool task_velocity_valid =
      m_sharedState->task_velocity_valid.load(std::memory_order_relaxed);
  const double task_velocity =
      m_sharedState->task_velocity.load(std::memory_order_relaxed);
  if (m_dragging && !m_stopRequested && task_valid) {
    const double axis = static_cast<double>(m_joystickSlider->value()) / 100.0;
    const double commandedVelocity = RealtimeJoystickMapper::axisToVelocity(
        axis, m_maxVelocitySpinBox->value());
    if (m_directionGuard.observe(commandedVelocity, task_position)) {
      m_directionGuardStopped = true;
      stopRealtime("measured motion opposed the held command");
      setStatus("STOPPED: measured motion opposed the held command");
      return;
    }
  }
  m_measuredLabel->clear();
  m_taskPositionValue->setText(
      task_valid ? QString::number(task_position, 'f', 4) : QString::fromUtf8("\u2014"));
  m_taskVelocityValue->setText(
      task_velocity_valid ? QString::number(task_velocity, 'f', 3)
                          : QString::fromUtf8("\u2014"));
  m_mechanismPositionValue->setText(
      position_valid ? QString::number(position, 'f', 4) : QString::fromUtf8("\u2014"));
  m_mechanismVelocityValue->setText(
      velocity_valid ? QString::number(velocity, 'f', 3) : QString::fromUtf8("\u2014"));
  m_mechanismPositionUnit->setText(m_rgCoordinateProfile ? "rad" : "m");
  m_mechanismVelocityUnit->setText(m_rgCoordinateProfile ? "rad/s" : "m/s");
  const bool showForce = !m_rgCoordinateProfile;
  m_forceCaption->setVisible(showForce);
  m_forceValue->setVisible(showForce);
  m_forceUnit->setVisible(showForce);
  if (showForce) {
    m_forceValue->setText(force_valid ? QString::number(force, 'f', 1)
                                     : QString::fromUtf8("\u2014"));
  }

  if (safety_blocked) {
    setStatus("STOPPED: RG safety input active");
    return;
  }
  if (!has_limits) {
    setStatus("State online; waiting for live task-aperture limits");
    return;
  }
  if (!task_valid) {
    setStatus("State online; task-aperture feedback is unavailable");
    return;
  }

  if (m_positionCommandActive && !m_stopRequested) {
    if (steady_now_ns() - m_positionCommandStartedNs >
        kPositionCommandTimeoutNs) {
      stopRealtime("position command timeout");
      setStatus("STOPPED: realtime position target timed out");
      return;
    }
    if (m_positionTracker.observe(task_valid, task_position)) {
      const double target = m_positionTracker.target();
      stopRealtime("position target reached");
      setStatus(QString("Position reached: %1 m").arg(target, 0, 'f', 4));
      return;
    }
    setStatus(QString("Moving to %1 m with realtime position control")
                  .arg(m_positionTracker.target(), 0, 'f', 4));
    return;
  }

  if (m_forceCommandActive && !m_stopRequested) {
    setStatus(QString("Closing-force control: %1 N target; keep Hold to grip pressed")
                  .arg(m_forceCommand.force, 0, 'f', 0));
    return;
  }

  if (m_directionGuardStopped) {
    setStatus("STOPPED: measured motion opposed the held command");
    return;
  }

  if (m_sharedState->realtime_active.load(std::memory_order_acquire)) {
    setStatus(QString("Connected: realtime active (%1 cycles)")
                  .arg(static_cast<qulonglong>(
                      m_sharedState->successful_cycles.load())));
  } else {
    setStatus("Connected: realtime driver idle");
  }
}

void RealtimeControlPanel::setStatus(const QString &i_text) {
  if (!m_statusLabel) {
    return;
  }
  m_statusLabel->setText(i_text);
  updateWrappedLabelHeight(m_statusLabel);
  if (i_text.startsWith("DISCONNECTED") ||
      i_text.startsWith("CONNECTION LOST") ||
      i_text.startsWith("REALTIME FAULT")) {
    m_statusLabel->setStyleSheet(
        "QLabel { color: #d32f2f; font-weight: bold; }");
  } else if (i_text.startsWith("Connected")) {
    m_statusLabel->setStyleSheet(
        "QLabel { color: #2e7d32; font-weight: bold; }");
  } else {
    m_statusLabel->setStyleSheet(QString());
  }
}

void RealtimeControlPanel::load(const rviz_common::Config &i_config) {
  rviz_common::Panel::load(i_config);
  m_panelSettings.load(i_config);
  m_panelSettings.readDouble("target_aperture_m", m_savedTargetM);
  m_panelSettings.readDouble("maximum_velocity", m_savedMaximumVelocity);
  m_panelSettings.readDouble("position_force_n", m_savedPositionForce);
  m_rgCoordinateProfile = false;
  std::string coordinateProfile;
  if (m_panelSettings.readString("realtime_coordinate_profile",
                                 coordinateProfile)) {
    m_rgCoordinateProfile = coordinateProfile == "rg";
  }
  m_panelSettings.readString("realtime_command_topic", m_commandTopic);
  m_panelSettings.readString("realtime_state_topic", m_stateTopic);
  m_panelSettings.readString("limits_topic", m_limitsTopic);
  m_panelSettings.readString("gripper_state_topic", m_gripperStateTopic);
  if (m_targetPositionSpinBox && std::isfinite(m_savedTargetM)) {
    m_targetPositionSpinBox->setValue(m_savedTargetM);
  }
  if (m_maxVelocitySpinBox && std::isfinite(m_savedMaximumVelocity)) {
    m_maxVelocitySpinBox->setValue(m_savedMaximumVelocity);
  }
  if (m_positionForceSpinBox && std::isfinite(m_savedPositionForce)) {
    m_positionForceSpinBox->setValue(m_savedPositionForce);
  }
  if (m_node) {
    stopRealtime("panel configuration reload");
    onInitialize();
  }
}

void RealtimeControlPanel::save(rviz_common::Config i_config) const {
  rviz_common::Panel::save(i_config);
  m_panelSettings.writeDouble(
      "target_aperture_m",
      m_targetPositionSpinBox ? m_targetPositionSpinBox->value()
                              : m_savedTargetM);
  m_panelSettings.writeDouble(
      "maximum_velocity",
      m_maxVelocitySpinBox ? m_maxVelocitySpinBox->value()
                           : m_savedMaximumVelocity);
  m_panelSettings.writeDouble(
      "position_force_n",
      m_positionForceSpinBox ? m_positionForceSpinBox->value()
                             : m_savedPositionForce);
  m_panelSettings.writeString(
      "realtime_coordinate_profile",
      m_rgCoordinateProfile ? "rg" : "2fg");
  m_panelSettings.writeString("realtime_command_topic", m_commandTopic);
  m_panelSettings.writeString("realtime_state_topic", m_stateTopic);
  m_panelSettings.writeString("limits_topic", m_limitsTopic);
  m_panelSettings.writeString("gripper_state_topic", m_gripperStateTopic);
  m_panelSettings.writeDouble("realtime_maximum_angular_velocity_rad_s",
                              m_maxVelocitySpinBox
                                  ? m_maxVelocitySpinBox->maximum()
                                  : 0.1);
  m_panelSettings.writeDouble("realtime_maximum_position_force_n",
                              m_positionForceSpinBox
                                  ? m_positionForceSpinBox->maximum()
                                  : 40.0);
  m_panelSettings.writeDouble("realtime_default_position_force_n",
                              m_positionForceSpinBox
                                  ? m_positionForceSpinBox->value()
                                  : 10.0);
  m_panelSettings.save(i_config);
}

} // namespace onrobot_gripper_rviz_plugins

PLUGINLIB_EXPORT_CLASS(onrobot_gripper_rviz_plugins::RealtimeControlPanel,
                       rviz_common::Panel)

#include "onrobot_gripper_rviz_plugins/force_history_panel.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include <QFrame>
#include <QDockWidget>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
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

double messageAgeSeconds(
    const onrobot_gripper_msgs::msg::GripperState &i_message) {
  return static_cast<double>(i_message.sample_age.sec) +
         static_cast<double>(i_message.sample_age.nanosec) * 1.0e-9;
}

bool supportsMeasuredForce(const std::string &i_model) {
  return i_model == "2fg7" || i_model == "2fg14";
}

QString modelName(const std::string &i_model) {
  return i_model.empty() ? QStringLiteral("unknown model")
                         : QString::fromStdString(i_model);
}

QString forceHistoryLegend(const std::string &i_model) {
  if (i_model == "rg2" || i_model == "rg6") {
    return QStringLiteral(
        "Measured force: unavailable for this model  •  shaded: grip detected  •  "
        "dashed markers: state changes");
  }
  if (i_model == "2fg7" || i_model == "2fg14") {
    return QStringLiteral(
        "Blue: valid measured force  •  shaded: grip detected  •  "
        "dashed markers: state changes  •  gaps: invalid or stale");
  }
  return QStringLiteral(
      "Measured-force availability follows the model  •  shaded: grip detected  •  "
      "dashed markers: state changes");
}

} // namespace

class ForceHistoryPanel::Plot final : public QWidget {
public:
  explicit Plot(QWidget *i_parent = nullptr) : QWidget(i_parent) {
    setObjectName(QStringLiteral("forceHistoryPlot"));
    // RViz places the standard control and history docks in the same column
    // on a fresh showcase.  Keep the plot readable while allowing both docks
    // to remain visible at the documented 1400x900 default window size.
    setMinimumSize(240, 108);
    setMaximumHeight(360);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    setAttribute(Qt::WA_OpaquePaintEvent);
  }

  void setSamples(const std::deque<ForceHistorySample> &i_samples) {
    m_samples.assign(i_samples.begin(), i_samples.end());
    update();
  }

  QSize sizeHint() const override { return QSize(420, 150); }

protected:
  void paintEvent(QPaintEvent *) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor(QStringLiteral("#26292B")));

    // Keep the time label inside a reserved axis band.  The legend below the
    // plot is word-wrapped for narrow RViz docks, so allowing the axis label
    // to extend to the widget edge makes the two regions overlap.
    const QRectF plotRect(47.0, 14.0,
                          std::max(1.0, static_cast<double>(width()) - 57.0),
                          std::max(1.0, static_cast<double>(height()) - 66.0));
    painter.setPen(QPen(QColor(QStringLiteral("#7E878E")), 1.0));
    painter.drawRect(plotRect);

    painter.setPen(QColor(QStringLiteral("#DFE4E8")));
    painter.drawText(QRectF(3.0, 0.0, width() - 6.0, 14.0),
                     Qt::AlignLeft | Qt::AlignVCenter,
                     QStringLiteral("measured force (N)"));
    painter.drawText(QRectF(plotRect.left(), plotRect.bottom() + 4.0,
                            plotRect.width(), 18.0),
                     Qt::AlignRight | Qt::AlignVCenter,
                     QStringLiteral("time"));

    if (m_samples.empty()) {
      drawMessage(painter, plotRect, QStringLiteral("Waiting for state"));
      return;
    }

    std::vector<double> times;
    times.reserve(m_samples.size());
    const bool hasRosStamp = std::any_of(
        m_samples.begin(), m_samples.end(),
        [](const ForceHistorySample &i_sample) { return i_sample.stamp_ns > 0; });
    for (std::size_t index = 0; index < m_samples.size(); ++index) {
      const auto &sample = m_samples[index];
      if (hasRosStamp && sample.stamp_ns > 0) {
        times.push_back(static_cast<double>(sample.stamp_ns) * 1.0e-9);
      } else if (sample.received_steady_ns > 0) {
        times.push_back(static_cast<double>(sample.received_steady_ns) *
                        1.0e-9);
      } else {
        times.push_back(static_cast<double>(index));
      }
    }
    const auto minTime = *std::min_element(times.begin(), times.end());
    const auto maxTime = *std::max_element(times.begin(), times.end());
    const double timeSpan = std::max(1.0e-9, maxTime - minTime);
    const auto xFor = [&](std::size_t i_index) {
      return plotRect.left() +
             (times[i_index] - minTime) / timeSpan * plotRect.width();
    };

    double minForce = 0.0;
    double maxForce = 0.0;
    bool hasValidForce = false;
    for (const auto &sample : m_samples) {
      if (forceSampleQuality(sample) != ForceSampleQuality::Valid) {
        continue;
      }
      minForce = hasValidForce ? std::min(minForce, sample.force_n)
                               : sample.force_n;
      maxForce = hasValidForce ? std::max(maxForce, sample.force_n)
                               : sample.force_n;
      hasValidForce = true;
    }
    if (!hasValidForce) {
      const bool unavailable = std::all_of(
          m_samples.begin(), m_samples.end(), [](const ForceHistorySample &s) {
            return forceSampleQuality(s) == ForceSampleQuality::Unavailable;
          });
      drawMessage(painter, plotRect,
                  unavailable ? QStringLiteral("Measured force unavailable")
                              : QStringLiteral("No fresh valid force samples"));
      drawStateBands(painter, plotRect, xFor);
      return;
    }

    const double magnitude = std::max(1.0, std::max(std::abs(minForce),
                                                    std::abs(maxForce)));
    minForce = std::min(minForce, -magnitude * 0.12);
    maxForce = std::max(maxForce, magnitude * 0.12);
    const double forceSpan = std::max(1.0e-9, maxForce - minForce);
    const auto yFor = [&](double i_force) {
      return plotRect.bottom() - (i_force - minForce) / forceSpan *
                                    plotRect.height();
    };

    drawStateBands(painter, plotRect, xFor);

    painter.setPen(QPen(QColor(223, 228, 232, 80), 1.0, Qt::DashLine));
    const double zeroY = yFor(0.0);
    painter.drawLine(QPointF(plotRect.left(), zeroY),
                     QPointF(plotRect.right(), zeroY));
    painter.setPen(QColor(QStringLiteral("#DFE4E8")));
    painter.drawText(QRectF(1.0, plotRect.top() - 2.0, 43.0, 16.0),
                     Qt::AlignRight | Qt::AlignVCenter,
                     QString::number(maxForce, 'f', 1));
    painter.drawText(QRectF(1.0, plotRect.bottom() - 14.0, 43.0, 16.0),
                     Qt::AlignRight | Qt::AlignVCenter,
                     QString::number(minForce, 'f', 1));

    QPainterPath path;
    bool pathOpen = false;
    for (std::size_t index = 0; index < m_samples.size(); ++index) {
      const auto &sample = m_samples[index];
      const bool valid =
          forceSampleQuality(sample) == ForceSampleQuality::Valid &&
          !sample.clock_rollback_before;
      if (!valid) {
        if (pathOpen) {
          painter.setPen(QPen(QColor(QStringLiteral("#499DDA")), 2.0));
          painter.drawPath(path);
          path = QPainterPath();
          pathOpen = false;
        }
        if (forceSampleQuality(sample) == ForceSampleQuality::Stale ||
            forceSampleQuality(sample) == ForceSampleQuality::Invalid) {
          painter.setPen(QPen(QColor(223, 228, 232, 130), 1.0));
          const auto x = xFor(index);
          painter.drawLine(QPointF(x - 2.0, plotRect.center().y() - 2.0),
                           QPointF(x + 2.0, plotRect.center().y() + 2.0));
          painter.drawLine(QPointF(x - 2.0, plotRect.center().y() + 2.0),
                           QPointF(x + 2.0, plotRect.center().y() - 2.0));
        }
        continue;
      }
      const QPointF point(xFor(index), yFor(sample.force_n));
      if (!pathOpen) {
        path.moveTo(point);
        pathOpen = true;
      } else {
        path.lineTo(point);
      }
    }
    if (pathOpen) {
      painter.setPen(QPen(QColor(QStringLiteral("#499DDA")), 2.0));
      painter.drawPath(path);
    }

    drawEvents(painter, plotRect, times, xFor);
    if (!m_samples.empty()) {
      const auto latest = m_samples.back();
      const auto quality = forceSampleQuality(latest);
      painter.setPen(QColor(QStringLiteral("#DFE4E8")));
      painter.drawText(QRectF(plotRect.left() + 4.0, plotRect.top() + 3.0,
                              plotRect.width() - 8.0, 15.0),
                       Qt::AlignRight | Qt::AlignVCenter,
                       QString::fromLatin1(forceSampleQualityName(quality)));
    }
  }

private:
  template <typename XFunction>
  void drawStateBands(QPainter &io_painter, const QRectF &i_rect,
                      const XFunction &i_xFor) const {
    if (m_samples.empty()) {
      return;
    }
    io_painter.save();
    for (std::size_t index = 0; index < m_samples.size(); ++index) {
      const auto &sample = m_samples[index];
      const double right = index + 1 < m_samples.size()
                               ? i_xFor(index + 1)
                               : i_rect.right();
      const double left = i_xFor(index);
      if (sample.grip_detected && right >= left) {
        io_painter.fillRect(QRectF(left, i_rect.top(), right - left,
                                   i_rect.height()),
                            QColor(73, 157, 218, 38));
      }
    }
    io_painter.restore();
  }

  template <typename XFunction>
  void drawEvents(QPainter &io_painter, const QRectF &i_rect,
                  const std::vector<double> &i_times,
                  const XFunction &i_xFor) const {
    (void)i_times;
    if (m_samples.size() < 2) {
      return;
    }
    for (std::size_t index = 1; index < m_samples.size(); ++index) {
      const auto &previous = m_samples[index - 1];
      const auto &current = m_samples[index];
      QColor color;
      if (previous.fault_source != current.fault_source ||
          previous.fault_code != current.fault_code) {
        color = QColor(QStringLiteral("#D32F2F"));
      } else if (previous.connection_state != current.connection_state) {
        color = QColor(QStringLiteral("#D32F2F"));
      } else if (previous.active_mode != current.active_mode) {
        color = QColor(QStringLiteral("#F0A23A"));
      } else if (previous.grip_detected != current.grip_detected) {
        color = QColor(QStringLiteral("#75C878"));
      } else if (current.clock_rollback_before) {
        color = QColor(QStringLiteral("#DFE4E8"));
      } else {
        continue;
      }
      io_painter.setPen(QPen(color, 1.0, Qt::DashLine));
      const double x = i_xFor(index);
      io_painter.drawLine(QPointF(x, i_rect.top()),
                          QPointF(x, i_rect.bottom()));
    }
  }

  void drawMessage(QPainter &io_painter, const QRectF &i_rect,
                   const QString &i_message) const {
    io_painter.setPen(QColor(QStringLiteral("#DFE4E8")));
    io_painter.drawText(i_rect, Qt::AlignCenter | Qt::TextWordWrap,
                        i_message);
  }

  std::vector<ForceHistorySample> m_samples;
};

ForceHistoryPanel::ForceHistoryPanel(QWidget *i_parent)
    : rviz_common::Panel(i_parent),
      m_sharedState(std::make_shared<SharedState>()),
      m_history(std::make_unique<ForceHistoryBuffer>()) {
  setObjectName(QStringLiteral("OnRobotForceHistoryPanel"));
  applyOnRobotPanelStyle(this);

  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(4, 4, 4, 4);
  layout->setSpacing(3);
  layout->addWidget(createOnRobotPanelHeader(
      this, QStringLiteral("Force & state history"),
      QStringLiteral("Read-only measured force and device events"), 70));

  m_plot = new Plot(this);
  layout->addWidget(m_plot);

  m_legendLabel = new QLabel(
      forceHistoryLegend(std::string()),
      this);
  m_legendLabel->setWordWrap(true);
  m_legendLabel->setObjectName(QStringLiteral("forceHistoryLegend"));
  m_legendLabel->setStyleSheet(QStringLiteral("QLabel { color: #7E878E; }"));
  layout->addWidget(m_legendLabel);

  auto *controls = new QHBoxLayout();
  m_statusLabel = new QLabel(QStringLiteral("Waiting for gripper state"), this);
  m_statusLabel->setObjectName(QStringLiteral("forceHistoryStatus"));
  m_statusLabel->setWordWrap(true);
  m_statusLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  controls->addWidget(m_statusLabel, 1);
  m_clearButton = new QPushButton(QStringLiteral("Clear"), this);
  m_clearButton->setObjectName(QStringLiteral("forceHistoryClear"));
  m_clearButton->setMinimumWidth(56);
  m_clearButton->setMaximumWidth(70);
  controls->addWidget(m_clearButton);
  layout->addLayout(controls);
  layout->addStretch(1);

  m_timer = new QTimer(this);
  m_timer->setInterval(100);
  connect(m_timer, &QTimer::timeout, this, [this]() { updatePlot(); });
  connect(m_clearButton, &QPushButton::clicked, this, [this]() {
    m_history->clear();
    std::lock_guard<std::mutex> lock(m_sharedState->mutex);
    m_sharedState->pending.clear();
    m_plot->setSamples(m_history->samples());
    m_statusLabel->setText(QStringLiteral("History cleared"));
  });
  m_timer->start();
}

ForceHistoryPanel::~ForceHistoryPanel() {
  if (m_timer) {
    m_timer->stop();
  }
  m_subscription.reset();
}

void ForceHistoryPanel::bindRos() {
  m_subscription.reset();
  if (!m_node) {
    setStatus(QStringLiteral("RViz ROS node is unavailable"));
    return;
  }
  const auto state = m_sharedState;
  const auto topic = m_gripperStateTopic;
  const auto maxAge = m_maxSampleAgeS;
  m_subscription = m_node->create_subscription<
      onrobot_gripper_msgs::msg::GripperState>(
      topic, rclcpp::SensorDataQoS(),
      [state, maxAge](const onrobot_gripper_msgs::msg::GripperState::SharedPtr
                          i_message) {
        if (!i_message) {
          return;
        }
        ForceHistorySample sample;
        sample.stamp_ns = rclcpp::Time(i_message->header.stamp).nanoseconds();
        sample.received_steady_ns = steadyNowNs();
        sample.sequence = i_message->sample_sequence;
        sample.force_n = i_message->force;
        sample.force_valid = i_message->force_valid;
        sample.force_fresh = std::isfinite(messageAgeSeconds(*i_message)) &&
                             messageAgeSeconds(*i_message) >= 0.0 &&
                             messageAgeSeconds(*i_message) <= maxAge;
        sample.force_supported = supportsMeasuredForce(i_message->model);
        sample.grip_detected = i_message->grip_detected;
        sample.active_mode = i_message->active_mode;
        sample.connection_state = i_message->connection_state;
        sample.fault_source = i_message->fault_source;
        sample.fault_code = i_message->fault_code;
        std::lock_guard<std::mutex> lock(state->mutex);
        if (state->pending.size() >= 2400) {
          state->pending.erase(state->pending.begin());
        }
        state->model = i_message->model;
        state->last_received_steady_ns = sample.received_steady_ns;
        state->pending.push_back(sample);
      });
  setStatus(QStringLiteral("Listening for typed gripper state"));
}

void ForceHistoryPanel::placeInRightDock() {
  // RViz creates all custom panels in the left dock unless a saved window
  // layout says otherwise.  Keeping the history beside, rather than beneath,
  // the operator controls makes the standard 1400x900 showcase immediately
  // useful and still leaves the dock fully user-repositionable afterwards.
  QDockWidget *dock = nullptr;
  QWidget *ancestor = parentWidget();
  QMainWindow *main_window = nullptr;
  while (ancestor != nullptr) {
    if (dock == nullptr) {
      dock = qobject_cast<QDockWidget *>(ancestor);
    }
    if (main_window == nullptr) {
      main_window = qobject_cast<QMainWindow *>(ancestor);
    }
    if (dock != nullptr && main_window != nullptr) {
      break;
    }
    ancestor = ancestor->parentWidget();
  }
  if (dock == nullptr || main_window == nullptr ||
      main_window->dockWidgetArea(dock) != Qt::LeftDockWidgetArea) {
    return;
  }
  main_window->addDockWidget(Qt::RightDockWidgetArea, dock);
  dock->setMinimumWidth(300);
  dock->resize(420, std::max(240, dock->height()));
}

void ForceHistoryPanel::onInitialize() {
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    setStatus(QStringLiteral("RViz ROS node is unavailable"));
    return;
  }
  m_node = abstraction->get_raw_node();
  m_sharedState = std::make_shared<SharedState>();
  if (!m_panelSettings.readString(QStringLiteral("gripper_state_topic"),
                                  m_gripperStateTopic)) {
    m_gripperStateTopic = readOrDeclare(
        m_node, "force_history_gripper_state_topic", m_gripperStateTopic);
  }
  if (!m_panelSettings.readDouble(QStringLiteral("max_sample_age_s"),
                                  m_maxSampleAgeS)) {
    m_maxSampleAgeS = readOrDeclare(m_node, "force_history_max_sample_age_s",
                                    m_maxSampleAgeS);
  }
  if (m_gripperStateTopic.empty()) {
    m_gripperStateTopic = "gripper_state_broadcaster/state";
  }
  if (!std::isfinite(m_maxSampleAgeS) || m_maxSampleAgeS <= 0.0) {
    m_maxSampleAgeS = 0.25;
  }
  m_history = std::make_unique<ForceHistoryBuffer>();
  bindRos();
  // RViz restores the configured dock layout after plugin initialization.
  // Schedule this after that restore, otherwise a fresh config moves the
  // panel straight back below the operator dock.
  QTimer::singleShot(250, this, [this]() { placeInRightDock(); });
}

void ForceHistoryPanel::updatePlot() {
  if (!m_sharedState || !m_history || !m_plot) {
    return;
  }
  std::vector<ForceHistorySample> pending;
  std::string model;
  {
    std::lock_guard<std::mutex> lock(m_sharedState->mutex);
    pending.swap(m_sharedState->pending);
    model = m_sharedState->model;
  }
  for (auto &sample : pending) {
    m_history->append(std::move(sample));
  }
  const auto max_age_ns = static_cast<std::int64_t>(
      std::max(0.0, m_maxSampleAgeS) * 1.0e9);
  m_history->markLatestStaleIfExpired(steadyNowNs(), max_age_ns);
  m_legendLabel->setText(forceHistoryLegend(model));
  m_plot->setSamples(m_history->samples());
  if (m_history->empty()) {
    return;
  }
  const auto &latest = m_history->samples().back();
  const auto quality = forceSampleQuality(latest);
  QString message = modelName(model) + QStringLiteral(" • ") +
                    QString::number(static_cast<qlonglong>(m_history->size())) +
                    QStringLiteral(" samples • ") +
                    QString::fromLatin1(forceSampleQualityName(quality));
  if (quality == ForceSampleQuality::Unavailable) {
    message += QStringLiteral(" • measured force unavailable for this model");
  } else if (quality == ForceSampleQuality::Valid) {
    message += QStringLiteral(" • latest ") +
               QString::number(latest.force_n, 'f', 1) + QStringLiteral(" N");
  }
  m_statusLabel->setText(message);
}

void ForceHistoryPanel::setStatus(const QString &i_text) {
  if (m_statusLabel) {
    m_statusLabel->setText(i_text);
  }
}

void ForceHistoryPanel::load(const rviz_common::Config &i_config) {
  rviz_common::Panel::load(i_config);
  m_panelSettings.load(i_config);
  m_panelSettings.readString(QStringLiteral("gripper_state_topic"),
                             m_gripperStateTopic);
  m_panelSettings.readDouble(QStringLiteral("max_sample_age_s"),
                             m_maxSampleAgeS);
  if (m_node) {
    onInitialize();
  }
}

void ForceHistoryPanel::save(rviz_common::Config i_config) const {
  rviz_common::Panel::save(i_config);
  m_panelSettings.writeString(QStringLiteral("gripper_state_topic"),
                              m_gripperStateTopic);
  m_panelSettings.writeDouble(QStringLiteral("max_sample_age_s"),
                              m_maxSampleAgeS);
  m_panelSettings.save(i_config);
}

} // namespace onrobot_gripper_rviz_plugins

PLUGINLIB_EXPORT_CLASS(onrobot_gripper_rviz_plugins::ForceHistoryPanel,
                       rviz_common::Panel)

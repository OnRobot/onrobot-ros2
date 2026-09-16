#include <atomic>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <limits>
#include <iostream>
#include <signal.h>
#include <string>
#include <thread>
#include <vector>

#include <QApplication>
#include <QComboBox>
#include <QFontMetrics>
#include <QDoubleSpinBox>
#include <QDir>
#include <QFile>
#include <QGroupBox>
#include <QDockWidget>
#include <QMainWindow>
#include <QLabel>
#include <QProcess>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QTest>

#include <gtest/gtest.h>

#include <control_msgs/action/parallel_gripper_command.hpp>
#include <onrobot_gripper_msgs/msg/gripper_state.hpp>
#include <onrobot_gripper_msgs/msg/realtime_command.hpp>
#include <onrobot_gripper_msgs/msg/realtime_state.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "onrobot_gripper_rviz_plugins/gripper_control_panel.hpp"
#include "onrobot_gripper_rviz_plugins/force_history_panel.hpp"
#include "onrobot_gripper_rviz_plugins/realtime_control_panel.hpp"
#include "onrobot_gripper_rviz_plugins/three_finger_control_panel.hpp"

namespace {

class TestRosNodeAbstraction final
    : public rviz_common::ros_integration::RosNodeAbstractionIface {
public:
  explicit TestRosNodeAbstraction(rclcpp::Node::SharedPtr i_node)
      : m_node(std::move(i_node)) {}

  std::string get_node_name() const override { return m_node->get_name(); }

  std::map<std::string, std::vector<std::string>> get_topic_names_and_types()
      const override {
    return {};
  }

  rclcpp::Node::SharedPtr get_raw_node() override { return m_node; }

private:
  rclcpp::Node::SharedPtr m_node;
};

class TestDisplayContext final : public rviz_common::DisplayContext {
public:
  explicit TestDisplayContext(rclcpp::Node::SharedPtr i_node)
      : m_abstraction(std::make_shared<TestRosNodeAbstraction>(
            std::move(i_node))) {}

  Ogre::SceneManager *getSceneManager() const override { return nullptr; }
  rviz_common::WindowManagerInterface *getWindowManager() const override {
    return nullptr;
  }
  std::shared_ptr<rviz_common::interaction::SelectionManagerIface>
  getSelectionManager() const override {
    return nullptr;
  }
  std::shared_ptr<rviz_common::interaction::HandlerManagerIface>
  getHandlerManager() const override {
    return nullptr;
  }
  std::shared_ptr<rviz_common::interaction::ViewPickerIface> getViewPicker()
      const override {
    return nullptr;
  }
  rviz_common::FrameManagerIface *getFrameManager() const override {
    return nullptr;
  }
  QString getFixedFrame() const override { return {}; }
  std::uint64_t getFrameCount() const override { return 0; }
  rviz_common::DisplayFactory *getDisplayFactory() const override {
    return nullptr;
  }
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr
  getRosNodeAbstraction() const override {
    return m_abstraction;
  }
  void handleChar(QKeyEvent *, rviz_common::RenderPanel *) override {}
  void handleMouseEvent(const rviz_common::ViewportMouseEvent &) override {}
  rviz_common::ToolManager *getToolManager() const override { return nullptr; }
  rviz_common::ViewManager *getViewManager() const override { return nullptr; }
  rviz_common::transformation::TransformationManager *
  getTransformationManager() override {
    return nullptr;
  }
  rviz_common::DisplayGroup *getRootDisplayGroup() const override {
    return nullptr;
  }
  std::uint32_t getDefaultVisibilityBit() const override { return 0; }
  rviz_common::BitAllocator *visibilityBits() override { return nullptr; }
  void setStatus(const QString &) override {}
  QString getHelpPath() const override { return {}; }
  std::shared_ptr<rclcpp::Clock> getClock() override { return nullptr; }
  void lockRender() override {}
  void unlockRender() override {}
  void queueRender() override {}

private:
  std::shared_ptr<TestRosNodeAbstraction> m_abstraction;
};

class BackgroundExecutor final {
public:
  explicit BackgroundExecutor(const rclcpp::Node::SharedPtr &node)
      : executor(rclcpp::ExecutorOptions(), 2) {
    executor.add_node(node);
    worker = std::thread([this]() { executor.spin(); });
  }

  ~BackgroundExecutor() {
    executor.cancel();
    if (worker.joinable()) {
      worker.join();
    }
  }

  BackgroundExecutor(const BackgroundExecutor &) = delete;
  BackgroundExecutor &operator=(const BackgroundExecutor &) = delete;

private:
  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread worker;
};

class PanelIntegrationFixture : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!QApplication::instance()) {
      int argc = 1;
      static char appName[] = "test_panel_integration";
      static char *argv[] = {appName, nullptr};
      m_application = new QApplication(argc, argv);
    }
    if (!rclcpp::ok()) {
      int argc = 1;
      static char appName[] = "test_panel_integration_ros";
      static char *argv[] = {appName, nullptr};
      rclcpp::init(argc, argv);
      m_ownsRclcpp = true;
    }
  }

  static void TearDownTestSuite() {
    if (m_ownsRclcpp) {
      rclcpp::shutdown();
    }
    delete m_application;
    m_application = nullptr;
  }

  void SetUp() override {
    m_node = std::make_shared<rclcpp::Node>(
        "panel_integration_test_" + std::to_string(++m_nodeNumber));
    m_context = std::make_unique<TestDisplayContext>(m_node);
    m_executor.add_node(m_node);
  }

  void TearDown() override {
    m_executor.remove_node(m_node);
    m_context.reset();
    m_node.reset();
  }

  void pumpEvents(int i_iterations = 30) {
    for (int index = 0; index < i_iterations; ++index) {
      m_executor.spin_some();
      QApplication::processEvents();
      QTest::qWait(10);
    }
  }

  bool pumpUntil(const std::function<bool()> &i_condition,
                 int i_iterations = 600) {
    for (int index = 0; index < i_iterations; ++index) {
      pumpEvents(1);
      if (i_condition()) {
        return true;
      }
    }
    return false;
  }

  static rviz_common::Config realtimeConfig(
      const char *i_profile, const char *i_stateTopic, const char *i_limitsTopic,
      double i_target, double i_maxVelocity) {
    rviz_common::Config config;
    config.setType(rviz_common::Config::Map);
    auto settings = config.mapMakeChild("OnRobot panel settings");
    settings.mapSetValue("realtime_coordinate_profile", i_profile);
    settings.mapSetValue("realtime_state_topic", i_stateTopic);
    settings.mapSetValue("limits_topic", i_limitsTopic);
    settings.mapSetValue("realtime_command_topic", "/panel/command");
    settings.mapSetValue("gripper_state_topic", "/panel/safety");
    settings.mapSetValue("target_aperture_m", i_target);
    settings.mapSetValue("maximum_velocity", i_maxVelocity);
    settings.mapSetValue("position_force_n", 7.0);
    settings.mapSetValue("realtime_maximum_angular_velocity_rad_s", 0.2);
    settings.mapSetValue("realtime_maximum_position_force_n", 25.0);
    settings.mapSetValue("realtime_default_position_force_n", 8.0);
    return config;
  }

  void publishReadyState(const std::string &i_stateTopic, double i_position) {
    auto statePublisher = m_node->create_publisher<
        onrobot_gripper_msgs::msg::RealtimeState>(i_stateTopic,
                                                  rclcpp::SensorDataQoS());
    auto limitPublisher = m_node->create_publisher<
        control_msgs::msg::Float64Values>(
        "/panel/limits", rclcpp::SensorDataQoS());
    onrobot_gripper_msgs::msg::RealtimeState state;
    state.realtime_active = false;
    state.task_position_valid = true;
    state.task_position = i_position;
    state.task_velocity_valid = true;
    state.task_velocity = 0.0;
    state.mechanism_linear_position_valid = true;
    state.mechanism_linear_position = i_position;
    state.mechanism_linear_velocity_valid = true;
    state.mechanism_linear_velocity = 0.0;
    statePublisher->publish(state);
    control_msgs::msg::Float64Values limits;
    limits.values = {0.0, 0.1};
    limitPublisher->publish(limits);
    pumpEvents();
  }

  static QApplication *m_application;
  static bool m_ownsRclcpp;
  static std::uint64_t m_nodeNumber;
  rclcpp::Node::SharedPtr m_node;
  std::unique_ptr<TestDisplayContext> m_context;
  rclcpp::executors::SingleThreadedExecutor m_executor;
};

QApplication *PanelIntegrationFixture::m_application = nullptr;
bool PanelIntegrationFixture::m_ownsRclcpp = false;
std::uint64_t PanelIntegrationFixture::m_nodeNumber = 0;

QString evidencePath(const QString &i_name);

class FakeStack final {
public:
  FakeStack(const std::string &i_model, const std::string &i_namespace,
            bool i_realtime, double i_motion_speed_m_s = 0.05)
      : m_model(i_model), m_namespace(i_namespace),
        m_logPath(evidencePath(
            QStringLiteral("panel-fake-ui-%1-%2.log")
                .arg(QString::fromStdString(m_model))
                .arg(QString::fromStdString(m_namespace)))) {
    const QString command =
        QStringLiteral("exec ros2 launch onrobot_gripper_bringup gripper.launch.py "
                       "model:=%1 backend:=fake namespace:=%2 "
                       "start_realtime_controller:=%3 fake_motion_speed_m_s:=%4")
            .arg(QString::fromStdString(m_model))
            .arg(QString::fromStdString(m_namespace))
            .arg(i_realtime ? QStringLiteral("true") : QStringLiteral("false"))
            .arg(i_motion_speed_m_s, 0, 'f', 3);
    m_process.setProcessChannelMode(QProcess::MergedChannels);
    // Fast DDS can emit repeated sandbox diagnostics.  Keeping the complete
    // fake stack on a QProcess pipe without draining it eventually blocks the
    // child and makes this otherwise deterministic UI test hang.  Stream the
    // merged log directly to the evidence file instead.
    m_process.setStandardOutputFile(m_logPath, QIODevice::Truncate);
    // ros2 launch creates several children; put the complete fake stack in
    // its own process group so teardown cannot leave controllers behind.
    m_process.start(QStringLiteral("/usr/bin/setsid"),
                   {QStringLiteral("/bin/bash"), QStringLiteral("-c"),
                    command});
  }

  ~FakeStack() { stop(); }

  bool started() { return m_process.waitForStarted(5000); }

  void stop() {
    if (m_process.state() != QProcess::NotRunning) {
      const auto processGroup = static_cast<pid_t>(m_process.processId());
      if (processGroup > 0) {
        killpg(processGroup, SIGTERM);
      } else {
        m_process.terminate();
      }
      if (!m_process.waitForFinished(5000)) {
        if (processGroup > 0) {
          killpg(processGroup, SIGKILL);
        } else {
          m_process.kill();
        }
        m_process.waitForFinished(5000);
      }
    }
  }

private:
  std::string m_model;
  std::string m_namespace;
  QString m_logPath;
  QProcess m_process;
};

struct ModelCase {
  const char *name;
  double configured_maximum_aperture_m;
  double live_maximum_aperture_m;
  double minimum_aperture_m{0.0};
};

const std::vector<ModelCase> kFakeModels = {
    {"2fg7", 0.071, 0.071, 0.033},
    {"2fg14", 0.105, 0.105, 0.055},
    // The RG fake backend limits the task endpoint to the supplied CAD
    // linkage so that a stock inward fingertip cannot be represented as an
    // impossible mesh interpenetration. These are the resulting live limits.
    {"rg2", 0.110, 0.1011},
    {"rg6", 0.160, 0.150}};

rviz_common::Config standardFakeConfig(const ModelCase &i_model,
                                       const std::string &i_namespace) {
  rviz_common::Config config;
  config.setType(rviz_common::Config::Map);
  auto settings = config.mapMakeChild("OnRobot panel settings");
  const auto prefix = "/" + i_namespace;
  settings.mapSetValue("joint_name", "grip_stroke");
  settings.mapSetValue("action_name", QString::fromStdString(
      prefix + "/gripper_controller/gripper_cmd"));
  settings.mapSetValue("joint_states_topic", QString::fromStdString(
      prefix + "/joint_state_broadcaster/joint_states"));
  settings.mapSetValue("limits_topic", QString::fromStdString(
      prefix + "/parallel_gripper_limit_broadcaster/values"));
  settings.mapSetValue("gripper_state_topic", QString::fromStdString(
      prefix + "/gripper_state_broadcaster/state"));
  settings.mapSetValue("controller_manager_service", QString::fromStdString(
      prefix + "/controller_manager/list_controllers"));
  settings.mapSetValue("joint_lower_m", 0.0);
  settings.mapSetValue("joint_upper_m", i_model.configured_maximum_aperture_m);
  const std::string modelName{i_model.name};
  const bool isTwoFinger = modelName == "2fg7" || modelName == "2fg14";
  const double minimumEffort = modelName == "2fg7" ? 20.0 :
                               modelName == "2fg14" ? 40.0 : 0.0;
  const double maximumEffort = modelName == "2fg7" ? 140.0 :
                               modelName == "2fg14" ? 280.0 : 95.0;
  settings.mapSetValue("minimum_effort_n", isTwoFinger ? minimumEffort : 0.0);
  settings.mapSetValue("maximum_effort_n", maximumEffort);
  settings.mapSetValue("default_effort_n", isTwoFinger ? minimumEffort : 10.0);
  return config;
}

rviz_common::Config realtimeFakeConfig(const ModelCase &i_model,
                                       const std::string &i_namespace) {
  rviz_common::Config config;
  config.setType(rviz_common::Config::Map);
  auto settings = config.mapMakeChild("OnRobot panel settings");
  const auto prefix = "/" + i_namespace;
  settings.mapSetValue(
      "realtime_coordinate_profile",
      std::string(i_model.name).rfind("rg", 0) == 0 ? "rg" : "2fg");
  settings.mapSetValue("realtime_command_topic", QString::fromStdString(
      prefix + "/realtime_controller/command"));
  settings.mapSetValue("realtime_state_topic", QString::fromStdString(
      prefix + "/realtime_controller/state"));
  settings.mapSetValue("limits_topic", QString::fromStdString(
      prefix + "/parallel_gripper_limit_broadcaster/values"));
  settings.mapSetValue("gripper_state_topic", QString::fromStdString(
      prefix + "/gripper_state_broadcaster/state"));
  settings.mapSetValue("target_aperture_m",
                       i_model.configured_maximum_aperture_m * 0.5);
  settings.mapSetValue("maximum_velocity", 0.04);
  settings.mapSetValue("realtime_maximum_angular_velocity_rad_s", 0.2);
  settings.mapSetValue("realtime_maximum_position_force_n", 25.0);
  settings.mapSetValue("realtime_default_position_force_n", 8.0);
  return config;
}

struct JointMonitor {
  std::atomic<double> position{0.0};
  std::atomic<int> samples{0};
};

QPushButton *buttonWithText(QWidget *i_panel, const QString &i_text) {
  for (auto *button : i_panel->findChildren<QPushButton *>()) {
    if (button->text() == i_text) {
      return button;
    }
  }
  return nullptr;
}

QString evidencePath(const QString &i_name) {
  const QByteArray configured = qgetenv("ONROBOT_PANEL_TEST_EVIDENCE_DIR");
  const QString directory = configured.isEmpty()
                                ? QDir::currentPath() + "/panel-test-evidence"
                                : QString::fromUtf8(configured);
  QDir().mkpath(directory);
  return QDir(directory).filePath(i_name);
}


TEST_F(PanelIntegrationFixture, ProfileReloadResetsRgPresentationAndDisablesCommands) {
  onrobot_gripper_rviz_plugins::RealtimeControlPanel panel;
  panel.initialize(m_context.get());
  auto rg = realtimeConfig("rg", "/panel/rg_state", "/panel/limits", 0.04,
                           0.08);
  panel.load(rg);
  panel.show();
  publishReadyState("/panel/rg_state", 0.04);

  auto *positionGroup =
      panel.findChild<QGroupBox *>("realtimePositionGroup");
  auto *joystickGroup =
      panel.findChild<QGroupBox *>("realtimeJoystickGroup");
  auto *settingsGroup =
      panel.findChild<QGroupBox *>("realtimeSettingsGroup");
  auto *velocity =
      panel.findChild<QDoubleSpinBox *>("realtimeMaximumVelocity");
  auto *force = panel.findChild<QDoubleSpinBox *>("realtimePositionForce");
  auto *forceCaption =
      panel.findChild<QLabel *>("realtimePositionForceCaption");
  ASSERT_NE(positionGroup, nullptr);
  ASSERT_NE(joystickGroup, nullptr);
  ASSERT_NE(settingsGroup, nullptr);
  ASSERT_NE(velocity, nullptr);
  ASSERT_NE(force, nullptr);
  ASSERT_NE(forceCaption, nullptr);
  ASSERT_TRUE(positionGroup->isEnabled());
  ASSERT_TRUE(joystickGroup->isEnabled());
  ASSERT_TRUE(settingsGroup->isEnabled());
  ASSERT_EQ(velocity->suffix(), QStringLiteral(" rad/s"));
  ASSERT_TRUE(force->isVisible());
  ASSERT_TRUE(forceCaption->isVisible());

  auto twoFg = realtimeConfig("2fg", "/panel/twofg_state", "/panel/limits",
                              0.04, 0.12);
  panel.load(twoFg);
  EXPECT_FALSE(positionGroup->isEnabled());
  EXPECT_FALSE(joystickGroup->isEnabled());
  EXPECT_FALSE(settingsGroup->isEnabled());
  EXPECT_EQ(velocity->suffix(), QStringLiteral(" m/s"));
  EXPECT_FALSE(force->isVisible());
  EXPECT_FALSE(forceCaption->isVisible());
  EXPECT_NEAR(velocity->value(), 0.12, 1e-6);
}

TEST_F(PanelIntegrationFixture, FreshLiveLimitsClampSavedTargetWithoutPublishing) {
  auto commandCount = std::make_shared<std::atomic<int>>(0);
  auto commandSubscription = m_node->create_subscription<
      onrobot_gripper_msgs::msg::RealtimeCommand>(
      "/panel/command", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [commandCount](const onrobot_gripper_msgs::msg::RealtimeCommand::SharedPtr) {
        commandCount->fetch_add(1, std::memory_order_relaxed);
      });

  onrobot_gripper_rviz_plugins::RealtimeControlPanel panel;
  auto config = realtimeConfig("2fg", "/panel/state", "/panel/limits", 0.017,
                               0.06);
  panel.load(config);
  panel.initialize(m_context.get());
  panel.show();
  auto statePublisher = m_node->create_publisher<
      onrobot_gripper_msgs::msg::RealtimeState>("/panel/state",
                                                rclcpp::SensorDataQoS());
  auto limitPublisher = m_node->create_publisher<
      control_msgs::msg::Float64Values>("/panel/limits", rclcpp::SensorDataQoS());
  onrobot_gripper_msgs::msg::RealtimeState state;
  state.task_position_valid = true;
  state.task_position = 0.006;
  state.mechanism_linear_position_valid = true;
  state.mechanism_linear_position = 0.006;
  statePublisher->publish(state);
  control_msgs::msg::Float64Values limits;
  limits.values = {0.002, 0.010};
  limitPublisher->publish(limits);
  pumpEvents();

  auto *target = panel.findChild<QDoubleSpinBox *>("realtimeTargetAperture");
  ASSERT_NE(target, nullptr);
  EXPECT_NEAR(target->value(), 0.010, 1e-6);
  EXPECT_EQ(commandCount->load(std::memory_order_relaxed), 0);
  (void)commandSubscription;
}

TEST_F(PanelIntegrationFixture,
       ForceHistoryPanelRendersMeasuredAndUnavailableFamilies) {
  const std::vector<std::string> models = {"2fg7", "2fg14", "rg2", "rg6"};
  for (const auto &model : models) {
    const auto topic = "/panel/force_history/model_" + model;
    auto publisher = m_node->create_publisher<
        onrobot_gripper_msgs::msg::GripperState>(topic,
                                                 rclcpp::SensorDataQoS());
    onrobot_gripper_rviz_plugins::ForceHistoryPanel panel;
    panel.initialize(m_context.get());
    rviz_common::Config config;
    config.setType(rviz_common::Config::Map);
    auto settings = config.mapMakeChild("OnRobot panel settings");
    settings.mapSetValue("gripper_state_topic",
                         QString::fromStdString(topic));
    settings.mapSetValue("max_sample_age_s", 0.5);
    panel.load(config);
    panel.resize(520, 360);
    panel.show();

    onrobot_gripper_msgs::msg::GripperState state;
    state.header.stamp = m_node->now();
    state.model = model;
    state.sample_sequence = 1;
    state.sample_age.sec = 0;
    state.sample_age.nanosec = 0;
    state.force_valid = model == "2fg7" || model == "2fg14";
    state.force = -59.0;
    state.grip_detected = true;
    state.active_mode = 1;
    state.connection_state = 3;
    publisher->publish(state);
    ASSERT_TRUE(pumpUntil([&panel, model] {
      auto *label = panel.findChild<QLabel *>("forceHistoryStatus");
      if (label == nullptr) {
        return false;
      }
      const auto text = label->text();
      return model == "rg2" || model == "rg6"
                 ? text.contains("unavailable")
                 : text.contains("valid");
    })) << model << " force-history status did not update";
    auto *legend = panel.findChild<QLabel *>("forceHistoryLegend");
    ASSERT_NE(legend, nullptr);
    if (model == "rg2" || model == "rg6") {
      EXPECT_TRUE(legend->text().contains("unavailable"))
          << model << ": " << legend->text().toStdString();
      EXPECT_FALSE(legend->text().contains("valid measured force"))
          << model << ": " << legend->text().toStdString();
    } else {
      EXPECT_TRUE(legend->text().contains("valid measured force"))
          << model << ": " << legend->text().toStdString();
    }

    // A real state transition and signed-force sample must remain visible in
    // the rendered panel; the RG path must still be labeled unavailable.
    state.sample_sequence = 2;
    state.header.stamp = m_node->now();
    state.grip_detected = false;
    state.active_mode = 2;
    state.force = -12.0;
    publisher->publish(state);
    pumpEvents(10);
    ASSERT_TRUE(panel.grab().save(evidencePath(
        QStringLiteral("panel_force_history_%1.png")
            .arg(QString::fromStdString(model)))));

    if (model == "2fg7" || model == "2fg14") {
      // An over-range but finite device reading is diagnostic evidence, not
      // a value to clip to the command/rated limit. A subsequent fault must
      // make it unavailable, not leave that spike displayed as live force.
      state.sample_sequence++;
      state.header.stamp = m_node->now();
      state.force = -270.0;
      publisher->publish(state);
      ASSERT_TRUE(pumpUntil([&panel] {
        return panel.findChild<QLabel *>("forceHistoryStatus")->text().contains("-270");
      }));
      ASSERT_TRUE(panel.grab().save(evidencePath(
          QStringLiteral("panel_force_spike_%1.png").arg(QString::fromStdString(model)))));
      state.sample_sequence++;
      state.force_valid = false;
      state.connection_state = 4;
      publisher->publish(state);
      ASSERT_TRUE(pumpUntil([&panel] {
        return panel.findChild<QLabel *>("forceHistoryStatus")->text().contains("invalid");
      }));
      state.sample_sequence++;
      state.force_valid = true;
      state.connection_state = 3;
      state.force = -59.0;
      publisher->publish(state);
      ASSERT_TRUE(pumpUntil([&panel] {
        return panel.findChild<QLabel *>("forceHistoryStatus")->text().contains("-59");
      }));
    }
  }
}

TEST_F(PanelIntegrationFixture, ForceGripUsesActualPressReleaseAndLiveCapability) {
  using Command = onrobot_gripper_msgs::msg::RealtimeCommand;
  using GS = onrobot_gripper_msgs::msg::GripperState;
  for (const std::string model : {"2fg7", "2fg14"}) {
    SCOPED_TRACE(model);
    auto states = m_node->create_publisher<onrobot_gripper_msgs::msg::RealtimeState>(
        "/panel/state", rclcpp::SensorDataQoS());
    auto limits = m_node->create_publisher<control_msgs::msg::Float64Values>(
        "/panel/limits", rclcpp::SensorDataQoS());
    auto typed = m_node->create_publisher<GS>("/panel/gripper_state",
                                            rclcpp::SensorDataQoS());
    std::vector<Command> commands;
    auto received = m_node->create_subscription<Command>("/panel/command",
        rclcpp::QoS(1).reliable(), [&](Command::SharedPtr message) {
          commands.push_back(*message);
        });
    onrobot_gripper_msgs::msg::RealtimeState state;
    state.task_position_valid = true;
    state.task_position = 0.065;
    state.force_valid = true;
    state.force = -56.2;
    GS identity;
    identity.model = model;
    identity.task_aperture_valid = true;
    identity.task_aperture = state.task_position;
    identity.mapping_validity = GS::MAPPING_VALID;
    identity.connection_state = GS::CONNECTION_IDLE;
    identity.force_valid = true;
    bool publishTyped = true;
    auto refresh = m_node->create_wall_timer(std::chrono::milliseconds(20), [&] {
      states->publish(state);
      control_msgs::msg::Float64Values range;
      range.values = {0.033, 0.071};
      limits->publish(range);
      if (publishTyped) { ++identity.sample_sequence; typed->publish(identity); }
    });
    onrobot_gripper_rviz_plugins::RealtimeControlPanel panel;
    panel.initialize(m_context.get());
    auto config = realtimeConfig("2fg", "/panel/state", "/panel/limits", 0.040, 0.01);
    config.mapGetChild("OnRobot panel settings").mapSetValue(
        "gripper_state_topic", "/panel/gripper_state");
    panel.load(config);
    panel.resize(420, 980);
    panel.show();
    auto *hold = panel.findChild<QPushButton *>("realtimeHoldGripButton");
    auto *release = panel.findChild<QPushButton *>("realtimeReleaseGripButton");
    auto *force = panel.findChild<QDoubleSpinBox *>("realtimeGripForce");
    auto *approach = panel.findChild<QComboBox *>("realtimeForceApproach");
    auto *target = panel.findChild<QDoubleSpinBox *>("realtimeTargetAperture");
    auto *speed = panel.findChild<QDoubleSpinBox *>("realtimeMaximumVelocity");
    ASSERT_NE(hold, nullptr); ASSERT_NE(release, nullptr);
    pumpEvents(30);
    EXPECT_FALSE(hold->isEnabled());
    EXPECT_TRUE(commands.empty());
    identity.realtime_force_control_available = true;
    ASSERT_TRUE(pumpUntil([&] { return hold->isEnabled(); }));
    EXPECT_DOUBLE_EQ(force->minimum(), model == "2fg7" ? 30.0 : 40.0);
    EXPECT_DOUBLE_EQ(force->maximum(), model == "2fg7" ? 95.0 : 196.0);
    target->setValue(0.040);
    speed->setValue(0.01);
    force->setValue(50.0);
    QTest::mousePress(hold, Qt::LeftButton);
    ASSERT_TRUE(pumpUntil([&] {
      return !commands.empty() && commands.back().mode == Command::FORCE_POSITION;
    }));
    EXPECT_DOUBLE_EQ(commands.back().force, 50.0);
    EXPECT_DOUBLE_EQ(commands.back().task_position, 0.040);
    EXPECT_DOUBLE_EQ(commands.back().task_velocity, 0.01);
    // Endpoint arrival isn't contact; continue the held force command, not Stop.
    state.task_position = 0.040;
    identity.task_aperture = 0.040;
    identity.grip_detected = true;
    pumpEvents(30);
    ASSERT_EQ(commands.back().mode, Command::FORCE_POSITION);
    auto *reportedForce = panel.findChild<QLabel *>("realtimeForceValue");
    ASSERT_NE(reportedForce, nullptr);
    EXPECT_EQ(reportedForce->text(), "-56.2");
    ASSERT_TRUE(panel.grab().save(evidencePath(
        QString::fromStdString("force-controls-" + model + ".png"))));
    QTest::mouseRelease(hold, Qt::LeftButton);
    ASSERT_TRUE(pumpUntil([&] { return commands.back().mode == Command::STOP; }));
    const auto stopped = commands.size();
    pumpEvents(20);
    EXPECT_EQ(commands.size(), stopped);
    QTest::mouseClick(release, Qt::LeftButton);
    ASSERT_TRUE(pumpUntil([&] { return commands.back().mode == Command::POSITION; }));
    EXPECT_DOUBLE_EQ(commands.back().force, 0.0);
    EXPECT_DOUBLE_EQ(commands.back().task_position, 0.071);
    approach->setCurrentIndex(1);
    QTest::mousePress(hold, Qt::LeftButton);
    ASSERT_TRUE(pumpUntil([&] { return commands.back().mode == Command::FORCE_VELOCITY; }));
    EXPECT_DOUBLE_EQ(commands.back().task_velocity, -0.01);
    EXPECT_DOUBLE_EQ(commands.back().force, 50.0);
    publishTyped = false;
    ASSERT_TRUE(pumpUntil([&] { return commands.back().mode == Command::STOP; }, 1000));
    ASSERT_TRUE(pumpUntil([&] { return !hold->isEnabled(); }, 200));
    QTest::mouseRelease(hold, Qt::LeftButton);
    publishTyped = true;
    ASSERT_TRUE(pumpUntil([&] { return hold->isEnabled(); }));
    const auto reconnected = commands.size();
    pumpEvents(20);
    EXPECT_EQ(commands.size(), reconnected); // reconnect never replays.
    QTest::mousePress(hold, Qt::LeftButton);
    ASSERT_TRUE(pumpUntil([&] { return commands.back().mode == Command::FORCE_VELOCITY; }));
    target->setFocus(Qt::OtherFocusReason);
    ASSERT_TRUE(pumpUntil([&] { return commands.back().mode == Command::STOP; }));
    QTest::mouseRelease(hold, Qt::LeftButton);
    // Dock/window deactivation is not necessarily delivered to the focused
    // child button. Losing the application itself must retire a held command.
    QTest::mousePress(hold, Qt::LeftButton);
    ASSERT_TRUE(pumpUntil([&] { return commands.back().mode == Command::FORCE_VELOCITY; }));
    QEvent deactivate(QEvent::ApplicationDeactivate);
    QApplication::sendEvent(QApplication::instance(), &deactivate);
    ASSERT_TRUE(pumpUntil([&] { return commands.back().mode == Command::STOP; }, 50));
    QTest::mouseRelease(hold, Qt::LeftButton);
    QTest::mousePress(hold, Qt::LeftButton);
    ASSERT_TRUE(pumpUntil([&] { return commands.back().mode == Command::FORCE_VELOCITY; }));
    panel.hide();
    ASSERT_TRUE(pumpUntil([&] { return commands.back().mode == Command::STOP; }, 50));
    QTest::mouseRelease(hold, Qt::LeftButton);
    panel.show();
    identity.model = "rg2";
    ASSERT_TRUE(pumpUntil([&] { return !hold->isEnabled(); }));
  }
}

TEST_F(PanelIntegrationFixture, ConventionalDefaultsUseGripperFeedbackNotRobotJointStates) {
  // Use the shipped defaults, not a test-specific panel route. The general
  // joint_states topic belongs to another robot in this workcell.
  m_executor.remove_node(m_node);
  m_node = std::make_shared<rclcpp::Node>("panel", "/panel_feedback_routing");
  m_context = std::make_unique<TestDisplayContext>(m_node);
  m_executor.add_node(m_node);
  m_node->declare_parameter("minimum_effort_n", 20.0);
  m_node->declare_parameter("default_effort_n", 20.0);

  using Action = control_msgs::action::ParallelGripperCommand;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  sensor_msgs::msg::JointState gripper;
  gripper.name = {"grip_stroke", "finger_stroke"};
  gripper.position = {0.0329, 0.0};
  gripper.velocity = {0.0, 0.0};
  gripper.effort = {std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN()};
  // Optional unavailable force must not block motion.
  std::vector<Action::Goal> goals;
  auto action = rclcpp_action::create_server<Action>(
      m_node, "gripper_controller/gripper_cmd",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<Handle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [&](std::shared_ptr<Handle> handle) {
        goals.push_back(*handle->get_goal());
        gripper.position[0] = goals.back().command.position.at(0);
        auto result = std::make_shared<Action::Result>();
        result->state.position = {gripper.position[0]};
        handle->succeed(result);
      });
  auto raw = m_node->create_publisher<sensor_msgs::msg::JointState>(
      "joint_state_broadcaster/joint_states", rclcpp::SensorDataQoS());
  auto robot = m_node->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS());
  auto limits = m_node->create_publisher<control_msgs::msg::Float64Values>(
      "parallel_gripper_limit_broadcaster/values", rclcpp::SensorDataQoS());
  sensor_msgs::msg::JointState arm;
  arm.name = {"other_robot_shoulder_joint", "other_robot_wrist_joint"};
  arm.position = {-0.328, -3.299};
  arm.effort = {-0.096, 0.634};
  control_msgs::msg::Float64Values bounds;
  bounds.values = {0.033, 0.071};
  const auto publish = [&] {
    raw->publish(gripper);
    robot->publish(arm);
    limits->publish(bounds);
  };

  onrobot_gripper_rviz_plugins::GripperControlPanel panel;
  panel.initialize(m_context.get());
  panel.resize(420, 540);
  panel.show();
  auto *open = buttonWithText(&panel, "Open");
  auto *close = buttonWithText(&panel, "Close");
  auto *send = buttonWithText(&panel, "Send target");
  auto *target = panel.findChild<QDoubleSpinBox *>("conventionalTargetAperture");
  ASSERT_NE(open, nullptr);
  ASSERT_NE(close, nullptr);
  ASSERT_NE(send, nullptr);
  ASSERT_NE(target, nullptr);
  ASSERT_TRUE(pumpUntil([&] { publish(); return open->isEnabled(); }))
      << "Fresh gripper feedback must enable the default panel even when "
         "joint_states contains only robot-arm joints";
  QTest::mouseClick(open, Qt::LeftButton);
  ASSERT_TRUE(pumpUntil([&] { publish(); return goals.size() == 1; }));
  EXPECT_NEAR(goals.back().command.position.at(0), 0.071, 1e-9);
  EXPECT_DOUBLE_EQ(goals.back().command.effort.at(0), 20.0);
  QTest::mouseClick(close, Qt::LeftButton);
  ASSERT_TRUE(pumpUntil([&] { publish(); return goals.size() == 2; }));
  EXPECT_NEAR(goals.back().command.position.at(0), 0.033, 1e-9);
  target->setValue(0.052);
  QTest::mouseClick(send, Qt::LeftButton);
  ASSERT_TRUE(pumpUntil([&] { publish(); return goals.size() == 3; }));
  EXPECT_NEAR(goals.back().command.position.at(0), 0.052, 1e-9);
  pumpEvents(15);
  EXPECT_TRUE(panel.grab().save(evidencePath("conventional_scoped_feedback.png")));

  // Raw state is not cleaned for visualization: invalid task positions must
  // revoke readiness, never be displayed as a number or used as a command.
  gripper.position[0] = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(pumpUntil([&] { publish(); return !send->isEnabled(); }));
  QTest::mouseClick(send, Qt::LeftButton);
  pumpEvents(5);
  EXPECT_EQ(goals.size(), 3u);
  gripper.position[0] = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 15; ++i) { publish(); pumpEvents(1); }
  EXPECT_FALSE(send->isEnabled());
  gripper.position[0] = 0.052;
  ASSERT_TRUE(pumpUntil([&] { publish(); return send->isEnabled(); }));
  gripper.position.clear();
  ASSERT_TRUE(pumpUntil([&] { publish(); return !send->isEnabled(); }));
  gripper.position = {0.052, 0.0095};
  ASSERT_TRUE(pumpUntil([&] { publish(); return send->isEnabled(); }));
  // Unrelated robot updates cannot renew the gripper's freshness.
  ASSERT_TRUE(pumpUntil([&] { robot->publish(arm); return !send->isEnabled(); }, 250));
  ASSERT_TRUE(pumpUntil([&] { publish(); return send->isEnabled(); }));

  // Explicit per-panel routing remains supported, but a rebind must not
  // retain readiness from the previous stream.
  rviz_common::Config custom;
  custom.mapMakeChild("OnRobot panel settings").mapSetValue(
      "joint_states_topic", "custom_feedback");
  panel.load(custom);
  EXPECT_FALSE(send->isEnabled());
  for (int i = 0; i < 15; ++i) { publish(); pumpEvents(1); }
  EXPECT_FALSE(send->isEnabled());
  auto remapped = m_node->create_publisher<sensor_msgs::msg::JointState>(
      "custom_feedback", rclcpp::SensorDataQoS());
  ASSERT_TRUE(pumpUntil([&] {
    remapped->publish(gripper);
    limits->publish(bounds);
    return send->isEnabled();
  }));
  QTest::mouseClick(send, Qt::LeftButton);
  ASSERT_TRUE(pumpUntil([&] { return goals.size() == 4; }));
}

TEST_F(PanelIntegrationFixture,
       ConventionalSpeedLostResponseReconcilesAndIgnoresLateReply) {
  using Action = control_msgs::action::ParallelGripperCommand;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  using GetParameters = rcl_interfaces::srv::GetParameters;
  using SetParametersAtomically = rcl_interfaces::srv::SetParametersAtomically;

  auto serviceNode = std::make_shared<rclcpp::Node>("speed_service_peer");
  BackgroundExecutor serviceExecutor(serviceNode);
  std::atomic<int> controllerSpeed{50};
  std::atomic<bool> applyRequestedValue{true};
  std::atomic<int> setRequests{0};
  std::atomic<int> setResponses{0};
  std::atomic<int> getRequests{0};
  std::atomic<bool> malformedGetRequest{false};
  std::atomic<bool> malformedSetRequest{false};

  const auto handleGetParameters =
      [&](const std::shared_ptr<GetParameters::Request> request,
          std::shared_ptr<GetParameters::Response> response) {
        getRequests.fetch_add(1);
        if (request->names.size() != 2u) {
          malformedGetRequest.store(true);
        }
        rcl_interfaces::msg::ParameterValue capability;
        capability.type =
            rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        capability.bool_value = true;
        rcl_interfaces::msg::ParameterValue speed;
        speed.type =
            rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        speed.integer_value = controllerSpeed.load();
        response->values = {capability, speed};
      };
  auto getService = serviceNode->create_service<GetParameters>(
      "/timeout_controller/get_parameters", handleGetParameters);
  auto setService = serviceNode->create_service<SetParametersAtomically>(
      "/timeout_controller/set_parameters_atomically",
      [&](const std::shared_ptr<SetParametersAtomically::Request> request,
          std::shared_ptr<SetParametersAtomically::Response> response) {
        setRequests.fetch_add(1);
        const bool valid_request =
            request->parameters.size() == 1u &&
            request->parameters.front().name ==
                "conventional_speed_percent" &&
            request->parameters.front().value.type ==
                rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        if (!valid_request) {
          malformedSetRequest.store(true);
        } else if (applyRequestedValue.load()) {
          controllerSpeed.store(static_cast<int>(
              request->parameters.front().value.integer_value));
        }
        // Simulate a controller that consumed the request but whose response
        // was delayed beyond the panel deadline. A second executor thread
        // remains available for the recovery GetParameters request.
        std::this_thread::sleep_for(std::chrono::milliseconds(3400));
        response->result.successful = true;
        setResponses.fetch_add(1);
      });

  std::vector<Action::Goal> goals;
  auto action = rclcpp_action::create_server<Action>(
      m_node, "/timeout_controller/gripper_cmd",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<Handle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [&](std::shared_ptr<Handle> handle) {
        goals.push_back(*handle->get_goal());
        auto result = std::make_shared<Action::Result>();
        result->state.position = handle->get_goal()->command.position;
        handle->succeed(result);
      });
  auto joints = m_node->create_publisher<sensor_msgs::msg::JointState>(
      "/timeout/joint_states", rclcpp::SensorDataQoS());
  auto limits = m_node->create_publisher<control_msgs::msg::Float64Values>(
      "/timeout/limits", rclcpp::SensorDataQoS());
  sensor_msgs::msg::JointState joint;
  joint.name = {"grip_stroke"};
  joint.position = {0.052};
  control_msgs::msg::Float64Values bounds;
  bounds.values = {0.033, 0.071};
  const auto publishState = [&] {
    joints->publish(joint);
    limits->publish(bounds);
  };

  rviz_common::Config config;
  config.setType(rviz_common::Config::Map);
  auto settings = config.mapMakeChild("OnRobot panel settings");
  settings.mapSetValue("action_name", "/timeout_controller/gripper_cmd");
  settings.mapSetValue("joint_states_topic", "/timeout/joint_states");
  settings.mapSetValue("limits_topic", "/timeout/limits");
  settings.mapSetValue("minimum_effort_n", 20.0);
  settings.mapSetValue("maximum_effort_n", 140.0);
  settings.mapSetValue("default_effort_n", 20.0);

  onrobot_gripper_rviz_plugins::GripperControlPanel panel;
  panel.load(config);
  panel.initialize(m_context.get());
  panel.resize(460, 650);
  panel.show();
  auto *open = buttonWithText(&panel, "Open");
  auto *speed = panel.findChild<QSpinBox *>("conventionalSpeedPercent");
  auto *apply = panel.findChild<QPushButton *>("applyConventionalSpeed");
  auto *current = panel.findChild<QLabel *>("conventionalSpeedCurrent");
  auto *status = panel.findChild<QLabel *>("conventionalSpeedStatus");
  ASSERT_NE(open, nullptr);
  ASSERT_NE(speed, nullptr);
  ASSERT_NE(apply, nullptr);
  ASSERT_NE(current, nullptr);
  ASSERT_NE(status, nullptr);
  ASSERT_TRUE(pumpUntil([&] {
    publishState();
    return open->isEnabled() && current->text().contains("50%");
  }, 300));

  speed->setValue(75);
  ASSERT_TRUE(pumpUntil([&] { publishState(); return apply->isEnabled(); }, 100));
  QTest::mouseClick(apply, Qt::LeftButton);
  ASSERT_TRUE(pumpUntil([&] {
    publishState();
    return setRequests.load() == 1 && !open->isEnabled();
  }, 100));
  EXPECT_TRUE(goals.empty());
  const int readsBeforeTimeout = getRequests.load();
  ASSERT_TRUE(pumpUntil([&] {
    publishState();
    return status->text().contains("Applied") &&
           current->text().contains("75%") && open->isEnabled();
  }, 500)) << "lost SetParameters response did not reconcile by readback; status='"
           << status->text().toStdString() << "'";
  EXPECT_GT(getRequests.load(), readsBeforeTimeout);
  EXPECT_TRUE(goals.empty());
  ASSERT_TRUE(pumpUntil([&] { return setResponses.load() == 1; }, 150));
  pumpEvents(30);
  EXPECT_FALSE(status->text().contains("accepted"));
  EXPECT_FALSE(status->text().contains("checking"));
  EXPECT_TRUE(current->text().contains("75%"));
  EXPECT_TRUE(open->isEnabled());

  // A second delayed write remains unapplied. Readback must settle the
  // transaction as a mismatch using the real controller value, and the late
  // successful service response must not overwrite that outcome.
  applyRequestedValue.store(false);
  speed->setValue(35);
  ASSERT_TRUE(pumpUntil([&] { publishState(); return apply->isEnabled(); }, 100));
  QTest::mouseClick(apply, Qt::LeftButton);
  ASSERT_TRUE(pumpUntil([&] {
    publishState();
    return setRequests.load() == 2 && !open->isEnabled();
  }, 100));
  ASSERT_TRUE(pumpUntil([&] {
    publishState();
    return status->text().contains("controller reports 75%") &&
           current->text().contains("75%") && open->isEnabled();
  }, 500)) << "unapplied delayed request did not reconcile as mismatch; status='"
           << status->text().toStdString() << "'";
  ASSERT_TRUE(pumpUntil([&] { return setResponses.load() == 2; }, 150));
  pumpEvents(30);
  EXPECT_TRUE(status->text().contains("controller reports 75%"));
  EXPECT_TRUE(current->text().contains("75%"));
  EXPECT_TRUE(open->isEnabled());
  EXPECT_TRUE(goals.empty());

  // If readback disappears as well, the panel must remain fail-safe and avoid
  // a tight retry loop. It may resume only after a valid controller value is
  // available again.
  getService.reset();
  pumpEvents(30);
  applyRequestedValue.store(true);
  speed->setValue(60);
  ASSERT_TRUE(pumpUntil([&] { publishState(); return apply->isEnabled(); }, 100));
  const int readsBeforeUnavailable = getRequests.load();
  QTest::mouseClick(apply, Qt::LeftButton);
  ASSERT_TRUE(pumpUntil([&] {
    publishState();
    return setRequests.load() == 3 && !open->isEnabled();
  }, 100));
  ASSERT_TRUE(pumpUntil([&] {
    publishState();
    return setResponses.load() == 3;
  }, 500));
  pumpEvents(50);
  EXPECT_FALSE(open->isEnabled());
  EXPECT_TRUE(goals.empty());
  EXPECT_EQ(getRequests.load(), readsBeforeUnavailable);
  EXPECT_TRUE(status->text().contains("checking") ||
              status->text().contains("readback"));
  getService = serviceNode->create_service<GetParameters>(
      "/timeout_controller/get_parameters", handleGetParameters);
  ASSERT_TRUE(pumpUntil([&] {
    publishState();
    return current->text().contains("60%") && open->isEnabled();
  }, 500));
  EXPECT_FALSE(malformedGetRequest.load());
  EXPECT_FALSE(malformedSetRequest.load());

  (void)getService;
  (void)setService;
  (void)action;
}

TEST_F(PanelIntegrationFixture, ActualQtInputMovesEveryFakeModelThroughControllers) {
  const char *onlyModel = std::getenv("ONROBOT_PANEL_TEST_ONLY_MODEL");
  const auto screenshotSafety = [this](const std::string &i_topic,
                                        QWidget *i_panel,
                                        const QString &i_path) {
    auto publisher = m_node->create_publisher<
        onrobot_gripper_msgs::msg::GripperState>(i_topic,
                                                 rclcpp::SensorDataQoS());
    onrobot_gripper_msgs::msg::GripperState safety;
    safety.safety_status_valid = true;
    safety.safety_1_pushed = true;
    safety.safety_1_triggered = true;
    safety.safety_2_pushed = true;
    safety.safety_2_triggered = true;
    safety.safety_dc_error = true;
    for (int index = 0; index < 30; ++index) {
      publisher->publish(safety);
      pumpEvents(1);
    }
    i_panel->resize(280, i_panel->sizeHint().height());
    i_panel->show();
    QApplication::processEvents();
    ASSERT_TRUE(i_panel->grab().save(i_path));
  };

  for (const auto &model : kFakeModels) {
    if (onlyModel != nullptr && model.name != std::string(onlyModel)) {
      continue;
    }
    const std::string name = model.name;
    const std::string standardNamespace = "panel_ui_standard_" + name;
    FakeStack stack(name, standardNamespace, false);
    ASSERT_TRUE(stack.started()) << name;

    auto monitor = std::make_shared<JointMonitor>();
    auto jointSubscription = m_node->create_subscription<sensor_msgs::msg::JointState>(
        "/" + standardNamespace + "/joint_states", rclcpp::SensorDataQoS(),
        [monitor](const sensor_msgs::msg::JointState::SharedPtr i_message) {
          for (std::size_t index = 0; index < i_message->name.size(); ++index) {
            if (i_message->name[index] == "grip_stroke" &&
                index < i_message->position.size()) {
              monitor->position.store(i_message->position[index]);
              monitor->samples.fetch_add(1);
              return;
            }
          }
        });

    onrobot_gripper_rviz_plugins::GripperControlPanel panel;
    panel.initialize(m_context.get());
    panel.load(standardFakeConfig(model, standardNamespace));
    panel.resize(460, 650);
    panel.show();
    auto *closeButton = buttonWithText(&panel, QStringLiteral("Close"));
    auto *openButton = buttonWithText(&panel, QStringLiteral("Open"));
    auto *sendButton = buttonWithText(&panel, QStringLiteral("Send target"));
    auto *target =
        panel.findChild<QDoubleSpinBox *>("conventionalTargetAperture");
    auto *effort = panel.findChild<QDoubleSpinBox *>("conventionalEffort");
    ASSERT_NE(closeButton, nullptr);
    ASSERT_NE(openButton, nullptr);
    ASSERT_NE(sendButton, nullptr);
    ASSERT_NE(target, nullptr);
    ASSERT_NE(effort, nullptr);
    ASSERT_TRUE(pumpUntil([&] {
      return monitor->samples.load() > 0 && closeButton->isEnabled();
    }, 8000)) << name;
    if (name == "2fg7") {
      EXPECT_DOUBLE_EQ(effort->minimum(), 20.0);
      EXPECT_DOUBLE_EQ(effort->value(), 20.0);
    } else if (name == "2fg14") {
      EXPECT_DOUBLE_EQ(effort->minimum(), 40.0);
      EXPECT_DOUBLE_EQ(effort->value(), 40.0);
    }

    QTest::mouseClick(closeButton, Qt::LeftButton);
    ASSERT_TRUE(pumpUntil([&] {
      return monitor->position.load() <= model.minimum_aperture_m + 0.002;
    })) << name << " close position=" << monitor->position.load();
    QTest::mouseClick(openButton, Qt::LeftButton);
    ASSERT_TRUE(pumpUntil([&] {
      return monitor->position.load() >= model.live_maximum_aperture_m - 0.002;
    })) << name << " open position=" << monitor->position.load();

    const double midpoint = (model.minimum_aperture_m + model.live_maximum_aperture_m) * 0.5;
    target->setValue(midpoint);
    QTest::mouseClick(sendButton, Qt::LeftButton);
    ASSERT_TRUE(pumpUntil([&] {
      return std::abs(monitor->position.load() -
                      midpoint) <=
             0.003;
    })) << name << " target position=" << monitor->position.load();

    auto *speedGroup =
        panel.findChild<QGroupBox *>("conventionalSpeedGroup");
    auto *speedSpinBox = panel.findChild<QSpinBox *>("conventionalSpeedPercent");
    auto *speedApply = panel.findChild<QPushButton *>("applyConventionalSpeed");
    auto *speedCurrent =
        panel.findChild<QLabel *>("conventionalSpeedCurrent");
    auto *speedStatus = panel.findChild<QLabel *>("conventionalSpeedStatus");
    ASSERT_NE(speedGroup, nullptr);
    ASSERT_NE(speedSpinBox, nullptr);
    ASSERT_NE(speedApply, nullptr);
    ASSERT_NE(speedCurrent, nullptr);
    ASSERT_NE(speedStatus, nullptr);
    if (name == "2fg7" || name == "2fg14") {
      ASSERT_TRUE(pumpUntil([&] {
        return speedGroup->isVisible() &&
               speedCurrent->text().contains("50%");
      }, 1000)) << name << " speed setting was not discovered";
      EXPECT_EQ(speedSpinBox->minimum(), 1);
      EXPECT_EQ(speedSpinBox->maximum(), 100);
      EXPECT_EQ(speedSpinBox->value(), 50);
      EXPECT_FALSE(speedApply->isEnabled());

      onrobot_gripper_rviz_plugins::GripperControlPanel secondPanel;
      secondPanel.initialize(m_context.get());
      secondPanel.load(standardFakeConfig(model, standardNamespace));
      secondPanel.resize(460, 650);
      secondPanel.show();
      auto *secondSpeedGroup =
          secondPanel.findChild<QGroupBox *>("conventionalSpeedGroup");
      auto *secondSpeedSpinBox =
          secondPanel.findChild<QSpinBox *>("conventionalSpeedPercent");
      auto *secondSpeedCurrent =
          secondPanel.findChild<QLabel *>("conventionalSpeedCurrent");
      auto *secondSpeedStatus =
          secondPanel.findChild<QLabel *>("conventionalSpeedStatus");
      ASSERT_NE(secondSpeedGroup, nullptr);
      ASSERT_NE(secondSpeedSpinBox, nullptr);
      ASSERT_NE(secondSpeedCurrent, nullptr);
      ASSERT_NE(secondSpeedStatus, nullptr);
      ASSERT_TRUE(pumpUntil([&] {
        return secondSpeedGroup->isVisible() &&
               secondSpeedCurrent->text().contains("50%");
      }, 1000)) << name << " second panel did not read the controller setting";

      const double positionBeforeSpeedChange = monitor->position.load();
      speedSpinBox->setValue(75);
      ASSERT_TRUE(pumpUntil([&] { return speedApply->isEnabled(); }, 1000))
          << name << " Apply did not enable for a changed speed";
      QTest::mouseClick(speedApply, Qt::LeftButton);
      QTest::qWait(150);
      EXPECT_TRUE(speedStatus->text().contains("Applying"))
          << name << " did not show the pending parameter update";
      EXPECT_FALSE(openButton->isEnabled())
          << name << " allowed motion while the speed update was pending";
      EXPECT_NEAR(monitor->position.load(), positionBeforeSpeedChange, 0.0001)
          << "a pending speed change must not move the gripper";
      ASSERT_TRUE(pumpUntil([&] {
        return speedStatus->text().contains("Applied") &&
               speedCurrent->text().contains("75%") &&
               secondSpeedCurrent->text().contains("75%") &&
               secondSpeedSpinBox->value() == 75;
      }, 1000)) << name << " speed update did not apply and synchronize";
      EXPECT_NEAR(monitor->position.load(), positionBeforeSpeedChange, 0.0001)
          << "changing speed must not move the gripper";
      ASSERT_TRUE(panel.grab().save(evidencePath(
          QStringLiteral("conventional-speed-%1.png")
              .arg(QString::fromStdString(name)))));

      secondSpeedSpinBox->setValue(35);
      auto *secondApply =
          secondPanel.findChild<QPushButton *>("applyConventionalSpeed");
      ASSERT_NE(secondApply, nullptr);
      ASSERT_TRUE(pumpUntil([&] { return secondApply->isEnabled(); }, 1000))
          << name << " second panel Apply did not enable for a changed speed";

      QTest::mouseClick(secondApply, Qt::LeftButton);
      ASSERT_TRUE(pumpUntil([&] {
        return secondSpeedStatus->text().contains("Applied") &&
               secondSpeedCurrent->text().contains("35%") &&
               speedCurrent->text().contains("35%") &&
               speedSpinBox->value() == 35;
      }, 1000)) << name << " first panel did not follow the second panel";

      if (name == "2fg7") {
        // The deterministic fake velocity hook is currently a 2FG7 fixture
        // option. Use it to hold a controller goal active long enough to
        // exercise the user-visible busy/rejected parameter response.
        const std::string busyNamespace = standardNamespace + "_busy_speed";
        FakeStack slowStack(name, busyNamespace, false, 0.001);
        ASSERT_TRUE(slowStack.started()) << name << " busy-test fake stack failed";
        onrobot_gripper_rviz_plugins::GripperControlPanel busyPanel;
        busyPanel.initialize(m_context.get());
        busyPanel.load(standardFakeConfig(model, busyNamespace));
        busyPanel.resize(460, 650);
        busyPanel.show();
        auto *busySpeedGroup =
            busyPanel.findChild<QGroupBox *>("conventionalSpeedGroup");
        auto *busySpeedSpinBox =
            busyPanel.findChild<QSpinBox *>("conventionalSpeedPercent");
        auto *busySpeedApply =
            busyPanel.findChild<QPushButton *>("applyConventionalSpeed");
        auto *busySpeedStatus =
            busyPanel.findChild<QLabel *>("conventionalSpeedStatus");
        ASSERT_NE(busySpeedGroup, nullptr);
        ASSERT_NE(busySpeedSpinBox, nullptr);
        ASSERT_NE(busySpeedApply, nullptr);
        ASSERT_NE(busySpeedStatus, nullptr);
        ASSERT_TRUE(pumpUntil([&] {
          return busySpeedGroup->isVisible() &&
                 !busySpeedStatus->text().contains("Checking");
        }, 1000)) << name << " busy-test speed support was not discovered";
        auto busyMonitor = std::make_shared<JointMonitor>();
        auto busyJointSubscription = m_node->create_subscription<
            sensor_msgs::msg::JointState>(
            "/" + busyNamespace + "/joint_state_broadcaster/joint_states",
            rclcpp::SensorDataQoS(),
            [busyMonitor](const sensor_msgs::msg::JointState::SharedPtr message) {
              for (std::size_t index = 0; index < message->name.size(); ++index) {
                if (message->name[index] == "grip_stroke" &&
                    index < message->position.size() &&
                    std::isfinite(message->position[index])) {
                  busyMonitor->position.store(message->position[index]);
                  busyMonitor->samples.fetch_add(1);
                  return;
                }
              }
            });
        ASSERT_TRUE(pumpUntil([&] { return busyMonitor->samples.load() > 0; },
                              1000))
            << name << " busy-test fake state was not received";
        const double initialBusyPosition = busyMonitor->position.load();
        using GripperAction = control_msgs::action::ParallelGripperCommand;
        const auto busyActionClient =
            rclcpp_action::create_client<GripperAction>(
                m_node, "/" + busyNamespace +
                            "/gripper_controller/gripper_cmd");
        ASSERT_TRUE(pumpUntil(
            [&] { return busyActionClient->action_server_is_ready(); }, 1000))
            << name << " busy-test conventional action endpoint did not become ready";
        GripperAction::Goal busyGoal;
        busyGoal.command.name = {"grip_stroke"};
        const double busyTarget =
            std::abs(initialBusyPosition - model.minimum_aperture_m) >
                    std::abs(initialBusyPosition - model.live_maximum_aperture_m)
                ? model.minimum_aperture_m
                : model.live_maximum_aperture_m;
        busyGoal.command.position = {busyTarget};
        busyGoal.command.effort = {20.0};
        auto busyGoalResponse = busyActionClient->async_send_goal(busyGoal);
        ASSERT_TRUE(pumpUntil([&] {
          return busyGoalResponse.wait_for(std::chrono::milliseconds(0)) ==
                 std::future_status::ready;
        }, 1000)) << name << " busy-test motion was not admitted";
        ASSERT_NE(busyGoalResponse.get(), nullptr)
            << name << " busy-test motion was rejected";
        ASSERT_TRUE(pumpUntil([&] {
          return std::abs(busyMonitor->position.load() - initialBusyPosition) >
                 0.0001;
        }, 1000)) << name << " busy-test goal did not start moving";
        busySpeedSpinBox->setValue(60);
        ASSERT_TRUE(pumpUntil([&] { return busySpeedApply->isEnabled(); }, 1000))
            << name << " busy-test Apply did not enable";
        QTest::mouseClick(busySpeedApply, Qt::LeftButton);
        ASSERT_TRUE(pumpUntil([&] {
          return busySpeedStatus->text().startsWith("Rejected:");
        }, 1000)) << name << " did not display controller rejection while moving; status='"
                   << busySpeedStatus->text().toStdString() << "'";
        busyJointSubscription.reset();
      }
    } else {
      ASSERT_TRUE(pumpUntil([&] { return speedGroup->isHidden(); }, 1000))
          << name << " must not advertise 2FG conventional speed";
    }

    screenshotSafety(
        "/" + standardNamespace + "/gripper_state_broadcaster/state", &panel,
        evidencePath(QStringLiteral("panel_fake_ui_standard_%1.png")
                         .arg(QString::fromStdString(name))));
    jointSubscription.reset();
  }

  for (const auto &model : kFakeModels) {
    if (onlyModel != nullptr && model.name != std::string(onlyModel)) {
      continue;
    }
    const std::string name = model.name;
    const std::string realtimeNamespace = "panel_ui_realtime_" + name;
    FakeStack stack(name, realtimeNamespace, true);
    ASSERT_TRUE(stack.started()) << name;

    auto monitor = std::make_shared<JointMonitor>();
    auto jointSubscription = m_node->create_subscription<sensor_msgs::msg::JointState>(
        "/" + realtimeNamespace + "/joint_states", rclcpp::SensorDataQoS(),
        [monitor](const sensor_msgs::msg::JointState::SharedPtr i_message) {
          for (std::size_t index = 0; index < i_message->name.size(); ++index) {
            if (i_message->name[index] == "grip_stroke" &&
                index < i_message->position.size()) {
              monitor->position.store(i_message->position[index]);
              monitor->samples.fetch_add(1);
              return;
            }
          }
        });
    std::atomic<int> stopCount{0};
    auto commandSubscription = m_node->create_subscription<
        onrobot_gripper_msgs::msg::RealtimeCommand>(
        "/" + realtimeNamespace + "/realtime_controller/command",
        rclcpp::SensorDataQoS(),
        [&stopCount](const onrobot_gripper_msgs::msg::RealtimeCommand::SharedPtr
                         i_message) {
          if (i_message->mode ==
              onrobot_gripper_msgs::msg::RealtimeCommand::STOP) {
            stopCount.fetch_add(1);
          }
        });

    onrobot_gripper_rviz_plugins::RealtimeControlPanel panel;
    panel.initialize(m_context.get());
    panel.load(realtimeFakeConfig(model, realtimeNamespace));
    panel.resize(460, 700);
    panel.show();
    auto *sendButton = panel.findChild<QPushButton *>(
        "realtimeSendTargetButton");
    auto *joystick = panel.findChild<QSlider *>("realtimeVelocityJoystick");
    auto *target = panel.findChild<QDoubleSpinBox *>("realtimeTargetAperture");
    auto *centerButton = buttonWithText(&panel, QStringLiteral("Center / stop"));
    ASSERT_NE(sendButton, nullptr);
    ASSERT_NE(joystick, nullptr);
    ASSERT_NE(target, nullptr);
    ASSERT_NE(centerButton, nullptr);
    ASSERT_TRUE(pumpUntil([&] {
      return monitor->samples.load() > 0 && sendButton->isEnabled();
    }, 8000)) << name;

    target->setFocus();
    QTest::keyClick(target, Qt::Key_A, Qt::ControlModifier);
    const double midpoint = (model.minimum_aperture_m + model.live_maximum_aperture_m) * 0.5;
    QTest::keyClicks(target, QStringLiteral("%1").arg(midpoint));
    QTest::keyClick(target, Qt::Key_Return);
    QTest::mouseClick(sendButton, Qt::LeftButton);
    ASSERT_TRUE(pumpUntil([&] {
      return std::abs(monitor->position.load() -
                      midpoint) <=
             0.003;
    })) << name << " RT target position=" << monitor->position.load();

    joystick->resize(220, 40);
    const double beforeVelocity = monitor->position.load();
    QTest::mousePress(joystick, Qt::LeftButton, Qt::NoModifier,
                      QPoint(joystick->width() * 3 / 4, joystick->height() / 2));
    ASSERT_TRUE(pumpUntil([&] {
      return monitor->position.load() < beforeVelocity - 0.0001;
    }, 200)) << name << " RT velocity did not move";
    const double afterVelocity = monitor->position.load();
    QTest::mouseRelease(joystick, Qt::LeftButton, Qt::NoModifier,
                        QPoint(joystick->width() * 3 / 4, joystick->height() / 2));
    ASSERT_TRUE(pumpUntil([&] {
      return std::abs(monitor->position.load() - afterVelocity) <= 0.003;
    }, 80)) << name << " RT release did not stop";
    const int stopsBeforeFocusLoss = stopCount.load();

    // A real focus transfer while the joystick is held must issue the same
    // Stop fence as release; this is intentionally an actual Qt focus event.
    joystick->setFocusPolicy(Qt::StrongFocus);
    joystick->setFocus(Qt::OtherFocusReason);
    ASSERT_TRUE(joystick->hasFocus()) << name << " joystick did not take focus";
    QTest::mousePress(joystick, Qt::LeftButton, Qt::NoModifier,
                      QPoint(joystick->width() * 3 / 5, joystick->height() / 2));
    ASSERT_TRUE(pumpUntil([&] {
      return monitor->position.load() < afterVelocity - 0.0001;
    }, 50)) << name << " RT focus-loss setup did not move";
    const double beforeFocusLoss = monitor->position.load();
    QPushButton focusSink(&panel);
    focusSink.setFocusPolicy(Qt::StrongFocus);
    focusSink.show();
    focusSink.setFocus(Qt::OtherFocusReason);
    QApplication::processEvents();
    ASSERT_TRUE(focusSink.hasFocus()) << name << " focus transfer failed";
    ASSERT_FALSE(joystick->hasFocus()) << name << " joystick retained focus";
    ASSERT_TRUE(pumpUntil([&] {
      return stopCount.load() > stopsBeforeFocusLoss;
    }, 80)) << name << " RT focus loss did not publish Stop";
    ASSERT_TRUE(pumpUntil([&] {
      return std::abs(monitor->position.load() - beforeFocusLoss) <= 0.003;
    }, 80)) << name << " RT focus loss did not stop";
    QTest::mouseRelease(joystick, Qt::LeftButton, Qt::NoModifier,
                        QPoint(joystick->width() * 3 / 5, joystick->height() / 2));
    QTest::mouseClick(centerButton, Qt::LeftButton);
    focusSink.hide();
    ASSERT_TRUE(pumpUntil([&] {
      return std::abs(monitor->position.load() - beforeFocusLoss) <= 0.003;
    }, 80));

    auto *forceGroup = panel.findChild<QGroupBox *>("realtimeForceGroup");
    ASSERT_NE(forceGroup, nullptr);
    if (name == "2fg7" || name == "2fg14") {
      auto *hold = panel.findChild<QPushButton *>("realtimeHoldGripButton");
      auto *approach = panel.findChild<QComboBox *>("realtimeForceApproach");
      auto *force = panel.findChild<QDoubleSpinBox *>("realtimeGripForce");
      auto *speed = panel.findChild<QDoubleSpinBox *>("realtimeMaximumVelocity");
      ASSERT_TRUE(pumpUntil([&] { return hold->isEnabled(); }));
      speed->setValue(0.01);
      force->setValue(50.0);
      target->setValue(model.minimum_aperture_m);
      for (int kind : {0, 1}) {
        approach->setCurrentIndex(kind);
        const double before = monitor->position.load();
        const int previousStops = stopCount.load();
        QTest::mousePress(hold, Qt::LeftButton);
        ASSERT_TRUE(pumpUntil([&] { return monitor->position.load() < before - 0.001; }, 1000));
        QTest::mouseRelease(hold, Qt::LeftButton);
        ASSERT_TRUE(pumpUntil([&] { return stopCount.load() > previousStops; }));
        pumpEvents(10);
        const double stoppedAt = monitor->position.load();
        pumpEvents(20);
        EXPECT_NEAR(monitor->position.load(), stoppedAt, 0.0001);
        EXPECT_EQ(panel.findChild<QLabel *>("realtimeForceValue")->text(), QString::fromUtf8("\u2014"));
      }
      panel.resize(420, 980);
      pumpEvents(10);
      ASSERT_TRUE(panel.grab().save(evidencePath(QString::fromStdString(
          "force-fake-" + name + ".png"))));
    } else {
      EXPECT_FALSE(forceGroup->isVisible());
    }

    screenshotSafety(
        "/" + realtimeNamespace + "/gripper_state_broadcaster/state", &panel,
        evidencePath(QStringLiteral("panel_fake_ui_realtime_%1.png")
                         .arg(QString::fromStdString(name))));
    QApplication::processEvents();
    auto *measuredLabel = panel.findChild<QLabel *>("realtimeMeasuredLabel");
    auto *statusLabel = panel.findChild<QLabel *>("realtimeStatusLabel");
    auto *safetyLabel = panel.findChild<QLabel *>("realtimeSafetyLabel");
    ASSERT_NE(measuredLabel, nullptr);
    ASSERT_NE(statusLabel, nullptr);
    ASSERT_NE(safetyLabel, nullptr);
    ASSERT_FALSE(measuredLabel->geometry().intersects(statusLabel->geometry()))
        << name << " measured/status rows overlap";
    const auto renderedHeight = [](const QLabel *label) {
      const QRect renderRect(0, 0, label->width(), 10000);
      return label->fontMetrics().boundingRect(
          renderRect, Qt::TextWordWrap, label->text()).height();
    };
    ASSERT_LE(renderedHeight(measuredLabel) + 2, measuredLabel->height())
        << name << " measured text exceeds its allocated row";
    ASSERT_LE(renderedHeight(statusLabel) + 2, statusLabel->height())
        << name << " status text exceeds its allocated row";
    ASSERT_LE(renderedHeight(safetyLabel) + 2, safetyLabel->height())
        << name << " safety text exceeds its allocated row";
    commandSubscription.reset();
    jointSubscription.reset();
  }
}

TEST_F(PanelIntegrationFixture, ThreeFingerFeedbackValidityAndClicks) {
  using State = onrobot_gripper_msgs::msg::GripperState;
  onrobot_gripper_rviz_plugins::ThreeFingerControlPanel panel;
  rviz_common::Config config;
  auto settings = config.mapMakeChild("OnRobot panel settings");
  settings.mapSetValue("joint_states_topic", "/three_test/joints");
  settings.mapSetValue("gripper_state_topic", "/three_test/state");
  settings.mapSetValue("limits_topic", "/three_test/limits");
  settings.mapSetValue("three_finger_command_topic", "/three_test/command");
  panel.load(config);
  panel.initialize(m_context.get());
  panel.resize(360, 560);
  panel.show();
  auto joints = m_node->create_publisher<sensor_msgs::msg::JointState>(
      "/three_test/joints", rclcpp::SensorDataQoS());
  auto typed = m_node->create_publisher<State>("/three_test/state", rclcpp::SensorDataQoS());
  auto limits = m_node->create_publisher<control_msgs::msg::Float64Values>(
      "/three_test/limits", rclcpp::SensorDataQoS());
  std::vector<double> commands;
  auto receiver = m_node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/three_test/command", 1,
      [&](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        commands.push_back(msg->data.at(0));
      });
  auto *measured = panel.findChild<QLabel *>("threeFingerMeasured");
  auto *connection = panel.findChild<QLabel *>("threeFingerConnection");
  ASSERT_NE(measured, nullptr);
  ASSERT_NE(connection, nullptr);
  QPushButton *send = nullptr;
  for (auto *button : panel.findChildren<QPushButton *>())
    if (button->text() == "70 mm") send = button;
  ASSERT_NE(send, nullptr);
  sensor_msgs::msg::JointState joint;
  joint.name = {"grip_diameter"};
  joint.position = {0.0723};
  State state;
  state.connection_state = State::CONNECTION_IDLE;
  state.task_aperture_valid = true;
  control_msgs::msg::Float64Values bounds;
  bounds.values = {0.019, 0.139};
  auto publish = [&] { joints->publish(joint); typed->publish(state); limits->publish(bounds); };
  ASSERT_TRUE(pumpUntil([&] { publish(); return send->isEnabled(); }));
  EXPECT_TRUE(measured->text().contains("72.3"));
  EXPECT_TRUE(measured->text().contains("velocity unavailable"));
  EXPECT_TRUE(measured->text().contains("joint unavailable"));
  EXPECT_EQ(connection->text(), "Connected: idle");
  QTest::mouseClick(send, Qt::LeftButton);
  ASSERT_TRUE(pumpUntil([&] { return !commands.empty(); }));
  EXPECT_DOUBLE_EQ(commands.back(), 0.070);

  joint.position = {std::numeric_limits<double>::quiet_NaN()};
  ASSERT_TRUE(pumpUntil([&] { publish(); return !send->isEnabled(); }));
  EXPECT_TRUE(measured->text().contains("unavailable"));
  EXPECT_FALSE(measured->text().contains("nan"));
  QTest::mouseClick(send, Qt::LeftButton);
  pumpEvents(5);
  EXPECT_EQ(commands.size(), 1u);

  joint.position = {0.073};
  joint.velocity = {std::numeric_limits<double>::infinity()};
  ASSERT_TRUE(pumpUntil([&] { publish(); return send->isEnabled(); }));
  EXPECT_TRUE(measured->text().contains("velocity unavailable"));
  // Other joints arriving must not keep an old diameter fresh.
  joint.name = {"finger_angle"};
  joint.position = {0.76};
  ASSERT_TRUE(pumpUntil([&] { publish(); return !send->isEnabled(); }, 250));
  EXPECT_TRUE(measured->text().contains("stale"));

  joint.name = {"grip_diameter"};
  joint.position = {0.074};
  ASSERT_TRUE(pumpUntil([&] { publish(); return send->isEnabled(); }));
  state.sample_age.sec = 2;
  ASSERT_TRUE(pumpUntil([&] { publish(); return !send->isEnabled(); }));
  EXPECT_TRUE(connection->text().contains("CONNECTION LOST"));
  state.sample_age.sec = 0;
  state.connection_state = State::CONNECTION_FAULTED;
  state.fault_code = 17;
  ASSERT_TRUE(pumpUntil([&] { publish(); return connection->text().contains("17"); }));
  EXPECT_FALSE(send->isEnabled());
  state.connection_state = State::CONNECTION_IDLE;
  state.fault_code = 0;
  ASSERT_TRUE(pumpUntil([&] { publish(); return send->isEnabled(); }));
  // Reloading a panel must discard the previous device's limits and readings.
  settings.mapSetValue("joint_states_topic", "/other_device/joints");
  panel.load(config);
  pumpEvents(15);
  EXPECT_FALSE(send->isEnabled());
  EXPECT_FALSE(measured->text().contains("74.0"));
}

// Explicit opt-in for a cleared physical 3FG fixture already running through
// the native ROS showcase. No device connection or configuration is made here.
TEST_F(PanelIntegrationFixture, ThreeFingerLivePanelClickAndCapture) {
  const auto *prefix = std::getenv("ONROBOT_THREE_FINGER_LIVE_NAMESPACE");
  const auto *output = std::getenv("ONROBOT_THREE_FINGER_EVIDENCE_DIR");
  if (!prefix || !output) GTEST_SKIP() << "Live fixture is not selected";
  const std::string ns(prefix);
  ASSERT_EQ(ns, "/n36_live");
  QMainWindow window;
  auto *dock = new QDockWidget("3FG Control", &window);
  auto *panel = new onrobot_gripper_rviz_plugins::ThreeFingerControlPanel(dock);
  dock->setWidget(panel);
  window.addDockWidget(Qt::LeftDockWidgetArea, dock);
  rviz_common::Config config;
  auto settings = config.mapMakeChild("OnRobot panel settings");
  for (const auto &entry : std::vector<std::pair<std::string, std::string>>{
           {"joint_states_topic", "joint_states"},
           {"gripper_state_topic", "gripper_state_broadcaster/state"},
           {"limits_topic", "three_finger_limit_broadcaster/values"},
           {"three_finger_command_topic", "diameter_controller/commands"}})
    settings.mapSetValue(QString::fromStdString(entry.first),
                         QString::fromStdString(ns + "/" + entry.second));
  panel->load(config);
  panel->initialize(m_context.get());
  window.resize(440, 580);
  window.show();
  double diameter = NAN, minimum = NAN, maximum = NAN;
  auto state = m_node->create_subscription<sensor_msgs::msg::JointState>(
      ns + "/joint_states", rclcpp::SensorDataQoS(),
      [&](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
          if (msg->name[i] == "grip_diameter") diameter = msg->position[i];
      });
  auto limits = m_node->create_subscription<control_msgs::msg::Float64Values>(
      ns + "/three_finger_limit_broadcaster/values", rclcpp::SensorDataQoS(),
      [&](control_msgs::msg::Float64Values::ConstSharedPtr msg) {
        if (msg->values.size() == 2) { minimum = msg->values[0]; maximum = msg->values[1]; }
      });
  auto *target = panel->findChild<QDoubleSpinBox *>();
  auto *measured = panel->findChild<QLabel *>("threeFingerMeasured");
  auto *connection = panel->findChild<QLabel *>("threeFingerConnection");
  QPushButton *send = nullptr;
  for (auto *button : panel->findChildren<QPushButton *>())
    if (button->text() == "Send target") send = button;
  ASSERT_NE(send, nullptr);
  ASSERT_TRUE(pumpUntil([&] { return send->isEnabled() && std::isfinite(diameter) &&
                                   std::isfinite(minimum) && std::isfinite(maximum); }, 1500));
  const double initial = diameter;
  const double destination = std::min(maximum - 0.001, std::max(minimum, initial) + 0.005);
  ASSERT_GT(destination, initial + 0.002);
  target->setFocus();
  target->selectAll();
  QTest::keyClicks(target, QString::number(destination * 1000.0, 'f', 1));
  QTest::keyClick(target, Qt::Key_Return);
  QTest::mouseClick(send, Qt::LeftButton);
  ASSERT_TRUE(pumpUntil([&] { return std::abs(diameter - destination) < 0.001; }, 1200));
  EXPECT_GT(diameter - initial, 0.002);
  EXPECT_TRUE(connection->text().startsWith("Connected"));
  EXPECT_GE(measured->width(), 180);
  EXPECT_GE(measured->height(), measured->heightForWidth(measured->width()));
  QDir().mkpath(output);
  ASSERT_TRUE(window.grab().save(QString::fromUtf8(output) + "/live-panel.png"));
  std::cout << "physical diameter before=" << initial << " after=" << diameter << std::endl;
  // Return to a nearby valid diameter through the same actual UI path.
  target->setValue(std::max(minimum, initial) * 1000.0);
  QTest::mouseClick(send, Qt::LeftButton);
  EXPECT_TRUE(pumpUntil([&] { return std::abs(diameter - std::max(minimum, initial)) < 0.001; }, 1200));
}

} // namespace

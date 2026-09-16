#include <QApplication>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QFormLayout>
#include <QStyleFactory>
#include <QProxyStyle>
#include <QLayout>
#include <QPoint>
#include <QSlider>
#include <QTimer>
#include <QtTest/QTest>

#include <gtest/gtest.h>

#include "onrobot_gripper_rviz_plugins/gripper_control_panel.hpp"
#include "onrobot_gripper_rviz_plugins/force_history_panel.hpp"
#include "onrobot_gripper_rviz_plugins/realtime_control_panel.hpp"
#include "onrobot_gripper_rviz_plugins/three_finger_control_panel.hpp"

namespace {

class QtApplication {
public:
  QtApplication() {
    if (!QApplication::instance()) {
      int argc = 1;
      static char applicationName[] = "test_panel_configuration";
      static char *argv[] = {applicationName, nullptr};
      m_application = new QApplication(argc, argv);
    }
  }

  ~QtApplication() {
    if (m_application) {
      delete m_application;
    }
  }

private:
  QApplication *m_application{nullptr};
};

TEST(PanelConfiguration, ConventionalSettingsSurvivePanelSaveAndLoad) {
  QtApplication application;
  rviz_common::Config source;
  source.setType(rviz_common::Config::Map);
  auto settings = source.mapMakeChild("OnRobot panel settings");
  settings.mapSetValue("action_name", "/left/gripper_controller/gripper_cmd");
  settings.mapSetValue("limits_topic", "/left/limits");
  settings.mapSetValue("gripper_state_topic", "/left/state");

  onrobot_gripper_rviz_plugins::GripperControlPanel panel;
  panel.load(source);
  rviz_common::Config saved;
  panel.save(saved);

  auto savedSettings = saved.mapGetChild("OnRobot panel settings");
  QString action;
  QString limits;
  ASSERT_TRUE(savedSettings.mapGetString("action_name", &action));
  ASSERT_TRUE(savedSettings.mapGetString("limits_topic", &limits));
  EXPECT_EQ(action, QStringLiteral("/left/gripper_controller/gripper_cmd"));
  EXPECT_EQ(limits, QStringLiteral("/left/limits"));
}

TEST(PanelConfiguration, DefaultsFollowTheRvizNamespace) {
  QtApplication application;

  onrobot_gripper_rviz_plugins::GripperControlPanel conventional;
  rviz_common::Config conventionalSaved;
  conventional.save(conventionalSaved);
  auto conventionalSettings =
      conventionalSaved.mapGetChild("OnRobot panel settings");
  QString action;
  QString jointStates;
  ASSERT_TRUE(conventionalSettings.mapGetString("action_name", &action));
  ASSERT_TRUE(
      conventionalSettings.mapGetString("joint_states_topic", &jointStates));
  EXPECT_EQ(action, QStringLiteral("gripper_controller/gripper_cmd"));
  EXPECT_EQ(jointStates, QStringLiteral("joint_state_broadcaster/joint_states"));

  onrobot_gripper_rviz_plugins::RealtimeControlPanel realtime;
  rviz_common::Config realtimeSaved;
  realtime.save(realtimeSaved);
  auto realtimeSettings = realtimeSaved.mapGetChild("OnRobot panel settings");
  QString command;
  ASSERT_TRUE(
      realtimeSettings.mapGetString("realtime_command_topic", &command));
  EXPECT_EQ(command, QStringLiteral("realtime_controller/command"));

  onrobot_gripper_rviz_plugins::ForceHistoryPanel history;
  rviz_common::Config historySaved;
  history.save(historySaved);
  auto historySettings = historySaved.mapGetChild("OnRobot panel settings");
  QString state;
  ASSERT_TRUE(
      historySettings.mapGetString("gripper_state_topic", &state));
  EXPECT_EQ(state, QStringLiteral("gripper_state_broadcaster/state"));
}

TEST(PanelConfiguration, ConventionalLiveStateRemainsVisibleInShortDock) {
  QtApplication application;
  onrobot_gripper_rviz_plugins::GripperControlPanel panel;
  panel.ensurePolished();

  auto *measured = panel.findChild<QLabel *>("conventionalMeasuredState");
  auto *status = panel.findChild<QLabel *>("conventionalActionStatus");
  auto *safety = panel.findChild<QLabel *>("conventionalSafetyStatus");
  ASSERT_NE(measured, nullptr);
  ASSERT_NE(status, nullptr);
  ASSERT_NE(safety, nullptr);

  auto *timer = panel.findChild<QTimer *>();
  ASSERT_NE(timer, nullptr);
  timer->stop();
  measured->setText(QStringLiteral("0.0514 m"));
  status->setText(QStringLiteral("Goal succeeded"));
  panel.resize(340, 360);
  panel.layout()->setGeometry(panel.rect());

  EXPECT_LE(panel.minimumWidth(), 340);
  for (auto *label : {measured, status}) {
    EXPECT_FALSE(label->isHidden());
    EXPECT_EQ(label->sizePolicy().horizontalPolicy(), QSizePolicy::Preferred);
    EXPECT_GE(label->minimumWidth(), 180);
    EXPECT_GT(label->sizeHint().width(), 0);
    EXPECT_GT(label->width(), 0);
    EXPECT_GT(label->height(), 0);
    const QPoint bottom_right = label->mapTo(
        &panel, QPoint(label->width() - 1, label->height() - 1));
    EXPECT_TRUE(panel.contentsRect().contains(bottom_right));
  }
  EXPECT_EQ(safety->sizePolicy().horizontalPolicy(), QSizePolicy::Preferred);
  EXPECT_GE(safety->minimumWidth(), 180);
}

TEST(PanelConfiguration, RealtimeJoystickUsesActualPointerPressAndRelease) {
  QtApplication application;
  onrobot_gripper_rviz_plugins::RealtimeControlPanel panel;
  auto *joystick =
      panel.findChild<QSlider *>("realtimeVelocityJoystick");
  ASSERT_NE(joystick, nullptr);
  ASSERT_NE(joystick->parentWidget(), nullptr);
  joystick->parentWidget()->setEnabled(true);
  joystick->resize(220, 40);
  joystick->show();
  QApplication::processEvents();

  QTest::mousePress(joystick, Qt::LeftButton, Qt::NoModifier,
                    QPoint(joystick->width() - 2, joystick->height() / 2));
  EXPECT_GE(joystick->value(), 80);
  QTest::mouseRelease(joystick, Qt::LeftButton, Qt::NoModifier,
                      QPoint(joystick->width() - 2, joystick->height() / 2));
  EXPECT_EQ(joystick->value(), 0);
}

TEST(PanelConfiguration, ConventionalAndRealtimeUseTheSameApertureInput) {
  QtApplication application;
  onrobot_gripper_rviz_plugins::GripperControlPanel conventional;
  onrobot_gripper_rviz_plugins::RealtimeControlPanel realtime;

  auto *conventionalTarget = conventional.findChild<QDoubleSpinBox *>(
      "conventionalTargetAperture");
  auto *realtimeTarget =
      realtime.findChild<QDoubleSpinBox *>("realtimeTargetAperture");
  ASSERT_NE(conventionalTarget, nullptr);
  ASSERT_NE(realtimeTarget, nullptr);
  EXPECT_EQ(conventionalTarget->decimals(), realtimeTarget->decimals());
  EXPECT_DOUBLE_EQ(conventionalTarget->singleStep(),
                   realtimeTarget->singleStep());
  EXPECT_EQ(conventionalTarget->suffix(), realtimeTarget->suffix());
}

TEST(PanelConfiguration, ThreeFingerDefaultsAreRelativeAndPersistPerPanel) {
  QtApplication application;
  onrobot_gripper_rviz_plugins::ThreeFingerControlPanel panel;
  rviz_common::Config saved;
  panel.save(saved);

  auto settings = saved.mapGetChild("OnRobot panel settings");
  QString command;
  QString joints;
  QString limits;
  QString state;
  ASSERT_TRUE(settings.mapGetString("three_finger_command_topic", &command));
  ASSERT_TRUE(settings.mapGetString("joint_states_topic", &joints));
  ASSERT_TRUE(settings.mapGetString("limits_topic", &limits));
  ASSERT_TRUE(settings.mapGetString("gripper_state_topic", &state));
  EXPECT_EQ(command, QStringLiteral("diameter_controller/commands"));
  EXPECT_EQ(joints, QStringLiteral("joint_states"));
  EXPECT_EQ(limits,
            QStringLiteral("three_finger_limit_broadcaster/values"));
  EXPECT_EQ(state, QStringLiteral("gripper_state_broadcaster/state"));

  rviz_common::Config configured;
  auto configuredSettings = configured.mapMakeChild("OnRobot panel settings");
  configuredSettings.mapSetValue("joint_states_topic", "/fixture/joint_states");
  configuredSettings.mapSetValue(
      "gripper_state_topic", "/fixture/gripper_state_broadcaster/state");
  onrobot_gripper_rviz_plugins::ThreeFingerControlPanel restored;
  restored.load(configured);
  rviz_common::Config restoredConfig;
  restored.save(restoredConfig);
  auto restoredSettings =
      restoredConfig.mapGetChild("OnRobot panel settings");
  ASSERT_TRUE(restoredSettings.mapGetString("joint_states_topic", &joints));
  ASSERT_TRUE(restoredSettings.mapGetString("gripper_state_topic", &state));
  EXPECT_EQ(joints, QStringLiteral("/fixture/joint_states"));
  EXPECT_EQ(state,
            QStringLiteral("/fixture/gripper_state_broadcaster/state"));
}

TEST(PanelConfiguration, ThreeFingerFeedbackFitsACompactDock) {
  QtApplication application;
  // Some desktop styles leave form fields at their size hint. Ignored labels
  // then get zero width, even though their text and subscriptions are valid.
  class CompactFormStyle : public QProxyStyle {
  public:
    CompactFormStyle() : QProxyStyle(QStyleFactory::create("Fusion")) {}
    int styleHint(StyleHint hint, const QStyleOption *option = nullptr,
                  const QWidget *widget = nullptr,
                  QStyleHintReturn *result = nullptr) const override {
      if (hint == SH_FormLayoutFieldGrowthPolicy)
        return QFormLayout::FieldsStayAtSizeHint;
      return QProxyStyle::styleHint(hint, option, widget, result);
    }
  };
  QApplication::setStyle(new CompactFormStyle);
  onrobot_gripper_rviz_plugins::ThreeFingerControlPanel panel;
  for (auto *timer : panel.findChildren<QTimer *>()) {
    timer->stop();
  }
  for (auto *form : panel.findChildren<QFormLayout *>()) {
    for (int row = 0; row < form->rowCount(); ++row) {
      auto *field = form->itemAt(row, QFormLayout::FieldRole);
      auto *caption = form->itemAt(row, QFormLayout::LabelRole);
      if (!field || !caption) continue;
      auto *value = qobject_cast<QLabel *>(field->widget());
      auto *name = qobject_cast<QLabel *>(caption->widget());
      if (!value || !name) continue;
      if (name->text() == "Measured:")
        value->setText("diameter 72.3 mm (0.0 mm/s)\nURDF finger joint 0.762 rad");
      if (name->text() == "Connection:")
        value->setText("Connected: idle");
      if (name->text() == "Force:")
        value->setText("20% (hardware default)");
    }
  }
  panel.resize(440, 500);
  panel.show();
  QTest::qWait(50);
  int checked = 0;
  for (auto *form : panel.findChildren<QFormLayout *>()) {
    for (int row = 0; row < form->rowCount(); ++row) {
      auto *field = form->itemAt(row, QFormLayout::FieldRole);
      if (!field) continue;
      auto *value = qobject_cast<QLabel *>(field->widget());
      if (!value) continue;
      ++checked;
      EXPECT_GE(value->width(), 180) << value->text().toStdString();
      EXPECT_GE(value->height(), value->heightForWidth(value->width()));
      EXPECT_TRUE(panel.contentsRect().contains(value->mapTo(
          &panel, QPoint(value->width() - 1, value->height() - 1))));
    }
  }
  EXPECT_EQ(checked, 3);
}

} // namespace

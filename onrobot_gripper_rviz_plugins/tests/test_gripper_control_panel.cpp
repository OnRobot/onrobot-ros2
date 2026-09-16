#include <limits>

#include <gtest/gtest.h>

#include <QFile>

#include "onrobot_gripper_rviz_plugins/branded_panel.hpp"
#include "onrobot_gripper_rviz_plugins/gripper_control_panel.hpp"
#include "onrobot_gripper_rviz_plugins/panel_settings.hpp"
#include "onrobot_gripper_rviz_plugins/safety_panel_semantics.hpp"

namespace {

using onrobot_gripper_rviz_plugins::formatMeasuredGripperState;
using onrobot_gripper_rviz_plugins::formatSafetyState;

TEST(GripperControlPanel, BrandHeaderUsesEmbeddedApprovedLogo) {
  const QFile logo(":/onrobot/branding/logo_onrobot_rgb.png");
  EXPECT_TRUE(logo.exists());
  EXPECT_GT(logo.size(), 0);
}

TEST(GripperControlPanel, FormatsAvailableMeasuredForce) {
  EXPECT_EQ(formatMeasuredGripperState(0.1061, 12.34),
            QString("0.1061 m, 12.3 N"));
}

TEST(GripperControlPanel, FormatsRgSafetyStateForOperatorAction) {
  EXPECT_EQ(formatSafetyState(false, false, false, false, false), "Clear");
  EXPECT_TRUE(formatSafetyState(true, false, false, false, false)
                  .startsWith("STOPPED"));
  EXPECT_TRUE(formatSafetyState(false, true, false, false, false)
                  .startsWith("Triggered"));
  EXPECT_TRUE(formatSafetyState(false, false, false, false, true)
                  .startsWith("DC ERROR"));
}

TEST(GripperControlPanel, OmitsUnavailableMeasuredForce) {
  EXPECT_EQ(formatMeasuredGripperState(
                0.1061, std::numeric_limits<double>::quiet_NaN()),
            QString("0.1061 m"));
  EXPECT_EQ(formatMeasuredGripperState(0.1061,
                                       std::numeric_limits<double>::infinity()),
            QString("0.1061 m"));
}

TEST(GripperControlPanel, PanelSettingsRoundTripIsLocalToEachConfigEntry) {
  using onrobot_gripper_rviz_plugins::PanelSettings;

  rviz_common::Config first;
  PanelSettings firstSettings;
  firstSettings.writeString("limits_topic", "/left/limits");
  firstSettings.writeString("gripper_state_topic", "/left/state");
  firstSettings.writeDouble("target_aperture_m", 0.011);
  firstSettings.save(first);

  rviz_common::Config second;
  PanelSettings secondSettings;
  secondSettings.writeString("limits_topic", "/right/limits");
  secondSettings.writeString("gripper_state_topic", "/right/state");
  secondSettings.writeDouble("target_aperture_m", 0.021);
  secondSettings.save(second);

  PanelSettings loadedFirst;
  PanelSettings loadedSecond;
  loadedFirst.load(first);
  loadedSecond.load(second);
  std::string topic;
  double target = 0.0;
  ASSERT_TRUE(loadedFirst.readString("limits_topic", topic));
  EXPECT_EQ(topic, "/left/limits");
  ASSERT_TRUE(loadedSecond.readString("limits_topic", topic));
  EXPECT_EQ(topic, "/right/limits");
  ASSERT_TRUE(loadedFirst.readDouble("target_aperture_m", target));
  EXPECT_NEAR(target, 0.011, 1e-6);
  ASSERT_TRUE(loadedSecond.readDouble("target_aperture_m", target));
  EXPECT_NEAR(target, 0.021, 1e-6);
}

} // namespace

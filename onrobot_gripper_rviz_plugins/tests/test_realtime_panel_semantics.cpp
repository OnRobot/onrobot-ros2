#include <limits>

#include <gtest/gtest.h>

#include "onrobot_gripper_rviz_plugins/realtime_panel_semantics.hpp"

namespace {

using onrobot_gripper_rviz_plugins::RealtimeDirectionGuard;
using onrobot_gripper_rviz_plugins::RealtimeJoystickMapper;
using onrobot_gripper_rviz_plugins::RealtimePositionTargetTracker;

TEST(RealtimeJoystickMapper, CenterAndDeadbandAreStopped) {
  EXPECT_DOUBLE_EQ(RealtimeJoystickMapper::axisToVelocity(0.0), 0.0);
  EXPECT_DOUBLE_EQ(RealtimeJoystickMapper::axisToVelocity(0.04), 0.0);
  EXPECT_DOUBLE_EQ(RealtimeJoystickMapper::axisToVelocity(-0.05), 0.0);
}

TEST(RealtimeJoystickMapper, LeftOpensAndRightCloses) {
  EXPECT_NEAR(RealtimeJoystickMapper::axisToVelocity(-1.0), 0.03, 1e-12);
  EXPECT_NEAR(RealtimeJoystickMapper::axisToVelocity(1.0), -0.03, 1e-12);
  EXPECT_GT(RealtimeJoystickMapper::axisToVelocity(-0.5), 0.0);
  EXPECT_LT(RealtimeJoystickMapper::axisToVelocity(0.5), 0.0);
}

TEST(RealtimeJoystickMapper, ClampsAxisAndScalesConfiguredSpeed) {
  EXPECT_DOUBLE_EQ(RealtimeJoystickMapper::axisToVelocity(-2.0, 0.02), 0.02);
  EXPECT_DOUBLE_EQ(RealtimeJoystickMapper::axisToVelocity(2.0, 0.02), -0.02);
  EXPECT_NEAR(RealtimeJoystickMapper::axisToVelocity(-0.525, 0.02, 0.05), 0.01,
              1e-12);
}

TEST(RealtimeJoystickMapper, RejectsNonFiniteConfigurationSafely) {
  EXPECT_DOUBLE_EQ(RealtimeJoystickMapper::axisToVelocity(
                       std::numeric_limits<double>::quiet_NaN()),
                   0.0);
  EXPECT_DOUBLE_EQ(RealtimeJoystickMapper::axisToVelocity(0.5, -0.1), 0.0);
  EXPECT_DOUBLE_EQ(RealtimeJoystickMapper::axisToVelocity(0.5, 0.1, 1.0), 0.0);
}

TEST(RealtimeJoystickMapper, PointerTrackingIsAbsoluteAndStable) {
  EXPECT_EQ(RealtimeJoystickMapper::pointerToSliderValue(0, 400), -100);
  EXPECT_EQ(RealtimeJoystickMapper::pointerToSliderValue(200, 400), 0);
  EXPECT_EQ(RealtimeJoystickMapper::pointerToSliderValue(400, 400), 100);
  EXPECT_EQ(
      RealtimeJoystickMapper::pointerToSliderValue(0, 400, -100, 100, true),
      100);

  const int heldOpen = RealtimeJoystickMapper::pointerToSliderValue(0, 400);
  for (int sample = 0; sample < 100; ++sample) {
    EXPECT_EQ(RealtimeJoystickMapper::pointerToSliderValue(0, 400), heldOpen);
  }
}

TEST(RealtimeDirectionGuard, StopsOpeningCommandOnMeasuredClosing) {
  RealtimeDirectionGuard guard(0.001);
  EXPECT_FALSE(guard.observe(0.7, 0.100));
  EXPECT_FALSE(guard.observe(0.7, 0.110));
  EXPECT_FALSE(guard.observe(0.7, 0.1095));
  EXPECT_TRUE(guard.observe(0.7, 0.1089));
}

TEST(RealtimeDirectionGuard, StopsClosingCommandOnMeasuredOpening) {
  RealtimeDirectionGuard guard(0.001);
  EXPECT_FALSE(guard.observe(-0.7, 0.100));
  EXPECT_FALSE(guard.observe(-0.7, 0.090));
  EXPECT_FALSE(guard.observe(-0.7, 0.0905));
  EXPECT_TRUE(guard.observe(-0.7, 0.0911));
}

TEST(RealtimeDirectionGuard, DirectionChangeStartsANewObservation) {
  RealtimeDirectionGuard guard(0.001);
  EXPECT_FALSE(guard.observe(0.7, 0.100));
  EXPECT_FALSE(guard.observe(0.7, 0.110));
  EXPECT_FALSE(guard.observe(-0.7, 0.110));
  // The prior opening motion may continue while firmware decelerates and
  // applies the requested reversal. It must not trip the guard.
  EXPECT_FALSE(guard.observe(-0.7, 0.114));
  EXPECT_FALSE(guard.observe(-0.7, 0.116));
  EXPECT_FALSE(guard.observe(-0.7, 0.114));
  EXPECT_FALSE(guard.observe(-0.7, 0.110));
  // Only after closing has been observed can renewed opening be contradictory.
  EXPECT_TRUE(guard.observe(-0.7, 0.112));
}

TEST(RealtimePositionTargetTracker, RequiresStableFeedbackAtTarget) {
  RealtimePositionTargetTracker tracker(0.0005, 2);
  tracker.reset(0.050);
  EXPECT_FALSE(tracker.observe(true, 0.040));
  EXPECT_FALSE(tracker.observe(true, 0.0496));
  EXPECT_TRUE(tracker.observe(true, 0.0503));
}

TEST(RealtimePositionTargetTracker, InvalidOrDepartingFeedbackResetsMatch) {
  RealtimePositionTargetTracker tracker(0.0005, 2);
  tracker.reset(0.050);
  EXPECT_FALSE(tracker.observe(true, 0.0501));
  EXPECT_FALSE(tracker.observe(false, 0.0501));
  EXPECT_FALSE(tracker.observe(true, 0.0501));
  EXPECT_FALSE(tracker.observe(true, 0.051));
  EXPECT_FALSE(tracker.observe(true, 0.0501));
  EXPECT_TRUE(tracker.observe(true, 0.0499));
}

} // namespace

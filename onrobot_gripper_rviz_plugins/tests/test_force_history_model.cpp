#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "onrobot_gripper_rviz_plugins/force_history_model.hpp"

namespace {

using onrobot_gripper_rviz_plugins::ForceHistoryBuffer;
using onrobot_gripper_rviz_plugins::ForceHistorySample;
using onrobot_gripper_rviz_plugins::ForceSampleQuality;
using onrobot_gripper_rviz_plugins::forceSampleQuality;

ForceHistorySample sample(std::int64_t i_stamp, std::uint64_t i_sequence,
                          double i_force) {
  ForceHistorySample value;
  value.stamp_ns = i_stamp;
  value.received_steady_ns = i_stamp + 1;
  value.sequence = i_sequence;
  value.force_n = i_force;
  value.force_valid = true;
  value.force_fresh = true;
  value.force_supported = true;
  return value;
}

TEST(ForceHistoryModel, KeepsSignedMeasuredForceAndClassifiesItAsValid) {
  auto value = sample(100, 1, -59.0);
  EXPECT_EQ(forceSampleQuality(value), ForceSampleQuality::Valid);
  EXPECT_DOUBLE_EQ(value.force_n, -59.0);
}

TEST(ForceHistoryModel, DistinguishesInvalidStaleAndUnavailable) {
  auto invalid = sample(100, 1, 0.0);
  invalid.force_valid = false;
  EXPECT_EQ(forceSampleQuality(invalid), ForceSampleQuality::Invalid);

  auto stale = sample(100, 2, 3.0);
  stale.force_fresh = false;
  EXPECT_EQ(forceSampleQuality(stale), ForceSampleQuality::Stale);

  auto unavailable = sample(100, 3, 10.0);
  unavailable.force_supported = false;
  EXPECT_EQ(forceSampleQuality(unavailable), ForceSampleQuality::Unavailable);

  auto nonfinite = sample(100, 4, std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(forceSampleQuality(nonfinite), ForceSampleQuality::Invalid);
}

TEST(ForceHistoryModel, DeduplicatesRepeatedPhysicalSamples) {
  ForceHistoryBuffer history(4);
  EXPECT_TRUE(history.append(sample(100, 7, -1.0)));
  auto duplicate = sample(110, 7, -2.0);
  EXPECT_FALSE(history.append(duplicate));
  ASSERT_EQ(history.size(), 1U);
  EXPECT_DOUBLE_EQ(history.samples().front().force_n, -1.0);
}

TEST(ForceHistoryModel, AgesTheLastForceWhenThePublisherDisappears) {
  ForceHistoryBuffer history;
  auto value = sample(100, 7, -12.0);
  value.received_steady_ns = 1'000'000'000;
  ASSERT_TRUE(history.append(value));

  EXPECT_FALSE(history.markLatestStaleIfExpired(1'200'000'000,
                                                250'000'000));
  EXPECT_EQ(forceSampleQuality(history.samples().back()),
            ForceSampleQuality::Valid);
  EXPECT_TRUE(history.markLatestStaleIfExpired(1'300'000'001,
                                               250'000'000));
  EXPECT_EQ(forceSampleQuality(history.samples().back()),
            ForceSampleQuality::Stale);
  EXPECT_FALSE(history.markLatestStaleIfExpired(1'400'000'000,
                                                250'000'000));
}

TEST(ForceHistoryModel, RetainsExplicitClockRollbackBoundaryAndBoundsMemory) {
  ForceHistoryBuffer history(2);
  EXPECT_TRUE(history.append(sample(200, 1, -1.0)));
  EXPECT_TRUE(history.append(sample(100, 2, -2.0)));
  ASSERT_EQ(history.size(), 2U);
  EXPECT_TRUE(history.samples().back().clock_rollback_before);

  EXPECT_TRUE(history.append(sample(300, 3, -3.0)));
  ASSERT_EQ(history.size(), 2U);
  EXPECT_EQ(history.samples().front().sequence, 2U);
  EXPECT_EQ(history.samples().back().sequence, 3U);
}

TEST(ForceHistoryModel, ModeAndForceContractNamesAreStable) {
  EXPECT_STREQ(onrobot_gripper_rviz_plugins::realtimeModeName(0), "Idle");
  EXPECT_STREQ(onrobot_gripper_rviz_plugins::realtimeModeName(3),
               "RT velocity");
  EXPECT_STREQ(onrobot_gripper_rviz_plugins::forceSampleQualityName(
                   ForceSampleQuality::Unavailable),
               "unavailable");
}

TEST(ForceHistoryModel, RetainsFaultTransitionsForTheTimeline) {
  auto value = sample(100, 1, -4.0);
  value.fault_source = 4;
  value.fault_code = 23;
  ForceHistoryBuffer history;
  ASSERT_TRUE(history.append(value));
  ASSERT_EQ(history.size(), 1U);
  EXPECT_EQ(history.samples().back().fault_source, 4U);
  EXPECT_EQ(history.samples().back().fault_code, 23U);
}

} // namespace

#include <algorithm>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/panel.hpp>

TEST(OnRobotRvizPlugins, PluginDescriptionsAreDiscoverable) {
  pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common",
                                                    "rviz_common::Panel");
  const auto classes = loader.getDeclaredClasses();
  EXPECT_NE(std::find(classes.begin(), classes.end(),
                      "onrobot_gripper_rviz_plugins/GripperControlPanel"),
            classes.end());
  EXPECT_NE(std::find(classes.begin(), classes.end(),
                      "onrobot_gripper_rviz_plugins/RealtimeControlPanel"),
            classes.end());
}

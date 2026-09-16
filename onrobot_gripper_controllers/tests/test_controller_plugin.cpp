#include <algorithm>

#include <controller_interface/controller_interface.hpp>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>

TEST(OnRobotRealtimeController, PluginDescriptionIsDiscoverable) {
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
      "controller_interface", "controller_interface::ControllerInterface");
  const auto classes = loader.getDeclaredClasses();
  EXPECT_NE(std::find(classes.begin(), classes.end(),
                      "onrobot_gripper_controllers/OnRobotRealtimeController"),
            classes.end());
}

TEST(OnRobotStateBroadcaster, PluginDescriptionIsDiscoverable) {
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
      "controller_interface", "controller_interface::ControllerInterface");
  EXPECT_TRUE(loader.isClassAvailable(
      "onrobot_gripper_controllers/GripperStateBroadcaster"));
}

TEST(OnRobotRecoveryController, PluginDescriptionIsDiscoverable) {
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
      "controller_interface", "controller_interface::ControllerInterface");
  EXPECT_TRUE(loader.isClassAvailable(
      "onrobot_gripper_controllers/GripperRecoveryController"));
}

TEST(OnRobotParallelGripperActionController, PluginDescriptionIsDiscoverable) {
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
      "controller_interface", "controller_interface::ControllerInterface");
  EXPECT_TRUE(loader.isClassAvailable(
      "onrobot_gripper_controllers/ParallelGripperActionController"));
}

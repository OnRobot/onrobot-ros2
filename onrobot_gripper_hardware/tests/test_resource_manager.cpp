#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <control_msgs/action/parallel_gripper_command.hpp>
#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/resource_manager_params.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "support/modbus_tcp_test_server.hpp"
#include "support/modbus_rtu_test_server.hpp"

namespace {

using namespace std::chrono_literals;

std::string fake2Fg7Urdf() {
  return R"(
<?xml version="1.0"?>
<robot name="onrobot_2fg7_test">
  <link name="base_link"/>
  <link name="task_link"/>
  <link name="finger_link"/>
  <joint name="grip_stroke" type="prismatic">
    <parent link="base_link"/>
    <child link="task_link"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="0.107" effort="100" velocity="1"/>
  </joint>
  <joint name="finger_stroke" type="prismatic">
    <parent link="base_link"/>
    <child link="finger_link"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="0.019" effort="100" velocity="1"/>
  </joint>
  <ros2_control name="OnRobot2FG7TestSystem" type="system">
    <hardware>
      <plugin>onrobot_gripper_hardware/OnRobotParallelGripperFakeSystem</plugin>
      <param name="model">2fg7</param>
      <param name="fake_task_min_m">0.0</param>
      <param name="fake_task_max_m">0.107</param>
      <param name="finger_joint_upper_m">0.019</param>
      <param name="raw_linear_min_mm">1.0</param>
      <param name="raw_linear_max_mm">39.0</param>
    </hardware>
    <joint name="grip_stroke">
      <command_interface name="position"/>
      <command_interface name="effort"/>
      <command_interface name="conventional_command_sequence"/>
      <command_interface name="fault_recovery_command_sequence"/>
      <command_interface name="stop_command_sequence"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
      <state_interface name="task_position_valid"/>
      <state_interface name="force_valid"/>
      <state_interface name="busy"/>
      <state_interface name="grip_detected"/>
      <state_interface name="active_mode"/>
      <state_interface name="connection_state"/>
      <state_interface name="faulted"/>
      <state_interface name="fault_code"/>
      <state_interface name="sample_sequence"/>
      <state_interface name="sample_age"/>
      <state_interface name="requested_command_sequence"/>
      <state_interface name="applied_command_sequence"/>
      <state_interface name="successful_cycles"/>
      <state_interface name="failed_cycles"/>
      <state_interface name="missed_deadlines"/>
      <state_interface name="watchdog_stops"/>
      <state_interface name="reconnects"/>
      <state_interface name="last_cycle_duration"/>
      <state_interface name="minimum_task_aperture"/>
      <state_interface name="maximum_task_aperture"/>
    </joint>
    <joint name="finger_stroke">
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="measured_position"/>
      <state_interface name="measured_velocity"/>
      <state_interface name="position_valid"/>
      <state_interface name="velocity_valid"/>
    </joint>
  </ros2_control>
</robot>)";
}

std::string real2Fg7Urdf(uint16_t port) {
  auto urdf = fake2Fg7Urdf();
  const std::string fake_plugin =
      "onrobot_gripper_hardware/OnRobotParallelGripperFakeSystem";
  const std::string real_plugin =
      "onrobot_gripper_hardware/OnRobotGripperSystem";
  urdf.replace(urdf.find(fake_plugin), fake_plugin.size(), real_plugin);

  const std::string model = "<param name=\"model\">2fg7</param>";
  const std::string transport = model +
                                "<param name=\"transport\">tcp</param>"
                                "<param name=\"host\">127.0.0.1</param>"
                                "<param name=\"port\">" +
                                std::to_string(port) +
                                "</param>"
                                "<param name=\"timeout_ms\">250</param>";
  urdf.replace(urdf.find(model), model.size(), transport);
  return urdf;
}

std::string real2Fg7RtuUrdf(const std::string &device) {
  auto urdf = fake2Fg7Urdf();
  const std::string fake_plugin =
      "onrobot_gripper_hardware/OnRobotParallelGripperFakeSystem";
  const std::string real_plugin =
      "onrobot_gripper_hardware/OnRobotGripperSystem";
  urdf.replace(urdf.find(fake_plugin), fake_plugin.size(), real_plugin);

  const std::string model = "<param name=\"model\">2fg7</param>";
  const std::string transport = model +
                                "<param name=\"transport\">rtu</param>"
                                "<param name=\"serial_device\">" +
                                device +
                                "</param>"
                                "<param name=\"baud_rate\">1000000</param>"
                                "<param name=\"rtu_parity\">even</param>"
                                "<param name=\"slave_id\">65</param>"
                                "<param name=\"timeout_ms\">250</param>";
  urdf.replace(urdf.find(model), model.size(), transport);
  return urdf;
}

void addConventionalSpeedResource(std::string &urdf) {
  const std::string marker =
      "<command_interface name=\"conventional_command_sequence\"/>";
  const auto position = urdf.find(marker);
  if (position == std::string::npos) {
    throw std::runtime_error("missing conventional command resource marker");
  }
  urdf.insert(position + marker.size(),
              "<command_interface name=\"conventional_speed_percent\"/>");
}

std::string fake2Fg7RealtimeUrdf() {
  auto urdf = fake2Fg7Urdf();
  const std::string recovery =
      "<command_interface name=\"fault_recovery_command_sequence\"/>";
  const std::string realtime =
      "<command_interface name=\"realtime_mode\"/>"
      "<command_interface name=\"realtime_task_position\"/>"
      "<command_interface name=\"realtime_task_velocity\"/>"
      "<command_interface name=\"realtime_force\"/>"
      "<command_interface name=\"realtime_command_sequence\"/>" +
      recovery;
  const auto position = urdf.find(recovery);
  if (position == std::string::npos) {
    throw std::runtime_error(
        "missing recovery interface in fake 2FG7 test URDF");
  }
  urdf.replace(position, recovery.size(), realtime);
  return urdf;
}

std::string controllerManagerFakeUrdf() {
  auto urdf = fake2Fg7Urdf();
  const std::string marker =
      "<param name=\"fake_task_min_m\">0.0</param>";
  const auto position = urdf.find(marker);
  if (position == std::string::npos) {
    throw std::runtime_error("missing fake motion parameter insertion point");
  }
  urdf.insert(position, "<param name=\"fake_motion_speed_m_s\">0.02</param>");
  return urdf;
}

std::string real2Fg7RealtimeUrdf(uint16_t port) {
  auto urdf = real2Fg7Urdf(port);
  const std::string recovery =
      "<command_interface name=\"fault_recovery_command_sequence\"/>";
  const std::string realtime =
      "<command_interface name=\"realtime_mode\"/>"
      "<command_interface name=\"realtime_task_position\"/>"
      "<command_interface name=\"realtime_task_velocity\"/>"
      "<command_interface name=\"realtime_force\"/>"
      "<command_interface name=\"realtime_command_sequence\"/>" +
      recovery;
  const auto position = urdf.find(recovery);
  if (position == std::string::npos) {
    throw std::runtime_error(
        "missing recovery interface in real 2FG7 test URDF");
  }
  urdf.replace(position, recovery.size(), realtime);
  return urdf;
}

std::string realRg6Urdf(uint16_t port) {
  auto urdf = fake2Fg7Urdf();
  const auto replace_all = [&urdf](const std::string &from,
                                   const std::string &to) {
    size_t position = 0;
    while ((position = urdf.find(from, position)) != std::string::npos) {
      urdf.replace(position, from.size(), to);
      position += to.size();
    }
  };
  replace_all("onrobot_2fg7_test", "onrobot_rg6_test");
  replace_all("OnRobot2FG7TestSystem", "OnRobotRG6TestSystem");
  replace_all("onrobot_gripper_hardware/OnRobotParallelGripperFakeSystem",
              "onrobot_gripper_hardware/OnRobotRgSystem");
  replace_all("<param name=\"model\">2fg7</param>",
              "<param name=\"model\">rg6</param>"
              "<param name=\"transport\">tcp</param>"
              "<param name=\"host\">127.0.0.1</param>"
              "<param name=\"port\">" + std::to_string(port) + "</param>"
              "<param name=\"timeout_ms\">250</param>"
              "<param name=\"safe_width_min_mm\">0</param>"
              "<param name=\"safe_width_max_mm\">160</param>");
  replace_all("finger_stroke", "finger_joint");
  replace_all("measured_position", "measured_angular_position");
  replace_all("measured_velocity", "measured_angular_velocity");
  replace_all("0.107", "0.160");
  replace_all("0.019", "1.1928");
  return urdf;
}

std::string isaac2Fg7Urdf() {
  auto urdf = fake2Fg7Urdf();
  const std::string fake_plugin =
      "onrobot_gripper_hardware/OnRobotParallelGripperFakeSystem";
  const std::string isaac_plugin =
      "onrobot_gripper_hardware/OnRobotIsaacSystem";
  urdf.replace(urdf.find(fake_plugin), fake_plugin.size(), isaac_plugin);

  const std::string model = "<param name=\"model\">2fg7</param>";
  const std::string parameters =
      model +
      "<param name=\"task_min_m\">0.0</param>"
      "<param name=\"task_max_m\">0.107</param>"
      "<param name=\"physical_min_m\">0.0</param>"
      "<param name=\"physical_max_m\">0.019</param>"
      "<param name=\"isaac_joint_commands_topic\">"
      "isaac_test_commands</param>"
      "<param name=\"isaac_joint_states_topic\">isaac_test_states</param>"
      "<param name=\"isaac_state_timeout_ms\">50</param>";
  urdf.replace(urdf.find(model), model.size(), parameters);

  const std::string recovery =
      "<command_interface name=\"fault_recovery_command_sequence\"/>";
  const std::string realtime =
      "<command_interface name=\"realtime_mode\"/>"
      "<command_interface name=\"realtime_task_position\"/>"
      "<command_interface name=\"realtime_task_velocity\"/>"
      "<command_interface name=\"realtime_force\"/>"
      "<command_interface name=\"realtime_command_sequence\"/>" +
      recovery;
  urdf.replace(urdf.find(recovery), recovery.size(), realtime);
  return urdf;
}

std::string isaac2Fg14Urdf() {
  auto urdf = isaac2Fg7Urdf();
  const auto replace_once = [&urdf](const std::string &from,
                                    const std::string &to) {
    const auto position = urdf.find(from);
    if (position == std::string::npos) {
      throw std::runtime_error("missing Isaac test URDF token: " + from);
    }
    urdf.replace(position, from.size(), to);
  };
  replace_once("<param name=\"model\">2fg7</param>",
               "<param name=\"model\">2fg14</param>");
  replace_once("<param name=\"task_max_m\">0.107</param>",
               "<param name=\"task_max_m\">0.140</param>");
  replace_once("<param name=\"physical_max_m\">0.019</param>",
               "<param name=\"physical_max_m\">0.025</param>");
  replace_once("<param name=\"raw_linear_max_mm\">39.0</param>",
               "<param name=\"raw_linear_max_mm\">51.0</param>");
  return urdf;
}

std::string isaacRg2Urdf() {
  auto urdf = isaac2Fg7Urdf();
  const auto replace_all = [&urdf](const std::string &from,
                                   const std::string &to) {
    size_t position = 0;
    bool replaced = false;
    while ((position = urdf.find(from, position)) != std::string::npos) {
      urdf.replace(position, from.size(), to);
      position += to.size();
      replaced = true;
    }
    if (!replaced) {
      throw std::runtime_error("missing RG2 Isaac test URDF token: " + from);
    }
  };
  replace_all("2fg7", "rg2");
  replace_all("finger_stroke", "finger_joint");
  replace_all("0.107", "0.110");
  replace_all("0.019", "1.22277767395");
  replace_all("<param name=\"raw_linear_min_mm\">1.0</param>",
              "<param name=\"raw_linear_min_mm\">0.0</param>");
  replace_all("<param name=\"raw_linear_max_mm\">39.0</param>",
              "<param name=\"raw_linear_max_mm\">1222.77767395</param>");
  const std::string model = "<param name=\"model\">rg2</param>";
  const auto model_position = urdf.find(model);
  if (model_position == std::string::npos) {
    throw std::runtime_error("missing RG2 model parameter");
  }
  urdf.insert(model_position + model.size(),
              "<param name=\"isaac_joint\">finger_joint</param>");
  replace_all("realtime_task_velocity",
              "realtime_mechanism_angular_velocity");
  replace_all("measured_position", "measured_angular_position");
  replace_all("measured_velocity", "measured_angular_velocity");
  return urdf;
}

std::string isaacRg6Urdf() {
  auto urdf = isaacRg2Urdf();
  const auto replace_all = [&urdf](const std::string &from,
                                   const std::string &to) {
    size_t position = 0;
    bool replaced = false;
    while ((position = urdf.find(from, position)) != std::string::npos) {
      urdf.replace(position, from.size(), to);
      position += to.size();
      replaced = true;
    }
    if (!replaced) {
      throw std::runtime_error("missing RG6 Isaac test URDF token: " + from);
    }
  };
  replace_all("rg2", "rg6");
  replace_all("0.110", "0.160");
  replace_all("1.22277767395", "1.1928");
  replace_all("1222.77767395", "1192.8");
  return urdf;
}

class ResourceManagerTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(ResourceManagerTest, ActivatesClaimsAndCyclesAtManagerBoundary) {
  auto node = std::make_shared<rclcpp::Node>("onrobot_resource_manager_test");
  hardware_interface::ResourceManager manager(
      fake2Fg7Urdf(), node->get_node_clock_interface(),
      node->get_node_logging_interface(), true, 500);

  EXPECT_TRUE(manager.command_interface_exists("grip_stroke/position"));
  EXPECT_TRUE(manager.command_interface_exists(
      "grip_stroke/fault_recovery_command_sequence"));
  EXPECT_TRUE(
      manager.command_interface_exists("grip_stroke/stop_command_sequence"));
  EXPECT_FALSE(manager.command_interface_exists("grip_stroke/realtime_mode"));
  EXPECT_TRUE(manager.state_interface_exists("grip_stroke/connection_state"));
  EXPECT_TRUE(manager.state_interface_exists("finger_stroke/position"));
  EXPECT_TRUE(
      manager.state_interface_exists("finger_stroke/measured_position"));

  std::chrono::steady_clock::duration elapsed;
  {
    auto position = manager.claim_command_interface("grip_stroke/position");
    auto recovery = manager.claim_command_interface(
        "grip_stroke/fault_recovery_command_sequence");
    ASSERT_TRUE(position.set_value(0.050));
    ASSERT_TRUE(recovery.set_value(1.0));

    const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.002);
    const auto started = std::chrono::steady_clock::now();
    for (std::size_t cycle = 0; cycle < 1000; ++cycle) {
      const auto time = node->now();
      static_cast<void>(manager.write(time, period));
      static_cast<void>(manager.read(time, period));
    }
    elapsed = std::chrono::steady_clock::now() - started;

    auto connection =
        manager.claim_state_interface("grip_stroke/connection_state");
    auto measured_position =
        manager.claim_state_interface("grip_stroke/position");
    auto finger_position =
        manager.claim_state_interface("finger_stroke/position");
    auto raw_mechanism_position =
        manager.claim_state_interface("finger_stroke/measured_position");
    const auto connection_value = connection.get_optional<double>();
    const auto measured_value = measured_position.get_optional<double>();
    const auto finger_value = finger_position.get_optional<double>();
    const auto raw_mechanism_value =
        raw_mechanism_position.get_optional<double>();
    ASSERT_TRUE(connection_value.has_value());
    ASSERT_TRUE(measured_value.has_value());
    ASSERT_TRUE(finger_value.has_value());
    ASSERT_TRUE(raw_mechanism_value.has_value());
    EXPECT_DOUBLE_EQ(*connection_value, 3.0);
    EXPECT_DOUBLE_EQ(*measured_value, 0.050);
    // Stock outward fingers have a 33 mm aperture at zero finger stroke.
    // Task aperture must not stretch the CAD's 19 mm stroke to a 107 mm gap.
    EXPECT_NEAR(*finger_value, (0.050 - 0.033) / 2.0, 1e-12);
    EXPECT_NEAR(*raw_mechanism_value, 0.001 + (0.050 - 0.033), 1e-12);
  }

  // A fake 500 Hz manager workload must remain a memory-only operation. This
  // generous bound detects accidental sleeps or transport calls without making
  // the test sensitive to shared CI scheduling.
  EXPECT_LT(elapsed, std::chrono::milliseconds(250));

  rclcpp_lifecycle::State inactive(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  EXPECT_EQ(manager.set_component_state("OnRobot2FG7TestSystem", inactive),
            hardware_interface::return_type::OK);
  rclcpp_lifecycle::State unconfigured(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  EXPECT_EQ(manager.set_component_state("OnRobot2FG7TestSystem", unconfigured),
            hardware_interface::return_type::OK);
}

TEST_F(ResourceManagerTest,
       FakeStopConsumesPendingCommandAndSuppressesStaleMotion) {
  auto node = std::make_shared<rclcpp::Node>(
      "onrobot_fake_resource_manager_stop_handoff_test");
  hardware_interface::ResourceManager manager(
      fake2Fg7RealtimeUrdf(), node->get_node_clock_interface(),
      node->get_node_logging_interface(), true, 500);

  auto mode = manager.claim_command_interface("grip_stroke/realtime_mode");
  auto velocity =
      manager.claim_command_interface("grip_stroke/realtime_task_velocity");
  auto sequence = manager.claim_command_interface(
      "grip_stroke/realtime_command_sequence");
  auto stop =
      manager.claim_command_interface("grip_stroke/stop_command_sequence");
  ASSERT_TRUE(mode.set_value(1.0));
  ASSERT_TRUE(velocity.set_value(0.050));
  ASSERT_TRUE(sequence.set_value(1.0));
  ASSERT_EQ(manager.write(node->now(), rclcpp::Duration::from_seconds(0.1)).result,
            hardware_interface::return_type::OK);

  auto position = manager.claim_state_interface("grip_stroke/position");
  auto active_mode =
      manager.claim_state_interface("grip_stroke/active_mode");
  ASSERT_EQ(manager.read(node->now(), rclcpp::Duration::from_seconds(0.1)).result,
            hardware_interface::return_type::OK);
  ASSERT_DOUBLE_EQ(*active_mode.get_optional<double>(), 3.0);
  const auto stopped_position = position.get_optional<double>();
  ASSERT_TRUE(stopped_position.has_value());

  // Simulate controller deactivation: the controller has no later update
  // opportunity, so only the Stop event is written while stale realtime
  // inputs remain in the other command interfaces.
  ASSERT_TRUE(stop.set_value(2.0));
  ASSERT_EQ(manager.write(node->now(), rclcpp::Duration::from_seconds(0.1)).result,
            hardware_interface::return_type::OK);
  ASSERT_EQ(manager.read(node->now(), rclcpp::Duration::from_seconds(0.1)).result,
            hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(*active_mode.get_optional<double>(), 0.0);
  EXPECT_DOUBLE_EQ(*position.get_optional<double>(), *stopped_position);

  // A controller may be recreated after this event. The marker is allowed to
  // repeat because it is consumed and cleared on every write; the second
  // event must still stop a newly admitted realtime command.
  ASSERT_TRUE(mode.set_value(1.0));
  ASSERT_TRUE(velocity.set_value(0.050));
  ASSERT_TRUE(sequence.set_value(2.0));
  ASSERT_EQ(manager.write(node->now(), rclcpp::Duration::from_seconds(0.1)).result,
            hardware_interface::return_type::OK);
  ASSERT_EQ(manager.read(node->now(), rclcpp::Duration::from_seconds(0.1)).result,
            hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(*active_mode.get_optional<double>(), 3.0);
  ASSERT_TRUE(stop.set_value(2.0));
  ASSERT_EQ(manager.write(node->now(), rclcpp::Duration::from_seconds(0.1)).result,
            hardware_interface::return_type::OK);
  ASSERT_EQ(manager.read(node->now(), rclcpp::Duration::from_seconds(0.1)).result,
            hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(*active_mode.get_optional<double>(), 0.0);
  EXPECT_DOUBLE_EQ(*position.get_optional<double>(), *stopped_position);

  // The stale realtime sequence and target must not replay after the Stop.
  ASSERT_EQ(manager.write(node->now(), rclcpp::Duration::from_seconds(0.1)).result,
            hardware_interface::return_type::OK);
  ASSERT_EQ(manager.read(node->now(), rclcpp::Duration::from_seconds(0.1)).result,
            hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(*active_mode.get_optional<double>(), 0.0);
  EXPECT_DOUBLE_EQ(*position.get_optional<double>(), *stopped_position);

  // Invalid event tokens are rejected rather than truncated into a valid
  // command. A ResourceManager deactivates a component after the first write
  // error, so the physical adapter tests cover the complete invalid-value
  // matrix while this boundary test verifies the manager-visible error.
  ASSERT_TRUE(stop.set_value(0.0));
  EXPECT_EQ(
      manager.write(node->now(), rclcpp::Duration::from_seconds(0.1)).result,
      hardware_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(*active_mode.get_optional<double>(), 0.0);
}

TEST_F(ResourceManagerTest,
       ControllerManagerDeactivationWritesStopWithoutAnotherUpdate) {
  using Action = control_msgs::action::ParallelGripperCommand;
  const auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file",
                     N02_CONTROLLER_PARAMS_FILE});

  const auto resource_node =
      std::make_shared<rclcpp::Node>("n02_controller_manager_resource");
  executor->add_node(resource_node);
  auto resource_manager = std::make_unique<hardware_interface::ResourceManager>(
      controllerManagerFakeUrdf(), resource_node->get_node_clock_interface(),
      resource_node->get_node_logging_interface(), true, 500);
  auto *resource_manager_view = resource_manager.get();
  controller_manager::ControllerManager manager(
      std::move(resource_manager), executor, "n02_controller_manager", "",
      options);
  executor->add_node(manager.get_node_base_interface());
  const auto client_node = std::make_shared<rclcpp::Node>(
      "n02_controller_manager_action_client");
  executor->add_node(client_node);

  auto controller = manager.load_controller(
      "gripper_controller",
      "onrobot_gripper_controllers/ParallelGripperActionController");
  ASSERT_NE(controller, nullptr);
  ASSERT_EQ(manager.configure_controller("gripper_controller"),
            controller_interface::return_type::OK);
  const auto cycle = [&] {
    const auto time = manager.get_trigger_clock()->now();
    manager.read(time, rclcpp::Duration::from_seconds(0.01));
    manager.update(time, rclcpp::Duration::from_seconds(0.01));
    manager.write(time, rclcpp::Duration::from_seconds(0.01));
    executor->spin_some();
  };
  const auto switch_with_cycles = [&](const std::vector<std::string> &activate,
                                      const std::vector<std::string> &deactivate) {
    auto result = std::async(
        std::launch::async, [&manager, &activate, &deactivate] {
          return manager.switch_controller(
              activate, deactivate,
              controller_manager_msgs::srv::SwitchController::Request::STRICT,
              false, rclcpp::Duration::from_seconds(2.0));
        });
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (result.wait_for(0s) != std::future_status::ready &&
           std::chrono::steady_clock::now() < deadline) {
      cycle();
      std::this_thread::sleep_for(5ms);
    }
    return result.get();
  };

  auto action_client =
      rclcpp_action::create_client<Action>(client_node,
                                           "/gripper_controller/gripper_cmd");
  EXPECT_FALSE(action_client->wait_for_action_server(200ms));

  ASSERT_EQ(switch_with_cycles({"gripper_controller"}, {}),
            controller_interface::return_type::OK);

  const auto server_deadline = std::chrono::steady_clock::now() + 2s;
  while (!action_client->action_server_is_ready() &&
         std::chrono::steady_clock::now() < server_deadline) {
    executor->spin_some();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(action_client->action_server_is_ready());
  double initial_position;
  {
    auto measured = resource_manager_view->claim_state_interface("grip_stroke/position");
    initial_position = *measured.get_optional<double>();
  }

  // Cancellation before the first controller update must still issue the
  // controller-owned Stop/hold handoff and must not leave a goal buffered for
  // a later update cycle.
  Action::Goal pre_dispatch_goal;
  pre_dispatch_goal.command.name = {"grip_stroke"};
  pre_dispatch_goal.command.position = {0.080};
  pre_dispatch_goal.command.effort = {40.0};
  auto pre_dispatch_future = action_client->async_send_goal(pre_dispatch_goal);
  const auto pre_dispatch_deadline = std::chrono::steady_clock::now() + 2s;
  while (pre_dispatch_future.wait_for(0s) != std::future_status::ready &&
         std::chrono::steady_clock::now() < pre_dispatch_deadline) {
    executor->spin_some();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(pre_dispatch_future.wait_for(0s), std::future_status::ready);
  auto pre_dispatch_handle = pre_dispatch_future.get();
  ASSERT_NE(pre_dispatch_handle, nullptr);

  auto cancel_future = action_client->async_cancel_goal(pre_dispatch_handle);
  const auto cancel_deadline = std::chrono::steady_clock::now() + 2s;
  while (cancel_future.wait_for(0s) != std::future_status::ready &&
         std::chrono::steady_clock::now() < cancel_deadline) {
    executor->spin_some();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(cancel_future.wait_for(0s), std::future_status::ready);
  const auto cancel_response = cancel_future.get();
  ASSERT_NE(cancel_response, nullptr);
  ASSERT_EQ(cancel_response->return_code, 0);
  auto canceled_result = action_client->async_get_result(pre_dispatch_handle);
  for (int i = 0; i < 40; ++i) {
    cycle();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(canceled_result.wait_for(0s), std::future_status::ready);
  EXPECT_EQ(canceled_result.get().code, rclcpp_action::ResultCode::CANCELED);
  {
    auto measured = resource_manager_view->claim_state_interface("grip_stroke/position");
    EXPECT_DOUBLE_EQ(*measured.get_optional<double>(), initial_position);
  }
  pre_dispatch_handle.reset();

  // Replacing an active conventional goal with the same aperture and effort
  // must still issue the replacement after Stop. The hardware adapter's
  // numeric deduplication cannot be the identity of an action goal.
  Action::Goal first_goal;
  first_goal.command.name = {"grip_stroke"};
  first_goal.command.position = {0.040};
  first_goal.command.effort = {50.0};
  auto first_goal_future = action_client->async_send_goal(first_goal);
  const auto first_goal_deadline = std::chrono::steady_clock::now() + 2s;
  while (first_goal_future.wait_for(0s) != std::future_status::ready &&
         std::chrono::steady_clock::now() < first_goal_deadline) {
    cycle();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(first_goal_future.wait_for(0s), std::future_status::ready);
  auto first_goal_handle = first_goal_future.get();
  ASSERT_NE(first_goal_handle, nullptr);
  for (int cycle_index = 0; cycle_index < 10; ++cycle_index) {
    cycle();
    std::this_thread::sleep_for(5ms);
  }
  auto moving_position =
      resource_manager_view->claim_state_interface("grip_stroke/position");
  const auto position_before_same_target = moving_position.get_optional<double>();
  ASSERT_TRUE(position_before_same_target.has_value());

  auto replacement_future = action_client->async_send_goal(first_goal);
  const auto replacement_deadline = std::chrono::steady_clock::now() + 2s;
  while (replacement_future.wait_for(0s) != std::future_status::ready &&
         std::chrono::steady_clock::now() < replacement_deadline) {
    cycle();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(replacement_future.wait_for(0s), std::future_status::ready);
  auto replacement_handle = replacement_future.get();
  ASSERT_NE(replacement_handle, nullptr);
  for (int cycle_index = 0; cycle_index < 15; ++cycle_index) {
    cycle();
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_LT(*moving_position.get_optional<double>(),
            *position_before_same_target - 1e-6);

  auto first_result = action_client->async_get_result(first_goal_handle);
  auto replacement_result = action_client->async_get_result(replacement_handle);
  // Give the accepted goal time to enter the fake device. The controller is
  // then switched out without another update() call; only its deactivation
  // handoff may reach the following hardware write.
  ASSERT_EQ(switch_with_cycles({}, {"gripper_controller"}),
            controller_interface::return_type::OK);

  const auto time = manager.get_trigger_clock()->now();
  manager.write(time, rclcpp::Duration::from_seconds(0.01));
  manager.read(time, rclcpp::Duration::from_seconds(0.01));
  auto position =
      resource_manager_view->claim_state_interface("grip_stroke/position");
  const auto stopped_position = position.get_optional<double>();
  ASSERT_TRUE(stopped_position.has_value());
  ASSERT_TRUE(std::isfinite(*stopped_position));

  // A later write with no controller update must not replay the old target.
  for (int cycle = 0; cycle < 5; ++cycle) {
    manager.write(manager.get_trigger_clock()->now(),
                  rclcpp::Duration::from_seconds(0.05));
    manager.read(manager.get_trigger_clock()->now(),
                 rclcpp::Duration::from_seconds(0.05));
  }
  EXPECT_DOUBLE_EQ(*position.get_optional<double>(), *stopped_position);

  // Lifecycle interruption must deliver a terminal result and reject new
  // motion while inactive, even though the server remains to deliver results.
  const auto terminal_deadline = std::chrono::steady_clock::now() + 2s;
  while ((first_result.wait_for(0s) != std::future_status::ready ||
          replacement_result.wait_for(0s) != std::future_status::ready) &&
         std::chrono::steady_clock::now() < terminal_deadline) {
    executor->spin_some();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(first_result.wait_for(0s), std::future_status::ready);
  ASSERT_EQ(replacement_result.wait_for(0s), std::future_status::ready);
  EXPECT_EQ(first_result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(replacement_result.get().code, rclcpp_action::ResultCode::ABORTED);
  first_goal_handle.reset();
  replacement_handle.reset();

  Action::Goal inactive_goal_message;
  inactive_goal_message.command.name = {"grip_stroke"};
  inactive_goal_message.command.position = {0.100};
  inactive_goal_message.command.effort = {50.0};
  auto inactive_goal = action_client->async_send_goal(inactive_goal_message);
  ASSERT_EQ(executor->spin_until_future_complete(inactive_goal, 2s),
            rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(inactive_goal.get(), nullptr);
  action_client.reset();
  controller.reset();

  // Unload waits for the manager update loop to acknowledge its controller
  // list change. Keep that loop running as a real controller manager does.
  ASSERT_EQ(manager.cleanup_controller("gripper_controller"),
            controller_interface::return_type::OK);
  auto unload = std::async(std::launch::async, [&] {
    return manager.unload_controller("gripper_controller");
  });
  while (unload.wait_for(0s) != std::future_status::ready) {
    cycle();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(unload.get(), controller_interface::return_type::OK);
  auto reload = std::async(std::launch::async, [&] { return manager.load_controller(
      "gripper_controller",
      "onrobot_gripper_controllers/ParallelGripperActionController"); });
  while (reload.wait_for(0s) != std::future_status::ready) {
    cycle();
    std::this_thread::sleep_for(5ms);
  }
  controller = reload.get();
  ASSERT_NE(controller, nullptr);
  auto reconfigure = std::async(std::launch::async, [&] {
    return manager.configure_controller("gripper_controller");
  });
  while (reconfigure.wait_for(0s) != std::future_status::ready) {
    cycle();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(reconfigure.get(), controller_interface::return_type::OK);
  ASSERT_EQ(switch_with_cycles({"gripper_controller"}, {}),
            controller_interface::return_type::OK);
  ASSERT_EQ(switch_with_cycles({}, {"gripper_controller"}),
            controller_interface::return_type::OK);
}

TEST_F(ResourceManagerTest, ControllerManagerSwitchesConventionalAndRealtime) {
  const auto executor =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file",
                     N02_CONTROLLER_PARAMS_FILE});

  const auto resource_node =
      std::make_shared<rclcpp::Node>("n02_controller_switch_resource");
  executor->add_node(resource_node);
  auto resource_manager = std::make_unique<hardware_interface::ResourceManager>(
      fake2Fg7RealtimeUrdf(), resource_node->get_node_clock_interface(),
      resource_node->get_node_logging_interface(), true, 500);
  controller_manager::ControllerManager manager(
      std::move(resource_manager), executor, "n02_controller_switch", "",
      options);
  executor->add_node(manager.get_node_base_interface());

  auto conventional = manager.load_controller(
      "gripper_controller",
      "onrobot_gripper_controllers/ParallelGripperActionController");
  ASSERT_NE(conventional, nullptr);
  ASSERT_EQ(manager.configure_controller("gripper_controller"),
            controller_interface::return_type::OK);
  auto realtime = manager.load_controller(
      "realtime_controller",
      "onrobot_gripper_controllers/OnRobotRealtimeController");
  ASSERT_NE(realtime, nullptr);
  ASSERT_EQ(manager.configure_controller("realtime_controller"),
            controller_interface::return_type::OK);

  const auto cycle = [&] {
    const auto time = manager.get_trigger_clock()->now();
    static_cast<void>(manager.read(time, rclcpp::Duration::from_seconds(0.01)));
    static_cast<void>(manager.update(time, rclcpp::Duration::from_seconds(0.01)));
    static_cast<void>(manager.write(time, rclcpp::Duration::from_seconds(0.01)));
    executor->spin_some();
  };
  const auto switch_with_cycles = [&](const std::vector<std::string> &activate,
                                      const std::vector<std::string> &deactivate) {
    auto result = std::async(
        std::launch::async, [&manager, &activate, &deactivate] {
          return manager.switch_controller(
              activate, deactivate,
              controller_manager_msgs::srv::SwitchController::Request::STRICT,
              false, rclcpp::Duration::from_seconds(2.0));
        });
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (result.wait_for(0s) != std::future_status::ready &&
           std::chrono::steady_clock::now() < deadline) {
      cycle();
      std::this_thread::sleep_for(5ms);
    }
    return result.get();
  };

  ASSERT_EQ(switch_with_cycles({"gripper_controller"}, {}),
            controller_interface::return_type::OK);
  ASSERT_EQ(switch_with_cycles({"realtime_controller"},
                               {"gripper_controller"}),
            controller_interface::return_type::OK);
  ASSERT_EQ(switch_with_cycles({"gripper_controller"},
                               {"realtime_controller"}),
            controller_interface::return_type::OK);
  ASSERT_EQ(switch_with_cycles({}, {"gripper_controller"}),
            controller_interface::return_type::OK);

  ASSERT_EQ(manager.cleanup_controller("realtime_controller"),
            controller_interface::return_type::OK);
  ASSERT_EQ(manager.cleanup_controller("gripper_controller"),
            controller_interface::return_type::OK);
}

TEST_F(ResourceManagerTest, IsaacAdapterMapsPhysicalDofAndRealtimeCommands) {
  auto node = std::make_shared<rclcpp::Node>("onrobot_isaac_adapter_test");
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);

  hardware_interface::ResourceManagerParams parameters;
  parameters.robot_description = isaac2Fg7Urdf();
  parameters.clock = node->get_clock();
  parameters.logger = node->get_logger();
  parameters.executor = executor;
  parameters.activate_all = true;
  parameters.update_rate = 100;
  hardware_interface::ResourceManager manager(parameters, true);

  auto state_publisher = node->create_publisher<sensor_msgs::msg::JointState>(
      "isaac_test_states", rclcpp::QoS(1).reliable());
  std::optional<sensor_msgs::msg::JointState> last_command;
  auto command_subscription =
      node->create_subscription<sensor_msgs::msg::JointState>(
          "isaac_test_commands", rclcpp::QoS(1).reliable(),
          [&last_command](const sensor_msgs::msg::JointState &message) {
            last_command = message;
          });

  const auto spin_for = [&executor](std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  };
  spin_for(std::chrono::milliseconds(30));

  sensor_msgs::msg::JointState physical_state;
  physical_state.header.stamp = node->now();
  physical_state.name = {"finger_stroke"};
  physical_state.position = {0.0095};
  physical_state.velocity = {0.001};
  physical_state.effort = {0.0};
  state_publisher->publish(physical_state);
  spin_for(std::chrono::milliseconds(20));
  static_cast<void>(
      manager.read(node->now(), rclcpp::Duration::from_seconds(0.01)));

  auto task_position = manager.claim_state_interface("grip_stroke/position");
  auto task_velocity = manager.claim_state_interface("grip_stroke/velocity");
  auto raw_position =
      manager.claim_state_interface("finger_stroke/measured_position");
  auto valid = manager.claim_state_interface("grip_stroke/task_position_valid");
  auto force_valid = manager.claim_state_interface("grip_stroke/force_valid");
  ASSERT_TRUE(task_position.get_optional<double>().has_value());
  ASSERT_TRUE(task_velocity.get_optional<double>().has_value());
  ASSERT_TRUE(raw_position.get_optional<double>().has_value());
  ASSERT_TRUE(valid.get_optional<double>().has_value());
  ASSERT_TRUE(force_valid.get_optional<double>().has_value());
  EXPECT_NEAR(*task_position.get_optional<double>(), 0.0535, 1e-12);
  EXPECT_NEAR(*task_velocity.get_optional<double>(), 0.001 / 0.019 * 0.107,
              1e-12);
  EXPECT_NEAR(*raw_position.get_optional<double>(), 0.020, 1e-12);
  EXPECT_DOUBLE_EQ(*valid.get_optional<double>(), 1.0);
  EXPECT_DOUBLE_EQ(*force_valid.get_optional<double>(), 0.0);

  {
    auto position = manager.claim_command_interface("grip_stroke/position");
    ASSERT_TRUE(position.set_value(0.107));
    static_cast<void>(
        manager.write(node->now(), rclcpp::Duration::from_seconds(0.01)));
    spin_for(std::chrono::milliseconds(20));
    ASSERT_TRUE(last_command.has_value());
    ASSERT_EQ(last_command->name, std::vector<std::string>({"finger_stroke"}));
    ASSERT_EQ(last_command->position.size(), 1U);
    ASSERT_EQ(last_command->velocity.size(), 1U);
    ASSERT_EQ(last_command->effort.size(), 1U);
    EXPECT_DOUBLE_EQ(last_command->position.front(), 0.019);
    EXPECT_TRUE(std::isnan(last_command->velocity.front()));
    EXPECT_TRUE(std::isnan(last_command->effort.front()));
  }

  {
    auto mode = manager.claim_command_interface("grip_stroke/realtime_mode");
    auto position =
        manager.claim_command_interface("grip_stroke/realtime_task_position");
    auto velocity =
        manager.claim_command_interface("grip_stroke/realtime_task_velocity");
    auto force = manager.claim_command_interface("grip_stroke/realtime_force");
    auto sequence = manager.claim_command_interface(
        "grip_stroke/realtime_command_sequence");
    ASSERT_TRUE(mode.set_value(0.0));
    ASSERT_TRUE(position.set_value(0.0535));
    ASSERT_TRUE(velocity.set_value(0.107));
    ASSERT_TRUE(force.set_value(10.0));
    ASSERT_TRUE(sequence.set_value(1.0));
    static_cast<void>(
        manager.write(node->now(), rclcpp::Duration::from_seconds(0.01)));
    spin_for(std::chrono::milliseconds(20));
    ASSERT_TRUE(last_command.has_value());
    EXPECT_DOUBLE_EQ(last_command->position.front(), 0.0095);
    EXPECT_TRUE(std::isnan(last_command->velocity.front()));

    ASSERT_TRUE(mode.set_value(1.0));
    ASSERT_TRUE(velocity.set_value(0.107));
    ASSERT_TRUE(sequence.set_value(2.0));
    static_cast<void>(
        manager.write(node->now(), rclcpp::Duration::from_seconds(0.01)));
    spin_for(std::chrono::milliseconds(20));
    EXPECT_TRUE(std::isnan(last_command->position.front()));
    EXPECT_DOUBLE_EQ(last_command->velocity.front(), 0.019);

    ASSERT_TRUE(mode.set_value(-1.0));
    ASSERT_TRUE(sequence.set_value(3.0));
    static_cast<void>(
        manager.write(node->now(), rclcpp::Duration::from_seconds(0.01)));
    spin_for(std::chrono::milliseconds(20));
    EXPECT_TRUE(std::isfinite(last_command->position.front()));
    EXPECT_TRUE(std::isnan(last_command->velocity.front()));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  static_cast<void>(
      manager.read(node->now(), rclcpp::Duration::from_seconds(0.01)));
  auto connection =
      manager.claim_state_interface("grip_stroke/connection_state");
  EXPECT_DOUBLE_EQ(*valid.get_optional<double>(), 0.0);
  EXPECT_DOUBLE_EQ(*connection.get_optional<double>(), 1.0);
  (void)command_subscription;
}

TEST_F(ResourceManagerTest, IsaacAdapterMaps2Fg14Coordinates) {
  auto node =
      std::make_shared<rclcpp::Node>("onrobot_2fg14_isaac_adapter_test");
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);

  hardware_interface::ResourceManagerParams parameters;
  parameters.robot_description = isaac2Fg14Urdf();
  parameters.clock = node->get_clock();
  parameters.logger = node->get_logger();
  parameters.executor = executor;
  parameters.activate_all = true;
  parameters.update_rate = 100;
  hardware_interface::ResourceManager manager(parameters, true);

  auto state_publisher = node->create_publisher<sensor_msgs::msg::JointState>(
      "isaac_test_states", rclcpp::QoS(1).reliable());
  std::optional<sensor_msgs::msg::JointState> last_command;
  auto command_subscription =
      node->create_subscription<sensor_msgs::msg::JointState>(
          "isaac_test_commands", rclcpp::QoS(1).reliable(),
          [&last_command](const sensor_msgs::msg::JointState &message) {
            last_command = message;
          });
  const auto spin_for = [&executor](std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  };
  spin_for(std::chrono::milliseconds(30));

  sensor_msgs::msg::JointState physical_state;
  physical_state.header.stamp = node->now();
  physical_state.name = {"finger_stroke"};
  physical_state.position = {0.0125};
  physical_state.velocity = {0.0025};
  state_publisher->publish(physical_state);
  spin_for(std::chrono::milliseconds(20));
  static_cast<void>(
      manager.read(node->now(), rclcpp::Duration::from_seconds(0.01)));

  auto task_position = manager.claim_state_interface("grip_stroke/position");
  auto task_velocity = manager.claim_state_interface("grip_stroke/velocity");
  auto raw_position =
      manager.claim_state_interface("finger_stroke/measured_position");
  ASSERT_TRUE(task_position.get_optional<double>().has_value());
  ASSERT_TRUE(task_velocity.get_optional<double>().has_value());
  ASSERT_TRUE(raw_position.get_optional<double>().has_value());
  EXPECT_NEAR(*task_position.get_optional<double>(), 0.070, 1e-12);
  EXPECT_NEAR(*task_velocity.get_optional<double>(), 0.014, 1e-12);
  EXPECT_NEAR(*raw_position.get_optional<double>(), 0.026, 1e-12);

  auto position = manager.claim_command_interface("grip_stroke/position");
  ASSERT_TRUE(position.set_value(0.140));
  static_cast<void>(
      manager.write(node->now(), rclcpp::Duration::from_seconds(0.01)));
  spin_for(std::chrono::milliseconds(20));
  ASSERT_TRUE(last_command.has_value());
  EXPECT_DOUBLE_EQ(last_command->position.front(), 0.025);
  EXPECT_TRUE(std::isnan(last_command->velocity.front()));
  (void)command_subscription;
}

TEST_F(ResourceManagerTest, IsaacAdapterMapsRg2AngularCoordinates) {
  auto node = std::make_shared<rclcpp::Node>("onrobot_rg2_isaac_adapter_test");
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);

  hardware_interface::ResourceManagerParams parameters;
  parameters.robot_description = isaacRg2Urdf();
  parameters.clock = node->get_clock();
  parameters.logger = node->get_logger();
  parameters.executor = executor;
  parameters.activate_all = true;
  parameters.update_rate = 100;
  hardware_interface::ResourceManager manager(parameters, true);

  auto state_publisher = node->create_publisher<sensor_msgs::msg::JointState>(
      "isaac_test_states", rclcpp::QoS(1).reliable());
  std::optional<sensor_msgs::msg::JointState> last_command;
  auto command_subscription =
      node->create_subscription<sensor_msgs::msg::JointState>(
          "isaac_test_commands", rclcpp::QoS(1).reliable(),
          [&last_command](const sensor_msgs::msg::JointState &message) {
            last_command = message;
          });
  const auto spin_for = [&executor](std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  };
  spin_for(std::chrono::milliseconds(30));

  constexpr double maximum_angle_rad = 1.22277767395;
  sensor_msgs::msg::JointState physical_state;
  physical_state.header.stamp = node->now();
  physical_state.name = {"finger_joint"};
  physical_state.position = {maximum_angle_rad};
  physical_state.velocity = {0.2};
  state_publisher->publish(physical_state);
  spin_for(std::chrono::milliseconds(20));
  static_cast<void>(
      manager.read(node->now(), rclcpp::Duration::from_seconds(0.01)));

  auto task_position = manager.claim_state_interface("grip_stroke/position");
  auto angular_position = manager.claim_state_interface(
      "finger_joint/measured_angular_position");
  auto angular_velocity = manager.claim_state_interface(
      "finger_joint/measured_angular_velocity");
  ASSERT_TRUE(task_position.get_optional<double>().has_value());
  ASSERT_TRUE(angular_position.get_optional<double>().has_value());
  ASSERT_TRUE(angular_velocity.get_optional<double>().has_value());
  EXPECT_NEAR(*task_position.get_optional<double>(), 0.110, 1e-12);
  EXPECT_DOUBLE_EQ(*angular_position.get_optional<double>(), maximum_angle_rad);
  EXPECT_DOUBLE_EQ(*angular_velocity.get_optional<double>(), 0.2);

  {
    auto position = manager.claim_command_interface("grip_stroke/position");
    ASSERT_TRUE(position.set_value(0.110));
    static_cast<void>(
        manager.write(node->now(), rclcpp::Duration::from_seconds(0.01)));
    spin_for(std::chrono::milliseconds(20));
    ASSERT_TRUE(last_command.has_value());
    ASSERT_EQ(last_command->name, std::vector<std::string>({"finger_joint"}));
    EXPECT_NEAR(last_command->position.front(), maximum_angle_rad, 1e-12);
    EXPECT_TRUE(std::isnan(last_command->velocity.front()));
  }

  {
    auto mode = manager.claim_command_interface("grip_stroke/realtime_mode");
    auto position =
        manager.claim_command_interface("grip_stroke/realtime_task_position");
    auto velocity = manager.claim_command_interface(
        "grip_stroke/realtime_mechanism_angular_velocity");
    auto force = manager.claim_command_interface("grip_stroke/realtime_force");
    auto sequence = manager.claim_command_interface(
        "grip_stroke/realtime_command_sequence");
    ASSERT_TRUE(mode.set_value(1.0));
    ASSERT_TRUE(position.set_value(0.110));
    ASSERT_TRUE(velocity.set_value(-0.4));
    ASSERT_TRUE(force.set_value(10.0));
    ASSERT_TRUE(sequence.set_value(1.0));
    static_cast<void>(
        manager.write(node->now(), rclcpp::Duration::from_seconds(0.01)));
    spin_for(std::chrono::milliseconds(20));
    ASSERT_TRUE(last_command.has_value());
    EXPECT_TRUE(std::isnan(last_command->position.front()));
    EXPECT_DOUBLE_EQ(last_command->velocity.front(), -0.4);
  }
  (void)command_subscription;
}

TEST_F(ResourceManagerTest, IsaacAdapterMapsRg6AngularCoordinates) {
  auto node = std::make_shared<rclcpp::Node>("onrobot_rg6_isaac_adapter_test");
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);

  hardware_interface::ResourceManagerParams parameters;
  parameters.robot_description = isaacRg6Urdf();
  parameters.clock = node->get_clock();
  parameters.logger = node->get_logger();
  parameters.executor = executor;
  parameters.activate_all = true;
  parameters.update_rate = 100;
  hardware_interface::ResourceManager manager(parameters, true);

  auto state_publisher = node->create_publisher<sensor_msgs::msg::JointState>(
      "isaac_test_states", rclcpp::QoS(1).reliable());
  std::optional<sensor_msgs::msg::JointState> last_command;
  auto command_subscription =
      node->create_subscription<sensor_msgs::msg::JointState>(
          "isaac_test_commands", rclcpp::QoS(1).reliable(),
          [&last_command](const sensor_msgs::msg::JointState &message) {
            last_command = message;
          });
  const auto spin_for = [&executor](std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  };
  spin_for(std::chrono::milliseconds(30));

  constexpr double maximum_angle_rad = 1.1928;
  sensor_msgs::msg::JointState physical_state;
  physical_state.header.stamp = node->now();
  physical_state.name = {"finger_joint"};
  physical_state.position = {maximum_angle_rad};
  physical_state.velocity = {0.2};
  state_publisher->publish(physical_state);
  spin_for(std::chrono::milliseconds(20));
  static_cast<void>(
      manager.read(node->now(), rclcpp::Duration::from_seconds(0.01)));

  auto task_position = manager.claim_state_interface("grip_stroke/position");
  auto angular_position = manager.claim_state_interface(
      "finger_joint/measured_angular_position");
  auto angular_velocity = manager.claim_state_interface(
      "finger_joint/measured_angular_velocity");
  ASSERT_TRUE(task_position.get_optional<double>().has_value());
  ASSERT_TRUE(angular_position.get_optional<double>().has_value());
  ASSERT_TRUE(angular_velocity.get_optional<double>().has_value());
  EXPECT_NEAR(*task_position.get_optional<double>(), 0.160, 1e-12);
  EXPECT_DOUBLE_EQ(*angular_position.get_optional<double>(), maximum_angle_rad);
  EXPECT_DOUBLE_EQ(*angular_velocity.get_optional<double>(), 0.2);

  {
    auto position = manager.claim_command_interface("grip_stroke/position");
    ASSERT_TRUE(position.set_value(0.160));
    static_cast<void>(
        manager.write(node->now(), rclcpp::Duration::from_seconds(0.01)));
    spin_for(std::chrono::milliseconds(20));
    ASSERT_TRUE(last_command.has_value());
    ASSERT_EQ(last_command->name, std::vector<std::string>({"finger_joint"}));
    EXPECT_NEAR(last_command->position.front(), maximum_angle_rad, 1e-12);
    EXPECT_TRUE(std::isnan(last_command->velocity.front()));
  }

  {
    auto mode = manager.claim_command_interface("grip_stroke/realtime_mode");
    auto position =
        manager.claim_command_interface("grip_stroke/realtime_task_position");
    auto velocity = manager.claim_command_interface(
        "grip_stroke/realtime_mechanism_angular_velocity");
    auto force = manager.claim_command_interface("grip_stroke/realtime_force");
    auto sequence = manager.claim_command_interface(
        "grip_stroke/realtime_command_sequence");
    ASSERT_TRUE(mode.set_value(1.0));
    ASSERT_TRUE(position.set_value(0.160));
    ASSERT_TRUE(velocity.set_value(-0.4));
    ASSERT_TRUE(force.set_value(10.0));
    ASSERT_TRUE(sequence.set_value(1.0));
    static_cast<void>(
        manager.write(node->now(), rclcpp::Duration::from_seconds(0.01)));
    spin_for(std::chrono::milliseconds(20));
    ASSERT_TRUE(last_command.has_value());
    EXPECT_TRUE(std::isnan(last_command->position.front()));
    EXPECT_DOUBLE_EQ(last_command->velocity.front(), -0.4);
  }

  (void)command_subscription;
}

TEST_F(ResourceManagerTest, RealActivationDoesNotResendMeasuredPosition) {
  using namespace std::chrono_literals;

  ModbusTcpTestServer server;
  auto node = std::make_shared<rclcpp::Node>(
      "onrobot_resource_manager_no_activation_command_test");
  hardware_interface::ResourceManager manager(
      real2Fg7Urdf(server.port()), node->get_node_clock_interface(),
      node->get_node_logging_interface(), true, 100);

  auto position = manager.claim_command_interface("grip_stroke/position");
  ASSERT_TRUE(position.set_value(0.020));
  const auto writes_at_activation = server.function_count(16);
  static_cast<void>(
      manager.write(node->now(), rclcpp::Duration::from_seconds(0.01)));
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(server.function_count(16), writes_at_activation);

  ASSERT_TRUE(position.set_value(0.030));
  static_cast<void>(
      manager.write(node->now(), rclcpp::Duration::from_seconds(0.01)));
  const auto command_deadline = std::chrono::steady_clock::now() + 500ms;
  while (server.function_count(16) == writes_at_activation &&
         std::chrono::steady_clock::now() < command_deadline) {
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_GT(server.function_count(16), writes_at_activation);

  rclcpp_lifecycle::State inactive(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  EXPECT_EQ(manager.set_component_state("OnRobot2FG7TestSystem", inactive),
            hardware_interface::return_type::OK);
  rclcpp_lifecycle::State unconfigured(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  EXPECT_EQ(manager.set_component_state("OnRobot2FG7TestSystem", unconfigured),
            hardware_interface::return_type::OK);
}

TEST_F(ResourceManagerTest, RealRgActivationDoesNotResendMeasuredPosition) {
  using namespace std::chrono_literals;

  ModbusTcpTestServer server(0x0021);
  server.set_register(0x010B, 1501);
  auto node = std::make_shared<rclcpp::Node>(
      "onrobot_rg_resource_manager_no_activation_command_test");
  hardware_interface::ResourceManager manager(
      realRg6Urdf(server.port()), node->get_node_clock_interface(),
      node->get_node_logging_interface(), true, 100);

  auto position = manager.claim_command_interface("grip_stroke/position");
  ASSERT_TRUE(position.set_value(0.1501));
  const auto writes_at_activation = server.function_count(16);
  static_cast<void>(
      manager.write(node->now(), rclcpp::Duration::from_seconds(0.01)));
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(server.function_count(16), writes_at_activation);

  ASSERT_TRUE(position.set_value(0.100));
  static_cast<void>(
      manager.write(node->now(), rclcpp::Duration::from_seconds(0.01)));
  const auto command_deadline = std::chrono::steady_clock::now() + 500ms;
  while (server.function_count(16) == writes_at_activation &&
         std::chrono::steady_clock::now() < command_deadline) {
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_GT(server.function_count(16), writes_at_activation);

  rclcpp_lifecycle::State inactive(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  EXPECT_EQ(manager.set_component_state("OnRobotRG6TestSystem", inactive),
            hardware_interface::return_type::OK);
  rclcpp_lifecycle::State unconfigured(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  EXPECT_EQ(manager.set_component_state("OnRobotRG6TestSystem", unconfigured),
            hardware_interface::return_type::OK);
}

TEST_F(ResourceManagerTest,
       RealtimePositionConvertsRosSiToGuideV14RegisterImage) {
  using namespace std::chrono_literals;

  ModbusTcpTestServer server;
  auto node = std::make_shared<rclcpp::Node>(
      "onrobot_resource_manager_realtime_position_encoding_test");
  hardware_interface::ResourceManager manager(
      real2Fg7RealtimeUrdf(server.port()), node->get_node_clock_interface(),
      node->get_node_logging_interface(), true, 500);

  auto mode = manager.claim_command_interface("grip_stroke/realtime_mode");
  auto position =
      manager.claim_command_interface("grip_stroke/realtime_task_position");
  auto velocity =
      manager.claim_command_interface("grip_stroke/realtime_task_velocity");
  auto force = manager.claim_command_interface("grip_stroke/realtime_force");
  auto sequence =
      manager.claim_command_interface("grip_stroke/realtime_command_sequence");

  ASSERT_TRUE(mode.set_value(0.0));
  ASSERT_TRUE(position.set_value(0.0604));
  ASSERT_TRUE(velocity.set_value(0.015));
  ASSERT_TRUE(force.set_value(0.0));
  ASSERT_TRUE(sequence.set_value(1.0));

  const auto realtime_before = server.function_count(23);
  static_cast<void>(
      manager.write(node->now(), rclcpp::Duration::from_seconds(0.002)));
  const auto command_deadline = std::chrono::steady_clock::now() + 500ms;
  while (server.function_count(23) == realtime_before &&
         std::chrono::steady_clock::now() < command_deadline) {
    // Admission is deliberately nonblocking: a Busy handoff is retried on the
    // next manager cycle, not by a blocking write callback.
    ASSERT_EQ(manager.read(node->now(), rclcpp::Duration::from_seconds(0.002)).result,
              hardware_interface::return_type::OK);
    ASSERT_EQ(manager.write(node->now(), rclcpp::Duration::from_seconds(0.002)).result,
              hardware_interface::return_type::OK);
    std::this_thread::sleep_for(2ms);
  }

  ASSERT_GT(server.function_count(23), realtime_before);
  EXPECT_EQ(server.register_value(0x0000), 604U);
  EXPECT_EQ(server.register_value(0x0001), 0U);
  EXPECT_EQ(server.register_value(0x0002), 150U);
  EXPECT_EQ(server.register_value(0x0003), 6U);

  ASSERT_TRUE(mode.set_value(-1.0));
  ASSERT_TRUE(sequence.set_value(2.0));
  static_cast<void>(
      manager.write(node->now(), rclcpp::Duration::from_seconds(0.002)));
  const auto stop_deadline = std::chrono::steady_clock::now() + 500ms;
  while (server.register_value(0x0003) != 3U &&
         std::chrono::steady_clock::now() < stop_deadline) {
    ASSERT_EQ(manager.read(node->now(), rclcpp::Duration::from_seconds(0.002)).result,
              hardware_interface::return_type::OK);
    ASSERT_EQ(manager.write(node->now(), rclcpp::Duration::from_seconds(0.002)).result,
              hardware_interface::return_type::OK);
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_EQ(server.register_value(0x0003), 3U);

  rclcpp_lifecycle::State inactive(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  EXPECT_EQ(manager.set_component_state("OnRobot2FG7TestSystem", inactive),
            hardware_interface::return_type::OK);
  rclcpp_lifecycle::State unconfigured(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  EXPECT_EQ(manager.set_component_state("OnRobot2FG7TestSystem", unconfigured),
            hardware_interface::return_type::OK);
}

TEST_F(ResourceManagerTest,
       UnqualifiedRealtimeForceModeFailsClosedWithoutFunction23Motion) {
  using namespace std::chrono_literals;

  ModbusTcpTestServer server;
  auto node = std::make_shared<rclcpp::Node>(
      "onrobot_resource_manager_realtime_force_gate_test");
  hardware_interface::ResourceManager manager(
      real2Fg7RealtimeUrdf(server.port()), node->get_node_clock_interface(),
      node->get_node_logging_interface(), true, 500);

  auto mode = manager.claim_command_interface("grip_stroke/realtime_mode");
  auto position =
      manager.claim_command_interface("grip_stroke/realtime_task_position");
  auto velocity =
      manager.claim_command_interface("grip_stroke/realtime_task_velocity");
  auto force = manager.claim_command_interface("grip_stroke/realtime_force");
  auto sequence =
      manager.claim_command_interface("grip_stroke/realtime_command_sequence");

  ASSERT_TRUE(mode.set_value(2.0));
  ASSERT_TRUE(position.set_value(0.050));
  ASSERT_TRUE(velocity.set_value(0.005));
  ASSERT_TRUE(force.set_value(30.0));
  ASSERT_TRUE(sequence.set_value(1.0));

  const auto realtime_before = server.function_count(23);
  static_cast<void>(
      manager.write(node->now(), rclcpp::Duration::from_seconds(0.002)));

  const auto stop_deadline = std::chrono::steady_clock::now() + 500ms;
  while (server.register_value(0x0003) != 3U &&
         std::chrono::steady_clock::now() < stop_deadline) {
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_EQ(server.register_value(0x0003), 3U);
  EXPECT_EQ(server.function_count(23), realtime_before);

  rclcpp_lifecycle::State inactive(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  EXPECT_EQ(manager.set_component_state("OnRobot2FG7TestSystem", inactive),
            hardware_interface::return_type::OK);
  rclcpp_lifecycle::State unconfigured(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  EXPECT_EQ(manager.set_component_state("OnRobot2FG7TestSystem", unconfigured),
            hardware_interface::return_type::OK);
}

TEST_F(ResourceManagerTest, TransportStallDoesNotEnterManagerCallbacks) {
  using namespace std::chrono_literals;

  ModbusTcpTestServer server;
  auto node = std::make_shared<rclcpp::Node>(
      "onrobot_resource_manager_transport_stall_test");
  hardware_interface::ResourceManager manager(
      real2Fg7Urdf(server.port()), node->get_node_clock_interface(),
      node->get_node_logging_interface(), true, 500);

  const auto reads_before = server.function_count(3);
  server.set_response_delay(100ms);
  const auto observation_deadline = std::chrono::steady_clock::now() + 1s;
  while (server.function_count(3) == reads_before &&
         std::chrono::steady_clock::now() < observation_deadline) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_GT(server.function_count(3), reads_before)
      << "session worker did not enter the delayed transport transaction";

  auto position = manager.claim_command_interface("grip_stroke/position");
  ASSERT_TRUE(position.set_value(0.050));
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.002);
  const auto started = std::chrono::steady_clock::now();
  for (std::size_t cycle = 0; cycle < 1000; ++cycle) {
    const auto time = node->now();
    static_cast<void>(manager.write(time, period));
    static_cast<void>(manager.read(time, period));
  }
  const auto elapsed = std::chrono::steady_clock::now() - started;

  // The server is holding the sole transport worker for 100 ms. The manager
  // path must still complete memory-only work comfortably below that stall.
  EXPECT_LT(elapsed, 50ms);

  server.set_response_delay(0ms);
  rclcpp_lifecycle::State inactive(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  EXPECT_EQ(manager.set_component_state("OnRobot2FG7TestSystem", inactive),
            hardware_interface::return_type::OK);
  rclcpp_lifecycle::State unconfigured(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  EXPECT_EQ(manager.set_component_state("OnRobot2FG7TestSystem", unconfigured),
            hardware_interface::return_type::OK);
}

TEST_F(ResourceManagerTest, ConventionalActionForceAdmissionReachesWire) {
  using Action = control_msgs::action::ParallelGripperCommand;
  for (const std::string model : {"2fg7", "2fg14", "rg2", "rg6"}) {
    SCOPED_TRACE(model);
    std::cout << "Action-to-wire model: " << model << std::endl;
    const bool two_fg = model == "2fg7" || model == "2fg14";
    const bool large = model == "2fg14" || model == "rg6";
    ModbusTcpTestServer peer(two_fg ? (large ? 0xc1 : 0xc0)
                                    : (large ? 0x21 : 0x20));
    if (!two_fg) {
      peer.set_register(0x010b, 200);
      peer.set_register(0x0113, 200);
    }
    auto urdf = two_fg ? real2Fg7Urdf(peer.port()) : realRg6Urdf(peer.port());
    const auto replace = [&](const std::string &from, const std::string &to) {
      const auto at = urdf.find(from);
      if (at == std::string::npos)
        throw std::runtime_error("test URDF marker missing");
      urdf.replace(at, from.size(), to);
    };
    if (two_fg && large)
      replace("name=\"model\">2fg7", "name=\"model\">2fg14");
    if (model == "rg2") {
      replace("name=\"model\">rg6", "name=\"model\">rg2");
      replace("name=\"safe_width_max_mm\">160",
              "name=\"safe_width_max_mm\">110");
    }
    if (two_fg)
      replace("</hardware>",
              "<param name=\"speed_percent\">1</param></hardware>");
    if (two_fg) addConventionalSpeedResource(urdf);
    auto executor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto resource_node =
        std::make_shared<rclcpp::Node>("force_admission_resource");
    executor->add_node(resource_node);
    auto resources = std::make_unique<hardware_interface::ResourceManager>(
        urdf, resource_node->get_node_clock_interface(),
        resource_node->get_node_logging_interface(), true, 100);
    rclcpp::NodeOptions options;
    options.arguments(
        {"--ros-args", "--params-file", N02_CONTROLLER_PARAMS_FILE});
    controller_manager::ControllerManager manager(
        std::move(resources), executor, "n02_controller_manager", "", options);
    executor->add_node(manager.get_node_base_interface());
    auto node = std::make_shared<rclcpp::Node>("force_admission_client");
    executor->add_node(node);
    const auto controller = manager.load_controller(
        "gripper_controller",
        "onrobot_gripper_controllers/ParallelGripperActionController");
    ASSERT_NE(controller, nullptr);
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        rejecting_parameter_callback;
    if (two_fg) {
      controller->get_node()->declare_parameter<bool>(
          "test_reject_atomic_transaction", false);
      rejecting_parameter_callback =
          controller->get_node()->add_on_set_parameters_callback(
              [](const std::vector<rclcpp::Parameter> &parameters) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto &parameter : parameters) {
                  if (parameter.get_name() ==
                          "test_reject_atomic_transaction" &&
                      parameter.as_bool()) {
                    result.successful = false;
                    result.reason = "test peer rejected atomic transaction";
                    break;
                  }
                }
                return result;
              });
      ASSERT_TRUE(controller->get_node()
                      ->set_parameter(
                          rclcpp::Parameter("conventional_speed_control", true))
                      .successful);
    }
    const double default_force = two_fg ? (large ? 40.0 : 20.0) : 10.0;
    // A different action default catches unintended activation writes and
    // distinguishes omitted effort from the explicit hardware-default zero.
    const double action_default = two_fg ? 80.0 : (large ? 60.0 : 30.0);
    ASSERT_TRUE(
        controller->get_node()
            ->set_parameter(rclcpp::Parameter("max_effort", action_default))
            .successful);
    ASSERT_EQ(manager.configure_controller("gripper_controller"),
              controller_interface::return_type::OK);
    const auto cycle = [&](bool spin_executor = true) {
      const auto now = manager.get_trigger_clock()->now();
      const auto period = rclcpp::Duration::from_seconds(0.005);
      manager.read(now, period);
      manager.update(now, period);
      manager.write(now, period);
      if (spin_executor)
        executor->spin_some();
      std::this_thread::sleep_for(5ms);
    };
    const auto wait = [&](const auto &predicate, bool spin_executor = true) {
      const auto deadline = std::chrono::steady_clock::now() + 4s;
      while (!predicate() && std::chrono::steady_clock::now() < deadline)
        cycle(spin_executor);
      return predicate();
    };
    const auto switch_controller = [&](bool activate) {
      auto future = std::async(std::launch::async, [&] {
        return manager.switch_controller(
            activate ? std::vector<std::string>{"gripper_controller"}
                     : std::vector<std::string>{},
            activate ? std::vector<std::string>{}
                     : std::vector<std::string>{"gripper_controller"},
            controller_manager_msgs::srv::SwitchController::Request::STRICT,
            false, rclcpp::Duration::from_seconds(2));
      });
      EXPECT_TRUE(wait(
          [&] { return future.wait_for(0s) == std::future_status::ready; }, false));
      EXPECT_EQ(future.get(), controller_interface::return_type::OK);
    };
    switch_controller(true);
    if (two_fg) {
      const auto speed = controller->get_node()->set_parameters_atomically(
          {rclcpp::Parameter("conventional_speed_percent", 25)});
      ASSERT_TRUE(speed.successful) << speed.reason;

      // Another callback rejects this whole transaction. The committed ROS
      // parameter and the next native command must both remain at 25%.
      const auto rejected_atomic_update =
          controller->get_node()->set_parameters_atomically(
              {rclcpp::Parameter("conventional_speed_percent", 100),
               rclcpp::Parameter("test_reject_atomic_transaction", true)});
      ASSERT_FALSE(rejected_atomic_update.successful);
      EXPECT_EQ(controller->get_node()
                    ->get_parameter("conventional_speed_percent")
                    .as_int(),
                25);
    }
    const auto client = rclcpp_action::create_client<Action>(
        node, "/gripper_controller/gripper_cmd");
    ASSERT_TRUE(wait([&] { return client->action_server_is_ready(); }));
    const auto send = [&](double aperture, std::vector<double> effort) {
      Action::Goal goal;
      goal.command.name = {"grip_stroke"};
      goal.command.position = {aperture};
      goal.command.effort = std::move(effort);
      auto response = client->async_send_goal(goal);
      EXPECT_TRUE(wait(
          [&] { return response.wait_for(0s) == std::future_status::ready; }));
      return response.get();
    };
    // Count only native grip writes, not power setup or explicit Stop writes.
    const auto grip_count = [&] {
      size_t count = 0;
      for (const auto &pdu : peer.requests()) {
        if (pdu.size() == (two_fg ? 14u : 12u) && pdu[0] == 16 && pdu[1] == 0 &&
            pdu[2] == 0 && pdu.back() == (two_fg ? 1 : 0x10))
          ++count;
      }
      return count;
    };
    const auto assert_aborted = [&](const auto &handle) {
      ASSERT_NE(handle, nullptr);
      auto result = client->async_get_result(handle);
      ASSERT_TRUE(wait(
          [&] { return result.wait_for(0s) == std::future_status::ready; }));
      const auto outcome = result.get();
      EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::ABORTED);
      EXPECT_FALSE(outcome.result->stalled);
      EXPECT_FALSE(outcome.result->reached_goal);
    };
    const double invalid =
        two_fg ? (large ? 39.0 : 19.0) : (large ? 121.0 : 41.0);
    // Reject even at current position: upstream reach detection must wait for
    // admission.
    const auto before = grip_count();
    EXPECT_EQ(before, 0u) << "Activation must not send a grip";
    assert_aborted(send(0.020, {invalid}));
    for (int i = 0; i < 30; ++i)
      cycle();
    EXPECT_EQ(grip_count(), before);

    for (const auto &malformed : std::vector<std::vector<double>>{
             {-1.0},
             {std::numeric_limits<double>::quiet_NaN()},
             {std::numeric_limits<double>::infinity()},
             {20.0, 30.0}}) {
      EXPECT_EQ(send(0.030, malformed), nullptr);
    }
    EXPECT_EQ(grip_count(), before);
    const double valid = two_fg ? 60.0 : (large ? 80.0 : 30.0);
    auto accepted = send(0.030, {valid});
    ASSERT_NE(accepted, nullptr);
    ASSERT_TRUE(wait([&] { return grip_count() > before; }));
    EXPECT_EQ(peer.register_value(two_fg ? 1 : 0), valid * (two_fg ? 1 : 10));
    EXPECT_EQ(peer.register_value(two_fg ? 0 : 1), 300);
    if (two_fg) {
      EXPECT_EQ(peer.register_value(2), 25);
    }

    // Replace a valid in-flight goal with an invalid one. Stop may be written,
    // but neither the rejected target nor a default-force replay may be sent.
    const auto valid_writes = grip_count();
    assert_aborted(send(0.040, {invalid}));
    auto retired = client->async_get_result(accepted);
    ASSERT_TRUE(wait(
        [&] { return retired.wait_for(0s) == std::future_status::ready; }));
    EXPECT_NE(retired.get().code, rclcpp_action::ResultCode::SUCCEEDED);
    for (int i = 0; i < 30; ++i)
      cycle();
    EXPECT_EQ(grip_count(), valid_writes);

    if (two_fg) {
      const auto speed = controller->get_node()->set_parameters_atomically(
          {rclcpp::Parameter("conventional_speed_percent", 75)});
      ASSERT_TRUE(speed.successful) << speed.reason;
    }

    // Live conventional ceiling, not the higher model rating (peer advertises
    // 100 N).
    if (two_fg) {
      assert_aborted(send(0.030, {101.0}));
      EXPECT_EQ(grip_count(), valid_writes);
    }
    assert_aborted(send(0.500, {valid}));
    EXPECT_EQ(grip_count(), valid_writes);

    // Rebind loaned effort after a full deactivate/reactivate, then issue a
    // fresh valid request. A stationary peer still permits legitimate stall
    // success.
    switch_controller(false);
    switch_controller(true);
    auto fresh = send(0.030, {valid});
    ASSERT_NE(fresh, nullptr);
    ASSERT_TRUE(wait([&] { return grip_count() > valid_writes; }));
    EXPECT_EQ(peer.register_value(two_fg ? 1 : 0), valid * (two_fg ? 1 : 10));
    if (two_fg) {
      EXPECT_EQ(peer.register_value(2), 75);
    }
    auto result = client->async_get_result(fresh);
    ASSERT_TRUE(
        wait([&] { return result.wait_for(0s) == std::future_status::ready; }));
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_TRUE(result.get().result->stalled);
    for (const auto &[effort, expected_force] :
         std::vector<std::pair<std::vector<double>, double>>{
             {{default_force}, default_force}, {{}, action_default},
             {{0.0}, default_force}}) {
      const auto count = grip_count();
      auto target = send(0.020, effort);
      ASSERT_NE(target, nullptr);
      ASSERT_TRUE(wait([&] { return grip_count() > count; }));
      EXPECT_EQ(peer.register_value(two_fg ? 1 : 0),
                expected_force * (two_fg ? 1 : 10));
      auto terminal = client->async_get_result(target);
      ASSERT_TRUE(wait(
          [&] { return terminal.wait_for(0s) == std::future_status::ready; }));
      EXPECT_EQ(terminal.get().code, rclcpp_action::ResultCode::SUCCEEDED);
      EXPECT_TRUE(terminal.get().result->reached_goal);
    }
    switch_controller(false);
    // ControllerManager lifecycle operations can wait for an update-loop
    // acknowledgement. Pump read/update/write without running non-RT callbacks
    // on that same thread: an activity callback may need the controller-list
    // lock held by the lifecycle operation waiting for our next update.
    auto cleanup = std::async(std::launch::async, [&] {
      return manager.cleanup_controller("gripper_controller");
    });
    ASSERT_TRUE(wait([&] { return cleanup.wait_for(0s) == std::future_status::ready; }, false));
    ASSERT_EQ(cleanup.get(), controller_interface::return_type::OK);
    auto unload = std::async(std::launch::async, [&] {
      return manager.unload_controller("gripper_controller");
    });
    ASSERT_TRUE(wait([&] { return unload.wait_for(0s) == std::future_status::ready; }, false));
    ASSERT_EQ(unload.get(), controller_interface::return_type::OK);
    executor->remove_node(node);
    executor->remove_node(manager.get_node_base_interface());
    executor->remove_node(resource_node);
    std::cout << "Action-to-wire complete: " << model << std::endl;
  }
}

TEST_F(ResourceManagerTest,
       RuntimeConventionalSpeedActionReachesTwoFingerRtuWire) {
  using Action = control_msgs::action::ParallelGripperCommand;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

  ModbusRtuTestServer peer;
  auto urdf = real2Fg7RtuUrdf(peer.device());
  addConventionalSpeedResource(urdf);
  auto executor =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto resource_node = std::make_shared<rclcpp::Node>(
      "runtime_speed_rtu_resource");
  executor->add_node(resource_node);
  auto resources = std::make_unique<hardware_interface::ResourceManager>(
      urdf, resource_node->get_node_clock_interface(),
      resource_node->get_node_logging_interface(), true, 100);
  rclcpp::NodeOptions options;
  options.arguments(
      {"--ros-args", "--params-file", N02_CONTROLLER_PARAMS_FILE});
  controller_manager::ControllerManager manager(
      std::move(resources), executor, "runtime_speed_rtu_manager", "", options);
  executor->add_node(manager.get_node_base_interface());
  auto node = std::make_shared<rclcpp::Node>("runtime_speed_rtu_client");
  executor->add_node(node);
  const auto controller = manager.load_controller(
      "gripper_controller",
      "onrobot_gripper_controllers/ParallelGripperActionController");
  ASSERT_NE(controller, nullptr);
  ASSERT_TRUE(controller->get_node()
                  ->set_parameter(rclcpp::Parameter(
                      "conventional_speed_control", true))
                  .successful);
  ASSERT_EQ(manager.configure_controller("gripper_controller"),
            controller_interface::return_type::OK);

  const auto cycle = [&](bool spin_executor = true) {
    const auto now = manager.get_trigger_clock()->now();
    const auto period = rclcpp::Duration::from_seconds(0.005);
    manager.read(now, period);
    manager.update(now, period);
    manager.write(now, period);
    if (spin_executor) executor->spin_some();
    std::this_thread::sleep_for(5ms);
  };
  const auto wait = [&](const auto &predicate, bool spin_executor = true) {
    const auto deadline = std::chrono::steady_clock::now() + 4s;
    while (!predicate() && std::chrono::steady_clock::now() < deadline)
      cycle(spin_executor);
    return predicate();
  };
  const auto lifecycle = [&](const bool activate) {
    auto operation = std::async(std::launch::async, [&] {
      return manager.switch_controller(
          activate ? std::vector<std::string>{"gripper_controller"}
                   : std::vector<std::string>{},
          activate ? std::vector<std::string>{}
                   : std::vector<std::string>{"gripper_controller"},
          controller_manager_msgs::srv::SwitchController::Request::STRICT,
          false, rclcpp::Duration::from_seconds(2));
    });
    ASSERT_TRUE(wait(
        [&] { return operation.wait_for(0s) == std::future_status::ready; },
        false));
    ASSERT_EQ(operation.get(), controller_interface::return_type::OK);
  };
  lifecycle(true);

  const auto client = rclcpp_action::create_client<Action>(
      node, "/gripper_controller/gripper_cmd");
  ASSERT_TRUE(wait([&] { return client->action_server_is_ready(); }));
  const auto set_speed = [&](const int percent) {
    const auto result = controller->get_node()->set_parameters_atomically(
        {rclcpp::Parameter("conventional_speed_percent", percent)});
    ASSERT_TRUE(result.successful) << result.reason;
  };
  const auto send = [&](const double target) {
    Action::Goal goal;
    goal.command.name = {"grip_stroke"};
    goal.command.position = {target};
    goal.command.effort = {60.0};
    auto response = client->async_send_goal(goal);
    if (!wait(
            [&] { return response.wait_for(0s) == std::future_status::ready; })) {
      ADD_FAILURE() << "RTU action goal response timed out";
      return GoalHandle::SharedPtr{};
    }
    const auto goal_handle = response.get();
    if (!goal_handle) {
      ADD_FAILURE() << "RTU action goal was rejected";
      return GoalHandle::SharedPtr{};
    }
    return goal_handle;
  };
  const auto cancel_and_wait = [&](const auto &goal_handle) {
    auto cancel = client->async_cancel_goal(goal_handle);
    ASSERT_TRUE(wait(
        [&] { return cancel.wait_for(0s) == std::future_status::ready; }));
    auto result = client->async_get_result(goal_handle);
    ASSERT_TRUE(wait(
        [&] { return result.wait_for(0s) == std::future_status::ready; }));
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  };

  // Drive the public runtime parameter, then an action, through the resource
  // manager and real RTU framing. The peer sees the native 2FG write block.
  set_speed(25);
  const auto writes_before_first = peer.function_count(16);
  const auto first = send(0.030);
  ASSERT_TRUE(wait([&] { return peer.function_count(16) > writes_before_first; }));
  EXPECT_EQ(peer.register_value(0), 300);
  EXPECT_EQ(peer.register_value(1), 60);
  EXPECT_EQ(peer.register_value(2), 25);
  EXPECT_EQ(peer.register_value(3), 1);
  cancel_and_wait(first);

  // A completed lifecycle releases the controller's admission state. The next
  // goal must carry a new run-time selection, rather than startup/default speed.
  set_speed(75);
  const auto writes_before_second = peer.function_count(16);
  const auto second = send(0.040);
  ASSERT_TRUE(wait([&] { return peer.function_count(16) > writes_before_second; }));
  EXPECT_EQ(peer.register_value(0), 400);
  EXPECT_EQ(peer.register_value(1), 60);
  EXPECT_EQ(peer.register_value(2), 75);
  EXPECT_EQ(peer.register_value(3), 1);
  cancel_and_wait(second);

  lifecycle(false);
  auto cleanup = std::async(std::launch::async, [&] {
    return manager.cleanup_controller("gripper_controller");
  });
  ASSERT_TRUE(wait(
      [&] { return cleanup.wait_for(0s) == std::future_status::ready; }, false));
  ASSERT_EQ(cleanup.get(), controller_interface::return_type::OK);
  auto unload = std::async(std::launch::async, [&] {
    return manager.unload_controller("gripper_controller");
  });
  ASSERT_TRUE(wait(
      [&] { return unload.wait_for(0s) == std::future_status::ready; }, false));
  ASSERT_EQ(unload.get(), controller_interface::return_type::OK);
  executor->remove_node(node);
  executor->remove_node(manager.get_node_base_interface());
  executor->remove_node(resource_node);
}

// Read the actual manager-owned handle for deterministic scheduling tests;
// release the temporary claim before loading the controller.
struct InspectCommandLoan : hardware_interface::LoanedCommandInterface {
  explicit InspectCommandLoan(hardware_interface::LoanedCommandInterface &&loan)
      : LoanedCommandInterface(std::move(loan)) {}
  hardware_interface::CommandInterface *handle() { return &command_interface_; }
};

TEST_F(ResourceManagerTest, ConventionalTerminalHandoffOrdersStopBeforeHardwareAdmission) {
  using Action = control_msgs::action::ParallelGripperCommand;
  for (const std::string model : {"2fg7", "2fg14", "rg2", "rg6"}) {
    // Cancel before dispatch, after dispatch, after admission, and with a
    // contended Stop handle; replacement before write, including contention;
    // and a fresh goal accepted immediately after Cancel, before retirement.
    for (int schedule = 0; schedule < 7; ++schedule) {
      SCOPED_TRACE(model + " terminal schedule=" + std::to_string(schedule));
      const bool replacement = schedule == 4 || schedule == 5;
      const bool early_fresh = schedule == 6;
      const bool busy = schedule == 3 || schedule == 5;
      const bool two_fg = model == "2fg7" || model == "2fg14";
      const bool large = model == "2fg14" || model == "rg6";
      ModbusTcpTestServer peer(two_fg ? (large ? 0xc1 : 0xc0) : (large ? 0x21 : 0x20));
      if (!two_fg) {
        peer.set_register(0x010b, 200);
        peer.set_register(0x0113, 200);
      }
      auto urdf = two_fg ? real2Fg7Urdf(peer.port()) : realRg6Urdf(peer.port());
      const auto replace = [&](const std::string &from, const std::string &to) {
        const auto at = urdf.find(from);
        ASSERT_NE(at, std::string::npos);
        urdf.replace(at, from.size(), to);
      };
      if (two_fg && large) replace("name=\"model\">2fg7", "name=\"model\">2fg14");
      if (model == "rg2") {
        replace("name=\"model\">rg6", "name=\"model\">rg2");
        replace("name=\"safe_width_max_mm\">160", "name=\"safe_width_max_mm\">110");
      }
      auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      auto node = std::make_shared<rclcpp::Node>("cancel_order_client");
      executor->add_node(node);
      auto resources = std::make_unique<hardware_interface::ResourceManager>(
          urdf, node->get_node_clock_interface(), node->get_node_logging_interface(), true, 100);
      hardware_interface::CommandInterface *stop;
      {
        InspectCommandLoan loan(resources->claim_command_interface("grip_stroke/stop_command_sequence"));
        stop = loan.handle();
      }
      rclcpp::NodeOptions options;
      options.arguments({"--ros-args", "--params-file", N02_CONTROLLER_PARAMS_FILE});
      controller_manager::ControllerManager manager(
          std::move(resources), executor, "n02_controller_manager", "", options);
      executor->add_node(manager.get_node_base_interface());
      ASSERT_NE(manager.load_controller("gripper_controller",
          "onrobot_gripper_controllers/ParallelGripperActionController"), nullptr);
      ASSERT_EQ(manager.configure_controller("gripper_controller"), controller_interface::return_type::OK);
      const auto period = rclcpp::Duration::from_seconds(.01);
      const auto cycle = [&](bool spin = true) {
        const auto now = manager.get_trigger_clock()->now();
        manager.read(now, period); manager.update(now, period); manager.write(now, period);
        if (spin) executor->spin_some();
        std::this_thread::sleep_for(5ms);
      };
      const auto wait = [&](auto predicate, bool spin = true) {
        const auto deadline = std::chrono::steady_clock::now() + 3s;
        while (!predicate() && std::chrono::steady_clock::now() < deadline) cycle(spin);
        return predicate();
      };
      const auto lifecycle = [&](auto operation) {
        auto future = std::async(std::launch::async, operation);
        EXPECT_TRUE(wait([&] { return future.wait_for(0s) == std::future_status::ready; }, false));
        EXPECT_EQ(future.get(), controller_interface::return_type::OK);
      };
      lifecycle([&] { return manager.switch_controller({"gripper_controller"}, {},
          controller_manager_msgs::srv::SwitchController::Request::STRICT, false,
          rclcpp::Duration::from_seconds(2)); });
      auto client = rclcpp_action::create_client<Action>(node, "/gripper_controller/gripper_cmd");
      ASSERT_TRUE(client->wait_for_action_server(2s));
      const auto count = [&](bool grips) {
        size_t result = 0;
        for (const auto &pdu : peer.requests()) {
          const auto selector = grips ? (two_fg ? 1 : 0x10) : (two_fg ? 3 : 0x28);
          if (pdu.size() == (two_fg ? 14u : 12u) && pdu[0] == 16 &&
              pdu[1] == 0 && pdu[2] == 0 && pdu.back() == selector) ++result;
        }
        return result;
      };
      Action::Goal goal;
      goal.command.name = {"grip_stroke"};
      goal.command.position = {.030};
      goal.command.effort = {two_fg ? 60.0 : (large ? 80.0 : 30.0)};
      auto sent = client->async_send_goal(goal);
      ASSERT_EQ(executor->spin_until_future_complete(sent, 2s), rclcpp::FutureReturnCode::SUCCESS);
      auto handle = sent.get();
      ASSERT_NE(handle, nullptr);
      if (schedule > 0) {
        const auto now = manager.get_trigger_clock()->now();
        manager.read(now, period); manager.update(now, period);
      }
      if (schedule == 2) {
        ASSERT_TRUE(wait([&] { return count(true) > 0; }));
      }
      const auto grips_before = count(true);
      const auto stops_before = count(false);

      // A separate thread holds exactly the mutex shared with hardware write.
      // Cancel and write must both return promptly under contention, not block.
      std::promise<void> locked, unlock;
      auto release = unlock.get_future();
      std::future<void> locker;
      if (busy) {
        locker = std::async(std::launch::async, [&] {
          std::unique_lock<std::shared_mutex> guard(stop->get_mutex());
          locked.set_value(); release.wait();
        });
        locked.get_future().wait();
        manager.write(manager.get_trigger_clock()->now(), period);
        EXPECT_EQ(count(true), grips_before);
      }
      rclcpp_action::ClientGoalHandle<Action>::SharedPtr replacement_handle;
      if (replacement) {
        goal.command.position = {.040};
        auto next = client->async_send_goal(goal);
        const auto ready = executor->spin_until_future_complete(next, 2s);
        if (busy) { unlock.set_value(); locker.get(); }
        ASSERT_EQ(ready, rclcpp::FutureReturnCode::SUCCESS);
        replacement_handle = next.get();
        ASSERT_NE(replacement_handle, nullptr);
      } else {
        auto cancel = client->async_cancel_goal(handle);
        const auto ready = executor->spin_until_future_complete(cancel, 2s);
        if (busy) { unlock.set_value(); locker.get(); }
        ASSERT_EQ(ready, rclcpp::FutureReturnCode::SUCCESS);
        EXPECT_EQ(cancel.get()->goals_canceling.size(), busy ? 0u : 1u);
      }
      if (early_fresh) {
        goal.command.position = {.040};
        auto next = client->async_send_goal(goal);
        ASSERT_EQ(executor->spin_until_future_complete(next, 2s), rclcpp::FutureReturnCode::SUCCESS);
        replacement_handle = next.get();
        ASSERT_NE(replacement_handle, nullptr);
      }
      if (!busy) {
        // No next update: the accepted Cancel itself must already fence write.
        manager.write(manager.get_trigger_clock()->now(), period);
        std::this_thread::sleep_for(50ms);
        // Updates may yield while an executor callback owns the lifecycle
        // lock. Even after Stop is acknowledged, write must not replay the
        // canceled positive event while output retirement is still pending.
        for (int i = 0; i < 10; ++i) {
          const auto now = manager.get_trigger_clock()->now();
          manager.read(now, period);
          manager.write(now, period);
          std::this_thread::sleep_for(5ms);
        }
        EXPECT_EQ(count(true), grips_before);
      }
      auto terminal = client->async_get_result(handle);
      ASSERT_TRUE(wait([&] { return count(false) > stops_before &&
          terminal.wait_for(0s) == std::future_status::ready; }));
      EXPECT_EQ(terminal.get().code, busy ? rclcpp_action::ResultCode::ABORTED :
                                           rclcpp_action::ResultCode::CANCELED);
      if ((replacement && !busy) || early_fresh) {
        ASSERT_TRUE(wait([&] { return count(true) == grips_before + 1; }));
        EXPECT_EQ(peer.register_value(two_fg ? 0 : 1), 400);
      }
      if (replacement && busy) {
        auto rejected = client->async_get_result(replacement_handle);
        ASSERT_TRUE(wait([&] { return rejected.wait_for(0s) == std::future_status::ready; }));
        EXPECT_EQ(rejected.get().code, rclcpp_action::ResultCode::ABORTED);
      }
      for (int i = 0; i < 10; ++i) cycle();
      const auto expected_grips = grips_before + ((replacement && !busy) || early_fresh ? 1u : 0u);
      EXPECT_EQ(count(true), expected_grips) << "Retired command replayed after Stop";
      // Fresh intent still works; a permanent Stop/rejection fence is not a fix.
      goal.command.position = {.050};
      auto fresh = client->async_send_goal(goal);
      ASSERT_TRUE(wait([&] { return fresh.wait_for(0s) == std::future_status::ready; }));
      ASSERT_NE(fresh.get(), nullptr);
      ASSERT_TRUE(wait([&] { return count(true) > expected_grips; }));
      EXPECT_EQ(peer.register_value(two_fg ? 0 : 1), 500);
      lifecycle([&] { return manager.switch_controller({}, {"gripper_controller"},
          controller_manager_msgs::srv::SwitchController::Request::STRICT, false,
          rclcpp::Duration::from_seconds(2)); });
      lifecycle([&] { return manager.cleanup_controller("gripper_controller"); });
      lifecycle([&] { return manager.unload_controller("gripper_controller"); });
      std::cout << "Terminal ordering exercised: " << model << " schedule=" << schedule << std::endl;
    }
  }
}

} // namespace

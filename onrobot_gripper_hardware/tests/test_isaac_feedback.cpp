#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/resource_manager_params.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace {

using namespace std::chrono_literals;

std::string isaacUrdf(const std::string &topic, int timeout_ms = 50) {
  return R"(
<?xml version="1.0"?>
<robot name="isaac_feedback_test">
  <link name="base_link"/><link name="task_link"/><link name="finger_link"/>
  <joint name="grip_stroke" type="prismatic">
    <parent link="base_link"/><child link="task_link"/><axis xyz="1 0 0"/>
    <limit lower="0" upper="0.107" effort="100" velocity="1"/>
  </joint>
  <joint name="finger_stroke" type="prismatic">
    <parent link="base_link"/><child link="finger_link"/><axis xyz="1 0 0"/>
    <limit lower="0" upper="0.019" effort="100" velocity="1"/>
  </joint>
  <ros2_control name="IsaacFeedbackTestSystem" type="system">
    <hardware>
      <plugin>onrobot_gripper_hardware/OnRobotIsaacSystem</plugin>
      <param name="model">2fg7</param>
      <param name="task_min_m">0.0</param><param name="task_max_m">0.107</param>
      <param name="physical_min_m">0.0</param><param name="physical_max_m">0.019</param>
      <param name="isaac_joint_commands_topic">)" +
         topic + R"(</param>
      <param name="isaac_joint_states_topic">)" +
         topic + R"(_states</param>
      <param name="isaac_state_timeout_ms">)" +
         std::to_string(timeout_ms) +
         R"(</param>
    </hardware>
    <joint name="grip_stroke">
      <command_interface name="position"/>
      <command_interface name="realtime_mode"/>
      <command_interface name="realtime_task_position"/>
      <command_interface name="realtime_task_velocity"/>
      <command_interface name="realtime_force"/>
      <command_interface name="realtime_command_sequence"/>
      <command_interface name="fault_recovery_command_sequence"/>
      <command_interface name="stop_command_sequence"/>
      <command_interface name="conventional_command_sequence"/>
      <state_interface name="position"/><state_interface name="velocity"/>
      <state_interface name="effort"/><state_interface name="task_position_valid"/>
      <state_interface name="force_valid"/><state_interface name="busy"/>
      <state_interface name="active_mode"/><state_interface name="connection_state"/>
      <state_interface name="faulted"/><state_interface name="fault_code"/>
      <state_interface name="sample_sequence"/><state_interface name="sample_age"/>
      <state_interface name="requested_command_sequence"/>
      <state_interface name="applied_command_sequence"/>
    </joint>
    <joint name="finger_stroke">
      <state_interface name="position"/><state_interface name="velocity"/>
      <state_interface name="measured_position"/><state_interface name="measured_velocity"/>
      <state_interface name="position_valid"/><state_interface name="velocity_valid"/>
    </joint>
  </ros2_control>
</robot>)";
}

sensor_msgs::msg::JointState sample(int sec, double position,
                                    std::optional<double> velocity = 0.0) {
  sensor_msgs::msg::JointState message;
  message.header.stamp.sec = sec;
  message.name = {"finger_stroke"};
  message.position = {position};
  if (velocity.has_value()) {
    message.velocity = {*velocity};
  }
  return message;
}

class IsaacFeedbackTest : public ::testing::Test {
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

  struct Harness {
    std::shared_ptr<rclcpp::Node> node =
        std::make_shared<rclcpp::Node>("isaac_feedback_peer");
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state;
    std::vector<sensor_msgs::msg::JointState> commands;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub;
    std::unique_ptr<hardware_interface::ResourceManager> manager;

    explicit Harness(const std::string &id, int timeout_ms = 50) {
      executor->add_node(node);
      const auto topic = "/isaac_feedback_" + id;
      state = node->create_publisher<sensor_msgs::msg::JointState>(
          topic + "_states", rclcpp::QoS(1).reliable());
      command_sub = node->create_subscription<sensor_msgs::msg::JointState>(
          topic, rclcpp::QoS(1).reliable(),
          [this](const sensor_msgs::msg::JointState &message) {
            commands.push_back(message);
          });
      hardware_interface::ResourceManagerParams parameters;
      parameters.robot_description = isaacUrdf(topic, timeout_ms);
      parameters.clock = node->get_clock();
      parameters.logger = node->get_logger();
      parameters.executor = executor;
      parameters.activate_all = true;
      parameters.update_rate = 100;
      manager = std::make_unique<hardware_interface::ResourceManager>(
          parameters, true);
    }

    void spinFor(std::chrono::milliseconds duration) {
      const auto deadline = std::chrono::steady_clock::now() + duration;
      while (std::chrono::steady_clock::now() < deadline) {
        executor->spin_some();
        std::this_thread::sleep_for(1ms);
      }
    }

    void publish(const sensor_msgs::msg::JointState &message) {
      state->publish(message);
      spinFor(10ms);
    }

    void read() {
      static_cast<void>(
          manager->read(node->now(), rclcpp::Duration::from_seconds(0.01)));
    }

    void write() {
      static_cast<void>(
          manager->write(node->now(), rclcpp::Duration::from_seconds(0.01)));
      spinFor(5ms);
    }

    double stateValue(const std::string &name) {
      auto handle = manager->claim_state_interface(name);
      const auto value = handle.get_optional<double>();
      EXPECT_TRUE(value.has_value()) << name;
      return value.value_or(std::numeric_limits<double>::quiet_NaN());
    }
  };
};

TEST_F(IsaacFeedbackTest, StartupWaitsForFeedbackAndDiscardsCommand) {
  Harness harness("startup");
  harness.spinFor(20ms);
  harness.write();
  EXPECT_TRUE(harness.commands.empty());
  EXPECT_TRUE(std::isnan(harness.stateValue("grip_stroke/position")));

  auto command =
      harness.manager->claim_command_interface("grip_stroke/position");
  ASSERT_TRUE(command.set_value(0.050));
  harness.write();
  EXPECT_TRUE(harness.commands.empty());

  harness.publish(sample(1, 0.0095, 0.001));
  harness.read();
  harness.write();
  ASSERT_FALSE(harness.commands.empty());
  EXPECT_DOUBLE_EQ(harness.commands.back().position.front(), 0.0095);

  // A controller rewriting the same pre-feedback value still has no new
  // operator intent. A numerically changed target is the explicit re-arm.
  ASSERT_TRUE(command.set_value(0.050));
  harness.write();
  EXPECT_DOUBLE_EQ(harness.commands.back().position.front(), 0.0095);
  ASSERT_TRUE(command.set_value(0.040));
  harness.write();
  EXPECT_NEAR(harness.commands.back().position.front(), 0.040 / 0.107 * 0.019,
              1e-12);
}

TEST_F(IsaacFeedbackTest,
       ConventionalGoalIdentityReissuesSameTargetAfterNumericDeduplication) {
  Harness harness("conventional_identity");
  harness.publish(sample(1, 0.0095, 0.001));
  harness.read();
  harness.write();
  harness.commands.clear();

  auto position =
      harness.manager->claim_command_interface("grip_stroke/position");
  auto identity = harness.manager->claim_command_interface(
      "grip_stroke/conventional_command_sequence");
  ASSERT_TRUE(position.set_value(0.050));
  harness.write();
  ASSERT_EQ(harness.commands.size(), 1U);
  const auto first_command = harness.commands.back().position.front();

  // Isaac is a streaming backend, so ordinary writes may refresh the target.
  harness.write();
  ASSERT_EQ(harness.commands.size(), 2U);

  // Stop retires every event published before it. The controller buffers a
  // replacement until hardware consumes Stop, then exports a fresh event,
  // even when the new target is numerically unchanged.
  auto stop =
      harness.manager->claim_command_interface("grip_stroke/stop_command_sequence");
  ASSERT_TRUE(stop.set_value(1.0));
  ASSERT_TRUE(identity.set_value(1.0));
  harness.write();
  ASSERT_EQ(harness.commands.size(), 3U);
  EXPECT_TRUE(std::isfinite(harness.commands.back().position.front()));
  EXPECT_TRUE(std::isnan(harness.commands.back().velocity.front()));

  harness.write();
  ASSERT_EQ(harness.commands.size(), 4U);
  EXPECT_DOUBLE_EQ(harness.commands.back().position.front(), 0.0095);
  ASSERT_TRUE(std::isnan(identity.get_optional<double>().value()));
  ASSERT_TRUE(position.set_value(0.050));
  ASSERT_TRUE(identity.set_value(2.0));
  harness.write();
  ASSERT_EQ(harness.commands.size(), 5U);
  EXPECT_DOUBLE_EQ(harness.commands.back().position.front(), first_command);
}

TEST_F(IsaacFeedbackTest, SampleIdentityAgeAndMissingVelocityAreTruthful) {
  Harness harness("identity");
  harness.publish(sample(1, 0.0095, 0.001));
  harness.read();
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/sample_sequence"), 1.0);
  const auto age = harness.stateValue("grip_stroke/sample_age");
  EXPECT_GE(age, 0.0);
  harness.read();
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/sample_sequence"), 1.0);
  EXPECT_GT(harness.stateValue("grip_stroke/sample_age"), age);

  harness.publish(sample(1, 0.012, 0.001));
  harness.read();
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/sample_sequence"), 1.0);
  EXPECT_NEAR(harness.stateValue("grip_stroke/position"), 0.0535, 1e-12);

  harness.publish(sample(2, 0.012, std::nullopt));
  harness.read();
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/sample_sequence"), 2.0);
  EXPECT_TRUE(std::isnan(harness.stateValue("grip_stroke/velocity")));
  EXPECT_DOUBLE_EQ(harness.stateValue("finger_stroke/velocity_valid"), 0.0);
  EXPECT_TRUE(std::isnan(harness.stateValue("grip_stroke/effort")));
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/force_valid"), 0.0);
}

TEST_F(IsaacFeedbackTest, FeedbackLossStopsOnceAndRequiresExplicitRecovery) {
  Harness harness("loss", 20);
  harness.publish(sample(1, 0.0095, 0.001));
  harness.read();
  harness.write();
  const auto commands_before_loss = harness.commands.size();
  std::this_thread::sleep_for(30ms);
  harness.read();
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/task_position_valid"), 0.0);
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/faulted"), 1.0);
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/fault_code"), 3.0);
  harness.write();
  ASSERT_GT(harness.commands.size(), commands_before_loss);
  const auto stop = harness.commands.back();
  EXPECT_TRUE(std::isnan(stop.position.front()));
  EXPECT_DOUBLE_EQ(stop.velocity.front(), 0.0);
  const auto commands_after_stop = harness.commands.size();
  harness.read();
  harness.write();
  EXPECT_EQ(harness.commands.size(), commands_after_stop);

  auto recovery = harness.manager->claim_command_interface(
      "grip_stroke/fault_recovery_command_sequence");
  ASSERT_TRUE(recovery.set_value(1.0));
  harness.write();
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/faulted"), 1.0);

  harness.publish(sample(2, 0.010, 0.001));
  harness.read();
  harness.write();
  EXPECT_EQ(harness.commands.size(), commands_after_stop);

  // The same recovery sequence is accepted now that current feedback exists;
  // an early recovery request did not consume it.
  ASSERT_TRUE(recovery.set_value(1.0));
  harness.write();
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/faulted"), 0.0);

  auto position =
      harness.manager->claim_command_interface("grip_stroke/position");
  ASSERT_TRUE(position.set_value(0.015));
  harness.write();
  ASSERT_GT(harness.commands.size(), commands_after_stop);
  EXPECT_NEAR(harness.commands.back().position.front(), 0.015 / 0.107 * 0.019,
              1e-12);
}

TEST_F(IsaacFeedbackTest, RollbackRetiresStreamUntilNewBaselineAdvances) {
  Harness harness("rollback");
  harness.publish(sample(10, 0.0095));
  harness.read();
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/sample_sequence"), 1.0);
  harness.publish(sample(5, 0.012));
  harness.read();
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/task_position_valid"), 0.0);
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/faulted"), 1.0);
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/fault_code"), 4.0);
  harness.publish(sample(6, 0.012));
  harness.read();
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/sample_sequence"), 2.0);
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/task_position_valid"), 1.0);
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/faulted"), 1.0);
}

TEST_F(IsaacFeedbackTest, InvalidRealtimePayloadFailsClosed) {
  const double qnan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  struct InvalidPayload {
    std::string id;
    double mode;
    double position;
    double velocity;
    double force;
    double sequence;
    double expected_fault;
  };
  const std::vector<InvalidPayload> cases{
      {"position_nan", 0.0, qnan, 0.1, 10.0, 1.0, 6.0},
      {"position_inf", 0.0, infinity, 0.1, 10.0, 1.0, 6.0},
      {"velocity_nan", 0.0, 0.050, qnan, 10.0, 1.0, 6.0},
      {"velocity_inf", 0.0, 0.050, infinity, 10.0, 1.0, 6.0},
      {"force_nan", 0.0, 0.050, 0.1, qnan, 1.0, 6.0},
      {"force_inf", 0.0, 0.050, 0.1, infinity, 1.0, 6.0},
      {"fractional_mode", 0.5, 0.050, 0.1, 10.0, 1.0, 6.0},
      {"negative_sequence", 0.0, 0.050, 0.1, 10.0, -1.0, 6.0},
      {"unsupported_force", 2.0, 0.050, 0.1, 10.0, 1.0, 7.0},
  };

  for (const auto &item : cases) {
    SCOPED_TRACE(item.id);
    Harness harness("invalid_rt_" + item.id);
    harness.publish(sample(1, 0.0095, 0.001));
    harness.read();
    harness.write();
    const auto commands_before = harness.commands.size();

    auto mode =
        harness.manager->claim_command_interface("grip_stroke/realtime_mode");
    auto position = harness.manager->claim_command_interface(
        "grip_stroke/realtime_task_position");
    auto velocity = harness.manager->claim_command_interface(
        "grip_stroke/realtime_task_velocity");
    auto force =
        harness.manager->claim_command_interface("grip_stroke/realtime_force");
    auto sequence = harness.manager->claim_command_interface(
        "grip_stroke/realtime_command_sequence");
    ASSERT_TRUE(mode.set_value(item.mode));
    ASSERT_TRUE(position.set_value(item.position));
    ASSERT_TRUE(velocity.set_value(item.velocity));
    ASSERT_TRUE(force.set_value(item.force));
    ASSERT_TRUE(sequence.set_value(item.sequence));
    harness.write();

    EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/faulted"), 1.0);
    EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/fault_code"),
                     item.expected_fault);
    ASSERT_GT(harness.commands.size(), commands_before);
    EXPECT_TRUE(std::isnan(harness.commands.back().position.front()));
    EXPECT_DOUBLE_EQ(harness.commands.back().velocity.front(), 0.0);
  }
}

TEST_F(IsaacFeedbackTest, ReactivationWaitsWhenPreviousFeedbackExpired) {
  Harness harness("reactivation", 20);
  harness.publish(sample(1, 0.0095, 0.001));
  harness.read();
  harness.write();

  rclcpp_lifecycle::State inactive(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  ASSERT_EQ(
      harness.manager->set_component_state("IsaacFeedbackTestSystem", inactive),
      hardware_interface::return_type::OK);
  harness.spinFor(10ms);
  harness.commands.clear();

  std::this_thread::sleep_for(30ms);
  rclcpp_lifecycle::State active(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(
      harness.manager->set_component_state("IsaacFeedbackTestSystem", active),
      hardware_interface::return_type::OK);
  harness.spinFor(10ms);
  EXPECT_TRUE(harness.commands.empty());

  harness.read();
  harness.write();
  EXPECT_TRUE(harness.commands.empty());
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/task_position_valid"), 0.0);
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/faulted"), 1.0);
}

TEST_F(IsaacFeedbackTest, RecoveryRequiresFeedbackCurrentAtWrite) {
  Harness harness("stale_recovery", 20);
  harness.publish(sample(1, 0.0095, 0.001));
  harness.read();
  harness.write();
  std::this_thread::sleep_for(30ms);
  harness.read();
  harness.write();

  harness.publish(sample(2, 0.010, 0.001));
  harness.read();
  ASSERT_DOUBLE_EQ(harness.stateValue("grip_stroke/task_position_valid"), 1.0);
  ASSERT_DOUBLE_EQ(harness.stateValue("grip_stroke/faulted"), 1.0);

  std::this_thread::sleep_for(30ms);
  // A newer callback receipt must not make the expired sample applied by the
  // previous read look current. Recovery remains blocked until read() applies
  // fresh measured state.
  harness.publish(sample(3, 0.011, 0.001));
  auto recovery = harness.manager->claim_command_interface(
      "grip_stroke/fault_recovery_command_sequence");
  ASSERT_TRUE(recovery.set_value(1.0));
  harness.write();
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/faulted"), 1.0);

  harness.read();
  ASSERT_TRUE(recovery.set_value(1.0));
  harness.write();
  EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/faulted"), 0.0);
}

TEST_F(IsaacFeedbackTest, RecoveryRejectsRetiredConventionalTarget) {
  Harness harness("conventional_replay", 20);
  harness.publish(sample(1, 0.0095, 0.001));
  harness.read();
  harness.write();

  auto position =
      harness.manager->claim_command_interface("grip_stroke/position");
  ASSERT_TRUE(position.set_value(0.050));
  harness.write();
  std::this_thread::sleep_for(30ms);
  harness.read();
  harness.write();

  harness.publish(sample(2, 0.010, 0.001));
  harness.read();
  auto recovery = harness.manager->claim_command_interface(
      "grip_stroke/fault_recovery_command_sequence");
  ASSERT_TRUE(recovery.set_value(1.0));
  harness.write();
  ASSERT_DOUBLE_EQ(harness.stateValue("grip_stroke/faulted"), 0.0);

  ASSERT_TRUE(position.set_value(0.050));
  harness.write();
  ASSERT_FALSE(harness.commands.empty());
  EXPECT_DOUBLE_EQ(harness.commands.back().position.front(), 0.010);

  ASSERT_TRUE(position.set_value(0.040));
  harness.write();
  EXPECT_NEAR(harness.commands.back().position.front(), 0.040 / 0.107 * 0.019,
              1e-12);
}

TEST_F(IsaacFeedbackTest, DeactivationDoesNotRefreshExpiredHold) {
  Harness harness("stale_deactivation", 20);
  harness.publish(sample(1, 0.0095, 0.001));
  harness.read();
  harness.write();
  harness.commands.clear();

  // No read observes this expiry before the lifecycle transition. Deactivate
  // must recheck monotonic freshness instead of trusting cached validity.
  std::this_thread::sleep_for(30ms);
  rclcpp_lifecycle::State inactive(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  ASSERT_EQ(
      harness.manager->set_component_state("IsaacFeedbackTestSystem", inactive),
      hardware_interface::return_type::OK);
  harness.spinFor(10ms);

  ASSERT_EQ(harness.commands.size(), 1U);
  ASSERT_EQ(harness.commands.back().position.size(), 1U);
  ASSERT_EQ(harness.commands.back().velocity.size(), 1U);
  EXPECT_TRUE(std::isnan(harness.commands.back().position.front()));
  EXPECT_DOUBLE_EQ(harness.commands.back().velocity.front(), 0.0);
}

TEST_F(IsaacFeedbackTest, MalformedTimestampAndDuplicateJointFailClosed) {
  for (const std::string id : {"timestamp", "duplicate_joint"}) {
    SCOPED_TRACE(id);
    Harness harness("malformed_" + id);
    harness.publish(sample(1, 0.0095, 0.001));
    harness.read();
    harness.write();
    const auto commands_before = harness.commands.size();

    auto malformed = sample(2, 0.012, 0.002);
    if (id == "timestamp") {
      malformed.header.stamp.nanosec = 1000000000U;
    } else {
      malformed.name.push_back("finger_stroke");
      malformed.position.push_back(0.013);
      malformed.velocity.push_back(0.003);
    }
    harness.publish(malformed);
    harness.read();
    harness.write();

    EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/sample_sequence"), 1.0);
    EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/task_position_valid"),
                     0.0);
    EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/faulted"), 1.0);
    EXPECT_DOUBLE_EQ(harness.stateValue("grip_stroke/fault_code"), 4.0);
    ASSERT_EQ(harness.commands.size(), commands_before + 1U);
    EXPECT_TRUE(std::isnan(harness.commands.back().position.front()));
    EXPECT_DOUBLE_EQ(harness.commands.back().velocity.front(), 0.0);
  }
}

} // namespace

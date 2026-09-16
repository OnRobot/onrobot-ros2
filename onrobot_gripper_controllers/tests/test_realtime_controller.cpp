#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <controller_interface/controller_interface_params.hpp>
#include <gtest/gtest.h>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <onrobot_gripper_controllers/onrobot_realtime_controller.hpp>
#include <onrobot_gripper_msgs/msg/realtime_command.hpp>
#include <onrobot_gripper_msgs/msg/realtime_state.hpp>
#include <rcl/time.h>
#include <rclcpp/rclcpp.hpp>

namespace onrobot_gripper_controllers {
class RealtimeControllerTestAccess {
public:
  static auto lockCommandBox(OnRobotRealtimeController &controller) {
    return std::unique_lock(controller.m_commandBox.get_mutex());
  }
  static auto &commandBoxMutex(OnRobotRealtimeController &controller) {
    return controller.m_commandBox.get_mutex();
  }
  static uint64_t nextSequence(const OnRobotRealtimeController &controller) {
    return controller.m_nextSequence.load(std::memory_order_acquire);
  }
};
} // namespace onrobot_gripper_controllers

namespace {

using Command = onrobot_gripper_msgs::msg::RealtimeCommand;
using State = onrobot_gripper_msgs::msg::RealtimeState;

class RealtimeControllerFixture {
public:
  void setUp(const std::string &coordinate_profile = "2fg",
             bool activate_controller = true) {
    coordinate_profile_ = coordinate_profile;
    controller_ = std::make_unique<
        onrobot_gripper_controllers::OnRobotRealtimeController>();
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "realtime_controller";
    params.node_namespace = "/";
    params.update_rate = 100;
    params.controller_manager_update_rate = 100;
    ASSERT_EQ(controller_->init(params), controller_interface::return_type::OK);
    const auto profile_result = controller_->get_node()->set_parameter(
        rclcpp::Parameter("coordinate_profile", coordinate_profile));
    ASSERT_TRUE(profile_result.successful) << profile_result.reason;
    ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);

    const auto velocity_interface = coordinate_profile_ == "rg"
                                        ? "realtime_mechanism_angular_velocity"
                                        : "realtime_task_velocity";
    const auto configured_commands =
        controller_->command_interface_configuration().names;
    ASSERT_EQ(configured_commands.at(4),
              std::string("grip_stroke/") + velocity_interface);
    const std::array<std::string, 7> command_names = {
        "position",
        "effort",
        "realtime_mode",
        "realtime_task_position",
        velocity_interface,
        "realtime_force",
        "realtime_command_sequence"};
    for (const auto &name : command_names) {
      command_handles_.push_back(
          std::make_shared<hardware_interface::CommandInterface>(
              "grip_stroke", name, "double", "0"));
      command_loans_.emplace_back(command_handles_.back(), nullptr);
    }
    const auto measured_position = coordinate_profile_ == "rg"
                                       ? "measured_angular_position"
                                       : "measured_position";
    const auto measured_velocity = coordinate_profile_ == "rg"
                                       ? "measured_angular_velocity"
                                       : "measured_velocity";
    const std::array<std::string, 19> state_names = {
        std::string("finger_stroke/") + measured_position,
        std::string("finger_stroke/") + measured_velocity,
        "finger_stroke/position_valid",
        "finger_stroke/velocity_valid",
        "grip_stroke/position",
        "grip_stroke/velocity",
        "grip_stroke/task_position_valid",
        "grip_stroke/effort",
        "grip_stroke/force_valid",
        "grip_stroke/active_mode",
        "grip_stroke/faulted",
        "grip_stroke/successful_cycles",
        "grip_stroke/failed_cycles",
        "grip_stroke/missed_deadlines",
        "grip_stroke/watchdog_stops",
        "grip_stroke/last_cycle_duration",
        "grip_stroke/requested_command_sequence",
        "grip_stroke/applied_command_sequence",
        "grip_stroke/reconnects"};
    for (const auto &full_name : state_names) {
      const auto slash = full_name.find('/');
      const auto joint = full_name.substr(0, slash);
      const auto name = full_name.substr(slash + 1);
      state_handles_.push_back(
          std::make_shared<hardware_interface::StateInterface>(joint, name,
                                                               "double", "0"));
      state_loans_.emplace_back(state_handles_.back(), nullptr);
    }
    controller_->assign_interfaces(std::move(command_loans_),
                                   std::move(state_loans_));
    if (activate_controller) {
      ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()),
                controller_interface::CallbackReturn::SUCCESS);
    }

    node_ = std::make_shared<rclcpp::Node>("realtime_controller_test_peer");
    publisher_ = node_->create_publisher<Command>(
        "/realtime_controller/command", rclcpp::QoS(1).reliable());
    state_subscription_ = node_->create_subscription<State>(
        "/realtime_controller/state", rclcpp::SensorDataQoS(),
        [this](const State::SharedPtr) {
          state_message_count_.fetch_add(1, std::memory_order_relaxed);
        });
    executor_.add_node(node_);
    executor_.add_node(controller_->get_node()->get_node_base_interface());
  }

  void tearDown() {
    if (controller_) {
      controller_->on_deactivate(rclcpp_lifecycle::State());
      controller_->release_interfaces();
    }
    executor_.remove_node(node_);
    executor_.remove_node(controller_->get_node()->get_node_base_interface());
    publisher_.reset();
    state_subscription_.reset();
    node_.reset();
    controller_.reset();
    command_handles_.clear();
    state_handles_.clear();
  }

  template <typename Predicate>
  void waitFor(Predicate &&predicate, const char *description) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!predicate()) {
      ASSERT_LT(std::chrono::steady_clock::now(), deadline) << description;
      executor_.spin_some(std::chrono::milliseconds(1));
    }
  }

  void waitForCommandSubscription() {
    waitFor(
        [this] {
          return node_->count_subscribers("/realtime_controller/command") > 0;
        },
        "realtime command subscription discovery timed out");
  }

  void waitForStatePublisher() {
    waitFor(
        [this] {
          return node_->count_publishers("/realtime_controller/state") > 0;
        },
        "realtime state publisher discovery timed out");
  }

  std::size_t stateMessageCount() const {
    return state_message_count_.load(std::memory_order_relaxed);
  }

  void spinUntilStateCount(std::size_t expected) {
    waitFor([this, expected] { return stateMessageCount() >= expected; },
            "realtime state message receipt timed out");
  }

  void deliverFrom(const rclcpp::Publisher<Command>::SharedPtr &publisher,
                   Command message) {
    waitForCommandSubscription();
    const auto prior_sequence =
        onrobot_gripper_controllers::RealtimeControllerTestAccess::nextSequence(
            *controller_);
    publisher->publish(message);
    waitFor(
        [this, prior_sequence] {
          return onrobot_gripper_controllers::RealtimeControllerTestAccess::
                     nextSequence(*controller_) > prior_sequence;
        },
        "realtime command callback receipt timed out");
  }

  void deliver(Command message) { deliverFrom(publisher_, std::move(message)); }

  void publishWithoutSpinning(Command message) {
    waitForCommandSubscription();
    publisher_->publish(std::move(message));
    ASSERT_TRUE(publisher_->wait_for_all_acked(std::chrono::seconds(2)));
  }

  void deliverDiscarded(Command message) {
    waitForCommandSubscription();
    publisher_->publish(std::move(message));
    ASSERT_TRUE(publisher_->wait_for_all_acked(std::chrono::seconds(2)));
    executor_.spin_some(std::chrono::milliseconds(1));
  }

  double command(std::size_t index) const {
    return command_handles_.at(index)->get_optional<double>().value();
  }

  hardware_interface::CommandInterface &commandHandle(std::size_t index) {
    return *command_handles_.at(index);
  }

  double expectedVelocity() const {
    return coordinate_profile_ == "rg" ? 0.25 : 0.01;
  }

  void setControllerRosTime(const rclcpp::Time &time) {
    auto *clock = controller_->get_node()->get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock, time.nanoseconds()), RCL_RET_OK);
  }

  std::unique_ptr<onrobot_gripper_controllers::OnRobotRealtimeController>
      controller_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_handles_;
  std::vector<hardware_interface::StateInterface::SharedPtr> state_handles_;
  std::vector<hardware_interface::LoanedCommandInterface> command_loans_;
  std::vector<hardware_interface::LoanedStateInterface> state_loans_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<Command>::SharedPtr publisher_;
  rclcpp::Subscription<State>::SharedPtr state_subscription_;
  std::atomic<std::size_t> state_message_count_{0};
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::string coordinate_profile_;
};

class RealtimeControllerTest : public ::testing::TestWithParam<std::string> {
protected:
  static void SetUpTestSuite() {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override { fixture_.setUp(GetParam()); }
  void TearDown() override { fixture_.tearDown(); }
  RealtimeControllerFixture fixture_;
};

class ConfiguredRealtimeControllerTest
    : public ::testing::TestWithParam<std::string> {
protected:
  static void SetUpTestSuite() {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override { fixture_.setUp(GetParam(), false); }
  void TearDown() override { fixture_.tearDown(); }
  RealtimeControllerFixture fixture_;
};

Command motion(const rclcpp::Time &stamp, double position = 0.02) {
  Command message;
  message.header.stamp = stamp;
  message.mode = Command::POSITION;
  message.task_position = position;
  message.task_velocity = 0.01;
  message.mechanism_angular_velocity = 0.25;
  message.force = 0.0;
  return message;
}

TEST_P(RealtimeControllerTest, PublishesStateAtEveryDefault100HzUpdate) {
  EXPECT_DOUBLE_EQ(fixture_.controller_->get_node()
                       ->get_parameter("state_publish_rate_hz")
                       .as_double(),
                   100.0);
  fixture_.waitForStatePublisher();

  const auto start = fixture_.node_->now();
  for (std::size_t cycle = 0; cycle < 20; ++cycle) {
    const auto time = start + rclcpp::Duration::from_seconds(0.01 * cycle);
    const auto expected_count = fixture_.stateMessageCount() + 1;
    ASSERT_EQ(fixture_.controller_->update(
                  time, rclcpp::Duration::from_seconds(0.01)),
              controller_interface::return_type::OK);
    fixture_.spinUntilStateCount(expected_count);
  }
  EXPECT_EQ(fixture_.stateMessageCount(), 20U);
}

TEST_P(RealtimeControllerTest, ActivationAlwaysStopsBeforeMotion) {
  const auto now = fixture_.node_->now();
  fixture_.deliverDiscarded(motion(now));
  ASSERT_EQ(
      fixture_.controller_->update(now, rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), -1.0);

  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), -1.0);

  fixture_.deliver(motion(fixture_.node_->now()));
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), static_cast<double>(Command::POSITION));
  EXPECT_DOUBLE_EQ(fixture_.command(3), 0.02);
  EXPECT_DOUBLE_EQ(fixture_.command(4), fixture_.expectedVelocity());
}

TEST_P(RealtimeControllerTest, StopFencePrecedesFollowingMotion) {
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  fixture_.deliver(motion(fixture_.node_->now(), 0.01));
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  Command stop;
  stop.header.stamp = fixture_.node_->now();
  stop.mode = Command::STOP;
  fixture_.deliver(stop);
  fixture_.deliver(motion(fixture_.node_->now(), 0.03));
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), -1.0);
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(fixture_.command(3), 0.03);
}

TEST_P(RealtimeControllerTest, ZeroStampAndMalformedMotionStop) {
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  fixture_.deliver(motion(rclcpp::Time(0)));
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), -1.0);
  const auto prior_sequence = fixture_.command(6);
  auto malformed = motion(fixture_.node_->now());
  malformed.task_velocity = std::numeric_limits<double>::quiet_NaN();
  malformed.mechanism_angular_velocity =
      std::numeric_limits<double>::quiet_NaN();
  fixture_.deliver(malformed);
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), -1.0);
  EXPECT_NE(fixture_.command(6), prior_sequence);
  EXPECT_DOUBLE_EQ(fixture_.command(3), 0.0);
}

TEST_P(RealtimeControllerTest,
       InvalidRosStampStopsWithoutThrowingOrReplayingMotion) {
  const auto stamp = fixture_.node_->now();
  ASSERT_EQ(
      fixture_.controller_->update(stamp, rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
  // The first update is the activation boundary and mandatory Stop.  Motion
  // must carry a strictly newer source stamp; reusing the boundary stamp would
  // correctly exercise stale-input rejection instead of this malformed case.
  const auto motion_stamp = stamp + rclcpp::Duration::from_nanoseconds(1);
  fixture_.deliver(motion(motion_stamp, 0.025));
  ASSERT_EQ(fixture_.controller_->update(motion_stamp,
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  ASSERT_EQ(fixture_.command(2), static_cast<double>(Command::POSITION));

  const auto fresh_stamp = fixture_.node_->now();
  std::array<Command, 2> malformed_commands = {motion(fresh_stamp),
                                               motion(fresh_stamp)};
  malformed_commands[0].header.stamp.sec = -1;
  malformed_commands[0].header.stamp.nanosec = 0;
  // This pair is wire-invalid but normalizes to fresh_stamp if an unchecked
  // rclcpp::Time constructor accepts it, so the regression cannot pass merely
  // because the old value is stale.
  --malformed_commands[1].header.stamp.sec;
  malformed_commands[1].header.stamp.nanosec += 1000000000U;

  for (const auto &malformed : malformed_commands) {
    const auto prior_sequence = fixture_.command(6);
    EXPECT_NO_THROW(fixture_.deliver(malformed));
    ASSERT_EQ(fixture_.controller_->update(
                  fixture_.node_->now(), rclcpp::Duration::from_seconds(0.01)),
              controller_interface::return_type::OK);
    EXPECT_EQ(fixture_.command(2), -1.0);
    EXPECT_GT(fixture_.command(6), prior_sequence);
  }
}

TEST_P(RealtimeControllerTest, SourceTimeoutStopsAppliedMotion) {
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  const auto stamp = fixture_.node_->now();
  fixture_.deliver(motion(stamp));
  ASSERT_EQ(
      fixture_.controller_->update(stamp, rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), static_cast<double>(Command::POSITION));

  const auto expired = stamp + rclcpp::Duration::from_seconds(0.101);
  ASSERT_EQ(fixture_.controller_->update(expired,
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), -1.0);
}

TEST_P(RealtimeControllerTest, MotionDeliveredWhileInactiveIsNotReplayed) {
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  ASSERT_EQ(fixture_.controller_->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  fixture_.deliverDiscarded(motion(fixture_.node_->now(), 0.01));
  ASSERT_EQ(fixture_.controller_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), -1.0);
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), -1.0);
}

TEST_P(ConfiguredRealtimeControllerTest,
       InactiveDeliveryBeforeFirstActivationIsDiscarded) {
  fixture_.deliverDiscarded(motion(fixture_.node_->now(), 0.01));
  ASSERT_EQ(fixture_.controller_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), -1.0);

  fixture_.deliver(motion(fixture_.node_->now(), 0.03));
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), static_cast<double>(Command::POSITION));
  EXPECT_DOUBLE_EQ(fixture_.command(3), 0.03);
}

TEST_P(RealtimeControllerTest, FutureAndOldSourceStampsStop) {
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  const auto now = fixture_.node_->now();
  fixture_.deliver(motion(now + rclcpp::Duration::from_seconds(1.0)));
  ASSERT_EQ(
      fixture_.controller_->update(now, rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), -1.0);
  fixture_.deliver(
      motion(fixture_.node_->now() - rclcpp::Duration::from_seconds(0.2)));
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), -1.0);
}

TEST_P(ConfiguredRealtimeControllerTest,
       ControllerClockLagDoesNotRejectCommandFreshAtUpdateTime) {
  const rclcpp::Time controller_callback_time(1000000000LL, RCL_ROS_TIME);
  const rclcpp::Time publisher_and_update_time(1010000000LL, RCL_ROS_TIME);
  fixture_.setControllerRosTime(controller_callback_time);
  ASSERT_EQ(fixture_.controller_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  ASSERT_EQ(fixture_.controller_->update(controller_callback_time,
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  ASSERT_EQ(fixture_.command(2), -1.0);

  // /clock subscriptions are independent.  The publisher can observe the
  // next simulation tick before the controller node while the controller
  // manager already supplies that tick to update().  The update time is the
  // authority for deciding whether the source stamp is in the future.
  fixture_.deliver(motion(publisher_and_update_time, 0.03));
  ASSERT_EQ(fixture_.controller_->update(publisher_and_update_time,
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), static_cast<double>(Command::POSITION));
  EXPECT_DOUBLE_EQ(fixture_.command(3), 0.03);
}

TEST_P(ConfiguredRealtimeControllerTest,
       QueuedPreactivationCommandCannotCrossLaggingNodeClockBoundary) {
  const rclcpp::Time lagging_node_time(1000000000LL, RCL_ROS_TIME);
  const rclcpp::Time activation_update_time(1010000000LL, RCL_ROS_TIME);
  fixture_.setControllerRosTime(lagging_node_time);
  fixture_.publishWithoutSpinning(motion(activation_update_time, 0.03));

  ASSERT_EQ(fixture_.controller_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(fixture_.controller_->update(activation_update_time,
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  ASSERT_EQ(fixture_.command(2), -1.0);

  const auto prior_sequence =
      onrobot_gripper_controllers::RealtimeControllerTestAccess::nextSequence(
          *fixture_.controller_);
  fixture_.waitFor(
      [&] {
        return onrobot_gripper_controllers::RealtimeControllerTestAccess::
                   nextSequence(*fixture_.controller_) > prior_sequence;
      },
      "queued preactivation callback was not delivered");
  ASSERT_EQ(fixture_.controller_->update(activation_update_time,
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), -1.0);
  EXPECT_DOUBLE_EQ(fixture_.command(3), 0.0);
}

TEST_P(ConfiguredRealtimeControllerTest,
       SameTickPostActivationCommandStopsThenNextTickCommandIsAdmitted) {
  const rclcpp::Time activation_time(1000000000LL, RCL_ROS_TIME);
  const rclcpp::Time next_time(1010000000LL, RCL_ROS_TIME);
  fixture_.setControllerRosTime(activation_time);
  ASSERT_EQ(fixture_.controller_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(fixture_.controller_->update(activation_time,
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  ASSERT_EQ(fixture_.command(2), -1.0);

  // The activation Stop owns its exact controller-manager time.  A command
  // stamped at that same source boundary must not become motion; the first
  // later simulation tick is the earliest eligible command.
  const auto before_same_tick =
      onrobot_gripper_controllers::RealtimeControllerTestAccess::nextSequence(
          *fixture_.controller_);
  fixture_.deliver(motion(activation_time, 0.01));
  ASSERT_EQ(fixture_.controller_->update(activation_time,
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), -1.0);
  EXPECT_GT(
      onrobot_gripper_controllers::RealtimeControllerTestAccess::nextSequence(
          *fixture_.controller_),
      before_same_tick);

  fixture_.deliver(motion(next_time, 0.03));
  ASSERT_EQ(fixture_.controller_->update(next_time,
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), static_cast<double>(Command::POSITION));
  EXPECT_DOUBLE_EQ(fixture_.command(3), 0.03);
}

TEST_P(RealtimeControllerTest, SourceClockRollbackStopsAndInvalidatesCache) {
  const auto activation_stamp = fixture_.node_->now();
  ASSERT_EQ(fixture_.controller_->update(activation_stamp,
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  // Establish motion after the activation Stop so this case proves rollback
  // invalidates an applied command rather than replaying the boundary sample.
  const auto motion_stamp =
      activation_stamp + rclcpp::Duration::from_nanoseconds(1);
  fixture_.deliver(motion(motion_stamp));
  ASSERT_EQ(fixture_.controller_->update(motion_stamp,
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), static_cast<double>(Command::POSITION));
  const auto rolled_back = motion_stamp - rclcpp::Duration::from_nanoseconds(1);
  ASSERT_EQ(fixture_.controller_->update(rolled_back,
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), -1.0);
}

TEST_P(RealtimeControllerTest, SteadyWatchdogStopsWhileRosTimeIsPaused) {
  const auto stamp = fixture_.node_->now();
  ASSERT_EQ(
      fixture_.controller_->update(stamp, rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
  fixture_.deliver(motion(stamp));
  ASSERT_EQ(
      fixture_.controller_->update(stamp, rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  ASSERT_EQ(
      fixture_.controller_->update(stamp, rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), -1.0);
}

TEST_P(RealtimeControllerTest, TwoPublishersUseAcceptedCallbackOrder) {
  const auto stamp = fixture_.node_->now();
  ASSERT_EQ(
      fixture_.controller_->update(stamp, rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
  auto second_publisher = fixture_.node_->create_publisher<Command>(
      "/realtime_controller/command", rclcpp::QoS(1).reliable());
  fixture_.deliverFrom(second_publisher, motion(fixture_.node_->now(), 0.01));
  fixture_.deliver(motion(fixture_.node_->now(), 0.03));
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(fixture_.command(3), 0.03);
}

TEST_P(RealtimeControllerTest, HandoffContentionKeepsAcceptedCommand) {
  const auto now = fixture_.node_->now();
  ASSERT_EQ(
      fixture_.controller_->update(now, rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
  fixture_.deliver(motion(fixture_.node_->now(), 0.02));
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  std::promise<void> acquired;
  std::atomic<bool> release{false};
  std::thread holder([&] {
    auto box_lock = std::unique_lock(
        onrobot_gripper_controllers::RealtimeControllerTestAccess::
            commandBoxMutex(*fixture_.controller_));
    acquired.set_value();
    while (!release.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
  });
  acquired.get_future().wait();
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), static_cast<double>(Command::POSITION));
  release.store(true, std::memory_order_release);
  holder.join();
}

TEST_P(RealtimeControllerTest, StopSelectorFailureDoesNotAdvanceSequence) {
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  const double prior_sequence = fixture_.command(6);
  auto mode_lock = std::unique_lock(fixture_.commandHandle(2).get_mutex());
  ASSERT_EQ(fixture_.controller_->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::ERROR);
  EXPECT_DOUBLE_EQ(fixture_.command(6), prior_sequence);
  mode_lock.unlock();
}

TEST_P(RealtimeControllerTest, PayloadFailureStopsWithoutMotionSequenceCommit) {
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  fixture_.deliver(motion(fixture_.node_->now(), 0.04));
  const double prior_sequence = fixture_.command(6);
  auto position_lock = std::unique_lock(fixture_.commandHandle(3).get_mutex());
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::ERROR);
  EXPECT_GT(fixture_.command(6), prior_sequence);
  position_lock.unlock();
  ASSERT_EQ(fixture_.controller_->update(fixture_.node_->now(),
                                         rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_EQ(fixture_.command(2), -1.0);
}

INSTANTIATE_TEST_SUITE_P(CoordinateProfiles, RealtimeControllerTest,
                         ::testing::Values(std::string("2fg"),
                                           std::string("rg")),
                         [](const ::testing::TestParamInfo<std::string> &info) {
                           return info.param;
                         });

INSTANTIATE_TEST_SUITE_P(CoordinateProfiles, ConfiguredRealtimeControllerTest,
                         ::testing::Values(std::string("2fg"),
                                           std::string("rg")),
                         [](const ::testing::TestParamInfo<std::string> &info) {
                           return info.param;
                         });

} // namespace

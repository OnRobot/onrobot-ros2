#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <controller_interface/controller_interface_params.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <gtest/gtest.h>
#include <onrobot_gripper_controllers/gripper_state_broadcaster.hpp>
#include <onrobot_gripper_msgs/diagnostic_interfaces.hpp>
#include <onrobot_gripper_msgs/firmware_identity.hpp>
#include <onrobot_gripper_msgs/msg/gripper_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace onrobot_gripper_controllers {

class GripperStateBroadcasterTestAccess {
public:
  static bool setHealthySnapshot(GripperStateBroadcaster &controller,
                                 std::chrono::seconds age) {
    GripperStateBroadcaster::DiagnosticSnapshot snapshot;
    snapshot.connection_state = 3;
    snapshot.task_valid = true;
    snapshot.mechanism_valid = true;
    snapshot.sample_age_s = 0.0;
    snapshot.captured_at_steady_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            (std::chrono::steady_clock::now() - age).time_since_epoch())
            .count();
    snapshot.device_diagnostics.fill(std::numeric_limits<double>::quiet_NaN());
    snapshot.device_diagnostics[onrobot_gripper_msgs::diagnosticIndex(
        onrobot_gripper_msgs::DiagnosticInterface::StatusValid)] = 1.0;
    snapshot.device_diagnostics[onrobot_gripper_msgs::diagnosticIndex(
        onrobot_gripper_msgs::DiagnosticInterface::Age)] = 0.0;
    return controller.m_diagnosticBox.try_set(snapshot);
  }

  static void publishDiagnostics(GripperStateBroadcaster &controller) {
    controller.publishDiagnostics();
  }

  static void setConfiguredFirmware(GripperStateBroadcaster &controller,
                                    const std::string &firmware) {
    controller.m_firmware = firmware;
  }
};

} // namespace onrobot_gripper_controllers

namespace {

using Broadcaster = onrobot_gripper_controllers::GripperStateBroadcaster;
using Access = onrobot_gripper_controllers::GripperStateBroadcasterTestAccess;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using GripperState = onrobot_gripper_msgs::msg::GripperState;
using Field = onrobot_gripper_msgs::DiagnosticInterface;
using IdentityValues = std::array<double, onrobot_gripper_msgs::kDiagnosticInterfaceCount>;

std::string firmwareText(const IdentityValues &values) {
  const auto text = onrobot_gripper_msgs::firmwareIdentityText(values);
  return {text.data.data(), text.size};
}

IdentityValues identityValues() {
  IdentityValues result;
  result.fill(std::numeric_limits<double>::quiet_NaN());
  result[onrobot_gripper_msgs::diagnosticIndex(Field::IdentityValid)] = 1;
  result[onrobot_gripper_msgs::diagnosticIndex(Field::FirmwareMajor)] = 1;
  result[onrobot_gripper_msgs::diagnosticIndex(Field::FirmwareMinor)] = 0;
  result[onrobot_gripper_msgs::diagnosticIndex(Field::FirmwareBuild)] = 33;
  result[onrobot_gripper_msgs::diagnosticIndex(Field::FirmwareSourceValid)] = 0;
  return result;
}

TEST(FirmwareIdentity, BoundedSemverAndExactSourceHash) {
  auto values = identityValues();
  EXPECT_EQ(firmwareText(values), "1.0.33");
  values[onrobot_gripper_msgs::diagnosticIndex(Field::FirmwareMajor)] = 255;
  values[onrobot_gripper_msgs::diagnosticIndex(Field::FirmwareMinor)] = 255;
  values[onrobot_gripper_msgs::diagnosticIndex(Field::FirmwareBuild)] = 65535;
  values[onrobot_gripper_msgs::diagnosticIndex(Field::FirmwareSourceValid)] = 1;
  const std::array<uint32_t, 5> words{0x00000001, 0x12345678, 0xffffffff, 0, 0xabcdef09};
  for (std::size_t index = 0; index < words.size(); ++index) {
    values[onrobot_gripper_msgs::diagnosticIndex(Field::FirmwareGitWord0) + index] = words[index];
  }
  EXPECT_EQ(firmwareText(values),
            "255.255.65535+g0000000112345678ffffffff00000000abcdef09");
  EXPECT_LT(firmwareText(values).size(), 64U);
}

TEST(FirmwareIdentity, RejectsUnavailableAndMalformedNumericIdentity) {
  for (const auto field : {Field::IdentityValid, Field::FirmwareMajor,
                           Field::FirmwareMinor, Field::FirmwareBuild,
                           Field::FirmwareSourceValid}) {
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity(), -1.0, 0.5, 65536.0}) {
      auto values = identityValues();
      values[onrobot_gripper_msgs::diagnosticIndex(field)] = invalid;
      EXPECT_TRUE(firmwareText(values).empty());
    }
  }
  auto values = identityValues();
  values[onrobot_gripper_msgs::diagnosticIndex(Field::IdentityValid)] = 0;
  EXPECT_TRUE(firmwareText(values).empty());
  values = identityValues();
  values[onrobot_gripper_msgs::diagnosticIndex(Field::FirmwareSourceValid)] = 1;
  for (std::size_t index = 0; index < 5; ++index) {
    values[onrobot_gripper_msgs::diagnosticIndex(Field::FirmwareGitWord0) + index] = 0;
  }
  for (std::size_t index = 0; index < 5; ++index) {
    auto corrupt = values;
    corrupt[onrobot_gripper_msgs::diagnosticIndex(Field::FirmwareGitWord0) + index] = 4294967296.0;
    EXPECT_TRUE(firmwareText(corrupt).empty());
  }
}

class GripperStateBroadcasterTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    controller_ = std::make_unique<Broadcaster>();
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "gripper_state_broadcaster";
    params.node_namespace = "/";
    params.update_rate = 100;
    params.controller_manager_update_rate = 100;
    ASSERT_EQ(controller_->init(params), controller_interface::return_type::OK);
    ASSERT_TRUE(controller_->get_node()
                    ->set_parameter(rclcpp::Parameter("model", "2fg7"))
                    .successful);
    ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);

    peer_ = std::make_shared<rclcpp::Node>("gripper_diagnostics_test_peer");
    subscription_ = peer_->create_subscription<DiagnosticArray>(
        "/diagnostics", 10, [this](DiagnosticArray::SharedPtr message) {
          last_message_ = std::move(message);
          message_count_.fetch_add(1, std::memory_order_relaxed);
        });
    state_subscription_ = peer_->create_subscription<GripperState>(
        "/gripper_state_broadcaster/state", rclcpp::SensorDataQoS(),
        [this](const GripperState::SharedPtr message) {
          last_state_ = message;
          state_message_count_.fetch_add(1, std::memory_order_relaxed);
        });
    executor_.add_node(peer_);
    executor_.add_node(controller_->get_node()->get_node_base_interface());
    waitFor([this] { return peer_->count_publishers("/diagnostics") > 0; });
  }

  void TearDown() override {
    controller_->on_deactivate(rclcpp_lifecycle::State());
    executor_.remove_node(peer_);
    executor_.remove_node(controller_->get_node()->get_node_base_interface());
    subscription_.reset();
    state_subscription_.reset();
    peer_.reset();
    controller_.reset();
  }

  template <typename Predicate> void waitFor(Predicate predicate) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!predicate()) {
      ASSERT_LT(std::chrono::steady_clock::now(), deadline);
      executor_.spin_some(std::chrono::milliseconds(1));
    }
  }

  void publishAndWait() {
    const auto expected = message_count_.load(std::memory_order_relaxed) + 1;
    Access::publishDiagnostics(*controller_);
    waitFor([this, expected] {
      return message_count_.load(std::memory_order_relaxed) >= expected;
    });
  }

  std::string value(const std::string &key) const {
    for (const auto &entry : last_message_->status.at(0).values) {
      if (entry.key == key) {
        return entry.value;
      }
    }
    return {};
  }

  std::unique_ptr<Broadcaster> controller_;
  std::shared_ptr<rclcpp::Node> peer_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr subscription_;
  rclcpp::Subscription<GripperState>::SharedPtr state_subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::atomic<std::size_t> message_count_{0};
  std::atomic<std::size_t> state_message_count_{0};
  DiagnosticArray::SharedPtr last_message_;
  GripperState::SharedPtr last_state_;
};

TEST_F(GripperStateBroadcasterTest, PublishesTypedStateAtDefault100Hz) {
  EXPECT_DOUBLE_EQ(
      controller_->get_node()->get_parameter("publish_rate").as_double(),
      100.0);
  waitFor([this] {
    return peer_->count_publishers("/gripper_state_broadcaster/state") > 0;
  });

  const auto start = peer_->now();
  for (std::size_t cycle = 0; cycle < 20; ++cycle) {
    const auto expected =
        state_message_count_.load(std::memory_order_relaxed) + 1;
    const auto time = start + rclcpp::Duration::from_seconds(0.01 * cycle);
    ASSERT_EQ(controller_->update(time, rclcpp::Duration::from_seconds(0.01)),
              controller_interface::return_type::OK);
    waitFor([this, expected] {
      return state_message_count_.load(std::memory_order_relaxed) >= expected;
    });
  }
  EXPECT_EQ(state_message_count_.load(std::memory_order_relaxed), 20U);
}

TEST_F(GripperStateBroadcasterTest, CachedSnapshotAgesOutWhenUpdatesStop) {
  ASSERT_TRUE(
      Access::setHealthySnapshot(*controller_, std::chrono::seconds(0)));
  publishAndWait();
  ASSERT_TRUE(last_message_);
  ASSERT_EQ(last_message_->status.size(), 1U);
  EXPECT_EQ(last_message_->status[0].level, DiagnosticStatus::OK);

  ASSERT_TRUE(
      Access::setHealthySnapshot(*controller_, std::chrono::seconds(4)));
  publishAndWait();
  ASSERT_TRUE(last_message_);
  ASSERT_EQ(last_message_->status.size(), 1U);
  EXPECT_EQ(last_message_->status[0].level, DiagnosticStatus::WARN);
  EXPECT_EQ(last_message_->status[0].message, "device diagnostics are stale");
  EXPECT_GE(std::stod(value("sample_age_s")), 3.9);
  EXPECT_GE(std::stod(value("diagnostic_age")), 3.9);
}

TEST_F(GripperStateBroadcasterTest, DeviceIdentityReplacesConfiguredLabelAndClearsWhenInvalid) {
  Access::setConfiguredFirmware(*controller_, "configured-not-observed");
  auto values = identityValues();
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> handles;
  std::vector<hardware_interface::LoanedStateInterface> loans;
  for (std::size_t index = 0; index < values.size(); ++index) {
    handles.push_back(std::make_shared<hardware_interface::StateInterface>(
        "grip_stroke", onrobot_gripper_msgs::kDiagnosticInterfaceNames[index], "double", "0"));
    ASSERT_TRUE(handles.back()->set_value(values[index]));
    loans.emplace_back(handles.back(), nullptr);
  }
  controller_->assign_interfaces({}, std::move(loans));
  const auto publishState = [&] {
    const auto expected = state_message_count_.load() + 1;
    ASSERT_EQ(controller_->update(peer_->now(), rclcpp::Duration::from_seconds(0.01)),
              controller_interface::return_type::OK);
    waitFor([&] { return state_message_count_.load() >= expected; });
    publishAndWait();
  };
  publishState();
  ASSERT_TRUE(last_state_);
  EXPECT_EQ(last_state_->firmware, "1.0.33");
  EXPECT_EQ(value("firmware"), "1.0.33");
  EXPECT_EQ(value("firmware_source"), "device-connection");
  EXPECT_EQ(value("configured_firmware"), "configured-not-observed");
  ASSERT_TRUE(handles[onrobot_gripper_msgs::diagnosticIndex(Field::FirmwareBuild)]->set_value(34.0));
  publishState();
  EXPECT_EQ(last_state_->firmware, "1.0.34");
  EXPECT_EQ(value("firmware"), "1.0.34");
  ASSERT_TRUE(handles[onrobot_gripper_msgs::diagnosticIndex(Field::IdentityValid)]->set_value(0.0));
  publishState();
  EXPECT_TRUE(last_state_->firmware.empty());
  EXPECT_EQ(value("firmware"), "unknown");
  EXPECT_EQ(value("firmware_source"), "unavailable");
  controller_->release_interfaces();
}

} // namespace

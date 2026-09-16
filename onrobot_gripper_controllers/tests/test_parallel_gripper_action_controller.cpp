#include <chrono>
#include <cmath>
#include <cstdint>
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
#include <control_msgs/action/parallel_gripper_command.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <onrobot_gripper_controllers/parallel_gripper_action_controller.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

using Controller =
    onrobot_gripper_controllers::ParallelGripperActionController;
using CommandHandle = hardware_interface::CommandInterface::SharedPtr;
using StateHandle = hardware_interface::StateInterface::SharedPtr;

struct ControllerSlot {
  std::unique_ptr<Controller> controller;
  std::vector<CommandHandle> command_handles;
  std::vector<StateHandle> state_handles;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      rejecting_parameter_callback;
};

CommandHandle findCommand(const std::vector<CommandHandle> &handles,
                          const std::string &name) {
  for (const auto &handle : handles) {
    if (handle->get_name() == name) {
      return handle;
    }
  }
  return {};
}

StateHandle findState(const std::vector<StateHandle> &handles,
                      const std::string &name) {
  for (const auto &handle : handles) {
    if (handle->get_name() == name) {
      return handle;
    }
  }
  return {};
}

ControllerSlot
makeController(std::vector<CommandHandle> command_handles = {},
               std::vector<StateHandle> state_handles = {},
               bool allow_stalling = false,
               bool conventional_speed_control = false,
               int conventional_speed_percent = 50,
               bool install_rejecting_parameter_callback = false) {
  ControllerSlot slot;
  slot.command_handles = std::move(command_handles);
  slot.state_handles = std::move(state_handles);
  slot.controller = std::make_unique<Controller>();

  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "gripper_controller";
  params.node_namespace = "/";
  params.node_options.parameter_overrides(
      {rclcpp::Parameter("joint", "grip_stroke"),
       rclcpp::Parameter("state_interfaces",
                         std::vector<std::string>{"position", "velocity"}),
       rclcpp::Parameter("max_effort_interface", "grip_stroke/effort"),
       rclcpp::Parameter("max_effort", 10.0),
       rclcpp::Parameter("max_velocity_interface", ""),
       rclcpp::Parameter("max_velocity", 0.1),
       rclcpp::Parameter("allow_stalling", allow_stalling),
       rclcpp::Parameter("stall_timeout", 0.02),
       rclcpp::Parameter("conventional_speed_control",
                         conventional_speed_control),
       rclcpp::Parameter("conventional_speed_percent",
                         conventional_speed_percent)});
  EXPECT_EQ(slot.controller->init(params),
            controller_interface::return_type::OK);
  if (install_rejecting_parameter_callback) {
    auto node = slot.controller->get_node();
    node->declare_parameter<bool>("test_reject_atomic_transaction", false);
    slot.rejecting_parameter_callback = node->add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter> &parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto &parameter : parameters) {
            if (parameter.get_name() == "test_reject_atomic_transaction" &&
                parameter.as_bool()) {
              result.successful = false;
              result.reason = "test peer rejected atomic transaction";
              break;
            }
          }
          return result;
        });
  }
  const auto configure_result =
      slot.controller->on_configure(rclcpp_lifecycle::State());
  EXPECT_EQ(configure_result, controller_interface::CallbackReturn::SUCCESS);
  if (configure_result != controller_interface::CallbackReturn::SUCCESS) {
    return slot;
  }

  const auto command_configuration =
      slot.controller->command_interface_configuration();
  for (const auto &name : command_configuration.names) {
    if (!findCommand(slot.command_handles, name)) {
      const auto slash = name.find('/');
      const auto initial_value =
          name == "grip_stroke/stop_command_sequence" ||
                  name == "grip_stroke/conventional_command_sequence"
              ? ""
              : "0";
      slot.command_handles.push_back(
          std::make_shared<hardware_interface::CommandInterface>(
              name.substr(0, slash), name.substr(slash + 1), "double",
              initial_value));
    }
  }
  const auto state_configuration = slot.controller->state_interface_configuration();
  for (const auto &name : state_configuration.names) {
    if (!findState(slot.state_handles, name)) {
      const auto slash = name.find('/');
      const auto initial_value =
          name == "grip_stroke/position" ? "0.04" : "0";
      slot.state_handles.push_back(
          std::make_shared<hardware_interface::StateInterface>(
              name.substr(0, slash), name.substr(slash + 1), "double",
              initial_value));
    }
  }

  std::vector<hardware_interface::LoanedCommandInterface> command_loans;
  for (const auto &handle : slot.command_handles) {
    command_loans.emplace_back(handle, nullptr);
  }
  std::vector<hardware_interface::LoanedStateInterface> state_loans;
  for (const auto &handle : slot.state_handles) {
    state_loans.emplace_back(handle, nullptr);
  }
  slot.controller->assign_interfaces(std::move(command_loans),
                                     std::move(state_loans));
  return slot;
}

double commandValue(const ControllerSlot &slot, const std::string &name) {
  const auto handle = findCommand(slot.command_handles, name);
  EXPECT_TRUE(handle != nullptr);
  return handle->get_optional<double>().value_or(
      std::numeric_limits<double>::quiet_NaN());
}

class ParallelGripperActionControllerTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(ParallelGripperActionControllerTest,
       DeactivationPublishesStopWithoutAnotherUpdate) {
  auto slot = makeController();
  ASSERT_EQ(slot.controller->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(std::isnan(commandValue(slot, "grip_stroke/stop_command_sequence")));

  ASSERT_EQ(slot.controller->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(commandValue(slot, "grip_stroke/position"), 0.04);
  const auto stop = commandValue(slot, "grip_stroke/stop_command_sequence");
  EXPECT_TRUE(std::isfinite(stop));
  EXPECT_GT(stop, 0.0);
  EXPECT_EQ(std::floor(stop), stop);
  slot.controller->release_interfaces();
}

TEST_F(ParallelGripperActionControllerTest,
       ReactivationPreservesPendingEventAndAllowsMarkerReuse) {
  auto first = makeController();
  ASSERT_EQ(first.controller->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(first.controller->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  const auto first_stop =
      commandValue(first, "grip_stroke/stop_command_sequence");
  first.controller->release_interfaces();

  auto second = makeController(first.command_handles, first.state_handles);
  ASSERT_EQ(second.controller->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(commandValue(second, "grip_stroke/stop_command_sequence"),
                   first_stop);

  ASSERT_EQ(second.controller->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  const auto second_stop =
      commandValue(second, "grip_stroke/stop_command_sequence");
  EXPECT_TRUE(std::isfinite(second_stop));
  // The marker is a one-shot event value, not a hardware-persistent identity.
  // A newly loaded controller may restart at one; the adapter must still
  // consume the event because the previous marker was already cleared.
  EXPECT_DOUBLE_EQ(second_stop, first_stop);
  second.controller->release_interfaces();
}

TEST_F(ParallelGripperActionControllerTest,
       DeactivationRejectsNonFiniteMeasuredPosition) {
  for (const auto measured : {std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::infinity()}) {
    auto slot = makeController();
    ASSERT_EQ(slot.controller->on_activate(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
    const auto state = findState(slot.state_handles, "grip_stroke/position");
    ASSERT_NE(state, nullptr);
    ASSERT_TRUE(state->set_value(measured));

    EXPECT_EQ(slot.controller->on_deactivate(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::ERROR);
    EXPECT_TRUE(std::isfinite(
        commandValue(slot, "grip_stroke/stop_command_sequence")));

    // Once the state becomes valid, a retry can complete the handoff and
    // issue a fresh one-shot Stop marker.
    ASSERT_TRUE(state->set_value(0.04));
    EXPECT_EQ(slot.controller->on_deactivate(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
    slot.controller->release_interfaces();
  }
}

TEST_F(ParallelGripperActionControllerTest,
       DeactivationReportsStopInterfaceWriteFailure) {
  auto slot = makeController();
  ASSERT_EQ(slot.controller->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  const auto stop_handle =
      findCommand(slot.command_handles, "grip_stroke/stop_command_sequence");
  ASSERT_NE(stop_handle, nullptr);
  auto lock = std::unique_lock(stop_handle->get_mutex());
  EXPECT_EQ(slot.controller->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::ERROR);
  lock.unlock();
  EXPECT_EQ(slot.controller->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  slot.controller->release_interfaces();
}

TEST_F(ParallelGripperActionControllerTest,
       RejectsUnsupportedPerGoalVelocityInsteadOfIgnoringIt) {
  using Action = control_msgs::action::ParallelGripperCommand;
  using namespace std::chrono_literals;

  auto slot = makeController();
  ASSERT_EQ(slot.controller->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(slot.controller->get_node()->get_node_base_interface());
  const auto client_node =
      std::make_shared<rclcpp::Node>("per_goal_velocity_action_client");
  executor.add_node(client_node);
  const auto client = rclcpp_action::create_client<Action>(
      client_node, "/gripper_controller/gripper_cmd");

  const auto ready_deadline = std::chrono::steady_clock::now() + 2s;
  while (!client->action_server_is_ready() &&
         std::chrono::steady_clock::now() < ready_deadline) {
    executor.spin_some();
  }
  ASSERT_TRUE(client->action_server_is_ready());

  for (const double requested_velocity : {0.0, 0.01}) {
    Action::Goal goal;
    goal.command.name = {"grip_stroke"};
    goal.command.position = {0.04};
    goal.command.velocity = {requested_velocity};
    const auto response = client->async_send_goal(goal);
    const auto response_deadline = std::chrono::steady_clock::now() + 2s;
    while (response.wait_for(0s) != std::future_status::ready &&
           std::chrono::steady_clock::now() < response_deadline) {
      executor.spin_some();
    }
    ASSERT_EQ(response.wait_for(0s), std::future_status::ready);
    EXPECT_EQ(response.get(), nullptr);
  }
  EXPECT_TRUE(std::isnan(
      commandValue(slot, "grip_stroke/conventional_command_sequence")));

  executor.remove_node(client_node);
  executor.remove_node(slot.controller->get_node()->get_node_base_interface());
  ASSERT_EQ(slot.controller->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  slot.controller->release_interfaces();
}

TEST_F(ParallelGripperActionControllerTest,
       InvalidFeedbackAbortsRatherThanReportingSuccessfulStallOrGoal) {
  using Action = control_msgs::action::ParallelGripperCommand;
  using namespace std::chrono_literals;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const auto &[position, velocity] :
       std::vector<std::pair<double, double>>{
           {nan, 0.0}, {inf, 0.0}, {-inf, 0.0}, {0.06, nan}, {0.06, inf}}) {
    SCOPED_TRACE("position=" + std::to_string(position) +
                 " velocity=" + std::to_string(velocity));
    auto slot = makeController({}, {}, true);
    ASSERT_EQ(slot.controller->on_activate(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(slot.controller->get_node()->get_node_base_interface());
    const auto node = std::make_shared<rclcpp::Node>("invalid_feedback_client");
    executor.add_node(node);
    const auto client = rclcpp_action::create_client<Action>(
        node, "/gripper_controller/gripper_cmd");
    const auto waitFor = [&](const auto &predicate) {
      const auto deadline = std::chrono::steady_clock::now() + 2s;
      while (!predicate() && std::chrono::steady_clock::now() < deadline) {
        executor.spin_once(2ms);
      }
      return predicate();
    };
    ASSERT_TRUE(waitFor([&] { return client->action_server_is_ready(); }));
    Action::Goal goal;
    goal.command.name = {"grip_stroke"};
    goal.command.position = {0.06};
    auto response = client->async_send_goal(goal);
    ASSERT_TRUE(waitFor([&] { return response.wait_for(0s) == std::future_status::ready; }));
    const auto handle = response.get();
    ASSERT_NE(handle, nullptr);
    auto result = client->async_get_result(handle);
    auto task_position = findState(slot.state_handles, "grip_stroke/position");
    auto task_velocity = findState(slot.state_handles, "grip_stroke/velocity");
    ASSERT_TRUE(task_position->set_value(position));
    ASSERT_TRUE(task_velocity->set_value(velocity));
    ASSERT_TRUE(waitFor([&] {
      slot.controller->update(node->now(), rclcpp::Duration::from_seconds(0.01));
      return result.wait_for(0s) == std::future_status::ready;
    }));
    const auto outcome = result.get();
    EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::ABORTED);
    EXPECT_FALSE(outcome.result->reached_goal);
    EXPECT_FALSE(outcome.result->stalled) << "Invalid feedback is not measured contact";
    EXPECT_TRUE(std::isfinite(commandValue(slot, "grip_stroke/stop_command_sequence")));
    EXPECT_TRUE(std::isnan(commandValue(slot, "grip_stroke/conventional_command_sequence")));

    // Still-invalid feedback rejects new goals; becoming valid must not cause
    // the old target to be written again. Only fresh explicit intent may move.
    response = client->async_send_goal(goal);
    ASSERT_TRUE(waitFor([&] { return response.wait_for(0s) == std::future_status::ready; }));
    EXPECT_EQ(response.get(), nullptr);
    ASSERT_TRUE(task_position->set_value(0.04));
    ASSERT_TRUE(task_velocity->set_value(0.0));
    ASSERT_TRUE(findCommand(slot.command_handles, "grip_stroke/position")->set_value(0.04));
    EXPECT_EQ(slot.controller->update(node->now(), rclcpp::Duration::from_seconds(0.01)),
              controller_interface::return_type::OK);
    EXPECT_DOUBLE_EQ(commandValue(slot, "grip_stroke/position"), 0.04);

    response = client->async_send_goal(goal);
    ASSERT_TRUE(waitFor([&] { return response.wait_for(0s) == std::future_status::ready; }));
    const auto fresh_handle = response.get();
    ASSERT_NE(fresh_handle, nullptr);
    auto fresh_result = client->async_get_result(fresh_handle);
    // A new goal stays buffered while Stop has not been consumed by hardware.
    slot.controller->update(node->now(), rclcpp::Duration::from_seconds(0.01));
    EXPECT_DOUBLE_EQ(commandValue(slot, "grip_stroke/position"), 0.04);
    ASSERT_TRUE(findCommand(slot.command_handles, "grip_stroke/stop_command_sequence")
                    ->set_value(std::numeric_limits<double>::quiet_NaN()));
    ASSERT_TRUE(task_position->set_value(0.06));
    // The backend must acknowledge admission before an at-target goal succeeds.
    slot.controller->update(node->now(), rclcpp::Duration::from_seconds(0.01));
    ASSERT_TRUE(findCommand(slot.command_handles,
                            "grip_stroke/conventional_command_sequence")
                    ->set_value(std::numeric_limits<double>::quiet_NaN()));
    ASSERT_TRUE(waitFor([&] {
      slot.controller->update(node->now(), rclcpp::Duration::from_seconds(0.01));
      return fresh_result.wait_for(0s) == std::future_status::ready;
    }));
    EXPECT_EQ(fresh_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_TRUE(fresh_result.get().result->reached_goal);

    ASSERT_EQ(slot.controller->on_deactivate(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
    executor.remove_node(node);
    executor.remove_node(slot.controller->get_node()->get_node_base_interface());
    slot.controller->release_interfaces();
  }
}

TEST_F(ParallelGripperActionControllerTest,
       ConfiguredEffortIsWrittenAndAdmissionPrecedesReachOrStall) {
  using Action = control_msgs::action::ParallelGripperCommand;
  using namespace std::chrono_literals;
  auto slot = makeController({}, {}, true);
  ASSERT_EQ(slot.controller->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(slot.controller->get_node()->get_node_base_interface());
  auto node = std::make_shared<rclcpp::Node>("admission_client");
  executor.add_node(node);
  auto client = rclcpp_action::create_client<Action>(
      node, "/gripper_controller/gripper_cmd");
  const auto wait = [&](const auto &predicate) {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!predicate() && std::chrono::steady_clock::now() < deadline)
      executor.spin_once(2ms);
    return predicate();
  };
  ASSERT_TRUE(wait([&] { return client->action_server_is_ready(); }));
  for (const bool reject : {true, false}) {
    Action::Goal goal;
    goal.command.position = {0.04};
    goal.command.effort = {60.0};
    auto response = client->async_send_goal(goal);
    ASSERT_TRUE(wait(
        [&] { return response.wait_for(0s) == std::future_status::ready; }));
    auto handle = response.get();
    ASSERT_NE(handle, nullptr);
    auto result = client->async_get_result(handle);
    // Callback alone must not expose an event paired with stale output values.
    if (!reject) {
      EXPECT_LT(commandValue(slot, "grip_stroke/conventional_command_sequence"),
                0.0);
      // Rejection queued Stop. Emulate its hardware consumption before a
      // replacement is eligible to publish a fresh target/effort/event.
      slot.controller->update(node->now(), rclcpp::Duration::from_seconds(0.01));
      EXPECT_LT(commandValue(slot, "grip_stroke/conventional_command_sequence"), 0.0);
      ASSERT_TRUE(findCommand(slot.command_handles, "grip_stroke/stop_command_sequence")
                      ->set_value(std::numeric_limits<double>::quiet_NaN()));
    }
    slot.controller->update(node->now(), rclcpp::Duration::from_seconds(0.01));
    EXPECT_DOUBLE_EQ(commandValue(slot, "grip_stroke/effort"), 60.0);
    const auto event = findCommand(slot.command_handles,
                                   "grip_stroke/conventional_command_sequence");
    const double token =
        commandValue(slot, "grip_stroke/conventional_command_sequence");
    ASSERT_GT(token, 0.0);
    // At-target and older than stall_timeout, but the device has not admitted
    // it.
    const auto deadline = std::chrono::steady_clock::now() + 100ms;
    while (std::chrono::steady_clock::now() < deadline) {
      slot.controller->update(node->now(),
                              rclcpp::Duration::from_seconds(0.01));
      executor.spin_once(2ms);
    }
    EXPECT_NE(result.wait_for(0s), std::future_status::ready);
    ASSERT_TRUE(event->set_value(
        reject ? -token : std::numeric_limits<double>::quiet_NaN()));
    ASSERT_TRUE(wait([&] {
      slot.controller->update(node->now(),
                              rclcpp::Duration::from_seconds(0.01));
      return result.wait_for(0s) == std::future_status::ready;
    }));
    EXPECT_EQ(result.get().code, reject ? rclcpp_action::ResultCode::ABORTED
                                        : rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_FALSE(result.get().result->stalled);
    EXPECT_EQ(result.get().result->reached_goal, !reject);
  }
  ASSERT_EQ(slot.controller->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  executor.remove_node(node);
  executor.remove_node(slot.controller->get_node()->get_node_base_interface());
  slot.controller->release_interfaces();
}

TEST_F(ParallelGripperActionControllerTest,
       ActivationRequiresConfiguredEffortHandle) {
  auto slot = makeController();
  slot.controller->release_interfaces();
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  std::vector<hardware_interface::LoanedStateInterface> states;
  for (const auto &handle : slot.command_handles) {
    if (handle->get_name() != "grip_stroke/effort")
      commands.emplace_back(handle, nullptr);
  }
  for (const auto &handle : slot.state_handles)
    states.emplace_back(handle, nullptr);
  slot.controller->assign_interfaces(std::move(commands), std::move(states));
  EXPECT_EQ(slot.controller->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::ERROR);
  slot.controller->release_interfaces();
}

TEST_F(ParallelGripperActionControllerTest,
       InvalidFeedbackRetriesStopWhenCommandHandleIsContended) {
  auto slot = makeController();
  ASSERT_EQ(slot.controller->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(findState(slot.state_handles, "grip_stroke/position")->set_value(
      std::numeric_limits<double>::quiet_NaN()));
  const auto stop = findCommand(slot.command_handles, "grip_stroke/stop_command_sequence");
  const auto period = rclcpp::Duration::from_seconds(0.01);
  {
    std::unique_lock lock(stop->get_mutex());
    EXPECT_EQ(slot.controller->update(rclcpp::Time{}, period),
              controller_interface::return_type::ERROR);
  }
  EXPECT_EQ(slot.controller->update(rclcpp::Time{}, period),
            controller_interface::return_type::OK);
  const auto token = commandValue(slot, "grip_stroke/stop_command_sequence");
  EXPECT_TRUE(std::isfinite(token));
  EXPECT_GT(token, 0.0);
  // A persistent invalid interval must not generate a new Stop every cycle.
  EXPECT_EQ(slot.controller->update(rclcpp::Time{}, period),
            controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(commandValue(slot, "grip_stroke/stop_command_sequence"), token);
  ASSERT_TRUE(findState(slot.state_handles, "grip_stroke/position")->set_value(0.04));
  ASSERT_EQ(slot.controller->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  slot.controller->release_interfaces();
}

TEST_F(ParallelGripperActionControllerTest,
       ConventionalSpeedIsOptionalAndValidatedWhileIdle) {
  auto unsupported = makeController();
  EXPECT_EQ(findCommand(unsupported.command_handles,
                        "grip_stroke/conventional_speed_percent"),
            nullptr);
  const auto unsupported_result =
      unsupported.controller->get_node()->set_parameters_atomically(
          {rclcpp::Parameter("conventional_speed_percent", 25)});
  EXPECT_FALSE(unsupported_result.successful);
  EXPECT_NE(unsupported_result.reason.find("unavailable"), std::string::npos);
  unsupported.controller->release_interfaces();

  auto slot = makeController({}, {}, false, true, 25);
  const auto speed = findCommand(slot.command_handles,
                                 "grip_stroke/conventional_speed_percent");
  ASSERT_NE(speed, nullptr);
  ASSERT_EQ(slot.controller->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(commandValue(slot, "grip_stroke/conventional_speed_percent"), 25.0);
  EXPECT_TRUE(std::isnan(commandValue(
      slot, "grip_stroke/conventional_command_sequence")));

  for (const auto &parameter : std::vector<rclcpp::Parameter>{
           rclcpp::Parameter("conventional_speed_percent", 0),
           rclcpp::Parameter("conventional_speed_percent", 101),
           rclcpp::Parameter("conventional_speed_percent",
                             static_cast<std::int64_t>(4294967296LL)),
           rclcpp::Parameter("conventional_speed_percent", 25.0),
           rclcpp::Parameter("conventional_speed_percent", std::string("25"))}) {
    const auto result =
        slot.controller->get_node()->set_parameters_atomically({parameter});
    EXPECT_FALSE(result.successful);
    EXPECT_EQ(slot.controller->get_node()
                  ->get_parameter("conventional_speed_percent").as_int(),
              25);
  }
  const auto atomic_result =
      slot.controller->get_node()->set_parameters_atomically(
          {rclcpp::Parameter("conventional_speed_percent", 75),
           rclcpp::Parameter("conventional_speed_control", false)});
  EXPECT_FALSE(atomic_result.successful);
  EXPECT_EQ(slot.controller->get_node()
                ->get_parameter("conventional_speed_percent").as_int(),
            25);

  const auto accepted =
      slot.controller->get_node()->set_parameters_atomically(
          {rclcpp::Parameter("conventional_speed_percent", 75)});
  ASSERT_TRUE(accepted.successful) << accepted.reason;
  // The write merely selects the next goal's device setting; it emits no
  // motion event and does not mutate the currently loaned output.
  EXPECT_DOUBLE_EQ(commandValue(slot, "grip_stroke/conventional_speed_percent"),
                   25.0);
  EXPECT_TRUE(std::isnan(commandValue(
      slot, "grip_stroke/conventional_command_sequence")));

  // A Stop event is a hardware handoff that has not yet been consumed. Do not
  // let a new speed selection race ahead of that handoff, even when there is
  // no active action goal left in the controller.
  const auto stop = findCommand(slot.command_handles,
                                "grip_stroke/stop_command_sequence");
  ASSERT_NE(stop, nullptr);
  ASSERT_TRUE(stop->set_value(1.0));
  const auto pending_stop =
      slot.controller->get_node()->set_parameters_atomically(
          {rclcpp::Parameter("conventional_speed_percent", 50)});
  EXPECT_FALSE(pending_stop.successful);
  EXPECT_NE(pending_stop.reason.find("Stop"), std::string::npos);
  ASSERT_TRUE(stop->set_value(std::numeric_limits<double>::quiet_NaN()));

  for (const int selected_speed : {1, 25, 50, 100}) {
    const auto selected =
        slot.controller->get_node()->set_parameters_atomically(
            {rclcpp::Parameter("conventional_speed_percent", selected_speed)});
    ASSERT_TRUE(selected.successful) << selected.reason;
    EXPECT_EQ(slot.controller->get_node()
                  ->get_parameter("conventional_speed_percent").as_int(),
              selected_speed);
  }

  ASSERT_EQ(slot.controller->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  const auto inactive =
      slot.controller->get_node()->set_parameters_atomically(
          {rclcpp::Parameter("conventional_speed_percent", 50)});
  EXPECT_FALSE(inactive.successful);
  EXPECT_NE(inactive.reason.find("inactive"), std::string::npos);
  slot.controller->release_interfaces();
}

TEST_F(ParallelGripperActionControllerTest,
       ConventionalSpeedSnapshotsWithGoalsAndRejectsBusyChanges) {
  using Action = control_msgs::action::ParallelGripperCommand;
  using namespace std::chrono_literals;
  auto slot = makeController({}, {}, true, true, 25, true);
  ASSERT_EQ(slot.controller->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(slot.controller->get_node()->get_node_base_interface());
  const auto node = std::make_shared<rclcpp::Node>("conventional_speed_client");
  executor.add_node(node);
  const auto client = rclcpp_action::create_client<Action>(
      node, "/gripper_controller/gripper_cmd");
  const auto wait = [&](const auto &predicate) {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_once(2ms);
    }
    return predicate();
  };
  ASSERT_TRUE(wait([&] { return client->action_server_is_ready(); }));

  // The speed validator is one of several parameter callbacks on this node.
  // A later validator may reject the complete atomic transaction, so merely
  // accepting the speed member must not mutate the effective goal snapshot.
  const auto rejected_atomic_update =
      slot.controller->get_node()->set_parameters_atomically(
          {rclcpp::Parameter("conventional_speed_percent", 100),
           rclcpp::Parameter("test_reject_atomic_transaction", true)});
  ASSERT_FALSE(rejected_atomic_update.successful);
  EXPECT_EQ(slot.controller->get_node()
                ->get_parameter("conventional_speed_percent")
                .as_int(),
            25);
  const auto repeated_speed_update =
      slot.controller->get_node()->set_parameters_atomically(
          {rclcpp::Parameter("conventional_speed_percent", 10),
           rclcpp::Parameter("conventional_speed_percent", 25)});
  ASSERT_TRUE(repeated_speed_update.successful) << repeated_speed_update.reason;
  EXPECT_EQ(slot.controller->get_node()
                ->get_parameter("conventional_speed_percent")
                .as_int(),
            25);

  Action::Goal goal;
  goal.command.name = {"grip_stroke"};
  goal.command.position = {0.06};
  goal.command.effort = {40.0};
  auto response = client->async_send_goal(goal);
  ASSERT_TRUE(
      wait([&] { return response.wait_for(0s) == std::future_status::ready; }));
  const auto first_goal = response.get();
  ASSERT_NE(first_goal, nullptr);
  auto first_result = client->async_get_result(first_goal);
  const auto busy_change =
      slot.controller->get_node()->set_parameters_atomically(
          {rclcpp::Parameter("conventional_speed_percent", 75)});
  EXPECT_FALSE(busy_change.successful);
  EXPECT_NE(busy_change.reason.find("active"), std::string::npos);

  ASSERT_EQ(slot.controller->update(node->now(), rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(commandValue(slot, "grip_stroke/conventional_speed_percent"), 25.0);
  const auto marker = findCommand(slot.command_handles,
                                  "grip_stroke/conventional_command_sequence");
  ASSERT_NE(marker, nullptr);
  ASSERT_TRUE(marker->set_value(std::numeric_limits<double>::quiet_NaN()));
  ASSERT_TRUE(findState(slot.state_handles, "grip_stroke/position")->set_value(0.06));
  ASSERT_TRUE(wait([&] {
    slot.controller->update(node->now(), rclcpp::Duration::from_seconds(0.01));
    return first_result.wait_for(0s) == std::future_status::ready;
  }));
  EXPECT_EQ(first_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);

  const auto idle_change = slot.controller->get_node()->set_parameters_atomically(
      {rclcpp::Parameter("conventional_speed_percent", 75)});
  ASSERT_TRUE(idle_change.successful) << idle_change.reason;
  goal.command.position = {0.05};
  response = client->async_send_goal(goal);
  ASSERT_TRUE(wait([&] { return response.wait_for(0s) == std::future_status::ready; }));
  ASSERT_NE(response.get(), nullptr);
  ASSERT_EQ(slot.controller->update(node->now(), rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(commandValue(slot, "grip_stroke/conventional_speed_percent"), 75.0);

  ASSERT_EQ(slot.controller->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  executor.remove_node(node);
  executor.remove_node(slot.controller->get_node()->get_node_base_interface());
  slot.controller->release_interfaces();
}

TEST_F(ParallelGripperActionControllerTest,
       ConventionalSpeedReservationIsRetiredBeforeAcceptedFeedbackAbort) {
  using Action = control_msgs::action::ParallelGripperCommand;
  using namespace std::chrono_literals;

  auto slot = makeController({}, {}, false, true, 25);
  ASSERT_EQ(slot.controller->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(slot.controller->get_node()->get_node_base_interface());
  const auto node = std::make_shared<rclcpp::Node>(
      "conventional_speed_reservation_client");
  executor.add_node(node);
  const auto client = rclcpp_action::create_client<Action>(
      node, "/gripper_controller/gripper_cmd");

  std::promise<void> reservation_created;
  std::promise<void> release_goal_response;
  auto release = release_goal_response.get_future().share();
  onrobot_gripper_controllers::
      set_parallel_gripper_action_controller_test_goal_reservation_hook(
      [&] {
        reservation_created.set_value();
        release.wait();
      });
  std::thread spinner([&] { executor.spin(); });
  const auto cleanup = [&] {
    onrobot_gripper_controllers::
        set_parallel_gripper_action_controller_test_goal_reservation_hook({});
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    executor.remove_node(node);
    executor.remove_node(slot.controller->get_node()->get_node_base_interface());
    slot.controller->on_deactivate(rclcpp_lifecycle::State());
    slot.controller->release_interfaces();
  };

  if (!client->wait_for_action_server(2s)) {
    release_goal_response.set_value();
    cleanup();
    FAIL() << "action server did not become available";
  }
  Action::Goal goal;
  goal.command.name = {"grip_stroke"};
  goal.command.position = {0.060};
  goal.command.effort = {40.0};
  auto response = client->async_send_goal(goal);
  if (reservation_created.get_future().wait_for(2s) !=
      std::future_status::ready) {
    release_goal_response.set_value();
    cleanup();
    FAIL() << "goal response did not reserve its conventional speed";
  }

  // The response callback has reserved the UUID, but the accepted callback is
  // blocked. A manager update can make the feedback invalid in this interval.
  const auto position = findState(slot.state_handles, "grip_stroke/position");
  if (!position || !position->set_value(std::numeric_limits<double>::quiet_NaN())) {
    release_goal_response.set_value();
    cleanup();
    FAIL() << "could not invalidate the accepted goal's measured state";
  }
  release_goal_response.set_value();

  if (response.wait_for(2s) != std::future_status::ready) {
    cleanup();
    FAIL() << "goal response did not complete";
  }
  const auto goal_handle = response.get();
  if (!goal_handle) {
    cleanup();
    FAIL() << "reserved goal was unexpectedly rejected";
  }
  auto result = client->async_get_result(goal_handle);
  if (result.wait_for(2s) != std::future_status::ready) {
    cleanup();
    FAIL() << "accepted callback did not abort invalid feedback";
  }
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);

  // This is the regression assertion: the accepted callback must retire the
  // reservation before the feedback abort, otherwise the controller remains
  // permanently busy and rejects this idle future-goal speed selection.
  const auto speed_change = slot.controller->get_node()->set_parameters_atomically(
      {rclcpp::Parameter("conventional_speed_percent", 75)});
  EXPECT_TRUE(speed_change.successful) << speed_change.reason;
  EXPECT_EQ(slot.controller->get_node()
                ->get_parameter("conventional_speed_percent").as_int(),
            75);
  ASSERT_TRUE(position->set_value(0.060));
  cleanup();
}

} // namespace

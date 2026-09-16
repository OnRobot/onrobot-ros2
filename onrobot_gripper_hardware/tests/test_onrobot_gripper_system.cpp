#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <thread>
#include <utility>

#include <gtest/gtest.h>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_loader.hpp>

#include "support/modbus_tcp_test_server.hpp"
#include "support/modbus_rtu_test_server.hpp"

#include "onrobot_gripper_hardware/onrobot_gripper_system.hpp"
#include "onrobot_gripper_hardware/onrobot_parallel_gripper_fake_system.hpp"
#include "onrobot_gripper_hardware/onrobot_rg_system.hpp"
#include "onrobot_gripper_hardware/onrobot_three_finger_system.hpp"

namespace {

using namespace std::chrono_literals;

// Exercise the shared-handle export used by Jazzy, retaining ownership in
// direct plugin tests. The separate 3FG plugin still uses the legacy export.
struct TestCommand {
  hardware_interface::CommandInterface::SharedPtr handle;
  const std::string &get_name() const { return handle->get_name(); }
  template<class T> bool set_value(const T &value) { return handle->set_value(value); }
  template<class T> std::optional<T> get_optional() const { return handle->get_optional<T>(); }
};

std::vector<TestCommand> export_commands(hardware_interface::SystemInterface &system) {
  auto legacy = system.export_command_interfaces();
  std::vector<TestCommand> result;
  if (legacy.empty()) {
    for (auto &handle : system.on_export_command_interfaces()) result.push_back({handle});
  } else {
    for (auto &handle : legacy) result.push_back({
        std::make_shared<hardware_interface::CommandInterface>(std::move(handle))});
  }
  return result;
}

hardware_interface::InterfaceInfo interface_info(const std::string &name) {
  hardware_interface::InterfaceInfo interface;
  interface.name = name;
  return interface;
}

hardware_interface::HardwareComponentInterfaceParams
component_params(hardware_interface::HardwareInfo info) {
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = std::move(info);
  return params;
}

hardware_interface::HardwareInfo valid_info() {
  hardware_interface::HardwareInfo info;
  info.name = "onrobot_2fg7";
  info.type = "system";
  info.hardware_parameters = {
      {"model", "2fg7"}, {"transport", "tcp"}, {"host", "127.0.0.1"},
      {"port", "502"},   {"slave_id", "65"},
  };

  hardware_interface::ComponentInfo joint;
  joint.name = "grip_stroke";
  joint.command_interfaces.push_back(
      interface_info(hardware_interface::HW_IF_POSITION));
  joint.command_interfaces.push_back(
      interface_info(hardware_interface::HW_IF_EFFORT));
  joint.command_interfaces.push_back(
      interface_info("fault_recovery_command_sequence"));
  joint.command_interfaces.push_back(interface_info("stop_command_sequence"));
  joint.command_interfaces.push_back(
      interface_info("conventional_command_sequence"));
  joint.state_interfaces.push_back(
      interface_info(hardware_interface::HW_IF_POSITION));
  joint.state_interfaces.push_back(
      interface_info(hardware_interface::HW_IF_VELOCITY));
  joint.state_interfaces.push_back(
      interface_info(hardware_interface::HW_IF_EFFORT));
  info.joints.push_back(joint);
  return info;
}

hardware_interface::HardwareInfo valid_rg_info() {
  auto info = valid_info();
  info.name = "onrobot_rg2";
  info.hardware_parameters["model"] = "rg2";
  info.hardware_parameters["port"] = "502";
  info.hardware_parameters["safe_width_min_mm"] = "0";
  info.hardware_parameters["safe_width_max_mm"] = "110";
  info.joints.front().state_interfaces.pop_back();
  hardware_interface::ComponentInfo visual_joint;
  visual_joint.name = "finger_joint";
  visual_joint.state_interfaces.push_back(
      interface_info(hardware_interface::HW_IF_POSITION));
  visual_joint.state_interfaces.push_back(
      interface_info(hardware_interface::HW_IF_VELOCITY));
  info.joints.push_back(visual_joint);
  return info;
}

hardware_interface::HardwareInfo
realtime_rg_fault_info(uint16_t i_port, const std::string &i_model,
                       double i_maximumWidthMm) {
  auto info = valid_rg_info();
  info.name = "onrobot_" + i_model;
  info.hardware_parameters["model"] = i_model;
  info.hardware_parameters["port"] = std::to_string(i_port);
  info.hardware_parameters["safe_width_max_mm"] =
      std::to_string(i_maximumWidthMm);
  // Keep fault injection deterministic: one failed transport cycle is enough
  // to latch the session, so the test does not depend on a long TCP timeout.
  info.hardware_parameters["realtime_maximum_consecutive_failures"] = "1";

  auto &joint = info.joints.front();
  for (const auto *name : {"realtime_mode", "realtime_task_position",
                           "realtime_mechanism_angular_velocity",
                           "realtime_force", "realtime_command_sequence"}) {
    joint.command_interfaces.push_back(interface_info(name));
  }
  joint.state_interfaces.push_back(interface_info("task_position_valid"));
  joint.state_interfaces.push_back(interface_info("force_valid"));
  joint.state_interfaces.push_back(interface_info("faulted"));
  for (const auto *name :
       {"active_mode", "reconnects", "requested_command_sequence",
        "applied_command_sequence"}) {
    joint.state_interfaces.push_back(interface_info(name));
  }
  return info;
}

void assert_rg_realtime_stop_does_not_recover(const std::string &i_model,
                                              uint16_t i_productCode,
                                              double i_maximumWidthMm) {
  ModbusTcpTestServer server(i_productCode);
  server.set_register(0x010B, 730);
  server.set_register(0x0113, 730);
  server.set_register(0x0109, 200);
  server.set_register(0x0114, 0);
  server.set_register(0x0115, 0);

  onrobot_gripper_hardware::OnRobotRgSystem system;
  ASSERT_EQ(system.on_init(component_params(realtime_rg_fault_info(
                server.port(), i_model, i_maximumWidthMm))),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);

  auto commands = export_commands(system);
  const auto find_command = [&commands](const std::string &i_name) {
    return std::find_if(commands.begin(), commands.end(),
                        [&i_name](const auto &i_interface) {
                          return i_interface.get_name() == i_name;
                        });
  };
  const auto mode = find_command("grip_stroke/realtime_mode");
  const auto position = find_command("grip_stroke/realtime_task_position");
  const auto angularVelocity =
      find_command("grip_stroke/realtime_mechanism_angular_velocity");
  const auto force = find_command("grip_stroke/realtime_force");
  const auto sequence = find_command("grip_stroke/realtime_command_sequence");
  const auto recovery =
      find_command("grip_stroke/fault_recovery_command_sequence");
  ASSERT_NE(mode, commands.end());
  ASSERT_NE(position, commands.end());
  ASSERT_NE(angularVelocity, commands.end());
  ASSERT_NE(force, commands.end());
  ASSERT_NE(sequence, commands.end());
  ASSERT_NE(recovery, commands.end());

  const auto states = system.export_state_interfaces();
  const auto find_state = [&states](const std::string &i_name) {
    return std::find_if(states.begin(), states.end(),
                        [&i_name](const auto &i_interface) {
                          return i_interface.get_name() == i_name;
                        });
  };
  const auto faulted = find_state("grip_stroke/faulted");
  const auto reconnects = find_state("grip_stroke/reconnects");
  const auto requestedSequence =
      find_state("grip_stroke/requested_command_sequence");
  const auto appliedSequence =
      find_state("grip_stroke/applied_command_sequence");
  ASSERT_NE(faulted, states.end());
  ASSERT_NE(reconnects, states.end());
  ASSERT_NE(requestedSequence, states.end());
  ASSERT_NE(appliedSequence, states.end());

  ASSERT_TRUE(mode->set_value(0.0));
  ASSERT_TRUE(position->set_value(0.050));
  ASSERT_TRUE(angularVelocity->set_value(0.0));
  ASSERT_TRUE(force->set_value(10.0));
  ASSERT_TRUE(sequence->set_value(1.0));
  ASSERT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
            hardware_interface::return_type::OK);
  const auto commandDeadline = std::chrono::steady_clock::now() + 500ms;
  ASSERT_TRUE([&] {
    while (std::chrono::steady_clock::now() < commandDeadline) {
      if (server.function_count(23) > 0) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }());

  // A dropped transport response latches the session without involving a
  // physical gripper or a firmware safety condition.
  server.disconnect_client();
  ASSERT_TRUE([&] {
    const auto faultDeadline = std::chrono::steady_clock::now() + 1500ms;
    while (std::chrono::steady_clock::now() < faultDeadline) {
      if (system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)) !=
          hardware_interface::return_type::OK) {
        return false;
      }
      if (faulted->get_value() > 0.5) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }());
  const auto function16AtFault = server.function_count(16);
  const auto function23AtFault = server.function_count(23);
  const auto requestedSequenceAtFault = requestedSequence->get_value();
  const auto appliedSequenceAtFault = appliedSequence->get_value();

  // Releasing the realtime command must only queue Stop. It must not turn a
  // transport fault into an implicit reconnect or replay an old command.
  ASSERT_TRUE(mode->set_value(-1.0));
  ASSERT_TRUE(sequence->set_value(2.0));
  ASSERT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
            hardware_interface::return_type::OK);
  for (int i = 0; i < 50; ++i) {
    ASSERT_EQ(system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
              hardware_interface::return_type::OK);
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_DOUBLE_EQ(faulted->get_value(), 1.0);
  EXPECT_DOUBLE_EQ(reconnects->get_value(), 0.0);
  EXPECT_GT(requestedSequence->get_value(), requestedSequenceAtFault);
  EXPECT_DOUBLE_EQ(appliedSequence->get_value(), appliedSequenceAtFault);
  EXPECT_EQ(server.function_count(16), function16AtFault);
  EXPECT_EQ(server.function_count(23), function23AtFault);

  // The dedicated recovery interface remains the only operation allowed to
  // reconnect a latched session.
  ASSERT_TRUE(recovery->set_value(1.0));
  ASSERT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
            hardware_interface::return_type::OK);
  ASSERT_TRUE([&] {
    const auto recoveryDeadline = std::chrono::steady_clock::now() + 1500ms;
    while (std::chrono::steady_clock::now() < recoveryDeadline) {
      if (system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)) !=
          hardware_interface::return_type::OK) {
        return false;
      }
      if (faulted->get_value() < 0.5 && reconnects->get_value() >= 1.0) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }());
  EXPECT_GT(server.function_count(16), function16AtFault);
  EXPECT_EQ(server.function_count(23), function23AtFault);
  EXPECT_DOUBLE_EQ(appliedSequence->get_value(),
                   requestedSequence->get_value());
  ASSERT_EQ(system.on_deactivate(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);
}

hardware_interface::HardwareInfo fault_observation_info(uint16_t i_port) {
  auto info = valid_info();
  info.hardware_parameters["port"] = std::to_string(i_port);
  info.joints.front().state_interfaces.push_back(
      interface_info("task_position_valid"));
  info.joints.front().state_interfaces.push_back(interface_info("force_valid"));
  info.joints.front().state_interfaces.push_back(interface_info("faulted"));
  return info;
}

hardware_interface::HardwareInfo realtime_fault_info(uint16_t i_port) {
  auto info = fault_observation_info(i_port);
  // Keep the transport fault bounded and deterministic: one missing response
  // is enough to enter the latched state under test.
  info.hardware_parameters["realtime_maximum_consecutive_failures"] = "1";
  auto &joint = info.joints.front();
  for (const auto *name :
       {"realtime_mode", "realtime_task_position", "realtime_task_velocity",
        "realtime_force", "realtime_command_sequence"}) {
    joint.command_interfaces.push_back(interface_info(name));
  }
  for (const auto *name :
       {"active_mode", "reconnects", "requested_command_sequence",
        "applied_command_sequence"}) {
    joint.state_interfaces.push_back(interface_info(name));
  }
  return info;
}

void assert_recovery_supersedes_pending_stop(bool i_rg, uint16_t i_productCode,
                                             double i_maximumWidthMm) {
  ModbusTcpTestServer server(i_productCode);
  server.set_register(0x0109, 500);
  server.set_register(0x0115, 196);
  server.set_register(0x0116, 0);
  server.set_register(0x0117, 562);
  if (i_rg) {
    server.set_register(0x010B, 730);
    server.set_register(0x0113, 730);
    server.set_register(0x0109, 200);
    server.set_register(0x0114, 0);
    server.set_register(0x0115, 0);
  }

  auto info =
      i_rg ? realtime_rg_fault_info(server.port(),
                                    i_productCode == 0x0021 ? "rg6" : "rg2",
                                    i_maximumWidthMm)
           : realtime_fault_info(server.port());
  if (!i_rg && i_productCode == 0x00c1) {
    info.hardware_parameters["model"] = "2fg14";
  }
  onrobot_gripper_hardware::OnRobotRgSystem rgSystem;
  onrobot_gripper_hardware::OnRobotGripperSystem twoFgSystem;
  auto *system =
      i_rg ? static_cast<hardware_interface::SystemInterface *>(&rgSystem)
           : static_cast<hardware_interface::SystemInterface *>(&twoFgSystem);
  ASSERT_EQ(system->on_init(component_params(info)),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);

  auto commands = export_commands(*system);
  const auto find_command = [&commands](const std::string &i_name) {
    return std::find_if(commands.begin(), commands.end(),
                        [&i_name](const auto &i_interface) {
                          return i_interface.get_name() == i_name;
                        });
  };
  const auto stop = find_command("grip_stroke/stop_command_sequence");
  const auto recovery =
      find_command("grip_stroke/fault_recovery_command_sequence");
  const auto position = find_command("grip_stroke/position");
  ASSERT_NE(stop, commands.end());
  ASSERT_NE(recovery, commands.end());
  ASSERT_NE(position, commands.end());

  const auto states = system->export_state_interfaces();
  const auto find_state = [&states](const std::string &i_name) {
    return std::find_if(states.begin(), states.end(),
                        [&i_name](const auto &i_interface) {
                          return i_interface.get_name() == i_name;
                        });
  };
  const auto faulted = find_state("grip_stroke/faulted");
  const auto reconnects = find_state("grip_stroke/reconnects");
  const auto requested = find_state("grip_stroke/requested_command_sequence");
  const auto applied = find_state("grip_stroke/applied_command_sequence");
  ASSERT_NE(faulted, states.end());
  ASSERT_NE(reconnects, states.end());
  ASSERT_NE(requested, states.end());
  ASSERT_NE(applied, states.end());

  // The worker must be in a realtime exchange before the Stop barrier is
  // armed. This makes the paused function-16 request the actual Stop
  // transaction, rather than the activation Stop.
  auto set_realtime_command = [&] {
    const auto mode = find_command("grip_stroke/realtime_mode");
    const auto sequence = find_command("grip_stroke/realtime_command_sequence");
    ASSERT_NE(mode, commands.end());
    ASSERT_NE(sequence, commands.end());
    ASSERT_TRUE(mode->set_value(0.0));
    ASSERT_TRUE(sequence->set_value(1.0));
    if (i_rg) {
      const auto position = find_command("grip_stroke/realtime_task_position");
      const auto angularVelocity =
          find_command("grip_stroke/realtime_mechanism_angular_velocity");
      const auto force = find_command("grip_stroke/realtime_force");
      ASSERT_NE(position, commands.end());
      ASSERT_NE(angularVelocity, commands.end());
      ASSERT_NE(force, commands.end());
      ASSERT_TRUE(position->set_value(0.050));
      ASSERT_TRUE(angularVelocity->set_value(0.0));
      ASSERT_TRUE(force->set_value(10.0));
    } else {
      const auto position = find_command("grip_stroke/realtime_task_position");
      const auto velocity = find_command("grip_stroke/realtime_task_velocity");
      const auto force = find_command("grip_stroke/realtime_force");
      ASSERT_NE(position, commands.end());
      ASSERT_NE(velocity, commands.end());
      ASSERT_NE(force, commands.end());
      ASSERT_TRUE(position->set_value(0.020));
      ASSERT_TRUE(velocity->set_value(0.010));
      ASSERT_TRUE(force->set_value(0.0));
    }
    ASSERT_EQ(
        system->write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
        hardware_interface::return_type::OK);
  };
  set_realtime_command();
  ASSERT_TRUE([&] {
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (std::chrono::steady_clock::now() < deadline) {
      if (server.function_count(23) > 0) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }());

  server.pause_next_request(16);
  ASSERT_TRUE(stop->set_value(2.0));
  ASSERT_EQ(system->write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
            hardware_interface::return_type::OK);
  ASSERT_TRUE(server.wait_for_paused_request(500ms));
  const auto requestedBeforeFault = requested->get_value();
  const auto appliedBeforeFault = applied->get_value();

  // Drop the peer while the submitted Stop is awaiting its response. This is
  // a deterministic transport failure between submission and acknowledgement;
  // recovery later establishes a fresh connection on the same endpoint.
  server.suppress_responses(true);
  server.abort_paused_request();
  ASSERT_TRUE([&] {
    const auto deadline = std::chrono::steady_clock::now() + 1500ms;
    while (std::chrono::steady_clock::now() < deadline) {
      system->read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0));
      if (faulted->get_value() > 0.5) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }());
  server.suppress_responses(false);
  EXPECT_GT(requested->get_value(), requestedBeforeFault);
  // A failed Stop may be followed by the session's protective Stop while it
  // latches the fault. That later acknowledged Stop may advance the applied
  // identity, but it must never replay the preceding realtime command.
  EXPECT_GE(applied->get_value(), appliedBeforeFault);
  EXPECT_LE(applied->get_value(), requested->get_value());
  const auto realtimeAtFault = server.function_count(23);
  const auto stopTransactionsAtFault = server.function_count(16);

  // A conventional target arriving while the failed Stop is still pending is
  // also barred. This is the adapter-side half of the safety contract; the
  // session's fault admission is covered independently by Tool API tests.
  ASSERT_TRUE(position->set_value(i_rg ? i_maximumWidthMm / 1000.0 : 0.020));
  ASSERT_EQ(system->write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
            hardware_interface::return_type::OK);
  EXPECT_EQ(server.function_count(16), stopTransactionsAtFault);
  EXPECT_EQ(server.function_count(23), realtimeAtFault);

  // Recovery sequence identity is an integer marker, not an arbitrary
  // finite floating-point value. Invalid values must not reconnect.
  for (const auto invalid :
       {-1.0, 0.0, 1.5, std::numeric_limits<double>::infinity(),
        9007199254740992.0}) {
    ASSERT_TRUE(recovery->set_value(invalid));
    ASSERT_DOUBLE_EQ(recovery->get_optional<double>().value(), invalid);
    EXPECT_EQ(
        system->write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
        hardware_interface::return_type::ERROR);
    EXPECT_EQ(server.function_count(16), stopTransactionsAtFault);
    EXPECT_EQ(server.function_count(23), realtimeAtFault);
  }

  // This must bypass only the adapter's pending-Stop fence. Recovery itself
  // queues an idle image before the worker reconnects.
  ASSERT_TRUE(recovery->set_value(1.0));
  ASSERT_EQ(system->write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
            hardware_interface::return_type::OK);
  ASSERT_TRUE([&] {
    const auto deadline = std::chrono::steady_clock::now() + 1500ms;
    while (std::chrono::steady_clock::now() < deadline) {
      system->read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0));
      if (faulted->get_value() < 0.5 && reconnects->get_value() >= 1.0) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }());
  EXPECT_GT(server.function_count(16), stopTransactionsAtFault);
  EXPECT_EQ(server.function_count(23), realtimeAtFault);
  EXPECT_DOUBLE_EQ(applied->get_value(), requested->get_value());

  // The pre-fault conventional target is still held by ros2_control. Several
  // ordinary write cycles must not reinterpret it as fresh post-recovery
  // intent or emit another conventional Modbus command.
  const auto stopTransactionsAfterRecovery = server.function_count(16);
  for (int i = 0; i < 5; ++i) {
    ASSERT_EQ(
        system->write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
        hardware_interface::return_type::OK);
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(server.function_count(16), stopTransactionsAfterRecovery);
  EXPECT_EQ(server.function_count(23), realtimeAtFault);

  // A distinct, valid conventional target is accepted after recovery.
  ASSERT_TRUE(position->set_value(i_rg ? 0.010 : 0.030));
  ASSERT_EQ(system->write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
            hardware_interface::return_type::OK);
  ASSERT_TRUE([&] {
    const auto motionDeadline = std::chrono::steady_clock::now() + 500ms;
    while (std::chrono::steady_clock::now() < motionDeadline) {
      if (server.function_count(16) > stopTransactionsAfterRecovery) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }());
  ASSERT_EQ(system->on_deactivate(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);
}

hardware_interface::HardwareInfo velocity_observation_info(uint16_t i_port) {
  auto info = valid_info();
  info.hardware_parameters["port"] = std::to_string(i_port);
  info.hardware_parameters["raw_linear_min_mm"] = "5";
  info.hardware_parameters["raw_linear_max_mm"] = "45";
  info.hardware_parameters["finger_joint_upper_m"] = "0.02";
  hardware_interface::ComponentInfo mechanism;
  mechanism.name = "finger_stroke";
  mechanism.state_interfaces.push_back(
      interface_info(hardware_interface::HW_IF_POSITION));
  mechanism.state_interfaces.push_back(
      interface_info(hardware_interface::HW_IF_VELOCITY));
  mechanism.state_interfaces.push_back(interface_info("measured_position"));
  mechanism.state_interfaces.push_back(interface_info("measured_velocity"));
  mechanism.state_interfaces.push_back(interface_info("position_valid"));
  mechanism.state_interfaces.push_back(interface_info("velocity_valid"));
  info.joints.push_back(mechanism);
  return info;
}

hardware_interface::HardwareInfo valid_three_finger_info() {
  hardware_interface::HardwareInfo info;
  info.name = "onrobot_3fg25";
  info.type = "system";
  info.hardware_parameters = {
      {"model", "3fg25"}, {"transport", "tcp"}, {"host", "127.0.0.1"},
      {"port", "502"},    {"slave_id", "65"},   {"default_force_percent", "20"},
  };

  hardware_interface::ComponentInfo diameter;
  diameter.name = "grip_diameter";
  diameter.command_interfaces.push_back(
      interface_info(hardware_interface::HW_IF_POSITION));
  diameter.command_interfaces.push_back(
      interface_info(hardware_interface::HW_IF_EFFORT));
  diameter.state_interfaces.push_back(
      interface_info(hardware_interface::HW_IF_POSITION));
  diameter.state_interfaces.push_back(
      interface_info(hardware_interface::HW_IF_VELOCITY));

  hardware_interface::ComponentInfo angle;
  angle.name = "finger_angle";
  angle.state_interfaces.push_back(
      interface_info(hardware_interface::HW_IF_POSITION));
  angle.state_interfaces.push_back(
      interface_info(hardware_interface::HW_IF_VELOCITY));

  info.joints = {diameter, angle};
  return info;
}

TEST(OnRobotGripperSystem, InitializesAndExportsDeclaredInterfaces) {
  onrobot_gripper_hardware::OnRobotGripperSystem system;
  ASSERT_EQ(system.on_init(component_params(valid_info())),
            hardware_interface::CallbackReturn::SUCCESS);

  const auto states = system.export_state_interfaces();
  ASSERT_EQ(states.size(), 3u);
  EXPECT_EQ(states[0].get_name(), "grip_stroke/position");
  EXPECT_EQ(states[1].get_name(), "grip_stroke/velocity");
  EXPECT_EQ(states[2].get_name(), "grip_stroke/effort");

  const auto commands = export_commands(system);
  ASSERT_EQ(commands.size(), 5u);
  EXPECT_EQ(commands[0].get_name(), "grip_stroke/position");
  EXPECT_EQ(commands[1].get_name(), "grip_stroke/effort");
  EXPECT_EQ(commands[2].get_name(),
            "grip_stroke/fault_recovery_command_sequence");
  EXPECT_EQ(commands[3].get_name(), "grip_stroke/stop_command_sequence");
  EXPECT_EQ(commands[4].get_name(),
            "grip_stroke/conventional_command_sequence");
}

TEST(OnRobotGripperSystem, ExportsPhysicalFingerAndRawMechanismState) {
  auto info = valid_info();
  hardware_interface::ComponentInfo mechanism;
  mechanism.name = "finger_stroke";
  mechanism.state_interfaces.push_back(
      interface_info(hardware_interface::HW_IF_POSITION));
  mechanism.state_interfaces.push_back(
      interface_info(hardware_interface::HW_IF_VELOCITY));
  info.joints.push_back(mechanism);

  onrobot_gripper_hardware::OnRobotGripperSystem system;
  ASSERT_EQ(system.on_init(component_params(info)),
            hardware_interface::CallbackReturn::SUCCESS);
  const auto states = system.export_state_interfaces();
  ASSERT_EQ(states.size(), 5u);
  EXPECT_EQ(states[3].get_name(), "finger_stroke/position");
  EXPECT_EQ(states[4].get_name(), "finger_stroke/velocity");
  EXPECT_EQ(export_commands(system).size(), 5u);
}

TEST(OnRobotGripperSystem,
     ConventionalGoalIdentityReissuesSameTargetAfterNumericDeduplication) {
  ModbusTcpTestServer server;
  server.set_register(0x0109, 500);
  server.set_register(0x0115, 196);
  server.set_register(0x0116, 0);
  server.set_register(0x0117, 562);

  auto info = valid_info();
  info.hardware_parameters["port"] = std::to_string(server.port());
  onrobot_gripper_hardware::OnRobotGripperSystem configured;
  ASSERT_EQ(configured.on_init(component_params(info)),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(configured.on_configure(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(configured.on_activate(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);

  auto commands = export_commands(configured);
  const auto find_command = [&commands](const std::string &name) {
    return std::find_if(
        commands.begin(), commands.end(),
        [&name](const auto &command) { return command.get_name() == name; });
  };
  const auto position = find_command("grip_stroke/position");
  const auto effort = find_command("grip_stroke/effort");
  const auto identity =
      find_command("grip_stroke/conventional_command_sequence");
  ASSERT_NE(position, commands.end());
  ASSERT_NE(effort, commands.end());
  ASSERT_NE(identity, commands.end());
  ASSERT_TRUE(position->set_value(0.020));
  // The fixture's initial aperture is 20 mm and the 2FG7 default force is
  // now the documented 20 N minimum.  Use a distinct valid effort so this
  // test exercises the first conventional command rather than the activation
  // seed-deduplication path.
  ASSERT_TRUE(effort->set_value(30.0));

  const auto before_first = server.function_count(16);
  ASSERT_EQ(
      configured.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
      hardware_interface::return_type::OK);
  ASSERT_TRUE([&] {
    // The worker may still be starting after the identity read. Keep this
    // bounded, and model the controller manager's repeated write cycles so a
    // deliberately nonblocking Busy admission is retried.
    const auto deadline = std::chrono::steady_clock::now() + 1500ms;
    while (std::chrono::steady_clock::now() < deadline) {
      if (server.function_count(16) > before_first) {
        return true;
      }
      if (configured.write(rclcpp::Time{},
                           rclcpp::Duration::from_seconds(0.0)) !=
          hardware_interface::return_type::OK) {
        return false;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }());
  const auto after_first = server.function_count(16);

  // Repeated controller updates for one goal remain deduplicated.
  ASSERT_EQ(
      configured.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
      hardware_interface::return_type::OK);
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(server.function_count(16), after_first);

  // A distinct accepted goal is identified separately from its numeric
  // target, so the same aperture/effort is sent again after the controller's
  // Stop fence.
  ASSERT_TRUE(identity->set_value(1.0));
  ASSERT_EQ(
      configured.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
      hardware_interface::return_type::OK);
  ASSERT_TRUE([&] {
    const auto deadline = std::chrono::steady_clock::now() + 1500ms;
    while (std::chrono::steady_clock::now() < deadline) {
      if (server.function_count(16) > after_first) {
        return true;
      }
      if (configured.write(rclcpp::Time{},
                           rclcpp::Duration::from_seconds(0.0)) !=
          hardware_interface::return_type::OK) {
        return false;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }());
  EXPECT_EQ(configured.on_deactivate(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);
}

TEST(OnRobotGripperSystem,
     RgConventionalGoalIdentityReissuesSameTargetAfterNumericDeduplication) {
  ModbusTcpTestServer server(0x0020);
  server.set_register(0x010B, 730);
  server.set_register(0x0113, 730);
  server.set_register(0x0109, 200);
  server.set_register(0x0114, 0);
  server.set_register(0x0115, 0);
  auto info = valid_rg_info();
  info.hardware_parameters["port"] = std::to_string(server.port());

  onrobot_gripper_hardware::OnRobotRgSystem system;
  ASSERT_EQ(system.on_init(component_params(info)),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);

  auto commands = export_commands(system);
  const auto find_command = [&commands](const std::string &name) {
    return std::find_if(
        commands.begin(), commands.end(),
        [&name](const auto &command) { return command.get_name() == name; });
  };
  const auto position = find_command("grip_stroke/position");
  const auto effort = find_command("grip_stroke/effort");
  const auto identity =
      find_command("grip_stroke/conventional_command_sequence");
  ASSERT_NE(position, commands.end());
  ASSERT_NE(effort, commands.end());
  ASSERT_NE(identity, commands.end());
  ASSERT_TRUE(position->set_value(0.050));
  ASSERT_TRUE(effort->set_value(20.0));

  const auto before_first = server.function_count(16);
  ASSERT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
            hardware_interface::return_type::OK);
  ASSERT_TRUE([&] {
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (std::chrono::steady_clock::now() < deadline) {
      if (server.function_count(16) > before_first) {
        return true;
      }
      if (system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)) !=
          hardware_interface::return_type::OK) {
        return false;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }());
  const auto after_first = server.function_count(16);
  ASSERT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
            hardware_interface::return_type::OK);
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(server.function_count(16), after_first);

  ASSERT_TRUE(identity->set_value(1.0));
  ASSERT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
            hardware_interface::return_type::OK);
  ASSERT_TRUE([&] {
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (std::chrono::steady_clock::now() < deadline) {
      if (server.function_count(16) > after_first) {
        return true;
      }
      if (system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)) !=
          hardware_interface::return_type::OK) {
        return false;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }());
  EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);
}

TEST(OnRobotGripperSystem, ConventionalSpeedPercentReachesModbusCommand) {
  for (const bool large : {false, true}) {
    SCOPED_TRACE(large ? "2FG14" : "2FG7");
    ModbusTcpTestServer server(large ? 0x00c1 : 0x00c0);
    auto info = valid_info();
    info.hardware_parameters["model"] = large ? "2fg14" : "2fg7";
    info.hardware_parameters["port"] = std::to_string(server.port());
    info.hardware_parameters["speed_percent"] = "1";

    onrobot_gripper_hardware::OnRobotGripperSystem system;
    ASSERT_EQ(system.on_init(component_params(info)),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}),
              hardware_interface::CallbackReturn::SUCCESS);

    auto commands = export_commands(system);
    const auto find_command = [&commands](const std::string &name) {
      return std::find_if(
          commands.begin(), commands.end(),
          [&name](const auto &command) { return command.get_name() == name; });
    };
    const auto position = find_command("grip_stroke/position");
    const auto effort = find_command("grip_stroke/effort");
    const auto identity =
        find_command("grip_stroke/conventional_command_sequence");
    ASSERT_NE(position, commands.end());
    ASSERT_NE(effort, commands.end());
    ASSERT_NE(identity, commands.end());

    ASSERT_TRUE(position->set_value(0.020));
    ASSERT_TRUE(effort->set_value(large ? 50.0 : 30.0));
    ASSERT_TRUE(identity->set_value(1.0));

    const auto before = server.function_count(16);
    ASSERT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
              hardware_interface::return_type::OK);
    ASSERT_TRUE([&] {
      const auto deadline = std::chrono::steady_clock::now() + 1500ms;
      while (std::chrono::steady_clock::now() < deadline) {
        if (system.write(rclcpp::Time{},
                         rclcpp::Duration::from_seconds(0.0)) !=
            hardware_interface::return_type::OK) {
          return false;
        }
        if (server.function_count(16) > before &&
            server.register_value(0x0003) == 1) {
          return true;
        }
        std::this_thread::sleep_for(1ms);
      }
      return false;
    }());
    EXPECT_EQ(server.register_value(0x0000), 200);
    EXPECT_EQ(server.register_value(0x0001), large ? 50 : 30);
    EXPECT_EQ(server.register_value(0x0002), 1);
    EXPECT_EQ(server.register_value(0x0003), 1);
    ASSERT_EQ(system.on_deactivate(rclcpp_lifecycle::State{}),
              hardware_interface::CallbackReturn::SUCCESS);
  }
}

TEST(OnRobotGripperSystem,
     RuntimeConventionalSpeedResourceIsSnapshotAtEachTwoFingerIntent) {
  for (const bool large : {false, true}) {
    SCOPED_TRACE(large ? "2FG14" : "2FG7");
    ModbusTcpTestServer server(large ? 0x00c1 : 0x00c0);
    auto info = valid_info();
    info.hardware_parameters["model"] = large ? "2fg14" : "2fg7";
    info.hardware_parameters["port"] = std::to_string(server.port());
    info.joints.front().command_interfaces.push_back(
        interface_info("conventional_speed_percent"));

    onrobot_gripper_hardware::OnRobotGripperSystem system;
    ASSERT_EQ(system.on_init(component_params(info)),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}),
              hardware_interface::CallbackReturn::SUCCESS);

    auto commands = export_commands(system);
    const auto find_command = [&commands](const std::string &name) {
      return std::find_if(commands.begin(), commands.end(),
                          [&name](const auto &command) {
                            return command.get_name() == name;
                          });
    };
    const auto position = find_command("grip_stroke/position");
    const auto effort = find_command("grip_stroke/effort");
    const auto speed = find_command("grip_stroke/conventional_speed_percent");
    const auto identity =
        find_command("grip_stroke/conventional_command_sequence");
    ASSERT_NE(position, commands.end());
    ASSERT_NE(effort, commands.end());
    ASSERT_NE(speed, commands.end());
    ASSERT_NE(identity, commands.end());

    const auto dispatch = [&](double selected_speed, double sequence,
                              uint32_t before) {
      ASSERT_TRUE(position->set_value(0.020));
      ASSERT_TRUE(effort->set_value(large ? 50.0 : 30.0));
      ASSERT_TRUE(speed->set_value(selected_speed));
      ASSERT_TRUE(identity->set_value(sequence));
      ASSERT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
                hardware_interface::return_type::OK);
      ASSERT_TRUE([&] {
        const auto deadline = std::chrono::steady_clock::now() + 1500ms;
        while (std::chrono::steady_clock::now() < deadline) {
          if (system.write(rclcpp::Time{},
                           rclcpp::Duration::from_seconds(0.0)) !=
              hardware_interface::return_type::OK) {
            return false;
          }
          if (server.function_count(16) > before &&
              server.register_value(0x0003) == 1) {
            return true;
          }
          std::this_thread::sleep_for(1ms);
        }
        return false;
      }());
      EXPECT_EQ(server.register_value(0x0000), 200);
      EXPECT_EQ(server.register_value(0x0001), large ? 50 : 30);
      EXPECT_EQ(server.register_value(0x0002), selected_speed);
      EXPECT_EQ(server.register_value(0x0003), 1);
    };

    uint32_t before = server.function_count(16);
    double sequence = 1.0;
    for (const double selected_speed : {1.0, 25.0, 50.0, 100.0}) {
      // A new identity with exactly the same target and force must retain its
      // own selected speed rather than being swallowed by numeric deduplication.
      dispatch(selected_speed, sequence++, before);
      before = server.function_count(16);
    }
    EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State{}),
              hardware_interface::CallbackReturn::SUCCESS);
  }
}

TEST(OnRobotGripperSystem,
     RuntimeConventionalSpeedResourceReachesBothTwoFingerRtuPeers) {
  for (const bool large : {false, true}) {
    SCOPED_TRACE(large ? "2FG14 RTU" : "2FG7 RTU");
    ModbusRtuTestServer server(large ? 0x00c1 : 0x00c0);
    auto info = valid_info();
    info.hardware_parameters["model"] = large ? "2fg14" : "2fg7";
    info.hardware_parameters["transport"] = "rtu";
    info.hardware_parameters["serial_device"] = server.device();
    info.hardware_parameters["baud_rate"] = "1000000";
    info.hardware_parameters["rtu_parity"] = "even";
    info.hardware_parameters["slave_id"] = "65";
    info.joints.front().command_interfaces.push_back(
        interface_info("conventional_speed_percent"));

    onrobot_gripper_hardware::OnRobotGripperSystem system;
    ASSERT_EQ(system.on_init(component_params(info)),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}),
              hardware_interface::CallbackReturn::SUCCESS);

    auto commands = export_commands(system);
    const auto find_command = [&commands](const std::string &name) {
      return std::find_if(commands.begin(), commands.end(),
                          [&name](const auto &command) {
                            return command.get_name() == name;
                          });
    };
    const auto position = find_command("grip_stroke/position");
    const auto effort = find_command("grip_stroke/effort");
    const auto speed = find_command("grip_stroke/conventional_speed_percent");
    const auto identity =
        find_command("grip_stroke/conventional_command_sequence");
    ASSERT_NE(position, commands.end());
    ASSERT_NE(effort, commands.end());
    ASSERT_NE(speed, commands.end());
    ASSERT_NE(identity, commands.end());

    uint32_t before = server.function_count(16);
    double sequence = 1.0;
    for (const double selected_speed : {1.0, 25.0, 50.0, 100.0}) {
      ASSERT_TRUE(position->set_value(0.020));
      ASSERT_TRUE(effort->set_value(large ? 50.0 : 30.0));
      ASSERT_TRUE(speed->set_value(selected_speed));
      ASSERT_TRUE(identity->set_value(sequence++));
      ASSERT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
                hardware_interface::return_type::OK);
      ASSERT_TRUE([&] {
        const auto deadline = std::chrono::steady_clock::now() + 1500ms;
        while (std::chrono::steady_clock::now() < deadline) {
          if (system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)) !=
              hardware_interface::return_type::OK) {
            return false;
          }
          if (server.function_count(16) > before &&
              server.register_value(0x0003) == 1) {
            return true;
          }
          std::this_thread::sleep_for(1ms);
        }
        return false;
      }());
      EXPECT_EQ(server.register_value(0x0000), 200);
      EXPECT_EQ(server.register_value(0x0001), large ? 50 : 30);
      EXPECT_EQ(server.register_value(0x0002), selected_speed);
      EXPECT_EQ(server.register_value(0x0003), 1);
      before = server.function_count(16);
    }
    EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State{}),
              hardware_interface::CallbackReturn::SUCCESS);
  }
}

TEST(OnRobotGripperSystem, RejectsOutOfRangeConventionalSpeedBeforeConnecting) {
  for (const auto *value : {"0", "101"}) {
    auto info = valid_info();
    info.hardware_parameters["speed_percent"] = value;

    onrobot_gripper_hardware::OnRobotGripperSystem system;
    EXPECT_EQ(system.on_init(component_params(info)),
              hardware_interface::CallbackReturn::ERROR);
  }
}

TEST(OnRobotGripperSystem,
     RejectsInvalidTwoFingerForceWithoutReplayingThePosition) {
  for (const bool large : {false, true}) {
    SCOPED_TRACE(large ? "2FG14" : "2FG7");
    ModbusTcpTestServer server(large ? 0x00c1 : 0x00c0);
    auto info = valid_info();
    info.hardware_parameters["model"] = large ? "2fg14" : "2fg7";
    info.hardware_parameters["port"] = std::to_string(server.port());

    onrobot_gripper_hardware::OnRobotGripperSystem system;
    ASSERT_EQ(system.on_init(component_params(info)),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}),
              hardware_interface::CallbackReturn::SUCCESS);

    auto commands = export_commands(system);
    const auto find_command = [&commands](const std::string &name) {
      return std::find_if(
          commands.begin(), commands.end(),
          [&name](const auto &command) { return command.get_name() == name; });
    };
    const auto position = find_command("grip_stroke/position");
    const auto effort = find_command("grip_stroke/effort");
    const auto identity =
        find_command("grip_stroke/conventional_command_sequence");
    ASSERT_NE(position, commands.end());
    ASSERT_NE(effort, commands.end());
    ASSERT_NE(identity, commands.end());

    const auto before = server.function_count(16);
    ASSERT_TRUE(position->set_value(0.020));
    ASSERT_TRUE(effort->set_value(large ? 39.0 : 19.0));
    ASSERT_TRUE(identity->set_value(1.0));
    ASSERT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
              hardware_interface::return_type::OK);
    for (int i = 0; i < 25; ++i) {
      ASSERT_EQ(
          system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
          hardware_interface::return_type::OK);
      std::this_thread::sleep_for(1ms);
    }
    EXPECT_EQ(server.function_count(16), before);

    // A negative effort is invalid too, and must not resurrect the discarded
    // position on a later periodic write.
    ASSERT_TRUE(effort->set_value(-1.0));
    ASSERT_TRUE(identity->set_value(2.0));
    ASSERT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
              hardware_interface::return_type::OK);
    for (int i = 0; i < 25; ++i) {
      ASSERT_EQ(
          system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
          hardware_interface::return_type::OK);
      std::this_thread::sleep_for(1ms);
    }
    EXPECT_EQ(server.function_count(16), before);

    const double minimum = large ? 40.0 : 20.0;
    ASSERT_TRUE(effort->set_value(minimum));
    ASSERT_TRUE(identity->set_value(3.0));
    ASSERT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
              hardware_interface::return_type::OK);
    const auto deadline = std::chrono::steady_clock::now() + 1500ms;
    while (server.function_count(16) == before &&
           std::chrono::steady_clock::now() < deadline) {
      ASSERT_EQ(
          system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
          hardware_interface::return_type::OK);
      std::this_thread::sleep_for(1ms);
    }
    EXPECT_GT(server.function_count(16), before);
    EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State{}),
              hardware_interface::CallbackReturn::SUCCESS);
  }
}

TEST(OnRobotGripperSystem, RejectsMissingPositionInterface) {
  auto info = valid_info();
  info.joints.front().command_interfaces.clear();
  info.joints.front().command_interfaces.push_back(
      interface_info(hardware_interface::HW_IF_EFFORT));

  onrobot_gripper_hardware::OnRobotGripperSystem system;
  EXPECT_EQ(system.on_init(component_params(info)),
            hardware_interface::CallbackReturn::ERROR);
}

TEST(OnRobotGripperSystem, AcceptsDocumentedTwoFingerSupplyPower) {
  auto info = valid_info();
  info.hardware_parameters["supply_power_w"] = "48";

  onrobot_gripper_hardware::OnRobotGripperSystem system;
  EXPECT_EQ(system.on_init(component_params(info)),
            hardware_interface::CallbackReturn::SUCCESS);
}

TEST(OnRobotGripperSystem, AcceptsZeroToPreserveTwoFingerSupplyPower) {
  auto info = valid_info();
  info.hardware_parameters["supply_power_w"] = "0";

  onrobot_gripper_hardware::OnRobotGripperSystem system;
  EXPECT_EQ(system.on_init(component_params(info)),
            hardware_interface::CallbackReturn::SUCCESS);
}

TEST(OnRobotGripperSystem, RejectsInvalidTwoFingerSupplyPower) {
  for (const auto *value : {"13", "49"}) {
    auto info = valid_info();
    info.hardware_parameters["supply_power_w"] = value;

    onrobot_gripper_hardware::OnRobotGripperSystem system;
    EXPECT_EQ(system.on_init(component_params(info)),
              hardware_interface::CallbackReturn::ERROR);
  }
}

TEST(OnRobotGripperSystem, RejectsUnknownModelBeforeConnecting) {
  auto info = valid_info();
  info.hardware_parameters["model"] = "rg2";

  onrobot_gripper_hardware::OnRobotGripperSystem system;
  EXPECT_EQ(system.on_init(component_params(info)),
            hardware_interface::CallbackReturn::ERROR);
}

TEST(OnRobotGripperSystem, RejectsInvalidStopEventValues) {
  ModbusTcpTestServer server;
  auto info = valid_info();
  info.hardware_parameters["port"] = std::to_string(server.port());
  onrobot_gripper_hardware::OnRobotGripperSystem system;
  ASSERT_EQ(system.on_init(component_params(info)),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);

  auto commands = export_commands(system);
  const auto stop =
      std::find_if(commands.begin(), commands.end(), [](const auto &command) {
        return command.get_name() == "grip_stroke/stop_command_sequence";
      });
  ASSERT_NE(stop, commands.end());
  for (const auto invalid :
       {-1.0, 0.0, 0.5, std::numeric_limits<double>::infinity(),
        9007199254740992.0}) {
    ASSERT_TRUE(stop->set_value(invalid));
    EXPECT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
              hardware_interface::return_type::ERROR);
  }
  EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);
}

TEST(OnRobotRgSystem, RejectsInvalidStopEventValues) {
  ModbusTcpTestServer server(0x0020);
  server.set_register(0x010B, 730);
  server.set_register(0x0113, 730);
  server.set_register(0x0109, 200);
  server.set_register(0x0114, 0);
  server.set_register(0x0115, 0);

  auto info = valid_rg_info();
  info.hardware_parameters["port"] = std::to_string(server.port());
  onrobot_gripper_hardware::OnRobotRgSystem system;
  ASSERT_EQ(system.on_init(component_params(info)),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);

  auto commands = export_commands(system);
  const auto stop =
      std::find_if(commands.begin(), commands.end(), [](const auto &command) {
        return command.get_name() == "grip_stroke/stop_command_sequence";
      });
  ASSERT_NE(stop, commands.end());
  for (const auto invalid :
       {-1.0, 0.0, 0.5, std::numeric_limits<double>::infinity(),
        9007199254740992.0}) {
    ASSERT_TRUE(stop->set_value(invalid));
    EXPECT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
              hardware_interface::return_type::ERROR);
  }
  EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);
}

TEST(OnRobotGripperSystem, ClearsJointMeasurementsWhenTwoFingerFaults) {
  ModbusTcpTestServer server;
  auto info = fault_observation_info(server.port());
  onrobot_gripper_hardware::OnRobotGripperSystem system;
  ASSERT_EQ(system.on_init(component_params(info)),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);

  const auto states = system.export_state_interfaces();
  auto find_state = [&states](const std::string &i_name)
      -> const hardware_interface::StateInterface * {
    const auto found = std::find_if(states.begin(), states.end(),
                                    [&i_name](const auto &i_interface) {
                                      return i_interface.get_name() == i_name;
                                    });
    return found == states.end() ? nullptr : &(*found);
  };
  const auto *position = find_state("grip_stroke/position");
  const auto *velocity = find_state("grip_stroke/velocity");
  const auto *taskValid = find_state("grip_stroke/task_position_valid");
  const auto *forceValid = find_state("grip_stroke/force_valid");
  const auto *faulted = find_state("grip_stroke/faulted");
  ASSERT_NE(position, nullptr);
  ASSERT_NE(velocity, nullptr);
  ASSERT_NE(taskValid, nullptr);
  ASSERT_NE(forceValid, nullptr);
  ASSERT_NE(faulted, nullptr);

  server.set_register(0x0100, 0x0018);
  ASSERT_TRUE([&] {
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (std::chrono::steady_clock::now() < deadline) {
      system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0));
      if (faulted->get_value() > 0.5) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }());
  system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0));
  EXPECT_DOUBLE_EQ(taskValid->get_value(), 0.0);
  EXPECT_DOUBLE_EQ(forceValid->get_value(), 0.0);
  EXPECT_DOUBLE_EQ(faulted->get_value(), 1.0);
  EXPECT_TRUE(std::isnan(position->get_value()));
  EXPECT_TRUE(std::isnan(velocity->get_value()));
  system.on_deactivate(rclcpp_lifecycle::State{});
}

TEST(OnRobotGripperSystem,
     MapsSignedMechanismVelocityThroughConfiguredFingerKinematics) {
  ModbusTcpTestServer server;
  // The test server reports raw mechanism position in 0.01 mm.  Use a
  // deliberately non-default geometry so direct mm/s-to-m/s scaling cannot
  // accidentally satisfy the assertion.
  server.set_register(0x0109, 500);
  auto info = velocity_observation_info(server.port());
  onrobot_gripper_hardware::OnRobotGripperSystem system;
  ASSERT_EQ(system.on_init(component_params(info)),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);

  const auto states = system.export_state_interfaces();
  const auto find_state = [&states](const std::string &i_name) {
    return std::find_if(states.begin(), states.end(),
                        [&i_name](const auto &i_interface) {
                          return i_interface.get_name() == i_name;
                        });
  };
  const auto mechanismVelocity = find_state("finger_stroke/measured_velocity");
  const auto fingerVelocity = find_state("finger_stroke/velocity");
  ASSERT_NE(mechanismVelocity, states.end());
  ASSERT_NE(fingerVelocity, states.end());

  const auto waitForVelocity = [&](uint16_t i_rawPosition, bool i_positive) {
    server.set_register(0x0109, i_rawPosition);
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (std::chrono::steady_clock::now() < deadline) {
      if (system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)) !=
          hardware_interface::return_type::OK) {
        return false;
      }
      const double rawVelocity = mechanismVelocity->get_value();
      const double jointVelocity = fingerVelocity->get_value();
      if (std::isfinite(rawVelocity) && std::isfinite(jointVelocity) &&
          ((i_positive && rawVelocity > 1e-4) ||
           (!i_positive && rawVelocity < -1e-4))) {
        EXPECT_TRUE(std::isfinite(jointVelocity));
        EXPECT_NEAR(jointVelocity, rawVelocity * 1000.0 * 0.02 / 40.0, 1e-9);
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  };

  ASSERT_TRUE(waitForVelocity(900, true));
  ASSERT_TRUE(waitForVelocity(500, false));
  ASSERT_EQ(system.on_deactivate(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);
}

TEST(OnRobotGripperSystem,
     OrdinaryRealtimeStopDoesNotRecoverLatchedDeviceFault) {
  ModbusTcpTestServer server;
  server.set_register(0x0109, 500);
  server.set_register(0x0115, 196);
  server.set_register(0x0116, 0);
  server.set_register(0x0117, 562);
  auto info = realtime_fault_info(server.port());
  onrobot_gripper_hardware::OnRobotGripperSystem system;
  ASSERT_EQ(system.on_init(component_params(info)),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);

  auto commands = export_commands(system);
  const auto find_command = [&commands](const std::string &i_name) {
    return std::find_if(commands.begin(), commands.end(),
                        [&i_name](const auto &i_interface) {
                          return i_interface.get_name() == i_name;
                        });
  };
  const auto mode = find_command("grip_stroke/realtime_mode");
  const auto taskPosition = find_command("grip_stroke/realtime_task_position");
  const auto taskVelocity = find_command("grip_stroke/realtime_task_velocity");
  const auto force = find_command("grip_stroke/realtime_force");
  const auto sequence = find_command("grip_stroke/realtime_command_sequence");
  const auto recovery =
      find_command("grip_stroke/fault_recovery_command_sequence");
  ASSERT_NE(mode, commands.end());
  ASSERT_NE(taskPosition, commands.end());
  ASSERT_NE(taskVelocity, commands.end());
  ASSERT_NE(force, commands.end());
  ASSERT_NE(sequence, commands.end());
  ASSERT_NE(recovery, commands.end());

  const auto states = system.export_state_interfaces();
  const auto find_state = [&states](const std::string &i_name) {
    return std::find_if(states.begin(), states.end(),
                        [&i_name](const auto &i_interface) {
                          return i_interface.get_name() == i_name;
                        });
  };
  const auto faulted = find_state("grip_stroke/faulted");
  const auto activeMode = find_state("grip_stroke/active_mode");
  const auto reconnects = find_state("grip_stroke/reconnects");
  ASSERT_NE(faulted, states.end());
  ASSERT_NE(activeMode, states.end());
  ASSERT_NE(reconnects, states.end());

  ASSERT_TRUE(mode->set_value(0.0));
  ASSERT_TRUE(taskPosition->set_value(0.02));
  ASSERT_TRUE(taskVelocity->set_value(0.01));
  ASSERT_TRUE(force->set_value(0.0));
  ASSERT_TRUE(sequence->set_value(1.0));
  ASSERT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
            hardware_interface::return_type::OK);
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  ASSERT_TRUE([&] {
    while (std::chrono::steady_clock::now() < deadline) {
      if (server.function_count(23) > 0) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }());

  const auto stopsBeforeFault = server.function_count(16);
  server.set_register(0x0100, 0x0008);
  ASSERT_TRUE([&] {
    const auto faultDeadline = std::chrono::steady_clock::now() + 500ms;
    while (std::chrono::steady_clock::now() < faultDeadline) {
      if (system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)) !=
          hardware_interface::return_type::OK) {
        return false;
      }
      if (faulted->get_value() > 0.5) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }());
  EXPECT_DOUBLE_EQ(activeMode->get_value(), 0.0);
  EXPECT_DOUBLE_EQ(reconnects->get_value(), 0.0);
  // The fault image is published before the worker's protective Stop is
  // necessarily visible at the peer. Wait for that expected fault-path Stop
  // before measuring whether the later ordinary Stop adds transport I/O.
  ASSERT_TRUE([&] {
    const auto stopDeadline = std::chrono::steady_clock::now() + 500ms;
    while (std::chrono::steady_clock::now() < stopDeadline) {
      if (server.function_count(16) > stopsBeforeFault) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }());
  const auto stopsAtFault = server.function_count(16);
  const auto realtimeAtFault = server.function_count(23);

  // Clearing the peer flag does not clear the session's latched fault.  An
  // ordinary RT Stop is deliberately not an implicit recovery request.
  server.set_register(0x0100, 0);
  ASSERT_TRUE(mode->set_value(-1.0));
  ASSERT_TRUE(sequence->set_value(2.0));
  ASSERT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
            hardware_interface::return_type::OK);
  for (int i = 0; i < 50; ++i) {
    ASSERT_EQ(system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
              hardware_interface::return_type::OK);
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_DOUBLE_EQ(faulted->get_value(), 1.0);
  EXPECT_DOUBLE_EQ(activeMode->get_value(), 0.0);
  EXPECT_DOUBLE_EQ(reconnects->get_value(), 0.0);
  EXPECT_EQ(server.function_count(16), stopsAtFault);
  EXPECT_EQ(server.function_count(23), realtimeAtFault);

  // Only the dedicated recovery sequence reconnects. It returns idle and
  // does not replay the old realtime request until a new sequence is sent.
  ASSERT_TRUE(recovery->set_value(1.0));
  ASSERT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
            hardware_interface::return_type::OK);
  ASSERT_TRUE([&] {
    const auto recoveryDeadline = std::chrono::steady_clock::now() + 500ms;
    while (std::chrono::steady_clock::now() < recoveryDeadline) {
      if (system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)) !=
          hardware_interface::return_type::OK) {
        return false;
      }
      if (faulted->get_value() < 0.5 && reconnects->get_value() >= 1.0) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }());
  EXPECT_DOUBLE_EQ(activeMode->get_value(), 0.0);
  EXPECT_EQ(server.function_count(23), realtimeAtFault);

  ASSERT_TRUE(mode->set_value(0.0));
  ASSERT_TRUE(taskPosition->set_value(0.02));
  ASSERT_TRUE(taskVelocity->set_value(0.01));
  ASSERT_TRUE(force->set_value(0.0));
  ASSERT_TRUE(sequence->set_value(3.0));
  const auto realtimeBeforeNewCommand = server.function_count(23);
  ASSERT_EQ(system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
            hardware_interface::return_type::OK);
  ASSERT_TRUE([&] {
    const auto motionDeadline = std::chrono::steady_clock::now() + 500ms;
    while (std::chrono::steady_clock::now() < motionDeadline) {
      if (system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)) !=
          hardware_interface::return_type::OK) {
        return false;
      }
      if (server.function_count(23) > realtimeBeforeNewCommand) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }());
  ASSERT_EQ(system.on_deactivate(rclcpp_lifecycle::State{}),
            hardware_interface::CallbackReturn::SUCCESS);
}

TEST(OnRobotRgSystem, Rg2RealtimeStopDoesNotRecoverLatchedTransportFault) {
  assert_rg_realtime_stop_does_not_recover("rg2", 0x0020, 110.0);
}

TEST(OnRobotRgSystem, Rg6RealtimeStopDoesNotRecoverLatchedTransportFault) {
  assert_rg_realtime_stop_does_not_recover("rg6", 0x0021, 160.0);
}

TEST(OnRobotGripperSystem,
     ExplicitRecoverySupersedesUnacknowledgedTwoFingerStop) {
  assert_recovery_supersedes_pending_stop(false, 0x00c0, 0.0);
}

TEST(OnRobotRgSystem, ExplicitRecoverySupersedesUnacknowledgedRg2Stop) {
  assert_recovery_supersedes_pending_stop(true, 0x0020, 110.0);
}

TEST(OnRobotGripperSystem,
     ExplicitRecoverySupersedesUnacknowledgedTwoFg14Stop) {
  assert_recovery_supersedes_pending_stop(false, 0x00c1, 0.0);
}

TEST(OnRobotRgSystem, ExplicitRecoverySupersedesUnacknowledgedRg6Stop) {
  assert_recovery_supersedes_pending_stop(true, 0x0021, 160.0);
}

TEST(OnRobotGripperSystem, RejectsMultipleJointsBeforeConnecting) {
  auto info = valid_info();
  info.joints.push_back(info.joints.front());

  onrobot_gripper_hardware::OnRobotGripperSystem system;
  EXPECT_EQ(system.on_init(component_params(info)),
            hardware_interface::CallbackReturn::ERROR);
}

TEST(OnRobotGripperSystem, PluginDescriptionIsDiscoverable) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");

  const auto classes = loader.getDeclaredClasses();
  EXPECT_NE(std::find(classes.begin(), classes.end(),
                      "onrobot_gripper_hardware/OnRobotGripperSystem"),
            classes.end());
  EXPECT_NE(std::find(classes.begin(), classes.end(),
                      "onrobot_gripper_hardware/OnRobotRgSystem"),
            classes.end());
  EXPECT_NE(std::find(classes.begin(), classes.end(),
                      "onrobot_gripper_hardware/OnRobotThreeFingerSystem"),
            classes.end());
  EXPECT_NE(
      std::find(classes.begin(), classes.end(),
                "onrobot_gripper_hardware/OnRobotParallelGripperFakeSystem"),
      classes.end());
}

TEST(OnRobotGripperSystem, RgPluginInitializesSemanticApertureInterfaces) {
  onrobot_gripper_hardware::OnRobotRgSystem system;
  ASSERT_EQ(system.on_init(component_params(valid_rg_info())),
            hardware_interface::CallbackReturn::SUCCESS);
  const auto states = system.export_state_interfaces();
  ASSERT_EQ(states.size(), 4u);
  EXPECT_EQ(states[0].get_name(), "grip_stroke/position");
  EXPECT_EQ(states[1].get_name(), "grip_stroke/velocity");
  EXPECT_EQ(states[2].get_name(), "finger_joint/position");
  EXPECT_EQ(states[3].get_name(), "finger_joint/velocity");

  const auto commands = export_commands(system);
  ASSERT_EQ(commands.size(), 5u);
  EXPECT_EQ(commands[0].get_name(), "grip_stroke/position");
  EXPECT_EQ(commands[1].get_name(), "grip_stroke/effort");
  EXPECT_EQ(commands[2].get_name(),
            "grip_stroke/fault_recovery_command_sequence");
  EXPECT_EQ(commands[3].get_name(), "grip_stroke/stop_command_sequence");
  EXPECT_EQ(commands[4].get_name(),
            "grip_stroke/conventional_command_sequence");
}

TEST(OnRobotGripperSystem, RgVisualJointUsesMeasuredMechanismAngle) {
  for (const bool rg6 : {false, true}) {
    SCOPED_TRACE(rg6 ? "RG6" : "RG2");
    ModbusTcpTestServer server(rg6 ? 0x0021 : 0x0020);
    server.set_register(0x0109, 358); // 0.358 rad, independent of aperture.
    server.set_register(0x010B, 396);
    server.set_register(0x0113, 396);
    server.set_register(0x0114, 0);
    server.set_register(0x0115, 0);

    auto info = valid_rg_info();
    info.hardware_parameters["port"] = std::to_string(server.port());
    info.hardware_parameters["model"] = rg6 ? "rg6" : "rg2";
    onrobot_gripper_hardware::OnRobotRgSystem system;
    ASSERT_EQ(system.on_init(component_params(info)),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}),
              hardware_interface::CallbackReturn::SUCCESS);

    auto states = system.export_state_interfaces();
    const auto finger =
        std::find_if(states.begin(), states.end(), [](auto &state) {
          return state.get_name() == "finger_joint/position";
        });
    ASSERT_NE(finger, states.end());
    ASSERT_EQ(system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
              hardware_interface::return_type::OK);
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    do {
      std::this_thread::sleep_for(5ms);
      ASSERT_EQ(
          system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
          hardware_interface::return_type::OK);
    } while ((!finger->get_optional<double>().has_value() ||
              !std::isfinite(*finger->get_optional<double>())) &&
             std::chrono::steady_clock::now() < deadline);

    ASSERT_TRUE(finger->get_optional<double>().has_value());
    EXPECT_NEAR(*finger->get_optional<double>(),
                0.358 + (rg6 ? std::asin(4.2 / 80.0) : std::asin(2.5 / 55.0)),
                1e-9);
    EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State{}),
              hardware_interface::CallbackReturn::SUCCESS);
  }
}

TEST(OnRobotGripperSystem, FakeTwoFingerForceApproachDoesNotInventContactOrForce) {
  for (const bool large : {false, true}) {
    SCOPED_TRACE(large ? "2FG14" : "2FG7");
    auto info = realtime_fault_info(502);
    info.hardware_parameters["model"] = large ? "2fg14" : "2fg7";
    info.hardware_parameters["fake_task_min_m"] = large ? "0.055" : "0.033";
    info.hardware_parameters["fake_task_max_m"] = large ? "0.105" : "0.071";
    info.hardware_parameters["finger_joint_upper_m"] = large ? "0.025" : "0.019";
    info.joints.front().state_interfaces.push_back(interface_info("grip_detected"));
    info.joints.front().state_interfaces.push_back(interface_info("realtime_force_control_available"));
    hardware_interface::ComponentInfo finger;
    finger.name = "finger_stroke";
    finger.state_interfaces.push_back(interface_info("position"));
    info.joints.push_back(finger);
    onrobot_gripper_hardware::OnRobotParallelGripperFakeSystem system;
    ASSERT_EQ(system.on_init(component_params(info)), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_activate({}), hardware_interface::CallbackReturn::SUCCESS);
    auto states = system.export_state_interfaces();
    auto commands = export_commands(system);
    const auto state = [&](const std::string &name) {
      for (const auto &s : states)
        if (s.get_name() == "grip_stroke/" + name) return s.get_optional<double>().value();
      ADD_FAILURE() << name;
      return std::numeric_limits<double>::quiet_NaN();
    };
    const auto command = [&](const std::string &name, double value) {
      for (auto &c : commands) if (c.get_name() == "grip_stroke/" + name) {
        EXPECT_TRUE(c.set_value(value)); return;
      }
      ADD_FAILURE() << name;
    };
    const auto step = [&] { return system.write({}, rclcpp::Duration::from_seconds(0.02)); };
    EXPECT_DOUBLE_EQ(state("realtime_force_control_available"), 1.0);
    const double opened = state("position");
    command("realtime_mode", 2);
    command("realtime_force", 50.0);
    command("realtime_task_position", opened - 0.01);
    command("realtime_task_velocity", 0.01);
    command("realtime_command_sequence", 1);
    EXPECT_EQ(step(), hardware_interface::return_type::OK);
    EXPECT_NEAR(state("position"), opened - 0.0002, 1e-12);
    EXPECT_DOUBLE_EQ(state("force_valid"), 0.0);
    EXPECT_TRUE(std::isnan(state("effort")));
    EXPECT_DOUBLE_EQ(state("grip_detected"), 0.0);
    command("realtime_mode", 3);
    command("realtime_task_velocity", -0.01);
    command("realtime_command_sequence", 2);
    EXPECT_EQ(step(), hardware_interface::return_type::OK);
    EXPECT_NEAR(state("position"), opened - 0.0004, 1e-12);
    command("stop_command_sequence", 3);
    EXPECT_EQ(step(), hardware_interface::return_type::OK);
    for (int n = 0; n < 10; ++n) EXPECT_EQ(step(), hardware_interface::return_type::OK);
    EXPECT_NEAR(state("position"), opened - 0.0004, 1e-12);
    command("realtime_force", -50.0);
    command("realtime_command_sequence", 4);
    EXPECT_EQ(step(), hardware_interface::return_type::ERROR);
    EXPECT_NEAR(state("position"), opened - 0.0004, 1e-12);

    // An explicit negative conventional target must not be reinterpreted as
    // the model default.  The real Tool API rejects it before I/O, so the
    // fake backend must reject it before applying the pending position too.
    command("position", opened - 0.005);
    command("effort", -1.0);
    command("conventional_command_sequence", 5);
    EXPECT_EQ(step(), hardware_interface::return_type::OK);
    const auto event = std::find_if(commands.begin(), commands.end(), [](const auto &c) {
      return c.get_name() == "grip_stroke/conventional_command_sequence";
    });
    ASSERT_NE(event, commands.end());
    EXPECT_DOUBLE_EQ(event->get_optional<double>().value(), -5.0);
    EXPECT_NEAR(state("position"), opened - 0.0004, 1e-12);
  }
}

TEST(OnRobotGripperSystem, FakeRgConventionalAndRealtimeCloseAtRubberContact) {
  for (const bool rg6 : {false, true}) {
    SCOPED_TRACE(rg6 ? "RG6" : "RG2");
    auto info =
        realtime_rg_fault_info(502, rg6 ? "rg6" : "rg2", rg6 ? 160 : 110);
    info.hardware_parameters["fake_task_min_m"] = "0";
    info.hardware_parameters["fake_task_max_m"] = rg6 ? "0.160" : "0.110";
    info.hardware_parameters["visual_finger_joint_upper_rad"] =
        rg6 ? "1.2978487644385668" : "1.3136012652574625";
    for (const auto *name :
         {"measured_angular_position", "measured_angular_velocity"}) {
      info.joints.back().state_interfaces.push_back(interface_info(name));
    }
    onrobot_gripper_hardware::OnRobotParallelGripperFakeSystem system;
    ASSERT_EQ(system.on_init(component_params(info)),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}),
              hardware_interface::CallbackReturn::SUCCESS);
    auto states = system.export_state_interfaces();
    auto commands = export_commands(system);
    const auto state = [&](const std::string &name) {
      for (const auto &s : states) {
        if (s.get_name() == name)
          return s.get_optional<double>().value();
      }
      ADD_FAILURE() << "Missing state " << name;
      return std::numeric_limits<double>::quiet_NaN();
    };
    const auto command = [&](const std::string &name, double value) {
      for (auto &c : commands) {
        if (c.get_name() == name) {
          EXPECT_TRUE(c.set_value(value));
          return;
        }
      }
      ADD_FAILURE() << "Missing command " << name;
    };
    const auto step = [&]() {
      EXPECT_EQ(
          system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.02)),
          hardware_interface::return_type::OK);
    };
    const onrobot_gripper_hardware::RgCadKinematics cad(rg6);
    EXPECT_NEAR(state("grip_stroke/position"), rg6 ? 0.150 : 0.1011, 1e-12);
    for (const double width : {0.0, 0.005, 0.01, 0.02}) {
      command("grip_stroke/position", width);
      step();
      EXPECT_NEAR(cad.fingerAngleToWidth(state("finger_joint/position")), width,
                  1e-12);
      EXPECT_NEAR(state("finger_joint/measured_angular_position"),
                  state("finger_joint/position") - cad.cadZeroPhase(), 1e-12);
    }
    command("grip_stroke/realtime_mode", 0);
    command("grip_stroke/realtime_task_position", 0.01);
    command("grip_stroke/realtime_command_sequence", 1);
    step();
    EXPECT_NEAR(cad.fingerAngleToWidth(state("finger_joint/position")), 0.01,
                1e-12);
    command("grip_stroke/realtime_mode", 1);
    command("grip_stroke/realtime_mechanism_angular_velocity", -0.5);
    command("grip_stroke/realtime_command_sequence", 2);
    for (int i = 0; i < 100; ++i)
      step();
    EXPECT_NEAR(cad.fingerAngleToWidth(state("finger_joint/position")), 0.0,
                1e-12);
    EXPECT_DOUBLE_EQ(state("grip_stroke/position"), 0.0);
    command("grip_stroke/realtime_mechanism_angular_velocity", 0.5);
    step();
    EXPECT_GT(state("grip_stroke/position"), 0.0);
  }
}

TEST(OnRobotGripperSystem,
     Rg2StandardCadFollowsMeasuredGapWithoutChangingRawState) {
  for (const double aperture : {-0.1, 0.0, 5.0, 10.0, 20.0, 100.0}) {
    SCOPED_TRACE(aperture);
    ModbusTcpTestServer server(0x0020);
    server.set_register(0x0109, 358);
    server.set_register(0x010B, 396);
    server.set_register(
        0x0113, static_cast<uint16_t>(static_cast<int16_t>(aperture * 10)));
    auto info = valid_rg_info();
    info.hardware_parameters["port"] = std::to_string(server.port());
    info.hardware_parameters["visual_gap_from_aperture"] = "true";
    info.joints.back().state_interfaces.push_back(
        interface_info("measured_angular_position"));
    onrobot_gripper_hardware::OnRobotRgSystem system;
    ASSERT_EQ(system.on_init(component_params(info)),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}),
              hardware_interface::CallbackReturn::SUCCESS);
    auto states = system.export_state_interfaces();
    const auto get = [&](const std::string &name) {
      for (const auto &s : states)
        if (s.get_name() == name)
          return s.get_optional<double>().value();
      ADD_FAILURE() << name;
      return std::numeric_limits<double>::quiet_NaN();
    };
    const onrobot_gripper_hardware::RgCadKinematics cad(false);
    EXPECT_NEAR(get("grip_stroke/position"), aperture / 1000, 1e-12);
    EXPECT_NEAR(get("finger_joint/measured_angular_position"), 0.358, 1e-12);
    EXPECT_NEAR(cad.fingerAngleToWidth(get("finger_joint/position")),
                std::max(0.0, aperture / 1000), 1e-12);
  }
}

TEST(OnRobotGripperSystem,
     ThreeFingerPluginInitializesNativeDiameterAndAngleInterfaces) {
  onrobot_gripper_hardware::OnRobotThreeFingerSystem system;
  ASSERT_EQ(system.on_init(component_params(valid_three_finger_info())),
            hardware_interface::CallbackReturn::SUCCESS);

  const auto states = system.export_state_interfaces();
  ASSERT_EQ(states.size(), 6u);
  EXPECT_EQ(states[0].get_name(), "grip_diameter/position");
  EXPECT_EQ(states[1].get_name(), "grip_diameter/velocity");
  EXPECT_EQ(states[2].get_name(), "finger_angle/position");
  EXPECT_EQ(states[3].get_name(), "finger_angle/velocity");
  EXPECT_EQ(states[4].get_name(), "grip_diameter/minimum_external_diameter");
  EXPECT_EQ(states[5].get_name(), "grip_diameter/maximum_external_diameter");

  const auto commands = export_commands(system);
  ASSERT_EQ(commands.size(), 2u);
  EXPECT_EQ(commands[0].get_name(), "grip_diameter/position");
  EXPECT_EQ(commands[1].get_name(), "grip_diameter/effort");
}

} // namespace

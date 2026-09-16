// Exercise the installed physical callbacks with deterministic numeric handoffs.
// Only this test executable interposes snapshot/command admission; the runtime
// library has no test hook. Actual mutex contention is covered by the API tests.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <new>
#include <stdexcept>
#include <string>
#include <utility>

#include <gtest/gtest.h>
#include <onrobot_gripper_msgs/diagnostic_interfaces.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "support/modbus_tcp_test_server.hpp"
#include "onrobot_gripper_hardware/onrobot_gripper_system.hpp"
#include "onrobot_gripper_hardware/onrobot_rg_system.hpp"

namespace {
using namespace std::chrono_literals;
hardware_interface::InterfaceInfo interface_info(const std::string &name) {
  hardware_interface::InterfaceInfo result;
  result.name = name;
  return result;
}
hardware_interface::HardwareComponentInterfaceParams
component_params(hardware_interface::HardwareInfo info) {
  hardware_interface::HardwareComponentInterfaceParams result;
  result.hardware_info = std::move(info);
  return result;
}
hardware_interface::HardwareInfo handoff_info(bool rg, const std::string &model,
                                             uint16_t port) {
  hardware_interface::HardwareInfo info;
  info.name = "onrobot_" + model;
  info.type = "system";
  info.hardware_parameters = {
      {"model", model}, {"transport", "tcp"},
      {"host", "127.0.0.1"}, {"port", std::to_string(port)}, {"slave_id", "65"}};
  if (rg) {
    info.hardware_parameters["safe_width_min_mm"] = "0";
    info.hardware_parameters["safe_width_max_mm"] = model == "rg6" ? "160" : "110";
  } else {
    info.hardware_parameters["raw_linear_min_mm"] = "5";
    info.hardware_parameters["raw_linear_max_mm"] = "45";
    info.hardware_parameters["finger_joint_upper_m"] = "0.02";
  }
  hardware_interface::ComponentInfo task;
  task.name = "grip_stroke";
  for (const char *name : {"position", "effort", "fault_recovery_command_sequence",
                           "stop_command_sequence",
                           "conventional_command_sequence"})
    task.command_interfaces.push_back(interface_info(name));
  for (const char *name : {"position", "velocity", "effort", "force_valid",
                           "task_position_valid", "faulted",
                           "firmware_qualification", "sample_sequence",
                           "sample_age", "requested_command_sequence",
                           "applied_command_sequence"})
    task.state_interfaces.push_back(interface_info(name));
  for (const char *name : onrobot_gripper_msgs::kDiagnosticInterfaceNames)
    task.state_interfaces.push_back(interface_info(name));
  hardware_interface::ComponentInfo physical;
  physical.name = rg ? "finger_joint" : "finger_stroke";
  physical.state_interfaces.push_back(interface_info("position"));
  physical.state_interfaces.push_back(interface_info("velocity"));
  physical.state_interfaces.push_back(interface_info(rg ? "measured_angular_position" : "measured_position"));
  physical.state_interfaces.push_back(interface_info(rg ? "measured_angular_velocity" : "measured_velocity"));
  physical.state_interfaces.push_back(interface_info("position_valid"));
  physical.state_interfaces.push_back(interface_info("velocity_valid"));
  info.joints = {task, physical};
  return info;
}
}
namespace handoff_probe {
thread_local bool count_enabled = false;
thread_local std::size_t allocations = 0;
bool deliver = false;
bool allocation_control = false;
std::size_t calls = 0;
onrobot::ParallelGripperState state;
onrobot::ParallelGripperIdentity identity;
bool command_busy = false;
unsigned stop_busy_count = 0;
unsigned command_calls = 0;
unsigned stop_calls = 0;
onrobot::ParallelGripCommand last_command;
void *allocate(std::size_t size, std::size_t alignment = alignof(std::max_align_t)) {
  if (count_enabled) ++allocations;
  void *result = nullptr;
  if (alignment <= alignof(std::max_align_t)) result = std::malloc(size ? size : 1);
  else if (posix_memalign(&result, alignment, size ? size : 1) != 0) result = nullptr;
  if (!result) throw std::bad_alloc();
  return result;
}
}
void *operator new(std::size_t size) { return handoff_probe::allocate(size); }
void *operator new[](std::size_t size) { return handoff_probe::allocate(size); }
void *operator new(std::size_t size, std::align_val_t a) {
  return handoff_probe::allocate(size, static_cast<std::size_t>(a));
}
void *operator new[](std::size_t size, std::align_val_t a) { return ::operator new(size, a); }
void operator delete(void *p) noexcept { std::free(p); }
void operator delete[](void *p) noexcept { std::free(p); }
void operator delete(void *p, std::size_t) noexcept { std::free(p); }
void operator delete[](void *p, std::size_t) noexcept { std::free(p); }
void operator delete(void *p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void *p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void *p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void *p, std::size_t, std::align_val_t) noexcept { std::free(p); }

bool onrobot::ParallelGripperSession::trySnapshot(ParallelGripperState &out) const {
  ++handoff_probe::calls;
  if (handoff_probe::allocation_control) {
    void *memory = ::operator new(65536);
    ::operator delete(memory);
  }
  if (!handoff_probe::deliver) return false;
  out = handoff_probe::state;
  return true;
}

bool onrobot::ParallelGripperSession::trySnapshot(
    ParallelGripperState &out, ParallelGripperIdentity &identity) const {
  if (!trySnapshot(out)) return false;
  identity = handoff_probe::identity;
  return true;
}

onrobot::CommandAdmission onrobot::ParallelGripperSession::tryCommand(
    const ParallelGripCommand &command, uint64_t &sequence) {
  ++handoff_probe::command_calls;
  handoff_probe::last_command = command;
  if (handoff_probe::command_busy) return CommandAdmission::Busy;
  sequence = 1235;
  return CommandAdmission::Accepted;
}

onrobot::CommandAdmission onrobot::ParallelGripperSession::tryStop(uint64_t &sequence) {
  ++handoff_probe::stop_calls;
  if (handoff_probe::stop_busy_count > 0) {
    --handoff_probe::stop_busy_count;
    return CommandAdmission::Busy;
  }
  sequence = 1234;
  return CommandAdmission::Accepted;
}

namespace {
template <typename System>
void exerciseRead(bool rg, const std::string &model, uint16_t product,
                  onrobot::Model apiModel) {
  ModbusTcpTestServer server(product);
  server.set_register(0x0101, 200);
  server.set_register(0x0109, 500);
  server.set_register(0x0113, 680);
  if (rg) server.set_register(0x0605, 10);
  auto info = handoff_info(rg, model, server.port());
  System system;
  ASSERT_EQ(system.on_init(component_params(info)), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  const auto states = system.export_state_interfaces();
  const auto stateValue = [&](const char *name) -> double {
    const auto found = std::find_if(states.begin(), states.end(), [&](const auto &entry) {
      return entry.get_name() == name;
    });
    if (found == states.end()) throw std::runtime_error(std::string("missing interface ") + name);
    return found->get_optional().value();
  };
  const rclcpp::Time time{};
  const auto period = rclcpp::Duration::from_seconds(0.01);
  const auto readCounted = [&](int iterations, bool expect_allocation = false) {
    handoff_probe::allocations = 0;
    handoff_probe::calls = 0;
    handoff_probe::count_enabled = true;
    hardware_interface::return_type result = hardware_interface::return_type::OK;
    for (int i = 0; i < iterations; ++i) result = system.read(time, period);
    handoff_probe::count_enabled = false;
    EXPECT_EQ(result, hardware_interface::return_type::OK);
    EXPECT_EQ(handoff_probe::calls, static_cast<std::size_t>(iterations));
    if (expect_allocation) EXPECT_GT(handoff_probe::allocations, 0u);
    else EXPECT_EQ(handoff_probe::allocations, 0u);
  };

  // First missed handoff must retain activation's actual lifecycle seed.
  handoff_probe::deliver = false;
  readCounted(1);
  EXPECT_TRUE(std::isfinite(stateValue("grip_stroke/position")));
  EXPECT_GT(stateValue("grip_stroke/sample_sequence"), 0.0);

  auto &sample = handoff_probe::state;
  sample = {};
  sample.model = apiModel;
  sample.session_state = onrobot::SessionState::Active;
  sample.active_mode = onrobot::ParallelControlMode::Conventional;
  sample.firmware_qualification = onrobot::FirmwareQualification::Qualified;
  sample.task_aperture_valid = true;
  sample.task_aperture_mm = 25.0;
  sample.task_velocity_valid = true;
  sample.task_velocity_mm_s = 4.0;
  sample.mechanism_linear_position_valid = !rg;
  sample.mechanism_linear_position_mm = 9.0;
  sample.mechanism_linear_velocity_valid = !rg;
  sample.mechanism_linear_velocity_mm_s = 2.0;
  sample.mechanism_angular_position_valid = rg;
  sample.mechanism_angular_position_rad = 0.5;
  sample.mechanism_angular_velocity_valid = rg;
  sample.mechanism_angular_velocity_rad_s = 0.1;
  sample.sample_sequence = 1001;
  sample.requested_command_sequence = 777;
  sample.applied_command_sequence = 776;
  sample.received_at = std::chrono::steady_clock::now() - 2s;
  handoff_probe::identity = {};
  handoff_probe::identity.valid = true;
  handoff_probe::identity.firmware_major = 1;
  handoff_probe::identity.firmware_build = 33;
  handoff_probe::identity.firmware_source_valid = !rg;
  handoff_probe::identity.firmware_git_hash_words[0] = 0xffffffff;
  handoff_probe::deliver = true;
  readCounted(1);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/position"), 0.025);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/velocity"), 0.004);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/sample_sequence"), 1001.0);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/firmware_qualification"), 1.0);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/diagnostic_identity_valid"), 1.0);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/diagnostic_firmware_build"), 33.0);
  if (!rg) {
    EXPECT_DOUBLE_EQ(stateValue("grip_stroke/diagnostic_firmware_git_word_0"),
                     4294967295.0);
  } else {
    EXPECT_TRUE(std::isnan(stateValue("grip_stroke/diagnostic_firmware_git_word_0")));
  }
  const double age0 = stateValue("grip_stroke/sample_age");
  EXPECT_GE(age0, 2.0);
  const double seededPhysicalPose = stateValue(
      rg ? "finger_joint/position" : "finger_stroke/position");

  // Preserve finite signed 2FG force, even an implausible spike: clipping it
  // here would hide the very diagnostic evidence needed to find its source.
  sample.force_valid = !rg; // RG session does not label command-derived force measured.
  sample.force_n = -270.0;
  readCounted(1);
  if (!rg) {
    EXPECT_DOUBLE_EQ(stateValue("grip_stroke/effort"), -270.0);
    EXPECT_DOUBLE_EQ(stateValue("grip_stroke/force_valid"), 1.0);
  } else {
    EXPECT_TRUE(std::isnan(stateValue("grip_stroke/effort")));
    EXPECT_DOUBLE_EQ(stateValue("grip_stroke/force_valid"), 0.0);
  }

  // Mutate the undelivered source to prove reads retain their own cached state.
  handoff_probe::deliver = false;
  sample.task_aperture_mm = 70.0;
  sample.sample_sequence = 2002;
  handoff_probe::identity.firmware_build = 34;
  sample.received_at = std::chrono::steady_clock::now();
  readCounted(1000);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/position"), 0.025);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/velocity"), 0.004);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/sample_sequence"), 1001.0);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/requested_command_sequence"), 777.0);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/applied_command_sequence"), 776.0);
  EXPECT_GT(stateValue("grip_stroke/sample_age"), age0);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/diagnostic_firmware_build"), 33.0);

  // An allocation injected inside the actual read call must be observed.
  handoff_probe::allocation_control = true;
  readCounted(1, true);
  handoff_probe::allocation_control = false;

  if (rg) {
    // A malformed velocity marked valid must not suppress the finite-position
    // derivative fallback. Same aperture and advancing time means zero speed.
    handoff_probe::deliver = true;
    sample.task_aperture_mm = 25.0;
    sample.sample_sequence = 1002;
    sample.received_at += 20ms;
    sample.task_velocity_valid = true;
    sample.task_velocity_mm_s = std::numeric_limits<double>::quiet_NaN();
    readCounted(1);
    EXPECT_DOUBLE_EQ(stateValue("grip_stroke/velocity"), 0.0);
    sample.task_velocity_mm_s = 4.0;
  }

  // A position field can be lost independently of the task aperture. Retain
  // the visual joint only, and never publish an old raw measurement as live.
  handoff_probe::deliver = true;
  sample.task_aperture_mm = 25.0;
  sample.sample_sequence = 1002;
  sample.mechanism_linear_position_valid = false;
  sample.mechanism_angular_position_valid = false;
  // The old numeric value may remain in the image after its flag is cleared.
  readCounted(1);
  EXPECT_TRUE(std::isfinite(stateValue(
      rg ? "finger_joint/position" : "finger_stroke/position")));
  EXPECT_TRUE(std::isnan(stateValue(rg ? "finger_joint/measured_angular_position"
                                     : "finger_stroke/measured_position")));

  // Numeric validity must agree with the exported flag, even if a malformed
  // producer sets the flag while supplying NaN/Inf.
  sample.task_aperture_valid = true;
  sample.task_aperture_mm = std::numeric_limits<double>::quiet_NaN();
  sample.force_valid = true;
  sample.force_n = std::numeric_limits<double>::infinity();
  sample.mechanism_linear_position_valid = !rg;
  sample.mechanism_angular_position_valid = rg;
  sample.mechanism_linear_position_mm = std::numeric_limits<double>::quiet_NaN();
  sample.mechanism_angular_position_rad = std::numeric_limits<double>::quiet_NaN();
  readCounted(1);
  EXPECT_TRUE(std::isnan(stateValue("grip_stroke/position")))
      << "A held visualization pose must not enter standard action feedback";
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/task_position_valid"), 0.0);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/force_valid"), 0.0);
  EXPECT_DOUBLE_EQ(stateValue(rg ? "finger_joint/position_valid"
                                : "finger_stroke/position_valid"), 0.0);
  EXPECT_TRUE(std::isnan(stateValue("grip_stroke/effort")));

  // A non-fault sample can lose its position fields while the worker is
  // reconnecting. Keep the last finite pose for TF, but expose the sample as
  // invalid and do not retain velocity or force as if it were current.
  sample.session_state = onrobot::SessionState::Active;
  sample.task_aperture_valid = false;
  sample.task_velocity_valid = false;
  sample.mechanism_linear_position_valid = false;
  sample.mechanism_angular_position_valid = false;
  sample.mechanism_linear_velocity_valid = false;
  sample.mechanism_angular_velocity_valid = false;
  sample.sample_sequence = 1002;
  sample.received_at = std::chrono::steady_clock::now();
  handoff_probe::deliver = true;
  readCounted(1);
  EXPECT_TRUE(std::isnan(stateValue("grip_stroke/position")));
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/task_position_valid"), 0.0);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/faulted"), 0.0);
  EXPECT_TRUE(std::isfinite(stateValue(
      rg ? "finger_joint/position" : "finger_stroke/position")));
  EXPECT_TRUE(std::isnan(stateValue("grip_stroke/velocity")));

  sample.session_state = onrobot::SessionState::Faulted;
  sample.task_aperture_valid = false;
  sample.task_velocity_valid = false;
  sample.mechanism_linear_position_valid = false;
  sample.mechanism_angular_position_valid = false;
  sample.mechanism_linear_velocity_valid = false;
  sample.mechanism_angular_velocity_valid = false;
  sample.last_error_code = onrobot::ErrorCode::DeviceFault;
  sample.sample_sequence = 1001; // fault publication need not be a new sample
  sample.received_at = std::chrono::steady_clock::now() - 3s;
  handoff_probe::deliver = true;
  readCounted(1);
  EXPECT_TRUE(std::isnan(stateValue("grip_stroke/position")));
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/faulted"), 1.0);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/diagnostic_identity_valid"), 0.0);
  EXPECT_TRUE(std::isnan(stateValue("grip_stroke/diagnostic_firmware_build")));
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/task_position_valid"), 0.0);
  EXPECT_TRUE(std::isnan(stateValue(
      rg ? "finger_joint/position" : "finger_stroke/position")))
      << "An explicit fault must invalidate the visualization cache too";
  const double fault_age = stateValue("grip_stroke/sample_age");
  handoff_probe::deliver = false;
  sample.session_state = onrobot::SessionState::Active;
  sample.task_aperture_valid = true;
  readCounted(1000);
  EXPECT_TRUE(std::isnan(stateValue("grip_stroke/position")));
  EXPECT_TRUE(std::isnan(stateValue("grip_stroke/velocity")));
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/faulted"), 1.0);
  EXPECT_GT(stateValue("grip_stroke/sample_age"), fault_age);

  sample.task_aperture_mm = 30.0;
  sample.task_velocity_valid = false;
  sample.task_velocity_mm_s = 0.0;
  sample.sample_sequence = 1002;
  sample.received_at = std::chrono::steady_clock::now();
  handoff_probe::deliver = true;
  readCounted(1);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/position"), 0.03);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/sample_sequence"), 1002.0);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/faulted"), 0.0);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/diagnostic_identity_valid"), 1.0);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/diagnostic_firmware_build"), 34.0);
  EXPECT_TRUE(std::isnan(stateValue("grip_stroke/velocity")))
      << "First valid sample after fault must not derive velocity across the invalid gap";
  if (rg) {
    // RG can derive a new CAD pose from the recovered aperture. That is not
    // the old held pose, and the raw angular measurement must stay invalid.
    EXPECT_TRUE(std::isfinite(stateValue("finger_joint/position")));
    EXPECT_NE(stateValue("finger_joint/position"), seededPhysicalPose);
    EXPECT_TRUE(std::isnan(stateValue("finger_joint/measured_angular_position")));
    EXPECT_TRUE(std::isnan(stateValue("finger_joint/velocity")));
  } else {
    EXPECT_TRUE(std::isnan(stateValue("finger_stroke/position")))
        << "Task-only recovery must not resurrect a pre-fault physical pose";
  }

  sample.mechanism_linear_position_valid = !rg;
  sample.mechanism_angular_position_valid = rg;
  sample.mechanism_linear_position_mm = 12.0;
  sample.mechanism_angular_position_rad = 0.6;
  ++sample.sample_sequence;
  sample.received_at += 20ms;
  readCounted(1);
  EXPECT_TRUE(std::isfinite(stateValue(
      rg ? "finger_joint/position" : "finger_stroke/position")));
  EXPECT_DOUBLE_EQ(stateValue(rg ? "finger_joint/position_valid"
                                : "finger_stroke/position_valid"), 1.0);

  ASSERT_EQ(system.on_deactivate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  handoff_probe::deliver = false;
  readCounted(1);
  EXPECT_TRUE(std::isnan(stateValue("grip_stroke/position")));
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/sample_sequence"), 0.0);
  EXPECT_NE(stateValue("grip_stroke/diagnostic_identity_valid"), 1.0);
  EXPECT_DOUBLE_EQ(stateValue("grip_stroke/sample_age"), 0.0);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  readCounted(1);
  EXPECT_TRUE(std::isfinite(stateValue("grip_stroke/position")));
  EXPECT_NE(stateValue("grip_stroke/sample_sequence"), 1002.0);
  EXPECT_NE(stateValue("grip_stroke/position"), 0.03);
  ASSERT_EQ(system.on_deactivate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
}
TEST(PhysicalReadHandoff, TwoFg7CacheAgeFaultAndLifecycle) {
  exerciseRead<onrobot_gripper_hardware::OnRobotGripperSystem>(
      false, "2fg7", 0x00c0, onrobot::Model::TwoFG7);
}
TEST(PhysicalReadHandoff, TwoFg14CacheAgeFaultAndLifecycle) {
  exerciseRead<onrobot_gripper_hardware::OnRobotGripperSystem>(
      false, "2fg14", 0x00c1, onrobot::Model::TwoFG14);
}
TEST(PhysicalReadHandoff, Rg2CacheAgeFaultAndLifecycle) {
  exerciseRead<onrobot_gripper_hardware::OnRobotRgSystem>(
      true, "rg2", 0x0020, onrobot::Model::RG2);
}
TEST(PhysicalReadHandoff, Rg6CacheAgeFaultAndLifecycle) {
  exerciseRead<onrobot_gripper_hardware::OnRobotRgSystem>(
      true, "rg6", 0x0021, onrobot::Model::RG6);
}

TEST(PhysicalCommandHandoff, StopBusyRetainsFenceAndNeverRetriesCanceledGrip) {
  for (const std::string model : {"2fg7", "2fg14", "rg2", "rg6"}) {
    SCOPED_TRACE(model);
    const bool rg = model == "rg2" || model == "rg6";
    const bool large = model == "rg6" || model == "2fg14";
    ModbusTcpTestServer server(rg ? (large ? 0x21 : 0x20) : (large ? 0xc1 : 0xc0));
    std::unique_ptr<hardware_interface::SystemInterface> system;
    if (rg) system = std::make_unique<onrobot_gripper_hardware::OnRobotRgSystem>();
    else system = std::make_unique<onrobot_gripper_hardware::OnRobotGripperSystem>();
    ASSERT_EQ(system->on_init(component_params(handoff_info(rg, model, server.port()))),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    const auto commands = system->on_export_command_interfaces();
    const auto handle = [&](const std::string &name) {
      const auto found = std::find_if(commands.begin(), commands.end(), [&](const auto &entry) {
        return entry->get_interface_name() == name;
      });
      if (found == commands.end()) throw std::runtime_error("missing command " + name);
      return *found;
    };
    const auto position = handle("position");
    const auto effort = handle("effort");
    const auto event = handle("conventional_command_sequence");
    const auto stop = handle("stop_command_sequence");
    auto &sample = handoff_probe::state;
    sample = {};
    sample.session_state = onrobot::SessionState::Active;
    sample.task_aperture_valid = true;
    sample.task_velocity_valid = true;
    sample.task_aperture_mm = 20.0;
    sample.task_aperture_limits_valid = true;
    sample.maximum_task_aperture_mm = 100.0;
    sample.sample_sequence = 100;
    sample.received_at = std::chrono::steady_clock::now();
    handoff_probe::deliver = true;
    handoff_probe::command_busy = true;
    handoff_probe::command_calls = 0;
    handoff_probe::stop_calls = 0;
    handoff_probe::stop_busy_count = 3;
    const rclcpp::Time now{};
    const auto period = rclcpp::Duration::from_seconds(.01);
    ASSERT_EQ(system->read(now, period), hardware_interface::return_type::OK);
    ASSERT_TRUE(position->set_value(.030));
    ASSERT_TRUE(effort->set_value(rg ? 30.0 : 60.0));
    ASSERT_TRUE(event->set_value(9.0));
    ASSERT_EQ(system->write(now, period), hardware_interface::return_type::OK);
    EXPECT_EQ(handoff_probe::command_calls, 1u);
    EXPECT_DOUBLE_EQ(event->get_optional<double>().value(), 9.0);
    ASSERT_TRUE(stop->set_value(11.0));
    for (unsigned attempt = 1; attempt <= 3; ++attempt) {
      ASSERT_EQ(system->write(now, period), hardware_interface::return_type::OK);
      EXPECT_EQ(handoff_probe::stop_calls, attempt);
      EXPECT_EQ(handoff_probe::command_calls, 1u);
      EXPECT_DOUBLE_EQ(stop->get_optional<double>().value(), 11.0);
    }
    ASSERT_EQ(system->write(now, period), hardware_interface::return_type::OK);
    EXPECT_EQ(handoff_probe::stop_calls, 4u);
    EXPECT_TRUE(std::isnan(stop->get_optional<double>().value()));
    EXPECT_LT(event->get_optional<double>().value(), 0.0);
    for (int i = 0; i < 5; ++i) {
      ASSERT_EQ(system->write(now, period), hardware_interface::return_type::OK);
      EXPECT_EQ(handoff_probe::command_calls, 1u);
    }
    sample.applied_command_sequence = 1234;
    sample.active_mode = onrobot::ParallelControlMode::Idle;
    ASSERT_EQ(system->read(now, period), hardware_interface::return_type::OK);
    ASSERT_EQ(system->write(now, period), hardware_interface::return_type::OK);
    EXPECT_EQ(handoff_probe::command_calls, 1u);
    ASSERT_TRUE(event->set_value(std::numeric_limits<double>::quiet_NaN()));
    ASSERT_TRUE(position->set_value(.020)); // controller's numerical hold
    ASSERT_EQ(system->write(now, period), hardware_interface::return_type::OK);
    EXPECT_EQ(handoff_probe::command_calls, 1u);
    // An explicit new event can reuse the canceled target and force.
    handoff_probe::command_busy = false;
    ASSERT_TRUE(position->set_value(.030));
    ASSERT_TRUE(event->set_value(10.0));
    ASSERT_EQ(system->write(now, period), hardware_interface::return_type::OK);
    EXPECT_EQ(handoff_probe::command_calls, 2u);
    EXPECT_DOUBLE_EQ(handoff_probe::last_command.task_aperture_mm, 30.0);
    EXPECT_DOUBLE_EQ(handoff_probe::last_command.force_n, rg ? 30.0 : 60.0);
    EXPECT_TRUE(std::isnan(event->get_optional<double>().value()));
    handoff_probe::deliver = false;
    ASSERT_EQ(system->on_deactivate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  }
}
}

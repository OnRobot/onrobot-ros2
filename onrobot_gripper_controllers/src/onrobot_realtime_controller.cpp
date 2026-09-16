#include "onrobot_gripper_controllers/onrobot_realtime_controller.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace onrobot_gripper_controllers {
namespace {

constexpr std::size_t kPosition = 0;
constexpr std::size_t kEffort = 1;
constexpr std::size_t kMode = 2;
constexpr std::size_t kTaskPosition = 3;
constexpr std::size_t kTaskVelocity = 4;
constexpr std::size_t kForce = 5;
constexpr std::size_t kSequence = 6;

double validStateValue(double i_value, bool &o_valid) {
  o_valid = o_valid && std::isfinite(i_value);
  return o_valid ? i_value : 0.0;
}

uint64_t counterStateValue(double i_value) {
  return std::isfinite(i_value) && i_value >= 0.0
             ? static_cast<uint64_t>(i_value)
             : 0U;
}

} // namespace

controller_interface::InterfaceConfiguration
OnRobotRealtimeController::command_interface_configuration() const {
  const std::string prefix = m_jointName + "/";
  const std::string velocity = m_rgCoordinateProfile
                                   ? "realtime_mechanism_angular_velocity"
                                   : "realtime_task_velocity";
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {prefix + "position", prefix + "effort", prefix + "realtime_mode",
           prefix + "realtime_task_position", prefix + velocity,
           prefix + "realtime_force", prefix + "realtime_command_sequence"}};
}

controller_interface::InterfaceConfiguration
OnRobotRealtimeController::state_interface_configuration() const {
  const std::string task = m_jointName + "/";
  const std::string mechanism = m_mechanismJointName + "/";
  const std::string measuredPosition =
      m_rgCoordinateProfile ? "measured_angular_position" : "measured_position";
  const std::string measuredVelocity =
      m_rgCoordinateProfile ? "measured_angular_velocity" : "measured_velocity";
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {mechanism + measuredPosition, mechanism + measuredVelocity,
           mechanism + "position_valid", mechanism + "velocity_valid",
           task + "position", task + "velocity", task + "task_position_valid",
           task + "effort", task + "force_valid", task + "active_mode",
           task + "faulted", task + "successful_cycles", task + "failed_cycles",
           task + "missed_deadlines", task + "watchdog_stops",
           task + "last_cycle_duration", task + "requested_command_sequence",
           task + "applied_command_sequence", task + "reconnects"}};
}

controller_interface::CallbackReturn OnRobotRealtimeController::on_init() {
  try {
    auto node = get_node();
    node->declare_parameter<std::string>("joint", m_jointName);
    node->declare_parameter<std::string>("mechanism_joint",
                                         m_mechanismJointName);
    node->declare_parameter<std::string>("coordinate_profile", "2fg");
    node->declare_parameter<int>("command_timeout_ms", 100);
    node->declare_parameter<double>("state_publish_rate_hz", 100.0);
  } catch (const std::exception &i_error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Realtime init failed: %s",
                 i_error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
OnRobotRealtimeController::on_configure(const rclcpp_lifecycle::State &) {
  auto node = get_node();
  m_jointName = node->get_parameter("joint").as_string();
  m_mechanismJointName = node->get_parameter("mechanism_joint").as_string();
  const auto coordinateProfile =
      node->get_parameter("coordinate_profile").as_string();
  if (coordinateProfile != "2fg" && coordinateProfile != "rg") {
    RCLCPP_ERROR(node->get_logger(),
                 "coordinate_profile must be '2fg' or 'rg'");
    return controller_interface::CallbackReturn::ERROR;
  }
  m_rgCoordinateProfile = coordinateProfile == "rg";
  const int timeoutMs = node->get_parameter("command_timeout_ms").as_int();
  m_statePublishRateHz =
      node->get_parameter("state_publish_rate_hz").as_double();
  if (m_jointName.empty() || m_mechanismJointName.empty() || timeoutMs <= 0 ||
      !std::isfinite(m_statePublishRateHz) || m_statePublishRateHz <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "Realtime controller parameters invalid");
    return controller_interface::CallbackReturn::ERROR;
  }
  m_timeout = std::chrono::milliseconds(timeoutMs);
  m_epoch.store(0, std::memory_order_relaxed);
  m_acceptMotion.store(false, std::memory_order_relaxed);
  m_sourceBoundaryNs.store(0, std::memory_order_relaxed);
  m_stopFenceSequence.store(0, std::memory_order_relaxed);
  m_stopFenceEpoch.store(0, std::memory_order_relaxed);
  m_cachedValid = false;
  m_seenSequence = 0;
  m_lastRosNowNs = 0;
  m_lastStatePublish = rclcpp::Time(0);
  m_appliedSequence = 0;
  m_stopSequence = 0;
  m_stopPending = true;
  m_activationBoundaryPending = false;
  m_subscription =
      node->create_subscription<onrobot_gripper_msgs::msg::RealtimeCommand>(
          "~/command", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
          [this](const onrobot_gripper_msgs::msg::RealtimeCommand::SharedPtr
                     message) {
            // Stop is handled by the independent fence, so a motion box
            // update can never overwrite a Stop which is waiting for update.
            if (message->mode ==
                onrobot_gripper_msgs::msg::RealtimeCommand::STOP) {
              queueStop();
              return;
            }
            if (!m_acceptMotion.load(std::memory_order_acquire)) {
              return;
            }

            // rclcpp::Time rejects negative seconds but may normalize an
            // out-of-range nanosecond field. Validate the wire-level ROS stamp
            // before constructing it so malformed DDS input follows the
            // ordered Stop path rather than escaping the callback and leaving
            // the prior motion live.
            if (message->header.stamp.sec < 0 ||
                message->header.stamp.nanosec >= 1000000000U) {
              queueStop();
              return;
            }

            PendingCommand next;
            next.mode = message->mode;
            next.task_position = message->task_position;
            next.task_velocity = message->task_velocity;
            next.mechanism_angular_velocity =
                message->mechanism_angular_velocity;
            next.force = message->force;
            next.source_stamp_ns =
                rclcpp::Time(message->header.stamp).nanoseconds();
            next.received_at_steady_ns =
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count();
            next.valid = true;

            // Sequence allocation and publication are one serialized
            // producer operation.  update and lifecycle callbacks never take
            // this lock.  A blocking set is safe here because the payload is
            // fixed-size and update only holds the box lock for its copy.
            std::lock_guard<std::mutex> lock(m_inputMutex);
            if (!m_acceptMotion.load(std::memory_order_acquire)) {
              return;
            }
            next.epoch = m_epoch.load(std::memory_order_acquire);
            next.sequence = m_nextSequence.fetch_add(1);
            const auto boundary =
                m_sourceBoundaryNs.load(std::memory_order_acquire);
            // The controller node and the controller-manager trigger clock
            // receive /clock independently.  Do not compare the publisher
            // stamp with this callback's node clock: either subscription can
            // be one simulation tick ahead.  update() applies the strict
            // source-time check against its authoritative i_time immediately
            // before writing command interfaces.  The activation boundary is
            // still safe to check here because it is fixed for this epoch.
            if (!validMotion(next) || next.source_stamp_ns <= boundary) {
              publishStopFence(next.sequence, next.epoch);
              return;
            }
            m_commandBox.set(next);
          });
  m_stopService = node->create_service<std_srvs::srv::Trigger>(
      "~/stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        queueStop();
        response->success = true;
        response->message = "realtime Stop queued";
      });
  m_statePublisher = std::make_unique<realtime_tools::RealtimePublisher<
      onrobot_gripper_msgs::msg::RealtimeState>>(
      node->create_publisher<onrobot_gripper_msgs::msg::RealtimeState>(
          "~/state", rclcpp::SensorDataQoS()));
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
OnRobotRealtimeController::on_activate(const rclcpp_lifecycle::State &) {
  m_acceptMotion.store(false, std::memory_order_release);
  m_epoch.fetch_add(1, std::memory_order_acq_rel);
  // update() supplies the controller-manager trigger clock.  Keep motion
  // admission closed until its first cycle establishes the activation source
  // boundary and successfully writes the mandatory activation Stop.
  m_activationBoundaryPending = true;
  // Old box contents are deliberately not read or cleared here: the epoch
  // check in update makes them inert without taking the producer mutex.
  m_cachedValid = false;
  m_seenSequence = 0;
  m_lastRosNowNs = 0;
  m_lastStatePublish = rclcpp::Time(0);
  m_appliedSequence = 0;
  m_stopPending = true;
  m_stopSequence = nextSequence();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
OnRobotRealtimeController::on_deactivate(const rclcpp_lifecycle::State &) {
  // Close admission and advance the epoch before touching the interfaces.
  // A callback which straddles this transition is rejected by update.
  m_acceptMotion.store(false, std::memory_order_release);
  m_epoch.fetch_add(1, std::memory_order_acq_rel);
  m_activationBoundaryPending = false;
  m_cachedValid = false;
  m_stopPending = true;
  m_stopSequence = nextSequence();
  if (writeStop(m_stopSequence)) {
    m_appliedSequence = m_stopSequence;
    m_stopPending = false;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  // An interface failure must be reported rather than
  // claiming that deactivation confirmed a physical Stop.
  return controller_interface::CallbackReturn::ERROR;
}

controller_interface::return_type
OnRobotRealtimeController::update(const rclcpp::Time &i_time,
                                  const rclcpp::Duration &) {
  const int64_t ros_now_ns = i_time.nanoseconds();
  // Source clock rollback is a command boundary.  It invalidates both the
  // coalesced box entry and the RT-local cache without waiting for a callback.
  if (m_lastRosNowNs != 0 && ros_now_ns < m_lastRosNowNs) {
    m_epoch.fetch_add(1, std::memory_order_acq_rel);
    m_sourceBoundaryNs.store(ros_now_ns, std::memory_order_release);
    m_cachedValid = false;
    m_lastStatePublish = rclcpp::Time(0);
    requestStop();
  }
  m_lastRosNowNs = ros_now_ns;

  // A failed nonblocking read means contention, not missing input.  Keep
  // evaluating the last accepted command from its original receipt time.
  const auto pending = m_commandBox.try_get();
  if (pending && pending->valid && pending->sequence > m_seenSequence) {
    m_seenSequence = pending->sequence;
    if (pending->epoch == m_epoch.load(std::memory_order_acquire) &&
        m_acceptMotion.load(std::memory_order_acquire)) {
      m_cachedCommand = *pending;
      m_cachedValid = true;
    }
  }

  const uint64_t fence_sequence =
      m_stopFenceSequence.load(std::memory_order_acquire);
  const uint64_t fence_epoch = m_stopFenceEpoch.load(std::memory_order_relaxed);
  if (fence_sequence > m_appliedSequence &&
      fence_epoch == m_epoch.load(std::memory_order_acquire)) {
    if (!m_cachedValid || m_cachedCommand.sequence <= fence_sequence) {
      m_cachedValid = false;
    }
    m_stopPending = true;
    m_stopSequence = fence_sequence;
  }

  // Stop has priority over a motion published after it.  Leave that motion
  // cached for the next update, which preserves the explicit input order.
  if (m_stopPending && m_stopSequence != m_appliedSequence) {
    if (!writeStop(m_stopSequence)) {
      return controller_interface::return_type::ERROR;
    }
    m_appliedSequence = m_stopSequence;
    m_stopPending = false;
    if (!m_cachedValid || m_cachedCommand.sequence <= m_stopSequence) {
      m_cachedValid = false;
    }
    if (m_activationBoundaryPending) {
      m_sourceBoundaryNs.store(ros_now_ns, std::memory_order_release);
      m_activationBoundaryPending = false;
      m_acceptMotion.store(true, std::memory_order_release);
    }
    return publishState(i_time);
  }

  if (m_cachedValid) {
    const auto now_steady_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    const auto timeout_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(m_timeout).count();
    if (!validMotion(m_cachedCommand) ||
        !freshSource(m_cachedCommand, ros_now_ns) ||
        now_steady_ns - m_cachedCommand.received_at_steady_ns > timeout_ns ||
        now_steady_ns < m_cachedCommand.received_at_steady_ns) {
      m_cachedValid = false;
      requestStop();
    } else if (m_cachedCommand.sequence > m_appliedSequence &&
               m_cachedCommand.sequence > fence_sequence) {
      const auto &command = m_cachedCommand;
      const double velocity = m_rgCoordinateProfile
                                  ? command.mechanism_angular_velocity
                                  : command.task_velocity;
      const bool mode_written =
          writeInterface(kMode, static_cast<double>(command.mode));
      const bool position_written =
          writeInterface(kTaskPosition, command.task_position);
      const bool velocity_written = writeInterface(kTaskVelocity, velocity);
      const bool force_written = writeInterface(kForce, command.force);
      const bool sequence_written =
          mode_written && position_written && velocity_written &&
          force_written &&
          writeInterface(kSequence, static_cast<double>(command.sequence));
      if (!sequence_written) {
        m_cachedValid = false;
        requestStop();
        // A failed payload write may have changed some command fields.  Try
        // the Stop immediately as well as retaining the pending fence for a
        // later update; the failed motion sequence is never committed.
        if (writeStop(m_stopSequence)) {
          m_appliedSequence = m_stopSequence;
          m_stopPending = false;
        }
        return controller_interface::return_type::ERROR;
      }
      m_appliedSequence = command.sequence;
      m_stopPending = false;
    }
  }

  if (m_stopPending && m_stopSequence != m_appliedSequence) {
    if (!writeStop(m_stopSequence)) {
      return controller_interface::return_type::ERROR;
    }
    m_appliedSequence = m_stopSequence;
    m_stopPending = false;
  }

  return publishState(i_time);
}

controller_interface::return_type
OnRobotRealtimeController::publishState(const rclcpp::Time &i_time) {

  const auto controllerRateHz = get_update_rate();
  const bool publishEveryUpdate =
      controllerRateHz > 0 && m_statePublishRateHz >= controllerRateHz;
  if (publishEveryUpdate || !m_lastStatePublish.nanoseconds() ||
      (i_time - m_lastStatePublish).seconds() >= 1.0 / m_statePublishRateHz) {
    if (!m_statePublisher->trylock()) {
      return controller_interface::return_type::OK;
    }
    auto &state = m_statePublisher->msg_;
    state.header.stamp = i_time;
    state.mechanism_linear_position_valid = false;
    state.mechanism_linear_velocity_valid = false;
    state.mechanism_angular_position_valid = false;
    state.mechanism_angular_velocity_valid = false;
    if (m_rgCoordinateProfile) {
      const double position = stateValue(0);
      const double velocity = stateValue(1);
      state.mechanism_angular_position_valid = stateValue(2) > 0.5;
      state.mechanism_angular_velocity_valid = stateValue(3) > 0.5;
      state.mechanism_angular_position =
          validStateValue(position, state.mechanism_angular_position_valid);
      state.mechanism_angular_velocity =
          validStateValue(velocity, state.mechanism_angular_velocity_valid);
    } else {
      const double position = stateValue(0);
      const double velocity = stateValue(1);
      state.mechanism_linear_position_valid = stateValue(2) > 0.5;
      state.mechanism_linear_velocity_valid = stateValue(3) > 0.5;
      state.mechanism_linear_position =
          validStateValue(position, state.mechanism_linear_position_valid);
      state.mechanism_linear_velocity =
          validStateValue(velocity, state.mechanism_linear_velocity_valid);
    }
    const double taskPosition = stateValue(4);
    const double taskVelocity = stateValue(5);
    state.task_position_valid = stateValue(6) > 0.5;
    state.task_velocity_valid = std::isfinite(taskVelocity);
    state.task_position =
        validStateValue(taskPosition, state.task_position_valid);
    state.task_velocity =
        validStateValue(taskVelocity, state.task_velocity_valid);
    const double force = stateValue(7);
    state.force_valid = stateValue(8) > 0.5;
    state.force = validStateValue(force, state.force_valid);
    const auto sessionMode = static_cast<uint8_t>(stateValue(9));
    state.realtime_active = sessionMode >= 2;
    state.active_mode = state.realtime_active
                            ? static_cast<uint8_t>(sessionMode - 1)
                            : onrobot_gripper_msgs::msg::RealtimeState::IDLE;
    state.faulted = stateValue(10) > 0.5;
    state.successful_cycles = counterStateValue(stateValue(11));
    state.failed_cycles = counterStateValue(stateValue(12));
    state.missed_deadlines = counterStateValue(stateValue(13));
    state.watchdog_stops = counterStateValue(stateValue(14));
    state.last_cycle_duration = stateValue(15);
    state.requested_command_sequence = counterStateValue(stateValue(16));
    state.applied_command_sequence = counterStateValue(stateValue(17));
    state.reconnects = counterStateValue(stateValue(18));
    m_statePublisher->unlockAndPublish();
    m_lastStatePublish = i_time;
  }
  return controller_interface::return_type::OK;
}

bool OnRobotRealtimeController::writeInterface(std::size_t i_index,
                                               double i_value) {
  return i_index < command_interfaces_.size() && std::isfinite(i_value) &&
         command_interfaces_[i_index].set_value(i_value);
}

bool OnRobotRealtimeController::writeStop(uint64_t i_sequence) {
  // Selector-first is deliberate: do not advance the sequence if Stop could
  // not replace a potentially actionable motion selector.
  if (!writeInterface(kMode, -1.0)) {
    return false;
  }
  return writeInterface(kSequence, static_cast<double>(i_sequence));
}

double OnRobotRealtimeController::stateValue(std::size_t i_index) const {
  if (i_index >= state_interfaces_.size()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return state_interfaces_[i_index].get_optional<double>().value_or(
      std::numeric_limits<double>::quiet_NaN());
}

void OnRobotRealtimeController::requestStop() {
  m_stopPending = true;
  m_stopSequence = nextSequence();
}

void OnRobotRealtimeController::queueStop() {
  std::lock_guard<std::mutex> lock(m_inputMutex);
  publishStopFence(nextSequence(), m_epoch.load(std::memory_order_acquire));
}

bool OnRobotRealtimeController::validMotion(
    const PendingCommand &i_command) const {
  using Command = onrobot_gripper_msgs::msg::RealtimeCommand;
  if (i_command.mode != Command::POSITION &&
      i_command.mode != Command::VELOCITY &&
      i_command.mode != Command::FORCE_POSITION &&
      i_command.mode != Command::FORCE_VELOCITY) {
    return false;
  }
  if (!std::isfinite(i_command.task_position) ||
      !std::isfinite(i_command.force)) {
    return false;
  }
  const double velocity = m_rgCoordinateProfile
                              ? i_command.mechanism_angular_velocity
                              : i_command.task_velocity;
  if (!std::isfinite(velocity)) {
    return false;
  }
  if (!m_rgCoordinateProfile && i_command.force < 0.0) {
    return false;
  }
  if (!m_rgCoordinateProfile &&
      (i_command.mode == Command::POSITION ||
       i_command.mode == Command::FORCE_POSITION) &&
      i_command.task_velocity < 0.0) {
    return false;
  }
  if (m_rgCoordinateProfile && (i_command.mode == Command::FORCE_POSITION ||
                                i_command.mode == Command::FORCE_VELOCITY)) {
    return false;
  }
  return i_command.source_stamp_ns > 0 && i_command.received_at_steady_ns > 0;
}

bool OnRobotRealtimeController::freshSource(const PendingCommand &i_command,
                                            int64_t i_ros_now_ns) const {
  if (i_command.source_stamp_ns <= 0 ||
      i_ros_now_ns < i_command.source_stamp_ns ||
      i_command.source_stamp_ns <=
          m_sourceBoundaryNs.load(std::memory_order_acquire)) {
    return false;
  }
  const auto age_ns = i_ros_now_ns - i_command.source_stamp_ns;
  const auto timeout_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(m_timeout).count();
  return age_ns <= timeout_ns;
}

uint64_t OnRobotRealtimeController::nextSequence() {
  return m_nextSequence.fetch_add(1, std::memory_order_relaxed);
}

void OnRobotRealtimeController::publishStopFence(uint64_t i_sequence,
                                                 uint64_t i_epoch) {
  m_stopFenceEpoch.store(i_epoch, std::memory_order_relaxed);
  m_stopFenceSequence.store(i_sequence, std::memory_order_release);
}

} // namespace onrobot_gripper_controllers

PLUGINLIB_EXPORT_CLASS(onrobot_gripper_controllers::OnRobotRealtimeController,
                       controller_interface::ControllerInterface)

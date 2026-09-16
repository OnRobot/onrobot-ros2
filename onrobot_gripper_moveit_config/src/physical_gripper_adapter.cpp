#include <algorithm>
#include <chrono>
#include <csignal>
#include <cmath>
#include <memory>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include <control_msgs/action/parallel_gripper_command.hpp>
#include <control_msgs/msg/float64_values.hpp>
#include <onrobot_gripper_description/gripper_profile.hpp>
#include <onrobot_gripper_msgs/msg/gripper_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace {
using Action = control_msgs::action::ParallelGripperCommand;
using ServerHandle = rclcpp_action::ServerGoalHandle<Action>;
using ClientHandle = rclcpp_action::ClientGoalHandle<Action>;
using Profile = onrobot_gripper_description::GripperProfile;
using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
using State = onrobot_gripper_msgs::msg::GripperState;
volatile std::sig_atomic_t interrupted = 0;
void handleSignal(int) { interrupted = 1; }

double secondsSince(Time value) {
  return std::chrono::duration<double>(Clock::now() - value).count();
}

// All state belongs to the default mutually-exclusive callback group. No
// detached workers, blocking device calls, or additional hardware owner.
class PhysicalGripperAdapter : public rclcpp::Node {
public:
  PhysicalGripperAdapter() : Node("physical_gripper_adapter") {
    const auto path = declare_parameter<std::string>("profile", "");
    profile_ = std::make_unique<Profile>(Profile::load(path));
    const auto expected = declare_parameter<std::string>("profile_hash", "");
    if (expected.empty() || expected != profile_->profileHash())
      throw std::invalid_argument("profile_hash must match the resolved profile");
    const bool linear = profile_->model().rfind("2fg", 0) == 0;
    const auto prefix = declare_parameter<std::string>("joint_prefix", "");
    physical_joint_ = prefix + (linear ? "finger_stroke" : "finger_joint");
    task_joint_ = prefix + "grip_stroke";
    freshness_s_ = declare_parameter<double>("state_timeout_s", 0.5);
    timeout_s_ = declare_parameter<double>("goal_timeout_s", 20.0);
    agreement_ = declare_parameter<double>("joint_agreement_tolerance", linear ? 0.00025 : 0.003);
    for (double value : {freshness_s_, timeout_s_, agreement_}) {
      if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument("adapter timeouts and tolerance must be positive and finite");
    }
    client_ = rclcpp_action::create_client<Action>(
        this, declare_parameter<std::string>("task_action", "gripper_controller/gripper_cmd"));
    state_sub_ = create_subscription<State>(
        declare_parameter<std::string>("state_topic", "gripper_state_broadcaster/state"),
        rclcpp::SensorDataQoS(), [this](State::ConstSharedPtr state) {
          if (!state_ || state_->sample_sequence != state->sample_sequence)
            sample_received_ = Clock::now();
          state_received_ = Clock::now();
          state_ = state;
        });
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        declare_parameter<std::string>("joint_states_topic", "joint_states"),
        rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::JointState::ConstSharedPtr state) {
          const auto it = std::find(state->name.begin(), state->name.end(), physical_joint_);
          if (it == state->name.end()) return;
          const auto index = static_cast<std::size_t>(it - state->name.begin());
          joint_ = index < state->position.size() && std::isfinite(state->position[index])
                       ? std::optional<double>(state->position[index]) : std::nullopt;
          const auto task = std::find(state->name.begin(), state->name.end(), task_joint_);
          const auto task_index = static_cast<std::size_t>(task - state->name.begin());
          joint_task_ = task != state->name.end() && task_index < state->position.size() &&
                            std::isfinite(state->position[task_index])
                        ? std::optional<double>(state->position[task_index]) : std::nullopt;
          joint_received_ = Clock::now();
        });
    limits_sub_ = create_subscription<control_msgs::msg::Float64Values>(
        declare_parameter<std::string>("limits_topic", "parallel_gripper_limit_broadcaster/values"),
        rclcpp::SensorDataQoS(), [this](control_msgs::msg::Float64Values::ConstSharedPtr message) {
          const auto &v = message->values;
          if (v.size() != 2 || !std::isfinite(v[0]) || !std::isfinite(v[1]) ||
              v[0] < 0.0 || v[0] >= v[1]) {
            limits_.reset();
          } else {
            limits_ = std::make_pair(v[0], v[1]);
          }
          limits_received_ = Clock::now();
        });
    server_ = rclcpp_action::create_server<Action>(
        this, declare_parameter<std::string>("physical_action", "physical_gripper_controller/gripper_cmd"),
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) {
          try {
            if (latched_ || pending_ || !healthy() || !client_->action_server_is_ready())
              throw std::invalid_argument("fresh matching geometry/state and an idle action slot are required");
            const auto &command = goal->command;
            if (command.name != std::vector<std::string>{physical_joint_} ||
                command.position.size() != 1 || !command.velocity.empty() || !command.effort.empty())
              throw std::invalid_argument("use the physical joint name, one position and empty velocity/effort arrays");
            validateTarget(command.position[0]);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
          } catch (const std::exception &error) {
            RCLCPP_WARN(get_logger(), "Planning goal rejected: %s", error.what());
            return rclcpp_action::GoalResponse::REJECT;
          }
        },
        [this](std::shared_ptr<ServerHandle> goal) {
          if (pending_ == goal) return rclcpp_action::CancelResponse::ACCEPT;
          if (active_ && active_->upstream == goal) {
            requestCancel(active_);
            return rclcpp_action::CancelResponse::ACCEPT;
          }
          return rclcpp_action::CancelResponse::REJECT;
        },
        [this](std::shared_ptr<ServerHandle> goal) {
          // Preemption is bounded to one pending request. Do not submit the new
          // command until the previous downstream action reaches terminal state.
          pending_ = goal;
          if (active_) {
            active_->superseded = true;
            requestCancel(active_);
          }
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { tick(); });
    const auto safe = profile_->safeAperture();
    RCLCPP_INFO(get_logger(), "Physical planning for %s: aperture %.6f..%.6f m; profile %s. "
                "Device limits and raw telemetry are unchanged.",
                profile_->model().c_str(), safe.minimum, safe.maximum, expected.c_str());
  }

  void beginShutdown() {
    latched_ = true;
    if (active_ && !active_->finished) {
      active_->failed = true;
      requestCancel(active_);
    }
  }

  bool shutdownComplete() const { return !active_ && !pending_; }
  bool stopUnconfirmed() const { return stop_unconfirmed_; }

private:
  struct Request {
    std::shared_ptr<ServerHandle> upstream;
    ClientHandle::SharedPtr downstream;
    Time started{Clock::now()}, canceled{};
    bool superseded{false}, cancel_sent{false};
    bool failed{false}, finished{false};
  };

  bool healthy() const {
    if (!state_ || !joint_ || !joint_task_ || !limits_ ||
        secondsSince(limits_received_) > freshness_s_ || secondsSince(state_received_) > freshness_s_ ||
        secondsSince(sample_received_) > freshness_s_ ||
        secondsSince(joint_received_) > freshness_s_) return false;
    const auto &s = *state_;
    const double age = s.sample_age.sec + s.sample_age.nanosec * 1e-9;
    if (s.model != profile_->model() || !s.task_aperture_valid ||
        !std::isfinite(s.task_aperture) || age < 0.0 || age > freshness_s_ ||
        s.mapping_validity != State::MAPPING_VALID || s.fault_source != State::FAULT_SOURCE_NONE ||
        s.fault_code != 0 || (s.connection_state != State::CONNECTION_IDLE &&
                             s.connection_state != State::CONNECTION_ACTIVE) ||
        (s.safety_status_valid && (s.safety_1_pushed || s.safety_2_pushed ||
          s.safety_1_triggered || s.safety_2_triggered || s.safety_dc_error))) return false;
    try {
      // Compare coordinates from the same JointState sample. Independently
      // scheduled typed-state and joint-state publications can straddle motion.
      return std::abs(measuredJoint(*joint_task_) - *joint_) <= agreement_;
    } catch (const std::exception &) { return false; }
  }

  double validateTarget(double joint) const {
    const double aperture = profile_->planningJointToAperture(joint);
    // Command input has already passed strict profile-domain validation. Only
    // absorb arithmetic rounding of the derived value at a live decimal bound.
    constexpr double roundoff = 8.0 * std::numeric_limits<double>::epsilon();
    if (!limits_ || aperture < limits_->first - roundoff || aperture > limits_->second + roundoff)
      throw std::invalid_argument("planning target exceeds the live device aperture limits");
    return std::clamp(aperture, limits_->first, limits_->second);
  }

  double measuredJoint(double aperture) const {
    if (!std::isfinite(aperture)) throw std::invalid_argument("non-finite measured aperture");
    const auto a = profile_->safeAperture();
    const double bounded = std::clamp(aperture, a.minimum, a.maximum);
    const double q = profile_->apertureToPlanningJoint(bounded);
    // Measured pad compression/encoder quantization can lie just outside the
    // ideal rigid geometry. Keep that raw task feedback untouched on its own
    // topic, and permit only the already-declared joint agreement tolerance
    // for the derived physical state. Never apply this to command targets.
    const double tolerance = std::abs(profile_->map().apertureJacobian(q)) * agreement_;
    if (std::abs(aperture - bounded) > tolerance)
      throw std::invalid_argument("measured aperture is outside the geometry agreement tolerance");
    return q;
  }

  sensor_msgs::msg::JointState physicalState(const sensor_msgs::msg::JointState &state) const {
    if (state.position.size() != 1 ||
        (!state.name.empty() && state.name != std::vector<std::string>{task_joint_}))
      throw std::invalid_argument("downstream action returned an invalid task state");
    sensor_msgs::msg::JointState output;
    output.header = state.header;
    output.name = {physical_joint_};
    output.position = {measuredJoint(state.position[0])};
    // Neither task force in N nor its optional speed is a physical-joint
    // effort/velocity contract. Do not forward dimensionally different fields.
    return output;
  }

  void requestCancel(const std::shared_ptr<Request> &request) {
    if (request->canceled == Time{}) request->canceled = Clock::now();
    if (request->downstream && !request->cancel_sent) {
      request->cancel_sent = true;
      try { client_->async_cancel_goal(request->downstream); }
      catch (const std::exception &error) {
        request->failed = true;
        RCLCPP_ERROR(get_logger(), "Downstream cancellation failed: %s", error.what());
      }
    }
  }

  void finish(const std::shared_ptr<Request> &request,
              const ClientHandle::WrappedResult *wrapped = nullptr) {
    if (request->finished) return;
    request->finished = true;
    // A terminal callback can precede the next watchdog tick. Never report a
    // successful motion using state that already became invalid in that gap.
    if (!healthy()) request->failed = true;
    try { validateTarget(request->upstream->get_goal()->command.position[0]); }
    catch (const std::exception &) { request->failed = true; }
    auto result = std::make_shared<Action::Result>();
    if (wrapped && wrapped->result) {
      try {
        result->state = physicalState(wrapped->result->state);
        result->stalled = wrapped->result->stalled;
        result->reached_goal = wrapped->result->reached_goal &&
            std::abs(result->state.position[0] - request->upstream->get_goal()->command.position[0]) <= agreement_;
      } catch (const std::exception &error) {
        request->failed = true;
        RCLCPP_ERROR(get_logger(), "Invalid action result: %s", error.what());
      }
    }
    if (request->upstream->is_canceling()) request->upstream->canceled(result);
    else if (wrapped && wrapped->result &&
             wrapped->code == rclcpp_action::ResultCode::SUCCEEDED &&
             (result->reached_goal || result->stalled) &&
             !request->failed && !request->superseded) request->upstream->succeed(result);
    else request->upstream->abort(result);
  }

  void begin(std::shared_ptr<ServerHandle> goal) {
    auto request = std::make_shared<Request>();
    request->upstream = std::move(goal);
    active_ = request;
    Action::Goal converted;
    converted.command.name = {task_joint_};
    converted.command.position = {
        validateTarget(request->upstream->get_goal()->command.position[0])};
    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback = [this, request](ClientHandle::SharedPtr handle) {
      request->downstream = handle;
      if (!handle) { finish(request); return; }
      // Includes cancellation received before the async acceptance response.
      if (request->canceled != Time{} || request->finished) requestCancel(request);
    };
    options.feedback_callback = [this, request](ClientHandle::SharedPtr,
                                               std::shared_ptr<const Action::Feedback> feedback) {
      if (request->finished || !request->upstream->is_active()) return;
      try {
        auto converted_feedback = std::make_shared<Action::Feedback>();
        converted_feedback->state = physicalState(feedback->state);
        request->upstream->publish_feedback(converted_feedback);
      } catch (const std::exception &error) {
        request->failed = true;
        requestCancel(request);
        RCLCPP_ERROR(get_logger(), "Invalid action feedback: %s", error.what());
      }
    };
    options.result_callback = [this, request](const ClientHandle::WrappedResult &result) {
      finish(request, &result);
    };
    try { client_->async_send_goal(converted, options); }
    catch (const std::exception &error) {
      request->failed = true;
      finish(request);
      RCLCPP_ERROR(get_logger(), "Task action submission failed: %s", error.what());
    }
  }

  void tick() {
    if (active_ && !active_->finished) {
      bool target_valid = true;
      try { validateTarget(active_->upstream->get_goal()->command.position[0]); }
      catch (const std::exception &) { target_valid = false; }
      if (!target_valid || !healthy() || secondsSince(active_->started) > timeout_s_) {
        active_->failed = true;
        requestCancel(active_);
      }
      if (active_->canceled != Time{} && secondsSince(active_->canceled) > 2.0) {
        // An unacknowledged cancellation must never free the slot for another
        // motion. Recreate the adapter after diagnosing the downstream owner.
        latched_ = true;
        stop_unconfirmed_ = true;
        active_->failed = true;
        finish(active_);
        RCLCPP_ERROR(get_logger(), "Cancellation did not reach terminal state; adapter inhibited");
      }
    }
    if (active_ && active_->finished) active_.reset();
    if (pending_ && pending_->is_canceling()) {
      pending_->canceled(std::make_shared<Action::Result>());
      pending_.reset();
    }
    if (!active_ && pending_) {
      auto goal = std::move(pending_);
      pending_.reset();
      if (latched_ || !healthy() || !client_->action_server_is_ready())
        goal->abort(std::make_shared<Action::Result>());
      else {
        try { begin(goal); }
        catch (const std::exception &error) {
          if (active_) { active_->failed = true; finish(active_); }
          else goal->abort(std::make_shared<Action::Result>());
          RCLCPP_ERROR(get_logger(), "Planning target became invalid: %s", error.what());
        }
      }
    }
  }

  std::unique_ptr<Profile> profile_;
  std::string physical_joint_, task_joint_;
  double freshness_s_, timeout_s_, agreement_;
  bool latched_{false};
  bool stop_unconfirmed_{false};
  State::ConstSharedPtr state_;
  std::optional<double> joint_, joint_task_;
  std::optional<std::pair<double, double>> limits_;
  Time sample_received_{}, state_received_{}, joint_received_{}, limits_received_{};
  std::shared_ptr<Request> active_;
  std::shared_ptr<ServerHandle> pending_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::Subscription<State>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<control_msgs::msg::Float64Values>::SharedPtr limits_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);
  try {
    auto node = std::make_shared<PhysicalGripperAdapter>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !interrupted)
      executor.spin_once(std::chrono::milliseconds(50));
    node->beginShutdown();
    const auto deadline = Clock::now() + std::chrono::seconds(3);
    while (rclcpp::ok() && !node->shutdownComplete() && Clock::now() < deadline)
      executor.spin_once(std::chrono::milliseconds(20));
    if (!node->shutdownComplete() || node->stopUnconfirmed()) {
      RCLCPP_ERROR(node->get_logger(), "Shutdown cannot confirm downstream motion stopped");
      rclcpp::shutdown();
      return 2;
    }
  }
  catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("physical_gripper_adapter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

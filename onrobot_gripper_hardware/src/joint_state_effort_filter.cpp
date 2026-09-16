#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "onrobot_gripper_hardware/gripper_semantics.hpp"

namespace onrobot_gripper_hardware {

using onrobot_gripper_hardware::clearUnavailableJointStateField;

class JointStateEffortFilter final : public rclcpp::Node {
public:
  JointStateEffortFilter() : Node("joint_state_effort_filter") {
    const auto inputTopic = declare_parameter<std::string>(
        "input_topic", "joint_state_broadcaster/joint_states");
    const auto outputTopic =
        declare_parameter<std::string>("output_topic", "joint_states");
    if (inputTopic == outputTopic) {
      throw std::invalid_argument(
          "joint-state effort filter input and output topics must differ");
    }

    m_publisher = create_publisher<sensor_msgs::msg::JointState>(
        outputTopic, rclcpp::SensorDataQoS());
    m_subscription = create_subscription<sensor_msgs::msg::JointState>(
        inputTopic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::UniquePtr i_message) {
          const auto count = i_message->name.size();
          if ((!i_message->position.empty() && i_message->position.size() != count) ||
              (!i_message->velocity.empty() && i_message->velocity.size() != count) ||
              (!i_message->effort.empty() && i_message->effort.size() != count)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                "Ignoring joint state with mismatched array lengths");
            return;
          }
          clearUnavailableJointStateField(i_message->velocity);
          clearUnavailableJointStateField(i_message->effort);
          if (!i_message->position.empty()) {
            // Preserve alignment and the original timestamp. An unavailable
            // task coordinate must not generate TF_NAN_INPUT or be invented
            // as a fresh zero/held measurement. Finite physical joints still
            // render; consumers needing validity use the typed state topic.
            for (std::size_t i = count; i-- > 0;) {
              if (std::isfinite(i_message->position[i])) continue;
              i_message->name.erase(i_message->name.begin() + i);
              i_message->position.erase(i_message->position.begin() + i);
              if (!i_message->velocity.empty()) i_message->velocity.erase(i_message->velocity.begin() + i);
              if (!i_message->effort.empty()) i_message->effort.erase(i_message->effort.begin() + i);
            }
          }
          // A complete fault may remove every joint. Do not send empty
          // updates into robot_state_publisher or invent a fresh held sample.
          // Existing TF remains the last finite observation at its old stamp.
          if (i_message->name.empty()) return;
          m_publisher->publish(std::move(i_message));
        });
    RCLCPP_INFO(get_logger(),
                "Publishing validity-cleaned joint states from %s to %s",
                inputTopic.c_str(), outputTopic.c_str());
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_publisher;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_subscription;
};

} // namespace onrobot_gripper_hardware

int main(int i_argumentCount, char **i_arguments) {
  rclcpp::init(i_argumentCount, i_arguments);
  try {
    rclcpp::spin(
        std::make_shared<onrobot_gripper_hardware::JointStateEffortFilter>());
  } catch (const std::exception &i_error) {
    RCLCPP_FATAL(rclcpp::get_logger("joint_state_effort_filter"), "%s",
                 i_error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

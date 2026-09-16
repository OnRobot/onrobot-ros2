#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>

class GripperEffortMarkerNode : public rclcpp::Node
{
public:
  GripperEffortMarkerNode()
  : Node("gripper_effort_marker")
  {
    joint_name_ = declare_parameter<std::string>("joint_name", "grip_stroke");
    joint_state_topic_ = declare_parameter<std::string>(
      "joint_state_topic", "/joint_states");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/gripper_effort_marker");
    frame_id_ = declare_parameter<std::string>(
      "frame_id", "right_finger_base_link");
    wrench_frame_id_ = declare_parameter<std::string>(
      "wrench_frame_id", "gripper_effort_frame");
    wrench_topic_ = declare_parameter<std::string>(
      "wrench_topic", "/gripper_effort_wrench");

    max_force_n_ = declare_parameter<double>("max_force_n", 140.0);
    max_arrow_length_m_ = declare_parameter<double>("max_arrow_length_m", 0.10);
    origin_x_ = declare_parameter<double>("origin_x", 0.0);
    origin_y_ = declare_parameter<double>("origin_y", 0.0);
    origin_z_ = declare_parameter<double>("origin_z", 0.02144);

    if (max_force_n_ <= 0.0) {
      throw std::invalid_argument("max_force_n must be positive");
    }
    if (max_arrow_length_m_ <= 0.0) {
      throw std::invalid_argument("max_arrow_length_m must be positive");
    }

    marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      marker_topic_, 10);
    wrench_publisher_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      wrench_topic_, 10);

    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      rclcpp::QoS(10),
      std::bind(
        &GripperEffortMarkerNode::joint_state_callback,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Visualizing effort for joint '%s' on '%s' in frame '%s'",
      joint_name_.c_str(), marker_topic_.c_str(), frame_id_.c_str());
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    const auto joint = std::find(message->name.begin(), message->name.end(), joint_name_);
    if (joint == message->name.end()) {
      return;
    }

    const auto index = static_cast<std::size_t>(
      std::distance(message->name.begin(), joint));
    if (index >= message->effort.size() || !std::isfinite(message->effort[index])) {
      return;
    }

    const double signed_effort_n = message->effort[index];
    const double force_n = std::abs(signed_effort_n);
    // Preserve the sign for the direction, while using the absolute value
    // for the 0..140 N length and color scale. Zero points in +Z.
    const double direction = signed_effort_n < 0.0 ? -1.0 : 1.0;
    const double normalized_force = std::clamp(force_n / max_force_n_, 0.0, 1.0);

    geometry_msgs::msg::WrenchStamped wrench;
    wrench.header.stamp = now();
    wrench.header.frame_id = wrench_frame_id_;
    wrench.wrench.force.z = signed_effort_n;
    wrench_publisher_->publish(wrench);

    // Keep the arrow long enough for RViz to render its arrowhead at zero/
    // very low force; the variable part remains proportional to the absolute
    // force up to max_force_n_.
    const double arrow_length = std::max(
      0.040,
      normalized_force * max_arrow_length_m_);
    const double arrow_head_length = std::min(0.020, arrow_length * 0.45);

    // Green at zero force, yellow at half scale, red at max_force_n_.
    float red = 0.0F;
    float green = 0.0F;
    if (normalized_force <= 0.5) {
      red = static_cast<float>(2.0 * normalized_force);
      green = 1.0F;
    } else {
      red = 1.0F;
      green = static_cast<float>(2.0 * (1.0 - normalized_force));
    }

    visualization_msgs::msg::Marker arrow;
    set_common_marker_fields(arrow, 0);
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.points.resize(2);
    arrow.points[0].x = origin_x_;
    arrow.points[0].y = origin_y_;
    arrow.points[0].z = origin_z_;
    arrow.points[1].x = origin_x_;
    arrow.points[1].y = origin_y_;
    arrow.points[1].z = origin_z_ + direction * arrow_length;
    arrow.scale.x = 0.012;  // shaft diameter
    arrow.scale.y = 0.024;  // head diameter
    arrow.scale.z = arrow_head_length;  // head length
    arrow.frame_locked = true;
    set_color(arrow, red, green);
    marker_publisher_->publish(arrow);

    visualization_msgs::msg::Marker text;
    set_common_marker_fields(text, 1);
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.pose.position.x = origin_x_;
    text.pose.position.y = origin_y_;
    text.pose.position.z = origin_z_ + 0.020;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.025;
    text.frame_locked = true;
    set_color(text, red, green);

    std::ostringstream force_text;
    force_text << std::fixed << std::setprecision(1) << force_n << " N";
    text.text = force_text.str();
    marker_publisher_->publish(text);
  }

  void set_common_marker_fields(
    visualization_msgs::msg::Marker &marker,
    int id) const
  {
    marker.header.stamp = now();
    marker.header.frame_id = frame_id_;
    marker.ns = "gripper_effort";
    marker.id = id;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 500000000;
  }

  static void set_color(
    visualization_msgs::msg::Marker &marker,
    float red,
    float green)
  {
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = 0.0F;
    marker.color.a = 1.0F;
  }

  std::string joint_name_;
  std::string joint_state_topic_;
  std::string marker_topic_;
  std::string frame_id_;
  std::string wrench_frame_id_;
  std::string wrench_topic_;
  double max_force_n_{140.0};
  double max_arrow_length_m_{0.10};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_z_{0.02144};

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperEffortMarkerNode>());
  rclcpp::shutdown();
  return 0;
}

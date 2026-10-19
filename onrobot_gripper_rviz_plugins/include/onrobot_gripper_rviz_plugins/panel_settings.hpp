#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>

namespace onrobot_gripper_rviz_plugins {

/// @cond INTERNAL
/// Settings saved for one RViz panel.
///
/// RViz panels share a ROS node. Each panel loads and saves these values in its
/// own Config entry so changing one panel does not change another panel's settings.
class PanelSettings {
public:
  void load(const rviz_common::Config &i_config);
  void save(rviz_common::Config i_config) const;

  bool readString(const QString &i_key, std::string &o_value) const;
  bool readDouble(const QString &i_key, double &o_value) const;

  void writeString(const QString &i_key, const std::string &i_value);
  void writeDouble(const QString &i_key, double i_value);

private:
  rviz_common::Config m_values;
};
/// @endcond

/// Read a default from the shared ROS node without redeclaring a parameter
/// that already exists. Use the panel's saved Config value first, if available.
template <typename T>
T readOrDeclare(const rclcpp::Node::SharedPtr &i_node,
                const std::string &i_name, const T &i_fallback) {
  if (!i_node) {
    return i_fallback;
  }

  if (!i_node->has_parameter(i_name)) {
    try {
      return i_node->declare_parameter<T>(i_name, i_fallback);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
      // Another panel may have declared this shared default between the
      // has_parameter() check and declaration.  Read it below.
    } catch (const rclcpp::exceptions::InvalidParameterTypeException &) {
      return i_fallback;
    }
  }

  T value = i_fallback;
  try {
    if (i_node->get_parameter(i_name, value)) {
      return value;
    }
  } catch (const rclcpp::ParameterTypeException &) {
    // Keep the panel's safe compiled-in default for an incompatible override.
  }
  return i_fallback;
}

} // namespace onrobot_gripper_rviz_plugins

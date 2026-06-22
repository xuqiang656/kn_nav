#pragma once

#include <algorithm>
#include <string>

#include <art_planner/params.h>
#include <rclcpp/rclcpp.hpp>



namespace art_planner {



template <typename T>
inline T getParamWithDefaultWarning(const rclcpp::Node::SharedPtr& node,
                             const std::string& name,
                             const T& default_val) {
  auto parameter_name = name;
  std::replace(parameter_name.begin(), parameter_name.end(), '/', '.');
  T param = default_val;

  if (!node->has_parameter(parameter_name)) {
    node->declare_parameter<T>(parameter_name, default_val);
  }
  if (!node->get_parameter(parameter_name, param)) {
    RCLCPP_WARN_STREAM(node->get_logger(), "Could not find ROS param \"" << name <<
                       "\", set to default: " << default_val);
  }

  return param;
}

template <>
inline unsigned int getParamWithDefaultWarning<unsigned int>(const rclcpp::Node::SharedPtr& node,
                                                      const std::string& name,
                                                      const unsigned int& default_val) {
  return getParamWithDefaultWarning(node, name, static_cast<int>(default_val));
}



ParamsPtr loadRosParameters(const rclcpp::Node::SharedPtr& node);



}

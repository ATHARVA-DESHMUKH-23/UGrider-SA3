#pragma once
// Read a named SA3 cross-section task from a motion sequence, independent of
// sa2_description/config/workcell.yaml (fixed scene/TF/robot hardware only).
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include "sa3_motion/workcell_program.hpp"
namespace sa3_recipe {
template<typename T>
T parameter(rclcpp::Node & node, const std::string & key, const T & fallback) {
  if (!node.has_parameter(key)) node.declare_parameter<T>(key, fallback);
  return node.get_parameter(key).get_value<T>();
}
inline YAML::Node loadTask(rclcpp::Node & node, const std::string & expected_type) {
  const auto package = parameter<std::string>(node, "motion_config_package", "sa3_motion");
  const auto file = parameter<std::string>(node, "motion_config_file", "config/ms_2_re.yaml");
  const auto override_path = parameter<std::string>(node, "motion_config_path", "");
  const auto task_id = parameter<std::string>(node, "motion_task", "");
  if (task_id.empty()) throw std::runtime_error("Required ROS parameter motion_task is empty");
  const std::string path = override_path.empty() ?
    ament_index_cpp::get_package_share_directory(package) + "/" + file : override_path;
  const auto root = YAML::LoadFile(path);
  const auto sequence = sa3_yaml::require(root, "motion_sequence", "root");
  if (sequence["schema_version"].as<int>(0) != 2)
    throw std::runtime_error("Motion recipe schema_version must be 2: " + path);
  const auto tasks = sa3_yaml::require(sequence, "tasks", "motion_sequence");
  const auto task = sa3_yaml::require(tasks, task_id, "motion_sequence.tasks");
  if (task["type"].as<std::string>("") != expected_type)
    throw std::runtime_error("Task '" + task_id + "' is not type '" + expected_type + "'");
  RCLCPP_INFO(node.get_logger(), "Loaded SA3 %s task '%s' from %s",
    expected_type.c_str(), task_id.c_str(), path.c_str());
  return task;
}
inline YAML::Node loadTask(const rclcpp::Node::SharedPtr & node,
                            const std::string & expected_type) {
  return loadTask(*node, expected_type);
}
} // namespace sa3_recipe

#pragma once
// Shared SA3 YAML program reader: intentionally independent of motion execution.
// Install at sa3_motion/include/sa3_motion/workcell_program.hpp.
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <array>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace sa3_yaml {

template <class T>
T parameter(const rclcpp::Node::SharedPtr & node, const std::string & key, const T & fallback) {
  if (!node->has_parameter(key)) node->declare_parameter<T>(key, fallback);
  return node->get_parameter(key).get_value<T>();
}

inline YAML::Node require(const YAML::Node & node, const std::string & key, const std::string & path) {
  if (!node || !node.IsMap() || !node[key]) {
    throw std::runtime_error("Missing required workcell YAML field: " + path + "." + key);
  }
  return node[key];
}

inline std::string name(const YAML::Node & node, const std::string & path) {
  const auto value = node.as<std::string>();
  if (value.empty()) throw std::runtime_error("Empty workcell YAML name: " + path);
  return value;
}

inline double finite(const YAML::Node & node, const std::string & path) {
  const double v = node.as<double>();
  if (!std::isfinite(v)) throw std::runtime_error("Nonfinite workcell YAML value: " + path);
  return v;
}

inline std::array<double, 3> xyz(const YAML::Node & node, const std::string & path) {
  if (!node || !node.IsSequence() || node.size() != 3)
    throw std::runtime_error(path + " must be [x, y, z]");
  return {finite(node[0], path), finite(node[1], path), finite(node[2], path)};
}

inline std::vector<double> scales(const YAML::Node & node, const std::string & path) {
  if (!node || !node.IsSequence() || node.size() == 0)
    throw std::runtime_error(path + " must be a nonempty velocity scale sequence");
  std::vector<double> result;
  for (const auto & n : node) {
    const double v = finite(n, path);
    if (v <= 0.0 || v > 1.0) throw std::runtime_error(path + " velocity scales must be in (0,1]");
    result.push_back(v);
  }
  return result;
}

inline std::vector<std::string> strings(const YAML::Node & node, const std::string & path) {
  if (!node || !node.IsSequence()) throw std::runtime_error(path + " must be a sequence");
  std::vector<std::string> result;
  for (const auto & n : node) result.push_back(name(n, path));
  return result;
}

inline std::vector<std::string> actions(const YAML::Node & node, const std::string & path) {
  if (!node) return {};
  return strings(node, path);
}

inline YAML::Node loadWorkcell(const rclcpp::Node::SharedPtr & node) {
  const auto package = parameter<std::string>(node, "workcell_config_package", "sa2_description");
  const auto file = parameter<std::string>(node, "workcell_config_file", "config/workcell.yaml");
  const auto explicit_path = parameter<std::string>(node, "workcell_config_path", "");
  if (package.empty() || file.empty()) throw std::runtime_error("Workcell package/file cannot be empty");
  const auto path = explicit_path.empty() ?
    ament_index_cpp::get_package_share_directory(package) + "/" + file : explicit_path;
  RCLCPP_INFO(node->get_logger(), "Loading SA3 motion program from %s", path.c_str());
  auto root = YAML::LoadFile(path);
  auto workcell = require(root, "workcell", "root");
  if (!workcell.IsMap()) throw std::runtime_error("workcell must be a mapping");
  return workcell;
}

struct Step {
  std::string id;
  std::string description;
  std::string reference;
  std::string motion;
  std::string offset_frame{"world"};
  std::array<double, 3> xyz{{0.0, 0.0, 0.0}};
  std::array<double, 3> rpy{{0.0, 0.0, 0.0}};
  std::vector<std::string> before_actions;
  std::vector<std::string> after_actions;
};

inline std::vector<Step> steps(const YAML::Node & section, const std::string & path) {
  if (!section || !section.IsSequence() || section.size() == 0)
    throw std::runtime_error(path + " must be a nonempty list of steps");
  std::set<std::string> ids;
  std::vector<Step> out;
  for (const auto & entry : section) {
    Step s;
    s.id = name(require(entry, "id", path), path + ".id");
    if (!ids.insert(s.id).second) throw std::runtime_error(path + ": duplicate step id " + s.id);
    s.description = entry["description"].as<std::string>(s.id);
    s.reference = name(require(entry, "reference", path + "." + s.id), path + ".reference");
    s.motion = name(require(entry, "motion", path + "." + s.id), path + ".motion");
    s.offset_frame = entry["offset_frame"].as<std::string>("world");
    if (s.offset_frame != "world" && s.offset_frame != "local")
      throw std::runtime_error(path + "." + s.id + ": offset_frame must be world/local");
    if (entry["xyz"]) s.xyz = xyz(entry["xyz"], path + "." + s.id + ".xyz");
    if (entry["rpy"]) s.rpy = xyz(entry["rpy"], path + "." + s.id + ".rpy");
    s.before_actions = actions(entry["before_actions"], path + "." + s.id + ".before_actions");
    s.after_actions = actions(entry["after_actions"], path + "." + s.id + ".after_actions");
    out.push_back(std::move(s));
  }
  return out;
}

inline void expectedStep(const Step & s, const std::string & reference, const std::string & motion,
                         const std::vector<std::string> & required_after = {}) {
  if (s.reference != reference || s.motion != motion || s.after_actions != required_after ||
      !s.before_actions.empty() || s.rpy != std::array<double, 3>{{0.,0.,0.}})
    throw std::runtime_error("Unsafe/unsupported planner step '" + s.id + "': expected reference=" +
                             reference + ", motion=" + motion + ", configured action sequence must match implementation");
}

inline geometry_msgs::msg::Pose offset(const geometry_msgs::msg::Pose & p,
                                       const std::array<double, 3> & d,
                                       const std::string & frame) {
  auto out = p;
  tf2::Vector3 delta(d[0], d[1], d[2]);
  if (frame == "local") {
    tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    if (q.length2() < 1e-12) throw std::runtime_error("Invalid quaternion in YAML pose reference");
    q.normalize();
    delta = tf2::quatRotate(q, delta);
  } else if (frame != "world") throw std::runtime_error("Unsupported offset frame: " + frame);
  out.position.x += delta.x();
  out.position.y += delta.y();
  out.position.z += delta.z();
  return out;
}

}  // namespace sa3_yaml

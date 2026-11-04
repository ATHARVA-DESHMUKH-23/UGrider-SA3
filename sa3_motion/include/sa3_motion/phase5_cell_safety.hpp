#pragma once
// Phase 5 single-motion-owner protocol and sampled full-arm-state validation.
// Both programs may plan/perform non-motion work concurrently; the gate gives
// at most ONE program permission to command joints at a time. No joint-time
// synchronized 24-joint trajectory is claimed by this implementation.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>

namespace sa3_phase5 {
using namespace std::chrono_literals;
using Plan = moveit::planning_interface::MoveGroupInterface::Plan;

inline const moveit_msgs::msg::RobotTrajectory & trajectoryOf(const Plan & plan) {
  return plan.trajectory_;
}
inline const moveit_msgs::msg::RobotTrajectory & trajectoryOf(
  const moveit_msgs::msg::RobotTrajectory & trajectory) { return trajectory; }

inline bool validateFullArmTrajectory(
  const rclcpp::Node::SharedPtr & node,
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const moveit::core::RobotState & current,
  const std::string & label, double max_joint_step = 0.05)
{
  using Validity = moveit_msgs::srv::GetStateValidity;
  const auto & jt = trajectory.joint_trajectory;
  if (!(max_joint_step > 0 && max_joint_step <= 0.2) ||
      !trajectory.multi_dof_joint_trajectory.joint_names.empty() ||
      jt.joint_names.empty() || jt.points.empty()) return false;
  const auto * all = current.getJointModelGroup("sa3_all_robots");
  if (!all || all->getVariableCount() != 24) {
    RCLCPP_ERROR(node->get_logger(), "[%s] missing sa3_all_robots 24-joint model", label.c_str());
    return false;
  }
  for (const auto & point : jt.points)
    if (point.positions.size() != jt.joint_names.size()) return false;

  // Reject stale trajectory starts. The exclusive motion gate must be held
  // throughout planning, validation, execution, and scene actions.
  for (size_t i = 0; i < jt.joint_names.size(); ++i) {
    const double actual = current.getVariablePosition(jt.joint_names[i]);
    if (std::abs(actual - jt.points.front().positions[i]) > 0.025) {
      RCLCPP_ERROR(node->get_logger(), "[%s] stale plan start joint %s (%.4f vs %.4f)",
        label.c_str(), jt.joint_names[i].c_str(), actual, jt.points.front().positions[i]);
      return false;
    }
  }

  // Validate the joints that actually MOVE against the full planning scene.
  // Group-scoped MoveIt collision checking still checks these links against
  // other arms, attached objects, and the world; it avoids unrelated
  // collisions wholly within stationary arms triggering a false rejection.
  // The request's RobotState continues to contain the full 24-arm snapshot.
  std::string collision_group = "sa3_all_robots";
  for (const char * candidate : {"robot3_arm", "sa3_robots_1_2", "robot3_robot4_arms"}) {
    const auto * group = current.getJointModelGroup(candidate);
    if (!group) continue;
    const auto & variables = group->getVariableNames();
    const bool covers_all_moving_joints = std::all_of(
      jt.joint_names.begin(), jt.joint_names.end(),
      [&variables](const std::string & name) {
        return std::find(variables.begin(), variables.end(), name) != variables.end();
      });
    if (covers_all_moving_joints) {
      collision_group = candidate;
      break;
    }
  }
  RCLCPP_INFO(node->get_logger(),
              "[%s] collision validation group='%s' (all arms remain in planning scene)",
              label.c_str(), collision_group.c_str());

  auto client = node->create_client<Validity>("/check_state_validity");
  if (!client->wait_for_service(10s)) {
    RCLCPP_ERROR(node->get_logger(), "[%s] /check_state_validity unavailable", label.c_str());
    return false;
  }
  size_t samples = 0;
  auto check = [&](const std::vector<double> & values) -> bool {
    moveit::core::RobotState state(current);
    for (size_t j = 0; j < values.size(); ++j) {
      if (!std::isfinite(values[j])) return false;
      state.setVariablePosition(jt.joint_names[j], values[j]);
    }
    state.update();
    auto req = std::make_shared<Validity::Request>();
    req->group_name = collision_group;
    moveit::core::robotStateToRobotStateMsg(state, req->robot_state);
    // Joint diff overlays MoveIt's current planning scene, retaining attached
    // rebar/tool bodies even if the local state monitor omitted them.
    req->robot_state.attached_collision_objects.clear();
    req->robot_state.is_diff = true;
    auto future = client->async_send_request(req);
    if (future.wait_for(10s) != std::future_status::ready) {
      client->remove_pending_request(future);
      RCLCPP_ERROR(node->get_logger(), "[%s] full-arm validity timeout sample %zu",
                   label.c_str(), samples);
      return false;
    }
    const auto reply = future.get();
    if (!reply) {
      RCLCPP_ERROR(node->get_logger(),
                   "[%s] null validity response at sample %zu (group='%s')",
                   label.c_str(), samples, collision_group.c_str());
      return false;
    }
    if (!reply->valid) {
      RCLCPP_ERROR(node->get_logger(),
                   "[%s] collision/constraint at sample %zu (group='%s'; %zu contact(s))",
                   label.c_str(), samples, collision_group.c_str(), reply->contacts.size());
      const size_t shown = std::min<size_t>(reply->contacts.size(), 12);
      for (size_t i = 0; i < shown; ++i) {
        const auto & contact = reply->contacts[i];
        RCLCPP_ERROR(node->get_logger(),
                     "[%s] contact %zu: %s <-> %s",
                     label.c_str(), i + 1,
                     contact.contact_body_1.c_str(), contact.contact_body_2.c_str());
      }
      if (reply->contacts.empty()) {
        RCLCPP_ERROR(node->get_logger(),
                     "[%s] invalid sample returned no contacts; inspect active constraints/planning scene",
                     label.c_str());
      }
      return false;
    }
    return true;
  };
  if (!check(jt.points.front().positions)) return false;
  ++samples;
  for (size_t i = 1; i < jt.points.size(); ++i) {
    const auto & prev = jt.points[i - 1].positions;
    const auto & next = jt.points[i].positions;
    double largest = 0.0;
    for (size_t j = 0; j < prev.size(); ++j)
      largest = std::max(largest, std::abs(next[j] - prev[j]));
    const size_t count = std::max<size_t>(1, static_cast<size_t>(std::ceil(largest / max_joint_step)));
    if (samples + count > 4000) {
      RCLCPP_ERROR(node->get_logger(), "[%s] more than 4000 validity samples; fail closed", label.c_str());
      return false;
    }
    for (size_t k = 1; k <= count; ++k) {
      const double alpha = static_cast<double>(k) / static_cast<double>(count);
      std::vector<double> values(next.size());
      for (size_t j = 0; j < next.size(); ++j)
        values[j] = prev[j] + alpha * (next[j] - prev[j]);
      if (!check(values)) return false;
      ++samples;
    }
  }
  RCLCPP_INFO(node->get_logger(), "[%s] %zu full-arm validity samples PASSED", label.c_str(), samples);
  return true;
}

// Recheck the entire 24-arm-joint snapshot immediately before sending a
// command. This also catches unexpected motion by actors outside the gate.
inline bool fullArmStartStillCurrent(
  const rclcpp::Node::SharedPtr & node,
  const moveit::core::RobotState & snapshot,
  const moveit::core::RobotState & live,
  const std::string & label,
  double tolerance_rad = 0.01)
{
  const auto * all = snapshot.getJointModelGroup("sa3_all_robots");
  if (!all || all->getVariableCount() != 24) return false;
  for (const auto & variable : all->getVariableNames()) {
    const double before = snapshot.getVariablePosition(variable);
    const double now = live.getVariablePosition(variable);
    if (!std::isfinite(now) || std::abs(now - before) > tolerance_rad) {
      RCLCPP_ERROR(node->get_logger(),
        "[%s] arm joint moved during validation (%s: %.5f -> %.5f)",
        label.c_str(), variable.c_str(), before, now);
      return false;
    }
  }
  return true;
}

class GateClient {
public:
  GateClient(rclcpp::Node::SharedPtr node, std::string owner, bool enabled)
    : node_(std::move(node)), owner_(std::move(owner)), enabled_(enabled) {
    if (enabled_ && owner_ != "robot3" && owner_ != "robot12")
      throw std::runtime_error("Phase5 gate owner must be robot3 or robot12");
  }
  ~GateClient() {
    // A failed task must not leave another arm waiting for the long timeout.
    // This is best-effort; an abrupt process crash still needs the stage timeout.
    if (enabled_ && started_ && !finished_ && rclcpp::ok()) {
      std::string ignored;
      (void)request("/sa3_phase5/" + owner_ + "_abort", ignored);
    }
  }
  bool enabled() const { return enabled_; }
  bool acquire(double deadline_sec = 600.) {
    if (!enabled_) return true;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(deadline_sec);
    const auto service = "/sa3_phase5/" + owner_ + "_acquire";
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      std::string reason;
      if (request(service, reason)) {
        started_ = true;
        return true;
      }
      // A lost response to an acquire is safely retried: repeated acquire by
      // the current owner is idempotent in the C++ coordinator.
      std::this_thread::sleep_for(250ms);
    }
    RCLCPP_ERROR(node_->get_logger(), "Phase5 gate acquire timed out for %s", owner_.c_str());
    return false;
  }
  bool release() {
    if (!enabled_) return true;
    std::string why;
    // Never issue motion following a release ambiguity; stop on failure.
    if (request("/sa3_phase5/" + owner_ + "_release", why)) return true;
    RCLCPP_ERROR(node_->get_logger(), "Phase5 gate release failed for %s: %s",
                 owner_.c_str(), why.c_str());
    return false;
  }
  bool done() {
    if (!enabled_) return true;
    std::string why;
    if (request("/sa3_phase5/" + owner_ + "_done", why)) {
      finished_ = true;
      return true;
    }
    RCLCPP_ERROR(node_->get_logger(), "Phase5 done not acknowledged for %s: %s",
                 owner_.c_str(), why.c_str());
    return false;
  }
private:
  bool request(const std::string & service, std::string & reason) {
    auto client = node_->create_client<std_srvs::srv::Trigger>(service);
    if (!client->wait_for_service(3s)) {
      reason = "service unavailable";
      return false;
    }
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(5s) != std::future_status::ready) {
      client->remove_pending_request(future);
      reason = "response timeout";
      return false;
    }
    auto result = future.get();
    reason = result ? result->message : "null response";
    return result && result->success;
  }
  rclcpp::Node::SharedPtr node_;
  std::string owner_;
  bool enabled_;
  bool started_{false};
  bool finished_{false};
};

inline bool enabledParam(const rclcpp::Node::SharedPtr & node) {
  if (!node->has_parameter("phase5_gate_enabled"))
    node->declare_parameter<bool>("phase5_gate_enabled", false);
  return node->get_parameter("phase5_gate_enabled").as_bool();
}
} // namespace sa3_phase5
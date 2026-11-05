// SA3 combined motion executor (ROS 2 / MoveIt 2)
// Plan ONE 24-joint OMPL trajectory through existing SRDF group sa3_all_robots.
// YAML references supported:
//   flange_3c_4c:H1  or flange_3d_4d:H1  -> robot3/robot4 SRDF home
//   shear_links_2_re:P1 (pair_index N)   -> robot1/robot2 pick approach
// Additional explicit operations: type: home or type: pose.
// This executable runs one checkpointed motion beat, NOT the complete cell workflow.
// The orchestrator must guarantee exclusive ownership and perform scene actions.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <future>
#include <mutex>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using MGI = moveit::planning_interface::MoveGroupInterface;
namespace
{
using namespace std::chrono_literals;

YAML::Node require(const YAML::Node & parent, const std::string & key,
                   const std::string & where)
{
  auto n = parent[key];
  if (!n || n.IsNull()) throw std::runtime_error(where + ": missing '" + key + "'");
  return n;
}

std::array<double, 3> xyz(const YAML::Node & node, const std::string & where)
{
  if (!node || !node.IsSequence() || node.size() != 3)
    throw std::runtime_error(where + ": expected [x,y,z]");
  std::array<double, 3> a{};
  for (size_t i = 0; i < 3; ++i) {
    a[i] = node[i].as<double>();
    if (!std::isfinite(a[i])) throw std::runtime_error(where + ": nonfinite offset");
  }
  return a;
}

std::array<double, 3> optionalXYZ(const YAML::Node & parent, const std::string & key)
{
  return parent[key] ? xyz(parent[key], key) : std::array<double, 3>{{0., 0., 0.}};
}

std::string stringParam(const rclcpp::Node::SharedPtr & node,
                        const std::string & key, const std::string & default_value)
{
  if (!node->has_parameter(key)) node->declare_parameter<std::string>(key, default_value);
  return node->get_parameter(key).as_string();
}

bool boolParam(const rclcpp::Node::SharedPtr & node,
               const std::string & key, bool default_value)
{
  if (!node->has_parameter(key)) node->declare_parameter<bool>(key, default_value);
  return node->get_parameter(key).as_bool();
}

double doubleParam(const rclcpp::Node::SharedPtr & node,
                   const std::string & key, double default_value)
{
  if (!node->has_parameter(key)) node->declare_parameter<double>(key, default_value);
  return node->get_parameter(key).as_double();
}

struct ArmInfo
{
  std::string name, group, tip;
};

ArmInfo armInfo(const YAML::Node & workcell, const std::string & name)
{
  if (name != "robot1" && name != "robot2" &&
      name != "robot3" && name != "robot4")
    throw std::runtime_error("Unknown robot: " + name);
  auto n = require(require(workcell, "robots", "workcell"), name, "workcell.robots");
  ArmInfo a;
  a.name = name;
  a.group = require(n, "group", name).as<std::string>();
  // Robot 1/2 use planning_link; robot 3/4 use their flange planning link.
  if (n["planning_link"]) a.tip = n["planning_link"].as<std::string>();
  else if (n["link"]) a.tip = n["link"].as<std::string>();
  return a;
}

geometry_msgs::msg::Pose framePose(tf2_ros::Buffer & tf, const std::string & frame)
{
  auto transform = tf.lookupTransform(
    "world", frame, tf2::TimePointZero, tf2::durationFromSec(10.0));
  geometry_msgs::msg::Pose p;
  p.position.x = transform.transform.translation.x;
  p.position.y = transform.transform.translation.y;
  p.position.z = transform.transform.translation.z;
  p.orientation = transform.transform.rotation;
  return p;
}

geometry_msgs::msg::Pose offsetPose(
  geometry_msgs::msg::Pose p, const std::array<double, 3> & d,
  const std::string & frame)
{
  tf2::Vector3 v(d[0], d[1], d[2]);
  if (frame == "local") {
    tf2::Quaternion q(p.orientation.x, p.orientation.y,
                      p.orientation.z, p.orientation.w);
    q.normalize();
    v = tf2::quatRotate(q, v);
  } else if (frame != "world") {
    throw std::runtime_error("offset_frame must be world or local");
  }
  p.position.x += v.x();
  p.position.y += v.y();
  p.position.z += v.z();
  return p;
}

void claim(const ArmInfo & a, std::set<std::string> & used)
{
  if (!used.insert(a.name).second)
    throw std::runtime_error("Robot assigned by two operations: " + a.name);
}

void setHome(moveit::core::RobotState & goal, const ArmInfo & a,
             std::set<std::string> & used)
{
  claim(a, used);
  const auto * jmg = goal.getJointModelGroup(a.group);
  if (!jmg) throw std::runtime_error("SRDF group unavailable: " + a.group);
  if (!goal.setToDefaultValues(jmg, "home"))
    throw std::runtime_error("SRDF home not defined for: " + a.group);
}

void setPoseIK(moveit::core::RobotState & goal, const ArmInfo & a,
               const geometry_msgs::msg::Pose & target,
               std::set<std::string> & used, double timeout)
{
  claim(a, used);
  const auto * jmg = goal.getJointModelGroup(a.group);
  if (!jmg) throw std::runtime_error("SRDF group unavailable: " + a.group);
  if (a.tip.empty()) throw std::runtime_error("No tip link configured for " + a.name);
  if (!goal.setFromIK(jmg, target, a.tip, timeout))
    throw std::runtime_error("IK failed for " + a.name + " at " + a.tip);
}

void applyFlangeH1(const YAML::Node & tasks, const YAML::Node & workcell,
                   const std::string & task_name, const std::string & step,
                   moveit::core::RobotState & goal, std::set<std::string> & used)
{
  if (step != "H1") throw std::runtime_error("Flange combined resolver supports only H1");
  const auto task = require(tasks, task_name, "tasks");
  if (task["type"].as<std::string>() != "flange_pp")
    throw std::runtime_error(task_name + " is not a flange_pp task");
  bool found = false;
  for (const auto & s : require(task, "steps", task_name)) {
    if (s["id"].as<std::string>() == "H1") {
      if (s["reference"].as<std::string>("") != "home" ||
          s["motion"].as<std::string>("") != "moveJ")
        throw std::runtime_error(task_name + ".H1 must be moveJ/home");
      found = true;
    }
  }
  if (!found) throw std::runtime_error(task_name + ": H1 not found");
  setHome(goal, armInfo(workcell, "robot3"), used);
  setHome(goal, armInfo(workcell, "robot4"), used);
}

void applyShearP1(const YAML::Node & tasks, const YAML::Node & workcell,
                  const std::string & task_name, const std::string & step,
                  size_t pair_index, tf2_ros::Buffer & tf,
                  moveit::core::RobotState & goal,
                  std::set<std::string> & used, double ik_timeout)
{
  if (step != "P1") throw std::runtime_error("Shear combined resolver supports only P1");
  const auto task = require(tasks, task_name, "tasks");
  if (task["type"].as<std::string>() != "shear_links_pp")
    throw std::runtime_error(task_name + " is not a shear_links_pp task");
  bool found = false;
  for (const auto & s : require(task, "stages", task_name)) {
    if (s["id"].as<std::string>() == "P1" &&
        s["reference"].as<std::string>("") == "pick_approach") found = true;
  }
  if (!found) throw std::runtime_error(task_name + ": P1 pick_approach not found");
  const auto config = require(task, "config", task_name);
  const auto approach = require(config, "approach", task_name);
  const auto approach_xyz = xyz(require(approach, "xyz", "approach"), "approach.xyz");
  const auto approach_frame = require(approach, "offset_frame", "approach").as<std::string>();
  const auto planning = require(config, "planning", task_name);
  const auto default_planner = planning["first_pick_approach_planner"].as<std::string>("moveJ");

  for (const auto & robot : {std::string("robot1"), std::string("robot2")}) {
    const auto settings = require(config, robot, task_name + ".config");
    const std::string requested_planner =
      settings["first_pick_approach_planner"].as<std::string>(default_planner);
    if (requested_planner != "moveJ")
      throw std::runtime_error(robot + " P1 is " + requested_planner +
        "; this combined executor supports joint-space OMPL goals only");
    const auto pairs = require(settings, "pairs", robot);
    if (!pairs.IsSequence() || pair_index >= pairs.size())
      throw std::runtime_error(robot + ": pair_index out of range");
    const auto pair = pairs[pair_index];
    auto target = framePose(tf, require(pair, "pick_frame", robot).as<std::string>());
    // shear_links_pp.cpp: first add pick_offset in WORLD, then approach offset.
    target = offsetPose(target, xyz(require(pair, "pick_offset", robot), "pick_offset"), "world");
    target = offsetPose(target, approach_xyz, approach_frame);
    setPoseIK(goal, armInfo(workcell, robot), target, used, ik_timeout);
  }
}

void applyOperation(const YAML::Node & op, const YAML::Node & tasks,
                    const YAML::Node & workcell, tf2_ros::Buffer & tf,
                    moveit::core::RobotState & goal,
                    std::set<std::string> & used, double ik_timeout)
{
  if (op["task"]) {
    const auto name = op["task"].as<std::string>();
    const auto step = require(op, "step", name).as<std::string>();
    const auto task = require(tasks, name, "tasks");
    const auto type = require(task, "type", name).as<std::string>();
    if (type == "flange_pp") {
      applyFlangeH1(tasks, workcell, name, step, goal, used);
    } else if (type == "shear_links_pp") {
      const int idx = op["pair_index"].as<int>(0);
      if (idx < 0) throw std::runtime_error("pair_index must be nonnegative");
      applyShearP1(tasks, workcell, name, step, static_cast<size_t>(idx),
                   tf, goal, used, ik_timeout);
    } else {
      throw std::runtime_error("Task resolver not implemented for " + name + " (" + type + ")");
    }
    return;
  }

  const auto robot = require(op, "robot", "operation").as<std::string>();
  const auto a = armInfo(workcell, robot);
  const auto type = require(op, "type", "operation").as<std::string>();
  if (type == "home") {
    setHome(goal, a, used);
  } else if (type == "pose") {
    auto p = framePose(tf, require(op, "frame", robot).as<std::string>());
    p = offsetPose(p, optionalXYZ(op, "xyz"),
                   op["offset_frame"].as<std::string>("world"));
    setPoseIK(goal, a, p, used, ik_timeout);
  } else {
    throw std::runtime_error("Unknown operation type: " + type);
  }
}

// Check full 24-joint states against MoveIt's CURRENT planning scene and ACM.
// Each planned segment is additionally sampled in joint space to bound gaps.
// This is sampled checking, not mathematical continuous-collision checking.
bool validatePath(const rclcpp::Node::SharedPtr & node,
                  const MGI::Plan & plan,
                  const moveit::core::RobotState & start_state,
                  const std::string & group, double max_joint_step)
{
  using Service = moveit_msgs::srv::GetStateValidity;
  auto client = node->create_client<Service>("/check_state_validity");
  if (!client->wait_for_service(15s)) {
    RCLCPP_ERROR(node->get_logger(), "MoveIt /check_state_validity is unavailable");
    return false;
  }
  const auto & jt = plan.trajectory_.joint_trajectory;
  if (jt.joint_names.empty() || jt.points.empty()) return false;
  for (const auto & p : jt.points)
    if (p.positions.size() != jt.joint_names.size()) return false;

  auto check = [&](const std::vector<double> & positions, size_t sample_index) {
    moveit::core::RobotState state(start_state);
    for (size_t j = 0; j < jt.joint_names.size(); ++j)
      state.setVariablePosition(jt.joint_names[j], positions[j]);
    state.update();
    auto req = std::make_shared<Service::Request>();
    req->group_name = group;
    moveit::core::robotStateToRobotStateMsg(state, req->robot_state);
    req->robot_state.attached_collision_objects.clear();
    req->robot_state.is_diff = true;
    auto future = client->async_send_request(req);
    if (future.wait_for(10s) != std::future_status::ready) {
      client->remove_pending_request(future);
      RCLCPP_ERROR(node->get_logger(), "State-validity timeout at sample %zu", sample_index);
      return false;
    }
    const auto response = future.get();
    if (!response->valid) {
      RCLCPP_ERROR(node->get_logger(), "Full-group collision/constraint violation at sample %zu (%zu contacts)",
                   sample_index, response->contacts.size());
      for (size_t i = 0; i < std::min<size_t>(response->contacts.size(), 8); ++i) {
        const auto & c = response->contacts[i];
        RCLCPP_ERROR(node->get_logger(), "Combined contact %zu: %s <-> %s", i + 1,
                     c.contact_body_1.c_str(), c.contact_body_2.c_str());
      }
      return false;
    }
    return true;
  };

  size_t samples = 0;
  const size_t max_samples = 4000;
  if (!check(jt.points.front().positions, ++samples)) return false;
  for (size_t i = 1; i < jt.points.size(); ++i) {
    const auto & prev = jt.points[i-1].positions;
    const auto & next = jt.points[i].positions;
    double largest = 0.0;
    for (size_t j = 0; j < prev.size(); ++j)
      largest = std::max(largest, std::abs(next[j] - prev[j]));
    const auto count = static_cast<size_t>(std::max(1.0, std::ceil(largest / max_joint_step)));
    if (samples + count > max_samples) {
      RCLCPP_ERROR(node->get_logger(), "Path validation exceeds %zu samples; refusing execution", max_samples);
      return false;
    }
    for (size_t k = 1; k <= count; ++k) {
      const double u = static_cast<double>(k) / static_cast<double>(count);
      std::vector<double> values(prev.size());
      for (size_t j = 0; j < prev.size(); ++j)
        values[j] = prev[j] + u * (next[j] - prev[j]);
      if (!check(values, ++samples)) return false;
    }
  }
  RCLCPP_INFO(node->get_logger(), "Combined path passed %zu full-group validity samples", samples);
  return true;
}

// Reusable service checkpoint protocol. A successful Trigger response means
// the motion owner verified its physical/scene state; a failed response means
// not-yet-held or a specific invalid checkpoint.
using CheckpointService = std_srvs::srv::Trigger;

std::shared_ptr<CheckpointService::Response> checkpointRequest(
  const rclcpp::Node::SharedPtr & node, const std::string & service,
  std::chrono::seconds service_wait)
{
  auto client = node->create_client<CheckpointService>(service);
  if (!client->wait_for_service(service_wait))
    throw std::runtime_error("Checkpoint service unavailable: " + service);
  auto request = std::make_shared<CheckpointService::Request>();
  auto future = client->async_send_request(request);  // Humble: non-const .get()
  if (future.wait_for(10s) != std::future_status::ready) {
    client->remove_pending_request(future);
    throw std::runtime_error("Checkpoint service timeout: " + service);
  }
  return future.get();
}

void waitForCheckpoint(const rclcpp::Node::SharedPtr & node,
                       const std::string & service, double timeout_seconds,
                       const std::string & wanted)
{
  if (!std::isfinite(timeout_seconds) || timeout_seconds <= 0.0)
    throw std::runtime_error("Checkpoint timeout must be positive");
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_seconds);
  unsigned int transport_timeouts = 0;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    try {
      const auto status = checkpointRequest(node, service, 5s);
      transport_timeouts = 0;
      if (status->success && status->message.find(wanted) != std::string::npos) {
        RCLCPP_INFO(node->get_logger(), "Checkpoint ready: %s", status->message.c_str());
        return;
      }
      if (status->message.find("INVALID") != std::string::npos)
        throw std::runtime_error("Unsafe checkpoint: " + status->message);
    } catch (const std::runtime_error & e) {
      const std::string reason = e.what();
      // A middleware timeout is not proof that the physical checkpoint is
      // invalid. Retry status reads using a fresh client until the stage's
      // overall deadline. Never advance without a positive verification.
      if (reason.find("Checkpoint service timeout:") != 0 &&
          reason.find("Checkpoint service unavailable:") != 0)
        throw;
      ++transport_timeouts;
      RCLCPP_WARN(node->get_logger(),
        "Checkpoint status transport failure (%u), retrying %s: %s",
        transport_timeouts, service.c_str(), reason.c_str());
    }
    rclcpp::sleep_for(500ms);
  }
  throw std::runtime_error("Timed out waiting for " + wanted + " at " + service);
}

// Gantry handoff after shear: Robot4 is expected to be at home because
// Phase2 now executes H1 before Robot1/2 shear. If already home to tolerance,
// skip redundant 12-joint planning. If not, execute the existing held-Robot3
// collision-validated Robot4 home plan before resuming gantry G5.
int executePhase3Handoff(const rclcpp::Node::SharedPtr & node)
{
  const auto gantry_status = stringParam(node, "gantry_g4_status_service", "/gantry_checkpoint_status");
  const auto gantry_resume = stringParam(node, "gantry_g4_resume_service", "/resume_gantry_motion");
  const auto pickup_status = stringParam(node, "robot3_pick_status_service", "/gantry_robot3_pick_checkpoint_status");
  const auto planner_group = stringParam(node, "phase3_combined_group", "robot3_robot4_arms");
  const auto full_validation_group = stringParam(node, "phase3_collision_group", "sa3_all_robots");
  const auto robot3_group = stringParam(node, "robot3_group", "robot3_arm");
  const auto robot4_group = stringParam(node, "robot4_group", "robot4_arm");
  const bool allow_execute = boolParam(node, "execute_motion", false);
  const double timeout_sec = doubleParam(node, "phase3_checkpoint_timeout_sec", 240.0);
  const double speed = doubleParam(node, "phase3_home_velocity_scale", 0.20);
  const double accel = doubleParam(node, "phase3_home_acceleration_scale", 0.20);
  const double joint_step = doubleParam(node, "phase3_validation_joint_step_rad", 0.05);
  if (planner_group != "robot3_robot4_arms" || full_validation_group != "sa3_all_robots" ||
      !(speed > 0 && speed <= 1 && accel > 0 && accel <= 1 &&
        joint_step > 0 && joint_step <= 0.2))
    throw std::runtime_error("Invalid Phase3 combined-group/planner configuration");

  // Phase 2 flange node has already exited. G4 continues holding the rebar.
  waitForCheckpoint(node, gantry_status, timeout_sec, "G4_HOLD_VERIFIED");

  MGI combined(node, planner_group);
  combined.setPlanningPipelineId("ompl");
  combined.setPlannerId("RRTConnectkConfigDefault");
  combined.setPlanningTime(20.0);
  combined.setNumPlanningAttempts(10);
  combined.setMaxVelocityScalingFactor(speed);
  combined.setMaxAccelerationScalingFactor(accel);
  combined.clearPoseTargets();
  combined.clearPathConstraints();

  const auto start = combined.getCurrentState(10.0);
  if (!start) throw std::runtime_error("Phase3: combined current robot state missing");
  const auto * flange_jmg = start->getJointModelGroup(planner_group);
  const auto * j3 = start->getJointModelGroup(robot3_group);
  const auto * j4 = start->getJointModelGroup(robot4_group);
  if (!flange_jmg || !j3 || !j4 || flange_jmg->getVariableCount() != 12)
    throw std::runtime_error("Phase3: wrong/missing SRDF flange joint group");

  moveit::core::RobotState goal(*start);
  if (!goal.setToDefaultValues(j4, "home"))
    throw std::runtime_error("Phase3: Robot4 SRDF named home missing");
  goal.update();

  // Check the actual 6-joint feedback before deciding whether Robot4 needs
  // another home motion. Phase2 should already have homed both flange robots.
  bool robot4_already_home = true;
  for (const auto & joint : j4->getVariableNames()) {
    if (std::abs(start->getVariablePosition(joint) - goal.getVariablePosition(joint)) > 0.02) {
      robot4_already_home = false;
      break;
    }
  }
  if (!robot4_already_home && boolParam(node, "require_robot4_already_home", false)) {
    throw std::runtime_error("Early tool-change stage requires Robot4 already at home; refusing to move Robot4 alongside Robot1/2");
  }
  if (robot4_already_home) {
    RCLCPP_INFO(node->get_logger(),
      "[HANDOFF] Robot4 verified at SRDF home (0.02 rad tolerance); skipping redundant home plan");
    if (!allow_execute) {
      RCLCPP_INFO(node->get_logger(), "HANDOFF DRY RUN: Robot4 already home; G5 not resumed");
      return 0;
    }
  } else {
    // Constrain Robot3 to current joints for the entire Robot4 home motion.
    moveit_msgs::msg::Constraints constraints;
    for (const auto & joint : j3->getVariableNames()) {
      moveit_msgs::msg::JointConstraint c;
      c.joint_name = joint;
      c.position = start->getVariablePosition(joint);
      c.tolerance_above = 0.001;
      c.tolerance_below = 0.001;
      c.weight = 1.0;
      constraints.joint_constraints.push_back(c);
    }
    combined.setPathConstraints(constraints);
    moveit_msgs::msg::RobotState start_msg;
    moveit::core::robotStateToRobotStateMsg(*start, start_msg);
    combined.setStartState(start_msg);
    if (!combined.setJointValueTarget(goal))
      throw std::runtime_error("Phase3: Robot4 home target rejected");
    MGI::Plan plan;
    const auto planning_result = combined.plan(plan);
    combined.clearPathConstraints();
    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Phase3: Robot4 home planning failed " +
                               std::to_string(planning_result.val));

    // Explicitly reject any planned motion of Robot 3, even within the holding
    // constraints. The merged group still includes Robot3 joints for execution.
    const auto & jt = plan.trajectory_.joint_trajectory;
    for (const auto & point : jt.points) {
      if (point.positions.size() != jt.joint_names.size())
        throw std::runtime_error("Phase3: invalid combined trajectory dimension");
      for (size_t i = 0; i < jt.joint_names.size(); ++i)
        if (std::find(j3->getVariableNames().begin(), j3->getVariableNames().end(),
                      jt.joint_names[i]) != j3->getVariableNames().end() &&
            std::abs(point.positions[i] - start->getVariablePosition(jt.joint_names[i])) > 0.001)
          throw std::runtime_error("Phase3: Robot3 must remain stationary: " + jt.joint_names[i]);
    }
    if (!validatePath(node, plan, *start, full_validation_group, joint_step))
      throw std::runtime_error("Phase3: Robot4 home failed full-cell sampled collision validation");

    if (!allow_execute) {
      RCLCPP_WARN(node->get_logger(), "PHASE3 DRY RUN PASSED: no home or gantry motion executed");
      return 0;
    }

    const auto fresh = combined.getCurrentState(5.0);
    if (!fresh) throw std::runtime_error("Phase3: cannot verify fresh start state");
    for (const auto & name : flange_jmg->getVariableNames())
      if (std::abs(fresh->getVariablePosition(name) - start->getVariablePosition(name)) > 0.01)
        throw std::runtime_error("Phase3: flange arm moved during planning; refusing execution");

    const auto t0 = std::chrono::steady_clock::now();
    const auto exec_result = combined.execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Phase3: Robot4 combined home execution failed " +
                              std::to_string(exec_result.val));
    RCLCPP_INFO(node->get_logger(), "[PHASE3] Robot4 HOME, Robot3 held; execute only %.3f s",
                std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count());

    const auto current = combined.getCurrentState(5.0);
    if (!current) throw std::runtime_error("Phase3: Robot4 home feedback missing");
    for (const auto & name : j4->getVariableNames())
      if (std::abs(current->getVariablePosition(name) - goal.getVariablePosition(name)) > 0.02)
        throw std::runtime_error("Phase3: Robot4 home not verified: " + name);
  }
  // Check the G4 checkpoint again: flange home could have altered the scene.
  // Rechecking after Robot4 home needs its own retry budget: a single
  // lost service response must not abort an otherwise safe stationary G4.
  waitForCheckpoint(node, gantry_status, 45.0, "G4_HOLD_VERIFIED");
  const auto resumed = checkpointRequest(node, gantry_resume, 5s);
  if (!resumed->success)
    throw std::runtime_error("G4 resume denied: " + resumed->message);
  RCLCPP_INFO(node->get_logger(), "[PHASE3] G4 resumed; gantry lowering G5, Robot3 R1-R3");
  waitForCheckpoint(node, pickup_status, timeout_sec, "R3_HOLD_VERIFIED");
  RCLCPP_INFO(node->get_logger(), "[HANDOFF] VERIFIED Robot3 holds rebar 1A after shear completion");
  return 0;
}


// Phase 5 coordinator: checkpoint ordering + completion reporting ONLY.
// It never grants exclusive ownership of arm motion.
int executePhase5Parallel(const rclcpp::Node::SharedPtr & node, bool start_from_g4 = false)
{
  if (!start_from_g4) {
    waitForCheckpoint(node, "/gantry_robot3_pick_checkpoint_status",
                      doubleParam(node, "phase5_checkpoint_timeout_sec", 240.0),
                      "R3_HOLD_VERIFIED");
  }
  struct Progress {
    std::mutex mutex;
    bool done_robot3{false};
    bool done_robot12{false};
    bool aborted{false};
  };
  auto progress = std::make_shared<Progress>();
  using Trigger = std_srvs::srv::Trigger;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> services;
  for (const std::string owner : {std::string("robot3"), std::string("robot12")}) {
    services.push_back(node->create_service<Trigger>(
      "/sa3_phase5/" + owner + "_abort",
      [progress, owner](const Trigger::Request::SharedPtr,
                        Trigger::Response::SharedPtr reply) {
        std::lock_guard<std::mutex> lock(progress->mutex);
        progress->aborted = true;
        reply->success = true;
        reply->message = owner + " reported failure";
      }));
    services.push_back(node->create_service<Trigger>(
      "/sa3_phase5/" + owner + "_done",
      [progress, owner](const Trigger::Request::SharedPtr,
                        Trigger::Response::SharedPtr reply) {
        std::lock_guard<std::mutex> lock(progress->mutex);
        reply->success = !progress->aborted;
        if (reply->success) {
          if (owner == "robot3") progress->done_robot3 = true;
          else progress->done_robot12 = true;
        }
        reply->message = reply->success ? "DONE_CONFIRMED" : "ABORTED";
      }));
  }
  RCLCPP_INFO(node->get_logger(),
    "[PHASE5] Parallel Robot3/4 and Robot1/2 execution enabled; NO exclusive motion gate. "
    "Independent trajectory collision checks do not guarantee synchronized separation.");

  if (start_from_g4) {
    if (executePhase3Handoff(node) != 0)
      throw std::runtime_error("Early Phase5: gantry handoff / Robot3+4 pickup failed");
  }

  // Handoff remains mandatory: R4 must never start before successful R3 pick.
  const auto response = checkpointRequest(node, "/resume_robot3_place", 5s);
  if (!response->success)
    throw std::runtime_error("Phase5 Robot3+4 placement resume refused: " + response->message);
  RCLCPP_INFO(node->get_logger(),
    "[PHASE5] Robot3+4 placement resumed; Robot1/2 gripper change can execute concurrently");

  const double budget = doubleParam(node, "phase5_timeout_sec", 1800.0);
  if (!(budget > 0.0 && budget <= 7200.0))
    throw std::runtime_error("phase5_timeout_sec must be in (0,7200]");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(budget);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(progress->mutex);
      if (progress->aborted)
        throw std::runtime_error("Phase5 arm program reported failure");
      if (progress->done_robot3 && progress->done_robot12) {
        RCLCPP_INFO(node->get_logger(), "[PHASE5] Both concurrent workstreams completed");
        return 0;
      }
    }
    std::this_thread::sleep_for(100ms);
  }
  throw std::runtime_error("Phase5 completion timeout; inspect physical cell before recovery");
}

// Phase 6 end: plan any remaining home motions within a single 24-joint
// MoveIt group, validate the entire path, execute, and verify feedback.
int executeAllFourHome(const rclcpp::Node::SharedPtr & node)
{
  MGI all(node, "sa3_all_robots");
  all.setPlanningPipelineId("ompl");
  all.setPlannerId("RRTConnectkConfigDefault");
  all.setPlanningTime(30.0);
  all.setNumPlanningAttempts(10);
  const double speed = doubleParam(node, "final_home_velocity_scale", 0.2);
  const double accel = doubleParam(node, "final_home_acceleration_scale", 0.2);
  if (!(speed > 0 && speed <= 1 && accel > 0 && accel <= 1))
    throw std::runtime_error("Invalid final home scaling");
  all.setMaxVelocityScalingFactor(speed);
  all.setMaxAccelerationScalingFactor(accel);
  auto start = all.getCurrentState(10.0);
  if (!start) throw std::runtime_error("Final home: current state missing");
  const auto * all_jmg = start->getJointModelGroup("sa3_all_robots");
  if (!all_jmg || all_jmg->getVariableCount() != 24)
    throw std::runtime_error("Final home: expected 24 SRDF joints");
  moveit::core::RobotState goal(*start);
  for (const auto & robot : {"robot1", "robot2", "robot3", "robot4"}) {
    const std::string group = std::string(robot) + "_arm";
    const auto * arm = start->getJointModelGroup(group);
    if (!arm || !goal.setToDefaultValues(arm, "home"))
      throw std::runtime_error("Final home: named SRDF home unavailable: " + group);
  }
  goal.update();
  bool already_home = true;
  for (const auto & joint : all_jmg->getVariableNames())
    if (std::abs(start->getVariablePosition(joint) - goal.getVariablePosition(joint)) > 0.02)
      already_home = false;

  if (!already_home) {
    moveit_msgs::msg::RobotState start_msg;
    moveit::core::robotStateToRobotStateMsg(*start, start_msg);
    all.setStartState(start_msg);
    if (!all.setJointValueTarget(goal))
      throw std::runtime_error("Final home joint goal rejected");
    MGI::Plan plan;
    const auto result = all.plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Final all-four home plan failed: " + std::to_string(result.val));
    if (!validatePath(node, plan, *start, "sa3_all_robots", 0.05))
      throw std::runtime_error("Final all-four home path failed full-state collision validation");
    if (!boolParam(node, "execute_motion", false))
      throw std::runtime_error("Final home requires execute_motion:=true or all arms already home");
    const auto fresh = all.getCurrentState(5.0);
    if (!fresh) throw std::runtime_error("Final home fresh robot state unavailable");
    for (const auto & joint : all_jmg->getVariableNames())
      if (std::abs(fresh->getVariablePosition(joint) - start->getVariablePosition(joint)) > 0.01)
        throw std::runtime_error("Final home start changed during validation: " + joint);
    const auto exec = all.execute(plan);
    if (exec != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Final home execution failed " + std::to_string(exec.val));
  }
  const auto final_state = all.getCurrentState(10.0);
  if (!final_state) throw std::runtime_error("Final home feedback missing");
  for (const auto & joint : all_jmg->getVariableNames())
    if (std::abs(final_state->getVariablePosition(joint) - goal.getVariablePosition(joint)) > 0.02)
      throw std::runtime_error("Final home feedback not within 0.02 rad: " + joint);
  RCLCPP_INFO(node->get_logger(), "ALL FOUR ROBOTS VERIFIED AT SRDF HOME (0.02 rad tolerance)");
  return 0;
}

int execute(const rclcpp::Node::SharedPtr & node)
{
  const auto workflow_mode = stringParam(node, "workflow_mode", "");
  if (workflow_mode == "phase3_handoff") return executePhase3Handoff(node);
  if (workflow_mode == "phase5_parallel") return executePhase5Parallel(node);
  if (workflow_mode == "phase3_phase5_parallel") return executePhase5Parallel(node, true);
  if (workflow_mode == "all_four_home") return executeAllFourHome(node);
  if (!workflow_mode.empty() && workflow_mode != "combined_goal")
    throw std::runtime_error("Unknown combined workflow_mode: " + workflow_mode);
  const auto config_path = stringParam(node, "sequence_config_path", "");
  const auto workcell_path = stringParam(node, "workcell_config_path", "");
  const auto selection = stringParam(node, "combined_step", "");
  const bool allow_execute = boolParam(node, "execute_motion", false);
  if (config_path.empty() || workcell_path.empty() || selection.empty())
    throw std::runtime_error("Set sequence_config_path, workcell_config_path, combined_step");

  const auto recipe = YAML::LoadFile(config_path);
  const auto workcell_yaml = YAML::LoadFile(workcell_path);
  const auto motion_sequence = require(recipe, "motion_sequence", "root");
  const auto tasks = require(motion_sequence, "tasks", "motion_sequence");
  const auto combined = require(motion_sequence, "combined_motions", "motion_sequence");
  const auto instruction = require(combined, selection, "combined_motions");
  const auto workcell = require(workcell_yaml, "workcell", "root");
  const std::string group = instruction["group"].as<std::string>("sa3_all_robots");
  if (group != "sa3_all_robots")
    throw std::runtime_error("Only the existing SRDF sa3_all_robots group is supported");
  const double velocity = instruction["velocity_scale"].as<double>(0.2);
  const double acceleration = instruction["acceleration_scale"].as<double>(0.2);
  const double planning_time = instruction["planning_time"].as<double>(20.0);
  const int planning_attempts = instruction["planning_attempts"].as<int>(10);
  const double ik_timeout = instruction["ik_timeout"].as<double>(1.0);
  const double sample_step = instruction["validation_joint_step_rad"].as<double>(0.05);
  if (!(velocity > 0 && velocity <= 1 && acceleration > 0 && acceleration <= 1 &&
        planning_time > 0 && planning_attempts > 0 && ik_timeout > 0 &&
        sample_step > 0 && sample_step <= 0.2))
    throw std::runtime_error("Invalid combined planning settings");

  tf2_ros::Buffer tf(node->get_clock());
  tf2_ros::TransformListener listener(tf, node, true);
  MGI all(node, group);
  all.setPlanningPipelineId("ompl");
  all.setPlannerId(instruction["planner_id"].as<std::string>("RRTConnectkConfigDefault"));
  all.setPlanningTime(planning_time);
  all.setNumPlanningAttempts(planning_attempts);
  all.setMaxVelocityScalingFactor(velocity);
  all.setMaxAccelerationScalingFactor(acceleration);
  all.clearPoseTargets();
  all.clearPathConstraints();

  const auto start = all.getCurrentState(15.0);
  if (!start) throw std::runtime_error("Current robot state unavailable");
  const auto * full_jmg = start->getJointModelGroup(group);
  if (!full_jmg || full_jmg->getVariableCount() != 24)
    throw std::runtime_error("Unexpected all-robot joint model; expected 24 arm joints");
  moveit::core::RobotState goal(*start);
  std::set<std::string> used;
  auto operations = require(instruction, "operations", selection);
  if (!operations.IsSequence() || operations.size() == 0)
    throw std::runtime_error("combined operations must be nonempty list");
  for (const auto & op : operations)
    applyOperation(op, tasks, workcell, tf, goal, used, ik_timeout);
  goal.update();

  // Fix unassigned arms at their captured start configuration throughout the path.
  moveit_msgs::msg::Constraints hold_constraints;
  for (const auto & robot : {std::string("robot1"), std::string("robot2"),
                           std::string("robot3"), std::string("robot4")}) {
    if (used.count(robot)) continue;
    const auto * arm_jmg = start->getJointModelGroup(armInfo(workcell, robot).group);
    if (!arm_jmg) throw std::runtime_error("Missing group for unassigned " + robot);
    for (const auto & joint : arm_jmg->getVariableNames()) {
      moveit_msgs::msg::JointConstraint c;
      c.joint_name = joint;
      c.position = start->getVariablePosition(joint);
      c.tolerance_above = 0.001;
      c.tolerance_below = 0.001;
      c.weight = 1.0;
      hold_constraints.joint_constraints.push_back(c);
    }
  }
  if (!hold_constraints.joint_constraints.empty()) all.setPathConstraints(hold_constraints);

  moveit_msgs::msg::RobotState start_msg;
  moveit::core::robotStateToRobotStateMsg(*start, start_msg);
  all.setStartState(start_msg);
  if (!all.setJointValueTarget(goal))
    throw std::runtime_error("Combined target rejected by MoveIt");
  RCLCPP_INFO(node->get_logger(), "Planning %s in %s with %zu assigned robots",
              selection.c_str(), group.c_str(), used.size());
  MGI::Plan plan;
  const auto planning_result = all.plan(plan);
  all.clearPathConstraints();
  if (planning_result != moveit::core::MoveItErrorCode::SUCCESS)
    throw std::runtime_error("Combined OMPL planning failed, MoveIt code " +
                             std::to_string(planning_result.val));

  // Verify all joint trajectories from the same full-state model. Check every
  // path segment with interpolated full-state collision queries.
  if (!validatePath(node, plan, *start, group, sample_step))
    throw std::runtime_error("Combined path validity check FAILED; nothing executed");
  if (!allow_execute) {
    RCLCPP_WARN(node->get_logger(), "DRY RUN PASSED: set execute_motion:=true to execute");
    return 0;
  }
  // Reject stale starts if another process moved any arm during planning/validation.
  const auto fresh = all.getCurrentState(5.0);
  if (!fresh) throw std::runtime_error("Cannot recheck live state before execute");
  for (const auto & variable : full_jmg->getVariableNames()) {
    if (std::abs(fresh->getVariablePosition(variable) -
                 start->getVariablePosition(variable)) > 0.01)
      throw std::runtime_error("Robot state changed before combined execution: " + variable);
  }
  RCLCPP_WARN(node->get_logger(), "Executing combined motion; ensure no other node commands these arms");
  const auto t0 = std::chrono::steady_clock::now();
  const auto result = all.execute(plan);
  const double seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t0).count();
  if (result != moveit::core::MoveItErrorCode::SUCCESS)
    throw std::runtime_error("Combined execution failed, code " + std::to_string(result.val));
  RCLCPP_INFO(node->get_logger(), "COMBINED EXECUTION SUCCEEDED: %.3f s (planning excluded)", seconds);
  return 0;
}
} // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sa3_combined_motion",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread thread([&executor]() {executor.spin();});
  int rc = 1;
  try {
    rc = execute(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "sa3_combined_motion: %s", e.what());
  }
  executor.cancel();
  thread.join();
  rclcpp::shutdown();
  return rc;
}
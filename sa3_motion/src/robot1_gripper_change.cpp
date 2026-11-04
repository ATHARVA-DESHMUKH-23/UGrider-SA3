// robot1_gripper_change.cpp -- coordinated dual-arm tool change executor.
// source_tool and target_tool are selected by the chosen ms_*.yaml recipe.
// Supports schunk -> tying_gun AND tying_gun -> schunk.
// Motion trajectories and planning-scene attach/detach are synchronized for
// Robot 1 and Robot 2. Physical tool locking/IO is NOT implemented here.
// Test with mock/RViz before hardware.

#include <algorithm>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include "sa3_motion/workcell_program.hpp"
#include "sa3_motion/motion_task.hpp"
#include "sa3_motion/phase5_cell_safety.hpp"

using namespace std::chrono_literals;
using MGI = moveit::planning_interface::MoveGroupInterface;
using JT = trajectory_msgs::msg::JointTrajectory;
using JTP = trajectory_msgs::msg::JointTrajectoryPoint;

namespace
{
constexpr char kReferenceFrame[] = "world";
std::string kCombinedGroup = "sa3_robots_1_2";
double kPlanningTime = 15.0;
int kMoveJPlanRetries = 3, kOmplAttempts = 10;
double kIkTimeout = 1.0;
double kMoveJVelocity = 0.30, kMoveJAcceleration = 0.20;
double kMoveLVelocity = 0.20, kMoveLAcceleration = 0.10;
// Execution mode is independent of MoveIt planning (OMPL / Pilz are unchanged).
std::string kExecutionBackend = "moveit";
std::string kRobot1ControllerAction = "/robot1_controller/follow_joint_trajectory";
std::string kRobot2ControllerAction = "/robot2_controller/follow_joint_trajectory";
double kControllerStartDelaySec = 0.5;
double kControllerResultTimeoutMarginSec = 30.0;
double kControllerStartToleranceRad = 0.05;
std::vector<sa3_yaml::Step> kGripperSteps;
std::string kSourceTool, kTargetTool;

// ---------------------------------------------------------------------------
// Per-robot configuration
// ---------------------------------------------------------------------------
struct RobotConfig
{
  std::string name;              // log prefix
  std::string group;
  std::string planning_link;
  std::string schunk_dock_frame;
  std::string gun_dock_frame;
  std::vector<std::string> schunk_ids;   // collision object IDs
  std::string gun_id;
  std::vector<std::string> arm_links;    // links that may overlap a gripper at its dock
};

RobotConfig gRobot1Config, gRobot2Config;

RobotConfig makeConfig(const std::string & name) {
  if (name == "robot1") return gRobot1Config;
  if (name == "robot2") return gRobot2Config;
  throw std::runtime_error("Unknown robot name for gripper changer: " + name);
}

void loadGripperProgram(const rclcpp::Node::SharedPtr & node) {
  const auto w = sa3_yaml::loadWorkcell(node);
  const auto task = sa3_recipe::loadTask(node, "robot1_gripper_change");
  const auto config = sa3_yaml::require(task, "config", "gripper task");
  const auto robots = sa3_yaml::require(w, "robots", "workcell");
  if (w["world_frame"].as<std::string>("world") != "world")
    throw std::runtime_error("gripper change requires workcell.world_frame=world");
  kCombinedGroup = sa3_yaml::name(robots["combined_groups"]["shear_and_tying_pair"], "combined_group");
  kSourceTool = sa3_yaml::name(task["source_tool"], "source_tool");
  kTargetTool = sa3_yaml::name(task["target_tool"], "target_tool");
  const bool source_valid = kSourceTool == "schunk" || kSourceTool == "tying_gun";
  const bool target_valid = kTargetTool == "schunk" || kTargetTool == "tying_gun";
  if (!source_valid || !target_valid || kSourceTool == kTargetTool)
    throw std::runtime_error("Gripper change requires two different tools: schunk and tying_gun");
  const auto planning = sa3_yaml::require(config, "planning", "gripper task.config");
  kPlanningTime = sa3_yaml::finite(planning["planning_time"], "gripper planning_time");
  kMoveJPlanRetries = planning["movej_plan_retries"].as<int>();
  kOmplAttempts = planning["ompl_attempts"].as<int>();
  kIkTimeout = sa3_yaml::finite(planning["ik_timeout"], "gripper ik_timeout");
  kMoveJVelocity = sa3_yaml::finite(planning["movej_velocity"], "gripper movej_velocity");
  kMoveJAcceleration = sa3_yaml::finite(planning["movej_acceleration"], "gripper movej_acceleration");
  kMoveLVelocity = sa3_yaml::finite(planning["movel_velocity"], "gripper movel_velocity");
  kMoveLAcceleration = sa3_yaml::finite(planning["movel_acceleration"], "gripper movel_acceleration");
  if (kPlanningTime <= 0 || kMoveJPlanRetries < 1 || kOmplAttempts < 1 || kIkTimeout <= 0 ||
      kMoveJVelocity <= 0 || kMoveJVelocity > 1 || kMoveJAcceleration <= 0 || kMoveJAcceleration > 1 ||
      kMoveLVelocity <= 0 || kMoveLVelocity > 1 || kMoveLAcceleration <= 0 || kMoveLAcceleration > 1)
    throw std::runtime_error("Invalid gripper planning settings");
  const auto dock_frames = sa3_yaml::require(w, "tool_dock_frames", "workcell");
  const auto available_tools = sa3_yaml::require(w, "extra_grippers", "workcell");
  auto arm = [&](const char * name, RobotConfig & r) {
    const auto physical = sa3_yaml::require(robots, name, "workcell.robots");
    r.name = name;
    r.group = sa3_yaml::name(physical["group"], std::string(name) + ".group");
    r.planning_link = sa3_yaml::name(physical["tool_mount_link"], "tool_mount_link");
    const std::string prefix(name);
    r.schunk_dock_frame = sa3_yaml::name(
      dock_frames[prefix + "_shuck_dock"]["child"], "schunk_dock_frame");
    r.gun_dock_frame = sa3_yaml::name(
      dock_frames[prefix + "_tyinggun_dock"]["child"], "tyinggun_dock_frame");
    // Discover the Schunk collision objects actually configured in workcell.yaml.
    // The current SHEARGRP.STL combines the gripper into one *_schunk_base object;
    // older workcells can still include separate *_schunk_link7/8 objects.
    const std::string schunk_base = prefix + "_schunk_base";
    if (!available_tools[schunk_base])
      throw std::runtime_error("Missing workcell gripper object: " + schunk_base);
    r.schunk_ids = {schunk_base};
    const std::string schunk_prefix = prefix + "_schunk_";
    std::vector<std::string> additional_schunk_ids;
    for (const auto & entry : available_tools) {
      const auto id = entry.first.as<std::string>();
      if (id != schunk_base && id.compare(0, schunk_prefix.size(), schunk_prefix) == 0)
        additional_schunk_ids.push_back(id);
    }
    std::sort(additional_schunk_ids.begin(), additional_schunk_ids.end());
    r.schunk_ids.insert(r.schunk_ids.end(), additional_schunk_ids.begin(), additional_schunk_ids.end());
    RCLCPP_INFO(node->get_logger(), "[%s] Schunk collision objects configured: %zu",
      prefix.c_str(), r.schunk_ids.size());
    for (const auto & id : r.schunk_ids)
      RCLCPP_INFO(node->get_logger(), "[%s]   %s", prefix.c_str(), id.c_str());
    r.gun_id = prefix + "_tying_gun";
    if (!available_tools[r.gun_id])
      throw std::runtime_error("Missing workcell tying gun: " + r.gun_id);
    r.arm_links = sa3_yaml::strings(physical["gripper_change_arm_touch_links"],
      "workcell.gripper_change_arm_touch_links");
    if (r.arm_links.empty()) throw std::runtime_error("gripper-change arm contact links empty");
  };
  arm("robot1", gRobot1Config);
  arm("robot2", gRobot2Config);
  // An optional per-recipe execution switch. The default preserves existing behavior.
  const auto execution = config["execution"];
  if (execution) {
    if (!execution.IsMap()) throw std::runtime_error("gripper config.execution must be a map");
    kExecutionBackend = execution["executer"].as<std::string>("moveit");
    kRobot1ControllerAction = execution["robot1_action"].as<std::string>(kRobot1ControllerAction);
    kRobot2ControllerAction = execution["robot2_action"].as<std::string>(kRobot2ControllerAction);
    kControllerStartDelaySec = execution["start_delay_sec"].as<double>(kControllerStartDelaySec);
    kControllerResultTimeoutMarginSec = execution["result_timeout_margin_sec"].as<double>(kControllerResultTimeoutMarginSec);
    kControllerStartToleranceRad = execution["start_tolerance_rad"].as<double>(kControllerStartToleranceRad);
  }
  if ((kExecutionBackend != "moveit" && kExecutionBackend != "follow_joint_trajectory") ||
      kRobot1ControllerAction.empty() || kRobot2ControllerAction.empty() ||
      kRobot1ControllerAction == kRobot2ControllerAction ||
      !std::isfinite(kControllerStartDelaySec) || kControllerStartDelaySec < 0.2 || kControllerStartDelaySec > 10.0 ||
      !std::isfinite(kControllerResultTimeoutMarginSec) || kControllerResultTimeoutMarginSec < 1.0 ||
      !std::isfinite(kControllerStartToleranceRad) ||
      kControllerStartToleranceRad <= 0.0 || kControllerStartToleranceRad > 0.2)
    throw std::runtime_error("Invalid gripper config.execution settings");
  kGripperSteps = sa3_yaml::steps(task["steps"], "motion task.gripper.steps");
  // Mandatory and audited scene-action ordering is independent of the direction.
  // Operators can change target tools and approach distances but not silently
  // skip collision restoration or attach/detach ownership transitions.
  if (kGripperSteps.size() != 6)
    throw std::runtime_error("gripper change requires six ordered steps");
  const std::vector<std::string> ref = {
    "source_dock", "source_dock", "source_dock", "target_dock", "target_dock", "target_dock"};
  const std::vector<std::string> motion = {"moveJ", "moveL", "moveL", "moveJ", "moveL", "moveL"};
  const std::vector<std::vector<std::string>> actions = {
    {}, {"detach_source", "allow_source_contacts"},
    {"restore_source_contacts", "allow_target_contacts"}, {},
    {"attach_target"}, {"restore_target_contacts"}};
  for (std::size_t i = 0; i < kGripperSteps.size(); ++i) {
    const auto & step = kGripperSteps[i];
    sa3_yaml::expectedStep(step, ref[i], motion[i], actions[i]);
    if (step.offset_frame != "world" || step.xyz[0] != 0.0 || step.xyz[1] != 0.0)
      throw std::runtime_error("Gripper dock steps must use vertical WORLD Z approach");
    const bool dock = (i == 1 || i == 4);
    if ((dock && step.xyz[2] != 0.0) || (!dock && (step.xyz[2] <= 0.0 || step.xyz[2] > 1.0)))
      throw std::runtime_error("Dock steps must be exact; approach/retract WORLD Z in (0,1] m");
  }
  RCLCPP_INFO(node->get_logger(), "Loaded gripper task %s -> %s (%zu steps)",
    kSourceTool.c_str(), kTargetTool.c_str(), kGripperSteps.size());
}

// ---------------------------------------------------------------------------
// pose <-> tf2 helpers
// ---------------------------------------------------------------------------
tf2::Transform poseToTf(const geometry_msgs::msg::Pose & p)
{
  return tf2::Transform(
    tf2::Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
    tf2::Vector3(p.position.x, p.position.y, p.position.z));
}

geometry_msgs::msg::Pose tfToPose(const tf2::Transform & t)
{
  geometry_msgs::msg::Pose p;
  p.position.x = t.getOrigin().x();
  p.position.y = t.getOrigin().y();
  p.position.z = t.getOrigin().z();
  const auto q = t.getRotation();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

// ---------------------------------------------------------------------------
// Allowed-collision-matrix helpers
// ---------------------------------------------------------------------------
size_t acmIndex(moveit_msgs::msg::AllowedCollisionMatrix & acm, const std::string & name)
{
  for (size_t i = 0; i < acm.entry_names.size(); ++i) {
    if (acm.entry_names[i] == name) {
      return i;
    }
  }
  acm.entry_names.push_back(name);
  const size_t n = acm.entry_names.size();
  for (auto & row : acm.entry_values) {
    row.enabled.push_back(false);
  }
  moveit_msgs::msg::AllowedCollisionEntry new_row;
  new_row.enabled.assign(n, false);
  acm.entry_values.push_back(new_row);
  return n - 1;
}

void acmSetPair(
  moveit_msgs::msg::AllowedCollisionMatrix & acm,
  const std::string & a, const std::string & b, bool allowed)
{
  const size_t ia = acmIndex(acm, a);
  const size_t ib = acmIndex(acm, b);
  acm.entry_values[ia].enabled[ib] = allowed;
  acm.entry_values[ib].enabled[ia] = allowed;
}

// ---------------------------------------------------------------------------
// trajectory merge (two single-arm trajectories -> one 12-joint trajectory)
// ---------------------------------------------------------------------------
double timeOf(const JTP & p) {return rclcpp::Duration(p.time_from_start).seconds();}

// A plan that is already at its goal has zero duration: turn it into a hold.
void ensureDuration(JT & jt)
{
  if (jt.points.empty()) {return;}
  if (jt.points.size() >= 2 && timeOf(jt.points.back()) > 1e-6) {return;}
  JTP p = jt.points.back();
  p.velocities.assign(p.positions.size(), 0.0);
  p.accelerations.assign(p.positions.size(), 0.0);
  p.time_from_start.sec = 0;
  p.time_from_start.nanosec = 0;
  jt.points.clear();
  jt.points.push_back(p);
  p.time_from_start.sec = 1;
  jt.points.push_back(p);
}

// Sample one trajectory at a time on the stretched time axis (factor >= 1).
JTP sampleTrajectory(const JT & trajectory, double scaled_time, double factor)
{
  const auto & points = trajectory.points;
  JTP out;
  if (points.empty()) {return out;}

  const double source_time = scaled_time / factor;
  std::size_t left = 0, right = 0;
  double alpha = 0.0;

  if (source_time >= timeOf(points.back())) {
    left = right = points.size() - 1;
  } else {
    for (std::size_t i = 1; i < points.size(); ++i) {
      if (timeOf(points[i]) >= source_time) {
        left = i - 1;
        right = i;
        const double t0 = timeOf(points[left]);
        const double t1 = timeOf(points[right]);
        alpha = (t1 > t0) ? std::clamp((source_time - t0) / (t1 - t0), 0.0, 1.0) : 0.0;
        break;
      }
    }
  }

  const auto & pa = points[left];
  const auto & pb = points[right];
  auto interpolate = [alpha](const std::vector<double> & va, const std::vector<double> & vb,
      std::vector<double> & o, double scale) {
      if (va.empty() || va.size() != vb.size()) {return false;}
      o.resize(va.size());
      for (std::size_t i = 0; i < va.size(); ++i) {
        o[i] = (va[i] + alpha * (vb[i] - va[i])) * scale;
      }
      return true;
    };

  if (!interpolate(pa.positions, pb.positions, out.positions, 1.0)) {return {};}
  interpolate(pa.velocities, pb.velocities, out.velocities, 1.0 / factor);
  interpolate(pa.accelerations, pb.accelerations, out.accelerations, 1.0 / (factor * factor));
  return out;
}

bool mergeTrajectories(
  const rclcpp::Logger & logger, JT jt1, JT jt2, JT & merged, double & shared_duration)
{
  if (jt1.joint_names.empty() || jt2.joint_names.empty() ||
    jt1.points.empty() || jt2.points.empty())
  {
    RCLCPP_ERROR(logger, "Cannot merge empty trajectories.");
    return false;
  }
  ensureDuration(jt1);
  ensureDuration(jt2);

  const double d1 = timeOf(jt1.points.back());
  const double d2 = timeOf(jt2.points.back());
  if (d1 <= 0.0 || d2 <= 0.0) {
    RCLCPP_ERROR(logger, "A trajectory has invalid duration.");
    return false;
  }

  // Stretch the shorter trajectory to the longer one, sample at the union of times.
  shared_duration = std::max(d1, d2);
  const double f1 = shared_duration / d1;
  const double f2 = shared_duration / d2;

  std::vector<double> times{0.0, shared_duration};
  for (const auto & p : jt1.points) {times.push_back(timeOf(p) * f1);}
  for (const auto & p : jt2.points) {times.push_back(timeOf(p) * f2);}
  std::sort(times.begin(), times.end());
  times.erase(
    std::unique(times.begin(), times.end(), [](double x, double y) {return std::abs(x - y) < 1e-9;}),
    times.end());

  merged = JT();
  merged.header = jt1.header;
  merged.joint_names = jt1.joint_names;
  merged.joint_names.insert(merged.joint_names.end(), jt2.joint_names.begin(), jt2.joint_names.end());

  for (double t : times) {
    const auto p1 = sampleTrajectory(jt1, t, f1);
    const auto p2 = sampleTrajectory(jt2, t, f2);
    if (p1.positions.size() != jt1.joint_names.size() ||
      p2.positions.size() != jt2.joint_names.size())
    {
      RCLCPP_ERROR(logger, "Failed to sample one of the trajectories.");
      return false;
    }
    JTP point;
    point.positions = p1.positions;
    point.positions.insert(point.positions.end(), p2.positions.begin(), p2.positions.end());
    if (p1.velocities.size() == jt1.joint_names.size() &&
      p2.velocities.size() == jt2.joint_names.size())
    {
      point.velocities = p1.velocities;
      point.velocities.insert(point.velocities.end(), p2.velocities.begin(), p2.velocities.end());
    }
    if (p1.accelerations.size() == jt1.joint_names.size() &&
      p2.accelerations.size() == jt2.joint_names.size())
    {
      point.accelerations = p1.accelerations;
      point.accelerations.insert(
        point.accelerations.end(), p2.accelerations.begin(), p2.accelerations.end());
    }
    const auto whole = static_cast<int32_t>(std::floor(t));
    auto nanos = static_cast<int64_t>(std::llround((t - static_cast<double>(whole)) * 1e9));
    point.time_from_start.sec = whole;
    if (nanos >= 1000000000LL) {
      point.time_from_start.sec += 1;
      nanos -= 1000000000LL;
    }
    point.time_from_start.nanosec = static_cast<uint32_t>(nanos);
    merged.points.push_back(std::move(point));
  }
  return true;
}

struct Arm
{
  RobotConfig cfg;
  std::unique_ptr<MGI> mg;
  geometry_msgs::msg::Pose schunk_dock, gun_dock;
};

}  // namespace

// ===========================================================================
// Both robots' gripper change, in lockstep through the combined group
// ===========================================================================
class DualGripperChanger
{
public:
  DualGripperChanger(const rclcpp::Node::SharedPtr & node, tf2_ros::Buffer & tf)
  : node_(node), tf_(tf)
  {
    combined_ = std::make_unique<MGI>(node_, kCombinedGroup);
    combined_->setPoseReferenceFrame(kReferenceFrame);
    combined_->setPlanningTime(kPlanningTime);

    const std::array<std::string, 2> names = {"robot1", "robot2"};
    for (std::size_t i = 0; i < arms_.size(); ++i) {
      arms_[i].cfg = makeConfig(names[i]);
      arms_[i].mg = std::make_unique<MGI>(node_, arms_[i].cfg.group);
      arms_[i].mg->setPoseReferenceFrame(kReferenceFrame);
      arms_[i].mg->setEndEffectorLink(arms_[i].cfg.planning_link);
      arms_[i].mg->setPlanningTime(kPlanningTime);
    }

    phase5_gate_enabled_ = sa3_phase5::enabledParam(node_);
    if (kExecutionBackend == "follow_joint_trajectory") {
      robot1_action_client_ = rclcpp_action::create_client<FollowTrajectory>(node_, kRobot1ControllerAction);
      robot2_action_client_ = rclcpp_action::create_client<FollowTrajectory>(node_, kRobot2ControllerAction);
      logI("DIRECT FJT excuter enabled: R1=" + kRobot1ControllerAction +
        ", R2=" + kRobot2ControllerAction + "; MoveIt is still used for planning");
    } else {
      logI("MoveIt execution excuter enabled (original behavior)");
    }
    get_scene_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
    apply_scene_ =
      node_->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");
  }

  bool run()
  {
    const auto toolIds = [](const Arm & a, const std::string & tool) {
      return tool == "schunk" ? a.cfg.schunk_ids : std::vector<std::string>{a.cfg.gun_id};
    };
    for (auto & a : arms_) {
      if (!lookupPose(a, a.cfg.schunk_dock_frame, a.schunk_dock) ||
          !lookupPose(a, a.cfg.gun_dock_frame, a.gun_dock)) return false;
      const auto source_ids = toolIds(a, kSourceTool);
      const auto target_ids = toolIds(a, kTargetTool);
      if (psi_.getAttachedObjects(source_ids).size() != source_ids.size()) {
        logE(a, "Source tool '" + kSourceTool + "' is not fully attached. Refusing change.");
        return false;
      }
      if (psi_.getObjects(target_ids).size() != target_ids.size()) {
        logE(a, "Target tool '" + kTargetTool + "' is not fully in the world planning scene.");
        return false;
      }
    }
    bool source_contacts_allowed = false;
    bool target_contacts_allowed = false;
    const auto restore = [&]() {
      bool ok = true;
      if (source_contacts_allowed) {
        for (auto & a : arms_)
          ok = setContacts(a, toolIds(a, kSourceTool), false) && ok;
        source_contacts_allowed = false;
      }
      if (target_contacts_allowed) {
        for (auto & a : arms_)
          ok = setContacts(a, toolIds(a, kTargetTool), false) && ok;
        target_contacts_allowed = false;
      }
      return ok;
    };
    sa3_phase5::GateClient phase5_gate(node_, "robot12", phase5_gate_enabled_);
    for (std::size_t step_index = 0; step_index < kGripperSteps.size(); ++step_index) {
      if (!phase5_gate.acquire()) return false;
      const auto & step = kGripperSteps[step_index];
      active_step_index_ = step_index;
      std::array<geometry_msgs::msg::Pose, 2> targets;
      const std::string selected_tool = step.reference == "source_dock" ? kSourceTool : kTargetTool;
      for (std::size_t i = 0; i < arms_.size(); ++i) {
        const auto & dock = selected_tool == "schunk" ? arms_[i].schunk_dock : arms_[i].gun_dock;
        targets[i] = sa3_yaml::offset(dock, step.xyz, step.offset_frame);
      }
      const auto label = step.id + " " + step.description;
      logI("YAML step " + label + " [" + step.motion + "]");
      const bool moved = step.motion == "moveJ" ?
        movejBoth(targets[0], targets[1], label) : movelBoth(targets[0], targets[1], label);
      if (!moved) {
        if (direct_motion_unresolved_) {
          logE("Controller motion status unresolved: NOT changing tool collision state. "
            "Stop controllers and inspect the physical cell before recovery.");
          return false;
        }
        restore(); return false;
      }
      for (const auto & action : step.after_actions) {
        bool ok = true;
        if (action == "detach_source") {
          for (auto & a : arms_) ok = detachTool(a, toolIds(a, kSourceTool)) && ok;
        } else if (action == "allow_source_contacts") {
          source_contacts_allowed = true;
          for (auto & a : arms_) ok = setContacts(a, toolIds(a, kSourceTool), true) && ok;
        } else if (action == "restore_source_contacts") {
          for (auto & a : arms_) ok = setContacts(a, toolIds(a, kSourceTool), false) && ok;
          if (ok) source_contacts_allowed = false;
        } else if (action == "allow_target_contacts") {
          target_contacts_allowed = true;
          for (auto & a : arms_) ok = setContacts(a, toolIds(a, kTargetTool), true) && ok;
        } else if (action == "attach_target") {
          for (auto & a : arms_) ok = attachTool(a, toolIds(a, kTargetTool)) && ok;
        } else if (action == "restore_target_contacts") {
          for (auto & a : arms_) ok = setContacts(a, toolIds(a, kTargetTool), false) && ok;
          if (ok) target_contacts_allowed = false;
        } else {
          logE("Unexpected gripper YAML action " + action); ok = false;
        }
        if (!ok) {
          logE("Action '" + action + "' failed at " + step.id);
          restore();
          return false;
        }
      }
      // Keep the cell locked through the final scene restoration, too.
      if (step_index + 1 == kGripperSteps.size() && !restore()) return false;
      if (!phase5_gate.release()) return false;
    }
    if (!phase5_gate.done()) return false;
    logI("combined gripper change completed: " + kSourceTool + " -> " + kTargetTool);
    return true;
  }

  // Reports completed execute() calls only. Planning, TF, scene operations and
  // physical tool-change IO (not implemented) are excluded.
  void printExecutionSummary() const
  {
    double total_success_seconds = 0.0;
    std::printf("\n================ GRIPPER CHANGE EXECUTION-ONLY TIME ================\n");
    std::printf("Tool change: %s -> %s (both arms, combined execution)\n",
      kSourceTool.c_str(), kTargetTool.c_str());
    std::printf("%-5s %-36s %-8s %12s\n", "STEP", "DESCRIPTION / PLANNER", "STATUS", "EXEC TIME");
    for (std::size_t i = 0; i < kGripperSteps.size(); ++i) {
      const auto & step = kGripperSteps[i];
      const auto & timing = execution_times_[i];
      const char * status = timing.succeeded ? "OK" :
        timing.attempted ? "FAILED" : "NOT RUN";
      if (timing.succeeded) total_success_seconds += timing.seconds;
      const std::string description = step.description + " [" + step.motion + "]";
      std::printf("%-5s %-36.36s %-8s %9.3f s\n",
        step.id.c_str(), description.c_str(), status, timing.seconds);
    }
    std::printf("TOTAL SUCCESSFUL EXECUTION TIME: %.3f s\n", total_success_seconds);
    std::printf("Robot1 and Robot2 move together; time is counted once per combined execution.\n");
    std::printf("Planning, TF lookups, collision-scene operations and tool IO excluded.\n");
    std::printf("====================================================================\n");
    std::fflush(stdout);
  }

private:
  struct StepExecutionTime {
    double seconds = 0.0;
    bool attempted = false;
    bool succeeded = false;
  };
  std::array<StepExecutionTime, 6> execution_times_{};
  std::size_t active_step_index_ = 0;

  // ---- logging ------------------------------------------------------------
  void logI(const std::string & m) {RCLCPP_INFO(node_->get_logger(), "[dual] %s", m.c_str());}
  void logW(const std::string & m) {RCLCPP_WARN(node_->get_logger(), "[dual] %s", m.c_str());}
  void logE(const std::string & m) {RCLCPP_ERROR(node_->get_logger(), "[dual] %s", m.c_str());}
  void logI(const Arm & a, const std::string & m)
  {
    RCLCPP_INFO(node_->get_logger(), "[%s] %s", a.cfg.name.c_str(), m.c_str());
  }
  void logE(const Arm & a, const std::string & m)
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] %s", a.cfg.name.c_str(), m.c_str());
  }

  // ---- scene: detach / attach ---------------------------------------------
  bool detachTool(Arm & a, const std::vector<std::string> & ids)
  {
    // Verify ownership first: a missing part must not be ignored.
    if (psi_.getAttachedObjects(ids).size() != ids.size()) {
      logE(a, "Cannot detach tool: not all parts attached"); return false;
    }
    std::vector<moveit_msgs::msg::AttachedCollisionObject> removals;
    for (const auto & id : ids) {
      moveit_msgs::msg::AttachedCollisionObject removal;
      removal.link_name = a.cfg.planning_link;
      removal.object.id = id;
      removal.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      removals.push_back(removal);
    }
    if (!psi_.applyAttachedCollisionObjects(removals)) {
      logE(a, "Failed to request tool detach"); return false;
    }
    for (int i = 0; i < 50; ++i) {
      if (psi_.getAttachedObjects(ids).empty() &&
          psi_.getObjects(ids).size() == ids.size()) {
        logI(a, "detached " + std::to_string(ids.size()) + " tool object(s).");
        return true;
      }
      rclcpp::sleep_for(100ms);
    }
    logE(a, "Tool parts did not detach into world scene.");
    return false;
  }

  bool attachTool(Arm & a, const std::vector<std::string> & ids)
  {
    // Preserve all part poses while transferring ownership from world to mount.
    const auto world = psi_.getObjects(ids);
    if (world.size() != ids.size()) {
      logE(a, "Cannot attach tool: one or more world objects missing"); return false;
    }
    const auto mount_pose = combined_->getCurrentPose(a.cfg.planning_link).pose;
    const tf2::Transform inverse = poseToTf(mount_pose).inverse();
    std::vector<moveit_msgs::msg::AttachedCollisionObject> attachments;
    for (const auto & id : ids) {
      const auto it = world.find(id);
      if (it == world.end()) return false;
      auto object = it->second;
      object.pose = tfToPose(inverse * poseToTf(object.pose));
      object.header.frame_id = a.cfg.planning_link;
      object.operation = moveit_msgs::msg::CollisionObject::ADD;
      moveit_msgs::msg::AttachedCollisionObject attached;
      attached.link_name = a.cfg.planning_link;
      attached.object = object;
      attached.touch_links = a.cfg.arm_links;
      attachments.push_back(attached);
    }
    psi_.removeCollisionObjects(ids);
    if (!psi_.applyAttachedCollisionObjects(attachments)) {
      logE(a, "Failed to request tool attach"); return false;
    }
    for (int i = 0; i < 50; ++i) {
      if (psi_.getAttachedObjects(ids).size() == ids.size()) {
        logI(a, "attached " + std::to_string(ids.size()) + " tool object(s).");
        return true;
      }
      rclcpp::sleep_for(100ms);
    }
    logE(a, "Tool parts did not attach."); return false;
  }

  // ---- ACM ----------------------------------------------------------------
  std::optional<moveit_msgs::msg::AllowedCollisionMatrix> fetchAcm()
  {
    auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    req->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto fut = get_scene_->async_send_request(req);
    if (fut.wait_for(5s) != std::future_status::ready) {
      logE("get_planning_scene timed out.");
      return std::nullopt;
    }
    return fut.get()->scene.allowed_collision_matrix;
  }

  // Allow (true) or restore (false) collisions between each object and each
  // arm link of this robot, and between the objects themselves.
  bool setContacts(const Arm & a, const std::vector<std::string> & ids, bool allowed)
  {
    if (!get_scene_->wait_for_service(5s) || !apply_scene_->wait_for_service(5s)) {
      logE(a, "planning scene services not available.");
      return false;
    }

    auto acm = fetchAcm();
    if (!acm) {return false;}

    for (const auto & id : ids) {
      for (const auto & link : a.cfg.arm_links) {
        acmSetPair(*acm, id, link, allowed);
      }
      for (const auto & other : ids) {
        if (other != id) {
          acmSetPair(*acm, id, other, allowed);
        }
      }
    }

    auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    req->scene.is_diff = true;
    req->scene.allowed_collision_matrix = *acm;
    auto fut = apply_scene_->async_send_request(req);
    if (fut.wait_for(5s) != std::future_status::ready || !fut.get()->success) {
      logE(a, "apply_planning_scene failed.");
      return false;
    }
    logI(a, std::string("collisions ") + (allowed ? "ALLOWED" : "RESTORED") + " for " +
      std::to_string(ids.size()) + " object(s).");
    return true;
  }

  // ---- TF -----------------------------------------------------------------
  bool lookupPose(const Arm & a, const std::string & frame, geometry_msgs::msg::Pose & pose)
  {
    try {
      const auto t = tf_.lookupTransform(
        kReferenceFrame, frame, tf2::TimePointZero, tf2::durationFromSec(5.0));
      pose.position.x = t.transform.translation.x;
      pose.position.y = t.transform.translation.y;
      pose.position.z = t.transform.translation.z;
      pose.orientation = t.transform.rotation;
      logI(a, "TF '" + frame + "' resolved in '" + kReferenceFrame + "'.");
      return true;
    } catch (const tf2::TransformException & ex) {
      logE(a, "TF lookup failed for '" + frame + "': " + ex.what());
      return false;
    }
  }

  geometry_msgs::msg::Pose offsetZ(const geometry_msgs::msg::Pose & original, double dz) const
  {
    auto result = original;
    result.position.z += dz;   // WORLD Z
    return result;
  }

  // ---- execution ------------------------------------------------------------
  using FollowTrajectory = control_msgs::action::FollowJointTrajectory;
  using FjtGoalHandle = rclcpp_action::ClientGoalHandle<FollowTrajectory>;

  // Extract exactly the joints owned by one controller. Do not silently send
  // a combined 12-joint command to a 6-joint trajectory controller.
  bool splitForControllers(
    const JT & merged, JT & r1, JT & r2, const std::string & label)
  {
    if (merged.points.size() < 2 || merged.joint_names.empty()) {
      logE(label + ": empty/short planned trajectory"); return false;
    }
    std::array<JT *, 2> outputs{{&r1, &r2}};
    std::array<std::vector<std::size_t>, 2> indices;
    std::unordered_map<std::string, std::size_t> index_by_name;
    for (std::size_t j = 0; j < merged.joint_names.size(); ++j) {
      if (!index_by_name.emplace(merged.joint_names[j], j).second) {
        logE(label + ": duplicate trajectory joint: " + merged.joint_names[j]); return false;
      }
    }
    for (std::size_t arm = 0; arm < 2; ++arm) {
      auto & output = *outputs[arm];
      output.header = merged.header;
      const auto expected = arms_[arm].mg->getActiveJoints();
      if (expected.empty()) {logE("Empty active joints for " + arms_[arm].cfg.name); return false;}
      std::unordered_set<std::string> unique_expected;
      for (const auto & joint : expected) {
        if (!unique_expected.insert(joint).second) return false;
        const auto it = index_by_name.find(joint);
        if (it == index_by_name.end()) {
          logE(label + ": missing joint " + joint); return false;
        }
        output.joint_names.push_back(joint);
        indices[arm].push_back(it->second);
      }
    }
    if (indices[0].size() + indices[1].size() != merged.joint_names.size()) {
      logE(label + ": trajectory contains unexpected joints or overlap between controllers");
      return false;
    }
    for (const auto & point : merged.points) {
      const auto n = merged.joint_names.size();
      if (point.positions.size() != n ||
          (!point.velocities.empty() && point.velocities.size() != n) ||
          (!point.accelerations.empty() && point.accelerations.size() != n) ||
          (!point.effort.empty() && point.effort.size() != n)) {
        logE(label + ": incomplete trajectory point fields"); return false;
      }
      for (std::size_t arm = 0; arm < 2; ++arm) {
        JTP p;
        p.time_from_start = point.time_from_start;
        for (const auto i : indices[arm]) {
          if (!std::isfinite(point.positions[i]) ||
              (!point.velocities.empty() && !std::isfinite(point.velocities[i])) ||
              (!point.accelerations.empty() && !std::isfinite(point.accelerations[i])) ||
              (!point.effort.empty() && !std::isfinite(point.effort[i]))) {
            logE(label + ": nonfinite trajectory value"); return false;
          }
          p.positions.push_back(point.positions[i]);
          if (!point.velocities.empty()) p.velocities.push_back(point.velocities[i]);
          if (!point.accelerations.empty()) p.accelerations.push_back(point.accelerations[i]);
          if (!point.effort.empty()) p.effort.push_back(point.effort[i]);
        }
        outputs[arm]->points.push_back(std::move(p));
      }
    }
    double last = -1.0;
    for (const auto & p : merged.points) {
      const double t = rclcpp::Duration(p.time_from_start).seconds();
      if (!std::isfinite(t) || t <= last || t < 0.0) {
        logE(label + ": invalid or nonincreasing trajectory timestamps"); return false;
      }
      last = t;
    }
    return true;
  }

  // Compare both controller start states before dispatch, using the same live
  // MoveIt robot state that planning relies on. No direct command if it drifted.
  bool verifyDirectStart(const JT & t1, const JT & t2, const std::string & label)
  {
    const auto state = combined_->getCurrentState(5.0);
    if (!state) return false;
    for (const auto * traj : {&t1, &t2}) {
      for (std::size_t i = 0; i < traj->joint_names.size(); ++i) {
        const double actual = state->getVariablePosition(traj->joint_names[i]);
        const double expected = traj->points.front().positions[i];
        if (!std::isfinite(actual) || std::abs(actual - expected) > kControllerStartToleranceRad) {
          logE(label + ": start joint drift at " + traj->joint_names[i] +
            " current=" + std::to_string(actual) + " planned=" + std::to_string(expected));
          return false;
        }
      }
    }
    return true;
  }

  bool executeDirectControllers(const MGI::Plan & plan, const std::string & label)
  {
    if (!robot1_action_client_ || !robot2_action_client_) return false;
    if (!plan.trajectory_.multi_dof_joint_trajectory.joint_names.empty()) {
      logE(label + ": multi-DOF trajectory not supported by direct controllers"); return false;
    }
    JT t1, t2;
    if (!splitForControllers(plan.trajectory_.joint_trajectory, t1, t2, label)) return false;
    if (!robot1_action_client_->wait_for_action_server(5s) ||
        !robot2_action_client_->wait_for_action_server(5s)) {
      logE(label + ": Robot 1/2 FollowJointTrajectory action server unavailable"); return false;
    }
    if (!verifyDirectStart(t1, t2, label)) return false;

    // Both controllers receive an identical future timestamp, so transport
    // latency between the two send requests does not skew their start times.
    const auto start_stamp = node_->now() + rclcpp::Duration::from_seconds(kControllerStartDelaySec);
    t1.header.stamp = start_stamp;
    t2.header.stamp = start_stamp;
    FollowTrajectory::Goal goal1, goal2;
    goal1.trajectory = t1;
    goal2.trajectory = t2;
    auto start = std::chrono::steady_clock::now();
    direct_motion_unresolved_ = true;
    auto f1 = robot1_action_client_->async_send_goal(goal1);
    auto f2 = robot2_action_client_->async_send_goal(goal2);
    using namespace std::chrono_literals;
    const auto accept_deadline = std::chrono::steady_clock::now() + 10s;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < accept_deadline &&
           (f1.wait_for(0ms) != std::future_status::ready ||
            f2.wait_for(0ms) != std::future_status::ready)) {
      std::this_thread::sleep_for(20ms);
    }
    const bool a1_ready = f1.wait_for(0ms) == std::future_status::ready;
    const bool a2_ready = f2.wait_for(0ms) == std::future_status::ready;
    auto h1 = a1_ready ? f1.get() : nullptr;
    auto h2 = a2_ready ? f2.get() : nullptr;
    auto cancel_both = [&]() {
      if (h1) robot1_action_client_->async_cancel_goal(h1);
      if (h2) robot2_action_client_->async_cancel_goal(h2);
    };
    if (!h1 || !h2) {
      cancel_both();
      // If only one goal was accepted, wait for its cancel result. Otherwise
      // remain in the unresolved state and do not mutate the scene on failure.
      const auto stop_deadline = std::chrono::steady_clock::now() + 5s;
      if (h1 && !h2) {
        auto stopped = robot1_action_client_->async_get_result(h1);
        while (rclcpp::ok() && stopped.wait_for(20ms) != std::future_status::ready &&
               std::chrono::steady_clock::now() < stop_deadline) {}
        if (stopped.wait_for(0ms) == std::future_status::ready && a2_ready)
          direct_motion_unresolved_ = false;
      } else if (h2 && !h1) {
        auto stopped = robot2_action_client_->async_get_result(h2);
        while (rclcpp::ok() && stopped.wait_for(20ms) != std::future_status::ready &&
               std::chrono::steady_clock::now() < stop_deadline) {}
        if (stopped.wait_for(0ms) == std::future_status::ready && a1_ready)
          direct_motion_unresolved_ = false;
      } else if (a1_ready && a2_ready) {
        direct_motion_unresolved_ = false;
      }
      logE(label + ": controller goal was rejected or timed out; cancel requested for accepted goals");
      return false;
    }

    auto r1 = robot1_action_client_->async_get_result(h1);
    auto r2 = robot2_action_client_->async_get_result(h2);
    const double duration = std::max(
      rclcpp::Duration(t1.points.back().time_from_start).seconds(),
      rclcpp::Duration(t2.points.back().time_from_start).seconds());
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(duration + kControllerStartDelaySec + kControllerResultTimeoutMarginSec);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      const bool d1 = r1.wait_for(0ms) == std::future_status::ready;
      const bool d2 = r2.wait_for(0ms) == std::future_status::ready;
      if (d1 && d2) break;
      auto succeeded = [](const FjtGoalHandle::WrappedResult & w) {
        return w.code == rclcpp_action::ResultCode::SUCCEEDED && w.result &&
          w.result->error_code == FollowTrajectory::Result::SUCCESSFUL;
      };
      if ((d1 && !succeeded(r1.get())) || (d2 && !succeeded(r2.get()))) break;
      std::this_thread::sleep_for(20ms);
    }
    const bool d1 = r1.wait_for(0ms) == std::future_status::ready;
    const bool d2 = r2.wait_for(0ms) == std::future_status::ready;
    if (!d1 || !d2) {
      cancel_both();
      const auto stop_deadline = std::chrono::steady_clock::now() + 5s;
      while (rclcpp::ok() && std::chrono::steady_clock::now() < stop_deadline &&
             (r1.wait_for(0ms) != std::future_status::ready ||
              r2.wait_for(0ms) != std::future_status::ready)) {
        std::this_thread::sleep_for(20ms);
      }
      if (r1.wait_for(0ms) == std::future_status::ready &&
          r2.wait_for(0ms) == std::future_status::ready) direct_motion_unresolved_ = false;
      logE(label + ": controller result timeout or peer abort; requested both goal cancellations");
      return false;
    }
    direct_motion_unresolved_ = false;
    const auto w1 = r1.get();
    const auto w2 = r2.get();
    const bool ok1 = w1.code == rclcpp_action::ResultCode::SUCCEEDED && w1.result &&
      w1.result->error_code == FollowTrajectory::Result::SUCCESSFUL;
    const bool ok2 = w2.code == rclcpp_action::ResultCode::SUCCEEDED && w2.result &&
      w2.result->error_code == FollowTrajectory::Result::SUCCESSFUL;
    if (!ok1 || !ok2) {
      cancel_both();
      logE(label + ": controller failure (robot1=" + std::to_string(static_cast<int>(w1.code)) +
        ", robot2=" + std::to_string(static_cast<int>(w2.code)) + ")");
      return false;
    }
    logI(label + ": both FollowJointTrajectory controllers succeeded in " +
      std::to_string(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count()) + " s");
    return true;
  }

  bool executeCombined(MGI::Plan & plan, const std::string & label)
  {
    if (phase5_gate_enabled_ || kExecutionBackend == "follow_joint_trajectory") {
      auto state = combined_->getCurrentState(5.0);
      if (!state || !sa3_phase5::validateFullArmTrajectory(
          node_, plan.trajectory_, *state, label)) {
        logE("Full-arm sampled collision validation failed for " + label);
        return false;
      }
      const auto fresh = combined_->getCurrentState(5.0);
      if (!fresh || !sa3_phase5::fullArmStartStillCurrent(
          node_, *state, *fresh, label)) {
        logE("Cell arm state changed before gripper execute for " + label);
        return false;
      }
    }
    // Wall-clock time of the blocking execution call only (no planning / TF / scene updates).
    const auto started = std::chrono::steady_clock::now();
    bool success = false;
    int moveit_result_code = 0;
    if (kExecutionBackend == "follow_joint_trajectory") {
      success = executeDirectControllers(plan, label);
    } else {
      const auto r = combined_->execute(plan);
      moveit_result_code = r.val;
      success = (r == moveit::core::MoveItErrorCode::SUCCESS);
    }
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();
    auto & timing = execution_times_.at(active_step_index_);
    timing.seconds = elapsed;
    timing.attempted = true;
    timing.succeeded = success;
    if (!timing.succeeded) {
      logE("combined execution failed for " + label + " (excuter " + kExecutionBackend +
        ", MoveIt code " + std::to_string(moveit_result_code) +
        ", execute wall time " + std::to_string(elapsed) + " s).");
      return false;
    }
    logI("[EXEC ONLY] " + label + ": " + std::to_string(elapsed) +
      " s (Robot1+Robot2, excuter=" + kExecutionBackend + ")");
    return true;
  }

  // ---- MOVEJ: simple combined planning --------------------------------------
  // IK for each tool mount on the SAME RobotState, then one combined OMPL goal.
  bool movejBoth(
    const geometry_msgs::msg::Pose & target1, const geometry_msgs::msg::Pose & target2,
    const std::string & label)
  {
    logI("[MOVEJ combined] " + label);
    auto & mg = *combined_;

    for (int attempt = 1; attempt <= kMoveJPlanRetries; ++attempt) {
      auto state = mg.getCurrentState(10.0);
      if (!state) {
        logE("[MOVEJ] current combined robot state unavailable.");
        return false;
      }
      const auto * group1 = state->getJointModelGroup(arms_[0].cfg.group);
      const auto * group2 = state->getJointModelGroup(arms_[1].cfg.group);
      if (!group1 || !group2) {
        logE("[MOVEJ] SRDF joint model groups '" + arms_[0].cfg.group + "' / '" +
          arms_[1].cfg.group + "' unavailable.");
        return false;
      }
      const bool ik1 = state->setFromIK(group1, target1, arms_[0].cfg.planning_link, kIkTimeout);
      const bool ik2 = ik1 &&
        state->setFromIK(group2, target2, arms_[1].cfg.planning_link, kIkTimeout);
      if (!ik1 || !ik2) {
        logW("[MOVEJ] " + label + " IK attempt " + std::to_string(attempt) + "/" +
          std::to_string(kMoveJPlanRetries) + " failed (robot1=" + (ik1 ? "OK" : "FAIL") +
          ", robot2=" + (ik2 ? "OK" : "FAIL") + ").");
        continue;
      }

      mg.setStartStateToCurrentState();
      mg.setPlanningPipelineId("ompl");
      mg.setPlannerId("RRTConnectkConfigDefault");
      mg.setPlanningTime(kPlanningTime);
      mg.setNumPlanningAttempts(kOmplAttempts);
      mg.setMaxVelocityScalingFactor(kMoveJVelocity);
      mg.setMaxAccelerationScalingFactor(kMoveJAcceleration);
      mg.clearPoseTargets();
      mg.clearPathConstraints();

      if (!mg.setJointValueTarget(*state)) {
        logW("[MOVEJ] could not set combined 12-joint target for " + label + ".");
        continue;
      }

      MGI::Plan plan;
      const auto planned = mg.plan(plan);
      if (planned != moveit::core::MoveItErrorCode::SUCCESS) {
        logW("[MOVEJ] combined OMPL attempt " + std::to_string(attempt) + "/" +
          std::to_string(kMoveJPlanRetries) + " failed for " + label + " (code " +
          std::to_string(planned.val) + ").");
        continue;
      }

      if (!executeCombined(plan, label)) {return false;}
      logI("[MOVEJ combined] " + label + " complete.");
      return true;
    }

    logE("[MOVEJ] all combined planning attempts failed for " + label + ".");
    return false;
  }

  // ---- MOVEL: separate LIN per arm, merged, executed through the combined group ----
  bool movelBoth(
    const geometry_msgs::msg::Pose & target1, const geometry_msgs::msg::Pose & target2,
    const std::string & label)
  {
    logI("[MOVEL combined] " + label);

    auto start_state = combined_->getCurrentState(10.0);
    if (!start_state) {
      logE("[MOVEL] current combined robot state unavailable.");
      return false;
    }
    moveit_msgs::msg::RobotState start_msg;
    moveit::core::robotStateToRobotStateMsg(*start_state, start_msg);

    const std::array<const geometry_msgs::msg::Pose *, 2> targets = {&target1, &target2};
    std::array<MGI::Plan, 2> plans;

    // Plan both arms from the same start state; nothing moves unless both succeed.
    for (std::size_t i = 0; i < arms_.size(); ++i) {
      auto & mg = *arms_[i].mg;
      mg.setStartState(start_msg);
      mg.setPlanningPipelineId("pilz_industrial_motion_planner");
      mg.setPlannerId("LIN");
      mg.setPlanningTime(kPlanningTime);
      mg.setNumPlanningAttempts(1);
      mg.setMaxVelocityScalingFactor(kMoveLVelocity);
      mg.setMaxAccelerationScalingFactor(kMoveLAcceleration);
      mg.clearPoseTargets();
      mg.clearPathConstraints();

      if (!mg.setPoseTarget(*targets[i], arms_[i].cfg.planning_link)) {
        logE(arms_[i], "[MOVEL] could not set pose target for " + label + ".");
        mg.clearPoseTargets();
        return false;
      }
      const auto result = mg.plan(plans[i]);
      mg.clearPoseTargets();
      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        logE(arms_[i], "[MOVEL] Pilz LIN planning failed for " + label + " (code " +
          std::to_string(result.val) + "); nothing was executed.");
        return false;
      }
    }

    for (const auto & p : plans) {
      if (!p.trajectory_.multi_dof_joint_trajectory.joint_names.empty()) {
        logE("[MOVEL] multi-DOF trajectories cannot be merged (" + label + ").");
        return false;
      }
    }

    MGI::Plan combined_plan;
    combined_plan.start_state_ = start_msg;
    combined_plan.planning_time_ = plans[0].planning_time_ + plans[1].planning_time_;
    double shared = 0.0;
    if (!mergeTrajectories(
        node_->get_logger(), plans[0].trajectory_.joint_trajectory,
        plans[1].trajectory_.joint_trajectory, combined_plan.trajectory_.joint_trajectory, shared))
    {
      return false;
    }
    logI("[MOVEL] " + label + ": both LIN plans succeeded; executing merged " +
      std::to_string(combined_plan.trajectory_.joint_trajectory.joint_names.size()) +
      "-joint trajectory (" + std::to_string(shared) + " s).");

    if (!executeCombined(combined_plan, label)) {return false;}
    logI("[MOVEL combined] " + label + " complete.");
    return true;
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer & tf_;
  bool phase5_gate_enabled_{false};
  bool direct_motion_unresolved_{false};
  rclcpp_action::Client<FollowTrajectory>::SharedPtr robot1_action_client_;
  rclcpp_action::Client<FollowTrajectory>::SharedPtr robot2_action_client_;

  std::unique_ptr<MGI> combined_;
  std::array<Arm, 2> arms_;
  moveit::planning_interface::PlanningSceneInterface psi_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_;
};

// Optional handoff barrier: G5 is completed (and the gantry rebar has been
// presented) before Robot1/2 start D1. The phase5 gate still forbids motion
// by Robot3 and Robot1/2 at the same instant.
bool waitForG5BeforeToolChange(const rclcpp::Node::SharedPtr & node)
{
  const bool enabled = node->get_parameter("wait_for_gantry_g5").as_bool();
  if (!enabled) return true;
  const auto budget = node->get_parameter("gantry_g5_wait_timeout_sec").as_double();
  if (!(std::isfinite(budget) && budget > 0.0)) return false;
  auto client = node->create_client<std_srvs::srv::Trigger>("/gantry_g5_complete_status");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(budget);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (!client->wait_for_service(1s)) continue;
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(3s) != std::future_status::ready) {
      client->remove_pending_request(future);
      continue;
    }
    const auto result = future.get();
    if (result && result->success && result->message == "G5_COMPLETE") {
      RCLCPP_INFO(node->get_logger(), "G5 complete: gripper change may begin with phase5 gate");
      return true;
    }
    rclcpp::sleep_for(200ms);
  }
  RCLCPP_ERROR(node->get_logger(), "No verified G5 completion; refusing gripper-change motion");
  return false;
}

// ===========================================================================
// main
// ===========================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "dual_gripper_change",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() {executor.spin();});

  bool ok = false;

  try {
    RCLCPP_INFO(node->get_logger(), "Waiting for robot states and TF.");
    rclcpp::sleep_for(2s);

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf_buffer.setUsingDedicatedThread(true);
    tf2_ros::TransformListener tf_listener(tf_buffer, node, false);

    loadGripperProgram(node);
    // Optional early Phase5 mode, defaults false for the existing programs.
    if (!node->has_parameter("wait_for_gantry_g5"))
      node->declare_parameter<bool>("wait_for_gantry_g5", false);
    if (!node->has_parameter("gantry_g5_wait_timeout_sec"))
      node->declare_parameter<double>("gantry_g5_wait_timeout_sec", 240.0);
    DualGripperChanger changer(node, tf_buffer);
    ok = waitForG5BeforeToolChange(node) && changer.run();
    changer.printExecutionSummary();

    (void)tf_listener;
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(node->get_logger(), "Exception: %s", ex.what());
  }

  RCLCPP_INFO(node->get_logger(), "RESULT: both robots %s", ok ? "OK" : "FAILED");

  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
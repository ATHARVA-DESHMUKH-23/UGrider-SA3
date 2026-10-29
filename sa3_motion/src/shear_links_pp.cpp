// shear_links_pp.cpp  (sa3_motion)
//
// Robot 1 + Robot 2 pick/place: SEPARATE planning, COMBINED execution.
//
// Every cycle ("tick") each arm plans its own next motion on its own
// MoveGroupInterface (robot1_arm / robot2_arm) from the same captured start
// state. The two single-arm plans are time-aligned and merged into ONE 12-joint
// trajectory, which is executed through the combined group (sa3_robots_1_2).
// So one arm can be on LIN while the other is on MoveJ - whatever worked for
// each arm is mixed and executed together.
//
// Per-arm stage list (same as robot1_motion / robot2_motion):
//   PICK_APPROACH : OMPL movej
//   PICK          : LIN                    -> then ATTACH rebar
//   PICK_RETRACT  : LIN                    -> restore web_ring / gripper collision
//   TRANSIT       : (pick retract -> place approach) strategy fallback:
//                     1) LIN directly      (5 velocity-scale retries)
//                     2) CIRC via point    (5 velocity-scale retries)
//                     3) OMPL movej        (3 retries)
//   PLACE         : LIN                    -> then DETACH rebar
//   PLACE_RETRACT : LIN                    -> restore collisions, next pair
//
// If one arm has no pairs left while the other still has work, the finished arm
// is given a "hold" trajectory (same pose) so it stays still but is still part
// of the combined execution. When both arms are done, both go HOME together.
//
// If either arm cannot plan its next motion, NOTHING moves for that tick.
//
// Offsets are in the WORLD frame.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "sa3_motion/planning_scene_objects.hpp"
#include "sa3_motion/workcell_program.hpp"

using namespace std::chrono_literals;
using MoveItErr = moveit_msgs::msg::MoveItErrorCodes;
using MGI = moveit::planning_interface::MoveGroupInterface;
using JT = trajectory_msgs::msg::JointTrajectory;
using JTP = trajectory_msgs::msg::JointTrajectoryPoint;

namespace
{

constexpr char kNodeName[] = "shear_links_pp";
// YAML overrides are loaded before constructing MoveGroupInterface objects.
std::string kCombinedGroup = "sa3_robots_1_2";
constexpr char kWorld[] = "world";
std::string kRobot1Group = "robot1_arm", kRobot1Link = "robot1_end_effactor";
std::string kRobot2Group = "robot2_arm", kRobot2Link = "robot2_end_effactor";
std::array<double, 3> kApproachOffset = {0.0, -0.25, 0.0};
std::string kApproachFrame = "world";
std::string kJigClampId = "jig_clamp";
double kPlanningTime = 15.0;
int kOmplAttempts = 10, kPlanRetries = 3;
double kMoveJVelocity = 0.30, kMoveJAcceleration = 0.20;
double kMoveLAcceleration = 0.10, kMoveCAcceleration = 0.10;
std::vector<double> kLinVelScales = {0.10, 0.05, 0.03, 0.02, 0.01};
std::vector<double> kCircVelScales = {0.10, 0.05, 0.03, 0.02, 0.01};
std::vector<std::string> kTransitStrategies = {"moveL", "moveC", "moveJ"};
std::vector<sa3_yaml::Step> kConfiguredStages;
std::string kRobot1Ring = "web_ring", kRobot2Ring = "web_ring_2";
std::vector<std::string> kRobot1GripperLinks, kRobot2GripperLinks;
double kRobot1ViaX = 0.235, kRobot2ViaX = 5.22;
std::vector<std::string> kRobot1PairSpecs, kRobot2PairSpecs;

void loadShearProgram(const rclcpp::Node::SharedPtr & node) {
  const auto w = sa3_yaml::loadWorkcell(node);
  const auto c = sa3_yaml::require(w, "shear_links_pp", "workcell");
  const auto p = sa3_yaml::require(c, "planning", "workcell.shear_links_pp");
  kCombinedGroup = sa3_yaml::name(sa3_yaml::require(c, "combined_group", "shear_links_pp"), "combined_group");
  if (c["world_frame"].as<std::string>("world") != "world") throw std::runtime_error("shear_links_pp world_frame must be world");
  kJigClampId = sa3_yaml::name(sa3_yaml::require(c, "jig_object_id", "shear_links_pp"), "jig_object_id");
  kPlanningTime = sa3_yaml::finite(p["planning_time"], "shear planning_time");
  kOmplAttempts = p["ompl_attempts"].as<int>();
  kPlanRetries = p["plan_retries"].as<int>();
  kMoveJVelocity = sa3_yaml::finite(p["movej_velocity"], "movej_velocity");
  kMoveJAcceleration = sa3_yaml::finite(p["movej_acceleration"], "movej_acceleration");
  kMoveLAcceleration = sa3_yaml::finite(p["movel_acceleration"], "movel_acceleration");
  kMoveCAcceleration = sa3_yaml::finite(p["movec_acceleration"], "movec_acceleration");
  kLinVelScales = sa3_yaml::scales(p["lin_velocity_scales"], "shear LIN scales");
  kCircVelScales = sa3_yaml::scales(p["circ_velocity_scales"], "shear CIRC scales");
  kTransitStrategies = sa3_yaml::strings(p["transit_strategies"], "transit_strategies");
  if (kTransitStrategies != std::vector<std::string>{"moveL", "moveC", "moveJ"})
    throw std::runtime_error("Transit fallback must remain moveL -> moveC -> moveJ for this implementation");
  if (kPlanningTime <= 0 || kOmplAttempts < 1 || kPlanRetries < 1 ||
      kMoveJVelocity <= 0 || kMoveJVelocity > 1 || kMoveJAcceleration <= 0 || kMoveJAcceleration > 1 ||
      kMoveLAcceleration <= 0 || kMoveLAcceleration > 1 || kMoveCAcceleration <= 0 || kMoveCAcceleration > 1)
    throw std::runtime_error("Invalid shear planning settings");
  const auto approach = sa3_yaml::require(c, "approach", "shear_links_pp");
  kApproachOffset = sa3_yaml::xyz(approach["xyz"], "shear approach xyz");
  kApproachFrame = approach["offset_frame"].as<std::string>();
  if (kApproachFrame != "world" && kApproachFrame != "local")
    throw std::runtime_error("shear approach offset_frame must be world/local");

  auto arm = [&](const char * key, std::string & group, std::string & link, std::string & ring,
                 double & via, std::vector<std::string> & touch, std::vector<std::string> & pairs) {
    const auto a = sa3_yaml::require(c, key, "shear_links_pp");
    group = sa3_yaml::name(a["group"], std::string(key) + ".group");
    link = sa3_yaml::name(a["planning_link"], std::string(key) + ".planning_link");
    ring = sa3_yaml::name(a["web_ring"], std::string(key) + ".web_ring");
    via = sa3_yaml::finite(a["via_world_x"], std::string(key) + ".via_world_x");
    touch = sa3_yaml::strings(a["gripper_touch_links"], std::string(key) + ".gripper_touch_links");
    const auto entries = sa3_yaml::require(a, "pairs", key);
    if (!entries.IsSequence()) throw std::runtime_error("shear pairs must be a list");
    pairs.clear();
    for (const auto & entry : entries) {
      const auto pick = sa3_yaml::name(entry["pick_frame"], "pick_frame");
      const auto place = sa3_yaml::name(entry["place_frame"], "place_frame");
      const auto object = sa3_yaml::name(entry["object_id"], "object_id");
      const auto d = sa3_yaml::xyz(entry["pick_offset"], "pick_offset");
      std::ostringstream spec;
      spec.precision(17);
      spec << pick << ',' << place << ',' << object << ',' << d[0] << ',' << d[1] << ',' << d[2];
      pairs.push_back(spec.str());
    }
  };
  arm("robot1", kRobot1Group, kRobot1Link, kRobot1Ring, kRobot1ViaX, kRobot1GripperLinks, kRobot1PairSpecs);
  arm("robot2", kRobot2Group, kRobot2Link, kRobot2Ring, kRobot2ViaX, kRobot2GripperLinks, kRobot2PairSpecs);
  auto sequence = sa3_yaml::require(w, "planner_sequence", "workcell");
  auto program = sa3_yaml::require(sequence, "shear_links_pp", "planner_sequence");
  kConfiguredStages = sa3_yaml::steps(program["stages"], "planner_sequence.shear_links_pp.stages");
  const std::vector<std::string> references = {"pick_approach", "pick", "pick_approach", "place_approach", "place", "place_approach"};
  const std::vector<std::string> motions = {"moveJ", "moveL", "moveL", "fallback", "moveL", "moveL"};
  const std::vector<std::vector<std::string>> actions = {
    {"allow_pick_contact"}, {"attach_rebar", "allow_web_ring_contact"},
    {"restore_pick_contact"}, {"allow_place_contact"}, {"detach_rebar"},
    {"restore_place_contact", "complete_pair"}};
  if (kConfiguredStages.size() != references.size()) throw std::runtime_error("shear requires six ordered stages");
  for (std::size_t i = 0; i < references.size(); ++i) {
    sa3_yaml::expectedStep(kConfiguredStages[i], references[i], motions[i], actions[i]);
    if (kConfiguredStages[i].xyz != std::array<double, 3>{0.,0.,0.} || kConfiguredStages[i].offset_frame != "world")
      throw std::runtime_error("shear stage xyz must be zero: approach is configured under shear_links_pp.approach");
  }
  const auto finish = sa3_yaml::steps(program["finish"], "shear finish");
  if (finish.size()!=1) throw std::runtime_error("shear requires combined home step");
  sa3_yaml::expectedStep(finish[0], "home", "moveJ");
  RCLCPP_INFO(node->get_logger(), "Shear YAML program loaded: robot1=%zu pairs, robot2=%zu pairs", kRobot1PairSpecs.size(), kRobot2PairSpecs.size());
}

// ===========================================================================
// types
// ===========================================================================
enum class Stage
{
  kPickApproach, kPick, kPickRetract, kTransit, kPlace, kPlaceRetract, kDone
};

const char * stageName(Stage s)
{
  switch (s) {
    case Stage::kPickApproach: return "pick approach";
    case Stage::kPick: return "pick";
    case Stage::kPickRetract: return "pick retract";
    case Stage::kTransit: return "transit (pick retract -> place approach)";
    case Stage::kPlace: return "place";
    case Stage::kPlaceRetract: return "place retract";
    default: return "done";
  }
}

struct PairInput
{
  std::string pick;
  std::string place;
  std::string rebar;
  std::array<double, 3> pick_offset = {0.0, 0.0, 0.0};
};

struct Arm
{
  std::string name;
  std::string link;
  std::string web_ring;
  double via_x = 0.0;
  std::vector<std::string> gripper_links;
  MGI * mg = nullptr;

  std::vector<PairInput> pairs;
  std::size_t idx = 0;
  std::size_t completed = 0;
  Stage stage = Stage::kPickApproach;
  bool pair_ready = false;

  geometry_msgs::msg::Pose pick, place, pick_approach, place_approach, via;

  bool done() const {return stage == Stage::kDone;}
  const std::string & rebar() const {return pairs[idx].rebar;}
};

// ===========================================================================
// helpers
// ===========================================================================
geometry_msgs::msg::Pose offsetWorld(
  const geometry_msgs::msg::Pose & p, const std::array<double, 3> & d)
{
  auto r = p;
  r.position.x += d[0];
  r.position.y += d[1];
  r.position.z += d[2];
  return r;
}

std::string poseStr(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  char buf[200];
  std::snprintf(
    buf, sizeof(buf), "xyz=[%.4f, %.4f, %.4f] rpy=[%.4f, %.4f, %.4f]",
    pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw);
  return buf;
}

std::string errStr(int c)
{
  switch (c) {
    case MoveItErr::SUCCESS: return "SUCCESS";
    case MoveItErr::FAILURE: return "FAILURE";
    case MoveItErr::PLANNING_FAILED: return "PLANNING_FAILED";
    case MoveItErr::INVALID_MOTION_PLAN: return "INVALID_MOTION_PLAN";
    case MoveItErr::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case MoveItErr::CONTROL_FAILED: return "CONTROL_FAILED";
    case MoveItErr::TIMED_OUT: return "TIMED_OUT";
    case MoveItErr::PREEMPTED: return "PREEMPTED";
    case MoveItErr::START_STATE_IN_COLLISION: return "START_STATE_IN_COLLISION";
    case MoveItErr::GOAL_IN_COLLISION: return "GOAL_IN_COLLISION";
    case MoveItErr::GOAL_CONSTRAINTS_VIOLATED: return "GOAL_CONSTRAINTS_VIOLATED";
    case MoveItErr::INVALID_GROUP_NAME: return "INVALID_GROUP_NAME";
    case MoveItErr::INVALID_LINK_NAME: return "INVALID_LINK_NAME";
    case MoveItErr::FRAME_TRANSFORM_FAILURE: return "FRAME_TRANSFORM_FAILURE";
    case MoveItErr::ROBOT_STATE_STALE: return "ROBOT_STATE_STALE";
    case MoveItErr::NO_IK_SOLUTION: return "NO_IK_SOLUTION";
    default: return "CODE_" + std::to_string(c);
  }
}

// A slower speed cannot fix these.
bool slowerCannotHelp(int code)
{
  return code == MoveItErr::NO_IK_SOLUTION ||
         code == MoveItErr::GOAL_IN_COLLISION ||
         code == MoveItErr::START_STATE_IN_COLLISION ||
         code == MoveItErr::INVALID_GROUP_NAME ||
         code == MoveItErr::INVALID_LINK_NAME ||
         code == MoveItErr::FRAME_TRANSFORM_FAILURE;
}

std::string tagOf(const Arm & a)
{
  std::ostringstream s;
  s << "[" << a.name << " | pair " << (a.idx + 1) << "/" << a.pairs.size();
  if (a.idx < a.pairs.size()) {
    s << " | " << a.pairs[a.idx].pick << " -> " << a.pairs[a.idx].place
      << " | " << a.pairs[a.idx].rebar;
  }
  s << " | " << stageName(a.stage) << "] ";
  return s.str();
}

bool lookupPose(
  const rclcpp::Logger & logger, tf2_ros::Buffer & tf,
  const std::string & frame, geometry_msgs::msg::Pose & out)
{
  try {
    const auto t = tf.lookupTransform(
      kWorld, frame, tf2::TimePointZero, tf2::durationFromSec(10.0));
    out.position.x = t.transform.translation.x;
    out.position.y = t.transform.translation.y;
    out.position.z = t.transform.translation.z;
    out.orientation = t.transform.rotation;
    return true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(logger, "TF lookup '%s' -> '%s' failed: %s", kWorld, frame.c_str(), e.what());
    return false;
  }
}

// ===========================================================================
// single-arm planning primitives (planning only, nothing is executed here)
// ===========================================================================

// OMPL, pose target, kPlanRetries attempts.
bool planOmpl(
  const rclcpp::Logger & logger, Arm & a, const moveit_msgs::msg::RobotState & start,
  const geometry_msgs::msg::Pose & target, MGI::Plan & plan)
{
  auto & mg = *a.mg;
  int last_code = 0;
  for (int attempt = 1; attempt <= kPlanRetries; ++attempt) {
    mg.setStartState(start);
    mg.setPlanningPipelineId("ompl");
    mg.setPlannerId("RRTConnectkConfigDefault");
    mg.setPlanningTime(kPlanningTime);
    mg.setNumPlanningAttempts(kOmplAttempts);
    mg.setMaxVelocityScalingFactor(kMoveJVelocity);
    mg.setMaxAccelerationScalingFactor(kMoveJAcceleration);
    mg.setGoalPositionTolerance(0.001);
    mg.setGoalOrientationTolerance(0.01);
    mg.clearPoseTargets();
    mg.clearPathConstraints();
    if (!mg.setPoseTarget(target, a.link)) {
      RCLCPP_WARN(logger, "%sOMPL: pose target rejected.", tagOf(a).c_str());
      mg.clearPoseTargets();
      return false;
    }
    const auto r = mg.plan(plan);
    mg.clearPoseTargets();
    last_code = r.val;
    if (r == moveit::core::MoveItErrorCode::SUCCESS) {return true;}
    RCLCPP_WARN(
      logger, "%sOMPL attempt %d/%d failed: %s", tagOf(a).c_str(), attempt, kPlanRetries,
      errStr(last_code).c_str());
  }
  return false;
}

// Pilz LIN, pose target, one attempt per velocity scale.
bool planLin(
  const rclcpp::Logger & logger, Arm & a, const moveit_msgs::msg::RobotState & start,
  const geometry_msgs::msg::Pose & target, MGI::Plan & plan)
{
  auto & mg = *a.mg;
  const int total = static_cast<int>(kLinVelScales.size());
  for (int attempt = 1; attempt <= total; ++attempt) {
    const double scale = kLinVelScales[attempt - 1];
    mg.setStartState(start);
    mg.setPlanningPipelineId("pilz_industrial_motion_planner");
    mg.setPlannerId("LIN");
    mg.setPlanningTime(kPlanningTime);
    mg.setNumPlanningAttempts(1);
    mg.setMaxVelocityScalingFactor(scale);
    mg.setMaxAccelerationScalingFactor(std::min(kMoveLAcceleration, scale));
    mg.clearPoseTargets();
    mg.clearPathConstraints();
    if (!mg.setPoseTarget(target, a.link)) {
      RCLCPP_WARN(logger, "%sLIN: pose target rejected.", tagOf(a).c_str());
      mg.clearPoseTargets();
      return false;
    }
    const auto r = mg.plan(plan);
    mg.clearPoseTargets();
    if (r == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(
        logger, "%sLIN planned on attempt %d/%d (velocity scale %.3f)",
        tagOf(a).c_str(), attempt, total, scale);
      return true;
    }
    RCLCPP_WARN(
      logger, "%sLIN attempt %d/%d failed at velocity scale %.3f: %s",
      tagOf(a).c_str(), attempt, total, scale, errStr(r.val).c_str());
    if (slowerCannotHelp(r.val)) {break;}
  }
  return false;
}

// Pilz CIRC through an interim point, one attempt per velocity scale.
bool planCirc(
  const rclcpp::Logger & logger, Arm & a, const moveit_msgs::msg::RobotState & start,
  const geometry_msgs::msg::Pose & interim, const geometry_msgs::msg::Pose & target,
  MGI::Plan & plan)
{
  auto & mg = *a.mg;

  // Pilz requires exactly one position constraint and the name "interim".
  moveit_msgs::msg::PositionConstraint via_constraint;
  via_constraint.header.frame_id = kWorld;
  via_constraint.link_name = a.link;
  via_constraint.constraint_region.primitive_poses.push_back(interim);
  via_constraint.weight = 1.0;
  moveit_msgs::msg::Constraints path_constraints;
  path_constraints.name = "interim";
  path_constraints.position_constraints.push_back(via_constraint);

  bool planned = false;
  const int total = static_cast<int>(kCircVelScales.size());
  for (int attempt = 1; attempt <= total && !planned; ++attempt) {
    const double scale = kCircVelScales[attempt - 1];
    mg.setStartState(start);
    mg.setPlanningPipelineId("pilz_industrial_motion_planner");
    mg.setPlannerId("CIRC");
    mg.setPlanningTime(kPlanningTime);
    mg.setNumPlanningAttempts(1);
    mg.setMaxVelocityScalingFactor(scale);
    mg.setMaxAccelerationScalingFactor(std::min(kMoveCAcceleration, scale));
    mg.clearPoseTargets();
    mg.clearPathConstraints();
    mg.setPathConstraints(path_constraints);
    if (!mg.setPoseTarget(target, a.link)) {
      RCLCPP_WARN(logger, "%sCIRC: pose target rejected.", tagOf(a).c_str());
      break;
    }
    const auto r = mg.plan(plan);
    if (r == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(
        logger, "%sCIRC planned on attempt %d/%d (velocity scale %.3f)",
        tagOf(a).c_str(), attempt, total, scale);
      planned = true;
      break;
    }
    RCLCPP_WARN(
      logger, "%sCIRC attempt %d/%d failed at velocity scale %.3f: %s",
      tagOf(a).c_str(), attempt, total, scale, errStr(r.val).c_str());
    if (slowerCannotHelp(r.val)) {break;}
  }
  // The interim constraint must never leak into LIN / OMPL.
  mg.clearPathConstraints();
  mg.clearPoseTargets();
  return planned;
}

// Pick retract -> place approach: LIN direct, then CIRC via point, then OMPL.
bool planTransit(
  const rclcpp::Logger & logger, Arm & a, const moveit_msgs::msg::RobotState & start,
  MGI::Plan & plan, std::string & method)
{
  for (const auto & strategy : kTransitStrategies) {
    if (strategy == "moveL" && planLin(logger, a, start, a.place_approach, plan)) {
      method = "LIN direct"; return true;
    }
    if (strategy == "moveC" && planCirc(logger, a, start, a.via, a.place_approach, plan)) {
      method = "CIRC via"; return true;
    }
    if (strategy == "moveJ" && planOmpl(logger, a, start, a.place_approach, plan)) {
      method = "OMPL movej"; return true;
    }
  }
  RCLCPP_ERROR(logger, "%sall three transit strategies failed.", tagOf(a).c_str());
  return false;
}

// Execute the currently configured (validated) SA1-style YAML planner step.
// The fixed six-step stage order is safety-checked during configuration load.
bool planStage(
  const rclcpp::Logger & logger, Arm & a, const moveit_msgs::msg::RobotState & start,
  MGI::Plan & plan, std::string & method)
{
  const auto & step = kConfiguredStages.at(static_cast<std::size_t>(a.stage));
  RCLCPP_INFO(logger, "%sYAML step %s: %s -> %s", tagOf(a).c_str(),
              step.id.c_str(), step.motion.c_str(), step.reference.c_str());
  if (step.motion == "fallback") return planTransit(logger, a, start, plan, method);
  const geometry_msgs::msg::Pose * target = nullptr;
  if (step.reference == "pick") target = &a.pick;
  else if (step.reference == "pick_approach") target = &a.pick_approach;
  else if (step.reference == "place") target = &a.place;
  else if (step.reference == "place_approach") target = &a.place_approach;
  if (target == nullptr) return false;
  if (step.motion == "moveJ") {
    method = "OMPL movej";
    return planOmpl(logger, a, start, *target, plan);
  }
  if (step.motion == "moveL") {
    method = "LIN";
    return planLin(logger, a, start, *target, plan);
  }
  return false;
}

// ===========================================================================
// trajectory merge
// ===========================================================================
double timeOf(const JTP & p) {return rclcpp::Duration(p.time_from_start).seconds();}

// A finished arm: constant pose for 1 s (stretched later to the shared duration).
bool makeHold(Arm & a, const moveit::core::RobotState & state, JT & jt)
{
  jt = JT();
  jt.joint_names = a.mg->getActiveJoints();
  if (jt.joint_names.empty()) {return false;}
  JTP p;
  for (const auto & name : jt.joint_names) {
    p.positions.push_back(state.getVariablePosition(name));
  }
  p.velocities.assign(p.positions.size(), 0.0);
  p.accelerations.assign(p.positions.size(), 0.0);
  p.time_from_start.sec = 0;
  p.time_from_start.nanosec = 0;
  jt.points.push_back(p);
  p.time_from_start.sec = 1;
  jt.points.push_back(p);
  return true;
}

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

// ===========================================================================
// pair setup and scene bookkeeping
// ===========================================================================
bool beginPair(
  const rclcpp::Logger & logger, tf2_ros::Buffer & tf, PlanningSceneObjects & scene, Arm & a)
{
  const auto & pr = a.pairs[a.idx];
  geometry_msgs::msg::Pose pick, place;
  if (!lookupPose(logger, tf, pr.pick, pick) || !lookupPose(logger, tf, pr.place, place)) {
    return false;
  }
  pick.position.x += pr.pick_offset[0];
  pick.position.y += pr.pick_offset[1];
  pick.position.z += pr.pick_offset[2];

  a.pick = pick;
  a.place = place;
  a.pick_approach = sa3_yaml::offset(pick, kApproachOffset, kApproachFrame);
  a.place_approach = sa3_yaml::offset(place, kApproachOffset, kApproachFrame);
  a.via = a.place_approach;
  a.via.position.x = a.via_x;

  if (!scene.verifyObjectState(pr.rebar, true, false)) {
    RCLCPP_ERROR(
      logger, "%sExpected '%s' to be a world object before picking.",
      tagOf(a).c_str(), pr.rebar.c_str());
    return false;
  }
  a.pair_ready = true;
  RCLCPP_INFO(logger, "%s---- pair start ----", tagOf(a).c_str());
  return true;
}

// Scene operations after the stage's motion has been executed, then advance.
bool finishStage(const rclcpp::Logger & logger, PlanningSceneObjects & scene, Arm & a)
{
  const auto & step = kConfiguredStages.at(static_cast<std::size_t>(a.stage));
  const std::string rebar = a.rebar();
  for (const auto & action : step.after_actions) {
    bool ok = true;
    if (action == "allow_pick_contact") {
      ok = scene.setObjectCollisionsAllowed(rebar, a.gripper_links, true);
    } else if (action == "attach_rebar") {
      ok = scene.attachWorldObjectPreservingPose(*a.mg, rebar, a.link, a.gripper_links) &&
        scene.verifyObjectState(rebar, false, true, a.link);
    } else if (action == "allow_web_ring_contact") {
      ok = scene.setCollisionPairAllowed(rebar, a.web_ring, true);
    } else if (action == "restore_pick_contact") {
      ok = scene.setCollisionPairAllowed(rebar, a.web_ring, false) &&
        scene.setObjectCollisionsAllowed(rebar, a.gripper_links, false);
    } else if (action == "allow_place_contact") {
      ok = scene.setObjectCollisionsAllowed(rebar, a.gripper_links, true) &&
        scene.setCollisionPairAllowed(rebar, kJigClampId, true);
    } else if (action == "detach_rebar") {
      ok = scene.detachObjectToWorldPreservingPose(*a.mg, rebar) &&
        scene.verifyObjectState(rebar, true, false);
    } else if (action == "restore_place_contact") {
      ok = scene.setObjectCollisionsAllowed(rebar, a.gripper_links, false) &&
        scene.setCollisionPairAllowed(rebar, kJigClampId, false);
    } else if (action == "complete_pair") {
      ++a.completed;
      ++a.idx;
      a.pair_ready = false;
      a.stage = a.idx < a.pairs.size() ? Stage::kPickApproach : Stage::kDone;
      RCLCPP_INFO(logger, "[%s] completed pair %zu/%zu", a.name.c_str(), a.completed, a.pairs.size());
    } else {
      RCLCPP_ERROR(logger, "Unknown shear YAML action: %s", action.c_str());
      return false;
    }
    if (!ok) {
      RCLCPP_ERROR(logger, "%sYAML action '%s' failed", tagOf(a).c_str(), action.c_str());
      return false;
    }
  }
  // Stage progression is fixed to the validated six-step safety program.
  if (step.after_actions.empty() || step.after_actions.back() != "complete_pair") {
    a.stage = static_cast<Stage>(static_cast<int>(a.stage) + 1);
  }
  return true;
}

// After a failure: release a still-held rebar and clear every temporary allowance.
void recover(PlanningSceneObjects & scene, Arm & a)
{
  if (a.idx >= a.pairs.size()) {return;}
  const std::string rebar = a.rebar();
  if (scene.verifyObjectState(rebar, false, true, a.link)) {
    scene.detachObjectToWorldPreservingPose(*a.mg, rebar);
  }
  scene.setCollisionPairAllowed(rebar, a.web_ring, false);
  scene.setCollisionPairAllowed(rebar, kJigClampId, false);
  scene.setObjectCollisionsAllowed(rebar, a.gripper_links, false);
}

// ===========================================================================
// combined HOME (named state "home" of sa3_robots_1_2)
// ===========================================================================
bool moveCombinedHome(const rclcpp::Logger & logger, MGI & combined)
{
  combined.setStartStateToCurrentState();
  combined.setPlanningPipelineId("ompl");
  combined.setPlannerId("RRTConnectkConfigDefault");
  combined.setPlanningTime(kPlanningTime);
  combined.setNumPlanningAttempts(kOmplAttempts);
  combined.setMaxVelocityScalingFactor(kMoveJVelocity);
  combined.setMaxAccelerationScalingFactor(kMoveJAcceleration);
  combined.clearPoseTargets();
  combined.clearPathConstraints();

  if (!combined.setNamedTarget("home")) {
    RCLCPP_ERROR(logger, "Named state 'home' is not defined for '%s'.", kCombinedGroup.c_str());
    return false;
  }
  MGI::Plan plan;
  for (int attempt = 1; attempt <= kPlanRetries; ++attempt) {
    const auto planned = combined.plan(plan);
    if (planned == moveit::core::MoveItErrorCode::SUCCESS) {
      const auto executed = combined.execute(plan);
      if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Combined HOME execution failed (%s).", errStr(executed.val).c_str());
        return false;
      }
      RCLCPP_INFO(logger, "Both arms reached HOME.");
      return true;
    }
    RCLCPP_WARN(
      logger, "Combined HOME plan attempt %d/%d failed (%s).", attempt, kPlanRetries,
      errStr(planned.val).c_str());
  }
  RCLCPP_ERROR(logger, "Combined HOME planning failed.");
  return false;
}

// ===========================================================================
// main loop
// ===========================================================================
bool runSequence(
  const rclcpp::Node::SharedPtr & node, MGI & combined, tf2_ros::Buffer & tf,
  PlanningSceneObjects & scene, Arm & a1, Arm & a2)
{
  const auto logger = node->get_logger();
  std::array<Arm *, 2> arms = {&a1, &a2};
  int tick = 0;

  while (!(a1.done() && a2.done())) {
    ++tick;

    auto state = combined.getCurrentState(10.0);
    if (!state) {
      RCLCPP_ERROR(logger, "Current combined robot state unavailable.");
      return false;
    }
    moveit_msgs::msg::RobotState start_msg;
    moveit::core::robotStateToRobotStateMsg(*state, start_msg);

    // ---- plan each arm separately from the same start state ----
    std::array<JT, 2> jts;
    std::array<std::string, 2> methods;
    for (std::size_t i = 0; i < arms.size(); ++i) {
      Arm & a = *arms[i];
      if (a.done()) {
        if (!makeHold(a, *state, jts[i])) {
          RCLCPP_ERROR(logger, "[%s] could not build hold trajectory.", a.name.c_str());
          return false;
        }
        methods[i] = "HOLD";
        continue;
      }
      if (a.stage == Stage::kPickApproach && !a.pair_ready && !beginPair(logger, tf, scene, a)) {
        return false;
      }
      MGI::Plan plan;
      if (!planStage(logger, a, start_msg, plan, methods[i])) {
        RCLCPP_ERROR(logger, "%splanning failed; nothing was executed this tick.",
          tagOf(a).c_str());
        return false;
      }
      if (!plan.trajectory_.multi_dof_joint_trajectory.joint_names.empty()) {
        RCLCPP_ERROR(logger, "%smulti-DOF trajectory cannot be merged.", tagOf(a).c_str());
        return false;
      }
      jts[i] = plan.trajectory_.joint_trajectory;
    }

    // ---- merge into one 12-joint trajectory and execute through the combined group ----
    MGI::Plan combined_plan;
    combined_plan.start_state_ = start_msg;
    double shared = 0.0;
    if (!mergeTrajectories(logger, jts[0], jts[1], combined_plan.trajectory_.joint_trajectory,
        shared))
    {
      return false;
    }
    RCLCPP_INFO(
      logger, "tick %d: robot1 [%s | %s]  +  robot2 [%s | %s]  -> merged %zu joints, %.3f s",
      tick, a1.done() ? "done" : stageName(a1.stage), methods[0].c_str(),
      a2.done() ? "done" : stageName(a2.stage), methods[1].c_str(),
      combined_plan.trajectory_.joint_trajectory.joint_names.size(), shared);

    const auto executed = combined.execute(combined_plan);
    if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Combined execution failed (%s).", errStr(executed.val).c_str());
      return false;
    }

    // ---- scene operations / stage advance for every arm that moved ----
    for (Arm * a : arms) {
      if (a->done()) {continue;}
      if (!finishStage(logger, scene, *a)) {return false;}
    }
  }

  RCLCPP_INFO(logger, "All pick/place pairs finished. Returning both arms HOME.");
  return moveCombinedHome(logger, combined);
}

// ===========================================================================
// parameters
// ===========================================================================
std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {return "";}
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

// "pick_frame,place_frame,rebar_id,offset_x,offset_y,offset_z"
bool parsePairInput(const std::string & spec, PairInput & result, std::string & error)
{
  std::vector<std::string> fields;
  std::stringstream stream(spec);
  std::string field;
  while (std::getline(stream, field, ',')) {fields.push_back(trim(field));}

  if (fields.size() != 6) {
    error = "expected 6 comma-separated fields: "
      "pick_frame,place_frame,rebar_id,offset_x,offset_y,offset_z";
    return false;
  }
  if (fields[0].empty() || fields[1].empty() || fields[2].empty()) {
    error = "pick_frame, place_frame, and rebar_id must not be empty";
    return false;
  }
  std::array<double, 3> offsets{};
  for (std::size_t i = 0; i < 3; ++i) {
    try {
      std::size_t parsed = 0;
      offsets[i] = std::stod(fields[i + 3], &parsed);
      if (parsed != fields[i + 3].size() || !std::isfinite(offsets[i])) {
        error = "offset values must be finite numbers";
        return false;
      }
    } catch (const std::exception &) {
      error = "invalid numeric offset: '" + fields[i + 3] + "'";
      return false;
    }
  }
  result.pick = fields[0];
  result.place = fields[1];
  result.rebar = fields[2];
  result.pick_offset = offsets;
  return true;
}

bool loadPairs(
  const rclcpp::Node::SharedPtr & node, const std::string & param,
  const std::vector<std::string> & defaults, std::vector<PairInput> & out)
{
  if (!node->has_parameter(param)) {
    node->declare_parameter<std::vector<std::string>>(param, defaults);
  }
  std::vector<std::string> specs;
  if (!node->get_parameter(param, specs)) {specs = defaults;}   // empty list = arm has no work

  out.clear();
  for (std::size_t i = 0; i < specs.size(); ++i) {
    PairInput in;
    std::string err;
    if (!parsePairInput(specs[i], in, err)) {
      RCLCPP_ERROR(node->get_logger(), "Invalid %s entry %zu ('%s'): %s",
        param.c_str(), i + 1, specs[i].c_str(), err.c_str());
      return false;
    }
    out.push_back(in);
  }
  return true;
}

double loadDouble(const rclcpp::Node::SharedPtr & node, const std::string & name, double def)
{
  if (!node->has_parameter(name)) {node->declare_parameter<double>(name, def);}
  return node->get_parameter(name).as_double();
}

}  // namespace

// ===========================================================================
// main
// ===========================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const auto node_options =
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared(kNodeName, node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() {executor.spin();});

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf_buffer.setUsingDedicatedThread(true);
  tf2_ros::TransformListener tf_listener(tf_buffer, node, false);

  bool ok = false;
  try {
    ok = [&]() -> bool
      {
        loadShearProgram(node);
        RCLCPP_INFO(node->get_logger(), "Waiting for MoveIt, joint states and TF.");
        rclcpp::sleep_for(2s);

        MGI combined(node, kCombinedGroup);
        MGI robot1(node, kRobot1Group);
        MGI robot2(node, kRobot2Group);
        for (MGI * g : {&combined, &robot1, &robot2}) {
          g->setPoseReferenceFrame(kWorld);
          g->setPlanningTime(kPlanningTime);
        }
        robot1.setEndEffectorLink(kRobot1Link);
        robot2.setEndEffectorLink(kRobot2Link);

        if (!combined.getCurrentState(10.0) || !robot1.getCurrentState(10.0) ||
          !robot2.getCurrentState(10.0))
        {
          RCLCPP_ERROR(node->get_logger(), "Current robot state is unavailable.");
          return false;
        }

        PlanningSceneObjects scene(node);

        Arm a1, a2;
        a1.name = "robot1";
        a1.link = kRobot1Link;
        a1.mg = &robot1;
        a1.web_ring = kRobot1Ring;
        a1.via_x = loadDouble(node, "robot1_via_x", kRobot1ViaX);
        a1.gripper_links = kRobot1GripperLinks;
        a2.name = "robot2";
        a2.link = kRobot2Link;
        a2.mg = &robot2;
        a2.web_ring = kRobot2Ring;
        a2.via_x = loadDouble(node, "robot2_via_x", kRobot2ViaX);
        a2.gripper_links = kRobot2GripperLinks;
        // YAML list is authoritative; optional ROS pair-list overrides remain supported.
        if (!loadPairs(node, "robot1_pick_place_pairs", kRobot1PairSpecs, a1.pairs) ||
            !loadPairs(node, "robot2_pick_place_pairs", kRobot2PairSpecs, a2.pairs)) return false;

        a1.stage = a1.pairs.empty() ? Stage::kDone : Stage::kPickApproach;
        a2.stage = a2.pairs.empty() ? Stage::kDone : Stage::kPickApproach;

        RCLCPP_INFO(
          node->get_logger(), "Loaded %zu pairs for robot1 and %zu pairs for robot2.",
          a1.pairs.size(), a2.pairs.size());

        const bool success = runSequence(node, combined, tf_buffer, scene, a1, a2);
        if (!success) {
          recover(scene, a1);
          recover(scene, a2);
        }

        std::printf("\n==================== RUN SUMMARY ====================\n");
        std::printf("robot1: %zu / %zu pairs completed\n", a1.completed, a1.pairs.size());
        std::printf("robot2: %zu / %zu pairs completed\n", a2.completed, a2.pairs.size());
        std::printf("result: %s\n", success ? "OK (both arms home)" : "FAILED");
        std::printf("=====================================================\n");
        return success;
      }();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Unhandled exception: %s", e.what());
  } catch (...) {
    RCLCPP_FATAL(node->get_logger(), "Unhandled non-standard exception.");
  }

  (void)tf_listener;
  executor.cancel();
  if (executor_thread.joinable()) {executor_thread.join();}
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
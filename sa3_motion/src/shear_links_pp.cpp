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

using namespace std::chrono_literals;
using MoveItErr = moveit_msgs::msg::MoveItErrorCodes;
using MGI = moveit::planning_interface::MoveGroupInterface;
using JT = trajectory_msgs::msg::JointTrajectory;
using JTP = trajectory_msgs::msg::JointTrajectoryPoint;

namespace
{

constexpr char kNodeName[] = "shear_links_pp";
constexpr char kCombinedGroup[] = "sa3_robots_1_2";
constexpr char kWorld[] = "world";

constexpr char kRobot1Group[] = "robot1_arm";
constexpr char kRobot1Link[] = "robot1_end_effactor";   // (sic) matches SRDF
constexpr char kRobot2Group[] = "robot2_arm";
constexpr char kRobot2Link[] = "robot2_end_effactor";   // (sic) matches SRDF

constexpr std::array<double, 3> kApproachOffset = {0.0, -0.25, 0.0};

const std::string kJigClampId = "jig_clamp";

constexpr double kPlanningTime = 15.0;
constexpr int kOmplAttempts = 10;
constexpr int kPlanRetries = 3;

constexpr double kMoveJVelocity = 0.30;
constexpr double kMoveJAcceleration = 0.20;
constexpr double kMoveLAcceleration = 0.10;
constexpr double kMoveCAcceleration = 0.10;
constexpr std::array<double, 5> kLinVelScales = {0.10, 0.05, 0.03, 0.02, 0.01};
constexpr std::array<double, 5> kCircVelScales = {0.10, 0.05, 0.03, 0.02, 0.01};

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
  RCLCPP_INFO(logger, "%stransit strategy 1/3: LIN direct", tagOf(a).c_str());
  if (planLin(logger, a, start, a.place_approach, plan)) {
    method = "LIN direct";
    return true;
  }
  RCLCPP_WARN(
    logger, "%sLIN direct failed. Strategy 2/3: CIRC via x=%.3f", tagOf(a).c_str(), a.via_x);
  if (planCirc(logger, a, start, a.via, a.place_approach, plan)) {
    method = "CIRC via";
    return true;
  }
  RCLCPP_WARN(logger, "%sCIRC failed. Strategy 3/3: OMPL movej direct.", tagOf(a).c_str());
  if (planOmpl(logger, a, start, a.place_approach, plan)) {
    method = "OMPL movej";
    return true;
  }
  RCLCPP_ERROR(logger, "%sall three transit strategies failed.", tagOf(a).c_str());
  return false;
}

bool planStage(
  const rclcpp::Logger & logger, Arm & a, const moveit_msgs::msg::RobotState & start,
  MGI::Plan & plan, std::string & method)
{
  switch (a.stage) {
    case Stage::kPickApproach:
      method = "OMPL movej";
      return planOmpl(logger, a, start, a.pick_approach, plan);
    case Stage::kPick:
      method = "LIN";
      return planLin(logger, a, start, a.pick, plan);
    case Stage::kPickRetract:
      method = "LIN";
      return planLin(logger, a, start, a.pick_approach, plan);
    case Stage::kTransit:
      return planTransit(logger, a, start, plan, method);
    case Stage::kPlace:
      method = "LIN";
      return planLin(logger, a, start, a.place, plan);
    case Stage::kPlaceRetract:
      method = "LIN";
      return planLin(logger, a, start, a.place_approach, plan);
    default:
      return false;
  }
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
  a.pick_approach = offsetWorld(pick, kApproachOffset);
  a.place_approach = offsetWorld(place, kApproachOffset);
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
  const std::string rebar = a.rebar();
  switch (a.stage) {
    case Stage::kPickApproach:
      // gripper <-> rebar collision OFF until after the pick retract.
      if (!scene.setObjectCollisionsAllowed(rebar, a.gripper_links, true)) {
        RCLCPP_ERROR(logger, "%sfailed to allow gripper/rebar contact.", tagOf(a).c_str());
        return false;
      }
      a.stage = Stage::kPick;
      break;

    case Stage::kPick:
      if (!scene.attachWorldObjectPreservingPose(*a.mg, rebar, a.link, a.gripper_links) ||
        !scene.verifyObjectState(rebar, false, true, a.link))
      {
        RCLCPP_ERROR(logger, "%sfailed to attach '%s'.", tagOf(a).c_str(), rebar.c_str());
        return false;
      }
      RCLCPP_INFO(logger, "%sATTACHED: %s follows '%s'.", tagOf(a).c_str(), rebar.c_str(),
        a.link.c_str());
      // The attached rebar is still inside the rack slot while retracting.
      if (!scene.setCollisionPairAllowed(rebar, a.web_ring, true)) {
        RCLCPP_ERROR(logger, "%sfailed to allow %s <-> %s.", tagOf(a).c_str(), rebar.c_str(),
          a.web_ring.c_str());
        return false;
      }
      a.stage = Stage::kPickRetract;
      break;

    case Stage::kPickRetract:
      if (!scene.setCollisionPairAllowed(rebar, a.web_ring, false) ||
        !scene.setObjectCollisionsAllowed(rebar, a.gripper_links, false))
      {
        RCLCPP_ERROR(logger, "%sfailed to restore collisions after pick retract.",
          tagOf(a).c_str());
        return false;
      }
      a.stage = Stage::kTransit;
      break;

    case Stage::kTransit:
      // jig_clamp <-> rebar and gripper <-> rebar OFF until after the place retract.
      if (!scene.setObjectCollisionsAllowed(rebar, a.gripper_links, true) ||
        !scene.setCollisionPairAllowed(rebar, kJigClampId, true))
      {
        RCLCPP_ERROR(logger, "%sfailed to allow place contacts.", tagOf(a).c_str());
        return false;
      }
      a.stage = Stage::kPlace;
      break;

    case Stage::kPlace:
      if (!scene.detachObjectToWorldPreservingPose(*a.mg, rebar) ||
        !scene.verifyObjectState(rebar, true, false))
      {
        RCLCPP_ERROR(logger, "%sfailed to detach '%s'.", tagOf(a).c_str(), rebar.c_str());
        return false;
      }
      RCLCPP_INFO(logger, "%sRELEASED: '%s' is now a world object.", tagOf(a).c_str(),
        rebar.c_str());
      a.stage = Stage::kPlaceRetract;
      break;

    case Stage::kPlaceRetract:
      if (!scene.setObjectCollisionsAllowed(rebar, a.gripper_links, false) ||
        !scene.setCollisionPairAllowed(rebar, kJigClampId, false))
      {
        RCLCPP_ERROR(logger, "%sfailed to restore place collisions.", tagOf(a).c_str());
        return false;
      }
      RCLCPP_INFO(logger, "%s---- pair done ----", tagOf(a).c_str());
      ++a.completed;
      ++a.idx;
      a.pair_ready = false;
      a.stage = (a.idx < a.pairs.size()) ? Stage::kPickApproach : Stage::kDone;
      if (a.stage == Stage::kDone) {
        RCLCPP_INFO(logger, "[%s] all pairs finished; arm will hold its pose.", a.name.c_str());
      }
      break;

    default:
      break;
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
    RCLCPP_ERROR(logger, "Named state 'home' is not defined for '%s'.", kCombinedGroup);
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
        a1.web_ring = "web_ring";
        a1.via_x = loadDouble(node, "robot1_via_x", 0.235);
        a1.gripper_links = {
          kRobot1Link, "robot1_tool_mount", "robot1_J6_link",
          "robot1_schunk_base", "robot1_schunk_link7", "robot1_schunk_link8"};

        a2.name = "robot2";
        a2.link = kRobot2Link;
        a2.mg = &robot2;
        a2.web_ring = "web_ring_2";
        a2.via_x = loadDouble(node, "robot2_via_x", 5.22);
        a2.gripper_links = {
          kRobot2Link, "robot2_tool_mount", "robot2_J6_link",
          "robot2_schunk_base", "robot2_schunk_link7", "robot2_schunk_link8"};

        // Pair lists: override with robot1_pick_place_pairs / robot2_pick_place_pairs in YAML.
        // An empty list for one arm means that arm just holds while the other works.
        const std::vector<std::string> default_pairs_1 = {
          "5pf,5_placef_l,rebar_5a,0.0,0.0,0.0",
          "13pf,13_placef1_l,rebar_13_1,0.0,0.0,0.0",
          "13pf,13_placef2_l,rebar_13_2,0.10,0.0,0.0",
          "13pf,13_placef3_l,rebar_13_3,0.20,0.0,0.0",
          "13pf,13_placef4_l,rebar_13_4,0.30,0.0,0.0",
          "14pf,14_placef1_l,rebar_14_1,0.0,0.0,0.0",
          "14pf,14_placef2_l,rebar_14_2,0.20,0.0,0.0",
          "14pf,14_placef3_l,rebar_14_3,0.40,0.0,0.0",
          "14pf,14_placef4_l,rebar_14_4,0.60,0.0,0.0"};
        const std::vector<std::string> default_pairs_2 = {
          "6pf,6_placef_r,rebar_6,0.0,0.0,0.0",
          "13_1pf,13_placef1_r,rebar_13_1_R,0.0,0.0,0.0",
          "13_1pf,13_placef2_r,rebar_13_2_R,0.118,0.0,0.0",
          "13_2pf,13_placef3_r,rebar_13_3_R,0.0,0.0,0.0",
          "14_4pf,14_placef4_r,rebar_14_1_R,0.0,0.0,0.0",
          "14_4pf,14_placef3_r,rebar_14_2_R,0.112,0.0,0.0",
          "14_4pf,14_placef2_r,rebar_14_3_R,0.224,0.0,0.0",
          "14_4pf,14_placef1_r,rebar_14_4_R,0.336,0.0,0.0"};

        if (!loadPairs(node, "robot1_pick_place_pairs", default_pairs_1, a1.pairs) ||
          !loadPairs(node, "robot2_pick_place_pairs", default_pairs_2, a2.pairs))
        {
          return false;
        }
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
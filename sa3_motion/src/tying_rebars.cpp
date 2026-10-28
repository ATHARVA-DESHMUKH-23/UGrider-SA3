// tying_rebars.cpp  (sa3_motion)
//
// Rebar tying for robot1_arm + robot2_arm: SEPARATE planning, COMBINED execution.
//
// Per tying point <clamp>_t<n> each arm does (same as robot1_tying / robot2_tying):
//     1  tie approach   tp -20 (tying-frame X)     OMPL movej
//     2  tie            tp                         Pilz LIN
//     3  tie retract    tp -20                     Pilz LIN
//
// Every tick, each arm plans its own next step on its own MoveGroupInterface
// (robot1_arm / robot2_arm) from the same captured start state. The two plans are
// time-aligned, merged into ONE 12-joint trajectory and executed through the
// combined group (sa3_robots_1_2). The arms are not locked to each other: robot1
// can be on its tie retract while robot2 is still on its approach.
//
// * If an arm has no points left, it gets a hold trajectory (same pose) and stays
//   part of the combined execution until the other arm finishes.
// * If one arm cannot plan a step, that point is reported as FAILED, that arm
//   holds for the tick and moves on to its next point; the other arm keeps going.
//   (stop_on_first_failure:=true stops everything instead.)
// * When both arms are done, both go to the SRDF named state "home" together,
//   even if some points failed.
//
// Parameters (all optional):
//   robot1_clamp_frames : string[]  clamps, tying frames are <clamp>_t1 .. <clamp>_t<n>
//   robot1_tie_counts   : int[]     ties per clamp (one per clamp, or one value for all)
//   robot2_clamp_frames / robot2_tie_counts : same for robot2
//   robot1_enabled / robot2_enabled : bool (default true); false -> that arm only holds
//   stop_on_first_failure : bool (default false)
// Without *_clamp_frames the built-in default job tables are used.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace std::chrono_literals;
using MoveItErr = moveit_msgs::msg::MoveItErrorCodes;
using MGI = moveit::planning_interface::MoveGroupInterface;
using JT = trajectory_msgs::msg::JointTrajectory;
using JTP = trajectory_msgs::msg::JointTrajectoryPoint;

namespace
{

constexpr char kNodeName[] = "tying_rebars";
constexpr char kCombinedGroup[] = "sa3_robots_1_2";
constexpr char kWorld[] = "world";

constexpr char kRobot1Group[] = "robot1_arm";
constexpr char kRobot1Link[] = "robot1_tg_end_effactor";   // (sic) matches SRDF
constexpr char kRobot2Group[] = "robot2_arm";
constexpr char kRobot2Link[] = "robot2_tg_end_effactor";   // (sic) matches SRDF

// Tie approach / retract offset from the tying pose (metres), "tp - 20z".
constexpr std::array<double, 3> kApproachOffset = {-0.20, 0.0, 0.0};
// true: offset along the tying pose's OWN axes. false: along WORLD axes.
constexpr bool kOffsetInTyingFrame = true;

constexpr double kPlanningTime = 15.0;
constexpr int kOmplAttempts = 10;
constexpr int kPlanRetries = 3;

constexpr double kMoveJVelocity = 0.30;
constexpr double kMoveJAcceleration = 0.20;
constexpr double kMoveLAcceleration = 0.05;
// One LIN attempt per entry (velocity scale). Add smaller values for retries.
constexpr std::array<double, 1> kLinVelScales = {0.10};

// ===========================================================================
// types
// ===========================================================================
struct Job
{
  std::string clamp;
  int points;
};

const std::vector<Job> kDefaultJobs1 = {
  {"5_placef_l", 3},
  {"13_placef1_l", 2}, {"13_placef2_l", 2}, {"13_placef3_l", 2}, {"13_placef4_l", 2},
  {"14_placef1_l", 2}, {"14_placef2_l", 2}, {"14_placef3_l", 2}, {"14_placef4_l", 2},
};

const std::vector<Job> kDefaultJobs2 = {
  {"6_placef_r", 3},
  {"13_placef1_r", 2}, {"13_placef2_r", 2}, {"13_placef3_r", 2},
  {"14_placef1_r", 2}, {"14_placef2_r", 2}, {"14_placef3_r", 2}, {"14_placef4_r", 2},
};

enum class Stage {kApproach, kTie, kRetract, kDone};

int stepNumber(Stage s)
{
  switch (s) {
    case Stage::kApproach: return 1;
    case Stage::kTie: return 2;
    case Stage::kRetract: return 3;
    default: return 0;
  }
}

const char * stageName(Stage s)
{
  switch (s) {
    case Stage::kApproach: return "tie approach";
    case Stage::kTie: return "tie";
    case Stage::kRetract: return "tie retract";
    default: return "done";
  }
}

enum class Status {kNotRun, kOk, kFailed};

struct Failure
{
  int step_idx = 0;
  std::string step;
  std::string planner;   // OMPL | LIN | TF | COMBINED
  std::string stage;     // planning | execution | frame lookup
  int code = 0;
};

struct Point
{
  std::string clamp, frame;
  Status status = Status::kNotRun;
  Failure failure;
};

struct Arm
{
  std::string name;
  std::string link;
  MGI * mg = nullptr;

  std::vector<Point> points;
  std::size_t idx = 0;
  Stage stage = Stage::kDone;
  bool ready = false;

  geometry_msgs::msg::Pose tp, tp_approach;

  bool done() const {return stage == Stage::kDone;}
};

// ===========================================================================
// helpers
// ===========================================================================
geometry_msgs::msg::Pose offsetPose(
  const geometry_msgs::msg::Pose & original, const std::array<double, 3> & d)
{
  auto result = original;
  if (kOffsetInTyingFrame) {
    tf2::Quaternion q(
      original.orientation.x, original.orientation.y,
      original.orientation.z, original.orientation.w);
    const tf2::Vector3 r = tf2::quatRotate(q, tf2::Vector3(d[0], d[1], d[2]));
    result.position.x += r.x();
    result.position.y += r.y();
    result.position.z += r.z();
  } else {
    result.position.x += d[0];
    result.position.y += d[1];
    result.position.z += d[2];
  }
  return result;
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
    case MoveItErr::INVALID_GOAL_CONSTRAINTS: return "INVALID_GOAL_CONSTRAINTS";
    case MoveItErr::INVALID_ROBOT_STATE: return "INVALID_ROBOT_STATE";
    case MoveItErr::INVALID_LINK_NAME: return "INVALID_LINK_NAME";
    case MoveItErr::FRAME_TRANSFORM_FAILURE: return "FRAME_TRANSFORM_FAILURE";
    case MoveItErr::ROBOT_STATE_STALE: return "ROBOT_STATE_STALE";
    case MoveItErr::NO_IK_SOLUTION: return "NO_IK_SOLUTION";
    default: return "CODE_" + std::to_string(c);
  }
}

std::string hintFor(const std::string & planner, int code)
{
  switch (code) {
    case MoveItErr::NO_IK_SOLUTION:
      return "target pose has no IK solution (out of reach / orientation not achievable).";
    case MoveItErr::GOAL_IN_COLLISION:
      return "the target pose is in collision with the scene.";
    case MoveItErr::START_STATE_IN_COLLISION:
      return "the robot's start state is in collision (previous step ended in contact?).";
    case MoveItErr::INVALID_MOTION_PLAN:
    case MoveItErr::GOAL_CONSTRAINTS_VIOLATED:
      return planner == "LIN" ?
             "straight line is not feasible (collision on the line, joint limit or singularity)." :
             "planner returned a path that violates the goal constraints.";
    case MoveItErr::PLANNING_FAILED:
      return planner == "OMPL" ?
             "OMPL found no collision-free path within the budget." :
             "Pilz could not build the straight-line path.";
    case MoveItErr::CONTROL_FAILED:
      return "the controller rejected or aborted the trajectory.";
    default:
      return "";
  }
}

template<typename T>
T getOrDeclare(const rclcpp::Node::SharedPtr & node, const std::string & name, const T & def)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, def);
  }
  return node->get_parameter(name).get_value<T>();
}

// <prefix>_clamp_frames / <prefix>_tie_counts. No clamp_frames -> defaults.
bool loadJobs(
  const rclcpp::Node::SharedPtr & node, const std::string & prefix,
  const std::vector<Job> & defaults, std::vector<Job> & out)
{
  const auto logger = node->get_logger();
  const auto clamps =
    getOrDeclare<std::vector<std::string>>(node, prefix + "_clamp_frames", {});
  const auto counts =
    getOrDeclare<std::vector<int64_t>>(node, prefix + "_tie_counts", {});

  if (clamps.empty()) {
    RCLCPP_INFO(
      logger, "[%s] no clamp_frames given, using default job table (%zu clamps).",
      prefix.c_str(), defaults.size());
    out = defaults;
    return true;
  }
  if (counts.size() != 1 && counts.size() != clamps.size()) {
    RCLCPP_ERROR(
      logger, "[%s] tie_counts has %zu entries but clamp_frames has %zu. "
      "Give one count per clamp, or a single count for all.",
      prefix.c_str(), counts.size(), clamps.size());
    return false;
  }
  out.clear();
  for (std::size_t i = 0; i < clamps.size(); ++i) {
    const int64_t n = counts.size() == 1 ? counts[0] : counts[i];
    if (n < 1) {
      RCLCPP_ERROR(
        logger, "[%s] clamp '%s' has tie count %ld, must be >= 1.",
        prefix.c_str(), clamps[i].c_str(), static_cast<long>(n));
      return false;
    }
    out.push_back(Job{clamps[i], static_cast<int>(n)});
  }
  RCLCPP_INFO(logger, "[%s] loaded %zu clamps from parameters.", prefix.c_str(), out.size());
  return true;
}

void buildPoints(Arm & a, const std::vector<Job> & jobs)
{
  a.points.clear();
  for (const auto & job : jobs) {
    for (int n = 1; n <= job.points; ++n) {
      Point p;
      p.clamp = job.clamp;
      p.frame = job.clamp + "_t" + std::to_string(n);
      a.points.push_back(p);
    }
  }
  a.idx = 0;
  a.ready = false;
  a.stage = a.points.empty() ? Stage::kDone : Stage::kApproach;
}

std::string tagOf(const Arm & a)
{
  std::ostringstream s;
  s << "[" << a.name << " | point " << (a.idx + 1) << "/" << a.points.size();
  if (a.idx < a.points.size()) {s << " | " << a.points[a.idx].frame;}
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
// single-arm planning (planning only, nothing is executed here)
// ===========================================================================
bool planOmpl(
  const rclcpp::Logger & logger, Arm & a, const moveit_msgs::msg::RobotState & start,
  const geometry_msgs::msg::Pose & target, MGI::Plan & plan, int & code)
{
  auto & mg = *a.mg;
  code = 0;
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
      code = MoveItErr::FAILURE;
      return false;
    }
    const auto r = mg.plan(plan);
    mg.clearPoseTargets();
    code = r.val;
    if (r == moveit::core::MoveItErrorCode::SUCCESS) {return true;}
    RCLCPP_WARN(
      logger, "%sOMPL attempt %d/%d failed: %s", tagOf(a).c_str(), attempt, kPlanRetries,
      errStr(code).c_str());
  }
  return false;
}

bool planLin(
  const rclcpp::Logger & logger, Arm & a, const moveit_msgs::msg::RobotState & start,
  const geometry_msgs::msg::Pose & target, MGI::Plan & plan, int & code)
{
  auto & mg = *a.mg;
  code = 0;
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
      code = MoveItErr::FAILURE;
      return false;
    }
    const auto r = mg.plan(plan);
    mg.clearPoseTargets();
    code = r.val;
    if (r == moveit::core::MoveItErrorCode::SUCCESS) {return true;}
    RCLCPP_WARN(
      logger, "%sLIN attempt %d/%d failed at velocity scale %.3f: %s",
      tagOf(a).c_str(), attempt, total, scale, errStr(code).c_str());
    // A slower speed cannot fix these.
    if (code == MoveItErr::NO_IK_SOLUTION || code == MoveItErr::GOAL_IN_COLLISION ||
      code == MoveItErr::START_STATE_IN_COLLISION)
    {
      break;
    }
  }
  return false;
}

bool planStage(
  const rclcpp::Logger & logger, Arm & a, const moveit_msgs::msg::RobotState & start,
  MGI::Plan & plan, std::string & method, int & code)
{
  switch (a.stage) {
    case Stage::kApproach:
      method = "OMPL";
      return planOmpl(logger, a, start, a.tp_approach, plan, code);
    case Stage::kTie:
      method = "LIN";
      return planLin(logger, a, start, a.tp, plan, code);
    case Stage::kRetract:
      method = "LIN";
      return planLin(logger, a, start, a.tp_approach, plan, code);
    default:
      method = "-";
      code = MoveItErr::FAILURE;
      return false;
  }
}

// ===========================================================================
// trajectory merge
// ===========================================================================
double timeOf(const JTP & p) {return rclcpp::Duration(p.time_from_start).seconds();}

// A finished (or failed-this-tick) arm: constant pose for 1 s, stretched later.
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
// point bookkeeping
// ===========================================================================
void advance(const rclcpp::Logger & logger, Arm & a)
{
  ++a.idx;
  a.ready = false;
  a.stage = (a.idx < a.points.size()) ? Stage::kApproach : Stage::kDone;
  if (a.stage == Stage::kDone) {
    RCLCPP_INFO(logger, "[%s] all tying points processed; arm will hold its pose.",
      a.name.c_str());
  }
}

// Report the failure, mark the point FAILED and move the arm on to its next point.
void failPoint(
  const rclcpp::Logger & logger, Arm & a, int step_idx, const std::string & step,
  const std::string & planner, const std::string & stage, int code, const std::string & note = "")
{
  auto & p = a.points[a.idx];
  p.status = Status::kFailed;
  p.failure.step_idx = step_idx;
  p.failure.step = step;
  p.failure.planner = planner;
  p.failure.stage = stage;
  p.failure.code = code;

  std::ostringstream s;
  s << "\n================ FAILURE (" << a.name << ") ================\n"
    << " point     : " << (a.idx + 1) << " of " << a.points.size()
    << "   (clamp " << p.clamp << ", frame " << p.frame << ")\n"
    << " step      : " << step_idx << " of 3   '" << step << "'\n"
    << " planner   : " << planner << "\n"
    << " stage     : " << stage << "\n"
    << " error     : " << errStr(code) << " (" << code << ")\n";
  const auto hint = hintFor(planner, code);
  if (!hint.empty()) {s << " meaning   : " << hint << "\n";}
  if (!note.empty()) {s << " note      : " << note << "\n";}
  s << " action    : point skipped, " << a.name << " continues with its next point.\n"
    << "=========================================";
  RCLCPP_ERROR(logger, "%s", s.str().c_str());

  advance(logger, a);
}

bool beginPoint(const rclcpp::Logger & logger, tf2_ros::Buffer & tf, Arm & a)
{
  const auto & p = a.points[a.idx];
  if (!lookupPose(logger, tf, p.frame, a.tp)) {return false;}
  a.tp_approach = offsetPose(a.tp, kApproachOffset);
  a.ready = true;
  RCLCPP_INFO(
    logger, "%s---- tying point start (clamp %s) ---- tp %s", tagOf(a).c_str(),
    p.clamp.c_str(), poseStr(a.tp).c_str());
  return true;
}

// Called after the stage's motion has been executed.
void finishStage(const rclcpp::Logger & logger, Arm & a)
{
  switch (a.stage) {
    case Stage::kApproach:
      a.stage = Stage::kTie;
      break;
    case Stage::kTie:
      // >>> trigger the tying tool for this arm here <<<
      a.stage = Stage::kRetract;
      break;
    case Stage::kRetract:
      a.points[a.idx].status = Status::kOk;
      RCLCPP_INFO(logger, "%s---- tying point done ----", tagOf(a).c_str());
      advance(logger, a);
      break;
    default:
      break;
  }
}

// ===========================================================================
// combined HOME (named state "home" of sa3_robots_1_2)
// ===========================================================================
bool moveCombinedHome(const rclcpp::Logger & logger, MGI & combined)
{
  RCLCPP_INFO(logger, "Planning both arms to named target 'home'.");
  combined.clearPoseTargets();
  combined.clearPathConstraints();
  combined.setStartStateToCurrentState();
  combined.setPlanningPipelineId("ompl");
  combined.setPlannerId("RRTConnectkConfigDefault");
  combined.setPlanningTime(kPlanningTime);
  combined.setNumPlanningAttempts(kOmplAttempts);
  combined.setMaxVelocityScalingFactor(kMoveJVelocity);
  combined.setMaxAccelerationScalingFactor(kMoveJAcceleration);

  if (!combined.setNamedTarget("home")) {
    RCLCPP_ERROR(logger, "Named state 'home' is not defined for '%s'.", kCombinedGroup);
    return false;
  }
  MGI::Plan plan;
  for (int attempt = 1; attempt <= kPlanRetries; ++attempt) {
    combined.setStartStateToCurrentState();
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
// main loop. Returns false only if it had to abort (robot state unavailable /
// merge failure); point failures are recorded in the arms' results.
// ===========================================================================
bool runSequence(
  const rclcpp::Node::SharedPtr & node, MGI & combined, tf2_ros::Buffer & tf,
  Arm & a1, Arm & a2, bool stop_on_first_failure)
{
  const auto logger = node->get_logger();
  std::array<Arm *, 2> arms = {&a1, &a2};
  bool stop = false;
  int tick = 0;

  while (!stop && !(a1.done() && a2.done())) {
    auto state = combined.getCurrentState(10.0);
    if (!state) {
      RCLCPP_ERROR(logger, "Current combined robot state unavailable.");
      return false;
    }
    moveit_msgs::msg::RobotState start_msg;
    moveit::core::robotStateToRobotStateMsg(*state, start_msg);

    std::array<JT, 2> jts;
    std::array<std::string, 2> methods;
    std::array<bool, 2> moving = {false, false};

    // ---- plan each arm separately from the same start state ----
    for (std::size_t i = 0; i < arms.size(); ++i) {
      Arm & a = *arms[i];

      if (!a.done() && !a.ready && !beginPoint(logger, tf, a)) {
        failPoint(logger, a, 0, "frame lookup", "TF", "frame lookup",
          MoveItErr::FRAME_TRANSFORM_FAILURE, "'" + a.points[a.idx].frame + "' not in TF.");
        if (stop_on_first_failure) {stop = true;}
      }

      if (!a.done() && a.ready) {
        MGI::Plan plan;
        int code = 0;
        const int step_idx = stepNumber(a.stage);
        const std::string step_name = stageName(a.stage);
        if (planStage(logger, a, start_msg, plan, methods[i], code)) {
          if (!plan.trajectory_.multi_dof_joint_trajectory.joint_names.empty()) {
            failPoint(logger, a, step_idx, step_name, methods[i], "planning",
              MoveItErr::FAILURE, "multi-DOF trajectory cannot be merged.");
            if (stop_on_first_failure) {stop = true;}
          } else {
            jts[i] = plan.trajectory_.joint_trajectory;
            moving[i] = true;
          }
        } else {
          failPoint(logger, a, step_idx, step_name, methods[i], "planning", code,
            "target " + poseStr(a.stage == Stage::kTie ? a.tp : a.tp_approach));
          if (stop_on_first_failure) {stop = true;}
        }
      }

      if (!moving[i]) {
        if (!makeHold(a, *state, jts[i])) {
          RCLCPP_ERROR(logger, "[%s] could not build hold trajectory.", a.name.c_str());
          return false;
        }
        methods[i] = "HOLD";
      }
    }

    if (stop) {break;}
    if (!moving[0] && !moving[1]) {continue;}   // both idle/failed this tick

    ++tick;
    // ---- merge into one 12-joint trajectory, execute through the combined group ----
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
      for (std::size_t i = 0; i < arms.size(); ++i) {
        if (!moving[i]) {continue;}
        Arm & a = *arms[i];
        failPoint(logger, a, stepNumber(a.stage), stageName(a.stage), "COMBINED", "execution",
          executed.val);
      }
      if (stop_on_first_failure) {stop = true;}
      continue;
    }

    for (std::size_t i = 0; i < arms.size(); ++i) {
      if (moving[i]) {finishStage(logger, *arms[i]);}
    }
  }
  return true;
}

// ===========================================================================
// summary
// ===========================================================================
bool printSummary(const Arm & a)
{
  int ok = 0, failed = 0, not_run = 0;
  for (const auto & p : a.points) {
    if (p.status == Status::kOk) {++ok;}
    else if (p.status == Status::kFailed) {++failed;}
    else {++not_run;}
  }
  std::printf("\n==================== TYING SUMMARY (%s) ====================\n", a.name.c_str());
  const int total = static_cast<int>(a.points.size());
  int idx = 0;
  for (const auto & p : a.points) {
    ++idx;
    char col[16];
    std::snprintf(col, sizeof(col), "%d/%d", idx, total);
    if (p.status == Status::kOk) {
      std::printf("%-7s %-18s | OK\n", col, p.frame.c_str());
    } else if (p.status == Status::kNotRun) {
      std::printf("%-7s %-18s | NOT RUN\n", col, p.frame.c_str());
    } else {
      std::printf(
        "%-7s %-18s | FAILED | step %d/3 %-13s | %-8s | %-12s | %s\n", col, p.frame.c_str(),
        p.failure.step_idx, p.failure.step.c_str(), p.failure.planner.c_str(),
        p.failure.stage.c_str(), errStr(p.failure.code).c_str());
    }
  }
  std::printf("Points OK: %d / %d    FAILED: %d    NOT RUN: %d\n", ok, total, failed, not_run);
  return failed == 0 && not_run == 0;
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

        Arm a1, a2;
        a1.name = "robot1";
        a1.link = kRobot1Link;
        a1.mg = &robot1;
        a2.name = "robot2";
        a2.link = kRobot2Link;
        a2.mg = &robot2;

        const bool en1 = getOrDeclare<bool>(node, "robot1_enabled", true);
        const bool en2 = getOrDeclare<bool>(node, "robot2_enabled", true);
        const bool stop_on_first_failure =
          getOrDeclare<bool>(node, "stop_on_first_failure", false);

        std::vector<Job> jobs1, jobs2;
        if (!loadJobs(node, "robot1", kDefaultJobs1, jobs1) ||
          !loadJobs(node, "robot2", kDefaultJobs2, jobs2))
        {
          RCLCPP_ERROR(node->get_logger(), "Invalid job input, nothing to do.");
          return false;
        }
        buildPoints(a1, en1 ? jobs1 : std::vector<Job>{});
        buildPoints(a2, en2 ? jobs2 : std::vector<Job>{});
        if (a1.done() && a2.done()) {
          RCLCPP_ERROR(node->get_logger(), "Both arms have no tying points, nothing to do.");
          return false;
        }
        RCLCPP_INFO(
          node->get_logger(), "robot1: %zu tying points, robot2: %zu tying points.",
          a1.points.size(), a2.points.size());

        const bool completed = runSequence(node, combined, tf_buffer, a1, a2,
            stop_on_first_failure);

        const bool ok1 = printSummary(a1);
        const bool ok2 = printSummary(a2);
        std::printf("=====================================================================\n");

        // Always attempt to return home, even if some points failed.
        const bool home_ok = moveCombinedHome(node->get_logger(), combined);

        if (!completed) {RCLCPP_ERROR(node->get_logger(), "Tying sequence was aborted.");}
        if (!ok1 || !ok2) {RCLCPP_ERROR(node->get_logger(), "Tying sequence had failures.");}
        if (!home_ok) {RCLCPP_ERROR(node->get_logger(), "Robots did not return home.");}

        if (!completed || !ok1 || !ok2 || !home_ok) {return false;}
        RCLCPP_INFO(node->get_logger(), "All tying points completed and both robots are home.");
        return true;
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
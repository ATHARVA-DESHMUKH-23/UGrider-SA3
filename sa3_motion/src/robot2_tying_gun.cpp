// robot2_tying.cpp  (sa3_motion)
//
// Rebar tying node for robot2_arm.
//
// For every clamp, for every tying point <clamp>_t<n>:
//     1  movej  tp  -20 Z   (tie approach)      OMPL
//     2  movel  tp          (tie)               Pilz LIN
//     3  movel  tp  -20 Z   (tie retract)       Pilz LIN
// then the next tying point, until all are done.
//
// Clamps / tie counts come from ROS parameters:
//     clamp_frames : string[]   clamp frames (tying frames are <clamp>_t1 .. <clamp>_t<n>)
//     tie_counts   : int[]      ties per clamp (same length as clamp_frames,
//                               or one value applied to every clamp)
// If clamp_frames is not given, kDefaultJobs is used.
//
// Tying frames come from TF (see tying_frames YAML block).
// Failures print: point x/N, step k/3, planner, stage, MoveIt error code,
// planning time, target pose and start joints. A summary table is printed at the end.

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;
using MoveItErr = moveit_msgs::msg::MoveItErrorCodes;

namespace
{

constexpr const char * kNodeName = "sa3_tying_robot2";

// ---------------------------------------------------------------------------
// Robot / frames
// ---------------------------------------------------------------------------
constexpr char kGroup[] = "robot2_arm";
constexpr char kPlanningLink[] = "robot2_tg_end_effactor";   // (sic) matches SRDF
constexpr char kReferenceFrame[] = "world";

// Tie approach / retract offset from the tying pose (metres).
// "tp - 20z"
constexpr std::array<double, 3> kApproachOffset = {-0.20, 0.0, 0.0};

// true  : offset is along the tying pose's OWN z axis (tool approach direction,
//         follows the roll of the tying frame)
// false : offset is along WORLD z
constexpr bool kOffsetInTyingFrame = true;

// ---------------------------------------------------------------------------
// Behaviour
// ---------------------------------------------------------------------------
// false : a failed tying point is reported, node carries on with the next point
//         (every point starts with an OMPL movej, so it works from any pose).
// true  : stop at first failed point; the rest are listed as NOT RUN.
constexpr bool kStopOnFirstFailure = false;

// ---------------------------------------------------------------------------
// Planning settings
// ---------------------------------------------------------------------------
constexpr double kPlanningTime = 15.0;
constexpr int kOmplAttempts = 10;
constexpr int kPlanRetries = 3;

constexpr double kMoveJVelocity = 0.30;
constexpr double kMoveJAcceleration = 0.20;
constexpr double kMoveLVelocity = 0.10;
constexpr double kMoveLAcceleration = 0.05;

constexpr int kStepsPerPoint = 3;

// ---------------------------------------------------------------------------
// Job table: clamp frame + number of tying points
//   tying frame names = <clamp>_t1 ... <clamp>_t<points>
// ---------------------------------------------------------------------------
struct Job
{
  std::string clamp;
  int points;
};

// Used when the node is started without the `clamp_frames` parameter.
const std::vector<Job> kDefaultJobs = {
  {"6_placef_r", 3},

  {"13_placef1_r", 2},
  {"13_placef2_r", 2},
  {"13_placef3_r", 2},

  {"14_placef1_r", 2},
  {"14_placef2_r", 2},
  {"14_placef3_r", 2},
  {"14_placef4_r", 2},
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

// Reads a parameter that may already have been declared from overrides
// (automatically_declare_parameters_from_overrides) or not given at all.
template<typename T>
T getOrDeclare(const rclcpp::Node::SharedPtr & node, const std::string & name, const T & def)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, def);
  }
  return node->get_parameter(name).get_value<T>();
}

// clamp_frames : string[]  clamp frames, tying frames are <clamp>_t1 .. <clamp>_t<n>
// tie_counts   : int[]     ties per clamp. Same length as clamp_frames,
//                          or a single value that is applied to every clamp.
// No clamp_frames -> kDefaultJobs.
std::vector<Job> loadJobs(const rclcpp::Node::SharedPtr & node)
{
  const auto logger = node->get_logger();

  const auto clamps = getOrDeclare<std::vector<std::string>>(node, "clamp_frames", {});
  const auto counts = getOrDeclare<std::vector<int64_t>>(node, "tie_counts", {});

  if (clamps.empty()) {
    RCLCPP_INFO(
      logger, "No 'clamp_frames' given, using the default job table (%zu clamps).",
      kDefaultJobs.size());
    return kDefaultJobs;
  }

  if (counts.size() != 1 && counts.size() != clamps.size()) {
    RCLCPP_ERROR(
      logger,
      "'tie_counts' has %zu entries but 'clamp_frames' has %zu. "
      "Give one count per clamp, or a single count for all.",
      counts.size(), clamps.size());
    return {};
  }

  std::vector<Job> jobs;
  for (size_t i = 0; i < clamps.size(); ++i) {
    const int64_t n = counts.size() == 1 ? counts[0] : counts[i];
    if (n < 1) {
      RCLCPP_ERROR(
        logger, "Clamp '%s' has tie count %ld, must be >= 1.",
        clamps[i].c_str(), static_cast<long>(n));
      return {};
    }
    jobs.push_back(Job{clamps[i], static_cast<int>(n)});
  }

  RCLCPP_INFO(logger, "Loaded %zu clamps from parameters.", jobs.size());
  return jobs;
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
      return "target pose has no IK solution (out of reach / orientation not achievable / IK timeout).";
    case MoveItErr::GOAL_IN_COLLISION:
      return "the target pose is in collision with the scene.";
    case MoveItErr::START_STATE_IN_COLLISION:
      return "the robot's start state is in collision (previous step ended in contact?).";
    case MoveItErr::INVALID_MOTION_PLAN:
    case MoveItErr::GOAL_CONSTRAINTS_VIOLATED:
      return planner == "LIN" ?
             "straight line start->target is not feasible (collision on the line, joint limit or singularity)." :
             "planner returned a path that violates the goal constraints.";
    case MoveItErr::PLANNING_FAILED:
      return planner == "OMPL" ?
             "OMPL found no collision-free path within the time/attempt budget." :
             "Pilz could not build the straight-line path.";
    case MoveItErr::TIMED_OUT:
      return "planning or execution timed out.";
    case MoveItErr::CONTROL_FAILED:
      return "the controller rejected or aborted the trajectory.";
    default:
      return "";
  }
}

// ===========================================================================
// step / failure bookkeeping
// ===========================================================================
struct StepInfo
{
  int idx = 0;          // tying point index (1-based, overall)
  int total = 0;        // total tying points
  int step_idx = 0;
  int step_total = kStepsPerPoint;
  std::string clamp;
  std::string frame;    // tying frame
  std::string step;
};

struct Failure
{
  StepInfo info;
  std::string planner;   // "OMPL" | "LIN" | "TF"
  std::string stage;
  int code = 0;
  double plan_time = 0.0;
  geometry_msgs::msg::Pose target;
  bool has_target = false;
};

struct Ctx
{
  rclcpp::Node::SharedPtr node;
  moveit::planning_interface::MoveGroupInterface * mg = nullptr;
  tf2_ros::Buffer * tf = nullptr;
  std::string link = kPlanningLink;

  StepInfo cur;
  std::vector<Failure> failures;
};

std::string stepTag(const Ctx & c)
{
  std::ostringstream s;
  s << "[point " << c.cur.idx << "/" << c.cur.total
    << " | step " << c.cur.step_idx << "/" << c.cur.step_total
    << " | " << c.cur.frame << " | " << c.cur.step << "] ";
  return s.str();
}

std::string plannerLabel(const std::string & p)
{
  if (p == "OMPL") {return "OMPL (RRTConnect)";}
  if (p == "LIN") {return "Pilz LIN (straight line)";}
  return p;
}

void reportFailure(
  Ctx & c, const std::string & planner, const std::string & stage, int code,
  double plan_time, const geometry_msgs::msg::Pose * target, const std::string & note = "")
{
  Failure f;
  f.info = c.cur;
  f.planner = planner;
  f.stage = stage;
  f.code = code;
  f.plan_time = plan_time;
  if (target) {
    f.target = *target;
    f.has_target = true;
  }
  c.failures.push_back(f);

  std::ostringstream s;
  s << "\n================ FAILURE #" << c.failures.size() << " ================\n"
    << " point     : " << c.cur.idx << " of " << c.cur.total
    << "   (clamp " << c.cur.clamp << ", frame " << c.cur.frame << ")\n"
    << " step      : " << c.cur.step_idx << " of " << c.cur.step_total
    << "   '" << c.cur.step << "'\n"
    << " planner   : " << plannerLabel(planner) << "\n"
    << " stage     : " << stage << "\n"
    << " error     : " << errStr(code) << " (" << code << ")\n"
    << " time      : " << std::fixed << std::setprecision(2) << plan_time << " s\n";
  if (target) {
    s << " target    : " << poseStr(*target) << "\n";
  }
  if (c.mg) {
    const auto names = c.mg->getActiveJoints();
    const auto vals = c.mg->getCurrentJointValues();
    s << " start jnts: ";
    for (size_t i = 0; i < names.size() && i < vals.size(); ++i) {
      s << names[i] << "=" << std::setprecision(3) << vals[i] << " ";
    }
    s << "\n";
  }
  const auto hint = hintFor(planner, code);
  if (!hint.empty()) {
    s << " meaning   : " << hint << "\n";
  }
  if (!note.empty()) {
    s << " note      : " << note << "\n";
  }
  s << "=========================================";
  RCLCPP_ERROR(c.node->get_logger(), "%s", s.str().c_str());
}

// ===========================================================================
// TF
// ===========================================================================
bool lookupFramePose(Ctx & c, const std::string & frame, geometry_msgs::msg::Pose & pose)
{
  try {
    const auto t = c.tf->lookupTransform(
      kReferenceFrame, frame, tf2::TimePointZero, tf2::durationFromSec(10.0));
    pose.position.x = t.transform.translation.x;
    pose.position.y = t.transform.translation.y;
    pose.position.z = t.transform.translation.z;
    pose.orientation = t.transform.rotation;
    return true;
  } catch (const tf2::TransformException & e) {
    reportFailure(
      c, "TF", "frame lookup", MoveItErr::FRAME_TRANSFORM_FAILURE, 0.0, nullptr,
      "'" + frame + "' -> '" + kReferenceFrame + "': " + e.what());
    return false;
  }
}

// ===========================================================================
// MOVEJ : OMPL, pose target
// ===========================================================================
bool movej(Ctx & c, const geometry_msgs::msg::Pose & target)
{
  auto & mg = *c.mg;
  const auto logger = c.node->get_logger();
  const std::string tag = stepTag(c);

  if (!mg.getCurrentState(10.0)) {
    reportFailure(c, "OMPL", "current state", MoveItErr::ROBOT_STATE_STALE, 0.0, &target);
    return false;
  }
  mg.setStartStateToCurrentState();
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

  RCLCPP_INFO(logger, "%smovej target %s", tag.c_str(), poseStr(target).c_str());
  if (!mg.setPoseTarget(target, c.link)) {
    reportFailure(c, "OMPL", "set target", 0, 0.0, &target, "MoveIt rejected the pose target.");
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool planned = false;
  int last_code = 0;
  const auto t0 = std::chrono::steady_clock::now();
  for (int attempt = 1; attempt <= kPlanRetries; ++attempt) {
    const auto code = mg.plan(plan);
    last_code = code.val;
    if (code == moveit::core::MoveItErrorCode::SUCCESS) {
      planned = true;
      break;
    }
    RCLCPP_WARN(
      logger, "%sOMPL attempt %d/%d failed: %s", tag.c_str(), attempt, kPlanRetries,
      errStr(last_code).c_str());
  }
  const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  mg.clearPoseTargets();

  if (!planned) {
    reportFailure(
      c, "OMPL", "planning", last_code, dt, &target,
      "all " + std::to_string(kPlanRetries) + " retries x " + std::to_string(kOmplAttempts) +
      " attempts failed.");
    return false;
  }
  const auto exec = mg.execute(plan);
  if (exec != moveit::core::MoveItErrorCode::SUCCESS) {
    reportFailure(c, "OMPL", "execution", exec.val, dt, &target);
    return false;
  }
  RCLCPP_INFO(logger, "%smovej done (planned in %.2f s).", tag.c_str(), dt);
  return true;
}

// ===========================================================================
// MOVEL : Pilz LIN, pose target
// ===========================================================================
bool movel(Ctx & c, const geometry_msgs::msg::Pose & target)
{
  auto & mg = *c.mg;
  const auto logger = c.node->get_logger();
  const std::string tag = stepTag(c);

  if (!mg.getCurrentState(10.0)) {
    reportFailure(c, "LIN", "current state", MoveItErr::ROBOT_STATE_STALE, 0.0, &target);
    return false;
  }
  mg.setStartStateToCurrentState();
  mg.setPlanningPipelineId("pilz_industrial_motion_planner");
  mg.setPlannerId("LIN");
  mg.setPlanningTime(kPlanningTime);
  mg.setMaxVelocityScalingFactor(kMoveLVelocity);
  mg.setMaxAccelerationScalingFactor(kMoveLAcceleration);
  mg.clearPoseTargets();
  mg.clearPathConstraints();

  RCLCPP_INFO(logger, "%smovel target %s", tag.c_str(), poseStr(target).c_str());
  if (!mg.setPoseTarget(target, c.link)) {
    reportFailure(c, "LIN", "set target", 0, 0.0, &target, "MoveIt rejected the pose target.");
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto t0 = std::chrono::steady_clock::now();
  const auto code = mg.plan(plan);
  const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  mg.clearPoseTargets();

  if (code != moveit::core::MoveItErrorCode::SUCCESS) {
    reportFailure(c, "LIN", "planning", code.val, dt, &target);
    return false;
  }
  const auto exec = mg.execute(plan);
  if (exec != moveit::core::MoveItErrorCode::SUCCESS) {
    reportFailure(c, "LIN", "execution", exec.val, dt, &target);
    return false;
  }
  RCLCPP_INFO(logger, "%smovel done (planned in %.2f s).", tag.c_str(), dt);
  return true;
}

// ===========================================================================
// RETURN TO HOME
// Uses the SRDF named target "home" for this robot's MoveIt group.
 // ===========================================================================
bool moveHome(Ctx & c)
{
  auto & mg = *c.mg;
  const auto logger = c.node->get_logger();

  RCLCPP_INFO(logger, "All tying points processed; planning return to named target 'home'.");

  mg.clearPoseTargets();
  mg.clearPathConstraints();
  mg.setStartStateToCurrentState();
  mg.setPlanningPipelineId("ompl");
  mg.setPlannerId("RRTConnectkConfigDefault");
  mg.setPlanningTime(kPlanningTime);
  mg.setNumPlanningAttempts(kOmplAttempts);
  mg.setMaxVelocityScalingFactor(kMoveJVelocity);
  mg.setMaxAccelerationScalingFactor(kMoveJAcceleration);

  if (!mg.setNamedTarget("home")) {
    RCLCPP_ERROR(
      logger, "Could not set named target 'home' for group '%s'. Check the SRDF group_state.",
      kGroup);
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool planned = false;
  int last_code = 0;

  for (int attempt = 1; attempt <= kPlanRetries; ++attempt) {
    mg.setStartStateToCurrentState();
    const auto code = mg.plan(plan);
    last_code = code.val;
    if (code == moveit::core::MoveItErrorCode::SUCCESS) {
      planned = true;
      break;
    }
    RCLCPP_WARN(
      logger, "Home planning attempt %d/%d failed: %s",
      attempt, kPlanRetries, errStr(last_code).c_str());
  }

  mg.clearPoseTargets();

  if (!planned) {
    RCLCPP_ERROR(
      logger, "Failed to plan return to home after %d attempts: %s",
      kPlanRetries, errStr(last_code).c_str());
    return false;
  }

  const auto exec = mg.execute(plan);
  if (exec != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution of return-to-home trajectory failed: %s", errStr(exec.val).c_str());
    return false;
  }

  RCLCPP_INFO(logger, "Robot returned to home successfully.");
  return true;
}

// ===========================================================================
// TIE ONE POINT
//   c.cur.idx / total / clamp / frame are set by the caller.
// ===========================================================================
bool tiePoint(Ctx & c)
{
  const auto logger = c.node->get_logger();

  RCLCPP_INFO(
    logger, "---- tying point %d/%d: %s (clamp %s) ----",
    c.cur.idx, c.cur.total, c.cur.frame.c_str(), c.cur.clamp.c_str());

  auto step = [&](int idx, const std::string & name) {
    c.cur.step_idx = idx;
    c.cur.step = name;
  };

  step(0, "frame lookup");
  geometry_msgs::msg::Pose tp;
  if (!lookupFramePose(c, c.cur.frame, tp)) {
    return false;
  }

  const auto tp_approach = offsetPose(tp, kApproachOffset);

  step(1, "tie approach (movej)");
  if (!movej(c, tp_approach)) {return false;}

  step(2, "tie (movel)");
  if (!movel(c, tp)) {return false;}

  // >>> trigger the tying tool here <<<

  step(3, "tie retract (movel)");
  if (!movel(c, tp_approach)) {return false;}

  RCLCPP_INFO(logger, "---- tying point %d/%d done ----", c.cur.idx, c.cur.total);
  return true;
}

// ===========================================================================
// results / summary
// ===========================================================================
enum class Status { kOk, kFailed, kNotRun };

struct PointResult
{
  std::string clamp, frame;
  Status status = Status::kNotRun;
  Failure failure;
};

void printSummary(const std::vector<PointResult> & results)
{
  int ok = 0, failed = 0, not_run = 0;
  int by_ompl = 0, by_lin = 0, by_other = 0;
  for (const auto & r : results) {
    if (r.status == Status::kOk) {++ok;}
    else if (r.status == Status::kNotRun) {++not_run;}
    else {
      ++failed;
      if (r.failure.planner == "OMPL") {++by_ompl;}
      else if (r.failure.planner == "LIN") {++by_lin;}
      else {++by_other;}
    }
  }

  std::printf("\n==================== TYING SUMMARY (robot2_arm) ====================\n");
  std::printf(
    "%-7s %-18s | %-7s | %-26s | %-5s | %-12s | %s\n",
    "point", "tying frame", "result", "failed step", "tool", "stage", "error");
  const int total = static_cast<int>(results.size());
  int idx = 0;
  for (const auto & r : results) {
    ++idx;
    char col[16];
    std::snprintf(col, sizeof(col), "%d/%d", idx, total);
    if (r.status == Status::kOk) {
      std::printf("%-7s %-18s | OK\n", col, r.frame.c_str());
    } else if (r.status == Status::kNotRun) {
      std::printf("%-7s %-18s | NOT RUN\n", col, r.frame.c_str());
    } else {
      char step_col[40];
      std::snprintf(
        step_col, sizeof(step_col), "%d/%d %s", r.failure.info.step_idx,
        r.failure.info.step_total, r.failure.info.step.c_str());
      std::printf(
        "%-7s %-18s | FAILED  | %-26s | %-5s | %-12s | %s\n",
        col, r.frame.c_str(), step_col, r.failure.planner.c_str(),
        r.failure.stage.c_str(), errStr(r.failure.code).c_str());
    }
  }
  std::printf("\nPoints OK: %d / %d    FAILED: %d    NOT RUN: %d\n", ok, total, failed, not_run);
  std::printf(
    "Failures by planner:  OMPL=%d   Pilz LIN=%d   other (TF etc.)=%d\n", by_ompl, by_lin, by_other);
  std::printf("====================================================================\n");
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

        moveit::planning_interface::MoveGroupInterface move_group(node, kGroup);
        move_group.setPoseReferenceFrame(kReferenceFrame);
        move_group.setEndEffectorLink(kPlanningLink);
        move_group.setPlanningTime(kPlanningTime);

        if (!move_group.getCurrentState(10.0)) {
          RCLCPP_ERROR(node->get_logger(), "Current robot state is unavailable.");
          return false;
        }
        RCLCPP_INFO(
          node->get_logger(), "Group '%s', planning link '%s', reference '%s'.",
          kGroup, kPlanningLink, kReferenceFrame);

        Ctx ctx;
        ctx.node = node;
        ctx.mg = &move_group;
        ctx.tf = &tf_buffer;

        // clamps + tie counts: from parameters, or kDefaultJobs
        const auto jobs = loadJobs(node);
        if (jobs.empty()) {
          RCLCPP_ERROR(node->get_logger(), "Invalid job input, nothing to do.");
          return false;
        }

        // flat list of tying points: <clamp>_t1 .. <clamp>_t<n>
        std::vector<PointResult> results;
        for (const auto & job : jobs) {
          for (int n = 1; n <= job.points; ++n) {
            PointResult r;
            r.clamp = job.clamp;
            r.frame = job.clamp + "_t" + std::to_string(n);
            results.push_back(r);
          }
        }
        const int total = static_cast<int>(results.size());

        bool stop = false;
        for (int i = 0; i < total && !stop; ++i) {
          auto & r = results[i];
          ctx.cur = StepInfo{};
          ctx.cur.idx = i + 1;
          ctx.cur.total = total;
          ctx.cur.clamp = r.clamp;
          ctx.cur.frame = r.frame;

          const size_t failures_before = ctx.failures.size();
          if (tiePoint(ctx)) {
            r.status = Status::kOk;
          } else {
            r.status = Status::kFailed;
            if (ctx.failures.size() > failures_before) {
              r.failure = ctx.failures.back();
            }
            if (kStopOnFirstFailure) {
              stop = true;
              RCLCPP_ERROR(
                node->get_logger(), "Stopping: point %d/%d (%s) failed.",
                i + 1, total, r.frame.c_str());
            }
          }
        }

        printSummary(results);

        // Always attempt to return home after the tying sequence has finished,
        // even if one or more tying points failed.
        const bool home_ok = moveHome(ctx);

        bool tying_ok = true;
        for (const auto & r : results) {
          if (r.status != Status::kOk) {
            tying_ok = false;
            break;
          }
        }

        if (!tying_ok) {
          RCLCPP_ERROR(node->get_logger(), "Tying sequence had failures.");
        }
        if (!home_ok) {
          RCLCPP_ERROR(node->get_logger(), "Robot did not return home successfully.");
        }

        if (!tying_ok || !home_ok) {return false;}

        RCLCPP_INFO(node->get_logger(), "All %d tying points completed and robot is home.", total);
        return true;
      }();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Unhandled exception: %s", e.what());
  } catch (...) {
    RCLCPP_FATAL(node->get_logger(), "Unhandled non-standard exception.");
  }

  (void)tf_listener;
  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
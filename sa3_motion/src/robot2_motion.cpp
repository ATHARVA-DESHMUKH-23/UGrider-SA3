// robot2_motion.cpp  (sa3_motion)
//
// Robot 2 pick/place with rebar attach/detach (same scene API as Robot 1 / Robot 3).
//
//   pickPlace(pick_frame, place_frame, rebar):
//     1  movej pick approach
//        gripper <-> rebar collision OFF
//     2  movel pick            -> ATTACH rebar to the planning link
//     3  movel pick retract    -> gripper <-> rebar collision ON again
//     4  CIRC via transition -> LIN via stop -> OMPL direct fallback
//     5  reach place approach
//        jig_clamp <-> rebar and gripper <-> rebar collision OFF
//     6  movel place           -> DETACH rebar into the world
//     7  movel place retract   -> collisions ON again
//
// Offsets are in the WORLD frame.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sa3_motion/direct_execute.hpp"
#include "sa3_motion/planning_scene_objects.hpp"

using namespace std::chrono_literals;
using MoveItErr = moveit_msgs::msg::MoveItErrorCodes;

namespace
{

constexpr const char * kNodeName = "sa3_robot2_pick_place_fallback";

// ---------------------------------------------------------------------------
// Robot / frames
// ---------------------------------------------------------------------------
constexpr char kGroup[] = "robot2_arm";
constexpr char kPlanningLink[] = "robot2_end_effactor";   // (sic) matches SRDF
constexpr char kReferenceFrame[] = "world";
constexpr char kControllerAction[] = "/robot2_arm_controller/follow_joint_trajectory";

constexpr std::array<double, 3> kApproachOffset = {0.0, -0.25, 0.0};

// true : spawn the rebar collision objects here.
// false: rely on rebar_planning_scene_node having spawned them already.
constexpr bool kSpawnRebars = false;

constexpr bool kStopOnFirstFailure = true;

// Fixed world-frame X coordinate of the CIRC/LIN via point (Robot 2).
constexpr double kViaX = 5.22;

// ---------------------------------------------------------------------------
// Collision settings
// ---------------------------------------------------------------------------
// Must match the collision object IDs in the planning scene.
const std::string kJigClampId = "jig_clamp";
const std::string kWebRingId = "web_ring_2";   // right web rack (parent of the *_R rebars)

// Links that touch the rebar while it is picked / released.
const std::vector<std::string> kGripperLinks = {
  kPlanningLink, "robot2_tool_mount", "robot2_J6_link",
  "robot2_schunk_base", "robot2_schunk_link7", "robot2_schunk_link8"};

// ---------------------------------------------------------------------------
// Planning settings
// ---------------------------------------------------------------------------
constexpr double kPlanningTime = 15.0;
constexpr int kOmplAttempts = 10;
constexpr int kPlanRetries = 3;

constexpr double kMoveJVelocity = 0.30;
constexpr double kMoveJAcceleration = 0.20;
constexpr double kMoveLVelocity = 0.10;
constexpr double kMoveLAcceleration = 0.10;
constexpr double kMoveCVelocity = 0.10;
constexpr double kMoveCAcceleration = 0.10;
constexpr std::array<double, 5> kMoveCVelScales = {0.10, 0.05, 0.03, 0.02, 0.01};
constexpr std::array<double, 5> kMoveLVelScales = {0.10, 0.05, 0.03, 0.02, 0.01};

constexpr int kStepsPerPair = 7;


// ===========================================================================
// helpers
// ===========================================================================
geometry_msgs::msg::Pose offsetWorld(
  const geometry_msgs::msg::Pose & original, const std::array<double, 3> & d)
{
  auto result = original;
  result.position.x += d[0];
  result.position.y += d[1];
  result.position.z += d[2];
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
      return "target pose has no IK solution.";
    case MoveItErr::GOAL_IN_COLLISION:
      return "the target pose is in collision with the scene.";
    case MoveItErr::START_STATE_IN_COLLISION:
      return "the robot's start state is in collision.";
    case MoveItErr::INVALID_MOTION_PLAN:
    case MoveItErr::GOAL_CONSTRAINTS_VIOLATED:
      return planner == "LIN" ?
             "straight line is not feasible (collision on the line, joint limit or singularity)." :
             "planner returned a path that violates the goal constraints.";
    case MoveItErr::PLANNING_FAILED:
      return planner == "OMPL" ?
             "OMPL found no collision-free path within the budget." :
             "Pilz could not build the straight-line path.";
    default:
      return "";
  }
}

// Runs a function on scope exit, or earlier through runNow().
class ScopeGuard
{
public:
  explicit ScopeGuard(std::function<void()> fn) : fn_(std::move(fn)) {}
  ScopeGuard(const ScopeGuard &) = delete;
  ScopeGuard & operator=(const ScopeGuard &) = delete;
  void runNow()
  {
    if (fn_) {
      auto fn = std::move(fn_);
      fn_ = nullptr;
      fn();
    }
  }
  ~ScopeGuard() {runNow();}

private:
  std::function<void()> fn_;
};

// ===========================================================================
// step / failure bookkeeping
// ===========================================================================
struct StepInfo
{
  int pair_idx = 0;
  int pair_total = 0;
  int step_idx = 0;
  int step_total = kStepsPerPair;
  std::string pick;
  std::string place;
  std::string rebar;
  std::string step;
};

struct Failure
{
  StepInfo info;
  std::string planner;
  std::string stage;
  int code = 0;
  double plan_time = 0.0;
};

struct Ctx
{
  rclcpp::Node::SharedPtr node;
  moveit::planning_interface::MoveGroupInterface * mg = nullptr;
  tf2_ros::Buffer * tf = nullptr;
  PlanningSceneObjects * scene = nullptr;
  std::string link = kPlanningLink;
  rclcpp_action::Client<sa3_motion::FJT>::SharedPtr fjt;

  StepInfo cur;
  std::array<double, 3> pick_offset = {0.0, 0.0, 0.0};
  std::vector<Failure> failures;
};

std::string stepTag(const Ctx & c)
{
  std::ostringstream s;
  s << "[pair " << c.cur.pair_idx << "/" << c.cur.pair_total
    << " | step " << c.cur.step_idx << "/" << c.cur.step_total
    << " | " << c.cur.pick << " -> " << c.cur.place
    << " | " << c.cur.rebar
    << " | " << c.cur.step << "] ";
  return s.str();
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
  c.failures.push_back(f);

  std::ostringstream s;
  s << "\n================ FAILURE #" << c.failures.size() << " ================\n"
    << " pair      : " << c.cur.pair_idx << " of " << c.cur.pair_total
    << "   (" << c.cur.pick << " -> " << c.cur.place << ", " << c.cur.rebar << ")\n"
    << " step      : " << c.cur.step_idx << " of " << c.cur.step_total
    << "   '" << c.cur.step << "'\n"
    << " planner   : " << planner << "\n"
    << " stage     : " << stage << "\n"
    << " error     : " << errStr(code) << " (" << code << ")\n"
    << " time      : " << std::fixed << std::setprecision(2) << plan_time << " s\n";
  if (target) {
    s << " target    : " << poseStr(*target) << "\n";
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
    reportFailure(c, "OMPL", "set target", MoveItErr::FAILURE, 0.0, &target,
      "MoveIt rejected the pose target.");
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
    reportFailure(c, "OMPL", "planning", last_code, dt, &target, "all retries failed.");
    return false;
  }
  int exec_code = 0;
  std::string exec_note;
  if (!sa3_motion::executeDirect(c.fjt, plan.trajectory_.joint_trajectory, exec_code, exec_note)) {
    reportFailure(c, "OMPL", "execution", exec_code, dt, &target, exec_note);
    return false;
  }
  RCLCPP_INFO(logger, "%smovej done (%.2f s).", tag.c_str(), dt);
  return true;
}

// ===========================================================================
// MOVEL : Pilz LIN, pose target
// ===========================================================================
bool movel(Ctx & c, const geometry_msgs::msg::Pose & target, bool * planning_failed = nullptr)
{
  if (planning_failed) *planning_failed = false;
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
    reportFailure(c, "LIN", "set target", MoveItErr::FAILURE, 0.0, &target,
      "MoveIt rejected the pose target.");
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool planned = false;
  int last_code = 0;
  const int total = static_cast<int>(kMoveLVelScales.size());
  const auto t0 = std::chrono::steady_clock::now();

  for (int attempt = 1; attempt <= total; ++attempt) {
    const double scale = kMoveLVelScales[attempt - 1];

    mg.setStartStateToCurrentState();
    mg.setMaxVelocityScalingFactor(scale);
    mg.setMaxAccelerationScalingFactor(std::min(kMoveLAcceleration, scale));

    const auto result = mg.plan(plan);
    last_code = result.val;

    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      planned = true;
      RCLCPP_INFO(
        logger, "%sLIN planning succeeded on attempt %d/%d (velocity scale %.3f)",
        tag.c_str(), attempt, total, scale);
      break;
    }

    RCLCPP_WARN(
      logger, "%sLIN attempt %d/%d failed at velocity scale %.3f: %s",
      tag.c_str(), attempt, total, scale, errStr(last_code).c_str());

    // A slower speed cannot fix these, so stop early.
    if (last_code == MoveItErr::NO_IK_SOLUTION ||
      last_code == MoveItErr::GOAL_IN_COLLISION ||
      last_code == MoveItErr::START_STATE_IN_COLLISION)
    {
      break;
    }
  }
  const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  mg.clearPoseTargets();

  if (!planned) {
    reportFailure(c, "LIN", "planning", last_code, dt, &target, "all velocity scales failed.");
    if (planning_failed) *planning_failed = true;
    return false;
  }
  int exec_code = 0;
  std::string exec_note;
  if (!sa3_motion::executeDirect(c.fjt, plan.trajectory_.joint_trajectory, exec_code, exec_note)) {
    reportFailure(c, "LIN", "execution", exec_code, dt, &target, exec_note);
    return false;
  }
  RCLCPP_INFO(logger, "%smovel done (%.2f s).", tag.c_str(), dt);
  return true;
}

// ===========================================================================
// MOVEC : Pilz CIRC, Cartesian circular arc through an interim point
// ===========================================================================
bool movec(
  Ctx & c,
  const geometry_msgs::msg::Pose & interim,
  const geometry_msgs::msg::Pose & target,
  bool * planning_failed = nullptr)
{
  if (planning_failed) *planning_failed = false;
  auto & mg = *c.mg;
  const auto logger = c.node->get_logger();
  const std::string tag = stepTag(c);

  if (!mg.getCurrentState(10.0)) {
    reportFailure(c, "CIRC", "current state", MoveItErr::ROBOT_STATE_STALE, 0.0, &target);
    return false;
  }

  // Pilz requires exactly one position constraint and the name "interim".
  moveit_msgs::msg::PositionConstraint via_constraint;
  via_constraint.header.frame_id = kReferenceFrame;
  via_constraint.link_name = c.link;
  via_constraint.constraint_region.primitive_poses.push_back(interim);
  via_constraint.weight = 1.0;

  moveit_msgs::msg::Constraints path_constraints;
  path_constraints.name = "interim";
  path_constraints.position_constraints.push_back(via_constraint);

  mg.setStartStateToCurrentState();
  mg.setPlanningPipelineId("pilz_industrial_motion_planner");
  mg.setPlannerId("CIRC");
  mg.setPlanningTime(kPlanningTime);
  mg.setNumPlanningAttempts(1);
  mg.setMaxVelocityScalingFactor(kMoveCVelocity);
  mg.setMaxAccelerationScalingFactor(kMoveCAcceleration);
  mg.clearPoseTargets();
  mg.clearPathConstraints();
  mg.setPathConstraints(path_constraints);

  // Ensure the CIRC-only interim constraint never leaks into LIN / OMPL.
  ScopeGuard constraint_guard([&mg]() {
    mg.clearPathConstraints();
    mg.clearPoseTargets();
  });

  RCLCPP_INFO(logger, "%smovec interim %s", tag.c_str(), poseStr(interim).c_str());
  RCLCPP_INFO(logger, "%smovec target  %s", tag.c_str(), poseStr(target).c_str());
  if (!mg.setPoseTarget(target, c.link)) {
    reportFailure(c, "CIRC", "set target", MoveItErr::FAILURE, 0.0, &target,
      "MoveIt rejected the pose target.");
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool planned = false;
  int last_code = MoveItErr::PLANNING_FAILED;
  const int total = static_cast<int>(kMoveCVelScales.size());
  const auto t0 = std::chrono::steady_clock::now();

  for (int attempt = 1; attempt <= total; ++attempt) {
    const double scale = kMoveCVelScales[attempt - 1];
    mg.setStartStateToCurrentState();
    mg.setMaxVelocityScalingFactor(scale);
    mg.setMaxAccelerationScalingFactor(std::min(kMoveCAcceleration, scale));

    const auto result = mg.plan(plan);
    last_code = result.val;
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      planned = true;
      RCLCPP_INFO(logger,
        "%sCIRC planning succeeded on attempt %d/%d (velocity scale %.3f)",
        tag.c_str(), attempt, total, scale);
      break;
    }

    RCLCPP_WARN(logger,
      "%sCIRC attempt %d/%d failed at velocity scale %.3f: %s",
      tag.c_str(), attempt, total, scale, errStr(last_code).c_str());

    // Slowing down cannot fix these failures.
    if (last_code == MoveItErr::NO_IK_SOLUTION ||
        last_code == MoveItErr::GOAL_IN_COLLISION ||
        last_code == MoveItErr::START_STATE_IN_COLLISION ||
        last_code == MoveItErr::INVALID_GROUP_NAME ||
        last_code == MoveItErr::INVALID_LINK_NAME ||
        last_code == MoveItErr::FRAME_TRANSFORM_FAILURE) {
      break;
    }
  }

  const double dt = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - t0).count();
  // Clear CIRC-only constraints before other motion commands.
  constraint_guard.runNow();

  if (!planned) {
    reportFailure(c, "CIRC", "planning", last_code, dt, &target,
      "CIRC retries failed down to velocity scale 0.01. "
      "Check arc geometry, collisions, IK continuity/singularities and joint limits.");
    if (planning_failed) *planning_failed = true;
    return false;
  }

  int exec_code = 0;
  std::string exec_note;
  if (!sa3_motion::executeDirect(c.fjt, plan.trajectory_.joint_trajectory,
      exec_code, exec_note)) {
    reportFailure(c, "CIRC", "execution", exec_code, dt, &target, exec_note);
    return false;
  }

  RCLCPP_INFO(logger, "%smovec done (%.2f s).", tag.c_str(), dt);
  return true;
}

// ===========================================================================
// PICK_PLACE(pick_frame, place_frame, rebar)
// ===========================================================================
bool pickPlace(Ctx & c)
{
  const auto logger = c.node->get_logger();
  auto & scene = *c.scene;

  const std::string pick_frame = c.cur.pick;
  const std::string place_frame = c.cur.place;
  const std::string rebar = c.cur.rebar;

  RCLCPP_INFO(
    logger, "---- pair %d/%d: %s -> %s (%s) ----",
    c.cur.pair_idx, c.cur.pair_total, pick_frame.c_str(), place_frame.c_str(), rebar.c_str());

  auto step = [&](int idx, const std::string & name) {
      c.cur.step_idx = idx;
      c.cur.step = name;
    };

  step(0, "frame lookup");

  geometry_msgs::msg::Pose pick, place;
  if (!lookupFramePose(c, pick_frame, pick) || !lookupFramePose(c, place_frame, place)) {
    return false;
  }

  pick.position.x += c.pick_offset[0];
  pick.position.y += c.pick_offset[1];
  pick.position.z += c.pick_offset[2];

  const auto pick_approach = offsetWorld(pick, kApproachOffset);
  const auto place_approach = offsetWorld(place, kApproachOffset);

  // Robot 2 via point: fixed world X; Y, Z and orientation from place approach.
  auto via_pose = place_approach;
  via_pose.position.x = kViaX;

  // The rebar must be a world object before the pick.
  if (!scene.verifyObjectState(rebar, true, false)) {
    RCLCPP_ERROR(logger, "Expected '%s' to be a world object before picking.", rebar.c_str());
    return false;
  }

  // ============================================================
  // PICK
  // ============================================================
  step(1, "pick approach (movej)");
  if (!movej(c, pick_approach)) {
    return false;
  }

  // Gripper <-> rebar collisions OFF until after the pick retract.
  step(1, "allow gripper/rebar contact");
  if (!scene.setObjectCollisionsAllowed(rebar, kGripperLinks, true)) {
    RCLCPP_ERROR(logger, "Failed to allow contact for '%s'.", rebar.c_str());
    return false;
  }
  ScopeGuard pick_guard([&]() {
      scene.setObjectCollisionsAllowed(rebar, kGripperLinks, false);
    });

  step(2, "pick (movel)");
  if (!movel(c, pick)) {
    return false;
  }

  step(2, "attach " + rebar);
  if (!scene.attachWorldObjectPreservingPose(*c.mg, rebar, c.link, kGripperLinks) ||
    !scene.verifyObjectState(rebar, false, true, c.link))
  {
    RCLCPP_ERROR(logger, "Failed to attach '%s' to '%s'.", rebar.c_str(), c.link.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "ATTACHED: %s follows '%s'.", rebar.c_str(), c.link.c_str());

  // Temporarily allow the attached rebar to intersect the web ring (it is
  // still inside the rack slot while retracting).
  step(3, "allow attached rebar/web_ring contact");
  if (!scene.setCollisionPairAllowed(rebar, kWebRingId, true)) {
    RCLCPP_ERROR(
      logger, "Failed to allow collision between '%s' and '%s'.", rebar.c_str(), kWebRingId.c_str());
    return false;
  }
  // Runs on any early return below, so the allowance never leaks.
  ScopeGuard web_ring_guard([&]() {
      scene.setCollisionPairAllowed(rebar, kWebRingId, false);
    });

  step(3, "pick retract (movel)");
  if (!movel(c, pick_approach)) {
    RCLCPP_ERROR(logger, "Retract failed; '%s' remains attached.", rebar.c_str());
    return false;
  }

  // // Custom transition move (still carrying the rebar, allowance still on).
  // step(4, "transition (movel)");
  // if (!movel(c, transition_pose)) {
  //   return false;
  // }

  // // Clear of the rack: restore normal collision checking.
  // step(4, "restore rebar/web_ring + gripper/rebar collision");
  // web_ring_guard.runNow();
  // pick_guard.runNow();
  // RCLCPP_INFO(logger, "Collision checking restored for '%s'.", rebar.c_str());

  // ============================================================
  // PLACE APPROACH: CIRC -> LIN with via stop -> OMPL direct
  // Robot 2 via point = (kViaX, place_approach.y, place_approach.z),
  // with orientation from place_approach.
  // ============================================================
  bool approach_done = false;
  bool planning_failed = false;

  step(5, "place approach (CIRC through fixed via point)");
  RCLCPP_INFO(logger, "Approach strategy 1/3: CIRC through robot2 fixed via point");
  if (movec(c, via_pose, place_approach, &planning_failed)) {
    approach_done = true;
  } else if (!planning_failed) {
    RCLCPP_ERROR(logger, "CIRC non-planning failure; aborting automatic fallback.");
    return false;
  }

  if (!approach_done) {
    RCLCPP_WARN(logger, "CIRC planning failed. Strategy 2/3: LIN with a stop at fixed via point.");
    step(5, "place approach (LIN to fixed via point)");
    bool lin_failed = false;
    if (movel(c, via_pose, &lin_failed)) {
      step(5, "place approach (LIN via point to goal)");
      if (movel(c, place_approach, &lin_failed)) {
        approach_done = true;
      } else if (!lin_failed) {
        RCLCPP_ERROR(logger, "LIN to goal non-planning failure; aborting fallback.");
        return false;
      }
    } else if (!lin_failed) {
      RCLCPP_ERROR(logger, "LIN to fixed via point non-planning failure; aborting fallback.");
      return false;
    }
  }

  if (!approach_done) {
    RCLCPP_WARN(logger, "LIN failed. Strategy 3/3: movej directly to place approach (no via).");
    step(5, "place approach (movej direct fallback)");
    if (!movej(c, place_approach)) {
      RCLCPP_ERROR(logger, "All three place-approach strategies failed.");
      return false;
    }
  }
  RCLCPP_INFO(logger, "Place approach reached successfully.");

  // jig_clamp <-> rebar and gripper <-> rebar OFF until after the place retract.
  step(5, "allow rebar/jig_clamp and gripper contact");
  if (!scene.setObjectCollisionsAllowed(rebar, kGripperLinks, true) ||
    !scene.setCollisionPairAllowed(rebar, kJigClampId, true))
  {
    scene.setObjectCollisionsAllowed(rebar, kGripperLinks, false);
    scene.setCollisionPairAllowed(rebar, kJigClampId, false);
    RCLCPP_ERROR(logger, "Failed to allow place contacts for '%s'.", rebar.c_str());
    return false;
  }
  ScopeGuard place_guard([&]() {
      scene.setObjectCollisionsAllowed(rebar, kGripperLinks, false);
      scene.setCollisionPairAllowed(rebar, kJigClampId, false);
    });

  step(6, "place (movel)");
  if (!movel(c, place)) {
    return false;
  }

  step(6, "detach " + rebar);
  if (!scene.detachObjectToWorldPreservingPose(*c.mg, rebar) ||
    !scene.verifyObjectState(rebar, true, false))
  {
    RCLCPP_ERROR(logger, "Failed to detach '%s'.", rebar.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "RELEASED: '%s' is now a world object.", rebar.c_str());

  step(7, "place retract (movel)");
  if (!movel(c, place_approach)) {
    RCLCPP_ERROR(logger, "'%s' was placed but the retract failed.", rebar.c_str());
    return false;
  }

  step(7, "restore collisions");
  place_guard.runNow();

  RCLCPP_INFO(logger, "---- pair %d/%d done ----", c.cur.pair_idx, c.cur.pair_total);
  return true;
}

// ---------------------------------------------------------------------------
// Explicit pick/place pair input from ROS 2 YAML parameters.
// Each entry: pick_frame,place_frame,rebar_id,pick_offset_x,pick_offset_y,pick_offset_z
// Offsets are applied to the pick pose in the WORLD frame, in metres.
// The place pose is taken directly from its TF frame.
// ---------------------------------------------------------------------------
struct PairInput
{
  std::string pick;
  std::string place;
  std::string rebar;
  std::array<double, 3> pick_offset = {0.0, 0.0, 0.0};
};

std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) { return ""; }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

bool parsePairInput(const std::string & spec, PairInput & result, std::string & error)
{
  std::vector<std::string> fields;
  std::stringstream stream(spec);
  std::string field;
  while (std::getline(stream, field, ',')) { fields.push_back(trim(field)); }

  if (fields.size() != 6) {
    error = "expected 6 comma-separated fields: pick_frame,place_frame,rebar_id,offset_x,offset_y,offset_z";
    return false;
  }
  if (fields[0].empty() || fields[1].empty() || fields[2].empty()) {
    error = "pick_frame, place_frame, and rebar_id must not be empty";
    return false;
  }
  std::array<double, 3> offsets{};
  for (std::size_t i = 0; i < 3; ++i) {
    try {
      std::size_t parsed_chars = 0;
      offsets[i] = std::stod(fields[i + 3], &parsed_chars);
      if (parsed_chars != fields[i + 3].size() || !std::isfinite(offsets[i])) {
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

enum class PairStatus { kOk, kFailed, kNotRun };

struct PairResult
{
  std::string pick, place, rebar;
  std::array<double, 3> pick_offset = {0.0, 0.0, 0.0};
  PairStatus status = PairStatus::kNotRun;
  Failure failure;
};

void printSummary(const std::vector<PairResult> & results)
{
  int ok = 0, failed = 0, not_run = 0;
  for (const auto & r : results) {
    if (r.status == PairStatus::kOk) {++ok;}
    else if (r.status == PairStatus::kNotRun) {++not_run;}
    else {++failed;}
  }

  std::printf("\n==================== RUN SUMMARY (robot2_arm) ====================\n");
  const int total = static_cast<int>(results.size());
  int idx = 0;
  for (const auto & r : results) {
    ++idx;
    if (r.status == PairStatus::kOk) {
      std::printf(
        "%d/%d %-7s -> %-14s %-14s | OK\n", idx, total,
        r.pick.c_str(), r.place.c_str(), r.rebar.c_str());
    } else if (r.status == PairStatus::kNotRun) {
      std::printf(
        "%d/%d %-7s -> %-14s %-14s | NOT RUN\n", idx, total,
        r.pick.c_str(), r.place.c_str(), r.rebar.c_str());
    } else {
      std::printf(
        "%d/%d %-7s -> %-14s %-14s | FAILED | step %d/%d %s | %s | %s | %s\n", idx, total,
        r.pick.c_str(), r.place.c_str(), r.rebar.c_str(),
        r.failure.info.step_idx, r.failure.info.step_total, r.failure.info.step.c_str(),
        r.failure.planner.c_str(), r.failure.stage.c_str(), errStr(r.failure.code).c_str());
    }
  }
  std::printf("\nPairs OK: %d / %d    FAILED: %d    NOT RUN: %d\n", ok, total, failed, not_run);
  std::printf("===================================================================\n");
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

        auto fjt = rclcpp_action::create_client<sa3_motion::FJT>(node, kControllerAction);
        if (!fjt->wait_for_action_server(10s)) {
          RCLCPP_ERROR(node->get_logger(), "Controller '%s' unavailable.", kControllerAction);
          return false;
        }

        // Same scene helper Robot 1 / Robot 3 use.
        PlanningSceneObjects scene_objects(node);
        if (kSpawnRebars && !scene_objects.addExtraRebars()) {
          RCLCPP_ERROR(node->get_logger(), "Failed to initialize rebar collision objects.");
          return false;
        }

        Ctx ctx;
        ctx.node = node;
        ctx.mg = &move_group;
        ctx.tf = &tf_buffer;
        ctx.scene = &scene_objects;
        ctx.fjt = fjt;

        // Use the same runtime-configurable parameter format as robot1_motion.
        // A task YAML passed through --params-file overrides these defaults.
        const std::vector<std::string> default_pair_specs = {
          "6pf,6_placef_r,rebar_6,0.0,0.0,0.0",
          "13_1pf,13_placef1_r,rebar_13_1_R,0.0,0.0,0.0",
          "13_1pf,13_placef2_r,rebar_13_2_R,0.118,0.0,0.0",
          "13_2pf,13_placef3_r,rebar_13_3_R,0.0,0.0,0.0",
          "14_4pf,14_placef4_r,rebar_14_1_R,0.0,0.0,0.0",
          "14_4pf,14_placef3_r,rebar_14_2_R,0.112,0.0,0.0",
          "14_4pf,14_placef2_r,rebar_14_3_R,0.224,0.0,0.0",
          "14_4pf,14_placef1_r,rebar_14_4_R,0.336,0.0,0.0"
        };
        if (!node->has_parameter("pick_place_pairs")) {
          node->declare_parameter<std::vector<std::string>>("pick_place_pairs", default_pair_specs);
        }
        std::vector<std::string> pair_specs;
        if (!node->get_parameter("pick_place_pairs", pair_specs) || pair_specs.empty()) {
          RCLCPP_ERROR(node->get_logger(),
            "Parameter 'pick_place_pairs' is empty or unavailable. Provide entries in task.yaml.");
          return false;
        }

        std::vector<PairResult> results;
        results.reserve(pair_specs.size());
        for (std::size_t i = 0; i < pair_specs.size(); ++i) {
          PairInput input;
          std::string parse_error;
          if (!parsePairInput(pair_specs[i], input, parse_error)) {
            RCLCPP_ERROR(node->get_logger(), "Invalid pick_place_pairs entry %zu ('%s'): %s",
              i + 1, pair_specs[i].c_str(), parse_error.c_str());
            return false;
          }
          PairResult r;
          r.pick = input.pick;
          r.place = input.place;
          r.rebar = input.rebar;
          r.pick_offset = input.pick_offset;
          results.push_back(std::move(r));
        }

        const int total = static_cast<int>(results.size());

        bool stop = false;
        for (int i = 0; i < total && !stop; ++i) {
          auto & r = results[i];
          ctx.cur = StepInfo{};
          ctx.cur.pair_idx = i + 1;
          ctx.cur.pair_total = total;
          ctx.cur.pick = r.pick;
          ctx.cur.place = r.place;
          ctx.cur.rebar = r.rebar;
          ctx.pick_offset = r.pick_offset;

          const size_t failures_before = ctx.failures.size();
          if (pickPlace(ctx)) {
            r.status = PairStatus::kOk;
          } else {
            // Still holding the rebar? Release it so the next pair starts clean.
            if (scene_objects.verifyObjectState(r.rebar, false, true, ctx.link)) {
              scene_objects.detachObjectToWorldPreservingPose(move_group, r.rebar);
            }
            // Make sure temporary allowances do not leak into the next pair.
            scene_objects.setCollisionPairAllowed(r.rebar, kWebRingId, false);
            scene_objects.setCollisionPairAllowed(r.rebar, kJigClampId, false);
            scene_objects.setObjectCollisionsAllowed(r.rebar, kGripperLinks, false);
            r.status = PairStatus::kFailed;
            if (ctx.failures.size() > failures_before) {
              r.failure = ctx.failures.back();
            }
            if (kStopOnFirstFailure) {
              stop = true;
              RCLCPP_ERROR(node->get_logger(), "Stopping: pair %d/%d failed.", i + 1, total);
            }
          }
        }

        printSummary(results);

        for (const auto & r : results) {
          if (r.status != PairStatus::kOk) {return false;}
        }
        RCLCPP_INFO(node->get_logger(), "All %d pick/place pairs completed.", total);
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
// Robot 4 arc pick-and-place with rebar_4c attached during movement.
//
// Sequence:
//   1. Plan and execute pick approach.
//   2. Allow collision between rebar_4c and Robot 4 touch links.
//   3. Move to pick pose.
//   4. Attach rebar_4c to robot4_end_effactor.
//   5. Retract with the rebar attached (rebar/inventory collision allowed
//      from attach until this retract completes).
//   6. Follow the circular arc through three TF frames.
//   7. Move to the place approach.
//   8. Move to place pose.
//   9. Detach rebar_4c into the world at the place frame.
//  10. Retract and restore collision checking.
//
// Parameters (all optional):
//   pick_frame   : TF frame of the pick pose
//   pick_offset  : [x, y, z] world-frame offset from pick_frame to the REAL pick pose
//   place_frame  : TF frame of the place pose
//   place_offset : [x, y, z] world-frame offset from place_frame to the REAL place pose
//   object_id    : planning-scene object to carry
//
// The approach/retract offsets (kPickOffset / kPlaceOffset) are applied on top
// of the already-shifted pick/place poses.
//
// Trajectory execution is sent directly to Robot 4's controller.
// The Planning Scene is still managed by MoveIt.
//
// IMPORTANT:
//   No physical gripper actuation is implemented here.
//   Test the attachment, mesh pose, and full trajectory in mock/RViz.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sa3_motion/direct_execute.hpp"
#include "sa3_motion/planning_scene_objects.hpp"

using namespace std::chrono_literals;

using MoveItErr = moveit_msgs::msg::MoveItErrorCodes;

namespace
{

// ===========================================================================
// Robot / frames
// ===========================================================================

constexpr char kNodeName[] = "robot4_pick_place";

constexpr char kGroup[] = "robot4_arm";
constexpr char kPlanningLink[] = "robot4_end_effactor_ff";
constexpr char kReferenceFrame[] = "world";

constexpr char kControllerAction[] =
  "/robot4_arm_controller/follow_joint_trajectory";

// The ID must match the key in workcell.yaml, not the mesh filename.
constexpr char kDefaultPickFrame[] = "inventory_3a_4a_frame_2";
constexpr char kDefaultPlaceFrame[] = "place_frame_10";
constexpr char kDefaultObjectId[] = "rebar_4c";

// Planning-scene link name for the placement fixture.
constexpr char kJigClampName[] = "jig_clamp";

// Inventory collision object the picked rebar starts in. Collision between
// the ATTACHED rebar and this object is allowed from attach until the
// pick retract is complete.
constexpr char kInventoryName[] = "inventory_3a_4a_2";

constexpr char kJ6Link[] = "robot4_J6_link";


// ===========================================================================
// Approach / retract offsets (world frame), relative to the pick/place pose
// ===========================================================================

constexpr std::array<double, 3> kPickOffset = {
  0.6, 0.00, 0.6
};

constexpr std::array<double, 3> kPlaceOffset = {
  0.0, -0.25, 0.0
};


// ===========================================================================
// Planning settings
// ===========================================================================

constexpr double kPlanningTime = 15.0;

constexpr int kOmplAttempts = 10;
constexpr int kPlanRetries = 3;

constexpr double kMoveJVelocity = 0.30;
constexpr double kMoveJAcceleration = 0.20;

constexpr double kMoveLVelocity = 0.20;
constexpr double kMoveLAcceleration = 0.10;

constexpr int kStepsPerPair = 6;

constexpr char kHomeState[] = "home";


// ===========================================================================
// Pose helpers
// ===========================================================================

geometry_msgs::msg::Pose offsetWorld(
  const geometry_msgs::msg::Pose & original,
  const std::array<double, 3> & offset)
{
  auto result = original;

  result.position.x += offset[0];
  result.position.y += offset[1];
  result.position.z += offset[2];

  return result;
}


std::string poseStr(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  char buffer[250];

  std::snprintf(
    buffer,
    sizeof(buffer),
    "xyz=[%.4f, %.4f, %.4f] rpy=[%.4f, %.4f, %.4f]",
    pose.position.x,
    pose.position.y,
    pose.position.z,
    roll,
    pitch,
    yaw);

  return buffer;
}


// ===========================================================================
// ARC through three poses
//
// Position: circle through the three positions.
// Orientation: quaternion slerp a->b->c.
// Returns empty if the points are collinear or invalid.
// ===========================================================================

std::vector<geometry_msgs::msg::Pose> arcThrough3(
  const geometry_msgs::msg::Pose & a,
  const geometry_msgs::msg::Pose & b,
  const geometry_msgs::msg::Pose & c,
  int n)
{
  using tf2::Vector3;

  auto pos = [](const geometry_msgs::msg::Pose & pose) {
      return Vector3(
        pose.position.x,
        pose.position.y,
        pose.position.z);
    };

  auto quat = [](const geometry_msgs::msg::Pose & pose) {
      tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
      q.normalize();
      return q;
    };

  n = std::max(n, 2);

  const Vector3 A = pos(a);
  const Vector3 B = pos(b);
  const Vector3 C = pos(c);

  const Vector3 u = B - A;
  const Vector3 v = C - A;
  const Vector3 w = u.cross(v);

  const double w2 = w.length2();

  if (w2 < 1e-12) {
    return {};
  }

  const Vector3 center =
    A + ((v * u.length2() - u * v.length2()).cross(w)) /
    (2.0 * w2);

  const double radius = (A - center).length();

  if (radius < 1e-9) {
    return {};
  }

  const Vector3 e1 = (A - center) / radius;
  const Vector3 normal = w.normalized();
  const Vector3 e2 = normal.cross(e1);

  auto angleOf = [&](const Vector3 & point) {
      const Vector3 d = point - center;
      return std::atan2(d.dot(e2), d.dot(e1));
    };

  const double angle_b = angleOf(B);
  double angle_c = angleOf(C);

  constexpr double kPi = 3.14159265358979323846;

  // Select sweep direction that passes through B.
  if (angle_b > 0.0) {
    if (angle_c < angle_b) {
      angle_c += 2.0 * kPi;
    }
  } else {
    if (angle_c > angle_b) {
      angle_c -= 2.0 * kPi;
    }
  }

  const double sweep = angle_c;

  if (std::abs(sweep) < 1e-9) {
    return {};
  }

  double fraction_b = angle_b / sweep;
  fraction_b = std::clamp(fraction_b, 0.0, 1.0);

  const tf2::Quaternion qa = quat(a);
  const tf2::Quaternion qb = quat(b);
  const tf2::Quaternion qc = quat(c);

  std::vector<geometry_msgs::msg::Pose> output;
  output.reserve(static_cast<std::size_t>(n));

  for (int i = 0; i < n; ++i) {
    const double f =
      static_cast<double>(i) / static_cast<double>(n - 1);

    const double theta = f * sweep;

    const Vector3 point =
      center + radius *
      (std::cos(theta) * e1 + std::sin(theta) * e2);

    tf2::Quaternion q;

    if (f <= fraction_b && fraction_b > 1e-9) {
      q = qa.slerp(qb, f / fraction_b);
    } else if (fraction_b < 1.0 - 1e-9) {
      const double local_fraction =
        (f - fraction_b) / (1.0 - fraction_b);
      q = qb.slerp(qc, local_fraction);
    } else {
      q = qc;
    }

    q.normalize();

    geometry_msgs::msg::Pose pose;

    pose.position.x = point.x();
    pose.position.y = point.y();
    pose.position.z = point.z();

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    output.push_back(pose);
  }

  return output;
}


// ===========================================================================
// Failure / step context
// ===========================================================================

struct StepInfo
{
  int step_idx = 0;
  int step_total = kStepsPerPair;

  std::string pick;
  std::string place;
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

  rclcpp_action::Client<sa3_motion::FJT>::SharedPtr fjt;

  // Shared SA3 Planning Scene helper.
  PlanningSceneObjects * scene = nullptr;

  std::string link = kPlanningLink;
  std::string object_id = kDefaultObjectId;

  // World-frame offsets from the TF pick/place frames to the real
  // pick/place poses (set from the pick_offset / place_offset parameters).
  std::array<double, 3> pick_offset = {0.0, 0.0, 0.0};
  std::array<double, 3> place_offset = {0.0, 0.0, 0.0};

  StepInfo cur;

  std::vector<Failure> failures;
};


std::string errStr(int code)
{
  switch (code) {
    case MoveItErr::SUCCESS:
      return "SUCCESS";
    case MoveItErr::FAILURE:
      return "FAILURE";
    case MoveItErr::PLANNING_FAILED:
      return "PLANNING_FAILED";
    case MoveItErr::INVALID_MOTION_PLAN:
      return "INVALID_MOTION_PLAN";
    case MoveItErr::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case MoveItErr::CONTROL_FAILED:
      return "CONTROL_FAILED";
    case MoveItErr::TIMED_OUT:
      return "TIMED_OUT";
    case MoveItErr::PREEMPTED:
      return "PREEMPTED";
    case MoveItErr::START_STATE_IN_COLLISION:
      return "START_STATE_IN_COLLISION";
    case MoveItErr::GOAL_IN_COLLISION:
      return "GOAL_IN_COLLISION";
    case MoveItErr::INVALID_GROUP_NAME:
      return "INVALID_GROUP_NAME";
    case MoveItErr::INVALID_LINK_NAME:
      return "INVALID_LINK_NAME";
    case MoveItErr::FRAME_TRANSFORM_FAILURE:
      return "FRAME_TRANSFORM_FAILURE";
    case MoveItErr::NO_IK_SOLUTION:
      return "NO_IK_SOLUTION";
    default:
      return "CODE_" + std::to_string(code);
  }
}


std::string stepTag(const Ctx & ctx)
{
  std::ostringstream out;

  out
    << "[step "
    << ctx.cur.step_idx
    << "/"
    << ctx.cur.step_total
    << " | "
    << ctx.cur.pick
    << " -> "
    << ctx.cur.place
    << " | "
    << ctx.cur.step
    << "] ";

  return out.str();
}


void reportFailure(
  Ctx & ctx,
  const std::string & planner,
  const std::string & stage,
  int code,
  double plan_time,
  const geometry_msgs::msg::Pose * target,
  const std::string & note = "")
{
  Failure failure;

  failure.info = ctx.cur;
  failure.planner = planner;
  failure.stage = stage;
  failure.code = code;
  failure.plan_time = plan_time;

  ctx.failures.push_back(failure);

  std::ostringstream out;

  out
    << "\n================ MOTION FAILURE ================\n"
    << "Step: " << ctx.cur.step_idx
    << "/" << ctx.cur.step_total << " - " << ctx.cur.step << "\n"
    << "Pick: " << ctx.cur.pick << "\n"
    << "Place: " << ctx.cur.place << "\n"
    << "Planner: " << planner << "\n"
    << "Stage: " << stage << "\n"
    << "Error: " << errStr(code) << " (" << code << ")\n"
    << "Planning time: " << std::fixed << std::setprecision(2)
    << plan_time << " s\n";

  if (target != nullptr) {
    out << "Target: " << poseStr(*target) << "\n";
  }

  if (!note.empty()) {
    out << "Note: " << note << "\n";
  }

  out << "=================================================";

  RCLCPP_ERROR(
    ctx.node->get_logger(),
    "%s",
    out.str().c_str());
}


// ===========================================================================
// TF lookup
// ===========================================================================

bool lookupFramePose(
  Ctx & ctx,
  const std::string & frame,
  geometry_msgs::msg::Pose & pose)
{
  try {
    const auto transform = ctx.tf->lookupTransform(
      kReferenceFrame,
      frame,
      tf2::TimePointZero,
      tf2::durationFromSec(10.0));

    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;

    pose.orientation = transform.transform.rotation;

    return true;
  }
  catch (const tf2::TransformException & error) {
    reportFailure(
      ctx,
      "TF",
      "frame lookup",
      MoveItErr::FRAME_TRANSFORM_FAILURE,
      0.0,
      nullptr,
      "Unable to resolve frame '" + frame + "': " + error.what());

    return false;
  }
}


// ===========================================================================
// MoveJ-style pose target: plan with MoveIt, execute through the controller
// ===========================================================================

bool movej(
  Ctx & ctx,
  const geometry_msgs::msg::Pose & target)
{
  auto & group = *ctx.mg;
  const auto logger = ctx.node->get_logger();
  const std::string tag = stepTag(ctx);

  if (!group.getCurrentState(10.0)) {
    reportFailure(
      ctx, "OMPL", "current state",
      MoveItErr::ROBOT_STATE_STALE, 0.0, &target);
    return false;
  }

  group.setStartStateToCurrentState();
  group.setPlanningPipelineId("ompl");
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPlanningTime(kPlanningTime);
  group.setNumPlanningAttempts(kOmplAttempts);
  group.setMaxVelocityScalingFactor(kMoveJVelocity);
  group.setMaxAccelerationScalingFactor(kMoveJAcceleration);
  group.setGoalPositionTolerance(0.001);
  group.setGoalOrientationTolerance(0.01);
  group.clearPoseTargets();
  group.clearPathConstraints();

  RCLCPP_INFO(
    logger, "%smovej target %s",
    tag.c_str(), poseStr(target).c_str());

  if (!group.setPoseTarget(target, ctx.link)) {
    reportFailure(
      ctx, "OMPL", "set target",
      MoveItErr::FAILURE, 0.0, &target,
      "MoveIt rejected the pose target.");
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  int last_code = MoveItErr::FAILURE;
  bool planned = false;

  const auto start = std::chrono::steady_clock::now();

  for (int attempt = 1; attempt <= kPlanRetries; ++attempt) {
    const auto result = group.plan(plan);
    last_code = result.val;

    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      planned = true;
      break;
    }

    RCLCPP_WARN(
      logger,
      "%sOMPL planning attempt %d/%d failed: %s",
      tag.c_str(), attempt, kPlanRetries, errStr(last_code).c_str());
  }

  const double elapsed =
    std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();

  group.clearPoseTargets();

  if (!planned) {
    reportFailure(
      ctx, "OMPL", "planning",
      last_code, elapsed, &target,
      "All pose-planning attempts failed.");
    return false;
  }

  int exec_code = 0;
  std::string exec_note;

  if (!sa3_motion::executeDirect(
      ctx.fjt,
      plan.trajectory.joint_trajectory,
      exec_code,
      exec_note))
  {
    reportFailure(
      ctx, "OMPL", "direct controller execution",
      exec_code, elapsed, &target, exec_note);
    return false;
  }

  RCLCPP_INFO(
    logger, "%smovej completed in %.2f s.",
    tag.c_str(), elapsed);

  return true;
}


// ===========================================================================
// MoveL-style pose target: Pilz LIN, direct controller execution
// ===========================================================================

bool movel(
  Ctx & ctx,
  const geometry_msgs::msg::Pose & target)
{
  auto & group = *ctx.mg;
  const auto logger = ctx.node->get_logger();
  const std::string tag = stepTag(ctx);

  if (!group.getCurrentState(10.0)) {
    reportFailure(
      ctx, "LIN", "current state",
      MoveItErr::ROBOT_STATE_STALE, 0.0, &target);
    return false;
  }

  group.setStartStateToCurrentState();
  group.setPlanningPipelineId("pilz_industrial_motion_planner");
  group.setPlannerId("LIN");
  group.setPlanningTime(kPlanningTime);
  group.setMaxVelocityScalingFactor(kMoveLVelocity);
  group.setMaxAccelerationScalingFactor(kMoveLAcceleration);
  group.clearPoseTargets();
  group.clearPathConstraints();

  RCLCPP_INFO(
    logger, "%smovel target %s",
    tag.c_str(), poseStr(target).c_str());

  if (!group.setPoseTarget(target, ctx.link)) {
    reportFailure(
      ctx, "LIN", "set target",
      MoveItErr::FAILURE, 0.0, &target,
      "MoveIt rejected the pose target.");
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  const auto start = std::chrono::steady_clock::now();
  const auto result = group.plan(plan);

  const double elapsed =
    std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();

  group.clearPoseTargets();

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    reportFailure(
      ctx, "LIN", "planning",
      result.val, elapsed, &target);
    return false;
  }

  int exec_code = 0;
  std::string exec_note;

  if (!sa3_motion::executeDirect(
      ctx.fjt,
      plan.trajectory.joint_trajectory,
      exec_code,
      exec_note))
  {
    reportFailure(
      ctx, "LIN", "direct controller execution",
      exec_code, elapsed, &target, exec_note);
    return false;
  }

  RCLCPP_INFO(
    logger, "%smovel completed in %.2f s.",
    tag.c_str(), elapsed);

  return true;
}


// ===========================================================================
// Go to the named home state: plan with OMPL, execute through the controller
// ===========================================================================

bool goHome(Ctx & ctx)
{
  auto & group = *ctx.mg;
  const auto logger = ctx.node->get_logger();
  const std::string tag = stepTag(ctx);

  if (!group.getCurrentState(10.0)) {
    reportFailure(
      ctx, "OMPL", "current state",
      MoveItErr::ROBOT_STATE_STALE, 0.0, nullptr);
    return false;
  }

  group.setStartStateToCurrentState();
  group.setPlanningPipelineId("ompl");
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPlanningTime(kPlanningTime);
  group.setNumPlanningAttempts(kOmplAttempts);
  group.setMaxVelocityScalingFactor(kMoveJVelocity);
  group.setMaxAccelerationScalingFactor(kMoveJAcceleration);
  group.clearPoseTargets();
  group.clearPathConstraints();

  RCLCPP_INFO(logger, "%sgoing to named state '%s'", tag.c_str(), kHomeState);

  if (!group.setNamedTarget(kHomeState)) {
    reportFailure(
      ctx, "OMPL", "set target",
      MoveItErr::FAILURE, 0.0, nullptr,
      std::string("Named state '") + kHomeState +
      "' not found for group '" + kGroup + "' in the SRDF.");
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  int last_code = MoveItErr::FAILURE;
  bool planned = false;

  const auto start = std::chrono::steady_clock::now();

  for (int attempt = 1; attempt <= kPlanRetries; ++attempt) {
    const auto result = group.plan(plan);
    last_code = result.val;

    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      planned = true;
      break;
    }

    RCLCPP_WARN(
      logger, "%shome planning attempt %d/%d failed: %s",
      tag.c_str(), attempt, kPlanRetries, errStr(last_code).c_str());
  }

  const double elapsed =
    std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();

  if (!planned) {
    reportFailure(
      ctx, "OMPL", "planning",
      last_code, elapsed, nullptr,
      "Could not plan to the home state.");
    return false;
  }

  int exec_code = 0;
  std::string exec_note;

  if (!sa3_motion::executeDirect(
      ctx.fjt,
      plan.trajectory.joint_trajectory,
      exec_code,
      exec_note))
  {
    reportFailure(
      ctx, "OMPL", "direct controller execution",
      exec_code, elapsed, nullptr, exec_note);
    return false;
  }

  RCLCPP_INFO(logger, "%shome reached in %.2f s.", tag.c_str(), elapsed);

  return true;
}

// ===========================================================================
// Visit a list of arc poses in order using MoveL (MoveJ fallback)
// ===========================================================================


bool followPosesJ(
  Ctx & ctx,
  const std::vector<geometry_msgs::msg::Pose> & poses,
  int first_step_idx)
{
  const auto logger = ctx.node->get_logger();

  auto movel_or_movej =
    [&](const geometry_msgs::msg::Pose & target,
        const std::string & name,
        int n) -> bool
    {
      // First attempt: Cartesian linear motion using Pilz LIN.
      ctx.cur.step_idx = n;
      ctx.cur.step = name + " (movel)";

      if (movel(ctx, target)) {
        return true;
      }

      // Fallback: retry the same target using OMPL MoveJ.
      RCLCPP_WARN(
        logger,
        "%sMoveL failed for %s. Trying MoveJ fallback.",
        stepTag(ctx).c_str(),
        name.c_str());

      ctx.cur.step_idx = n;
      ctx.cur.step = name + " (movej fallback)";

      if (movej(ctx, target)) {
        RCLCPP_INFO(
          logger,
          "%sMoveJ fallback succeeded for %s.",
          stepTag(ctx).c_str(),
          name.c_str());

        return true;
      }

      RCLCPP_ERROR(
        logger,
        "%sBoth MoveL and MoveJ failed for %s.",
        stepTag(ctx).c_str(),
        name.c_str());

      return false;
    };

  for (std::size_t i = 0; i < poses.size(); ++i) {
    const int step_idx =
      first_step_idx + static_cast<int>(i);

    const std::string name =
      "arc point " + std::to_string(i + 1) + "/" +
      std::to_string(poses.size());

    if (!movel_or_movej(poses[i], name, step_idx)) {
      return false;
    }
  }

  return true;
}



// ===========================================================================
// ROBOT 4 PICK AND PLACE WITH REBAR ATTACHMENT
// ===========================================================================

bool pickPlace(
  Ctx & ctx,
  const std::array<std::string, 3> & arc_frames,
  int arc_points)
{
  const auto logger = ctx.node->get_logger();

  if (ctx.scene == nullptr) {
    RCLCPP_ERROR(logger, "PlanningSceneObjects was not supplied.");
    return false;
  }

  arc_points = std::max(arc_points, 2);

  ctx.cur.step_total = kStepsPerPair + arc_points + 1; // +1 for final home

  int step = 0;

  const std::string pick_frame = ctx.cur.pick;
  const std::string place_frame = ctx.cur.place;

  const std::vector<std::string> touch_links = {
    ctx.link,
    kJ6Link,
    "robot4_sa3_grpr01_r"
  };

  RCLCPP_INFO(logger, "==============================================");
  RCLCPP_INFO(logger, "Robot 4 Pick & Place with rebar attachment");
  RCLCPP_INFO(logger, "Rebar ID    : %s", ctx.object_id.c_str());
  RCLCPP_INFO(logger, "Pick frame  : %s", pick_frame.c_str());
  RCLCPP_INFO(
    logger, "Pick offset : [%.3f, %.3f, %.3f]",
    ctx.pick_offset[0], ctx.pick_offset[1], ctx.pick_offset[2]);
  RCLCPP_INFO(logger, "Place frame : %s", place_frame.c_str());
  RCLCPP_INFO(
    logger, "Place offset: [%.3f, %.3f, %.3f]",
    ctx.place_offset[0], ctx.place_offset[1], ctx.place_offset[2]);
  RCLCPP_INFO(
    logger,
    "Arc frames  : %s, %s, %s (%d points)",
    arc_frames[0].c_str(),
    arc_frames[1].c_str(),
    arc_frames[2].c_str(),
    arc_points);
  RCLCPP_INFO(logger, "==============================================");

  // ----------------------------------------------------------
  // Look up pick, place, and arc frames before moving.
  // ----------------------------------------------------------

  ctx.cur.step_idx = 0;
  ctx.cur.step = "frame lookup";

  geometry_msgs::msg::Pose pick_frame_pose;
  geometry_msgs::msg::Pose place_frame_pose;

  if (!lookupFramePose(ctx, pick_frame, pick_frame_pose) ||
      !lookupFramePose(ctx, place_frame, place_frame_pose))
  {
    return false;
  }

  // Actual pick/place poses = TF frame pose + user offset (world frame).
  const auto pick = offsetWorld(pick_frame_pose, ctx.pick_offset);
  const auto place = offsetWorld(place_frame_pose, ctx.place_offset);

  geometry_msgs::msg::Pose arc_a;
  geometry_msgs::msg::Pose arc_b;
  geometry_msgs::msg::Pose arc_c;

  if (!lookupFramePose(ctx, arc_frames[0], arc_a) ||
      !lookupFramePose(ctx, arc_frames[1], arc_b) ||
      !lookupFramePose(ctx, arc_frames[2], arc_c))
  {
    return false;
  }

  // The arc consists of TF-defined target poses. Each segment is
  // planned only when reached, so planning happens with 4C attached.
  const auto arc = arcThrough3(arc_a, arc_b, arc_c, arc_points);

  if (arc.empty()) {
    RCLCPP_ERROR(
      logger,
      "Arc frames are collinear or invalid; no unique arc can be generated.");
    return false;
  }

  RCLCPP_INFO(
    logger,
    "Generated %zu arc points through the three specified frames.",
    arc.size());

  RCLCPP_INFO(logger, "Pick frame pose  : %s", poseStr(pick_frame_pose).c_str());
  RCLCPP_INFO(logger, "Place frame pose : %s", poseStr(place_frame_pose).c_str());
  RCLCPP_INFO(logger, "Pick pose  (with offset): %s", poseStr(pick).c_str());
  RCLCPP_INFO(logger, "Place pose (with offset): %s", poseStr(place).c_str());

  const auto pick_approach = offsetWorld(pick, kPickOffset);
  const auto place_approach = offsetWorld(place, kPlaceOffset);

  RCLCPP_INFO(
    logger,
    "Pick approach  : %s",
    poseStr(pick_approach).c_str());

  RCLCPP_INFO(
    logger,
    "Place approach : %s",
    poseStr(place_approach).c_str());

  // Ensure the expected collision object exists in WORLD state.
  if (!ctx.scene->verifyObjectState(ctx.object_id, true, false)) {
    RCLCPP_ERROR(
      logger,
      "Expected '%s' to be a world object before picking.",
      ctx.object_id.c_str());
    return false;
  }

  // ----------------------------------------------------------
  // STEP 1: MoveJ to pick approach.
  //
  // Contact allowance is not yet enabled.
  // ----------------------------------------------------------

  ctx.cur.step_idx = ++step;
  ctx.cur.step = "pick approach (movej)";

  if (!movej(ctx, pick_approach)) {
    return false;
  }

  // ----------------------------------------------------------
  // STEP 2: Allow only the requested rebar/gripper link pairs.
  //
  // We do not disable collision checking for the entire rebar.
  // ----------------------------------------------------------

  ctx.cur.step_idx = ++step;
  ctx.cur.step = "allow rebar/gripper contact";

  if (!ctx.scene->setObjectCollisionsAllowed(
      ctx.object_id,
      touch_links,
      true))
  {
    RCLCPP_ERROR(
      logger,
      "Failed to allow contact between '%s' and Robot 4's touch links.",
      ctx.object_id.c_str());
    return false;
  }

  // ----------------------------------------------------------
  // STEP 3: MoveL to the pick pose.
  // ----------------------------------------------------------

  ctx.cur.step_idx = ++step;
  ctx.cur.step = "pick (movel)";

  if (!movel(ctx, pick)) {
    // It has not been attached. Restore collision checking.
    if (!ctx.scene->setObjectCollisionsAllowed(
        ctx.object_id,
        touch_links,
        false))
    {
      RCLCPP_ERROR(
        logger,
        "Failed to restore collision checking after pick motion failure.");
    }

    return false;
  }

  // ----------------------------------------------------------
  // STEP 4: Attach the object to the planning link.
  //
  // MoveIt calculates the attached object's relative transform
  // from its world pose and the robot link pose. No mesh-origin
  // alignment to the TCP is performed here.
  // ----------------------------------------------------------

  ctx.cur.step = "attach " + ctx.object_id;

  if (!ctx.scene->attachWorldObjectPreservingPose(
      *ctx.mg,
      ctx.object_id,
      ctx.link,
      touch_links))
  {
    RCLCPP_ERROR(
      logger,
      "Failed to attach '%s' to '%s'. Stopping before retract.",
      ctx.object_id.c_str(),
      ctx.link.c_str());

    // If it remains in WORLD state, undo the temporary pair allowance.
    // If it became attached despite a timeout, leave it attached for
    // controlled recovery.
    if (ctx.scene->verifyObjectState(ctx.object_id, true, false)) {
      ctx.scene->setObjectCollisionsAllowed(
        ctx.object_id,
        touch_links,
        false);
    }

    return false;
  }

  if (!ctx.scene->verifyObjectState(
      ctx.object_id,
      false,
      true,
      ctx.link))
  {
    RCLCPP_ERROR(
      logger,
      "Attachment state verification failed for '%s'.",
      ctx.object_id.c_str());
    return false;
  }

  RCLCPP_INFO(
    logger,
    "ATTACHED: %s now follows link '%s'.",
    ctx.object_id.c_str(),
    ctx.link.c_str());

  // ----------------------------------------------------------
  // Allow the attached rebar to touch the inventory it is being
  // lifted out of. Done after attach so the attached body's ACM
  // entry is the one that gets changed. Only this pair is allowed;
  // robot-to-inventory collision checking stays enabled.
  // ----------------------------------------------------------

  ctx.cur.step = "allow rebar/inventory contact for pick retract";

  if (!ctx.scene->setObjectCollisionsAllowed(
      ctx.object_id,
      {kInventoryName},
      true))
  {
    RCLCPP_ERROR(
      logger,
      "Could not allow collision between '%s' and '%s'. "
      "'%s' remains attached for recovery.",
      ctx.object_id.c_str(),
      kInventoryName,
      ctx.object_id.c_str());
    return false;
  }

  RCLCPP_INFO(
    logger,
    "Collision allowed: '%s' <-> '%s' until pick retract completes.",
    ctx.object_id.c_str(),
    kInventoryName);

  // ----------------------------------------------------------
  // STEP 5: Retract from pick with the rebar attached.
  // ----------------------------------------------------------

  ctx.cur.step_idx = ++step;
  ctx.cur.step = "pick retract (movel)";

  if (!movel(ctx, pick_approach)) {
    RCLCPP_ERROR(
      logger,
      "Retract failed. '%s' remains attached for recovery; "
      "its inventory contact allowance stays enabled.",
      ctx.object_id.c_str());
    return false;
  }

  // Pick retract complete: restore normal rebar/inventory collision checking.
  ctx.cur.step = "restore rebar/inventory collision checking";

  if (!ctx.scene->setObjectCollisionsAllowed(
      ctx.object_id,
      {kInventoryName},
      false))
  {
    RCLCPP_ERROR(
      logger,
      "Pick retract finished, but collision checking between '%s' and '%s' "
      "could not be restored.",
      ctx.object_id.c_str(),
      kInventoryName);
    return false;
  }

  RCLCPP_INFO(
    logger,
    "Collision restored: '%s' <-> '%s' checked again.",
    ctx.object_id.c_str(),
    kInventoryName);

  // ----------------------------------------------------------
  // STEPS 6...: Follow the circular arc with the rebar attached.
  //
  // Each MoveL is planned after attachment. MoveIt therefore
  // includes the attached rebar geometry in those plans.
  // ----------------------------------------------------------

  if (!followPosesJ(ctx, arc, step + 1)) {
    RCLCPP_ERROR(
      logger,
      "Arc motion failed even after MoveJ fallback. "
      "'%s' remains attached for recovery.",
      ctx.object_id.c_str());
    return false;
  }
  step += static_cast<int>(arc.size());

  // ----------------------------------------------------------
  // Allow the carried rebar to contact the jig clamp for placement.
  // This changes only the object <-> jig_clamp collision pair;
  // collisions between the robot links and jig_clamp stay enabled.
  // Keep this pair allowed after release because the rebar is placed
  // in/against the fixture. The helper updates the MoveIt ACM.
  // ----------------------------------------------------------

  ctx.cur.step = "allow rebar/jig_clamp contact for placement";

  if (!ctx.scene->setObjectCollisionsAllowed(
      ctx.object_id,
      {kJigClampName},
      true))
  {
    RCLCPP_ERROR(
      logger,
      "Could not allow collision between '%s' and '%s'; stopping before placement.",
      ctx.object_id.c_str(),
      kJigClampName);
    return false;
  }

  RCLCPP_WARN(
    logger,
    "Placement contact enabled: '%s' may contact '%s'. "
    "Robot-to-clamp collision checking remains enabled.",
    ctx.object_id.c_str(),
    kJigClampName);

  // ----------------------------------------------------------
  // Move to the place approach while carrying the object.
  // ----------------------------------------------------------

  ctx.cur.step_idx = ++step;
  ctx.cur.step = "place approach (movel)";

  if (!movel(ctx, place_approach)) {
    RCLCPP_ERROR(
      logger,
      "Place approach failed. '%s' remains attached.",
      ctx.object_id.c_str());
    return false;
  }

  // ----------------------------------------------------------
  // MoveL to the actual place target (place frame + place_offset).
  // ----------------------------------------------------------

  ctx.cur.step_idx = ++step;
  ctx.cur.step = "place (movel)";

  if (!movel(ctx, place)) {
    RCLCPP_ERROR(
      logger,
      "Place motion failed. '%s' remains attached.",
      ctx.object_id.c_str());
    return false;
  }

  // ----------------------------------------------------------
  // Detach at the actual release pose.
  //
  // The implementation calculates the object's world pose from
  // the current attachment-link pose and stored relative pose.
  // It transfers attached -> WORLD in the Planning Scene.
  // ----------------------------------------------------------

  ctx.cur.step = "detach " + ctx.object_id;

  if (!ctx.scene->detachObjectToWorldPreservingPose(
      *ctx.mg,
      ctx.object_id))
  {
    RCLCPP_ERROR(
      logger,
      "Detachment failed for '%s'. Check its current ownership.",
      ctx.object_id.c_str());
    return false;
  }

  if (!ctx.scene->verifyObjectState(ctx.object_id, true, false)) {
    RCLCPP_ERROR(
      logger,
      "'%s' was not verified in the world after detachment.",
      ctx.object_id.c_str());
    return false;
  }

  RCLCPP_INFO(
    logger,
    "RELEASED: '%s' is now a world collision object.",
    ctx.object_id.c_str());

  // ----------------------------------------------------------
  // Retract while pair-specific contact allowance stays enabled.
  // ----------------------------------------------------------

  ctx.cur.step_idx = ++step;
  ctx.cur.step = "place retract (movel)";

  if (!movel(ctx, place_approach)) {
    RCLCPP_ERROR(
      logger,
      "Rebar was placed, but retreat failed. Its contact allowance "
      "remains enabled for recovery.");
    return false;
  }

  // ----------------------------------------------------------
  // Restore normal collision checking after retreat.
  // ----------------------------------------------------------

  ctx.cur.step = "restore rebar/gripper collision checking";

  if (!ctx.scene->setObjectCollisionsAllowed(
      ctx.object_id,
      touch_links,
      false))
  {
    RCLCPP_ERROR(
      logger,
      "Rebar was placed, but collision checking could not be restored.");
    return false;
  }

  // ----------------------------------------------------------
  // Final step: return the arm to its home state.
  // ----------------------------------------------------------

  ctx.cur.step_idx = ++step;
  ctx.cur.step = "go home";

  if (!goHome(ctx)) {
    RCLCPP_ERROR(
      logger,
      "Rebar was placed, but the arm could not return home.");
    return false;
  }

  RCLCPP_INFO(logger, "==============================================");
  RCLCPP_INFO(logger, "Robot 4 pick/place completed successfully.");
  RCLCPP_INFO(logger, "Rebar '%s' was placed and detached.", ctx.object_id.c_str());
  RCLCPP_INFO(logger, "==============================================");

  return true;
}

// Declare (if needed) and read a 3-element double-array parameter.
// Returns false if the parameter does not have exactly 3 values.
bool readOffsetParam(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  std::array<double, 3> & out)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<std::vector<double>>(
      name, std::vector<double>{0.0, 0.0, 0.0});
  }

  const auto values = node->get_parameter(name).as_double_array();

  if (values.size() != 3) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Parameter '%s' must have exactly 3 values [x, y, z], got %zu.",
      name.c_str(), values.size());
    return false;
  }

  out = {values[0], values[1], values[2]};
  return true;
}

}  // namespace


// ===========================================================================
// MAIN
// ===========================================================================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const auto node_options =
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);

  auto node = rclcpp::Node::make_shared(
    kNodeName,
    node_options);

  // These parameters can be overridden from a launch file, YAML file, or CLI.
  // Preserve the original behavior when no overrides are supplied.
  if (!node->has_parameter("pick_frame")) {
    node->declare_parameter<std::string>(
      "pick_frame", kDefaultPickFrame);
  }
  if (!node->has_parameter("place_frame")) {
    node->declare_parameter<std::string>(
      "place_frame", kDefaultPlaceFrame);
  }
  if (!node->has_parameter("object_id")) {
    node->declare_parameter<std::string>(
      "object_id", kDefaultObjectId);
  }

  const std::string pick_frame =
    node->get_parameter("pick_frame").as_string();
  const std::string place_frame =
    node->get_parameter("place_frame").as_string();
  const std::string object_id =
    node->get_parameter("object_id").as_string();

  if (pick_frame.empty() || place_frame.empty() || object_id.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Parameters 'pick_frame', 'place_frame', and 'object_id' must not be empty.");
    rclcpp::shutdown();
    return 1;
  }

  // World-frame offsets from the TF frames to the real pick/place poses.
  std::array<double, 3> pick_offset = {0.0, 0.0, 0.0};
  std::array<double, 3> place_offset = {0.0, 0.0, 0.0};

  if (!readOffsetParam(node, "pick_offset", pick_offset) ||
      !readOffsetParam(node, "place_offset", place_offset))
  {
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread executor_thread(
    [&executor]() {
      executor.spin();
    });

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf_buffer.setUsingDedicatedThread(true);

  tf2_ros::TransformListener tf_listener(
    tf_buffer,
    node,
    false);

  bool ok = false;

  try {
    ok = [&]() -> bool {
      RCLCPP_INFO(node->get_logger(), "==============================================");
      RCLCPP_INFO(node->get_logger(), "Starting Robot 4 Pick & Place with configurable object");
      RCLCPP_INFO(node->get_logger(), "Pick frame   : %s", pick_frame.c_str());
      RCLCPP_INFO(
        node->get_logger(), "Pick offset  : [%.3f, %.3f, %.3f]",
        pick_offset[0], pick_offset[1], pick_offset[2]);
      RCLCPP_INFO(node->get_logger(), "Place frame  : %s", place_frame.c_str());
      RCLCPP_INFO(
        node->get_logger(), "Place offset : [%.3f, %.3f, %.3f]",
        place_offset[0], place_offset[1], place_offset[2]);
      RCLCPP_INFO(node->get_logger(), "Object ID    : %s", object_id.c_str());
      RCLCPP_INFO(node->get_logger(), "Group        : %s", kGroup);
      RCLCPP_INFO(node->get_logger(), "Planning link: %s", kPlanningLink);
      RCLCPP_INFO(node->get_logger(), "Controller   : %s", kControllerAction);
      RCLCPP_INFO(node->get_logger(), "==============================================");

      rclcpp::sleep_for(2s);

      // --------------------------------------------------------
      // MoveIt group
      // --------------------------------------------------------

      moveit::planning_interface::MoveGroupInterface move_group(
        node,
        kGroup);

      move_group.setPoseReferenceFrame(kReferenceFrame);
      move_group.setEndEffectorLink(kPlanningLink);
      move_group.setPlanningTime(kPlanningTime);

      if (!move_group.getCurrentState(10.0)) {
        RCLCPP_ERROR(
          node->get_logger(),
          "Current Robot 4 state is unavailable.");
        return false;
      }

      // --------------------------------------------------------
      // Direct trajectory controller
      // --------------------------------------------------------

      auto fjt =
        rclcpp_action::create_client<sa3_motion::FJT>(
          node,
          kControllerAction);

      if (!fjt->wait_for_action_server(10s)) {
        RCLCPP_ERROR(
          node->get_logger(),
          "Controller action server '%s' is unavailable.",
          kControllerAction);
        return false;
      }

      // --------------------------------------------------------
      // Planning Scene
      //
      // The combined launch runs Robot 3 as the shared rebar-scene
      // initializer. It adds all four rebars from workcell.yaml.
      // Do NOT call addExtraRebars() here: that method rebuilds the
      // whole rebar set and could interfere with an object already
      // attached by the other robot.
      // --------------------------------------------------------

      PlanningSceneObjects scene_objects(node);
      moveit::planning_interface::PlanningSceneInterface scene_interface;

      RCLCPP_INFO(
        node->get_logger(),
        "Waiting for the shared Planning Scene to finish initializing '%s'...",
        object_id.c_str());

      // Robot 3 is responsible for loading the shared four-rebar set from
      // workcell.yaml. Do not treat one successful query as readiness: on a
      // relaunch, stale objects can briefly remain in MoveIt's scene while
      // Robot 3 replaces/reloads the set. Require the expected scene to be
      // present for several consecutive polls and verify the object using the
      // same helper that will be used for attachment/detachment.
      const std::vector<std::string> expected_rebar_ids = {
        "rebar_3c", "rebar_3d", "rebar_4c", "rebar_4d"};

      bool rebar_in_world = false;
      int stable_scene_samples = 0;
      constexpr int kRequiredStableSamples = 5;
      constexpr int kSceneWaitAttempts = 120;  // 30 seconds at 250 ms

      for (
        int attempt = 0;
        attempt < kSceneWaitAttempts && rclcpp::ok();
        ++attempt)
      {
        const auto world_objects = scene_interface.getObjects(expected_rebar_ids);
        const auto attached_objects =
          scene_interface.getAttachedObjects(expected_rebar_ids);

        const auto exists_in_world = [&](const std::string & id) {
            return world_objects.find(id) != world_objects.end();
          };
        const auto exists_attached = [&](const std::string & id) {
            return attached_objects.find(id) != attached_objects.end();
          };

        // 3C may already be attached by Robot 3 if that arm started first.
        // 3D, 4C and 4D should still be world objects at Robot 4 startup.
        const bool scene_has_expected_rebars =
          (exists_in_world("rebar_3c") || exists_attached("rebar_3c")) &&
          exists_in_world("rebar_3d") && !exists_attached("rebar_3d") &&
          exists_in_world("rebar_4c") && !exists_attached("rebar_4c") &&
          exists_in_world("rebar_4d") && !exists_attached("rebar_4d");

        if (
          scene_has_expected_rebars &&
          scene_objects.verifyObjectState(object_id, true, false))
        {
          ++stable_scene_samples;
          if (stable_scene_samples >= kRequiredStableSamples) {
            rebar_in_world = true;
            break;
          }
        } else {
          stable_scene_samples = 0;
        }

        std::this_thread::sleep_for(250ms);
      }

      if (!rebar_in_world) {
        RCLCPP_ERROR(
          node->get_logger(),
          "Shared rebar Planning Scene did not become stable within 30 seconds. "
          "Expected '%s' in WORLD state and all four rebar IDs present. "
          "Check Robot 3's addExtraRebars() log before starting motion.",
          object_id.c_str());
        return false;
      }

      RCLCPP_INFO(
        node->get_logger(),
        "Shared Planning Scene is stable; '%s' is ready for Robot 4 pickup.",
        object_id.c_str());

      // --------------------------------------------------------
      // Context
      // --------------------------------------------------------

      Ctx ctx;

      ctx.node = node;
      ctx.mg = &move_group;
      ctx.tf = &tf_buffer;
      ctx.fjt = fjt;
      ctx.scene = &scene_objects;
      ctx.link = kPlanningLink;
      ctx.object_id = object_id;
      ctx.pick_offset = pick_offset;
      ctx.place_offset = place_offset;

      ctx.cur.pick = pick_frame;
      ctx.cur.place = place_frame;

      // --------------------------------------------------------
      // Pick the object, carry it on the existing arc, and place it.
      // --------------------------------------------------------

      const bool result = pickPlace(
        ctx,
        {
          "r4_arc_start",
          "r4_arc_mid",
          "r4_arc_end"
        },
        8);

      if (!result && !ctx.failures.empty()) {
        const auto & failure = ctx.failures.back();

        RCLCPP_ERROR(
          node->get_logger(),
          "Last recorded failure: step '%s', planner '%s', stage '%s', code '%s'.",
          failure.info.step.c_str(),
          failure.planner.c_str(),
          failure.stage.c_str(),
          errStr(failure.code).c_str());
      }

      return result;
    }();
  }
  catch (const std::exception & error) {
    RCLCPP_FATAL(
      node->get_logger(),
      "Unhandled motion-program exception: %s",
      error.what());
  }
  catch (...) {
    RCLCPP_FATAL(
      node->get_logger(),
      "Unhandled non-standard motion-program exception.");
  }

  (void)tf_listener;

  executor.cancel();

  if (executor_thread.joinable()) {
    executor_thread.join();
  }

  rclcpp::shutdown();

  return ok ? 0 : 1;
}
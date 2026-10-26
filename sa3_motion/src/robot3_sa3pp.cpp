// Simple SA3 pick & place of ONE object together with the objects that sit on it
// (the "assembly"), e.g. rebar_1A carrying the web flanges placed on it.
//
// MoveIt cannot attach one collision object to another, so when the main object
// is attached to the gripper, every child object is attached to the SAME gripper
// link with its pose preserved. The whole assembly then follows the gripper.
//
// Sequence:
//   1. MoveJ  pick approach   (pick frame, y - 0.20)
//   2. Allow collision: object <-> gripper links, jig_clamp, sa3_main_jig;
//      once active children are identified, each child <-> both jigs
//   3. MoveL  pick frame
//   4. Attach object + all child objects to the gripper link
//   5. MoveL  pick approach   (retract)
//   6. MoveJ  place approach  (place frame, z + 0.20)
//   7. MoveL  place frame
//   8. Detach child objects and object into the world
//   9. MoveL  place retract   (place frame, x - 0.10)
//  10. Restore collision (only after the place retract is complete):
//      object <-> gripper links/jigs, each active child <-> gripper links/jigs,
//      and object <-> children
//  11. Go to home pose
//
// Parameters:
//   pick_frame, place_frame, object_id   main object (default rebar_1A)
//   children                             objects riding on it
//                                        (default rebar_3c, rebar_4c; children that
//                                         are not world objects are skipped)
//
// Planning is done by MoveIt, execution goes directly to the controller.
// No physical gripper actuation is implemented here.

#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sa3_motion/direct_execute.hpp"
#include "sa3_motion/planning_scene_objects.hpp"

using namespace std::chrono_literals;

namespace
{

// ---------------------------------------------------------------------------
// Settings (edit here)
// ---------------------------------------------------------------------------

constexpr char kGroup[] = "robot3_arm";
constexpr char kRobot1Group[] = "robot1_arm";
constexpr char kEndEffector[] = "robot3_end_effactor";
constexpr char kGripperLink2[] = "robot3_J6_link";   // extra gripper link
constexpr char kJigClamp[] = "jig_clamp";            // fixture the rebar sits in
constexpr char kMainJig[] = "sa3_main_jig";           // main jig that may contact the carried assembly
constexpr char kWorld[] = "world";
constexpr char kHomeState[] = "home";
constexpr char kRobot1SleepState[] = "robot1_sleep";
constexpr char kController[] = "/robot3_arm_controller/follow_joint_trajectory";
constexpr char kRobot1Controller[] = "/robot1_arm_controller/follow_joint_trajectory";
// Independent approach / retract offsets in WORLD frame: frame pose + offset.
// Pick approach: -0.10 m along X.
// Pick retract:  -0.20 m along Y.
// Place approach: +0.30 m along Z (unchanged).
// Place retract:  -0.10 m along X.
constexpr std::array<double, 3> kPickApproachOffset = {-0.10, 0.0, 0.0};
constexpr std::array<double, 3> kPickRetractOffset = {0.0, -0.20, 0.0};
constexpr std::array<double, 3> kPlaceApproachOffset = {0.0, 0.0, 0.30};
constexpr std::array<double, 3> kPlaceRetractOffset = {-0.05, 0.0, 0.0};

constexpr double kPlanningTime = 15.0;
constexpr int kMoveJPlanRetries = 3;  // Separate OMPL planning requests if one fails.
constexpr int kMoveLPlanRetries = 3;  // Separate Pilz LIN planning requests if one fails.
constexpr int kNamedPlanRetries = 3;  // Separate OMPL requests for named-state moves (robot1 sleep, home).
constexpr double kMoveJVel = 0.30;
constexpr double kMoveJAcc = 0.20;
constexpr double kMoveLVel = 0.20;
constexpr double kMoveLAcc = 0.10;
// Pilz LIN retries: each attempt uses the next (lower) velocity scale.
constexpr std::array<double, 5> kMoveLVelScales = {kMoveLVel, 0.08, 0.04, 0.02, 0.01};


// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

struct Ctx
{
  rclcpp::Node::SharedPtr node;
  moveit::planning_interface::MoveGroupInterface * mg = nullptr;
  tf2_ros::Buffer * tf = nullptr;
  rclcpp_action::Client<sa3_motion::FJT>::SharedPtr fjt;
  PlanningSceneObjects * scene = nullptr;

  std::string object_id;                      // main object
  std::vector<std::string> children;          // requested child objects
  std::vector<std::string> children_active;   // children actually riding on the object
};


geometry_msgs::msg::Pose withOffset(
  geometry_msgs::msg::Pose pose, const std::array<double, 3> & o)
{
  pose.position.x += o[0];
  pose.position.y += o[1];
  pose.position.z += o[2];
  return pose;
}


bool lookupPose(Ctx & ctx, const std::string & frame, geometry_msgs::msg::Pose & pose)
{
  try {
    const auto t = ctx.tf->lookupTransform(
      kWorld, frame, tf2::TimePointZero, tf2::durationFromSec(10.0));
    pose.position.x = t.transform.translation.x;
    pose.position.y = t.transform.translation.y;
    pose.position.z = t.transform.translation.z;
    pose.orientation = t.transform.rotation;
    return true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(
      ctx.node->get_logger(), "TF lookup failed for '%s': %s", frame.c_str(), e.what());
    return false;
  }
}


// Plan the current group target (retrying up to kNamedPlanRetries times),
// then execute the first successful plan on the controller.
bool planAndExecute(Ctx & ctx, const std::string & name)
{
  const auto logger = ctx.node->get_logger();

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool planned = false;
  int last_code = 0;

  for (int attempt = 1; attempt <= kNamedPlanRetries; ++attempt) {
    // Refresh the start state so each retry plans from where the robot actually is.
    ctx.mg->setStartStateToCurrentState();

    const auto result = ctx.mg->plan(plan);
    last_code = result.val;

    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      planned = true;
      RCLCPP_INFO(
        logger, "[%s] planning succeeded on attempt %d/%d",
        name.c_str(), attempt, kNamedPlanRetries);
      break;
    }

    RCLCPP_WARN(
      logger, "[%s] planning attempt %d/%d failed (code %d)",
      name.c_str(), attempt, kNamedPlanRetries, last_code);
  }

  ctx.mg->clearPoseTargets();

  if (!planned) {
    RCLCPP_ERROR(
      logger, "[%s] planning failed after %d attempts (last code %d)",
      name.c_str(), kNamedPlanRetries, last_code);
    return false;
  }

  int code = 0;
  std::string note;
  if (!sa3_motion::executeDirect(ctx.fjt, plan.trajectory_.joint_trajectory, code, note)) {
    RCLCPP_ERROR(
      logger, "[%s] execution failed (code %d): %s", name.c_str(), code, note.c_str());
    return false;
  }

  RCLCPP_INFO(logger, "[%s] done", name.c_str());
  return true;
}


// OMPL joint-space move to a pose.
bool movej(Ctx & ctx, const geometry_msgs::msg::Pose & target, const std::string & name)
{
  auto & mg = *ctx.mg;

  if (!mg.getCurrentState(10.0)) {
    RCLCPP_ERROR(ctx.node->get_logger(), "[%s] current state unavailable", name.c_str());
    return false;
  }

  mg.setStartStateToCurrentState();
  mg.setPlanningPipelineId("ompl");
  mg.setPlannerId("RRTConnectkConfigDefault");
  mg.setPlanningTime(kPlanningTime);
  mg.setNumPlanningAttempts(10);
  mg.setMaxVelocityScalingFactor(kMoveJVel);
  mg.setMaxAccelerationScalingFactor(kMoveJAcc);
  mg.setGoalPositionTolerance(0.001);
  mg.setGoalOrientationTolerance(0.01);
  mg.clearPoseTargets();

  if (!mg.setPoseTarget(target, kEndEffector)) {
    RCLCPP_ERROR(ctx.node->get_logger(), "[%s] pose target rejected", name.c_str());
    return false;
  }

  // Retry the OMPL planning request up to three times. Execute only the
  // first successful plan; do not retry controller execution here.
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool planned = false;
  int last_code = 0;

  for (int attempt = 1; attempt <= kMoveJPlanRetries; ++attempt) {
    const auto result = mg.plan(plan);
    last_code = result.val;

    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      planned = true;
      RCLCPP_INFO(
        ctx.node->get_logger(),
        "[%s] OMPL planning succeeded on attempt %d/%d",
        name.c_str(), attempt, kMoveJPlanRetries);
      break;
    }

    RCLCPP_WARN(
      ctx.node->get_logger(),
      "[%s] OMPL planning attempt %d/%d failed (code %d)",
      name.c_str(), attempt, kMoveJPlanRetries, last_code);
  }

  mg.clearPoseTargets();

  if (!planned) {
    RCLCPP_ERROR(
      ctx.node->get_logger(),
      "[%s] OMPL planning failed after %d attempts (last code %d)",
      name.c_str(), kMoveJPlanRetries, last_code);
    return false;
  }

  int code = 0;
  std::string note;
  if (!sa3_motion::executeDirect(
      ctx.fjt, plan.trajectory_.joint_trajectory, code, note))
  {
    RCLCPP_ERROR(
      ctx.node->get_logger(),
      "[%s] execution failed (code %d): %s",
      name.c_str(), code, note.c_str());
    return false;
  }

  RCLCPP_INFO(ctx.node->get_logger(), "[%s] done", name.c_str());
  return true;
}


// Pilz LIN straight-line move to a pose.
bool movel(Ctx & ctx, const geometry_msgs::msg::Pose & target, const std::string & name)
{
  auto & mg = *ctx.mg;

  if (!mg.getCurrentState(10.0)) {
    RCLCPP_ERROR(ctx.node->get_logger(), "[%s] current state unavailable", name.c_str());
    return false;
  }

  mg.setStartStateToCurrentState();
  mg.setPlanningPipelineId("pilz_industrial_motion_planner");
  mg.setPlannerId("LIN");
  mg.setPlanningTime(kPlanningTime);
  mg.setMaxVelocityScalingFactor(kMoveLVel);
  mg.setMaxAccelerationScalingFactor(kMoveLAcc);
  mg.clearPoseTargets();

  if (!mg.setPoseTarget(target, kEndEffector)) {
    RCLCPP_ERROR(ctx.node->get_logger(), "[%s] pose target rejected", name.c_str());
    return false;
  }

  // Retry Pilz LIN with progressively lower velocity/acceleration scaling.
  // Execute only the first successful plan.
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool planned = false;
  int last_code = 0;
  const int total = static_cast<int>(kMoveLVelScales.size());

  for (int attempt = 1; attempt <= total; ++attempt) {
    const double scale = kMoveLVelScales[attempt - 1];

    mg.setStartStateToCurrentState();
    mg.setMaxVelocityScalingFactor(scale);
    mg.setMaxAccelerationScalingFactor(std::min(kMoveLAcc, scale));

    const auto result = mg.plan(plan);
    last_code = result.val;

    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      planned = true;
      RCLCPP_INFO(
        ctx.node->get_logger(),
        "[%s] LIN planning succeeded on attempt %d/%d (velocity scale %.3f)",
        name.c_str(), attempt, total, scale);
      break;
    }

    RCLCPP_WARN(
      ctx.node->get_logger(),
      "[%s] LIN planning attempt %d/%d failed at velocity scale %.3f (code %d)",
      name.c_str(), attempt, total, scale, last_code);
  }

  mg.clearPoseTargets();

  if (!planned) {
    RCLCPP_ERROR(
      ctx.node->get_logger(),
      "[%s] LIN planning failed after %d attempts (last code %d)",
      name.c_str(), total, last_code);
    return false;
  }

  int code = 0;
  std::string note;
  if (!sa3_motion::executeDirect(
      ctx.fjt, plan.trajectory_.joint_trajectory, code, note))
  {
    RCLCPP_ERROR(
      ctx.node->get_logger(),
      "[%s] execution failed (code %d): %s",
      name.c_str(), code, note.c_str());
    return false;
  }

  RCLCPP_INFO(ctx.node->get_logger(), "[%s] done", name.c_str());
  return true;
}


// OMPL move to any named SRDF state. Used for Robot 1 sleep and Robot 3 home.
bool goToNamedState(Ctx & ctx, const std::string & state, const std::string & label)
{
  auto & mg = *ctx.mg;

  if (!mg.getCurrentState(10.0)) {
    RCLCPP_ERROR(
      ctx.node->get_logger(), "[%s] current state unavailable", label.c_str());
    return false;
  }

  mg.setStartStateToCurrentState();
  mg.setPlanningPipelineId("ompl");
  mg.setPlannerId("RRTConnectkConfigDefault");
  mg.setPlanningTime(kPlanningTime);
  mg.setNumPlanningAttempts(10);
  mg.setMaxVelocityScalingFactor(kMoveJVel);
  mg.setMaxAccelerationScalingFactor(kMoveJAcc);
  mg.clearPoseTargets();

  if (!mg.setNamedTarget(state)) {
    RCLCPP_ERROR(
      ctx.node->get_logger(), "[%s] named state '%s' not found for group",
      label.c_str(), state.c_str());
    return false;
  }

  return planAndExecute(ctx, label);
}

bool goHome(Ctx & ctx)
{
  return goToNamedState(ctx, kHomeState, "home");
}


// Allow / restore only the intended contact pairs for the carried assembly:
// main object <-> gripper links, jig_clamp, sa3_main_jig;
// each active child <-> gripper links, main object, jig_clamp, sa3_main_jig.
// Children that were not present as world collision objects are skipped.
// Allowed at pick, and only restored (allowed=false) after the place retract is done,
// so the gripper may touch the released rebars while it backs away.
bool setPickCollisions(Ctx & ctx, const std::vector<std::string> & touch_links, bool allowed)
{
  bool ok = true;
  ok &= ctx.scene->setObjectCollisionsAllowed(ctx.object_id, touch_links, allowed);
  ok &= ctx.scene->setObjectCollisionsAllowed(ctx.object_id, {kJigClamp}, allowed);
  ok &= ctx.scene->setObjectCollisionsAllowed(ctx.object_id, {kMainJig}, allowed);
  for (const auto & child : ctx.children_active) {
    ok &= ctx.scene->setObjectCollisionsAllowed(child, touch_links, allowed);
    ok &= ctx.scene->setCollisionPairAllowed(ctx.object_id, child, allowed);
    ok &= ctx.scene->setCollisionPairAllowed(child, kJigClamp, allowed);
    ok &= ctx.scene->setCollisionPairAllowed(child, kMainJig, allowed);
  }
  return ok;
}


// Attach the main object and every child that is currently a world object to the
// gripper link. Children keep their world pose (they stay where they sit on it).
bool attachAssembly(Ctx & ctx, const std::vector<std::string> & touch_links)
{
  const auto logger = ctx.node->get_logger();

  // Decide which children actually ride on the object.
  ctx.children_active.clear();
  for (const auto & child : ctx.children) {
    if (child == ctx.object_id) {
      continue;
    }
    if (!ctx.scene->verifyObjectState(child, true, false)) {
      RCLCPP_WARN(
        logger, "Child '%s' is not a world object; it will not follow '%s'.",
        child.c_str(), ctx.object_id.c_str());
      continue;
    }
    ctx.children_active.push_back(child);
  }

  // Allow the full carried assembly to overlap the jig clamp during lift/retract.
  // This includes object <-> child and child <-> jig_clamp pairs.
  if (!setPickCollisions(ctx, touch_links, true)) {
    RCLCPP_ERROR(
      logger,
      "Failed to allow collisions for '%s' and its active children against the jig clamp.",
      ctx.object_id.c_str());
    return false;
  }

  if (!ctx.scene->attachWorldObjectPreservingPose(
      *ctx.mg, ctx.object_id, kEndEffector, touch_links) ||
    !ctx.scene->verifyObjectState(ctx.object_id, false, true, kEndEffector))
  {
    RCLCPP_ERROR(logger, "Failed to attach '%s'.", ctx.object_id.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "ATTACHED: %s", ctx.object_id.c_str());

  for (const auto & child : ctx.children_active) {
    if (!ctx.scene->attachWorldObjectPreservingPose(
        *ctx.mg, child, kEndEffector, touch_links) ||
      !ctx.scene->verifyObjectState(child, false, true, kEndEffector))
    {
      RCLCPP_ERROR(
        logger, "Failed to attach child '%s'. Assembly left attached for recovery.",
        child.c_str());
      return false;
    }
    RCLCPP_INFO(logger, "ATTACHED (child): %s", child.c_str());
  }

  return true;
}


// Release the children first, then the main object, all back into the world.
bool detachAssembly(Ctx & ctx)
{
  const auto logger = ctx.node->get_logger();

  for (const auto & child : ctx.children_active) {
    if (!ctx.scene->detachObjectToWorldPreservingPose(*ctx.mg, child) ||
      !ctx.scene->verifyObjectState(child, true, false))
    {
      RCLCPP_ERROR(logger, "Failed to detach child '%s'.", child.c_str());
      return false;
    }
    RCLCPP_INFO(logger, "RELEASED (child): %s", child.c_str());
  }

  if (!ctx.scene->detachObjectToWorldPreservingPose(*ctx.mg, ctx.object_id) ||
    !ctx.scene->verifyObjectState(ctx.object_id, true, false))
  {
    RCLCPP_ERROR(logger, "Failed to detach '%s'.", ctx.object_id.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "RELEASED: %s", ctx.object_id.c_str());

  return true;
}


// ---------------------------------------------------------------------------
// Pick & place sequence
// ---------------------------------------------------------------------------

bool pickPlace(Ctx & ctx, const std::string & pick_frame, const std::string & place_frame)
{
  const auto logger = ctx.node->get_logger();
  const std::vector<std::string> touch_links = {kEndEffector, kGripperLink2,"robot3_sa3_grpr01"};

  geometry_msgs::msg::Pose pick;
  geometry_msgs::msg::Pose place;
  if (!lookupPose(ctx, pick_frame, pick) || !lookupPose(ctx, place_frame, place)) {
    return false;
  }

  const auto pick_approach = withOffset(pick, kPickApproachOffset);
  const auto pick_retract = withOffset(pick, kPickRetractOffset);
  const auto place_approach = withOffset(place, kPlaceApproachOffset);
  const auto place_retract = withOffset(place, kPlaceRetractOffset);

  // Object must start as a world collision object.
  if (!ctx.scene->verifyObjectState(ctx.object_id, true, false)) {
    RCLCPP_ERROR(logger, "'%s' is not a world object.", ctx.object_id.c_str());
    return false;
  }
  RCLCPP_INFO(
    logger,
    "Pick approach target: x=%.3f y=%.3f z=%.3f",
    pick_approach.position.x,
    pick_approach.position.y,
    pick_approach.position.z);

  // 1. MoveJ pick approach
  if (!movej(ctx, pick_approach, "pick approach")) {
    return false;
  }

  // 2. Allow intended contacts during pick: object <-> gripper, jig_clamp,
  // and sa3_main_jig. Active child pairs are enabled in attachAssembly().
  if (!ctx.scene->setObjectCollisionsAllowed("rebar_3c", touch_links, true) ||
    !ctx.scene->setObjectCollisionsAllowed(ctx.object_id, {kJigClamp}, true) ||
    !ctx.scene->setObjectCollisionsAllowed(ctx.object_id, {kMainJig}, true))
  {
    RCLCPP_ERROR(logger, "Failed to allow object collisions.");
    return false;
  }

  // 3. MoveL pick
  if (!movel(ctx, pick, "pick")) {
    setPickCollisions(ctx, touch_links, false);
    return false;
  }

  // 4. Attach object + children
  if (!attachAssembly(ctx, touch_links)) {
    if (ctx.scene->verifyObjectState(ctx.object_id, true, false)) {
      setPickCollisions(ctx, touch_links, false);
    }
    return false;
  }

  // 5. MoveL retract
  if (!movel(ctx, pick_retract, "pick retract")) {
    return false;
  }

  // 6. MoveL place approach
  if (!movel(ctx, place_approach, "place approach")) {
    return false;
  }

  // 7. MoveL place
  if (!movel(ctx, place, "place")) {
    return false;
  }

  // 8. Detach children and object
  if (!detachAssembly(ctx)) {
    return false;
  }

  // 9. MoveL retract (collision still allowed, gripper may touch object and children)
  if (!movel(ctx, place_retract, "place retract")) {
    return false;
  }

  // 10. Restore collision (only now, after the place retract has completed)
  if (!setPickCollisions(ctx, touch_links, false)) {
    RCLCPP_ERROR(logger, "Failed to restore object collisions.");
    return false;
  }

  // 11. Home
  if (!goHome(ctx)) {
    return false;
  }

  RCLCPP_INFO(logger, "Pick & place completed.");
  return true;
}

// Resume an already-attached assembly at the PLACE APPROACH step.
// No pick, no attachment, and no Robot 1 sleep motion is performed.
bool placeOnlyFromAttached(Ctx & ctx, const std::string & place_frame)
{
  const auto logger = ctx.node->get_logger();
  const std::vector<std::string> touch_links = {
    kEndEffector, kGripperLink2, "robot3_sa3_grpr01"};

  geometry_msgs::msg::Pose place;
  if (!lookupPose(ctx, place_frame, place)) {
    return false;
  }
  const auto place_approach = withOffset(place, kPlaceApproachOffset);
  const auto place_retract = withOffset(place, kPlaceRetractOffset);

  // Check the existing MoveIt planning scene, not just the current gripper pose.
  if (!ctx.scene->verifyObjectState(ctx.object_id, false, true, kEndEffector)) {
    RCLCPP_ERROR(logger,
      "Resume aborted: '%s' must already be attached to '%s' in MoveIt.",
      ctx.object_id.c_str(), kEndEffector);
    return false;
  }

  // Recover the list of attached children, since attachAssembly() is skipped.
  ctx.children_active.clear();
  for (const auto & child : ctx.children) {
    if (child == ctx.object_id) continue;
    if (ctx.scene->verifyObjectState(child, false, true, kEndEffector)) {
      ctx.children_active.push_back(child);
    } else {
      RCLCPP_INFO(logger,
        "Resume: '%s' is not attached to Robot 3; leaving it untouched.",
        child.c_str());
    }
  }

  RCLCPP_INFO(logger,
    "Resuming place-only with '%s' and %zu attached children. No pick or attach.",
    ctx.object_id.c_str(), ctx.children_active.size());

  // Maintain the same intended contact allowances as the original sequence.
  // Restore them only after placement and retraction succeed.
  if (!setPickCollisions(ctx, touch_links, true)) {
    RCLCPP_ERROR(logger, "Could not establish assembly contact allowances.");
    return false;
  }

  // Original steps 6-11, unchanged.
  if (!movej(ctx, place_approach, "place approach")) return false;
  if (!movel(ctx, place, "place")) return false;
  if (!detachAssembly(ctx)) return false;
  if (!movel(ctx, place_retract, "place retract")) return false;
  if (!setPickCollisions(ctx, touch_links, false)) {
    RCLCPP_ERROR(logger, "Failed to restore object collisions.");
    return false;
  }
  if (!goHome(ctx)) return false;

  RCLCPP_INFO(logger, "Place-only recovery completed.");
  return true;
}

}  // namespace


// ---------------------------------------------------------------------------
// MAIN
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "simple_pick_place_resume",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Parameters (override from launch / CLI).
  auto param = [&](const std::string & name, const std::string & def) {
      if (!node->has_parameter(name)) {
        node->declare_parameter<std::string>(name, def);
      }
      return node->get_parameter(name).as_string();
    };

  const std::string pick_frame = param("pick_frame", "inventory_3a_4a_frame");
  const std::string place_frame = param("place_frame", "place_frame_10");
  const std::string object_id = param("object_id", "rebar_1A");


  if (!node->has_parameter("children")) {
    node->declare_parameter<std::vector<std::string>>(
      "children",
      std::vector<std::string>{
        // From kJobs
        "rebar_6",
        "rebar_13_1_R",
        "rebar_13_2_R",
        "rebar_13_3_R",
        "rebar_14_4_R",
        "rebar_14_3_R",
        "rebar_14_2_R",
        "rebar_14_1_R",

        // From default_pair_specs
        "rebar_5a",
        "rebar_13_1",
        "rebar_13_2",
        "rebar_13_3",
        "rebar_13_4",
        "rebar_14_1",
        "rebar_14_2",
        "rebar_14_3",
        "rebar_14_4",
        "rebar_3c", "rebar_4c"
      });
  }

  const std::vector<std::string> children =
    node->get_parameter("children").as_string_array();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf_buffer.setUsingDedicatedThread(true);
  tf2_ros::TransformListener tf_listener(tf_buffer, node, false);

  bool ok = false;

  try {
    rclcpp::sleep_for(2s);

    // Resume Robot 3 only: Robot 1 and all pick/attach steps are skipped.
    moveit::planning_interface::MoveGroupInterface move_group(node, kGroup);
    move_group.setPoseReferenceFrame(kWorld);
    move_group.setEndEffectorLink(kEndEffector);

    auto fjt = rclcpp_action::create_client<sa3_motion::FJT>(node, kController);
    if (!fjt->wait_for_action_server(10s)) {
      RCLCPP_ERROR(node->get_logger(),
        "Robot 3 controller '%s' unavailable.", kController);
    } else {
      PlanningSceneObjects scene(node);
      Ctx ctx;
      ctx.node = node;
      ctx.mg = &move_group;
      ctx.tf = &tf_buffer;
      ctx.fjt = fjt;
      ctx.scene = &scene;
      ctx.object_id = object_id;
      ctx.children = children;

      RCLCPP_INFO(node->get_logger(),
        "PLACE-ONLY RESUME: place=%s, already-attached main object=%s",
        place_frame.c_str(), object_id.c_str());
      ok = placeOnlyFromAttached(ctx, place_frame);
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Unhandled exception: %s", e.what());
  }

  (void)tf_listener;
  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();

  return ok ? 0 : 1;
}
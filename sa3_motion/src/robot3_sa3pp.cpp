// Simple SA3 pick & place of ONE object together with the objects that sit on it
// (the "assembly"), e.g. rebar_1A carrying the web flanges placed on it.
//
// MoveIt cannot attach one collision object to another, so when the main object
// is attached to the gripper, every child object is attached to the SAME gripper
// link with its pose preserved. The whole assembly then follows the gripper.
//
// Sequence:
//   1. MoveJ  pick approach   (pick frame, y - 0.20)
//   2. Allow collision: object <-> gripper links, object <-> jig_clamp
//   3. MoveL  pick frame
//   4. Attach object + all child objects to the gripper link
//   5. MoveL  pick approach   (retract)
//   6. MoveJ  place approach  (place frame, y - 0.20)
//   7. MoveL  place frame
//   8. Detach child objects and object into the world
//   9. MoveL  place approach  (retract)
//  10. Restore collision: object <-> gripper links, object <-> jig_clamp,
//      object <-> children
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
constexpr char kEndEffector[] = "robot3_end_effactor";
constexpr char kGripperLink2[] = "robot3_J6_link";   // extra gripper link
constexpr char kJigClamp[] = "jig_clamp";            // fixture the rebar sits in
constexpr char kWorld[] = "world";
constexpr char kHomeState[] = "home";
constexpr char kController[] = "/robot3_arm_controller/follow_joint_trajectory";

// Approach / retract offset in WORLD frame: frame pose + this offset.
constexpr std::array<double, 3> kApproachOffset = {0.0, -0.20, 0.0};

constexpr double kPlanningTime = 15.0;
constexpr double kMoveJVel = 0.30;
constexpr double kMoveJAcc = 0.20;
constexpr double kMoveLVel = 0.20;
constexpr double kMoveLAcc = 0.10;


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


// Plan the current group target, then execute on the controller.
bool planAndExecute(Ctx & ctx, const std::string & name)
{
  const auto logger = ctx.node->get_logger();

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto result = ctx.mg->plan(plan);
  ctx.mg->clearPoseTargets();

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "[%s] planning failed (code %d)", name.c_str(), result.val);
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

  return planAndExecute(ctx, name);
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

  return planAndExecute(ctx, name);
}


// OMPL move to the named "home" state.
bool goHome(Ctx & ctx)
{
  auto & mg = *ctx.mg;

  if (!mg.getCurrentState(10.0)) {
    RCLCPP_ERROR(ctx.node->get_logger(), "[home] current state unavailable");
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

  if (!mg.setNamedTarget(kHomeState)) {
    RCLCPP_ERROR(
      ctx.node->get_logger(), "[home] named state '%s' not found in SRDF", kHomeState);
    return false;
  }

  return planAndExecute(ctx, "home");
}


// Allow / restore collision for object <-> gripper links, object <-> jig_clamp and
// object <-> each active child.
bool setPickCollisions(Ctx & ctx, const std::vector<std::string> & touch_links, bool allowed)
{
  bool ok = true;
  ok &= ctx.scene->setObjectCollisionsAllowed(ctx.object_id, touch_links, allowed);
  ok &= ctx.scene->setObjectCollisionsAllowed(ctx.object_id, {kJigClamp}, allowed);
  for (const auto & child : ctx.children_active) {
    ok &= ctx.scene->setCollisionPairAllowed(ctx.object_id, child, allowed);
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

  // Object <-> children contact allowed (they touch while carried).
  for (const auto & child : ctx.children_active) {
    if (!ctx.scene->setCollisionPairAllowed(ctx.object_id, child, true)) {
      RCLCPP_ERROR(
        logger, "Failed to allow collision '%s' <-> '%s'.",
        ctx.object_id.c_str(), child.c_str());
      return false;
    }
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
  const std::vector<std::string> touch_links = {kEndEffector, kGripperLink2};

  geometry_msgs::msg::Pose pick;
  geometry_msgs::msg::Pose place;
  if (!lookupPose(ctx, pick_frame, pick) || !lookupPose(ctx, place_frame, place)) {
    return false;
  }

  const auto pick_approach = withOffset(pick, kApproachOffset);
  const auto place_approach = withOffset(place, kApproachOffset);

  // Object must start as a world collision object.
  if (!ctx.scene->verifyObjectState(ctx.object_id, true, false)) {
    RCLCPP_ERROR(logger, "'%s' is not a world object.", ctx.object_id.c_str());
    return false;
  }

  // 1. MoveJ pick approach
  if (!movej(ctx, pick_approach, "pick approach")) {
    return false;
  }

  // 2. Allow collision: object <-> gripper, object <-> jig_clamp
  if (!ctx.scene->setObjectCollisionsAllowed(ctx.object_id, touch_links, true) ||
    !ctx.scene->setObjectCollisionsAllowed(ctx.object_id, {kJigClamp}, true))
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
  if (!movel(ctx, pick_approach, "pick retract")) {
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

  // 9. MoveL retract (collision still allowed, gripper may touch object)
  if (!movel(ctx, place_approach, "place retract")) {
    return false;
  }

  // 10. Restore collision
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

}  // namespace


// ---------------------------------------------------------------------------
// MAIN
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "simple_pick_place",
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
        "rebar_14_4","rebar_3c", "rebar_4c"
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

    moveit::planning_interface::MoveGroupInterface move_group(node, kGroup);
    move_group.setPoseReferenceFrame(kWorld);
    move_group.setEndEffectorLink(kEndEffector);

    auto fjt = rclcpp_action::create_client<sa3_motion::FJT>(node, kController);
    if (!fjt->wait_for_action_server(10s)) {
      RCLCPP_ERROR(node->get_logger(), "Controller '%s' unavailable.", kController);
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

        RCLCPP_INFO(
        node->get_logger(), "Pick: %s | Place: %s | Object: %s | Children: %zu",
        pick_frame.c_str(), place_frame.c_str(), object_id.c_str(), children.size());

        ok = pickPlace(ctx, pick_frame, place_frame);

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
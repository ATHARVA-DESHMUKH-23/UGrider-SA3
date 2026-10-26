// Combined Robot 3 + Robot 4 pick/place node.
// Plans both arms together through the SRDF group "robot3_robot4_arms".
// All motion stages use OMPL MoveJ-style joint-space planning.
// Verify all frames, object IDs, touch links, and controller execution in RViz/mock first.

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sa3_motion/planning_scene_objects.hpp"

using namespace std::chrono_literals;
using MoveItErr = moveit_msgs::msg::MoveItErrorCodes;

namespace
{
constexpr char kNodeName[] = "robot3_robot4_combined_pick_place";
constexpr char kCombinedGroup[] = "robot3_robot4_arms";
constexpr char kRobot3Group[] = "robot3_arm";
constexpr char kRobot4Group[] = "robot4_arm";
constexpr char kRobot3Link[] = "robot3_end_effactor_ff";
constexpr char kRobot4Link[] = "robot4_end_effactor_ff";
constexpr char kWorld[] = "world";

constexpr double kPlanningTime = 15.0;
constexpr int kPlanningAttempts = 10;
constexpr int kRetries = 3;
constexpr double kVelocity = 0.30;
constexpr double kAcceleration = 0.20;
constexpr double kIkTimeout = 1.0;

// Retain the offsets from the supplied single-arm nodes.
constexpr std::array<double, 3> kRobot3PickApproachOffset{-0.6, 0.0, 0.6};
constexpr std::array<double, 3> kRobot4PickApproachOffset{ 0.6, 0.0, 0.6};
constexpr std::array<double, 3> kPlaceApproachOffset{0.0, -0.25, 0.0};

struct ArmConfig
{
  std::string name;
  std::string group;
  std::string link;
  std::string pick_frame;
  std::string place_frame;
  std::array<std::string, 3> arc_frames;
  std::string object_id;
  std::string inventory_id;
  std::vector<std::string> touch_links;
  std::array<double, 3> pick_offset{0.0, 0.0, 0.0};
  std::array<double, 3> place_offset{0.0, 0.0, 0.0};
};

geometry_msgs::msg::Pose offsetPose(
  const geometry_msgs::msg::Pose & p, const std::array<double, 3> & d)
{
  auto out = p;
  out.position.x += d[0];
  out.position.y += d[1];
  out.position.z += d[2];
  return out;
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
    RCLCPP_ERROR(logger, "TF lookup '%s' -> '%s' failed: %s",
      kWorld, frame.c_str(), e.what());
    return false;
  }
}

// Returns a circular arc through A, B, C. Orientation is interpolated A->B->C.
std::vector<geometry_msgs::msg::Pose> arcThrough3(
  const geometry_msgs::msg::Pose & a,
  const geometry_msgs::msg::Pose & b,
  const geometry_msgs::msg::Pose & c, int n)
{
  using V = tf2::Vector3;
  auto pos = [](const geometry_msgs::msg::Pose & p) {
    return V(p.position.x, p.position.y, p.position.z);
  };
  auto quat = [](const geometry_msgs::msg::Pose & p) {
    tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    q.normalize();
    return q;
  };
  n = std::max(n, 2);
  const V A = pos(a), B = pos(b), C = pos(c);
  const V u = B - A, v = C - A, w = u.cross(v);
  const double w2 = w.length2();
  if (w2 < 1e-12) return {};
  const V center = A + ((v * u.length2() - u * v.length2()).cross(w)) / (2.0 * w2);
  const double radius = (A - center).length();
  if (radius < 1e-9) return {};
  const V e1 = (A - center) / radius;
  const V normal = w.normalized();
  const V e2 = normal.cross(e1);
  auto angleOf = [&](const V & p) {
    const V d = p - center;
    return std::atan2(d.dot(e2), d.dot(e1));
  };
  const double angleB = angleOf(B);
  double angleC = angleOf(C);
  constexpr double pi = 3.14159265358979323846;
  if (angleB > 0.0) { if (angleC < angleB) angleC += 2.0 * pi; }
  else { if (angleC > angleB) angleC -= 2.0 * pi; }
  if (std::abs(angleC) < 1e-9) return {};
  const double fracB = std::clamp(angleB / angleC, 0.0, 1.0);
  const auto qa = quat(a), qb = quat(b), qc = quat(c);
  std::vector<geometry_msgs::msg::Pose> out;
  out.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    const double f = static_cast<double>(i) / static_cast<double>(n - 1);
    const double theta = f * angleC;
    const V p = center + radius * (std::cos(theta) * e1 + std::sin(theta) * e2);
    tf2::Quaternion q;
    if (f <= fracB && fracB > 1e-9) q = qa.slerp(qb, f / fracB);
    else if (fracB < 1.0 - 1e-9) q = qb.slerp(qc, (f - fracB) / (1.0 - fracB));
    else q = qc;
    q.normalize();
    geometry_msgs::msg::Pose pose;
    pose.position.x = p.x(); pose.position.y = p.y(); pose.position.z = p.z();
    pose.orientation.x = q.x(); pose.orientation.y = q.y();
    pose.orientation.z = q.z(); pose.orientation.w = q.w();
    out.push_back(pose);
  }
  return out;
}

// Solve each end-effector IK against the same RobotState, then plan one combined
// 12-joint goal. This avoids overwriting one arm's pose target with the other.
bool planAndExecuteBoth(
  const rclcpp::Logger & logger,
  moveit::planning_interface::MoveGroupInterface & combined,
  const geometry_msgs::msg::Pose & target3,
  const geometry_msgs::msg::Pose & target4,
  const std::string & stage)
{
  for (int attempt = 1; attempt <= kRetries; ++attempt) {
    auto state = combined.getCurrentState(10.0);
    if (!state) {
      RCLCPP_ERROR(logger, "[%s] Current combined robot state unavailable.", stage.c_str());
      return false;
    }
    const auto * group3 = state->getJointModelGroup(kRobot3Group);
    const auto * group4 = state->getJointModelGroup(kRobot4Group);
    if (!group3 || !group4) {
      RCLCPP_ERROR(logger, "SRDF joint model groups '%s'/'%s' unavailable.",
        kRobot3Group, kRobot4Group);
      return false;
    }
    const bool ik3 = state->setFromIK(group3, target3, kRobot3Link, kIkTimeout);
    const bool ik4 = ik3 && state->setFromIK(group4, target4, kRobot4Link, kIkTimeout);
    if (!ik3 || !ik4) {
      RCLCPP_WARN(logger, "[%s] IK attempt %d/%d failed (Robot3=%s, Robot4=%s).",
        stage.c_str(), attempt, kRetries, ik3 ? "OK" : "FAIL", ik4 ? "OK" : "FAIL");
      continue;
    }

    combined.setStartStateToCurrentState();
    combined.setPlanningPipelineId("ompl");
    combined.setPlannerId("RRTConnectkConfigDefault");
    combined.setPlanningTime(kPlanningTime);
    combined.setNumPlanningAttempts(kPlanningAttempts);
    combined.setMaxVelocityScalingFactor(kVelocity);
    combined.setMaxAccelerationScalingFactor(kAcceleration);
    combined.clearPoseTargets();
    combined.clearPathConstraints();

    if (!combined.setJointValueTarget(*state)) {
      RCLCPP_WARN(logger, "[%s] Could not set combined 12-joint target.", stage.c_str());
      continue;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planned = combined.plan(plan);
    if (planned != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(logger, "[%s] combined OMPL plan attempt %d/%d failed (code %d).",
        stage.c_str(), attempt, kRetries, planned.val);
      continue;
    }

    const auto executed = combined.execute(plan);
    if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "[%s] combined trajectory execution failed (code %d).",
        stage.c_str(), executed.val);
      return false;
    }
    RCLCPP_INFO(logger, "[%s] both arms completed combined MoveJ.", stage.c_str());
    return true;
  }
  RCLCPP_ERROR(logger, "[%s] all combined planning attempts failed.", stage.c_str());
  return false;
}

bool moveBoth(
  const rclcpp::Logger & logger,
  moveit::planning_interface::MoveGroupInterface & group,
  const geometry_msgs::msg::Pose & p3, const geometry_msgs::msg::Pose & p4,
  const std::string & stage)
{
  return planAndExecuteBoth(logger, group, p3, p4, stage);
}

bool runSequence(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & combined,
  tf2_ros::Buffer & tf,
  PlanningSceneObjects & scene,
  ArmConfig & a3, ArmConfig & a4,
  int arc_points)
{
  const auto logger = node->get_logger();
  geometry_msgs::msg::Pose pickFrame3, placeFrame3, pickFrame4, placeFrame4;
  geometry_msgs::msg::Pose a3p0, a3p1, a3p2, a4p0, a4p1, a4p2;
  if (!lookupPose(logger, tf, a3.pick_frame, pickFrame3) ||
      !lookupPose(logger, tf, a3.place_frame, placeFrame3) ||
      !lookupPose(logger, tf, a4.pick_frame, pickFrame4) ||
      !lookupPose(logger, tf, a4.place_frame, placeFrame4) ||
      !lookupPose(logger, tf, a3.arc_frames[0], a3p0) ||
      !lookupPose(logger, tf, a3.arc_frames[1], a3p1) ||
      !lookupPose(logger, tf, a3.arc_frames[2], a3p2) ||
      !lookupPose(logger, tf, a4.arc_frames[0], a4p0) ||
      !lookupPose(logger, tf, a4.arc_frames[1], a4p1) ||
      !lookupPose(logger, tf, a4.arc_frames[2], a4p2)) return false;

  const auto pick3 = offsetPose(pickFrame3, a3.pick_offset);
  const auto place3 = offsetPose(placeFrame3, a3.place_offset);
  const auto pick4 = offsetPose(pickFrame4, a4.pick_offset);
  const auto place4 = offsetPose(placeFrame4, a4.place_offset);
  const auto pickApproach3 = offsetPose(pick3, kRobot3PickApproachOffset);
  const auto pickApproach4 = offsetPose(pick4, kRobot4PickApproachOffset);
  const auto placeApproach3 = offsetPose(place3, kPlaceApproachOffset);
  const auto placeApproach4 = offsetPose(place4, kPlaceApproachOffset);
  const auto arc3 = arcThrough3(a3p0, a3p1, a3p2, arc_points);
  const auto arc4 = arcThrough3(a4p0, a4p1, a4p2, arc_points);
  if (arc3.empty() || arc4.empty() || arc3.size() != arc4.size()) {
    RCLCPP_ERROR(logger, "Could not build both arcs, or arc point counts differ.");
    return false;
  }

  if (!scene.verifyObjectState(a3.object_id, true, false) ||
      !scene.verifyObjectState(a4.object_id, true, false)) {
    RCLCPP_ERROR(logger, "Both rebars must be world collision objects before starting.");
    return false;
  }

  // 1) Move both arms to pick approach in one combined plan.
  if (!moveBoth(logger, combined, pickApproach3, pickApproach4, "pick approach")) return false;

  // 2) Allow only each rebar's gripper-touch links.
  if (!scene.setObjectCollisionsAllowed(a3.object_id, a3.touch_links, true) ||
      !scene.setObjectCollisionsAllowed(a4.object_id, a4.touch_links, true)) {
    RCLCPP_ERROR(logger, "Failed to configure rebar/gripper contact allowances.");
    return false;
  }

  // 3) Move both arms to their pick poses together.
  if (!moveBoth(logger, combined, pick3, pick4, "pick")) return false;

  // 4) Attach both objects, preserving their world poses.
  if (!scene.attachWorldObjectPreservingPose(combined, a3.object_id, a3.link, a3.touch_links)) {
    RCLCPP_ERROR(logger, "Failed to attach %s to %s.", a3.object_id.c_str(), a3.link.c_str());
    return false;
  }
  if (!scene.attachWorldObjectPreservingPose(combined, a4.object_id, a4.link, a4.touch_links)) {
    RCLCPP_ERROR(logger, "Failed to attach %s to %s. Robot 3 object remains attached for recovery.",
      a4.object_id.c_str(), a4.link.c_str());
    return false;
  }

  // 5) Allow carried rebars to contact their respective source inventory during retract.
  if (!scene.setObjectCollisionsAllowed(a3.object_id, {a3.inventory_id}, true) ||
      !scene.setObjectCollisionsAllowed(a4.object_id, {a4.inventory_id}, true)) {
    RCLCPP_ERROR(logger, "Failed to enable rebar/inventory contact during retract.");
    return false;
  }
  if (!moveBoth(logger, combined, pickApproach3, pickApproach4, "pick retract")) return false;
  if (!scene.setObjectCollisionsAllowed(a3.object_id, {a3.inventory_id}, false) ||
      !scene.setObjectCollisionsAllowed(a4.object_id, {a4.inventory_id}, false)) {
    RCLCPP_ERROR(logger, "Failed to restore inventory collision checking.");
    return false;
  }

  // 6) Carry both rebars along their arcs. Each waypoint is a single combined plan.
  for (std::size_t i = 0; i < arc3.size(); ++i) {
    if (!moveBoth(logger, combined, arc3[i], arc4[i],
        "arc waypoint " + std::to_string(i + 1) + "/" + std::to_string(arc3.size()))) {
      RCLCPP_ERROR(logger, "Arc failed; attached rebars remain attached for recovery.");
      return false;
    }
  }

  // 7) Permit rebar-to-jig contact for placement (robot-to-jig checks remain enabled).
  if (!scene.setObjectCollisionsAllowed(a3.object_id, {"jig_clamp"}, true) ||
      !scene.setObjectCollisionsAllowed(a4.object_id, {"jig_clamp"}, true)) {
    RCLCPP_ERROR(logger, "Failed to allow rebar/jig contact.");
    return false;
  }

  // 8) Move to place approach and then place, with both arms planned together.
  if (!moveBoth(logger, combined, placeApproach3, placeApproach4, "place approach") ||
      !moveBoth(logger, combined, place3, place4, "place")) return false;

  // 9) Detach each rebar into the world at its actual release pose.
  if (!scene.detachObjectToWorldPreservingPose(combined, a3.object_id)) {
    RCLCPP_ERROR(logger, "Failed to detach %s.", a3.object_id.c_str());
    return false;
  }
  if (!scene.detachObjectToWorldPreservingPose(combined, a4.object_id)) {
    RCLCPP_ERROR(logger, "Failed to detach %s.", a4.object_id.c_str());
    return false;
  }

  // 10) Retract both arms, then restore gripper collision checks.
  if (!moveBoth(logger, combined, placeApproach3, placeApproach4, "place retract")) return false;
  if (!scene.setObjectCollisionsAllowed(a3.object_id, a3.touch_links, false) ||
      !scene.setObjectCollisionsAllowed(a4.object_id, a4.touch_links, false)) {
    RCLCPP_ERROR(logger, "Objects placed, but failed to restore gripper collision checking.");
    return false;
  }

  RCLCPP_INFO(logger, "Combined Robot 3 + Robot 4 pick/place completed.");
  return true;
}

std::array<double, 3> readOffset(
  const rclcpp::Node::SharedPtr & node, const std::string & name)
{
  // The offset parameters are declared in main(). Only declare here if this
  // helper is reused with a parameter that has not already been declared.
  if (!node->has_parameter(name)) {
    node->declare_parameter<std::vector<double>>(
      name, std::vector<double>{0.0, 0.0, 0.0});
  }

  const auto v = node->get_parameter(name).as_double_array();
  if (v.size() != 3) {
    throw std::runtime_error(name + " must contain [x,y,z]");
  }
  return {v[0], v[1], v[2]};
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Declare parameters explicitly below; ROS parameter overrides are applied
  // when each parameter is declared.
  auto node = rclcpp::Node::make_shared(kNodeName);

  // Frame and object parameters can be overridden from the launch file.
  node->declare_parameter<std::string>("robot3_pick_frame", "inventory_3a_4a_frame");
  node->declare_parameter<std::string>("robot3_place_frame", "place_frame_10_l");
  node->declare_parameter<std::string>("robot3_object_id", "rebar_3c");
  node->declare_parameter<std::string>("robot3_inventory_id", "inventory_3a_4a");
  node->declare_parameter<std::string>("robot4_pick_frame", "inventory_3a_4a_frame_2");
  node->declare_parameter<std::string>("robot4_place_frame", "place_frame_10");
  node->declare_parameter<std::string>("robot4_object_id", "rebar_4c");
  node->declare_parameter<std::string>("robot4_inventory_id", "inventory_3a_4a_2");
  node->declare_parameter<std::vector<std::string>>(
    "robot3_arc_frames", {"r3_arc_start", "r3_arc_mid", "r3_arc_end"});
  node->declare_parameter<std::vector<std::string>>(
    "robot4_arc_frames", {"r4_arc_start", "r4_arc_mid", "r4_arc_end"});
  node->declare_parameter<int>("arc_points", 8);
  node->declare_parameter<std::vector<double>>("robot3_pick_offset", {0.0, 0.0, 0.0});
  node->declare_parameter<std::vector<double>>("robot3_place_offset", {0.0, 0.0, 0.0});
  node->declare_parameter<std::vector<double>>("robot4_pick_offset", {0.0, 0.0, 0.0});
  node->declare_parameter<std::vector<double>>("robot4_place_offset", {0.0, 0.0, 0.0});

  bool success = false;
  try {
    ArmConfig a3, a4;
    a3.name = "robot3"; a3.group = kRobot3Group; a3.link = kRobot3Link;
    a3.pick_frame = node->get_parameter("robot3_pick_frame").as_string();
    a3.place_frame = node->get_parameter("robot3_place_frame").as_string();
    a3.object_id = node->get_parameter("robot3_object_id").as_string();
    a3.inventory_id = node->get_parameter("robot3_inventory_id").as_string();
    a3.touch_links = {kRobot3Link, "robot3_J6_link", "robot3_sa3_grpr01_r"};
    a3.pick_offset = readOffset(node, "robot3_pick_offset");
    a3.place_offset = readOffset(node, "robot3_place_offset");
    auto f3 = node->get_parameter("robot3_arc_frames").as_string_array();
    if (f3.size() != 3) throw std::runtime_error("robot3_arc_frames must have 3 frame names");
    std::copy(f3.begin(), f3.end(), a3.arc_frames.begin());

    a4.name = "robot4"; a4.group = kRobot4Group; a4.link = kRobot4Link;
    a4.pick_frame = node->get_parameter("robot4_pick_frame").as_string();
    a4.place_frame = node->get_parameter("robot4_place_frame").as_string();
    a4.object_id = node->get_parameter("robot4_object_id").as_string();
    a4.inventory_id = node->get_parameter("robot4_inventory_id").as_string();
    a4.touch_links = {kRobot4Link, "robot4_J6_link", "robot4_sa3_grpr01_r"};
    a4.pick_offset = readOffset(node, "robot4_pick_offset");
    a4.place_offset = readOffset(node, "robot4_place_offset");
    auto f4 = node->get_parameter("robot4_arc_frames").as_string_array();
    if (f4.size() != 3) throw std::runtime_error("robot4_arc_frames must have 3 frame names");
    std::copy(f4.begin(), f4.end(), a4.arc_frames.begin());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf_buffer.setUsingDedicatedThread(true);
    tf2_ros::TransformListener tf_listener(tf_buffer, node, false);

    RCLCPP_INFO(node->get_logger(), "Initializing combined group '%s'.", kCombinedGroup);
    moveit::planning_interface::MoveGroupInterface combined(node, kCombinedGroup);
    combined.setPoseReferenceFrame(kWorld);
    combined.setPlanningTime(kPlanningTime);

    if (!combined.getCurrentState(10.0)) {
      RCLCPP_ERROR(node->get_logger(), "Combined group's current state unavailable.");
    } else {
      PlanningSceneObjects scene(node);
      success = runSequence(
        node, combined, tf_buffer, scene, a3, a4,
        std::max<int64_t>(2, node->get_parameter("arc_points").as_int()));
    }

    (void)tf_listener;
    executor.cancel();
    if (spin_thread.joinable()) spin_thread.join();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Combined motion node failed: %s", e.what());
  }
  rclcpp::shutdown();
  return success ? 0 : 1;
}
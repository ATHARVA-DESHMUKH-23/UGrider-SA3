// Combined Robot 3 + Robot 4 pick/place node.
// Combined transitions use OMPL; dual-arm LIN moves are planned per arm,
// merged into one 12-joint trajectory, and executed through the combined group.
// Verify all frames, object IDs, touch links, and controller execution in RViz/mock first.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <utility>
#include <stdexcept>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
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
constexpr double kVelocity = 0.10;
constexpr double kAcceleration = 0.10;
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

// Sample one planned trajectory at a requested time after time-scaling it.
// factor > 1 stretches the trajectory to the shared duration.
trajectory_msgs::msg::JointTrajectoryPoint sampleTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  double scaled_time, double factor)
{
  const auto & points = trajectory.points;
  trajectory_msgs::msg::JointTrajectoryPoint out;
  if (points.empty()) return out;

  const double source_time = scaled_time / factor;
  auto pointTime = [](const trajectory_msgs::msg::JointTrajectoryPoint & p) {
    return rclcpp::Duration(p.time_from_start).seconds();
  };

  std::size_t left = 0;
  std::size_t right = 0;
  double alpha = 0.0;

  if (source_time >= pointTime(points.back())) {
    left = right = points.size() - 1;
  } else {
    for (std::size_t i = 1; i < points.size(); ++i) {
      if (pointTime(points[i]) >= source_time) {
        left = i - 1;
        right = i;
        const double t0 = pointTime(points[left]);
        const double t1 = pointTime(points[right]);
        alpha = (t1 > t0) ? std::clamp((source_time - t0) / (t1 - t0), 0.0, 1.0) : 0.0;
        break;
      }
    }
  }

  const auto & a = points[left];
  const auto & b = points[right];
  auto interpolate = [alpha](const std::vector<double> & va,
                              const std::vector<double> & vb,
                              std::vector<double> & out_values,
                              double scale) {
    if (va.empty() || va.size() != vb.size()) return false;
    out_values.resize(va.size());
    for (std::size_t i = 0; i < va.size(); ++i)
      out_values[i] = (va[i] + alpha * (vb[i] - va[i])) * scale;
    return true;
  };

  if (!interpolate(a.positions, b.positions, out.positions, 1.0)) return {};
  // If either plan omits derivatives, leave that derivative array empty.
  interpolate(a.velocities, b.velocities, out.velocities, 1.0 / factor);
  interpolate(a.accelerations, b.accelerations, out.accelerations, 1.0 / (factor * factor));
  return out;
}

// Plan both arms with Pilz LIN from the same captured start state, merge their
// joint trajectories on a shared time axis, then execute only through the
// combined group. No per-point trajectory debug logging is emitted.
bool movel_combined(
  const rclcpp::Logger & logger,
  moveit::planning_interface::MoveGroupInterface & robot1,
  moveit::planning_interface::MoveGroupInterface & robot2,
  moveit::planning_interface::MoveGroupInterface & combinedgroup,
  const geometry_msgs::msg::Pose & goal1,
  const geometry_msgs::msg::Pose & goal2,
  const std::string & stage)
{
  auto start_state = combinedgroup.getCurrentState(10.0);
  if (!start_state) {
    RCLCPP_ERROR(logger, "[%s] Current combined robot state unavailable.", stage.c_str());
    return false;
  }

  moveit_msgs::msg::RobotState start_state_msg;
  moveit::core::robotStateToRobotStateMsg(*start_state, start_state_msg);

  auto configureLin = [&](moveit::planning_interface::MoveGroupInterface & group) {
    group.setStartState(start_state_msg);
    group.setPlanningPipelineId("pilz_industrial_motion_planner");
    group.setPlannerId("LIN");
    group.setPlanningTime(kPlanningTime);
    group.setNumPlanningAttempts(1);
    group.setMaxVelocityScalingFactor(0.1);
    group.setMaxAccelerationScalingFactor(0.1);
    group.clearPoseTargets();
    group.clearPathConstraints();
  };

  configureLin(robot1);
  configureLin(robot2);

  if (!robot1.setPoseTarget(goal1, kRobot3Link) ||
      !robot2.setPoseTarget(goal2, kRobot4Link)) {
    RCLCPP_ERROR(logger, "[%s] One of the individual LIN pose targets was rejected.", stage.c_str());
    robot1.clearPoseTargets();
    robot2.clearPoseTargets();
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan1, plan2;
  const auto result1 = robot1.plan(plan1);
  robot1.clearPoseTargets();
  if (result1 != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "[%s] Robot1 LIN planning failed (code %d).", stage.c_str(), result1.val);
    robot2.clearPoseTargets();
    return false;
  }

  const auto result2 = robot2.plan(plan2);
  robot2.clearPoseTargets();
  if (result2 != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "[%s] Robot2 LIN planning failed (code %d).", stage.c_str(), result2.val);
    return false;
  }

  const auto & jt1 = plan1.trajectory_.joint_trajectory;
  const auto & jt2 = plan2.trajectory_.joint_trajectory;
  if (jt1.joint_names.empty() || jt2.joint_names.empty() ||
      jt1.points.empty() || jt2.points.empty() ||
      !plan1.trajectory_.multi_dof_joint_trajectory.joint_names.empty() ||
      !plan2.trajectory_.multi_dof_joint_trajectory.joint_names.empty()) {
    RCLCPP_ERROR(logger, "[%s] Cannot merge empty or multi-DOF LIN trajectories.", stage.c_str());
    return false;
  }

  auto durationOf = [](const trajectory_msgs::msg::JointTrajectory & jt) {
    return rclcpp::Duration(jt.points.back().time_from_start).seconds();
  };
  const double duration1 = durationOf(jt1);
  const double duration2 = durationOf(jt2);
  if (duration1 <= 0.0 || duration2 <= 0.0) {
    RCLCPP_ERROR(logger, "[%s] One of the LIN trajectories has invalid duration.", stage.c_str());
    return false;
  }

  // Stretch the shorter plan to the longer plan's duration, then sample both
  // trajectories at the union of their timestamps.
  const double shared_duration = std::max(duration1, duration2);
  const double factor1 = shared_duration / duration1;
  const double factor2 = shared_duration / duration2;
  std::vector<double> times{0.0, shared_duration};
  for (const auto & point : jt1.points)
    times.push_back(rclcpp::Duration(point.time_from_start).seconds() * factor1);
  for (const auto & point : jt2.points)
    times.push_back(rclcpp::Duration(point.time_from_start).seconds() * factor2);
  std::sort(times.begin(), times.end());
  times.erase(std::unique(times.begin(), times.end(),
      [](double a, double b) { return std::abs(a - b) < 1e-9; }), times.end());

  moveit::planning_interface::MoveGroupInterface::Plan combined_plan;
  combined_plan.start_state_ = start_state_msg;
  combined_plan.planning_time_ = plan1.planning_time_ + plan2.planning_time_;
  auto & merged = combined_plan.trajectory_.joint_trajectory;
  merged.header = jt1.header;
  merged.joint_names = jt1.joint_names;
  merged.joint_names.insert(merged.joint_names.end(), jt2.joint_names.begin(), jt2.joint_names.end());

  for (double t : times) {
    auto p1 = sampleTrajectory(jt1, t, factor1);
    auto p2 = sampleTrajectory(jt2, t, factor2);
    if (p1.positions.size() != jt1.joint_names.size() ||
        p2.positions.size() != jt2.joint_names.size()) {
      RCLCPP_ERROR(logger, "[%s] Failed to sample one of the LIN trajectories.", stage.c_str());
      return false;
    }

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = p1.positions;
    point.positions.insert(point.positions.end(), p2.positions.begin(), p2.positions.end());

    if (p1.velocities.size() == jt1.joint_names.size() &&
        p2.velocities.size() == jt2.joint_names.size()) {
      point.velocities = p1.velocities;
      point.velocities.insert(point.velocities.end(), p2.velocities.begin(), p2.velocities.end());
    }
    if (p1.accelerations.size() == jt1.joint_names.size() &&
        p2.accelerations.size() == jt2.joint_names.size()) {
      point.accelerations = p1.accelerations;
      point.accelerations.insert(point.accelerations.end(),
                                  p2.accelerations.begin(), p2.accelerations.end());
    }
    // Populate builtin_interfaces/Duration directly for ROS 2 Jazzy.
    const auto whole_seconds = static_cast<int32_t>(std::floor(t));
    auto nanoseconds = static_cast<int64_t>(
      std::llround((t - static_cast<double>(whole_seconds)) * 1e9));
    point.time_from_start.sec = whole_seconds;
    if (nanoseconds >= 1000000000LL) {
      point.time_from_start.sec += 1;
      nanoseconds -= 1000000000LL;
    }
    point.time_from_start.nanosec = static_cast<uint32_t>(nanoseconds);
    merged.points.push_back(std::move(point));
  }

  RCLCPP_INFO(logger,
    "[%s] Both LIN plans succeeded; executing merged %zu-joint trajectory (%.3f s).",
    stage.c_str(), merged.joint_names.size(), shared_duration);
  const auto executed = combinedgroup.execute(combined_plan);
  if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "[%s] Combined LIN execution failed (code %d).", stage.c_str(), executed.val);
    return false;
  }

  RCLCPP_INFO(logger, "[%s] Combined LIN execution succeeded.", stage.c_str());
  return true;
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

// Plan and execute one collision-checked 12-joint MoveJ to the SRDF named home state.
bool moveCombinedHome(
  const rclcpp::Logger & logger,
  moveit::planning_interface::MoveGroupInterface & combined)
{
  combined.setStartStateToCurrentState();
  combined.setPlanningPipelineId("ompl");
  combined.setPlannerId("RRTConnectkConfigDefault");
  combined.setPlanningTime(kPlanningTime);
  combined.setNumPlanningAttempts(kPlanningAttempts);
  combined.setMaxVelocityScalingFactor(kVelocity);
  combined.setMaxAccelerationScalingFactor(kAcceleration);
  combined.clearPoseTargets();
  combined.clearPathConstraints();

  if (!combined.setNamedTarget("home")) {
    RCLCPP_ERROR(logger, "Named state 'home' is not defined for '%s'.", kCombinedGroup);
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto planned = combined.plan(plan);
  if (planned != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Combined HOME planning failed (code %d).", planned.val);
    return false;
  }

  const auto executed = combined.execute(plan);
  if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Combined HOME execution failed (code %d).", executed.val);
    return false;
  }

  RCLCPP_INFO(logger, "Both arms reached combined HOME.");
  return true;
}

bool runSequence(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & combined,
  moveit::planning_interface::MoveGroupInterface & robot3,
  moveit::planning_interface::MoveGroupInterface & robot4,
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

  // 1) Keep the initial pick-approach movement as one combined OMPL plan.
  if (!moveBoth(logger, combined, pickApproach3, pickApproach4, "pick approach")) {
    return false;
  }

  // 2) Allow only each rebar's gripper-touch links during the pick.
  if (!scene.setObjectCollisionsAllowed(a3.object_id, a3.touch_links, true) ||
      !scene.setObjectCollisionsAllowed(a4.object_id, a4.touch_links, true)) {
    RCLCPP_ERROR(logger, "Failed to configure rebar/gripper contact allowances.");
    return false;
  }

  // 3) Plan both pick LIN trajectories independently, merge, and execute together.
  if (!movel_combined(logger, robot3, robot4, combined, pick3, pick4,
      "pick approach -> pick")) {
    RCLCPP_ERROR(logger, "Stopping sequence: combined pick LIN failed.");
    return false;
  }

  // 4) Attach both objects, preserving their world poses.
  if (!scene.attachWorldObjectPreservingPose(combined, a3.object_id, a3.link, a3.touch_links)) {
    RCLCPP_ERROR(logger, "Failed to attach %s to %s.",
      a3.object_id.c_str(), a3.link.c_str());
    return false;
  }
  if (!scene.attachWorldObjectPreservingPose(combined, a4.object_id, a4.link, a4.touch_links)) {
    RCLCPP_ERROR(logger,
      "Failed to attach %s to %s. Robot 3 object remains attached for recovery.",
      a4.object_id.c_str(), a4.link.c_str());
    return false;
  }

  // 5) Allow carried rebars to contact their respective source inventory during retract.
  if (!scene.setObjectCollisionsAllowed(a3.object_id, {a3.inventory_id}, true) ||
      !scene.setObjectCollisionsAllowed(a4.object_id, {a4.inventory_id}, true)) {
    RCLCPP_ERROR(logger, "Failed to enable rebar/inventory contact during retract.");
    return false;
  }

  // 6) Retract both arms from pick using the combined LIN function.
  if (!movel_combined(logger, robot3, robot4, combined,
      pickApproach3, pickApproach4, "pick -> pick retract")) {
    RCLCPP_ERROR(logger, "Combined pick retract LIN failed; objects remain attached.");
    return false;
  }

  if (!scene.setObjectCollisionsAllowed(a3.object_id, {a3.inventory_id}, false) ||
      !scene.setObjectCollisionsAllowed(a4.object_id, {a4.inventory_id}, false)) {
    RCLCPP_ERROR(logger, "Failed to restore inventory collision checking.");
    return false;
  }

  // 7) Carry both rebars along their arcs. Each paired waypoint uses combined LIN.
  for (std::size_t i = 0; i < arc3.size(); ++i) {
    if (!movel_combined(logger, robot3, robot4, combined, arc3[i], arc4[i],
        "arc waypoint " + std::to_string(i + 1) + "/" + std::to_string(arc3.size()))) {
      RCLCPP_ERROR(logger, "Arc failed; attached rebars remain attached for recovery.");
      return false;
    }
  }

  // 8) Permit rebar-to-jig contact for placement; robot-to-jig checks remain enabled.
  if (!scene.setObjectCollisionsAllowed(a3.object_id, {"jig_clamp"}, true) ||
      !scene.setObjectCollisionsAllowed(a4.object_id, {"jig_clamp"}, true)) {
    RCLCPP_ERROR(logger, "Failed to allow rebar/jig contact.");
    return false;
  }

  // 9) Place approach and place both use the combined LIN function.
  if (!movel_combined(logger, robot3, robot4, combined,
      placeApproach3, placeApproach4, "place approach")) {
    return false;
  }
  if (!movel_combined(logger, robot3, robot4, combined,
      place3, place4, "place")) {
    return false;
  }

  // 10) Detach each rebar into the world at its actual release pose.
  if (!scene.detachObjectToWorldPreservingPose(combined, a3.object_id)) {
    RCLCPP_ERROR(logger, "Failed to detach %s.", a3.object_id.c_str());
    return false;
  }
  if (!scene.detachObjectToWorldPreservingPose(combined, a4.object_id)) {
    RCLCPP_ERROR(logger, "Failed to detach %s.", a4.object_id.c_str());
    return false;
  }

  // 11) Retract both arms using combined LIN, then restore gripper collision checks.
  if (!movel_combined(logger, robot3, robot4, combined,
      placeApproach3, placeApproach4, "place retract")) {
    return false;
  }
  if (!scene.setObjectCollisionsAllowed(a3.object_id, a3.touch_links, false) ||
      !scene.setObjectCollisionsAllowed(a4.object_id, a4.touch_links, false)) {
    RCLCPP_ERROR(logger,
      "Objects placed, but failed to restore gripper collision checking.");
    return false;
  }

  // 12) Once place retract is complete, return both arms home together.
  if (!moveCombinedHome(logger, combined)) {
    RCLCPP_ERROR(logger, "Combined HOME motion failed after place retract.");
    return false;
  }

  RCLCPP_INFO(logger, "Combined Robot 3 + Robot 4 pick/place and HOME completed.");
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
    moveit::planning_interface::MoveGroupInterface robot3(node, kRobot3Group);
    moveit::planning_interface::MoveGroupInterface robot4(node, kRobot4Group);
    combined.setPoseReferenceFrame(kWorld);
    robot3.setPoseReferenceFrame(kWorld);
    robot4.setPoseReferenceFrame(kWorld);
    combined.setPlanningTime(kPlanningTime);
    robot3.setPlanningTime(kPlanningTime);
    robot4.setPlanningTime(kPlanningTime);

    if (!combined.getCurrentState(10.0) ||
        !robot3.getCurrentState(10.0) ||
        !robot4.getCurrentState(10.0)) {
      RCLCPP_ERROR(node->get_logger(), "Combined group's current state unavailable.");
    } else {
      PlanningSceneObjects scene(node);
      success = runSequence(
        node, combined, robot3, robot4, tf_buffer, scene, a3, a4,
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
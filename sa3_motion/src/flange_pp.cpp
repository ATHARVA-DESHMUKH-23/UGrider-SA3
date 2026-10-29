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
#include <map>
#include <set>
#include <functional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
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
constexpr char kWorld[] = "world";

// These defaults preserve the original motion settings. The workcell YAML
// supplies their actual values at runtime.
struct MotionTuning
{
  double planning_time{15.0};
  int planning_attempts{10};
  int retries{3};
  double movej_velocity{0.10};
  double movej_acceleration{0.10};
  double movel_velocity{0.10};
  double movel_acceleration{0.10};
  double ik_timeout{1.0};
};

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
  const MotionTuning & tuning,
  const ArmConfig & a3, const ArmConfig & a4,
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
    group.setPlanningTime(tuning.planning_time);
    group.setNumPlanningAttempts(1);
    group.setMaxVelocityScalingFactor(tuning.movel_velocity);
    group.setMaxAccelerationScalingFactor(tuning.movel_acceleration);
    group.clearPoseTargets();
    group.clearPathConstraints();
  };

  configureLin(robot1);
  configureLin(robot2);

  if (!robot1.setPoseTarget(goal1, a3.link) ||
      !robot2.setPoseTarget(goal2, a4.link)) {
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
  const MotionTuning & tuning,
  const ArmConfig & a3, const ArmConfig & a4,
  moveit::planning_interface::MoveGroupInterface & combined,
  const geometry_msgs::msg::Pose & target3,
  const geometry_msgs::msg::Pose & target4,
  const std::string & stage)
{
  for (int attempt = 1; attempt <= tuning.retries; ++attempt) {
    auto state = combined.getCurrentState(10.0);
    if (!state) {
      RCLCPP_ERROR(logger, "[%s] Current combined robot state unavailable.", stage.c_str());
      return false;
    }
    const auto * group3 = state->getJointModelGroup(a3.group);
    const auto * group4 = state->getJointModelGroup(a4.group);
    if (!group3 || !group4) {
      RCLCPP_ERROR(logger, "SRDF joint model groups '%s'/'%s' unavailable.",
        a3.group.c_str(), a4.group.c_str());
      return false;
    }
    const bool ik3 = state->setFromIK(group3, target3, a3.link, tuning.ik_timeout);
    const bool ik4 = ik3 && state->setFromIK(group4, target4, a4.link, tuning.ik_timeout);
    if (!ik3 || !ik4) {
      RCLCPP_WARN(logger, "[%s] IK attempt %d/%d failed (Robot3=%s, Robot4=%s).",
        stage.c_str(), attempt, tuning.retries, ik3 ? "OK" : "FAIL", ik4 ? "OK" : "FAIL");
      continue;
    }

    combined.setStartStateToCurrentState();
    combined.setPlanningPipelineId("ompl");
    combined.setPlannerId("RRTConnectkConfigDefault");
    combined.setPlanningTime(tuning.planning_time);
    combined.setNumPlanningAttempts(tuning.planning_attempts);
    combined.setMaxVelocityScalingFactor(tuning.movej_velocity);
    combined.setMaxAccelerationScalingFactor(tuning.movej_acceleration);
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
        stage.c_str(), attempt, tuning.retries, planned.val);
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

// Plan and execute one collision-checked 12-joint MoveJ to the SRDF named home state.
bool moveCombinedHome(
  const rclcpp::Logger & logger,
  const MotionTuning & tuning,
  const std::string & combined_group,
  moveit::planning_interface::MoveGroupInterface & combined)
{
  combined.setStartStateToCurrentState();
  combined.setPlanningPipelineId("ompl");
  combined.setPlannerId("RRTConnectkConfigDefault");
  combined.setPlanningTime(tuning.planning_time);
  combined.setNumPlanningAttempts(tuning.planning_attempts);
  combined.setMaxVelocityScalingFactor(tuning.movej_velocity);
  combined.setMaxAccelerationScalingFactor(tuning.movej_acceleration);
  combined.clearPoseTargets();
  combined.clearPathConstraints();

  if (!combined.setNamedTarget("home")) {
    RCLCPP_ERROR(logger, "Named state 'home' is not defined for '%s'.", combined_group.c_str());
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

// ---------------------------------------------------------------------------
// YAML-driven paired-motion program: same load/validate/resolve/execute design
// as rebar_pick_place.cpp, with one pose entry per arm for synchronized moves.
// ---------------------------------------------------------------------------
enum class OffsetFrame { kWorld, kLocal };
enum class MotionType { kMoveJ, kMoveL };

struct ArmTargetSpec
{
  std::string reference;
  OffsetFrame xyz_frame{OffsetFrame::kWorld};
  OffsetFrame rpy_frame{OffsetFrame::kLocal};
  std::array<double, 3> xyz{0.0, 0.0, 0.0};
  std::array<double, 3> rpy{0.0, 0.0, 0.0};
};

struct ConfiguredPlannerStep
{
  std::string id;
  std::string description;
  MotionType motion{MotionType::kMoveJ};
  bool home{false};
  bool repeat_arc{false};
  ArmTargetSpec robot3;
  ArmTargetSpec robot4;
  std::vector<std::string> before_actions;
  std::vector<std::string> after_actions;
};

struct FlangeProgram
{
  std::string combined_group{"robot3_robot4_arms"};
  std::string world_frame{"world"};
  std::string jig_object_id{"jig_clamp"};
  ArmConfig robot3;
  ArmConfig robot4;
  int arc_points{8};
  MotionTuning tuning;
  std::vector<ConfiguredPlannerStep> steps;
};

std::array<double, 3> requiredVector3(
  const YAML::Node & entry, const std::string & field, const std::string & context)
{
  const auto value = entry[field];
  if (!value || !value.IsSequence() || value.size() != 3) {
    throw std::runtime_error(context + "." + field + " must be [x, y, z]");
  }
  std::array<double, 3> result{};
  for (std::size_t i = 0; i < 3; ++i) {
    result[i] = value[i].as<double>();
    if (!std::isfinite(result[i])) {
      throw std::runtime_error(context + "." + field + " contains a non-finite value");
    }
  }
  return result;
}

OffsetFrame readOffsetFrame(
  const std::string & value, const std::string & context)
{
  if (value == "world") return OffsetFrame::kWorld;
  if (value == "local") return OffsetFrame::kLocal;
  throw std::runtime_error(context + " must be 'world' or 'local'");
}

std::vector<std::string> readActions(
  const YAML::Node & yaml_step,
  const std::string & plural, const std::string & singular,
  const std::string & context)
{
  if (yaml_step[plural] && yaml_step[singular]) {
    throw std::runtime_error(context + " cannot contain both " + plural + " and " + singular);
  }
  const auto actions = yaml_step[plural];
  if (!actions) {
    const auto single = yaml_step[singular];
    if (!single) return {};
    const auto action = single.as<std::string>();
    return action.empty() ? std::vector<std::string>{} : std::vector<std::string>{action};
  }
  if (!actions.IsSequence()) {
    throw std::runtime_error(context + "." + plural + " must be a list");
  }
  std::vector<std::string> result;
  for (const auto & value : actions) {
    const auto action = value.as<std::string>();
    if (action.empty()) throw std::runtime_error(context + " contains an empty action");
    result.push_back(action);
  }
  return result;
}

ArmTargetSpec readArmTarget(const YAML::Node & entry, const std::string & context)
{
  if (!entry || !entry.IsMap()) {
    throw std::runtime_error(context + " must be a target dictionary");
  }
  ArmTargetSpec target;
  target.reference = entry["reference"].as<std::string>();
  if (target.reference != "pick_frame" && target.reference != "place_frame" &&
      target.reference != "arc_waypoint" && target.reference != "current_tcp") {
    throw std::runtime_error(context + " has unknown reference: " + target.reference);
  }
  target.xyz_frame = readOffsetFrame(
    entry["offset_frame"].as<std::string>(), context + ".offset_frame");
  target.rpy_frame = readOffsetFrame(
    entry["rpy_frame"].as<std::string>("local"), context + ".rpy_frame");
  target.xyz = requiredVector3(entry, "xyz", context);
  target.rpy = requiredVector3(entry, "rpy", context);
  return target;
}

std::vector<ConfiguredPlannerStep> readPlannerSteps(const YAML::Node & workcell)
{
  const auto program = workcell["planner_sequence"]["flange_pp"];
  if (!program || !program.IsMap()) {
    throw std::runtime_error("workcell.planner_sequence.flange_pp is missing");
  }
  std::vector<ConfiguredPlannerStep> steps;
  std::set<std::string> ids;
  const std::set<std::string> permitted_actions{
    "prepare_pick", "attach_objects", "allow_inventory_contact",
    "restore_inventory_contact", "allow_place_contact",
    "detach_objects", "restore_gripper_contact"};
  for (const auto & stage_name : {"pick", "transfer", "place", "finish"}) {
    const auto stage = program[stage_name];
    if (!stage || !stage.IsSequence() || stage.size() == 0) {
      throw std::runtime_error(std::string("flange_pp.") + stage_name + " must be a nonempty list");
    }
    for (const auto & entry : stage) {
      ConfiguredPlannerStep step;
      step.id = entry["id"].as<std::string>();
      if (step.id.empty() || !ids.insert(step.id).second) {
        throw std::runtime_error("Empty/duplicate flange planner step id: " + step.id);
      }
      step.description = entry["description"].as<std::string>(step.id);
      const auto motion = entry["motion"].as<std::string>();
      if (motion == "moveJ") step.motion = MotionType::kMoveJ;
      else if (motion == "moveL") step.motion = MotionType::kMoveL;
      else throw std::runtime_error(step.id + ": motion must be moveJ or moveL");
      step.before_actions = readActions(
        entry, "before_actions", "before_action", step.id);
      step.after_actions = readActions(
        entry, "after_actions", "after_action", step.id);
      for (const auto & action : step.before_actions) {
        if (!permitted_actions.count(action)) {
          throw std::runtime_error(step.id + ": unknown before_action '" + action + "'");
        }
      }
      for (const auto & action : step.after_actions) {
        if (!permitted_actions.count(action)) {
          throw std::runtime_error(step.id + ": unknown after_action '" + action + "'");
        }
      }
      step.home = entry["reference"].as<std::string>("") == "home";
      step.repeat_arc = entry["repeat"].as<std::string>("") == "arc_waypoints";
      if (entry["repeat"] && !step.repeat_arc) {
        throw std::runtime_error(step.id + ": unknown repeat type");
      }
      if (step.home) {
        if (step.repeat_arc || step.motion != MotionType::kMoveJ) {
          throw std::runtime_error(step.id + ": home must be a nonrepeated moveJ");
        }
      } else {
        if (entry["reference"]) {
          throw std::runtime_error(step.id + ": only 'home' supports a step-level reference");
        }
        step.robot3 = readArmTarget(entry["robot3"], step.id + ".robot3");
        step.robot4 = readArmTarget(entry["robot4"], step.id + ".robot4");
        const bool uses_arc = step.robot3.reference == "arc_waypoint" ||
          step.robot4.reference == "arc_waypoint";
        if (uses_arc != step.repeat_arc ||
            (step.repeat_arc && (step.motion != MotionType::kMoveL ||
              step.robot3.reference != "arc_waypoint" ||
              step.robot4.reference != "arc_waypoint"))) {
          throw std::runtime_error(step.id + ": arc requires paired repeat: arc_waypoints and moveL");
        }
      }
      steps.push_back(std::move(step));
    }
  }
  return steps;
}

ArmConfig readArmConfig(const YAML::Node & entry, const std::string & name)
{
  if (!entry || !entry.IsMap()) {
    throw std::runtime_error("workcell.flange_pp." + name + " must be a dictionary");
  }
  ArmConfig arm;
  arm.name = name;
  arm.group = entry["group"].as<std::string>();
  arm.link = entry["link"].as<std::string>();
  arm.pick_frame = entry["pick_frame"].as<std::string>();
  arm.place_frame = entry["place_frame"].as<std::string>();
  arm.object_id = entry["object_id"].as<std::string>();
  arm.inventory_id = entry["inventory_id"].as<std::string>();
  arm.touch_links = entry["touch_links"].as<std::vector<std::string>>();
  arm.pick_offset = requiredVector3(entry, "pick_offset", name);
  arm.place_offset = requiredVector3(entry, "place_offset", name);
  const auto arc_frames = entry["arc_frames"].as<std::vector<std::string>>();
  if (arc_frames.size() != 3 || arm.touch_links.empty()) {
    throw std::runtime_error(name + ": expected 3 arc frames and nonempty touch_links");
  }
  std::copy(arc_frames.begin(), arc_frames.end(), arm.arc_frames.begin());
  for (const auto & value : {arm.group, arm.link, arm.pick_frame,
    arm.place_frame, arm.object_id, arm.inventory_id}) {
    if (value.empty()) throw std::runtime_error(name + ": empty required name");
  }
  return arm;
}

MotionTuning readMotionTuning(const YAML::Node & config)
{
  if (!config || !config.IsMap()) {
    throw std::runtime_error("workcell.flange_pp.planning is missing");
  }
  MotionTuning t;
  t.planning_time = config["planning_time"].as<double>();
  t.planning_attempts = config["planning_attempts"].as<int>();
  t.retries = config["retries"].as<int>();
  t.ik_timeout = config["ik_timeout"].as<double>();
  t.movej_velocity = config["movej_velocity"].as<double>();
  t.movej_acceleration = config["movej_acceleration"].as<double>();
  t.movel_velocity = config["movel_velocity"].as<double>();
  t.movel_acceleration = config["movel_acceleration"].as<double>();
  if (!std::isfinite(t.planning_time) || t.planning_time <= 0.0 ||
      t.planning_attempts < 1 || t.retries < 1 ||
      !std::isfinite(t.ik_timeout) || t.ik_timeout <= 0.0) {
    throw std::runtime_error("Invalid flange planning time/attempts/retries/IK timeout");
  }
  for (double value : {t.movej_velocity, t.movej_acceleration,
                       t.movel_velocity, t.movel_acceleration}) {
    if (!std::isfinite(value) || value <= 0.0 || value > 1.0) {
      throw std::runtime_error("All flange scaling factors must be in (0, 1]");
    }
  }
  return t;
}

FlangeProgram loadFlangeProgram(const std::string & yaml_path)
{
  const auto doc = YAML::LoadFile(yaml_path);
  const auto workcell = doc["workcell"];
  if (!workcell || !workcell.IsMap()) {
    throw std::runtime_error("Config file lacks a 'workcell' dictionary: " + yaml_path);
  }
  const auto config = workcell["flange_pp"];
  if (!config || !config.IsMap()) {
    throw std::runtime_error("Config lacks workcell.flange_pp: " + yaml_path);
  }
  FlangeProgram result;
  result.world_frame = workcell["world_frame"].as<std::string>("world");
  result.combined_group = config["combined_group"].as<std::string>("robot3_robot4_arms");
  result.jig_object_id = config["jig_object_id"].as<std::string>();
  result.robot3 = readArmConfig(config["robot3"], "robot3");
  result.robot4 = readArmConfig(config["robot4"], "robot4");
  result.arc_points = config["arc_points"].as<int>();
  result.tuning = readMotionTuning(config["planning"]);
  result.steps = readPlannerSteps(workcell);
  if (result.world_frame != "world") {
    // The original combined MoveIt planning frame and collision scene use world.
    throw std::runtime_error("flange_pp currently requires workcell.world_frame == world");
  }
  if (result.combined_group.empty() || result.jig_object_id.empty() ||
      result.arc_points < 2 || result.arc_points > 1000) {
    throw std::runtime_error("Invalid combined_group, jig_object_id or arc_points");
  }
  if (result.robot3.object_id == result.robot4.object_id) {
    throw std::runtime_error("Robot3 and Robot4 must not share a collision object ID");
  }
  return result;
}

// Preserve the pre-existing ROS parameter interface: launch overrides take
// precedence over YAML values for individual robot/task-frame parameters.
void applyRosOverrides(const rclcpp::Node::SharedPtr & node, FlangeProgram & program)
{
  auto override_arm = [&](ArmConfig & arm) {
    const auto prefix = arm.name + "_";
    arm.pick_frame = node->declare_parameter<std::string>(prefix + "pick_frame", arm.pick_frame);
    arm.place_frame = node->declare_parameter<std::string>(prefix + "place_frame", arm.place_frame);
    arm.object_id = node->declare_parameter<std::string>(prefix + "object_id", arm.object_id);
    arm.inventory_id = node->declare_parameter<std::string>(prefix + "inventory_id", arm.inventory_id);
    const auto frames = node->declare_parameter<std::vector<std::string>>(
      prefix + "arc_frames",
      std::vector<std::string>(arm.arc_frames.begin(), arm.arc_frames.end()));
    if (frames.size() != 3) throw std::runtime_error(prefix + "arc_frames must contain 3 frames");
    std::copy(frames.begin(), frames.end(), arm.arc_frames.begin());
    const auto pick = node->declare_parameter<std::vector<double>>(
      prefix + "pick_offset",
      std::vector<double>(arm.pick_offset.begin(), arm.pick_offset.end()));
    const auto place = node->declare_parameter<std::vector<double>>(
      prefix + "place_offset",
      std::vector<double>(arm.place_offset.begin(), arm.place_offset.end()));
    if (pick.size() != 3 || place.size() != 3) {
      throw std::runtime_error(prefix + "pick/place_offset must contain 3 values");
    }
    std::copy(pick.begin(), pick.end(), arm.pick_offset.begin());
    std::copy(place.begin(), place.end(), arm.place_offset.begin());
  };
  override_arm(program.robot3);
  override_arm(program.robot4);
  program.arc_points = node->declare_parameter<int>("arc_points", program.arc_points);
  if (program.arc_points < 2 || program.arc_points > 1000) {
    throw std::runtime_error("arc_points must be in [2, 1000]");
  }
}

geometry_msgs::msg::Pose applyTargetOffset(
  const geometry_msgs::msg::Pose & reference, const ArmTargetSpec & spec)
{
  geometry_msgs::msg::Pose target = reference;
  tf2::Quaternion q_ref(reference.orientation.x, reference.orientation.y,
    reference.orientation.z, reference.orientation.w);
  q_ref.normalize();
  const tf2::Vector3 delta(spec.xyz[0], spec.xyz[1], spec.xyz[2]);
  const auto translated = spec.xyz_frame == OffsetFrame::kLocal ?
    tf2::quatRotate(q_ref, delta) : delta;
  target.position.x += translated.x();
  target.position.y += translated.y();
  target.position.z += translated.z();
  tf2::Quaternion q_offset;
  q_offset.setRPY(spec.rpy[0], spec.rpy[1], spec.rpy[2]);
  const auto q_target = spec.rpy_frame == OffsetFrame::kLocal ?
    (q_ref * q_offset) : (q_offset * q_ref);
  tf2::Quaternion normalized = q_target;
  normalized.normalize();
  target.orientation.x = normalized.x();
  target.orientation.y = normalized.y();
  target.orientation.z = normalized.z();
  target.orientation.w = normalized.w();
  return target;
}

geometry_msgs::msg::Pose resolveTarget(
  const ArmTargetSpec & spec,
  moveit::planning_interface::MoveGroupInterface & move_group,
  const std::string & tip_link,
  const std::map<std::string, geometry_msgs::msg::Pose> & references,
  const std::string & context)
{
  if (spec.reference == "current_tcp") {
    if (!move_group.getCurrentState(10.0)) {
      throw std::runtime_error(context + ": current TCP unavailable");
    }
    return applyTargetOffset(move_group.getCurrentPose(tip_link).pose, spec);
  }
  const auto it = references.find(spec.reference);
  if (it == references.end()) {
    throw std::runtime_error(context + ": unknown/unavailable pose '" + spec.reference + "'");
  }
  return applyTargetOffset(it->second, spec);
}

using ConfiguredActionHandler = std::function<bool(const std::string &)>;

bool executeConfiguredPlannerSteps(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & combined,
  moveit::planning_interface::MoveGroupInterface & robot3,
  moveit::planning_interface::MoveGroupInterface & robot4,
  const FlangeProgram & program,
  const std::map<std::string, geometry_msgs::msg::Pose> & references3,
  const std::map<std::string, geometry_msgs::msg::Pose> & references4,
  const std::vector<geometry_msgs::msg::Pose> & arc3,
  const std::vector<geometry_msgs::msg::Pose> & arc4,
  const ConfiguredActionHandler & action_handler)
{
  const auto logger = node->get_logger();
  auto run_actions = [&](const std::vector<std::string> & actions,
                         const std::string & label) {
    for (const auto & action : actions) {
      if (!action_handler(action)) {
        RCLCPP_ERROR(logger, "[%s] collision/action '%s' failed",
          label.c_str(), action.c_str());
        return false;
      }
    }
    return true;
  };

  for (const auto & step : program.steps) {
    const std::string label = step.id + " " + step.description;
    if (!run_actions(step.before_actions, label)) return false;

    if (step.home) {
      if (!moveCombinedHome(logger, program.tuning, program.combined_group, combined)) return false;
    } else {
      const std::size_t count = step.repeat_arc ? arc3.size() : 1;
      for (std::size_t i = 0; i < count; ++i) {
        auto refs3 = references3;
        auto refs4 = references4;
        if (step.repeat_arc) {
          refs3["arc_waypoint"] = arc3[i];
          refs4["arc_waypoint"] = arc4[i];
        }
        const auto goal3 = resolveTarget(
          step.robot3, robot3, program.robot3.link, refs3, label + ".robot3");
        const auto goal4 = resolveTarget(
          step.robot4, robot4, program.robot4.link, refs4, label + ".robot4");
        const auto waypoint_label = step.repeat_arc ?
          "arc waypoint " + std::to_string(i + 1) + "/" + std::to_string(count) : label;
        RCLCPP_INFO(logger, "[%s] YAML motion=%s", waypoint_label.c_str(),
          step.motion == MotionType::kMoveJ ? "moveJ (combined OMPL)" : "moveL (paired Pilz LIN)");
        const bool succeeded = step.motion == MotionType::kMoveJ ?
          planAndExecuteBoth(logger, program.tuning, program.robot3, program.robot4,
            combined, goal3, goal4, waypoint_label) :
          movel_combined(logger, program.tuning, program.robot3, program.robot4,
            robot3, robot4, combined, goal3, goal4, waypoint_label);
        if (!succeeded) {
          RCLCPP_ERROR(logger, "[%s] Motion failed, aborting YAML program", waypoint_label.c_str());
          return false;
        }
      }
    }
    if (!run_actions(step.after_actions, label)) return false;
  }
  return true;
}

bool runConfiguredProgram(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & combined,
  moveit::planning_interface::MoveGroupInterface & robot3,
  moveit::planning_interface::MoveGroupInterface & robot4,
  tf2_ros::Buffer & tf,
  PlanningSceneObjects & scene,
  const FlangeProgram & program)
{
  const auto logger = node->get_logger();
  const auto & a3 = program.robot3;
  const auto & a4 = program.robot4;
  geometry_msgs::msg::Pose pick3, place3, pick4, place4;
  geometry_msgs::msg::Pose a3p0, a3p1, a3p2, a4p0, a4p1, a4p2;
  if (!lookupPose(logger, tf, a3.pick_frame, pick3) ||
      !lookupPose(logger, tf, a3.place_frame, place3) ||
      !lookupPose(logger, tf, a4.pick_frame, pick4) ||
      !lookupPose(logger, tf, a4.place_frame, place4) ||
      !lookupPose(logger, tf, a3.arc_frames[0], a3p0) ||
      !lookupPose(logger, tf, a3.arc_frames[1], a3p1) ||
      !lookupPose(logger, tf, a3.arc_frames[2], a3p2) ||
      !lookupPose(logger, tf, a4.arc_frames[0], a4p0) ||
      !lookupPose(logger, tf, a4.arc_frames[1], a4p1) ||
      !lookupPose(logger, tf, a4.arc_frames[2], a4p2)) {
    return false;
  }

  std::map<std::string, geometry_msgs::msg::Pose> refs3{
    {"pick_frame", offsetPose(pick3, a3.pick_offset)},
    {"place_frame", offsetPose(place3, a3.place_offset)}};
  std::map<std::string, geometry_msgs::msg::Pose> refs4{
    {"pick_frame", offsetPose(pick4, a4.pick_offset)},
    {"place_frame", offsetPose(place4, a4.place_offset)}};
  const auto arc3 = arcThrough3(a3p0, a3p1, a3p2, program.arc_points);
  const auto arc4 = arcThrough3(a4p0, a4p1, a4p2, program.arc_points);
  if (arc3.empty() || arc4.empty() || arc3.size() != arc4.size()) {
    RCLCPP_ERROR(logger, "Could not build both arcs, or arc point counts differ");
    return false;
  }
  if (!scene.verifyObjectState(a3.object_id, true, false) ||
      !scene.verifyObjectState(a4.object_id, true, false)) {
    RCLCPP_ERROR(logger, "Both flange objects must be world collision objects before starting");
    return false;
  }

  // Only these reviewed actions may change MoveIt's scene/attachment state.
  // Do not auto-detach on a failed motion: the actual carried pose is unknown.
  const ConfiguredActionHandler action = [&](const std::string & name) -> bool {
    if (name.empty()) return true;
    if (name == "prepare_pick") {
      return scene.setObjectCollisionsAllowed(a3.object_id, a3.touch_links, true) &&
        scene.setObjectCollisionsAllowed(a4.object_id, a4.touch_links, true);
    }
    if (name == "attach_objects") {
      return scene.attachWorldObjectPreservingPose(combined, a3.object_id, a3.link, a3.touch_links) &&
        scene.attachWorldObjectPreservingPose(combined, a4.object_id, a4.link, a4.touch_links);
    }
    if (name == "allow_inventory_contact") {
      return scene.setObjectCollisionsAllowed(a3.object_id, {a3.inventory_id}, true) &&
        scene.setObjectCollisionsAllowed(a4.object_id, {a4.inventory_id}, true);
    }
    if (name == "restore_inventory_contact") {
      return scene.setObjectCollisionsAllowed(a3.object_id, {a3.inventory_id}, false) &&
        scene.setObjectCollisionsAllowed(a4.object_id, {a4.inventory_id}, false);
    }
    if (name == "allow_place_contact") {
      return scene.setObjectCollisionsAllowed(a3.object_id, {program.jig_object_id}, true) &&
        scene.setObjectCollisionsAllowed(a4.object_id, {program.jig_object_id}, true);
    }
    if (name == "detach_objects") {
      return scene.detachObjectToWorldPreservingPose(combined, a3.object_id) &&
        scene.detachObjectToWorldPreservingPose(combined, a4.object_id);
    }
    if (name == "restore_gripper_contact") {
      return scene.setObjectCollisionsAllowed(a3.object_id, a3.touch_links, false) &&
        scene.setObjectCollisionsAllowed(a4.object_id, a4.touch_links, false);
    }
    RCLCPP_ERROR(logger, "Unknown YAML planner safety action '%s'", name.c_str());
    return false;
  };

  if (!executeConfiguredPlannerSteps(node, combined, robot3, robot4, program,
      refs3, refs4, arc3, arc4, action)) {
    return false;
  }
  RCLCPP_INFO(logger, "Robot3 + Robot4 YAML flange pick/place and combined home completed");
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(kNodeName);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });
  bool success = false;
  try {
    const auto config_package = node->declare_parameter<std::string>(
      "workcell_config_package", "sa2_description");
    const auto config_file = node->declare_parameter<std::string>(
      "workcell_config_file", "config/workcell.yaml");
    const auto config_path_override = node->declare_parameter<std::string>(
      "workcell_config_path", "");
    const auto config_path = config_path_override.empty() ?
      ament_index_cpp::get_package_share_directory(config_package) + "/" + config_file :
      config_path_override;
    RCLCPP_INFO(node->get_logger(), "Loading flange workcell YAML: %s", config_path.c_str());
    auto program = loadFlangeProgram(config_path);
    applyRosOverrides(node, program);
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf_buffer.setUsingDedicatedThread(true);
    tf2_ros::TransformListener tf_listener(tf_buffer, node, false);
    moveit::planning_interface::MoveGroupInterface combined(node, program.combined_group);
    moveit::planning_interface::MoveGroupInterface robot3(node, program.robot3.group);
    moveit::planning_interface::MoveGroupInterface robot4(node, program.robot4.group);
    combined.setPoseReferenceFrame(program.world_frame);
    robot3.setPoseReferenceFrame(program.world_frame);
    robot4.setPoseReferenceFrame(program.world_frame);
    combined.setPlanningTime(program.tuning.planning_time);
    robot3.setPlanningTime(program.tuning.planning_time);
    robot4.setPlanningTime(program.tuning.planning_time);
    if (!combined.getCurrentState(10.0) || !robot3.getCurrentState(10.0) ||
        !robot4.getCurrentState(10.0)) {
      RCLCPP_ERROR(node->get_logger(), "Combined/individual groups' current state unavailable");
    } else {
      PlanningSceneObjects scene(node);
      success = runConfiguredProgram(
        node, combined, robot3, robot4, tf_buffer, scene, program);
    }
    (void)tf_listener;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Flange motion node failed: %s", e.what());
  }
  executor.cancel();
  if (spin_thread.joinable()) spin_thread.join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
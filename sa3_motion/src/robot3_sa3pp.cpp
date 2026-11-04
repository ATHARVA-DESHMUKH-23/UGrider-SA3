// Synchronized Robot3+Robot4 final assembly. YAML assigns each object to one arm.
// Combined Robot 3 + Robot 4 pick/place node.
// Combined MoveJ uses OMPL; combined transfer PTP uses Pilz PTP.
// Dual Cartesian moves are solved into one 12-joint state at every step;
// only combined-state collision validation is used.
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
#include <exception>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <future>
#include <mutex>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <Eigen/Geometry>
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

// Motion scales are provided by the selected cross-section recipe at runtime.
// Robot identities and tool links are read from the fixed workcell YAML.
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
  std::array<double, 3> pick_offset_rpy{0.0, 0.0, 0.0};
  std::array<double, 3> place_offset_rpy{0.0, 0.0, 0.0};
  bool pick_offset_local{false};
  bool place_offset_local{false};
  bool pick_offset_rpy_local{true};
  bool place_offset_rpy_local{true};
};

// XYZ is applied using the ORIGINAL reference orientation; RPY then rotates
// the target tool pose. This avoids unintentionally rotating the XYZ offset.
// RPY values are in radians. Local RPY is post-multiplied; world is pre-multiplied.
geometry_msgs::msg::Pose offsetPose(
  const geometry_msgs::msg::Pose & p, const std::array<double, 3> & d,
  bool xyz_local, const std::array<double, 3> & rpy, bool rpy_local)
{
  auto out = p;
  tf2::Quaternion q_ref(p.orientation.x, p.orientation.y,
                        p.orientation.z, p.orientation.w);
  q_ref.normalize();
  tf2::Vector3 offset(d[0], d[1], d[2]);
  if (xyz_local) offset = tf2::quatRotate(q_ref, offset);
  out.position.x += offset.x();
  out.position.y += offset.y();
  out.position.z += offset.z();

  tf2::Quaternion q_delta;
  q_delta.setRPY(rpy[0], rpy[1], rpy[2]);
  tf2::Quaternion q_target = rpy_local ? q_ref * q_delta : q_delta * q_ref;
  q_target.normalize();
  out.orientation.x = q_target.x();
  out.orientation.y = q_target.y();
  out.orientation.z = q_target.z();
  out.orientation.w = q_target.w();
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

// Set before each MoveL; validates the merged paired trajectory.
std::function<bool(moveit::planning_interface::MoveGroupInterface &,
  const moveit::planning_interface::MoveGroupInterface::Plan &,
  const rclcpp::Logger &, const std::string &)> validateMergedTrajectory;

// Coordinated Cartesian motion. Generate both TCP interpolation paths together,
// solve IK into ONE shared 12-joint state and validate the combined collision state.
// Neither arm has an independently collision-checked/stationary-other-arm plan.
// The whole combined trajectory is timed and validated before execution.
bool movel_combined(
  const rclcpp::Logger & logger,
  const MotionTuning & tuning,
  const ArmConfig & a3, const ArmConfig & a4,
  moveit::planning_interface::MoveGroupInterface & arm3,
  moveit::planning_interface::MoveGroupInterface & arm4,
  moveit::planning_interface::MoveGroupInterface & both,
  const geometry_msgs::msg::Pose & goal3,
  const geometry_msgs::msg::Pose & goal4,
  const std::string & stage)
{
  auto start = both.getCurrentState(10.0);
  if (!start) return false;
  const auto * j3 = start->getJointModelGroup(a3.group);
  const auto * j4 = start->getJointModelGroup(a4.group);
  if (!j3 || !j4) return false;
  start->update();

  const auto toPose = [](const Eigen::Isometry3d & t) {
    geometry_msgs::msg::Pose p;
    p.position.x = t.translation().x();
    p.position.y = t.translation().y();
    p.position.z = t.translation().z();
    Eigen::Quaterniond q(t.rotation());
    q.normalize();
    p.orientation.x=q.x(); p.orientation.y=q.y();
    p.orientation.z=q.z(); p.orientation.w=q.w();
    return p;
  };
  const auto from3 = toPose(start->getGlobalLinkTransform(a3.link));
  const auto from4 = toPose(start->getGlobalLinkTransform(a4.link));
  const auto interpolationCount = [](const geometry_msgs::msg::Pose & a,
                                     const geometry_msgs::msg::Pose & b) {
    const double dx=b.position.x-a.position.x;
    const double dy=b.position.y-a.position.y;
    const double dz=b.position.z-a.position.z;
    Eigen::Quaterniond qa(a.orientation.w,a.orientation.x,a.orientation.y,a.orientation.z);
    Eigen::Quaterniond qb(b.orientation.w,b.orientation.x,b.orientation.y,b.orientation.z);
    const double angle=qa.angularDistance(qb);
    return static_cast<int>(std::ceil(std::max(std::sqrt(dx*dx+dy*dy+dz*dz)/0.01,angle/0.025)));
  };
  const int n=std::clamp(std::max({1,interpolationCount(from3,goal3),interpolationCount(from4,goal4)}),1,2000);
  if (n==2000) {
    RCLCPP_ERROR(logger,"[%s] Cartesian path sample limit reached",stage.c_str());
    return false;
  }
  const auto lerp=[](const geometry_msgs::msg::Pose &a,const geometry_msgs::msg::Pose &b,double f) {
    geometry_msgs::msg::Pose p;
    p.position.x=a.position.x+f*(b.position.x-a.position.x);
    p.position.y=a.position.y+f*(b.position.y-a.position.y);
    p.position.z=a.position.z+f*(b.position.z-a.position.z);
    Eigen::Quaterniond qa(a.orientation.w,a.orientation.x,a.orientation.y,a.orientation.z);
    Eigen::Quaterniond qb(b.orientation.w,b.orientation.x,b.orientation.y,b.orientation.z);
    const Eigen::Quaterniond q=qa.slerp(f,qb).normalized();
    p.orientation.w=q.w();p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();
    return p;
  };
  robot_trajectory::RobotTrajectory rt(both.getRobotModel(),both.getName());
  rt.addSuffixWayPoint(*start,0.0);
  auto working=std::make_shared<moveit::core::RobotState>(*start);
  for (int i=1;i<=n;++i) {
    const double f=static_cast<double>(i)/n;
    // Both IK solutions are in the SAME evolving RobotState.
    const bool ok3=working->setFromIK(j3,lerp(from3,goal3,f),a3.link,tuning.ik_timeout);
    const bool ok4=ok3 && working->setFromIK(j4,lerp(from4,goal4,f),a4.link,tuning.ik_timeout);
    if(!ok3 || !ok4) {
      RCLCPP_WARN(logger,"[%s] coordinated Cartesian IK failed at %d/%d (r3=%d r4=%d)",
        stage.c_str(),i,n,ok3,ok4);
      return false;
    }
    working->update();
    rt.addSuffixWayPoint(*working,0.0);
  }
  trajectory_processing::IterativeParabolicTimeParameterization time_parameterizer;
  if (!time_parameterizer.computeTimeStamps(rt,tuning.movel_velocity,tuning.movel_acceleration)) {
    RCLCPP_ERROR(logger,"[%s] 12-joint time parameterization failed",stage.c_str());
    return false;
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::robotStateToRobotStateMsg(*start,plan.start_state_);
  rt.getRobotTrajectoryMsg(plan.trajectory_);
  if (!validateMergedTrajectory(both,plan,logger,stage)) return false;
  RCLCPP_INFO(logger,"[%s] coordinated, collision-validated 12-joint Cartesian path: %d steps, scale %.3f",
    stage.c_str(),n,tuning.movel_velocity);
  const auto executed=both.execute(plan);
  if(executed!=moveit::core::MoveItErrorCode::SUCCESS)
    throw std::runtime_error("Combined execution failed; no planner fallback");
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
  const std::string & stage,
  bool pilz_ptp = false)
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
    // PTP is a JOINT-space point-to-point transfer. Both TCP targets are
    // solved with IK, then both arms are planned TOGETHER so the planning
    // scene checks inter-arm and attached-object collisions.
    combined.setPlanningPipelineId(
      pilz_ptp ? "pilz_industrial_motion_planner" : "ompl");
    combined.setPlannerId(pilz_ptp ? "PTP" : "RRTConnectkConfigDefault");
    combined.setPlanningTime(tuning.planning_time);
    combined.setNumPlanningAttempts(pilz_ptp ? 1 : tuning.planning_attempts);
    combined.setMaxVelocityScalingFactor(tuning.movej_velocity);
    combined.setMaxAccelerationScalingFactor(tuning.movej_acceleration);
    combined.clearPoseTargets();
    combined.clearPathConstraints();

    if (!combined.setJointValueTarget(*state)) {
      RCLCPP_WARN(logger, "[%s] Could not set combined 12-joint target.", stage.c_str());
      continue;
    }

    RCLCPP_INFO(logger, "[%s] Planning combined 12-joint %s to TF goals.",
      stage.c_str(), pilz_ptp ? "Pilz PTP" : "OMPL MoveJ");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planned = combined.plan(plan);
    if (planned != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(logger, "[%s] combined %s planning attempt %d/%d failed (code %d).",
        stage.c_str(), pilz_ptp ? "Pilz PTP" : "OMPL",
        attempt, tuning.retries, planned.val);
      continue;
    }

    const auto executed = combined.execute(plan);
    if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "[%s] combined trajectory execution failed (code %d).",
        stage.c_str(), executed.val);
      throw std::runtime_error("Controller execution failed; do not retry planning or fallback");
    }
    RCLCPP_INFO(logger, "[%s] both arms completed combined %s.",
      stage.c_str(), pilz_ptp ? "Pilz PTP" : "MoveJ");
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


// Final assembly: YAML assigns each object to Robot3 or Robot4.
struct FinalAssemblyContext {
  rclcpp::Node::SharedPtr node;
  PlanningSceneObjects *scene{};
  moveit::planning_interface::MoveGroupInterface *arm3{};
  moveit::planning_interface::MoveGroupInterface *arm4{};
  moveit::planning_interface::MoveGroupInterface *both{};
  std::string object;
  std::vector<std::string> children, active;
  std::vector<std::string> robot3_children, robot4_children;
  std::vector<std::string> touch3, touch4;
  int main_owner{3};
  MotionTuning tuning;
  ArmConfig a3, a4;
  std::vector<double> lin_scales;
  std::vector<std::string> strategies;
};

// Every carried item has ONE owner. Its contact allowance is scoped to that
// owner's gripper and the jigs; other robot links are never blanket-exempted.
int owner(const FinalAssemblyContext & c,const std::string & id) {
  if (id==c.object) return c.main_owner;
  if (std::find(c.robot3_children.begin(),c.robot3_children.end(),id)!=c.robot3_children.end()) return 3;
  if (std::find(c.robot4_children.begin(),c.robot4_children.end(),id)!=c.robot4_children.end()) return 4;
  throw std::runtime_error("No configured robot owner for "+id);
}

bool contact(FinalAssemblyContext & c, bool allow) {
  // c.active already contains the main object (inserted in runAssembly),
  // followed by all available children. Never insert c.object a second time:
  // doing so creates an invalid object-vs-itself ACM pair.
  const auto & objects = c.active;
  if (std::count(objects.begin(), objects.end(), c.object) != 1 ||
      std::set<std::string>(objects.begin(), objects.end()).size() != objects.size()) {
    RCLCPP_ERROR(c.node->get_logger(),
      "Invalid assembly collision configuration: duplicate/missing object IDs");
    return false;
  }
  bool ok=true;
  for (const auto & obj:objects) {
    ok &= c.scene->setObjectCollisionsAllowed(obj, owner(c,obj)==3 ? c.touch3 : c.touch4,allow);
    ok &= c.scene->setObjectCollisionsAllowed(obj,{"jig_clamp","sa3_main_jig"},allow);
  }
  // A rigid assembly's overlapping rebars must be permitted to touch one another,
  // regardless of which arm is carrying each member.
  for (size_t i=0;i<objects.size();++i)
    for (size_t j=i+1;j<objects.size();++j)
      ok &= c.scene->setCollisionPairAllowed(objects[i],objects[j],allow);
  return ok;
}

bool attach(FinalAssemblyContext & c) {
  for (const auto & id:c.active) {
    const int which=owner(c,id);
    auto & arm=which==3 ? *c.arm3 : *c.arm4;
    const auto & link=which==3 ? c.a3.link : c.a4.link;
    const auto & touch=which==3 ? c.touch3 : c.touch4;
    if (!c.scene->attachWorldObjectPreservingPose(arm,id,link,touch) ||
        !c.scene->verifyObjectState(id,false,true,link)) {
      RCLCPP_ERROR(c.node->get_logger(),"Failed to attach %s to Robot%d; recovery required",id.c_str(),which);
      return false;
    }
    RCLCPP_INFO(c.node->get_logger(),"ATTACHED %s -> Robot%d (%s)",id.c_str(),which,link.c_str());
  }
  return true;
}

bool detach(FinalAssemblyContext & c) {
  for (const auto & id:c.active) {
    const int which=owner(c,id);
    auto & arm=which==3 ? *c.arm3 : *c.arm4;
    if (!c.scene->detachObjectToWorldPreservingPose(arm,id) ||
        !c.scene->verifyObjectState(id,true,false)) {
      RCLCPP_ERROR(c.node->get_logger(),"Failed to detach %s from Robot%d",id.c_str(),which);
      return false;
    }
  }
  return true;
}

bool pairedMotion(FinalAssemblyContext & c,
                  const geometry_msgs::msg::Pose & p3,
                  const geometry_msgs::msg::Pose & p4,
                  const std::string & label, const std::string & strategy) {
  if (strategy == "moveJ") {
    return planAndExecuteBoth(c.node->get_logger(), c.tuning, c.a3, c.a4,
       *c.both, p3, p4, label);
  }
  if (strategy == "moveL") {
    for (double scale : c.lin_scales) {
      auto tuned = c.tuning;
      tuned.movel_velocity = scale;
      tuned.movel_acceleration = std::min(c.tuning.movel_acceleration, scale);
      RCLCPP_INFO(c.node->get_logger(), "[%s] dual Pilz LIN scale %.3f", label.c_str(), scale);
      if (movel_combined(c.node->get_logger(), tuned, c.a3, c.a4,
           *c.arm3, *c.arm4, *c.both, p3, p4, label)) return true;
    }
    return false;
  }
  // Pilz CIRC for independently specified two-arm arcs is NOT safely mapped
  // to this paired-trajectory implementation; reject instead of swapping modes.
  RCLCPP_WARN(c.node->get_logger(), "[%s] paired %s not implemented", label.c_str(), strategy.c_str());
  return false;
}

bool motion(FinalAssemblyContext & c,
            const geometry_msgs::msg::Pose & p3,
            const geometry_msgs::msg::Pose & p4,
            const std::string & label, const std::string & method) {
  if (method != "fallback") return pairedMotion(c,p3,p4,label,method);
  for (const auto & strategy : c.strategies)
    if (pairedMotion(c,p3,p4,label,strategy)) return true;
  return false;
}

bool robot1Sleep(const rclcpp::Node::SharedPtr & node, const MotionTuning & tuning) {
  moveit::planning_interface::MoveGroupInterface arm(node, "robot1_arm");
  arm.setStartStateToCurrentState();
  arm.setPlanningPipelineId("ompl");
  arm.setPlannerId("RRTConnectkConfigDefault");
  arm.setPlanningTime(tuning.planning_time);
  arm.setNumPlanningAttempts(tuning.planning_attempts);
  arm.setMaxVelocityScalingFactor(tuning.movej_velocity);
  arm.setMaxAccelerationScalingFactor(tuning.movej_acceleration);
  if (!arm.setNamedTarget("robot1_sleep")) return false;
  for (int i=0; i<tuning.retries; ++i) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
      return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }
  return false;
}

bool bothHome(FinalAssemblyContext & c) {
  auto state = c.both->getCurrentState(10.0);
  if (!state) return false;
  // Use individual named HOME states, not a possibly undefined combined HOME.
  auto s3 = c.arm3->getCurrentState(5.0);
  auto s4 = c.arm4->getCurrentState(5.0);
  if (!s3 || !s4) return false;
  const auto *home3=s3->getRobotModel()->getJointModelGroup(c.a3.group);
  const auto *home4=s4->getRobotModel()->getJointModelGroup(c.a4.group);
  if (!home3 || !home4) return false;
  const auto named3 = c.arm3->getNamedTargetValues("home");
  const auto named4 = c.arm4->getNamedTargetValues("home");
  if (named3.empty() || named4.empty()) return false;
  for (const auto & [joint, value] : named3) s3->setVariablePosition(joint, value);
  for (const auto & [joint, value] : named4) s4->setVariablePosition(joint, value);
  const auto *j3=state->getJointModelGroup(c.a3.group);
  const auto *j4=state->getJointModelGroup(c.a4.group);
  if (!j3 || !j4) return false;
  std::vector<double> q3,q4;
  s3->copyJointGroupPositions(j3,q3);
  s4->copyJointGroupPositions(j4,q4);
  state->setJointGroupPositions(j3,q3);
  state->setJointGroupPositions(j4,q4);
  c.both->setStartStateToCurrentState();
  c.both->setPlanningPipelineId("ompl");
  c.both->setPlannerId("RRTConnectkConfigDefault");
  c.both->setNumPlanningAttempts(c.tuning.planning_attempts);
  c.both->setPlanningTime(c.tuning.planning_time);
  c.both->setMaxVelocityScalingFactor(c.tuning.movej_velocity);
  c.both->setMaxAccelerationScalingFactor(c.tuning.movej_acceleration);
  if (!c.both->setJointValueTarget(*state)) return false;
  for (int i=0;i<c.tuning.retries;++i) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (c.both->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
      return c.both->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }
  return false;
}

template<class T>
T param(const rclcpp::Node::SharedPtr & n, const std::string & key, T fallback) {
  if (!n->has_parameter(key)) n->declare_parameter<T>(key, fallback);
  return n->get_parameter(key).get_value<T>();
}

std::array<double,3> offsetParam(const rclcpp::Node::SharedPtr & n, const std::string & key,
                                std::array<double,3> fallback) {
  auto v=param<std::vector<double>>(n,key,{fallback[0],fallback[1],fallback[2]});
  if (v.size()!=3) throw std::runtime_error(key+" requires 3 values");
  return {v[0],v[1],v[2]};
}

bool runAssembly(FinalAssemblyContext & c, tf2_ros::Buffer & tf) {
  const auto & n=c.node;
  const auto pick3=param<std::string>(n,"pick_frame","sa3_pick_point");
  const auto place3=param<std::string>(n,"place_frame","sa3_place");
  const auto pick4=param<std::string>(n,"robot4_pick_frame","sa3_pick_point_r");
  const auto place4=param<std::string>(n,"robot4_place_frame","sa3_place_r");
  geometry_msgs::msg::Pose pk3,pl3,pk4,pl4;
  if (!lookupPose(n->get_logger(),tf,pick3,pk3) || !lookupPose(n->get_logger(),tf,place3,pl3) ||
      !lookupPose(n->get_logger(),tf,pick4,pk4) || !lookupPose(n->get_logger(),tf,place4,pl4)) return false;

  // Separate WORLD-frame pick-approach offsets for each TCP.  The legacy
  // parameter remains a fallback for Robot3 for compatibility with old recipes.
  const auto pick_app3=offsetParam(n,"robot3_pick_approach_offset_world",
      offsetParam(n,"pick_approach_offset_world",{-0.1,0,0}));
  const auto pick_app4=offsetParam(n,"robot4_pick_approach_offset_world",{0.1,0,0});
  RCLCPP_INFO(n->get_logger(),
      "Pick approach offsets (world m): Robot3 [%.3f, %.3f, %.3f]; Robot4 [%.3f, %.3f, %.3f]",
      pick_app3[0], pick_app3[1], pick_app3[2],
      pick_app4[0], pick_app4[1], pick_app4[2]);
  const auto pick_ret=offsetParam(n,"pick_retract_offset_world",{0,-0.2,0});
  const auto place_app=offsetParam(n,"place_approach_offset_world",{0,0,0.3});
  // Independent WORLD-frame place-retract offsets for each arm.
  // Preserve legacy shared offset as Robot3's fallback for older recipes.
  const auto place_ret3=offsetParam(n,"robot3_place_retract_offset_world",
      offsetParam(n,"place_retract_offset_world",{-0.05,0,0}));
  const auto place_ret4=offsetParam(n,"robot4_place_retract_offset_world",{0.05,0,0});
  RCLCPP_INFO(n->get_logger(),
      "Place retract offsets (world m): Robot3 [%.3f, %.3f, %.3f]; Robot4 [%.3f, %.3f, %.3f]",
      place_ret3[0], place_ret3[1], place_ret3[2],
      place_ret4[0], place_ret4[1], place_ret4[2]);
  const auto method=[&](const char *key, const char *def) {
    return param<std::string>(n,key,def);
  };
  if (!c.scene->verifyObjectState(c.object,true,false)) return false;
  c.active.clear();
  c.active.push_back(c.object);
  for (const auto & id:c.children) {
    if (id==c.object) throw std::runtime_error("Main object repeated in children: "+id);
    if (c.scene->verifyObjectState(id,true,false)) c.active.push_back(id);
    else RCLCPP_WARN(n->get_logger(),"Child %s not in world; skipping",id.c_str());
  }

  // P1: independent WORLD-frame pick approach targets (same synchronized execution).
  if (!motion(c, offsetPose(pk3,pick_app3,false,{0,0,0},true),
                 offsetPose(pk4,pick_app4,false,{0,0,0},true),
                 "pick approach",method("pick_approach_motion","fallback"))) return false;
  // Permit only each picked object's own gripper contact before approaching.
  if (!contact(c,true)) return false;
  if (!motion(c,pk3,pk4,"pick",method("pick_motion","moveL"))) return false;
  if (!attach(c)) return false;
  if (!motion(c,offsetPose(pk3,pick_ret,false,{0,0,0},true),
                offsetPose(pk4,pick_ret,false,{0,0,0},true),
                "pick retract",method("pick_retract_motion","moveL"))) return false;
  if (!motion(c,offsetPose(pl3,place_app,false,{0,0,0},true),
                offsetPose(pl4,place_app,false,{0,0,0},true),
                "place approach",method("place_approach_motion","fallback"))) return false;
  if (!motion(c,pl3,pl4,"place",method("place_motion","moveL"))) return false;
  if (!detach(c)) return false;
  if (!motion(c,offsetPose(pl3,place_ret3,false,{0,0,0},true),
                offsetPose(pl4,place_ret4,false,{0,0,0},true),
                "place retract",method("place_retract_motion","moveL"))) return false;
  if (!contact(c,false)) return false;
  return bothHome(c);
}

} // namespace

int main(int argc, char ** argv) {
  rclcpp::init(argc,argv);
  auto node=rclcpp::Node::make_shared("simple_pick_place",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  // Keep the executor alive while motion APIs execute, but stop it BEFORE
  // MoveGroupInterface / TF / PlanningSceneObjects are destroyed.
  std::thread spinner([&]() {
    try {
      executor.spin();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node->get_logger(), "Executor spin exception: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(node->get_logger(), "Unknown executor spin exception");
    }
  });
  auto stopExecutor = [&]() noexcept {
    // Release persistent callback capturing node before ROS shutdown.
    validateMergedTrajectory = {};
    try {
      executor.cancel();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node->get_logger(), "Executor cancel exception: %s", e.what());
    }
    if (spinner.joinable()) {
      try {
        spinner.join();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node->get_logger(), "Executor join exception: %s", e.what());
      }
    }
  };
  // Declared after scene creation below, so its destructor runs before
  // scene, TF listener and MoveGroupInterface destructors.
  struct StopOnScopeExit {
    decltype(stopExecutor) & stop;
    ~StopOnScopeExit() noexcept { stop(); }
  };
  int exit_code=1;
  try {
    rclcpp::sleep_for(2s);
    MotionTuning tuning;
    tuning.planning_time=param<double>(node,"planning_time",15.0);
    tuning.planning_attempts=static_cast<int>(param<int64_t>(node,"ompl_attempts",10));
    tuning.retries=static_cast<int>(param<int64_t>(node,"plan_retries",3));
    tuning.movej_velocity=param<double>(node,"movej_velocity",0.5);
    tuning.movej_acceleration=param<double>(node,"movej_acceleration",0.5);
    tuning.movel_acceleration=param<double>(node,"movel_acceleration",0.5);
    tuning.ik_timeout=param<double>(node,"ik_timeout",3.0);
    if(tuning.retries<1 || tuning.planning_attempts<1) throw std::runtime_error("invalid planning attempts");
    auto lin=param<std::vector<double>>(node,"lin_velocity_scales",{0.45,0.4,0.35,0.3,0.2,0.1,0.05});
    if(lin.empty()) throw std::runtime_error("empty LIN velocity list");
    for(double v:lin) if(!std::isfinite(v) || v<=0 || v>1) throw std::runtime_error("invalid LIN scale");
    auto strategies=param<std::vector<std::string>>(node,"transit_strategies",{"moveL","moveJ"});
    for(const auto & v:strategies) if(v!="moveJ" && v!="moveL" && v!="moveC")
      throw std::runtime_error("unsupported transit strategy");
    if (!robot1Sleep(node,tuning)) throw std::runtime_error("Robot1 sleep failed");
    const std::string group3=param<std::string>(node,"robot3_group","robot3_arm");
    const std::string group4=param<std::string>(node,"robot4_group","robot4_arm");
    const std::string combined_name=param<std::string>(node,"combined_group","robot3_robot4_arms");
    const std::string link3=param<std::string>(node,"robot3_planning_link","robot3_end_effactor");
    const std::string link4=param<std::string>(node,"robot4_planning_link","robot4_end_effactor");
    moveit::planning_interface::MoveGroupInterface arm3(node,group3),arm4(node,group4),both(node,combined_name);
    arm3.setPoseReferenceFrame(kWorld); arm3.setEndEffectorLink(link3);
    arm4.setPoseReferenceFrame(kWorld); arm4.setEndEffectorLink(link4);
    tf2_ros::Buffer tf(node->get_clock()); tf.setUsingDedicatedThread(true);
    tf2_ros::TransformListener listener(tf,node,false);
    PlanningSceneObjects scene(node);
    StopOnScopeExit stop_before_moveit_destruction{stopExecutor};
    FinalAssemblyContext c;
    c.node=node; c.arm3=&arm3; c.arm4=&arm4; c.both=&both; c.scene=&scene;
    c.tuning=tuning; c.lin_scales=lin; c.strategies=strategies;
    c.object=param<std::string>(node,"object_id","rebar_1A");
    c.children=param<std::vector<std::string>>(node,"children",{});
    c.robot3_children=param<std::vector<std::string>>(node,"robot3_children",{});
    c.robot4_children=param<std::vector<std::string>>(node,"robot4_children",{});
    c.main_owner=static_cast<int>(param<int64_t>(node,"main_object_robot",3));
    if(c.main_owner!=3 && c.main_owner!=4)
      throw std::runtime_error("main_object_robot must be 3 or 4");
    std::set<std::string> expected(c.children.begin(),c.children.end());
    std::set<std::string> assigned;
    if(expected.size()!=c.children.size()) throw std::runtime_error("duplicate child in children list");
    for(const auto & id:c.robot3_children)
      if(!expected.count(id) || !assigned.insert(id).second)
        throw std::runtime_error("Unknown/duplicate robot3_children object: "+id);
    for(const auto & id:c.robot4_children)
      if(!expected.count(id) || !assigned.insert(id).second)
        throw std::runtime_error("Unknown/duplicate robot4_children object: "+id);
    if(assigned!=expected) throw std::runtime_error("Every child must have exactly one owner in YAML");
    c.touch3={link3,"robot3_J6_link","robot3_sa3_grpr01"};
    c.touch4={link4,"robot4_J6_link","robot4_sa3_grpr01_r"};
    c.a3.group=group3;c.a3.link=link3;c.a3.name="robot3";
    c.a4.group=group4;c.a4.link=link4;c.a4.name="robot4";
    RCLCPP_INFO(node->get_logger(),"Dual arm TCPs Robot3=%s Robot4=%s; main object owner and child owners from YAML",
                link3.c_str(),link4.c_str());
    // Check every merged 12-joint sample, including attached-object collision.
    validateMergedTrajectory=[node,combined_name](auto & group,const auto & plan,
      const rclcpp::Logger & logger, const std::string & stage) {
      using Validity=moveit_msgs::srv::GetStateValidity;
      auto client=node->create_client<Validity>("/check_state_validity");
      if(!client->wait_for_service(5s)) return false;
      const auto &jt=plan.trajectory_.joint_trajectory;
      if(jt.points.empty()) return false;
      for(std::size_t i=0;i<jt.points.size();++i) {
        if(jt.points[i].positions.size()!=jt.joint_names.size()) return false;
        int pieces=1;
        if(i>0) {
          double max_delta=0;
          for(size_t j=0;j<jt.joint_names.size();++j)
            max_delta=std::max(max_delta,std::abs(jt.points[i].positions[j]-jt.points[i-1].positions[j]));
          pieces=std::max(1,static_cast<int>(std::ceil(max_delta/0.02)));
        }
        for(int k=1;k<=pieces;++k) {
          const double f=static_cast<double>(k)/pieces;
          auto req=std::make_shared<Validity::Request>();
          req->group_name=combined_name;
          req->robot_state=plan.start_state_;
          auto &js=req->robot_state.joint_state;
          for(std::size_t j=0;j<jt.joint_names.size();++j) {
            auto it=std::find(js.name.begin(),js.name.end(),jt.joint_names[j]);
            if(it==js.name.end()) return false;
            const double q=i==0 ? jt.points[i].positions[j] :
              jt.points[i-1].positions[j]+f*(jt.points[i].positions[j]-jt.points[i-1].positions[j]);
            js.position[static_cast<std::size_t>(std::distance(js.name.begin(),it))]=q;
          }
          auto result=client->async_send_request(req);
          if(result.wait_for(5s)!=std::future_status::ready || !result.get()->valid) {
            RCLCPP_ERROR(logger,"[%s] combined state collision/unknown at point %zu/%zu substep %d/%d",
                         stage.c_str(),i+1,jt.points.size(),k,pieces);
            return false;
          }
        }
      }
      return true;
    };
    exit_code=runAssembly(c,tf) ? 0:1;
  } catch(const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(),"Dual-arm final assembly error: %s",e.what());
  }
  // Fallback if construction threw before StopOnScopeExit was installed.
  stopExecutor();
  rclcpp::shutdown();
  return exit_code;
}
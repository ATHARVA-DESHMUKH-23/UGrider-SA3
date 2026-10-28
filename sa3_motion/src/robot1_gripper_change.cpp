// dual_gripper_change.cpp  (sa3_motion)
//
// ONE node, both robots change gripper in lockstep through the COMBINED group
// (sa3_robots_1_2). Same sequence for both robots:
//
//   1  movej  Schunk dock approach           (combined OMPL, 12-joint goal)
//   2  movel  Schunk dock                    (separate LIN per arm, merged)
//      DETACH Schunk parts
//   3  Schunk collisions OFF -> movel retract -> collisions ON
//   4  Tying-gun collisions OFF -> movej approach -> movel dock
//      -> ATTACH tying gun -> movel retract -> collisions ON
//
// MOVEJ  : simple combined planning. Both target poses are solved by IK on one
//          RobotState and the combined group plans one 12-joint OMPL goal.
// MOVEL  : each arm plans its own Pilz LIN (robot1_arm / robot2_arm) from the same
//          start state; the two joint trajectories are time-aligned, merged into one
//          12-joint trajectory and executed through the combined group.
//
// Every motion is executed with combined.execute(); executeDirect is no longer used.
// Verify frames, object IDs and controller execution in RViz/mock first.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace std::chrono_literals;
using MGI = moveit::planning_interface::MoveGroupInterface;
using JT = trajectory_msgs::msg::JointTrajectory;
using JTP = trajectory_msgs::msg::JointTrajectoryPoint;

namespace
{
constexpr char kReferenceFrame[] = "world";
constexpr char kCombinedGroup[] = "sa3_robots_1_2";

constexpr double kApproachZ = 0.50;
constexpr double kPlanningTime = 15.0;
constexpr int kMoveJPlanRetries = 3;   // separate combined OMPL requests per MOVEJ
constexpr int kOmplAttempts = 10;
constexpr double kIkTimeout = 1.0;

constexpr double kMoveJVelocity = 0.30;
constexpr double kMoveJAcceleration = 0.20;
constexpr double kMoveLVelocity = 0.20;
constexpr double kMoveLAcceleration = 0.10;

// ---------------------------------------------------------------------------
// Per-robot configuration
// ---------------------------------------------------------------------------
struct RobotConfig
{
  std::string name;              // log prefix
  std::string group;
  std::string planning_link;
  std::string schunk_dock_frame;
  std::string gun_dock_frame;
  std::vector<std::string> schunk_ids;   // collision object IDs
  std::string gun_id;
  std::vector<std::string> arm_links;    // links that may overlap a gripper at its dock
};

RobotConfig makeConfig(const std::string & r)   // r = "robot1" | "robot2"
{
  RobotConfig c;
  c.name = r;
  c.group = r + "_arm";
  c.planning_link = r + "_tool_mount";
  c.schunk_dock_frame = r + "_shuck_dock";
  c.gun_dock_frame = r + "_tyinggun_dock";
  c.schunk_ids = {r + "_schunk_base", r + "_schunk_link7", r + "_schunk_link8"};
  c.gun_id = r + "_tying_gun";
  c.arm_links = {c.planning_link, r + "_J4_link", r + "_J5_link", r + "_J6_link"};
  return c;
}

// ---------------------------------------------------------------------------
// pose <-> tf2 helpers
// ---------------------------------------------------------------------------
tf2::Transform poseToTf(const geometry_msgs::msg::Pose & p)
{
  return tf2::Transform(
    tf2::Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
    tf2::Vector3(p.position.x, p.position.y, p.position.z));
}

geometry_msgs::msg::Pose tfToPose(const tf2::Transform & t)
{
  geometry_msgs::msg::Pose p;
  p.position.x = t.getOrigin().x();
  p.position.y = t.getOrigin().y();
  p.position.z = t.getOrigin().z();
  const auto q = t.getRotation();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

// ---------------------------------------------------------------------------
// Allowed-collision-matrix helpers
// ---------------------------------------------------------------------------
size_t acmIndex(moveit_msgs::msg::AllowedCollisionMatrix & acm, const std::string & name)
{
  for (size_t i = 0; i < acm.entry_names.size(); ++i) {
    if (acm.entry_names[i] == name) {
      return i;
    }
  }
  acm.entry_names.push_back(name);
  const size_t n = acm.entry_names.size();
  for (auto & row : acm.entry_values) {
    row.enabled.push_back(false);
  }
  moveit_msgs::msg::AllowedCollisionEntry new_row;
  new_row.enabled.assign(n, false);
  acm.entry_values.push_back(new_row);
  return n - 1;
}

void acmSetPair(
  moveit_msgs::msg::AllowedCollisionMatrix & acm,
  const std::string & a, const std::string & b, bool allowed)
{
  const size_t ia = acmIndex(acm, a);
  const size_t ib = acmIndex(acm, b);
  acm.entry_values[ia].enabled[ib] = allowed;
  acm.entry_values[ib].enabled[ia] = allowed;
}

// ---------------------------------------------------------------------------
// trajectory merge (two single-arm trajectories -> one 12-joint trajectory)
// ---------------------------------------------------------------------------
double timeOf(const JTP & p) {return rclcpp::Duration(p.time_from_start).seconds();}

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

struct Arm
{
  RobotConfig cfg;
  std::unique_ptr<MGI> mg;
  geometry_msgs::msg::Pose schunk_dock, gun_dock;
};

}  // namespace

// ===========================================================================
// Both robots' gripper change, in lockstep through the combined group
// ===========================================================================
class DualGripperChanger
{
public:
  DualGripperChanger(const rclcpp::Node::SharedPtr & node, tf2_ros::Buffer & tf)
  : node_(node), tf_(tf)
  {
    combined_ = std::make_unique<MGI>(node_, kCombinedGroup);
    combined_->setPoseReferenceFrame(kReferenceFrame);
    combined_->setPlanningTime(kPlanningTime);

    const std::array<std::string, 2> names = {"robot1", "robot2"};
    for (std::size_t i = 0; i < arms_.size(); ++i) {
      arms_[i].cfg = makeConfig(names[i]);
      arms_[i].mg = std::make_unique<MGI>(node_, arms_[i].cfg.group);
      arms_[i].mg->setPoseReferenceFrame(kReferenceFrame);
      arms_[i].mg->setEndEffectorLink(arms_[i].cfg.planning_link);
      arms_[i].mg->setPlanningTime(kPlanningTime);
    }

    get_scene_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
    apply_scene_ =
      node_->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");
  }

  bool run()
  {
    for (auto & a : arms_) {
      if (!lookupPose(a, a.cfg.schunk_dock_frame, a.schunk_dock)) {return false;}
      if (!lookupPose(a, a.cfg.gun_dock_frame, a.gun_dock)) {return false;}

      // The Schunk gripper must already be attached (by the attach node).
      if (psi_.getAttachedObjects(a.cfg.schunk_ids).size() != a.cfg.schunk_ids.size()) {
        logE(a, "Schunk parts are not all attached to " + a.cfg.planning_link +
          "; run the attach node first.");
        return false;
      }
    }

    const auto schunk_approach1 = offsetZ(arms_[0].schunk_dock, kApproachZ);
    const auto schunk_approach2 = offsetZ(arms_[1].schunk_dock, kApproachZ);
    const auto gun_approach1 = offsetZ(arms_[0].gun_dock, kApproachZ);
    const auto gun_approach2 = offsetZ(arms_[1].gun_dock, kApproachZ);

    logI("starting combined gripper change sequence.");

    if (!movejBoth(schunk_approach1, schunk_approach2, "TF1 approach")) {return false;}
    if (!movelBoth(arms_[0].schunk_dock, arms_[1].schunk_dock, "TF1 dock")) {return false;}

    for (auto & a : arms_) {
      if (!detachSchunk(a)) {return false;}
    }

    for (auto & a : arms_) {
      if (!setContacts(a, a.cfg.schunk_ids, true)) {return false;}
    }
    const bool retract_ok = movelBoth(schunk_approach1, schunk_approach2, "TF1 retract");
    bool restore_ok = true;
    for (auto & a : arms_) {                                   // always restore
      restore_ok = setContacts(a, a.cfg.schunk_ids, false) && restore_ok;
    }
    if (!retract_ok || !restore_ok) {return false;}

    for (auto & a : arms_) {
      if (!setContacts(a, {a.cfg.gun_id}, true)) {return false;}
    }
    const bool ok =
      movejBoth(gun_approach1, gun_approach2, "TF2 approach") &&
      movelBoth(arms_[0].gun_dock, arms_[1].gun_dock, "TF2 dock") &&
      attachTyingGun(arms_[0]) &&
      attachTyingGun(arms_[1]) &&
      movelBoth(gun_approach1, gun_approach2, "TF2 retract");
    bool gun_restore_ok = true;
    for (auto & a : arms_) {                                   // always restore
      gun_restore_ok = setContacts(a, {a.cfg.gun_id}, false) && gun_restore_ok;
    }
    if (!ok || !gun_restore_ok) {return false;}

    logI("combined gripper change sequence completed.");
    return true;
  }

private:
  // ---- logging ------------------------------------------------------------
  void logI(const std::string & m) {RCLCPP_INFO(node_->get_logger(), "[dual] %s", m.c_str());}
  void logW(const std::string & m) {RCLCPP_WARN(node_->get_logger(), "[dual] %s", m.c_str());}
  void logE(const std::string & m) {RCLCPP_ERROR(node_->get_logger(), "[dual] %s", m.c_str());}
  void logI(const Arm & a, const std::string & m)
  {
    RCLCPP_INFO(node_->get_logger(), "[%s] %s", a.cfg.name.c_str(), m.c_str());
  }
  void logE(const Arm & a, const std::string & m)
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] %s", a.cfg.name.c_str(), m.c_str());
  }

  // ---- scene: detach / attach ---------------------------------------------
  bool detachSchunk(Arm & a)
  {
    std::vector<moveit_msgs::msg::AttachedCollisionObject> removals;
    for (const auto & id : a.cfg.schunk_ids) {
      moveit_msgs::msg::AttachedCollisionObject r;
      r.link_name = a.cfg.planning_link;
      r.object.id = id;
      r.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      removals.push_back(r);
    }
    // REMOVE on an attached object detaches it and leaves it in the world
    // at the pose it currently has.
    psi_.applyAttachedCollisionObjects(removals);

    for (int i = 0; i < 50; ++i) {
      if (psi_.getAttachedObjects(a.cfg.schunk_ids).empty() &&
        psi_.getObjects(a.cfg.schunk_ids).size() == a.cfg.schunk_ids.size())
      {
        logI(a, "detached Schunk gripper at " + a.cfg.schunk_dock_frame + ".");
        return true;
      }
      rclcpp::sleep_for(100ms);
    }
    logE(a, "Schunk parts did not detach into the world.");
    return false;
  }

  bool attachTyingGun(Arm & a)
  {
    const auto world = psi_.getObjects({a.cfg.gun_id});
    if (world.find(a.cfg.gun_id) == world.end()) {
      logE(a, "'" + a.cfg.gun_id + "' is not in the planning scene.");
      return false;
    }

    // Keep the gun exactly where it is: express its pose in the tool-mount frame.
    const auto mount_pose = combined_->getCurrentPose(a.cfg.planning_link).pose;
    const tf2::Transform mount_inverse = poseToTf(mount_pose).inverse();

    moveit_msgs::msg::CollisionObject object = world.at(a.cfg.gun_id);
    object.pose = tfToPose(mount_inverse * poseToTf(object.pose));
    object.header.frame_id = a.cfg.planning_link;
    object.operation = moveit_msgs::msg::CollisionObject::ADD;

    moveit_msgs::msg::AttachedCollisionObject att;
    att.link_name = a.cfg.planning_link;
    att.object = object;
    att.touch_links = a.cfg.arm_links;

    psi_.removeCollisionObjects({a.cfg.gun_id});   // drop the world copy
    psi_.applyAttachedCollisionObjects({att});

    for (int i = 0; i < 50; ++i) {
      if (psi_.getAttachedObjects({a.cfg.gun_id}).size() == 1) {
        logI(a, "attached tying gun to " + a.cfg.planning_link + ".");
        return true;
      }
      rclcpp::sleep_for(100ms);
    }
    logE(a, "tying gun did not attach.");
    return false;
  }

  // ---- ACM ----------------------------------------------------------------
  std::optional<moveit_msgs::msg::AllowedCollisionMatrix> fetchAcm()
  {
    auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    req->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto fut = get_scene_->async_send_request(req);
    if (fut.wait_for(5s) != std::future_status::ready) {
      logE("get_planning_scene timed out.");
      return std::nullopt;
    }
    return fut.get()->scene.allowed_collision_matrix;
  }

  // Allow (true) or restore (false) collisions between each object and each
  // arm link of this robot, and between the objects themselves.
  bool setContacts(const Arm & a, const std::vector<std::string> & ids, bool allowed)
  {
    if (!get_scene_->wait_for_service(5s) || !apply_scene_->wait_for_service(5s)) {
      logE(a, "planning scene services not available.");
      return false;
    }

    auto acm = fetchAcm();
    if (!acm) {return false;}

    for (const auto & id : ids) {
      for (const auto & link : a.cfg.arm_links) {
        acmSetPair(*acm, id, link, allowed);
      }
      for (const auto & other : ids) {
        if (other != id) {
          acmSetPair(*acm, id, other, allowed);
        }
      }
    }

    auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    req->scene.is_diff = true;
    req->scene.allowed_collision_matrix = *acm;
    auto fut = apply_scene_->async_send_request(req);
    if (fut.wait_for(5s) != std::future_status::ready || !fut.get()->success) {
      logE(a, "apply_planning_scene failed.");
      return false;
    }
    logI(a, std::string("collisions ") + (allowed ? "ALLOWED" : "RESTORED") + " for " +
      std::to_string(ids.size()) + " object(s).");
    return true;
  }

  // ---- TF -----------------------------------------------------------------
  bool lookupPose(const Arm & a, const std::string & frame, geometry_msgs::msg::Pose & pose)
  {
    try {
      const auto t = tf_.lookupTransform(
        kReferenceFrame, frame, tf2::TimePointZero, tf2::durationFromSec(5.0));
      pose.position.x = t.transform.translation.x;
      pose.position.y = t.transform.translation.y;
      pose.position.z = t.transform.translation.z;
      pose.orientation = t.transform.rotation;
      logI(a, "TF '" + frame + "' resolved in '" + kReferenceFrame + "'.");
      return true;
    } catch (const tf2::TransformException & ex) {
      logE(a, "TF lookup failed for '" + frame + "': " + ex.what());
      return false;
    }
  }

  geometry_msgs::msg::Pose offsetZ(const geometry_msgs::msg::Pose & original, double dz) const
  {
    auto result = original;
    result.position.z += dz;   // WORLD Z
    return result;
  }

  // ---- execution ------------------------------------------------------------
  bool executeCombined(MGI::Plan & plan, const std::string & label)
  {
    const auto r = combined_->execute(plan);
    if (r != moveit::core::MoveItErrorCode::SUCCESS) {
      logE("combined execution failed for " + label + " (code " + std::to_string(r.val) + ").");
      return false;
    }
    return true;
  }

  // ---- MOVEJ: simple combined planning --------------------------------------
  // IK for each tool mount on the SAME RobotState, then one combined OMPL goal.
  bool movejBoth(
    const geometry_msgs::msg::Pose & target1, const geometry_msgs::msg::Pose & target2,
    const std::string & label)
  {
    logI("[MOVEJ combined] " + label);
    auto & mg = *combined_;

    for (int attempt = 1; attempt <= kMoveJPlanRetries; ++attempt) {
      auto state = mg.getCurrentState(10.0);
      if (!state) {
        logE("[MOVEJ] current combined robot state unavailable.");
        return false;
      }
      const auto * group1 = state->getJointModelGroup(arms_[0].cfg.group);
      const auto * group2 = state->getJointModelGroup(arms_[1].cfg.group);
      if (!group1 || !group2) {
        logE("[MOVEJ] SRDF joint model groups '" + arms_[0].cfg.group + "' / '" +
          arms_[1].cfg.group + "' unavailable.");
        return false;
      }
      const bool ik1 = state->setFromIK(group1, target1, arms_[0].cfg.planning_link, kIkTimeout);
      const bool ik2 = ik1 &&
        state->setFromIK(group2, target2, arms_[1].cfg.planning_link, kIkTimeout);
      if (!ik1 || !ik2) {
        logW("[MOVEJ] " + label + " IK attempt " + std::to_string(attempt) + "/" +
          std::to_string(kMoveJPlanRetries) + " failed (robot1=" + (ik1 ? "OK" : "FAIL") +
          ", robot2=" + (ik2 ? "OK" : "FAIL") + ").");
        continue;
      }

      mg.setStartStateToCurrentState();
      mg.setPlanningPipelineId("ompl");
      mg.setPlannerId("RRTConnectkConfigDefault");
      mg.setPlanningTime(kPlanningTime);
      mg.setNumPlanningAttempts(kOmplAttempts);
      mg.setMaxVelocityScalingFactor(kMoveJVelocity);
      mg.setMaxAccelerationScalingFactor(kMoveJAcceleration);
      mg.clearPoseTargets();
      mg.clearPathConstraints();

      if (!mg.setJointValueTarget(*state)) {
        logW("[MOVEJ] could not set combined 12-joint target for " + label + ".");
        continue;
      }

      MGI::Plan plan;
      const auto planned = mg.plan(plan);
      if (planned != moveit::core::MoveItErrorCode::SUCCESS) {
        logW("[MOVEJ] combined OMPL attempt " + std::to_string(attempt) + "/" +
          std::to_string(kMoveJPlanRetries) + " failed for " + label + " (code " +
          std::to_string(planned.val) + ").");
        continue;
      }

      if (!executeCombined(plan, label)) {return false;}
      logI("[MOVEJ combined] " + label + " complete.");
      return true;
    }

    logE("[MOVEJ] all combined planning attempts failed for " + label + ".");
    return false;
  }

  // ---- MOVEL: separate LIN per arm, merged, executed through the combined group ----
  bool movelBoth(
    const geometry_msgs::msg::Pose & target1, const geometry_msgs::msg::Pose & target2,
    const std::string & label)
  {
    logI("[MOVEL combined] " + label);

    auto start_state = combined_->getCurrentState(10.0);
    if (!start_state) {
      logE("[MOVEL] current combined robot state unavailable.");
      return false;
    }
    moveit_msgs::msg::RobotState start_msg;
    moveit::core::robotStateToRobotStateMsg(*start_state, start_msg);

    const std::array<const geometry_msgs::msg::Pose *, 2> targets = {&target1, &target2};
    std::array<MGI::Plan, 2> plans;

    // Plan both arms from the same start state; nothing moves unless both succeed.
    for (std::size_t i = 0; i < arms_.size(); ++i) {
      auto & mg = *arms_[i].mg;
      mg.setStartState(start_msg);
      mg.setPlanningPipelineId("pilz_industrial_motion_planner");
      mg.setPlannerId("LIN");
      mg.setPlanningTime(kPlanningTime);
      mg.setNumPlanningAttempts(1);
      mg.setMaxVelocityScalingFactor(kMoveLVelocity);
      mg.setMaxAccelerationScalingFactor(kMoveLAcceleration);
      mg.clearPoseTargets();
      mg.clearPathConstraints();

      if (!mg.setPoseTarget(*targets[i], arms_[i].cfg.planning_link)) {
        logE(arms_[i], "[MOVEL] could not set pose target for " + label + ".");
        mg.clearPoseTargets();
        return false;
      }
      const auto result = mg.plan(plans[i]);
      mg.clearPoseTargets();
      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        logE(arms_[i], "[MOVEL] Pilz LIN planning failed for " + label + " (code " +
          std::to_string(result.val) + "); nothing was executed.");
        return false;
      }
    }

    for (const auto & p : plans) {
      if (!p.trajectory.multi_dof_joint_trajectory.joint_names.empty()) {
        logE("[MOVEL] multi-DOF trajectories cannot be merged (" + label + ").");
        return false;
      }
    }

    MGI::Plan combined_plan;
    combined_plan.start_state = start_msg;
    combined_plan.planning_time = plans[0].planning_time + plans[1].planning_time;
    double shared = 0.0;
    if (!mergeTrajectories(
        node_->get_logger(), plans[0].trajectory.joint_trajectory,
        plans[1].trajectory.joint_trajectory, combined_plan.trajectory.joint_trajectory, shared))
    {
      return false;
    }
    logI("[MOVEL] " + label + ": both LIN plans succeeded; executing merged " +
      std::to_string(combined_plan.trajectory.joint_trajectory.joint_names.size()) +
      "-joint trajectory (" + std::to_string(shared) + " s).");

    if (!executeCombined(combined_plan, label)) {return false;}
    logI("[MOVEL combined] " + label + " complete.");
    return true;
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer & tf_;

  std::unique_ptr<MGI> combined_;
  std::array<Arm, 2> arms_;
  moveit::planning_interface::PlanningSceneInterface psi_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_;
};

// ===========================================================================
// main
// ===========================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "dual_gripper_change",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() {executor.spin();});

  bool ok = false;

  try {
    RCLCPP_INFO(node->get_logger(), "Waiting for robot states and TF.");
    rclcpp::sleep_for(2s);

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf_buffer.setUsingDedicatedThread(true);
    tf2_ros::TransformListener tf_listener(tf_buffer, node, false);

    DualGripperChanger changer(node, tf_buffer);
    ok = changer.run();

    (void)tf_listener;
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(node->get_logger(), "Exception: %s", ex.what());
  }

  RCLCPP_INFO(node->get_logger(), "RESULT: both robots %s", ok ? "OK" : "FAILED");

  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
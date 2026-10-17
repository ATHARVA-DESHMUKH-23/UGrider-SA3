// dual_gripper_change.cpp  (sa3_motion)
//
// ONE node, both robots change gripper at the same time.
// Each robot runs its own sequence in its own thread:
//
//   1  movej  Schunk dock approach
//   2  movel  Schunk dock            -> DETACH Schunk parts
//   3  Schunk collisions OFF -> movel retract -> collisions ON
//   4  Tying-gun collisions OFF -> movej approach -> movel dock
//      -> ATTACH tying gun -> movel retract -> collisions ON
//
// Motion is executed directly through sa3_motion::executeDirect()
// (sa3_motion/direct_execute.hpp), not through MoveGroup execute.

#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sa3_motion/direct_execute.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr char kReferenceFrame[] = "world";

constexpr double kApproachZ = 0.50;
constexpr double kPlanningTime = 15.0;

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
  std::string controller_action;
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
  c.controller_action = "/" + r + "_arm_controller/follow_joint_trajectory";
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

// State shared by both robots.
struct Shared
{
  rclcpp::Node::SharedPtr node;
  tf2_ros::Buffer * tf = nullptr;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene;
  // The ACM is read-modify-written as a whole, so both robots take this lock.
  std::mutex acm_mutex;
};

}  // namespace

// ===========================================================================
// One robot's gripper change
// ===========================================================================
class GripperChanger
{
public:
  GripperChanger(const RobotConfig & cfg, Shared & shared)
  : cfg_(cfg), sh_(shared), node_(shared.node)
  {
    mg_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, cfg_.group);
    mg_->setPoseReferenceFrame(kReferenceFrame);
    mg_->setEndEffectorLink(cfg_.planning_link);
    mg_->setPlanningTime(kPlanningTime);

    fjt_ = rclcpp_action::create_client<sa3_motion::FJT>(node_, cfg_.controller_action);
  }

  bool run()
  {
    if (!fjt_->wait_for_action_server(10s)) {
      logE("controller '" + cfg_.controller_action + "' not available.");
      return false;
    }

    geometry_msgs::msg::Pose tf1, tf2;
    if (!lookupPose(cfg_.schunk_dock_frame, tf1)) {return false;}
    if (!lookupPose(cfg_.gun_dock_frame, tf2)) {return false;}

    // The Schunk gripper must already be attached (by the attach node).
    if (psi_.getAttachedObjects(cfg_.schunk_ids).size() != cfg_.schunk_ids.size()) {
      logE("Schunk parts are not all attached to " + cfg_.planning_link +
        "; run the attach node first.");
      return false;
    }

    const auto tf1_approach = offsetZ(tf1, kApproachZ);
    const auto tf2_approach = offsetZ(tf2, kApproachZ);
    const std::vector<std::string> gun_ids = {cfg_.gun_id};

    logI("starting gripper change sequence.");

    if (!movej(tf1_approach, "TF1 approach")) {return false;}
    if (!movel(tf1, "TF1 dock")) {return false;}

    if (!detachSchunk()) {return false;}

    if (!setContacts(cfg_.schunk_ids, true)) {return false;}
    const bool retract_ok = movel(tf1_approach, "TF1 retract");
    const bool restore_ok = setContacts(cfg_.schunk_ids, false);   // always restore
    if (!retract_ok || !restore_ok) {return false;}

    if (!setContacts(gun_ids, true)) {return false;}
    const bool ok = movej(tf2_approach, "TF2 approach") &&
      movel(tf2, "TF2 dock") &&
      attachTyingGun() &&
      movel(tf2_approach, "TF2 retract");
    const bool gun_restore_ok = setContacts(gun_ids, false);       // always restore
    if (!ok || !gun_restore_ok) {return false;}

    logI("gripper change sequence completed.");
    return true;
  }

private:
  // ---- logging ------------------------------------------------------------
  void logI(const std::string & m) {RCLCPP_INFO(node_->get_logger(), "[%s] %s", cfg_.name.c_str(), m.c_str());}
  void logW(const std::string & m) {RCLCPP_WARN(node_->get_logger(), "[%s] %s", cfg_.name.c_str(), m.c_str());}
  void logE(const std::string & m) {RCLCPP_ERROR(node_->get_logger(), "[%s] %s", cfg_.name.c_str(), m.c_str());}

  // ---- scene: detach / attach ---------------------------------------------
  bool detachSchunk()
  {
    std::vector<moveit_msgs::msg::AttachedCollisionObject> removals;
    for (const auto & id : cfg_.schunk_ids) {
      moveit_msgs::msg::AttachedCollisionObject a;
      a.link_name = cfg_.planning_link;
      a.object.id = id;
      a.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      removals.push_back(a);
    }
    // REMOVE on an attached object detaches it and leaves it in the world
    // at the pose it currently has.
    psi_.applyAttachedCollisionObjects(removals);

    for (int i = 0; i < 50; ++i) {
      if (psi_.getAttachedObjects(cfg_.schunk_ids).empty() &&
        psi_.getObjects(cfg_.schunk_ids).size() == cfg_.schunk_ids.size())
      {
        logI("detached Schunk gripper at " + cfg_.schunk_dock_frame + ".");
        return true;
      }
      rclcpp::sleep_for(100ms);
    }
    logE("Schunk parts did not detach into the world.");
    return false;
  }

  bool attachTyingGun()
  {
    const auto world = psi_.getObjects({cfg_.gun_id});
    if (world.find(cfg_.gun_id) == world.end()) {
      logE("'" + cfg_.gun_id + "' is not in the planning scene.");
      return false;
    }

    // Keep the gun exactly where it is: express its pose in the tool-mount frame.
    const auto mount_pose = mg_->getCurrentPose(cfg_.planning_link).pose;
    const tf2::Transform mount_inverse = poseToTf(mount_pose).inverse();

    moveit_msgs::msg::CollisionObject object = world.at(cfg_.gun_id);
    object.pose = tfToPose(mount_inverse * poseToTf(object.pose));
    object.header.frame_id = cfg_.planning_link;
    object.operation = moveit_msgs::msg::CollisionObject::ADD;

    moveit_msgs::msg::AttachedCollisionObject a;
    a.link_name = cfg_.planning_link;
    a.object = object;
    a.touch_links = cfg_.arm_links;

    psi_.removeCollisionObjects({cfg_.gun_id});   // drop the world copy
    psi_.applyAttachedCollisionObjects({a});

    for (int i = 0; i < 50; ++i) {
      if (psi_.getAttachedObjects({cfg_.gun_id}).size() == 1) {
        logI("attached tying gun to " + cfg_.planning_link + ".");
        return true;
      }
      rclcpp::sleep_for(100ms);
    }
    logE("tying gun did not attach.");
    return false;
  }

  // ---- ACM ----------------------------------------------------------------
  std::optional<moveit_msgs::msg::AllowedCollisionMatrix> fetchAcm()
  {
    auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    req->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto fut = sh_.get_scene->async_send_request(req);
    if (fut.wait_for(5s) != std::future_status::ready) {
      logE("get_planning_scene timed out.");
      return std::nullopt;
    }
    return fut.get()->scene.allowed_collision_matrix;
  }

  // Allow (true) or restore (false) collisions between each object and each
  // arm link, and between the objects themselves.
  bool setContacts(const std::vector<std::string> & ids, bool allowed)
  {
    if (!sh_.get_scene->wait_for_service(5s) || !sh_.apply_scene->wait_for_service(5s)) {
      logE("planning scene services not available.");
      return false;
    }

    // Both robots edit the whole matrix: serialize so neither overwrites the other.
    std::lock_guard<std::mutex> lock(sh_.acm_mutex);

    auto acm = fetchAcm();
    if (!acm) {return false;}

    for (const auto & id : ids) {
      for (const auto & link : cfg_.arm_links) {
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
    auto fut = sh_.apply_scene->async_send_request(req);
    if (fut.wait_for(5s) != std::future_status::ready || !fut.get()->success) {
      logE("apply_planning_scene failed.");
      return false;
    }
    logI(std::string("collisions ") + (allowed ? "ALLOWED" : "RESTORED") + " for " +
      std::to_string(ids.size()) + " object(s).");
    return true;
  }

  // ---- TF / motion --------------------------------------------------------
  bool lookupPose(const std::string & frame, geometry_msgs::msg::Pose & pose)
  {
    try {
      const auto t = sh_.tf->lookupTransform(
        kReferenceFrame, frame, tf2::TimePointZero, tf2::durationFromSec(5.0));
      pose.position.x = t.transform.translation.x;
      pose.position.y = t.transform.translation.y;
      pose.position.z = t.transform.translation.z;
      pose.orientation = t.transform.rotation;
      logI("TF '" + frame + "' resolved in '" + kReferenceFrame + "'.");
      return true;
    } catch (const tf2::TransformException & ex) {
      logE("TF lookup failed for '" + frame + "': " + ex.what());
      return false;
    }
  }

  geometry_msgs::msg::Pose offsetZ(const geometry_msgs::msg::Pose & original, double dz) const
  {
    auto result = original;
    result.position.z += dz;   // WORLD Z
    return result;
  }

  bool executePlan(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const char * tag, const std::string & label)
  {
    int exec_code = 0;
    std::string exec_note;
    if (!sa3_motion::executeDirect(fjt_, plan.trajectory_.joint_trajectory, exec_code, exec_note)) {
      logE(std::string("[") + tag + "] direct execution failed for " + label +
        ": code=" + std::to_string(exec_code) + ", " + exec_note);
      return false;
    }
    return true;
  }

  bool movej(const geometry_msgs::msg::Pose & target, const std::string & label)
  {
    auto & mg = *mg_;
    logI("[MOVEJ] " + label);

    if (!mg.getCurrentState(10.0)) {
      logE("[MOVEJ] current robot state unavailable.");
      return false;
    }

    mg.setStartStateToCurrentState();
    mg.setPlanningPipelineId("ompl");
    mg.setPlannerId("RRTConnectkConfigDefault");
    mg.setPlanningTime(kPlanningTime);
    mg.setNumPlanningAttempts(10);
    mg.setMaxVelocityScalingFactor(kMoveJVelocity);
    mg.setMaxAccelerationScalingFactor(kMoveJAcceleration);
    mg.setGoalPositionTolerance(0.001);
    mg.setGoalOrientationTolerance(0.01);
    mg.clearPoseTargets();
    mg.clearPathConstraints();

    if (!mg.setPoseTarget(target, cfg_.planning_link)) {
      logE("[MOVEJ] could not set pose target.");
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto result = mg.plan(plan);
    mg.clearPoseTargets();

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      logE("[MOVEJ] planning failed for " + label + " (code " + std::to_string(result.val) + ").");
      return false;
    }
    if (!executePlan(plan, "MOVEJ", label)) {return false;}

    logI("[MOVEJ] " + label + " complete.");
    return true;
  }

  bool movel(const geometry_msgs::msg::Pose & target, const std::string & label)
  {
    auto & mg = *mg_;
    logI("[MOVEL] " + label);

    if (!mg.getCurrentState(10.0)) {
      logE("[MOVEL] current robot state unavailable.");
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

    if (!mg.setPoseTarget(target, cfg_.planning_link)) {
      logE("[MOVEL] could not set pose target.");
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto result = mg.plan(plan);
    mg.clearPoseTargets();

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      logE("[MOVEL] Pilz LIN planning failed for " + label + " (code " +
        std::to_string(result.val) + ").");
      return false;
    }
    if (!executePlan(plan, "MOVEL", label)) {return false;}

    logI("[MOVEL] " + label + " complete.");
    return true;
  }

  RobotConfig cfg_;
  Shared & sh_;
  rclcpp::Node::SharedPtr node_;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> mg_;
  moveit::planning_interface::PlanningSceneInterface psi_;
  rclcpp_action::Client<sa3_motion::FJT>::SharedPtr fjt_;
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

  // Two robots are planning/executing at once, so spin with several threads.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
  executor.add_node(node);
  std::thread executor_thread([&executor]() {executor.spin();});

  bool ok1 = false, ok2 = false;

  try {
    RCLCPP_INFO(node->get_logger(), "Waiting for robot states and TF.");
    rclcpp::sleep_for(2s);

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf_buffer.setUsingDedicatedThread(true);
    tf2_ros::TransformListener tf_listener(tf_buffer, node, false);

    Shared shared;
    shared.node = node;
    shared.tf = &tf_buffer;
    shared.get_scene =
      node->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
    shared.apply_scene =
      node->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");

    // Build both before starting either, so setup is not interleaved with motion.
    GripperChanger robot1(makeConfig("robot1"), shared);
    GripperChanger robot2(makeConfig("robot2"), shared);

    // Both sequences run simultaneously; an exception in one cannot kill the other.
    auto runSafe = [&](GripperChanger & r, bool & result, const char * name) {
        try {
          result = r.run();
        } catch (const std::exception & e) {
          RCLCPP_FATAL(node->get_logger(), "[%s] exception: %s", name, e.what());
          result = false;
        }
      };

    std::thread t1([&]() {runSafe(robot1, ok1, "robot1");});
    std::thread t2([&]() {runSafe(robot2, ok2, "robot2");});
    t1.join();
    t2.join();

    (void)tf_listener;
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(node->get_logger(), "Exception: %s", ex.what());
  }

  RCLCPP_INFO(
    node->get_logger(), "RESULT: robot1 %s | robot2 %s",
    ok1 ? "OK" : "FAILED", ok2 ? "OK" : "FAILED");

  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }

  rclcpp::shutdown();
  return (ok1 && ok2) ? 0 : 1;
}
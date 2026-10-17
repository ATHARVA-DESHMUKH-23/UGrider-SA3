#include <chrono>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace
{
constexpr char kWorld[] = "world";

// true : keep each object's pose relative to the dock, re-applied at the tool mount.
// false: keep each object's current world pose (same as the old node).
constexpr bool kSnapToDock = true;

// Everything that differs between robots is derived from the prefix ("robot1", "robot2").
struct RobotConfig
{
  std::string name;
  std::string tool_mount;
  std::string dock;
  std::vector<std::string> object_ids;
  std::vector<std::string> touch_links;
  bool done = false;
};

RobotConfig makeConfig(const std::string & prefix)
{
  RobotConfig c;
  c.name = prefix;
  c.tool_mount = prefix + "_tool_mount";
  c.dock = prefix + "_shuck_dock";
  c.object_ids = {
    prefix + "_schunk_base", prefix + "_schunk_link7", prefix + "_schunk_link8"};
  // Arm links that may overlap the gripper. Add real link names if needed.
  c.touch_links = {
    c.tool_mount, prefix + "_J4_link", prefix + "_J5_link", prefix + "_J6_link",
    prefix + "_schunk_base", prefix + "_schunk_link7", prefix + "_schunk_link8"};
  return c;
}

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

tf2::Transform transformToTf(const geometry_msgs::msg::TransformStamped & t)
{
  return tf2::Transform(
    tf2::Quaternion(
      t.transform.rotation.x, t.transform.rotation.y,
      t.transform.rotation.z, t.transform.rotation.w),
    tf2::Vector3(
      t.transform.translation.x, t.transform.translation.y, t.transform.translation.z));
}
}  // namespace

class AttachSchunk : public rclcpp::Node
{
public:
  AttachSchunk()
  : Node("attach_schunk"),
    tf_buffer_(this->get_clock())
  {
    // Which robots to handle. Override with: --ros-args -p robots:="['robot1']"
    const auto prefixes = declare_parameter<std::vector<std::string>>(
      "robots", std::vector<std::string>{"robot1", "robot2"});
    for (const auto & p : prefixes) {
      robots_.push_back(makeConfig(p));
    }

    // true = own spin thread, so TF keeps arriving while the timer callback runs.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this, true);

    timer_ = create_wall_timer(1s, std::bind(&AttachSchunk::tryAttach, this));

    for (const auto & r : robots_) {
      RCLCPP_INFO(
        get_logger(), "Schunk attach node started. %s: %s -> %s",
        r.name.c_str(), r.dock.c_str(), r.tool_mount.c_str());
    }
  }

private:
  void tryAttach()
  {
    if (busy_) {
      return;
    }
    busy_ = true;

    bool all_done = true;
    for (auto & robot : robots_) {
      if (robot.done) {
        continue;
      }
      try {
        robot.done = attachRobot(robot);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          get_logger(), "[%s] Attach attempt failed: %s", robot.name.c_str(), e.what());
      }
      all_done = all_done && robot.done;
    }

    busy_ = false;

    if (all_done) {
      RCLCPP_INFO(get_logger(), "All Schunk grippers are attached.");
      finish();
    }
  }

  // Returns true once every Schunk part of this robot is attached.
  bool attachRobot(const RobotConfig & cfg)
  {
    const char * rn = cfg.name.c_str();
    const auto world_objects = psi_.getObjects(cfg.object_ids);
    const auto attached_objects = psi_.getAttachedObjects(cfg.object_ids);

    std::unordered_set<std::string> already;
    for (const auto & entry : attached_objects) {
      already.insert(entry.first);
    }

    if (already.size() == cfg.object_ids.size()) {
      RCLCPP_INFO(get_logger(), "[%s] All Schunk parts are already attached.", rn);
      return true;
    }

    // Every part must be either attached or waiting in the world.
    for (const auto & id : cfg.object_ids) {
      if (already.count(id) == 0 && world_objects.find(id) == world_objects.end()) {
        RCLCPP_INFO(get_logger(), "[%s] Waiting for world collision object: %s", rn, id.c_str());
        return false;
      }
    }

    // Reference transform: parts are expressed relative to this frame in world.
    tf2::Transform reference_inverse;
    try {
      const std::string & ref_frame = kSnapToDock ? cfg.dock : cfg.tool_mount;
      // world -> ref_frame
      const auto t = tf_buffer_.lookupTransform(
        kWorld, ref_frame, tf2::TimePointZero, tf2::durationFromSec(1.0));
      reference_inverse = transformToTf(t).inverse();
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "[%s] Waiting for TF: %s", rn, e.what());
      return false;
    }

    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;
    scene.robot_state.is_diff = true;

    for (const auto & id : cfg.object_ids) {
      if (already.count(id) != 0) {
        continue;
      }

      const auto & source = world_objects.at(id);

      if (!source.header.frame_id.empty() && source.header.frame_id != kWorld) {
        RCLCPP_ERROR(
          get_logger(), "[%s] '%s' is in frame '%s', expected '%s'.",
          rn, id.c_str(), source.header.frame_id.c_str(), kWorld);
        return false;
      }

      moveit_msgs::msg::AttachedCollisionObject attached;
      attached.link_name = cfg.tool_mount;
      attached.touch_links = cfg.touch_links;
      attached.object = source;
      attached.object.header.frame_id = cfg.tool_mount;
      attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;

      // Only object.pose changes. mesh_poses stay relative to object.pose.
      attached.object.pose = tfToPose(reference_inverse * poseToTf(source.pose));

      scene.robot_state.attached_collision_objects.push_back(attached);

      moveit_msgs::msg::CollisionObject remove_object;
      remove_object.id = id;
      remove_object.header.frame_id = kWorld;
      remove_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      scene.world.collision_objects.push_back(remove_object);

      RCLCPP_INFO(
        get_logger(), "[%s] Preparing attachment: %s -> %s", rn, id.c_str(),
        cfg.tool_mount.c_str());
    }

    if (scene.robot_state.attached_collision_objects.empty()) {
      return false;
    }

    if (!psi_.applyPlanningScene(scene)) {
      RCLCPP_ERROR(get_logger(), "[%s] MoveIt rejected the Schunk attachment update.", rn);
      return false;
    }

    const auto result = psi_.getAttachedObjects(cfg.object_ids);
    if (result.size() == cfg.object_ids.size()) {
      RCLCPP_INFO(
        get_logger(), "[%s] SUCCESS: all Schunk parts are attached to %s.",
        rn, cfg.tool_mount.c_str());
      return true;
    }

    RCLCPP_WARN(
      get_logger(), "[%s] Attachment not yet visible (%zu/%zu); retrying.",
      rn, result.size(), cfg.object_ids.size());
    return false;
  }

  void finish()
  {
    timer_->cancel();
    // Let the process exit so it can be used as a one-shot step in a launch file.
    rclcpp::shutdown();
  }

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  moveit::planning_interface::PlanningSceneInterface psi_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<RobotConfig> robots_;
  bool busy_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AttachSchunk>());
  rclcpp::shutdown();
  return 0;
}
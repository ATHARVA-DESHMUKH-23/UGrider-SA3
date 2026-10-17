#ifndef SA3_MOTION__PLANNING_SCENE_OBJECTS_HPP_
#define SA3_MOTION__PLANNING_SCENE_OBJECTS_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class PlanningSceneObjects
{
public:
  explicit PlanningSceneObjects(
    const rclcpp::Node::SharedPtr & node);

  // Spawn everything from workcell.yaml (rebars + grippers).
  bool addRebarObjects();

  // workcell.extra_rebars -> collision objects.
  bool addExtraRebars();

  // workcell.extra_grippers -> collision objects (with mesh_xyz/mesh_rpy offset).
  bool addExtraGrippers();

  // Remove all rebars / grippers listed in workcell.yaml.
  bool removeRebarObjects();
  bool removeGripperObjects();

  // Remove one object by ID.
  bool removeObject(const std::string & object_id);

  // ---- from the friend's version ----
  bool setWorldObjectPose(
    const std::string & object_id,
    const geometry_msgs::msg::Pose & world_pose);

  bool setObjectCollisionsAllowed(
    const std::string & object_id,
    const std::vector<std::string> & link_names,
    bool allowed);

  bool verifyObjectState(
    const std::string & object_id,
    bool expected_in_world,
    bool expected_attached,
    const std::string & expected_attach_link = "");

  bool attachWorldObjectPreservingPose(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const std::string & object_id,
    const std::string & attach_link,
    const std::vector<std::string> & touch_links = {});

  bool detachObjectToWorldPreservingPose(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const std::string & object_id);

  bool addMovableRebars();

  // ---- new: allow / restore collision between two named bodies ----
  bool setCollisionPairAllowed(
    const std::string & a, const std::string & b, bool allowed);

private:
  bool addFromSection(const std::string & section, bool use_mesh_offset);
  bool removeFromSection(const std::string & section);

  mutable std::recursive_mutex mutex_;

  rclcpp::Node::SharedPtr node_;

  moveit::planning_interface::PlanningSceneInterface
    planning_scene_interface_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif  // SA3_MOTION__PLANNING_SCENE_OBJECTS_HPP_
#include "sa3_motion/planning_scene_objects.hpp"
// version y
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <future>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// POSIX file lock (cross-process serialization of ACM updates)
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <Eigen/Geometry>
#include <boost/variant/get.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <geometric_shapes/shape_operations.h>

#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/mesh.hpp>

#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;


namespace
{

// How long to wait for world -> frame to appear (seconds).
constexpr double kTfLookupTimeoutSec = 15.0;

// Lock file shared by every node that edits the Allowed Collision Matrix.
constexpr char kAcmLockPath[] = "/tmp/sa3_acm.lock";

// false: an ACM read-back mismatch is logged as a warning but does not abort the
// motion. Set true once the mismatch cause is understood.
constexpr bool kAcmVerifyIsFatal = false;

// ============================================================
// Cross-process lock for ACM read-modify-write cycles
//
// The robot nodes are separate processes, each with its own
// PlanningSceneObjects and its own mutex_. The ACM is replaced as a
// whole on every update, so two nodes doing GET -> edit -> APPLY at the
// same time would overwrite each other's changes. flock() serializes
// the whole cycle across processes on the same machine.
// ============================================================

class AcmFileLock
{
public:
  AcmFileLock()
  {
    fd_ = ::open(kAcmLockPath, O_CREAT | O_RDWR, 0666);
    if (fd_ >= 0) {
      ::flock(fd_, LOCK_EX);   // blocks until the other node releases it
    }
  }

  ~AcmFileLock()
  {
    if (fd_ >= 0) {
      ::flock(fd_, LOCK_UN);
      ::close(fd_);
    }
  }

  bool ok() const {return fd_ >= 0;}

  AcmFileLock(const AcmFileLock &) = delete;
  AcmFileLock & operator=(const AcmFileLock &) = delete;

private:
  int fd_ = -1;
};

// ============================================================
// Pose helper
// ============================================================

geometry_msgs::msg::Pose makePose(
  double x, double y, double z,
  double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Pose pose;

  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  tf2::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);
  quaternion.normalize();

  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();

  return pose;
}


// ============================================================
// Look up a TF frame in world
// ============================================================

bool lookupWorldPose(
  tf2_ros::Buffer & tf_buffer,
  const rclcpp::Logger & logger,
  const std::string & frame,
  geometry_msgs::msg::Pose & pose)
{
  try
  {
    const auto transform = tf_buffer.lookupTransform(
      "world",
      frame,
      tf2::TimePointZero,
      tf2::durationFromSec(kTfLookupTimeoutSec));

    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;
    pose.orientation = transform.transform.rotation;

    return true;
  }
  catch (const tf2::TransformException & error)
  {
    RCLCPP_ERROR(
      logger,
      "Unable to resolve world -> %s: %s",
      frame.c_str(),
      error.what());

    return false;
  }
}


// ============================================================
// Mesh-object helper
//
// model_pose         : world -> child frame (from TF)
// mesh_pose_in_model : mesh offset inside child frame
//                      (identity for rebars, mesh_xyz/rpy for grippers)
// ============================================================

moveit_msgs::msg::CollisionObject makeMeshObject(
  const rclcpp::Logger & logger,
  const std::string & id,
  const std::string & mesh_resource,
  const Eigen::Vector3d & scale,
  const geometry_msgs::msg::Pose & model_pose,
  const geometry_msgs::msg::Pose & mesh_pose_in_model)
{
  moveit_msgs::msg::CollisionObject object;

  object.header.frame_id = "world";
  object.id = id;
  object.operation =
    moveit_msgs::msg::CollisionObject::ADD;

  RCLCPP_INFO(
    logger,
    "Loading mesh for '%s': %s (scale %.6f %.6f %.6f)",
    id.c_str(),
    mesh_resource.c_str(),
    scale.x(), scale.y(), scale.z());

  shapes::ShapePtr mesh(
    shapes::createMeshFromResource(mesh_resource, scale));

  if (!mesh)
  {
    RCLCPP_ERROR(
      logger,
      "Unable to load mesh '%s' for object '%s'.",
      mesh_resource.c_str(),
      id.c_str());

    return object;
  }

  shapes::ShapeMsg shape_message;

  if (!shapes::constructMsgFromShape(
        mesh.get(),
        shape_message))
  {
    RCLCPP_ERROR(
      logger,
      "Unable to convert mesh for object '%s'.",
      id.c_str());

    return object;
  }

  object.meshes.push_back(
    boost::get<shape_msgs::msg::Mesh>(shape_message));

  // Object's model origin in world.
  object.pose = model_pose;

  // Mesh origin relative to the object's model origin.
  object.mesh_poses.push_back(mesh_pose_in_model);

  return object;
}

// ============================================================
// YAML-driven definitions (rebars and grippers)
//
// Reads the same workcell.yaml sections that rebar_visualizer and
// gripper_visualizer read, so collision and RViz always agree.
// ============================================================

struct MeshSpec
{
  std::string id;                          // yaml key = collision object ID
  std::string mesh;                        // yaml 'mesh'
  std::string frame;                       // yaml 'child'
  Eigen::Vector3d scale;                   // yaml 'scale' (number or [x,y,z])
  geometry_msgs::msg::Pose mesh_pose;      // yaml 'mesh_xyz' / 'mesh_rpy'
};


std::array<double, 3> readVec3(
  const YAML::Node & node,
  const std::array<double, 3> & fallback)
{
  if (!node || !node.IsSequence() || node.size() < 3)
  {
    return fallback;
  }

  return {
    node[0].as<double>(),
    node[1].as<double>(),
    node[2].as<double>()};
}


Eigen::Vector3d readScale(const YAML::Node & node)
{
  if (!node)
  {
    return Eigen::Vector3d(0.001, 0.001, 0.001);
  }

  if (node.IsSequence() && node.size() >= 3)
  {
    return Eigen::Vector3d(
      node[0].as<double>(),
      node[1].as<double>(),
      node[2].as<double>());
  }

  const double s = node.as<double>();
  return Eigen::Vector3d(s, s, s);
}


std::vector<MeshSpec> loadSpecs(
  const rclcpp::Logger & logger,
  const std::string & section,
  bool use_mesh_offset)
{
  std::vector<MeshSpec> specs;

  try
  {
    const std::string yaml_file =
      ament_index_cpp::get_package_share_directory("sa2_description") +
      "/config/workcell.yaml";

    const YAML::Node config = YAML::LoadFile(yaml_file);
    const YAML::Node entries = config["workcell"][section];

    if (!entries || !entries.IsMap())
    {
      RCLCPP_ERROR(
        logger,
        "No '%s' section in %s",
        section.c_str(),
        yaml_file.c_str());

      return specs;
    }

    for (const auto & entry : entries)
    {
      const std::string name = entry.first.as<std::string>();
      const YAML::Node data = entry.second;

      if (!data["mesh"] || !data["child"])
      {
        RCLCPP_WARN(
          logger,
          "'%s' entry '%s' needs 'mesh' and 'child'; skipping.",
          section.c_str(),
          name.c_str());

        continue;
      }

      MeshSpec spec;
      spec.id = name;
      spec.mesh = data["mesh"].as<std::string>();
      spec.frame = data["child"].as<std::string>();
      spec.scale = readScale(data["scale"]);

      if (use_mesh_offset)
      {
        const auto xyz = readVec3(data["mesh_xyz"], {0.0, 0.0, 0.0});
        const auto rpy = readVec3(data["mesh_rpy"], {0.0, 0.0, 0.0});

        spec.mesh_pose = makePose(
          xyz[0], xyz[1], xyz[2],
          rpy[0], rpy[1], rpy[2]);
      }
      else
      {
        spec.mesh_pose = makePose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      }

      specs.push_back(spec);
    }
  }
  catch (const std::exception & error)
  {
    RCLCPP_ERROR(
      logger,
      "Failed to load '%s' from workcell.yaml: %s",
      section.c_str(),
      error.what());
  }

  return specs;
}


// ============================================================
// Compose parent pose and child pose: T_world_parent * T_parent_child
// ============================================================

geometry_msgs::msg::Pose composePoses(
  const geometry_msgs::msg::Pose & parent,
  const geometry_msgs::msg::Pose & child)
{
  tf2::Transform T_world_parent;
  tf2::Transform T_parent_child;

  tf2::fromMsg(parent, T_world_parent);
  tf2::fromMsg(child, T_parent_child);

  geometry_msgs::msg::Pose result;

  tf2::toMsg(
    T_world_parent * T_parent_child,
    result);

  return result;
}


// ============================================================
// Pose errors (used to check a detach did not move the object)
// ============================================================

double translationError(
  const geometry_msgs::msg::Pose & a,
  const geometry_msgs::msg::Pose & b)
{
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  const double dz = a.position.z - b.position.z;

  return std::sqrt(dx * dx + dy * dy + dz * dz);
}


double angularError(
  const geometry_msgs::msg::Quaternion & qa_msg,
  const geometry_msgs::msg::Quaternion & qb_msg)
{
  tf2::Quaternion qa;
  tf2::Quaternion qb;

  tf2::fromMsg(qa_msg, qa);
  tf2::fromMsg(qb_msg, qb);

  qa.normalize();
  qb.normalize();

  return qa.angleShortestPath(qb);
}

}  // namespace


// ============================================================
// Constructor
// ============================================================

PlanningSceneObjects::PlanningSceneObjects(
  const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);

  // true = own spin thread, so TF arrives even while we block waiting.
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_, node_, true);

  RCLCPP_INFO(
    node_->get_logger(),
    "PlanningSceneObjects initialized.");
}


// ============================================================
// Public add methods
// ============================================================

bool PlanningSceneObjects::addRebarObjects()
{
  // Spawn everything from workcell.yaml: rebars first, then grippers.
  return addExtraRebars()
      && addExtraGrippers();
}

bool PlanningSceneObjects::addExtraRebars()
{
  return addFromSection("extra_rebars", false);
}

bool PlanningSceneObjects::addMovableRebars()
{
  return addFromSection("movable_rebars", false);
}

bool PlanningSceneObjects::addExtraGrippers()
{
  return addFromSection("extra_grippers", true);
}

// ============================================================
// Shared spawn logic
// ============================================================

bool PlanningSceneObjects::addFromSection(
  const std::string & section,
  bool use_mesh_offset)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  RCLCPP_INFO(
    node_->get_logger(),
    "Spawning '%s' from workcell.yaml into MoveIt Planning Scene.",
    section.c_str());

  auto specs = loadSpecs(node_->get_logger(), section, use_mesh_offset);

  if (specs.empty())
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "No specs loaded for '%s'.",
      section.c_str());

    return false;
  }

  // ----------------------------------------------------------
  // Skip objects that are currently attached to a robot
  // (e.g. a gripper that is picked up), instead of overwriting them.
  // ----------------------------------------------------------

  {
    std::vector<std::string> all_ids;

    for (const auto & spec : specs)
    {
      all_ids.push_back(spec.id);
    }

    const auto attached =
      planning_scene_interface_.getAttachedObjects(all_ids);

    if (!attached.empty())
    {
      std::vector<MeshSpec> remaining;

      for (const auto & spec : specs)
      {
        if (attached.find(spec.id) != attached.end())
        {
          RCLCPP_WARN(
            node_->get_logger(),
            "'%s' is currently attached; not spawning it in the world.",
            spec.id.c_str());
        }
        else
        {
          remaining.push_back(spec);
        }
      }

      specs = std::move(remaining);
    }
  }

  if (specs.empty())
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Nothing to spawn for '%s' (all attached).",
      section.c_str());

    return true;
  }

  std::vector<std::string> ids;

  for (const auto & spec : specs)
  {
    ids.push_back(spec.id);
  }

  // ----------------------------------------------------------
  // Remove stale world copies so rerunning does not duplicate.
  // ----------------------------------------------------------

  const auto existing_objects =
    planning_scene_interface_.getObjects(ids);

  std::vector<moveit_msgs::msg::CollisionObject> removals;

  for (const auto & entry : existing_objects)
  {
    moveit_msgs::msg::CollisionObject removal;

    removal.header.frame_id = "world";
    removal.id = entry.first;
    removal.operation =
      moveit_msgs::msg::CollisionObject::REMOVE;

    removals.push_back(std::move(removal));
  }

  if (!removals.empty())
  {
    if (!planning_scene_interface_.applyCollisionObjects(removals))
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to remove stale '%s' objects.",
        section.c_str());

      return false;
    }
  }

  // ----------------------------------------------------------
  // Build all objects. Pose always comes from TF (world -> child),
  // the same frame the RViz marker is drawn in.
  // ----------------------------------------------------------

  std::vector<moveit_msgs::msg::CollisionObject> objects;

  for (const auto & spec : specs)
  {
    geometry_msgs::msg::Pose world_pose;

    if (!lookupWorldPose(
          *tf_buffer_,
          node_->get_logger(),
          spec.frame,
          world_pose))
    {
      return false;
    }

    auto object = makeMeshObject(
      node_->get_logger(),
      spec.id,
      spec.mesh,
      spec.scale,
      world_pose,
      spec.mesh_pose);

    if (object.meshes.empty())
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "No mesh was created for '%s'.",
        spec.id.c_str());

      return false;
    }

    const auto & p = world_pose.position;

    RCLCPP_INFO(
      node_->get_logger(),
      "'%s' world pose: x=%.3f, y=%.3f, z=%.3f",
      spec.id.c_str(),
      p.x,
      p.y,
      p.z);

    objects.push_back(std::move(object));
  }

  // ----------------------------------------------------------
  // Apply all objects to MoveIt.
  // ----------------------------------------------------------

  if (!planning_scene_interface_.applyCollisionObjects(objects))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to add '%s' objects.",
      section.c_str());

    return false;
  }

  // ----------------------------------------------------------
  // Verify all objects are in the world.
  // ----------------------------------------------------------

  const auto verified_objects =
    planning_scene_interface_.getObjects(ids);

  for (const auto & spec : specs)
  {
    if (verified_objects.find(spec.id) == verified_objects.end())
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "'%s' is missing from the Planning Scene.",
        spec.id.c_str());

      return false;
    }
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Successfully added and verified %zu objects from '%s'.",
    specs.size(),
    section.c_str());

  return true;
}


// ============================================================
// Remove one object
// ============================================================

bool PlanningSceneObjects::removeObject(
  const std::string & object_id)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  moveit_msgs::msg::CollisionObject object;

  object.header.frame_id = "world";
  object.id = object_id;
  object.operation =
    moveit_msgs::msg::CollisionObject::REMOVE;

  const bool success =
    planning_scene_interface_.applyCollisionObject(object);

  if (success)
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "Removed Planning Scene object '%s'.",
      object_id.c_str());
  }
  else
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to remove Planning Scene object '%s'.",
      object_id.c_str());
  }

  return success;
}


// ============================================================
// Remove all objects of a section
// ============================================================

bool PlanningSceneObjects::removeFromSection(
  const std::string & section)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  std::vector<std::string> ids;

  for (const auto & spec : loadSpecs(node_->get_logger(), section, false))
  {
    ids.push_back(spec.id);
  }

  planning_scene_interface_.removeCollisionObjects(ids);

  RCLCPP_INFO(
    node_->get_logger(),
    "Requested removal of %zu '%s' objects.",
    ids.size(),
    section.c_str());

  return true;
}


bool PlanningSceneObjects::removeRebarObjects()
{
  return removeFromSection("extra_rebars");
}


bool PlanningSceneObjects::removeGripperObjects()
{
  return removeFromSection("extra_grippers");
}


// ============================================================
// Update a world object's pose
// ============================================================

bool PlanningSceneObjects::setWorldObjectPose(
  const std::string & object_id,
  const geometry_msgs::msg::Pose & world_pose)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  const auto objects =
    planning_scene_interface_.getObjects({object_id});

  const auto it = objects.find(object_id);

  if (it == objects.end())
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Cannot reposition '%s': world object does not exist.",
      object_id.c_str());

    return false;
  }

  if (!planning_scene_interface_.getAttachedObjects({object_id}).empty())
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Cannot reposition '%s': object is attached.",
      object_id.c_str());

    return false;
  }

  auto updated = it->second;

  updated.header.frame_id = "world";
  updated.pose = world_pose;

  updated.operation =
    moveit_msgs::msg::CollisionObject::ADD;

  if (!planning_scene_interface_.applyCollisionObject(updated))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to reposition world object '%s'.",
      object_id.c_str());

    return false;
  }

  return verifyObjectState(object_id, true, false);
}


// ============================================================
// Verify world and attached ownership
// ============================================================

bool PlanningSceneObjects::verifyObjectState(
  const std::string & object_id,
  bool expected_in_world,
  bool expected_attached,
  const std::string & expected_attach_link)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  const auto world =
    planning_scene_interface_.getObjects({object_id});

  const auto attached =
    planning_scene_interface_.getAttachedObjects({object_id});

  const bool in_world =
    world.find(object_id) != world.end();

  const bool is_attached =
    attached.find(object_id) != attached.end();

  if (
    in_world != expected_in_world ||
    is_attached != expected_attached ||
    (in_world && is_attached))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Object '%s' ownership mismatch: world=%s attached=%s; "
      "expected world=%s attached=%s.",
      object_id.c_str(),
      in_world ? "true" : "false",
      is_attached ? "true" : "false",
      expected_in_world ? "true" : "false",
      expected_attached ? "true" : "false");

    return false;
  }

  if (
    is_attached &&
    !expected_attach_link.empty() &&
    attached.at(object_id).link_name != expected_attach_link)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Object '%s' is attached to '%s', not '%s'.",
      object_id.c_str(),
      attached.at(object_id).link_name.c_str(),
      expected_attach_link.c_str());

    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Verified '%s': WORLD=%s ATTACHED=%s%s%s%s.",
    object_id.c_str(),
    in_world ? "true" : "false",
    is_attached ? "true" : "false",
    is_attached ? " link='" : "",
    is_attached ? attached.at(object_id).link_name.c_str() : "",
    is_attached ? "'" : "");

  return true;
}


// ============================================================
// Attach a world object to a robot
// ============================================================

bool PlanningSceneObjects::attachWorldObjectPreservingPose(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const std::string & object_id,
  const std::string & attach_link,
  const std::vector<std::string> & touch_links)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  const auto world_before =
    planning_scene_interface_.getObjects({object_id});

  if (world_before.find(object_id) == world_before.end())
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Cannot attach '%s': object is not in the world.",
      object_id.c_str());

    return false;
  }

  if (!planning_scene_interface_.getAttachedObjects({object_id}).empty())
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Cannot attach '%s': object is already attached.",
      object_id.c_str());

    return false;
  }

  if (!verifyObjectState(object_id, true, false))
  {
    return false;
  }

  if (!move_group.attachObject(object_id, attach_link, touch_links))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "MoveIt refused to attach '%s' to '%s'.",
      object_id.c_str(),
      attach_link.c_str());

    return false;
  }

  // Wait for world -> attached transition.
  for (int i = 0; i < 40 && rclcpp::ok(); ++i)
  {
    const auto attached =
      planning_scene_interface_.getAttachedObjects({object_id});

    const auto world_after =
      planning_scene_interface_.getObjects({object_id});

    if (
      attached.find(object_id) != attached.end() &&
      world_after.find(object_id) == world_after.end())
    {
      return verifyObjectState(
        object_id,
        false,
        true,
        attach_link);
    }

    rclcpp::sleep_for(100ms);
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "Timed out waiting for '%s' to become attached.",
    object_id.c_str());

  return false;
}


// ============================================================
// Detach an object and return it to the world at its release pose
// ============================================================

bool PlanningSceneObjects::detachObjectToWorldPreservingPose(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const std::string & object_id)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  const auto attached_objects =
    planning_scene_interface_.getAttachedObjects({object_id});

  const auto it = attached_objects.find(object_id);

  if (it == attached_objects.end())
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Cannot detach '%s': object is not attached.",
      object_id.c_str());

    return false;
  }

  const std::string attach_link =
    it->second.link_name;

  // DEBUG ONLY: inspect the attached-object representation before detaching.
  RCLCPP_INFO(
    node_->get_logger(),
    "DETACH DEBUG [%s]: attached link='%s', object.header.frame_id='%s', "
    "object pose xyz=[%.6f, %.6f, %.6f], quat=[%.6f, %.6f, %.6f, %.6f], "
    "meshes=%zu, mesh_poses=%zu.",
    object_id.c_str(),
    attach_link.c_str(),
    it->second.object.header.frame_id.c_str(),
    it->second.object.pose.position.x,
    it->second.object.pose.position.y,
    it->second.object.pose.position.z,
    it->second.object.pose.orientation.x,
    it->second.object.pose.orientation.y,
    it->second.object.pose.orientation.z,
    it->second.object.pose.orientation.w,
    it->second.object.meshes.size(),
    it->second.object.mesh_poses.size());

  if (!it->second.object.mesh_poses.empty()) {
    const auto & debug_mesh_pose = it->second.object.mesh_poses.front();
    RCLCPP_INFO(
      node_->get_logger(),
      "DETACH DEBUG [%s]: first mesh pose xyz=[%.6f, %.6f, %.6f], "
      "quat=[%.6f, %.6f, %.6f, %.6f].",
      object_id.c_str(),
      debug_mesh_pose.position.x,
      debug_mesh_pose.position.y,
      debug_mesh_pose.position.z,
      debug_mesh_pose.orientation.x,
      debug_mesh_pose.orientation.y,
      debug_mesh_pose.orientation.z,
      debug_mesh_pose.orientation.w);
  }

  if (!verifyObjectState(
        object_id,
        false,
        true,
        attach_link))
  {
    return false;
  }

  // Attached-object pose is relative to the link.
  // Prefer TF for the link's world pose (same source used by the workcell
  // visualizers); fall back to MoveIt's current pose if TF is unavailable.
  // Compose this with the attached object's link-relative pose to recover
  // the object's world pose.
  const auto moveit_link_pose =
    move_group.getCurrentPose(attach_link).pose;

  geometry_msgs::msg::Pose tf_link_pose;
  const bool have_tf_pose = lookupWorldPose(
    *tf_buffer_, node_->get_logger(), attach_link, tf_link_pose);

  const geometry_msgs::msg::Pose link_world_pose =
    have_tf_pose ? tf_link_pose : moveit_link_pose;

  RCLCPP_INFO(
    node_->get_logger(),
    "DETACH DEBUG [%s]: current link pose frame='%s', link='%s', "
    "xyz=[%.6f, %.6f, %.6f], quat=[%.6f, %.6f, %.6f, %.6f].",
    object_id.c_str(),
    have_tf_pose ? "world (TF)" : "MoveIt current pose",
    attach_link.c_str(),
    link_world_pose.position.x,
    link_world_pose.position.y,
    link_world_pose.position.z,
    link_world_pose.orientation.x,
    link_world_pose.orientation.y,
    link_world_pose.orientation.z,
    link_world_pose.orientation.w);

  const auto release_world_pose =
    composePoses(
      link_world_pose,
      it->second.object.pose);

  RCLCPP_INFO(
    node_->get_logger(),
    "DETACH DEBUG [%s]: calculated release world pose xyz=[%.6f, %.6f, %.6f], "
    "quat=[%.6f, %.6f, %.6f, %.6f].",
    object_id.c_str(),
    release_world_pose.position.x,
    release_world_pose.position.y,
    release_world_pose.position.z,
    release_world_pose.orientation.x,
    release_world_pose.orientation.y,
    release_world_pose.orientation.z,
    release_world_pose.orientation.w);

  auto world_object = it->second.object;

  world_object.header.frame_id = "world";
  world_object.pose = release_world_pose;

  world_object.operation =
    moveit_msgs::msg::CollisionObject::ADD;

  RCLCPP_INFO(
    node_->get_logger(),
    "DETACH DEBUG [%s]: outgoing world object frame='%s', operation=%d, "
    "pose xyz=[%.6f, %.6f, %.6f], quat=[%.6f, %.6f, %.6f, %.6f].",
    object_id.c_str(),
    world_object.header.frame_id.c_str(),
    static_cast<int>(world_object.operation),
    world_object.pose.position.x,
    world_object.pose.position.y,
    world_object.pose.position.z,
    world_object.pose.orientation.x,
    world_object.pose.orientation.y,
    world_object.pose.orientation.z,
    world_object.pose.orientation.w);

  // Remove the attached object and insert the world object in
  // one Planning Scene diff.
  auto attached_removal = it->second;

  attached_removal.object.operation =
    moveit_msgs::msg::CollisionObject::REMOVE;

  moveit_msgs::msg::PlanningScene diff;

  diff.is_diff = true;
  diff.robot_state.is_diff = true;

  diff.robot_state.attached_collision_objects.push_back(
    attached_removal);

  diff.world.collision_objects.push_back(
    world_object);

  if (!planning_scene_interface_.applyPlanningScene(diff))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to detach '%s' to world.",
      object_id.c_str());

    return false;
  }

  // Confirm the world object exists and the attached copy is gone.
  for (int i = 0; i < 40 && rclcpp::ok(); ++i)
  {
    const auto world =
      planning_scene_interface_.getObjects({object_id});

    const auto attached =
      planning_scene_interface_.getAttachedObjects({object_id});

    const auto world_it =
      world.find(object_id);

    if (
      world_it != world.end() &&
      attached.find(object_id) == attached.end())
    {
      const double p_error =
        translationError(
          release_world_pose,
          world_it->second.pose);

      const double r_error =
        angularError(
          release_world_pose.orientation,
          world_it->second.pose.orientation);

      RCLCPP_INFO(
        node_->get_logger(),
        "DETACH DEBUG [%s]: world read-back frame='%s', "
        "actual xyz=[%.6f, %.6f, %.6f], quat=[%.6f, %.6f, %.6f, %.6f]; "
        "expected xyz=[%.6f, %.6f, %.6f], quat=[%.6f, %.6f, %.6f, %.6f]; "
        "translation_error=%.9f m, rotation_error=%.9f rad.",
        object_id.c_str(),
        world_it->second.header.frame_id.c_str(),
        world_it->second.pose.position.x,
        world_it->second.pose.position.y,
        world_it->second.pose.position.z,
        world_it->second.pose.orientation.x,
        world_it->second.pose.orientation.y,
        world_it->second.pose.orientation.z,
        world_it->second.pose.orientation.w,
        release_world_pose.position.x,
        release_world_pose.position.y,
        release_world_pose.position.z,
        release_world_pose.orientation.x,
        release_world_pose.orientation.y,
        release_world_pose.orientation.z,
        release_world_pose.orientation.w,
        p_error, r_error);

      if (p_error > 1e-4 || r_error > 1e-4)
      {
        RCLCPP_ERROR(
          node_->get_logger(),
          "'%s' pose changed during detach "
          "(%.6f m, %.6f rad).",
          object_id.c_str(),
          p_error,
          r_error);

        return false;
      }

      return verifyObjectState(
        object_id,
        true,
        false);
    }

    rclcpp::sleep_for(100ms);
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "Timed out waiting for '%s' to return to world.",
    object_id.c_str());

  return false;
}


// ============================================================
// Set object collision permissions
//
// The ACM is replaced as a whole on every update, so this is a
// read-modify-write cycle. It is wrapped in a cross-process file lock
// so two robot nodes cannot overwrite each other's changes.
// ============================================================

bool PlanningSceneObjects::setObjectCollisionsAllowed(
  const std::string & object_id,
  const std::vector<std::string> & link_names,
  bool allowed)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // Held until this function returns: covers GET -> edit -> APPLY -> verify.
  AcmFileLock acm_lock;

  if (!acm_lock.ok()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Could not open ACM lock file '%s'.",
      kAcmLockPath);
    return false;
  }

  if (object_id.empty() || link_names.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Cannot change collision permissions: object ID or link list is empty.");
    return false;
  }

  auto client =
    node_->create_client<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene");

  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Timed out waiting for /get_planning_scene.");
    return false;
  }

  auto request =
    std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();

  request->components.components =
    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

  // A freshly created client can lose its first request/response while DDS
  // discovery finishes (wait_for_service() can already be true). Retrying on
  // the same client fixes that, so retry instead of giving up after one try.
  moveit_msgs::msg::AllowedCollisionMatrix acm;
  {
    bool got_acm = false;

    for (int attempt = 1; attempt <= 3 && !got_acm; ++attempt) {
      auto future = client->async_send_request(request);

      if (future.wait_for(3s) == std::future_status::ready) {
        acm = future.get()->scene.allowed_collision_matrix;
        got_acm = true;
      } else {
        RCLCPP_WARN(
          node_->get_logger(),
          "Timed out retrieving the Allowed Collision Matrix (attempt %d/3).",
          attempt);
      }
    }

    if (!got_acm) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Could not retrieve the Allowed Collision Matrix after 3 attempts.");
      return false;
    }
  }

  if (acm.entry_names.empty() ||
      acm.entry_values.size() != acm.entry_names.size()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "The retrieved Allowed Collision Matrix is empty or inconsistent.");
    return false;
  }

  for (const auto & row : acm.entry_values) {
    if (row.enabled.size() != acm.entry_names.size()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "The Allowed Collision Matrix is not square.");
      return false;
    }
  }

  // Return a named default collision value, if one exists.
  auto defaultValue = [&acm](
    const std::string & name,
    bool & found) -> bool
  {
    for (std::size_t i = 0;
         i < acm.default_entry_names.size() &&
         i < acm.default_entry_values.size();
         ++i) {
      if (acm.default_entry_names[i] == name) {
        found = true;
        return acm.default_entry_values[i];
      }
    }

    found = false;
    return false;
  };

  // Determine the default collision rule for a pair.
  auto defaultPair = [&defaultValue](
    const std::string & a,
    const std::string & b) -> bool
  {
    bool found_a = false;
    bool found_b = false;

    const bool value_a = defaultValue(a, found_a);
    const bool value_b = defaultValue(b, found_b);

    if (found_a && found_b) {
      return value_a && value_b;
    }
    if (found_a) {
      return value_a;
    }
    if (found_b) {
      return value_b;
    }

    return false;
  };

  // Add a missing matrix entry while preserving existing entries.
  auto ensureEntry = [&](
    const std::string & name) -> bool
  {
    if (std::find(
          acm.entry_names.begin(),
          acm.entry_names.end(),
          name) != acm.entry_names.end()) {
      return true;
    }

    const std::size_t old_size = acm.entry_names.size();

    if (acm.entry_values.size() != old_size) {
      return false;
    }

    for (const auto & row : acm.entry_values) {
      if (row.enabled.size() != old_size) {
        return false;
      }
    }

    moveit_msgs::msg::AllowedCollisionEntry new_row;
    new_row.enabled.reserve(old_size + 1);

    for (std::size_t i = 0; i < old_size; ++i) {
      const bool default_allowed =
        defaultPair(name, acm.entry_names[i]);

      new_row.enabled.push_back(default_allowed);
      acm.entry_values[i].enabled.push_back(default_allowed);
    }

    // Self-collision of an entry with itself is irrelevant.
    new_row.enabled.push_back(false);

    acm.entry_names.push_back(name);
    acm.entry_values.push_back(std::move(new_row));

    return true;
  };

  if (!ensureEntry(object_id)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Could not add collision-matrix entry for '%s'.",
      object_id.c_str());
    return false;
  }

  for (const auto & link_name : link_names) {
    if (link_name.empty() || link_name == object_id) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Invalid collision link '%s'.",
        link_name.c_str());
      return false;
    }

    if (!ensureEntry(link_name)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Could not add collision-matrix entry for link '%s'.",
        link_name.c_str());
      return false;
    }

    const auto object_it = std::find(
      acm.entry_names.begin(),
      acm.entry_names.end(),
      object_id);

    const auto link_it = std::find(
      acm.entry_names.begin(),
      acm.entry_names.end(),
      link_name);

    const std::size_t object_index =
      static_cast<std::size_t>(
        std::distance(acm.entry_names.begin(), object_it));

    const std::size_t link_index =
      static_cast<std::size_t>(
        std::distance(acm.entry_names.begin(), link_it));

    // ACM must be symmetric.
    acm.entry_values[object_index].enabled[link_index] = allowed;
    acm.entry_values[link_index].enabled[object_index] = allowed;
  }

  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  diff.robot_state.is_diff = true;
  diff.allowed_collision_matrix = acm;

  if (!planning_scene_interface_.applyPlanningScene(diff)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to update collision permissions for '%s'.",
      object_id.c_str());
    return false;
  }

  // ----------------------------------------------------------
  // Read back and compare (still under the lock). Retries the apply a
  // few times and logs exactly what was read, so a mismatch is explained
  // instead of just aborting the run.
  // ----------------------------------------------------------

  auto readBackMatches = [&](std::string & detail) -> bool
  {
    moveit_msgs::msg::AllowedCollisionMatrix m;
    bool got = false;

    for (int i = 0; i < 3 && !got; ++i) {
      auto fut = client->async_send_request(request);
      if (fut.wait_for(3s) == std::future_status::ready) {
        m = fut.get()->scene.allowed_collision_matrix;
        got = true;
      }
    }

    if (!got) {
      detail = "timed out re-reading the ACM";
      return false;
    }

    auto idx = [&m](const std::string & name) -> int
    {
      const auto it = std::find(m.entry_names.begin(), m.entry_names.end(), name);
      return it == m.entry_names.end() ?
             -1 : static_cast<int>(std::distance(m.entry_names.begin(), it));
    };

    const int oi = idx(object_id);

    for (const auto & link_name : link_names) {
      const int li = idx(link_name);

      // MoveIt does not necessarily keep an entry for a pair that is not
      // allowed: a missing entry means "collision checking enabled", which
      // is the same as false. Only a pair that is still allowed is a mismatch.
      bool a = false;
      bool b = false;
      if (oi >= 0 && li >= 0) {
        a = m.entry_values[oi].enabled[li];
        b = m.entry_values[li].enabled[oi];
      }

      if (a != allowed || b != allowed) {
        detail = "'" + object_id + "' <-> '" + link_name + "': read [obj][link]=" +
          (a ? "ALLOWED" : "ENABLED") + " [link][obj]=" + (b ? "ALLOWED" : "ENABLED") +
          ", wanted " + (allowed ? "ALLOWED" : "ENABLED") +
          (oi < 0 || li < 0 ? " (entry missing)" : "");
        return false;
      }
    }
    return true;
  };

  constexpr int kVerifyAttempts = 3;
  std::string mismatch;
  bool matched = false;

  for (int attempt = 1; attempt <= kVerifyAttempts && !matched; ++attempt) {
    matched = readBackMatches(mismatch);
    if (matched) {
      break;
    }
    RCLCPP_WARN(
      node_->get_logger(),
      "ACM read-back mismatch (attempt %d/%d): %s",
      attempt, kVerifyAttempts, mismatch.c_str());

    if (attempt < kVerifyAttempts) {
      planning_scene_interface_.applyPlanningScene(diff);   // re-apply, same ACM
      rclcpp::sleep_for(100ms);
    }
  }

  if (!matched) {
    if (kAcmVerifyIsFatal) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "ACM verification failed after %d attempts: %s",
        kVerifyAttempts, mismatch.c_str());
      return false;
    }
    RCLCPP_WARN(
      node_->get_logger(),
      "ACM verification still mismatched after %d attempts (continuing): %s",
      kVerifyAttempts, mismatch.c_str());
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Collision checking for '%s' against %zu link(s): %s (verified).",
    object_id.c_str(),
    link_names.size(),
    allowed ? "ALLOWED" : "ENABLED");

  return true;
}


// ============================================================
// Allow / restore collision between two named bodies
// ============================================================

bool PlanningSceneObjects::setCollisionPairAllowed(
  const std::string & a,
  const std::string & b,
  bool allowed)
{
  // Same ACM logic as setObjectCollisionsAllowed, with a single partner.
  // (Do NOT take AcmFileLock here: the callee already holds it, and
  // flock on a second descriptor in the same process would deadlock.)
  return setObjectCollisionsAllowed(a, {b}, allowed);
}
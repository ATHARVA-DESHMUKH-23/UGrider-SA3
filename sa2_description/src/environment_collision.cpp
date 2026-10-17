#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <urdf/model.h>

#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>


class EnvironmentCollision : public rclcpp::Node
{
public:

  EnvironmentCollision()
  : Node("environment_collision")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));

    qos.reliable();
    qos.transient_local();

    subscription_ =
      this->create_subscription<std_msgs::msg::String>(
        "/environment/robot_description",
        qos,
        std::bind(
          &EnvironmentCollision::robot_description_callback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Waiting for /environment/robot_description..."
    );
  }


private:

  // ------------------------------------------------------------
  // Convert URDF pose to Eigen transform
  // ------------------------------------------------------------

  Eigen::Isometry3d urdf_pose_to_eigen(
    const urdf::Pose & pose)
  {
    Eigen::Isometry3d transform =
      Eigen::Isometry3d::Identity();

    transform.translation() <<
      pose.position.x,
      pose.position.y,
      pose.position.z;

    Eigen::Quaterniond q(
      pose.rotation.w,
      pose.rotation.x,
      pose.rotation.y,
      pose.rotation.z);

    if (q.norm() > 1e-12)
    {
      q.normalize();
      transform.linear() =
        q.toRotationMatrix();
    }

    return transform;
  }


  // ------------------------------------------------------------
  // Convert Eigen transform to ROS Pose
  // ------------------------------------------------------------

  geometry_msgs::msg::Pose eigen_to_pose(
    const Eigen::Isometry3d & transform)
  {
    geometry_msgs::msg::Pose pose;

    pose.position.x =
      transform.translation().x();

    pose.position.y =
      transform.translation().y();

    pose.position.z =
      transform.translation().z();

    Eigen::Quaterniond q(
      transform.rotation());

    q.normalize();

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
  }


  // ------------------------------------------------------------
  // Calculate world -> link transform recursively
  // ------------------------------------------------------------

  Eigen::Isometry3d get_world_to_link_transform(
    const urdf::Model & model,
    const urdf::LinkConstSharedPtr & link)
  {
    // World/root link
    if (!link->parent_joint)
    {
      return Eigen::Isometry3d::Identity();
    }

    const auto & parent_joint =
      link->parent_joint;

    auto parent_link =
      model.getLink(
        parent_joint->parent_link_name);

    if (!parent_link)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Could not find parent link '%s' for '%s'.",
        parent_joint->parent_link_name.c_str(),
        link->name.c_str());

      return Eigen::Isometry3d::Identity();
    }

    Eigen::Isometry3d world_to_parent =
      get_world_to_link_transform(
        model,
        parent_link);

    Eigen::Isometry3d parent_to_child =
      urdf_pose_to_eigen(
        parent_joint->parent_to_joint_origin_transform);

    return world_to_parent * parent_to_child;
  }


  // ------------------------------------------------------------
  // Receive environment URDF
  // ------------------------------------------------------------

  void robot_description_callback(
    const std_msgs::msg::String::SharedPtr msg)
  {
    if (robot_description_received_)
    {
      return;
    }

    robot_description_received_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Received environment robot_description."
    );

    parse_environment(msg->data);
  }


  // ------------------------------------------------------------
  // Parse environment and create MoveIt collision objects
  // ------------------------------------------------------------

  void parse_environment(
    const std::string & xml)
  {
    urdf::Model model;

    if (!model.initString(xml))
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to parse environment URDF.");

      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Successfully parsed environment URDF: %s",
      model.getName().c_str());


    std::vector<
      moveit_msgs::msg::CollisionObject>
      collision_objects;


    // ----------------------------------------------------------
    // Process every link in the environment URDF
    // ----------------------------------------------------------

    for (const auto & pair : model.links_)
    {
      const auto & link = pair.second;

      if (!link)
      {
        continue;
      }

      // No collision geometry
      if (!link->collision)
      {
        continue;
      }

      if (!link->collision->geometry)
      {
        continue;
      }


      // --------------------------------------------------------
      // We currently handle mesh collision geometry
      // --------------------------------------------------------

      if (
        link->collision->geometry->type !=
        urdf::Geometry::MESH)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Skipping non-mesh collision geometry: %s",
          link->name.c_str());

        continue;
      }


      auto mesh =
        std::dynamic_pointer_cast<urdf::Mesh>(
          link->collision->geometry);

      if (!mesh)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Could not read mesh geometry from link: %s",
          link->name.c_str());

        continue;
      }


      // --------------------------------------------------------
      // Get mesh scale directly from URDF
      // --------------------------------------------------------

      Eigen::Vector3d scale(
        mesh->scale.x,
        mesh->scale.y,
        mesh->scale.z);


      RCLCPP_INFO(
        this->get_logger(),
        "");
      
      RCLCPP_INFO(
        this->get_logger(),
        "Environment object: %s",
        link->name.c_str());

      RCLCPP_INFO(
        this->get_logger(),
        "  mesh: %s",
        mesh->filename.c_str());

      RCLCPP_INFO(
        this->get_logger(),
        "  scale: %.6f %.6f %.6f",
        mesh->scale.x,
        mesh->scale.y,
        mesh->scale.z);


      // --------------------------------------------------------
      // Calculate:
      //
      // world
      //   ↓
      // parent link
      //   ↓
      // environment link
      //   ↓
      // collision origin
      // --------------------------------------------------------

      Eigen::Isometry3d world_to_link =
        get_world_to_link_transform(
          model,
          link);

      Eigen::Isometry3d link_to_collision =
        urdf_pose_to_eigen(
          link->collision->origin);

      Eigen::Isometry3d world_to_collision =
        world_to_link *
        link_to_collision;


      geometry_msgs::msg::Pose collision_pose =
        eigen_to_pose(
          world_to_collision);


      RCLCPP_INFO(
        this->get_logger(),
        "  world position: %.4f %.4f %.4f",
        collision_pose.position.x,
        collision_pose.position.y,
        collision_pose.position.z);


      RCLCPP_INFO(
        this->get_logger(),
        "  world orientation: %.4f %.4f %.4f %.4f",
        collision_pose.orientation.x,
        collision_pose.orientation.y,
        collision_pose.orientation.z,
        collision_pose.orientation.w);


      // --------------------------------------------------------
      // Load mesh
      // --------------------------------------------------------

      shapes::Mesh * shape_mesh =
        shapes::createMeshFromResource(
          mesh->filename,
          scale);

      if (shape_mesh == nullptr)
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to load mesh: %s",
          mesh->filename.c_str());

        continue;
      }


      // --------------------------------------------------------
      // Convert geometric_shapes mesh to ROS mesh message
      // --------------------------------------------------------

      shapes::ShapeMsg shape_msg;

      if (!shapes::constructMsgFromShape(
            shape_mesh,
            shape_msg))
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to convert mesh: %s",
          mesh->filename.c_str());

        delete shape_mesh;

        continue;
      }

      delete shape_mesh;


      // --------------------------------------------------------
      // Create MoveIt collision object
      // --------------------------------------------------------

      moveit_msgs::msg::CollisionObject object;

      object.id =
        link->name;

      object.header.frame_id =
        "world";


      object.meshes.push_back(
        boost::get<shape_msgs::msg::Mesh>(
          shape_msg));


      object.mesh_poses.push_back(
        collision_pose);


      object.operation =
        moveit_msgs::msg::CollisionObject::ADD;


      collision_objects.push_back(
        object);
    }


    // ----------------------------------------------------------
    // Check result
    // ----------------------------------------------------------

    if (collision_objects.empty())
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "No mesh collision objects were found.");

      return;
    }


    RCLCPP_INFO(
      this->get_logger(),
      "----------------------------------------");

    RCLCPP_INFO(
      this->get_logger(),
      "Prepared %zu environment collision objects.",
      collision_objects.size());

    RCLCPP_INFO(
      this->get_logger(),
      "----------------------------------------");


    // ----------------------------------------------------------
    // Send objects to MoveIt PlanningScene
    // ----------------------------------------------------------

    moveit::planning_interface::
      PlanningSceneInterface
      planning_scene_interface;


    planning_scene_interface.applyCollisionObjects(
      collision_objects);


    RCLCPP_INFO(
      this->get_logger(),
      "Added %zu environment collision objects to MoveIt.",
      collision_objects.size());
  }


  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    subscription_;

  bool robot_description_received_{false};
};


int main(
  int argc,
  char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<
      EnvironmentCollision>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("add_collision_box");
  const auto id = node->declare_parameter<std::string>("box_id", "my_collision_box");
  const auto frame = node->declare_parameter<std::string>("frame_id", "world");
  const auto sx = node->declare_parameter<double>("size_x", 10.0);
  const auto sy = node->declare_parameter<double>("size_y", 0.5);
  const auto sz = node->declare_parameter<double>("size_z", 10.3);
  const auto x = node->declare_parameter<double>("x", -1.0);
  const auto y = node->declare_parameter<double>("y", 0.4);
  const auto z = node->declare_parameter<double>("z", 5.15);
  const auto roll = node->declare_parameter<double>("roll", 0.0);
  const auto pitch = node->declare_parameter<double>("pitch", 0.0);
  const auto yaw = node->declare_parameter<double>("yaw", 0.0);
  const auto operation = node->declare_parameter<std::string>("operation", "add");

  const std::vector<double> values{sx, sy, sz, x, y, z, roll, pitch, yaw};
  for (double v : values) {
    if (!std::isfinite(v)) {
      RCLCPP_ERROR(node->get_logger(), "All box parameters must be finite.");
      rclcpp::shutdown();
      return 1;
    }
  }
  if (id.empty() || frame.empty() ||
      (operation != "add" && operation != "remove") ||
      (operation == "add" && (sx <= 0.0 || sy <= 0.0 || sz <= 0.0))) {
    RCLCPP_ERROR(node->get_logger(), "Invalid id/frame/operation or box dimensions.");
    rclcpp::shutdown();
    return 1;
  }

  moveit::planning_interface::PlanningSceneInterface scene;
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame;
  object.id = id;

  if (operation == "remove") {
    object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  } else {
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {sx, sy, sz};
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
  }

  const bool ok = scene.applyCollisionObject(object);
  if (ok) {
    RCLCPP_INFO(node->get_logger(), "%s collision object '%s' in frame '%s'.",
      operation == "add" ? "Applied" : "Removed", id.c_str(), frame.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "Planning scene failed to apply '%s'.", id.c_str());
  }
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
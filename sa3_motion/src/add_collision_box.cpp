#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("add_collision_box");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // ============================================================
    // Collision object
    // ============================================================

    moveit_msgs::msg::CollisionObject collision_object;

    collision_object.header.frame_id = "world";
    collision_object.id = "my_collision_box";

    // ============================================================
    // Define cuboid
    // ============================================================

    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;

    // BOX dimensions = X, Y, Z
    primitive.dimensions.resize(3);

    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 10.0;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.5;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 10.3;

    // ============================================================
    // Position of cuboid
    // ============================================================

    geometry_msgs::msg::Pose box_pose;

    box_pose.position.x = -1.0;
    box_pose.position.y = 0.4;
    box_pose.position.z = 5.15;

    // No rotation
    box_pose.orientation.x = 0.0;
    box_pose.orientation.y = 0.0;
    box_pose.orientation.z = 0.0;
    box_pose.orientation.w = 1.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);

    // ADD object to planning scene
    collision_object.operation =
        moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_interface.applyCollisionObject(collision_object);

    RCLCPP_INFO(
        node->get_logger(),
        "Added collision box to MoveIt planning scene."
    );

    rclcpp::sleep_for(std::chrono::seconds(2));

    rclcpp::shutdown();

    return 0;
}
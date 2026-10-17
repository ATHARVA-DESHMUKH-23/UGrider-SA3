#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <vector>

int main(int argc, char * argv[])
{
  // ------------------------------------------------------------
  // Start ROS 2
  // ------------------------------------------------------------
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "sa3_motion",
    rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true));

  RCLCPP_INFO(
    node->get_logger(),
    "========================================");

  RCLCPP_INFO(
    node->get_logger(),
    "SA3 MOTION TEST STARTING");

  RCLCPP_INFO(
    node->get_logger(),
    "========================================");


  // ------------------------------------------------------------
  // Connect to MoveIt
  // ------------------------------------------------------------
  moveit::planning_interface::MoveGroupInterface move_group(
    node,
    "robot1_arm");

  RCLCPP_INFO(
    node->get_logger(),
    "Connected to MoveIt group: robot1_arm");

  RCLCPP_INFO(
    node->get_logger(),
    "Planning frame: %s",
    move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(
    node->get_logger(),
    "End-effector link: %s",
    move_group.getEndEffectorLink().c_str());


  // ------------------------------------------------------------
  // Get current robot state
  // ------------------------------------------------------------
  auto current_state =
    move_group.getCurrentState(10.0);

  if (current_state == nullptr)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Could not obtain current robot state.");

    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Current robot state received.");


  // ------------------------------------------------------------
  // Get robot1_arm joint group
  // ------------------------------------------------------------
  const auto * joint_model_group =
    current_state->getJointModelGroup("robot1_arm");

  if (joint_model_group == nullptr)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Could not find joint model group: robot1_arm");

    rclcpp::shutdown();
    return 1;
  }


  // ------------------------------------------------------------
  // Read current joint values
  // ------------------------------------------------------------
  std::vector<double> current_joint_values;

  current_state->copyJointGroupPositions(
    joint_model_group,
    current_joint_values);

  RCLCPP_INFO(
    node->get_logger(),
    "Current robot1_arm joint values:");

  for (std::size_t i = 0;
       i < current_joint_values.size();
       ++i)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "  joint[%zu] = %.6f rad",
      i,
      current_joint_values[i]);
  }


  // ------------------------------------------------------------
  // Check that this is a 6-axis robot
  // ------------------------------------------------------------
  if (current_joint_values.size() != 6)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Expected 6 joints, but received %zu joints.",
      current_joint_values.size());

    rclcpp::shutdown();
    return 1;
  }


  // ------------------------------------------------------------
  // Create target
  //
  // Only J1 is changed.
  // J1 will move +10 degrees from the current position.
  //
  // This is ONLY a planning test.
  // The robot will NOT execute the trajectory.
  // ------------------------------------------------------------

  constexpr double PI =
    3.14159265358979323846;

  constexpr double DEG_TO_RAD =
    PI / 180.0;

  std::vector<double> target_joint_values =
    current_joint_values;

  target_joint_values[0] +=
    10.0 * DEG_TO_RAD;


  RCLCPP_INFO(
    node->get_logger(),
    "Target joint values:");

  for (std::size_t i = 0;
       i < target_joint_values.size();
       ++i)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "  joint[%zu] = %.6f rad",
      i,
      target_joint_values[i]);
  }


  // ------------------------------------------------------------
  // Give target to MoveIt
  // ------------------------------------------------------------
  bool target_set =
    move_group.setJointValueTarget(
      target_joint_values);

  if (!target_set)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "MoveIt rejected the joint target.");

    rclcpp::shutdown();
    return 1;
  }


  // ------------------------------------------------------------
  // Planning configuration
  // ------------------------------------------------------------
  move_group.setPlanningTime(5.0);

  move_group.setNumPlanningAttempts(5);


  RCLCPP_INFO(
    node->get_logger(),
    "Planning +10 degrees on robot1 J1...");


  // ------------------------------------------------------------
  // Plan
  // ------------------------------------------------------------
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  const auto result =
    move_group.plan(plan);


  // ------------------------------------------------------------
  // Check result
  // ------------------------------------------------------------
  if (result == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "========================================");

    RCLCPP_INFO(
      node->get_logger(),
      "PLANNING SUCCESS");

    RCLCPP_INFO(
      node->get_logger(),
      "MoveIt found a valid trajectory.");

    RCLCPP_INFO(
      node->get_logger(),
      "The trajectory was NOT executed.");

    RCLCPP_INFO(
      node->get_logger(),
      "========================================");
  }
  else
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "========================================");

    RCLCPP_ERROR(
      node->get_logger(),
      "PLANNING FAILED");

    RCLCPP_ERROR(
      node->get_logger(),
      "MoveIt could not find a valid trajectory.");

    RCLCPP_ERROR(
      node->get_logger(),
      "========================================");
  }


  // ------------------------------------------------------------
  // Shutdown
  // ------------------------------------------------------------
  rclcpp::shutdown();

  if (result == moveit::core::MoveItErrorCode::SUCCESS)
  {
    return 0;
  }

  return 1;
}
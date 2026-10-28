#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

using FJT = control_msgs::action::FollowJointTrajectory;
using MoveItErr = moveit_msgs::msg::MoveItErrorCodes;

namespace
{

constexpr double kHomeTolerance = 0.02;  // radians
constexpr auto kVerifyTimeout = 5s;
constexpr auto kVerifyPollPeriod = 250ms;
constexpr double kExecuteMargin = 10.0;

struct RobotConfig
{
  std::string name;
  std::string group;
  std::string controller_action;
};

RobotConfig makeRobot(
  const std::string & name,
  const std::string & group,
  const std::string & controller)
{
  return RobotConfig{name, group, controller};
}

// Execute the planned trajectory directly on this robot's controller.
// This avoids routing execution through MoveIt's shared execute_trajectory action.
bool executeDirect(
  const rclcpp::Logger & logger,
  const rclcpp_action::Client<FJT>::SharedPtr & client,
  const moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
  const auto & trajectory = plan.trajectory.joint_trajectory;

  if (trajectory.points.empty()) {
    RCLCPP_ERROR(logger, "Planned trajectory contains no points.");
    return false;
  }

  if (!client->wait_for_action_server(10s)) {
    RCLCPP_ERROR(logger, "Controller action server is unavailable.");
    return false;
  }

  FJT::Goal goal;
  goal.trajectory = trajectory;

  auto goal_future = client->async_send_goal(goal);

  if (goal_future.wait_for(10s) != std::future_status::ready) {
    RCLCPP_ERROR(logger, "Timed out while sending controller goal.");
    return false;
  }

  auto goal_handle = goal_future.get();

  if (!goal_handle) {
    RCLCPP_ERROR(logger, "Controller rejected the trajectory.");
    return false;
  }

  const auto & last = trajectory.points.back().time_from_start;
  const double duration =
    static_cast<double>(last.sec) +
    1e-9 * static_cast<double>(last.nanosec);

  auto result_future = client->async_get_result(goal_handle);

  if (result_future.wait_for(
      std::chrono::duration<double>(duration + kExecuteMargin)) !=
      std::future_status::ready)
  {
    (void)client->async_cancel_goal(goal_handle);
    RCLCPP_ERROR(logger, "Controller execution timed out.");
    return false;
  }

  const auto result = result_future.get();

  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(
      logger, "Controller execution failed. Result code: %d",
      static_cast<int>(result.code));

    if (result.result) {
      RCLCPP_ERROR(
        logger, "Controller error: %d, %s",
        result.result->error_code,
        result.result->error_string.c_str());
    }

    return false;
  }

  if (result.result &&
      result.result->error_code != FJT::Result::SUCCESSFUL)
  {
    RCLCPP_ERROR(
      logger, "Controller reported trajectory error: %d, %s",
      result.result->error_code,
      result.result->error_string.c_str());
    return false;
  }

  return true;
}

bool verifyHomeOnce(
  const rclcpp::Logger & logger,
  moveit::planning_interface::MoveGroupInterface & move_group)
{
  const auto home_values = move_group.getNamedTargetValues("home");
  const auto current_state = move_group.getCurrentState(2.0);

  if (home_values.empty()) {
    RCLCPP_ERROR(
      logger, "Named target 'home' has no joint values for this group.");
    return false;
  }

  if (!current_state) {
    RCLCPP_WARN(logger, "Current robot state is not available yet.");
    return false;
  }

  const auto * robot_model = current_state->getRobotModel().get();
  if (!robot_model) {
    RCLCPP_ERROR(logger, "Current state has no robot model.");
    return false;
  }

  bool all_home = true;

  for (const auto & joint : home_values) {
    const std::string & joint_name = joint.first;
    const double target = joint.second;

    if (robot_model->getVariableIndex(joint_name) < 0) {
      RCLCPP_ERROR(
        logger, "Joint variable '%s' is missing from the robot model.",
        joint_name.c_str());
      all_home = false;
      continue;
    }

    const double actual = current_state->getVariablePosition(joint_name);
    const double error =
      std::abs(std::remainder(actual - target, 2.0 * M_PI));

    if (error > kHomeTolerance) {
      RCLCPP_WARN(
        logger,
        "%s not home: actual=%.4f target=%.4f error=%.4f rad (tolerance %.4f)",
        joint_name.c_str(), actual, target, error, kHomeTolerance);
      all_home = false;
    } else {
      RCLCPP_INFO(
        logger, "%s verified: %.4f rad (target %.4f, error %.4f)",
        joint_name.c_str(), actual, target, error);
    }
  }

  return all_home;
}

bool verifyHomeWithRetry(
  const rclcpp::Logger & logger,
  moveit::planning_interface::MoveGroupInterface & move_group)
{
  const auto deadline = std::chrono::steady_clock::now() + kVerifyTimeout;

  while (std::chrono::steady_clock::now() < deadline) {
    if (verifyHomeOnce(logger, move_group)) {
      return true;
    }
    rclcpp::sleep_for(kVerifyPollPeriod);
  }

  RCLCPP_ERROR(
    logger, "Home verification did not pass within %.1f seconds.",
    std::chrono::duration<double>(kVerifyTimeout).count());
  return false;
}

bool runRobot(const RobotConfig & cfg)
{
  const auto options =
    rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp::Node>(cfg.name, options);
  auto logger = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread executor_thread([&executor]() {
    executor.spin();
  });

  bool success = false;

  try {
    RCLCPP_INFO(
      logger, "Waiting for MoveIt before homing group '%s'.",
      cfg.group.c_str());

    rclcpp::sleep_for(2s);

    moveit::planning_interface::MoveGroupInterface move_group(
      node, cfg.group);

    move_group.setPlanningTime(15.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setMaxVelocityScalingFactor(0.15);
    move_group.setMaxAccelerationScalingFactor(0.10);

    const auto current_state = move_group.getCurrentState(10.0);

    if (!current_state) {
      RCLCPP_ERROR(logger, "Current joint state is unavailable.");
      throw std::runtime_error("Current state unavailable");
    }

    auto controller = rclcpp_action::create_client<FJT>(
      node, cfg.controller_action);

    if (!controller->wait_for_action_server(10s)) {
      RCLCPP_ERROR(
        logger, "Controller action unavailable: %s",
        cfg.controller_action.c_str());
      throw std::runtime_error("Controller unavailable");
    }

    RCLCPP_INFO(
      logger, "Planning home for group '%s'.", cfg.group.c_str());

    if (!move_group.setNamedTarget("home")) {
      RCLCPP_ERROR(
        logger, "Named target 'home' unavailable for group '%s'.",
        cfg.group.c_str());
      throw std::runtime_error("Home target unavailable");
    }

    move_group.setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group.plan(plan);

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(
        logger, "Planning home failed for group '%s' (code %d).",
        cfg.group.c_str(), plan_result.val);
      throw std::runtime_error("Planning failed");
    }

    RCLCPP_INFO(
      logger, "Home trajectory planned for '%s'.", cfg.group.c_str());

    RCLCPP_INFO(
      logger, "Executing home trajectory on %s.",
      cfg.controller_action.c_str());

    if (!executeDirect(logger, controller, plan)) {
      throw std::runtime_error("Execution failed");
    }

    RCLCPP_INFO(
      logger, "Controller completed trajectory for '%s'; verifying feedback.",
      cfg.group.c_str());

    if (!verifyHomeWithRetry(logger, move_group)) {
      throw std::runtime_error("Home verification failed");
    }

    RCLCPP_INFO(
      logger, "SUCCESS: '%s' is confirmed at home.", cfg.group.c_str());

    success = true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger, "Robot '%s' failed: %s", cfg.name.c_str(), e.what());
  }

  executor.cancel();

  if (executor_thread.joinable()) {
    executor_thread.join();
  }

  return success;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const std::vector<RobotConfig> robots = {
    makeRobot(
      "robot1_home", "robot1_arm",
      "/robot1_arm_controller/follow_joint_trajectory"),

    makeRobot(
      "robot2_home", "robot2_arm",
      "/robot2_arm_controller/follow_joint_trajectory"),

    makeRobot(
      "robot3_home", "robot3_arm",
      "/robot3_arm_controller/follow_joint_trajectory"),

    makeRobot(
      "robot4_home", "robot4_arm",
      "/robot4_arm_controller/follow_joint_trajectory")
  };

  // int is used rather than vector<bool>: vector<bool> packs bits and can
  // cause a data race when different worker threads write concurrently.
  std::vector<int> results(robots.size(), 0);
  std::vector<std::thread> workers;
  workers.reserve(robots.size());

  for (std::size_t i = 0; i < robots.size(); ++i) {
    workers.emplace_back([&, i]() {
      results[i] = runRobot(robots[i]) ? 1 : 0;
    });
  }

  for (auto & worker : workers) {
    if (worker.joinable()) {
      worker.join();
    }
  }

  bool all_home = true;

  for (std::size_t i = 0; i < results.size(); ++i) {
    if (results[i] == 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("home_all_robots"),
        "%s did not complete and verify home.", robots[i].name.c_str());
      all_home = false;
    }
  }

  if (all_home) {
    RCLCPP_INFO(
      rclcpp::get_logger("home_all_robots"),
      "SUCCESS: All four arms completed home verification.");
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("home_all_robots"),
      "FAILED: At least one arm did not verify home.");
  }

  rclcpp::shutdown();
  return all_home ? 0 : 1;
}
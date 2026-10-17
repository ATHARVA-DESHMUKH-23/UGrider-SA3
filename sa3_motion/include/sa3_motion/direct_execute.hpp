// direct_execute.hpp  (sa3_motion)
//
// Send an already-planned trajectory straight to ONE arm's controller
// (FollowJointTrajectory action), bypassing move_group's single
// execute_trajectory server. Used by the standalone robot1 / robot2 nodes so
// they can run in parallel from separate processes.
//
// Put this file next to robot1_motion.cpp / robot2_motion.cpp (in src/) and
// include it with:   #include "direct_execute.hpp"
//
// The node that owns the action client MUST be spun by another thread
// (your main() already does this with executor_thread), because this
// function blocks on futures.

#pragma once

#include <chrono>
#include <future>
#include <string>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace sa3_motion
{

using FJT = control_msgs::action::FollowJointTrajectory;

// Returns true on success. On failure, error_code is a MoveItErrorCodes value
// and note explains what happened (use them in reportFailure()).
inline bool executeDirect(
  const rclcpp_action::Client<FJT>::SharedPtr & client,
  const trajectory_msgs::msg::JointTrajectory & traj,
  int & error_code, std::string & note, double margin_s = 10.0)
{
  using MoveItErr = moveit_msgs::msg::MoveItErrorCodes;
  using namespace std::chrono_literals;

  if (traj.points.empty()) {
    error_code = MoveItErr::INVALID_MOTION_PLAN;
    note = "planned trajectory has no points.";
    return false;
  }
  if (!client->wait_for_action_server(5s)) {
    error_code = MoveItErr::CONTROL_FAILED;
    note = "controller action server not available.";
    return false;
  }

  FJT::Goal goal;
  goal.trajectory = traj;

  auto goal_fut = client->async_send_goal(goal);
  if (goal_fut.wait_for(10s) != std::future_status::ready) {
    error_code = MoveItErr::TIMED_OUT;
    note = "sending the goal to the controller timed out.";
    return false;
  }
  auto goal_handle = goal_fut.get();
  if (!goal_handle) {
    error_code = MoveItErr::CONTROL_FAILED;
    note = "controller rejected the goal.";
    return false;
  }

  const auto & last = traj.points.back().time_from_start;
  const double duration = static_cast<double>(last.sec) + 1e-9 * static_cast<double>(last.nanosec);

  auto result_fut = client->async_get_result(goal_handle);
  if (result_fut.wait_for(std::chrono::duration<double>(duration + margin_s)) !=
    std::future_status::ready)
  {
    client->async_cancel_goal(goal_handle);
    error_code = MoveItErr::TIMED_OUT;
    note = "controller did not finish within trajectory duration + margin; goal cancelled.";
    return false;
  }

  const auto result = result_fut.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    error_code = MoveItErr::CONTROL_FAILED;
    note = "controller result code " + std::to_string(static_cast<int>(result.code));
    if (result.result) {
      note += ", error_code " + std::to_string(result.result->error_code);
      if (!result.result->error_string.empty()) {
        note += " (" + result.result->error_string + ")";
      }
    }
    return false;
  }

  error_code = MoveItErr::SUCCESS;
  note.clear();
  return true;
}

}  // namespace sa3_motion
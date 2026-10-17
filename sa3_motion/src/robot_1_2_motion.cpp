// dual_robot_motion.cpp  (sa3_motion)
//
// Uses ONE planning group per arm (robot1_arm, robot2_arm). Pilz LIN needs a
// single-chain group with an IK solver, so the combined group sa3_robots_1_2
// cannot be used for movel.
//
// One process, one worker thread per arm. Each worker has its own node,
// executor, TF buffer, MoveGroupInterface and FollowJointTrajectory client.
//
// PLANNING goes through MoveGroupInterface (move_group).
// EXECUTION is sent DIRECTLY to each arm's controller (FollowJointTrajectory
// action), bypassing move_group's single execute_trajectory server. This lets
// both arms really move at the same time instead of one waiting for the other.
//
//   movej : OMPL pose-target move
//   movel : Pilz LIN straight line
//
// Steps per pair:
//   1  movej  pick approach
//   2  movel  pick
//   3  movel  pick retract
//   4  movel  transition            (only if use_transition = true)
//   .  place approach  (movel, optional movej fallback)
//   .  movel  place
//   .  movel  place retract
//
// ALL EDITABLE VALUES ARE IN THE "USER CONFIG" SECTION BELOW.
// Offsets are in the WORLD frame (metres).
//
// Sync options (also in USER CONFIG):
//   kLockstepPairs : both arms wait for each other before starting each pair index.
//   Job::exclusive : a pair marked exclusive holds a shared mutex, so only one
//                    arm works in that job at a time (shared zone protection).

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <future>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;
using MoveItErr = moveit_msgs::msg::MoveItErrorCodes;
using FJT = control_msgs::action::FollowJointTrajectory;
using Vec3 = std::array<double, 3>;   // {x, y, z} in world frame, metres

namespace
{

// ===========================================================================
//                              USER CONFIG
// ===========================================================================

// Frame all poses are expressed in.
constexpr char kReferenceFrame[] = "world";

// true  : arms wait for each other at the start of every pair index
// false : arms run completely independently (except Job::exclusive zones)
constexpr bool kLockstepPairs = false;

// true  : a robot stops at its first failed pair (rest are NOT RUN)
// false : report the failure and carry on with its next pair
constexpr bool kStopOnFirstFailure = false;

// Extra seconds allowed on top of the trajectory duration before the
// controller result is treated as timed out.
constexpr double kExecuteMargin = 10.0;

// One pick frame -> several place frames.
//   pick_step : added to the pick position, multiplied by the place index j
//               (j = 0 -> 0, j = 1 -> 1*pick_step, j = 2 -> 2*pick_step ...)
//               {x, y, z} in metres. Use it to walk along a row of items.
//   exclusive : true -> only one arm may run this job at a time.
struct Job
{
  std::string pick;
  std::vector<std::string> places;
  Vec3 pick_step = {0.0, 0.0, 0.0};
  bool exclusive = false;
};

struct Motion
{
  double planning_time = 15.0;
  int ompl_attempts = 10;
  int plan_retries = 3;
  double movej_vel = 0.30;
  double movej_acc = 0.20;
  double movel_vel = 0.20;
  double movel_acc = 0.10;
  double goal_pos_tol = 0.001;
  double goal_ori_tol = 0.01;
};

struct RobotConfig
{
  std::string name;               // used for node name and log prefix
  std::string group;              // MoveIt group
  std::string link;               // end effector link (spelling matches SRDF)
  std::string controller_action;  // FollowJointTrajectory action of this arm's controller

  // Approach and retract offset from pick / place (world frame).
  Vec3 approach_offset = {0.0, -0.25, 0.0};

  // Fixed fine-tuning added to every pick / every place pose (world frame).
  Vec3 pick_offset = {0.0, 0.0, 0.0};
  Vec3 place_offset = {0.0, 0.0, 0.0};

  // Transition step between pick retract and place approach:
  // starts from pick_approach, then copies the ticked axes from place_approach.
  bool use_transition = false;
  std::array<bool, 3> transition_copy_axes = {false, true, true};   // {x, y, z}

  // Place approach: try movel first, fall back to movej if movel fails.
  bool place_approach_fallback = true;

  Motion motion;
  std::vector<Job> jobs;
};

RobotConfig makeRobot1()
{
  RobotConfig c;
  c.name = "robot1_pick_place";
  c.group = "robot1_arm";
  c.link = "robot1_end_effactor";   // (sic) matches SRDF
  // check the exact name with:  ros2 action list | grep follow_joint
  c.controller_action = "/robot1_arm_controller/follow_joint_trajectory";

  c.approach_offset = {0.0, -0.25, 0.0};
  c.pick_offset = {0.0, 0.0, 0.0};
  c.place_offset = {0.0, 0.0, 0.0};

  c.use_transition = false;
  c.place_approach_fallback = true;

  //          pick     places                                                         pick_step           exclusive
  c.jobs = {
    {"5pf",  {"5_placef_l"},                                                          {0.00, 0.0, 0.0},   false},
    {"13pf", {"13_placef1_l", "13_placef2_l", "13_placef3_l", "13_placef4_l"},        {0.10, 0.0, 0.0},   false},
    {"14pf", {"14_placef1_l", "14_placef2_l", "14_placef3_l", "14_placef4_l"},        {0.20, 0.0, 0.0},   false},
  };
  return c;
}

RobotConfig makeRobot2()
{
  RobotConfig c;
  c.name = "robot2_pick_place";
  c.group = "robot2_arm";
  c.link = "robot2_end_effactor";   // (sic) matches SRDF
  c.controller_action = "/robot2_arm_controller/follow_joint_trajectory";

  c.approach_offset = {0.0, -0.25, 0.0};
  c.pick_offset = {0.0, 0.0, 0.0};
  c.place_offset = {0.0, 0.0, 0.0};

  c.use_transition = true;
  c.transition_copy_axes = {false, true, true};
  c.place_approach_fallback = true;

  c.jobs = {
    {"6pf",    {"6_placef_r"},                                                        {0.000, 0.0, 0.0},  false},
    {"13_1pf", {"13_placef1_r", "13_placef2_r"},                                      {0.118, 0.0, 0.0},  false},
    {"13_2pf", {"13_placef3_r"},                                                      {0.000, 0.0, 0.0},  false},
    {"14_4pf", {"14_placef1_r", "14_placef2_r", "14_placef3_r", "14_placef4_r"},      {0.112, 0.0, 0.0},  false},
  };
  return c;
}

// ===========================================================================
//                           END OF USER CONFIG
// ===========================================================================

// ---------------------------------------------------------------------------
// Shared between the two workers
// ---------------------------------------------------------------------------
class PairBarrier
{
public:
  explicit PairBarrier(int participants)
  : active_(participants) {}

  void arriveAndWait()
  {
    std::unique_lock<std::mutex> lk(m_);
    const auto gen = gen_;
    if (++arrived_ >= active_) {
      arrived_ = 0;
      ++gen_;
      cv_.notify_all();
    } else {
      cv_.wait(lk, [&] {return gen != gen_;});
    }
  }

  // A worker that has finished (or died) must call this so the other never waits for it.
  void drop()
  {
    std::lock_guard<std::mutex> lk(m_);
    --active_;
    if (active_ > 0 && arrived_ >= active_) {
      arrived_ = 0;
      ++gen_;
      cv_.notify_all();
    }
  }

private:
  std::mutex m_;
  std::condition_variable cv_;
  int active_;
  int arrived_ = 0;
  unsigned long gen_ = 0;
};

struct Shared
{
  explicit Shared(int n)
  : barrier(n) {}
  PairBarrier barrier;
  std::mutex zone;        // Job::exclusive
  std::mutex print;       // keeps the summary tables from interleaving
};

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
geometry_msgs::msg::Pose offsetWorld(const geometry_msgs::msg::Pose & p, const Vec3 & d)
{
  auto r = p;
  r.position.x += d[0];
  r.position.y += d[1];
  r.position.z += d[2];
  return r;
}

std::string poseStr(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  char buf[200];
  std::snprintf(
    buf, sizeof(buf), "xyz=[%.4f, %.4f, %.4f] rpy=[%.4f, %.4f, %.4f]",
    pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw);
  return buf;
}

std::string errStr(int c)
{
  switch (c) {
    case MoveItErr::SUCCESS: return "SUCCESS";
    case MoveItErr::FAILURE: return "FAILURE";
    case MoveItErr::PLANNING_FAILED: return "PLANNING_FAILED";
    case MoveItErr::INVALID_MOTION_PLAN: return "INVALID_MOTION_PLAN";
    case MoveItErr::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case MoveItErr::CONTROL_FAILED: return "CONTROL_FAILED";
    case MoveItErr::UNABLE_TO_AQUIRE_SENSOR_DATA: return "UNABLE_TO_AQUIRE_SENSOR_DATA";
    case MoveItErr::TIMED_OUT: return "TIMED_OUT";
    case MoveItErr::PREEMPTED: return "PREEMPTED";
    case MoveItErr::START_STATE_IN_COLLISION: return "START_STATE_IN_COLLISION";
    case MoveItErr::START_STATE_VIOLATES_PATH_CONSTRAINTS: return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case MoveItErr::GOAL_IN_COLLISION: return "GOAL_IN_COLLISION";
    case MoveItErr::GOAL_VIOLATES_PATH_CONSTRAINTS: return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case MoveItErr::GOAL_CONSTRAINTS_VIOLATED: return "GOAL_CONSTRAINTS_VIOLATED";
    case MoveItErr::INVALID_GROUP_NAME: return "INVALID_GROUP_NAME";
    case MoveItErr::INVALID_GOAL_CONSTRAINTS: return "INVALID_GOAL_CONSTRAINTS";
    case MoveItErr::INVALID_ROBOT_STATE: return "INVALID_ROBOT_STATE";
    case MoveItErr::INVALID_LINK_NAME: return "INVALID_LINK_NAME";
    case MoveItErr::FRAME_TRANSFORM_FAILURE: return "FRAME_TRANSFORM_FAILURE";
    case MoveItErr::COLLISION_CHECKING_UNAVAILABLE: return "COLLISION_CHECKING_UNAVAILABLE";
    case MoveItErr::ROBOT_STATE_STALE: return "ROBOT_STATE_STALE";
    case MoveItErr::SENSOR_INFO_STALE: return "SENSOR_INFO_STALE";
    case MoveItErr::NO_IK_SOLUTION: return "NO_IK_SOLUTION";
    default: return "CODE_" + std::to_string(c);
  }
}

std::string hintFor(const std::string & planner, int code)
{
  switch (code) {
    case MoveItErr::NO_IK_SOLUTION:
      return "target pose has no IK solution (out of reach / orientation not achievable / IK timeout).";
    case MoveItErr::GOAL_IN_COLLISION:
      return "the target pose is in collision with the scene (or with the other arm).";
    case MoveItErr::START_STATE_IN_COLLISION:
      return "the robot's start state is in collision (previous step ended in contact?).";
    case MoveItErr::INVALID_MOTION_PLAN:
    case MoveItErr::GOAL_CONSTRAINTS_VIOLATED:
      return planner == "LIN" ?
             "straight line start->target is not feasible (collision on the line, joint limit or singularity)." :
             "planner returned a path that violates the goal constraints.";
    case MoveItErr::PLANNING_FAILED:
      return planner == "OMPL" ?
             "OMPL found no collision-free path within the time/attempt budget." :
             "Pilz could not build the straight-line path.";
    case MoveItErr::TIMED_OUT:
      return "planning or execution timed out.";
    case MoveItErr::CONTROL_FAILED:
      return "the controller rejected or aborted the trajectory.";
    default:
      return "";
  }
}

// ---------------------------------------------------------------------------
// step / failure bookkeeping (per robot)
// ---------------------------------------------------------------------------
struct StepInfo
{
  int pair_idx = 0;
  int pair_total = 0;
  int step_idx = 0;
  int step_total = 6;
  std::string pick;
  std::string place;
  std::string step;
};

struct Failure
{
  StepInfo info;
  std::string planner;   // "OMPL" | "LIN" | "TF"
  std::string stage;
  int code = 0;
  double plan_time = 0.0;
};

struct Ctx
{
  const RobotConfig * cfg = nullptr;
  rclcpp::Node::SharedPtr node;
  moveit::planning_interface::MoveGroupInterface * mg = nullptr;
  tf2_ros::Buffer * tf = nullptr;
  rclcpp_action::Client<FJT>::SharedPtr fjt;   // this arm's controller

  StepInfo cur;
  Vec3 pick_shift = {0.0, 0.0, 0.0};   // j * Job::pick_step for the current pair
  std::vector<Failure> failures;
};

std::string stepTag(const Ctx & c)
{
  std::ostringstream s;
  s << "[pair " << c.cur.pair_idx << "/" << c.cur.pair_total
    << " | step " << c.cur.step_idx << "/" << c.cur.step_total
    << " | " << c.cur.pick << " -> " << c.cur.place
    << " | " << c.cur.step << "] ";
  return s.str();
}

std::string plannerLabel(const std::string & p)
{
  if (p == "OMPL") {return "OMPL (RRTConnect)";}
  if (p == "LIN") {return "Pilz LIN (straight line)";}
  return p;
}

void reportFailure(
  Ctx & c, const std::string & planner, const std::string & stage, int code,
  double plan_time, const geometry_msgs::msg::Pose * target, const std::string & note = "")
{
  Failure f;
  f.info = c.cur;
  f.planner = planner;
  f.stage = stage;
  f.code = code;
  f.plan_time = plan_time;
  c.failures.push_back(f);

  std::ostringstream s;
  s << "\n================ " << c.cfg->name << " FAILURE #" << c.failures.size() << " ================\n"
    << " pair      : " << c.cur.pair_idx << " of " << c.cur.pair_total
    << "   (" << c.cur.pick << " -> " << c.cur.place << ")\n"
    << " step      : " << c.cur.step_idx << " of " << c.cur.step_total
    << "   '" << c.cur.step << "'\n"
    << " planner   : " << plannerLabel(planner) << "\n"
    << " stage     : " << stage << "\n"
    << " error     : " << errStr(code) << " (" << code << ")\n"
    << " time      : " << std::fixed << std::setprecision(2) << plan_time << " s\n";
  if (target) {
    s << " target    : " << poseStr(*target) << "\n";
  }
  if (c.mg) {
    const auto names = c.mg->getActiveJoints();
    const auto vals = c.mg->getCurrentJointValues();
    s << " start jnts: ";
    for (size_t i = 0; i < names.size() && i < vals.size(); ++i) {
      s << names[i] << "=" << std::setprecision(3) << vals[i] << " ";
    }
    s << "\n";
  }
  const auto hint = hintFor(planner, code);
  if (!hint.empty()) {
    s << " meaning   : " << hint << "\n";
  }
  if (!note.empty()) {
    s << " note      : " << note << "\n";
  }
  s << "=========================================";
  RCLCPP_ERROR(c.node->get_logger(), "%s", s.str().c_str());
}

// ---------------------------------------------------------------------------
// TF
// ---------------------------------------------------------------------------
bool lookupFramePose(Ctx & c, const std::string & frame, geometry_msgs::msg::Pose & pose)
{
  try {
    const auto t = c.tf->lookupTransform(
      kReferenceFrame, frame, tf2::TimePointZero, tf2::durationFromSec(10.0));
    pose.position.x = t.transform.translation.x;
    pose.position.y = t.transform.translation.y;
    pose.position.z = t.transform.translation.z;
    pose.orientation = t.transform.rotation;
    return true;
  } catch (const tf2::TransformException & e) {
    reportFailure(
      c, "TF", "frame lookup", MoveItErr::FRAME_TRANSFORM_FAILURE, 0.0, nullptr,
      "'" + frame + "' -> '" + kReferenceFrame + "': " + e.what());
    return false;
  }
}

// ---------------------------------------------------------------------------
// EXECUTE : send the planned trajectory straight to this arm's controller.
// Each worker has its own node + executor thread, so waiting on futures here
// is safe (the action client callbacks are served by that executor thread).
// ---------------------------------------------------------------------------
bool executeDirect(
  Ctx & c, const moveit::planning_interface::MoveGroupInterface::Plan & plan,
  const std::string & planner, double plan_time, const geometry_msgs::msg::Pose & target)
{
  const auto & traj = plan.trajectory_.joint_trajectory;
  if (traj.points.empty()) {
    reportFailure(
      c, planner, "execution", MoveItErr::INVALID_MOTION_PLAN, plan_time, &target,
      "planned trajectory has no points.");
    return false;
  }

  if (!c.fjt->wait_for_action_server(5s)) {
    reportFailure(
      c, planner, "execution", MoveItErr::CONTROL_FAILED, plan_time, &target,
      "controller action server not available: " + c.cfg->controller_action);
    return false;
  }

  FJT::Goal goal;
  goal.trajectory = traj;

  auto goal_fut = c.fjt->async_send_goal(goal);
  if (goal_fut.wait_for(10s) != std::future_status::ready) {
    reportFailure(
      c, planner, "execution", MoveItErr::TIMED_OUT, plan_time, &target,
      "sending the goal to the controller timed out.");
    return false;
  }
  auto goal_handle = goal_fut.get();
  if (!goal_handle) {
    reportFailure(
      c, planner, "execution", MoveItErr::CONTROL_FAILED, plan_time, &target,
      "controller rejected the goal.");
    return false;
  }

  const auto & last = traj.points.back().time_from_start;
  const double duration = static_cast<double>(last.sec) + 1e-9 * static_cast<double>(last.nanosec);

  auto result_fut = c.fjt->async_get_result(goal_handle);
  if (result_fut.wait_for(std::chrono::duration<double>(duration + kExecuteMargin)) !=
    std::future_status::ready)
  {
    c.fjt->async_cancel_goal(goal_handle);
    reportFailure(
      c, planner, "execution", MoveItErr::TIMED_OUT, plan_time, &target,
      "controller did not finish within trajectory duration + margin; goal cancelled.");
    return false;
  }

  const auto result = result_fut.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    std::string note = "controller result code " + std::to_string(static_cast<int>(result.code));
    if (result.result) {
      note += ", error_code " + std::to_string(result.result->error_code);
      if (!result.result->error_string.empty()) {
        note += " (" + result.result->error_string + ")";
      }
    }
    reportFailure(c, planner, "execution", MoveItErr::CONTROL_FAILED, plan_time, &target, note);
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// MOVEJ : OMPL pose target
// ---------------------------------------------------------------------------
bool movej(Ctx & c, const geometry_msgs::msg::Pose & target)
{
  auto & mg = *c.mg;
  const auto & m = c.cfg->motion;
  const auto logger = c.node->get_logger();
  const std::string tag = stepTag(c);

  if (!mg.getCurrentState(10.0)) {
    reportFailure(c, "OMPL", "current state", MoveItErr::ROBOT_STATE_STALE, 0.0, &target);
    return false;
  }
  mg.setStartStateToCurrentState();
  mg.setPlanningPipelineId("ompl");
  mg.setPlannerId("RRTConnectkConfigDefault");
  mg.setPlanningTime(m.planning_time);
  mg.setNumPlanningAttempts(m.ompl_attempts);
  mg.setMaxVelocityScalingFactor(m.movej_vel);
  mg.setMaxAccelerationScalingFactor(m.movej_acc);
  mg.setGoalPositionTolerance(m.goal_pos_tol);
  mg.setGoalOrientationTolerance(m.goal_ori_tol);
  mg.clearPoseTargets();
  mg.clearPathConstraints();

  RCLCPP_INFO(logger, "%smovej target %s", tag.c_str(), poseStr(target).c_str());
  if (!mg.setPoseTarget(target, c.cfg->link)) {
    reportFailure(
      c, "OMPL", "set target", MoveItErr::FAILURE, 0.0, &target, "MoveIt rejected the pose target.");
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool planned = false;
  int last_code = 0;
  const auto t0 = std::chrono::steady_clock::now();
  for (int attempt = 1; attempt <= m.plan_retries; ++attempt) {
    const auto code = mg.plan(plan);
    last_code = code.val;
    if (code == moveit::core::MoveItErrorCode::SUCCESS) {
      planned = true;
      break;
    }
    RCLCPP_WARN(
      logger, "%sOMPL attempt %d/%d failed: %s", tag.c_str(), attempt, m.plan_retries,
      errStr(last_code).c_str());
  }
  const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  mg.clearPoseTargets();

  if (!planned) {
    reportFailure(
      c, "OMPL", "planning", last_code, dt, &target,
      "all " + std::to_string(m.plan_retries) + " retries x " + std::to_string(m.ompl_attempts) +
      " attempts failed.");
    return false;
  }
  if (!executeDirect(c, plan, "OMPL", dt, target)) {
    return false;
  }
  RCLCPP_INFO(logger, "%smovej done (planned in %.2f s).", tag.c_str(), dt);
  return true;
}

// ---------------------------------------------------------------------------
// MOVEL : Pilz LIN pose target
// ---------------------------------------------------------------------------
bool movel(Ctx & c, const geometry_msgs::msg::Pose & target)
{
  auto & mg = *c.mg;
  const auto & m = c.cfg->motion;
  const auto logger = c.node->get_logger();
  const std::string tag = stepTag(c);

  if (!mg.getCurrentState(10.0)) {
    reportFailure(c, "LIN", "current state", MoveItErr::ROBOT_STATE_STALE, 0.0, &target);
    return false;
  }
  mg.setStartStateToCurrentState();
  mg.setPlanningPipelineId("pilz_industrial_motion_planner");
  mg.setPlannerId("LIN");
  mg.setPlanningTime(m.planning_time);
  mg.setMaxVelocityScalingFactor(m.movel_vel);
  mg.setMaxAccelerationScalingFactor(m.movel_acc);
  mg.clearPoseTargets();
  mg.clearPathConstraints();

  RCLCPP_INFO(logger, "%smovel target %s", tag.c_str(), poseStr(target).c_str());
  if (!mg.setPoseTarget(target, c.cfg->link)) {
    reportFailure(
      c, "LIN", "set target", MoveItErr::FAILURE, 0.0, &target, "MoveIt rejected the pose target.");
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto t0 = std::chrono::steady_clock::now();
  const auto code = mg.plan(plan);
  const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  mg.clearPoseTargets();

  if (code != moveit::core::MoveItErrorCode::SUCCESS) {
    reportFailure(c, "LIN", "planning", code.val, dt, &target);
    return false;
  }
  if (!executeDirect(c, plan, "LIN", dt, target)) {
    return false;
  }
  RCLCPP_INFO(logger, "%smovel done (planned in %.2f s).", tag.c_str(), dt);
  return true;
}

// ---------------------------------------------------------------------------
// PICK_PLACE for the pair in c.cur
// ---------------------------------------------------------------------------
bool pickPlace(Ctx & c)
{
  const auto & cfg = *c.cfg;
  const auto logger = c.node->get_logger();

  RCLCPP_INFO(
    logger, "---- %s pair %d/%d: %s -> %s ----", cfg.name.c_str(),
    c.cur.pair_idx, c.cur.pair_total, c.cur.pick.c_str(), c.cur.place.c_str());

  c.cur.step_total = cfg.use_transition ? 7 : 6;
  c.cur.step_idx = 0;
  c.cur.step = "frame lookup";

  geometry_msgs::msg::Pose pick, place;
  if (!lookupFramePose(c, c.cur.pick, pick) || !lookupFramePose(c, c.cur.place, place)) {
    return false;
  }

  // fixed offsets, then the per-pair walking shift on the pick
  pick = offsetWorld(pick, cfg.pick_offset);
  pick = offsetWorld(pick, c.pick_shift);
  place = offsetWorld(place, cfg.place_offset);

  RCLCPP_INFO(
    logger, "pick shift = [%.3f, %.3f, %.3f] | final pick %s", c.pick_shift[0], c.pick_shift[1],
    c.pick_shift[2], poseStr(pick).c_str());

  auto pick_approach = offsetWorld(pick, cfg.approach_offset);
  const auto place_approach = offsetWorld(place, cfg.approach_offset);

  int n = 0;
  auto step = [&](const std::string & name) {
      c.cur.step_idx = ++n;
      c.cur.step = name;
    };

  // 1 pick approach
  step("pick approach (movej)");
  if (!movej(c, pick_approach)) {return false;}

  // 2 pick
  step("pick (movel)");
  if (!movel(c, pick)) {return false;}

  // >>> close gripper / attach object here <<<

  // 3 pick retract
  step("pick retract (movel)");
  if (!movel(c, pick_approach)) {return false;}

  // 4 optional transition
  if (cfg.use_transition) {
    auto transition = pick_approach;
    if (cfg.transition_copy_axes[0]) {transition.position.x = place_approach.position.x;}
    if (cfg.transition_copy_axes[1]) {transition.position.y = place_approach.position.y;}
    if (cfg.transition_copy_axes[2]) {transition.position.z = place_approach.position.z;}
    step("transition (movel)");
    if (!movel(c, transition)) {return false;}
  }

  // place approach
  step("place approach (movel)");
  if (!movel(c, place_approach)) {
    if (!cfg.place_approach_fallback) {return false;}
    RCLCPP_WARN(logger, "%smovel failed -> trying movej fallback", stepTag(c).c_str());
    c.cur.step = "place approach (movej fallback)";
    if (!movej(c, place_approach)) {return false;}
  }

  // place
  step("place (movel)");
  if (!movel(c, place)) {return false;}

  // >>> open gripper / detach object here <<<

  // place retract
  step("place retract (movel)");
  if (!movel(c, place_approach)) {return false;}

  RCLCPP_INFO(logger, "---- %s pair %d/%d done ----", cfg.name.c_str(), c.cur.pair_idx, c.cur.pair_total);
  return true;
}

// ---------------------------------------------------------------------------
// results / summary
// ---------------------------------------------------------------------------
enum class PairStatus { kOk, kFailed, kNotRun };

struct PairResult
{
  std::string pick, place;
  Vec3 pick_shift = {0.0, 0.0, 0.0};
  bool exclusive = false;
  PairStatus status = PairStatus::kNotRun;
  Failure failure;
};

void printSummary(const RobotConfig & cfg, const std::vector<PairResult> & results)
{
  int ok = 0, failed = 0, not_run = 0;
  int by_ompl = 0, by_lin = 0, by_other = 0;
  for (const auto & r : results) {
    if (r.status == PairStatus::kOk) {++ok;}
    else if (r.status == PairStatus::kNotRun) {++not_run;}
    else {
      ++failed;
      if (r.failure.planner == "OMPL") {++by_ompl;}
      else if (r.failure.planner == "LIN") {++by_lin;}
      else {++by_other;}
    }
  }

  std::printf("\n============ RUN SUMMARY (%s / %s) ============\n", cfg.name.c_str(), cfg.group.c_str());
  std::printf(
    "%-7s %-8s -> %-14s | %-7s | %-30s | %-5s | %-13s | %s\n",
    "pair", "pick", "place", "result", "failed step", "tool", "stage", "error");
  const int total = static_cast<int>(results.size());
  int idx = 0;
  for (const auto & r : results) {
    ++idx;
    char pair_col[16];
    std::snprintf(pair_col, sizeof(pair_col), "%d/%d", idx, total);
    if (r.status == PairStatus::kOk) {
      std::printf("%-7s %-8s -> %-14s | OK\n", pair_col, r.pick.c_str(), r.place.c_str());
    } else if (r.status == PairStatus::kNotRun) {
      std::printf("%-7s %-8s -> %-14s | NOT RUN\n", pair_col, r.pick.c_str(), r.place.c_str());
    } else {
      char step_col[48];
      std::snprintf(
        step_col, sizeof(step_col), "%d/%d %s", r.failure.info.step_idx, r.failure.info.step_total,
        r.failure.info.step.c_str());
      std::printf(
        "%-7s %-8s -> %-14s | FAILED  | %-30s | %-5s | %-13s | %s\n",
        pair_col, r.pick.c_str(), r.place.c_str(), step_col, r.failure.planner.c_str(),
        r.failure.stage.c_str(), errStr(r.failure.code).c_str());
    }
  }
  std::printf("\nPairs OK: %d / %d    FAILED: %d    NOT RUN: %d\n", ok, total, failed, not_run);
  std::printf("Failures by planner:  OMPL=%d   Pilz LIN=%d   other (TF etc.)=%d\n", by_ompl, by_lin, by_other);
  std::printf("====================================================\n");
  std::fflush(stdout);
}

// ---------------------------------------------------------------------------
// Worker: runs one robot completely
// ---------------------------------------------------------------------------
struct BarrierGuard   // makes sure the other arm never waits on a finished/dead worker
{
  explicit BarrierGuard(PairBarrier & b)
  : b_(b) {}
  ~BarrierGuard() {b_.drop();}
  PairBarrier & b_;
};

bool runRobot(const RobotConfig & cfg, Shared & shared)
{
  BarrierGuard guard(shared.barrier);

  const auto node_options =
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared(cfg.name, node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() {executor.spin();});

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf_buffer.setUsingDedicatedThread(true);
  tf2_ros::TransformListener tf_listener(tf_buffer, node, false);

  bool ok = false;
  try {
    ok = [&]() -> bool
      {
        RCLCPP_INFO(node->get_logger(), "Waiting for MoveIt, joint states and TF.");
        rclcpp::sleep_for(2s);

        moveit::planning_interface::MoveGroupInterface move_group(node, cfg.group);
        move_group.setPoseReferenceFrame(kReferenceFrame);
        move_group.setEndEffectorLink(cfg.link);
        move_group.setPlanningTime(cfg.motion.planning_time);

        if (!move_group.getCurrentState(10.0)) {
          RCLCPP_ERROR(node->get_logger(), "Current robot state is unavailable.");
          return false;
        }
        RCLCPP_INFO(
          node->get_logger(), "Group '%s', planning link '%s', reference '%s'.",
          cfg.group.c_str(), cfg.link.c_str(), kReferenceFrame);

        Ctx ctx;
        ctx.cfg = &cfg;
        ctx.node = node;
        ctx.mg = &move_group;
        ctx.tf = &tf_buffer;
        ctx.fjt = rclcpp_action::create_client<FJT>(node, cfg.controller_action);

        if (!ctx.fjt->wait_for_action_server(10s)) {
          RCLCPP_ERROR(
            node->get_logger(), "Controller action server '%s' not available.",
            cfg.controller_action.c_str());
          return false;
        }
        RCLCPP_INFO(
          node->get_logger(), "Executing directly on '%s'.", cfg.controller_action.c_str());

        // flat list of pairs
        std::vector<PairResult> results;
        for (const auto & job : cfg.jobs) {
          for (size_t j = 0; j < job.places.size(); ++j) {
            PairResult r;
            r.pick = job.pick;
            r.place = job.places[j];
            for (int k = 0; k < 3; ++k) {
              r.pick_shift[k] = static_cast<double>(j) * job.pick_step[k];
            }
            r.exclusive = job.exclusive;
            results.push_back(r);
          }
        }
        const int total = static_cast<int>(results.size());

        bool stop = false;
        for (int i = 0; i < total && !stop; ++i) {
          if (kLockstepPairs) {
            RCLCPP_INFO(node->get_logger(), "Waiting at barrier before pair %d/%d.", i + 1, total);
            shared.barrier.arriveAndWait();
          }

          auto & r = results[i];
          ctx.cur = StepInfo{};
          ctx.cur.pair_idx = i + 1;
          ctx.cur.pair_total = total;
          ctx.cur.pick = r.pick;
          ctx.cur.place = r.place;
          ctx.pick_shift = r.pick_shift;

          std::unique_lock<std::mutex> zone(shared.zone, std::defer_lock);
          if (r.exclusive) {
            RCLCPP_INFO(node->get_logger(), "Pair %d needs the shared zone, waiting for it.", i + 1);
            zone.lock();
          }

          const size_t failures_before = ctx.failures.size();
          if (pickPlace(ctx)) {
            r.status = PairStatus::kOk;
          } else {
            r.status = PairStatus::kFailed;
            if (ctx.failures.size() > failures_before) {
              r.failure = ctx.failures.back();
            }
            if (kStopOnFirstFailure) {
              stop = true;
              RCLCPP_ERROR(
                node->get_logger(), "Stopping: pair %d/%d (%s -> %s) failed.",
                i + 1, total, r.pick.c_str(), r.place.c_str());
            }
          }
          if (zone.owns_lock()) {zone.unlock();}
        }

        {
          std::lock_guard<std::mutex> lk(shared.print);
          printSummary(cfg, results);
        }

        for (const auto & r : results) {
          if (r.status != PairStatus::kOk) {return false;}
        }
        RCLCPP_INFO(node->get_logger(), "All %d pick/place pairs completed.", total);
        return true;
      }();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Unhandled exception: %s", e.what());
  } catch (...) {
    RCLCPP_FATAL(node->get_logger(), "Unhandled non-standard exception.");
  }

  (void)tf_listener;
  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  return ok;
}

}  // namespace

// ===========================================================================
// main
// ===========================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const RobotConfig robot1 = makeRobot1();
  const RobotConfig robot2 = makeRobot2();
  Shared shared(2);

  bool ok1 = false, ok2 = false;
  std::thread t1([&]() {ok1 = runRobot(robot1, shared);});
  std::thread t2([&]() {ok2 = runRobot(robot2, shared);});
  t1.join();
  t2.join();

  rclcpp::shutdown();
  return (ok1 && ok2) ? 0 : 1;
}
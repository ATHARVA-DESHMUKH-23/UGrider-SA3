// ik_reach_test.cpp  (sa3_motion)  -- v2
// Plan-only reachability test for robot1_arm:
//   5pf  -> place_frame_1
//   13pf -> place_frame_2..5
//   14pf -> place_frame_5..9
// For every frame:  collision-aware IK (/compute_ik) with random-seed retries
//                   + distance-from-base diagnostic
// For every pair :  OMPL plan  (IK(pick) -> IK(place)).
// Execution is in a marked block inside the loop ("EXECUTION"); comment it out for plan-only.

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using MoveItErr = moveit_msgs::msg::MoveItErrorCodes;
using GetIK = moveit_msgs::srv::GetPositionIK;

static const std::string GROUP = "robot1_arm";
static const std::string EE_LINK = "robot1_end_effactor";  // (sic) matches SRDF
static const std::string BASE_LINK = "robot1_base_link";
static const int IK_ATTEMPTS = 50;        // attempt 0 = current state, rest = random seeds
static const double IK_TIMEOUT = 0.2;     // seconds per attempt

struct Job
{
  std::string pick;
  std::vector<std::string> places;
};

static const std::vector<Job> JOBS = {
  {"5pf", {"place_frame_1"}},
  {"13pf", {"place_frame_2", "place_frame_3", "place_frame_4", "place_frame_5"}},
  {"14pf", {"place_frame_5", "place_frame_6", "place_frame_7", "place_frame_8", "place_frame_9"}},
};

struct IkResult
{
  bool ok = false;
  int code = 0;
  int attempts = 0;                          // attempts used
  double dist = -1.0;                        // base -> target distance
  geometry_msgs::msg::PoseStamped pose;
  sensor_msgs::msg::JointState js;           // full solution
  std::vector<double> group_vals;            // only robot1_arm joints, in group order
};

static std::string errStr(int c)
{
  switch (c)
  {
    case MoveItErr::SUCCESS: return "SUCCESS";
    case MoveItErr::FAILURE: return "FAILURE";
    case MoveItErr::PLANNING_FAILED: return "PLANNING_FAILED";
    case MoveItErr::INVALID_MOTION_PLAN: return "INVALID_MOTION_PLAN";
    case MoveItErr::TIMED_OUT: return "TIMED_OUT";
    case MoveItErr::START_STATE_IN_COLLISION: return "START_STATE_IN_COLLISION";
    case MoveItErr::GOAL_IN_COLLISION: return "GOAL_IN_COLLISION";
    case MoveItErr::NO_IK_SOLUTION: return "NO_IK_SOLUTION";
    case MoveItErr::GOAL_VIOLATES_PATH_CONSTRAINTS: return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case MoveItErr::INVALID_GROUP_NAME: return "INVALID_GROUP_NAME";
    case MoveItErr::FRAME_TRANSFORM_FAILURE: return "FRAME_TRANSFORM_FAILURE";
    default: return "CODE_" + std::to_string(c);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("ik_reach_test", opts);
  auto log = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  // ---- TF ----
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // ---- MoveIt ----
  moveit::planning_interface::MoveGroupInterface mg(node, GROUP);
  mg.setPlanningPipelineId("ompl");
  mg.setPlannerId("RRTConnectkConfigDefault");
  mg.setPlanningTime(5.0);
  mg.setNumPlanningAttempts(5);
  mg.setMaxVelocityScalingFactor(0.5);
  mg.setMaxAccelerationScalingFactor(0.5);
  mg.setEndEffectorLink(EE_LINK);

  const std::string planning_frame = mg.getPlanningFrame();
  const std::vector<std::string> group_joints = mg.getActiveJoints();
  RCLCPP_INFO(log, "Group: %s | planning frame: %s | EE link: %s | joints: %zu",
              GROUP.c_str(), planning_frame.c_str(), EE_LINK.c_str(), group_joints.size());

  auto robot_model = mg.getRobotModel();
  const auto* jmg = robot_model->getJointModelGroup(GROUP);

  auto ik_client = node->create_client<GetIK>("/compute_ik");
  if (!ik_client->wait_for_service(10s))
  {
    RCLCPP_ERROR(log, "/compute_ik not available. Is move_group running?");
    exec.cancel(); spinner.join(); rclcpp::shutdown();
    return 1;
  }

  // ---- robot1 base position (for reach diagnostic) ----
  bool have_base = false;
  geometry_msgs::msg::Vector3 base_pos;
  try
  {
    auto b = tf_buffer.lookupTransform(planning_frame, BASE_LINK, tf2::TimePointZero, 5s);
    base_pos = b.transform.translation;
    have_base = true;
    std::printf("robot1 base (%s) in %s: (%.3f, %.3f, %.3f)\n", BASE_LINK.c_str(),
                planning_frame.c_str(), base_pos.x, base_pos.y, base_pos.z);
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(log, "Could not look up %s: %s (reach diagnostic disabled)", BASE_LINK.c_str(), e.what());
  }

  // ---- seed state ----
  moveit::core::RobotState seed(robot_model);
  {
    auto cur = mg.getCurrentState(3.0);
    if (cur) seed = *cur; else seed.setToDefaultValues();
  }
  const moveit::core::RobotState initial_seed = seed;

  // ---- IK cache (place_frame_5 is used twice) ----
  std::map<std::string, IkResult> ik_cache;

  auto solveIk = [&](const std::string& frame) -> IkResult&
  {
    auto it = ik_cache.find(frame);
    if (it != ik_cache.end()) return it->second;

    IkResult r;
    try
    {
      auto tf = tf_buffer.lookupTransform(planning_frame, frame, tf2::TimePointZero, 5s);
      r.pose.header.frame_id = planning_frame;
      r.pose.pose.position.x = tf.transform.translation.x;
      r.pose.pose.position.y = tf.transform.translation.y;
      r.pose.pose.position.z = tf.transform.translation.z;
      r.pose.pose.orientation = tf.transform.rotation;
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(log, "TF lookup %s -> %s failed: %s", planning_frame.c_str(), frame.c_str(), e.what());
      r.code = MoveItErr::FRAME_TRANSFORM_FAILURE;
      return ik_cache[frame] = r;
    }

    if (have_base)
    {
      double dx = r.pose.pose.position.x - base_pos.x;
      double dy = r.pose.pose.position.y - base_pos.y;
      double dz = r.pose.pose.position.z - base_pos.z;
      r.dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    moveit::core::RobotState s = initial_seed;
    for (int a = 0; a < IK_ATTEMPTS && !r.ok; ++a)
    {
      if (a > 0) s.setToRandomPositions(jmg);  // attempt 0 = current state
      r.attempts = a + 1;

      auto req = std::make_shared<GetIK::Request>();
      req->ik_request.group_name = GROUP;
      req->ik_request.ik_link_name = EE_LINK;
      req->ik_request.pose_stamped = r.pose;
      req->ik_request.avoid_collisions = true;
      req->ik_request.timeout = rclcpp::Duration::from_seconds(IK_TIMEOUT);
      moveit::core::robotStateToRobotStateMsg(s, req->ik_request.robot_state);

      auto fut = ik_client->async_send_request(req);
      if (fut.wait_for(5s) != std::future_status::ready)
      {
        r.code = MoveItErr::TIMED_OUT;
        continue;
      }
      auto res = fut.get();
      r.code = res->error_code.val;
      if (r.code == MoveItErr::SUCCESS)
      {
        r.ok = true;
        r.js = res->solution.joint_state;
        for (const auto& j : group_joints)
          for (size_t i = 0; i < r.js.name.size(); ++i)
            if (r.js.name[i] == j) { r.group_vals.push_back(r.js.position[i]); break; }
      }
    }
    return ik_cache[frame] = r;
  };

  auto printIk = [&](const std::string& frame, const IkResult& r)
  {
    const auto& p = r.pose.pose;
    std::printf("  [IK] %-14s xyz=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f) -> %s (attempts: %d/%d)\n",
                frame.c_str(), p.position.x, p.position.y, p.position.z,
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
                errStr(r.code).c_str(), r.attempts, IK_ATTEMPTS);
    if (r.dist >= 0.0)
    {
      double dx = p.position.x - base_pos.x;
      double dy = p.position.y - base_pos.y;
      double dz = p.position.z - base_pos.z;
      std::printf("       reach: dist from base %.2f m (horizontal %.2f, dz %.2f)\n",
                  r.dist, std::sqrt(dx * dx + dy * dy), dz);
    }
    if (r.ok)
    {
      std::printf("       joints: ");
      for (size_t i = 0; i < group_joints.size(); ++i)
        std::printf("%s=%.3f ", group_joints[i].c_str(), r.group_vals[i]);
      std::printf("\n");
    }
  };

  // ---- simple "plan from current state to joint goal and execute" helper ----
  // Only used by the EXECUTION block in the loop below.
  auto moveTo = [&](const std::vector<double>& goal, const std::string& label) -> bool
  {
    mg.setStartStateToCurrentState();
    mg.setJointValueTarget(goal);
    moveit::planning_interface::MoveGroupInterface::Plan p;
    if (mg.plan(p) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      std::printf("  [EXEC] %s: plan from current state FAILED\n", label.c_str());
      return false;
    }
    bool ok = (mg.execute(p) == moveit::core::MoveItErrorCode::SUCCESS);
    std::printf("  [EXEC] %s: %s\n", label.c_str(), ok ? "done" : "execution FAILED");
    return ok;
  };

  // ---- results table ----
  struct Row { std::string pick, place; bool ik_pick, ik_place, plan; int code; size_t pts; double t, len; };
  std::vector<Row> rows;

  for (const auto& job : JOBS)
  {
    std::printf("\n=== PICK %s ===\n", job.pick.c_str());
    const IkResult pick_ik = solveIk(job.pick);
    printIk(job.pick, pick_ik);

    for (const auto& place : job.places)
    {
      std::printf("\n--- %s -> %s ---\n", job.pick.c_str(), place.c_str());
      const IkResult place_ik = solveIk(place);
      printIk(place, place_ik);

      Row row{job.pick, place, pick_ik.ok, place_ik.ok, false, 0, 0, 0.0, 0.0};

      if (!pick_ik.ok || !place_ik.ok)
      {
        std::printf("  [OMPL] skipped (IK failed)\n");
        rows.push_back(row);
        continue;
      }

      moveit_msgs::msg::RobotState start;
      start.joint_state = pick_ik.js;
      start.is_diff = true;
      mg.setStartState(start);
      mg.setJointValueTarget(place_ik.group_vals);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto code = mg.plan(plan);  // plan only, never execute
      row.code = code.val;
      row.plan = (code == moveit::core::MoveItErrorCode::SUCCESS);
      row.t = plan.planning_time_;

      if (row.plan)
      {
        const auto& pts = plan.trajectory_.joint_trajectory.points;
        row.pts = pts.size();
        for (size_t i = 1; i < pts.size(); ++i)
        {
          double s = 0;
          for (size_t j = 0; j < pts[i].positions.size(); ++j)
            s += std::pow(pts[i].positions[j] - pts[i - 1].positions[j], 2);
          row.len += std::sqrt(s);
        }
        const auto& last = pts.back().time_from_start;
        std::printf("  [OMPL] SUCCESS | planning %.3fs | %zu waypoints | joint path len %.3f rad | duration %.2fs\n",
                    row.t, row.pts, row.len, last.sec + last.nanosec * 1e-9);
      }
      else
      {
        std::printf("  [OMPL] FAILED  | %s | planning %.3fs\n", errStr(row.code).c_str(), row.t);
      }
      // ================= EXECUTION (comment out this block for plan-only) =================
      // Current pose -> pick, then pick -> place. Stops this pair if a move fails.
      if (row.plan)
      {
        if (moveTo(pick_ik.group_vals, "-> " + job.pick))
          moveTo(place_ik.group_vals, "-> " + place);
      }
      // ================= END EXECUTION =================

      rows.push_back(row);
    }
  }

  // ---- summary ----
  std::printf("\n================ SUMMARY (robot1_arm) ================\n");
  std::printf("%-6s -> %-14s | IK pick | IK place | OMPL  | pts | t[s]  | len[rad]\n", "pick", "place");
  int ok = 0;
  for (const auto& r : rows)
  {
    std::printf("%-6s -> %-14s | %-7s | %-8s | %-5s | %3zu | %5.2f | %6.2f\n",
                r.pick.c_str(), r.place.c_str(),
                r.ik_pick ? "OK" : "FAIL", r.ik_place ? "OK" : "FAIL",
                r.plan ? "OK" : "FAIL", r.pts, r.t, r.len);
    if (r.plan) ++ok;
  }
  std::printf("Reachable & plannable: %d / %zu\n", ok, rows.size());

  exec.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
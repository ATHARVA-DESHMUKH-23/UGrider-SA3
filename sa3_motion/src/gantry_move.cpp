#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <future>
#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "builtin_interfaces/msg/duration.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/conversions.h"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "sa3_motion/planning_scene_objects.hpp"

using namespace std::chrono_literals;

class GantryMotion final : public rclcpp::Node
{
public:
  using FollowTrajectory =
    control_msgs::action::FollowJointTrajectory;
  using GoalHandle =
    rclcpp_action::ClientGoalHandle<FollowTrajectory>;

  explicit GantryMotion(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("gantry_move", options)
  {
    // parameterOrDeclare() is used for every parameter because main() enables
    // automatically_declare_parameters_from_overrides(true). Plain
    // declare_parameter() would throw ParameterAlreadyDeclared for any value
    // that the params file sets.
    controller_action_ = parameterOrDeclare<std::string>(
      "controller_action",
      "/gantry_controller/follow_joint_trajectory");

    horizontal_joint_ = parameterOrDeclare<std::string>(
      "horizontal_joint", "gantry_slide_joint");

    vertical_joint_ = parameterOrDeclare<std::string>(
      "vertical_joint", "gantry_down_joint");

    object_id_ = parameterOrDeclare<std::string>(
      "rebar_object_id", "rebar_1A");

    attach_link_ = parameterOrDeclare<std::string>(
      "attach_link", "gantry_part_link");

    move_group_name_ = parameterOrDeclare<std::string>(
      "move_group_name", "robot3_arm");

    horizontal_home_ = parameterOrDeclare<double>("horizontal_home_m", 0.0);
    vertical_home_ = parameterOrDeclare<double>("vertical_home_m", 0.0);
    robot3_group_name_ = parameterOrDeclare<std::string>("robot3_group", "robot3_arm");
    robot3_link_ = parameterOrDeclare<std::string>("robot3_planning_link", "robot3_end_effactor");
    rebar_id_ = parameterOrDeclare<std::string>("robot_rebar_object_id", "rebar_1A");
    pick3_frame_ = parameterOrDeclare<std::string>("robot3_pick_frame", "bm_pkL");
    place3_frame_ = parameterOrDeclare<std::string>("robot3_place_frame", "bm_pfl");
    // Collision object / link the carried rebar is allowed to touch while it is
    // being placed. Set to an empty string to disable the allowance.
    jig_clamp_name_ = parameterOrDeclare<std::string>("jig_clamp_name", "jig_clamp");
    // Optional: collision objects the carried rebar may touch during transport
    // (for example the inventory rack it was resting on). Empty by default;
    // every entry removes collision protection against that object.
    rebar_contact_objects_ = parameterOrDeclare<std::vector<std::string>>(
      "rebar_allowed_contact_objects", std::vector<std::string>{});
    approach_height_ = parameterOrDeclare<double>("approach_height_m", 0.60);
    // Robot 3 stops this far in front of the place frame before the final MoveL.
    place_approach_distance_ = parameterOrDeclare<double>("place_approach_distance_m", 0.10);
    // Pick approach defaults to a vertical lift in the world frame.
    // The selected pick approach pose is also used as the post-pick retract pose.
    pick_approach_dx_ = parameterOrDeclare<double>("pick_approach_dx_m", 0.0);
    pick_approach_dz_ = parameterOrDeclare<double>("pick_approach_dz_m", 0.60);

    // Gantry first picks rebar_1A at the inventory position, then transports
    // it to the existing hand-off position for the unchanged Robot 3 sequence.
    horizontal_pick_target_ = parameterOrDeclare<double>(
      "horizontal_pick_target_m", 0.0);
    vertical_pick_target_ = parameterOrDeclare<double>(
      "vertical_pick_target_m", 4.1);

    horizontal_target_ = parameterOrDeclare<double>(
      "horizontal_target_m", -6.0);
    vertical_target_ = parameterOrDeclare<double>(
      "down_target_m", 1.2);

    speed_ = parameterOrDeclare<double>("speed_mps", 0.30);
    acceleration_ = parameterOrDeclare<double>(
      "acceleration_mps2", 0.50);

    horizontal_min_ = parameterOrDeclare<double>(
      "horizontal_min_m", -10.6);
    horizontal_limit_ = parameterOrDeclare<double>(
      "horizontal_limit_m", 10.6);

    vertical_limit_ = parameterOrDeclare<double>(
      "vertical_limit_m", 6.6);

    settle_time_sec_ = parameterOrDeclare<double>(
      "settle_time_sec", 0.5);

    trajectory_timeout_sec_ = parameterOrDeclare<double>(
      "trajectory_timeout_sec", 60.0);

    // Auto-start: run the sequence once, by itself, after startup.
    auto_start_ = parameterOrDeclare<bool>("auto_start", true);

    start_delay_sec_ = parameterOrDeclare<double>(
      "start_delay_sec", 3.0);

    ready_timeout_sec_ = parameterOrDeclare<double>(
      "ready_timeout_sec", 90.0);

    validateParameters();

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::QoS(20),
      std::bind(
        &GantryMotion::jointStateCallback,
        this,
        std::placeholders::_1));

    trajectory_client_ =
      rclcpp_action::create_client<FollowTrajectory>(
        this, controller_action_);

    ik_client_ = create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");

    start_service_ = create_service<std_srvs::srv::Trigger>(
      "/start_gantry_motion",
      std::bind(
        &GantryMotion::startMotion,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Gantry motion node ready. Start service: /start_gantry_motion");

    RCLCPP_INFO(
      get_logger(),
      "Trajectory action: %s",
      controller_action_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Sequence: gantry to pick X=%.3f m, lower to Z-joint=%.3f m, "
      "attach '%s', raise, move to X=%.3f m, lower to %.3f m, detach, "
      "return gantry to raised start position, then start Robot 3 pick/place.",
      horizontal_pick_target_, vertical_pick_target_, object_id_.c_str(),
      horizontal_target_, vertical_target_);
  }

  ~GantryMotion() override
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  // Called from main() once MoveIt is initialized. Starts the sequence
  // automatically (once) unless the auto_start parameter is false.
  bool autoStartIfEnabled()
  {
    if (!auto_start_) {
      RCLCPP_INFO(
        get_logger(),
        "auto_start is false. Waiting for /start_gantry_motion to run the sequence.");
      return true;
    }

    std::string message;
    if (launchSequence(true, message)) {
      RCLCPP_INFO(
        get_logger(),
        "Auto-start enabled: sequence begins in %.1f s once the "
        "controller and '%s' are ready.",
        start_delay_sec_, object_id_.c_str());
      return true;
    }

    RCLCPP_ERROR(get_logger(), "Auto-start could not start: %s", message.c_str());
    return false;
  }

  // Blocks until the one-shot sequence (auto-started or requested through the
  // service) finishes. The process can then return exit code 0 for success
  // and 1 for failure, allowing ros2 launch to detect the result.
  bool waitForSequenceResult()
  {
    std::unique_lock<std::mutex> lock(sequence_mutex_);
    while (!sequence_finished_ && rclcpp::ok()) {
      sequence_cv_.wait_for(lock, 100ms);
    }

    if (!sequence_finished_) {
      RCLCPP_ERROR(get_logger(), "ROS shutdown occurred before the sequence completed.");
      return false;
    }

    return sequence_success_;
  }

  void initializeMoveIt(
    moveit::planning_interface::MoveGroupInterface * move_group,
    moveit::planning_interface::MoveGroupInterface * robot3_group,
    PlanningSceneObjects * scene_objects,
    tf2_ros::Buffer * tf_buffer)
  {
    if (move_group == nullptr || robot3_group == nullptr ||
        scene_objects == nullptr || tf_buffer == nullptr) {
      throw std::invalid_argument("Both MoveGroup interfaces, planning scene and TF buffer are required.");
    }
    move_group_ = move_group;
    robot3_group_ = robot3_group;
    scene_objects_ = scene_objects;
    tf_buffer_ = tf_buffer;
    robot3_group_->setPoseReferenceFrame("world");
    robot3_group_->setEndEffectorLink(robot3_link_);
    if (!captureHomeState(*robot3_group_, robot3_home_)) {
      throw std::runtime_error("Could not capture initial Robot 3 home state. Start Robot 3 in its intended home position.");
    }
  }

private:
  rclcpp_action::Client<FollowTrajectory>::SharedPtr trajectory_client_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;

  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;

  moveit::planning_interface::MoveGroupInterface * move_group_{nullptr};
  moveit::planning_interface::MoveGroupInterface * robot3_group_{nullptr};
  PlanningSceneObjects * scene_objects_{nullptr};
  tf2_ros::Buffer * tf_buffer_{nullptr};
  std::vector<double> robot3_home_;

  std::mutex joint_state_mutex_;
  std::unordered_map<std::string, double> joint_positions_;

  // launch_mutex_ serializes launch attempts. Never join worker_ while holding
  // sequence_mutex_: the worker locks sequence_mutex_ before it exits.
  std::mutex launch_mutex_;
  std::mutex sequence_mutex_;
  std::condition_variable sequence_cv_;
  bool sequence_running_{false};
  bool sequence_finished_{false};
  bool sequence_success_{false};

  std::string controller_action_;
  std::string horizontal_joint_;
  std::string vertical_joint_;
  std::string object_id_;
  std::string attach_link_;
  std::string move_group_name_;
  std::string robot3_group_name_;
  std::string robot3_link_, rebar_id_;
  std::string pick3_frame_, place3_frame_;
  std::string jig_clamp_name_;
  std::vector<std::string> rebar_contact_objects_;
  double horizontal_home_{0.0}, vertical_home_{0.0}, approach_height_{0.60};
  double place_approach_distance_{0.10};
  double pick_approach_dx_{-0.60}, pick_approach_dz_{0.0};

  double horizontal_pick_target_{10.0};
  double vertical_pick_target_{4.1};
  double horizontal_target_{4.0};
  double vertical_target_{3.2};
  double horizontal_min_{-10.6};
  double horizontal_limit_{10.6};
  double vertical_limit_{6.6};
  double speed_{0.20};
  double acceleration_{0.20};
  double settle_time_sec_{0.5};
  double trajectory_timeout_sec_{60.0};

  bool auto_start_{true};
  double start_delay_sec_{3.0};
  double ready_timeout_sec_{90.0};

  std::thread worker_;

  template<typename T>
  T parameterOrDeclare(
    const std::string & name,
    const T & default_value)
  {
    if (has_parameter(name)) {
      T value{};
      if (!get_parameter(name, value)) {
        throw std::runtime_error(
          "Could not read parameter: " + name);
      }
      return value;
    }

    return declare_parameter<T>(name, default_value);
  }

  void validateParameters()
  {
    if (controller_action_.empty() ||
        horizontal_joint_.empty() ||
        vertical_joint_.empty() ||
        object_id_.empty() ||
        attach_link_.empty() ||
        move_group_name_.empty() || robot3_group_name_.empty() ||
        robot3_link_.empty() ||
        rebar_id_.empty() || pick3_frame_.empty() ||
        place3_frame_.empty()) {
      throw std::invalid_argument(
        "Joint, controller, object, link and group names must not be empty.");
    }

    if (!std::isfinite(horizontal_pick_target_) ||
        !std::isfinite(horizontal_target_) ||
        !std::isfinite(vertical_pick_target_) ||
        vertical_pick_target_ < 0.0 ||
        !std::isfinite(vertical_target_) ||
        vertical_target_ < 0.0 ||
        !std::isfinite(horizontal_home_) ||
        !std::isfinite(vertical_home_)) {
      throw std::invalid_argument(
        "Gantry targets and home positions must be finite; vertical positions must be non-negative.");
    }

    if (!std::isfinite(horizontal_min_) ||
        !std::isfinite(horizontal_limit_) ||
        horizontal_min_ >= horizontal_limit_ ||
        !std::isfinite(vertical_limit_) ||
        vertical_limit_ <= 0.0) {
      throw std::invalid_argument(
        "Gantry limits must be finite and valid (horizontal_min < horizontal_limit; vertical_limit > 0).");
    }

    if (horizontal_pick_target_ < horizontal_min_ ||
        horizontal_pick_target_ > horizontal_limit_ ||
        horizontal_target_ < horizontal_min_ ||
        horizontal_target_ > horizontal_limit_ ||
        horizontal_home_ < horizontal_min_ ||
        horizontal_home_ > horizontal_limit_ ||
        vertical_pick_target_ > vertical_limit_ ||
        vertical_target_ > vertical_limit_ ||
        vertical_home_ < 0.0 || vertical_home_ > vertical_limit_) {
      throw std::invalid_argument(
        "Requested target exceeds configured gantry travel limits. Verify the URDF and ros2_control limits before changing these parameters.");
    }

    if (!std::isfinite(speed_) || speed_ <= 0.0 ||
        !std::isfinite(acceleration_) || acceleration_ <= 0.0) {
      throw std::invalid_argument(
        "Speed and acceleration must be finite and positive.");
    }

    if (!std::isfinite(settle_time_sec_) ||
        settle_time_sec_ < 0.0 ||
        !std::isfinite(trajectory_timeout_sec_) ||
        trajectory_timeout_sec_ <= 0.0) {
      throw std::invalid_argument(
        "Settle time must be non-negative and trajectory timeout positive.");
    }

    if (!std::isfinite(approach_height_) || approach_height_ <= 0.0) {
      throw std::invalid_argument("approach_height_m must be finite and positive.");
    }
    if (!std::isfinite(place_approach_distance_) || place_approach_distance_ <= 0.0) {
      throw std::invalid_argument("place_approach_distance_m must be finite and positive.");
    }

    if (horizontal_joint_ == vertical_joint_) {
      throw std::invalid_argument(
        "Horizontal and vertical joint names must be different.");
    }

    if (!std::isfinite(start_delay_sec_) || start_delay_sec_ < 0.0 ||
        !std::isfinite(ready_timeout_sec_) || ready_timeout_sec_ <= 0.0) {
      throw std::invalid_argument(
        "start_delay_sec must be non-negative and ready_timeout_sec positive.");
    }
  }

  void jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);

    const std::size_t count =
      std::min(msg->name.size(), msg->position.size());

    for (std::size_t i = 0; i < count; ++i) {
      if (std::isfinite(msg->position[i])) {
        joint_positions_[msg->name[i]] = msg->position[i];
      }
    }
  }

  bool getJointPosition(
    const std::string & joint_name,
    double & position)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);

    const auto it = joint_positions_.find(joint_name);
    if (it == joint_positions_.end()) {
      return false;
    }

    position = it->second;
    return true;
  }

  static builtin_interfaces::msg::Duration durationMessage(
    double seconds)
  {
    if (!std::isfinite(seconds) || seconds < 0.0) {
      throw std::invalid_argument(
        "Duration must be finite and non-negative.");
    }

    builtin_interfaces::msg::Duration msg;

    const double whole_seconds = std::floor(seconds);

    msg.sec = static_cast<std::int32_t>(whole_seconds);
    msg.nanosec = static_cast<std::uint32_t>(
      std::llround(
        (seconds - whole_seconds) * 1'000'000'000.0));

    if (msg.nanosec >= 1'000'000'000U) {
      ++msg.sec;
      msg.nanosec = 0U;
    }

    return msg;
  }

  bool waitForJointState(
    const std::string & joint_name,
    double & position,
    double timeout_sec = 5.0)
  {
    const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_sec);

    while (rclcpp::ok() &&
           std::chrono::steady_clock::now() < deadline) {
      if (getJointPosition(joint_name, position)) {
        return true;
      }

      std::this_thread::sleep_for(50ms);
    }

    RCLCPP_ERROR(
      get_logger(),
      "No joint state received for '%s'.",
      joint_name.c_str());

    return false;
  }

  // Sends ONE goal that lists BOTH gantry joints, because gantry_controller
  // is configured with both joints and allow_partial_joints_goal=false.
  // The moving joint goes to its target; the other joint holds its position.
  bool executeTrajectory(
    const std::string & moving_joint,
    double target_position,
    double current_position,
    const std::string & holding_joint,
    double holding_position)
  {
    if (!trajectory_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(
        get_logger(),
        "Action server '%s' is unavailable.",
        controller_action_.c_str());
      return false;
    }

    const double distance =
      std::abs(target_position - current_position);

    if (distance < 1e-6) {
      RCLCPP_INFO(
        get_logger(),
        "Joint '%s' is already at target %.3f m.",
        moving_joint.c_str(), target_position);
      return true;
    }

    const double minimum_duration = distance / speed_;
    const double acceleration_duration =
      2.0 * std::sqrt(distance / acceleration_);

    const double duration = std::max(
      {minimum_duration, acceleration_duration, 1.0});

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = {moving_joint, holding_joint};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {target_position, holding_position};
    point.velocities = {0.0, 0.0};
    point.accelerations = {0.0, 0.0};
    point.time_from_start = durationMessage(duration);

    trajectory.points.push_back(point);

    FollowTrajectory::Goal goal;
    goal.trajectory = trajectory;
    goal.goal_time_tolerance = durationMessage(3.0);

    RCLCPP_INFO(
      get_logger(),
      "Sending trajectory: %s from %.3f m to %.3f m "
      "(duration %.2f s), holding %s at %.3f m.",
      moving_joint.c_str(),
      current_position,
      target_position,
      duration,
      holding_joint.c_str(),
      holding_position);

    auto send_goal_options =
      rclcpp_action::Client<FollowTrajectory>::SendGoalOptions();

    auto goal_handle_future =
      trajectory_client_->async_send_goal(goal, send_goal_options);

    const auto goal_deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration<double>(10.0);

    while (rclcpp::ok() &&
           goal_handle_future.wait_for(50ms) !=
             std::future_status::ready) {
      if (std::chrono::steady_clock::now() >= goal_deadline) {
        RCLCPP_ERROR(get_logger(), "Timed out waiting for trajectory goal acceptance.");
        return false;
      }
    }

    if (!rclcpp::ok()) {
      return false;
    }

    auto goal_handle = goal_handle_future.get();

    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Trajectory goal was rejected.");
      return false;
    }

    auto result_future =
      trajectory_client_->async_get_result(goal_handle);

    const auto result_deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration<double>(
        duration + trajectory_timeout_sec_);

    while (rclcpp::ok() &&
           result_future.wait_for(100ms) !=
             std::future_status::ready) {
      if (std::chrono::steady_clock::now() >= result_deadline) {
        RCLCPP_ERROR(
          get_logger(),
          "Timed out waiting for trajectory completion.");
        return false;
      }
    }

    if (!rclcpp::ok()) {
      return false;
    }

    const auto wrapped_result = result_future.get();

    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
        !wrapped_result.result ||
        wrapped_result.result->error_code !=
          FollowTrajectory::Result::SUCCESSFUL) {
      RCLCPP_ERROR(
        get_logger(),
        "Trajectory execution failed for joint '%s'.",
        moving_joint.c_str());
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Trajectory completed for '%s'.",
      moving_joint.c_str());

    return true;
  }

  bool moveJointTo(
    const std::string & joint_name,
    double target_position)
  {
    const std::string & holding_joint =
      (joint_name == horizontal_joint_) ? vertical_joint_ : horizontal_joint_;

    double current_position = 0.0;
    double holding_position = 0.0;

    if (!waitForJointState(joint_name, current_position) ||
        !waitForJointState(holding_joint, holding_position)) {
      return false;
    }

    return executeTrajectory(
      joint_name, target_position, current_position,
      holding_joint, holding_position);
  }

  using MoveGroup = moveit::planning_interface::MoveGroupInterface;

  bool captureHomeState(MoveGroup & group, std::vector<double> & home)
  {
    auto state = group.getCurrentState(10.0);
    if (!state) {
      RCLCPP_ERROR(get_logger(), "Could not get current state for group '%s'.", group.getName().c_str());
      return false;
    }
    const auto * jmg = state->getJointModelGroup(group.getName());
    if (jmg == nullptr) {
      RCLCPP_ERROR(get_logger(), "Joint model group '%s' was not found.", group.getName().c_str());
      return false;
    }
    state->copyJointGroupPositions(jmg, home);
    return !home.empty();
  }

  bool lookupPose(const std::string & frame, geometry_msgs::msg::Pose & pose)
  {
    try {
      const auto t = tf_buffer_->lookupTransform("world", frame, tf2::TimePointZero, tf2::durationFromSec(8.0));
      pose.position.x = t.transform.translation.x;
      pose.position.y = t.transform.translation.y;
      pose.position.z = t.transform.translation.z;
      pose.orientation = t.transform.rotation;
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_ERROR(get_logger(), "TF lookup world -> '%s' failed: %s", frame.c_str(), e.what());
      return false;
    }
  }

  geometry_msgs::msg::Pose elevatedPose(const geometry_msgs::msg::Pose & source) const
  {
    auto p = source;
    p.position.z += approach_height_;
    return p;
  }

  bool planAndExecutePose(
    MoveGroup & group,
    const geometry_msgs::msg::Pose & pose,
    bool linear,
    const std::string & label,
    double linear_velocity_scaling = 0.20)
  {
    group.setStartStateToCurrentState();
    group.setPoseReferenceFrame("world");
    group.setPlanningTime(15.0);
    // Pilz LIN can fail when the IK solution changes sharply between nearby
    // Cartesian samples. Allow a per-motion scale so the final place approach
    // can run much more slowly than ordinary retract motions.
    const double safe_linear_velocity = std::clamp(linear_velocity_scaling, 0.005, 0.40);
    group.setMaxVelocityScalingFactor(linear ? safe_linear_velocity : 0.60);
    group.setMaxAccelerationScalingFactor(linear ? std::min(0.10, safe_linear_velocity) : 0.50);
    group.clearPoseTargets();
    group.clearPathConstraints();
    group.setPlanningPipelineId(linear ? "pilz_industrial_motion_planner" : "ompl");
    group.setPlannerId(linear ? "LIN" : "RRTConnectkConfigDefault");
    RCLCPP_INFO(
      get_logger(), "%s: target xyz=[%.3f, %.3f, %.3f] quat=[%.3f, %.3f, %.3f, %.3f] (%s)",
      label.c_str(), pose.position.x, pose.position.y, pose.position.z,
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
      linear ? "LIN" : "OMPL");
    if (!group.setPoseTarget(pose)) {
      RCLCPP_ERROR(get_logger(), "%s: MoveIt rejected pose target.", label.c_str());
      return false;
    }
    MoveGroup::Plan plan;
    const auto result = group.plan(plan);
    group.clearPoseTargets();
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "%s: planning failed (code %d).", label.c_str(), result.val);
      return false;
    }
    const auto execution = group.execute(plan);
    if (execution != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "%s: trajectory execution failed (code %d).", label.c_str(), execution.val);
      return false;
    }
    RCLCPP_INFO(get_logger(), "%s completed (%s).", label.c_str(), linear ? "Pilz LIN" : "OMPL");
    return true;
  }

  // MoveL (Pilz LIN) only. If Pilz rejects the line because a joint velocity
  // limit (e.g. robot3_J4) would be exceeded, it is retried at lower velocity
  // scaling. It never falls back to a joint-space move.
  bool planAndExecuteLinearSlow(
    MoveGroup & group,
    const geometry_msgs::msg::Pose & pose,
    const std::string & label)
  {
    for (const double scale : {0.08, 0.04, 0.02, 0.01}) {
      RCLCPP_INFO(
        get_logger(), "%s: Pilz LIN attempt with velocity scale %.3f.",
        label.c_str(), scale);
      if (planAndExecutePose(group, pose, true, label, scale)) {
        return true;
      }
      RCLCPP_WARN(
        get_logger(), "%s: LIN failed at velocity scale %.3f.",
        label.c_str(), scale);
    }
    return false;
  }

  // Execute one continuous Cartesian MoveL trajectory from the current pose
  // to the requested target. Waypoints are sampled along a single straight
  // translation with quaternion interpolation, but they are planned and
  // executed as ONE trajectory (not separate visible stop/start segments).
  bool planAndExecuteCartesianMoveL(
    MoveGroup & group,
    const geometry_msgs::msg::Pose & target,
    const std::string & label,
    double max_translation_step_m = 0.01,
    double max_rotation_step_rad = 0.03)
  {
    const auto current_stamped = group.getCurrentPose(robot3_link_);
    if (current_stamped.header.frame_id.empty()) {
      RCLCPP_ERROR(get_logger(), "%s: current end-effector pose has no frame id.", label.c_str());
      return false;
    }

    const auto & start = current_stamped.pose;
    const double dx = target.position.x - start.position.x;
    const double dy = target.position.y - start.position.y;
    const double dz = target.position.z - start.position.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    tf2::Quaternion start_q(
      start.orientation.x, start.orientation.y,
      start.orientation.z, start.orientation.w);
    tf2::Quaternion target_q(
      target.orientation.x, target.orientation.y,
      target.orientation.z, target.orientation.w);
    if (start_q.length2() < 1e-12 || target_q.length2() < 1e-12) {
      RCLCPP_ERROR(get_logger(), "%s: invalid start or target quaternion.", label.c_str());
      return false;
    }
    start_q.normalize();
    target_q.normalize();
    const double angle = start_q.angleShortestPath(target_q);
    const int translation_steps = static_cast<int>(std::ceil(
      distance / std::max(0.001, max_translation_step_m)));
    const int rotation_steps = static_cast<int>(std::ceil(
      angle / std::max(0.005, max_rotation_step_rad)));
    const int steps = std::clamp(std::max({1, translation_steps, rotation_steps}), 1, 2000);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(static_cast<std::size_t>(steps));
    for (int i = 1; i <= steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      geometry_msgs::msg::Pose waypoint;
      waypoint.position.x = start.position.x + t * dx;
      waypoint.position.y = start.position.y + t * dy;
      waypoint.position.z = start.position.z + t * dz;
      const tf2::Quaternion q = start_q.slerp(target_q, t);
      waypoint.orientation.x = q.x();
      waypoint.orientation.y = q.y();
      waypoint.orientation.z = q.z();
      waypoint.orientation.w = q.w();
      waypoints.push_back(waypoint);
    }

    group.setStartStateToCurrentState();
    group.clearPoseTargets();
    group.clearPathConstraints();
    group.setMaxVelocityScalingFactor(0.15);
    group.setMaxAccelerationScalingFactor(0.10);

    moveit_msgs::msg::RobotTrajectory trajectory;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    RCLCPP_INFO(
      get_logger(),
      "%s: planning ONE continuous Cartesian MoveL trajectory, distance %.3f m, rotation %.3f rad, %d interpolation samples.",
      label.c_str(), distance, angle, steps);

    const double fraction = group.computeCartesianPath(
      waypoints, 0.01, 0.0, trajectory, true, &error_code);
    RCLCPP_INFO(
      get_logger(), "%s: Cartesian path fraction %.3f, MoveIt error code %d.",
      label.c_str(), fraction, error_code.val);
    if (fraction < 0.99 || trajectory.joint_trajectory.points.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "%s: Cartesian MoveL path incomplete (fraction %.3f); refusing to execute a partial path or use MoveJ.",
        label.c_str(), fraction);
      return false;
    }

    const auto execution = group.execute(trajectory);
    if (execution != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(
        get_logger(), "%s: Cartesian MoveL trajectory execution failed (code %d).",
        label.c_str(), execution.val);
      return false;
    }
    RCLCPP_INFO(get_logger(), "%s completed as one continuous Cartesian MoveL trajectory.", label.c_str());
    return true;
  }

  // Solves IK through move_group's /compute_ik service (collision-aware, 5 s
  // timeout) instead of relying on the planner's own goal sampling, which uses
  // the very short IK timeout from kinematics.yaml. Returns the joint values
  // of the planning group.
  bool solveCollisionFreeIk(
    MoveGroup & group, const geometry_msgs::msg::Pose & pose,
    std::map<std::string, double> & joint_goal, const std::string & label,
    bool avoid_collisions = true, double timeout_sec = 5.0)
  {
    if (!ik_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "%s: /compute_ik service is unavailable.", label.c_str());
      return false;
    }

    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    request->ik_request.group_name = group.getName();
    request->ik_request.ik_link_name = robot3_link_;
    request->ik_request.avoid_collisions = avoid_collisions;
    request->ik_request.timeout = durationMessage(timeout_sec);
    request->ik_request.pose_stamped.header.frame_id = "world";
    request->ik_request.pose_stamped.pose = pose;

    // IMPORTANT: /compute_ik needs a valid seed robot state. Leaving
    // ik_request.robot_state empty causes move_group to log
    // "Found empty JointState message" and can make every candidate fail.
    // Use the latest complete state from this planning group as the seed.
    const auto current_state = group.getCurrentState(5.0);
    if (!current_state) {
      RCLCPP_ERROR(
        get_logger(),
        "%s: cannot call /compute_ik because MoveIt has no current robot state. "
        "Check /joint_states and the robot_state_publisher.",
        label.c_str());
      return false;
    }
    moveit::core::robotStateToRobotStateMsg(
      *current_state, request->ik_request.robot_state);

    auto future = ik_client_->async_send_request(request);
    if (future.wait_for(15s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "%s: /compute_ik timed out.", label.c_str());
      return false;
    }

    const auto response = future.get();
    if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(
        get_logger(), "%s: /compute_ik failed (code %d; 1 = success, -31 = no IK solution).",
        label.c_str(), response->error_code.val);
      return false;
    }

    joint_goal.clear();
    const auto & names = response->solution.joint_state.name;
    const auto & positions = response->solution.joint_state.position;
    for (const auto & joint : group.getActiveJoints()) {
      const auto it = std::find(names.begin(), names.end(), joint);
      if (it == names.end()) {
        RCLCPP_ERROR(get_logger(), "%s: IK solution has no value for joint '%s'.", label.c_str(), joint.c_str());
        return false;
      }
      joint_goal[joint] = positions[static_cast<std::size_t>(std::distance(names.begin(), it))];
    }
    return true;
  }

  // OMPL move to a pose, using a collision-free IK joint goal from /compute_ik.
  bool planAndExecuteViaIk(MoveGroup & group, const geometry_msgs::msg::Pose & pose, const std::string & label)
  {
    RCLCPP_INFO(
      get_logger(), "%s: target xyz=[%.3f, %.3f, %.3f] (IK via /compute_ik)",
      label.c_str(), pose.position.x, pose.position.y, pose.position.z);

    std::map<std::string, double> joint_goal;
    if (!solveCollisionFreeIk(group, pose, joint_goal, label)) {
      return false;
    }

    group.setStartStateToCurrentState();
    group.setPlanningTime(15.0);
    group.setMaxVelocityScalingFactor(0.40);
    group.setMaxAccelerationScalingFactor(0.30);
    group.clearPoseTargets();
    group.clearPathConstraints();
    group.setPlanningPipelineId("ompl");
    group.setPlannerId("RRTConnectkConfigDefault");
    if (!group.setJointValueTarget(joint_goal)) {
      RCLCPP_ERROR(get_logger(), "%s: MoveIt rejected the IK joint goal (outside joint bounds?).", label.c_str());
      return false;
    }

    MoveGroup::Plan plan;
    const auto result = group.plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "%s: planning to IK joint goal failed (code %d).", label.c_str(), result.val);
      return false;
    }
    const auto execution = group.execute(plan);
    if (execution != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "%s: trajectory execution failed (code %d).", label.c_str(), execution.val);
      return false;
    }
    RCLCPP_INFO(get_logger(), "%s completed (OMPL, IK joint goal).", label.c_str());
    return true;
  }

  // Logs the target pose, reports whether it is reachable with collisions
  // ignored, then returns the first (dx, dz) offset of the target (world frame)
  // that has a collision-free IK solution.
  bool chooseApproachPose(
    MoveGroup & group, const geometry_msgs::msg::Pose & target,
    const std::vector<std::array<double, 2>> & offsets,
    geometry_msgs::msg::Pose & approach, const std::string & label)
  {
    RCLCPP_INFO(
      get_logger(), "%s frame in world: xyz=[%.3f, %.3f, %.3f] quat=[%.3f, %.3f, %.3f, %.3f]",
      label.c_str(), target.position.x, target.position.y, target.position.z,
      target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w);

    std::map<std::string, double> unused;
    const bool reachable = solveCollisionFreeIk(
      group, target, unused, label + " pose check", false, 3.0);
    RCLCPP_INFO(
      get_logger(), "%s pose IK with collisions ignored: %s",
      label.c_str(), reachable ? "SOLUTION FOUND" : "NO SOLUTION");

    for (const auto & offset : offsets) {
      auto candidate = target;
      candidate.position.x += offset[0];
      candidate.position.z += offset[1];
      if (solveCollisionFreeIk(
          group, candidate, unused, label + " approach candidate", true, 3.0))
      {
        approach = candidate;
        RCLCPP_INFO(
          get_logger(), "%s: using approach offset dx=%.2f dz=%.2f.",
          label.c_str(), offset[0], offset[1]);
        return true;
      }
    }
    RCLCPP_ERROR(get_logger(), "%s: no approach offset has a collision-free IK solution.", label.c_str());
    return false;
  }

  // Same IK screening as chooseApproachPose(), but returns EVERY approach
  // candidate that has a collision-free IK solution, in priority order, so the
  // caller can fall through to the next one if planning to a candidate fails.
  bool collectApproachPoses(
    MoveGroup & group, const geometry_msgs::msg::Pose & target,
    const std::vector<std::array<double, 2>> & offsets,
    std::vector<geometry_msgs::msg::Pose> & approaches, const std::string & label)
  {
    approaches.clear();

    std::map<std::string, double> unused;
    const bool reachable = solveCollisionFreeIk(
      group, target, unused, label + " pose check", false, 3.0);
    RCLCPP_INFO(
      get_logger(), "%s pose IK with collisions ignored: %s",
      label.c_str(), reachable ? "SOLUTION FOUND" : "NO SOLUTION");

    for (const auto & offset : offsets) {
      auto candidate = target;
      candidate.position.x += offset[0];
      candidate.position.z += offset[1];
      if (solveCollisionFreeIk(
          group, candidate, unused, label + " approach candidate", true, 3.0))
      {
        approaches.push_back(candidate);
        RCLCPP_INFO(
          get_logger(), "%s: approach offset dx=%.2f dz=%.2f has IK.",
          label.c_str(), offset[0], offset[1]);
      }
    }

    if (approaches.empty()) {
      RCLCPP_ERROR(get_logger(), "%s: no approach offset has a collision-free IK solution.", label.c_str());
      return false;
    }
    return true;
  }

  // MoveJ-style (OMPL) pose-goal move that re-plans several times.
  // OMPL can return a path that the planning pipeline then rejects as invalid
  // (code -2, "Motion plan was found but it seems to be invalid") when an
  // interpolated state clips an object. The planner is randomised, so a new
  // attempt often produces a valid path.
  bool planAndExecuteMoveJ(
    MoveGroup & group,
    const geometry_msgs::msg::Pose & pose,
    const std::string & label,
    int attempts = 6)
  {
    group.setStartStateToCurrentState();
    group.setPoseReferenceFrame("world");
    group.setPlanningTime(15.0);
    group.setNumPlanningAttempts(10);
    group.setMaxVelocityScalingFactor(0.40);
    group.setMaxAccelerationScalingFactor(0.30);
    group.clearPoseTargets();
    group.clearPathConstraints();
    group.setPlanningPipelineId("ompl");
    group.setPlannerId("RRTConnectkConfigDefault");

    RCLCPP_INFO(
      get_logger(), "%s: target xyz=[%.3f, %.3f, %.3f] quat=[%.3f, %.3f, %.3f, %.3f] (OMPL, up to %d attempts)",
      label.c_str(), pose.position.x, pose.position.y, pose.position.z,
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
      attempts);

    if (!group.setPoseTarget(pose)) {
      RCLCPP_ERROR(get_logger(), "%s: MoveIt rejected pose target.", label.c_str());
      return false;
    }

    MoveGroup::Plan plan;
    bool planned = false;
    for (int attempt = 1; attempt <= attempts; ++attempt) {
      group.setStartStateToCurrentState();
      const auto result = group.plan(plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        planned = true;
        break;
      }
      RCLCPP_WARN(
        get_logger(), "%s: planning attempt %d/%d failed (code %d).",
        label.c_str(), attempt, attempts, result.val);
    }
    group.clearPoseTargets();

    if (!planned) {
      RCLCPP_ERROR(get_logger(), "%s: all %d planning attempts failed.", label.c_str(), attempts);
      return false;
    }

    const auto execution = group.execute(plan);
    if (execution != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "%s: trajectory execution failed (code %d).", label.c_str(), execution.val);
      return false;
    }
    RCLCPP_INFO(get_logger(), "%s completed (OMPL).", label.c_str());
    return true;
  }

  bool moveHome(MoveGroup & group, const std::vector<double> & home, const std::string & label)
  {
    if (home.empty()) return false;
    group.setStartStateToCurrentState();
    group.setPlanningPipelineId("ompl");
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setMaxVelocityScalingFactor(0.50);
    group.setMaxAccelerationScalingFactor(0.40);
    group.setJointValueTarget(home);
    MoveGroup::Plan plan;
    const auto planned = group.plan(plan);
    if (planned != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "%s home planning failed (code %d).", label.c_str(), planned.val);
      return false;
    }
    const auto executed = group.execute(plan);
    if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "%s home execution failed (code %d).", label.c_str(), executed.val);
      return false;
    }
    RCLCPP_INFO(get_logger(), "%s returned to its startup/home state.", label.c_str());
    return true;
  }

  // NOTE: This sequence updates MoveIt's planning scene; it does not actuate a physical gripper.
  // The rebar is attached to Robot 3 only. Robot 4 is not used.
  //
  // Motion sequence:
  //   1. MoveL (Cartesian path) to a SIDE pick approach, then MoveL into the pick frame.
  //   2. Attach the rebar to Robot 3.
  //   3. Only after Robot 3 has picked the rebar, raise and return the gantry home.
  //   4. Robot 3 uses MoveL to 10 cm pre-place, straight MoveL into place,
  //      retracts sideways along placement-frame local +Y, then returns home.
  bool runRobotPickPlace()
  {
    if (!robot3_group_ || !scene_objects_ || !tf_buffer_) {
      RCLCPP_ERROR(get_logger(), "Robot 3 MoveIt/TF interfaces are not initialized.");
      return false;
    }
    if (!scene_objects_->verifyObjectState(rebar_id_, true, false)) {
      RCLCPP_ERROR(get_logger(), "Expected '%s' in the world before robot pick.", rebar_id_.c_str());
      return false;
    }

    geometry_msgs::msg::Pose pick3, place3;
    if (!lookupPose(pick3_frame_, pick3) || !lookupPose(place3_frame_, place3)) {
      return false;
    }

    // Robot 3 approaches the rebar from the side. After pickup and gantry
    // clearance, it moves to the 10 cm pre-place pose, places, then retracts
    // along the same linear approach path and returns to its captured home state.
    geometry_msgs::msg::Pose place3_safe = place3;

    RCLCPP_INFO(
      get_logger(), "Pick frame '%s' in world: xyz=[%.3f, %.3f, %.3f] quat=[%.3f, %.3f, %.3f, %.3f]",
      pick3_frame_.c_str(), pick3.position.x, pick3.position.y, pick3.position.z,
      pick3.orientation.x, pick3.orientation.y, pick3.orientation.z, pick3.orientation.w);

    // The gripper mesh is link 'robot3_sa3_grpr01' (robot3_end_effactor has no
    // collision geometry), so it must be in the touch-link list. Allow contact
    // with the rebar BEFORE the approach: the gripper envelops the rebar here.
    const std::vector<std::string> touch_links = {
      robot3_link_, "robot3_J6_link", "robot3_sa3_grpr01"
    };
    if (!scene_objects_->setObjectCollisionsAllowed(rebar_id_, touch_links, true)) {
      RCLCPP_ERROR(get_logger(), "Could not allow rebar/gripper contact for Robot 3.");
      return false;
    }

    // Is the pick pose itself reachable (collisions ignored)?
    {
      std::map<std::string, double> unused;
      const bool reachable = solveCollisionFreeIk(
        *robot3_group_, pick3, unused, "Robot 3 pick pose check", false, 3.0);
      RCLCPP_INFO(
        get_logger(), "Pick pose IK with collisions ignored: %s",
        reachable ? "SOLUTION FOUND" : "NO SOLUTION");
    }

    // SIDE approach candidates only: stay at the pick frame's Z and approach
    // from X/Y directions. Deliberately do not try the old vertical-above
    // candidates, because Robot 3 must come in from the side, not bottom-to-top.
    // The selected approach pose is saved so Robot 3 can retract back to it
    // with MoveL after attaching the rebar, before the gantry starts lifting.
    struct ApproachCandidate
    {
      bool local;
      std::array<double, 3> offset;
    };
    const std::vector<ApproachCandidate> approach_candidates = {
      // World-frame side approaches, no Z displacement.
      {false, {-0.30,  0.00, 0.00}},
      {false, { 0.30,  0.00, 0.00}},
      {false, { 0.00, -0.30, 0.00}},
      {false, { 0.00,  0.30, 0.00}},
      {false, {-0.50,  0.00, 0.00}},
      {false, { 0.50,  0.00, 0.00}},
      {false, { 0.00, -0.50, 0.00}},
      {false, { 0.00,  0.50, 0.00}},
      // Tool-frame side approaches, also with zero local Z offset.
      {true, {-0.30,  0.00, 0.00}},
      {true, { 0.30,  0.00, 0.00}},
      {true, { 0.00, -0.30, 0.00}},
      {true, { 0.00,  0.30, 0.00}},
      {true, {-0.50,  0.00, 0.00}},
      {true, { 0.50,  0.00, 0.00}},
      {true, { 0.00, -0.50, 0.00}},
      {true, { 0.00,  0.50, 0.00}}
    };

    bool pick_approach_reached = false;
    for (std::size_t i = 0; i < approach_candidates.size(); ++i) {
      const auto & c = approach_candidates[i];
      auto candidate = pick3;

      if (c.local) {
        tf2::Quaternion q(
          pick3.orientation.x, pick3.orientation.y,
          pick3.orientation.z, pick3.orientation.w);
        if (q.length2() < 1e-12) {
          continue;
        }
        q.normalize();
        const tf2::Vector3 w = tf2::quatRotate(
          q, tf2::Vector3(c.offset[0], c.offset[1], c.offset[2]));
        candidate.position.x += w.x();
        candidate.position.y += w.y();
        candidate.position.z += w.z();
      } else {
        candidate.position.x += c.offset[0];
        candidate.position.y += c.offset[1];
        candidate.position.z += c.offset[2];
      }

      std::map<std::string, double> unused;
      if (!solveCollisionFreeIk(
          *robot3_group_, candidate, unused, "Robot 3 pick approach candidate", true, 3.0))
      {
        RCLCPP_WARN(
          get_logger(), "Pick approach candidate %zu/%zu (%s %.2f %.2f %.2f) has no IK; skipping.",
          i + 1, approach_candidates.size(), c.local ? "local" : "world",
          c.offset[0], c.offset[1], c.offset[2]);
        continue;
      }

      RCLCPP_INFO(
        get_logger(), "Trying SIDE pick approach candidate %zu/%zu (%s dx=%.2f dy=%.2f dz=%.2f) with MoveL.",
        i + 1, approach_candidates.size(), c.local ? "local" : "world",
        c.offset[0], c.offset[1], c.offset[2]);

      // Use Cartesian MoveL to reach the side-pick approach pose as requested.
      // Do not fall back to MoveJ/OMPL if the Cartesian path is incomplete.
      if (planAndExecuteCartesianMoveL(
            *robot3_group_, candidate, "Robot 3 side pick approach MoveL"))
      {
        pick_approach_reached = true;
        break;
      }

      RCLCPP_WARN(
        get_logger(), "Pick approach candidate %zu/%zu could not be planned to; trying the next one.",
        i + 1, approach_candidates.size());
    }

    if (!pick_approach_reached) {
      RCLCPP_ERROR(
        get_logger(),
        "No pick approach candidate could be reached; stopping before pick. "
        "Check which link collides with the rebar in the move_group log.");
      return false;
    }

    // Rebar/gripper contact was already allowed before the approach (see above).
    RCLCPP_INFO(get_logger(), "Robot 3 uses MoveL from the side-approach pose into its pick frame.");
    if (!planAndExecutePose(*robot3_group_, pick3, true, "Robot 3 pick MoveL")) {
      RCLCPP_ERROR(get_logger(), "Robot 3 pick motion failed; gantry will not return home automatically.");
      return false;
    }

    if (!scene_objects_->attachWorldObjectPreservingPose(*robot3_group_, rebar_id_, robot3_link_, touch_links)) {
      RCLCPP_ERROR(get_logger(), "Failed to attach '%s' to Robot 3 after the pick motion.", rebar_id_.c_str());
      return false;
    }
    if (!scene_objects_->verifyObjectState(rebar_id_, false, true, robot3_link_)) {
      RCLCPP_ERROR(get_logger(), "Could not verify Robot 3 attachment of '%s'.", rebar_id_.c_str());
      return false;
    }

    // Optional: let the carried rebar touch listed objects (see parameter
    // 'rebar_allowed_contact_objects'). Empty by default.
    if (!rebar_contact_objects_.empty()) {
      if (!scene_objects_->setObjectCollisionsAllowed(
            rebar_id_, rebar_contact_objects_, true)) {
        RCLCPP_ERROR(
          get_logger(), "Could not allow '%s' contact with the configured objects.",
          rebar_id_.c_str());
        return false;
      }
      RCLCPP_WARN(
        get_logger(),
        "Rebar contact allowed with %zu configured object(s); collision "
        "protection against them is OFF.",
        rebar_contact_objects_.size());
    }

    // After picking and attaching the rebar, retract exactly 10 cm along
    // the Robot 3 tool-frame NEGATIVE X axis. This is the tool's local -X,
    // transformed into world coordinates using the pick-frame orientation;
    // it is not world -X and is not selected from the side-approach direction.
    // Keep the pick orientation fixed and execute a straight Cartesian MoveL
    // while the gantry remains stationary. The gantry may lift only after this
    // retract completes successfully.
    geometry_msgs::msg::Pose post_pick_retract_pose = pick3;
    constexpr double post_pick_retract_distance_m = 0.10;
    tf2::Quaternion pick_tool_q(
      pick3.orientation.x, pick3.orientation.y,
      pick3.orientation.z, pick3.orientation.w);
    if (pick_tool_q.length2() < 1e-12) {
      RCLCPP_ERROR(get_logger(), "Invalid tool orientation at the pick pose; cannot calculate tool -X retract.");
      return false;
    }
    pick_tool_q.normalize();
    const tf2::Vector3 tool_negative_x_world = tf2::quatRotate(
      pick_tool_q, tf2::Vector3(-1.0, 0.0, 0.0));
    post_pick_retract_pose.position.x += post_pick_retract_distance_m * tool_negative_x_world.x();
    post_pick_retract_pose.position.y += post_pick_retract_distance_m * tool_negative_x_world.y();
    post_pick_retract_pose.position.z += post_pick_retract_distance_m * tool_negative_x_world.z();

    RCLCPP_INFO(
      get_logger(),
      "Robot 3 picked the rebar. Retracting %.0f cm along tool-frame -X with MoveL before the gantry lifts.",
      post_pick_retract_distance_m * 100.0);
    {
      std::map<std::string, double> retract_ik;
      if (!solveCollisionFreeIk(
            *robot3_group_, post_pick_retract_pose, retract_ik,
            "Robot 3 10 cm post-pick retract pose", true, 5.0))
      {
        RCLCPP_ERROR(
          get_logger(),
          "The 10 cm post-pick retract pose has no collision-free IK solution. Rebar remains attached; stopping before gantry lift.");
        return false;
      }
    }
    if (!planAndExecuteCartesianMoveL(
          *robot3_group_, post_pick_retract_pose,
          "Robot 3 post-pick 10 cm retract MoveL"))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Robot 3 picked the rebar but could not complete the 10 cm retract MoveL. Keeping the gantry stationary for recovery.");
      return false;
    }

    // Now Robot 3 has picked the rebar and cleared the hand-off area. Raise the
    // gantry and wait for the vertical lift to finish. Once raised, the gantry
    // starts its horizontal return in a background thread while Robot 3 begins
    // the MoveL approach to the place frame; neither waits for the other's
    // horizontal/place travel to finish.
    RCLCPP_INFO(
      get_logger(),
      "Robot 3 has retracted clear of the hand-off area; raising the gantry. Parallel travel starts after the lift completes.");
    if (!moveJointTo(vertical_joint_, vertical_home_)) {
      RCLCPP_ERROR(
        get_logger(),
        "Gantry failed to raise after Robot 3 picked the rebar; stopping Robot 3 place motion.");
      return false;
    }

    bool horizontal_return_success = false;
    std::thread horizontal_return_thread(
      [this, &horizontal_return_success]() {
        RCLCPP_INFO(
          get_logger(),
          "Gantry has reached its raised position. Returning horizontally to home in parallel with Robot 3 MoveL.");
        horizontal_return_success = moveJointTo(horizontal_joint_, horizontal_home_);
        if (horizontal_return_success) {
          RCLCPP_INFO(get_logger(), "Gantry horizontal return to home completed.");
        } else {
          RCLCPP_ERROR(
            get_logger(),
            "Gantry horizontal return failed while Robot 3 was performing its pick/place sequence.");
        }
      });

    // If any later Robot 3 step exits early, still join the background thread
    // before this function returns. This avoids a dangling reference to the
    // local result variable and prevents a joinable std::thread destructor.
    struct JoinHorizontalReturnOnExit
    {
      std::thread & thread;
      ~JoinHorizontalReturnOnExit()
      {
        if (thread.joinable()) {
          thread.join();
        }
      }
    } join_horizontal_return_on_exit{horizontal_return_thread};

    RCLCPP_INFO(
      get_logger(),
      "Gantry is raised. Robot 3 is starting its MoveL approach now; it will not wait for the gantry's horizontal return to finish.");

    // FIX: allow the carried rebar to touch the jig clamp while it is being
    // placed. Without this, the place pose (rebar seated in the clamp) is in
    // collision, so planning to it aborts after the full 15 s planning time.
    // Robot-to-clamp collision checking stays enabled; only rebar<->clamp is
    // allowed. This is done before the approach is chosen so the IK checks
    // match the real placement.
    if (!jig_clamp_name_.empty()) {
      if (!scene_objects_->setObjectCollisionsAllowed(
            rebar_id_, std::vector<std::string>{jig_clamp_name_}, true)) {
        RCLCPP_ERROR(
          get_logger(), "Could not allow contact between '%s' and '%s'.",
          rebar_id_.c_str(), jig_clamp_name_.c_str());
        return false;
      }
      RCLCPP_WARN(
        get_logger(),
        "Placement contact enabled: '%s' may touch '%s'. Robot-to-clamp "
        "collision checking remains enabled.",
        rebar_id_.c_str(), jig_clamp_name_.c_str());
    }

    // Construct the exact 10 cm pre-place target along the place frame's
    // local -X axis. Preserve the configured place pose and orientation.
    geometry_msgs::msg::Pose place3_pre = place3;
    tf2::Quaternion place_q(
      place3.orientation.x, place3.orientation.y,
      place3.orientation.z, place3.orientation.w);
    if (place_q.length2() < 1e-12) {
      RCLCPP_ERROR(
        get_logger(), "Place frame '%s' has an invalid orientation quaternion.",
        place3_frame_.c_str());
      return false;
    }
    place_q.normalize();
    const tf2::Vector3 pre_offset = tf2::quatRotate(
      place_q, tf2::Vector3(-place_approach_distance_, 0.0, 0.0));
    place3_pre.position.x += pre_offset.x();
    place3_pre.position.y += pre_offset.y();
    place3_pre.position.z += pre_offset.z();

    // Go directly from the post-pick pose to the exact 10 cm pre-place pose.
    // This is one continuous Cartesian MoveL trajectory, not 20 separately
    // executed segments and not a MoveJ/OMPL fallback.
    RCLCPP_INFO(
      get_logger(),
      "Robot 3 moving directly with MoveL to the pose %.3f m before '%s'.",
      place_approach_distance_, place3_frame_.c_str());

    {
      std::map<std::string, double> preplace_ik;
      if (!solveCollisionFreeIk(
            *robot3_group_, place3_pre, preplace_ik,
            "Robot 3 exact 10 cm pre-place pose", true, 5.0))
      {
        RCLCPP_ERROR(
          get_logger(),
          "The exact 10 cm pre-place pose has no collision-free IK solution. Rebar remains attached; stopping for recovery.");
        return false;
      }
    }

    if (!planAndExecuteCartesianMoveL(
          *robot3_group_, place3_pre,
          "Robot 3 continuous MoveL to exact 10 cm pre-place pose"))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Robot 3 could not reach the 10 cm pre-place pose with one continuous MoveL path. Rebar remains attached; stopping for recovery.");
      return false;
    }

    // Final insertion: a straight MoveL from the 10 cm pre-place pose directly
    // to the configured place frame. Do not replace this with MoveJ.
    RCLCPP_INFO(
      get_logger(), "Robot 3 is at the pre-place pose; moving straight to '%s' using MoveL.",
      place3_frame_.c_str());
    if (!planAndExecuteCartesianMoveL(
          *robot3_group_, place3, "Robot 3 final straight MoveL to place frame"))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Robot 3 could not move straight from pre-place to the place frame. Rebar remains attached for recovery.");
      return false;
    }

    if (!scene_objects_->detachObjectToWorldPreservingPose(*robot3_group_, rebar_id_)) {
      RCLCPP_ERROR(get_logger(), "Failed to detach '%s' into the world at the place frame.", rebar_id_.c_str());
      return false;
    }

    // After placement, retract sideways along the placement frame's LOCAL +Y
    // axis (not back along the local -X insertion/approach line). Use the same
    // configured clearance distance, then return home only after this MoveL
    // succeeds. Keep the place orientation unchanged during the sideways move.
    geometry_msgs::msg::Pose place3_retract_side = place3;
    const tf2::Vector3 side_retract_offset = tf2::quatRotate(
      place_q, tf2::Vector3(0.0, place_approach_distance_, 0.0));
    place3_retract_side.position.x += side_retract_offset.x();
    place3_retract_side.position.y += side_retract_offset.y();
    place3_retract_side.position.z += side_retract_offset.z();

    RCLCPP_INFO(
      get_logger(),
      "Rebar placed. Robot 3 retracting with MoveL by %.3f m along placement frame local +Y.",
      place_approach_distance_);
    {
      std::map<std::string, double> retract_ik;
      if (!solveCollisionFreeIk(
            *robot3_group_, place3_retract_side, retract_ik,
            "Robot 3 sideways retract pose (local +Y)", true, 5.0))
      {
        RCLCPP_ERROR(
          get_logger(),
          "The sideways retract pose along local +Y has no collision-free IK solution. Rebar is placed; leaving Robot 3 stopped for recovery.");
        return false;
      }
    }
    if (!planAndExecuteCartesianMoveL(
          *robot3_group_, place3_retract_side,
          "Robot 3 sideways MoveL retract along placement-frame local +Y"))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Robot 3 placed the rebar but could not retract sideways along local +Y. Leaving robot stopped for recovery.");
      return false;
    }

    // Wait for the gantry's parallel horizontal return only after Robot 3 has
    // completed the place and sideways retract. This ensures the gantry is
    // clear before Robot 3 begins its own home-return motion, without delaying
    // Robot 3's initial MoveL approach to the placement area.
    if (horizontal_return_thread.joinable()) {
      RCLCPP_INFO(
        get_logger(),
        "Robot 3 has completed placement and sideways retraction; checking that the gantry horizontal return has finished before Robot 3 goes home.");
      horizontal_return_thread.join();
    }
    if (!horizontal_return_success) {
      RCLCPP_ERROR(
        get_logger(),
        "Robot 3 placed and retracted the rebar, but the gantry did not return horizontally to home. Robot 3 will not start its home-return motion.");
      return false;
    }

    // Once clear of the jig and once the gantry is home, return Robot 3 to the
    // joint state captured at startup. moveHome uses OMPL joint-space planning
    // (MoveJ-style), not LIN.
    RCLCPP_INFO(
      get_logger(),
      "Robot 3 is clear of the jig and the gantry is home; returning Robot 3 to its startup/home joint position.");
    if (!moveHome(*robot3_group_, robot3_home_, "Robot 3")) {
      RCLCPP_ERROR(
        get_logger(),
        "Robot 3 placed and retracted the rebar but could not return home.");
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Robot 3 sequence completed: side-pick approach MoveL, pick MoveL, post-pick retract MoveL, gantry lift, parallel gantry horizontal return and Robot 3 MoveL to place, straight MoveL insertion, sideways MoveL retract along placement-frame local +Y, then return to Robot 3 home.");
    return true;
  }

  bool performSequence()
  {
    if (move_group_ == nullptr || scene_objects_ == nullptr) {
      RCLCPP_ERROR(
        get_logger(),
        "MoveIt interfaces have not been initialized.");
      return false;
    }

    if (!trajectory_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(
        get_logger(),
        "Gantry trajectory controller is not available.");
      return false;
    }

    double horizontal_position = 0.0;
    double vertical_position = 0.0;

    if (!waitForJointState(horizontal_joint_, horizontal_position) ||
        !waitForJointState(vertical_joint_, vertical_position)) {
      return false;
    }

    if (!scene_objects_->verifyObjectState(object_id_, true, false)) {
      RCLCPP_ERROR(
        get_logger(),
        "MoveIt object '%s' must exist in the world and must not "
        "already be attached.",
        object_id_.c_str());
      return false;
    }

    // 1. Keep the gantry raised while moving to the rebar pickup position.
    RCLCPP_INFO(
      get_logger(),
      "Step 1/6: moving horizontally to rebar pickup target %.3f m.",
      horizontal_pick_target_);
    if (!moveJointTo(horizontal_joint_, horizontal_pick_target_)) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to reach horizontal rebar pickup target; stopping.");
      return false;
    }

    // 2. Lower to the requested pickup position. The rebar collision object
    //    must already be placed at this physical/modelled pickup location.
    RCLCPP_INFO(
      get_logger(),
      "Step 2/6: lowering to rebar pickup target %.3f m.",
      vertical_pick_target_);
    if (!moveJointTo(vertical_joint_, vertical_pick_target_)) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to lower to the rebar pickup target; stopping.");
      return false;
    }

    rclcpp::sleep_for(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(settle_time_sec_)));

    // 3. Attach only after the gantry has reached the pickup position.
    RCLCPP_INFO(
      get_logger(),
      "Step 3/6: picking '%s' by attaching it to '%s'.",
      object_id_.c_str(), attach_link_.c_str());
    if (!scene_objects_->attachWorldObjectPreservingPose(
          *move_group_, object_id_, attach_link_, {attach_link_})) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to attach '%s' at the pickup position. Stopping.",
        object_id_.c_str());
      return false;
    }

    rclcpp::sleep_for(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(settle_time_sec_)));

    // 4. Raise before any horizontal transport.
    RCLCPP_INFO(
      get_logger(),
      "Step 4/6: raising gantry to vertical home %.3f m.",
      vertical_home_);
    if (!moveJointTo(vertical_joint_, vertical_home_)) {
      RCLCPP_ERROR(
        get_logger(),
        "Gantry failed to raise; rebar remains attached and horizontal "
        "transport is cancelled.");
      return false;
    }

    // 5. Transport the attached rebar to the existing 3.7 m hand-off position.
    RCLCPP_INFO(
      get_logger(),
      "Step 5/6: transporting rebar horizontally to %.3f m.",
      horizontal_target_);
    if (!moveJointTo(horizontal_joint_, horizontal_target_)) {
      RCLCPP_ERROR(
        get_logger(),
        "Horizontal transport failed; rebar remains attached.");
      return false;
    }

    // 6. Lower at the existing hand-off position, then release the rebar.
    RCLCPP_INFO(
      get_logger(),
      "Step 6/6: lowering to hand-off target %.3f m.",
      vertical_target_);
    if (!moveJointTo(vertical_joint_, vertical_target_)) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to lower at the hand-off position; rebar remains attached.");
      return false;
    }

    rclcpp::sleep_for(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(settle_time_sec_)));

    RCLCPP_INFO(
      get_logger(),
      "DETACH DEBUG [gantry_move] BEGIN: object='%s', attach_link='%s'.",
      object_id_.c_str(), attach_link_.c_str());

    // Debug only: record gantry joint positions and the attachment-link pose
    // immediately before calling the existing detach implementation.
    double debug_horizontal = 0.0;
    double debug_vertical = 0.0;
    const bool have_debug_horizontal =
      getJointPosition(horizontal_joint_, debug_horizontal);
    const bool have_debug_vertical =
      getJointPosition(vertical_joint_, debug_vertical);

    RCLCPP_INFO(
      get_logger(),
      "DETACH DEBUG [gantry_move] joints: %s=%.6f m (%s), %s=%.6f m (%s).",
      horizontal_joint_.c_str(), debug_horizontal,
      have_debug_horizontal ? "available" : "unavailable",
      vertical_joint_.c_str(), debug_vertical,
      have_debug_vertical ? "available" : "unavailable");

    const auto debug_link_pose = move_group_->getCurrentPose(attach_link_);
    RCLCPP_INFO(
      get_logger(),
      "DETACH DEBUG [gantry_move] MoveIt pose for link '%s' (frame='%s'): "
      "xyz=[%.6f, %.6f, %.6f], quat=[%.6f, %.6f, %.6f, %.6f].",
      attach_link_.c_str(), debug_link_pose.header.frame_id.c_str(),
      debug_link_pose.pose.position.x,
      debug_link_pose.pose.position.y,
      debug_link_pose.pose.position.z,
      debug_link_pose.pose.orientation.x,
      debug_link_pose.pose.orientation.y,
      debug_link_pose.pose.orientation.z,
      debug_link_pose.pose.orientation.w);

    RCLCPP_INFO(
      get_logger(),
      "Detaching '%s' into the world at the hand-off position.",
      object_id_.c_str());
    if (!scene_objects_->detachObjectToWorldPreservingPose(
          *move_group_, object_id_)) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to detach '%s'. Check the MoveIt planning scene.",
        object_id_.c_str());
      return false;
    }

    const auto debug_link_pose_after = move_group_->getCurrentPose(attach_link_);
    RCLCPP_INFO(
      get_logger(),
      "DETACH DEBUG [gantry_move] AFTER detach: link '%s' pose (frame='%s'): "
      "xyz=[%.6f, %.6f, %.6f], quat=[%.6f, %.6f, %.6f, %.6f].",
      attach_link_.c_str(), debug_link_pose_after.header.frame_id.c_str(),
      debug_link_pose_after.pose.position.x,
      debug_link_pose_after.pose.position.y,
      debug_link_pose_after.pose.position.z,
      debug_link_pose_after.pose.orientation.x,
      debug_link_pose_after.pose.orientation.y,
      debug_link_pose_after.pose.orientation.z,
      debug_link_pose_after.pose.orientation.w);

    RCLCPP_INFO(
      get_logger(),
      "Gantry transported '%s' to the hand-off position and detached it.",
      object_id_.c_str());

    // Robot 3 must first pick and attach the rebar. The gantry return to its
    // raised start/clear position happens inside runRobotPickPlace() immediately
    // after that attachment succeeds, before Robot 3 starts the place motion.
    RCLCPP_INFO(get_logger(), "Starting Robot 3 side-pick sequence; gantry will return home after the rebar is attached.");

    if (!runRobotPickPlace()) {
      RCLCPP_ERROR(
        get_logger(),
        "Robot 3 pick/place phase failed. Inspect robot position and "
        "planning-scene state before recovery.");
      return false;
    }

    return true;
  }

  // Waits until the gantry controller, joint states and the rebar object
  // are all available. Used only for the automatic start.
  bool waitUntilReady()
  {
    const auto start_delay =
      std::chrono::duration<double>(start_delay_sec_);
    const auto delay_deadline =
      std::chrono::steady_clock::now() + start_delay;

    while (rclcpp::ok() &&
           std::chrono::steady_clock::now() < delay_deadline) {
      std::this_thread::sleep_for(100ms);
    }

    const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration<double>(ready_timeout_sec_);

    while (rclcpp::ok() &&
           std::chrono::steady_clock::now() < deadline) {
      double unused = 0.0;

      const bool controller_ready =
        trajectory_client_->wait_for_action_server(1s);

      const bool joints_ready =
        getJointPosition(horizontal_joint_, unused) &&
        getJointPosition(vertical_joint_, unused);

      if (controller_ready && joints_ready &&
          scene_objects_->verifyObjectState(object_id_, true, false)) {
        return true;
      }

      std::this_thread::sleep_for(2s);
    }

    RCLCPP_ERROR(
      get_logger(),
      "Auto-start timed out after %.0f s waiting for the controller, "
      "joint states and object '%s'.",
      ready_timeout_sec_, object_id_.c_str());

    return false;
  }

  // Shared by the service and the automatic start.
  // Do not join worker_ while holding sequence_mutex_: the worker also locks
  // sequence_mutex_ when it finishes, which would deadlock.
  bool launchSequence(bool wait_for_ready, std::string & message)
  {
    std::lock_guard<std::mutex> launch_lock(launch_mutex_);

    {
      std::lock_guard<std::mutex> state_lock(sequence_mutex_);
      if (sequence_running_) {
        message = "Gantry sequence is already running.";
        return false;
      }
      if (sequence_finished_) {
        message =
          "This node has already completed a sequence. "
          "Restart after restoring the rebar to its initial state.";
        return false;
      }
    }

    // The previous worker has stopped if sequence_running_ is false.
    // Join it without holding sequence_mutex_.
    if (worker_.joinable()) {
      worker_.join();
    }

    {
      std::lock_guard<std::mutex> state_lock(sequence_mutex_);
      sequence_running_ = true;
    }

    try {
      worker_ = std::thread([this, wait_for_ready]() {
        bool success = false;
        try {
          if (!wait_for_ready || waitUntilReady()) {
            success = performSequence();
          } else {
            RCLCPP_ERROR(
              get_logger(),
              "Gantry sequence did not start because readiness timed out.");
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(
            get_logger(), "Gantry sequence exception: %s", e.what());
        } catch (...) {
          RCLCPP_ERROR(
            get_logger(), "Unknown exception during gantry sequence.");
        }

        {
          std::lock_guard<std::mutex> state_lock(sequence_mutex_);
          sequence_running_ = false;
          sequence_finished_ = true;
          sequence_success_ = success;
        }
        sequence_cv_.notify_all();

        if (!success) {
          RCLCPP_ERROR(
            get_logger(),
            "Gantry sequence stopped. Inspect controller and attachment state "
            "before attempting recovery.");
        }
      });
    } catch (const std::exception & e) {
      std::lock_guard<std::mutex> state_lock(sequence_mutex_);
      sequence_running_ = false;
      message = std::string("Could not start gantry worker: ") + e.what();
      return false;
    }

    message = "Gantry sequence started.";
    return true;
  }

  void startMotion(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    std::string message;
    response->success = launchSequence(false, message);
    response->message = message;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int exit_code = 0;

  try {
    const auto options =
      rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<GantryMotion>(options);

    // Spin the node in a separate executor thread so action results,
    // joint-state updates, and the start service can be processed.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread executor_thread([&executor]() {
      executor.spin();
    });

    std::unique_ptr<
      moveit::planning_interface::MoveGroupInterface> move_group;

    std::unique_ptr<PlanningSceneObjects> scene_objects;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> robot3_group;
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf_buffer.setUsingDedicatedThread(true);
    tf2_ros::TransformListener tf_listener(tf_buffer, node, false);

    try {
      rclcpp::sleep_for(1s);

      move_group =
        std::make_unique<
          moveit::planning_interface::MoveGroupInterface>(
            node, node->get_parameter("move_group_name").as_string());

      move_group->setPoseReferenceFrame("world");

      robot3_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node, node->get_parameter("robot3_group").as_string());
      scene_objects = std::make_unique<PlanningSceneObjects>(node);

      node->initializeMoveIt(
        move_group.get(), robot3_group.get(), scene_objects.get(), &tf_buffer);

      RCLCPP_INFO(
        node->get_logger(),
        "MoveIt planning-scene interfaces initialized.");

      // Auto-start the sequence unless auto_start:=false. In either mode,
      // wait for the one-shot sequence result so this process exits with:
      //   0 = sequence succeeded
      //   1 = sequence failed or could not start
      if (!node->autoStartIfEnabled()) {
        exit_code = 1;
        executor.cancel();
        if (executor_thread.joinable()) {
          executor_thread.join();
        }
      } else {
        const bool sequence_success = node->waitForSequenceResult();
        exit_code = sequence_success ? 0 : 1;

        executor.cancel();
        if (executor_thread.joinable()) {
          executor_thread.join();
        }

        RCLCPP_INFO(
          node->get_logger(),
          "Gantry node exiting with status %d (%s).",
          exit_code, sequence_success ? "SUCCESS" : "FAILURE");
      }
    } catch (...) {
      executor.cancel();

      if (executor_thread.joinable()) {
        executor_thread.join();
      }

      throw;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("gantry_move"),
      "Initialization failed: %s", e.what());
    exit_code = 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return exit_code;
}
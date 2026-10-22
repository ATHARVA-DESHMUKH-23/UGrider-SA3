#include <algorithm>
#include <chrono>
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
    // Pick approach defaults to a vertical lift in the world frame.
    // The selected pick approach pose is also used as the post-pick retract pose.
    pick_approach_dx_ = parameterOrDeclare<double>("pick_approach_dx_m", 0.0);
    pick_approach_dz_ = parameterOrDeclare<double>("pick_approach_dz_m", 0.60);

    horizontal_target_ = parameterOrDeclare<double>(
      "horizontal_target_m", 3.7);

    vertical_target_ = parameterOrDeclare<double>(
      "down_target_m", 3.2);

    speed_ = parameterOrDeclare<double>("speed_mps", 0.20);
    acceleration_ = parameterOrDeclare<double>(
      "acceleration_mps2", 0.50);

    horizontal_limit_ = parameterOrDeclare<double>(
      "horizontal_limit_m", 5.0);

    vertical_limit_ = parameterOrDeclare<double>(
      "vertical_limit_m", 5.0);

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
      "Sequence: gantry attach/move/lower/detach '%s', robot3 pick, "
      "gantry home, robot3 MoveL place, release and return home.",
      object_id_.c_str());
  }

  ~GantryMotion() override
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  // Called from main() once MoveIt is initialized. Starts the sequence
  // automatically (once) unless the auto_start parameter is false.
  void autoStartIfEnabled()
  {
    if (!auto_start_) {
      RCLCPP_INFO(
        get_logger(),
        "auto_start is false. Call /start_gantry_motion to run the sequence.");
      return;
    }

    std::string message;
    if (launchSequence(true, message)) {
      RCLCPP_INFO(
        get_logger(),
        "Auto-start enabled: sequence begins in %.1f s once the "
        "controller and '%s' are ready.",
        start_delay_sec_, object_id_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Auto-start skipped: %s", message.c_str());
    }
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

  std::mutex sequence_mutex_;
  bool sequence_running_{false};
  bool sequence_finished_{false};

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
  double pick_approach_dx_{-0.60}, pick_approach_dz_{0.0};

  double horizontal_target_{4.3};
  double vertical_target_{3.0};
  double horizontal_limit_{4.3};
  double vertical_limit_{3.0};
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

    if (!std::isfinite(horizontal_target_) ||
        horizontal_target_ < 0.0 ||
        !std::isfinite(vertical_target_) ||
        vertical_target_ < 0.0) {
      throw std::invalid_argument(
        "Gantry targets must be finite and non-negative.");
    }

    if (!std::isfinite(horizontal_limit_) ||
        horizontal_limit_ <= 0.0 ||
        !std::isfinite(vertical_limit_) ||
        vertical_limit_ <= 0.0) {
      throw std::invalid_argument(
        "Gantry limits must be finite and positive.");
    }

    if (horizontal_target_ > horizontal_limit_ ||
        vertical_target_ > vertical_limit_ ||
        horizontal_home_ < 0.0 || horizontal_home_ > horizontal_limit_ ||
        vertical_home_ < 0.0 || vertical_home_ > vertical_limit_) {
      throw std::invalid_argument(
        "Requested target exceeds the configured gantry travel limit. "
        "Verify the URDF and controller limits before changing these parameters.");
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
    const double safe_linear_velocity = std::clamp(linear_velocity_scaling, 0.005, 0.20);
    group.setMaxVelocityScalingFactor(linear ? safe_linear_velocity : 0.30);
    group.setMaxAccelerationScalingFactor(linear ? std::min(0.02, safe_linear_velocity) : 0.20);
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
    for (const double scale : {0.05, 0.02, 0.01}) {
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
    group.setMaxVelocityScalingFactor(0.30);
    group.setMaxAccelerationScalingFactor(0.20);
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
    group.setMaxVelocityScalingFactor(0.30);
    group.setMaxAccelerationScalingFactor(0.20);
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
    group.setMaxVelocityScalingFactor(0.25);
    group.setMaxAccelerationScalingFactor(0.20);
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
  // Motion types:
  //   pick approach   : MoveJ (OMPL)
  //   pick            : MoveL (Pilz LIN)
  //   pick retract    : MoveL
  //   place approach  : MoveJ (OMPL)
  //   place           : MoveL (slowed Pilz LIN, no MoveJ fallback)
  //   place retract   : MoveL
  //   return home     : joint-space (a LIN move cannot target a joint state)
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

    // Robot 3 first tries a vertical approach above the pick frame.
    // The same selected safe pose is used for retracting after the pick.
    auto pick3_safe = elevatedPose(pick3);
    auto place3_safe = elevatedPose(place3);

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

    // Pick approach candidates. Each candidate is (a) screened with a
    // collision-aware IK check and (b) actually PLANNED to with OMPL (with
    // re-planning). The first candidate that can be planned to is used, so a
    // candidate whose path sweeps the wrist through the rebar (for example
    // robot3_J5_link hitting rebar_1A) is skipped instead of aborting the run.
    // Offsets are in metres. 'local' offsets are along the pick frame's own
    // axes (the tool axes); 'world' offsets are along world axes. The reached
    // candidate is also the retract pose after the pick.
    struct ApproachCandidate
    {
      bool local;
      std::array<double, 3> offset;
    };
    const std::vector<ApproachCandidate> approach_candidates = {
      // Along the tool axes first (the gripper closes on the rebar along them).
      {true, {0.00, 0.00, -0.30}},
      {true, {0.00, 0.00,  0.30}},
      {true, {-0.30, 0.00, 0.00}},
      {true, { 0.30, 0.00, 0.00}},
      {true, {0.00, -0.30, 0.00}},
      {true, {0.00,  0.30, 0.00}},
      // Above the pick frame.
      {false, {0.00,  0.00,  0.40}},
      {false, {0.00,  0.00,  0.30}},
      {false, {0.00,  0.00,  0.20}},
      // Approach from either side along world Y.
      {false, {0.00,  0.30,  0.20}},
      {false, {0.00, -0.30,  0.20}},
      {false, {0.00,  0.50,  0.20}},
      {false, {0.00, -0.50,  0.20}},
      // Approach from either side along world X.
      {false, { 0.30, 0.00,  0.20}},
      {false, {-0.30, 0.00,  0.20}},
      {false, { 0.50, 0.00,  0.20}},
      {false, {-0.50, 0.00,  0.20}},
      {false, {-0.60, 0.00,  0.00}},
      {false, {-0.40, 0.00,  0.00}},
      // Lower side approaches as final fallbacks.
      {false, {0.00,  0.30,  0.10}},
      {false, {0.00, -0.30,  0.10}},
      {false, { 0.30, 0.00,  0.10}},
      {false, {-0.30, 0.00,  0.10}}
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
        get_logger(), "Trying pick approach candidate %zu/%zu (%s %.2f %.2f %.2f) with MoveJ (OMPL).",
        i + 1, approach_candidates.size(), c.local ? "local" : "world",
        c.offset[0], c.offset[1], c.offset[2]);

      // MoveJ-style pick approach (OMPL pose goal), re-planned a few times.
      if (planAndExecuteMoveJ(
            *robot3_group_, candidate, "Robot 3 pick approach MoveJ", 3))
      {
        pick3_safe = candidate;
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
    RCLCPP_INFO(get_logger(), "Robot 3 moves linearly into its pick frame.");
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

    // Now that the pick and the planning-scene attachment succeeded,
    // raise the gantry first (vertical joint back to its home value, which is
    // the fully raised position), and only then move it horizontally home.
    // If either step fails, do not start the transport phase.
    RCLCPP_INFO(get_logger(), "Pick succeeded; raising gantry.");
    if (!moveJointTo(vertical_joint_, vertical_home_)) {
      RCLCPP_ERROR(get_logger(), "Gantry failed to go up; stopping before robot transport.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Gantry raised; moving gantry to home position.");
    if (!moveJointTo(horizontal_joint_, horizontal_home_)) {
      RCLCPP_ERROR(get_logger(), "Gantry failed to reach home; stopping before robot transport.");
      return false;
    }

    // MoveL retract to the exact safe pose that was IK-checked before the pick.
    if (!planAndExecutePose(*robot3_group_, pick3_safe, true, "Robot 3 retract MoveL after pick")) return false;

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

    // Choose a place approach offset that has a collision-free IK solution
    // (the rebar is attached now, so it is part of the collision check).
    const std::vector<std::array<double, 2>> place_offsets = {
      {0.0, approach_height_}, {0.0, 0.45}, {0.0, 0.30}, {0.0, 0.15},
      {-0.20, 0.0}, {-0.40, 0.0}, {-0.20, 0.15}, {-0.40, 0.30}, {-0.60, 0.0}};
    std::vector<geometry_msgs::msg::Pose> place_candidates;
    if (!collectApproachPoses(*robot3_group_, place3, place_offsets, place_candidates, "Robot 3 place")) return false;

    // Is the place pose itself valid with collision checking ON (jig contact
    // allowed)? This tells you before the final move whether anything other
    // than the rebar/jig pair blocks the place pose.
    {
      std::map<std::string, double> unused;
      const bool place_ok = solveCollisionFreeIk(
        *robot3_group_, place3, unused, "Robot 3 place pose check (collision-aware)", true, 3.0);
      RCLCPP_INFO(
        get_logger(), "Place pose IK with collisions ON (jig contact allowed): %s",
        place_ok ? "SOLUTION FOUND" : "NO SOLUTION");
      if (!place_ok) {
        RCLCPP_WARN(
          get_logger(),
          "The place pose is unreachable or in collision with something other "
          "than '%s'. The place MoveL is expected to fail.",
          jig_clamp_name_.c_str());
      }
    }

    // MoveJ-style place approach (OMPL pose goal). Each IK-valid candidate is
    // tried in turn, with re-planning per candidate. The candidate that is
    // reached becomes the pose used for the later retract.
    bool approach_reached = false;
    for (std::size_t i = 0; i < place_candidates.size() && !approach_reached; ++i) {
      RCLCPP_INFO(
        get_logger(), "Place approach candidate %zu/%zu.",
        i + 1, place_candidates.size());
      if (planAndExecuteMoveJ(
            *robot3_group_, place_candidates[i], "Robot 3 place approach MoveJ"))
      {
        place3_safe = place_candidates[i];
        approach_reached = true;
      } else {
        RCLCPP_WARN(
          get_logger(), "Place approach candidate %zu/%zu failed; trying the next one.",
          i + 1, place_candidates.size());
      }
    }
    if (!approach_reached) {
      RCLCPP_ERROR(
        get_logger(),
        "Robot 3 failed to reach any place approach; leaving rebar attached for recovery. "
        "If the log names a rebar/object collision, see parameter "
        "'rebar_allowed_contact_objects' or lift the rebar higher before the approach.");
      return false;
    }

    // MoveL into the place pose. Slow, retried at lower velocity scaling if
    // Pilz rejects the line on a joint velocity limit. No MoveJ fallback.
    if (!planAndExecuteLinearSlow(*robot3_group_, place3, "Robot 3 place MoveL")) {
      RCLCPP_ERROR(get_logger(), "Robot 3 failed to reach its place frame with MoveL; leaving rebar attached for recovery.");
      return false;
    }

    if (!scene_objects_->detachObjectToWorldPreservingPose(*robot3_group_, rebar_id_)) {
      RCLCPP_ERROR(get_logger(), "Failed to detach '%s' into the world at the place frame.", rebar_id_.c_str());
      return false;
    }
    // Retract first: the gripper still overlaps the released rebar, so contact
    // must stay allowed until it has moved away. Restore collisions afterwards.
    // (The rebar<->jig_clamp allowance is intentionally left in place: the
    // placed rebar rests in the clamp, and both are world objects.)
    if (!planAndExecuteLinearSlow(*robot3_group_, place3_safe, "Robot 3 retract MoveL from place")) return false;

    if (!scene_objects_->setObjectCollisionsAllowed(rebar_id_, touch_links, false)) {
      RCLCPP_WARN(get_logger(), "Could not restore all rebar/gripper collision pairs after retract.");
    }

    if (!moveHome(*robot3_group_, robot3_home_, "Robot 3")) return false;

    RCLCPP_INFO(get_logger(), "Gantry and Robot 3 pick/place sequence completed.");
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

    if (!scene_objects_->verifyObjectState(
          object_id_, true, false)) {
      RCLCPP_ERROR(
        get_logger(),
        "MoveIt object '%s' must exist in the world and must not "
        "already be attached.",
        object_id_.c_str());
      return false;
    }

    // 1. Attach first, at the current pose. The touch-link list lets the
    //    rebar overlap the gantry part without counting as a collision.
    RCLCPP_INFO(
      get_logger(),
      "Attaching '%s' to '%s' (collision allowed).",
      object_id_.c_str(), attach_link_.c_str());

    if (!scene_objects_->attachWorldObjectPreservingPose(
          *move_group_, object_id_, attach_link_, {attach_link_})) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to attach '%s'. Stopping sequence.",
        object_id_.c_str());
      return false;
    }

    rclcpp::sleep_for(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(settle_time_sec_)));

    // 2. Move horizontally to the target.
    if (!moveJointTo(horizontal_joint_, horizontal_target_)) {
      RCLCPP_ERROR(
        get_logger(),
        "Horizontal movement failed. Rebar remains attached.");
      return false;
    }

    // 3. From that point, lower the vertical axis.
    if (!moveJointTo(vertical_joint_, vertical_target_)) {
      RCLCPP_ERROR(
        get_logger(),
        "Vertical movement failed. Rebar remains attached.");
      return false;
    }

    rclcpp::sleep_for(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(settle_time_sec_)));

    // 4. Detach only after both trajectories succeed.
    RCLCPP_INFO(
      get_logger(),
      "Detaching '%s' into the world.",
      object_id_.c_str());

    if (!scene_objects_->detachObjectToWorldPreservingPose(
          *move_group_, object_id_)) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to detach '%s'. Check the MoveIt planning scene.",
        object_id_.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "Gantry placed '%s' into the world. Starting Robot 3 sequence.", object_id_.c_str());

    if (!runRobotPickPlace()) {
      RCLCPP_ERROR(get_logger(), "Robot 3 pick/place phase failed. Inspect robot position and planning-scene state before recovery.");
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
  bool launchSequence(bool wait_for_ready, std::string & message)
  {
    std::lock_guard<std::mutex> lock(sequence_mutex_);

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

    if (worker_.joinable()) {
      worker_.join();  // previous attempt has already ended
    }

    sequence_running_ = true;

    worker_ = std::thread([this, wait_for_ready]() {
      bool success = false;

      try {
        if (!wait_for_ready || waitUntilReady()) {
          success = performSequence();
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          get_logger(),
          "Gantry sequence exception: %s", e.what());
      } catch (...) {
        RCLCPP_ERROR(
          get_logger(),
          "Unknown exception during gantry sequence.");
      }

      {
        std::lock_guard<std::mutex> lock(sequence_mutex_);
        sequence_running_ = false;
        sequence_finished_ = success;
      }

      if (!success) {
        RCLCPP_ERROR(
          get_logger(),
          "Gantry sequence stopped. Inspect controller and attachment state "
          "before attempting recovery.");
      }
    });

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

      // Run the gantry sequence automatically (unless auto_start:=false).
      node->autoStartIfEnabled();

      executor_thread.join();
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
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <future>
#include <array>
#include <set>
#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
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
#include "sa3_motion/motion_task.hpp"

// The SA1-style declarative planner record. The schema is validated before
// any motion starts; pose offsets and collision actions come from the selected motion recipe.
struct ConfiguredGantryOffset
{
  std::string frame{"world"};
  std::array<double, 3> xyz{0.0, 0.0, 0.0};
};

struct ConfiguredGantryStep
{
  std::string id, description, motion, planner, reference, joint;
  std::string offset_frame{"world"}, rpy_frame{"local"};
  std::array<double, 3> xyz{0.0, 0.0, 0.0};
  std::array<double, 3> rpy{0.0, 0.0, 0.0};
  std::vector<ConfiguredGantryOffset> candidates;
  std::vector<std::string> before_actions, after_actions;
  bool ik_check{false};
};

static std::array<double, 3> configuredVector3(
  const YAML::Node & item, const std::string & field, const std::string & id)
{
  const auto values = item[field];
  if (!values || !values.IsSequence() || values.size() != 3) {
    throw std::runtime_error(id + ": '" + field + "' must be [x, y, z].");
  }
  std::array<double, 3> result{};
  for (std::size_t i = 0; i < 3; ++i) {
    result[i] = values[i].as<double>();
    if (!std::isfinite(result[i])) {
      throw std::runtime_error(id + ": nonfinite '" + field + "'.");
    }
  }
  return result;
}

static std::vector<std::string> configuredActions(
  const YAML::Node & item, const std::string & field, const std::string & id)
{
  const auto actions = item[field];
  if (!actions) return {};
  if (!actions.IsSequence()) {
    throw std::runtime_error(id + ": '" + field + "' must be a list.");
  }
  const std::set<std::string> allowed = {
    "settle", "attach_gantry", "detach_gantry", "prepare_robot3_pick",
    "attach_robot3", "allow_jig_contact", "detach_robot3",
    "lift_gantry_start_parallel_return", "wait_gantry_parallel_return"};
  std::vector<std::string> result;
  for (const auto & entry : actions) {
    const auto name = entry.as<std::string>();
    if (!allowed.count(name)) {
      throw std::runtime_error(id + ": unsupported action '" + name + "'.");
    }
    result.push_back(name);
  }
  return result;
}

static std::vector<ConfiguredGantryStep> readConfiguredGantrySteps(
  const YAML::Node & sequence, bool robot_steps)
{
  if (!sequence || !sequence.IsSequence() || sequence.size() == 0) {
    throw std::runtime_error("motion task.gantry_move requires nonempty gantry/robot3 lists.");
  }
  std::vector<ConfiguredGantryStep> result;
  std::set<std::string> ids;
  for (const auto & item : sequence) {
    ConfiguredGantryStep step;
    step.id = item["id"].as<std::string>();
    if (step.id.empty() || !ids.insert(step.id).second) {
      throw std::runtime_error("Empty or duplicate gantry planner step id: " + step.id);
    }
    step.description = item["description"].as<std::string>(step.id);
    step.motion = item["motion"].as<std::string>();
    step.reference = item["reference"].as<std::string>();
    step.planner = item["planner"].as<std::string>("");
    step.joint = item["joint"].as<std::string>("");
    step.offset_frame = item["offset_frame"].as<std::string>("world");
    step.rpy_frame = item["rpy_frame"].as<std::string>("local");
    for (const auto & frame : {step.offset_frame, step.rpy_frame}) {
      if (frame != "world" && frame != "local") {
        throw std::runtime_error(step.id + ": offset_frame/rpy_frame must be world or local.");
      }
    }
    if (!robot_steps) {
      if (step.motion != "joint" ||
          (step.joint != "horizontal" && step.joint != "vertical") ||
          (step.reference != "pick" && step.reference != "home" && step.reference != "handoff")) {
        throw std::runtime_error(step.id + ": invalid gantry joint motion/reference.");
      }
    } else {
      if (step.motion != "moveJ" && step.motion != "moveL") {
        throw std::runtime_error(step.id + ": Robot 3 motion must be moveJ or moveL.");
      }
      if (step.reference != "pick_frame" && step.reference != "place_frame" && step.reference != "home") {
        throw std::runtime_error(step.id + ": unknown Robot 3 reference.");
      }
      if (step.motion == "moveL" && step.planner != "pilz" &&
          step.planner != "cartesian") {
        throw std::runtime_error(step.id + ": MoveL planner must be pilz or cartesian.");
      }
      if (step.reference == "home" && step.motion != "moveJ") {
        throw std::runtime_error(step.id + ": home must be a moveJ.");
      }
      if (step.reference != "home") {
        step.xyz = configuredVector3(item, "xyz", step.id);
        step.rpy = configuredVector3(item, "rpy", step.id);
      }
      step.ik_check = item["ik_check"].as<bool>(false);
      // No route anchors are supported: R4 is a direct OMPL MoveJ goal.
      if (item["waypoints"] || item["min_samples"] || item["max_samples"]) {
        throw std::runtime_error(step.id + ": intermediate waypoints and sample counts are not supported.");
      }
      const auto candidates = item["candidates"];
      if (candidates) {
        if (!candidates.IsSequence() || candidates.size() == 0 ||
            step.motion != "moveL" || step.planner != "cartesian") {
          throw std::runtime_error(step.id + ": candidates require nonempty list and Cartesian MoveL.");
        }
        for (const auto & c : candidates) {
          ConfiguredGantryOffset offset;
          offset.frame = c["offset_frame"].as<std::string>();
          if (offset.frame != "world" && offset.frame != "local") {
            throw std::runtime_error(step.id + ": candidate offset_frame must be world/local.");
          }
          offset.xyz = configuredVector3(c, "xyz", step.id + ".candidate");
          step.candidates.push_back(offset);
        }
      }
    }
    step.before_actions = configuredActions(item, "before_actions", step.id);
    step.after_actions = configuredActions(item, "after_actions", step.id);
    result.push_back(std::move(step));
  }
  // Safety interlocks deliberately prevent YAML edits from reordering scene
  // ownership or commanding the gantry away while Robot 3 is still in contact.
  const std::vector<std::string> expected_ids = robot_steps ?
    std::vector<std::string>{"R1","R2","R3","R4","R5","R6","R7"} :
    std::vector<std::string>{"G1","G2","G3","G4","G5"};
  const std::vector<std::vector<std::string>> expected_before = robot_steps ?
    std::vector<std::vector<std::string>>{{"prepare_robot3_pick"},{},{},{"allow_jig_contact"},{},{},{}} :
    std::vector<std::vector<std::string>>(5);
  const std::vector<std::vector<std::string>> expected_after = robot_steps ?
    std::vector<std::vector<std::string>>{{},{"attach_robot3"},{"lift_gantry_start_parallel_return"},{},{"detach_robot3"},{"wait_gantry_parallel_return"},{}} :
    std::vector<std::vector<std::string>>{{},{"settle","attach_gantry","settle"},{},{},{"settle","detach_gantry"}};
  if (result.size() != expected_ids.size()) {
    throw std::runtime_error("gantry_move: missing required safety-gated planner steps.");
  }
  for (std::size_t i = 0; i < result.size(); ++i) {
    if (result[i].id != expected_ids[i] ||
        result[i].before_actions != expected_before[i] ||
        result[i].after_actions != expected_after[i]) {
      throw std::runtime_error("gantry_move: invalid safety action ordering at step " + result[i].id);
    }
  }
  if (!robot_steps) {
    const std::vector<std::pair<std::string,std::string>> targets = {
      {"horizontal","pick"},{"vertical","pick"},{"vertical","home"},
      {"horizontal","handoff"},{"vertical","handoff"}};
    for (std::size_t i=0;i<result.size();++i) {
      if (result[i].joint != targets[i].first || result[i].reference != targets[i].second) {
        throw std::runtime_error("gantry_move: unsafe gantry joint order at " + result[i].id);
      }
    }
  } else if (result[0].reference != "pick_frame" || result[0].motion != "moveJ" ||
             !result[0].candidates.empty() ||
             result[1].motion != "moveL" || result[1].planner != "pilz" || result[1].reference != "pick_frame" ||
             result[2].motion != "moveL" || result[2].planner != "cartesian" || result[2].reference != "pick_frame" ||
             result[3].motion != "moveJ" || result[3].planner != "ompl" || result[3].reference != "place_frame" ||
             !result[3].candidates.empty() || !result[3].ik_check ||
             result[4].motion != "moveL" || result[4].planner != "cartesian" || result[4].reference != "place_frame" ||
             result[5].motion != "moveL" || result[5].planner != "cartesian" || result[5].reference != "place_frame" ||
             result[6].reference != "home" || result[6].motion != "moveJ") {
    throw std::runtime_error("gantry_move: Robot 3 safety-gated pick/place planner shape is invalid.");
  }
  // R1 is the sole pick approach: strictly in front of the pick frame,
  // along its local negative-X axis. R3 retracts along the same line.
  if (robot_steps) {
    const auto & approach = result[0];
    const auto & retract = result[2];
    constexpr double epsilon = 1e-8;
    if (approach.offset_frame != "local" ||
        approach.xyz[0] >= -epsilon ||
        std::abs(approach.xyz[1]) > epsilon || std::abs(approach.xyz[2]) > epsilon ||
        retract.offset_frame != "local" ||
        std::abs(retract.xyz[0] - approach.xyz[0]) > epsilon ||
        std::abs(retract.xyz[1]) > epsilon || std::abs(retract.xyz[2]) > epsilon) {
      throw std::runtime_error("gantry_move: R1/R3 must use same local -X front-of-pick offset, with no sideways displacement.");
    }
    const auto & place_approach = result[3];
    const auto & place_retract = result[5];
    if (place_approach.offset_frame != "local" ||
        place_approach.xyz[0] >= -epsilon ||
        std::abs(place_approach.xyz[1]) > epsilon ||
        std::abs(place_approach.xyz[2]) > epsilon ||
        place_retract.offset_frame != "local" ||
        std::abs(place_retract.xyz[0] - place_approach.xyz[0]) > epsilon ||
        std::abs(place_retract.xyz[1]) > epsilon ||
        std::abs(place_retract.xyz[2]) > epsilon) {
      throw std::runtime_error("gantry_move: R4/R6 must use the same local -X place approach/retract offset, with no sideways displacement.");
    }
  }
  return result;
}

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
    loadWorkcellProgram();
    // parameterOrDeclare() is used for every parameter because main() enables
    // automatically_declare_parameters_from_overrides(true). Plain
    // declare_parameter() would throw ParameterAlreadyDeclared for any value
    // that the params file sets.
    controller_action_ = configuredParameterOrDeclare<std::string>(
      "controller_action",
      "/gantry_controller/follow_joint_trajectory");

    horizontal_joint_ = configuredParameterOrDeclare<std::string>(
      "horizontal_joint", "gantry_slide_joint");

    vertical_joint_ = configuredParameterOrDeclare<std::string>(
      "vertical_joint", "gantry_down_joint");

    object_id_ = configuredParameterOrDeclare<std::string>(
      "rebar_object_id", "rebar_1A");

    attach_link_ = configuredParameterOrDeclare<std::string>(
      "attach_link", "gantry_part_link");

    move_group_name_ = configuredParameterOrDeclare<std::string>(
      "move_group_name", "robot3_arm");

    horizontal_home_ = configuredParameterOrDeclare<double>("horizontal_home_m", 0.0);
    vertical_home_ = configuredParameterOrDeclare<double>("vertical_home_m", 0.0);
    robot3_group_name_ = configuredParameterOrDeclare<std::string>("robot3_group", "robot3_arm");
    robot3_link_ = configuredParameterOrDeclare<std::string>("robot3_planning_link", "robot3_end_effactor");
    rebar_id_ = configuredParameterOrDeclare<std::string>("robot_rebar_object_id", "rebar_1A");
    pick3_frame_ = configuredParameterOrDeclare<std::string>("robot3_pick_frame", "bm_pkL");
    place3_frame_ = configuredParameterOrDeclare<std::string>("robot3_place_frame", "bm_pfl");
    // Collision object / link the carried rebar is allowed to touch while it is
    // being placed. Set to an empty string to disable the allowance.
    jig_clamp_name_ = configuredParameterOrDeclare<std::string>("jig_clamp_name", "jig_clamp");
    // Optional: collision objects the carried rebar may touch during transport
    // (for example the inventory rack it was resting on). Empty by default;
    // every entry removes collision protection against that object.
    rebar_contact_objects_ = configuredParameterOrDeclare<std::vector<std::string>>(
      "rebar_allowed_contact_objects", std::vector<std::string>{});
    approach_height_ = configuredParameterOrDeclare<double>("approach_height_m", 0.60);
    // Robot 3 stops this far in front of the place frame before the final MoveL.
    place_approach_distance_ = configuredParameterOrDeclare<double>("place_approach_distance_m", 0.10);
    // Legacy scalar approach parameters kept for CLI/launch compatibility.
    // Recipe robot3_steps.R1/R3 control the actual approach and retract.
    pick_approach_dx_ = configuredParameterOrDeclare<double>("pick_approach_dx_m", 0.0);
    pick_approach_dz_ = configuredParameterOrDeclare<double>("pick_approach_dz_m", 0.60);

    // Gantry first picks rebar_1A at the inventory position, then transports
    // it to the existing hand-off position for the unchanged Robot 3 sequence.
    horizontal_pick_target_ = configuredParameterOrDeclare<double>(
      "horizontal_pick_target_m", 0.0);
    vertical_pick_target_ = configuredParameterOrDeclare<double>(
      "vertical_pick_target_m", 4.1);

    horizontal_target_ = configuredParameterOrDeclare<double>(
      "horizontal_target_m", -6.0);
    vertical_target_ = configuredParameterOrDeclare<double>(
      "down_target_m", 1.2);

    speed_ = configuredParameterOrDeclare<double>("speed_mps", 0.30);
    acceleration_ = configuredParameterOrDeclare<double>(
      "acceleration_mps2", 0.50);

    horizontal_min_ = configuredParameterOrDeclare<double>(
      "horizontal_min_m", -10.6);
    horizontal_limit_ = configuredParameterOrDeclare<double>(
      "horizontal_limit_m", 10.6);

    vertical_limit_ = configuredParameterOrDeclare<double>(
      "vertical_limit_m", 6.6);

    settle_time_sec_ = configuredParameterOrDeclare<double>(
      "settle_time_sec", 0.5);

    trajectory_timeout_sec_ = configuredParameterOrDeclare<double>(
      "trajectory_timeout_sec", 60.0);

    // Auto-start: run the sequence once, by itself, after startup.
    auto_start_ = configuredParameterOrDeclare<bool>("auto_start", true);

    start_delay_sec_ = configuredParameterOrDeclare<double>(
      "start_delay_sec", 3.0);

    ready_timeout_sec_ = configuredParameterOrDeclare<double>(
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

  YAML::Node yaml_parameters_;
  std::vector<ConfiguredGantryStep> gantry_planner_steps_, robot3_planner_steps_;
  std::vector<std::string> robot3_touch_links_;
  double planning_time_{15.0};
  double home_velocity_{0.50}, home_acceleration_{0.40};
  double cartesian_velocity_{0.15}, cartesian_acceleration_{0.10};
  double cartesian_translation_step_{0.01}, cartesian_rotation_step_{0.03};
  double cartesian_eef_step_{0.01}, cartesian_min_fraction_{0.99};
  double pilz_velocity_{0.20}, pilz_acceleration_{0.10};

  // Load before declaring regular parameters: explicit --ros-args/-p/launch
  // overrides win over the defaults stored in motion task.gantry_move.
  void loadWorkcellProgram()
  {
    const auto pkg = parameterOrDeclare<std::string>(
      "workcell_config_package", "sa2_description");
    const auto file = parameterOrDeclare<std::string>(
      "workcell_config_file", "config/workcell.yaml");
    const auto override_path = parameterOrDeclare<std::string>(
      "workcell_config_path", "");
    const std::string path = override_path.empty() ?
      ament_index_cpp::get_package_share_directory(pkg) + "/" + file :
      override_path;
    const auto root = YAML::LoadFile(path);
    const auto workcell = sa3_yaml::require(root, "workcell", "root");
    const auto task = sa3_recipe::loadTask(*this, "gantry_move");
    const auto config = sa3_yaml::require(task, "config", "gantry task");
    const auto hardware = sa3_yaml::require(workcell, "gantry_robot", "workcell");
    yaml_parameters_ = YAML::Clone(config["ros_parameters"]);
    if (!yaml_parameters_ || !yaml_parameters_.IsMap())
      throw std::runtime_error("gantry task.config.ros_parameters must be a mapping");
    // Fixed physical hardware values are owned by workcell.yaml, not the recipe.
    for (const auto & key : {"controller_action", "horizontal_joint", "vertical_joint", "attach_link",
                              "move_group_name", "robot3_group", "robot3_planning_link"}) {
      yaml_parameters_[key] = hardware[key];
    }
    if (!yaml_parameters_ || !yaml_parameters_.IsMap()) {
      throw std::runtime_error("motion task.gantry_move.ros_parameters must be a mapping.");
    }
    gantry_planner_steps_ = readConfiguredGantrySteps(task["gantry_steps"], false);
    robot3_planner_steps_ = readConfiguredGantrySteps(task["robot3_steps"], true);
    const auto links = workcell["robots"]["robot3"]["gantry_touch_links"];
    if (!links || !links.IsSequence() || links.size() == 0) {
      throw std::runtime_error("workcell.robots.robot3.gantry_touch_links required.");
    }
    for (const auto & link : links) {
      robot3_touch_links_.push_back(link.as<std::string>());
      if (robot3_touch_links_.back().empty()) {
        throw std::runtime_error("gantry_move robot3_touch_links cannot include empty links.");
      }
    }
    const auto planning = config["planning"];
    const auto planningValue = [&](const std::string & key, double min, double max) {
      const double value = planning[key].as<double>();
      if (!std::isfinite(value) || value < min || value > max) {
        throw std::runtime_error("gantry_move.planning." + key + " out of range.");
      }
      return value;
    };
    planning_time_ = planningValue("planning_time", 0.1, 600.0);
    home_velocity_ = planningValue("joint_movej_velocity", 0.001, 1.0);
    home_acceleration_ = planningValue("joint_movej_acceleration", 0.001, 1.0);
    cartesian_velocity_ = planningValue("cartesian_velocity", 0.001, 1.0);
    cartesian_acceleration_ = planningValue("cartesian_acceleration", 0.001, 1.0);
    cartesian_translation_step_ = planningValue("cartesian_max_translation_step_m", 0.001, 0.20);
    cartesian_rotation_step_ = planningValue("cartesian_max_rotation_step_rad", 0.001, 1.0);
    cartesian_eef_step_ = planningValue("cartesian_eef_step_m", 0.001, 0.20);
    cartesian_min_fraction_ = planningValue("cartesian_min_fraction", 0.90, 1.0);
    pilz_velocity_ = planningValue("pilz_velocity", 0.001, 1.0);
    pilz_acceleration_ = planningValue("pilz_acceleration", 0.001, 1.0);
    RCLCPP_INFO(get_logger(), "Loaded %zu gantry and %zu Robot3 YAML steps from %s",
      gantry_planner_steps_.size(), robot3_planner_steps_.size(), path.c_str());
  }

  template<typename T>
  T configuredParameterOrDeclare(
    const std::string & name, const T & original_default)
  {
    if (yaml_parameters_[name]) {
      return parameterOrDeclare<T>(name, yaml_parameters_[name].as<T>());
    }
    return parameterOrDeclare<T>(name, original_default);
  }

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
    group.setPlanningTime(planning_time_);
    // Pilz LIN can fail when the IK solution changes sharply between nearby
    // Cartesian samples. Allow a per-motion scale so the final place approach
    // can run much more slowly than ordinary retract motions.
    const double safe_linear_velocity = std::clamp(linear_velocity_scaling, 0.005, 0.40);
    group.setMaxVelocityScalingFactor(linear ? safe_linear_velocity : home_velocity_);
    group.setMaxAccelerationScalingFactor(linear ? std::min(pilz_acceleration_, safe_linear_velocity) : home_acceleration_);
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
    const std::string & label)
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
      distance / std::max(0.001, cartesian_translation_step_)));
    const int rotation_steps = static_cast<int>(std::ceil(
      angle / std::max(0.001, cartesian_rotation_step_)));
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
    group.setMaxVelocityScalingFactor(cartesian_velocity_);
    group.setMaxAccelerationScalingFactor(cartesian_acceleration_);

    moveit_msgs::msg::RobotTrajectory trajectory;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    RCLCPP_INFO(
      get_logger(),
      "%s: planning ONE continuous Cartesian MoveL trajectory, distance %.3f m, rotation %.3f rad, %d interpolation samples.",
      label.c_str(), distance, angle, steps);

    const double fraction = group.computeCartesianPath(
      waypoints, cartesian_eef_step_, 0.0, trajectory, true, &error_code);
    RCLCPP_INFO(
      get_logger(), "%s: Cartesian path fraction %.3f, MoveIt error code %d.",
      label.c_str(), fraction, error_code.val);
    if (fraction < cartesian_min_fraction_ || trajectory.joint_trajectory.points.empty()) {
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
    group.setPlanningTime(planning_time_);
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
    group.setPlanningTime(planning_time_);
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
    group.setMaxVelocityScalingFactor(home_velocity_);
    group.setMaxAccelerationScalingFactor(home_acceleration_);
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

  geometry_msgs::msg::Pose offsetConfiguredPose(
    const geometry_msgs::msg::Pose & base,
    const std::array<double, 3> & xyz,
    const std::string & frame,
    const std::array<double, 3> & rpy = {0.0, 0.0, 0.0},
    const std::string & rotation_frame = "local") const
  {
    geometry_msgs::msg::Pose target = base;
    tf2::Quaternion reference(base.orientation.x, base.orientation.y,
      base.orientation.z, base.orientation.w);
    if (reference.length2() < 1e-12) {
      throw std::runtime_error("Invalid TF reference quaternion in YAML planner target.");
    }
    reference.normalize();
    tf2::Vector3 delta(xyz[0], xyz[1], xyz[2]);
    if (frame == "local") delta = tf2::quatRotate(reference, delta);
    target.position.x += delta.x();
    target.position.y += delta.y();
    target.position.z += delta.z();
    tf2::Quaternion rotation;
    rotation.setRPY(rpy[0], rpy[1], rpy[2]);
    tf2::Quaternion new_orientation = rotation_frame == "local" ?
      reference * rotation : rotation * reference;
    new_orientation.normalize();
    target.orientation.x = new_orientation.x();
    target.orientation.y = new_orientation.y();
    target.orientation.z = new_orientation.z();
    target.orientation.w = new_orientation.w();
    return target;
  }

  bool executeRobot3ConfiguredMotion(
    const ConfiguredGantryStep & step,
    const geometry_msgs::msg::Pose & pick,
    const geometry_msgs::msg::Pose & place)
  {
    const std::string label = step.id + " " + step.description;
    if (step.reference == "home") {
      return moveHome(*robot3_group_, robot3_home_, label);
    }
    const auto & reference = step.reference == "pick_frame" ? pick : place;
    const auto base_target = offsetConfiguredPose(
      reference, step.xyz, step.offset_frame, step.rpy, step.rpy_frame);
    if (!step.candidates.empty()) {
      std::map<std::string, double> unused;
      const bool pick_reachable = solveCollisionFreeIk(
        *robot3_group_, reference, unused, label + " exact pick IK diagnostic", false, 3.0);
      RCLCPP_INFO(get_logger(), "%s exact pose IK ignoring collisions: %s",
        label.c_str(), pick_reachable ? "FOUND" : "NOT FOUND");
      for (std::size_t i = 0; i < step.candidates.size(); ++i) {
        const auto & candidate_offset = step.candidates[i];
        const auto target = offsetConfiguredPose(base_target,
          candidate_offset.xyz, candidate_offset.frame);
        if (!solveCollisionFreeIk(
             *robot3_group_, target, unused, label + " approach candidate", true, 3.0)) {
          RCLCPP_WARN(get_logger(), "%s: candidate %zu/%zu failed collision-aware IK.",
            label.c_str(), i + 1, step.candidates.size());
          continue;
        }
        if (planAndExecuteCartesianMoveL(*robot3_group_, target,
              label + " candidate " + std::to_string(i + 1))) {
          return true;
        }
        RCLCPP_WARN(get_logger(), "%s: Cartesian MoveL candidate %zu failed; trying next.",
          label.c_str(), i + 1);
      }
      RCLCPP_ERROR(get_logger(), "%s: no collision-free candidate approach succeeded.", label.c_str());
      return false;
    }
    if (step.ik_check) {
      std::map<std::string, double> joint_goal;
      if (!solveCollisionFreeIk(*robot3_group_, base_target, joint_goal,
        label + " precheck", true, 5.0)) {
        return false;
      }
    }
    if (step.motion == "moveJ") {
      return planAndExecuteMoveJ(*robot3_group_, base_target, label);
    }
    if (step.planner == "pilz") {
      return planAndExecutePose(*robot3_group_, base_target, true,
        label, pilz_velocity_);
    }
    // A single continuous Cartesian trajectory: no MoveJ fallback, no
    // execution when MoveIt reports an incomplete path.
    return planAndExecuteCartesianMoveL(*robot3_group_, base_target, label);
  }

  // Executes the safety-gated sequence with per-step offsets and actions
  // read from the selected motion-task YAML.
  bool performSequence()
  {
    if (!move_group_ || !robot3_group_ || !scene_objects_ || !tf_buffer_) {
      RCLCPP_ERROR(get_logger(), "MoveIt/TF interfaces are not initialized.");
      return false;
    }
    if (!trajectory_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "Gantry joint trajectory controller unavailable.");
      return false;
    }
    double horizontal = 0.0, vertical = 0.0;
    if (!waitForJointState(horizontal_joint_, horizontal) ||
        !waitForJointState(vertical_joint_, vertical)) return false;
    if (!scene_objects_->verifyObjectState(object_id_, true, false)) {
      RCLCPP_ERROR(get_logger(), "Rebar '%s' must be a world object before gantry pick.",
        object_id_.c_str());
      return false;
    }

    auto settle = [&]() {
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(settle_time_sec_)));
    };

    auto executeJointStep = [&](const ConfiguredGantryStep & step) {
      const bool horizontal_axis = step.joint == "horizontal";
      const auto & joint = horizontal_axis ? horizontal_joint_ : vertical_joint_;
      const double target = step.reference == "home" ?
        (horizontal_axis ? horizontal_home_ : vertical_home_) :
        step.reference == "pick" ?
        (horizontal_axis ? horizontal_pick_target_ : vertical_pick_target_) :
        (horizontal_axis ? horizontal_target_ : vertical_target_);
      RCLCPP_INFO(get_logger(), "[%s] %s: %s -> %.3f m",
        step.id.c_str(), step.description.c_str(), joint.c_str(), target);
      return moveJointTo(joint, target);
    };
    for (const auto & step : gantry_planner_steps_) {
      // The pre-validated G1..G5 actions/order protects the rebar during
      // loading, raised transport, and hand-off.
      if (!executeJointStep(step)) return false;
      for (const auto & action : step.after_actions) {
        if (action == "settle") {
          settle();
        } else if (action == "attach_gantry") {
          RCLCPP_INFO(get_logger(), "[%s] attaching %s to gantry link %s",
            step.id.c_str(), object_id_.c_str(), attach_link_.c_str());
          if (!scene_objects_->attachWorldObjectPreservingPose(
              *move_group_, object_id_, attach_link_, {attach_link_})) return false;
          if (!scene_objects_->verifyObjectState(object_id_, false, true, attach_link_)) {
            RCLCPP_ERROR(get_logger(), "Gantry attachment verification failed.");
            return false;
          }
        } else if (action == "detach_gantry") {
          RCLCPP_INFO(get_logger(), "[%s] hand-off joint positions: %s, %s",
            step.id.c_str(), horizontal_joint_.c_str(), vertical_joint_.c_str());
          const auto link_pose = move_group_->getCurrentPose(attach_link_);
          RCLCPP_INFO(get_logger(), "[%s] gantry attachment link %s pose frame=%s xyz=[%.6f,%.6f,%.6f]",
            step.id.c_str(), attach_link_.c_str(), link_pose.header.frame_id.c_str(),
            link_pose.pose.position.x, link_pose.pose.position.y, link_pose.pose.position.z);
          if (!scene_objects_->detachObjectToWorldPreservingPose(
              *move_group_, object_id_)) return false;
          if (!scene_objects_->verifyObjectState(object_id_, true, false)) {
            RCLCPP_ERROR(get_logger(), "Gantry detach verification failed.");
            return false;
          }
          const auto pose_after = move_group_->getCurrentPose(attach_link_);
          RCLCPP_INFO(get_logger(), "[%s] AFTER hand-off gantry link %s xyz=[%.6f,%.6f,%.6f]",
            step.id.c_str(), attach_link_.c_str(), pose_after.pose.position.x,
            pose_after.pose.position.y, pose_after.pose.position.z);
        }
      }
    }

    if (!scene_objects_->verifyObjectState(rebar_id_, true, false)) {
      RCLCPP_ERROR(get_logger(), "Robot 3 expects '%s' in the world after hand-off.",
        rebar_id_.c_str());
      return false;
    }
    geometry_msgs::msg::Pose pick, place;
    if (!lookupPose(pick3_frame_, pick) || !lookupPose(place3_frame_, place)) {
      return false;
    }

    bool horizontal_return_success = false;
    bool horizontal_return_started = false;
    std::thread horizontal_return_thread;
    struct JoinReturnThread {
      std::thread & thread;
      ~JoinReturnThread() { if (thread.joinable()) thread.join(); }
    } join_on_exit{horizontal_return_thread};

    // Each action is explicitly declared by its YAML step, and the reader
    // validates the exact critical action order before any motion starts.
    auto executeAction = [&](const std::string & action) -> bool {
      if (action == "prepare_robot3_pick") {
        return scene_objects_->setObjectCollisionsAllowed(
          rebar_id_, robot3_touch_links_, true);
      }
      if (action == "attach_robot3") {
        return scene_objects_->attachWorldObjectPreservingPose(
          *robot3_group_, rebar_id_, robot3_link_, robot3_touch_links_) &&
          scene_objects_->verifyObjectState(rebar_id_, false, true, robot3_link_)
          && (rebar_contact_objects_.empty() ||
            scene_objects_->setObjectCollisionsAllowed(
              rebar_id_, rebar_contact_objects_, true));
      }
      if (action == "lift_gantry_start_parallel_return") {
        if (!scene_objects_->verifyObjectState(rebar_id_, false, true, robot3_link_)) {
          RCLCPP_ERROR(get_logger(), "Cannot lift gantry: Robot 3 has not attached the rebar.");
          return false;
        }
        // This action occurs ONLY after R3's 5 cm retract MoveL succeeded.
        if (!moveJointTo(vertical_joint_, vertical_home_)) return false;
        horizontal_return_started = true;
        horizontal_return_thread = std::thread([&]() {
          horizontal_return_success = moveJointTo(horizontal_joint_, horizontal_home_);
          if (!horizontal_return_success) {
            RCLCPP_ERROR(get_logger(), "Parallel gantry horizontal return failed.");
          }
        });
        return true;
      }
      if (action == "allow_jig_contact") {
        if (jig_clamp_name_.empty()) return true;
        return scene_objects_->setObjectCollisionsAllowed(
          rebar_id_, std::vector<std::string>{jig_clamp_name_}, true);
      }
      if (action == "detach_robot3") {
        return scene_objects_->detachObjectToWorldPreservingPose(*robot3_group_, rebar_id_)
          && scene_objects_->verifyObjectState(rebar_id_, true, false);
      }
      if (action == "wait_gantry_parallel_return") {
        if (!horizontal_return_started) {
          RCLCPP_ERROR(get_logger(), "Gantry return never started; cannot proceed to home.");
          return false;
        }
        if (horizontal_return_thread.joinable()) horizontal_return_thread.join();
        return horizontal_return_success;
      }
      RCLCPP_ERROR(get_logger(), "Unknown configured robot action '%s'.", action.c_str());
      return false;
    };

    for (const auto & step : robot3_planner_steps_) {
      RCLCPP_INFO(get_logger(), "[%s] %s", step.id.c_str(), step.description.c_str());
      for (const auto & action : step.before_actions) {
        if (!executeAction(action)) {
          RCLCPP_ERROR(get_logger(), "[%s] before action '%s' failed.",
            step.id.c_str(), action.c_str());
          return false;
        }
      }
      if (!executeRobot3ConfiguredMotion(step, pick, place)) {
        RCLCPP_ERROR(get_logger(), "[%s] motion failed; stopping safely. Inspect attachment state.",
          step.id.c_str());
        return false;
      }
      for (const auto & action : step.after_actions) {
        if (!executeAction(action)) {
          RCLCPP_ERROR(get_logger(), "[%s] after action '%s' failed.",
            step.id.c_str(), action.c_str());
          return false;
        }
      }
    }
    RCLCPP_INFO(get_logger(), "Gantry + Robot 3 YAML planner sequence completed.");
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
// SA3 action-library executor, ROS 2 Humble/Jazzy + MoveIt 2.
// Simulation prototype. All robot trajectories are submitted through MoveIt execute().
// No direct FollowJointTrajectory client, checkpoint or task-specific dispatch.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <std_msgs/msg/string.hpp>
#include <condition_variable>

using MGI=moveit::planning_interface::MoveGroupInterface;
using Plan=MGI::Plan;
using JT=trajectory_msgs::msg::JointTrajectory;
using JP=trajectory_msgs::msg::JointTrajectoryPoint;
using namespace std::chrono_literals;

namespace {
std::string str(const YAML::Node& n,const std::string& key,const std::string& d="") {return n&&n[key]?n[key].as<std::string>():d;}
double num(const YAML::Node& n,const std::string& key,double d) {return n&&n[key]?n[key].as<double>():d;}
bool flag(const YAML::Node& n,const std::string& key,bool d=false) {return n&&n[key]?n[key].as<bool>():d;}
YAML::Node need(const YAML::Node& n,const std::string& key) {if(!n||!n[key])throw std::runtime_error("Required YAML field missing: "+key);return n[key];}
std::vector<std::string> strings(const YAML::Node& n) {return n&&n.IsSequence()?n.as<std::vector<std::string>>():std::vector<std::string>{};}
std::array<double,3> v3(const YAML::Node& n) {if(!n)return {0,0,0};if(!n.IsSequence()||n.size()!=3)throw std::runtime_error("Expected XYZ/RPY list of 3");return {n[0].as<double>(),n[1].as<double>(),n[2].as<double>()};}
void checkScale(double d) {if(!std::isfinite(d)||d<=0||d>1)throw std::runtime_error("Velocity/acceleration scale must be in (0,1]");}
YAML::Node lookup(const YAML::Node& p, const std::string& dotted) {
  if (dotted.empty()) throw std::runtime_error("Empty action parameter name");
  const auto dot = dotted.find('.');
  const auto key = dotted.substr(0, dot);
  if (!p || !p.IsMap() || !p[key])
    throw std::runtime_error("Missing action parameter: " + dotted);
  const YAML::Node child = p[key];
  if (dot == std::string::npos) return child;
  return lookup(child, dotted.substr(dot + 1));
}
YAML::Node interpolate(const YAML::Node& src,const YAML::Node& params) {
  if(!src)return YAML::Node();
  if(src.IsMap()) {YAML::Node out(YAML::NodeType::Map);for(const auto& kv:src)out[kv.first.as<std::string>()]=interpolate(kv.second,params);return out;}
  if(src.IsSequence()) {YAML::Node out(YAML::NodeType::Sequence);for(const auto& n:src)out.push_back(interpolate(n,params));return out;}
  if(!src.IsScalar())return YAML::Clone(src);
  const std::string value=src.Scalar();
  static const std::regex re(R"(\{\{\s*([A-Za-z_][A-Za-z0-9_.]*)\s*\}\})");
  std::smatch match;
  if(std::regex_match(value,match,re))return YAML::Clone(lookup(params,match[1].str()));
  std::string result;auto from=value.cbegin();
  while(std::regex_search(from,value.cend(),match,re)) {
    result.append(from,match[0].first);auto x=lookup(params,match[1].str());
    if(!x.IsScalar())throw std::runtime_error("Interpolated string requires scalar: "+match[1].str());
    result+=x.as<std::string>();from=match[0].second;
  }
  result.append(from,value.cend());return YAML::Node(result);
}
inline double sec(const JP& p) {return rclcpp::Duration(p.time_from_start).seconds();}
inline void setSec(JP& p,double s) {rclcpp::Duration t= rclcpp::Duration::from_seconds(s);const auto ns = static_cast<int64_t>(std::llround(s * 1e9));
    p.time_from_start.sec = static_cast<int32_t>(ns / 1000000000LL);
    p.time_from_start.nanosec =
        static_cast<uint32_t>(ns % 1000000000LL);}
JP interpolateJT(const JT& t,double seconds,double stretch) {
  const double u=seconds/stretch;size_t hi=0;
  while(hi+1<t.points.size()&&sec(t.points[hi])<u)++hi;
  size_t lo=hi?hi-1:hi;if(u>=sec(t.points.back()))lo=hi=t.points.size()-1;
  auto &a=t.points[lo],&b=t.points[hi];const double d=sec(b)-sec(a),alpha=d>1e-9?std::clamp((u-sec(a))/d,0.,1.):0.;
  JP r;
  for(size_t i=0;i<t.joint_names.size();++i) {
    r.positions.push_back(a.positions.at(i)+(b.positions.at(i)-a.positions.at(i))*alpha);
    if(a.velocities.size()==t.joint_names.size()&&b.velocities.size()==t.joint_names.size())r.velocities.push_back((a.velocities[i]+(b.velocities[i]-a.velocities[i])*alpha)/stretch);
    if(a.accelerations.size()==t.joint_names.size()&&b.accelerations.size()==t.joint_names.size())r.accelerations.push_back((a.accelerations[i]+(b.accelerations[i]-a.accelerations[i])*alpha)/(stretch*stretch));
  }return r;
}
JT mergeTracks(const std::vector<JT>& tracks) {
  if(tracks.empty())throw std::runtime_error("No planned tracks");
  if(tracks.size()==1)return tracks[0];
  std::set<std::string> jointNames;double total=0;
  for(const auto& t:tracks){if(t.joint_names.empty()||t.points.empty()||sec(t.points.back())<=0)throw std::runtime_error("Invalid planned track");
    for(auto& j:t.joint_names)if(!jointNames.insert(j).second)throw std::runtime_error("Overlapping joint: "+j);
    total=std::max(total,sec(t.points.back()));}
  std::vector<double> times{0,total};
  for(const auto& t:tracks){auto scale=total/sec(t.points.back());for(const auto& p:t.points)times.push_back(sec(p)*scale);}
  std::sort(times.begin(),times.end());times.erase(std::unique(times.begin(),times.end(),[](double x,double y){return std::abs(x-y)<1e-8;}),times.end());
  JT out;for(const auto& t:tracks)out.joint_names.insert(out.joint_names.end(),t.joint_names.begin(),t.joint_names.end());
  for(double time:times){JP p;setSec(p,time);bool allV=true,allA=true;std::vector<JP> parts;
    for(const auto& t:tracks){auto x=interpolateJT(t,time,total/sec(t.points.back()));parts.push_back(x);allV&=x.velocities.size()==t.joint_names.size();allA&=x.accelerations.size()==t.joint_names.size();}
    for(auto& x:parts){p.positions.insert(p.positions.end(),x.positions.begin(),x.positions.end());if(allV)p.velocities.insert(p.velocities.end(),x.velocities.begin(),x.velocities.end());if(allA)p.accelerations.insert(p.accelerations.end(),x.accelerations.begin(),x.accelerations.end());}
    out.points.push_back(std::move(p));}
  return out;
}
JT holdTrack(const moveit::core::RobotState& state,const moveit::core::JointModelGroup* jm,double duration) {
  if(!jm)throw std::runtime_error("Missing robot group for stationary hold");
  JT out;out.joint_names=jm->getVariableNames();if(out.joint_names.empty())throw std::runtime_error("Hold has no joints");
  JP p;for(const auto& j:out.joint_names)p.positions.push_back(state.getVariablePosition(j));
  p.velocities.assign(p.positions.size(),0);p.accelerations.assign(p.positions.size(),0);
  setSec(p,0);out.points.push_back(p);setSec(p,std::max(0.05,duration));out.points.push_back(p);return out;
}
tf2::Transform asTF(const geometry_msgs::msg::Pose& p) {return tf2::Transform(tf2::Quaternion(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w),tf2::Vector3(p.position.x,p.position.y,p.position.z));}
geometry_msgs::msg::Pose asPose(const tf2::Transform& t) {geometry_msgs::msg::Pose p;auto q=t.getRotation();q.normalize();p.position.x=t.getOrigin().x();p.position.y=t.getOrigin().y();p.position.z=t.getOrigin().z();p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w();return p;}
struct Goal{std::string robot,link,state;geometry_msgs::msg::Pose pose;};
struct Lock {int fd{-1};explicit Lock(const std::string& path){fd=::open(path.c_str(),O_CREAT|O_RDWR,0666);if(fd<0||flock(fd,LOCK_EX))throw std::runtime_error("Cannot obtain ACM lock: "+path);}~Lock(){if(fd>=0){flock(fd,LOCK_UN);::close(fd);}}};
}

class ActionEngine {
public:
  explicit ActionEngine(rclcpp::Node::SharedPtr node):n_(std::move(node)),tf_(n_->get_clock()) {
    tf_.setUsingDedicatedThread(true);listener_=std::make_shared<tf2_ros::TransformListener>(tf_,n_,false);
    const auto lib=n_->declare_parameter<std::string>("action_library_path","");
    name_=n_->declare_parameter<std::string>("action_name","");
    instance_=n_->declare_parameter<std::string>("instance_id","manual");
    dry_=n_->declare_parameter<bool>("dry_run",true);
    sim_=n_->declare_parameter<bool>("simulation_confirmed",false);
    // Allow new MoveGroup action clients to stabilize after DDS discovery.
    // Bounded startup delay only; planning/execution timeouts stay unchanged.
    const auto settleMs=n_->declare_parameter<int>("moveit_client_settle_ms",500);
    if(settleMs<0 || settleMs>5000)
      throw std::runtime_error("moveit_client_settle_ms must be between 0 and 5000");
    if(lib.empty()||name_.empty())throw std::runtime_error("action_library_path and action_name required");
    YAML::Node root=YAML::LoadFile(lib);YAML::Node library=need(root,"action_library");
    fullGroup_=str(library,"collision_group","sa3_all_robots");
    acmLock_=str(library,"acm_lock","/tmp/sa3_action_engine_acm.lock");
    globalPlanning_=need(library,"planning");
    fullCellCollisionCheck_=flag(globalPlanning_,"full_cell_collision_check",true);
    previewRejected_=flag(globalPlanning_,"preview_rejected_trajectories",false);
    if(!fullCellCollisionCheck_) RCLCPP_WARN(n_->get_logger(),"[FULL VALIDATION DISABLED] Skipping SA3 workcell collision sampling; simulation only. MoveIt planning collision checks remain active.");
    if(previewRejected_) {
      rejectedPublisher_=n_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "/display_planned_path",rclcpp::QoS(10).reliable().transient_local());
    }
    auto libraryActions=need(library,"actions");if(!libraryActions[name_])throw std::runtime_error("Unknown action: "+name_);
    auto act=libraryActions[name_];
    YAML::Node defaults=act["defaults"]?YAML::Clone(act["defaults"]):YAML::Node(YAML::NodeType::Map);
    auto arguments=YAML::Load(n_->declare_parameter<std::string>("action_inputs_yaml","{}"));
    if(arguments&&arguments.IsMap())for(const auto& item:arguments)defaults[item.first.as<std::string>()]=item.second;
    params_=defaults;action_=interpolate(act,params_);
    const auto wcPackage=str(library,"workcell_package","sa2_description");
    const auto wcFile=str(library,"workcell_file","config/workcell.yaml");
    // This path may be supplied explicitly to inspect the user's live workcell.
    auto wcPath=n_->declare_parameter<std::string>("workcell_config_path","");
    if(wcPath.empty())wcPath=(std::filesystem::path(ament_index_cpp::get_package_share_directory(wcPackage))/wcFile).string();
    auto wcRoot=YAML::LoadFile(wcPath);workcell_=need(wcRoot,"workcell");
    for(const auto& item:need(workcell_,"robots")) {
      const auto robot=item.first.as<std::string>();auto config=item.second;
      robotGroup_[robot]=str(config,"group");
      robotLink_[robot]=str(config,"planning_link",str(config,"link"));
      robotTyingLink_[robot]=str(config,"tying_link",robotLink_[robot]);
      robotToolMount_[robot]=str(config,"tool_mount_link",robot+"_tool_mount");
    }
    validity_=n_->create_client<moveit_msgs::srv::GetStateValidity>("/check_state_validity");
    getScene_=n_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    // Standard topic bridge. Only the gantry node touches its FollowJointTrajectory action.
    gantryCmd_=n_->create_publisher<std_msgs::msg::String>("/sa3/gantry/command",rclcpp::QoS(20).reliable());
    gantryCancel_=n_->create_publisher<std_msgs::msg::String>("/sa3/gantry/cancel",rclcpp::QoS(20).reliable());
    gantryStatus_=n_->create_subscription<std_msgs::msg::String>("/sa3/gantry/status",
      rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        try {
          const auto status=YAML::Load(msg->data);
          const auto id=str(status,"request_id");
          if(id.empty())return;
          {
            std::lock_guard<std::mutex> lock(gantryMutex_);
            gantryResults_[id]={str(status,"status"),str(status,"message")};
          }
          gantryCv_.notify_all();
        } catch(const std::exception& e) {
          RCLCPP_WARN(n_->get_logger(),"[GANTRY] malformed status message: %s",e.what());
        }
      });
  }
  int run(){
    try {
      if(!dry_&&!sim_)throw std::runtime_error("Execution restricted to explicitly confirmed simulation");
      auto steps=need(action_,"steps");if(!steps.IsSequence())throw std::runtime_error("Action steps must be a list");
      for(const auto& step:steps)stepRun(step);
      restoreOutstanding();
      RCLCPP_INFO(n_->get_logger(),"[SA3 ACTION] SUCCESS %s",name_.c_str());return 0;
    }catch(const std::exception& e){
      RCLCPP_ERROR(n_->get_logger(),"[SA3 ACTION] FAILED %s: %s",name_.c_str(),e.what());
      // Cancel any outstanding gantry FJT command on failure (cannot guarantee hard stop).
      cancelPendingGantry();
      try{restoreOutstanding();}catch(const std::exception& x){RCLCPP_ERROR(n_->get_logger(),"ACM RESTORE FAILED: %s",x.what());}
      return 1;
    }
  }
private:
  MGI& group(const std::string& name){
    if(name.empty())throw std::runtime_error("Empty SRDF group");
    auto& ptr=groups_[name];
    if(!ptr){
      const auto begin=std::chrono::steady_clock::now();
      RCLCPP_INFO(n_->get_logger(),"[MOVEIT CLIENT INIT] constructing group=%s",name.c_str());
      ptr=std::make_unique<MGI>(n_,name);
      const auto elapsed=std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now()-begin).count();
      RCLCPP_INFO(n_->get_logger(),"[MOVEIT CLIENT READY] group=%s init_ms=%ld",
                  name.c_str(),static_cast<long>(elapsed));
      // New SA3 action processes create fresh ROS 2 action clients. Give the
      // discovery graph a brief stabilization window before the first request.
      // This is a mitigation for startup races, not a fix for RMW/DDS failures.
      if(!clientStartupSettled_){
        const auto settleMs=n_->get_parameter("moveit_client_settle_ms").as_int();
        if(settleMs>0){
          RCLCPP_INFO(n_->get_logger(),"[MOVEIT CLIENT SETTLE] %ld ms before first request",
                      static_cast<long>(settleMs));
          std::this_thread::sleep_for(std::chrono::milliseconds(settleMs));
        }
        clientStartupSettled_=true;
      }
    }
    return *ptr;
  }
  geometry_msgs::msg::Pose tfPose(const std::string& frame){
    if(frame.empty())throw std::runtime_error("Empty target reference frame");
    auto t=tf_.lookupTransform("world",frame,tf2::TimePointZero,tf2::durationFromSec(10));
    geometry_msgs::msg::Pose p;p.position.x=t.transform.translation.x;p.position.y=t.transform.translation.y;p.position.z=t.transform.translation.z;p.orientation=t.transform.rotation;return p;
  }
  Goal goalOf(const YAML::Node& n) {
    Goal g;g.robot=str(n,"robot");if(!robotGroup_.count(g.robot))throw std::runtime_error("Robot missing from workcell: "+g.robot);
    g.link=str(n,"link",robotLink_.at(g.robot));
    if(g.link=="@tying")g.link=robotTyingLink_.at(g.robot);
    else if(g.link=="@tool")g.link=robotToolMount_.at(g.robot);
    g.state=str(n,"named_state");
    if(!g.state.empty())return g;
    auto target=need(n,"target");g.pose=tfPose(str(target,"reference"));
    auto offset=v3(target["xyz"]),rpy=v3(target["rpy"]);
    const auto offsetFrame=str(target,"offset_frame","world"),rpyFrame=str(target,"rpy_frame","local");
    tf2::Quaternion q(g.pose.orientation.x,g.pose.orientation.y,g.pose.orientation.z,g.pose.orientation.w);q.normalize();
    // Reproduce the working flange_pp pick-frame behavior: apply the original
    // pick XYZ offset first (using the raw TF orientation), then the step's
    // world-frame retract offset, and finally the pick RPY orientation.
    if(target["base_xyz"]) {
      const auto base=v3(target["base_xyz"]);
      const auto baseFrame=str(target,"base_offset_frame","local");
      tf2::Vector3 baseShift(base[0],base[1],base[2]);
      if(baseFrame=="local")baseShift=tf2::quatRotate(q,baseShift);
      else if(baseFrame!="world")throw std::runtime_error("Unsupported base_offset_frame: "+baseFrame);
      g.pose.position.x+=baseShift.x();g.pose.position.y+=baseShift.y();g.pose.position.z+=baseShift.z();
    }
    tf2::Vector3 shift(offset[0],offset[1],offset[2]);
    if(offsetFrame=="local")shift=tf2::quatRotate(q,shift);
    else if(offsetFrame!="world")throw std::runtime_error("Unsupported offset_frame: "+offsetFrame);
    g.pose.position.x+=shift.x();g.pose.position.y+=shift.y();g.pose.position.z+=shift.z();
    tf2::Quaternion dq;dq.setRPY(rpy[0],rpy[1],rpy[2]);
    if(rpyFrame=="local")q=q*dq;else if(rpyFrame=="world")q=dq*q;
    else throw std::runtime_error("Unsupported rpy_frame: "+rpyFrame);
    q.normalize();g.pose.orientation.x=q.x();g.pose.orientation.y=q.y();g.pose.orientation.z=q.z();g.pose.orientation.w=q.w();return g;
  }
  void configure(MGI& mg,const std::string& motion,const YAML::Node& cfg) {
    mg.clearPoseTargets();mg.clearPathConstraints();mg.setPoseReferenceFrame("world");
    const auto planning=globalPlanning_;
    const auto pipeline=(motion=="moveL"||motion=="movePTP"||motion=="moveC")?"pilz_industrial_motion_planner":"ompl";
    mg.setPlanningPipelineId(pipeline);
    mg.setPlannerId(motion=="moveL"?"LIN":motion=="movePTP"?"PTP":motion=="moveC"?"CIRC":str(cfg,"planner_id",str(planning["moveJ"],"planner_id","PRMstarkConfigDefault")));
    const double vel=num(cfg,"velocity",num(planning[motion],"velocity",0.5));
    const double acc=num(cfg,"acceleration",num(planning,"acceleration",0.35));checkScale(vel);checkScale(acc);
    mg.setPlanningTime(num(cfg,"planning_time",num(planning,"planning_time",15)));
    mg.setNumPlanningAttempts(static_cast<int>(num(planning,"planning_attempts",10)));
    mg.setMaxVelocityScalingFactor(vel);mg.setMaxAccelerationScalingFactor(acc);
  }
  // Planning-only fallback for moveJ. Reuse the exact goal and start state;
  // never execute either candidate until a successful plan has been validated.
  moveit::core::MoveItErrorCode planWithJointFallback(
      MGI& mg, Plan& plan, const std::string& motion, const YAML::Node& cfg) {
    auto status=mg.plan(plan);
    if(motion!="moveJ" || status==moveit::core::MoveItErrorCode::SUCCESS)
      return status;
    const auto policy=globalPlanning_["moveJ"];
    const std::string primary=str(cfg,"planner_id",str(policy,"planner_id","PRMstarkConfigDefault"));
    const auto fallbacks=policy["fallback_planner_ids"];
    if(!fallbacks)return status;
    if(!fallbacks.IsSequence())
      throw std::runtime_error("moveJ.fallback_planner_ids must be a list");
    for(const auto& candidate:fallbacks){
      const auto fallback=candidate.as<std::string>();
      if(fallback.empty() || fallback==primary)continue;
      RCLCPP_WARN(n_->get_logger(),
        "[MOVEJ PLANNER FALLBACK] primary=%s failed code=%d; trying=%s",
        primary.c_str(),status.val,fallback.c_str());
      mg.setPlannerId(fallback);
      plan=Plan{};
      status=mg.plan(plan);
      if(status==moveit::core::MoveItErrorCode::SUCCESS){
        RCLCPP_INFO(n_->get_logger(),"[MOVEJ PLANNER FALLBACK SUCCESS] planner=%s",fallback.c_str());
        return status;
      }
      RCLCPP_WARN(n_->get_logger(),"[MOVEJ PLANNER FALLBACK FAILED] planner=%s code=%d",
                  fallback.c_str(),status.val);
    }
    return status;
  }
  // Optional per-step MoveIt Cartesian planner for moveL + separate_merge.
  // Unlike Pilz LIN, this uses MoveGroupInterface::computeCartesianPath().
  // Keep start state identical across every independently planned robot.
  Plan separateCartesian(const Goal& g, const YAML::Node& cfg,
                         const moveit_msgs::msg::RobotState& start) {
    if(!g.state.empty())throw std::runtime_error("Cartesian planner needs pose target: "+g.robot);
    auto& mg=group(robotGroup_.at(g.robot));
    mg.clearPoseTargets();mg.clearPathConstraints();mg.setPoseReferenceFrame("world");
    mg.setStartState(start);
    auto seed=mg.getCurrentState(10.0);
    if(!seed)throw std::runtime_error("No robot state for Cartesian planner: "+g.robot);
    moveit::core::robotStateMsgToRobotState(start,*seed);
    seed->update();
    const auto* tip=seed->getLinkModel(g.link);
    if(!tip)throw std::runtime_error("Unknown Cartesian tip link: "+g.link);
    const Eigen::Isometry3d begin=seed->getGlobalLinkTransform(tip);
    const Eigen::Vector3d from=begin.translation();
    const Eigen::Vector3d to(g.pose.position.x,g.pose.position.y,g.pose.position.z);
    const Eigen::Quaterniond q0(begin.rotation());
    Eigen::Quaterniond q1(g.pose.orientation.w,g.pose.orientation.x,
                          g.pose.orientation.y,g.pose.orientation.z);
    if(q1.norm()<1e-9)throw std::runtime_error("Invalid Cartesian goal quaternion: "+g.robot);
    q1.normalize();
    const auto cart=globalPlanning_["cartesian"];
    const double translationStep=num(cfg,"cartesian_translation_step_m",
                                     num(cart,"translation_step_m",0.05));
    const double rotationStep=num(cfg,"cartesian_rotation_step_rad",
                                  num(cart,"rotation_step_rad",0.05));
    const double eefStep=num(cfg,"cartesian_eef_step_m",
                             num(cart,"eef_step_m",0.05));
    const double minFraction=num(cfg,"cartesian_min_fraction",
                                 num(cart,"min_fraction",0.99));
    if(!std::isfinite(translationStep)||translationStep<=0||
       !std::isfinite(rotationStep)||rotationStep<=0||
       !std::isfinite(eefStep)||eefStep<=0||
       !std::isfinite(minFraction)||minFraction<=0||minFraction>1)
      throw std::runtime_error("Invalid Cartesian sampling configuration");
    const int samples=std::max(1,static_cast<int>(std::ceil(std::max(
       (to-from).norm()/translationStep,q0.angularDistance(q1)/rotationStep))));
    if(samples>2000)throw std::runtime_error("Cartesian path exceeds 2000 samples: "+g.robot);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(static_cast<size_t>(samples));
    for(int k=1;k<=samples;++k){
      const double u=static_cast<double>(k)/samples;
      const Eigen::Vector3d p=from+(to-from)*u;
      const Eigen::Quaterniond q=q0.slerp(u,q1).normalized();
      geometry_msgs::msg::Pose pose;
      pose.position.x=p.x();pose.position.y=p.y();pose.position.z=p.z();
      pose.orientation.x=q.x();pose.orientation.y=q.y();
      pose.orientation.z=q.z();pose.orientation.w=q.w();
      waypoints.push_back(pose);
    }
    const double velocity=num(cfg,"velocity",0.5);
    const double acceleration=num(cfg,"acceleration",num(globalPlanning_,"acceleration",0.45));
    checkScale(velocity);checkScale(acceleration);
    mg.setMaxVelocityScalingFactor(velocity);
    mg.setMaxAccelerationScalingFactor(acceleration);
    moveit_msgs::msg::RobotTrajectory trajectory;
    moveit_msgs::msg::MoveItErrorCodes error;
    const double fraction=mg.computeCartesianPath(waypoints,eefStep,0.0,
                                                    trajectory,true,&error);
    RCLCPP_INFO(n_->get_logger(),"[CARTESIAN] robot=%s samples=%d fraction=%.4f error=%d",
      g.robot.c_str(),samples,fraction,error.val);
    if(!std::isfinite(fraction)||fraction<minFraction||
       trajectory.joint_trajectory.points.empty())
      throw std::runtime_error("Cartesian path incomplete for "+g.robot+
         " fraction="+std::to_string(fraction));
    robot_trajectory::RobotTrajectory timed(mg.getRobotModel(),mg.getName());
    timed.setRobotTrajectoryMsg(*seed,trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization parameterizer;
    if(!parameterizer.computeTimeStamps(timed,velocity,acceleration))
      throw std::runtime_error("Cartesian time parameterization failed for "+g.robot);
    Plan plan;
    plan.start_state_=start;
    timed.getRobotTrajectoryMsg(plan.trajectory_);
    if(plan.trajectory_.joint_trajectory.points.empty() ||
       sec(plan.trajectory_.joint_trajectory.points.back())<=0)
      throw std::runtime_error("Cartesian path has no executable duration for "+g.robot);
    return plan;
  }

  Plan separate(const Goal& g,const std::string& motion,const YAML::Node& cfg,const moveit_msgs::msg::RobotState& start){
    // 'planner: cartesian' is opt-in; unspecified moveL remains Pilz LIN.
    const std::string planner=str(cfg,"planner","pilz");
    if(motion=="moveL" && planner=="cartesian")
      return separateCartesian(g,cfg,start);
    if(motion=="moveL" && planner!="pilz")
      throw std::runtime_error("Unknown moveL planner: "+planner);
    auto& mg=group(robotGroup_.at(g.robot));configure(mg,motion,cfg);mg.setStartState(start);
    if(!g.state.empty()) {
      if(motion!="moveJ")throw std::runtime_error("Named target requires moveJ");
      if(!mg.setNamedTarget(g.state))throw std::runtime_error("Unknown named state: "+g.state);
    }else if(!mg.setPoseTarget(g.pose,g.link))throw std::runtime_error("Pose goal rejected: "+g.robot);
    // Pilz CIRC: intermediate point on the arc, expressed in world coordinates.
    // This mirrors the old shear_links_pp transit strategy using an offset from
    // the midpoint of the live start and target tool positions.
    if(motion=="moveC") {
      if(!g.state.empty())throw std::runtime_error("CIRC needs a Cartesian target");
      auto robotState=mg.getCurrentState(10);
      if(!robotState)throw std::runtime_error("Cannot obtain start state for CIRC interim point");
      moveit::core::robotStateMsgToRobotState(start,*robotState);
      robotState->update();
      const Eigen::Vector3d begin=robotState->getGlobalLinkTransform(g.link).translation();
      const Eigen::Vector3d end(g.pose.position.x,g.pose.position.y,g.pose.position.z);
      Eigen::Vector3d via;
      if(cfg["circ_via_world_x"] && cfg["circ_via_world_x"].IsMap() &&
         cfg["circ_via_world_x"][g.robot]) {
        // Exact legacy shear_links_pp construction: via = target approach
        // position, with a fixed world X per robot (0.235 / 5.22).
        via=end;
        via.x()=cfg["circ_via_world_x"][g.robot].as<double>();
      } else if(cfg["circ_via_offset_world"]) {
        const auto offset=v3(cfg["circ_via_offset_world"]);
        via=(begin+end)*0.5+Eigen::Vector3d(offset[0],offset[1],offset[2]);
      } else {
        throw std::runtime_error("moveC requires circ_via_world_x or circ_via_offset_world");
      }
      if((via-begin).norm()<1e-5 || (via-end).norm()<1e-5 ||
         ((end-begin).cross(via-begin)).norm()<1e-7)
        throw std::runtime_error("Degenerate CIRC interim point; change circ_via_offset_world");
      moveit_msgs::msg::PositionConstraint pc;
      pc.header.frame_id="world";
      pc.link_name=g.link;
      pc.weight=1.0;
      shape_msgs::msg::SolidPrimitive sphere;
      sphere.type=shape_msgs::msg::SolidPrimitive::SPHERE;
      sphere.dimensions={0.001};
      pc.constraint_region.primitives.push_back(sphere);
      geometry_msgs::msg::Pose viaPose;
      viaPose.orientation.w=1.0;
      viaPose.position.x=via.x();viaPose.position.y=via.y();viaPose.position.z=via.z();
      pc.constraint_region.primitive_poses.push_back(viaPose);
      moveit_msgs::msg::Constraints constraints;
      constraints.name="interim";
      constraints.position_constraints.push_back(pc);
      mg.setPathConstraints(constraints);
      RCLCPP_INFO(n_->get_logger(),"[CIRC] robot=%s interim_world=[%.3f, %.3f, %.3f]",
        g.robot.c_str(),via.x(),via.y(),via.z());
    }
    Plan plan;auto status=planWithJointFallback(mg,plan,motion,cfg);mg.clearPoseTargets();mg.clearPathConstraints();
    if(status!=moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Individual planning failed for "+g.robot+" code="+std::to_string(status.val));
    return plan;
  }
  Plan combined(const std::vector<Goal>& goals,const std::string& combinedGroup,const YAML::Node& cfg) {
    auto& mg=group(combinedGroup);configure(mg,"moveJ",cfg);auto current=mg.getCurrentState(10);
    if(!current)throw std::runtime_error("Cannot get live state for combined plan");
    auto target=std::make_shared<moveit::core::RobotState>(*current);
    for(const auto& g:goals){const auto jm=target->getJointModelGroup(robotGroup_.at(g.robot));
      if(!jm)throw std::runtime_error("Robot group missing: "+g.robot);
      if(!g.state.empty()){if(!target->setToDefaultValues(jm,g.state))throw std::runtime_error("Unknown named state "+g.state);}
      else if(!target->setFromIK(jm,g.pose,g.link,num(cfg,"ik_timeout",1.0)))throw std::runtime_error("IK failed for "+g.robot);}
    mg.setStartStateToCurrentState();
    if(!mg.setJointValueTarget(*target))throw std::runtime_error("Combined goal rejected");
    Plan plan;auto code=planWithJointFallback(mg,plan,"moveJ",cfg);
    if(code!=moveit::core::MoveItErrorCode::SUCCESS)throw std::runtime_error("Combined planning failed code="+std::to_string(code.val));
    return plan;
  }
  Plan merged(const std::vector<Goal>& goals,const std::vector<std::string>& participants,
              const std::string& combinedGroup,const std::string& motion,const YAML::Node& cfg) {
    auto& mg=group(combinedGroup);auto current=mg.getCurrentState(10);
    if(!current)throw std::runtime_error("No current state for merged planning");
    moveit_msgs::msg::RobotState start;moveit::core::robotStateToRobotStateMsg(*current,start);
    std::set<std::string> active;std::vector<JT> tracks;
    // Per-robot planner fallback is planning-only. All arms use the same start
    // state and all successful tracks still execute as ONE combined trajectory.
    // Robot3/4 can be strict moveL while robot1/2 fall back to moveJ.
    const auto goalConfigs=need(cfg,"goals");
    if(!goalConfigs.IsSequence() || goalConfigs.size()!=goals.size())
      throw std::runtime_error("Merged goal configuration count mismatch");
    for(size_t i=0;i<goals.size();++i) {
      const auto& g=goals[i];
      if(!active.insert(g.robot).second)throw std::runtime_error("Duplicate goal robot: "+g.robot);
      const auto goalCfg=goalConfigs[i];
      // The goal-level planner override, if supplied, applies only to this arm.
      YAML::Node local=YAML::Clone(cfg);
      if(goalCfg["planner"])local["planner"]=goalCfg["planner"].as<std::string>();
      const std::string primary=str(goalCfg,"motion",motion);
      if(primary!=motion)
        throw std::runtime_error("Per-robot primary motion must match step motion for "+g.robot);
      std::string failure;
      bool succeeded=false;
      // Primary plan is attempted first; fallback only after planning failure.
      const auto tryPlan=[&](const std::string& selected,const YAML::Node& settings)->bool {
        try {
          auto planned=separate(g,selected,settings,start);
          tracks.push_back(planned.trajectory_.joint_trajectory);
          RCLCPP_INFO(n_->get_logger(),"[ARM PLAN SUCCESS] robot=%s motion=%s",
                      g.robot.c_str(),selected.c_str());
          return true;
        }catch(const std::exception& e){
          failure=e.what();
          RCLCPP_WARN(n_->get_logger(),"[ARM PLAN FAILURE] robot=%s motion=%s: %s",
                      g.robot.c_str(),selected.c_str(),failure.c_str());
          return false;
        }
      };
      succeeded=tryPlan(primary,local);
      if(!succeeded && goalCfg["fallbacks"]) {
        const auto fallbacks=goalCfg["fallbacks"];
        if(!fallbacks.IsSequence())throw std::runtime_error("Goal fallbacks must be a list for "+g.robot);
        for(const auto& fb:fallbacks){
          const auto fallbackMotion=str(fb,"motion");
          if(fallbackMotion!="moveJ")
            throw std::runtime_error("Only moveJ per-robot fallback supported: "+g.robot);
          YAML::Node backup=YAML::Clone(local);
          // A moveJ fallback MUST use OMPL, not an inherited Cartesian backend.
          backup["planner"]="ompl";
          const auto movejPolicy=globalPlanning_["moveJ"];
          const int retries=static_cast<int>(num(fb,"retries",num(movejPolicy,"retries",3)));
          if(retries<1||retries>20)throw std::runtime_error("Robot moveJ fallback retries must be 1..20");
          const double velocity=num(fb,"velocity",num(movejPolicy,"velocity",0.5));
          checkScale(velocity);backup["velocity"]=velocity;
          for(int attempt=1;attempt<=retries && !succeeded;++attempt){
            RCLCPP_WARN(n_->get_logger(),"[ARM FALLBACK] robot=%s primary=%s fallback=moveJ attempt=%d/%d",
                        g.robot.c_str(),primary.c_str(),attempt,retries);
            succeeded=tryPlan("moveJ",backup);
          }
          if(succeeded)break;
        }
      }
      if(!succeeded)throw std::runtime_error("All allowed planners failed for "+g.robot+": "+failure);
    }
    double duration=0;for(const auto& t:tracks)if(!t.points.empty())duration=std::max(duration,sec(t.points.back()));
    if(duration<=0)throw std::runtime_error("No executable track duration");
    // Stationary participant arms are represented explicitly in the combined trajectory.
    for(const auto& robot:participants)if(!active.count(robot)){
      if(!robotGroup_.count(robot))throw std::runtime_error("Unknown participant: "+robot);
      tracks.push_back(holdTrack(*current,current->getJointModelGroup(robotGroup_.at(robot)),duration));
    }
    Plan result;result.start_state_=start;result.trajectory_.joint_trajectory=mergeTracks(tracks);
    const auto jmg=current->getJointModelGroup(combinedGroup);
    if(!jmg)throw std::runtime_error("Missing combined group: "+combinedGroup);
    std::set<std::string> actual(result.trajectory_.joint_trajectory.joint_names.begin(),result.trajectory_.joint_trajectory.joint_names.end());
    // Reject trajectories that omit joints in the requested execution group.
    for(const auto& joint:jmg->getVariableNames())if(!actual.count(joint))throw std::runtime_error("Combined group joint not covered: "+joint);
    return result;
  }
  Plan coordinatedCartesian(const std::vector<Goal>& goals,const std::string& combinedGroup,const YAML::Node& cfg) {
    auto& mg=group(combinedGroup);auto start=mg.getCurrentState(10);if(!start)throw std::runtime_error("No live state for Cartesian");
    robot_trajectory::RobotTrajectory rt(mg.getRobotModel(),combinedGroup);rt.addSuffixWayPoint(*start,0);
    auto working=std::make_shared<moveit::core::RobotState>(*start);
    int n=1;std::vector<Eigen::Isometry3d> from;
    for(const auto& g:goals){if(!g.state.empty())throw std::runtime_error("Cartesian named state unsupported");
      auto begin=start->getGlobalLinkTransform(g.link);from.push_back(begin);
      Eigen::Vector3d target(g.pose.position.x,g.pose.position.y,g.pose.position.z);
      Eigen::Quaterniond q0(begin.rotation()),q1(g.pose.orientation.w,g.pose.orientation.x,g.pose.orientation.y,g.pose.orientation.z);
      n=std::max(n,static_cast<int>(std::ceil(std::max((target-begin.translation()).norm()/0.01,q0.angularDistance(q1)/0.025))));}
    if(n>2000)throw std::runtime_error("Cartesian segment requires too many samples");
    for(int k=1;k<=n;++k){double a=double(k)/n;
      for(size_t i=0;i<goals.size();++i){const auto& g=goals[i];const auto& startT=from[i];
        Eigen::Vector3d dest(g.pose.position.x,g.pose.position.y,g.pose.position.z),pos=startT.translation()*(1-a)+dest*a;
        Eigen::Quaterniond q0(startT.rotation()),q1(g.pose.orientation.w,g.pose.orientation.x,g.pose.orientation.y,g.pose.orientation.z);
        auto q=q0.slerp(a,q1).normalized();
        geometry_msgs::msg::Pose p=g.pose;p.position.x=pos.x();p.position.y=pos.y();p.position.z=pos.z();p.orientation.w=q.w();p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();
        const auto jm=working->getJointModelGroup(robotGroup_.at(g.robot));
        if(!jm||!working->setFromIK(jm,p,g.link,num(cfg,"ik_timeout",1.0)))throw std::runtime_error("Cartesian IK failed for "+g.robot+" sample="+std::to_string(k));
      }working->update();rt.addSuffixWayPoint(*working,0);}
    trajectory_processing::IterativeParabolicTimeParameterization parameterizer;
    double vel=num(cfg,"velocity",num(globalPlanning_["moveL"],"velocity",0.5));
    double acc=num(cfg,"acceleration",num(globalPlanning_,"acceleration",0.45));checkScale(vel);checkScale(acc);
    if(!parameterizer.computeTimeStamps(rt,vel,acc))throw std::runtime_error("Cartesian timestamp generation failed");
    Plan result;moveit::core::robotStateToRobotStateMsg(*start,result.start_state_);rt.getRobotTrajectoryMsg(result.trajectory_);return result;
  }
  void previewRejectedPlan(const Plan& plan, const std::string& stepId, const std::string& why) {
    if(!previewRejected_ || !rejectedPublisher_ || plan.trajectory_.joint_trajectory.points.empty())return;
    moveit_msgs::msg::DisplayTrajectory msg;
    msg.model_id=group(fullGroup_).getRobotModel()->getName();
    msg.trajectory_start=plan.start_state_;
    msg.trajectory.push_back(plan.trajectory_);
    rejectedPublisher_->publish(msg);
    RCLCPP_ERROR(n_->get_logger(),
      "[RVIZ REJECTED PATH] step=%s published to /display_planned_path; NOT EXECUTED (%s)",
      stepId.c_str(),why.c_str());
  }
  void validate(const Plan& plan) {
    if(dry_)return; // Dry run is static configuration preflight, not a simulated trajectory approval.
    if(!fullCellCollisionCheck_){RCLCPP_WARN(n_->get_logger(),"[FULL VALIDATION SKIPPED] Global workcell collision check disabled in YAML");return;}
    if(!validity_->wait_for_service(10s))throw std::runtime_error("/check_state_validity not available");
    const auto& tr=plan.trajectory_.joint_trajectory;if(tr.points.empty()||tr.joint_names.empty())throw std::runtime_error("Empty trajectory");
    size_t counter=0;
    auto inspect=[&](const std::vector<double>& q) {
      if(++counter>5000)throw std::runtime_error("Collision sample limit exceeded");
      auto req=std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();req->group_name=fullGroup_;
      req->robot_state=plan.start_state_;req->robot_state.is_diff=true;req->robot_state.attached_collision_objects.clear();
      auto& js=req->robot_state.joint_state;
      for(size_t i=0;i<tr.joint_names.size();++i){auto it=std::find(js.name.begin(),js.name.end(),tr.joint_names[i]);
        if(it==js.name.end()||static_cast<size_t>(it-js.name.begin())>=js.position.size())throw std::runtime_error("Joint missing from full state: "+tr.joint_names[i]);
        js.position[it-js.name.begin()]=q.at(i);}
      auto future=validity_->async_send_request(req);
      if(future.wait_for(20s)!=std::future_status::ready){validity_->remove_pending_request(future);throw std::runtime_error("State validity timeout");}
      const auto result=future.get();
      if(!result->valid){for(const auto& c:result->contacts)RCLCPP_ERROR(n_->get_logger(),"COLLISION sample=%zu %s <-> %s",counter,c.contact_body_1.c_str(),c.contact_body_2.c_str());throw std::runtime_error("Full-cell collision validation rejected at sample="+std::to_string(counter));}
    };
    inspect(tr.points.front().positions);
    for(size_t i=1;i<tr.points.size();++i){const auto& a=tr.points[i-1].positions,&b=tr.points[i].positions;
      if(a.size()!=b.size())throw std::runtime_error("Malformed trajectory point");
      double maxStep=0;for(size_t j=0;j<a.size();++j)maxStep=std::max(maxStep,std::abs(b[j]-a[j]));
      const int pieces=std::max(1,static_cast<int>(std::ceil(maxStep/0.025)));
      for(int k=1;k<=pieces;++k){double alpha=double(k)/pieces;std::vector<double> q;
        for(size_t j=0;j<a.size();++j)q.push_back(a[j]+alpha*(b[j]-a[j]));inspect(q);}}
    RCLCPP_INFO(n_->get_logger(),"[FULL VALIDATION] %zu samples passed",counter);
  }
  void verifyStart(const Plan& p,const std::string& combinedGroup) {
    auto live=group(combinedGroup).getCurrentState(10);if(!live)throw std::runtime_error("No live state before execute");
    const auto& t=p.trajectory_.joint_trajectory;
    if(t.points.empty())throw std::runtime_error("Empty plan");
    for(size_t i=0;i<t.joint_names.size();++i){double difference=std::abs(live->getVariablePosition(t.joint_names[i])-t.points.front().positions.at(i));
      if(difference>0.05)throw std::runtime_error("Robot moved during planning (stale start): "+t.joint_names[i]);}
  }
  void motion(const YAML::Node& step) {
    auto op=need(step,"goals");std::vector<Goal> goals;std::set<std::string> seen;
    for(const auto& item:op){auto g=goalOf(item);if(!seen.insert(g.robot).second)throw std::runtime_error("Duplicate active robot in step");goals.push_back(g);}
    if(goals.empty())throw std::runtime_error("Motion step has no goals");
    const auto combinedGroup=str(step,"group");const auto motion=str(step,"motion","moveJ");const auto strategy=str(step,"strategy","separate_merge");
    // Planning fallback candidates are entirely declarative and BEFORE execution.
    // Execution results are never retried or replayed.
    // Global, pre-execution planning-only retries. Never retry a submitted trajectory.
    std::vector<std::pair<std::string,std::string>> candidates{{motion,strategy}};
    if(step["fallbacks"]) for(const auto& fb:step["fallbacks"])
      candidates.emplace_back(str(fb,"motion",motion),str(fb,"strategy",strategy));
    // For flange actions, automatically fall back Pilz PTP -> OMPL moveJ.
    if(name_=="flange_pp" || name_=="flange_reverse_pp") {
      const auto fallbackMap=globalPlanning_["fallbacks"];
      const auto next=fallbackMap ? fallbackMap[motion] : YAML::Node();
      if(next && next.IsSequence()) for(const auto& fallback:next) {
        const auto fallbackMode=fallback.as<std::string>();
        bool exists=false;
        for(const auto& c:candidates) if(c.first==fallbackMode) exists=true;
        if(!exists) candidates.emplace_back(fallbackMode,strategy);
      }
    }
    Plan plan;bool found=false;std::string reason;
    for(const auto& candidate:candidates){
      const auto& mode=candidate.first;const auto& path=candidate.second;
      const auto policy=globalPlanning_[mode];
      if(!policy || !policy.IsMap()) throw std::runtime_error("No global planning policy for "+mode);
      std::vector<double> velocities;
      if(mode=="moveJ") {
        const int retries=static_cast<int>(num(policy,"retries",3));
        if(retries<1 || retries>20)throw std::runtime_error("moveJ retries must be 1..20");
        velocities.assign(retries,num(policy,"velocity",0.5));
      }else{
        auto seq=need(policy,"velocity_scales");
        if(!seq.IsSequence()||seq.size()==0)throw std::runtime_error("Empty global velocity schedule: "+mode);
        for(const auto& v:seq)velocities.push_back(v.as<double>());
      }
      for(size_t attempt=0;attempt<velocities.size();++attempt){
        const double velocity=velocities[attempt];checkScale(velocity);
        YAML::Node cfg=YAML::Clone(step);cfg["velocity"]=velocity;
        RCLCPP_INFO(n_->get_logger(),"[PLAN ATTEMPT] step=%s planner=%s strategy=%s velocity=%.2f attempt=%zu/%zu",
          str(step,"id").c_str(),mode.c_str(),path.c_str(),velocity,attempt+1,velocities.size());
        if(mode=="moveL")RCLCPP_INFO(n_->get_logger(),"[MOVE L PLANNER] step=%s backend=%s",
          str(step,"id").c_str(),str(cfg,"planner","pilz").c_str());
        bool planned=false;
        try {
          if(path=="separate_merge"){
            if(mode!="moveJ"&&mode!="moveL"&&mode!="movePTP"&&mode!="moveC")throw std::runtime_error("Unknown separate planner: "+mode);
            plan=merged(goals,strings(need(step,"participants")),combinedGroup,mode,cfg);
          }else if(path=="combined"){
            if(mode!="moveJ")throw std::runtime_error("Combined planning supports moveJ here");
            plan=combined(goals,combinedGroup,cfg);
          }else if(path=="coordinated_cartesian"){
            if(mode!="moveL")throw std::runtime_error("Coordinated Cartesian supports moveL here");
            plan=coordinatedCartesian(goals,combinedGroup,cfg);
          }else throw std::runtime_error("Unsupported planning strategy: "+path);
          planned=true;
          if(!dry_){validate(plan);verifyStart(plan,combinedGroup);}
          found=true;break;
        }catch(const std::exception& e){
          reason=e.what();
          if(planned && reason.find("Full-cell collision validation rejected")!=std::string::npos)
            previewRejectedPlan(plan,str(step,"id"),reason);
          RCLCPP_WARN(n_->get_logger(),"[PRE-EXEC RETRY] step=%s planner=%s velocity=%.2f attempt=%zu/%zu failed: %s",
            str(step,"id").c_str(),mode.c_str(),velocity,attempt+1,velocities.size(),reason.c_str());
        }
      }
      if(found)break;
    }
    if(!found)throw std::runtime_error("All pre-execution planning candidates failed: "+reason);
    if(dry_){RCLCPP_INFO(n_->get_logger(),"[DRY] parsed step=%s strategy=%s",str(step,"id").c_str(),strategy.c_str());return;}
    // Only MoveIt executes robot motion. It dispatches to configured controllers internally.
    const auto code=group(combinedGroup).execute(plan);
    if(code!=moveit::core::MoveItErrorCode::SUCCESS)throw std::runtime_error("MoveIt execution uncertain/failed code="+std::to_string(code.val));
  }
  // The scene/ACM operations below remain owned by the action engine, not the gantry node.
  void attachObject(const YAML::Node& step){
    const auto id=str(step,"object"),link=str(step,"link"),dock=str(step,"dock_frame");
    if(id.empty()||link.empty())throw std::runtime_error("attach requires object and link");
    if(dry_)return;
    auto objects=scene_.getObjects({id});
    if(!objects.count(id)&&flag(step,"optional",false))return;
    if(!objects.count(id))throw std::runtime_error("Attach object absent from world: "+id);
    auto source=objects.at(id);
    if(!source.header.frame_id.empty()&&source.header.frame_id!="world")throw std::runtime_error("Attached object must be world-frame: "+id);
    const auto mode=str(step,"placement_mode","preserve_world");
    if(mode=="preserve_world")source.pose=asPose(asTF(tfPose(link)).inverse()*asTF(source.pose));
    else if(mode=="dock_to_mount") {if(dock.empty())throw std::runtime_error("dock_to_mount needs dock_frame");source.pose=asPose(asTF(tfPose(dock)).inverse()*asTF(source.pose));}
    else throw std::runtime_error("Unknown placement_mode");
    source.header.frame_id=link;source.operation=moveit_msgs::msg::CollisionObject::ADD;
    moveit_msgs::msg::AttachedCollisionObject a;a.link_name=link;a.object=source;a.touch_links=strings(step["touch_links"]);
    auto original=objects.at(id);
    scene_.removeCollisionObjects({id});
    if(!scene_.applyAttachedCollisionObjects({a})) {
      original.operation=moveit_msgs::msg::CollisionObject::ADD;
      scene_.applyCollisionObject(original);
      throw std::runtime_error("Scene attach rejected; attempted world-object rollback: "+id);
    }
    bool stable=false;
    for(int i=0;i<25;++i){auto attached=scene_.getAttachedObjects({id});auto world=scene_.getObjects({id});
      if(attached.count(id)&&!world.count(id)&&attached.at(id).link_name==link){stable=true;break;}std::this_thread::sleep_for(100ms);}
    if(!stable)throw std::runtime_error("Scene attachment not verified: "+id);
  }
  void detachObject(const YAML::Node& step){
    auto id=str(step,"object"),link=str(step,"link");if(dry_)return;
    auto attached=scene_.getAttachedObjects({id});
    if(!attached.count(id)&&flag(step,"optional",false))return;
    if(!attached.count(id))throw std::runtime_error("Cannot detach unowned object: "+id);
    auto obj=attached.at(id);if(!link.empty()&&obj.link_name!=link)throw std::runtime_error("Wrong detach owner: "+id);
    auto world=obj.object;world.pose=asPose(asTF(tfPose(obj.link_name))*asTF(world.pose));
    world.header.frame_id="world";world.operation=moveit_msgs::msg::CollisionObject::ADD;
    moveit_msgs::msg::PlanningScene diff;diff.is_diff=true;diff.robot_state.is_diff=true;
    moveit_msgs::msg::AttachedCollisionObject remove;remove.link_name=obj.link_name;remove.object.id=id;remove.object.operation=moveit_msgs::msg::CollisionObject::REMOVE;
    diff.robot_state.attached_collision_objects.push_back(remove);diff.world.collision_objects.push_back(world);
    if(!scene_.applyPlanningScene(diff))throw std::runtime_error("Scene detach rejected: "+id);
    bool stable=false;for(int i=0;i<25;++i){if(!scene_.getAttachedObjects({id}).count(id)&&scene_.getObjects({id}).count(id)){stable=true;break;}std::this_thread::sleep_for(100ms);}
    if(!stable)throw std::runtime_error("Scene detach not verified: "+id);
  }
  moveit_msgs::msg::AllowedCollisionMatrix getAcm() {
    if(!getScene_->wait_for_service(10s))throw std::runtime_error("/get_planning_scene unavailable");
    auto req=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    req->components.components=moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto future=getScene_->async_send_request(req);if(future.wait_for(20s)!=std::future_status::ready){getScene_->remove_pending_request(future);throw std::runtime_error("ACM service timed out");}
    return future.get()->scene.allowed_collision_matrix;
  }
  std::optional<bool> pairValue(const moveit_msgs::msg::AllowedCollisionMatrix& acm,const std::string& a,const std::string& b){
    auto x=std::find(acm.entry_names.begin(),acm.entry_names.end(),a),y=std::find(acm.entry_names.begin(),acm.entry_names.end(),b);
    if(x==acm.entry_names.end()||y==acm.entry_names.end())return std::nullopt;
    auto i=x-acm.entry_names.begin(),j=y-acm.entry_names.begin();
    if(static_cast<size_t>(i)>=acm.entry_values.size()||static_cast<size_t>(j)>=acm.entry_values[i].enabled.size())throw std::runtime_error("Malformed ACM");
    return acm.entry_values[i].enabled[j];
  }
  size_t ensureAcmName(moveit_msgs::msg::AllowedCollisionMatrix& acm,const std::string& key){
    auto it=std::find(acm.entry_names.begin(),acm.entry_names.end(),key);if(it!=acm.entry_names.end())return it-acm.entry_names.begin();
    acm.entry_names.push_back(key);for(auto& row:acm.entry_values)row.enabled.push_back(false);
    moveit_msgs::msg::AllowedCollisionEntry x;x.enabled.resize(acm.entry_names.size(),false);acm.entry_values.push_back(x);return acm.entry_names.size()-1;
  }
  // ACM normalization can remove explicit entries for attached objects. An
  // absent pair is NOT necessarily allowed: inspect default permissions.
  bool effectivePermission(const moveit_msgs::msg::AllowedCollisionMatrix& acm,
                           const std::string& x,const std::string& y){
    const auto explicitValue=pairValue(acm,x,y);
    if(explicitValue.has_value())return *explicitValue;
    auto defaultOf=[&](const std::string& name)->std::optional<bool>{
      const auto it=std::find(acm.default_entry_names.begin(),acm.default_entry_names.end(),name);
      if(it==acm.default_entry_names.end())return std::nullopt;
      const auto k=static_cast<size_t>(it-acm.default_entry_names.begin());
      if(k>=acm.default_entry_values.size())throw std::runtime_error("Malformed ACM default entries");
      return acm.default_entry_values[k];
    };
    const auto dx=defaultOf(x),dy=defaultOf(y);
    // Conservative: if either default permits contact, don't call the
    // missing explicit pair a verified forbidden contact.
    return dx.value_or(false)||dy.value_or(false);
  }
  void setAcm(const std::string& x,const std::string& y,bool allow,bool restore){
    if(x.empty()||y.empty()||x==y)throw std::runtime_error("Invalid ACM pair");
    auto key=std::minmax(x,y);std::pair<std::string,std::string> pair{key.first,key.second};
    if(dry_)return;
    Lock guard(acmLock_);
    if(restore&&!saved_.count(pair))throw std::runtime_error("ACM restore without saved previous value: "+x+"/"+y);
    // Record the effective prior permission, not only an explicitly named pair.
    if(!restore&&!saved_.count(pair)){
      auto before=getAcm();saved_[pair]=effectivePermission(before,x,y);
    }
    const bool value=restore?saved_[pair].value_or(false):allow;
    std::string lastObserved="unread";
    // Each retry merges into the latest scene; don't replace other concurrent changes.
    for(int attempt=0;attempt<5;++attempt){
      auto acm=getAcm();
      const auto i=ensureAcmName(acm,x),j=ensureAcmName(acm,y);
      acm.entry_values[i].enabled[j]=value;acm.entry_values[j].enabled[i]=value;
      moveit_msgs::msg::PlanningScene diff;
      diff.is_diff=true;diff.robot_state.is_diff=true;diff.allowed_collision_matrix=acm;
      if(!scene_.applyPlanningScene(diff)){
        lastObserved="applyPlanningScene rejected update";
      }else{
        for(int poll=0;poll<5;++poll){
          const auto seen=getAcm();
          const auto explicitValue=pairValue(seen,x,y);
          const bool effective=effectivePermission(seen,x,y);
          lastObserved=explicitValue.has_value()?(*explicitValue?"explicit true":"explicit false"):
            (effective?"absent but default permits":"absent and default forbids");
          // An absent pair with no permissive default is a valid restored
          // false; it is NOT an acceptable way to confirm an allow=true.
          if(explicitValue==std::optional<bool>(value) || (!value&&!effective)){
            if(restore)saved_.erase(pair);
            return;
          }
          std::this_thread::sleep_for(100ms);
        }
      }
      std::this_thread::sleep_for(100ms);
    }
    throw std::runtime_error("ACM verification failed: "+x+"/"+y+
       " expected="+(value?"allow":"forbid")+" observed="+lastObserved);
  }
  void restoreOutstanding(){
    if(dry_){saved_.clear();return;}
    const auto previous=saved_;
    std::vector<std::string> failures;
    for(const auto& p:previous){
      try{setAcm(p.first.first,p.first.second,false,true);}
      catch(const std::exception& e){failures.push_back(e.what());
        RCLCPP_ERROR(n_->get_logger(),"[ACM CLEANUP] %s",e.what());}
    }
    if(!failures.empty())throw std::runtime_error(
      "ACM cleanup could not verify "+std::to_string(failures.size())+" pair(s); inspect log");
  }
  void cancelPendingGantry(){
    std::vector<std::string> toCancel;
    {
      std::lock_guard<std::mutex> lock(gantryMutex_);
      for(const auto& id:pendingGantry_) {
        const auto it=gantryResults_.find(id);
        if(it==gantryResults_.end() || (it->second.first!="DONE" && it->second.first!="ERROR"))
          toCancel.push_back(id);
      }
    }
    for(const auto& id:toCancel) {
      std_msgs::msg::String msg;msg.data=id;gantryCancel_->publish(msg);
      RCLCPP_WARN(n_->get_logger(),"[GANTRY] cancel requested id=%s",id.c_str());
    }
  }
  std::pair<std::string,std::string> waitGantryStatus(const std::string& id,double timeout,bool acceptanceOnly){
    if(timeout<=0||!std::isfinite(timeout))throw std::runtime_error("Invalid gantry timeout");
    std::unique_lock<std::mutex> lock(gantryMutex_);
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(timeout);
    while(rclcpp::ok()) {
      const auto it=gantryResults_.find(id);
      if(it!=gantryResults_.end()) {
        if(it->second.first=="DONE"){
          pendingGantry_.erase(id);
          return it->second;
        }
        if(it->second.first=="ERROR" || it->second.first=="REJECTED"){
          pendingGantry_.erase(id);
          throw std::runtime_error("Gantry "+id+": "+it->second.first+": "+it->second.second);
        }
        if(acceptanceOnly && it->second.first=="RUNNING")return it->second;
      }
      if(gantryCv_.wait_until(lock,deadline)==std::cv_status::timeout)break;
    }
    throw std::runtime_error("Gantry "+id+" did not report "+(acceptanceOnly?"acceptance":"completion")+" within "+std::to_string(timeout)+"s");
  }
  void gantry(const YAML::Node& op){
    if(dry_)return;
    // YAML step id is the public command name. The topic bridge still uses
    // request_id internally to correlate controller status with this command.
    const std::string stepId=str(op,"id");
    if(stepId.empty())throw std::runtime_error("gantry_send requires id");
    const std::string id=instance_+"__"+stepId;
    const double x=num(op,"x",0),z=num(op,"z",0);
    const double speed=num(op,"velocity",0.4),acc=num(op,"acceleration",0.4);
    if(!std::isfinite(x)||!std::isfinite(z)||!std::isfinite(speed)||!std::isfinite(acc)||speed<=0||acc<=0)
      throw std::runtime_error("Invalid gantry target/speed/acceleration");
    auto start=std::chrono::steady_clock::now();
    while(gantryCmd_->get_subscription_count()==0){
      if(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()>10)
        throw std::runtime_error("Gantry JTC topic subscriber unavailable");
      std::this_thread::sleep_for(100ms);
    }
    {
      std::lock_guard<std::mutex> lock(gantryMutex_);
      if(!pendingGantry_.insert(id).second)throw std::runtime_error("Duplicate gantry request id: "+id);
      gantryResults_.erase(id);
    }
    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "request_id" << YAML::Value << id
        << YAML::Key << "x" << YAML::Value << x << YAML::Key << "z" << YAML::Value << z
        << YAML::Key << "velocity" << YAML::Value << speed
        << YAML::Key << "acceleration" << YAML::Value << acc << YAML::EndMap;
    std_msgs::msg::String command;command.data=out.c_str();gantryCmd_->publish(command);
    RCLCPP_INFO(n_->get_logger(),"[GANTRY TOPIC] requested id=%s x=%.3f z=%.3f speed=%.3f acc=%.3f",
      id.c_str(),x,z,speed,acc);
    waitGantryStatus(id,20.0,true);
    // Sequential by default. Explicit wait: false allows asynchronous execution.
    if(flag(op,"wait",true))waitGantry(id,num(op,"timeout",120));
  }
  void waitGantry(const std::string& id,double timeout){
    if(dry_)return;
    const auto result=waitGantryStatus(id,timeout,false);
    RCLCPP_INFO(n_->get_logger(),"[GANTRY TOPIC] completed id=%s status=%s",id.c_str(),result.first.c_str());
  }
  void stepRun(const YAML::Node& op){
    if(op["enabled"]&&!op["enabled"].as<bool>())return;
    const auto id=str(op,"id",str(op,"op")),kind=str(op,"op");
    RCLCPP_INFO(n_->get_logger(),"[ACTION STEP] %s / %s",name_.c_str(),id.c_str());
    if(kind=="move"){
      // Run pre-motion scene operations ONCE, even if planning is retried.
      if(op["before"])for(const auto& item:op["before"])stepRun(item);
      // step_retries means EXTRA full planning rounds after the first one.
      // Each round retains the existing per-planner velocity/fallback attempts.
      // Never replay a robot trajectory after execution has been submitted.
      const int extraRetries=op["step_retries"]
        ? op["step_retries"].as<int>()
        : static_cast<int>(num(globalPlanning_,"step_retries",3));
      const double delay=op["retry_delay_s"]
        ? op["retry_delay_s"].as<double>()
        : num(globalPlanning_,"retry_delay_s",0.5);
      if(extraRetries<0||extraRetries>20)throw std::runtime_error("step_retries must be 0..20");
      if(!std::isfinite(delay)||delay<0||delay>60)throw std::runtime_error("retry_delay_s must be 0..60");
      for(int round=0;round<=extraRetries;++round){
        RCLCPP_INFO(n_->get_logger(),"[STEP ATTEMPT] action=%s step=%s round=%d/%d",
          name_.c_str(),id.c_str(),round+1,extraRetries+1);
        try {
          motion(op);
          RCLCPP_INFO(n_->get_logger(),"[STEP SUCCESS] action=%s step=%s round=%d/%d",
            name_.c_str(),id.c_str(),round+1,extraRetries+1);
          break;
        }catch(const std::exception& e){
          const std::string reason=e.what();
          // Only retry errors thrown BEFORE MoveIt execution is called.
          // In particular, 'MoveIt execution uncertain/failed' must NOT replay.
          const bool planningFailed=reason.rfind("All pre-execution planning candidates failed:",0)==0;
          if(!planningFailed||round==extraRetries){
            RCLCPP_ERROR(n_->get_logger(),"[STEP FAILED] action=%s step=%s round=%d/%d: %s",
              name_.c_str(),id.c_str(),round+1,extraRetries+1,reason.c_str());
            throw;
          }
          RCLCPP_WARN(n_->get_logger(),
            "[STEP RETRY] action=%s step=%s failed round=%d/%d; retrying only this step: %s",
            name_.c_str(),id.c_str(),round+1,extraRetries+1,reason.c_str());
          if(delay>0)std::this_thread::sleep_for(std::chrono::duration<double>(delay));
        }
      }
      // Run post-motion scene operations ONCE, only after motion succeeds.
      if(op["after"])for(const auto& item:op["after"])stepRun(item);
    }else if(kind=="attach")attachObject(op);
    else if(kind=="detach")detachObject(op);
    else if(kind=="acm_allow"||kind=="acm_restore"){
      for(const auto& p:need(op,"pairs")){if(!p.IsSequence()||p.size()!=2)throw std::runtime_error("ACM pairs require [name,name]");
        setAcm(p[0].as<std::string>(),p[1].as<std::string>(),kind=="acm_allow",kind=="acm_restore");}
    }else if(kind=="gantry_send")gantry(op);
    else if(kind=="gantry_wait") {
      // Optional legacy async operation: target_id identifies the gantry_send
      // step to join. Fall back to request_id for older YAML configurations.
      const auto targetId=str(op,"target_id",str(op,"request_id"));
      if(targetId.empty())throw std::runtime_error("gantry_wait requires target_id");
      waitGantry(instance_+"__"+targetId,num(op,"timeout",120));
    }
    else if(kind=="sleep") {if(!dry_)std::this_thread::sleep_for(std::chrono::duration<double>(num(op,"seconds",0)));}
    else throw std::runtime_error("Unknown action step operation: "+kind);
  }
  rclcpp::Node::SharedPtr n_;tf2_ros::Buffer tf_;std::shared_ptr<tf2_ros::TransformListener> listener_;
  moveit::planning_interface::PlanningSceneInterface scene_;
  std::map<std::string,std::unique_ptr<MGI>> groups_;
  bool clientStartupSettled_{false};
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr validity_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr getScene_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gantryCmd_,gantryCancel_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr rejectedPublisher_;
  bool previewRejected_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gantryStatus_;
  std::mutex gantryMutex_;std::condition_variable gantryCv_;
  std::map<std::string,std::pair<std::string,std::string>> gantryResults_;
  std::set<std::string> pendingGantry_;
  std::map<std::pair<std::string,std::string>,std::optional<bool>> saved_;
  std::map<std::string,std::string> robotGroup_,robotLink_,robotTyingLink_,robotToolMount_;
  YAML::Node globalPlanning_;
  YAML::Node params_,action_,workcell_;std::string name_,fullGroup_,acmLock_,instance_;bool dry_{true},sim_{false},fullCellCollisionCheck_{true};
};

int main(int argc,char** argv){rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("sa3_action_engine");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),4);
  executor.add_node(node);std::thread spin([&]{executor.spin();});int rc=1;
  try{ActionEngine app(node);rc=app.run();}catch(const std::exception& e){RCLCPP_ERROR(node->get_logger(),"ACTION INIT FAILED: %s",e.what());}
  executor.cancel();if(spin.joinable())spin.join();rclcpp::shutdown();return rc;
}
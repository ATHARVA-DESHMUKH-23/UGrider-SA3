// SA3 generic YAML task executor — ROS 2 Humble/Jazzy / MoveIt 2.
// One executable handles all supported task types. One process per task is
// intentional: launch owns sequencing, while persistent gantry owns G4/R3 holds.
// WARNING: NOT hardware-certified. New implementation, validate in mock/RViz.
// Defaults to dry_run=true, and rejects unsupported/uncertain operations.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>

using namespace std::chrono_literals;
using MGI = moveit::planning_interface::MoveGroupInterface;
using Plan = MGI::Plan;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
using JointPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using Trigger = std_srvs::srv::Trigger;
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

namespace {
std::string s(const YAML::Node& n,const std::string& key,const std::string& d="") {
  return n && n[key] ? n[key].as<std::string>() : d;
}
double f(const YAML::Node& n,const std::string& key,double d) {
  return n && n[key] ? n[key].as<double>() : d;
}
int integer(const YAML::Node& n,const std::string& key,int d) {
  return n && n[key] ? n[key].as<int>() : d;
}
bool b(const YAML::Node& n,const std::string& key,bool d=false) {
  return n && n[key] ? n[key].as<bool>() : d;
}
YAML::Node require(const YAML::Node& n,const std::string& key) {
  if(!n || !n[key]) throw std::runtime_error("Missing YAML key: "+key);
  return n[key];
}
std::vector<std::string> strs(const YAML::Node& n) {
  return n && n.IsSequence() ? n.as<std::vector<std::string>>() : std::vector<std::string>{};
}
std::vector<double> doubles(const YAML::Node& n) {
  return n && n.IsSequence() ? n.as<std::vector<double>>() : std::vector<double>{};
}
std::array<double,3> vec(const YAML::Node& n) {
  if(!n) return {0,0,0};
  if(!n.IsSequence() || n.size()!=3) throw std::runtime_error("Expected [x,y,z] vector");
  std::array<double,3> v{};
  for(int i=0;i<3;++i) {v[i]=n[i].as<double>();if(!std::isfinite(v[i])) throw std::runtime_error("Nonfinite offset");}
  return v;
}
void checkScale(double x) {if(!std::isfinite(x)||x<=0||x>1)throw std::runtime_error("Invalid velocity/acceleration scale");}
double seconds(Clock::time_point t) {return std::chrono::duration<double>(Clock::now()-t).count();}
double timeOf(const JointPoint& p) {return rclcpp::Duration(p.time_from_start).seconds();}
void setTime(JointPoint& p,double t) {
  auto ns=static_cast<std::int64_t>(std::llround(t*1e9));
  p.time_from_start.sec=static_cast<std::int32_t>(ns/1000000000LL);
  p.time_from_start.nanosec=static_cast<std::uint32_t>(ns%1000000000LL);
}
tf2::Transform tfPose(const geometry_msgs::msg::Pose& p) {
  return tf2::Transform(
      tf2::Quaternion(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w),
      tf2::Vector3(p.position.x,p.position.y,p.position.z));
}
geometry_msgs::msg::Pose poseTf(const tf2::Transform& t) {
  geometry_msgs::msg::Pose p; auto q=t.getRotation();q.normalize();
  p.position.x=t.getOrigin().x();p.position.y=t.getOrigin().y();p.position.z=t.getOrigin().z();
  p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w();return p;
}
geometry_msgs::msg::Pose applyOffset(geometry_msgs::msg::Pose p,
   const std::array<double,3>& xyz,const std::string& xyzFrame,
   const std::array<double,3>& rpy={0,0,0},const std::string& rpyFrame="local") {
  tf2::Quaternion q(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w);q.normalize();
  tf2::Vector3 d(xyz[0],xyz[1],xyz[2]);
  if(xyzFrame=="local")d=tf2::quatRotate(q,d);
  else if(xyzFrame!="world")throw std::runtime_error("offset_frame must be local/world");
  p.position.x+=d.x();p.position.y+=d.y();p.position.z+=d.z();
  tf2::Quaternion dq;dq.setRPY(rpy[0],rpy[1],rpy[2]);
  auto res=rpyFrame=="local" ? q*dq : dq*q;
  if(rpyFrame!="local" && rpyFrame!="world")throw std::runtime_error("rpy_frame must be local/world");
  res.normalize();p.orientation.x=res.x();p.orientation.y=res.y();p.orientation.z=res.z();p.orientation.w=res.w();
  return p;
}
std::string csvEscape(const std::string& a) {
  std::string out="\""; for(char c:a){if(c=='\"')out+='\"';out+=c;}return out+"\"";
}
struct Row {
  std::string phase,task,robot,object,point,step,motion,planner,status,error;
  int attempt{1};double velocity{0},planning{0},validation{0},execution{0},scene{0},elapsed{0};
};
struct ArmDef {std::string name,group,link,tool_mount,tying_link,web_ring;
  std::vector<std::string> touch;};
struct Tuning {double planning_time{15},vel{0.3},acc{0.3},ik{1};int attempts{10},retries{3};};
struct ArmGoal {std::string robot,link;geometry_msgs::msg::Pose pose;};
struct MotionAttempt {std::string planner;double scale{0};double planning{0},validation{0},execution{0};bool issued{false};};
struct ContextPair {std::string robot,object,inventory,ring,link;std::vector<std::string> touch;};

JointPoint interpolate(const JointTrajectory& jt,double t,double stretch) {
  JointPoint p;if(jt.points.empty())return p;
  const double src=t/stretch;
  size_t hi=0;while(hi+1<jt.points.size() && timeOf(jt.points[hi])<src)++hi;
  size_t lo=hi>0 ? hi-1:hi;
  if(src>=timeOf(jt.points.back()))lo=hi=jt.points.size()-1;
  auto &a=jt.points[lo], &z=jt.points[hi];
  const double dt=timeOf(z)-timeOf(a),alpha=dt>1e-9?std::clamp((src-timeOf(a))/dt,0.,1.):0.;
  auto blend=[&](const auto& v,const auto& w,auto& target,double factor) {
    if(v.size()!=jt.joint_names.size()||w.size()!=v.size())return;
    for(size_t i=0;i<v.size();++i)target.push_back((v[i]+alpha*(w[i]-v[i]))*factor);
  };
  blend(a.positions,z.positions,p.positions,1);
  blend(a.velocities,z.velocities,p.velocities,1/stretch);
  blend(a.accelerations,z.accelerations,p.accelerations,1/(stretch*stretch));
  return p;
}
JointTrajectory hold(const JointTrajectory& model,const std::vector<double>& current,double duration) {
  JointTrajectory jt;jt.joint_names=model.joint_names;
  if(current.size()!=jt.joint_names.size())throw std::runtime_error("Hold joint count mismatch");
  JointPoint a;a.positions=current;a.velocities.assign(current.size(),0);a.accelerations.assign(current.size(),0);
  setTime(a,0);jt.points.push_back(a);setTime(a,std::max(duration,0.01));jt.points.push_back(a);return jt;
}
JointTrajectory mergeTrajectories(JointTrajectory a,JointTrajectory z) {
  if(a.points.empty()||z.points.empty())throw std::runtime_error("Cannot merge empty trajectory");
  const double da=timeOf(a.points.back()),dz=timeOf(z.points.back());
  if(da<=0||dz<=0)throw std::runtime_error("Cannot merge zero-duration trajectory");
  const double total=std::max(da,dz),fa=total/da,fz=total/dz;
  std::set<std::string> unique(a.joint_names.begin(),a.joint_names.end());
  for(const auto& name:z.joint_names)if(!unique.insert(name).second)throw std::runtime_error("Overlapping joint in paired plans: "+name);
  JointTrajectory result;result.joint_names=a.joint_names;
  result.joint_names.insert(result.joint_names.end(),z.joint_names.begin(),z.joint_names.end());
  std::vector<double> times{0.,total};
  for(const auto& p:a.points)times.push_back(timeOf(p)*fa);
  for(const auto& p:z.points)times.push_back(timeOf(p)*fz);
  std::sort(times.begin(),times.end());
  times.erase(std::unique(times.begin(),times.end(),[](double x,double y){return std::abs(x-y)<1e-9;}),times.end());
  for(double t:times) {
    auto x=interpolate(a,t,fa),y=interpolate(z,t,fz);
    if(x.positions.size()!=a.joint_names.size()||y.positions.size()!=z.joint_names.size())throw std::runtime_error("Invalid merged position array");
    JointPoint p;p.positions=x.positions;p.positions.insert(p.positions.end(),y.positions.begin(),y.positions.end());
    if(x.velocities.size()==a.joint_names.size()&&y.velocities.size()==z.joint_names.size()) {
      p.velocities=x.velocities;p.velocities.insert(p.velocities.end(),y.velocities.begin(),y.velocities.end());
    }
    if(x.accelerations.size()==a.joint_names.size()&&y.accelerations.size()==z.joint_names.size()) {
      p.accelerations=x.accelerations;p.accelerations.insert(p.accelerations.end(),y.accelerations.begin(),y.accelerations.end());
    }
    setTime(p,t);result.points.push_back(std::move(p));
  }
  return result;
}
} // namespace

class GenericExecutor {
 public:
  explicit GenericExecutor(const rclcpp::Node::SharedPtr& n)
   : node_(n),tf_(n->get_clock()) {
    tf_.setUsingDedicatedThread(true);
    listener_=std::make_shared<tf2_ros::TransformListener>(tf_,node_,false);
    configPath_=param<std::string>("motion_config_path","");
    if(configPath_.empty())throw std::runtime_error("motion_config_path is mandatory: no fallback to installed recipe");
    recipe_=require(YAML::LoadFile(configPath_),"motion_sequence");
    if(integer(recipe_,"schema_version",0)!=3)throw std::runtime_error("Generic executor requires schema_version: 3");
    taskId_=param<std::string>("motion_task","");phase_=param<std::string>("stage_id","");
    dryRun_=param<bool>("dry_run",true);
    workcellPath_=param<std::string>("workcell_config_path","");
    if(workcellPath_.empty()) {
      auto w=require(recipe_,"workcell");
      workcellPath_=(fs::path(ament_index_cpp::get_package_share_directory(s(w,"package"))) / s(w,"file")).string();
    }
    workcell_=require(YAML::LoadFile(workcellPath_),"workcell");
    if(s(workcell_,"world_frame","world")!="world")throw std::runtime_error("Workcell world frame must be world");
    for(auto name:{"robot1","robot2","robot3","robot4"}) {
      auto v=require(require(workcell_,"robots"),name);ArmDef arm;
      arm.name=name;arm.group=s(v,"group");
      arm.link=s(v,"planning_link",s(v,"link"));
      arm.tying_link=s(v,"tying_link",arm.link);
      arm.tool_mount=s(v,"tool_mount_link",std::string(name)+"_tool_mount");
      arm.web_ring=s(v,"web_ring");
      arm.touch=strs(v["touch_links"]);if(arm.touch.empty())arm.touch=strs(v["gripper_touch_links"]);
      if(arm.group.empty()||arm.link.empty())throw std::runtime_error("Missing fixed workcell robot group/link: "+std::string(name));
      arms_[name]=arm;
    }
    getScene_=node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    applyScene_=node_->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
    validity_=node_->create_client<moveit_msgs::srv::GetStateValidity>("/check_state_validity");
    jointSub_=node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states",20,[this](sensor_msgs::msg::JointState::SharedPtr msg){
      std::lock_guard<std::mutex> lock(jointMutex_);
      for(size_t i=0;i<msg->name.size()&&i<msg->position.size();++i)jointPositions_[msg->name[i]]=msg->position[i];
    });
    start_=Clock::now();
  }
  ~GenericExecutor(){try{writeReport();}catch(...){}}
  int run() {
    try {
      if(!taskId_.empty()) {
        task_=require(require(recipe_,"tasks"),taskId_);
        runTask();
      } else {
        auto key=param<std::string>("combined_step","");
        if(key.empty())throw std::runtime_error("Must specify motion_task or combined_step");
        runCombined(require(require(recipe_,"combined_motions"),key),key);
      }
      RCLCPP_INFO(node_->get_logger(),"GENERIC TASK SUCCEEDED: %s",taskId_.c_str());writeReport();return 0;
    } catch(const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(),"SA3 ABORT %s: %s",taskId_.c_str(),e.what());
      if(b(task_["execution"],"notify_robot3_on_failure",false))notifyCompletion("robot3",false);
      if(s(task_,"completion_owner")=="robot12")
        notifyCompletion("robot12",false);
      rows_.push_back(Row{phase_,taskId_,"","","","TASK","","","FAILED",e.what()});
      writeReport();return 1;
    }
  }
 private:
  template<class T>T param(const std::string& k,const T& value) {
    if(!node_->has_parameter(k))node_->declare_parameter<T>(k,value);
    return node_->get_parameter(k).get_value<T>();
  }
  void report(const Row& row) {
    rows_.push_back(row);
    RCLCPP_INFO(node_->get_logger(),"[SA3 TIMING] task=%s step=%s robot=%s planner=%s planning=%.3f validation=%.3f execution=%.3f scene=%.3f status=%s",
      row.task.c_str(),row.step.c_str(),row.robot.c_str(),row.planner.c_str(),row.planning,row.validation,row.execution,row.scene,row.status.c_str());
  }
  void writeReport() {
    if(reportWritten_||rows_.empty()) return;
    reportWritten_=true;
    try {
      std::string dir=s(recipe_["reporting"],"output_directory","logs/sa3_runs");
      fs::create_directories(dir);
      const auto stamp=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      const std::string filename=(fs::path(dir)/("sa3_"+taskId_+"_"+std::to_string(stamp)+".csv")).string();
      std::ofstream output(filename);output<<"phase,task,robot,object_id,point_id,step_id,motion,planner,velocity_scale,attempt,planning_seconds,validation_seconds,execution_seconds,scene_seconds,elapsed_seconds,status,error\n";
      for(const auto& r:rows_) output<<csvEscape(r.phase)<<","<<csvEscape(r.task)<<","<<csvEscape(r.robot)<<","<<csvEscape(r.object)<<","<<csvEscape(r.point)<<","<<csvEscape(r.step)<<","<<csvEscape(r.motion)<<","<<csvEscape(r.planner)<<","<<r.velocity<<","<<r.attempt<<","<<r.planning<<","<<r.validation<<","<<r.execution<<","<<r.scene<<","<<r.elapsed<<","<<csvEscape(r.status)<<","<<csvEscape(r.error)<<"\n";
      RCLCPP_INFO(node_->get_logger(),"[SA3 REPORT] %s",filename.c_str());
    } catch(const std::exception& e){RCLCPP_ERROR(node_->get_logger(),"Cannot save execution report: %s",e.what());}
  }
  MGI& group(const std::string& name) {
    if(name.empty())throw std::runtime_error("Empty SRDF group");
    auto it=groups_.find(name);
    if(it==groups_.end())it=groups_.emplace(name,std::make_unique<MGI>(node_,name)).first;
    return *it->second;
  }
  ArmDef arm(const std::string& name,const std::string& special="") const {
    auto it=arms_.find(name);if(it==arms_.end())throw std::runtime_error("Unknown arm: "+name);
    auto a=it->second;if(special=="tying")a.link=a.tying_link;else if(special=="tool")a.link=a.tool_mount;
    return a;
  }
  geometry_msgs::msg::Pose framePose(const std::string& frame) {
    if(frame.empty())throw std::runtime_error("Missing target TF frame");
    auto t=tf_.lookupTransform("world",frame,tf2::TimePointZero,tf2::durationFromSec(10));
    geometry_msgs::msg::Pose p;p.position.x=t.transform.translation.x;p.position.y=t.transform.translation.y;p.position.z=t.transform.translation.z;p.orientation=t.transform.rotation;return p;
  }
  geometry_msgs::msg::Pose resolveTarget(const YAML::Node& spec,const YAML::Node& taskArm={}) {
    auto ref=s(spec,"reference");
    if(ref=="pick_frame")ref=s(taskArm,"pick_frame");
    else if(ref=="place_frame")ref=s(taskArm,"place_frame");
    if(ref=="home")throw std::runtime_error("Cannot use home as Cartesian target");
    auto pose=framePose(ref);
    if(ref==s(taskArm,"pick_frame") && taskArm && taskArm["pick_offset"]) {
      pose=applyOffset(pose,vec(taskArm["pick_offset"]),s(taskArm,"pick_offset_frame","world"),
           vec(taskArm["pick_offset_rpy"]),s(taskArm,"pick_offset_rpy_frame","local"));
    } else if(ref==s(taskArm,"place_frame") && taskArm && taskArm["place_offset"]) {
      pose=applyOffset(pose,vec(taskArm["place_offset"]),s(taskArm,"place_offset_frame","world"),
           vec(taskArm["place_offset_rpy"]),s(taskArm,"place_offset_rpy_frame","local"));
    }
    return applyOffset(pose,vec(spec["xyz"]),s(spec,"offset_frame","world"),
      vec(spec["rpy"]),s(spec,"rpy_frame","local"));
  }
  YAML::Node planSettings() const {
    if(task_["planning"])return task_["planning"];
    if(task_["config"] && task_["config"]["planning"])return task_["config"]["planning"];
    return {};
  }
  Tuning tuning(const std::string& motion,double forcedVel=-1) const {
    auto p=planSettings();Tuning t;t.planning_time=f(p,"planning_time",15);
    t.attempts=integer(p,"planning_attempts",integer(p,"ompl_attempts",10));
    t.retries=integer(p,"retries",integer(p,"plan_retries",integer(p,"movej_plan_retries",3)));
    t.ik=f(p,"ik_timeout",1);
    bool joint=motion=="moveJ"||motion=="movePTP"||motion=="named_state";
    t.vel=f(p,joint?"movej_velocity":"movel_velocity",0.3);
    t.acc=f(p,joint?"movej_acceleration":"movel_acceleration",0.3);
    if(motion=="moveC")t.acc=f(p,"movec_acceleration",t.acc);
    if(forcedVel>0)t.vel=forcedVel;
    if(t.retries<1||t.attempts<1||t.ik<=0||t.planning_time<=0)throw std::runtime_error("Invalid planning settings");
    checkScale(t.vel);checkScale(t.acc);return t;
  }
  void setPlanning(MGI& mg,const std::string& motion,const Tuning& t) {
    mg.clearPoseTargets();mg.clearPathConstraints();mg.setPoseReferenceFrame("world");
    if(motion=="moveL"||motion=="moveC"||motion=="movePTP") {
      mg.setPlanningPipelineId("pilz_industrial_motion_planner");
      mg.setPlannerId(motion=="movePTP"?"PTP":motion=="moveC"?"CIRC":"LIN");
      mg.setNumPlanningAttempts(1);
    } else {mg.setPlanningPipelineId("ompl");mg.setPlannerId("RRTConnectkConfigDefault");mg.setNumPlanningAttempts(t.attempts);}
    mg.setPlanningTime(t.planning_time);mg.setMaxVelocityScalingFactor(t.vel);mg.setMaxAccelerationScalingFactor(t.acc);
  }
  std::shared_ptr<moveit::core::RobotState> currentState(MGI& mg) {
    auto st=mg.getCurrentState(10);
    if(!st)throw std::runtime_error("No fresh MoveIt robot state");
    return st;
  }
  bool setPoseIK(moveit::core::RobotState& state,const ArmGoal& g,double timeout) {
    const auto jmg=state.getJointModelGroup(arm(g.robot).group);
    return jmg && state.setFromIK(jmg,g.pose,g.link,timeout);
  }
  // Called only after a shear P1 pose-goal plan fails. The original shear node
  // used OMPL pose constraints rather than forcing one IK joint target.
  // Show START-state contacts instead of repeatedly guessing ACM entries.
  void diagnoseShearP1Start(const std::string& robot,
      const moveit_msgs::msg::RobotState& start) {
    try {
      if(!validity_->wait_for_service(2s)) {
        RCLCPP_WARN(node_->get_logger(), "[SHEAR P1 DIAG] /check_state_validity unavailable");
        return;
      }
      auto req=std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
      req->group_name=fullCollisionGroup_;
      req->robot_state=start;
      req->robot_state.is_diff=true;
      req->robot_state.attached_collision_objects.clear();
      auto future=validity_->async_send_request(req);
      if(future.wait_for(3s)!=std::future_status::ready) {
        validity_->remove_pending_request(future);
        RCLCPP_WARN(node_->get_logger(), "[SHEAR P1 DIAG] Start-state service timed out for %s",robot.c_str());
        return;
      }
      const auto result=future.get();
      RCLCPP_WARN(node_->get_logger(),
        "[SHEAR P1 DIAG] %s full-group START valid=%d contacts=%zu",
        robot.c_str(),int(result->valid),result->contacts.size());
      for(const auto& contact:result->contacts) {
        RCLCPP_ERROR(node_->get_logger(),
          "[SHEAR P1 START COLLISION] %s <-> %s",
          contact.contact_body_1.c_str(),contact.contact_body_2.c_str());
      }
    } catch(const std::exception& e) {
      RCLCPP_WARN(node_->get_logger(),"[SHEAR P1 DIAG] %s",e.what());
    }
  }

  Plan planIndividual(const ArmGoal& target,const std::string& motion,const Tuning& t,
      const moveit_msgs::msg::RobotState* start=nullptr,
      const geometry_msgs::msg::Pose* via=nullptr,
      bool shearP1PoseGoal=false) {
    auto& mg=group(arm(target.robot).group);
    setPlanning(mg,motion,t);
    if(start)mg.setStartState(*start);else mg.setStartStateToCurrentState();
    if(shearP1PoseGoal && motion!="moveJ")
      throw std::runtime_error("shearP1PoseGoal requires moveJ");
    if(shearP1PoseGoal) {
      // Match shear_links_pp.cpp planOmpl(): a Cartesian pose GOAL lets OMPL
      // sample feasible IK branches rather than fixating on one IK solution.
      mg.setGoalPositionTolerance(0.001);
      mg.setGoalOrientationTolerance(0.01);
      if(!mg.setPoseTarget(target.pose,target.link))
        throw std::runtime_error("Shear P1 pose target rejected for "+target.robot);
    } else if(motion=="moveJ"||motion=="movePTP") {
      auto st=currentState(mg);auto goal=std::make_shared<moveit::core::RobotState>(*st);
      if(!setPoseIK(*goal,target,t.ik))throw std::runtime_error("IK failed for "+target.robot);
      if(!mg.setJointValueTarget(*goal))throw std::runtime_error("Joint goal rejected: "+target.robot);
    } else if(motion=="moveL"||motion=="moveC") {
      if(motion=="moveC") {
        if(!via)throw std::runtime_error("Pilz CIRC requires explicit interim point");
        moveit_msgs::msg::PositionConstraint pc;
        pc.header.frame_id="world";pc.link_name=target.link;
        pc.constraint_region.primitive_poses.push_back(*via);pc.weight=1.0;
        moveit_msgs::msg::Constraints constraints;constraints.name="interim";
        constraints.position_constraints.push_back(pc);mg.setPathConstraints(constraints);
      }
      if(!mg.setPoseTarget(target.pose,target.link))throw std::runtime_error("Pilz pose target rejected: "+target.robot);
    } else throw std::runtime_error("Unsupported individual planner: "+motion);

    Plan plan;
    int lastCode=0;
    // Configured LIN retries apply to program motions such as D5. The default
    // remains one attempt, so existing shear velocity/fallback logic is intact.
    const auto planningCfg=planSettings();
    const bool linearRetry=(motion=="moveL" && !shearP1PoseGoal);
    const int attempts=shearP1PoseGoal?t.retries:
      (linearRetry?integer(planningCfg,"linear_plan_retries",1):1);
    const double retryDelay=f(planningCfg,"linear_plan_retry_delay_sec",0.25);
    if(attempts<1 || attempts>10 || !std::isfinite(retryDelay) || retryDelay<0 || retryDelay>30)
      throw std::runtime_error("Invalid linear planning retry settings");
    for(int attempt=1;attempt<=attempts;++attempt) {
      // Retry from the current robot state; no controller goal has been issued.
      if(start)mg.setStartState(*start);
      else mg.setStartStateToCurrentState();
      const auto result=mg.plan(plan);
      lastCode=result.val;
      if(result==moveit::core::MoveItErrorCode::SUCCESS) {
        mg.clearPoseTargets();mg.clearPathConstraints();
        if(shearP1PoseGoal)
          RCLCPP_INFO(node_->get_logger(),
            "[POSE OMPL] %s pose-goal success attempt=%d/%d",
            target.robot.c_str(),attempt,attempts);
        else if(linearRetry && attempts>1)
          RCLCPP_INFO(node_->get_logger(),
            "[LIN REPLAN] robot=%s succeeded attempt=%d/%d",
            target.robot.c_str(),attempt,attempts);
        return plan;
      }
      if(shearP1PoseGoal)
        RCLCPP_WARN(node_->get_logger(),
          "[POSE OMPL] %s pose-goal failed attempt=%d/%d code=%d",
          target.robot.c_str(),attempt,attempts,lastCode);
      else if(linearRetry && attempts>1)
        RCLCPP_WARN(node_->get_logger(),
          "[LIN REPLAN] robot=%s attempt=%d/%d code=%d (before execution)",
          target.robot.c_str(),attempt,attempts,lastCode);
      if(attempt<attempts && linearRetry && retryDelay>0)
        std::this_thread::sleep_for(std::chrono::duration<double>(retryDelay));
    }
    mg.clearPoseTargets();mg.clearPathConstraints();
    if(shearP1PoseGoal) {
      if(start)diagnoseShearP1Start(target.robot,*start);
      else {
        moveit_msgs::msg::RobotState live;
        moveit::core::robotStateToRobotStateMsg(*currentState(mg),live);
        diagnoseShearP1Start(target.robot,live);
      }
    }
    throw std::runtime_error("Planning failed for "+target.robot+" ("+
      std::to_string(lastCode)+") after "+std::to_string(attempts)+" attempt(s)"+
      (shearP1PoseGoal?" [OMPL pose goal; inspect P1 DIAG logs]":""));
  }
  Plan planCombined(const std::string& combined,const std::vector<ArmGoal>& targets,
      const std::string& motion,const Tuning& t,bool home=false) {
    auto& mg=group(combined);setPlanning(mg,motion,t);
    auto st=currentState(mg);auto goal=std::make_shared<moveit::core::RobotState>(*st);
    if(home) {
      if(targets.empty()) {
        const auto jmg=goal->getJointModelGroup(combined);
        if(!jmg||!goal->setToDefaultValues(jmg,"home"))throw std::runtime_error("Missing home for "+combined);
      } else {
        for(const auto& armGoal:targets) {
          const auto jmg=goal->getJointModelGroup(arm(armGoal.robot).group);
          if(!jmg||!goal->setToDefaultValues(jmg,"home"))throw std::runtime_error("Missing home for "+armGoal.robot);
        }
      }
    } else {
      for(const auto& g:targets)if(!setPoseIK(*goal,g,t.ik))throw std::runtime_error("Combined IK failed: "+g.robot);
    }
    mg.setStartStateToCurrentState();
    if(!mg.setJointValueTarget(*goal))throw std::runtime_error("Combined joint goal rejected");
    // MoveIt can reject an OMPL path during its own post-plan collision check
    // (e.g. attached rebar versus gantry). That failure occurs before our
    // full-state validator; replan a fresh candidate without touching the ACM.
    const int maxPlanningAttempts=std::min(t.retries,3);
    int lastCode=0;
    for(int attempt=1;attempt<=maxPlanningAttempts;++attempt) {
      mg.setStartStateToCurrentState();
      Plan result;
      const auto err=mg.plan(result);
      lastCode=err.val;
      if(err==moveit::core::MoveItErrorCode::SUCCESS) {
        if(attempt>1)RCLCPP_INFO(node_->get_logger(),
          "[COMBINED REPLAN] group=%s succeeded attempt=%d/%d",
          combined.c_str(),attempt,maxPlanningAttempts);
        return result;
      }
      RCLCPP_WARN(node_->get_logger(),
        "[COMBINED REPLAN] group=%s rejected candidate attempt=%d/%d code=%d (no execution)",
        combined.c_str(),attempt,maxPlanningAttempts,lastCode);
    }
    throw std::runtime_error("Combined planning failed ("+std::to_string(lastCode)+
      ") after "+std::to_string(maxPlanningAttempts)+" attempt(s)");
  }
  Plan planMerged(const std::string& combined,const std::vector<ArmGoal>& goals,
      const std::vector<std::string>& motions,const std::vector<double>& velocities,
      double& planSec,const std::vector<geometry_msgs::msg::Pose>& vias={},
      bool shearP1PoseGoal=false) {
    if(goals.size()!=2||motions.size()!=2||velocities.size()!=2)throw std::runtime_error("Paired merge requires two arms");
    const auto capture=currentState(group(combined));
    moveit_msgs::msg::RobotState start;moveit::core::robotStateToRobotStateMsg(*capture,start);
    std::vector<Plan> plans;
    for(int i=0;i<2;++i) {
      auto t=tuning(motions[i],velocities[i]);auto t0=Clock::now();
      const auto via=i<static_cast<int>(vias.size())?&vias[i]:nullptr;
      plans.push_back(planIndividual(goals[i],motions[i],t,&start,via,
        shearP1PoseGoal && motions[i]=="moveJ"));planSec+=seconds(t0);
    }
    Plan result;result.start_state_=start;
    result.trajectory_.joint_trajectory=mergeTrajectories(plans[0].trajectory_.joint_trajectory,plans[1].trajectory_.joint_trajectory);
    return result;
  }
  // Coordinated Cartesian: IK for both arms on a shared state at each sample.
  // Unlike a simple trajectory merge, this preserves inter-arm joint-state coupling.
  Plan planCartesian(const std::string& combined,const std::vector<ArmGoal>& goals,const Tuning& t) {
    if(goals.empty())throw std::runtime_error("No Cartesian goals");
    auto& mg=group(combined);auto start=currentState(mg);
    robot_trajectory::RobotTrajectory trajectory(mg.getRobotModel(),combined);
    trajectory.addSuffixWayPoint(*start,0.0);
    auto working=std::make_shared<moveit::core::RobotState>(*start);
    std::vector<Eigen::Isometry3d> from;
    int samples=1;
    for(const auto& g:goals) {
      Eigen::Isometry3d origin=start->getGlobalLinkTransform(g.link);from.push_back(origin);
      auto delta=Eigen::Vector3d(g.pose.position.x,g.pose.position.y,g.pose.position.z)-origin.translation();
      const Eigen::Quaterniond initial(origin.rotation());
      const Eigen::Quaterniond dest(g.pose.orientation.w,g.pose.orientation.x,g.pose.orientation.y,g.pose.orientation.z);
      samples=std::max(samples,static_cast<int>(std::ceil(std::max(delta.norm()/0.01,initial.angularDistance(dest)/0.025))));
    }
    if(samples>2000)throw std::runtime_error("Cartesian sampling bound exceeded");
    for(int i=1;i<=samples;++i) {
      double u=static_cast<double>(i)/samples;
      for(size_t k=0;k<goals.size();++k) {
        auto target=goals[k].pose;
        const auto& startT=from[k];
        Eigen::Vector3d dest(target.position.x,target.position.y,target.position.z);
        auto p=startT.translation()*(1-u)+dest*u;
        Eigen::Quaterniond q0(startT.rotation()),q1(target.orientation.w,target.orientation.x,target.orientation.y,target.orientation.z);
        auto q=q0.slerp(u,q1).normalized();
        target.position.x=p.x();target.position.y=p.y();target.position.z=p.z();
        target.orientation.w=q.w();target.orientation.x=q.x();target.orientation.y=q.y();target.orientation.z=q.z();
        if(!setPoseIK(*working,{goals[k].robot,goals[k].link,target},t.ik))
          throw std::runtime_error("Coordinated Cartesian IK failed at sample "+std::to_string(i)+" for "+goals[k].robot);
      }
      working->update();trajectory.addSuffixWayPoint(*working,0.0);
    }
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    if(!iptp.computeTimeStamps(trajectory,t.vel,t.acc))throw std::runtime_error("Cartesian time parameterization failed");
    Plan plan;moveit::core::robotStateToRobotStateMsg(*start,plan.start_state_);trajectory.getRobotTrajectoryMsg(plan.trajectory_);
    return plan;
  }
  double validate(const Plan& plan,const std::string& collisionGroup) {
    const auto start=Clock::now();
    if(!validity_->wait_for_service(10s))throw std::runtime_error("/check_state_validity unavailable; refusing motion");
    // The full-scene validity service is shared by several concurrent SA3
    // executors. Make its response deadline configurable, without treating a
    // service timeout as a collision or bypassing validation.
    const auto localSettings=planSettings();
    const auto globalSettings=recipe_["defaults"]["planning"];
    const double validityResponseTimeoutSec=f(localSettings,"state_validity_response_timeout_sec",
      f(globalSettings,"state_validity_response_timeout_sec",8.0));
    if(!std::isfinite(validityResponseTimeoutSec) || validityResponseTimeoutSec<1.0 || validityResponseTimeoutSec>60.0)
      throw std::runtime_error("Invalid state_validity_response_timeout_sec (1..60 seconds)");
    const auto& jt=plan.trajectory_.joint_trajectory;
    if(jt.joint_names.empty()||jt.points.empty())throw std::runtime_error("Empty trajectory");
    size_t checks=0;
    auto inspect=[&](const std::vector<double>& q) {
      if(++checks>5000)throw std::runtime_error("Collision validation sample limit exceeded");
      auto req=std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
      req->group_name=collisionGroup;req->robot_state=plan.start_state_;req->robot_state.is_diff=true;
      req->robot_state.attached_collision_objects.clear();
      if(q.size()!=jt.joint_names.size())throw std::runtime_error("Trajectory joint count mismatch");
      auto& j=req->robot_state.joint_state;
      for(size_t k=0;k<jt.joint_names.size();++k) {
        const auto it=std::find(j.name.begin(),j.name.end(),jt.joint_names[k]);
        if(it==j.name.end())throw std::runtime_error("Trajectory joint absent from state: "+jt.joint_names[k]);
        const size_t idx=std::distance(j.name.begin(),it);
        if(idx>=j.position.size())throw std::runtime_error("Joint state positions incomplete");
        j.position[idx]=q[k];
      }
      auto future=validity_->async_send_request(req);
      if(future.wait_for(std::chrono::duration<double>(validityResponseTimeoutSec))!=std::future_status::ready) {
        validity_->remove_pending_request(future);
        throw std::runtime_error("State validity timed out after "+
          std::to_string(validityResponseTimeoutSec)+"s (collision validation incomplete)");
      }
      auto validity=future.get();
      if(!validity->valid) {
        const auto liveAcm=getAcm();
        for(const auto& contact:validity->contacts) {
          const auto& a=contact.contact_body_1;
          const auto& b=contact.contact_body_2;
          RCLCPP_ERROR(node_->get_logger(),
            "VALIDATION COLLISION sample=%zu %s <-> %s (live ACM effective=%d explicit=%s)",
            checks,a.c_str(),b.c_str(),int(effectiveAcmPermission(liveAcm,a,b)),
            acmPrevious(liveAcm,a,b).has_value()?
              (*acmPrevious(liveAcm,a,b)?"true":"false"):"absent");
        }
        for(const auto& constraint:validity->constraint_result) {
          if(!constraint.result)RCLCPP_ERROR(node_->get_logger(),
            "VALIDATION CONSTRAINT sample=%zu distance=%.6f",checks,constraint.distance);
        }
        throw std::runtime_error("Full-group collision/constraint validation failed at sample "+std::to_string(checks));
      }
    };
    inspect(jt.points.front().positions);
    for(size_t i=1;i<jt.points.size();++i) {
      const auto& a=jt.points[i-1].positions;const auto& z=jt.points[i].positions;
      if(a.size()!=z.size()||a.size()!=jt.joint_names.size())throw std::runtime_error("Inconsistent trajectory positions");
      double largest=0;for(size_t j=0;j<a.size();++j)largest=std::max(largest,std::abs(z[j]-a[j]));
      int n=std::max(1,static_cast<int>(std::ceil(largest/0.02)));
      for(int k=1;k<=n;++k){std::vector<double> pos(a.size());double u=double(k)/n;
        for(size_t j=0;j<a.size();++j) pos[j]=a[j]+u*(z[j]-a[j]);
        inspect(pos);
      }
    }
    RCLCPP_INFO(node_->get_logger(),"Validated %zu full-state samples for group %s",checks,collisionGroup.c_str());
    return seconds(start);
  }
  // Dual FJT sends both arm goals with the SAME future start time.
  // This is for the existing gripper-change execution backend only.
  void dualControllers(const Plan& plan,MotionAttempt& a) {
    using Follow=control_msgs::action::FollowJointTrajectory;
    const auto config=require(task_["config"],"execution");
    const auto& source=plan.trajectory_.joint_trajectory;
    if(source.points.empty())throw std::runtime_error("FJT source trajectory empty");
    const auto st=currentState(group("sa3_robots_1_2"));
    std::array<rclcpp_action::Client<Follow>::SharedPtr,2> clients;
    std::array<Follow::Goal,2> requests;
    const double delay=f(config,"start_delay_sec",0.5);
    const double tolerance=f(config,"start_tolerance_rad",0.05);
    for(int i=0;i<2;++i) {
      const auto robot="robot"+std::to_string(i+1);
      const auto jm=st->getJointModelGroup(arm(robot).group);
      if(!jm)throw std::runtime_error("FJT missing group: "+robot);
      const auto variables=jm->getVariableNames();
      for(size_t j=0;j<source.joint_names.size();++j) {
        const auto& joint=source.joint_names[j];
        if(std::find(variables.begin(),variables.end(),joint)==variables.end())continue;
        requests[i].trajectory.joint_names.push_back(joint);
        double q=0;
        if(!jointPosition(joint,q)||std::abs(q-source.points.front().positions.at(j))>tolerance)
          throw std::runtime_error("FJT start feedback mismatch for "+joint);
      }
      if(requests[i].trajectory.joint_names.empty())throw std::runtime_error("FJT joint set empty: "+robot);
      for(const auto& point:source.points) {
        JointPoint part;part.time_from_start=point.time_from_start;
        for(const auto& joint:requests[i].trajectory.joint_names) {
          auto it=std::find(source.joint_names.begin(),source.joint_names.end(),joint);
          size_t j=std::distance(source.joint_names.begin(),it);
          part.positions.push_back(point.positions.at(j));
          if(point.velocities.size()==source.joint_names.size())part.velocities.push_back(point.velocities.at(j));
          if(point.accelerations.size()==source.joint_names.size())part.accelerations.push_back(point.accelerations.at(j));
        }
        requests[i].trajectory.points.push_back(part);
      }
      auto name=s(config,robot+"_action", "/"+robot+"_controller/follow_joint_trajectory");
      clients[i]=rclcpp_action::create_client<Follow>(node_,name);
      if(!clients[i]->wait_for_action_server(10s))throw std::runtime_error("FJT controller missing: "+name);
    }
    if(dryRun_)return;
    const auto beginning=Clock::now();a.issued=true;
    auto stamp=node_->now()+rclcpp::Duration::from_seconds(delay);
    for(auto& request:requests)request.trajectory.header.stamp=stamp;
    auto first=clients[0]->async_send_goal(requests[0]);
    auto second=clients[1]->async_send_goal(requests[1]);
    if(first.wait_for(10s)!=std::future_status::ready||second.wait_for(10s)!=std::future_status::ready)
      throw std::runtime_error("EXECUTION_UNCERTAIN: FJT goal acceptance timeout");
    auto h1=first.get(),h2=second.get();
    if(!h1||!h2)throw std::runtime_error("EXECUTION_UNCERTAIN: one arm rejected FJT goal");
    auto r1=clients[0]->async_get_result(h1),r2=clients[1]->async_get_result(h2);
    const auto timeout=std::chrono::duration<double>(timeOf(source.points.back())+
      f(config,"result_timeout_margin_sec",30));
    if(r1.wait_for(timeout)!=std::future_status::ready||r2.wait_for(timeout)!=std::future_status::ready)
      throw std::runtime_error("EXECUTION_UNCERTAIN: FJT result timeout");
    if(r1.get().code!=rclcpp_action::ResultCode::SUCCEEDED||r2.get().code!=rclcpp_action::ResultCode::SUCCEEDED)
      throw std::runtime_error("EXECUTION_UNCERTAIN: FJT controller failure");
    a.execution=seconds(beginning);
  }
  void executePlan(const std::string& groupName,const Plan& plan,MotionAttempt& a) {
    if(dryRun_) {RCLCPP_WARN(node_->get_logger(),"DRY RUN: planned but not executing %s",groupName.c_str());return;}
    if((s(task_["execution"],"backend",s(task_["config"]["execution"],"executer","moveit"))
        =="follow_joint_trajectory")) {
      dualControllers(plan,a);return;
    }
    // Mark issued BEFORE execute(); failure may mean partial physical motion.
    a.issued=true;const auto t0=Clock::now();
    const auto code=group(groupName).execute(plan);a.execution=seconds(t0);
    if(code!=moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("EXECUTION_UNCERTAIN: "+groupName+" returned "+std::to_string(code.val)+"; manual/live-state reconciliation required");
  }
  // Global validation-only replanning policy; local task keys override defaults.
  // No retry is permitted after a trajectory has been issued to a controller.
  std::pair<int,double> collisionReplanPolicy() const {
    const auto local=planSettings();
    const auto defaults=recipe_["defaults"]["planning"];
    const int count=integer(local,"collision_validation_max_attempts",
                            integer(defaults,"collision_validation_max_attempts",1));
    const double delay=f(local,"collision_validation_retry_delay_sec",
                         f(defaults,"collision_validation_retry_delay_sec",0.25));
    if(count<1 || count>10 || !std::isfinite(delay) || delay<0 || delay>30)
      throw std::runtime_error("Invalid collision validation retry settings");
    return {count,delay};
  }
  static bool isCollisionValidationFailure(const std::string& reason) {
    return reason.find("Full-group collision/constraint validation failed at sample ")
           !=std::string::npos;
  }
  void logCollisionReplan(const std::string& step,int attempt,int maxAttempts,
                          const std::string& reason,double delay) {
    RCLCPP_WARN(node_->get_logger(),
      "[COLLISION REPLAN] task=%s step=%s attempt=%d/%d rejected before execution: %s; refreshing plan",
      taskId_.c_str(),step.c_str(),attempt,maxAttempts,reason.c_str());
    if(delay>0)std::this_thread::sleep_for(std::chrono::duration<double>(delay));
  }
  void motion(const std::string& stepId,const std::vector<ArmGoal>& goals,
      const std::string& motionName,const std::string& strategy,const std::string& combined,
      const std::string& object="",const std::string& point="",const std::vector<std::string>& perArmMotion={},
      const std::vector<double>& perArmVelocity={},
      const std::vector<geometry_msgs::msg::Pose>& vias={},
      bool shearP1PoseGoal=false) {
    Row r;r.phase=phase_;r.task=taskId_;r.step=stepId;r.motion=motionName;r.object=object;r.point=point;r.status="FAILED";
    for(const auto& a:goals){if(!r.robot.empty())r.robot+="+";r.robot+=a.robot;}
    auto taskStart=Clock::now();auto t=tuning(motionName=="fallback"?"moveL":motionName);
    r.velocity=t.vel;
    try {
      // Retry only a collision/constraint rejection discovered BEFORE executePlan().
      // Replan from live joint state on each attempt; never replay motion after
      // a controller goal may have been issued. YAML controls attempts and delay.
      const auto [maxAttempts,delaySec]=collisionReplanPolicy();
      for(int attempt=1;attempt<=maxAttempts;++attempt) {
        try {
      ensureTaskAcm(stepId+" before planning");
      Plan plan;MotionAttempt ma;ma.scale=t.vel;
      if(strategy=="merged") {
        if(goals.size()!=2)throw std::runtime_error("Merged motion requires exactly two arms");
        std::vector<std::string> motions=perArmMotion.empty()?std::vector<std::string>{motionName,motionName}:perArmMotion;
        std::vector<double> velocity=perArmVelocity.empty()?std::vector<double>{t.vel,t.vel}:perArmVelocity;
        ma.planner=motions[0]+"+"+motions[1];ma.scale=*std::min_element(velocity.begin(),velocity.end());
        plan=planMerged(combined,goals,motions,velocity,ma.planning,vias,shearP1PoseGoal);
      } else if(strategy=="coordinated_cartesian") {
        ma.planner="coordinated_cartesian";auto begin=Clock::now();plan=planCartesian(combined,goals,t);ma.planning=seconds(begin);
      } else if(strategy=="single") {
        if(goals.size()!=1)throw std::runtime_error("Single motion requires exactly one target");
        ma.planner=motionName;auto begin=Clock::now();
        plan=planIndividual(goals.front(),motionName,t,nullptr,
          vias.empty()?nullptr:&vias.front(),
          shearP1PoseGoal && motionName=="moveJ");ma.planning=seconds(begin);
      } else if(strategy=="combined_group") {
        ma.planner=motionName;auto begin=Clock::now();
        plan=planCombined(combined,goals,motionName,t,false);ma.planning=seconds(begin);
      } else throw std::runtime_error("Unsupported execution strategy: "+strategy);
      const std::string groupName=strategy=="single"?arm(goals.front().robot).group:combined;
      // Full-cell sampled collision checks include stationary other arms. For true
      // simultaneous unrelated processes, temporal cross-trajectory safety still
      // needs a supervisor/interlock; a validity service alone is insufficient.
      ensureTaskAcm(stepId+" before validation");
      ma.validation=validate(plan,fullCollisionGroup_);
      executePlan(groupName,plan,ma);
      r.planner=ma.planner;r.velocity=ma.scale;r.planning=ma.planning;r.validation=ma.validation;r.execution=ma.execution;
      r.status=dryRun_?"PLANNED_ONLY":"SUCCESS";
      break;
        } catch(const std::exception& e) {
          const std::string why=e.what();
          // Only the known pre-execution validity exception is retryable.
          // Planning failures, timeouts, scene inconsistencies and any
          // EXECUTION_UNCERTAIN result retain the original fail-closed policy.
          if(!isCollisionValidationFailure(why) || attempt==maxAttempts)throw;
          logCollisionReplan(stepId,attempt,maxAttempts,why,delaySec);
        }
      }
    } catch(const std::exception& e) {r.error=e.what();r.elapsed=seconds(taskStart);report(r);throw;}
    r.elapsed=seconds(taskStart);report(r);
  }
  void home(const std::string& step,const std::vector<std::string>& robots,const std::string& combined) {
    auto begin=Clock::now();Row r;r.phase=phase_;r.task=taskId_;r.step=step;r.motion="moveJ";r.planner="OMPL home";
    r.status="FAILED";
    for(const auto& name:robots){if(!r.robot.empty())r.robot+="+";r.robot+=name;}
    try {
      auto t=tuning("moveJ");const auto [attempts,delay]=collisionReplanPolicy();
      std::vector<ArmGoal> goals;
      for(const auto& name:robots)goals.push_back({name,arm(name).link,geometry_msgs::msg::Pose()});
      for(int attempt=1;attempt<=attempts;++attempt) {
        try {
          ensureTaskAcm(step+" before home planning");
          const auto t0=Clock::now();auto plan=planCombined(combined,goals,"moveJ",t,true);
          r.planning+=seconds(t0);
          ensureTaskAcm(step+" before home validation");
          r.validation+=validate(plan,fullCollisionGroup_);
          MotionAttempt a;executePlan(combined,plan,a);r.execution=a.execution;
          r.status=dryRun_?"PLANNED_ONLY":"SUCCESS";break;
        }catch(const std::exception& e) {
          if(!isCollisionValidationFailure(e.what()) || attempt==attempts)throw;
          logCollisionReplan(step,attempt,attempts,e.what(),delay);
        }
      }
    }catch(const std::exception& e){r.error=e.what();r.elapsed=seconds(begin);report(r);throw;}
    r.elapsed=seconds(begin);report(r);
  }
  void namedState(const std::string& step,const std::string& robot,const std::string& state) {
    Row r;r.phase=phase_;r.task=taskId_;r.step=step;r.robot=robot;r.motion="named_state";r.planner="OMPL";r.status="FAILED";
    auto begin=Clock::now();
    try {
      const auto [attempts,delay]=collisionReplanPolicy();
      for(int attempt=1;attempt<=attempts;++attempt) {
        try {
          auto a=arm(robot);auto& mg=group(a.group);auto t=tuning("moveJ");setPlanning(mg,"moveJ",t);
          mg.setStartStateToCurrentState();
          if(!mg.setNamedTarget(state))throw std::runtime_error("Named state missing: "+state);
          ensureTaskAcm(step+" before named state planning");
          Plan p;const auto t0=Clock::now();auto rc=mg.plan(p);r.planning+=seconds(t0);
          if(rc!=moveit::core::MoveItErrorCode::SUCCESS)throw std::runtime_error("Named state planning failed");
          ensureTaskAcm(step+" before named state validation");
          r.validation+=validate(p,fullCollisionGroup_);
          MotionAttempt ma;executePlan(a.group,p,ma);r.execution=ma.execution;
          r.status=dryRun_?"PLANNED_ONLY":"SUCCESS";break;
        }catch(const std::exception& e) {
          if(!isCollisionValidationFailure(e.what()) || attempt==attempts)throw;
          logCollisionReplan(step,attempt,attempts,e.what(),delay);
        }
      }
    }catch(const std::exception& e){r.error=e.what();r.elapsed=seconds(begin);report(r);throw;}
    r.elapsed=seconds(begin);report(r);
  }
  bool checkAttached(const std::string& id,const std::string& link="") {
    auto attached=scene_.getAttachedObjects({id});
    return attached.count(id) && (link.empty()||attached.at(id).link_name==link);
  }
  // Generic attachment geometry policy, supplied by YAML (never by task ID).
  // preserve_world: no teleport; dock_to_mount: align the object's original
  // dock-relative pose to the mounting link. This changes MoveIt's collision
  // object only; any independently spawned Gazebo model needs its own coupling.
  void attach(const std::string& object,const std::string& link,const std::vector<std::string>& touches,
     const std::string& dockFrame="",const std::string& placementMode="preserve_world",
     double poseTolerance=0.02) {
    if(object.empty()||link.empty())throw std::runtime_error("Attach object/link cannot be empty");
    if(poseTolerance<=0 || !std::isfinite(poseTolerance))
      throw std::runtime_error("verify_pose_tolerance_m must be positive");
    if(placementMode!="preserve_world" && placementMode!="dock_to_mount")
      throw std::runtime_error("Unsupported YAML attachment placement_mode: "+placementMode);
    if(placementMode=="dock_to_mount" && dockFrame.empty())
      throw std::runtime_error("dock_to_mount requires a YAML dock_frame for "+object);

    auto attached=scene_.getAttachedObjects({object});
    if(!attached.empty()) {
      if(scene_.getObjects({object}).count(object))
        throw std::runtime_error("Inconsistent planning scene: "+object+" is both WORLD and ATTACHED");
      if(attached.at(object).link_name!=link)
        throw std::runtime_error("Attachment belongs to a different link: "+object);
      // A pre-existing attachment is not proof of correct positioning.
      // Without the original world pose we cannot safely reconstruct a
      // dock-relative offset, so fail rather than silently passing Phase 0.
      if(placementMode=="dock_to_mount")
        throw std::runtime_error("Pre-attached object "+object+
          " cannot be verified against dock pose. Reset the scene or provide a dedicated mounted-pose verification.");
      return;
    }
    auto objs=scene_.getObjects({object});if(!objs.count(object))throw std::runtime_error("World object not found: "+object);
    if(dryRun_){RCLCPP_WARN(node_->get_logger(),"DRY RUN: attach %s -> %s mode=%s",object.c_str(),link.c_str(),placementMode.c_str());return;}
    auto source=objs.at(object);if(source.header.frame_id!="world" && !source.header.frame_id.empty())
      throw std::runtime_error("Attachment only supports world-frame collision objects: "+object);
    const auto mountWorld=framePose(link);
    const auto baseFrame=placementMode=="dock_to_mount" ? framePose(dockFrame) : mountWorld;
    const auto expectedLocal=tfPose(baseFrame).inverse()*tfPose(source.pose);
    source.pose=poseTf(expectedLocal);
    source.header.frame_id=link;source.operation=moveit_msgs::msg::CollisionObject::ADD;
    moveit_msgs::msg::AttachedCollisionObject a;a.object=source;
    a.link_name=link;a.touch_links=touches;
    const auto original=objs.at(object);
    scene_.removeCollisionObjects({object});
    if(!scene_.applyAttachedCollisionObjects({a})) {
      if(!checkAttached(object,link)) {
        auto restore=original;restore.operation=moveit_msgs::msg::CollisionObject::ADD;
        if(scene_.getObjects({object}).empty())scene_.applyCollisionObject(restore);
        throw std::runtime_error("Failed to attach: "+object+
            " (MoveIt rejected attached-object update, world object restored where possible). "+link);
      }
    }
    // Shared barrier for every task adapter (flange, gantry, shear, tools).
    // A transition is complete only when the object is ATTACHED to the
    // requested link AND absent from the world object list in repeated reads.
    const auto sync=recipe_["defaults"]["scene_sync"];
    const double timeout=f(sync,"attach_timeout_sec",8.0);
    const double poll=f(sync,"poll_interval_sec",0.15);
    const int stable=integer(sync,"stable_polls",3);
    if(timeout<=0.0 || poll<=0.0 || stable<1)
      throw std::runtime_error("Invalid defaults.scene_sync attach configuration");
    auto syncStart=Clock::now();int consecutive=0;std::string last="pending";
    while(rclcpp::ok() && seconds(syncStart)<timeout) {
      const auto result=scene_.getAttachedObjects({object});
      const auto worldNow=scene_.getObjects({object});
      if(!result.count(object))last="missing attached object";
      else if(result.at(object).link_name!=link)last="attached to wrong link "+result.at(object).link_name;
      else if(worldNow.count(object))last="still present as WORLD object";
      else {
        const auto actualLocal=tfPose(result.at(object).object.pose);
        const double drift=(actualLocal.getOrigin()-expectedLocal.getOrigin()).length();
        const double angle=actualLocal.getRotation().angleShortestPath(expectedLocal.getRotation());
        if(!std::isfinite(drift)||!std::isfinite(angle)||drift>poseTolerance||angle>0.1)
          last="attached pose mismatch translation="+std::to_string(drift)+
               "m rotation="+std::to_string(angle)+"rad";
        else if(++consecutive>=stable) {
          RCLCPP_INFO(node_->get_logger(),
            "[SCENE TRANSITION VERIFIED] action=ATTACH object=%s link=%s polls=%d elapsed=%.3fs",
            object.c_str(),link.c_str(),consecutive,seconds(syncStart));
          break;
        } else last="attached pose confirmed; awaiting stable scene reads";
      }
      if(last!="attached pose confirmed; awaiting stable scene reads")consecutive=0;
      std::this_thread::sleep_for(std::chrono::duration<double>(poll));
    }
    if(consecutive<stable)throw std::runtime_error("[SCENE TRANSITION FAILED] ATTACH object="+object+
      " link="+link+" after "+std::to_string(seconds(syncStart))+"s: "+last);
    RCLCPP_INFO(node_->get_logger(),"[ATTACH VERIFIED] object=%s link=%s mode=%s local_xyz=[%.4f %.4f %.4f]",
      object.c_str(),link.c_str(),placementMode.c_str(),
      source.pose.position.x,source.pose.position.y,source.pose.position.z);
  }
  // Fail-closed placed-object barrier. A successful ApplyPlanningScene reply is
  // not by itself proof that the canonical scene still contains a fixed rebar.
  // Only inspect the one object; do not fetch every workcell mesh.
  void verifyPlacedObject(const std::string& object,const std::string& checkpoint) {
    if(dryRun_)return;
    const auto saved=placedObjectPoses_.find(object);
    if(saved==placedObjectPoses_.end())
      throw std::runtime_error("[PLACE SYNC] No verified release pose for "+object+" at "+checkpoint);
    const auto sync=recipe_["defaults"]["scene_sync"];
    const double timeout=f(sync,"detach_timeout_sec",8.0);
    const double poll=f(sync,"poll_interval_sec",0.15);
    const double xyzTol=f(sync,"world_position_tolerance_m",0.01);
    const double rotTol=f(sync,"world_orientation_tolerance_rad",0.05);
    const int stable=integer(sync,"stable_polls",3);
    if(timeout<=0||poll<=0||xyzTol<=0||rotTol<=0||stable<1)
      throw std::runtime_error("Invalid YAML defaults.scene_sync configuration");
    auto start=Clock::now();int consecutive=0;
    std::string last="not checked";
    while(rclcpp::ok() && seconds(start)<timeout) {
      const auto attached=scene_.getAttachedObjects({object});
      const auto world=scene_.getObjects({object});
      if(attached.count(object))last="still ATTACHED to "+attached.at(object).link_name;
      else if(!world.count(object))last="missing WORLD object";
      else {
        const auto& current=world.at(object);
        if(current.header.frame_id!="world")last="unexpected frame "+current.header.frame_id;
        else {
          const auto& want=saved->second;
          const double distance=(tfPose(current.pose).getOrigin()-tfPose(want).getOrigin()).length();
          const double angular=tfPose(current.pose).getRotation().angleShortestPath(tfPose(want).getRotation());
          if(std::isfinite(distance)&&std::isfinite(angular)&&distance<=xyzTol&&angular<=rotTol) {
            if(++consecutive>=stable) {
              RCLCPP_INFO(node_->get_logger(),
                "[PLACE SYNC VERIFIED] object=%s checkpoint=%s world_xyz=[%.4f %.4f %.4f] polls=%d elapsed=%.3fs",
                object.c_str(),checkpoint.c_str(),current.pose.position.x,current.pose.position.y,
                current.pose.position.z,consecutive,seconds(start));
              return;
            }
            last="world pose stable, confirming repeated scene reads";
          } else {
            last="world pose mismatch translation="+std::to_string(distance)+
              "m orientation="+std::to_string(angular)+"rad";
          }
        }
      }
      if(last!="world pose stable, confirming repeated scene reads")consecutive=0;
      std::this_thread::sleep_for(std::chrono::duration<double>(poll));
    }
    throw std::runtime_error("[PLACE SYNC FAILED] object="+object+" at "+checkpoint+
      " after "+std::to_string(seconds(start))+"s: "+last);
  }
  void detach(const std::string& object,const std::string& expectedLink="") {
    auto attached=scene_.getAttachedObjects({object});
    if(!attached.count(object))throw std::runtime_error("Cannot detach: object not attached: "+object);
    auto old=attached.at(object);if(!expectedLink.empty()&&old.link_name!=expectedLink)
      throw std::runtime_error("Detached object has unexpected owner: "+object);
    if(dryRun_){RCLCPP_WARN(node_->get_logger(),"DRY RUN: detach %s",object.c_str());return;}
    auto world=old.object;
    world.pose=poseTf(tfPose(framePose(old.link_name))*tfPose(world.pose));
    world.header.frame_id="world";world.operation=moveit_msgs::msg::CollisionObject::ADD;
    moveit_msgs::msg::PlanningScene diff;diff.is_diff=true;diff.robot_state.is_diff=true;
    moveit_msgs::msg::AttachedCollisionObject rem;rem.link_name=old.link_name;rem.object.id=object;
    rem.object.operation=moveit_msgs::msg::CollisionObject::REMOVE;
    diff.robot_state.attached_collision_objects.push_back(rem);diff.world.collision_objects.push_back(world);
    if(!scene_.applyPlanningScene(diff))throw std::runtime_error("Failed to detach: "+object);
    // Record the computed world-frame release pose, then verify in several
    // independent canonical planning-scene reads before permitting retraction.
    placedObjectPoses_[object]=world.pose;
    verifyPlacedObject(object,"immediately_after_detach");
    RCLCPP_INFO(node_->get_logger(),
      "[SCENE TRANSITION VERIFIED] action=DETACH object=%s from_link=%s",
      object.c_str(),old.link_name.c_str());
  }
  static size_t indexAcm(moveit_msgs::msg::AllowedCollisionMatrix& matrix,const std::string& name) {
    for(size_t i=0;i<matrix.entry_names.size();++i)if(matrix.entry_names[i]==name)return i;
    matrix.entry_names.push_back(name);
    for(auto& row:matrix.entry_values)row.enabled.push_back(false);
    moveit_msgs::msg::AllowedCollisionEntry entry;entry.enabled.assign(matrix.entry_names.size(),false);
    matrix.entry_values.push_back(entry);return matrix.entry_names.size()-1;
  }
  std::optional<bool> acmPrevious(const moveit_msgs::msg::AllowedCollisionMatrix& matrix,
    const std::string& x,const std::string& y) {
    auto ix=std::find(matrix.entry_names.begin(),matrix.entry_names.end(),x);
    auto iy=std::find(matrix.entry_names.begin(),matrix.entry_names.end(),y);
    if(ix==matrix.entry_names.end()||iy==matrix.entry_names.end())return std::nullopt;
    const auto i=std::distance(matrix.entry_names.begin(),ix),j=std::distance(matrix.entry_names.begin(),iy);
    if(static_cast<size_t>(i)>=matrix.entry_values.size()||static_cast<size_t>(j)>=matrix.entry_values[i].enabled.size())return std::nullopt;
    return matrix.entry_values[i].enabled[j];
  }
  // A GetPlanningScene server can be advertised but temporarily unable to
  // respond (for example, while updating a busy planning scene). Do not
  // mistake service discovery for a successful reply. Retry boundedly, and
  // discard unanswered requests to avoid leaking client pending-request state.
  // Fail CLOSED: no motion or ACM changes are allowed without a confirmed read.
  moveit_msgs::msg::AllowedCollisionMatrix getAcm() {
    std::lock_guard<std::mutex> guard(acmReadMutex_);
    const auto cfg=recipe_["defaults"]["acm"];
    const int attempts=integer(cfg,"get_read_attempts",3);
    const double timeout=f(cfg,"get_response_timeout_sec",20.0);
    const double retryDelay=f(cfg,"get_retry_delay_sec",0.5);
    if(attempts<1||attempts>5||!std::isfinite(timeout)||timeout<1.0||timeout>60.0||
       !std::isfinite(retryDelay)||retryDelay<0.0||retryDelay>5.0)
      throw std::runtime_error("Invalid YAML defaults.acm GetPlanningScene retry settings");
    if(!getScene_->wait_for_service(10s))
      throw std::runtime_error("/get_planning_scene unavailable (check move_group/scene monitor)");
    for(int attempt=1;attempt<=attempts;++attempt) {
      if(!rclcpp::ok())throw std::runtime_error("ROS shutdown while reading ACM");
      auto req=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
      req->components.components=moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
      const auto start=Clock::now();
      auto response=getScene_->async_send_request(req);
      const auto status=response.wait_for(std::chrono::duration<double>(timeout));
      if(status==std::future_status::ready) {
        const auto reply=response.get();
        if(!reply)throw std::runtime_error("/get_planning_scene returned an empty ACM response");
        RCLCPP_DEBUG(node_->get_logger(),
          "[ACM GET] received %zu matrix names in %.3f s (attempt %d/%d)",
          reply->scene.allowed_collision_matrix.entry_names.size(),
          std::chrono::duration<double>(Clock::now()-start).count(),attempt,attempts);
        return reply->scene.allowed_collision_matrix;
      }
      // Required by rclcpp when async_send_request() does not complete.
      getScene_->remove_pending_request(response);
      RCLCPP_WARN(node_->get_logger(),
        "[ACM GET] /get_planning_scene no response after %.1f s, attempt %d/%d; "
        "discarded pending request (service may be busy or unresponsive)",
        timeout,attempt,attempts);
      if(attempt<attempts)
        std::this_thread::sleep_for(std::chrono::duration<double>(retryDelay));
    }
    throw std::runtime_error("Get ACM timed out after "+std::to_string(attempts)+
      " attempts; /get_planning_scene is unresponsive. Stop the sequence, inspect "
      "move_group service callbacks and scene monitor; collision validation must not be bypassed");
  }
  // Inter-process advisory lock for ACM read/modify/write operations.
  // All instances of this executor share a machine and the same MoveIt scene.
  // This does NOT serialize non-cooperating external planning-scene writers.
  struct AcmFileLock {
    int fd{-1};
    explicit AcmFileLock(const std::string& path) {
      fd=::open(path.c_str(),O_CREAT|O_RDWR|O_CLOEXEC,0666);
      if(fd<0)throw std::runtime_error("Cannot open ACM lock "+path+": "+std::strerror(errno));
      if(::flock(fd,LOCK_EX)!=0) {
        const int error=errno;::close(fd);fd=-1;
        throw std::runtime_error("Cannot acquire ACM lock: "+std::string(std::strerror(error)));
      }
    }
    ~AcmFileLock(){if(fd>=0){::flock(fd,LOCK_UN);::close(fd);}}
    AcmFileLock(const AcmFileLock&)=delete;
    AcmFileLock& operator=(const AcmFileLock&)=delete;
  };

  // Evaluate an ACM pair, including MoveIt's per-name default entries. An
  // absent explicit pair is NOT itself a failed restore: when both defaults
  // forbid contact, the effective permission is false.
  bool effectiveAcmPermission(
      const moveit_msgs::msg::AllowedCollisionMatrix& acm,
      const std::string& x,const std::string& y) {
    const auto explicitPair=acmPrevious(acm,x,y);
    if(explicitPair) return *explicitPair;
    auto defaultFor=[&](const std::string& name)->std::optional<bool> {
      const auto it=std::find(acm.default_entry_names.begin(),acm.default_entry_names.end(),name);
      if(it==acm.default_entry_names.end())return std::nullopt;
      const auto idx=static_cast<size_t>(std::distance(acm.default_entry_names.begin(),it));
      if(idx>=acm.default_entry_values.size())
        throw std::runtime_error("Malformed ACM default entries for "+name);
      return acm.default_entry_values[idx];
    };
    const auto a=defaultFor(x),z=defaultFor(y);
    // MoveIt's default-entry precedence for two differing defaults can be
    // ambiguous across scene versions; fail closed rather than assuming that
    // an absent explicit pair is forbidden.
    if(a && z && *a!=*z)
      throw std::runtime_error("Conflicting ACM defaults for "+x+" / "+y+"); explicit pair required");
    if(a)return *a;
    if(z)return *z;
    return false;
  }

  void collision(const std::string& x,const std::string& y,bool allow) {
    if(x.empty()||y.empty()||x==y)throw std::runtime_error("Invalid ACM pair");
    const auto key=std::minmax(x,y);
    const auto pair=std::make_pair(key.first,key.second);
    // Task-wide ACM owns this pair for the entire shear sequence. Original
    // per-step pick/place actions must NOT restore it prematurely: after G2_1,
    // the already placed rebar may still be adjacent to the moving Schunk base
    // during P1_2. Keep the bulk permission active until collisionBatch(false)
    // restores the saved pre-task value. Before motion(), ensureTaskAcm()
    // checks and reconciles the CURRENT scene for every task-owned pair.
    if(std::find(activeTaskAcmPairs_.begin(),activeTaskAcmPairs_.end(),pair)!=activeTaskAcmPairs_.end()) {
      RCLCPP_DEBUG(node_->get_logger(),
        "[YAML ACM TASK] deferred per-step %s for task-owned pair %s/%s",
        allow?"allow":"restore",x.c_str(),y.c_str());
      return;
    }
    const bool restore=!allow;
    if(restore && !oldAcm_.count(pair))
      throw std::runtime_error("Cannot restore ACM pair not saved in this task: "+x+" / "+y);
    if(dryRun_) {
      RCLCPP_WARN(node_->get_logger(),"DRY RUN: ACM %s/%s %s",x.c_str(),y.c_str(),restore?"restore":"allow");
      return;
    }
    const auto acmCfg=recipe_["defaults"]["acm"];
    const std::string lockPath=s(acmCfg,"lock_file","/tmp/sa3_generic_executor_acm.lock");
    const int maxAttempts=integer(acmCfg,"update_attempts",4);
    const int polls=integer(acmCfg,"verification_polls",5);
    const double delay=f(acmCfg,"poll_interval_sec",0.1);
    if(maxAttempts<1||maxAttempts>30||polls<1||polls>100||!std::isfinite(delay)||delay<0||delay>5)
      throw std::runtime_error("Invalid YAML defaults.acm verification settings");
    AcmFileLock lock(lockPath);
    auto current=getAcm();
    if(allow && !oldAcm_.count(pair))oldAcm_[pair]=acmPrevious(current,x,y);
    const auto previous=oldAcm_.at(pair);
    const bool expected=allow ? true : previous.value_or(false);
    const std::string operation=restore?"restore":"allow";
    for(int attempt=1;attempt<=maxAttempts;++attempt) {
      current=getAcm();
      const auto before=acmPrevious(current,x,y);
      const bool effective=effectiveAcmPermission(current,x,y);
      // An absent original pair must be restored to EFFECTIVELY forbidden.
      // Explicit false and absent with no permissive default are equivalent
      // for collision checking, but not structurally identical.
      // On restoration, MoveIt may normalize an explicitly false pair into
      // an absent pair. Verify the effective collision permission instead
      // of requiring the original matrix representation to be retained.
      // Allowing contact remains stricter: require both explicit directions.
      const bool match=allow
          ? (before==std::optional<bool>(true) &&
             acmPrevious(current,y,x)==std::optional<bool>(true))
          : (effective==expected);
      if(match) {
        RCLCPP_INFO(node_->get_logger(),
          "[ACM] %s %s/%s verified effective=%d explicit=%s attempt=%d",
          operation.c_str(),x.c_str(),y.c_str(),int(effective),
          before?(*before?"true":"false"):"absent",attempt);
        if(restore)oldAcm_.erase(pair);
        return;
      }
      auto updated=current;
      const auto ix=indexAcm(updated,x),iy=indexAcm(updated,y);
      updated.entry_values[ix].enabled[iy]=expected;
      updated.entry_values[iy].enabled[ix]=expected;
      moveit_msgs::msg::PlanningScene diff;
      diff.is_diff=true;
      diff.robot_state.is_diff=true;
      diff.allowed_collision_matrix=updated;
      const bool accepted=scene_.applyPlanningScene(diff);
      if(!accepted) {
        RCLCPP_WARN(node_->get_logger(),
          "[ACM] %s %s/%s apply rejected attempt=%d/%d",
          operation.c_str(),x.c_str(),y.c_str(),attempt,maxAttempts);
      }
      for(int poll=1;poll<=polls;++poll) {
        const auto observedMatrix=getAcm();
        const auto observed=acmPrevious(observedMatrix,x,y);
        const auto reverse=acmPrevious(observedMatrix,y,x);
        const bool effectiveNow=effectiveAcmPermission(observedMatrix,x,y);
        const bool verified=allow
            ? (observed==std::optional<bool>(true) &&
               reverse==std::optional<bool>(true))
            : (effectiveNow==expected);
        if(accepted && verified) {
          RCLCPP_INFO(node_->get_logger(),
            "[ACM] %s %s/%s verified effective=%d explicit=%s attempt=%d poll=%d",
            operation.c_str(),x.c_str(),y.c_str(),int(effectiveNow),
            observed?(*observed?"true":"false"):"absent",attempt,poll);
          if(restore)oldAcm_.erase(pair);
          return;
        }
        RCLCPP_WARN(node_->get_logger(),
          "[ACM] mismatch pair=%s/%s operation=%s expected=%d effective=%d observed=%s reverse=%s attempt=%d/%d poll=%d/%d",
          x.c_str(),y.c_str(),operation.c_str(),int(expected),int(effectiveNow),
          observed?(*observed?"true":"false"):"absent",
          reverse?(*reverse?"true":"false"):"absent",
          attempt,maxAttempts,poll,polls);
        std::this_thread::sleep_for(std::chrono::duration<double>(delay));
      }
    }
    throw std::runtime_error("ACM update not verified for "+x+" / "+y+
      " ("+operation+", effective expected="+(expected?"true":"false")+")");
  }
  // Apply an explicitly enumerated YAML ACM set in one planning-scene diff.
  // All identifiers come from YAML; no project-specific collision names here.
  // Preserve original effective permissions, verify the resulting scene, and
  // restore on success/failure. Shares the same inter-process lock as collision().
  void collisionBatch(const std::vector<std::pair<std::string,std::string>>& pairs,bool allow) {
    if(pairs.empty())return;
    if(dryRun_) {
      RCLCPP_WARN(node_->get_logger(),"DRY RUN: batch ACM %s %zu pairs",allow?"allow":"restore",pairs.size());
      return;
    }
    const auto cfg=recipe_["defaults"]["acm"];
    const std::string lockPath=s(cfg,"lock_file","/tmp/sa3_generic_executor_acm.lock");
    const int attempts=integer(cfg,"update_attempts",4);
    const int polls=integer(cfg,"verification_polls",5);
    const double delay=f(cfg,"poll_interval_sec",0.1);
    if(attempts<1||attempts>30||polls<1||polls>100||!std::isfinite(delay)||delay<0||delay>5)
      throw std::runtime_error("Invalid YAML defaults.acm batch settings");
    AcmFileLock lock(lockPath);
    const auto initial=getAcm();
    for(const auto& pair:pairs) {
      const auto& x=pair.first;const auto& y=pair.second;
      if(x.empty()||y.empty()||x==y||x>y)throw std::runtime_error("Invalid or noncanonical ACM batch pair");
      if(allow) {
        if(oldAcm_.count(pair))throw std::runtime_error("Task ACM pair already managed by another scope: "+x+" / "+y);
      } else if(!oldAcm_.count(pair)) {
        throw std::runtime_error("Task ACM pair missing saved permission: "+x+" / "+y);
      }
    }
    // Capture all originals before the first write, including an absent entry.
    if(allow)for(const auto& pair:pairs)
      oldAcm_[pair]=acmPrevious(initial,pair.first,pair.second);
    for(int attempt=1;attempt<=attempts;++attempt) {
      auto updated=getAcm();
      for(const auto& pair:pairs) {
        const auto& x=pair.first;const auto& y=pair.second;
        const bool expected=allow?true:oldAcm_.at(pair).value_or(false);
        const size_t ix=indexAcm(updated,x),iy=indexAcm(updated,y);
        updated.entry_values[ix].enabled[iy]=expected;
        updated.entry_values[iy].enabled[ix]=expected;
      }
      moveit_msgs::msg::PlanningScene diff;
      diff.is_diff=true;diff.robot_state.is_diff=true;
      diff.allowed_collision_matrix=updated;
      const bool accepted=scene_.applyPlanningScene(diff);
      for(int poll=1;poll<=polls;++poll) {
        const auto observed=getAcm();
        bool verified=accepted;
        size_t mismatch=0;
        for(const auto& pair:pairs) {
          const bool expected=allow?true:oldAcm_.at(pair).value_or(false);
          const bool actual=effectiveAcmPermission(observed,pair.first,pair.second);
          const bool match=allow
            ? (acmPrevious(observed,pair.first,pair.second)==std::optional<bool>(true) &&
               acmPrevious(observed,pair.second,pair.first)==std::optional<bool>(true))
            : (actual==expected);
          if(!match){verified=false;++mismatch;}
        }
        if(verified) {
          if(!allow)for(const auto& pair:pairs)oldAcm_.erase(pair);
          RCLCPP_INFO(node_->get_logger(),"[YAML ACM TASK] %s %zu pairs verified in batch attempt=%d poll=%d",
                      allow?"allow":"restore",pairs.size(),attempt,poll);
          return;
        }
        RCLCPP_WARN(node_->get_logger(),"[YAML ACM TASK] batch %s mismatches=%zu attempt=%d poll=%d",
                    allow?"allow":"restore",mismatch,attempt,poll);
        std::this_thread::sleep_for(std::chrono::duration<double>(delay));
      }
    }
    throw std::runtime_error("Batch ACM "+std::string(allow?"allow":"restore")+" verification failed");
  }
  // Reconcile task-wide ACM against MoveIt's CURRENT scene after other
  // planning-scene writers (attachment, detach, or independent task updates).
  // Do not overwrite the originally saved values; they belong to collisionBatch.
  void ensureTaskAcm(const std::string& context) {
    if(activeTaskAcmPairs_.empty() || dryRun_)return;
    const auto observed=getAcm();
    std::vector<std::pair<std::string,std::string>> missing;
    for(const auto& pair:activeTaskAcmPairs_) {
      const auto fwd=acmPrevious(observed,pair.first,pair.second);
      const auto rev=acmPrevious(observed,pair.second,pair.first);
      if(fwd!=std::optional<bool>(true)||rev!=std::optional<bool>(true)) missing.push_back(pair);
    }
    if(missing.empty())return;
    RCLCPP_WARN(node_->get_logger(),
      "[YAML ACM TASK] %s lost %zu/%zu expected permissions; reconciling current scene",
      context.c_str(),missing.size(),activeTaskAcmPairs_.size());
    const auto cfg=recipe_["defaults"]["acm"];
    AcmFileLock lock(s(cfg,"lock_file","/tmp/sa3_generic_executor_acm.lock"));
    auto refreshed=getAcm();
    std::size_t corrected=0;
    for(const auto& pair:activeTaskAcmPairs_) {
      if(acmPrevious(refreshed,pair.first,pair.second)==std::optional<bool>(true)&&
         acmPrevious(refreshed,pair.second,pair.first)==std::optional<bool>(true))continue;
      const auto a=indexAcm(refreshed,pair.first),b=indexAcm(refreshed,pair.second);
      refreshed.entry_values[a].enabled[b]=true;
      refreshed.entry_values[b].enabled[a]=true;
      ++corrected;
    }
    if(corrected) {
      moveit_msgs::msg::PlanningScene diff;
      diff.is_diff=true;diff.robot_state.is_diff=true;
      diff.allowed_collision_matrix=refreshed;
      if(!scene_.applyPlanningScene(diff))
        throw std::runtime_error("Task ACM reconciliation rejected by planning scene");
    }
    const auto verified=getAcm();
    for(const auto& pair:activeTaskAcmPairs_) {
      if(acmPrevious(verified,pair.first,pair.second)!=std::optional<bool>(true)||
         acmPrevious(verified,pair.second,pair.first)!=std::optional<bool>(true))
        throw std::runtime_error("Task ACM reconciliation failed: "+pair.first+" / "+pair.second);
    }
    RCLCPP_INFO(node_->get_logger(),"[YAML ACM TASK] %s repaired and verified %zu permissions",
      context.c_str(),corrected);
  }
  void collisionSet(const std::string& x,const std::vector<std::string>& ys,bool allow) {
    for(const auto& y:ys)if(y!=x)collision(x,y,allow);
  }
  bool jointPosition(const std::string& name,double& q) {
    std::lock_guard<std::mutex> lock(jointMutex_);auto it=jointPositions_.find(name);
    if(it==jointPositions_.end()) return false;
    q=it->second;
    return true;
  }
  // FJT controller commands are complete joint sets. Any uncommanded axis
  // is held at its latest observed position, not omitted from the goal.
  // controllerJoints must match the active ros2_control controller configuration.
  void controllerJoint(const std::string& step,const std::string& joint,double target,double speed,double acceleration,
    const std::string& action="/gantry_controller/follow_joint_trajectory",
    const std::vector<std::string>& controllerJoints={}) {
    if(!std::isfinite(target)||!std::isfinite(speed)||!std::isfinite(acceleration)||
       speed<=0||acceleration<=0)throw std::runtime_error("Invalid gantry target/dynamics");
    if(controllerJoints.empty())throw std::runtime_error(
      "Missing controller_joints in YAML for "+step+"; refusing partial FJT goal");
    if(std::set<std::string>(controllerJoints.begin(),controllerJoints.end()).size()!=controllerJoints.size())
      throw std::runtime_error("Duplicate controller_joints in YAML");
    if(std::find(controllerJoints.begin(),controllerJoints.end(),joint)==controllerJoints.end())
      throw std::runtime_error("Target joint "+joint+" not listed in controller_joints");
    Row r;r.phase=phase_;r.task=taskId_;r.robot="gantry";r.step=step;
    r.motion="joint";r.planner="FollowJointTrajectory";r.status="FAILED";
    const auto begin=Clock::now();
    try {
      // Collect the entire position vector under one lock so the held axis
      // does not inadvertently command an obsolete/hardcoded home position.
      std::vector<double> initial;
      const auto deadline=Clock::now()+10s;
      while(rclcpp::ok()) {
        bool available=true;
        {
          std::lock_guard<std::mutex> lock(jointMutex_);
          initial.clear();
          for(const auto& name:controllerJoints) {
            auto it=jointPositions_.find(name);
            if(it==jointPositions_.end()||!std::isfinite(it->second)) {available=false;break;}
            initial.push_back(it->second);
          }
        }
        if(available)break;
        if(Clock::now()>=deadline)throw std::runtime_error("Missing/invalid gantry joint feedback for controller joint set");
        std::this_thread::sleep_for(100ms);
      }
      if(!rclcpp::ok())throw std::runtime_error("ROS shutdown while waiting for gantry joint feedback");
      const auto moving=static_cast<size_t>(std::distance(controllerJoints.begin(),
                       std::find(controllerJoints.begin(),controllerJoints.end(),joint)));
      const double current=initial.at(moving);
      auto final=initial;
      final[moving]=target;
      const double duration=std::max(0.2,std::abs(target-current)/speed+speed/acceleration);
      if(!dryRun_) {
        using Follow=control_msgs::action::FollowJointTrajectory;
        auto client=rclcpp_action::create_client<Follow>(node_,action);
        if(!client->wait_for_action_server(10s))throw std::runtime_error("Gantry controller unavailable: "+action);
        Follow::Goal goal;
        goal.trajectory.joint_names=controllerJoints;
        JointPoint p0;p0.positions=initial;setTime(p0,0.0);
        JointPoint p1;p1.positions=final;setTime(p1,duration);
        goal.trajectory.points={p0,p1};
        std::ostringstream details;
        for(size_t i=0;i<controllerJoints.size();++i) {
          if(i)details<<", ";
          details<<controllerJoints[i]<<"="<<initial[i]<<"->"<<final[i];
        }
        RCLCPP_INFO(node_->get_logger(),"Gantry FJT step=%s action=%s joints=[%s] duration=%.3fs",
          step.c_str(),action.c_str(),details.str().c_str(),duration);
        auto future=client->async_send_goal(goal);
        if(future.wait_for(8s)!=std::future_status::ready)
          throw std::runtime_error("EXECUTION_UNCERTAIN: Controller goal acceptance timeout");
        auto handle=future.get();
        if(!handle)throw std::runtime_error("Gantry goal rejected by "+action+
           " for complete configured joint set; inspect controller logs and actual controller joints");
        auto result=client->async_get_result(handle);
        if(result.wait_for(std::chrono::duration<double>(duration+30))!=std::future_status::ready)
          throw std::runtime_error("EXECUTION_UNCERTAIN: gantry controller result timed out");
        auto wrapped=result.get();
        if(wrapped.code!=rclcpp_action::ResultCode::SUCCEEDED ||
           !wrapped.result || wrapped.result->error_code!=Follow::Result::SUCCESSFUL) {
          const auto explanation=wrapped.result ? wrapped.result->error_string : "no result message";
          throw std::runtime_error("EXECUTION_UNCERTAIN: gantry controller error: "+explanation);
        }
        double reported=0;
        if(!jointPosition(joint,reported)||std::abs(reported-target)>0.01)
          throw std::runtime_error("EXECUTION_UNCERTAIN: gantry target feedback not within 0.01m");
      }
      r.status=dryRun_?"PLANNED_ONLY":"SUCCESS";
      r.execution=seconds(begin);
    }catch(const std::exception& e){r.error=e.what();r.elapsed=seconds(begin);report(r);throw;}
    r.elapsed=seconds(begin);report(r);
  }
  // This action dispatcher is shared by flange, shear, tying, gantry and tool
  // change task adapters. Unknown aliases fail closed instead of being ignored.
  void action(const std::string& name,const ContextPair& ctx) {
    auto begin=Clock::now();
    const bool robotContext=ctx.robot.rfind("robot",0)==0;
    const auto a=robotContext?arm(ctx.robot):ArmDef{};
    if(name=="complete_pair"||name=="mark_point_complete")return;
    if(name=="settle") {std::this_thread::sleep_for(std::chrono::duration<double>(f(task_["config"]["ros_parameters"],"settle_time_sec",0.5)));return;}
    if(name=="prepare_pick"||name=="allow_pick_contact"||name=="allow_gripper_contact") {
      collisionSet(ctx.object,ctx.touch,true);
    } else if(name=="attach_objects"||name=="attach_rebar"||name=="attach_robot3"||name=="attach_gantry"||name=="attach_active_objects") {
      attach(ctx.object,ctx.link.empty()?a.link:ctx.link,ctx.touch);
    } else if(name=="allow_inventory_contact") {
      collision(ctx.object,ctx.inventory,true);
    } else if(name=="restore_inventory_contact") {
      collision(ctx.object,ctx.inventory,false);
    } else if(name=="allow_web_ring_contact") {
      collision(ctx.object,ctx.ring,true);
    } else if(name=="restore_pick_contact") {
      collision(ctx.object,ctx.ring,false);collisionSet(ctx.object,ctx.touch,false);
    } else if(name=="allow_place_contact"||name=="allow_jig_contact") {
      collisionSet(ctx.object,ctx.touch,true);collision(ctx.object,s(recipe_["defaults"]["scene_objects"],"jig_clamp"),true);
    } else if(name=="detach_objects"||name=="detach_rebar"||name=="detach_robot3"||name=="detach_gantry"||name=="detach_active_objects") {
      detach(ctx.object,ctx.link);
    } else if(name=="restore_gripper_contact"||name=="restore_place_contact") {
      collisionSet(ctx.object,ctx.touch,false);
      // Restore jig only if previously changed; flange G3 does not always
      // own this pair, but the earlier allow_place_contact does.
      if(oldAcm_.count(std::make_pair(std::min(ctx.object,s(recipe_["defaults"]["scene_objects"],"jig_clamp")),std::max(ctx.object,s(recipe_["defaults"]["scene_objects"],"jig_clamp")))))
        collision(ctx.object,s(recipe_["defaults"]["scene_objects"],"jig_clamp"),false);
    } else if(name=="prepare_robot3_pick") {
      collisionSet(ctx.object,ctx.touch,true);
    } else if(name=="lift_gantry_start_parallel_return"||name=="wait_gantry_parallel_return") {
      // Ordered by gantry task adapter. No unverified asynchronous bypass.
      RCLCPP_INFO(node_->get_logger(),"Gantry return action %s is owned by gantry state machine",name.c_str());
    } else throw std::runtime_error("UNMAPPED YAML action: "+name);
    Row r;r.phase=phase_;r.task=taskId_;r.step="ACTION:"+name;r.robot=ctx.robot;r.object=ctx.object;r.motion="scene";
    r.scene=seconds(begin);r.elapsed=r.scene;r.status=dryRun_?"PLANNED_ONLY":"SUCCESS";report(r);
  }
  void actions(const YAML::Node& items,const std::vector<ContextPair>& contexts) {
    if(!items)return;
    if(!items.IsSequence())throw std::runtime_error("YAML actions must be sequence");
    for(const auto& item:items) {
      if(!item.IsScalar())throw std::runtime_error("Expected action alias string");
      for(const auto& ctx:contexts) action(item.as<std::string>(),ctx);
    }
  }
  std::vector<ContextPair> flangeContexts() {
    std::vector<ContextPair> v;
    for(auto robot:{"robot3","robot4"}) {
      const auto a=arm(robot);const auto t=require(task_,robot);ContextPair c;
      c.robot=robot;c.object=s(t,"object_id");c.inventory=s(t,"inventory_id");c.link=a.link;c.touch=a.touch;v.push_back(c);
    }
    return v;
  }
  void runFlange() {
    int attachments=0,detachments=0;
    for(const auto& step:require(task_,"steps")) {
      for(const auto& alias:step["after_actions"]) {
        if(alias.as<std::string>()=="attach_objects")++attachments;
        if(alias.as<std::string>()=="detach_objects")++detachments;
      }
    }
    if(attachments!=1||detachments!=1)
      throw std::runtime_error("Flange recipe must contain exactly one attach and detach");
    auto contexts=flangeContexts();const auto pairGroup=s(task_["execution"],"combined_group","robot3_robot4_arms");
    if(param<bool>("wait_for_gantry_g4",false))waitCheckpoint("gantry_g4",param<double>("gantry_checkpoint_timeout_sec",180));
    for(const auto& step:require(task_,"steps")) {
      const std::string id=s(step,"id"),kind=s(step,"motion");
      actions(step["before_actions"],contexts);
      if(s(step,"reference")=="home") {
        if(!param<bool>("skip_home",false))home(id,{"robot3","robot4"},pairGroup);
      } else {
        std::vector<ArmGoal> targets;
        for(auto robot:{"robot3","robot4"})targets.push_back({robot,arm(robot).link,resolveTarget(require(step,robot),require(task_,robot))});
        auto strategy=s(step,"execution_strategy",kind=="moveL"?"merged":"combined_group");
        if(id=="B2") {
          auto old=task_["planning"]["movej_velocity"];
          auto oldAcc=task_["planning"]["movej_acceleration"];
          task_["planning"]["movej_velocity"]=f(task_["planning"],"reverse_ptp_velocity",tuning("moveJ").vel);
          task_["planning"]["movej_acceleration"]=f(task_["planning"],"reverse_ptp_acceleration",tuning("moveJ").acc);
          try{motion(id,targets,kind,strategy,pairGroup);}
          catch(...){task_["planning"]["movej_velocity"]=old;task_["planning"]["movej_acceleration"]=oldAcc;throw;}
          task_["planning"]["movej_velocity"]=old;
          task_["planning"]["movej_acceleration"]=oldAcc;
        } else if(kind=="moveL" && (id=="T2"||(id=="P1"&&taskId_=="flange_3d_4d"))) {
          try {motion(id,targets,"moveL","merged",pairGroup);}
          catch(const std::exception& e) {
            if(std::string(e.what()).find("EXECUTION_UNCERTAIN")!=std::string::npos)throw;
            RCLCPP_WARN(node_->get_logger(),"%s LIN planning failed, combined OMPL MoveJ fallback",id.c_str());
            motion(id+"_fallback",targets,"moveJ","combined_group",pairGroup);
          }
        } else motion(id,targets,kind,strategy,pairGroup);
      }
      actions(step["after_actions"],contexts);
    }
  }
  void runSceneOnly() {
    for(const auto& step:require(task_,"steps")) {
      if(s(step,"action")!="attach_at_dock")throw std::runtime_error("Unsupported scene-only action");
      const std::string objectLink=s(step,"attachment_link"),dock=s(step,"dock_frame");
      for(const auto& id:strs(step["object_ids"])) {
        auto tick=Clock::now();attach(id,objectLink,strs(step["touch_links"]),dock);
        Row r;r.phase=phase_;r.task=taskId_;r.robot=s(step,"robot");r.step=s(step,"id");r.object=id;
        r.motion="attach_at_dock";r.scene=seconds(tick);r.elapsed=r.scene;r.status=dryRun_?"PLANNED_ONLY":"SUCCESS";report(r);
      }
    }
  }
  // Resolve shear P1 first or subsequent pair independently. The YAML retains
  // existing pair and approach definitions, so no frame names are hard-coded.
  geometry_msgs::msg::Pose shearPose(const YAML::Node& pair,const YAML::Node& cfg,const std::string& ref) {
    const auto pick=applyOffset(framePose(s(pair,"pick_frame")),vec(pair["pick_offset"]),"world");
    const auto place=framePose(s(pair,"place_frame"));
    if(ref=="pick")return pick;
    if(ref=="place")return place;
    const auto approach=require(cfg,"approach");
    const auto delta=vec(approach["xyz"]);const auto coordinate=s(approach,"offset_frame","world");
    if(ref=="pick_approach")return applyOffset(pick,delta,coordinate);
    if(ref=="place_approach")return applyOffset(place,delta,coordinate);
    throw std::runtime_error("Unknown shear target: "+ref);
  }
  void runShear() {
    if(require(task_,"stages").size()!=6)throw std::runtime_error("Shear requires six stage definitions");
    auto c=require(task_,"config"),p=require(c,"planning");
    auto one=require(require(c,"robot1"),"pairs"),two=require(require(c,"robot2"),"pairs");
    const size_t count=std::max(one.size(),two.size());const std::string combined=s(task_["execution"],"combined_group","sa3_robots_1_2");
    const auto stages=require(task_,"stages");
    const auto strategyList=strs(p["transit_strategies"]);
    const auto scales=doubles(p["lin_velocity_scales"]);if(scales.empty())throw std::runtime_error("Shear LIN scales missing");
    // Optional task-wide YAML collision pairs: one atomic ACM update instead of
    // repeatedly applying hundreds of pairs for each shear-link cycle.
    std::vector<std::pair<std::string,std::string>> taskAcmPairs;
    if(const auto cfgPairs=task_["acm_task_pairs"]) {
      if(!cfgPairs.IsSequence())throw std::runtime_error("acm_task_pairs must be a list");
      for(const auto& item:cfgPairs) {
        if(!item.IsSequence()||item.size()!=2)
          throw std::runtime_error("acm_task_pairs entries must be [a,b]");
        const auto a=item[0].as<std::string>(),z=item[1].as<std::string>();
        if(a.empty()||z.empty()||a==z)throw std::runtime_error("Invalid acm_task_pairs item");
        const auto key=std::minmax(a,z);
        const std::pair<std::string,std::string> pair{key.first,key.second};
        if(std::find(taskAcmPairs.begin(),taskAcmPairs.end(),pair)!=taskAcmPairs.end())
          throw std::runtime_error("Duplicate acm_task_pairs entry: "+a+" / "+z);
        taskAcmPairs.push_back(pair);
      }
    }
    // Validate the entire requested collision-contact contract before ANY
    // planning-scene change. This catches omitted pairs or unexpected extras
    // deterministically, rather than after a robot has completed prior cycles.
    if(const auto coverage=task_["acm_coverage"]) {
      if(!coverage.IsMap())throw std::runtime_error("acm_coverage must be a mapping");
      std::set<std::pair<std::string,std::string>> expected;
      const auto canonical=[](const std::string& x,const std::string& y) {
        if(x.empty()||y.empty()||x==y)
          throw std::runtime_error("Invalid acm_coverage pair: "+x+" / "+y);
        const auto ordered=std::minmax(x,y);
        return std::make_pair(ordered.first,ordered.second);
      };
      const auto members=strs(coverage["all_pairs_within"]);
      const auto tools=strs(coverage["each_with"]);
      if(members.empty()||tools.empty())
        throw std::runtime_error("acm_coverage requires all_pairs_within and each_with");
      for(size_t i=0;i<members.size();++i) {
        for(size_t j=i+1;j<members.size();++j)expected.insert(canonical(members[i],members[j]));
        for(const auto& tool:tools)expected.insert(canonical(members[i],tool));
      }
      if(const auto extras=coverage["additional_pairs"]) {
        if(!extras.IsSequence())throw std::runtime_error("acm_coverage.additional_pairs must be a list");
        for(const auto& extra:extras) {
          if(!extra.IsSequence()||extra.size()!=2)
            throw std::runtime_error("acm_coverage additional pair must contain two names");
          expected.insert(canonical(extra[0].as<std::string>(),extra[1].as<std::string>()));
        }
      }
      const std::set<std::pair<std::string,std::string>> actual(taskAcmPairs.begin(),taskAcmPairs.end());
      for(const auto& pair:expected) {
        if(!actual.count(pair))throw std::runtime_error(
          "Missing required shear ACM permission: "+pair.first+" / "+pair.second);
      }
      if(b(coverage,"require_exact_match",true)) {
        for(const auto& pair:actual)
          if(!expected.count(pair))throw std::runtime_error(
            "Unexpected shear ACM permission: "+pair.first+" / "+pair.second);
      }
      RCLCPP_INFO(node_->get_logger(),
        "[ACM PREFLIGHT] %zu contact pairs match YAML coverage contract; no missing or extra permissions",
        actual.size());
    }
    // YAML-controlled, temporary collision permissions for pair-indexed steps.
    // No robot, object, or phase identity is embedded in this handler.
    std::vector<std::pair<std::string,std::string>> activeScopedAcm;
    auto stageAcm = [&](const std::string& stepId,size_t pairNumber,bool entering) {
      const auto scopes=task_["acm_scopes"];
      if(!scopes)return;
      if(!scopes.IsSequence())throw std::runtime_error("acm_scopes must be a sequence");
      for(const auto& scope:scopes) {
        const auto event=s(scope,entering?"start_step":"end_step");
        if(event!=stepId)continue;
        const int specified=integer(scope,"pair_index",0);
        if(specified>0 && size_t(specified)!=pairNumber)continue;
        for(const auto& pair:require(scope,"pairs")) {
          if(!pair.IsSequence()||pair.size()!=2)
            throw std::runtime_error("acm_scopes pairs must be [object_a, object_b]");
          const std::string x=pair[0].as<std::string>(),y=pair[1].as<std::string>();
          const auto key=std::minmax(x,y);
          const std::pair<std::string,std::string> ordered{key.first,key.second};
          if(entering) {
            if(std::find(activeScopedAcm.begin(),activeScopedAcm.end(),ordered)!=activeScopedAcm.end())
              throw std::runtime_error("Duplicate active ACM scope for "+x+" / "+y);
            collision(x,y,true);
            activeScopedAcm.push_back(ordered);
          } else {
            auto it=std::find(activeScopedAcm.begin(),activeScopedAcm.end(),ordered);
            if(it==activeScopedAcm.end())throw std::runtime_error("ACM scope end without start for "+x+" / "+y);
            collision(x,y,false);
            activeScopedAcm.erase(it);
          }
          RCLCPP_INFO(node_->get_logger(),"[YAML ACM SCOPE] step=%s pair=%zu %s %s / %s",
                      stepId.c_str(),pairNumber,entering?"allow":"restore",x.c_str(),y.c_str());
        }
      }
    };
    bool batchActive=false;
    try {
    if(!taskAcmPairs.empty()) {
      batchActive=true;  // cleanup attempts restoration even after a partial failed allow
      collisionBatch(taskAcmPairs,true);
      activeTaskAcmPairs_=taskAcmPairs;
    }
    for(size_t index=0;index<count;++index) {
      for(const auto& step:stages) {
        const auto id=s(step,"id"),ref=s(step,"reference"),kind=s(step,"motion");
        std::vector<ArmGoal> targets;std::vector<ContextPair> contexts;
        for(auto robot:{"robot1","robot2"}) {
          auto entries=std::string(robot)=="robot1"?one:two;
          if(index>=entries.size())continue;
          const auto pair=entries[index];const auto a=arm(robot);
          targets.push_back({robot,a.link,shearPose(pair,c,ref)});
          contexts.push_back({robot,s(pair,"object_id"),"",a.web_ring,a.link,a.touch});
        }
        if(targets.empty())continue;
        // Before the next pick, make sure the previous pair really remains
        // detached and fixed in the canonical MoveIt world scene.
        if(id=="P1" && index>0) {
          const auto prev1=one[index-1];
          verifyPlacedObject(s(prev1,"object_id"),"before_next_P1_"+std::to_string(index+1));
          if(index-1<two.size()) {
            const auto prev2=two[index-1];
            verifyPlacedObject(s(prev2,"object_id"),"before_next_P1_"+std::to_string(index+1));
          }
        }
        stageAcm(id,index+1,true);
        // Phase 3A has already executed the first pair's P1 approach.
        // Do not repeat that motion, but verify the live end-effector poses
        // match the P1 goals before trusting the checkpoint.
        if(index==0 && id=="P1" && param<bool>("first_p1_prepositioned",false)) {
          constexpr double kPositionToleranceM = 0.02;
          constexpr double kAngularToleranceRad = 0.10;
          for(const auto& goal:targets) {
            const auto actual=group(arm(goal.robot).group).getCurrentPose(goal.link).pose;
            const auto& expected=goal.pose;
            const double dx=actual.position.x-expected.position.x;
            const double dy=actual.position.y-expected.position.y;
            const double dz=actual.position.z-expected.position.z;
            const double xyz_error=std::sqrt(dx*dx+dy*dy+dz*dz);
            tf2::Quaternion qa(actual.orientation.x,actual.orientation.y,
                               actual.orientation.z,actual.orientation.w);
            tf2::Quaternion qe(expected.orientation.x,expected.orientation.y,
                               expected.orientation.z,expected.orientation.w);
            qa.normalize();qe.normalize();
            const double angle_error=qa.angleShortestPath(qe);
            if(!std::isfinite(xyz_error)||!std::isfinite(angle_error)||
               xyz_error>kPositionToleranceM||angle_error>kAngularToleranceRad) {
              throw std::runtime_error("Prepositioned shear P1 verification failed for "+goal.robot+
                " (position error="+std::to_string(xyz_error)+" m, orientation error="+
                std::to_string(angle_error)+" rad)");
            }
            RCLCPP_INFO(node_->get_logger(),
              "[SHEAR P1] %s already at first pick approach: translation=%.4f m angle=%.4f rad",
              goal.robot.c_str(),xyz_error,angle_error);
          }
          // A motion prepositioned by an earlier stage is not the same as
          // skipping the whole YAML step: its scene/ACM actions still matter.
          // The YAML explicitly selects whether verified prepositioning runs
          // the step's post-actions. No object names or ACM pairs are in C++.
          const std::string onPrepositioned=s(step,"on_prepositioned", "error");
          if(onPrepositioned=="run_after_actions") {
            actions(step["after_actions"],contexts);
            RCLCPP_INFO(node_->get_logger(),
              "[PREPOSITIONED] Verified motion %s; executed YAML after_actions for %zu robots",
              id.c_str(),contexts.size());
          } else if(onPrepositioned=="skip_all") {
            RCLCPP_INFO(node_->get_logger(),
              "[PREPOSITIONED] Verified motion %s; YAML explicitly skipped after_actions",
              id.c_str());
          } else {
            throw std::runtime_error("Prepositioned step "+id+
              " requires YAML on_prepositioned: run_after_actions or skip_all");
          }
          continue;
        }
        // P1 planner can differ by robot and by first/subsequent pair.
        if(id=="P1") {
          const auto goalMode=s(p,"pick_approach_goal_mode","pose");
          if(goalMode!="pose"&&goalMode!="fixed_joint")
            throw std::runtime_error("Invalid shear pick_approach_goal_mode: "+goalMode);
          const bool poseGoal=(goalMode=="pose");
          std::vector<std::string> chosen;
          std::vector<double> velocity;
          for(const auto& goal:targets) {
            auto config=require(c,goal.robot);
            auto name=index==0?"first_pick_approach_planner":"subsequent_pick_approach_planner";
            auto select=s(config,name,s(p,name,"moveJ"));
            if(select!="moveJ"&&select!="moveL")throw std::runtime_error("Unsupported shear P1 planner "+select);
            chosen.push_back(select);
            velocity.push_back(select=="moveL"?scales.front():tuning("moveJ").vel);
          }
          if(targets.size()==2)
            motion(id+"_"+std::to_string(index+1),targets,"moveJ","merged",combined,
              contexts.front().object,std::to_string(index+1),chosen,velocity,{},poseGoal);
          else motion(id+"_"+std::to_string(index+1),targets,chosen.front(),"single",combined,
              contexts.front().object,std::to_string(index+1),{}, {}, {},poseGoal);
          actions(step["after_actions"],contexts);
          continue;
        }
        // Original shear code has post-motion contact updates. Apply only
        // explicitly listed legacy aliases; don't silently change ordering.
        std::vector<std::string> candidates=kind=="fallback"?strategyList:std::vector<std::string>{kind};
        bool planned=false;std::string prior;
        for(const auto& candidate:candidates) {
          std::vector<geometry_msgs::msg::Pose> vias;
          if(candidate=="moveC") {
            for(const auto& g:targets) {
              auto mid=g.pose;
              mid.position.x=f(require(c,g.robot),"via_world_x",g.pose.position.x);
              vias.push_back(mid);
            }
          }
          std::vector<double> attempts=candidate=="moveL"?scales:
            candidate=="moveC"?doubles(p["circ_velocity_scales"]):
            std::vector<double>{tuning("moveJ").vel};
          for(double v:attempts) {
            try {
              // Per-arm mixed strategies are handled by planMerged; no arm
              // silently changes mode after its plan has been sent.
              // Use the original shear node's pose-goal OMPL planning for
              // T1 MoveJ fallback as well as for P1. A single fixed IK target
              // can be unreachable from this arm's current configuration.
              const bool poseGoal=(id=="T1" && candidate=="moveJ");
              if(targets.size()==2)motion(id+"_"+std::to_string(index+1),targets,candidate,"merged",combined,
                 contexts.front().object,std::to_string(index+1),{candidate,candidate},{v,v},vias,poseGoal);
              else motion(id+"_"+std::to_string(index+1),targets,candidate,"single",combined,
                 contexts.front().object,std::to_string(index+1),{}, {},vias,poseGoal);
              planned=true;break;
            } catch(const std::exception& e) {
              prior=e.what();if(prior.find("EXECUTION_UNCERTAIN")!=std::string::npos)throw;
            }
          }
          if(planned)break;
        }
        // The legacy shear node supports DIFFERENT planners for the two arms
        // in the same merged tick. Try these combinations after uniform
        // LIN/CIRC/MoveJ have failed, without changing any pick/place targets.
        // Only T1 uses this fallback; every candidate is still full-state
        // validated before any combined execution is issued.
        if(!planned && id=="T1" && kind=="fallback" && targets.size()==2) {
          for(const std::array<std::string,2>& modes :
                {std::array<std::string,2>{"moveL","moveJ"},
                 std::array<std::string,2>{"moveJ","moveL"}}) {
            for(const double linScale : scales) {
              const std::vector<std::string> perArm{modes[0],modes[1]};
              const std::vector<double> vels{
                modes[0]=="moveL"?linScale:tuning("moveJ").vel,
                modes[1]=="moveL"?linScale:tuning("moveJ").vel};
              RCLCPP_INFO(node_->get_logger(),
                "[SHEAR T1 MIXED] pair=%zu robot1=%s robot2=%s LIN scale=%.3f",
                index+1,modes[0].c_str(),modes[1].c_str(),linScale);
              try {
                motion(id+"_"+std::to_string(index+1),targets,"moveJ","merged",combined,
                  contexts.front().object,std::to_string(index+1),perArm,vels,{},true);
                planned=true;
                break;
              } catch(const std::exception& e) {
                prior=e.what();
                if(prior.find("EXECUTION_UNCERTAIN")!=std::string::npos)throw;
              }
            }
            if(planned)break;
          }
        }
        if(!planned)throw std::runtime_error("Shear planning failed (no execution): "+id+" "+prior);
        actions(step["after_actions"],contexts);
        // G2 retract must not silently move an object that has just been placed.
        if(id=="G2")for(const auto& ctx:contexts)
          verifyPlacedObject(ctx.object,"after_G2_"+std::to_string(index+1));
        stageAcm(id,index+1,false);
      }
    }
    if(!activeScopedAcm.empty())throw std::runtime_error("YAML ACM scopes not restored by end of shear sequence");
    // Finish/home while task-level permissions are active. After home, restore.
    home("H1",{"robot1","robot2"},combined);
    if(batchActive) {collisionBatch(taskAcmPairs,false);batchActive=false;activeTaskAcmPairs_.clear();}
    } catch(...) {
      // Restore even if planning or validation failed; never leave a temporary
      // permission enabled on an aborted motion without an explicit warning.
      for(auto it=activeScopedAcm.rbegin();it!=activeScopedAcm.rend();++it) {
        try {collision(it->first,it->second,false);}
        catch(const std::exception& e) {
          RCLCPP_ERROR(node_->get_logger(),"[YAML ACM SCOPE] cleanup failed %s / %s: %s",
                       it->first.c_str(),it->second.c_str(),e.what());
        }
      }
      activeTaskAcmPairs_.clear();
      if(batchActive) {
        // Avoid an unverified partial update being left active on abort.
        std::vector<std::pair<std::string,std::string>> saved;
        for(const auto& pair:taskAcmPairs)if(oldAcm_.count(pair))saved.push_back(pair);
        if(!saved.empty())try {collisionBatch(saved,false);}
        catch(const std::exception& e) {
          RCLCPP_ERROR(node_->get_logger(),"[YAML ACM TASK] abort cleanup failed: %s",e.what());
        }
      }
      throw;
    }
  }
  void runTying() {
    if(require(task_,"point_steps").size()!=3)throw std::runtime_error("Tying requires three point steps");
    auto c=require(task_,"config"),p=require(c,"planning");
    struct Point{std::string robot,frame,clamp;};
    std::map<std::string,std::vector<Point>> jobs;
    for(auto robot:{"robot1","robot2"}) {
      const auto armCfg=require(c,robot);if(!b(armCfg,"enabled",true))continue;
      for(const auto& j:require(armCfg,"jobs")) {
        for(int i=1;i<=integer(j,"tie_count",0);++i)jobs[robot].push_back({robot,s(j,"clamp")+"_t"+std::to_string(i),s(j,"clamp")});
      }
    }
    const size_t count=std::max(jobs["robot1"].size(),jobs["robot2"].size());
    const auto combined=s(task_["execution"],"combined_group","sa3_robots_1_2");
    const auto approach=require(c,"approach");const auto offset=vec(approach["xyz"]);const auto offsetFrame=s(approach,"offset_frame","local");
    for(size_t i=0;i<count;++i) {
      bool failedPoint=false;
      for(const auto& step:require(task_,"point_steps")) {
        const auto id=s(step,"id"),reference=s(step,"reference"),kind=s(step,"motion");
        std::vector<ArmGoal> targets;
        for(auto robot:{"robot1","robot2"}) {
          if(i>=jobs[robot].size())continue;
          const auto pt=jobs[robot][i];auto pose=framePose(pt.frame);
          if(reference=="tie_approach")pose=applyOffset(pose,offset,offsetFrame);
          targets.push_back({robot,arm(robot,"tying").link,pose});
        }
        if(targets.empty())continue;
        std::vector<std::string> strategies=kind=="fallback"?
          strs(p["approach_strategies"] ? p["approach_strategies"]:p["transit_strategies"]):std::vector<std::string>{kind};
        if(strategies.empty())strategies={"moveL","moveJ"};
        std::string lastError;bool success=false;
        for(const auto& strategy:strategies) {
          std::vector<geometry_msgs::msg::Pose> vias;
          if(strategy=="moveC") {
            for(const auto& goal:targets) {
              auto current=group(arm(goal.robot).group).getCurrentPose(goal.link).pose;
              auto via=goal.pose;
              const auto offset=vec(p["circ_via_offset_world"]);
              via.position.x=0.5*(current.position.x+via.position.x)+offset[0];
              via.position.y=0.5*(current.position.y+via.position.y)+offset[1];
              via.position.z=0.5*(current.position.z+via.position.z)+offset[2];
              vias.push_back(via);
            }
          }
          auto scales=strategy=="moveL"?doubles(p["lin_velocity_scales"]):
            strategy=="moveC"?doubles(p["circ_velocity_scales"]):
            std::vector<double>{f(p,"movej_velocity",0.3)};
          for(double scale:scales) {
            try {
              if(targets.size()==2)motion(id+"_"+std::to_string(i+1),targets,strategy,"merged",combined,"",std::to_string(i+1),{strategy,strategy},{scale,scale},vias);
              else motion(id+"_"+std::to_string(i+1),targets,strategy,"single",combined,"",std::to_string(i+1),{}, {},vias);
              success=true;break;
            }catch(const std::exception& e){lastError=e.what();if(lastError.find("EXECUTION_UNCERTAIN")!=std::string::npos)throw;}
          }
          if(success)break;
        }
        if(!success) {
          if(b(c,"stop_on_first_failure",false))throw std::runtime_error("Tying point planning failed: "+lastError);
          RCLCPP_ERROR(node_->get_logger(),"Tying point %zu failed before execution; skipping remaining point steps",i+1);
          failedPoint=true;break;
        }
      }
      if(failedPoint)++tyingFailures_;
    }
    home("H1",{"robot1","robot2"},combined);
    if(tyingFailures_)throw std::runtime_error(std::to_string(tyingFailures_)+" tying points failed; report records incomplete work");
  }
  std::string dockFrame(const std::string& robot,const std::string& tool) {
    const auto d=workcell_["tool_dock_frames"];
    const std::string key=robot+(tool=="schunk"?"_shuck_dock":"_tyinggun_dock");
    return s(d[key],"child",key);
  }
  std::vector<std::string> toolIds(const std::string& robot,const std::string& tool) {
    std::vector<std::string> ids;
    if(tool=="tying_gun")return {robot+"_tying_gun"};
    const auto tools=workcell_["extra_grippers"];
    const auto prefix=robot+"_schunk_";
    if(tools && tools.IsMap())for(const auto& kv:tools){
      auto id=kv.first.as<std::string>();if(id.compare(0,prefix.size(),prefix)==0)ids.push_back(id);
    }
    if(ids.empty())ids={robot+"_schunk_base"};
    std::sort(ids.begin(),ids.end());return ids;
  }
  void gripperAction(const std::string& alias,const std::string& robot) {
    const std::string source=s(task_,"source_tool"),target=s(task_,"target_tool");
    auto a=arm(robot,"tool");auto touch=strs(workcell_["robots"][robot]["gripper_change_arm_touch_links"]);
    if(touch.empty())touch={a.tool_mount,robot+"_J4_link",robot+"_J5_link",robot+"_J6_link"};
    if(alias=="detach_source")for(const auto& id:toolIds(robot,source))detach(id,a.tool_mount);
    else if(alias=="attach_target")for(const auto& id:toolIds(robot,target))attach(id,a.tool_mount,touch);
    else if(alias=="allow_source_contacts"||alias=="restore_source_contacts"||alias=="allow_target_contacts"||alias=="restore_target_contacts") {
      const bool enabling=alias.find("allow_")==0;
      const auto tool=alias.find("source")!=std::string::npos?source:target;
      for(const auto& id:toolIds(robot,tool))collisionSet(id,touch,enabling);
    } else throw std::runtime_error("Unknown tool change action: "+alias);
  }
  void runGripper() {
    if(require(task_,"steps").size()!=6 || s(task_,"source_tool")==s(task_,"target_tool"))
      throw std::runtime_error("Tool change requires six steps and distinct tools");
    // Tool IO (mechanical lock/unlock) is not in the supplied legacy code.
    // Refuse real hardware until external lock feedback is integrated.
    if(!dryRun_ && !param<bool>("simulation_confirmed",false))
      throw std::runtime_error("Tool change requires simulation_confirmed:=true; hardware tool-lock feedback not implemented");
    if(param<bool>("wait_for_gantry_g5",false)) {
      waitTrigger("/gantry_g5_complete_status","G5_COMPLETE",param<double>("gantry_g5_wait_timeout_sec",240));
    }
    const auto combined=s(task_["execution"],"combined_group","sa3_robots_1_2");
    for(const auto& step:require(task_,"steps")) {
      const auto id=s(step,"id"),reference=s(step,"reference"),motionName=s(step,"motion");
      std::vector<ArmGoal> goals;
      for(auto robot:{"robot1","robot2"}) {
        auto a=arm(robot,"tool");
        const auto dock=reference=="source_dock"?dockFrame(robot,s(task_,"source_tool")):dockFrame(robot,s(task_,"target_tool"));
        auto pose=applyOffset(framePose(dock),vec(step["xyz"]),s(step,"offset_frame","world"),vec(step["rpy"]),"local");
        goals.push_back({robot,a.link,pose});
      }
      motion(id,goals,motionName,motionName=="moveJ"?"combined_group":"merged",combined);
      if(step["after_actions"])for(const auto& act:step["after_actions"])
        for(auto robot:{"robot1","robot2"})gripperAction(act.as<std::string>(),robot);
    }
    notifyCompletion("robot12",true);
  }
  void makeCheckpointServices() {
    g4Status_=node_->create_service<Trigger>("/gantry_checkpoint_status",
      [this](const Trigger::Request::SharedPtr,Trigger::Response::SharedPtr response){
        std::lock_guard<std::mutex> lock(checkpointMutex_);
        const bool good=g4Held_&&verifyGantryAtHandoff()&&checkAttached(gantryObject_,gantryAttachLink_);
        response->success=good;response->message=good?"G4_HOLD_VERIFIED":"G4_NOT_HELD_OR_INVALID";
      });
    g4Resume_=node_->create_service<Trigger>("/resume_gantry_motion",
      [this](const Trigger::Request::SharedPtr,Trigger::Response::SharedPtr response){
        std::lock_guard<std::mutex> lock(checkpointMutex_);
        response->success=g4Held_&&verifyGantryAtHandoff()&&checkAttached(gantryObject_,gantryAttachLink_);
        if(response->success){g4ResumeRequested_=true;checkpointCv_.notify_all();}
        response->message=response->success?"G4_RESUME_ACCEPTED":"G4_NOT_VERIFIED";
      });
    r3Status_=node_->create_service<Trigger>("/gantry_robot3_pick_checkpoint_status",
      [this](const Trigger::Request::SharedPtr,Trigger::Response::SharedPtr response){
        std::lock_guard<std::mutex> lock(checkpointMutex_);
        const bool good=r3Held_&&verifyGantryAtHandoff()&&checkAttached(gantryObject_,arm("robot3").link);
        response->success=good;response->message=good?"R3_HOLD_VERIFIED":"R3_NOT_HELD_OR_INVALID";
      });
    r3Resume_=node_->create_service<Trigger>("/resume_robot3_place",
      [this](const Trigger::Request::SharedPtr,Trigger::Response::SharedPtr response){
        std::lock_guard<std::mutex> lock(checkpointMutex_);
        response->success=r3Held_&&verifyGantryAtHandoff()&&checkAttached(gantryObject_,arm("robot3").link);
        if(response->success){r3ResumeRequested_=true;checkpointCv_.notify_all();}
        response->message=response->success?"R3_RESUME_ACCEPTED":"R3_NOT_VERIFIED";
      });
    g5Status_=node_->create_service<Trigger>("/gantry_g5_complete_status",
      [this](const Trigger::Request::SharedPtr,Trigger::Response::SharedPtr response){
        std::lock_guard<std::mutex> lock(checkpointMutex_);
        response->success=g5Complete_;response->message=g5Complete_?"G5_COMPLETE":"G5_INCOMPLETE";
      });
  }
  bool verifyGantryAtHandoff() {
    double x=0,z=0;return jointPosition(gantryHorizontal_,x)&&jointPosition(gantryVertical_,z)&&
      std::abs(x-gantryHandoffX_)<=gantryTolerance_&&std::abs(z-gantryHomeZ_)<=gantryTolerance_;
  }
  void holdUntilResume(bool g4) {
    if(dryRun_)return;
    std::unique_lock<std::mutex> lock(checkpointMutex_);
    if(g4)g4Held_=true;else r3Held_=true;
    const auto budget=std::chrono::duration<double>(param<double>("checkpoint_timeout_sec",1800.));
    bool resumed=checkpointCv_.wait_for(lock,budget,[&]{return !rclcpp::ok()||(g4?g4ResumeRequested_:r3ResumeRequested_);});
    if(g4)g4Held_=false;else r3Held_=false;
    lock.unlock();
    if(!resumed||!rclcpp::ok())throw std::runtime_error("Checkpoint resume timeout/shutdown");
    if(!verifyGantryAtHandoff())throw std::runtime_error("Checkpoint gantry drift before resume");
  }
  void gantryStep(const YAML::Node& step) {
    const auto id=s(step,"id"),jointType=s(step,"joint"),ref=s(step,"reference");
    const std::string joint=jointType=="horizontal"?gantryHorizontal_:gantryVertical_;
    double position=0;
    if(jointType=="horizontal")position=ref=="pick"?gantryPickX_:ref=="handoff"?gantryHandoffX_:0.0;
    else position=ref=="pick"?gantryPickZ_:ref=="handoff"?gantryHandoffZ_:gantryHomeZ_;
    if(jointType!="horizontal"&&jointType!="vertical")throw std::runtime_error("Unknown gantry axis");
    const auto rp=require(require(task_,"config"),"ros_parameters");
    if(jointType=="horizontal" &&
       (position<f(rp,"horizontal_min_m",-10.6)||position>f(rp,"horizontal_limit_m",10.6)))
      throw std::runtime_error("Gantry X target outside configured limits");
    if(jointType=="vertical" &&
       (position<0.0||position>f(rp,"vertical_limit_m",6.6)))
      throw std::runtime_error("Gantry Z target outside configured limits");
    controllerJoint(id,joint,position,gantrySpeed_,gantryAcceleration_,
      s(rp,"controller_action","/gantry_controller/follow_joint_trajectory"),strs(rp["controller_joints"]));
    const ContextPair ctx{"gantry",gantryObject_,"","",gantryAttachLink_,{gantryAttachLink_}};
    actions(step["after_actions"],{ctx});
  }
  void runGantry() {
    auto cfg=require(task_,"config"),rp=require(cfg,"ros_parameters"),planning=require(cfg,"planning");
    gantryObject_=s(rp,"rebar_object_id","rebar_1A");gantryAttachLink_=s(rp,"gantry_attach_link","gantry_part_link");
    gantryHorizontal_=s(rp,"horizontal_joint","gantry_slide_joint");
    gantryVertical_=s(rp,"vertical_joint","gantry_down_joint");
    gantryPickX_=f(rp,"horizontal_pick_target_m",0);gantryPickZ_=f(rp,"vertical_pick_target_m",4.1);
    gantryHandoffX_=f(rp,"horizontal_target_m",-6);gantryHandoffZ_=f(rp,"down_target_m",1.2);
    gantryHomeZ_=f(rp,"vertical_home_m",0);
    gantrySpeed_=f(rp,"speed_mps",0.3);gantryAcceleration_=f(rp,"acceleration_mps2",0.5);
    gantryTolerance_=param<double>("checkpoint_tolerance_m",0.01);
    makeCheckpointServices();
    const auto gs=require(task_,"gantry_steps");
    const auto r3=require(task_,"robot3_steps"),r4=require(task_,"robot4_steps");
    if(gs.size()!=5||r3.size()!=7||r4.size()!=7)
      throw std::runtime_error("Gantry program must have G1-G5 and Robot3/4 R1-R7");
    for(size_t i=0;i<5;++i)if(s(gs[i],"id")!="G"+std::to_string(i+1))
      throw std::runtime_error("Gantry step ordering must remain G1-G5");
    for(size_t i=0;i<7;++i)if(s(r3[i],"id")!="R"+std::to_string(i+1)||
                                  s(r4[i],"id")!="R"+std::to_string(i+1))
      throw std::runtime_error("Robot handoff step ordering must remain R1-R7");
    // G1-G4, checkpoint, G5: no G5 while G4 is held.
    for(size_t i=0;i<4;++i)gantryStep(gs[i]);
    if(!dryRun_ && !verifyGantryAtHandoff())throw std::runtime_error("G4 gantry position not verified");
    if(param<bool>("pause_after_g4",true))holdUntilResume(true);
    gantryStep(gs[4]);
    {std::lock_guard<std::mutex> lock(checkpointMutex_);g5Complete_=true;}
    const std::string combined=s(task_["execution"],"combined_group","robot3_robot4_arms");
    const bool useRobot4=b(rp,"use_robot4",true);
    for(size_t i=0;i<r3.size();++i) {
      const auto st=r3[i];const auto id=s(st,"id");
      if(id=="R7") {home(id,{"robot3","robot4"},combined);continue;}
      const ContextPair context{"robot3",gantryObject_,"","",arm("robot3").link,arm("robot3").touch};
      actions(st["before_actions"],{context});
      std::vector<ArmGoal> targets;
      for(auto robot:{"robot3","robot4"}) {
        if(robot==std::string("robot4")&&!useRobot4)continue;
        const auto source=robot==std::string("robot3")?st:r4[i];
        const auto reference=s(source,"reference");const auto frame=reference=="pick_frame"?
          s(rp,robot==std::string("robot3")?"robot3_pick_frame":"robot4_pick_frame"):
          s(rp,robot==std::string("robot3")?"robot3_place_frame":"robot4_place_frame");
        const auto p=applyOffset(framePose(frame),vec(source["xyz"]),s(source,"offset_frame","world"),vec(source["rpy"]),"local");
        targets.push_back({robot,arm(robot).link,p});
      }
      const auto motionKind=s(st,"motion"),planner=s(st,"planner");
      std::vector<std::string> attempts{motionKind};
      for(const auto& extra:strs(st["fallback_motions"]))attempts.push_back(extra);
      // Gantry handoff R1: combined OMPL to one fixed IK solution can fail
      // even when each arm has a reachable Cartesian pose goal. Keep the
      // original combined-group attempt first, then use independently planned
      // pose-goal OMPL trajectories, synchronized and full-state validated.
      // R4 also supports this safe, full-state-validated fallback when the
      // combined OMPL plan is rejected. All other steps keep their paths.
      constexpr const char* kR1PoseFallback="moveJ_pose_merged";
      if((id=="R1" || id=="R4") && useRobot4 && motionKind=="moveJ")
        attempts.push_back(kR1PoseFallback);
      bool moved=false;std::string lastError;
      for(const auto& attempt:attempts) {
        const bool r1PoseFallback=(attempt==kR1PoseFallback);
        const auto requestedMotion=r1PoseFallback?std::string("moveJ"):attempt;
        const auto strategy=r1PoseFallback?std::string("merged"):
          useRobot4?(attempt=="moveJ"?"combined_group":
          (attempt!=motionKind || planner=="cartesian")?"coordinated_cartesian":"merged"):
          ((attempt!=motionKind || planner=="cartesian")?"coordinated_cartesian":"single");
        try {
          if(r1PoseFallback)
            RCLCPP_WARN(node_->get_logger(),
              "[GANTRY %s] Combined plan failed; trying merged per-arm OMPL pose goals",
              id.c_str());
          motion(id,targets,requestedMotion,strategy,
            useRobot4?combined:arm("robot3").group,
            "","",{},{},{},r1PoseFallback);
          moved=true;break;
        }catch(const std::exception& e){
          lastError=e.what();
          RCLCPP_WARN(node_->get_logger(),
            "[GANTRY %s] planner attempt=%s failed: %s",
            id.c_str(),attempt.c_str(),lastError.c_str());
          const bool planningFailure=(lastError.find("Planning failed")!=std::string::npos ||
             lastError.find("planning failed")!=std::string::npos ||
             lastError.find("IK failed")!=std::string::npos ||
             lastError.find("pose target rejected")!=std::string::npos);
          if(!planningFailure)throw;
        }
      }
      if(!moved)throw std::runtime_error("Gantry paired motion planners exhausted: "+lastError);
      if(id=="R3") {
        // After robot3 has attached/retracted the part, restore raised gantry Z
        // before announcing a verified R3 hold.
        controllerJoint("G5_RETURN_UP",gantryVertical_,gantryHomeZ_,gantrySpeed_,gantryAcceleration_,
          s(rp,"controller_action","/gantry_controller/follow_joint_trajectory"),strs(rp["controller_joints"]));
      }
      // The lift and wait actions are managed by R3 and the checkpoint, not
      // by detached asynchronous threads that might outlive the task.
      actions(st["after_actions"],{context});
      if(id=="R3"&&param<bool>("pause_after_r3",true))holdUntilResume(false);
    }
    notifyCompletion("robot3",true);
  }
  std::shared_ptr<Trigger::Response> requestTrigger(const std::string& service) {
    auto client=node_->create_client<Trigger>(service);
    if(!client->wait_for_service(5s))throw std::runtime_error("Service unavailable: "+service);
    auto future=client->async_send_request(std::make_shared<Trigger::Request>());
    if(future.wait_for(10s)!=std::future_status::ready)throw std::runtime_error("Service timeout: "+service);
    return future.get();
  }
  void waitTrigger(const std::string& service,const std::string& contains,double timeout) {
    const auto limit=Clock::now()+std::chrono::duration<double>(timeout);
    while(rclcpp::ok()&&Clock::now()<limit) {
      try {auto response=requestTrigger(service);
        if(response->success && response->message.find(contains)!=std::string::npos)return;
        // R3 is reached only after G5 lowers, the paired R1/R2 motions,
        // the R3 retract, and the gantry's verified return-up motion.
        // R3_NOT_HELD_OR_INVALID is therefore a *pending* state while
        // waiting; treating it as an immediate abort races G5/R1-R3.
        if(response->message.find("INVALID")!=std::string::npos) {
          const bool r3_pending = service == "/gantry_robot3_pick_checkpoint_status" &&
            response->message == "R3_NOT_HELD_OR_INVALID";
          if(!r3_pending)
            throw std::runtime_error("Invalid checkpoint: "+response->message);
        }
      }catch(const std::runtime_error& e) {
        const std::string error=e.what();
        if(error.find("Invalid checkpoint:")==0)throw;
      }
      std::this_thread::sleep_for(200ms);
    }
    throw std::runtime_error("Timeout waiting for verified checkpoint: "+service);
  }
  void waitCheckpoint(const std::string& key,double timeout=-1) {
    auto check=require(require(recipe_,"checkpoints"),key);
    waitTrigger(s(check,"status_service"),s(check,"success_contains"),timeout>0?timeout:f(check,"timeout_sec",240));
  }
  void resumeCheckpoint(const std::string& key) {
    auto check=require(require(recipe_,"checkpoints"),key);
    auto result=requestTrigger(s(check,"resume_service"));
    if(!result->success)throw std::runtime_error("Checkpoint resume rejected: "+key+" "+result->message);
  }
  void verifyNamed(const YAML::Node& step) {
    auto a=arm(s(step,"robot"));auto state=currentState(group(a.group));
    auto expected=std::make_shared<moveit::core::RobotState>(*state);
    const auto jmg=state->getJointModelGroup(a.group);
    if(!jmg||!expected->setToDefaultValues(jmg,s(step,"named_state","home")))
      throw std::runtime_error("Cannot resolve named home: "+a.name);
    const double tol=f(step,"tolerance_rad",0.02);
    for(const auto& variable:jmg->getVariableNames()) {
      if(std::abs(state->getVariablePosition(variable)-expected->getVariablePosition(variable))>tol)
        throw std::runtime_error("Named state tolerance failed: "+a.name+" "+variable);
    }
  }
  void notifyCompletion(const std::string& owner,bool success) {
    if(!rclcpp::ok())return;
    try {auto reply=requestTrigger("/sa3_phase5/"+owner+(success?"_done":"_abort"));
      if(!reply->success)RCLCPP_ERROR(node_->get_logger(),"Completion notification rejected: %s",reply->message.c_str());
    }catch(const std::exception& e){RCLCPP_ERROR(node_->get_logger(),"Completion notification unavailable: %s",e.what());}
  }
  void runWorkflow() {
    using Service=rclcpp::Service<Trigger>::SharedPtr;
    std::vector<Service> completions;
    for(const auto owner:{"robot3","robot12"}) {
      const std::string who(owner);
      completions.push_back(node_->create_service<Trigger>("/sa3_phase5/"+who+"_done",
        [this,who](const Trigger::Request::SharedPtr,Trigger::Response::SharedPtr response){
          std::lock_guard<std::mutex> lock(phase5Mutex_);if(who=="robot3")robot3Done_=true;else robot12Done_=true;
          response->success=!phase5Abort_;response->message=response->success?"DONE_CONFIRMED":"ABORTED";
        }));
      completions.push_back(node_->create_service<Trigger>("/sa3_phase5/"+who+"_abort",
        [this,who](const Trigger::Request::SharedPtr,Trigger::Response::SharedPtr response){
          std::lock_guard<std::mutex> lock(phase5Mutex_);phase5Abort_=true;
          response->success=true;response->message=who+"_ABORT_RECORDED";
        }));
    }
    for(const auto& step:require(task_,"steps")) {
      const auto id=s(step,"id"),op=s(step,"action");
      if(op=="wait_checkpoint")waitCheckpoint(s(step,"checkpoint"),f(step,"timeout_sec",-1));
      else if(op=="resume_checkpoint")resumeCheckpoint(s(step,"checkpoint"));
      else if(op=="verify_named_state")verifyNamed(step);
      else if(op=="wait_completion_notifications") {
        const auto deadline=Clock::now()+std::chrono::duration<double>(f(step,"timeout_sec",1800));
        bool completed=false;
        while(rclcpp::ok()&&Clock::now()<deadline){
          {std::lock_guard<std::mutex> lock(phase5Mutex_);
           if(phase5Abort_)throw std::runtime_error("Phase5 child reported abort");
           if(robot3Done_&&robot12Done_){completed=true;break;}}
          std::this_thread::sleep_for(100ms);
        }
        if(!completed)throw std::runtime_error("Phase5 completion timeout");
      }else throw std::runtime_error("Unsupported workflow action: "+op);
      Row r;r.phase=phase_;r.task=taskId_;r.step=id;r.motion=op;r.status="SUCCESS";report(r);
    }
  }
  void finalContact(const std::vector<ContextPair>& owned,bool allow) {
    for(const auto& ctx:owned) {
      collisionSet(ctx.object,ctx.touch,allow);
      collision(ctx.object,s(recipe_["defaults"]["scene_objects"],"jig_clamp"),allow);
      collision(ctx.object,s(recipe_["defaults"]["scene_objects"],"main_jig"),allow);
    }
    for(size_t i=0;i<owned.size();++i)
      for(size_t j=i+1;j<owned.size();++j)collision(owned[i].object,owned[j].object,allow);
  }
  void runFinal() {
    const auto config=require(task_,"config");
    const auto groupName=s(config,"combined_group","robot3_robot4_arms");
    const auto activeMain=s(config,"object_id","rebar_1A");
    const auto children=strs(config["children"]),children3=strs(config["robot3_children"]),children4=strs(config["robot4_children"]);
    const auto mainOwner=integer(config,"main_object_robot",3);
    if(mainOwner!=3&&mainOwner!=4)throw std::runtime_error("Invalid final main object owner");
    std::set<std::string> expected(children.begin(),children.end()),found;
    if(expected.size()!=children.size())throw std::runtime_error("Duplicate final assembly child ID");
    for(const auto& child:children3)if(!expected.count(child)||!found.insert(child).second)throw std::runtime_error("Bad Robot3 child owner: "+child);
    for(const auto& child:children4)if(!expected.count(child)||!found.insert(child).second)throw std::runtime_error("Bad Robot4 child owner: "+child);
    if(found!=expected)throw std::runtime_error("Every final child must have exactly one owner");
    std::vector<ContextPair> owned;
    auto add=[&](const std::string& object,int robot) {
      if(!scene_.getObjects({object}).count(object)) {
        if(object==activeMain)throw std::runtime_error("Required main final object missing: "+object);
        RCLCPP_WARN(node_->get_logger(),"Optional final child absent: %s",object.c_str());return;
      }
      auto name="robot"+std::to_string(robot);
      auto link=s(config,(name+"_planning_link"),arm(name).link);
      std::vector<std::string> touch=robot==3?
        std::vector<std::string>{link,"robot3_J6_link","robot3_sa3_grpr01"}:
        std::vector<std::string>{link,"robot4_J6_link","robot4_sa3_grpr01_r"};
      owned.push_back({name,object,"","",link,touch});
    };
    add(activeMain,mainOwner);for(const auto& x:children3)add(x,3);for(const auto& x:children4)add(x,4);
    for(const auto& step:require(task_,"steps")) {
      const auto id=s(step,"id"),kind=s(step,"motion");
      if(kind=="named_state") {namedState(id,s(step,"robot"),s(step,"target"));continue;}
      if(s(step,"reference")=="home") {home(id,{"robot3","robot4"},groupName);continue;}
      const auto before=step["before_actions"];
      if(before)for(const auto& a:before) {
        auto alias=a.as<std::string>();if(alias=="allow_gripper_contact")finalContact(owned,true);
        else throw std::runtime_error("Unknown final before-action: "+alias);
      }
      const auto left=require(step,"robot3"),right=require(step,"robot4");
      std::vector<ArmGoal> goals={{"robot3",s(config,"robot3_planning_link",arm("robot3").link),resolveTarget(left)},
                                  {"robot4",s(config,"robot4_planning_link",arm("robot4").link),resolveTarget(right)}};
      if(kind=="fallback") {
        std::string lastError;bool ok=false;
        for(const auto& tryMotion:strs(config["transit_strategies"])) {
          if(tryMotion=="moveC")continue;
          try {
            motion(id,goals,tryMotion,tryMotion=="moveL"?"coordinated_cartesian":"combined_group",groupName);
            ok=true;break;
          }catch(const std::exception& e){lastError=e.what();if(lastError.find("EXECUTION_UNCERTAIN")!=std::string::npos)throw;}
        }
        if(!ok)throw std::runtime_error("Final fallback exhausted: "+lastError);
      } else {
        motion(id,goals,kind,kind=="moveL"?"coordinated_cartesian":"combined_group",groupName);
      }
      if(step["after_actions"])for(const auto& entry:step["after_actions"]) {
        const auto alias=entry.as<std::string>();
        if(alias=="attach_active_objects")for(const auto& c:owned)attach(c.object,c.link,c.touch);
        else if(alias=="detach_active_objects")for(const auto& c:owned)detach(c.object,c.link);
        else if(alias=="restore_gripper_contact")finalContact(owned,false);
        else throw std::runtime_error("Unknown final after-action: "+alias);
      }
    }
  }
  void runCombined(const YAML::Node& cfg,const std::string& name) {
    if(!param<bool>("execute_motion",false))
      throw std::runtime_error("Combined motion requires explicit execute_motion:=true");
    if(!dryRun_&&!param<bool>("simulation_confirmed",false))
      throw std::runtime_error("Combined execution is simulation-only until validated");
    const auto start=Clock::now();Row r;r.phase=phase_;r.task=name;r.step="COMBINED";r.robot="robot1+robot2+robot3+robot4";
    r.motion="moveJ";r.planner="OMPL full-group";r.status="FAILED";
    try {
      const std::string srdf=s(cfg,"group","sa3_all_robots");
      auto& mg=group(srdf);auto st=currentState(mg);
      auto goal=std::make_shared<moveit::core::RobotState>(*st);
      auto settings=cfg;Tuning t;t.vel=f(settings,"velocity_scale",0.3);t.acc=f(settings,"acceleration_scale",0.3);
      t.planning_time=f(settings,"planning_time",30);t.attempts=integer(settings,"planning_attempts",10);
      t.ik=f(settings,"ik_timeout",2);checkScale(t.vel);checkScale(t.acc);
      std::set<std::string> assigned;
      if(cfg["goals"] && cfg["goals"].IsSequence()) {
        for(const auto& entry:cfg["goals"]) {
          const auto robot=s(entry,"robot");
          if(!assigned.insert(robot).second)throw std::runtime_error("Duplicate combined goal for "+robot);
          const auto a=arm(robot);
          if(entry["named_state"]) {
            const auto jmg=goal->getJointModelGroup(a.group);
            if(!jmg||!goal->setToDefaultValues(jmg,s(entry,"named_state")))
              throw std::runtime_error("Missing combined named state: "+robot);
          } else if(!setPoseIK(*goal,{robot,s(entry,"link",a.link),resolveTarget(require(entry,"target"))},t.ik)) {
            throw std::runtime_error("Combined IK failed for "+robot);
          }
        }
      } else {
        throw std::runtime_error("Combined motion requires YAML combined_motions.<name>.goals; task-name-based goal inference is not supported");
      }
      setPlanning(mg,"moveJ",t);mg.setStartStateToCurrentState();
      if(!mg.setJointValueTarget(*goal))throw std::runtime_error("Full-group goal rejected");
      Plan result;auto before=Clock::now();auto code=mg.plan(result);r.planning=seconds(before);
      if(code!=moveit::core::MoveItErrorCode::SUCCESS)throw std::runtime_error("24-joint planning failed");
      r.validation=validate(result,srdf);
      MotionAttempt a;executePlan(srdf,result,a);r.execution=a.execution;
      r.status=dryRun_?"PLANNED_ONLY":"SUCCESS";
    }catch(const std::exception& e){r.error=e.what();r.elapsed=seconds(start);report(r);throw;}
    r.elapsed=seconds(start);report(r);
  }
  // Generic YAML operation interpreter. New pick/place recipes contain only
  // motions, object IDs, target frame specifications and operations in YAML.
  // No task ID appears in this dispatcher.
  void runProgram() {
    const auto ops=require(task_,"program");
    if(!ops.IsSequence() || ops.size()==0)
      throw std::runtime_error("program must be a nonempty YAML operation list");
    for(const auto& op:ops) {
      const std::string kind=s(op,"op"), id=s(op,"id",kind);
      if(kind=="move") {
        const std::string groupName=s(op,"group"),strategy=s(op,"strategy","combined_group");
        const std::string primary=s(op,"motion","moveJ");
        std::vector<ArmGoal> goals;
        for(const auto& entry:require(op,"goals")) {
          const std::string robot=s(entry,"robot"), link=s(entry,"link",arm(robot).link);
          goals.push_back({robot,link,resolveTarget(require(entry,"target"),task_[robot])});
        }
        if(goals.empty())throw std::runtime_error("Move requires one or more goals: "+id);
        // Step-specific overrides (e.g. faster reverse PTP B2) come from YAML.
        const auto overrides=op["planning_override"];
        const bool changed=overrides && overrides.IsMap();
        YAML::Node saved;
        if(changed) {
          if(!task_["planning"] || !task_["planning"].IsMap())
            throw std::runtime_error("planning_override requires task.planning map: "+id);
          saved=YAML::Clone(task_["planning"]);
          for(const auto& x:overrides)task_["planning"][x.first.as<std::string>()]=x.second;
        }
        try {
          std::vector<std::string> modes;
          modes.push_back(primary);
          if(op["fallback_motions"])for(const auto& f:op["fallback_motions"])
            modes.push_back(f.as<std::string>());
          bool succeeded=false;std::string prior;
          for(size_t modeIndex=0;modeIndex<modes.size();++modeIndex) {
            const auto& mode=modes[modeIndex];
            // Compare by fallback position, not name: a secondary moveL can
            // use coordinated_cartesian instead of retrying the same Pilz LIN.
            const std::string path=(modeIndex==0)?strategy:
              (mode=="moveL"?s(op,"fallback_linear_strategy","coordinated_cartesian"):
               s(op,"fallback_joint_strategy","combined_group"));
            if(modeIndex>0)RCLCPP_WARN(node_->get_logger(),
              "[PROGRAM PLANNING FALLBACK] step=%s motion=%s strategy=%s",
              id.c_str(),mode.c_str(),path.c_str());
            try {
              // Optional, explicitly configured joint-to-pose fallback for two-arm
              // program moves. Keep the existing combined MoveJ as first choice.
              // Each arm uses OMPL pose constraints (not one fixed IK branch),
              // from a shared captured start state. The merged result still
              // passes the full-cell validity gate inside motion().
              if(mode=="moveJ_pose_merged") {
                if(goals.size()!=2 || primary!="moveJ")
                  throw std::runtime_error("moveJ_pose_merged requires two MoveJ arm goals");
                RCLCPP_WARN(node_->get_logger(),
                  "[PROGRAM POSE FALLBACK] %s combined goal failed; planning per-arm OMPL pose targets",
                  id.c_str());
                motion(id,goals,"moveJ","merged",groupName,
                  "","",{}, {}, {},true);
              } else {
                motion(id,goals,mode,path,groupName);
              }
              succeeded=true;break;
            } catch(const std::exception& e) {
              prior=e.what();
              // Never retry if execution began, collision validation failed,
              // or a service/action did not acknowledge a result.
              const bool planningFailure=(prior.find("Planning failed")!=std::string::npos ||
                  prior.find("planning failed")!=std::string::npos ||
                  prior.find("IK failed")!=std::string::npos ||
                  prior.find("plan failed")!=std::string::npos);
              if(!planningFailure)throw;
              RCLCPP_WARN(node_->get_logger(),"%s: planner %s failed before execute: %s",
                id.c_str(),mode.c_str(),prior.c_str());
            }
          }
          if(!succeeded)throw std::runtime_error("Fallback planners exhausted: "+prior);
        }catch(...) {if(changed)task_["planning"]=saved;throw;}
        if(changed)task_["planning"]=saved;
      } else if(kind=="home") {
        if(!s(op,"skip_if_param").empty() && param<bool>(s(op,"skip_if_param"),false))continue;
        const auto robots=strs(require(op,"robots"));
        home(id,robots,s(op,"group"));
      } else if(kind=="named_state") {
        namedState(id,s(op,"robot"),s(op,"state"));
      } else if(kind=="attach") {
        auto begin=Clock::now();
        attach(s(op,"object"),s(op,"link"),strs(op["touch_links"]),s(op,"dock_frame"),
          s(op,"placement_mode","preserve_world"),f(op,"verify_pose_tolerance_m",0.02));
        Row r;r.phase=phase_;r.task=taskId_;r.step=id;r.robot=s(op,"robot");
        r.object=s(op,"object");r.motion="attach";r.scene=seconds(begin);
        r.status=dryRun_?"PLANNED_ONLY":"SUCCESS";report(r);
      } else if(kind=="detach") {
        auto begin=Clock::now();detach(s(op,"object"),s(op,"link"));
        Row r;r.phase=phase_;r.task=taskId_;r.step=id;r.robot=s(op,"robot");
        r.object=s(op,"object");r.motion="detach";r.scene=seconds(begin);
        r.status=dryRun_?"PLANNED_ONLY":"SUCCESS";report(r);
      } else if(kind=="acm") {
        const auto x=s(op,"object");
        for(const auto& other:strs(op["with"]))collision(x,other,b(op,"allow",false));
      } else if(kind=="scene_actions") {
        std::vector<ContextPair> contexts;
        for(const auto& ctx:require(op,"contexts")) {
          ContextPair c;c.robot=s(ctx,"robot");c.object=s(ctx,"object");
          c.inventory=s(ctx,"inventory");c.ring=s(ctx,"ring");
          c.link=s(ctx,"link");c.touch=strs(ctx["touch_links"]);
          contexts.push_back(c);
        }
        actions(require(op,"actions"),contexts);
      } else if(kind=="joint") {
        controllerJoint(id,s(op,"joint"),f(op,"position",0),
          f(op,"speed",0.3),f(op,"acceleration",0.5),
          s(op,"controller_action","/gantry_controller/follow_joint_trajectory"),
          strs(require(op,"controller_joints")));
      } else if(kind=="wait_checkpoint") {
        waitCheckpoint(s(op,"checkpoint"),f(op,"timeout_sec",-1));
      } else if(kind=="resume_checkpoint") {
        resumeCheckpoint(s(op,"checkpoint"));
      } else if(kind=="verify_named_state") {
        verifyNamed(op);
      } else if(kind=="wait_service") {
        if(s(op,"only_if_param").empty() || param<bool>(s(op,"only_if_param"),false))
          waitTrigger(s(op,"service"),s(op,"success_contains"),
            s(op,"timeout_param").empty()?f(op,"timeout_sec",120):
              param<double>(s(op,"timeout_param"),f(op,"timeout_sec",120)));
      } else if(kind=="sleep") {
        if(!dryRun_)std::this_thread::sleep_for(
          std::chrono::duration<double>(f(op,"duration_sec",0)));
      } else if(kind=="notify_completion") {
        notifyCompletion(s(op,"owner"),b(op,"success",true));
      } else if(kind=="paired_pick_place_sequence") {
        runShear();
      } else if(kind=="paired_tying_points") {
        runTying();
      } else if(kind=="gantry_handoff_sequence") {
        runGantry();
      } else if(kind=="checkpoint_workflow") {
        runWorkflow();
      } else if(kind=="multi_object_pick_place_sequence") {
        runFinal();
      } else {
        throw std::runtime_error("Unsupported universal YAML op '"+kind+"' at "+id);
      }
    }
  }

  void runTask() {
    const auto type=s(task_,"type");
    RCLCPP_INFO(node_->get_logger(),"SA3 generic task=%s type=%s dry_run=%d",taskId_.c_str(),type.c_str(),int(dryRun_));
    fullCollisionGroup_=s(recipe_["defaults"],"collision_validation_group","sa3_all_robots");
    if(!dryRun_&&!param<bool>("simulation_confirmed",false))
      throw std::runtime_error("Execution requires simulation_confirmed:=true; generic engine is not validated for hardware");
    if(!task_["program"])throw std::runtime_error("Task requires YAML program: "+taskId_);
    runProgram();
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  moveit::planning_interface::PlanningSceneInterface scene_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr getScene_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr applyScene_;
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr validity_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSub_;
  std::unordered_map<std::string,std::unique_ptr<MGI>> groups_;
  std::map<std::string,ArmDef> arms_;
  std::map<std::pair<std::string,std::string>,std::optional<bool>> oldAcm_;
  std::vector<std::pair<std::string,std::string>> activeTaskAcmPairs_;
  std::map<std::string,geometry_msgs::msg::Pose> placedObjectPoses_;
  std::mutex jointMutex_,checkpointMutex_,phase5Mutex_,acmReadMutex_;
  std::condition_variable checkpointCv_;
  std::map<std::string,double> jointPositions_;
  rclcpp::Service<Trigger>::SharedPtr g4Status_,g4Resume_,r3Status_,r3Resume_,g5Status_;
  bool g4Held_{false},r3Held_{false},g4ResumeRequested_{false},r3ResumeRequested_{false},g5Complete_{false};
  bool robot3Done_{false},robot12Done_{false},phase5Abort_{false};
  std::string gantryObject_,gantryAttachLink_,gantryHorizontal_,gantryVertical_;
  double gantryPickX_{0},gantryPickZ_{0},gantryHandoffX_{0},gantryHandoffZ_{0},gantryHomeZ_{0};
  double gantrySpeed_{0.3},gantryAcceleration_{0.5},gantryTolerance_{0.01};
  int tyingFailures_{0};
  bool dryRun_{true},reportWritten_{false};
  YAML::Node recipe_,task_,workcell_;
  std::string configPath_,workcellPath_,taskId_,phase_,fullCollisionGroup_="sa3_all_robots";
  Clock::time_point start_;
  std::vector<Row> rows_;
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("sa3_generic_executor",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),4);
  executor.add_node(node);
  std::thread spin([&]{executor.spin();});
  int code=1;
  std::unique_ptr<GenericExecutor> app;
  try {app=std::make_unique<GenericExecutor>(node);code=app->run();}
  catch(const std::exception& e){RCLCPP_FATAL(node->get_logger(),"Generic executor startup failure: %s",e.what());}
  executor.cancel();if(spin.joinable())spin.join();
  app.reset();
  executor.remove_node(node);
  rclcpp::shutdown();return code;
}
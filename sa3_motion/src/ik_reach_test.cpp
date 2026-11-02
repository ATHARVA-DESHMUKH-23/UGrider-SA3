// SA3 schema-v2 recipe audit. Planning only: NEVER calls execute(), move(), or controllers.
// Not a substitute for production scene mutation, attachment, or synchronized execution.
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>
using MGI=moveit::planning_interface::MoveGroupInterface;
namespace {
using V3=std::array<double,3>;
struct Target {
 std::string task,group,link,id,frame,motion="moveJ",planner="",description="",joint_name="";
 V3 xyz{0,0,0},rpy{0,0,0}; bool local=false,rpy_local=true,home=false,joint=false,unsupported=false;
 double joint_value=0,vel=.1,acc=.1,planning_time=15,ik_timeout=.2;
 int attempts=1,retries=1; std::vector<double> lin_scales; std::string note;
};
std::string val(const YAML::Node& n,const char* k,const std::string& d="") {return n&&n[k]?n[k].as<std::string>():d;}
double num(const YAML::Node& n,const char* k,double d) {return n&&n[k]?n[k].as<double>():d;}
int integer(const YAML::Node& n,const char* k,int d) {return n&&n[k]?n[k].as<int>():d;}
V3 vec(const YAML::Node& n,const char* k,V3 d={0,0,0}) {
 if (!n || !n[k]) return d;
 auto x = n[k];
 if (!x.IsSequence() || x.size() != 3)
  throw std::runtime_error(std::string("Invalid vector ") + k);
 return {x[0].as<double>(),x[1].as<double>(),x[2].as<double>()};
}
std::vector<double> scales(const YAML::Node& n,const char* k) {std::vector<double> v; if(n&&n[k])for(const auto& x:n[k])v.push_back(x.as<double>());return v;}
std::string csv(const std::string& x){std::string y="\"";for(char c:x){if(c=='\"')y+='\"';y+=c;}return y+'\"';}
void tuning(Target& t,const YAML::Node& p){
 t.planning_time=num(p,"planning_time",15);t.attempts=integer(p,"planning_attempts",integer(p,"ompl_attempts",10));
 t.retries=integer(p,"retries",integer(p,"plan_retries",1));t.ik_timeout=num(p,"ik_timeout",.2);
 t.vel=num(p,"movej_velocity",.1);t.acc=num(p,"movej_acceleration",.1);
 t.lin_scales=scales(p,"lin_velocity_scales");
}
void add(std::vector<Target>& out,Target t){out.push_back(std::move(t));}
Target make(const std::string& task,const std::string& group,const std::string& link,const std::string& id,
 const std::string& frame,const std::string& motion,const YAML::Node& p){Target t; t.task=task;t.group=group;t.link=link;t.id=id;t.frame=frame;t.motion=motion;tuning(t,p);return t;}
void shear(std::vector<Target>& out,const std::string& name,const YAML::Node& task,const YAML::Node& robots){
 auto c=task["config"],p=c["planning"];auto stages=task["stages"];
 for(auto key:{"robot1","robot2"}){
  auto arm=c[key],physical=robots[key];auto group=val(physical,"group"),link=val(physical,"planning_link");
  int n=0;for(const auto& pair:arm["pairs"]){++n;
   const auto pick = val(pair, "pick_frame");
   const auto place = val(pair, "place_frame");
   const auto d = vec(pair, "pick_offset");
   const auto a = vec(c["approach"], "xyz");
   const bool local=val(c["approach"],"offset_frame")=="local";
   for(const auto& st:stages){auto ref=val(st,"reference"),motion=val(st,"motion");
    Target t=make(name,group,link,std::string(key)+"/pair"+std::to_string(n)+"/"+val(st,"id"),
     (ref=="pick"||ref=="pick_approach")?pick:place,motion,p);
    if(ref=="pick"||ref=="pick_approach")t.xyz=d;
    if(ref=="pick_approach"||ref=="place_approach"){
     // Approach is applied relative to the initial TF in the production nodes.
     for (int i = 0; i < 3; ++i) {
      t.xyz[i] += a[i];
     }
     t.local = local;
     if(local && ref=="pick_approach" && d!=V3{0,0,0})t.note="mixed local approach plus pick offset; approximate";
    }
    if(ref=="place_approach"&&motion=="fallback")t.note="LIN -> CIRC via -> OMPL; CIRC via geometry not reproduced";
    add(out,t);
   }
  }
  if(task["finish"]){auto t=make(name,group,link,std::string(key)+"/home","home","moveJ",p);t.home=true;add(out,t);}
 }
}
void tying(std::vector<Target>& out,const std::string& name,const YAML::Node& task,const YAML::Node& robots){
 auto c=task["config"],p=c["planning"];auto a=vec(c["approach"],"xyz");bool local=val(c["approach"],"offset_frame")=="local";
 for(auto key:{"robot1","robot2"}){
  auto arm=c[key],physical=robots[key];if(arm["enabled"]&&!arm["enabled"].as<bool>())continue;
  auto group=val(physical,"group"),link=val(physical,"tying_link");
  for(const auto& job:arm["jobs"]){const auto clamp=val(job,"clamp");int count=integer(job,"tie_count",0);
   for(int i=1;i<=count;++i){auto frame=clamp+"_t"+std::to_string(i);
    for(const auto& st:task["point_steps"]){auto ref=val(st,"reference");
     auto t=make(name,group,link,std::string(key)+"/"+frame+"/"+val(st,"id"),frame,val(st,"motion"),p);
     if(ref=="tie_approach"){t.xyz=a;t.local=local;} add(out,t);
    }
   }
  }
  if(task["finish"]){auto t=make(name,group,link,std::string(key)+"/home","home","moveJ",p);t.home=true;add(out,t);}
 }
}
void flange(std::vector<Target>& out,const std::string& name,const YAML::Node& task,const YAML::Node& robots){
 auto p=task["planning"];
 for(const auto& st:task["steps"]){auto id=val(st,"id"),motion=val(st,"motion");
  for(auto key:{"robot3","robot4"}){
   auto arm=task[key],phys=robots[key];auto group=val(phys,"group"),link=val(phys,"link");
   if(val(st,"reference")=="home") {auto t=make(name,group,link,id+"/"+key,"home","moveJ",p);t.home=true;add(out,t);continue;}
   auto spec=st[key]; if(!spec)continue;
   auto ref=val(spec,"reference");
   if(ref=="pick_frame")ref=val(arm,"pick_frame");
   if(ref=="place_frame")ref=val(arm,"place_frame");
   auto t=make(name,group,link,id+"/"+key,ref,motion,p);
   t.xyz=vec(spec,"xyz");t.rpy=vec(spec,"rpy");t.local=val(spec,"offset_frame")=="local";
   t.rpy_local=val(spec,"rpy_frame","local")=="local";
   if(val(spec,"reference")=="pick_frame"){
    auto off=vec(arm,"pick_offset");for(int j=0;j<3;++j)t.xyz[j]+=off[j];
    auto rot=vec(arm,"pick_offset_rpy");for(int j=0;j<3;++j)t.rpy[j]+=rot[j];
    if(val(arm,"pick_offset_frame","world")!=val(spec,"offset_frame","world")||
      val(arm,"pick_offset_rpy_frame","local")!=val(spec,"rpy_frame","local"))
      t.note="pick offset composition is approximate when offset frames differ";
   }
   if(val(spec,"reference")=="place_frame"){
    auto off=vec(arm,"place_offset");for(int j=0;j<3;++j)t.xyz[j]+=off[j];
   }
   if(ref=="arc_waypoint"){
    t.unsupported=true;t.note="sampled arc requires production arcThrough3 interpolation; not independently planned";
   }
   add(out,t);
  }
 }
}
void gantry(std::vector<Target>& out,const std::string& name,const YAML::Node& task,const YAML::Node& w){
 auto c=task["config"],rp=c["ros_parameters"],hw=w["gantry_robot"],p=c["planning"];
 for(const auto& step:task["gantry_steps"]){auto j=val(step,"joint"),r=val(step,"reference");
  Target t; t.task=name;t.id="gantry/"+val(step,"id");t.motion="joint";t.joint=true;
  t.group=val(hw,"move_group_name","robot3_arm");t.joint_name=val(hw,j=="horizontal"?"horizontal_joint":"vertical_joint",j=="horizontal"?"gantry_slide_joint":"gantry_down_joint");
  const char* k=(j=="horizontal"?(r=="pick"?"horizontal_pick_target_m":r=="home"?"horizontal_home_m":"horizontal_target_m"):
                      (r=="pick"?"vertical_pick_target_m":r=="home"?"vertical_home_m":"down_target_m"));
  t.joint_value=num(rp,k,0);t.planning_time=num(p,"planning_time",15);t.vel=num(p,"joint_movej_velocity",.5);t.acc=num(p,"joint_movej_acceleration",.4);
  t.note="gantry production uses direct controller; this checks joint bounds only";add(out,t);
 }
 auto phys=w["robots"]["robot3"];
 for(const auto& step:task["robot3_steps"]){auto ref=val(step,"reference"),motion=val(step,"motion");
  auto t=make(name,val(hw,"robot3_group",val(phys,"group")),val(hw,"robot3_planning_link","robot3_end_effactor"),"robot3/"+val(step,"id"),"",motion,p);
  if(ref=="home"){t.home=true;t.frame="home";}else{
   t.frame=val(rp,ref=="pick_frame"?"robot3_pick_frame":"robot3_place_frame");t.xyz=vec(step,"xyz");t.rpy=vec(step,"rpy");
   t.local=val(step,"offset_frame")=="local";t.rpy_local=val(step,"rpy_frame","local")=="local";
  }
  t.planner=val(step,"planner");
  if(t.planner=="cartesian") {t.unsupported=true;t.note="production Cartesian interpolation/fraction not reproduced; IK only";}
  if(t.planner=="pilz"){t.vel=num(p,"pilz_velocity",.2);t.acc=num(p,"pilz_acceleration",.1);}
  else {t.vel=num(p,"joint_movej_velocity",.5);t.acc=num(p,"joint_movej_acceleration",.4);}
  add(out,t);
 }
}
Eigen::Isometry3d eigenPose(const geometry_msgs::msg::Pose& p){Eigen::Isometry3d m=Eigen::Isometry3d::Identity();m.translation()<<p.position.x,p.position.y,p.position.z;
 Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);m.linear()=q.normalized().toRotationMatrix();return m;}
geometry_msgs::msg::Pose targetPose(tf2_ros::Buffer& tf,const Target& t,const std::string& world){
 auto transform=tf.lookupTransform(world,t.frame,tf2::TimePointZero,tf2::durationFromSec(5));
 geometry_msgs::msg::Pose p;p.position.x=transform.transform.translation.x;p.position.y=transform.transform.translation.y;
 p.position.z=transform.transform.translation.z;p.orientation=transform.transform.rotation;
 Eigen::Vector3d d(t.xyz[0],t.xyz[1],t.xyz[2]);auto q=Eigen::Quaterniond(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z).normalized();
 if (t.local) {
  d = q * d;
 }
 p.position.x += d.x();
 p.position.y += d.y();
 p.position.z += d.z();
 Eigen::Quaterniond dq=Eigen::AngleAxisd(t.rpy[2],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(t.rpy[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(t.rpy[0],Eigen::Vector3d::UnitX());
 auto qt=(t.rpy_local?q*dq:dq*q).normalized();p.orientation.x=qt.x();p.orientation.y=qt.y();p.orientation.z=qt.z();p.orientation.w=qt.w();return p;
}
bool validate(const MGI::Plan& plan,const moveit::core::RobotState& start,std::string& message){
 auto& jt=plan.trajectory_.joint_trajectory;if(jt.joint_names.empty()||jt.points.empty()){message="empty trajectory";return false;}
 auto s=std::make_shared<moveit::core::RobotState>(start);double prev=-1;const auto model=start.getRobotModel();
 for(size_t i=0;i<jt.points.size();++i){auto& p=jt.points[i];if(p.positions.size()!=jt.joint_names.size()){message="position width mismatch";return false;}
  double time=static_cast<double>(p.time_from_start.sec)+1e-9*p.time_from_start.nanosec;
  if(!std::isfinite(time)||time<0||(i>0&&time<=prev)){message="invalid trajectory timestamps";return false;}prev=time;
  for(size_t j=0;j<jt.joint_names.size();++j){if(!std::isfinite(p.positions[j])){message="nonfinite joint position";return false;}
   s->setVariablePosition(jt.joint_names[j],p.positions[j]);
   const auto& b=model->getVariableBounds(jt.joint_names[j]);if(b.position_bounded_&&(p.positions[j]<b.min_position_-1e-5||p.positions[j]>b.max_position_+1e-5)){message="joint position bound violated";return false;}
   if(!p.velocities.empty()&&p.velocities.size()==jt.joint_names.size()&&b.velocity_bounded_&&std::abs(p.velocities[j])>b.max_velocity_+1e-4){message="velocity bound violated";return false;}
   if(!p.accelerations.empty()&&p.accelerations.size()==jt.joint_names.size()&&b.acceleration_bounded_&&std::abs(p.accelerations[j])>b.max_acceleration_+1e-3){message="acceleration bound violated";return false;}
  }
 }
 return true;
}
std::string path(const std::string& pkg,const std::string& rel,const std::string& override){return override.empty()?ament_index_cpp::get_package_share_directory(pkg)+"/"+rel:override;}
}

// Compact terminal report. Full error messages remain in the CSV.
struct AuditRow {
 std::string task, step, group, frame, motion, detail;
 bool ik=false, plan=false, trajectory=false;
 int points=0;
 double duration=0.0;
};
std::string clipped(const std::string& s, std::size_t width) {
 if (s.size() <= width) return s;
 return width > 3 ? s.substr(0, width - 3) + "..." : s.substr(0, width);
}
std::string failureReason(const AuditRow& r) {
 if (r.detail.find("NOT_TESTED:") != std::string::npos) return "NOT TESTED: " + r.detail.substr(r.detail.find("NOT_TESTED:") + 11);
 if (r.detail.find("SKIPPED:") != std::string::npos) return "SKIPPED: dependent step";
 if (r.detail.find("TF:") == 0) return "TF ERROR: " + r.frame;
 if (r.detail.find("ERROR:") == 0) return r.detail;
 if (!r.ik) return "IK FAILED";
 if (!r.plan && !r.trajectory && r.motion == "joint") return "BOUNDS ONLY; trajectory not tested";
 if (!r.plan) {
  const auto pos = r.detail.find("planner error=");
  return pos == std::string::npos ? "PLANNING FAILED" : "PLANNING FAILED (" + r.detail.substr(pos) + ")";
 }
 if (!r.trajectory) return r.detail.empty() ? "TRAJECTORY INVALID" : r.detail;
 return "OK";
}
void printReport(const std::vector<AuditRow>& rows, const std::string& csv_path,
                 int tf_fail, int not_tested) {
 constexpr int W1=19,W2=23,W3=12,W4=7,W5=5,W6=6,W7=6,W8=37;
 const std::string rule="+"+std::string(W1+2,'-')+"+"+std::string(W2+2,'-')+"+"+
   std::string(W3+2,'-')+"+"+std::string(W4+2,'-')+"+"+
   std::string(W5+2,'-')+"+"+std::string(W6+2,'-')+"+"+
   std::string(W7+2,'-')+"+"+std::string(W8+2,'-')+"+";
 auto cell=[](const std::string& x,int w){std::ostringstream o; o<<" "<<std::left<<std::setw(w)<<clipped(x,w)<<" ";return o.str();};
 int ik_count=0,plan_count=0,traj_count=0,fail_count=0;
 for(const auto& r:rows){ik_count+=r.ik;plan_count+=r.plan;traj_count+=r.trajectory;
  fail_count+=(failureReason(r)!="OK" && failureReason(r).find("NOT TESTED:")!=0 &&
               failureReason(r).find("BOUNDS ONLY")!=0 && failureReason(r).find("SKIPPED:")!=0);}
 std::cout<<"\n\n========== SA3 IK / MOTION-PLANNING AUDIT (NO EXECUTION) ==========\n";
 std::cout<<rule<<"\n|"<<cell("TASK",W1)<<"|"<<cell("STEP / ROBOT",W2)<<"|"
          <<cell("GROUP",W3)<<"|"<<cell("MOTION",W4)<<"|"
          <<cell("IK",W5)<<"|"<<cell("PLAN",W6)<<"|"
          <<cell("TRAJ",W7)<<"|"<<cell("RESULT / REASON",W8)<<"|\n"<<rule<<"\n";
 for(const auto& r:rows){
  const auto why=failureReason(r);
  const bool nt=why.find("NOT TESTED:")==0||why.find("BOUNDS ONLY")==0||why.find("SKIPPED:")==0;
  auto status=[&](bool passed){return passed?std::string("PASS"):(nt?std::string("N/A"):std::string("FAIL"));};
  std::cout<<"|"<<cell(r.task,W1)<<"|"<<cell(r.step,W2)<<"|"
           <<cell(r.group,W3)<<"|"<<cell(r.motion,W4)<<"|"
           <<cell(status(r.ik),W5)<<"|"<<cell(status(r.plan),W6)<<"|"
           <<cell(status(r.trajectory),W7)<<"|"<<cell(why,W8)<<"|\n";
 }
 std::cout<<rule<<"\n";
 std::cout<<"TOTAL: "<<rows.size()<<" targets  |  IK: "<<ik_count<<"/"<<rows.size()
          <<"  |  PLAN: "<<plan_count<<"/"<<rows.size()
          <<"  |  TRAJ: "<<traj_count<<"/"<<rows.size()
          <<"  |  TF errors: "<<tf_fail<<"  |  Not tested: "<<not_tested
          <<"  |  Failed rows: "<<fail_count<<"\n";
 std::cout<<"Full details: "<<csv_path<<"\n";
 std::cout<<"NOTE: planning only; attachments, collision-scene mutations and synchronized execution are not simulated.\n";
 std::cout.flush();
}

int main(int argc,char**argv){
 rclcpp::init(argc,argv);auto node=rclcpp::Node::make_shared("ik_reach_test",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
 auto str=[&](const std::string& k,const std::string& d){if(!node->has_parameter(k))node->declare_parameter<std::string>(k,d);return node->get_parameter(k).as_string();};
 auto boolean=[&](const std::string& k,bool d){if(!node->has_parameter(k))node->declare_parameter<bool>(k,d);return node->get_parameter(k).as_bool();};
 std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> ex;std::thread spin;int rc=0;
 try{
  auto wp=path(str("workcell_config_package","sa2_description"),str("workcell_config_file","config/workcell.yaml"),str("workcell_config_path",""));
  auto mp=path(str("motion_config_package","sa3_motion"),str("motion_config_file","config/ms_2_re.yaml"),str("motion_config_path",""));
  auto report_path=str("report_csv","/tmp/sa3_ik_reachability.csv");auto requested=str("motion_task","");
  bool all=boolean("include_inactive_tasks",false),chain=boolean("chain_plans",true),stop=boolean("stop_on_failure",false);
  auto w=YAML::LoadFile(wp)["workcell"],root=YAML::LoadFile(mp)["motion_sequence"];
  if(!w||!w["robots"]||!root||root["schema_version"].as<int>()!=2||!root["tasks"])throw std::runtime_error("Expected physical workcell and motion_sequence schema_version: 2 / tasks");
  std::set<std::string> selected;
  if(!requested.empty())selected.insert(requested);
  else if(all){for(const auto& kv:root["tasks"])selected.insert(kv.first.as<std::string>());}
  else for(const auto& stage:root["stages"])for(const auto& e:stage["executables"]){
   if(e.IsMap()&&e["parameters"]&&e["parameters"]["motion_task"])selected.insert(e["parameters"]["motion_task"].as<std::string>());
  }
  std::vector<Target> targets;
  for(const auto& name:selected){auto task=root["tasks"][name];if(!task)throw std::runtime_error("Unknown task: "+name);
   auto type=val(task,"type");if(type=="flange_pp")flange(targets,name,task,w["robots"]);
   else if(type=="shear_links_pp")shear(targets,name,task,w["robots"]);
   else if(type=="tying_rebars")tying(targets,name,task,w["robots"]);
   else if(type=="gantry_move")gantry(targets,name,task,w);
   else {Target t;t.task=name;t.id="task-level";t.unsupported=true;t.note="unsupported task type "+type;add(targets,t);}
  }
  if(targets.empty())throw std::runtime_error("No tasks selected. Set motion_task or include_inactive_tasks.");
  ex=std::make_shared<rclcpp::executors::MultiThreadedExecutor>();ex->add_node(node);spin=std::thread([&]{ex->spin();});
  tf2_ros::Buffer tf(node->get_clock());tf2_ros::TransformListener listener(tf,node,false);
  std::map<std::string,std::unique_ptr<MGI>> groups;std::map<std::string,moveit::core::RobotStatePtr> last;
  std::set<std::string> blocked;std::ofstream report(report_path);if(!report)throw std::runtime_error("Cannot write "+report_path);
  report<<"task,id,group,frame,motion,ik,plan,trajectory_valid,points,duration_s,detail\n";
  int okik=0,okplan=0,oktraj=0,tf_fail=0,skipped=0;
  std::vector<AuditRow> audit_rows;
  for(const auto& t:targets){bool ik=false,planned=false,valid=false;int points=0;double duration=0;std::string detail=t.note;
   try{
    if(t.unsupported){detail="NOT_TESTED: "+detail;++skipped;}
    else if(blocked.count(t.group)){detail="SKIPPED: previous failed stage in this group";++skipped;}
    else if(t.joint){ // Unlike robot MoveIt plans, gantry movements use direct controller trajectories.
     if(groups.empty()){} // no MoveGroup required for bounds-only audit
     auto model_group=t.group;
     if(!groups.count(model_group))groups[model_group]=std::make_unique<MGI>(node,model_group);
     auto state=groups[model_group]->getCurrentState(10);if(!state)throw std::runtime_error("Missing joint states");
     auto model=state->getRobotModel();auto jmg=model->getJointModelGroup(model_group);
     if(!jmg)throw std::runtime_error("Missing group: "+model_group);
     auto names=jmg->getVariableNames();if(std::find(names.begin(),names.end(),t.joint_name)==names.end()){
      detail="NOT_TESTED: gantry joint is outside MoveIt group; production direct controller";++skipped;
     }else{auto test=std::make_shared<moveit::core::RobotState>(*state);test->setVariablePosition(t.joint_name,t.joint_value);ik=test->satisfiesBounds(jmg);detail=ik?"gantry joint bounds PASS; controller trajectory NOT_TESTED":"gantry joint outside bounds";++skipped;}
    }else{
     if(!groups.count(t.group)){groups[t.group]=std::make_unique<MGI>(node,t.group);groups[t.group]->setPoseReferenceFrame(val(w,"world_frame","world"));}
     auto& mg=*groups[t.group];auto state=chain?last[t.group]:moveit::core::RobotStatePtr{};if(!state)state=mg.getCurrentState(10);
     if(!state)throw std::runtime_error("Current /joint_states unavailable");
     auto model=state->getRobotModel();auto jmg=model->getJointModelGroup(t.group);if(!jmg)throw std::runtime_error("Unknown group "+t.group);
     geometry_msgs::msg::Pose pose;
     if(t.home){ik=true;}else{
      pose=targetPose(tf,t,val(w,"world_frame","world"));if(!model->hasLinkModel(t.link))throw std::runtime_error("Unknown planning link "+t.link);
      auto test=std::make_shared<moveit::core::RobotState>(*state);
      ik=test->setFromIK(jmg,eigenPose(pose),t.link,t.ik_timeout)&&test->satisfiesBounds(jmg);
     }
     if(ik){mg.setStartState(*state);mg.setPlanningTime(t.planning_time);mg.setNumPlanningAttempts(std::max(1,t.attempts));
      mg.setMaxVelocityScalingFactor(std::clamp(t.vel,.001,1.));mg.setMaxAccelerationScalingFactor(std::clamp(t.acc,.001,1.));
      mg.clearPoseTargets();mg.clearPathConstraints();
      bool target_ok=false;
      if(t.home){mg.setPlanningPipelineId("ompl");mg.setPlannerId("RRTConnectkConfigDefault");target_ok=mg.setNamedTarget("home");}
      else{bool lin=t.motion=="moveL"||t.motion=="fallback";
       mg.setPlanningPipelineId(lin?"pilz_industrial_motion_planner":"ompl");mg.setPlannerId(lin?"LIN":"RRTConnectkConfigDefault");target_ok=mg.setPoseTarget(pose,t.link);}
      if(target_ok){MGI::Plan plan;moveit::core::MoveItErrorCode result;
       int attempts=std::max(1,t.retries);
       for(int retry=0;retry<attempts;++retry){result=mg.plan(plan);if(result==moveit::core::MoveItErrorCode::SUCCESS)break;}
       planned=result==moveit::core::MoveItErrorCode::SUCCESS;
       if(!planned && t.motion=="fallback"){
        mg.setPlanningPipelineId("ompl");mg.setPlannerId("RRTConnectkConfigDefault");
        result=mg.plan(plan);planned=result==moveit::core::MoveItErrorCode::SUCCESS;
        if(planned)detail+="; fallback OMPL (CIRC not attempted)";
       }
       if(!planned)detail+="; planner error="+std::to_string(result.val);
       else {valid=validate(plan,*state,detail);const auto& jt=plan.trajectory_.joint_trajectory;points=jt.points.size();
        if(!jt.points.empty())duration=rclcpp::Duration(jt.points.back().time_from_start).seconds();
        if(valid&&chain){auto next=std::make_shared<moveit::core::RobotState>(*state);
         if(!jt.points.empty()){const auto& end=jt.points.back();for(size_t k=0;k<jt.joint_names.size();++k)next->setVariablePosition(jt.joint_names[k],end.positions[k]);next->update();last[t.group]=next;}
        }
       }
      }else detail+="; target rejected";mg.clearPoseTargets();
     }else detail+="; IK unavailable/outside bounds";
    }
   }catch(const tf2::TransformException& e){++tf_fail;detail=std::string("TF: ")+e.what();}
    catch(const std::exception& e){detail=std::string("ERROR: ")+e.what();}
   if (ik) ++okik;
   if (planned) ++okplan;
   if (valid) ++oktraj;
   if(stop&&!t.group.empty()&&!valid&&!t.joint&&!t.unsupported)blocked.insert(t.group);
   audit_rows.push_back(AuditRow{t.task,t.id,t.group,t.frame,t.motion,detail,ik,planned,valid,points,duration});
   report<<csv(t.task)<<","<<csv(t.id)<<","<<csv(t.group)<<","<<csv(t.frame)<<","<<csv(t.motion)<<","<<ik<<","<<planned<<","<<valid<<","<<points<<","<<duration<<","<<csv(detail)<<"\n";
  }
  printReport(audit_rows, report_path, tf_fail, skipped);
 }catch(const std::exception& e){RCLCPP_ERROR(node->get_logger(),"SA3 audit failed: %s",e.what());rc=2;}
 if (ex) ex->cancel();
 if (spin.joinable()) spin.join();
 rclcpp::shutdown();
 return rc;
}
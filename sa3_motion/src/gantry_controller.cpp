// SA3 gantry bridge: ONLY gantry uses direct FollowJointTrajectory.
// Uses the existing /gantry_controller/follow_joint_trajectory action.
// Standard ROS 2 std_msgs/String topics (YAML payload), no custom .srv files.
// Simulation-only until verified. Robot1..4 are never commanded here.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <set>
#include <future>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <thread>
#include <vector>
#include <stdexcept>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

using Follow = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<Follow>;
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

class GantryJtcBridge {
public:
  explicit GantryJtcBridge(rclcpp::Node::SharedPtr n) : n_(std::move(n)) {
    const std::string wc = n_->declare_parameter<std::string>("workcell_config_path", "");
    const std::string path = wc.empty()
      ? ament_index_cpp::get_package_share_directory("sa2_description") + "/config/workcell.yaml" : wc;
    const YAML::Node w = YAML::LoadFile(path)["workcell"];
    if (!w || !w["gantry_robot"]) throw std::runtime_error("Missing workcell.gantry_robot");
    const YAML::Node g = w["gantry_robot"];
    xJoint_ = g["horizontal_joint"].as<std::string>();
    zJoint_ = g["vertical_joint"].as<std::string>();
    controller_ = g["controller_action"].as<std::string>();
    if (controller_ != "/gantry_controller/follow_joint_trajectory")
      throw std::runtime_error("Refusing unexpected gantry controller action: " + controller_);
    dry_ = n_->declare_parameter<bool>("dry_run", true);
    sim_ = n_->declare_parameter<bool>("simulation_confirmed", false);
    xMin_ = n_->declare_parameter<double>("horizontal_min_m", -10.6);
    xMax_ = n_->declare_parameter<double>("horizontal_max_m", 10.6);
    zMin_ = n_->declare_parameter<double>("vertical_min_m", 0.0);
    zMax_ = n_->declare_parameter<double>("vertical_max_m", 6.6);
    if (!std::isfinite(xMin_) || !std::isfinite(xMax_) || !std::isfinite(zMin_) || !std::isfinite(zMax_) ||
        xMin_ >= xMax_ || zMin_ >= zMax_) throw std::runtime_error("Invalid gantry joint bounds");
    client_ = rclcpp_action::create_client<Follow>(n_, controller_);
    const auto statusQos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local();
    statusPub_ = n_->create_publisher<std_msgs::msg::String>("/sa3/gantry/status", statusQos);
    jointSub_ = n_->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 20,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mu_);
        for (size_t i=0; i<msg->name.size() && i<msg->position.size(); ++i) {
          if (msg->name[i]==xJoint_) { xCurrent_=msg->position[i]; haveX_=true; }
          if (msg->name[i]==zJoint_) { zCurrent_=msg->position[i]; haveZ_=true; }
        }
        if (haveX_ && haveZ_) jointStamp_=Clock::now();
      });
    commandSub_ = n_->create_subscription<std_msgs::msg::String>("/sa3/gantry/command", 20,
      [this](const std_msgs::msg::String::SharedPtr msg) { onCommand(msg->data); });
    cancelSub_ = n_->create_subscription<std_msgs::msg::String>("/sa3/gantry/cancel", 20,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        GoalHandle::SharedPtr handle;
        {
          std::lock_guard<std::mutex> lk(mu_);
          if (!busy_ || active_ != msg->data) return;
          handle=handle_;
        }
        if (handle) {
          client_->async_cancel_goal(handle);
          RCLCPP_WARN(n_->get_logger(), "[GANTRY JTC] cancellation requested id=%s",msg->data.c_str());
        }
      });
    RCLCPP_INFO(n_->get_logger(), "[GANTRY JTC] using existing %s joints=[%s,%s]",
      controller_.c_str(),xJoint_.c_str(),zJoint_.c_str());
  }
  ~GantryJtcBridge() {
    stopping_.store(true);
    GoalHandle::SharedPtr handle;
    {std::lock_guard<std::mutex> lk(mu_);handle=handle_;}
    if (handle) client_->async_cancel_goal(handle);
    if (worker_.joinable()) worker_.join();
  }
private:
  void publish(const std::string& id, const std::string& state, const std::string& explanation) {
    YAML::Emitter e;
    e << YAML::BeginMap << YAML::Key << "request_id" << YAML::Value << id
      << YAML::Key << "status" << YAML::Value << state
      << YAML::Key << "message" << YAML::Value << explanation << YAML::EndMap;
    std_msgs::msg::String msg;msg.data=e.c_str();statusPub_->publish(msg);
    RCLCPP_INFO(n_->get_logger(), "[GANTRY JTC] id=%s status=%s %s",id.c_str(),state.c_str(),explanation.c_str());
  }
  void onCommand(const std::string& raw) {
    std::string id;
    try {
      const YAML::Node y=YAML::Load(raw);
      if (!y || !y.IsMap() || !y["request_id"]) throw std::runtime_error("Command missing request_id");
      id=y["request_id"].as<std::string>();
      if (id.empty()) throw std::runtime_error("Empty request_id");
      const double x=y["x"].as<double>(),z=y["z"].as<double>();
      const double speed=y["velocity"].as<double>(),acc=y["acceleration"].as<double>();
      if (!dry_ && !sim_) throw std::runtime_error("Gantry requires simulation_confirmed=true");
      if (!std::isfinite(x)||!std::isfinite(z)||!std::isfinite(speed)||!std::isfinite(acc)||
          speed<=0||acc<=0||speed>2.0||acc>5.0) throw std::runtime_error("Invalid gantry target/speed/acceleration");
      if (x<xMin_ || x>xMax_ || z<zMin_ || z>zMax_)
        throw std::runtime_error("Gantry target exceeds configured joint limits");
      double x0,z0;
      {
        std::lock_guard<std::mutex> lk(mu_);
        if (busy_) throw std::runtime_error("Gantry already executing id="+active_);
        if (used_.count(id)) throw std::runtime_error("Duplicate request_id");
        if (!dry_ && (!haveX_||!haveZ_||std::chrono::duration<double>(Clock::now()-jointStamp_).count()>2.0))
          throw std::runtime_error("Missing/stale gantry joint feedback");
        x0=xCurrent_;z0=zCurrent_;
        used_.insert(id);busy_=true;active_=id;
      }
      if (worker_.joinable()) worker_.join();
      worker_=std::thread([this,id,x,z,x0,z0,speed,acc] { runGoal(id,x,z,x0,z0,speed,acc); });
    } catch(const std::exception& e) {
      if (!id.empty()) publish(id,"ERROR",e.what());
      else RCLCPP_ERROR(n_->get_logger(),"[GANTRY JTC] rejected malformed command: %s",e.what());
    }
  }
  void runGoal(const std::string& id,double x,double z,double x0,double z0,double speed,double acc) {
    std::string state="DONE",explanation="Controller reported goal success";
    try {
      if (dry_) {
        explanation="DRY RUN: gantry was not commanded";
      } else {
        // A cubic interpolation with zero endpoint velocities has max speed
        // 1.5*distance/T and max acceleration 6*distance/T^2.
        const double maxD=std::max(std::abs(x-x0),std::abs(z-z0));
        const double duration=std::max({0.5,1.5*maxD/speed,std::sqrt(6.0*maxD/acc)});
        if (duration>120.0) throw std::runtime_error("Gantry trajectory exceeds 120 s limit");
        if (!client_->wait_for_action_server(8s)) throw std::runtime_error("Existing gantry FJT server unavailable");
        Follow::Goal goal;
        goal.trajectory.header.stamp=n_->now();
        goal.trajectory.joint_names={xJoint_,zJoint_};
        trajectory_msgs::msg::JointTrajectoryPoint p0,p1;
        p0.positions={x0,z0};p0.velocities={0.0,0.0};
        p1.positions={x,z};p1.velocities={0.0,0.0};
        p0.time_from_start.sec=0; p0.time_from_start.nanosec=100000000;
        {
        const auto ns = static_cast<int64_t>(
            std::llround((duration + 0.1) * 1e9));
        p1.time_from_start.sec = static_cast<int32_t>(ns / 1000000000LL);
        p1.time_from_start.nanosec =
            static_cast<uint32_t>(ns % 1000000000LL);
    }
        goal.trajectory.points={p0,p1};
        auto future=client_->async_send_goal(goal);
        {
          const auto started=Clock::now();
          while(future.wait_for(100ms)!=std::future_status::ready) {
            if(stopping_.load())throw std::runtime_error("Gantry bridge shutting down during goal acceptance");
            if(std::chrono::duration<double>(Clock::now()-started).count()>10.0)
              throw std::runtime_error("Gantry goal acceptance timeout (execution uncertain)");
          }
        }
        auto handle=future.get();
        if (!handle) throw std::runtime_error("Gantry controller rejected trajectory");
        {
          std::lock_guard<std::mutex> lk(mu_);handle_=handle;
        }
        publish(id,"RUNNING","Accepted by existing gantry_controller");
        auto result=client_->async_get_result(handle);
        {
          const auto started=Clock::now();
          while(result.wait_for(100ms)!=std::future_status::ready) {
            if(stopping_.load()) {
              client_->async_cancel_goal(handle);
              throw std::runtime_error("Gantry bridge shutting down: cancel requested; actual motion uncertain");
            }
            if(std::chrono::duration<double>(Clock::now()-started).count()>duration+30.0) {
              client_->async_cancel_goal(handle);
              throw std::runtime_error("Gantry result timeout: cancel requested, actual motion uncertain");
            }
          }
        }
        auto reply=result.get();
        if (reply.code!=rclcpp_action::ResultCode::SUCCEEDED || !reply.result ||
            reply.result->error_code!=Follow::Result::SUCCESSFUL)
          throw std::runtime_error("Gantry controller returned failure/cancel; reconcile joint state");
        // Verify final /joint_states independently after controller-reported success.
        bool reached=false;
        const auto verifyStart=Clock::now();
        while(std::chrono::duration<double>(Clock::now()-verifyStart).count()<4.0) {
          {
            std::lock_guard<std::mutex> lk(mu_);
            const double freshness=std::chrono::duration<double>(Clock::now()-jointStamp_).count();
            if(haveX_&&haveZ_&&freshness<1.0&&std::abs(xCurrent_-x)<=0.01&&std::abs(zCurrent_-z)<=0.01) {
              reached=true;break;
            }
          }
          std::this_thread::sleep_for(100ms);
        }
        if(!reached)throw std::runtime_error("Gantry controller succeeded but final /joint_states disagreed (>0.01m); reconcile actual motion");
      }
    } catch(const std::exception& e) {state="ERROR";explanation=e.what();}
    {
      std::lock_guard<std::mutex> lk(mu_);
      handle_.reset();busy_=false;active_.clear();
    }
    publish(id,state,explanation);
  }
  rclcpp::Node::SharedPtr n_;
  rclcpp_action::Client<Follow>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr commandSub_,cancelSub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSub_;
  std::string xJoint_,zJoint_,controller_,active_;
  std::mutex mu_;std::thread worker_;GoalHandle::SharedPtr handle_;
  std::set<std::string> used_;
  std::atomic<bool> stopping_{false};
  bool busy_{false},dry_{true},sim_{false},haveX_{false},haveZ_{false};
  double xCurrent_{0},zCurrent_{0},xMin_{-10.6},xMax_{10.6},zMin_{0},zMax_{6.6};
  Clock::time_point jointStamp_{Clock::now()-10s};
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("sa3_gantry_jtc_server");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),4);
  executor.add_node(node);
  try {auto server=std::make_unique<GantryJtcBridge>(node);executor.spin();server.reset();}
  catch(const std::exception& e){RCLCPP_FATAL(node->get_logger(),"Gantry bridge failure: %s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
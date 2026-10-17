#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "sa3_motion/planning_scene_objects.hpp"

using namespace std::chrono_literals;

class RebarPlanningSceneNode : public rclcpp::Node
{
public:
  RebarPlanningSceneNode()
  : Node("rebar_planning_scene_node")
  {
    // Wait until MoveIt's planning-scene services and parent TFs are ready.
    startup_timer_ = create_wall_timer(
      1s,
      std::bind(&RebarPlanningSceneNode::tryAddRebars, this));
    RCLCPP_INFO(
      get_logger(),
      "Rebar planning-scene node started; waiting to load rebars from workcell.yaml.");
  }

private:
  void tryAddRebars()
  {
    if (finished_) {
      return;
    }

    if (!planning_scene_objects_) {
      planning_scene_objects_ =
        std::make_shared<PlanningSceneObjects>(shared_from_this());
    }

    RCLCPP_INFO(
      get_logger(),
      "Attempting to add YAML-defined rebars to the MoveIt planning scene.");

    if (planning_scene_objects_->addExtraRebars() && planning_scene_objects_->addExtraGrippers()) {
      finished_ = true;
      startup_timer_->cancel();
      RCLCPP_INFO(
        get_logger(),
        "Rebar planning-scene setup completed successfully.");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Could not add rebars yet; will retry in 1 second. Check MoveIt, TF parent frames, YAML and mesh paths.");
    }
  }

  std::shared_ptr<PlanningSceneObjects> planning_scene_objects_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  bool finished_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RebarPlanningSceneNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

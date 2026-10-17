#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

using namespace std::chrono_literals;

class WorkspaceScanner
{
public:
  WorkspaceScanner(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
  }

  bool run()
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "============================================================");

    RCLCPP_INFO(
      node_->get_logger(),
      " Robot 1 Fixed-Orientation Workspace Scanner");

    RCLCPP_INFO(
      node_->get_logger(),
      "============================================================");

    // ------------------------------------------------------------
    // Parameters
    // ------------------------------------------------------------

    const double resolution =
      node_->declare_parameter<double>("resolution", 0.01);

    const double x_min =
      node_->declare_parameter<double>("x_min", -0.30);

    const double x_max =
      node_->declare_parameter<double>("x_max", 0.30);

    const double y_min =
      node_->declare_parameter<double>("y_min", -0.30);

    const double y_max =
      node_->declare_parameter<double>("y_max", 0.30);

    const double z_min =
      node_->declare_parameter<double>("z_min", -0.30);

    const double z_max =
      node_->declare_parameter<double>("z_max", 0.30);

    const double ik_timeout =
      node_->declare_parameter<double>("ik_timeout", 0.002);

    RCLCPP_INFO(
      node_->get_logger(),
      "Resolution: %.3f m", resolution);

    RCLCPP_INFO(
      node_->get_logger(),
      "Local search volume:");

    RCLCPP_INFO(
      node_->get_logger(),
      "  X: %.3f -> %.3f", x_min, x_max);

    RCLCPP_INFO(
      node_->get_logger(),
      "  Y: %.3f -> %.3f", y_min, y_max);

    RCLCPP_INFO(
      node_->get_logger(),
      "  Z: %.3f -> %.3f", z_min, z_max);

    // ------------------------------------------------------------
    // MoveIt
    // ------------------------------------------------------------

    RCLCPP_INFO(
      node_->get_logger(),
      "Waiting for MoveIt...");

    moveit::planning_interface::MoveGroupInterface move_group(
      node_,
      "robot1_arm");

    move_group.setPoseReferenceFrame("world");
    move_group.setEndEffectorLink("robot1_end_effactor");

    // ------------------------------------------------------------
    // Current state
    // ------------------------------------------------------------

    if (!move_group.getCurrentState(10.0))
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Could not obtain current robot state.");

      return false;
    }

    auto current_state = move_group.getCurrentState(10.0);

    if (!current_state)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Current robot state is null.");

      return false;
    }

    // ------------------------------------------------------------
    // Current EE pose
    // ------------------------------------------------------------

    const geometry_msgs::msg::Pose current_pose =
      move_group.getCurrentPose("robot1_end_effactor").pose;

    RCLCPP_INFO(
      node_->get_logger(),
      "Current EE position:");

    RCLCPP_INFO(
      node_->get_logger(),
      "  X = %.4f",
      current_pose.position.x);

    RCLCPP_INFO(
      node_->get_logger(),
      "  Y = %.4f",
      current_pose.position.y);

    RCLCPP_INFO(
      node_->get_logger(),
      "  Z = %.4f",
      current_pose.position.z);

    RCLCPP_INFO(
      node_->get_logger(),
      "Current EE orientation:");

    RCLCPP_INFO(
      node_->get_logger(),
      "  qx = %.5f",
      current_pose.orientation.x);

    RCLCPP_INFO(
      node_->get_logger(),
      "  qy = %.5f",
      current_pose.orientation.y);

    RCLCPP_INFO(
      node_->get_logger(),
      "  qz = %.5f",
      current_pose.orientation.z);

    RCLCPP_INFO(
      node_->get_logger(),
      "  qw = %.5f",
      current_pose.orientation.w);

    // ------------------------------------------------------------
    // IMPORTANT:
    //
    // Search limits are RELATIVE to the current EE position.
    // ------------------------------------------------------------

    const double scan_x_min =
      current_pose.position.x + x_min;

    const double scan_x_max =
      current_pose.position.x + x_max;

    const double scan_y_min =
      current_pose.position.y + y_min;

    const double scan_y_max =
      current_pose.position.y + y_max;

    const double scan_z_min =
      current_pose.position.z + z_min;

    const double scan_z_max =
      current_pose.position.z + z_max;

    const long nx =
      static_cast<long>(
        std::floor((scan_x_max - scan_x_min) / resolution)) + 1;

    const long ny =
      static_cast<long>(
        std::floor((scan_y_max - scan_y_min) / resolution)) + 1;

    const long nz =
      static_cast<long>(
        std::floor((scan_z_max - scan_z_min) / resolution)) + 1;

    const long long total_points =
      static_cast<long long>(nx) *
      static_cast<long long>(ny) *
      static_cast<long long>(nz);

    RCLCPP_INFO(
      node_->get_logger(),
      "Number of XYZ samples: %lld",
      total_points);

    RCLCPP_WARN(
      node_->get_logger(),
      "This will perform approximately %lld IK checks.",
      total_points);

    // ------------------------------------------------------------
    // Robot model / JointModelGroup
    // ------------------------------------------------------------

    const auto robot_model =
      current_state->getRobotModel();

    const auto * joint_model_group =
      robot_model->getJointModelGroup("robot1_arm");

    if (!joint_model_group)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Could not find JointModelGroup 'robot1_arm'.");

      return false;
    }

    // ------------------------------------------------------------
    // RViz Marker
    // ------------------------------------------------------------

    marker_pub_ =
      node_->create_publisher<visualization_msgs::msg::Marker>(
        "/robot1_workspace",
        rclcpp::QoS(1).transient_local());

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = node_->now();

    marker.ns = "robot1_workspace";
    marker.id = 0;

    marker.type =
      visualization_msgs::msg::Marker::POINTS;

    marker.action =
      visualization_msgs::msg::Marker::ADD;

    // Each point is 1 cm visually.
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.75;

    // ------------------------------------------------------------
    // Scan
    // ------------------------------------------------------------

    long long tested = 0;
    long long reachable = 0;

    const auto start_time =
      std::chrono::steady_clock::now();

    RCLCPP_INFO(
      node_->get_logger(),
      "Starting workspace scan...");

    for (long ix = 0; ix < nx; ++ix)
    {
      const double x =
        scan_x_min +
        static_cast<double>(ix) * resolution;

      for (long iy = 0; iy < ny; ++iy)
      {
        const double y =
          scan_y_min +
          static_cast<double>(iy) * resolution;

        for (long iz = 0; iz < nz; ++iz)
        {
          const double z =
            scan_z_min +
            static_cast<double>(iz) * resolution;

          ++tested;

          // ------------------------------------------------------
          // Construct target pose.
          //
          // POSITION changes.
          //
          // ORIENTATION DOES NOT CHANGE.
          // ------------------------------------------------------

          geometry_msgs::msg::Pose target;

          target.position.x = x;
          target.position.y = y;
          target.position.z = z;

          target.orientation =
            current_pose.orientation;

          // ------------------------------------------------------
          // Start every IK test from current robot state.
          //
          // This means:
          //
          // target XYZ
          // +
          // current EE orientation
          //
          // -> ask IK if robot can achieve it.
          // ------------------------------------------------------

          auto test_state =
            *current_state;

          const bool ik_success =
            test_state.setFromIK(
              joint_model_group,
              target,
              "robot1_end_effactor",
              ik_timeout);

          if (ik_success)
          {
            ++reachable;

            geometry_msgs::msg::Point point;

            point.x = x;
            point.y = y;
            point.z = z;

            marker.points.push_back(point);
          }
        }
      }

      // ----------------------------------------------------------
      // Progress
      // ----------------------------------------------------------

      if (ix % 5 == 0 || ix == nx - 1)
      {
        const double progress =
          100.0 *
          static_cast<double>(ix + 1) /
          static_cast<double>(nx);

        RCLCPP_INFO(
          node_->get_logger(),
          "Progress: %.1f%% | tested=%lld | reachable=%lld",
          progress,
          tested,
          reachable);

        // Publish progressively so RViz starts showing the
        // workspace before the entire scan finishes.
        marker.header.stamp = node_->now();
        marker_pub_->publish(marker);
      }
    }

    // ------------------------------------------------------------
    // Final result
    // ------------------------------------------------------------

    marker.header.stamp = node_->now();
    marker_pub_->publish(marker);

    const double elapsed =
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time).count();

    RCLCPP_INFO(
      node_->get_logger(),
      "============================================================");

    RCLCPP_INFO(
      node_->get_logger(),
      " WORKSPACE SCAN COMPLETE");

    RCLCPP_INFO(
      node_->get_logger(),
      "============================================================");

    RCLCPP_INFO(
      node_->get_logger(),
      "Total points tested : %lld",
      tested);

    RCLCPP_INFO(
      node_->get_logger(),
      "Reachable points    : %lld",
      reachable);

    RCLCPP_INFO(
      node_->get_logger(),
      "Unreachable points  : %lld",
      tested - reachable);

    RCLCPP_INFO(
      node_->get_logger(),
      "Resolution          : %.3f m",
      resolution);

    RCLCPP_INFO(
      node_->get_logger(),
      "Time                : %.2f s",
      elapsed);

    if (tested > 0)
    {
      RCLCPP_INFO(
        node_->get_logger(),
        "Reachability        : %.2f%%",
        100.0 *
        static_cast<double>(reachable) /
        static_cast<double>(tested));
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "RViz topic          : /robot1_workspace");

    RCLCPP_INFO(
      node_->get_logger(),
      "============================================================");

    return true;
  }

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<
    visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    rclcpp::Node::make_shared(
      "robot1_workspace_scanner");

  WorkspaceScanner scanner(node);

  const bool success =
    scanner.run();

  rclcpp::shutdown();

  return success ? 0 : 1;
}
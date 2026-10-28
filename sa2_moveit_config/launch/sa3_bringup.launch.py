"""SA3 full bringup with per-robot FANUC/mock hardware selection."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_pkg = get_package_share_directory("sa2_description")
    moveit_pkg = get_package_share_directory("sa2_moveit_config")

    # Expose the same arguments provided by sa3_mixed_hardware.launch.py.
    declarations = [
        DeclareLaunchArgument(
            "hardware_mode", default_value="mock",
            choices=["mock", "fanuc", "gazebo"],
            description="Default hardware mode for all four robots",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="false", choices=["true", "false"],
            description="Launch description RViz (normally keep false when using MoveIt RViz)",
        ),
    ]
    forwarded_args = {
        "hardware_mode": LaunchConfiguration("hardware_mode"),
        "use_rviz": LaunchConfiguration("use_rviz"),
    }

    for i in range(1, 5):
        mode_key = f"robot{i}_hardware"
        declarations.append(
            DeclareLaunchArgument(
                mode_key,
                default_value=LaunchConfiguration("hardware_mode"),
                choices=["mock", "fanuc", "gazebo"],
            )
        )
        forwarded_args[mode_key] = LaunchConfiguration(mode_key)
        for suffix, default in (
            ("ip", "192.168.2.3"),
            ("rmi_port", "16001"),
            ("stream_motion_port", "60015"),
        ):
            arg = f"robot{i}_{suffix}"
            declarations.append(DeclareLaunchArgument(arg, default_value=default))
            forwarded_args[arg] = LaunchConfiguration(arg)

    # Hardware + joint controllers + robot and environment state publishers.
    ros2_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, "launch", "sa3_mixed_hardware.launch.py")
        ),
        launch_arguments=forwarded_args.items(),
    )

    # Main MoveIt planning process (unchanged).
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, "launch", "move_group.launch.py")
        )
    )

    workcell_visualizer = Node(
        package="sa2_description",
        executable="workcell_visualizer.py",
        output="screen",
    )
    rebar_visualizer = Node(
        package="sa2_description", executable="rebar_visualizer.py", output="screen"
    )
    gripper_visualizer = Node(
        package="sa2_description", executable="gripper_visualizer.py", output="screen"
    )
    environment_collision = Node(
        package="sa2_description", executable="environment_collision", output="screen"
    )
    planning_scene = Node(
        package="sa3_motion", executable="rebar_planning_scene_node", output="screen"
    )

    rviz_config = os.path.join(moveit_pkg, "config", "moveit.rviz")
    moveit_rviz = Node(
        package="rviz2", executable="rviz2", name="moveit_rviz",
        output="screen", arguments=["-d", rviz_config],
    )

    moveit_config = MoveItConfigsBuilder(
        "sa3_robots", package_name="sa2_moveit_config"
    ).to_moveit_configs()
    ik_reach_test = Node(
        package="sa3_motion", executable="ik_reach_test", output="screen",
        parameters=[moveit_config.to_dict()],
    )
    ik_reach_test_delayed = TimerAction(period=20.0, actions=[ik_reach_test])

    return LaunchDescription(
        declarations + [
            ros2_controller_launch,
            move_group_launch,
            workcell_visualizer,
            rebar_visualizer,
            gripper_visualizer,
            environment_collision,
            planning_scene,
            # moveit_rviz,            # Optional: uncomment for MoveIt RViz.
            # ik_reach_test_delayed,  # Optional: uncomment for reachability test.
        ]
    )
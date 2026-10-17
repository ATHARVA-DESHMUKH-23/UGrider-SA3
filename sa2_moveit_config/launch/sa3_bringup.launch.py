import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from launch.actions import TimerAction
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # =========================================================
    # PACKAGE DIRECTORIES
    # =========================================================

    description_pkg = get_package_share_directory(
        "sa2_description"
    )

    moveit_pkg = get_package_share_directory(
        "sa2_moveit_config"
    )

    # =========================================================
    # 1. ROS2 CONTROLLER BRINGUP
    #
    # Equivalent to:
    #
    # ros2 launch sa2_description ros2_controller.launch.py
    # =========================================================

    ros2_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                description_pkg,
                "launch",
                "ros2_controller.launch.py"
            )
        )
    )

    # =========================================================
    # 2. MOVE GROUP
    #
    # Equivalent to:
    #
    # ros2 launch sa2_moveit_config move_group.launch.py
    # =========================================================

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_pkg,
                "launch",
                "move_group.launch.py"
            )
        )
    )

    # =========================================================
    # 3. WORKCELL VISUALIZER abd rebars visualizer
    # =========================================================

    workcell_visualizer = Node(
        package="sa2_description",
        executable="workcell_visualizer.py",
        output="screen",
    )

    rebar_visualizer = Node(
        package="sa2_description",
        executable="rebar_visualizer.py",
        output="screen",
    )

    gripper_visualizer = Node(
            package="sa2_description",
            executable="gripper_visualizer.py",
            output="screen",
        )
    movable_rebar_tf = Node(
            package="sa2_description",
            executable="movable_rebar_tf.py",
            output="screen",
        )
    
    movable_rebar_marker = Node(
            package="sa2_description",
            executable="movable_rebar_marker.py",
            output="screen",
        )

    # =========================================================
    # 4. ENVIRONMENT COLLISION
    #
    # Equivalent to:
    #
    # ros2 run sa2_description environment_collision
    # =========================================================

    environment_collision = Node(
        package="sa2_description",
        executable="environment_collision",
        output="screen",
    )

    planning_scence = Node(
            package="sa3_motion",
            executable="rebar_planning_scene_node",
            output="screen",
        )

    # =========================================================
    # 5. MOVEIT RVIZ
    #
    # Equivalent to:
    #
    # rviz2 -d
    # /home/atharva/ros_ws/sa2/src/sa2_moveit_config/config/moveit.rviz
    # =========================================================

    rviz_config = os.path.join(
        moveit_pkg,
        "config",
        "moveit.rviz"
    )

    moveit_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_rviz",
        output="screen",
        arguments=[
            "-d",
            rviz_config
        ],
    )

    # =========================================================
    # 6. IK / OMPL REACH TEST (robot1)
    #
    # Equivalent to:
    #
    # ros2 run sa3_motion ik_reach_test   (with moveit params)
    # =========================================================

    moveit_config = MoveItConfigsBuilder(
        "sa3_robots", package_name="sa2_moveit_config"
    ).to_moveit_configs()

    ik_reach_test = Node(
        package="sa3_motion",
        executable="ik_reach_test",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # start after move_group, visualizer and collision scene are up
    ik_reach_test_delayed = TimerAction(
        period=20.0,
        actions=[ik_reach_test],
    )

    # =========================================================
    # LAUNCH EVERYTHING
    # =========================================================

    return LaunchDescription([

        # 1. Robot + ros2_control + controllers
        ros2_controller_launch,

        # 2. MoveIt move_group
        move_group_launch,

        # 3. Workcell & rebar visualizer
        workcell_visualizer,
        rebar_visualizer,
        gripper_visualizer,
        movable_rebar_tf,
        movable_rebar_marker,

        # 4. Environment collision
        environment_collision,
        planning_scence,

        # # 5. MoveIt RViz
        # moveit_rviz,

        # 6. IK + OMPL reachability test (plan only)
        # ik_reach_test_delayed,

    ])
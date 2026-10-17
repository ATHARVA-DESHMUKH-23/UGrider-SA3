import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    pkg = get_package_share_directory(
        "sa2_description"
    )

    # =========================================================
    # ROBOT XACRO
    # =========================================================

    robot_xacro = os.path.join(
        pkg,
        "urdf",
        "robot.urdf.xacro"
    )

    robot_description = xacro.process_file(
        robot_xacro
    ).toxml()

    # =========================================================
    # ENVIRONMENT XACRO
    # =========================================================

    env_xacro = os.path.join(
        pkg,
        "urdf",
        "sa3_env.urdf.xacro"
    )

    env_description = xacro.process_file(
        env_xacro
    ).toxml()

    # =========================================================
    # RVIZ
    # =========================================================

    rviz = os.path.join(
        pkg,
        "rviz",
        "sa2_description.rviz"
    )

    return LaunchDescription([

        # -----------------------------------------------------
        # ROBOT STATE PUBLISHER
        # -----------------------------------------------------

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description
                }
            ],
        ),

        # -----------------------------------------------------
        # ROBOT JOINT STATES
        # -----------------------------------------------------

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),

        # -----------------------------------------------------
        # ENVIRONMENT
        # -----------------------------------------------------

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="environment_state_publisher",
            namespace="environment",
            output="screen",
            parameters=[
                {
                    "robot_description": env_description
                }
            ],
        ),

        # -----------------------------------------------------
        # RVIZ
        # -----------------------------------------------------

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz],
        ),
    ])
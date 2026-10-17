
import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def vector_to_string(values):
    """Convert a YAML XYZ/RPY list into a space-separated Xacro argument."""
    if len(values) != 3:
        raise ValueError(f"Expected 3 values, received: {values}")
    return " ".join(str(value) for value in values)


def generate_launch_description():

    pkg = get_package_share_directory("sa2_description")

    # =========================================================
    # LAUNCH ARGUMENTS
    # =========================================================

    hardware_mode = LaunchConfiguration("hardware_mode")

    hardware_mode_arg = DeclareLaunchArgument(
        "hardware_mode",
        default_value="mock",
        description="Hardware mode: mock, gazebo, or fanuc"
    )

    # =========================================================
    # LOAD ENVIRONMENT COORDINATES FROM YAML
    # =========================================================

    workcell_yaml = os.path.join(
        pkg,
        "config",
        "workcell.yaml"
    )

    with open(workcell_yaml, "r") as file:
        workcell_config = yaml.safe_load(file)

    environment = workcell_config["environment"]

    # =========================================================
    # ROBOT XACRO
    # =========================================================

    
    # =========================================================
    # ROBOT XACRO - PEDESTAL POSITIONS FROM YAML
    # =========================================================

    robot_xacro = os.path.join(
        pkg,
        "urdf",
        "robot.urdf.xacro"
    )

    # Read pedestal configurations
    robot_pedestals = workcell_config["robot_pedestals"]

    robot_mappings = {
        "hardware_mode": "mock"
    }

    pedestal_names = [
        "robot1_pedestal",
        "robot2_pedestal",
        "robot3_pedestal",
        "robot4_pedestal",
    ]

    for name in pedestal_names:
        if name not in robot_pedestals:
            raise KeyError(
                f"Missing pedestal '{name}' in {workcell_yaml}"
            )

        pedestal = robot_pedestals[name]

        robot_mappings[f"{name}_xyz"] = vector_to_string(
            pedestal["xyz"]
        )

        robot_mappings[f"{name}_rpy"] = vector_to_string(
            pedestal["rpy"]
        )

    robot_description = xacro.process_file(
        robot_xacro,
        mappings=robot_mappings
    ).toxml()

    print("\nRobot pedestal positions loaded from YAML:")
    for name in pedestal_names:
        print(
            f"  {name}: "
            f"xyz={robot_pedestals[name]['xyz']}, "
            f"rpy={robot_pedestals[name]['rpy']}"
        )


    # =========================================================
    # ENVIRONMENT XACRO
    # =========================================================

    env_xacro = os.path.join(
        pkg,
        "urdf",
        "sa3_env.urdf.xacro"
    )

    # Build Xacro mappings from the YAML environment section.
    env_mappings = {}

    environment_objects = [
        "sa3_main_jig",
        "inventory_3a_4a",
        "inventory_3a_4a_2",
        "jig_clamp",
        "web_ring",
        "web_ring_2",
    ]

    for name in environment_objects:
        if name not in environment:
            raise KeyError(
                f"Missing environment object '{name}' "
                f"in {workcell_yaml}"
            )

        obj = environment[name]

        env_mappings[f"{name}_xyz"] = vector_to_string(
            obj["xyz"]
        )

        env_mappings[f"{name}_rpy"] = vector_to_string(
            obj["rpy"]
        )

    env_description = xacro.process_file(
        env_xacro,
        mappings=env_mappings
    ).toxml()

    print("\nEnvironment Xacro parameters loaded from YAML:")
    for name in environment_objects:
        print(
            f"  {name}: "
            f"xyz={environment[name]['xyz']}, "
            f"rpy={environment[name]['rpy']}"
        )

    # =========================================================
    # ROS2 CONTROLLERS YAML
    # =========================================================

    controllers_yaml = os.path.join(
        pkg,
        "config",
        "ros2_controller.yaml"
    )

    # =========================================================
    # RVIZ CONFIGURATION
    # =========================================================

    rviz = os.path.join(
        pkg,
        "rviz",
        "sa2_description.rviz"
    )

    # =========================================================
    # ROBOT STATE PUBLISHER
    # =========================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description
            }
        ],
    )

    # =========================================================
    # ENVIRONMENT STATE PUBLISHER
    # =========================================================

    environment_state_publisher = Node(
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
    )

    # =========================================================
    # ROS2 CONTROL NODE
    # =========================================================

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description
            },
            controllers_yaml
        ],
    )

    # =========================================================
    # JOINT STATE BROADCASTER
    # =========================================================

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # =========================================================
    # ROBOT CONTROLLERS
    # =========================================================

    robot1_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robot1_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    robot2_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robot2_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    robot3_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robot3_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    robot4_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robot4_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # =========================================================
    # RVIZ2
    # =========================================================

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz
        ],
    )

    # =========================================================
    # LAUNCH
    # =========================================================

    return LaunchDescription([
        hardware_mode_arg,

        robot_state_publisher,
        environment_state_publisher,

        ros2_control_node,

        joint_state_broadcaster,
        robot1_controller,
        robot2_controller,
        robot3_controller,
        robot4_controller,

        # rviz_node,
    ])

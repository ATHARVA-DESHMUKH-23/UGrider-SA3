
import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xml.etree.ElementTree as ET
from xml.dom import minidom


def vector_to_string(values):
    """Convert a YAML XYZ/RPY/scale list into a space-separated Xacro argument."""
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
        description="Hardware mode: mock, gazebo, or fanuc",
    )

    # =========================================================
    # LOAD WORKCELL YAML
    # =========================================================

    workcell_yaml = os.path.join(pkg, "config", "workcell.yaml")

    with open(workcell_yaml, "r") as file:
        workcell_config = yaml.safe_load(file)

    environment = workcell_config["environment"]
    robot_pedestals = workcell_config["robot_pedestals"]

    # =========================================================
    # ROBOT XACRO
    # =========================================================

    robot_xacro = os.path.join(pkg, "urdf", "robot.urdf.xacro")

    gantry = environment.get("gantry")

    if gantry is None:
        raise KeyError(
            f"Missing 'environment.gantry' configuration in {workcell_yaml}"
        )

    for key in ("xyz", "rpy", "scale"):
        if key not in gantry:
            raise KeyError(
                f"Missing 'environment.gantry.{key}' in {workcell_yaml}"
            )

    # Pass the selected hardware mode to Xacro instead of hardcoding mock.
    robot_mappings = {
    "hardware_mode": "mock",
}

    # ---------------------------------------------------------
    # PEDESTAL MAPPINGS
    # ---------------------------------------------------------

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

        robot_mappings[f"{name}_xyz"] = vector_to_string(pedestal["xyz"])
        robot_mappings[f"{name}_rpy"] = vector_to_string(pedestal["rpy"])

    # ---------------------------------------------------------
    # GANTRY MAPPINGS
    # ---------------------------------------------------------

    robot_mappings["gantry_xyz"] = vector_to_string(gantry["xyz"])
    robot_mappings["gantry_rpy"] = vector_to_string(gantry["rpy"])
    robot_mappings["gantry_scale"] = vector_to_string(gantry["scale"])

    robot_description = xacro.process_file(
        robot_xacro,
        mappings=robot_mappings,
    ).toxml()

    print("\nRobot pedestal positions loaded from YAML:")
    for name in pedestal_names:
        print(
            f"  {name}: "
            f"xyz={robot_pedestals[name]['xyz']}, "
            f"rpy={robot_pedestals[name]['rpy']}"
        )

    print("\nGantry configuration loaded from YAML:")
    print(f"  xyz={gantry['xyz']}")
    print(f"  rpy={gantry['rpy']}")
    print(f"  scale={gantry['scale']}")

    # =========================================================
    # DYNAMIC ENVIRONMENT XACRO FROM YAML
    # =========================================================

    env_xacro = os.path.join(
        pkg, "urdf", "sa3_env.urdf.xacro"
    )

    # Load reusable environment macro
    env_doc = xacro.parse(open(env_xacro, "r"))

    # Access root <robot>
    root = env_doc.documentElement

    # Xacro namespace
    XACRO_NS = "http://www.ros.org/wiki/xacro"

    # All environment objects except gantry
    environment_objects = {
        name: obj
        for name, obj in environment.items()
        if name != "gantry"
    }

    # Validate parent references before generating URDF
    for name, obj in environment_objects.items():

        if not isinstance(obj, dict):
            raise ValueError(
                f"Invalid environment object '{name}'"
            )

        parent = obj.get("parent", "world")

        if parent == name:
            raise ValueError(
                f"Environment object '{name}' cannot parent itself"
            )

        if parent != "world" and parent not in environment_objects:
            raise ValueError(
                f"Unknown parent '{parent}' for '{name}'"
            )

        for field in ("xyz", "rpy", "mesh"):
            if field not in obj:
                raise KeyError(
                    f"Missing environment.{name}.{field}"
                )

        if not isinstance(obj["mesh"], str) or not obj["mesh"]:
            raise ValueError(f"Invalid mesh for '{name}'")

        # Create a Xacro macro invocation
        element = env_doc.createElementNS(
            XACRO_NS,
            "xacro:environment_object"
        )

        element.setAttribute("name", name)
        element.setAttribute("parent", parent)

        element.setAttribute(
            "xyz", vector_to_string(obj["xyz"])
        )

        element.setAttribute(
            "rpy", vector_to_string(obj["rpy"])
        )

        element.setAttribute(
            "mesh", obj["mesh"]
        )

        element.setAttribute(
            "scale",
            vector_to_string(
                obj.get("scale", [0.001, 0.001, 0.001])
            )
        )

        root.appendChild(element)

    # Expand all Xacro macros
    xacro.process_doc(env_doc)

    # Final environment URDF
    env_description = env_doc.toxml()

    print("\nEnvironment objects loaded from YAML:")

    for name, obj in environment_objects.items():
        print(
            f"  {name}: "
            f"parent={obj.get('parent', 'world')}, "
            f"xyz={obj['xyz']}, "
            f"rpy={obj['rpy']}"
        )

    # =========================================================
    # ROS 2 CONTROLLERS YAML
    # =========================================================

    controllers_yaml = os.path.join(
        pkg,
        "config",
        "ros2_controller.yaml",
    )

    # =========================================================
    # RVIZ CONFIGURATION
    # =========================================================

    rviz = os.path.join(pkg, "rviz", "sa2_description.rviz")

    # =========================================================
    # ROBOT STATE PUBLISHER
    # =========================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
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
            {"robot_description": env_description},
        ],
    )

    # =========================================================
    # ROS 2 CONTROL NODE
    # =========================================================

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml,
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
    # ROBOT 1 CONTROLLER
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

    # =========================================================
    # ROBOT 2 CONTROLLER
    # =========================================================

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

    # =========================================================
    # ROBOT 3 CONTROLLER
    # =========================================================

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

    # =========================================================
    # ROBOT 4 CONTROLLER
    # =========================================================

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
    # GANTRY CONTROLLER
    # =========================================================

    gantry_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gantry_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # =========================================================
    # RVIZ 2
    # =========================================================

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz],
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
        gantry_controller,

        # Uncomment to launch RViz automatically.
        rviz_node,
    ])

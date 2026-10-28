"""SA3 hardware selector: four FANUC arms, each independently mock or real.

Matches the paths in the user's pasted launcher (package: sa2_description).
If the package was renamed to sa3_description, change PACKAGE below and
change the Gazebo plugin's $(find sa2_description) path in the Xacro.
"""

import os

import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = "sa2_description"  # Change to "sa3_description" if that is the real package.


def vector_to_string(values):
    if len(values) != 3:
        raise ValueError(f"Expected 3 XYZ/RPY/scale values, received: {values}")
    return " ".join(str(value) for value in values)


def launch_setup(context, *args, **kwargs):
    # OpaqueFunction resolves launch substitutions BEFORE calling xacro.process_file.
    # Calling xacro.process_file with an unresolved LaunchConfiguration will NOT work.
    mode = LaunchConfiguration("hardware_mode").perform(context)
    hardware_modes = {
        i: LaunchConfiguration(f"robot{i}_hardware").perform(context)
        for i in range(1, 5)
    }
    # Prevent two physical plugins from opening the same FANUC RMI endpoint.
    endpoints = {}
    for i, selected_mode in hardware_modes.items():
        if selected_mode == "fanuc":
            endpoint = (
                LaunchConfiguration(f"robot{i}_ip").perform(context),
                LaunchConfiguration(f"robot{i}_rmi_port").perform(context),
            )
            if endpoint in endpoints:
                raise RuntimeError(
                    f"Robot {i} and robot {endpoints[endpoint]} both target "
                    f"FANUC endpoint {endpoint}. Use distinct controllers/endpoints."
                )
            endpoints[endpoint] = i

    all_gazebo = all(value == "gazebo" for value in hardware_modes.values())
    if any(value == "gazebo" for value in hardware_modes.values()) and not all_gazebo:
        raise RuntimeError(
            "Mixed Gazebo with FANUC/mock is not supported by this launcher. "
            "Use mock/fanuc for independent robot selection, or all gazebo."
        )

    pkg = get_package_share_directory(PACKAGE)
    workcell_yaml = os.path.join(pkg, "config", "workcell.yaml")
    with open(workcell_yaml, "r", encoding="utf-8") as f:
        workcell_config = yaml.safe_load(f)

    environment = workcell_config["environment"]
    robot_pedestals = workcell_config["robot_pedestals"]
    gantry = environment["gantry"]

    mappings = {"hardware_mode": mode}
    for i in range(1, 5):
        mappings[f"robot{i}_hardware"] = hardware_modes[i]
        for param in ("ip", "rmi_port", "stream_motion_port"):
            key = f"robot{i}_{param}"
            mappings[key] = LaunchConfiguration(key).perform(context)
        pedestal_name = f"robot{i}_pedestal"
        pedestal = robot_pedestals[pedestal_name]
        mappings[f"{pedestal_name}_xyz"] = vector_to_string(pedestal["xyz"])
        mappings[f"{pedestal_name}_rpy"] = vector_to_string(pedestal["rpy"])

    for transform in ("xyz", "rpy", "scale"):
        mappings[f"gantry_{transform}"] = vector_to_string(gantry[transform])

    robot_description = xacro.process_file(
        os.path.join(pkg, "urdf", "robot.urdf.xacro"),
        mappings=mappings,
    ).toxml()

    env_mappings = {}
    for name in (
        "sa3_main_jig", "inventory_3a_4a", "inventory_3a_4a_2",
        "jig_clamp", "web_ring", "web_ring_2",
    ):
        obj = environment[name]
        env_mappings[f"{name}_xyz"] = vector_to_string(obj["xyz"])
        env_mappings[f"{name}_rpy"] = vector_to_string(obj["rpy"])

    env_description = xacro.process_file(
        os.path.join(pkg, "urdf", "sa3_env.urdf.xacro"),
        mappings=env_mappings,
    ).toxml()

    controllers_yaml = os.path.join(pkg, "config", "ros2_controller.yaml")
    rviz_config = os.path.join(pkg, "rviz", "sa2_description.rviz")

    print("\nSA3 hardware selection:")
    for i in range(1, 5):
        print(f"  Robot {i}: {hardware_modes[i]}",
              f"IP={mappings[f'robot{i}_ip']}" if hardware_modes[i] == "fanuc" else "")
    print("  Gantry:", "gazebo" if all_gazebo else "mock")

    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description,
                         "use_sim_time": all_gazebo}],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="environment_state_publisher",
            namespace="environment",
            output="screen",
            parameters=[{"robot_description": env_description,
                         "use_sim_time": all_gazebo}],
            remappings=[("joint_states", "/environment_joint_states")],
        ),
    ]

    if all_gazebo:
        # Gazebo Classic owns /controller_manager when all four robots are Gazebo.
        gazebo_share = get_package_share_directory("gazebo_ros")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_share, "launch", "gazebo.launch.py")
                ),
            )
        )
        actions.append(
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                output="screen",
                arguments=["-topic", "/robot_description", "-entity", "sa3"],
            )
        )
    else:
        # One manager loads five independent ros2_control systems.
        actions.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                output="screen",
                parameters=[
                    {"robot_description": robot_description,
                     "use_sim_time": False},
                    controllers_yaml,
                ],
            )
        )

    controllers = [
        "joint_state_broadcaster",
        "robot1_arm_controller",
        "robot2_arm_controller",
        "robot3_arm_controller",
        "robot4_arm_controller",
        "gantry_controller",
    ]
    # Spawn sequentially to keep the ros2_control startup logs understandable.
    for index, controller_name in enumerate(controllers):
        actions.append(
            TimerAction(
                period=3.0 + 1.5 * index,
                actions=[Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        controller_name,
                        "--controller-manager", "/controller_manager",
                        "--controller-manager-timeout", "120",
                    ],
                    output="screen",
                )],
            )
        )

    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": all_gazebo}],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )
    return actions


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(
            "hardware_mode", default_value="mock",
            choices=["mock", "fanuc", "gazebo"],
            description="Default mode for all four robots",
        ),
    ]

    for i in range(1, 5):
        declarations.extend([
            DeclareLaunchArgument(
                f"robot{i}_hardware", default_value=LaunchConfiguration("hardware_mode"),
                choices=["mock", "fanuc", "gazebo"],
            ),
            DeclareLaunchArgument(f"robot{i}_ip", default_value="192.168.2.3"),
            DeclareLaunchArgument(f"robot{i}_rmi_port", default_value="16001"),
            DeclareLaunchArgument(f"robot{i}_stream_motion_port", default_value="60015"),
        ])

    declarations.append(
        DeclareLaunchArgument("use_rviz", default_value="false", choices=["true", "false"])
    )
    return LaunchDescription(declarations + [OpaqueFunction(function=launch_setup)])
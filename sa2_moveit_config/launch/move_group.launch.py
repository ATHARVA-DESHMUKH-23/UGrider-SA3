import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

from launch.actions import ExecuteProcess, TimerAction


def vector_to_string(values):
    """Convert a YAML XYZ/RPY list to a space-separated Xacro argument."""
    if len(values) != 3:
        raise ValueError(
            f"Expected exactly 3 XYZ/RPY values, received: {values}"
        )
    return " ".join(str(value) for value in values)


def generate_launch_description():

    # =========================================================
    # PACKAGES
    # =========================================================

    description_pkg = get_package_share_directory(
        "sa2_description"
    )

    moveit_pkg = get_package_share_directory(
        "sa2_moveit_config"
    )

    # =========================================================
    # LOAD PEDESTAL POSITIONS FROM WORKCELL YAML
    # =========================================================

    workcell_yaml = os.path.join(
        description_pkg,
        "config",
        "workcell.yaml"
    )

    with open(workcell_yaml, "r") as file:
        workcell_config = yaml.safe_load(file) or {}

    robot_pedestals = workcell_config.get("robot_pedestals")

    if not isinstance(robot_pedestals, dict):
        raise KeyError(
            f"'robot_pedestals' section not found in {workcell_yaml}"
        )

    pedestal_names = [
        "robot1_pedestal",
        "robot2_pedestal",
        "robot3_pedestal",
        "robot4_pedestal",
    ]

    # Start with the existing hardware mode.
    robot_mappings = {
        "hardware_mode": "mock"
    }

    # Pass XYZ and RPY for all four pedestals into Xacro.
    for name in pedestal_names:
        if name not in robot_pedestals:
            raise KeyError(
                f"Missing '{name}' in {workcell_yaml}"
            )

        pedestal = robot_pedestals[name]

        if "xyz" not in pedestal or "rpy" not in pedestal:
            raise KeyError(
                f"Pedestal '{name}' must contain both xyz and rpy"
            )

        robot_mappings[f"{name}_xyz"] = vector_to_string(
            pedestal["xyz"]
        )

        robot_mappings[f"{name}_rpy"] = vector_to_string(
            pedestal["rpy"]
        )

    print("\nMoveIt pedestal positions loaded from workcell.yaml:")

    for name in pedestal_names:
        print(
            f"  {name}: "
            f"xyz={robot_pedestals[name]['xyz']}, "
            f"rpy={robot_pedestals[name]['rpy']}"
        )

    # =========================================================
    # ROBOT XACRO
    # =========================================================

    robot_xacro = os.path.join(
        description_pkg,
        "urdf",
        "robot.urdf.xacro"
    )

    # =========================================================
    # MOVEIT RVIZ CONFIG
    # =========================================================

    rviz_config = os.path.join(
        moveit_pkg,
        "config",
        "moveit.rviz"
    )

    # =========================================================
    # MOVEIT CONFIGURATION
    # =========================================================

    moveit_config = (
        MoveItConfigsBuilder(
            "sa3_robots",
            package_name="sa2_moveit_config",
        )

        # -----------------------------------------------------
        # Robot description - pedestal positions from YAML
        # -----------------------------------------------------

        .robot_description(
            file_path=robot_xacro,
            mappings=robot_mappings,
        )

        # -----------------------------------------------------
        # Semantic description
        # -----------------------------------------------------

        .robot_description_semantic(
            file_path="config/sa3_robots.srdf",
        )

        # -----------------------------------------------------
        # Kinematics
        # -----------------------------------------------------

        .robot_description_kinematics(
            file_path="config/kinematics.yaml",
        )

        # -----------------------------------------------------
        # Joint limits
        # -----------------------------------------------------

        .joint_limits(
            file_path="config/joint_limits.yaml",
        )

        # -----------------------------------------------------
        # MoveIt controllers
        # -----------------------------------------------------

        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
        )

        # -----------------------------------------------------
        # Planning pipelines
        # -----------------------------------------------------

        .planning_pipelines(
            pipelines=[
                "ompl",
                "pilz_industrial_motion_planner",
            ],
            default_planning_pipeline="ompl",
        )

        # -----------------------------------------------------
        # Pilz Cartesian limits
        # -----------------------------------------------------

        .pilz_cartesian_limits(
            file_path="config/pilz_cartesian_limits.yaml",
        )

        .to_moveit_configs()
    )

    # =========================================================
    # MOVE GROUP
    # =========================================================

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",

        arguments=[
            "--ros-args",

            "--log-level", "moveit_ros_planning:=DEBUG",
            "--log-level", "pilz_industrial_motion_planner:=DEBUG",
            "--log-level", "moveit_core:=DEBUG",
        ],

        parameters=[
            moveit_config.to_dict()
        ],
    )

    
    # =========================================================
    # SAVE CURRENT MOVE_GROUP PARAMETERS FOR OTHER NODES
    # =========================================================

    output_yaml = os.path.expanduser(
        "~/ros_ws/sa3_j/src/sa3_motion/config/move_group.yaml"
    )

    dump_script = f"""
OUTPUT="{output_yaml}"
TEMP="${{OUTPUT}}.tmp"

# Wait until move_group's parameter services are available.
for i in $(seq 1 30); do
    if ros2 param dump /move_group > "$TEMP" 2>/dev/null; then
        # Convert the node-specific root to a wildcard root
        # so other nodes can load these parameters.
        sed -i '1s#^.*$#/**:#' "$TEMP"

        if grep -q 'ros__parameters:' "$TEMP"; then
            mv "$TEMP" "$OUTPUT"
            echo "[MoveIt] Updated parameter YAML: $OUTPUT"
            exit 0
        fi
    fi

    sleep 1
done

rm -f "$TEMP"
echo "[MoveIt] ERROR: Could not dump /move_group parameters."
exit 1
"""

    save_move_group_yaml = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=["bash", "-lc", dump_script],
                output="screen",
            )
        ],
    )


    # =========================================================
    # MOVEIT RVIZ
    # =========================================================

    moveit_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_rviz",
        output="screen",

        arguments=[
            "-d",
            rviz_config
        ],

        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # =========================================================
    # LAUNCH
    # =========================================================

    return LaunchDescription([
        move_group,
        moveit_rviz,
        # save_move_group_yaml,
    ])
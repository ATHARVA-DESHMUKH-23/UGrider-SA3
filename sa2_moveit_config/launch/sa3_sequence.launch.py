import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    """
    SA3 motion sequence.

    Start sa3_bringup.launch.py separately first. This launch file assumes
    MoveIt, ros2_control/controllers, TF, and the planning-scene nodes are
    already running.

    Sequence:
      1. robot3_motion + robot4_motion (both must exit with code 0)
      2. home_all_robots + attach_tool (wait for both exits; ignore result)
      3. robot1_motion + robot2_motion (both must exit with code 0)
      4. robot1_gripper_change (wait for exit; ignore result)
      5. robot1_tying_gun + robot2_tying_gun (wait for both exits; ignore result)
      6. home_all_robots (final operation; report its exit code)
    """

    # Build the same MoveIt configuration used by the existing bringup's
    # ik_reach_test node. Each motion executable receives these parameters.
    moveit_config = MoveItConfigsBuilder(
        "sa3_robots",
        package_name="sa2_moveit_config",
    ).to_moveit_configs()

    moveit_parameters = [moveit_config.to_dict()]

    def motion_node(executable, node_name=None):
        return Node(
            package="sa3_motion",
            executable=executable,
            name=node_name or executable,
            output="screen",
            parameters=moveit_parameters,
        )

    # Phase 1: Robot 3 and Robot 4 run simultaneously. Both must succeed.
    robot3 = motion_node("robot3_motion")
    robot4 = motion_node("robot4_motion")

    # Phase 2: run both simultaneously; any exit code allows progression.
    home_after_setup = motion_node(
        "home_all_robots"
    )
    attach_tool = motion_node("attach_tool")

    # Phase 3: Robot 1 and Robot 2 run simultaneously. Both must succeed.
    robot1 = motion_node("robot1_motion")
    robot2 = motion_node("robot2_motion")

    # Phase 4: run alone; any exit code allows progression.
    gripper_change = motion_node("robot1_gripper_change")

    # Phase 5: both tying-gun nodes run simultaneously; any exit code allows
    # progression once both processes have exited.
    tying_gun1 = motion_node("robot1_tying_gun")
    tying_gun2 = motion_node("robot2_tying_gun")

    # Phase 6: separate process action for final homing.
    home_final = motion_node(
        "home_all_robots"
    )

    def make_phase_handlers(
        phase_name,
        processes,
        next_processes=None,
        require_success=False,
        final_phase=False,
    ):
        next_processes = next_processes or []
        state = {"done": set(), "codes": {}}
        handlers = []

        for process in processes:

            def on_exit(
                event,
                context,
                process=process,
                state=state,
                processes=processes,
                phase_name=phase_name,
                next_processes=next_processes,
                require_success=require_success,
                final_phase=final_phase,
            ):
                process_id = id(process)
                state["done"].add(process_id)
                state["codes"][process_id] = event.returncode

                # Do not move on until every process in this phase has exited.
                if len(state["done"]) < len(processes):
                    return []

                codes = [state["codes"][id(p)] for p in processes]
                all_succeeded = all(code == 0 for code in codes)

                if require_success and not all_succeeded:
                    return [
                        LogInfo(
                            msg=(
                                f"[SA3 SEQUENCE] {phase_name} FAILED. "
                                f"Exit codes: {codes}. Stopping sequence."
                            )
                        )
                    ]

                if final_phase:
                    return [
                        LogInfo(
                            msg=(
                                "[SA3 SEQUENCE] Sequence finished. "
                                f"Final homing exit code: {codes[0]}"
                            )
                        )
                    ]

                return [
                    LogInfo(
                        msg=(
                            f"[SA3 SEQUENCE] {phase_name} finished. "
                            f"Exit codes: {codes}. Starting next phase."
                        )
                    ),
                    *next_processes,
                ]

            handlers.append(
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=process,
                        on_exit=on_exit,
                    )
                )
            )

        return handlers

    phase_handlers = []

    # Phase 1 -> Phase 2: both Robot 3 and Robot 4 must succeed.
    phase_handlers += make_phase_handlers(
        "Robot 3 + Robot 4",
        [robot3, robot4],
        next_processes=[home_after_setup, attach_tool],
        require_success=True,
    )

    # Phase 2 -> Phase 3: wait for both processes, regardless of exit code.
    phase_handlers += make_phase_handlers(
        "Initial homing + tool attachment",
        [home_after_setup, attach_tool],
        next_processes=[robot1, robot2],
    )

    # Phase 3 -> Phase 4: both Robot 1 and Robot 2 must succeed.
    phase_handlers += make_phase_handlers(
        "Robot 1 + Robot 2",
        [robot1, robot2],
        next_processes=[gripper_change],
        require_success=True,
    )

    # Phase 4 -> Phase 5: any gripper-change exit result allows progression.
    phase_handlers += make_phase_handlers(
        "Robot 1 gripper change",
        [gripper_change],
        next_processes=[tying_gun1, tying_gun2],
    )

    # Phase 5 -> Phase 6: wait for both tying-gun processes, regardless of
    # whether either returned success or failure.
    phase_handlers += make_phase_handlers(
        "Robot 1 + Robot 2 tying gun",
        [tying_gun1, tying_gun2],
        next_processes=[home_final],
    )

    # Phase 6: final homing has no successor; report its result.
    phase_handlers += make_phase_handlers(
        "Final homing",
        [home_final],
        final_phase=True,
    )

    # The event handlers must be registered before Robot 3 and Robot 4 start.
    return LaunchDescription([
        *phase_handlers,
        robot3,
        robot4,
    ])
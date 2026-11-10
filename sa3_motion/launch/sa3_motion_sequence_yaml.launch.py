#!/usr/bin/env python3
"""Run SA3 schema-v3 motion sequence using only sa3_generic_executor.

Usage:
  ros2 launch sa3_motion sa3_10m_motion_sequence_generic.launch.py
  ros2 launch sa3_motion sa3_10m_motion_sequence_generic.launch.py sequence_config:=/abs/path/my_sequence.yaml
  ros2 launch sa3_motion sa3_10m_motion_sequence_generic.launch.py sequence_config:=my_sequence.yaml
      (looked up in the current directory first, then in <sa3_motion share>/config/)
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

PACKAGE_NAME = "sa3_motion"


def resolve_config_path(value, config_dir):
    """Find the YAML file from a CLI value.

    Accepts an absolute path, '~' path, a path relative to the current directory,
    or a bare file name inside the package config directory.
    """
    path = os.path.expanduser(value.strip())
    candidates = [path]
    if not os.path.isabs(path):
        candidates.append(os.path.join(config_dir, path))

    for candidate in candidates:
        if os.path.isfile(candidate):
            return os.path.abspath(candidate)

    tried = "\n  ".join(os.path.abspath(c) for c in candidates)
    raise RuntimeError(f"sequence_config '{value}' not found. Tried:\n  {tried}")


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory(PACKAGE_NAME), "config")
    default_config = os.path.join(config_dir, "ms_2_re_generic_executor.yaml")
    config_arg = DeclareLaunchArgument(
        "sequence_config",
        default_value=default_config,
        description=(
            "YAML motion-sequence file: absolute path, path relative to the current "
            "directory, or a file name inside the sa3_motion config directory"
        ),
    )

    dry_run_arg = DeclareLaunchArgument(
        "dry_run", default_value="true",
        description="Safety default: plan/validate only, never execute trajectories or alter the scene",
    )
    sim_arg = DeclareLaunchArgument(
        "simulation_confirmed", default_value="false",
        description="Explicit simulation-only acknowledgement before real execution",
    )

    def setup_sequence(context):
        config_path = resolve_config_path(
            LaunchConfiguration("sequence_config").perform(context), config_dir
        )
        with open(config_path, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}

        try:
            stages = config["motion_sequence"]["stages"]
        except (KeyError, TypeError) as exc:
            raise RuntimeError(
                f"Expected 'motion_sequence.stages' in YAML: {config_path}"
            ) from exc
        if not isinstance(stages, list) or not stages:
            raise RuntimeError("motion_sequence.stages must be a non-empty list")

        if config.get("motion_sequence", {}).get("schema_version") != 3:
            raise RuntimeError("Expected generic schema_version: 3")
        dry_run = LaunchConfiguration("dry_run").perform(context).lower() == "true"
        simulation_confirmed = (
            LaunchConfiguration("simulation_confirmed").perform(context).lower() == "true"
        )
        if not dry_run and not simulation_confirmed:
            raise RuntimeError(
                "Generic execution is simulation-only: set simulation_confirmed:=true "
                "after confirming MoveIt and controllers are in simulation"
            )
        workcell_def = config["motion_sequence"].get("workcell", {})
        workcell_path = os.path.join(
            get_package_share_directory(workcell_def.get("package", "sa2_description")),
            workcell_def.get("file", "config/workcell.yaml"),
        )
        if not os.path.isfile(workcell_path):
            raise RuntimeError(f"Workcell YAML not found: {workcell_path}")

        # A dry run is a schema preflight only. Planning an entire stateful
        # sequence without executing earlier steps would use incorrect start
        # states and could hang on G4/R3; do not pretend otherwise.
        if dry_run:
            task_map = config["motion_sequence"].get("tasks", {})
            combined_map = config["motion_sequence"].get("combined_motions", {})
            for stage in stages:
                if stage.get("mode") == "combined":
                    if stage.get("combined_step") not in combined_map:
                        raise RuntimeError(f"Unknown combined_step in {stage.get('name')}")
                for entry in stage.get("executables", []):
                    params = entry.get("parameters", {}) if isinstance(entry, dict) else {}
                    if params.get("motion_task") not in task_map:
                        raise RuntimeError(f"Unknown motion_task in {stage.get('name')}")
            return [
                LogInfo(msg="[SA3 GENERIC] Schema-v3 preflight passed. NO robot execution performed. "
                            "Use dry_run:=false simulation_confirmed:=true only in a verified simulator."),
                EmitEvent(event=Shutdown(reason="SA3 generic static preflight completed")),
            ]

        moveit_parameters = MoveItConfigsBuilder(
            "sa3_robots", package_name="sa2_moveit_config"
        ).to_moveit_configs().to_dict()

        def normalize_entry(raw, stage_name):
            if isinstance(raw, str):
                name, params = raw, {}
            elif isinstance(raw, dict):
                name = raw.get("name", raw.get("executable"))
                params = raw.get("parameters", {})
                if not isinstance(params, dict):
                    raise RuntimeError(
                        f"Stage '{stage_name}': parameters for '{name}' must be a mapping"
                    )
            else:
                raise RuntimeError(
                    f"Stage '{stage_name}': executable must be a string or mapping"
                )
            if not isinstance(name, str) or not name.strip():
                raise RuntimeError(f"Stage '{stage_name}' has an executable with no valid name")
            return {"name": name.strip(), "parameters": params,
                    "background": bool(raw.get("background", False)) if isinstance(raw, dict) else False}

        def make_node(entry, index):
            if entry["name"] != "sa3_generic_executor":
                raise RuntimeError(
                    f"Legacy executable is not permitted in generic sequence: {entry['name']}"
                )
            params = dict(entry["parameters"])
            params.update({
                "motion_config_path": config_path,
                "workcell_config_path": workcell_path,
                "stage_id": str(stages[index].get("id", f"stage_{index + 1}")),
                "dry_run": dry_run,
                "simulation_confirmed": simulation_confirmed,
            })
            # This node is deliberately instantiated once per task, using ONE
            # generic C++ executable. Persistent gantry services remain alive.
            return Node(
                package=PACKAGE_NAME,
                executable="sa3_generic_executor",
                name=f"sa3_generic_executor_{index}_{params.get('motion_task', 'combined')}",
                output="screen",
                parameters=[moveit_parameters, params],
            )

        def stop(reason):
            return [LogInfo(msg=f"[motion_sequence] STOP: {reason}"),
                    EmitEvent(event=Shutdown(reason=reason))]

        # Persistent background processes, e.g. the gantry waiting at G4.
        background_state = {
            "process": None,
            "finished": False,
            "phase5_started": False,
            "gantry_exited_successfully": False,
        }

        def run_stage(index):
            if index >= len(stages):
                background_state["finished"] = True
                if (background_state["process"] is not None and
                        not background_state["gantry_exited_successfully"]):
                    return [LogInfo(msg="[motion_sequence] All stages passed; waiting for the "
                        "gantry/Robot3 node to exit successfully")]
                return stop("ALL GENERIC STAGES COMPLETED")
            stage = stages[index]
            if not isinstance(stage, dict):
                return stop(f"Stage {index + 1} must be a YAML mapping")

            stage_name = str(stage.get("name", f"Stage {index + 1}"))
            raw_entries = stage.get("executables", [])
            # This is a six-phase recipe: the persistent gantry must remain
            # alive through G4 and the R3 hold, but it is EXPECTED to exit
            # with code 0 after completing Robot3 R4-R7 in Phase 5.
            # The phase5 coordinator and gripper-changing task run concurrently.
            if bool(stage.get("marks_background_completion_phase", False)):
                background_state["phase5_started"] = True
            mode = str(stage.get("mode", "sequential")).lower()
            retries = int(stage.get("retries", 1))
            policy = str(stage.get("on_failure", "abort")).lower()

            if mode == "combined":
                # A combined stage has one universal executor instance.
                # It reads the SAME source recipe path this launcher was given.
                key = stage.get("combined_step")
                if not isinstance(key, str) or not key.strip():
                    return stop(f"Stage '{stage_name}': combined_step is required")
                workcell_package = str(stage.get("workcell_config_package", "sa2_description"))
                workcell_file = str(stage.get("workcell_config_file", "config/workcell.yaml"))
                workcell_path = os.path.join(
                    get_package_share_directory(workcell_package), workcell_file
                )
                if not os.path.isfile(workcell_path):
                    return stop(f"Stage '{stage_name}': workcell file not found: {workcell_path}")
                # Explicit opt-in required to execute combined motion.
                run_motion = stage.get("execute_motion", False)
                if not isinstance(run_motion, bool):
                    return stop(f"Stage '{stage_name}': execute_motion must be true/false")
                entry = {
                    "name": "sa3_generic_executor",
                    "parameters": {
                        "motion_config_path": config_path,
                        "workcell_config_path": workcell_path,
                        "combined_step": key.strip(),
                        "execute_motion": run_motion,
                    },
                }
                return run_single(index, stage_name, entry, retries, 1, policy)

            if not isinstance(raw_entries, list) or not raw_entries:
                return stop(f"Stage '{stage_name}': executables must be a non-empty list")
            if mode not in ("parallel", "sequential", "overlap"):
                return stop(f"Stage '{stage_name}': invalid mode '{mode}'")
            if retries != 1:
                return stop(f"Stage '{stage_name}': automatic stage retries are unsafe after motion may have started")
            if policy not in ("abort", "continue", "final_home", "finish"):
                return stop(f"Stage '{stage_name}': invalid on_failure '{policy}'")
            if mode == "sequential" and len(raw_entries) != 1:
                return stop(f"Stage '{stage_name}': sequential mode requires one executable")
            try:
                entries = [normalize_entry(entry, stage_name) for entry in raw_entries]
            except RuntimeError as exc:
                return stop(str(exc))

            if mode == "sequential":
                return run_single(index, stage_name, entries[0], retries, 1, policy)
            if mode == "overlap":
                if retries != 1:
                    return stop("Overlap cannot retry: the gantry may already hold a rebar")
                if len(entries) != 2 or sum(bool(e["background"]) for e in entries) != 1:
                    return stop("Overlap requires exactly two executables, one with background: true")
                if background_state["process"] is not None:
                    return stop("Only one persistent background operation is supported")
                return run_overlap(index, stage_name, entries)
            return run_parallel(index, stage_name, entries, retries, 1, policy)

        def run_overlap(index, stage_name, entries):
            """Launch both nodes now. Advance after foreground exits successfully,
            keeping the gantry alive and watching for an unexpected exit.
            """
            bg = next(e for e in entries if e["background"])
            fg = next(e for e in entries if not e["background"])
            bg_process = make_node(bg, index)
            fg_process = make_node(fg, index)
            background_state["process"] = bg_process

            def background_exited(event, _context):
                rc = event.returncode
                if rc != 0:
                    return stop(f"Background {bg['name']} exited with error {rc}; gantry/scene needs inspection")
                if not background_state["phase5_started"]:
                    return stop("Gantry exited before Phase 5 placement; inspect attachment and joints")
                background_state["gantry_exited_successfully"] = True
                if background_state["finished"]:
                    return stop("ALL GENERIC STAGES COMPLETED; gantry exited successfully")
                return [LogInfo(msg="[motion_sequence] Gantry/Robot3 R4-R7 exited "
                    "successfully; allowing Phase 5 gripper change and coordinator to finish")]

            def foreground_exited(event, _context):
                rc = event.returncode
                if rc != 0:
                    return stop(f"Foreground {fg['name']} failed with code {rc}")
                return [LogInfo(msg=f"[motion_sequence] {fg['name']} finished; gantry remains "
                    "running, checking G4 hold before Phase 2.")] + run_stage(index + 1)

            return [RegisterEventHandler(OnProcessExit(
                        target_action=bg_process, on_exit=background_exited)),
                    RegisterEventHandler(OnProcessExit(
                        target_action=fg_process, on_exit=foreground_exited)),
                    bg_process, fg_process]

        def run_single(index, stage_name, entry, retries, attempt, policy):
            process = make_node(entry, index)
            exe = entry["name"]

            def on_exit(event, _context):
                rc = event.returncode
                if rc == 0:
                    return [LogInfo(msg=f"[motion_sequence] {stage_name}: {exe} succeeded ({attempt}/{retries})")] + run_stage(index + 1)
                if attempt < retries:
                    return [LogInfo(msg=f"[motion_sequence] {stage_name}: {exe} failed ({rc}); retrying {attempt + 1}/{retries}")] + run_single(index, stage_name, entry, retries, attempt + 1, policy)
                return [LogInfo(msg=f"[motion_sequence] ERROR: {stage_name}: {exe} failed after {retries} attempt(s), exit code {rc}")] + handle_failure(index, stage_name, policy)

            return [RegisterEventHandler(OnProcessExit(target_action=process, on_exit=on_exit)), process]

        def run_parallel(index, stage_name, entries, retries, attempt, policy):
            # Use numeric indexes, not set(entries): entries are dictionaries and unhashable.
            state = {"remaining": set(range(len(entries))), "return_codes": {}, "failed": False}
            actions = []

            for entry_index, entry in enumerate(entries):
                process = make_node(entry, index)
                exe = entry["name"]

                def handler_for(idx, executable):
                    def on_exit(event, _context):
                        rc = event.returncode
                        state["return_codes"][idx] = rc
                        state["failed"] = state["failed"] or rc != 0
                        state["remaining"].discard(idx)
                        if state["remaining"]:
                            return [LogInfo(msg=f"[motion_sequence] {stage_name}: {executable} exited {rc}; waiting for other process(es)")]

                        codes = [
                            f"{entries[i]['name']}={state['return_codes'].get(i)}"
                            for i in range(len(entries))
                        ]
                        if not state["failed"]:
                            return [LogInfo(msg=f"[motion_sequence] {stage_name}: all processes succeeded")] + run_stage(index + 1)
                        if attempt < retries:
                            return [LogInfo(msg=f"[motion_sequence] ERROR: {stage_name} failed ({', '.join(codes)}); retrying whole stage {attempt + 1}/{retries}")] + run_parallel(index, stage_name, entries, retries, attempt + 1, policy)
                        return [LogInfo(msg=f"[motion_sequence] ERROR: {stage_name} failed after {retries} attempt(s): {', '.join(codes)}")] + handle_failure(index, stage_name, policy)
                    return on_exit

                actions.append(RegisterEventHandler(OnProcessExit(target_action=process, on_exit=handler_for(entry_index, exe))))
                actions.append(process)
            return actions

        def handle_failure(index, stage_name, policy):
            if policy == "abort":
                return stop(f"Stage '{stage_name}' failed; sequence aborted")
            if policy == "continue":
                return [LogInfo(msg=f"[motion_sequence] Continuing after failed stage '{stage_name}'")] + run_stage(index + 1)
            if policy == "final_home":
                final_index = len(stages) - 1
                if final_index <= index:
                    return stop(f"Stage '{stage_name}' failed and no later final-home stage is configured")
                final_name = stages[final_index].get("name", "Final stage")
                return [LogInfo(msg=f"[motion_sequence] Stage '{stage_name}' failed; running final stage '{final_name}'")] + run_stage(final_index)
            return stop(f"Stage '{stage_name}' finished with failure; sequence ended")

        return [LogInfo(msg=f"[motion_sequence] Generic executor config: {config_path}; "
                             f"dry_run={dry_run}; simulation_confirmed={simulation_confirmed}")] + run_stage(0)

    return LaunchDescription([config_arg, dry_run_arg, sim_arg,
                              OpaqueFunction(function=setup_sequence)])


if __name__ == "__main__":
    generate_launch_description()
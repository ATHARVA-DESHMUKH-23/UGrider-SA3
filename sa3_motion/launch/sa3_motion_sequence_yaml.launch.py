#!/usr/bin/env python3
"""Run a configurable SA3 motion sequence from YAML.

Usage:
  ros2 launch sa3_motion sa3_motion_sequence.launch.py
  ros2 launch sa3_motion sa3_motion_sequence.launch.py sequence_config:=/abs/path/my_sequence.yaml
  ros2 launch sa3_motion sa3_motion_sequence.launch.py sequence_config:=my_sequence.yaml
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
    default_config = os.path.join(config_dir, "motion_sequence.yaml")
    config_arg = DeclareLaunchArgument(
        "sequence_config",
        default_value=default_config,
        description=(
            "YAML motion-sequence file: absolute path, path relative to the current "
            "directory, or a file name inside the sa3_motion config directory"
        ),
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
            return {"name": name.strip(), "parameters": params}

        def make_node(entry):
            return Node(
                package=PACKAGE_NAME,
                executable=entry["name"],
                output="screen",
                parameters=[moveit_parameters, entry["parameters"]],
            )

        def stop(reason):
            return [LogInfo(msg=f"[motion_sequence] STOP: {reason}"),
                    EmitEvent(event=Shutdown(reason=reason))]

        def run_stage(index):
            if index >= len(stages):
                return stop("All configured stages finished.")
            stage = stages[index]
            if not isinstance(stage, dict):
                return stop(f"Stage {index + 1} must be a YAML mapping")

            stage_name = str(stage.get("name", f"Stage {index + 1}"))
            raw_entries = stage.get("executables", [])
            mode = str(stage.get("mode", "sequential")).lower()
            retries = int(stage.get("retries", 1))
            policy = str(stage.get("on_failure", "abort")).lower()

            if not isinstance(raw_entries, list) or not raw_entries:
                return stop(f"Stage '{stage_name}': executables must be a non-empty list")
            if mode not in ("parallel", "sequential"):
                return stop(f"Stage '{stage_name}': invalid mode '{mode}'")
            if retries < 1:
                return stop(f"Stage '{stage_name}': retries must be at least 1")
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
            return run_parallel(index, stage_name, entries, retries, 1, policy)

        def run_single(index, stage_name, entry, retries, attempt, policy):
            process = make_node(entry)
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
                process = make_node(entry)
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

        return [LogInfo(msg=f"[motion_sequence] Using config: {config_path}")] + run_stage(0)

    return LaunchDescription([config_arg, OpaqueFunction(function=setup_sequence)])


if __name__ == "__main__":
    generate_launch_description()
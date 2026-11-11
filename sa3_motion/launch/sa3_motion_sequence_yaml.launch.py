#!/usr/bin/env python3
"""YAML action-library-driven SA3 orchestration (simulation only).

Action library: how motion/scene actions work.
Execution sequence: which named actions run, and sequential/parallel ordering.
Gantry: persistent node commanding the EXISTING gantry_controller via FJT; no custom srv.
"""
import os
import re
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

PKG = 'sa3_motion'

def get_file(name, config_dir):
    val = os.path.expanduser(name.strip())
    for candidate in (val, os.path.join(config_dir, val)):
        if os.path.isfile(candidate):
            return os.path.abspath(candidate)
    raise RuntimeError(f'Configuration file does not exist: {name}')


def merge_params(defaults, overrides):
    if not isinstance(defaults, dict) or not isinstance(overrides, dict):
        raise RuntimeError('Action defaults and with: must be mappings')
    return {**defaults, **overrides}


def validate_params(action_name, spec, args):
    defaults = spec.get('defaults', {})
    values = merge_params(defaults, args)
    missing = set()
    # Validate all placeholder names, including those in nested lists/maps.
    def walk(v):
        if isinstance(v, dict):
            for t in v.values(): walk(t)
        elif isinstance(v, list):
            for t in v: walk(t)
        elif isinstance(v, str):
            for key in re.findall(r'\{\{\s*([a-zA-Z_][\w.]*)\s*\}\}', v):
                current = values
                for part in key.split('.'):
                    if not isinstance(current, dict) or part not in current:
                        missing.add(key)
                        break
                    current = current[part]
    walk(spec.get('steps', []))
    if missing:
        raise RuntimeError(f"{action_name}: action inputs missing {sorted(missing)}")


def generate_launch_description():
    cfgdir = os.path.join(get_package_share_directory(PKG), 'config')
    args = [
        DeclareLaunchArgument('sequence_config', default_value=os.path.join(cfgdir, 'sa3_execution_sequence.yaml')),
        DeclareLaunchArgument('action_library', default_value=os.path.join(cfgdir, 'sa3_execution_sequence.yaml')),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('simulation_confirmed', default_value='true'),
        DeclareLaunchArgument('workcell_config_path', default_value=''),
        DeclareLaunchArgument('gantry_horizontal_min_m', default_value='-10.6'),
        DeclareLaunchArgument('gantry_horizontal_max_m', default_value='10.6'),
        DeclareLaunchArgument('gantry_vertical_min_m', default_value='0.0'),
        DeclareLaunchArgument('gantry_vertical_max_m', default_value='6.6'),
    ]

    def setup(context):
        seq_file = get_file(LaunchConfiguration('sequence_config').perform(context), cfgdir)
        lib_file = get_file(LaunchConfiguration('action_library').perform(context), cfgdir)
        with open(seq_file, encoding='utf8') as f: sequence = (yaml.safe_load(f) or {}).get('sa3_sequence', {})
        with open(lib_file, encoding='utf8') as f: library = (yaml.safe_load(f) or {}).get('action_library', {})
        if sequence.get('schema_version') != 1 or library.get('schema_version') != 1:
            raise RuntimeError('Expected schema_version: 1 in both YAML files')
        actions = library.get('actions', {})
        stages = sequence.get('stages', [])
        if not stages or not isinstance(stages, list) or not isinstance(actions, dict):
            raise RuntimeError('Sequence stages and library actions required')
        for i, stage in enumerate(stages):
            if not isinstance(stage, dict):
                raise RuntimeError(f'Stage {i} must be mapping')
            mode = stage.get('mode', 'sequential')
            if mode not in ('sequential', 'parallel'):
                raise RuntimeError(f'Unknown stage mode: {mode}')
            for entry in stage.get('actions', []):
                name = entry.get('action', '')
                if name not in actions:
                    raise RuntimeError(f'Unknown action library name: {name}')
                validate_params(name, actions[name], entry.get('with', {}))
        dry = LaunchConfiguration('dry_run').perform(context).lower() == 'true'
        simulation = LaunchConfiguration('simulation_confirmed').perform(context).lower() == 'true'
        if dry:
            count = sum(len(s.get('actions', [])) for s in stages if s.get('enabled', True))
            return [LogInfo(msg=f'[SA3 PREFLIGHT] {len(stages)} stages / {count} action invocations validated. '
                'No motors or planning-scene objects were commanded.'),
                EmitEvent(event=Shutdown(reason='SA3 action/sequence YAML preflight passed'))]
        if not simulation:
            raise RuntimeError('Set simulation_confirmed:=true for simulation motion; not hardware certified')
        for stage in stages:
            if stage.get('enabled', True) and stage.get('mode') == 'parallel' and not stage.get('verified_non_interference', False):
                raise RuntimeError(f"Parallel stage '{stage.get('name')}' requires verified_non_interference: true "
                                   'after checking gantry/robot moving volumes in simulation')
        moveit = MoveItConfigsBuilder('sa3_robots', package_name='sa2_moveit_config').to_moveit_configs().to_dict()
        # Gantry uses EXISTING joint_trajectory_controller, independently of MoveIt.
        # No custom service generation, no gantry MoveIt/SRDF planning group.
        gantry = Node(package=PKG, executable='gantry_controller', name='gantry_controller',
            output='screen', parameters=[{
                'dry_run': False, 'simulation_confirmed': True,
                'workcell_config_path': LaunchConfiguration('workcell_config_path').perform(context),
                'horizontal_min_m': float(LaunchConfiguration('gantry_horizontal_min_m').perform(context)),
                'horizontal_max_m': float(LaunchConfiguration('gantry_horizontal_max_m').perform(context)),
                'vertical_min_m': float(LaunchConfiguration('gantry_vertical_min_m').perform(context)),
                'vertical_max_m': float(LaunchConfiguration('gantry_vertical_max_m').perform(context)),
            }])
        done = {'shutdown': False}
        def stop(reason):
            if done['shutdown']: return []
            done['shutdown'] = True
            return [LogInfo(msg='[SA3 ACTIONS] STOP: ' + reason), EmitEvent(event=Shutdown(reason=reason))]
        def new_process(action, stage_num, instance_num):
            name = action['action']
            p = {
                'action_library_path': lib_file, 'action_name': name,
                'action_inputs_yaml': yaml.safe_dump(action.get('with', {}), default_flow_style=False),
                'workcell_config_path': LaunchConfiguration('workcell_config_path').perform(context),
                'dry_run': False, 'simulation_confirmed': True,
                'instance_id': f'{stage_num}_{instance_num}_{name}',
            }
            return Node(package=PKG, executable='sa3_generic_executor',
                        name=f'sa3_action_{stage_num}_{instance_num}_{name}', output='screen',
                        parameters=[moveit, p])
        def run_stage(index):
            while index < len(stages) and not stages[index].get('enabled', True): index += 1
            if index >= len(stages): return stop('All enabled SA3 actions completed')
            stage = stages[index]
            entries = stage.get('actions', [])
            if not entries: return stop(f"Stage '{stage.get('name')}' has no actions")
            mode = stage.get('mode', 'sequential')
            label = stage.get('name', f'Stage {index}')
            if mode == 'sequential':
                return run_serial(index, 0, label)
            state = {'left': set(range(len(entries))), 'failed': False}
            nodes = []
            for j, item in enumerate(entries):
                node = new_process(item, index, j)
                def callback_factory(k):
                    def callback(event, _context):
                        if done['shutdown']: return []
                        state['left'].discard(k)
                        state['failed'] |= (event.returncode != 0)
                        if event.returncode != 0:
                            return stop(f"{label}, action {entries[k]['action']} failed: {event.returncode}")
                        if not state['left'] and not state['failed']:
                            return [LogInfo(msg=f'[SA3 ACTIONS] {label} succeeded')] + run_stage(index + 1)
                        return []
                    return callback
                nodes += [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=callback_factory(j))), node]
            return nodes
        def run_serial(stage_index, instance_index, label):
            entries = stages[stage_index]['actions']
            if instance_index >= len(entries):
                return [LogInfo(msg=f'[SA3 ACTIONS] {label} succeeded')] + run_stage(stage_index + 1)
            node = new_process(entries[instance_index], stage_index, instance_index)
            def completed(event, _context):
                if done['shutdown']: return []
                if event.returncode != 0:
                    return stop(f"{label} action {entries[instance_index]['action']} failed: {event.returncode}")
                return run_serial(stage_index, instance_index + 1, label)
            return [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=completed)), node]
        def gantry_exited(event, _context):
            if not done['shutdown']:
                return stop(f'Gantry server exited unexpectedly: {event.returncode}')
            return []
        return [LogInfo(msg=f'[SA3 ACTIONS] library={lib_file}; sequence={seq_file}'),
                RegisterEventHandler(OnProcessExit(target_action=gantry, on_exit=gantry_exited)),
                gantry] + run_stage(0)

    return LaunchDescription(args + [OpaqueFunction(function=setup)])
# SA1 Robot Planning

ROS 2 Humble and MoveIt 2 packages for the SA1 dual-FANUC workcell. The
repository contains the robot and EOAT descriptions, the YAML-driven workcell,
controller and MoveIt configuration, collision-scene utilities, and custom
pick/place/tying programs.

## Packages

- `sa1_description` — robot and EOAT Xacros, environment meshes, generated
  workcell frames, controllers, RViz configuration, and the central
  `config/workcell.yaml` file.
- `sa1_moveit_config` — SRDF, kinematics, joint limits, planning pipelines,
  controller mappings, and launch files.
- `sa1_motion` — MoveGroupInterface programs and planning-scene object
  lifecycle utilities.
- `fanuc_description` — FANUC robot descriptions used by the SA1 Xacros.

## Build the workspace

Clone the repository directly into the `src` directory of a ROS 2 workspace:

```bash
mkdir -p ~/sa1/src
cd ~/sa1/src
git clone git@github.com:Dexsent-Robotics/moveit-planning-package.git .
```

Install dependencies and build:

```bash
cd ~/sa1
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Source ROS 2 and this workspace in every new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/sa1/install/setup.bash
```

## Run with RViz and mock hardware

To open the workcell for inspection and interactive planning:

```bash
ros2 launch sa1_moveit_config rviz.launch.py hardware_mode:=mock
```

To run a motion program as a self-contained launch:

```bash
ros2 launch sa1_moveit_config motion.launch.py \
  hardware_mode:=mock \
  motion_program:=complete_pick_place
```

The motion launch starts the robot description, controllers, MoveIt
`move_group`, planning scene, and selected program. Do not also start the
default RViz launch, because that would create a second environment/controller
stack.

For a two-terminal workflow, start the environment first and tell the motion
launch to reuse it:

```bash
# Terminal 1: environment and RViz
ros2 launch sa1_moveit_config rviz.launch.py hardware_mode:=mock

# Terminal 2: MoveIt and motion program
ros2 launch sa1_moveit_config motion.launch.py \
  hardware_mode:=mock \
  motion_program:=complete_pick_place \
  start_environment:=false
```

### Optional aliases

Add these aliases to `~/.bashrc`, then run `source ~/.bashrc`:

```bash
alias env_rviz='ros2 launch sa1_moveit_config rviz.launch.py hardware_mode:=mock'
alias move_jbar='ros2 launch sa1_moveit_config motion.launch.py hardware_mode:=mock motion_program:=j_bar_pick_place'
alias move_complete='ros2 launch sa1_moveit_config motion.launch.py hardware_mode:=mock motion_program:=complete_pick_place'
```

- `env_rviz` opens the workcell without starting an automatic program.
- `move_jbar` changes from the Special tool to Schunk, processes six
  `15C_1_*` parts, indexes the jig by `-1.0 m` local Y, processes six
  `15C_2_*` parts, indexes the jig again, and homes both robots.
- `move_complete` performs the same two-family workflow after ROD1–ROD4.

## `workcell.yaml`: the single source of truth

[workcell.yaml](sa1_description/config/workcell.yaml) contains workcell layout,
tool geometry, task frames, motion offsets, and ordered motion steps. Xacro,
the RViz workcell visualizer, planning-scene utilities, and motion programs all
read this file.

Coordinate conventions:

- XYZ values are in metres.
- RPY values are `[roll, pitch, yaw]` in radians.
- `world_xyz` / `world_rpy` are relative to `world`.
- `local_xyz` / `local_rpy` are relative to the entry's `parent` frame.
- Mesh scale is dimensionless; `[0.001, 0.001, 0.001]` converts millimetres to
  metres.
- A negative offset moves along the negative direction of the selected world
  or local axis. It is not automatically an approach or retreat.

### 1. Environment entities and spawn poses

The first group of dictionaries describes physical entities and generated
inventory objects.

| YAML section | Purpose |
| --- | --- |
| `sa1_jig` | Jig collision-object ID, TF frame, mesh, joint-zero pose, per-family local-Y index, limits and commanded speed. |
| `inventory` | Parent frame and mesh for the straight-rod inventory. |
| `j_bars_inventory` | Parent environment object for the 15C inventories. |
| `rebar1`, `rebar2` | Generated 10 mm and 12 mm rod columns. |
| `rebar_stack.columns` | Names of rod-column dictionaries to instantiate. |
| `template_15c`, `template_15c_2` | Generated `15C_1_*` and `15C_2_*` families. |
| `template_stacks.layouts` | Template-family dictionaries instantiated by TF, RViz and MoveIt. |
| `special_gripper`, `schunk_gripper` | Removable-tool meshes, mesh origins, mount transforms and TCPs. |

A generated family uses these common fields:

| Field | Meaning |
| --- | --- |
| `parent` | TF frame in which the first object pose is expressed. |
| `id_prefix` | Prefix used for MoveIt collision-object IDs. |
| `frame_prefix` | Prefix used for generated TF frame names. |
| `first_index` | Numeric suffix of the first object. |
| `index_step` | Suffix increment between objects. |
| `count` | Number of objects, including the first object. |
| `local_xyz`, `local_rpy` | Pose of the first object relative to `parent`. |
| `clone_step_xyz` | Parent-local translation added once per clone index. |
| `mesh` | `package://` mesh resource shared by the family. |

For example, `rebar1` uses `first_index: 1` and `index_step: 2`, so it creates
`ROD1`, `ROD3`, `ROD5`, and so on. `rebar2` starts at 2 and creates the even
rod IDs. The pose of clone index `i` is:

```text
clone_local_xyz = local_xyz + i * clone_step_xyz
```

`template_stacks.layouts` instantiates both template families and defines their
execution order for `j_bar_pick_place` and `complete_pick_place`. Both families
reuse `planner_sequence.template_15c`; their IDs, pick frames, place frames,
tie frames, clone spacing and tie offsets come from their own layout entries.

### 2. Task frames

The `frames:` dictionary defines fixed task frames. Each entry has exactly one
parent and a pose relative to that parent:

```yaml
frames:
  15C_2_pf:
    parent: 15C_2_1
    xyz: [0.0, 0.12, 0.005]
    rpy: [-1.5708, -0.7853, 0.0]
```

Important frame categories are:

- Dock frames: `special_gripper_dock`, `schunk_gripper_dock`.
- Rod pick frames: `10mm_pf`, `12mm_pf`.
- Rod place frames: `10mm_placeF`, `12mm_placeF`.
- Template pick frames: `15C_1_pf`, `15C_2_pf`.
- First-family place/tie frames: `15C_1_placeF`, `15C_1_tf_1`, `15C_1_tf_2`.
- Second-family place/tie frames: `15C_2_placeF`, `15C_2_tf_1`, `15C_2_tf_2`.
- Final workcell target: `sa1_placeF`.

The object frame and pick frame are different concepts. For example,
`15C_2_1` locates the object itself, while `15C_2_pf` locates the Robot2 TCP
relative to that object. `vc_frames.urdf.xacro` converts these dictionaries
into fixed TF links whenever the robot description is expanded.

### 3. Planning variables and offsets

The `pick_place:` dictionary contains reusable ROD1–ROD4 geometry:

- Pick approach and side-entry/retreat offsets.
- Place approach and side-entry/retreat offsets.
- `Transfer_height_Z`, which is an absolute world-Z coordinate.
- `Lower_rod_place_Z` and `Lower_rod_mid_Z`. The loader enforces
  `Lower_rod_mid_Z == Lower_rod_place_Z * 0.5`.
- ROD3/ROD4 place offsets and intentional seated-rod contacts.

The `tool_change:` dictionary contains 100 mm approach translations expressed
in the corresponding dock frame. The loader checks that each approach vector
has a magnitude of exactly `0.100 m`.

Tool dictionaries separate four transforms that should not be confused:

- Mount/dock transform — positions the collision-object base at the flange or
  dock.
- Mesh pose — compensates for the CAD origin inside the tool base.
- Planning TCP — the physical endpoint targeted by robot motion.
- Visualization/object TCP — the equivalent TCP expressed in the removable
  collision object's coordinate system.

`template_15c` also provides family-level motion values:

- `place_step_world_y` — world-Y spacing between successive placement poses.
- `tie_step_world_y` — world-Y spacing between successive tying poses.
- `tie_offset` — negative local-X offset from each tying frame to its exact tie
  target.

### 4. Planner sequences

`planner_sequence:` defines ordered pick and place steps for `ROD1`–`ROD4`
and `template_15c`. The code executes every list entry in file order.

| Step field | Required | Meaning |
| --- | --- | --- |
| `id` | Yes | Unique step name within its list, such as `P1` or `G2`. |
| `description` | Yes | Human-readable text included in logs. |
| `reference` | Yes | Named reference pose or `current_tcp`. |
| `orientation_reference` | No | Pose supplying orientation; defaults to `reference`. |
| `motion` | Yes | `moveJ` for OMPL joint-space planning or `moveL` for Pilz LIN. |
| `offset_frame` | Yes | Apply XYZ in `world` or the reference pose's `local` axes. |
| `rpy_frame` | No | Apply RPY in `world` or `local`; defaults to `local`. |
| `xyz` | Yes | Translation offset `[x, y, z]` in metres. |
| `rpy` | Yes | Orientation offset `[roll, pitch, yaw]` in radians. |
| `inherit_position` | No | Replace selected `x`, `y`, or `z` values with the previous target's values. |
| `before_action` | No | Safety/object-lifecycle action executed before motion. |
| `after_action` | No | Safety/object-lifecycle action executed after successful motion. |
| `acceleration_scaling` | No | Per-step MoveL acceleration scale in `(0, 1]`. |

Rod action handlers recognize `prepare_pick`, `attach_rod`,
`finish_pick_extraction`, `allow_place_contact`, `detach_rod`, and
`finish_place`. Template handlers recognize `prepare_pick`, `attach_template`,
`finish_pick_extraction`, and `allow_place_contact`. An unknown action causes
the sequence to stop instead of silently skipping a safety operation.

### Short editing example

This abbreviated example shows how one template family, its pick frame, and
its first planner step relate:

```yaml
workcell:
  template_15c:
    id_prefix: 15C_1_
    frame_prefix: 15C_1_
    first_index: 1
    index_step: 1
    count: 6                         # Change to 4 to spawn/process four parts
    active_index: 1
    pick_frame: 15C_1_pf
    parent: J_BARS_INVENTORY
    local_xyz: [0.29, 0.01, 1.68]   # First object pose in parent axes
    local_rpy: [1.5708, 0.0, 1.5708]
    clone_step_xyz: [-0.035, 0.0, 0.0]
    mesh: package://sa1_description/meshes/15C-3.STL

  frames:
    15C_1_pf:
      parent: 15C_1_1
      xyz: [0.0, 0.12, 0.005]
      rpy: [-1.5708, -0.7853, 0.0]

  planner_sequence:
    template_15c:
      pick:
        - id: P1
          description: pick approach
          reference: pick_frame
          motion: moveJ
          offset_frame: local
          xyz: [-0.100, 0.0, 0.0]
          rpy: [0.0, 0.0, 0.0]
```

Changing `count` adds or removes generated objects at the end of the family.
Changing P1 `xyz` changes the approach offset without recompiling. To add a
new template family, give it unique ID/frame prefixes, register its dictionary
under `template_stacks.layouts`, and add its task frame under `frames`. A new
family is not automatically added to the motion program; the C++ workflow must
explicitly select it before its planner sequence will execute.

## How C++ interprets the YAML

The main implementation is
[rebar_pick_place.cpp](sa1_motion/src/rebar_pick_place.cpp). Its configuration
path is:

```text
workcell.yaml
  -> validate and parse YAML dictionaries
  -> look up configured TF reference poses
  -> resolve each target pose and its world/local offsets
  -> execute before_action
  -> plan and execute MoveJ or MoveL
  -> execute after_action
```

### Configuration and execution helpers

| Helper | Input | Output / responsibility |
| --- | --- | --- |
| `requiredScalar()` | YAML dictionary and key | Validated finite `double`. |
| `requiredOffset()` | YAML dictionary and key | Validated three-value XYZ/RPY array. |
| `plannerVector3()` | One planner step and field name | Validated three-value planner array. |
| `configuredOffsetFrame()` | `world` or `local` text | Internal offset-frame enum; rejects any other value. |
| `readFourRodConfig()` | Complete `workcell` node | Validated rod offsets, clone steps, transfer height and collision-contact configuration. |
| `configuredPose()` | YAML node plus XYZ/RPY field names | `geometry_msgs::msg::Pose`. |
| `readToolChangeConfig()` | Complete `workcell` node | Tool transforms and validated dock-approach vectors. |
| `readConfiguredPlannerSteps()` | YAML step list and diagnostic name | Ordered vector of validated `ConfiguredPlannerStep` objects. |
| `lookupFramePose()` | TF buffer, source frame and target frame | Source-frame pose expressed in the requested target frame. |
| `offsetWorld()` | Reference pose and world XYZ delta | Pose translated in world axes with orientation preserved. |
| `offsetInLocalFrame()` | Reference pose and local XYZ delta | Pose translated along the reference pose's rotated local axes. |
| `applyConfiguredRpyOffset()` | Reference quaternion, RPY delta and frame choice | Normalized target quaternion. |
| `resolveConfiguredPlannerTarget()` | Parsed step, reference-pose map, current TCP and optional previous target | Final Cartesian target for one step. |
| `executeConfiguredPlannerSteps()` | Move group, tip link, ordered steps, references and action callback | Runs all steps sequentially and returns `true` only if every action, plan and execution succeeds. |
| `planAndExecutePoseMoveJ()` | Move group, tip link and target pose | OMPL pose-target planning and trajectory execution result. |
| `planAndExecuteMoveL()` | Move group, tip link, target pose and acceleration scale | Pilz LIN planning and trajectory execution result. |

`resolveConfiguredPlannerTarget()` first resolves `reference`, applies XYZ in
the selected offset frame, resolves `orientation_reference`, applies RPY in
the selected rotation frame, and finally copies any axes listed in
`inherit_position` from the previous target.

`executeConfiguredPlannerSteps()` obtains the live TCP pose for every step,
calls the workflow's action handler, selects MoveJ or MoveL, stops immediately
on failure, and makes the successful target available to the next step. This
is why list ordering and `inherit_position` are significant.

## Motion programs

Select a program with `motion_program:=<name>`. The main workcell programs are:

- `j_bar_pick_place` — Special-to-Schunk tool change, both configured six-part
  template families, per-family jig indexing, upper/lower tying, safe
  retracts, collision restoration, and dual-arm home.
- `complete_pick_place` — YAML-driven ROD1–ROD4 pick/place followed by the
  complete `j_bar_pick_place` workflow.
- `rebar_pick_place` — legacy four-rod workflow.
- `rebar_pick_place_modular` — modular four-rod development/reference program.
- `tool_change_tying_test` — isolated tool-change and tying test.
- `pose_sequence` — general Robot2 pose-sequence example.
- `tcp_test_runner`, `simultaneous_test_runner`, `sequential_test_runner`, and
  `ik_reliability_test` — diagnostic/test executables.

Example:

```bash
ros2 launch sa1_moveit_config motion.launch.py \
  hardware_mode:=mock \
  motion_program:=j_bar_pick_place
```

## When to rebuild

With the documented `--symlink-install` build, changes to
`sa1_description/config/workcell.yaml` are read when the affected nodes launch.
Stop the active launch, edit the YAML, and relaunch; a rebuild is not normally
required.

Rebuild after changing C++, Xacro structure, launch files, CMake, package
metadata, or installed scripts:

```bash
cd ~/sa1
colcon build --symlink-install
source install/setup.bash
```

## Important frames

- `world` — MoveIt planning reference.
- `sa1_jig_base_link` / `SA1_JIG` — jig model and task-frame parent.
- `inventory` — parent for generated straight rods.
- `J_BARS_INVENTORY` — parent for generated 15C objects.
- `ROD1`, `ROD2`, ... — generated straight-rod object frames.
- `15C_1_1` ... `15C_1_6` and `15C_2_1` ... `15C_2_6` — generated template
  object frames with the current default counts.
- `10mm_pf`, `12mm_pf`, `15C_1_pf`, `15C_2_pf` — pick task frames.
- `10mm_placeF`, `12mm_placeF`, `15C_1_placeF`, `15C_2_placeF` — place task frames.
- `15C_1_tf_1/2`, `15C_2_tf_1/2` — Robot1 upper/lower tying targets.
- `robot2_gripper_mount` — Robot2 flange target used during tool change.
- `robot2_spl_end_effector_link` — Robot2 Special Gripper planning TCP.
- `robot2_end_effector_link` — Robot2 Schunk planning TCP.

## Troubleshooting

- Source `/opt/ros/humble/setup.bash` and `~/sa1/install/setup.bash` in every
  terminal.
- If a YAML edit is not visible, stop and relaunch all affected nodes. Confirm
  the terminal sourced the intended workspace rather than an older overlay.
- YAML indentation is structural. Use spaces consistently and keep sequence
  entries under their correct `pick:` or `place:` list.
- Duplicate generated IDs or step IDs are rejected during startup.
- `xyz` and `rpy` planner fields must contain exactly three finite numbers.
- `motion` accepts only `moveJ` and `moveL`; offset frames accept only `world`
  and `local`.
- A pose can have valid IK and still be rejected because of collision or joint
  velocity/acceleration limits. Read the first MoveIt error before changing
  offsets or disabling collision checks.
- If RViz reports a missing SRDF, launch through
  `sa1_moveit_config/launch/rviz.launch.py` rather than starting RViz manually.
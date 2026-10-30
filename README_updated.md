# SA3 U-Girder Subassembly Simulation

ROS 2 and MoveIt-based simulation for the L&T U-Girder Subassembly 3 (SA3) workcell.

## Project concept: one workcell, multiple assembly sequences

The **physical workcell stays the same** while the motion/task sequence changes for different rebar cross-sections.

| Component | Responsibility | Varies by cross-section? |
| --- | --- | --- |
| `sa2_description/config/workcell.yaml` | Physical environment: robots, tools, fixtures, available rebar objects and inventories, transforms/frames, and collision geometry | **Normally no** |
| `sa3_motion/config/ms_2_re.yaml` | Motion/task sequence for one cross-section: operations, selected objects and frames, offsets, order, retries, and error handling | **Yes** |
| `sa3_motion/config/ms_<variant>.yaml` | A different cross-section's motion/task sequence | **Yes** |
| `sa3_motion/src/*.cpp` | Reusable ROS/MoveIt executables implementing robot operations | **Ideally no** |
| `sa3_motion/include/sa3_motion/workcell_program.hpp` | Shared YAML-loading and validation helpers for the refactored C++ programs | **No** |

**Design rule:** `workcell.yaml` defines **what exists** in the cell; `ms_*.yaml` defines **what the robots do, how they move, and in what order**. A new cross-section should require a new `ms_*.yaml`, not renaming/replacing `workcell.yaml`, as long as the required physical objects and TF frames are already defined.

### Intended execution flow

```text
sa2_description/config/workcell.yaml  (common physical workcell)
                 |
                 v
       SA3 workcell / MoveIt scene
                 |
                 +----------------------------------+
                                                    |
sa3_motion/config/ms_2_re.yaml (cross-section A)    |
sa3_motion/config/ms_<variant>.yaml (section B)     |
                 |                                  |
                 v                                  v
  sa3_motion_sequence_yaml.launch.py --> reusable C++ executables
                                            |
                                            v
                                  MoveIt / robot controllers
```

> **Current implementation versus target design:** The YAML-driven C++ refactor supplied so far still stores detailed motion instructions in `workcell.planner_sequence` (and some job settings in `workcell.flange_pp`, `workcell.shear_links_pp`, etc.). The current `ms_2_re.yaml` primarily launches executables and passes ROS parameters. **The intended separation above is not fully implemented yet.** The launch parser and executables must be updated to accept detailed task steps from `ms_*.yaml` before removing those task sections from `workcell.yaml`. Do not remove them from the current working configuration prematurely.

## Build

Use the ROS 2 distribution installed for your branch (Humble or Jazzy). The paths below follow the uploaded reference workspace layout:

```bash
cd ~/ros_ws/sa3_j
source /opt/ros/humble/setup.bash

chmod +x src/src/sa2_description/scripts/rebar_visualizer.py
chmod +x src/src/sa2_description/scripts/workcell_visualizer.py
chmod +x src/src/sa2_description/scripts/gripper_visualizer.py

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

For Jazzy, replace `humble` with `jazzy`. If your workspace is `~/sa3` or has `src/sa2_description` rather than `src/src/sa2_description`, adjust the paths accordingly.

### CMake requirements for the YAML-driven executables

The targets `flange_pp`, `shear_links_pp`, `tying_rebars`, `gantry_move`, and `robot1_gripper_change` must be built and installed if your sequence uses them.

The refactored `shear_links_pp`, `tying_rebars`, and `robot1_gripper_change` targets require the shared header and YAML-loading dependencies. In `sa3_motion/CMakeLists.txt`, retain their existing MoveIt and ROS dependencies, and include:

```cmake
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

foreach(target shear_links_pp tying_rebars robot1_gripper_change)
  if(TARGET ${target})
    target_include_directories(${target} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
    ament_target_dependencies(${target} ament_index_cpp)
    target_link_libraries(${target} yaml-cpp)
  endif()
endforeach()
```

Place the header at `sa3_motion/include/sa3_motion/workcell_program.hpp`. The `if(TARGET ...)` check does not create missing executables; keep the relevant `add_executable(...)`, other target dependencies, links to `planning_scene_objects` where needed, and `install(TARGETS ...)` declarations. Ensure `package.xml` declares the corresponding dependencies where missing.

Check the executables after a successful build:

```bash
ros2 pkg executables sa3_motion | grep -E \
  'flange_pp|shear_links_pp|tying_rebars|gantry_move|robot1_gripper_change'
```

## Launch

### 1. Start the common workcell and planning scene

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/sa3_j/install/setup.bash
ros2 launch sa2_moveit_config sa3_bringup.launch.py
```

### 2. Run a cross-section motion sequence (in another terminal)

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/sa3_j/install/setup.bash

ros2 launch sa3_motion sa3_motion_sequence_yaml.launch.py \
  sequence_config:="$(ros2 pkg prefix sa3_motion)/share/sa3_motion/config/ms_2_re.yaml"
```

For a different cross-section, **leave `workcell.yaml` unchanged** and select its sequence file instead:

```bash
ros2 launch sa3_motion sa3_motion_sequence_yaml.launch.py \
  sequence_config:="$(ros2 pkg prefix sa3_motion)/share/sa3_motion/config/ms_<variant>.yaml"
```

Replace `ms_<variant>.yaml` with a real installed file (for example, `ms_3_re.yaml`). Add that file to `sa3_motion/config/` before building, or launch directly with its absolute source path. The filenames are examples; only files you have actually created can be launched.

### Optional: SA3 pick-and-place-only sequence

If `ms_sa3_pp.yaml` is present in your `sa3_motion/config/` directory:

```bash
ros2 launch sa3_motion sa3_motion_sequence_yaml.launch.py \
  sequence_config:="$(ros2 pkg prefix sa3_motion)/share/sa3_motion/config/ms_sa3_pp.yaml"
```

## YAML responsibilities

### `workcell.yaml`: environment, not manufacturing workflow

Keep reusable physical definitions here, such as:

- Robot groups, tool-mount links, gripper/tool geometry, collision meshes, planning-scene objects.
- Rebar/inventory objects and their static configuration.
- Workcell fixtures and the named TF frames used by the programs.

For example, `inventory_3a_4a_frame`, `inventory_3a_4a_frame_2`, `place_frame_10_l`, and `place_frame_10` may be **defined** here. The actual instruction to pick `rebar_3c` and then `rebar_3d` belongs in the assembly sequence.

### `ms_*.yaml`: task and motion program for a cross-section

The intended responsibilities of the motion sequence file are:

- Task order: attach tool → pick/place flanges → pick/place shear links → tool change → tie → gantry → final assembly → home.
- Which objects and frames each robot uses at each stage.
- Pick/place offsets, approach/retract and motion commands such as MoveJ, LIN and CIRC.
- Repetitions, parallel versus sequential execution, retry limits and failure handling.

The **currently supported** stage-level format is similar to:

```yaml
motion_sequence:
  stages:
    - name: "Pick/place Half web flanges"
      executables:
        - name: flange_pp
          parameters:
            workcell_config_package: sa2_description
            workcell_config_file: config/workcell.yaml
      mode: sequential
      retries: 1
      on_failure: abort

    - name: "Pick/place shear links"
      executables:
        - name: shear_links_pp
          parameters:
            workcell_config_package: sa2_description
            workcell_config_file: config/workcell.yaml
      mode: sequential
      retries: 1
      on_failure: abort
```

This format selects and schedules executables, but the supplied YAML-driven C++ programs **still load their internal motion stages from the workcell file**. Consequently, adding different pick offsets, a second flange job or a new `motion_steps` block to `ms_*.yaml` will **not automatically change their movements** until the relevant C++ executables and sequence launcher are extended.

### Example: consecutive flange pairs (target architecture; not yet executable)

For a cross-section requiring two flange pairs, the *intended* task sequence is:

```text
1. flange_pp: Robot 3 picks rebar_3c; Robot 4 picks rebar_4c
2. flange_pp: Robot 3 picks rebar_3d; Robot 4 picks rebar_4d
3. shear_links_pp: perform shear-link placement
4. ... continue assembly steps
```

Each `flange_pp` invocation should receive its own object IDs, pick/place frames, offsets and motion-step data **from the selected `ms_*.yaml`** while sharing the same physical `workcell.yaml`. The new multi-job argument/step-passing support has **not** been implemented in the current `flange_pp.cpp` refactor. The existing version cannot achieve this merely by inserting two identical `flange_pp` entries into the sequence.

### Execution modes and failure behavior

- `mode: sequential` executes a stage's executables in order; `mode: parallel` requests concurrent execution of the executables listed within that stage. A stage with **one** combined dual-arm executable still has one process, regardless of `mode`.
- `retries` is the **total number of attempts**, including the first attempt.
- `on_failure` policies in the reference sequence include `abort`, `continue`, `final_home`, and `finish` (verify exact handling in the installed sequence launcher).
- Internal dual-arm coordination, attachments and collision-allowance transitions are implemented by the respective C++ programs, not created automatically by `mode: parallel`.

## Switching cross-sections

1. Keep `sa2_description/config/workcell.yaml` as the common workcell definition.
2. Create another assembly sequence in `sa3_motion/config/`, for example `ms_3_re.yaml`.
3. Define the required task order, objects, robot operations and eventual motion steps in that sequence.
4. Build/source the workspace so the new sequence is installed, then select the new `sequence_config` at launch.
5. Validate that all referenced TF frames, scene object IDs and tool states exist in the common workcell.

**Do not rename `workcell1.yaml` over `workcell.yaml` just to change assembly type.** A different physical cell layout would be a separate case and could warrant a different workcell file, but different assembly workflows alone should not.

## Run a specific executable

For an executable that accepts a ROS parameters file:

```bash
ros2 run sa3_motion <node_name> --ros-args \
  --params-file ~/ros_ws/sa3_j/src/src/sa3_motion/config/move_grouph.yaml
```

Replace `<node_name>` with the executable's name; the parameters file must exist and match that executable's expectations. Starting an individual motion node requires MoveIt, controllers and the relevant planning scene to be ready.

## Hardware bringup (example from the reference)

```bash
ros2 launch sa2_moveit_config sa3_bringup.launch.py \
  robot1_hardware:=mock \
  robot2_hardware:=mock \
  robot3_hardware:=fanuc \
  robot3_ip:=192.168.2.3 \
  robot4_hardware:=mock
```

This is the example configuration from the reference README, **not a verified hardware connection setup**. Confirm the actual controller mode, device IP, network, robot safety interlocks and permitted workcell state before commanding real motion.

## MoveIt trajectory start tolerance

If needed, for a running MoveIt `move_group` node:

```bash
ros2 param set /move_group \
  trajectory_execution.allowed_start_tolerance 0.02
```

This changes the allowed difference between the expected trajectory **start state** and the robot's current state; it is **not** a general TCP goal-position or collision tolerance. For revolute joints, 0.02 rad is approximately 1.15 degrees; for prismatic joints, position is measured in metres. Check whether your ROS/MoveIt configuration accepts runtime changes and whether the tolerance is appropriate for the robot; relaxing it should not mask calibration or state-feedback errors.

## Known limitations and migration status

- **Architecture migration incomplete:** The supplied YAML-driven executables still rely on `workcell.planner_sequence` for motion steps. Separating those steps into `ms_*.yaml` requires updates to the launcher and C++ parsing/execution code. This README documents both the **target design** and the **current behavior**; it does not imply that those code changes have been completed.
- **Alternate cross-section picking:** The original reference identifies this as unfinished. The proposed sequence-only change of cross-section has not been validated end-to-end.
- **Tying-gun geometry:** The reference reports incorrect tying-gun STL meshes causing trajectory failures; inspect and correct meshes, TF and collisions.
- **Tying-gun action:** The refactored `tying_rebars.cpp` moves the tying tool to the requested points but does not implement physical tying actuation.
- **Coordinated trajectories:** A merged dual-arm execution must be checked for cross-arm collisions and synchronization, not just successful independent-arm planning.
- **Testing:** The previous refactor was statically checked but not compiled or run against your ROS 2 / MoveIt installation. Test first in simulation and verify planning-scene operations and failure recovery before hardware use.

**Status:** Under development. One shared workcell with per-cross-section motion YAMLs is the intended architecture; full implementation and validation are still required.

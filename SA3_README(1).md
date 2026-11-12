# SA3 Robot Assembly Yaml

ROS 2 + MoveIt 2 simulation workflow for the SA3 four-robot, gantry-assisted rebar assembly workcell.

**Start here:** All assembly scheduling, reusable actions, steps, target overrides, gantry commands, and planner settings are edited in **`sa3_execution_sequence.yaml`**. There is **no separate action-library YAML in this workflow**. Physical frame positions and robot/workcell configuration remain in `sa2_description/config/workcell.yaml`.

This README is intended for a developer who did not create the project. It explains how to bring up the simulation, run the assembly, read and change the YAML, add new steps without adding C++ task nodes, and investigate failures. Its organization follows the earlier SA1 developer README, adapted to SA3's generic action executor.

> **Simulation prototype only.** The supplied C++ and launch files explicitly restrict execution to confirmed simulation. Editing a YAML file does not automatically make a path collision-free or safe for hardware. Validate against the actual workcell before enabling motion.

## 1. Architecture: what controls what?

| Component | Role | How it is controlled |
| --- | --- | --- |
| `robot1` | Left shear-link handling, Schunk tool exchange, left tying | MoveIt 2 |
| `robot2` | Right shear-link handling, tool exchange, right tying | MoveIt 2 |
| `robot3` | Left flange and bottom-mat rebar handling | MoveIt 2 |
| `robot4` | Right flange and coordinated bottom-mat motions | MoveIt 2 |
| Two-axis gantry | Pick, lift, transport and lower `rebar_1A` | Existing `/gantry_controller/follow_joint_trajectory` action, via a gantry bridge |

**Read the system from outside to inside:**

```text
sa3_execution_sequence.yaml              THE ONLY MOTION/SEQUENCE YAML
│
├── sa3_sequence                       1. WHICH jobs execute? In what order?
     └── stages[]                      2. One stage, sequential or parallel
     └── actions[]                     3. Call a reusable action + with: inputs
                         ▼
sa3_motion_sequence_yaml.launch.py   Stage scheduling, processes, failures
├── sa3_generic_executor.cpp          Resolves action steps, MoveIt/scene
└── gantry_controller.cpp             Bridge to existing gantry controller

sa2_description/config/workcell.yaml   Physical frames, objects, links, gantry joints
```

The `action_library:` section is **inside the same execution-sequence file**. You will see `action_library.actions` as a YAML *path*, but it is **not another YAML file**. For example, `action_library.actions.flange_pp.steps` means “scroll to the `action_library:` section at the bottom of `sa3_execution_sequence.yaml`, then find `actions:` → `flange_pp:` → `steps:`.”

Unlike SA1's task-specific C++ programs, the SA3 launch starts a generic executor process for each action invocation. It is the same executable reused with different action names and `with:` values; a process is not a separate hardcoded motion program.

## 2. Repository files and placement

The checked-out repository must contain the following ROS packages. This is the relevant layout; additional description, MoveIt, and controller files will exist in your full repository.

```text
~/ros_ws/sa3_ws/
└── src/
    ├── sa3_motion/
    │   ├── config/
    │   │   └── sa3_execution_sequence.yaml     <-- EDIT THIS FOR MOTION
    │   ├── launch/
    │   │   └── sa3_motion_sequence_yaml.launch.py
    │   └── src/
    │       ├── sa3_generic_executor.cpp
    │       └── gantry_controller.cpp
    ├── sa2_description/
    │   └── config/
    │       └── workcell.yaml                   <-- PHYSICAL FRAME POSITIONS
    └── sa2_moveit_config/
        └── launch/
            └── sa3_bringup.launch.py           <-- SIMULATION STARTUP
```

| File | Responsible for | Typical edit |
| --- | --- | --- |
| `sa3_execution_sequence.yaml`, first section (`sa3_sequence`) | Overall assembly order, stage enable/disable, action calls | Reorder jobs, choose rebar, choose frames, pass offsets |
| **Same** `sa3_execution_sequence.yaml`, second section (`action_library`) | Steps performed by named actions, motion types, speeds, collisions and gantry commands | Add a waypoint, change LIN/PTP, attach/detach timing |
| `workcell.yaml` | Workcell TF frames, physical objects, robot links and gantry controller configuration | Correct a measured pickup-frame XYZ/RPY |
| `sa3_motion_sequence_yaml.launch.py` | Reads **both sections of the same YAML** and schedules action processes | Change orchestration behavior or defaults |
| `sa3_generic_executor.cpp` | Generic interpreter of YAML operations | Only when adding a genuinely new operation/feature |
| `gantry_controller.cpp` | Gantry transport bridge and validation | Only when changing low-level gantry behavior |

> **Important for existing checkouts:** The originally supplied launch file read a separate action-library file. The **updated launch file included with this README** reads both `sa3_sequence:` and `action_library:` from `sequence_config`, and passes that one path to the C++ executor. Use the updated launch when following this README. An old installed launch will not automatically switch to the single-file behavior.

## 3. Build and launch: first-time use

These commands assume your workspace is `~/ros_ws/sa3_ws`. Replace the path if your repository is elsewhere. ROS 2 Humble and Jazzy are identified in the supplied executor comments; use the version actually installed on your machine.

### 3.1 Source ROS and build

```bash
cd ~/ros_ws/sa3_ws
source /opt/ros/humble/setup.bash
# For a Jazzy installation, use: source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Start each **new terminal** with:

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/sa3_ws/install/setup.bash
```

Check that the executables are installed:

```bash
ros2 pkg executables sa3_motion
```

You should find `sa3_generic_executor` and `gantry_controller`. Their build definitions (`CMakeLists.txt`, `package.xml`) were not included in the uploaded sources, so confirm the names against your repository if the command does not find them.

### 3.2 Terminal 1 — launch the SA3 simulation

**Run the workcell simulation/MoveIt environment first:**

```bash
ros2 launch sa2_moveit_config sa3_bringup.launch.py
```

This is the SA3 simulation command supplied for this project. Leave it running while you execute the assembly in another terminal. It should provide the running robot descriptions, joint feedback, TFs, controllers and MoveIt environment required by the motion launch. The `sa3_bringup.launch.py` source was not among the uploaded files, so its detailed internal launch options have not been independently checked here.

Do **not** start a second bring-up stack for the same simulation; duplicate controllers, `move_group` instances or TF publishers can cause confusing errors.

### 3.3 Terminal 2 — check YAML, then run the assembly

Source ROS and the workspace in Terminal 2. For a safe static configuration check:

```bash
ros2 launch sa3_motion sa3_motion_sequence_yaml.launch.py dry_run:=true
```

The launch reads the two sections of **`sa3_execution_sequence.yaml`**, validates action references and missing `{{...}}` placeholders, and exits without commanding robot motion. This is **not** a collision test or trajectory simulation.

To run all **enabled** assembly stages in the simulation:

```bash
ros2 launch sa3_motion sa3_motion_sequence_yaml.launch.py simulation_confirmed:=true
```

The launch starts the gantry bridge, runs the first enabled stage, and continues to the next stage after the current one succeeds. A failed action stops the sequence; there is no automatic resume from the last successful step.

### 3.4 Use a YAML file directly from your source checkout

The launch defaults to the installed file in `sa3_motion/config`. You can override it with **one YAML path**:

```bash
ros2 launch sa3_motion sa3_motion_sequence_yaml.launch.py \
  sequence_config:=$HOME/ros_ws/sa3_ws/src/sa3_motion/config/sa3_execution_sequence.yaml \
  dry_run:=true
```

Then run the same path with `simulation_confirmed:=true` and without `dry_run:=true` after reviewing the planned changes.

To override physical workcell geometry as well:

```bash
ros2 launch sa3_motion sa3_motion_sequence_yaml.launch.py \
  sequence_config:=$HOME/ros_ws/sa3_ws/src/sa3_motion/config/sa3_execution_sequence.yaml \
  workcell_config_path:=$HOME/ros_ws/sa3_ws/src/sa2_description/config/workcell.yaml \
  simulation_confirmed:=true
```

**There is no `action_library:=...` argument in the updated single-YAML launcher.** It always reads the library embedded in the specified sequence file.

### 3.5 Available launch arguments

| Argument | Default | What it changes |
| --- | --- | --- |
| `sequence_config` | Installed `sa3_motion/config/sa3_execution_sequence.yaml` | Path to the **one YAML file** containing both top-level sections |
| `dry_run` | `false` | Static preflight only; no robot/gantry motion |
| `simulation_confirmed` | `true` | Simulation-execution gate; **not** a hardware safety approval |
| `workcell_config_path` | Empty | Optional override for `sa2_description/config/workcell.yaml` |
| `gantry_horizontal_min_m` / `gantry_horizontal_max_m` | `-10.6` / `10.6` | Gantry X joint limits in metres |
| `gantry_vertical_min_m` / `gantry_vertical_max_m` | `0.0` / `6.6` | Gantry Z joint limits in metres |

**Stopping:** `Ctrl+C` requests shutdown. If the launch stops while a controller is active, check the live robot and gantry states. A cancel request is not confirmation that every physical axis has stopped.

## 4. The YAML editing guide — the most important section

### 4.1 Exactly which part of the single YAML should you edit?

| I want to change… | Edit inside `sa3_execution_sequence.yaml` | Other requirements |
| --- | --- | --- |
| The **order** of assembly jobs | `sa3_sequence.stages` | Preserve dependency ordering |
| Which jobs run together | Stage `mode` and `actions` | Verify non-interference for `parallel` |
| Enable or skip tying/final assembly | Stage `enabled` | Check the robot's state before resuming |
| One rebar name or pick/place location | That call's `with:` | Object ID and TF frame must exist |
| One invocation's pickup/placement offset | That call's `with.pick_a_offset`, etc. | The action must reference the placeholder |
| Add/remove an **entire** reusable action | Stage `actions` list | Use a defined action name |
| Add a **new waypoint or step within an action** | `action_library.actions.<name>.steps` **in the same file** | The shared change affects all callers |
| Switch MoveJ/LIN/PTP/CIRC for a step | That step's `motion`, `strategy` | Planner and strategy must be compatible |
| Change planning speed/retries | `action_library.planning` | Arm speeds use scaling factors `(0, 1]` |
| Change a gantry destination | `with.gantry_*` or action step `x` / `z` | Physical bounds and joint feedback |
| Change attachment/collision sequence | Action step `before`, `after`, `attach`, `detach`, ACM operations | Preserve ownership and allowed-collision cleanup |
| Move a physical pick/place/tie frame | **`workcell.yaml`** | Restart corresponding TF/workcell publishers |

### 4.2 The actual hierarchy: sequence → action → step → motion

The **first** YAML section calls reusable actions:

```yaml
sa3_sequence:
  schema_version: 1
  stages:
    - name: Gantry transfer and flange 3C/4C
      mode: parallel
      verified_non_interference: true
      actions:
        - action: gantry_pick_transfer
          with:
            rebar: rebar_1A
            gantry_pick_z: 4.1
            gantry_transport_x: -6.0
        - action: flange_pp
          with:
            pick_a: 3c
            pick_b: 4c
            object_a: rebar_3c
            object_b: rebar_4c
            # The real invocation has additional required frames/parameters.
```

The **second** section, farther down **in the same YAML file**, defines how a called action runs:

```yaml
action_library:
  schema_version: 1
  planning:
    planning_time: 15
    planning_attempts: 10
    acceleration: 0.45
    moveJ:
      velocity: 0.5
      retries: 3
    moveL:
      velocity_scales: [0.5, 0.45, 0.4, 0.35, 0.3, 0.25, 0.2, 0.15, 0.1, 0.05]
    movePTP:
      velocity_scales: [0.5, 0.45, 0.4, 0.35, 0.3, 0.25, 0.2, 0.15, 0.1, 0.05]
    moveC:
      velocity_scales: [0.5, 0.45, 0.4, 0.35, 0.3, 0.25, 0.2, 0.15, 0.1, 0.05]
  actions:
    flange_pp:
      defaults:
        robot_a: robot3
        robot_b: robot4
        group: robot3_robot4_arms
        pick_a_offset: [0, 0, 0]
        pick_b_offset: [0, 0, 0]
      steps:
        - op: move
          id: P1
          motion: movePTP
          strategy: separate_merge
          group: '{{group}}'
          # Full participants/goals fields are in the real file.
```

These examples are **abbreviated explanations**, not complete drop-in replacements. Do not delete existing keys when copying or editing a section.

**How the substitution works:** In the `flange_pp` call, `with.pick_a: 3c` overrides any `defaults.pick_a`. When a step contains `reference: '{{pick_a}}'`, the executor substitutes `3c` before resolving its TF pose. `object_a: rebar_3c` is a **planning-scene object ID** for attachment, **not** the target TF name `3c`.

`defaults:` makes reusable actions possible; `with:` supplies the values that differ between the 3C/4C and 3D/4D runs. The executor reads the action steps in list order.

### 4.3 Stage fields in `sa3_sequence`

| Field | Meaning | Example |
| --- | --- | --- |
| `name` | Human-readable stage description shown in logs | `Reverse transfer and flange 3D/4D` |
| `mode` | `sequential` or `parallel` for **action calls** in this stage | `mode: parallel` |
| `enabled` | `false` skips the stage; omitted means enabled | `enabled: false` |
| `verified_non_interference` | Required assertion for enabled parallel stages by this launcher | `true` after testing |
| `actions` | Ordered list of calls to reusable action definitions | `- action: flange_pp` |
| `action` | Exact action name in the embedded `action_library.actions` map | `shear_pair` |
| `with` | Invocation-specific input values | `pick_a: 14pf` |

**Two different kinds of concurrency:** `mode: parallel` runs *multiple action invocations* concurrently. `strategy: separate_merge` runs multiple robot tracks inside *one move step*, constructing one merged trajectory. These are not interchangeable. The launch's `verified_non_interference: true` flag is a required assertion, **not** an automatic collision-proof certificate.

### 4.4 Current assembly stage order

The uploaded combined YAML has **10 stages**; stages 1–7 are enabled, and stages 8–10 are disabled.

| # | YAML stage | Purpose | Active? |
| --- | --- | --- | --- |
| 1 | Initialize attached Schunk tools | Attach Robot1/2 Schunk tool collision objects | Yes |
| 2 | Gantry transfer and flange 3C/4C | Gantry transports `rebar_1A` while Robot3/4 handle flanges **in parallel** | Yes |
| 3 | Reverse transfer and flange 3D/4D | Reverse Robot3/4 transfer, then second flange pair | Yes |
| 4 | Four-arm flange home plus first shear approach | Robot3/4 home while Robot1/2 approach shear pickup | Yes |
| 5 | Shear links paired pick/place 8 ticks | Eight `shear_pair` calls for Robot1/2 | Yes |
| 6 | Final unpaired Robot1 shear link | One `shear_single` call | Yes |
| 7 | Bottom mat Robot3/4 + Robot1/2 gripper change fused | Bottom-mat placement plus change to tying tools | Yes |
| 8 | Tying all shear links | 17 paired plus 2 single tying calls | **No** (`enabled: false`) |
| 9 | Tying bottom mat | Three paired tying positions | **No** |
| 10 | Pick and place final assembly | Final assembly action | **No** |

The names above describe the **sequence currently configured**, not evidence of a successfully completed run. Disabled stages will not execute even if the rest of the simulation starts normally.

### 4.5 Available reusable action names

All these actions are defined in the **embedded `action_library` section of `sa3_execution_sequence.yaml`**:

| Action name | What it does | Key required `with:` inputs without defaults |
| --- | --- | --- |
| `initialize_schunk_grippers` | Scene attachments for Robot1/2 tools | None |
| `gantry_pick_transfer` | Gantry G1 home → G2 pick → G3 lift → G4 transport | None |
| `flange_pp` | Dual Robot3/4 pick, transfer, place, optional home | `approach_a/b`, `pick_a/b`, `object_a/b`, `transfer_start_a/b`, `transfer_end_a/b`, `place_a/b` |
| `flange_reverse_pp` | Reverse Robot3/4 transfer path | `approach_a/b`, `transfer_start_a/b`, `transfer_end_a/b` |
| `four_arm_home_shear_approach` | Robot3/4 home, Robot1/2 preposition | `shear_pick_a`, `shear_pick_b` |
| `shear_pair` | Robot1/2 paired shear-link handling | `pick_a/b`, `place_a/b`, `object_a/b` |
| `shear_single` | Last single Robot1 shear-link transfer | `pick`, `place`, `object` |
| `bottom_mat_pp` | Independent bottom-mat workflow | `pick_a/b`, `place_a/b` |
| `gripper_change` | Tool change for Robot1/2 | None |
| `bottom_mat_and_gripper_change` | Fused bottom-mat and tool-change workflow | `pick_a/b`, `place_a/b` |
| `tying_pair` | Robot1/2 tying at two TF references | `tie_a`, `tie_b` |
| `tying_single` | Robot1 single tying target | `tie_a` |
| `final_assembly` | Final assembly pick/place motions | None |

These inputs are taken from the action definitions and their placeholders. They do **not** guarantee that frames, objects, tools or controllers will be present during runtime.

### 4.6 How a `steps:` list works

Each entry has `op:` (what to do) and a descriptive `id:` (which step it is). A `move` step can contain `before:` and `after:` operation lists:

```yaml
- op: move
  id: P2
  motion: moveL
  strategy: separate_merge
  group: '{{group}}'
  participants: ['{{robot_a}}', '{{robot_b}}']
  goals:
    - robot: '{{robot_a}}'
      target:
        reference: '{{pick_a}}'
        offset_frame: local
        xyz: '{{pick_a_offset}}'
        rpy: '{{pick_a_rpy}}'
        rpy_frame: local
    - robot: '{{robot_b}}'
      target:
        reference: '{{pick_b}}'
        offset_frame: local
        xyz: '{{pick_b_offset}}'
        rpy: '{{pick_b_rpy}}'
        rpy_frame: local
  before:
    - op: acm_allow
      id: P2_allow_grip
      pairs: [['{{object_a}}', '{{tool_link_a}}']]
  after:
    - op: attach
      id: P2_attach_a
      object: '{{object_a}}'
      link: robot3_end_effactor_ff
      placement_mode: preserve_world
```

This is a **simplified illustration** of the first flange pickup; the real step has more collision permissions and attaches both rebar objects. `before` runs before the motion; `after` runs only if the motion succeeds. Moving an `attach` or `detach` to the wrong point changes what the collision scene believes the robot is carrying.

### 4.7 Supported `op` types

| `op` | Purpose | Important fields |
| --- | --- | --- |
| `move` | Plan and execute arm motion using MoveIt | `id`, `motion`, `strategy`, `group`, `goals`, sometimes `participants` |
| `attach` | Attach an **existing collision object** to a robot/tool/gantry link | `object`, `link`, `placement_mode`, optional `touch_links`, `dock_frame` |
| `detach` | Detach object back to the world planning scene | `object`, optional owning `link` |
| `acm_allow` | Temporarily permit specified contact pairs | `pairs: [[name1, name2], ...]` |
| `acm_restore` | Restore the original permission for matching pairs | `pairs` |
| `gantry_send` | Send gantry X and Z target | `id`, `x`, `z`, `velocity`, `acceleration`, optional `wait`, `timeout` |
| `gantry_wait` | Wait for an earlier asynchronous gantry command | Matching request key; not needed for the current synchronous YAML |
| `sleep` | Pause a specified number of seconds | `seconds` |

`attach`/`detach` only change **MoveIt's planning scene**. They do **not** command the physical gripper to open or close. `acm_allow` is for a deliberate, verified contact between named bodies—not a method for ignoring unexpected collision errors.

**Placement modes:** `preserve_world` attaches without changing the object's current world pose. `dock_to_mount` uses a supplied `dock_frame` to maintain the tool's dock-relative placement when it is moved to its mount. Verify that the dock TF and tool geometry are correct.

### 4.8 Motion type, robot groups, and strategies

| `motion` | Planner | Typical use |
| --- | --- | --- |
| `moveJ` | OMPL / joint-space planning | Joint moves, home, large transfers |
| `moveL` | Pilz LIN | Straight Cartesian approach/retraction |
| `movePTP` | Pilz PTP | Joint-space point-to-point with Pilz |
| `moveC` | Pilz CIRC | Circular transfer via a specified intermediate point |

| `strategy` | Meaning | Supported use in supplied C++ |
| --- | --- | --- |
| `separate_merge` | Plan moving robots individually and time-align/merge tracks into one group trajectory | `moveJ`, `moveL`, `movePTP`, `moveC` |
| `combined` | Plan a full multi-robot group together | `moveJ` |
| `coordinated_cartesian` | Compute an interpolated simultaneous Cartesian multi-robot path | `moveL` |

Examples of SRDF group names used in the YAML are `robot3_robot4_arms`, `sa3_robots_1_2` and `sa3_all_robots`. The group has to exist in `sa2_moveit_config`; adding a name to YAML does not create a new MoveIt group. `participants` in a `separate_merge` step specifies the robots whose joints are represented in the merged trajectory, including a stationary holding arm when needed.

A step may set `fallbacks` if its primary planner fails. For example, the provided shear-link motion attempts a LIN transit, then a CIRC alternative using `circ_via_world_x`, and then an OMPL joint move. Planning fallbacks do **not** mean that a failed or interrupted physical execution is automatically retried.

### 4.9 Target positions and orientation

A robot goal can specify a named SRDF state (`named_state: home`), or a Cartesian target relative to a TF:

```yaml
goals:
  - robot: robot3
    link: robot3_end_effactor_ff
    target:
      reference: '{{pick_a}}'
      offset_frame: local
      xyz: [0.0, 0.0, 0.0]
      rpy: [0.0, 0.0, 0.0]
      rpy_frame: local
```

| Field | Interpretation |
| --- | --- |
| `robot` | Configured logical robot name (`robot1`–`robot4`) |
| `link` | Tool/planning link to position (if omitted, uses workcell robot default) |
| `target.reference` | Existing TF frame, resolved against `world` |
| `target.xyz` | `[x, y, z]` in **metres** |
| `target.offset_frame: world` | Apply XYZ along the world X/Y/Z axes |
| `target.offset_frame: local` | Rotate XYZ by the target frame orientation first |
| `target.rpy` | `[roll, pitch, yaw]` in **radians** |
| `target.rpy_frame` | Apply RPY in `local` or `world`; default is `local` |
| `target.base_xyz` | Extra translation before the step's XYZ (used in flange retraction) |
| `target.base_offset_frame` | World/local axes for `base_xyz` |

For example, `xyz: [0.1, 0, 0]` with `offset_frame: world` means +100 mm **along the world X axis**. The same numbers with `offset_frame: local` mean +100 mm along the *rotated local X axis of the reference frame*. A negative value means the negative direction of the chosen axis; it does **not** automatically mean a safe retract.

### 4.10 Planning speeds, retries, and collision checking

Global settings are within **the same file**, under `action_library.planning`:

```yaml
action_library:
  planning:
    full_cell_collision_check: false
    preview_rejected_trajectories: false
    planning_time: 15
    planning_attempts: 10
    acceleration: 0.45
    moveJ:
      velocity: 0.5
      retries: 3
    moveL:
      velocity_scales: [0.5, 0.45, 0.4, 0.35, 0.3, 0.25, 0.2, 0.15, 0.1, 0.05]
    movePTP:
      velocity_scales: [0.5, 0.45, 0.4, 0.35, 0.3, 0.25, 0.2, 0.15, 0.1, 0.05]
    moveC:
      velocity_scales: [0.5, 0.45, 0.4, 0.35, 0.3, 0.25, 0.2, 0.15, 0.1, 0.05]
    fallbacks:
      movePTP: [moveJ]
```

The values shown match the supplied combined YAML. For arms, velocity and acceleration values are **scaling factors in `(0, 1]`**. A list of scales is tried in order while planning; for example, lower PTP scales may be attempted if an earlier plan fails. They are not absolute joint speeds and do not reduce a moving robot's speed dynamically. The C++ also supports compatible per-step planner settings and fallbacks.

`full_cell_collision_check: false` is the **current uploaded setting**. That disables the executor's extra full-cell trajectory sampling pass; it does **not** turn off every MoveIt collision check. Enable and verify full-cell validation for appropriate multi-robot safety testing rather than assuming the flag is safe to leave off.

### 4.11 Gantry commands: **only `id`, no YAML `request_id`**

The combined YAML uses synchronous gantry commands. A representative real step:

```yaml
- op: gantry_send
  id: G4_transport
  x: '{{gantry_transport_x}}'
  z: '{{gantry_up_z}}'
  velocity: 0.45
  acceleration: 0.5
```

Under the **updated executor included with this README**:

- `id` is the step's readable label and is used to make the internal gantry request key. The executor prefixes the current action instance to avoid collisions between repeated actions.
- `wait` **defaults to `true`**. The next step starts only after a successful gantry completion response. You can omit `wait: true` from every current gantry step.
- `timeout` defaults to **120 seconds** for completion waiting when omitted. It is a maximum wait, **not** the planned travel time and not a motion slowdown.
- A request failure or completion timeout aborts that action. Verify actual joint positions before retrying.
- The controller bridge still transmits an internal protocol field named `request_id`; **users do not write that field in the single YAML**.

The current combined YAML has no `gantry_wait` steps; `G6` also waits synchronously. This differs from earlier asynchronous examples. If you intentionally add `wait: false`, you must also design the later completion wait and verify that another gantry command is not sent while the first is running. The current bridge rejects a second concurrent gantry motion.

**Gantry units and limits:** `x`/`z` are metres, and the bridge accepts the configured default range X `[-10.6, 10.6]` and Z `[0, 6.6]`. It requires positive velocity/acceleration with `velocity <= 2.0` and `acceleration <= 5.0`. Unlike arm `velocity_scales`, gantry `velocity`/`acceleration` are values the bridge uses to derive a trajectory duration; they are not MoveIt scaling factors.

**Existing issue in an inactive action:** The uploaded `bottom_mat_pp` action (not the enabled fused `bottom_mat_and_gripper_change` action) has a `G5_lower` gantry `velocity: 2.45`, above the bridge's limit. Reduce and validate it before choosing that standalone action. A YAML syntax check or static preflight will not catch the gantry speed limit.

## 5. Practical examples: edit the YAML without writing task C++

### A. Change a specific flange pickup

Find stage `Reverse transfer and flange 3D/4D` in the top `sa3_sequence:` section. Its second action is `flange_pp`. The current uploaded `with:` values include:

```yaml
pick_a: 3d
pick_b: 4d
object_a: rebar_3d
object_b: rebar_4d
pick_a_offset: [0.127, -0.045, 0.008]
pick_b_offset: [-0.155, -0.057, -0.01]
do_approach: false
```

Edit only the measured offset needed for that particular part. Because the action uses `{{pick_a_offset}}`, this affects this **invocation**, not the earlier 3C/4C call. Keep the other required `with:` fields intact. `do_approach: false` skips the optional P1 move only for this call.

### B. Add another paired shear-link placement

In the `Shear links paired pick/place 8 ticks` stage, copy a complete existing `- action: shear_pair` block and insert it in the desired order. Change its `with:` values:

```yaml
- action: shear_pair
  with:
    pick_a: 5pf
    place_a: 5_placef_l
    object_a: rebar_5a
    pick_b: 6pf
    place_b: 6_placef_r
    object_b: rebar_6
    pick_a_offset: [0, 0, 0]
    pick_b_offset: [0, 0, 0]
    first_approach: true
```

This example **duplicates an existing pair** to show the syntax—it is **not** a valid new part. Replace the six names with the next real left/right pick frames, place frames, and object IDs, ensuring those objects exist. The same shared `shear_pair` steps are reused; no new C++ action is needed.

### C. Add a waypoint inside flange motion

Scroll to the **embedded** `action_library:` section of `sa3_execution_sequence.yaml`, find `actions:` → `flange_pp:` → `steps:`, and insert the following between the current transfer steps `T1` and `T2`:

```yaml
- op: move
  id: T1_extra_via
  motion: movePTP
  strategy: separate_merge
  group: '{{group}}'
  participants: ['{{robot_a}}', '{{robot_b}}']
  goals:
    - robot: '{{robot_a}}'
      target:
        reference: '{{via_a}}'
        offset_frame: world
        xyz: [0, 0, 0]
        rpy: [0, 0, 0]
    - robot: '{{robot_b}}'
      target:
        reference: '{{via_b}}'
        offset_frame: world
        xyz: [0, 0, 0]
        rpy: [0, 0, 0]
```

Then add real `via_a` and `via_b` frame names to **each `flange_pp` call's** `with:` mapping. For example, `via_a: r3_ptp_start` references an existing frame, but an actual *new* waypoint needs its own verified TF frame. Missing placeholders fail static preflight.

**Shared-template rule:** Editing `flange_pp.steps` affects **both** 3C/4C and 3D/4D flange actions. That is why parameterizing the new references with `{{via_a}}` and `{{via_b}}` is usually better than hardcoding a frame inside the shared step.

### D. Change a motion type or speed

To change the transfer between `T1` and `T2`, edit the corresponding step under `action_library.actions.flange_pp.steps`:

```yaml
- op: move
  id: T2
  motion: movePTP           # Pilz PTP; alternative: moveJ for OMPL joint planning
  strategy: separate_merge
```

Preserve the existing `group`, `participants`, and `goals`. To change its global fallback speed schedule, edit `action_library.planning.movePTP.velocity_scales`. For a single step override, verify the specific step setting supported by the executor instead of assuming every invented YAML key is recognized.

### E. Change a gantry X/Z destination

In the **top** `sa3_sequence:` section, find `Gantry transfer and flange 3C/4C` → `gantry_pick_transfer` → `with:`. Its current target override is:

```yaml
with:
  rebar: rebar_1A
  gantry_pick_z: 4.1
  gantry_transport_x: -6.0
```

Changing `gantry_transport_x` changes the target of the action's `G4_transport` step (because its `x` contains `{{gantry_transport_x}}`). For speed changes, find that `G4_transport` step in the **embedded** action definition. Do not add `request_id`; `id: G4_transport` is sufficient with the updated C++.

### F. Enable one of the disabled tying stages

Find the top-level stage named `Tying all shear links (19 Robot1 / 17 Robot2 positions)` and change:

```yaml
enabled: false
```

to:

```yaml
enabled: true
```

The sequence has 17 paired `tying_pair` invocations and two Robot1 `tying_single` invocations. Before enabling, verify that the tool-change action succeeded, the correct tool is attached in the planning scene, and all tie TFs exist.

### G. Change stage ordering or run two actions together

To change job order, move **entire** stage mappings under `sa3_sequence.stages`. To run complete action invocations at the same time, put both calls in one stage with:

```yaml
- name: Example overlap (only after verifying clearance)
  mode: parallel
  verified_non_interference: true
  actions:
    - action: gantry_pick_transfer
      with:
        rebar: rebar_1A
    - action: flange_pp
      with:
        # Copy the FULL valid list of flange parameters from the real stage.
```

The example is abbreviated. Parallel execution does **not** guarantee collision safety, scheduling checkpoints, or mutual exclusion over shared objects. Only use it for actions proven not to interfere with each other.

### H. Change a physical pickup frame

In `sa2_description/config/workcell.yaml`, the bottom-mat left pick frame currently reads:

```yaml
workcell:
  inventory_frames:
    bm_pkL:
      parent: world
      child: bm_pkL
      xyz: [0.420, 1.864, 4.116]
      rpy: [-0.571, -0.007, 1.573]
```

The workcell `xyz` is the **pose of the frame relative to `parent`**, not a per-step offset. Edit the workcell when the actual target location is wrong; edit an action's `target.xyz` or `with.*_offset` when only one approach/retract should change. Restart the environment's relevant TF publishers so the new geometry actually becomes visible to MoveIt.

## 6. `workcell.yaml`: geometry, frames, and robot identities

The workcell file is the other important editable YAML, but it is **not** a separate motion/action library. Its roles are:

| Section | What it defines | Examples in the uploaded file |
| --- | --- | --- |
| `web_ring_frames` / `web_ring2_frames` | Shear-link pickup reference frames | `5pf`, `13pf`, `14pf`, `6pf`, `13_1pf` |
| `inventory_frames` | Flange approaches and bottom-mat pickup references | `inventory_3a_4a_frame`, `bm_pkL`, `bm_pkR` |
| `place_frames` | Rebar placement destinations | `place_frame_10_l`, `place_frame_10`, `bm_pfl`, `bm_pfr` |
| `tying_frames` | Tying positions and final assembly references | `5_placef_l_t1`, `1A_l_t1`, `sa3_pick_point_l` |
| `arc_frames` | Arc and point-to-point transfer references | `r3_ptp_start`, `r4_ptp_end` |
| `tool_dock_frames` | Schunk/tying-gun docking positions | `robot1_shuck_dock`, `robot1_tyinggun_dock` |
| `extra_rebars` | Object meshes, IDs and associated frames | `rebar_3c`, `rebar_4d`, `rebar_13_1` |
| `robots` / `gantry_robot` | Planning groups, links, gantry joints and action name | Read actual values from your installed config |

A TF reference (`3c`) and a planning-scene object name (`rebar_3c`) are separate concepts even when associated with the same physical rebar. A target frame must be visible under `world` for `tfPose()` to resolve it. An object must exist in the world planning scene before `attach` can claim it.

**Coordinate conventions:** XYZ is in metres; RPY is in radians; `parent` defines a workcell frame's reference axes; `offset_frame: local` in a motion step uses the reference pose's rotated axes.

## 7. YAML syntax, variables, and anchors

- **Indentation matters.** Use spaces, do not use tabs for nesting. Each `-` is a list item and its fields must be aligned consistently.
- **`{{name}}`** means interpolate an input from action `defaults:` plus that call's `with:` overrides. Use `{{...}}` only for fields provided by the action.
- **`&anchor_name`** creates a YAML anchor; **`*anchor_name`** reuses the same YAML value/list. Neither is a motion step nor a ROS node. The combined file uses descriptive anchor names for participant/touch-link lists.
- **`# comment`** is never executed. Commenting out a list item changes the sequence and can leave a malformed YAML list if indentation is wrong.
- **`id:`** is the local operation label, such as `P2`, `T1`, `G4_transport`, or `R5`. Use meaningful names; different actions may reuse short IDs because the running action context distinguishes them.
- **`enabled:` on a step** suppresses only that step. **`enabled:` on a stage** skips the whole stage. Do not skip attach/detach or collision cleanup casually.

If you need to change execution order, edit `sa3_sequence.stages`. If you need to add a new movement **inside** a task, edit its embedded `action_library.actions.<action>.steps`. If you need a different robot target, first decide whether it is an invocation offset or a physical frame correction.

## 8. Validate an edit before motion

### 8.1 Check the single YAML's syntax and both required sections

```bash
python3 - <<'PY'
import yaml
path = 'src/sa3_motion/config/sa3_execution_sequence.yaml'
with open(path, encoding='utf-8') as f:
    cfg = yaml.safe_load(f)
assert cfg['sa3_sequence']['schema_version'] == 1
assert cfg['action_library']['schema_version'] == 1
assert isinstance(cfg['sa3_sequence']['stages'], list)
assert isinstance(cfg['action_library']['actions'], dict)
print('Combined SA3 YAML syntax and top-level sections: OK')
PY
```

Run this from the workspace root. This checks parsing/basic shape, **not** safety, TF availability, velocity limits, or trajectory feasibility.

### 8.2 Launch preflight

```bash
ros2 launch sa3_motion sa3_motion_sequence_yaml.launch.py \
  sequence_config:=$HOME/ros_ws/sa3_ws/src/sa3_motion/config/sa3_execution_sequence.yaml \
  dry_run:=true
```

This validates the action calls and their required placeholders; it does not execute robot trajectories or test IK/collisions. It also checks disabled stages for invalid action references, even though disabled stages do not execute.

### 8.3 Bring up simulation, confirm prerequisites, and execute

After running `ros2 launch sa2_moveit_config sa3_bringup.launch.py` in Terminal 1, inspect from Terminal 2:

```bash
ros2 node list | grep move_group
ros2 topic list | grep joint_states
ros2 action list | grep gantry_controller
ros2 service list | grep -E 'get_planning_scene|check_state_validity'
ros2 run tf2_ros tf2_echo world bm_pkL
```

Stop the continuous TF output with `Ctrl+C`. If the required endpoints are present and the planned motions have been reviewed in simulation, run the enabled sequence as in Section 3. Do not run the same assembly sequence twice against one shared environment.

### 8.4 Review a failure before restarting

If an action stops mid-run, check live joint states, attached collision objects, gantry status, which tools are mounted, and which rebars are already placed. The launch does **not** restore the entire workcell to its initial state, and replaying the sequence blindly can make later attachments fail.

## 9. C++ control flow: when does a YAML step become motion?

```text
Launch reads combined YAML once for stage validation
    -> stage 1...N (skip disabled)
    -> launch an sa3_generic_executor process for each action call
    -> executor loads the SAME YAML from action_library_path
    -> resolves defaults + with: inputs
    -> for each step: check enabled, read id/op
        -> move: run before -> plan -> verify -> execute -> run after
        -> attach/detach: edit MoveIt planning-scene object ownership
        -> acm_allow/acm_restore: update and verify chosen collision pairs
        -> gantry_send: publish command and wait for completion by default
    -> action returns success/failure
    -> launcher advances to next stage or shuts down on failure
```

The executor uses `MoveGroupInterface` for robot plans. It chooses OMPL for `moveJ` and the Pilz planner pipeline for `moveL`/`movePTP`/`moveC`. It can plan robot tracks separately and merge them or use the combined group. Gantry motion instead goes through `/sa3/gantry/command` → gantry bridge → existing FollowJointTrajectory server, with status returned on `/sa3/gantry/status`.

The source-only modifications supplied with this documentation are deliberately small: the updated **launch reads one YAML**, and the updated **executor uses the gantry step's `id` and defaults to waiting for completion**. The bridge's internal `request_id` protocol remains intact. No ROS runtime, controller trajectory, or hardware behavior was executed while preparing this README.

## 10. Common errors and how to fix them

| Symptom | Likely cause | What to check |
| --- | --- | --- |
| `Package 'sa2_moveit_config' not found` | Workspace/overlay not sourced or not built | Build, then source ROS and workspace in the **same terminal** |
| `Launch file ... not found` | Installed package does not contain the launch file | Check source placement, install rules, rebuild and resource sourcing |
| `Unknown action library name` | Stage calls an undefined action | Match its `action:` to a name in the **embedded** action definitions |
| `action inputs missing ...` | A step uses `{{name}}` not supplied by `defaults:` or `with:` | Add the required value to each invocation or define a safe default |
| `Gantry command needs request_id` | **Old executor** still installed with combined id-only YAML | Build/source the updated `sa3_generic_executor.cpp` supplied with this guide |
| Gantry does not wait before next step | Old executor uses `wait: false` default | Install updated executor; it defaults to `true` |
| `Gantry already executing` | An earlier gantry move is still running | Do not issue another gantry command; inspect active controller feedback |
| `Invalid gantry target/speed/acceleration` | Nonfinite or out-of-range value | Check gantry X/Z limits and speed cap (`2.0`) |
| `Invalid frame ID` or TF lookup failure | Frame missing/not yet published | Check `workcell.yaml` and `tf2_echo world <frame>` |
| Planning fails or IK rejected | Pose unreachable, collision, planner/strategy mismatch | Inspect specific failed step, link, frame and planning-group configuration |
| `Full-cell collision validation rejected` | One sampled combined state is invalid | Read collision body names; fix geometry, timing, path or intended contact rules |
| `Cannot attach object` / `wrong detach owner` | Scene object absent or attached to another link | Check object ID, tool state and whether the previous action completed |
| Edits to YAML appear ignored | Old installed file or old launch still in use | Pass `sequence_config:=<absolute path>`; check symlink install/overlay |
| Two robot stacks seem to be running | Duplicate bring-up launches | Stop duplicate `move_group`, joint publishers and controller instances |

To inspect the runtime:

```bash
ros2 node list
ros2 action list
ros2 topic echo /sa3/gantry/status
ros2 topic echo /joint_states
ros2 run tf2_ros tf2_echo world r3_ptp_start
```

Review the earliest meaningful error in the ROS terminal log; later errors may simply be consequences of the first failure.

## 11. When to rebuild vs only relaunch

**YAML-only edits:** With a correctly installed `--symlink-install` workspace, changes to `sa3_execution_sequence.yaml` are normally read on the next launch. Stop the existing sequence and relaunch the motion process. For `workcell.yaml` edits, the TF/description publishers may also need restarting. If the installed config is a copy rather than a symlink, rebuild or reinstall.

**C++ or launch edits:** Rebuild `sa3_motion` and re-source the workspace. For this updated single-file workflow, you **must** install the included updated launch and generic executor once:

```bash
cd ~/ros_ws/sa3_ws
colcon build --packages-select sa3_motion --symlink-install
source install/setup.bash
```

Any changes to Xacro, MoveIt SRDF groups, controller mappings, package dependencies, or install rules may require rebuilding their owning packages as well. Do not assume changing YAML creates a new planning group, controller, TF publisher or unsupported `op` type.

## 12. Known limits and safety checklist

The supplied source is a simulation implementation, not a hardware-certified cell controller. Before running any new sequence, check the actual robot/gateway positions, object occupancy, attached tools, published TFs, MoveIt group configuration, controller readiness, and safe clearances. In particular:

- **Only one motion YAML:** the updated launch uses the embedded action definitions in `sa3_execution_sequence.yaml`. An older installed launch/executor can still behave differently; build/source the updated files.
- **Gantry defaults:** the updated executor derives the request key from `id` and waits by default. The bridge still uses an internal request identifier and verifies final joint state within `0.01 m`.
- **Collision safety:** `full_cell_collision_check` is currently `false` in the provided YAML. This reduces an extra validation layer; it is not an assurance that multi-arm motion is safe.
- **Invalid unused step:** the separate `bottom_mat_pp` *action definition inside the combined YAML* has a gantry speed of `2.45`; validate/correct it before adding that action to an enabled stage.
- **Physical grippers:** MoveIt `attach`/`detach` is planning-scene bookkeeping; the uploaded executor does not directly actuate physical gripper hardware.
- **Parallel actions:** the system does not prove that two independent actions are free of interference just because they are grouped in a `parallel` stage.
- **Preflight scope:** `dry_run` checks YAML/action references but does not execute motions, validate all runtime values, or prove safety.
- **Bring-up internals:** the simulation command comes from the project instructions; the body of `sa3_bringup.launch.py` was not among the uploaded source files. Check its behavior in your repository.

## 13. Quick command reference

```bash
# Terminal 1 — simulation environment
source /opt/ros/humble/setup.bash
source ~/ros_ws/sa3_ws/install/setup.bash
ros2 launch sa2_moveit_config sa3_bringup.launch.py

# Terminal 2 — preflight, then execution
source /opt/ros/humble/setup.bash
source ~/ros_ws/sa3_ws/install/setup.bash
ros2 launch sa3_motion sa3_motion_sequence_yaml.launch.py dry_run:=true
ros2 launch sa3_motion sa3_motion_sequence_yaml.launch.py simulation_confirmed:=true

# Useful checks
ros2 pkg executables sa3_motion
ros2 node list | grep move_group
ros2 action list | grep gantry_controller
ros2 topic echo /sa3/gantry/status
ros2 run tf2_ros tf2_echo world bm_pkL
```

**New developer's working rule:** Edit `sa3_execution_sequence.yaml` for sequence **and** action steps. Edit `workcell.yaml` only when the actual workcell geometry/configuration needs to change. Run YAML validation, start the simulation with `sa3_bringup.launch.py`, perform dry-run preflight, and then execute in confirmed simulation after verifying clearances.
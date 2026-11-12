# SA3 Robot Assembly — YAML-Driven Motion Execution

ROS 2 + MoveIt 2 simulation workflow for the SA3 four-robot, gantry-assisted rebar assembly workcell.

This README is written for a developer who did not create the original code. It explains **what each file does, how to install and start the system, how the YAML becomes robot motion, how to change the assembly sequence without rewriting C++, and how to diagnose failures**. It follows the structure of the supplied SA1 README, but documents the **SA3 files supplied with this project**, not SA1's older task-specific motion programs.

> **Status: simulation prototype, not hardware-certified.** The supplied launch file explicitly requires simulation confirmation. Test new motions in RViz / the simulated workcell before execution. The supplied files do not include the complete environment bring-up launch, CMakeLists.txt, or package.xml; commands that depend on those files are identified below.

## 1. System overview

The workcell contains four FANUC robot arms and an independently controlled two-axis gantry:

| Component | Main role in the supplied sequence | MoveIt / control interface |
| --- | --- | --- |
| `robot1` | Left shear-link handling, Schunk-to-tying-tool change, left-side ties | `robot1_arm`, MoveIt 2 |
| `robot2` | Right shear-link handling, tool change, right-side ties | `robot2_arm`, MoveIt 2 |
| `robot3` | Left flange handling and bottom-mat rebar handling | `robot3_arm`, MoveIt 2 |
| `robot4` | Right flange handling; accompanies Robot3 for bottom-mat motions | `robot4_arm`, MoveIt 2 |
| Gantry | Pick, raise, transfer, lower, and retract `rebar_1A` | Existing `/gantry_controller/follow_joint_trajectory` action |

The common MoveIt groups used by the YAML include `robot3_robot4_arms`, `sa3_robots_1_2`, and `sa3_all_robots`. These must exist in the installed MoveIt configuration; naming a group in YAML does **not** create it.

**Main design:** reusable actions are defined once in YAML, called by an ordered execution sequence, and interpreted by one generic C++ executor. The gantry has a separate C++ bridge because it is controlled through an existing FollowJointTrajectory controller rather than the arm MoveIt group.

```text
sa3_execution_sequence.yaml     (WHICH jobs run, and in what order?)
  └── sa3_sequence.stages[]
       └── actions[]: action name + with: parameters
            │
            ▼
sa3_action_library.yaml         (HOW does this named action work?)
  └── action_library.actions.<action_name>
       ├── defaults: fallback input values
       └── steps[]: ordered operations with optional before:/after:
            │
            ▼
sa3_motion_sequence_yaml.launch.py   (orchestration)
  ├── starts a sa3_generic_executor process per action invocation
  │    ├── op: move  ────────────► MoveIt planning + execution
  │    ├── op: attach / detach ──► MoveIt planning scene
  │    ├── op: acm_allow / restore ► allowed-collision matrix
  │    └── op: gantry_send / wait ► ROS 2 topic bridge
  └── starts persistent gantry_controller process
                               └──► existing gantry FollowJointTrajectory

workcell.yaml                  (WHERE are frames, objects, robot links and joints?)
```

**Read order for a newcomer:** (1) execution sequence → (2) named action in action library → (3) action steps → (4) each step's `op`, `motion`, `strategy`, and `goals` → (5) matching TF and robot configuration in `workcell.yaml` → (6) the C++ executor if necessary.

## 2. Packages and file layout

The supplied code references the following ROS packages:

| Package | Purpose |
| --- | --- |
| `sa3_motion` | Generic executor, gantry bridge, action/sequence YAML and orchestration launch |
| `sa2_description` | SA3 workcell definitions, collision objects, frame definitions and meshes; hosts `config/workcell.yaml` |
| `sa2_moveit_config` | `sa3_robots` MoveIt configuration, robot groups, planners, kinematics, controllers |
| Robot description packages | The robot meshes/Xacros required by the workcell; exact package list is defined by the surrounding repository, not by these uploads |

**Recommended source-tree placement** (verify the names against your actual repository):

```text
<ros_workspace>/
└── src/
    ├── sa3_motion/
    │   ├── config/
    │   │   ├── sa3_execution_sequence.yaml
    │   │   └── sa3_action_library.yaml
    │   ├── launch/
    │   │   └── sa3_motion_sequence_yaml.launch.py
    │   └── src/
    │       ├── sa3_generic_executor.cpp
    │       └── gantry_controller.cpp
    ├── sa2_description/
    │   └── config/
    │       └── workcell.yaml
    └── sa2_moveit_config/
        └── ...
```

`sa3_motion` must install the two executables (`sa3_generic_executor`, `gantry_controller`) and the launch/config files. The provided source set does **not** include its `CMakeLists.txt` or `package.xml`, so this README cannot verify that the executables are already registered and installed.

### Which uploaded file is responsible for what?

| File | What you change there |
| --- | --- |
| `sa3_execution_sequence.yaml` | Stage order, which reusable actions run, action parameters, sequential/parallel execution, whether a stage is enabled |
| `sa3_action_library.yaml` | Actual reusable steps, targets/offsets, planning strategies, fallback velocities, object attachments and collision allowances |
| `workcell.yaml` | TF frame locations, pick/place/tie points, objects and meshes, robot links/groups, gantry joint/controller names |
| `sa3_motion_sequence_yaml.launch.py` | ROS launch arguments, action-process lifecycle, stage scheduling, preflight checks and failure shutdown |
| `sa3_generic_executor.cpp` | Interprets `op`, resolves `{{...}}`, plans/executes with MoveIt, updates planning scene and communicates with gantry bridge |
| `gantry_controller.cpp` | Receives gantry requests, validates limits, sends FollowJointTrajectory goals, checks `/joint_states`, publishes result |

### Important: two action-library copies exist in the uploads

The supplied `sa3_execution_sequence.yaml` contains **both** `sa3_sequence:` and an embedded `action_library:`. Separately, you supplied `sa3_action_library.yaml`, which also contains `action_library:`. **The supplied launch loads `sa3_sequence:` from the sequence path and `action_library:` from the separately configured action-library path.** It does not use the embedded library unless you explicitly pass the sequence file as `action_library:=...`.

Therefore:

- **Default workflow:** edit the `sa3_sequence:` section in `sa3_execution_sequence.yaml` and edit reusable action definitions in **`sa3_action_library.yaml`**.
- Do not assume changes under the embedded `action_library:` in the sequence file affect the default execution.
- `default_action_library:` within the sequence file is **not used** by the provided launch code; the launch's `action_library` argument determines the file.
- The embedded library and separate file are **not identical**, especially for gantry commands. Keep a single canonical definition, or explicitly configure which one to load after validating compatibility.

This is an existing source-file inconsistency, not just a documentation convention.

## 3. Prerequisites and first build

You need a Linux ROS 2 workspace with **ROS 2 Humble or Jazzy, MoveIt 2, ros2_control/controllers, TF2, YAML-CPP, Python `yaml`, `colcon`, and the SA3 packages/descriptions/controllers installed**. The source comments target Humble/Jazzy, but the complete dependency manifests were not supplied, so confirm compatibility with your checked-out repository.

The examples use `~/ros_ws/sa3_ws` as a **sample workspace path**, matching the existing SA3 development convention. Replace it if your checkout is elsewhere.

```bash
# Terminal setup — choose the distro actually installed on this machine.
source /opt/ros/humble/setup.bash
# On Jazzy instead: source /opt/ros/jazzy/setup.bash

cd ~/ros_ws/sa3_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Source the same ROS version and workspace in **every new terminal**:

```bash
source /opt/ros/humble/setup.bash       # or jazzy
source ~/ros_ws/sa3_ws/install/setup.bash
```

Verify the packages, installed launch file, and executables:

```bash
ros2 pkg prefix sa3_motion
ros2 pkg prefix sa2_description
ros2 pkg prefix sa2_moveit_config
ros2 pkg executables sa3_motion
ros2 launch sa3_motion sa3_motion_sequence_yaml.launch.py --show-args
```

`ros2 pkg executables sa3_motion` should include **both** `sa3_generic_executor` and `gantry_controller`. If a package/executable/launch cannot be found, check your CMake install rules, selected workspace, ROS overlay, and rebuild; do not continue to motion execution.

> This README does not invent a repository clone URL or an SA3 environment launch filename: those are not included in the uploaded material. Obtain the actual repository and environment bring-up instructions from your project's source tree.

## 4. How to run

### A. YAML preflight — do this first

```bash
ros2 launch sa3_motion sa3_motion_sequence_yaml.launch.py \
  dry_run:=true
```

This checks YAML structure, schema version, named action existence, and missing action placeholders. The launcher reports something like `[SA3 PREFLIGHT] ... validated` and then shuts down **without commanding motors or planning-scene objects**.

**Preflight does not** build or execute trajectories, test IK/collisions, verify TF availability, connect to the gantry, or validate every runtime value. A passing preflight is not proof that a motion is safe or executable.

### B. Bring up the simulation environment

Before normal motion, your repository's **existing SA3 simulation/MoveIt environment** must be running. It must supply:

- Robot descriptions and current joint feedback (`/joint_states`).
- Published `world` and relevant workcell task frames (TF).
- The `move_group` action/services and supported OMPL/Pilz pipelines.
- Motion controllers for the four robot arms.
- The **existing** `/gantry_controller/follow_joint_trajectory` action server.
- The workcell collision objects and the MoveIt planning-scene services.

**The uploaded `sa3_motion_sequence_yaml.launch.py` is an orchestration launch, not a full RViz/Gazebo/MoveIt bring-up.** It starts the gantry **bridge** plus action executor processes; it does not launch `move_group`, RViz, robot descriptions, or all the physical/simulated controllers. Use the actual environment launch shipped with your repository (its filename was not supplied here). Avoid starting duplicate environment/controller stacks.

Check that the environment is ready:

```bash
ros2 node list | grep move_group
ros2 topic list | grep joint_states
ros2 action list | grep gantry_controller
ros2 service list | grep -E 'get_planning_scene|check_state_validity'
ros2 run tf2_ros tf2_echo world bm_pkL
```

For the final command, seeing a valid transformation proves the requested frame is published; stop the continuous output with `Ctrl+C`.

### C. Start the SA3 sequence

In another terminal, source ROS and the workspace, then run:

```bash
ros2 launch sa3_motion sa3_motion_sequence_yaml.launch.py \
  simulation_confirmed:=true
```

This uses the default installed `config/sa3_execution_sequence.yaml` and `config/sa3_action_library.yaml` from `sa3_motion`. The launch starts the gantry bridge, then the first enabled sequence stage, and progresses according to the stage `mode`.

**Custom YAML paths** (useful when testing edits outside the installed package):

```bash
ros2 launch sa3_motion sa3_motion_sequence_yaml.launch.py \
  sequence_config:=/absolute/path/sa3_execution_sequence.yaml \
  action_library:=/absolute/path/sa3_action_library.yaml \
  workcell_config_path:=/absolute/path/workcell.yaml \
  simulation_confirmed:=true
```

Use `dry_run:=true` with those same paths **before** a normal launch. The workcell path override is optional; when empty, C++ loads `sa2_description/config/workcell.yaml` from the installed package.

### Launch arguments

| Argument | Supplied default | Meaning |
| --- | --- | --- |
| `sequence_config` | `sa3_motion/config/sa3_execution_sequence.yaml` | YAML file providing `sa3_sequence:` |
| `action_library` | `sa3_motion/config/sa3_action_library.yaml` | YAML file providing `action_library:` |
| `dry_run` | `false` | Launcher-only static preflight when `true` |
| `simulation_confirmed` | `true` | Required for normal simulation execution; this is **not** hardware approval |
| `workcell_config_path` | Empty | Override installed `sa2_description/config/workcell.yaml` |
| `gantry_horizontal_min_m`, `gantry_horizontal_max_m` | `-10.6`, `10.6` | Accepted gantry X joint limits (metres) |
| `gantry_vertical_min_m`, `gantry_vertical_max_m` | `0.0`, `6.6` | Accepted gantry Z joint limits (metres) |

**Stop the workflow:** `Ctrl+C` in the orchestration terminal requests shutdown. If execution fails, the code attempts a gantry cancellation and collision-matrix cleanup, but neither a cancel request nor a process exit proves every robot has stopped. Check actual joint feedback and the controller state before a retry.

## 5. What the current assembly sequence does

There are **10 stages**, with **7 enabled** in the supplied sequence file. The final three are present but `enabled: false`.

| # | Stage in the supplied YAML | Enabled | Scheduling / purpose |
| --- | --- | --- | --- |
| 1 | Initialize attached Schunk tools | Yes | Attach Robot1/Robot2 Schunk collision objects to tool mounts |
| 2 | Gantry transfer and flange 3C/4C | Yes | **Parallel**: gantry transports `rebar_1A` while Robot3/Robot4 pick/place 3C/4C |
| 3 | Reverse transfer and flange 3D/4D | Yes | Reverse Robot3/4 transfer, then pick/place 3D/4D without the initial approach |
| 4 | Four-arm flange home plus first shear approach | Yes | Robot3/4 home while Robot1/2 approach first shear picks |
| 5 | Shear links paired pick/place 8 ticks | Yes | Eight sequential `shear_pair` invocations using Robot1 and Robot2 |
| 6 | Final unpaired Robot1 shear link | Yes | One `shear_single` action for the remaining left shear link |
| 7 | Bottom mat Robot3/4 + Robot1/2 gripper change fused | Yes | Lower/release bottom mat, Robot3/4 place while Robot1/2 change to tying guns |
| 8 | Tying all shear links (19 Robot1 / 17 Robot2 positions) | **No** | 17 paired `tying_pair` calls + 2 `tying_single` calls |
| 9 | Tying bottom mat (three paired tie positions) | **No** | 3 paired `tying_pair` calls |
| 10 | Pick and place final assembly | **No** | `final_assembly` |

`mode: sequential` runs a stage's listed action invocations one after another. `mode: parallel` starts all listed actions at the same time and advances only after all succeed. The parallel stage above declares `verified_non_interference: true`; the launch requires this flag for enabled parallel stages, but the flag **does not itself prove spatial clearance**.

An action call is **not** a new C++ program. For example, all eight `shear_pair` invocations use the *same action definition* with different `with:` values.

## 6. Understanding the three YAML layers

### Layer 1 — sequence: which action runs?

In `sa3_execution_sequence.yaml`, the scheduler reads `sa3_sequence.stages` in order:

```yaml
sa3_sequence:
  schema_version: 1
  stages:
    - name: Example shear-link placement
      mode: sequential
      actions:
        - action: shear_pair
          with:
            pick_a: 5pf
            pick_b: 6pf
            place_a: 5_placef_l
            place_b: 6_placef_r
            object_a: rebar_5a
            object_b: rebar_6
            first_approach: true
```

- `name`: human-readable stage label in logs.
- `mode`: `sequential` or `parallel` (at the **stage's action list** level).
- `actions[].action`: must exactly match a key in `action_library.actions`.
- `with:`: parameters for this particular invocation; they override that action's `defaults:`.
- `enabled: false` on a stage: launcher skips running that stage. Disabled entries are still structurally inspected during preflight.

### Layer 2 — action definition: reusable steps

In the **separate** `sa3_action_library.yaml`:

```yaml
action_library:
  actions:
    shear_pair:
      defaults:
        first_approach: true
        group: sa3_robots_1_2
        pick_a_offset: [0, 0, 0]
        pick_b_offset: [0, 0, 0]
      steps:
        - op: move
          id: P1
          motion: moveL
          strategy: separate_merge
          group: sa3_robots_1_2
          participants: [robot1, robot2]
          goals:
            - robot: robot1
              target:
                reference: '{{pick_a}}'
                offset_frame: world
                xyz: [0, -0.25, 0]
                rpy: [0, 0, 0]
            - robot: robot2
              target:
                reference: '{{pick_b}}'
                offset_frame: world
                xyz: [0, -0.25, 0]
                rpy: [0, 0, 0]
          enabled: '{{first_approach}}'
```

The example is an **abbreviated extraction** of the real `shear_pair` action: the full definition has additional fallback options, other steps, attachment and collision logic. Do not replace the entire real action with this snippet.

`'{{pick_a}}'` is a placeholder, replaced using `with:` and then `defaults:`. The executor handles nested lists/dictionaries too. `id: P1` names the local step; different actions can use the same step ID.

### Layer 3 — motion target: where and how does it move?

For a `move` step, the key fields are:

| Field | Explanation |
| --- | --- |
| `op: move` | Execute a robot motion operation |
| `id` | Step label shown in logs, e.g. `P1`, `T2`, `S5_R5_D5` |
| `motion` | Planner family: `moveJ`, `moveL`, `movePTP`, `moveC` |
| `strategy` | How one or several robot paths are constructed |
| `group` | MoveIt group that will execute the resulting trajectory |
| `participants` | Required by `separate_merge`; robots represented in the merged trajectory, including stationary holds |
| `goals` | One or more robot targets, their links and reference poses or named states |
| `target.reference` | Existing TF frame to move toward, e.g. `5pf`, `bm_pkL` |
| `target.xyz` | `[x,y,z]` translation in **metres** |
| `target.rpy` | `[roll,pitch,yaw]` rotation in **radians** |
| `offset_frame` | `world` or `local` for XYZ offset |
| `rpy_frame` | `world` or `local` for rotation (executor defaults to `local`) |
| `base_xyz`, `base_offset_frame` | Optional first/base translation, followed by `xyz` (used in flange retract) |
| `named_state: home` | Send robot to an existing SRDF named state; current implementation requires `moveJ` |
| `enabled` | Optional per-step enable/disable boolean or substituted value |
| `before`, `after` | Scene/collision operations run before planning or after **successful** execution |
| `fallbacks` | Extra planner/strategy candidates tried before submitting a trajectory |

**World vs local offset:** `xyz: [0, -0.25, 0]` with `offset_frame: world` shifts along world −Y. With `local`, the same offset is rotated by the reference frame orientation. Do not confuse a workcell frame's **parent** orientation with the world axes.

**Orientation:** `rpy_frame: local` applies the RPY rotation after the reference orientation; `world` applies it before. All RPY entries are radians, not degrees.

### YAML conventions

- YAML uses indentation; use spaces, not tabs. Keep `steps:` elements aligned.
- `#` starts a comment and the commented line does not execute.
- `[0.0, 0.0, 0.0]` is a three-number XYZ/RPY array.
- `&some_name` defines a YAML anchor; `*some_name` reuses that value. Anchors are YAML syntax, not action IDs.
- Every referenced TF frame, scene-object ID, joint name, robot, and MoveIt group must really exist in the configured environment.
- The code resolves action inputs at **action invocation** start; edits to YAML while a motion is running are not hot-reloaded.

## 7. Operations (`op`) supported by the executor

| `op` | Required / notable fields | Effect |
| --- | --- | --- |
| `move` | `id`, `motion`, `strategy`, `group`, `goals`; for `separate_merge`, `participants` | Plan and execute via MoveIt |
| `attach` | `object`, `link`; optional `placement_mode`, `dock_frame`, `touch_links` | Attach a **MoveIt collision object** to a robot/tool link |
| `detach` | `object`; optional `link` (ownership check) | Return attached collision object to the world |
| `acm_allow` | `pairs: [[object, link], ...]` | Temporarily permit only selected collision pairs |
| `acm_restore` | `pairs: [[object, link], ...]` | Restore previously saved collision permissions |
| `gantry_send` | `request_id`, `x`, `z`; optional `velocity`, `acceleration`, `wait`, `timeout` | Send gantry X/Z target through bridge |
| `gantry_wait` | `request_id`; optional `timeout` | Wait for previously sent gantry request to finish |
| `sleep` | `seconds` | Delay inside an action (implemented in C++; not used in supplied action definitions) |

A `move` step can include `before:` and `after:` lists, each containing one or more operations. **`after:` is reached only if motion execution reports success.** If an operation fails, its action fails, and the launch stops the remaining sequence; there is no automatic replay of a partially executed action.

### Attachment placement modes

- `preserve_world` — attach an existing world collision object to the specified link without changing its world pose at the instant of attachment.
- `dock_to_mount` — compute the object's transform relative to its `dock_frame`, then use that relative transform when attaching to the mount. Used for the Schunk tools that start at known docks.
- `touch_links` — allow specified links to touch the attached object as part of the attachment representation.

`attach` and `detach` **only modify MoveIt's planning-scene object ownership**; they do not directly energize or release a physical gripper. Any real gripper actuation requires separately integrated hardware controls.

### Collision permissions

`acm_allow` and `acm_restore` update the Allowed Collision Matrix (ACM). The executor saves a pair's previous effective setting and restores it after the matching operation; it also attempts cleanup at the end of an action. Never remove restoration steps casually: a persistent allowed collision can hide a real interference.

## 8. Supported motion types, strategies, and fallback policies

| `motion` | Planner | Used for |
| --- | --- | --- |
| `moveJ` | OMPL (`RRTConnectkConfigDefault`) | Joint-space motions, home states and fallback planning |
| `moveL` | Pilz `LIN` | Linear Cartesian approaches, picks, releases and retracts |
| `movePTP` | Pilz `PTP` | Point-to-point transfer motions |
| `moveC` | Pilz `CIRC` | Circular-arc fallback with configured via/interim point |

| `strategy` | Behavior | Limitation |
| --- | --- | --- |
| `separate_merge` | Plan participating robot goals individually, time-align tracks, merge into a single trajectory for the named group | Must include all group joints; collision validation matters for concurrently moving arms |
| `combined` | One combined-group joint-space plan | Supported for `moveJ` only in this source |
| `coordinated_cartesian` | Interpolate all robot end-effector poses together, solve IK and time-parameterize joint path | Supported for `moveL` only in this source |

The library's current global planner settings are:

```yaml
planning:
  full_cell_collision_check: false     # Currently OFF in supplied files
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

The **velocity schedule is a pre-execution planning fallback**, not a command to change velocity halfway through an executing trajectory. The executor first attempts the preferred planner at higher scaling, then lower scales until planning/validation succeeds. For `flange_pp` and `flange_reverse_pp`, the global `movePTP → moveJ` fallback is also considered. Explicit step `fallbacks:` are supported, e.g. `moveL → moveC → moveJ` in the shear action.

**Important:** The executor does **not** automatically rerun a robot trajectory after MoveIt has started executing it. A failed/uncertain execution stops the action. That distinction is essential when handling an object or coordinating four arms.

### Full-cell collision validation

The supplied YAML sets `full_cell_collision_check: false`. In that configuration, the executor skips its additional all-robot `/check_state_validity` sampling pass; MoveIt planner collision checks still apply. For safe validation of multi-robot merged motions, enable full-cell checking **after correcting any intended-contact ACM permissions and confirming the environment scene**. Never bypass a real collision by blanket-disabling checks. `preview_rejected_trajectories: true` can publish rejected trajectories for RViz diagnosis but does not execute them.

## 9. Gantry control: commands, IDs, waiting, and timeouts

The gantry is a persistent ROS bridge (`gantry_controller.cpp`). It subscribes to `/sa3/gantry/command` and `/sa3/gantry/cancel`, publishes `/sa3/gantry/status`, and calls the **already-running** `/gantry_controller/follow_joint_trajectory` action. The joint names come from `workcell.gantry_robot`:

```yaml
gantry_robot:
  horizontal_joint: gantry_slide_joint
  vertical_joint: gantry_down_joint
  controller_action: /gantry_controller/follow_joint_trajectory
  attach_link: gantry_part_link
```

### What the supplied C++ **actually** accepts

This is especially important because the combined sequence file contains comments proposing a different convention.

```yaml
- op: gantry_send
  id: G4_transport
  request_id: gantry_pick_g4
  x: '{{gantry_transport_x}}'
  z: '{{gantry_up_z}}'
  velocity: 0.45
  acceleration: 0.5
  wait: true
  timeout: 120
```

- `id` is **only the readable step name** in action logs.
- `request_id` is **currently mandatory in `sa3_generic_executor.cpp`**. The executor prefixes it with its action-instance ID to prevent conflicts, and the bridge uses it to match status/cancellation messages.
- `wait: true` means do not proceed until the gantry reports `DONE` (or an error/timeout occurs).
- `wait: false` or an **omitted** `wait` means wait for acceptance (`RUNNING`/`DONE`) but do not require completion before the next step. **Current C++ default is false**, not true.
- `timeout: 120` applies to completion waiting when `wait: true`; `gantry_wait` can have its own timeout. An initial acceptance wait has a separate fixed 20-second limit.
- A separate `gantry_wait` must use the **same `request_id`** as the earlier `gantry_send` to wait for the corresponding request.

**Current-code warning:** The `action_library:` embedded in `sa3_execution_sequence.yaml` removes `request_id` and assumes omitted `wait` defaults to `true`; the supplied C++ **does not implement that behavior**. If you select that embedded copy as the library without updating the C++, the first gantry command will fail with `Gantry command needs request_id`. For the uploaded binary/source pairing, use the separate `sa3_action_library.yaml`, whose gantry steps include request IDs and explicit wait flags, or update and retest **both** code and YAML consistently.

### Gantry safety checks from the supplied bridge

- Checks X/Z targets against launch-configured bounds.
- Requires fresh `/joint_states` feedback before sending a non-dry command.
- Rejects commanded `velocity > 2.0` or `acceleration > 5.0` (also rejects non-positive/non-finite values).
- Rejects concurrent requests while busy or duplicate request IDs within the running bridge.
- After controller success, checks final X and Z are each within **0.01 m** of the requested goal, using fresh joint feedback.
- Uses bounded controller acceptance, result and joint-state verification waits; an error/cancel response does not guarantee instantaneous mechanical stopping.

**Known configuration issue:** The separate `sa3_action_library.yaml` sets `bottom_mat_pp → G5_lower → velocity: 2.45`, but `gantry_controller.cpp` rejects velocities above `2.0`. That particular action is **not** called in the current enabled sequence (the enabled bottom-mat workflow calls `bottom_mat_and_gripper_change` instead). Correct the value and validate it before using standalone `bottom_mat_pp`.

## 10. `workcell.yaml`: positions, TFs, robots, and objects

The workcell configuration is the physical-geometry source: the executor uses it for robot group/TCP mappings and the gantry bridge uses it for gantry joint/controller names. The rest of your environment must publish the referenced frames and load the object geometry.

| Section | Examples / purpose |
| --- | --- |
| `workcell.web_ring_frames` | Left-side shear-link pick frames (`5pf`, `13pf`, `14pf`) |
| `workcell.web_ring2_frames` | Right-side pick frames (`6pf`, `13_1pf`, `13_2pf`, `14_4pf`) |
| `workcell.inventory_frames` | Flange pick approaches and bottom-mat pick frames (`bm_pkL`, `bm_pkR`) |
| `workcell.place_frames` | Jig placement frames, including `place_frame_10_l`, `place_frame_10`, `bm_pfl`, `bm_pfr` |
| `workcell.tying_frames` | Shear-link and bottom-mat tie targets (`*_t1`, `*_t2`, `1A_l_t1`, etc.) |
| `workcell.arc_frames` | Transfer frames including `r3_ptp_start`, `r3_ptp_end`, `r4_ptp_start`, `r4_ptp_end` |
| `workcell.tool_dock_frames` | Schunk and tying-gun dock frames |
| `workcell.extra_rebars` | Rebar collision meshes/poses and object IDs |
| `workcell.extra_grippers` | Removable tool meshes/poses and IDs |
| `workcell.robots` | Per-robot MoveIt group, TCP/planning link, tool mount and tying link |
| `workcell.gantry_robot` | Gantry joints, controller action and attachment link |
| Top-level `environment` | Jig, racks, inventory, gantry and other environment meshes/poses |
| Top-level `robot_pedestals` | Robot support placement transforms |

**Frame vs object:** `3c` is a pick TF frame, while `rebar_3c` is the corresponding planning-scene object ID. Passing the latter as `target.reference` will only work if a TF of that exact name also exists. Conversely, `attach` requires an object ID, not just a TF frame.

Example frame definition from the supplied workcell:

```yaml
workcell:
  inventory_frames:
    bm_pkL:
      parent: world
      child: bm_pkL
      xyz: [0.420, 1.864, 4.116]
      rpy: [-0.571, -0.007, 1.573]
```

Changing this frame changes the target of **every** action that references `bm_pkL`. Changing only a `with: pick_a_offset: [...]` in the sequence affects the corresponding action invocation rather than the global frame.

## 11. How to make changes without changing C++

### Example A — change a pick target or offset

**Goal:** adjust the 3D/4D pickup. Open `sa3_execution_sequence.yaml` under the `Reverse transfer and flange 3D/4D` stage, then edit only the `flange_pp` invocation:

```yaml
- action: flange_pp
  with:
    pick_a: 3d
    pick_b: 4d
    object_a: rebar_3d
    object_b: rebar_4d
    pick_a_offset: [0.127, -0.045, 0.008]
    pick_b_offset: [-0.155, -0.057, -0.01]
    do_approach: false
    # Keep the other existing required inputs here:
    # approach_a/b, transfer_start_a/b, transfer_end_a/b, place_a/b,
    # pick_a_rpy, pick_b_rpy, place_a_offset, place_b_offset, do_home.
```

This **illustrates just the fields being edited**, not a full drop-in replacement for the current action call. Keep existing `with:` fields not shown in the snippet. When using `flange_pp`, its defaults supply optional values such as `robot_a`, `robot_b`, `group`, `do_home`, and zero offsets.

### Example B — add a new movement between existing steps

**Goal:** add a safe transfer waypoint to `flange_pp` without creating another C++ motion node.

1. Create/publish both waypoint frames in your workcell/environment (for example, `my_safe_r3_via` and `my_safe_r4_via`). Verify TFs in RViz.
2. Open the **canonical** `sa3_action_library.yaml`.
3. In `action_library.actions.flange_pp.steps`, insert a new `move` after `T1` and before `T2`:

```yaml
- op: move
  id: T1_extra_via
  motion: movePTP
  strategy: separate_merge
  group: '{{group}}'
  participants:
    - '{{robot_a}}'
    - '{{robot_b}}'
  goals:
    - robot: '{{robot_a}}'
      target:
        reference: my_safe_r3_via
        offset_frame: world
        xyz: [0, 0, 0]
        rpy: [0, 0, 0]
    - robot: '{{robot_b}}'
      target:
        reference: my_safe_r4_via
        offset_frame: world
        xyz: [0, 0, 0]
        rpy: [0, 0, 0]
```

4. Run YAML preflight, then simulate and inspect the full two-arm trajectory/collisions.

**Scope warning:** `flange_pp` is called for 3C/4C **and** 3D/4D, so changing its action definition affects **both** calls. To use different waypoints, add `via_a` / `via_b` placeholders to the action and set those variables separately in each call's `with:` block (and ensure each reference resolves).

### Example C — enable the tying stages

In `sa3_execution_sequence.yaml`, the supplied tying stages use `enabled: false`. Change only the stage(s) you intend to test to `enabled: true` **after** validating tool attachment, all tie-frame TFs, collision allowances and tool trajectories. The code knows `tying_pair` and `tying_single` already; enabling a stage is a scheduling change, not a C++ change. The supplied YAML defines **17 paired + 2 single shear tying invocations** and **3 paired bottom-mat tie invocations**.

### Example D — change planning speed / fallback settings

Edit `action_library.planning` in **`sa3_action_library.yaml`**. To try slower Pilz LIN motion first, put the preferred lower scale at the beginning of the `moveL.velocity_scales` list. To change OMPL joint-motion speed, edit `moveJ.velocity`. To adjust PTP fallback attempts, edit `movePTP.velocity_scales`. Keep scales within `(0, 1]`. Changes are loaded at the next invocation/launch, not during a currently executing path.

## 12. How the C++ implementation works

The primary executor is `sa3_generic_executor.cpp`. It is reusable because the source does **not** hard-code which shear item, flange, or transfer action to run; it selects one library entry using `action_name` passed by the launch.

| Component / method | Responsibility |
| --- | --- |
| Launch `setup()` | Load both YAML paths, check schema, action names and placeholders |
| Launch `run_stage()` / `run_serial()` | Start the correct action processes in sequential or parallel order |
| `ActionEngine` constructor | Load named action/defaults/inputs, workcell groups, TF, ROS topics/services |
| `interpolate()` | Resolve `{{name}}` substitutions, including arrays/maps |
| `goalOf()` | Read robot target, resolve TF, apply local/world XYZ and RPY offsets |
| `configure()` | Configure MoveIt OMPL/Pilz pipeline, planner, acceleration/velocity and attempts |
| `separate()` | Plan one robot goal |
| `merged()` / `mergeTracks()` | Combine separate arm trajectories with synchronized duration and participant holds |
| `combined()` | Plan one combined-group joint-space target |
| `coordinatedCartesian()` | Generate synchronized Cartesian samples and IK for several arms |
| `motion()` | Try planners/scales, optionally validate the path, execute successful plan once |
| `attachObject()` / `detachObject()` | Change MoveIt planning-scene ownership and verify it |
| `setAcm()` / `restoreOutstanding()` | Manage per-pair allowed collisions and cleanup |
| `gantry()` / `waitGantry()` | Send and track gantry commands via ROS topics |
| `stepRun()` | Dispatch each YAML `op`, including `before:` and `after:` operations |

The gantry bridge `gantry_controller.cpp` owns the FollowJointTrajectory client. It is **not** a substitute for the existing trajectory controller/action server; it translates high-level YAML-like topic messages to that controller.

**Process boundaries:** action invocations run in separate ROS processes (`sa3_generic_executor` instances). Therefore, it is correct to call this a *generic executor*, but inaccurate to assume one persistent executor instance runs the whole sequence. The launch supervises the invocation processes and stops the sequence if one returns a non-zero code.

## 13. Safety and execution rules

1. **Simulation only:** `simulation_confirmed:=true` acknowledges use of the simulation route, not proof of a safe real robot motion.
2. **Run preflight first, then test in RViz:** a valid YAML is not a collision-free trajectory.
3. **Do not let two independent actions command the same joints concurrently.** Parallel stages must use nonconflicting resources and verified workspaces. `verified_non_interference: true` is a user declaration, not an automated geometric proof.
4. **Treat an execution failure as an uncertain real state.** Confirm actual joint states, attached objects and tool positions before replaying a pick/place.
5. **Keep `acm_allow` narrow and restore it.** Only allow intended contacts (such as grasping or placement contact), never arbitrary robot/environment collisions.
6. **Do not assume attach/detach means gripper actuation.** This prototype operates on MoveIt scene objects.
7. **Do not launch a second simulated controller stack** on top of the existing one.

## 14. Debugging and troubleshooting

Start with the **first actual error** (`[SA3 ACTION] FAILED`, `[PRE-EXEC RETRY]`, `[GANTRY JTC]`, MoveIt planning errors), rather than the last generic `STOP` line.

| Symptom / log | Likely reason | What to check |
| --- | --- | --- |
| `Package 'sa3_motion' not found` | Wrong workspace overlay or package not built | Source correct `/opt/ros/...` and `install/setup.bash`; check `ros2 pkg prefix` |
| `file 'sa3_motion_sequence_yaml.launch.py' was not found` | Launch file missing from install tree | Verify `launch/` install rules and rebuild |
| `No executable found` | Missing CMake target/install | `ros2 pkg executables sa3_motion`; inspect CMake |
| `Expected schema_version: 1` | Wrong YAML layout | Check `sa3_sequence.schema_version` and `action_library.schema_version` |
| `Unknown action library name` | Sequence references nonexistent action | Match `actions[].action` to `action_library.actions.<name>` |
| `action inputs missing` / `Missing action parameter` | Placeholder has no `defaults:` / `with:` value | Search for the exact `{{...}}` name in the actual action library |
| `Gantry command needs request_id` | Embedded id-only library loaded with existing C++ | Use separate compatible action library or update C++ and YAML together |
| Gantry `already executing` | A new command arrived before prior request completed | Use `wait: true` and/or explicit matching `gantry_wait`; check bridge status |
| Gantry `Invalid ... speed/acceleration` | Speed >2.0, acceleration >5.0 or invalid input | Inspect gantry command; unused standalone bottom-mat action currently has 2.45 speed |
| Gantry goal rejected / unavailable | Existing FJT action not running or mismatched name | `ros2 action list | grep gantry`; check controller manager and YAML joint names |
| Gantry final feedback disagreed `>0.01m` | Controller result and live joint feedback differ | Check `/joint_states`, target, controller and tolerance; do not blindly change a limit |
| `Invalid frame ID` / TF lookup failure | Pick/place/dock frame not published | `ros2 run tf2_ros tf2_echo world <frame>` and frame producer |
| `Unknown named state: home` | Named state missing or wrong group | Check SRDF and group naming |
| Individual planning failed / IK failed | Unreachable pose, wrong TCP or motion planner | Verify TF, link, offsets, starting state, group and collision contacts |
| Pilz LIN/PTP/CIRC fails repeatedly | Cartesian/geometric constraints or planner not configured | Check actual Pilz pipeline, start pose and fallback logs; do not assume lower speed solves IK |
| `Combined group joint not covered` | `participants`/group combination inconsistent | Match robot group membership and goals; include all required group joints |
| `Full-cell collision validation rejected` | Collision found on combined motion | Inspect reported contact objects, planned path and allowed collision pair scope |
| `Attach object absent from world` | Scene object not spawned or wrong object ID | Verify planning-scene objects; distinguish TF frame from object ID |
| `Cannot detach unowned object` | Scene ownership differs from expected | Check earlier attachment, object ID and owner link |
| `ACM restore without saved previous value` | Restore called without matching allow in that executor instance | Check step order and process boundaries |
| Duplicate ROS node names warning | An environment node/bridge was started more than once | Inspect `ros2 node list`, running launches and controller processes |

Useful commands:

```bash
# Nodes, planners, controllers and gantry communication
ros2 node list
ros2 action list
ros2 topic echo /sa3/gantry/status
ros2 topic echo /joint_states
ros2 service list | grep -E 'get_planning_scene|check_state_validity'

# Confirm reference frames (Ctrl+C to stop)
ros2 run tf2_ros tf2_echo world 3c
ros2 run tf2_ros tf2_echo world bm_pkL
ros2 run tf2_ros tf2_echo robot3_base_link robot3_end_effactor_ff

# ROS logs: use the actual run directory if ~/.ros/log/latest exists
ls -ltr ~/.ros/log
rg -n 'FAILED|ERROR|PRE-EXEC RETRY|GANTRY JTC|COLLISION|ACTION STEP' ~/.ros/log/latest/ 2>/dev/null
```

`rg` is the optional `ripgrep` utility; if it is not installed, replace the last command with `grep -RniE 'FAILED|ERROR|PRE-EXEC RETRY|GANTRY JTC|COLLISION|ACTION STEP' ~/.ros/log/latest/`.

### Suggested validation workflow after any YAML change

```text
1. Edit the correct source file (sequence vs separate action library vs workcell).
2. Confirm YAML parses and no placeholders were accidentally renamed.
3. Run launch dry_run:=true using the exact file paths intended for execution.
4. Check TF frames, MoveIt group names, scene-object IDs and controller endpoints.
5. Run in a controlled simulation and inspect all robot/gantry paths in RViz.
6. Confirm object attachment/detachment and ACM cleanup after each action.
7. Review logs; only then enable additional stages or higher speeds.
```

## 15. When to rebuild

| Change | Rebuild required? |
| --- | --- |
| Edit stage order, `with:` parameters, or action-library steps | **Usually no** when config files are installed with `--symlink-install`; stop/relaunch to reload |
| Edit `workcell.yaml` frame/pose values | **Depends on the frame publisher**: restart/regenerate environment as needed; a YAML-only edit cannot update an already-published static TF automatically |
| Edit `sa3_generic_executor.cpp` or `gantry_controller.cpp` | **Yes**, rebuild `sa3_motion` |
| Edit launch Python | With symlink install often reloadable on new launch; rebuild/install if not linked or not installed |
| Edit Xacro, SRDF, robot/joint limits, controller config | Rebuild/relaunch whichever description or MoveIt/controller package changed |
| Edit CMakeLists.txt or package.xml | **Yes**, rebuild the affected package |

For C++ changes:

```bash
cd ~/ros_ws/sa3_ws
source /opt/ros/humble/setup.bash    # or jazzy
colcon build --symlink-install --packages-select sa3_motion
source install/setup.bash
```

Always restart the existing action/gantry processes after rebuilding. A running C++ binary does not reload itself.

## 16. Known source-version limitations and checklist for maintainers

These points are grounded in the **specific uploaded files** and should be reviewed before handing the system to a new operator:

- **Split vs embedded library:** the sequence YAML includes an extra copy of `action_library:`; default launch uses the separate file. This can easily lead to changing the wrong copy.
- **Requested `id`-only gantry syntax is not in the supplied C++:** `sa3_generic_executor.cpp` still requires `request_id` and defaults omitted `wait` to `false`. Comments in the combined YAML saying otherwise are aspirational, not executable behavior.
- **The separate library retains asynchronous G6 steps:** `bottom_mat_pp` and `bottom_mat_and_gripper_change` use `gantry_send` with `wait: false` followed by `gantry_wait` for the same request. They are not fully converted to synchronous one-step sends.
- **Unsafe-speed mismatch in an unused action:** `bottom_mat_pp.G5_lower` asks for `2.45`, above the bridge's `2.0` maximum.
- **Full-cell validation is OFF by default** in both supplied libraries; review this before validating multi-robot trajectories.
- **No complete environment launch or build manifests were provided.** They must be checked in the actual repository; this README does not assert an unverified full bring-up command.
- **The code has not been run against your live ROS workspace as part of preparing this document.** The explanations above are a source-level review, not a claim of successful motion execution.

## 17. Quick command reference

```bash
# New terminal — choose installed ROS distro
source /opt/ros/humble/setup.bash
source ~/ros_ws/sa3_ws/install/setup.bash

# Validate YAML only — no motion
ros2 launch sa3_motion sa3_motion_sequence_yaml.launch.py dry_run:=true

# Start your repository's SA3 simulation + MoveIt environment separately.
# Its launch filename is not provided in the uploaded files.

# Execute enabled SA3 assembly sequence in confirmed simulation
ros2 launch sa3_motion sa3_motion_sequence_yaml.launch.py simulation_confirmed:=true

# Inspect installed programs
ros2 pkg executables sa3_motion

# Inspect gantry status and frames
ros2 topic echo /sa3/gantry/status
ros2 run tf2_ros tf2_echo world bm_pkL
```

---

**First-day checklist for a new developer:** find the actual repository's environment bring-up launch, build/source the three packages, verify TF and `move_group`, run YAML preflight, read stages 1–7 in the sequence, inspect the exact matching actions in the **separate** library, and test only in simulation. Before adopting the proposed id-only gantry YAML convention, reconcile the C++ and YAML incompatibility described in Sections 2, 9, and 16.

# SA3 schema-v2 planning-only recipe audit

## Files

Copy `ik_reach_test.cpp` into `sa3_motion/src/ik_reach_test.cpp`.

The audit reads `workcell.yaml` **only** for physical robot definitions and planning links, and `ms_2_re.yaml` for `motion_sequence.tasks`, stages, offsets and motion tuning. It does not look for the old `workcell.planner_sequence`.

## CMake

Confirm:

```cmake
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moveit_core REQUIRED)

add_executable(ik_reach_test src/ik_reach_test.cpp)
ament_target_dependencies(ik_reach_test
  rclcpp moveit_ros_planning_interface moveit_core moveit_msgs
  geometry_msgs sensor_msgs tf2 tf2_ros ament_index_cpp)
target_link_libraries(ik_reach_test yaml-cpp)
install(TARGETS ik_reach_test DESTINATION lib/${PROJECT_NAME})
```

Keep your existing target definitions rather than adding duplicates. Eigen is provided transitively by MoveIt in typical installations; include Eigen3 explicitly if your build requires it.

## Run (with robot state publisher, TF, MoveIt, and planning scene already running)

```bash
colcon build --packages-select sa3_motion --symlink-install
source install/setup.bash

# Active recipe tasks only (currently flange_3c_4c + gantry_bottom_mat_2_re):
ros2 run sa3_motion ik_reach_test --ros-args \
  -p motion_config_path:=/absolute/path/to/ms_2_re.yaml \
  -p workcell_config_path:=/absolute/path/to/workcell.yaml \
  -p report_csv:=/tmp/sa3_ik_reachability.csv

# One specific task (can select even a disabled stage):
ros2 run sa3_motion ik_reach_test --ros-args \
  -p motion_task:=shear_links_2_re

# All declared tasks (includes unsupported types recorded as NOT_TESTED):
ros2 run sa3_motion ik_reach_test --ros-args \
  -p include_inactive_tasks:=true
```

Optional `-p chain_plans:=true` (default) reuses each group's most recently *validated* trajectory endpoint. `-p stop_on_failure:=true` skips later steps in the same group after a failure. This is more representative of a sequential path audit; it does not synchronize two robots or maintain complete multi-robot scene state.

## Important coverage boundaries

- **Supported**: task loading by exact recipe ID; Robot 3/4 flange frame targets, Robot 1/2 shear pairs, Robot 1/2 tying jobs, Robot 3 gantry pose goals; FK-frame offsets (position and RPY); IK; OMPL MoveJ/Pilz LIN planning; joint bounds, trajectory point widths, finite positions, velocity/acceleration bound checks when derivatives are present, and timestamp monotonicity.
- **Not production-equivalent**: dual-arm combined MoveJ, merged and synchronized LIN trajectories, dynamic collision matrix edits and attach/detach, validation of collision at every trajectory point, exact flange `arc_waypoints`, shear CIRC via-point fallback, gantry controller trajectories and Cartesian `computeCartesianPath` fraction. Those cases are explicitly marked approximate or `NOT_TESTED` where relevant.
- **No motion execution**: never invokes `move()`, `execute()` or a controller action.
- A successful trajectory check only covers sampled joint bounds / provided derivatives and timing; it is **not** proof of collision freedom or safe production execution.
- **No compilation performed** in this environment: build in your ROS 2 / MoveIt workspace and share any diagnostic errors for correction.
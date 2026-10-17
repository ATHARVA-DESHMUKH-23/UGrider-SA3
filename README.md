# SA3 U-Girder Subassembly Simulation

ROS 2 and MoveIt-based simulation for the L&T U-Girder Subassembly 3 (SA3) workcell.

## Build

Source the appropriate ROS 2 distribution for your branch (Humble or Jazzy).

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

For Jazzy, replace `humble` with `jazzy`.

## Launch

**1. Start the workcell and planning scene**

```bash
ros2 launch sa2_moveit_config sa3_bringup.launch.py
```

**2. Start the motion sequence** in a separate terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/sa3_j/install/setup.bash

ros2 launch sa3_motion sa3_motion_sequence_yaml.launch.py \
  sequence_config:=./src/sa3_motion/config/motion_sequence.yaml
```

For Jazzy, source the corresponding Jazzy setup instead.

## Configuration

The simulation is primarily configured through two YAML files:

- **`workcell.yaml`** — Defines the workcell environment, geometry, and frames.
- **`motion_sequence.yaml`** — Defines the manufacturing sequence, including robot operations, pick/place parameters, tool changes, tying operations, parallel execution, retries, and failure handling.

### Switching Cross-Sections

To simulate the alternate cross-section, use **`workcell1.yaml` instead of `workcell.yaml`** and launch the corresponding motion sequence using **`motion_sequence1.yaml` instead of `motion_sequence.yaml`**.

Both files must correspond to the same cross-section.

1. In `sa2_description/config/`, replace `workcell.yaml` with `workcell1.yaml` (rename the files so the active configuration is named `workcell.yaml`).
2. In `sa3_motion/config/`, launch the alternate motion sequence:

   ```bash
   ros2 launch sa3_motion sa3_motion_sequence_yaml.launch.py \
     sequence_config:=./src/sa3_motion/config/motion_sequence1.yaml
   ```

No motion-sequence source-code changes are required for this configuration switch.

## Known Limitations

- **Tying-gun trajectories:** Some trajectories may fail because the current tying-gun STL meshes are incorrect. The STL geometry must be corrected before these trajectories can be expected to work reliably.
- **Alternate cross-section picking:** Picking for alternate cross-sections is not yet fully implemented. The corresponding workcell and motion-sequence YAML files are provided, but the picking configuration still requires further development.

**2. Run the SA3 pick-and-place motion sequence**

In a separate terminal, launch the motion sequence configured in `ms_sa3_pp.yaml`:

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/sa3_j/install/setup.bash

ros2 launch sa3_motion sa3_motion_sequence_yaml.launch.py \
  sequence_config:=./src/src/sa3_motion/config/ms_sa3_pp.yaml
```

For Jazzy, source the corresponding Jazzy setup instead.

**Configuration:** `src/src/sa3_motion/config/ms_sa3_pp.yaml` defines the sequence to execute up to and including the SA3 pick-and-place operation. The sequence order, executable parameters, execution mode, retries, and failure handling are controlled by this YAML file.

**Status:** Under development; cleanup and validation are ongoing.


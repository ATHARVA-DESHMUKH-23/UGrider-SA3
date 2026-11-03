# ROS 2 Jazzy ↔ Humble API Field Conversion

This guide documents the `MoveGroupInterface::Plan` member-name changes
encountered in `sa3_motion` when switching between ROS 2 Jazzy and
Humble.

Run the commands from the workspace root:

``` bash
cd ~/ros_ws/sa3
```

The commands below only modify files under `src/sa3_motion`.

## Before changing versions

Check your working tree and consider committing or backing up your
changes:

``` bash
git status
git add src/sa3_motion
git commit -m "Save sa3_motion before ROS distro API conversion"
```

Skip the commit if you are not ready to commit; make a backup or use a
Git branch instead.

## Jazzy → Humble

The Jazzy-style member names used in this project are converted to the
Humble-style names by adding the trailing underscore:

-   `trajectory` → `trajectory_`
-   `planning_time` → `planning_time_`
-   `start_state` → `start_state_`

### Preview matching files

``` bash
grep -rnE 'trajectory|planning_time|start_state' src/sa3_motion
```

### Apply the conversion

``` bash
grep -rlZ 'trajectory\.' src/sa3_motion | xargs -0 -r sed -i 's/trajectory\./trajectory_./g'
grep -rlZ 'planning_time' src/sa3_motion | xargs -0 -r sed -i 's/planning_time/planning_time_/g'
grep -rlZ 'start_state' src/sa3_motion | xargs -0 -r sed -i 's/start_state/start_state_/g'
```

**Important:** These are broad text substitutions for the named
identifiers. Review the diff afterwards, because they may also alter
other identifiers containing these strings.

## Humble → Jazzy

Remove the trailing underscores from these member names:

-   `trajectory_` → `trajectory`
-   `planning_time_` → `planning_time`
-   `start_state_` → `start_state`

### Apply the conversion

``` bash
grep -rlZ 'trajectory_\.' src/sa3_motion | xargs -0 -r sed -i 's/trajectory_\./trajectory./g'
grep -rlZ 'planning_time_' src/sa3_motion | xargs -0 -r sed -i 's/planning_time_/planning_time/g'
grep -rlZ 'start_state_' src/sa3_motion | xargs -0 -r sed -i 's/start_state_/start_state/g'
```

## Review changes

Always inspect the changes before building:

``` bash
git diff -- src/sa3_motion
```

Search for any old-style names that should have been converted:

``` bash
# After Jazzy → Humble
grep -rnE 'trajectory\.|planning_time([^_[:alnum:]]|$)|start_state([^_[:alnum:]]|$)' src/sa3_motion

# After Humble → Jazzy
grep -rnE 'trajectory_\.|planning_time_|start_state_' src/sa3_motion
```

A search can still match legitimate identifiers, so review results in
context rather than assuming every match is an error.

## Build

After conversion, source the intended ROS distribution and rebuild:

``` bash
# For Jazzy
source /opt/ros/jazzy/setup.bash

# OR for Humble
source /opt/ros/humble/setup.bash

# Then build from the workspace root
colcon build --packages-select sa3_motion
```

Use only the `source` command for the ROS distribution installed and
intended for the current terminal. These substitutions handle the listed
member names only; they do not guarantee full Humble/Jazzy source
compatibility.

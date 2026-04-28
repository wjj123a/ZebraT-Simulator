# ZebraT-Simulator

ROS Noetic simulator workspace for the ZebraT `R1` robot.

Paper: [ZebraT Gazebo Simulator](https://arxiv.org/pdf/2205.07944.pdf)

## Default Behavior

The default entrypoint is now:

```bash
cd /home/w/ZebraT-Simulator/ZebraT-Simulator
source /opt/ros/noetic/setup.bash
bash run_noetic.sh
```

That launches:

- Gazebo with `zebrat/worlds/area.world`
- the local `R1` robot model
- RViz with a navigation layout
- `map_server + AMCL + move_base`
- laser-based local obstacle avoidance
- `/cmd_vel` arbitration between RViz navigation and keyboard teleop

After startup you can use RViz:

- `2D Pose Estimate` to relocalize
- `2D Nav Goal` to send navigation goals through the safe goal relay

The RViz goal tool publishes to `/safe_move_base_simple/goal`. The `safe_goal_relay`
checks the active costmaps, waits briefly if the requested goal is temporarily occupied,
and relocates the navigation goal to the nearest safe cell when the requested point stays
blocked, unknown, or inside a configured dynamic obstacle route.

## Navigation Modes

Two navigation modes are supported.

### Static Map Mode

Default mode for `area.world`.

```bash
bash run_noetic.sh
```

Equivalent explicit form:

```bash
bash run_noetic.sh navigation_mode:=map map_file:=$(pwd)/zebrat/maps/area.yaml
```

This uses:

- `map_server`
- `amcl`
- `move_base`
- `global_planner/GlobalPlanner`
- `dwa_local_planner/DWAPlannerROS`

### SLAM Mode

For rebuilding a map or running on worlds without a prepared map. The default SLAM backend remains `gmapping`:

```bash
bash run_noetic.sh navigation_mode:=slam
```

To use the RGB-D + 2D laser fusion backend powered by RTAB-Map:

```bash
bash run_noetic.sh navigation_mode:=slam navigation_backend:=rtabmap
```

To save a SLAM snapshot:

```bash
bash run_noetic.sh navigation_mode:=slam save_map:=true
```

Saved maps are written under `zebrat/maps/`. RTAB-Map databases are stored under `zebrat/maps/rtabmap/`.

### Dynamic Obstacle Tests

The `area.world` scene can spawn repeatable mixed dynamic obstacles, including pedestrians and small carts:

```bash
bash run_noetic.sh world_name:=$(pwd)/zebrat/worlds/area.world navigation_mode:=map enable_dynamic_obstacles:=true
```

For RTAB-Map SLAM with the same dynamic obstacles:

```bash
bash run_noetic.sh world_name:=$(pwd)/zebrat/worlds/area.world navigation_mode:=slam navigation_backend:=rtabmap enable_dynamic_obstacles:=true
```

Use `dynamic_obstacle_speed_scale:=0.6`, `1.0`, or `1.4` to run slower or faster obstacle tests.
Gazebo is automatically unpaused by default for R1 launches. Use `auto_unpause:=false` only when you intentionally want Gazebo to stay paused for debugging.

To disable safe goal relocation for raw `move_base` behavior tests:

```bash
roslaunch zebrat r1_navigation_regression.launch enable_dynamic_obstacles:=true goal_set:=dynamic_goal_path_probe safe_goal_enabled:=false
```

## Teleop

Keyboard teleop publishes to `/cmd_vel_teleop`, and the arbiter forwards either teleop or navigation output to `/cmd_vel`.

Start teleop in a second terminal:

```bash
cd /home/w/ZebraT-Simulator/ZebraT-Simulator
source /opt/ros/noetic/setup.bash
bash teleop_noetic.sh
```

Teleop has short-term priority. When you stop sending keys, control returns to navigation automatically.

## RViz-Only Model View

For robot model inspection without Gazebo navigation bringup:

```bash
cd /home/w/ZebraT-Simulator/ZebraT-Simulator
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch zebrat display.launch robot:=r1
```

## Alternate Worlds

The current navigation tuning is validated for `area.world`.

To open another world:

```bash
bash run_noetic.sh world_name:=$(pwd)/zebrat/worlds/racetrack.world
```

If a non-`area` world is selected without an explicit navigation mode, `run_noetic.sh` falls back to `navigation_mode:=slam`.

## Regression Tests

These commands assume a clean system ROS shell. If you usually work inside Conda, prefer `bash run_noetic.sh` / `bash teleop_noetic.sh`, or clear the Conda environment first.

Headless map-mode regression:

```bash
cd /home/w/ZebraT-Simulator/ZebraT-Simulator
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch zebrat r1_navigation_regression.launch navigation_mode:=map
```

Headless SLAM map builder:

```bash
cd /home/w/ZebraT-Simulator/ZebraT-Simulator
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch zebrat r1_map_builder.launch
```

Headless RTAB-Map SLAM map builder:

```bash
cd /home/w/ZebraT-Simulator/ZebraT-Simulator
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch zebrat r1_map_builder.launch navigation_backend:=rtabmap
```

## Notes

- `run_noetic.sh` sanitizes Conda/compiler environment variables before building.
- The script automatically repairs the `src/zebrat` workspace link if needed.
- `R1` no longer depends on an external `../clb_ws/src/R1` package.
- Gazebo uses `zebrat/urdf/R1.urdf`; RViz model display uses `zebrat/urdf/R1_rviz.urdf`.
- The default static map asset is `zebrat/maps/area.yaml`.
- `navigation_backend:=rtabmap` automatically selects `zebrat/rviz/rtabmap_navigation.rviz` unless you pass an explicit `rviz_config:=...`.

## Paper

`@inproceedings{tian2022design,
  title={Design and implement an enhanced simulator for autonomous delivery robot},
  author={Tian, Zhaofeng and Shi, Weisong},
  booktitle={2022 Fifth International Conference on Connected and Autonomous Driving (MetroCAD)},
  pages={21--29},
  year={2022},
  organization={IEEE}
}`

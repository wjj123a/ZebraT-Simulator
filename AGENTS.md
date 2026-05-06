# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS Noetic catkin workspace for the ZebraT `R1` simulator. The ROS package lives in `zebrat/`; `src/` is used as the catkin workspace source area and may contain a generated `src/zebrat` symlink.

- `zebrat/scripts/`: Python ROS nodes for control, teleop, navigation safety, mapping helpers, and regression utilities.
- `zebrat/launch/`: Gazebo, RViz, navigation, SLAM, and regression launch files.
- `zebrat/config/`: navigation, controller, obstacle, and world profile YAML files.
- `zebrat/worlds/`, `zebrat/urdf/`, `zebrat/meshes/`: Gazebo worlds and robot model assets.
- `zebrat/maps/` and `zebrat/rviz/`: map assets, RTAB-Map database location, and RViz layouts.
- `zebrat/test/`: Python `unittest`/nose tests registered through catkin.

## Build, Test, and Development Commands

Run commands from the repository root.

```bash
source /opt/ros/noetic/setup.bash
catkin_make
```

Builds the catkin workspace. The main helper script also performs this build and repairs the `src/zebrat` symlink when needed:

```bash
bash run_noetic.sh
```

Launches the default `area.world` Gazebo simulation with RViz, AMCL, `move_base`, dynamic obstacles, and safety supervision.

```bash
bash run_noetic.sh navigation_mode:=slam
bash run_noetic.sh navigation_backend:=rtabmap navigation_mode:=slam
bash teleop_noetic.sh
```

Start SLAM variants or keyboard Ackermann teleop in a second terminal.

```bash
catkin_make run_tests
roslaunch zebrat r1_navigation_regression.launch navigation_mode:=map
```

Runs catkin tests or the headless navigation regression launch.

## Coding Style & Naming Conventions

Python scripts target ROS Noetic Python 3. Use four-space indentation, `snake_case` for functions, variables, files, ROS params, and topics, and `CamelCase` for classes. Keep ROS node scripts executable and list new installed scripts in `zebrat/CMakeLists.txt` under `catkin_install_python`. Prefer focused YAML changes in `zebrat/config/` over hard-coded tuning values.

## Testing Guidelines

Place tests under `zebrat/test/` as `test_*.py`. Use `unittest.TestCase` and keep fast geometry or safety logic tests independent of Gazebo where possible. For launch-level behavior, prefer existing regression launch files and document required world, map, and backend arguments.

## Commit & Pull Request Guidelines

Recent history uses short, outcome-focused subject lines, often in Chinese. Keep commits concise and imperative, for example `Improve Ackermann reverse gating`. Pull requests should describe the simulator behavior changed, list commands run, note ROS/Gazebo dependencies, and include screenshots or short recordings for RViz, Gazebo, model, or navigation UI changes. Link related issues when available.

## Security & Configuration Tips

Do not commit generated `build/`, `devel/`, `install/`, transient RTAB-Map databases, or local machine paths. Keep dependency additions in `zebrat/package.xml`, and mention any required `ros-noetic-*` packages in the PR description.

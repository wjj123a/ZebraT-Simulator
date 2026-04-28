#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$ROOT_DIR/src"

use_system_ros_toolchain() {
  unset CC CXX CPPFLAGS CFLAGS CXXFLAGS LDFLAGS PYTHONHOME
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE
  unset CONDA_EXE CONDA_SHLVL _CE_CONDA _CE_M
  export PATH="/usr/bin:/bin:/usr/sbin:/sbin:/opt/ros/noetic/bin:$PATH"
  export CMAKE_PREFIX_PATH="/opt/ros/noetic"
}

source /opt/ros/noetic/setup.bash
use_system_ros_toolchain

mkdir -p "$SRC_DIR"

export DISABLE_ROS1_EOL_WARNINGS=1
export GAZEBO_MODEL_PATH="$ROOT_DIR:$ROOT_DIR/zebrat/gazebo_models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"

is_catkin_package_dir() {
  local path="$1"
  [ -d "$path" ] && [ -f "$path/package.xml" ]
}

reset_generated_workspace_if_moved() {
  local cache_file="$ROOT_DIR/build/CMakeCache.txt"

  if [ -f "$cache_file" ] && { ! grep -Fq "CMAKE_HOME_DIRECTORY:INTERNAL=$ROOT_DIR/src" "$cache_file" || grep -Fq "/miniconda3/" "$cache_file"; }; then
    echo "Detected stale or Conda-contaminated catkin cache; resetting build, devel, and install." >&2
    rm -rf "$ROOT_DIR/build" "$ROOT_DIR/devel" "$ROOT_DIR/install"
  fi
}

ensure_repo_package_link() {
  local name="$1"
  local target="$2"
  local link_path="$SRC_DIR/$name"

  if is_catkin_package_dir "$link_path"; then
    return 0
  fi

  if [ -d "$link_path" ]; then
    echo "Expected $link_path to be a catkin package or symlink, but it has no package.xml." >&2
    return 1
  fi

  if [ -L "$link_path" ] || [ -e "$link_path" ]; then
    rm -f "$link_path"
  fi

  ln -s "$target" "$link_path"
}

reset_generated_workspace_if_moved
ensure_repo_package_link "zebrat" "$ROOT_DIR/zebrat"

launch_robot="r1"
launch_navigation="true"
launch_world="$ROOT_DIR/zebrat/worlds/area.world"
launch_navigation_mode="map"
launch_navigation_backend="gmapping"
launch_map_file="$ROOT_DIR/zebrat/maps/area.yaml"
launch_rviz_config=""
save_map="false"
explicit_navigation_mode="false"
explicit_map_file="false"
explicit_rviz_config="false"

for arg in "$@"; do
  case "$arg" in
    world_name:=*)
      launch_world="${arg#world_name:=}"
      ;;
    navigation_mode:=*)
      launch_navigation_mode="${arg#navigation_mode:=}"
      explicit_navigation_mode="true"
      ;;
    navigation_backend:=*)
      launch_navigation_backend="${arg#navigation_backend:=}"
      ;;
    map_file:=*)
      launch_map_file="${arg#map_file:=}"
      explicit_map_file="true"
      ;;
    rviz_config:=*)
      launch_rviz_config="${arg#rviz_config:=}"
      explicit_rviz_config="true"
      ;;
    save_map:=*)
      save_map="${arg#save_map:=}"
      ;;
  esac
done

if [[ "$launch_world" != *"area.world" && "$launch_navigation_mode" == "map" && "$explicit_navigation_mode" != "true" ]]; then
  echo "Non-area world selected without an explicit navigation mode; falling back to slam." >&2
  launch_navigation_mode="slam"
fi

if [[ "$launch_navigation_mode" == "map" && ! -f "$launch_map_file" && "$explicit_map_file" != "true" ]]; then
  echo "Missing map file $launch_map_file; falling back to slam." >&2
  launch_navigation_mode="slam"
fi

if [[ "$explicit_rviz_config" != "true" ]]; then
  if [[ "$launch_navigation_backend" == "rtabmap" ]]; then
    launch_rviz_config="$ROOT_DIR/zebrat/rviz/rtabmap_navigation.rviz"
  else
    launch_rviz_config="$ROOT_DIR/zebrat/rviz/navigation.rviz"
  fi
fi

if [[ "$launch_navigation_mode" == "slam" || "$save_map" == "true" ]]; then
  case "$launch_navigation_backend" in
    gmapping)
      if ! rospack find gmapping >/dev/null 2>&1 && [ ! -d "$SRC_DIR/openslam_gmapping" ]; then
        git clone --depth 1 https://github.com/ros-perception/openslam_gmapping.git "$SRC_DIR/openslam_gmapping"
      fi

      if ! rospack find gmapping >/dev/null 2>&1 && [ ! -d "$SRC_DIR/slam_gmapping" ]; then
        git clone --depth 1 https://github.com/ros-perception/slam_gmapping.git "$SRC_DIR/slam_gmapping"
      fi
      ;;
    rtabmap)
      for dependency in rtabmap_ros rtabmap_slam robot_localization; do
        if ! rospack find "$dependency" >/dev/null 2>&1; then
          echo "Missing required ROS package '$dependency' for navigation_backend:=rtabmap." >&2
          exit 1
        fi
      done
      ;;
    *)
      echo "Unsupported navigation backend '$launch_navigation_backend'. Expected 'gmapping' or 'rtabmap'." >&2
      exit 1
      ;;
  esac
fi

mkdir -p "$ROOT_DIR/zebrat/maps/rtabmap"

python3 "$ROOT_DIR/zebrat/scripts/generate_local_gazebo_models.py"

cd "$ROOT_DIR"
catkin_make
source "$ROOT_DIR/devel/setup.bash"

exec roslaunch zebrat zebrat_with_world.launch robot:="$launch_robot" world_name:="$launch_world" rviz:=true rviz_config:="$launch_rviz_config" enable_navigation:="$launch_navigation" navigation_mode:="$launch_navigation_mode" navigation_backend:="$launch_navigation_backend" map_file:="$launch_map_file" "$@"

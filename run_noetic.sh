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

required_ros_packages=(
  ackermann_msgs
  controller_manager
  effort_controllers
  gazebo_ros_control
  joint_state_controller
  position_controllers
  teb_local_planner
  velocity_controllers
)

missing_ros_packages=()
for dependency in "${required_ros_packages[@]}"; do
  if ! rospack find "$dependency" >/dev/null 2>&1; then
    missing_ros_packages+=("$dependency")
  fi
done

if [ "${#missing_ros_packages[@]}" -gt 0 ]; then
  echo "Missing required ROS packages: ${missing_ros_packages[*]}" >&2
  echo "Install them with:" >&2
  echo "  sudo apt-get install ros-noetic-ackermann-msgs ros-noetic-effort-controllers ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-position-controllers ros-noetic-teb-local-planner ros-noetic-velocity-controllers" >&2
  exit 1
fi

launch_robot="r1"
launch_navigation="true"
launch_world="$ROOT_DIR/zebrat/worlds/area.world"
launch_run_mode="navigation"
launch_navigation_mode="map"
launch_navigation_backend="gmapping"
launch_map_file="$ROOT_DIR/zebrat/maps/area.yaml"
launch_rviz_config=""
launch_gazebo_gui="true"
launch_rviz="true"
launch_headless="false"
save_map="false"
explicit_navigation_mode="false"
explicit_map_file="false"
explicit_rviz_config="false"
explicit_run_mode="false"
explicit_rtabmap_delete_db_on_start="false"
rtabmap_delete_db_on_start="false"
requested_run_mode=""
pass_args=()

normalize_navigation_mode() {
  case "$1" in
    map|navigation)
      echo "map"
      ;;
    slam|mapping)
      echo "slam"
      ;;
    *)
      echo "Unsupported navigation mode '$1'. Expected map, navigation, slam, or mapping." >&2
      return 1
      ;;
  esac
}

run_mode_from_navigation_mode() {
  case "$1" in
    map)
      echo "navigation"
      ;;
    slam)
      echo "mapping"
      ;;
  esac
}

for arg in "$@"; do
  case "$arg" in
    mode:=*|run_mode:=*)
      launch_run_mode="${arg#*:=}"
      requested_run_mode="$launch_run_mode"
      explicit_run_mode="true"
      ;;
    robot:=*)
      launch_robot="${arg#robot:=}"
      ;;
    enable_navigation:=*)
      launch_navigation="${arg#enable_navigation:=}"
      ;;
    world_name:=*)
      launch_world="${arg#world_name:=}"
      ;;
    navigation_mode:=*)
      launch_navigation_mode="$(normalize_navigation_mode "${arg#navigation_mode:=}")"
      launch_run_mode="$(run_mode_from_navigation_mode "$launch_navigation_mode")"
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
    gui:=*)
      launch_gazebo_gui="${arg#gui:=}"
      ;;
    rviz:=*)
      launch_rviz="${arg#rviz:=}"
      ;;
    headless:=*)
      launch_headless="${arg#headless:=}"
      ;;
    save_map:=*)
      save_map="${arg#save_map:=}"
      ;;
    rtabmap_delete_db_on_start:=*)
      rtabmap_delete_db_on_start="${arg#rtabmap_delete_db_on_start:=}"
      explicit_rtabmap_delete_db_on_start="true"
      ;;
    *)
      pass_args+=("$arg")
      ;;
  esac
done

case "$launch_run_mode" in
  navigation|map)
    launch_run_mode="navigation"
    if [[ "$explicit_navigation_mode" != "true" ]]; then
      launch_navigation_mode="map"
    fi
    ;;
  mapping|slam)
    launch_run_mode="mapping"
    if [[ "$explicit_navigation_mode" != "true" ]]; then
      launch_navigation_mode="slam"
    fi
    if [[ "$explicit_rtabmap_delete_db_on_start" != "true" ]]; then
      rtabmap_delete_db_on_start="true"
    fi
    ;;
  *)
    echo "Unsupported mode '$launch_run_mode'. Expected mode:=navigation or mode:=mapping." >&2
    exit 1
    ;;
esac

if [[ "$explicit_run_mode" == "true" && "$explicit_navigation_mode" == "true" ]]; then
  requested_navigation_mode="$(normalize_navigation_mode "$requested_run_mode")"
  if [[ "$requested_navigation_mode" != "$launch_navigation_mode" ]]; then
    echo "Conflicting mode arguments: mode:=$requested_run_mode implies navigation_mode:=$requested_navigation_mode, but navigation_mode:=$launch_navigation_mode was also set." >&2
    exit 1
  fi
fi

if [[ "$launch_world" != *"area.world" && "$launch_navigation_mode" == "map" && "$explicit_navigation_mode" != "true" && "$explicit_run_mode" != "true" ]]; then
  echo "Non-area world selected without an explicit navigation mode; falling back to slam." >&2
  launch_navigation_mode="slam"
  launch_run_mode="mapping"
fi

if [[ "$launch_navigation_mode" == "map" && ! -f "$launch_map_file" ]]; then
  if [[ "$explicit_navigation_mode" == "true" || "$explicit_run_mode" == "true" || "$explicit_map_file" == "true" ]]; then
    echo "Missing map file $launch_map_file for navigation mode." >&2
    echo "Use mode:=mapping for an empty map, or pass map_file:=/path/to/map.yaml." >&2
    exit 1
  else
    echo "Missing map file $launch_map_file; falling back to slam." >&2
    launch_navigation_mode="slam"
    launch_run_mode="mapping"
    if [[ "$explicit_rtabmap_delete_db_on_start" != "true" ]]; then
      rtabmap_delete_db_on_start="true"
    fi
  fi
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

echo "Starting $launch_run_mode mode with navigation_mode:=$launch_navigation_mode" >&2
if [[ "$launch_navigation_mode" == "map" ]]; then
  echo "Loading map: $launch_map_file" >&2
else
  echo "Starting SLAM with an empty map; move_base remains enabled for navigation while mapping." >&2
fi
if [[ "$launch_gazebo_gui" != "true" ]]; then
  echo "Gazebo GUI disabled by request; pass gui:=true to open it." >&2
fi

exec roslaunch zebrat zebrat_with_world.launch \
  robot:="$launch_robot" \
  world_name:="$launch_world" \
  gui:="$launch_gazebo_gui" \
  headless:="$launch_headless" \
  rviz:="$launch_rviz" \
  rviz_config:="$launch_rviz_config" \
  enable_navigation:="$launch_navigation" \
  navigation_mode:="$launch_navigation_mode" \
  navigation_backend:="$launch_navigation_backend" \
  rtabmap_delete_db_on_start:="$rtabmap_delete_db_on_start" \
  map_file:="$launch_map_file" \
  save_map:="$save_map" \
  "${pass_args[@]}"

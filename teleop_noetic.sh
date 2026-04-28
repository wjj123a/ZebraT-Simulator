#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/noetic/setup.bash

unset PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER
unset CONDA_PYTHON_EXE CONDA_EXE CONDA_SHLVL _CE_CONDA _CE_M
export PATH="/usr/bin:/bin:/usr/sbin:/sbin:/opt/ros/noetic/bin:$PATH"
export CMAKE_PREFIX_PATH="/opt/ros/noetic"
export DISABLE_ROS1_EOL_WARNINGS=1

if [ -f "$ROOT_DIR/devel/setup.bash" ]; then
  source "$ROOT_DIR/devel/setup.bash"
fi

exec rosrun zebrat keyboard_teleop.py _output_topic:=/cmd_vel_teleop "$@"

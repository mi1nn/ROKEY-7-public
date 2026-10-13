#!/usr/bin/env bash
set -e
task_workspace_root="${1:?Usage: bash start_viz.sh ROBOT_WORKSPACE}"
cd "$task_workspace_root"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
task_bundle_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec ros2 run rviz2 rviz2 -d "$task_bundle_dir/search.rviz" --ros-args \
  -r __node:=rokey_search_rviz -r __ns:=/robot6 -r /tf:=tf -r /tf_static:=tf_static

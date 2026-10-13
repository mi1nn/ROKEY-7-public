#!/usr/bin/env bash
# Starts the existing Nav2 and watchdog nodes. No arm/reset/task command.
set -e
task_workspace_root="${1:?Usage: bash start_navigation.sh WORKSPACE}"
cd "$task_workspace_root"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
task_bundle_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Apply after setup files: do not inherit interactive SUPER_CLIENT mode.
export ROS_SUPER_CLIENT=False
exec ros2 launch "$task_bundle_dir/nav2_watchdog.launch.py" \
 namespace:=/robot6 watchdog_version:=search-live-20261012 \
 step_period:=0.05 heartbeat_max_age_s:=0.5 command_max_age_s:=0.5 \
 max_linear_mps:=0.30 max_angular_rps:=1.20

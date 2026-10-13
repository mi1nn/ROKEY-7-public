#!/usr/bin/env bash
set -e
task_workspace_root="${1:?Usage: bash start_localization.sh ROBOT_WORKSPACE}"
cd "$task_workspace_root"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
# Apply after setup files: do not inherit interactive SUPER_CLIENT mode.
export ROS_SUPER_CLIENT=False
exec ros2 launch turtlebot4_navigation localization.launch.py \
  namespace:=/robot6 map:="$PWD/assets/maps/pre_quake/map.yaml" use_sim_time:=false

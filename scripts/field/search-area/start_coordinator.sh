#!/usr/bin/env bash
# User-assigned provisional policy (2026-10-12). No arm or mission command.
set -e
task_workspace_root="${1:?Usage: bash start_coordinator.sh WORKSPACE}"
cd "$task_workspace_root"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
exec ros2 run rescue_coordination mission_coordinator --ros-args \
  -p "profile_path:=$PWD/config/deployment.v2.example.json" \
  -p 'entry_yaws:=[0.0, 0.0]' \
  -p policy_version:=search-live-20261012 \
  -p "executable_bases:=['PRIOR_MAP']" \
  -p task_duration_s:=120.0 \
  -p lease_duration_s:=3.0 \
  -p executor_policy_version:=search-live-20261012 \
  -p mission_frame_id:=map \
  -p authority_epoch:=66666666-6666-4666-8666-666666666666 \
  -p coordinator_id:=77777777-7777-4777-8777-777777777777 \
  -p envelope_schema_version:=rescue-envelope/1 \
  -p status_max_age_s:=3.0 \
  -p tick_period_s:=0.1 \
  -p 'route_segment_goals:=[0.0, 0.0, 0.0]' \
  -p route_revision:=0 \
  -p 'observe_goal:=[0.0, 0.0, 0.0]' \
  -p goal_map_session_id:=22222222-2222-4222-8222-222222222222 \
  -p goal_map_revision:=0 \
  -p goal_transform_revision:=1 \
  -p merge_distance_m:=0.5 \
  -p report_retry_s:=0.5 \
  -p lease_renew_period_s:=0.5 \
  -p admission_timeout_s:=10.0 \
  -p lease_message_ttl_s:=4.0 \
  -p 'route_segment_passage_free:=[false]' \
  -p live_inputs.enabled:=true \
  -p live_inputs.clock_backend:=timesyncd \
  -p live_inputs.clock_domain_id:=rokey7-utc-field \
  -p live_inputs.clock_max_age_ns:=3600000000000 \
  -p live_inputs.clock_drift_ppm:=100.0 \
  -p live_inputs.poll_period_s:=1.0 \
  -p live_inputs.search_goal_topic:=/rescue/search_goal

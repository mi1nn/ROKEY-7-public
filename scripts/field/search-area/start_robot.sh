#!/usr/bin/env bash
# Startup only: actual map read, no input files, arm or task command.
# Provisional policy selected under the user's 2026-10-12 instruction.
set -e -o pipefail
task_workspace_root="${1:?Usage: bash start_robot.sh WORKSPACE}"
cd "$task_workspace_root"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash

echo 'Reading /robot6/map to bind its actual occupancy hash (no files written).'
task_map_sha="$(timeout 15s ros2 topic echo /robot6/map nav_msgs/msg/OccupancyGrid \
  --once --full-length --qos-durability transient_local --qos-reliability reliable | python3 -c '
import sys, yaml
from nav_msgs.msg import OccupancyGrid
from rosidl_runtime_py.set_message import set_message_fields
from rescue_robot.live_sources import Grid
records = [x for x in yaml.safe_load_all(sys.stdin) if x is not None]
if len(records) != 1:
    raise SystemExit("Expected exactly one real OccupancyGrid; no hash assigned")
payload = records[0]
data = payload.get("data") if isinstance(payload, dict) else None
if not isinstance(data, list) or not all(type(x) is int for x in data):
    raise SystemExit("Map data is truncated or invalid; no occupancy hash assigned")
message = OccupancyGrid()
set_message_fields(message, payload)
grid = Grid(message)
if grid.frame != "map":
    raise SystemExit("Map frame differs from configured mission frame map; refusing identity alignment")
print(grid.sha256)
')"
if [[ ! "$task_map_sha" =~ ^[0-9a-f]{64}$ ]]; then
  echo 'Valid measured occupancy hash unavailable; task_input not started.' >&2
  exit 2
fi
echo "occupancy_sha256=$task_map_sha; mission/native frame=map; declared identity transform"
# Validity is a declared session window, not a fabricated sensor observation.
task_alignment_from_ns="$(python3 -c 'import time; print(time.time_ns()-3000000000)')"
task_alignment_until_ns="$(python3 -c 'import time; print(time.time_ns()+3600000000000)')"
exec ros2 run rescue_robot task_input --ros-args -r __ns:=/robot6 \
  -p "allowed_robot_ids:=['robot_a', 'robot_b']" \
  -p robot_id:=robot_a \
  -p namespace:=/robot6 \
  -p authority_epoch:=66666666-6666-4666-8666-666666666666 \
  -p nav_action:=/robot6/navigate_to_pose \
  -p odom_topic:=/robot6/odom \
  -p dock_status_topic:=/robot6/dock_status \
  -p policy_version:=search-live-20261012 \
  -p step_period:=0.05 \
  -p clock_error_bound_s:=2.0 \
  -p scan_topic:=/robot6/scan \
  -p tf_topic:=/robot6/tf \
  -p map_frame:=map \
  -p odom_frame:=odom \
  -p readiness.version:=search-live-20261012 \
  -p executor_heartbeat_topic:=/robot6/rescue/executor_heartbeat \
  -p policy.acceptance_timeout:=10.0 \
  -p policy.stop_timeout:=5.0 \
  -p policy.stop_window:=0.5 \
  -p policy.max_sample_gap:=0.3 \
  -p policy.evidence_max_age:=3.0 \
  -p policy.linear_stop_max:=0.01 \
  -p policy.angular_stop_max:=0.02 \
  -p readiness.max_age_s.odom:=1.0 \
  -p readiness.max_age_s.scan:=2.0 \
  -p readiness.max_age_s.camera:=2.0 \
  -p readiness.max_age_s.wheel_status:=2.0 \
  -p readiness.max_age_s.map_to_odom_tf:=2.0 \
  -p readiness.max_age_s.watchdog_heartbeat:=0.5 \
  -p "camera_topics:=['/robot6/oakd/rgb/image_raw/compressed']" \
  -p 'wheel_status_topic:=""' \
  -p watchdog_heartbeat_topic:=/robot6/rescue/watchdog_heartbeat \
  -p task.runtime_factory:=rescue_robot.task_site_runtime:create_runtime \
  -p task.input_mode:=LIVE_ROS \
  -p task.mission_frame:=map \
  -p task.schema_version:=rescue-envelope/1 \
  -p task.publisher_id:=99999999-9999-4999-8999-999999999999 \
  -p task.coordinator_id:=77777777-7777-4777-8777-777777777777 \
  -p task.coordinator_state_topic:=/rescue/system_state \
  -p task.clock_error_bound_ns:=2000000000 \
  -p task.lease.duration_s:=3.0 \
  -p task.lease.max_age_ns:=4000000000 \
  -p task.lease.max_error_ns:=2000000000 \
  -p task.lease.clock_domain_id:=rokey7-utc-field \
  -p task.mapping_authority:=88888888-8888-4888-8888-888888888888 \
  -p task.max_position_error_m:=0.1 \
  -p task.max_rotation_error_rad:=0.1 \
  -p task.input_max_age_ns:=30000000000 \
  -p task.poll_period_s:=0.5 \
  -p task.live.coordinator_service:=/mission_coordinator/task_inputs \
  -p task.live.clock_backend:=timesyncd \
  -p task.live.clock_max_age_ns:=3600000000000 \
  -p task.live.clock_drift_ppm:=100.0 \
  -p task.saved_map.map_topic:=/robot6/map \
  -p task.saved_map.localization_topic:=/robot6/amcl_pose \
  -p task.saved_map.map_frame:=map \
  -p "task.saved_map.occupancy_sha256:=$task_map_sha" \
  -p task.saved_map.map_session_id:=22222222-2222-4222-8222-222222222222 \
  -p task.saved_map.map_revision:=0 \
  -p task.saved_map.transform_revision:=1 \
  -p 'task.saved_map.transform_xy_yaw:=[0.0, 0.0, 0.0]' \
  -p "task.saved_map.valid_from_ns:=$task_alignment_from_ns" \
  -p "task.saved_map.transform_valid_until_ns:=$task_alignment_until_ns" \
  -p task.saved_map.position_error_m:=0.0 \
  -p task.saved_map.rotation_error_rad:=0.0 \
  -p task.saved_map.max_position_variance_m2:=0.25 \
  -p task.saved_map.max_yaw_variance_rad2:=0.25

# SDD v0.1 message_schemas.RecheckObservation (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# EMPTY 탐지 배열만으로 ABSENT 금지; 실제 시야/거리/센서 품질이 필요.
string observation_id
string task_id
string target_id
string robot_id
string source_boot_id
ObservationTime time
FrameRef frame_ref
geometry_msgs/Point watched_native_point
string CAMERA_STATE_PRESENT="PRESENT"
string CAMERA_STATE_ABSENT="ABSENT"
string CAMERA_STATE_UNKNOWN="UNKNOWN"
string camera_state
string LIDAR_STATE_BLOCKED="BLOCKED"
string LIDAR_STATE_CLEAR="CLEAR"
string LIDAR_STATE_UNKNOWN="UNKNOWN"
string lidar_state
string COVERAGE_STATE_VALID="VALID"
string COVERAGE_STATE_INVALID="INVALID"
string COVERAGE_STATE_UNKNOWN="UNKNOWN"
string coverage_state
int64 sensor_window_start_ns
int64 sensor_window_end_ns
string detector_health
EvidenceRef[] evidence

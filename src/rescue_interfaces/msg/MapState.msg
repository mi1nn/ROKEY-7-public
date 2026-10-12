# SDD v0.1 message_schemas.MapState (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# 단순 TF tick마다 map_revision을 올리지 않음.
Envelope envelope
string robot_id
string snapshot_id
string map_session_id
uint64 map_revision
uint64 map_update_sequence
int64 generated_at_ns
string localization_quality
string CORRECTION_DETECTION_STATUS_KNOWN="KNOWN"
string CORRECTION_DETECTION_STATUS_UNKNOWN="UNKNOWN"
string correction_detection_status
string current_native_map_frame

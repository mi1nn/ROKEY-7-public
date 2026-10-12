# SDD v0.1 message_schemas.CameraStatus (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# 정지 영상·끊김·가림은 UNKNOWN.
Envelope envelope
string camera_id
string source_boot_id
ObservationTime time
string STREAM_STATE_ONLINE="ONLINE"
string STREAM_STATE_OFFLINE="OFFLINE"
string STREAM_STATE_DEGRADED="DEGRADED"
string STREAM_STATE_UNKNOWN="UNKNOWN"
string stream_state
uint32 image_width
uint32 image_height
uint64 frame_sequence
string quality_reason

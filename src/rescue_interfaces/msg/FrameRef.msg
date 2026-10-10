# SDD v0.1 message_schemas.FrameRef (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# 실제 frame 문자열은 profile 대응.
string robot_id
string native_frame
string mission_frame
string map_session_id
uint64 map_revision
uint64 transform_revision
string ALIGNMENT_STATUS_VALID="VALID"
string ALIGNMENT_STATUS_DEGRADED="DEGRADED"
string ALIGNMENT_STATUS_INVALID="INVALID"
string alignment_status

# SDD v0.1 message_schemas.PassageAssessment (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# 측정 실패 값은 *_known=false이며 판정에 사용 금지.
string assessment_id
string route_id
uint64 route_revision
uint32 segment_index
FrameRef frame_ref
ObservationTime time
string STATUS_PASSABLE="PASSABLE"
string STATUS_BLOCKED="BLOCKED"
string STATUS_UNKNOWN="UNKNOWN"
string status
bool width_known
float64 min_width_m
bool height_known
float64 min_headroom_m
bool floor_geometry_known
float64 uncertainty_m
string footprint_profile_id
string policy_version
string reason
EvidenceRef[] evidence

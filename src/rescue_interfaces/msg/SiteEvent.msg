# SDD v0.1 message_schemas.SiteEvent (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# 재난 징후는 target/watch 비어 있음; 소실 후보는 둘 다 필수.
Envelope envelope
string event_id
string camera_id
string source_boot_id
string KIND_DISASTER_SIGN="DISASTER_SIGN"
string KIND_TARGET_DISAPPEARED_CANDIDATE="TARGET_DISAPPEARED_CANDIDATE"
string kind
ObservationTime time
string target_id
string watch_id
float64 confidence
string VISIBILITY_STATE_VALID="VALID"
string VISIBILITY_STATE_OCCLUDED="OCCLUDED"
string VISIBILITY_STATE_UNKNOWN="UNKNOWN"
string visibility_state
string[] metrics_name
float64[] metrics_value
string policy_version
EvidenceRef evidence

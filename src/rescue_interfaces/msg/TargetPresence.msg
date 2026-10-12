# SDD v0.1 message_schemas.TargetPresence (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# v0.1은 카메라별 활성 watch 1개. 여러 target 감시는 별도 전체 snapshot 타입이 필요.
Envelope envelope
string camera_id
string watch_id
string target_id
uint64 watch_revision
ObservationTime time
float64[4] roi_xyxy_normalized
string STATE_PRESENT="PRESENT"
string STATE_ABSENT_CANDIDATE="ABSENT_CANDIDATE"
string STATE_UNKNOWN="UNKNOWN"
string state
string VISIBILITY_STATE_VALID="VALID"
string VISIBILITY_STATE_OCCLUDED="OCCLUDED"
string VISIBILITY_STATE_UNKNOWN="UNKNOWN"
string visibility_state
uint32 consecutive_valid_frames
EvidenceRef evidence

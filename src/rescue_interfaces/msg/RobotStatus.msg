# SDD v0.1 message_schemas.RobotStatus (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# 미확인 값을 정상으로 채우지 않음.
Envelope envelope
string robot_id
string robot_boot_id
uint64 state_sequence
int64 observed_at_ns
string authority_epoch
string active_task_id
uint64 generation
string READINESS_READY="READY"
string READINESS_DEGRADED="DEGRADED"
string READINESS_NOT_READY="NOT_READY"
string READINESS_UNKNOWN="UNKNOWN"
string readiness
string task_state
string MOTION_STATE_STOPPED="STOPPED"
string MOTION_STATE_MOVING="MOVING"
string MOTION_STATE_STOPPING="STOPPING"
string MOTION_STATE_STOP_UNCONFIRMED="STOP_UNCONFIRMED"
string motion_state
uint32 segment_index
string stop_reason
EvidenceRef[] stop_evidence
string DOCK_STATE_DOCKED="DOCKED"
string DOCK_STATE_UNDOCKED="UNDOCKED"
string DOCK_STATE_UNKNOWN="UNKNOWN"
string dock_state
int64 dock_observed_at_ns
bool lease_valid
string sensor_quality
string nav2_state
bool battery_known
float64 battery_fraction

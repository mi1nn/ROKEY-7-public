# SDD v0.1 message_schemas.TargetEstimate (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# locations가 관측별 provenance를 보존. AWAITING_APPROVAL는 coordinator 상태이며 종료한 executor action의 feedback을 다시 발행하지 않음.
string target_id
uint64 estimate_revision
TargetLocation[] locations
int64 latest_observed_at_ns
int64 valid_until_ns
geometry_msgs/Point mission_position
float64[9] covariance
string CONFIRMATION_STATE_CANDIDATE="CANDIDATE"
string CONFIRMATION_STATE_CONFIRMED="CONFIRMED"
string CONFIRMATION_STATE_NEEDS_REOBSERVATION="NEEDS_REOBSERVATION"
string CONFIRMATION_STATE_DISMISSED="DISMISSED"
string confirmation_state
string APPROACH_STATE_UNASSIGNED="UNASSIGNED"
string APPROACH_STATE_ASSIGNED="ASSIGNED"
string APPROACH_STATE_IN_PROGRESS="IN_PROGRESS"
string APPROACH_STATE_APPROACH_COMPLETE="APPROACH_COMPLETE"
string APPROACH_STATE_BLOCKED="BLOCKED"
string APPROACH_STATE_UNREACHABLE="UNREACHABLE"
string APPROACH_STATE_CANCELED="CANCELED"
string APPROACH_STATE_FAILED="FAILED"
string approach_state
string SEGMENT_PHASE_IDLE="IDLE"
string SEGMENT_PHASE_EVALUATING="EVALUATING"
string SEGMENT_PHASE_AWAITING_APPROVAL="AWAITING_APPROVAL"
string SEGMENT_PHASE_EXECUTING="EXECUTING"
string SEGMENT_PHASE_STOPPING="STOPPING"
string SEGMENT_PHASE_STOPPED="STOPPED"
string SEGMENT_PHASE_UNKNOWN="UNKNOWN"
string segment_phase
string RESCUE_STATE_NOT_CHECKED="NOT_CHECKED"
string RESCUE_STATE_RECHECK_NEEDED="RECHECK_NEEDED"
string RESCUE_STATE_RESCUE_CONFIRMED="RESCUE_CONFIRMED"
string rescue_state
string persisted_report_id

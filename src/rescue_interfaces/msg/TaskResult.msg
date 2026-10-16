# SDD v0.1 message_schemas.TaskResult (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# 구간 성공을 접근/구조 성공으로 바꾸지 않음.
string task_id
string command_id
string robot_id
uint64 generation
string OUTCOME_COMPLETED="COMPLETED"
string OUTCOME_CANCELED="CANCELED"
string OUTCOME_EXPIRED="EXPIRED"
string OUTCOME_UNREACHABLE="UNREACHABLE"
string OUTCOME_FAILED="FAILED"
string outcome
string COMPLETION_SCOPE_SEARCH="SEARCH"
string COMPLETION_SCOPE_SEGMENT="SEGMENT"
string COMPLETION_SCOPE_TARGET_APPROACH="TARGET_APPROACH"
string COMPLETION_SCOPE_RECHECK="RECHECK"
string COMPLETION_SCOPE_RETURN="RETURN"
string COMPLETION_SCOPE_HOLD="HOLD"
string COMPLETION_SCOPE_SPIN="SPIN"
string completion_scope
string terminal_reason
string original_cause
bool stop_confirmed
string motion_state
RobotPose final_pose
string[] used_record_ids
FrameRef frame_ref
RecheckObservation recheck
bool has_recheck
EvidenceRef[] stop_evidence

# SDD v0.1 message_schemas.RescueReport (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# 해당 기록이 없는 optional section은 명시적으로 빈 배열/has flag.
Envelope envelope
string report_id
string target_id
string task_id
uint64 generation
string[] record_ids
TargetLocation[] locations
SiteEvent[] site_events
RecheckObservation[] rechecks
RouteCandidate[] routes
OperatorApproval[] approvals
TaskAdmission[] task_admissions
bool has_task_result
TaskResult task_result
string confirmation_state
string approach_state
string rescue_state
EvidenceRef[] evidence_refs
int64 first_issued_at_ns
string meaning_sha256

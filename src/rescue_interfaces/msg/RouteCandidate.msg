# SDD v0.1 message_schemas.RouteCandidate (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# 대상 좌표·접근 목표는 다른 값.
string route_id
uint64 route_revision
string robot_id
string target_id
uint32 segment_index
FrameRef frame_ref
nav_msgs/Path planned_path
geometry_msgs/PoseStamped approach_goal
int64 generated_at_ns
int64 valid_until_ns
string path_sha256
string approval_corridor_sha256
PassageAssessment assessment
string planner_evidence_id
string policy_version

# SDD v0.1 message_schemas.OperatorApproval (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# 같은 구간의 같은 의미만 멱등 허용.
string approval_id
string command_id
string operator_id
string robot_id
string robot_boot_id
string authority_epoch
string target_id
uint64 target_estimate_revision
string route_id
uint64 route_revision
uint32 segment_index
int64 approved_at_ns
int64 valid_until_ns
string SCOPE_ONE_SEGMENT="ONE_SEGMENT"
string scope

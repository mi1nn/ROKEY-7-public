# SDD v0.1 message_schemas.ObservationTime (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# 없으면 has=false; 0으로 성공을 표현하지 않음.
int64 observed_at_ns
int64 valid_until_ns
string STAMP_SEMANTICS_CAPTURE="CAPTURE"
string STAMP_SEMANTICS_DRIVER_RECEIVE="DRIVER_RECEIVE"
string STAMP_SEMANTICS_UNKNOWN="UNKNOWN"
string stamp_semantics
string clock_domain_id
string CLOCK_STATE_SYNCED="SYNCED"
string CLOCK_STATE_UNKNOWN="UNKNOWN"
string clock_state
bool has_clock_error_bound
uint64 clock_error_bound_ns

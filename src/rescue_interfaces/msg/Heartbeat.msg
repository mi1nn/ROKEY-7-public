# SDD v0.1 message_schemas.Heartbeat (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# 수신자의 local monotonic watchdog과 별개.
Envelope envelope
string robot_id
string PRODUCER_ROLE_EXECUTOR="EXECUTOR"
string PRODUCER_ROLE_WATCHDOG="WATCHDOG"
string producer_role
string robot_boot_id
string authority_epoch
uint64 generation
int64 emitted_at_ns
bool command_fenced

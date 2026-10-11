# SDD v0.1 message_schemas.CommandLease (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# 만료 lease의 늦은 갱신은 부활 금지.
Envelope envelope
string lease_id
string robot_id
string robot_boot_id
string coordinator_id
string authority_epoch
uint64 generation
uint64 renewal_sequence
int64 issued_at_ns
int64 valid_until_ns

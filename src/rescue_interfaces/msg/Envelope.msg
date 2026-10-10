# SDD v0.1 message_schemas.Envelope (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# 전송 identity와 원 관측 identity를 분리.
string schema_version
string publisher_id
string publisher_boot_id
uint64 sequence
int64 published_at_ns

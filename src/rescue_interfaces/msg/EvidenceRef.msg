# SDD v0.1 message_schemas.EvidenceRef (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# bytes는 HTTP로 별도 전송; hash·원 시각 보존.
string evidence_id
string sha256
string mime_type
string source_id
string source_boot_id
int64 observed_at_ns

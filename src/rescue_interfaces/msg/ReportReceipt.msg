# SDD v0.1 message_schemas.ReportReceipt (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# included_record_ids는 저장한 보고의 내용과 일치.
Envelope envelope
string receipt_id
string report_id
string meaning_sha256
string[] included_record_ids
string STATE_RECEIVED="RECEIVED"
string STATE_PENDING_EVIDENCE="PENDING_EVIDENCE"
string STATE_PERSISTED="PERSISTED"
string STATE_REJECTED="REJECTED"
string state
int64 persisted_at_ns
bool has_persisted_at
uint64 storage_revision
string reason_code

# SDD v0.1 message_schemas.TargetSnapshot (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# 전체 상태 snapshot. 웹의 저장 완료 좌표 표시는 operations 장부 기준.
Envelope envelope
string snapshot_id
TargetEstimate[] targets

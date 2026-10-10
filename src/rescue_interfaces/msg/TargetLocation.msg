# SDD v0.1 message_schemas.TargetLocation (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# UNKNOWN 위치 수치는 사용 금지. 동일 location_id 재발행의 의미 payload는 불변.
Envelope envelope
string location_id
TargetObservation observation
FrameRef frame_ref
string PROJECTION_STATUS_VALID="VALID"
string PROJECTION_STATUS_STALE_DERIVATION="STALE_DERIVATION"
string PROJECTION_STATUS_NEEDS_REOBSERVATION="NEEDS_REOBSERVATION"
string PROJECTION_STATUS_EXPIRED="EXPIRED"
string projection_status
geometry_msgs/Point mission_position
float64[9] covariance

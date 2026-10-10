# SDD v0.1 message_schemas.TargetObservation (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# 탐지와 좌표 성공은 별도.
Envelope envelope
string observation_id
string source_robot
string source_camera
string source_boot_id
ObservationTime time
string class_label
float64 confidence
string model_id
string model_sha256
string tracking_mode
bool has_track_id
string track_id
float64[4] bbox_xyxy_pixels
string sensor_frame
string LOCALIZATION_STATUS_VALID="VALID"
string LOCALIZATION_STATUS_UNKNOWN="UNKNOWN"
string LOCALIZATION_STATUS_INVALID="INVALID"
string localization_status
geometry_msgs/Point sensor_position
float64[9] position_covariance
string depth_status
EvidenceRef evidence

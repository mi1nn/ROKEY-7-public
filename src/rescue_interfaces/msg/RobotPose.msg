# SDD v0.1 message_schemas.RobotPose (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# header stamp·frame과 time/ref 일치 검사.
Envelope envelope
string robot_id
string source_boot_id
ObservationTime time
geometry_msgs/PoseWithCovarianceStamped native_pose
geometry_msgs/PoseWithCovarianceStamped mission_pose
FrameRef frame_ref
string MISSION_POSE_STATUS_VALID="VALID"
string MISSION_POSE_STATUS_UNKNOWN="UNKNOWN"
string MISSION_POSE_STATUS_INVALID="INVALID"
string mission_pose_status

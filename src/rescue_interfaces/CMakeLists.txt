cmake_minimum_required(VERSION 3.8)
project(rescue_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Phase 1 of SDD v0.1 (contracts.md C-10). tests/test_interfaces_schema.py checks this list.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Envelope.msg"
  "msg/ObservationTime.msg"
  "msg/FrameRef.msg"
  "msg/EvidenceRef.msg"
  "msg/RobotStatus.msg"
  "msg/RobotPose.msg"
  "msg/Heartbeat.msg"
  "msg/CommandLease.msg"
  "msg/TaskAdmission.msg"
  "msg/TaskResult.msg"
  "msg/OperatorApproval.msg"
  "msg/RouteCandidate.msg"
  "msg/SiteEvent.msg"
  "msg/TargetObservation.msg"
  "msg/TargetLocation.msg"
  "msg/MapState.msg"
  "msg/ReportReceipt.msg"
  "msg/RecheckObservation.msg"
  "msg/PassageAssessment.msg"
  "msg/TargetPresence.msg"
  "msg/CameraStatus.msg"
  "msg/SystemState.msg"
  "msg/RescueReport.msg"
  "msg/TargetEstimate.msg"
  "msg/TargetSnapshot.msg"
  "srv/RequestStop.srv"
  "srv/RegisterAuthority.srv"
  "srv/ResetStop.srv"
  "srv/SubmitOperatorCommand.srv"
  "srv/SetCameraWatch.srv"
  "action/ExecuteTask.action"
  "action/ObserveTarget.action"
  DEPENDENCIES geometry_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

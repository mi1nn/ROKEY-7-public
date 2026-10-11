---
title: 단일 웹캠·구간 승인 SDD 노드 및 인터페이스 검토안 v0.1
status: draft-proposal
updated: 2026-10-09
---

# ROKEY-7 SDD 노드·인터페이스 정의 v0.1

상태: **[제안, 2026-10-09] 논리 계약 초안**. 실제 `.msg/.srv/.action` 생성·컴파일 또는 ROS 연결 검증 결과가 아니다.

이 문서는 PM 측 Codex 세션에서 정의한 **웹캠 1대·관리자 구간 승인 시나리오의 검토안**이다. [기계 판독 명세 JSON](sdd-node-contracts-v0.1.json), [발행·수신 연결표 CSV](sdd-node-connections-v0.1.csv), [편집 가능한 draw.io 다이어그램](rokey7-sdd.drawio)을 함께 제공한다. JSON의 `nodes`·`interfaces`가 노드와 연결의 원본이며 CSV는 source×target 관계를 한 행씩 펼친 것이다. 이 JSON은 현재 `config/` 배포 프로파일이나 이미 생성된 ROS IDL이 아니다.

## 기존 팀 설계와의 적용 경계

**이 문서 등록은 기존 팀 BRD·실행 계약의 채택 결정을 바꾸지 않는다.** 이 PR의 base `d03b6c8`에 머지된 팀 문서 기준은 [범위 RQ-08·RQ-09와 Q](../requirements/scope.md), [BRD Part I](../requirements/brd-part1-v0.1.md), [계약 C-01~C-09](contracts.md), [상태 전이](state-machines.md), [네트워크 채택안](network-discovery.md)이다. 아래 차이는 단일 웹캠 시연을 검토하기 위한 별도 제안이며, 기존 Python 계약에 바로 넣을 수 있는 호환 타입이라고 주장하지 않는다.

| 항목 | 기존 문서·Phase 1 배치안 | 이 v0.1의 검토안 | 채택 전 필요한 작업 |
| --- | --- | --- | --- |
| 고정 카메라·구출 조건 | 카메라 A 재난 징후 + B 출구 카운트 + AMR1 재관측, 부재만으로 완료 금지(Q-10·Q-15) | 구조 지점을 계속 촬영하는 웹캠 하나의 소실 후보 + fresh AMR1 관측. 사람 이송을 증명하지 않는 **시연 판정** | 카메라 1대 결정에 맞춘 scope·SiteEvent·완료 상태/정책·인수 시험 개정은 소유자의 SoT PR과 연결. 이 PR은 기존 실행 코드를 직접 바꾸지 않음 |
| AMR2 출동·이동 | 자동 출동을 기본 후보로 두고 승인 버튼은 선택(Q-12), 기존 task kind 집합(C-06) | 관리자 출동 승인 + `MOVE_SEGMENT`마다 “계속” 승인, `RETURN_HOME` 추가 | 명령/승인·TaskResult·상태 전이·프로파일 및 실행기/관제 소비자 개정 |
| 중앙 배치 | 로봇 담당 PC 둘 + coordination/operations 분리 후보 | PC3 웹캠, PC4 coordination·operations·웹·DB 동거 | 4-PC 부하·장애 검증과 프로파일 개정. PC1~4는 도면의 논리 배치이며 실제 호스트 이름이 아님 |
| Discovery Server | 로봇별 온보드 서버 토폴로지 A 권고 | PC4 서버 하나에 참여자를 연결한 논리 발견 상세도 | 온보드/오프보드 연결·서버 목록·고유 ID를 별도 결정. 이 도면으로 네트워크 채택안을 덮어쓰지 않음. domain ID만으로 격리된다고 가정하지 않음 |
| watchdog 위치 | 로봇 담당 PC에서 실행하는 Phase 1 방향([#35 OPS-17 결정](https://github.com/jaebeom/ROKEY-7/pull/35#issuecomment-6058147235)) | 도면의 AMR1·AMR2 온보드 watchdog(`PROPOSED_NOT_INSTALLED`) | 온보드 배치는 채택·설치 검증이 필요한 별도 제안. PC↔로봇 링크 단절을 처리하는 Create 3 timeout은 실측 전이며 TR-11·T-05에서 확인. 도면으로 Phase 1 배치 결정을 변경하지 않음 |
| ROS 타입·HTTP | C-08의 토픽/API 제안과 현재 Python DTO 의미 | 필드·서비스·액션·HTTP 경로의 상세 초안 | IDL 생성/컴파일, Python DTO 변환과 인증·배선·QoS 검증. 경로 이름을 곧바로 실제 서버의 주소로 사용하지 않음 |

[검토 기록, 2026-10-09] [OPS-20 ② 갱신](https://github.com/jaebeom/ROKEY-7/pull/44#issuecomment-6078811090)은 카메라 1대 사용이 저장소 소유자 확인으로 결정됐고, 카메라 B·`EXIT_PASS`·출구 카운트 상한을 제외해 소실 후보를 재확인 트리거로 사용한다고 기록한다. 그 결정의 팀 범위 SoT는 저장소 소유자의 별도 PR에서 관리한다. 위 표의 카메라 A+B는 이 PR base 문서의 비교 기준이며 카메라 대수를 다시 결정 대기에 둔다는 뜻이 아니다. 이 PR의 [Q-25](../requirements/scope.md)는 승인 관문 3곳과 통로 판정 범위로 한정한다.

PM 측 Codex 세션의 문서 등록 요청(2026-10-09): “해당 인터페이스 명세에 대한 내용이 https://github.com/jaebeom/ROKEY-7/tree/main 여기에 없다면 PR 넣어줘.” 적용 범위는 명세의 저장소 등록·PR 검토이며, Q-10·Q-12·Q-15·Q-22의 채택이나 실기 실행 승인이 아니다. 앞선 PM 측 세션의 시나리오 확인 원문은 “인형이 보이는 구조 지점을 계속 촬영”이고, PC별 배치 요청 원문은 “PC1에 AMR1에 대한 내용이 있고, PC2에 AMR2가 있고, PC3에 CAM, PC4에 웹”이다(이 세션의 기록일 2026-10-09).

## 적용 시나리오와 현재 증거

- [PM 요구, 2026-10-09] PC1=AMR1 정찰·재확인, PC2=AMR2 구급상자·구간 이동, PC3=웹캠 1대, PC4=웹·API·DB·임무 조정·Discovery. 실제 호스트 이름/IP/로봇 ID는 profile로 대응한다.
- [PM 요구, 2026-10-09] 고정 웹캠은 인형이 보이는 구조 지점을 계속 촬영한다. 인형 소실은 완료 후보이며 가림·stream failure는 UNKNOWN.
- [PM 요구, 2026-10-09] AMR2는 운영자 출동 승인 후 정지점 한 구간씩 저속 이동하고, 매번 “계속”으로 다음 구간을 승인한다. 사람 다리 추종은 이번 계약에 넣지 않는다.
- [문서 확인, 2026-10-09] 확인 기준은 `origin/main` SHA `d03b6c8d70db49e098818b4d8463b22c258d9065`. `git ls-tree -r --name-only origin/main docs`와 `git grep -n -E 'SubmitOperatorCommand|SetCameraWatch|EvaluatePassage|MOVE_SEGMENT|ABSENT_CANDIDATE|PrepareMapping|ExecuteTask|sdd-node|rokey7-sdd' origin/main -- docs README.md`로 검사했다. 기존 ExecuteTask/RequestStop 의미는 contracts에 있으나, 이 상세 명세·JSON·CSV·draw.io는 해당 main의 docs에 없었다. 이 검색은 런타임 구현 유무나 열린 PR의 구현 완료를 판정하지 않는다.
- [기존 계약] 명령 소유·원 관측 시각·좌표 revision·실제 정지·영속 receipt 원칙은 유지한다. 새 타입과 완료/승인 시나리오의 차이는 위 적용 경계 및 아래 확장 목록을 따른다. 저장소 실행 코드·실물 설정은 변경하지 않는다.

## 이름과 통신 원칙

`amr1`/`amr2`, `cam_fixed`는 도면의 논리 예시 이름이다. profile의 실제 ID/namespace/native topic으로 대응하며 sensor topic remap만으로 frame ID가 고유해졌다고 보지 않는다. `/rescue/robots/{robot_id}/...`는 공통 계약, `/{robot_id}/...`는 해당 로봇의 native sensor/Nav2 계약이다.

Topic은 연속 관측·상태, Service는 짧은 요청/접수, Action은 종료까지 오래 걸리며 feedback/cancel이 필요한 수색·이동·평가에 사용한다. 웹↔operations, 영상/증거/지도 파일은 HTTP·SSE로 정의한다. service accepted, action accepted, action COMPLETED, DB PERSISTED는 각각 다른 상태다. [ROS 2 Jazzy 인터페이스 공식 정의](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Basic/About-Interfaces.rst)

단일 ROS 그래프 직접 연결은 배치 제안이다. robot domains가 분리되면 도메인별 gateway를 추가하고 같은 논리 계약을 보존한다. Discovery Server는 endpoint 발견 정보를 전달하며 application data broker로 그리지 않는다. 실제 server 목록/ID/domain/participant 설정은 실기 전에 검증한다.

## 시나리오별 발행·수신 순서

1. PC3 capture → quake/presence/video. 웹은 PC3 video gateway를 HTTP로 조회한다.
2. quake → `/rescue/site_events(DISASTER_SIGN)` → coordinator. 유효 영상/시간·중복 event ID·판정 정책을 검사해 DISASTER를 표시하고 근거를 저장한다. 근거 PERSISTED 뒤 AMR1 `PrepareMapping(START)`로 기준 지도 기반 SLAM 시작/준비를 요청한다. fresh map/TF·로봇 준비/권한/lease가 유효하면 설정한 재난 대응 정책이 AMR1 `ExecuteTask(SEARCH_AREA)`를 한 번 발급한다. 트리거 자체는 주행 명령이 아니며, AMR2의 출동/계속은 사용자 승인 관문을 갖는다. AMR2 위치 추정용 backend는 대기 단계의 PREPARE/START로 준비하되 이동하지 않는다.
3. AMR1 perception → 원 관측 → mapping → mission-frame TargetLocation → coordinator. 유효 target ID/좌표·보고를 만들고 `/rescue/reports` → operations → DB·필수 증거 commit → `/rescue/report_receipts(PERSISTED)` 순서를 거친다. 웹 지도는 operations 장부에서 조회한다.
4. target 저장·좌표·AMR2 준비가 유효하면 coordinator → AMR2 `EvaluateRoute`. 실행기가 planner에 경로를 묻고 필요 구간의 3D passage 평가를 얻는다. 승인 가능한 구간·접근 목표·평가 근거를 웹에 표시한다.
5. DISPATCH_SUPPORT 승인 → `ExecuteTask(MOVE_SEGMENT)` 한 구간. 도착 후 실제 정지와 Result를 확인해 coordinator의 segment_phase=AWAITING_APPROVAL로 표시한다. CONTINUE_SEGMENT마다 새 command/task/generation과 ONE_SEGMENT 승인을 발급한다. 최종 구간 뒤 AMR2 `REOBSERVE_TARGET` → `ObserveTarget`으로 인형을 재탐지한다.
6. 고정 webcam target/ROI 연결은 검증된 projection 또는 운영자 ROI 선택으로 만든다. coordinator → `SetCameraWatch`. 유효 시야에서 연속 소실이면 `TARGET_DISAPPEARED_CANDIDATE(target_id,watch_id)`를 발행한다. track ID만으로 로봇 대상과 동일 인형이라고 가정하지 않는다.
7. coordinator → AMR1 `ExecuteTask(REOBSERVE_TARGET)` → 해당 지점 이동·정지 → `ObserveTarget`. 결과는 camera, LiDAR, coverage를 각각 전달한다. 이미 오래된 대상 좌표는 제한된 재수색 정책으로 새로 검증하거나 재관측을 보류한다.
8. 같은 사건의 유효 소실 후보 + 소실 후 취득한 AMR1 관측 창 + camera ABSENT + LiDAR CLEAR + coverage VALID + 동일 좌표 세션/품질이 모두 확인되면 `RESCUE_CONFIRMED`를 **시연 판정**으로 기록한다. 현재 두 입력의 clock 오차와 원 시각으로 선후 관계를 검사한다. 하나라도 UNKNOWN이면 RECHECK_NEEDED다. 2D LiDAR CLEAR는 사람의 부재나 실제 구출을 단독 증명하지 않는다.
9. 완료 보고의 PERSISTED 이후 조정기는 각 로봇의 home_ref를 현재 지도 기준에서 검증해 `RETURN_HOME` task를 배정한다. 두 로봇 모두 종료·STOPPED와 필수 보고 PERSISTED 뒤 MISSION_FINISHED. 운영 API의 RETURN_HOME/FINISH_MISSION은 수동 요청도 같은 관문으로 처리하는 인터페이스다. 도킹/언도킹은 제조사 플랫폼 확인 후 별도 운용 action 경로로 매핑하며 이 도면이 장치 API를 새로 확정하지 않는다.

## 노드 책임 목록

표의 ROS_NODE는 논리 실행 노드다. driver는 실제 제조사 프로세스에 대응하며 개수를 실측했다고 주장하지 않는다. standard costmap은 각 Nav2 server가 소유하는 내부 노드로 구분한다.
| ID | 배치 | 이름 | 종류 | 책임 | 상태 |
| --- | --- | --- | --- | --- | --- |
| amr1.base | AMR1 | base_driver | ROS_NODE | odom·장치 상태·구동 입력. 실제 제조사 노드 이름/형식은 배포 프로파일로 대응 | NATIVE_BINDING_UNVERIFIED |
| amr1.lidar | AMR1 | lidar_driver | ROS_NODE | 2D LaserScan 취득 | NATIVE_BINDING_UNVERIFIED |
| amr1.rgbd | AMR1 | rgbd_driver | ROS_NODE | RGB·정렬 Depth·CameraInfo 취득. RGB-D 장착/정합 확인 필요 | NATIVE_BINDING_UNVERIFIED |
| amr1.static_tf | AMR1 | static_tf_publisher | ROS_NODE | 실측한 base↔센서 고정 변환; 장비 robot_state_publisher 등에 대응 | NATIVE_BINDING_UNVERIFIED |
| amr1.watchdog | AMR1 | command_watchdog_node | ROS_NODE | PC 연결과 독립된 최종 명령 만료·stop fence·장치 출력 억제. 설치/지원은 미확인 | PROPOSED_NOT_INSTALLED |
| amr1.slam | PC1 | slam_toolbox | ROS_NODE | scan·TF로 local map 및 map→odom 제공. backend 채택 제안 | STANDARD_CANDIDATE |
| amr1.perception | PC1 | target_perception_node | ROS_NODE | 인형 탐지·sensor-frame 위치·증거·관측 창 기반 재확인. 주행 권한 없음 | PROPOSED |
| amr1.mapping | PC1 | mapping_adapter_node | ROS_NODE | SLAM 준비 요청·원 관측 시각 TF·mission 위치·지도 revision·지도 변화 표시 자료·pose snapshot | PROPOSED |
| amr1.executor | PC1 | task_executor_node | ROS_NODE | 임무/승인/lease 검증·readiness·Nav2 호출·취소·실제 정지 판정 | PARTIAL_ADAPTER_EXISTS_NEW_ENDPOINTS_PROPOSED |
| amr1.bt | PC1 | bt_navigator | ROS_NODE | NavigateToPose 서버; 지정한 BT가 planner/controller를 호출 | STANDARD_CANDIDATE |
| amr1.planner | PC1 | planner_server | ROS_NODE | ComputePathToPose 서버; global costmap 기반 경로 계산 | STANDARD_CANDIDATE |
| amr1.controller | PC1 | controller_server | ROS_NODE | FollowPath 서버; local costmap 기반 제어 속도 | STANDARD_CANDIDATE |
| amr1.global_costmap | PC1 | global_costmap | ROS_NODE | map·scan·TF 기반 전역 비용지도; planner의 내부 소유 노드 | STANDARD_CANDIDATE |
| amr1.local_costmap | PC1 | local_costmap | ROS_NODE | scan·TF 기반 지역 비용지도; controller의 내부 소유 노드 | STANDARD_CANDIDATE |
| amr1.smoother | PC1 | velocity_smoother | ROS_NODE | 속도 평활화; 낮은 속도 제한은 회차 프로파일로 설정 | STANDARD_CANDIDATE |
| amr1.collision | PC1 | collision_monitor | ROS_NODE | fresh scan·footprint 기반 최종 Nav2 속도 보호 | STANDARD_CANDIDATE |
| amr2.base | AMR2 | base_driver | ROS_NODE | odom·장치 상태·구동 입력. 실제 제조사 노드 이름/형식은 배포 프로파일로 대응 | NATIVE_BINDING_UNVERIFIED |
| amr2.lidar | AMR2 | lidar_driver | ROS_NODE | 2D LaserScan 취득 | NATIVE_BINDING_UNVERIFIED |
| amr2.rgbd | AMR2 | rgbd_driver | ROS_NODE | RGB·정렬 Depth·CameraInfo 취득. RGB-D 장착/정합 확인 필요 | NATIVE_BINDING_UNVERIFIED |
| amr2.static_tf | AMR2 | static_tf_publisher | ROS_NODE | 실측한 base↔센서 고정 변환; 장비 robot_state_publisher 등에 대응 | NATIVE_BINDING_UNVERIFIED |
| amr2.watchdog | AMR2 | command_watchdog_node | ROS_NODE | PC 연결과 독립된 최종 명령 만료·stop fence·장치 출력 억제. 설치/지원은 미확인 | PROPOSED_NOT_INSTALLED |
| amr2.slam | PC2 | slam_toolbox | ROS_NODE | scan·TF로 local map 및 map→odom 제공. backend 채택 제안 | STANDARD_CANDIDATE |
| amr2.perception | PC2 | target_perception_node | ROS_NODE | 인형 탐지·sensor-frame 위치·증거·관측 창 기반 재확인. 주행 권한 없음 | PROPOSED |
| amr2.mapping | PC2 | mapping_adapter_node | ROS_NODE | SLAM 준비 요청·원 관측 시각 TF·mission 위치·지도 revision·지도 변화 표시 자료·pose snapshot | PROPOSED |
| amr2.executor | PC2 | task_executor_node | ROS_NODE | 임무/승인/lease 검증·readiness·Nav2 호출·취소·실제 정지 판정 | PARTIAL_ADAPTER_EXISTS_NEW_ENDPOINTS_PROPOSED |
| amr2.bt | PC2 | bt_navigator | ROS_NODE | NavigateToPose 서버; 지정한 BT가 planner/controller를 호출 | STANDARD_CANDIDATE |
| amr2.planner | PC2 | planner_server | ROS_NODE | ComputePathToPose 서버; global costmap 기반 경로 계산 | STANDARD_CANDIDATE |
| amr2.controller | PC2 | controller_server | ROS_NODE | FollowPath 서버; local costmap 기반 제어 속도 | STANDARD_CANDIDATE |
| amr2.global_costmap | PC2 | global_costmap | ROS_NODE | map·scan·TF 기반 전역 비용지도; planner의 내부 소유 노드 | STANDARD_CANDIDATE |
| amr2.local_costmap | PC2 | local_costmap | ROS_NODE | scan·TF 기반 지역 비용지도; controller의 내부 소유 노드 | STANDARD_CANDIDATE |
| amr2.smoother | PC2 | velocity_smoother | ROS_NODE | 속도 평활화; 낮은 속도 제한은 회차 프로파일로 설정 | STANDARD_CANDIDATE |
| amr2.collision | PC2 | collision_monitor | ROS_NODE | fresh scan·footprint 기반 최종 Nav2 속도 보호 | STANDARD_CANDIDATE |
| amr1.search | PC1 | search_candidate_node | ROS_NODE | 지정 구역의 waypoint/frontier 후보 생성. Nav2 goal 발행 없음 | PROPOSED |
| amr2.passage | PC2 | passage_evaluator_node | ROS_NODE | RGB-D로 평가 경로의 폭·바닥·천장 여유 측정; PASSABLE/BLOCKED/UNKNOWN | PROPOSED |
| cam.capture | PC3 | webcam_capture_node | ROS_NODE | USB 웹캠 원 영상·관측 시각·카메라 상태 | PROPOSED |
| cam.quake | PC3 | quake_detector_node | ROS_NODE | 영상 엣지 흔들림을 DISASTER_SIGN 증거로 발행 | PROPOSED |
| cam.presence | PC3 | target_presence_node | ROS_NODE | 연결된 target/ROI 감시; 가림·유효 시야를 검사해 소실 후보 발행 | PROPOSED |
| cam.video | PC3 | video_gateway | PROCESS | 웹캠 HTTP 영상 스트림; 영상 인코딩/서빙 | PROPOSED |
| central.frames | PC4 | frame_registry_node | ROS_NODE | 공통 mission 정렬의 단일 authority. 로봇별 map session/revision 변화에 정렬 무효화 | PROPOSED |
| central.coordinator | PC4 | mission_coordinator_node | ROS_NODE | 사건 연결·대상 ID·출동/구간 승인·임무 세대·완료 시연 판정 | PROPOSED |
| central.operations | PC4 | operations_node | ROS_NODE | HTTP API·ROS 어댑터·보고/증거 수신·DB 단일 쓰기·조회·SSE | PROPOSED |
| central.web | PC4 | web_console | WEB_FRONTEND | 영상·지도·상태·승인/계속/정지 UI. ROS 직접 연결 없음 | PROPOSED |
| central.db | PC4 | DB / evidence store | STORE | operations만 쓰는 장부·증거 저장 | PROPOSED |
| central.discovery | PC4 | Fast DDS Discovery Server | DISCOVERY_PROCESS | 참여자/endpoint 발견. 애플리케이션 데이터 중계 없음 | DEPLOYMENT_PROPOSED |

## 토픽 연결 정의

`IDL` 열: `1차` 는 `src/rescue_interfaces` 에 정의가 있는 사용자 정의 타입, `2차` 는 아직 정의가 없는 타입, `표준` 은 ROS·Nav2 표준 타입이다. 기준은 [contracts C-10](contracts.md)(2026-10-10)이고 아래 서비스·액션·메시지 표에도 같은 표시를 둔다. IDL 정의가 있다는 것이지 그 통신을 하는 노드가 구현됐다는 뜻은 아니다.

| ID | endpoint | 타입 | 발행자/Client | 구독자/Server | 전달 의미 | QoS/상태 | IDL |
| --- | --- | --- | --- | --- | --- | --- | --- |
| IF-001 | /amr1/scan | sensor_msgs/msg/LaserScan | amr1.lidar | amr1.slam, amr1.executor, amr1.perception, amr1.global_costmap, amr1.local_costmap, amr1.collision | 장애물·SLAM·readiness·재확인 | BEST_EFFORT / VOLATILE / KEEP_LAST 1; native publisher QoS 확인; STANDARD_BINDING_PROPOSED | 표준 |
| IF-002 | /amr1/odom | nav_msgs/msg/Odometry | amr1.base | amr1.executor, amr1.controller, amr1.mapping, amr1.smoother | 속도·pose·정지 증거 | BEST_EFFORT / VOLATILE / KEEP_LAST 1; native publisher QoS 확인; STANDARD_BINDING_PROPOSED | 표준 |
| IF-003 | /amr1/rgb/image_raw | sensor_msgs/msg/Image | amr1.rgbd | amr1.perception, amr1.executor | RGB/Depth/CameraInfo 정합; 실제 native 이름 remap | BEST_EFFORT / VOLATILE / KEEP_LAST 1; native publisher QoS 확인; STANDARD_BINDING_PROPOSED | 표준 |
| IF-004 | /amr1/depth/image_raw | sensor_msgs/msg/Image | amr1.rgbd | amr1.perception, amr1.executor | RGB/Depth/CameraInfo 정합; 실제 native 이름 remap | BEST_EFFORT / VOLATILE / KEEP_LAST 1; native publisher QoS 확인; STANDARD_BINDING_PROPOSED | 표준 |
| IF-005 | /amr1/rgb/camera_info | sensor_msgs/msg/CameraInfo | amr1.rgbd | amr1.perception, amr1.executor | RGB/Depth/CameraInfo 정합; 실제 native 이름 remap | BEST_EFFORT / VOLATILE / KEEP_LAST 1; native publisher QoS 확인; STANDARD_BINDING_PROPOSED | 표준 |
| IF-006 | /amr1/tf | tf2_msgs/msg/TFMessage | amr1.base, amr1.slam | amr1.slam, amr1.mapping, amr1.executor, amr1.perception, amr1.planner, amr1.controller, amr1.global_costmap, amr1.local_costmap, amr1.collision | 각 로봇 전용 TF buffer; edge authority 중복 금지 | 표준 TF QoS; 설치 확인; STANDARD_BINDING_PROPOSED | 표준 |
| IF-007 | /amr1/tf_static | tf2_msgs/msg/TFMessage | amr1.static_tf | amr1.slam, amr1.mapping, amr1.executor, amr1.perception, amr1.planner, amr1.controller, amr1.global_costmap, amr1.local_costmap, amr1.collision | 고정 센서 TF; 두 로봇 접두 없는 frame 혼합 금지 | 표준 static TF QoS; STANDARD_BINDING_PROPOSED | 표준 |
| IF-008 | /amr1/map | nav_msgs/msg/OccupancyGrid | amr1.slam | amr1.global_costmap, amr1.mapping, amr1.search | 자기 local map. AMR1 지도를 AMR2에 그대로 goal로 적용하지 않음 | RELIABLE / TRANSIENT_LOCAL / KEEP_LAST 1; 최신 전체 snapshot; STANDARD_BINDING_PROPOSED | 표준 |
| IF-009 | /rescue/robots/amr1/map_state | rescue_interfaces/msg/MapState | amr1.mapping | central.frames, central.coordinator, central.operations | session/revision/quality; map 단독 stamp와 구분 | RELIABLE / TRANSIENT_LOCAL / KEEP_LAST 1; 최신 전체 snapshot; PROPOSED | 1차 |
| IF-010 | /rescue/robots/amr1/pose | rescue_interfaces/msg/RobotPose | amr1.mapping | central.coordinator, central.operations | mission pose + native pose + 정렬/시각/불확실성 | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; PROPOSED | 1차 |
| IF-011 | /rescue/robots/amr1/status | rescue_interfaces/msg/RobotStatus | amr1.executor | central.coordinator, central.operations | task/readiness/motion/segment 상태; retained는 live 증거 아님 | RELIABLE / TRANSIENT_LOCAL / KEEP_LAST 1; 최신 전체 snapshot; PROPOSED | 1차 |
| IF-012 | /rescue/robots/amr1/heartbeat | rescue_interfaces/msg/Heartbeat | amr1.executor | central.coordinator, central.operations, amr1.watchdog | live executor heartbeat | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; PROPOSED | 1차 |
| IF-013 | /rescue/robots/amr1/watchdog_heartbeat | rescue_interfaces/msg/Heartbeat | amr1.watchdog | amr1.executor, central.coordinator, central.operations | 장치 최종 감시 생존·fence 상태 | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; PROPOSED | 1차 |
| IF-014 | /rescue/robots/amr1/command_lease | rescue_interfaces/msg/CommandLease | central.coordinator | amr1.executor | 단일 authority/generation의 기간 갱신. 새 이동 goal 아님 | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; PROPOSED | 1차 |
| IF-015 | /rescue/robots/amr1/command_permit | rescue_interfaces/msg/CommandPermit | amr1.executor | amr1.watchdog | 유효 permit/heartbeat/속도 중 하나라도 만료 시 장치 출력 억제 | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; PROPOSED | 2차 |
| IF-016 | /rescue/robots/amr1/task_admission | rescue_interfaces/msg/TaskAdmission | amr1.executor | central.coordinator, central.operations | 수락/거절 및 거절 이유; action Result와 구분 | RELIABLE / VOLATILE / KEEP_LAST 10; 중요 사건 outbox + 저장 receipt; PROPOSED | 1차 |
| IF-027 | /amr1/cmd_vel_nav | geometry_msgs/msg/Twist | amr1.controller | amr1.smoother | Nav2 제어 속도; Jazzy 기준, native TwistStamped 사용 여부 확인 | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; STANDARD_BINDING_PROPOSED | 표준 |
| IF-028 | /amr1/cmd_vel_smoothed | geometry_msgs/msg/Twist | amr1.smoother | amr1.collision | 평활화 속도 → Collision Monitor | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; STANDARD_BINDING_PROPOSED | 표준 |
| IF-029 | /amr1/cmd_vel_safe | geometry_msgs/msg/Twist | amr1.collision | amr1.watchdog | Nav2 보호 이후 장치 watchdog으로 전달 | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; BINDING_PROPOSED_NOT_INSTALLED | 표준 |
| IF-030 | /amr1/cmd_vel | geometry_msgs/msg/Twist | amr1.watchdog | amr1.base | 실제 구동 최종 입력. 다른 publish 경로는 허용하지 않음 | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; BINDING_PROPOSED_NOT_INSTALLED | 표준 |
| IF-033 | /amr2/scan | sensor_msgs/msg/LaserScan | amr2.lidar | amr2.slam, amr2.executor, amr2.perception, amr2.global_costmap, amr2.local_costmap, amr2.collision | 장애물·SLAM·readiness·재확인 | BEST_EFFORT / VOLATILE / KEEP_LAST 1; native publisher QoS 확인; STANDARD_BINDING_PROPOSED | 표준 |
| IF-034 | /amr2/odom | nav_msgs/msg/Odometry | amr2.base | amr2.executor, amr2.controller, amr2.mapping, amr2.smoother | 속도·pose·정지 증거 | BEST_EFFORT / VOLATILE / KEEP_LAST 1; native publisher QoS 확인; STANDARD_BINDING_PROPOSED | 표준 |
| IF-035 | /amr2/rgb/image_raw | sensor_msgs/msg/Image | amr2.rgbd | amr2.perception, amr2.executor, amr2.passage | RGB/Depth/CameraInfo 정합; 실제 native 이름 remap | BEST_EFFORT / VOLATILE / KEEP_LAST 1; native publisher QoS 확인; STANDARD_BINDING_PROPOSED | 표준 |
| IF-036 | /amr2/depth/image_raw | sensor_msgs/msg/Image | amr2.rgbd | amr2.perception, amr2.executor, amr2.passage | RGB/Depth/CameraInfo 정합; 실제 native 이름 remap | BEST_EFFORT / VOLATILE / KEEP_LAST 1; native publisher QoS 확인; STANDARD_BINDING_PROPOSED | 표준 |
| IF-037 | /amr2/rgb/camera_info | sensor_msgs/msg/CameraInfo | amr2.rgbd | amr2.perception, amr2.executor, amr2.passage | RGB/Depth/CameraInfo 정합; 실제 native 이름 remap | BEST_EFFORT / VOLATILE / KEEP_LAST 1; native publisher QoS 확인; STANDARD_BINDING_PROPOSED | 표준 |
| IF-038 | /amr2/tf | tf2_msgs/msg/TFMessage | amr2.base, amr2.slam | amr2.slam, amr2.mapping, amr2.executor, amr2.perception, amr2.planner, amr2.controller, amr2.global_costmap, amr2.local_costmap, amr2.collision, amr2.passage | 각 로봇 전용 TF buffer; edge authority 중복 금지 | 표준 TF QoS; 설치 확인; STANDARD_BINDING_PROPOSED | 표준 |
| IF-039 | /amr2/tf_static | tf2_msgs/msg/TFMessage | amr2.static_tf | amr2.slam, amr2.mapping, amr2.executor, amr2.perception, amr2.planner, amr2.controller, amr2.global_costmap, amr2.local_costmap, amr2.collision, amr2.passage | 고정 센서 TF; 두 로봇 접두 없는 frame 혼합 금지 | 표준 static TF QoS; STANDARD_BINDING_PROPOSED | 표준 |
| IF-040 | /amr2/map | nav_msgs/msg/OccupancyGrid | amr2.slam | amr2.global_costmap, amr2.mapping | 자기 local map. AMR1 지도를 AMR2에 그대로 goal로 적용하지 않음 | RELIABLE / TRANSIENT_LOCAL / KEEP_LAST 1; 최신 전체 snapshot; STANDARD_BINDING_PROPOSED | 표준 |
| IF-041 | /rescue/robots/amr2/map_state | rescue_interfaces/msg/MapState | amr2.mapping | central.frames, central.coordinator, central.operations | session/revision/quality; map 단독 stamp와 구분 | RELIABLE / TRANSIENT_LOCAL / KEEP_LAST 1; 최신 전체 snapshot; PROPOSED | 1차 |
| IF-042 | /rescue/robots/amr2/pose | rescue_interfaces/msg/RobotPose | amr2.mapping | central.coordinator, central.operations | mission pose + native pose + 정렬/시각/불확실성 | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; PROPOSED | 1차 |
| IF-043 | /rescue/robots/amr2/status | rescue_interfaces/msg/RobotStatus | amr2.executor | central.coordinator, central.operations | task/readiness/motion/segment 상태; retained는 live 증거 아님 | RELIABLE / TRANSIENT_LOCAL / KEEP_LAST 1; 최신 전체 snapshot; PROPOSED | 1차 |
| IF-044 | /rescue/robots/amr2/heartbeat | rescue_interfaces/msg/Heartbeat | amr2.executor | central.coordinator, central.operations, amr2.watchdog | live executor heartbeat | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; PROPOSED | 1차 |
| IF-045 | /rescue/robots/amr2/watchdog_heartbeat | rescue_interfaces/msg/Heartbeat | amr2.watchdog | amr2.executor, central.coordinator, central.operations | 장치 최종 감시 생존·fence 상태 | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; PROPOSED | 1차 |
| IF-046 | /rescue/robots/amr2/command_lease | rescue_interfaces/msg/CommandLease | central.coordinator | amr2.executor | 단일 authority/generation의 기간 갱신. 새 이동 goal 아님 | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; PROPOSED | 1차 |
| IF-047 | /rescue/robots/amr2/command_permit | rescue_interfaces/msg/CommandPermit | amr2.executor | amr2.watchdog | 유효 permit/heartbeat/속도 중 하나라도 만료 시 장치 출력 억제 | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; PROPOSED | 2차 |
| IF-048 | /rescue/robots/amr2/task_admission | rescue_interfaces/msg/TaskAdmission | amr2.executor | central.coordinator, central.operations | 수락/거절 및 거절 이유; action Result와 구분 | RELIABLE / VOLATILE / KEEP_LAST 10; 중요 사건 outbox + 저장 receipt; PROPOSED | 1차 |
| IF-059 | /amr2/cmd_vel_nav | geometry_msgs/msg/Twist | amr2.controller | amr2.smoother | Nav2 제어 속도; Jazzy 기준, native TwistStamped 사용 여부 확인 | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; STANDARD_BINDING_PROPOSED | 표준 |
| IF-060 | /amr2/cmd_vel_smoothed | geometry_msgs/msg/Twist | amr2.smoother | amr2.collision | 평활화 속도 → Collision Monitor | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; STANDARD_BINDING_PROPOSED | 표준 |
| IF-061 | /amr2/cmd_vel_safe | geometry_msgs/msg/Twist | amr2.collision | amr2.watchdog | Nav2 보호 이후 장치 watchdog으로 전달 | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; BINDING_PROPOSED_NOT_INSTALLED | 표준 |
| IF-062 | /amr2/cmd_vel | geometry_msgs/msg/Twist | amr2.watchdog | amr2.base | 실제 구동 최종 입력. 다른 publish 경로는 허용하지 않음 | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; BINDING_PROPOSED_NOT_INSTALLED | 표준 |
| IF-065 | /rescue/observations | rescue_interfaces/msg/TargetObservation | amr1.perception, amr2.perception | amr1.mapping, amr2.mapping | 연속 원 관측; 각 mapping은 자기 source_robot만 소비 | BEST_EFFORT / VOLATILE / KEEP_LAST 10; PROPOSED | 1차 |
| IF-066 | /rescue/target_locations | rescue_interfaces/msg/TargetLocation | amr1.mapping, amr2.mapping | central.coordinator | 첫 발견·중요 좌표 변화·원 관측/근거; 저장 receipt까지 outbox 재전송 | RELIABLE / VOLATILE / KEEP_LAST 10; 중요 사건 outbox + 저장 receipt; PROPOSED | 1차 |
| IF-067 | /rescue/frame_alignments | rescue_interfaces/msg/FrameAlignmentSnapshot | central.frames | amr1.mapping, amr2.mapping, amr1.executor, amr2.executor, central.coordinator, central.operations | 두 로봇 mission 정렬의 전체 snapshot | RELIABLE / TRANSIENT_LOCAL / KEEP_LAST 1; 최신 전체 snapshot; PROPOSED | 2차 |
| IF-068 | /rescue/robots/amr1/search_candidates | rescue_interfaces/msg/SearchCandidates | amr1.search | amr1.executor | frontier/waypoint 후보; search task 없이 스스로 이동시키지 않음 | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; PROPOSED | 2차 |
| IF-070 | /rescue/targets | rescue_interfaces/msg/TargetSnapshot | central.coordinator | central.operations | target 전체 snapshot; target 좌표와 approach/segment goal 구분 | RELIABLE / TRANSIENT_LOCAL / KEEP_LAST 1; 최신 전체 snapshot; PROPOSED | 1차 |
| IF-071 | /rescue/system_state | rescue_interfaces/msg/SystemState | central.coordinator | central.operations | NORMAL/DISASTER/MISSION_FINISHED 및 원인 event/report IDs, 임무 phase·트리거·출동 구간(C-10 2026-10-11) | RELIABLE / TRANSIENT_LOCAL / KEEP_LAST 1; 최신 전체 snapshot; PROPOSED | 1차 |
| IF-072 | /rescue/reports | rescue_interfaces/msg/RescueReport | central.coordinator | central.operations | 대상/임무/재확인 기록; 같은 report_id·payload 재시도 | RELIABLE / VOLATILE / KEEP_LAST 10; 중요 사건 outbox + 저장 receipt; PROPOSED | 1차 |
| IF-073 | /rescue/report_receipts | rescue_interfaces/msg/ReportReceipt | central.operations | central.coordinator, amr1.mapping, amr2.mapping, amr1.executor, amr2.executor, cam.quake, cam.presence | DB commit·필수 증거 저장 뒤 PERSISTED; included_record_ids로 outbox 정리 | RELIABLE / VOLATILE / KEEP_LAST 10; 중요 사건 outbox + 저장 receipt; PROPOSED | 1차 |
| IF-074 | /rescue/cameras/cam_fixed/image_raw | sensor_msgs/msg/Image | cam.capture | cam.quake, cam.presence, cam.video | 인형이 있는 고정 시야의 웹캠 원 영상 | BEST_EFFORT / VOLATILE / KEEP_LAST 1; native publisher QoS 확인; PROPOSED | 표준 |
| IF-075 | /rescue/cameras/cam_fixed/status | rescue_interfaces/msg/CameraStatus | cam.capture | cam.quake, cam.presence, central.coordinator, central.operations | stream/clock/frame 품질; 오프라인을 인형 소실로 처리하지 않음 | RELIABLE / VOLATILE / KEEP_LAST 1; stamp/boot/유효기한 검사; PROPOSED | 2차 |
| IF-076 | /rescue/site_events | rescue_interfaces/msg/SiteEvent | cam.quake, cam.presence | central.coordinator | DISASTER_SIGN 또는 TARGET_DISAPPEARED_CANDIDATE 증거 | RELIABLE / VOLATILE / KEEP_LAST 10; 중요 사건 outbox + 저장 receipt; PROPOSED | 1차 |
| IF-077 | /rescue/cameras/cam_fixed/presence | rescue_interfaces/msg/TargetPresence | cam.presence | central.coordinator, central.operations | target/ROI 별 PRESENT/ABSENT_CANDIDATE/UNKNOWN; 가림 상태 포함 | RELIABLE / TRANSIENT_LOCAL / KEEP_LAST 1; 최신 전체 snapshot; PROPOSED | 2차 |

## 서비스 연결 정의

| ID | endpoint | 타입 | 발행자/Client | 구독자/Server | 전달 의미 | QoS/상태 | IDL |
| --- | --- | --- | --- | --- | --- | --- | --- |
| IF-017 | /rescue/robots/amr1/register_authority | rescue_interfaces/srv/RegisterAuthority | central.coordinator | amr1.executor | stopped/disarmed에서 새 authority 등록; 등록만으로 이동 없음 | N/A; PROPOSED | 1차 |
| IF-018 | /rescue/robots/amr1/request_stop | rescue_interfaces/srv/RequestStop | central.coordinator, central.operations | amr1.executor | 빠른 정지 접수; 실제 STOPPED는 후속 status/관측으로 확인 | N/A; PROPOSED | 1차 |
| IF-019 | /rescue/robots/amr1/reset_stop | rescue_interfaces/srv/ResetStop | central.coordinator | amr1.executor | 원인 해결·실제 정지·새 권한 검증 뒤 software latch 해제 | N/A; PROPOSED | 1차 |
| IF-049 | /rescue/robots/amr2/register_authority | rescue_interfaces/srv/RegisterAuthority | central.coordinator | amr2.executor | stopped/disarmed에서 새 authority 등록; 등록만으로 이동 없음 | N/A; PROPOSED | 1차 |
| IF-050 | /rescue/robots/amr2/request_stop | rescue_interfaces/srv/RequestStop | central.coordinator, central.operations | amr2.executor | 빠른 정지 접수; 실제 STOPPED는 후속 status/관측으로 확인 | N/A; PROPOSED | 1차 |
| IF-051 | /rescue/robots/amr2/reset_stop | rescue_interfaces/srv/ResetStop | central.coordinator | amr2.executor | 원인 해결·실제 정지·새 권한 검증 뒤 software latch 해제 | N/A; PROPOSED | 1차 |
| IF-078 | /rescue/cameras/cam_fixed/set_watch | rescue_interfaces/srv/SetCameraWatch | central.coordinator | cam.presence | target_id와 ROI 연결을 설정; 확인된 보정 또는 운영자 ROI 연결 필요 | N/A; PROPOSED | 2차 |
| IF-079 | /rescue/operator_command | rescue_interfaces/srv/SubmitOperatorCommand | central.operations | central.coordinator | 출동·계속·재확인·복귀/종료 요청 접수; 긴 실행은 ExecuteTask로 연결 | N/A; PROPOSED | 1차 |

## 액션 연결 정의

| ID | endpoint | 타입 | 발행자/Client | 구독자/Server | 전달 의미 | QoS/상태 | IDL |
| --- | --- | --- | --- | --- | --- | --- | --- |
| IF-020 | /rescue/robots/amr1/prepare_mapping | rescue_interfaces/action/PrepareMapping | central.coordinator | amr1.mapping | baseline map/profile로 SLAM 준비·시작 요청 후 fresh map/TF 확인. 주행 없음 | N/A; PROPOSED | 2차 |
| IF-021 | /rescue/robots/amr1/execute_task | rescue_interfaces/action/ExecuteTask | central.coordinator | amr1.executor | 수색/재관측/접근/한 구간 이동/복귀의 Goal→Feedback→Result | N/A; PROPOSED | 1차 |
| IF-022 | /rescue/robots/amr1/evaluate_route | rescue_interfaces/action/EvaluateRoute | central.coordinator | amr1.executor | 주행 없이 접근·다음 구간의 planner 경로와 승인용 RouteCandidate 생성 | N/A; PROPOSED | 2차 |
| IF-023 | /rescue/robots/amr1/observe_target | rescue_interfaces/action/ObserveTarget | amr1.executor | amr1.perception | 관측 창을 실제 취득해 카메라·LiDAR·시야 유효성을 결과로 반환 | N/A; PROPOSED | 1차 |
| IF-024 | /amr1/navigate_to_pose | nav2_msgs/action/NavigateToPose | amr1.executor | amr1.bt | 표준 Nav2 주행; 프로젝트 호출자는 executor 한 곳 | N/A; STANDARD_JAZZY | 표준 |
| IF-025 | /amr1/compute_path_to_pose | nav2_msgs/action/ComputePathToPose | amr1.executor, amr1.bt | amr1.planner | 경로 조회; 결과만으로 주행/통로 통과 성공 아님 | N/A; STANDARD_JAZZY | 표준 |
| IF-026 | /amr1/follow_path | nav2_msgs/action/FollowPath | amr1.bt | amr1.controller | Nav2 내부 경로 제어 | N/A; STANDARD_JAZZY | 표준 |
| IF-052 | /rescue/robots/amr2/prepare_mapping | rescue_interfaces/action/PrepareMapping | central.coordinator | amr2.mapping | baseline map/profile로 SLAM 준비·시작 요청 후 fresh map/TF 확인. 주행 없음 | N/A; PROPOSED | 2차 |
| IF-053 | /rescue/robots/amr2/execute_task | rescue_interfaces/action/ExecuteTask | central.coordinator | amr2.executor | 수색/재관측/접근/한 구간 이동/복귀의 Goal→Feedback→Result | N/A; PROPOSED | 1차 |
| IF-054 | /rescue/robots/amr2/evaluate_route | rescue_interfaces/action/EvaluateRoute | central.coordinator | amr2.executor | 주행 없이 접근·다음 구간의 planner 경로와 승인용 RouteCandidate 생성 | N/A; PROPOSED | 2차 |
| IF-055 | /rescue/robots/amr2/observe_target | rescue_interfaces/action/ObserveTarget | amr2.executor | amr2.perception | 관측 창을 실제 취득해 카메라·LiDAR·시야 유효성을 결과로 반환 | N/A; PROPOSED | 1차 |
| IF-056 | /amr2/navigate_to_pose | nav2_msgs/action/NavigateToPose | amr2.executor | amr2.bt | 표준 Nav2 주행; 프로젝트 호출자는 executor 한 곳 | N/A; STANDARD_JAZZY | 표준 |
| IF-057 | /amr2/compute_path_to_pose | nav2_msgs/action/ComputePathToPose | amr2.executor, amr2.bt | amr2.planner | 경로 조회; 결과만으로 주행/통로 통과 성공 아님 | N/A; STANDARD_JAZZY | 표준 |
| IF-058 | /amr2/follow_path | nav2_msgs/action/FollowPath | amr2.bt | amr2.controller | Nav2 내부 경로 제어 | N/A; STANDARD_JAZZY | 표준 |
| IF-069 | /rescue/robots/amr2/evaluate_passage | rescue_interfaces/action/EvaluatePassage | amr2.executor | amr2.passage | route/segment 및 로봇 box의 3D 여유 평가; 주행 없음 | N/A; PROPOSED | 2차 |

## HTTP·웹 연결 정의

| ID | endpoint | 타입 | 발행자/Client | 구독자/Server | 전달 의미 | QoS/상태 |
| --- | --- | --- | --- | --- | --- | --- |
| IF-080 | POST /api/operator-commands | JSON OperatorCommand + accepted/request_id | central.web | central.operations | 세션 인증된 운영 요청; STOP_ROBOT는 operations가 RequestStop로 우선 전달, 나머지는 coordinator 서비스로 전달 | N/A; PROPOSED |
| IF-081 | GET /api/snapshot + GET /api/events (SSE) | JSON ConsoleSnapshot / EventStream | central.operations | central.web | DB에 저장된 대상 좌표·보고 상태와 수신 시각이 있는 live 로봇 상태 | N/A; PROPOSED |
| IF-082 | GET /video/cam_fixed.mjpg | multipart/x-mixed-replace | cam.video | central.web | PC3의 HTTP 영상 gateway를 PC4 브라우저가 조회 | N/A; PROPOSED |
| IF-083 | PUT /api/evidence/{evidence_id} | bytes + sha256 -> PERSISTED/REJECTED | amr1.perception, amr2.perception, cam.quake, cam.presence, amr2.passage | central.operations | 증거 원 bytes와 hash; ROS event는 EvidenceRef만 사용 | N/A; PROPOSED |
| IF-084 | PUT /api/maps/{snapshot_id} | MapSnapshot: grid + MapState + baseline-diff + sha256 | amr1.mapping, amr2.mapping | central.operations | 지도 내용/metadata/변화 표시 자료를 같은 snapshot ID로 원자적으로 저장 | N/A; PROPOSED |

## 내부 호출 연결 정의

| ID | endpoint | 타입 | 발행자/Client | 구독자/Server | 전달 의미 | QoS/상태 |
| --- | --- | --- | --- | --- | --- | --- |
| IF-031 | global_costmap access | nav2_costmap_2d internal API | amr1.global_costmap | amr1.planner | planner가 자기 Costmap2DROS 객체를 사용; ROS topic으로 꾸미지 않음 | N/A; STANDARD_CANDIDATE |
| IF-032 | local_costmap access | nav2_costmap_2d internal API | amr1.local_costmap | amr1.controller | controller가 자기 Costmap2DROS 객체를 사용 | N/A; STANDARD_CANDIDATE |
| IF-063 | global_costmap access | nav2_costmap_2d internal API | amr2.global_costmap | amr2.planner | planner가 자기 Costmap2DROS 객체를 사용; ROS topic으로 꾸미지 않음 | N/A; STANDARD_CANDIDATE |
| IF-064 | local_costmap access | nav2_costmap_2d internal API | amr2.local_costmap | amr2.controller | controller가 자기 Costmap2DROS 객체를 사용 | N/A; STANDARD_CANDIDATE |
| IF-085 | ledger / evidence store transaction | SQL + file store | central.operations | central.db | DB 단일 writer 및 보고/증거 commit | N/A; PROPOSED |

## Discovery 연결 정의

| ID | endpoint | 타입 | 발행자/Client | 구독자/Server | 전달 의미 | QoS/상태 |
| --- | --- | --- | --- | --- | --- | --- |
| IF-086 | DDS discovery metadata | Fast DDS discovery protocol | amr1.base, amr1.lidar, amr1.rgbd, amr1.static_tf, amr1.watchdog, amr1.slam, amr1.perception, amr1.mapping, amr1.executor, amr1.bt, amr1.planner, amr1.controller, amr1.global_costmap, amr1.local_costmap, amr1.smoother, amr1.collision, amr2.base, amr2.lidar, amr2.rgbd, amr2.static_tf, amr2.watchdog, amr2.slam, amr2.perception, amr2.mapping, amr2.executor, amr2.bt, amr2.planner, amr2.controller, amr2.global_costmap, amr2.local_costmap, amr2.smoother, amr2.collision, amr1.search, amr2.passage, cam.capture, cam.quake, cam.presence, central.frames, central.coordinator, central.operations | central.discovery | 참여자 ↔ 서버 발견 정보. Topic/Service/Action 데이터 흐름과 분리 | N/A; DEPLOYMENT_PROPOSED |

## 사용자 정의 메시지 필수 필드

UUID는 문자열 UUID, 거리 m·각도 rad·시간 정수 ns. enum은 ROS 직렬화 상수로 고정하기 전의 논리 어휘다. 배열 상한·문자열 상한·minor 호환·오류 코드 번호는 IDL 구현 전에 고정한다. `has_*` false인 payload를 0 좌표/거리로 해석하지 않는다.

| 메시지/구조 | 필수 필드와 의미 | IDL |
| --- | --- | --- |
| Envelope | schema_version:string, publisher_id:string, publisher_boot_id:UUID string, sequence:uint64, published_at_ns:int64. 전송 identity와 원 관측 identity를 분리. | 1차 |
| ObservationTime | observed_at_ns:int64, valid_until_ns:int64, stamp_semantics:string(CAPTURE/DRIVER_RECEIVE/UNKNOWN), clock_domain_id:string, clock_state:string(SYNCED/UNKNOWN), has_clock_error_bound:bool, clock_error_bound_ns:uint64. 없으면 has=false; 0으로 성공을 표현하지 않음. | 1차 |
| EvidenceRef | evidence_id:UUID string, sha256:string, mime_type:string, source_id:string, source_boot_id:UUID string, observed_at_ns:int64. bytes는 HTTP로 별도 전송; hash·원 시각 보존. | 1차 |
| FrameRef | robot_id:string, native_frame:string, mission_frame:string, map_session_id:UUID string, map_revision:uint64, transform_revision:uint64, alignment_status:string(VALID/DEGRADED/INVALID). 실제 frame 문자열은 profile 대응. | 1차 |
| TargetObservation | envelope:Envelope, observation_id:UUID, source_robot:string, source_camera:string, source_boot_id:UUID, time:ObservationTime, class_label:string, confidence:float64, model_id:string, model_sha256:string, tracking_mode:string, has_track_id:bool, track_id:string, bbox_xyxy_pixels:float64[4], sensor_frame:string, localization_status:string(VALID/UNKNOWN/INVALID), sensor_position:geometry_msgs/Point, position_covariance:float64[9], depth_status:string, evidence:EvidenceRef. 탐지와 좌표 성공은 별도. | 1차 |
| TargetLocation | envelope:Envelope, location_id:UUID, observation:TargetObservation, frame_ref:FrameRef, projection_status:string(VALID/STALE_DERIVATION/NEEDS_REOBSERVATION/EXPIRED), mission_position:geometry_msgs/Point, covariance:float64[9]. UNKNOWN 위치 수치는 사용 금지. 동일 location_id 재발행의 의미 payload는 불변. | 1차 |
| MapState | envelope:Envelope, robot_id:string, snapshot_id:UUID, map_session_id:UUID, map_revision:uint64, map_update_sequence:uint64, generated_at_ns:int64, localization_quality:string, correction_detection_status:string(KNOWN/UNKNOWN), current_native_map_frame:string. 단순 TF tick마다 map_revision을 올리지 않음. | 1차 |
| RobotPose | envelope:Envelope, robot_id:string, source_boot_id:UUID, time:ObservationTime, native_pose:geometry_msgs/PoseWithCovarianceStamped, mission_pose:geometry_msgs/PoseWithCovarianceStamped, frame_ref:FrameRef, mission_pose_status:string(VALID/UNKNOWN/INVALID). header stamp·frame과 time/ref 일치 검사. | 1차 |
| FrameAlignmentSnapshot | envelope:Envelope, snapshot_id:UUID, alignments:FrameAlignment[]. 전체 snapshot이며 delta와 혼합하지 않음. | 2차 |
| FrameAlignment | frame_ref:FrameRef, authority_id:string, transform:geometry_msgs/Transform, covariance:float64[36], valid_from_ns:int64, valid_until_ns:int64, status:string(VALID/DEGRADED/INVALID), calibration_evidence:EvidenceRef[]. 계측/검증 없는 alignment는 INVALID. | 2차 |
| SearchCandidates | envelope:Envelope, robot_id:string, area_ref:string, frame_ref:FrameRef, generated_at_ns:int64, valid_until_ns:int64, candidate_poses:geometry_msgs/PoseStamped[], policy_version:string. 실제 주행은 executor의 유효 SEARCH_AREA task 안에서만. | 2차 |
| Heartbeat | envelope:Envelope, robot_id:string, producer_role:string(EXECUTOR/WATCHDOG), robot_boot_id:UUID, authority_epoch:UUID, generation:uint64, emitted_at_ns:int64, command_fenced:bool. 수신자의 local monotonic watchdog과 별개. | 1차 |
| CommandLease | envelope:Envelope, lease_id:UUID, robot_id:string, robot_boot_id:UUID, coordinator_id:string, authority_epoch:UUID, generation:uint64, renewal_sequence:uint64, issued_at_ns:int64, valid_until_ns:int64. 만료 lease의 늦은 갱신은 부활 금지. | 1차 |
| CommandPermit | envelope:Envelope, permit_id:UUID, robot_id:string, robot_boot_id:UUID, authority_epoch:UUID, generation:uint64, lease_id:UUID, mode:string(ALLOW/FENCED), issued_at_ns:int64, valid_until_ns:int64, reason:string. 장치 로컬 watchdog은 유효 permit·fresh heartbeat·fresh 속도를 모두 확인. | 2차 |
| RobotStatus | envelope:Envelope, robot_id:string, robot_boot_id:UUID, state_sequence:uint64, observed_at_ns:int64, authority_epoch:UUID, active_task_id:string, generation:uint64, readiness:string(READY/DEGRADED/NOT_READY/UNKNOWN), task_state:string, motion_state:string(STOPPED/MOVING/STOPPING/STOP_UNCONFIRMED), segment_index:uint32, stop_reason:string, stop_evidence:EvidenceRef[], dock_state:string(DOCKED/UNDOCKED/UNKNOWN), dock_observed_at_ns:int64, lease_valid:bool, sensor_quality:string, nav2_state:string, battery_known:bool, battery_fraction:float64. 미확인 값을 정상으로 채우지 않음. | 1차 |
| TaskAdmission | envelope:Envelope, admission_id:UUID, command_id:UUID, task_id:UUID, robot_id:string, robot_boot_id:UUID, authority_epoch:UUID, generation:uint64, accepted:bool, reason_code:string, observed_at_ns:int64. goal accepted=false의 상세 이유 및 수락 장부의 근거. | 1차 |
| TargetEstimate | target_id:UUID, estimate_revision:uint64, locations:TargetLocation[], latest_observed_at_ns:int64, valid_until_ns:int64, mission_position:geometry_msgs/Point, covariance:float64[9], confirmation_state:string(CANDIDATE/CONFIRMED/NEEDS_REOBSERVATION/DISMISSED), approach_state:string(UNASSIGNED/ASSIGNED/IN_PROGRESS/APPROACH_COMPLETE/BLOCKED/UNREACHABLE/CANCELED/FAILED), segment_phase:string(IDLE/EVALUATING/AWAITING_APPROVAL/EXECUTING/STOPPING/STOPPED/UNKNOWN), rescue_state:string(NOT_CHECKED/RECHECK_NEEDED/RESCUE_CONFIRMED), persisted_report_id:string. locations가 관측별 provenance를 보존. AWAITING_APPROVAL는 coordinator 상태이며 종료한 executor action의 feedback을 다시 발행하지 않음. | 1차 |
| TargetSnapshot | envelope:Envelope, snapshot_id:UUID, targets:TargetEstimate[]. 전체 상태 snapshot. 웹의 저장 완료 좌표 표시는 operations 장부 기준. | 1차 |
| SystemState | envelope:Envelope, state_sequence:uint64, mode:string(NORMAL/DISASTER/MISSION_FINISHED), changed_at_ns:int64, cause_event_ids:UUID[], active_task_ids:UUID[], policy_version:string, reason:string, phase:string(IDLE/DISASTER/SEARCHING/TARGET_FOUND/DISPATCH_APPROVED/SEGMENT_MOVING/AWAITING_CONTINUE/ARRIVED/CANDIDATE/RECHECKING/RESCUE_CONFIRMED/RECHECK_NEEDED/RETURNING/FINISHED), has_trigger:bool, trigger_event_id:UUID, trigger_observed_at_ns:int64, has_dispatch:bool, target_id:UUID, target_revision:uint64, route_revision:uint64, segment_index:uint32, segment_count:uint32, segment_phase:string(IDLE/EVALUATING/AWAITING_APPROVAL/EXECUTING/STOPPING/STOPPED/UNKNOWN). phase 이하는 2026-10-11 1차 채택 때 임무 화면용으로 더함(contracts C-10). Phase 1 은 AMR2 출동 대상이 한 번에 하나라 출동 정보를 이 snapshot 에 둔다 | 1차 |
| SiteEvent | envelope:Envelope, event_id:UUID, camera_id:string(cam_fixed 예시), source_boot_id:UUID, kind:string(DISASTER_SIGN/TARGET_DISAPPEARED_CANDIDATE), time:ObservationTime, target_id:string, watch_id:string, confidence:float64, visibility_state:string(VALID/OCCLUDED/UNKNOWN), metrics_name:string[], metrics_value:float64[], policy_version:string, evidence:EvidenceRef. 재난 징후는 target/watch 비어 있음; 소실 후보는 둘 다 필수. | 1차 |
| CameraStatus | envelope:Envelope, camera_id:string, source_boot_id:UUID, time:ObservationTime, stream_state:string(ONLINE/OFFLINE/DEGRADED/UNKNOWN), image_width:uint32, image_height:uint32, frame_sequence:uint64, quality_reason:string. 정지 영상·끊김·가림은 UNKNOWN. | 2차 |
| TargetPresence | envelope:Envelope, camera_id:string, watch_id:UUID, target_id:UUID, watch_revision:uint64, time:ObservationTime, roi_xyxy_normalized:float64[4], state:string(PRESENT/ABSENT_CANDIDATE/UNKNOWN), visibility_state:string(VALID/OCCLUDED/UNKNOWN), consecutive_valid_frames:uint32, evidence:EvidenceRef. v0.1은 카메라별 활성 watch 1개. 여러 target 감시는 별도 전체 snapshot 타입이 필요. | 2차 |
| RecheckObservation | observation_id:UUID, task_id:UUID, target_id:UUID, robot_id:string, source_boot_id:UUID, time:ObservationTime, frame_ref:FrameRef, watched_native_point:geometry_msgs/Point, camera_state:string(PRESENT/ABSENT/UNKNOWN), lidar_state:string(BLOCKED/CLEAR/UNKNOWN), coverage_state:string(VALID/INVALID/UNKNOWN), sensor_window_start_ns:int64, sensor_window_end_ns:int64, detector_health:string, evidence:EvidenceRef[]. EMPTY 탐지 배열만으로 ABSENT 금지; 실제 시야/거리/센서 품질이 필요. | 1차 |
| PassageAssessment | assessment_id:UUID, route_id:UUID, route_revision:uint64, segment_index:uint32, frame_ref:FrameRef, time:ObservationTime, status:string(PASSABLE/BLOCKED/UNKNOWN), width_known:bool, min_width_m:float64, height_known:bool, min_headroom_m:float64, floor_geometry_known:bool, uncertainty_m:float64, footprint_profile_id:string, policy_version:string, reason:string, evidence:EvidenceRef[]. 측정 실패 값은 *_known=false이며 판정에 사용 금지. | 1차 |
| RouteCandidate | route_id:UUID, route_revision:uint64, robot_id:string, target_id:string, segment_index:uint32, frame_ref:FrameRef, planned_path:nav_msgs/Path, approach_goal:geometry_msgs/PoseStamped, generated_at_ns:int64, valid_until_ns:int64, path_sha256:string, approval_corridor_sha256:string, assessment:PassageAssessment, planner_evidence_id:string, policy_version:string. 대상 좌표·접근 목표는 다른 값. | 1차 |
| OperatorApproval | approval_id:UUID, command_id:UUID, operator_id:string(서버 인증값), robot_id:string, robot_boot_id:UUID, authority_epoch:UUID, target_id:string, target_estimate_revision:uint64, route_id:UUID, route_revision:uint64, segment_index:uint32, approved_at_ns:int64, valid_until_ns:int64, scope:string(ONE_SEGMENT). 같은 구간의 같은 의미만 멱등 허용. | 1차 |
| RescueReport | envelope:Envelope, report_id:UUID, target_id:string, task_id:string, generation:uint64, record_ids:UUID[], locations:TargetLocation[], site_events:SiteEvent[], rechecks:RecheckObservation[], routes:RouteCandidate[], approvals:OperatorApproval[], task_admissions:TaskAdmission[], has_task_result:bool, task_result:TaskResult, confirmation_state:string, approach_state:string, rescue_state:string, evidence_refs:EvidenceRef[], first_issued_at_ns:int64, meaning_sha256:string. 해당 기록이 없는 optional section은 명시적으로 빈 배열/has flag. | 1차 |
| ReportReceipt | envelope:Envelope, receipt_id:UUID, report_id:UUID, meaning_sha256:string, included_record_ids:UUID[], state:string(RECEIVED/PENDING_EVIDENCE/PERSISTED/REJECTED), persisted_at_ns:int64, has_persisted_at:bool, storage_revision:uint64, reason_code:string. included_record_ids는 저장한 보고의 내용과 일치. | 1차 |
| TaskResult | task_id:UUID, command_id:UUID, robot_id:string, generation:uint64, outcome:string(COMPLETED/CANCELED/EXPIRED/UNREACHABLE/FAILED), completion_scope:string(SEARCH/SEGMENT/TARGET_APPROACH/RECHECK/RETURN/HOLD), terminal_reason:string, original_cause:string, stop_confirmed:bool, motion_state:string, final_pose:RobotPose, used_record_ids:UUID[], frame_ref:FrameRef, recheck:RecheckObservation, has_recheck:bool, stop_evidence:EvidenceRef[]. 구간 성공을 접근/구조 성공으로 바꾸지 않음. | 1차 |

## 서비스 Request / Response 계약

### SubmitOperatorCommand

IDL: 1차 ([contracts C-10](contracts.md)).

| 구분 | 정의 |
| --- | --- |
| Request | envelope:Envelope, command_id:UUID, operation:string(DISPATCH_SUPPORT/CONTINUE_SEGMENT/REQUEST_RECHECK/RETURN_HOME/FINISH_MISSION/RESET_STOP/SET_CAMERA_WATCH), operator_id:string(operations 인증값), robot_id:string, target_id:string, expected_task_id:string, expected_generation:uint64, expected_target_revision:uint64, route_id:string, route_revision:uint64, segment_index:uint32, issued_at_ns:int64, deadline_ns:int64, roi_xyxy_normalized:float64[4], association_evidence_id:string. operation별 필수 필드는 검증; 나머지는 명시적 비적용. |
| Response | envelope:Envelope, accepted:bool, command_id:UUID, request_state:string(ACCEPTED/REJECTED/DUPLICATE), reason_code:string, created_task_id:string, approval_id:string. accepted는 접수이며 이동·구조·저장 완료가 아님. |
| Rules | 웹 operator_id 문자열을 신뢰하지 않고 서버 세션에서 설정. CONTINUE_SEGMENT는 현재 STOPPED, 이전 구간 종료, 해당 route/target revision, ONE_SEGMENT 승인 및 유효시간이 모두 맞을 때만 접수. 장부 재전송은 같은 command_id/의미 payload. |

### RequestStop

IDL: 1차 ([contracts C-10](contracts.md)).

| 구분 | 정의 |
| --- | --- |
| Request | envelope:Envelope, stop_command_id:UUID, requester_id:string, authority_epoch:string, robot_id:string, scope:string(ACTIVE_TASK/ALL_MOTION), reason_code:string, issued_at_ns:int64, deadline_ns:int64. |
| Response | envelope:Envelope, accepted:bool, stop_command_id:UUID, reason_code:string, state:string(ACCEPTED/REJECTED/DUPLICATE). |
| Rules | 일반 임무보다 우선 접수. callback에서 실제 정지를 기다리지 않음. 후속 RobotStatus STOPPED/STOP_UNCONFIRMED 및 TaskResult로 판정. 권한 있는 정지 요청은 활성 task ID를 몰라도 가능. |

### RegisterAuthority

IDL: 1차 ([contracts C-10](contracts.md)).

| 구분 | 정의 |
| --- | --- |
| Request | envelope:Envelope, command_id:UUID, robot_id:string, expected_robot_boot_id:UUID, coordinator_id:string, authority_epoch:UUID, policy_version:string, issued_at_ns:int64, deadline_ns:int64. |
| Response | envelope:Envelope, accepted:bool, robot_boot_id:UUID, authority_epoch:UUID, reason_code:string. |
| Rules | 기존 이동/제어권 종료와 실제 정지 확인 후 등록. authority_epoch UUID의 크기로 신구 판단하지 않음. 등록은 ARMED 절차의 일부이고 lease/이동 허가를 자동 생성하지 않음. |

### ResetStop

IDL: 1차 ([contracts C-10](contracts.md)).

| 구분 | 정의 |
| --- | --- |
| Request | envelope:Envelope, reset_command_id:UUID, robot_id:string, expected_robot_boot_id:UUID, authority_epoch:UUID, expected_latch_id:UUID, resolved_reason_code:string, issued_at_ns:int64, deadline_ns:int64. |
| Response | envelope:Envelope, accepted:bool, latch_released:bool, reason_code:string. |
| Rules | 현장 E-stop/fault 해제를 대신하지 않음. 정지·원인 해결·live readiness·권한 확인 후 software latch만 해제. 이전 task 자동 재개 없음; 새 generation 필요. |

### SetCameraWatch

IDL: 2차 ([contracts C-10](contracts.md)).

| 구분 | 정의 |
| --- | --- |
| Request | envelope:Envelope, command_id:UUID, camera_id:string, watch_id:UUID, target_id:UUID, expected_watch_revision:uint64, roi_xyxy_normalized:float64[4], association_method:string(CALIBRATED_PROJECTION/OPERATOR_ROI), association_evidence_id:string, enable:bool, issued_at_ns:int64, deadline_ns:int64. |
| Response | envelope:Envelope, accepted:bool, watch_id:UUID, watch_revision:uint64, reason_code:string. |
| Rules | ROI [xmin,ymin,xmax,ymax]는 0≤min<max≤1. 카메라별 활성 watch 1개로 시작하며 교체/해제는 revision을 확인. image_size/camera_pose 변경 시 watch 무효화. target_id 없이 소실 이벤트 발행하지 않음. 카메라를 robot_id 목록에 넣지 않음. |

## 액션 Goal / Result / Feedback 계약

### PrepareMapping

IDL: 2차 ([contracts C-10](contracts.md)).

| 구분 | 정의 |
| --- | --- |
| Goal | envelope:Envelope, request_id:UUID, robot_id:string, operation:string(PREPARE/START), baseline_map_ref:string, mapping_profile_id:string, expected_map_session_id:string, issued_at_ns:int64, deadline_ns:int64. |
| Result | envelope:Envelope, outcome:string(READY/FAILED/UNKNOWN), map_state:MapState, reason_code:string. |
| Feedback | envelope:Envelope, request_id:UUID, phase:string(CHECKING_PROFILE/STARTING_BACKEND/WAITING_FRESH_MAP_TF), reason_code:string. |
| Rules | mapping 소유 adapter가 bringup의 지정 backend 기동/activation 절차를 요청하고 fresh map/scan/TF로 준비 결과를 확인. 단순 process 시작을 READY로 표시하지 않음. 실제 slam_toolbox lifecycle/process API는 설치 버전 확인 후 binding을 고정하며 임의의 표준 service 이름을 가정하지 않음. 활성 이동 중 session reset/기준 지도 교체 금지. PREPARE는 사전 점검, START는 지진 이후 SLAM 시작 요청. Nav2 이동은 별도 executor guard. |

### ExecuteTask

IDL: 1차 ([contracts C-10](contracts.md)).

| 구분 | 정의 |
| --- | --- |
| Goal | envelope:Envelope, command_id:UUID, task_id:UUID, robot_id:string, expected_robot_boot_id:UUID, coordinator_id:string, authority_epoch:UUID, generation:uint64, kind:string(SEARCH_AREA/APPROACH_TARGET/REOBSERVE_TARGET/HOLD/MOVE_SEGMENT/RETURN_HOME), issued_at_ns:int64, deadline_ns:int64, lease_id:UUID, target_id:string, target_estimate_revision:uint64, search_area_ref:string, home_ref:string, frame_ref:FrameRef, route:RouteCandidate, has_route:bool, approval:OperatorApproval, has_approval:bool, policy_version:string. |
| Result | envelope:Envelope, result:TaskResult. |
| Feedback | envelope:Envelope, task_id:UUID, generation:uint64, state:string(VALIDATING/PLANNING/NAVIGATING/VERIFYING/STOPPING), segment_index:uint32, current_pose:RobotPose, distance_remaining_known:bool, distance_remaining_m:float64, reason_code:string. UNKNOWN pose/distance는 status/known=false로 명시하고 숫자 사용 금지. |
| Rules | 로봇당 활성 이동 task 1개. MOVE_SEGMENT는 APPROACH와 별도 새 kind: 한 구간 도착·실제 정지 뒤 Result(COMPLETED, scope=SEGMENT). coordinator는 다음 구간을 자동 발급하지 않음. 최종 구간 뒤 REOBSERVE_TARGET로 AMR2 카메라 재확인. RETURN_HOME은 현재 유효 home_ref를 재해석하며 초기 pose를 stale goal로 재사용하지 않음. 수락 전 거절은 action Result가 아니라 accepted=false + TaskAdmission. 이동 출력이 있었던 모든 종료 경로는 stop 관문. |

### EvaluateRoute

IDL: 2차 ([contracts C-10](contracts.md)).

| 구분 | 정의 |
| --- | --- |
| Goal | envelope:Envelope, request_id:UUID, robot_id:string, target_id:string, target_estimate_revision:uint64, purpose:string(APPROACH/SEGMENT/RECHECK/RETURN), segment_index:uint32, candidate_goal:geometry_msgs/PoseStamped, has_candidate_goal:bool, home_ref:string, frame_ref:FrameRef, footprint_profile_id:string, policy_version:string, deadline_ns:int64. |
| Result | envelope:Envelope, outcome:string(VALID/BLOCKED/UNKNOWN/FAILED), has_route:bool, route:RouteCandidate, reason_code:string. |
| Feedback | envelope:Envelope, request_id:UUID, phase:string(PLANNING/EVALUATING_PASSAGE/VALIDATING), reason_code:string. |
| Rules | executor가 ComputePathToPose를 호출하고 AMR2는 필요한 구간에서 EvaluatePassage를 호출. 주행 없음. 승인은 평가된 route corridor/segment/revision에 결합. 재계획이 승인 범위를 벗어나거나 의미 있는 지도/위치/평가 변화이면 승인 무효화·정지·재평가. 계산 실패를 경로 가능으로 대체하지 않음. |

### ObserveTarget

IDL: 1차 ([contracts C-10](contracts.md)).

| 구분 | 정의 |
| --- | --- |
| Goal | envelope:Envelope, request_id:UUID, task_id:UUID, target_id:UUID, robot_id:string, watched_native_point:geometry_msgs/Point, frame_ref:FrameRef, requested_window_ns:uint64, policy_version:string, deadline_ns:int64. |
| Result | envelope:Envelope, outcome:string(OBSERVED/UNKNOWN/FAILED), observation:RecheckObservation, reason_code:string. |
| Feedback | envelope:Envelope, request_id:UUID, phase:string(COLLECTING/CHECKING_COVERAGE/FINALIZING), camera_state:string, lidar_state:string. |
| Rules | perception은 센서 취득·판정만 수행. target point의 실제 시야/거리/검출기/TF 유효성이 충족돼야 ABSENT 또는 CLEAR를 발행. 고정 카메라 소실 이후 완료 판정용 AMR1 관측 창이 새로 확보돼야 함. 취소/timeout은 UNKNOWN이며 부재 성공으로 만들지 않음. |

### EvaluatePassage

IDL: 2차 ([contracts C-10](contracts.md)).

| 구분 | 정의 |
| --- | --- |
| Goal | envelope:Envelope, request_id:UUID, route_id:UUID, route_revision:uint64, segment_index:uint32, path:nav_msgs/Path, frame_ref:FrameRef, footprint_profile_id:string, policy_version:string, deadline_ns:int64. |
| Result | envelope:Envelope, assessment:PassageAssessment. |
| Feedback | envelope:Envelope, request_id:UUID, phase:string(COLLECTING_DEPTH/PROJECTING/CHECKING_CLEARANCE), valid_samples:uint32, reason_code:string. |
| Rules | footprint_profile은 로봇 box의 폭·높이·길이·하중 포함 envelope와 불확실성/안전 여유를 정의. 경로 swept volume 전체의 바닥/천장·폭을 평가. 상부 3D 여유가 알려지지 않으면 UNKNOWN. 2D LiDAR만으로 기울어진 기둥 하부 통과 승인 금지. 카메라 box overlay는 검토용 증거이며 PASSABLE 계산을 대신하지 않음. |

## 표준 Nav2 액션

| 공식 타입 | 의미 |
| --- | --- |
| nav2_msgs/action/NavigateToPose | Goal은 stamped 목표 pose와 BT 선택, Feedback은 현재 pose·진행 시간/거리, Result는 오류 코드/메시지. 성공과 실제 정지·대상 재확인은 분리. |
| nav2_msgs/action/ComputePathToPose | Goal은 목표·시작 pose와 planner 선택/시작 사용 여부, Result는 path·계산 시간·오류. Jazzy feedback에는 사용자 필드 없음. 경로 조회는 로봇 이동 아님. |
| nav2_msgs/action/FollowPath | Goal은 경로 및 controller/checker 선택, Feedback은 추종 거리/속도, Result는 오류. Nav2 내부 bt→controller 연결. |

## QoS·저장·실패 계약

- 센서 큐의 depth 1, 이벤트 depth 10, 상태 depth 1은 **[설정 제안, 2026-10-09]**이다. 실제 publisher/subscriber QoS를 맞추고 동시 부하에서 확정한다. 센서 최신성, snapshot late-join, event reliable은 [ROS 2 Jazzy QoS](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)를 기준으로 선택했다.
- TRANSIENT_LOCAL 상태는 UI 복원용이며 새 boot의 readiness·명령을 승인하는 live 근거가 아니다. 명령/lease/service는 VOLATILE이며 응용 계층에서 ID·권한·boot·generation·deadline을 검사한다.
- 중요 발견(TargetLocation)·SiteEvent·TaskAdmission은 원 생산자의 보류 outbox에 남긴다. coordinator는 record ID를 중복 제거해 불변 RescueReport를 만들고 operations의 PERSISTED receipt의 included_record_ids/meaning hash로 원 outbox를 정리한다. 중요 task 종료/recheck도 coordinator가 report로 저장한다. outbox/발급 장부 복구는 구현 작업이며 프로세스 재시작만으로 과거 goal을 재개하지 않는다. outbox 용량/재전송 횟수/overflow 정책은 회차 profile에서 정하며 초과는 관제에 드러낸다.
- DB write는 operations 하나가 소유한다. PERSISTED는 필요한 payload와 참조된 evidence bytes/hash가 모두 영속 저장된 뒤의 의미다. DDS reliable 수신이나 RECEIVED는 이 상태를 대신하지 않는다.
- request_stop 응답은 접수. cancel 요청, Nav2 terminal result, command fence, fresh 실제 정지 관측을 따로 기록한다. odom/driver 단절·정지 확인 timeout은 STOP_UNCONFIRMED와 새 이동 차단이다.
- 원 관측의 정수 ns·source boot·frame/ref·해시를 보존한다. 시각·좌표·높이를 모르면 UNKNOWN/INVALID로 남긴다. header를 현재 수신 시각으로 다시 찍거나 최신 TF로 조용히 대체하지 않는다.
- permission/HTTP 인증은 operations, 단일 임무 authority/generation은 coordinator/executor 경계가 책임진다. REST reset 또는 heartbeat 재수신만으로 움직이지 않는다. 현장 정지 수단은 웹 연결과 독립적이어야 한다.

## 기존 설계·코드에 필요한 확장

| 항목 | 기존 checkout | 이 계약 v0.1에서 제안한 변경 | 관련 책임 |
| --- | --- | --- | --- |
| 배치 | 로봇 PC 2대 + coordination/operations PC 2대의 후보 | PC3 webcam, PC4 coordination/operations/web/DB/discovery를 함께 배치 | bringup/프로파일·전체 SDD |
| camera 이벤트 | SiteEvent kind=DISASTER_SIGN/EXIT_PASS | 웹캠 1대: TARGET_DISAPPEARED_CANDIDATE 추가, target/watch/visibility 필드 추가; EXIT_PASS를 이번 완료 조건에 쓰지 않음 | rescue_contracts/interfaces/perception/coordination |
| 임무 종류 | SEARCH_AREA/APPROACH_TARGET/REOBSERVE_TARGET/HOLD | MOVE_SEGMENT·RETURN_HOME 추가 및 TaskResult completion_scope | contracts/robot/coordination/profile |
| 승인 | 일반 임무 요청·authority 개념 | route/segment/revision/한 구간에 묶인 OperatorApproval와 operator service | operations/coordination/robot/console |
| 완료 | 출구 카운트 + AMR1 재관측 계약 | target에 연결된 fixed-camera 소실 후보 + fresh AMR1 재관측의 시연 조건 | scope/state-machines/coordination/perception |
| 보고 | ReportReceipt에 report ID/저장 revision | included_record_ids/meaning hash와 원 생산자 outbox ACK 연결 | operations/coordination/perception/mapping/robot |
| ROS 연결 | 부분 Nav2 입력 adapter, dispatch guard 차단 | 사용자 정의 ROS IDL·서버·pub/sub·mapping authority·장치 watchdog 연동 | 통합/기반/인식/관제 각 레인 |

이 검토안을 실행 계약으로 채택할 때는 공통 계약 담당의 PR에서 scope·contracts·state-machines·profile/schema 및 소비 패키지 영향을 함께 선언하고 검증해야 한다. 이번 PR은 명세와 도면의 문서 등록이며, 위 실행 계약·구현은 변경하지 않는다.

## 수치를 정해야 하는 항목

| 값 | 정하는 곳 | 미정일 때 동작 |
| --- | --- | --- |
| earthquake 엣지 지표·연속 frame·해제 조건 | camera 판정 profile | 이벤트/로그 관찰만; 모드·출발 허가 없음 |
| target/ROI 연결·가림 판정·소실 유지 기간 | camera watch profile·ROI 연결 증거 | 소실은 UNKNOWN, 완료 후보 생성 보류 |
| low speed·구간 길이·정지점·구간 대기·deadline | route/robot/mission profile | 구간 이동 발급 보류 |
| footprint/kit 포함 높이·폭·길이·바닥/천장 오차·안전 여유 | footprint/passage profile | passage UNKNOWN, 해당 통로 진입 보류 |
| sensor/TF/pose/lease 최대 나이·clock 오차·정지 속도/관측 창 | readiness/stop/time profile | NOT_READY/STOP_UNCONFIRMED; 자동 이동·완료 보류 |
| mission alignment·loop-closure 처리·home/return 정책 | mapping/mission profile | 교차 로봇 접근·자동 복귀 보류 |
| 카메라 overlay 구현·RGB-D 센서와 캘리브레이션 | hardware/vision profile | overlay는 승인 근거로 사용하지 않음 |

## draw.io 페이지와 연결 표기

[draw.io 원본](rokey7-sdd.drawio)은 다음 상세 페이지로 구성한다. 같은 노드가 여러 페이지나 행에 반복되어도 동일한 논리 노드다. 인터페이스 포트 박스는 통신 정의를 표시하며 별도 실행 노드가 아니다.

| 페이지 | 내용 |
| --- | --- |
| 01 | 하드웨어·노드 전체도 |
| 02 / 03 | PC1 센서·지도 / 명령·주행 |
| 04 / 05 | PC2 센서·지도 / 명령·주행 |
| 06 | PC3 CAM·지진·소실 |
| 07 | PC4 좌표·상태·수신확인 |
| 08 | PC4 웹·DB·HTTP |
| 09 | DDS 발견·데이터 경계 |
| 10 | `rescue_interfaces` 통신 흐름: [패키지 README](../../src/rescue_interfaces/README.md) 「통신별 입력과 출력」 표의 그림(2026-10-10 추가) |

포트 박스의 `rescue_interfaces` 타입 뒤 `[1차]`·`[2차]`는 위 표의 `IDL` 열과 같다(2026-10-10 추가). `[2차]` 박스와 10쪽의 `[2차]` 선은 점선이다. 이 추가로 draw.io 해시가 2026-10-09 [정적 검사 기록](../evidence/sdd-node-interface-validation.json)의 값과 달라졌다 — 그 기록은 그날 파일의 것이고, 01~09 쪽의 연결(셀 id·source·target)은 바꾸지 않았다. 재검사는 미실행.

모든 상세 연결은 `IF-001`~`IF-086`과 endpoint/type을 표시한다. `[T]`는 PUB→SUB, `[S]`·`[A]`는 client→server 요청이며 response/feedback/result는 역방향으로 돌아온다. HTTP snapshot/SSE/video의 선은 **응답 데이터 흐름**이며 브라우저의 GET 요청 방향과 반대다. 내부 costmap/DB 호출과 DDS discovery는 ROS 토픽·서비스·액션과 분리한다. Discovery 선은 참여자↔서버의 발견 정보만 뜻하고 application data 중계가 아니다. 논리 ROS 노드 수는 DDS participant 프로세스 수의 실측이 아니다.

Nav2 내부 action은 [NavigateToPose Jazzy](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_msgs/action/NavigateToPose.action), [ComputePathToPose Jazzy](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_msgs/action/ComputePathToPose.action), [FollowPath Jazzy](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_msgs/action/FollowPath.action)를 사용한다. endpoint는 native namespace에 remap한 제안이며 설치된 action 이름을 조회해 확인한다. Context7의 rolling/main 결과에 있는 새 필드를 Jazzy 계약에 추가하지 않았다. Collision Monitor 연결은 [Jazzy 공식 지침](https://docs.nav2.org/jazzy/tutorials/general_tutorials/using_collision_monitor/using_collision_monitor/)을 따르고, 별도 장치 watchdog은 프로젝트의 미설치 설계다.

사용하는 BT에 recovery behavior가 있으면 behavior_server와 해당 action을 별도 추가해야 한다. 이 초안은 무조건적인 회전/후진 recovery를 승인하지 않으며 최소 경로 실패→정지/보고 구성을 전제로 한다. lifecycle·파라미터·diagnostics 서비스는 bringup/운용 상세도에서 설치 버전에 맞춰 별도 표시한다.

## 정적 검증과 미실행

노드 ID·endpoint/type 일치, 참조 노드 존재, 요청 server 단일성, Nav2 주행 caller=executor, DB writer=operations, PC 4개/AMR 2개/웹캠 1개의 논리 배치를 assert로 검증한다. 표의 이름/타입/edge는 JSON에서 생성하므로 CSV와 같은 원본이다.

등록 산출물의 노드 참조, ID·endpoint/type, CSV와 draw.io 발행·수신 연결의 일치, XML 참조 무결성과 파일 해시는 [정적 검사 기록](../evidence/sdd-node-interface-validation.json)에 남긴다. 2026-10-09 정적 집계는 논리 노드 44개, 배포 인터페이스 그룹 86개, source×target 관계 255개다. 그룹에는 로봇별 중복 배치 endpoint가 포함되므로 86개를 서로 다른 메시지 타입 수로 해석하지 않는다.

같은 draw.io 바이트는 앞선 작업(2026-10-09)에서 실제 draw.io 화면의 9페이지·86개 interface ID를 확인했다. 이번 문서 등록에서는 도면을 수정하지 않고 원본과 SHA-256 일치를 확인한다. 이 GUI 기록은 ROS 연결이나 실제 센서의 통신 증거가 아니다.

ROS IDL 생성·컴파일, 실제 ROS 그래프·QoS 통신, 카메라/깊이/TF·정지·주행·다중 PC 장애 시험은 **NOT_RUN**. 현 파일의 계약은 검토 가능한 제안이다.

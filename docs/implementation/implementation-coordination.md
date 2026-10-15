---
title: 임무 조정기 구현 기록 — 조정기 코드 기준
status: current   # 본문은 main d217680 의 rescue_coordination 코드 기준. 아래 「기록」 절은 2026-10-13 판 전문
updated: 2026-10-14
---

# 임무 조정기 구현 기록

## 한눈에 (2026-10-14, main `d217680`)

이 문서가 답하는 질문은 하나다. **조정기 코드는 어느 파일이 무엇을 맡고, 현장에서 어떤 값으로 돌며, 무엇으로 시험했나?**

| 파일(`src/rescue_coordination/rescue_coordination/`) | 맡은 일 | 현장 노드가 쓰나 |
| --- | --- | --- |
| `ros_node.py` | 판단 코어 `CoordinatorCore` 와 ROS 노드 `mission_coordinator`(`main()`). 판단은 코어가 하고 `main()` 은 메시지만 옮긴다 | 씀 |
| `mission.py` | 임무 한 건의 상태 `MissionCoordinator`. 재난 모드, 운영자 명령 관문, 임무 단계(`phase()`) | 씀 |
| `search.py` | 역할별 예약 `SearchCoordinator`. TB-6 수색 자리(`SEARCH_AREA`) 예약과 실행기 수락 기록 | 씀 |
| `planning.py` | 예약 장부 `Allocator`·`Reservation`, 발급 세대(generation), 공통 오류 `CoordinationError` | 장부, 오류, 값 검사 함수만 씀 |
| `live_task_inputs.py` | `~/task_inputs`(로봇이 읽는 목적지), RViz 수색 목표, `~/refresh_state` | 씀(RViz 목표는 `AREAS` 에서만) |
| `watch.py` | 웹캠 감시 지정(`SetCameraWatch`) 장부 | 씀 |
| `verdict.py` | TB-6 재확인 관측으로 구출 판정 | `completion_mode=RECHECK` 에서만 |
| `dispatch.py` | 확정 대상이 여럿일 때 TB-5 가 갈 대상 고르기 | 노드는 쓰지 않음(`search.py` 의 `select_approach_target` 만 감쌈) |
| `planning.py` 의 `select_candidate`·`ApproachTracker`·`Corridor` | 접근 후보, 접근 추적, 통로 점유 | 노드는 쓰지 않음 |

- 현장 설정은 지도 고정(`map_mode=SLAM_FREEZE`), 지도 전체 탐색(`search_mode=FRONTIER`), 출구 도착으로 구출 완료(`completion_mode=EXIT_ARRIVAL`), 자동 탐색 시작(`search_start=AUTO`), 자동 출동(`support_dispatch=AUTO`), 교통 관제(`traffic_control=TB5_PRIORITY`)다. 값은 [현장 파일](../../config/field/search_area.field.json) `coordinator` 절에 있다.
- 임무 단계와 이유 코드는 [상태 전이](../design/state-machines.md)에, 토픽 계약은 [인터페이스 계약](../design/contracts.md)에, 기능별 구현 상태는 [rescue_coordination README](../../src/rescue_coordination/README.md)에 있다. 로봇과 PC 이름은 [용어집](../glossary.md)을 따른다.
- 모든 기능은 로봇 없는 시험(단위 시험, E3 리허설)만 통과했다. 실기 시험은 하지 않았다(NOT_RUN).

## 한 번의 판단 주기

조정기는 `tick_period_s`(현장 0.1 s)마다 `node_tick` 을 한 번 돈다. 보내는 순서는 아래와 같다(`ros_node.py` `node_tick`).

1. 바뀐 대상 목록(`/rescue/targets`). 로봇은 임무에 적힌 대상 revision 을 이 목록에서 찾으므로 임무보다 먼저 보낸다.
2. 코어의 `tick()` 이 이번에 낼 임무를 정한다. 순서는 지도 고정 → TB-5 접근 → TB-5 출구 → TB-6 복귀 → TB-5 구간 이동 → TB-6 재확인 → 순찰 → 미탐색 경계(frontier) → TB-6 수색이다.
3. 교통 관제가 멈추게 한 TB-6 임무의 취소(`ExecuteTask` goal 취소). 새 임무보다 먼저 보낸다.
4. 새 임무(`ExecuteTask` goal), 지도 고정 호출, 노드 로그, 웹캠 감시 요청, 보고(`RescueReport`), 명령 임대 갱신(`CommandLease`).
5. 대상 목록을 한 번 더 보내고(임무로 바뀐 접근 상태), 바뀌었으면 `SystemState` 를 보낸다.

이번 주기에 임무를 막은 이유(`issue_block`)는 주기마다 다시 계산한다. 막은 조건이 풀리면 이유도 지운다.

## ROS 연결

| 이름 | 종류 | 방향 | 조정기가 하는 일 |
| --- | --- | --- | --- |
| `/rescue/site_events` | `SiteEvent` | 받음 | 웹캠 재난 신호(`DISASTER_SIGN`)로 재난 모드를 켠다. 시각 기준이 선언된 신호만 받는다 |
| `/rescue/robots/<id>/status` | `RobotStatus` | 받음 | `READY` 이고 `status_max_age_s` 안이어야 새 임무를 낸다 |
| `/rescue/robots/<id>/pose` | `RobotPose` | 받음 | TB-5 위치는 접근 방향과 교통 관제에, TB-6 위치는 frontier 출발점과 교통 관제에 쓴다 |
| `/rescue/robots/<id>/map_state` | `MapState` | 받음 | 로봇이 지금 어느 지도에서 위치를 잡았는지 기록한다. TB-5 는 고정 지도 revision 이어야 출동한다 |
| `/rescue/robots/<id>/execute_task` | `ExecuteTask` 액션 | 보냄 | 임무를 보낸다. 교통 관제는 TB-6 임무를 여기서 취소한다 |
| `/rescue/robots/<id>/task_admission` | `TaskAdmission` | 받음 | 실행기가 수락해야 임무가 진행 중(ACTIVE)이 된다 |
| `/rescue/robots/<id>/command_lease` | `CommandLease` | 보냄 | 임무마다 `lease_renew_period_s` 로 갱신한다. 갱신을 멈추면 실행기가 스스로 선다 |
| `/rescue/target_locations` | `TargetLocation` | 받음 | 인형 위치. `merge_distance_m` 안이면 같은 대상으로 합친다 |
| `/rescue/exit_location` | `TargetLocation` | 받음 | QR 코드(`class_label=exit_qr`) 출구. QR id 마다 최신 위치를 기록한다 |
| `/rescue/reports` · `/rescue/report_receipts` | `RescueReport` · `ReportReceipt` | 보냄 · 받음 | 대상 revision 마다 보고를 보내고 저장 확인(`PERSISTED`)을 받는다 |
| `/rescue/targets` | `TargetSnapshot` | 보냄 | 바뀔 때만 보낸다. 늦게 붙은 구독자도 마지막 값을 받는다(QoS 설정 TRANSIENT_LOCAL) |
| `/rescue/system_state` | `SystemState` | 보냄 | 임무 모드·단계·이유. 바뀌면 보내고, 그대로면 `lease_renew_period_s` 마다 다시 보낸다(늦게 뜬 실행기용, 로그 `SYSTEM_STATE_RESENT`) |
| `/rescue/operator_command` | `SubmitOperatorCommand` 서비스 | 받음 | 운영자 명령 관문 |
| `/rescue/cameras/<camera>/set_watch` | `SetCameraWatch` 서비스 | 부름 | 운영자 `SET_CAMERA_WATCH` 마다 웹캠 감시를 건다 |
| `/rescue/map/freeze` | `std_srvs/Trigger` 서비스 | 부름 | `SLAM_FREEZE` 에서 지도 고정을 요청한다 |
| `/rescue/map/frozen` · `/rescue/map/frozen_revision` | `OccupancyGrid` · `String`(JSON) | 받음 | 고정 지도와 그 revision |
| `/rescue/map/start_pose/<TB-6 id>` | `PoseWithCovarianceStamped` | 받음 | TB-6 출발 자세. 도킹 접근점과 TB-5 출발 자리의 기준이다 |
| `/robot6/map` | `OccupancyGrid` | 받음 | `FRONTIER` 에서 TB-6 실시간 SLAM(동시 위치 추정과 지도 작성) 지도 [추정: 토픽 이름 실기 미확인] |
| `~/task_inputs` | `GetParameters` 서비스 | 제공 | 로봇이 읽는 목적지 목록(수색 자리, `home/<id>`) |
| `~/refresh_state` | `std_srvs/Trigger` 서비스 | 제공 | 마지막 `SystemState` 를 다시 보낸다 |

## 현장 흐름 (코드 기준)

### 탐색 (`search_mode=FRONTIER`)

- 재난 신호를 받으면 TB-6 탐색이 자동으로 시작된다(`search_start=AUTO`, 로그 `SEARCH_AUTO_STARTED`). 운영자 `START_SEARCH` 는 `NOT_AUTHORIZED` 로 거절한다.
- TB-6 실시간 지도와 TB-6 위치로 미탐색 경계 덩어리를 찾는다(`rescue_contracts.grid_search.find_frontiers`). 덩어리는 `frontier_min_cells` 칸 이상이고, 목표 칸은 덩어리에서 `frontier_reach_m` 안이다.
- 이미 시도한 곳에서 `frontier_blacklist_m` 안은 고르지 않는다. 남은 것은 실패 횟수가 적은 것이 먼저고, 그다음 explore_lite 비용이 낮은 것이 먼저다. 비용은 (`frontier_potential_scale` × TB-6 에서의 거리(m) − `frontier_gain_scale` × 경계 칸 수) × 지도 해상도로, explore_lite 원본식 그대로다(`src/vendor/m-explore-ros2/explore/src/frontier_search.cpp` `frontierCost`, 192~197행). 원본은 이미 m 인 거리에 해상도를 한 번 더 곱한다. 그래서 크기가 거의 정하고, 거리는 크기가 비슷할 때 가른다. 현장 값 3.0 / 1.0 에서는 거리 1 m 가 경계 3칸과 같다. 원본 거리는 가장 가까운 경계 칸까지의 직선거리이고, 여기서는 경계 목표 칸까지의 격자 경로 길이를 쓴다. 5회차(2026-10-16) 시뮬레이션에서 TB-6 이 라이다로 이미 본 쪽의 작은 틈으로 먼저 돈 일 때문이다. 고른 경계의 경로 위 `frontier_goal_max_m` 지점을 목표로 삼는다.
- 목표가 그렇게 경계 앞에서 잘렸으면(경로가 `frontier_goal_max_m` 보다 김) 그 목표에 도착(COMPLETED)해도 경계를 시도한 것으로 치지 않는다(로그 `FRONTIER_GOAL_CUT`). 다음 계획에서 같은 비용 규칙으로 다시 견주는데, TB-6 이 그만큼 다가섰으니 보통 같은 경계를 이어 간다. 경계의 목표 칸(경계에서 `frontier_reach_m` 안)까지 가서 COMPLETED 인데도 남아 있을 때만 시도한 것으로 친다. 5회차에서 잘린 목표에 도착한 뒤 경계가 시도함이 되어 메인 구역에 못 들어가고 탐색이 끝난 일(C11) 때문이다. 같은 경계를 따라가는 횟수에 따로 상한은 두지 않는다. 한 번 갈 때마다 경로를 따라 `frontier_goal_max_m` 씩 다가서고, 끝은 `exploration_timeout_s` 가 막는다.
- 경로는 `freeze_clearance_m` 여유로 찾는다. 그래서 0.68 m 폭 틈도 지나갈 수 있는 길로 본다. 목표는 벽에서 `frontier_goal_clearance_m` 떨어진 칸에만 둔다. 현장 값 0.35 는 TB-6 이 목표에서 서고 제자리 회전할 때 RPP·회전 충돌 검사가 막히지 않는 거리다(재범 결정 2026-10-16, C3).
  - 경계의 목표 칸은 경계 중심에서 `frontier_reach_m` 안의 그런 칸 가운데, TB-6 에서 갈 수 있으면서 경계 중심에 가장 가까운 칸이다. 틈 너머 칸도 후보다. 9회차(2026-10-16)에서 경로가 가장 짧은 칸(#420)은 틈을 비스듬히 보는 자리라 너머가 안 보였고, 경계가 시도함이 되어 출발 구역에서 탐색이 끝났다(C14). 중심에 가장 가까운 칸은 틈과 같은 줄이라 너머를 정면으로 본다.
  - 경계의 목표 칸까지 가서 COMPLETED 인데 경계가 최신 지도에 그대로 있으면, 시도함으로 넣는 것은 같고 로그 `FRONTIER_DONE … goal_at=(x, y) frontier_remains=True` 로 남긴다.
  - 경로 위 `frontier_goal_max_m` 지점이 그보다 좁은 곳(틈 안)이면, 경로를 따라 그 너머 첫 넉넉한 칸까지 가서 목표로 삼는다(로그 `FRONTIER_GOAL_PAST_TIGHT`). 그래서 목표는 `frontier_goal_max_m` 보다 틈 길이만큼 멀어질 수 있다.
  - 그런 칸이 하나도 없는 경계는 이번 지도 갱신에서 빼고 다음 경계로 간다(로그 `FRONTIER_GOAL_TIGHT … left out`, `FRONTIER none` 의 TIGHT).
  - 7회차(2026-10-16) 시뮬레이션에서 틈 너머 경계 네 개가 모두 틈 안 같은 자리로 잘려 0.2 m 여유로 보내졌고, 모두 EXPIRED 로 끝난 일(C12) 때문이다. 처음 넣은 #369 는 목표를 0.2 m 로 남겨 두었다.
- 목표를 먼저 `~/task_inputs` 에 넣고 `frontier_issue_delay_s` 뒤에 `SEARCH_AREA` 를 낸다. 로봇이 목적지를 읽을 시간이다.
- 로봇이 출발하지 못한 목표는 시도로 치지 않는다. Nav2 가 목표를 거절했거나(`GOAL_REJECTED`), 제때 받지 않았거나(`ACCEPT_TIMEOUT`), 보내기 전 정지 근거가 오래된(`STOP_EVIDENCE_STALE`) 경우다. 그 곳은 다시 골라 `frontier_issue_delay_s` 뒤에 또 보내고, TB-6 이 새 목표를 받을 때까지 `SCOUT_GOAL_NOT_STARTED` 를 보인다(로그 `FRONTIER_NOT_STARTED`). 2026-10-15 현장에서 TB-6 이 서 있는 채로 `NO_SURVIVOR_FOUND` 로 끝난 일 때문이다.
- 목표를 골랐는데 TB-6 이 READY 가 아니면(예: FAILED 로 끝난 뒤 실행기가 스스로 다시 arm 하지 않음, OPS-231) 목표는 기다리고 `SCOUT_NOT_READY` 를 보인다. 로그 `SCOUT_NOT_READY frontier=… stop_reason=…` 는 목표·정지 사유마다 한 번 남긴다. 기다리는 동안에도 `exploration_timeout_s` 는 흐른다.
- 고를 곳이 없으면 로그 `FRONTIER none …: frontiers=N small= no_goal= tried= tight= no_path=` 에 후보별로 빠진 이유를 센다. 큰 것부터 다섯 개는 이유·칸 수·중심도 적는다. SMALL 은 `frontier_min_cells` 미달, NO_GOAL 은 `frontier_reach_m` 안에 빈 칸 없음, TRIED 는 시도한 곳 `frontier_blacklist_m` 안, TIGHT 는 `frontier_reach_m` 안에 `frontier_goal_clearance_m` 여유 칸이 없음, NO_PATH 는 TB-6 에서 경로 없음이다. TOO_CLOSE 는 서 있는 TB-5 에서 `traffic_min_separation_m` 안이라 빠진 것이다.
- 교통 관제가 켜져 있고 TB-6 이 TB-5 에서 `traffic_min_separation_m` 이상 떨어져 있으면, 경계를 고를 때 TB-5 둘레를 막힌 칸으로 본다. 목표와 그 경로 위 자른 점이 TB-5 에 그만큼 가까워지지 않는다. TB-5 옆 경계를 골라 `TRAFFIC_ROBOTS_TOO_CLOSE` 로 계속 기다리던 일(시뮬레이션 C10, 2026-10-16) 때문이다.
- Nav2 가 받은 뒤 실패한 경계는 `FRONTIER_ATTEMPTS`(2)번 실패할 때까지 다시 고른다(재범 결정 2026-10-16, C9). 한 번도 시도하지 않은 경계가 먼저다. 로그는 `FRONTIER_RETRY … failed 1/2`, 두 번째 실패 뒤 `FRONTIER_GAVE_UP`. 도착(COMPLETED)한 경계는 바로 다시 고르지 않는다. 실패는 TB-6 이 받은 목표 자리로도 센다. 경계가 달라도 같은 자리(`frontier_blacklist_m` 안)에서 `FRONTIER_ATTEMPTS` 번 실패했으면, 그 자리로 가는 경계는 TRIED 로 뺀다(C12).
- 고를 곳이 없는 지도 갱신이 `exploration_empty_checks` 번 이어지면 `EXPLORATION_COMPLETE` 다. 남은 경계가 두 번 실패해 포기한 것뿐이면 `EXPLORATION_FRONTIERS_FAILED`(로그 `failed_frontiers=N`)로 끝내고, 인형이 없어도 `NO_SURVIVOR_FOUND` 대신 이 사유를 보인다. `exploration_timeout_s` 가 지나면 `EXPLORATION_TIMEOUT` 이고, 완료와 똑같이 다음 단계로 간다.

### 지도 고정 두 번 (`map_mode=SLAM_FREEZE`)

- 고정 revision 은 `/rescue/map/frozen_revision`(지도 해시, map session, revision)으로 확인한다. 해시와 빈 칸 규칙은 로봇과 같은 함수(`rescue_contracts.occupancy`)다.
- 1차 고정(R0+1)은 인형 보고가 저장되고(`PERSISTED`) TB-6 이 섰을 때 한 번 요청한다. TB-5 는 이 지도로만 출동한다. TB-6 은 SLAM 지도(R0)로 탐색을 이어 간다.
- 2차 고정(R0+2)은 `EXPLORATION_COMPLETE` 뒤 TB-6 이 섰을 때 요청한다. 대상과 출구를 R0+2 로 옮기고, 보고는 다시 보내지 않는다.
- 같은 map session 이면 고정 때 대상과 출구의 최신 위치를 고정 revision 으로 옮긴다(로그 `TARGET_REBASED_ON_FREEZE`·`EXIT_REBASED_ON_FREEZE`). 다른 session 이면 옮기지 않는다.
- 응답 실패, 잘못된 지도, `map_freeze_timeout_s` 초과는 `MAP_FREEZE_FAILED` 이고 다시 요청하지 않는다.
- 고정 지도에서 목표 자리가 `freeze_clearance_m` 반지름 안에 비어 있지 않으면 보내지 않는다. 다른 자리를 만들지 않고 이유만 낸다(`SUPPORT_APPROACH_NOT_FREE`·`SUPPORT_EXIT_NOT_FREE`·`SCOUT_HOME_NOT_FREE`).

### TB-5 출동과 출구

- 대상이 확정이고 지금 revision 의 보고가 저장되면 TB-5 에 `APPROACH_TARGET` 을 한 번 낸다(`support_dispatch=AUTO`). 운영자 승인은 받지 않는다. 접근점은 인형에서 TB-5 쪽으로 `approach_standoff_m` 떨어진 자리이고, 도착 방향은 인형을 바라본다.
- 접근이 실패하면 다시 시도하지 않고 `APPROACH_FAILED_<사유>` 를 보인다. 출구로도 보내지 않는다.
- 도착 뒤 `support_exit_wait_s` 가 지나고 2차 고정이 끝나면 출구로 간다(`RETURN_HOME`, 목적지 `home/<TB-5 id>`).
- 출구는 최종 지도에서 격자 경로가 가장 짧은 QR 출구의 접근점(출구에서 인형 쪽으로 `exit_standoff_m`)이다(로그 `EXIT_CHOSEN`). QR 출구가 없으면 TB-5 출발 자리(`support_start_pose`)로 간다. 둘 다 없으면 `SUPPORT_START_UNKNOWN`, 출구가 있으나 경로가 없으면 `SUPPORT_EXIT_UNREACHABLE` 이다.
- TB-5 가 출구에 도착하면 구출 판정을 `RESCUE_CONFIRMED`(출처 `EXIT_ARRIVAL`)로 기록한다.

### TB-6 복귀와 임무 끝

- TB-6 은 최종 지도 뒤 도킹 접근점으로 간다(`RETURN_HOME`, 목적지 `home/<TB-6 id>`). 도킹은 사람이 한다. 도킹 접근점은 TB-6 출발 자세 기준 좌표이고 `compose_planar` 로 지도 좌표가 된다. 출발 자세가 없으면 `START_POSE_UNKNOWN` 이다.
- 두 로봇이 모두 도착하면 단계가 `FINISHED`, 모드가 `MISSION_FINISHED` 다.
- 탐색을 끝냈는데 인형이 없으면 고정은 그대로 하고 TB-6 만 복귀한다. TB-5 는 출발 자리에 남고 이유는 `NO_SURVIVOR_FOUND` 다. 다만 TB-6 복귀가 실패하면(결과 COMPLETED 아님) 이유는 `SCOUT_RETURN_FAILED` 로 바뀌고 로그에 `SCOUT_RETURN_FAILED task= outcome= reason=` 이 남는다. 복귀는 다시 보내지 않는다(#194). 11회차(2026-10-16)에 Nav2 TF 오류로 복귀가 실패했는데 `NO_SURVIVOR_FOUND` 에 가려 웹에 안 보였다.

### 교통 관제 (`traffic_control=TB5_PRIORITY`)

TB-5 가 늘 먼저 간다. TB-5 가 사람을 이끌고 사람이 그 뒤를 따라가기 때문이다. 교통 관제는 `SLAM_FREEZE` 에서만 켜고 `FRONTIER` 가 필요하다(비켜서기 자리를 frontier 자리로 보냄). 이유 코드 10개는 [상태 전이](../design/state-machines.md) 「교통 관제 이유 코드」 표에 있다.

- TB-5 새 명령 전에 고정 지도에서 TB-5 격자 경로를 구한다(`grid_path`).
- TB-6 이 서 있는데 그 경로에서 `traffic_path_clearance_m` 안이거나 TB-5 와 `traffic_min_separation_m` 보다 가까우면 먼저 비키게 한다. 비킬 자리는 TB-6 실시간 지도에서 경로와 TB-5 로부터 그만큼 떨어진 빈 칸 가운데 격자 경로가 가장 짧은 곳(`nearest_clear_outside`)이고, `traffic_yield_max_m` 안이어야 한다. 도착 방향은 TB-6 에서 자리 쪽이다(제자리 회전 뒤 전진, 후진 없음).
- TB-6 이 움직이는 중이고 지금 위치나 목표가 그 경로 근처이면 그 임무를 취소한다. 실행기는 취소를 받으면 로봇을 세우고 CANCELED 로 끝낸다. 취소한 frontier 는 시도로 치지 않고, 취소한 TB-6 복귀는 다시 낸다.
- TB-5 가 움직이거나 출발을 기다리는 동안 TB-5 경로 근처로 가는 TB-6 새 목표는 보류한다. 다른 곳으로 가는 목표는 보낸다.
- 두 로봇이 `traffic_min_separation_m` 보다 가깝거나 새 목표가 상대 로봇과 그만큼 가까우면 새 명령을 막는다. 움직이는 임무를 이 이유로 취소하지는 않는다.
- TB-5 는 첫 임무 전에는 출발 자리에 있다. 출발 자리를 아직 재지 않았으면(현장 파일 `robots.robot_b.initial_pose_xy_yaw` 없음) TB-6 을 막지 않고 간격 검사만 끈다. TB-6 을 막으면 1차 고정이 막히고, 그러면 TB-5 출동도 막히기 때문이다. 이 상태에서 TB-6 목표를 보낼 때마다 로그 `TRAFFIC_SEPARATION_UNCHECKED task=… goal=…` 을 남긴다.
- 로그는 `TRAFFIC_YIELD`·`TRAFFIC_YIELD_DONE`·`TRAFFIC_SCOUT_CANCELED`·`FRONTIER_STOPPED_FOR_TB5`·`SCOUT_RETURN_STOPPED_FOR_TB5` 다. 보류가 바뀔 때만 `TRAFFIC_HOLD <역할> <이유>`·`TRAFFIC_RELEASE <역할>` 을 남긴다.

### 현장 설정에서 쓰지 않는 길

- `completion_mode=RECHECK` 이면 웹캠 소실 후보가 TB-6 재확인(`REOBSERVE_TARGET`)을 열고, 재확인 관측을 `verdict.judge` 로 판정한다([계약](../design/contracts.md) C-11). `RESCUE_CONFIRMED` 이면 TB-6 이 스스로 복귀한다. `RECHECK_NEEDED` 이면 운영자 `RETURN_HOME` 을 받아야 TB-6 복귀를 낸다([#233](https://github.com/jaebeom/ROKEY-7/issues/233) 1번, 2026-10-15). 판정이 없으면 그 명령을 거절한다. 응답 `reason_code` 는 `PENDING_EVIDENCE` 이고, 노드 로그의 임무 관문 이유는 `NO_VERDICT` 다. 현장(`EXIT_ARRIVAL`)에서는 소실 후보를 기록만 한다.
- `support_dispatch=OPERATOR` 이면 운영자 `DISPATCH_SUPPORT` 와 `CONTINUE_SEGMENT` 로 TB-5 를 구간(`MOVE_SEGMENT`)마다 승인해 보낸다.
- `search_mode=PATROL` 은 `patrol_points` 를 `patrol_rounds` 바퀴 돈다. `AREAS` 는 프로파일 수색 구역이나 RViz 에서 고른 목표로 간다.
- `map_mode=SAVED_MAP` 은 launch 의 `goal_map_*` 기준 지도 하나로 끝까지 간다.

## 운영자 명령 거절 코드

운영자 서비스(`/rescue/operator_command`)의 `reason_code` 는 [계약](../design/contracts.md) C-09 공통 코드다. 임무 관문의 자세한 이유는 노드 로그에 남긴다. 대응표는 `ros_node.py` 의 `REASON_CODES` 와 `on_command` 다.

| 임무 관문 이유 | `reason_code` |
| --- | --- |
| `MODE_NOT_DISASTER` | `NOT_AUTHORIZED` |
| `AUTO_SEARCH_ENABLED`(`search_start=AUTO` 의 `START_SEARCH`), `AUTO_DISPATCH_ENABLED`(`support_dispatch=AUTO` 의 `DISPATCH_SUPPORT`) | `NOT_AUTHORIZED` |
| `UNKNOWN_TARGET`, `NO_HOME_POSITION` | `NOT_FOUND` |
| `TARGET_NOT_CONFIRMED`, `NO_ROUTE`, `NO_VERDICT` | `PENDING_EVIDENCE` |
| `ALREADY_DISPATCHED`, `NOT_AWAITING_APPROVAL`, `RECHECK_ALREADY_REQUESTED`, `ALREADY_RETURNING`, `SEARCH_ALREADY_STARTED` | `TASK_BUSY` |
| `TARGET_REVISION_STALE`, `ROUTE_REVISION_STALE`, `SEGMENT_INDEX_MISMATCH`, `NOT_DISPATCHED_TARGET`, `TARGET_ALREADY_JUDGED` | `INVALID_INPUT` |
| `HOME_BASIS_MISMATCH` | `FRAME_REVISION_MISMATCH` |
| `SCOUT_NOT_READY`(TB-6 상태를 모르거나 준비 전의 `START_SEARCH`) | `NAV_NOT_READY` |

같은 `command_id` 와 같은 내용이 다시 오면 `DUPLICATE` 로 첫 응답을 돌려준다. 내용이 다르면 `ID_CONTENT_CONFLICT` 다.

## 파라미터

`PARAMETERS` 의 값은 모두 필수이고 기본값이 없다. 하나라도 빠지면 `missing parameters` FATAL 로 exit 2 다. `camera_watch_roi`·`home_poses`·`patrol_points`·`support_start_pose` 는 비워 둘 수 있다(`UNSET_OK_PARAMETERS`). `route_segment_passage_free` 는 넘기지 않아도 된다(`OPTIONAL_PARAMETERS`). 기동 때 git SHA, boot id, 적용한 값, 역할을 로그에 남긴다. 현장 값 칸은 [현장 파일](../../config/field/search_area.field.json) `coordinator` 절 값이고, 「—」는 현장 launch 가 프로파일이나 다른 절에서 넣는 값이다.

| 파라미터 | 뜻 | 현장 값 |
| --- | --- | --- |
| `profile_path`, `entry_yaws` | 배포 프로파일과 수색 구역별 도착 방향 | — |
| `policy_version`, `executor_policy_version`, `mission_frame_id`, `authority_epoch`, `coordinator_id`, `envelope_schema_version` | 정책·goal·메시지 머리말(envelope) 값 | 현장 파일 값 |
| `executable_bases` | 실행해도 되는 자리 근거 종류 | `PRIOR_MAP`, `FRONTIER` |
| `task_duration_s`, `status_max_age_s`, `tick_period_s`, `report_retry_s` | 임무 기한, 로봇 상태 나이 상한, 판단 주기, 보고 재전송 주기 | 120.0, 3.0, 0.1, 0.5 |
| `lease_duration_s`, `lease_renew_period_s`, `lease_message_ttl_s` | 실행기와 같은 임대 길이, 갱신 주기(길이의 절반 이하), 갱신 메시지 수신 만료 | 3.0, 0.5, 4.0 |
| `admission_timeout_s` | 실행기 수락을 기다리는 시간. 넘으면 임대 갱신을 멈추고, 거절한 임무는 이만큼 기다린 뒤 다시 낸다 | 10.0 |
| `route_segment_goals`, `route_revision`, `route_segment_passage_free` | `OPERATOR` 출동의 구간 끝 `[x, y, yaw]`×N, 그 목록의 revision, 구간별 「통로 판정 불필요」 | — |
| `observe_goal` | 재확인 관측 지점 `[x, y, yaw]` | — |
| `goal_map_session_id`, `goal_map_revision`, `goal_transform_revision` | `SAVED_MAP` 목표의 지도 기준 | — |
| `merge_distance_m` | 같은 대상으로 합치는 거리 | 0.5 |
| `target_clock_domain_id`, `target_max_age_s`, `target_max_future_s` | 인형 위치의 시계 도메인, 최대 나이, 앞선 시각 허용 | `rokey7-utc-field`, 3.0, 0.05 |
| `camera_watch_timeout_s`, `camera_watch_roi` | 웹캠 감시 요청 마감, 요청에 관심 영역(ROI)이 없을 때 쓸 값 | 2.0, 없음 |
| `home_poses` | `SAVED_MAP` 의 복귀 위치. TB-6, TB-5 순 `[x, y, yaw]` | — |
| `support_dispatch`, `approach_standoff_m`, `support_pose_max_age_s` | TB-5 출동 방식, 인형 앞 접근 거리, TB-5 위치 최대 나이 | `AUTO`, 0.8, 2.0 |
| `support_exit_wait_s` | TB-5 가 인형 앞에서 출구로 떠나기까지 기다리는 시간 | 15.0 |
| `recheck_max_clock_error_s` | 재확인 판정의 시계 오차 합 상한(C-11 8행) | 2.0 |
| `map_mode`, `freeze_clearance_m`, `map_freeze_timeout_s` | 지도 방식, 고정 지도 빈 칸 반지름(로봇 `goal_clear_radius_m` 과 같아야 함), 고정 대기 상한 | `SLAM_FREEZE`, 0.2, 10.0 |
| `search_start`, `search_mode` | 탐색 시작 방식, 탐색 방식(`PATROL`·`AREAS`·`FRONTIER`) | `AUTO`, `FRONTIER` |
| `patrol_points`, `patrol_rounds` | 순찰 지점(TB-6 출발 자세 기준)과 바퀴 수 | 비움, 1 |
| `frontier_min_cells`, `frontier_reach_m`, `frontier_goal_max_m`, `frontier_blacklist_m` | 경계 덩어리 최소 칸 수, 덩어리에서 목표 칸까지 거리, 목표를 자르는 경로 길이, 다시 고르지 않는 반경 | 10, 1.0, 1.5, 0.5 [추정, 실측 아님] |
| `frontier_goal_clearance_m` | 보내는 탐색 목표점이 벽에서 떨어질 거리. `freeze_clearance_m` 보다 작을 수 없다 | 0.35 (재범 결정 2026-10-16). 로봇 반지름 0.189 + costmap 칸 0.042 + 잡음 한 칸 0.06 + 위치 오차 0.05 [추정 포함] |
| `frontier_potential_scale`, `frontier_gain_scale` | 경계 고르기 비용의 거리 가중치와 경계 크기 가중치. 비용은 (가중치 × 경로 길이 − 가중치 × 칸 수) × 해상도 | 3.0, 1.0 [문서: explore_lite 기본값, 저장소 `explore.yaml`] |
| `frontier_issue_delay_s`, `exploration_empty_checks`, `exploration_timeout_s` | 목적지를 넣고 임무를 내기까지(≥ `live_inputs.poll_period_s`), 빈 지도 갱신 횟수, 탐색 시간 상한 | 2.0, 3, 600.0 [추정] |
| `exit_standoff_m` | TB-5 가 QR 출구 앞에서 서는 거리 | 0.5 [추정] |
| `support_start_pose` | TB-5 출발 자리 `[x, y, yaw]`(TB-6 출발 자세 기준). 현장 launch 가 `robots.<TB-5>.initial_pose_xy_yaw` 에서 넣는다 | 아직 재지 않음 |
| `completion_mode` | 구출 완료 방식(`EXIT_ARRIVAL`·`RECHECK`) | `EXIT_ARRIVAL` |
| `traffic_control` | `TB5_PRIORITY` 또는 `DISABLED`(로봇 없는 시험용). `OFF` 는 ROS 명령줄에서 YAML 참거짓으로 읽혀 쓰지 않는다. 현장 launch 는 `SLAM_FREEZE` 에서 `DISABLED` 를 거절한다 | `TB5_PRIORITY` |
| `traffic_min_separation_m`, `traffic_path_clearance_m`, `traffic_yield_max_m`, `traffic_stop_timeout_s` | 새 명령을 막는 두 로봇 거리, TB-6 이 TB-5 경로와 둘 거리(최소 간격 이상), 비켜서기 최대 경로 길이, 취소한 TB-6 정지 대기 상한 | 0.6, 0.6, 2.0, 3.0 [추정, 실측 아님] |

`live_inputs.*`(시계, 목적지 갱신 주기, RViz 목표 토픽)는 `live_task_inputs.py` 가 읽는다. 설명은 [rescue_coordination README](../../src/rescue_coordination/README.md) 「현장 연동 세부」에 있다.

## 시험

ROS 없는 단위 시험은 `scripts/run_tests.py` 가 모두 모은다. 파일은 2026-10-14 main `d217680` 의 `ls tests/test_coordination*.py` 기준이다.

| 파일 | 확인하는 것 |
| --- | --- |
| `test_coordination.py`, `test_coordination_review.py` | 예약 장부, 실행기 수락, 접근 후보와 통로 점유(노드 밖 코어) |
| `test_coordination_search.py`, `test_coordination_loader.py`, `test_coordination_dispatch.py` | 역할별 예약, 프로파일 수색 구역 읽기, 출동 대상 고르기 |
| `test_coordination_mission.py` | 재난 모드, 운영자 명령 관문, 임무 단계 |
| `test_coordination_ros_core.py`, `test_coordination_ros_targets.py`, `test_coordination_ros_tick.py`, `test_coordination_ros_snapshot.py`, `test_coordination_ros_passage.py`, `test_coordination_ros_refusal.py` | 노드 코어의 메시지 변환, 대상 병합, 보고, 대상 목록, 발급 순서, 실행기 거절 |
| `test_coordination_target_goal.py`, `test_coordination_target_time.py` | 목표 경로 증거 규칙, 인형 위치 시각 검사 |
| `test_coordination_auto_search.py`, `test_coordination_frontier.py` | 자동 탐색 시작, 미탐색 경계 탐색 |
| `test_coordination_map_freeze.py`, `test_coordination_two_freezes.py` | 지도 고정 한 번과 두 번 |
| `test_coordination_approach.py`, `test_coordination_support_exit.py` | TB-5 자동 출동, 인형 앞 대기 뒤 출구 이동 |
| `test_coordination_exit_patrol.py` | 순찰, QR 출구 기록, TB-6 출발 자세 |
| `test_coordination_mission_end.py` | `FRONTIER` 임무 끝. 최종 지도의 출구 선택, TB-6 복귀, 인형 없음 |
| `test_coordination_home.py` | `home_poses` 복귀 위치 |
| `test_coordination_rescue_flow.py` | `RECHECK` 흐름. 웹캠 후보, TB-6 재확인, 판정, 복귀 |
| `test_coordination_traffic.py` | 교통 관제 |
| `test_coordination_watch.py`, `test_coordination_verdict.py` | 웹캠 감시 지정, 재확인 판정 |
| `test_coordination_shutdown.py` | SIGTERM 종료 |

로봇 없는 E3 시험은 ROS 가 필요해 `run_tests.py` 가 모으지 않는다. `ROS_DOMAIN_ID` 는 10 이상, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, Discovery Server 해제로 돌린다.

| 파일 | 확인하는 것 |
| --- | --- |
| `tests/ros/e3_rehearsal_abc.py` | 실제 조정기·target_locator·db_writer·report_node 와 가짜 로봇으로 재난 신호부터 임무 끝까지 |
| `tests/ros/e3_coordinator_probe.py` | 실제 조정기와 가짜 실행기 2대. 모드 관문, 임대, 보고, 구간 이동, 재확인 |
| `tests/ros/e3_report_gate_probe.py` | 실제 조정기와 report_node. 저장 확인 뒤 출동 관문 |
| `tests/ros/e3_camera_watch_probe.py` | 실제 조정기와 웹캠 `WatchSlot`. 감시 지정과 해제 |

## 아직 없는 것

- 조정기 상태 영속. 상태는 메모리에만 있어 조정기를 다시 켜면 임무를 처음부터 센다.
- 실기 시험. 위 시험은 모두 로봇 없이 돌았다.

## 기록 (2026-10-14 옮김)

아래는 2026-10-13 판 이 문서의 전문이다(옛 제목 「계획 PR 8 — 접근·배정·통로 점유 코어」). 위 본문으로 대체했고, 이력 확인용으로만 남긴다. 아래와 위가 다르면 위가 맞다. 2026-10-14 main `d217680` 코드와 견주어 찾은 차이는 아래와 같다.

- 「지금 안 되는 것」의 「실제 로봇에서는 AMR2(TB-5)가 움직이지 않는다. 실행기는 `SEARCH_AREA` 만 받는다」는 지금 맞지 않는다. 현장 파일의 실행기 허용 목록은 TB-6 이 `REOBSERVE_TARGET`·`RETURN_HOME`, TB-5 가 `APPROACH_TARGET`·`RETURN_HOME` 이다.
- 「RETURN_HOME 은 수락돼도 task 를 발급하지 않는다」와 「아직 하지 않은 것」의 `RETURN_HOME` 배정은 지금 TB-5 출구 이동과 TB-6 복귀로 자동 발급한다. 운영자 `RETURN_HOME` 명령만 아직 임무를 내지 않는다(#233).
- 「열려 있는 PR」의 #148(웹캠 감시 지정)과 #154(SIGTERM 종료)는 머지됐다. 교통 관제(#272)도 머지됐다.
- `search_mode` 파라미터 줄의 「`PATROL` 또는 `AREAS`」와 「현장 지금 `AREAS`」는 지금 `FRONTIER` 가 더해졌고 현장 값도 `FRONTIER` 다.
- 「인형을 찾은 뒤 계속 탐색·두 번 고정은 다음 PR」과 「출구는 지금 직선 거리, 격자 경로 길이는 2차 고정 PR」은 구현됐다. 출구는 최종 지도의 격자 경로 길이로 고른다.
- 「아직 하지 않은 것」의 소실 후보 → TB-6 재확인 자동 예약과 판정 연결은 `completion_mode=RECHECK` 에서 구현됐다.
- 로봇 이름 AMR1·AMR2 는 TB-6·TB-5 다([용어집](../glossary.md)).

### 2026-10-13 판 원문

> 이 문서의 앞 절들은 계획 PR 8 이후 쌓인 기록이다. 지금 코드가 무엇을 하는지는 바로 아래 「지금 상태」가 기준이고, 뒤 절과 어긋나면 이 절이 맞다(2026-10-13).

#### 지금 상태 (2026-10-13, main `2550f1d` 기준)

**되는 것 — 로봇 없이 A·B 단계가 코드로 이어진다** [측정 2026-10-13]

- 로봇 없는 리허설: 재난 신호(DISASTER_SIGN) → 관제 웹 START_SEARCH → AMR1 SEARCH_AREA → 탑재 카메라 탐지 → TargetLocation → 대상 생성 → 보고 저장 영수증(PERSISTED) → 관제 웹 출동 승인 버튼 켜짐 → DISPATCH_SUPPORT 수락 → AMR2 MOVE_SEGMENT.
  - 결과: 7단계가 3회 모두 PASS, 단계마다 1 s 안쪽.
  - 조건: `ROS_DOMAIN_ID` 47·LOCALHOST. 코드는 main `a578d23` + #142 `c2fb926` + #147 `6762a2e` 를 로컬에서만 합쳤다.
  - 실제로 돈 노드: `mission_coordinator`, `target_locator`, `db_writer`(+HTTP API), `report_node`. 관제 웹 화면은 같은 DB 로 만들었다.
  - **가짜였던 것**: 웹캠 DISASTER_SIGN(SYNCED·오차 5 ms 는 시험값), 두 로봇의 실행기와 RobotStatus, AMR 카메라 TargetObservation 과 TF. 영수증·TargetLocation·SystemState 는 가짜가 아니라 실제 노드가 냈다.
  - 스크립트 `tests/ros/e3_rehearsal_abc.py`(A·B·C 와 지도 고정 F 단계). 아래 결과는 main `782371d` 위에서 잰 것이다 [측정 2026-10-13, 3회]:
    - SAVED_MAP A1 ~ C5 11단계: 3회 모두 PASS, 단계마다 2.5 s 안쪽. 재난 신호부터 임무 FINISHED 까지 실제 조정기·target_locator·db_writer·report_node 가 이어진다.
    - SLAM_FREEZE F1 ~ F3: 3회 모두 PASS. 대상 저장 → 고정 호출 1회 → 고정 grid·TB-6 MapState R0+1 → R0 대상 보류(`TARGET_NOT_ON_FROZEN_MAP`) → R0+1 관측 다시 저장 → `SUPPORT_NOT_LOCALIZED` → TB-5 `APPROACH_TARGET`(map_revision 1).
    - 가짜: 웹캠(DISASTER_SIGN·소실 후보·감시 슬롯은 실제 WatchSlot 규칙), 두 로봇 실행기·상태·TB-5 RobotPose, TB-6 카메라 관측·TF·MapState, 고정 서비스·고정 grid. 시계는 시험값(SYNCED, 5 ms).
    - 3회 중 1회 SLAM_FREEZE 실행 끝에 target_locator 가 SIGTERM 때 RCLError(context 무효)로 exit 1 이었다. 조정기 #154 와 같은 종류의 종료 경합이다(통합-인식 레인).
- 출동 승인 관문: 확정(CONFIRMED)이고, 지금 revision 의 보고가 PERSISTED 이고, 운영자가 그 revision 으로 누를 때만 열린다.
  - 단위 시험: `tests/test_coordination_ros_targets.py` `DispatchGateTests`(#139).
  - 실제 report_node 와 함께 한 E3: PASS 6/0 [측정 2026-10-13]. probe 는 `tests/ros/e3_report_gate_probe.py`(#151, 머지됨).
- 보고 크기(#139): 보고에는 아직 PERSISTED 영수증에 들어가지 않은 위치만 싣는다(SDD 391 기록 중복 제거). 같은 대상의 위치가 42개일 때 가장 큰 보고는 고치기 전 59,944 B, 고친 뒤 4,021 B [측정 2026-10-13].

**TB-5 자동 출동·출구 (이 PR 뒤, 저장소 소유자 결정 2026-10-13 1B·2B·Q1-b·5A)**

- `support_dispatch=AUTO` 면 대상이 CONFIRMED 이고 지금 revision 의 보고가 PERSISTED 일 때 TB-5 에 `APPROACH_TARGET` 하나(`has_approval=false`, route.approach_goal = standoff 접근 지점, frame_ref = 대상의 것). 한 임무에 한 번. 거절되면 `TASK_REFUSED` 대기 뒤 새 task(#167).
- 접근이 COMPLETED·정지 확인으로 끝나면 `support_exit_wait_s` 뒤 `RETURN_HOME`(`home_ref=home/<robot_id>`, 좌표는 로봇이 프로파일에서 가짐) 하나. 재난 모드가 끝나면 취소(`SUPPORT_EXIT_CANCELED`), 구출 판정은 취소하지 않는다(5A). 접근 실패면 `APPROACH_FAILED_<사유>`(실행기 결과 `terminal_reason`, 비어 있으면 outcome; 예 `APPROACH_FAILED_PASSAGE_BLOCKED`) 로 보이고 출구로 보내지 않는다(작전 OPS-119).
- `SystemState.reason`: `SUPPORT_EXIT_IN_<n>S` → `SUPPORT_EXITING` → `SUPPORT_AT_EXIT`(실패 `SUPPORT_EXIT_FAILED`).

**지도 고정 뒤 출동 (이 PR 뒤, 저장소 소유자 결정 2026-10-13, scope Q-29 ⑦)**

`map_mode=SLAM_FREEZE`: 기준 지도 없이 TB-6 만 탐색 중 SLAM 을 하고, TB-5 출동 전에 그 지도를 고정한다. `SAVED_MAP` 은 예전처럼 launch 의 `goal_map_*` 기준 하나로 끝까지 간다.

- 고정 요청: 대상 하나가 PERSISTED 이고 TB-6 이 READY·STOPPED·작업 없음이면 `/rescue/map/freeze`(std_srvs/Trigger)를 한 번 부른다. 그 뒤 새 `SEARCH_AREA` 는 내지 않는다(진행 중인 것은 끝날 때까지 기다림).
- 고정 판정: 응답 success 뒤 `/rescue/map/frozen`(OccupancyGrid, latched) 의 해시와 TB-6 MapState(탐색 revision 이 아닌 것)의 session·revision·프로파일 transform_revision 으로 계산한 snapshot id 가 그 MapState 의 `snapshot_id` 와 같을 때만 고정 basis 로 정한다. 해시·snapshot·빈 칸 규칙은 `rescue_contracts.occupancy`(로봇과 같은 함수)다. 응답 실패·잘못된 지도·`map_freeze_timeout_s` 안에 맞지 않음은 `MAP_FREEZE_FAILED` 이고 다시 부르지 않는다. 노드 로그 `MAP_FREEZE`·`MAP_FROZEN`·`MAP_FREEZE_FAILED` 한 줄씩.
- 고정 뒤: 모든 목표(접근·재확인·구간)와 판정이 고정 basis 를 쓴다. 다른 basis 의 위치는 `FRAME_REVISION_MISMATCH` 로 거절하고, 고정 전 대상에서 `merge_distance_m` 안의 고정 지도 관측은 그 대상을 새 revision 으로 옮긴다(보고를 다시 PERSISTED 해야 출동). 복귀 위치도 같은 xy 로 고정 basis 에 옮긴다.
- TB-5 출동 관문(`SystemState.reason`): `MAP_FREEZE_WAITING_SCOUT_STOP` → `MAP_FREEZE_REQUESTED` → `MAP_NOT_FROZEN`(응답 뒤 지도·MapState 대기) / `MAP_FREEZE_FAILED`, `TARGET_NOT_ON_FROZEN_MAP`, `SCOUT_NOT_LOCALIZED`·`SUPPORT_NOT_LOCALIZED`(MapState 가 고정 revision 에서 VALID 아님).
- 빈 칸 검사(반지름 `freeze_clearance_m` = 로봇 `goal_clear_radius_m`, 현장 launch 가 같음을 검사): 접근 지점 `SUPPORT_APPROACH_NOT_FREE`, TB-5 출구 `SUPPORT_EXIT_NOT_FREE`, TB-6 도킹 접근 지점 `SCOUT_HOME_NOT_FREE`, 복귀 위치 없음 `NO_HOME_POSITION`. 다른 지점을 만들지 않고 보낸 것 없이 사유만 낸다.
- 출구 = QR(안 「나」, 작전 OPS-136·138·139·156·158): `/rescue/exit_location`(TargetLocation, `class_label=exit_qr`, QR id 는 `observation.track_id`)이 출구를 **QR id 별로** 등록한다(id 마다 최신 위치, 고정 전 탐색 basis·고정 뒤 고정 basis 만, 노드 로그 `EXIT_REGISTERED`; 허용 목록은 인식 쪽 한 곳에서만 검사). id 없는 관측, 대상 토픽의 `exit_qr`, 출구 토픽의 다른 class 는 거절한다. **지도 고정은 출구를 기다리지 않는다**(OPS-156: 1차 고정은 인형의 것). TB-5 출동도 출구와 무관하고, 출구 이동만 출구가 필요하다: 고정 지도의 출구 가운데 인형에서 가장 가까운 것(지금은 직선 거리, 격자 경로 길이는 2차 고정 PR), 출구에서 대상 쪽으로 `exit_standoff_m` 앞 지점(빈 칸 검사, 막히면 `SUPPORT_EXIT_NOT_FREE`), 출구가 없으면 `SUPPORT_EXIT_UNREGISTERED`.
- 고정 때 재기준(OPS-138·139): 고정 지도가 같은 map_session 이면 탐색 revision 의 대상·출구 최신 위치를 고정 revision 으로 옮긴다. 대상은 revision+1 로 최신 위치를 다시 보고하고 PERSISTED 될 때까지 `TARGET_REBASED_ON_FREEZE`, 로그 `TARGET_REBASED_ON_FREEZE`·`EXIT_REBASED_ON_FREEZE`. 다른 session 은 옮기지 않는다(`TARGET_NOT_ON_FROZEN_MAP`·`EXIT_NOT_ON_FROZEN_MAP`).
- TB-6 출발 자세(OPS-141·142): `/rescue/map/start_pose/<TB-6>`(PoseWithCovarianceStamped, map, latched)를 TB-6 의 VALID MapState(탐색 basis)와 묶어 한 번 받는다(같은 회차 다른 값은 `START_POSE_CONFLICT`). 순찰 지점과(SLAM_FREEZE 의) TB-6 도킹 접근 지점은 출발 자세 기준 좌표이고 `rescue_contracts.occupancy.compose_planar` 로 map 좌표가 된다. 없으면 순찰·TB-6 복귀를 내지 않는다(`START_POSE_UNKNOWN`).
- `~/task_inputs` 목적지에 `home/<TB-5>`(출구 접근점)·`home/<TB-6>`(도킹 접근점)을 고정 basis 로 넣는다(로봇 #232 가 자기 `home/<id>` 키를 받음).
- 지도 고정 두 번(작전 OPS-154·156·159, 통합-로봇 #242): 고정 revision 은 `/rescue/map/frozen_revision`(grid 해시·map session·revision)으로 안다(TB-6 MapState snapshot 방식은 없앰). 1차(R0+1)는 인형 PERSISTED·TB-6 정지 때, TB-5 만 R1 로 출동하고 TB-6 는 SLAM(R0)으로 탐색을 계속한다(FRONTIER). 2차(R0+2)는 `EXPLORATION_COMPLETE`·TB-6 정지 때, 대상·출구를 R2 로 재기준하고 보고는 다시 하지 않는다(OPS-156 ②). TB-6 관측은 지금 TB-6 의 기준(2차 전 R0, 뒤 R2)만 받고, R0 에서 다시 본 인형은 대상을 고정 지도에 둔 채 합친다(같은 SLAM 계보, OPS-156 ④). TB-5 접근 경로의 frame_ref 는 대상 기준이다. TB-6 복귀는 최종 지도 뒤(`MAP_NOT_FINAL`).
- 탐색 끝의 임무 종료(FRONTIER, OPS-154 3·155·173): TB-5 는 인형 앞에서 최종 지도까지 기다린 뒤(`MAP_NOT_FINAL`), R2 격자 경로 길이로 가장 가까운 출구의 접근점(출구 앞 `exit_standoff_m`, 인형 쪽)으로 나간다(로그 `EXIT_CHOSEN`). QR 출구가 없으면 TB-5 시작점(`support_start_pose` = 현장 파일 `robots.<TB-5>.initial_pose_xy_yaw`, TB-6 출발 자세 기준; 없으면 `SUPPORT_START_UNKNOWN`), 출구가 있으나 갈 수 없으면 `SUPPORT_EXIT_UNREACHABLE`. `completion_mode=EXIT_ARRIVAL`(현장): 웹캠 소실 후보는 기록만 하고 재확인을 열지 않으며, TB-6 은 최종 지도 뒤 도킹 접근으로, TB-5 출구 도착이 RESCUE_CONFIRMED(source=EXIT_ARRIVAL), 둘 다 도착하면 FINISHED. `RECHECK` 는 C-11 흐름 그대로. 탐색이 끝났는데 인형이 없으면 두 고정을 그대로 하고 TB-6 만 복귀, TB-5 는 출동하지 않으며 `NO_SURVIVOR_FOUND` 로 FINISHED.
- 교통 관제 1단계(저장소 소유자 결정 2026-10-14, scope Q-29 ⑤, 작전 OPS-204·205): `traffic_control=TB5_PRIORITY`(현장, SLAM_FREEZE 에서만). **TB-5 가 늘 먼저 간다. TB-5 가 사람을 이끌고 사람이 그 뒤를 따라가기 때문이다.**
  - TB-5 새 명령(접근·출구·구간) 전에 고정 지도 격자 경로(`rescue_contracts.grid_search.grid_path`)를 구한다. 없으면 `TRAFFIC_SUPPORT_PATH_UNKNOWN`.
  - TB-6 이 그 경로에서 `traffic_path_clearance_m` 안이면 먼저 비킨다. 비킬 자리는 TB-6 실시간 지도에서 경로·TB-5 로부터 그만큼 밖인 빈 칸 중 격자 경로가 가장 짧은 곳(`nearest_clear_outside`)이고, `traffic_yield_max_m` 이내여야 한다. FRONTIER 자리 하나로 보내며, 목표 yaw 는 TB-6 → 자리 방향이다(제자리 회전 + 전진, 후진 목표 없음). TB-5 는 `TRAFFIC_SUPPORT_WAITING_SCOUT_YIELD`. 자리가 없으면 TB-6 은 그대로 서 있고 TB-5 도 보류한다(`TRAFFIC_SCOUT_YIELD_NO_SPACE`, 대체 동작 없음).
  - TB-6 이 그 경로 근처로 움직이는 중이면 그 task 를 취소한다(`ExecuteTask` goal cancel, 실행기가 세우고 CANCELED). 정지까지 `TRAFFIC_SUPPORT_WAITING_SCOUT_STOP`, `traffic_stop_timeout_s` 를 넘으면 `TRAFFIC_SCOUT_STOP_TIMEOUT`. 취소된 프론티어는 시도로 치지 않고, 취소된 TB-6 복귀는 다시 낸다.
  - TB-5 가 움직이거나 출발을 기다리는 동안 그 경로 근처의 TB-6 새 목표는 보류한다(`TRAFFIC_SCOUT_HELD_FOR_SUPPORT`). 다른 곳이면 TB-6 도 움직인다.
  - 두 로봇이 `traffic_min_separation_m` 미만이거나 새 목표가 상대 로봇과 그만큼 가까우면 새 명령을 막는다(`TRAFFIC_ROBOTS_TOO_CLOSE`). 움직이는 명령은 이 때문에 취소하지 않는다. 예외는 TB-6 비켜서기 하나다(TB-5 에서 멀어지는 쪽, 교착 방지).
  - 상대 위치를 모르면 보류한다(`TRAFFIC_SCOUT_POSE_UNKNOWN`·`TRAFFIC_SUPPORT_POSE_UNKNOWN`). TB-5 는 고정 지도에서만 위치를 잡고 첫 task 전에는 움직이지 않으므로, 그 전 위치는 시작 자리(`support_start_pose`)다. 시작 자리를 아직 안 쟀으면(현장 파일 `robots.robot_b.initial_pose_xy_yaw` 없음) TB-6 을 막지 않고 간격 검사만 끈다. 이 상태에서 TB-6 목표를 낼 때마다 로그 `TRAFFIC_SEPARATION_UNCHECKED` 를 남기고, 다른 이유가 없으면 `SystemState.reason` 에도 같은 코드를 표시한다. 막으면 인형 고정도, 그래서 TB-5 출동도 막히기 때문이다. 두 위치는 같은 SLAM 지도(R0·R1·R2)라 그대로 비교한다(OPS-156 ④).
  - 로그: `TRAFFIC_YIELD`·`TRAFFIC_YIELD_DONE`·`TRAFFIC_SCOUT_CANCELED`·`FRONTIER_STOPPED_FOR_TB5`·`SCOUT_RETURN_STOPPED_FOR_TB5`, 보류가 바뀔 때만 `TRAFFIC_HOLD <역할> <사유>`·`TRAFFIC_RELEASE <역할>`.
- 지도 전체 탐색(`search_mode=FRONTIER`, 저장소 소유자 결정 2026-10-13 밤, 작전 OPS-154·156): TB-6 실시간 SLAM 지도(`/robot6/map`)와 TB-6 RobotPose 로 frontier 덩어리(`frontier_min_cells` 이상, 목표 칸은 `frontier_reach_m` 안)를 찾고(`rescue_contracts.grid_search`, 통합-인식 #241), 이미 시도한 곳(`frontier_blacklist_m` 안) 말고 TB-6 에서 격자 경로가 가장 짧은 것을 고른다. 목표는 그 경로 위 `frontier_goal_max_m` 지점(짧은 목표). SEARCH_AREA 자리(uuid5, 탐색 basis)를 `~/task_inputs` 에 먼저 넣고 `frontier_issue_delay_s`(≥ 로봇 poll 주기) 뒤에 task 를 낸다. 끝난 frontier 는 결과와 무관하게 다시 고르지 않는다. 고를 것이 없는 지도 갱신이 `exploration_empty_checks` 번 이어지면 `EXPLORATION_COMPLETE`, `exploration_timeout_s` 가 지나면 `EXPLORATION_TIMEOUT`(완료로 넘어감). 지도·pose 가 없으면 `EXPLORATION_MAP_UNKNOWN`·`SCOUT_POSE_UNKNOWN`. 노드 로그 `FRONTIER …`·`FRONTIER_DONE …`·`EXPLORATION_COMPLETE …`. 인형을 찾은 뒤 계속 탐색·두 번 고정은 다음 PR.
- 순찰(`search_mode=PATROL`, OPS-139, 지점 방식은 재범 확인 대기): `patrol_points`(출발 자세 기준 [x, y, yaw]*N)를 순서대로 SEARCH_AREA(지점 id = uuid5(map_session, 순번))로 내고, 끝나지 않은 지점은 넘긴다. `patrol_rounds` 바퀴 뒤 `PATROL_EXHAUSTED`, 도는 동안 `PATROL_ROUND_<n>`. RViz 목표 선택과 함께 쓰지 않는다(현장 launch 가 거절). `AREAS` 는 프로파일 구역.

**지금 안 되는 것**

- 실제 로봇에서는 AMR2 가 움직이지 않는다. 로봇 실행기의 LIVE_ROS 입력은 지금 `SEARCH_AREA` 만 받는다(`src/rescue_robot/rescue_robot/task_site_runtime.py:256`). 조정기가 내는 `MOVE_SEGMENT`·`REOBSERVE_TARGET` 은 실제 실행기가 거절한다. 위 리허설의 MOVE_SEGMENT 는 가짜 실행기가 받은 것이다(C1 보류, AMR2 시나리오 #131·#133 결정 대기).
- RETURN_HOME: 복귀 위치는 `home_poses`(TB-6 도킹 접근 지점, TB-5 출구; 저장소 소유자 결정 2026-10-13 5A)로 받고, 로봇의 현재 지도 기준은 각 로봇의 MapState(#176)에서 받는다(이 PR 뒤).
  - 복귀 위치가 없으면 `NO_HOME_POSITION`, 로봇의 MapState 가 VALID 가 아니거나 지도 기준이 복귀 위치와 다르면 `HOME_BASIS_MISMATCH` 다. 다른 위치나 배치 기준으로 대신하지 않는다.
  - 수락돼도 task 는 아직 발급하지 않는다(C단계 판정 → 복귀 연결 뒤).
- 웹캠 소실 후보 → TB-6 재확인 → 구출 판정 → TB-6 복귀(이 PR 뒤, 저장소 소유자 결정 2026-10-13 4A·5A): 수용된 후보가 그 대상의 CAMERA 재확인을 연다(C-11 5행, 운영자 재확인이 열려 있으면 기록만 10행). REOBSERVE_TARGET 이 COMPLETED 면 RecheckObservation 을 `verdict.judge` 로 판정한다(7 ~ 9행: 연결된 task·frame_ref·시각 기준과 순서(`recheck_max_clock_error_s`)·카메라 ABSENT·LiDAR CLEAR·범위 VALID). 관측이 없으면 `RECHECK_NEEDED`(`RECHECK_OBSERVATION_MISSING`)로 닫고, COMPLETED 가 아니면 새 task. 판정마다 노드 로그에 `VERDICT target=… state=… reason=… source=… link=… task=…` 한 줄, `SystemState.reason` 은 `RECHECK_NEEDED_<이유>`. RESCUE_CONFIRMED 뒤 TB-6 에 `RETURN_HOME`(`home/<robot_id>`, 도킹 접근 지점, 도킹은 사람이 함) 하나 → `SCOUT_RETURNING` → `SCOUT_AT_DOCK_APPROACH_DOCK_BY_HAND`. TB-6 도착과 TB-5 출구 도착이 모두면 phase `FINISHED`·mode `MISSION_FINISHED`. 운영자 재확인(11행)은 조정기 자신의 시계를 이 경로에서 재지 않아 늘 `CANDIDATE_TIME_BASIS_UNUSABLE`(동결 뒤 후보: live_inputs 시계 사용).
- `route_segment_passage_free` 는 ROS 파라미터로 빈 배열을 줄 수 없다(`-p`·`--params-file` 모두 값 없음 → 기동 거절) [측정, 통합-로봇 2026-10-13]. 모든 구간을 보류하려면 모두 `false` 로 준다. 아래 파라미터 표의 문구는 #148 머지 때 함께 고쳐진다.

**열려 있는 PR (이 레인)**

- #148: 웹캠 감시 지정(SetCameraWatch).
  - 운영자 `SET_CAMERA_WATCH` 마다 `watch_id` 를 발급한다. 재난 모드에서만 받는다.
  - 소실 후보를 받을지 판단한다(C-11 2 ~ 4행). 끝나면 해제한다.
  - 새 파라미터 `camera_watch_timeout_s`(필수)·`camera_watch_roi`(생략 가능, 생략하면 요청 ROI 만).
  - 머지 전 main 에서는 `SET_CAMERA_WATCH` 가 아래 대응표대로 `INVALID_INPUT` 이다.
- #154: SIGTERM 으로 끌 때 context 가 이미 닫힌 뒤 나는 RCLError 를 정상 종료로 본다. 그 밖의 오류는 그대로 낸다.
  - 리허설에서 고치기 전에는 조정기가 8회 중 4회 exit 1 로 끝났고, 고친 뒤 5회 모두 exit 0 이었다 [측정 2026-10-13].

**현장 연동(#132)**: `live_inputs.*` 파라미터·`~/task_inputs`·`~/refresh_state`·RViz 목적지 선택은 [rescue_coordination README](../../src/rescue_coordination/README.md)「현장 연동」에 있다. 아래 파라미터 표에는 아직 없다.

계획 PR 8(실제 GitHub PR 9)의 현재 범위는 접근 후보·배정 수락·대상 확인·통로 점유의 Python 정책 코어다. **수색 구역 분할·SEARCH_AREA 구역 배정·D-05 B안 frontier 후보 어댑터·자동 역할 교대는 F-01 후속 작업이며 GitHub PR 미발행**이다. 해당 기능을 이 PR이나 통합 리허설의 완료에 포함하지 않는다. 실제 Nav2 path service·ROS 임무 전송도 미구현이다. `PathCandidate`는 외부 planner가 제공할 증거이며 후보 선택이 이동 허가는 아니다. 실제 SLAM 보정 연동이 없어 mapping의 `auto_approach_eligible`은 false다.

#### 구현한 경계

| API | 입력·판정 | 결과 |
| --- | --- | --- |
| `select_candidate(..., now_ns=...)` | 명시적인 가용 로봇, 경로 길이, footprint/경로 free 여부, clearance, 불확실성, `valid_until_ns` | 검증 입력을 만족하는 최단 경로 후보 또는 None. 대상 점 자체를 목표로 삼지 않음 |
| `Allocator(authority_epoch, allowed_robot_ids=...)` | 프로파일의 정확히 두 ID | DTO는 공통 ID 문법만 확인. allocator가 배포에 속하지 않은 로봇의 발급/예약을 거절 |
| `issue_generation(robot_id)` | 로봇별 발급 이력 | 0부터 증가하는 uint64. 발급은 수락이 아니며 `issued_generations`만 변경 |
| `reserve(Reservation)` | 발급된 미소비 세대, task/command/target/robot/boot/authority, 사건·로봇 단일 예약 | `pending`에만 저장. True는 예약 접수이며 실행기 수락을 뜻하지 않음. 두 카운터를 변경하지 않음 |
| `record_admission(TaskAdmission)` | 실행기 수락 응답의 command/task/robot/boot/authority/generation 정확한 일치 | accepted=true일 때만 `active`와 `accepted_generations` 갱신. 거절·늦은 응답·내용 충돌은 새 활성 임무를 만들지 않음 |
| `release` | 같은 task/robot/generation/authority와 stop_confirmed | pending/active 해제. 수락 응답 유실 중에도 확인된 정지 없이 예약을 버리지 않음 |
| `ApproachTracker.observe/guard` | 원 관측 ns 시각·기한, 증가하는 관측 시각, 사건/revision·신원·최신성, 로컬 수신 간격 | 새 관측만 local gap 갱신. 원 기한·짧은 유실 허용·guard·시계 순서가 깨지면 비가역 CANCEL |
| `ApproachTracker.complete` | 종료 시각 이후 재관측, 나이·거리·불확실성, 경로 성공·정지 확인 | 모든 조건을 만족할 때만 접근 확인 true |
| `Corridor.confirm_vacant(holder=...)` | 현재 revision, fresh 이탈 확인, 점유자의 command/task/robot/boot/authority/generation 일치 | HELD와 holder를 보존한 UNKNOWN 모두 동일 holder 확인이 필요. 최초 holder 없는 UNKNOWN은 holder=None과 fresh 비움 증거로만 FREE |

#### generation과 실행기 수락

세대의 발급 주체는 coordinator다. `issued_generations`는 발급 번호를 재사용하지 않는 로컬 장부이고, `accepted_generations`는 실행기 admission의 수락 이력이다. 실행기 자체의 최고 수락 세대 검사는 유지한다. coordinator가 예약했다고 실행기 카운터와 수락 결과를 가정하지 않는다. `Reservation`에는 `command_id`와 `robot_boot_id`가 필수이고 `rescue_contracts.TaskAdmission`과 같은 여섯 신원 필드를 비교한다.

PENDING도 사건·로봇을 독점한다. 거절 receipt는 pending을 제거하지만 수락 카운터를 올리지 않으며 세대를 재사용하지 않는다. 수락된 receipt의 같은 내용 재전송은 멱등이다. 동일 receipt의 거절/수락 또는 내용 변경은 `ID_CONTENT_CONFLICT`다. 해제된 예약의 늦은 수락은 active를 되살리지 않는다. command ID를 다른 task에 재사용하거나 이미 소비한 세대로 새 task를 예약하지 않는다.

이 상태는 메모리 안에 있으며 durable coordinator·재시작 세대 복구·권한 인증을 구현하지 않는다. 살아 있는 실행기와 연결한 채 coordinator만 재시작해서 0부터 발급하는 것은 유효한 복구 절차가 아니다. 후속 운영 어댑터는 명시적 authority 재등록·정지 관문과 영속 발급 이력/실행기 수락 이력 조정을 구현해야 한다. E2 receipt 객체 자체가 네트워크 발신자 인증은 아니다.

#### 시각·오류·외부 입력 책임

`PathCandidate.valid_until_ns`, `select_candidate(now_ns=...)`, `ApproachTracker.observe(observed_at_ns, valid_until_ns, now_wall_ns, ...)`, `guard(now_wall_ns, ...)`, `complete(now_wall_ns, navigation_finished_at_ns, ...)`는 양의 int64 ns다. `confirmation_max_age_ns`는 음수 아닌 int64 ns duration이다. float·bool·범위 밖 정수는 거절한다. 실제 센서의 sec/nanosec은 공통 `ros_time_to_ns`로 옮기며 epoch float로 만든 뒤 정밀도를 복원하지 않는다. `received_steady`, `now_steady`, `short_loss_max`와 통로 `now`/`duration`만 프로세스 로컬 monotonic 초다.

`CoordinationError`는 공통 `RescueError`를 상속한다. 소비자는 C-09 canonical `code`, `component`, namespace가 붙은 `detail_code`, 설명 `detail`을 읽는다. `DETAIL_CATEGORIES['rescue_coordination']`에 등록한 대문자 진단 코드만 사용하며 자유 문장은 `detail`에 둔다. 입력 진단은 명시한 대응표에 따라 INVALID_INPUT으로 분류한다. 미등록 코드나 오타는 ValueError를 전파하며 기본 분류로 조용히 바꾸지 않는다. 시계 역행은 CLOCK_UNCERTAIN, 의미 payload 충돌은 ID_CONTENT_CONFLICT다. 예상 가능한 배정 거절은 bool false이고 실제 실행기 거절의 canonical reason은 원 receipt에 남는다.

C-03/06/07, CT-06/09/10/11의 일부 E2 불변식을 구현한다. 호출자는 perception의 시각 검증, mapping의 원본별 좌표/목표 snapshot, executor admission과 stop 결과를 실제로 연결해야 한다. bool 입력 자체는 fresh 센서/정지/경로 증명이 아니다. `CONTINUE_EXISTING`은 현재 접근 목표의 짧은 유실 허용이며 새 목표 발행 허가가 아니다. A의 수색을 B의 대상 만료로 중단하는 전역 guard로 사용하지 않는다.

거리·속도·유실 시간을 장비값으로 확정하지 않는다. 메모리 통로 상태는 처음부터 UNKNOWN이며 lease 만료와 pose 손실 뒤에도 holder를 보존한다. 분산 election/lease 인증·영속 배정·분할 통로 deadlock 해결·실제 수색 분담은 후속 과제다.

#### 역할 입력 수색·대기·접근 예약 (F-01 축소분, 2026-10-08)

BRD v0.1 시나리오(TB1 정찰, TB2 구조 지원)에 맞춘 최소 배정 정책이 `rescue_coordination/search.py`에 있다. 역할은 `SearchPolicy.roles`(`SCOUT`/`SUPPORT`) 입력이며 코드에 로봇 ID를 고정하지 않는다. 역할 입력을 바꾸면 두 로봇의 임무가 뒤집힌다. 수락 판정과 세대 장부는 두지 않고 위 `Allocator`만 읽고 쓴다.

| API | 판정 | 결과 |
| --- | --- | --- |
| `MissionSite`, `SearchPolicy` | 구역 ID·mission 좌표·근거 태그. `executable_bases`에 있고 map session/revision·transform revision이 모두 있는 구역만 실행 대상 | 나머지는 `excluded_areas()`에 닫힌 사유(`EXCLUSION_REASONS`)로 표시 |
| `plan(robot, robot_boot_id, ready, now_ns)` | 이미 pending/active가 있으면 `KEEP`. SCOUT는 설정 순서상 첫 미완료·미예약 구역, SUPPORT는 제자리 `HOLD` | `RESERVED`는 PENDING 예약과 `executor.Task` 필드. 비용 비교가 없어 작은 비용 변화로 임무가 바뀌지 않음. 동률은 설정 순서로 생기지 않음 |
| `request_approach(event_id, ...)` | mapping projection `VALID`, SUPPORT 소유의 유효한 `select_candidate` 결과, SUPPORT의 HOLD 해제 | 사건 ID당 접근 예약 1개. HOLD 중이면 `RELEASE_REQUIRED`, 다른 접근 중이면 `SUPPORT_BUSY` |
| `record_admission`, `finish(item, outcome, stop_confirmed)` | 실행기 receipt만 ACTIVE를 만들고, `release(stop_confirmed=True)`만 해제 | `COMPLETED`만 구역 수색 완료로 집계. 해제된 task의 늦은 receipt·결과는 새 배정을 되살리지 않음 |

재배정은 (1) 현재 구역 결과 수신·정지 확인 (2) `withdraw_area`로 구역 근거 철회(`RELEASE_REQUIRED`) (3) 로봇 부적격 뒤 실행기 정지 결과에서만 일어난다. SUPPORT가 접근 중일 때 SCOUT는 기본적으로 다음 구역 수색을 계속하고(`scout_continues_during_approach=True`), False면 새 구역 제안만 멈춘다. 반환 사유는 `DECISION_REASONS` 닫힌 집합이다.

`Reservation.task_kind`가 추가됐다. `SEARCH_AREA`·`HOLD`는 `candidate=None`을 허용한다. `APPROACH_TARGET`은 `PathCandidate`가 필수다(기본값이 `APPROACH_TARGET`이라 기존 호출도 후보가 필요). 2026-10-11(#88) 경로 증거 Phase 1 규칙부터, ONE_SEGMENT 승인과 구간 끝 `GoalPoint` 가 있는 `MOVE_SEGMENT`, 고정 관측 지점이 있는 `REOBSERVE_TARGET` 도 후보 없이 받는다(아래 「임무 조정기 ROS 노드」, `planning.py` `Reservation.__post_init__`). 예약은 PENDING이며 후보는 접근 전용 경로 증거다. `HOLD`는 실행기 의미대로 제자리 정지 유지이며, 대기 위치로의 이동은 기반 레인 후속이다. `task_fields`는 #14의 `search_area_ref`(SEARCH_AREA는 구역 ID, 그 외 None)와 `goal_pose`(SEARCH_AREA는 구역 mission 좌표와 근거 map session/revision, APPROACH는 후보 `goal_xy`와 후보 map session/revision, HOLD는 None)를 채운다. frame은 정책의 `mission_frame_id`다. yaw는 리터럴로 채우지 않는다: SEARCH_AREA는 구역의 필수 입력 `MissionSite.entry_yaw`(프로파일 v2.1 예정), APPROACH는 후보 지점에서 대상을 바라보는 각도 `atan2(target − goal)`(재관측 카메라가 대상을 향함)이다. 후보 지점과 대상이 같으면 방향이 없으므로 예약하지 않는다. `goal_pose`는 일반 dict로 주고 로봇 레인이 `GoalPose(**d)`로 바꾼다. coordination이 `rescue_robot`을 import하지 않게 하기 위해서다. 실제 지도 좌표 근거·frontier·전송·두 AMR·4 PC는 미실행이다.

공통 어휘와 로더(OPS-01, 2026-10-08): `planning.TASK_KINDS`·`TARGET_TASK_KINDS`와 `search.ROLES`는 복제하지 않고 `rescue_contracts`의 `TASK_KINDS`·`TARGET_TASK_KINDS`·`ROBOT_ROLES`를 그대로 쓴다. `search.mission_sites(areas, *, entry_yaws)`는 프로파일 v2 `mission.search_areas`를 순서대로 `MissionSite`로 옮긴다. 구역 ID·mission 좌표·근거 종류·map session/revision·transform revision을 그대로 복사하고, 실행 가능 여부는 판단하지 않는다(`SearchPolicy.executable_bases`가 정한다). 프로파일 v2에는 아직 `entry_yaw`가 없으므로 구역 ID마다 도착 방향을 `entry_yaws`로 따로 받는다. 빠지거나 남거나 유한하지 않으면 거부하고 기본값으로 채우지 않는다. 프로파일 v2.1이 `search_areas[].entry_yaw`를 갖게 되면 호출하는 쪽이 그 값을 넘긴다. coordination은 `rescue_bringup`을 import하지 않고 속성으로만 읽는다.

`goal_pose` dict에는 `transform_revision`도 넣는다(APPROACH는 후보의 값, SEARCH_AREA는 구역 근거의 값). `GoalPose.transform_revision`은 필수 필드다. 이 변경은 로봇·지도의 지도 보호 PR #22에서 들어왔고, 협업·관제가 그 부분을 검토했다.

#### 출처와 변경 이유

- P4 [배정기](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/dispatcher.py#L253-L276)는 배정 책임을 한 곳에 둔 원리만 참고한다. 도킹한 로봇만 선택하거나 도크와 구역의 직선거리로 비용을 계산하는 정책은 사용하지 않는다.
- AMR [목표 갱신·도착](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L388-L446), [만료](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L517-L528)의 갱신 억제/유실 경험을 참고했다. 목표 자체로 접근하는 정책을 거리 구간·불확실성 조건으로 대체하고 도착을 fresh 재관측·정지와 분리했다.

원본 소스 복사 없이 새로 작성했다. 실제 길찾기 성공이나 사람 신원 판별 성능을 증명하지 않는다.

#### 출동 대상 선택 (BRD Part I 3.4.3·3.4.5, Q-19, 2026-10-08)

`rescue_coordination/dispatch.py`의 `select_dispatch_target`은 확정된 대상이 여럿일 때 SUPPORT가 다음에 갈 대상을 고른다. `SearchCoordinator.select_approach_target`이 이를 감싸며, 이미 접근 예약이 있는 사건(`approach_tasks`)을 넘겨준다. 고르기만 하고 예약은 하지 않는다. 고른 결과를 `request_approach`에 넘기면 사건당 접근 하나가 지켜진다.

- 거리 기준은 정책 필수값 `DispatchPolicy.order`이고 기본값이 없다(Q-19).
  - `STRAIGHT_LINE`(제안 시작값): SUPPORT의 mission 좌표 `SupportOrigin`에서 대상까지의 직선거리다. 위치와 후보의 map session·revision·transform revision이 같을 때만 비교하고, 다르면 `BASIS_MISMATCH`다. 위치가 없거나, 다른 로봇이거나, `max_origin_age_ns`보다 오래됐거나, 미래 시각이면 아무것도 고르지 않는다(`ORIGIN_UNUSABLE`).
  - `PATH_LENGTH`: SUPPORT 후보의 검증된 경로 비용이다.
- 동률은 사건 ID가 작은 쪽이다. 입력 순서와 무관하다. 지원이 비었을 때만 고르므로, 작은 거리 변화로 진행 중인 접근을 바꾸지 않는다.
- 제외 사유는 닫힌 집합 `DISPATCH_SKIP_REASONS`다:
  - `NOT_CONFIRMED`: 미확정 후보(3.3.5)
  - `RESCUED`: 구출 완료
  - `EVENT_ALREADY_HAS_APPROACH`: 중복 출동 금지(3.4.5)
  - `PROJECTION_NOT_VALID`
  - `NO_SUPPORT_CANDIDATE`
  - `CANDIDATE_EXPIRED`
  - `BASIS_MISMATCH`
- `RECHECK_NEEDED` 대상은 다시 갈 수 있다. 접근이 실패해 해제된 사건도 `approach_tasks`에 남으므로 자동으로 다시 고르지 않는다. 재출동 규칙은 미결정이다.

#### 임무 모드·운영자 명령 관문 (scope Q-12, 2026-10-10)

`rescue_coordination/mission.py`의 `MissionCoordinator`는 `SearchCoordinator` 위에 얹은 임무 한 건의 상태다. 세대 장부는 두지 않고 예약은 `SearchCoordinator.plan`에 맡긴다. 흐름과 관문은 [관제 웹·DB 계획](../design/console-and-db-plan.md) 1·4절과 [상태 전이](../design/state-machines.md) S-00을 따른다.

| API | 판정 | 결과 |
| --- | --- | --- |
| `record_site_event(event)` | `DISASTER_SIGN`이고 `SiteEvent.time_basis_declared`(C-03: stamp 의미 ≠ UNKNOWN, `SYNCED`, 오차 상한 있음)일 때만 NORMAL → DISASTER | 닫힌 사유 `MODE_EVENT_REASONS`. 다른 종류는 `NOT_DISASTER_SIGN`, 시각 기준이 없으면 `TIME_BASIS_UNUSABLE`, 이미 재난이면 `ALREADY_DISASTER`(첫 트리거 유지). 같은 event_id는 첫 결과를 그대로, 내용이 다르면 `ID_CONTENT_CONFLICT` |
| `plan(robot, ...)` | SCOUT는 DISASTER 전에, 그리고 운영자 `START_SEARCH` 를 받기 전에는 `SearchCoordinator.plan`을 부르지 않는다(저장소 소유자 결정 2026-10-11, #85). SUPPORT의 HOLD(제자리 정지)는 통과 | `PlanGate(reason, decision)`. `MODE_NOT_DISASTER`·`SEARCH_NOT_STARTED`면 예약이 없다 |
| `handle_command(OperatorCommand, now_ns)` | 서버 쪽 관문. 모든 명령은 DISASTER에서만. `START_SEARCH` 는 한 번만(다시 오면 `SEARCH_ALREADY_STARTED`). `DISPATCH_SUPPORT`는 확정(`detection_confirmed`)·저장(`report_persisted`)·최신 revision·경로가 있는 대상에 한 번. `CONTINUE_SEGMENT`는 `segment_phase == AWAITING_APPROVAL`이고 대상 revision·경로 revision·구간 번호가 지금 것과 같을 때. `REQUEST_RECHECK`는 [계약](../design/contracts.md) C-11 10행(Q-26, 2026-10-11)대로 기록된 대상이고, `RESCUE_CONFIRMED`로 판정이 끝나지 않았고, 그 대상에 열린 재관측이 없을 때(출동·도착은 조건이 아니다). 받으면 대상별 `RecheckRequest`(출처 `OPERATOR`, command_id 를 후보 자리에, 수락 시각 `now_ns`)를 연다. `RETURN_HOME`은 판정이 나온 뒤, 두 역할 로봇 모두 복귀 위치(`HomePosition`)가 있고 그 지도 근거(map session·revision·transform revision)가 `record_map_basis`로 받은 지금 지도와 같을 때. 아니면 `NO_HOME_POSITION`·`HOME_BASIS_MISMATCH`로 거절하고 다른 위치로 대신하지 않는다 | `CommandResult(accepted, reason)`, 거절 사유는 닫힌 집합 `COMMAND_REJECT_REASONS`. 같은 command_id는 첫 결과를 그대로(그 뒤 대상이 판정됐어도 재판정하지 않음), 내용이 다르면 `ID_CONTENT_CONFLICT`. 종류 밖 명령·필드 누락/과잉은 `INVALID_INPUT` |
| `segment_started`·`segment_finished` | 승인 한 건이 구간 하나다(`SegmentApproval`: 명령 ID, `target_id`·`target_revision`·`route_revision`·`segment_index`). 이 네 키가 같은 구간만 `APPROVED → MOVING`. 끝은 정지 확인이 있어야 받고, 받으면 승인을 다 쓴다 | 완료면 다음 구간이 `AWAITING_APPROVAL`(마지막이면 `ARRIVED`), 미완료면 같은 구간이 다시 승인 대기. **다음 구간을 스스로 승인하지 않는다** |
| `record_route(target, route_revision, segment_count)` | 새 revision은 정지해 승인을 기다릴 때(또는 출동 전)만 바꿀 수 있고, 바뀌면 구간 0부터 다시 센다. 낮은 revision은 거절 | 이전 revision으로 누른 "계속"은 `ROUTE_REVISION_STALE` |
| `phase()`·`view()` | 1절의 표시 단계(`MISSION_PHASES`)를 계산한다. 화면은 읽기만 한다 | `CANDIDATE`는 `TARGET_DISAPPEARED_CANDIDATE`(#48) 전까지 나오지 않는다 |

`MOVE_SEGMENT`·`RETURN_HOME`은 이제 `rescue_contracts.TASK_KINDS`에 있다(`MOVE_SEGMENT` 는 대상 임무). 이 모듈(`MissionCoordinator`)은 승인 상태만 바꾸고, 발급은 노드 코어(`CoordinatorCore`, 아래 절)가 한다.

- `MOVE_SEGMENT`: 승인 한 건마다 SUPPORT 에 하나를 발급한다. `approval` 의 네 키를 그대로 싣는다.
- `REOBSERVE_TARGET`: 열린 재확인마다 SCOUT 에 하나를 발급한다.
- `RETURN_HOME`: 수락해도 상태만 바뀌고 task 는 없다. 복귀 위치와 현재 지도 기준이 없어 지금은 늘 거절된다(위 「지금 상태」).
- 후속: 카메라 후보(`TARGET_DISAPPEARED_CANDIDATE`) 수용과 출처 `CAMERA` 경로(#148 머지 뒤 수용 판단까지; 재관측 자동 예약은 C-11 5행 후속), 11행의 수락 시각 clock domain·오차 기록, 판정 연결.

ROS·DB·웹 연결은 로봇 없는 E3 로 확인했다(위 「지금 상태」).

#### 임무 조정기 ROS 노드 — 얇은 버전 (10-13 전 구간, 2026-10-11)

`rescue_coordination/ros_node.py`의 `mission_coordinator`(console_scripts)는 SDD `central.coordinator`의 얇은 버전이다. 판단은 ROS 없이 시험하는 `CoordinatorCore`가 하고, `main()`은 메시지만 옮긴다. ROS 패키지는 함수 안에서 import 하므로 ROS 없는 환경에서도 모듈과 코어를 시험할 수 있다. 로봇을 움직이는 것은 실행기이고, 이 노드는 `ExecuteTask` goal 을 보낼 뿐이다.

| 통신 | 방향 | 하는 일 |
| --- | --- | --- |
| `/rescue/site_events` (SiteEvent) | 받음 | `VALID` 시야만 사건으로 바꾼다(C-10). `DISASTER_SIGN` → 모드 관문(재난 모드 표시·저장까지) |
| `/rescue/robots/{r}/status` (RobotStatus) | 받음 | `READY`·나이 ≤ `status_max_age_s` 일 때만 새 task 를 낼 준비가 됨. `lease_valid` 는 작업 중 정지 감시에만 쓴다(대기 중인 로봇은 lease 가 없다, #100) |
| `/rescue/robots/{r}/execute_task` (ExecuteTask) | 보냄 | 운영자 `START_SEARCH` 를 받은 뒤 SCOUT 에 `SEARCH_AREA`(저장소 소유자 결정 2026-10-11, #85: 재난 신호만으로는 출동하지 않음). SCOUT 상태를 모르거나 준비 전이면 `START_SEARCH` 는 `REJECTED`·`NAV_NOT_READY` 이고 시작으로 치지 않는다(#98: 버튼 뒤 알 수 없는 시점에 출발하지 않음, 준비 뒤 다시 누름). 준비됐지만 다른 task 중이면 받고, 수색은 그 task 뒤에 나간다. 기다리는 동안 `SystemState.reason` 은 `SEARCH_QUEUED` 이고, 그 사이 재난 모드가 끝나거나 SCOUT 상태가 STOPPING·STOP_UNCONFIRMED 이거나 SCOUT task 가 CANCELED(실행기의 정지 결과)로 끝나면 대기 수색을 거둔다(운영자가 다시 누름, 작전 2026-10-11), 승인마다 SUPPORT 에 `MOVE_SEGMENT`, 열린 재확인마다 SCOUT 에 `REOBSERVE_TARGET`. 결과는 정지 확인이 있어야 예약을 푼다 |
| `/rescue/robots/{r}/task_admission` (TaskAdmission) | 받음 | ACTIVE 로 가는 유일한 길. goal 수락만으로는 ACTIVE 가 아니다 |
| `/rescue/target_locations` (TargetLocation) | 받음 | `VALID` 투영만 대상이 된다. 아래 병합 규칙 |
| `/rescue/reports` (RescueReport) | 보냄 | 대상 revision 마다 보고 하나. `report_retry_s` 마다 PERSISTED·REJECTED 까지 다시 보낸다. 위치는 아직 PERSISTED 영수증에 들어가지 않은 것만 싣는다(기록 중복 제거, SDD 391). 같은 대상의 앞선 미저장 보고는 새 보고가 덮으므로 재전송을 멈춘다 |
| `/rescue/report_receipts` (ReportReceipt) | 받음 | 그 보고의 기록을 모두 포함한 `PERSISTED` 만 저장 확인(출동 관문의 "저장") |
| `/rescue/operator_command` (SubmitOperatorCommand) | 서비스 | 임무 관문 판정. 같은 command_id·같은 내용은 `DUPLICATE` 로 첫 응답을, 내용이 다르면 `ID_CONTENT_CONFLICT` |
| `/rescue/robots/{r}/command_lease` (CommandLease) | 보냄 | task 마다 `lease_id` 를 `ExecuteTask.lease_id` 에 싣고 `lease_renew_period_s` 마다 갱신한다(`renewal_sequence` 1씩 증가, 같은 boot·authority·generation). task 종료·거절, 재난 모드 아님, 그 로봇 상태가 낡음·다른 boot, authority/generation 불일치면 그 lease 는 다시 갱신하지 않는다. 그러면 실행기가 `LEASE_EXPIRED` 로 스스로 정지한다(C-10 「명령 lease」, #88). 보조 규칙(작전 2026-10-11): 그 task 를 가리키는 RobotStatus 에 `stop_reason` 이 있거나 `motion_state` 가 STOPPING·STOP_UNCONFIRMED 이거나 `lease_valid=false` 이면, 또는 수락 뒤 도착한 상태가 다른 task 를 가리키면 갱신을 멈춘다. 정지된 로봇에 lease 를 계속 연장하지 않는다는 뜻이고, 정지 자체는 실행기 latch 몫이다. 수락 전 상태와 이전 task 의 정지 사유는 새 lease 를 끊지 않는다. `valid_until_ns` 는 그 갱신 메시지의 수신 만료(`lease_message_ttl_s`, AST88-02) |
| `/rescue/targets` (TargetSnapshot, TRANSIENT_LOCAL) | 보냄 | 바뀔 때만, 대상마다 `TargetEstimate` 하나(target_id 순). `persisted_report_id` 는 지금 revision 의 저장된 보고만, 확정 상태는 `CONFIRMED`(얇은 버전), 출동 대상의 `approach_state` 는 APPROVED→ASSIGNED·MOVING/AWAITING_APPROVAL→IN_PROGRESS·ARRIVED→APPROACH_COMPLETE [제안], 나머지는 UNASSIGNED |
| `/rescue/system_state` (SystemState, TRANSIENT_LOCAL) | 보냄 | 바뀔 때만. `segment_phase` 는 APPROVED·MOVING→EXECUTING, AWAITING_APPROVAL→AWAITING_APPROVAL, ARRIVED→STOPPED. phase `FINISHED` 면 mode `MISSION_FINISHED`(C-10, #88). `reason` 은 지금 남아 있는 발급 막힘(`PASSAGE_ASSESSMENT_REQUIRED`·`ADMISSION_TIMEOUT`·`TARGET_ALIGNMENT_NOT_VALID` 등, tick 마다 다시 판정해 풀리면 지움, OPS-70-01)이 먼저, 없으면 `SEARCH_QUEUED`, 둘 다 없으면 빈 값 |
| `/rescue/cameras/{camera}/set_watch` (SetCameraWatch, IF-078) | 클라이언트 | 운영자 `SET_CAMERA_WATCH`(재난 모드, 기록된·확인된·판정 전 대상, #124)마다 새 `watch_id` 를 발급하고 프로파일의 `TARGET_WATCH` 카메라 1대에 enable 요청을 보낸다(`association_method=OPERATOR_ROI`, `association_evidence_id`=command_id, C-11 2행). ROI 는 요청 값, 없으면 `camera_watch_roi`, 둘 다 없으면 거절(`PENDING_EVIDENCE`, 화면 전체 기본값 없음). 카메라가 수락해야 활성 감시이고 거절·마감 초과면 지우고 `SystemState.reason=WATCH_REARM_NEEDED`. 요청은 한 번에 하나. 재난 모드가 아니게 되거나 복귀가 시작되거나 대상이 `RESCUE_CONFIRMED` 이거나 후보 1건을 받으면 enable=false 로 해제한다 |
| `/rescue/robots/{r}/map_state` (MapState, RELIABLE·TRANSIENT_LOCAL·depth 1, #176) | 받음 | `localization_quality` VALID 면 그 로봇의 지금 지도 기준(map_session_id·map_revision, transform_revision 은 프로파일 값 = `goal_transform_revision`)으로 기록(`record_map_basis`). 그 밖이면 기록을 지운다. RETURN_HOME 이 복귀 위치와 비교한다 |

- **경로 증거 Phase 1 규칙**(작전 2026-10-11, #88): `EvaluateRoute` 가 2차라 planner 경로 증거가 없다. `planning.Reservation` 은 ONE_SEGMENT 승인과 구간 끝 `GoalPoint` 가 있는 `MOVE_SEGMENT`, 고정 관측 지점이 있는 `REOBSERVE_TARGET` 만 `PathCandidate` 없이 받는다. 승인 없는 MOVE_SEGMENT, 관측 지점 없는 REOBSERVE, APPROACH_TARGET 은 여전히 거절한다. `ExecuteTask.route` 에는 `approach_goal`(구간 끝 또는 관측 지점)만 싣고, `planned_path`·`path_sha256`·`planner_evidence_id` 는 비우며, `assessment` 는 `UNKNOWN`(알려진 값 없음)이다. `route.frame_ref` 는 대상의 마지막 `TargetLocation.frame_ref` 를 그대로 쓰고, alignment 가 `VALID` 가 아니거나 목표의 지도 기준과 다르면 발급하지 않고 로그에 남긴다(`TARGET_ALIGNMENT_NOT_VALID`·`ROUTE_BASIS_MISMATCH`). 실제 경로 검증은 실행기 guard·Nav2 몫이고 실패는 `TaskResult` 로 드러난다.
- **대상 병합**(작전 2026-10-11, Q-13·Q-21): 같은 지도 기준(map session·revision·transform revision)이고 `merge_distance_m` 안이면 같은 대상이고 revision 이 1 오른다. 밖이면 새 `target_id`(조정기 발급 UUID, C-11 1행)다. 기준이 다른 위치는 새 대상으로 나누지 않고 `FRAME_REVISION_MISMATCH` 로 거절한다. 같은 `location_id` 는 멱등, 내용이 다르면 `ID_CONTENT_CONFLICT`. 얇은 버전은 인식이 확정 탐지만 발행한다고 본다(작전 결정 3).
- **관측 시각 자격**(OPS-103, #142, 2026-10-13): 새 `location_id` 는 `observation.time` 이 쓸 만할 때만 대상이 된다. 시각 기준 선언(stamp 의미 ≠ UNKNOWN·SYNCED·오차 상한 있음), `target_clock_domain_id` 와 같은 도메인, 지금보다 `target_max_future_s` + 관측 자신의 오차 상한(`clock_error_bound_ns`) 넘게 앞서지 않음(PC 사이 시계 차가 timesyncd 로 수백 ms [측정 2026-10-13]), `target_max_age_s` 이내이고 생산자의 `valid_until_ns` 전. 어기면 대상·보고를 만들지 않고 `SystemState.reason` 에 `TARGET_TIME_BASIS_UNUSABLE`·`TARGET_CLOCK_DOMAIN_MISMATCH`·`TARGET_OBSERVATION_FUTURE`·`TARGET_OBSERVATION_STALE` 를 다음 위치가 받아들여질 때까지 남긴다(발급 막힘·SEARCH_QUEUED 보다 뒤). 이미 받은 `location_id` 의 재전송은 시각을 다시 보지 않고 같은 대상이다. perception 의 `validate_freshness` 와 달리 받는 쪽 오차 합·상한은 보지 않는다(조정기 자신의 오차를 여기서 재지 않음) [제안].
- **발급 순서**: 한 tick 에서 승인된 구간 → 열린 재확인 → SCOUT 수색 순서다. 그래서 비어 있는 SCOUT 은 새 구역보다 열린 재확인을 먼저 받는다. 막힌 재확인은 수색을 멈추지 않는다. 진행 중인 수색 구역은 끊지 않는다. 다음 구간은 운영자 `CONTINUE_SEGMENT` 없이 발급하지 않는다. 완료되지 않은 재관측은 새 task 로 다시 예약한다(C-11 5행). 실행기가 수락 전에 거절한 구간 이동·재관측은 `admission_timeout_s` 동안 `SystemState.reason=TASK_REFUSED` 로 기다린 뒤 같은 승인·재확인으로 새 task 를 낸다(#90 의 재발급 분기가 돌지 않던 것을 고침, 2026-10-13). 계속 거절하는 로봇에 tick 마다 다시 보내지 않으려는 대기다.

운영자 서비스의 `reason_code` 는 C-09 공통 코드다(OPS-58). 임무 관문 사유는 노드 로그에 남긴다. REQUEST_RECHECK 의 세 줄은 C-11 10행이고, 나머지는 작전 승인(2026-10-11)이다.

| 임무 관문 사유 | `reason_code` |
| --- | --- |
| `MODE_NOT_DISASTER` | `NOT_AUTHORIZED` |
| `UNKNOWN_TARGET`, `NO_HOME_POSITION` | `NOT_FOUND` |
| `TARGET_NOT_CONFIRMED`, `NO_ROUTE`, `NO_VERDICT` | `PENDING_EVIDENCE` |
| `ALREADY_DISPATCHED`, `NOT_AWAITING_APPROVAL`, `RECHECK_ALREADY_REQUESTED`, `ALREADY_RETURNING`, `SEARCH_ALREADY_STARTED` | `TASK_BUSY` |
| `TARGET_REVISION_STALE`, `ROUTE_REVISION_STALE`, `SEGMENT_INDEX_MISMATCH`, `NOT_DISPATCHED_TARGET`, `TARGET_ALREADY_JUDGED` | `INVALID_INPUT` |
| `HOME_BASIS_MISMATCH` | `FRAME_REVISION_MISMATCH` |
| 처리하지 않는 operation(`FINISH_MISSION`·`RESET_STOP`·`SET_CAMERA_WATCH`), 필수 필드 누락, 역할과 다른 `robot_id` | `INVALID_INPUT` |
| SCOUT 상태를 모르거나 준비 전의 `START_SEARCH`(노드 경계 판정, 로그 사유 `SCOUT_NOT_READY`) | `NAV_NOT_READY` |

파라미터는 모두 필수이고 기본값이 없다. 하나라도 없으면 `missing parameters` FATAL 로 exit 2 다. 기동할 때 git SHA·boot ID·적용 파라미터·역할을 로그에 남긴다. 종료는 `rclpy.try_shutdown()`.

| 파라미터 | 타입 | 뜻 |
| --- | --- | --- |
| `profile_path` | string | 배포 프로파일 v2(역할·수색 구역) |
| `entry_yaws` | double[] | 수색 구역 순서대로 도착 방향(rad) |
| `policy_version`, `executor_policy_version`, `mission_frame_id`, `authority_epoch`, `coordinator_id`, `envelope_schema_version` | string | `SearchPolicy`·goal·envelope 값 |
| `executable_bases` | string[] | 실행해도 되는 구역 근거 종류 |
| `task_duration_s`, `status_max_age_s`, `tick_period_s`, `report_retry_s` | double | 임무 기한·상태 나이 상한·판단 주기·보고 재전송 주기 |
| `lease_duration_s`, `lease_renew_period_s` | double | 실행기 정책의 lease 길이와 같은 값, 갱신 주기. 주기가 길이의 절반을 넘으면 기동 거부(exit 2) |
| `lease_message_ttl_s` | double | `CommandLease.valid_until_ns` = 발급 시각 + 이 값. 그 갱신 메시지의 수신 만료다(AST88-02). 실행기는 시간 기준이 불명이면 갱신을 버리므로 Q-06(chrony) 측정이 이동의 선행 조건이다 |
| `admission_timeout_s` | double | 수락 시한. 이때까지 TaskAdmission 이 없는 task 는 lease 갱신을 영구히 멈추고, 시한 뒤 도착한 RobotStatus 가 STOPPED·다른 task 일 때 예약을 풀어 다시 발급한다(작전 2026-10-11) |
| `route_segment_goals` | double[] | 구간 끝 `[x, y, yaw]` × N (mission frame) |
| `route_revision` | int | 그 목록의 revision. 목록이 바뀌면 올린다(이전 승인은 쓰지 않음) |
| `route_segment_passage_free` | bool[] | 구간마다 "통로 판정 불필요"(현장 확인값, AST88-01·OPS-60). true 가 아니거나 길이가 구간 수와 다르면 그 구간의 MOVE_SEGMENT 를 내지 않고 `PASSAGE_ASSESSMENT_REQUIRED` 로 보류한다(SystemState.reason·로그, Q-25 기본값). 모두 보류하려면 모두 false 로 준다. ROS 파라미터로는 빈 배열을 넣을 수 없다(`-p`·`--params-file` 모두 값 없음으로 처리돼 기동 거절, 통합-로봇 측정 2026-10-13) |
| `camera_watch_roi` | double[] | 감시 ROI `[x0, y0, x1, y1]`(정규화). 운영자 요청에 ROI 가 없을 때만 쓴다. 넘기지 않아도 된다(그때는 요청 ROI 만. ROS 파라미터로는 빈 배열을 넣을 수 없으므로 "비움"은 "넘기지 않음"이다). 전부 0 은 받지 않는다. ROI 출처는 #76·Q-13 미정 |
| `camera_watch_timeout_s` | double | SetCameraWatch 요청의 `deadline_ns` = 발급 + 이 값. 그때까지 답이 없으면 그 요청을 버리고 재무장 필요로 둔다 |
| `home_poses` | double[] | 복귀 위치 `[x, y, yaw]` 를 SCOUT(TB-6, 도킹 접근 지점), SUPPORT(TB-5, 출구) 순으로(임무 frame, `goal_map_*` 기준). launch 가 프로파일에서 넣는다. 넘기지 않으면 복귀 위치 없음 → `NO_HOME_POSITION`. 길이가 6 이 아니면 기동 거절(exit 2) |
| `support_dispatch` | string | `AUTO`·`OPERATOR`. AUTO(저장소 소유자 결정 2026-10-13, 1B)면 확정·저장된 대상에 TB-5 를 승인 없이 `APPROACH_TARGET` 으로 한 번 보낸다. 운영자 `DISPATCH_SUPPORT` 는 `NOT_AUTHORIZED`(로그 `AUTO_DISPATCH_ENABLED`). OPERATOR 면 기존 DISPATCH_SUPPORT → MOVE_SEGMENT |
| `approach_standoff_m` | double | 대상 위치에서 TB-5 의 지금 위치 쪽으로 이만큼 떨어진 접근 지점(yaw 는 대상을 바라봄). 이미 그 안이면 제자리에서 돌기만 한다 |
| `support_pose_max_age_s` | double | (TB-5 상태가 READY·fresh 가 아니면 먼저 `SUPPORT_NOT_READY` 로 보류한다. 로봇은 arm 돼야 READY 다) TB-5 RobotPose(`/rescue/robots/{SUPPORT}/pose`)의 최대 나이. 더 오래됐거나 VALID 가 아니면 `SUPPORT_POSE_UNKNOWN`, 지도 기준·mission frame 이 다르면 `SUPPORT_POSE_BASIS_MISMATCH` 로 보류 |
| `support_exit_wait_s` | double | TB-5 가 접근을 마친(COMPLETED·정지 확인) 뒤 출구로 출발하기까지 기다리는 시간(Q1-b). 그동안 `SystemState.reason=SUPPORT_EXIT_IN_<n>S` |
| `search_start` | string | `AUTO`(지진 감지 DISASTER 가 곧 수색 시작, 운영자 START_SEARCH 는 `NOT_AUTHORIZED`/`AUTO_SEARCH_ENABLED`) 또는 `OPERATOR`(버튼, #85). 관문(TB-6 READY·lease·시계·정지·재난 모드)은 같다. 정지·재난 해제로 철회되고 TB-6 이 다시 준비되면 다시 시작한다. RViz 목표 선택은 AUTO 에서 첫 구역이 나가기 전까지 열려 있고, 없으면 `SEARCH_GOAL_NOT_SELECTED`. 현장 `AUTO` [근거: 저장소 소유자 결정 2026-10-13 저녁, 작전 OPS-136] |
| `search_mode` | string | `PATROL`(현장 파일 순찰 지점) 또는 `AREAS`(프로파일 search_areas). 둘 중 하나만. 현장 지금 `AREAS`(RViz 목표) [근거: 작전 OPS-139, 순찰 방식은 재범 확인 대기] |
| `patrol_points` | double[] | 생략 가능. TB-6 출발 자세 기준 [x, y, yaw]*N. `PATROL` 이면 필요하고 `executable_bases` 에 `PATROL_POINT` 가 있어야 한다 |
| `patrol_rounds` | int | 순찰 바퀴 수(1 이상). 다 돌면 `PATROL_EXHAUSTED` [근거: 작전 OPS-139] |
| `traffic_control` | string | `TB5_PRIORITY`(교통 관제, TB-5 먼저) 또는 `DISABLED`(로봇 없는 시험 고정값; `OFF` 는 ROS 명령줄에서 YAML 참거짓이 돼 쓰지 않음). 현장 launch 는 SLAM_FREEZE 에서 `DISABLED` 를 거절한다. `TB5_PRIORITY` 는 `search_mode=FRONTIER` 가 필요하다(비켜서기가 FRONTIER 자리) [근거: 저장소 소유자 결정 2026-10-14, OPS-204] |
| `traffic_min_separation_m` / `traffic_path_clearance_m` / `traffic_yield_max_m` / `traffic_stop_timeout_s` | double | 새 명령을 막는 두 로봇 중심 거리 / TB-6 이 TB-5 경로·TB-5 와 둘 거리(최소 간격 이상) / 가장 먼 비켜서기(격자 경로 길이) / 취소한 TB-6 정지 대기 상한. 현장 0.6 / 0.6 / 2.0 / 3.0 [추정, 실측 아님] |
| `completion_mode` | string | `EXIT_ARRIVAL`(TB-5 출구 도착 = 구출 완료, TB-6 재확인 없음) 또는 `RECHECK`(C-11). 현장 `EXIT_ARRIVAL` [근거: 저장소 소유자 결정 2026-10-13 밤, OPS-155 ②] |
| `support_start_pose` | double[] | 생략 가능. TB-5 시작점 [x, y, yaw](TB-6 출발 자세 기준) = 현장 파일 `robots.<TB-5>.initial_pose_xy_yaw`(현장 launch 가 넘김, 측정 전에는 없음). QR 출구가 없을 때의 출구 [OPS-155 ①·173] |
| `frontier_min_cells`·`frontier_reach_m`·`frontier_goal_max_m`·`frontier_blacklist_m` | int·double | `FRONTIER`: 가장 작은 frontier 덩어리(칸), 덩어리에서 목표 칸까지 최대 거리, TB-6 목표를 자르는 경로 길이, 시도한 frontier 근처 재선택 금지 반경. 현장 10·1.0·1.5·0.5 [제안: 통합-조정 2026-10-13, 미측정] |
| `frontier_issue_delay_s`·`exploration_empty_checks`·`exploration_timeout_s` | double·int·double | 목적지를 `~/task_inputs` 에 넣고 task 를 내기까지(현장 launch 가 ≥ `live_inputs.poll_period_s` 검사), 빈 지도 갱신 횟수, 탐색 시간 상한(작전 OPS-156 ①). 현장 2.0·3·600 [제안] |
| `exit_standoff_m` | double | TB-5 가 QR 출구 앞, 대상 쪽으로 멈추는 거리. 현장 0.5 [제안: 통합-조정 2026-10-13, 미측정] |
| `map_mode` | string | `SLAM_FREEZE`(탐색 SLAM → 대상 저장 뒤 지도 고정 → 고정 지도에서만 TB-5 출동) 또는 `SAVED_MAP`(launch `goal_map_*` 기준 하나). 현장 `SLAM_FREEZE` [근거: 저장소 소유자 결정 2026-10-13, scope Q-29] |
| `freeze_clearance_m` | double | 고정 지도에서 접근 지점·출구·도킹 접근 지점이 비어 있어야 하는 반지름. 로봇 `task.live.auto_target.goal_clear_radius_m` 과 같아야 한다(현장 launch 가 검사). 현장 0.2 |
| `map_freeze_timeout_s` | double | 고정 요청 뒤 응답·고정 지도·TB-6 MapState 가 맞기까지 기다리는 시간. 넘으면 `MAP_FREEZE_FAILED`. 현장 10.0 [제안: 통합-조정 2026-10-13, 미측정] |
| `recheck_max_clock_error_s` | double | C-11 8행: 후보와 재확인 관측의 시계 오차 합 상한. 넘으면 `CANDIDATE_TIME_BASIS_UNUSABLE`. 현장 제안 2.0 [근거: 측정 웹캠 PC 271 ~ 365 ms + TB-6 탐지 PC 385 ~ 465 ms + Pi, 2026-10-13] |
| `observe_goal` | double[] | 재관측 지점 `[x, y, yaw]` |
| `goal_map_session_id`, `goal_map_revision`, `goal_transform_revision` | string, int, int | 위 목표들의 지도 기준 |
| `merge_distance_m` | double | 대상 병합 거리(Q-21) |

소실 후보(`TARGET_DISAPPEARED_CANDIDATE`)는 재난 모드에서만 판단한다(#124). 활성 감시와 카메라·대상이 같고 시간 기준이 선언돼 있으면 `CANDIDATE_ACCEPTED`(대상별 최근 수용 후보로 기록), 아니면 `CANDIDATE_NOT_LINKED`·`CANDIDATE_TIME_BASIS_UNUSABLE` 이고, 어느 쪽이든 그 감시를 끝낸다(C-11 2·4행). 같은 watch_id 에 다른 event_id 는 `ID_CONTENT_CONFLICT`(3행). 재난 모드가 아니면 `CANDIDATE_NOT_DISASTER` 로 기록만 한다.

아직 하지 않은 것: `HomePosition` 파라미터와 `RETURN_HOME` 배정(Q-22), 수용 후보 → SCOUT `REOBSERVE_TARGET` 자동 예약(C-11 5행)·`REQUEST_RECHECK` 의 후보 연결(10행 ④)·판정 연결(#80·#32), 카메라 재시작 감지(CameraStatus 를 받지 않아서 재시작으로 끝난 감시를 모름), 진행 중 수색 구역의 선점. E3 확인: `tests/ros/e3_coordinator_probe.py`, `tests/ros/e3_camera_watch_probe.py`(웹캠의 `WatchSlot` 을 그대로 쓰는 감시 서버)(가짜 실행기 2대, 로봇 없음, `ROS_DOMAIN_ID` 10 이상·`LOCALHOST`·Discovery Server 해제).

#### 검증

`tests/test_coordination.py`는 후보 보호 거리·경로/footprint/불확실성 거절, 비가용 관측 로봇, 중복 예약, 관측 원 기한·유실·확인, 통로 UNKNOWN/lease를 확인한다. `tests/test_coordination_review.py`는 profile ID 주입, PENDING→정확한 실행기 수락→ACTIVE, 거절·지연·충돌·재전송·세대 재사용 차단, HELD와 UNKNOWN의 holder 신원, 큰 epoch에서 1 ns 만료/종료 경계, 공통 오류를 확인한다. 이 PR의 수락 receipt 시험은 직접 만든 DTO를 사용한다. 실제 executor 연결 통합 경로와 E3/E4는 별도 검증이다. `tests/test_coordination_mission.py`는 재난 전·START_SEARCH 전 SCOUT 수색 차단, 사건·명령 ID 멱등과 내용 충돌, 시각 기준 없는 사건 거절, 명령마다의 관문 사유, 정지 확인 없는 구간 종료 거절, 다음 구간 자동 승인 없음, 승인 키(대상·경로 revision·구간) 불일치 거절, 재확인 지시의 C-11 10행 순서(모르는 대상·판정 끝난 대상·열린 재관측)와 대상별 기록, 경로 교체 뒤 이전 revision 거절, 복귀 위치 없음·지도 불일치 거절, 시나리오 순서의 표시 단계를 확인한다. `tests/test_coordination_target_goal.py`는 경로 증거 Phase 1 규칙(승인 없는 MOVE_SEGMENT·관측 지점 없는 REOBSERVE·APPROACH 거절)을, `tests/test_coordination_ros_core.py`·`test_coordination_ros_targets.py`는 노드 코어(메시지 변환, START_SEARCH 뒤 SCOUT 발급, 운영자 서비스와 공통 코드, MOVE_SEGMENT·REOBSERVE 발급, 대상 병합, 보고 저장 확인, 저장 안 된 위치만 싣는 보고, 출동 승인 관문, route frame, SystemState)를 ROS 없이 확인한다.

나머지 시험 파일(2026-10-13 `ls tests/test_coordination*.py` 기준):

| 파일 | 확인하는 것 |
| --- | --- |
| `tests/test_coordination_ros_passage.py` | 구간마다 "통로 판정 불필요"가 아니면 MOVE_SEGMENT 를 보류(`PASSAGE_ASSESSMENT_REQUIRED`) |
| `tests/test_coordination_ros_snapshot.py` | TargetSnapshot: 대상마다 TargetEstimate 하나, `persisted_report_id` 는 지금 revision 의 저장된 보고만 |
| `tests/test_coordination_loader.py` | 공통 어휘(TASK_KINDS·ROBOT_ROLES)를 그대로 쓰는지, 프로파일 수색 구역 → MissionSite, 구역별 도착 방향 |
| `tests/test_coordination_search.py` | 역할 입력 수색·대기·예약(SearchCoordinator) |
| `tests/test_coordination_dispatch.py` | 출동 대상 선택(`select_dispatch_target`) |

로봇 없는 E3(ROS 필요, `scripts/run_tests.py` 는 모으지 않는다; `ROS_DOMAIN_ID` 10 이상·LOCALHOST·Discovery Server 해제):

| 파일 | 확인하는 것 |
| --- | --- |
| `tests/ros/e3_coordinator_probe.py` | 실제 조정기 + 가짜 실행기 2대: 모드 관문, START_SEARCH, lease, 보고·영수증, MOVE_SEGMENT, REOBSERVE 재발급 |
| `tests/ros/e3_report_gate_probe.py` (#151) | 실제 조정기 + 실제 report_node: PERSISTED → `persisted_report_id` → DISPATCH_SUPPORT 수락, 보고 크기 |
| `tests/ros/e3_target_locator_probe.py` (#142) | 가짜 관측 + TF → target_locator → 조정기 대상 |
| `tests/ros/e3_camera_watch_probe.py` (#148) | 실제 조정기 + 웹캠 `WatchSlot` 그대로: 재난 모드에서만, ROI 없으면 거절, 후보 → 해제 |
| `tests/ros/e3_rehearsal_abc.py` | A·B·C 전체(SAVED_MAP)와 지도 고정 F1 ~ F3(SLAM_FREEZE), 위 「지금 상태」 |

# rescue_coordination

임무 조정기(`mission_coordinator`)와 그 판단 코어가 있는 패키지다. 판단은 ROS 없이 시험할 수 있는 `CoordinatorCore`·`MissionCoordinator`·`SearchCoordinator` 가 맡는다. `ros_node.main()` 은 메시지만 주고받는다. 로봇을 움직이는 주체는 로봇 실행기이고, 조정기는 `cmd_vel`·Nav2 액션을 직접 호출하지 않는다.

로봇 이름은 TB-6(코드 값 `robot_a`, 탐색 담당 SCOUT)과 TB-5(`robot_b`, 구조대원 안내 담당 SUPPORT)로 쓴다. 시나리오 원문과 결정은 [scope.md](../../docs/requirements/scope.md) Q-29(2026-10-13 밤 시나리오 변경)에 있다. 시나리오를 한 표로 정리한 것도 같은 문서에 있다(#268). 현장 값은 [현장 파일](../../config/field/search_area.field.json) `coordinator` 절에 있다.

## 현재 상태 (2026-10-15, main `5d9304a`)

근거는 모두 `rescue_coordination/ros_node.py` 의 `CoordinatorCore` 메서드다(다른 파일은 경로를 적음). 「실기 NOT_RUN」은 로봇 없는 시험(단위 시험·E3 리허설)만 통과했다는 뜻이다.

| 기능 | 상태 | 근거 |
| --- | --- | --- |
| 재난 신호(웹캠 `DISASTER_SIGN`)로 재난 모드 전환 | 구현됨, 실기 NOT_RUN | `on_site_event` |
| 재난 신호로 TB-6 탐색 자동 시작(`search_start=AUTO`) | 구현됨, 실기 NOT_RUN | `tick`(SEARCH_AUTO_STARTED) |
| TB-6 지도 전체 탐색(`search_mode=FRONTIER`, 미탐색 경계를 차례로) | 구현됨, 실기 NOT_RUN | `_frontier_tick`, `_exploration_done`(EXPLORATION_COMPLETE·EXPLORATION_TIMEOUT) |
| 인형 위치 병합과 보고(`RescueReport` → 저장 확인 `PERSISTED`) | 구현됨, 실기 NOT_RUN | `on_target_location`, `reports_due`, `on_receipt` |
| QR 출구 여러 개 기록(QR id 별) | 구현됨, 실기 NOT_RUN | `on_exit_location` |
| 1차 지도 고정(인형 저장 + TB-6 정지 → R0+1) | 구현됨, 실기 NOT_RUN | `_freeze_tick`, `_try_freeze`, `on_frozen_revision`(#242·#248) |
| TB-5 자동 출동(인형 앞 접근점, `APPROACH_TARGET`) | 구현됨, 실기 NOT_RUN | `_issue_approach`, `_frozen_gate` |
| 2차 지도 고정(탐색 완료 + TB-6 정지 → R0+2, 인형·출구 재기준) | 구현됨, 실기 NOT_RUN | `_freeze_tick`, `_rebase_on_freeze` |
| TB-5 출구 선택(최종 지도의 격자 경로 길이가 가장 짧은 출구, QR 출구가 없으면 TB-5 출발 자리) | 구현됨, 실기 NOT_RUN | `_final_exit_ready`(EXIT_CHOSEN, SUPPORT_START_UNKNOWN, SUPPORT_EXIT_UNREACHABLE) |
| 복귀 목적지 전달(`~/task_inputs` 의 `home/<robot_id>`) | 구현됨, 실기 NOT_RUN | `live_destinations`, `live_task_inputs.py` `destinations`(#232) |
| TB-5 출구 이동(`RETURN_HOME`) | 구현됨, 실기 NOT_RUN | `_issue_support_exit` |
| TB-6 도킹 접근점 복귀(`RETURN_HOME`, 최종 지도 뒤. 도킹은 사람이 함) | 구현됨, 실기 NOT_RUN | `_issue_scout_return`, `_home_free`(MAP_NOT_FINAL) |
| 구출 완료 = TB-5 출구 도착(`completion_mode=EXIT_ARRIVAL`, TB-6 재확인 없음) | 구현됨, 실기 NOT_RUN | `_confirm_by_exit_arrival` |
| 탐색을 끝냈는데 인형이 없으면 TB-5 출동 없이 종료(NO_SURVIVOR_FOUND) | 구현됨, 실기 NOT_RUN | `_no_survivor`, `mission_reason` |
| 두 로봇 도착 뒤 임무 종료(`MISSION_FINISHED`) | 구현됨, 실기 NOT_RUN | `_returned`, `mission.py` `record_returned` |
| 웹캠 감시 지정(`SET_CAMERA_WATCH` → `SetCameraWatch`) | 구현됨(#148), 실기 NOT_RUN | `on_command`, `_watch_tick`, `watch.py` |
| 웹캠 소실 후보 → TB-6 재확인 → 구출 판정(`completion_mode=RECHECK`) | 구현됨. 현장 설정(`EXIT_ARRIVAL`)에서는 쓰지 않음 | `_open_camera_recheck`, `_judge`, `verdict.py` |
| RViz 수색 목표 고르기 | 구현됨. `search_mode=AREAS` 에서만 켜짐(현장은 FRONTIER) | `live_task_inputs.py` `goal_pick_topic` |
| 로봇 명령 임대 갱신(`CommandLease`)과 미수락 임무 만료 | 구현됨 | `leases_due`, `_expire_unadmitted` |
| 교통 관제 1단계(TB-5 우선, TB-6 비켜서기와 임무 취소, 최소 간격, `traffic_control=TB5_PRIORITY`) | 구현됨(#272), 실기 NOT_RUN | `_traffic_support`, `_traffic_scout`, `_step_aside`, `_stop_scout`, `take_cancels` |
| 조정기 상태 영속(재기동 뒤 복구) | 미구현. 상태는 메모리에만 있음 | `ros_node.py` 에서 파일·DB 쓰기를 찾지 못함(grep `sqlite`·`open(`) |

로봇 실행기도 이제 SEARCH_AREA 밖의 임무를 받는다. 현장 파일의 `robots.robot_a.task.live.auto_target.kinds` 는 `REOBSERVE_TARGET`·`RETURN_HOME`, `robots.robot_b` 는 `APPROACH_TARGET`·`RETURN_HOME` 이다 [설정 2026-10-14]. 실행기 쪽 근거는 `rescue_robot/task_site_runtime.py` `_allowed` 다.

## 임무 흐름 (현장 설정, 코드 기준)

현장 설정은 지도 고정(`map_mode=SLAM_FREEZE`), 전체 탐색(`search_mode=FRONTIER`), 출구 도착 완료(`completion_mode=EXIT_ARRIVAL`)다.

| 단계 | 조정기가 하는 일 | 다음 단계로 가는 조건 | 막힐 때 보이는 이유 코드(`SystemState.reason`) |
| --- | --- | --- | --- |
| 1. 재난 신호 | 재난 모드로 바꾸고 TB-6 탐색을 허가 | 시각 기준이 선언된 `DISASTER_SIGN` | — |
| 2. 탐색 | 미탐색 경계 하나를 고르고 `~/task_inputs` 에 먼저 올린 뒤 `SEARCH_AREA` 발급 | 인형 확정 또는 탐색 완료 | `EXPLORATION_MAP_UNKNOWN`, `SCOUT_POSE_UNKNOWN` |
| 3. 인형 확정 | 보고를 보내고 저장 확인을 기다림 | `PERSISTED` | — |
| 4. 1차 고정 | TB-6 이 멈추면 고정 요청. 지도 revision 은 R0+1 | 고정 지도 hash 일치 | `MAP_FREEZE_WAITING_SCOUT_STOP`, `MAP_FREEZE_REQUESTED`, `MAP_FREEZE_FAILED` |
| 5. TB-5 출동·대기 | R0+1 에서 인형 앞 접근점으로 보냄. 도착 뒤 대기. TB-6 은 R0 에서 탐색을 계속함 | 탐색 완료 | `SUPPORT_NOT_LOCALIZED`, `SUPPORT_NOT_READY`, `SUPPORT_POSE_UNKNOWN`, `SUPPORT_APPROACH_NOT_FREE`, `MAP_NOT_FINAL` |
| 6. 2차 고정 | TB-6 이 멈추면 고정 요청. R0+2 로 인형·출구를 재기준(새 보고 없음) | 고정 지도 hash 일치 | 4단계와 같음 |
| 7. 출구 | TB-5 는 가장 가까운 출구로, TB-6 은 도킹 접근점으로 | TB-5 출구 도착 = 구출 완료 | `SUPPORT_EXIT_UNREACHABLE`, `SUPPORT_START_UNKNOWN`, `SCOUT_HOME_NOT_FREE`, `START_POSE_UNKNOWN` |
| 8. 종료 | 두 로봇이 모두 도착하면 `MISSION_FINISHED` | — | 인형이 없으면 `NO_SURVIVOR_FOUND` 로 종료 |

5단계부터는 교통 관제가 함께 돈다. TB-5 가 늘 먼저 가고, TB-6 은 TB-5 경로에서 비키거나 임무를 취소당한다. 이때 보이는 `TRAFFIC_*` 이유 코드는 [상태 전이](../../docs/design/state-machines.md) 「교통 관제 이유 코드」 표에 있다.

## 아직 없는 것

- TB-5 출발 자리(현장 파일 `robots.robot_b.initial_pose_xy_yaw`)는 2026-10-15 줄자로 잰 `[0.0, -1.86, 0.0]` 이다. 그래서 조정기가 `support_start_pose` 로 받아 두 로봇 간격을 검사한다. 이 값이 없는 현장 파일이면 TB-6 이 간격 검사 없이 움직이고, `SystemState.reason` 에 `TRAFFIC_SEPARATION_UNCHECKED` 가 보인다. 교통 관제 값(최소 간격 0.6 m, 경로와 둘 거리 0.6 m, 비켜서기 최대 2.0 m, 정지 대기 3.0 s)도 실측이 아니다 [추정 2026-10-14].
- 터널 판정(C-04b)을 탐색 경계 선택이나 TB-5 경로에 반영하는 기능은 아직 없다.
- 조정기를 재기동하면 발급 장부·대상·보고·고정 단계가 사라진다(영속 복구 없음).
- `ros_node.py` 가 약 2,800줄이다. 통합이 안정되면 기능별(지도 고정, 탐색, 출구, 교통 관제) 모듈로 나눌 후보다.

토픽은 [구현 기록](../../docs/implementation/implementation-coordination.md)「ROS 연결」, 파라미터와 현장 값은 「파라미터」, `reason_code` 대응표는 「운영자 명령 거절 코드」 절에 있다.

## 현장 연동 세부 (#132, 2026-10-12, 지금도 유효)

아래 세 기능은 지금 코드에도 있다. RViz 목표 고르기는 2026-10-13 밤부터 `search_mode=AREAS` 에서만 켜진다(현장 파일은 FRONTIER).

### 늦게 접속한 로봇의 세션 등록 (2026-10-12 현장 오류)

[현장 관측] 중앙 NORMAL/IDLE 기동 후 로봇이 나중에 접속하면 초기 시계 미준비로
SystemState를 거절하고, 시계 공급이 준비된 뒤에도 새 상태 발행이 없어 세션 등록이
대기한다. 저장 snapshot의 원 발행 나이 검사는 유지한다.

live_inputs를 켠 기존 중앙에 `~/refresh_state` 표준 std_srvs/Trigger 서비스를
추가했다. 운영자가 요청하면 중앙이 현재 상태를 실제로 다시 발행한다. 상태가
변하지 않았다면 state_sequence/changed_at_ns는 원 값이고 Envelope의 publication
sequence/published_at_ns만 실제 새 발행에 맞춰 변경한다.
2026-10-15 부터는 상태가 그대로여도 `lease_renew_period_s` 마다 같은 방식으로 다시 낸다.
조정기보다 늦게 뜬 실행기가 손으로 `~/refresh_state` 를 부르지 않아도 세션을 맺게 하려는 것이다.
이것도 중앙 clock 이 SYNCED 일 때만 하고, 노드 로그 `SYSTEM_STATE_RESENT` 는 상태마다 한 번만 남긴다.
서비스는 임무 tick/lease 갱신/arm/재난 모드 전환/Nav2 전송을 호출하지 않는다.
중앙 실측 clock이 UNKNOWN/만료면 거절한다. 로봇의 실제 clock/신원/boot/나이 검사와
DISARMED 및 watchdog 관문은 유지한다. 새 노드나 IDL은 추가하지 않았다.

### RViz에서 SEARCH_AREA 목적지 선택 (2026-10-12)

[사용자 요구, 2026-10-12] “아 아니다 고르는 식으로 해줘.”
`live_inputs.search_goal_topic`에 전용 절대 토픽을 지정하면 기존 중앙 노드가
PoseStamped를 받는다. RViz 기본 `rviz_default_plugins/SetGoal` 도구의 Topic을
그 이름으로 바꾸고 mission frame에서 위치/방향을 드래그한다. 기존 Nav2 Goal
도구를 선택 입력으로 사용하지 않는다. 이 토픽은 Nav2에 연결하지 않는다.

선택 모드에서는 프로파일의 첫 구역 ID/지도 기준만 유지하며 예제 좌표는 공급하지
않는다. 선택 후 구역 하나만 기존 service와 SearchPolicy에 넣는다. 선택 기록은
메모리에만 보관한다. 선택만으로 mode 변경/START_SEARCH/arm/Nav2 전송은 하지 않는다.
START_SEARCH 전 재선택은 허용하고, 수색 승인 후 또는 pending/active 임무 중에는
거절한다. 선택 전 START_SEARCH는 PENDING_EVIDENCE로 거절한다. frame 불일치,
비평면/비유한 좌표, 영 quaternion도 거절한다. 재기동하면 새 선택이 필요하다.
지도/AMCL/TF 정합과 lease/watchdog 준비는 로봇의 기존 관문이 계속 확인한다.
목적점을 클릭한 것으로 지도 기준 검증이나 안전한 통로를 입증했다고 주장하지 않는다.

### 2026-10-12 현장 수정: 로봇에 직접 공급하는 입력

기존 mission_coordinator에서 `live_inputs.enabled=true`로 켜면 `~/task_inputs`
표준 GetParameters 서비스를 추가한다. 새 노드/IDL은 없다. 로봇의 LIVE_ROS가 이
서비스의 names=['source']를 비동기 조회한다. STRING 값은 내부 직렬화 기록이며
사용자가 만들거나 전달하는 JSON 파일이 아니다.

다음은 모두 명시적 ROS 파라미터다:
`live_inputs.clock_domain_id`(선언된 공통 시간 기준),
`live_inputs.clock_backend`(chrony/timesyncd), `live_inputs.clock_max_age_ns`(원 NTP 관측 최대 나이),
`live_inputs.clock_drift_ppm`(선정된 oscillator 오차 예산), `live_inputs.poll_period_s`(OS 읽기 주기).
이들 값과 기존 lease/임무 파라미터는 운영 정책이며 자동 예시값을 주지 않는다.

수색 목적점은 `core.search.policy.areas`에서 생성하고 실제 SCOUT ID·지도 기준·방향을
보존한다. 시계는 같은 PC의 OS에서 실측한다. robot이 응답의 coordinator ID/boot,
authority/schema/domain, lease 길이·갱신 주기를 자신의 설정과 대조한다. SystemState,
ExecuteTask, CommandLease의 IDL·토픽·의미를 변경하지 않았다. service의 clock 관측 시각과
만료는 원 NTP reply/reference 시각에 묶이며 조회로 연장되지 않는다.

중앙 설정 자체의 기존 profile_path와 실행 파라미터는 유지한다. 이 수정은 로봇에
중앙 설정 파일을 복사하는 절차를 없애며, 중앙의 프로파일 계약을 임의로 바꾸지 않는다.

Python 회귀 확인만 실행했다. 실제 ROS 서비스/DDS·두 PC 시계·로봇 주행은 NOT_RUN이다.


## 정책 코어

경로 증거 기반 후보 선택, 중복 임무 배정 방지, 대상 최신성/접근 확인, 통로 점유 상태의 Python 코어다. 코어 자체는 로봇을 움직이거나 실제 경로를 계산하지 않는다.

API와 시험용 fixture는 [`tests/test_coordination.py`](../../tests/test_coordination.py), 출처·계약·미구현은 [구현 기록](../../docs/implementation/implementation-coordination.md)을 참고한다. 외부 증거를 생성하는 어댑터와 지도 자동 접근 허가가 연결되기 전에는 물리 로봇 실행 계층으로 사용하지 않는다.

프로파일의 두 ID를 `Allocator(..., allowed_robot_ids=...)`에 주입한다. DTO는 공통 ID 문법만 검사한다. `issue_generation`은 발급만 하고, `reserve`는 PENDING 예약만 만든다. 실행기에서 받은 공통 `TaskAdmission`을 `record_admission`에 넣어 정확한 명령·로봇·boot·authority·세대가 일치한 수락을 확인해야 ACTIVE가 된다. 발급 카운터와 실행기가 수락한 카운터를 별도로 기록한다.

## 기록 (2026-10-14 옮김)

아래는 예전 상태 설명이다. 지금과 맞지 않는 부분이 있어 맨 위 「현재 상태」로 대체했다. 이력 확인용으로만 남긴다.

### 2026-10-13 기준 「지금 하는 일」·「지금 안 되는 것」 (지금은 아님)

2026-10-14 에 다시 확인한 차이는 아래와 같다.
- 「`RETURN_HOME` 은 늘 거절된다」는 지금 틀리다. `home/<robot_id>` 목적지(#232)와 두 로봇 복귀 뒤 `MISSION_FINISHED` 경로가 있다.
- 「웹캠 감시 지정은 #148 머지 뒤에 생긴다」는 지금 틀리다. #148 이 머지됐다.
- 「실제 로봇은 SEARCH_AREA 만 받는다」는 지금 틀리다. 현장 파일의 `auto_target.kinds` 가 다른 종류를 연다.
- 「소실 후보 → 재관측 → 구출 판정 연결은 없다」는 지금 틀리다. `completion_mode=RECHECK` 에서 연결돼 있고, 현장 설정은 이 흐름을 쓰지 않는다.
- 「운영자 `START_SEARCH` 뒤 SCOUT 에 `SEARCH_AREA`」, 「`DISPATCH_SUPPORT`·`CONTINUE_SEGMENT` 승인마다 `MOVE_SEGMENT`」는 운영자 방식(`search_start=OPERATOR`, `support_dispatch=OPERATOR`)의 설명이다. 현장 설정은 둘 다 AUTO 다.

#### 지금 하는 일 (2026-10-13)

- **임무 상태**: 웹캠 `DISASTER_SIGN`(시각 기준이 선언된 것만)으로 NORMAL → DISASTER 가 된다. 바뀔 때마다 `SystemState` 를 낸다.
- **로봇에 task 발급**(`ExecuteTask`) 과 `CommandLease` 갱신:
  - 운영자 `START_SEARCH` 뒤 SCOUT 에 `SEARCH_AREA`.
  - `DISPATCH_SUPPORT`·`CONTINUE_SEGMENT` 승인마다 SUPPORT 에 `MOVE_SEGMENT`.
  - `REQUEST_RECHECK` 마다 SCOUT 에 `REOBSERVE_TARGET`.
  - ACTIVE 는 실행기의 `TaskAdmission` 으로만 되고, 예약은 정지 확인이 있어야 푼다.
- **대상과 보고**: `TargetLocation`(VALID 만)을 위치로 병합해 대상을 만든다. 바뀔 때 `TargetSnapshot` 을 내고, 대상 revision 마다 `RescueReport` 를 보낸다.
  - 보고에는 아직 저장 확인이 안 된 위치만 싣는다(#139).
  - report_node 의 `ReportReceipt` PERSISTED 가 와야 그 revision 의 `persisted_report_id` 가 생긴다. 그래야 AMR2 출동 승인(`DISPATCH_SUPPORT`)이 열린다.
- **운영자 명령 관문**: `/rescue/operator_command`(SubmitOperatorCommand). 같은 command_id 는 첫 응답 그대로(`DUPLICATE`)다.
- 확인 [측정 2026-10-13]: 로봇 없는 A·B 리허설에서 재난 신호부터 출동 승인 수락까지 7단계가 3회 모두 PASS 했다. 로봇 실행기·웹캠·AMR 카메라는 가짜였다. 자세한 것은 [구현 기록](../../docs/implementation/implementation-coordination.md) 「기록」 절의 「지금 상태」에 있다.

#### 지금 안 되는 것 (2026-10-13)

- 실제 로봇은 `SEARCH_AREA` 만 받는다. 실행기 LIVE_ROS 가 그 외 종류를 거절하므로(`rescue_robot/task_site_runtime.py:256`) AMR2 출동과 AMR1 재확인은 실기에서 아직 움직이지 않는다(C1 보류).
- `RETURN_HOME` 은 늘 거절된다. 복귀 위치가 없고, 로봇의 현재 지도 기준을 받는 곳도 없다.
- 웹캠 감시 지정(SetCameraWatch)은 PR #148 머지 뒤에 생긴다. 그 전에는 `SET_CAMERA_WATCH` 가 `INVALID_INPUT` 이다.
- 소실 후보 → 재관측 자동 예약 → 구출 판정 연결은 없다(C-11 5·7행, C4 보류).
- 상태는 메모리 안에만 있다. 조정기를 다시 띄우면 발급 장부·대상·보고가 사라진다(영속 복구 없음).

토픽은 [구현 기록](../../docs/implementation/implementation-coordination.md)「ROS 연결」, 파라미터는 「파라미터」, `reason_code` 대응표는 「운영자 명령 거절 코드」 절에 있다. 정책 코어(후보 선택·배정·통로 점유)의 옛 설명은 같은 문서 「기록」 절에 있다.

### 2026-10-08 기준 정책 코어 설명의 뒷부분 (일부는 지금 아님)

2026-10-14 차이: frontier 탐색은 지금 구현돼 있다(`_frontier_tick`). 아래의 「frontier 어댑터는 후속 범위」 문장은 2026-10-08 당시 설명이다.

외부 관측·기한·확인 시각은 정수 ns이며 `*_ns` 인자만 받는다. 로컬 수신 간격과 통로 lease는 monotonic 초다. `rescue_contracts`의 공통 오류/ID/시각/receipt 경계에 의존한다. 수색 구역 분할·frontier 어댑터·자동 역할 교대는 F-01 후속 범위이며 구현 완료로 집계하지 않는다.

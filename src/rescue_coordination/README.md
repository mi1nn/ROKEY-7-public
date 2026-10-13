# rescue_coordination

임무 조정기(`mission_coordinator`)와 그 판단 코어다. 판단은 ROS 없이 시험하는 `CoordinatorCore`·`MissionCoordinator`·`SearchCoordinator` 가 하고, `ros_node.main()` 은 메시지만 옮긴다. 로봇을 움직이는 것은 로봇 실행기다. 조정기는 `cmd_vel`·Nav2 액션을 부르지 않는다.

## 지금 하는 일 (2026-10-13)

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
- 확인 [측정 2026-10-13]: 로봇 없는 A·B 리허설에서 재난 신호부터 출동 승인 수락까지 7단계가 3회 모두 PASS 했다. 로봇 실행기·웹캠·AMR 카메라는 가짜였다. 자세한 것은 [구현 기록](../../docs/implementation/implementation-coordination.md)「지금 상태」에 있다.

## 지금 안 되는 것

- 실제 로봇은 `SEARCH_AREA` 만 받는다. 실행기 LIVE_ROS 가 그 외 종류를 거절하므로(`rescue_robot/task_site_runtime.py:256`) AMR2 출동과 AMR1 재확인은 실기에서 아직 움직이지 않는다(C1 보류).
- `RETURN_HOME` 은 늘 거절된다. 복귀 위치가 없고, 로봇의 현재 지도 기준을 받는 곳도 없다.
- 웹캠 감시 지정(SetCameraWatch)은 PR #148 머지 뒤에 생긴다. 그 전에는 `SET_CAMERA_WATCH` 가 `INVALID_INPUT` 이다.
- 소실 후보 → 재관측 자동 예약 → 구출 판정 연결은 없다(C-11 5·7행, C4 보류).
- 상태는 메모리 안에만 있다. 조정기를 다시 띄우면 발급 장부·대상·보고가 사라진다(영속 복구 없음).

토픽·파라미터·`reason_code` 대응표는 [구현 기록](../../docs/implementation/implementation-coordination.md)「임무 조정기 ROS 노드」에, 정책 코어(후보 선택·배정·통로 점유)는 같은 문서 앞 절에 있다.

## 현장 연동 (#132, 2026-10-12)

## 늦게 접속한 로봇의 세션 등록 (2026-10-12 현장 오류)

[현장 관측] 중앙 NORMAL/IDLE 기동 후 로봇이 나중에 접속하면 초기 시계 미준비로
SystemState를 거절하고, 시계 공급이 준비된 뒤에도 새 상태 발행이 없어 세션 등록이
대기한다. 저장 snapshot의 원 발행 나이 검사는 유지한다.

live_inputs를 켠 기존 중앙에 `~/refresh_state` 표준 std_srvs/Trigger 서비스를
추가했다. 운영자가 요청하면 중앙이 현재 상태를 실제로 다시 발행한다. 상태가
변하지 않았다면 state_sequence/changed_at_ns는 원 값이고 Envelope의 publication
sequence/published_at_ns만 실제 새 발행에 맞춰 변경한다. 반복 heartbeat는 아니다.
서비스는 임무 tick/lease 갱신/arm/재난 모드 전환/Nav2 전송을 호출하지 않는다.
중앙 실측 clock이 UNKNOWN/만료면 거절한다. 로봇의 실제 clock/신원/boot/나이 검사와
DISARMED 및 watchdog 관문은 유지한다. 새 노드나 IDL은 추가하지 않았다.

## RViz에서 SEARCH_AREA 목적지 선택 (2026-10-12)

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

## 2026-10-12 현장 수정: 로봇에 직접 공급하는 입력

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

외부 관측·기한·확인 시각은 정수 ns이며 `*_ns` 인자만 받는다. 로컬 수신 간격과 통로 lease는 monotonic 초다. `rescue_contracts`의 공통 오류/ID/시각/receipt 경계에 의존한다. 수색 구역 분할·frontier 어댑터·자동 역할 교대는 F-01 후속 범위이며 구현 완료로 집계하지 않는다.

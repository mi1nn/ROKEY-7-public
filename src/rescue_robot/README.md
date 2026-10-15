# rescue_robot

## 현재 상태 (2026-10-14, main 4163a25)

코드 기준이다. 「실기 NOT_RUN」은 코드와 로봇 없는 시험(`scripts/run_tests.py`)은 있지만 실제 로봇에서 확인하지 않았다는 뜻이다. 맨 아래 「기록」 절은 당시 설명이다. 이 표와 다르면 이 표가 지금 상태다.

| 기능 | 상태 | 근거 |
| --- | --- | --- |
| 로봇 쪽 실행(Nav2 + `command_watchdog` + `task_input` 한 번에) | 구현됨, 실기 NOT_RUN | `rescue_bringup/launch/robot.launch.py`, #152 |
| 임무 입력 LIVE_ROS `SEARCH_AREA` | 구현됨 | `task_site_runtime.py:_issuer_rejection`, `live_sources.py` |
| LIVE_ROS 자동 대상 이동: TB-5 `APPROACH_TARGET`, TB-6 `REOBSERVE_TARGET`, 두 로봇 `RETURN_HOME` | 구현됨, 실기 NOT_RUN | `task_site_runtime.py:_allowed`·`target_gate`, #166·#179·#192. 종류는 현장 파일 `robots.<id>.task.live.auto_target.kinds` |
| LIVE_ROS `MOVE_SEGMENT` | 미구현(`LIVE_SEARCH_ONLY` 로 거절, FILES 경로만) | `task_site_runtime.py:_issuer_rejection` |
| 재확인 관측(ObserveTarget) 호출 | 구현됨. 서버는 통합-인식 몫 | `live_recheck.py:RecheckClient` |
| 자동 도킹 | 미구현(사람이 감독하는 절차) | 아래 「복귀」 |
| RobotPose·MapState 발행 | 구현됨 | `live_pose.py:LivePosePublisher` |
| TB-6 탐색 중 위치 = TF `map→base_link`, 공분산은 마지막 `/robot6/pose`(30 s 넘으면 무효). 두 TF 를 3D 로 합성한 뒤 지도 평면으로 내린다. 합성한 기울기가 10도(제안)를 넘으면 위치를 만들지 않는다 | 구현됨, 현장 TF 값으로 리허설 확인(#291), 실기 NOT_RUN | `slam_tf_pose.py:SlamTfPose`, `live_sources.py:attach_slam_tf_pose`, #253 |
| 지도 두 번 고정(R0+1 인형 발견, R0+2 탐색 완료)·짝 토픽 `frozen_revision`·TB-5 재정합 | 구현됨, 실기 NOT_RUN | `map_freeze.py`, `live_sources.py:SavedMapSources`, `frozen_map_relay.py`, #242·#255 |
| TB-5 출발 정렬(`start_align`). 지금 launch 는 AMCL 을 잰 출발값에 퍼짐 0 으로 놓는다[Nav2 Jazzy `amcl_node.cpp:283-295`]. 그래서 서 있는 동안 잰 값의 오차를 고칠 수 없다. TB-5 가 언독하고 멈추면 이 노드가 AMCL 퍼짐을 다시 주고(`set_initial_pose` 서비스), 제자리 갱신(`request_nomotion_update`)을 여러 번 부른다. 퍼짐·안정·이동량 상한·스캔 일치 점수를 모두 만족하면 `ALIGNED`, 아니면 이유 코드(`START_ALIGN_*`)를 `<ns>/rescue_start_align` 에 낸다. 값은 현장 파일 `robots.robot_b.start_align.*`(모두 [제안]). TB-5 위치 유효 조건에 정렬 완료를 더하는 것은 다음 PR 이다 | 구현됨(노드와 판정), 실기 NOT_RUN | `start_align.py`, `launch/frozen_localization.launch.py` |
| 통로 정지 알림 `PASSAGE_BLOCKED` | 구현됨, 실기 NOT_RUN | `live_passage.py:PassageMonitor`, `task_site_runtime.py:feedback_reason`, #219 |
| watchdog 영구 정지·리셋, 끝날 때 `cmd_vel` 0 세 번 | 구현됨 | `watchdog.py`, `watchdog_node.py:zero_commands`, #256 |
| 임무가 끝난 뒤 다시 READY | 구현됨, 실기 NOT_RUN. 이동이 정상으로 끝나고(`COMPLETED`) 정지가 확인되면 스스로 다시 준비한다(`ARMED`). 준비 조건은 `~/task_arm` 과 같다. 조건이 맞지 않으면 다시 준비하지 않고, 로그와 `RobotStatus.stop_reason`(`REARM_REFUSED_<이유>`)에 이유를 남긴다. 실패·만료·취소·정지 버튼·정지 미확인 뒤에는 지금처럼 `~/task_arm` 을 기다린다. 로봇마다 `task.live.auto_rearm` 으로 켜고, 두 로봇 모두 켰다(재범 4등급 승인 2026-10-14, 이슈 #233 3번) | `executor.py:rearm_after_completion`, `nav2_adapter.py:_after`, `task_site_runtime.py:rearm_after_completion`, `task_status.py`, OPS-231 |
| watchdog 현장 값 | 임시값(#132), 확정 안 됨(#94) | `config/field/search_area.field.json` `watchdog` 절 |
| `command_adapter` 단독 실행 | 임무 입력이 없어 로봇을 움직이지 않는다. 현장은 `task_input` | `setup.py` console_scripts, `task_input_node.py:main` |
| 빈 배열 ROS 파라미터 | 넘길 수 없다(launch 가 거절) | launch_ros. 아래 「실행」 |

## 실행

**실행.** 로봇 쪽은 `rescue_bringup` 의 `robot.launch.py`(#152)로 띄운다. TurtleBot4 Nav2(최종 `cmd_vel` 을 watchdog 입력으로 remap) + `command_watchdog` + `task_input` 을 한 번에 띄운다. 값은 배포 프로파일, 현장 실행 파일 `config/field/search_area.field.json`(#132 값), 현장에서 잰 지도 해시(`scripts/field/map_occupancy_sha256.py`)에서 온다. 명령·순서는 [rescue_bringup README](../rescue_bringup/README.md) 「지금 현장 실행 순서」·「현장 실행 순서: PC 역할별」 절에 있다. #132 의 `scripts/field/search-area/start_*.sh` 는 같은 값의 원본으로 남아 있고, 시험(`tests/test_bringup_field_launch.py`)이 launch 값과 대조한다.

**실행 전 점검.** `scripts/field/preflight_role.py --role <host id>`(#156)가 이 PC 의 패키지(`nav2_msgs`·`turtlebot4_navigation`)·현장 파일·지도·시계·Discovery 설정을 미리 본다.

**안 되는 것**은 위 「현재 상태」 표의 미구현 줄이다. 그 밖에 ROS 파라미터로 빈 배열을 넘길 수 없어(launch_ros 거절, `-p`·`--params-file` 은 미설정) 빈 `camera_topics` 는 launch 가 거절한다.

**자동 대상 접근(APPROACH_TARGET, LIVE_ROS, 2026-10-13)**

TB-5(SUPPORT)는 조정기가 보낸 대상 접근을 운영자 승인 없이 받는다(재범 결정 1B·2B). 정책이 꺼져 있거나 없으면 지금까지처럼 `LIVE_SEARCH_ONLY` 로 거절한다.

| 파라미터(기본값 없음) | 뜻 |
| --- | --- |
| `task.live.auto_target.enabled` (bool) | false 면 자동 대상 이동을 모두 거절 |
| `task.live.auto_target.kinds` (string[]) | 받을 종류: `APPROACH_TARGET`·`REOBSERVE_TARGET`·`RETURN_HOME` 중(RETURN_HOME 은 권한만, 해석은 configured destination) |
| `task.live.auto_target.goal_clear_radius_m` (double) | 저장 지도에서 목표 중심 이 반경 안이 모두 비어 있어야 한다(현장 실행 파일 0.2 m [제안]) |
| `task.live.auto_target.policy_version` (string) | 결정 기록용 버전 |

현장 실행 파일에서는 이 넷을 로봇별(`robots.<id>`)로 둔다. 각 로봇은 자기 역할 임무만 받는다(최소 권한, 2026-10-13). TB-5(robot_b) = `APPROACH_TARGET`, TB-6(robot_a) = `REOBSERVE_TARGET` 만.

받는 조건(하나라도 아니면 거절):
- 목표 모양: 경로가 있고 운영자 승인이 없다. `route.approach_goal` 은 임무 frame 의 평면 자세다. 계획 증거(planned_path·path_sha256·planner_evidence_id)는 비어 있고, 통로 판정은 UNKNOWN 이다. 이 모양은 Phase 1 과 같다.
- 지도 기준: 목표의 `frame_ref`(mission frame·map session·map revision·transform revision)가 이 로봇의 현재 선언 정합과 같아야 한다. 지도는 하나다(프로파일 `map_basis`). 대상 좌표를 낸 TB-6 의 실시간 지도는 읽지 않는다(Discovery 구성상 다른 PC 에서 안 보일 수 있다).
- 경로 근거: 이 로봇이 받아 해시까지 맞춘 저장 지도에서 목표 발자국이 비어 있어야 한다(`Grid.clear`). 점유·미지(-1)·지도 밖이면 `UNREACHABLE`, 지도가 없으면 `LOCALIZATION_UNCERTAIN` 이다. 실제 경로는 Nav2 가 실시간 costmap 으로 계획한다. 그래서 지진 뒤 바뀐 장애물은 Nav2 가 본다. 계획이 안 되면 Nav2 ABORTED 로 끝나고 결과는 `UNREACHABLE` 이다.
- 수락 중 Nav2 `ComputePathToPose` 를 부르지 않는 이유: 수락은 실행기 잠금 안에서 동기로 돌고, 노드는 단일 스레드 executor 다. 수락 중에 액션 응답을 기다리면 그 응답을 처리할 스레드가 없어 멈춘다(교착).
- 실행 중에는 매 step 다시 확인한다. 저장 지도·정합이 바뀌거나 발자국이 막히면 취소하고, 새 명령이 필요하다. 운영자 승인 기록은 읽지도 만들지도 않는다. 정지·lease·watchdog 관문은 그대로다.

**재관측(REOBSERVE_TARGET)과 재확인 관측(ObserveTarget), TB-6(2026-10-13)**

구출 완료 판정(재범 결정 4A)은 웹캠의 출구 소실 후보에 TB-6 의 현장 재확인을 더한 것이다. TB-6 은 조정기가 보낸 REOBSERVE_TARGET 을 자기 정책(`robots.robot_a.task.live.auto_target.kinds` 에 `REOBSERVE_TARGET`)으로 받는다.
- 받는 조건: 자동 접근과 같다(Phase 1 모양, 지도 기준 일치, 관측 위치 발자국이 비어 있음). 더해서 대상이 조정기 TargetSnapshot(`/rescue/targets`)에 있고 revision 이 같아야 한다. Phase 1 보호 경로 기록은 요구하지 않는다(그 자리를 정책이 대신한다).
- 도착하고 정지가 확인(stop_confirmed)된 뒤에만 `/rescue/robots/<robot_id>/observe_target`(#165)에 관측을 묻는다. 볼 지점은 TargetSnapshot 의 대상 `mission_position` 을 native 좌표로 바꾼 값이다. 창은 `task.live.recheck.window_ns`, 호출 전체 마감은 `task.live.recheck.deadline_ns` 다(현장 실행 파일 3 s / 10 s [제안]).
- 서버의 답(OBSERVED·UNKNOWN·FAILED)은 그대로 `TaskResult.recheck` 에 싣는다(`has_recheck=true`). 묻지 못했으면 `has_recheck=false` 다(대상 위치 없음, 서버 없음, 마감까지 답 없음 → 취소 요청). 작업 결과(outcome)는 바꾸지 않는다.
- 호출은 ExecuteTask 의 `execute` 코루틴에서 `await` 한다(실행기 잠금 밖). 단일 스레드 executor 에서도 멈추지 않는다.

**복귀(RETURN_HOME), TB-5·TB-6(2026-10-13)**

재범 결정 5A: TB-6 은 도킹 접근 지점으로, TB-5 는 출구로 돌아간다. 복귀 위치는 배포 프로파일 한 곳(`home_positions: {robot_id: [x, y, yaw]}`, `map_basis` 기준)에 둔다. 조정기 launch 는 이것을 `home_poses`(SCOUT → SUPPORT)로 넘긴다. 둘 중 하나만 있으면 거절하고, 블록이 없으면 넘기지 않는다(조정기 `NO_HOME_POSITION`). 로봇 launch 는 자기 위치를 `task.live.home_xy_yaw` 로 넘긴다.
- 실행기는 이것을 `configured_destinations['home/<robot_id>']` 로 두고, 기존 RETURN_HOME 해석(지도 기준 일치·실행 중 계속 재확인)을 쓴다. 조정기는 `home_ref = "home/<robot_id>"` 로 보낸다.
- 로봇별 정책 `kinds` 에 `RETURN_HOME` 이 있을 때만 받는다(TB-6 `[REOBSERVE_TARGET, RETURN_HOME]`, TB-5 `[APPROACH_TARGET, RETURN_HOME]`). home 이 없으면 거절한다.
- 자동 도킹은 하지 않는다. TurtleBot4 Dock 액션은 Nav2·watchdog 속도 경로 밖에서 Create3 를 직접 움직여 watchdog 관문을 거치지 않는다. 도킹은 지금처럼 사람이 감독하는 절차로 한다.

**로봇 위치(RobotPose)와 지도 상태(MapState) 발행(2026-10-13)**

`task_input`(LIVE_ROS)은 자기 로봇의 위치와 지도 상태를 낸다. 조정기는 TB-5 위치로 자동 출동을 판단하고 지도 상태로 복귀 위치를 비교한다. db_writer 는 이 위치를 관제 지도에 그린다.

| 토픽 | QoS | 언제 | 내용 |
| --- | --- | --- | --- |
| `/rescue/robots/<robot_id>/pose` | RELIABLE·VOLATILE·1 (IF-010) | 받아들인 AMCL 자세마다 | `native_pose` = AMCL 원값. `mission_pose` 는 선언 정합으로 변환한 값이고, AMCL 유효·지도 적재·정합 유효·시계 SYNCED 가 모두 맞을 때만 `VALID`, 아니면 `UNKNOWN` 이며 숫자를 넣지 않는다. `time.valid_until_ns` = AMCL stamp + `task.live.pose_valid_ns`. 시각 오차 상한 = 이 PC 측정값 + 로봇 Pi 값(`task.live.sensor_host_clock_error_bound_ns`, 현장 실행 파일 `robots.<id>` 절, AMCL stamp 는 Pi 가 찍은 scan 시각). Pi 시계 영역(`task.live.sensor_host_clock_domain_id`)이 다르거나 Pi 값이 -1(측정 전)이면 `UNKNOWN`. 현장 값 150 ms: 2026-10-13 `chronyc tracking` 으로 `clock_measurement.parse_chrony` 식(차이 + root delay/2 + root dispersion + 갱신 간격 × 100 ppm)을 계산하면 TB-6 약 109 ms, TB-5 약 107 ms 다. 재부팅·재동기화 뒤 값이 바뀌므로 실기 당일 다시 잰다 |
| `/rescue/robots/<robot_id>/map_state` | RELIABLE·TRANSIENT_LOCAL·1 (IF-009) | 의미가 바뀔 때(지도 적재·위치 품질) | 선언된 map session·revision, `map_update_sequence` 0, `snapshot_id` = 지도 해시·기준의 UUIDv5, `localization_quality` VALID/UNKNOWN, `correction_detection_status` UNKNOWN. revision 은 고정할 때마다 R0 → R0+1 → R0+2 로 오른다(아래) |

**지도 수명 주기: TB-6 SLAM → 고정 지도(2026-10-13 저녁, scope Q-29 ⑦, OPS-113)**

기준 지도 없이 TB-6 SLAM 지도 frame 을 공통 frame 으로 쓴다. `task.saved_map.mode` 로 로봇마다 하나를 고르고, 모드 사이에 대체는 없다. 현장 실행 파일의 `robots.<id>.task.saved_map.mode` 로 고르고, 고정 노드·위치 추정 launch 는 rescue_bringup README 를 본다.

지도는 회차마다 두 번 고정한다(OPS-154): R0+1 = 인형 발견(TB-5 출동, TB-6 은 SLAM 으로 계속 탐색), R0+2 = 탐색 완료(TB-6 SLAM→AMCL, TB-5 는 인형 앞에서 정지한 채 재정합). `/rescue/map/frozen` grid 는 짝 `/rescue/map/frozen_revision`(해시가 같을 때만, `docs/design/contracts.md` C-04a)으로만 revision 을 안다.

| 모드 | 로봇 | 1차 고정 전 | 1차 고정(R0+1) 뒤 | 2차 고정(R0+2) 뒤 |
| --- | --- | --- | --- | --- |
| `SAVED_MAP_AMCL` | 로봇 없는 시험 | 저장 지도(측정 해시) + AMCL, 선언 revision | 고정 지도는 무시 | 같음 |
| `SLAM_THEN_FROZEN` | TB-6 | 바뀌는 SLAM 지도(frame 만 검사) + SLAM 위치 = TF `map→odom→base_link`(`task.saved_map.slam_tf_topic`, 실행기 poll 주기로 읽음, 시각은 두 edge 중 오래된 쪽) + 공분산은 마지막 `task.saved_map.slam_localization_topic`(이 값이 `slam_covariance_max_age_ns` 보다 오래되면 무효, OPS-177), revision R0, `snapshot_id` 의 해시 자리 = `slam`. 자동 이동(REOBSERVE·RETURN_HOME·APPROACH)은 빈 칸 검사용 지도가 없어 거절, SEARCH_AREA 만 R0 로 | 그대로(SLAM·R0) | 자기 `<ns>/map` 이 R0+2 grid 와 해시가 같을 때만 지도 적재, AMCL 위치만 인정(SLAM 위치 무시), revision R0+2. SLAM 으로 돌아가지 않음 |
| `FROZEN_MAP_AMCL` | TB-5 | 지도 없음, `UNKNOWN`, revision R0, 해시 자리 = `unfrozen` | R0+1 지도 + AMCL(launch 출발값) | 정지·작업 없음일 때 R0+2 로 재정합: `<ns>/map` 이 R0+2 가 되면 직전 자기 유효 AMCL 위치를 `<ns>/initialpose` 로 다시 넣음(`MAP_REALIGN`), 그 뒤 AMCL 위치만 인정 |

- 더 높은 revision 은 작업이 없고 odometry 가 `STOPPED` 일 때만 받는다(`MAP_REVISION_ADOPTED`). 아니면 `MAP_REVISION_DEFERRED <이유>` 를 남기고 R 을 유지했다가 멈추면 자동으로 받는다. 기다리는 동안 새 목표는 `FRAME_REVISION_MISMATCH` (`map revision pending`)로 거절하고, 진행 중 작업은 끝까지 간다(D2, OPS-159). HOLD 는 받는다.
- 받는 순간 MapState 는 새 revision·`UNKNOWN` 이 되고, `frozen_map_relay` 가 그 revision 의 grid 를 `<ns>/map` 으로 복사한다(자기 MapState revision 만 따름).
- 해시가 다른 지도가 `<ns>/map` 에 오면 적재하지 않고 오류 로그(`map is not the frozen map`). AMCL 공분산이 한도를 넘으면(미수렴) `UNKNOWN`.
- 같은 고정 지도가 다시 오면(고정 노드 재시작) 그대로, 이미 아는 revision 에 **다른** grid 가 오면 지도 차단(새 회차 필요). 낮은 revision 이 늦게 오면 쓰지 않는다. 이 회차가 아닌 revision(세션 다름, R0+3 이상)은 거절.
- TB-6 귀환 목적지(`task.live.home_xy_yaw`, 도킹 접근 지점)는 2차 고정 뒤에 쓰므로 revision R0+2 로 묶는다. TB-5 는 revision 이 R0+1→R0+2 로 바뀌므로 launch 귀환 위치가 없고, 출구는 조정기 `home/robot_b` 로 받는다.
- 두 고정 모드는 `task.saved_map.occupancy_sha256` 를 빈 문자열로 받는다(고정이 해시를 정함).

**watchdog 영구 정지(STOP_LATCHED)와 리셋**

로봇이 멈췄는데 Nav2 는 목표를 계속 보내고 있다면 watchdog 을 먼저 본다. 로그에 이 줄이 있으면 watchdog 이 세운 것이다(WARN):

```text
[<ns>.rescue_command_watchdog]: watchdog STOP_LATCHED reason=<이유> t=<초>
```

| 이유 | 뜻 | 먼저 볼 것 |
| --- | --- | --- |
| `EXECUTOR_HEARTBEAT_STALE` | `task_input` 의 heartbeat(`<ns>/rescue/executor_heartbeat`, `step_period` 마다 1개)가 `heartbeat_max_age_s` 넘게 안 옴 | `ros2 topic hz <ns>/rescue/executor_heartbeat -w 200` 의 max 간격. `task_input` 이 죽었거나 같은 스레드의 다른 콜백이 오래 걸린 경우 [추정] |
| `COMMAND_OUT_OF_LIMIT` | Nav2(또는 docking_server) 출력의 직진·회전 속도가 `max_linear_mps`·`max_angular_rps` 를 한 번이라도 넘음. 잘라 내지 않고 세운다 | `ros2 topic echo <ns>/cmd_vel_watchdog_in --field twist` 최대값과 Nav2 설정의 최대 속도 |
| `COMMAND_NOT_FINITE` | NaN·inf 속도 | Nav2 출력 |
| `CLOCK_REGRESSION` | watchdog 의 단조 시계가 뒤로 감 | — |

- 정지된 동안에는 들어오는 모든 명령이 0 이 되고 watchdog 자기 heartbeat 도 멈춘다. 그래서 `task_input` 의 `watchdog` 준비 flag 가 꺼지고 새 목표가 나가지 않는다.
- heartbeat 가 돌아와도 저절로 풀리지 않는다(C-06). 푸는 방법은 리셋 서비스 하나다. 리셋은 executor heartbeat 가 살아 있을 때만 성공한다:
  ```bash
  ros2 service call /<ns>/rescue_command_watchdog/reset std_srvs/srv/Trigger {}   # 예: /robot6/...
  # 응답 success=True, message='ARMED' 이면 풀림. False 면 heartbeat 가 없다(task_input 확인)
  ```
- 리셋은 로봇을 다시 움직일 수 있게 하는 조작이다. 원인을 확인하고 로봇이 멈춘 것을 눈으로 본 뒤 사람이 판단해서 한다. 실행기 쪽 STOP latch·새 generation 규칙은 따로다. 같은 임무를 이어 가는 대신, #132 규칙대로 조정기·로봇 launch 를 다시 켜고 새 목표를 고르는 것이 확인된 절차다.
- 입력 명령이 `command_max_age_s` 넘게 끊기면 0 을 한 번 보낼 뿐 래치하지 않는다(로그 없음). 이것으로는 영구 정지가 생기지 않는다.


## 실행기 코어 (`executor.py`)

명령 수락·단일 활성 임무·임대·취소·정지 확인의 순수 Python 코어(`executor.py`)와, 그 코어를 Nav2 `NavigateToPose` 에 잇는 명령 어댑터(`nav2_adapter.py`, ROS 경계 `ros_node.py`)다. SLAM 실행·언도킹은 하지 않는다.

공개 API는 `Policy`, `Readiness`, `Task`, `GoalPose`, `Executor`, `CommandAdapter`, `MissionGoalBinder`, `ReadinessPolicy`, `ReadinessMonitor`다. 인수와 시간 진행 예시는 [`tests/test_executor.py`](../../tests/test_executor.py)와 [`tests/test_nav2_adapter.py`](../../tests/test_nav2_adapter.py)를 참고한다. 모든 운용 임계값은 호출자가 명시해야 한다. 시험값은 실기 기본값이 아니다.

자세한 구현 경계·출처·남은 검증은 [구현 기록](../../docs/implementation/implementation-robot.md)에 있다. 현장에서 이 코어는 `task_input`(인증·lease·지도·정합 입력)과 `command_watchdog`(마지막 속도 경로) 뒤에서만 로봇에 닿는다(`robot.launch.py`).

## Nav2 명령 어댑터

`CommandAdapter`(ROS import 없음)는 로봇 하나의 `Executor` 와 그 로봇의 `NavigationPort`(`send_goal(command_id, goal_pose)` / `cancel_goal(command_id)`)를 묶고, 아래 사건을 서로 다른 입력으로 넘긴다.

| 사건 | 어댑터 입력 | 코어 호출 | 뜻하지 않는 것 |
| --- | --- | --- | --- |
| 임무 수락 | `submit(task)` | `admit` → `TaskAdmission` | — (협업·관제가 쓰는 유일한 수락 근거) |
| goal 전송 | `step()` 주기 호출 | `dispatch`/`tick` → `('SEND', id)` → `send_goal(id, task.goal_pose)` | 로봇이 움직였다는 증거 |
| Nav2 goal 수락 | `on_goal_response(id, accepted)` | `goal_response` | `TaskAdmission`(allocator 예약의 수락) |
| 취소 요청 | `request_stop(cause)`·만료·guard 실패 | `('CANCEL', id)` → `cancel_goal` | 정지 |
| 취소 응답 | `on_cancel_response(id, accepted)` | 없음(기록만) | 정지 |
| action 종료 | `on_terminal(id, GoalStatus 코드)` | `goal_terminal`(4/5/6 만) | 정지 |
| 정지 관측 | `on_odom(stamp, linear, angular)` | `observe_motion` | — (종료 fence 뒤 연속 표본만 `stop_confirmed`) |

- 이 어댑터가 보내지 않은 `command_id` 의 콜백(`FOREIGN_*`)은 실행기에 넘기지 않는다. 로봇마다 어댑터·포트·네임스페이스가 따로라서 한 로봇의 콜백이 다른 로봇의 취소를 만들지 않는다.
- `Readiness` 는 여섯 상태(`driver`, `sensors`, `slam`, `nav2`, `undocked`, `watchdog`)가 모두 들어와야 하고 가장 오래된 수신 시각을 쓴다. 하나라도 없으면 준비 안 됨이다.
- `guards(task) -> bool` 은 호출자가 주입하고 SEND 직전과 매 `step` 에 다시 본다(지도 revision 등). LIVE_ROS 에서는 `live_sources.SavedMapSources` 가 지도·정합을 준다.
- **goal frame**: `bind_goal` 을 주지 않으면 `task.goal_pose` 를 그대로 보낸다. 지도 guard(아래)를 쓰면 mission 좌표를 로봇 map 좌표로 바꿔 보낸다. 어느 경우든 `Task` 자체는 바꾸지 않는다.
- **odom 시각 [미확인]**: `sample_steady = received_steady − (wall_now − header_ns)/1e9`. 로봇과 PC 시계가 `clock_error_bound_s`(로컬 float 초; C-03 의 int `clock_error_bound_ns` 와 다름) 안에서 맞는다고 전제한다(측정 전). |차이| 가 상한을 넘거나 stamp 가 무효이면 표본으로 쓰지 않고 `Executor.invalidate_stop_evidence` 로 정지 창을 초기화한다 → 증거 부족은 `STOP_EVIDENCE_STALE`·`STOP_UNCONFIRMED` 로 드러난다.
- 두 로봇은 같은 어댑터를 쓰고 task kind 만 다르다(SCOUT: `SEARCH_AREA`·`REOBSERVE_TARGET`·`RETURN_HOME`, SUPPORT: `HOLD`·`APPROACH_TARGET`·`RETURN_HOME`. LIVE_ROS 에서 자동으로 받는 종류는 로봇별 `task.live.auto_target.kinds`). 역할은 설정에서 오고 코드에 두지 않는다. `HOLD` 는 goal 없이 정지 확인 뒤 `COMPLETED` 다.

## 지도 guard (`goal_binding.MissionGoalBinder`)

`goal_pose` 의 `map_session_id`·`map_revision`·`transform_revision` 은 이동하는 로봇 지도에서 그 goal 이 검증된 버전이다. binder 는 `MappingRegistry`(import 하지 않고 받음)의 `map_state`·`current_alignment`(읽기 전용)로 다음을 본다.

- `bind(task)`(dispatch 직전): 버전이 현재와 같으면 정렬의 역변환으로 mission → 로봇 map 좌표·방향을 만들어 SEND 에만 쓴다. 실패하면 아무것도 보내지 않고 guard 실패(`GUARD_INVALID`)다.
- `check(task)`(매 `step`): goal 의 frame 이 mission 이 아님, 버전 불일치(지도 보정·세션 교체·정렬 변경), 정렬 없음·만료·오차 초과 중 하나면 False → 취소 → 정지 관문. 주행 중 정렬이 바뀐 경우도 여기서 잡힌다(C-04).
- `REOBSERVE_TARGET`·`APPROACH_TARGET` 은 `target_gate(task)` 가 True 여야 한다. 현장 runtime 은 `task_site_runtime.target_gate` 로 판단한다. 로봇별 자동 대상 정책이 그 종류를 받으면 True 이고 매 step `_resolve_auto_target` 로 다시 확인한다. 정책이 없으면 Phase 1 보호 경로 관문을 통과한 `MOVE_SEGMENT`·`REOBSERVE_TARGET` 만 True 다. 정책 없는 `APPROACH_TARGET` 은 SEND 하지 않는다. `SEARCH_AREA` 는 지도 guard 만으로 출발하고, `HOLD` 는 지도와 무관하다.

## 준비 신호 (`readiness.ReadinessMonitor`)

출처별 마지막 **로컬 수신 시각**과 `ReadinessPolicy` 의 최대 나이로 `driver`·`sensors`·`slam`·`watchdog` flag 를 만든다. 수신 여부만 보고 센서 header 나이(인식 몫)나 SLAM 보정은 판단하지 않는다. 출처가 없는 flag 는 만들지 않으므로 그 flag 는 계속 준비 안 됨이다.

| flag | 출처 | 비고 |
| --- | --- | --- |
| `driver` | odom (+ `wheel_status`, 있으면) | |
| `sensors` | scan (+ 카메라 토픽들) | |
| `slam` | 네임스페이스 `tf` 의 `map_frame → odom_frame` 수신 | freshness 만 |
| `nav2` | action server ready | 기존 |
| `undocked` | dock status | 기존 |
| `watchdog` | 명령 watchdog(`command_watchdog`)의 heartbeat | `watchdog_heartbeat_topic` 이 비면 출처 없음 → 움직이지 않음. watchdog 이 `ARMED` 일 때만 발행. Create 3 의 cmd_vel timeout 은 검증된 증거가 아님 |

최대 나이 현장 값은 배포 프로파일 로봇별 `readiness.max_age_s`(저장소 밖)와 현장 파일 `robot` 절 `readiness.max_age_s.watchdog_heartbeat` 0.5 s 에서 온다(`rescue_bringup/field_launch.py`). 측정으로 정한 값은 아니다 [제안].

## ROS 노드 (`command_adapter`)

`ros_node.main` 은 기본값 없는 파라미터만 받는다. 프로파일(`config/deployment.*.json`)의 값은 bringup 이 넘긴다(이 패키지는 `rescue_bringup` 을 import 하지 않는다).

| 파라미터 | 출처 |
| --- | --- |
| `allowed_robot_ids`(문자열 배열), `robot_id`, `namespace` | 프로파일의 두 로봇 ID 와 해당 로봇 namespace. 노드 namespace 와 다르면 기동 거부 |
| `nav_action`, `odom_topic`, `dock_status_topic` | namespace 기준 이름. 프로파일 v2(#17)가 들어오면 그 값을 쓴다 |
| `authority_epoch`, `policy_version`, `policy.*`(7개), `step_period`, `clock_error_bound_s`(초) | 운용 정책. 현장 값은 현장 파일 `robot` 절(`policy_version` search-live-20261012, `clock_error_bound_s` 2.0 등). 측정 근거로 확정한 값은 아니다(T-01) |
| `scan_topic`, `tf_topic`, `map_frame`, `odom_frame` | 준비 신호 출처(필수) |
| `camera_topics`(문자열 배열), `wheel_status_topic` | 준비 신호 출처. 비워 둘 수 있음. `/compressed` 로 끝나는 카메라 토픽은 `CompressedImage`, 그 밖은 `Image` 로 구독(TB4 OAK-D 기본 발행이 compressed, P4 DEC-004) |
| `watchdog_heartbeat_topic` | 명령 watchdog 의 heartbeat(`std_msgs/Header`). 비워 둘 수 있으나 그러면 `watchdog` flag 가 없어 이동 불가 |
| `executor_heartbeat_topic` | 이 노드가 step 마다 발행하는 heartbeat(`std_msgs/Header`). 명령 watchdog 이 듣는다(필수) |
| `readiness.version`, `readiness.max_age_s.{odom,scan,camera,wheel_status,map_to_odom_tf,watchdog_heartbeat}`(초) | 준비 신호 정책. 출처는 위 「준비 신호」 [제안] |

`command_adapter`(`ros_node.main`)만 띄우면 임무 입력이 없어 로봇을 움직이지 않는다. 현장은 같은 노드에 임무 입력·지도·정합을 붙이는 `task_input`(`task_input_node.main` → `ros_node.main(task_runtime_factory=...)`)을 `robot.launch.py` 로 띄운다. 필요한 ROS 패키지: `rclpy`, `rcl_interfaces`, `nav_msgs`, `nav2_msgs`, `irobot_create_msgs`.

파라미터는 `declare_required` 로 타입만 선언한다(jazzy rclpy 는 값 없는 `descriptor=` 선언을 거부해 이전 코드는 기동하지 못했다). 빠진 값은 `missing parameters` 로 기동 거부한다.

## 명령 watchdog (`command_watchdog`, C-01·TR-11)

`watchdog.CommandWatchdog`(ROS import 없음)와 `watchdog_node`(ROS 경계). 로봇 하나의 마지막 속도 경로에 하나씩 둔다: launch 가 Nav2 의 최종 속도 출력을 `cmd_vel_in_topic` 으로 remap 하고, 로봇의 `cmd_vel` 은 이 노드만 쓴다.

| 상태 | 들어온 명령 | 자기 heartbeat | 나가는 조건 |
| --- | --- | --- | --- |
| `WAITING` | 0 으로 바꿔 발행 | 없음 | 첫 executor heartbeat → `ARMED` |
| `ARMED` | 한계 안이면 그대로 전달 | 발행 | executor heartbeat 만료·한계 초과·비유한값·시간 역행 → `STOP_LATCHED` |
| `STOP_LATCHED` | 0 으로 바꿔 발행(전이 때 0 한 번) | 없음 | `~/reset`(`std_srvs/Trigger`) + fresh heartbeat 만. heartbeat 회복만으로는 풀리지 않음(C-06) |

- 한계 초과 명령은 잘라 내지(clamp) 않고 latch 한다. `ARMED` 에서 이동 명령 뒤 입력이 끊기면 0 을 한 번 보낸다(latch 아님).
- watchdog 이 latch 되면 자기 heartbeat 가 멈춰 어댑터의 `watchdog` flag 가 만료되고 새 goal 이 나가지 않는다. 실행기의 STOP latch·새 generation 규칙은 따로 그대로다.
- 이것은 ROS 프로세스 안의 소프트웨어 watchdog 이다. 로봇에서 돌릴지 PC 에서 돌릴지, Create 3 timeout 과의 관계, 실제 정지 시간은 배포·측정 몫이다(T-05). 물리 안전 인증이 아니다.

| 파라미터 | 출처 |
| --- | --- |
| `namespace` | 노드 namespace 와 다르면 기동 거부 |
| `cmd_vel_in_topic`, `cmd_vel_out_topic`, `cmd_vel_stamped`(bool) | 마지막 속도 경로. 둘이 같으면 기동 거부. `Twist`/`TwistStamped` 선택은 설치 Nav2·Create 3 설정 확인 뒤 [미확인] |
| `executor_heartbeat_topic`, `watchdog_heartbeat_topic` | 어댑터 노드의 같은 이름 파라미터와 맞춘다 |
| `step_period`, `watchdog.version`, `watchdog.{heartbeat_max_age_s,command_max_age_s,max_linear_mps,max_angular_rps}` | 정책. 현장 임시값은 `config/field/search_area.field.json` 의 `watchdog` 절(#132: 0.05 s, 0.5 s, 0.5 s, 0.30 m/s, 1.20 rad/s). 확정은 #94·T-05 [미확인] |

## 검사

```bash
python3 -I scripts/run_tests.py                     # E2, ROS 없이 (tests/test_nav2_adapter.py 포함)
# E3 일부: ROS 2 환경을 source 한 뒤, 로봇 도메인과 겹치지 않는 그래프에서
ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  PYTHONPATH=$PWD/src/rescue_contracts:$PWD/src/rescue_robot:$PYTHONPATH \
  python3 tests/ros/e3_odom_probe.py
# 같은 환경에서
python3 tests/ros/e3_readiness_probe.py
python3 tests/ros/e3_watchdog_probe.py
```

E3 탐침은 실제 토픽이 네임스페이스별로 섞이지 않는지 본다: odom 탐침은 `/robot_a/odom`·`/robot_b/odom` 정지 증거, 준비 신호 탐침은 odom·LaserScan·`map → odom` TF, watchdog 탐침은 전달·heartbeat 만료 latch·회복 뒤에도 latch 유지·`~/reset`·robot_b 격리와 어댑터 `watchdog` flag. `NavigateToPose` action 은 다루지 않는다.

## 기록

아래는 당시 설명을 날짜와 함께 그대로 옮긴 것이다. 지금 상태는 맨 위 「현재 상태」 표를 본다. 예를 들어 「LIVE_ROS 는 SEARCH_AREA 만」은 2026-10-13 자동 대상 이동(#166·#179·#192) 전의 설명이다.

### 2026-10-13 (낮): 「지금 안 되는 것」 표

**지금 안 되는 것**

| 무엇 | 왜 / 어디 |
| --- | --- |
| LIVE_ROS 입력의 MOVE_SEGMENT·REOBSERVE_TARGET·RETURN_HOME | LIVE_ROS 는 SEARCH_AREA 와, 정책이 켜진 자동 대상 접근(APPROACH_TARGET, 아래 절, #166)만 받는다. 나머지는 `LIVE_SEARCH_ONLY` 로 거절하고 FILES 경로에만 있다. REOBSERVE_TARGET 은 PR #179 머지 뒤, RETURN_HOME 은 다음 PR(C1-c). MOVE_SEGMENT 는 FILES 경로만 |
| 빈 배열 파라미터 | ROS 파라미터로 빈 배열을 넘길 수 없다(launch_ros 거절, `-p`·`--params-file` 은 미설정). 그래서 빈 `camera_topics` 는 launch 가 거절한다 |
| watchdog 현장 값 확정 | 지금 값은 #132 임시값(아래 표). 결정 방식은 #94, 미확정 |

### 2026-10-08~10: 바뀐 문장

- 2026-10-10, 「ROS 노드」 절: 이 노드는 아직 **로봇을 움직이지 못한다**: 임무 입력이 없고(`ExecuteTask` IDL 은 `rescue_interfaces` 에 있음(#62), ExecuteTask 입력 노드는 미구현), 이 프로세스에 정렬 입력(mapping ROS 어댑터)이 없어 guard 가 모든 dispatch 를 거절한다. 필요한 ROS 패키지: `rclpy`, `rcl_interfaces`, `nav_msgs`, `nav2_msgs`, `irobot_create_msgs`.
- 2026-10-08, 「실행기 코어」: 이 코어를 단독으로 로봇에 연결하지 않는다. 인증된 단일 어댑터, 시각/센서 증거, 장비측 watchdog이 필요한 상태다.
- 2026-10-08, 「Nav2 명령 어댑터」: `guards(task) -> bool` 은 호출자가 주입하고 SEND 직전과 매 `step` 에 다시 본다(지도 revision 등; mapping 연결은 후속).
- 2026-10-08, 「Nav2 명령 어댑터」: 두 로봇은 같은 어댑터를 쓰고 task kind 만 다르다(SCOUT: `SEARCH_AREA`·`REOBSERVE_TARGET`, SUPPORT: `HOLD`·`APPROACH_TARGET`).
- 2026-10-08, 「지도 guard」: `REOBSERVE_TARGET`·`APPROACH_TARGET` 은 `target_gate(task)` 가 True 여야 한다. 호출자가 GoalSnapshot 검증(`coordinates_valid`)과 `auto_approach_eligible`(지금은 항상 False)을 묶어 넣는다. 그래서 backend 보정 검증 전에는 대상 지점 SEND 가 나가지 않는다. `SEARCH_AREA` 는 지도 guard 만으로 출발하고, `HOLD` 는 지도와 무관하다.
- 2026-10-08, 「준비 신호」: 최대 나이 값은 정해지지 않았다 [미확인]. 시험값만 있다.
- 2026-10-08, 「ROS 노드」 표: `policy.*` 등 「운용 정책. 실기 값 미정(T-01)」, `readiness.*` 「준비 신호 정책. 값 미정 [미확인]」.

### 2026-10-12 현장 수정: 파일 없는 SEARCH_AREA 입력

현재 권장 실행 경로는 `task.input_mode=LIVE_ROS`다. 아래의 FILES/task_prepare 설명은
기존 설정 호환용이며 이 경로의 선행 조건이 아니다. LIVE_ROS는 runtime.json,
inputs.json, input-sources.json, clock_basis_file, task.params.yaml을 읽거나 생성하지 않는다.
기존 `task_input` 하나에 지도/AMCL/SystemState 구독, ExecuteTask 수신, CommandLease 갱신을 연결한다.

고정 정책과 측정된 지도 정합은 타입을 명시한 ROS 파라미터로 준다(`live_parameters.py`).
정책·좌표·시각 기본값은 없다. YAML은 필수가 아니며 `--ros-args -p` 또는 launch의
parameters로 줄 수 있다. 기존 command_adapter와 동시에 실행하지 않는다.

중앙도 `live_inputs.enabled=true`로 실행해야 한다. 중앙의 `~/task_inputs`
(`rcl_interfaces/srv/GetParameters`, request.names=['source'])를 비동기 조회한다.
STRING 응답은 `rescue-live-task-source/1`의 메모리 기록이며 파일 교환용 JSON이 아니다.
응답에는 coordinator_id/publisher_boot_id/authority_epoch/schema_version/clock_domain_id,
실측 clock, 중앙 정책의 lease_duration_s/lease_renew_period_s 및 수색 목적점이 있다.
중앙 core에서 생성한 목적점과 양 PC의 실제 NTP 측정만 사용하고, 수신 시각으로 관측
시각·만료를 연장하지 않는다. 서비스 timeout은 lease.max_age_ns로 제한하며 cache를
무효화한다. SystemState의 새 boot 등록과 명시적 arm은 별도로 유지한다.

이 로컬 LIVE_ROS 모드는 SEARCH_AREA만 받는다. MOVE_SEGMENT·대상 접근은 기존 FILES
경로에서 유지하며 새 모드의 수색 조건으로 우회하지 않는다. 원래의 watchdog READY,
lease 수신 나이/만료·오차, 지도 session/revision/정합, Nav2 terminal + fresh odom
정지 관문은 유지한다. 시계 daemon 재시작/검출된 clock step은 epoch를 바꾸며 기존
TaskIngress 규칙대로 runtime 재시작을 요구한다.

clock backend는 `chrony` 또는 `timesyncd`를 명시한다. timesyncd는 systemd D-Bus
NTPMessage 원 reply stamp, root delay/dispersion, peer delay/offset, jitter/precision,
명시한 drift 정책을 사용한다. `timedatectl`의 synchronized=yes 한 줄을 오차 증거로
대체하지 않는다. 관련 OS 조회는 worker에서 실행하며 ROS callback은 기다리지 않는다.

확인: 파일 의존 실패를 원 첨부본에서 재현했고 Python 회귀 12건을 통과했다.
ROS/DDS 실송수신, colcon 빌드, 실제 두 PC의 D-Bus 반환, watchdog 배선, 로봇 주행은
이 작업 환경에 ROS가 없으므로 NOT_RUN이다. 고정 정책·지도 정합 수치가 미정이면
missing parameters로 거절한다. 입력 연결 수정이 현장 정책 확정을 대신하지 않는다.


### 설치 가능한 임무 입력 진입점 (`task_input`, 2026-10-11)

`ros2 run rescue_robot task_input`은 기존 `ros_node.main`과 동일한 프로세스에서
`task.runtime_factory`의 설치된 `module:callable`을 호출한다. 별도 Nav2 발행자나
두 번째 command adapter를 띄우지 않는다. factory는 `(node, adapter)`를 받아
`attach_task_runtime`의 인증·lease·지도·목표·결과 입력을 연결하고 `destroy()`를
가진 runtime을 반환한다. Jazzy의 타입 지정 파라미터 선언은 기존
`declare_required`를 재사용한다. factory 파라미터 누락은 기동 오류이며 자동 arm은 없다.

실기·배포 launch는 기존 command_adapter 프로세스를 이 진입점으로 교체하고
동일 namespace/정책/heartbeat 설정 및 설치된 factory를 넘겨야 한다. 둘을 동시에
실행하지 않는다. 아래 운영 factory는 포함했으며 실제 데이터 입력 제공자의 연결은 남아 있다.
단일 프로세스 설치와 구간 guard 결합은
[`tests/test_robot_task_entrypoint.py`](../../tests/test_robot_task_entrypoint.py) 및
[`tests/test_robot_task_runtime_integration.py`](../../tests/test_robot_task_runtime_integration.py)에서
모의 입력으로 검증한다. 시계 역행은 runtime을 차단하며 정상 복구 후 새 runtime이 필요하다.

`RETURN_HOME`은 수락할 때와 매 step에서 같은 `home_ref`의 현재 기록을 조회한다.
위치·frame·revision이 바뀌거나 조회가 실패하면 취소·정지 관문으로 들어가며
새 귀환 임무를 받아야 한다. 구간/귀환 cache 오류는 기동 중 로그에 남기고 guard를
거부한다. resolver와 검증 콜백은 검증된 로컬 cache를 읽으며 동기 RPC를 호출하지 않는다.

### 임무 입력 런타임과 watchdog 독립 개발 (2026-10-11)

[사용자 요구, 2026-10-11] “watchdog 없는 상태에서 구현을 완료하고 watchdog을 나중에
추가하는게 맞지 않아? … 필요한거 너가 제작해.” 적용 범위는 임무 입력 구현·개발 검증이다.
watchdog 현장 파라미터 결정(#94)을 소스 작성의 선행 조건으로 두지 않는다.

#85의 [소유자 결정 전달](https://github.com/jaebeom/ROKEY-7/pull/85#issuecomment-6106387180)과
[제환 대상 작업 순서](https://github.com/jaebeom/ROKEY-7/pull/85#issuecomment-6106393544)를
반영해 공통 ExecuteTask/CommandLease/수락·결과·정지 및 SEARCH_AREA·REOBSERVE_TARGET을
먼저 연결한다. MOVE_SEGMENT는 예비 모드로 유지한다. 연속 추종의 담당·입력/액션·거리·유실·
도착 규칙은 작전·PM의 Q-27 결정 대상이며, 기존 구간 이동을 추종으로 바꾸지 않는다.
AMR1 관제 START_SEARCH는 중앙이 ExecuteTask를 발급하는 조건이다. 로봇은 버튼/지진
이벤트를 독자 처리하지 않는다. main의 중앙 조정기는 DISASTER 상태에서 READY 로봇에
대한 START_SEARCH 요청을 받아 goal을 보내고 lease를 갱신한다.

`task_site_runtime:create_runtime`을 패키지에 포함했다. 사용자가 임의 factory/encoder를
새로 작성하지 않아도 된다. 한 CommandAdapter에 action/lease/admission/status를 붙이고,
`~/task_arm`, `~/task_stop`을 제공한다. 설치·재접속은 자동 arm이 아니다. watchdog 토픽이
없어도 endpoint를 생성하고 상태/거절 이유를 발행하지만, C-01의 실제 이동 READY 관문에서
watchdog가 없음을 정상으로 바꾸지 않는다. #92의 수동 Nav2 실습은 별도 구성이다.

운영 코드의 임무·lease·취소·정지 검사는 기존 시험 모음으로 실행한다:

```bash
python3 -I scripts/run_tests.py
```

별도 모의 런타임과 ROS 탐침은 설치하지 않는다. 현장 goal과 lease 발행에는 기존
`mission_coordinator`를 사용한다. 아래의 입력·기동 순서와 실제 설정을 적용한 뒤
`/rescue/operator_command`의 START_SEARCH로 수색을 요청한다. 좌표·권한·시계·Nav2·odom·
watchdog 연결은 현장에서 확인해야 한다. 로컬 시험 통과는 실제 ROS/로봇 주행 증거가 아니다.

CommandLease 거절 로그에는 lease/robot/boot/coordinator/authority/generation/sequence를
남긴다. 등록된 발행자·boot·권한 불일치, 중복/역행 sequence, 로컬 lease 만료와 STOPPING은
warning으로 구분한다. goal보다 먼저 저장한 lease와 다른 lease_id의 메시지는 debug로
기록한다. 이 진단은 lease 수락 조건이나 로컬 정지 기한을 바꾸지 않는다.

`task_prepare --directory <출력폴더> --robot-id <논리ID> --namespace <물리namespace>`는
runtime.json, input-sources.json, task.params.yaml, preparation.json을 만든다. 기존 adapter
파라미터에 task.params.yaml을 겹쳐 로드한다. 결정된 수치를 `--settings <JSON>`으로 넣을
수 있고 미정값은 null과 NOT_CONFIGURED 목록으로 남는다. 예시·모의 수치를 현장에 승계하지
않으며, 재실행해도 기존 input-sources.json의 시각/내용을 덮어쓰지 않는다. inputs.json을
새로 생성하지 않는다. 새 준비본은 아래 메모리 세션 모드를 사용하고 coordinator_state_topic도 명시해야 한다.

새 준비본은 실제 관측 기록(`rescue-robot-sources/1`)만 파일로 읽고 boot는 메모리에서 관리한다.
기존 수동 설정은 `InputStore`의 trusted IPC 파일(`rescue-robot-inputs/1`)을 계속 읽는다.
clock/authority/maps 및 임무별 targets/approvals/protected_routes/destinations를 원 관측 시각과
만료, map/transform revision, 증거 ID와 함께 제공한다. 이는 중앙·지도·경로 보호 adapter의
실제 입력을 수용하는 경계다. 제어된 파일/디렉터리의 신뢰를 전제로 하며 DDS 암호 인증을
구현한 것은 아니다. 파일 읽기로 유효기간을 늘리지 않는다. 데이터 없음·가림·만료·지도 변경·
권한 철회·입력 형식 오류는 거절/취소가 되고 정상값으로 대체하지 않는다. 실제 producer의
지속적인 export/캘리브레이션·Nav2 보호 증거 연결은 E3/E4 통합에서 확인한다.

시각 측정은 각 관련 PC에서 `clock_capture capture --host-id <호스트역할> --clock-domain
<팀기준> --output <JSON>`을 실행한다. `--interval-s <주기>`로 계속 갱신할 수 있다. 도구는
chronyc tracking/sources와 chronyd 프로세스/boot를 읽고 원문·종료 코드·보수적 오차 추정치를
저장한다. 설정/시각을 변경하지 않는다. 두 결과는 `clock_capture combine --sender <JSON>
--receiver <JSON> --clock-domain <팀기준> --max-age-ns <정책> --max-error-ns <정책>
--output <clock-basis.json>`으로 합친다. 양쪽 오차를 합산하며 미동기·미측정·낡음·상한 초과는
UNKNOWN/거절이다. formula는 [chrony 공식 문서](https://chrony-project.org/doc/4.5/chronyc.html#tracking)
기준이며 공통 시간 기준의 정확성·팀 domain 선택까지 자동 증명하지 않는다.

소프트웨어 구현/합성 검사와 실제 ROS·장비 검증을 구분한다. 실제 움직임을 주장하려면 생성
IDL/Jazzy, 중앙/지도/clock/protection 입력, Nav2 종료와 fresh odom 정지, watchdog 최종 속도
배선을 현장에서 확인해야 한다. 별도 사이트 runtime 코드 작성 요청으로 미루지 않는다.

### 현장 입력 구성과 범위 (2026-10-12)

`task_input`과 `task.runtime_factory` 선택 구조는 유지한다. 전용 SearchRuntime을
추가하지 않았다. FILES의 RETURN_HOME·MOVE_SEGMENT·REOBSERVE_TARGET은 기존
시각·승인·경로 관문을 사용한다. LIVE_ROS는 현재 SEARCH_AREA에 한정한다.

`saved_maps`는 실제 점유 지도 해시, AMCL 원 stamp·공분산, 명시한 지도
session/revision/정합·오차·유효기간을 검사한다. 실제 지도 파일은 별도 제공한다.
SystemState의 지정 publisher·boot·sequence·발행 나이를 검증하여 중앙 세션을
등록한다. 부트 변경·입력 만료는 취소/비무장을 요구하며 자동 재무장하지 않는다.
SEARCH는 특정 대상 snapshot이나 운영자 구간 승인을 요구하지 않는다.

FILES에서 `coordinator_targets_topic`을 설정하면 TargetSnapshot을 받아 대상
revision과 관측 만료를 확인한다. 기존 파일 입력은 호환 경로로 유지한다.
신규 승인 relay·Nav2 보호 mask 연결·추가 RequestStop 서비스는 이번 SEARCH
변경 범위에서 제외한다. 해당 설정은 거절하며 기존 phase1 gate를 우회하지 않는다.
정지는 기존 task_stop 서비스와 Nav2 terminal + fresh odom 관문을 사용한다.

현장 실행 순서·명시적 시험 설정은
[실행 안내](../../scripts/field/search-area/START_HERE.txt)에 있다. 스크립트는
단일 로봇 시험 fixture이며 일반 배포 기본값이 아니다. 같은 값을 launch 로 띄우는
방법(#152)은 맨 위 「실행」 절을 본다.

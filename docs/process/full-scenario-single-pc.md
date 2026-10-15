# 한 PC 전체 시나리오 (방식 B, 실시간 지도 공유) — 현장 절차서

기준일 2026-10-15. 사용자 작업 지시서(「PC 한 대에서 두 로봇 전체 시나리오 실행 — 실시간 지도 공유(방식 B)」)의 현장용 한 장이다. 방식 A(지도 두 번 고정, `config/field/search_area.field.json`)는 그대로 있고 이 절차와 섞지 않는다.

> **⚠ 실제 로봇은 사람이 현장을 지켜볼 때만 움직인다.** 아래 「리허설에서 확인한 것」은 모의 입력 결과이지 실기 결과가 아니다. 실기 확인 순서는 맨 아래 표를 따른다.

## 무엇이 자동인가

`scripts/field/run_full_scenario.sh` 한 번이 「웹캠이 지진을 감지했다」는 가정 이후를 모두 한다(웹캠 지진 판정 자체는 하지 않는다).

| 순서 | 단계 | 준비 완료 판정(프로세스가 떴다는 것만으로 넘어가지 않음) |
| --- | --- | --- |
| 0 | preflight | ROS Jazzy, `ROS_DOMAIN_ID`가 프로파일 hosts 의 domain_id 와 같음, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, 현장 파일(오버레이) 검사, TB-5 시작 자세 있음, 새 실행 폴더 |
| 1 | `live_map` | TB-6 출발 자세 기록기·SLAM 보정 감시·SLAM 위치 반복 노드가 그래프에 보임 (SLAM 보다 먼저: 서 있는 TB-6 은 SLAM 위치를 한 번만 낸다) |
| 2 | `tb6_slam` | `/robot6/map` 수신, `/rescue/map/start_pose/robot_a` 수신, `/robot6/rescue/slam_pose_latest` 수신. 시간 안에 안 되면 이 단계만 최대 2회 다시 시작 |
| 3 | `passage` | 두 로봇 `<ns>/passage_points` 수신 (`--no-overhead` 면 터널 판정 노드는 띄우지 않음) |
| 4 | `tb5_localization` | `/robot5/rescue_start_align` = `ALIGNED` (seed 적용 → 입자 퍼뜨림 → 정지 갱신 → 수렴·스캔 일치 확인). `LOW_MATCH`/`NOT_CONVERGED` 만 5 s 간격 최대 3회 재시작, 나머지 실패는 즉시 중단 |
| 5 | `robots` | 두 로봇 `task_arm` 서비스, `bt_navigator` active, `navigate_to_pose` 액션 서버, TB-6 도킹 서비스 `/rescue/robots/robot_a/dock` |
| 6 | `perception` | `/rescue/target_locations` 발행자(OAK-D 감지 + target_locator) |
| 7 | `operations` | `/rescue/report_receipts` 발행자(보고 저장, PERSISTED 가 TB-5 출동 조건) |
| 8 | `coordinator` | `/rescue/system_state` 수신, `refresh_state` 서비스 |
| 9 | `alert` (+`web`, `rviz`) | 노드/프로세스 확인 |
| 10 | `arm` | `refresh_state` → 두 로봇 `task_arm` 성공(거절 사유를 그대로 출력, 90 s) |
| 11 | `disaster` | 두 로봇 READY·STOPPED·작업 없음 → 합성 `DISASTER_SIGN` 1회(카메라 `cam_full_scenario_start`, 정책 `FULL_SCENARIO_SYNTHETIC_START`) → 조정기 DISASTER 와 원인 event 확인 |
| 12 | `mission` | `/rescue/mission_outcome`·SystemState 출력, 10 s 마다 두 Nav2 lifecycle 확인 |

TB-5 의 Nav2 는 AMCL 의 `map → odom` 이 있어야 활성화되므로 지시서 권장 순서와 달리 TB-5 위치 추정(4)을 로봇 launch(5) 앞에 둔다.

**끝 조건:** 조정기 `MISSION_FINISHED` + TB-6 도킹 확인(`scout_dock=DOCKED`) + TB-5 출구 도착(`support_exit=AT_EXIT`, 생존자 없음이면 TB-5 는 출발하지 않음) + 탐색이 `EXPLORATION_COMPLETE` 로 끝남 → `SUCCESS`(종료 코드 0). 탐색이 시간 제한(`EXPLORATION_TIMEOUT`)으로 끝났으면 TB-6 이 돌아와 도킹해도 `FINISHED_INCOMPLETE`(종료 코드 3). 그 밖은 `FAILED`(종료 코드 1, 단계·사유 출력).

**시나리오 상태 전이(조정기 `completion_mode` `EXIT_WAIT_SCOUT_DOCK`):**
- 인형 확정·보고 PERSISTED → TB-5 접근(APPROACH_TARGET, 대상에서 0.8 m 앞) → 도착(정지 확인) → `support_exit_wait_s` 15 s → 출구로(RETURN_HOME). 출구 QR 은 없다(2026-10-15): 출구 = 프로파일 `home_positions` 의 TB-5 좌표(TB-6 출발 자세 기준 실측), 없으면 TB-5 출발 자리. 출구 도착 뒤 TB-5 에는 아무 작업도 보내지 않는다(실행기가 지켜봄: 출구 뒤 작업이 생기면 `SUPPORT_TASK_AFTER_EXIT` 로 실패 처리).
- TB-6 은 TB-5 와 상관없이 frontier 를 다 볼 때까지(`exploration_empty_checks` 연속 빈 지도) 탐색을 계속한다 → `EXPLORATION_COMPLETE` → 기록한 자기 출발 자세(프로파일에 TB-6 도킹 접근점이 있으면 그 점, 역시 출발 자세 기준)로 RETURN_HOME → 도착(정지 확인) → 도킹 노드에 요청 → Create3 `dock` 액션 결과 `is_docked` **와** 그 뒤의 `dock_status.is_docked` 둘 다 참일 때만 `DOCKED` → `FINISHED`.

## 사전 준비 · 안전 확인

1. 두 로봇의 Pi·드라이버·센서가 켜져 있고 **둘 다 언도킹된 상태**로 각자 출발 자리에 서 있다(언도킹은 기존 감독 절차. 도킹된 채면 TB-5 정렬이 `START_ALIGN_DOCKED`, arm 이 `NAV_NOT_READY` 로 멈춘다). Pi 무선 절전·시계는 [field README 0단계](../../src/rescue_bringup/README.md)를 따른다.
2. 같은 로봇에 다른 SLAM·AMCL·Nav2·explore 가 떠 있지 않다(`bash scripts/field/stop_robot.sh /robot6` 끝의 프로세스 목록).
3. TB-5 출구(QR 없음)는 TB-6 출발 자리에서 `traffic_min_separation_m` 0.6 m 보다 멀어야 한다(리허설 26회차: 1.0 m 에서도 TB-6 복귀 끝이 collision_monitor 감속에 걸려 기한을 넘긴 적이 있다 — 가능하면 더 멀리, 또는 프로파일 `home_positions` 로 출구를 따로 준다). 가까우면 TB-6 복귀가 `TRAFFIC_ROBOTS_TOO_CLOSE` 로 계속 보류된다(충돌 방지라 일부러 풀지 않음).
4. 작업공간 빌드(저장소 루트):
   ```bash
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install --packages-up-to rescue_bringup
   ```

## ROS 2 환경 · `ROS_DOMAIN_ID`

실행 전에 셸에서 export 한다. 실행기가 값을 출력하고 프로파일과 다르면 시작하지 않는다.

```bash
export ROS_DOMAIN_ID=<프로파일 hosts 의 domain_id>
export ROS_DISCOVERY_SERVER='<현장 Discovery 설정, field README 0단계와 같게>'
```

실행기는 `ROS_SUPER_CLIENT=False`(띄우는 노드), `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 를 넣는다. 실행기 자신만 그래프 확인용 super client 다.

## TB-5 시작 자세 (`x`, `y`, `yaw`)

- 단위 m, m, rad. **기준은 TB-6 출발 자세**(TB-6 출발 자리에서 앞이 +x, 왼쪽이 +y. 방식 A 와 같은 OPS-141 규칙).
- 넣는 곳(하나): 실행 인자 `--tb5-start X Y YAW`, 또는 환경 변수 `TB5_START_X`·`TB5_START_Y`·`TB5_START_YAW`, 또는 현장 파일 `robots.robot_b.initial_pose_xy_yaw`. 이 셋이 모두 없으면 시작하지 않는다(값을 지어내지 않음).
- 실측 방법은 [field README 「TB-5 출발값 재기」](../../src/rescue_bringup/README.md)의 `capture_initial_pose.py` 를 그대로 쓴다. 줄자 값도 된다: start_align 이 0.5 m·20° 안의 오차를 바로잡는다(값 [제안]). 리허설 실행 스크립트의 `0.08 0.94 0.05` 는 **모의 정답에 일부러 오차를 넣은 합성값**이지 현장값이 아니다.

## 전체 실행 명령

```bash
bash scripts/field/run_full_scenario.sh --profile <프로파일.json> --perception-args <amr_perception 인자.json> --tb5-start <X> <Y> <YAW>
```

| 인자 | 뜻 |
| --- | --- |
| `--perception-args` | `amr_perception.launch.py` 의 현장 인자 전부를 담은 JSON 객체(카메라 토픽·frame·시계·재확인·exit_qr 등). 필수 |
| `--field` | 기본 `config/field/live_map.overlay.json`(방식 B 오버레이) |
| `--run-dir` | 새 폴더(기본 `~/rokey7_runs/full-<시각>`). 입력 사본·`runner.log`·`logs/<단계>.log`·`result.json` |
| `--no-rviz` / `--no-overhead` / `--no-alert` | 부하·소리 끄기. 끄더라도 필수 흐름(지도·위치·감지·출동·복귀)은 그대로다 |
| `--web-accounts <db>` `--web-port 8080` | 관제 웹도 띄움(계정 DB 는 rescue_console README) |
| `--webcam-params <yaml>` | 운영 launch 에 웹캠 노드도 띄움(시작 신호는 어차피 합성) |
| `--coordinator-args <json>` | 조정기 launch 의 route/observe/ROI 자리표시 값 바꾸기 |
| `--keep-running` | 끝난 뒤 Ctrl+C 까지 띄워 둠(RViz 확인용) |

## 실행 중 볼 것

- 화면: 단계마다 `[단계] 시작` → `[단계] 준비 완료: [...]` 또는 `[단계] FAILED <사유>`. 임무 중에는 `outcome {...}`, `system mode=... phase=... reason=...` 이 바뀔 때만 나온다.
- RViz(기본 켜짐): `shared_map_view` 한 화면에 `/robot6/map`, 두 로봇 위치 화살표(`/rescue/view/robots`), TB-5 RobotModel.
- 정상 흐름의 `reason`: `SEARCH_QUEUED` → (`SEARCHING`) → 인형 뒤 `SUPPORT_EXIT_IN_<n>S` → `SUPPORT_EXITING` → `SUPPORT_AT_EXIT` → `SCOUT_RETURNING` → `SCOUT_AT_START_POSE` → `SCOUT_DOCKING` → `SCOUT_DOCKED`, phase `FINISHED`.
- TB-5 출구 대기 확인: outcome `support_exit=AT_EXIT`, `/rescue/robots/robot_b/status` 의 `active_task_id` 가 비어 있고 `motion_state=STOPPED`.
- TB-6 AutoSLAM 완료: 조정기 로그 `EXPLORATION_COMPLETE reason=EXPLORATION_COMPLETE`(시간 제한이면 `reason=EXPLORATION_TIMEOUT`), outcome `exploration_complete=true`.
- TB-6 복귀·도킹: outcome `scout_return=AT_DOCK_APPROACH` → `scout_dock=DOCKED`, `/rescue/robots/robot_a/dock_state` JSON `state=DOCKED, is_docked=true`.
- SLAM 보정: 조정기 로그 `MAP_CORRECTION shift_m=... targets_observed_before=[...]`, outcome `map_corrections`.
- 교통 관제: `TRAFFIC_SCOUT_CANCELED`(TB-5 먼저) → `TRAFFIC_REARM robot=robot_a armed`(조정기가 자기가 멈춘 TB-6 를 다시 arm, 교통 취소·frontier 기한 초과 뒤만) → `TRAFFIC_YIELD`(비켜서기). `TRAFFIC_REARM_GAVE_UP` 이면 TB-6 가 정지 고정 상태다: 상황을 본 뒤 손으로 `task_arm`.
- 접근점: `APPROACH_POINT_MOVED`(인형 옆이 막혀 같은 거리의 다른 빈 점으로). 계속 `SUPPORT_APPROACH_NOT_FREE` 면 인형 둘레가 모두 막힌 것이다.

## 중단 · 비상

- **Ctrl+C**(실행기 터미널): 두 로봇에 실행기 `task_stop` → Nav2 목표 전부 취소 → TB-6 도킹 노드 취소·Create3 dock 목표 취소 → `cmd_vel` 0 ×5 를 보내고(줄마다 `[보냄]/[없음]/[실패]`), 그다음 모든 프로세스 그룹을 INT → TERM → KILL 로 끈다. 단계 실패·시간 초과도 같은 순서로 정리한다.
- 실행기가 응답하지 않으면 다른 터미널에서 기존 정지 수단: `bash scripts/field/stop_robot.sh /robot6`, `bash scripts/field/stop_robot.sh /robot5`.
- **그래도 움직이면 사람이 로봇을 들어 올린다.** 현장 정지를 웹이나 이 PC 에만 의존하지 않는다.

## 사람 발견 알림음

- TB-5 의 APPROACH_TARGET 이 COMPLETED(정지 확인)로 끝나 조정기가 그 대상을 `APPROACH_COMPLETE` 로 올린 순간에 0.6 s 두음 확인음(최대 음량의 1/4)을 한 번 낸다. 목표 수신·접근 시작(ASSIGNED/IN_PROGRESS)에는 울리지 않는다.
- 같은 (임무 trigger event, 대상)에는 한 번만. 다음 임무(새 DISASTER event)는 다시 울린다. 재생기는 `paplay` → `pw-play` → `aplay` 순으로 찾고, 없거나 실패하면 `FOUND_ALERT_NOT_PLAYED <사유>` 로그만 남기고 진행한다.
- 끄기: `--no-alert`(이벤트 로그 `FOUND_ALERT_DISABLED` 만). 로그: `logs/found_alert.log`.

## 실패 시 볼 로그 · 복구 순서

1. 화면 마지막 `[단계] FAILED <사유>` 와 `result.json`.
2. 그 단계의 `logs/<단계>.log`(예: `logs/robot_robot_b.log`, `logs/tb5_localization.log`, `logs/coordinator.log`).
3. 자주 보는 사유: `start_align FAILED START_ALIGN_JUMP_TOO_LARGE`(시작 자세 오차 0.5 m 초과 → 다시 재기), `START_ALIGN_DOCKED`(언도킹), `CLOCK_UNCERTAIN`(시계·Discovery 확인), `NAV2_DOWN`, `APPROACH_FAILED_<사유>`, `SCOUT_DOCK_FAILED <DOCK_...>`.
4. 복구: 로봇을 사람이 안전하게 세운 것을 확인 → 원인 수정 → **새 `--run-dir` 로** 처음부터 다시 실행(중간부터 이어 하지 않는다. TB-6 출발 자세가 회차마다 새로 기록된다).

## 리허설(로봇 없이, 이 PC 에서만)

```bash
bash scripts/field/full-scenario-rehearsal/run.sh <colcon install> <결과 폴더> 195
```

로봇 Pi·라이다·도크·OAK-D 감지기만 `sim_world.py`(2D, `world.json`)로 바꾸고 나머지는 실제 노드다. 로컬 Discovery Server(127.0.0.1, 현장 설정을 지움)로 현장 그래프와 섞이지 않는다. 같은 Discovery Server 에 붙으면 도메인 번호가 달라도 토픽 목록이 보였으므로(2026-10-15 측정) 그래프를 나누는 것은 서버다. 같은 도메인에 앞 회차 프로세스가 남아 있으면 시작하지 않는다. 결과·남은 위험은 [리허설 기록](../evidence/rehearsal-2026-10-15-full-scenario-one-pc.md)에 있다.

## 리허설에서 확인한 것 / 실기에서 따로 확인할 것

| 리허설(모의 입력)에서 확인 | 실기에서 처음 확인할 것(순서) |
| --- | --- |
| 단계별 준비 판정, 실패 시 단계·사유 출력, Ctrl+C·실패 시 안전 정지 명령 전송과 프로세스 정리 | 1. 두 로봇 언도킹 상태에서 실행 → `tb6_slam`·`tb5_localization` 준비(실제 라이다로 start_align `ALIGNED`, 일치율) |
| 실제 slam_toolbox·AMCL·Nav2 두 벌·실행기·조정기 연결, TB-5 seed 오차 보정 | 2. arm 까지(실제 시계·Discovery, 한 PC `single_host`) |
| 합성 지진 이벤트 → 탐색 자동 시작 | 3. TB-6 탐색 주행(사람 감시), 정지 상태 SLAM 처리 설정의 영향 |
| TB-6 탐색 완료 → 출발 자세 복귀 → 모의 도크 `DOCKED` → `FINISHED`, 인형 발견 → 보고 PERSISTED → TB-5 접근(결과는 리허설 기록의 결과 표) | 4. 실제 인형 감지 → 좌표 → TB-5 접근·알림음 → 출구 대기 |
| 교통 관제 취소·기한 초과 뒤 자동 재 arm, 막힌 접근점 대체 | 5. TB-6 복귀 → Create3 도킹 성공(`dock_status`) |
| | 6. 인형 위치를 바꿔 카메라 탐색 범위 확인(라이다 지도가 끝나도 카메라가 못 본 곳이 남을 수 있음) |

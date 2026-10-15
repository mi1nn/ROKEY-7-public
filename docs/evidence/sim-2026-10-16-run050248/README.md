# 가제보 실습 열한째 회차 (full-1016-050248, 2026-10-16 05:02 ~ 05:11, sim1)

측정·로그 자료를 먼저 적는다. 가제보 실습의 해석은 맨 끝 「참고: 가제보 실습 의견」에만 둔다(재범 지시 2026-10-16).

- 빌드: `b8a758d`(main, `src/`·`config/` diff 0줄). 10회차 빌드(`982aa58`) 뒤 들어간 것:
  - #430·#433(C15: 고정 지도 단계에서 실행기가 AMCL 위치를 놓치지 않게 함, `AMCL_NOMOTION_REQUESTED`)
  - #428(sim 터널 덮개), #429(sim 깊이 실물화), #431(sim 라이다 실물화)
- world: sim1(`arena.sdf`).
- 현장 값과 다름: `SIM_HOME=odom`. 10회차와 같다.
- 시간 동기: 기동 직전 timesyncd spike=false 였다. `logs/timesync_status_at_start.txt` 는 Offset +6.2 ms, Delay 211.6 ms 다.
- 기동: run_full 이 TB-6 `ARMED` 까지 손 없이 끝냈다(05:04:47).
- 재난 선포: 재범 위임으로 가제보 실습이 관제 웹 `POST /commands kind=DECLARE_DISASTER`(admin) 를 보냈다. `accepted: true`, 조정기 `DISASTER_DECLARED` 05:06:06.15.
- 녹화: Gazebo 화면 클라이언트와 run_full 의 두 로봇 RViz 를 좌우로 녹화했다(05:05:43 ~ 05:11:10).
- 손으로 한 것: 런북 7.5.1·7.5.2 를 운영자 대역 대신 했다. 05:08:17 slam_toolbox 에 SIGINT, 05:08:41 frozen_localization robot_a 기동. 아래 「2차 고정 뒤 SLAM 이 안 꺼짐」 절에 있다.

## 단계별 결과

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1. 언도킹으로 시작 | 통과 | run_full 이 두 대를 언도킹했다 |
| 2. 선포 즉시 TB-6 이 움직여 실습장을 훑음 | **실패** | 경계 5개가 `TOO_CLOSE` 3·`SMALL` 2 → 05:06:07.93 `EXPLORATION_COMPLETE reason=EXPLORATION_COMPLETE frontiers_sent=0`. TB-6 정답 위치는 (0.880, 0.722)에서 바뀌지 않았다 |
| 3. 웹 지도에 터널·인형이 뜸 | 실패 | `overhead (PASSABLE\|BLOCKED)` 0, `FAKE_TARGET published` 0 |
| 4. 인형 발견 → TB-5 출발 | 실행 안 됨 | `NO_SURVIVOR_FOUND`. TB-5 출발 정렬 `ALIGNED`(05:06:12, `match_ratio 0.88`) → `ARMED`(05:07:35) |

## 회차 표

| 시각 | 로그 |
| --- | --- |
| 05:06:06.15 | `DISASTER_DECLARED` → `SEARCH_AUTO_STARTED on DISASTER` → `FRONTIER none on map update 98 (1/3): frontiers=5 small=2 no_goal=0 tried=0 tight=0 no_path=0 too_close=3 \| TOO_CLOSE cells=90 centroid=(-1.32, 1.67); TOO_CLOSE cells=13 centroid=(-0.33, 2.22); TOO_CLOSE cells=13 centroid=(-0.23, 2.22); SMALL cells=4 centroid=(-0.79, -0.78); SMALL cells=4 centroid=(-0.58, 0.52)` |
| 05:06:07.02, 05:06:07.93 | update 99·100 도 같은 줄(2/3, 3/3) |
| 05:06:07.93 | `EXPLORATION_COMPLETE reason=EXPLORATION_COMPLETE frontiers_sent=0` → `SYSTEM_STATE … reason NO_SURVIVOR_FOUND` |
| 05:06:07.93 ~ .98 | `MAP_FREEZE 1`·`2` `success=True`(sha `47d091…`, 50×70) → `SEND RETURN_HOME … generation=0` → `ADMISSION -> False` |
| 05:06:07.99 | TB-6 `ExecuteTask rejected: FRAME_REVISION_MISMATCH … map handover in progress …` |
| 05:06:08.82 | TB-6 `MAP_HANDOVER robot_a R2 frozen grid on its map topic: SLAM pose until now, AMCL from here` → `MAP_STATE … rev=2 quality=UNKNOWN` → 05:06:10.82 `quality=VALID` |
| 05:06:12.19 | TB-5 `START_ALIGN_ALIGNED`(아래 TB-5 절) |
| 05:06:18.02 ~ 05:08:09.52 | `SEND RETURN_HOME` generation 1 ~ 12(10 s 간격). TB-6 은 매번 `FRAME_REVISION_MISMATCH … map handover in progress` 로 거절했다(generation 0 을 포함해 13회) |
| 05:06:42 | 운영자 대역 `런북 7.5 2차 고정 뒤 tb6_slam 끄기(launch PID …에 TERM, slam_toolbox PID 1007904)` |
| 05:07:12 | 운영자 대역 `slam_toolbox(PID 1007904)가 아직 살아 있음`, `/robot6/map 발행자 수 모름(--no-daemon)` |
| 05:07:29 | 운영자 대역 `SLAM 이 아직 지도를 냄 — frozen_localization 을 띄우지 않음` |
| 05:07:35 | 운영자 대역 robot5 `task_arm` `ARMED` |
| 05:08:17 ~ 05:08:18 | 가제보 실습이 손으로: slam_toolbox(PID 1007904)에 SIGINT → 1 s 안에 끝남 |
| 05:08:41 | 가제보 실습이 손으로: `/robot6/map 발행자 수 0` 확인 → frozen_localization robot_a 기동 |
| 05:08:44.97 | TB-6 amcl `Setting pose (…): -0.350 0.013 3.133` |
| 05:08:49.80 ~ .81 | amcl `Received a 50 X 70 map` / `MAP_RELAYED /rescue/map/frozen R2 -> /robot6/map` / TB-6 `MAP_HANDOVER robot_a R2 …` → `quality=UNKNOWN` |
| 05:08:49.94 | TB-6 `MAP_STATE … quality=VALID`, RobotPose 다시 나옴(`GAP robot_a 32.4 s`) |
| 05:08:50.51 ~ .53 | `SEND RETURN_HOME f738… generation=13` → `ADMISSION -> True` → TB-6 `SEND … GoalPose(frame_id='map', x=-1.2e-05, y=-0.0029, …)` |
| 05:08:50.63 | TB-6 bt_navigator `Extrapolation Error … Requested time 1792094930.625660 but the earliest data is at time 1792094930.895750` → `Initial robot pose is not available.` → `[navigate_to_pose] Aborting handle` |
| 05:08:51.44 | 조정기 `RESULT task=f738… outcome=FAILED reason=ACTION_ABORTED` |
| 05:08:51.51, 05:08:59.01 | 조정기 `SEARCH_AUTO_STARTED on DISASTER`. 운영자 대역 robot6 `task_arm` `ARMED`(05:08:59). 그 뒤 05:11:10 까지 `SEND` 없음 |
| 05:11:10 ~ 05:11:30 | 녹화 끔 → `stop_full.sh` |

## 선포 즉시 경계 0개 (작전 C16)

- 선포 직전 지도: `maps/050550-before-declare`(50×70, 0.05 m). 칸 수는 free 2502, unknown 694, occupied 304 다(`logs/map_counts_before_declare.txt`).
  - 10회차 선포 직전은 49×68 에 2662/572/98 이었다.
- 조정기 설정(기동 줄): `traffic_min_separation_m 0.6`, `freeze_clearance_m 0.2`, `frontier_goal_clearance_m 0.35`, `traffic_control TB5_PRIORITY`.
- `TOO_CLOSE` 는 `ros_node.py`(`b8a758d`) 757 ~ 809 줄에서 정한다. TB-5 둘레 `traffic_min_separation_m` 원을 막은 지도에서 목표나 길이 없고, 원을 뺀 지도에서는 목표와 길이 있는 경계다.
- TB-5 RobotPose(`track/pose_gap.txt`):
  - 선포(05:06:06.15) 전에는 `/rescue/robots/robot_b/pose` 메시지가 없었다(05:05:46 `COUNT robot_a=50`, robot_b 없음).
  - 05:06:08.16 `None -> UNKNOWN`, 05:06:12.49 `UNKNOWN -> VALID mission=(-0.431, 1.795)`.
- TB-6 RobotPose: 05:05:22.06 `VALID mission=(-0.350, 0.003)`. 05:06:10.07 ~ 05:06:11.06 은 `UNKNOWN`(1 s)이었다.
- 정답 위치: TB-6 (0.880, 0.722, 89.5°), TB-5 (2.700, 0.722, 89.6°). 회차 내내 둘 다 움직이지 않았다.

## 2차 고정 뒤 SLAM 이 안 꺼짐 (시뮬레이션 운영자 대역)

- 운영자 대역(`scripts/sim/sim_operator.sh` `slam_to_frozen`)은 05:06:42 에 tb6_slam launch PID 에 TERM 을 보냈다.
- `logs/diag_slam_not_stopped.txt`(05:07:50):
  - launch 프로세스는 없었다.
  - `sync_slam_toolbox_node`(PID 1007904)는 PPID 2450(subreaper), PGID·SID 는 launch PID 그대로이고 상태 `Sl` 이었다.
  - `ros2 topic info /robot6/map --no-daemon` 의 Publisher count 는 1 이었다.
  - `tb6_slam.log` 에는 TERM 뒤 종료 줄이 없다.
- SLAM 은 고정 뒤에도 지도를 저장했다. `maps/050655-live-r2-…` 가 그 예다.
- 가제보 실습이 그 PID 에 SIGINT 를 보냈고 1 s 안에 끝났다(`logs/operator_manual.log`). 그 뒤 발행자 0 을 확인하고 런북 7.5.2 명령으로 frozen_localization robot_a 를 띄웠다.
- 이 사이 TB-6 RobotPose 는 05:08:17.56 ~ 05:08:49.94 에 32.4 s 끊겼다(SLAM 끄기 → AMCL 첫 위치).

## C15 확인 (넘김 뒤 TB-6 위치)

- `AMCL_NOMOTION` 줄: 0. `/robot6/amcl_pose` 는 05:08:49.896 stamp 로 나왔다(`logs/diag_tb6_after_handover.txt`, latched 한 건).
- RobotPose 는 `MAP_HANDOVER`(05:08:49.81) 0.13 s 뒤 `VALID` 였고, 그 뒤 끊김이 없다. 05:10:48 의 `mission_pose` 는 (−0.350, 0.013)이다.
- 10회차(넘김 뒤 375.4 s 끊김)와 달리 이번에는 AMCL 이 지도를 받은 직후 `amcl_pose` 를 냈다. 그래서 C15 가 부르는 자리(새 `amcl_pose` 가 오지 않을 때)는 이 회차에 생기지 않았다.
- 05:10:48 `/robot6/tf` 4 s 동안 map→odom 3개, odom→base_link 101개였다. 10회차 같은 측정(5 s)은 map→odom 78개였다.

## RETURN_HOME 이 넘김 직후 Nav2 에서 거절됨

- 05:08:50.51 `SEND RETURN_HOME generation=13` 은 `MAP_HANDOVER`(05:08:49.81)보다 0.70 s 늦다.
- Nav2 bt_navigator 는 05:08:50.63 에 `Initial robot pose is not available` 로 중단했다. 그 직전 줄은 요청 시각(…930.625660)이 TF 버퍼의 가장 이른 시각(…930.895750)보다 0.27 s 앞이라는 `Extrapolation Error` 다.
- 같은 때 실행기의 RobotPose 는 `VALID` 였다(05:08:49.94 부터).
- 조정기는 `ACTION_ABORTED` 뒤 RETURN_HOME 을 다시 보내지 않았다(05:11:10 까지 2 분 19 s).

## TB-5 출발 정렬

- 05:06:12.19 `START_ALIGN_ALIGNED {"first_guess": [-0.3531, 1.8201, 3.1341], "jump_xy_m": 0.0822, "jump_yaw_rad": 0.0221, "map_sha256": "47d091…", "match_points": 625, "match_ratio": 0.8816, "pose": [-0.4312, 1.7945, -3.1270], "sigma_xy_m": 0.0747, …}`.
- 10회차에 이어 두 번째 `ALIGNED` 다. 정렬 순간 라이다는 `logs/diag_tb5_scan_at_align.txt`·`_2.txt` 다. TB-6 은 출발 자리 (0.880, 0.722)에 있었다.

## 그 밖의 자료

- 인형 대역 안 보낸 사유: `OUT_OF_RANGE` 2, `NO_START_POSE`·`TF_STALE`·`NO_TF` 각 1.
- RTF(30 s, 12표본): 중앙값 0.35, 최소 0.15, 최대 1.00, 0.5 미만 8. 부하 32 ~ 56(화면 녹화 켬).

## 10회차와 같은 것·다른 것

- 다른 것:
  - 탐색: 10회차는 경계 4개를 보냈다. 11회차는 0개였다(`TOO_CLOSE` 3).
  - 선포 직전 지도 occupied 98 → 304. 이 빌드에는 sim 라이다·깊이 실물화(#429·#431)가 들어 있다.
  - TB-6 넘김 뒤 끊김: 10회차 375.4 s → 11회차 0.13 s(SLAM 손으로 끈 뒤 AMCL 기동 기준).
  - RETURN_HOME: 10회차는 Nav2 가 받아 입구에서 `ACTION_ABORTED`. 11회차는 넘김 0.70 s 뒤 `Initial robot pose is not available` 로 바로 중단됐다.
  - 운영자 대역의 SLAM 끄기가 이번에는 안 됐다.
- 같은 것: 넘김 중 RETURN_HOME 을 `FRAME_REVISION_MISMATCH` 로 미룸(#426), TB-5 `ALIGNED`, 인형 관측 0.

## 파일

| 파일 | 무엇 |
| --- | --- |
| `logs/*.log` | 회차 폴더 원문(색 코드 지움, 경로·PC 이름 바꿈, 줄인 줄 없음). `operator_manual.log` 는 손으로 한 것(SLAM SIGINT, frozen_localization 기동) |
| `logs/diag_slam_not_stopped.txt` | SLAM 이 안 꺼졌을 때 ps·로그 끝·`/robot6/map` 발행자 |
| `logs/diag_tb6_after_handover.txt` | 넘김 뒤 `amcl_pose`·RobotPose·TF |
| `logs/diag_c7_*`, `logs/diag_c8_tb5_nav2.txt` | 운영자 대역이 남긴 자료 |
| `logs/diag_tb5_scan_at_align*.txt` | TB-5 정렬 순간 라이다 |
| `logs/map_counts_before_declare.txt`, `logs/timesync_status_at_start.txt` | 선포 직전 지도 칸 수, 기동 직전 시간 동기 |
| `maps/` | 선포 직전 지도(`050550-before-declare`), 1·2차 고정 지도, 고정 뒤 SLAM 저장본 한 장 |
| `track/` | 정답 위치(0.5 s), RobotPose 상태·끊김, RTF |
| `video/screen_gazebo_rviz_small.mp4`, `video/shot_*.png` | 화면 녹화 줄인 판과 녹화 시작(05:05:43) 기준 장면 5장: 20 s 선포 전, 26 s 탐색 0개 끝, 150 s SLAM 남음, 190 s 넘김·복귀 중단, 320 s 끝 |

## 참고: 가제보 실습 의견 (정확하지 않을 수 있음)

아래는 위 자료를 보고 짐작한 것이다. 확인하지 않은 해석이라 틀릴 수 있다.

- 경계 0개는 TB-5 둘레 0.6 m 원이 TB-6 출발 자리에서 북쪽으로 가는 길을 막은 모양으로 보인다. 지도 occupied 가 늘어 길이 좁아진 것도 겹쳤을 수 있다.
- RETURN_HOME 중단은 AMCL 의 map→odom TF 가 Nav2 버퍼에 들어오기 전에 목표가 간 탓으로 보인다. 현장에서도 넘김 직후라면 같을 수 있다.
- SLAM 이 안 꺼진 것은 시뮬레이션 운영자 대역 쪽 일로 보인다. 현장은 터미널 Ctrl+C 다.

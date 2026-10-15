# 가제보 실습 열두째 회차 (full-1016-052901, 2026-10-16 05:29 ~ 05:38, sim1)

측정·로그 자료를 먼저 적는다. 가제보 실습의 해석은 맨 끝 「참고: 가제보 실습 의견」에만 둔다(재범 지시 2026-10-16).

- 빌드: `d01431b`(main, `src/`·`config/` diff 0줄). 11회차 빌드(`b8a758d`) 뒤 들어간 것은 #440(C16: TB-5 둘레 원은 목표에만, `TOO_CLOSE` 만 남으면 완료로 세지 않음)이다. #437(운영자 대역 SLAM 끄기)은 이 빌드에 없다.
- world: sim1(`arena.sdf`).
- 현장 값과 다름: `SIM_HOME=odom`. 11회차와 같다.
- 시간 동기: 기동 직전 timesyncd spike=false 였다. `logs/timesync_status_at_start.txt` 는 Offset +1.1 ms, Delay 237.2 ms 다.
- 기동: run_full 이 TB-6 `ARMED` 까지 손 없이 끝냈다(05:30:58).
- 재난 선포: 재범 위임으로 가제보 실습이 관제 웹 `POST /commands kind=DECLARE_DISASTER`(admin) 를 보냈다. `accepted: true`, 조정기 `DISASTER_DECLARED` 05:31:57.35.
- 녹화: Gazebo 화면 클라이언트와 run_full 의 두 로봇 RViz 를 좌우로 녹화했다(05:31:45 ~ 05:38:03).
- 손으로 한 것: 런북 7.5.1·7.5.2 를 운영자 대역 대신 했다(작전 지시). 05:34:05 slam_toolbox 에 SIGINT, 05:34:18 frozen_localization robot_a 기동.
- 새 기록: `logs/diag_tf_handover_robot6.txt` 에 `/robot6/tf` map→odom(전부)·odom→base_link(1 s)·`/robot6/amcl_pose` 의 받은 시각과 stamp 를 적었다. 넘김 앞뒤 05:33:55 ~ 05:35:00 만 남겼다.

## 단계별 결과

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1. 언도킹으로 시작 | 통과 | run_full 이 두 대를 언도킹했다 |
| 2. 선포 즉시 TB-6 이 움직여 실습장을 훑음 | **실패** | 첫 경계(88칸)는 `COMPLETED`(05:32:27)였다. 남은 경계가 `FRONTIER_GOAL_TIGHT` 로 빠져 05:32:29.51 `EXPLORATION_COMPLETE frontiers_sent=1` 이 됐다. TB-6 정답 위치 최대 y 1.311, 이동 거리 1.84 m |
| 3. 웹 지도에 터널·인형이 뜸 | 실패 | `overhead (PASSABLE\|BLOCKED)` 0, `FAKE_TARGET published` 0 |
| 4. 인형 발견 → TB-5 출발 | 실행 안 됨 | `NO_SURVIVOR_FOUND`. TB-5 출발 정렬 `ALIGNED`(05:32:33.68, `match_ratio 0.92`) → `ARMED`(05:34:01) |
| (복귀) | **안 나감** | 2차 고정 뒤 비켜서기 `TRAFFIC_YIELD` 47회가 반복됐다. `SEND RETURN_HOME` 0회 |

## 회차 표

| 시각 | 로그 |
| --- | --- |
| 05:31:57.35 | `FRONTIER_GOAL_PAST_TIGHT (-0.76, 1.35) -> (-0.96, 1.97) for 0.35 m clear and 0.6 m from TB-5` → `FRONTIER e291… cells=88 centroid=(-1.332, 1.663) path_m=2.20 cost=-4.08 cut=False goal=(-0.96, 1.97, 1.57)` → 05:31:59.31 `SEND … generation=0` |
| 05:32:27.64 | `RESULT … COMPLETED` → `FRONTIER_DONE e291… goal_at=(-0.96, 1.97) frontier_remains=False` |
| 05:32:27.81 ~ 05:32:29.51 | `FRONTIER_GOAL_TIGHT cells=56 centroid=(-2.80, 2.29): no cell 0.35 m clear within 1.0 m, left out`(뒤 두 번은 cells=43 (-2.72, 2.24)) → `FRONTIER none … tight=1` 3회 |
| 05:32:29.51 | `EXPLORATION_COMPLETE reason=EXPLORATION_COMPLETE frontiers_sent=1` → `MAP_FREEZE 1`·`2` `success=True`(sha `5b5517…`, 87×70) |
| 05:32:29.59 | `TRAFFIC_YIELD 7bed… from=(-0.92, 1.72) to=(-0.97, 1.72, -3.06) path_m=0.05 why=TRAFFIC_ROBOTS_TOO_CLOSE` → `TRAFFIC_HOLD SCOUT TRAFFIC_ROBOTS_TOO_CLOSE` |
| 05:32:30.41 | TB-6 `MAP_HANDOVER robot_a R2 …`(SLAM 이 돌던 때, 첫째) |
| 05:32:31.61 ~ 05:34:07.11 | `SEND SEARCH_AREA` 950건(generation 1 ~ 950), 약 0.1 s 간격, 모두 `ADMISSION -> False`. TB-6 거절은 `INVALID_INPUT … input observation time` 7, `FRAME_REVISION_MISMATCH … map handover in progress` 943 |
| 05:32:33.68 | TB-5 `START_ALIGN_ALIGNED`(아래 TB-5 절) |
| 05:33:01 ~ 05:33:48 | 운영자 대역: tb6_slam launch 에 TERM → `slam_toolbox(PID …)가 아직 살아 있음` → `SLAM 이 아직 지도를 냄 — frozen_localization 을 띄우지 않음` |
| 05:34:05 ~ 05:34:18 | 가제보 실습이 손으로: slam_toolbox 에 SIGINT(1 s 안에 끝남) → `/robot6/map 발행자 수 0` → frozen_localization robot_a 기동 |
| 05:34:21.45 | TB-6 `MAP_HANDOVER robot_a R2 frozen grid on its map topic …`(둘째) → `quality=UNKNOWN` |
| 05:34:23.64 | TB-6 `AMCL_NOMOTION_REQUESTED robot_a: no amcl_pose since the map switch 2.2 s ago (standing robot)` |
| 05:34:24.39 ~ .42 | `/robot6/amcl_pose`(stamp …464.391) → RobotPose `VALID`(`GAP robot_a 18.3 s`) → `MAP_STATE … quality=VALID` |
| 05:34:24.65 ~ .76 | TB-6 비켜서기 목표 `GoalPose(… x=-0.9706, y=1.7157, yaw=-3.0569 …)` → bt_navigator `Extrapolation Error … Requested time 1792096464.751807 but the earliest data is at time 1792096465.391125` → `Initial robot pose is not available.` → `Aborting handle` |
| 05:34:25.61 | 조정기 `RESULT … FAILED reason=ACTION_ABORTED` → `TRAFFIC_YIELD_DONE 7bed… outcome=FAILED` |
| 05:34:33 ~ 05:37:24 | `TRAFFIC_YIELD` 46회 더 → `TRAFFIC_HOLD` → 약 2 s 뒤 `SEND SEARCH_AREA … ADMISSION -> True` → `TRAFFIC_RELEASE`. 05:35:32 부터는 약 3 s 간격으로 `from=(-0.95, 1.71) to=(-1.02, 1.72, 3.11) path_m=0.05` 와 `RESULT … COMPLETED reason=NAVIGATION_SUCCEEDED` 가 반복됐다 |
| 05:37:50 | `logs/diag_yield_loop_robot6.txt`(8 s) → 05:38:03 녹화 끔 → 05:38:21 `stop_full.sh` |

## 탐색 (C16 뒤 첫 회차)

- 선포 직전 지도: `maps/053156-before-declare`(50×70). 칸 수는 free 2545, unknown 680, occupied 275 다.
- 첫 경계는 11회차에 `TOO_CLOSE` 로 빠졌던 90칸 경계와 같은 자리(88칸, centroid (−1.332, 1.663))로 나갔다.
  - 목표는 `FRONTIER_GOAL_PAST_TIGHT … for 0.35 m clear and 0.6 m from TB-5` 로 (−0.96, 1.97)이 됐다.
  - 05:32:27 `COMPLETED` 였다. 그때 TB-6 정답 위치는 (2.600, 1.290)이었다.
- 남은 경계 (−2.80, 2.29)·(−2.72, 2.24)는 `FRONTIER_GOAL_TIGHT: no cell 0.35 m clear within 1.0 m` 로 빠졌다. 그래서 `frontiers_sent=1` 로 끝났다.
- 정답 위치 최소 거리 TB-6 ↔ TB-5: 0.576 m(05:32:26.25).

## 넘김 중 0.1 s 간격 송신

- `SEND SEARCH_AREA` 는 회차 전체에 1009건(generation 0 ~ 1008)이다. `ADMISSION` 은 True 59, False 950 이었다.
- False 950 은 모두 05:32:31.61 ~ 05:34:07.11 이다. 그 사이 TB-6 의 거절 사유는 `FRAME_REVISION_MISMATCH … map handover in progress` 943, `INVALID_INPUT … input observation time` 7 이다.
- 같은 때 조정기에는 `TRAFFIC_YIELD 7bed…`(05:32:29.59) 하나가 걸려 있었다. 그 작업은 05:34:25.61 `TRAFFIC_YIELD_DONE … FAILED` 로 끝났다.
- 둘째 `MAP_HANDOVER`(05:34:21.45) 뒤에는 이 송신이 멈췄다.

## C15 확인 (처음으로 동작)

- 05:34:21.45 `MAP_HANDOVER`(둘째) → 05:34:23.64 `AMCL_NOMOTION_REQUESTED robot_a: no amcl_pose since the map switch 2.2 s ago (standing robot)`.
- 05:34:24.39 `amcl_pose` 를 받았다(`diag_tf_handover_robot6.txt`, 받은 시각 05:34:24.418, stamp …464.391).
- RobotPose 는 05:34:24.41 에 `VALID` 였다. SLAM 을 끈 05:34:06.13 부터 18.3 s 끊겼다.

## C17 재현 (넘김 직후 Nav2 TF)

- bt_navigator 05:34:24.76: `Requested time 1792096464.751807 but the earliest data is at time 1792096465.391125` → `Initial robot pose is not available.` 요청 시각이 버퍼의 가장 이른 시각보다 0.639 s 앞이다.
- TF 기록(`logs/diag_tf_handover_robot6.txt`):
  - SLAM 의 마지막 map→odom 은 stamp …445.947(05:34:05.9)이고, 받은 시각 − stamp 는 −0.085 ~ −0.125 s 다.
  - 05:34:05.86 ~ 05:34:24.42 의 18.6 s 동안 map→odom 은 없었다.
  - AMCL 의 첫 map→odom 은 05:34:24.419 에 받았고 stamp 는 …465.391 이다. 받은 시각 − stamp 는 −0.972 s 다.
  - 그 뒤 AMCL map→odom 의 받은 시각 − stamp 는 −0.943 ~ −0.988 s 다.
  - 같은 때 odom→base_link 는 +0.001 ~ +0.005 s 다.
- AMCL 설정: `turtlebot4_navigation` `localization.yaml` 32줄 `transform_tolerance: 1.0` [설정].
- 넘김 전 Nav2 planner 는 요청 시각 …445.947(SLAM 마지막 stamp)로 `Extrapolation Error` 를 1 s 마다 냈다(05:34:18 ~ 05:34:23).

## 비켜서기 반복 (복귀 안 나감)

- `TRAFFIC_YIELD` 47회, `TRAFFIC_YIELD_DONE` COMPLETED 43·FAILED 3. `SEND RETURN_HOME` 0회.
- 05:35:35 부터 TB-6 정답 위치는 (2.607, 1.311, 75.8°)에서 바뀌지 않았다. TB-5 (2.700, 0.722)까지 0.596 m 다.
- mission 좌표에서는 TB-6 (−0.95, 1.71), TB-5 `VALID mission=(-0.420, 1.783)` 이다.
- 반복 목표: `to=(-1.02, 1.72, 3.11) path_m=0.05`. Nav2 목표 판정은 0.25 m·0.25 rad 다(`src/rescue_bringup/README.md` RPP 항목 [문서]).
- 조정기 설정: `traffic_min_separation_m 0.6`.
- `logs/diag_yield_loop_robot6.txt`(05:37:50, 8 s): odom→base_link 211, map→odom 30, `rescue/low_obstacle_points` 24건. `lookahead_point`·`local_plan` 은 0건(그 순간 따라가는 경로 없음).

## TB-5 출발 정렬

- 05:32:33.68 `START_ALIGN_ALIGNED {"first_guess": [-0.3521, 1.8200, 3.1336], "jump_xy_m": 0.0774, "jump_yaw_rad": 0.0075, "map_sha256": "5b5517…", "match_points": 625, "match_ratio": 0.9168, "pose": [-0.4199, 1.7827, 3.141…], …}`. 10·11회차에 이어 세 번째 `ALIGNED` 다.
- 정렬 순간 라이다: `logs/diag_tb5_scan_at_align.txt`·`_2.txt`.

## 그 밖의 자료

- RobotPose 끊김(`track/pose_gap.txt`):
  - TB-6: 05:32:30.13 ~ 05:32:32.63 2.5 s(첫 `MAP_HANDOVER` 직후), 05:34:06.13 ~ 05:34:24.41 18.3 s. 그 밖에 05:34:40.14·05:34:58.64 에 `UNKNOWN` 이 각 0.1 s 안이었다.
  - TB-5: 05:32:29.75 `UNKNOWN` → 05:32:33.81 `VALID`.
- 인형 대역 안 보낸 사유: `NOT_IN_FRONT` 3, `OUT_OF_RANGE` 2, `NO_TF` 2, `NO_START_POSE`·`TF_STALE` 각 1.
- RTF(30 s, 14표본): 중앙값 0.98, 최소 0.15, 최대 1.19, 0.5 미만 4. 부하 36 ~ 59(화면 녹화 켬).
- Nav2: `detected collision ahead` 0, `Failed to make progress` 2.

## 11회차와 같은 것·다른 것

- 다른 것:
  - 탐색: 11회차 `frontiers_sent=0`(`TOO_CLOSE` 3) → 12회차 첫 경계 88칸 `COMPLETED`, 남은 경계 `TIGHT` 로 `frontiers_sent=1`.
  - 2차 고정 뒤: 11회차는 RETURN_HOME 을 10 s 간격으로 13회 보냈다. 12회차는 `TRAFFIC_YIELD` 가 먼저 걸려 SEARCH_AREA 를 0.1 s 간격으로 950회 보냈다.
  - C15: 11회차 `AMCL_NOMOTION` 0 → 12회차 1회(2.2 s 뒤)로 RobotPose 를 다시 냈다.
  - 넘김 직후 Nav2 TF 중단(C17): 11회차는 RETURN_HOME, 12회차는 비켜서기 목표에서 났다. 0.27 s → 0.64 s.
- 같은 것: 운영자 대역 SLAM 끄기 실패(손으로 SIGINT), TB-5 `ALIGNED`, 인형 관측 0.

## 파일

| 파일 | 무엇 |
| --- | --- |
| `logs/*.log` | 회차 폴더 원문(색 코드 지움, 경로·PC 이름 바꿈, 줄인 줄 없음). `operator_manual.log` 는 손으로 한 것 |
| `logs/diag_tf_handover_robot6.txt` | 넘김 앞뒤 map→odom·odom→base_link·amcl_pose 의 받은 시각과 stamp(05:33:55 ~ 05:35:00) |
| `logs/diag_slam_not_stopped.txt` | SLAM 이 안 꺼졌을 때 ps·로그 끝 |
| `logs/diag_yield_loop_robot6.txt` | 비켜서기 반복 중 RPP·낮은 장애물 점·TF(8 s) |
| `logs/diag_c7_*`, `logs/diag_c8_tb5_nav2.txt` | 운영자 대역이 남긴 자료 |
| `logs/diag_tb5_scan_at_align*.txt` | TB-5 정렬 순간 라이다 |
| `logs/map_counts_before_declare.txt`, `logs/timesync_status_at_start.txt` | 선포 직전 지도 칸 수, 기동 직전 시간 동기 |
| `maps/` | 선포 직전 지도, 1·2차 고정 지도 |
| `track/` | 정답 위치(0.5 s), RobotPose 상태·끊김, RTF |
| `video/screen_gazebo_rviz_small.mp4`, `video/shot_*.png` | 화면 녹화 줄인 판과 녹화 시작(05:31:45) 기준 장면 5장: 12 s 선포, 42 s 첫 경계 도착, 120 s 넘김 기다림, 160 s 넘김·C17, 300 s 비켜서기 반복 |

## 참고: 가제보 실습 의견 (정확하지 않을 수 있음)

아래는 위 자료를 보고 짐작한 것이다. 확인하지 않은 해석이라 틀릴 수 있다.

- 0.1 s 간격 송신은 비켜서기 작업이 넘김 거절을 받고도 기다리지 않고 바로 다시 보낸 모양으로 보인다.
- C17 은 AMCL 이 map→odom 을 약 1 s 미래 시각으로 내기 때문에, 첫 TF 가 오기 직전·직후의 「지금」 조회가 버퍼 앞에서 걸리는 것 같다. SLAM 을 끈 뒤 18.6 s 동안 map→odom 이 없었던 것도 겹친다.
- 비켜서기 반복은 비켜설 목표가 Nav2 목표 판정(0.25 m) 안이라 움직이지 않고 성공으로 끝나서, TB-5 와의 거리가 0.6 m 를 넘지 못한 것으로 보인다.
- 남은 경계 `TIGHT` 는 터널 1 너머 좁은 곳이라 빠진 것 같다. 10회차는 같은 쪽으로 가다 입구에서 멈췄다.

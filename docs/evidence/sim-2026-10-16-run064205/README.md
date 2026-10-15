# 가제보 실습 열넷째 회차 (full-1016-064205, 2026-10-16 06:42 ~ 06:52, sim1)

측정·로그 자료를 먼저 적는다. 가제보 실습의 해석은 맨 끝 「참고: 가제보 실습 의견」에만 둔다(재범 지시 2026-10-16).

- 빌드: `a5d4d64`(main, `src/`·`config/` diff 0줄). 13회차 빌드(`e6f21d6`) 뒤 들어간 것:
  - #444(경계 제자리 회전)
  - #447(복귀 재시도)
  - #448(Nav2 직진 0.26 m/s)
  - #450(비켜서기 거리)
  - #451(거절 뒤 보류)
  - #452(#435 되돌림)
  - #453(좁은 경계도 한 번)
- world: sim1(`arena.sdf` 마지막 변경 `c5c4961`, 터널 지붕 판).
- 현장 값과 다름: `SIM_HOME=odom`. 13회차와 같다.
- 시간 동기: 기동 직전 timesyncd spike=false 였다. `logs/timesync_status_at_start.txt` 는 Offset +0.5 ms, Delay 204.1 ms 다.
- **기동 실패 1회(06:32:46 폴더, `failed-start-063246/`)**: TB-6 Nav2 가 켜지다 멈췄다.
  - lifecycle manager 는 06:34:16 `Configuring controller_server` 뒤로 나아가지 못했다.
  - 06:34:16.51 controller_server 가 `failed to send response to /robot6/controller_server/change_state (timeout): client will not receive response` 를 냈다.
  - 06:34:28 TB-5 amcl 도 같은 줄(`/robot5/amcl/change_state`)을 냈다.
  - 06:40:33 상태는 `robot6/controller_server inactive`, `planner_server`·`bt_navigator`·`behavior_server` `unconfigured`, `robot5/amcl inactive` 였다(`failed-start-063246/diag_nav2_bringup_stuck.txt`).
  - run_full 의 TB-6 arm 은 `STOP_OR_READINESS_NOT_READY NAV_NOT_READY missing=['nav2']` 였다. 06:41:10 에 끄고, 06:42:05 에 새 폴더로 다시 띄웠다.
  - 다시 띄우기 전 06:41:57 에 재범 지시로 이 PC 의 Firefox 와 GNOME 설정(Wi-Fi) 창을 껐다.
- 기동(두 번째): run_full 이 TB-6 `ARMED` 까지 손 없이 끝냈다(06:44:05).
- 재난 선포: 재범 위임으로 가제보 실습이 관제 웹 `POST /commands kind=DECLARE_DISASTER`(admin) 를 보냈다. `accepted: true`, 06:45:00.
- 녹화: 06:44:57 ~ 06:51:42.

## 단계별 결과

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1. 언도킹으로 시작 | 통과 | run_full 이 두 대를 언도킹했다 |
| 2. 선포 즉시 TB-6 이 움직여 실습장을 훑음 | **실패** | 경계 3개를 보냈다. 첫 경계 `COMPLETED` 뒤 터널 1 너머 두 경계가 입구에서 `ACTION_ABORTED` 였다(06:46:03 부터 `detected collision ahead`). 06:49:06 `EXPLORATION_COMPLETE reason=EXPLORATION_FRONTIERS_FAILED frontiers_sent=3 failed_frontiers=1`. TB-6 정답 위치 최대 y 1.63 |
| 3. 웹 지도에 터널·인형이 뜸 | 터널만 | 06:45:43 ~ 06:45:52 `overhead PASSABLE lowest=0.464~0.465 map=(-1.67~-1.69, 2.14~2.23)` → 장부 `overhead_passages` 1행(PASSABLE). 인형 `FAKE_TARGET published` 0 |
| 4. 인형 발견 → TB-5 출발 | 실행 안 됨 | 인형 관측 0. TB-5 출발 정렬 `ALIGNED`(06:49:13.91) → `ARMED`(06:50:11) |
| (복귀) | **실패** | RETURN_HOME 이 Nav2 까지 들어갔다(C17 재현 없음). 두 번 다 `localization uncertain` 으로 취소돼 `SCOUT_RETURN_FAILED` 로 끝났다 |

## 회차 표

| 시각 | 로그 |
| --- | --- |
| 06:45:00.44 | `FRONTIER_GOAL_PAST_TIGHT (-0.73, 1.35) -> (-0.93, 2.02) for 0.35 m clear and 0.6 m from TB-5` → `FRONTIER e291… cells=90 … goal=(-0.93, 2.02, 1.57)` → 06:45:02.37 `SEND … generation=0` |
| 06:45:22.17 | `RESULT … COMPLETED` → `FRONTIER_LOOK_SENT … turn 1/4 at e291… goal=(-0.93, 2.02, 3.14)`(#444) |
| 06:45:24.30 | `TRAFFIC_YIELD 7bed… from=(-0.92, 1.77) to=(-1.21, 1.77, -3.13) path_m=0.30 why=TRAFFIC_ROBOTS_TOO_CLOSE` → `TRAFFIC_HOLD` → `TRAFFIC_RELEASE` → 06:45:26.27 `SEND … generation=1` `ADMISSION -> True` |
| 06:45:43 ~ 06:45:52 | `overhead PASSABLE lowest=0.464 map=(-1.67, 2.23)` 외 4줄 |
| 06:45:53.65 | `RESULT … COMPLETED`(비켜서기) |
| 06:45:53.78 | `FRONTIER_GOAL_TIGHT_ONCE cells=69 centroid=(-3.39, 2.19) goal=(-2.89, 2.32): its own 0.2 m clear goal, once for this frontier`(#453) → `FRONTIER a116… path_m=2.18 goal=(-2.89, 2.32, 3.14)` → 06:45:55.77 `SEND … generation=2` |
| 06:46:03.07 ~ 06:49:04.44 | TB-6 `RegulatedPurePursuitController detected collision ahead!` 127줄. 정답 위치는 06:46:47 부터 (2.814, 1.628) 근처였다 |
| 06:47:24.15 | `RESULT … FAILED reason=ACTION_ABORTED` → `FRONTIER_RETRY a116… failed 1/2` → `FRONTIER f0bf… cells=61 … goal=(-3.40, 2.11, 3.14)` |
| 06:47:26 ~ 06:47:36 | `SCOUT_NOT_READY … TB-6 needs task_arm` → 운영자 대역 arm → 06:47:36.67 `SEND … generation=3` |
| 06:49:05.15 | `RESULT … ACTION_ABORTED` → `FRONTIER_GAVE_UP f0bf… after 2 failed attempts` → `FRONTIER none … TRIED cells=61 centroid=(-3.60, 1.73)` 3회 |
| 06:49:06.67 | `EXPLORATION_COMPLETE reason=EXPLORATION_FRONTIERS_FAILED frontiers_sent=3 failed_frontiers=1` |
| 06:49:10.17 ~ .20 | `MAP_FREEZE 1`·`2`(sha `c0c366…`, 113×70) → `SEND RETURN_HOME generation=4` |
| 06:49:10 ~ 06:49:30 | RETURN_HOME generation 4·5·6 → TB-6 `ExecuteTask rejected … map handover in progress` |
| 06:49:13.91 | TB-5 `START_ALIGN_ALIGNED` |
| 06:49:39.24 | 마지막 TB-6 RobotPose(SLAM). 운영자 대역이 SLAM 을 끄기 시작했다 |
| 06:50:06 | 운영자 대역 `frozen_localization robot_a 띄움`(손 없이) |
| 06:50:10.85 ~ 06:50:12.79 | amcl `Setting pose (…): -1.266 1.933 3.044` → `Received a 113 X 70 map` → `MAP_RELAYED … R2 -> /robot6/map` → `createLaserObject` |
| 06:50:12.66 | TB-6 `MAP_HANDOVER robot_a R2 …` → `quality=UNKNOWN` |
| 06:50:14.74 | TB-6 `AMCL_NOMOTION_REQUESTED robot_a: no amcl_pose since the map switch 2.1 s ago (standing robot)` → `AMCL_NOMOTION robot_a: /robot6/request_nomotion_update not available; asked again later` |
| 06:50:18.04 | `MAP_STATE … quality=VALID`, RobotPose 다시 나옴(`GAP robot_a 38.8 s`) → `SEND RETURN_HOME generation=7` |
| 06:50:28.77 ~ .80 | `SEND RETURN_HOME generation=8` → TB-6 `Begin navigating from current location (-1.27, 1.93) to (-0.00, -0.00)` |
| 06:50:38.82, 06:50:48.90 | TB-6 `[follow_path] [ActionServer] Aborting handle.` |
| 06:50:53.24 ~ .33 | TB-6 `MAP_STATE … quality=UNKNOWN` → `task guard rejected …: FRAME_REVISION_MISMATCH … localization uncertain` → `CANCEL` → `Goal canceled` → `MAP_STATE … quality=VALID`(0.09 s 뒤) |
| 06:50:54.15 | `RESULT … FAILED reason=GUARD_INVALID` → `SCOUT_RETURN_AGAIN … failed 1/2: sent again after admission_timeout_s`(#447) |
| 06:51:04.27 ~ .30 | `SEND RETURN_HOME generation=9` → TB-6 `Begin navigating from current location (-1.33, 1.90) to (-0.00, -0.00)` |
| 06:51:13.74 ~ .78 | `MAP_STATE … UNKNOWN` → `task guard rejected … localization uncertain` → `CANCEL` → `MAP_STATE … VALID`(0.02 s 뒤) |
| 06:51:15.15 | `RESULT … GUARD_INVALID` → `SCOUT_RETURN_FAILED` → `SYSTEM_STATE … reason SCOUT_RETURN_FAILED` |
| 06:51:42 ~ 06:52:03 | 녹화 끔 → `stop_full.sh`(06:51:45 뒤 Traceback·`process has died` 는 종료 신호 뒤) |

## 터널 1 입구 멈춤 (06:46:03 ~ 06:49:04)

- 정답 위치: 06:46:47 ~ 06:50:47 에 TB-6 은 (2.814, 1.628) 근처에 있었다. 7회차 (2.820, 1.703), 10회차 (2.948, 1.796)와 같은 입구다.
- world(`c5c4961`)에서 몸 중심까지 거리(충돌 상자):
  - `whitebox_south`·`whitebox_east`(모서리 (2.759, 2.001)): 0.377 m
  - `tunnel1_roof`(z 0.47 ~ 0.49): 0.382 m
  - `outer_east`: 0.607 m
- 06:46:49 ~ 06:46:57 원자료(`logs/diag_stuck_robot6.txt`):
  - `lookahead_point` 3건: base_link 기준 (−0.106, −0.386)
  - `lookahead_collision_arc`: 1점 odom (−1.278, 1.877)
  - `rescue/low_obstacle_points` 19건, 첫 장 8860점
  - odom→base_link (−1.259, 1.934, −1.854 rad)
- 낮은 장애물 점 첫 장(06:46:52.997, stamp …812.890):
  - 바닥 위 0.05 m 이상인 점은 3340개다. 그중 앞 0 ~ 0.8 m·좌우 ±0.3 m 안이 2377개, 가장 가까운 점은 0.338 m 다. 높이는 0.05 ~ 0.23 m 에 고르게 퍼져 있다.
  - 이 2377점을 정답 위치(06:46:53.13 (2.814, 1.628, 155.6°))로 world 에 놓으면 모두 `whitebox_south` 에서 0.10 m 안이다[계산].
  - `depth_low_layer` 는 0.05 ~ 0.23 m 를 칠한다(robot.launch 줄 `depth_low_layer 0.05..0.23 m above the floor`).
- `logs/diag_tunnel_*`(06:46:47 ~ 49): local costmap, `/robot6/plan`, `/robot6/scan`, keepout 마스크.
- Nav2(회차 전체): `detected collision ahead` 127, `Failed to make progress` 13, `PassageSlow` 25, `spin failed` 0.

## 넘김과 복귀 (C21·C22·C17·#447)

- SLAM 끄기·frozen_localization 은 운영자 대역이 손 없이 했다(06:50:06 기동). frozen pose 를 받았고(C21 재현 없음), 지도를 받은 0.12 s 뒤 `createLaserObject` 가 나왔다(C22 재현 없음). 그래서 `logs/diag_c22_robot6.txt` 는 잡지 않았다.
- RobotPose 는 06:49:39.24 ~ 06:50:18.04 의 38.8 s 끊겼다.
- C15: `AMCL_NOMOTION_REQUESTED`(06:50:14.74) 직후 `/robot6/request_nomotion_update not available; asked again later` 가 나왔다. RobotPose 는 06:50:18.04 에 다시 나왔다.
- C17: 넘김 뒤 RETURN_HOME(generation 8·9)은 둘 다 TB-6 Nav2 `Begin navigating …` 까지 갔다. `Initial robot pose is not available` 은 0이다.
- 넘김 전 RETURN_HOME 거절(`ExecuteTask rejected`)은 4건이다. 13회차의 SEARCH_AREA 반복 송신(301건)은 없었다.
- 복귀 실패: 두 번 다 RobotPose 가 아주 짧게 `UNKNOWN` 이 된 순간 실행기가 `localization uncertain` 으로 목표를 취소했다.

  | 순간 | UNKNOWN 길이 | 그때 TB-6 mission 위치 |
  | --- | --- | --- |
  | 06:50:53.24 → .33 | 0.09 s | (−1.332, 1.904) |
  | 06:51:13.74 → .76 | 0.02 s | (−1.133, 1.587) |

- 두 번째 복귀 중 TB-6 은 입구를 빠져나왔다. 정답 위치가 (2.814, 1.630)에서 06:51:14 (2.448, 1.417)로, 0.43 m 움직였다.
- #447: 첫 실패 뒤 `SCOUT_RETURN_AGAIN … failed 1/2` 로 10 s 뒤 다시 보냈고, 두 번째 실패 뒤 `SCOUT_RETURN_FAILED` 였다.

## TB-5 출발 정렬

- 06:49:13.91 `START_ALIGN_ALIGNED {"first_guess": [-0.3530, 1.8201, 3.1341], "jump_xy_m": 0.0687, "jump_yaw_rad": 0.0089, "map_sha256": "c0c366…", …}`(#435 되돌린 빌드). 정렬 순간 라이다는 `logs/diag_tb5_scan_at_align*.txt`.

## 부하 (프로세스별, `track/proc_load.txt`, 06:44:28 ~ 06:52:03, 16장)

| 프로세스 | 평균 %CPU | 최대 |
| --- | --- | --- |
| `gz sim arena.sdf`(Gazebo 서버) | 168 | 227 |
| `wall_clock_relay.py` | 75 | 100 |
| `parameter_bridge` | 74 | 192 |
| `task_input`(두 실행기) | 69 | 100 |
| `rviz2` | 31 | 67 |
| `strip_sensors.py` | 28 | 42 |
| `joint_state_publisher` | 24 | 42 |
| `ffmpeg` | 23 | 50 |

- firefox 는 이 회차 목록에 없다(06:41:57 에 끔).
- 시스템 부하 28 ~ 48. RTF(30 s, 15표본)는 중앙값 1.00, 최소 0.24, 최대 1.82, 0.5 미만 4 다.

## 그 밖의 자료

- 정답 위치: TB-6 이동 거리 2.69 m. TB-6 ↔ TB-5 최소 0.569 m(06:45:21.14). TB-5 는 움직이지 않았다.
- 인형 대역 안 보낸 사유: `OUT_OF_RANGE` 5, `NOT_IN_FRONT`·`OUT_OF_VIEW`·`NO_TF` 각 2, `NO_START_POSE`·`TF_STALE` 각 1.
- 선포 직전 지도 `maps/064500-before-declare`(50×70): free 2504, unknown 694, occupied 302.

## 13회차와 같은 것·다른 것

- 다른 것:
  - 탐색: 13회차 `frontiers_sent=1`(남은 경계 `TIGHT`) → 14회차 `frontiers_sent=3`. 좁은 경계를 `FRONTIER_GOAL_TIGHT_ONCE` 로 보냈고(#453), 첫 경계에서 제자리 회전을 했다(#444).
  - 고정 뒤 비켜서기: 13회차 SEARCH_AREA 반복 301건 → 14회차 비켜서기 1회(0.30 m)에 반복 없음(#450·#451).
  - 넘김: 13회차 frozen pose 못 받음·AMCL 라이다 미처리 → 14회차 둘 다 없음.
  - 복귀: 13회차 0회 → 14회차 Nav2 까지 2회, 둘 다 `localization uncertain` 취소.
  - 터널 판정: 13회차 0 → 14회차 PASSABLE 1행.
- 같은 것: 터널 1 입구 `collision ahead`(7·10회차와 같은 자리), TB-5 `ALIGNED`, 인형 관측 0.

## 파일

| 파일 | 무엇 |
| --- | --- |
| `logs/*.log` | 회차 폴더 원문(색 코드 지움, 경로·PC 이름 바꿈, 줄인 줄 없음) |
| `logs/diag_stuck_robot6.txt`, `logs/diag_tunnel_*` | 입구 멈춤 순간 RPP lookahead·충돌 호·낮은 장애물 점·TF, costmap·경로·라이다·keepout |
| `logs/diag_tf_handover_robot6.txt` | 06:49:00 ~ 06:51:30 map→odom·amcl_pose·odom→base_link 받은 시각과 stamp |
| `logs/diag_c7_*`, `logs/diag_c8_tb5_nav2.txt` | 운영자 대역 자료 |
| `logs/diag_tb5_scan_at_align*.txt`, `logs/map_counts_before_declare.txt`, `logs/timesync_status_at_start.txt` | TB-5 정렬 라이다, 선포 직전 지도 칸 수, 시간 동기 |
| `failed-start-063246/` | 기동 실패 폴더의 arm·로봇·조정기 로그와 lifecycle 상태 |
| `maps/`, `track/` | 선포 직전·고정 지도, 정답 위치·RobotPose·RTF·프로세스별 부하 |
| `video/screen_gazebo_rviz_small.mp4`, `video/shot_*.png` | 화면 녹화 줄인 판과 녹화 시작(06:44:57) 기준 장면 5장 |

## 참고: 가제보 실습 의견 (정확하지 않을 수 있음)

아래는 위 자료를 보고 짐작한 것이다. 확인하지 않은 해석이라 틀릴 수 있다.

- 입구 `collision ahead` 는 흰 상자 남쪽 벽을 실제로 본 것으로 보인다(점이 모두 벽 0.10 m 안). 입구에서 터널 쪽으로 돌 자리가 좁은 것 같다.
- 복귀 취소는 RobotPose 가 0.02 ~ 0.09 s 만 `UNKNOWN` 이 되어도 바로 목표를 끊는 동작으로 보인다. 현장에서도 같은 코드라 날 수 있어 보인다.
- 기동 실패의 `failed to send response … (timeout)` 는 13회차 첫 frozen_localization 실패와 비슷한 통신 지연 모양으로 보인다.

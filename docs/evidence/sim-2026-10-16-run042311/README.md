# 가제보 실습 열째 회차 (full-1016-042311, 2026-10-16 04:23 ~ 04:44, sim1)

측정·로그 자료를 먼저 적는다. 가제보 실습의 해석은 맨 끝 「참고: 가제보 실습 의견」에만 둔다(재범 지시 2026-10-16).

- 빌드: `982aa58`(main, `src/`·`config/` diff 0줄). 9회차 빌드(`b5e5872`) 뒤 들어간 것:
  - #424(NTP 사유)
  - #426(넘김 중 목표 미룸)
  - #427(C14, 목표를 가운데 가까운 가장 넓은 칸에)
  - #428·#429 는 이 빌드에 없다.
- world: sim1(`arena.sdf` 마지막 변경 `92a35b9`).
- 현장 값과 다름: `SIM_HOME=odom`(home_positions 를 언도킹 뒤 자세로 넣음). 9회차와 같다.
- 시간 동기: 기동 직전 timesyncd spike=false 였다. `logs/timesync_status_at_start.txt` 는 Offset −4.3 ms, Delay 200.4 ms, 서버 ntp.ubuntu.com 이다.
- 기동 때 TB-6 arm 이 watchdog `STOP_LATCHED EXECUTOR_HEARTBEAT_STALE` 로 실패했다. 가제보 실습이 런북 48행 watchdog reset(04:26:23) → refresh_state → `task_arm` `ARMED`(04:26:31)를 손으로 했다(`logs/operator_manual.log`).
- 재난 선포: 재범 위임으로 가제보 실습이 관제 웹 `POST /commands kind=DECLARE_DISASTER`(admin) 를 보냈다. `accepted: true`, 조정기 `DISASTER_DECLARED` 04:27:18.16.
- 녹화: Gazebo 화면 클라이언트와 run_full 이 띄운 두 로봇 RViz 를 좌우로 녹화했다(04:27:05 ~ 04:43:56).
- 손으로 한 진단 1회: 04:42:13 `/robot6/request_nomotion_update`(작전 지시). 아래 「넘김 뒤 TB-6 위치 끊김」 절에 있다.

## 단계별 결과

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1. 언도킹으로 시작 | 통과 | run_full 이 두 대를 언도킹했다 |
| 2. 선포 즉시 TB-6 이 움직여 실습장을 훑음 | **실패** | 04:29:36 부터 터널 1 입구 정답 위치 (2.948, 1.796)에서 끝까지 자리 이동 없음. 경계 작업 4개가 `EXPIRED` 2·`ACTION_ABORTED` 2 → 04:35:17 `EXPLORATION_COMPLETE reason=EXPLORATION_FRONTIERS_FAILED frontiers_sent=4 failed_frontiers=3`. TB-6 정답 위치 최대 y 1.796, 이동 거리 2.46 m |
| 3. 웹 지도에 터널·인형이 뜸 | 터널만 | 04:28:44 `overhead PASSABLE lowest=0.47 map=(-1.86, 2.13)` → 장부 `overhead_passages` 1행(PASSABLE). 인형 `FAKE_TARGET published` 0 |
| 4. 인형 발견 → TB-5 출발 | 실행 안 됨 | 인형 관측 0. TB-5 출발 정렬은 **처음으로 `ALIGNED`**(04:35:35, `match_ratio 0.83`) → TB-5 `ARMED`(04:36:39) |

## 회차 표

| 시각 | 로그 |
| --- | --- |
| 04:27:18.16 | `FRONTIER e291… cells=13 centroid=(-1.825, 2.204) path_m=2.54 cost=-0.22 cut=True goal=(-1.32, 2.10, 1.57)`(앞 줄 `FRONTIER_GOAL_PAST_TIGHT (-1.32, 1.11) -> (-1.32, 2.10) for 0.35 m clear`) → 04:27:20.05 `SEND … generation=0` |
| 04:27:44 ~ 04:29:18 | TB-6 `Failed to make progress` 8, `Running spin` 2(`spin completed successfully`), `Running wait` 2 |
| 04:29:01 | TB-6 정답 위치 (2.952, 1.665). 지도 좌표로 바꾸면 (−1.299, 2.072)이고 목표 (−1.32, 2.10)에서 0.035 m 다. 방향은 지도 기준 −172° ~ −140° 이고 목표 yaw 는 90° 다 |
| 04:29:20.12 ~ 04:29:21.12 | TB-6 `Goal canceled` → 조정기 `RESULT … EXPIRED reason=DEADLINE_EXPIRED` → `FRONTIER_RETRY e291… failed 1/2` |
| 04:29:21.32 | `FRONTIER_GOAL_PAST_TIGHT (-2.74, 2.00) -> (-3.07, 2.00)` → `FRONTIER a116… cells=37 centroid=(-3.020, 1.640) path_m=1.83 cut=False goal=(-3.07, 2.00, 3.14)` → 04:29:25.66 `SEND … generation=1` |
| 04:29:36 ~ | TB-6 `RegulatedPurePursuitController detected collision ahead!` 시작. 정답 위치 (2.948, 1.796) 에서 이 뒤로 자리 이동 없음(방향만 바뀜) |
| 04:31:26.62 | `RESULT … EXPIRED` → `FRONTIER f0bf… cells=61 centroid=(-3.632, 1.375) path_m=2.37 cut=True goal=(-3.53, 1.80, 3.14)` → 04:31:36 `SEND … generation=2` |
| 04:33:14.61 | `RESULT … FAILED reason=ACTION_ABORTED` → `FRONTIER_RETRY f0bf… failed 1/2` → `FRONTIER 9c32… cells=61 centroid=(-3.632, 1.374) … goal=(-3.53, 1.80, 3.14)`(같은 자리, 다른 id) → 04:33:21 `SEND … generation=3` |
| 04:35:15.12 | `RESULT … ACTION_ABORTED` → `FRONTIER_GAVE_UP 9c32… after 2 failed attempts` → `FRONTIER none … tried=3 … TRIED cells=61 (-3.63, 1.37); TRIED cells=23 (-4.58, 1.25); TRIED cells=16 (-4.66, 1.55)` 3회 |
| 04:35:17.07 | `EXPLORATION_COMPLETE reason=EXPLORATION_FRONTIERS_FAILED frontiers_sent=4 failed_frontiers=3` |
| 04:35:25.15 ~ .19 | `MAP_FREEZE 1`·`2` `success=True`(sha `cf4364…`) / TB-6 `MAP_REVISION_ADOPTED robot_a RNone -> R2` / `MAP_RELAYED … R1 -> /robot5/map` |
| 04:35:25.20 ~ 04:35:55.75 | `SEND RETURN_HOME` generation 4·5·6·7(10 s 간격). TB-6 은 매번 `ExecuteTask rejected: FRAME_REVISION_MISMATCH … map handover in progress (R2: SLAM pose until the frozen grid is on the map topic, then AMCL); new goals wait for it` → 조정기 `ADMISSION -> False` → `task not issued: TASK_REFUSED` |
| 04:35:35 | TB-5 `START_ALIGN_ALIGNED`(아래 TB-5 절) |
| 04:35:54 ~ 04:36:28 | 운영자 대역: 자료 남김 → tb6_slam 에 TERM → 04:36:11 `/robot6/map 발행자 수 0` → 자료 남김 → 04:36:28 frozen_localization robot_a 기동 |
| 04:36:31.74 | TB-6 amcl `Setting pose (…): -1.431 2.068 -2.996` |
| 04:36:33.36 | `MAP_RELAYED /rescue/map/frozen R2 -> /robot6/map` / TB-6 `MAP_HANDOVER robot_a R2 frozen grid on its map topic: SLAM pose until now, AMCL from here` → `MAP_STATE robot_a … rev=2 quality=UNKNOWN` |
| 04:36:36.65 | 조정기 `TRAFFIC_HOLD SCOUT TRAFFIC_SCOUT_POSE_UNKNOWN` → `task not issued: TRAFFIC_SCOUT_POSE_UNKNOWN`. 이 뒤 04:42:15 까지 `SEND` 없음 |
| 04:36:39 | 운영자 대역 robot5 `task_arm` `ARMED` |
| 04:42:13.10 | 손으로 진단 1회 `/robot6/request_nomotion_update`(작전 지시) |
| 04:42:15.20 ~ .30 | TB-6 `MAP_STATE robot_a … rev=2 quality=VALID` → 조정기 `SEND RETURN_HOME a717… generation=8` → TB-6 `Begin navigating from current location (-1.43, 2.07) to (-0.00, -0.00)` |
| 04:42:25 ~ 04:42:55 | TB-6 `Failed to make progress` 1, `Running spin` 5 → `Collision Ahead - Exiting Spin` 5, `detected collision ahead` 61, `Controller patience exceeded` 9 |
| 04:42:55.37 ~ 04:42:56.12 | TB-6 `Goal failed` → 조정기 `RESULT task=a717… outcome=FAILED reason=ACTION_ABORTED` |
| 04:43:01 ~ 04:43:03 | 운영자 대역 refresh_state → robot6 `task_arm` `ARMED`. 그 뒤 `SEND` 없음 |
| 04:43:51 | `logs/diag_return_*` 저장 → 04:43:56 녹화 끔 → 04:44:21 `stop_full.sh` 끝(04:44:03 뒤 Traceback·`process has died` 는 종료 신호 뒤) |

## 터널 1 입구 멈춤 (04:29:36 ~ 끝)

- 정답 위치: TB-6 (2.948, 1.796). 방향은 90° ~ 156° 사이에서 바뀌었다(정답 world 기준).
- 지도 좌표로 바꾸기 [계산]: 출발 자세 한 쌍(정답 (0.880, 0.721, 89.6°) ↔ 지도 (−0.355, 0, 179.5°))으로 맞췄다. 결과는 지도 = (−y + 0.366, x − 0.880), 방향 +90° 다. 둘째 쌍으로 확인했다. 멈춘 자리의 변환값은 (−1.430, 2.068)이고, 04:36:31 amcl `Setting pose -1.431 2.068` 와 같다.
- world `arena.sdf`(`92a35b9`)에서 몸 중심까지 거리(충돌 상자 기준):
  - 흰 상자 남동 모서리(`whitebox_south`·`whitebox_east` 가 만나는 (2.759, 2.001)): 0.279 m. Nav2 반지름 0.189 m 를 빼면 몸 가장자리 0.090 m 다.
  - 동쪽 벽 `outer_east` 안쪽 면 x=3.421: 0.473 m.
  - 흰 상자 동쪽 면 x=2.759 와 동쪽 벽 사이 틈은 0.662 m 다. 틈 가운데 x=3.090 에서 몸 중심까지 0.142 m(서쪽으로 치우침)다.
  - 터널 1 덮개 판 `tunnel1_lintel`(y=2.23, z 0.47 ~ 0.88): 0.429 m.
- 원자료(04:30:43 ~ 04:30:52, 둘째 경계 작업 중):
  - `logs/diag_tunnel_local_costmap.txt`: `/robot6/local_costmap/costmap`, frame `odom`, 0.06 m, 50×50, 창 가운데 (−1.380, 2.040). 값 ≥ 99 칸 662개(100 = 111, 99 = 551)의 좌표다.
    - 가운데에서 0.2 m 안 100 칸은 0개, 0.3 m 안은 2개, 0.4 m 안은 8개다. 가장 가까운 100 칸은 (−1.59, 1.89), 0.258 m 다.
    - 가장 가까운 99 칸은 (−1.47, 2.01), 0.095 m 다.
    - 지도 x −1.95 ~ −1.77 줄의 100 칸은 y=1.89(흰 상자 동쪽 면 자리)와 y=2.55(동쪽 벽 자리)뿐이다. 덮개 판 자리(x≈−1.86)에 칠한 100 칸은 없다.
  - `logs/diag_tunnel_plan.txt`: `/robot6/plan` 69점, (−1.407, 2.072) → (−3.070, 1.997), 길이 1.77 m. 경로 앞 1 m 의 각 점에서 가장 가까운 100 칸은 0.258 ~ 0.364 m 다.
  - `logs/diag_tunnel_global_costmap.txt`: `/robot6/global_costmap/costmap` 첫 메시지(stamp 04:24:44, transient local).
  - `logs/diag_tunnel_scan.txt`: `/robot6/scan` 한 장(frame `rplidar_link`, 640빔). 구간별 최소는 `rplidar_link` 각도 기준으로 −90 ~ −30° 0.320 m(−82°), 90 ~ 150° 0.451 m, 150 ~ 180° 0.452 m, −180 ~ −150° 0.545 m 다.
  - `logs/diag_tunnel_keepout_mask.txt`: `/rescue/passage_keepout/mask` 1×1(칠한 자리 없음).
- 복귀 실패 뒤(04:43:51) `logs/diag_return_local_costmap.txt`: 같은 창 가운데. 값 ≥ 99 칸 698개(100 = 136)이고, 0.3 m 안 100 칸 4개, 가장 가까운 100 칸 (−1.53, 1.89) 0.212 m, 가장 가까운 99 칸 (−1.41, 2.01) 0.042 m 다. `logs/diag_return_scan.txt` 도 같은 때다.
- 잡지 못한 것: RPP lookahead 점, `/robot6/rescue/low_obstacle_points`, depth_low_layer 만 칠한 칸(costmap 은 층을 합친 값만 저장했다). 시뮬레이션을 끈 뒤라 다시 잡을 수 없다.
- Nav2(robot_robot_a.log, 회차 전체): `detected collision ahead` 236, `Controller patience exceeded` 33, `Failed to make progress` 22, `spin failed` 15(`Collision Ahead - Exiting Spin` 15), `PassageSlow` 52. `depth_low_layer` 플러그인 사용.

## 넘김 뒤 TB-6 위치 끊김 (04:35:59.81 ~ 04:42:15.20, 375.4 s)

- `/rescue/robots/robot_a/pose` 마지막 메시지는 04:35:59.81, 다음은 04:42:15.20 이다(`track/pose_gap.txt` `GAP robot_a 375.4 s`). 이 사이 `mission_pose_status` 바뀜 기록은 없다.
- 04:39:15 `ros2 topic hz /rescue/robots/robot_a/pose`: 메시지 0. 발행자는 `/robot6/rescue_command_adapter` 1개였다.
- 같은 때(04:38:21 ~ 04:38:40):
  - `/robot6/amcl` lifecycle `active [3]`
  - `/robot6/amcl_pose` 는 stamp 04:36:33.789 한 건(latched)뿐
  - `/robot6/tf` 5 s 동안 map→odom 78개, odom→base_link 113개
  - `/robot6/scan` 32.7 Hz
- 실행기 설정(`logs/diag_tb6_pose_after_handover.txt`): `amcl_update_min_d 0.25`, `amcl_update_min_a 0.2`, `amcl_tf_topic /robot6/tf`, `localization_topic /robot6/amcl_pose`. 04:41:49 `ros2 param get` 값도 0.25·0.2 였다. 그 직전 `ros2 param list` 는 15 s 안에 답이 없었다.
- 실행기 줄: 04:36:33.37 `MAP_STATE … quality=UNKNOWN` 뒤 04:42:15.20 `quality=VALID` 까지 실행기 줄이 없다.
- 진단 1회(`logs/diag_tb6_nomotion.txt`): 04:42:13.10 `/robot6/request_nomotion_update` 호출, 04:42:15.47 응답.
  - 새 `amcl_pose` 는 04:42:15.209 에 왔다(stamp 04:42:15.165). 그 뒤 04:42:20.41·04:42:25.10 에도 왔다(복귀 작업으로 움직이던 때).
  - RobotPose 는 04:42:15.208 부터 `VALID` 였고, 그 뒤 36 s 동안 77건이다.
  - 같은 순간 조정기가 `SEND RETURN_HOME generation=8` 을 보냈다.
- TB-5 쪽: 04:35:25.26 `UNKNOWN` → 04:35:30.74 `VALID mission=(-0.414, 1.747)`, 5.5 s.

## TB-5 출발 정렬

- 04:35:35 `START_ALIGN_ALIGNED {"first_guess": [-0.3516, 1.8200, 3.1338], "jump_xy_m": 0.0964, "jump_yaw_rad": 0.0076, "map_sha256": "cf4364…", "match_points": 640, "match_ratio": 0.8266, "pose": [-0.4139, 1.7465, 3.1414], "sigma_xy_m": 0.0799, "sigma_yaw_rad": 0.0577, "status": "ALIGNED", "updates": 10}`.
- 8·9회차는 `START_ALIGN_LOW_MATCH`(match_ratio 0.6)였다. 이번이 처음 `ALIGNED` 다. 다시 시작(restart)은 하지 않았다.
- 정렬 순간 라이다: `logs/diag_tb5_scan_at_align.txt`, `diag_tb5_scan_at_align_2.txt`(`/robot5/scan` 640빔).
- 그때 정답 위치는 TB-5 (2.700, 0.721, 89.6°), TB-6 (2.948, 1.796)이다. TB-6 이 9회차(2.088, 1.552)와 다른 자리에 있었다.

## 지도 칸 수 (작전 요청, 지도 격자 안 칸)

| 지도 | 크기 | free | unknown | occupied |
| --- | --- | --- | --- | --- |
| 선포 직전(04:27:17, `maps/042717-before-declare`) | 49×68 | 2662 | 572 | 98 |
| 탐색 끝(04:35:2x live_map, `logs/map_counts_at_exploration_end.txt`) | 112×69 | 4022 | 3377 | 329 |
| 2차 고정 R2(`maps/043525-frozen-r2-…`) | 112×69 | 4020 | 3377 | 331 |

- unknown 은 격자 사각형 안의 205 칸이다. 격자가 커지면 바깥 빈 자리도 unknown 으로 센다.

## 그 밖의 자료

- 접촉(정답 위치, 몸 중심 최소): TB-6 ↔ TB-5 0.620 m(04:27:38.91), TB-6 ↔ 인형 1.940 m(04:27:30.42). TB-5 는 움직이지 않았다.
- 속도: TB-6 움직인 표본 102, 평균 0.046 m/s, 최고 0.141 m/s.
- 인형 대역 안 보낸 사유: `OUT_OF_RANGE` 5, `NOT_IN_FRONT` 2, `TF_STALE` 2, `NO_START_POSE`·`NO_TF`·`OUT_OF_VIEW` 각 1.
- RTF(30 s, 35표본): 중앙값 0.53, 최소 0.13, 최대 1.03, 0.5 미만 17. 부하 33 ~ 63(화면 녹화 켬).
- 장부 `overhead_passages`: `robot_a PASSABLE lowest_m 0.47 (-1.858, 2.135) map_revision 0 frames 3`.

## 9회차와 같은 것·다른 것

- 다른 것:
  - 첫 경계 e291 목표가 9회차 (−1.17, 1.17)에서 (−1.32, 2.10)으로 바뀌었다(`FRONTIER_GOAL_PAST_TIGHT`, #427). 9회차는 `COMPLETED`, 10회차는 0.035 m 앞에서 `EXPIRED`.
  - 둘째 경계부터 터널 1 너머(x < −2.7)를 목표로 잡았고, TB-6 은 입구에서 멈췄다. 7회차 멈춘 자리 (2.820, 1.703)와 0.16 m 떨어져 있다.
  - RETURN_HOME: 9회차 `GUARD_INVALID` → 10회차 넘김 중 `FRAME_REVISION_MISMATCH` 거절과 10 s 뒤 다시 보냄(#426). 그 뒤 위치 끊김 375 s 가 있었다.
  - TB-5 정렬: `LOW_MATCH` → `ALIGNED`.
  - 터널 판정: 0 → PASSABLE 1.
- 같은 것: 인형 관측 0, TB-5 출발 없음.

## 파일

| 파일 | 무엇 |
| --- | --- |
| `logs/*.log` | 회차 폴더 원문(색 코드 지움, 경로·PC 이름 바꿈, 줄인 줄 없음). `operator_manual.log` 는 손으로 한 것(watchdog reset·arm, 진단 no-motion) |
| `logs/diag_tunnel_*`, `logs/diag_return_*` | 터널 1 입구 멈춤 때와 복귀 실패 뒤 costmap·경로·라이다·keepout |
| `logs/diag_tb6_pose_after_handover.txt`, `logs/diag_tb6_nomotion.txt` | 넘김 뒤 실행기 줄·설정, 진단 1회 전후 수신 |
| `logs/diag_tb5_scan_at_align*.txt` | TB-5 정렬 순간 라이다 |
| `logs/map_counts_*.txt`, `logs/timesync_status_at_start.txt` | 지도 칸 수, 기동 직전 시간 동기 |
| `maps/` | 선포 직전 지도, SLAM 저장본(약 30 s 간격과 마지막), 1·2차 고정 지도 |
| `track/` | 정답 위치(0.5 s), RobotPose 끊김, RTF |
| `video/screen_gazebo_rviz_small.mp4`, `video/shot_*.png` | 화면 녹화 줄인 판(1280×720, 5 fps)과 녹화 시작(04:27:05) 기준 장면 6장: 12 s 선포, 155 s 입구 멈춤, 501 s 탐색 끝·고정, 511 s TB-5 정렬, 912 s 진단 뒤 복귀 보냄, 951 s 복귀 실패 |

## 참고: 가제보 실습 의견 (정확하지 않을 수 있음)

아래는 위 자료를 보고 짐작한 것이다. 확인하지 않은 해석이라 틀릴 수 있다.

- 넘김 뒤 끊김은 실행기가 넘김 뒤 새 `amcl_pose` 를 기다리는 모양으로 보인다. TF 는 그동안 나오고 있었다. 서 있는 TB-6 에 AMCL 이 latched 한 건만 낸 것과 겹친 것 같다. 순서가 런북 7.5 그대로라 현장에서도 날 수 있어 보인다.
- 입구 멈춤에서 costmap 이 칠한 100 칸은 흰 상자 모서리·동쪽 벽 자리뿐이었다. 그래서 `collision ahead` 가 무엇을 보고 나는지는 lookahead 점과 낮은 장애물 점을 잡아야 알 수 있을 것 같다.
- 첫 경계 작업은 목표 0.035 m 앞에서 yaw 를 맞추다 기한에 걸린 것으로 보인다.

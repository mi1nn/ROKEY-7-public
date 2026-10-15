# 가제보 실습 열셋째 회차 (full-1016-060431, 2026-10-16 06:04 ~ 06:14, sim1)

측정·로그 자료를 먼저 적는다. 가제보 실습의 해석은 맨 끝 「참고: 가제보 실습 의견」에만 둔다(재범 지시 2026-10-16).

- 빌드: `e6f21d6`(main, `src/`·`config/` diff 0줄). 12회차 빌드(`d01431b`) 뒤 들어간 것:
  - #435(TB-5 출발 정렬 지도 밖 빔 빼기). 이 회차 뒤 #452 로 되돌려졌다.
  - #437(운영자 대역 SLAM 끄기를 프로세스 그룹에)
  - #439(목표 위치에서 목표 방향으로 돌기)
  - #441(TB-6 복귀 실패 표시)
  - #442(C17: 고정 지도 단계 새 목표는 AMCL 첫 map→odom stamp 뒤에)
  - #443(sim drive_wall), #446·#449(관제 웹 로그인 유지·비밀 키 파일)
  - #450·#451(비켜서기·거절 반복 고침)은 이 빌드에 없다.
- world: sim1(`arena.sdf`).
- 현장 값과 다름: `SIM_HOME=odom`. 12회차와 같다.
- 시간 동기: 기동 직전 timesyncd spike=false 였다. `logs/timesync_status_at_start.txt` 는 Offset −21.7 ms, Delay 203.9 ms 다.
- 기동: run_full 이 TB-6 `ARMED` 까지 손 없이 끝냈다(06:06:29).
- 재난 선포: 재범 위임으로 가제보 실습이 관제 웹 `POST /commands kind=DECLARE_DISASTER`(admin) 를 보냈다. `accepted: true`, 조정기 `DISASTER_DECLARED` 06:07:26.
- 녹화: Gazebo 화면 클라이언트와 run_full 의 두 로봇 RViz 를 좌우로 녹화했다(06:07:23 ~ 06:14:24).
- 손으로 한 것: 06:11:33 frozen_localization robot_a 를 같은 명령으로 한 번 다시 띄웠다(런북 7.5.2, `logs/operator_manual.log`). 운영자 대역이 띄운 첫 번째가 실패했기 때문이다.
- 새 기록: `track/proc_load.txt` 에 30 s 마다 순간 CPU 상위 20 프로세스와 GPU 를 적었다(17장).

## 단계별 결과

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1. 언도킹으로 시작 | 통과 | run_full 이 두 대를 언도킹했다 |
| 2. 선포 즉시 TB-6 이 움직여 실습장을 훑음 | **실패** | 첫 경계 88칸 `COMPLETED`(06:07:56) → 남은 경계 `FRONTIER_GOAL_TIGHT` → 06:07:58 `EXPLORATION_COMPLETE frontiers_sent=1`. 12회차와 같다 |
| 3. 웹 지도에 터널·인형이 뜸 | 실패 | `overhead (PASSABLE\|BLOCKED)` 0, `FAKE_TARGET published` 0 |
| 4. 인형 발견 → TB-5 출발 | 실행 안 됨 | `NO_SURVIVOR_FOUND`. TB-5 출발 정렬 `ALIGNED`(06:08:02.17) |
| (복귀) | **안 나감** | 2차 고정 뒤 TB-6 위치가 06:08:29.54 부터 끝까지 없었다. `SEND RETURN_HOME` 0회 |

## 회차 표

| 시각 | 로그 |
| --- | --- |
| 06:07:26 ~ 06:07:28 | `FRONTIER_GOAL_PAST_TIGHT (-0.75, 1.35) -> (-0.95, 1.97) for 0.35 m clear and 0.6 m from TB-5` → `FRONTIER e291… cells=88 … goal=(-0.95, 1.97, 1.57)` → `SEND SEARCH_AREA … generation=0` |
| 06:07:56 | TB-6 `Reached the goal!` → `RESULT … COMPLETED` → `FRONTIER_GOAL_TIGHT cells=57 centroid=(-2.81, 2.29): no cell 0.35 m clear within 1.0 m, left out` |
| 06:07:58.22 | `EXPLORATION_COMPLETE reason=EXPLORATION_COMPLETE frontiers_sent=1` → `MAP_FREEZE 1`·`2`(sha `5ad990…`, 88×70) |
| 06:07:58.30 | `TRAFFIC_YIELD 7bed… from=(-0.91, 1.72) to=(-0.98, 1.72, -3.06) path_m=0.05 why=TRAFFIC_ROBOTS_TOO_CLOSE` → `TRAFFIC_HOLD` |
| 06:07:58 ~ 06:08:30.42 | `SEND SEARCH_AREA` generation 1 ~ 301, 모두 `ADMISSION -> False`. TB-6 거절 `FRAME_REVISION_MISMATCH … map handover in progress` 301 |
| 06:08:02.17 | TB-5 `START_ALIGN_ALIGNED`(아래 TB-5 절) |
| 06:08:29 | 운영자 대역 `런북 7.5 2차 고정 뒤 tb6_slam 끄기(launch 프로세스 그룹 1075549에 TERM, slam_toolbox PID 1075624)` |
| 06:08:29.54 | 마지막 TB-6 RobotPose(`track/pose_gap.txt` `SILENT robot_a > 2.0 s (last 06:08:29.54)`) |
| 06:08:41 | 운영자 대역 `/robot6/map 발행자 수 0(--no-daemon)`(손 없이 SLAM 이 꺼졌다, #437) |
| 06:08:57 | 운영자 대역 `frozen_localization robot_a 띄움` |
| 06:09:09 | `logs/frozen_loc_a_first_fail.log`: `[ERROR] [launch]: Caught exception in launch …: no pose on /rescue/map/frozen_pose/robot_a within 10.0 s (frozen pose: freeze the map first; start pose: map_freeze must see TB-6 SLAM); no AMCL start is made up` |
| 06:11:28 ~ 06:11:33 | 가제보 실습 확인: `ros2 topic echo --once … /rescue/map/frozen_pose/robot_a` 가 4.3 s 에 `(-0.914, 1.722)` 를 받았다(`logs/diag_frozen_pose_robot_a.txt`). 발행자 `rescue_map_freeze`, TRANSIENT_LOCAL |
| 06:11:33 | 가제보 실습이 손으로 같은 명령으로 frozen_localization robot_a 를 다시 띄웠다 |
| 06:11:37.13 | amcl `Setting pose (…): -0.914 1.722 1.756` |
| 06:11:39.14 ~ .27 | amcl `Received a 88 X 70 map` / `MAP_RELAYED … R2 -> /robot6/map` / TB-6 `MAP_HANDOVER robot_a R2 …` |
| 06:11:41.55 ~ 06:14:25.55 | TB-6 `AMCL_NOMOTION_REQUESTED robot_a: no amcl_pose since the map switch 2.3 s ago … 7.8 s … 12.8 s …` 약 5 s 간격 32회. `amcl_pose` 는 오지 않았다 |
| 06:13:34 | `logs/diag_amcl_no_laser_robot6.txt`(아래 절) → 06:14:24 녹화 끔 → 06:14:45 `stop_full.sh` |

## 넘김 뒤 TB-6 위치 없음

- 첫 frozen_localization(06:08:57, 운영자 대역)은 10 s 안에 `/rescue/map/frozen_pose/robot_a` 를 받지 못하고 06:09:09 에 끝났다.
  - map_freeze 는 06:07:58 에 `MAP_FROZEN … rev=2 final=True tb6 amcl start=(-0.914, 1.722, 1.756)` 를 냈다.
  - 06:11:28 에 같은 토픽을 `--qos-durability transient_local` 로 받는 데 4.3 s 걸렸다.
- 다시 띄운 frozen_localization(06:11:33)은 pose 를 받아 amcl 을 06:11:37 에 켰다. amcl 은 06:11:39.14 에 지도를 받았다.
  - 그 뒤 06:14:26 까지 amcl 로그에 `createLaserObject` 가 없다. 10·11·12회차에는 지도를 받은 직후 이 줄이 있었다.
  - `amcl_pose` 0, AMCL map→odom 0(`logs/diag_tf_handover_robot6.txt`, 06:08:20 ~ 06:12:00).
- 같은 때(`logs/diag_amcl_no_laser_robot6.txt`, 06:13:34):
  - `/robot6/amcl` 는 `active [3]` 이었다.
  - 파라미터: `scan_topic scan`, `odom_frame_id odom`, `base_frame_id base_link`, `global_frame_id map`, `transform_tolerance 1.0`, `tf_broadcast True`, `laser_model_type likelihood_field`.
  - `/robot6/scan` 구독자 중에 `amcl`(VOLATILE)이 있었다.
- `/robot6/scan`(`logs/diag_scan_stamp_robot6.txt`, 06:13:14 ~ 19): 5 s 에 26건, 받은 시각 − stamp +0.001 ~ +0.023 s, frame `rplidar_link`.
- odom→base_link(`diag_tf_handover_robot6.txt`): 1 s 마다 있었고, 받은 시각 − stamp 는 +0.001 ~ +0.005 s 다.
- TB-6 Nav2(robot_robot_a.log):
  - `Extrapolation Error` 347줄(06:08:40 ~ 06:14:26)
  - global_costmap `Message Filter dropping message: frame 'rplidar_link' … earlier than all the data in the transform cache` 146줄(06:06:07 ~ 06:14:25)

## 넘김 전 반복 송신

- `SEND SEARCH_AREA` 는 302건이다(generation 0 ~ 301, 06:07:28.12 ~ 06:08:30.42). `ADMISSION` 은 True 1, False 301 이다.
- 반복은 RobotPose 가 끊긴 06:08:29.54 직후 멈췄다. `TRAFFIC_YIELD` 는 1회였다.

## TB-5 출발 정렬 (#435 빌드)

- 06:08:02.17 `START_ALIGN_ALIGNED {"first_guess": [-0.3533, 1.8200, 3.1360], "jump_xy_m": 0.0867, "jump_yaw_rad": 0.0067, "map_sha256": "5ad990…", "match_points": 490, …}`. 12회차는 `match_points 625` 였다.
- #435 는 이 회차 뒤 #452 로 되돌려졌다.

## 부하 (프로세스별, 처음 기록)

`track/proc_load.txt`, 06:06:52 ~ 06:14:57, 30 s 마다 `top` 한 장, 17장(마지막 한 장은 `stop_full.sh` 뒤). 아래는 장마다 같은 이름의 %CPU 를 더한 값의 평균이다(100 = 코어 하나).

| 프로세스 | 평균 %CPU | 최대 |
| --- | --- | --- |
| firefox(이 PC 의 브라우저, 회차 밖) | 163 | 200 |
| `gz sim arena.sdf -r`(Gazebo 서버) | 155 | 192 |
| `task_input`(두 실행기) | 72 | 108 |
| `wall_clock_relay.py` | 67 | 92 |
| `parameter_bridge` | 59 | 100 |
| `gz sim -g`(녹화용 Gazebo 화면) | 58 | 83 |
| `rviz2`(두 로봇 RViz) | 43 | 100 |
| `joint_state_publisher` | 26 | 58 |
| `ffmpeg`(녹화) | 26 | 67 |
| `strip_sensors.py` | 22 | 42 |

- GPU: 회차 중 34 %, 1128 MiB.
- 시스템 부하 28 ~ 60. RTF(30 s, 16표본)는 중앙값 1.00, 최소 0.09, 최대 1.30, 0.5 미만 6 이다.

## 그 밖의 자료

- 정답 위치: TB-6 이동 거리 1.81 m, 마지막 (2.602, 1.282, 10.6°). TB-6 ↔ TB-5 최소 0.568 m(06:07:54.25).
- RobotPose: TB-5 06:07:58.37 `UNKNOWN` → 06:08:02.38 `VALID mission=(-0.433, 1.785)`.
- 선포 직전 지도 `maps/060725-before-declare`(50×70): free 2537, unknown 661, occupied 302.

## 12회차와 같은 것·다른 것

- 같은 것: 첫 경계 88칸 `COMPLETED`, 남은 경계 `TIGHT` 로 `frontiers_sent=1`, 고정 직후 `TRAFFIC_YIELD` 와 SEARCH_AREA 반복 거절, TB-5 `ALIGNED`.
- 다른 것:
  - SLAM 끄기: 12회차는 손으로 SIGINT 했다. 13회차는 운영자 대역이 손 없이 껐다(#437).
  - frozen_localization: 12회차는 한 번에 됐다. 13회차는 첫 번째가 frozen pose 를 10 s 안에 못 받아 실패했다.
  - 넘김 뒤: 12회차는 `AMCL_NOMOTION_REQUESTED` 1회 뒤 `amcl_pose` 가 왔다. 13회차는 32회 불러도 오지 않았다(`createLaserObject` 없음).
  - 반복 송신은 12회차 950건, 13회차 301건이다(13회차는 RobotPose 가 끊기며 멈춤).

## 파일

| 파일 | 무엇 |
| --- | --- |
| `logs/*.log` | 회차 폴더 원문(색 코드 지움, 경로·PC 이름 바꿈, 줄인 줄 없음). `frozen_loc_a_first_fail.log` 는 운영자 대역이 띄운 첫 frozen_localization, `frozen_loc_a.log` 는 다시 띄운 것 |
| `logs/diag_frozen_pose_robot_a.txt` | 06:11:28 frozen pose 토픽 받기 |
| `logs/diag_amcl_no_laser_robot6.txt`, `logs/diag_scan_stamp_robot6.txt` | AMCL 상태·파라미터·scan 구독, scan 받은 시각 − stamp |
| `logs/diag_tf_handover_robot6.txt` | 06:08:20 ~ 06:12:00 map→odom·amcl_pose·odom→base_link 받은 시각과 stamp |
| `logs/diag_c7_*`, `logs/diag_c8_tb5_nav2.txt` | 운영자 대역이 남긴 자료 |
| `logs/diag_tb5_scan_at_align*.txt`, `logs/map_counts_before_declare.txt`, `logs/timesync_status_at_start.txt` | TB-5 정렬 순간 라이다, 선포 직전 지도 칸 수, 시간 동기 |
| `maps/` | 선포 직전 지도, 1·2차 고정 지도 |
| `track/` | 정답 위치, RobotPose, RTF, 프로세스별 부하(`proc_load.txt`) |
| `video/screen_gazebo_rviz_small.mp4`, `video/shot_*.png` | 화면 녹화 줄인 판과 녹화 시작(06:07:23) 기준 장면 6장 |

## 참고: 가제보 실습 의견 (정확하지 않을 수 있음)

아래는 위 자료를 보고 짐작한 것이다. 확인하지 않은 해석이라 틀릴 수 있다.

- 넘김 뒤 AMCL 이 라이다를 한 번도 처리하지 않은 것이 TB-6 위치가 없었던 직접 원인으로 보인다. 왜 처리하지 않았는지는 이 자료로 모른다. AMCL 쪽 message filter 로그를 debug 로 보면 알 수 있을 것 같다.
- 첫 frozen_localization 실패는 부하가 높을 때(부하 50 ~ 60) 10 s 기다림이 모자란 것일 수 있다. 같은 토픽을 받는 데 4.3 s 가 걸렸다.
- 부하는 Gazebo 서버 다음으로 이 PC 의 브라우저가 컸다. 회차 밖 프로세스라 RTF 비교 때 따로 봐야 할 것 같다.

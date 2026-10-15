# 가제보 실습 여덟째 회차 (full-1016-031028, 2026-10-16 03:10 ~ 03:21, sim1)

측정·로그 자료를 먼저 적는다. 가제보 실습의 해석은 맨 끝 「참고: 가제보 실습 의견」에만 둔다(재범 지시 2026-10-16).

- 빌드: `9a3e274`(main 커밋). 7회차 빌드(`909097a`) 뒤 들어간 것은 #407(기록), #408(RETURN_HOME frame), #409(시뮬레이션 odom 회전 보정), #410(런북)이다. run_full 첫 줄의 `src 는 main 과 diff 112줄` 은 회차 시작 때 main 이 이미 `b84aaf2`(#411)였기 때문이다. 작전 지시로 `9a3e274` 로 돌렸다.
- world: sim1(`arena.sdf` 마지막 변경 `08dfbe4`), 흰 상자 없음.
- 현장 값과 다름: `robots.robot_b.initial_pose_xy_yaw` 시뮬레이션 사본 `[0.013, -1.82, 0.0007]`(현장 `[0.0, -1.86, 0.0]`), `home_positions` `robot_a=-0.35297,-3.5e-11,3.13381 robot_b=-0.352,1.82,3.1345`(`SIM_HOME=odom`).
- 언도킹 뒤 TB-6 odom 방향 3.1338 rad(179.6°)이고, 정답 위치 방향 차이로 본 실제 회전은 179.6° 다. 5~7회차는 odom −2.949 rad(191.0°)였다.
- 재난 선포: 재범 위임으로 가제보 실습이 관제 웹 `POST /commands kind=DECLARE_DISASTER`(admin), 03:13:09.24 요청 → `accepted: true`. 조정기 `DISASTER_DECLARED` 03:13:09.57.
- 화면 녹화(Gazebo 화면 클라이언트 + 두 로봇 RViz + ffmpeg) 켬, 03:12:55~03:20:30.

## 단계별 결과

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1. 언도킹으로 시작 | 통과 | `언도킹 robot6` 03:11:08, `언도킹 robot5` 03:11:24 |
| 2. 선포 즉시 TB-6 이 움직여 실습장을 훑음 | **통과에 가까움** | 보낸 경계 4개 모두 `COMPLETED`(03:14:48, 03:16:46, 03:17:01, 03:17:25). 터널 1 통과(정답 위치 03:16:52 (3.045, 2.336)). 메인 구역 최대 y 4.200. 이동 거리 5.46 m. 03:17:26 `EXPLORATION_COMPLETE reason=EXPLORATION_COMPLETE frontiers_sent=4`(남은 경계 4개 모두 `small`). 탐색 끝 지도 free 칸 6,532 / 7,642 = 85.5%, 그중 메인 구역(y > 2.46) 3,569 |
| 3. 웹 지도에 터널·인형이 뜸 | **일부** | 터널 1: `overhead PASSABLE lowest=0.473 map=(-1.85, 2.10)`(03:16:46), 장부 `overhead_passages` 1행. 터널 2·인형: 없음(`FAKE_TARGET published` 0) |
| 4. 인형 발견 → TB-5 출발 | 실행 안 됨 | `NO_SURVIVOR_FOUND`. TB-5 출발 정렬 실패 2회로 arm 시도 없음 |

## 회차 표

| 시각 | 로그 |
| --- | --- |
| 03:13:09.57 | `FRONTIER_GOAL_MOVED (-1.37, 1.09) -> (-1.22, 1.10) for 0.35 m clear` → `FRONTIER e291… cells=13 centroid=(-1.824, 2.204) path_m=2.85 cost=-0.22 cut=True goal=(-1.22, 1.10, 1.57)` → `SEND … generation=0` |
| 03:14:48.65 | `RESULT … COMPLETED` → `FRONTIER_GOAL_CUT e291… reached 1.5 m along 2.85 m: the frontier is not tried yet` → `FRONTIER_GOAL_TIGHT (-1.62, 2.30): no cell 0.35 m clear near it, kept at 0.2 m` → `FRONTIER a116… cells=12 centroid=(-1.841, 2.225) path_m=1.39 cost=-0.39 cut=False goal=(-1.62, 2.30, 3.14)` |
| 03:16:46.15 | `overhead PASSABLE lowest=0.473 map=(-1.85, 2.10) … published` / `RESULT … COMPLETED` → `FRONTIER_GOAL_MOVED (-2.83, 1.74) -> (-2.87, 1.85)` → `FRONTIER f0bfa088… cells=61 centroid=(-3.618, 1.000) path_m=2.71 cost=-2.64 cut=True goal=(-2.87, 1.85, -2.36)` |
| 03:17:01.15 | `RESULT … COMPLETED` → `FRONTIER_GOAL_CUT f0bfa088… reached 1.5 m along 2.71 m` → `FRONTIER 9c321c01… cells=23 centroid=(-4.569, 1.214) path_m=2.26 cost=-0.81 cut=True goal=(-3.81, 1.00, 3.14)` |
| 03:17:25.15 | `RESULT … COMPLETED` → `FRONTIER_GOAL_CUT 9c321c01… reached 1.5 m along 2.26 m` |
| 03:17:25.44~03:17:26.82 | `FRONTIER none on map update 317~319: frontiers=4 small=4 no_goal=0 tried=0 no_path=0 too_close=0 \| SMALL cells=8 centroid=(-1.84, 0.47); SMALL cells=6 centroid=(-4.39, 1.60); SMALL cells=2 …` → `EXPLORATION_COMPLETE … frontiers_sent=4` |
| 03:17:26.85 | `MAP_FROZEN 1`·`2`(sha `51230b…`) → `SEND RETURN_HOME … generation=4` → `ADMISSION -> False` → TB-6 `ExecuteTask rejected: INVALID_INPUT: INVALID_INPUT [rescue_robot.INVALID_INPUT]: mission frame` → `TASK_REFUSED`. `mission frame` 거절 합계 4회 |
| 03:17:26.95~03:17:47 | TB-5 `START_ALIGN_FAILED START_ALIGN_NOT_CONVERGED {"first_guess": [-0.3518, 1.8200, 3.1345], "last_pose": [-0.3919, 1.7928, 3.1361], "last_sigma_xy_m": 0.140, "not_settled": "SPREAD_ABOVE_ACCEPT", "poses_received": 21, "settle_spread_xy_m": 0.0074, "updates": 20}` |
| 03:17:41 / 03:17:52 | `ros2 topic hz`(10 s): `/robot5/rescue/watchdog_heartbeat` 19.988 Hz, `/robot6/rescue/watchdog_heartbeat` 19.999 Hz |
| 03:17:59~03:18:42 | 런북 7.5(운영자 대역): TERM → 03:18:16 발행자 0 → 03:18:34 `frozen_localization robot_a` → 03:18:42 `MAP_RELAYED … R2 -> /robot6/map`. TB-6 `MAP_HANDOVER robot_a R2 frozen grid on its map topic: SLAM pose until now, AMCL from here` |
| 03:18:17 | 손으로 런북 7단계 `rescue_start_align/restart` → `success=True` |
| 03:18:42 | TB-5 `START_ALIGN_FAILED START_ALIGN_LOW_MATCH … "jump_xy_m": 0.0285, "sigma_xy_m": 0.097, "updates": 18` |
| 03:18:42.73 | `TRAFFIC_HOLD SCOUT TRAFFIC_SCOUT_POSE_UNKNOWN` |
| 03:20:30~03:21:03 | 녹화 끔, `stop_full.sh`(이 구간의 Traceback 은 종료 신호 뒤) |

## RobotPose·접촉·속도·RTF·인형

- RobotPose: TB-6 은 03:17:27.33 → 03:17:31.83 의 4.5 s 끊김 한 번(`STATUS VALID -> UNKNOWN -> VALID mission=(-3.830, 0.995)`)이다. 7회차는 22.5 s + 40.3 s 였다. TB-5 는 03:17:26.95 `UNKNOWN` 뒤 `VALID` 줄이 없다.
- 접촉(정답 위치, 몸 중심 최소): TB-6 도크 0.509 m(출발), TB-5 도크 1.286 m, TB-5 0.779 m(03:15:05), 인형 (1.20, 2.87) 1.294 m(03:17:12).
- 인형 쪽: TB-6 이 인형 2.0 m 안에 있던 정답 위치 표본 731개, 그중 인형 방위가 TB-6 진행 방향 ±35° 안이던 표본 29개(첫 03:14:09, 1.50 m). 인형 대역 안 보낸 사유: `NOT_IN_FRONT` 4, `OUT_OF_RANGE` 3, `OUT_OF_VIEW` 3, `NO_CAMERA_INFO`·`NO_MAP`·`NO_TF`·`OCCLUDED`·`TF_STALE` 각 1.
- Nav2(robot_robot_a.log): `PassageSlow` 19, `Failed to make progress` 15, `detected collision ahead` 0, `spin failed` 1.
- 속도: 움직인 표본 271/513, 평균 0.040 m/s, 최고 0.139 m/s.
- RTF(30 s, 16표본): 중앙값 0.85, 최소 0.14, 최대 1.00, 0.5 미만 7개. 부하 24~64(화면 녹화 켬).

## 7회차와 같은 것·다른 것

- 다른 것:
  - TB-6 이 터널 1 입구에서 멈추지 않고 통과했다. 정답 위치 최대 y 1.703 → 4.200, `detected collision ahead` 114 → 0 이다.
  - 탐색 끝: 7회차는 `EXPLORATION_TIMEOUT frontiers_sent=6`, 8회차는 `EXPLORATION_COMPLETE frontiers_sent=4`(small 4) 였다.
  - 지도 free 칸이 48.0% → 85.5% 로 늘었다.
  - RETURN_HOME 거절 이유가 `native frame` → `mission frame` 으로 바뀌었다.
  - TB-5 정렬: 7회차 `ALIGNED jump 0.034 m` → 8회차 `NOT_CONVERGED`, 재시작 뒤 `LOW_MATCH` 다.
  - TB-6 7.5 끊김 22.5 s + 40.3 s → 4.5 s.
- 같은 것: 터널 1 `PASSABLE lowest=0.473`, 인형 관측 0.

## 파일

| 파일 | 무엇 |
| --- | --- |
| `logs/*.log` | 회차 폴더 원문(색 코드 지움, 경로·PC 이름 바꿈, 줄인 줄 없음) |
| `logs/diag_watchdog_hz.txt` | watchdog heartbeat 토픽 10 s 측정 |
| `logs/operator_manual.log` | 손으로 한 `rescue_start_align/restart` |
| `maps/<시각>-live-r0-…`, `maps/timeout-…`, `maps/frozen-r2-…` | 선포 직전~탐색 끝 SLAM 저장본(시각 순서), 탐색 끝 마지막 저장본, 2차 고정 지도 |
| `track/` | 정답 위치 0.5 s, RobotPose 끊김, RTF |
| `video/screen_gazebo_rviz_small.mp4` | 화면 녹화(왼쪽 Gazebo, 오른쪽 TB-5·TB-6 RViz), 1280×720·5 fps 줄인 판. 원본 10.7 MB 는 시뮬레이션 PC 회차 폴더 |
| `video/shot_*.png` | 녹화 시작(03:12:55) 기준 14 s(선포), 231 s(터널 판정), 237 s(터널 안), 257 s(메인 구역), 271 s(탐색 끝), 347 s(TB-5 정렬 실패) |

## 참고: 가제보 실습 의견 (정확하지 않을 수 있음)

아래는 위 자료를 보고 짐작한 것이다. 확인하지 않은 해석이라 틀릴 수 있다.

- 터널 1 통과와 지도 정렬은 #409(odom 회전 보정)로 지도가 축과 나란해진 효과로 보인다. 7회차의 입구 멈춤이 지도 기울기 탓이었는지, 다른 차이 탓이었는지는 이 두 회차만으로 가리기 어렵다.
- TB-5 정렬 실패는 첫 추측이 정답과 거의 같았는데도 수렴하지 않았다. 고정 지도 품질이나 TB-5 주변 특징 부족과 관계가 있을 수 있다.
- 인형 관측이 없던 것은 인형이 칸막이·터널 2 덮개 뒤쪽 방(터널 2 틈 0.30 m)에 있어 TB-6 카메라 정면에 잘 들지 않은 탓일 수 있다. 시뮬레이션 인형 대역의 판정 조건도 함께 봐야 한다.
- `mission frame` 거절은 #408 뒤에도 RETURN_HOME 좌표계가 실행기 기대와 다르다는 뜻으로 보인다.

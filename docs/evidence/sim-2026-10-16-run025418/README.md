# 가제보 실습 일곱째 회차 (full-1016-025418, 2026-10-16 02:54 ~ 03:09, sim1)

측정·로그 자료를 먼저 적는다. 가제보 실습의 해석은 맨 끝 「참고: 가제보 실습 의견」에만 둔다(재범 지시 2026-10-16).

- 빌드: `909097a`(main, `src/`·`config/` diff 0줄). 6회차 빌드(`4850a8f`) 뒤 들어간 것: #404(경계 크기 비용·잘린 목표 도착은 방문으로), #405(2차 고정 뒤 TB-6 위치를 SLAM 끌 때까지 이어감), #406(진입 금지 푸는 서비스). #408(RETURN_HOME frame)·#409(odom 회전 보정)는 없다.
- world: sim1(`arena.sdf` 마지막 변경 `08dfbe4`), 흰 상자 없음.
- 현장 값과 다름: `robots.robot_b.initial_pose_xy_yaw` 시뮬레이션 사본 `[0.016, -1.82, 0.0021]`(현장 `[0.0, -1.86, 0.0]`), `home_positions` `robot_a=-0.3528,-3.2e-11,-2.9486 robot_b=-0.717,1.783,-2.9466`(`SIM_HOME=odom`).
- 재난 선포: 재범 위임으로 가제보 실습이 관제 웹 `POST /commands kind=DECLARE_DISASTER`(admin), 02:57:08.90 요청 → `accepted: true`. 조정기 `DISASTER_DECLARED` 02:57:09.25.
- 이 회차부터 녹화를 켰다. Gazebo 화면 클라이언트(`gz sim -g`, 같은 partition)와 두 로봇을 함께 띄운 RViz 를 가상 화면에 띄워 ffmpeg 로 녹화했다(02:56:46~03:09:33). RViz 용 보기 전용 TF·라이다 중계(`tb6/`·`tb5/` 접두)는 저장소 밖 도구이고 어떤 노드에도 값을 주지 않는다. 화면 클라이언트를 켠 만큼 부하가 늘었다(아래 RTF).

## 단계별 결과

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1. 언도킹으로 시작 | 통과 | `언도킹 robot6` 02:54:54, `언도킹 robot5` 02:55:10 |
| 2. 선포 즉시 TB-6 이 움직여 실습장을 훑음 | **실패** | 첫 경계 작업 `COMPLETED`(02:57:32). 02:58:35 부터 터널 1 입구 정답 위치 (2.820, 1.703)에서 끝까지 자리 이동 없음(방향만 −176~180° 사이로 바뀜). 그 뒤 작업 4개 `EXPIRED`, 1개 `FAILED GUARD_INVALID`. 03:07:09 `EXPLORATION_TIMEOUT frontiers_sent=6`. 이동 거리 2.27 m. 시간 초과 지도 free 칸 3,669(실습장 안) / 7,642 = 48.0%. 메인 구역(y > 2.46) 정답 위치 표본 0, 도달한 최대 y 1.703 |
| 3. 웹 지도에 터널·인형이 뜸 | **일부** | 터널 1: 02:58:22 `overhead PASSABLE lowest=0.473 map=(-2.27, 1.86) … published`, 장부 `overhead_passages` 1행. world 의 터널 1 덮개 틈 0.47 m 와 차이 0.003 m. 터널 2·인형: 없음(`FAKE_TARGET published` 0) |
| 4. 인형 발견 → TB-5 출발 | 실행 안 됨 | `NO_SURVIVOR_FOUND`. TB-5 arm `success=False`(`TASK_ARM refused NAV_NOT_READY missing=['watchdog'] … sources=['watchdog_heartbeat(719.5 s)']`) |

## 회차 표 (조정기·실행기)

| 시각 | 로그 |
| --- | --- |
| 02:57:09.25 | `FRONTIER e291… cells=31 centroid=(-2.176, 1.898) path_m=1.84 cost=-1.27 cut=True goal=(-1.40, 1.08, 2.36)` → 02:57:11 `SEND … generation=0` |
| 02:57:32.84 | `RESULT … COMPLETED` → `FRONTIER_GOAL_CUT e291… reached 1.5 m along 1.84 m: the frontier is not tried yet` |
| 02:57:33.02 | `FRONTIER a116… cells=11 centroid=(-2.261, 1.866) path_m=1.46 cost=-0.33 cut=False goal=(-2.06, 1.97, 3.14)` → `SEND … generation=1` |
| 02:58:22.66 | `overhead PASSABLE lowest=0.473 map=(-2.27, 1.86) … published`(overhead_robot_a.log) |
| 02:58:32 ~ | `RegulatedPurePursuitController detected collision ahead!`(회차 합계 114), `Failed to make progress`(33) |
| 02:59:35.84 | `RESULT … EXPIRED` → `FRONTIER_RETRY a116… failed 1/2` → `FRONTIER f0bfa088… cells=59 centroid=(-3.879, 1.045) cost=-2.56 cut=True goal=(-2.97, 1.58)` |
| 03:01:41 / 03:03:47 / 03:05:53 | `EXPIRED` → `FRONTIER 9c321c01… cells=38 cost=-1.60` / `c8a70d57… cells=17 cost=-0.28` / `54297c07…`(같은 중심), 목표는 모두 (−2.97, 1.58) |
| 03:07:09.20 | `EXPLORATION_COMPLETE reason=EXPLORATION_TIMEOUT frontiers_sent=6` → 03:07:11 `RESULT … FAILED reason=GUARD_INVALID` → `FRONTIER_GAVE_UP 54297c07… after 2 failed attempts` |
| 03:07:19.42 | `MAP_FROZEN 1`·`2`(sha `bc1b19…`) → `SEND RETURN_HOME … generation=6` → `ADMISSION -> False` → `TASK_REFUSED`. 그 뒤 generation 14 까지 `native frame` 거절 9회 |
| 03:07:24~29 | TB-5 `START_ALIGN_ALIGNED … "jump_xy_m": 0.034` → `MAP_STATE robot_b VALID`(03:07:24.80) |
| 03:08:44 | `MAP_RELAYED /rescue/map/frozen R2 -> /robot6/map` |
| 03:08:43 | 운영자 대역 `robot5 task_arm: success=False` / 실행기 `TASK_ARM refused NAV_NOT_READY missing=['watchdog'] motion=STOPPED sources=['watchdog_heartbeat(719.5 s)']` |

## 터널 1 입구에서 멈춘 자리 (작전 요청 자료)

- 정답 위치: 02:58:35 ~ 03:07:09 (2.820, 1.703), 방향 −176~180° 사이를 오감.
- 몸 중심 거리:
  - 흰 상자 칸막이 남동 모서리 (2.75, 2.01)까지 0.315 m. Nav2 반지름 0.189 m 를 빼면 몸 가장자리 0.126 m 다.
  - 동쪽 벽 x=3.43 까지 0.61 m 다.
  - 칸막이 동쪽 면 x=2.75 와 동쪽 벽 사이 틈은 0.68 m 다.
  - 터널 1 덮개 판(world `tunnel1_lintel`, 0.81×0.01 m, 아래 틈 0.47 m)은 y=2.23 에 있고 몸 중심에서 0.53 m 다.
- 원자료(03:09:00 즈음, TB-6 은 같은 자리; 7.5 뒤라 TB-6 Nav2 지도는 고정 지도):
  - `logs/diag_tunnel_local_costmap.txt`: `/robot6/local_costmap/costmap`, frame `odom`, 0.06 m, 50×50, 값 ≥ 99 칸 729개의 좌표
  - `logs/diag_tunnel_global_costmap.txt`: `/robot6/global_costmap/costmap`, frame `map`, 0.05 m, 62×75
  - `logs/diag_tunnel_scan.txt`: `/robot6/scan` 한 장(frame `rplidar_link`)
  - `logs/diag_tunnel_plan.txt`: `/robot6/plan` 12 s 동안 메시지 없음(그 시각 진행 중 작업 없음)
  - `logs/diag_tunnel_keepout_mask.txt`: `/rescue/passage_keepout/mask` 1×1(칠해진 자리 없음). 로그 `PASSAGE_KEEPOUT 0 place(s) kept out: []`
- 라이다 높이는 0.23 m(`passage.overhead.lidar_plane_m`)이고 덮개 판 아래 끝은 0.47 m 다.

## RobotPose·접촉·속도·RTF

- RobotPose: TB-6 끊김은 03:07:20.18 → 03:07:42.68 의 22.5 s(`STATUS VALID -> UNKNOWN -> VALID mission=(-1.694, 1.707)`), 03:08:04.68 → 03:08:44.96 의 40.3 s. TB-5 는 03:07:24.80 `VALID` 뒤 끊김 0.
- 접촉(정답 위치, 몸 중심 최소): TB-6 도크 0.509 m(출발), TB-5 도크 1.274 m, TB-5 0.769 m(02:57:50), 인형 1.648 m.
- Nav2(robot_robot_a.log): `PassageSlow` 50, `Failed to make progress` 33, `detected collision ahead` 114, `spin failed` 0.
- 속도: 움직인 표본 98/1201, 평균 0.046 m/s, 최고 0.140 m/s.
- RTF(30 s, 26표본): 중앙값 0.62, 최소 0.06, 최대 1.50, 0.5 미만 12개. 부하 27~55. 화면 클라이언트·RViz·ffmpeg 를 켠 회차다.
- 인형 대역 안 보낸 사유: `OUT_OF_RANGE` 4, `NOT_IN_FRONT` 2, `NO_START_POSE`·`NO_TF`·`OUT_OF_VIEW`·`TF_STALE` 각 1.

## 6회차와 같은 것·다른 것

- 다른 것:
  - 첫 경계: 6회차는 14칸 (0.235, −0.509) 남서 구석이었고, 7회차는 31칸 (−2.176, 1.898) 터널 1 입구, `cost=-1.27 cut=True` 다.
  - 탐색 끝: 6회차는 `EXPLORATION_COMPLETE frontiers_sent=2`(02:40:57)였고, 7회차는 `FRONTIER_GOAL_CUT … not tried yet` 뒤 6개 경계를 보내고 시간 초과로 끝났다.
  - 터널 판정 0회 → `PASSABLE 0.473` 1회.
  - TB-6 최대 y 1.39 → 1.703.
  - 7.5 끊김 71.1 s → 22.5 s + 40.3 s.
- 같은 것: RETURN_HOME `native frame` 거절(#408 이 이 빌드에 없음).

## 파일

| 파일 | 무엇 |
| --- | --- |
| `logs/*.log` | 회차 폴더 원문(색 코드 지움, 경로·PC 이름 바꿈, 줄인 줄 없음). `overhead_robot_a.log` 포함 |
| `logs/diag_tunnel_*.txt` | 위 「터널 1 입구」 원자료 |
| `logs/diag_c*.txt` | 운영자 대역 7.5 자료, TB-5 Nav2 lifecycle |
| `maps/<시각>-live-r0-…` | 선포 직전부터 탐색 끝까지 SLAM 저장본(시각 순서) |
| `maps/timeout-…`, `maps/frozen-r2-…` | 탐색 끝 마지막 저장본, 2차 고정 지도 |
| `track/` | 정답 위치 0.5 s, RobotPose 끊김, RTF |
| `video/screen_gazebo_rviz_small.mp4` | 화면 녹화(왼쪽 Gazebo, 오른쪽 TB-5·TB-6 RViz), 1280×720·5 fps 로 줄인 판. 원본(1920×1080·10 fps, 33 MB)은 시뮬레이션 PC 회차 폴더 |
| `video/shot_*.png` | 녹화 시작(02:56:46) 기준 23 s(선포), 96 s(터널 판정), 134 s(입구 멈춤), 634 s(고정·복귀) 장면 |

## 참고: 가제보 실습 의견 (정확하지 않을 수 있음)

아래는 위 자료를 보고 짐작한 것이다. 확인하지 않은 해석이라 틀릴 수 있다.

- 첫 경계 선택과 C11 해소는 #404 효과로 보인다. 터널 판정 0.473 은 #398 의 비전 판정이 처음 돈 것으로 보인다.
- 입구에서 멈춘 것은 칸막이 남동 모서리가 몸 가장자리 0.13 m 안에 있어 RPP 충돌 판정이 앞길을 막은 것 같다. 틈 0.68 m 는 몸(0.38 m)보다 넓지만 비용지도 여유를 더하면 좁을 수 있다. 실물 실습장 폭이 같다면 실물에서도 날 수 있다고 본다. 시뮬레이션 지도가 11° 기운 영향(#409 전)은 가리지 못했다.
- TB-5 `watchdog_heartbeat(719.5 s)` 거절은 이 회차에서 처음 봤다. 화면 녹화로 부하가 늘어난 것과 관계가 있을 수도 없을 수도 있다.

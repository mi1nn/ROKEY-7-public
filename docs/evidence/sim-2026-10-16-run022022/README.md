# 가제보 실습 다섯째 회차 (full-1016-022022, 2026-10-16 02:20 ~ 02:28, sim1)

이 문서는 측정·로그 자료를 먼저 적는다. 가제보 실습의 해석과 의견은 맨 끝 「참고: 가제보 실습 의견」에만 둔다(재범 지시 2026-10-16).

- 빌드: `be4053b`(main 그대로, `src/`·`config/` diff 0줄)다. 시뮬레이션 실행 코드도 main 에 있다(#393).
  - main 에 든 것: #398(깊이 장애물 정지 빼고 비전 터널 판정 BLOCKED 자리만 Nav2 진입 금지), #395(도크 IR 광선 표시 끄기), #400(진입 금지 푸는 서비스), 그리고 4회차에 들어간 #379·#385·#387·#389·#390·#391·#392
- world: sim1(arena, `config/sim/worlds/arena.sdf` 마지막 변경 `08dfbe4`)이다. 흰 상자는 빠졌고 칸막이는 그대로다. 벽은 0.018 m 이어진 상자다.
- 고치기 전: TB-5 출발값 계산(#401 머지 전)이다.
- 현장 값과 다른 시뮬레이션 사본 값: `robots.robot_b.initial_pose_xy_yaw` 현장 `[0.0, -1.86, 0.0]`, 시뮬레이션 `[-0.348, -1.786, 0.0022]` 이다. `home_positions` 는 현장처럼 비워 두었다. 다음 회차부터는 작전 결정 (a)로 `SIM_HOME=odom` 을 켠다.
- 재난 선포: 재범, 휴대폰 웹, 02:23:02.71.
- Gazebo 는 이 회차 하나뿐이었다.
- 실습장 좌표는 확인용 정답 위치(`track/gt_track.txt`)로만 썼다. 지도 변환은 4회차와 같은 회전 −101.3° 다.

## 단계별 결과

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1. TB-5·TB-6 언도킹으로 시작 | 통과 | `언도킹 robot6` 02:21:07, `언도킹 robot5` 02:21:25 |
| 2. 선포 순간 TB-6 이 곧바로 움직여 실습장을 훑음 | **실패** | 회전 시작 02:23:06(선포 + 3.4 s), 자리 이동 시작 02:23:14. 보낸 경계 2개 모두 `COMPLETED` 다. 02:25:18 `EXPLORATION_COMPLETE`. 이동 거리 1.49 m. 정지 구간: 02:23:58~02:24:00, 02:25:16~끝. 시간 초과 지도 free 칸 2,669 / 실습장 칸 7,642 = 34.9%. 메인 구역(y > 2.46)에 들어간 정답 위치 표본 0 |
| 3. 웹 지도에 터널·인형이 뜸 | **실패** | 장부 `overhead_passages` 0행, `PASSAGE_KEEPOUT 0 place(s)` 1회, 인형 대역 `FAKE_TARGET published` 0회 |
| 4. 인형 발견 순간 TB-5 출발 | 실행 안 됨 | `NO_SURVIVOR_FOUND`(02:25:18). TB-5 `ARMED` 02:26:39 |

## 회차 표

| 시각 | 무엇 | 로그 | 비고 |
| --- | --- | --- | --- |
| 02:20:22 | 시작 | `빌드 be4053b(src 는 main 과 diff 0줄), 관제 웹 빌드 be4053b` | |
| 02:21:53 | 진입 금지 | `PASSAGE_KEEPOUT 0 place(s) kept out: []` | 로봇 launch 줄 `+ no depth stop + tunnel keepout` |
| 02:22:30 | 준비 끝 | `TB-6 ARMED` | |
| 02:23:02.71 | 재난 선포 | `FRONTIER_GOAL_MOVED (-0.14, -0.53) -> (-0.29, -0.43) for 0.35 m clear` → `FRONTIER e291… cells=14 centroid=(0.235, -0.509) path_m=0.63 goal=(-0.29, -0.43, -0.79)` | 중심 실습장 (0.27, 0.24), 목표 (0.45, 0.74) |
| 02:23:04 | 첫 작업 | `SEND SEARCH_AREA e35310d1… generation=0` | |
| 02:23:06~02:23:58 | 이동 | 정답 위치 (0.880, 0.722) → 서쪽. `Robot to slowdown for 30.000000 percents due to PassageSlow polygon` | |
| 02:23:58.56 | 결과 | `RESULT … outcome=COMPLETED reason=NAVIGATION_SUCCEEDED` | 1~5회차 중 첫 `COMPLETED` |
| 02:23:58.72 | 둘째 경계 | `FRONTIER_GOAL_MOVED (-1.41, 0.78) -> (-1.34, 0.77)` → `FRONTIER a116… cells=11 centroid=(-2.244, 1.866) path_m=3.01 goal=(-1.34, 0.77, 2.36)` → 02:24:00 `SEND … generation=1` | 중심 실습장 (3.08, 2.21), 목표 약 (1.8, 1.5). 이 사이 운영자 대역 `task_arm` 없음 |
| 02:25:17 | 결과 | `RESULT … COMPLETED` | 정답 위치 02:25:16 (1.63, 1.39, 32.8°) |
| 02:25:17.68~02:25:18.98 | 탐색 끝 | `FRONTIER none on map update 192~194: frontiers=1 small=0 no_goal=0 tried=1 no_path=0 too_close=0 \| TRIED cells=11 centroid=(-2.24, 1.86)` → `EXPLORATION_COMPLETE reason=EXPLORATION_COMPLETE frontiers_sent=2` | 남은 경계 1개 = a116 중심과 같은 자리 |
| 02:25:18 | 인형 없음 | `NO_SURVIVOR_FOUND` | |
| 02:25:19 | 지도 고정 | `MAP_FROZEN 1`·`2`(sha `097db7…`) → `task not issued: NO_HOME_POSITION` | |
| 02:25:19~02:25:24 | TB-5 | `START_ALIGN_ALIGNING` → `START_ALIGN_ALIGNED … "jump_xy_m": 0.323` → `MAP_STATE robot_b … VALID` | |
| 02:25:20~02:25:24 | TB-5 Nav2 | `NAV2_STARTUP_SENT` → `Managed nodes are active` | |
| 02:25:19~02:26:34 | 런북 7.5 | 아래 「7.5 구간」 | |
| 02:26:39 | TB-5 arm(운영자 대역) | `robot5 task_arm: success=True message='ARMED` | |
| 02:28:18~02:28:37 | 끄기 | `stop_full.sh` | 이 구간의 Traceback 은 종료 신호 뒤에 찍혔다 |

## 선포 직전 지도의 경계 칸 묶음

선포 직전 지도(`maps/022155-live-r0-307e903eca51`, TB-6 출발 자리)에서 가제보 실습이 경계 칸(free 옆 unknown, 8연결)을 묶어 센 값이다. 조정기 내부 계산과 같다는 보장은 없다.

| 칸 수 | 지도 중심 | 실습장 | TB-6 에서 직선 거리 |
| --- | --- | --- | --- |
| 32 | (−0.77, 2.20) | (3.12, 0.70) | 2.24 m |
| 31 | (−2.18, 1.90) | (3.10, 2.14) | 2.63 m |
| 14 | (0.24, −0.51) | (0.27, 0.24) — 조정기가 고른 `e291…`(cells=14) | 0.78 m |
| 10 | (−1.57, 2.19) | (3.27, 1.49) | 2.51 m |
| 10 | (−1.26, −0.91) | (0.17, 1.79) | 1.29 m |

## 7.5 구간 (TB-6 RobotPose 끊김 76.0 s)

| 시각 | 로그 | 어디 |
| --- | --- | --- |
| 02:25:18.69 | 마지막 `/rescue/robots/robot_a/pose`(SLAM) | `track/pose_gap.txt` |
| 02:25:19.03 | `MAP_REVISION_ADOPTED robot_a RNone -> R2` | robot_robot_a |
| 02:25:19.04 | `MAP_STATE robot_a …/2 UNKNOWN` | coordinator |
| 02:25:19~02:25:56 | 운영자 대역 원인 확인 자료(`diag_c8_tb5_nav2`, `diag_c7_before_slam_stop`) | operator |
| 02:25:56 | `런북 7.5 2차 고정 뒤 tb6_slam 끄기(launch PID …에 TERM, slam_toolbox PID …)` | operator |
| 02:26:08 | `런북 7.5 /robot6/map 발행자 수 0(--no-daemon)` | operator |
| 02:26:08~02:26:25 | `diag_c7_before_frozen_loc` 자료 | operator |
| 02:26:25 | `런북 7.5 frozen_localization robot_a 띄움` | operator |
| 02:26:28.33 | `initialPoseReceived` → `Setting pose (…): -1.159 0.603 2.346` | frozen_loc_a |
| 02:26:28.44 | `Managed nodes are active` | frozen_loc_a |
| 02:26:30.80 | `MAP_RELAYED /rescue/map/frozen R2 -> /robot6/map` | frozen_loc_a |
| 02:26:34.71 | AMCL `Received a 62 X 75 map` | frozen_loc_a |
| 02:26:34.72 | `MAP_STATE robot_a …/2 VALID`, 끊긴 뒤 첫 RobotPose(`GAP robot_a 76.0 s`) | coordinator, pose_gap |

구간별 길이: 운영자 대역 자료 수집 37 s + 17 s, SLAM TERM → 발행자 0 12 s, frozen_localization 띄움 → VALID 9.7 s.

## 위치(RobotPose)·접촉·속도·RTF

- RobotPose:
  - TB-6: 02:22:54 부터 2 Hz. 2 s 넘는 끊김은 위 76.0 s 한 번.
  - TB-5: 02:25:24 `VALID` 뒤 2 Hz, 끊김 0.
  - 02:28:20 `SILENT` 두 줄은 `stop_full` 뒤.
- 접촉(정답 위치, 몸 중심 최소 거리): TB-6 도크 (0.88, 0.213) 0.51 m(02:23:02, 출발), TB-5 도크 (2.70, 0.213) 1.59 m, TB-5 1.26 m, 인형 (1.20, 2.87) 1.54 m, 바깥 벽 0.60 m.
- Nav2(robot_robot_a.log): `PassageSlow` 13회, `Failed to make progress` 9회, `detected collision ahead` 0, `spin failed` 0, `invalid source` 0.
- 속도(정답 위치 0.5 s 차분, 02:23:02~02:25:17): 움직인 표본 77/270, 평균 0.039 m/s, 최고 0.158 m/s. 설정 최고 속도 `max_vel_x: 0.26`(turtlebot4_navigation nav2.yaml).
- RTF(30 s, 11표본): 중앙값 0.24, 최소 0.13, 최대 1.00, 0.5 미만 6개. 부하 26~41.
- 인형 대역 안 보낸 사유: `NO_TF` 2, `OUT_OF_RANGE` 2, `OUT_OF_VIEW` 2, `NO_MAP`·`NO_START_POSE`·`NOT_IN_FRONT`·`OCCLUDED`·`TF_STALE` 각 1.

## 실제 지도와의 차이 (재범 지적, 02:3x)

- 재범이 실제 회차 웹 지도(2026-10-15 17:05 화면)와 견주며 「너무 다른데?」라고 했다.
- 실제 화면: 벽이 지도 축과 나란하고, TB-6 은 (−0.12, −0.06) 169° 다.
- 시뮬레이션: 정답 위치로 바꾼 로봇 구역 모서리가 지도에서 SW (0.53, −0.72), SE (−0.14, 2.64), NE (−2.12, 2.25), NW (−1.44, −1.12)다. 남쪽 벽 방향은 101.3° 다. 매 회차 TB-6 언도킹 뒤 odom 방향은 101° 이고, 정답 방향은 89.5° 다.
- 가제보_지도 측정(02:33, 원문은 가제보_지도 보고): 언도킹·도킹 7번 모두 odom 191° 대 정답 179.5°, 명령 회전 360° 에 odom 361° 대 정답 339°, 직진 0.800 m 대 0.800 m.

## 파일

| 파일 | 무엇 |
| --- | --- |
| `logs/coordinator.log`, `robot_robot_a.log`, `robot_robot_b.log`, `frozen_loc_a.log`, `frozen_loc_b.log`, `tb6_slam.log`, `map_freeze.log`, `fake_target.log`, `operator.log` | 회차 폴더 원문. 색 코드를 지우고 경로·PC 이름을 바꿨다. 줄인 줄은 없다 |
| `logs/diag_*.txt` | 운영자 대역이 남긴 7.5 앞뒤 `/robot6/map` 발행자·프로세스, 1차 고정 뒤 TB-5 Nav2 lifecycle |
| `maps/022155-live-r0-307e903eca51` | 선포 직전 지도(TB-6 출발 자리) |
| `maps/022305-…` ~ `maps/022516-…` | 선포 뒤 저장본, 시각 순서. `022353-live-r0-e752c1c35e59` 는 둘째 경계를 고른 순간(02:23:58) 직전 것 |
| `maps/022519-frozen-r1·r2-…`, `maps/022521-live-r2-…`, `maps/frozen-r2-097db775952b-…` | 고정 지도와 메타 |
| `maps/timeout-map_f355518d3ce7` | 탐색 끝 순간 마지막 저장본(02:25:16) |
| `track/gt_track.txt` | 확인용 정답 위치, 0.5 s 마다 |
| `track/pose_gap.txt` | RobotPose 끊김(2 s 초과)·상태 변화·30 s 개수 |
| `track/rtf.txt` | Gazebo RTF·부하, 30 s 마다 |

## 참고: 가제보 실습 의견 (정확하지 않을 수 있음)

아래는 위 자료를 보고 가제보 실습이 짐작한 것이다. 확인하지 않은 해석이라 틀릴 수 있다.

- 탐색 종료(C11): 둘째 경계 목표가 벽에서 떼어져 터널 1 앞에 멈췄고, 그 자리에서는 칸막이 동쪽 면(x=2.75)이 터널 안을 가린 것으로 보인다. 조정기가 도착한 경계를 `TRIED` 로 빼서 끝난 것 같고, 조정기 동작이라 실물에서도 날 수 있다고 본다.
- 첫 경계 선택: 터널 1 입구 쪽 31칸이 있었는데도 가장 가까운 14칸을 고른 것은 거리 우선 선택으로 보인다.
- 7.5 끊김 76 s 중 54 s 는 운영자 대역 탓(시뮬레이션 쪽)이고, 런북 순서 몫 약 22 s 는 현장에서도 생길 것으로 본다.
- 낮은 RTF 와 지도 기울어짐(시뮬레이션 odom 회전 과대)은 시뮬레이션 쪽으로 본다.

# 가제보 실습 넷째 회차 (full-1016-012149, 2026-10-16 01:21 ~ 01:41)

- 빌드: `2d7df28` 이다. main `f314bbf` 에 시뮬레이션 실행 스크립트 브랜치 `feat/sim-full-run` 을 합쳤고, `src/`·`config/` 는 main 과 diff 0줄이다. 관제 웹도 같은 빌드다.
  - main 에 든 것: #379(TB-5 Nav2), #385(TB-5 둘레 비킴), #387(출발 정렬 진단), #389(목표를 벽에서 0.35 m 뗌), #390(실패 경계 재시도), #391(흰 상자 뺌), #392(서 있는 로봇 위치 유지)
- world: main 의 `config/sim/worlds/` 이고, 마지막 변경은 `62e5738`(흰 상자 뺌, 칸막이 네 면 그대로)이다.
- 아직 고치기 전인 시뮬레이션 쪽 두 가지:
  - 도크 IR 광선 표시가 라이다를 바깥 벽 너머로 보냄(S10, 고침은 가제보_지도 #395)
  - 실행 스크립트의 TB-5 출발값 계산
- 현장 값과 다른 시뮬레이션 사본 값은 3회차와 같이 `robots.robot_b.initial_pose_xy_yaw` 하나다. 현장 `[0.0, -1.86, 0.0]`, 시뮬레이션 `[-0.349, -1.786, 0.0021]` 이다.
- 재난 선포는 재범이 휴대폰 웹에서 했다(01:26:45.63).
- Gazebo 는 이 회차 하나뿐이었다. 동시 실행은 없었다.
- 실습장 좌표는 확인용 정답 위치(`track/gt_track.txt`)로만 썼다.
  - 지도→실습장 변환: 출발 자세 map (−0.353, 0, −169.0°)↔정답 (0.880, 0.722, 89.5°)로 맞췄다. 7.5 의 AMCL `Setting pose -0.354 0.012 -2.106`↔정답 138.3° 로 회전 −101.3° 를 확인했다.
  - 이 변환으로 TB-5 정렬 자세 (−0.642, 1.809)는 실습장 (2.71, 0.65)이다. 정답 (2.700, 0.722)과 7 cm 차이다.
- 시작 순서와 단계는 [시뮬레이션 회차가 돌아야 하는 모습](../../../config/sim/scenario.md)(#388)을 따른다.

## 단계별 결과

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1. TB-5·TB-6 언도킹으로 시작 | 통과 | `언도킹 robot6` 01:22:27, `언도킹 robot5` 01:22:43 |
| 2. 선포 순간 TB-6 이 SLAM 으로 곧바로 움직여 실습장을 훑음 | **실패** | 선포 2.6 s 뒤(01:26:48.24) 제자리에서 돌기 시작했다(89.5° → 138.3°, 01:26:52 까지). 그 뒤 회차 끝까지 **한 번도 자리를 옮기지 못했다**(정답 위치 (0.880, 0.722) 그대로, 14분 10초). 훑은 넓이는 시간 초과 지도 free 칸 2,639 / 실습장 칸 7,642 = 34.5% 다. 언도킹 자리에서 라이다로 본 넓이다 |
| 3. 웹 지도에 터널·인형 위치가 뜸 | **실패** | 관제 장부 `overhead_passages` 0행, 인형 대역 `FAKE_TARGET published` 0회다 |
| 4. 인형 발견 순간 TB-5 출발 | 실행 안 됨 | 인형을 찾지 못했다. 다만 TB-5 는 처음으로 출동 준비까지 됐다(01:38:08 `ARMED`) |

## 회차 표

| 시각 | 무엇 | 로그 | 나눔 |
| --- | --- | --- | --- |
| 01:21:49 | 시작 | `빌드 2d7df28(src 는 main 과 diff 0줄), 관제 웹 빌드 2d7df28` | |
| 01:22:47 | 사본 값 | `robots.robot_b.initial_pose_xy_yaw 현장 [0.0, -1.86, 0.0], 시뮬레이션 사본 [-0.349, -1.786, 0.0021]` | 현장 값과 다름 |
| 01:23:14 | TB-5 Nav2 | `NAV2_STARTUP_WAITING map -> base_link not on TF yet (first frozen map not localized): Nav2 not started` | #379 대로 기다림 |
| 01:23:47 | 준비 끝 | `TB-6 ARMED` | |
| 01:26:45.63 | 재난 선포 | `DISASTER_DECLARED` → `FRONTIER_GOAL_MOVED (-0.14, -0.54) -> (-0.29, -0.44) for 0.35 m clear` → `FRONTIER e291… cells=14 centroid=(0.23, -0.51) path_m=0.63 goal=(-0.29, -0.44, -0.79)` | 경계 중심은 실습장 (0.26, 0.25), 로봇 구역 남서 구석이다. 목표는 (0.31, 0.62)에서 (0.44, 0.75)로 옮겨졌다 |
| 01:26:47 | 첫 작업 | `SEND SEARCH_AREA 8a16dbca… generation=0` | |
| 01:26:48~52 | TB-6 회전 | 정답 위치 yaw 89.5° → 138.3° | 서쪽 목표 쪽으로 돌았다 |
| 01:26:50.72 | 정지 | `Robot to stop due to PassageStop polygon` → `PASSAGE_BLOCKED polygon='PassageStop' action=1`(01:26:50.91 한 번 더) | C1. 실물에서도 날 것. 아래 「TB-6 이 못 움직인 까닭」 |
| 01:26:57~01:36:45 | Nav2 | `Failed to make progress` 39회, `spin failed`(`Exceeded time allowance`) 9회 | 명령은 나갔지만 로봇에 닿지 않음 |
| 01:28:48 | 결과 | `RESULT … EXPIRED` → `FRONTIER_RETRY e291… failed 1/2: picked again after untried frontiers` → `FRONTIER_GOAL_TIGHT (-1.43, 1.00): no cell 0.35 m clear near it, kept at 0.2 m` → `FRONTIER a116… cells=24 centroid=(-0.73, 2.21)` | a116 중심은 실습장 (3.12, 0.66)이다. TB-5 와 동쪽 벽 사이, 실습장 안이다 |
| 01:28:56 / 01:31:05 / 01:33:10 / 01:35:19 | 다시 arm·작업 | 운영자 대역 `robot6 task_arm … ARMED` → `SEND … generation=1·2·3·4` | 매번 120 s 만료 |
| 01:30:57 | 셋째 경계 | `FRONTIER f0bfa088… cells=11 centroid=(-2.24, 1.86) goal=(-1.43, 1.00)` | 중심은 실습장 (3.08, 2.21), 터널 1 자리다 |
| 01:33:06 | 넷째 경계 | `FRONTIER 9c321c01… centroid=(-0.73, 2.21)` | a116 과 같은 자리, 다른 ID |
| 01:35:11 | 포기 | `FRONTIER_GAVE_UP 9c321c01… after 2 failed attempts` | 같은 자리는 ID 가 달라도 두 번째 실패에서 포기했다 |
| 01:36:45.60 | 탐색 시간 초과 | `EXPLORATION_COMPLETE reason=EXPLORATION_TIMEOUT frontiers_sent=5` | |
| 01:36:47 | 마지막 작업 | `RESULT … FAILED reason=GUARD_INVALID` → `FRONTIER_GAVE_UP c8a70d57… after 2 failed attempts` | 운영자 대역이 다시 arm 함(01:36:55) |
| 01:36:50 | 인형 없음 | `SYSTEM_STATE … 'reason': 'NO_SURVIVOR_FOUND'` | |
| 01:36:56 | 지도 고정 | `MAP_FROZEN 1`·`MAP_FROZEN 2`(같은 sha `af2a14…`) → `task not issued: NO_HOME_POSITION` | C5. 현장과 같음(1차 고정 지도에서 재서 넣는 값) |
| 01:36:56~01:37:02 | TB-5 출발 정렬 | `START_ALIGN_ALIGNING` → `START_ALIGN_ALIGNED … "jump_xy_m": 0.290` | 3회차와 같은 첫 추측에서 이번에는 맞췄다 |
| 01:36:57~01:37:02 | TB-5 Nav2 | `NAV2_STARTUP_SENT … map -> base_link is on TF` → `Managed nodes are active` → `NAV2_STARTUP … active` | C8 이 고쳐졌다(#379). 1차 고정 뒤 `ros2 lifecycle get` 도 네 서버 모두 `active [3]` |
| 01:37:25~01:38:08 | 런북 7.5(운영자 대역) | SLAM launch 에 TERM → 01:37:42 `/robot6/map 발행자 수 0` → 01:37:58 `frozen_localization robot_a` → 01:38:08 `MAP_RELAYED /rescue/map/frozen R2 -> /robot6/map` | 운영자 대역이 원인 확인 자료를 먼저 남기느라 R2 에서 SLAM 끄기까지 29 s 걸렸다 |
| 01:38:08 | TB-5 arm(운영자 대역) | `robot5 task_arm: success=True message='ARMED` | 네 회차 중 처음 |
| 01:40:43~01:41:02 | 끄기 | `stop_full.sh` | 끄는 중 `frozen_map_relay`·`start_align` Traceback 은 종료 신호 때 나온 것이다 |

## 이번에 새로 본 로그

| 로그 | 횟수 | 본 것 |
| --- | --- | --- |
| `FRONTIER_GOAL_MOVED` | 1 | 첫 목표를 벽에서 0.35 m 떼어 옮겼다(#389) |
| `FRONTIER_GOAL_TIGHT` | 4 | 셋째 이후 목표 (−1.43, 1.00), 실습장 (2.07, 1.58)은 0.35 m 빈 칸이 없어 0.2 m 로 두었다 |
| `FRONTIER_RETRY` | 3 | 실패한 경계를 아직 안 해 본 경계 뒤에 한 번 더 골랐다(#390) |
| `FRONTIER_GAVE_UP` | 2 | 같은 자리 두 번째 실패에서 포기했다. ID 가 바뀌어도 이어서 셌다 |
| `TRAFFIC_HOLD` | 0 | 3회차의 C10(TB-5 옆 목표 대기)은 나지 않았다 |
| `TRAFFIC_SCOUT_POSE_UNKNOWN` | 0 | 3회차의 7.5 뒤 위치 없음은 나지 않았다 |
| `invalid source` | 0 | 동시 실행이 없던 이번 회차에서는 0이다 |

### 서 있는 로봇의 위치(RobotPose)가 2 s 넘게 끊기는지 (`track/pose_gap.txt`)

- TB-6 `/rescue/robots/robot_a/pose`: 01:24:09 부터 30 s 마다 60개(2 Hz)였다.
  - 끊김은 한 번, 01:36:55.69 → 01:38:08.88 의 **73.2 s** 다. 2차 고정 뒤 SLAM 을 끄고 `frozen_localization` 이 자리를 잡기까지(런북 7.5)다.
  - 그 뒤 서 있는 동안 2 Hz 로 이어졌다(#392).
- TB-5 `/rescue/robots/robot_b/pose`: 01:36:56 `UNKNOWN` → 01:37:02 `VALID` 뒤 끝까지 2 Hz 였고 끊김은 없었다.
- 회차를 끈 01:40:43 뒤의 `SILENT` 두 줄은 끄면서 난 것이다.

## TB-6 이 못 움직인 까닭

- TB-6 이 첫 목표(실습장 (0.44, 0.75), 서쪽) 쪽으로 138.3° 까지 돌았을 때 PassageStop 상자 왼쪽 앞 모서리가 서쪽 벽에 닿았다.
  - 상자는 base 기준 앞 0.4~1.0 m, 옆 ±0.2 m, 높이 0.05~0.46 m 다.
  - 모서리(앞 1.0, 왼쪽 0.2)는 실습장 (0.00, 1.24)이다.
- `/robot6/passage_points` 5장(01:31:11)은 모두 같았다. stamp 나이는 0.02~0.03 s 였다.
  - 점 49,193개 중 상자 안은 38개, 가장 가까운 앞 거리는 1.00 m, 높이 p5/50/95 는 0.06/0.14/0.22 m 다.
  - TB-6 은 01:26:52 부터 같은 자세라 정지 순간과 같은 장면이다.
- 속도 명령 10 s(01:30:28) 측정: `cmd_vel_nav` 100, `cmd_vel_smoothed` 163(각속도 1.0), `cmd_vel_watchdog_in` 0, `cmd_vel` 0.
  - collision_monitor 가 회전까지 막았다.
  - 상태 로그는 01:26:50.91 `Robot to stop due to PassageStop polygon` 뒤 바뀌지 않았다.
- 나눔: **실물에서도 날 것.** C1(#363·#365)과 같은 교착이다. 현장 값 `stop` 그대로이고, 벽이 상자 모서리에 들어오면 실물에서도 같다.
  - 처음에는 도크 IR 광선 표시를 의심했지만, 도크는 TB-6 뒤쪽 (0.88, 0.21)이라 반대 방향이어서 철회했다.

## 나눔 요약

| 막힌 곳 | 나눔 | 근거 |
| --- | --- | --- |
| TB-6 이 출발 자리에서 PassageStop 으로 묶임 | 실물에서도 날 것 | 위 절. 서쪽 벽 깊이 점 38개, 명령 사슬이 collision_monitor 에서 0 |
| 터널·인형이 웹에 안 뜸 | 실물에서도 날 것(위 교착의 결과) | TB-6 이 출발 자리를 벗어나지 못했다. 인형 대역 사유는 `OUT_OF_RANGE` 2, `NO_CAMERA_INFO`·`NO_START_POSE`·`NO_TF`·`TF_STALE` 각 1 이다 |
| 7.5 동안 TB-6 위치 73 s 없음 | 모름 | 런북 7.5 의 정해진 끊김이다. 그중 29 s 는 운영자 대역이 원인 확인 자료를 먼저 남긴 탓이라 시뮬레이션 쪽이다 |
| RTF 가 자주 0.5 아래 | 시뮬레이션 탓 | 30 s 표본 34개 중 15개가 0.5 미만이고, 중앙값 0.90, 범위 0.16~1.58, 부하 19~50 이다. Gazebo 는 하나뿐이었다. 원인은 모른다 |

## 운영자 대역이 런북과 다르게 한 것

3회차 자료의 네 줄과 같다. 이번에는 `START_ALIGN_FAILED` 가 나지 않아 손으로 한 조작은 없다.

## 파일

| 파일 | 무엇 |
| --- | --- |
| `logs/coordinator.log`, `robot_robot_a.log`, `robot_robot_b.log`, `frozen_loc_a.log`, `frozen_loc_b.log`, `map_freeze.log`, `fake_target.log`, `operator.log` | 회차 폴더 원문. 색 코드를 지우고 경로·PC 이름을 바꿨다. 줄인 줄은 없다 |
| `logs/diag_c7_*.txt`, `diag_c8_tb5_nav2.txt` | 운영자 대역이 남긴 7.5 앞뒤 `/robot6/map` 발행자·프로세스, 1차 고정 뒤 TB-5 Nav2 lifecycle |
| `maps/live-r0-b7466969dc82` | 선포 전 01:23:14 TB-6 SLAM 지도 |
| `maps/live-r0-9eb4bb380351` | 선포 직후 01:26:48 지도 |
| `maps/timeout-map_d2b6e4fde836` | 탐색 시간 초과(01:36:45) 직전 마지막 저장본(01:36:30) |
| `maps/frozen-r2-af2a14bfd84e-…` | 2차 고정 지도와 메타(1차와 같은 sha) |
| `track/gt_track.txt` | 확인용 정답 위치, 0.5 s 마다(01:24~01:41) |
| `track/pose_gap.txt` | `/rescue/robots/<id>/pose` 끊김(2 s 초과)·상태 변화·30 s 개수 |
| `track/rtf.txt` | Gazebo RTF·부하, 30 s 마다 |

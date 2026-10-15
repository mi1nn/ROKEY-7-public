# 가제보 실습 셋째 회차 (full-1015-235518, 2026-10-15 23:55 ~ 10-16 00:37)

- 빌드: `fe54428` 이다. `src/` 는 main `4896338` 과 diff 0줄이고, 관제 웹도 같은 빌드다. TB-5 Nav2 고침 #379 와 #385 는 없다.
- world: main 의 `config/sim/worlds/arena.sdf`(blob `3bdb523`)다. 1·2회차는 가제보 실습의 임시 world 파일이었다([둘째 회차 자료](../sim-2026-10-15-run232037/README.md)).
- 현장 값과 다른 시뮬레이션 사본 값은 하나다. `robots.robot_b.initial_pose_xy_yaw` 가 현장 `[0.0, -1.86, 0.0]` 인데 시뮬레이션은 `[-0.348, -1.786, -0.0001]` 이다. 실행 스크립트가 언도킹 뒤 odom 자세로 계산했다. 그 밖의 값(PassageStop, `home_positions` 없음)은 현장 값 그대로다.
- 재난 선포는 재범이 휴대폰 웹에서 했다(00:22:24).
- 실습장 좌표(x 동쪽, y 북쪽, 원점 로봇 구역 남서 모서리)는 확인용 정답 위치(`track/gt_track.txt`)로만 썼다. 어떤 노드에도 주지 않았다.

## 단계별 결과 (재범이 정한 시뮬레이션 회차 순서, 2026-10-16)

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1. TB-5·TB-6 언도킹으로 시작 | 통과 | `언도킹 robot6` 23:55:56, `언도킹 robot5` 23:56:18 |
| 2. 선포 순간 TB-6 이 SLAM 으로 곧바로 움직여 실습장을 훑음 | **실패** | 첫 이동은 선포 2.5 s 뒤(00:22:26.58)였다. 00:22:50 부터 회차 끝까지 14분 43초 동안 한자리에 섰다. 훑은 넓이는 실습장 칸 7,642개 중 free 칸 2,646개(34.6%)다. 분모는 바깥 사각형 3.43×5.57 m 전부라 벽·상자 자리도 들어 있다 |
| 3. 웹 지도에 터널·인형 위치가 뜸 | **실패** | 터널(`overhead_passages`)과 인형 모두 뜨지 않았다. 터널 판정은 한 번도 나오지 않았고 인형 관측은 한 번도 보내지 않았다 |
| 4. 인형 발견 순간 TB-5 출발 | 실행 안 됨 | 인형을 찾지 못했다 |

## 회차 표

| 시각 | 무엇 | 로그 | 나눔 |
| --- | --- | --- | --- |
| 23:55:18 | 시작 | `빌드 fe54428(src 는 main 과 diff 0줄), 관제 웹 빌드 fe54428` | |
| 23:56:20 | 사본 값 | `robots.robot_b.initial_pose_xy_yaw 현장 [0.0, -1.86, 0.0], 시뮬레이션 사본 [-0.348, -1.786, -0.0001]` | 현장 값과 다름 |
| 23:56:46~23:57:48 | TB-5 Nav2 | `Activating planner_server` → 0.5 s 마다 `Timed out waiting for transform from base_link to map` → `Failed to activate global_costmap …` → `Aborting bringup` | C8. #375 분석과 같은 모양이다(2회차는 `Configuring smoother_server` 뒤 아무 줄 없음) |
| 23:56:53 | 출발 자세 | `START_POSE captured robot_a map=(-0.353, -0.000, -2.949)` | |
| 23:57:18 | 준비 끝 | `TB-6 ARMED` | 운영자 대역은 23:58:39 에 고친 판으로 다시 띄웠다(`operator.log`) |
| 00:12:2x~00:26:13 | **시뮬레이션 동시 실행 구간** | 지도 자리 Gazebo(다른 partition·도메인)가 같이 돌았다 | RTF 중앙값 0.56(0.14~1.17), 부하 46~79. 꺼진 뒤 00:27:49 부터 RTF 약 1.0 |
| 00:22:24.09 | 재난 선포 | `DISASTER_DECLARED` → `FRONTIER e291… cells=95 centroid=(-1.89, 1.73) goal=(-1.49, 0.86)` | 첫 경계는 실습장 안이다. 터널 1 입구 쪽 (2.90~2.92, 1.82~1.88)이다 |
| 00:22:26.05 | 첫 작업 | `SEND SEARCH_AREA d8969fdb…` | 목표는 실습장 (1.96~1.98, 1.63~1.66)이고, 흰 상자 남쪽 벽(y=2.01)에서 0.35~0.38 m 다 |
| 00:22:32.69 | 정지 | `[passage_depth]: Latest source and current collision monitor node timestamps differ on 1.043168 seconds` → `Robot to stop due to invalid source` | 시뮬레이션 탓(동시 실행 구간). 아래 「나눔」 |
| 00:22:49.67 | 정지 | `Robot to stop due to PassageStop polygon`, `PASSAGE_BLOCKED polygon='PassageStop'` | C1, 실물에서도 날 것 |
| 00:22:50 | TB-6 멈춤 | 정답 위치 (1.528, 1.340, 29.8°). 흰 상자 남쪽 벽까지 중심 0.67 m, 몸 가장자리 약 0.48 m | 목표에 서지 못했다(목표까지 0.4~0.6 m) |
| 00:23:36 | 회전 | `Exceeded time allowance before reaching the Spin goal` → `spin failed` | |
| 00:24:26.94 | 결과 | `RESULT … outcome=EXPIRED reason=DEADLINE_EXPIRED` | 120 s 만료 |
| 00:24:27 | 둘째 경계 | `FRONTIER a116… cells=12 centroid=(-1.27, 2.62) goal=(-0.37, 1.65)` | 중심은 실습장 **밖**, 동쪽 벽(x=3.43) 바깥 0.21 m (3.64, 1.01~1.09)다. 목표 (2.49~2.51, 0.36~0.40)는 TB-5 에서 0.39 m 다 |
| 00:24:29 / 00:24:37 | 다시 arm | `SCOUT_NOT_READY … DEADLINE_EXPIRED` → 운영자 대역 `robot6 task_arm: success=True message='ARMED` | |
| 00:24:36.9 | 교통 대기 | `TRAFFIC_HOLD SCOUT TRAFFIC_ROBOTS_TOO_CLOSE` → `task not issued` | C10. 그 뒤 SEND 없음 |
| 00:32:24 | 탐색 시간 초과 | `EXPLORATION_COMPLETE reason=EXPLORATION_TIMEOUT frontiers_sent=2` → `MAP_FROZEN 1`·`MAP_FROZEN 2`(같은 sha `6630b6…`) | 선포 + 600 s |
| 00:32:24.19~00:32:34.89 | TB-5 출발 정렬 | `START_ALIGN_ALIGNING` → `START_ALIGN_FAILED START_ALIGN_NOT_CONVERGED … "updates": 20` | 시뮬레이션 탓 [추정]. 아래 「나눔」 |
| 00:32:51~00:33:35 | 런북 7.5(운영자 대역) | SLAM launch 에 TERM → `/robot6/map 발행자 수 0` → `frozen_localization robot_a` → `MAP_RELAYED /rescue/map/frozen R2 -> /robot6/map` | C7 재현 안 됨(운영자 대역을 고친 뒤) |
| 00:33:37 | TB-6 위치 | `TRAFFIC_HOLD SCOUT TRAFFIC_SCOUT_POSE_UNKNOWN` | `/robot6/amcl_pose` 마지막 stamp 는 00:33:35 다. 서 있는 TB-6 은 더 내지 않았다(`logs/diag_tb6_amcl_pose.txt`) |
| 00:36:08 | 출발 정렬 다시(런북 7단계, 가제보 실습이 손으로) | `/robot5/rescue_start_align/restart` → `success=True` | `logs/operator_manual.log` |
| 00:36:22 | TB-5 정렬 | `START_ALIGN_ALIGNED … "pose": [-0.690, 1.785, -2.960], "jump_xy_m": 0.092, "match_ratio": 0.939, "updates": 18` → `MAP_STATE robot_b … /2 VALID` | 첫 추측이 직전 AMCL 자세였다 |
| 00:36:30·00:36:49·00:37:07 | TB-5 arm(운영자 대역) | `TASK_ARM refused NAV_NOT_READY missing=['nav2']` | C8 |
| 00:37:14~00:37:33 | 끄기 | `stop_full.sh` | 끄는 중 `task_input`·`start_align`·`frozen_map_relay`·`overhead_passage` Traceback 은 종료 신호 때 나온 것이다 |

C2(TB-5 정렬 판정 버려짐, #370)는 이 회차 로그에 `Logger severity` 줄이 0 이다.

## 나눔: 실물에서도 날 것 / 시뮬레이션 탓 / 모름

| 막힌 곳 | 나눔 | 근거 |
| --- | --- | --- |
| TB-6 이 흰 상자 남쪽 벽 앞에서 PassageStop 으로 섬 | 실물에서도 날 것 | 현장 값 그대로인 C1(#363·#365)이 다시 났다. 벽이 PassageStop 상자(앞 0.4~1.0 m) 안에 들어왔다 |
| `invalid source`(깊이 점이 1.04~1.09 s 늦음) | 시뮬레이션 탓 | 모두 00:22:32~00:24:26 에 났고, 이 구간은 동시 실행으로 RTF 중앙값 0.56 이었다. RTF 가 1.0 으로 돌아온 뒤에는 TB-6 에 주행 명령이 없어서, 사라졌는지는 확인하지 못했다(미확인) |
| 둘째 경계 a116 이 실습장 밖 | 시뮬레이션 탓 | 아래 「라이다가 바깥 벽을 지남」 |
| a116 목표가 TB-5 옆이라 `TRAFFIC_ROBOTS_TOO_CLOSE` 로 대기, 다음 경계를 고르지 않음(C10) | 실물에서도 날 수 있음 [추정] | 조정기 동작이다. 현장에서도 TB-5 가까이 목표가 잡히면 같은 순서가 된다. 근거는 로그 순서이고 코드는 보지 않았다 |
| 7.5 뒤 서 있는 TB-6 의 위치가 낡아 `TRAFFIC_SCOUT_POSE_UNKNOWN` | 실물에서도 날 수 있음 [추정] | AMCL 은 움직이지 않으면 자세를 내지 않는다. 런북 7단계 2번도 「서 있는 TB-6 에서 `/robot6/pose` 가 끊기는지 기록」을 둔다 |
| TB-5 `START_ALIGN_NOT_CONVERGED` | 시뮬레이션 탓 [추정] | 아래 「TB-5 첫 추측이 0.4 m 어긋남」 |
| TB-5 Nav2 가 active 가 안 됨(C8) | 실물에서도 날 것 | #375 분석과 같은 줄이다. #379 가 이 빌드에 없다 |
| 터널 판정이 한 번도 안 나옴 | 모름 | TB-6 이 터널에 가지 못했다. `overhead_passage` 는 `camera view …: no RGB frame within 20000000 ns or no CameraInfo` 를 464회(TB-5 465회) 냈고, `not judged: LocationRefused: NO_VALID_MAP_STATE` 를 14회 냈다. RGB·깊이 stamp 를 따로 재지 않아 중계 탓인지 가리지 못했다 |
| 인형 관측이 안 나감 | 모름 | 인형 대역 로그는 `FAKE_TARGET not sent: OCCLUDED`·`OUT_OF_VIEW` 뿐이다. TB-6 이 인형을 볼 자리에 가지 못했다 |

## 라이다가 바깥 벽을 지남 (S10, 3회차 main world)

- 지도 변환은 TB-6 정답 위치와 SLAM 자세 두 쌍으로 맞췄다. 두 쌍은 출발 map(−0.353, 0, −169°)↔(0.880, 0.722, 89.5°)와 00:26 `/robot6/pose` (−1.099, 0.484, 132.5°)↔(1.528, 1.340, 29.8°)다. 회전은 −101.5~−103.4°, 잔차는 1.5 cm 이하다. 이 변환으로 지도 동쪽 벽 줄이 x=3.43 에 놓인다.
- 선포 전 23:56:46 지도(`maps/live-r0-63e8224c05bc`)부터 동쪽 벽 바깥 (3.51~3.91, 0.94~1.12)에 free 칸 14개가 있다. 이것이 a116 이 됐다.
- 시간 초과 지도(`maps/timeout-map_9ad0fc7ac1d9`)에는 남쪽 벽 바깥 (2.47~3.43, −0.61~−0.08)에도 free 칸이 있다. 실습장 밖 free 칸은 모두 106개다.
- 라이다 직접 측정(정답 위치, 20스캔, 계산은 [둘째 회차 자료](../sim-2026-10-15-run232037/README.md)와 같음):
  - TB-6 (1.53, 1.34)은 00:27:01 에 12,800빔 중 340빔(2.7%)이 남쪽 벽 밖으로 나갔다. 방위 −43~−55° 로 매 스캔 같은 각도였다.
  - TB-5 (2.70, 0.72)는 00:17:14 와 00:27:01 모두 0 이었다.
- 1회차의 TB-5 자리 5.3% 는 여기서도 재현되지 않았다. 가제보_지도는 로봇 한 대와 정지 라이다만 둔 world 에서는 같은 벽에서 0 이었다고 알려 왔다. 두 번째 로봇과 그 도크 근처에서 빔이 지나는 것으로 의심한다 [추정]. 이 PR 시점에 지도 자리의 고침은 없다.

## TB-5 첫 추측이 0.4 m 어긋남

- 시뮬레이션 사본의 `robot_b` 값 `[-0.348, -1.786]` 은 실행 스크립트가 TB-6 odom 방향(101°)으로 계산했다. 정답 위치로 본 TB-6 방향은 89.5° 다. 언도킹 회전에서 시뮬레이션 odom 이 약 11.5° 더 돌았다.
- 그래서 `first_guess` (−0.353, 1.820)는 정답 위치를 지도로 바꾼 TB-5 자세 (−0.72~−0.77, 1.77, −167~−169°)에서 0.37~0.42 m 떨어져 있었다.
- AMCL 은 20번 갱신으로 (−0.609, 1.827)까지 왔고 참값에서 0.12~0.17 m 남았다(`logs/diag_tb5_amcl_pose.txt`, stamp 00:32:34.79).
- 런북대로 다시 하자 직전 AMCL 자세에서 출발해 ALIGNED (−0.690, 1.785)가 됐다. 참값과 3~8 cm 차이다.
- 정답 방향(89.5°)으로 계산했으면 사본 값은 (0.016, −1.82)이다. 현장 값 `[0.0, -1.86, 0.0]` 과 0.04 m 차이다 [계산].
- 그래서 이 실패는 시뮬레이션 사본 값 탓으로 본다. 1·2회차는 같은 `first_guess` 로 ALIGNED 였는데, 왜 이번만 수렴하지 않았는지는 모른다.
- 스캔 한 장은 `logs/diag_tb5_scan.txt`(00:34:56, 도메인 78 중계 뒤)에 있다. 두 로봇 모두 00:22:50 부터 서 있어서 실패 순간과 같은 장면이다.

## 운영자 대역이 런북과 다르게 한 것 (sim-to-field 표 2 후보)

- 7.5: 런북은 `tb6_slam.launch.py` 터미널에서 Ctrl+C 를 누른다. 운영자 대역은 백그라운드 launch 에 SIGTERM 을 보내고 slam_toolbox PID 가 끝났는지 본다. 백그라운드 launch 는 SIGINT 를 무시한다.
- 7.1: 런북은 재난 전에 TB-5 를 arm 한다. 운영자 대역은 `START_ALIGN_ALIGNED` 뒤에 한다. 고정 전에는 TB-5 에 지도가 없어서다.
- `START_ALIGN_FAILED` 뒤 `restart` 는 운영자 대역에 없다. 이 회차는 가제보 실습이 손으로 한 번 했다(00:36:08).
- 운영자 대역이 원인 확인 자료를 먼저 남기느라 7.5 가 27 s 늦었다. `MAP_REVISION_ADOPTED robot_a … R2` 00:32:24 → SLAM 끄기 00:32:51 이다. `ros2 lifecycle get` 다섯 번과 `topic info` 때문이다.

## 파일

| 파일 | 무엇 |
| --- | --- |
| `logs/coordinator.log`, `robot_robot_a.log`, `robot_robot_b.log`, `frozen_loc_a.log`, `frozen_loc_b.log`, `map_freeze.log`, `fake_target.log` | 회차 폴더 원문. 색 코드를 지우고 경로·PC 이름을 바꿨다. 줄인 줄은 없다 |
| `logs/operator.log`, `logs/operator_manual.log` | 운영자 대역과 손으로 한 런북 조작 |
| `logs/diag_c7_*.txt`, `diag_c8_tb5_nav2.txt` | 운영자 대역이 남긴 7.5 앞뒤 `/robot6/map` 발행자·프로세스, 1차 고정 뒤 TB-5 Nav2 lifecycle |
| `logs/diag_tb5_amcl_pose.txt`, `diag_tb5_scan.txt`, `diag_tb6_amcl_pose.txt` | 회차 끝 무렵 `--once` 로 받은 TB-5 AMCL 자세·스캔, TB-6 AMCL 자세 |
| `maps/live-r0-63e8224c05bc` | 선포 전 23:56:46 TB-6 SLAM 지도(동쪽 벽 밖 free 칸이 이미 있음) |
| `maps/live-r0-f8bde8a4e225` | 선포 직후 00:22:27 지도 |
| `maps/timeout-map_9ad0fc7ac1d9` | 탐색 시간 초과 순간(00:32:24)의 마지막 저장본(00:31:51) |
| `maps/frozen-r2-6630b6dae2f4-…` | 2차 고정 지도와 메타(1차와 같은 sha) |
| `track/gt_track.txt` | 확인용 정답 위치, 0.5 s 마다(00:05~00:37). 형식 `시각 robot5=(x,y,yaw°) robot6=(x,y,yaw°)` |
| `track/rtf.txt` | Gazebo RTF·부하, 30 s 마다(00:14~00:36) |

- RTF 표본에는 동시 실행 구간 밖에서도 낮은 것이 두 개 있다(00:28:19 0.18, 00:34:52 0.13). 원인은 모른다.

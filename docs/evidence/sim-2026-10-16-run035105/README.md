# 가제보 실습 아홉째 회차 (full-1016-035105, 2026-10-16 03:51 ~ 03:58, sim1)

측정·로그 자료를 먼저 적는다. 가제보 실습의 해석은 맨 끝 「참고: 가제보 실습 의견」에만 둔다(재범 지시 2026-10-16).

- 빌드: `b5e5872`(main, `src/`·`config/` diff 0줄). 8회차 빌드(`9a3e274`) 뒤 들어간 것:
  - #411(터널 높이 바닥 기준)
  - #412·#415(터널 통과 승인)
  - #413(sim1 박스 기둥 되살림)
  - #414·#422(두 로봇 RViz, run_full 이 띄움)
  - #417(라이다 아래 낮은 물체 `depth_low_layer`)
  - #419(watchdog 보이기)
  - #420(C12)
  - #421(복귀 명령 수락)
- world: sim1(`arena.sdf` 마지막 변경 `92a35b9`, 기둥 되살림), 흰 상자 없음.
- 현장 값과 다름: `robots.robot_b.initial_pose_xy_yaw` 시뮬레이션 사본 `[0.013, -1.82, 0.0002]`, `home_positions` `robot_a=-0.35506,-3.1e-11,3.13324 robot_b=-0.353,1.82,3.1334`(`SIM_HOME=odom`).
- 시간 동기(기동 직전 03:51:05, `logs/timesync_status_at_start.txt`): timesyncd `NTPMessage` spike=false, Offset +15.7 ms, Delay 204.1 ms, 서버 ntp.ubuntu.com.
  - 이 회차 전 03:31·03:34 두 번은 조정기 `central live clock: no valid synchronized NTP reply` 로 기동이 멈췄다. 그때 spike=true 였고, 마지막 응답은 03:21:15, delay 317 ms 였다.
  - 03:38 에는 가제보 실습이 회차 폴더를 미리 만들어 run_full 이 `이미 있습니다` 로 멈췄다(실습 쪽 실수).
- 재난 선포: 재범 위임으로 가제보 실습이 관제 웹 `POST /commands kind=DECLARE_DISASTER`(admin), 03:54:02.25 요청 → `accepted: true`. 조정기 `DISASTER_DECLARED` 03:54:02.61.
- 녹화: Gazebo 화면 클라이언트(녹화용 가상 화면 :99)와 run_full 이 띄운 두 로봇 RViz(#422, `DISPLAY=:98` 로 띄움)를 좌우로 녹화했다(03:53:43~03:58:30).

## 단계별 결과

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1. 언도킹으로 시작 | 통과 | run_full 언도킹 두 대 |
| 2. 선포 즉시 TB-6 이 움직여 실습장을 훑음 | **실패** | 경계 2개 `COMPLETED`(03:55:49, 03:55:52) 뒤 03:55:54.90 `EXPLORATION_COMPLETE frontiers_sent=2`. TB-6 정답 위치 최대 y 1.552, 이동 거리 1.48 m, 최종 (2.088, 1.552) |
| 3. 웹 지도에 터널·인형이 뜸 | **실패** | `overhead (PASSABLE\|BLOCKED)` 줄 0, 장부 `overhead_passages` 0행, `FAKE_TARGET published` 0 |
| 4. 인형 발견 → TB-5 출발 | 실행 안 됨 | `NO_SURVIVOR_FOUND`. TB-5 출발 정렬 `LOW_MATCH` 2회 |

## 회차 표

| 시각 | 로그 |
| --- | --- |
| 03:54:02.61 | `FRONTIER e291… cells=13 centroid=(-1.825, 2.203) path_m=1.78 cost=-0.22 cut=True goal=(-1.17, 1.17, 1.57)` → 03:54:04 `SEND … generation=0` → `ADMISSION -> True` |
| 03:55:49.24 | `RESULT … COMPLETED` → `FRONTIER_GOAL_CUT e291… reached 1.5 m along 1.78 m: the frontier is not tried yet` |
| 03:55:49.38 | `FRONTIER a116… cells=13 centroid=(-1.841, 2.200) path_m=0.30 cost=-0.45 cut=False goal=(-1.19, 1.45, 1.57)` → 03:55:51 `SEND … generation=1` |
| 03:55:52.74 | `RESULT … COMPLETED`(TB-6 `Reached the goal!`) |
| 03:55:52.89~03:55:54.90 | `FRONTIER none on map update 191~193: frontiers=1 small=0 no_goal=0 tried=1 tight=0 no_path=0 too_close=0 \| TRIED cells=13 centroid=(-1.84, 2.20)` → `EXPLORATION_COMPLETE reason=EXPLORATION_COMPLETE frontiers_sent=2` |
| 03:55:54.91 | `MAP_FROZEN 1`·`2`(sha `af5ddd…`) / TB-6 `MAP_REVISION_ADOPTED robot_a RNone -> R2` |
| 03:55:54.93 | `SEND RETURN_HOME e27a9007… generation=2` → `ADMISSION -> True` → `GOAL … accepted=True` / TB-6 `Begin navigating from current location (-1.18, 1.21) to (-0.36, -0.00)` |
| 03:55:55.75 | TB-6 `MAP_HANDOVER robot_a R2 frozen grid on its map topic: SLAM pose until now, AMCL from here` → `MAP_STATE … quality=UNKNOWN` |
| 03:55:55.77 | TB-6 `task guard rejected 3b7e6381…: INVALID_INPUT: INVALID_INPUT [rescue_robot.INVALID_INPUT]: input observation time` → `CANCEL` → `Goal canceled` |
| 03:55:56.74 | 조정기 `RESULT task=e27a9007… outcome=FAILED reason=GUARD_INVALID`. 그 뒤 `SEND RETURN_HOME` 없음 |
| 03:55:54~03:57:11 | TB-5 `START_ALIGN_ALIGNING` → `START_ALIGN_FAILED START_ALIGN_LOW_MATCH {"first_guess": [-0.3529, 1.8200, 3.1334], "jump_xy_m": 0.054, "match_points": 640, "match_ratio": 0.6, "pose": [-0.4011, 1.7953, -3.1327], "sigma_xy_m": 0.095, "updates": 11}` |
| 03:57:12 | 운영자 대역 `robot6 task_arm: success=False` / `TASK_ARM refused NAV_NOT_READY missing=['slam'] motion=STOPPED sources=['map_to_odom_tf(35.6 s)']` → 03:57:28 `ARMED` |
| 03:57:21 | 손으로 런북 7단계 `rescue_start_align/restart` → `success=True` |
| 03:57:34 | `MAP_RELAYED /rescue/map/frozen R2 -> /robot6/map` |
| 03:58:17 | TB-5 `START_ALIGN_FAILED START_ALIGN_LOW_MATCH … "jump_xy_m": 0.0166, "sigma_xy_m": 0.094, "updates": 15` |
| 03:58:18~03:58:35 | 녹화 끔, `stop_full.sh`(이 구간 Traceback 은 종료 신호 뒤) |

## 그 밖의 자료

- TB-5 정렬 순간 라이다: `logs/diag_tb5_scan_at_align.txt`(03:55:59), `diag_tb5_scan_at_align_2.txt`(3 s 뒤), `diag_tb5_scan_at_restart.txt`(03:57:23). `/robot5/scan` 원 배열(640빔, frame `rplidar_link`)이다. 그때 정답 위치는 TB-5 (2.700, 0.722, 89.6°), TB-6 (2.088, 1.552) 다.
- RobotPose: TB-6 끊김은 03:55:56.86 `VALID -> UNKNOWN` → 03:55:57.87 `VALID mission=(-1.184, 1.208)`, 1 s 다. TB-5 는 03:55:54.98 `UNKNOWN` 뒤 `VALID` 없음.
- 접촉(정답 위치, 몸 중심 최소): TB-6 도크 0.509 m(출발), TB-5 도크 1.427 m, TB-5 1.031 m, 인형 1.574 m.
- Nav2(robot_robot_a.log): `PassageSlow` 10, `Failed to make progress` 7, `detected collision ahead` 0. `depth_low_layer` 플러그인 사용(`Subscribed to Topics: depth_low depth_floor`).
- 인형 대역 안 보낸 사유: `NOT_IN_FRONT` 2, `OUT_OF_VIEW` 2, `NO_START_POSE`·`NO_TF`·`OCCLUDED`·`OUT_OF_RANGE`·`TF_STALE` 각 1.
- RTF(30 s, 10표본): 중앙값 0.29, 최소 0.14, 최대 1.21, 0.5 미만 6. 부하 32~49(화면 녹화 켬).

## 8회차와 같은 것·다른 것

- 다른 것:
  - 둘째 경계 a116 이 8회차는 `path_m=1.39 goal=(-1.62, 2.30)`(터널 1 안까지 감), 9회차는 `path_m=0.30 goal=(-1.19, 1.45)`(TB-6 바로 옆)였다.
  - 탐색 끝: 9회차는 그 뒤 `TRIED` 로 끝났다(8회차는 경계 4개, 메인 구역까지).
  - RETURN_HOME: 8회차 `ExecuteTask rejected … mission frame`, 9회차 `ADMISSION -> True` 뒤 `task guard rejected … input observation time` → `GUARD_INVALID`.
  - TB-6 7.5 끊김 4.5 s → 1 s.
- 같은 것: TB-5 정렬 실패(`LOW_MATCH`), 인형 관측 0.

## 파일

| 파일 | 무엇 |
| --- | --- |
| `logs/*.log` | 회차 폴더 원문(색 코드 지움, 경로·PC 이름 바꿈, 줄인 줄 없음). `rviz.log` 는 run_full 이 띄운 RViz |
| `logs/diag_tb5_scan_*.txt` | TB-5 정렬 순간 라이다 |
| `logs/timesync_status_at_start.txt` | 기동 직전 시간 동기 상태 |
| `maps/`, `track/` | SLAM 저장본(선포 직전~탐색 끝), 고정 지도, 정답 위치·RobotPose·RTF |
| `video/screen_gazebo_rviz_small.mp4`, `video/shot_*.png` | 화면 녹화 줄인 판(1280×720·5 fps)과 녹화 시작(03:53:43) 기준 19 s(선포), 126 s(첫 성공), 133 s(복귀 실패), 208 s(TB-5 정렬 실패) 장면 |

## 참고: 가제보 실습 의견 (정확하지 않을 수 있음)

아래는 위 자료를 보고 짐작한 것이다. 확인하지 않은 해석이라 틀릴 수 있다.

- 탐색이 일찍 끝난 것은 둘째 경계 목표가 TB-6 바로 옆(0.30 m)에 잡혀 `cut=False` 로 도착 처리된 뒤 같은 경계가 남은 C11 과 비슷한 모양으로 보인다.
- 복귀 실패는 2차 고정 직후 SLAM→AMCL 넘김(`MAP_HANDOVER`)과 RETURN_HOME 이 같은 순간에 겹쳐 입력 시각 검사에 걸린 것 같다. 현장도 같은 순서라면 같을 수 있다.
- TB-5 정렬은 첫 추측이 정답과 같아도 `match_ratio 0.6` 에서 멈추는 일이 8·9회차 모두 났다. 저장한 스캔으로 원인을 볼 수 있을 것 같다.

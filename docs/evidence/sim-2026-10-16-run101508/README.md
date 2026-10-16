# 가제보 실습 스물두째 회차 (full-1016-101508, 2026-10-16 10:15 ~ 10:34, sim1 arena 기둥 축소, 웹캠 지진 시작)

측정·로그 자료를 먼저 적는다. 가제보 실습의 해석은 맨 끝 「참고: 가제보 실습 의견」에만 둔다(재범 지시 2026-10-16).

- 재범 지시(2026-10-16 10:0x, 원문): 「실습 진행할껀데 어떻게 진행할꺼냐면 자동 시작이 아니라, <시뮬레이션 PC>에 웹캠을 연결해서 흔들어서 인공지진을 만들어서 실습을 진행할꺼임. 그리고 지금 지도가 터널 2에 있는 사각형을 좀 작게 만들꺼임. 그거 반영하고 해당 실습을 진행하자」. PC 이름만 자리 이름으로 바꿨다.
- 빌드: main `54ecade`(#476 watchdog 자동 해제, #480 가운데 기둥 0.40 × 0.40 m, #465 RViz TB-5 출발 자리 포함)에 머지 전 #481 head `137978b`(run_full 웹캠 옵션, `scripts/sim`·`config/sim`·`tests` 만)를 이 PC 에서 합쳐(`cf255eb`) 띄웠다. #466(TB-5 접근 지점)은 열려 있어 빠졌다.
- 웹캠: `SIM_WEBCAM_DEVICE=usb-Web_Camera_Web_Camera_202400126002-video-index0`. run_full `웹캠 …, 관제 웹 --camera cam_fixed=http://<웹 주소>:18190/cam_fixed.mjpg`, webcam_node `YOLO warm-up 2.26s`, `clock SYNCED … error_bound_ns=274089687`(`logs/operations_webcam.log`).
- 화면: 21회차와 같다(이 PC 화면 왼쪽 Gazebo, 오른쪽 두 로봇 RViz, 녹화 없음). 관제 웹은 Tailscale 주소로 띄웠다.
- 출발 정답 위치: TB-6 (0.880, 0.722), TB-5 (2.700, 0.722). 복귀 지점 `robot_a=-0.353,0.000,3.134 robot_b=-0.353,1.82,3.1361`(SIM_HOME=odom, 현장 값과 다름).
- TB-6 `ARMED` 10:17:18. 관제 웹 재난 선포 버튼은 누르지 않았다.

## 앞선 기동 두 번 (자료는 넣지 않음)

| 회차 폴더 | 빌드 | 끝 |
| --- | --- | --- |
| full-1016-100519 | main `78d0c50` | 10:07:18 관제 웹 선포 41 s 뒤 재범 지시(위 원문)로 stop_full |
| full-1016-100958 | #480 head `011f0d3` | run_full `TB-6 arm 실패`, `STOP_OR_READINESS_NOT_READY NAV_NOT_READY missing=['nav2']`(10:12:58). 웹캠 옵션(#481)을 기다리느라 껐다 |

## 단계별 결과

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1 | 통과(웹캠) | 10:19:37.47 webcam_node `SiteEvent DISASTER_SIGN … reversals=5>=4 window_ns=3000000000` → 조정기 `ModeResult(… reason='ENTERED_DISASTER', mode='DISASTER')` → `SEARCH_AUTO_STARTED on DISASTER` → 10:19:39 `SEND SEARCH_AREA` |
| 2 | 통과 | 경계 5개(`EXPLORATION_COMPLETE reason=EXPLORATION_COMPLETE frontiers_sent=5`, 10:26:20, 선포 뒤 6분 43초). 첫 작업 `COMPLETED`. a116 goal (−2.95, 1.96)에서 4/4 회전. 9c32 는 `DEADLINE_EXPIRED`(10:25:17) |
| 3 | **복귀 중 통과** | 탐색 중 `FAKE_TARGET published` 0(아래 표). 복귀 중 10:32:16.99 `FAKE_TARGET published … map=(-2.501, 0.320) … dist=1.34 camera_map=(-3.795, 0.650)` → target_locator `TargetLocation … VALID` → 조정기 `TARGET_LOCATION` |
| 4 | **실행 안 됨** | 10:32:17.11 조정기 `task not issued: SUPPORT_APPROACH_NOT_FREE` → 10:33:47 `SYSTEM_STATE {'mode': 'MISSION_FINISHED' … 'reason': 'SUPPORT_APPROACH_NOT_FREE'`. TB-5 는 움직이지 않았다 |
| (TB-6 복귀) | 통과(가제보 실습 조작 1회 뒤) | 아래 「2차 고정 뒤 TB-6 위치 추정 멈춤」. 10:32:16 gen 15 `Begin navigating from current location (-3.75, 0.61)` → 10:33:46 `Goal succeeded` → `RESULT … COMPLETED`. 정답 위치 (0.996, 0.433), 출발 자리에서 0.31 m |

## 3단계: 탐색 중 인형 보고가 안 나간 이유

`logs/fake_target.log` 의 `FAKE_TARGET not sent` 이유별 줄 수다. 인형 정답 위치는 world (1.20, 2.87)(`config/sim/worlds/arena.layout.json`), 지도 (−2.501, 0.320)이다.

| 이유 | 줄 | 값 |
| --- | --- | --- |
| `OUT_OF_VIEW` | 9 | bearing ±37.1 ~ ±79.3° |
| `NOT_IN_FRONT` | 6 | `depth=0.21 ~ 0.28 m < 0.3` |
| `OUT_OF_RANGE` | 3 | `dist=2.01 ~ 2.24 m > 2.0` |
| `OCCLUDED` | 2 | |
| `NO_START_POSE`·`NO_CAMERA_INFO`·`TF_STALE`·`NO_TF` | 1씩 | 기동 직후·넘김 중 |

- TB-6 정답 궤적(`track/gt_track.txt`)에서 인형과 가장 가까웠던 때는 10:26:18 이다. 거리 1.21 m, TB-6 yaw −139.2°, 인형 방향은 정면에서 +37.8° 였다. 2차 고정으로 선 자리이고, 10:32:16 까지 그대로 있었다.
- 2.0 m 안에서 인형이 정면(0.2°)에 가장 가까웠던 때는 10:25:45, 거리 1.93 m 다. 그 무렵 fake_target 줄은 10:25:39 `OUT_OF_RANGE dist=2.01 m > 2.0` 이다.
- 탐색 중 FRONTIER_LOOK 은 e291 1/4, a116 4/4 다(`FRONTIER_LOOK_SENT` 줄).

## 2차 고정 뒤 TB-6 위치 추정 멈춤 (C22 와 같은 모양)

| 시각 | 줄 | 로그 |
| --- | --- | --- |
| 10:26:20 | `RETURN_HOME` gen 10 ~ 13 (10:26:20 ~ 10:26:51) → 실행기 `ExecuteTask rejected: FRAME_REVISION_MISMATCH … map handover in progress` | `coordinator.log`, `robot_robot_a.log` |
| 10:26:58 | `런북 7.5 2차 고정 뒤 tb6_slam 끄기` | `operator.log` |
| 10:27:10 | `런북 7.5 /robot6/map 발행자 수 0(--no-daemon)` | `operator.log` |
| 10:27:28 | `런북 7.5 frozen_localization robot_a 띄움` | `operator.log` |
| 10:27:32 | `[robot6.amcl.rclcpp]: failed to send response to /robot6/amcl/change_state (timeout)` | `frozen_loc_a.log` |
| 10:27:37 | `Received a 114 X 71 map @ 0.050 m/pix`. 이 파일에 `createLaserObject` 0줄 | `frozen_loc_a.log` |
| 10:27:09 ~ 10:32 | planner `Extrapolation Error … Requested time 1792114018.632935`(10:26:58.63) 298줄 | `robot_robot_a.log` |
| 10:31:07 | `ros2 lifecycle get /robot6/amcl` → `inactive [2]`. base_link→rplidar_link TF 있음, `/robot6/scan` 구독자에 amcl 있음 | `diag_c22_robot6.txt` |
| 10:32:06 | RobotPose 간격 `GAP robot_a 307.3 s (since 10:26:58.74)` | `track/pose_gap.txt` |

조작(`logs/operator_manual.log`): 10:31:32 frozen_localization robot_a 프로세스 그룹에 INT 를 보내고 같은 명령으로 다시 띄웠다(런북 7.5.2). 그 뒤 10:32:03 `Activating amcl`, 10:32:05 `MAP_RELAYED /rescue/map/frozen R2 -> /robot6/map`·`createLaserObject` 가 나왔다(`frozen_loc_a_relaunch.log`). gen 14(10:32:06)는 `FRAME_REVISION_MISMATCH: AMCL map …` 으로 거절됐고, gen 15(10:32:16)가 실행됐다.

## 4단계: TB-5 접근 지점

- 조정기 `approach_point`(`src/rescue_coordination/rescue_coordination/ros_node.py` 304 ~ 318줄)는 인형에서 TB-5 쪽으로 직선 0.8 m(`approach_standoff_m`) 자리를 고른다. TB-5 를 복귀 지점 (−0.353, 1.82)에 두고 계산하면 (−1.845, 0.778)이다 [계산].
- r2 고정 지도(`maps/102620-frozen-r2.*`)에서 그 칸은 미지(205)다. 반지름 0.2 m(`freeze_clearance_m`) 안에 점유 1칸, 미지 48칸이 있다. 가장 가까운 점유 칸은 (−1.627, 0.756), 0.22 m 거리다 [계산].
- 같은 지도에서 인형 둘레 0.8 m 고리 위 0.2 m 안이 모두 빈 방향은 80 ~ 100°, 170 ~ 310°다(10° 간격) [계산].

## 그 밖의 자료

- #465: `logs/tf_merge.log` 10:17:33 `TF_PROVISIONAL robot5: start value (0.0, -1.86, 0.0) from <저장소>/config/field/search_area.field.json` → 10:26:20 `TF_PROVISIONAL_OFF robot5: its own map -> odom came`. 화면 캡처는 없다(Wayland 화면).
- watchdog: 로그 전체에 `WATCHDOG_AUTO_RESET` 0줄, watchdog 잠김도 없었다.
- TB-5 실행기 `Assertion` 0줄. TB-5 `ALIGNED`, 10:27:41 `robot5 task_arm: … ARMED`(`operator.log`).
- run_full 운영자(런북 7.1): 10:25:17 9c32 `EXPIRED` 뒤 refresh_state·`robot6 task_arm: … ARMED`.
- RTF(`track/rtf.txt`, 30 s 간격 34개): 중앙값 0.81, 최저 0.134(10:31:09, load 83.7), 0.5 미만 8개. 탐색 중 load 45 ~ 51.

## 참고: 가제보 실습 의견 (정확하지 않을 수 있음)

아래는 위 자료를 보고 짐작한 것이다. 확인하지 않은 해석이라 틀릴 수 있다.

- TB-5 가 출발하지 않은 것은 15·20회차와 같이, 직선 접근 지점이 인형 옆 구조물 칸에 떨어진 탓으로 보인다. 기둥 축소(#480) 뒤에도 그 점은 같은 자리였다.
- TB-6 위치 추정 멈춤은 AMCL 이 켜지는 도중 change_state 응답 시간 초과로 inactive 에 남은 것으로 보인다. 그때 load 가 높았다.

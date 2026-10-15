# 가제보 실습 여섯째 회차 (full-1016-023619, 2026-10-16 02:36 ~ 02:44, sim1)

측정·로그 자료를 먼저 적는다. 가제보 실습의 해석은 맨 끝 「참고: 가제보 실습 의견」에만 둔다(재범 지시 2026-10-16).

- 빌드: `4850a8f`(main 그대로, `src/`·`config/` diff 0줄). 5회차 빌드(`be4053b`) 뒤 main 에 든 것은 #401(시뮬레이션 사본 TB-5 출발값을 정답 위치로 계산)이다.
- world: sim1(`config/sim/worlds/arena.sdf` 마지막 변경 `08dfbe4`). 흰 상자 없음, 칸막이 그대로, 벽 0.018 m 이어진 상자.
- 현장 값과 다른 시뮬레이션 사본 값(run_full 기동 로그):
  - `robots.robot_b.initial_pose_xy_yaw 현장 [0.0, -1.86, 0.0], 시뮬레이션 사본 [0.016, -1.82, -0.0002] (언도킹 뒤 TB-6 몸 기준 TB-5, 확인용 정답 위치로 잼)`
  - `home_positions 현장 없음(측정 전), 시뮬레이션 robot_a=-0.3529…,-3.2e-11,-2.9489… robot_b=-0.717,1.783,-2.9491 (SIM_HOME=odom, 언도킹 뒤 자세)`(작전 결정 (a))
- 재난 선포: 재범 위임(2026-10-16 02:3x)으로 가제보 실습이 관제 웹 `POST /commands kind=DECLARE_DISASTER`(admin)로 눌렀다. 02:38:45.86 요청, 답 `{"accepted": true, … "request_state": "ACCEPTED", "sent": true}`(`logs/declare_by_sim_practice.log`). 조정기 `DISASTER_DECLARED by operator admin` 은 02:38:46.16.
- Gazebo 는 이 회차 하나, 화면 없음(headless).
- 실습장 좌표는 확인용 정답 위치(`track/gt_track.txt`)로만 썼다. 지도 변환은 출발 자세와 정답 위치로 맞춘 회전 −101.3° 다.

## 단계별 결과

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1. 언도킹으로 시작 | 통과 | `언도킹 robot6` 02:36:55, `언도킹 robot5` 02:37:13 |
| 2. 선포 즉시 TB-6 이 움직여 실습장을 훑음 | **실패** | 보낸 경계 2개 모두 `COMPLETED`. 02:40:57.87 `EXPLORATION_COMPLETE … frontiers_sent=2`. 이동 거리 1.52 m. 정지 구간 02:40:57~끝(TB-6 정답 위치 (1.631, 1.390, 33.0°)). 시간 초과 지도 free 칸 2,662 / 7,642 = 34.8%. 메인 구역(y > 2.46) 정답 위치 표본 0 |
| 3. 웹 지도에 터널·인형이 뜸 | **실패** | `overhead_passages` 0행, `PASSAGE_KEEPOUT 0 place(s)` 1회, `FAKE_TARGET published` 0회 |
| 4. 인형 발견 → TB-5 출발 | 실행 안 됨 | `NO_SURVIVOR_FOUND`(02:40:57.87). TB-5 `ARMED` 02:42:14 |

## 회차 표

| 시각 | 무엇 | 로그 |
| --- | --- | --- |
| 02:36:19 | 시작 | `빌드 4850a8f(src 는 main 과 diff 0줄), 관제 웹 빌드 4850a8f` |
| 02:38:11 | 준비 끝 | `TB-6 ARMED` |
| 02:38:46.16 | 선포·첫 경계 | `DISASTER_DECLARED by operator admin` → `FRONTIER_GOAL_MOVED (-0.14, -0.53) -> (-0.29, -0.43) for 0.35 m clear` → `FRONTIER e291… cells=14 centroid=(0.235, -0.509) path_m=0.63 goal=(-0.29, -0.43, -0.79)` |
| 02:38:48.10 | 첫 작업 | `SEND SEARCH_AREA e88ca865… generation=0` → `ADMISSION … -> True` |
| 02:39:43.05 | 결과 | `RESULT … outcome=COMPLETED reason=NAVIGATION_SUCCEEDED` |
| 02:39:43.22 | 둘째 경계 | `FRONTIER_GOAL_MOVED (-1.41, 0.77) -> (-1.35, 0.76)` → `FRONTIER a116… cells=11 centroid=(-2.251, 1.861) path_m=3.04 goal=(-1.35, 0.76, 2.36)` → 02:39:45.20 `SEND … generation=1` |
| 02:40:55.68 | TB-6 도착 | `Reached the goal!` → `Goal succeeded` → 02:40:56.31 `completed with stop confirmed: re-armed for the next task` |
| 02:40:56.10 | 조정기 | `SCOUT_NOT_READY frontier=a116… stop_reason=NAVIGATION_SUCCEEDED: TB-6 needs task_arm (or Nav2/readiness) before its next frontier` |
| 02:40:56.55 | 결과 | `RESULT … COMPLETED` |
| 02:40:56.67~02:40:57.87 | 탐색 끝 | `FRONTIER none on map update 181~183: frontiers=1 small=0 no_goal=0 tried=1 no_path=0 too_close=0 \| TRIED cells=11 centroid=(-2.25, 1.86)` → `EXPLORATION_COMPLETE reason=EXPLORATION_COMPLETE frontiers_sent=2` |
| 02:40:57.92 | 지도 고정 | `MAP_FROZEN 1`·`MAP_FROZEN 2`(sha `e877b4…`), TB-6 `MAP_REVISION_ADOPTED robot_a RNone -> R2` |
| 02:40:57.94 | 복귀 1 | `SEND RETURN_HOME 37b6ec0a… generation=2` → `ADMISSION … -> False` → `GOAL task=37b6ec0a… accepted=False (ACTIVE only from task_admission)` → TB-6 `ExecuteTask rejected: INVALID_INPUT: INVALID_INPUT [rescue_robot.INVALID_INPUT]: native frame` → `task not issued: TASK_REFUSED` |
| 02:40:57~02:41:03 | TB-5 | `START_ALIGN_ALIGNING first_guess=[-0.717, 1.783, -2.949]` → `START_ALIGN_ALIGNED … "jump_xy_m": 0.008` → `MAP_STATE robot_b … VALID` |
| 02:41:01~ | TB-6 실행기 | `robot_a map is not the frozen map; not loaded` 33회 |
| 02:41:08.01 | 조정기 | `TRAFFIC_HOLD SCOUT TRAFFIC_SCOUT_POSE_UNKNOWN` → 02:41:35.10 `TRAFFIC_RELEASE SCOUT` |
| 02:41:28~02:42:08 | 런북 7.5(운영자 대역) | 02:41:28 SLAM TERM → 02:41:44 `/robot6/map 발행자 수 0` → 02:42:01 `frozen_localization robot_a` → 02:42:08 `MAP_RELAYED /rescue/map/frozen R2 -> /robot6/map` |
| 02:42:09~02:43:40 | 복귀 2~12 | `SEND RETURN_HOME … generation=3`~`12`, 약 10 s 간격, 모두 `ADMISSION -> False`·`TASK_REFUSED`. TB-6 `native frame` 거절 합계 11회 |
| 02:42:14 | TB-5 arm(운영자 대역) | `robot5 task_arm: success=True message='ARMED` |
| 02:43:43~02:44:02 | 끄기 | `stop_full.sh`. 이 구간의 Traceback 은 종료 신호 뒤 |

## RobotPose·접촉·속도·RTF

- RobotPose(`track/pose_gap.txt`):
  - TB-6: 2 Hz. 끊김은 02:40:57.67 → 02:42:08.74 의 71.1 s 한 번(7.5 구간).
  - TB-5: 02:41:03 `VALID` 뒤 끊김 0.
- 접촉(정답 위치, 몸 중심 최소 거리): TB-6 도크 0.51 m(출발), TB-5 도크 1.59 m, TB-5 1.26 m, 인형 1.54 m, 바깥 벽 0.59 m.
- Nav2(robot_robot_a.log): `PassageSlow` 14, `Failed to make progress` 9, `detected collision ahead` 0, `spin failed` 0, `invalid source` 0.
- 속도(정답 위치 0.5 s 차분, 선포~탐색 끝): 움직인 표본 76/263, 평균 0.040 m/s, 최고 0.155 m/s.
- RTF(30 s, 11표본): 중앙값 0.71, 최소 0.19, 최대 1.37, 0.5 미만 4개. 부하 25~38.
- 인형 대역 안 보낸 사유: `OUT_OF_RANGE` 2, `OUT_OF_VIEW` 2, `NO_START_POSE`·`NO_TF`·`NOT_IN_FRONT`·`TF_STALE` 각 1.

## 5회차와 같은 것·다른 것

- 같은 것: 첫 두 경계와 그 목표(±0.01 m), 두 경계 모두 `COMPLETED`, `FRONTIER none … tried=1 | TRIED cells=11 centroid=(-2.25, 1.86)` → `EXPLORATION_COMPLETE frontiers_sent=2`, TB-6 최종 위치(5회차 (1.63, 1.39), 6회차 (1.631, 1.390)).
- 다른 것:
  - TB-5 `START_ALIGN_ALIGNED jump_xy_m`: 5회차 0.323 → 6회차 0.008 m(#401 뒤 first_guess (−0.717, 1.783)).
  - `SIM_HOME=odom` 으로 `NO_HOME_POSITION` 대신 `SEND RETURN_HOME` 이 나갔다. TB-6 실행기에서 11번 모두 `native frame` 으로 거절됐다.

## 파일

| 파일 | 무엇 |
| --- | --- |
| `logs/*.log` | 회차 폴더 원문(색 코드 지움, 경로·PC 이름 바꿈, 줄인 줄 없음). `declare_by_sim_practice.log` 는 선포 요청과 답 |
| `logs/diag_*.txt` | 운영자 대역이 남긴 7.5 앞뒤 자료, 1차 고정 뒤 TB-5 Nav2 lifecycle |
| `maps/<시각>-live-r0-…` | 선포 직전(023745)부터 탐색 끝까지 TB-6 SLAM 저장본, 시각 순서 |
| `maps/timeout-…`, `maps/frozen-r2-…` | 탐색 끝 순간 마지막 저장본, 2차 고정 지도와 메타 |
| `track/gt_track.txt`, `pose_gap.txt`, `rtf.txt` | 정답 위치 0.5 s, RobotPose 끊김·상태, RTF·부하 30 s |
| `video/run6_gt_vs_slam.mp4` | 기록으로 그린 영상(2 s 간격, 10 fps). 왼쪽은 정답 위치 평면도, 오른쪽은 그 시각 SLAM 저장본 위에 정답 위치를 지도 좌표로 바꾼 로봇과 조정기 목표. Gazebo·RViz 화면 녹화가 아니다 |

## 참고: 가제보 실습 의견 (정확하지 않을 수 있음)

아래는 위 자료를 보고 짐작한 것이다. 확인하지 않은 해석이라 틀릴 수 있다.

- C11(도착한 경계를 `TRIED` 로 빼서 탐색이 끝남)이 5회차와 똑같이 재현됐다. 조정기 쪽 동작으로 보인다.
- 복귀 거절(`native frame`)은 시뮬레이션 사본의 `home_positions`(언도킹 뒤 TB-6 odom 자세를 그대로 넣음)가 실행기가 기대하는 좌표계와 맞지 않아서일 수 있다. 현장처럼 고정 지도에서 잰 값이면 다를 수 있어 「모름」으로 둔다.
- 7.5 끊김 71.1 s 중 운영자 대역의 자료 수집이 큰 몫이고, SLAM 이 아직 지도를 내는 동안 `map is not the frozen map` 이 33회 찍힌 것도 같은 구간이다.

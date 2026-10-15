# 가제보 실습 열다섯째 회차 (full-1016-073537, 2026-10-16 07:35 ~ 07:44, sim1)

측정·로그 자료를 먼저 적는다. 가제보 실습의 해석은 맨 끝 「참고: 가제보 실습 의견」에만 둔다(재범 지시 2026-10-16).

- 빌드: `100a610`(main, `src/`·`config/` diff 0줄). 14회차 빌드(`a5d4d64`) 뒤 들어간 것:
  - #454(Nav2 속도 Create3 최대)
  - #456(NTP spike 받기)
  - #457(sim 센서 주기, RTF)
  - #458(정렬 벽 넘김)
  - #459(sim1 터널 1 넓힘)
- world: sim1(`arena.sdf`, #459 뒤).
- 현장 값과 다름: `SIM_HOME=odom`. 14회차와 같다.
- 시간 동기: 기동 직전 timesyncd spike=false 였다. `logs/timesync_status_at_start.txt` 는 Offset +3.0 ms, Delay 198.6 ms 다.
- 기동: run_full 이 TB-6 `ARMED` 까지 손 없이 끝냈다(07:37:28).
- 재난 선포: 재범 위임(02:3x, 06:5x 에 09:30 까지 연장)으로 가제보 실습이 관제 웹 `POST /commands kind=DECLARE_DISASTER`(admin) 를 보냈다. `accepted: true`, 07:38:23.
- 녹화: 07:38:20 ~ 07:44:05. 손으로 한 조작은 없다.

## 단계별 결과

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1. 언도킹으로 시작 | 통과 | run_full 이 두 대를 언도킹했다 |
| 2. 선포 즉시 TB-6 이 움직여 실습장을 훑음 | **통과(처음)** | 경계 2개를 모두 `COMPLETED` 했다. 터널 1 을 지나 정답 위치 최대 y 3.146(07:39:21)까지 갔다. 이전 회차 최대는 1.80 이다. 이동 거리 4.38 m |
| 3. 웹 지도에 터널·인형이 뜸 | **통과(처음, 장부 기준)** | 장부 `target_states` 1(`detection_confirmed: true`), `overhead_passages` 3(PASSABLE 2, BLOCKED 1). 웹 화면은 따로 확인하지 않았다 |
| 4. 인형 발견 → TB-5 출발 | **실패** | 인형 발견(07:39:38) 뒤 TB-5 출발 정렬 `ALIGNED`(07:40:05)·`ARMED`(07:41:02). 07:41:02 부터 조정기 `task not issued: SUPPORT_APPROACH_NOT_FREE` 로 TB-5 에 명령이 가지 않았다 |
| (TB-6 복귀) | 실패 | RETURN_HOME 이 두 번 Nav2 까지 갔지만 `localization uncertain` 으로 취소돼 `SCOUT_RETURN_FAILED`(07:41:46.99) |

## 회차 표

| 시각 | 로그 |
| --- | --- |
| 07:38:23.11 | `FRONTIER_GOAL_PAST_TIGHT (-0.76, 1.36) -> (-0.97, 2.02) for 0.35 m clear and 0.6 m from TB-5` → `FRONTIER e291… cells=70 … goal=(-0.97, 2.02, 1.57)` → 07:38:25.07 `SEND … generation=0` |
| 07:38:39.04 | `RESULT … COMPLETED`(14 s) → `FRONTIER_LOOK_SENT … turn 1/4` |
| 07:38:41.21 | `TRAFFIC_YIELD 7bed… from=(-0.94, 1.78) to=(-1.24, 1.77, -3.11) path_m=0.30` → 07:38:43.27 `SEND … generation=1` → 07:39:02.99 `COMPLETED` |
| 07:38:58 | `overhead PASSABLE lowest=0.465 map=(-1.69, 2.21)` |
| 07:39:03.20 | `FRONTIER_GOAL_PAST_TIGHT (-2.46, 1.91) -> (-3.00, 1.96) …` → `FRONTIER a116… cells=37 … path_m=2.06 goal=(-3.00, 1.96, 2.36)` → 07:39:05.17 `SEND … generation=2` |
| 07:39:06 ~ 07:39:21 | 정답 위치 TB-6 (2.705, 1.477) → (2.869, 1.895) → (2.883, 2.818) → (2.851, 3.146). 터널 1 을 지남(`detected collision ahead` 0) |
| 07:39:21.57 | `RESULT … COMPLETED` → `FRONTIER_LOOK_SENT … turn 1/4 at a116…` → 07:39:29.49 `FRONTIER_LOOK 1/4 … COMPLETED` → `turn 2/4` |
| 07:39:38.14 | `FAKE_TARGET published … map=(-2.501, 0.320) sensor=(-0.921, 0.144, 1.459) dist=1.73 camera_map=(-2.750, 2.027) bbox=(9, 127, 31, 157) … clock=SYNCED` → 조정기 `TARGET_LOCATION … -> 1f7000cc…` → `phase TARGET_FOUND` |
| 07:39:53.99 | `FRONTIER_LOOK 2/4 … COMPLETED` → `FRONTIER_LOOK_DONE a116… turned 2/4: a doll was reported` |
| 07:39:54.07 ~ .27 | `MAP_FREEZE 1 requested: target saved, TB-6 stopped` → `TARGET_REBASED_ON_FREEZE … xy=(-2.5007, 0.3200)` → `task not issued: SUPPORT_NOT_LOCALIZED` |
| 07:39:54.24 ~ 07:39:55.59 | `FRONTIER none on map update 167 ~ 169: frontiers=7 small=2 … no_path=5` → `EXPLORATION_COMPLETE reason=EXPLORATION_COMPLETE frontiers_sent=2` |
| 07:39:55.59 ~ .62 | `MAP_FREEZE 2` → `SEND RETURN_HOME generation=5` → `MAP_RELAYED … R2 -> /robot5/map` |
| 07:39:59.72 | TB-5 RobotPose `VALID mission=(-0.402, 1.765)` |
| 07:40:05 | TB-5 `START_ALIGN_ALIGNED`(jump 0.074 m) |
| 07:40:05.67 ~ 07:40:36.17 | `task not issued: SUPPORT_NOT_READY` 4회(TB-5 arm 전) |
| 07:40:26.58 ~ 07:41:04.26 | TB-6 RobotPose 끊김 37.7 s(운영자 대역이 SLAM 끄고 07:40:54 frozen_localization 기동, 07:41:02 `MAP_HANDOVER`) |
| 07:41:02 | 운영자 대역 robot5 `task_arm` `ARMED` → 조정기 `task not issued: SUPPORT_APPROACH_NOT_FREE` |
| 07:41:14.67 | `SEND RETURN_HOME generation=10` `ADMISSION -> True` / `task not issued: SUPPORT_APPROACH_NOT_FREE` |
| 07:41:24.09 ~ .99 | TB-6 RobotPose `UNKNOWN` 0.07 s → `localization uncertain` → `RESULT … GUARD_INVALID` → `SCOUT_RETURN_AGAIN … failed 1/2` |
| 07:41:35.07 | `SEND RETURN_HOME generation=11` |
| 07:41:46.09 ~ .99 | TB-6 RobotPose `UNKNOWN` 0.07 s → `localization uncertain` → `RESULT … GUARD_INVALID` → `SCOUT_RETURN_FAILED` |
| 07:41:47 ~ 07:44:05 | `SEND` 없음. `SYSTEM_STATE … reason SUPPORT_APPROACH_NOT_FREE` |
| 07:44:05 ~ 07:44:25 | 녹화 끔 → `stop_full.sh`(07:44:07 뒤 Traceback 은 종료 신호 뒤) |

## TB-5 출동 막힘 (`SUPPORT_APPROACH_NOT_FREE`)

- 조정기 코드(`ros_node.py` `100a610`):
  - `approach_point` 304 ~ 318줄: 인형에서 TB-5 쪽으로 `approach_standoff_m` 떨어진 직선 위의 점이다.
  - `_issue_approach` 2331줄: 그 점이 고정 지도에서 `freeze_clearance_m` 안이 비어 있지 않으면 `SUPPORT_APPROACH_NOT_FREE` 로 멈춘다.
  - 기동 줄 값은 `approach_standoff_m 0.8`, `freeze_clearance_m 0.2` 다.
- 계산[계산]:
  - 인형 `TARGET_REBASED_ON_FREEZE xy=(-2.5007, 0.3200)`, TB-5 RobotPose `VALID mission=(-0.402, 1.765)`(07:39:59.72), 거리 2.548 m.
  - 접근 지점은 mission (−1.842, 0.774)이고, 정답 좌표로 (1.654, 2.208)이다. 바꾸는 식은 출발 자세 쌍으로 맞춘 (x_gt = y + 0.880, y_gt = 0.366 − x)다.
  - 그 점은 world 흰 상자(x 1.391 ~ 2.759, y 2.001 ~ 2.469) 안이다.
- 고정 지도 R2(`maps/073955-frozen-r2`)에서 접근 지점 둘레 0.2 m(9×9 칸)는 free 0, occupied 16, unknown 65 다.
- 같은 고정 지도에서 인형 둘레 0.8 m 원을 10° 마다 보면, 둘레 0.2 m 가 모두 free 인 방향은 80·90·240 ~ 310° 다(mission 기준).
- 인형 위치: `FAKE_TARGET` map (−2.501, 0.320)을 정답 좌표로 바꾸면 (1.200, 2.867)이고, world 인형 (1.20, 2.87)과 같다. 시뮬레이션 인형 대역은 정답 위치로 관측을 만든다(model_id `sim`).

## 탐색·터널 (#459·#457·#454 뒤)

- 터널 1: `detected collision ahead` 0. 07:39:06 ~ 07:39:16 에 지났다.
- 장부 `overhead_passages`(`logs/ledger_targets_passages.json`):
  - PASSABLE (−2.052, 2.154) lowest 0.467 rev 0, frames 63
  - PASSABLE (−1.705, 2.093) lowest 0.467 rev 2, frames 18
  - BLOCKED (−2.403, 1.225) lowest 0.299 rev 0, frames 24. world 터널 2 지붕은 z 0.30 ~ 0.32 다.
- 탐색 끝: 인형 보고 뒤 `FRONTIER none … frontiers=7 small=2 no_path=5`(`NO_PATH cells=40 centroid=(-3.12, -0.05)` 등) 3회 → `EXPLORATION_COMPLETE frontiers_sent=2`.
- 첫 경계까지 14 s 걸렸다(14회차 20 s, 12회차 28 s).
- RTF(30 s, 13표본): 중앙값 1.00, 최소 0.74, 최대 1.01, 0.5 미만 0. 14회차는 최소 0.24, 0.5 미만 4 였다. 부하는 26 ~ 63 이다.

## TB-6 복귀

- RETURN_HOME 10·11 은 `ADMISSION -> True` 였다. 07:41:24.09 와 07:41:46.09 에 RobotPose 가 각 0.07 s `UNKNOWN` 이었고, 그때마다 실행기가 `task guard rejected … localization uncertain` 로 취소했다.
- `Initial robot pose is not available` 은 0, `AMCL_NOMOTION` 은 0 이다.
- TB-6 정답 위치는 마지막에 (2.926, 2.542)로, 터널 1 너머에 남았다.

## 부하 (`track/proc_load.txt`, 07:37:51 ~ 07:44:25, 14장, 평균 %CPU)

| 프로세스 | 평균 | 최대 |
| --- | --- | --- |
| `gz sim arena.sdf` | 128 | 158 |
| `gz sim -g`(녹화용 화면) | 89 | 118 |
| `wall_clock_relay.py` | 87 | 100 |
| `task_input` | 78 | 92 |
| `parameter_bridge` | 60 | 118 |
| `rviz2` | 50 | 92 |

## 그 밖의 자료

- TB-6 ↔ 인형 최소 1.648 m(07:41:40.76).
- 인형 대역 안 보낸 사유: `NOT_IN_FRONT`·`OUT_OF_RANGE`·`OUT_OF_VIEW` 각 2, `NO_START_POSE` 1. 보낸 것 1.
- 선포 직전 지도 `maps/073822-before-declare`(55×70): free 2646, unknown 949, occupied 255.

## 14회차와 같은 것·다른 것

- 다른 것:
  - 터널 1 을 지났다. 14회차는 입구 (2.814, 1.628)에서 `collision ahead` 127 이었고, 15회차는 0 이다.
  - 인형 관측 0 → 1, 장부 target 1, 4단계까지 갔다.
  - RTF 0.5 미만 표본 4 → 0 이다.
- 같은 것: TB-6 복귀가 짧은 `UNKNOWN` 에 `localization uncertain` 으로 두 번 취소돼 `SCOUT_RETURN_FAILED`, TB-5 `ALIGNED`.

## 파일

| 파일 | 무엇 |
| --- | --- |
| `logs/*.log` | 회차 폴더 원문(색 코드 지움, 경로·PC 이름 바꿈, 줄인 줄 없음) |
| `logs/ledger_targets_passages.json` | 장부 `target_states`·`overhead_passages` 행 |
| `logs/diag_tf_handover_robot6.txt` | 07:40:20 ~ 07:42:00 map→odom·amcl_pose·odom→base_link |
| `logs/diag_c7_*`, `logs/diag_c8_tb5_nav2.txt`, `logs/diag_tb5_scan_at_align*.txt` | 운영자 대역 자료, TB-5 정렬 라이다 |
| `maps/` | 선포 직전, 1·2차 고정 지도 |
| `track/` | 정답 위치, RobotPose, RTF, 프로세스별 부하 |
| `video/screen_gazebo_rviz_small.mp4`, `video/shot_*.png` | 녹화 줄인 판과 녹화 시작(07:38:20) 기준 장면 6장: 3 s 선포, 52 s 터널 1 지남, 78 s 인형 관측, 95 s 고정, 165 s 넘김, 206 s 복귀 취소 |

## 참고: 가제보 실습 의견 (정확하지 않을 수 있음)

아래는 위 자료를 보고 짐작한 것이다. 확인하지 않은 해석이라 틀릴 수 있다.

- TB-5 출동 막힘은 접근 지점을 TB-5 쪽 직선 한 점만 보고, 그 점이 흰 상자 안이라 멈춘 것으로 보인다. 같은 거리의 다른 방향은 비어 있었다. 현장에서도 인형과 TB-5 사이에 벽이 있으면 같을 수 있어 보인다.
- 복귀 취소는 14회차와 같은 모양이다(0.07 s `UNKNOWN`).
- 터널 1 통과와 RTF 개선은 #459·#457 뒤라 시뮬레이션 쪽 고침의 효과로 보인다.

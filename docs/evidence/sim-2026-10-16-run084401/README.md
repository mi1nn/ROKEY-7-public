# 가제보 실습 열여덟째 회차 (full-1016-084401, 2026-10-16 08:44 ~ 08:55, sim1 새 arena)

측정·로그 자료를 먼저 적는다. 가제보 실습의 해석은 맨 끝 「참고: 가제보 실습 의견」에만 둔다(재범 지시 2026-10-16).

- **빌드·world**: PR #462 head `8270540`(머지 전). 재범 「지도도 바뀌었자너」(08:4x)에 따라 사진대로 다시 만든 arena(터널 1·2 세운 판, 벽 받침, 바닥 색)로 돌렸다.
  - #462 는 `100a610` 기준이다. main(`c28abcb`)과 차이는 `config/sim/`(README·scenario·`arena.layout.json`·`arena.sdf`)와 `scripts/sim/map_to_world.py` 뿐이고, `src/` 는 같다.
- 그 전에 옛 arena(main `c28abcb`)로 띄운 기동(08:41)은 선포 전에 껐다(`logs/old_arena_start_stopped_before_declare.txt`).
- 기동 뒤 TB-6 arm 이 watchdog `STOP_LATCHED EXECUTOR_HEARTBEAT_STALE` 로 실패했다. 가제보 실습이 런북 48행대로 손으로 풀었다: reset → refresh_state → `task_arm` `ARMED`(08:47:04, `logs/operator_manual.log`).
- 재난 선포: 재범 위임으로 08:47:37(`accepted: true`).

## 단계별 결과

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1 | 통과 | |
| 2 | 통과(새 arena) | 경계 3개 `COMPLETED`, 터널 1 통과(08:48:34 a116 도착), 정답 위치 TB-6 (2.874, 4.807) |
| 3 | 통과(장부 기준) | 08:49:07 `FAKE_TARGET published … map=(-2.501, 0.320) … dist=1.74`. 장부 `target_states` 1, `overhead_passages` 1(`logs/ledger_targets_passages.json`). 터널 판정 `overhead PASSABLE` 4줄(lowest 0.467 ~ 0.469 m, map x −1.85), `NO_FLOOR_SEEN` 0 |
| 4 | **실패** | 2차 고정 순간 TB-5 실행기 중단(16·17회차에 이어 세 번째) → `SUPPORT_NOT_LOCALIZED` |
| (TB-6 복귀) | 실패 | 08:51:52 `GridBased plugin failed to plan from (0.00, 0.00) to (-0.00, -0.00): "Unable to get start pose"` → `ACTION_ABORTED` → 다시 보냄 → 08:54:48 `localization uncertain` → `SCOUT_RETURN_FAILED` |

## 새 arena 에서 TB-5 출발 정렬

- 첫 정렬: 08:49:11(stamp) `START_ALIGN_FAILED START_ALIGN_LOW_MATCH`, `match_ratio 0.6688`, `match_points 625`, `match_points_off_map 0`, `jump_xy_m 0.081`(`logs/frozen_loc_b.log`). 그 순간 `/robot5/scan` 은 `logs/diag_tb5_scan_at_lowmatch.txt` 에 있다.
- 08:51:09 가제보 실습이 런북 7단계 `/robot5/rescue_start_align/restart` 를 1회 불렀다(`success=True`).
- 08:51:13 `START_ALIGN_ALIGNED`, `match_ratio 0.9792`, `jump_xy_m 0.014`.

## TB-5 실행기 중단 (세 번째)

- `logs/robot_robot_b.log`: 2차 고정(08:50:56) 순간 `robot_b map is not the frozen map; not loaded` → `geometry_msgs__msg__pose__convert_from_py: Assertion … failed` → `process has died … exit code -6 … task_input … __ns:=/robot5`.
- 그 뒤 웹 지도에는 「TB-5: 위치 미확정」이 나왔다(재범 화면 08:51).

## 파일

| 파일 | 무엇 |
| --- | --- |
| `logs/*.log` | 회차 폴더 원문(색 코드 지움, 경로·PC 이름 바꿈) |
| `logs/ledger_targets_passages.json` | 장부 target·터널 행 |
| `logs/diag_tb5_scan_at_lowmatch.txt` | 첫 정렬 실패 직후 TB-5 라이다 |
| `maps/`, `track/` | 1·2차 고정 지도, 정답 위치·RobotPose·RTF·프로세스별 부하 |
| `video/` | 녹화 줄인 판, 녹화 시작(08:47:37) 기준 90 s(인형 관측 뒤), 200 s(복귀 실패) 장면 |

## 참고: 가제보 실습 의견 (정확하지 않을 수 있음)

아래는 위 자료를 보고 짐작한 것이다. 확인하지 않은 해석이라 틀릴 수 있다.

- 새 arena 에서도 탐색·터널 통과·인형 관측은 옛 arena 와 같게 됐다.
- 첫 정렬 `LOW_MATCH` 는 새로 생긴 벽 받침 등과 관계있을 수 있지만, 다시 시작하자 0.98 로 맞았다.
- 복귀 `Unable to get start pose` 는 넘김 직후 Nav2 계획기에 TB-6 위치가 아직 없었던 것으로 보인다.

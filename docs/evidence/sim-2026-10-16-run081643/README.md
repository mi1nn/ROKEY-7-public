# 가제보 실습 열일곱째 회차 (full-1016-081643, 2026-10-16 08:16 ~ 08:25, sim1)

측정·로그 자료를 먼저 적는다. 가제보 실습의 해석은 맨 끝 「참고: 가제보 실습 의견」에만 둔다(재범 지시 2026-10-16).

- 빌드: `c28abcb`(main). `src/`·`config/`·`scripts/sim/` 는 15·16회차(`100a610`)와 같다. 재범 「테스트할껀 빨리 테스트해」(08:0x)에 따라 #462·#463 머지 전에 같은 조건으로 한 번 더 돌렸다.
- 기동: TB-6 `ARMED` 08:18:33. 재난 선포는 재범 위임으로 08:19:15 에 했다(`accepted: true`). 녹화는 08:19:15 ~ 08:24:52 이다. 손으로 한 조작은 없다.

## 단계별 결과

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1 | 통과 | |
| 2 | 통과(세 번째) | 경계 3개 `COMPLETED`. 정답 위치 최대 y 4.638, 이동 거리 5.32 m |
| 3 | 통과(장부 기준, 세 번째) | 조정기 `TARGET_LOCATION` 08:20:39.86. 장부 `target_states` 1, `overhead_passages` 2(`logs/ledger_targets_passages.json`) |
| 4 | **실패** | TB-5 `ARMED`(08:21:47) 뒤 `task not issued: SUPPORT_APPROACH_NOT_FREE` 4회(15회차와 같음). 2차 고정 순간 TB-5 실행기가 중단됐다(16회차와 같음) |
| (TB-6 복귀) | 실패 | `localization uncertain` 2회 → `SCOUT_RETURN_FAILED`(08:24:31.36) |

## TB-5 실행기 중단 (두 번째, 16회차와 같은 줄)

`logs/robot_robot_b.log`:

```text
[task_input-14] [INFO] [1792106559.980322544] [robot5.rescue_command_adapter]: MAP_FROZEN R2 sha256=7fd1dd71… 113x70 res=0.05000000074505806
[task_input-14] [INFO] [1792106559.981953622] [robot5.rescue_command_adapter]: MAP_REVISION_ADOPTED robot_b R1 -> R2 sha256=7fd1dd71…
[task_input-14] [INFO] [1792106559.982670791] [robot5.rescue_command_adapter]: MAP_STATE robot_b snapshot=7bb96144-… rev=2 quality=UNKNOWN
[task_input-14] [ERROR] [1792106559.983071270] [robot5.rescue_command_adapter]: robot_b map is not the frozen map; not loaded
[task_input-14] python3: ./.obj-x86_64-linux-gnu/rosidl_generator_py/geometry_msgs/msg/_pose_s.c:54: geometry_msgs__msg__pose__convert_from_py: Assertion `strncmp("geometry_msgs.msg._pose.Pose", full_classname_dest, 28) == 0' failed.
```

- 1792106559.98 = 08:22:39.98. 조정기 `MAP_FREEZE 2 requested` 는 08:22:39.97 이었다. 뒤이어 `process has died … exit code -6 … task_input … __ns:=/robot5`.
- 16회차와 다른 점: 이번에는 TB-5 가 R1 에서 이미 `VALID`(08:21:20.70)·`ARMED`(08:21:47) 상태였다. 16회차는 R1 직후 아직 `UNKNOWN` 이었다.
- 회차별로 보면 15회차(`100a610`)는 2차 고정이 1차 고정 1.5 s 뒤에 왔고 중단이 없었다. 16·17회차는 둘 다 2차 고정 순간 중단이 났다.

## 그 밖의 자료

- TB-5 접근 지점 막힘은 15회차와 같다(근거·계산은 15회차 자료 README 「TB-5 출동 막힘」).
- RTF(30 s, 13표본): 중앙값 1.00, 최소 0.99, 0.5 미만 0.
- TB-6 은 이번에 세 번째 경계 f0bf(goal (−4.51, 1.97))까지 갔다. 정답 위치 y 4.638 은 지금까지 가장 멀다.

## 파일

| 파일 | 무엇 |
| --- | --- |
| `logs/*.log` | 회차 폴더 원문(색 코드 지움, 경로·PC 이름 바꿈) |
| `logs/ledger_targets_passages.json` | 장부 target·터널 행 |
| `maps/`, `track/` | 1·2차 고정 지도, 정답 위치·RobotPose·RTF·프로세스별 부하 |
| `video/` | 녹화 줄인 판, 녹화 시작(08:19:15) 기준 84 s(인형 관측 뒤), 205 s(2차 고정·TB-5 실행기 중단) 장면 |

## 참고: 가제보 실습 의견 (정확하지 않을 수 있음)

아래는 위 자료를 보고 짐작한 것이다. 확인하지 않은 해석이라 틀릴 수 있다.

- TB-5 실행기 중단은 2차 고정(R2) 순간 `not loaded` 처리에서 매번 나는 쪽에 가까워 보인다(16·17회차 연속). 15회차만 비켜 간 것 같다.

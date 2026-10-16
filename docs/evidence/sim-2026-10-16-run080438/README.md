# 가제보 실습 열여섯째 회차 (full-1016-080438, 2026-10-16 08:04 ~ 08:12, sim1)

측정·로그 자료를 먼저 적는다. 가제보 실습의 해석은 맨 끝 「참고: 가제보 실습 의견」에만 둔다(재범 지시 2026-10-16).

- 빌드: `100a610`(main, `src/`·`config/` diff 0줄). 15회차와 같은 빌드·world 로 다시 돌렸다(재범 「그래도 너는 실습을 계속 돌려야지」 2026-10-16 08:0x). 작전은 실물과 다른 arena 를 다시 만들 때까지 보류하라고 했지만, 재범 지시를 따랐다.
- 현장 값과 다름: `SIM_HOME=odom`. 시간 동기: 기동 직전 timesyncd spike=false 였다.
- 기동: run_full 이 TB-6 `ARMED` 까지 손 없이 끝냈다(08:06:31). 재난 선포는 재범 위임으로 08:07:21 에 했다(`accepted: true`). 녹화는 08:07:19 ~ 08:12:20 이다.
- 손으로 한 조작은 없다.

## 단계별 결과

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1. 언도킹으로 시작 | 통과 | |
| 2. 선포 즉시 TB-6 이 움직여 실습장을 훑음 | 통과(15회차 재현) | 경계 2개 `COMPLETED`, 터널 1 통과, 정답 위치 최대 y 3.137, 이동 거리 3.76 m |
| 3. 웹 지도에 터널·인형이 뜸 | 통과(장부 기준, 재현) | 08:08:35 `FAKE_TARGET published … map=(-2.501, 0.320) … dist=1.75`. 장부 `target_states` 1, `overhead_passages` 2(`logs/ledger_targets_passages.json`) |
| 4. 인형 발견 → TB-5 출발 | **실패(새 원인)** | TB-5 실행기(`/robot5/rescue_command_adapter`)가 08:08:54 에 죽었다. 조정기 `task not issued: SUPPORT_NOT_LOCALIZED` 15회 |
| (TB-6 복귀) | 실패 | 08:10:21.94 RobotPose `UNKNOWN` 0.03 s → `localization uncertain` → `GUARD_INVALID` → `SCOUT_RETURN_AGAIN` |

## TB-5 실행기 중단 (새 막힘)

`logs/diag_tb5_executor_abort.txt`, `logs/robot_robot_b.log`:

```text
[task_input-14] [INFO] [1792105734.030571327] [robot5.rescue_command_adapter]: MAP_FROZEN R2 sha256=6f7261d5… 113x70 res=0.05000000074505806
[task_input-14] [INFO] [1792105734.032635896] [robot5.rescue_command_adapter]: MAP_REVISION_ADOPTED robot_b R1 -> R2 sha256=6f7261d5…
[task_input-14] [INFO] [1792105734.033705451] [robot5.rescue_command_adapter]: MAP_STATE robot_b snapshot=e89852cf-… rev=2 quality=UNKNOWN
[task_input-14] [ERROR] [1792105734.034138627] [robot5.rescue_command_adapter]: robot_b map is not the frozen map; not loaded
[task_input-14] python3: ./.obj-x86_64-linux-gnu/rosidl_generator_py/geometry_msgs/msg/_pose_s.c:54: geometry_msgs__msg__pose__convert_from_py: Assertion `strncmp("geometry_msgs.msg._pose.Pose", full_classname_dest, 28) == 0' failed.
[ERROR] [task_input-14]: process has died [pid 1196927, exit code -6, cmd '…/rescue_robot/lib/rescue_robot/task_input --ros-args -r __node:=rescue_command_adapter -r __ns:=/robot5 …
```

- 시각: 1792105734.034 = 08:08:54.034. 2차 고정(`MAP_FREEZE 2`, 08:08:54)과 같은 순간이다.
- 그 뒤 TB-5 RobotPose 는 `UNKNOWN`(08:08:52.01)에서 끝까지 바뀌지 않았다(`track/pose_gap.txt`).
- TB-5 출발 정렬은 `START_ALIGN_ALIGNED`(08:08:56, `frozen_loc_b.log`)였다.
- 운영자 대역 robot5 `task_arm`(08:10:47)은 답이 빈칸이었다(`logs/operator.log`).
- `/robot5/amcl_pose` 는 나왔다(08:11:25 확인, stamp 08:10:01).
- 다른 회차와 비교했다. 7 ~ 16회차 모두 `robot_b map is not the frozen map; not loaded` 가 1줄씩 있다. Assertion·중간 중단은 16회차뿐이다. 15회차 TB-5 실행기는 `stop_full` 때(exit code 1) 끝났다. 검색어는 `robot_robot_b.log` 의 `map is not the frozen map`, `Assertion`, `task_input-14]: process has died` 다.

## 그 밖의 자료

- 15회차와 같은 것:
  - 첫 경계 14 s, 터널 1 통과, a116 경계에서 회전 2/4 때 인형 관측
  - 인형 위치 (−2.5007, 0.3200)
  - TB-6 복귀 `localization uncertain`
- RTF(30 s, 11표본): 중앙값 1.00, 최소 0.17, 0.5 미만 2. 부하 24 ~ 47.
- TB-6 RobotPose 끊김: 08:09:29.43 ~ 08:10:01.96 의 32.5 s(SLAM 끄기 → AMCL).
- 선포 직전 지도 `maps/080721-before-declare`(55×70): free 2637, unknown 963, occupied 250.

## 파일

| 파일 | 무엇 |
| --- | --- |
| `logs/*.log` | 회차 폴더 원문(색 코드 지움, 경로·PC 이름 바꿈, 줄인 줄 없음) |
| `logs/diag_tb5_executor_abort.txt` | TB-5 실행기 마지막 줄과 중단 줄 |
| `logs/ledger_targets_passages.json` | 장부 target·터널 행 |
| `logs/diag_tf_handover_robot6.txt`, `logs/diag_c7_*`, `logs/diag_c8_tb5_nav2.txt` | 넘김 TF, 운영자 대역 자료 |
| `maps/`, `track/` | 선포 직전·고정 지도, 정답 위치·RobotPose·RTF·프로세스별 부하 |
| `video/` | 녹화 줄인 판과 녹화 시작(08:07:19) 기준 장면 4장 |

## 참고: 가제보 실습 의견 (정확하지 않을 수 있음)

아래는 위 자료를 보고 짐작한 것이다. 확인하지 않은 해석이라 틀릴 수 있다.

- TB-5 실행기는 지도를 안 받는 쪽(`not loaded`) 처리 중 Pose 칸에 Pose 가 아닌 값을 넣어 C 변환에서 중단된 것으로 보인다. 다른 회차에는 같은 줄 뒤에 중단이 없어, 그 순간의 입력에 따라 가끔 나는 것 같다.
- 15회차와 같은 조건에서 2·3단계가 다시 통과해, 두 단계는 재현되는 것으로 보인다.

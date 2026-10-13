# 로봇 없는 리허설 공식 3회 기록 (2026-10-13)

- 목적: `tests/ros/e3_rehearsal_abc.py`(#220) 의 공식 3회. #222(target_locator 종료 경합) 머지 뒤 16/0 ×3 을 남기기로 했다(작전 OPS-131·151).
- 층위: E3, 로봇 없음. `input = sim`(가짜 실행기·카메라·지도 수명 주기)이라 판정 회차가 아니다(원 test-plan 3절 회차 기록 규칙). 그래서 `runs.csv` 가 아니라 이 증거 파일로 남긴다.
- 실행: 이 PC, ROS 2 jazzy, `ROS_DOMAIN_ID=47`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, Discovery Server 없음, 매 회 시작 전 `ros2 node list --no-daemon` 비어 있음. 시계는 시험값(SYNCED, 5 ms).
- 실제로 돈 노드: `mission_coordinator`, `target_locator`, `db_writer`(+운영자 명령 HTTP API), `report_node`. 가짜: 웹캠 사건·감시 슬롯(실제 `WatchSlot` 규칙), 두 로봇 실행기·RobotStatus·TB-5 RobotPose, TB-6 카메라 관측·TF·MapState, 재확인 관측, 지도 고정 서비스·고정 grid.
- 경로는 `<RUN_DIR>`·`<REPO>`·`<HOME>` 로 바꿨고 줄 끝 공백을 지웠다. 판정은 각 실행의 마지막 줄과 종료 코드다(16 = 단계 14 + 두 실행의 자식 종료 2).

| 회 | 코드 | 결과 | 종료 코드 | 로그 |
| --- | --- | --- | --- | --- |
| 수정 전 1 | main `2098ca2` | PASS 16 / FAIL 0 | 0 | [before-run1](rehearsal-2026-10-13-before-run1.log) |
| 수정 전 2 | main `2098ca2` | PASS 16 / FAIL 0 | 0 | [before-run2](rehearsal-2026-10-13-before-run2.log) |
| 수정 전 3 | main `2098ca2` | PASS 15 / FAIL 1 — 14단계는 모두 PASS, SLAM_FREEZE 실행 끝에 **db_writer exit 1**(SIGTERM 때 `RCLError: ... context is not valid`) | 1 | [before-run3](rehearsal-2026-10-13-before-run3.log), [db_writer 끝](rehearsal-2026-10-13-before-run3-db_writer-tail.log) |
| 수정 뒤 1 | `ee345d5`(main `2098ca2` + 이 PR) | PASS 16 / FAIL 0 | 0 | [after-run1](rehearsal-2026-10-13-after-run1.log) |
| 수정 뒤 2 | `ee345d5` | PASS 16 / FAIL 0 | 0 | [after-run2](rehearsal-2026-10-13-after-run2.log) |
| 수정 뒤 3 | `ee345d5` | PASS 16 / FAIL 0 | 0 | [after-run3](rehearsal-2026-10-13-after-run3.log) |

- 수정 전 3회차 원인: 조정기(#154)·target_locator(#222)와 같은 SIGTERM 종료 경합이 db_writer 에도 있었다. 이 PR 이 db_writer·report_node 에 같은 규칙(`rescue_operations.shutdown.spin_until_shutdown`: context 가 닫힌 뒤의 RCLError 만 정상 종료)을 넣었다.
- 단계별 시간(수정 뒤 1회): 로그의 각 줄 괄호 안 초. 모두 3 s 안쪽.
- 알려진 문제(고치지 않음, 동결 뒤): report_node 가 첫 SIGTERM 을 놓친 일이 #220 사전 6회 중 1회 있었다(#220 본문). 이번 6회에는 없었다.
- NOT_RUN: 실기(E4), 실제 로봇 실행기, 실제 지도 고정 노드·AMCL, 관제 웹 화면, #234(순찰·QR 출구, 미머지)의 흐름.

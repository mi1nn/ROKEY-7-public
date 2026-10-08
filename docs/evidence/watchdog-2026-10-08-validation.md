# 명령 watchdog PR 검사 기록 (2026-10-08)

- 대상 코드: `ff6dbff` (main `ce09d98` 위). 증거 파일은 그 위 커밋에 더했다.
- 실행: 이 PC, ROS 2 jazzy. 로봇·실제 `cmd_vel` 없음. ROS 시험은 `ROS_DOMAIN_ID=42`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
- 경로는 `<REPO>`·`<USER_CACHE>`·`<TMP>`·`<HOME>` 로 바꿨고 줄 끝 공백은 지웠다. 판정은 종료 코드다.

| 검사 | 대상 | 종료 코드 | 로그 |
| --- | --- | --- | --- |
| 수정 전 실패 (E2) | `9858317` run_tests | 1 (`No module named 'rescue_robot.watchdog'`) | [run-tests-before](watchdog-2026-10-08-run-tests-before.log) |
| 수정 전 실패 (E3 기동) | main `ce09d98` 의 `ros_node.main` | 1 (`ValueError: Cannot declare a statically typed parameter with default value of type PARAMETER_NOT_SET`) | [adapter-start-before](watchdog-2026-10-08-adapter-start-before.log) |
| E2 전체 | `ff6dbff` `python3 -I scripts/run_tests.py` | 0 (324 OK) | [run-tests](watchdog-2026-10-08-run-tests.log) |
| 문서 | `python3 -I scripts/check_docs.py --sources <USER_CACHE>/p4/ROKEY_P4 <USER_CACHE>/p4/amr-mini-project` | 0 | [check-docs](watchdog-2026-10-08-check-docs.log) |
| wheel | `python3 -I scripts/check_wheels.py` | 0 | [check-wheels](watchdog-2026-10-08-check-wheels.log) |
| 공백 | `git diff --check ce09d98..ff6dbff` | 0 | — |
| E3 watchdog | `tests/ros/e3_watchdog_probe.py` (3회 반복도 모두 0) | 0 | [e3-watchdog-probe](watchdog-2026-10-08-e3-watchdog-probe.log) |
| E3 준비 신호·odom | `tests/ros/e3_readiness_probe.py`, `e3_odom_probe.py` | 0, 0 | [readiness](watchdog-2026-10-08-e3-readiness-probe.log), [odom](watchdog-2026-10-08-e3-odom-probe.log) |
| E3 watchdog 기동: 파라미터 없음 | `watchdog_node.main` | 2 (`missing parameters`) | [start-missing](watchdog-2026-10-08-start-missing.log) |
| E3 watchdog 기동: 정상 값 | 4초 뒤 SIGINT | 124 (timeout 이 끊음, `git_sha` 기동 로그) | [start-ok](watchdog-2026-10-08-start-ok.log) |
| E3 watchdog 기동: 입력=출력 토픽 / namespace 불일치 / 한계 0 | | 2, 2, 2 | [same-topic](watchdog-2026-10-08-start-same-topic.log), [ns](watchdog-2026-10-08-start-ns-mismatch.log), [policy](watchdog-2026-10-08-start-bad-policy.log) |
| E3 어댑터 기동 (수정 뒤): 파라미터 일부 | `ros_node.main` | 2 (`missing parameters`) | [adapter-start-after](watchdog-2026-10-08-adapter-start-after.log) |
| E3 어댑터 기동 (수정 뒤): 전체 파라미터 | `ros_node.main` | 1 (`No module named 'nav2_msgs'`: 파라미터 단계 통과, 아래 NOT_RUN) | [adapter-start-full](watchdog-2026-10-08-adapter-start-full.log) |
| E3 NavigateToPose·dock·어댑터 실기동 | — | NOT_RUN: `nav2_msgs`·`irobot_create_msgs` 미설치(sudo 필요) | — |
| E4 실기 정지 시간 | — | NOT_RUN: 로봇 미사용 | — |

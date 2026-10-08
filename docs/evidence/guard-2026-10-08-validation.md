# guard PR 검사 기록 (2026-10-08)

- 대상 코드: `d72d873` (main `d9e6cf8` 위). 증거 파일은 그 위 커밋에 더했다.
- 실행: 이 PC, ROS 2 jazzy. 경로는 `<REPO>`·`<USER_CACHE>`·`<TMP>`·`<HOME>` 로 바꿨고 줄 끝 공백은 지웠다.
- 판정은 종료 코드다.

| 검사 | 명령 | 종료 코드 | 로그 |
| --- | --- | --- | --- |
| 수정 전 실패 | `python3 -I scripts/run_tests.py` (`d0498b4`) | 1 (errors=11, `KeyError: 'transform_revision'`) | [run-tests-before-fix](guard-2026-10-08-run-tests-before-fix.log) |
| E2 전체 | `python3 -I scripts/run_tests.py` | 0 (291 OK) | [run-tests](guard-2026-10-08-run-tests.log) |
| 문서 | `python3 -I scripts/check_docs.py --sources <USER_CACHE>/p4/ROKEY_P4 <USER_CACHE>/p4/amr-mini-project` | 0 | [check-docs](guard-2026-10-08-check-docs.log) |
| wheel | `python3 -I scripts/check_wheels.py` | 0 | [check-wheels](guard-2026-10-08-check-wheels.log) |
| 공백 | `git diff --check d9e6cf8..d72d873` | 0 | — |
| E3 odom | `ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 tests/ros/e3_odom_probe.py` | 0 | [e3-odom-probe](guard-2026-10-08-e3-odom-probe.log) |
| E3 준비 신호 | `ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 tests/ros/e3_readiness_probe.py` | 0 | [e3-readiness-probe](guard-2026-10-08-e3-readiness-probe.log) |
| E3 NavigateToPose·dock | — | NOT_RUN: `nav2_msgs`·`irobot_create_msgs` 미설치(sudo 필요) | — |

## 커밋 더함: 카메라 compressed 구독 (작전 OPS-07 권고)

| 검사 | 대상 | 종료 코드 | 로그 |
| --- | --- | --- | --- |
| 수정 전 실패 | `882c034` run_tests | 1 (errors=1, `ImportError: cannot import name 'camera_message_type'`) | [camera-run-tests-before-fix](guard-2026-10-08-camera-run-tests-before-fix.log) |
| E2 전체 | `8a03824` run_tests | 0 | [camera-run-tests](guard-2026-10-08-camera-run-tests.log) |
| E3 준비 신호(robot_a 에 `CompressedImage` 카메라) | `8a03824` | 0 | [camera-e3-readiness-probe](guard-2026-10-08-camera-e3-readiness-probe.log) |
| 문서·wheel·공백·E3 odom | `8a03824` | 0, 0, 0, 0 | — |

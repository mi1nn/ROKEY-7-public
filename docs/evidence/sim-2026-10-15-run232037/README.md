# 가제보 실습 둘째 회차 자료 (full-1015-232037, 2026-10-15)

[가제보 실습 기록](../sim-2026-10-15-gazebo-practice.md) 6절 둘째 회차의 원문이다. 통합-조정(C9)과 통합-로봇(C7·C8)이 원인을 보도록 올린다. 시뮬레이션 PC 의 회차 폴더에서 그대로 옮기고 색 코드를 지웠다. 저장소 경로는 `<저장소>`, 회차 폴더는 `<회차 폴더>`, ROS 로그 폴더는 `<ROS 로그>` 로 바꿨다. 빌드는 `src/` = main `97f2cef` 다.

## 파일

| 파일 | 무엇 | 줄인 것 |
| --- | --- | --- |
| `maps/live-r0-27be82b2a9e5.pgm`·`.yaml` | TB-6 SLAM 지도, `EXPLORATION_COMPLETE`(23:25:28) 순간 저장본(운영 launch `map_snapshot`) | 없음 |
| `maps/frozen-r1-…`·`frozen-r2-…` | 23:25:32 1·2차 고정 지도(같은 내용) | 없음 |
| `logs/coordinator.log` | 조정기 로그 전부(회차 시작~끝) | 없음 |
| `logs/robot_robot_b.log` | TB-5 `robot.launch`(Nav2·실행기) 로그 전부 | 없음 |
| `logs/frozen_loc_b.log` | TB-5 `frozen_localization`(고정 지도 relay·AMCL·출발 정렬) | 1초마다 나오는 `Requesting no-motion update`·`Waiting for map` 줄 |
| `logs/frozen_loc_a.log` | 7.5 뒤 TB-6 `frozen_localization` | `Requesting no-motion update`·`Message Filter dropping` 줄 |
| `logs/tb6_slam.log` | TB-6 SLAM | `Message Filter dropping` 줄 |
| `logs/operator.log` | 시뮬레이션 운영자 대역(런북 7.1·7.5 대신) | 없음 |

## 읽을 때

- **C9(탐색이 경계 2개 뒤 끝남):** `coordinator.log` 의 `FRONTIER none on map update 193~195` → `EXPLORATION_COMPLETE … frontiers_sent=2`(23:25:26~28)와 `maps/live-r0-27be82b2a9e5`. 이 빌드에는 #373(후보별로 빠진 이유) 로그가 없다. 작전 판단(2026-10-15): 첫 경계가 벽 쪽이던 것은 시뮬레이션 벽 관통(S10)으로 SLAM 벽 줄에 틈이 생긴 탓일 수 있다 — 「시뮬레이션 world(S10) 후보」.
- **C7(TB-6 relay 막힘):** 시뮬레이션 운영자 대역의 잘못으로 본다. `operator.log` 23:25:32 에 tb6_slam launch PID 에 SIGINT 를 보냈지만, 백그라운드로 띄운 launch 는 SIGINT 를 무시로 물려받아 SLAM 이 꺼지지 않았다. `tb6_slam.log` 의 slam_toolbox 종료 줄은 회차를 끈 23:37:23 하나뿐이다. 「발행자 0」(23:25:53)은 `--no-daemon --spin-time 3` 이라 발행자를 못 본 것으로 본다. 그래서 `frozen_loc_a.log` 23:25:59 `MAP_RELAY_BLOCKED` 는 맞는 판정이었고, 그 뒤 `MAP_RELAYED` 는 없다. 운영자 대역은 SIGTERM + slam_toolbox PID 종료 확인으로 고쳤다.
- **C8(TB-5 Nav2 가 active 가 안 됨):** `robot_robot_b.log` 의 `lifecycle_manager_navigation` 줄은 23:21:56 `Starting managed nodes bringup...` → `Configuring controller_server` → `Configuring smoother_server` 다음이 회차 끝(23:37:23 `rcl preshutdown`)이다. `Aborting bringup`·`Timed out waiting for transform`·`Failed to activate` 줄은 없다. 원인 분석은 #375.
- **C2(TB-5 정렬 판정 버려짐):** `robot_robot_b.log` 의 `live input rejected: Logger severity cannot be changed between calls.` #370 이 고쳤다(이 회차 빌드에는 없음).

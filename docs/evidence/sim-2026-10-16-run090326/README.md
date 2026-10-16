# 가제보 실습 열아홉째 회차 (full-1016-090326, 2026-10-16 09:03 ~ 09:15, sim1 새 arena)

측정·로그 자료를 먼저 적는다. 가제보 실습의 해석은 맨 끝 「참고: 가제보 실습 의견」에만 둔다(재범 지시 2026-10-16).

- **빌드·world**: main `adad6d8`(#463 복귀 짧은 끊김 유예)에 PR #462 head `8270540` 의 `config/sim/`·`scripts/sim/map_to_world.py` 만 얹어 돌렸다. 새 arena 를 쓰라는 재범 지시(08:4x)를 따랐다. 회차 뒤 작업 폴더는 main 으로 되돌렸다.
- **기동 실패 2회**:
  - 08:55:39 폴더(`failed-start-085539/`): TB-6 Nav2 bringup 이 멈췄다(`planner_server … failed to send response to /robot6/planner_server/change_state (timeout)`, `robot5/amcl … get_state (timeout)`). arm 은 `NAV_NOT_READY missing=['nav2']` 였다.
  - 08:59:12 폴더: run_full 이 `robot6 dock_status 가 도킹 상태로 오지 않습니다` 로 스스로 멈췄다.
  - 세 번째 기동(09:03:26)은 TB-6 `ARMED`(09:05:26)까지 갔다.
- 재난 선포: 재범 위임으로 09:05:58(`accepted: true`).

## 단계별 결과

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1 | 통과 | |
| 2 | 통과 | 경계 3개 `COMPLETED`, 터널 1 통과, 정답 위치 TB-6 (2.881, 4.848) |
| 3 | **인형 없음** | a116 경계가 `cut=True`(09:06:58)로 잡혀 그 자리 회전 훑기를 하지 않았다. 서쪽 끝 f0bf 에서 4/4 회전(`FRONTIER_LOOK_DONE … turned 4/4: round`, 09:09:03). `FAKE_TARGET published` 0. 대역 사유 `NOT_IN_FRONT`·`OUT_OF_RANGE` 각 3, `OUT_OF_VIEW` 2, `NO_MAP`·`NO_START_POSE` 각 1 |
| 4 | 실행 안 됨 | 인형 없음. 2차 고정(09:09:05)에서 TB-5 실행기 중단은 이번에는 없었다(`Assertion` 0) |
| (TB-6 복귀) | **실패(C22 재현)** | 넘김 뒤 AMCL 위치가 나오지 않아 RETURN_HOME 이 나가지 못했다 |

## C22 재현: 넘김 뒤 TB-6 AMCL 이 라이다를 처리하지 않음

- 09:10:05 운영자 대역이 frozen_localization robot_a 를 띄웠다. 09:10:11 `MAP_HANDOVER`(실행기 stamp 1792109411.44).
- 그 뒤 TB-6 `AMCL_NOMOTION_REQUESTED robot_a: no amcl_pose since the map switch …` 가 57회 나왔다(마지막 `235.9 s ago`).
- `logs/frozen_loc_a.log` 에 `createLaserObject` 는 0줄이다. TB-6 RobotPose 마지막은 09:09:36.79 다(`track/pose_gap.txt`).
- 작전이 요청한 자료(`logs/diag_c22_robot6.txt`, 09:14:20 ~):
  - `tf2_echo base_link rplidar_link`(`/robot6/tf`·`/robot6/tf_static` remap): 처음 몇 초 `Invalid frame ID "base_link"` 뒤 `Translation [-0.040, 0.000, 0.193]`, RPY 0, 0, 90°.
  - `tf2_echo odom base_link`: (−4.479, 2.001), yaw 166.9°.
  - `/robot6/tf_static` 발행자 1(`wall_clock_relay_field`).
- 조정기: `SEND RETURN_HOME` 가 09:09:05 ~ 09:09:35 사이에 4회 나갔고 모두 `ADMISSION -> False` 였다(넘김 중). 그 뒤 TB-6 위치가 없어 보내지 않았다.

## 파일

| 파일 | 무엇 |
| --- | --- |
| `logs/*.log` | 회차 폴더 원문(색 코드 지움, 경로·PC 이름 바꿈) |
| `logs/diag_c22_robot6.txt` | C22 순간 TF·tf_static·AMCL 상태 |
| `logs/diag_tf_handover_robot6.txt` | 09:09 ~ 09:11 map→odom·amcl_pose·odom→base_link |
| `failed-start-085539/` | 첫 기동 실패 자료 |
| `maps/`, `track/`, `video/` | 고정 지도, 정답 위치·RobotPose·RTF·부하, 녹화 줄인 판과 200 s 장면 |

## 참고: 가제보 실습 의견 (정확하지 않을 수 있음)

아래는 위 자료를 보고 짐작한 것이다. 확인하지 않은 해석이라 틀릴 수 있다.

- C22 는 13회차(옛 arena)와 19회차(새 arena) 두 번 났다. TF 는 다 있는데 AMCL 이 라이다를 받지 않으니, AMCL 이 뜨는 순간의 통신·부하 쪽일 수 있어 보인다.
- 기동 실패의 lifecycle 응답 시간 초과는 14회차 첫 기동과 같은 모양이다.
- 인형 미관측은 a116 경계가 `cut=True` 로 잘려 회전 훑기를 건너뛴 탓으로 보인다.

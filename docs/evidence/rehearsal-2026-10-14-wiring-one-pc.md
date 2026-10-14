# 한 PC 배선 리허설 기록 (2026-10-14)

## 한눈에

질문은 두 가지였다. 실제 namespace(`/robot6`, `/robot5`)로 이 저장소의 현장 노드를 모두 띄우면 배선 점검 도구(`scripts/field/wiring_check.py`)가 보는 연결이 모두 이어지는가? 그리고 지도 고정과 임무 시작 흐름이 실제 메시지로 도는가?

| 회차 | 코드 | 본 것 | 결과 |
| --- | --- | --- | --- |
| 1 | main 2f9d9df | 배선, SEARCHING | 61줄 중 FAILED 44. 두 로봇의 실행기가 시작하자마자 꺼짐 → #298 |
| 2·3 | 2f9d9df + #298 | 배선, SEARCHING·FROZEN | 61줄 모두 PASSED, 같은 이름 노드 없음 |
| 6·8 | 2f9d9df + #298 | 지도 고정 경로, TB-6 TF 평면 | 1차 고정(6단계), 2차 고정과 TB-6 SLAM 끄기(10단계) 모두 PASSED |
| 9 | 2f9d9df + #298 | 지도 고정 경로, TB-6 TF 현장 측정값 | 현장 실패 재현. TB-6 MapState UNKNOWN, 출발 자세 없음, 고정 두 번 다 실패 |
| 10 | #291 head 4619504 + #298 | 같음 | 9회차와 같은 실패(#291 은 아직 시험 커밋뿐) |
| 12 | 2f9d9df + #298 | 임무 시작, TB-6 TF 평면 | 5단계 모두 PASSED. 거절된 목표 뒤 TB-6 은 정지 latch 유지 |
| 확인 3회 | main ee1ebd8(#298 머지 뒤) | 지도 고정 평면·측정 TF(FROZEN), 임무 시작 | 위와 같음. 평면 10단계 PASSED, 측정 TF 10단계 FAILED(현장 재현), 임무 5단계 PASSED |

로봇은 없었다. 가짜 Nav2 는 목표를 모두 거절하므로 아무것도 움직이지 않는다. 4·5·7·11회차는 probe 자체의 결함(서비스 목록, 지운 발행자, refresh_state 빠짐)을 고친 회차라 결과에서 뺐다.

## 환경

- PC 한 대(작전 지시, OPS-245 다음 단계). ROS 2 Jazzy, Fast DDS.
- Discovery Server 는 `fast-discovery-server -i 0 -l 127.0.0.1 -p 11888` 하나였다. 시작 전에 `ROS_DISCOVERY_SERVER` 를 지우고 `127.0.0.1:11888` 만 넣었다. 현장 서버 주소가 남아 있으면 실제 로봇 그래프에 붙는다. Discovery Server 는 도메인으로 갈리지 않는다(network-discovery.md 3절).
- `ROS_DOMAIN_ID=193`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, 노드는 `ROS_SUPER_CLIENT=False` 로 두었다. 점검 도구만 super client 다.
- 이 PC 에는 Nav2·slam_toolbox·turtlebot4_navigation·depth_image_proc 가 없다. `nav2_msgs`·`irobot_create_msgs`·`geographic_msgs` 도 없다. 메시지 세 개는 GitHub 소스(jazzy)를 리허설 작업공간에 빌드했다. 시스템 설치와 sudo 는 쓰지 않았다.
- 입력은 두 가지다.
  - 프로파일: `tests/test_bringup_field_launch.py` 의 `field_profile`(예시 프로파일 + robot_a=`/robot6`, robot_b=`/robot5`)
  - 현장 파일: 저장소 `config/field/search_area.field.json`. 여기에 리허설에서만 TB-5 출발값 `[0, 0, 0]` 을 더했다(`frozen_localization_values` 가 요구함, 잰 값 아님).

## 무엇이 진짜이고 무엇이 가짜인가

| 진짜(이 저장소 노드, 현장 launch 와 같은 값) | 가짜(`fake_graph.py` 끝점, `feed.py` 합성 데이터) |
| --- | --- |
| `operations.launch.py`(db_writer, report_node), `coordinator.launch.py`, `map_freeze.launch.py`, `overhead_passage.launch.py` 두 로봇 | 로봇 Pi(odom, scan, dock_status, 카메라, 깊이), Create3 의 `cmd_vel` 구독 |
| `robot.launch.py` 의 watchdog·`task_input`(Nav2 include 만 뺌) 두 로봇 | Nav2(`collision_monitor`, `docking_server`, `bt_navigator`), `slam_toolbox`, AMCL |
| `frozen_localization.launch.py` 의 `frozen_map_relay`(AMCL 만 뺌) | 깊이 컨테이너(`passage_points`), `webcam_node`, `target_locator`, `exit_locator` |

- 진짜 노드끼리의 연결은 이름·개수·타입·QoS 를 모두 봤다. 실행기 ↔ 조정기, 실행기 ↔ watchdog, 지도 고정 ↔ 조정기·relay·실행기, 터널 판정 ↔ db_writer, 보고 등이 여기에 들어간다.
- 가짜와의 연결은 이름과 개수만 의미가 있다. 가짜는 진짜 노드가 쓰는 타입을 따라 하고, 가짜 구독은 가장 약한 QoS 를 요청한다. 그래서 진짜 Pi·Nav2·SLAM 의 QoS 는 이 리허설로 확인되지 않는다.
- 리허설 전용 장치 셋
  - 실행기는 같은 호스트의 조정기를 거절한다(`LIVE_ROS expects two distinct PCs`, `rescue_robot/live_clock.py`). 4회차부터 두 `task_input` 프로세스에만 `host_shim`(sitecustomize)을 넣어 시계 측정의 host id 를 따로 줬다. chrony 값과 오차는 그대로다.
  - 지도 고정 서비스는 조정기 대신 `freeze_probe.py` 가 직접 불렀다. 그래서 조정기는 이 고정을 자기 기준으로 쓰지 않았다(`FROZEN_MAP ... frozen=False basis=None`).
  - 두 로봇 위치는 모두 (0,0)이다. 현장(출발 간격 약 1.95 m)과 다르다.

## 1회차에서 나온 것 (#298 로 고침)

두 로봇의 `task_input` 이 시작하자마자 꺼졌다.

```
command adapter failed: INVALID_INPUT [rescue_robot.INVALID_INPUT]: LIVE_ROS missing deployment parameters: task.saved_map.occupancy_sha256
```

- 고정 지도 모드에서 launch 는 지도 해시를 `''` 로 넘긴다. 실행기 파라미터 읽기는 빈 문자열을 「없음」으로 봤다.
- 실기 날 그대로 두면 두 로봇의 실행기가 뜨지 않는다. 담당은 통합-로봇이다. #298 로 고쳤다(머지, ee1ebd8).
- 배선표에서는 실행기와 이어진 줄 44개가 FAILED 였다(발행자 없음, 구독 없음).

## 지도 고정 경로 (6·8회차, TB-6 TF 평면)

| 단계 | 결과 |
| --- | --- |
| 1 TB-6 실행기 MapState R0 | VALID |
| 2 TB-6 출발 자세 | `START_POSE captured robot_a map=(0.000, 0.000, 0.000)` |
| 3 1차 고정 | success, rev=1, `final=False tb6 keeps SLAM` |
| 4 `/rescue/map/frozen` + `frozen_revision` | 조정기·두 실행기에 도착 |
| 5 TB-5 relay | `MAP_RELAYED /rescue/map/frozen R1 -> /robot5/map` |
| 6 TB-5 MapState | `MAP_REVISION_DEFERRED ... not stopped` → 정지 확인 뒤 `MAP_REVISION_ADOPTED` → R1 VALID |
| 7 2차 고정(FROZEN) | success, rev=2, `final=True tb6 amcl start=(0.000, 0.000, 0.000)` |
| 8 TB-6 SLAM 끄기 → TB-6 relay | `MAP_RELAYED /rescue/map/frozen R2 -> /robot6/map` |
| 9 TB-6 MapState | R2 VALID(`/robot6/amcl_pose`) |
| 10 TB-5 MapState | R2 VALID |

## 현장 TF 로 다시 본 지도 고정 경로 (9·10회차)

가짜 SLAM 의 TF 를 2026-10-14 TB-6 현장 실패 때 잰 값으로 바꿨다(#291 `measured_tb6_tf`). map→odom 은 roll 약 −0.55°, pitch 0.30°, z 1.5 mm 이고, odom→base_link 는 roll 약 −0.52°, pitch 0.27°, z −2.3 mm 에 0.1 s 더 오래됐다.

- main + #298 에서 `map_freeze` 와 TB-6 실행기가 모두 `SLAM_TF_POSE a non-planar map/odom/base edge was not used` 를 냈다.
- TB-6 MapState 는 R0 UNKNOWN 이었고 출발 자세가 없었다. 고정은 두 번 다 `MAP_FREEZE_FAILED no TB-6 start pose recorded: coordinates measured from the start cannot be placed` 로 실패했다(10단계 모두 FAILED). 현장 실패와 같다.
- TB-6 실행기 MapState 도 UNKNOWN 이므로, 현장에서는 TB-6 이 탐색 목표도 받지 못할 것이다[추정, 같은 `SlamTfPose` 경로].
- #291 head 4619504(+#298)도 같았다. 그때 #291 에는 시험 커밋 2개뿐이었다. 고친 커밋이 올라오면 `REHEARSAL_PROBE=freeze ... FROZEN`(기본 TF 가 측정값)으로 같은 확인을 한다.

## 임무 시작 (12회차, TB-6 TF 평면)

| 단계 | 결과 |
| --- | --- |
| 1 두 실행기 RobotStatus | 받음(IDLE, NOT_READY) |
| 2 `refresh_state` → 두 로봇 `task_arm` | 둘 다 `ARMED`, readiness READY |
| 3 가짜 웹캠 DISASTER_SIGN | DISASTER, `SEARCH_AUTO_STARTED on DISASTER` |
| 4 TB-6 FRONTIER 목표 | `SEND SEARCH_AREA ... robot=robot_a` → admission ACCEPTED |
| 5 가짜 `bt_navigator` 가 NavigateToPose 거절 | 조정기 `RESULT outcome=FAILED`. 실행기 FINISHED, NOT_READY, STOPPED, `stop_reason` GOAL_REJECTED |

- `refresh_state` 없이 `task_arm` 을 부르면 `NOT_AUTHORIZED: waiting for fresh configured coordinator SystemState` 로 거절됐다(11회차). README 3단계 순서(재발행 → arm)가 맞다.
- 5단계에서 실행기는 다시 준비하지 않았다. 정상 완료가 아니므로 OPS-231 자동 재준비 대상이 아니다. `REARM_REFUSED_*` 가 아니라 latch 가 그대로 남았다.
- 조정기는 그 뒤 다음 FRONTIER 목표를 만들었다. 실행기는 latch 로 받지 않는다.
- 목표가 나가기 전에 `task not issued: TRAFFIC_ROBOTS_TOO_CLOSE` 가 4번 나왔다. 리허설에서 두 로봇을 모두 (0,0)에 두었기 때문이다(`traffic_min_separation_m` 0.6 m).
  - 이 배치는 현장과 다르다. 현장 출발 간격은 1.95 m(가로)·0.05 m(세로)라 이 보류가 나지 않는다[측정 2026-10-14, 재범 줄자, 둘 다 도킹 상태].
  - TB-5 출발값(`initial_pose_xy_yaw`)이 현장 파일에 들어가기 전에는 조정기가 간격 검사를 끄고(`TRAFFIC_SEPARATION_UNCHECKED`) TB-6 을 보낸다(작전 확인).

## 도구 문제와 고친 것

- 처음 `run.sh` 는 1회차에서 launch 의 자식 노드를 고아로 남겼다. 비대화 bash 의 백그라운드 작업은 SIGINT 를 무시하기 때문이다. 남은 노드는 PID 를 지정해 껐다. 그 뒤 `set -m` 과 프로세스 그룹 단위 INT → TERM → KILL 로 고쳤고, 2회차부터 남은 프로세스가 없었다.
- 끌 때 나오는 `KeyboardInterrupt` traceback 은 점검이 끝난 뒤 Ctrl+C 로 끈 흔적이다.

## 아직 보지 않은 것 (NOT_RUN)

- 실제 PC 4대와 로봇 Pi Discovery Server 아래의 그래프
- Nav2·SLAM·AMCL·깊이 컨테이너·Pi 쪽의 실제 노드 이름과 QoS
- 조정기가 스스로 고정을 요청하는 흐름(인형 발견, 탐색 완료)
- 목표를 받아 실제로 움직이고 정상 완료되는 흐름(OPS-231 자동 재준비의 COMPLETED 쪽)

재현 방법은 [rescue_bringup README](../../src/rescue_bringup/README.md) 「한 PC 배선 리허설」에 있다.

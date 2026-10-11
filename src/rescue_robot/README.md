# rescue_robot

명령 수락·단일 활성 임무·임대·취소·정지 확인의 순수 Python 코어(`executor.py`)와, 그 코어를 Nav2 `NavigateToPose` 에 잇는 명령 어댑터(`nav2_adapter.py`, ROS 경계 `ros_node.py`)다. SLAM 실행·언도킹은 하지 않는다.

공개 API는 `Policy`, `Readiness`, `Task`, `GoalPose`, `Executor`, `CommandAdapter`, `MissionGoalBinder`, `ReadinessPolicy`, `ReadinessMonitor`다. 인수와 시간 진행 예시는 [`tests/test_executor.py`](../../tests/test_executor.py)와 [`tests/test_nav2_adapter.py`](../../tests/test_nav2_adapter.py)를 참고한다. 모든 운용 임계값은 호출자가 명시해야 한다. 시험값은 실기 기본값이 아니다.

자세한 구현 경계·출처·남은 검증은 [구현 기록](../../docs/implementation/implementation-robot.md)에 있다. 이 코어를 단독으로 로봇에 연결하지 않는다. 인증된 단일 어댑터, 시각/센서 증거, 장비측 watchdog이 필요한 상태다.

## Nav2 명령 어댑터

`CommandAdapter`(ROS import 없음)는 로봇 하나의 `Executor` 와 그 로봇의 `NavigationPort`(`send_goal(command_id, goal_pose)` / `cancel_goal(command_id)`)를 묶고, 아래 사건을 서로 다른 입력으로 넘긴다.

| 사건 | 어댑터 입력 | 코어 호출 | 뜻하지 않는 것 |
| --- | --- | --- | --- |
| 임무 수락 | `submit(task)` | `admit` → `TaskAdmission` | — (협업·관제가 쓰는 유일한 수락 근거) |
| goal 전송 | `step()` 주기 호출 | `dispatch`/`tick` → `('SEND', id)` → `send_goal(id, task.goal_pose)` | 로봇이 움직였다는 증거 |
| Nav2 goal 수락 | `on_goal_response(id, accepted)` | `goal_response` | `TaskAdmission`(allocator 예약의 수락) |
| 취소 요청 | `request_stop(cause)`·만료·guard 실패 | `('CANCEL', id)` → `cancel_goal` | 정지 |
| 취소 응답 | `on_cancel_response(id, accepted)` | 없음(기록만) | 정지 |
| action 종료 | `on_terminal(id, GoalStatus 코드)` | `goal_terminal`(4/5/6 만) | 정지 |
| 정지 관측 | `on_odom(stamp, linear, angular)` | `observe_motion` | — (종료 fence 뒤 연속 표본만 `stop_confirmed`) |

- 이 어댑터가 보내지 않은 `command_id` 의 콜백(`FOREIGN_*`)은 실행기에 넘기지 않는다. 로봇마다 어댑터·포트·네임스페이스가 따로라서 한 로봇의 콜백이 다른 로봇의 취소를 만들지 않는다.
- `Readiness` 는 여섯 상태(`driver`, `sensors`, `slam`, `nav2`, `undocked`, `watchdog`)가 모두 들어와야 하고 가장 오래된 수신 시각을 쓴다. 하나라도 없으면 준비 안 됨이다.
- `guards(task) -> bool` 은 호출자가 주입하고 SEND 직전과 매 `step` 에 다시 본다(지도 revision 등; mapping 연결은 후속).
- **goal frame**: `bind_goal` 을 주지 않으면 `task.goal_pose` 를 그대로 보낸다. 지도 guard(아래)를 쓰면 mission 좌표를 로봇 map 좌표로 바꿔 보낸다. 어느 경우든 `Task` 자체는 바꾸지 않는다.
- **odom 시각 [미확인]**: `sample_steady = received_steady − (wall_now − header_ns)/1e9`. 로봇과 PC 시계가 `clock_error_bound_s`(로컬 float 초; C-03 의 int `clock_error_bound_ns` 와 다름) 안에서 맞는다고 전제한다(측정 전). |차이| 가 상한을 넘거나 stamp 가 무효이면 표본으로 쓰지 않고 `Executor.invalidate_stop_evidence` 로 정지 창을 초기화한다 → 증거 부족은 `STOP_EVIDENCE_STALE`·`STOP_UNCONFIRMED` 로 드러난다.
- 두 로봇은 같은 어댑터를 쓰고 task kind 만 다르다(SCOUT: `SEARCH_AREA`·`REOBSERVE_TARGET`, SUPPORT: `HOLD`·`APPROACH_TARGET`). 역할은 설정에서 오고 코드에 두지 않는다. `HOLD` 는 goal 없이 정지 확인 뒤 `COMPLETED` 다.

## 지도 guard (`goal_binding.MissionGoalBinder`)

`goal_pose` 의 `map_session_id`·`map_revision`·`transform_revision` 은 이동하는 로봇 지도에서 그 goal 이 검증된 버전이다. binder 는 `MappingRegistry`(import 하지 않고 받음)의 `map_state`·`current_alignment`(읽기 전용)로 다음을 본다.

- `bind(task)`(dispatch 직전): 버전이 현재와 같으면 정렬의 역변환으로 mission → 로봇 map 좌표·방향을 만들어 SEND 에만 쓴다. 실패하면 아무것도 보내지 않고 guard 실패(`GUARD_INVALID`)다.
- `check(task)`(매 `step`): goal 의 frame 이 mission 이 아님, 버전 불일치(지도 보정·세션 교체·정렬 변경), 정렬 없음·만료·오차 초과 중 하나면 False → 취소 → 정지 관문. 주행 중 정렬이 바뀐 경우도 여기서 잡힌다(C-04).
- `REOBSERVE_TARGET`·`APPROACH_TARGET` 은 `target_gate(task)` 가 True 여야 한다. 호출자가 GoalSnapshot 검증(`coordinates_valid`)과 `auto_approach_eligible`(지금은 항상 False)을 묶어 넣는다. 그래서 backend 보정 검증 전에는 대상 지점 SEND 가 나가지 않는다. `SEARCH_AREA` 는 지도 guard 만으로 출발하고, `HOLD` 는 지도와 무관하다.

## 준비 신호 (`readiness.ReadinessMonitor`)

출처별 마지막 **로컬 수신 시각**과 `ReadinessPolicy` 의 최대 나이로 `driver`·`sensors`·`slam`·`watchdog` flag 를 만든다. 수신 여부만 보고 센서 header 나이(인식 몫)나 SLAM 보정은 판단하지 않는다. 출처가 없는 flag 는 만들지 않으므로 그 flag 는 계속 준비 안 됨이다.

| flag | 출처 | 비고 |
| --- | --- | --- |
| `driver` | odom (+ `wheel_status`, 있으면) | |
| `sensors` | scan (+ 카메라 토픽들) | |
| `slam` | 네임스페이스 `tf` 의 `map_frame → odom_frame` 수신 | freshness 만 |
| `nav2` | action server ready | 기존 |
| `undocked` | dock status | 기존 |
| `watchdog` | 명령 watchdog(`command_watchdog`)의 heartbeat | `watchdog_heartbeat_topic` 이 비면 출처 없음 → 움직이지 않음. watchdog 이 `ARMED` 일 때만 발행. Create 3 의 cmd_vel timeout 은 검증된 증거가 아님 |

최대 나이 값은 정해지지 않았다 [미확인]. 시험값만 있다.

## ROS 노드 (`command_adapter`)

`ros_node.main` 은 기본값 없는 파라미터만 받는다. 프로파일(`config/deployment.*.json`)의 값은 bringup 이 넘긴다(이 패키지는 `rescue_bringup` 을 import 하지 않는다).

| 파라미터 | 출처 |
| --- | --- |
| `allowed_robot_ids`(문자열 배열), `robot_id`, `namespace` | 프로파일의 두 로봇 ID 와 해당 로봇 namespace. 노드 namespace 와 다르면 기동 거부 |
| `nav_action`, `odom_topic`, `dock_status_topic` | namespace 기준 이름. 프로파일 v2(#17)가 들어오면 그 값을 쓴다 |
| `authority_epoch`, `policy_version`, `policy.*`(7개), `step_period`, `clock_error_bound_s`(초) | 운용 정책. 실기 값 미정(T-01) |
| `scan_topic`, `tf_topic`, `map_frame`, `odom_frame` | 준비 신호 출처(필수) |
| `camera_topics`(문자열 배열), `wheel_status_topic` | 준비 신호 출처. 비워 둘 수 있음. `/compressed` 로 끝나는 카메라 토픽은 `CompressedImage`, 그 밖은 `Image` 로 구독(TB4 OAK-D 기본 발행이 compressed, P4 DEC-004) |
| `watchdog_heartbeat_topic` | 명령 watchdog 의 heartbeat(`std_msgs/Header`). 비워 둘 수 있으나 그러면 `watchdog` flag 가 없어 이동 불가 |
| `executor_heartbeat_topic` | 이 노드가 step 마다 발행하는 heartbeat(`std_msgs/Header`). 명령 watchdog 이 듣는다(필수) |
| `readiness.version`, `readiness.max_age_s.{odom,scan,camera,wheel_status,map_to_odom_tf,watchdog_heartbeat}`(초) | 준비 신호 정책. 값 미정 [미확인] |

`command_adapter` 단독 실행은 임무·정렬 입력을 연결하지 않아 모든 dispatch를 거절한다. 아래 `task_input`은 동일 프로세스에 SEARCH 입력을 연결한다. 필요한 ROS 패키지: `rclpy`, `rcl_interfaces`, `nav_msgs`, `nav2_msgs`, `irobot_create_msgs`.

파라미터는 `declare_required`로 `Parameter.Type`과 타입이 명시된 읽기 전용 descriptor를 함께 선언한다. 빠진 값은 `missing parameters`로 기동 거부한다. 시작 실패 때에도 노드와 ROS context를 정리한다.

## 명령 watchdog (`command_watchdog`, C-01·TR-11)

`watchdog.CommandWatchdog`(ROS import 없음)와 `watchdog_node`(ROS 경계). 로봇 하나의 마지막 속도 경로에 하나씩 둔다: launch 가 Nav2 의 최종 속도 출력을 `cmd_vel_in_topic` 으로 remap 하고, 로봇의 `cmd_vel` 은 이 노드만 쓴다.

| 상태 | 들어온 명령 | 자기 heartbeat | 나가는 조건 |
| --- | --- | --- | --- |
| `WAITING` | 0 으로 바꿔 발행 | 없음 | 첫 executor heartbeat → `ARMED` |
| `ARMED` | 한계 안이면 그대로 전달 | 발행 | executor heartbeat 만료·한계 초과·비유한값·시간 역행 → `STOP_LATCHED` |
| `STOP_LATCHED` | 0 으로 바꿔 발행(전이 때 0 한 번) | 없음 | `~/reset`(`std_srvs/Trigger`) + fresh heartbeat 만. heartbeat 회복만으로는 풀리지 않음(C-06) |

- 한계 초과 명령은 잘라 내지(clamp) 않고 latch 한다. `ARMED` 에서 이동 명령 뒤 입력이 끊기면 0 을 한 번 보낸다(latch 아님).
- watchdog 이 latch 되면 자기 heartbeat 가 멈춰 어댑터의 `watchdog` flag 가 만료되고 새 goal 이 나가지 않는다. 실행기의 STOP latch·새 generation 규칙은 따로 그대로다.
- 이것은 ROS 프로세스 안의 소프트웨어 watchdog 이다. 로봇에서 돌릴지 PC 에서 돌릴지, Create 3 timeout 과의 관계, 실제 정지 시간은 배포·측정 몫이다(T-05). 물리 안전 인증이 아니다.

| 파라미터 | 출처 |
| --- | --- |
| `namespace` | 노드 namespace 와 다르면 기동 거부 |
| `cmd_vel_in_topic`, `cmd_vel_out_topic`, `cmd_vel_stamped`(bool) | 마지막 속도 경로. 둘이 같으면 기동 거부. `Twist`/`TwistStamped` 선택은 설치 Nav2·Create 3 설정 확인 뒤 [미확인] |
| `executor_heartbeat_topic`, `watchdog_heartbeat_topic` | 어댑터 노드의 같은 이름 파라미터와 맞춘다 |
| `step_period`, `watchdog.version`, `watchdog.{heartbeat_max_age_s,command_max_age_s,max_linear_mps,max_angular_rps}` | 정책. 값 미정 [미확인](T-05) |

## 검사

```bash
python3 -I scripts/run_tests.py                     # E2, ROS 없이 (tests/test_nav2_adapter.py 포함)
# E3 일부: ROS 2 환경을 source 한 뒤, 로봇 도메인과 겹치지 않는 그래프에서
ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  PYTHONPATH=$PWD/src/rescue_contracts:$PWD/src/rescue_robot:$PYTHONPATH \
  python3 tests/ros/e3_odom_probe.py
# 같은 환경에서
python3 tests/ros/e3_readiness_probe.py
python3 tests/ros/e3_watchdog_probe.py
```

E3 탐침은 실제 토픽이 네임스페이스별로 섞이지 않는지 본다: odom 탐침은 `/robot_a/odom`·`/robot_b/odom` 정지 증거, 준비 신호 탐침은 odom·LaserScan·`map → odom` TF, watchdog 탐침은 전달·heartbeat 만료 latch·회복 뒤에도 latch 유지·`~/reset`·robot_b 격리와 어댑터 `watchdog` flag. `NavigateToPose` action 은 다루지 않는다.

## 설치 가능한 임무 입력 진입점 (`task_input`, 2026-10-11)

`ros2 run rescue_robot task_input`과 기존 별칭 `robot_task_input`은 동일한 프로세스에서
내장 `SearchRuntime`을 연결한다. 한 로봇의 SEARCH_AREA 한 건만 활성화하며, 다른 종류의
임무는 거절한다. 별도 Nav2 발행자나 두 번째 command adapter를 띄우지 않는다.
`task.runtime_config`의 절대경로 JSON을 읽고 그 `input_file`의 절대경로로 live 입력을
읽는다. 동적 `task.runtime_factory`는 이 진입점의 필수 파라미터가 아니며 runtime을
선택하지 않는다. 정책·입력·명시적 arm 없이 출발하지 않는다.

실기·배포 launch는 기존 command_adapter 프로세스를 이 진입점으로 교체하고
동일 namespace/정책/heartbeat 설정 및 runtime 설정 경로를 넘겨야 한다. 둘을 동시에
실행하지 않는다. 실제 데이터 입력 제공자의 연결은 남아 있다.
단일 프로세스 설치와 SEARCH 입력 검사는
[`tests/test_robot_task_entrypoint.py`](../../tests/test_robot_task_entrypoint.py) 및
[`tests/test_robot_search_input.py`](../../tests/test_robot_search_input.py)에서
모의 입력으로 검증한다. 시계 역행은 runtime을 차단하며 정상 복구 후 새 runtime이 필요하다.

기존 `task_site_runtime` API의 `RETURN_HOME`은 수락할 때와 매 step에서 같은 `home_ref`의 현재 기록을 조회한다.
위치·frame·revision이 바뀌거나 조회가 실패하면 취소·정지 관문으로 들어가며
새 귀환 임무를 받아야 한다. 구간/귀환 cache 오류는 기동 중 로그에 남기고 guard를
거부한다. resolver와 검증 콜백은 검증된 로컬 cache를 읽으며 동기 RPC를 호출하지 않는다.

## 2026-10-12 단일 로봇 SEARCH 실기 준비

[사용자 요구, 2026-10-11] 로봇 1대가 SEARCH_AREA 한 건을 받아 실제 Nav2로 이동하는
단일 동작 통합 테스트를 준비한다. 구역 전체 경로 생성·카메라 수색 완료는 이 시험의 범위가 아니다.

```bash
colcon build --packages-up-to rescue_robot
source install/setup.bash
ros2 run rescue_robot task_input --ros-args --params-file <existing-adapter-params.yaml> \
  -p task.runtime_config:=<absolute-runtime-json-path> -r __ns:=<verified-namespace>
```

ROS 시작 파라미터는 위 표의 연결 이름·정지/readiness 기준을 유지한다. `task.params.yaml`은
전달 방법의 예시이며 필수 파일명이 아니다. runtime JSON은 `rescue-robot-runtime/1` 형식이고,
robot_id, mission_frame, schema_version, publisher_id, clock_error_bound_ns,
mapping_authority, max_position_error_m, max_rotation_error_rad, input_max_age_ns,
poll_period_s, input_file, lease의 duration_s/max_age_ns/max_error_ns/clock_domain_id,
endpoints의 action_name/lease_topic/admission_topic/status_topic을 명시한다.

inputs JSON은 `rescue-robot-inputs/1` 형식이며 generated_at_ns/valid_until_ns와
clock/authority/maps/destinations를 제공한다. SEARCH에는 targets/approvals/protected_routes가
필요 없다. search_area_ref로 선택한 destinations의 mission 좌표 끝 위치를 역변환하여
실제 Nav2 map 좌표로 보낸다. `transform_xy_yaw`는 map→mission 변환이며, 두 좌표계가
같은 것을 확인했으면 [0, 0, 0]을 사용한다. position_error_m/rotation_error_rad/evidence_id는
좌표 정합의 오차·검증 근거이고 Nav2 주행 후 도착 오차가 아니다.

노드를 DISARMED로 시작한 뒤 기동 로그/RobotStatus의 robot_boot_id를 inputs에 반영한다.
실제 시계·지도·odom·센서·TF·dock·Nav2·watchdog 준비를 확인하고 `~/task_arm`을 호출한다.
임무와 lease의 ID·발행자·권한·지도 버전은 입력 기록과 일치해야 한다. Nav2 성공만으로
완료하지 않고, action 종료 뒤 fresh odometry의 정지를 확인한다. 입력 만료·권한 철회·
목표/지도 변경은 취소·정지 확인으로 처리한다. 파일 읽기로 원 관측의 유효기간을 늘리지 않는다.
현재 변경에는 실기용 입력의 작성·주기 갱신 프로그램이나 실측 정책값을 포함하지 않는다.

| 준비 항목 | 담당안 | 비고 |
| --- | --- | --- |
| clock.* | 전제환 + 참여 PC 사용자 | 참여 호스트와 필요한 로봇 시각 동기·오차 확인 |
| map_frame, localization_status, transform_xy_yaw | 전제환 | 실제 설정·위치추정·좌표 관계 확인 |
| position_error_m, rotation_error_rad, evidence_id | 전제환·이태규 | 주행 전 정합 확인·근거 기록 |
| destinations[search_area_ref].xy_yaw | 전제환 | 도달 가능한 테스트 끝 위치·방향 선택 |
| authority.* | 전제환, 추후 임재범과 합의 | 테스트 발행자·세션 사용, robot_boot_id는 실행 노드에서 취득 |
| 지도/session/revision/sequence | 전제환·백승민 | 지도·정합 버전 공동 관리, 임무 발행 측과 일치 |
| 관측·생성·만료 시각 | 전제환, 정책은 백승민과 합의 | 실제 관측·생성 시각 기록, 유효기간 합의 |
| inputs JSON 작성·갱신, runtime 및 ROS 시작 설정 | 전제환 | 기존 설정 확인·조합, 실기 준비 후 연결 |

위 내용은 사용자의 테스트 준비 담당안이며 팀 합의 완료를 뜻하지 않는다. 테스트 조건과
정책은 PM 백승민과 합의 후 최종 확정한다. 로봇 번호·시험 시각·운용 임계값은 아직 정하지 않는다.

[`e3_search_nav2_probe.py`](../../tests/ros/e3_search_nav2_probe.py)는 사용하지 않는
LOCALHOST 도메인에서 SEARCH→NavigateToPose→lease→정지 결과의 실제 ROS 전송을 검사한다.
Nav2 서버·odom·준비 상태·시계·정합·권한은 TEST_ONLY 합성 입력이고 실제 planner/장비
주행은 검사하지 않는다. `python3 tests/ros/e3_search_nav2_probe.py --domain-id <unused-domain>`으로
실행하며 ROS Jazzy와 이 브랜치의 설치 overlay가 필요하다. 실제 CLI 시작 파라미터 검사는 별도다.

## 임무 입력 런타임과 watchdog 독립 개발 (2026-10-11)

아래는 기존 다종 임무 `task_site_runtime` API와 개발 도구의 설명이다. 코드·console script는
보존하며, 현재 기본 `task_input`의 SEARCH-only 수락 범위를 확장하지 않는다.

[사용자 요구, 2026-10-11] “watchdog 없는 상태에서 구현을 완료하고 watchdog을 나중에
추가하는게 맞지 않아? … 필요한거 너가 제작해.” 적용 범위는 임무 입력 구현·개발 검증이다.
watchdog 현장 파라미터 결정(#94)을 소스 작성의 선행 조건으로 두지 않는다.

#85의 [소유자 결정 전달](https://github.com/jaebeom/ROKEY-7/pull/85#issuecomment-6106387180)과
[제환 대상 작업 순서](https://github.com/jaebeom/ROKEY-7/pull/85#issuecomment-6106393544)를
반영해 공통 ExecuteTask/CommandLease/수락·결과·정지 및 SEARCH_AREA·REOBSERVE_TARGET을
먼저 연결한다. MOVE_SEGMENT는 예비 모드로 유지한다. 연속 추종의 담당·입력/액션·거리·유실·
도착 규칙은 작전·PM의 Q-27 결정 대상이며, 기존 구간 이동을 추종으로 바꾸지 않는다.
AMR1 관제 START_SEARCH는 중앙이 ExecuteTask를 발급하는 조건이다. 로봇은 버튼/지진
이벤트를 독자 처리하지 않는다. #90은 머지됐지만 버튼 전환과 최초 idle lease 관문은 후속이다.

`task_site_runtime:create_runtime`을 패키지에 포함했다. 사용자가 임의 factory/encoder를
새로 작성하지 않아도 된다. 한 CommandAdapter에 action/lease/admission/status를 붙이고,
`~/task_arm`, `~/task_stop`을 제공한다. 설치·재접속은 자동 arm이 아니다. watchdog 토픽이
없어도 endpoint를 생성하고 상태/거절 이유를 발행하지만, C-01의 실제 이동 READY 관문에서
watchdog가 없음을 정상으로 바꾸지 않는다. #92의 수동 Nav2 실습은 별도 구성이다.

watchdog와 실제 로봇 없이 현재 설치 런타임 전체를 재현한다:

```bash
python3 -m rescue_robot.task_runtime_demo
# 설치본: ros2 run rescue_robot task_runtime_demo
```

#85 요청 순서로 따로 검사하려면 먼저 `task_runtime_demo --stage common`, 다음
`task_runtime_demo --stage fallback`을 실행한다. common은 SEARCH_AREA·REOBSERVE_TARGET·HOLD,
fallback은 MOVE_SEGMENT만 실행한다. common은 최초 lease 기간보다 오래 유지하는 실제 갱신,
중복 갱신 거절, 취소 뒤 정지 확인, odom 없이 STOP_UNCONFIRMED도 모의 재현한다.
ROS 모드에도 `--stage common`/`--stage fallback`을 쓸 수 있다. 실제 ROS lease 갱신과
action 취소·결과는 현장 개발 PC에서 검사해야 하며 현재 준비 환경은 E3 NOT_RUN이다.

E2: SEARCH_AREA/REOBSERVE_TARGET/MOVE_SEGMENT/RETURN_HOME/HOLD와 lease 만료 취소를
돌린다. 좌표·권한·경로 보호·readiness·odom·Nav2 port는 명시적 SYNTHETIC이며 실기 증거가 아니다.
ROS/IDL을 설치한 PC에서, 실제 로봇 도메인과 겹치지 않는 사용하지 않는 테스트 도메인을
선택하고 Discovery Server를 끈 LOCALHOST 셸에서 `task_runtime_demo --ros --domain-id <선택값>`을
실행하면 실제 ExecuteTask/CommandLease/TaskAdmission/RobotStatus 왕복을 검사한다.
이 E3도 motion 입력은 합성이며 실제 Nav2/로봇 검증이 아니다.

CommandLease 거절 로그에는 lease/robot/boot/coordinator/authority/generation/sequence를
남긴다. 등록된 발행자·boot·권한 불일치, 중복/역행 sequence, 로컬 lease 만료와 STOPPING은
warning으로 구분한다. goal보다 먼저 저장한 lease와 다른 lease_id의 메시지는 debug로
기록한다. 이 진단은 lease 수락 조건이나 로컬 정지 기한을 바꾸지 않는다.

`task_prepare --directory <출력폴더> --robot-id <논리ID> --namespace <물리namespace>`는
runtime.json, inputs.json, task.params.yaml, preparation.json을 만든다. 기존 adapter
파라미터에 task.params.yaml을 겹쳐 로드한다. 결정된 수치를 `--settings <JSON>`으로 넣을
수 있고 미정값은 null과 NOT_CONFIGURED 목록으로 남는다. 예시·모의 수치를 현장에 승계하지
않으며, 재실행해도 기존 live inputs.json의 시각/내용을 덮어쓰지 않는다.

운영 입력은 `InputStore`가 읽는 로컬 trusted IPC 파일(`rescue-robot-inputs/1`)이다.
schema/clock/authority/maps/targets/approvals/protected_routes/destinations를 원 관측 시각과
만료, map/transform revision, 증거 ID와 함께 제공한다. 이는 중앙·지도·경로 보호 adapter의
실제 입력을 수용하는 경계다. 제어된 파일/디렉터리의 신뢰를 전제로 하며 DDS 암호 인증을
구현한 것은 아니다. 파일 읽기로 유효기간을 늘리지 않는다. 데이터 없음·가림·만료·지도 변경·
권한 철회·입력 형식 오류는 거절/취소가 되고 정상값으로 대체하지 않는다. 실제 producer의
지속적인 export/캘리브레이션·Nav2 보호 증거 연결은 E3/E4 통합에서 확인한다.

시각 측정은 각 관련 PC에서 `clock_capture capture --host-id <호스트역할> --clock-domain
<팀기준> --output <JSON>`을 실행한다. `--interval-s <주기>`로 계속 갱신할 수 있다. 도구는
chronyc tracking/sources와 chronyd 프로세스/boot를 읽고 원문·종료 코드·보수적 오차 추정치를
저장한다. 설정/시각을 변경하지 않는다. 두 결과는 `clock_capture combine --sender <JSON>
--receiver <JSON> --clock-domain <팀기준> --max-age-ns <정책> --max-error-ns <정책>
--output <clock-basis.json>`으로 합친다. 양쪽 오차를 합산하며 미동기·미측정·낡음·상한 초과는
UNKNOWN/거절이다. formula는 [chrony 공식 문서](https://chrony-project.org/doc/4.5/chronyc.html#tracking)
기준이며 공통 시간 기준의 정확성·팀 domain 선택까지 자동 증명하지 않는다.

소프트웨어 구현/합성 검사와 실제 ROS·장비 검증을 구분한다. 실제 움직임을 주장하려면 생성
IDL/Jazzy, 중앙/지도/clock/protection 입력, Nav2 종료와 fresh odom 정지, watchdog 최종 속도
배선을 현장에서 확인해야 한다. 별도 사이트 runtime 코드 작성 요청으로 미루지 않는다.

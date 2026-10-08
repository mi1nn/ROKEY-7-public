# rescue_robot

명령 수락·단일 활성 임무·임대·취소·정지 확인의 순수 Python 코어(`executor.py`)와, 그 코어를 Nav2 `NavigateToPose` 에 잇는 명령 어댑터(`nav2_adapter.py`, ROS 경계 `ros_node.py`)다. SLAM 실행·언도킹은 하지 않는다.

공개 API는 `Policy`, `Readiness`, `Task`, `GoalPose`, `Executor`, `CommandAdapter`, `MissionGoalBinder`, `ReadinessPolicy`, `ReadinessMonitor`다. 인수와 시간 진행 예시는 [`tests/test_executor.py`](../../tests/test_executor.py)와 [`tests/test_nav2_adapter.py`](../../tests/test_nav2_adapter.py)를 참고한다. 모든 운용 임계값은 호출자가 명시해야 한다. 시험값은 실기 기본값이 아니다.

자세한 구현 경계·출처·남은 검증은 [구현 기록](../../docs/implementation-robot.md)에 있다. 이 코어를 단독으로 로봇에 연결하지 않는다. 인증된 단일 어댑터, 시각/센서 증거, 장비측 watchdog이 필요한 상태다.

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

이 노드는 아직 **로봇을 움직이지 못한다**: 임무 입력(`ExecuteTask`, `rescue_interfaces` 미구현)이 없고, 이 프로세스에 정렬 입력(mapping ROS 어댑터)이 없어 guard 가 모든 dispatch 를 거절한다. 필요한 ROS 패키지: `rclpy`, `rcl_interfaces`, `nav_msgs`, `nav2_msgs`, `irobot_create_msgs`.

파라미터는 `declare_required` 로 타입만 선언한다(jazzy rclpy 는 값 없는 `descriptor=` 선언을 거부해 이전 코드는 기동하지 못했다). 빠진 값은 `missing parameters` 로 기동 거부한다.

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

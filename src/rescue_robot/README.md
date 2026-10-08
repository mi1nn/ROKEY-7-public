# rescue_robot

명령 수락·단일 활성 임무·임대·취소·정지 확인의 순수 Python 코어(`executor.py`)와, 그 코어를 Nav2 `NavigateToPose` 에 잇는 명령 어댑터(`nav2_adapter.py`, ROS 경계 `ros_node.py`)다. SLAM 실행·언도킹은 하지 않는다.

공개 API는 `Policy`, `Readiness`, `Task`, `GoalPose`, `Executor`, `CommandAdapter`다. 인수와 시간 진행 예시는 [`tests/test_executor.py`](../../tests/test_executor.py)와 [`tests/test_nav2_adapter.py`](../../tests/test_nav2_adapter.py)를 참고한다. 모든 운용 임계값은 호출자가 명시해야 한다. 시험값은 실기 기본값이 아니다.

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
- **goal frame**: 어댑터는 `task.goal_pose` 를 그대로 보낸다. Nav2 는 로봇 자신의 map frame 목표만 이해하므로, mission 기준 좌표(프로파일 v2 의 구역·대기 위치)를 로봇 map 으로 바꾸는 일은 mapping 의 책임이고 이 어댑터에는 아직 없다(후속).
- **odom 시각 [미확인]**: `sample_steady = received_steady − (wall_now − header_ns)/1e9`. 로봇과 PC 시계가 `clock_error_bound_s`(로컬 float 초; C-03 의 int `clock_error_bound_ns` 와 다름) 안에서 맞는다고 전제한다(측정 전). |차이| 가 상한을 넘거나 stamp 가 무효이면 표본으로 쓰지 않고 `Executor.invalidate_stop_evidence` 로 정지 창을 초기화한다 → 증거 부족은 `STOP_EVIDENCE_STALE`·`STOP_UNCONFIRMED` 로 드러난다.
- 두 로봇은 같은 어댑터를 쓰고 task kind 만 다르다(SCOUT: `SEARCH_AREA`·`REOBSERVE_TARGET`, SUPPORT: `HOLD`·`APPROACH_TARGET`). 역할은 설정에서 오고 코드에 두지 않는다. `HOLD` 는 goal 없이 정지 확인 뒤 `COMPLETED` 다.

## ROS 노드 (`command_adapter`)

`ros_node.main` 은 기본값 없는 파라미터만 받는다. 프로파일(`config/deployment.*.json`)의 값은 bringup 이 넘긴다(이 패키지는 `rescue_bringup` 을 import 하지 않는다).

| 파라미터 | 출처 |
| --- | --- |
| `allowed_robot_ids`(문자열 배열), `robot_id`, `namespace` | 프로파일의 두 로봇 ID 와 해당 로봇 namespace. 노드 namespace 와 다르면 기동 거부 |
| `nav_action`, `odom_topic`, `dock_status_topic` | namespace 기준 이름. 프로파일 v2(작전 PR)가 들어오면 그 값을 쓴다 |
| `authority_epoch`, `policy_version`, `policy.*`(7개), `step_period`, `clock_error_bound_s`(초) | 운용 정책. 실기 값 미정(T-01) |

이 노드는 아직 **로봇을 움직이지 못한다**: 임무 입력(`ExecuteTask`, `rescue_interfaces` 미구현)이 없고, 지도 revision guard 가 연결되기 전까지 guard 가 모든 dispatch 를 거절한다. `driver`·`sensors`·`slam`·`watchdog` 준비 provider 도 아직 없다. 필요한 ROS 패키지: `rclpy`, `rcl_interfaces`, `nav_msgs`, `nav2_msgs`, `irobot_create_msgs`.

## 검사

```bash
python3 -I scripts/run_tests.py                     # E2, ROS 없이 (tests/test_nav2_adapter.py 포함)
# E3 일부: ROS 2 환경을 source 한 뒤, 로봇 도메인과 겹치지 않는 그래프에서
ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  PYTHONPATH=$PWD/src/rescue_contracts:$PWD/src/rescue_robot:$PYTHONPATH \
  python3 tests/ros/e3_odom_probe.py
```

E3 탐침은 실제 `nav_msgs/Odometry` 토픽 두 개(`/robot_a/odom`, `/robot_b/odom`)가 각 실행기에만 들어가는지 본다. `NavigateToPose` action 은 다루지 않는다.

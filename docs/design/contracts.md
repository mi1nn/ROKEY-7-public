---
title: 구조 수색 시스템의 인터페이스 계약 초안
status: draft-proposal
updated: 2026-10-14
---

# 인터페이스 계약 초안

이 문서는 계획 PR 3의 계약과 후속 Python 코어 구현의 공통 의미를 관리한다. C-02/C-03/C-09의 실행 가능한 Python 계약은 `rescue_contracts`에 두며 ROS IDL `rescue_interfaces`는 C-10의 1차 범위만 정의했다(2026-10-10). PR 2의 책임·배포 구조를 전제로 검토하되, 실제 `.msg/.srv/.action`, 패키지 코드, ROS 설정을 이식하지 않는다. 장비·OS·ROS 배포판·센서 시각 의미·네트워크는 아직 확정되지 않았다. 아래 타입 이름과 토픽·QoS는 **계약 제안**이며 구현·실기 검증 완료를 뜻하지 않는다. 수치의 확정 절차는 [시험 계획](../process/acceptance-plan.md), 사건·임무 전이는 [상태 전이](state-machines.md)에 둔다.

두 AMR 활용은 사용자 확정 요구다. 고사양 PC 2대·중간 사양 PC 2대는 예정 자원이며, 네 대를 모두 사용하는 배포와 발견→안전 접근→보고는 현재 설계 기준안이다. 이 문서의 계약을 최종 인수 범위가 모두 확정된 것으로 해석하지 않는다.

## 한눈에 — 조정기 기준 (2026-10-15, main `5d9304a`)

이 절이 답하는 질문은 하나다. **이 문서의 계약 중 임무 조정기가 지금 실제로 쓰는 것은 무엇인가?** 아래 C-01~C-11 본문은 여러 패키지가 함께 쓰는 계약이라 그대로 두었다. 조정기 칸은 `rescue_coordination` 코드를 읽고 채웠다. 다른 패키지 칸은 그 담당의 확인이 필요하다.

| 계약 | 조정기에서 지금 | 조정기 근거 | 다른 패키지 |
| --- | --- | --- | --- |
| C-01 책임과 명령 권한 | 사용. 조정기는 임무(`ExecuteTask`)만 보내고 `cmd_vel`·Nav2 액션을 직접 호출하지 않는다 | `ros_node.py` `main` | 확인 필요: 통합-로봇, 통합-인식 |
| C-02 ID·envelope | 사용. 모든 발행 메시지에 envelope 을 채운다 | `main` 안 `envelope` | 확인 필요: 각 담당 |
| C-03 시각·최신성 | 사용. 위치 관측의 시계 기준·나이·미래 시각, 로봇 상태의 나이를 검사한다 | `_check_observation_time`, `ready` | 확인 필요: 통합-인식, 통합-로봇 |
| C-04 좌표·지도 revision | 사용. 관측·목표마다 지도 기준(session·revision·transform)을 맞춘다 | `on_target_location`, `_route_frame`, `_lineage` | 확인 필요: 통합-로봇 |
| C-04a 고정 지도 revision 짝 | 사용. hash 가 맞는 revision 글만 받는다 | `on_frozen_revision`, `_try_freeze` | 통합-로봇(만드는 쪽) |
| C-04b 터널 목록 | 쓰지 않음. 탐색 경계 선택에 쓰는 것은 후속이다 | `rescue_coordination/` grep `overhead` 결과 없음 | 인식(만드는 쪽), 운영(DB 기록, #262) |
| C-05 관측·사건·보고 스키마 | 일부 사용. `TargetLocation`, `SiteEvent`, `RobotStatus`, `RescueReport`, `ReportReceipt` 를 쓴다. `TargetObservation` 은 직접 받지 않는다(인식의 `target_locator` 가 `TargetLocation` 으로 바꿈) | `main` 의 구독·발행 목록 | 확인 필요: 통합-인식, 운영 |
| C-06 임무 action·lease·정지 | 일부 사용. `ExecuteTask`·`CommandLease`·`TaskAdmission` 을 쓴다. `RequestStop` 은 쓰지 않는다. 노드가 발급하는 종류는 `SEARCH_AREA`, `APPROACH_TARGET`, `RETURN_HOME`, `REOBSERVE_TARGET`(RECHECK 전용), `MOVE_SEGMENT`(운영자 구간 승인 전용)다. `HOLD` 는 코어(`search.py`)에만 있고 노드가 발급하지 않는다. 교통 관제(`traffic_control=TB5_PRIORITY`, #272)는 TB-5 경로 근처를 가는 TB-6 임무를 `ExecuteTask` goal 취소로 멈춘다. 실행기는 취소를 받으면 `request_stop('CANCEL_USER')` 로 로봇을 세운다(`rescue_robot/task_server.py` `on_cancel`) | `_issue_*`, `leases_due`, `_stop_scout`, `take_cancels`, `main` 안 `cancel_task` | 확인 필요: 통합-로봇 |
| C-07 안전 접근·통로 자원 | 일부 사용. 접근점은 대상에서 `approach_standoff_m` 떨어진 점이고, 고정 지도에서 빈 자리(`footprint_clear`)인지 확인한다. 통로 점유 코어(`planning.py` `confirm_vacant`)는 노드에 연결되지 않았다 | `approach_point`, `_free_cell` | 확인 필요: 통합-로봇 |
| C-08 토픽·QoS 제안 | 제안과 다르다. 실제 토픽은 아래 「조정기가 실제로 쓰는 토픽」 표다. `/rescue/observations`, heartbeat, `/rescue/frame_alignments` 는 조정기가 쓰지 않는다 | `ros_node.py` `*_TOPIC` 상수 | 확인 필요: 각 담당 |
| C-09 오류 코드 | 사용. 조정기 오류(`CoordinationError`)는 `RescueError` 의 공통 코드를 쓴다 | `planning.py` `CoordinationError` | 공통(`rescue_contracts`) |
| C-10 ROS IDL 1차 | 일부 사용. msg 12개, srv 2개(`SubmitOperatorCommand`, `SetCameraWatch`), action 1개(`ExecuteTask`) | `main` 의 import | 확인 필요: 각 담당 |
| C-11 소실 후보 → 재관측 → 판정 | `completion_mode=RECHECK` 에서만 사용. 현장 설정(`EXIT_ARRIVAL`)은 TB-5 출구 도착을 구출 완료로 본다(2026-10-13 밤 결정) | `_open_camera_recheck`, `_judge`, `_confirm_by_exit_arrival` | 확인 필요: 웹캠 담당 |
| C-12 TB-5 다리 추종 | 쓰지 않음. 2026-10-13 결정으로 다리 스켈레톤 추종을 뺐다(scope Q-29 ⑤). 본문은 맨 아래 「기록」으로 옮겼다 | — | — |
| C-13 TB-6 관제 버튼 출동 | 바뀜. 현장은 재난 신호로 탐색을 자동 시작한다(`search_start=AUTO`, OPS-136). 버튼 방식(`OPERATOR`)도 코드에 남아 있다. 본문은 「기록」으로 옮겼다 | `tick`, `on_command` | — |
| 운영자 재난 선포 `DECLARE_DISASTER` | 사용(2026-10-15 결정). 관제 웹 버튼으로 NORMAL 을 DISASTER 로 바꾼다. 필드는 `command_id`·`operator_id` 뿐이고 `robot_id` 를 넣으면 `INVALID_INPUT` 이다. 지진 감지(`DISASTER_SIGN`)와 같은 길로 들어가며, `search_start=AUTO` 면 다음 판단 주기에 탐색을 시작한다. `SystemState` 의 `trigger_event_id` 는 `command_id`, `trigger_observed_at_ns` 는 조정기가 받은 시각이다. `cause_event_ids` 는 비우고 `reason` 이 다른 사유가 없을 때 `DISASTER_DECLARED_BY_OPERATOR` 를 싣는다. 가짜 `SiteEvent` 를 만들지 않으므로 사건 목록과 KPI-04 표시 지연에는 들어가지 않는다. 이미 재난 모드면 `TASK_BUSY`(임무 관문 이유 `ALREADY_DISASTER`)로 거절한다. srv 상수와 SDD JSON 은 아직 없다(`operation` 은 문자열이라 그대로 실린다) | `mission.py` `MODE_COMMAND_FIELDS`·`_declare_disaster`, `on_command` | 운영(`command_api` 허용), 웹개발(버튼) |

### 조정기가 실제로 쓰는 토픽·서비스

`ros_node.py` 의 상수와 `main` 을 읽어서 적었다. 로봇별 이름의 `<id>` 는 프로파일의 로봇 ID(`robot_a`, `robot_b`)다.

| 이름 | 타입 | 방향 |
| --- | --- | --- |
| `/rescue/site_events` | `SiteEvent` | 받음(웹캠) |
| `/rescue/target_locations` | `TargetLocation` | 받음(인형 위치) |
| `/rescue/exit_location` | `TargetLocation`(`exit_qr`) | 받음(QR 출구) |
| `/rescue/robots/<id>/status`, `.../pose`, `.../map_state`, `.../task_admission` | `RobotStatus`, `RobotPose`, `MapState`, `TaskAdmission` | 받음(로봇) |
| `/rescue/robots/<id>/execute_task` | `ExecuteTask`(action) | 보냄. 교통 관제는 TB-6 goal 을 취소함 |
| `/rescue/robots/<id>/command_lease` | `CommandLease` | 보냄 |
| `/rescue/reports` / `/rescue/report_receipts` | `RescueReport` / `ReportReceipt` | 보냄 / 받음 |
| `/rescue/system_state`, `/rescue/targets` | `SystemState`, `TargetSnapshot` | 보냄(latched) |
| `/rescue/operator_command` | `SubmitOperatorCommand`(srv) | 받음(운영) |
| 웹캠 `SetCameraWatch` 서비스 | `SetCameraWatch`(srv) | 보냄 |
| `/rescue/map/freeze` | `std_srvs/Trigger` | 보냄(지도 고정 요청) |
| `/rescue/map/frozen`, `/rescue/map/frozen_revision` | `OccupancyGrid`, `std_msgs/String` | 받음 |
| `/rescue/map/start_pose/<TB-6 id>` | `PoseWithCovarianceStamped` | 받음(TB-6 출발 자세) |
| `/robot6/map` | `OccupancyGrid` | 받음(TB-6 실시간 지도) [추정: 실기 미확인] |
| `~/task_inputs`, `~/refresh_state` | 파라미터 조회, `std_srvs/Trigger` | 로봇이 조회 |


## C-01. 책임과 명령 권한

| 책임 | 유일한 소유자 제안 | 허용/금지 경계 |
| --- | --- | --- |
| 센서 드라이버·최종 명령 감시 | 각 로봇의 드라이버/로컬 watchdog | 최종 속도 제한·명령 만료·정지 latch를 로봇에서 적용. 정확한 설치 방법은 장비 확인 필요 |
| 로봇별 Nav2/SLAM 실행·임무 실행 | `rescue_robot` + `rescue_bringup`의 로봇별 인스턴스 | Nav2 goal/cancel·도킹 장치의 준비/운영 action은 로봇 실행기만 발행. 협업 임무와 준비용 UNDOCK/DOCK 권한을 분리(C-06) |
| 지도 세션·공통 좌표 변환 | `rescue_mapping` | frame/session/revision의 권한자. perception/coordinator가 변환을 임의 보정하지 않음 |
| 관측 생성 | `rescue_perception` | 탐지·거리·위치·관측 품질 발행. 임무·Nav2 goal·속도 발행 금지 |
| 수색 분담·대상 사건·임무 배정 | `rescue_coordination` | 로봇 실행기에 임무 요청. 직접 `cmd_vel`·Nav2 action·lifecycle 변경 금지 |
| 운영자 요청·보고 수신·영속 기록 | `rescue_operations` | API 요청을 coordination/robot 계약으로 전달. 직접 Nav2/속도 발행 금지 |
| ROS 메시지와 버전 | `rescue_interfaces` 담당 1명 | 관련 소비자·시험의 변경 영향과 함께 수정. 각 구현 세션이 독자 계약을 추가하지 않음 |
| 고정 카메라의 현장 사건(`SiteEvent`) | `rescue_perception` 의 고정 카메라 어댑터가 생성, `rescue_coordination` 이 소비 | 재난 징후(`DISASTER_SIGN`, 판정 조건은 정책 값·Part I 는 지진)·대상 소실 후보(`TARGET_DISAPPEARED_CANDIDATE`, 2026-10-09 카메라 1대 결정)는 **증거**이지 명령이 아니다. `EXIT_PASS` 는 이미 저장된 자료를 읽을 수 있게 어휘에만 남기며, 카메라 1대 구성에서는 만드는 쪽이 없다. 재난 모드 전환·출동 지시는 coordination 의 정책이 하고, 카메라 어댑터는 주행·모드 변경을 발행하지 않음(BRD v0.1 UC-01·06, 2026-10-08) |

고사양 PC 두 대에 로봇별 SLAM/Nav2/실행·추론을 배치하고, 중간 사양 PC 두 대에 coordination과 operations를 나누는 것은 부하 측정 전 가설이다. 순수 판단 로직은 ROS 입출력 어댑터와 분리한다. 파일 경계는 PR 2와 함께 확정한다.

Nav2·탐색 회전·수동 명령은 모두 같은 로봇별 명령 중재와 최종 watchdog 경로를 통과해야 한다. `explore_lite`는 통합 시 후보 생성 방식(B)을 평가하며 실행기가 후보를 검증해 Nav2 goal을 발행한다. `explore_lite`가 직접 goal을 보내는 방식(A)은 coordinator/controller가 같은 로봇을 제어하지 않는 격리 실험으로 한정한다. perception이 별도 회전 명령을 보내는 경로는 두지 않는다. 원격 연결이 끊겨도 로봇에서 명령 만료를 처리할 수 있다는 근거를 얻기 전에는 원격 자율주행 준비 완료로 판정하지 않는다.

## C-02. ID·버전·공통 envelope

- 논리 로봇 ID는 초안에서 `robot_a`, `robot_b`로 쓴다. 실제 robot3/robot6 등 장비 이름과의 매핑은 배포 설정이다.
- 모든 사용자 정의 메시지는 전송 envelope에 `schema_version`, `publisher_id`, `publisher_boot_id`, `sequence`를 가진다. publisher_boot_id는 발행 프로세스 기동별 UUID, sequence는 같은 발행 boot 내 단조 증가 `uint64`다. 원 관측 payload의 `source_robot/camera`, `source_boot_id`, 원 관측/발생 시각은 별도로 보존한다. 중계자의 identity를 원 센서/관측 생성자의 identity로 덮어쓰지 않는다.
- ID(`observation_id`, `target_id`, `task_id`, `command_id`, `report_id`)는 문자열 UUID를 제안한다. `generation`·revision·sequence는 `uint64`; 단위는 거리 m, 각도 rad, 시간 s/ns로 명시한다. NaN/Inf, 음수 거리·분산, 누락 필수 ID는 거절한다.
- 메시지 버전의 major 불일치는 자동 변환하지 않고 `INCOMPATIBLE_SCHEMA`로 거절한다. 허용하는 minor 확장과 기본값은 구현 전에 명시한다.
- 불변 관측·명령·보고의 재전송은 같은 ID와 같은 **의미 payload**다. 중복 검사에 사용하는 정규화 payload 해시에서 published_at, publisher identity/boot, 전송 sequence, 수신 시각·재전송 횟수 같은 전송 envelope는 제외한다. observed_at, 원 source identity/boot, 대상/임무·좌표 참조, 최초 deadline, 원 증거 해시는 포함한다. 같은 ID의 의미 payload가 다르면 `ID_CONTENT_CONFLICT`로 거절한다. 재시도 횟수가 늘어도 최초 deadline을 늘리지 않는다.
- 변경 가능한 상태는 ID만으로 중복 판정하지 않는다. TargetEstimate는 `(target_id, estimate_revision)`, 로봇 상태는 `(robot_id, robot_boot_id, state_sequence)`, 좌표 snapshot은 authority/session/revision으로 버전을 구분한다. 더 높은 정상 revision은 업데이트이며 같은 키의 다른 의미 payload만 충돌이다. ReportReceipt는 자체 receipt_id와 참조 report_id를 가지므로 RECEIVED→PERSISTED 전이를 보고 payload 변경과 혼동하지 않는다.
- boot 변경 시 이전 boot의 sequence를 새 sequence와 비교하지 않는다. 이전 boot의 임무·lease·frame 세션을 명시적으로 폐기하고 준비 절차를 다시 밟는다. retained 상태 한 개를 받았다는 이유로 새 boot가 READY가 되지 않는다.

로봇의 **역할**(`SCOUT` 정찰 / `SUPPORT` 구조 지원, BRD v0.1 TB1/TB2)은 배포 프로파일 v2 `mission.roles` 의 데이터이며 정확히 하나씩이다. 코드는 역할을 상수로 두지 않고 프로파일에서 받는다(역할을 바꿔 넣으면 두 로봇이 뒤집혀 동작해야 한다). 고정 카메라는 `cam_<id>` 형식의 `camera_id` 로 식별하며 `robot_id` 집합에 들어가지 않는다. `robot_id` 값은 예시 `robot_a`/`robot_b`로 고정하지 않는다. DTO는 `[A-Za-z][A-Za-z0-9_]{0,63}` 형식을 검사하고 실행기·배정기·보고 저장소 경계는 검증된 배포 프로파일에서 주입한 정확히 2개의 distinct ID를 허용한다. 기본 allowlist는 없으며 수신 데이터가 설정 집합을 변경하지 못한다.

## C-03. 시각·최신성 계약

Python 패키지 사이의 절대 시각은 양의 **int64 나노초** (`*_at_ns`, `valid_until_ns`, `deadline_ns`, `now_wall_ns` 등)로 고정한다. bool/float/0/음수/범위 초과는 거절한다. ROS header는 `ros_time_to_ns(int sec, int nanosec)`로 합성하고 `ns_to_ros_time`의 정수 divmod로 복원한다. 원 stamp를 float 초로 변환했다가 되돌리지 않는다. 실제 epoch `1700000000123456789` ns를 왕복해 하위 ns까지 보존하는 회귀시험을 유지한다. 문서의 suffix 없는 논리 필드도 이 표현을 따른다.

수신 프로세스 **내부** monotonic deadline/lease/stop-window는 유한 float 초를 사용할 수 있다. 이 값은 직렬화하거나 다른 PC의 monotonic 시각과 비교하지 않는다. 절대 시각에 적용하는 기간/오차 허용치는 정수 ns다. `seconds_to_ns`는 십진 정책값·합성 fixture 변환 전용이며 이미 float로 손상된 sensor stamp 복구 수단이 아니다.


| 값 | 의미와 사용 |
| --- | --- |
| `observed_at` | 원 센서 관측의 ROS system-time stamp. 재발행·변환·DB 저장 시 바꾸지 않음 |
| `stamp_semantics` | `CAPTURE`, `DRIVER_RECEIVE`, `UNKNOWN`. 실제 드라이버가 어느 시점을 찍는지 확인. 수신 stamp를 캡처 stamp로 부르지 않음 |
| `published_at` / `issued_at` | 메시지 발행/명령 최초 발급 시각. 원 관측 시각과 별개 |
| `valid_until` / `deadline` | 원 관측 또는 임무 정책에서 정한 절대 유효 시한. 중계가 연장하지 않음 |
| `received_steady` (로컬 초) | 각 수신 프로세스 내부의 monotonic 시각. 통신 watchdog·처리 지연에 사용하며 다른 PC의 monotonic 값과 비교하지 않음 |
| `clock_domain_id`, `clock_state`, `clock_error_bound` | 동일 시간 기준인지, 동기화 상태·최대 오차 추정이 사용 가능한지 표시. 단순한 header-수신 시각 차이만으로 동기화를 주장하지 않음 |

실기에서는 system time을 공통 기준으로 사용하고 `use_sim_time=false`를 제안한다. bag/시뮬레이션은 별도 profile·`clock_domain_id`를 사용하며 실제 로봇 명령으로 연결하지 않는다. replay 중 시간 되감김은 새 세션으로 처리하고 이전 lease/목표를 재사용하지 않는다.

원 관측의 최대 나이는 `수신측 현재 wall time - observed_at + 송수신 clock_error_bound 합 + 검증된 센서 stamp 지연 상한`으로 보수적으로 평가한다. 오차 상한을 알 수 없거나 `UNKNOWN` stamp이면 자동 접근의 유효 관측으로 쓰지 않는다. 표시·오프라인 기록에는 `시간 불확실`로 남길 수 있다. 유효성은 **원 관측 나이, 로컬 수신 단절 시간, 필요한 TF/센서 상태**를 모두 통과해야 한다. 어느 하나의 PASS가 다른 조건을 대신하지 않는다.

**짧은 탐지 공백과 만료는 다르다.** 대상에 의존하는 진행 중 임무는 마지막 유효 관측·기존 goal의 좌표/시간/lease·필수 센서가 계속 유효하고, 원 observed_at가 전진하는 마지막 새 유효 관측의 로컬 monotonic 수신 이후 공백이 T-01의 `SHORT_LOSS_MAX` 안일 때만 기존 goal을 유지할 수 있다. 허용 시간은 원 관측의 남은 유효시간과 SHORT_LOSS_MAX 중 먼저 끝나는 시점까지다. 새 탐지가 없다는 이유로 새 goal을 발급하거나 observed_at/valid_until을 갱신하지 않는다. hold/중복 재발행은 공백 타이머를 초기화하지 않으며, 이동 대상의 미래 위치를 임의로 외삽하지 않는다. 관측 만료·공백 상한 초과·좌표/시간/lease/필수 센서 유효성 상실 중 하나라도 발생하면 즉시 취소→정지 관측으로 전환한다. 이 조건은 watchdog을 늦추거나 만료된 관측으로 접근을 시작할 권한을 주지 않는다.

`SEARCH_AREA`는 수색 자체에 불필요한 특정 대상의 관측 만료만으로 중지하지 않는다. 해당 수색의 필수 scan/pose/인지 준비나 명령 권한이 상실되면 별도 readiness/정지 관문을 적용한다. 대상 의존 `APPROACH_TARGET`과 `REOBSERVE_TARGET`에는 위 공백/만료 계약을 적용하며, 오래된 위치 주변을 새로 수색하는 재관측은 S-03의 별도 구역 정책과 새 generation이 필요하다.

0·허용 오차보다 미래인 stamp·너무 오래된 stamp·다른 clock domain은 자동 임무 입력에서 거절한다. 시계 step/restart를 감지하면 활성 접근을 보류/정지하고 시간 기준을 다시 확인한다. 시간 임계값은 T-01~T-06을 실기 전에 고정한다.

## C-04. 좌표계·지도 revision 계약

| 좌표/식별자 | 의미 |
| --- | --- |
| `mission` | 협업에 사용할 공통 기준 frame 제안. 물리 기준점과 정렬 방법은 PR 2의 실험으로 결정 |
| `robot_a/map`, `robot_a/odom`, `robot_a/base_link` | 로봇 A/B를 구분하기 위한 논리 이름 예시. 장비의 실제 frame 문자열과 대응은 프로파일로 관리하며 접두사 없는 map/odom/base_link일 수도 있음. 토픽 namespace만으로 TF frame ID가 자동 분리되지 않음 |
| `map_session_id` | SLAM 시작·리셋·기준 지도 교체 등 새 좌표 세션의 ID |
| `map_revision` | loop closure 등 저장 관측의 지도 좌표 해석을 바꾸는 보정 세대. 매 센서/TF 업데이트 번호와 구분 |
| `map_update_sequence` | 점유 지도 내용 업데이트 순서. 이것만으로 공통 좌표 정렬 품질을 판정하지 않음 |
| `transform_revision` | mission↔로봇 map 정렬의 세대. authority와 유효 조건을 함께 기록 |

접두사 없는 native frame을 사용할 때는 로봇별 ROS domain 또는 context·토픽 remap·전용 TF buffer로 수신을 격리하고 `source_robot + native_frame`으로 해석한다. 단일 그래프에서 두 TF tree를 합치는 경우에는 충돌 없는 실제 frame ID/변환 어댑터를 검증해야 한다. 공유 TF buffer에 같은 `map/odom/base_link`를 그대로 합치지 않는다.

각 로봇의 SLAM은 자신의 `map→odom`, odometry는 `odom→base_link`, 고정 센서 TF는 각 장비의 정해진 발행자가 소유한다. `rescue_mapping`은 필요한 `mission→robot_i/map` 정렬의 단일 authority를 관리한다. TF 다중 부모·동일 edge의 중복 발행자를 허용하지 않는다. 구체적인 publisher 배치는 PR 2에서 제안하고 실제 그래프 검증 후 확정한다.

revision 생성은 실제 선택한 SLAM backend의 보정 이벤트·pose graph/세션 연계와 연결되어야 한다. 일반적인 map→odom TF 변화만 보고 loop closure가 발생했다고 추정하지 않는다. 현재 설치 버전에서 보정 발생을 감지하고 영향 관측을 판별할 수 있는지 확인하지 못하면 alignment의 자동 협업 접근 자격은 INVALID로 둔다. 독립된 기준으로 좌표를 재검증/재관측하여 유효성을 확보하기 전에는 A의 과거 좌표로 B를 자동 접근시키지 않는다. 실험실에서 조작한 revision 번호만 통과한 시험은 backend 연동 시험을 대신하지 않는다.

`FrameAlignment`는 `source_frame`, `target_frame`, `map_session_id`, `map_revision`, `transform_revision`, 변환, 위치/회전 불확실성, `valid_from`, `valid_until`, `status={VALID,DEGRADED,INVALID}`를 가진다. TF 메시지만으로 revision/불확실성을 표현할 수 없으므로 해당 메타데이터를 함께 검증한다.

관측의 원 sensor frame·원 stamp·sensor 좌표와 그때 사용한 보정/변환 참조는 불변 보존한다. mission 좌표는 파생 결과이며 원본을 덮어쓰지 않는다. 원 관측 시각의 올바른 TF가 없을 때 최신 TF로 조용히 대체하지 않는다.

좌표 해석에 영향을 주는 revision 변경 시: 새 접근 배정 중단 → 영향받은 goal 취소/정지 처리 → 원 관측을 새 기준으로 재계산할 수 있는지 판정 → 가능하면 새 파생 위치, 불가능하면 `NEEDS_REOBSERVATION` → 새 generation으로 재배정한다. loop closure가 항상 단일 강체 변환으로 수정 가능하다고 가정하지 않는다. 변환 갱신과 새 목표 수락 사이의 경합은 executor가 수락 시점과 실제 goal 발행 직전에 revision을 다시 확인해 거절한다.

`Projection.status`의 공통 어휘는 `VALID`, `STALE_DERIVATION`, `NEEDS_REOBSERVATION`, `EXPIRED`다. 유효한 map→mission 정렬만 변경되어 같은 원 관측·sensor-time TF로 재계산할 수 있는 파생 위치는 `STALE_DERIVATION`이다. map session/map revision 변경 또는 정렬 무효화로 원 관측의 새 위치를 증명할 수 없으면 `NEEDS_REOBSERVATION`이다. 상태 확인에서 VERSION_MISMATCH를 감지한 경우에도 같은 기준을 적용한다. STALE_DERIVATION 뒤 지도 보정·세션 교체·무효화가 발생하면 NEEDS_REOBSERVATION으로 승격한다. 원 관측/파생 위치 유효기한이 끝나면 EXPIRED이며 중계가 기한을 연장하지 않는다. VALID 이외의 상태는 기존 goal의 유효 근거가 아니며, 새 파생 위치 또는 새 관측 확인 후 별도 generation으로 재배정한다. VALID만으로 실제 backend 검증/자동 접근 자격을 승인하지 않는다.


### C-04a. 고정 지도의 revision 짝 (`/rescue/map/frozen_revision`, 2026-10-13, OPS-154/159)

TB-6 SLAM 지도는 회차마다 두 번 고정한다: R0+1(인형 발견, TB-5 출동·TB-6 은 SLAM 계속)과 R0+2(탐색 완료, TB-6 SLAM→AMCL·TB-5 정지 상태에서 재정합). 두 고정 모두 `/rescue/map/frozen`(nav_msgs/OccupancyGrid, RELIABLE·TRANSIENT_LOCAL·depth 1, 마지막 고정)으로 나가고, OccupancyGrid 에는 revision 칸이 없으므로 고정 노드(`map_freeze`)가 짝을 함께 낸다.

| 항목 | 값 |
| --- | --- |
| 토픽 | `/rescue/map/frozen_revision` |
| 형식 | std_msgs/String, RELIABLE·TRANSIENT_LOCAL·depth 1. 새 IDL 없음 |
| 내용 | JSON 객체 하나, 필드는 정확히 `format`(`"rescue-map-frozen/1"`), `map_session_id`(UUID 문자열), `map_revision`(정수 ≥ 1), `occupancy_sha256`(소문자 hex 64자, `rescue_contracts.occupancy.occupancy_sha256`) |
| 해석 | `rescue_contracts.occupancy.parse_frozen_revision` 하나로만 한다(로봇·조정기 공용). 다른 형식, 빠진·남는 필드, 잘못된 값은 거절(ValueError) |
| 짝 규칙 | 받은 grid 의 해시가 짝의 `occupancy_sha256` 과 같을 때만 그 grid 가 짝의 `map_revision` 이다(`paired_revision`). 다르면 revision 을 추정하지 않고 맞는 짝이 올 때까지 기다린다 |

같은 revision 에 다른 해시가 오면 지도를 막고 새 회차를 요구한다. 낮은 revision 이 늦게 오면 쓰지 않는다. 짝·grid 가 latched depth 1 이므로 R0+2 뒤에 붙은 구독자는 R0+1 grid 를 받지 못한다. 고정 노드가 재시작하면 그 회차 기록을 R0+1→R0+2 순으로 다시 낸다.

### C-04b. 지도의 머리 위 구조물 목록 (`/rescue/robots/<id>/overhead_passages`, 2026-10-13, OPS-185/186)

각 로봇의 `overhead_passage` 가 앞쪽 머리 위 구조물(터널)을 판정해 로봇별 목록 전체를 낸다. 웹 지도는 목록을 표시하고, db_writer 는 기록한다. 조정기는 나중에 frontier 를 고를 때 쓸 수 있다.

| 항목 | 값 |
| --- | --- |
| 토픽 | `/rescue/robots/<robot_id>/overhead_passages`(로봇별, 받는 쪽이 합침) |
| 형식 | std_msgs/String, RELIABLE·TRANSIENT_LOCAL·depth 1. 새 IDL 없음. 목록이 바뀔 때마다 전체를 다시 낸다. 기동 때 빈 목록을 한 번 낸다 |
| 내용 | JSON 객체 하나. 필드는 정확히 `format`(`"rescue-overhead-passages/1"`), `robot_id`, `map_frame`, `generated_at_ns`, `clearance_m`(통과 기준, 현장 0.46), `floor_noise_m`, `lidar_plane_m`, `items` |
| 항목 | 필드는 정확히 `id`(UUID), `state`(`PASSABLE`·`BLOCKED`·`UNKNOWN`), `lowest_m`(UNKNOWN 이면 null), `x`·`y`(map_frame 의 바닥 중심), `map_session_id`(UUID)·`map_revision`(정수 ≥ 0, 잰 때의 지도), `frames`, `first_seen_ns`·`last_seen_ns`, `reason` |
| 일관성 | PASSABLE ⇔ `lowest_m` ≥ `clearance_m`. BLOCKED 는 그 아래. UNKNOWN 은 높이를 주장하지 않는다(바닥을 못 봄, `NO_FLOOR_SEEN`). `0 ≤ floor_noise_m < lidar_plane_m < clearance_m`. id 는 서로 다르다 |
| 해석 | `rescue_contracts.overhead.parse_overhead_passages` 하나로만 한다(웹·db·조정기 공용). 다른 형식, 빠진·남는 필드, 어긋난 값은 거절(ValueError) |

항목은 잰 때의 revision 을 그대로 가진다. 지도를 고정한 뒤(R1, R2) 이전 revision 항목은 웹이 흐리게 그린다. 조정기가 쓸 때는 출구처럼 고정 때 재기준한다(후속). 판정 규칙과 현장 값은 `src/rescue_perception/README.md` 「지도의 머리 위 구조물」 절에 있다.

## C-05. 관측·사건·보고의 논리 스키마

실제 ROS 정의 파일 이름·배열 상한·직렬화 형식은 아래 의미를 보존하며 구현 PR에서 고정한다.

| 객체 | 필수 내용 | 의미/거절 조건 |
| --- | --- | --- |
| `TargetObservation` | 공통 envelope, observation_id, source_robot/camera, observed_at·stamp_semantics·clock 정보, class·confidence·tracking_mode·track_id, model_id/hash, sensor frame·bbox, 위치 유효성·sensor 위치/공분산, depth 유효성, map/transform 참조(변환한 경우), evidence_ref | 탐지와 위치 성공을 분리. 위치가 없으면 발견 기록은 가능하지만 자동 접근점 입력으로 사용 불가. class confidence를 위치 정확도나 부상 확률로 해석하지 않음 |
| `TargetEstimate` | target_id, 관련 observation_id 목록, estimate_revision, 원 최근 관측 시각/유효시한, mission 위치·공분산, **관측별** source robot·map_session_id/map_revision·transform_revision·보정 참조 목록, 확인 단계·접근 상태·보고 상태 | A/B 관측을 합쳐도 원본별 provenance를 공통 revision 하나로 덮어쓰지 않음. 두 카메라의 track ID만 같다고 병합하지 않으며 동일성 불확실하면 별개 사건/후보 관계로 보존 |
| `RobotStatus` | robot_id, boot·sequence, 로봇별 pose/frame/stamp/품질, 활성 task/generation, readiness, motion_state, stop 사유/증거, 센서·TF·시간·Nav2·명령 lease 상태, 도킹 상태/관측 시각·준비 action 상태(해당 플랫폼), 배터리 상태 | 상대 pose를 같은 mission frame으로 변환하지 않고 거리 비교 금지. 알 수 없는 값은 UNKNOWN. retained snapshot은 운영 표시용이며 live readiness 증거가 아님 |
| `RescueReport` | report_id, target_id, 관측·확인·접근 결과, 수행 로봇/task/generation, 위치/frame/revision, observed_at·confirmed_at·reported_at, 접근 불가 등 사유, evidence_ref, 스키마·모델·배포 버전 | 사람 이송/부상/생존을 확정하지 않음. 오래된 사진을 새 재확인 사진으로 대체하지 않음 |
| `ReportReceipt` | receipt_id, 참조 report_id, 수신 상태, 영속 저장 시각·저장 revision 또는 실패 코드 | `RECEIVED`와 `PERSISTED`를 구분. 보고 완료는 현재 정책에 필요한 payload와 증거가 영속 저장되고 해당 receipt가 돌아온 상태 |
| `SiteEvent` | event_id, camera_id(`cam_<id>`), kind(`DISASTER_SIGN`/`TARGET_DISAPPEARED_CANDIDATE`/`EXIT_PASS`), target_id·watch_id(소실 후보에만. coordinator 가 감시 설정 때 발급한 canonical UUID 를 카메라가 그대로 되돌린다. 다른 종류는 None), observed_at_ns + C-03 시간 기준(clock_domain_id·stamp_semantics·clock_state·clock_error_bound_ns), evidence_id/sha256(생산자가 발급, operations 는 bytes 를 그 ID 로 저장·해시 검증), confidence(규칙 기반 어댑터는 1.0 = "규칙 충족", 이진; 소비자는 confidence 로 2차 판정하지 않고 측정값은 detail 에서 감사), detail | 고정 카메라의 증거 사건. 로봇 관측(`TargetObservation`)과 별개이며 `source_robot` 에 카메라를 넣지 않는다. 소실 후보는 재확인의 **트리거**일 뿐 완료 증거가 아니다(KPI-06). 후보의 연결·멱등·시각 순서는 [C-11](#c-11). 고정 카메라는 1대로 확정됐다(2026-10-09). 그래서 `EXIT_PASS` 는 만드는 쪽이 없고, 전에 붙이려던 사람 식별 필드도 만들지 않는다. `time_basis_declared` 는 세 시간 메타데이터가 **선언됐다**는 사전 조건일 뿐이다 — True 만으로 다른 관측과의 순서·최신성·구출 완료를 승인할 수 없고, 소비자가 양측 `clock_domain_id` 일치, 송수신 오차 합의 정책 상한, 미래 시각·나이 한도를 따로 검사한다(C-03; `rescue_perception.validate_freshness` 와 같은 규칙). False 면 기록만 되고 판정 입력이 아니다. 실행 가능한 Python 정의는 `rescue_contracts.SiteEvent` |

`tracking_mode={DETECTION_ONLY,TRACKING,TRACKING_FALLBACK}`는 필수이며 관측 payload hash에 포함한다. TRACKING은 비어 있지 않은 track ID가 필요하고 다른 두 모드는 track_id=None이다. tracking fallback을 정상 tracking이나 단순 ID 누락과 구별한다.

`evidence_ref`는 소유 API의 불변 ID·해시·매체 유형·관측 ID·획득 시각이다. 다른 PC의 임의 로컬 파일 경로를 공유 가능한 URL처럼 사용하지 않는다. 사진 원본의 전송/저장은 operations API에서 처리하며 DDS에 대형 사진을 반복 포함하지 않는 안을 제안한다. 사진을 필수로 할지와 보관 범위는 [scope.md의 Q-07](../requirements/scope.md)에 두며 결정 시점은 계획 PR 7·9다. 이 계약은 `PENDING_EVIDENCE` 상태를 정의하고 사진 필수 여부를 결정하지 않는다. 필수 증거 업로드가 실패하면 보고를 `PENDING_EVIDENCE`로 남기며 완료로 표시하지 않는다.

## C-06. 임무 action·lease·정지

`ExecuteTask` action endpoint 제안: `/rescue/robots/{robot_id}/execute_task`. 이 주소는 C-08의 그래프 A 가설에서 직접 연결하며, 그래프 B에서는 도메인별 어댑터/API가 동일 의미를 전달한다.

| 구분 | 내용 |
| --- | --- |
| Goal | command_id, task_id, generation, robot_id, coordinator_id/authority_epoch, kind, issued_at/deadline, command lease 참조, target_id/estimate_revision 또는 search_area_ref, 필요한 frame/session/revision, 정책/설정 버전 |
| kind | `SEARCH_AREA`, `REOBSERVE_TARGET`, `APPROACH_TARGET`, `HOLD`, `MOVE_SEGMENT`, `RETURN_HOME`. 대상 임무(`TARGET_TASK_KINDS`)는 `REOBSERVE_TARGET`·`APPROACH_TARGET`·`MOVE_SEGMENT` 이고 `target_id`·`target_revision` 이 필수다. `MOVE_SEGMENT` 는 운영자 승인 한 건(`OperatorApproval`, 한 구간)에 묶인 AMR2 한 구간 이동이다(Q-12 결정 2026-10-10). 승인과 대상·revision·경로·`segment_index` 가 같아야 하고, 성공은 그 구간 도착·정지이지 접근 완료가 아니다. 다음 구간은 ‘계속’ 승인마다 새 명령·task·generation 으로만 발급하고 자동 발급하지 않는다(규칙 원문은 [SDD](sdd-node-interface-spec-v0.1.md) ExecuteTask·SubmitOperatorCommand). `RETURN_HOME` 은 임무 완료 또는 운영자 취소 뒤 두 로봇(SCOUT·SUPPORT) 각각이 복귀 위치로 가는 새 generation 임무이고(BRD Part I 3.9.1) 대상이 없다. 복귀 위치의 출처는 Q-22 이며, 위치가 없거나 지금 지도와 맞지 않으면 발급하지 않는다. 실행기가 어떤 종류를 받는지는 `rescue_robot` 의 허용 목록이 정한다. 도킹·물품 전달은 기본 범위에 추가하지 않음. 준비용 UNDOCK/DOCK는 아래 별도 경로 |
| 수락 조건 | ID/버전/시각/권한·generation 일치, 필요한 센서·시간·TF·Nav2 준비, stop latch 없음, lease 유효, 필수 설정 확정, 현재 지도 revision 호환. 도킹 장치가 있는 플랫폼의 수색/접근/재관측 이동은 fresh dock_status의 is_docked=false 필요 |
| Feedback | task/generation, 실행 상태, 현재 목표/경로 상태, 마지막 관측 나이, 장애/보류 사유, 실제 pose의 시각/품질. action feedback 수신 자체를 성공으로 보지 않음 |
| Result | 수락한 action의 outcome은 `COMPLETED`, `CANCELED`, `EXPIRED`, `UNREACHABLE`, `FAILED` 중 하나. terminal_reason, stop_confirmed, motion_state, 사용한 관측/좌표 revision, stop 증거, 보고 참조를 포함. 결과가 없으면 UNKNOWN/PENDING 유지 |

도킹 장치가 있는 플랫폼의 준비/운영 `UNDOCK`·`DOCK`는 협업 `ExecuteTask.kind`와 분리한 기동 절차 또는 운영자 요청으로만 실행한다. `rescue_bringup`/operations는 실행기에 요청하고 실제 장치 action 발행은 `rescue_robot` 한 곳이 소유한다. 이 경로도 command ID·권한·deadline·배타적 명령 소유·로컬 정지 경로를 요구한다. 센서가 도킹 중 꺼지는 장비의 준비 교착을 피하도록 UNDOCK는 완전한 Nav2/카메라 READY를 선행 조건으로 삼지 않고, fresh 도킹/driver 상태·실제 정지·장치 action 가용성·유효한 준비 제어권·fault/stop 해제 조건을 따로 검사한다. 이 준비 제어권은 협업 ARMED를 뜻하지 않으며 Nav2와 동시에 움직일 수 없다.

도킹 action의 접수/성공 응답만으로 물리 상태를 확정하지 않는다. UNDOCK 뒤 fresh `is_docked=false`·정지와 필요한 센서/TF/위치 추정 준비를 확인한 뒤 협업 이동을 허용한다. DOCK 뒤에도 fresh 도킹 상태·정지를 확인한다. 접수 뒤 무동작·상태 불명·timeout은 실패/NOT_READY와 S-04 정지 관문으로 처리한다. `power_saver: False`로 도킹 중 센서가 나오더라도 수색/접근/재관측 이동의 `is_docked=false` guard는 생략하지 않는다. 도킹 장치가 없는 플랫폼의 적용 제외는 장비 프로파일과 근거로 명시한다.

Nav2 terminal status `SUCCEEDED`/`CANCELED`/`ABORTED`는 별도 `action_status`로 보존한다. 외부 취소/실패와 실행기가 먼저 요청한 정지 원인을 섞지 않으며 `original_cause`와 canonical `terminal_reason`/qualified `detail_code`를 남긴다. action 성공 뒤에도 정지 증거가 필요하다.

action 수락 전 거절은 goal 응답의 `accepted=false`와 별도의 `TaskAdmission`/상태 기록에 command_id/task_id/robot_id/robot_boot_id/authority_epoch/generation 및 canonical 거절 이유를 남긴다. coordinator의 발급 세대와 실행기의 최고 수락 세대를 구분하며 예약 생성만으로 수락 완료를 기록하지 않는다. 도메인 로그의 `REJECTED`는 action Result enum이 아니다. 정지 확인이 실패한 수락 임무는 `outcome=FAILED`, `terminal_reason=STOP_UNCONFIRMED`, `stop_confirmed=false`, `motion_state=STOP_UNCONFIRMED`로 보고하고 최초 취소/만료/실패 원인도 별도 기록한다. 결과 전송 후에도 정지 latch와 감시를 유지하며 새 이동을 허용하지 않는다.

실행기는 로봇별 활성 이동 task를 하나만 소유한다. 새 임무는 기존 임무 종료·정지 관문을 거친 뒤 시작한다. 동일 command_id의 재전송은 이전 수락/결과를 돌려주며 새 goal을 만들지 않는다. 낮은 generation·이전 boot/authority의 메시지·완료한 goal의 늦은 feedback/result는 현재 상태를 덮어쓰지 않는다.

generation은 coordinator가 로봇별 발급하며 발급 장부와 수락 장부를 분리한다. 예약은 PENDING이고, command_id/task_id/robot_id/robot_boot_id/authority_epoch/generation이 정확히 일치하는 실행기 TaskAdmission의 accepted=true가 유일한 수락 근거다. 실행기는 최고 수락 세대를 검증하고 coordinator는 해당 응답을 받은 뒤에만 ACTIVE와 accepted generation을 갱신한다. 거절/해제된 명령의 지연·충돌·재전송으로 재활성화하지 않는다. 재시작 뒤 발급 이력·실행기 이력·authority를 조정하기 전에는 자율 임무를 재개하지 않는다. 예약의 `candidate`(planner 경로 증거)는 `APPROACH_TARGET`·`REOBSERVE_TARGET` 에만 필수이고 `SEARCH_AREA`·`HOLD` 는 None 이다. Phase 1의 `MOVE_SEGMENT`·`REOBSERVE_TARGET`에 한정한 경로 증거 예외는 C-10을 따른다. 이는 계약상 예외이며, 해당 예외를 구현·검증하기 전의 Python 예약 검사를 우회하라는 뜻이 아니다. `HOLD` 는 제자리 정지 유지 임무이며 프로파일 `hold_positions` 로 이동시키는 명령이 아니다(대기 위치로의 이동은 별도 kind 가 필요하며 미정, Q-14).

coordinator는 단일 인스턴스를 전제로 하고 자동 leader election/failover는 이번 기본 범위에 넣지 않는다. 재기동하면 새 `authority_epoch`가 되며 이전 lease/배정을 복구된 것으로 자동 인정하지 않는다. 로봇 실행기는 정지·기존 권한 폐기 확인 후 명시적인 재등록으로 새 authority를 허용한다. UUID 문자열 크기 비교로 권한의 신구를 판단하지 않는다.

command lease는 로봇별 현재 authority/generation에 묶고 유효기간·최대 갱신 간격·로컬 monotonic 만료를 가진다. 중복 또는 늦은 갱신으로 만료 lease를 부활시키지 않는다. coordinator lease, executor heartbeat, 최종 속도 입력이 각각 만료될 때의 중단 책임을 구분한다. 새 연결이나 heartbeat만으로 정지 latch가 해제되지 않는다. 예외는 하나다. 로봇을 한 번도 arm 하지 않았고 작업이 없는 기동 단계에서는, 실행기 heartbeat 가 늦어 걸린 최종 속도 watchdog 의 잠김(`EXECUTOR_HEARTBEAT_STALE`)이 기동 단계 heartbeat 가 돌아와 `heartbeat_max_age_s` 동안 이어지면 최대 3회 스스로 풀린다(재범 결정 2026-10-16 「B로 가자」, `rescue_robot/watchdog.py`).

`RequestStop` service 제안: `/rescue/robots/{robot_id}/request_stop`; 입력은 stop_command_id, requester/authority, reason, scope, issued_at이다. 응답 `ACCEPTED`는 정지 요청 접수만 뜻한다. 로봇 실행기는 활성 goal 취소와 최종 명령 억제를 요청하고 별도 `RobotStatus`로 `STOPPED` 또는 `STOP_UNCONFIRMED`를 보고한다. 정지 요청은 새 임무·추론 완료·지도 계산보다 우선 처리되어야 한다.

`STOPPED`는 fresh odometry의 속도가 사전 기준 이하로 일정 관측 구간 유지되고 최종 명령 경로가 억제된 것을 확인한 상태를 제안한다. 해당 값·시간·관측 방식은 실기 전 고정하고 실제 주행 시험으로 검증한다. cancel 응답이나 0 속도 한 번 발행은 이 증거를 대신하지 않는다. odometry/driver 상태도 끊기면 `STOP_UNCONFIRMED`다. 이 상태를 물리적 안전 인증으로 표현하지 않는다.

정지 latch의 해제는 유효한 권한의 명시적 `Resume/Reset` 요청, 원인 해결·readiness·새 generation을 모두 요구한다. 비상 정지/장비 fault의 해제 조건은 장비 매뉴얼·실물 확인 후 따로 고정한다. 웹 요청이 실패했다고 정지가 성공했다고 표시하지 않으며, 로봇 현장 정지 수단을 웹 연결에만 의존시키지 않는다.

## C-07. 안전 접근·통로 자원

대상의 위치와 Nav2 접근 목표를 별도 값으로 저장한다. 목표는 대상 보호 영역·위치 불확실성·로봇 footprint·제동/제어 오차를 고려해 자유 공간의 후보에서 선택하고 경로 가능성을 확인한다. 대상 좌표 자체나 로봇-대상 직선 위 한 점만으로 접근 성공을 정하지 않는다. Nav2 `SUCCEEDED` 이후에도 재확인·거리/정지·좌표 유효성 관문을 통과해야 `APPROACH_COMPLETE`다.

`D_MIN`, `D_MAX`, 보호 영역 정의, 위치 오차 상한, 재확인 최대 나이, 후보 검색 범위·재계획/retry budget은 시험 전에 고정한다. 사람/인형 형상·가림 등으로 이 조건을 평가할 수 없으면 `LOCATION_UNCERTAIN`/`NEEDS_REOBSERVATION`으로 정지·보고한다. 가까워지기 위한 후진/회전도 공통 명령·장애물 경로를 통과한다.

HELD 또는 holder를 보존한 OCCUPANCY_UNKNOWN에서 `confirm_vacant(holder=...)`는 저장된 robot/task/generation/authority와 정확히 일치해야 한다. 제3자나 이전 세대는 점유를 해제할 수 없다. holder가 없는 초기 UNKNOWN에서만 holder=None과 fresh verified vacancy를 함께 받을 수 있다.

좁은 통로를 공유 자원으로 채택하면 `resource_id`, holder robot/task/generation, revision, lease, 진입·이탈 확인을 계약화한다. lease 만료는 점유 로봇이 물리적으로 빠져나갔다는 증거가 아니다. holder 위치·이탈이 불확실하면 `OCCUPANCY_UNKNOWN`으로 새 진입을 보류한다. 다른 로봇의 stale pose만으로 통로가 비었다고 판단하지 않는다. 구체적인 자원 모델/경로 교착 정책은 PR 2 선택과 PR 8 구현에서 검증한다.

## C-08. 토픽·QoS·API 제안

**그래프 A 가설:** 아래 전역 토픽/action을 직접 연결하는 안은 두 로봇·네 PC가 단일 ROS 그래프를 공유한다는 전제다. PR 2의 A0/A1 실험에서 domain·Discovery Server 목록/고유 ID·참여자 수·호스트별 수신·명령 분리가 확인되기 전에는 배포 확정이 아니다. 원본 P4의 DEC-010도 A는 제안이고 B는 도메인 분리 대안이었다. 원본 프로젝트의 B 기본값 결정을 새 프로젝트의 사용자 확정 결정으로 승계하지 않는다.

**그래프 B 대체 경로:** 로봇별 domain 안에 독립 수신/실행 프로세스를 두고, coordination/operations는 도메인별 어댑터와 API로 관측·상태·임무 요청·정지·보고를 중계한다. Nav2/도킹 action은 해당 로봇 domain의 실행기가 호출한다. 범용 domain bridge가 action까지 투명하게 중계한다고 가정하지 않는다. API여도 원 source/관측 시각·ID·generation·좌표/session/revision·권한·deadline·정지 증거·영속 receipt 의미를 보존하고, 중계 단절도 lease/watchdog 대상이다. 한 프로세스의 다중 ROS context는 기본 구현으로 확정하지 않는다. 두 안 중 선택과 실제 endpoint/라우팅은 프로파일로 고정한다.

아래 depth 값은 큐 설정 제안이며 지연·누락·재시작 시험으로 고정한다. DDS reliability/durability를 DB 저장·명령 완료 보장으로 해석하지 않는다.

| 데이터 | 토픽/endpoint 제안 | QoS/전달 방식 제안 |
| --- | --- | --- |
| 로봇 native RGB/Depth/scan/odom | `/{robot_id}/...`의 상대 sensor topic | 실제 드라이버 QoS와 호환. 영상/센서는 BEST_EFFORT, KEEP_LAST 1 우선 검토; 처리 지연 시 최신 입력 유지. 정합·누락은 별도 계측 |
| 관측 | `/rescue/observations` | BEST_EFFORT, KEEP_LAST 10, VOLATILE; 관측 중복 제거·신선도 검증. 중요한 사건 확인/보고는 아래 reliable 경로 사용 |
| live heartbeat/readiness | `/rescue/robots/{robot_id}/heartbeat` | RELIABLE, KEEP_LAST 1, VOLATILE; 어플리케이션 watchdog. DDS deadline 이벤트만으로 물리 정지를 주장하지 않음 |
| 상태 snapshot | `/rescue/robots/{robot_id}/status`, `/rescue/targets` | RELIABLE, KEEP_LAST 1, TRANSIENT_LOCAL; envelope/stamp로 오래된 snapshot 식별. targets는 전체 snapshot인지 delta인지 혼합하지 않고 전체 snapshot을 제안 |
| 좌표 정렬 상태 | `/rescue/frame_alignments` | RELIABLE, KEEP_LAST 1, TRANSIENT_LOCAL; 현재 전체 alignment snapshot과 revision/validity. TF는 표준 전송을 사용하며 frame ID와 authority를 검사 |
| 임무·정지 | ExecuteTask action, RequestStop service | RELIABLE, VOLATILE 제안. 재접속 후 오래된 명령을 retained 값으로 재실행하지 않음. timeout/retry/idempotency는 응용 계약 |
| 보고 | `/rescue/reports`, `/rescue/report_receipts` 또는 동등 API | RELIABLE, VOLATILE + 송신측 보류 저장/동일 ID 재전송 + 수신측 중복 제거/영속 receipt. endpoint 최종 선택은 PR 2의 operations API와 일치시킴 |
| 지도/영상 관제 | operations API의 snapshot/stream | frame/session/revision·생성/관측 시각·유효 상태 포함. 지도·메타데이터를 같은 snapshot ID로 전달. PC간 DB 파일 공유 대신 API 책임 분리 |

Discovery 진단에는 `ROS_SUPER_CLIENT` 설정 여부·서버 목록/ID·프로세스 및 DDS participant 수·서버 CPU를 기록한다. 목록 조회용 설정과 일반 runtime의 필수 조건을 혼동하지 않으며, 토픽 목록만 보이는 것을 통신 PASS로 쓰지 않는다. 실제 메시지 수신 시각·양방향 요청/응답과 장애 복구를 시험한다.

관측은 누락될 수 있는 센서 흐름이다. 첫 발견·확인된 사건·보고의 보존은 별도 reliable/영속 경로에서 처리한다. 전송 보장과 탐지 성공률을 혼동하지 않는다. HTTP를 선택하는 경우에도 동일 command/report ID와 상태 단계를 유지하며 UI는 요청 중·수락·실행·저장·실패를 구분한다.

## C-09. 오류 코드와 복구 원칙

공통 Python 오류의 단일 사전은 [rescue_contracts](../../src/rescue_contracts/rescue_contracts/__init__.py)의 `CANONICAL_CODES`와 `DETAIL_CATEGORIES`다. 모든 도메인 예외는 `RescueError`를 상속하고 `.code`에는 공통 코드, `.component`에는 발생 패키지, `.detail_code`에는 `rescue_mapping.TF_TIME_MISMATCH` 같은 qualified 세부 원인, `.detail`에는 설명을 둔다. 소비자는 세부 문자열 대신 `.code`로 복구 분기한다. 예외를 발생시키지 않는 실패 결과도 같은 canonical code/detail_code 의미를 노출한다. 성공/상태 enum (`ACCEPTED`, `NAVIGATION_SUCCEEDED`, `HOLD`, `VALID`, `NEEDS_REOBSERVATION` 등)을 오류 코드와 혼합하지 않는다.

공통 코드: `INVALID_INPUT`, `INCOMPATIBLE_SCHEMA`, `WRONG_ROBOT`, `STALE_OBSERVATION`, `CLOCK_UNCERTAIN`, `STALE_TRANSFORM`, `FRAME_REVISION_MISMATCH`, `LOCALIZATION_UNCERTAIN`, `SENSOR_UNAVAILABLE`, `NAV_NOT_READY`, `NOT_AUTHORIZED`, `STALE_GENERATION`, `LEASE_EXPIRED`, `STOP_LATCHED`, `UNREACHABLE`, `RETRY_EXHAUSTED`, `REPORT_NOT_PERSISTED`, `STOP_UNCONFIRMED`, `ID_CONTENT_CONFLICT`, `TASK_BUSY`, `DEADLINE_EXPIRED`, `ACCEPT_TIMEOUT`, `GUARD_INVALID`, `STOP_EVIDENCE_STALE`, `SESSION_FAULT`, `GOAL_REJECTED`, `ACTION_CANCELED`, `ACTION_ABORTED`, `ROBOT_DOCKED`, `DRIVER_UNAVAILABLE`, `COMMAND_WATCHDOG_UNAVAILABLE`, `POLICY_MISMATCH`, `NOT_FOUND`, `PENDING_EVIDENCE`, `PASSAGE_BLOCKED`(TB-5 가 통로 앞 깊이 정지 영역 때문에 멈춤·그 뒤 Nav2 포기, 2026-10-13 OPS-128).

`NEEDS_REOBSERVATION`가 예외의 세부 원인일 때 `.code=FRAME_REVISION_MISMATCH`이며 상태 표기의 이름은 그대로 유지한다. 알려지지 않은 진단 코드를 사전에 등록하지 않고 성공으로 처리하지 않는다. E2 코어는 객체·시각·task 정보가 담긴 호출 입력/반환값을 제공하며, 전체 런타임 로그 envelope와 재시도 정책은 ROS/API 어댑터에서 결합해야 한다.

모든 오류에는 원인 컴포넌트·객체 ID·발생 시각·활성 task/generation·재시도 가능 여부를 기록한다. 원인을 모르면 UNKNOWN을 유지하고 성공으로 추정하지 않는다. sensor/time/frame 실패와 임무 정책 실패를 같은 무한 재시도로 숨기지 않는다. 재시도 budget·전체 deadline·정지 사유는 상태 전이와 함께 시험한다.

## C-10. ROS 인터페이스 1차 채택 (2026-10-10)

[SDD 검토안 v0.1](sdd-node-interface-spec-v0.1.md)의 필드 정의([JSON](sdd-node-contracts-v0.1.json))에서 아래만 `src/rescue_interfaces` 의 ROS IDL 로 채택한다. 사용자 지시 원문: "ㅇㅇ 그렇게 진행해줘"(2026-10-10, 이 세션이 제안한 1차 목록·IDL 규칙에 대한 답). 적용 범위는 IDL 정의이며 토픽 이름·QoS·노드 배치·Q-10·Q-12·Q-15·Q-22·Q-25 의 결정이 아니다.

| 구분 | 1차 채택 |
| --- | --- |
| msg (25) | Envelope, ObservationTime, FrameRef, EvidenceRef, RobotStatus, RobotPose, Heartbeat, CommandLease, TaskAdmission, TaskResult, OperatorApproval, RouteCandidate, SiteEvent, TargetObservation, TargetLocation, MapState, ReportReceipt, RecheckObservation, PassageAssessment, TargetPresence, CameraStatus, SystemState·RescueReport·TargetSnapshot·TargetEstimate(2026-10-11 추가) |
| srv (5) | RequestStop, RegisterAuthority, ResetStop, SubmitOperatorCommand, SetCameraWatch |
| action (2) | ExecuteTask, ObserveTarget |

`RecheckObservation`·`PassageAssessment` 는 `TaskResult`·`RouteCandidate`·`ObserveTarget` 이 품고 있어 같이 들어왔다. 구조만 있고 통로 판정의 채택(Q-25)은 아니다.

`TargetPresence`(IF-077)·`CameraStatus`(IF-075)·`SetCameraWatch`(IF-078)는 2026-10-10 에 2차에서 올렸다. 카메라 1대 결정(2026-10-09) 뒤 재관측 트리거는 고정 웹캠의 소실 후보이고, 후보를 만들려면 coordinator 가 카메라에 감시(watch)를 주는 `SetCameraWatch` 가 있어야 한다. 두 메시지는 감시 상태와 영상 상태를 알리는 쪽이다. 이 채택은 [제안]이며 저장소 소유자의 머지로 확정한다. 카메라 쪽 노드와 변환기는 아직 없다.

IDL 규칙:
- 필드 이름·순서·타입은 JSON 이 원본이다. JSON 을 먼저 고치고, `tests/test_interfaces_schema.py` 가 IDL 과 같은지 ROS 없이 검사한다.
- UUID 는 `string`, 시각은 `int64 *_ns`(`builtin_interfaces/Time` 아님, C-03), 없을 수 있는 값은 `bool has_*`/`*_known` + 값.
- JSON 의 `string(A/B)` 열거는 `string` 필드 + 같은 파일의 상수 `<FIELD>_<VALUE>="<VALUE>"` 다. 상수 선언 자체는 필드 값의 검증을 강제하지 않는다. UUID·열거값·시각·선택 필드의 유효성은 각 ROS 경계 모듈이 검사한다.
- SDD 의 IDL 과 현재 `rescue_contracts`·장부 DTO 는 1:1 변환 계약이 아니다. `MOVE_SEGMENT`·`RETURN_HOME` 은 Python `TASK_KINDS` 에 들어갔고, 실행기가 받는지는 `rescue_robot` 의 허용 목록이 정한다(C-06 kind 행). `SiteEvent` 의 소실 후보는 #48의 계약 반영이 선행되어야 한다. `EvidenceRef`·`ReportReceipt` 도 기존 장부와 필드 이름·구조가 달라 명시적 변환이 필요하다. 이 PR은 변환기나 실행 노드를 추가하지 않으며, 계약 정합·변환 검증 전에는 새 어휘의 실행 연결을 완료로 보지 않는다.
- Heartbeat(2026-10-10 작전 결정, OPS-17 ① 재확인): Phase 1 의 로봇 PC 안 구간(실행기 ↔ 명령 watchdog, #35)은 `rescue_interfaces/Heartbeat` 대신 `std_msgs/Header` 와 launch 파라미터의 토픽 이름으로 한다. SDD IF-012·013 의 중앙 구간은 구현하지 않았다. 토픽 이름은 SDD 이름(`/rescue/robots/{r}/heartbeat`·`…/watchdog_heartbeat`)과 겹치지 않게 둔다(같은 이름에 타입이 다르면 오류 없이 수신이 0 이 된다). `Heartbeat` 로 옮기는 때: coordination·operations 가 heartbeat 로 생존을 판정하는 코드가 생길 때, watchdog 을 로봇에 둘 때, 실행기 fence 를 watchdog 출력 억제로 전달할 때. 옮길 때는 안쪽·중앙 구간을 한 PR 에서 함께 옮긴다.
- AMR1 출동(2026-10-11 저장소 소유자 결정, #85): `SubmitOperatorCommand.operation` 에 `START_SEARCH` 를 더했다(필드 없음, JSON 먼저). 재난 모드에서 한 번만 받고, 수락하면 조정기가 첫 `SEARCH_AREA` 를 발급해 `created_task_id` 에 싣는다. 이미 시작했으면 `TASK_BUSY`, 재난 모드가 아니면 `NOT_AUTHORIZED` 다. `robot_id` 는 비우거나 SCOUT 로봇이어야 하고, 다른 로봇이면 `INVALID_INPUT` 이다. SCOUT 의 상태를 모르거나 준비(readiness READY·lease 가능)가 안 됐으면 받아 두었다가 나중에 출발시키지 않고 `NAV_NOT_READY` 로 거절한다. 이때 시작한 것으로 치지 않으므로 운영자가 준비 뒤 다시 누른다. 사람이 누른 뒤 알 수 없는 시점에 로봇이 출발하지 않게 하려는 것이다(AGENTS 실패 처리: 불확실하면 실행하지 않음). 재난 이벤트(`DISASTER_SIGN`)는 재난 모드 표시·저장까지이고 주행 명령이 아니다.
- 임무 조정기 얇은 연결(2026-10-11 작전 결정, 저장소 소유자의 조정기 구현 지시 "ㄱㄱ"): 10-13 얇은 전 구간에 필요한 `SystemState`(IF-071)·`RescueReport`(IF-072)·`TargetSnapshot`(IF-070)과 그 안의 `TargetEstimate` 를 1차로 올린다. `RescueReport` 가 있어야 operations 의 `ReportReceipt(PERSISTED)` 가 생기고, 그것이 AMR2 출동 관문(확정 + 저장 확인)을 연다.
  - `SystemState` 는 SDD 필드 뒤에 임무 화면용 필드를 더했다(JSON 을 먼저 고침): `phase`(console-and-db-plan 1절·site_state 1.1 `MISSION_PHASES` 14개), 트리거(`has_trigger`·`trigger_event_id`·`trigger_observed_at_ns`), 출동 구간(`has_dispatch`·`target_id`·`target_revision`·`route_revision`·`segment_index`·`segment_count`·`segment_phase`). Phase 1 은 AMR2 출동 대상이 한 번에 하나라 출동 구간을 이 snapshot 에 둔다. `segment_phase` 어휘는 `TargetEstimate.segment_phase` 와 같다. 조정기 내부 값(APPROVED·MOVING·AWAITING_APPROVAL·ARRIVED)은 노드 경계에서 EXECUTING·EXECUTING·AWAITING_APPROVAL·STOPPED 로 바꾼다. `mode` 는 SDD 대로 NORMAL/DISASTER/MISSION_FINISHED 이고, phase FINISHED 일 때 MISSION_FINISHED 다. 출동 대상의 접근이 막히면(`TargetEstimate.approach_state` BLOCKED·UNREACHABLE, 또는 통로 판정 필요로 보류) `phase` 는 마지막 단계를 그대로 두고 `reason` 에 사유 코드를 싣는다. 화면은 `TargetSnapshot` 의 approach_state 로 막힘을 표시한다(PM 검토 CLA-02, 2026-10-11). `DISASTER → SEARCHING` 전이는 운영자 `START_SEARCH`(관제 화면 버튼)로만 한다. 재난 이벤트는 재난 모드 표시·저장까지이고 주행 명령이 아니다(저장소 소유자 결정 2026-10-11, #85·#76 7번, Q-12 의 AMR1 자동 출동을 바꿈). `START_SEARCH` 1차 IDL은 #98, coordinator 명령 처리는 후속 구현으로 main에 반영됐다(#99). 재난 이벤트만으로 자동 출동하지 않으며 관제 버튼 요청을 기다린다(CLA-03). 현재 콘솔 버튼은 비활성 placeholder이고 operations POST·ROS 왕복은 Q-28 후속이다.
  - 경로 증거(Phase 1): `EvaluateRoute`(IF-022) 가 2차라 planner 경로 증거가 없다. 그래서 `MOVE_SEGMENT` 는 운영자 `OperatorApproval(scope=ONE_SEGMENT)` 이 있을 때만, `REOBSERVE_TARGET` 은 정책의 고정 관측 지점이 있을 때만, `APPROACH_TARGET` 은 조정기가 계산한 접근 지점(대상 위치에서 `approach_standoff_m` 만큼 TB-5 의 지금 위치 쪽, yaw 는 대상을 바라봄; 대상 frame_ref 와 TB-5 위치가 조정기 지도 기준과 같고 VALID 일 때만, 운영자 승인 없음 `has_approval=false`, 저장소 소유자 결정 2026-10-13 1B·2B)이 있을 때만 경로 증거 없이 예약한다. `ExecuteTask.route` 에는 `approach_goal`(구간 끝·관측 지점·접근 지점)만 싣고 `planned_path` 는 비우며, `planner_evidence_id`·`path_sha256` 은 빈 값이다. planner 가 낸 것처럼 보이는 값(통과 가능·도달 가능 등)은 만들지 않는다(AGENTS 실패 처리: 탐지값을 지어내지 않는다). 실제 경로 검증은 실행기 guard·Nav2 가 하고, 실패는 `TaskResult` FAILED 로 드러난다. `EvaluateRoute` 가 1차로 오면 이 예외를 없앤다.
  - **예외의 안전 경계(AST88-01):** 생략할 수 있는 것은 예약 시점의 planner 경로 증거뿐이다. C-04의 지도 기준, C-06의 권한·lease·readiness·정지, C-07의 대상 보호 영역·통로 조건은 그대로다. 머리 위 기둥 등 통로 판정이 필요한 구간은 이 예외로 예약·실행하지 않는다. Q-25 결정 및 필요한 통로 판정이 갖춰지기 전에는 접근 불가로 보고한다. 통로 판정이 필요한지 확인할 수 없는 구간도 보류한다. 목적지만 통로 밖이라는 이유로 허용하지 않는다. 실행기는 실제 주행 경로가 해당 통로에 들어가지 않음을 확인해야 하며, 경로 변경·재계획 뒤에도 같은 조건을 적용한다. 이를 확인할 수 없으면 새 이동을 발행하지 않고, 이미 이동 중이면 취소 → 실제 정지 관문을 따른다. `assessment=UNKNOWN`은 통과 허가가 아니며 Nav2의 2D 경로 성공이나 ONE_SEGMENT 승인이 깊이·상부 여유 판정을 대신하지 않는다.
  - 경로 증거 없는 `route` 의 채우는 법(로봇·지도 제안 반영): `route.frame_ref` 는 필수이고 그 대상의 현재 지도 기준(map_session_id·map_revision·transform_revision, alignment VALID)과 같아야 하며, `approach_goal.header.frame_id == frame_ref.mission_frame` 이다. `route.assessment` 는 `status=UNKNOWN`, `width_known`·`height_known`·`floor_geometry_known` 모두 false 로 보낸다(빈 status 는 어휘 밖). `MOVE_SEGMENT` 의 승인 결속은 `approval` 의 route_id·route_revision·segment_index·robot_id·robot_boot_id·authority_epoch·valid_until_ns 가 goal·route 와 같을 때만 유효하다. path_sha256 이 비어 있으므로 `approach_goal` 이 바뀌면 route_revision 을 올리고, 이전 승인은 쓰지 않는다. `REOBSERVE_TARGET` 은 운영자 승인 대상이 아니라 `has_approval=false` 다. 운영자 `REQUEST_RECHECK` 는 명령 기록(C-11 10행)이지 `OperatorApproval` 이 아니다. `route.generated_at_ns` 는 조정기가 그 경로를 만든 시각(대상이 처음 생길 때 경로를 기록한 시각)이고, 같은 경로를 다시 보낼 때도 보내는 시각으로 새로 찍지 않는다(#100).
  - 명령 lease(Phase 1, IF-014·046 `CommandLease`): 조정기는 task 마다 `lease_id` 를 만들어 `ExecuteTask.lease_id` 에 싣고, 그 task 가 끝나거나 재난 모드 해제·운영자 정지·로봇 상태 낡음·authority/generation 변경이 있을 때까지 `CommandLease` 를 주기 발행한다(같은 lease_id·robot_boot_id·authority_epoch·generation, `renewal_sequence` 증가). 실행기 ROS 입력은 goal 수락 때 `lease_id → command_id` 를 기록한다. 갱신은 아래 수신 적격성 검사와 활성 task의 robot_id·lease_id·robot_boot_id·coordinator_id·authority_epoch·generation 일치 검사를 모두 통과한 뒤에만 `Executor.renew(command_id, authority_epoch, generation, renewal_sequence)` 로 전달한다. 불일치·거절 이유는 로그에 남기며 기존 기한을 연장하지 않는다. **lease 길이는 실행기 정책 파라미터**(`lease_duration`, 로봇 쪽, 기본값 없음)다. 갱신이 끊긴 뒤의 정지 감시는 로봇 자신의 steady 시계를 쓴다. C-03이 금지하는 것은 다른 PC의 monotonic 값 비교이며, 검증된 system-time 기준의 발급 나이·수신 만료 검사는 생략하지 않는다. 조정기는 같은 `lease_duration` 값을 파라미터로 받아 갱신 주기가 그 절반 이하인지 기동 때 검사하고, 아니면 기동을 거부한다. 두 값의 출처를 하나로 묶는 것(배포 프로파일)은 후속이다. lease 는 task 에 딸리므로 작업이 없는 로봇의 `RobotStatus.lease_valid` 는 false 다. 조정기는 새 task 를 낼지 정할 때 `lease_valid` 를 요구하지 않고(readiness READY·상태 신선도만), 실행 중인 task 의 정지 감시에만 쓴다(#100).
  - **lease 수신 적격성(AST88-02):** `CommandLease.issued_at_ns`는 원 발급 시각이고, `valid_until_ns`는 그 갱신 메시지를 수락할 수 있는 절대 만료 시각이다(관제·로그 전용 값이 아님). 둘 다 양의 정수 ns이며 `issued_at_ns < valid_until_ns`여야 한다. ROS 입력은 송수신 호스트의 선언된 같은 clock domain, SYNCED 상태, 측정된 오차 상한과 정책상 오차 상한을 확인한다(C-03). 이 시간 기준의 근거가 없거나 clock step/restart가 있으면 갱신하지 않는다. 수신 wall 시각을 `now_ns`, 송수신 오차 합을 `error_ns`라 할 때 `issued_at_ns <= now_ns + error_ns`, `now_ns - issued_at_ns + error_ns <= lease_max_age_ns`, `now_ns + error_ns < valid_until_ns`를 모두 요구한다. `lease_max_age_ns`와 허용 오차 상한은 시험 전에 정한 정책 값이며 기본값은 없다. 미설정·미확인·미래 허용치 초과·낡음·만료 메시지는 로그에 이유를 남기고 버린다. 이미 끝난 로컬 steady lease, STOPPING, 이전/중복 sequence도 기존 renew 검사대로 거절한다. 수신 적격성을 통과한 새 갱신만 로컬 steady 기한을 정책 `lease_duration`만큼 갱신하며, 다른 PC의 절대 시각을 로컬 monotonic 값으로 직접 비교하지 않는다. 이 관문은 ROS 입력 경계가 구현해야 하며, 시각 인자가 없는 현재 `Executor.renew` 호출만으로 구현된 것으로 보지 않는다.
  - 후속 구현 필수 시험(AST88-01·02): 통로 판정 필요/불명 구간, 통로 밖 목적지로 가는 경로의 통로 진입, 재계획 후 통로 진입을 거절·정지시키는지 확인한다. lease는 정상 새 메시지, 증가 sequence라도 만료·낡은 메시지, 오차로 만료 경계를 넘는 메시지, 시간 기준 불명, 발행 중단 뒤 지연 큐, 로컬 만료 뒤 갱신을 확인한다. 거절 메시지는 lease 기한을 바꾸면 안 된다. 이 문서 변경은 해당 실행 시험의 통과 증거가 아니다.
- 문자열·배열 길이 상한은 두지 않았다 [제안]. 실측 뒤 정한다.
- 고정 카메라 감시(`SetCameraWatch`·`TargetPresence`·`CameraStatus`, 2026-10-10):
  - 재관측 트리거는 `SiteEvent(TARGET_DISAPPEARED_CANDIDATE)` 하나뿐이다. `TargetPresence` 의 소실 후보 상태(`ABSENT_CANDIDATE`)는 표시용이고 재관측이나 판정의 입력이 아니다. 같은 뜻의 주인을 둘 두지 않는다.
  - 경계 검사: ROI 는 `0≤min<max≤1` 이고, UUID 는 형식을 검사하며, 빈 열거값은 거절한다. 해제(`enable=false`)는 `watch_id`·`expected_watch_revision` 이 활성 감시와 같고 ROI 도 활성 감시 값과 같을 때만 받는다. 기본 생성 값(빈 문자열·0)을 설정 요청으로 받지 않는다.
  - 낡은 값: 값을 받은 적이 없거나, `valid_until_ns` 가 지났거나, `watch_id`·`watch_revision`·`publisher_boot_id` 가 기록한 활성 감시(수락 응답을 준 boot)와 다르면 UNKNOWN 이다. latch 된(TRANSIENT_LOCAL) `TargetPresence` 와 boot 가 바뀐 응답은 성공으로 보지 않는다.
  - `camera_id` 하나에 `CameraStatus` 를 내는 노드는 하나(`cam.capture`)다. `CameraStatus.stream_state` 가 ONLINE 이 아니거나 `TargetPresence.visibility_state` 가 VALID 가 아니면 후보를 만들지 않는다. CameraStatus 자체에는 visibility_state 필드가 없다.
  - Python DTO 는 아직 없다. 첫 소비자 PR 에서 `rescue_contracts` 에 넣는다. IDL 생성·import 를 변환 성공으로 보지 않는다.
  - `watch_id`·`watch_revision` 의 해석과 `SET_CAMERA_WATCH` → `SetCameraWatch` 변환 규칙은 SDD JSON 의 Rules 가 원본이다. SDD JSON·명세에 이 규칙을 기록했다 [제안, 2026-10-11]. `watch_id` 는 (camera, target, ROI) 에 고정한다. `watch_revision` 은 카메라당 감시 칸의 교체 번호이고 0 은 비어 있음이다. 변환은 프로파일에서 `TARGET_WATCH` 카메라가 정확히 1대일 때만 한다.
- 호환되지 않는 변경은 `Envelope.schema_version` 을 올린다. 버전 문자열 형식은 미정이다.
- `SiteEvent`(2026-10-10 작전 결정): IDL 의 `visibility_state` 에 대응하는 Python 필드는 두지 않는다. SDD 확장표(394행)의 "visibility 필드 추가" 와 다른 의도적 차이다. `SiteEvent` 는 시야가 유효(VALID)한 판정에서만 만든다(BRD v0.2 3.5.2·3.5.3, SRD 6.1). 그래서 Python → ROS 경계는 `VISIBILITY_STATE_VALID` 만 쓰고, ROS → Python 경계는 VALID 가 아닌 값(빈 문자열 포함)을 `INVALID_INPUT` 으로 거절한다. 거절한 사건은 판정·출동·재관측에 넣지 않고 이유를 로그에 남긴다. 판단 보류(UNKNOWN)와 그 이유(지진 판정의 OCCLUDED·MATCH_FAILED·SCENE_CHANGED·QUADRANT_COVERED)는 `SiteEvent` 로 보내지 않는다. 대상 가림은 1차 `TargetPresence`(cam.presence), 영상 끊김은 1차 `CameraStatus`(cam.capture)가 맡는다. `target_id`·`watch_id` 는 IDL 에서 DISASTER_SIGN 일 때 빈 문자열이고 Python 에서는 None 이다. 소실 후보인데 비었거나 UUID 가 아니면 경계 모듈이 거절한다.

2차(미채택): FrameAlignmentSnapshot·FrameAlignment, SearchCandidates, CommandPermit, PrepareMapping, EvaluateRoute, EvaluatePassage.

<a id="c-11"></a>

## C-11. 소실 후보 → 재관측 → 구출 판정 연결 (카메라 1대, 2026-10-10)

카메라 1대 결정(2026-10-09)으로 출구 카운트가 빠졌다. 그 자리를 고정 웹캠의 대상 소실 후보(`SiteEvent(kind=TARGET_DISAPPEARED_CANDIDATE)`)가 맡는다. 카메라는 감시 중인 인형이 화면의 출구 쪽 영역에서 사라졌을 때만 후보를 낸다. 다른 곳에서 사라지면 `TargetPresence` UNKNOWN(`NOT_VIA_EXIT`)을 내고 후보를 내지 않는다(scope Q-26 추가 항목, #135). IDL 은 바뀌지 않는다. 웹캠이 판정 불가(UNKNOWN)여서 후보를 낼 수 없을 때는 운영자가 영상을 보고 낸 재확인 지시(`REQUEST_RECHECK`)가 후보 자리를 맡는다(scope Q-26, 2026-10-11). 아래 10·11행이 그 규칙이다. 아래는 후보를 받아 SCOUT 재관측을 예약하고 구출 완료를 판정할 때 무엇으로 무엇을 잇는지의 계약이다. 판정 조건 자체는 [BRD v0.2](../requirements/brd-v0.2.md) 3.5.5(카메라 ABSENT·LiDAR CLEAR·관측 범위 VALID)를 따른다. 정책 수치는 프로파일 값이고 여기에 쓰지 않는다. 후보를 받을지, 재관측을 어느 후보에 붙일지는 coordination 이 한 곳에서 정한다.

| # | 항목 | 규칙 | 어기면 |
| --- | --- | --- | --- |
| 1 | `target_id` | coordination 의 발견 사건 UUID 이며, coordination 코드의 discovery `event_id` 와 같은 값이다(이름은 바꾸지 않는다). SET_CAMERA_WATCH, SetCameraWatch, 소실 후보, REOBSERVE_TARGET 의 goal, RecheckObservation, 판정이 모두 같은 값을 쓴다 | `INVALID_INPUT` |
| 2 | `watch_id` | coordination 이 발급한다. 입력은 운영자 `SET_CAMERA_WATCH`(SubmitOperatorCommand, 1차)다. SetCameraWatch 요청을 보낼 때 기록하고, 카메라가 거절하면 지운다. (camera_id, target_id, ROI) 는 watch_id 에 고정이고, 바꾸려면 새 watch_id 를 쓴다. `watch_revision` 은 카메라 슬롯의 교체·해제 횟수일 뿐 같은 watch_id 를 다시 묶는 데 쓰지 않는다. 카메라당 ACTIVE watch 는 1개이고, 프로파일 `TARGET_WATCH` 역할 카메라에만 무장한다. watch 가 끝나는 때: 교체·해제, 대상 판정 종료, 후보 1건을 받거나 거절했을 때, 카메라 어댑터 재시작 | 같은 watch_id 에 다른 결합 → `ID_CONTENT_CONFLICT`(C-09 예외) |
| 3 | 후보 `event_id` | 카메라 어댑터가 발급한다. 재전송은 같은 ID·같은 내용이다(C-02). 생산자는 watch 하나에 후보 하나만 낸다. event_id 검사(멱등·충돌)를 watch 상태 검사보다 먼저 한다 | 같은 ID·같은 내용 → 멱등(새 임무 없음). 같은 ID·다른 내용, 또는 같은 watch_id 에 다른 event_id → `ID_CONTENT_CONFLICT` |
| 4 | 후보 수용 | 조정기가 DISASTER 모드일 때만 받는다(아니면 기록만, #124). 기록된 watch 가 있고, `camera_id`·`target_id` 가 그 watch 와 같고, `time_basis_declared` 가 참이다. `stamp_semantics` 는 CAPTURE 나 DRIVER_RECEIVE 를 실제 값대로 쓴다 | `CANDIDATE_NOT_DISASTER` → 기록만 하고 출동하지 않는다. `CANDIDATE_NOT_LINKED`·`CANDIDATE_TIME_BASIS_UNUSABLE` → 출동하지 않는다. 그 watch 를 끝내고 "재무장 필요" 를 관제에 드러낸다 |
| 5 | 재관측 `task_id` | 받은 후보마다 SCOUT 의 REOBSERVE_TARGET 을 자동 예약하고, `후보 event_id → {task_id}` 를 coordination 이 기록한다. 열린 재관측은 후보당 1개다. 결과가 COMPLETED 가 아니면 같은 후보로 새 task_id·generation 을 다시 예약한다. COMPLETED 뒤에는 운영자 `REQUEST_RECHECK`(SCOUT 지정)만 받고, 그 대상의 가장 최근 수용 후보에 연결한다. 후보 없이 들어온 재관측과 SUPPORT 의 재관측은 기록만 하고 판정 근거가 아니다. 접근 완료와 출구 카운트는 조건이 아니다 | 기록에 없는 task 의 재관측 → 7행 `CANDIDATE_NOT_LINKED` |
| 6 | `target_revision` | 연결 키가 아니다. REOBSERVE goal 의 `target_estimate_revision` 과 같은 값이다(C-06) | — |
| 7 | 판정 연결 | 판정은 coordination 기록에서 그 대상의 후보와 연결된 task_id 들을 받는다. 재관측은 task_id 별로 보관하고, 연결된 task 의 재관측만 판정에 쓴다. 연결되지 않은 재관측은 판정 근거의 버전을 올리지 않는다. 확정 조건: 그 후보, `candidate.target_id == target_id`, 재관측 task 가 연결 목록에 있음, 로봇이 SCOUT, frame_ref 의 map session·revision 이 맞고 정렬 VALID, 카메라 ABSENT·LiDAR CLEAR·관측 범위 VALID. 판정 기록에는 쓴 후보 event_id 와 재관측 observation_id 를 남긴다 | `CANDIDATE_NOT_LINKED` 등 → `RECHECK_NEEDED` |
| 8 | 시각 순서 (C-03) | 비교하려면 후보와 재관측 모두 시간 기준이 선언돼 있고(UNKNOWN 아님, SYNCED, 오차 있음), `clock_domain_id` 가 같고, 오차 합이 정책 상한 이하여야 한다. 같은 `clock_domain_id` 는 설정만으로 주지 않는다. 같은 chrony 기준에 동기화했고, 그 호스트의 SYNCED 상태와 오차 상한을 측정해 기록했을 때만 같은 값을 쓴다 [제안, #76 CDX-76-03]. 측정 전에는 시간 기준을 UNKNOWN 또는 오차 None 으로 둔다. 순서: `sensor_window_start_ns − candidate.observed_at_ns > 오차 합`. LiDAR 는 같은 관측 창 안이라 따로 검사하지 않는다. watch_id·task_id 의 인과 순서로 이 검사를 대신하지 않는다 | 비교할 수 없음 → `CANDIDATE_TIME_BASIS_UNUSABLE`. 순서 위반 → `ORDER_VIOLATION` |
| 9 | IDL ↔ Python | `SiteEvent` 는 C-10 의 규칙을 따른다. RecheckObservation + TaskResult → 판정 입력: `camera_state` PRESENT/ABSENT/UNKNOWN → 사람 있음 True/False/None, `lidar_state` BLOCKED/CLEAR/UNKNOWN → 장애물 True/False/None, `coverage_state` VALID/INVALID/UNKNOWN → 시야·깊이·스캔 범위가 모두 True/False/None. LiDAR 시각·clock 은 `RecheckObservation.time` 이다. 시간 기준이 UNKNOWN 이거나 오차가 없으면 오차를 None 으로 둔다 | 변환 실패 → `INVALID_INPUT`, 로그. 대체값을 쓰지 않는다 |
| 10 | 운영자 재확인 지시: 경로 선택·연결·응답 (Q-26) | `SubmitOperatorCommand(operation=REQUEST_RECHECK)` 는 `target_id`(1행과 같은 값)가 있어야 한다. coordination 은 이 순서로 검사한다. ① `command_id` 먼저: 같은 ID·같은 내용이면 `DUPLICATE` 로 처음 응답의 `accepted`·`created_task_id` 를 그대로 돌려준다(그 뒤 대상이 완료됐어도 같다). ② 대상: 기록에 없거나 이미 판정이 끝난 대상이면 거절. ③ 열린 재관측: 그 대상에 어느 경로로든 열린 재관측이 있으면 거절하고, 열린 task 의 연결·출처는 바꾸지 않는다. ④ 경로 선택: 그 대상에 수용된 카메라 후보가 하나라도 있으면 5행대로 가장 최근 수용 후보에 연결하고 8행 검사를 쓴다(출처 `CAMERA`). 수용된 후보가 하나도 없을 때만 운영자 경로다. `command_id` 를 3행 후보 `event_id` 자리에 두고 `command_id → {task_id}` 를 기록하며, 시각은 11행으로 검사한다(출처 `OPERATOR`). 수락하면 `accepted=true`·`ACCEPTED`·`created_task_id`=새 task 이고, 거절하면 `accepted=false`·`REJECTED`·`created_task_id` 빈 값이다. 운영자 경로 task 가 열린 동안 카메라 후보가 오면 후보는 기록만 하고 재관측을 새로 열지 않는다. 그 task 가 끝나고 `RESCUE_CONFIRMED` 가 아니면 5행대로 그 후보의 재관측을 예약한다. 한 판정은 한 경로의 근거만 쓴다. 운영자 경로의 재관측을 카메라 후보의 근거로 옮겨 쓰지 않는다. 판정 기록에는 7행의 후보 event_id 대신 `command_id` 와 출처 `OPERATOR` 를 남긴다. 운영자 판단은 웹캠 판정이 아니다. 카메라 ABSENT·LiDAR CLEAR·관측 범위 VALID(7·9행)는 그대로 모두 필요하다 | `target_id` 없음 → `INVALID_INPUT`. 모르는 대상 → `NOT_FOUND`. 판정이 끝난 대상 → `INVALID_INPUT`. 열린 재관측 있음 → `TASK_BUSY`. 같은 command_id·다른 내용 → `ID_CONTENT_CONFLICT`. 모두 `REJECTED` 이고 출동하지 않는다 |
| 11 | 운영자 경로의 시각 순서 (C-03) | 8행과 같은 시간 기준 조건을 쓴다(양쪽 시간 기준 선언, SYNCED, 오차 있음, 같은 `clock_domain_id`, 오차 합이 정책 상한 이하). 후보 `observed_at_ns` 대신 coordination 이 지시를 수락한 시각 `accepted_at_ns` 를 쓴다. coordination 은 이 시각을 자기 호스트의 `clock_domain_id`·오차와 함께 기록한다. 순서: `sensor_window_start_ns − accepted_at_ns > 오차 합`. 지시 전에 취득해 버퍼에 있던 프레임·스캔과 늦게 도착한 이전 샘플은 이 검사로 빠진다. task 예약 순서나 명령 순서로 이 검사를 대신하지 않는다. 관측자는 `ObserveTarget` goal 을 받고 정지를 확인한 뒤 취득한 샘플로만 창을 만든다(SDD ObserveTarget Rules 의 "새 관측 창"을 운영자 경로에서는 지시 수락 이후로 읽는다). 관측 나이·품질 조건(7·9행, C-03)은 그대로다. 구현 시험(인식·협업·관제): 지시 전 프레임·스캔을 재사용하면 거절 | 비교할 수 없음 → `CANDIDATE_TIME_BASIS_UNUSABLE`. 순서 위반 → `ORDER_VIOLATION`. 둘 다 `RECHECK_NEEDED` |

남은 의존 (2026-10-10, 10·11행은 2026-10-11):
- 11행도 8행과 같은 시간 기준이 필요하다. 아래 시각 오차 상한(Q-06)이 없으면 운영자 경로도 `CANDIDATE_TIME_BASIS_UNUSABLE` 로 `RECHECK_NEEDED` 가 된다.
- 10행의 운영자 경로는 관제 웹에 "웹캠 판정 불가 + 인형 안 보임" 알림과 그 대상의 재확인 지시 버튼이 있어야 쓸 수 있다(관제·화면). `REQUEST_RECHECK` 는 1차 IDL 에 있어 IDL 변경은 없다.
- coordinator 가 카메라에 watch 를 주는 `SetCameraWatch`(IF-078)는 #77 로 1차가 됐다(2026-10-11). 남은 것은 ROI 출처(#76·Q-13)와 카메라 노드 재시작 감지다. 조정기 쪽 watch 생명주기는 #148 로 생겼다.
- 시각 오차 상한 값과 웹캠 PC·AMR1 의 오차 측정(Q-06)이 없으면 모든 판정이 `CANDIDATE_TIME_BASIS_UNUSABLE` 로 `RECHECK_NEEDED` 가 된다. 설계대로지만 시연이 막힌다.

## 고정 원본에서 가져온 계약 근거

- 관측 재발행·수신 시각 혼동: [AMR webcam publish](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/webcam_detector.py#L109-L159), [follower 수신](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L290-L304).
- 대상 자체 goal·만료 후 기존 goal 유지: [go_near](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L388-L446), [approach](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L517-L528).
- AMCL 준비·직접 lifecycle 복구: [기동](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L109-L123), [복구](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L263-L288).
- 원 관측 시각 TF의 재사용 후보: [P4 target_locator](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/target_locator.py#L74-L107). stamp의 유효성/지도 보정 처리까지 검증 완료됐다는 뜻은 아니다.
- 도메인 A/B는 [P4 DEC-010](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/decisions.md#L142-L151)의 당시 후보/기록이며 새 배포 선택이 아니다. native frame의 접두사 없는 예시는 [adapters.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/adapters.py#L20), [follow_nav.launch.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/rokey_mini/launch/follow_nav.launch.py#L49-L60)에서 확인된다.
- 도킹·센서의 조건부 관계는 [로봇 3번 baseline](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/reference/2026-10-01-lab-environment-baseline.md#L40-L63), 운영자 전용 준비 action은 [Task.msg](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_interfaces/msg/Task.msg#L15-L17)에 있다. 접수 뒤 무동작은 [실습 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L26-L27)의 관찰이며 [원인은 미확인으로 기록됐다](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L66).
- 늦은 action 수락·취소 관리 후보: [P4 goal_timer](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/goal_timer.py#L83-L173).

원본 링크는 근거일 뿐 이 초안 계약의 기존 구현이 아니다. 원본 접근이 안 되면 확인하지 못한 근거를 명시하고, 출처 검토가 끝났다고 처리하지 않는다.

## 기록 (2026-10-14 옮김)

아래 C-12·C-13 은 지금 조정기 코드와 현장 설정에 맞지 않는다. 지우지 않고 옮겼으며, 이력 확인용으로만 남긴다. 2026-10-14 에 확인한 차이는 아래와 같다.

- C-12 다리 스켈레톤 추종은 2026-10-13 결정으로 뺐다(scope Q-29 ⑤, OPS-102). TB-5 가 앞서고 사람이 따라간다.
- C-13 버튼 출동은 2026-10-13 저녁 결정(OPS-136)으로 바뀌었다. 지금 현장은 재난 신호로 TB-6 탐색을 자동 시작한다. 「콘솔 버튼은 비활성 placeholder」라는 설명도 지금과 다르다. 관제 웹의 출동 버튼은 운영 명령 API 와 연결돼 있다(로봇 없는 리허설 A2·A3 PASS, 2026-10-14).

<a id="c-12"></a>

## C-12. AMR2 카메라 다리 스켈레톤 연속 추종 요구 (2026-10-11)

`[사용자 요구]` 사용자 원문(2026-10-11, PM 측 Codex 대화): “사람과 나란히 이동하는 게 아닌 캠으로 사람의 다리를 스켈레톤으로 추종해 뒤따라 가는 방식으로 수정해서 PR에 같이 첨부해.” 승인 정책 확인 답변: “출동 승인 후 목적지까지 연속 추종”. 적용 범위는 AMR2 동행 요구와 설계 검토다. 실제 사람인 구조대원이 앞서고 AMR2가 탑재 카메라로 다리 스켈레톤을 추적해 뒤따른다. 요구조자는 인형이며 이 추종 대상과 구분한다. 구간별 ‘계속’ 승인은 새 동행 시나리오에서 제외한다. 모델·추종 입력/액션·거리 기준·재시작 계약과 실기 검증은 Q-27의 후속이다. 기존 실행 코드를 연속 추종 구현으로 보지 않는다. [scope RQ-10·Q-27](../requirements/scope.md)의 요구 변경이며 저장소 소유자는 2026-10-11 채택했다. 세부 실행 계약과 실기 검증은 완료되지 않았다.

- 출동 승인 한 번 뒤 구조대원이 앞서고 AMR2가 목적지까지 연속으로 뒤따른다. 구간별 `CONTINUE_SEGMENT`는 이 시나리오에 사용하지 않는다. 목적지 좌표·허용 경로·승인 유효 범위와 도착·실제 정지의 조건은 후속 계약에서 명시해야 한다.
- PC2의 인식은 AMR2 탑재 카메라에서 실제 사람의 다리 키포인트를 관측한다. 추종 사람의 track과 요구조자 인형의 `target_id`/PC3 watch를 구별한다. `TargetPresence`·`SetCameraWatch`를 사람 추종 입력으로 재사용하지 않는다.
- 인식은 주행 명령을 직접 발행하지 않는다. 이동·취소·정지는 `rescue_robot`만 발행하고 활성 이동은 로봇당 하나다. Nav2 이동과 추종 출력의 동시 발행을 허용하지 않는다 [기존 C-06 권한 경계 유지].
- `[제안]` 동일 사람의 유효 관측·거리·경로와 live 권한/lease가 유지될 때만 이동한다. 가림·소실·관측 만료·거리 불확실·경로 이탈·위험에서는 취소 → 실제 정지 관측, 미확인 시 STOP_UNCONFIRMED를 따른다. 다른 사람으로 자동 전환하거나 재획득만으로 자동 재시작하지 않는다. 원인 해소·정지 확인·새 승인/명령 뒤에만 재시작한다.
- ROS 추종 입력의 필드/타입, 실행 action 또는 task kind, keypoint 모델·신뢰도·거리/시각 기준·TF·재획득·경로/통로 검사는 Q-27에서 인식·로봇·협업·관제 담당이 함께 정의한다. 기존 `ExecuteTask(MOVE_SEGMENT)`·`OperatorApproval(scope=ONE_SEGMENT)`·`RouteCandidate.segment_index`는 구간 이동 계약이며 이 요구의 구현이 아니다. 이번 PR은 이 IDL을 연속 추종 의미로 바꾸거나 추종 실행 코드를 추가하지 않는다. E3/E4는 NOT_RUN이다.

<a id="c-13"></a>

## C-13. AMR1 관제 버튼 출동 요구 (2026-10-11)

`[사용자 요구 및 저장소 소유자 채택, 2026-10-11]` 사용자 원문(PM 측 Codex 대화): “AMR1 출동 방식은 관제 모니터의 버튼으로 신호를 받아 출동하는거야”. 저장소 소유자는 #85에서 이 방식을 채택했다. 적용 범위는 AMR1 최초 정찰의 시작 조건이다. 재난 이벤트는 표시·근거 저장만 하고, 운영자의 관제 출동 버튼 명령 뒤에 지도 준비와 수색을 요청한다. IDL과 coordinator 명령 처리는 main에 있으나 현재 콘솔 버튼은 비활성 placeholder이며 operations POST·ROS 왕복은 미연결이다.

- 요구 흐름: PC3 SiteEvent(DISASTER_SIGN) → PC4 coordinator의 재난 검증·표시·근거 저장 → 관제의 AMR1 출동 버튼 → operations → coordinator의 지도 준비·ExecuteTask(SEARCH_AREA) 요청. 버튼 전에는 지도 START·최초 수색을 자동 발급하지 않는다. launch/PREPARE 등 사전 준비와 최초 임무 출동은 구별한다.
- `DISASTER → SEARCHING`은 유효한 운영자 출동 명령과 기존 지도·센서·TF·권한·lease 준비 관문 뒤에 일어난다. UI 버튼 접수는 실제 출동·수색 시작 완료가 아니다. 인식·웹은 Nav2/cmd_vel을 직접 호출하지 않는다.
- `START_SEARCH`는 #98에서 `SubmitOperatorCommand` 1차 IDL에 추가됐고 main의 coordinator 명령 처리에도 연결됐다. 콘솔 화면에는 출동 버튼 항목이 있으나 비활성 placeholder이며 operations POST 경로와 ROS 왕복은 Q-28 후속이다. 미연결 UI 요청을 수락된 출동으로 표시하지 않는다. 이 PR은 최신 계약을 문서에 반영하며 실행 코드·필드를 추가하지 않는다.
- Q-12의 2026-10-10 저장소 소유자 결정(AMR1 자동 출동·AMR2 구간 승인)은 과거 근거로 보존한다. 2026-10-11 소유자 결정에 따라 현재 AMR1 출동은 `START_SEARCH` 버튼 명령으로 게이트하며, main의 coordinator 처리도 이를 따른다. UI/API 연결·종단 간 시험과 E3/E4 실제 출동은 미검증이다.
- AMR1 재관측·복귀 정책과 AMR2 RQ-10은 이 요구의 적용 범위가 아니다. E3 버튼 왕복·명령 발급과 E4 실제 출동은 NOT_RUN이다.

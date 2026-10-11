# 계획 PR 8 — 접근·배정·통로 점유 코어

계획 PR 8(실제 GitHub PR 9)의 현재 범위는 접근 후보·배정 수락·대상 확인·통로 점유의 Python 정책 코어다. **수색 구역 분할·SEARCH_AREA 구역 배정·D-05 B안 frontier 후보 어댑터·자동 역할 교대는 F-01 후속 작업이며 GitHub PR 미발행**이다. 해당 기능을 이 PR이나 통합 리허설의 완료에 포함하지 않는다. 실제 Nav2 path service·ROS 임무 전송도 미구현이다. `PathCandidate`는 외부 planner가 제공할 증거이며 후보 선택이 이동 허가는 아니다. 실제 SLAM 보정 연동이 없어 mapping의 `auto_approach_eligible`은 false다.

## 구현한 경계

| API | 입력·판정 | 결과 |
| --- | --- | --- |
| `select_candidate(..., now_ns=...)` | 명시적인 가용 로봇, 경로 길이, footprint/경로 free 여부, clearance, 불확실성, `valid_until_ns` | 검증 입력을 만족하는 최단 경로 후보 또는 None. 대상 점 자체를 목표로 삼지 않음 |
| `Allocator(authority_epoch, allowed_robot_ids=...)` | 프로파일의 정확히 두 ID | DTO는 공통 ID 문법만 확인. allocator가 배포에 속하지 않은 로봇의 발급/예약을 거절 |
| `issue_generation(robot_id)` | 로봇별 발급 이력 | 0부터 증가하는 uint64. 발급은 수락이 아니며 `issued_generations`만 변경 |
| `reserve(Reservation)` | 발급된 미소비 세대, task/command/target/robot/boot/authority, 사건·로봇 단일 예약 | `pending`에만 저장. True는 예약 접수이며 실행기 수락을 뜻하지 않음. 두 카운터를 변경하지 않음 |
| `record_admission(TaskAdmission)` | 실행기 수락 응답의 command/task/robot/boot/authority/generation 정확한 일치 | accepted=true일 때만 `active`와 `accepted_generations` 갱신. 거절·늦은 응답·내용 충돌은 새 활성 임무를 만들지 않음 |
| `release` | 같은 task/robot/generation/authority와 stop_confirmed | pending/active 해제. 수락 응답 유실 중에도 확인된 정지 없이 예약을 버리지 않음 |
| `ApproachTracker.observe/guard` | 원 관측 ns 시각·기한, 증가하는 관측 시각, 사건/revision·신원·최신성, 로컬 수신 간격 | 새 관측만 local gap 갱신. 원 기한·짧은 유실 허용·guard·시계 순서가 깨지면 비가역 CANCEL |
| `ApproachTracker.complete` | 종료 시각 이후 재관측, 나이·거리·불확실성, 경로 성공·정지 확인 | 모든 조건을 만족할 때만 접근 확인 true |
| `Corridor.confirm_vacant(holder=...)` | 현재 revision, fresh 이탈 확인, 점유자의 command/task/robot/boot/authority/generation 일치 | HELD와 holder를 보존한 UNKNOWN 모두 동일 holder 확인이 필요. 최초 holder 없는 UNKNOWN은 holder=None과 fresh 비움 증거로만 FREE |

## generation과 실행기 수락

세대의 발급 주체는 coordinator다. `issued_generations`는 발급 번호를 재사용하지 않는 로컬 장부이고, `accepted_generations`는 실행기 admission의 수락 이력이다. 실행기 자체의 최고 수락 세대 검사는 유지한다. coordinator가 예약했다고 실행기 카운터와 수락 결과를 가정하지 않는다. `Reservation`에는 `command_id`와 `robot_boot_id`가 필수이고 `rescue_contracts.TaskAdmission`과 같은 여섯 신원 필드를 비교한다.

PENDING도 사건·로봇을 독점한다. 거절 receipt는 pending을 제거하지만 수락 카운터를 올리지 않으며 세대를 재사용하지 않는다. 수락된 receipt의 같은 내용 재전송은 멱등이다. 동일 receipt의 거절/수락 또는 내용 변경은 `ID_CONTENT_CONFLICT`다. 해제된 예약의 늦은 수락은 active를 되살리지 않는다. command ID를 다른 task에 재사용하거나 이미 소비한 세대로 새 task를 예약하지 않는다.

이 상태는 메모리 안에 있으며 durable coordinator·재시작 세대 복구·권한 인증을 구현하지 않는다. 살아 있는 실행기와 연결한 채 coordinator만 재시작해서 0부터 발급하는 것은 유효한 복구 절차가 아니다. 후속 운영 어댑터는 명시적 authority 재등록·정지 관문과 영속 발급 이력/실행기 수락 이력 조정을 구현해야 한다. E2 receipt 객체 자체가 네트워크 발신자 인증은 아니다.

## 시각·오류·외부 입력 책임

`PathCandidate.valid_until_ns`, `select_candidate(now_ns=...)`, `ApproachTracker.observe(observed_at_ns, valid_until_ns, now_wall_ns, ...)`, `guard(now_wall_ns, ...)`, `complete(now_wall_ns, navigation_finished_at_ns, ...)`는 양의 int64 ns다. `confirmation_max_age_ns`는 음수 아닌 int64 ns duration이다. float·bool·범위 밖 정수는 거절한다. 실제 센서의 sec/nanosec은 공통 `ros_time_to_ns`로 옮기며 epoch float로 만든 뒤 정밀도를 복원하지 않는다. `received_steady`, `now_steady`, `short_loss_max`와 통로 `now`/`duration`만 프로세스 로컬 monotonic 초다.

`CoordinationError`는 공통 `RescueError`를 상속한다. 소비자는 C-09 canonical `code`, `component`, namespace가 붙은 `detail_code`, 설명 `detail`을 읽는다. `DETAIL_CATEGORIES['rescue_coordination']`에 등록한 대문자 진단 코드만 사용하며 자유 문장은 `detail`에 둔다. 입력 진단은 명시한 대응표에 따라 INVALID_INPUT으로 분류한다. 미등록 코드나 오타는 ValueError를 전파하며 기본 분류로 조용히 바꾸지 않는다. 시계 역행은 CLOCK_UNCERTAIN, 의미 payload 충돌은 ID_CONTENT_CONFLICT다. 예상 가능한 배정 거절은 bool false이고 실제 실행기 거절의 canonical reason은 원 receipt에 남는다.

C-03/06/07, CT-06/09/10/11의 일부 E2 불변식을 구현한다. 호출자는 perception의 시각 검증, mapping의 원본별 좌표/목표 snapshot, executor admission과 stop 결과를 실제로 연결해야 한다. bool 입력 자체는 fresh 센서/정지/경로 증명이 아니다. `CONTINUE_EXISTING`은 현재 접근 목표의 짧은 유실 허용이며 새 목표 발행 허가가 아니다. A의 수색을 B의 대상 만료로 중단하는 전역 guard로 사용하지 않는다.

거리·속도·유실 시간을 장비값으로 확정하지 않는다. 메모리 통로 상태는 처음부터 UNKNOWN이며 lease 만료와 pose 손실 뒤에도 holder를 보존한다. 분산 election/lease 인증·영속 배정·분할 통로 deadlock 해결·실제 수색 분담은 후속 과제다.

## 역할 입력 수색·대기·접근 예약 (F-01 축소분, 2026-10-08)

BRD v0.1 시나리오(TB1 정찰, TB2 구조 지원)에 맞춘 최소 배정 정책이 `rescue_coordination/search.py`에 있다. 역할은 `SearchPolicy.roles`(`SCOUT`/`SUPPORT`) 입력이며 코드에 로봇 ID를 고정하지 않는다. 역할 입력을 바꾸면 두 로봇의 임무가 뒤집힌다. 수락 판정과 세대 장부는 두지 않고 위 `Allocator`만 읽고 쓴다.

| API | 판정 | 결과 |
| --- | --- | --- |
| `MissionSite`, `SearchPolicy` | 구역 ID·mission 좌표·근거 태그. `executable_bases`에 있고 map session/revision·transform revision이 모두 있는 구역만 실행 대상 | 나머지는 `excluded_areas()`에 닫힌 사유(`EXCLUSION_REASONS`)로 표시 |
| `plan(robot, robot_boot_id, ready, now_ns)` | 이미 pending/active가 있으면 `KEEP`. SCOUT는 설정 순서상 첫 미완료·미예약 구역, SUPPORT는 제자리 `HOLD` | `RESERVED`는 PENDING 예약과 `executor.Task` 필드. 비용 비교가 없어 작은 비용 변화로 임무가 바뀌지 않음. 동률은 설정 순서로 생기지 않음 |
| `request_approach(event_id, ...)` | mapping projection `VALID`, SUPPORT 소유의 유효한 `select_candidate` 결과, SUPPORT의 HOLD 해제 | 사건 ID당 접근 예약 1개. HOLD 중이면 `RELEASE_REQUIRED`, 다른 접근 중이면 `SUPPORT_BUSY` |
| `record_admission`, `finish(item, outcome, stop_confirmed)` | 실행기 receipt만 ACTIVE를 만들고, `release(stop_confirmed=True)`만 해제 | `COMPLETED`만 구역 수색 완료로 집계. 해제된 task의 늦은 receipt·결과는 새 배정을 되살리지 않음 |

재배정은 (1) 현재 구역 결과 수신·정지 확인 (2) `withdraw_area`로 구역 근거 철회(`RELEASE_REQUIRED`) (3) 로봇 부적격 뒤 실행기 정지 결과에서만 일어난다. SUPPORT가 접근 중일 때 SCOUT는 기본적으로 다음 구역 수색을 계속하고(`scout_continues_during_approach=True`), False면 새 구역 제안만 멈춘다. 반환 사유는 `DECISION_REASONS` 닫힌 집합이다.

`Reservation.task_kind`가 추가됐다. `APPROACH_TARGET`·`REOBSERVE_TARGET`은 `PathCandidate`가 필수이고(기본값이 `APPROACH_TARGET`이라 기존 호출도 후보가 필요), `SEARCH_AREA`·`HOLD`만 `candidate=None`을 허용한다. 예약은 PENDING이며 후보는 접근 전용 경로 증거다. `HOLD`는 실행기 의미대로 제자리 정지 유지이며, 대기 위치로의 이동은 기반 레인 후속이다. `task_fields`는 #14의 `search_area_ref`(SEARCH_AREA는 구역 ID, 그 외 None)와 `goal_pose`(SEARCH_AREA는 구역 mission 좌표와 근거 map session/revision, APPROACH는 후보 `goal_xy`와 후보 map session/revision, HOLD는 None)를 채운다. frame은 정책의 `mission_frame_id`다. yaw는 리터럴로 채우지 않는다: SEARCH_AREA는 구역의 필수 입력 `MissionSite.entry_yaw`(프로파일 v2.1 예정), APPROACH는 후보 지점에서 대상을 바라보는 각도 `atan2(target − goal)`(재관측 카메라가 대상을 향함)이다. 후보 지점과 대상이 같으면 방향이 없으므로 예약하지 않는다. `goal_pose`는 일반 dict로 주고 로봇 레인이 `GoalPose(**d)`로 바꾼다. coordination이 `rescue_robot`을 import하지 않게 하기 위해서다. 실제 지도 좌표 근거·frontier·전송·두 AMR·4 PC는 미실행이다.

공통 어휘와 로더(OPS-01, 2026-10-08): `planning.TASK_KINDS`·`TARGET_TASK_KINDS`와 `search.ROLES`는 복제하지 않고 `rescue_contracts`의 `TASK_KINDS`·`TARGET_TASK_KINDS`·`ROBOT_ROLES`를 그대로 쓴다. `search.mission_sites(areas, *, entry_yaws)`는 프로파일 v2 `mission.search_areas`를 순서대로 `MissionSite`로 옮긴다. 구역 ID·mission 좌표·근거 종류·map session/revision·transform revision을 그대로 복사하고, 실행 가능 여부는 판단하지 않는다(`SearchPolicy.executable_bases`가 정한다). 프로파일 v2에는 아직 `entry_yaw`가 없으므로 구역 ID마다 도착 방향을 `entry_yaws`로 따로 받는다. 빠지거나 남거나 유한하지 않으면 거부하고 기본값으로 채우지 않는다. 프로파일 v2.1이 `search_areas[].entry_yaw`를 갖게 되면 호출하는 쪽이 그 값을 넘긴다. coordination은 `rescue_bringup`을 import하지 않고 속성으로만 읽는다.

`goal_pose` dict에는 `transform_revision`도 넣는다(APPROACH는 후보의 값, SEARCH_AREA는 구역 근거의 값). `GoalPose.transform_revision`은 필수 필드다. 이 변경은 로봇·지도의 지도 보호 PR #22에서 들어왔고, 협업·관제가 그 부분을 검토했다.

## 출처와 변경 이유

- P4 [배정기](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/dispatcher.py#L253-L276)는 배정 책임을 한 곳에 둔 원리만 참고한다. 도킹한 로봇만 선택하거나 도크와 구역의 직선거리로 비용을 계산하는 정책은 사용하지 않는다.
- AMR [목표 갱신·도착](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L388-L446), [만료](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L517-L528)의 갱신 억제/유실 경험을 참고했다. 목표 자체로 접근하는 정책을 거리 구간·불확실성 조건으로 대체하고 도착을 fresh 재관측·정지와 분리했다.

원본 소스 복사 없이 새로 작성했다. 실제 길찾기 성공이나 사람 신원 판별 성능을 증명하지 않는다.

## 출동 대상 선택 (BRD Part I 3.4.3·3.4.5, Q-19, 2026-10-08)

`rescue_coordination/dispatch.py`의 `select_dispatch_target`은 확정된 대상이 여럿일 때 SUPPORT가 다음에 갈 대상을 고른다. `SearchCoordinator.select_approach_target`이 이를 감싸며, 이미 접근 예약이 있는 사건(`approach_tasks`)을 넘겨준다. 고르기만 하고 예약은 하지 않는다. 고른 결과를 `request_approach`에 넘기면 사건당 접근 하나가 지켜진다.

- 거리 기준은 정책 필수값 `DispatchPolicy.order`이고 기본값이 없다(Q-19).
  - `STRAIGHT_LINE`(제안 시작값): SUPPORT의 mission 좌표 `SupportOrigin`에서 대상까지의 직선거리다. 위치와 후보의 map session·revision·transform revision이 같을 때만 비교하고, 다르면 `BASIS_MISMATCH`다. 위치가 없거나, 다른 로봇이거나, `max_origin_age_ns`보다 오래됐거나, 미래 시각이면 아무것도 고르지 않는다(`ORIGIN_UNUSABLE`).
  - `PATH_LENGTH`: SUPPORT 후보의 검증된 경로 비용이다.
- 동률은 사건 ID가 작은 쪽이다. 입력 순서와 무관하다. 지원이 비었을 때만 고르므로, 작은 거리 변화로 진행 중인 접근을 바꾸지 않는다.
- 제외 사유는 닫힌 집합 `DISPATCH_SKIP_REASONS`다:
  - `NOT_CONFIRMED`: 미확정 후보(3.3.5)
  - `RESCUED`: 구출 완료
  - `EVENT_ALREADY_HAS_APPROACH`: 중복 출동 금지(3.4.5)
  - `PROJECTION_NOT_VALID`
  - `NO_SUPPORT_CANDIDATE`
  - `CANDIDATE_EXPIRED`
  - `BASIS_MISMATCH`
- `RECHECK_NEEDED` 대상은 다시 갈 수 있다. 접근이 실패해 해제된 사건도 `approach_tasks`에 남으므로 자동으로 다시 고르지 않는다. 재출동 규칙은 미결정이다.

## 임무 모드·운영자 명령 관문 (scope Q-12, 2026-10-10)

`rescue_coordination/mission.py`의 `MissionCoordinator`는 `SearchCoordinator` 위에 얹은 임무 한 건의 상태다. 세대 장부는 두지 않고 예약은 `SearchCoordinator.plan`에 맡긴다. 흐름과 관문은 [관제 웹·DB 계획](../design/console-and-db-plan.md) 1·4절과 [상태 전이](../design/state-machines.md) S-00을 따른다.

| API | 판정 | 결과 |
| --- | --- | --- |
| `record_site_event(event)` | `DISASTER_SIGN`이고 `SiteEvent.time_basis_declared`(C-03: stamp 의미 ≠ UNKNOWN, `SYNCED`, 오차 상한 있음)일 때만 NORMAL → DISASTER | 닫힌 사유 `MODE_EVENT_REASONS`. 다른 종류는 `NOT_DISASTER_SIGN`, 시각 기준이 없으면 `TIME_BASIS_UNUSABLE`, 이미 재난이면 `ALREADY_DISASTER`(첫 트리거 유지). 같은 event_id는 첫 결과를 그대로, 내용이 다르면 `ID_CONTENT_CONFLICT` |
| `plan(robot, ...)` | SCOUT는 DISASTER 전에는 `SearchCoordinator.plan`을 부르지 않는다. SUPPORT의 HOLD(제자리 정지)는 통과 | `PlanGate(reason, decision)`. `MODE_NOT_DISASTER`면 예약이 없다 |
| `handle_command(OperatorCommand, now_ns)` | 서버 쪽 관문. 모든 명령은 DISASTER에서만. `DISPATCH_SUPPORT`는 확정(`detection_confirmed`)·저장(`report_persisted`)·최신 revision·경로가 있는 대상에 한 번. `CONTINUE_SEGMENT`는 `segment_phase == AWAITING_APPROVAL`이고 대상 revision·경로 revision·구간 번호가 지금 것과 같을 때. `REQUEST_RECHECK`는 [계약](../design/contracts.md) C-11 10행(Q-26, 2026-10-11)대로 기록된 대상이고, `RESCUE_CONFIRMED`로 판정이 끝나지 않았고, 그 대상에 열린 재관측이 없을 때(출동·도착은 조건이 아니다). 받으면 대상별 `RecheckRequest`(출처 `OPERATOR`, command_id 를 후보 자리에, 수락 시각 `now_ns`)를 연다. `RETURN_HOME`은 판정이 나온 뒤, 두 역할 로봇 모두 복귀 위치(`HomePosition`)가 있고 그 지도 근거(map session·revision·transform revision)가 `record_map_basis`로 받은 지금 지도와 같을 때. 아니면 `NO_HOME_POSITION`·`HOME_BASIS_MISMATCH`로 거절하고 다른 위치로 대신하지 않는다 | `CommandResult(accepted, reason)`, 거절 사유는 닫힌 집합 `COMMAND_REJECT_REASONS`. 같은 command_id는 첫 결과를 그대로(그 뒤 대상이 판정됐어도 재판정하지 않음), 내용이 다르면 `ID_CONTENT_CONFLICT`. 종류 밖 명령·필드 누락/과잉은 `INVALID_INPUT` |
| `segment_started`·`segment_finished` | 승인 한 건이 구간 하나다(`SegmentApproval`: 명령 ID, `target_id`·`target_revision`·`route_revision`·`segment_index`). 이 네 키가 같은 구간만 `APPROVED → MOVING`. 끝은 정지 확인이 있어야 받고, 받으면 승인을 다 쓴다 | 완료면 다음 구간이 `AWAITING_APPROVAL`(마지막이면 `ARRIVED`), 미완료면 같은 구간이 다시 승인 대기. **다음 구간을 스스로 승인하지 않는다** |
| `record_route(target, route_revision, segment_count)` | 새 revision은 정지해 승인을 기다릴 때(또는 출동 전)만 바꿀 수 있고, 바뀌면 구간 0부터 다시 센다. 낮은 revision은 거절 | 이전 revision으로 누른 "계속"은 `ROUTE_REVISION_STALE` |
| `phase()`·`view()` | 1절의 표시 단계(`MISSION_PHASES`)를 계산한다. 화면은 읽기만 한다 | `CANDIDATE`는 `TARGET_DISAPPEARED_CANDIDATE`(#48) 전까지 나오지 않는다 |

`MOVE_SEGMENT`·`RETURN_HOME`은 아직 `rescue_contracts.TASK_KINDS`에 없다(#65). 그래서 이 모듈의 승인은 상태만 바꾸고 `Task`를 발급하지 않는다. `MOVE_SEGMENT`는 대상 임무로 정할 예정이라(#65, C-06) 후속 발급은 `approval`의 네 키를 그대로 `Task`에 싣는다. 복귀 위치는 프로파일 v2에 아직 없어(SUPPORT의 `hold_positions`만 있음) 호출하는 쪽이 넘긴다. 발급과 실행기 결과 연결, 재확인·복귀의 자동 시작, 카메라 후보(`TARGET_DISAPPEARED_CANDIDATE`) 수용과 출처 `CAMERA` 경로, 11행의 수락 시각 clock domain·오차 기록은 후속이다. 실제 ROS·DB·웹 연결은 미실행이다.

## 임무 조정기 ROS 노드 — 얇은 버전 (10-13 전 구간, 2026-10-11)

`rescue_coordination/ros_node.py`의 `mission_coordinator`(console_scripts)는 SDD `central.coordinator`의 얇은 버전이다. 판단은 ROS 없이 시험하는 `CoordinatorCore`가 하고, `main()`은 메시지만 옮긴다. ROS 패키지는 함수 안에서 import 하므로 ROS 없는 환경에서도 모듈과 코어를 시험할 수 있다. 로봇을 움직이는 것은 실행기이고, 이 노드는 `ExecuteTask` goal 을 보낼 뿐이다.

| 통신 | 방향 | 하는 일 |
| --- | --- | --- |
| `/rescue/site_events` (SiteEvent) | 받음 | `VALID` 시야만 사건으로 바꾼다(C-10). `DISASTER_SIGN` → 모드 관문 |
| `/rescue/robots/{r}/status` (RobotStatus) | 받음 | `READY`·`lease_valid`·나이 ≤ `status_max_age_s` 일 때만 준비됨 |
| `/rescue/robots/{r}/execute_task` (ExecuteTask) | 보냄 | 재난 모드의 SCOUT 에 `SEARCH_AREA`(승인 없음, Q-12), 승인마다 SUPPORT 에 `MOVE_SEGMENT`, 열린 재확인마다 SCOUT 에 `REOBSERVE_TARGET`. 결과는 정지 확인이 있어야 예약을 푼다 |
| `/rescue/robots/{r}/task_admission` (TaskAdmission) | 받음 | ACTIVE 로 가는 유일한 길. goal 수락만으로는 ACTIVE 가 아니다 |
| `/rescue/target_locations` (TargetLocation) | 받음 | `VALID` 투영만 대상이 된다. 아래 병합 규칙 |
| `/rescue/reports` (RescueReport) | 보냄 | 대상 revision 마다 보고 하나. `report_retry_s` 마다 PERSISTED·REJECTED 까지 다시 보낸다 |
| `/rescue/report_receipts` (ReportReceipt) | 받음 | 그 보고의 기록을 모두 포함한 `PERSISTED` 만 저장 확인(출동 관문의 "저장") |
| `/rescue/operator_command` (SubmitOperatorCommand) | 서비스 | 임무 관문 판정. 같은 command_id·같은 내용은 `DUPLICATE` 로 첫 응답을, 내용이 다르면 `ID_CONTENT_CONFLICT` |
| `/rescue/robots/{r}/command_lease` (CommandLease) | 보냄 | task 마다 `lease_id` 를 `ExecuteTask.lease_id` 에 싣고 `lease_renew_period_s` 마다 갱신한다(`renewal_sequence` 1씩 증가, 같은 boot·authority·generation). task 종료·거절, 재난 모드 아님, 그 로봇 상태가 낡음·다른 boot, authority/generation 불일치면 그 lease 는 다시 갱신하지 않는다. 그러면 실행기가 `LEASE_EXPIRED` 로 스스로 정지한다(C-10 「명령 lease」, #88). `valid_until_ns` 는 정보용 |
| `/rescue/system_state` (SystemState, TRANSIENT_LOCAL) | 보냄 | 바뀔 때만. `segment_phase` 는 APPROVED·MOVING→EXECUTING, AWAITING_APPROVAL→AWAITING_APPROVAL, ARRIVED→STOPPED. phase `FINISHED` 면 mode `MISSION_FINISHED`(C-10, #88) |

- **경로 증거 Phase 1 규칙**(작전 2026-10-11, #88): `EvaluateRoute` 가 2차라 planner 경로 증거가 없다. `planning.Reservation` 은 ONE_SEGMENT 승인과 구간 끝 `GoalPoint` 가 있는 `MOVE_SEGMENT`, 고정 관측 지점이 있는 `REOBSERVE_TARGET` 만 `PathCandidate` 없이 받는다. 승인 없는 MOVE_SEGMENT, 관측 지점 없는 REOBSERVE, APPROACH_TARGET 은 여전히 거절한다. `ExecuteTask.route` 에는 `approach_goal`(구간 끝 또는 관측 지점)만 싣고, `planned_path`·`path_sha256`·`planner_evidence_id` 는 비우며, `assessment` 는 `UNKNOWN`(알려진 값 없음)이다. `route.frame_ref` 는 대상의 마지막 `TargetLocation.frame_ref` 를 그대로 쓰고, alignment 가 `VALID` 가 아니거나 목표의 지도 기준과 다르면 발급하지 않고 로그에 남긴다(`TARGET_ALIGNMENT_NOT_VALID`·`ROUTE_BASIS_MISMATCH`). 실제 경로 검증은 실행기 guard·Nav2 몫이고 실패는 `TaskResult` 로 드러난다.
- **대상 병합**(작전 2026-10-11, Q-13·Q-21): 같은 지도 기준(map session·revision·transform revision)이고 `merge_distance_m` 안이면 같은 대상이고 revision 이 1 오른다. 밖이면 새 `target_id`(조정기 발급 UUID, C-11 1행)다. 기준이 다른 위치는 새 대상으로 나누지 않고 `FRAME_REVISION_MISMATCH` 로 거절한다. 같은 `location_id` 는 멱등, 내용이 다르면 `ID_CONTENT_CONFLICT`. 얇은 버전은 인식이 확정 탐지만 발행한다고 본다(작전 결정 3).
- **발급 순서**: 한 tick 에서 승인된 구간 → 열린 재확인 → SCOUT 수색 순서다. 그래서 비어 있는 SCOUT 은 새 구역보다 열린 재확인을 먼저 받는다. 막힌 재확인은 수색을 멈추지 않는다. 진행 중인 수색 구역은 끊지 않는다. 다음 구간은 운영자 `CONTINUE_SEGMENT` 없이 발급하지 않는다. 완료되지 않은 재관측은 새 task 로 다시 예약한다(C-11 5행).

운영자 서비스의 `reason_code` 는 C-09 공통 코드다(OPS-58). 임무 관문 사유는 노드 로그에 남긴다. REQUEST_RECHECK 의 세 줄은 C-11 10행이고, 나머지는 작전 승인(2026-10-11)이다.

| 임무 관문 사유 | `reason_code` |
| --- | --- |
| `MODE_NOT_DISASTER` | `NOT_AUTHORIZED` |
| `UNKNOWN_TARGET`, `NO_HOME_POSITION` | `NOT_FOUND` |
| `TARGET_NOT_CONFIRMED`, `NO_ROUTE`, `NO_VERDICT` | `PENDING_EVIDENCE` |
| `ALREADY_DISPATCHED`, `NOT_AWAITING_APPROVAL`, `RECHECK_ALREADY_REQUESTED`, `ALREADY_RETURNING` | `TASK_BUSY` |
| `TARGET_REVISION_STALE`, `ROUTE_REVISION_STALE`, `SEGMENT_INDEX_MISMATCH`, `NOT_DISPATCHED_TARGET`, `TARGET_ALREADY_JUDGED` | `INVALID_INPUT` |
| `HOME_BASIS_MISMATCH` | `FRAME_REVISION_MISMATCH` |
| 처리하지 않는 operation(`FINISH_MISSION`·`RESET_STOP`·`SET_CAMERA_WATCH`), 필수 필드 누락, 역할과 다른 `robot_id` | `INVALID_INPUT` |

파라미터는 모두 필수이고 기본값이 없다. 하나라도 없으면 `missing parameters` FATAL 로 exit 2 다. 기동할 때 git SHA·boot ID·적용 파라미터·역할을 로그에 남긴다. 종료는 `rclpy.try_shutdown()`.

| 파라미터 | 타입 | 뜻 |
| --- | --- | --- |
| `profile_path` | string | 배포 프로파일 v2(역할·수색 구역) |
| `entry_yaws` | double[] | 수색 구역 순서대로 도착 방향(rad) |
| `policy_version`, `executor_policy_version`, `mission_frame_id`, `authority_epoch`, `coordinator_id`, `envelope_schema_version` | string | `SearchPolicy`·goal·envelope 값 |
| `executable_bases` | string[] | 실행해도 되는 구역 근거 종류 |
| `task_duration_s`, `status_max_age_s`, `tick_period_s`, `report_retry_s` | double | 임무 기한·상태 나이 상한·판단 주기·보고 재전송 주기 |
| `lease_duration_s`, `lease_renew_period_s` | double | 실행기 정책의 lease 길이와 같은 값, 갱신 주기. 주기가 길이의 절반을 넘으면 기동 거부(exit 2) |
| `route_segment_goals` | double[] | 구간 끝 `[x, y, yaw]` × N (mission frame) |
| `route_revision` | int | 그 목록의 revision. 목록이 바뀌면 올린다(이전 승인은 쓰지 않음) |
| `observe_goal` | double[] | 재관측 지점 `[x, y, yaw]` |
| `goal_map_session_id`, `goal_map_revision`, `goal_transform_revision` | string, int, int | 위 목표들의 지도 기준 |
| `merge_distance_m` | double | 대상 병합 거리(Q-21) |

아직 하지 않은 것: 운영자 정지(RequestStop 은 operations → 로봇으로 바로 가서 이 노드가 보지 못함; 정지는 실행기 latch 가 맡는다), `HomePosition` 파라미터와 `RETURN_HOME` 배정(Q-22), `SetCameraWatch`(#77)·카메라 후보 수용과 판정 연결(#80·#32), `TargetSnapshot` 발행, 진행 중 수색 구역의 선점. E3 확인: `tests/ros/e3_coordinator_probe.py`(가짜 실행기 2대, 로봇 없음, `ROS_DOMAIN_ID` 10 이상·`LOCALHOST`·Discovery Server 해제).

## 검증

`tests/test_coordination.py`는 후보 보호 거리·경로/footprint/불확실성 거절, 비가용 관측 로봇, 중복 예약, 관측 원 기한·유실·확인, 통로 UNKNOWN/lease를 확인한다. `tests/test_coordination_review.py`는 profile ID 주입, PENDING→정확한 실행기 수락→ACTIVE, 거절·지연·충돌·재전송·세대 재사용 차단, HELD와 UNKNOWN의 holder 신원, 큰 epoch에서 1 ns 만료/종료 경계, 공통 오류를 확인한다. 이 PR의 수락 receipt 시험은 직접 만든 DTO를 사용한다. 실제 executor 연결 통합 경로와 E3/E4는 별도 검증이다. `tests/test_coordination_mission.py`는 재난 전 SCOUT 수색 차단, 사건·명령 ID 멱등과 내용 충돌, 시각 기준 없는 사건 거절, 명령마다의 관문 사유, 정지 확인 없는 구간 종료 거절, 다음 구간 자동 승인 없음, 승인 키(대상·경로 revision·구간) 불일치 거절, 재확인 지시의 C-11 10행 순서(모르는 대상·판정 끝난 대상·열린 재관측)와 대상별 기록, 경로 교체 뒤 이전 revision 거절, 복귀 위치 없음·지도 불일치 거절, 시나리오 순서의 표시 단계를 확인한다. `tests/test_coordination_target_goal.py`는 경로 증거 Phase 1 규칙(승인 없는 MOVE_SEGMENT·관측 지점 없는 REOBSERVE·APPROACH 거절)을, `tests/test_coordination_ros_core.py`·`test_coordination_ros_targets.py`는 노드 코어(메시지 변환, SCOUT 자동 발급, 운영자 서비스와 공통 코드, MOVE_SEGMENT·REOBSERVE 발급, 대상 병합, 보고 저장 확인, route frame, SystemState)를 ROS 없이 확인한다.

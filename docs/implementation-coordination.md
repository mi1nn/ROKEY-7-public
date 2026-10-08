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

후속(작전 검토 2026-10-08): 공통 규격 PR #17이 `main`에 들어가면 `planning.TASK_KINDS`·`TARGET_TASK_KINDS` 복제를 `rescue_contracts` import로 바꾼다(OPS-01). 프로파일 v2 `mission.search_areas`를 `MissionSite`로 옮기는 로더 함수를 둔다.

## 구출 완료 판정 (FR-20~24, KPI-06, S-05 구출 축, 2026-10-08)

`rescue_coordination/rescue.py`는 발견 사건 하나마다 `rescue_contracts.RESCUE_STATES`(`NOT_CHECKED`·`RECHECK_NEEDED`·`RESCUE_CONFIRMED`)를 판정한다. 판정만 하고, 재관측(`REOBSERVE_TARGET`) 배정은 인계 PR에서 한다. 화면의 "구출 완료" 표시는 판정 보고가 `PERSISTED`된 뒤 operations가 맡는다(E-04).

`RESCUE_CONFIRMED`는 세 가지 양성 증거가 모두 있을 때만 준다.

| 증거 | 입력(임시 DTO. 작전 계약 PR이 `rescue_contracts`로 옮기고 import 한 줄로 바꾼다) | 확정에 필요한 값 |
| --- | --- | --- |
| 대상 지점을 실제로 관측함 | `RecheckEvidence`: SCOUT의 `REOBSERVE_TARGET` 결과 `COMPLETED`·정지 확인, `target_in_view`, `depth_valid_at_target` | 모두 `True` |
| 그 관측에서 사람·장애물 없음 | `RecheckEvidence.person_detected_at_target`, `LidarCheck.target_in_scan_range`·`obstacle_at_target`, `RescuePolicy.lidar_detects_target_measured`(TR-12 실측) | 사람 `False`, 스캔 범위 `True`, 장애물 `False`, 실측 `True` |
| 출구 통과가 그 사건과 연결됨 | `ExitLink(SiteEvent(EXIT_PASS), target_id, link_basis)` | 허용된 `link_basis`, 신뢰도 하한 이상, 시각 기준 사용 가능 |

- `None`(모름)은 증거가 아니다. 하나라도 `None`이면 `RECHECK_NEEDED`다(E-05, FR-24). "탐지 없음"은 관측 범위 증거와 짝일 때만 센다. 부재 증거만으로는 확정하지 않는다(KPI-06).
- `RECHECK_NEEDED` 사유는 닫힌 집합 `RESCUE_HOLD_REASONS` 12개다: `TASK_NOT_COMPLETED`, `RECHECK_ROBOT_NOT_SCOUT`, `TARGET_NOT_OBSERVED`, `PERSON_PRESENT_OR_UNKNOWN`, `OBSTACLE_PRESENT_OR_UNKNOWN`, `EVIDENCE_STALE`, `MAP_BASIS_CHANGED`, `EXIT_NOT_LINKED`, `EXIT_TIME_BASIS_UNUSABLE`, `TIME_BASIS_INCOMPARABLE`, `ORDER_VIOLATION`, `EXIT_COUNT_EXCEEDED`.
- 순서: 재관측과 LiDAR 스캔은 접근 완료 뒤, 재관측은 출구 통과 뒤여야 한다. 구조 전 관측을 완료 근거로 쓰지 않는다. 출구 시각과의 비교는 세 조건을 모두 만족할 때만 한다: 출구 사건의 `time_basis_declared`(사전 조건일 뿐), 비교 상대(재관측 시각, SINGLE이면 접근 완료 시각)와 같은 `clock_domain_id`, 양측 오차 한계의 합이 `RescuePolicy.max_clock_error_sum_ns` 이하. 하나라도 아니면 비교하지 않고 `EXIT_TIME_BASIS_UNUSABLE`로 보류한다(C-03: 시간 기준을 쓸 수 없는 사건은 기록만 하고 판정 순서에 쓰지 않음). 비교할 때도 "뒤"로 인정하려면 시각 차가 양측 오차 합보다 커야 한다. 출구와 무관한 비교(재관측·LiDAR 스캔 vs 접근 완료, 재관측 vs 스캔)에도 같은 게이트를 건다. SCOUT·SUPPORT와 그 센서가 같은 시계 domain이라고 가정하지 않는다. 비교할 수 없으면 `TIME_BASIS_INCOMPARABLE`이다. 이는 순서가 틀린 `ORDER_VIOLATION`과 다른 사실이다.
- 시각: 재관측 나이와 카메라·LiDAR 시각 차는 호출자가 준 `now_ns` 기준이며 내부 시계를 쓰지 않는다. 미래 시각 증거나 판정 시각 역행은 `CLOCK_UNCERTAIN`이다.
- 출구 통과 하나는 사건 하나에만 연결된다(FR-19). 같은 내용 재연결은 멱등, 다른 내용은 `ID_CONTENT_CONFLICT`다.
- 집계 상한(BRD Part I 3.5.5 [제안]): 구출 완료 수는 출구 누적 카운트를 넘지 않는다. 카운트는 operations 장부의 `EXIT_PASS` SiteEvent 누적 건수이며 `TargetBasis.exit_pass_count`로 받는다. 트랙 중복 제거는 인식이 하고 여기서 다시 하지 않는다(#21 TS-04). 다른 확정 사건 수 + 1이 카운트를 넘으면 `EXIT_COUNT_EXCEEDED`로 보류한다. 이미 확정된 사건을 다시 판정할 때는 자기 자신을 세지 않는다. 카운트가 줄어든 입력은 `INVALID_INPUT`이다.
- `SINGLE_TARGET_SCENARIO`(Q-13 전 시연용) 연결은 세 조건을 모두 만족할 때만 쓴다: 정책의 `accepted_link_bases`에 명시돼 있음(기본 집합에 없음), 열린 사건이 정확히 1개, 출구 통과가 접근 완료 뒤. 판정 기록에 `link_basis`를 남겨 Q-13 결정 뒤 다시 평가할 수 있게 한다.
- 이력: 판정은 `RescueVerdict`로 쌓이고 덮어쓰지 않는다(S-05). 확정 뒤에는 **새로 기록된 증거**가 있을 때만 다시 판정한다. 시간 경과나 지도 revision 변경만으로는 확정을 내리지 않는다. 구출 완료는 과거 사실의 결론이지 공간 투영이 아니기 때문이다. 그래서 `TargetBasis.projection_valid`는 확정 전 판정에만 쓴다. 반대 증거로 내려간 판정에는 `demoted_from_confirmed=True`와 이전 판정 ID·시각을 남기고, KPI-06 오판 1건으로 센다. 그 보고는 새 report revision 또는 새 `report_id`로 잇는다.
- 상태는 메모리에만 있다. 재시작 복구·영속 기록은 후속(ledger 확장은 작전 승인)이다. 실제 인식·LiDAR 입력, 출구 카메라, 두 AMR은 미실행이다.

## 출처와 변경 이유

- P4 [배정기](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/dispatcher.py#L253-L276)는 배정 책임을 한 곳에 둔 원리만 참고한다. 도킹한 로봇만 선택하거나 도크와 구역의 직선거리로 비용을 계산하는 정책은 사용하지 않는다.
- AMR [목표 갱신·도착](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L388-L446), [만료](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L517-L528)의 갱신 억제/유실 경험을 참고했다. 목표 자체로 접근하는 정책을 거리 구간·불확실성 조건으로 대체하고 도착을 fresh 재관측·정지와 분리했다.

원본 소스 복사 없이 새로 작성했다. 실제 길찾기 성공이나 사람 신원 판별 성능을 증명하지 않는다.

## 검증

`tests/test_coordination.py`는 후보 보호 거리·경로/footprint/불확실성 거절, 비가용 관측 로봇, 중복 예약, 관측 원 기한·유실·확인, 통로 UNKNOWN/lease를 확인한다. `tests/test_coordination_review.py`는 profile ID 주입, PENDING→정확한 실행기 수락→ACTIVE, 거절·지연·충돌·재전송·세대 재사용 차단, HELD와 UNKNOWN의 holder 신원, 큰 epoch에서 1 ns 만료/종료 경계, 공통 오류를 확인한다. 이 PR의 수락 receipt 시험은 직접 만든 DTO를 사용한다. `tests/test_coordination_rescue.py`는 구출 완료 판정의 세 양성 증거, `None` 거부, 사유 닫힌 집합, 시각 비교 게이트, 출구 1:1 연결, 출구 카운트 상한, 확정 뒤 강등 이력을 확인한다. 실제 executor 연결 통합 경로와 E3/E4는 별도 검증이다.

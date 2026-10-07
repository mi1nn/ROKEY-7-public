---
title: 두 로봇의 임무·대상·정지 상태 전이 초안
status: draft-proposal
updated: 2026-10-07
---

# 상태 전이 초안

계획 PR 3의 논리 제안이며 실행 코드·실기 결과가 아니다. 두 로봇 각각의 실행기는 [인터페이스 계약](contracts.md)의 시간·좌표·권한 관문을 적용한다. 협업 coordinator는 임무를 배정하고 operations는 요청과 결과를 표시/기록한다. 화면의 버튼 상태와 실제 로봇 상태를 같은 변수로 취급하지 않는다.

## S-01. 동시에 유지할 상태와 우선순위

로봇마다 다음 상태를 별도로 가진다.

| 상태 축 | 값 제안 | 의미 |
| --- | --- | --- |
| readiness | `INITIALIZING`, `READY`, `DEGRADED`, `NOT_READY` | 센서·시계·TF·Nav2·설정 준비 상태. READY가 움직이고 있다는 뜻은 아님 |
| motion | `STOPPED`, `MOVING`, `STOP_REQUESTED`, `STOP_UNCONFIRMED` | 명령 접수와 관측된 정지를 구분 |
| safety/control latch | `DISARMED`, `ARMED`, `STOP_LATCHED`, `FAULT_LATCHED` | 재연결/새 탐지/늦은 goal 응답으로 latch 해제 금지 |
| task | 아래 S-03 상태 | task_id·generation·goal UUID와 함께 관리 |

충돌하는 이벤트의 우선순위는 **현장/운영자 정지·장비 fault → 권한/lease/필수 센서·시간·좌표 유효성 상실 → deadline/취소 → 정상 결과 → 새 목표/탐지**다. 같은 시점의 모든 원인은 기록하되, 낮은 우선순위 이벤트가 정지/실패를 성공으로 덮어쓰지 않는다. 실제 물리 정지를 확인할 수 없으면 어떤 상위 임무 상태에서도 `STOP_UNCONFIRMED`를 숨기지 않는다.

## S-02. 준비·재시작·제어권

| 현재 | 이벤트/관문 | 다음 상태·조치 |
| --- | --- | --- |
| 시작 | 새 robot/executor boot | DISARMED + INITIALIZING. 이전 task·lease·goal을 자동 재실행하지 않음 |
| INITIALIZING | 필수 설정 고정, 로봇 ID/버전 일치, live 센서/시간/TF/Nav2 준비 | READY + DISARMED. 운영자/유효 coordinator의 명시적 제어권 등록 대기 |
| READY + DISARMED | 새 authority 등록, 기존 권한 폐기·정지 확인, lease 수락 | ARMED. 단일 coordinator authority와 로컬 실행기 boot에 결합 |
| INITIALIZING | 준비 timeout·필수 값 미확인 | NOT_READY. 원인과 필요한 조치 보고; 무기한 조용히 대기하지 않음 |
| ARMED | 접근/수색에 필요한 준비 상실 | 임무 중단 관문 S-04로 이동. DEGRADED/NOT_READY 이유 기록 |
| STOP/FAULT_LATCHED | 원인 해결·정지/준비 확인·명시적 reset/resume | 이전 임무를 되살리지 않고 새 generation/권한으로 재등록 |
| 어느 상태 | coordinator 또는 로봇 실행기 재시작 | 이전 boot/epoch의 lease와 수락 여부를 불확실로 처리. 로컬 watchdog·재등록 절차 적용 |

SLAM에서 AMCL active를 필수로 기다리지 않는다. 준비 상태는 실제 선택한 위치 추정 backend의 관측 가능한 품질과 TF 계약으로 정의한다. 임무 노드가 Nav2 lifecycle 복구를 직접 시작하지 않고 소유 관리자에게 복구 요청을 보낸다. 복구 중에는 새 이동 임무를 받지 않는다.

## S-03. 로봇별 임무 상태 머신

| 현재 | 이벤트/필수 guard | 다음 상태·효과 |
| --- | --- | --- |
| `IDLE` | ExecuteTask 수신; 아직 action 수락 전 | `VALIDATING`; command/task/generation과 admission 처리 기록. 이동 명령 없음 |
| VALIDATING | ARMED/READY, 다른 이동 task 없음, 시간·권한·대상/구역·좌표 revision·필수 정책 유효 | action 수락 후 `PLANNING`; 접근은 안전 후보, 수색은 지정 구역 내 다음 목표를 계획 |
| VALIDATING | 잘못된 입력·만료·불일치·준비 부족 | goal 응답 accepted=false, admission 기록 `REJECTED`와 사유를 남기고 IDLE. action Result는 생성하지 않음 |
| PLANNING | 경로/목표 유효, 통로 자원 확보, goal 발행 직전 revision 재검증 | `NAVIGATING`; 발급한 goal UUID와 관측/변환 revision 고정 |
| PLANNING | 후보 없음/경로 없음 | budget 안이면 `REPLAN_PENDING`, 초과면 `UNREACHABLE`; 대상 사건은 삭제하지 않음 |
| NAVIGATING | 정상 진행 | 현재 goal의 feedback만 반영. 새 위치가 들어왔다고 매번 재발급하지 않음 |
| NAVIGATING | 경로 차단·재계획 필요 | `REPLAN_PENDING`; 기존 goal 처리와 정지 필요 여부를 정책대로 완료 후 새 경로 선택 |
| REPLAN_PENDING | retry/전체 deadline 남음, 위치·센서·명령권 유효 | PLANNING. 재시도마다 전체 deadline을 초기화하지 않음 |
| NAVIGATING | 현재 goal의 성공 결과 | 접근/재관측은 `VERIFYING`, 수색은 다음 수색 지점 계획 또는 구역 완료 확인. Nav2 성공을 대상 확인 성공으로 바꾸지 않음 |
| VERIFYING | fresh 대상 재관측·동일성/좌표/이격 조건 통과, 정지 확인 | `COMPLETED`; 접근 증거·실제 정지 위치·관측 ID를 결과에 첨부 |
| VERIFYING | 대상 없음/동일성 불확실/위치 조건 미충족 | `STOPPING`; 정지 관문 후 FAILED(reason=NEEDS_REOBSERVATION). 사건은 NEEDS_REOBSERVATION, 별도 재관측은 새 generation으로 배정 |
| 어느 비종료 상태 | 취소·deadline·관측 만료·frame 변경·lease 만료·정지 요청 | S-04의 STOPPING. 성공 결과가 뒤늦게 와도 해당 중단을 덮어쓰지 않음 |

action 수락 후 Result outcome은 `COMPLETED`, `CANCELED`, `EXPIRED`, `UNREACHABLE`, `FAILED`로 구분한다. 수락 전 REJECTED는 별도 admission 기록이며 action Result가 아니다. 다른 task가 활성일 때 새 요청이 들어오면 admission에서 거절하고 기존 task 상태를 유지한다.

정지 미확인은 motion 상태로 남으며 수락 임무의 Result는 `outcome=FAILED`, `terminal_reason=STOP_UNCONFIRMED`, `stop_confirmed=false`로 통일한다. 최초 취소/만료 원인도 보존한다. 이동 goal/속도 출력이 있었던 임무는 정상 완료·취소·실패 어느 경로든 종료 전에 정지 관문을 거친다. 정지 확인 실패 결과를 보낸 뒤에도 latch·감시와 새 임무 차단을 유지한다. 진행 중 목표가 없다는 내부 변수만으로 STOPPED를 만들지 않는다.

`SEARCH_AREA`는 로봇별 지정 구역·수색 정책을 실행한다. frontier 방식/지정 waypoint 방식의 선택은 PR 2에서 비교하고, 지금 특정 탐색 알고리즘의 구현을 주장하지 않는다. 검색·재계획은 실행 예산을 갖는다. 탐지 노드의 실패를 이유 없이 “수색 완료”로 전환하지 않는다.

`REOBSERVE_TARGET`는 대상의 위치/동일성을 새 관측으로 확인하는 별도 임무다. 마지막 알려진 위치를 표시하는 것과 자동 접근에 쓸 fresh 목표는 다르다. 오래된 위치에 대한 재관측 이동을 허용할지는 제한된 수색 영역·현재 장애물/위치 품질·접근 금지 영역을 갖춘 별도 정책으로 사전 결정해야 한다. 그 정책이 없으면 이동을 보류한다.

## S-04. 취소·정지·늦은 응답

1. 중단 원인과 활성 task/generation/goal UUID를 먼저 기록하고 새 goal 수락·재발급을 막는다.
2. 실행기는 해당 goal의 cancel과 최종 명령 억제를 요청한다. `STOP_REQUESTED`/`STOPPING`으로 표시한다.
3. cancel 접수/응답, Nav2 terminal result, 최종 command fence, 실제 정지 관측을 별도 기록한다. 응답 누락 시 정지 감시는 계속한다.
4. 정지 기준이 관측되면 STOPPED. 취소/실패/만료 원인에 맞는 종료 결과를 내고 coordinator에 알린다.
5. 정지 확인 deadline을 넘거나 odom/driver 상태가 끊기면 motion=STOP_UNCONFIRMED. 수락 임무는 FAILED(reason=STOP_UNCONFIRMED, stop_confirmed=false)로 보고한다. 웹에는 확인 불가를 표시하고 해당 로봇의 새 임무를 차단한다.
6. 원인 해결 후 명시적 재등록/재개와 새 generation 없이는 움직이지 않는다. 같은 임무 ID 재전송으로 정지 latch가 풀리지 않는다.

늦게 수락된 goal은 **그 goal UUID에 대해** 취소/억제하고 새 세대의 goal을 잘못 취소하지 않는다. 이전 task 결과와 현재 task 결과를 같은 navigator의 마지막 결과 값으로 혼동하지 않도록 goal별 기록을 둔다. 중복 cancel/stop 요청은 같은 결과를 반환하며 부수 효과를 반복하지 않는다.

최종 watchdog은 master PC의 추론·executor callback 지연과 독립적으로 로컬 만료를 감시하는 구조를 제안한다. 이 독립성이 실제 장비에서 확인되지 않으면 시험 계획의 정지 관문을 통과할 수 없다.

## S-05. 대상 사건 상태와 보고 상태

대상 사건은 **확인 상태**, **접근 상태**, **보고 상태**를 분리한다. 한 enum에 모두 넣으면 “보고했으나 접근 불가” 같은 정상 결과를 표현하기 어렵다.

| 축 | 전이·조건 |
| --- | --- |
| 확인 | `CANDIDATE` → 서로 일관된 유효 관측과 동일성 기준 충족 시 `CONFIRMED`; 관측/지도 기준이 만료·불확실하면 `NEEDS_REOBSERVATION`; 운영자/정책의 근거 있는 판정으로 `DISMISSED` |
| 접근 | `UNASSIGNED` → `ASSIGNED` → `IN_PROGRESS` → 정지·재확인·이격 조건을 통과하면 `APPROACH_COMPLETE`; 그 외 `BLOCKED`, `UNREACHABLE`, `CANCELED`, `FAILED` |
| 보고 | `NOT_QUEUED` → `QUEUED` → `RECEIVED` → 필수 내용/증거가 저장되고 receipt 확인 시 `PERSISTED`; 실패 시 `RETRY_PENDING`/`PENDING_EVIDENCE` 유지 |

처음 발견했을 때 사건을 기록하고 접근 불가여도 위치·근거·실패 사유를 보고한다. `APPROACH_COMPLETE`와 `PERSISTED`가 모두 확인된 정상 접근·보고 시나리오를 완료로 집계한다. 접근 불가 시험은 접근 성공률의 성공 분자에 넣지 않고, 별도 “실패 처리·보고 성공” 지표로 집계한다.

새 관측으로 대상 위치가 바뀌어도 기존 report를 덮어쓰지 않고 report revision 또는 새 report_id로 연결한다. 같은 사람으로 병합한 근거·별개로 유지한 이유·운영자 정정 이력도 보존한다. 인물 탐지 confidence로 건강/생존 상태를 생성하지 않는다.

## S-06. 두 로봇 협업 전이

| 단계 | coordinator의 판단·관문 | A/B 동작 |
| --- | --- | --- |
| 분담 수색 | 두 로봇 READY/ARMED, 겹침/통행 정책·구역 정의 | A/B 각각 자신의 SearchArea 실행 |
| A 발견 | 관측의 시간·좌표·class·위치 품질 확인, 사건 생성/중복 검토 | A는 유효 관측을 전달; 미확인 사건을 즉시 접근 완료로 표시하지 않음 |
| B 선정 | 현재 위치·상태·경로/자원·배터리 정책 확인 | B의 수색 임무를 취소·정지 확인한 뒤 새 generation 배정 |
| B 접근 | 공통 좌표·대상 유효성·통로/명령 lease 유지 | B는 후보 목표에 접근. A의 관측 지원 필요 여부는 인계 정책대로 수행 |
| B 재확인 | B 관측이 같은 대상인지 확인, 이격·정지·좌표 관문 | B는 멈춰 결과 보고. A는 별도 구역 수색 유지/재개 |
| 결과 저장 | 보고 receipt·필수 근거 확인 | 접근 결과와 보고 결과를 구분한 사건 종료/후속 수색 결정 |

두 로봇은 영구적으로 발견 전용/접근 전용 역할로 고정하지 않는다. A 발견→B 접근은 협업을 보여주는 필수 시험 시나리오이며 역할을 바꾼 회차도 시험 목록에 둔다.

A가 수색을 계속하느라 대상이 관측에서 사라진 경우에는 마지막 좌표를 새 관측처럼 갱신하지 않는다. B의 로컬 재관측이 확보되기 전까지 A의 지원을 유지할지, 제한된 재관측 임무를 쓸지는 **시험 전에** 결정한다. 어떤 정책이든 유효기간 초과를 숨기고 접근을 계속하는 경로는 허용하지 않는다.

coordinator가 B의 정지/위치를 확인하지 못하면 접근 성공이나 통로 이탈을 추정하지 않는다. 다른 로봇으로 재배정할 때도 이전 로봇의 점유/이동 상태를 확인하거나 해당 영역을 차단한 대체 계획을 써야 한다. 단순 lease 만료로 두 로봇이 같은 좁은 통로에 진입하지 않게 한다.

## S-07. 주요 실패 전이

| 실패 | 즉시 처리 | 재개 조건·보고 |
| --- | --- | --- |
| 대상 관측 만료·모델 결과 지연 | 진행 중 접근 STOPPING, 사건 NEEDS_REOBSERVATION | 유효한 새 관측·새 generation; 원 관측과 새 관측을 구분 |
| frame/map revision 변경 | 영향받은 배정/goal 중단, 파생 위치 invalid | backend가 원 관측 보정을 지원하면 재계산·검증, 아니면 재관측 |
| localization/scan/TF 품질 상실 | 이동 제한·정지; 과거 pose를 live로 사용하지 않음 | readiness 복구·원인 확인·명시적 재개 |
| 장애물 추가 | 충돌 보호 경로 유지, 경로/자원 재검토 | budget 안에서 우회/대기, 없으면 UNREACHABLE |
| 장애물 제거 | 과거 막힘을 즉시 free로 덮어쓰지 않음 | 새 센서 관측·costmap/지도 상태와 경로 가능성 확인 |
| master/coordinator 통신 단절 | lease/로컬 watchdog에 따른 정지·상태 불확실 기록 | 연결 복구만으로 출발하지 않음; authority·task 재조정 |
| operations/DB 중단 | 로봇은 설정된 임무/정지 정책 수행, 보고는 로컬 보류 | 동일 report_id로 재전송·중복 제거, 저장 receipt 확인 |
| 다른 로봇 ID·낮은 generation·이전 boot 명령 | 거절·감사 기록 | 활성 정상 임무를 바꾸지 않음 |
| Nav2 실패 반복·action 결과 없음 | 제한된 재시도·전체 deadline·정지 관문 | FAILED/UNREACHABLE 등 관측된 outcome 보고. 정지 미확인이면 FAILED(reason=STOP_UNCONFIRMED, stop_confirmed=false) |

실패 후 자동 회복을 넣는 경우에도 권한·시간·좌표·정지 조건을 생략하지 않는다. 운영자 reset은 근거 없는 성공 판정을 대신하지 않는다.

## S-08. 상태 전이 시험의 최소 불변식

- 로봇당 활성 이동 권한/goal의 소유자는 하나이며 다른 로봇 ID가 상태를 바꾸지 않는다.
- 원 관측의 observed_at과 최초 deadline은 중계·재시도로 증가하지 않는다.
- 완료한 이전 task/goal의 늦은 이벤트가 현재 generation을 바꾸지 않는다.
- STOP/FAULT_LATCHED에서 명시적 재개 관문 없이 MOVING으로 가지 않는다.
- map/transform revision이 invalid인 관측으로 새 접근을 시작하지 않는다.
- Nav2 성공, cancel 응답, 0 속도 발행, DB 수신 ACK를 각각 접근 완료·정지 완료·보고 저장 완료로 과장하지 않는다.
- 보고 실패·접근 불가·통신 단절도 사건과 원인을 보존하며, 성공률 계산의 분모에서 임의로 빼지 않는다.

## 원본 근거와 한계

이 계약은 [P4의 임무 입력 관문](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/controller.py#L175-L210), [늦은 action 수락 처리 후보](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/goal_timer.py#L83-L173), [AMR의 목표/만료 경로](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L388-L446), [접근 정보 만료](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L517-L528)를 참고해 새 요구를 분리한 것이다. 원본의 상태 이름/상수·실기 성능을 승계하지 않았으며 테스트를 실행하지 않았다.

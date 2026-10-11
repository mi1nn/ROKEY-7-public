---
title: 관제 웹·DB 구성 계획 — PC4(operations·web·DB)
status: draft   # 테이블·API·화면 구성은 [제안]. 담당자(오케스트레이터·웹, 상태 감시) 확인과 PR 머지로 확정한다
updated: 2026-10-11
---

# 관제 웹·DB 구성 계획

[SDD 노드·인터페이스 검토안](sdd-node-interface-spec-v0.1.md)의 PC4 몫(central.operations · central.web · central.db, IF-080 ~ IF-084)을 **지금 코드 위에 어떻게 올릴지** 적는다. 시나리오는 [BRD Part I](../requirements/brd-part1-v0.1.md)과 2026-10-09 팀 결정(고정 카메라 1대, AMR2 승인 관문)을 따른다. 날짜·담당은 [일정](../process/schedule.md)과 PM 의 일정 v2(10-10 ~ 10-13 웹 줄) 기준이다.

AMR2 동행 요구는 2026-10-11 사용자 요청 [RQ-10·Q-27](../requirements/scope.md)으로 수정한다. 구조대원이 앞서고 AMR2가 탑재 카메라의 다리 스켈레톤을 추적해 출동 승인 후 목적지까지 연속 추종한다. 구간별 ‘계속’은 새 시나리오에서 제외한다. 이 계획의 추종 상태·재시작 명령은 [C-12](contracts.md#c-12)의 요구·제안이며 현재 API/IDL 구현을 뜻하지 않는다.

## 1. 흐름과 승인 관문

| 단계 | 누가 | 웹이 하는 일 |
| --- | --- | --- |
| 웹캠 진동 → `SiteEvent(DISASTER_SIGN)` | 인식(카메라) | 재난 모드 배지, 트리거 시각 표시 |
| AMR1 정찰 시작 | **coordinator 자동** (승인 없음) | AMR1 상태·지도·탐색 영역 표시. 수동 시작 버튼은 자동 실패 시 대체 수단 |
| 요구조자 좌표 → DB | 인식 → operations | 대상 마커·4단계 상태 |
| AMR2 출동 | **운영자 승인** (`DISPATCH_SUPPORT`) | 승인 버튼. 확정 대상이 있을 때만 활성 |
| 사람 뒤 연속 추종 → 목적지 정지 | 출동 승인 뒤 로봇 실행기 [Q-27 계약 미정] | 동일 구조대원 추종 상태·입력 유효성·목적지·중단 이유 표시. 구간별 계속 버튼 없음 |
| 추종 중단 → 재시작 | 원인 해소·정지 확인 뒤 **새 운영자 승인** [제안, Q-27] | 자동 대상 전환·자동 재시작 금지. 재시작 입력/액션은 미정 |
| 웹캠 소실 → `TARGET_DISAPPEARED_CANDIDATE` | 인식(카메라) | "구조 완료 후보" 표시 |
| AMR1 재확인 → 판정 | coordinator 자동 (수동 `REQUEST_RECHECK` 가능) | RESCUE_CONFIRMED / RECHECK_NEEDED |
| 복귀 → 종료 | coordinator 자동 (수동 `RETURN_HOME` 가능) | 복귀·종료 표시 |

새 동행 시나리오는 **AMR2 출동 승인 한 번 뒤 연속 추종**이다. 안전 정지 후 재시작 승인은 별도 검토한다(Q-27). 기존 `CONTINUE_SEGMENT`·`MOVE_SEGMENT` 계약은 구간 이동용으로 보존하지만 연속 추종 흐름에 연결하지 않는다. AMR1 시작은 자동이므로 KPI-02(이벤트 → 출동 명령)는 AMR1 에 그대로 적용하고, AMR2 는 "승인 후 n초" 로 따로 잰다(Q-25).

웹이 보는 임무 상태(표시용, 판정은 coordinator):
`IDLE → DISASTER → SEARCHING → TARGET_FOUND → DISPATCH_APPROVED → FOLLOWING (중단 시 STOPPING → STOPPED → 재승인 대기) → ARRIVED → CANDIDATE → RECHECKING → RESCUE_CONFIRMED | RECHECK_NEEDED → RETURNING → FINISHED`

## 2. 프로세스와 역할

PC4 한 대(관제 PC)에 프로세스 둘.

| 프로세스 | 패키지 | 책임 | 담당 |
| --- | --- | --- | --- |
| operations_node | `rescue_operations` | DB **단일 쓰기**, HTTP API(IF-080·081·083·084), ROS 어댑터(로봇 상태·현장 사건·보고 구독, coordinator 서비스 호출, RequestStop) | 오케스트레이터·웹 |
| web_console | `rescue_console` | 브라우저에 주는 HTML. 읽기는 DB 읽기 함수, 명령은 operations API 로 프록시 | 오케스트레이터·웹 |
| (DB) | SQLite 파일 2개 | 보고 장부(`reports`·`evidence`·`outbox`, 그대로) + 현장 상태(`site_state`, 1.0 → 1.1) | 테이블·쓰기 함수·오류 이력: 상태 감시 |

- 웹캠 영상은 IF-082 대로 카메라 PC 의 MJPEG 게이트웨이를 `<img src>` 로 넣는다. 콘솔이 영상을 중계하지 않는다.
- SSE(IF-081 의 `/api/events`)는 동결 뒤. Phase 1 은 #37 의 1초 fragment 폴링.
- 현장 정지 수단은 웹과 독립이다([작업 규칙](../../AGENTS.md) 6절, 명세 QoS·저장·실패 절).
- PC 이름은 저장소에 쓰지 않는다. 프로파일에는 논리 호스트(PC4 역할)와 IP 만 둔다.

## 3. DB (site_state schema 1.1) [제안]

기존 `robot_status`·`site_events`·`target_states`(#34) 에 더한다.

| 테이블 | 열 | 쓰는 시점 |
| --- | --- | --- |
| `mission_state` (1행) | mode(NORMAL/DISASTER), phase(1절 상태), trigger_event_id, started_at_ns, generation | 사건·승인·완료마다 |
| `operator_commands` | command_id, kind, operator_id, robot_id, target_id, segment_index, route_revision, requested_at_ns, accepted, result, reason | POST 마다 |
| `error_log` | at_ns, robot_id, task_id, code, detail | 실패 경로마다 |
| `map_snapshots` | snapshot_id, robot_id, revision, path, sha256, at_ns | IF-084 PUT 마다 |

기존 표 변경: `robot_status` 에 battery·dock_state·motion_state·segment_index·readiness, `site_events.kind` 에 `TARGET_DISAPPEARED_CANDIDATE`, `target_states` 에 segment_phase. `exit_pass_count` 는 카메라 1대 결정으로 항상 0 이며 화면에서 숨긴다.

## 4. HTTP API (operations) [제안]

| 경로 | 뜻 |
| --- | --- |
| `GET /api/snapshot` | mission + robots + targets(4단계 합성은 #37 규칙표) + events + 미처리 명령 |
| `POST /api/operator-commands` | kind ∈ {START_SEARCH(수동 대체), DISPATCH_SUPPORT, CONTINUE_SEGMENT, SET_CAMERA_WATCH, REQUEST_RECHECK, RETURN_HOME, PAUSE, RESUME, CANCEL, STOP_ROBOT}. 응답 accepted / request_id / reason. STOP 은 RequestStop 으로 먼저, 나머지는 coordinator 서비스 |
| `PUT /api/evidence/{id}` | 웹캠 후보 jpeg + sha256 → PERSISTED / REJECTED |
| `PUT /api/maps/{id}` | 지도 PNG + revision |
| 로그인 | 세션 쿠키. 역할 `ADMIN`(화면·명령·계정 관리)과 `CUSTOMER`(보기만). ADMIN 이 CUSTOMER 계정을 만들고·고치고·지운다(강사 day5 요구: 로그인 페이지 + SQLite 로그인 데이터). 계정 파일·세션 비밀은 저장소 밖([SDD 10절 목록](architecture.md) 에 추가). operator_id 가 명령에 박힌다 |

**감시 시작(SET_CAMERA_WATCH)** [제안, #76 CDX-76-02]: 대상 감시 영역(ROI)은 프로파일의 camera watch 값이다. 운영자가 확정 대상 하나를 고르고 그 ROI 를 연결·확인하면 operations 가 `SET_CAMERA_WATCH`(target_id, ROI, `association_method=OPERATOR_ROI`, `association_evidence_id`=이 명령의 command_id)를 coordinator 로 보낸다. coordinator 가 `watch_id` 를 발급해 `SetCameraWatch` 로 카메라에 준다([contracts C-11](contracts.md#c-11) 2행). 자동 등록은 하지 않는다. 카메라 설치 각도나 영상 크기가 바뀌면 그 watch 는 무효이고 다시 연결한다.

**명령 관문은 서버에 둔다.** CONTINUE_SEGMENT 는 `segment_phase == AWAITING_APPROVAL` 일 때만, DISPATCH_SUPPORT 는 확정 대상이 있을 때만, RESUME 은 정지 증거 뒤에만(Q-22). 버튼 활성화는 이 관문의 거울이지 관문 자체가 아니다. 받은 명령은 전부 `operator_commands` 에 남긴다(거절 포함).

## 5. 화면

한 페이지 + 로그인.

| 자리 | 내용 |
| --- | --- |
| 위 | 모드 배지(평시/재난, 트리거 시각), 웹캠 영상 |
| 왼쪽 | 지도 PNG + AMR1·AMR2 마커 + 대상 마커 |
| 가운데 | 로봇 표(준비·이동·배터리·도킹·임무·추종/중단 상태 [Q-27]), 대상 표(4단계 + 후보), 사건 로그(KPI-04 지연 열) |
| 오른쪽 | 버튼: AMR2 출동 승인 · 추종 재시작 승인 [Q-27] · 재확인 지시 · 복귀 · 일시정지 · 재개 · 취소 · 정지 · (수동 탐색 시작) |

AMR2 카메라·통과 박스 표시(일정 v2 r46)는 통로 판정 범위가 정해진 뒤(Q-25) 자리만 둔다.

## 6. 순서 (일정 v2 웹 줄)

| 날 | 할 일 | 담당 |
| --- | --- | --- |
| 10-10 오전 | 이 계획 확정, site_state 1.1 스키마 PR, 로그인 | 상태 감시, 오케스트레이터·웹 |
| 10-10 오후·저녁 | snapshot API, 웹캠 영상 표시, 지진 상태 표시 | 오케스트레이터·웹 |
| 10-11 | 지도, 상태 수집 → DB, 오류 이력, (수동 탐색 시작 버튼) | 상태 감시, 오케스트레이터·웹 |
| 10-12 | 출동 승인·추종/중단 상태 [Q-27 계약 검토 후], 제어 버튼, 통합 launch 에 operations_node | 오케스트레이터·웹, 상태 감시 |
| 10-13 | 후보 → 재확인 → 완료 → 복귀 흐름, E2E | 전원 |

PR 은 작게 넷으로 나눈다.
1. 상태 감시: site_state 1.1 + 쓰기 함수 + 시험(#34 위에 얹거나 #34 머지 뒤).
2. 오케스트레이터·웹: operations HTTP API(순수 Python 코어 + Flask 경계) + 명령 관문 시험.
3. 오케스트레이터·웹: #37 에 로그인·웹캠 embed·지도·명령 패널.
4. 오케스트레이터·웹: operations ROS 어댑터(E3, rclpy). 코어는 의존성 없이 유지.

기반이 되는 열린 PR: #29(MissionSite 로더), #34(읽기 API·site_state), #37(콘솔 골격), #38(출동 후보 선택). 이 넷이 먼저 main 에 들어가야 1 ~ 3 이 올라간다.

## 7. 열린 것

| 무엇 | 막는 것 | 정할 사람 |
| --- | --- | --- |
| 지도 소스: SLAM map PNG 를 IF-084 로 밀어 넣기(권장) vs map 토픽을 operations 가 직접 구독 | 10-11 지도 | 오케스트레이터·웹, 로봇·지도 |
| 로그인 계정 파일 위치와 형식 | 10-10 로그인 | 오케스트레이터·웹 |
| 통과 박스 화면(r46) 포함 여부 | 10-12 화면 | PM (Q-25) |
| `#41` 재확인 트리거를 출구 카운트 → 웹캠 후보로, `#32` 의 "확정 수 ≤ 출구 카운트" 조건 제거 | 10-13 판정 흐름 | 오케스트레이터·웹 |

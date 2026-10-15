---
title: DB 스키마 — 운영 장부·현장 상태·콘솔 계정
status: draft   # 1.0 절과 계정 표는 main 기준, 4절(현장 상태 1.1·1.2·1.3·1.4·1.5)은 구현됨·어휘는 [제안]. 테이블을 바꾸는 PR 이 이 문서를 같이 바꾼다
updated: 2026-10-15
---

# DB 스키마

## 한눈에 (2026-10-14, main 3278a3e)

이 문서가 답하는 질문은 「현장 상태 DB(스키마 1.5)의 표마다 누가 쓰고 누가 읽는가」다. 쓰는 프로세스는 운영 PC 의 `db_writer` 하나이고, 관제 웹은 읽기 전용(`mode=ro`)으로만 연다.

| 표 | 담는 것 | 쓰는 함수 (입력) | 읽는 쪽 |
| --- | --- | --- | --- |
| `robot_status` | 로봇당 최신 상태 1행 | `record_robot_status`·`record_robot_pose` (`/rescue/robots/<id>/status`·`/pose`) | 관제 웹 `read_robot_status` |
| `site_events` | 고정 카메라 사건(진동 감지·대상 소실 후보) | `record_site_event` (`/rescue/site_events`) | 관제 웹 `read_site_events` |
| `target_states` | 대상당 최신 상태 1행 | `record_target_state` (`/rescue/targets`) | 관제 웹 `read_target_states` |
| `mission_state` | 임무 상태 1행(모드·단계·출동·사유) | `record_mission_state` (`/rescue/system_state`) | 관제 웹 `read_mission_state` |
| `operator_commands` | 운영자 명령과 답(거절 포함) | `CommandGate` (`POST /api/operator-commands`) | 관제 웹 `read_operator_commands` |
| `error_log` | 실패·거절 기록(추가 전용) | 수신 노드(ROS 입력 거절)·`CommandGate`·`WebcamSwitch` | 관제 웹 `read_error_log` |
| `exit_locations` | 출구(QR) 관측(VALID 만) | `record_exit_location` (`/rescue/exit_location`) | 관제 웹 `read_exit_locations`(최신 1건) |
| `overhead_passage_lists`·`overhead_passages` | 로봇별 터널 판정 목록 | `record_overhead_passages` (`/rescue/robots/<id>/overhead_passages`) | 관제 웹 `read_overhead_passages` |
| `current_map` | TB-6(SCOUT) 최신 지도 revision 1행 | `record_current_map` (`/rescue/robots/<TB-6 id>/map_state`) | 관제 웹 `read_overhead_passages` |
| `map_snapshots` | 지도 이미지 경로·해시 | 함수 `record_map_snapshot` 은 있으나 부르는 노드가 없음 | 함수 `read_map_snapshots` 는 있으나 쓰는 화면이 없음(관제 웹 지도는 `map_snapshot` 노드가 쓰는 파일을 직접 읽음) |
| `runs` (1.5) | 회차 목록(시작·끝·지도 세션·메모) | `SiteStateLedger` 가 열 때(열린 회차가 없으면), `open_run`·`close_run` | 리플레이 `list_runs`, 모든 `read_*` 결과의 `run_id` |
| `robot_status_history`·`target_state_history` (1.5) | 리플레이용 로봇 상태·대상 상태 이력 | `record_robot_status`·`record_robot_pose`·`record_target_state` 가 함께 씀 | 리플레이 `read_robot_status_history`·`read_target_state_history` |

다른 SQLite 파일은 세 가지다. 운영 장부(`operations-ledger`)와 보고 송신 대기열(`report-outbox`)은 `rescue_operations/ledger.py`, 관제 웹 계정(`console-accounts`)은 `rescue_console/accounts.py` 가 쓴다. 구조 보고 저장 장부(`report_node`, `report_store.py`)는 이 문서 밖이다([운영 구현](../implementation/implementation-operations.md)).

SQLite 파일 역할은 송신 대기열을 포함해 네 가지이며, 역할마다 별도 파일을 쓴다. **쓰는 쪽은 파일마다 하나**이고, 다른 쪽은 읽기 전용(`mode=ro`, `query_only`)으로만 연다. 파일 경로는 코드에 두지 않고 인자·프로파일로 받으며, 네트워크 공유 폴더에 두지 않는다([implementation-operations](../implementation/implementation-operations.md) 운영 경로 규칙). 테이블을 바꾸는 사람은 상태 감시 담당이고 이 문서를 같은 PR 에서 바꾼다([console-and-db-plan](console-and-db-plan.md) 2절).

| 파일 (역할 값 `metadata.role`) | 쓰는 쪽 | 읽는 쪽 | 코드 | 상태 |
| --- | --- | --- | --- | --- |
| 운영 장부 `operations-ledger` (schema 1.0) | operations (`ReportLedger`) | 콘솔, 협업 | `rescue_operations/ledger.py` | main |
| 현장 상태 `operations-site-state` (schema 1.5, 1.0~1.4 파일은 열 때 올림) | operations (`SiteStateLedger`) | 콘솔 | `rescue_operations/site_state.py` | 1.0 은 main, 1.1 은 4절 |
| 콘솔 계정 `console-accounts` (schema 1.0) | 콘솔 (`AccountStore`) | 콘솔 | `rescue_console/accounts.py` | main (#50) |
| 보고 송신 대기열 `report-outbox` (생산자 쪽 별도 파일) | 보고 생산자 (`ReportOutbox`) | 생산자 | `rescue_operations/ledger.py` | main |

테이블 사이 관계를 그림으로 보려면 [운영 DB 파일과 테이블 관계도](../process/ops-db-tables.md)(웹마스터)를 본다. 표·열 정의의 기준은 이 문서와 코드다.

공통: 모든 파일에 `metadata(key TEXT PRIMARY KEY, value TEXT)` 표가 있고 `role`·`schema`(또는 `schema_version`) 를 담는다. 기존 파일을 재사용할 때 역할·스키마·정책이 다르면 `POLICY_MISMATCH` 로 거절하며, sqlite 오류는 `STORAGE_ERROR` 로 끝난다. 장부의 읽기 전용 조회와 `AccountStore.open()` 은 없는 파일을 만들지 않는다. 쓰기 담당의 `ReportLedger`·`ReportOutbox`·`SiteStateLedger` 초기화와 명시적 `AccountStore.create()` 는 새 파일을 만든다. 시각은 전부 정수 나노초(`*_ns`), JSON 열은 `canonical()`(키 정렬·공백 없음) 직렬화다.

## 1. 운영 장부 (`operations-ledger`, schema 1.0)

`metadata`: `schema=1.0`, `role=operations-ledger`, `evidence_policy`(증거 요구 정책), `allowed_robot_ids`(허용 로봇 ID JSON). 다시 열 때 하나라도 다르면 거절.

| 표 | 열 | 뜻 · 규칙 |
| --- | --- | --- |
| `reports` | `report_id TEXT PK`, `payload_sha256`, `payload_json`, `envelope_json`, `state`, `revision INTEGER`, `receipt_id`, `received_at_ns`, `persisted_at_ns NULL`, `error_code NULL` | 구조 보고 한 건. `state` ∈ RECEIVED / PENDING_EVIDENCE / PERSISTED. 같은 `report_id` 재전송은 payload 해시가 같으면 멱등(같은 receipt), 다르면 `ID_CONTENT_CONFLICT`. 필수 증거가 다 저장되기 전엔 PERSISTED 가 아니다 |
| `evidence` | `evidence_id TEXT PK`, `ref_json`, `content BLOB`, `stored_at_ns` | 증거 원 bytes. `ref_json` 의 sha256 과 bytes 해시가 다르면 거절 |
| `outbox` (생산자 쪽 별도 파일) | `report_id TEXT PK`, `payload_sha256`, `payload_json`, `state`, `attempts INTEGER`, `receipt_json NULL`, `last_error NULL` | 송신할 보고와 받은 영수증. `state` 는 넣을 때 `QUEUED`, 보내다 실패하면 `RETRY_PENDING`, 영수증을 받으면 그 영수증의 상태(`RECEIVED` / `PENDING_EVIDENCE` / `PERSISTED`)로 바뀐다. `PERSISTED` 행도 보존하지만 재전송 대상에서는 뺀다. 재전송은 같은 payload 로만 |

읽기: `read_reports(db_path, limit, report_id)` — 보고 공개 필드만(payload·state·receipt·시각).

## 2. 현장 상태 (`operations-site-state`, schema 1.0 의 표, main)

1.1·1.2·1.3 에서도 이 세 표는 그대로이고 4절의 표가 더해진다. `metadata`: `schema=1.4`(1.0~1.3 파일은 4절대로 올림), `role=operations-site-state`, `allowed_robot_ids`. 보고 장부와 **다른 파일**이다(재오픈 정책이 달라 한 파일에 섞지 않음, #34).

| 표 | 열 | 뜻 · 규칙 |
| --- | --- | --- |
| `robot_status` | `robot_id TEXT PK`, `robot_boot_id`, `state_sequence INTEGER`, `stamp_ns`, `status_json`, `received_at_ns` | 로봇당 최신 1행. 같은 boot 는 `state_sequence` 가 클 때만 교체(같은 sequence 에 다른 내용이면 `ID_CONTENT_CONFLICT`), 다른 boot 는 `stamp_ns` 가 클 때만. `status_json` 키: robot_id, robot_boot_id, state_sequence, role(SCOUT/SUPPORT/UNKNOWN), stamp_ns, pose{x,y,yaw,frame_id}, motion_state, readiness, battery_percent, battery_stamp_ns, active_task_id, generation, 그리고 선택 키 dock_state(DOCKED/UNDOCKED/UNKNOWN)·segment_index(1차 IDL `RobotStatus`, 2026-10-11; 없는 옛 행은 None 으로 읽음). pose 는 `record_robot_pose` 로 따로 바뀌고(받은 시각 `received_at_ns` 는 그대로) 같은 sequence 의 동일성 비교에서 빠진다 |
| `site_events` | `event_id TEXT PK`, `kind`, `observed_at_ns`, `event_json`, `received_at_ns` | 고정 카메라 사건 추가 전용(`SiteEvent` 를 `asdict` 로 저장). 같은 내용 재전송은 멱등, 다른 내용은 `ID_CONTENT_CONFLICT`. 현재 Python 계약 어휘는 `DISASTER_SIGN`·`TARGET_DISAPPEARED_CANDIDATE`(#48, `target_id`·`watch_id` 를 함께 저장)·`EXIT_PASS`(카메라 1대 결정으로 생산자 없음)다. #48 전에 저장된 행에는 `target_id`·`watch_id` 키가 없다. 행을 고쳐 쓰지 않고, 읽을 때와 재전송을 비교할 때 계약 기본값(None)으로 채운다 |
| `target_states` | `target_id TEXT PK`, `updated_at_ns`, `state_json` | 대상당 최신 1행. `updated_at_ns` 가 클 때만 교체, 같으면 내용이 같아야 함. `state_json` 키: target_id, position{x,y,frame_id,map_session_id,map_revision}, detection_confirmed, rescue_state(계약 `RESCUE_STATES`), rescue_reasons(RECHECK_NEEDED 일 때만, 비어도 됨 — `TargetEstimate` 에 사유 없음), verdict_id, support_state(NONE/ASSIGNED/ARRIVED/FAILED [제안]), support_robot_id, updated_at_ns, 그리고 선택 키 target_revision(조정기 `TargetEstimate.estimate_revision`, #115 콘솔이 TB-5 출동 승인에 그대로 보냄)·persisted_report_id(조정기가 그 revision 의 보고 저장 영수증을 받았을 때의 보고 ID, 빈 값은 None. #115 콘솔은 이 값이 있는 대상만 출동 승인에서 고름). 두 선택 키 모두 없는 옛 행은 None 으로 읽고 같은 내용 재전송은 멱등 |

읽기: `read_robot_status`, `read_site_events(limit)`, `read_target_states`. `exit_pass_count` 는 저장된 서로 다른 `EXIT_PASS` 수다. 카메라 1대 구성의 화면에서는 출구 카운트를 숨기지만, 기존·합성 기록이 있으면 DB 집계는 0이 아닐 수 있다. 구조 4단계(미확정/미구조/지원 중/구출 완료)는 저장하지 않고 콘솔이 합성한다(Q-20, #37 규칙표).

## 3. 콘솔 계정 (`console-accounts`, schema 1.0, main)

콘솔이 소유하는 파일. 운영 장부·현장 상태의 단일 쓰기 규칙과 무관하다. `metadata`: `role=console-accounts`, `schema_version=1.0`.

| 표 | 열 | 뜻 · 규칙 |
| --- | --- | --- |
| `accounts` | `username TEXT PK`, `account_id TEXT UNIQUE`, `display_name`, `email`, `role`, `status`, `password_hash`, `auth_version INTEGER`, `created_at_ns`, `updated_at_ns` | `role` ∈ ADMIN / CUSTOMER, `status` ∈ ACTIVE / INACTIVE. `password_hash` 는 `pbkdf2_sha256$반복$소금$해시`(평문 없음). `account_id` 는 생성마다 새 UUID(같은 이름을 지웠다 다시 만들면 다른 값), `auth_version` 은 비밀번호·역할·상태가 바뀔 때마다 +1 — 세션 쿠키가 둘을 담아 이전 세션이 서버에서 거절된다. 마지막 ACTIVE ADMIN 은 비활성화·강등·삭제 불가 |

읽기·쓰기는 모두 콘솔 프로세스. 관리자 1명은 `python3 -m rescue_console.accounts init` 으로 만든다. 계정 파일과 세션 비밀은 저장소 밖에 둔다(SDD 외부 파일 목록에 추가할 항목).

## 4. 현장 상태 1.1 (구현됨, 2026-10-11) — 2절 표에 더하는 표

[console-and-db-plan](console-and-db-plan.md) 3절의 추가분을 `rescue_operations/site_state.py` 에 구현했다. 2절 표는 그대로 두고 아래 네 표를 더한다. 어휘 중 `MISSION_PHASES` 와 콘솔 전용 명령(PAUSE·RESUME·CANCEL·STOP_ROBOT)은 계획 문서에서 옮긴 **[제안]** 이고, 명령 종류는 1차 IDL `SubmitOperatorCommand` 의 operation 을 모두 포함한다(시험이 IDL 파일과 대조). 어휘를 바꿀 때는 이 표와 코드 상수를 같은 PR 에서 바꾼다.

| 표 | 열 | 뜻 · 규칙 |
| --- | --- | --- |
| `error_log` | `seq INTEGER PK`, `at_ns`, `robot_id NULL`, `task_id NULL`, `code`, `detail` | 실패 경로마다 한 행(3.7.4). **추가 전용**이고 같은 실패가 반복되면 새 행이다. `code` 는 C-09 공통 코드(`rescue_contracts.CANONICAL_CODES`)만 받고, `rescue_robot.NAV2_INACTIVE` 같은 세부 원인은 `detail`(1~1024자)에 쓴다. `robot_id` 는 배치 허용 로봇 또는 None(시스템 오류), `task_id` 는 UUID 또는 None |
| `mission_state` (1행) | `singleton INTEGER PK (=1)`, `mode`, `phase`, `trigger_event_id NULL`, `started_at_ns NULL`, `generation NULL`, `updated_at_ns`, 1.2: `target_id NULL`, `target_revision NULL`, `route_revision NULL`, `segment_index NULL`, `segment_count NULL`, `segment_phase NULL`, `reason NULL` | `mode` ∈ 계약 `SYSTEM_MODES`(S-00: NORMAL / DISASTER / CLEARING — 오탐 해제 중) 와 `MISSION_FINISHED`(phase FINISHED 일 때만, C-10 `SystemState`), `phase` ∈ 계획 1절의 표시용 상태 14개. 출동 칸 여섯은 TB-5 출동이 없으면 모두 None, 있으면 모두 있고 `segment_index` ≤ `segment_count`, `segment_phase` ∈ `SystemState.SEGMENT_PHASE_*` 7개. `generation` 은 `SystemState` 에 없어 None 일 수 있다. `reason` 은 조정기가 임무를 붙잡을 때의 사유 코드(#91 CLA-02, 예: 접근 막힘)이고 없으면 None. `trigger_event_id`·`started_at_ns` 는 짝으로만 있다. `updated_at_ns` 가 클 때만 교체, 같으면 내용이 같아야 함(`ID_CONTENT_CONFLICT`). 판정은 coordinator 가 하고 이 행은 화면 표시용이다 |
| `operator_commands` | `command_id TEXT PK`, `kind`, `operator_id`, `robot_id NULL`, `target_id NULL`, `segment_index NULL`, `route_revision NULL`, `requested_at_ns`, `accepted INTEGER`, `result NULL`, `reason NULL` | 콘솔 명령 한 건과 그 응답, **거절도 기록**. `kind` 는 1차 IDL `SubmitOperatorCommand` 의 operation 8개와 콘솔 계획의 PAUSE·RESUME·CANCEL·STOP_ROBOT. 거절(`accepted=0`)이면 `reason` 필수. 같은 ID 같은 내용은 멱등, 다른 내용은 `ID_CONTENT_CONFLICT`. 관문 판정은 operations API 몫이고 이 표는 요청·응답만 남긴다 |
| `map_snapshots` | `snapshot_id TEXT PK`, `robot_id`, `revision`, `path`, `sha256`, `at_ns` | 지도 이미지 한 장(IF-084). 이미지 bytes 는 DB 에 넣지 않고 `path`·`sha256`(소문자 64자)만 둔다. 같은 ID 같은 내용은 멱등 |
| `exit_locations` (1.3) | `location_id TEXT PK`, `observed_at_ns`, `received_at_ns`, `map_session_id`, `map_revision`, `transform_revision`, `mission_frame`, `x REAL`, `y REAL`, `class_label` | 출구(QR) 관측 하나(`/rescue/exit_location`, 통합-인식 안 「나」, 작전 OPS-144·150). **VALID 만** 기록하고(아니면 거절해 `error_log`), `class_label` 은 늘 `exit_qr`. 관측 원본이지 조정기가 등록·재기준한 출구가 아니다(관제 웹은 「관측」으로 표시). 같은 ID 같은 관측은 멱등이고 처음 받은 `received_at_ns` 를 유지, 관측 내용이 다르면 `ID_CONTENT_CONFLICT` |
| `overhead_passage_lists` (1.4) | `robot_id TEXT PK`, `generated_at_ns`, `received_at_ns`, `map_frame`, `clearance_m REAL`, `floor_noise_m REAL`, `lidar_plane_m REAL` | 로봇별 머리 위 구조물(터널) 목록의 머리(C-04b, `/rescue/robots/<id>/overhead_passages`, 작전 OPS-194). 해석은 `rescue_contracts.overhead.parse_overhead_passages` 하나로만 하고 기록 때 계약 검사를 다시 거친다. 잘못된 형식, 토픽과 다른 `robot_id` 는 거절해 `error_log`. 빈 목록도 한 행이다(받은 적 없는 로봇은 행이 없음) |
| `overhead_passages` (1.4) | `robot_id`, `id`(PK 는 둘), `state`, `lowest_m REAL NULL`, `x REAL`, `y REAL`, `map_session_id`, `map_revision`, `frames`, `first_seen_ns`, `last_seen_ns`, `reason` | 그 목록의 항목, C-04b 필드 그대로(UNKNOWN 이면 `lowest_m` None). **목록이 기록이다**: `generated_at_ns` 가 더 큰 목록이 오면 그 로봇의 머리·항목을 한 트랜잭션에서 통째로 바꾸고, 새 목록에 없는 항목은 지운다(이전 revision 항목은 로봇이 목록에 남겨 보내므로 그대로 남는다). 같은 시각 같은 목록은 멱등(처음 받은 `received_at_ns` 유지), 같은 시각 다른 내용은 `ID_CONTENT_CONFLICT`, 더 오래된 목록은 버린다 |
| `current_map` (1행, 1.4) | `singleton INTEGER PK (=1)`, `robot_id`, `map_session_id`, `map_revision`, `generated_at_ns`, `received_at_ns` | SLAM 로봇(SCOUT, TB-6)의 최신 `MapState`. 두 로봇의 터널 항목은 모두 이 지도 위에서 잰다. 관제 웹이 이 revision 항목은 진하게, 다른 revision 은 흐리게 그린다(hv-06). `generated_at_ns` 가 클 때만 교체, 같으면 내용이 같아야 함 |

올리기 규칙(1.4 까지):  쓰는 쪽(`SiteStateLedger`)이 1.0 파일을 열면 **같은 트랜잭션**에서 빠진 표를 만들고, 1.1 파일을 열면 `mission_state`(1행)를 1.2 모양으로 다시 만들어 기존 행을 옮긴 뒤(SQLite 는 NOT NULL 을 그 자리에서 풀 수 없음), 1.2 파일은 `exit_locations` 표를, 1.3 파일은 1.4 의 세 표를 더한다. 1.5 에서 더하는 것은 4-1절에 있다. 다른 표와 열은 지우지 않는다. schema 말고 role·허용 로봇이 다르면 지금처럼 `POLICY_MISMATCH` 이고 파일을 바꾸지 않는다. 읽기 함수는 최신 schema 파일만 받으므로, 이전 파일은 operations 가 한 번 연 뒤에 읽힌다. 운영 PC 의 기존 파일은 열기 전에 복사본을 남긴다.

읽기: `read_error_log(limit)`·`read_operator_commands(limit)`·`read_map_snapshots(limit)` 는 최신 순(`limit` 은 최근 N건), `read_exit_locations(limit)` 는 `observed_at_ns` 최신 순·같으면 `location_id` 오름차순(관제 웹은 맨 위 하나를 지도에), `read_mission_state` 는 `mission` 하나(기록 전에는 None). `read_overhead_passages` 는 `robots`(로봇 ID → 머리 필드 + `items`, 항목은 `last_seen_ns` 최신 순·같으면 `id` 오름차순, 받은 적 없는 로봇은 키 없음)와 `current_map`(기록 전에는 None). `read_allowed_robot_ids` 는 metadata 의 배치 로봇 목록(정렬)이다.

보류 — JSON 키 추가: `robot_status.status_json` 의 `dock_state`·`segment_index` 는 1차 IDL `RobotStatus` 에 맞춰 선택 키로 넣었다(2절, 2026-10-11). `target_states.state_json` 의 `segment_phase`·`candidate_event_id` 는 아직 넣지 않았다. 지금 검증(`schema.exact`)은 키 집합이 정확히 맞아야 해서, 키를 더하면 발행 쪽(operations ROS 어댑터)과 읽는 쪽(콘솔)이 같이 바뀌어야 한다. 넣을 때는 5절대로 읽는 쪽이 없는 키를 기본값으로 읽게 한다.

## 4-1. 현장 상태 1.5 — 회차와 리플레이 (구현됨, 2026-10-15)

관제 웹의 초기화 버튼과 리플레이 페이지를 위해 더했다(작전·웹개발 합의 2026-10-15, 초기화 범위 (나)는 재범 결정). 초기화는 DB 파일을 바꾸지 않고 **새 회차**를 연다. 지난 기록은 지우지 않고 회차별로 남는다(재범 결정 2026-10-15). 회차 구분 키는 `run_id` 다. `map_session_id` 는 배치 프로파일의 고정값이라 초기화해도 바뀌지 않으므로 회차를 가르지 못한다.

| 표 | 열 | 뜻 · 규칙 |
| --- | --- | --- |
| `runs` | `run_id TEXT PK`, `started_at_ns`, `ended_at_ns NULL`, `map_session_id NULL`, `note NULL`, `previous_coordinator_boot_id NULL` | 회차 하나. `ended_at_ns` 가 None 이면 열린 회차이고 **한 번에 하나만** 열린다(부분 고유 색인 `runs_one_open`, 두 번째 `open_run` 은 `TASK_BUSY`). `SiteStateLedger` 를 열 때 열린 회차가 없으면 하나 연다. 같은 파일을 다시 열면 열린 회차를 잇는다. `map_session_id` 는 그 회차에서 처음 받은 `MapState` 의 값이다. `previous_coordinator_boot_id` 는 초기화 때 마지막으로 들은 조정기 boot 이고, 그 boot 의 `SystemState`·`TargetSnapshot` 은 이 회차에 기록하지 않는다 |
| 쌓이는 표 다섯(`site_events`, `operator_commands`, `error_log`, `map_snapshots`, `exit_locations`)의 `run_id NULL` | 마지막 열 | 쓸 때 열린 회차. 회차 사이(닫은 뒤 열기 전)에 쓴 행은 None 이고 지우지 않는다. 재전송 비교에는 `run_id` 를 넣지 않는다 |
| `robot_status_history` | `robot_id`, `run_id NULL`, `at_ns`(받은 시각), `status_json` | 로봇 상태(위치 포함)를 **로봇당 1초에 한 줄 [제안]**(`ROBOT_HISTORY_PERIOD_NS`)로 솎아 남긴다. 상태와 위치 토픽 어느 쪽으로 바뀌어도 그 순간의 스냅샷 전체를 쓴다 |
| `target_state_history` | `target_id`, `run_id NULL`, `updated_at_ns`, `state_json` | `target_states` 가 바뀐 때마다(더 새 `updated_at_ns` 로 교체됐을 때) 한 줄. 재전송과 더 오래된 상태는 남기지 않는다 |

나머지 표(`robot_status`, `target_states`, `mission_state`, `current_map`, `overhead_passage_lists`·`overhead_passages`)는 지금 값만 담는 표다. 초기화(`SiteStateLedger.reset_run`)는 `robot_status` 를 뺀 다섯 표를 비운다. `robot_status` 는 재기동 확인에 쓰므로 남긴다. 초기화는 닫는 회차의 `robot_status_history` 에 두 로봇의 그 순간 상태를 한 줄씩 더 남긴다(`read_reset_checklist` 의 「전」 boot).

열린 회차 ID 는 쓰는 객체가 메모리에 쥐지 않고 쓸 때마다 `runs` 에서 읽는다. 다른 연결이 회차를 바꿔도 다음 행이 DB 의 열린 회차로 들어간다. `operator_commands.kind` 에 `RESET_RUN`(관제 웹 초기화, operations 가 직접 답함)을 더했다.

화면용 읽기 함수는 이름·인자·정렬·한도를 바꾸지 않는다. 모든 결과에 열린 회차의 `run_id`(없으면 None)를 싣는다. 쌓이는 표를 읽는 `read_site_events`(`exit_pass_count` 포함), `read_operator_commands`, `read_error_log`, `read_map_snapshots`, `read_exit_locations` 는 **열린 회차의 행만** 돌려준다. 열린 회차가 없으면 빈 결과다. 지난 회차의 `DISASTER_SIGN` 사건이 새 회차 화면을 재난 모드로 보이게 하지 않으려는 것이다(관제 웹 `view.py` 는 `mission_state` 행이 없으면 사건으로 모드를 추정한다).

리플레이 읽기 함수는 새로 더했다. `list_runs` 는 회차를 연 순서대로 주고 `open_run_id` 를 같이 준다. `read_site_events_range`·`read_operator_commands_range`·`read_error_log_range`·`read_map_snapshots_range`·`read_exit_locations_range`·`read_robot_status_history`·`read_target_state_history` 는 `run_id` 하나와 시간 창(`start_ns` 이상 `end_ns` 미만, None 이면 열린 끝)을 받아 **오래된 것부터** 준다. `limit` 은 1~1000 이고 더 있으면 가장 새 행들이 잘리며 `truncated` 가 True 다(관제 웹은 「일부만 보임」으로 드러낸다).

1.4 파일을 열면 같은 트랜잭션에서 쌓이는 표에 `run_id` 를 더하고, 기존 행을 닫힌 회차 하나(`note` 「이전 기록」, 가장 오래된 행의 시각부터 연 시각까지)로 묶는다. 쌓인 행이 없는 파일에는 이 회차를 만들지 않는다. 그 뒤 새 회차가 열리므로, 올린 직후의 화면은 비어 있다. 운영 PC 의 기존 파일은 열기 전에 복사본을 남긴다.

## 5. 바꿀 때 지키는 것

- 쓰는 쪽은 파일마다 하나. 콘솔은 장부·현장 상태에 쓰지 않는다.
- 같은 ID 재전송은 같은 내용이면 멱등, 다른 내용이면 거절. 오래된 것(낮은 sequence·이른 시각)은 기록하지 않고 `False` 를 돌려준다.
- JSON 열의 키를 더할 때는 읽는 쪽이 없는 키를 기본값으로 읽을 수 있어야 한다. 키 삭제·이름 변경은 schema 버전을 올리고 마이그레이션 함수를 둔다.
- 개인 경로·호스트·비밀을 DB 나 문서에 쓰지 않는다. 운영 파일은 로컬 디스크.

## 기록 (2026-10-14)

- 맨 위 「한눈에」 표를 더했다(작전 OPS-219). 본문 표와 규칙은 2026-10-13 판 그대로다.
- 로봇 이름 「AMR2」를 「TB-5」로 바꿨다(재범 규칙 2026-10-13).
- `map_snapshots` 는 1.1 에서 만든 표다. 쓰기·읽기 함수(`record_map_snapshot`·`read_map_snapshots`)는 있지만, 2026-10-14 main 에는 쓰기 함수를 부르는 노드와 읽기 함수를 쓰는 화면이 없다. 관제 웹의 실시간 지도는 `map_snapshot` 노드가 쓰는 지도 파일(`live_map.yaml`)을 직접 읽는다(#209).


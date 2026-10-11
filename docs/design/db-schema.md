---
title: DB 스키마 — 운영 장부·현장 상태·콘솔 계정
status: draft   # 1.0 절과 계정 표는 main 기준, 4절(현장 상태 1.1·1.2)은 구현됨·어휘는 [제안]. 테이블을 바꾸는 PR 이 이 문서를 같이 바꾼다
updated: 2026-10-11
---

# DB 스키마

SQLite 파일 역할은 송신 대기열을 포함해 네 가지이며, 역할마다 별도 파일을 쓴다. **쓰는 쪽은 파일마다 하나**이고, 다른 쪽은 읽기 전용(`mode=ro`, `query_only`)으로만 연다. 파일 경로는 코드에 두지 않고 인자·프로파일로 받으며, 네트워크 공유 폴더에 두지 않는다([implementation-operations](../implementation/implementation-operations.md) 운영 경로 규칙). 테이블을 바꾸는 사람은 상태 감시 담당이고 이 문서를 같은 PR 에서 바꾼다([console-and-db-plan](console-and-db-plan.md) 2절).

| 파일 (역할 값 `metadata.role`) | 쓰는 쪽 | 읽는 쪽 | 코드 | 상태 |
| --- | --- | --- | --- | --- |
| 운영 장부 `operations-ledger` (schema 1.0) | operations (`ReportLedger`) | 콘솔, 협업 | `rescue_operations/ledger.py` | main |
| 현장 상태 `operations-site-state` (schema 1.2, 1.0·1.1 파일은 열 때 올림) | operations (`SiteStateLedger`) | 콘솔 | `rescue_operations/site_state.py` | 1.0 은 main, 1.1 은 4절 |
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

1.1·1.2 에서도 이 세 표는 그대로이고 4절의 표가 더해진다. `metadata`: `schema=1.2`(1.0·1.1 파일은 4절대로 올림), `role=operations-site-state`, `allowed_robot_ids`. 보고 장부와 **다른 파일**이다(재오픈 정책이 달라 한 파일에 섞지 않음, #34).

| 표 | 열 | 뜻 · 규칙 |
| --- | --- | --- |
| `robot_status` | `robot_id TEXT PK`, `robot_boot_id`, `state_sequence INTEGER`, `stamp_ns`, `status_json`, `received_at_ns` | 로봇당 최신 1행. 같은 boot 는 `state_sequence` 가 클 때만 교체(같은 sequence 에 다른 내용이면 `ID_CONTENT_CONFLICT`), 다른 boot 는 `stamp_ns` 가 클 때만. `status_json` 키: robot_id, robot_boot_id, state_sequence, role(SCOUT/SUPPORT/UNKNOWN), stamp_ns, pose{x,y,yaw,frame_id}, motion_state, readiness, battery_percent, battery_stamp_ns, active_task_id, generation, 그리고 선택 키 dock_state(DOCKED/UNDOCKED/UNKNOWN)·segment_index(1차 IDL `RobotStatus`, 2026-10-11; 없는 옛 행은 None 으로 읽음). pose 는 `record_robot_pose` 로 따로 바뀌고(받은 시각 `received_at_ns` 는 그대로) 같은 sequence 의 동일성 비교에서 빠진다 |
| `site_events` | `event_id TEXT PK`, `kind`, `observed_at_ns`, `event_json`, `received_at_ns` | 고정 카메라 사건 추가 전용(`SiteEvent` 를 `asdict` 로 저장). 같은 내용 재전송은 멱등, 다른 내용은 `ID_CONTENT_CONFLICT`. 현재 Python 계약 어휘는 `DISASTER_SIGN`·`EXIT_PASS`(카메라 1대 결정으로 생산자 없음)다. `TARGET_DISAPPEARED_CANDIDATE` 는 #48의 후속 계약 반영 대상이다 |
| `target_states` | `target_id TEXT PK`, `updated_at_ns`, `state_json` | 대상당 최신 1행. `updated_at_ns` 가 클 때만 교체, 같으면 내용이 같아야 함. `state_json` 키: target_id, position{x,y,frame_id,map_session_id,map_revision}, detection_confirmed, rescue_state(계약 `RESCUE_STATES`), rescue_reasons, verdict_id, support_state(NONE/ASSIGNED/ARRIVED/FAILED [제안]), support_robot_id, updated_at_ns |

읽기: `read_robot_status`, `read_site_events(limit)`, `read_target_states`. `exit_pass_count` 는 저장된 서로 다른 `EXIT_PASS` 수다. 카메라 1대 구성의 화면에서는 출구 카운트를 숨기지만, 기존·합성 기록이 있으면 DB 집계는 0이 아닐 수 있다. 구조 4단계(미확정/미구조/지원 중/구출 완료)는 저장하지 않고 콘솔이 합성한다(Q-20, #37 규칙표).

## 3. 콘솔 계정 (`console-accounts`, schema 1.0, main)

콘솔이 소유하는 파일. 운영 장부·현장 상태의 단일 쓰기 규칙과 무관하다. `metadata`: `role=console-accounts`, `schema_version=1.0`.

| 표 | 열 | 뜻 · 규칙 |
| --- | --- | --- |
| `accounts` | `username TEXT PK`, `account_id TEXT UNIQUE`, `display_name`, `email`, `role`, `status`, `password_hash`, `auth_version INTEGER`, `created_at_ns`, `updated_at_ns` | `role` ∈ ADMIN / CUSTOMER, `status` ∈ ACTIVE / INACTIVE. `password_hash` 는 `pbkdf2_sha256$반복$소금$해시`(평문 없음). `account_id` 는 생성마다 새 UUID(같은 이름을 지웠다 다시 만들면 다른 값), `auth_version` 은 비밀번호·역할·상태가 바뀔 때마다 +1 — 세션 쿠키가 둘을 담아 이전 세션이 서버에서 거절된다. 마지막 ACTIVE ADMIN 은 비활성화·강등·삭제 불가 |

읽기·쓰기는 모두 콘솔 프로세스. 관리자 1명은 `python3 -m rescue_console.accounts init` 으로 만든다. 계정 파일과 세션 비밀은 저장소 밖에 둔다(SDD 외부 파일 목록에 추가할 항목).

## 4. 현장 상태 1.1 (구현됨, 2026-10-11) — 2절 표에 더하는 표

[console-and-db-plan](console-and-db-plan.md) 3절의 추가분을 `rescue_operations/site_state.py` 에 구현했다. 2절 표는 그대로 두고 아래 네 표를 더한다. 어휘(`MISSION_PHASES`·`OPERATOR_COMMAND_KINDS`)는 계획 문서에서 옮긴 **[제안]** 이고, 바꿀 때는 이 표와 코드 상수를 같은 PR 에서 바꾼다.

| 표 | 열 | 뜻 · 규칙 |
| --- | --- | --- |
| `error_log` | `seq INTEGER PK`, `at_ns`, `robot_id NULL`, `task_id NULL`, `code`, `detail` | 실패 경로마다 한 행(3.7.4). **추가 전용**이고 같은 실패가 반복되면 새 행이다. `code` 는 C-09 공통 코드(`rescue_contracts.CANONICAL_CODES`)만 받고, `rescue_robot.NAV2_INACTIVE` 같은 세부 원인은 `detail`(1~1024자)에 쓴다. `robot_id` 는 배치 허용 로봇 또는 None(시스템 오류), `task_id` 는 UUID 또는 None |
| `mission_state` (1행) | `singleton INTEGER PK (=1)`, `mode`, `phase`, `trigger_event_id NULL`, `started_at_ns NULL`, `generation NULL`, `updated_at_ns`, 1.2: `target_id NULL`, `target_revision NULL`, `route_revision NULL`, `segment_index NULL`, `segment_count NULL`, `segment_phase NULL` | `mode` ∈ 계약 `SYSTEM_MODES`(S-00: NORMAL / DISASTER / CLEARING — 오탐 해제 중) 와 `MISSION_FINISHED`(phase FINISHED 일 때만, C-10 `SystemState`), `phase` ∈ 계획 1절의 표시용 상태 14개. 출동 칸 여섯은 AMR2 출동이 없으면 모두 None, 있으면 모두 있고 `segment_index` ≤ `segment_count`, `segment_phase` ∈ `SystemState.SEGMENT_PHASE_*` 7개. `generation` 은 `SystemState` 에 없어 None 일 수 있다. `trigger_event_id`·`started_at_ns` 는 짝으로만 있다. `updated_at_ns` 가 클 때만 교체, 같으면 내용이 같아야 함(`ID_CONTENT_CONFLICT`). 판정은 coordinator 가 하고 이 행은 화면 표시용이다 |
| `operator_commands` | `command_id TEXT PK`, `kind`, `operator_id`, `robot_id NULL`, `target_id NULL`, `segment_index NULL`, `route_revision NULL`, `requested_at_ns`, `accepted INTEGER`, `result NULL`, `reason NULL` | 콘솔 명령 한 건과 그 응답, **거절도 기록**. `kind` 는 계획 4절의 9종. 거절(`accepted=0`)이면 `reason` 필수. 같은 ID 같은 내용은 멱등, 다른 내용은 `ID_CONTENT_CONFLICT`. 관문 판정은 operations API 몫이고 이 표는 요청·응답만 남긴다 |
| `map_snapshots` | `snapshot_id TEXT PK`, `robot_id`, `revision`, `path`, `sha256`, `at_ns` | 지도 이미지 한 장(IF-084). 이미지 bytes 는 DB 에 넣지 않고 `path`·`sha256`(소문자 64자)만 둔다. 같은 ID 같은 내용은 멱등 |

올리기 규칙: `metadata.schema` 는 `1.2` 이다. 쓰는 쪽(`SiteStateLedger`)이 1.0 파일을 열면 **같은 트랜잭션**에서 빠진 표를 만들고, 1.1 파일을 열면 `mission_state`(1행)를 1.2 모양으로 다시 만들어 기존 행을 옮긴 뒤(SQLite 는 NOT NULL 을 그 자리에서 풀 수 없음) schema 를 `1.2` 로 올린다. 다른 표와 열은 지우지 않는다. schema 말고 role·허용 로봇이 다르면 지금처럼 `POLICY_MISMATCH` 이고 파일을 바꾸지 않는다. 읽기 함수는 최신 schema 파일만 받으므로, 이전 파일은 operations 가 한 번 연 뒤에 읽힌다. 운영 PC 의 기존 파일은 열기 전에 복사본을 남긴다.

읽기: `read_error_log(limit)`·`read_operator_commands(limit)`·`read_map_snapshots(limit)` 는 최신 순(`limit` 은 최근 N건), `read_mission_state` 는 `mission` 하나(기록 전에는 None).

보류 — JSON 키 추가: `robot_status.status_json` 의 `dock_state`·`segment_index` 는 1차 IDL `RobotStatus` 에 맞춰 선택 키로 넣었다(2절, 2026-10-11). `target_states.state_json` 의 `segment_phase`·`candidate_event_id` 는 아직 넣지 않았다. 지금 검증(`schema.exact`)은 키 집합이 정확히 맞아야 해서, 키를 더하면 발행 쪽(operations ROS 어댑터)과 읽는 쪽(콘솔)이 같이 바뀌어야 한다. 넣을 때는 5절대로 읽는 쪽이 없는 키를 기본값으로 읽게 한다.

## 5. 바꿀 때 지키는 것

- 쓰는 쪽은 파일마다 하나. 콘솔은 장부·현장 상태에 쓰지 않는다.
- 같은 ID 재전송은 같은 내용이면 멱등, 다른 내용이면 거절. 오래된 것(낮은 sequence·이른 시각)은 조용히 버린다(`False` 반환).
- JSON 열의 키를 더할 때는 읽는 쪽이 없는 키를 기본값으로 읽을 수 있어야 한다. 키 삭제·이름 변경은 schema 버전을 올리고 마이그레이션 함수를 둔다.
- 개인 경로·호스트·비밀을 DB 나 문서에 쓰지 않는다. 운영 파일은 로컬 디스크.

---
title: DB 스키마 — 운영 장부·현장 상태·콘솔 계정
status: draft   # 1.0 절은 main 의 코드 그대로, 1.1 절과 계정 표는 [제안]·열린 PR 기준. 테이블을 바꾸는 PR 이 이 문서를 같이 바꾼다
updated: 2026-10-10
---

# DB 스키마

SQLite 파일 세 개를 쓴다. **쓰는 쪽은 파일마다 하나**이고, 다른 쪽은 읽기 전용(`mode=ro`, `query_only`)으로만 연다. 파일 경로는 코드에 두지 않고 인자·프로파일로 받으며, 네트워크 공유 폴더에 두지 않는다([implementation-operations](../implementation/implementation-operations.md) 운영 경로 규칙). 테이블을 바꾸는 사람은 상태 감시 담당이고 이 문서를 같은 PR 에서 바꾼다([console-and-db-plan](console-and-db-plan.md) 2절).

| 파일 (역할 값 `metadata.role`) | 쓰는 쪽 | 읽는 쪽 | 코드 | 상태 |
| --- | --- | --- | --- | --- |
| 운영 장부 `operations-ledger` (schema 1.0) | operations (`ReportLedger`) | 콘솔, 협업 | `rescue_operations/ledger.py` | main |
| 현장 상태 `operations-site-state` (schema 1.0 → 1.1 [제안]) | operations (`SiteStateLedger`) | 콘솔 | `rescue_operations/site_state.py` | main (1.0), 1.1 은 4절 |
| 콘솔 계정 `console-accounts` (schema 1.0) | 콘솔 (`AccountStore`) | 콘솔 | `rescue_console/accounts.py` | main (#50) |
| 보고 outbox (장부와 같은 파일 또는 생산자 쪽 파일) | 보고 생산자 (`ReportOutbox`) | 생산자 | `rescue_operations/ledger.py` | main |

테이블 사이 관계를 그림으로 보려면 [운영 DB 파일과 테이블 관계도](../process/ops-db-tables.md)(웹마스터)를 본다. 표·열 정의의 기준은 이 문서와 코드다.

공통: 모든 파일에 `metadata(key TEXT PRIMARY KEY, value TEXT)` 표가 있고 `role`·`schema`(또는 `schema_version`) 를 담는다. 역할·스키마·정책이 다른 파일을 열면 `POLICY_MISMATCH`, sqlite 오류는 `STORAGE_ERROR` 로 끝나며 조용히 새 파일을 만들지 않는다. 시각은 전부 정수 나노초(`*_ns`), JSON 열은 `canonical()`(키 정렬·공백 없음) 직렬화다.

## 1. 운영 장부 (`operations-ledger`, schema 1.0)

`metadata`: `schema=1.0`, `role=operations-ledger`, `evidence_policy`(증거 요구 정책), `allowed_robot_ids`(허용 로봇 ID JSON). 다시 열 때 하나라도 다르면 거절.

| 표 | 열 | 뜻 · 규칙 |
| --- | --- | --- |
| `reports` | `report_id TEXT PK`, `payload_sha256`, `payload_json`, `envelope_json`, `state`, `revision INTEGER`, `receipt_id`, `received_at_ns`, `persisted_at_ns NULL`, `error_code NULL` | 구조 보고 한 건. `state` ∈ RECEIVED / PENDING_EVIDENCE / PERSISTED. 같은 `report_id` 재전송은 payload 해시가 같으면 멱등(같은 receipt), 다르면 `ID_CONTENT_CONFLICT`. 필수 증거가 다 저장되기 전엔 PERSISTED 가 아니다 |
| `evidence` | `evidence_id TEXT PK`, `ref_json`, `content BLOB`, `stored_at_ns` | 증거 원 bytes. `ref_json` 의 sha256 과 bytes 해시가 다르면 거절 |
| `outbox` (생산자 쪽) | `report_id TEXT PK`, `payload_sha256`, `payload_json`, `state`, `attempts INTEGER`, `receipt_json NULL`, `last_error NULL` | 아직 PERSISTED 영수증을 못 받은 보고. `state` 는 넣을 때 `QUEUED`, 보내다 실패하면 `RETRY_PENDING`, 영수증을 받으면 그 영수증의 상태(`RECEIVED` / `PENDING_EVIDENCE` / `PERSISTED`)로 바뀐다. `PERSISTED` 가 아닌 행만 다시 보낸다. 재전송은 같은 payload 로만 |

읽기: `read_reports(db_path, limit, report_id)` — 보고 공개 필드만(payload·state·receipt·시각).

## 2. 현장 상태 (`operations-site-state`, schema 1.0, main)

`metadata`: `schema=1.0`, `role=operations-site-state`, `allowed_robot_ids`. 보고 장부와 **다른 파일**이다(재오픈 정책이 달라 한 파일에 섞지 않음, #34).

| 표 | 열 | 뜻 · 규칙 |
| --- | --- | --- |
| `robot_status` | `robot_id TEXT PK`, `robot_boot_id`, `state_sequence INTEGER`, `stamp_ns`, `status_json`, `received_at_ns` | 로봇당 최신 1행. 같은 boot 는 `state_sequence` 가 클 때만 교체(같은 sequence 에 다른 내용이면 `ID_CONTENT_CONFLICT`), 다른 boot 는 `stamp_ns` 가 클 때만. `status_json` 키: robot_id, robot_boot_id, state_sequence, role(SCOUT/SUPPORT/UNKNOWN), stamp_ns, pose{x,y,yaw,frame_id}, motion_state, readiness, battery_percent, battery_stamp_ns, active_task_id, generation |
| `site_events` | `event_id TEXT PK`, `kind`, `observed_at_ns`, `event_json`, `received_at_ns` | 고정 카메라 사건 추가 전용(`SiteEvent` 를 `asdict` 로 저장). 같은 내용 재전송은 멱등, 다른 내용은 `ID_CONTENT_CONFLICT`. `kind` 는 계약 어휘(`DISASTER_SIGN`, `TARGET_DISAPPEARED_CANDIDATE`(#48), `EXIT_PASS`(카메라 1대 결정으로 생산자 없음)) |
| `target_states` | `target_id TEXT PK`, `updated_at_ns`, `state_json` | 대상당 최신 1행. `updated_at_ns` 가 클 때만 교체, 같으면 내용이 같아야 함. `state_json` 키: target_id, position{x,y,frame_id,map_session_id,map_revision}, detection_confirmed, rescue_state(계약 `RESCUE_STATES`), rescue_reasons, verdict_id, support_state(NONE/ASSIGNED/ARRIVED/FAILED [제안]), support_robot_id, updated_at_ns |

읽기: `read_robot_status`, `read_site_events(limit)`, `read_target_states`. `exit_pass_count` 는 서로 다른 `EXIT_PASS` 수이며 카메라 1대 구성에서는 항상 0 이라 화면에서 숨긴다. 구조 4단계(미확정/미구조/지원 중/구출 완료)는 저장하지 않고 콘솔이 합성한다(Q-20, #37 규칙표).

## 3. 콘솔 계정 (`console-accounts`, schema 1.0, main)

콘솔이 소유하는 파일. 운영 장부·현장 상태의 단일 쓰기 규칙과 무관하다. `metadata`: `role=console-accounts`, `schema_version=1.0`.

| 표 | 열 | 뜻 · 규칙 |
| --- | --- | --- |
| `accounts` | `username TEXT PK`, `account_id TEXT UNIQUE`, `display_name`, `email`, `role`, `status`, `password_hash`, `auth_version INTEGER`, `created_at_ns`, `updated_at_ns` | `role` ∈ ADMIN / CUSTOMER, `status` ∈ ACTIVE / INACTIVE. `password_hash` 는 `pbkdf2_sha256$반복$소금$해시`(평문 없음). `account_id` 는 생성마다 새 UUID(같은 이름을 지웠다 다시 만들면 다른 값), `auth_version` 은 비밀번호·역할·상태가 바뀔 때마다 +1 — 세션 쿠키가 둘을 담아 이전 세션이 서버에서 거절된다. 마지막 ACTIVE ADMIN 은 비활성화·강등·삭제 불가 |

읽기·쓰기는 모두 콘솔 프로세스. 관리자 1명은 `python3 -m rescue_console.accounts init` 으로 만든다. 계정 파일과 세션 비밀은 저장소 밖에 둔다(SDD 외부 파일 목록에 추가할 항목).

## 4. 현장 상태 1.1 [제안] — 상태 감시 담당이 구현하며 이 표를 확정한다

[console-and-db-plan](console-and-db-plan.md) 3절의 추가분. 2절 표에 더하는 것이고 기존 열은 지우지 않는다.

| 표 | 열 (제안) | 쓰는 시점 | 읽는 쪽 |
| --- | --- | --- | --- |
| `mission_state` (1행) | `mode`(NORMAL/DISASTER), `phase`(계획 1절 상태), `trigger_event_id`, `started_at_ns`, `generation`, `updated_at_ns` | 사건·승인·완료마다 operations 가 갱신 | 콘솔 배지·흐름 |
| `operator_commands` | `command_id TEXT PK`, `kind`, `operator_id`, `robot_id`, `target_id`, `segment_index`, `route_revision`, `requested_at_ns`, `accepted INTEGER`, `result`, `reason` | `POST /api/operator-commands` 마다, 거절도 기록 | 콘솔 명령 이력, 인수 시험 |
| `error_log` | `at_ns`, `robot_id`, `task_id`, `code`, `detail` | 실패 경로마다 | 콘솔 오류 이력(3.7.4) |
| `map_snapshots` | `snapshot_id TEXT PK`, `robot_id`, `revision`, `path`, `sha256`, `at_ns` | `PUT /api/maps/{id}` 마다 | 콘솔 지도 |
| `robot_status.status_json` 추가 키 | `dock_state`, `segment_index` (battery·motion_state·readiness 는 이미 있음) | 로봇 상태마다 | 콘솔 로봇 표 |
| `target_states.state_json` 추가 키 | `segment_phase`(AWAITING_APPROVAL 등), `candidate_event_id` | 승인·후보마다 | 콘솔 대상 표·버튼 활성화 |

규칙(제안): `metadata.schema` 를 `1.1` 로 올리고 1.0 파일은 열 때 표를 추가해 올린다(열 삭제 없음). 명령 ID 는 UUID, 같은 ID 재전송은 멱등. 시각·좌표 키 이름은 계약(C-03·C-04) 그대로 쓴다.

## 5. 바꿀 때 지키는 것

- 쓰는 쪽은 파일마다 하나. 콘솔은 장부·현장 상태에 쓰지 않는다.
- 같은 ID 재전송은 같은 내용이면 멱등, 다른 내용이면 거절. 오래된 것(낮은 sequence·이른 시각)은 조용히 버린다(`False` 반환).
- JSON 열의 키를 더할 때는 읽는 쪽이 없는 키를 기본값으로 읽을 수 있어야 한다. 키 삭제·이름 변경은 schema 버전을 올리고 마이그레이션 함수를 둔다.
- 개인 경로·호스트·비밀을 DB 나 문서에 쓰지 않는다. 운영 파일은 로컬 디스크.

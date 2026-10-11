---
title: PR 9 로컬 관제 보고·증거 저장·outbox 구현
status: implemented-local-only
updated: 2026-10-08
---

# PR 9 로컬 관제 보고·증거 저장·outbox 구현

## 목적과 완료 범위

보고를 받았다는 사실을 영속 보고 완료와 구분한다. 동일 report_id 재전송, ops 재시작, 업로드 실패, DB 쓰기 실패, 저장 뒤 receipt 유실을 처리하는 새 Python 표준 라이브러리 구현이다. [계약 C-02·C-05·C-08](../design/contracts.md)과 [시험 CT-12](../process/acceptance-plan.md)를 로컬 SQLite로 연결한다. 원본 실행 코드·사진·모델을 복사하지 않았다.

이번 구현의 `PERSISTED`는 **operations DB의 보고와 선택한 증거 정책이 로컬 transaction으로 저장됨**이다. 송신 측은 해당 ID와 payload 해시의 PERSISTED receipt를 받아 자기 outbox에도 기록한 뒤 재전송 대상에서 제외한다. ROS/DDS 전달, 실제 센서 관측, 로봇 정지, 4-PC 배포, 사진의 진위·영상 디코딩, 시스템 전원 손실 내구성은 이 로컬 시험으로 입증하지 않는다.

공개 네트워크 서버는 추가하지 않았다. `ReportTransport` Protocol과 `LedgerTransport` callable이 API 경계를 제공한다. 원격 어댑터는 인증·전송 크기 상한·연결 및 응답 timeout을 구현하고 아래 고정 스키마와 receipt 검증을 유지해야 한다. 어댑터 호출 1회가 끝나지 않는 경우를 로컬 outbox가 강제로 중단하지는 않는다.

## 파일과 책임

| 파일 | 책임 |
| --- | --- |
| `src/rescue_operations/rescue_operations/schema.py` | 고정 report/envelope/evidence 스키마와 입력 상한·시각·ID 검증, 의미 payload 정규화 해시 |
| `src/rescue_operations/rescue_operations/ledger.py` | operations 단독 소유 SQLite, 증거 bytes, receipt 전이, 송신 측 별도 outbox, typed callable, 로컬 읽기 snapshot |
| `src/rescue_operations/rescue_operations/__main__.py` | 기존 DB를 읽기 전용으로 열어 JSON 내보내기. 성공 0, 입력·저장소 오류 2 |
| `tests/test_operations.py` | 재시작, 실제 DB rollback, 증거 실패, receipt 유실·역순·잘못된 ID, 중복·동시 요청, CLI 오류 |

## 스키마 1.0

미선언 필드는 거절한다. `schema_version`은 현재 정확히 `1.0`만 허용하며 minor 확장을 암묵적으로 수용하지 않는다. ID는 소문자 표준 UUID 문자열, generation/revision/sequence는 bool을 제외한 uint64다. wall timestamp는 양수 uint64 **ns**다. 좌표는 유한한 m 값이고 정규화 시 float로 변환한다. NaN/Inf·누락 ID·잘못된 UUID·음수/0 timestamp·역순 시각을 거절한다. 출력 저장 시각은 operations 호스트의 `time.time_ns()`이며 다른 호스트 시계 동기화의 증거가 아니다.

### 의미 payload: 모든 필드가 해시에 포함됨

| 필드 | 형식과 의미 |
| --- | --- |
| `schema_version`, `report_id`, `target_id` | `1.0`, 불변 보고 ID, 사건 ID |
| `observation_id`, `source_robot`, `source_camera`, `source_boot_id` | 원 발견 관측 ID, 프로파일 식별자 형식의 로봇 ID, 원 카메라 이름, 원 센서/관측 생성 프로세스 UUID. 중계 publisher로 덮어쓰지 않음 |
| `clock_domain_id` | 보존하는 관측의 시간 기준 이름. 이 이름 자체로 동기화·최신성을 승인하지 않음 |
| `robot_id`, `task_id`, `generation` | 수행 로봇, 임무 ID와 세대. 원 발견 로봇과 다를 수 있음 |
| `observation_result` | `DETECTED`만 지원 |
| `confirmation_result` | `CONFIRMED` 또는 `UNCONFIRMED` |
| `confirmation_observation_id`, `confirmed_at_ns` | CONFIRMED이면 원 발견과 다른 관측 UUID와 시각. UNCONFIRMED이면 둘 다 null |
| `approach_result` | `APPROACH_COMPLETE`, `UNREACHABLE`, `FAILED`, `CANCELED`, `NOT_ATTEMPTED`. 의료·물리적 구조 결과 없음 |
| `position` | null 또는 아래 위치 객체. APPROACH_COMPLETE이면 위치와 CONFIRMED가 필수 |
| `observed_at_ns`, `reported_at_ns` | 원 발견과 최초 보고 생성 시각. `observed <= confirmed <= reported`; 재전송 때 바꾸지 않음 |
| `reason` | 1~1024자. 관측한 결과·실패 사유 |
| `evidence_refs` | 아래 증거 객체 최대 8개. evidence_id 중복 금지 |
| `model_id`, `model_sha256`, `deployment_version` | 모델 식별자·소문자 SHA-256과 배포 버전. 다른 source 의미를 새 ID에 연결하는 provenance |

`position`의 정확한 필드는 `x`, `y`, `z`, `frame_id`, `map_session_id`, `map_revision`, `transform_revision`, `authority_id`다. revision을 새롭게 검증하거나 좌표를 변환하는 기능은 PR 6의 책임이다. operations는 보고된 좌표 참조를 보존하며, 이를 주행 입력으로 되돌려 발행하지 않는다. 이 위치는 보고용 최종 위치 하나다. 모든 원 관측의 좌표 provenance를 합치는 TargetEstimate 저장소를 대신하지 않는다.

기본 문자열 상한은 256자, UUID 36자, SHA-256 64자다. 정규화 JSON은 UTF-8 65,536 bytes 이하이다. 객체 키는 정렬하고 공백을 제거하며 숫자 좌표를 정규화한다. 배열 순서는 의미 payload의 일부다. 전송 성공에 따라 필드·evidence_refs 순서를 바꾸지 않는다. 수정된 보고는 새 report_id를 사용한다. 현재 같은 target/task에 대한 서로 다른 report_id를 자동으로 하나의 최종 결과로 병합하거나 성공률로 집계하지 않는다.

### 전송 envelope: 의미 payload 해시에서 전부 제외

`schema_version`, `publisher_id`, `publisher_boot_id`, `sequence`, `published_at_ns`만 허용한다. boot는 프로세스별 UUID, sequence는 그 boot 내 uint64이고, 발행 시각은 양수 ns다. outbox 재전송은 이 envelope만 바꿀 수 있다. 저장소는 첫 envelope를 감사 정보로 보존하며, 동일 ID/동일 payload의 이후 envelope로 원본 보고를 덮어쓰지 않는다. 역순·중복 보고를 받아 복구해야 하므로 이 수신기는 envelope sequence 역순을 이유로 보고를 버리지 않는다. 센서 watchdog·lease가 요구하는 live sequence 정책과 구별한다.

### evidence_ref와 byte 저장

정확한 필드는 `evidence_id`, `sha256`, `media_type`, `observation_id`, `acquired_at_ns`다. media_type은 image/png 또는 image/jpeg다. URI나 다른 PC의 로컬 파일 경로는 허용하지 않는다. evidence ID가 API 조회의 키다. 참조의 observation ID와 획득 시각은 보고에 기록된 최초 관측 또는 재확인 관측에 정확하게 대응해야 한다. 오래된 사진에 새 확인 시각만 붙이는 입력은 거절한다.

`store_evidence(ref, content)`는 1~4,194,304 bytes의 실제 bytes를 받고 SHA-256을 직접 계산한 뒤 metadata와 BLOB을 같은 SQLite transaction에 저장한다. 해시 불일치·DB 실패이면 성공 결과를 돌려주지 않는다. 같은 evidence_id의 다른 metadata는 충돌이다. 이 함수는 영상 디코딩·카메라 서명·실제 인물 여부를 검증하지 않는다. MIME 선언과 hash는 그 검증을 대신하지 않는다. 시험 bytes는 문장 fixture이며 인물 사진은 포함하지 않았다.

사진 필수 여부 Q-07은 결정하지 않았다. 생성자가 다음 정책 중 하나를 명시하고 DB metadata에 보존한다. 재오픈 때 다른 정책을 적용하면 `POLICY_MISMATCH`로 거절한다.

| 정책 | PERSISTED의 추가 조건 |
| --- | --- |
| `OPTIONAL` | refs가 없으면 사진 없이 저장 가능. 선언한 refs가 있으면 모두 실제 저장되어야 함 |
| `OBSERVATION` | 원 발견 observation_id의 사진 참조가 최소 1개 필요. 선언한 모든 refs가 실제 저장되어야 함 |
| `CONFIRMATION` | confirmation_observation_id의 사진 참조가 최소 1개 필요. 선언한 모든 refs가 실제 저장되어야 함. 재확인하지 못한 보고는 이 엄격한 정책에서 PENDING_EVIDENCE 유지 |

실패 보고도 유실 없이 보존한다. CONFIRMATION 정책에서 UNCONFIRMED 보고는 완료될 수 없으므로 실패 보고 완료까지 요구하는 운영에는 정책을 그대로 채택하면 안 된다. 현재 구현은 사진 정책의 최종 결정을 숨기거나 누락 증거를 있다고 간주하지 않는다. 필요한 참조를 보고에 빠뜨린 경우에는 불변 report_id를 수정하지 않고 올바른 refs를 가진 새 보고를 발행해야 한다.

## 공개 API와 상태 전이

| 호출 | 의미 |
| --- | --- |
| `ReportLedger(path, evidence_policy=..., allowed_robot_ids=...)` | operations 호스트의 로컬 파일. :memory:·디렉터리·NUL 경로 거절. 부모 디렉터리는 먼저 준비 |
| `receive(payload, envelope) -> Receipt` | 검증된 불변 보고 저장 후 RECEIVED. 재전송이면 이미 저장된 가장 최근 receipt. 같은 ID의 다른 의미는 ID_CONTENT_CONFLICT |
| `store_evidence(ref, bytes) -> dict` | hash 검증 후 실제 증거 저장. 보고 상태를 암묵적으로 전진시키지 않음 |
| `persist(report_id) -> Receipt` | 필수 payload·증거 확인 후 PERSISTED, 부족하면 PENDING_EVIDENCE. 저장 실패는 예외이며 PERSISTED receipt 없음 |
| `get(report_id)`, `list_reports(limit=100)` | payload·receipt·첫 envelope·수신 시각. 목록 상한 1~1000 |
| `ReportOutbox(path, allowed_robot_ids=...).enqueue(payload)` | 송신 PC의 별도 로컬 DB에 QUEUED 보존. 동일 ID 의미 충돌 거절 |
| `deliver(report_id, envelope, transport)` | 시도 횟수를 먼저 저장하고 callable 1회 호출. 유효한 PERSISTED receipt만 완료. 전송 실패/잘못된 receipt는 RETRY_PENDING·오류 기록·None 반환 |
| `pending(limit=100)`, `get(report_id)` | 재시작 뒤 재전송 목록과 상태/시도 횟수/receipt 조회 |
| `LedgerTransport(ledger)` | 프로세스 내부 receive→persist API 어댑터. 네트워크 통신을 한 것으로 주장하지 않음 |
| `read_reports(path, limit=100, report_id=None)` | 기존 operations DB를 SQLite mode=ro/query_only로 읽음. 사진 BLOB은 내보내지 않음 |

`Receipt`는 `receipt_id`, `report_id`, `payload_sha256`, `status`, `revision`, `persisted_at_ns`, `error_code`의 frozen dataclass이며 `to_dict()`로 JSON 직렬화한다. 상태 변경마다 receipt ID와 revision이 바뀐다. 동일 상태의 중복 호출은 같은 receipt다. 수신기 상태는 RECEIVED→PENDING_EVIDENCE→PERSISTED 또는 RECEIVED→PERSISTED로 진행하고 PERSISTED를 후퇴시키지 않는다.

송신기는 receipt의 report ID·payload hash·상태·revision·영속 시각을 확인한다. 낮은 revision의 늦은 receipt나 늦은 실패로 이미 확인한 PERSISTED를 덮어쓰지 않는다. remote commit 뒤 응답만 유실되거나 송신 outbox의 receipt 쓰기가 실패하면, 다음 시도에서 같은 report_id를 보내 기존 receipt를 회수한다. DB 파일 자체 삭제/손상과 증거 백업·보존 기간·disk quota 관리·outbox 자동 retry scheduler는 후속 운영 작업이다. 자동 무한 재시도는 없다. 보존한 보고는 오래되었다는 이유로 새로운 관측이 되지 않는다.

## 실행 가능한 로컬 예시

아래 값은 synthetic fixture다. 실제 로봇·관측 성공의 예가 아니다. 부모 폴더가 있고 기존 DB가 다른 정책으로 만들어지지 않은 로컬 경로에서 실행한다.

```python
import uuid
from rescue_operations.ledger import ReportLedger, ReportOutbox, LedgerTransport

def uid():
    return str(uuid.uuid4())

payload = {
    "schema_version": "1.0", "report_id": uid(), "target_id": uid(),
    "observation_id": uid(), "source_robot": "robot_a", "source_camera": "front_rgb",
    "source_boot_id": uid(), "clock_domain_id": "offline-fixture",
    "robot_id": "robot_b", "task_id": uid(), "generation": 1,
    "observation_result": "DETECTED", "confirmation_result": "UNCONFIRMED",
    "confirmation_observation_id": None, "approach_result": "UNREACHABLE",
    "position": None, "observed_at_ns": 100, "confirmed_at_ns": None,
    "reported_at_ns": 300, "reason": "Synthetic route blocked",
    "evidence_refs": [], "model_id": "synthetic", "model_sha256": "a" * 64,
    "deployment_version": "offline-fixture",
}
envelope = {
    "schema_version": "1.0", "publisher_id": "coordinator", "publisher_boot_id": uid(),
    "sequence": 0, "published_at_ns": 400,
}
allowed_robot_ids = ("robot_a", "robot_b")  # 예시. 실제 값은 검증된 deployment.robots에서 주입
with ReportLedger("operations.sqlite3", evidence_policy="OPTIONAL", allowed_robot_ids=allowed_robot_ids) as ledger:
    with ReportOutbox("sender.sqlite3", allowed_robot_ids=allowed_robot_ids) as outbox:
        outbox.enqueue(payload)
        receipt = outbox.deliver(payload["report_id"], envelope, LedgerTransport(ledger))
        assert receipt.status == "PERSISTED"
        assert ledger.get(payload["report_id"])["payload"]["approach_result"] == "UNREACHABLE"
```

PERSISTED와 APPROACH_COMPLETE는 서로 다른 축이다. 위 예시는 실패 결과를 영속 보고한 것이며 접근 성공으로 집계하지 않는다.

## 소유권·transaction·배포 경계

operations만 operations DB를 쓴다. 네 PC가 파일을 NFS/SMB 등으로 공유하지 않는다. 이 코드가 네트워크 파일 시스템을 자동 탐지하는 것은 아니므로 운영 경로를 로컬 디스크로 설정해야 한다. 여러 호출은 프로세스 내 RLock과 SQLite BEGIN IMMEDIATE로 직렬화하고 다른 연결의 write lock은 최대 약 1초 기다린 뒤 STORAGE_ERROR로 반환한다. SQLite WAL와 synchronous=FULL을 설정한다. with 블록/close로 연결을 닫고 예외에서 rollback한다. 전송 callable 실행 중에는 DB write transaction을 붙잡지 않는다.

라이브 도메인 어댑터·DDS QoS·웹 로그인·공개 HTTP·실시간 영상 스트림·운영자 명령은 없다. operations API는 현재 로컬 함수 경계이며 4-PC 네트워크 API 완료로 표기하지 않는다. 현 단계의 공개 export는 read-only CLI다. PR 10은 이 callable과 임시 로컬 DB로 소프트웨어 통합을 검증할 수 있다.

## 검증과 한계

```bash
PYTHONPATH=src/rescue_contracts:src/rescue_operations python3 -m unittest discover -s tests -p test_operations.py -v
PYTHONPATH=src/rescue_contracts:src/rescue_operations python3 -m rescue_operations operations.sqlite3 --limit 100
```

실제 실행 증거는 [PR9 로컬 검증 기록](../evidence/pr09-local-validation.md)에 별도로 남긴다. 주요 시험은 DB close/reopen, 두 연결 동시 중복 수신, 보고/증거/receipt 쓰기 실패 rollback, commit 뒤 응답 유실, 송신 receipt DB 쓰기 실패, 필수 증거 누락·hash 불일치, 늦은 receipt, ID 충돌, 잘못된 schema·UUID·NaN·시각, 읽기 전용 CLI다. 외부 네트워크를 열지 않았고 장비·ROS·실기 시험을 실행하지 않았다.

## 원리 참고

고정 원본의 [P4 evidence 관측 시각 매칭](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_sysmon/rokey_sysmon/evidence.py#L24-L104), [P4 가짜 두 로봇 시험](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_sysmon/test/test_integration_fake_robots.py#L1-L52), [AMR 재발행 관측 시각 문제](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/webcam_detector.py#L109-L159)를 선별 근거로 삼았다. SQLite·outbox 코드를 해당 파일에서 이식한 것이 아니다. 과거 가짜 로봇 시험 결과는 이번 2 AMR 실기 증거가 아니다. 기존 저작권·라이선스가 새 구현에 자동 승계되었다고 추정하지 않는다.

## 프로파일 ID와 오류 계약

`source_robot`와 `robot_id`의 형식은 공통 `rescue_contracts.robot_id`로 검사한다. 저장소·outbox 생성자는 검증된 프로파일에서 정확히 두 개의 서로 다른 ID를 `allowed_robot_ids`로 받아야 하며 기본값은 없다. `robot3`/`robot7` 같은 ID도 가능하고, 두 필드 중 하나라도 허용 집합 밖이면 `WRONG_ROBOT`으로 쓰기 전에 거절한다. 허용 집합은 정렬한 불변 tuple로 보존하고 읽기 전용 속성으로 제공한다.

DB metadata에는 허용 ID 집합을 사진 정책과 함께 고정한다. 순서만 다른 같은 집합으로는 재오픈할 수 있고, 집합 변경이나 허용 ID metadata가 없는 이전 DB는 `POLICY_MISMATCH`다. 기존 DB에 현재 설정을 조용히 덮어쓰거나 과거 보고를 바꾸지 않는다. 필요한 데이터 이전은 원본 보존 후 별도 명시적 migration으로 다뤄야 한다. 읽기 전용 export도 저장된 `allowed_robot_ids`를 함께 표시한다.

`OperationsError`는 공통 `RescueError`이며 `.code`는 C-09, `.detail_code`는 `rescue_operations.<원 세부 코드>`다. 예를 들어 저장 실패는 `REPORT_NOT_PERSISTED`/`rescue_operations.STORAGE_ERROR`로 표현한다. `tests/test_review_operations.py`는 사용자 지정 ID, 알 수 없는 원본/수행 로봇, 정책 불변·재오픈 불일치·기존 metadata 부재를 실제 임시 SQLite로 검사한다. 기존 실행 증거·source hash는 수정하지 않는다.

## 관제 화면 읽기 API: 현장 상태 (BRD Part I 3.7.1~3.7.4, 2026-10-08, 1.1 은 2026-10-11)

관제·화면이 읽을 로봇 상태, 현장 사건·출구 카운트, 대상 상태를 `rescue_operations/site_state.py`에 둔다. 보고 장부와는 **다른 SQLite 파일**(metadata `role=operations-site-state`, `schema=1.1`)이다. 쓰는 쪽이 1.0 파일을 열면 같은 트랜잭션에서 빠진 표를 더하고 schema 를 올린다(열 삭제 없음, [db-schema](../design/db-schema.md) 4절). 읽기 함수는 1.1 파일만 받는다. 보고 스키마 1.0과 재오픈 정책은 바꾸지 않는다. 쓰는 쪽은 operations 하나다. 읽는 쪽은 `mode=ro`·`query_only`로 열고, 없는 파일을 만들지 않는다. 단, WAL 모드 DB를 읽기 전용으로 열어도 SQLite가 빈 `-wal`·`-shm` 파일을 만들 수 있다. 본문 파일 바이트는 바뀌지 않는다(관제·화면 확인, 2026-10-08). 시험은 본문 바이트 불변과 없는 DB 미생성만 검사한다.

| 함수 | 반환 | 규칙 |
| --- | --- | --- |
| `SiteStateLedger(path, allowed_robot_ids=...)` | 쓰기 객체 | 허용 로봇 ID는 metadata에 고정되며 바뀌면 `POLICY_MISMATCH` |
| `.record_robot_status(dict)` | 저장했으면 `True` | 로봇당 최신 1행. 같은 boot에서는 sequence가 큰 것만 받는다. 같은 sequence에 다른 내용이면 `ID_CONTENT_CONFLICT`. 다른 boot는 `stamp_ns`가 더 클 때만 받는다(늦게 온 이전 boot 메시지 차단). 배터리 값·시각, 활성 task·generation은 짝으로만 있음 |
| `.record_site_event(SiteEvent)` | 저장했으면 `True` | 사건 ID별 추가만 한다. 같은 내용 재전송은 `False`, 다른 내용은 `ID_CONTENT_CONFLICT` |
| `.record_target_state(dict)` | 저장했으면 `True` | 대상당 최신 1행. `updated_at_ns`가 더 큰 것만 받는다. 보류 사유는 `RECHECK_NEEDED`일 때만 있다. `support_robot_id`는 `support_state=NONE`일 때만 None |
| `.record_error(dict)` | 새 행의 `seq` | 오류 이력(3.7.4), 추가 전용. 키 `at_ns, robot_id, task_id, code, detail`. `code` 는 C-09 `CANONICAL_CODES` 만, 세부 원인은 `detail`. `robot_id`·`task_id` 는 None 가능 |
| `.record_mission_state(dict)` | 저장했으면 `True` | 1행. 키 `mode, phase, trigger_event_id, started_at_ns, generation, updated_at_ns`. `updated_at_ns` 가 큰 것만 받는다. `mode` 는 계약 `SYSTEM_MODES`(CLEARING 포함), `MISSION_PHASES` 는 [제안] |
| `.record_operator_command(dict)` | 저장했으면 `True` | 명령 ID별 추가만, 거절도 기록. `kind` ∈ `OPERATOR_COMMAND_KINDS`([제안]), 거절이면 `reason` 필수. 재전송 멱등 |
| `.record_map_snapshot(dict)` | 저장했으면 `True` | 스냅샷 ID별 추가만. 키 `snapshot_id, robot_id, revision, path, sha256, at_ns`. bytes 는 DB 밖 |
| `read_robot_status(path)` | `{schema_version, robots:[{robot_id, robot_boot_id, state_sequence, role, stamp_ns, pose, motion_state, readiness, battery_percent, battery_stamp_ns, active_task_id, generation, received_at_ns}]}` | 표시용 스냅샷이다. live readiness 증거가 아니다(C-05). frame이 다른 pose끼리 거리를 비교하지 않는다 |
| `read_site_events(path, *, limit=100)` | `{schema_version, exit_pass_count, site_events:[SiteEvent 필드]}` | `exit_pass_count`는 저장된 `EXIT_PASS`의 서로 다른 사건 ID 수이고 `limit`과 무관하다. 트랙 중복 제거는 인식이 하고 여기서 다시 하지 않는다(TS-04). 구출 판정의 `TargetBasis.exit_pass_count`도 이 값을 쓴다 |
| `read_target_states(path)` | `{schema_version, targets:[{target_id, position, detection_confirmed, rescue_state, rescue_reasons, verdict_id, support_state, support_robot_id, updated_at_ns}]}` | 구조 상태 4단계는 화면 표시층이 이 원자료로 합성한다(Q-20). 계약 어휘 `RESCUE_STATES`는 그대로 둔다 |
| `read_error_log(path, *, limit=100)` | `{schema_version, errors:[{seq, at_ns, robot_id, task_id, code, detail}]}` | 최신 순, `limit` 은 최근 N건 |
| `read_mission_state(path)` | `{schema_version, mission}` | 기록 전에는 `mission=None`. 표시용이고 판정은 coordinator |
| `read_operator_commands(path, *, limit=100)` | `{schema_version, commands:[명령 dict, accepted 는 bool]}` | `requested_at_ns` 최신 순 |
| `read_map_snapshots(path, *, limit=100)` | `{schema_version, maps:[스냅샷 dict]}` | `at_ns` 최신 순. 이미지는 `path` 에서 읽고 `sha256` 으로 확인 |

모르는 값은 None이나 `UNKNOWN`으로 둔다. `support_state` 네 값(`NONE`·`ASSIGNED`·`ARRIVED`·`FAILED`)은 관제·화면과 맞춘 화면 쪽 어휘이며 [제안], C-09 오류 코드가 아니다. 보류 사유의 닫힌 집합은 협업·관제 판정 정책 소유라서, 여기서는 형식(중복 없는 식별자)만 검사한다.

후속: `status_json`·`state_json` 의 키 추가(`dock_state`·`segment_index`·`segment_phase`·`candidate_event_id`)는 발행·소비 쪽 확인 뒤로 미뤘다([db-schema](../design/db-schema.md) 4절 「보류」). 판정 이력 영속화와 재시작 복구, ROS 수신 어댑터(누가 언제 `record_*`를 부르는지), 관제 화면 연결은 이 절 밖이다. `tests/test_operations_site_state.py`가 위 규칙을 실제 임시 SQLite로 검사한다.

### 오프라인 리허설: 현장 상태 DB 의 BRD 흐름 (E2, 2026-10-11)

`tests/test_operations_site_flow.py` 가 [console-and-db-plan](../design/console-and-db-plan.md) 1절 흐름(웹캠 1대)을 합성 데이터로 현장 상태 DB 에 차례로 기록하고, 단계마다 operations 읽기 함수로 화면에 나갈 값을 확인한다. 실제 로봇 0대, ROS·카메라·coordination 판정 없음. 언제 재난인지, 출동을 받을지, 구출 완료인지는 coordination 몫이라 **정해진 결과를 넣는다**. 이 리허설이 검증하는 것은 DB 와 읽기 API 가 흐름을 빠짐없이 담고 보여 주는지다. E3·E4 는 NOT_RUN 이다.

| 확인 | 결과 |
| --- | --- |
| 17단계: 평시 → 지진 감지 → AMR1 정찰 → 지도 → 발견(미확정) → 확정 전 출동 거절 → 확정 → 출동 승인 → 구간 정지 → 오류 → 계속 → 도착 → 구조 완료 후보 → 재확인 → 구출 완료 → 복귀 → 종료 | 단계마다 임무 단계가 기대대로 바뀌고 모든 기록이 저장됨 |
| 교차 확인 | 배지의 트리거 = 저장된 `DISASTER_SIGN`(시각 같음), 후보 사건의 `target_id` = 구출 완료 대상, 승인 명령의 대상·로봇 = 화면의 지원 로봇 |
| operations 재시작 뒤 전체 재전송 | 오류 이력만 늘고(추가 전용) 나머지 표는 그대로 |
| 늦게 온 스냅샷 | 임무·대상·로봇 상태를 되돌리지 않음 |
| 오탐 해제(S-00 DISASTER → CLEARING → NORMAL) | 리허설에서 처음 실패(`mode` 가 NORMAL/DISASTER 만 받음) → 계약 `SYSTEM_MODES` 로 고침 |

리허설이 드러낸 빈 곳 (결정 대기, 코드 변경 없음):

1. 카메라 감시 설정 `SET_CAMERA_WATCH`(C-11 2행)를 명령 이력에 남길 수 없다. `OPERATOR_COMMAND_KINDS` 와 1차 IDL 의 차이(#81 검토 ①)이며, 시험은 `expectedFailure` 로 둔다. 작전이 목록을 정하면 지운다.
2. 화면의 「구간 i/N」 을 담을 곳이 없다. 구간 번호는 운영자 명령에만 있고 임무·로봇 상태에는 없다(보류한 `segment_index`).
3. 도킹 상태를 담을 곳이 없다(보류한 `dock_state`).
4. 후보와 대상의 연결은 `site_events` 의 후보 `target_id` 로 이미 읽을 수 있다. 보류한 `target_states.state_json.candidate_event_id` 는 이 연결만을 위해서라면 필요 없을 수 있다 [제안].
5. 오류 이력에는 재전송 구분 키가 없어, 어댑터가 재시작 뒤 같은 오류를 다시 보내면 행이 늘어난다. 어댑터는 오류를 다시 보내지 않는다(설계 그대로).
6. 콘솔의 모드 표시(`view.py` `MODES`)에 CLEARING 이 없다. 콘솔이 `read_mission_state` 를 읽기 시작할 때 필요하다(관제·화면).

`FLOW` 는 데이터, `apply` 하나만 DB 에 쓴다. [통합 리허설](implementation-integration.md)의 `offline_demo`(기반 레인)로 옮길 때는 이 둘을 가져가 summary 검사로 잇고, coordination 코어가 main 에 들어오면 정해진 결과 자리를 코어 출력으로 바꾼다.

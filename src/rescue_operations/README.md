# rescue_operations

ROS 없이 실행할 수 있는 SQLite 보고 저장소, 증거 bytes 저장, 송신 outbox와 로컬 JSON 내보내기를 제공한다. C-02·C-05·C-08 및 CT-12의 로컬 구현이다. 사진 필수 여부는 확정하지 않고 저장소 생성자가 `evidence_policy`를 명시하게 한다.

```python
from rescue_operations.ledger import ReportLedger, ReportOutbox, LedgerTransport

allowed_robot_ids = ("robot_a", "robot_b")  # 예시. 실제 값은 검증된 deployment.robots에서 주입
with ReportLedger("operations.sqlite3", evidence_policy="OPTIONAL", allowed_robot_ids=allowed_robot_ids) as ledger:
    with ReportOutbox("sender.sqlite3", allowed_robot_ids=allowed_robot_ids) as outbox:
        outbox.enqueue(payload)  # schema 1.0의 검증된 보고 dict
        receipt = outbox.deliver(payload["report_id"], envelope, LedgerTransport(ledger))
        # RECEIVED/PENDING_EVIDENCE는 보고 완료가 아니다.
```

`payload`와 `envelope`의 고정 필드, 예시, 실패 의미는 [구현 문서](../../docs/implementation/implementation-operations.md)에 있다. `OPTIONAL`은 예시 선택이며 운영 사진 정책의 결정이 아니다. 필요한 증거가 선언됐으면 OPTIONAL에서도 실제 bytes 저장을 기다린다.

```bash
PYTHONPATH=src/rescue_contracts:src/rescue_operations python3 -m rescue_operations operations.sqlite3 --limit 100
# 설치한 패키지:
rescue-report-export operations.sqlite3 --report-id <report-uuid>
```

CLI는 operations 호스트의 기존 DB를 읽어 JSON을 출력한다. 없는 DB를 만들거나 보고를 변경하지 않는다. `PERSISTED`는 보고와 선택한 정책의 증거가 로컬 SQLite transaction으로 저장됐다는 뜻이며, 접근의 물리적 성공이나 사람 구조·생존을 판단하지 않는다.

operations DB의 소유자는 operations 호스트다. 다른 PC는 DB 파일을 마운트하거나 직접 쓰지 않는다. 송신 측 outbox는 별도 로컬 DB이고 API를 통해 같은 보고 ID를 재전송한다. 보고 장부의 API 어댑터는 프로세스 내부 callable이다. 현장 상태 DB 에는 ROS 구독 노드 `db_writer`(로봇 상태·위치·고정 카메라 사건·임무 상태·대상 목록, [구현 문서](../../docs/implementation/implementation-operations.md) 「DB 쓰기 어댑터」)가 있다. 원격 HTTP·인증·실기·4-PC 통신은 아직 구현하거나 검증하지 않았다. 다른 시스템의 Nav2 goal이나 속도 명령을 발행하지 않는다.

원본 코드를 복사하지 않은 새 구현이다. 패키지 버전 `0.0.0`, `LicenseRef-Unspecified`, 미지정 maintainer는 기존 기초 구조의 미결정 메타데이터를 유지한다. 배포 권한이나 라이선스 허가를 뜻하지 않는다.

## 프로파일 ID와 오류 계약

`source_robot`와 `robot_id`의 형식은 공통 `rescue_contracts.robot_id`로 검사한다. 저장소·outbox 생성자는 검증된 프로파일에서 정확히 두 개의 서로 다른 ID를 `allowed_robot_ids`로 받아야 하며 기본값은 없다. `robot3`/`robot7` 같은 ID도 가능하고, 두 필드 중 하나라도 허용 집합 밖이면 `WRONG_ROBOT`으로 쓰기 전에 거절한다. 허용 집합은 정렬한 불변 tuple로 보존하고 읽기 전용 속성으로 제공한다.

DB metadata에는 허용 ID 집합을 사진 정책과 함께 고정한다. 순서만 다른 같은 집합으로는 재오픈할 수 있고, 집합 변경이나 허용 ID metadata가 없는 이전 DB는 `POLICY_MISMATCH`다. 기존 DB에 현재 설정을 조용히 덮어쓰거나 과거 보고를 바꾸지 않는다. 필요한 데이터 이전은 원본 보존 후 별도 명시적 migration으로 다뤄야 한다. 읽기 전용 export도 저장된 `allowed_robot_ids`를 함께 표시한다.

`OperationsError`는 공통 `RescueError`이며 `.code`는 C-09, `.detail_code`는 `rescue_operations.<원 세부 코드>`다. 예를 들어 저장 실패는 `REPORT_NOT_PERSISTED`/`rescue_operations.STORAGE_ERROR`로 표현한다. `tests/test_review_operations.py`는 사용자 지정 ID, 알 수 없는 원본/수행 로봇, 정책 불변·재오픈 불일치·기존 metadata 부재를 실제 임시 SQLite로 검사한다. 기존 실행 증거·source hash는 수정하지 않는다.

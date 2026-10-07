# rescue_operations

ROS 없이 실행할 수 있는 SQLite 보고 저장소, 증거 bytes 저장, 송신 outbox와 로컬 JSON 내보내기를 제공한다. C-02·C-05·C-08 및 CT-12의 로컬 구현이다. 사진 필수 여부는 확정하지 않고 저장소 생성자가 `evidence_policy`를 명시하게 한다.

```python
from rescue_operations.ledger import ReportLedger, ReportOutbox, LedgerTransport

with ReportLedger("operations.sqlite3", evidence_policy="OPTIONAL") as ledger:
    with ReportOutbox("sender.sqlite3") as outbox:
        outbox.enqueue(payload)  # schema 1.0의 검증된 보고 dict
        receipt = outbox.deliver(payload["report_id"], envelope, LedgerTransport(ledger))
        # RECEIVED/PENDING_EVIDENCE는 보고 완료가 아니다.
```

`payload`와 `envelope`의 고정 필드, 예시, 실패 의미는 [구현 문서](../../docs/implementation-operations.md)에 있다. `OPTIONAL`은 예시 선택이며 운영 사진 정책의 결정이 아니다. 필요한 증거가 선언됐으면 OPTIONAL에서도 실제 bytes 저장을 기다린다.

```bash
PYTHONPATH=src/rescue_operations python3 -m rescue_operations operations.sqlite3 --limit 100
# 설치한 패키지:
rescue-report-export operations.sqlite3 --report-id <report-uuid>
```

CLI는 operations 호스트의 기존 DB를 읽어 JSON을 출력한다. 없는 DB를 만들거나 보고를 변경하지 않는다. `PERSISTED`는 보고와 선택한 정책의 증거가 로컬 SQLite transaction으로 저장됐다는 뜻이며, 접근의 물리적 성공이나 사람 구조·생존을 판단하지 않는다.

operations DB의 소유자는 operations 호스트다. 다른 PC는 DB 파일을 마운트하거나 직접 쓰지 않는다. 송신 측 outbox는 별도 로컬 DB이고 API를 통해 같은 보고 ID를 재전송한다. 현재 제공하는 API 어댑터는 프로세스 내부 callable이다. 원격 HTTP·ROS·인증·실기·4-PC 통신은 아직 구현하거나 검증하지 않았다. 다른 시스템의 Nav2 goal이나 속도 명령을 발행하지 않는다.

원본 코드를 복사하지 않은 새 구현이다. 패키지 버전 `0.0.0`, `LicenseRef-Unspecified`, 미지정 maintainer는 기존 기초 구조의 미결정 메타데이터를 유지한다. 배포 권한이나 라이선스 허가를 뜻하지 않는다.

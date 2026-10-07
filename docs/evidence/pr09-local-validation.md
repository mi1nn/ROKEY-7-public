# PR 9 로컬 검증 기록 — 2026-10-08

이 기록은 `4a1ff4583eaa0816215d980fdbacdec86bf1bbd5`를 읽기 전용 기준으로 삼아 별도 초안 디렉터리에 작성한 PR 9의 실행 기록이다. 아직 이 증거를 원격 PR의 최종 head 검증으로 주장하지 않는다. 통합 담당은 실제 head에서 필요한 회귀 검사를 다시 실행한다.

| 실행 | 결과 |
| --- | --- |
| Python 3.11.8 소스 unittest | 23 passed |
| Python 3.12.14 소스 unittest | 23 passed |
| Python 3.12 offline wheel build | rescue_operations 0.0.0 성공, `--no-index --no-deps --no-build-isolation --no-cache-dir` |
| 별도 Python 3.11 venv에 동일 wheel 설치 | 성공. 설치 경로 import와 ROS 미import 확인, 23 passed, 설치 CLI help 성공 |
| 별도 Python 3.12 venv에 동일 wheel 설치 | 성공. 설치 경로 import와 ROS 미import 확인, 23 passed, 설치 CLI help 성공 |

시험은 실제 임시 SQLite DB를 만들고 닫았다 다시 열었다. SQL trigger로 보고 INSERT, 증거 INSERT, PERSISTED 전이 UPDATE, 송신 outbox receipt UPDATE의 실패를 각각 주입했다. rollback 후 재전송으로 복구했고, commit 후 응답을 버린 경우 같은 보고가 중복 저장되지 않았다. 두 연결의 동시 동일 보고 20회 수신도 보고 1개·동일 receipt로 남았다. 사진 증거는 명시적으로 synthetic 문장 bytes를 사용해 byte/hash 저장만 검증했다.

실행 명령·출력·exit code는 [원시 로그](pr09-execution.log), 소스/시험/wheel SHA-256과 Python 버전은 [해시 기록](pr09-artifact-hashes.json)에 있다. 로그의 임시 venv 경로는 이 검증 환경의 실행 위치이며 제품 배포 경로가 아니다. 저장한 wheel과 venv는 작업 디렉터리의 임시 산출물이고 저장소에 포함하지 않는다.

시스템 Python에 설치하지 않았고 패키지를 네트워크로 다운로드하지 않았다. ROS·colcon·카메라·추론·Nav2·두 AMR·네 PC 통신·외부 네트워크 서버·전원 차단 시험은 실행하지 않았다. stdlib callable의 소프트웨어 통합과 실제 배포 통신은 별도다. SQLite FULL/WAL 설정은 하드웨어·파일 시스템 전원 손실 특성의 측정을 대신하지 않는다.

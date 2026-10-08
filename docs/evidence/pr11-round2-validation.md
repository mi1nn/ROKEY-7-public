---
title: PR11 round 2 통합 검증 기록
status: measured-local-e2
updated: 2026-10-08
---

# PR11 round 2 검증

이 기록은 origin PR11 `b03ff44`에 각 소유 PR의 round 2 코어와 통합 fixture를 모은 **격리 snapshot**에서 실행했다. 최종 PR HEAD나 GitHub Actions 성공을 주장하지 않는다. [소스 해시](pr11-round2-source-hashes.json)가 실제 검사한 파일을 식별한다. 이전 `pr10-local-validation.md`와 그 147개/6 wheels 로그는 당시 소스의 historical 증거로 그대로 보존했다.

## 실행 결과

| 실행 | 결과 | 원문 |
| --- | --- | --- |
| Python 3.11.8 source `python3 -B scripts/run_tests.py` | 213 tests PASS | [source 3.11](pr11-round2-source-py3.11.log) |
| Python 3.12.14 source `python3 -B scripts/run_tests.py` | 213 tests PASS | [source 3.12](pr11-round2-source-py3.12.log) |
| Python 3.12.14 `python3 -I -B scripts/check_wheels.py` | 7 wheels build/install, pip check, 격리 import, 설치본 213 tests PASS | [설치본·wheel SHA](pr11-round2-installed-py3.12.log) |
| 두 역할의 `run_scenario` | 14 checks PASS, 자동 접근 SEND 0 | [생성 summary](pr11-round2-synthetic-summary.json) |

Python 3.11.8 및 Python 3.12.14 실행기를 사용했다. macOS에서 실행했으며 ROS 배포판·장치 환경의 증거는 아니다. 빌드 실행기에는 setuptools/wheel이 이미 있었다. pip는 `--no-index --no-deps --no-build-isolation --no-cache-dir`로 소스 복사본에서 wheel을 만들었고, 새 임시 venv에 설치했다. 네트워크 다운로드나 시스템 Python 설치 변경은 하지 않았다.

설치 검사에서는 `rescue_bringup`, `rescue_contracts`, `rescue_coordination`, `rescue_mapping`, `rescue_operations`, `rescue_perception`, `rescue_robot`의 wheel 이름 집합을 정확히 확인했다. 각각의 import가 새 venv 안에 있음을 확인한 뒤 tests/config만 있는 별도 checkout에서 `-I -B`로 213개 시험을 실행했다. 해당 checkout에는 `src/`가 없으며 외부 `PYTHONPATH`/`PYTHONHOME`도 제거했다. wheel SHA는 이번 임시 빌드의 식별자이며 재빌드 바이너리 재현성을 주장하지 않는다.

## 닫힌 통합 경계

- 원 ROS stamp `(1700000000, 123456789)` → perception 관측 → `SourceObservation` → 관측 시각 `TimedTransform` → 실제 `MappingRegistry.project` → 보고/evidence의 값이 `1700000000123456789`로 유지된다. TF query/sample을 1 ns 바꾼 실제 project 호출은 canonical `STALE_TRANSFORM`, detail `rescue_mapping.TF_TIME_MISMATCH`로 거절된다.
- A 발견→B 인계와 B 발견→A 인계 모두, 같은 executor의 수색 SEND 의도 → CANCEL 요청 → terminal `CANCELED` → 새 합성 정지 샘플 → 재arm → 배정기가 발급한 다음 generation → 접근 admission → 실제 `TaskAdmission`을 받은 `Allocator.record_admission`까지 연결했다. 두 경로의 generation은 수색 0→접근 1이며, fixture가 외부 고정값 1/2를 따로 주지 않는다.
- terminal만으로 수색 결과가 완료되지 않았고, 정지 샘플 후 `CANCELED`/`stop_confirmed=true`가 된다. 접근 예약은 receipt 전에는 active가 아니며 접수 후 active로 승격된다. 좌표는 유효해도 mapping 운영 허가가 false이므로 접근 dispatch는 false, SEND 의도도 없다. 접근 작업은 `GUARD_INVALID` STOPPING에 남기며 접근 완료나 해제를 주장하지 않는다.
- 프로파일에서 얻은 두 로봇 allowlist를 executor/allocator/ledger/outbox에 전달한다. `TRACKING` 모드는 관측에 명시했다. 통로 초기 공백 확인에는 명시적 `holder=None`, 점유에는 실제 Reservation을 사용한다.

통합 시험 10개는 기존 8개를 새 API에 이식하고 exact epoch/1 ns 거절과 실제 인계 receipt 경계 회귀 2개를 추가한 것이다. 수정 도중 fixture에 남은 float `TimePolicy`를 새 코어가 `INVALID_INPUT`으로 거절한 것을 확인하고, 모든 정책 duration을 공통 정수 ns 변환으로 교체했다. 이것을 기존 제품 결함의 red 증거로 재분류하지 않는다.

## 증거 한계

CT-06/CT-07의 위 사슬은 E2 부분 검증이다. ROS cancel/result 연결, 실제 stop 관측·접근 주행·현장 재확인, 실시간 센서/모델/SLAM backend, 2 AMR와 예정 4-PC의 통신·부하·현장 인수는 NOT_RUN이다. `summary`의 실제 로봇 0대·프로세스 1개 표시를 유지한다. 생성 PNG는 단색 타일이며 사람 사진이 아니다. 두 보고의 `PERSISTED`는 로컬 저장 완료이고 `UNCONFIRMED`·`NOT_ATTEMPTED` 결과를 구조 완료로 바꾸지 않는다.

summary는 임시 출력 디렉터리에서 생성한 원문을 복사했다. 두 SQLite DB는 같은 실행과 시험 안에서 close/reopen 및 증거 bytes/해시·receipt 재시도를 검사한 뒤 임시 디렉터리와 함께 정리했다. DB 파일 자체는 이 기록에 보존하지 않았다.

## 실제 PR 브랜치에서 재확인

격리 snapshot을 PR 브랜치에 합치고 CLA-32 오류 진단 회귀 3개를 추가한 뒤 Python 3.11 source 216개, Python 3.12 source 216개 및 새 venv 설치본 216개를 다시 실행해 통과했다. [최종 소스 검사](pr11-review-round2/source-tests-final.log), [3.12 소스 검사](pr11-review-round2/source-python312-final.log), [최종 설치본 검사·wheel SHA](pr11-review-round2/installed-python312-final.log), [최종 검사 대상 해시](pr11-review-round2/manifest.json)에 기록한다. GitHub Actions 상태는 이 로컬 기록과 별도로 최종 PR 댓글에서 확인한다.

snapshot의 `src/rescue_bringup/README.md`는 설치 수를 여섯→일곱으로 정정했으므로 이전 snapshot manifest와 해당 문서 hash가 다르다. 위 snapshot의 213개 로그는 CLA-32 수정 전 기록이다. 최종 브랜치는 공통 진단 사전과 협업 오류 발생부를 수정하고 회귀 시험 3개를 추가했으므로 현재 소스·시험 해시는 최종 manifest를 기준으로 확인한다. snapshot 기록을 최신 216개 결과로 바꾸지 않았다. README도 wheel data에 포함되므로 최종 branch에서 wheel을 다시 만들어 검사했다. 로그의 실행기·임시 경로·traceback 경로는 개인 경로 대신 `<PYTHON312>`/`<TMP>`/`<USER_CACHE>`/저장소 상대 경로로 정규화했다. 검사 0개나 미실행을 성공으로 바꾸지 않았다.

# 계획 PR 10 최종 로컬 검증 — 2026-10-08

이번 증거는 Python 코어/합성 리허설 E2다. 실제 로봇 0대, ROS/모델 추론/실제 4-PC 실행 미실행이다. 로그의 개인 저장소/임시/번들 runtime 절대 경로만 `<REPO>`/`<TMP>`/`<PYTHON312>`로 정규화했다. PASS/실패 내용은 바꾸지 않았다.

| 검사 | 실행·결과 | 증거 |
| --- | --- | --- |
| source | Python 3.11.8, `python3 -I scripts/run_tests.py`, 147 tests OK | [source log](pr10-python311-tests.log) |
| 6 packages wheel | Python 3.12.14, `python3 -I scripts/check_wheels.py`, 오프라인 build→새 venv 설치→pip check→격리 import→147 tests OK | [installed log](pr10-installed-python312.log) |
| 파일 동일성 | src/tests/config/scripts/CI 설정의 SHA-256, 실행 당시 상위 base | [hash manifest](pr10-artifact-hashes.json) |
| 원본/문서 | `python3 -I scripts/check_docs.py --sources ../ROKEY_P4 ../amr-mini-project`, 고정 객체/줄 범위·집계·로컬 링크 PASS | 각 구현 문서의 고정 URL, source-audit.json |
| whitespace | `git diff --check`, PASS | PR diff |
| 물리 실행 | ROS/TF backend/Nav2/센서 모델/장비 watchdog/2 AMR/4 PC, NOT_RUN | [현장 관문](../process/field-runbook.md) |

source 시험 구성: profile 29 + executor 19 + mapping 30 + perception 24 + coordination 14 + operations 23 + offline integration 8 = 147. 통합 모듈은 이 중 8개 시험에서 11개의 합성 시나리오 검사와 실패 입력을 확인한다.

## 발견·수정한 포장 문제

선행 PR 5~9 스냅샷으로 6 wheel의 import·133 tests는 통과했지만, 별도 `pip check`는 새 Python 3.12 venv에 선언된 setuptools 런타임 의존성이 없어서 실패했다. runtime 코드에서 setuptools를 사용하지 않음을 대조하고 이번 PR에서 6개 `setup.py`의 불필요한 runtime 의존성을 제거했다. 빌드 도구는 여전히 필요하며 CI에서 실제 로컬 검증 버전 `setuptools==84.0.0`, `wheel==0.48.0`를 설치한다.

bringup의 offline extra와 ROS manifest에 다른 5개 코어 의존성을 기록했다. 프로파일 단독 CLI는 추가 패키지를 import하지 않는다. wheel 검사기는 복사본에서 빌드하고, src가 없는 별도 tests/config 폴더와 `-I`로 설치본을 시험한다. 수집 0개, 누락 wheel, 깨진 의존성은 실패다. 전체 시험을 source 경로에서 다시 실행한 것만으로 설치 검증을 주장하지 않는다.

## 리뷰 중 수정한 의미 경계

로봇의 발송 직전 정지 상태, action fence 이후의 정지 표본, 중복 terminal의 idempotency, 시계 fault 유지, 수치/lease 원자적 갱신 및 정지 timer 실패 시 취소 보장, mapping의 극소 quaternion, coordination의 장기 공백/도착 뒤 재관측/lease 입력을 독립 감사로 보강했다. perception의 receiver clock 회귀는 실패 회귀시험을 먼저 커밋하고 수정 후속 커밋으로 닫았다. 실제 Git 커밋·test 파일과 PR 본문에서 확인할 수 있다.

이 기록은 Claude/Cursor의 외부 교차 검토 완료를 대신하지 않는다. 모든 Draft의 최종 head 검토와 계획의 실기 관문은 별개다.

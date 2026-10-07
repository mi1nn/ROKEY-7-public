# PR 4 초안의 로컬 검증 기록

**과거 검증 기록:** 이 문서와 `pr04-corrected-artifact-hashes.json`은 2026-10-07 초안 및 최초 PR 4 코드의 이력이다. 2026-10-08 CLA-14~16 반영으로 소스가 달라졌으므로 여기에 기록한 해시·시험 수를 현재 코드의 검증 결과로 사용하지 않는다. 새 검증은 [2026-10-08 리뷰 수정 기록](pr04-review-2026-10-08.md)에 둔다. 과거 로그와 해시 파일은 덮어쓰지 않았다.

날짜: 2026-10-07 KST. 대상: `work/pr04-draft`의 새 구현. 이 기록은 아직 Git SHA가 부여되기 전 초안에서 수행한 검사다. PR에 옮긴 뒤 실제 head SHA에서 재검증한다.

첫 환경: Python 3.11.8. `pytest`, `yaml` 모듈은 발견됐고 `rclpy`는 발견되지 않았다. 후속 검증에는 wheel·setuptools가 이미 설치된 bundled Python 3.12.14를 사용했다. 순수 로직과 테스트는 표준 라이브러리만 사용한다. ROS 설치나 로봇 연결은 시도하지 않았다.

## 최종 수정: JSON·경로 ValueError를 CLI 오류로 정규화

독립 검토에서 파일 크기 제한 안의 5,000자리 JSON 정수와 NUL 문자를 포함한 경로가 처리되지 않은 `ValueError`를 발생시키는 문제가 확인됐다. 프로파일 경로와 `parameters_file` 경로 모두 재현했다. 수정 전에는 세 입력 모두 종료 코드 `1`과 traceback을 출력했다. [수정 전 실제 CLI 기록](pr04-valueerror-before.log).

먼저 세 회귀 시험을 추가한 뒤 기존 코드에서 전체 25개 중 3개 실패를 확인했다. [수정 전 시험](pr04-regression-tests-before.log). JSON 읽기와 두 경로 resolve 경계의 `ValueError`를 `ProfileError`로 정규화하고, 이미 `ProfileError`인 오류는 이중으로 감싸지 않도록 수정했다. NUL이 들어간 경로는 진단 문자열에 이스케이프해서 출력한다.

| 수정 후 실제 실행 | 결과 | 로그 |
| --- | --- | --- |
| 소스, Python 3.12.14, `unittest discover -s tests -v` | 25개 통과, 0.139초, 종료 0 | [source Python 3.12](pr04-regression-tests-after.log) |
| 소스, Python 3.11.8, 같은 명령 | 25개 통과, 0.126초, 종료 0 | [source Python 3.11](pr04-regression-tests-python311.log) |
| 수정 코드로 6개 wheel offline 재빌드 후 새 venv에 설치 | 빌드·설치·6개 설치 경로 import 모두 종료 0 | [재빌드·설치·시험 전체 로그](pr04-corrected-installed-python312.log) |
| 수정된 설치 CLI에 세 잘못된 입력 전달 | 모두 종료 2, `INVALID` 출력, 성공 출력·traceback 없음 | 같은 전체 로그의 `REGRESSION CASE` 3개 |
| 수정된 설치 wheel, Python 3.12.14 `-I -m unittest discover` | 25개 통과, 0.119초, 종료 0 | 같은 전체 로그 마지막 시험 |

수정된 빌드와 venv는 `.artifacts/python312-corrected/`에만 생성했다. 소스에서 격리한 작업 디렉터리에서 설치 CLI를 실행했으며, 패키지는 venv 안에서 import되고 `rclpy`는 import되지 않았다. [검증한 소스 파일·wheel의 SHA-256](pr04-corrected-artifact-hashes.json)으로 아래 초기 결과와 구분한다. OS 명령 인자는 NUL을 전달할 수 없으므로 최상위 경로 NUL 사례는 새 Python 프로세스 안에서 `main(["bad" + chr(0) + ".json"])`의 인자를 구성해 실행했다.

## 초기 검증 이력 — 위 ValueError 수정 전

아래 22개 시험 및 첫 wheel 결과는 초기 코드의 이력이다. **2026-10-07 ValueError 수정 코드의 검증 결과는 위 25개 시험과 재빌드 기록을 사용한다.** 2026-10-08 이후 소스에는 문서 첫머리의 새 검증 기록을 사용한다. 첫 Python 3.11 빌드 실패도 원인과 함께 보존한다.

| 실제 실행 | 결과 | 판정 범위 |
| --- | --- | --- |
| `PYTHONPATH=src/rescue_bringup python3 -m unittest discover -s tests -v` | 종료 0, `Ran 22 tests in 0.042s`, `OK` | 정적 프로파일의 허용/거절·CLI 동작 |
| `PYTHONPATH=src/rescue_bringup python3 -m rescue_bringup config/deployment.example.json` | 종료 0, 아래 출력 | 정적 예시 참조·숫자·identity 검사 |
| `package.xml` 6개를 `xml.etree.ElementTree`로 읽고 이름·ament marker·build type 확인 | 6개 통과 | XML 구조·리소스 존재, ROS dependency resolution은 제외 |
| 각 패키지 디렉터리에서 `python3 setup.py check` | 6개 모두 종료 0 | setuptools 메타데이터 검사. 빌드·설치 검증은 아님 |
| `python3 -m pip wheel --no-deps --no-build-isolation --no-index --wheel-dir .artifacts/wheels src/rescue_robot src/rescue_perception src/rescue_coordination src/rescue_mapping src/rescue_operations src/rescue_bringup` | 종료 1, `error: invalid command 'bdist_wheel'` | 첫 패키지 metadata 단계에서 환경 의존성 부족. wheel 빌드 성공으로 판정하지 않음 |
| bundled Python 3.12.14에서 `pip wheel --no-deps --no-build-isolation --no-index --no-cache-dir`로 6개 패키지 빌드 | 종료 0, 6개 wheel 생성. [빌드 로그](pr04-wheel-python312.log) | 순수 Python wheel 빌드. ROS/ament/colcon 빌드는 아님 |
| `.artifacts/python312/venv`에 `pip install --no-index --no-deps --no-cache-dir`로 위 wheel만 설치 | 종료 0. [설치·CLI 로그](pr04-installed-cli-python312.log) | 작업 폴더 내부 가상 환경에만 설치. 시스템·bundled 환경 패키지는 변경하지 않음 |
| venv Python `-I`로 6개 패키지를 import하고 각 `__file__`이 venv 안인지 검사 | 종료 0, 6개 경로 확인, `rclpy not imported` | 소스 폴더/PYTHONPATH에 의존하지 않는 설치 import |
| 설치된 `check_deployment`를 소스와 분리한 실행 디렉터리에서 검사 | 정상 예시 종료 0, `runtime_enabled=true`인 복사본 종료 2. [실제 출력](pr04-installed-cli-python312.log) | 설치된 console entry point와 거절 경로 |
| venv Python `-I -m unittest discover -s <draft>/tests -v` | 종료 0, `Ran 22 tests in 0.039s`, `OK`. [시험 로그](pr04-installed-tests-python312.log) | Python 3.12.14에 설치한 wheel의 순수 로직 22개 시험 |

예시 CLI 출력:

```text
STATIC CONFIG VALID: review_example; 2 robots; 4 hosts (2 high, 2 mid)
robot_a: namespace=/robot_a, command_host=compute_a
robot_b: namespace=/robot_b, command_host=compute_b
Runtime is disabled. No hardware, ROS graph, timing or safety validation was performed.
```

미실행: GitHub Actions의 Python matrix, `colcon build/test`, ROS 노드·launch, 실제 네 PC 동시 운용, 센서·TF·Nav2·SLAM·모델·정지 watchdog·로봇 시험. 이 PR에는 이를 실행할 runtime이 없다.

wheel 빌드를 위해 추가 패키지를 다운로드하거나 기존 시스템 환경을 변경하지 않았다. 첫 Python 3.11 실패는 그대로 남겼고, 이미 필요한 도구를 가진 다른 Python으로 빌드·설치 검증을 완료했다. 실제 ROS 환경의 버전과 빌드 의존성을 확인한 다음 `colcon` 검증을 별도 기록으로 추가한다.

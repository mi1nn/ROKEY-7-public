---
title: 두 로봇 계약의 오프라인 통합 리허설
status: draft-implementation-e2
updated: 2026-10-08
---

# 오프라인 통합 리허설

PR 10은 PR 5~9의 순수 코어를 **하나의 Python 프로세스**에서 연결한다. 구성 파일은 예정된 고사양 PC 2대·중간 사양 PC 2대를 검증하지만, 해당 PC에 프로세스를 배포하거나 ROS를 실행하지 않는다. 사용한 실제 로봇은 0대이며 실제 모델 추론·SLAM·Nav2·주행·4-PC 통신과 AC 실기는 NOT_RUN이다. AMR 2대 필수 요구와 PC 4대 예정 자원/현재 배포 기준안의 구분은 [범위](scope.md)를 따른다.

`src/rescue_bringup/rescue_bringup/offline_demo.py`의 `run_scenario(profile_path, output_directory)`는 summary dict를 반환한다. 모듈 CLI는 성공 0, 잘못된 프로파일/출력 경로 또는 리허설 실패 2를 반환한다. `load_deployment`로 프로파일을 먼저 검증하며, 이 합성 fixture는 논리 ID `robot_a`, `robot_b`만 지원한다. 다른 유효 ID를 입력해도 A/B 실행으로 조용히 대체하지 않는다. 프로파일의 네 호스트는 배치 데이터이며 실제 연결 대상이 아니다.

## 실행

저장소 루트에서 Python 3.10 이상으로 실행한다. 이 경로에는 ROS·YOLO·GPU 패키지가 필요하지 않다.

```sh
export PYTHONPATH=src/rescue_bringup:src/rescue_robot:src/rescue_mapping:src/rescue_perception:src/rescue_coordination:src/rescue_operations
python3 -m unittest discover -s tests -p test_offline_demo.py -v
python3 -m rescue_bringup.offline_demo config/deployment.example.json --output work/offline-run-001
python3 -m rescue_operations work/offline-run-001/operations.sqlite3 --limit 100
```

출력 디렉터리는 반드시 새 경로여야 한다. 재실행은 `work/offline-run-002`처럼 다른 경로를 사용한다. 기존 디렉터리/파일을 지우거나 덮어쓰지 않는다. 중간 실패 뒤 생긴 디렉터리도 그대로 보존하며 다음 실행에 재사용하지 않는다. `summary.json`의 `passed`는 아래 E2 검사 통과만 뜻한다. DB 내 `PERSISTED`도 로컬 저장 완료이며 구조·접근 성공을 뜻하지 않는다.

## 연결한 경로와 11개 검사

| 단계 | 합성 입력·연결 | 확인하는 것 |
| --- | --- | --- |
| 관측/위치 | 각각의 4×4 registered RGB-D 메타데이터, 2 m 깊이, 같은 local track 문자열 → perception | 원 관측 stamp·UUID·모델 manifest·증거 참조를 보존. 두 로봇은 별도 CANDIDATE 사건 |
| 공통 좌표 | optical XYZ → 관측 시각의 합성 3D transform → 로봇 map → mission | x/y/z 회전을 포함해 변환. 소스/세션/revision과 관측 시각을 mapping에 전달 |
| 역할 교환 제안 | A 관측→B 후보, B 관측→A 후보 → coordination → mapping snapshot | target 위치와 이격한 접근 후보를 만들고 좌표를 검사. 두 역할 모두 제안되지만 `auto_approach_eligible=false`, `approach_command_sent=false` 유지 |
| 만료/보정 | A 관측의 만료 뒤 재발행, A의 map revision 증가 | 만료 재발행 거절. A 관측만 NEEDS_REOBSERVATION, B 관측은 VALID. A가 source 또는 destination인 두 목표는 모두 무효 |
| 수색 실행 경계 | 각 로봇의 SEARCH_AREA → SEND **의도** → 합성 action 이벤트와 정지 샘플 | COMPLETED/stop_confirmed 결과를 순수 코어로 확인. ROS adapter·장치 명령은 실행하지 않음 |
| 통로 | 합성 점유 lease 만료 | 물리 이탈 증거 없이 FREE로 바꾸지 않고 OCCUPANCY_UNKNOWN 유지 |
| 증거/보고 | 각 관측에 연결한 합성 PNG bytes → operations → outbox | bytes 저장 전 PENDING_EVIDENCE, 저장 뒤 PERSISTED. A 보고의 응답 유실 뒤 같은 ID로 재전송하고 두 DB를 다시 열어 보고 2개·상태·시도 횟수 확인 |

검사 이름과 결과는 `summary.json.checks`에 모두 저장된다. 11개 항목은 서로 다른 완료 조건이며 하나라도 false면 `passed=false`와 실패 종료가 된다. `synthetic_search`의 SEND·stop_confirmed는 외부로 발행한 명령이나 실제 정지 관측이 아니다. 값의 출처를 `SYNTHETIC_ACTION_AND_ODOMETRY`와 `real_command_sent=false`로 함께 남긴다.

첫 제안의 좌표 유효성이 나중의 이동 권한을 보장하지 않는다. 이 데모는 실제 SLAM backend의 보정 이벤트를 검증하지 않았기 때문에 정상 기하에서도 자동 접근이 차단돼야 성공이다. 코드의 false를 true로 바꾸는 것은 실기 준비를 구현하거나 검증하는 방법이 아니다.

## 저장 파일과 증거 의미

- `summary.json`: E2 표시, 실제 로봇 0·프로세스 1, 모델/ROS 실행 false, 실기 NOT_RUN, 프로파일 이름·SHA·검증된 배치 snapshot, 제안·보정·수색·보고 검사 결과.
- `operations.sqlite3`: 로컬 operations 보고 2개, report/payload hash/receipt와 관측별 생성 PNG bytes. WAL/SHM은 SQLite가 필요에 따라 만든다.
- `outbox.sqlite3`: 같은 두 보고의 송신 상태·receipt·시도 횟수. 응답을 한 번 잃은 첫 보고는 2회, 다른 보고는 1회 후 PERSISTED가 된다.

PNG는 코드가 만든 4×4 단색 타일이며 카메라 촬영·사람 사진·YOLO 결과가 아니다. 관측의 evidence UUID/SHA와 보고 ref를 연결하고 저장한 실제 bytes의 hash를 확인한다. 모델 manifest의 SHA도 합성 fixture 식별자이며 실제 모델 파일 해시를 주장하지 않는다. 이 데모의 `OPTIONAL` 증거 정책은 Q-07의 운영 사진 정책 결정이 아니다. 선언한 두 ref는 OPTIONAL이어도 실제 bytes가 있어야 보고가 완료된다.

보고는 UNCONFIRMED·NOT_ATTEMPTED이며 위치는 null이다. 지도 보정/미검증 backend 때문에 접근하지 못한 발견 기록을 보존한 결과다. `PERSISTED`를 `APPROACH_COMPLETE`나 물리적 구조 완료로 올리지 않는다.

관측의 ns 시각은 `to_ns`가 그 관측의 float 초에서 만든다. 같은 시각을 초와 ns로 두 번 적지 않는다. 이 변환은 이 fixture 안에서만 하고, PR 5·7의 float 초와 PR 6의 int ns를 저장소 공통 형식으로 통일하지 않는다. 실서비스에서 float 초로 바꾼 뒤 나노초 정밀도를 복원했다고 주장하지 않으며, ROS adapter가 원 sec/nanosec과 시각 의미를 보존해야 한다. 내부 fixture 정책 `SYNTHETIC_NOT_LOCKED_FOR_RUN`의 거리·속도·시간 숫자를 장비 기본값으로 가져가지 않는다. `uid()`는 uuid5라 같은 라벨이면 반복된다. `GoalSnapshot.snapshot_id`는 실행마다 새로 나오는 uuid4라 summary.json의 그 필드는 실행마다 다르다.

## 검증 범위와 후속

`tests/test_offline_demo.py`의 8개 시험은 두 역할·자동 허가 차단, A 관측만의 무효화와 모든 의존 목표 차단, 합성 수색 결과, 실제 evidence bytes/receipt 유실/reopen, 기존 출력 보존, 잘못된 프로파일, 다른 로봇 ID, 모듈 CLI를 확인한다. 구성 요소별 E2 시험은 각 PR의 테스트와 별도로 실행한다. 하나의 시나리오 PASS가 모든 계약·경로를 검증했다는 뜻은 아니다.

이 리허설이 다루지 않는 경로:
- CT-06·CT-07의 발견 → 수색 취소 → 정지 확인 → 새 generation → 접근 후보 사슬. `_search_rehearsal`의 generation 1 `Executor`와 `Allocator`의 generation 2 배정은 서로 연결되지 않는다. 둘 다 순수 코어라 ROS 없이 E2로 이을 수 있고, field-runbook 4절의 E4만이 그 시험은 아니다.
- CT-08의 UNDOCK·준비 제어권.
- CT-09의 장애물·최소 거리 후보 거절.
- CT-03의 센서 정합. 여기의 alignment 근거는 `fixture-only`다.

실제 readiness/clock/TF/SLAM backend·ROS adapters·실시간 탐지·모델/센서 평가·최종 명령 감시·2 AMR 및 예정 4-PC 부하는 [현장 실행 관문](field-runbook.md)에 남긴다. 해당 관문 전에는 `runtime_enabled=false`인 정적 프로파일을 실제 주행 launch라고 취급하지 않는다.

## 고정 원본 근거

이 데모는 원본 실행 코드를 복사하지 않은 새 통합 fixture다. 과거 기록은 실패/시험 원리의 근거이며 이번 실행 결과가 아니다.

- [P4 가짜 두 로봇 관제 시험](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_sysmon/test/test_integration_fake_robots.py#L1-L52): 모의 사건·DB 확인과 실제 로봇 시험을 구분하는 참고.
- [P4 늦은 action 수락 시험](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/test/test_goal_timer.py#L161-L176): 비동기 action 지연을 별도로 시험할 근거.
- [AMR 과거 관측 hold 재발행](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/webcam_detector.py#L145-L159): 수신·발행 시각이 원 관측을 새롭게 만들지 않는지 확인할 근거.

## 설치 검증과 CI

6개 패키지는 runtime에서 setuptools를 import하지 않아 불필요한 `install_requires` 항목을 제거했다. bringup의 `offline` extra와 ROS manifest는 나머지 다섯 코어 의존성을 선언한다. 오프라인 빌드는 setuptools·wheel이 이미 있는 Python으로 `python3 -I scripts/check_wheels.py`를 실행한다. 이 도구는 소스 복사본의 wheel 6개를 새 venv에 설치하고 `pip check`, source 경로 없는 격리 import, 비어 있지 않은 설치본 전체 시험을 실행한다.

[최종 로컬 증거](evidence/pr10-local-validation.md)에 Python 3.11 source 147개 / Python 3.12 설치본 147개, 소스·wheel 해시와 실제 발견한 packaging 실패/수정 내용을 남겼다. CI에는 기존 3개 Python source matrix 외에 Python 3.12 설치본 검사를 추가했다. Actions 실행 전에는 새 job을 PASS로 표시하지 않는다.

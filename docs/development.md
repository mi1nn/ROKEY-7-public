# 개발 기반과 정적 배포 프로파일

이 단계에서 동작하는 기능은 `rescue_bringup`의 **로컬 JSON 프로파일 검사**다. ROS를 import하지 않으며 네트워크·ROS 그래프·로봇에 연결하지 않는다. 나머지 다섯 패키지는 책임을 구분하는 metadata와 README만 갖춘다. 실행되는 것처럼 보이는 dummy 노드나 성공을 반환하는 미구현 노드는 없다.

## 패키지와 현재 구현

| 패키지 | 예정 책임 | 현재 동작 |
| --- | --- | --- |
| `rescue_robot` | 로봇별 임무 실행·명령 중재·취소·정지 | 없음 |
| `rescue_perception` | 영상·사람 탐지·거리·관측 생성 | 없음 |
| `rescue_coordination` | 수색 분담·임무 배정·인계·통행 조정 | 없음 |
| `rescue_mapping` | SLAM 연동·공통 좌표·지도 보정 | 없음 |
| `rescue_operations` | 관제·보고·기록 | 없음 |
| `rescue_bringup` | 구성·기동·실행 전 점검 | 프로파일의 정적 구조·참조 검사만 구현 |

`rescue_interfaces`나 ROS 메시지·액션은 만들지 않았다. PR 3 계약과 실제 ROS 배포판·센서·Nav2 버전을 확인한 후 추가한다. 패키지 구조는 `ament_python` 형식이지만 이 PR의 Python 시험 통과를 `colcon build` 또는 ROS 기동 통과로 보고하지 않는다.

## 로컬 검사

저장소 루트에서 Python 3.10 이상으로 실행한다. 순수 로직의 필수 의존성은 Python 표준 라이브러리뿐이다.

```bash
PYTHONPATH=src/rescue_bringup python3 -m unittest discover -s tests -v
PYTHONPATH=src/rescue_bringup python3 -m rescue_bringup config/deployment.example.json
```

pytest가 설치된 환경에서는 `python3 -m pytest`도 같은 시험을 실행한다. `pyproject.toml`은 시험 탐색과 import 경로를 지정하며 저장소 전체를 단일 Python 패키지로 설치하는 파일이 아니다.

검사 CLI의 종료 코드는 정상 구조 `0`, 잘못된 입력 `2`다. 정상 출력의 `STATIC CONFIG VALID`는 파일과 정적 참조만 유효하다는 뜻이다. CLI는 실행 노드를 생성하지 않으며, 정상 종료 후에도 로봇 runtime은 없다.

## 프로파일 v1의 범위

[`config/deployment.example.json`](../config/deployment.example.json)은 검토용 배치 예시다. 물리 장비의 주소·번호나 최종 배치를 확정한 파일이 아니다. 두 로봇을 각각 고사양 호스트에 놓는 예시는 바꿀 수 있으며 검사기가 해당 배치를 강제하지 않는다.

| 필드 | 규칙 |
| --- | --- |
| `schema_version` | 정수 `1`. 문자열·boolean·미지원 버전 거절 |
| `deployment_id` | 영문자로 시작하고 영숫자·밑줄로 구성한 1~64자 ID |
| `hosts` | 고유 ID의 호스트 4개. `tier=high` 2개, `tier=mid` 2개 |
| `robots` | 고유 ID의 로봇 2개 |
| `robots[].namespace` | 고유한 단일 ROS namespace, 예: `/robot_a`. 전역 `/` 또는 중첩 namespace는 이 초기 형식에서 거절 |
| `robots[].command_host` | 해당 로봇의 명령 소유 호스트 ID 하나. 중복 제어 프로세스가 실제로 없는지는 ROS 그래프로 별도 검증해야 함 |
| `robots[].slam_host`, `perception_host` | 등록된 호스트를 가리키는 배치 후보. 명령 소유권을 추가로 부여하지 않음 |
| `robots[].parameters_file` | 프로파일 디렉터리 안의 상대 `.json` 파일. 절대 경로·상위 경로·디렉터리 밖 symlink 거절 |
| `services.coordination_host`, `operations_host` | 등록된 호스트 참조. 모든 호스트가 적어도 하나의 역할에 사용되어야 함 |

로봇별 JSON은 `schema_version`, `robot_id`, `namespace`, `runtime_enabled`, `use_sim_time`을 가진다. 로봇 ID·namespace가 부모 프로파일과 다르면 거절한다. `runtime_enabled`는 이 단계에서 반드시 `false`이고 `use_sim_time`은 두 로봇에서 같은 boolean이어야 한다. 이 값이 일치해도 실제 PC·로봇의 시계 동기화가 검증된 것은 아니다.

각 파일은 UTF-8 JSON이며 최대 65,536바이트다. 알 수 없는 필드, 중복 JSON 키, `NaN`/`Infinity`, 잘못된 JSON은 오류로 처리한다. JSON 정수 변환 제한을 넘는 입력과 NUL 문자를 포함한 프로파일/설정 경로도 traceback 대신 `INVALID`와 종료 코드 `2`로 처리한다. 65,536바이트는 입력 자원 제한이며 로봇 운용 임계값이 아니다. JSON 안의 명령·모듈·환경 변수·임의 launch는 해석하거나 실행하지 않는다.

이 프로파일은 **ROS 파라미터 YAML도, PR 3의 임무·관측 계약도 아니다.** 값 검증과 경로 참조를 검토하기 위한 별도 초기 형식이다. 속도·정지 거리·배터리·최신성·정지 시한 등 미확정 운용 수치를 예시 기본값으로 넣지 않았다.

## CI와 검증 한계

[`foundation.yml`](../.github/workflows/foundation.yml)은 Python 3.10·3.11·3.12에서 표준 라이브러리 시험과 예시 검사를 수행하도록 작성했다. 각 matrix 실행 결과는 GitHub Actions가 실행된 뒤 확인한다. CI에는 ROS 설치·`colcon`·실기 시연이 포함되지 않는다.

시험은 로봇/호스트 수·성능 구분, 중복 ID·namespace, 미등록/미사용 호스트, 다른 로봇의 설정 교환, 잘못된 경로·symlink·NUL, 중복 JSON 키·큰 파일·5,000자리 정수·깨진 UTF-8, runtime 활성화·시계 모드 혼합, CLI의 성공/실패 구분을 확인한다. 하드웨어의 실제 성능, 주소 충돌, 물리 로봇 중복 등록, 실제 발행자 수, 소유권 failover, 장비측 정지 watchdog, 센서·TF·주행·인식·보고는 확인하지 않는다.

## 메타데이터의 미결정 사항

프로젝트 라이선스와 maintainer 연락처는 아직 결정되지 않았다. `LicenseRef-Unspecified`는 미정 상태를 표시하며 사용 허가를 부여하지 않는다. `Maintainer not yet assigned` / `maintainer@example.invalid`는 패키지 형식을 위한 명시적 미정 표기다. 특정 사용자나 원본 작성자의 소유권·책임을 추정하지 않았다. 프로젝트가 결정한 값으로 교체하기 전에는 이 초기 패키지를 배포용 릴리스로 취급하지 않는다.

원본 P4/amr-mini-project의 코드·가중치·지도·파라미터를 복사하지 않았다. 이후 이식 시 출처·라이선스 고지·계약 변경·관련 시험을 별도 PR에서 확인한다.

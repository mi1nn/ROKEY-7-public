# 개발 기반과 정적 배포 프로파일

PR 4에서 동작하던 기능은 `rescue_bringup`의 **로컬 JSON 프로파일 검사**다. ROS를 import하지 않으며 네트워크·ROS 그래프·로봇에 연결하지 않는다. 후속 PR의 현재 구현 범위는 아래 표와 패키지별 README에서 추적한다. 실행되는 것처럼 보이는 dummy 노드나 성공을 반환하는 미구현 노드는 없다.

## 패키지와 현재 구현

| 패키지 | 예정 책임 | 현재 동작 |
| --- | --- | --- |
| `rescue_contracts` | Python 공통 값·오류·ID·시간·수락 계약 | canonical 오류+세부 원인, 정확한 int64 ns, 프로파일 ID 2개, TaskAdmission. ROS IDL 없음 |
| `rescue_robot` | 로봇별 임무 실행·명령 중재·취소·정지 | 명령·취소·정지 판정 Python 코어, ROS 어댑터 없음 |
| `rescue_perception` | 영상·사람 탐지·거리·관측 생성 | 입력된 탐지의 RGB-D 위치·시간 검증/사건 코어. 모델 추론 미구현 + ROS 모양(Image·CameraInfo·Detection2D) 입력 어댑터 `adapter.py`(E2, 기록·합성 입력; ROS 구독·실시간 처리 미구현, PR #16) |
| `rescue_coordination` | 수색 분담·임무 배정·인계·통행 조정 | 경로 후보·중복 배정·접근 확인·통로 코어. 실제 planner/탐색/ROS 전송 미구현 |
| `rescue_mapping` | SLAM 연동·공통 좌표·지도 보정 | 3D 좌표/원 관측/버전 무효화 코어. 실제 SLAM 보정 감지 미구현 |
| `rescue_operations` | 관제·보고·기록 | SQLite 보고·증거 ledger/outbox·읽기 CLI. 원격 API/관제 UI 미구현 |
| `rescue_bringup` | 구성·기동·실행 전 점검 | 정적 프로파일 검사 + 단일 프로세스 합성 통합 리허설. 실제 ROS launch 없음 |

`rescue_interfaces`나 ROS 메시지·액션은 만들지 않았다. PR 3 계약과 실제 ROS 배포판·센서·Nav2 버전을 확인한 후 추가한다. 패키지 구조는 `ament_python` 형식이지만 이 PR의 Python 시험 통과를 `colcon build` 또는 ROS 기동 통과로 보고하지 않는다.

## 로컬 검사

저장소 루트에서 Python 3.10 이상으로 실행한다. 순수 로직의 필수 의존성은 Python 표준 라이브러리뿐이다.

```bash
python3 -I scripts/run_tests.py
PYTHONPATH=src/rescue_bringup python3 -m rescue_bringup config/deployment.example.json
```

pytest가 설치된 환경에서는 `python3 -m pytest`도 같은 시험을 실행한다. `pyproject.toml`은 시험 탐색과 import 경로를 지정하며 저장소 전체를 단일 Python 패키지로 설치하는 파일이 아니다.

검사 CLI의 종료 코드는 정상 구조 `0`, 잘못된 입력 `2`다. 정상 출력의 `STATIC CONFIG VALID`는 파일과 정적 참조만 유효하다는 뜻이다. CLI는 실행 노드를 생성하지 않으며, 정상 종료 후에도 로봇 runtime은 없다.

## 프로파일 v1의 범위

[`config/deployment.example.json`](../../config/deployment.example.json)은 검토용 배치 예시다. 물리 장비의 주소·번호나 최종 배치를 확정한 파일이 아니다. 두 로봇을 각각 고사양 호스트에 놓는 예시는 바꿀 수 있으며 검사기가 해당 배치를 강제하지 않는다.

수량 정책은 `profile.py`의 `ROBOT_COUNT = 2`, `HOST_TIERS = {"high": 2, "mid": 2}`, `REQUIRE_ALL_HOSTS_USED = True` 한 곳에 모았다. 로봇 2대는 사용자 요구를 유지한다. **호스트 구성과 모든 호스트 사용 조건은 현재 검토 중인 배포 기준안**이며, CLA-01의 “PC 4대가 필수 실행 구성인지 가용 자원인지” 해석은 아직 미결정이다. 상수화는 그 결정을 미리 내리거나 실행 시 임의 수량을 허용하는 기능이 아니다. 예시 파일·인수 조건은 결정된 정책과 함께 갱신한다.

| 필드 | 규칙 |
| --- | --- |
| `schema_version` | 정수 `1`. 문자열·boolean·미지원 버전 거절 |
| `deployment_id` | 영문자로 시작하고 영숫자·밑줄로 구성한 1~64자 ID |
| `hosts` | 현재 `HOST_TIERS` 기준안에 맞는 고유 ID의 호스트. 현재 high 2개·mid 2개, 합계 4개 |
| `robots` | `ROBOT_COUNT`에 맞는 고유 ID의 로봇. 현재 2개 |
| `robots[].namespace` | 고유한 단일 ROS namespace, 예: `/robot_a`. 전역 `/` 또는 중첩 namespace는 이 초기 형식에서 거절 |
| `robots[].command_host` | 해당 로봇의 명령 소유 호스트 ID 하나. 중복 제어 프로세스가 실제로 없는지는 ROS 그래프로 별도 검증해야 함 |
| `robots[].slam_host`, `perception_host` | 등록된 호스트를 가리키는 배치 후보. 명령 소유권을 추가로 부여하지 않음 |
| `robots[].parameters_file` | 실제 프로파일 파일의 디렉터리 안의 상대 `.json` 파일. 절대 경로·상위 경로·그 디렉터리 밖 symlink 거절 |
| `services.coordination_host`, `operations_host` | 등록된 호스트 참조. 현재 `REQUIRE_ALL_HOSTS_USED` 기준안에서는 모든 호스트가 적어도 하나의 역할에 사용되어야 함. 고정 카메라 호스트(`site_cameras[].host`)도 역할로 센다 |

두 로봇의 `command_host`는 같은 호스트일 수 있다. 이는 로봇마다 하나인 논리적 명령 소유자를 한 PC에서 실행할 수 있다는 뜻이며, 같은 로봇을 여러 제어기가 동시에 지휘해도 된다는 뜻은 아니다. 협업 조정(`coordination_host`)과 관제·기록(`operations_host`)도 한 호스트에 둘 수 있다. 현재 기준안에서는 그 결과 다른 등록 호스트가 미사용이 되면 거절되므로, 해당 호스트는 SLAM·인식 등 다른 역할에 배정되어야 한다. 실제 프로세스 수·발행자·부하·장애 격리는 이후 실행 전 점검의 대상이다.

프로파일 경로 자체가 symlink이면 링크를 먼저 해석하고 **링크가 가리키는 실제 파일의 디렉터리**를 `parameters_file`의 기준으로 사용한다. 링크가 놓인 디렉터리나 현재 작업 디렉터리가 기준은 아니다. 파라미터 symlink는 그 실제 프로파일 디렉터리 안의 파일을 가리킬 때만 허용한다.

로봇별 JSON은 `schema_version`, `robot_id`, `namespace`, `runtime_enabled`, `use_sim_time`을 가진다. 로봇 ID·namespace가 부모 프로파일과 다르면 거절한다. `runtime_enabled`는 이 단계에서 반드시 `false`이고 `use_sim_time`은 두 로봇에서 같은 boolean이어야 한다. 이 값이 일치해도 실제 PC·로봇의 시계 동기화가 검증된 것은 아니다.

각 파일은 UTF-8 JSON이며 최대 65,536바이트다. 알 수 없는 필드, 중복 JSON 키, `NaN`/`Infinity`, 잘못된 JSON은 오류로 처리한다. JSON 정수 변환 제한을 넘는 입력과 NUL 문자를 포함한 프로파일/설정 경로도 traceback 대신 `INVALID`와 종료 코드 `2`로 처리한다. 65,536바이트는 입력 자원 제한이며 로봇 운용 임계값이 아니다. JSON 안의 명령·모듈·환경 변수·임의 launch는 해석하거나 실행하지 않는다.

이 프로파일은 **ROS 파라미터 YAML도, PR 3의 임무·관측 계약도 아니다.** 값 검증과 경로 참조를 검토하기 위한 별도 초기 형식이다. 속도·정지 거리·배터리·최신성·정지 시한 등 미확정 운용 수치를 예시 기본값으로 넣지 않았다.

## 프로파일 v2 — 임무 입력 (2026-10-08, 작업 묶음 1)

`schema_version: 2` 는 v1 의 모든 규칙에 `mission` 객체와 로봇별 ROS 이름을 더한다. v1 파일은 그대로 읽히고(`Deployment.mission is None`), v2 배포 파일은 v2 로봇 파라미터 파일만 받는다. 예시: [`config/deployment.v2.example.json`](../../config/deployment.v2.example.json), [`config/robots/robot_a.v2.json`](../../config/robots/robot_a.v2.json).

| 필드 | 규칙 |
| --- | --- |
| `mission.roles` | `{robot_id: "SCOUT"\|"SUPPORT"}` — 배포의 로봇 ID 와 정확히 같은 키, SCOUT 하나·SUPPORT 하나. 역할은 데이터이며 코드가 `robot_a`=SCOUT 처럼 고정하지 않는다(BRD TB1/TB2) |
| `mission.search_areas[]` | 1 ~ 64 개, `id` 는 canonical nonzero UUID(고유), `mission_xy=[x, y]` 유한수, `basis.kind` 는 `PRIOR_MAP`(사전 기준 지도 좌표, Q-09) 또는 `MAP_REVISION_BOUND`, `basis.map_session_id`(UUID)·`map_revision`·`transform_revision`(uint64). 좌표가 실제 지도에서 유효한지는 coordination·mapping 이 실행 시점에 다시 검사한다 |
| `mission.hold_positions` | `{robot_id: {frame_id, x, y, yaw, map_session_id, map_revision}}`. SUPPORT 로봇의 대기 위치는 필수, 키는 배포 로봇만 |
| 로봇 파라미터 v2 `nav_action`·`odom_topic`·`dock_status_topic` | `/<namespace>/…` 아래의 ROS 이름, 서로 다름. 두 로봇의 명령·관측 경로가 섞이지 않게 네임스페이스 밖 이름을 거절한다 |

`profile.py` 의 `ROBOT_ROLES`·`AREA_BASIS_KINDS` 는 `rescue_contracts` 의 값을 복제한다(프로파일 CLI 는 `rescue_bringup` 만으로 돌아야 해 contracts 를 import 하지 않음). 두 값의 일치는 `tests/test_profile_v2.py` 가 검사한다. `python -m rescue_bringup <v2 프로파일>` 은 역할과 구역 수를 함께 출력한다. v2 역시 정적 검사이며 ROS 를 켜거나 로봇을 움직이지 않는다. `runtime_enabled` 는 여전히 `false` 여야 한다.

### v2.1 — 추가 블록 (2026-10-08, 선택·엄격)

`schema_version` 은 그대로 `2` 이고 아래 블록은 **없어도 되지만 있으면 엄격히 검사**한다(기본값 없음). 예시 파일에 전부 들어 있다. 값은 예시이며 실측값이 아니다(`max_age_s` 는 [미확인], 주소는 RFC 5737 문서용, 서버 ID 0·1 은 실제 로봇 번호와 무관). 선택 블록에 JSON `null` 을 쓰면 거절한다(없음 ≠ null). 같은 `address:port` 는 모든 호스트에서 같은 `server_id` 여야 한다.

| 블록 | 위치 | 규칙 |
| --- | --- | --- |
| `readiness` | 로봇 파라미터 v2 | `version`, `max_age_s{odom,scan,camera,wheel_status,map_to_odom_tf}`(양의 유한수, 다섯 키 정확히), `scan_topic`·`tf_topic`·`camera_topics[]`·`wheel_status_topic`(`""` 허용)은 `/<namespace>/…` 아래 서로 다른 이름, `map_frame`·`odom_frame` 은 TF frame 규칙, `watchdog_heartbeat_topic` 은 로봇 측 watchdog 이 생기기 전까지 **반드시 `""`**. 이름은 `rescue_robot.ros_node` 의 파라미터와 같다(#22) |
| `hosts[].network` | 배포 파일 | `domain_id` 0..232, `discovery_servers[]` 1..8 개의 `{server_id 0..255, address, port 1..65535}`(ID·주소:포트 고유, ID = `ROS_DISCOVERY_SERVER` 의 자리), `super_client` 는 **false 만**(조회 도구용 설정은 운영 노드 값이 아님). 적용하지 않고 실행 전 점검이 실제 환경과 비교한다(네트워크 조사 문서(`docs/design/network-discovery.md`, PR #31)) |
| `site_cameras[]` | 배포 파일 | 0..8 개의 `{id: cam_<id>, host(배포 호스트), roles: [DISASTER_SIGN · TARGET_WATCH · EXIT_PASS 중 1개 이상, 중복 없음], source(문자열)}`, id 고유. 2026-10-09 결정대로 고정 카메라 1대가 두 역할(징후 감지 + 대상 감시)을 겸한다. 카메라 어댑터(인식)와 관제 화면이 식별자로 쓴다 |
| `mission.search_areas[].entry_yaw` | 배포 파일 | 유한수(rad). 협업·관제 `mission_sites(areas, entry_yaws=)` 의 입력. 없으면 `None` 이며 coordination 이 거절한다(기본값 없음) |
| `map_basis` | 배포 파일 (2026-10-13) | `{frame_id(TF frame 규칙), map_session_id(UUID), map_revision, transform_revision(0 이상 정수)}`. 한 실행의 지도 기준을 한 곳에 둔다: 조정기 launch 의 `mission_frame_id`·`goal_map_*` 와 AMR 대상 위치(TargetLocation) launch 가 같이 읽는다. 값이 다르면 조정기가 `ROUTE_BASIS_MISMATCH` 로 막는다. `PRIOR_MAP` 검색 영역의 basis 와 대기 위치의 map session·revision 은 이 값과 같아야 한다(대기 위치 frame 은 로봇별 TF frame 이 될 수 있어 비교하지 않음). 블록이 없으면 `Deployment.map_basis` 가 `None` 이고, 필요한 쪽은 `require_map_basis(deployment)` 로 받아 없으면 `ProfileError` 로 멈춘다(기본값 없음). 현장 값(#132): `map`, `22222222-2222-4222-8222-222222222222`, 0, 1 — 예시 파일은 예시 대기 위치에 맞춰 frame 만 `mission` |
| `home_positions` | 배포 파일 (2026-10-13) | `{robot_id: [x, y, yaw]}`(배포 로봇만, 유한수, `map_basis` frame·기준). 각 로봇의 복귀 위치(재범 결정 5A: TB-6 도킹 접근 지점, TB-5 출구). 조정기 launch 는 `home_poses`(SCOUT → SUPPORT, 둘 다 있어야 함), 로봇 launch 는 자기 `task.live.home_xy_yaw` 로 쓴다. `map_basis` 없이 두면 거절. 없으면 `Deployment.home_positions` 가 비고 복귀 위치 없음 |

`python -m rescue_bringup <v2 프로파일>` 은 "v2.1 blocks: N site cameras; M hosts with network; readiness for …" 한 줄을 더 찍는다. 역시 정적 검사다.

### v2 입력 후보와 실행 전 점검 — 미구현

아래는 CLA-16 및 다중 ROS 그래프 설계 검토에 연결할 **미구현 후보**다. v1 JSON에 추가하면 알 수 없는 필드로 거절한다. v2 스키마·자료형·실제 장비값과 Discovery Server/도메인 배치는 PR 2·3 및 실기 연결 전에 정한다.

| v2 입력 후보 | 필요한 확인 |
| --- | --- |
| 로봇의 `physical_id`/장비 번호, `hostname`, `address` | 논리 `robot_id`와 실제 장비의 대응을 확인. 같은 hostname만으로 로봇을 구분하지 않고 장비에서 읽을 수 있는 고유 근거를 선택 |
| 로봇·프로세스별 `ros_domain_id` | 명령·센서가 어느 ROS 그래프에 속하는지 기록. 두 로봇의 단일 도메인이나 도메인 간 중계가 이미 결정됐다고 해석하지 않음 |
| 프로세스별 `discovery_servers`의 주소·포트·서버 ID | 실제 discovery 설정과 기대 연결 대상을 비교. 서버 목록의 주소나 ID를 원본 robot3/robot6 설정에서 복사하지 않음 |
| 기대 namespace·지도/TF·명령 발행자와 프로세스 소유권 | 다른 PC의 중복 SLAM·제어기, 잘못된 로봇 연결, 예상 외 발행자를 실제 그래프와 비교 |
| 실행 전 점검 대상과 재연결 조건 | 로봇 재부팅·네트워크 변경 이후 낡은 discovery 목록과 실제 메시지 수신을 구분. P4의 `preflight.py`는 필요한 항목을 선별할 후보 |

위 필드를 기록하는 것만으로 물리 식별·discovery·중복 발행자가 검증되지는 않는다. 실제 장비 질의·메시지 수신·기대 그래프 비교·정지 경로 확인이 있어야 해당 실행 점검을 완료로 판정한다. 이 PR은 장비에 접속하거나 discovery 설정을 바꾸지 않는다.

## CI와 검증 한계

[`foundation.yml`](../../.github/workflows/foundation.yml)은 Python 3.10·3.11·3.12에서 표준 라이브러리 시험과 예시 검사를 수행하도록 작성했다. push는 `main`만 대상으로 하며 PR은 `pull_request` 이벤트로 검사해 같은 PR head의 push/PR 중복 실행을 줄인다. 각 matrix 실행 결과는 GitHub Actions가 실행된 뒤 확인한다. CI에는 ROS 설치·`colcon`·실기 시연이 포함되지 않는다.

시험은 로봇/호스트 수·성능 구분, 중복 ID·namespace, 미등록/미사용 호스트, 같은 PC의 로봇별 명령 소유·관리 역할 동거, 정책 상수 변경 시 CLI의 실제 tier 집계, 다른 로봇의 설정 교환, 잘못된 경로·프로파일 symlink 기준·NUL, 중복 JSON 키·큰 파일·5,000자리 정수·깨진 UTF-8, runtime 활성화·시계 모드 혼합, CLI의 성공/실패 구분을 확인한다. 하드웨어의 실제 성능, 주소 충돌, 물리 로봇 중복 등록, 실제 발행자 수, 소유권 failover, 장비측 정지 watchdog, 센서·TF·주행·인식·보고는 확인하지 않는다.

## 메타데이터의 미결정 사항

프로젝트 라이선스와 maintainer 연락처는 아직 결정되지 않았다. `LicenseRef-Unspecified`는 미정 상태를 표시하며 사용 허가를 부여하지 않는다. `Maintainer not yet assigned` / `maintainer@example.invalid`는 패키지 형식을 위한 명시적 미정 표기다. 특정 사용자나 원본 작성자의 소유권·책임을 추정하지 않았다. 프로젝트가 결정한 값으로 교체하기 전에는 이 초기 패키지를 배포용 릴리스로 취급하지 않는다.

원본 P4/amr-mini-project의 코드·가중치·지도·파라미터를 복사하지 않았다. 이후 이식 시 출처·라이선스 고지·계약 변경·관련 시험을 별도 PR에서 확인한다.

## 계획 PR 5~10의 구현 근거

패키지별 [로봇](implementation-robot.md)·[좌표](implementation-mapping.md)·[관측](implementation-perception.md)·[조정](implementation-coordination.md)·[보고](implementation-operations.md)·[통합](implementation-integration.md) 기록에 공개 API, 원본 출처, 자동시험과 미구현을 정리했다. 기존 위의 프로파일 v1 검사는 유지된다. 통합 리허설은 `runtime_enabled=false`를 바꾸지 않으며 물리 실행 권한이 없다.

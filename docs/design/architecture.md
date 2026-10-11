---
title: 두 AMR 재난 수색 시스템의 구조와 배포 제안
status: draft
updated: 2026-10-08
---

# 두 AMR 재난 수색 시스템의 구조와 배포 제안

이 문서는 [범위](../requirements/scope.md)의 **AMR 2대 필수 조건**과 **PC 4대 예정 자원(고사양 2·중간 사양 2)**을 바탕으로 책임과 배포를 제안한다. 고사양 PC를 로봇별 처리에, 중간 사양 PC를 협업 제어와 관제에 두는 4호스트 구성을 현재 설계 기준으로 삼는다. 네 PC 모두의 사용을 사용자가 확정한 인수 의무로 해석하지 않는다. 발견·접근·보고의 최종 인수 범위와 PC 사용 조건은 scope의 미결정 사항을 따른다. P4에 기록된 기존 장비·운용 근거는 아래에 보존하고, 파이널에 실제 배치할 장비와 달라진 조건을 재확인한다. 이 문서의 구조는 실행 성능이나 실기 안전 검증 결과가 아니다.

원본 근거는 [선별표](../process/source-selection.md)의 고정 SHA를 따른다. 패키지 이름과 아래 흐름은 신규 설계 제안이다. 상세 메시지·상태 전이·수치·시험 계약은 계획 PR 3에서 정하고, [알고리즘 결정](algorithm-decisions.md)의 실험 관문을 통과한 구성만 실행 기본값으로 승격한다.

## 기존 환경의 근거와 파이널에서 확인할 차이

아래 `[원본 기록: 측정]`은 P4가 당시 환경에서 측정했다고 기록한 사실이다. 이번 문서 수정에서 재측정했다는 뜻이나 파이널 두 로봇·네 PC가 이미 검증됐다는 뜻이 아니다. `[설정]`, `[강의]`, `[문서]`, `[추정]`을 측정과 구분한다.

| 항목 | 고정 원본에서 직접 확인한 근거 | 파이널 확인 범위 |
| --- | --- | --- |
| 플랫폼·센서 | TurtleBot4(Create 3 + Raspberry Pi 4), OAK-D Pro는 `[원본 기록: 측정]`; RPLIDAR A1M8 모델은 `[강의]`, scan 수신은 `[측정]`. [SRD](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/srd.md#L43-L49) | 실제 배치할 두 장비의 번호·모델·센서·설정 일치 여부. 기존 robot3 기록을 출발점으로 사용 |
| OS·ROS·PC | 기존 PC의 Ubuntu 24.04.5·ROS 2 Jazzy·RTX 4070 Laptop은 `[원본 기록: 측정]`; Jazzy·Ubuntu 24.04는 당시 `[강의]` 제약. [PC 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/srd.md#L49-L49), [과정 제약](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/srd.md#L64-L66) | 네 PC의 실제 CPU/GPU/RAM·설치 버전과 과정 제약의 적용 여부. 기존 PC 한 대의 사양을 네 대에 일반화하지 않음 |
| 통신 구성 | `/robot3`, 도메인 3, 로봇 안 Discovery Server는 P4의 확정·측정 기록. 두 로봇 동일 도메인 안은 제안이며, 미결정 시 기존 로봇별 설정을 유지하는 B안이 당시 기본값. [DEC-003](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/decisions.md#L71-L77), [DEC-010](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/decisions.md#L142-L151) | 두 번째 로봇과 실제 네트워크에서 A/B안 비교. 기존 로봇 설정을 자동 변경하지 않음 |
| SLAM/Nav2·정지 | PC에서 SLAM·Nav2를 돌리는 결정은 P4에서 확정. Create 3 명령 단절 후 0.5초 정지는 H.2.5 기준 `[문서]` 인용이며 당시 설치 펌웨어 값은 미확인. [DEC-011](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/decisions.md#L153-L160) | 실제 두 로봇의 펌웨어·timeout·정지 경로를 확인하고 측정. 문서의 0.5초를 파이널 정지 합격값으로 승계하지 않음 |
| Discovery Server 부하 | 2026-10-06 로봇 서버 CPU 92%는 `[원본 기록: 측정]`; 연결 프로세스가 많아서라는 설명은 `[추정]`. [현장 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L63-L69) | PC 수만 아니라 실제 participant·구독자 수를 기록하고, 두 로봇·기준 배치 동시 실행 시 로봇별 discovery CPU·수신 단절·지연 측정 |
| RGB+Depth 동시 수신 | 2026-10-07 설정 D에서 합 3.41 MB/s·RGB 지연 1244 ms는 `[원본 기록: 측정]`. 약 3.4 MB/s에서 막힌다는 원인은 `[추정]`. 같은 날 설정 E는 합 2.34 MB/s·RGB 114 ms·Depth 924 ms로 별도 측정. [D 측정·해석](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-07-depth-bandwidth.md#L9-L22), [E 측정](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-07-depth-bandwidth.md#L34-L43) | 해상도·구독 수·장면·로봇 수를 명시한 동시 부하 측정. D/E의 수치를 네트워크 고정 상한으로 쓰지 않음 |
| 정지 상태 RTT | 2026-10-01 정지 ping 10회의 평균 RTT 9.2 ms는 `[원본 기록: 측정]`; 그 기록에서 주행·영상 중 지연은 미측정. [baseline](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/reference/2026-10-01-lab-environment-baseline.md#L23-L38) | 위 RGB+Depth 동시 수신과 다른 날짜·부하·지표다. 제어 RTT와 영상 관측 지연을 각 조건에서 따로 측정 |

TF의 접두 없는 frame 이름과 namespace remap은 아래 좌표 절의 `[설정]` 근거를 사용한다. 해당 코드만으로 실기 TF 성공이나 `frame_prefix` 활성화 지원을 판정하지 않는다.

## 배포 후보와 책임

| 배치 후보 | 실행 책임 | 다른 호스트에 전달할 것 | 확인할 전제 |
| --- | --- | --- | --- |
| 고사양 H1 | A 센서 입력, A SLAM·Nav2, A 인식, A 임무 실행기 | 관측·지도/위치 메타데이터·로봇 상태·명령 결과 | A 센서 전송량, CPU/GPU·메모리, 지연, A와의 연결 |
| 고사양 H2 | B 센서 입력, B SLAM·Nav2, B 인식, B 임무 실행기 | H1과 같은 계약의 B 데이터 | 두 고사양 PC가 동일 성능이라는 가정 없이 각각 측정 |
| 중간 사양 M1 | 사건 목록, 수색·접근 배정, 공통 좌표 정렬/검증, 임무 기록 | 로봇별 임무 요청, 정렬 버전, 배정·처리 결과 | 공통 좌표 계산·상태 처리 부하, 재시작 시 중복 배정 차단 |
| 중간 사양 M2 | 관제 API·화면·증거 저장·보고 내보내기 | 운영 요청, 저장 확인, 관제 조회 | 데이터 저장 용량, 사진 전송 지연, 기록 복구 |
| 로봇 A/B 내장 장치 | 제조사 드라이버, odometry, 센서, 실제 구동과 장치 측 정지 | 로봇별 센서/상태와 명령 응답 | 모델별 구성 확인. 내장 컴퓨터를 마스터 PC 4대에 포함하지 않음 |

H1/H2는 각 로봇에 대한 계산 담당 후보이고, 단절에도 정지하는 최종 장치를 뜻하지 않는다. 원격 PC가 끊겨도 명령이 계속 유지되지 않도록 위 P4 근거를 바탕으로 실제 로봇 측 timeout/watchdog 또는 제조사 정지 경로를 확인한다. 그 경로를 검증하지 못한 구성은 자율 주행 관문을 통과하지 못한다. 현재 정적 프로파일 제안은 호스트 4개·등급 2+2·로봇 2개를 표현한다. 이는 기준 배치의 입력 검사이며 물리 PC 네 대의 사용 의무나 실제 가동 증거가 아니다. 배치와 무관한 원칙은 로봇별 유일한 명령 담당과 임무 상태·보고 저장의 책임 분리다. M1/M2의 동일 호스트 배치는 다른 호스트도 역할을 갖는 조건을 충족하면 현재 4호스트 프로파일에서도 허용된다. 물리 호스트 수를 줄이는 대안을 선택할 때는 프로파일의 수량 조건과 부하·장애 시험을 함께 개정한다.

```mermaid
flowchart LR
  A[AMR A 센서와 구동] <--> H1[H1: A SLAM / Nav2 / 인식 / 실행]
  B[AMR B 센서와 구동] <--> H2[H2: B SLAM / Nav2 / 인식 / 실행]
  H1 -->|관측 / 상태 / 결과| M1[M1: 좌표 검증 / 사건 / 임무 배정]
  H2 -->|관측 / 상태 / 결과| M1
  M1 -->|A 임무 요청| H1
  M1 -->|B 임무 요청| H2
  M1 -->|사건 / 임무 기록| M2[M2: API / 관제 / 증거 / 보고]
  M2 -->|운영 요청| M1
  H1 -->|증거 영상 / 지도 표시 자료| M2
  H2 -->|증거 영상 / 지도 표시 자료| M2
```

대용량 RGB/Depth는 필요한 로봇 담당 PC에서 처리하고, 중앙에는 결과와 제한된 표시 자료·사건 증거를 보낸다. 모든 PC가 모든 원본 영상에 구독하는 구성을 기본값으로 만들지 않는다. 실험 기록은 필요한 호스트에서 수집하고 공통 실행 ID로 묶는다. DDS domain·discovery는 위 DEC-003/010을 시작점으로 삼아 실제 두 로봇 구성에서 확정한다. QoS·압축·시간 동기화도 적용한 설정과 측정 조건을 프로파일에 기록한다. A1/A4에서는 로봇별 Discovery Server CPU와 participant/구독자 수, 메시지 수신 단절, 제어 RTT, RGB/Depth 관측 지연을 함께 남긴다.

## 저장소와 패키지 경계

```text
src/
  rescue_interfaces/     # 공통 msg/action/service; 계약 확정 후 생성
  rescue_robot/          # 로봇별 임무 실행·준비 판정·Nav2 adapter
  rescue_perception/     # 영상·탐지·깊이·센서 좌표 관측
  rescue_coordination/   # 수색·사건 연결·배정·접근 후보 정책
  rescue_mapping/        # SLAM adapter·관측 변환·공통 좌표·revision
  rescue_operations/     # API·저장·화면·보고
  rescue_bringup/        # 로봇/호스트 설정·launch·사전 점검
docs/                    # 범위·선별·설계·계약·실험 기록
```

일반 Python 패키지는 `<package>/domain/`에 ROS 없이 시험할 판단·기하 함수를, `<package>/adapters/`에 ROS·DB·HTTP 입출력을 두는 것을 제안한다. `test/`는 해당 책임의 시험을 둔다. 작은 패키지에 빈 계층을 강제로 늘리지 않는다. `rescue_interfaces`는 ROS 인터페이스 생성용 패키지이며 나머지 여섯 패키지의 Python 구조와 구분한다. launch/config는 `rescue_bringup`이 조합하고 각 패키지가 서로의 노드를 몰래 기동하지 않는다.

| 패키지 | 판단·쓰기 책임 | 받는 입력 / 내보내는 결과 | 경계 |
| --- | --- | --- | --- |
| `rescue_interfaces` | 공통 계약·오류/결과 의미 | 메시지 정의 | 정책·모델·장비 상수 미포함 |
| `rescue_robot` | 로봇별 임무 세대, Nav2 목표/취소, 준비·정지 확인, 외부 경로 평가 adapter | 임무·운용 dock/undock·경로 평가 요청 / 수락·거절·진행·정지·종료·경로 결과 | 프로젝트의 로봇별 주행·운용 dock/undock 요청의 유일 진입점. Nav2 내부의 planner/controller 연결과 구분 |
| `rescue_perception` | 센서 시각·class·깊이 유효성·로컬 추적 | RGB/Depth/CameraInfo / 원본 관측 | 구동 명령을 보내지 않음; 임상 상태를 추정하지 않음 |
| `rescue_mapping` | 좌표 변환, 정렬 유효성, 지도/변환 revision | SLAM·TF·기준점·관측 / 버전 있는 위치·무효화 | SLAM 라이브러리 내부 알고리즘은 표준 구성요소에 맡김 |
| `rescue_coordination` | 다중 사건 보존, 수색·확인·접근 배정 | 유효 위치·관측·경로 평가 / 임무 요청 | 직접 `cmd_vel`·Nav2 액션 호출 금지 |
| `rescue_operations` | 사건/보고 영속 저장·표시·운영 요청 | 상태·결과·증거 / 저장 확인·조회 | 화면 갱신 시각을 관측 시각으로 바꾸지 않음 |
| `rescue_bringup` | 프로파일 로드·검증·컴포넌트 조합·운용 기동/종료 요청 | 장비/호스트 설정 / launch·사전 점검·실행기로 전달할 dock/undock 요청 | 직접 dock/undock 액션 발행 금지. 원본 robot ID·도크 좌표·개인 경로 승계 금지 |

`rescue_mapping`의 로봇별 SLAM/TF adapter는 H1/H2에, 공통 정렬 registry는 M1에 둘 수 있다. 패키지 하나가 호스트 하나를 의미하지 않는다. M1의 임무 상태와 M2의 보고 DB는 서로 다른 논리 책임이며 공유 파일로 합치지 않는다. 실제 동거 배치를 선택해도 보고 DB는 operations 단일 서비스가 쓰고 다른 컴포넌트는 API/메시지로 접근한다. 임무·보고 ID의 재전송은 동일 사건을 중복 생성하지 않게 하며, 영속 저장 확인을 받기 전에는 `보고 완료`로 표시하지 않는다.

## 좌표와 지도 소유권

기본 검증 후보는 **독립 SLAM 두 개 + 측정한 공통 기준점**이다. A/B의 로컬 지도는 각각 주행에 사용하고, 교환하는 관측·목표만 공통 `mission` 기준으로 변환한다. 이 `mission`은 프로젝트의 좌표 계약 제안이며 두 occupancy grid를 하나로 자동 합쳤다는 뜻이 아니다. 정렬되지 않은 지도는 관제에서 별도 화면으로 보여 주고 교차 로봇 접근 배정은 제한한다.

개념 경로는 `mission → robot_a/map → robot_a/odom → robot_a/base_link → robot_a/camera_optical_frame`와 `robot_b`의 대응 경로다. 실제 장비 frame과 이 계약 이름의 대응은 프로파일에 명시한다. `odom → base`는 장비 odometry 제공자가, `map → odom`은 활성 SLAM 제공자 한 곳이 담당한다. `mission → map`은 정렬 검증을 통과한 mapping adapter만 발행하거나 좌표 변환 API에서 관리한다. REP-105가 설명하는 것처럼 odom은 연속성과 drift, map은 전역 보정과 위치 점프의 성격이 다르므로 두 값을 같은 안정성 기준으로 평가하지 않는다. [REP-105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)

P4는 `[설정]`으로 `/tf`·`/tf_static`을 노드 namespace의 상대 토픽으로 remap하고 접두 없는 `base_link`·`odom`을 사용한다. 테스트도 remap 문자열을 검사한다. [launch](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/rokey_mini/launch/follow_nav.launch.py#L49-L60), [설정 검사](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/rokey_mini/test/test_follow_nav.py#L71-L77). P4의 [MAP_FRAME](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/adapters.py#L20-L20)·[map 기본값](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/target_locator.py#L55-L55), AMR의 [goal frame](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L427-L430)도 접두 없는 `map`이다. **이 코드 근거는 실기 TF 성공의 측정이 아니며, `frame_prefix`를 켤 수 있는지는 미확인이다.**

따라서 초기 통합은 위 설정을 출발점으로 로봇별 TF topic/buffer를 분리하고, 공통 계약으로 내보낼 때 로봇 identity와 frame 대응을 명시하는 안을 우선 검증한다. 실제 설치에서 고유 frame prefix를 지원함이 확인되면 그 방식을 선택할 수 있다. 토픽 namespace만으로 frame ID가 분리되었다고 보거나, 두 로봇의 접두 없는 `map → odom`을 하나의 전역 TF에 섞지 않는다. [TurtleBot 4 공식 다중 로봇 문서](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/multiple_robots.html)는 namespace 실행의 참고이고 실제 그래프 검증은 A1에서 수행한다.

| 값 | 담당/변경 의미 | 적용 규칙 |
| --- | --- | --- |
| `map_session_id` | mapping: SLAM 재기동·리셋으로 좌표 세션 변경 | 이전 세션 위치를 새 세션 goal로 자동 사용하지 않음 |
| `map_revision` | mapping: 보정이 저장 관측의 좌표 의미를 바꿈 | 영향을 받는 관측·접근 목표를 무효화하거나 검증된 방식으로 재투영 |
| `transform_revision` | mapping: 로컬 지도와 `mission` 정렬의 의미 변경 | 정렬에 의존하는 위치·경로 평가·목표를 다시 검증 |
| 일반 TF 갱신 / 새 지도 셀 | 정상 시간 진행·관측 누적 | 모든 TF tick을 위 revision 증가로 해석하지 않음 |

원본 관측은 센서 frame의 위치·관측 시각·소스/부팅 ID·증거 참조와 함께 불변 보존한다. 파생 지도 위치에는 사용한 revision을 기록한다. **과거 시각의 TF 조회만으로 SLAM 보정 이후 과거 대상 위치가 자동 교정되지는 않는다.** backend가 관측과 posegraph node의 관계 및 최적화된 자세를 제공하는 경우에만 검증된 재투영을 한다. 그렇지 않으면 위치를 무효화하고 재관측한다. 초기 기준점 정렬을 한 번 수행한 사실도 이후 loop closure나 지도 변형의 정확성을 보장하지 않는다.

보정 감지·대상 영향 범위를 확신할 수 없으면 해당 지도의 접근 임무 전체를 보수적으로 보류한다. 새 배정 중지 → 기존 goal 취소 → 별도 정지 관측 → 정렬 재확인/재관측 → 새 임무 세대의 순서를 사용한다. 정확한 메시지·정지 시한·재개 권한은 PR 3에서 정한다.

## 명령·정지 경계

`rescue_robot`의 단독 명령 권한은 **프로젝트에서 주행/도킹 요청을 시작·취소하는 진입점을 하나로 둔다는 뜻**이다. Nav2가 내부에서 planner/controller를 호출하고, controller·선택한 smoother·충돌 감시기가 명시된 속도 처리 연결 안에서 발행하는 동작과 구분한다. 외부 후보 채점을 위한 `compute_path_to_pose` 요청도 실행기의 adapter로 모으지만 이는 경로 조회이고 주행 goal 전송이 아니다. 인식·협업·관제의 임의 속도 발행을 허용하지 않는다. 수동 조작을 넣는 경우도 명령 중재와 마지막 정지 경로를 통과하게 한다.

자동 복귀·도킹 시나리오를 파이널 필수 기능으로 추가하지는 않지만 **운용 기동/종료를 위한 dock/undock 절차는 필요하다.** P4의 기본 `power_saver` 관련 기록은 도킹 중 카메라·LiDAR 데이터가 없었던 당시 측정과 설정/강의 설명을 구분한다. [센서·도킹 baseline](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/reference/2026-10-01-lab-environment-baseline.md#L48-L51). 기본 power_saver를 임의로 끄지 않고 실제 설정을 확인한다. `rescue_bringup`의 운영 요청은 `rescue_robot`으로 전달하며 실행기만 dock/undock 액션을 발행한다. 수색·일반 Nav2 이동 전에는 신선한 `is_docked=false`와 언도킹 뒤 센서/TF·SLAM/Nav2 준비를 확인한다. 장치별 도킹 경로·충돌 감시 적용 범위와 취소/종료 관문은 PR 3에서 별도로 정한다.

`explore_lite`의 탐색 기능을 우선 평가하되 통합 경로는 **후보 생성기로 사용하고 goal은 `rescue_robot`이 발행하는 B안**이다. 직접 `NavigateToPose`를 보내는 upstream 노드를 이름만 바꿔 동시에 실행하지 않는다. 직접 전송 A안의 성능 확인은 협업/제어/관제의 이동 명령과 격리한 독립 실험으로 제한하고, 기동 전 이전 goal 종료·정지, 종료 후 잔여 goal/자동 재개·복귀 없음까지 확인한다. 상세 후보·취소·권한 계약은 PR 3, 실제 통합 시험은 PR 8에서 검증한다.

Collision Monitor를 사용하는 구성에서는 모든 일반 속도 경로를 그 입력으로 모으고 최종 속도 출력을 구동 입력에 연결한다. 공식 문서도 velocity 처리의 마지막 링크로 두도록 설명한다. 센서가 오래되었을 때의 정지 설정과 장치 측 명령 timeout은 별도로 시험한다. 이 소프트웨어 구성만으로 장치의 실제 정지나 인증된 안전 기능을 주장하지 않는다. [Nav2 연결 지침](https://docs.nav2.org/jazzy/tutorials/general_tutorials/using_collision_monitor/using_collision_monitor/)

| 상황 | 담당과 동작 제안 | 유지할 정보 |
| --- | --- | --- |
| 협업 PC/링크 단절 | 실행기의 임무 lease 만료와 장치 측 watchdog 경로로 취소·정지·보류 | 활성 task ID/세대, 마지막 수락, 단절 이유 |
| 짧은 탐지 공백 | 마지막 유효 관측과 기존 goal이 아직 유효하고 TF·위치·장애물 센서·lease가 정상인 제한 구간만 기존 goal 유지. 새 goal 없음, 최종 접근 완료 판정 없음 | 원 관측 시각/만료, short-loss 예산, 마지막 검증 상태. 공백·재발행으로 수명을 연장하지 않음 |
| 관측 만료·short-loss 예산 소진 또는 TF/위치/안전 센서/lease 실패 | 대상 만료는 해당 관측에 의존하는 접근 goal에, TF/센서/lease 실패는 그 입력에 의존하는 이동 goal에 취소→실제 정지 관측→재확인 적용. 정지 확인 전 새 이동 금지 | 발견 사건·실패 이유·원본 시각·정지 결과. 무관한 대상 만료만으로 `SEARCH_AREA`를 중단하지 않음 |
| 명령 수락/취소 응답 지연 | 늦은 수락 처리, 정지 관측 전 새 이동 금지 | 액션 요청/응답과 물리 정지 결과를 분리 |
| 관제/저장 PC 단절 | 데이터 수집·전송을 제한된 queue/outbox로 재시도, 저장 전 보고 완료 금지 | 사건과 증거의 유실/대기 상태, 저장 실패 이유 |
| PC·로봇 재기동 | boot/session 식별자 변경, 과거 명령 재실행 차단, 수동/정책 재개 조건 재검증 | 복구 전후 경계와 미완료 임무 |

시간 비교는 관측 source time, 처리 시각, 각 수신자의 local monotonic elapsed time을 구분한다. 서로 다른 PC의 monotonic 수치를 직접 빼지 않는다. source clock의 오차가 계약 범위를 넘으면 위치 변환·접근에 쓰지 않고 명시적으로 품질 저하를 보고한다. 이 경로는 관제 화면의 온라인 표시와 별개로 로봇 실행기에서 적용한다.

## 구현 전에 통과할 관문

| 관문 | 확인할 결과 | 막는 후속 작업 |
| --- | --- | --- |
| A0 기존 근거와 배치 차이 확인 | 위 P4 장비·Jazzy·PC 실행 결정을 출발점으로 실제 두 로봇(두 번째 번호 포함)의 모델·설정·펌웨어 차이, 예정 PC의 사양/설치, DEC-010 네트워크 선택을 기록. TF 접두 설정과 지원 여부·실측을 구별하고 장치 측 정지를 확인 | 실제 파이널 장비 launch·주행 |
| A1 두 대 독립 주행 | 동시 SLAM/Nav2, frame/명령 무혼선, 취소·정지·센서/링크 장애 처리. 각 로봇 Discovery Server CPU·participant/구독자 수와 수신/영상 지연을 실제 동시 부하에서 측정 | 교차 로봇 접근 |
| A2 좌표 교환 | 공통 물리점 오차, 기준점 이동/loop closure/재기동 이후 무효화·재확인 | 자동 발견 인계·공통 지도 표시 |
| A3 인식·접근 입력 | 실제 대상/센서에서 시각·깊이·class·불확실성 검증, 경로 있는 정지점 생성 | 사람 발견 기반 자동 접근 |
| A4 협업·기준 배치 통합 | 현재 4호스트 설계 기준에서 두 로봇 협업, 추가/제거 장애물, 장애 주입, 보고 저장·동일 사건 추적·discovery/영상 부하 확인. 배치 대안을 선택하면 책임 분리와 시험 구성을 함께 개정 | 선택한 배치의 종합 검증. PC 전원 사용의 사용자 의무를 새로 확정하지 않음 |

각 관문의 수치·시험 횟수·로그 항목과 짧은 소실 예산은 PR 3에서 시험 전에 정한다. A0~A4는 문서 승인이나 mock 시험으로 자동 통과하지 않는다. 이 수정에서는 원본 코드를 실행하지 않았고 파이널 관문의 실기 통과 증거도 아직 없다. P4에 존재하는 이전 실기 기록과 구분한다. 계획 PR 4의 구조·설정 검증은 이 계약을 준비하는 작업이며 A1 이후 실기를 대신하지 않는다. 문서 재검토 인계는 runtime 실행 승인과 별개다.

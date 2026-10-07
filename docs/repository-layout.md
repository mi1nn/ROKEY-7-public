---
title: 파이널 디렉터리 구조와 코드 이식 경계
status: proposed
updated: 2026-10-07
---

# 파이널 디렉터리 구조와 코드 이식 경계

파이널 코드는 기능 책임에 따라 ROS 패키지를 나누고, 순수 판단 로직과 ROS 입출력 어댑터를 분리한다. P4의 `homework/mini/src` 분류와 상대 저장소의 교육 패키지 구조를 새 저장소의 최상위 구조로 가져오지 않는다. 아래는 PR 2의 구조 제안이며 실제 존재하는 패키지 목록은 각 구현 PR에서 확인한다.

## 목표 디렉터리

```text
ROKEY-7/
  README.md
  docs/
    scope.md                    요구와 인수 시나리오
    source-selection.md         원본 고정 커밋과 후보 판정
    lessons-learned.md           교훈과 후속 검증
    review-workflow.md           작성·검토·수정·머지 인계
    repository-layout.md        이 문서
    architecture.md             노드·호스트·데이터·명령 책임
    algorithm-decisions.md      선택 근거·대안·미검증
    contracts.md                PR 3 메시지·시간·좌표 계약
    state-machines.md           PR 3 상태·실패·복구 전이
    acceptance-plan.md          PR 3 시험 조건·증거·담당
    evidence/                   작은 검증 manifest와 로그
  src/
    rescue_interfaces/          공통 메시지·서비스·액션 정의
    rescue_mapping/             공통 좌표·변환 유효성·지도 revision
    rescue_perception/          탐지·거리·관측 생성
    rescue_robot/               로봇별 임무 실행·Nav2 어댑터
    rescue_coordination/        수색 분담·대상 사건·임무 배정
    rescue_operations/          관제 API·영속 기록·보고
    rescue_bringup/             실행 구성·프로파일 검사·launch
  config/
    profiles/                   두 로봇·네 호스트의 배치 구성
    robots/                     로봇별 센서·frame·주행 설정
    perception/                 모델·클래스·추론 설정
  tests/
    integration/                패키지 사이 계약·모의 통합
  tools/
    evaluation/                 모델/위치/지연 평가 도구
    data/                       원본 bag 기반 데이터 추출
  assets/
    manifest/                   모델·지도·보정의 출처/해시/취득 절차
  .github/workflows/            실제 실행 가능한 검사만 등록
```

모든 경로를 빈 폴더나 미구현 노드로 먼저 채우지는 않는다. 해당 책임의 구현·설정·시험이 생기는 PR에서 추가한다. PR 4의 최소 기반과 이 최종 목표 구조의 차이는 상태로 명시한다.

## 패키지 책임과 의존 방향

| 패키지 | 책임 | 참조하는 계약 | 직접 하지 않을 일 |
| --- | --- | --- | --- |
| `rescue_interfaces` | 관측·대상 사건·임무·상태·보고 계약의 ROS 표현 | PR 3 계약 문서 | 알고리즘·하드웨어 실행 |
| `rescue_mapping` | 로봇별 SLAM frame과 공통 기준의 변환·revision 유효성 | frame·clock·변환 상태 | 로봇 임무 배정, 검증 없는 자동 지도 병합 |
| `rescue_perception` | 입력 헤더를 보존한 탐지·거리·관측 생성 | observation·class·quality | 로봇 주행 명령, 재발행으로 관측 수명 연장 |
| `rescue_robot` | 한 로봇의 임무 수락·실행·취소·결과, Nav2 연결 | task·robot state·map validity | 전 로봇의 배정, 별도 임의 속도 우회 경로 |
| `rescue_coordination` | 관측을 사건으로 관리, 수색·확인·접근 배정 | event·task·status·report | Nav2/속도 직접 명령, SLAM TF 재발행 |
| `rescue_operations` | UI/API·명령 요청·보고 저장·증거 검색 | 사건·명령 결과·보고 | 독자적 로봇 제어, 공유 SQLite 파일로 여러 PC 쓰기 |
| `rescue_bringup` | 호스트/로봇별 설정 검증과 실행 묶음 | profile·package/contract versions | 센서 사실·시험 PASS의 생성 |

도메인 모듈은 ROS 메시지 객체 대신 명시된 값/타입을 입력받고 결과·명령 의도를 반환한다. ROS 어댑터가 메시지·액션·시각·QoS를 연결한다. 판단 함수를 시험하기 위해 ROS 전체를 실행하지 않아도 되게 하고, 그 시험을 ROS·실기 시험과 구별한다.

의존 방향은 공통 계약→각 기능→실행 구성이다. 기능끼리 다른 노드의 내부 변수·DB 테이블을 직접 공유하지 않는다. 공통 라이브러리는 실제 중복 책임이 확인된 경우에만 추출하며 `utils` 패키지로 모든 코드를 모으지 않는다. ROS `.msg`와 도메인 타입이 함께 존재하면 변환 어댑터와 계약 시험으로 필드·단위·만료 의미를 맞춘다.

## 원본 코드의 목적지

| 선별표 ID | 목적지 후보 | 이식 경계와 필요한 시험 |
| --- | --- | --- |
| P4-01 | `rescue_interfaces` 및 각 어댑터 | ID·세대·만료 원리만 가져오고 구역/AMCL 계약은 PR 3에 맞춰 재정의 |
| P4-02·03 | `rescue_robot` | 액션 관리와 입력 관문. 기존 AMCL FSM 전체 복사 제외. 늦은 결과·취소·만료 시험 동반 |
| P4-04·08 | `rescue_coordination` | 재할당·중복/시간 불연속 원리. 도크 거리·한 경보 슬롯·소스 간 track ID 혼합 제외 |
| P4-05·06·07, AMR-01·02 | `rescue_perception`, 좌표 기준 부분은 `rescue_mapping` | 입력·동기화·기하 함수. 유사 Depth/영상 경로는 비교 후 한 구현만 선택 |
| P4-09·10 | `rescue_operations` | 저장·증거 매칭·화면 표시. 과거 schema/AMCL 상태/지도 단일 frame 전제 교체 |
| P4-11, AMR-03 | `tools/` 또는 해당 기능의 작은 수학 모듈 | 고정 외부 카메라를 실제 채택할 때만 평면 보정 도구 추가. 기존 행렬·launch는 제외 |
| AMR-04·05 및 P4 평가 도구 | `tools/evaluation`, `tools/data` | 명시적 CLI·동일 입력/장비·원본 해시·세션 분할. import 시 학습/추론 실행 금지 |
| AMR-06 및 P4 사전 점검 | `rescue_bringup`, `config/` | 장비/버전 확인 후 적용할 검사·설정. 이전 lifecycle 복구/속도/도크 숫자 자동 승계 제외 |

위 ID의 고정 원본 URL·함수·관련 시험은 [선별표](source-selection.md)에 둔다. 실제 이식 PR은 원본 SHA·파일/줄, 새 목적지, 유지·변경·제외한 동작과 시험을 PR 본문에 남긴다. 이식하지 않고 새로 구현한 경우에도 참고한 원리와 구현 출처를 구별한다.

## 설정과 자산

- 로봇 ID·namespace·frame·호스트·토픽·모델/보정 경로를 코드 상수로 고정하지 않는다. 사람이 수정하는 배치 프로파일과 로봇별 설정의 책임을 나누고 중복 키의 우선순위를 명시한다.
- 프로파일은 정확히 두 로봇과 네 호스트의 역할을 검증한다. 실제 CPU/GPU/OS·네트워크가 확인되지 않은 프로파일은 예시이며 배포 완료가 아니다.
- 모델/지도/보정 파일은 실행 코드와 별도 자산으로 관리한다. manifest에는 출처, 해시, 클래스/단위, 센서/지도 기준, 검증 환경을 기록한다. 실제 형식과 필수 키는 해당 구현 PR에서 정한다.
- 생성물 `build/`, `install/`, `log/`, 녹화·DB·가중치·대용량 학습 결과를 기본 추적 대상으로 만들지 않는다. 시험에 필요한 작은 fixture만 출처·목적을 명시해 둔다.
- 라이선스 선언이 불명확한 원본 코드·자산은 출처/고지 처리를 확인한 뒤 이식한다. 새 패키지 metadata에 원본의 TODO 선언이나 작성자 정보를 무심코 복사하지 않는다.

## 병렬 작성과 검토 경계

PR 2는 구조·배포·알고리즘 판단, PR 3은 계약·전이·시험, PR 4는 실제 빌드/프로파일 기반을 소유한다. 작성은 병렬로 진행하며 PR 3·4는 사용한 상위 가설과 변경 영향을 기록한다. 상위 문서가 바뀌면 소비 문서·코드·시험도 함께 수정한다. 최종 인계와 main 반영은 [검토 지침](review-workflow.md)을 따른다.

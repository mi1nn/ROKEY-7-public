---
title: 원본에서 선별한 레슨런과 파이널 검증
status: draft
updated: 2026-10-08
---

# 원본에서 선별한 레슨런과 파이널 검증

원본의 실패·제약을 새 파이널에서 확인할 시험으로 바꾼다. 아래 예방 규칙은 제안이다. 과거 로봇의 속도·거리·배터리·카메라 설정값은 새 임계값으로 승계하지 않는다.

`[원본 기록]`은 링크한 날짜·장비의 과거 기록이며 이번 재측정이 아니다. P1~P3 사례는 P4 회고가 재인용한 내용이다. 그 사고 원문은 이번 조사에서 확인하지 않았으므로 P4 회고의 주장으로만 인용한다. 원본 커밋과 조사 범위는 [선별표](source-selection.md)에 고정했다. 아래 PR 번호는 [계획 순서](scope.md)의 번호다.

## LL-01 문서의 구현 현황을 코드와 대조한다

**근거:** `[원본 코드·기록]` 같은 커밋의 [SDD 현황표](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/sdd.md#L116-L124)는 AMR·브리지·웹을 없다고 적지만 [제어 노드](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/node.py#L75)와 [브리지](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_sysmon/rokey_sysmon/bridge.py#L76)가 존재한다. 상대 저장소의 [추적 설계](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/docs/RC_CAR_TRACKING_DESIGN.md#L1-L7)도 미구현 초안이라고 적지만 [추적 노드](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L91-L145)는 구현되어 있다.

**적용:** 기능마다 구현 존재, 시험 파일 존재, 해당 커밋 실행 결과, 실기 결과를 구분한다. 오래된 문서만 근거로 없거나 완성됐다고 판단하지 않는다.

**확인:** PR 1의 모든 후보에 고정한 소스와 수정 조건을 붙이고, 실제 이식 PR에서 실행 증거를 갱신한다. 이 PR은 기능 실행·실기 완료를 주장하지 않는다. 연결: AC-01~07, PR 1~10.

## LL-02 두 로봇의 전체 흐름을 초기에 연결한다

**근거:** `[원본 기록]` [P4 회고](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/reference/2026-10-03-lessons-p1-p3.md#L25-L33)는 늦은 통합과 동결 뒤 변경을 반복 문제로 적는다. [기존 운영 관문](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/plan.md#L13-L22)의 단일 로봇 기준과 날짜는 새 요구에 맞지 않는다.

**적용:** PR 5에서 두 대 동시 주행, PR 6에서 좌표 인계, PR 8에서 A 발견→B 접근을 확인한다. 종합 PR 10까지 두 로봇 연결을 미루지 않는다. 기능·모델·설정의 동결 시점은 실제 일정 확인 후 정한다.

**확인:** 각 관문에 시작 조건·관측 결과·성공 분모를 실행 전에 정하고, 동결 뒤 행동을 바꾸는 PR에는 해당 시나리오 재시험을 붙인다. 연결: RQ-05, AC-01·02·05, PR 3·5·6·8·10.

## LL-03 명령뿐 아니라 지도와 TF의 발행 책임도 정한다

**근거:** `[원본 기록]` [2026-10-06 현장 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L33-L34)은 다른 PC의 SLAM 노드로 `/robot3/map` 발행자가 두 개가 되어 시험을 중단했다고 보고한다. [P4 명령 주체 원칙](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/reference/2026-10-03-lessons-p1-p3.md#L34-L40)도 재사용할 만하다.

**적용:** 로봇별 임무 실행기, Nav2, 속도 중재, teleop의 권한을 구분한다. 각 지도와 TF 연결의 발행자를 명시한다. 웹·탐지·협업 노드가 별도로 같은 로봇에 직접 주행 명령을 내지 않게 한다.

**확인:** 네 PC를 동시에 실행한 그래프와 중복 기동 시험에서 예상 외 발행자를 찾는다. 다른 로봇 ID 요청, 정지와 새 배정의 경합, 정지가 모든 활성 주행 경로에 도달하는지 확인한다. 연결: AC-01·06, PR 2~5·8~10.

## LL-04 실행한 코드와 자산의 버전을 기록한다

**근거:** `[원본 기록]` [DEC-018](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/decisions.md#L225-L233)은 ROS workspace가 개발 브랜치를 가리키는 문제와 설치 후 버전이 `unknown`이 되는 문제를 기록한다. [회차 기록 항목](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/test-plan.md#L41-L56)은 재현의 출발점이다.

**적용:** 네 PC에서 실행한 코드 SHA, 실제 패키지 경로, 모델·설정 해시, 지도·보정 버전, 적용 파라미터를 한 실행 기록에 묶는다. 개발용 폴더와 판정용 설치를 구분한다. 기존 workspace 경로와 설치 방식 자체를 강제하지 않는다.

**확인:** 잘못된 overlay, 계약 버전 불일치, 변경된 모델·설정을 실행 전 점검에서 식별하는지 확인한다. PR 시험과 최종 배포의 SHA를 각각 기록한다. 연결: AC-01·05, PR 3~5·7·10.

## LL-05 동시 운용 부하는 동시에 측정한다

**근거:** `[원본 기록]` [RGB·Depth 동시 수신 로그](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-07/rgbd-simultaneous-1301.txt#L1-L7)와 [비교 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-07-depth-bandwidth.md#L9-L22)은 따로 측정했을 때와 함께 구독했을 때의 지연 차이를 보고한다. 두 로봇 동시 운용을 입증한 기록은 아니다.

**적용:** 두 로봇의 영상·Depth·LiDAR·TF·지도와 웹 미리보기·기록을 함께 실행한 부하로 배치를 결정한다. 고사양 PC의 단독 추론 속도로 네트워크와 카메라 지연까지 해결됐다고 판단하지 않는다.

**확인:** 같은 장면·시간·구독자 조건에서 단독/전체 운용의 입력 나이, 처리 지연, 수신 주기, 누락, 네트워크·CPU·GPU·메모리를 비교한다. 과거 FPS·대역폭 값은 새 합격선으로 사용하지 않는다. 연결: AC-01·03·06, PR 2·3·5·7·9·10.

## LL-06 관측 시각과 다시 발행한 시각을 구분한다

**근거:** `[원본 기록]` [거리 보정 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-robot-detect.md#L23-L37)은 영상 지연 때문에 이전 거리의 상자가 섞였을 가능성과 측정 폐기를 적는다. `[원본 코드]` 상대 저장소의 [좌표 유지·재발행](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/webcam_detector.py#L100-L159)은 이전 좌표에도 현재 발행 헤더를 붙인다. [P4 좌표 변환](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/target_locator.py#L74-L105)은 관측 헤더의 시각으로 TF를 찾고, 실패하면 버린다.

**적용:** 센서 취득·수신·처리 시각과 마지막 유효 관측을 구분한다. 재발행만으로 신뢰 기간을 연장하지 않는다. RGB·Depth·TF의 시각 일치, PC 간 시계 오차, 지도 보정 이후 좌표 유효성을 계약으로 정한다.

**확인:** 지연·역순·중복 입력, TF 부재, 센서 중단, 시계 오차를 주입한다. 오래된 좌표가 새 접근 명령이나 최신 관제 표시로 바뀌지 않는지 확인한다. 수신 시각과 헤더의 차이만으로 시계 동기화가 검증됐다고 쓰지 않는다. 연결: AC-02·06, PR 3·6~10.

## LL-07 설정 일치와 공간 정확도를 따로 검증한다

**근거:** `[원본 기록]` [카메라 설정 검사](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-07/config-E-checks-1357.txt#L7-L15)는 설정 일치가 픽셀 정렬을 뜻하지 않는다고 명시한다. [Depth 정렬 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-depth-align.md#L30-L39)에는 overlay 증거가 남은 작업으로 적혀 있다.

**적용:** camera_info와 검증한 카메라 외부 보정을 사용한다. 도크 위치·RC카 크기·웹캠 보정값을 다른 로봇·장소·대상에 복사하지 않는다. 센서 거리 오차와 두 로봇의 공통 좌표 변환 오차를 분리한다.

**확인:** 로봇 각각에서 여러 거리·화면 위치·가림 조건의 RGB/Depth 겹침과 실거리 기준을 확보한다. A의 위치 추정과 B의 재관측을 같은 물리 기준점에 대조한다. 연결: AC-02·03, PR 2·3·6·7·10.

## LL-08 요청 접수와 현장 성공을 구분한다

**근거:** `[원본 기록]` [현장 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L25-L28)은 언도크가 접수된 뒤 동작이 멈춘 사례를 적는다. [시험 계획](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/test-plan.md#L30-L56)은 액션 결과와 실제 위치·도킹 관측을 구분한다.

**적용:** 발견·위치 확인·접근 완료·보고 완료·접근 불가를 구분한다. 접근 완료는 액션 결과와 신선한 위치·대상 재확인으로, 보고 완료는 관제의 수신·저장 근거로 판정하는 계약을 제안한다. 취소는 재시도 가능한 실패와 구분한다.

**확인:** 수락 후 결과 없는 액션, 취소 응답 누락, 이전 goal의 늦은 결과, 저장 실패, 접근 불가를 시험한다. UI와 로그가 관측한 수준보다 강한 성공을 표시하지 않는지 확인한다. 연결: RQ-04, AC-05~07, PR 3·5·8~10.

## LL-09 기존 모델 점수를 사람 탐지 성능으로 가져오지 않는다

**근거:** `[원본 기록]` [P4 모델 결정](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/decisions.md#L115-L122)은 `box/car` 미니 프로젝트 범위다. [모델 비교의 한계](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/model-selection-amr.md#L68-L104)에는 작은 평가 세트·학습 조건·서로 다른 loss 비교 문제가 적혀 있다. 상대 저장소의 [학습 소개](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/README.md#L1-L14)도 `car/dummy` 데이터의 결과다.

**적용:** 비교 도구와 데이터 분할·모델 해시 기록 방법을 가져온다. 사람 또는 합의한 인형, 가림·자세·조도·시점의 평가 세트를 따로 정한다. 모델·가중치는 이번 PR에서 채택하지 않는다.

**확인:** 학습/평가 간 유사 연속 프레임의 누출, 대상 클래스 누락을 점검하고 동일 조건에서 탐지 성능·위치 오차·실운용 지연을 평가한다. 라벨 없는 대리 지표를 recall이나 mAP로 보고하지 않는다. 연결: AC-03, PR 1~3·7·10.

## LL-10 교차 검증의 대상과 증거 범위를 명확히 남긴다

**근거:** `[원본 기록]` [P4 협업 회고](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/reference/2026-10-03-lessons-p1-p3.md#L47-L51)는 같은 파일·결함에 병렬 PR이 겹치고 머지 뒤 push가 생긴 문제를 적는다. [PR 템플릿](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/.github/pull_request_template.md#L1-L34)은 변경 범위·SoT·증거·로봇 영향을 분리한다. [문서 검사 코드](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/scripts/check_docs.py#L1-L10)는 문서 경로 등의 검사이지 시스템 동작 판정이 아니다.

**적용:** Codex는 머지를 기다리지 않고 PR을 작성하며 피드백을 반영한 뒤 최종 검토한다. Claude는 로직·계획 검토, Cursor는 검증·수정·머지를 맡는다. 시작 시 같은 base/head SHA·원본 커밋·완료 조건을 제공하고, 수정 후 의미 있는 변경은 Claude가 다시 검토한다. 지적 번호, 근거, 반영 커밋, 재검토 범위와 의존 순서에 따른 머지는 [검토 지침](review-workflow.md)에 따라 추적한다. 공통 계약 수정은 소비 모듈과 시험의 영향을 함께 기록한다. 원본 규약 전체를 사본으로 늘리지 않고 이번 프로젝트의 역할·계약을 별도로 정한다.

**확인:** 실제 실행한 명령·원문 결과·미실행 항목을 분리한다. 문서 검사 통과를 알고리즘·ROS·로봇 검증 통과로 바꾸어 쓰지 않는다. 원본 시험 수나 과거 PASS를 현재 커밋의 PASS로 쓰지 않는다. 연결: RQ-07, 모든 계획 PR.

## 이번 선별의 결과

유지할 원리는 근거를 남기는 기록, 명령·지도 발행 책임, 관측 시각 보존, 반복 가능한 배포, 동시 부하 측정, 실제 결과에 따른 판정이다. 당시 로봇 번호·PC 경로·일정·임계값·RC카 모델 선택은 승계하지 않는다. 위 검증은 후속 구현의 완료 조건이며 이번 PR에서 실행한 시험 목록이 아니다.

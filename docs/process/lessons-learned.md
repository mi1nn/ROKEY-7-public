---
title: 원본에서 선별한 레슨런과 파이널 검증
status: draft
updated: 2026-10-15
---

# 원본에서 선별한 레슨런과 파이널 검증

원본의 실패·제약을 새 파이널에서 확인할 시험으로 바꾼다. 아래 예방 규칙은 제안이다. 과거 로봇의 속도·거리·배터리·카메라 설정값은 새 임계값으로 승계하지 않는다.

`[원본 기록]`은 링크한 날짜·장비의 과거 기록이며 이번 재측정이 아니다. P1 ~ P3 사례는 P4 회고가 재인용한 내용이다. 그 사고 원문은 이번 조사에서 확인하지 않았으므로 P4 회고의 주장으로만 인용한다. 원본 커밋과 조사 범위는 [선별표](source-selection.md)에 고정했다. 아래 PR 번호는 [계획 순서](../requirements/scope.md)의 번호다.

## LL-01 문서의 구현 현황을 코드와 대조한다

**근거:** `[원본 코드·기록]` 같은 커밋의 [SDD 현황표](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/sdd.md#L116-L124)는 AMR·브리지·웹을 없다고 적지만 [제어 노드](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/node.py#L75)와 [브리지](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_sysmon/rokey_sysmon/bridge.py#L76)가 존재한다. 상대 저장소의 [추적 설계](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/docs/RC_CAR_TRACKING_DESIGN.md#L1-L7)도 미구현 초안이라고 적지만 [추적 노드](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L91-L145)는 구현되어 있다.

**적용:** 기능마다 구현 존재, 시험 파일 존재, 해당 커밋 실행 결과, 실기 결과를 구분한다. 오래된 문서만 근거로 없거나 완성됐다고 판단하지 않는다.

**확인:** PR 1의 모든 후보에 고정한 소스와 수정 조건을 붙이고, 실제 이식 PR에서 실행 증거를 갱신한다. 이 PR은 기능 실행·실기 완료를 주장하지 않는다. 연결: AC-01 ~ 07, PR 1 ~ 10.

## LL-02 두 로봇의 전체 흐름을 초기에 연결한다

**근거:** `[원본 기록]` [P4 회고](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/reference/2026-10-03-lessons-p1-p3.md#L25-L33)는 늦은 통합과 동결 뒤 변경을 반복 문제로 적는다. [기존 운영 관문](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/plan.md#L13-L22)의 단일 로봇 기준과 날짜는 새 요구에 맞지 않는다.

**적용:** PR 5에서 두 대 동시 주행, PR 6에서 좌표 인계, PR 8에서 A 발견→B 접근을 확인한다. 종합 PR 10까지 두 로봇 연결을 미루지 않는다. 기능·모델·설정의 동결 시점은 실제 일정 확인 후 정한다.

**확인:** 각 관문에 시작 조건·관측 결과·성공 분모를 실행 전에 정하고, 동결 뒤 행동을 바꾸는 PR에는 해당 시나리오 재시험을 붙인다. 연결: RQ-05, AC-01·02·05, PR 3·5·6·8·10.

## LL-03 명령뿐 아니라 지도와 TF의 발행 책임도 정한다

**근거:** `[원본 기록]` [2026-10-06 현장 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L33)은 다른 PC의 SLAM 노드로 `/robot3/map` 발행자가 두 개가 되어 시험을 중단했다고 보고한다. [P4 명령 주체 원칙](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/reference/2026-10-03-lessons-p1-p3.md#L34-L40)도 재사용할 만하다.

**적용:** 로봇별 임무 실행기, Nav2, 속도 중재, teleop의 권한을 구분한다. 각 지도와 TF 연결의 발행자를 명시한다. 웹·탐지·협업 노드가 별도로 같은 로봇에 직접 주행 명령을 내지 않게 한다.

**확인:** 네 PC를 동시에 실행한 그래프와 중복 기동 시험에서 예상 외 발행자를 찾는다. 다른 로봇 ID 요청, 정지와 새 배정의 경합, 정지가 모든 활성 주행 경로에 도달하는지 확인한다. 연결: AC-01·06, PR 2 ~ 5·8 ~ 10.

## LL-04 실행한 코드와 자산의 버전을 기록한다

**근거:** `[원본 기록]` [DEC-018](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/decisions.md#L225-L233)은 ROS workspace가 개발 브랜치를 가리키는 문제와 설치 후 버전이 `unknown`이 되는 문제를 기록한다. [회차 기록 항목](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/test-plan.md#L41-L56)은 재현의 출발점이다.

**적용:** 네 PC에서 실행한 코드 SHA, 실제 패키지 경로, 모델·설정 해시, 지도·보정 버전, 적용 파라미터를 한 실행 기록에 묶는다. 개발용 폴더와 판정용 설치를 구분한다. 기존 workspace 경로와 설치 방식 자체를 강제하지 않는다.

**확인:** 잘못된 overlay, 계약 버전 불일치, 변경된 모델·설정을 실행 전 점검에서 식별하는지 확인한다. PR 시험과 최종 배포의 SHA를 각각 기록한다. 연결: AC-01·05, PR 3 ~ 5·7·10.

## LL-05 동시 운용 부하는 동시에 측정한다

**근거:** `[원본 기록]` [RGB·Depth 동시 수신 로그](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-07/rgbd-simultaneous-1301.txt#L1-L7)와 [비교 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-07-depth-bandwidth.md#L9-L22)은 따로 측정했을 때와 함께 구독했을 때의 지연 차이를 보고한다. 두 로봇 동시 운용을 입증한 기록은 아니다.

**적용:** 두 로봇의 영상·Depth·LiDAR·TF·지도와 웹 미리보기·기록을 함께 실행한 부하로 배치를 결정한다. 고사양 PC의 단독 추론 속도로 네트워크와 카메라 지연까지 해결됐다고 판단하지 않는다.

**확인:** 같은 장면·시간·구독자 조건에서 단독/전체 운용의 입력 나이, 처리 지연, 수신 주기, 누락, 네트워크·CPU·GPU·메모리를 비교한다. 과거 FPS·대역폭 값은 새 합격선으로 사용하지 않는다. 연결: AC-01·03·06, PR 2·3·5·7·9·10.

## LL-06 관측 시각과 다시 발행한 시각을 구분한다

**근거:** `[원본 기록]` [거리 보정 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-robot-detect.md#L23-L37)은 영상 지연 때문에 이전 거리의 상자가 섞였을 가능성과 측정 폐기를 적는다. `[원본 코드]` 상대 저장소의 [좌표 유지·재발행](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/webcam_detector.py#L100-L159)은 이전 좌표에도 현재 발행 헤더를 붙인다. [P4 좌표 변환](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/target_locator.py#L74-L105)은 관측 헤더의 시각으로 TF를 찾고, 실패하면 버린다.

**적용:** 센서 취득·수신·처리 시각과 마지막 유효 관측을 구분한다. 재발행만으로 신뢰 기간을 연장하지 않는다. RGB·Depth·TF의 시각 일치, PC 간 시계 오차, 지도 보정 이후 좌표 유효성을 계약으로 정한다.

**확인:** 지연·역순·중복 입력, TF 부재, 센서 중단, 시계 오차를 주입한다. 오래된 좌표가 새 접근 명령이나 최신 관제 표시로 바뀌지 않는지 확인한다. 수신 시각과 헤더의 차이만으로 시계 동기화가 검증됐다고 쓰지 않는다. 연결: AC-02·06, PR 3·6 ~ 10.

## LL-07 설정 일치와 공간 정확도를 따로 검증한다

**근거:** `[원본 기록]` [카메라 설정 검사](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-07/config-E-checks-1357.txt#L7-L15)는 설정 일치가 픽셀 정렬을 뜻하지 않는다고 명시한다. [Depth 정렬 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-depth-align.md#L30-L39)에는 overlay 증거가 남은 작업으로 적혀 있다.

**적용:** camera_info와 검증한 카메라 외부 보정을 사용한다. 도크 위치·RC카 크기·웹캠 보정값을 다른 로봇·장소·대상에 복사하지 않는다. 센서 거리 오차와 두 로봇의 공통 좌표 변환 오차를 분리한다.

**확인:** 로봇 각각에서 여러 거리·화면 위치·가림 조건의 RGB/Depth 겹침과 실거리 기준을 확보한다. A의 위치 추정과 B의 재관측을 같은 물리 기준점에 대조한다. 연결: AC-02·03, PR 2·3·6·7·10.

## LL-08 요청 접수와 현장 성공을 구분한다

**근거:** `[원본 기록]` [현장 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L25-L28)은 언도크가 접수된 뒤 동작이 멈춘 사례를 적는다. [시험 계획](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/test-plan.md#L30-L56)은 액션 결과와 실제 위치·도킹 관측을 구분한다.

**적용:** 발견·위치 확인·접근 완료·보고 완료·접근 불가를 구분한다. 접근 완료는 액션 결과와 신선한 위치·대상 재확인으로, 보고 완료는 관제의 수신·저장 근거로 판정하는 계약을 제안한다. 취소는 재시도 가능한 실패와 구분한다.

**확인:** 수락 후 결과 없는 액션, 취소 응답 누락, 이전 goal의 늦은 결과, 저장 실패, 접근 불가를 시험한다. UI와 로그가 관측한 수준보다 강한 성공을 표시하지 않는지 확인한다. 연결: RQ-04, AC-05 ~ 07, PR 3·5·8 ~ 10.

## LL-09 기존 모델 점수를 사람 탐지 성능으로 가져오지 않는다

**근거:** `[원본 기록]` [P4 모델 결정](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/decisions.md#L115-L122)은 `box/car` 미니 프로젝트 범위다. [모델 비교의 한계](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/model-selection-amr.md#L68-L104)에는 작은 평가 세트·학습 조건·서로 다른 loss 비교 문제가 적혀 있다. 상대 저장소의 [학습 소개](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/README.md#L1-L14)도 `car/dummy` 데이터의 결과다.

**적용:** 비교 도구와 데이터 분할·모델 해시 기록 방법을 가져온다. 사람 또는 합의한 인형, 가림·자세·조도·시점의 평가 세트를 따로 정한다. 모델·가중치는 이번 PR에서 채택하지 않는다.

**확인:** 학습/평가 간 유사 연속 프레임의 누출, 대상 클래스 누락을 점검하고 동일 조건에서 탐지 성능·위치 오차·실운용 지연을 평가한다. 라벨 없는 대리 지표를 recall이나 mAP로 보고하지 않는다. 연결: AC-03, PR 1 ~ 3·7·10.

## LL-10 교차 검증의 대상과 증거 범위를 명확히 남긴다

**근거:** `[원본 기록]` [P4 협업 회고](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/reference/2026-10-03-lessons-p1-p3.md#L47-L51)는 같은 파일·결함에 병렬 PR이 겹치고 머지 뒤 push가 생긴 문제를 적는다. [PR 템플릿](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/.github/pull_request_template.md#L1-L34)은 변경 범위·SoT·증거·로봇 영향을 분리한다. [문서 검사 코드](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/scripts/check_docs.py#L1-L10)는 문서 경로 등의 검사이지 시스템 동작 판정이 아니다.

**적용:** 작성과 검토는 병렬로 진행하며, 담당과 인계 순서는 [현재 검토 절차](review-workflow.md#current-review-workflow)를 따른다. 시작 시 같은 base/head SHA·원본 커밋·완료 조건을 제공하고, 수정 후에는 독립 검토 담당이 변경과 영향 범위를 다시 확인한다. 지적 번호, 근거, 반영 커밋, 재검토 범위와 의존 순서에 따른 머지는 [검토 지침](review-workflow.md)에 따라 추적한다. 공통 계약 수정은 소비 모듈과 시험의 영향을 함께 기록한다. 원본 규약 전체를 사본으로 늘리지 않고 이번 프로젝트의 역할·계약을 별도로 정한다.

**확인:** 실제 실행한 명령·원문 결과·미실행 항목을 분리한다. 문서 검사 통과를 알고리즘·ROS·로봇 검증 통과로 바꾸어 쓰지 않는다. 원본 시험 수나 과거 PASS를 현재 커밋의 PASS로 쓰지 않는다. 연결: RQ-07, 모든 계획 PR.

## LL-11 완료까지 멈추는 도우미 호출을 실행기에 두지 않는다

**근거:** `[원본 코드]` 강사 예제 [3_1_a_nav_to_pose.py](https://github.com/kimandreas/to_students/blob/ad411c4ee4dd99fa4c9e2c62d487103099918b6e/day3/3_1_a_nav_to_pose.py#L35-L51)는 `undock()`·`startToPose()` 를 차례로 부르고, 이 도우미들은 완료까지 호출자를 멈춘다([turtlebot4_navigator.py undock](https://github.com/turtlebot/turtlebot4/blob/7fd29fb420e906f3aca4a904adb54b69b11c7c00/turtlebot4_navigation/turtlebot4_navigation/turtlebot4_navigator.py#L176-L181), [startToPose](https://github.com/turtlebot/turtlebot4/blob/7fd29fb420e906f3aca4a904adb54b69b11c7c00/turtlebot4_navigation/turtlebot4_navigation/turtlebot4_navigator.py#L266-L275)). 한 로봇의 목표를 기다리는 동안 같은 프로세스는 다른 로봇의 취소·정지·관측을 처리하지 못한다. P4 의 [goal_timer.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/goal_timer.py#L83-L173)가 수락·결과·취소를 나눈 이유가 여기 있다(P4-02).

**적용:** 실행기는 액션 클라이언트를 비동기로 쓰고, 취소와 정지 확인을 별도 경로로 둔다([계약](../design/contracts.md)의 정지·취소 의미, [상태 기계](../design/state-machines.md)). 교육용 도우미는 실기 튜토리얼과 수동 점검에만 쓴다.

**확인:** 한 로봇의 목표가 수락된 채 결과가 없는 동안 다른 로봇의 취소·정지 요청과 heartbeat 처리가 지연 없이 이루어지는지, 같은 로봇의 정지 요청이 진행 중 목표를 취소하고 정지 증거를 남기는지 시험한다. 연결: TS-06, P4-02, [인수 계획](acceptance-plan.md) CT-08, 로봇·지도 Nav2 어댑터 PR.

## 이번 선별의 결과

유지할 원리는 근거를 남기는 기록, 명령·지도 발행 책임, 관측 시각 보존, 반복 가능한 배포, 동시 부하 측정, 실제 결과에 따른 판정이다. 당시 로봇 번호·PC 경로·일정·임계값·RC카 모델 선택은 승계하지 않는다. 위 검증은 후속 구현의 완료 조건이며 이번 PR에서 실행한 시험 목록이 아니다. LL-11 은 2026-10-08 강사 예제 검토([선별표](source-selection.md) TS-06)에서 더했다.

## 파이널 현장에서 더한 레슨런 (2026-10-15)

아래는 원본이 아니라 이 저장소의 실습에서 나왔다. 근거는 [2026-10-15 실습 기록](../evidence/field-2026-10-15-run1015.md)이고, 그 기록의 B 번호로 잇는다.

## LL-12 「준비됨」은 실제로 일할 수 있는 상태를 본다

**근거:** Nav2 의 bt_navigator 가 inactive 인데 실행기는 `nav2_state=READY` 를 냈고 출발 승인이 통과됐다. 준비 판정이 액션 서버가 그래프에 있는지만 봤기 때문이다(B9). 준비 점검 카메라는 로봇이 내지 않는 미리보기 토픽을 보고 있었고(B7), 깊이 카메라는 발행자가 있는데 0 Hz 였다(B8).

**적용:** 준비 판정은 「있다」가 아니라 「지금 동작한다」를 본다. Nav2 는 lifecycle 이 모두 active 인지, 센서는 메시지가 기준 시간 안에 오는지 본다. 판정이 기대는 토픽은 로봇 설정에서 실제로 켜져 있는 것으로 고른다.

**확인:** 액션 서버는 있지만 lifecycle 이 inactive 인 경우에 출발 승인이 거절되는 시험. 실습 순서의 언도킹 뒤 RGB·깊이 확인 줄. 연결: B7·B8·B9, [실습 안내](field-runbook.md) 2·7단계.

## LL-13 로봇이 움직이지 않았는데 임무가 끝나면 안 된다

**근거:** 첫 탐색 목표가 Nav2 에서 거절되자 조정기는 그 경계를 「가 봤음」으로 막았고, 남은 경계가 없어 몇 초 만에 `NO_SURVIVOR_FOUND` 로 끝냈다. TB-6 은 한 걸음도 움직이지 않았다(B10).

**적용:** 로봇 쪽 실패(거절, 시간 초과, 준비 안 됨)와 「그곳에 갈 수 없음」을 구분한다. 앞의 것은 다시 시도하고 막힘을 화면에 보인다. 임무 결론(탐색 끝, 생존자 없음)은 실제로 수행한 결과에서만 낸다. AGENTS.md 의 「조용한 대체 동작 없음」과 같은 원리다.

**확인:** 첫 목표가 거절된 뒤 후보가 없을 때 탐색이 끝나지 않는 시험(#338). 연결: B10.

## LL-14 서 있는 로봇의 위치도 나와야 한다

**근거:** slam_toolbox 는 기본 설정에서 0.1 m·0.1 rad 이상 움직인 스캔만 처리해서, 출발 자리에 서 있는 TB-6 의 위치가 나오지 않았다. 그래서 출발 자세가 잡히지 않았다. 서 있어도 처리하게 바꾸자 2 Hz 로 나왔다(B3). 백그라운드로 띄운 slam_toolbox 는 inactive 에 머물기도 했다(B4).

**적용:** 출발 자세처럼 「서 있을 때」 필요한 값은 그 조건에서 나오는지 먼저 확인한다. 외부 패키지의 기본 설정은 우리 흐름의 조건(정지, 출발 자리)에 맞는지 본다. lifecycle 노드는 띄운 뒤 active 인지 확인한다.

**확인:** 저장소에 둔 TB-6 SLAM 설정으로 서 있는 상태에서 `/robot6/pose` 가 나오는지, 띄운 뒤 active 인지 실습 순서에서 본다. 연결: B3·B4.

## LL-15 무선은 한 줄이고, 매번 확인해야 하는 설정은 빠진다

**근거:** 로봇 Pi 무선 절전은 재부팅하면 다시 켜진다. 실습 안내에 「켤 때마다 끄기」가 있었지만 빠뜨렸고, TB-6 이 `on` 이었다. 같은 시간대에 TB-5 가 망에서 거의 떨어졌고 ping 이 1초 넘게 튀었다. 무거운 영상이 늘면(준비 점검 카메라를 큰 RGB 로 바꿈, 로봇당 약 1.5 MB/s) 다른 PC 와 로봇의 지연도 함께 나빠졌다(B5).

**적용:** 매번 손으로 해야 하는 설정은 확인 줄로 실습 순서의 맨 앞에 둔다. 영상은 받는 프로세스마다 Wi-Fi 로 한 번씩 더 오므로, 같은 영상을 두 곳이 받지 않게 하고 도착만 보면 되는 곳에는 가벼운 토픽을 쓴다.

**확인:** 실습 전 Pi 두 대의 `power_save` 값을 기록에 남긴다. 회차마다 로봇 담당 PC 수신량과 무거운 토픽의 구독자 수를 기록한다. 연결: B5·B7, [LL-05](#ll-05-동시-운용-부하는-동시에-측정한다).

## LL-16 다시 띄우는 순서를 정해 둔다

**근거:** Pi 의 turtlebot4 서비스를 다시 시작한 뒤 Create3 연결이 돌아오지 않아 언도킹이 멈췄다(B6, 두 로봇). 지도 고정 노드만 다시 띄우자 새 출발 자세가 생겨 조정기가 `START_POSE_CONFLICT` 로 거절했다(B11). Discovery 전용 PC 를 재부팅하자 서버가 30분 동안 꺼져 있었다(B1).

**적용:** 무엇을 다시 띄우면 무엇을 함께 다시 띄워야 하는지 실습 순서에 적는다. 다시 띄운 뒤에는 그 노드가 내는 것(도킹 상태, 출발 자세, Discovery 목록)이 실제로 오는지 확인하고 넘어간다. 꺼지면 안 되는 서버는 서비스로 등록한다.

**확인:** 실습 안내 2·6·8단계의 확인 줄. Discovery 서버 서비스 등록(재범 승인 대기). 연결: B1·B6·B11.

## LL-17 같은 망의 다른 노드는 우리 그래프에 들어온다

**근거:** 실습 중 다른 PC 의 지도 서버와 AMCL 이 `/robot6/map` 에 14분 동안 발행했고, 그 지도가 관제 웹에 우리 지도처럼 들어왔다(B2). [LL-03](#ll-03-명령뿐-아니라-지도와-tf의-발행-책임도-정한다)과 같은 사고가 파이널에서 다시 났다.

**적용:** 회차 전 배선 점검에서 지도 토픽의 발행 노드가 `slam_toolbox` 하나인지 본다. 팀원에게 실습 시간에는 같은 도메인에서 로봇 이름이 들어간 노드를 띄우지 말라고 알린다.

**확인:** 배선 점검 결과 JSON 에 지도 발행자 수와 이름을 남긴다. 연결: B2.

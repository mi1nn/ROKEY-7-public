---
title: 두 원본의 코드·알고리즘·시험 선별
status: draft
updated: 2026-10-08
---

# 두 원본의 코드·알고리즘·시험 선별

## 결론과 판단 수준

두 저장소를 합쳐 그대로 실행하는 방식은 채택하지 않는다. **P4의 액션 관리·영상/거리 처리·좌표 변환·관제 기록을 우선 후보로, amr-mini-project의 모델 평가·보정 도구와 추종 실패 경험을 보완 자료로 선별한다.** 두 로봇의 지속 수색, 공통 좌표, 지도 보정 후 대상 위치, 협업 배정은 새 설계가 필요하다.

위 내용은 `[제안]`이다. 이 PR은 원본 코드를 이식하거나 실행하지 않는다. 아래 `[원본 코드]` 근거는 정적 읽기이며, 위험은 해당 구현을 새 요구에 적용할 때의 추론이다. 실제 사고를 재현했거나 전체 저장소에 다른 구현이 없다는 주장이 아니다. 최종 채택과 상세 알고리즘은 계획 PR 2·3, 동작 검증은 각 이식 PR에서 수행한다.

## 조사 기준과 범위

| 저장소 | 고정한 커밋 | 추적 파일 | Python 파일 | ROS 패키지 | `test_*.py` 파일 |
| --- | --- | ---: | ---: | ---: | ---: |
| [ROKEY_P4](https://github.com/jaebeom/ROKEY_P4/tree/1a2d0a9811f59a501c7597edf54c72fec8a80e26) | `1a2d0a9811f59a501c7597edf54c72fec8a80e26` | 335 | 184 | 7 | 75 |
| [amr-mini-project](https://github.com/mi1nn/amr-mini-project/tree/ee355d0b5896ea58bccac73ce1a7857f62e96024) | `ee355d0b5896ea58bccac73ce1a7857f62e96024` | 477 | 65 | 5 | 15 |
| [ROKEY-7 기준 main](https://github.com/jaebeom/ROKEY-7/tree/4f5453da4425cb80410d6b838c7df2a19d62509f) | `4f5453da4425cb80410d6b838c7df2a19d62509f` | 1 | 0 | 0 | 0 |

집계는 `git ls-files -z` 기준이다. 패키지 수는 `package.xml`, 시험 파일 수는 파일 이름 기준이며 **시험 건수·통과 건수가 아니다.** 전체 파일 목록과 아래 관련 코드를 조사했으며, 모든 바이너리·그림·학습 결과를 내용까지 검증한 전수 감사는 아니다. 구조화된 집계는 [source-audit.json](evidence/source-audit.json)에 둔다.

조사에 포함한 것은 위 main 스냅샷의 추적 경로, 핵심 실행 경로, 관련 시험·설정·설계·현장 기록이다. 다른 브랜치, 모든 이슈·PR, 외부 데이터셋, 실제 설치 환경·로봇은 포함하지 않았다. P4는 비공개 저장소이므로 링크 검증자에게 원본 접근 권한이 필요하다.

### 최상위 경로 처리

| 원본 / 경로 (파일 수) | 확인·선별 방식 |
| --- | --- |
| P4 루트 (4), `.github/` (3) | README·작업 규약·PR 형식·CI 범위 확인. 개인/기존 과제 규약과 workflow를 통째로 복사하지 않음 |
| P4 `src/` (126) | 인터페이스, AMR 제어·배정, 관제·DB·지도 표시와 관련 시험을 핵심 분석 |
| P4 `homework/` (99) | day2 vision·학습/평가 도구와 시험만 후보. day1 예제와 과제 산출물 일괄 이식 제외 |
| P4 `mini/` (45) | 거리·평면 보정 수학, 실행 경로의 제약, 현장 기록을 분석. 추종 시연 프로그램 일괄 이식 제외 |
| P4 `config/` (13), `scripts/` (5), `tools/` (1) | 파라미터·사전 점검·평가 도구를 비교. 과거 로봇 ID·도크·경로·임계값과 RC카 대리 평가 점수는 승계하지 않음 |
| P4 `docs/` (37), `evidence/` (2) | 관련 설계·결정·시험·회고를 코드와 대조. 과거 기록은 새 실기 증거로 사용하지 않음 |
| AMR 루트 (16), `src/` (77) | 추종·웹캠·보정·평가 코드와 패키지 선언 확인. 교육 패키지·하드코딩 실행 경로 제외 |
| AMR `docs/` (3) | 설계/운용 문서를 실제 코드와 대조 |
| AMR `runs/` (375) | 학습 결과 자산 목록과 일부 설정·지표 확인. 전체 성능 재평가 없이 이식하지 않음 |
| AMR `maps/` (4), `bags/` (2) | 기존 지도와 bag 메타데이터 확인. 새 현장 지도·완전한 재생 입력으로 간주하지 않음 |

## 분류 기준

- **수정 후 이식 후보:** 재사용할 구현이 있지만 새 계약·장비·시험에 맞춰 바꿔야 한다. 현재 채택 완료가 아니다.
- **원리 참고:** 상태·알고리즘·운영 경험만 참고한다. 기존 노드·정책을 그대로 복사하지 않는다.
- **조건부 후보:** 해당 센서/기능을 후속 설계에서 채택할 때만 검토한다.
- **제외:** 현재 파이널의 기본 코드·모델·설정으로 가져오지 않는다. 원본은 삭제하지 않는다.
- **신규 설계:** 검토한 경로에 새 요구를 충족하는 완성 구현이 확인되지 않았다. 표준 ROS 구성요소 연동도 포함하며 SLAM 자체를 처음부터 구현한다는 뜻은 아니다.

## P4에서 가져올 후보

아래 링크는 모두 위 P4 커밋에 고정했다. 시험 링크는 **파일의 존재·검증 의도**를 뜻한다. 이번에 실행하지 않았다. 경로 전체를 옮기는 것이 아니라 명시한 책임과 필요한 시험만 선별한다.

| ID / 기능 | 분류와 원본 근거 | 가져올 부분 / 반드시 바꿀 부분 | 관련 시험 / 후속 계획 PR |
| --- | --- | --- | --- |
| P4-01 임무·상태 계약 | **원리 참고** · [Task.msg](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_interfaces/msg/Task.msg#L1-L60), [Alert.msg](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_interfaces/msg/Alert.msg#L1-L47) | 임무 ID·세대·만료·취소 의미 참고. 고정 구역·AMCL 전제를 제거하고 대상 사건·관측 시각·좌표 기준·보고 계약을 새로 정의 | [adapters 시험](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/test/test_adapters.py#L278-L311) / 2·3·4 |
| P4-02 액션 시간 제한·취소 | **수정 후 이식 후보** · [goal_timer.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/goal_timer.py#L83-L173) | 서버 대기·수락·결과·취소의 구분과 늦은 수락 취소. 새 frame/goal 계약과 정지 확인을 연결. 기본 timeout을 측정값처럼 승계하지 않음 | [늦은 수락 시험](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/test/test_goal_timer.py#L161-L176) / 5·8 |
| P4-03 임무 실행·입력 검증 | **원리 참고**, 일부 수정 후 이식 · [controller.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/controller.py#L175-L210) | 로봇 ID·세대·중복·만료·종료 관문. AMCL·도크·구역 확인 FSM을 수색·재확인·접근·보고·재개 흐름으로 재설계 | [controller 시험](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/test/test_controller.py#L643-L748) / 3·8 |
| P4-04 로봇 배정 | **원리 참고 + 신규 설계** · [dispatcher.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/dispatcher.py#L253-L276) | 재할당 세대·취소 의미만 참고. 도킹 완료 상태(DOCKED)의 로봇·도크 직선거리·고정 구역·단일 대기 사건 정책을 지속 수색 정책으로 교체 | [기존 선택 시험](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/test/test_dispatcher.py#L148-L156) / 2·3·8 |
| P4-05 YOLO 입력·출력·모델 확인 | **수정 후 이식 후보** · [yolo_infer.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/rokey_vision/rokey_vision/yolo_infer.py#L80-L174) | 토픽/모델/device 설정과 헤더·해시 보존. 대상 class·tracking 의존성을 새 계약과 일치시키고 실패를 표시 | [메시지 시험](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/rokey_vision/test/test_msgs.py#L11-L53) / 7 |
| P4-06 RGB/Depth·거리 역투영 | **수정 후 이식 후보** · [common.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/rokey_vision/rokey_vision/common.py#L172-L251), [depth_distance.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/rokey_vision/rokey_vision/depth_distance.py#L113-L132) | 압축 Depth·유효값 중앙값·동기화·CameraInfo 처리. 실제 RGB/Depth 정렬, 센서 시각, 실거리 오차 재검증 | [공통 함수 시험](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/rokey_vision/test/test_common.py#L84-L148) / 7 |
| P4-07 관측점의 지도 변환 | **수정 후 이식 후보** · [target_locator.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/target_locator.py#L74-L107) | 관측 시각의 TF 조회·유효 Depth 검사·기하 함수. 공통 좌표와 지도 보정 이후 유효성, 지연 관측 거절을 추가 | [geometry 시험](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/test/test_geometry.py#L43-L96) / 6·7 |
| P4-08 연속 확인·중복 억제 | **원리 참고 + 신규 설계** · [alert_rules.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/rokey_vision/rokey_vision/alert_rules.py#L38-L87), [zone_dwell.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/rokey_vision/rokey_vision/zone_dwell.py#L160-L240) | 시간 불연속·재확인 원리. 동일 클래스 연속 탐지를 동일 사람 확인으로 간주하지 않음. 다중 대상·센서별 ID·로봇 간 사건 연결 정의 | [track·시간 시험](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/rokey_vision/test/test_zone_dwell.py#L77-L118) / 3·7·8 |
| P4-09 관제·저장·증거 매칭 | **수정 후 이식 후보** · [bridge_core.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_sysmon/rokey_sysmon/bridge_core.py#L189-L236), [evidence.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_sysmon/rokey_sysmon/evidence.py#L24-L104) | 입력별 최신성·명령 단계·기록·관측 시각의 사진 매칭. 두 로봇의 사건/보고 스키마와 PC 간 요청·저장 경로 재설계 | [가짜 두 로봇 시험](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_sysmon/test/test_integration_fake_robots.py#L1-L52), [증거 시험](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_sysmon/test/test_evidence.py#L20-L104) / 9 |
| P4-10 지도 표시 | **수정 후 이식 후보** · [mapsnap.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_sysmon/rokey_sysmon/mapsnap.py#L127-L142) | 표시용 이미지·메타데이터의 원자 교체와 픽셀 변환. AMCL 경유 위치, 단일 지도·원점 yaw 제약을 새 표시 계약에 맞춤 | [교체 일관성 시험](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_sysmon/test/test_mapsnap.py#L116-L163) / 9 |
| P4-11 미니의 접근·보정 수학 | **원리 참고 / 고정 카메라는 조건부** · [goal_math.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/rokey_mini/rokey_mini/goal_math.py#L20-L34), [homography.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/rokey_mini/rokey_mini/homography.py#L1-L49) | 정지거리·목표 이동량·평면 보정/퇴화 검사. 도달 가능성·안전 접근점과 구분. 추종 노드/launch 일괄 이식은 제외 | [합성 평면 시험](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/rokey_mini/test/test_webcam_trigger.py#L14-L34) / 7·8 |

P4의 `scripts/preflight.py`와 학습·평가 도구는 배포/평가 구현 시 추가 추출 후보다. 과거 특정 PC·모델·설정의 점검 기준을 새 4 PC 구성에 그대로 강제하지 않는다. `homework/day2`라는 경로는 P4-05·06을 버릴 이유가 아니다.

## amr-mini-project에서 가져올 후보

| ID / 기능 | 분류와 원본 근거 | 가져올 부분 / 반드시 바꿀 부분 | 후속 계획 PR |
| --- | --- | --- | --- |
| AMR-01 최신 영상 수신·추론 분리 | **수정 후 이식 후보** · [yolo_detector.py](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/yolo_detector.py#L20-L60) | 최신 메시지만 유지하는 수신 패턴을 P4 입력과 비교. 영상 표시를 넘어 관측 헤더·결과 계약·지연 계측 필요. 두 구현을 중복 운영하지 않음 | 7 |
| AMR-02 Depth 중앙값·Nav2 목표 갱신 | **원리 참고** · [depth_at](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L482-L509), [go_near](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L388-L446) | 유효 거리 집계·목표 전송 빈도 억제 참고. P4-06과 비교해 한 경로만 채택. 고정 FOV·시각 혼합·대상 자체 goal은 새 설계로 대체. 취소 이후 실제 정지는 별도 검증 | 5·7·8 |
| AMR-03 웹캠 평면 좌표·보정 도구 | **조건부 후보** · [webcam_detector.py](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/webcam_detector.py#L91-L159), [calibrate_webcam_v2.py](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/calibrate_webcam_v2.py#L90-L165) | 고정 외부 카메라를 채택할 때만 대응점 기록·RANSAC·재투영 오차 도구 검토. 현재 H·지도·보정점은 제외. 두 AMR 수색의 필수 의존성으로 만들지 않음 | 2·7 |
| AMR-04 모델 비교·통계 | **수정 후 이식 우선 후보** · [yolo_report_utils.py](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/yolo_report_utils.py#L136-L201), [yolo_compare.py](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/yolo_compare.py#L68-L119) | 클래스별 TP/FP/FN·threshold·run 비교. CLI 진입점·동일 평가 데이터/장비/버전·실험 ID 명시. 추론 단계 시간을 센서→판단 전체 지연으로 사용하지 않음 | 7 |
| AMR-05 bag 이미지 추출 | **수정 후 이식 후보** · [extract_images.py](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/extract_images.py#L6-L27) | 토픽·간격·출력 경로를 인자로 변경. 원본 bag·로봇·시각·촬영 세션 기록 후 데이터 분할. 원본 robot6 고정값 제외 | 7 |
| AMR-06 Nav2 설정·복구 경험 | **원리 참고** · [nav2.yaml](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/nav2.yaml#L81-L181), [nav_recovery_fetch.sh](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/nav_recovery_fetch.sh#L59-L94) | obstacle marking/clearing·상태 진단 항목. lifecycle 관리 책임과 새 장비/버전/임계값을 정한 뒤 검증. 복구 스크립트를 자동 실행 경로로 이식하지 않음 | 2·5 |

`src/*/test/`의 15개 시험 파일은 copyright·flake8·pep257 계열이며 copyright 검사는 skip 선언이다. [rokey_pjt 시험](https://github.com/mi1nn/amr-mini-project/tree/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/test)이 대표적이다. 이 범위와 핵심 코드의 `pytest|unittest|assert |def test_|launch_testing` 검색에서 추종·취소·좌표 공유의 행동 시험은 확인하지 못했다. 원본 밖의 시험·현장 경험은 미확인이다. 후보 도구도 기능 시험을 새로 갖춰야 한다.

## 통째로 이식하면 안 되는 구체적 이유

### F-01 AMCL 제어기를 지속 SLAM 제어기로 그대로 사용할 수 없다

`[원본 코드]` P4 [node.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/node.py#L125-L132)와 [FSM](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/fsm.py#L770-L815)은 AMCL 자세·초기 자세·공분산 준비 상태에 결합되어 있다. 상대 저장소도 [기동 경로](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L109-L123)에서 AMCL 대기 후 도킹 상태이면 언도킹·고정 초기 자세 설정을 수행한다.

`[제안]` 위치 제공자·준비 조건·임무 정책을 분리한다. 토픽 이름만 바꾸는 것으로 완료하지 않는다. SLAM 상태에서 기동·주행·도착·관제가 같은 위치 계약을 사용하는지 검증한다. 연결: AC-01·04, PR 2·3·5·8·9.

### F-02 관측 시각 TF와 데이터 최신성은 별개다

`[원본 코드]` P4-07은 **관측 시각의 TF**를 조회한다. 최신 TF로 대체한다고 해석하면 잘못이다. 그러나 [수신 콜백](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/node.py#L163-L187)은 `time.monotonic()` 값을 얻고 [controller](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/controller.py#L161)가 어댑터에 전달하여 `rx`로 보존한다. `rx`는 헤더 `stamp`와 다른 시계이므로 둘을 빼서 관측 나이를 계산할 수 없다. 상대 저장소의 웹캠은 유지 중인 과거 좌표를 새 헤더 시각으로 재발행하며, [수신기](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L290-L304)는 수신 시각으로 freshness를 기록한다.

`[제안]` 관측·수신·처리 시각과 공통 시계 조건을 정의한다. P4 변환기는 전달받은 헤더를 조회에 사용하므로 헤더 시각 자체의 유효성 검사도 필요하다. 0·미래·과도하게 오래된 헤더, 지연·역순·TF 유실·시계 오차를 시험하고 재발행이 관측의 수명을 연장하지 않게 한다. 지도 보정 뒤 저장 좌표의 처리도 별도 계약으로 정한다. 연결: AC-02·06, PR 3·6~9.

### F-03 만료된 대상과 접근 목표를 함께 관리해야 한다

`[원본 코드]` 상대 저장소의 [approach](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L517-L528)는 대상이 없거나 오래되면 반환한다. 그 경로에서 기존 Nav2 goal을 취소하지 않으며, `go_near` 안의 거리 취소 검사에도 도달하지 않는다. AMR-02의 `go_near`는 대상 좌표 자체를 목표로 보내고 feedback/거리에서 취소를 시도한다. P4-11의 직선 정지점 수학 역시 비용지도상 도달 가능성을 판정하는 함수는 아니다.

`[원본 코드]` P4 [mission의 NAVIGATE](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/rokey_mini/rokey_mini/mission.py#L123-L139)는 웹캠 좌표 소실만으로 goal을 취소하지 않고 nav 실패·임무 시간 초과에서 취소한다. [소실 시 유지 시험](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/rokey_mini/test/test_webcam_trigger.py#L68-L75)이 이 동작을 고정한다. 이는 위 AMR의 만료 반환과 구별한다. 원본 mission의 임무 timeout을 파이널의 관측 최신성 timeout과 동일시하지 않는다.

`[제안]` 짧은 소실에는 최신성·정지 조건이 유효한 제한 시간 동안 기존 goal만 유지하고 새 goal은 내지 않는다. 관측 만료 시에는 취소·정지 확인을 요구한다. 대상 만료 시 활성 goal 처리, 취소 완료·실제 정지 관측, 도달 가능한 접근점, 제한된 재시도를 한 제어 흐름으로 설계한다. 위 코드는 정적 위험 근거이며 실기 충돌을 재현했다는 주장은 아니다. 연결: AC-05~07, PR 3·5·8.

### F-04 직접 속도 명령과 충돌 설정을 승계하지 않는다

`[원본 코드]` P4 미니의 [추종 launch](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/rokey_mini/launch/follow_nav.launch.py#L53-L61)는 별도 추종 서버에 `controller.use_collision_detection: False`를 지정한다. 상대 저장소의 [직접 회전](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L355-L361)은 기본 `cmd_vel`로 나가며, [설정의 collision monitor](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/nav2.yaml#L259-L265)는 `cmd_vel_smoothed`를 입력, `cmd_vel`을 출력으로 둔다. 이 기본 배선에서는 직접 회전이 해당 입력을 거치지 않는다. 실제 설치 remap과 실행 배선은 확인하지 않았다.

`[원본 코드]` 별도 [car_follower](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/rokey_mini/rokey_mini/car_follower.py#L1-L14)는 기본 `cmd_vel` 직접 발행과 `predict_s` 동안의 소실 예측을 사용한다. `[원본 기록]` [실기 명령](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L45-L59)은 `follow.launch.py`였으며, [미니 안내](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/README.md#L3-L5)는 웹 정지가 이 노드를 멈추지 못한다고 명시한다. `follow_nav` 쪽은 [주석과 기본 인자](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/rokey_mini/launch/follow_nav.launch.py#L1-L34)에 collision monitor 사용 시 `cmd_vel_smoothed`로 지정하라는 안내가 이미 있다. 실제 실행 노드와 배선을 따로 확인해야 한다. P4-11의 통째 이식 제외 판단은 유지한다.

`[제안]` 로봇별 명령·정지 경로와 발행 책임을 정의한다. UI·탐지·협업이 별도로 같은 로봇을 움직이지 않게 하며, 모든 활성 경로에서 정지가 적용되는지 실제 그래프로 검증한다. 연결: AC-01·06, PR 2~5·8.

### F-05 화면 탐지·대상 식별·협업 배정은 서로 다른 기능이다

`[원본 코드]` P4 [tracking fallback](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/rokey_vision/rokey_vision/yolo_infer.py#L111-L115)은 의존성 부재 시 일반 탐지로 바뀌지만 [제어 어댑터](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/adapters.py#L87-L105)는 정수 ID 없는 탐지를 버린다. P4-01의 track ID는 소스 내부 ID이고, P4-04 배정은 DOCKED 로봇의 도크→구역 직선거리 기준이다.

`[제안]` 입력 class·ID·시각·frame·의존성을 시작 전 검사한다. 다른 로봇의 track ID를 그대로 동일 사건으로 합치지 않는다. 수색 중인 로봇의 현재 상태·위치·접근 가능 경로로 배정하는 정책을 새로 정한다. 연결: AC-03·05, PR 2·3·7·8.

### F-06 지도 그림과 가짜 두 로봇 시험은 SLAM 통합 증거가 아니다

`[원본 코드]` P4-10은 표시용 스냅샷이고 [브리지 입력](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_sysmon/rokey_sysmon/bridge.py#L152-L161)은 단일 map 토픽이다. P4-09의 통합 시험은 ROS 없이 가짜 로봇→DB→Flask를 확인한다. AMR의 지도 파일·Nav2 obstacle 설정도 두 SLAM 좌표 정합 구현을 의미하지 않는다.

`[제안]` 공통 물리 기준점, map/odom/base TF 책임, 지도 보정 이후 대상 위치, 두 로봇의 통행·인계를 새로 검증한다. PC 사이 DB/API/영상/지도 전달 방식은 계획 PR 2에서 정한다. SQLite 파일을 네 PC가 네트워크로 공유하는 배치를 이미 채택한 것은 아니다. 연결: AC-01·02·04·05, PR 2·5·6·8·9·10.

## 신규 설계로 남길 기능과 선택 기준

다음은 검토한 핵심 경로에서 새 요구를 만족하는 완성 구현을 확인하지 못한 기능이다. P4의 `src/`, vision, mini, config/scripts에 대한 SLAM·frontier·map merge·reservation·traffic·loop closure·pose graph 검색과 관련 코드 읽기, AMR의 위 핵심 경로를 근거로 한다. 저장소 밖의 구현 부재를 단정하지 않는다.

`[원본 기록]` 이미 [2026-10-06 SLAM·Nav2·explore_lite 운용](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L18-L28)이 있었다. [DEC-011](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/decisions.md#L153-L160)은 SLAM·Nav2의 PC 실행을 확정하고 Wi-Fi 단절 시 속도 명령 단절과 두 로봇 합산 대역폭 미측정을 남겼다. [Nav2 플러그인 검토](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/reference/2026-10-06-nav2-plugins-adoption.md)는 PR 2의 기존 패키지 우선 평가 입력이다. `explore_lite`를 수색 비교 후보에 포함하되 두 로봇 협업 구현 완료로 해석하지 않는다. 이 기록은 Q-02·Q-03 및 AC-06의 입력이다.

| 신규 설계 | PR 2·3에서 비교·정할 내용 | 후속 검증 |
| --- | --- | --- |
| 지속 SLAM·Nav2 결합 | 실제 장비/버전에 맞는 표준 구성요소, 위치 준비 조건, 장애물 변화와 SLAM 지도 갱신의 역할 구분 | 두 대 동시 기동, 장애물 추가·제거, 위치 추정 실패 (PR 5) |
| 공통 좌표와 지도 보정 | 한 지도/각 로봇 지도 등 대안의 정렬 조건·계산/통신 비용·실패 처리. 자동 지도 병합을 기본 전제로 확정하지 않음 | 같은 물리점의 A/B 오차, 지도 보정·TF 중단 뒤 좌표 유효성 (PR 6) |
| 수색·배정·통행 | 수색 구역 분담, 현재 위치/경로/상태 기반 배정, 다중 사건 보존, 통로 충돌·교착 처리 | A 발견→B 접근 중 A 수색 지속, 접근 불가·취소·재배정 (PR 8) |
| 대상 사건·안전 접근 | 로봇별 관측과 동일 사건 연결, 재확인, 비용지도와 도달 가능성을 고려한 정지점, 만료·취소 | 가림·중복·대상 소실·오래된 좌표·막힌 접근점 (PR 7·8) |
| 4 PC 실행·보고 | 프로세스·명령·TF·DB 쓰기 책임, 통신·시계·계약 버전, 기록과 관제 경로 | 네 PC 동시 부하, 끊김/복구, 보고 수신·저장 (PR 4·9·10) |

고사양 PC 한 대씩을 각 로봇에 배정하는 안은 후보이며 장비 정보·동시 측정 전 확정하지 않는다. A 탐지→B 접근은 협업을 입증할 시나리오다. 영구 역할 고정 알고리즘이 아니다.

## 제외 자산·출처·재현성

- `homework/`, `mini/`, AMR의 `day3/pub_sub/test_nv2/turtlebot4_beep` 패키지를 통째로 가져오지 않는다. 위 후보가 필요로 하는 함수·시험·설정만 이식 PR에서 출처와 함께 추출한다.
- 기존 `car/box/dummy` 가중치·학습 zip·`runs/`, 고정 도크 좌표·로봇 번호·개인 홈 경로, 기존 `maps/`·`webcam_H.npy`·보정점·거리 계수는 새 실행 기본값에서 제외한다. 새 평가나 fixture에 필요하면 별도 목적·버전·조건을 검토한다.
- AMR [package.xml](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/package.xml#L8-L14)과 [setup.py](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/setup.py#L9-L19)는 실제 코드의 모든 실행 의존성·설정을 규정하지 않는다. 해당 추적 스냅샷에서 launch·CI·requirements/lock/pyproject는 확인하지 못했다. 원본 장비에서 실행 불가능하다는 뜻은 아니며, 새 설치는 PR 4에서 의존성과 자산 경로를 명시해야 한다.
- P4 루트의 LICENSE/NOTICE 계열 파일은 추적 목록에서 확인하지 못했다. `package.xml` 7개 중 Apache-2.0은 `mini/rokey_mini` 1개이며, 나머지 6개(`rokey_amr`, `rokey_interfaces`, `rokey_sysmon`, `rokey_vision`, `rokey_bipbop`, `rokey_pubsub`)는 TODO다. AMR 루트는 [Apache-2.0](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/LICENSE#L1-L3), `package.xml` 5개(`rokey_pjt`, `day3`, `pub_sub`, `test_nv2`, `turtlebot4_beep`)는 모두 `TODO: License declaration`이다. 선언 차이를 기록하고 실제 코드 이식 시 원본 출처·고지 처리를 확인한다. 이 PR은 코드·미디어·모델을 복사하지 않는다.

## 이 PR의 검증 한계와 검토 질문

증거 수준은 **1: 문서·정적 분석**이다. 원본 시험 파일과 과거 기록을 읽었으며, pytest·colcon·ROS·모델·rosbag 재생·실기 시험은 실행하지 않았다. 현재 목적지에는 이식된 기능이 없다. 문서 경로·고정 소스·집계 검사는 기능 통과로 해석하지 않는다.

Claude는 전체 로직·계획, Cursor는 원본 대조·검증·수정·머지를 맡는다. 같은 base/head에서 다음을 각 역할에 맞게 확인하고, 수정 이후 재검토·머지는 [검토 지침](review-workflow.md)에 따른다.

1. 선별 결론이 링크한 실제 코드와 일치하며 원본 성능을 새 성능으로 확대하지 않았는가.
2. P4-01~11, AMR-01~06에 더 적합한 원본 구현이나 빠진 핵심 의존성·시험이 있는가.
3. F-01~06의 수정 조건이 충분한가. 관측 시각 TF, 수신 최신성, 액션 취소와 실제 정지를 혼동하지 않았는가.
4. [범위의 AC-01~07](scope.md)과 [레슨런 LL-01~10](lessons-learned.md)이 새로 구현할 기능과 연결되는가.
5. PR 2에서 결정할 디렉터리·배포·알고리즘을 PR 1에서 확정한 것처럼 쓰거나, 불필요한 기존 과제 기능을 필수로 만든 부분이 있는가.

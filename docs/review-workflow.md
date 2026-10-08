---
title: PR 병렬 작성과 최종 검토 및 머지 지침
status: working-agreement
updated: 2026-10-08
---

# PR 병렬 작성과 최종 검토 및 머지 지침

사용자의 최신 지시에 따라 **Codex는 앞 PR의 머지를 기다리지 않고 필요한 Draft PR을 계속 작성한다.** Claude는 전체 로직·계획을 검토하고, 사용자가 전달하는 피드백을 Codex가 묶어서 반영한 다음 PR별 최종 검토를 수행한다. 최종 검토를 마친 PR은 Cursor의 검증·수정·머지 담당에게 넘긴다. 작성과 검토는 병렬로 진행하고, main 반영은 의존 순서를 지킨다.

현재 대상은 ROKEY-7의 계획 PR 1~10이며, 지금 작성된 PR 1~4를 우선 검토한다. 사용자가 해당 세션에 지시문을 전달한다. 역할을 적었다는 사실은 도구 간 메시지 전송이나 검토 완료를 뜻하지 않는다.

## 사용자 위임 근거와 적용 범위 (2026-10-07 대화, 2026-10-08 기록 보강)

`[사용자 요구]` 대화 원문: “너는 일단 쭉쭉 pr작업을 계속 진행해. 그리고 나서 하나씩 pr을 너가 최종 검토하고 커서의 머지 전문가한테 넘겨서 메인에 머지할꺼야.” 이어 “병렬적으로 작업을 하자. 너는 pr을 계속 써 너가 써야하는 것들을 그리고 내가 한꺼번에 수정할 기회를 줄께”. 이 지시는 **ROKEY-7 파이널 통합 PR의 병렬 작성, Codex 최종 검토 후 Cursor main 머지**에 적용한다. 원본 ROKEY_P4의 운영 권한이나 다른 저장소의 머지 위임으로 확대하지 않는다.

`[제안·채택한 작업 방식]` 병렬 작성에 필요한 의존 diff를 구분하기 위해 stacked Draft를 사용한다. 사용자가 `stacked` 또는 `merge commit`을 직접 지정한 것은 아니다. 이 저장소에서는 [P4 AGENTS 3절](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/AGENTS.md#L38)과 [DEC-001](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/decisions.md#L52-L57)의 전 저장소 규칙을 승계하지 않고 위 최신 위임에 맞춘다. **현재 PR 1~4 스택은 merge commit 방식 하나로 고정한다.** 최종 대상은 매번 `main`이며 상위 feature 브랜치에 머지하지 않는다.

2026-10-08 조회에서 merge commit/squash/rebase 모두 허용, `main` protection API는 404(Branch not protected), rulesets는 빈 배열이었다. 이번 수정은 저장소 전역 설정을 변경하지 않는다. 아래 검사·head·base 확인을 Cursor의 머지 절차로 강제한다. 보호 규칙 설치와 다른 머지 버튼 비활성화는 저장소 관리 후속 항목이며, **CI 통과가 브랜치 보호 적용을 뜻하지 않는다.**

## 역할과 작업 순서

| 담당 / 시점 | 할 일 | 남길 결과 |
| --- | --- | --- |
| Codex 계속 작성 | 작업을 기능·문서별로 나누어 병렬 작성. 상위 PR에 의존하면 그 브랜치 위에 하위 Draft PR을 쌓음. 앞 PR의 리뷰·머지 대기로 다음 초안 작성을 멈추지 않음 | PR별 base/head, 상위 PR URL과 작성 기준 SHA, 이번 PR만의 파일·변경·검증·미확인 |
| Claude 병렬 검토 | 준비된 PR의 요구·계획·구조·알고리즘·실패 흐름을 원본 코드와 대조. 파일 수정·머지는 수행하지 않음 | 검토 base/head와 `CLA-01` 형식의 지적·근거·수정 조건을 PR 코멘트로 기록 |
| Codex 일괄 수정 | 사용자가 전달한 피드백과 PR 지적을 모아 반영. 상위 계약 변경의 영향을 의존 PR에도 적용 | 지적별 처리·반영 SHA, 바뀐 의존 PR, 필요한 재검증 결과 |
| Codex PR별 최종 검토 | 원본 판단·요구 충족·diff·검증·차단 지적을 최종 head에서 다시 확인. 의미 있는 설계 변경은 Claude 논리 검토도 갱신 | `Cursor 인계 가능` 또는 남은 차단 사유, 최종 검토 head, 실제 증거와 미실행 |
| Cursor 검증·수정·머지 | 인계된 PR을 독립 검증하고 필요한 수정 수행. 아래 조건을 충족한 PR만 의존 순서대로 main에 반영 | 최종 검증 head, 수정의 영향·재검토, `MERGED` 상태·실제 머지 커밋 |

Codex가 `Cursor 인계 가능`이라고 기록하기 전에는 Draft 작성 중인 PR을 머지하지 않는다. 이것은 사용자에게 반복 승인을 요구하는 절차가 아니라 작성·검증 완료 상태를 전달하는 기록이다. 사용자에게 피드백을 받을 시점을 기다리는 동안에도 독립적인 다음 PR 초안 작업은 계속한다.

## 2026-10-08 이후의 자리와 검토 밀도 (작업 묶음 1 ~)

계획 PR 1 ~ 10 과 팀 규칙 PR #8 이 `main` 에 들어간 뒤의 운영이다. 위 표의 Codex 작성·최종검토 흐름은 그 기간의 기록으로 남긴다.

| 자리 | 하는 일 | 신호 |
| --- | --- | --- |
| 작전·총괄(Claude) | 작업 묶음 지정(`docs/work-bundles.md`, PR #13), 공통 계약·스키마·검증 설정 소유, 구현 PR 통합 검토(`OPS-nn`) | 지적이 닫히면 `Cursor 인계 가능` 코멘트(머리말 종류 `머지 판정 요청`) |
| 로봇·지도 / 인식 / 협업·관제(Claude) | 레인 안의 구현과 작성자 검토, 영향 받는 검사 실행, 「미결정·후속」에 계약 요청 | PR 본문 + push 한 같은 턴의 코멘트 |
| 검증·수정·머지(Cursor) | head 독립 검증(`CUR-nn`), 작은 수정, merge commit(`--match-head-commit`) | 머지 완료 코멘트 |
| 딥검증(아스트라) | 계약 PR 의 두 번째 독립 검토(사용자가 부름) | 지적 코멘트 |

검토 밀도(사용자 결정 2026-10-08): **구현·문서 PR** 은 작전 통합 검토 + Cursor 검증·수정·머지의 2중이고 Codex 최종 검토는 거치지 않는다. **계약 PR**(`rescue_contracts`·프로파일 스키마·계약 문서 — 작전이 작성자)은 작전 자기 검토가 독립 검토가 아니므로 아스트라 딥검증을 거친 뒤 Cursor 에 넘긴다. **E3(ROS·실기) PR** 은 다시 2중 검토 + E3 증거 형식(`docs/work-bundles.md` 8절, PR #13)을 요구한다. 로직검토(Claude) 자리는 공석이다(그 세션이 작전으로 전환). 세션 간 메시지는 작전 ↔ 구현 세션의 지시·보고에만 쓰고, 결정의 기록은 PR 본문과 `docs/` 다.

## 상위 PR과 작성 중 변경 관리

- 기본 계획은 PR 1~10이다. [scope.md](scope.md)의 선행 조건은 기술 의존성을 뜻하며 머지 전 작성 금지가 아니다.
- 상위 변경이 필요한 하위 PR은 상위 브랜치를 base로 삼아 자체 diff만 검토하게 한다. 본문에 상위 PR, 작성 당시 상위 head, 의존 문서·계약 버전을 적는다. 완성되지 않은 계약은 가설로 표시하고 수정 영향을 추적한다.
- 같은 브랜치·파일의 수정 담당은 한 번에 한 세션으로 둔다. Claude는 읽기 전용으로 검토하고, Cursor에 인계된 PR은 Codex가 동시에 push하지 않는다. 다른 PR의 작업은 계속할 수 있다.
- 리뷰 시점 이후 head가 바뀌면 기존 리뷰는 그 SHA에 대한 기록으로 보존한다. 새로운 변경분·의미·검증 범위를 명시하며 이전 결과를 새 head의 승인으로 바꾸지 않는다.
- 상위 PR을 main에 머지한 뒤 하위 PR의 base와 diff를 다시 확인한다. **merge commit**을 사용하고 다음 PR base를 최신 `main`으로 바꾼다. 필요하면 main을 하위 브랜치에 merge하여 통합 head를 다시 검사한다. 예외적으로 squash/rebase가 발생하면 정상 절차를 멈추고 하위 고유 변경·diff·리뷰 기준을 재구성한 뒤 재검증한다. 앞 PR 내용이 하위 PR의 새 변경으로 중복 표시된 상태로 머지하지 않는다.
- 요청·계약을 바꾸는 수정은 해당 PR과 소비 PR까지 함께 검토한다. 현재 PR의 필수 문제를 단순 후속 항목으로 옮겨 통과시키지 않는다.

## Cursor에게 넘기는 조건과 머지 조건

1. 실제 PR base/head, Codex의 최종 검토 head, Claude의 논리 검토 대상과 유효 범위를 대조한다. 상위 PR이 아직 main에 없으면 해당 의존성을 먼저 반영한다. 하위 PR을 먼저 main에 넣지 않는다.
2. 이번 PR의 머지를 막는 지적이 모두 해결되어야 한다. 원본 접근 불가·조건부 통과의 남은 조건·설계 이견은 해결됐다고 처리하지 않는다. Cursor가 사실과 다른 지적을 발견하면 파일·줄과 이유를 남기고 판단을 다시 요청한다.
3. 최종 head에서 PR 범위에 필요한 검사를 실행한다. 명령·입력/환경·결과·실패·미실행을 기록한다. CI가 없거나 검사 0개이면 `없음`으로 기록하고 자체 검증을 수행한다.
4. Cursor 수정이 요구·계획·계약·알고리즘·원본 판단·동작·인수 조건을 바꾸면 Codex 최종 검토를 다시 받고, 설계 변경은 Claude 논리 검토도 갱신한다. 오탈자처럼 의미가 바뀌지 않는 수정만 있으면 Cursor가 전후 SHA·diff·의미 불변 이유와 해당 검사를 남긴다. 분류가 불명확하면 재검토 대상으로 둔다.
5. 최신 base/main과의 충돌·통합 영향, 필수 CI·리뷰·보호 규칙을 확인한다. 머지 직전 원격 head가 검증한 head와 같은지 확인하고, 지원되는 경우 기대 head를 지정해 머지한다. 관리자 우회로 필수 검사를 생략하지 않는다.
6. 조건을 충족하면 Cursor가 Draft를 해제하고 `gh pr merge <번호> --merge --match-head-commit <검증한 전체 SHA>`로 머지한다(base가 `main`인지 먼저 확인). 추가 사용자 머지 승인 질문을 반복하지 않는다. 권한·도구 제한이 있으면 원인과 남은 조작을 정확히 보고한다.
7. 머지 후 `MERGED` 상태, 실제 머지 커밋, 최신 main에의 포함을 확인한다. merge commit SHA와 검증 head SHA를 함께 기록한다. 다음 하위 PR을 검증할 수 있도록 base·diff 정리 상태도 남긴다.

PR 1의 완료 조건은 문서·정적 분석이다. ROS·모델·실기 미실행 자체는 이 문서 PR의 머지 차단 사유가 아니다. 후속 구현 PR은 필요한 실기가 없어도 Draft와 로컬 검증 작업을 계속할 수 있지만, 그 PR의 인수 조건에 필수인 미실행 시험을 성공으로 처리하거나 검증 완료로 인계하지 않는다.

## Claude가 집중해서 볼 것

- [범위](scope.md)의 RQ-01~07과 AC-01~07이 재난 수색·지속 SLAM·AMR 2대·실제 PC 4대·발견/접근/보고를 빠짐없이 담는가.
- [선별표](source-selection.md)의 수정 후 이식/원리 참고/조건부/제외/신규 설계 판단이 실제 코드와 맞는가. AMCL 제어·고정 도크 배정을 SLAM 협업 수색으로 오인하지 않았는가.
- 로봇 A의 발견 위치를 B가 해석하는 좌표 기준, 지도 보정 뒤 유효성, 대상 재확인, 활성 goal의 만료·취소·접근 불가 처리에 논리적 공백이 있는가.
- [레슨런](lessons-learned.md)이 문구로 끝나지 않고 담당 PR·검증 조건으로 연결되는가. 두 로봇/4 PC의 전체 흐름 확인을 마지막까지 미루지 않았는가.
- 계획 PR 2·3에서 정할 구조·배포·알고리즘·수치를 PR 1에서 확정했다고 쓰지 않았는가. 새 요구를 검증하는 데 필요한 후속 작업이 계획에서 빠지지 않았는가.

## Cursor가 집중해서 볼 것

- 대상 diff·변경 파일·원본 SHA·링크의 파일/줄을 실제 객체와 대조하고, 인용한 코드가 해당 주장을 뒷받침하는지 확인한다. 경로 검사의 PASS만으로 의미 검증을 대신하지 않는다.
- PR 1에서는 저장소의 `scripts/check_docs.py`와 본문 재현 명령, `git diff --check`, 문서 간 요구/ID/수치/역할 일치와 최상위 경로 집계를 검증한다. 원본 실행 파일을 무작정 실행하지 않는다.
- Claude 지적과 독립 발견을 구별해 기록한다. 수정 시 요구·코드 설명·원본 사실·인수 조건을 함께 갱신하고, 실제로 변경한 범위에 맞는 검사를 수행한다.
- 실제 실행한 시험, 정적 관찰, 과거 기록, 미실행을 구분한다. 다음 PR의 실기 결과를 현재 PR의 문서 검사로 대신하지 않는다.
- Codex 최종 검토·최종 head·남은 차단 지적·필수 체크를 확인한 뒤 의존 순서대로 머지하고 실제 머지 커밋을 반환한다.

## 원본 URL과 검토 목적

아래는 **PR 1에서 분석·차용 후보로 지정한 원본**이다. 실제 코드 이식은 아직 없다. 원본의 최신 main이 아니라 링크에 고정된 커밋을 읽는다. 필요한 테스트와 추가 후보는 [전체 선별표](source-selection.md)에 연결되어 있다.

| 영역 / 원본 링크 | 차용하려는 것 | Claude와 Cursor가 대조할 핵심 |
| --- | --- | --- |
| P4 [액션 시간 제한](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/goal_timer.py#L83-L173), [관련 시험](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/test/test_goal_timer.py#L161-L176) | 수락·결과·취소 시간 제한과 늦은 수락 처리 | 취소 응답이 실제 정지까지 입증하는가. 새 목표/로봇/좌표 계약에 맞는가 |
| P4 [AMCL FSM](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/fsm.py#L770-L815), [배정기](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/dispatcher.py#L253-L276) | 제어·배정의 일부 원리. 기존 정책은 재설계 | AMCL 초기화, DOCKED만 선정, 도크→구역 직선거리의 기존 전제를 정확히 읽었는가 |
| P4 [관측 시각 TF](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/target_locator.py#L74-L107), [수신 시각 처리](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/adapters.py#L109-L125) | 관측 시각 변환 원칙과 기하 함수 | 과거 시각 TF의 정확성, 관측 자체의 최신성, 지도 보정 뒤 유효성을 구분했는가 |
| P4 [Depth 처리](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/rokey_vision/rokey_vision/common.py#L172-L251), [동기화](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/rokey_vision/rokey_vision/depth_distance.py#L113-L132) | 디코딩·유효 거리·역투영·입력 동기화 | CameraInfo 크기/설정 일치를 실제 RGB/Depth 정렬·공간 정확도로 확대하지 않았는가 |
| P4 [tracking fallback](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/rokey_vision/rokey_vision/yolo_infer.py#L111-L115), [대상 필터](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/adapters.py#L87-L105) | 영상·추론 래퍼의 일부 | 화면에 탐지가 보이더라도 정수 track ID가 없으면 제어 입력이 버려지는 조합을 처리하는가 |
| P4 [증거 매칭](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_sysmon/rokey_sysmon/evidence.py#L24-L104), [가짜 두 로봇 시험](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_sysmon/test/test_integration_fake_robots.py#L1-L52) | 관제 기록과 관측 시각에 맞는 사진 | ROS 없는 DB·Flask 시험을 실제 2 AMR·4 PC의 성공으로 사용하지 않았는가 |
| AMR [목표 갱신·도착](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L388-L446), [대상 만료 처리](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L517-L528) | 갱신 억제 원리와 실패 경험 | 대상 자체 goal, 만료 시 반환, 기존 활성 goal의 처리·실제 정지 검증이 빠지지 않았는가 |
| AMR [웹캠 좌표 발행](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/webcam_detector.py#L91-L159), [보정 도구](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/calibrate_webcam_v2.py#L90-L165) | 고정 외부 카메라를 채택할 때만 평면 보정 도구 | 과거 좌표 재발행의 새 stamp, 평면/카메라 고정 전제, 기존 보정값의 재사용 금지 |
| AMR [모델 평가](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/yolo_report_utils.py#L136-L201), [비교 도구](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/yolo_compare.py#L68-L119) | TP/FP/FN·threshold·실험 비교 방식 | 기존 car/dummy 성능과 새 대상 성능을 분리했는가. 데이터 누출·동일 장비/데이터·전체 지연을 확인하는가 |
| P4 [프로젝트 회고](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/reference/2026-10-03-lessons-p1-p3.md#L25-L51), [지도 중복 발행 현장 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L63-L71) | 조기 통합·명령/지도 책임·검증 기록의 교훈 | 회고의 재인용과 직접 확인한 소스를 구분했는가. 교훈이 새 AC·시험으로 연결되는가 |

ROKEY_P4는 비공개다. 접근할 수 없는 원본은 경로와 이유를 기록하고 검증 불가로 남긴다. PR 본문 요약만 읽고 원본 대조를 완료했다고 쓰지 않는다. 핵심 판단에 필요한 원본을 읽지 못하면 해당 판단이 해소되기 전 머지하지 않는다.

## 검토와 수정 기록 형식

공통 종류는 `검증 요청 / 수정 요청 / 수정 완료 / 머지 판정 요청 / 기록` 다섯 가지로 통일한다. 역할명은 이 문서의 표를 사용한다. Claude의 검토는 지적이 있으면 `수정 요청`, 통과·정정·근거 보충은 `기록`으로 적는다. Codex와 Cursor의 수정 결과는 `수정 완료`, 최종 인계는 `머지 판정 요청`을 사용한다. 예시:

```text
**보냄:** 로직검토(Claude) → **받음:** 작성·최종검토(Codex), 검증·수정·머지(Cursor)
**커밋:** <검토 head 7자리> · **종류:** 기록
```

이후 base/head 전체 SHA, 읽은 원본, `CLA-01`부터의 지적과 우선순위, 대상/원본 파일·줄, 문제·영향·필요한 수정, 결론을 적는다. 결론은 **논리 검토 통과 / 수정 요청 / 검증 불가** 중 하나로 남긴다. GitHub 권한상 정식 review를 제출할 수 없다면 같은 정보를 PR 코멘트로 남긴다.

Cursor의 수정·검증 코멘트 첫 두 줄:

```text
**보냄:** 검증·수정·머지(Cursor) → **받음:** 작성·최종검토(Codex), 로직검토(Claude)
**커밋:** <최종 검증 head 7자리> · **종류:** 수정 완료
```

| 지적 ID와 코멘트 URL | 처리와 근거 | 반영 커밋 | 재검증 결과 | 상태 |
| --- | --- | --- | --- | --- |
| CLA-01 또는 CUR-01 | 수정 / 근거를 들어 비해당 / 재검토 필요 | 실제 SHA | 명령·입력·결과·증거 | 해결 / 미해결 |

해결됐다는 문구만으로 thread를 닫지 않는다. 지적의 닫힘 조건과 증거를 연결한다. 머지 완료 보고에는 PR URL, 최종 검증 head, 실제 머지 커밋, 주요 검증, 후속 계획 PR에 넘길 항목을 남긴다. 후속 항목은 현재 PR의 미해결 차단 문제와 구별한다.

재현 가능한 결함 수정은 실패하는 회귀시험을 먼저 커밋하고 수정 커밋을 뒤에 남겨 전후 실행이 가능하게 한다. 기존 PR 4의 수정 전 로그는 과거 증거로 보존하며 없는 커밋을 사후에 만들었다고 주장하지 않는다. 새 기능·상수 정리에는 인위적인 실패를 만들지 않는다.

## 계획 PR 5~10 작성 배치의 번호와 판정

계획 단계 번호와 GitHub 번호는 동일하지 않을 수 있다. 팀 규칙 PR #8이 별도 생성되어 계획 8단계는 GitHub #9, 계획 9단계는 #10으로 등록되었다. 후속 URL은 실제 생성 결과를 기준으로 본문과 인계표에 기록한다. #8을 임무 조정 PR로 오인하지 않는다.

이 배치는 앞선 merge commit 절차를 그대로 적용한다. 각 base feature branch는 diff 검토용이며 최종 merge 대상은 main이다. E2 코어와 합성 리허설의 Draft를 병렬 작성했으며, 계획의 실제 ROS/실기 완료와 구분한다. 상세 구현 기록의 미실행 관문을 남긴 채 기능 전체 완료 또는 Cursor 최종 머지 인계로 표시하지 않는다.

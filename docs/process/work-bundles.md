---
title: 작업 묶음 — BRD v0.1 반영과 세션별 첫 PR
status: draft-bundle-1
updated: 2026-10-10
---

# 작업 묶음 1 — Python 코어를 실행 환경에 연결하고 BRD v0.1 시나리오를 반영한다

작전·총괄이 관리하는 문서다. 세션별 첫 PR 의 범위·입출력·완료 조건, 먼저 확정할 공통 규격, 사용자 확인 항목, 통합 순서를 적는다. 지시는 세션 간 메시지로 전달하지만 **기록은 이 문서와 PR 본문**이다. 기준: `main` `e3849a1`(계획 PR 1 ~ 10 + 팀 규칙 PR #8 머지), 팀 [BRD v0.1](../requirements/brd-v0.1.md)(2026-10-08), 사용자 지시 2026-10-08("시나리오가 추가적으로 나와서 이 부분도 반영"). 현재 세션 구성과 검토·인계 순서는 [현재 검토 절차](review-workflow.md#current-review-workflow)가 정한다. 이 문서의 첫 묶음 기준 SHA는 당시 범위 기록이다.

2026-10-09 ~ 10 의 결정으로 아래 내용 중 다음은 지금 기준이 아니다. ① 고정 카메라는 웹캠 1대라 카메라 B·출구 카운트가 빠졌다([OPS-20 ②](https://github.com/jaebeom/ROKEY-7/pull/44#issuecomment-6078811090), 2026-10-09). 1절 UC-06·UC-07, 2절 완료 상태·출구 카운트 행, 4절의 고정 카메라 A·B·출구 카운트 기록, 6절 3·5번의 출구 카운트·카메라 B 가 해당한다. ② 지진 판정은 화면 전체 왕복 횟수로 보고 가려짐은 판단 보류(UNKNOWN)로 둔다(#33 댓글 합의 2026-10-09, 구현 #66). 1절 UC-01 의 「흔들림·가려짐」이 해당한다. ③ TB2 는 관제 웹 운영자 승인으로만 출동한다(scope Q-12, 2026-10-10). 4절 모드 행의 「관리자 승인 여부는 OI-06」이 해당한다. 5절 확인 항목의 현재 상태는 [범위 문서 미결정 표](../requirements/scope.md#미결정-사항)에 둔다.

## 1. 구현 / 미구현 요약 (BRD 사용 사례 기준)

| BRD | 기존 코어(`main`) | 상태 | 담당 레인 |
| --- | --- | --- | --- |
| UC-01 지진 징후 감지(카메라 A: 흔들림·가려짐), 재난 모드 전환, TB1 출발 지시 | 없음 | **미구현** — 고정 카메라 입력, 징후 지표(TR-06), 시스템 모드 상태, 오탐 해제(E-01) | 인식(지표·감지) + 협업·관제(모드 상태·출발 지시) + 작전(계약) |
| UC-02 TB1 정찰, 실시간 SLAM | `rescue_mapping`(session/revision·projection), `rescue_robot`(`SEARCH_AREA`) | 코어 있음, **ROS 연결 미구현**. 수색 분담(F-01)은 BRD 에서 TB1 단독 | 로봇·지도 |
| UC-03 탐지 → 좌표 DB 저장 → TB2 통지 | `rescue_perception`(Observation), `rescue_mapping`(mission frame 투영), `rescue_operations`(ReportLedger = DB), `rescue_coordination`(Allocator 인계) | 코어 있음; 저장 확인 후 통지(TR-07)는 outbox receipt 로 가능. **실제 전송·DB 호스트 미구현** | 인식 / 협업·관제 |
| UC-04 기준 지도 대비 장애물 변화 → 관리자 화면 | `record_map_update` 는 갱신 순서만 | **미구현** — 사전 기준 지도·변화 레이어·화면 | 로봇·지도(지도) + 협업·관제(화면) |
| UC-05 TB2 대기 → 출동 → 요구조자 인근 정지 | `rescue_robot`(`HOLD`·`APPROACH_TARGET`), `rescue_coordination`(`select_candidate`·`ApproachTracker`·`Reservation`→`TaskAdmission`), AC-07(접근 불가 보고) | 코어 있음. 대기 중 준비 유지(NFR-02, OI-10)는 C-06 UNDOCK 절차와 같은 문제 | 로봇·지도 / 협업·관제 |
| UC-06 출구 카운트(카메라 B), DB·화면 반영 | 없음 | **미구현** — 고정 카메라 B 탐지·추적·중복 방지(FR-19), 카운트 기록 | 인식(탐지·추적) + 협업·관제(기록·화면) |
| UC-07 TB1 재확인 → "구출 완료" 표시 | `rescue_robot`(`REOBSERVE_TARGET`), `rescue_coordination`(`ApproachTracker.observe/complete`), S-05 사건 상태 | 재관측 코어 있음. **판정 정책 미구현**: FR-22(관측 + 사람 없음 + LiDAR 장애물 없음 + 출구 카운트), FR-24(재확인 필요 유지), LiDAR 장애물 판정 입력 없음(TR-12) | 협업·관제(정책) + 로봇·지도(LiDAR 증거) |
| 관리자 화면·DB·정지 명령(FR-25 ~ 27) | `rescue_operations` 읽기 전용 CLI, ledger/outbox | 화면·정지 명령 **미구현**. 로봇 측 통신 단절 정지(TR-11)는 `Readiness` watchdog 으로 있음 | 협업·관제 / 로봇·지도 |
| UC-08 옵션(온도·히트맵) | 없음 | **보류**(OI-07, 핵심 완료 뒤) | — |

## 2. 기존 설계와 달라지는 것

| 항목 | 기존(`scope.md`·`architecture.md`) | BRD | 처리 |
| --- | --- | --- | --- |
| 두 로봇의 역할 | A·B 대칭, 역할 교환 가능, 수색 분담 | TB1 정찰·재확인 / TB2 대기·구조 지원으로 **고정** | `scope.md` 첫 통합 시나리오·RQ-08 개정. 코드는 역할을 **프로파일 입력**으로 받고 상수로 고정하지 않음(A/B 가 뒤집혀도 동작) |
| 수색 분담(F-01) | 두 로봇 구역 분할, 자동 역할 교대 | TB1 단독 수색 | F-01 을 "TB1 구역 순차 배정 + TB2 HOLD" 로 축소. frontier 어댑터는 유지 |
| 트리거 | 없음(사용자가 임무 시작) | 카메라 A 지진 징후 → 재난 모드 | 새 상태 S-00(모드)와 고정 카메라 계약 추가(작전) |
| 구호물품 | 제외(운반·전달 장치) | TB2 가 구급상자를 **싣고 이동**, 사용은 현장 인력 | "탑재·운반" 은 포함(물리 탑재 방식은 사용자 확인), "전달 장치(매니퓰레이션)·안내(OI-02 b)" 는 계속 제외 |
| 완료 상태 | 발견 / 위치 확인 / 접근 완료 / 보고 완료, "구조 완료" 표시 금지 | "구출 완료" 를 지도에 표시(FR-22 조건부) | 사건 상태에 `RESCUE_CONFIRMED` / `RECHECK_NEEDED` 추가 — 세 입력(재관측·LiDAR·출구 카운트) 모두 있어야 확정, 부재 증거만으로 금지(KPI-06) |
| 출구 카운트 | 없음 | 카메라 B | 새 입력. 사건 ID 와의 연결은 OI-03(인형 식별)에 의존 |
| 수치 | T-xx 를 시험 전에 확정 | KPI-01 ~ 06(5 s·3 s·95 %) [제안] | T-xx 후보로 등록, 시험 전 확정. 강사 양식 예시값이라는 출처를 남김 |
| 4 PC | RQ-06·Q-08 | 서버·PC 사양 미확인 | Q-08 유지 |

## 3. 세션별 첫 PR (2026-10-08 세션 간 메시지로 지시, 각 하나)

| | 로봇·지도 | 인식 | 협업·관제 |
| --- | --- | --- | --- |
| 목적 | `rescue_robot` 실행기 ↔ Nav2 명령 어댑터 | RGB/Depth/CameraInfo·검출 결과 → `rescue_perception` 입력 어댑터 | 역할 고정 배정 정책: TB1 설정 구역 `SEARCH_AREA` 순차, TB2 `HOLD`, 발견 시 TB2 `APPROACH_TARGET` 예약 |
| 파일 | `src/rescue_robot/`(경계 모듈), `tests/`, `docs/implementation/implementation-robot.md` 보충 | `src/rescue_perception/`(어댑터), `tests/`, `assets/manifest/`, `docs/implementation/implementation-perception.md` 보충 | `src/rescue_coordination/`(정책), `tests/`, `docs/implementation/implementation-coordination.md` 보충 |
| 입력 | 프로파일 두 ID·namespace·action/topic, odom, dock/driver/watchdog/TF/Nav2 준비, Nav2 콜백 | Image(RGB·Depth)·CameraInfo·검출 결과(기록 또는 합성), 정렬·보정 근거 | 설정 구역 후보(id·mission 좌표·근거 태그), 두 로봇 적격 상태, 발견 사건(사건 ID·Observation ID·projection 상태), 역할 입력 |
| 출력 | `TaskAdmission`, `('SEND'|'CANCEL', command_id)` → NavigateToPose, `Result(outcome, action_status, stop_confirmed)` | `Observation`(int64 ns stamp, payload hash), `LocalizationResult.reason` | 로봇별 `Task` 후보·`Reservation`, ACTIVE 는 `record_admission` 뒤에만 |
| 완료 조건 | Nav2 수락 ≠ allocator 예약 / 취소 요청·응답·action 종료·물리 정지 분리 / 두 로봇 콜백 분리 / deadline·lease·boot·authority 우회 없음 / E2·E3 범위 분리 | 잘못된 정렬·시각·깊이는 위치 없음 / 탐지만 된 경우 명시 / 지연·역순·중복이 수명·최신 상태를 늘리지 않음 / 전수 픽셀 검사의 실시간 검증 여부 구분 / 합성 입력 사실 명시 | 중복 배정 통제 / 예약 ≠ 실행 / 전환 시 취소·정지·새 세대 / 역할 영구 고정 아님 / 접근 중 다른 로봇 정책 명시 / 오래된 수락·결과가 배정을 되살리지 않음 |
| 선행 조건 | 없음(E2). ROS 환경·로봇 배정(Q-01)은 E3 에만 | 없음(E2). bag 은 로봇·지도가 녹화 | 구역 후보 입력 규격·프로파일 role 필드 — PR 본문의 임시 가정으로 시작, 작전이 확정 |
| 증거 | E2 + (ROS 있으면) E3 일부, 없으면 NOT_RUN | E2 | E2 |

작전·총괄 자신의 첫 PR: 이 문서 + `brd-v0.1.md` + `scope.md` 개정(이 PR), 그다음 자리 개정(A-07·`review-workflow.md`·팀 `AGENTS.md` 레인 표)과 공통 규격(아래 4).

## 4. 먼저 확정할 공통 규격

| 규격 | 지금 있는 것(`main`) | 빠진 것 → 작전이 계약 PR 로 |
| --- | --- | --- |
| 관측 | `Observation`(원 시각 `observed_at_ns`, `source_robot`/`source_boot_id`, frame, 위치·품질 `LocalizationResult`, `Detection.tracking_mode`) — C-01·C-03 | 고정 카메라 A·B 의 관측·사건 식별: `source_robot` 은 로봇 전용이고 `robot_ids` 는 정확히 2개라 카메라를 로봇으로 넣을 수 없다 → `SiteEvent(camera_id, kind=DISASTER_SIGN(재난 중립 이름, #17 에서 변경)|EXIT_PASS, observed_at_ns, evidence)` 류 계약 신설(C-01 개정) |
| 지도 | `MappingRegistry`(session/revision, `VALID`/`STALE_DERIVATION`/`NEEDS_REOBSERVATION`/`EXPIRED`, `auto_approach_eligible`) — C-04 | 사전 기준 지도(OI-01)와 변화 레이어의 표현. mission frame = 기준 지도 frame 으로 가정 |
| 임무 | `Task(command/task/robot/boot/authority/generation, kind=SEARCH_AREA|REOBSERVE_TARGET|APPROACH_TARGET|HOLD)`, `TaskAdmission`, `Result` — C-06 | 프로파일의 로봇 **역할**(SCOUT/SUPPORT) 필드와 대기 위치·구역 후보 입력(`rescue_bringup` 스키마) |
| 보고 | `RescueReport`(사건·관측·증거 ID, `approach_result`, `confirmation_result`) + receipt — C-05·C-08 | 사건 상태 `RESCUE_CONFIRMED`/`RECHECK_NEEDED`, 출구 카운트 기록, 재난 모드 이벤트 기록(S-05·C-05 개정) |
| 모드 | 없음 | S-00 시스템 모드: `NORMAL → DISASTER(트리거·TB1 출발 지시) → … → 해제(E-01)`. 트리거 입력은 `SiteEvent`. 관리자 승인 여부는 OI-06 |
| 시각·오류·ID | `rescue_contracts`(int64 ns, C-09 사전, `robot_ids`) | 카메라 ID 형식, `rescue_coordination`/`rescue_operations` 새 진단 코드 등록 |

## 5. 사용자(팀) 확인이 필요한 항목

BRD 13장 OI 와 이 저장소의 Q 를 합쳤다. 답이 오기 전에는 「제안 기본값」으로 설계하고 완료로 적지 않는다.

| ID | 확인할 것 | 제안 기본값 | 막히는 작업 |
| --- | --- | --- | --- |
| Q-09 (OI-01) | 기준 지도: 사전 구축 지도를 두 로봇이 공유하는가, TB1 이 새로 SLAM 하는가 | 사전 기준 지도 공유 + 변화 레이어 | UC-04, 좌표 규격 |
| Q-10 (OI-04) | "구출 완료" 정의 | [제안] 웹캠 소실 후보 뒤 TB1 재관측에서 카메라 ABSENT·LiDAR CLEAR·관측 범위 VALID 가 모두 맞을 때(BRD v0.2 3.5.5). 출구 카운트는 웹캠 1대 결정(2026-10-09)으로 빠짐. 소실 후보 하나만으로, 또는 대상 지점 관측 없이 완료 금지(KPI-06) | 판정 정책 |
| Q-11 (OI-05) | 지진 판정 조건(지표, 임계값, 해제) | 지표는 화면 전체 왕복 횟수로 합의(#33 댓글 2026-10-09, 구현 #66). 가림은 지진 지표가 아니고 판단 보류(UNKNOWN). 임계값은 정책 값이라 예비 시험 뒤 확정 | 고정 웹캠 |
| Q-12 (OI-06) | TB2 출동 명령 주체(자동 / 관리자 승인) | 결정: TB2 는 관제 웹 운영자 승인으로만 출동하고 구간마다 ‘계속’ 승인. TB1 은 재난 이벤트 검증 뒤 자동(scope Q-12, 2026-10-10) | 모드 상태, 화면 |
| Q-13 (OI-03) | 인형 수와 개별 식별(마커·색) | N 개, 표식으로 ID | 출구 카운트 ↔ 사건 연결 |
| Q-14 (OI-10) | TB2 대기 위치·상태(도킹 중 센서 off, 언도킹·준비 시간) | 실측해 출동 지연에 포함 | NFR-02 |
| Q-15 (OI-12, Q-01·Q-02) | 로봇 번호·모델·센서, PC 사양, ROS 배포판, 고정 웹캠 1대 하드웨어와 호스트 | 장비 확인 뒤 갱신 | E3 전체 |
| Q-16 (OI-02) | "사람과 같이 출발" 의 뜻 | (a) 구급상자를 싣고 접근 | 범위 |
| Q-17 (OI-07·08·11) | 옵션(온도) 시점·범위, 재난 유형 명칭, 발표일·평가 기준 | 핵심 뒤. 재난 유형은 지진으로 결정, 발표일 10-16(scope Q-17) | 일정 |
| Q-18 | 사람(인형) 탐지 모델: 사람 class 가중치의 출처·라이선스와 추론 런타임(cv2.dnn ONNX / ultralytics / 기타; 인식 PC 메모리 7.6 GB, GPU 없음 [설정, 2026-10-08]). 현재 있는 가중치는 car/dummy 용이라 사람 탐지에 쓰지 않음(LL-09) | 결정 전에는 인식이 고정 웹캠의 대상 소실 후보를 합성 탐지로 E2 진행. 카메라 B 출구 통과는 웹캠 1대 결정(2026-10-09)으로 빠짐 | 인식 (a) 실제 모델 추론 연결 |
| — | BRD v0.1 자체가 팀 합의본인지(팀원 초안) | 합의 전까지 `[제안]` 로 취급 | — |
| Q-19 ~ Q-24 | [BRD Part I](../requirements/brd-part1-v0.1.md)(지진, 2026-10-08) 반영 항목: 출동 우선순위, 구조 상태 4단계, 병합·도착·재확인 반경, 관제 제어 명령·복귀·배터리, 수락 수치 초안, 탐색 방식 | [범위 문서 미결정 표](../requirements/scope.md#미결정-사항)에만 둔다 | Part I 부록 A·B |

## 6. 통합 순서

1. 작전: 이 PR(BRD·작업 묶음·scope) → 자리 개정 PR → 공통 규격 계약 PR(SiteEvent·역할·사건 상태·S-00).
2. 구현 3 세션: 첫 PR 병렬(E2). 공통 규격이 늦어도 막히지 않도록 임시 가정은 PR 본문에만.
3. 통합 리허설: `offline_demo` 에 BRD 흐름(모드 전환 → TB1 정찰 → 탐지 → TB2 접근 → 출구 카운트 → 재확인 → 판정)을 검사로 추가(협업·관제 주도, 작전 취합).
4. ROS 연결 E3: 로봇 배정(Q-15) 뒤 로봇·지도가 field-runbook §2 순서로. 인식은 bag 재생.
5. 고정 카메라 A·B, 관리자 화면, 보고 전송.
6. AC 회차(E4): `acceptance-plan.md` AC 와 BRD AC-01 ~ 09 를 합친 표로(자리 개정 PR 이후).

## 7. 진행 보고와 검토

- 구현 세션 → 작전: PR 코멘트 뒤 세션 간 메시지로 "PR URL · head · 변경 · 검사 · 막힌 의존성" 다섯 줄. 작전은 보고만으로 완료 처리하지 않고 계약과 실제 diff 를 대조한다(`OPS-nn`).
- 검토·인계 흐름과 머지 조건은 [현재 검토 절차](review-workflow.md#current-review-workflow)와 [머지 조건](review-workflow.md#current-merge-conditions)을 따른다. 첫 묶음 초안의 이전 Codex 최종 검토 순서를 현재 필수 조건으로 사용하지 않는다.
- 검사 중복 줄이기: 구현 세션은 영향 받는 검사만, 전체 `run_tests.py`·`check_docs`·설치본은 작전이 머지 전 한 번.

## 8. E3 증거 형식 (제안, A-02 E3 칸의 구체화)

run ID · 전체 git SHA + dirty diff 유무 · 프로파일·로봇 파라미터 해시 · `ROS_DOMAIN_ID`·discovery 설정 · rosbag(토픽 목록, 시작/끝 stamp, 크기) · 노드 로그(기동 시 SHA·모델 해시·적용 설정 포함, 팀 `AGENTS.md` 8절) · `summary.json`(검사 이름 → PASS/FAIL/NOT_RUN) · 개인 경로는 `<REPO>`/`<TMP>`/`<USER_CACHE>` 로 정규화. 이 형식이 없는 "돌렸다" 는 E2 로 적는다.

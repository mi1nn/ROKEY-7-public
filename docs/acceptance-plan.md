---
title: 사전 시험 기준·구현 분담·통합 관문 초안
status: draft-proposal
updated: 2026-10-08
---

# 시험·작업 분담 초안

계획 PR 3의 검증 설계다. 아직 실행한 시험·정한 장비 성능을 보고하는 문서가 아니다. [범위의 AC-01~07](scope.md), [인터페이스 계약](contracts.md), [상태 전이](state-machines.md)를 시험 가능한 관문으로 연결한다. 소프트웨어는 ROS 어댑터와 분리한 순수 로직부터 검증하고, 실제 두 로봇·예정된 네 PC 배포의 증거를 별도로 확보한다. 두 AMR 활용은 사용자 확정 요구다. 고사양 PC 2대·중간 사양 PC 2대는 예정 자원이며, 네 대를 모두 쓰는 구성과 발견→안전 접근→보고는 현재 설계 기준안이다. 아래 AC는 그 기준안의 검증 제안이며 네 PC 전체 사용이 최종 인수 조건으로 확정됐다는 뜻은 아니다. 현재 CT-01~12와 AC-01~07은 모두 `NOT_RUN`이다.

## A-01. 시험 전에 고정할 수치와 조건

임계값의 상태는 `PROVISIONAL` 또는 `LOCKED_FOR_RUN`으로 기록한다. 현재 아래 항목은 모두 미확정 제안이다. 사용자가 제공하지 않은 장비 성능·도착 거리·정지 시간을 임의의 확정 숫자로 채우지 않는다. `null/TBD`를 무제한·0·기본 PASS로 해석하지 않는다.

| ID | 사전 고정할 값/단위 | 결정 근거·소유자 |
| --- | --- | --- |
| T-01 | 관측 최대 나이·SHORT_LOSS_MAX·센서 수신 단절 시한·처리 지연 예산(s) | 센서 stamp 의미, 네트워크·추론 지연 분포와 아래 원본 기록; 공백 유예는 원 관측의 남은 유효시간을 넘지 않음; perception + robot |
| T-02 | 허용 clock 오차·미래 stamp 여유·동기화 재확인 조건(s) | 네 PC와 두 로봇의 시간 측정; 아래 header-수신 차이를 clock 오차로 치환하지 않음; bringup + 계약 담당 |
| T-03 | TF/pose 최대 나이, 위치/회전 불확실성 상한, alignment 유효기간(m/rad/s) | 공통 기준점·SLAM backend 보정 감지 시험; mapping |
| T-04 | 준비·action 수락·전체 task deadline, dock_status 최대 나이, retry 횟수/간격(s/count) | 시스템 기동·준비 UNDOCK/DOCK 및 경로 실패 기준; robot + coordination |
| T-05 | lease·heartbeat·최종 속도 만료 시한, cancel·정지 확인 deadline(s) | 로컬 watchdog/명령 경로와 장애 주입 측정; 아래 H.2.5 문서값은 설치 장비에서 재확인; robot |
| T-06 | 정지 확인 속도·각속도·지속 관측 구간(m/s, rad/s, s) | 실제 odom 의미와 외부 정지 관측 대조; robot |
| T-07 | 최저/최대 접근 거리, 대상 보호 영역, 위치 오차/제동 여유(m) | 사람/인형·로봇 footprint·제동 조건; mapping + robot + 현장 담당 |
| T-08 | 속도·각속도·가속도 제한, 도착 pose 허용 오차(m/s, rad/s, m, rad) | 장비/공간·장애물 시험; 아래 특정 PC Nav2 설정을 안전 접근 기준으로 승계하지 않음; robot |
| T-09 | 재확인에 필요한 관측 수·기간·최대 나이·동일성 기준(count/s/m) | 대상/가림/시점·중복 관측 데이터; perception + coordination |
| T-10 | 목표 갱신 최소 간격·이동량, 경로 재계획 예산(s/m/count) | oscillation·차단/우회 시험; robot + mapping |
| T-11 | 통로 진입·점유·이탈/UNKNOWN 판정·재배정 조건 | 경기장·로봇 footprint·통신 실패 시나리오; coordination |
| T-12 | 보고/증거 재시도·영속 완료 시한·보류 저장 한도(s/bytes) | 운영 네트워크·DB·사진 범위; operations |
| T-13 | 수색·탐지·위치·접근·보고 성공 기준, 반복 수·분모·최대 실패 수 | 평가 환경/발표 요구; 통합 담당과 사용자 요구 검토 |
| T-14 | 4 PC 동시 부하의 지연·누락·메모리/GPU·네트워크 한계 | 실제 CPU/GPU/RAM·배치 실험, discovery 서버 CPU·participant 수·양방향 메시지 지연; 모든 담당 |

**원본 기록을 참고할 때의 조건:** 다음은 P4의 기록·설정·문서 인용이다. 새 시스템의 측정 결과나 LOCKED_FOR_RUN 값이 아니며, 수치의 자릿수와 측정해야 할 조건을 정하는 출발점으로만 쓴다.

| 관련 기준 | 원본 값과 측정/인용 조건 | 해석 제한 |
| --- | --- | --- |
| T-01·02 | 2026-10-06 10:40 도킹 점검: 카메라 **14.6 Hz**, 원문 “시계 +2864 ms(= 영상 지연)”. [practice L26](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L26) | 이 행은 30 fps 시험이 아님. 이 짧은 기록만으로 header가 PC보다 앞선 것인지, 순수 clock 오차인지, 노출 시각 기준 지연인지 확정하지 않음 |
| T-01·02 | 같은 날 20:38~20:45, robot3 언도크·정지, RGB 1280×720 **30 fps 설정**, yolo_infer만 카메라 구독: 지연 중앙값 약 **3.3 s**. 이후 15 fps로 바꾼 기록은 **82 ms**. [조건/결과](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-robot-detect.md#L12-L33) | 30 fps는 설정값이며 실제 탐지 수신은 약 19 Hz. 82 ms와 같은 회차가 아니고 모델/구독 부하 조건도 함께 기록 |
| T-01·02·14 | 설정 D(RGBD 정렬 1280×720, RGB **15 fps**)에서 10-07 13:01 RGB 압축+Depth 동시 수신: RGB **9.6 Hz**, 지연 중앙값 **1244 ms**, 합계 **3.41 MB/s**. 15 s 측정에서 처음 3 s 제외, 한 프로세스가 각 토픽 하나씩 구독. [조건/결과](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-07-depth-bandwidth.md#L9-L20) | 비교 대상 82 ms는 전날 RGB를 따로 받은 회차. 도구의 지연은 receive wall time − header stamp이며 노출 시각 의미까지 증명하지 않음. [측정 정의](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/tools/rgbd_probe.py#L1-L14) |
| T-05 | Create 3 H.2.5 속도 명령 단절 timeout **0.5 s**라는 원본 문서 인용. [DEC-011](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/decisions.md#L160) | 원본도 설치 펌웨어의 값은 미확인. 이 프로젝트의 물리 정지 시간/거리나 watchdog 검증값이 아님 |
| T-08 | 2026-10-04 해당 PC의 turtlebot4_navigation Jazzy nav2.yaml: 최대 **0.26 m/s**, 회전 **1.0 rad/s**, 도착 허용 **0.25 m·0.25 rad**. [SR-008](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/srd.md#L49-L51) | 설정 확인 기록이며 실측 정지 거리·사람 안전 접근 거리나 새 기본값이 아님 |

원본 probe 주석의 시계 일치 확인 날짜는 2026-10-04다. 이를 새 시험의 시간 동기화 증거로 쓰지 않는다. T-01의 지연 예산과 T-02의 시계 오차는 별도로 측정하고, 지연이 크다는 이유로 관측 수명을 무조건 늘리지 않는다.

시험 프로파일에는 값·단위·근거·변경자·결정 시각·version/hash를 남긴다. 새 프로파일은 시험 시작 전에 확정한다. 실패 결과를 본 뒤 기준을 낮춰 같은 회차를 PASS로 다시 표시하지 않는다. 기준 변경이 필요하면 이유를 검토하고 새 run ID로 재시험하며 이전 결과도 보존한다.

문서 초안 PR을 검토/머지하는 것과 이 수치가 실기에 승인·고정된 것은 다르다. 미확정 값을 사용하는 의존 구현·자율주행은 readiness 관문에서 막는다. 기록 입력·순수 로직 시험에서는 명시적인 시험용 수치를 쓸 수 있지만 실물 기본값으로 승계하지 않는다.

## A-02. 새 프로젝트의 검증 층위와 상태

아래 E1~E4는 **이 프로젝트에서 새로 제안한 검증 층위 코드**다. 원본 P4 문서의 증거 등급 1~4와 정의·판정이 같다는 뜻이 아니며 서로 치환하지 않는다. 원본의 숫자 등급/PASS는 원문 체계로 인용하고, 이 프로젝트는 아래 범위와 실제 수행 여부를 따로 기록한다.

| 수준 | 확인 범위 | 대신할 수 없는 것 |
| --- | --- | --- |
| E1 정적/문서 | 경로·버전·스키마·책임·상태/예외·출처·프로파일 제약 | ROS 통신, 모델 성능, 장비 동작 |
| E2 순수 로직/기록 입력 | 결정·상태 전이·중복/만료·좌표 수학·평가 계산의 재현 | 설치된 드라이버/SLAM backend의 실제 의미, 물리 정지 |
| E3 ROS 통합/모의 입력 | 메시지·action·QoS·launch/remap·실제 프로세스 연동 | 실제 두 로봇·네 PC 부하·환경 변화 대응 전체 |
| E4 실기 | 실제 두 AMR·PC 네 대의 배포·환경·명령/정지·시연 증거 | 다른 경기장·다른 사람/장비/재난 환경의 일반 성능 |

각 시험 상태는 `NOT_RUN`, `BLOCKED`, `FAILED`, `PASSED`, `NOT_APPLICABLE` 중 하나다. N/A는 해당 PR 범위 밖이라는 근거와 후속 담당을 기록할 때만 쓴다. 원본 접근 불가, 장비 미연결, 미확정 합격선, 실행 실패는 PASS가 아니다. 문서 PR에서 실기 NOT_RUN을 표시하는 것은 정상이며 이후 실기 관문을 없애지 않는다.

## A-03. 계약별 최소 시험

| 시험 ID | 입력/장애 주입 | 기대 결과·증거 | 연결 |
| --- | --- | --- | --- |
| CT-01 ID·중복 | 같은 관측/명령 ID 재전송, envelope stamp/sequence만 변경, 동일 ID의 의미 payload 변경 | 단순 재전송은 같은 결과·새 goal 없음. 의미 payload 충돌은 거절. 변하는 상태는 ID+revision으로 정상 갱신 | C-02, S-08 |
| CT-02 최신성 | 짧은 탐지/모델 공백, SHORT_LOSS_MAX·원 관측 유효시간 경계, hold/중복 재발행, 지연·역순, stamp 0/미래, 다른 clock domain, wall-time step | 모든 독립 guard가 유효한 경계 안에서는 기존 goal만 유지, 새 goal·수명 연장 없음. 만료·공백 상한 초과·시간 불확실이면 대상 의존 접근/재관측 취소→정지 관측; SEARCH_AREA는 무관한 대상 만료만으로 중지하지 않음. 로컬 watchdog 별도 동작 | C-03, S-03·07, AC-06 |
| CT-03 RGB/Depth/TF | 서로 다른 시각·해상도·encoding·invalid depth, 올바른 관측 시각 TF 없음 | 동기화/단위/정렬 실패가 명시됨. 최신 TF 대체 금지. 위치 무효 탐지는 기록되지만 접근 입력은 아님 | C-05, AC-03 |
| CT-04 좌표·revision | A/B 다른 원점·초기 방향, frame 잘못 지정, map 재시작, 보정/정렬 revision 교체 | 잘못된 frame/session 거절. 영향 goal 정지 후 재계산/재관측. 목표 수락 직후 revision 변경 경합도 차단 | C-04, AC-02·06 |
| CT-05 혼합 사건 provenance | A/B 관측을 병합한 뒤 A의 map만 보정 | 각 관측의 원 session/revision 참조 유지. 영향 관측/파생 위치를 구분해 invalidate하고 B 관측까지 근거 없이 덮어쓰지 않음 | C-04·05 |
| CT-06 task 세대 | 동일 명령 재전송, 낮은 generation, 이전 boot/authority, 다른 robot_id | 잘못된 명령 거절·현 임무 보존. 재등록 없이 새 authority 수락 금지 | C-06, AC-01·06 |
| CT-07 action 생명주기 | 수락 지연·응답 누락, timeout 후 늦은 수락, 이전 goal의 늦은 성공, cancel 응답 누락 | goal UUID별 처리. 이전 goal만 취소/억제. 새 goal을 잘못 취소하거나 성공으로 덮어쓰지 않음 | C-06, S-04 |
| CT-08 준비·통신·정지 | UNDOCK 접수 뒤 무동작/timeout, dock 상태 불명, power_saver=False로 도킹 중 센서 수신; coordinator/고사양 PC/그래프 B 중계 단절, executor/추론·odom 중단; discovery 서버 목록/고유 ID·ROS_SUPER_CLIENT·participant 수 변화, 정지 뒤 새 탐지 | 준비 접수만으로 READY/이동 허가 없음. 도킹 중 센서가 있어도 수색/접근 이동 거절. 최종 로컬 만료/정지 동작, 확인 불가면 STOP_UNCONFIRMED. 수신 시각·서버 CPU·요청/응답·복구 기록; 목록 조회만으로 통신 PASS 금지. 재연결·새 영상만으로 재출발 없음 | C-01·06·08, S-02·04, AC-01·06 |
| CT-09 안전 접근 | 대상이 obstacle cell/벽 뒤/가림, 최소 거리 위반 후보, 경로 없음·반복 실패 | 대상 자체 goal을 정답으로 쓰지 않음. 도달 가능한 후보만 선택. budget 초과는 UNREACHABLE/FAILED·사건 보존 | C-07, AC-05·07 |
| CT-10 이중 관측 | 같은 사람 A/B 관측, 다른 사람의 같은 track ID, 사람 교차/동일 클래스 최고 confidence 교체 | 근거 있는 병합/별도 사건 유지. 최고 confidence 하나를 영속 동일 대상으로 가정하지 않음 | C-05, AC-03·05 |
| CT-11 통로 점유 | holder lease 만료·위치 단절, 반대쪽 로봇 동시 진입 요청 | UNKNOWN 점유를 free로 바꾸지 않음. 충돌/교착 처리와 근거 기록 | C-07, AC-01·05·06 |
| CT-12 보고·영속 | report 중복/순서 역전, DB/사진 업로드 실패, receipt 유실, ops 재시작 | 동일 report_id 중복 저장/완료 중복 집계 없음. RECEIVED와 PERSISTED 구분·재전송 복구. 필수 증거 없으면 완료 아님 | C-05·08, AC-05·07 |

위 시험은 실제 위험을 일으켜 검증하라는 지시가 아니다. E2/E3의 결함 주입으로 상태·명령 거절을 먼저 확인하고, E4에서는 합의한 모사 공간·속도·현장 정지 조건 안에서 재현한다. 최종 접근/정지 근거는 실제 하드웨어 시험으로 확보한다.

## A-04. 단계별 통합 관문

작성은 머지 대기 없이 stacked Draft PR로 진행할 수 있다. 아래는 **실행 증거의 선행 조건**이며 초안을 병렬로 작성하지 말라는 뜻이 아니다. 근거 없는 가정을 코드로 확정하거나 미실행 관문을 통과로 표시하지 않는다.

| 계획 PR | 결과·검증 범위 | 필수 관문/아직 주장할 수 없는 것 |
| --- | --- | --- |
| 1 | 범위·원본 선별·레슨런 | 원본 SHA와 근거·현재 요구 일치. 코드/실기 미실행 |
| 2 | 구조·알고리즘 대안·배포 가설 | frame·명령·lifecycle authority와 최소 실험 정의. 설치 backend 지원 여부/성능 미확정 표시 |
| 3 | 이 문서·계약·상태·시험·소유권 | CT/AC 연결과 수치 확정 관문. 아직 실제 메시지·노드 구현/시험 통과가 아님 |
| 4 | JSON 배포 프로파일 검사·패키지 예약 구조 | 정적 값/참조/역할 충돌을 검사. 실제 `.msg/.srv/.action` 생성·ROS runtime·주행은 아직 구현 완료로 표시하지 않음 |
| 5 | 두 로봇 SLAM·Nav2·변화 대응 기반 | 두 대 동시 위치·센서·명령 분리, 선택 backend readiness, 최종 정지 경로·장애물 관측/재계획. 장비 정보/T-02~08 필요 |
| 6 | 공통 좌표·관측 전달·지도 보정 | 같은 물리점 A/B 오차, **실제 backend의 보정 감지/세션 연계**, revision 변경 영향·재관측. 일반 TF 변화/수동 revision 주입만으로 완료 금지 |
| 7 | 탐지·위치·발견 사건 | 분리된 데이터 평가, 관측 시간/거리/좌표·모델 manifest. 기존 car/dummy 점수로 새 성공 주장 금지 |
| 8 | 접근 후보·배정 수락·대상 확인·통로 점유의 E2 정책 코어 | 예약 생성만으로 이동/수락 완료 금지. 실행기 admission 기준 generation 장부, ns 만료 경계, 정확한 holder. 실제 수색 분담/역할 교대는 F-01에 남음 |
| F-01 (GitHub PR 미발행) | 수색 구역 분할·SEARCH_AREA 배정·frontier 후보 어댑터·역할 교대 | A 발견→B 접근 중 A 수색 유지/재개, 역할 교환·취소/정지 후 재배정, 실제 지도/planner와 2 AMR 시험. 현재 NOT_RUN/미구현 |
| 9 | 관제·보고·영속 기록 | 로봇 상태와 UI 일치, 사진/관측·보고 receipt·재시작 복구, 단계별 결과 구분 |
| 10 | 통합 배포·시연/인수 증거 | 실제 AMR 2대와 현재 배포 기준안의 PC 4대에서 아래 AC 회차·실패 시나리오·임계값/분모/버전 기록. 네 PC 전체 사용의 최종 인수 확정과 구분 |

PR 7의 기록 입력 개발이나 PR 9의 모의 데이터 개발은 기반 실기와 병렬로 가능하다. 해당 개발 결과가 PR 6의 공통 좌표나 PR 8의 실제 임무와 통합됐다는 뜻은 아니다. 각 Draft의 기반 브랜치·선행 PR·현재 검증 가능한 범위를 명시하고, main 반영은 의존 순서를 지킨다.

## A-05. 실제 두 로봇·예정 네 PC 배포 기준안의 검증 회차

| ID | 실제 시험 | 필수 기록/판정 |
| --- | --- | --- |
| AC-01 | 두 AMR·고사양 PC 2·중간 사양 PC 2 동시 기동(현재 배포 기준안) | 선택 그래프 A/B와 호스트별 domain·discovery 서버 목록/고유 ID·ROS_SUPER_CLIENT·프로세스/DDS participant 수·서버 CPU·TF/명령 발행자 기록. 실제 메시지 수신/양방향 요청·응답·중계 단절 시험. A 요청이 B를 움직이지 않는 증거, 준비 action·fresh dock guard·READY·정지 경로 확인 |
| AC-02 | A 관측 기준점을 B가 재관측·접근 해석 | 물리 기준점의 실측값과 A/B 추정 오차. 다른 출발 자세, map 세션 변경, backend 보정 발생 시 유효성/정지·재관측 |
| AC-03 | 합의한 사람/인형을 여러 거리·방향·가림/조명에서 발견 | 이미지/시퀀스 단위 탐지 TP/FP/FN, 같은 사람 중복 사건 수, 위치 오차. 사람과 인형 결과를 혼합하지 않음 |
| AC-04 | 이동 중 장애물 추가 및 이후 제거 | scan 관측 시각, local/global costmap 변화, 경로 변경·대기/접근 불가 시각, SLAM 지도 갱신을 각각 기록. costmap clearing만으로 SLAM 지도 보정 성공을 주장하지 않음 |
| AC-05 | A 발견→B 수락·접근·재확인·정지·보고, A의 별도 수색 | 원 관측부터 receipt까지 task/target/observation ID로 연결. B의 fresh 재관측 전 A 지원/정보 만료 정책 확인. A/B 역할 교환 회차 포함 |
| AC-06 | 대상/TF/센서/명령·보고 경로의 중단·지연·복구 | 어느 관문이 언제 발동했는지, 실제 정지/확인 불가, 재개 권한·새 generation, 원 관측 시각 보존 |
| AC-07 | 우회로가 없는 대상 | 접근 실패와 발견/보고 성공을 구분. 제한된 재시도·접근 불가 이유·필수 현장 정보 저장 |

각 회차의 횟수·분모·허용 오차/시간·속도·거리와 장소 배치는 T-13의 사전 프로파일에 둔다. 한 로봇·한 PC의 단독 결과는 E2~E4 부분 증거로 기록할 수 있지만 사용자 확정 요구인 두 AMR 활용을 대체하지 않는다. 네 PC 모두의 시험은 현재 배포 기준안을 채택할 때 필요한 검증이며 최종 인수 범위는 별도로 확정한다. 배포안을 바꾸면 프로파일·AC·관련 설계를 함께 수정하고 이전 결과로 새 구성을 통과 처리하지 않는다.

원본은 robot3의 fast-discovery-server CPU **92%**와 master에서 Create 3 토픽 미수신을 기록했고, 연결 프로세스가 많다는 원인 해석은 **추정**이었다([원본 L34·L68](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L24-L68)). 이를 네 PC 부하가 재현됐다는 증거로 쓰지 않는다. AC-01에서 참여자 수·서버 부하와 메시지 지연/누락의 관계를 새로 측정한다.

지도 변화의 평가는 센서→costmap→경로→SLAM 지도 각 계층을 분리한다. 선택한 backend가 동적 장애물 제거를 지도에 어떤 방식으로 반영하는지 확인하고 그 한계를 표시한다. “SLAM 프로세스 실행 중”만으로 변화 대응 성공을 판정하지 않는다.

## A-06. 측정 정의·증거 manifest

- 발견 성공률: 사전에 정의한 대상/시퀀스별 성공 수 ÷ 전체 시험 수. 탐지 precision/recall, 중복 사건, 잘못된 대상 연계는 별도로 기록한다. 라벨 없는 대리 지표를 recall/mAP로 표기하지 않는다.
- 위치 오차: 공통 물리 기준점 대비 관측/추정 위치 거리의 분포·최대값·샘플 수. A/B 좌표가 서로 비슷한 것만으로 절대 오차가 작다고 판정하지 않는다.
- 접근 성공: 도달 가능·fresh 재관측·동일성·이격·정지 관문을 모두 통과한 회차 ÷ 사전 정의한 접근 시험 분모. UNREACHABLE 회차는 실패 처리 평가로 별도 집계한다.
- 변화 대응 시간: 장애물이 센서에 유효하게 처음 관측된 시점→정지/대기/새 경로 적용까지. 사람 손으로 장애물을 놓은 시각과 센서 관측 시각을 혼동하지 않는다.
- 보고 시간: 원 발견/확인 관측→필수 내용과 증거의 PERSISTED receipt까지. 이미지 추론 FPS는 이 지연이 아니다.
- 중단/정지 시간: 중단 원인 발생/탐지/stop 요청/명령 억제/정지 관측을 나누어 기록한다. 시계 오차보다 정밀한 네트워크 지연을 측정한 것처럼 보고하지 않는다.

각 run은 최소 `run_id`, 시험 코드/배포 SHA, stacked 기반/의존 commit, dirty 여부·필요한 diff 참조, 로봇/PC/OS/패키지 버전, profile hash·사전 기준, 모델/데이터/보정 해시, map/transform 세션·revision, clock 상태, 시험 ID/입력·seed, 실행 명령, 원문 결과/로그·필수 영상/지도/사진 참조, 실패/미실행 사유를 남긴다. 큰 bag/영상·모델은 저장 위치와 해시를 기록하고 무조건 Git에 복사하지 않는다.

수정 후 기존 결과를 사용할 때는 실제로 시험한 SHA를 바꾸지 않는다. 새 head와의 차이가 결과에 영향 없는 문서/메타데이터 수정인지 근거를 기록하고, 영향받은 시험만 다시 실행한다. source/runtime/profile/model 변화가 있으면 해당 계층과 소비자·통합 시나리오를 재검토한다.

## A-07. 구현 세션과 파일 소유권

세 구현 세션과 한 통합 담당을 제안한다. 별도 branch/worktree를 사용하며 다른 담당 경로를 직접 바꾸기 전에 계약 변경으로 요청한다. 정확한 하위 경로는 PR 2의 구조를 따른다.

| 담당 | 소유 범위 제안 | 검증 책임 |
| --- | --- | --- |
| 통합/계약 담당(Codex) | `rescue_interfaces`, 공통 schema/profile 계약·루트 문서/검증 설정 | 계약 호환, 임계값 상태, ID/시간/frame/권한 불변식, 세션 간 통합·최종 검토 |
| 기반 세션 | `rescue_robot`, `rescue_mapping`, `rescue_bringup`의 로봇별 실행/설정 | SLAM/Nav2·TF·native sensor·goal/cancel·최종 watchdog·좌표 revision/실측 |
| 인식 세션 | `rescue_perception`, 해당 모델/데이터 평가 도구·manifest | 동기화/거리/관측·다중 대상·시간/위치 품질·독립 평가·부하 |
| 협업·관제 세션 | `rescue_coordination`, `rescue_operations`, API·사건/보고 저장 | task/target 상태·통로·인계·재시작·명령 경합·UI/DB 일치. UI 범위는 최소 관제/보고로 제한 |

공통 메시지/시간/frame/상태 의미를 한 세션이 임의로 수정하지 않는다. 변경 제안에는 생산자·소비자·시험·마이그레이션 영향과 필요한 재검토를 적고 계약 담당이 통합한다. 각 세션은 소유한 순수 로직 시험과 ROS 어댑터 시험을 책임지며, 실제 두 로봇 통합 판정은 전체 증거를 보는 통합 담당이 수행한다. 장비 조작·물리 기준점 측정·촬영은 현장 접근 가능한 담당이 실행하고 원문 결과를 제공한다.

검증 책임 분담은 도구의 우열이나 자동 승인 권한을 뜻하지 않는다. 합의된 인터페이스로 모의 입력 개발을 병렬 수행해도 공유 실기 자원을 동시에 제어하지 않는다.

## A-08. Draft 작성·피드백·머지 운영

최신 사용자 요청에 따라 **초안 작성은 병렬/연속 진행하고 머지를 기다리지 않는다.** 각 PR은 자신의 base/head·부모 PR·추가 diff·미완료 관문을 기록한다. 피드백은 한 번에 모아 수정할 수 있으며, 수정 내용이 공통 계약에 영향을 주면 관련 Draft에도 반영한다.

Claude는 전체 논리·계획을 검토하고, Codex는 반영 후 최종 검토·일관성 확인을 수행하며, Cursor는 검증·필요한 수정·의존 순서의 main 머지를 담당한다. 의미를 바꾸는 Cursor 수정은 계약/계획 영향에 맞춰 Claude·Codex의 관련 재검토를 거친다. 오탈자·링크 수정도 실제 변경 범위의 검사를 다시 수행한다. 세부 운영은 [리뷰 작업 흐름](review-workflow.md)을 따른다.

하위 PR이 부모 Draft를 base로 했으면 부모 변경/머지 뒤 base와 diff를 갱신하고 변경된 검토 기준을 표시한다. 스택 아래의 새 커밋이 들어왔는데 이전 head의 승인/검증을 그대로 최종 결과로 쓰지 않는다. main 반영은 선행 의존 PR의 실제 MERGED 상태·반영 commit을 확인한 순서로 진행한다. 현재 PR 1~4는 리뷰 작업 흐름에 따라 **merge commit 방식으로 main에만** 머지하고 다음 PR base를 최신 main으로 갱신한다. 예외적으로 squash/rebase가 발생하면 진행을 멈추고 하위 고유 diff·기반·검토 head를 재구성하여 재검증한다. 검증 head와 실제 merge commit을 함께 기록한다.

모든 차단 지적과 해당 PR의 필수 검증이 닫힌 최종 head를 대상으로 머지한다. required checks가 없으면 ‘없음’을 기록하고 합의한 자체 검증을 수행한다. 필수 실기·원본 접근이 미확인인 경우 그 부분을 승인/실행 PASS로 바꾸지 않는다. 디자인 초안과 후속 실기의 미확정 가정은 명시적으로 남긴다.

## 원본 근거

시험 항목은 [AMR의 freshness 재발행](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/webcam_detector.py#L145-L159), [평면 보정 도구가 재투영·leave-one-out 오차를 계산하는 코드](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/calibrate_webcam_v2.py#L113-L165), [AMR의 추론 속도 집계 범위](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/yolo_report_utils.py#L191-L201), [P4 늦은 action 수락 시험](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/test/test_goal_timer.py#L161-L176) 등 선별 근거와 새 요구를 연결한 것이다. 기존 시험의 존재·결과를 이 프로젝트의 실행 결과로 승계하지 않는다.

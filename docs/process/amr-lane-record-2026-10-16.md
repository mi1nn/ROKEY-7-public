# AMR 레인 작업 기록과 책임 주장 검증 (2026-10-16)

2026-10-15 오후 「AMR 이 안 되는 것은 네비게이션·오퍼레이션을 맡은 임재범 책임이고, 미리 써 둔 계약과 `src` 코드 때문에 AMR 실습 테스트를 못 했다」는 주장이 구두로 나왔다. 이 문서는 그 주장을 저장소 기록(커밋, PR, 이슈, 회차 기록)과 대조한 결과다. 결론은 **주장이 기록과 맞지 않는다**는 것이다. AMR 코드·설정을 바꾼 머지 PR 146개 중 138개(94.5%)를 임재범이 썼고, 전제환은 2개(1.4%)다. 전제환의 마지막 커밋은 10-13 14:39 이고, 본인 PR #132 본문에 「계약 위에서 로봇이 실제 이동·도착했다」고 적혀 있다. 기준: `main` `2c625d9` [측정 2026-10-16].

> [!WARNING]
> 원문 미확인 (전달: 임재범, 2026-10-16): 2026-10-15 16시경 전제환이 구두로 「AMR 이 안 되는 것은 네비게이션·오퍼레이션 담당인 임재범 책임이며, 미리 작성된 계약과 src 코드 때문에 AMR 실습 테스트를 못 했다」고 말했다. 글로 남은 원문은 없다(6절).

## 1. AMR 코드·설정 PR 비율

AMR 코드·설정 경로를 바꾼 머지 PR 146개 중 임재범 138개, 전제환 2개다. 줄 수로도 임재범 16,664줄 추가, 전제환 3,226줄 추가다 [측정 2026-10-16, main `2c625d9`].

| 사람 | AMR 코드·설정 머지 PR (개) | 비율 | 추가 줄 | 삭제 줄 |
| --- | --- | --- | --- | --- |
| 임재범 (임재범이 띄운 Claude·Codex·Cursor 세션 포함) | 138 | 94.5% | 16,664 | 1,570 |
| 정해윤 | 4 | 2.7% | 125 | 13 |
| 전제환 | 2 | 1.4% | 3,226 | 664 |
| 박주은 | 1 | 0.7% | 130 | 0 |
| 이태규 | 1 | 0.7% | 187 | 2 |
| 합계 | 146 | 100% | | |

- 집계 기준: `main` 의 머지 커밋 433개 중 `src/rescue_robot/`, `src/rescue_bringup/`, `src/rescue_mapping/`, `config/` 파일을 하나라도 바꾼 PR. 시험 파일(`tests/`)과 스크립트(`scripts/field/`, `scripts/sim/`)만 바꾼 PR 은 세지 않았다. 작성자는 그 브랜치 커밋(머지 커밋 제외)의 다수 작성자다. #132 는 뒤에 임재범 커밋 2개가 얹혔지만 전제환이 연 PR 이라 전제환으로 셌다.
- 재현: 머지 커밋 제목에서 PR 번호를 읽고(`git log --first-parent --merges main`), `git diff --name-only <merge>^1 <merge>` 로 경로를, `--numstat` 으로 줄 수를, `git log --no-merges <merge>^1..<merge>` 로 작성자를 센다.
- 지금 코드 기준(`git blame`): `src/rescue_robot` 6,729줄 중 전제환 2,408줄(35.8%), 임재범 4,321줄. `src/rescue_bringup`(launch·프로파일) 3,574줄 중 전제환 0줄.
- 열린 PR 6개(#466 #467 #469 #471 #472 #473)는 모두 임재범이 열었고, 그중 AMR 관련은 4개(#466 #471 #472 #473)다. 위 표에는 넣지 않았다.
- 전제환의 TB2(TB-5) 담당 항목인 출동 구현·실기·안전 정지([일정표 CSV](schedule-timeline.csv) 32 ~ 35행)에 해당하는 코드(`start_align.py`, `frozen_localization.launch.py`, `frozen_map_relay.py`, `map_freeze.py`, 자동 접근)는 #213 #217 #219 #231 #242 #255 #291 #317 #370 #387 #435 #452 #458 #470 으로 전부 임재범이 썼다.

시점별로 나누면 임재범의 AMR PR 은 10-12 까지 12개, 전제환의 마지막 커밋이 있던 10-13 하루에 51개다. 10-15 16:00 KST 이전 기준으로도 AMR 코드·설정 머지 PR 87개 중 임재범 79개(90.8%), 전제환 2개다.

| 날짜 (KST) | 임재범 AMR 코드·설정 머지 PR (개) | 전제환 |
| --- | --- | --- |
| 10-08 | 9 | 0 |
| 10-11 | 1 | 1 (#100) |
| 10-12 | 2 | 0 |
| 10-13 | 51 | 1 (#132, 02:01 머지) |
| 10-14 | 8 | 0 |
| 10-15 16:00 전 | 8 | 0 |
| 10-15 16:00 후 ~ 10-16 | 59 | 0 |

## 2. 전제환의 작업 기록

전제환의 저장소 활동은 10-11 부터 10-13 14:39 까지 사흘이고, 그 뒤 커밋·PR·이슈·코멘트가 하나도 없다. 팀 일정에서 TB2 실기(10-13 ~ 10-15)와 안전 정지 시험(10-14)은 전제환 칸이다([일정과 담당](schedule.md) 2절).

| 날짜 (KST) | 무엇 | 결과 |
| --- | --- | --- |
| 10-11 | [#94](https://github.com/jaebeom/ROKEY-7/issues/94) watchdog 현장 파라미터 결정 요청 | 열림. 이후 본인 후속 없음 |
| 10-11 | [#100](https://github.com/jaebeom/ROKEY-7/pull/100) 임무 입력 노드·lease·수색·재관측 연결 | 당일 머지 |
| 10-11 | [#102](https://github.com/jaebeom/ROKEY-7/pull/102) CI, [#104](https://github.com/jaebeom/ROKEY-7/pull/104) 필드 축소 | 본인이 닫음([#105](https://github.com/jaebeom/ROKEY-7/issues/105)) |
| 10-11 | [#110](https://github.com/jaebeom/ROKEY-7/pull/110) SEARCH_AREA Nav2 연결 (Draft) | 검토 OPS-77: 진입점이 SEARCH_AREA 만 받게 되고 기존 런타임과 약 210줄 중복. 「main 에서 실행 옵션 하나로 바로 테스트된다」고 안내. 10-12 본인이 닫음 |
| 10-12 | [#132](https://github.com/jaebeom/ROKEY-7/pull/132) 파일 없는 단일 로봇 SEARCH_AREA 현장 연동 | 10-13 머지. 본문 「현장 단일 로봇 이동: 사용자 보고(2026-10-12) 실제 이동 및 설정 목적지 도착 완료」 |
| 10-13 14:39 | [#190](https://github.com/jaebeom/ROKEY-7/pull/190) #132 설명 문서 | 마지막 커밋 `a8416f6` |
| 10-13 오후 ~ 10-16 | 없음 | 커밋 0, PR 0, 이슈 0, PR 코멘트 0, 팀 채널 발신 메시지 0 |

- 같은 기간 임재범의 AMR 코드·설정 머지 PR 은 10-13 에 51개, 10-14 에 8개, 10-15 에 19개, 10-16 에 48개다.
- [`rescue_robot` README 「현재 상태」 표](../../src/rescue_robot/README.md)의 구현 항목 20여 개 중 전제환 커밋이 근거인 것은 「임무 입력 LIVE_ROS SEARCH_AREA」 한 줄이다. TB-5 관련 항목(자동 대상 이동, 고정 지도 재정합, 출발 정렬, 자동 재준비, Nav2 준비 판정)은 모두 10-13 이후 임재범 PR 이다.
- 10-12 「실제 이동·도착」의 원문 로그는 `docs/evidence/` 에 없다. 그날 파일은 preflight·단위 시험 로그뿐이다. 회차 기록([인수 계획](acceptance-plan.md) A-02 의 E3 형식)으로 남은 TB2 실기는 0건이다.

## 3. 주장별 검증

네 가지 주장 모두 저장소 기록과 어긋난다. 가장 강한 반증은 전제환 본인이 쓴 #132 본문과 `START_HERE.txt` 다.

| # | 주장 | 사실 | 근거 |
| --- | --- | --- | --- |
| A | 네비게이션·오퍼레이션은 임재범 책임이다 | 임재범의 역할은 오케스트레이터 + 웹이다. TB1 주행은 이태규, TB2 주행·출동·도착 보고·안전 정지는 전제환이다. 「오케스트레이터는 임무만 보내고 Nav2 목표·취소는 `rescue_robot` 이 낸다」고 명시돼 있다 | [일정과 담당](schedule.md) 2절 표와 마지막 줄, [일정표 CSV](schedule-timeline.csv) 32 ~ 35행 |
| B | 미리 써 둔 계약과 src 때문에 AMR 실습 테스트를 못 했다 | 10-12 에 전제환이 그 계약(`ExecuteTask`·`CommandLease`·C-03/04/06)과 src 위에서 TB-6 을 움직여 목적지에 도착시켰다고 본인이 기록했다. 계약이 막았다는 이슈·PR 코멘트·메시지도 없다 | [#132 본문 「증거와 검증」 표](https://github.com/jaebeom/ROKEY-7/pull/132), [`START_HERE.txt`](../../scripts/field/search-area/START_HERE.txt) 「Observed field result: user reported actual movement and selected destination arrival」 |
| C | 코드가 웹 기준으로 쓰여 있어 AMR 만 따로 테스트할 수 없었다 | AMR 코드는 웹에 의존하지 않는다. 판단 로직(`executor.py`, `nav2_adapter.py`, `task_site_runtime.py`)은 ROS 없이 import 되고 로봇 시험 45개 파일이 `scripts/run_tests.py` 한 줄로 돈다. 웹 없이 돌리는 절차도 따로 있다(4절) | `tests/test_executor.py`, `tests/test_robot_*.py`, [README 6절 「로컬 검사(로봇 없이)」](../../README.md), [AGENTS 8절](../../AGENTS.md) |
| D | AMR 이 안 되는 것은 임재범 탓이다 | 10-13 오후부터 AMR 코드는 전부 임재범이 썼다(1절). 10-15 실기에서 TB-5 를 못 쓴 원인은 Pi 와 Create3 연결 끊김과 무선망(ping 손실 97 ~ 100%)이고, TB-6 이 안 움직인 원인은 Nav2 가 반만 켜진 상태에서 첫 목표가 거절된 것이다. 둘 다 같은 날 저녁 임재범이 고쳤다(#338 #340 #342 #343). TB2 담당자의 실기 회차 기록은 없다 | [10-15 실습 기록](../evidence/field-2026-10-15-run1015.md), [10-15 회고](field-retro-2026-10-15.md) |

- #110 을 닫게 한 검토(OPS-77)는 테스트를 막은 것이 아니다. 본문에 「오늘 SEARCH 동작 하나만 빠르게 테스트해보려는 목적이라면 이 PR 을 머지하지 않고도 main 에서 실행 옵션(`task.runtime_factory=…`)만 주면 바로 테스트할 수 있다」고 적혀 있고, 다음 날 #132 가 그 길로 돌았다.
- 「AMR 작업이 하나도 없다」는 말은 정확하지 않다. #100 과 #132 가 있다. 정확한 문장은 「10-13 오후 이후 AMR 작업이 없고, TB2 담당 항목(출동·실기·안전 정지)은 전제환 커밋이 0」이다.

## 4. 웹·계약 없이 AMR 을 시험할 수 있었던 경로

10-11 부터 네 가지 경로가 `main` 에 있었고, 어느 것도 관제 웹이나 운영 PC 를 요구하지 않는다.

| 경로 | 언제부터 | 무엇이 필요한가 | 근거 |
| --- | --- | --- | --- |
| 수동 실습 launch (`mapping_navigation.launch.py`, `localization_navigation.launch.py`) | 10-11 (#92, 이태규) | 로봇 1대 + PC 1대. 계약·조정기·웹 없음. SLAM·Nav2·RViz 만 | [#92](https://github.com/jaebeom/ROKEY-7/pull/92) |
| 로봇 없는 단위 시험 `python3 -I scripts/run_tests.py` | 10-08 | Python 만. ROS 도 필요 없음 | [README 6절](../../README.md), [AGENTS 8절](../../AGENTS.md) |
| 임무 입력만 따로 띄우기 (`task_input`, `task.runtime_factory` 옵션) | 10-11 (#100, 전제환 본인) | 조정기 없이 파일 입력(FILES)으로 임무를 넣을 수 있다 | [#110 OPS-77 코멘트](https://github.com/jaebeom/ROKEY-7/pull/110), [`rescue_robot` README](../../src/rescue_robot/README.md) 「설치 가능한 임무 입력 진입점」 |
| 가제보 시뮬레이션 `scripts/sim/run_sim.sh`, `run_full.sh` | 10-15 | PC 1대. 로봇·망·웹 없음 | [sim-to-field](sim-to-field.md) |

- 「핵심 로직만 쓰는 py 파일을 찾아 테스팅 스크립트를 짜면 된다」는 이 저장소의 설계 그대로다. `executor.py` 는 Nav2 클라이언트·시각·정지 관측을 주입받는 순수 클래스이고 `tests/test_executor.py` 가 그렇게 쓴다. 전제환의 #100 에도 가짜 서버 탐침(`task_runtime_demo.py`, `task_server_probe.py`)이 있었다(#132 에서 본인이 지움).
- 실기용 절차도 웹 없이 돌아간다. 10-15 저녁에 「웹 없이 하기」 트랙(#344 명령 도구, #346 RViz, #351 마커)을 더했고 [현장 실행 안내](field-runbook.md)에 있다.

## 5. 10-15 실기에서 TB-5 가 안 된 실제 원인과 10-16 수정

10-15 오후 실습에서 막힌 곳은 열한 군데였고, 계약이나 웹 구조가 원인인 항목은 없다. 전부 망·하드웨어·띄우는 순서·준비 판정 문제였다.

| 시각 (10-15) | 무슨 일 | 원인 | 고친 PR |
| --- | --- | --- | --- |
| 15:16 ~ 15:44 | TB-6 언도킹 90초 무응답 | Pi turtlebot4 서비스 재시작 뒤 Create3 연결이 안 돌아옴 (하드웨어) | 실습 안내 복구 순서 #339 #345 |
| 15:45 | TB-5 Pi ping 손실 97 ~ 100% | 무선망, Pi 무선 절전 | #337 #339 |
| 15:58 ~ 16:27 | Discovery 서버 30분 꺼짐 | 터미널에서 띄운 프로세스가 재부팅에 사라짐 | 서비스 등록은 승인 대기 |
| 16:33 ~ 16:47 | 다른 PC 의 map_server·AMCL 이 `/robot6/map` 에 발행 | 같은 도메인의 다른 노드 (LL-17) | 배선 점검 |
| 16:54 ~ 16:59 | 서 있는 TB-6 위치가 안 나옴, slam_toolbox inactive | slam_toolbox 기본값 (LL-14) | #341 |
| 17시대 | TB-6 첫 목표 거절 뒤 2초 만에 `NO_SURVIVOR_FOUND` | Nav2 bt_navigator inactive 인데 READY 로 판정 (LL-12), 거절된 목표를 「가 봤음」으로 침 (LL-13) | #338 #340 #342 #343 |
| 종일 | TB-5 사용 불가 | Pi 와 Create3 사이 연결 끊김 | 하드웨어. 원인 [미확인] |

- 10-16 가제보 실습 18회차까지 돌리며 고친 것이 열린 PR 로 있다: [#466](https://github.com/jaebeom/ROKEY-7/pull/466) TB-5 접근 지점(C23), [#471](https://github.com/jaebeom/ROKEY-7/pull/471) 터널 높이 판정(C27), [#472](https://github.com/jaebeom/ROKEY-7/pull/472) 2차 고정 때 TB-5 실행기 중단(C26), [#473](https://github.com/jaebeom/ROKEY-7/pull/473) 18회차 증거. 같은 날 머지된 AMR 코드·설정 PR 은 48개다.
- 이 기간의 실기·시뮬레이션 회차 기록(`docs/evidence/`)에 TB2 담당자가 돌린 회차는 없다. 10-13 의 TB-5 담당 PC 측정([상황 로그](../evidence/field-2026-10-13-situation.log), [OAK-D 측정](../evidence/field-2026-10-13-oakd-robot6.md), [Pi 설정](../evidence/field-2026-10-13-pi-settings.md))도 임재범 세션이 했다.

## 6. 검색 범위와 미확인

「없다」고 적은 것은 아래 범위에서 찾지 못한 것이다. 구두 발언은 확인할 수 없다.

| 항목 | 어디서 어떻게 | 결과 |
| --- | --- | --- |
| 전제환의 커밋 | `git log --author=JeonJehwan`, main 전체 이력 (2026-10-16) | 6개, 마지막 10-13 14:39 KST |
| 전제환의 PR·이슈 | GitHub 검색 `author:JeonJehwan` (ROKEY-7) | PR 6개(#100 #102 #104 #110 #132 #190), 이슈 2개(#94 #105). 10-13 이후 0 |
| 「계약·src 때문에 못 했다」는 글 | ROKEY-7 열린 이슈 전부, #85 #110 #132 코멘트, 팀 채널 10-12 이후 전제환 발신 메시지, 키워드 「계약」「테스트」 | 없음. 10-15 16시경 발언은 글로 남은 것이 없다 |
| 담당 선언 | [일정과 담당](schedule.md), [일정표 CSV](schedule-timeline.csv), [인수 계획](acceptance-plan.md) A-07 | 사람 이름 배정은 schedule.md 뿐이고 「임시안 [제안]」 표시. PM 이 확정한 기록은 없다 |

- 전제환이 저장소 밖(개인 PC, 구두)에서 한 작업은 이 문서가 보지 못한다. 있다면 커밋·PR·회차 기록으로 보여 주면 이 표를 고친다.
- [일정과 담당](schedule.md)의 배정은 10-08 임시안이다. 다만 「백승민은 전제환 AMR 백업」이라는 팀 발언이 같은 표에 인용돼 있어, TB2 가 전제환 몫이었다는 것은 팀이 알고 있던 일이다.

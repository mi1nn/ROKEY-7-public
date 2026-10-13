---
title: 관제 웹 코드 리뷰 준비 — 설계·코드 주석·시험 증거
status: draft
updated: 2026-10-13
---

# 관제 웹 코드 리뷰 준비

## 0. 리뷰에서 하는 일과 이 문서의 범위

사용자 설명(2026-10-13, 이 PR 의 작성 세션):

- "튜터가 차트 플로우? 같은거를 보고 코드가 어떤 코드가 있는지 확인을 하면서 그 코드가 왜 그런 의도로 작성이 되었는지 설명하는 과정임."
- "설계만 보고 다른 사람도 코드를 짤 수 있을 정도로 설계를 촘촘하게 짜야함. 하지만 실제 코드를 짜는 것은 아님."
- "내가 맡은 섹션은 사실 웹만이야" — 그래서 이 문서의 범위는 **관제 웹(`src/rescue_console/`)** 이다. operations API·현장 상태 DB 는 웹이 기대는 인터페이스로만 다룬다.

튜터 슬라이드 「What to do in code review」(사용자 화면 캡처, 2026-10-13) [강의]:

| 튜터가 보는 것 | 슬라이드의 요구 | 이 저장소에서 보여 줄 것 |
| --- | --- | --- |
| Explain · Design | Process Flow Diagram(Functions·Interfaces·Dataflow). 프로그램(PGM.py)마다 Node/Class·변수, 입력 Topic(Type·필드), 함수 순서도, Main(), 출력 Topic | 1절(설계도 속 관제 웹) + [관제 웹 설계 명세](../design/console-spec.md) |
| Explain · Code | Program·Node·Function·줄 단위 `#` 주석, README.md | 3절 |
| Test/Validation | 시험 코드, 로그 출력, 스크린샷·영상, 사진·영상 | 4절 |

평가표에서 관제 웹이 걸린 항목 [강의: [원본 평가표](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/reference/2026-10-01-course-schedule-evaluation.md#L75-L91)]: 「System Monitor 모듈 코딩/테스트」 10점, 「System Monitor 완성 품질」 10점.

기준 코드: main `bd4bfa5`(2026-10-13). 그 뒤 main 이 바뀌면 줄 번호가 밀린다. 리뷰 화면에서는 함수 이름으로 찾는다.

### 리뷰 때 열 파일 (확정)

관제 웹 범위는 아래 파일이 전부다. 위에서부터 이 순서로 연다. 경로는 저장소 루트 기준 상대경로다.

| 순서 | 파일 | 슬라이드의 말로 | 이 파일에서 보여 줄 것 |
| --- | --- | --- | --- |
| 1 | `src/rescue_console/README.md` | README.md | 로봇 없이 띄우는 명령(「실행 (로봇 없이, E2)」), 화면 칸 설명 |
| 2 | `src/rescue_console/rescue_console/__main__.py` | Main() | 실행 인자 검사 → `create_app` → 서버 기동. 실패하면 서버를 띄우지 않고 종료코드 2 |
| 3 | `src/rescue_console/rescue_console/app.py` | Node/Class | 라우트 전부(13), 로그인·세션 무효화·CSRF, `POST /commands` 가 명령을 넘기는 곳, `POST /webcam`·`POST /robot-camera` 카메라 고르기 |
| 4 | `src/rescue_console/rescue_console/sources.py` | 입력 Topic | 현장 상태 DB 를 읽기 전용으로 여는 곳(웹은 DB 에 쓰지 않음) |
| 5 | `src/rescue_console/rescue_console/view.py` | Function(핵심) | 입력 → 화면 모델: 대상 4단계 규칙, 낡음 판정, 버튼 관문 거울 `_gate`·`command_gates` |
| 6 | `src/rescue_console/rescue_console/commands.py` | 출력 Topic | operations·로봇 카메라 중계로 한 번 보내기, 재시도 없음, 답 검사 |
| 7 | `src/rescue_console/rescue_console/render.py` | Function(화면) | HTML 조각, 1초 폴링·명령 확인 칸·영상 끊김 감지 JS |
| 8 | `src/rescue_console/rescue_console/mapimage.py` | Function | 지도 PGM → PNG, 로봇·대상 위치 표시, 실시간 지도 낡음 |
| 9 | `src/rescue_console/rescue_console/accounts.py`, `src/rescue_console/rescue_console/throttle.py` | Function | 계정·비밀번호 해시·역할, 로그인 연속 실패 잠금 |
| 10 | `src/rescue_console/rescue_console/labels.py` | 변수 | 코드 → 한국어 낱말 표 |

- **시험(Test Code):** `tests/test_console_web.py`, `tests/test_console_commands.py`, `tests/test_console_view.py` 셋을 보여 준다. 나머지 11개(`tests/test_console_*.py`)는 질문이 나오면 4절 표로 답한다.
- **설계·증거 문서:** `docs/design/console-dataflow.md`([관제 웹 데이터 흐름](../design/console-dataflow.md): 어느 노드·토픽이 어느 장부 표를 거쳐 웹의 어느 파일·화면 칸으로 오는지, 파일마다 받는 것·하는 것·내보내는 것), `docs/design/console-spec.md`([관제 웹 설계 명세](../design/console-spec.md), 순서도는 8절), `docs/process/console-code-review.md`(이 문서), `docs/evidence/console-2026-10-13/README.md`([스크린샷·시험 로그](../evidence/console-2026-10-13/README.md)).
- **보지 않는 것:** `src/rescue_operations/`(operations API·DB 쓰기)는 다른 담당이다. 웹이 기대는 약속만 [설계 명세](../design/console-spec.md) 「웹이 기대는 인터페이스」로 설명한다. `src/rescue_console/rescue_console/__init__.py` 는 빈 파일이다.
- 슬라이드의 말과 웹의 대응: 웹은 ROS 노드가 아니라서 「입력 Topic」은 HTTP 요청과 현장 상태 DB 표, 「출력 Topic」은 HTTP 응답과 operations 로 보내는 POST 다. 그 DB 표가 어느 노드의 어느 토픽에서 오는지는 [관제 웹 데이터 흐름](../design/console-dataflow.md) 2·3절 표에 있다. 「Node/Class」 자리는 `app.py` 의 `create_app`(Flask 앱)이 맡는다.

## 1. 설계도에서 관제 웹 찾기

[시스템 전체 설계도](../design/rokey7-system-overview.drawio)의 「관제 웹 — 영상·지도·로봇 상태 / 출동·계속·정지 버튼」 상자와 화살표 셋이다.

| 화살표 | 그림의 뜻 | 실제 통신 (bd4bfa5) |
| --- | --- | --- |
| 웹캠 영상 | 영상 취득·중계 → 관제 웹 | `mjpeg_relay` 의 HTTP MJPEG 주소를 브라우저 `<img>` 로 받는다. 웹 서버는 영상을 중계하지 않는다(IF-082) |
| ② 출동 · ⑤ 계속 · 정지 | 운영자 → 관제 웹 | 브라우저 → `POST /commands`(로그인 세션·CSRF) |
| 명령 / 화면 조회 | 관제 웹 → operations | 명령: `POST /api/operator-commands`(operations, 127.0.0.1). 화면 조회: **operations 를 거치지 않고** 현장 상태 SQLite 를 읽기 전용으로 직접 연다 |

### 그림과 실제가 다른 곳 (리뷰에서 먼저 말할 것)

| 그림 | 실제 코드 | 근거 |
| --- | --- | --- |
| 화면 조회도 operations 를 거친다 | 웹이 현장 상태 DB 를 `mode=ro`·`query_only` 로 직접 읽는다. operations 로는 명령만 보낸다 | `sources.py` `load_site_state`, `rescue_operations/site_state.py:486-547` |
| 정지 버튼 | 늘 꺼져 있다. `RequestStop` 경로는 아직 어디에도 없고, 위험하면 현장 비상 정지 장치를 쓴다 | `view.py:58-63`, AGENTS.md 2절 「현장 정지 수단을 웹 연결에만 의존시키지 않는다」 |
| ⑤ 계속(구간 승인) | 현장 설정이 `support_dispatch: AUTO` 라 출동 승인·계속 단계가 없다. AUTO 면 출동 승인 버튼 대신 「TB-5 자동 출동」 상태만 보인다 | `config/field/search_area.field.json:29`, `view.py:605-611`, 결정 1B(PR #187) |
| 웹캠 영상 | 원본이 아니라 판정 영상(격자·ROI·상자를 그린 것)이다 | `rescue_bringup/operations_launch.py:154-155` |
| 지도 | 기준 지도(`--map`) 또는 운영 PC 가 SLAM 지도를 파일로 쓴 실시간 지도(`--live-map`) 중 하나 | [관제 웹 README](../../src/rescue_console/README.md) 「지도」·「실시간 SLAM 지도」 |

설계도는 main `c79097d` 기준이라 그 뒤의 자동 출동(#187)과 자동 수색 시작(#228)이 반영돼 있지 않다(그림 안 범례). 관제 웹을 `--search-start AUTO` 로 띄우면 지진 감지가 곧 TB-6 수색 시작이고, 「수색 시작」 버튼 대신 「TB-6 자동 출발」 상태가 보인다([README](../../src/rescue_console/README.md) `--search-start`). 같은 이유로 그림의 「② 출동」 화살표는 현장 설정(둘 다 AUTO)에서는 운영자가 누르지 않는다.

## 2. 설계 (Design) — [관제 웹 설계 명세](../design/console-spec.md)

명세는 「이 문서만 보고 같은 웹을 다시 짤 수 있게」를 목표로 10절로 적었다: 1 책임과 경계 · 2 실행 단위 · 3 실행 인자와 코드 상수 · 4 내부 상태(로그인 세션·명령 칸·갱신 표시·실시간 지도·카메라 칸 상태 기계) · 5 입력(HTTP 경로 13개, 폼 검사, 파일, operations 답) · 6 출력(화면 모델 JSON, 명령 응답, operations 요청, 로그, 화면 칸 배치) · 7 처리 규칙(대상 4단계 규칙표, 사유 문구표, 명령×단계 버튼 관문표 포함) · 8 순서도 2개(폴링 한 번, 명령 한 번) · 9 오류·예외 표 · 10 시험. 끝에 「웹이 기대는 인터페이스」(operations 명령 API·현장 상태 DB·`live_map.yaml`·직접 관측 파일·웹캠 API·로봇 카메라 중계 계약)와 「문서와 코드의 차이」를 두었다. 리뷰에서 설명하는 순서:

1. 설계도의 관제 웹 상자와 화살표 셋(1절) → 「웹은 보여 주고 명령을 전달만 한다. DB 쓰기·ROS 직접 구독·로봇 명령은 없다」(AGENTS.md 2절 관제·화면 레인).
2. 화면 한 장([스크린샷](../evidence/console-2026-10-13/02-dashboard.png))의 칸마다 어느 모듈이 그리는지.
3. 버튼 하나를 누를 때의 흐름: `render.py` JS `send()` → `app.py` `POST /commands` → `commands.py` → operations → 조정기 → 답 표시. 명세의 순서도로 설명한다.
4. 데이터가 없거나 낡거나 실패했을 때 무엇을 보이는가(지어내지 않음).

## 3. 코드 주석과 README (Code)

튜터 요구: Program·Node·Function·줄 단위 주석, README. 사용자 지시(2026-10-13): "코드를 보면서 리뷰를 할테니까 주석을 과하지 않은 수준으로 세세하게 작성을 해라." 그래서 이 PR 이 관제 웹 코드에 docstring 과 줄 주석을 더했다. **코드 글자는 바꾸지 않았다** — docstring·주석을 뺀 AST 와 줄 단위 비교가 `origin/main` 과 같다(PR 코멘트에 커밋별 확인).

측정 도구 `scripts/check_comments.py`(이 PR). 함수는 본문 5줄 이상만 센다. 줄 단위 주석은 기계로 판단하지 않고 개수만 보인다. 「전」은 `bd4bfa5`, 「후」는 이 브랜치(주석 + `origin/main` merge, `32e8ee1`)다 [측정, 2026-10-13].

| 파일 | 프로그램 | 노드·클래스 전 → 후 | 함수 전 → 후 | `#` 주석 / 코드 줄 전 → 후 |
| --- | --- | --- | --- | --- |
| `__main__.py` | 있음 | — | 0/1 → 1/1 | 0/69 → 6/77 |
| `accounts.py` | 있음 | 2/2 → 2/2 | 1/12 → 12/12 | 8/243 → 22/296 |
| `app.py` | 있음 | — | 6/17 → 19/19 | 23/268 → 45/325 |
| `commands.py` | 있음 | 1/1 → 1/1 | 6/6 → 6/6 | 10/127 → 23/127 |
| `labels.py` | 있음 | — | 1/1 → 1/1 | 0/58 → 1/60 |
| `mapimage.py` | 있음 | 1/2 → 2/2 | 7/9 → 9/9 | 10/241 → 32/255 |
| `render.py` | 있음 | — | 14/23 → 24/24 | 25/942 → 26/1043 (그 밖에 JS 함수마다 `//` 한 줄) |
| `sources.py` | 있음 | — | 3/3 → 3/3 | 4/54 → 12/58 |
| `throttle.py` | 있음 | 0/1 → 1/1 | 1/2 → 2/2 | 4/46 → 5/57 |
| `view.py` | 있음 | 1/1 → 1/1 | 11/16 → 18/18 | 51/513 → 86/583 |
| 합계 | 11/11 | 5/7 → 7/7 | 50/90 → 95/95 | 135/2562 → 258/2882 (`__init__.py` 1줄 포함) |

코드 줄 수가 늘어난 것은 `origin/main` 의 새 기능(출구 QR 표시, `--search-start`, 사유 문구)과 여러 줄 docstring 이다. 빠진 docstring 은 0곳이다: `python3 -I scripts/check_comments.py --strict src/rescue_console` exit 0.

**주석을 쓴 기준.** 무엇을 하는지는 코드가 말한다. 주석은 **왜**와 **실패하면 무엇을 돌려주는가**를 적었다. 결정 번호·계약 ID 는 저장소에 이미 있는 것만 인용했다.

| 수준 | 적은 것 | 리뷰에서 보여 줄 예 |
| --- | --- | --- |
| 프로그램 | 이 파일이 맡은 일, 입력·출력, 하지 않는 일 | `commands.py`, `app.py`, `sources.py` 첫 docstring |
| 노드·클래스 | 책임과 내부 상태 | `accounts.py` `_Transaction`, `throttle.py` `LoginThrottle`, `mapimage.py` `MapImage`·`LiveMap` |
| 함수 | 입력 → 출력, 실패할 때의 반환·예외, 이유 | `view.py` `build_view`·`compose_stage`·`_gate`, `app.py` `current_view` |
| 줄 | 임계값·상수의 출처, 보안 선택(CSRF 상수 시간 비교, 세션 무효화, 읽기 전용 열기), 의외의 분기 | `view.py` 관문 순서 1~4, `app.py` 세션 쿠키 설정, `render.py` JS `poll`·`send` 위 `//` |

**README:** [`src/rescue_console/README.md`](../../src/rescue_console/README.md)에 실행 명령, 인자, 화면 칸, 명령 패널, 지도·실시간 지도, 사유 문구 표가 있다. 리뷰에서는 「실행 (로봇 없이, E2)」 절을 그대로 보여 주면 된다. 코드와 달랐던 3곳(읽기 함수 개수, 한 줄로 쌓이는 조건, 갱신 실패 문구)은 이 PR 에서 고쳤다.

## 4. 시험과 검증 (Test/Validation)

| 종류 | 무엇 | 결과 | 위치 |
| --- | --- | --- | --- |
| 시험 코드 | `tests/test_console_*.py` 14 파일 | Flask 3.1.3 설치 상태에서 `bd4bfa5` 198 OK(skip 0) [측정, 2026-10-13, Python 3.13, 로그]. `32e8ee1`(main merge 뒤) 209 OK [측정, 2026-10-13]. main `ede7f8b`(직접 관측·터널 판정 시험 2 파일 추가 뒤) 223 OK(skip 0), main `4163a25` 합친 뒤(웹캠 고르기·로봇 카메라 시험 2 파일 추가) 235 OK(skip 0) [측정, 2026-10-13, Python 3.13, Flask 3.1.3] | [시험 로그](../evidence/console-2026-10-13/console-tests-flask.log) |
| CI | `foundation`(Python 3.10 ~ 3.12) | 이 PR 전에는 CI 에 Flask 가 없어 웹 경로 시험 23건이 skip(NOT_RUN)이었다. 이 PR 이 `flask==3.1.3` 설치를 더했고, 커밋 `484c492` 의 3.10 작업에서 `Ran 1514 tests` · `OK (skipped=15)` [측정, 2026-10-13]. skip 15 는 로컬 Flask 환경의 skip 15(NumPy·OpenCV·PyYAML·ROS 없음)와 수가 같다. CI 쪽 이유별 내역은 로그에 없어 [미확인] | [.github/workflows/foundation.yml](../../.github/workflows/foundation.yml) |
| 스크린샷 | 로그인, 관제 화면(합성 데이터, `bd4bfa5` 코드 — 출구 QR 표시 전) | 2장 | [console-2026-10-13](../evidence/console-2026-10-13/README.md) |
| 현장 기록 | 운영 PC 에서 관제 웹 기동(장부 없이 #147), `--camera` 로 MJPEG 수신(#158) | PASS [측정, 2026-10-13 12:42·12:54] | [10-13 운영 PC 기록](../evidence/field-2026-10-13-operations-host.md) 25·27줄 |
| 영상 | 화면 녹화 | 없음 | NOT_RUN |

시험 파일마다 보장하는 것:

| 시험 파일 | 개수 | 보장하는 것 |
| --- | --- | --- |
| `test_console_web.py` | 42 | 모든 페이지에 로그인 필요, 계정 관리는 ADMIN 만, 데이터 경로는 읽기 전용(GET) |
| `test_console_commands.py` | 41 | 버튼 5종만 operations 로 한 번 보냄, 요청 필드가 조정기 관문과 같음, 재시도 없음, 답을 그대로 보임 |
| `test_console_view.py` | 42 | 장부 → 화면 모델이 1:1, 읽기 전용, Q-20 규칙표 |
| `test_console_map.py` | 20 | PGM → PNG 정확, 지도 좌표 → 픽셀, 위치를 지어내지 않음 |
| `test_console_reasons.py` | 26 | `SystemState.reason` 코드를 쉬운 한국어로 |
| `test_console_accounts.py` | 9 | admin/customer 계정, admin 이 customer 를 관리 |
| `test_console_history.py` | 8 | 명령·오류 기록(거절 포함)을 operations 가 기록한 그대로 |
| `test_console_live_map.py` | 11 | 실시간 지도 파일이 바뀌면 다시 읽음, 없음·낡음·읽기 실패를 숨기지 않음 |
| `test_console_throttle.py` | 7 | 같은 이름·같은 클라이언트의 연속 로그인 실패를 잠시 막음 |
| `test_console_labels.py` | 3 | 계약 값마다 한국어 표시가 있고 모르는 코드는 그대로 보임 |
| `test_console_robot_watch.py` | 9 | `--robot-watch` 직접 관측: 상태 없는 로봇의 카드·위치를 채우고, 프로젝트 상태가 먼저, 낡음·파일 멈춤·배치 밖·파일 깨짐을 숨기지 않음 |
| `test_console_webcam_switch.py` | 7 | 웹캠 고르기는 ADMIN 만, 재난 모드면 한 번 더 묻기, 한 번 보내고 답·거절을 그대로, operations 꺼짐은 문구 |
| `test_console_robot_camera.py` | 5 | 로봇 카메라 칸: http(s) 주소만, 누구나 확인 창 없이 고름, CSRF, 중계 꺼짐은 문구 |
| `test_console_tunnel_map.py` | 5 | 터널 판정 세 상태의 낱말·색, 옛 지도 revision 흐림, 지금 지도 모름 경고, 지도 밖은 목록 |

개수는 main `4163a25` 합친 뒤 `def test_` 를 센 값이다(합계 235, 실행 결과와 같음) [계산, 2026-10-13].

## 5. 예상 질문과 답

1. **웹은 왜 DB 에 쓰지 않나? 읽기는 왜 직접 하나?** SQLite 파일마다 쓰는 프로세스는 operations 하나다. 나머지는 읽기 전용으로만 연다([DB 스키마](../design/db-schema.md) 9줄). 그래서 명령 기록도 웹이 아니라 operations 가 남긴다. 읽기를 HTTP 로 감싸지 않은 것은 표시만 하는 데 프로세스를 하나 더 두지 않으려는 것이다. 계정 DB 만 웹이 소유한다.
2. **버튼을 꺼 두면 안전한가?** 아니다. 버튼을 켜고 끄는 것은 조정기 관문을 화면에 비춘 것뿐이다. 진짜 관문은 조정기다(`view.py:597-601` `command_gates` docstring 「A mirror of the coordinator gate」, 버튼별 관문 `_gate` `:530`, [관제 웹·DB 계획](../design/console-and-db-plan.md) 4절). 그래서 버튼이 켜져 있어도 조정기가 거절할 수 있고, 그 거절 사유를 그대로 보인다.
3. **명령 응답이 늦거나 끊기면?** 다시 보내지 않는다(`commands.py:1-7`, AGENTS.md 7절). operations 는 시간 초과 때 `accepted` 를 넣지 않고 503·504 를 준다. 조정기가 실제로 받았을 수 있기 때문이다. 웹은 「전달 여부 모름」으로 보이고, 같은 `command_id` 를 다시 보내면 조정기가 처음 답을 DUPLICATE 로 돌려준다.
4. **정지 버튼은 왜 없나?** 현장 정지 수단을 웹 연결에 기대지 않는다(AGENTS.md 2절). 웹·Wi-Fi 가 끊기면 정지도 안 되기 때문이다. `RequestStop` 별도 경로는 아직 구현되지 않았고, 위험하면 현장 비상 정지 장치를 쓴다(`view.py:62`).
5. **화면은 어떻게 갱신하나? 왜 WebSocket 이 아닌가?** 브라우저가 1초마다 `/fragment` 를 가져온다(`render.py:14`). 표준 라이브러리와 Flask 만으로 되고, 끊겨도 다음 폴링에서 복구된다. 영상 `<img>` 는 바뀌는 조각 밖에 있어서 폴링이 영상을 끊지 않는다. SSE(IF-081)는 계획만 있다.
6. **영상이 끊기면?** `--camera-stale-s` 초 동안 새 프레임이 없으면 「영상 끊김」을 띄우고 그 주기로 다시 연결한다. 기본값은 없다(현장 값). 정상으로 닫힌 multipart 스트림은 마지막 프레임을 남겨 끊긴 줄 모르기 때문이다(README 「웹캠」, 측정 2026-10-13 Firefox).
7. **보안은?** 모든 페이지 로그인, ADMIN 만 명령·계정 관리, 폼·명령마다 세션 CSRF 토큰(`app.py:1-6`), 로그인 연속 실패 제한(`throttle.py`). 세션 비밀값은 환경 변수(`RESCUE_CONSOLE_SECRET`)로만 받는다. operations API 는 자체 로그인이 없어서 127.0.0.1 에만 묶는다.
8. **낡은 데이터는?** 숨기지 않는다. 로봇 상태가 낡으면 「낡음: N초 동안 새 상태 없음」, 위치가 없는 로봇은 지도에 그리지 않고 목록에 「위치 미확정」, 지도 파일이 낡거나 못 읽으면 경고를 띄운다([스크린샷](../evidence/console-2026-10-13/02-dashboard.png)의 TB-6 카드).
9. **자동 출동(AUTO)이면 웹은 무엇을 하나?** 출동 승인 버튼 대신 「TB-5 자동 출동」 상태와 아직 출동하지 않은 이유(SUPPORT_*)를 보인다. 조정기의 설정값을 IDL 로 싣지 않아서 웹은 `--support-dispatch` 인자로 받는다(README 48줄).

## 6. 리뷰 전 확인

- [x] docstring 채우기 — 42곳 모두, 줄 주석까지(이 PR). `python3 -I scripts/check_comments.py --strict src/rescue_console` exit 0.
- [x] 관제 웹 README 낡은 3곳, `setup.py` 설명("no commands"), `labels.py` 의 "no producer yet" 주석 — 이 PR 에서 고침.
- [x] 이 PR 의 CI(`foundation`, Flask 포함) 초록, 3.10 에서 skip 15 로 줄어 웹 경로 시험이 실제로 돎(4절).
- [ ] 코드를 읽다 찾은 문제 4개(코드 읽기, 시험 없음). 고치려면 코드 변경이라 이 PR 에 넣지 않았다. 리뷰에서 튜터가 먼저 찾기 전에 고치거나 「알려진 한계」로 말한다:
  - `accounts.py:348` `exc_type is sqlite3.Error` 는 하위 클래스(`OperationalError` 등)와 맞지 않아, 계정 DB 트랜잭션 중 sqlite 오류가 `STORAGE_ERROR`(503) 대신 Flask 500 이 된다. 중복 계정은 미리 검사하므로 DB 잠김·디스크 오류 때의 경로다.
  - `render.py:1096` `setInterval(poll, 1000)` 은 앞 요청을 기다리지 않는다. 서버가 1초 넘게 늦으면 늦게 온 옛 조각이 새 조각을 덮을 수 있다(다음 폴링에서 바로잡힘).
  - `app.py` CSRF 비교가 `session.get('csrf', '-')` 라서 토큰이 없는 세션에서는 폼 `csrf=-` 가 통과한다. 로그인한 세션에는 늘 토큰이 있어 `POST /commands` 와 계정 관리에는 해당 없고, 로그인 전 `POST /login`·`/logout` 의 사이트 간 요청 위조 방어가 빈다.
  - `labels.py` 에 `RobotStatus` 의 `DEGRADED` 낱말이 없어 화면에 코드 그대로 보인다.
- [ ] 기동 로그에 git SHA 가 없다(AGENTS.md 8절 「노드는 기동할 때 git SHA … 를 로그에 남긴다」, [명세](../design/console-spec.md) 6.5). 웹은 ROS 노드가 아니지만 같은 이유로 남길지 정한다.
- [ ] 작전 레인 문서의 낡은 곳(`docs/design/console-and-db-plan.md`·`docs/process/ops-db-tables.md`·`docs/design/db-schema.md`, [명세](../design/console-spec.md) 끝 「문서와 코드의 차이」)은 작전 레인에 요청.
- [ ] 스크린샷 다시 찍기: 지금 2장은 `bd4bfa5` 코드라 출구(QR) 표시와 「TB-6 자동 출발」 상태가 없다. 현장에서는 실제 operations·조정기에 연결해 명령 접수·거절이 보이는 화면, 웹캠 영상이 나오는 화면, `--live-map` 실시간 지도 화면을 날짜·커밋과 함께 `docs/evidence/` 에.
- [ ] 화면 녹화 1개: 지진 표시 → 자동 수색 시작 → 대상 표시 → 자동 출동 → 판정.
- [ ] 설명 순서 연습: 설계도(1절) → 명세 순서도 → 코드(VS Code, 0절 파일 순서) → 시험·스크린샷(4절).

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

관제 웹 범위는 아래 파일이 전부다. 위에서부터 이 순서로 연다. 경로는 모두 `src/rescue_console/` 아래다.

| 순서 | 파일 | 슬라이드의 말로 | 이 파일에서 보여 줄 것 |
| --- | --- | --- | --- |
| 1 | `README.md` | README.md | 로봇 없이 띄우는 명령(「실행 (로봇 없이, E2)」), 화면 칸 설명 |
| 2 | `rescue_console/__main__.py` | Main() | 실행 인자 검사 → `create_app` → 서버 기동. 실패하면 서버를 띄우지 않고 종료코드 2 |
| 3 | `rescue_console/app.py` | Node/Class | 라우트 전부, 로그인·세션 무효화·CSRF, `POST /commands` 가 명령을 넘기는 곳 |
| 4 | `rescue_console/sources.py` | 입력 Topic | 현장 상태 DB 를 읽기 전용으로 여는 곳(웹은 DB 에 쓰지 않음) |
| 5 | `rescue_console/view.py` | Function(핵심) | 입력 → 화면 모델: 대상 4단계 규칙, 낡음 판정, 버튼 관문 거울 `_gate`·`command_gates` |
| 6 | `rescue_console/commands.py` | 출력 Topic | operations 로 한 번 POST, 재시도 없음, 답 검사 |
| 7 | `rescue_console/render.py` | Function(화면) | HTML 조각, 1초 폴링·명령 확인 칸·영상 끊김 감지 JS |
| 8 | `rescue_console/mapimage.py` | Function | 지도 PGM → PNG, 로봇·대상 위치 표시, 실시간 지도 낡음 |
| 9 | `rescue_console/accounts.py`, `rescue_console/throttle.py` | Function | 계정·비밀번호 해시·역할, 로그인 연속 실패 잠금 |
| 10 | `rescue_console/labels.py` | 변수 | 코드 → 한국어 낱말 표 |

- **시험(Test Code):** `tests/test_console_web.py`, `tests/test_console_commands.py`, `tests/test_console_view.py` 셋을 보여 준다. 나머지 7개(`test_console_*.py`)는 질문이 나오면 4절 표로 답한다.
- **설계·증거 문서:** [관제 웹 설계 명세](../design/console-spec.md)(순서도는 8절), 이 문서, [스크린샷·시험 로그](../evidence/console-2026-10-13/README.md).
- **보지 않는 것:** `src/rescue_operations/`(operations API·DB 쓰기)는 다른 담당이다. 웹이 기대는 약속만 [설계 명세](../design/console-spec.md) 「웹이 기대는 인터페이스」로 설명한다. `rescue_console/__init__.py` 는 빈 파일이다.
- 슬라이드의 말과 웹의 대응: 웹은 ROS 노드가 아니라서 「입력 Topic」은 HTTP 요청과 현장 상태 DB 표, 「출력 Topic」은 HTTP 응답과 operations 로 보내는 POST 다. 「Node/Class」 자리는 `app.py` 의 `create_app`(Flask 앱)이 맡는다.

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
| 화면 조회도 operations 를 거친다 | 웹이 현장 상태 DB 를 `mode=ro`·`query_only` 로 직접 읽는다. operations 로는 명령만 보낸다 | `sources.py` `load_site_state`, `rescue_operations/site_state.py:438-459` |
| 정지 버튼 | 늘 꺼져 있다. `RequestStop` 경로는 아직 어디에도 없고, 위험하면 현장 비상 정지 장치를 쓴다 | `view.py:58-63`, AGENTS.md 2절 「현장 정지 수단을 웹 연결에만 의존시키지 않는다」 |
| ⑤ 계속(구간 승인) | 현장 설정이 `support_dispatch: AUTO` 라 출동 승인·계속 단계가 없다. AUTO 면 출동 승인 버튼 대신 「TB-5 자동 출동」 상태만 보인다 | `config/field/search_area.field.json:29`, `view.py:533-538`, 결정 1B(PR #187) |
| 웹캠 영상 | 원본이 아니라 판정 영상(격자·ROI·상자를 그린 것)이다 | `rescue_bringup/operations_launch.py:143-144` |
| 지도 | 기준 지도(`--map`) 또는 운영 PC 가 SLAM 지도를 파일로 쓴 실시간 지도(`--live-map`) 중 하나 | [관제 웹 README](../../src/rescue_console/README.md) 「지도」·「실시간 SLAM 지도」 |

설계도는 main `c79097d` 기준이라 그 뒤의 자동 출동(#187)이 반영돼 있지 않다(그림 안 범례). #225(시나리오 결정: 지진 감지로 TB-6 자동 출발)가 머지되면 「수색 시작」 버튼의 뜻도 바뀔 수 있다 — 리뷰 전에 확인한다.

## 2. 설계 (Design) — [관제 웹 설계 명세](../design/console-spec.md)

명세는 「이 문서만 보고 같은 웹을 다시 짤 수 있게」를 목표로 10절로 적었다: 1 책임과 경계 · 2 실행 단위 · 3 실행 인자와 코드 상수 · 4 내부 상태(로그인 세션·명령 칸·갱신 표시·실시간 지도·카메라 칸 상태 기계) · 5 입력(HTTP 경로 11개, 폼 검사, 파일, operations 답) · 6 출력(화면 모델 JSON, 명령 응답, operations 요청, 로그, 화면 칸 배치) · 7 처리 규칙(대상 4단계 규칙표, 사유 문구표, 명령×단계 버튼 관문표 포함) · 8 순서도 2개(폴링 한 번, 명령 한 번) · 9 오류·예외 표 · 10 시험. 끝에 「웹이 기대는 인터페이스」(operations 명령 API·현장 상태 DB·`live_map.yaml` 계약)와 「문서와 코드의 차이」를 두었다. 리뷰에서 설명하는 순서:

1. 설계도의 관제 웹 상자와 화살표 셋(1절) → 「웹은 보여 주고 명령을 전달만 한다. DB 쓰기·ROS 직접 구독·로봇 명령은 없다」(AGENTS.md 2절 관제·화면 레인).
2. 화면 한 장([스크린샷](../evidence/console-2026-10-13/02-dashboard.png))의 칸마다 어느 모듈이 그리는지.
3. 버튼 하나를 누를 때의 흐름: `render.py` JS `send()` → `app.py` `POST /commands` → `commands.py` → operations → 조정기 → 답 표시. 명세의 순서도로 설명한다.
4. 데이터가 없거나 낡거나 실패했을 때 무엇을 보이는가(지어내지 않음).

## 3. 코드 주석과 README (Code)

튜터 요구: Program·Node·Function·줄 단위 주석, README. 측정 도구 `scripts/check_comments.py`(이 PR)로 쟀다 [측정, 2026-10-13, `bd4bfa5`]. 함수는 본문 5줄 이상만 센다. 줄 단위 주석은 기계로 판단하지 않고 개수만 보인다.

| 파일 | 프로그램(모듈 docstring) | 노드·클래스 | 함수 | `#` 주석 / 코드 줄 |
| --- | --- | --- | --- | --- |
| `__main__.py` | 있음 | — | 0/1 | 0 / 69 |
| `accounts.py` | 있음 | 2/2 | 1/12 | 8 / 243 |
| `app.py` | 있음 | — | 6/17 | 23 / 268 |
| `commands.py` | 있음 | 1/1 | 6/6 | 10 / 127 |
| `labels.py` | 있음 | — | 1/1 | 0 / 58 |
| `mapimage.py` | 있음 | 1/2 | 7/9 | 10 / 241 |
| `render.py` | 있음 | — | 14/23 | 25 / 942 |
| `sources.py` | 있음 | — | 3/3 | 4 / 54 |
| `throttle.py` | 있음 | 0/1 | 1/2 | 4 / 46 |
| `view.py` | 있음 | 1/1 | 11/16 | 51 / 513 |
| 합계 (11 파일, `__init__.py` 포함) | 11/11 | 5/7 | 50/90 | 135 / 2562 |

docstring 이 빠진 42곳(`파일:줄`):

| 파일 | 함수·클래스 |
| --- | --- |
| `__main__.py` | `main`:17 |
| `accounts.py` | `hash_password`:56, `verify_password`:64, `create`:109, `open`:138, `_connect`:150, `add`:191, `update`:203, `delete`:233, `__enter__`:255, `__exit__`:263, `main`:279 |
| `app.py` | `_check_sources`:36, `robot_name_map`:76, `harden`:122, `login_required`:159, `wrapped`:161, `admin_required`:172, `failed`:182, `login`:200, `map_png`:246, `accounts_create`:294, `accounts_change`:308 |
| `mapimage.py` | `MapImage`:128, `_scalar`:33, `__init__`:199 |
| `render.py` | `_short`:71, `_elapsed`:83, `_pose`:94, `_battery`:268, `_robots`:279, `_targets`:307, `_event_text`:347, `_events`:357, `_history`:389 |
| `throttle.py` | `LoginThrottle`:21, `failed`:43 |
| `view.py` | `_section`:118, `_latest_reports`:127, `_robots`:217, `_site_events`:258, `_events`:268 |

다시 재는 법: `python3 -I scripts/check_comments.py src/rescue_console` (보고, exit 0). 다 채웠는지 확인: `--strict` (빠진 곳이 있으면 exit 1).

**채우는 기준.** 무엇을 하는지는 코드가 말한다. 주석은 **왜**와 **실패하면 무엇을 돌려주는가**를 적는다.

| 수준 | 적을 것 | 이미 잘 된 예 |
| --- | --- | --- |
| 프로그램 | 이 파일이 맡은 일, 입력·출력, 하지 않는 일 | `commands.py:1-8`, `app.py:1-6` |
| 노드·클래스 | 책임과 내부 상태 | `view.py` `ConsoleError` |
| 함수 | 입력 → 출력, 실패할 때의 반환·예외, 근거(결정 번호·계약 ID) | `commands.py` 함수 6개 모두 |
| 줄 | 임계값·상수의 출처(누가 언제 정했나), 의외의 분기 이유 | `view.py:57`(`HELD_COMMANDS` 위 「작전 2026-10-12, #115」) |

**README:** [`src/rescue_console/README.md`](../../src/rescue_console/README.md)(246줄)에 실행 명령, 인자, 화면 칸, 명령 패널, 지도·실시간 지도, 사유 문구 표가 있다. 리뷰에서는 「실행 (로봇 없이, E2)」 절을 그대로 보여 주면 된다.

## 4. 시험과 검증 (Test/Validation)

| 종류 | 무엇 | 결과 | 위치 |
| --- | --- | --- | --- |
| 시험 코드 | `tests/test_console_*.py` 10 파일, 198개 | Flask 3.1.3 설치 상태에서 198 OK, skip 0, exit 0 [측정, 2026-10-13, Python 3.13] | [시험 로그](../evidence/console-2026-10-13/console-tests-flask.log) |
| CI | `foundation`(Python 3.10 ~ 3.12) | 이 PR 전에는 CI 에 Flask 가 없어 웹 경로 시험 23건이 skip(NOT_RUN)이었다. 이 PR 이 `flask==3.1.3` 설치를 더했고, 커밋 `484c492` 의 3.10 작업에서 `Ran 1514 tests` · `OK (skipped=15)` [측정, 2026-10-13]. skip 15 는 로컬 Flask 환경의 skip 15(NumPy·OpenCV·PyYAML·ROS 없음)와 수가 같다. CI 쪽 이유별 내역은 로그에 없어 [미확인] | [.github/workflows/foundation.yml](../../.github/workflows/foundation.yml) |
| 스크린샷 | 로그인, 관제 화면(합성 데이터) | 2장 | [console-2026-10-13](../evidence/console-2026-10-13/README.md) |
| 현장 기록 | 운영 PC 에서 관제 웹 기동(장부 없이 #147), `--camera` 로 MJPEG 수신(#158) | PASS [측정, 2026-10-13 12:42·12:54] | [10-13 운영 PC 기록](../evidence/field-2026-10-13-operations-host.md) 25·27줄 |
| 영상 | 화면 녹화 | 없음 | NOT_RUN |

시험 파일마다 보장하는 것:

| 시험 파일 | 개수 | 보장하는 것 |
| --- | --- | --- |
| `test_console_web.py` | 42 | 모든 페이지에 로그인 필요, 계정 관리는 ADMIN 만, 데이터 경로는 읽기 전용(GET) |
| `test_console_commands.py` | 41 | 버튼 5종만 operations 로 한 번 보냄, 요청 필드가 조정기 관문과 같음, 재시도 없음, 답을 그대로 보임 |
| `test_console_view.py` | 41 | 장부 → 화면 모델이 1:1, 읽기 전용, Q-20 규칙표 |
| `test_console_map.py` | 20 | PGM → PNG 정확, 지도 좌표 → 픽셀, 위치를 지어내지 않음 |
| `test_console_reasons.py` | 19 | `SystemState.reason` 코드를 쉬운 한국어로 |
| `test_console_accounts.py` | 9 | admin/customer 계정, admin 이 customer 를 관리 |
| `test_console_history.py` | 8 | 명령·오류 기록(거절 포함)을 operations 가 기록한 그대로 |
| `test_console_live_map.py` | 8 | 실시간 지도 파일이 바뀌면 다시 읽음, 없음·낡음·읽기 실패를 숨기지 않음 |
| `test_console_throttle.py` | 7 | 같은 이름·같은 클라이언트의 연속 로그인 실패를 잠시 막음 |
| `test_console_labels.py` | 3 | 계약 값마다 한국어 표시가 있고 모르는 코드는 그대로 보임 |

개수는 `def test_` 를 센 값이다 [계산].

## 5. 예상 질문과 답

1. **웹은 왜 DB 에 쓰지 않나? 읽기는 왜 직접 하나?** SQLite 파일마다 쓰는 프로세스는 operations 하나다. 나머지는 읽기 전용으로만 연다([DB 스키마](../design/db-schema.md) 9줄). 그래서 명령 기록도 웹이 아니라 operations 가 남긴다. 읽기를 HTTP 로 감싸지 않은 것은 표시만 하는 데 프로세스를 하나 더 두지 않으려는 것이다. 계정 DB 만 웹이 소유한다.
2. **버튼을 꺼 두면 안전한가?** 아니다. 버튼을 켜고 끄는 것은 조정기 관문을 화면에 비춘 것뿐이다. 진짜 관문은 조정기다(`view.py:526-529` `command_gates` docstring 「A mirror of the coordinator gate」, 버튼별 관문 `_gate` `:475`, [관제 웹·DB 계획](../design/console-and-db-plan.md) 4절). 그래서 버튼이 켜져 있어도 조정기가 거절할 수 있고, 그 거절 사유를 그대로 보인다.
3. **명령 응답이 늦거나 끊기면?** 다시 보내지 않는다(`commands.py:1-7`, AGENTS.md 7절). operations 는 시간 초과 때 `accepted` 를 넣지 않고 503·504 를 준다. 조정기가 실제로 받았을 수 있기 때문이다. 웹은 「전달 여부 모름」으로 보이고, 같은 `command_id` 를 다시 보내면 조정기가 처음 답을 DUPLICATE 로 돌려준다.
4. **정지 버튼은 왜 없나?** 현장 정지 수단을 웹 연결에 기대지 않는다(AGENTS.md 2절). 웹·Wi-Fi 가 끊기면 정지도 안 되기 때문이다. `RequestStop` 별도 경로는 아직 구현되지 않았고, 위험하면 현장 비상 정지 장치를 쓴다(`view.py:62`).
5. **화면은 어떻게 갱신하나? 왜 WebSocket 이 아닌가?** 브라우저가 1초마다 `/fragment` 를 가져온다(`render.py:14`). 표준 라이브러리와 Flask 만으로 되고, 끊겨도 다음 폴링에서 복구된다. 영상 `<img>` 는 바뀌는 조각 밖에 있어서 폴링이 영상을 끊지 않는다. SSE(IF-081)는 계획만 있다.
6. **영상이 끊기면?** `--camera-stale-s` 초 동안 새 프레임이 없으면 「영상 끊김」을 띄우고 그 주기로 다시 연결한다. 기본값은 없다(현장 값). 정상으로 닫힌 multipart 스트림은 마지막 프레임을 남겨 끊긴 줄 모르기 때문이다(README 「웹캠」, 측정 2026-10-13 Firefox).
7. **보안은?** 모든 페이지 로그인, ADMIN 만 명령·계정 관리, 폼·명령마다 세션 CSRF 토큰(`app.py:1-6`), 로그인 연속 실패 제한(`throttle.py`). 세션 비밀값은 환경 변수(`RESCUE_CONSOLE_SECRET`)로만 받는다. operations API 는 자체 로그인이 없어서 127.0.0.1 에만 묶는다.
8. **낡은 데이터는?** 숨기지 않는다. 로봇 상태가 낡으면 「낡음: N초 동안 새 상태 없음」, 위치가 없는 로봇은 지도에 그리지 않고 목록에 「위치 미확정」, 지도 파일이 낡거나 못 읽으면 경고를 띄운다([스크린샷](../evidence/console-2026-10-13/02-dashboard.png)의 TB-6 카드).
9. **자동 출동(AUTO)이면 웹은 무엇을 하나?** 출동 승인 버튼 대신 「TB-5 자동 출동」 상태와 아직 출동하지 않은 이유(SUPPORT_*)를 보인다. 조정기의 설정값을 IDL 로 싣지 않아서 웹은 `--support-dispatch` 인자로 받는다(README 48줄).

## 6. 리뷰 전 확인

- [ ] docstring 42곳 채우기(관제·화면 레인 PR). `python3 -I scripts/check_comments.py --strict src/rescue_console` 가 exit 0.
- [ ] 명세를 쓰며 찾은 코드 문제 2개(코드 읽기, 시험 없음) — 리뷰에서 튜터가 먼저 찾기 전에 고치거나 「알려진 한계」로 말한다:
  - `accounts.py:274` `exc_type is sqlite3.Error` 는 하위 클래스(`OperationalError` 등)와 맞지 않아, 계정 DB 트랜잭션 중 sqlite 오류가 `STORAGE_ERROR`(503) 대신 Flask 500 이 된다. 중복 계정은 미리 검사하므로 DB 잠김·디스크 오류 때의 경로다.
  - `render.py:959` `setInterval(poll, 1000)` 은 앞 요청을 기다리지 않는다. 서버가 1초 넘게 늦으면 늦게 온 옛 조각이 새 조각을 덮을 수 있다(다음 폴링에서 바로잡힘).
- [ ] 기동 로그에 git SHA 가 없다(AGENTS.md 8절 「노드는 기동할 때 git SHA … 를 로그에 남긴다」, [명세](../design/console-spec.md) 6.5). 웹은 ROS 노드가 아니지만 같은 이유로 남길지 정한다.
- [ ] 관제 웹 레인 문서의 낡은 곳 고치기([명세](../design/console-spec.md) 끝 「문서와 코드의 차이」): `src/rescue_console/README.md` 14·93·175줄, `setup.py` 20줄 설명("no commands"), `labels.py:8-12` 주석. `docs/design/console-and-db-plan.md`·`docs/process/ops-db-tables.md`·`docs/design/db-schema.md` 의 낡은 곳은 작전 레인에 요청.
- [ ] 이 PR 의 CI(`foundation`, Flask 포함)가 초록인지. 웹 경로 시험 23건이 3.10 ~ 3.12 에서 실제로 돌았는지 로그로 확인.
- [ ] 현장에서 찍을 것: 실제 operations·조정기에 연결해 명령 접수·거절이 보이는 화면, 웹캠 영상이 나오는 화면, `--live-map` 실시간 지도 화면. 날짜·커밋과 함께 `docs/evidence/` 에.
- [ ] 화면 녹화 1개: 지진 표시 → 수색 시작 → 대상 표시 → 자동 출동 → 판정.
- [ ] #225 머지 여부에 따라 「수색 시작」 버튼 설명 갱신.
- [ ] 설명 순서 연습: 설계도(1절) → 명세 순서도 → 코드(VS Code) → 시험·스크린샷(4절).

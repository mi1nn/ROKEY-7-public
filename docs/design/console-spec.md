---
title: 관제 웹(rescue_console) 설계 명세
status: draft
updated: 2026-10-13
---

# 관제 웹(rescue_console) 설계 명세

> 코드 리뷰에서 이 문서를 쓰는 순서와 주석·시험 증거는 [관제 웹 코드 리뷰 준비](../process/console-code-review.md)에 있다. 화면 예시는 [합성 데이터 스크린샷](../evidence/console-2026-10-13/02-dashboard.png)이다.

이 문서는 관제 웹 `rescue_console`(Flask 웹 서비스이고 ROS 노드가 아니다)을 **이 명세만 보고 같은 동작으로 다시 짤 수 있게** 적는다. 코드는 옮기지 않고 규칙·조건·문구·순서를 문장과 표로 적는다. 처음에는 `bd4bfa5` 를 보고 썼고 `32e8ee1` 에서 다시 확인했다. 2026-10-13 main `ede7f8b` 에서 그 뒤 들어온 로봇 직접 관측(`--robot-watch`)과 터널 판정 표시를 더했고(7.4.7·7.4.8·I-5), **모든 줄 표기를 `ede7f8b` 로 옮겼다**: 옛 표기가 가리키던 줄(웹·`rescue_operations/site_state.py`·`rescue_operations/ros_ingest.py` 는 `32e8ee1`, 그 밖은 `bd4bfa5`)을 diff 로 지금 줄에 맞춰 옮기고, 옛 줄과 새 줄의 글자가 크게 다른 끝점은 손으로 열어 봤다(달라진 것은 주석 문장뿐이고 구조가 바뀐 3곳은 고쳤다). 노드·토픽에서 웹 화면까지의 사슬과 파일별 입출력 요약은 [관제 웹 데이터 흐름](console-dataflow.md)이다. 줄 표기 규칙: 접두어 없는 `app.py:…`·`view.py:…` 등은 `src/rescue_console/rescue_console/` 아래 파일, `rescue_operations/…` 는 `src/rescue_operations/rescue_operations/`, `rescue_bringup/…` 는 `src/rescue_bringup/rescue_bringup/`, `launch/…` 는 `src/rescue_bringup/launch/`, `README:…` 는 `src/rescue_console/README.md` 다. operations 쪽 노드(db_writer·report_node·map_snapshot)의 내부는 다루지 않고, 마지막 「웹이 기대는 인터페이스」 절에 웹이 기대는 계약(명령 API·현장 상태 DB 의 쓰는 열·`live_map.yaml`)만 적는다. 값의 출처 태그는 팀 `AGENTS.md` 5절을 따른다.

## rescue-console — 현장 상태 DB 를 읽기 전용으로 읽어 한 화면에 보이고, ADMIN 의 운영자 명령을 operations HTTP API 로 한 번 넘기는 로그인 웹

### 1. 책임과 경계

하는 일
- 요청이 올 때마다 operations 의 현장 상태 DB(`operations-site-state` schema 1.4)를 **읽기 전용**으로 읽고, 주어졌으면 옛 보고 장부(`operations-ledger` 1.0)도 읽어, 화면 모델 하나로 합성해 HTML·JSON 으로 낸다(`app.py:50-82`, `sources.py:75-99`, `view.py:692-790`).
- 표시층에서 구조 상태 4단계(Q-20), 임무 띠, 조정기 사유 문구, 명령 버튼의 **관문 거울**을 계산한다. 거울은 관문이 아니다: 켜진 버튼도 조정기가 거절할 수 있고 그 답을 그대로 보인다(`view.py:598-601`).
- ADMIN 이 누른 명령 한 건을 operations `POST /api/operator-commands` 로 **한 번** 보내고 답(또는 실패)을 그대로 돌려준다(`app.py:324-356`, `commands.py:168-175`).
- 로그인·세션·CSRF·계정 관리. 콘솔이 쓰는 파일은 콘솔 소유 계정 DB 하나뿐이다(`accounts.py:1-12`).
- 지도(고정 map_server 지도 `--map` 또는 map_snapshot 이 쓰는 실시간 SLAM 지도 `--live-map`)를 PNG 로 바꿔 주고, 로봇·요구조자·출구(QR) 관측·터널 판정 표식을 SVG 로 겹친다(`mapimage.py`, `render.py:211-295`). 웹캠 MJPEG 주소를 `<img>` 로 넣는다(`render.py:1170-1187`). `--robot-watch` 면 operations 의 robot_watch 노드가 쓴 로봇 직접 관측 파일을 읽어 로봇 카드에 붙인다(7.4.7).

하지 않는 일(다른 쪽 책임)
- 운영 장부·현장 상태 DB 에 쓰거나 파일을 만들지 않는다(`sources.py:1-3`). 쓰기는 operations `db_writer` 하나다.
- ROS 구독·발행·서비스 호출을 하지 않는다. 명령을 받을지 정하지 않는다(조정기). 명령을 다시 보내지 않는다(`commands.py:1-7`).
- 영상을 중계하지 않는다. 브라우저가 카메라 PC 의 MJPEG 서버에 직접 붙는다(`render.py:1171-1174`, 중계는 perception `mjpeg_relay`).
- 현장 정지 수단이 아니다. `STOP_ROBOT` 버튼은 늘 꺼 두고(`view.py:58-63`), 명령 칸·실패 페이지에 「현장 비상 정지 장치를 쓰세요」를 늘 둔다(`render.py:545-546`, `render.py:1210-1211`; 팀 AGENTS 7절).
- 지도를 만들거나 고치지 않는다(map_snapshot 몫). 모르는 값을 지어내지 않는다: 없는 값은 None 이고 화면에 「—」로 보인다(`view.py:1-5`).

### 2. 실행 단위

- 패키지 `rescue_console`, 실행 파일 `rescue-console` = `rescue_console.__main__:main`(`src/rescue_console/setup.py:22`). `python3 -m rescue_console` 로도 뜬다. Flask 는 선택 의존(`extras_require web`, `src/rescue_console/setup.py:14-15`)이고 없으면 기동이 `DEPENDENCY_MISSING` 으로 끝난다(`app.py:125-129`).
- launch 파일 없음. `operations.launch.py` 는 콘솔을 띄우지 않고 콘솔에 줄 인자만 문서·로그로 알린다(`launch/operations.launch.py:18-20`, `:70`, `:75`).
- ROS 노드 이름·namespace: 해당 없음(ROS import 없음, `app.py:8-20`).
- 계정 초기화는 별도 명령 `python3 -m rescue_console.accounts init`(`accounts.py:353-374`).
- 서버 스레드 모델: Flask `app.run` 을 디버그 끈 채 호스트·포트로 부른다(`__main__.py:93`) = Werkzeug 개발 서버. 요청마다 스레드 하나 [문서: Flask 1.0 이후 `app.run` 의 threaded 기본값 참; `throttle.py:4-6` 주석도 같은 전제]. 서버 쪽 타이머·백그라운드 스레드는 없고 모든 계산은 요청 처리 안에서 한다.
- 잠금: 로그인 제한 기록 `threading.Lock`(`throttle.py:33`), 실시간 지도 캐시 `threading.Lock`(`mapimage.py:231`). 계정 DB 는 호출마다 새 SQLite 연결 + `BEGIN IMMEDIATE`(`accounts.py:179-191`, `:316-328`). 현장 상태 DB 는 읽기 함수마다 새 읽기 전용 연결(`rescue_operations/site_state.py:572-592`).
- 브라우저 쪽 타이머: 조각 폴링 인터벌 1000 ms(`render.py:1096`, `render.py:14`), 카메라 감시 `setInterval` 주기 = max(100, min(500, 끊김 기준 ms ÷ 2)) ms(`render.py:1163`), 실패 페이지 `meta refresh` 5 s(`render.py:1205-1206`, `render.py:69`).

### 3. 파라미터

명령행 인자(ROS 파라미터 대신). 「현장 값」 칸의 출처가 없으면 [미확인] 이다.

| 이름 | 타입 | 단위 | 현장 값(출처) | 필수? 기본값 | 기동 시 검사 규칙 | 뜻 |
| --- | --- | --- | --- | --- | --- | --- |
| `LEDGER`(위치 인자) | 경로 | — | 생략 [측정 2026-10-13, `docs/evidence/field-2026-10-13-operations-host.md:25`, `:165`] | 선택, 없음. `--site-state` 가 없으면 필수 | 둘 다 없으면 `INVALID_INPUT`(`__main__.py:70-71`, `app.py:46-47`). 기동 때 열지 않고 요청마다 `read_reports` 로 연다(실패는 그 요청의 503) | 옛 `ReportLedger` 파일(`operations-ledger` 1.0). report_node 의 `rescue-report-store` 파일은 역할이 달라 열리지 않는다 |
| `--accounts` | 경로 | — | 저장소 밖 설정 폴더, 권한 600 [측정 2026-10-10, `docs/process/ops-db-tables.md:42`] | **필수** | 파일이 없으면 `STORAGE_ERROR`, `metadata` 가 다르면 `POLICY_MISMATCH`(`app.py:133`, `accounts.py:160-177`) | 계정 DB |
| `--site-state` | 경로 | — | launch 의 `db_path` 와 같은 파일(`launch/operations.launch.py:19`) | 선택, 없음. `--status` 와 배타 | argparse 배타 그룹(`__main__.py:27-29`)과 `app.py:42-43`. 기동 때 열지 않음 | 현장 상태 DB |
| `--status` | 경로 | — | 현장에서 쓰지 않음(E2 전용) | 선택. `--site-state` 와 배타, `LEDGER` 필요 | 요청마다 JSON 을 읽고 최상위 키가 `STATUS_KEYS` 의 부분집합이어야 함(`sources.py:34-54`) | 픽스처 JSON |
| `--camera ID=URL`(반복) | 문자열 | — | launch 로그의 값 `cam_fixed=http://<video_bind_host>:<video_port>/cam_fixed.mjpg`(`rescue_bringup/operations_launch.py:213-215`, `launch/operations.launch.py:70`) | 선택, 빈 목록 | 첫 `=` 앞 ID 가 비지 않고 URL 이 `http://` 또는 `https://` 로 시작(`app.py:103-108`), 아니면 `INVALID_INPUT` | 웹캠 MJPEG 주소 |
| `--camera-stale-s` | float | s | [미확인] | `--camera` 가 있으면 필수, 기본값 없음 | 없으면 `CONFIG_MISSING`, float 이 아니거나 유한·양수가 아니면 `INVALID_INPUT`(`render.py:1126-1134`, 호출 `app.py:124`). 카메라가 없으면 값을 보지 않음 | 새 프레임 없이 이 시간이 지나면 「영상 끊김」·재연결 |
| `--robot-name ID=NAME`(반복) | 문자열 | — | `robot_a=TB-6`, `robot_b=TB-5` [사용자 결정 2026-10-13, `README:47`] | 선택 | `=` 가 있고 앞뒤 공백을 뺀 ID·이름이 비지 않음(`app.py:85-90`), 같은 ID 두 번이면 `INVALID_INPUT`(`app.py:93-100`) | 화면 표시 이름. 없으면 robot_id |
| `--support-dispatch` | `AUTO` 또는 `OPERATOR` | — | `AUTO` [소유자 결정 2026-10-13, `rescue_coordination/ros_node.py:20`] — 조정기의 같은 이름 값과 같아야 함 | 선택, 없음(= 버튼) | argparse choices(`__main__.py:35-37`), 요청마다 `view.py:707-708` 가 다시 검사 | AUTO 면 출동 승인 버튼 대신 「자동 출동」 상태 |
| `--search-start` | `AUTO` 또는 `OPERATOR` | — | `AUTO` [소유자 결정 2026-10-13, OPS-136, `config/field/search_area.field.json:3`, `:44`] — 조정기의 `search_start` 와 같아야 함(`README:49`) | 선택, 없음(= 버튼) | argparse choices(`__main__.py:38-40`), 요청마다 `view.py:709` 가 다시 검사 | AUTO 면 수색 시작 버튼 대신 「{정찰 이름} 자동 출발」 상태(7.5) |
| `--map` | 경로 | — | `assets/maps/pre_quake/map.yaml`(`README:139`) | 선택 | 기동 때 한 번 `load_map`, 실패하면 기동 실패(`app.py:134`) | 고정 지도 |
| `--live-map` | 경로 | — | `<map_dir>/live_map.yaml`(`launch/operations.launch.py:75`) | 선택. `--map` 과 배타(`app.py:135-136`), `--live-map-stale-s` 와 짝(`__main__.py:75-76`) | 파일이 아직 없어도 됨 | 실시간 SLAM 지도 |
| `--live-map-stale-s` | float | s | [미확인]. map_snapshot 의 alive 주기 2 s 보다 커야 함(`rescue_operations/map_snapshot.py:24`) | `--live-map` 과 짝, 기본값 없음 | 유한·양수(`mapimage.py:226-227`), 아니면 `INVALID_INPUT` | 지도 낡음 기준 |
| `--robot-watch` | 경로 | — | 운영 launch 인자 `robot_watch_path`(노드 `rescue_robot_watch`, `launch/operations.launch.py:79-81`) | 선택, 없음. `--robot-watch-stale-s` 와 짝(`__main__.py:73-74`) | 기동 때 열지 않음. 파일이 아직 없어도 됨(요청마다 읽고, 없으면 경고) | 로봇 기본 토픽 직접 관측 JSON(I-5) |
| `--robot-watch-stale-s` | float | s | [미확인]. robot_watch 쓰기 주기 1 s(`rescue_operations/robot_watch.py:19`)보다 커야 함 | `--robot-watch` 와 짝, 기본값 없음 | argparse float 만. 유한·양수 검사가 **없다**(`--live-map-stale-s` 와 다름) [관찰, 코드 읽기] — 9절 | 이 시간보다 오래 받은 것이 없으면 「직접 관측 낡음」 |
| `--operations URL` | 문자열 | — | `http://127.0.0.1:<api_port>`(`launch/operations.launch.py:19`, `rescue_bringup/operations_launch.py:26`) | 선택. 없으면 명령 버튼 꺼짐 + 표준 오류 한 줄(`__main__.py:89-91`) | scheme 이 http·https, host 있음, query·fragment 없음, 아니면 `INVALID_INPUT`. 끝 `/` 제거(`commands.py:60-65`) | operations API 기준 주소 |
| `--secret-env` | 문자열 | — | 기본 그대로 | 기본 `RESCUE_CONSOLE_SECRET` | 그 환경 변수가 없거나 비면 `CONFIG_MISSING`(`__main__.py:66-68`), 16자 미만이면 `CONFIG_MISSING`(`app.py:130-131`) | 세션 서명 비밀이 든 환경 변수 이름 |
| `--host` | 문자열 | — | 운영 PC 의 접속 주소는 프로파일 값 [`README:54`] | 기본 `127.0.0.1` | 없음 | 바인드 주소 |
| `--port` | int | — | 8080 [측정 2026-10-10, `docs/process/ops-host-record.md:54`] | 기본 8080 | argparse int | 포트 |

환경 변수: `RESCUE_CONSOLE_SECRET`(세션 서명 비밀, 16자 이상), `RESCUE_CONSOLE_ADMIN_PASSWORD`(계정 초기화용, 없으면 프롬프트, `accounts.py:364-367`).

코드 상수(현장에서 바꾸지 않는 값)

| 이름 | 값 | 단위 | 근거 | 누가·언제 | 뜻 |
| --- | --- | --- | --- | --- | --- |
| `POLL_INTERVAL_MS` | 1000 | ms | `render.py:14` | [제안] 관제·화면 2026-10-08 | 조각 폴링 주기(KPI-04 3 s 예산 중 1 s) |
| `KPI04_BUDGET_S` | 3.0 | s | `render.py:16` | BRD v0.1 KPI-04 | 표시 지연 예산 |
| `FAILURE_RETRY_S` | 5 | s | `render.py:69` | — | 실패 페이지 자동 새로고침 |
| `SHORT_ID` | 8 | 글자 | `render.py:55` | — | UUID 를 화면에 줄여 보일 때 앞 글자 수(9자 이하면 그대로) |
| `BATTERY_LOW`, `BATTERY_WARN` | 25, 50 | % | `render.py:57` | [제안] 2026-10-10 | 배터리 막대 빨강·노랑 경계 |
| `GRID_M` | 1.0 | m | `render.py:59` | — | 지도 격자 간격 |
| `ROBOT_MARK_PX`, `HEADING_PX`, `TARGET_MARK_PX` | 2.6, 4.8, 2.0 | 지도 px | `render.py:20` | [제안] 2026-10-10 | 표식 크기 |
| `FRESH_MS`(JS) | 3000 | ms | `render.py:795` | — | 새 사건 행 노란 강조 시간 |
| `LOG_ROWS`(JS) | 6 | 행 | `render.py:926` | — | 명령 칸 아래 이 페이지 명령 기록 길이 |
| `STATUS_STALE_S` | 5.0 | s | `view.py:75` | [제안] 관제·화면 2026-10-10 | 로봇 상태 낡음(표시 전용, 준비 판정 아님 C-05) |
| `EVENT_ROWS` | 50 | 행 | `view.py:68` | [제안] 2026-10-08 | 사건 기록 표 최대 행 |
| `HISTORY_ROWS` | 30 | 행 | `view.py:70` | [제안] 작전 2026-10-12 | 명령·오류 기록 최대 행(읽기 한도로도 씀) |
| `COMMAND_TIMEOUT_S` | 5.0 | s | `commands.py:22` | [제안] 관제·화면 2026-10-12 | operations 답 대기(한 번) |
| `MAX_ID` | 128 | 글자 | `commands.py:41` | — | 폼 `target_id` 최대 길이 |
| `LEDGER_READ_LIMIT`, `SITE_EVENT_READ_LIMIT` | 1000, 1000 | 행 | `sources.py:17`, `sources.py:20` | 읽기 함수 상한 | 장부·현장 사건 한 번 읽기 행 수 |
| `LOGIN_MAX_FAILURES`, `LOGIN_WINDOW_S`, `LOGIN_LOCK_S`, `LOGIN_MAX_KEYS` | 5, 300, 60, 10000 | 회, s, s, 쌍 | `throttle.py:14-18` | [제안] 관제·화면 2026-10-10 | 로그인 반복 실패 잠금 |
| `_ITERATIONS` | 200000 | 회 | `accounts.py:39` | — | PBKDF2 반복 |
| `MIN_PASSWORD` | 8 | 글자 | `accounts.py:33` | — | 비밀번호 최소 길이(최대 없음) |
| 소금 길이 | 16 | byte | `accounts.py:71` | — | PBKDF2 소금 |
| `MAP_FRAME` | `map` | — | `mapimage.py:20` | Q-09 후보 | 이 frame 의 좌표만 지도에 그림 |
| `ALIVE_FILE` | `live_map.alive` | — | `mapimage.py:22` | map_snapshot 과 같은 이름 | 지도 발행자 살아 있음 표시 파일 |
| `EXIT_NAME` | `출구(QR)` | — | `mapimage.py:288` | — | 지도 위 출구 관측 표식의 ID·이름표 머리 |
| `PASSAGE_WORDS` | PASSABLE 통과 가능, BLOCKED 통과 불가, UNKNOWN 판정 불가 | — | `view.py:626` | OPS-196 | 터널 판정 낱말(지도·카메라 공통) |

### 4. 내부 상태

서버 쪽

| 이름 | 타입(자료구조) | 초기값 | 바뀌는 때(어느 함수) | 뜻 |
| --- | --- | --- | --- | --- |
| Flask 세션 쿠키 | 서명된 쿠키(키: `account_id`, `auth_version`, `csrf`, Flask 의 flash 목록) | 비어 있음 | `csrf_token`(`app.py:185-189`)이 처음 필요할 때 `csrf` 생성, 로그인 성공 때 비우고 `account_id`·`auth_version`·새 `csrf`(`app.py:276-279`), 로그아웃·무효 계정 때 비움(`app.py:181`, `app.py:286`) | 누가 로그인했나. 서버에 세션 저장소는 없다 |
| `throttle._entries` | dict: (소문자·앞뒤 공백 뺀 이름, 접속 주소) → {실패 시각 목록, 잠금 끝 시각} | 비어 있음 | `failed`·`succeeded`(`throttle.py:52-73`) | 프로세스 메모리에만, 재시작하면 사라짐 |
| `LiveMap.image`, `_key`, `_error` | MapImage 또는 None, (mtime_ns, size) 또는 None, 오류 문구 또는 None | 모두 None | `LiveMap.current`(`mapimage.py:233-270`) | 마지막으로 잘 읽은 실시간 지도와 그 파일 열쇠 |
| `map_image` | MapImage 또는 None | `--map` 이면 기동 때 읽은 지도 | 바뀌지 않음(`app.py:134`) | 고정 지도 |
| `_DUMMY` | 해시 문자열 하나를 담는 목록 | 비어 있음 | 없는 아이디 로그인 첫 시도 때(`accounts.py:56-60`) | 없는 아이디도 같은 비용을 치르게 하는 가짜 해시 |
| `accounts`, `operations`, `transport`, `cameras`, `robot_names`, `support_dispatch`, `search_start`, `robot_watch_path`, `robot_watch_stale_s` | 설정 값 | 기동 때 | 바뀌지 않음(`app.py:123-147`, `:161-166`) | — |

화면 모델(요청마다 새로 만들고 버린다, 서버 캐시 없음): 6.1 절의 `/api/view` 표.

브라우저 쪽(페이지 스크립트, `render.py:789-1097`, `render.py:1141-1167`)

| 이름 | 타입 | 초기값 | 바뀌는 때 | 뜻 |
| --- | --- | --- | --- | --- |
| `firstSeen` | Map: 사건 ref_id → 처음 그린 시각(ms) | 비어 있음 | `stamp`(`render.py:814-835`) | KPI-04 표시 시각 |
| `window.kpi04` | 배열 | 빈 배열 | `stamp` 가 새 행마다 하나 추가 | 지연 기록 |
| `loaded` | bool | 거짓 | 첫 `stamp` 끝에 참 | 이 뒤에 나타난 행만 실시간(`live`) |
| `lastOk` | 시각 ms | 페이지 로드 시각 | 폴링 성공 때 | 끊김 문구의 「n 전 값」 |
| `timer` | 인터벌 ID | 1000 ms 마다 `poll` 을 부르는 인터벌 | 401·재지정 때 해제 | 폴링 |
| `gates` | 객체: 명령 → {enabled, reason, args, choices} | 페이지의 `#gates` | 폴링마다 `readGates`(`render.py:932-935`) | 최신 관문 거울 |
| `armed` | null 또는 {kind, name, args(JSON 문자열), choices} | null | `arm`/`disarm` | 확인 칸이 열린 명령 |
| `sending` | bool | 거짓 | `send` 시작·끝 | 전송 중 |
| `screenLost` | bool | 거짓 | `pollStatus`(`render.py:875-888`) | 화면이 최신이 아님 |
| 카메라 칸마다 `last`, `retried` | 시각 ms | 로드 시각 | 프레임 load 때 `last`, 재연결 때 `retried` | 끊김 판단 |

상태 기계 1 — 로그인 세션(서버)

| 현재 | 사건/조건 | 다음 | 부수 효과 |
| --- | --- | --- | --- |
| 익명 | `GET /login` | 익명(+csrf) | 세션에 `csrf` 생성 |
| 익명 | `POST /login`, CSRF 맞음, 잠금 아님, 비밀번호 맞음, 계정 ACTIVE | 로그인됨 | 실패 기록 지움, 세션 비우고 `account_id`·`auth_version`·새 `csrf` 저장, 302 `/` |
| 익명 | `POST /login` 실패(잠금 또는 틀림) | 익명 | 틀림이면 실패 기록 +1, flash, 302 `/login` |
| 로그인됨 | 요청 때 계정 행이 없거나 `status` ≠ ACTIVE 이거나 `auth_version` 이 다름 | 익명 | 세션 비움, 데이터 경로 401·페이지 302 `/login` |
| 로그인됨 | `POST /logout`(CSRF 맞음) | 익명 | 세션 비움, 302 `/login` |

상태 기계 2 — 명령 칸(브라우저, `render.py:918-1089`)

| 현재 | 사건/조건 | 다음 | 부수 효과 |
| --- | --- | --- | --- |
| 대기 | 켜진 버튼 클릭(ADMIN·주소 있음·관문 켬·끊김 아님·전송 중 아님) | 확인 중 | 확인 칸 표시(무엇·대상 고르기·보낼 값), 모든 명령 버튼 끔, 「보내기」에 초점 |
| 확인 중 | 「취소」 또는 Escape | 대기 | 확인 칸 닫음 |
| 확인 중 | 폴링 뒤 그 명령 관문이 꺼지거나 `args` 가 바뀌거나 고른 대상이 선택지에서 빠짐, 또는 갱신 끊김 | 대기 | 기록에 info 「상태가 바뀌어 {명령} 확인을 취소했습니다. 다시 누르세요.」 |
| 확인 중 | 「보내기」 | 보내는 중 | 「보내기」 글자 「보내는 중…」, 보내기·취소 끔, `POST commands` 한 번 |
| 보내는 중 | 답·실패 도착 | 대기 | 기록에 한 줄(accepted·rejected·failed), 확인 칸 닫음. 보내는 중에는 관문이 바뀌어도 취소하지 않는다(`render.py:966`) |

상태 기계 3 — 갱신 표시(브라우저): 「연결 중」(`#poll.wait`, 서버가 그린 초기값) → 스크립트 시작 즉시 「정상」(`render.py:1095`) → 폴링 성공이면 「정상: 실시간 갱신 HH:MM:SS」, 실패면 「끊김: 갱신 끊김: {이유}, {n} 전 값을 보는 중」 → 401 또는 다른 주소로 재지정되면 「종료」(인터벌 해제 후 이동). 정상·끊김은 폴링마다 오간다.

상태 기계 4 — 실시간 지도(`LiveMap.current`, 서버, 요청마다 계산): MISSING(파일 없음·한 번도 못 읽음) / ERROR(파일 stat 실패이거나 최신 파일 읽기 실패, 마지막 좋은 지도 유지) / STALE(지도 수정 시각과 alive 파일 시각 중 늦은 것이 `stale_s` 보다 오래) / OK. 전이는 파일 상태로만 정해지고 기록하지 않는다.

상태 기계 5 — 카메라 칸(브라우저): 정상 ↔ 끊김. 끊김 진입: `onerror` 즉시, 또는 마지막 프레임 load 뒤 `stale` ms 이상. 해제: 다음 프레임 load. 끊김 동안 `stale` ms 마다 한 번 재연결.

### 5. 입력

#### 5.1 HTTP 경로(브라우저 → 콘솔)

모든 응답에 보안 헤더 4개(`X-Content-Type-Options: nosniff`, `X-Frame-Options: DENY`, `Referrer-Policy: no-referrer`, `Content-Security-Policy: frame-ancestors 'none'; base-uri 'none'; form-action 'self'; object-src 'none'`)를 붙이고, 응답이 스스로 정하지 않았으면 `Cache-Control: no-store` 를 붙인다(`app.py:28-33`, `app.py:149-159`). 로그인이 없을 때 경로가 `/api/` 로 시작하거나 `/fragment`·`/map.png`·`/commands` 중 하나면 401 JSON `{"error": "login required"}`, 그 밖은 302 `/login`(`app.py:24`, `app.py:198-214`). 등록 안 된 경로는 Flask 기본 404, 틀린 메서드는 405.

| 메서드·경로 | 로그인 | 역할 | 입력 | 성공 응답 | 실패 응답 | 근거 |
| --- | --- | --- | --- | --- | --- | --- |
| `GET /login` | 불필요 | — | 없음 | 로그인돼 있으면 302 `/`. 아니면 200 HTML 로그인 페이지(flash 표시, 숨은 csrf) | — | `app.py:246-251` |
| `POST /login` | 불필요 | — | 폼 `csrf`, `username`, `password` | 302 `/` | CSRF 틀림 400(Flask 기본 HTML). 잠금이면 flash danger 「로그인을 너무 많이 틀렸습니다. {n}초 뒤에 다시 하세요.」+ 302 `/login`(비밀번호 확인 전). 틀림이면 flash danger 「아이디 또는 비밀번호가 틀립니다.」+ 302 `/login` | `app.py:253-280` |
| `POST /logout` | 불필요 | — | 폼 `csrf` | 302 `/login`(세션 비움) | CSRF 틀림 400. `GET /logout` 은 405 | `app.py:282-287` |
| `GET /` | 필요(302) | 모두 | 없음 | 200 HTML 전체 페이지 | 입력 읽기 실패 503 HTML 실패 페이지 | `app.py:290-296`, `app.py:238-239` |
| `GET /fragment` | 필요(401) | 모두 | 없음 | 200 `text/html` 조각 | 503 `text/plain` 한 줄 `{코드}: {확인할 것} ({원문})` | `app.py:298-302`, `app.py:243` |
| `GET /api/view` | 필요(401) | 모두 | 없음 | 200 JSON 화면 모델 | 503 JSON `{error, message, hint}` | `app.py:304-308`, `app.py:240-242` |
| `GET /map.png` | 필요(401) | 모두 | 쿼리 `v` 는 무시(캐시 열쇠일 뿐) | 200 `image/png`, `Cache-Control: private, max-age=3600` | 지도 없음 404 | `app.py:310-321` |
| `POST /commands` | 필요(401 JSON) | ADMIN | 폼 `csrf`, `kind`, 명령별 필드(5.2) | 200 JSON `sent: true` | 400·403·503·502 JSON `sent: false`(7.6 절) | `app.py:324-356` |
| `GET /admin/accounts` | 필요(302) | ADMIN(아니면 403) | 없음 | 200 HTML 계정 관리 | — | `app.py:365-368`, `app.py:216-227` |
| `POST /admin/accounts` | 필요(302) | ADMIN(403) | 폼 `csrf`, `username`, `password`, `role`(없으면 CUSTOMER), `display_name`, `email` | 303 `/admin/accounts` + flash success 「계정 {아이디} 추가」 | CSRF 400. 검사 실패는 flash danger 「추가 실패: {코드}: {문구}」+ 303 | `app.py:370-388` |
| `POST /admin/accounts/{username}` | 필요(302) | ADMIN(403) | 폼 `csrf`, `_action`(`delete` 또는 `update`), update 때 `display_name`, `email`, `role`, `status`, `password`(빈 값 = 안 바꿈) | 303 + flash 「계정 {아이디} 삭제」(info) 또는 「계정 {아이디} 저장」(success) | CSRF 400. 다른 `_action` 400. 검사 실패 flash danger 「변경 실패: …」+ 303 | `app.py:390-415` |

#### 5.2 폼 필드 검사

| 폼 | 필드 | 타입 | 검사 |
| --- | --- | --- | --- |
| 로그인 | `username` | 문자열 | 계정 DB 의 `username` 과 정확히 같아야 함(대소문자 구분). 잠금 열쇠는 소문자·공백 제거본(`throttle.py:40-42`) |
| 로그인 | `password` | 문자열 | PBKDF2 비교(7.7) |
| 계정 추가·수정 | `username` | 문자열 | 정규식 소문자로 시작 + 소문자·숫자·`_` `.` `-` 2~31자, 전체 3~32자(`accounts.py:30`, `:93-96`) |
| 계정 추가·수정 | `password` | 문자열 | 8자 이상(`accounts.py:63-70`). 수정 때 빈 값은 안 바꿈(`app.py:409`) |
| 계정 추가·수정 | `role` | 문자열 | ADMIN 또는 CUSTOMER(`accounts.py:28`) |
| 계정 수정 | `status` | 문자열 | ACTIVE 또는 INACTIVE(`accounts.py:29`) |
| 계정 추가·수정 | `display_name` | 문자열 | 128자 이하, 줄바꿈 없음. 빈 값이면 아이디로 채움(`accounts.py:99-101`, `:234`, `:273-274`) |
| 계정 추가·수정 | `email` | 문자열 | 128자 이하, 줄바꿈 없음, 빈 값 또는 `이름@호스트.최상위` 꼴(`accounts.py:32`, `:104-107`) |
| 명령 | `kind` | 문자열 | `COMMAND_FIELDS` 의 다섯 종류 중 하나(`commands.py:26-32`, `:97-99`) |
| 명령 | 그 밖 키 | — | `csrf`·`kind`·그 종류의 필드 말고 다른 키가 하나라도 있으면 거절(`commands.py:100-103`) |
| 명령 | `target_id` | 문자열 | 1~128자, 출력 가능 문자, 공백 없음(`commands.py:77-80`). UUID 여부는 operations 가 본다 |
| 명령 | `target_revision`, `route_revision`, `segment_index` | 10진 숫자 문자열 | ASCII 숫자만(음수·소수 없음) → 정수(`commands.py:83-88`). 상한은 operations 가 본다 |

#### 5.3 파일 입력

| 입력 | 여는 방식 | 이 웹이 쓰는 방법·검사 | 근거 |
| --- | --- | --- | --- |
| 현장 상태 DB(`--site-state`) | 요청마다 읽기 함수 9개를 이 순서로 따로 연다: 배치 로봇 목록, 로봇 상태, 현장 사건(최대 1000, 오래된 순), 대상 상태, 임무 상태, 운영자 명령(최근 30), 오류 기록(최근 30), 출구(QR) 관측(`read_exit_locations(limit=1)`, 가장 최근 관측 1행), 터널 판정(`read_overhead_passages`, 로봇별 목록 + `current_map`). 각각 `mode=ro` + `query_only`, 한 트랜잭션이 아니다. 하나가 실패하면 뒤의 읽기는 하지 않는다 | 읽기 실패(파일 없음·잠김·역할 다름·schema 1.4 아님)는 모두 `STORAGE_ERROR` 「site-state read failed: …」로 바꾼다. 쓰는 열은 「웹이 기대는 인터페이스」 I-2 | `sources.py:75-99`, 출구 `sources.py:93`, 터널 `sources.py:94` |
| 옛 보고 장부(`LEDGER`) | 요청마다 `read_reports(limit=1000)` | 실패는 `STORAGE_ERROR` 「ledger read failed: …」. 쓰는 필드는 I-4 | `sources.py:26-31` |
| 픽스처(`--status`) | 요청마다 UTF-8 JSON 읽기 | 읽기·JSON 실패 `INVALID_INPUT` 「status fixture unreadable: …」, 최상위가 객체가 아니거나 키가 `allowed_robots`·`robot_status`·`site_events`·`target_states`·`mission_state`·`operator_commands`·`error_log`·`exit_locations`·`overhead_passages` 밖이면 `INVALID_INPUT`. 빠진 키는 채우지 않는다: `robot_status`·`site_events`·`target_states`·`mission_state` 는 「{키} 입력 없음」 경고, `allowed_robots` 는 7.4 3번의 대체 목록, 명령·오류는 기록 칸의 「입력 없음」, `exit_locations` 는 경고 없이 출구 없음, `overhead_passages` 는 경고 없이 터널 없음(7.4) | `sources.py:22-23`, `:34-54` |
| 계정 DB(`--accounts`) | 기동 때 존재·메타 확인, 이후 호출마다 새 연결 | 7.7 | `accounts.py:160-191` |
| 고정 지도 YAML + PGM(`--map`) | 기동 때 한 번 | 7.9 `load_map` | `mapimage.py:156-207` |
| 로봇 직접 관측 JSON(`--robot-watch`) | 요청마다 UTF-8 읽기(쓰는 쪽이 임시 파일 + rename 이라 반쯤 쓴 파일은 보지 않음) | 없거나 못 읽거나 모양이 다르면 요청을 실패시키지 않고 (None, 이유)를 돌려 경고로 보인다: 「로봇 직접 관측 파일 없음」, 「로봇 직접 관측 파일 읽기 실패: {사유}」, 「… 읽기 실패: 모양이 다름」(최상위 객체, `robots` 객체, `written_at_ns` int 가 아니면). 처리는 7.4.7, 파일 계약은 I-5 | `sources.py:57-72` |
| 실시간 지도 YAML + PGM + `live_map.alive`(`--live-map`) | 요청마다 stat, 수정 시각·크기가 바뀌었을 때만 다시 읽음 | 7.9 `LiveMap.current`, 파일 계약은 I-3 | `mapimage.py:210-270` |

#### 5.4 operations 의 답(명령 전송 뒤)

`POST {operations}/api/operator-commands` 의 HTTP 상태와 본문(`commands.py:143-165`). 상태 코드와 상관없이 본문을 읽는다.

| 필드 | 타입 | 검사 |
| --- | --- | --- |
| `accepted` | bool | 반드시 bool(정수 1·0 은 안 됨) |
| `request_id`, `reason`, `request_state`, `created_task_id` | 문자열 또는 null | 없거나 null 이거나 빈 문자열이 아닌 문자열 |
| 관계 | — | `accepted` 가 거짓이면 `reason` 이 있어야 함. `accepted` 가 참인데 HTTP 상태가 2xx 가 아니면 형식 위반 |

#### 5.5 웹캠 MJPEG(브라우저가 직접 받음)

`<img src={URL}>` 의 `load` 사건(프레임마다 1회로 관찰됨 [측정 2026-10-13, Firefox, `render.py:1137-1140`])과 `error` 사건만 쓴다. Chrome 의 프레임별 load 동작은 [미확인] (`README:51`).

### 6. 출력

언제 내는가: 서버는 스스로 아무것도 내지 않는다. 모든 HTTP 응답은 요청이 올 때 한 번이고(브라우저가 1초마다 `/fragment` 를 부른다), operations 로 가는 POST 는 ADMIN 이 「보내기」를 누를 때 한 번, 계정 DB 쓰기는 ADMIN 의 계정 폼 제출 때 한 번이다. 재시도·주기 발행은 없다.

#### 6.1 `/api/view` JSON = 화면 모델(`view.py:770-790`)

| 필드 | 타입 | 값을 만드는 규칙 |
| --- | --- | --- |
| `built_at_ns` | int | 서버 현재 시각 ns(요청 시각). 모든 나이 계산의 기준 |
| `evidence_policy` | 문자열 또는 null | 장부의 `evidence_policy`, 장부가 없으면 null |
| `mission` | 객체 | 7.4.4 |
| `robots` | 배열 | 7.4.1. 행 키: `robot_id`, `name`, `has_status`, `role`, `pose`{x,y,yaw,frame_id}, `motion_state`, `readiness`, `battery_percent`, `battery_age_s`, `active_task_id`, `stamp_ns`, `age_s`, `received_age_s`, `stale`, `watch`(직접 관측이 없으면 null, 있으면 {namespace, age_s, stale, battery_percent, docked, pose}, 7.4.7) |
| `targets` | 배열 | 7.4.2. 행 키: `target_id`, `target_revision`, `persisted_report_id`, `stage`, `stage_label`, `rule`, `position`{x,y,frame_id,map_revision,source}, `detection_confirmed`, `approach_result`, `rescue_state`, `rescue_reasons`, `support_state`, `support_robot_id`, `report_id`, `report_count`, `receipt_status`, `observed_at_ns`, `disappeared_candidate`{event_id,observed_at_ns,camera_id}, `warnings` |
| `stage_counts` | 객체 | 네 단계(UNCONFIRMED·NOT_RESCUED·SUPPORTING·RESCUED)마다 대상 수, 0 포함 |
| `exit_count` | int 또는 null | 현장 사건 읽기의 `exit_pass_count` 그대로(다시 세지 않음). 화면에는 그리지 않음(`view.py:742-746`) |
| `exit` | 객체 또는 null | 가장 최근 출구(QR) 관측 하나: `location_id`, `x`, `y`, `frame_id`(= DB 의 `mission_frame`), `observed_at_ns`, `map_revision`. `exit_locations` 절이 없거나 비면 null(경고 없음). 조정기가 등록한 출구가 아니라 관측이다(`view.py:650-657`) |
| `passages` | 배열 | 7.4.8. 행 키: `id`, `robot_id`, `state`, `label`, `x`, `y`, `frame_id`, `map_revision`, `last_seen_ns`, `current`. 표가 없으면 빈 배열 |
| `events` | 배열(최대 50) | 7.4.3 |
| `event_total` | int | 자르기 전 사건 수 |
| `map` | 객체 | 지도 없으면 `{available: false, note: "지도 스냅샷 미연결 — 로봇·지도 제공 뒤 연결"}`, 있으면 7.9 `map_overlay` 결과(지도 meta + `robots`, `targets`, `exits`, `passages`, `off_map`, `frame_mismatch`, `no_pose`, `note`), 실시간 지도면 `live`{state, received_at_ns, age_s, message} 추가 |
| `commands` | 배열(9) | 7.5. 행 키: `name`, `label`, `group`, `enabled`, `reason`, `args`, `choices`, AUTO 행(출동 승인·수색 시작)만 `auto: true` |
| `support_dispatch` | 문자열 또는 null | 인자 그대로 |
| `search_start` | 문자열 또는 null | 인자 그대로 |
| `history` | 객체 또는 null | 7.4.5 |
| `warnings` | 문자열 배열 | 각 절에서 붙인 「확인할 것」 |

#### 6.2 `POST /commands` 응답 JSON(`app.py:334-356`, `commands.py:168-175`)

| 경우 | HTTP | 본문 |
| --- | --- | --- |
| 보냄 + 답 있음 | 200 | `sent: true`, `command_id`, `kind`, `accepted`, `request_id`, `reason`, `request_state`, `created_task_id`, `reason_label`(= `reply` 낱말 표로 바꾼 `reason`, 모르는 코드는 그대로, null 은 null) |
| 보냈지만 실패 | 502 | `sent: false`, `command_id`, `kind`, `error`(TIMEOUT·UNREACHABLE·SEND_FAILED·BAD_RESPONSE·HTTP_ERROR), `message`, `delivery`(NOT_SENT 또는 UNKNOWN) |
| CSRF 틀림 | 400 | `sent: false`, `error: "CSRF"`, `message: "csrf token mismatch — reload the page"` |
| CUSTOMER | 403 | `sent: false`, `error: "NOT_AUTHORIZED"`, `message: "보기 전용 계정 — 명령은 ADMIN 만 보냅니다"` |
| ADMIN, operations 주소 없음 | 503 | `sent: false`, `error: "NOT_CONNECTED"`, `message: "operations 주소 없음 — 실행할 때 --operations 로 주면 켜집니다"` |
| 폼 검사 실패 | 400 | `sent: false`, `error: "INVALID_INPUT"`, `message`(7.6) |

#### 6.3 operations 로 보내는 요청(명령 한 건당 정확히 한 번)

- `POST {--operations}/api/operator-commands`, 헤더 `Content-Type: application/json`, `Accept: application/json`, 대기 5.0 s(`commands.py:115-118`, `commands.py:19-22`). 본문은 아래 키만:

| 필드 | 타입 | 값 |
| --- | --- | --- |
| `command_id` | 문자열 | 누를 때마다 새 UUID4(`app.py:348`). 같은 명령을 다시 눌러도 새 ID |
| `kind` | 문자열 | 폼 `kind` |
| `operator_id` | 문자열 | 로그인 계정의 `username`(`app.py:348`). 폼에서 받지 않음 |
| `target_id` | 문자열 | DISPATCH_SUPPORT·CONTINUE_SEGMENT·REQUEST_RECHECK 만. 운영자가 화면에서 본 값 |
| `target_revision` | 정수 | DISPATCH_SUPPORT·CONTINUE_SEGMENT 만 |
| `route_revision`, `segment_index` | 정수 | CONTINUE_SEGMENT 만 |

#### 6.4 계정 DB 쓰기

계정 추가·수정·삭제와 `init`. 표 정의는 7.7. 다른 DB 에는 쓰지 않는다.

#### 6.5 로그

| 언제 | 어디 | 문구 |
| --- | --- | --- |
| 기동 실패 | 표준 오류, 종료코드 2 | `CONSOLE FAILED: {코드}: {문구}`(`__main__.py:86-88`) |
| `--operations` 없음 | 표준 오류 | `operations 주소 없음: 명령 비활성 (--operations URL 로 켬)`(`__main__.py:89-91`) |
| 명령을 보내기 전에 거절 | Flask logger warning | `command refused before sending: {코드} {문구} ({아이디})`(`app.py:335`) |
| 명령 보냄 | 성공이면 info, 실패면 warning | `operator command {kind} {command_id} by {아이디} -> {나머지 결과}`(`app.py:352-354`) |
| 계정 init | 표준 출력 또는 표준 오류(2) | `created {파일} with ADMIN {아이디}` 또는 `ACCOUNTS FAILED: …`(`accounts.py:368-374`) |

기동 때 git SHA 를 남기는 코드는 없다(검색: `grep -nwi "git" src/rescue_console/rescue_console/*.py`, `grep -n "rev-parse\|subprocess"` 같은 파일들 — 둘 다 결과 없음. 단어 경계 없는 `grep -n "git"` 은 `commands.py` 의 「digit」 주석에 걸린다).

#### 6.6 화면 배치(칸 단위)

페이지는 외부 자원을 하나도 읽지 않는다(CSS·JS 모두 인라인, `render.py:1100-1104`, `render.py:1223-1231`). HTML 뼈대(`render.py:1214-1231`, `render.py:506-509`):

| 순서 | 요소 | 안에 든 것 | 갱신 |
| --- | --- | --- | --- |
| 1 | `main.console` | 아래 3개 | — |
| 1.1 | `section#topbar` | `p#poll`(처음 「연결 중」), 쓰이지 않는 `div#poll-error`(숨김), `div.kpi`(`p#kpi` + 「지연 기록 저장」 버튼), `nav.who`(표시 이름(역할), ADMIN 이면 「계정 관리」 링크, 로그아웃 폼) | 페이지 고정, 스크립트가 글자만 바꿈 |
| 1.2 | `aside#side` | `section#cameras`(웹캠) 다음 `section#commands`(명령 칸) | 페이지 고정(폴링이 다시 그리지 않음) |
| 1.3 | `div#live`(`display: contents`) | 조각: `section#mission`, `section#warnings`, `section#map`, `div#middle`(`section#robots`, `section#targets`), `section#events`, `section#history`, 숨은 `div#gates`(관문 JSON) | 1초마다 통째로 교체 |
| 2 | `script` | 폴링·KPI·명령 칸 스크립트, 카메라가 있으면 카메라 감시 스크립트 | — |

격자(`render.py:569-580`): 열 = 가변(최소 0) · 400px · 470px, 행 = 내용 높이 · 내용 높이 · 남는 높이 · 최대 230px(내용 맞춤).

| 칸 | grid-area | 넓은 화면 자리 | 내용 |
| --- | --- | --- | --- |
| 임무 띠 `#mission` | band | 1행, 왼쪽 두 열 | 모드 낱말(큰 글씨), 사실 줄(트리거·경과·진동 감지 건수·단계·구간·사유), 모드 설명, 오른쪽 끝 「서버 집계 HH:MM:SS.mmm」. 재난이면 빨간 바탕 |
| 위쪽 막대 `#topbar` | top | 1행, 오른쪽 열 | 갱신 상태, KPI-04 요약·저장, 사용자 |
| 확인할 것 `#warnings` | warn | 2행, 왼쪽 두 열 | 경고가 없으면 숨김. 「확인할 것 {n}」 + 경고를 「／」로 이어 한 줄 |
| 옆 칸 `#side` | side | 2~4행, 오른쪽 열 | 위 웹캠, 아래 명령 칸 |
| 지도 `#map` | map | 3행, 왼쪽 열 | 범례, SVG 지도(요구조자·출구(QR) 관측·로봇 표식), 지도 설명 줄, 「지도에 그리지 못한 위치」 목록 |
| 가운데 `#middle` | middle | 3행, 가운데 열 | 위 로봇 카드(넘치면 그 안에서 스크롤), 아래 요구조자(최소 240px) |
| 사건 기록 `#events` | log | 4행, 왼쪽 열 | 사건 표(안에서 스크롤) |
| 명령·오류 기록 `#history` | hist | 4행, 가운데 열 | 기록 표(안에서 스크롤) |

- 창 폭 1400px 이상이고 높이 820px 이상이면 격자가 창 높이에 고정되고 페이지는 스크롤되지 않는다(`render.py:576`). 폭이 1399px 이하이면 한 열로 쌓이고 순서는 위쪽 막대 → 임무 띠 → 확인할 것 → 지도 → 옆 칸 → 가운데 → 사건 기록 → 명령·오류 기록(`render.py:577-580`). 폭 1400px 이상·높이 820px 미만은 세 열 그대로이고 페이지가 세로로 스크롤된다.
- 색은 의미에만 쓴다: 신호 빨강 = 재난·정지·실패, 구조 주황 = 미구조, 안내 파랑 = 지원 중·SUPPORT 로봇·승인 버튼·자동 상태 줄, 안전 초록 = 구출 완료·출구(QR) 관측, 회색 점선 = 미확정, 경고 노랑 = 낡음·끊김·확인할 것(`render.py:550-555`, `:614`, `:641`, `README:108-115`).

칸별 내용 규칙은 7.10 절.

### 7. 처리 명세

#### 7.1 `main` (`__main__.py:17-94`) — 명령행 → 서버 기동

1. 3절 표대로 인자를 읽는다. argparse 오류는 argparse 기본 동작(사용법 출력, 종료코드 2).
2. `--secret-env` 가 가리키는 환경 변수를 읽는다. 없거나 빈 문자열이면 `CONFIG_MISSING` 「set {변수} to a random string of 16+ characters」.
3. `LEDGER` 와 `--site-state` 가 둘 다 없으면 `INVALID_INPUT`.
4. `--live-map` 과 `--live-map-stale-s` 중 하나만 있으면 `INVALID_INPUT`.
5. `--camera` 마다 `parse_camera`, `--robot-name` 마다 `parse_robot_name` 뒤 중복 검사.
6. `create_app`(7.2).
7. 2~6 에서 `ConsoleError` 면 표준 오류에 `CONSOLE FAILED: …` 를 쓰고 2 를 돌려준다(서버를 띄우지 않음).
8. `--operations` 가 없으면 표준 오류에 안내 한 줄.
9. Flask 개발 서버를 `--host`·`--port` 로 디버그 끈 채 띄운다. 서버가 끝나면 0.

#### 7.2 `create_app` (`app.py:111-417`) — 설정 검사와 라우트 등록

기동 검사 순서(앞에서 실패하면 뒤는 하지 않음):
1. 카메라 끊김 시간 검사(`camera_stale_ms`) — Flask import 전에.
2. Flask import. 실패면 `DEPENDENCY_MISSING` 「Flask is required for the web console: python3 -m pip install flask」.
3. 비밀이 문자열이고 16자 이상. 아니면 `CONFIG_MISSING`.
4. 입력 원천: `--site-state` 와 `--status` 동시 → `INVALID_INPUT`, 장부·현장 상태 둘 다 없음 → `INVALID_INPUT`(`app.py:36-47`).
5. `AccountStore.open`(7.7).
6. `--map` 이 있으면 `load_map`(실패가 곧 기동 실패).
7. `--map` 과 `--live-map` 동시 → `INVALID_INPUT` 「--map and --live-map are exclusive: one map panel」.
8. `--live-map` 이 있으면 `LiveMap` 생성(파일은 아직 없어도 됨).
9. `--operations` 가 있으면 `parse_operations_url`.
10. Flask 앱: 비밀 키, 세션 쿠키 `HttpOnly`·`SameSite=Lax`(`app.py:141-146`). `Secure` 는 켜지 않는다(http 운영). 세션은 브라우저 세션 쿠키다 [문서: Flask 기본값, `permanent` 를 쓰지 않음].
11. 장부·현장 상태·픽스처 파일은 여기서 열지 않는다. 처음 요청에서 실패가 드러난다.

인증 규칙:
- `user()`(`app.py:169-183`): 세션에 `account_id` 가 없으면 익명. 있으면 그 ID 로 계정 행을 찾고, 행이 없거나 `status` ≠ ACTIVE 이거나 행의 `auth_version` ≠ 세션 `auth_version` 이면 세션을 비우고 익명. 그래서 계정 삭제 뒤 같은 이름으로 다시 만들어도(새 `account_id`), 비밀번호·역할·상태가 바뀌어도(`auth_version` +1) 이전 쿠키는 서버에서 거절된다.
- `login_required`: 익명이면 5.1 의 401/302 규칙. `admin_required`: 로그인 + 역할이 ADMIN 이 아니면 403 「admin only」.
- CSRF: 세션당 토큰 하나(`secrets.token_urlsafe` 32 byte 난수, 처음 필요할 때 생성, 로그인 성공 때 새로 생성). 검사는 폼 `csrf` 와 세션 `csrf` 를 상수 시간 비교, 세션에 없으면 비교 대상은 `-` 라 빈 폼은 늘 틀림(`app.py:191-196`). `/login`·`/logout`·`/admin/*` 는 400 으로 끊고 `/commands` 는 JSON 400 으로 답한다.
- `ConsoleError` 처리기(`app.py:229-243`): 요청 경로가 `/` 이면 503 HTML 실패 페이지, `/api/` 로 시작하면 503 JSON(`hint` 는 코드별 확인할 것), 그 밖(`/fragment`, `/map.png` 등)은 503 `text/plain` 한 줄. 어느 쪽도 빈 정상 화면처럼 보이지 않는다.

#### 7.3 `current_view` (`app.py:50-82`) + `load_site_state` (`sources.py:75-99`) — 한 번의 폴링 입력

1. 입력 원천 규칙을 다시 확인(`_check_sources`).
2. 상태 = `--site-state` 가 있으면 `load_site_state`, 아니면 `load_status`(픽스처, 경로가 없으면 빈 객체).
3. 장부 경로가 있으면 `load_ledger`, 없으면 None. `--robot-watch` 면 `load_robot_watch` 의 (data, problem) 에 `stale_s` 를 붙인 객체, 없으면 None(`app.py:63-64`).
4. `build_view` 에 장부·상태·서버 현재 ns(`built_at_ns`)·표시 이름·`--support-dispatch`·`--search-start` 값과 직접 관측 객체를 넘겨 화면 모델을 만든다(7.4, `app.py:65-66`).
5. `--live-map` 이면 `LiveMap.current` 로 (지도, live 상태)를 얻는다.
6. 지도가 있으면 `map` 을 `map_overlay`(지도 meta, 로봇 행, 대상 행, 화면 모델 `exit`, 화면 모델 `passages`) 결과로 바꾸고(`app.py:71`), 다른 frame 좌표마다 경고 「지도에 안 그림: {robot·target·exit·passage} {ID} frame '{frame}' ≠ 지도 frame」, 지도 밖 좌표마다 「지도 밖 좌표: {kind} {ID} ({x}, {y})」(소수 둘째 자리)를 붙인다. 출구의 ID 는 「출구(QR)」다.
7. live 가 있으면 `map.live` 로 붙이고 `message` 가 비지 않았으면 경고에도 붙인다.
- `load_site_state` 는 9개 읽기를 차례로 따로 열어 읽는다(5.3). 한 번의 폴링에서 표끼리 한 행씩 어긋날 수 있다(다음 폴링에서 맞춰짐, `sources.py:78-82`). 어느 하나라도 `OperationsError` 면 전체가 `STORAGE_ERROR` 다. 역할·스키마가 다른 DB(`POLICY_MISMATCH`)도 화면에는 `STORAGE_ERROR` 로 보인다.
- 한 폴링의 비용: 현장 상태 DB 연결 9번 + (장부 1번) + (직접 관측 파일 읽기 1번) + 계정 DB 연결 1번(`user()`) + 실시간 지도 stat 1~2번.

#### 7.4 `build_view` (`view.py:692-790`) — 입력 → 화면 모델

1. `built_at_ns` 가 0 이상 정수가 아니면 `INVALID_INPUT`. `support_dispatch`·`search_start` 가 각각 None·AUTO·OPERATOR 밖이면 `INVALID_INPUT`(`view.py:705-709`).
2. 각 절 모양 검사(`_section`, `view.py:122-133`): 절이 있으면 객체여야 하고 `schema_version` 이 문자열, 목록 키가 배열이어야 한다. 아니면 `INVALID_INPUT` 「{키} must be a read API result」. 없는 절은 None.
3. 배치 로봇 목록 정하기(`view.py:713-723`):
   - 장부 없음 + `allowed_robots` 절 있음 → 그 목록(정렬된 값) + 경고 「보고 장부 입력 없음 — 구조 보고 행·건수를 표시하지 않음」.
   - 장부 없음 + 목록 절 없음(픽스처) → 상태가 들어온 로봇 ID 정렬 목록 + 경고 「보고 장부 입력 없음 — 구조 보고 행·건수를 표시하지 않음, 로봇 목록은 상태가 들어온 로봇만」.
   - 장부 있음 → **장부의** `allowed_robot_ids`(현장 상태 DB 의 목록이 아님).
4. `robot_status`·`site_events`·`target_states`·`mission_state` 절이 없으면 각각 「{키} 입력 없음」 경고. 임무 절은 있는데 행이 None 인 것은 경고가 아니다. `exit_locations`·`overhead_passages` 절이 없는 것도 경고가 아니다(17번).
5. 대상 상태를 `target_id` 로 묶는다. 같은 ID 두 번이면 `INVALID_INPUT` 「duplicate target state {ID}」.
6. 현장 사건을 계약 `SiteEvent` 로 다시 만든다. 계약 위반이면 `INVALID_INPUT` 「site event: …」(`view.py:285-297`).
7. 소실 후보: 종류가 `TARGET_DISAPPEARED_CANDIDATE` 인 사건 중 대상마다 (`observed_at_ns`, `event_id`)가 가장 큰 것 하나(`view.py:161-171`).
8. 대상 행(7.4.2). 소실 후보가 가리키는 대상이 목록에 없으면 경고 「소실 후보의 대상이 목록에 없음: {ID}」.
9. `exit_count` = 현장 사건 절의 `exit_pass_count`(0 이상 정수가 아니면 `INVALID_INPUT`), 절이 없으면 None.
10. `stage_counts`, 사건 행(7.4.3), 임무(7.4.4), 로봇 행(7.4.1), 그 로봇 행에 직접 관측 붙이기(7.4.7, `view.py:753`). 직접 관측이 상태 없는 로봇의 `pose` 를 채울 수 있으므로 12번의 역할 셈보다 먼저다(역할은 바꾸지 않는다).
11. 로봇 행마다 `name` = `--robot-name` 의 이름, 없으면 robot_id.
12. 지원 이름 = 역할이 SUPPORT 인 로봇 행이 **정확히 하나**일 때 그 `name`, 아니면 None. 정찰 이름도 SCOUT 로 같게. 역할은 상태에서 오므로 상태가 아직 없는 로봇은 셈에 들지 않는다.
13. 명령 낱말: 지원 이름이 있으면 DISPATCH_SUPPORT 낱말 = 「{지원 이름} 출동 승인」, 없으면 「SUPPORT 출동 승인」(`view.py:491-496`).
14. `mission.reason_view` = `mission_reason` 에 사유·지원 이름·정찰 이름을 넘긴 결과(7.4.6).
15. `history`(7.4.5).
16. 자동 수색 불일치 경고(`view.py:763-767`): 사유 목록 = `history` 행들의 `reason`(기록이 None 이면 빈 목록, 오류 행은 None) + 임무 행의 `reason`. `search_start` 가 AUTO 가 아니고(OPERATOR 또는 없음) 이 목록에 `AUTO_SEARCH_ENABLED` 가 있으면 경고 「조정기는 자동 수색(search_start AUTO) — 관제 웹을 --search-start AUTO 로 다시 띄우세요」. 그 거절 명령이 최근 30행에 남아 있는 동안 경고도 남는다. 반대(콘솔 AUTO, 조정기 OPERATOR)는 조정기가 방식을 SystemState 에 싣지 않아 화면이 알 수 없다(`README:49`).
17. `exit` = `_exit`(`view.py:650-657`, `:768`): `exit_locations` 절(모양 검사는 2번, 목록 키 `exits`)이 없거나 목록이 비면 None, 아니면 첫 행(읽기가 최신 순 1행)에서 {location_id, x, y, frame_id = `mission_frame`, observed_at_ns, map_revision}. 행 값은 다시 검사하지 않는다. `passages` = `_passages`(7.4.8, `view.py:769`).
18. `commands` = `command_gates`(7.5)에 지원 이름·정찰 이름·`support_dispatch`·`search_start` 를 넘긴 결과, `map` 기본값 「지도 스냅샷 미연결」, `support_dispatch`·`search_start` 는 인자 그대로.

##### 7.4.1 로봇 행 `_robots` (`view.py:237-282`)
1. 상태 행 중 robot_id 가 배치 목록 밖이면 경고 「배치 밖 로봇 상태 무시: '{ID}'」 후 버린다. 같은 ID 두 번이면 `INVALID_INPUT`.
2. 배치 목록 순서대로 한 행씩 만든다. 상태가 없으면 `has_status` 거짓, 나머지 None, `stale` 참.
3. 상태가 있으면: `age_s` = (`built_at_ns` − `stamp_ns`) ÷ 1e9(로봇 시계와 이 PC 시계가 맞을 때만 뜻이 있음, C-03), `battery_age_s` = (`built_at_ns` − `battery_stamp_ns`) ÷ 1e9, `received_age_s` = (`built_at_ns` − `received_at_ns`) ÷ 1e9(operations 가 받은 시각, 같은 PC 시계).
4. `stale` = `received_age_s` 가 None 이거나 `received_age_s` > `STATUS_STALE_S`(5.0).
5. `dock_state`·`segment_index`·`generation` 은 화면 모델에 넣지 않는다.

##### 7.4.2 대상 행 `_targets` (`view.py:174-234`) + `compose_stage` (`view.py:99-119`)
1. 대상 ID = 장부 보고의 `target_id` 들 ∪ 대상 상태 ID 들, 정렬.
2. 대상마다 최신 보고 = (`reported_at_ns`, `received_at_ns`, `report_id`)가 가장 큰 보고, 보고 수 = 그 대상 보고 수(`view.py:136-150`).
3. 대상 상태가 있으면 확정 여부 = 상태의 `detection_confirmed`, 위치 = 상태 `position`(출처 `target_state`). 보고의 확정 여부(`confirmation_result == CONFIRMED`)와 다르면 경고 「장부 보고와 대상 상태의 확정 여부가 다름 — 대상 상태를 따름」.
4. 대상 상태가 없으면 확정 여부 = 보고의 확정 여부, 위치 = 보고 `position`(출처 `report`), 경고 「구조 상태 입력 없음」.
5. `target_revision` = 상태의 선택 키(없으면 None). 있는데 0 이상 정수가 아니면 `INVALID_INPUT` 「target_revision」. `persisted_report_id` = 상태의 선택 키, 있는데 빈 문자열이거나 문자열이 아니면 `INVALID_INPUT`. 장부의 영수증 상태로 대신 추정하지 않는다.
6. 장부가 있는데 그 대상 보고가 없으면 경고 「장부에 보고가 없는 대상」.
7. 단계: 아래 표에서 위부터 처음 맞는 줄. 미확정인데 구출 완료이거나 지원 ASSIGNED·ARRIVED 이면 경고 「미확정 대상에 구출/지원 상태가 있음 — 입력 불일치」. `rescue_state` 가 계약 `RESCUE_STATES` 밖이거나 `support_state` 가 NONE·ASSIGNED·ARRIVED·FAILED 밖이면 `INVALID_INPUT`. 확정 여부가 bool 이 아니면 `INVALID_INPUT`(그래서 상태도 보고도 없는 대상은 만들어지지 않는다).

| 규칙 | 조건 | 단계 | 화면 낱말 |
| --- | --- | --- | --- |
| R1 | 확정 여부 거짓 | UNCONFIRMED | 미확정 |
| R2 | `rescue_state` = RESCUE_CONFIRMED | RESCUED | 구출 완료 |
| R3 | `support_state` ∈ {ASSIGNED, ARRIVED} | SUPPORTING | 지원 중 |
| R4 | `support_state` 가 None(대상 상태가 없거나, 있어도 값이 null)이고 최신 보고 `approach_result` = APPROACH_COMPLETE | SUPPORTING | 지원 중 |
| R5 | 그 밖 | NOT_RESCUED | 미구조 |

8. 행 값: `report_count` 는 장부가 없으면 None(0 이 아님), `receipt_status` = 최신 보고 영수증 상태, `rescue_reasons` = 상태의 목록(없으면 빈 목록), `warnings` = 규칙 경고 + 위 경고.

##### 7.4.3 사건 행 `_events` (`view.py:300-343`)
1. 장부 보고마다 행: 시각 `observed_at_ns`, `received_at_ns` = 장부 수신 시각, 종류 REPORT, 출처 `source_robot`, 대상 `target_id`, 확정·접근·영수증 상태, 시계 `clock_domain_id`, ref = `report_id`.
2. 현장 사건마다 행: 시각 `observed_at_ns`, `received_at_ns` = None(읽기 함수가 주지 않음), 종류 = 사건 종류, 출처 `camera_id`, 대상 `target_id`(소실 후보만), 신뢰도, 메모 `detail`(빈 값은 None), 시계, ref = `event_id`.
3. (시각, ref) 내림차순 정렬, 앞 50행. `event_total` = 전체 수.

##### 7.4.4 임무 `_mission` (`view.py:466-484`) + `_mission_row` (`view.py:350-359`)
1. 임무 절이 있으면 객체·`schema_version` 문자열·`mission` 키가 있어야 하고, 행이 있으면 `mode`·`phase`·`trigger_event_id`·`started_at_ns`·`updated_at_ns`·`reason`·출동 6칸이 다 있어야 한다. 아니면 `INVALID_INPUT`.
2. 공통: `sign_count` = DISASTER_SIGN 사건 수.
3. 행이 있으면: `mode`, `label` = 모드 낱말, `source` = `mission_state`, 트리거 ID·시각(`started_at_ns`), `phase`, `reason`, `dispatch` = `target_id` 가 None 이면 None, 아니면 출동 6칸(target_id, target_revision, route_revision, segment_index, segment_count, segment_phase), `updated_at_ns`.
4. 행이 없고 현장 사건 절도 없으면 `mode` None, `label` 「모름」, `source` None.
5. 행이 없고 현장 사건이 있으면 DISASTER_SIGN 이 하나라도 있으면 DISASTER 아니면 NORMAL(`source` = `site_events`, 추정), 트리거 = 가장 이른 DISASTER_SIGN.

##### 7.4.5 명령·오류 기록 `_history` (`view.py:499-521`)
1. `operator_commands`·`error_log` 절이 둘 다 없으면 None(픽스처). 
2. 명령 행: 시각 `requested_at_ns`, ref = `command_id`, 무엇 = 명령 낱말(7.4 13번의 낱말, 모르는 종류는 코드), `accepted`, `result`, `reason`, `target_id`, `operator_id`.
3. 오류 행: 시각 `at_ns`, ref = `seq` 문자열, 무엇 = `code`, `detail`.
4. (시각, 종류, ref) 내림차순, 앞 30행. `missing` = 없는 절 이름 목록.

##### 7.4.6 `mission_reason` (`view.py:438-463`) — 조정기 사유 코드 → 문구
1. 코드가 None 이면 None.
2. 이름: {S} = 지원 이름, 없으면 「SUPPORT」. {C} = 정찰 이름, 없으면 「SCOUT」(일반 명칭을 쓰지 않음).
3. 범위(scope): 코드가 아래 「지원 코드 7개」이거나 `APPROACH_FAILED_` 로 시작하면 `support`, 아니면 `SUPPORT_` 로 시작 → `support`, `SCOUT_` → `scout`, `RECHECK_NEEDED_` → `target`, 그 밖 None. 지원 코드 7개 = APPROACH_FAILED, MODE_NOT_DISASTER, ALREADY_DISPATCHED, UNKNOWN_TARGET, TARGET_NOT_CONFIRMED, TARGET_REVISION_STALE, TARGET_ALREADY_JUDGED(`view.py:434-435`). 그래서 아래 표 끝의 코드 9개와 `PATROL_ROUND_{n}` 중 범위가 있는 것은 `SUPPORT_` 접두어인 SUPPORT_EXIT_UNREGISTERED(`support`) 하나뿐이다. 나머지는 None 이라, {C} 를 부르는 SEARCH_GOAL_NOT_SELECTED·START_POSE_*·PATROL_* 도 정찰 카드에는 붙지 않는다.
4. 문구(첫 번째로 맞는 것):
   - 전체가 `SUPPORT_EXIT_IN_{정수}S` → 「{S} 출구 출발까지 {정수}초」, info(정수는 int 로 읽어 앞자리 0 이 빠짐, `view.py:423`, `:447-449`).
   - 전체가 `PATROL_ROUND_{정수}` → 「{C} 순찰 {정수}바퀴째」, info(정수는 int 로 읽어 앞자리 0 이 빠짐, `view.py:424`, `:450-451`).
   - `APPROACH_FAILED_{이유}` → 「{S} 접근 실패: {이유 낱말} — 다시 보내지 않음」, alert. 이유 낱말 표에 없으면 이유 코드 그대로.
   - `RECHECK_NEEDED_{이유}` → 「재확인 필요: {이유 낱말}」, caution. 없으면 코드 그대로.
   - 아래 표에 있는 코드 → 표의 문구(이름 치환)와 등급.
   - 그 밖 → 문구 = 코드, 등급 None.
5. 결과 = {code, text, level, scope}. 화면은 문구 옆에 코드를 함께 보인다(문구와 코드가 같으면 한 번).

| 코드 | 문구 | 등급 |
| --- | --- | --- |
| SUPPORT_EXITING | {S} 출구로 이동 중 | info |
| SUPPORT_AT_EXIT | {S} 출구 도착 | info |
| SUPPORT_EXIT_CANCELED | {S} 출구 이동 취소(판정 종료·재난 해제) | info |
| SUPPORT_EXIT_FAILED | {S} 출구 이동 실패 — 다시 보내지 않음 | alert |
| SUPPORT_POSE_UNKNOWN | {S} 위치 모름 | caution |
| SUPPORT_POSE_BASIS_MISMATCH | {S} 위치의 지도 기준이 다름 — 자동 출동 보류 | caution |
| SUPPORT_AT_TARGET | {S} 가 이미 대상 바로 옆 — 접근 지점을 정할 수 없음 | caution |
| SUPPORT_NOT_READY | {S} 준비 안 됨 — arm 필요 | caution |
| APPROACH_FAILED | {S} 대상 접근 실패 — 다시 보내지 않음 | alert |
| MODE_NOT_DISASTER | 재난 모드가 아님 — {S} 출동 안 함 | info |
| ALREADY_DISPATCHED | {S} 이미 출동함 | info |
| UNKNOWN_TARGET | 모르는 대상 — {S} 출동 안 함 | caution |
| TARGET_NOT_CONFIRMED | 대상 미확정 — {S} 출동 대기 | caution |
| TARGET_REVISION_STALE | 대상 정보가 바뀜 — {S} 출동 다시 판단 | caution |
| TARGET_ALREADY_JUDGED | 이미 판정한 대상 — {S} 출동 안 함 | info |
| SCOUT_RETURNING | {C} 도킹 접근 지점으로 이동 중 | info |
| SCOUT_AT_DOCK_APPROACH_DOCK_BY_HAND | {C} 도킹 접근 지점 도착 — 도킹은 사람이 합니다 | info |
| SCOUT_RETURN_FAILED | {C} 도킹 접근 지점 이동 실패 — 다시 보내지 않음 | alert |
| TARGET_TIME_BASIS_UNUSABLE | 대상 위치의 관측 시각을 쓸 수 없음(시계 미측정·오차 상한 없음) | caution |
| TARGET_CLOCK_DOMAIN_MISMATCH | 대상 위치 관측의 시계 영역이 다름 | caution |
| TARGET_OBSERVATION_FUTURE | 대상 위치 관측 시각이 미래 | caution |
| TARGET_OBSERVATION_STALE | 대상 위치 관측이 오래됨 | caution |
| MAP_FREEZE_WAITING_SCOUT_STOP | {C} 이 멈추기를 기다림 — 멈추면 지도 고정 | info |
| MAP_FREEZE_REQUESTED | 지도 고정 요청함 — 고정 지도 확인 중 | info |
| MAP_NOT_FROZEN | 지도가 아직 고정되지 않음 — {S} 출동 대기 | caution |
| MAP_FREEZE_FAILED | 지도 고정 실패 — 다시 요청하지 않음 | alert |
| TARGET_NOT_ON_FROZEN_MAP | 고정 지도 기준 대상 위치가 아직 없음 — {S} 출동 대기 | caution |
| SCOUT_NOT_LOCALIZED | {C} 이 고정 지도에서 위치를 못 잡음 — {S} 출동 대기 | caution |
| SUPPORT_NOT_LOCALIZED | {S} 가 고정 지도에서 위치를 못 잡음 — 출동 대기 | caution |
| SUPPORT_APPROACH_NOT_FREE | {S} 접근 지점이 막힘 — 출동 보류 | caution |
| SUPPORT_EXIT_NOT_FREE | {S} 출구 지점이 막힘 — 출구 이동 보류(대상 옆 대기) | caution |
| SCOUT_HOME_NOT_FREE | {C} 도킹 접근 지점이 막힘 — {C} 복귀 보류 | caution |
| NO_HOME_POSITION | 복귀 위치(출구·도킹 접근) 설정 없음 — 복귀 이동 안 함 | caution |
| SEARCH_GOAL_NOT_SELECTED | {C} 수색 목표 선택 대기 | caution |
| AUTO_SEARCH_ENABLED | 자동 수색 — 지진 감지로 시작, 수색 시작 버튼은 받지 않음 | info |
| MAP_FREEZE_WAITING_EXIT | 출구 QR 찾는 중 — {S} 출동 대기 | info |
| TARGET_REBASED_ON_FREEZE | 대상 위치를 고정 지도로 옮김 — 보고 저장 대기 | info |
| EXIT_NOT_ON_FROZEN_MAP | 출구(QR) 위치가 고정 지도 기준에 없음 — {S} 출동·출구 이동 보류 | caution |
| SUPPORT_EXIT_UNREGISTERED | 출구(QR) 미등록 — {S} 출동·출구 이동 보류 | caution |
| START_POSE_UNKNOWN | {C} 출발 자세 모름 — 순찰·{C} 복귀 보류 | caution |
| START_POSE_CONFLICT | {C} 출발 자세가 다른 값으로 다시 옴 — 처음 값 유지, 확인 필요 | alert |
| PATROL_EXHAUSTED | {C} 순찰 끝(정한 바퀴 다 돎) — 더 돌지 않음 | caution |

근거: `view.py:365-414`(새 9개는 `:403-413`). 접근 실패 이유 낱말(`view.py:417-422`): PASSAGE_BLOCKED 통로 막힘, UNREACHABLE 갈 수 없는 위치, FAILED 실행 실패, CANCELED 취소됨, EXPIRED 시간 초과, NAV_NOT_READY 주행 준비 안 됨, DEADLINE_EXPIRED 시간 초과, STOP_LATCHED 정지 유지 중, LOCAL_LEASE_EXPIRED 명령 유효 시간 지남, CLOCK_UNCERTAIN 시계 불확실. 재확인 이유 낱말(`view.py:426-431`): CANDIDATE_NOT_LINKED 후보 연결 안 됨, FRAME_REVISION_MISMATCH 지도 기준 다름, CANDIDATE_TIME_BASIS_UNUSABLE 시각 기준 못 씀, ORDER_VIOLATION 시각 순서 틀림, CAMERA_NOT_ABSENT 인형이 아직 보임, LIDAR_NOT_CLEAR 자리가 막힘, COVERAGE_NOT_VALID 관측 범위 부족, RECHECK_OBSERVATION_MISSING 재확인 관측 없음 — 「재확인 지시」 버튼으로 다시 보낼 수 있음.

사유를 보이는 곳(`render.py:141-146`, `:166-167`, `:363`, `:388-392`): 임무 띠에는 늘. 로봇 카드에는 역할이 SUPPORT 이고 범위가 `support` 일 때, 역할이 SCOUT 이고 범위가 `scout` 일 때. 역할이 그 둘이 아닌 카드(UNKNOWN)에는 범위가 None 인 사유가 붙는다(관찰된 동작, `render.py:363`). 대상 줄에는 그 대상이 출동 중인 대상(`dispatch.target_id`)이고 범위가 `support`·`target` 일 때, 또는 출동이 없고 AUTO 이고 그 대상 단계가 미구조이며 범위가 `support` 일 때.

##### 7.4.7 로봇 직접 관측 `_apply_watch` (`view.py:660-689`)
입력은 7.3 3번의 객체 {data, problem, stale_s} 또는 None. 낡음은 이 서버의 시계로 잰다(관측 노드와 같은 운영 PC 라는 전제).
1. 모든 로봇 행에 `watch` = None. 입력이 None(`--robot-watch` 없음)이면 끝.
2. `data` 가 None(파일 없음·못 읽음·모양 다름)이면 `problem` 을 경고로 붙이고 끝(요청은 실패하지 않는다).
3. 파일 나이 = (`built_at_ns` − `written_at_ns`) ÷ 1e9. 이 값이 `stale_s` 보다 크면 「죽은 관측」이고 경고 「로봇 직접 관측 파일이 {초}초 동안 갱신 안 됨 — robot_watch 노드 확인」(초는 정수 반올림).
4. 파일의 로봇 ID 중 배치 목록 밖은 정렬해 하나씩 경고 「배치 밖 로봇 직접 관측 무시: '{ID}'」(버리되 숨기지 않음).
5. 배치 로봇마다 파일에 그 로봇이 있으면: `age_s` = (`built_at_ns` − `last_seen_ns`) ÷ 1e9(없으면 None), 위치 나이 = (`built_at_ns` − `pose_seen_ns`) ÷ 1e9(없으면 None). `stale` = 죽은 관측이거나 `age_s` 가 None 이거나 `age_s` > `stale_s`. 위치 = 죽은 관측이 아니고 위치 나이가 있고 `stale_s` 이하일 때만 파일의 `pose`, 아니면 None(낡은 위치를 그리지 않음).
6. `watch` = {namespace, age_s, stale, battery_percent, docked, pose}. 프로젝트 상태가 없는 로봇(`has_status` 거짓)이고 위치가 있으면 행의 `pose` 를 그 위치로 채운다. 프로젝트 상태가 있으면 상태의 위치가 먼저이고 바꾸지 않는다.

##### 7.4.8 터널 판정 `_passages` (`view.py:629-647`)
입력은 `overhead_passages` 절({robots: 로봇 ID → 목록, current_map: 객체 또는 None}) 또는 None. 이 절은 `_section` 모양 검사를 거치지 않는다.
1. 절이 None 이면 빈 배열(경고 없음).
2. 로봇 목록마다 항목마다 한 행: 낱말 = `PASSAGE_WORDS`[state](모르는 state 는 그대로). `label` = `lowest_m` 이 None 이거나 state 가 UNKNOWN 이면 낱말만, 아니면 「{낱말} ↑{lowest_m 소수 둘째 자리} m」. `frame_id` = 그 목록의 `map_frame`. `current` = `current_map` 이 None 이거나 항목의 (`map_session_id`, `map_revision`) 이 `current_map` 의 것과 같으면 참.
3. 행이 있는데 `current_map` 이 None 이면 경고 「현재 지도 revision 모름 — 터널 판정을 모두 진하게 그림」.
4. 판정을 다시 하지 않는다: 통과 여부는 인식(overhead_passage) 판정 그대로다. 지도 위치·frame 검사는 7.9 `map_overlay` 가 한다.

#### 7.5 `_gate` (`view.py:530-595`) + `command_gates` (`view.py:598-623`) — 버튼별 활성 관문 거울

명령 목록과 묶음(`view.py:38-56`): 승인 묶음 = 수색 시작 START_SEARCH, 출동 승인 DISPATCH_SUPPORT, 다음 구간 계속 CONTINUE_SEGMENT. 임무 묶음 = 재확인 지시 REQUEST_RECHECK, 복귀 RETURN_HOME, 일시정지 PAUSE, 재개 RESUME, 작업 취소 CANCEL. 정지 묶음 = 정지 STOP_ROBOT. 보낼 수 있는 것은 앞 다섯뿐(`commands.py:26-33`).

판정 순서(첫 번째로 걸리는 것이 답):
1. `HELD_COMMANDS`(`view.py:58-63`): PAUSE·RESUME·CANCEL → 끔 「Q-22 재개 관문 전」, STOP_ROBOT → 끔 「RequestStop 별도 경로 — 위험하면 현장 비상 정지 장치」. 상태와 무관하게 늘 끔.
2. 임무 `source` 가 `mission_state` 가 아니면(행 없음) → 끔 「임무 상태 없음 — operations 가 조정기 상태를 기록하면 켜짐」.
3. 모드가 DISASTER 가 아니면 → 끔 「재난 모드 아님 ({모드 낱말})」.
4. 명령별 규칙(아래).
5. `command_gates`(`view.py:604-622`)는 명령마다 먼저 1~4 로 판정한 뒤, 두 AUTO 검사를 이 순서로 본다. 결과를 버리고 상태 행으로 바꾼다(`args` {}, `choices` [], 꺼짐, `auto: true`). 화면은 버튼 대신 점선 상자 한 줄을 그린다.
   - DISPATCH_SUPPORT 이고 `--support-dispatch AUTO`(`view.py:606-613`): 낱말 「{S} 자동 출동」, 사유 「{S} 자동 출동 — 조정기가 보냄」.
   - START_SEARCH 이고 `--search-start AUTO`(`view.py:614-620`): 낱말 「{C} 자동 출발」({C} = 정찰 이름, 없으면 「SCOUT」), 사유 「{C} 자동 출발 — 지진 감지로 조정기가 수색 시작」. 조정기는 이때 운영자의 START_SEARCH 를 NOT_AUTHORIZED(`AUTO_SEARCH_ENABLED`)로 거절한다(`README:49`).
   - 두 검사는 행 이름이 달라 한 행에 둘 다 걸리지 않는다. 행 순서는 `COMMANDS` 순서 그대로라 승인 묶음에서 수색 시작 상태 줄이 출동 승인(버튼 또는 상태 줄) 위에 온다.

명령 × phase(모드 DISASTER 이고 임무 행이 있을 때). 「D」「C」「R」 은 phase 와 무관한 조건이다. `--search-start AUTO` 면 「수색 시작」 열이, `--support-dispatch AUTO` 면 「출동 승인」 열이 모드·phase 와 무관하게 5번의 상태 행이다(1~3 에서 꺼진 경우도).

| phase | 수색 시작 | 출동 승인 | 다음 구간 계속 | 재확인 지시 | 복귀 |
| --- | --- | --- | --- | --- | --- |
| IDLE | 끔 「이미 수색을 시작함」 | D | C | R | 끔 「구출 판정 전」 |
| DISASTER | **켬** 「재난 감지 — 아직 수색 전」 | D | C | R | 끔 「구출 판정 전」 |
| SEARCHING | 끔 「이미 수색을 시작함」 | D | C | R | 끔 「구출 판정 전」 |
| TARGET_FOUND | 끔(같음) | D | C | R | 끔 「구출 판정 전」 |
| DISPATCH_APPROVED | 끔 | D | C | R | 끔 「구출 판정 전」 |
| SEGMENT_MOVING | 끔 | D | C | R | 끔 「구출 판정 전」 |
| AWAITING_CONTINUE | 끔 | D | C | R | 끔 「구출 판정 전」 |
| ARRIVED | 끔 | D | C | R | 끔 「구출 판정 전」 |
| CANDIDATE | 끔 | D | C | R | 끔 「구출 판정 전」 |
| RECHECKING | 끔 | D | C | R | 끔 「구출 판정 전」 |
| RESCUE_CONFIRMED | 끔 | D | C | R | **켬** 「판정 뒤 — 두 로봇 복귀」 |
| RECHECK_NEEDED | 끔 | D | C | R | **켬** 「판정 뒤 — 두 로봇 복귀」 |
| RETURNING | 끔 | D | C | R | 끔 「이미 복귀 중」 |
| FINISHED | 끔 | D | C | R | 끔 「구출 판정 전」(보통은 모드가 MISSION_FINISHED 라 3번에서 꺼짐) |

- D(출동 승인), 차례로: 출동이 이미 있으면(`dispatch` 있음) 끔 「이미 출동함」 → 확정 대상 = 확정 여부 참이고 단계가 구출 완료가 아닌 대상, 없으면 끔 「확정 대상 없음」 → 그중 `target_revision` 이 있는 대상, 없으면 끔 「대상 revision 없음 — 대상 상태에 revision 이 기록되면 켜짐」 → 그중 `persisted_report_id` 가 있는 대상이 선택지, 없으면 끔 「보고 저장 대기 — 조정기가 저장 영수증을 받으면 켜짐」 → 켬 「확정 대상 {k}명 중 고름」, 빠진 대상이 w명(w > 0)이면 뒤에 「 (보고 저장 대기 {w}명 빠짐)」. 선택지 항목 = {target_id, target_revision, stage, stage_label}, `args` 없음.
- C(다음 구간 계속): 출동이 없으면 끔 「출동 전」 → `segment_phase` ≠ AWAITING_APPROVAL 이면 끔 「승인 대기 아님 ({구간 상태 낱말})」 → 켬 「구간 {segment_index}/{segment_count} 끝 — 다음 구간 승인 대기」, `args` = 임무 행의 {target_id, target_revision, route_revision, segment_index}, 선택지 없음.
- R(재확인 지시): `rescue_state` 가 RESCUE_CONFIRMED 가 아닌 대상(상태가 없는 대상 포함)이 선택지, 없으면 끔 「재확인할 대상 없음」, 있으면 켬 「대상 {k}명 중 고름」.
- 브라우저의 최종 활성(`render.py:959`): 버튼이 눌리려면 (ADMIN 이고 `--operations` 있음) 이고 관문 켬이고 갱신 끊김이 아니고 전송 중이 아니고 확인 칸이 닫혀 있어야 한다. 끊김이면 모든 버튼의 사유 글자가 「갱신 끊김 — 화면이 최신이 아니라 꺼 둠」으로 바뀐다. 관문 정보가 없는 버튼은 꺼짐 「관문 정보 없음」. AUTO 상태 줄은 `data-command` 가 없는 `<p>` 라 이 규칙 밖이다(`render.py:955`; 관문 JSON 에는 꺼짐으로 남는다).

#### 7.6 명령 보내기 — `POST /commands` (`app.py:324-356`), `build_request` (`commands.py:91-112`), `post_json` (`commands.py:115-136`), `read_reply` (`commands.py:143-165`), `send_command` (`commands.py:168-175`)

라우트 순서:
1. 로그인 확인(익명이면 401 JSON).
2. CSRF 확인 → 틀리면 400 `CSRF`.
3. `command_access` 에 역할과 operations 주소 유무를 넘김(`commands.py:68-74`): 역할이 ADMIN 이 아니면 403 `NOT_AUTHORIZED`, ADMIN 인데 주소가 없으면 503 `NOT_CONNECTED`. 켜졌을 때 칸 위 안내 「누르면 operations 로 한 번 보냅니다. 받을지는 조정기가 정합니다」.
4. `build_request`: `kind` 가 다섯 종류 밖이면 `INVALID_INPUT` 「command '{kind}' is not sendable from the console」(PAUSE 등 보류 명령도 여기서 걸림). 그 종류가 받지 않는 키가 있으면 「{kind} does not take {키들}」. 필드마다 정수 필드는 `_count`, `target_id` 는 `_target_id`. 실패는 400.
5. `send_command`: 주소 + `/api/operator-commands` 로 `post_json` 한 번, 그 결과를 `read_reply`.
6. 성공이면 200, 실패면 502. 다시 보내지 않는다.

`post_json` 실패 분류(`commands.py:119-136`):

| 상황 | 코드 | delivery |
| --- | --- | --- |
| HTTP 상태가 무엇이든 답이 옴 | (실패 아님) 상태와 본문을 돌려줌 | — |
| 연결 단계 시간 초과(URLError 의 원인이 timeout) | TIMEOUT 「no answer within 5s」 | UNKNOWN |
| 연결 거부 또는 이름 풀이 실패 | UNREACHABLE | NOT_SENT |
| 그 밖의 URLError | UNREACHABLE | UNKNOWN |
| 읽는 중 시간 초과 | TIMEOUT | UNKNOWN |
| 그 밖의 OS 오류 | SEND_FAILED | UNKNOWN |

`read_reply` 규칙: 본문 JSON 이 5.4 의 검사를 통과하면 5개 키 {accepted, request_id, reason, request_state, created_task_id}를 돌려준다. 통과하지 못하면 본문 앞 200 byte 를 붙여, HTTP 2xx 이거나 본문이 JSON 객체면 BAD_RESPONSE 「HTTP {n}: not an operator-command answer: …」, 아니면 HTTP_ERROR 「HTTP {n}: …」. 둘 다 delivery UNKNOWN(요청이 operations 에 닿았음).

브라우저 `send()`(`render.py:1052-1079`): 폼 = `csrf`(명령 칸의 `data-csrf`), `kind`, 선택지를 고른 경우 `target_id`(DISPATCH_SUPPORT 면 `target_revision` 도), 그리고 `args` 의 키·값. 상대 주소 `commands` 로 POST. 응답 JSON 을 못 읽으면 null.
`describe()`(`render.py:1035-1048`)가 남기는 기록 한 줄:
- `sent` 참 + 접수 → 초록 「{명령}: 접수됨 (실행 완료 아님) — request_id {ID} ({request_state})」.
- `sent` 참 + 거절 → 노랑 「{명령}: 거절 {reason} ({reason_label})」 + 같은 꼬리. 낱말이 코드와 같으면 괄호 생략.
- `sent` 거짓 → 빨강 「{명령}: 보내지 못함 — {error}: {message}」 + delivery 가 NOT_SENT 면 「 · 전달 안 됨」, 아니고 `command_id` 가 있으면 「 · 전달됐는지 모름 — command_id {ID} 로 명령 기록을 확인한 뒤 다시 누르세요」.
- 401 → 「{명령}: 보내지 못함 — 로그인이 끝났습니다. 다시 로그인하세요.」
- 그 밖(JSON 아님) → 「{명령}: 보내지 못함 — 콘솔 응답 HTTP {n}, 전달됐는지 모름」.
- fetch 자체 실패 → 「{명령}: 보내지 못함 — 콘솔 서버에 닿지 못함 ({오류}), 전달됐는지 모름」.
기록은 최신이 위, 6줄까지. 이 기록은 그 페이지에서 누른 것만이고, 모두가 보는 기록은 6.6 「명령·오류 기록」 칸(operations 가 DB 에 남긴 것)이다.

#### 7.7 계정 — `accounts.py`

계정 DB 스키마(`accounts.py:138-154`): 파일 하나, 저널 기본값(WAL 아님).

| 표 | 열 | 타입·키 | 규칙 |
| --- | --- | --- | --- |
| `metadata` | `key` | TEXT PRIMARY KEY | 행 두 개: `role` = `console-accounts`, `schema_version` = `1.0` |
| `metadata` | `value` | TEXT NOT NULL | — |
| `accounts` | `username` | TEXT PRIMARY KEY | 5.2 정규식 |
| `accounts` | `account_id` | TEXT NOT NULL UNIQUE | 만들 때마다 새 UUID4. 같은 이름을 지웠다 다시 만들면 다른 값 |
| `accounts` | `display_name`, `email` | TEXT NOT NULL | 빈 표시 이름은 아이디로 |
| `accounts` | `role` | TEXT NOT NULL | ADMIN 또는 CUSTOMER |
| `accounts` | `status` | TEXT NOT NULL | ACTIVE 또는 INACTIVE |
| `accounts` | `password_hash` | TEXT NOT NULL | `pbkdf2_sha256${반복}${소금 hex}${해시 hex}` |
| `accounts` | `auth_version` | INTEGER NOT NULL | 처음 1, 비밀번호·역할·상태가 실제로 바뀔 때마다 +1 |
| `accounts` | `created_at_ns`, `updated_at_ns` | INTEGER NOT NULL | 서버 ns |

- 해시(`accounts.py:63-73`): 비밀번호 8자 이상 → 소금 16 byte(`secrets`) → PBKDF2-HMAC-SHA256(UTF-8 비밀번호, 소금, 200000회) → 위 문자열. 평문은 저장·로그하지 않는다.
- 확인(`accounts.py:76-90`): 저장값을 `$` 로 4조각, scheme 이 `pbkdf2_sha256` 이 아니면 거짓, 반복 수는 **저장값의 것**으로 다시 계산, hex 문자열을 상수 시간 비교. 형식 오류는 거짓.
- `authenticate`(`accounts.py:209-223`): 아이디·비밀번호가 문자열이 아니면 None. 행을 찾고, 행이 없으면 가짜 해시(프로세스에서 한 번 만든 무작위 비밀의 해시)로, 있으면 행 해시로 늘 한 번 확인한다. 행 없음·ACTIVE 아님·틀림은 모두 같은 None(로그도 없음).
- `create`(`accounts.py:126-158`): 파일이 이미 있으면 `ALREADY_EXISTS`(덮어쓰지 않음). 아이디·비밀번호 검사 뒤 두 표를 만들고 관리자 한 명(표시 이름 = 아이디, 이메일 빈 값, ADMIN, ACTIVE, auth_version 1)을 넣는다. sqlite 오류는 `STORAGE_ERROR`.
- `open`(`accounts.py:160-177`): 파일이 없으면 `STORAGE_ERROR`(만들지 않음). `metadata` 를 못 읽으면 또는 `role`·`schema_version` 이 다르면 `POLICY_MISMATCH`.
- 모든 호출은 새 연결 + `BEGIN IMMEDIATE` … `COMMIT`/`ROLLBACK`(`accounts.py:307-350`). 시작 실패는 `STORAGE_ERROR` 「accounts database busy or broken」.
- `add`(`accounts.py:226-243`): 검사(아이디·표시 이름·이메일·역할·상태·비밀번호) → 같은 아이디가 있으면 `ALREADY_EXISTS` → 삽입(auth_version 1).
- `update`(`accounts.py:245-283`): 바꿀 것이 하나도 없으면 `INVALID_INPUT` 「nothing to update」. 행이 없으면 `NOT_FOUND`. 지금 (ADMIN, ACTIVE)인데 바뀐 뒤 (ADMIN, ACTIVE)가 아니면 다른 ACTIVE ADMIN 이 하나 이상 있어야 함, 아니면 `LAST_ADMIN`. 비밀번호가 바뀌거나 역할·상태 값이 실제로 달라지면 `auth_version` +1. `updated_at_ns` 갱신.
- `delete`(`accounts.py:285-297`): 지우려는 아이디가 로그인한 본인이면 `SELF_DELETE`. 행이 없으면 `NOT_FOUND`. ACTIVE ADMIN 이면 마지막 ADMIN 보호. 삭제.
- `init` 명령(`accounts.py:353-374`): `init {파일} --admin {아이디} [--password-env {변수}]`(기본 `RESCUE_CONSOLE_ADMIN_PASSWORD`, 비면 프롬프트). 실패는 `ACCOUNTS FAILED: …` + 종료코드 2.

#### 7.8 로그인 반복 실패 잠금 — `LoginThrottle` (`throttle.py:21-73`)

1. 열쇠 = (아이디를 문자열로·앞뒤 공백 제거·소문자, 접속 주소 `request.remote_addr`).
2. `locked_for`: 잠금 끝 시각 − 지금(단조 시계)을 올림한 초, 0 미만이면 0. 로그인 라우트는 이 값이 0 이 아니면 비밀번호를 확인하지 않고 거절한다.
3. `failed`: 300 s 안의 실패 시각만 남기고 지금을 더한다. 5개 이상이면 잠금 끝 = 지금 + 60 s, 목록 비움. 열쇠를 맨 뒤로 다시 넣고, 열쇠가 10000개를 넘으면 가장 오래된 것부터 지운다.
4. `succeeded`: 그 열쇠 기록 삭제.
5. 모든 메서드는 잠금 안에서. 계정 DB 에는 아무것도 쓰지 않는다. 다른 아이디·다른 주소는 영향이 없다.

#### 7.9 지도 — `mapimage.py`

`load_map`(`mapimage.py:156-207`), YAML + PGM → PNG:
1. YAML 을 byte 로 읽는다(실패 `STORAGE_ERROR` 「map file unreadable: …」). UTF-8 이 아니면 `INVALID_INPUT`.
2. `parse_map_yaml`(`mapimage.py:49-77`): 줄마다 `#` 뒤를 버리고, 빈 줄은 건너뜀. 들여쓴 줄·`:` 없는 줄·값이 빈 줄은 거절. 키는 `image`·`mode`·`resolution`·`origin`·`negate`·`occupied_thresh`·`free_thresh` 만(다른 키는 거절, 중복 거절). 값이 `[` 로 시작하면 `]` 로 끝나야 하고 쉼표로 나눈 스칼라 목록. 스칼라는 따옴표 문자열 → int → float → 문자열 순. `image`·`resolution`·`origin` 이 없으면 거절.
3. `origin` 은 수 3개 목록, 셋째(yaw)는 정확히 0.0 이어야 함. `resolution` 은 유한 양수. `negate` 는 0 또는 1(없으면 0). `image` 는 빈 값이 아닌 문자열, 상대 경로면 YAML 폴더 기준.
4. PGM(`mapimage.py:86-121`): 머리 토큰 4개(주석 `#…` 건너뜀) — `P5`, 폭, 높이, maxval(1~255). 머리 뒤 공백 1 byte 다음이 화소이고 개수 = 폭 × 높이 정확히. 아니면 `INVALID_INPUT`.
5. maxval ≠ 255 면 화소 × 255 ÷ maxval(내림). `negate` 면 255 − 화소. 
6. PNG(`mapimage.py:124-136`): 8-bit 회색조, 각 행 앞에 필터 0, zlib 9, IHDR·IDAT·IEND.
7. meta = {source(YAML 이름), image(PGM 이름), width_px, height_px, resolution, origin, frame_id = `map`, mode, yaml_sha256, pgm_sha256, url = `map.png?v={pgm_sha256 앞 12자}`}.

`LiveMap.current`(`mapimage.py:233-270`), 잠금 안에서:
1. YAML stat. 실패하면: 읽은 지도가 없으면 (None, MISSING 「실시간 지도 없음 — live_map.yaml 를 아직 받지 못함」), 있으면 (그 지도, ERROR 「지도 읽기 실패: {사유} — 마지막으로 읽은 지도를 보임」, 받은 시각 None).
2. 열쇠 (mtime_ns, 크기)가 지난번과 다르면 `load_map` 으로 다시 읽음. 실패하면 이전 지도를 남기고 오류 문구를 기억.
3. 받은 시각 = YAML mtime, `age_s` = (지금 − 받은 시각) ÷ 1e9 를 소수 첫째 자리로 반올림.
4. 오류가 기억돼 있으면 ERROR 「지도 읽기 실패: {문구}」(+ 이전 지도가 있으면 「 — 마지막으로 읽은 지도를 보임」).
5. alive = `live_map.alive` 의 mtime(없으면 받은 시각). 들은 시각 = max(받은 시각, alive). 지금 − 들은 시각 > `stale_s` × 1e9 이면 STALE 「지도 낡음: {초}초 동안 지도 발행 없음」.
6. 아니면 OK, 문구 빈 값.

`world_to_pixel`(`mapimage.py:273-285`): 열 = (x − origin_x) ÷ resolution, 행 = 높이 − (y − origin_y) ÷ resolution, 둘 다 소수 둘째 자리로 반올림한 뒤 0 ≤ 열 ≤ 폭 그리고 0 ≤ 행 ≤ 높이가 아니면 None(가장자리로 끌어오지 않음).

`map_overlay`(`mapimage.py:288-325`): 로봇 행, 대상 행, 화면 모델 `exit` 가 있으면 출구 하나(종류 `exit`, ID = `EXIT_NAME` 「출구(QR)」, 위치 = `exit` 의 x·y·frame_id, `mapimage.py:304-306`), 그리고 터널 행(종류 `passage`, ID = 항목 `id`, 위치 = 행의 x·y·frame_id, `mapimage.py:307-309`)마다 차례로
1. 위치가 None 이면 로봇만 `no_pose` 에 {id, name} 로 남긴다(지어내지 않음). 대상은 그냥 건너뜀. 출구는 `exit` 가 None 이면 처음부터 넣지 않는다.
2. `frame_id` ≠ `map` 이면 `frame_mismatch` 에 {kind, id, x, y, frame_id, label}(label 은 터널만 값이 있고 나머지는 None).
3. 지도 밖이면 `off_map` 에.
4. 아니면 로봇 {id, px, py, yaw, role, name} 또는 대상 {id, px, py, stage} 또는 출구 {id, px, py, observed_at_ns, map_revision}(목록 `exits`) 또는 터널 {id, px, py, state, label, current, robot_id, map_revision}(목록 `passages`).
5. `note` = 「기준 지도 {파일} ({폭}×{높이} px, {해상도} m/px) — 좌표 map_session 일치는 확인하지 않음(Q-09)」.

`/map.png` 는 실시간이면 `LiveMap.current` 가 준 지도(낡음·오류여도 마지막 좋은 지도), 아니면 고정 지도, 둘 다 없으면 404.

`--map` 과 `--live-map` 의 차이: `--map` 은 기동 때 한 번 읽고 실패하면 기동 실패, 낡음 개념 없음, 설명 줄 「재난 전 기준 지도, 격자 한 칸 1 m. 좌표가 같은 지도 세션인지는 아직 확인하지 않습니다 (Q-09).」. `--live-map` 은 요청마다 파일을 보고 바뀌면 다시 읽으며, 없음·낡음·오류를 칸과 경고에 드러낸다.

#### 7.10 화면 그리기 — `render.py`

시각·나이 형식:
- 시각(`render.py:28-35`): 서버 지역 시각 `HH:MM:SS.mmm`, 서버 집계 시각과 날짜가 다르면 `YYYY-MM-DD ` 를 앞에. 마우스를 올리면 원래 ns.
- 나이(`render.py:38-51`): 60 s 미만 소수 첫째 자리 + `s`, 1시간 미만 「{분}분 {초 두 자리}초」, 하루 미만 「{시}시간 {분}분」, 그 이상 「{일}일」, 음수는 앞에 `-`(상대 시계가 앞섬). 브라우저 쪽 같은 함수는 60 s 미만을 소수 **둘째** 자리로 쓴다(`render.py:803-809`).
- 경과(`render.py:89-101`): None·음수 「—」, 1시간 미만 「{분}분 {초 두 자리}초」, 하루 미만 「{시}시간 {분 두 자리}분」, 그 이상 「{일}일 {시}시간」.
- 짧은 ID(`render.py:72-81`): 9자 이하 그대로, 아니면 앞 8자, 전체는 title.
- 코드 → 낱말은 `labels.py`(7.11), 없는 값은 「—」.

임무 띠(`render.py:149-180`): `section#mission.band.{모드 소문자, 없으면 unknown}`, `data-mode`. 큰 글씨 = `mission.label`. 사실 줄: 트리거 시각이 있으면 「트리거」(시각), 「경과」(집계 − 트리거); 모드 DISASTER 이고 진동 감지가 있으면 「진동 감지 {n}건」; phase 가 있으면 「단계」(phase 낱말); 출동이 있으면 「구간」 「{segment_index}/{segment_count} {구간 상태 낱말}」(title 「끝낸 구간 / 전체 구간」); 사유가 있으면 「사유」. 설명: 출처가 임무 행이면 「조정기 임무 상태, {집계 − updated_at_ns 나이} 전 바뀜」, 추정 DISASTER 「임무 상태 기록 전 — 현장 카메라의 진동 감지로 추정」, 추정 NORMAL 「임무 상태 기록 전 — 진동 감지 없음」, 모름 「현장 사건을 읽지 못해 모드를 알 수 없음」. 오른쪽 끝 「서버 집계 {시각}」 + `data-built-ns`. 출구 누적은 그리지 않는다.

확인할 것(`render.py:183-187`): 경고 없으면 빈 숨은 칸, 있으면 「확인할 것 {n}」 + 목록.

지도 칸(`render.py:211-295`):
- 지도가 없고 실시간 상태가 있으면 「지도」 + live 문구 + 「운영 launch 의 map_topic·map_dir 와 SLAM 지도 발행을 확인하세요.」.
- 지도가 없으면 「기준 지도가 없습니다. 실행할 때 --map assets/maps/pre_quake/map.yaml 을 주세요.」.
- 있으면 SVG 하나(viewBox = 지도 픽셀 + 왼쪽 5·위 2·아래 9 여백): 바탕 `<image>`(PNG, 늘이기, 반투명), 테두리, 1 m 격자(0 m 축은 진하게)·축 숫자·1 m 축척 막대, 대상 V 표식(꼭짓점 = 좌표, 단계별 색, 구출 완료는 가로줄, 이름표 = ID 앞 4자, title = ID + 단계 낱말), 출구(QR) 관측 표식(`render.py:242-250`: 좌표가 중심인 한 변 2 × `TARGET_MARK_PX` = 4 px 정사각형, 채움 없음·초록 테두리(`render.py:641`), 이름표 = 「출구(QR) 관측 HH:MM:SS」(`observed_at_ns` 를 서버 지역 시각 초 단위로), title = 이름표 + 「 · map_revision {n} (조정기 등록 출구 아님 — 관측)」; 대상 다음·로봇 앞에 그려 로봇이 위에 온다), 터널 판정 표식(`render.py:234-241`: 대상 다음·출구 앞. 좌표가 가운데인 ∏ 모양 — 왼쪽 기둥·지붕·오른쪽 기둥, 반 폭 = `TARGET_MARK_PX`; 클래스 `passage {state 소문자} {current|old}` 로 색 초록 통과 가능·빨강 통과 불가·회색 판정 불가, 이전 지도(`old`)는 투명도 0.35(`render.py:642-645`); 이름표 = `label`; title = 「{label} · {robot_id} · map_revision {n}」 + 이전 지도면 「 (이전 지도)」), 로봇(yaw 가 있으면 진행 방향 화살촉 삼각형, 없으면 원; SVG 의 y 는 아래로 자라므로 +yaw 는 −y; 이름표 = 표시 이름; SUPPORT 는 파랑). 범례(네 단계 V + 「로봇과 진행 방향」, 출구는 범례에 없다). 설명 줄: 고정 지도면 7.9 끝의 문구, 실시간이면 「실시간 SLAM 지도 · 받은 시각 HH:MM:SS ({n}초 전) · {폭}×{높이} px · {해상도} m/px · 해시 {12자} · 격자 한 칸 1 m」 + 문구. 실시간 상태가 OK 가 아니면(STALE·ERROR) 칸에 `stale` 클래스(지도 흐림, 설명 빨강). 그리지 못한 위치 목록(`render.py:268-270`): 「{ID 8자}: 좌표계 {frame} — 지도 좌표계가 아님」, 「{ID 8자}: 지도 밖 ({x}, {y}) m」, 「{이름}: 위치 미확정」. 출구는 ID 가 「출구(QR)」(8자 미만)라 「출구(QR): 지도 밖 ({x}, {y}) m」처럼 보인다. 지도 밖 줄은 `label` 이 있으면 ID 대신 그것을 쓰므로 터널은 「통과 불가 ↑0.40 m: 지도 밖 (…) m」이다(`render.py:269`). 범례에 터널은 없다.

로봇 카드(`render.py:327-365`): 배치 로봇마다. 머리 = 표시 이름(큰 글씨)·역할 낱말·robot_id.
- 상태 없음 + 직접 관측 있음(7.4.7): `unit watch` 카드(직접 관측이 낡았으면 `stale`) + 「프로젝트 상태 없음 — 로봇 기본 토픽 직접 관측만 보임」 + 직접 관측 줄 + 「위치」(`render.py:340-345`).
- 상태 없음(직접 관측도 없음): 카드 노랑 테두리 + 「상태를 한 번도 받지 못했습니다. 로봇 상태 발행과 operations 연결을 확인하세요.」.
- 상태 있음: 이동 상태 낱말(STOP_UNCONFIRMED 는 빨강)·준비 낱말, 배터리 막대(없음 「배터리 값 없음」; 25 % 미만 빨강, 50 % 미만 노랑, 그 밖 초록; 폭은 0~100 으로 자름; 정수 % ; 「{나이} 전」), 「위치」 = 「(x, y) m {yaw 도}°」(소수 둘째 자리, 도는 정수, frame 이 `map` 이 아니면 「 [frame]」), 「작업」 = 짧은 `active_task_id`, 「수신」 = 「{received_age} 전」, 범위가 맞는 사유, 낡았으면 카드 흐림 + 「낡음: {나이} 동안 새 상태 없음」, 직접 관측이 있으면 맨 아래에 직접 관측 줄.
- 직접 관측 줄(`_watch`, `render.py:316-324`): 「{직접 관측 | 직접 관측 낡음} · {나이} 전(받은 것이 없으면 「수신 없음」) · 배터리 {정수}%(없으면 「배터리 값 없음」) · {도킹 | 도킹 아님 | 도킹 모름}」, 낡았으면 경고 색, title = 「{namespace} 구독만」.

요구조자(`render.py:368-410`): 「요구조자 {n}명」, 네 단계 개수 칸(2열), 대상 줄: V 아이콘·짧은 ID·단계 낱말·규칙 ID, 위치 「(x, y) m」·「구조 {구조 낱말}」·접근 낱말·지원(지원 상태가 None·NONE 이 아니면 「지원 {낱말} {지원 로봇 ID}」)·범위가 맞는 사유·소실 후보(「소실 후보 {시각}」, title 「고정 웹캠 {카메라} 사건 {ID} — 후보일 뿐, 로봇 재확인으로 구출을 판정」)·보고(장부 없으면 「보고 장부 입력 없음」, 있으면 「보고 {n}건 {영수증 낱말}」), 그리고 경고·재확인 사유 목록. 대상이 없으면 「아직 보고된 요구조자가 없습니다.」.

사건 기록(`render.py:430-455`): 「사건 기록 {전체}건」(잘렸으면 「 (최근 {n}건 표시)」). 열 = 발생·종류·출처·대상·내용·장부 기록·화면 표시·지연. 내용: 보고면 「{확정 낱말}, {접근 낱말}, {영수증 낱말}」, 사건이면 「신뢰도 {0.00}」 + 메모. 행 속성 `data-ref`·`data-event-ns`·`data-received-ns`·`data-kind`·`data-clock`. 「화면 표시」·「지연」은 브라우저가 채운다. 진동 감지 종류는 빨강, 소실 후보는 초록.

명령·오류 기록(`render.py:468-497`): 기록이 None 이면 「기록 입력 없음 — --site-state 로 띄우면 운영자 명령의 접수·거절과 operations 오류가 보입니다.」. 열 = 시각·무엇·결과 · 사유. 명령 행: 「접수」(초록) 또는 「거절」(빨강, title = `result`), 사유 = 「{reply 낱말} {코드}」(낱말이 없으면 코드만), 대상이 있으면 「 대상 {짧은 ID}」. 오류 행: 「오류」, 코드(빨강) + `detail`. 없는 입력은 「{키} 입력 없음」, 행이 없으면 「아직 기록 없음」.

관문 데이터(`render.py:500-503`): 숨은 `div#gates` 의 `data-gates` = 명령 → {enabled, reason, args, choices} JSON.

명령 칸(`render.py:514-546`): 머리 「명령」 + 권한 안내(꺼짐이면 빨강). 승인 묶음(한 열, 큰 파랑 버튼), 임무 묶음(세 열), 정지 묶음(빨강, 전체 폭). 버튼 = 낱말 + 그 아래 작은 글씨 사유, `data-command`, 꺼짐이면 `disabled`. AUTO 행(자동 출동·자동 출발)은 버튼 대신 점선 파랑 상자 `p.cmd-auto`(글자 = 낱말, title = 사유, `data-command` 없음, `render.py:524-527`). 권한이 있을 때만 `data-allowed="1"` 과 `data-csrf`. 확인 칸(숨김): 「{명령} — 아래 내용으로 보냅니다」, 선택지가 있으면 「대상」 드롭다운(「{짧은 ID} {단계 낱말}」), 보낼 값 목록(대상·대상 revision·경로 revision·「{segment_index + 1}번째 구간 (segment_index {n})」, 값이 없으면 「보내는 값: 명령 종류와 운영자 아이디만」), 「보내기」·「취소」. 그 아래 이 페이지 기록 목록, 맨 아래 「위험할 때는 현장 비상 정지 장치를 쓰세요. 이 화면의 명령은 보조 수단이고, 받을지는 조정기가 정합니다.」.

웹캠 칸(`render.py:1170-1187`): 카메라가 없으면 「웹캠」 + 「웹캠 주소가 없습니다. 실행할 때 --camera cam_a=http://… 로 카메라 PC 의 영상 주소를 주세요.」. 있으면 카메라마다 `figure`(16:9 `<img src>`, `onerror` 가 즉시 끊김 표시, 안내 「영상이 들어오지 않습니다. 카메라 PC 의 영상 서버와 주소를 확인하세요.」, 캡션 = 카메라 ID + 상태 칸).

폴링 스크립트(`render.py:893-915`) 한 바퀴:
1. `GET fragment`(상대 경로, 캐시 안 함).
2. 401 이거나 재지정되었으면 인터벌을 멈추고 401 이면 `/login`, 아니면 재지정된 주소로 이동.
3. 2xx 가 아니면 본문 앞 200자를 붙여 「HTTP {n} {본문}」 오류.
4. 성공이면 `#live` 내용을 통째로 교체 → `stamp()` → `readGates()` → 갱신 정상 처리(`pollStatus` 참).
5. 오류면 갱신 끊김 처리(`pollStatus` 거짓 + 문구): 탭 제목 「[갱신 끊김] 구조 관제」, 모든 버튼 끔, 확인 칸이 열려 있으면 취소, `#poll` 빨강 「갱신 끊김: {문구}, {마지막 성공 뒤 나이} 전 값을 보는 중」.
- 갱신 정상 처리: 탭 제목은 띠의 `data-mode` 가 DISASTER 면 「[재난] 구조 관제」, 아니면 「구조 관제」. `#poll` 「실시간 갱신 HH:MM:SS」.
- 인터벌은 앞 요청이 끝나기를 기다리지 않는다(`render.py:1096`).

KPI-04(`render.py:814-866`): `stamp` 가 사건 행마다 처음 본 시각을 기억하고, 처음이면 기록 {ref_id, kind, clock_domain_id, event_ns, received_at_ns, rendered_ms, latency_s = (그린 ms − 사건 ns ÷ 1e6) ÷ 1000, live = 첫 stamp 뒤인가}를 쌓는다. 「화면 표시」 칸 = 그린 시각, 「지연」 칸 = 나이. 첫 stamp 뒤에 나타나 3 s 가 안 된 행은 노랑. 요약: 실시간 행이 없으면 「KPI-04: 페이지를 연 뒤 들어온 사건 없음」, 있으면 「KPI-04: 실시간 사건 {n}건 중 3초 이내 {m}건, 최대 {나이}」(모두 이내면 초록, 아니면 빨강). 「지연 기록 저장」 = 브라우저 안에서 CSV(`kpi04-{ISO 시각}.csv`, 머리 8열, 지연은 소수 셋째 자리) 내려받기, 서버로 아무것도 보내지 않음.

카메라 감시(`render.py:1141-1167`): 카메라 칸마다 `last`·`retried` = 로드 시각. 프레임 `load` 마다 `last` = 지금, 끊김 표시가 있으면 지움. 주기 max(100, min(500, stale ÷ 2)) ms 로 검사해 지금 − `last` ≥ stale 이면 끊김 표시(「영상 끊김」), 그리고 지금 − `retried` ≥ stale 이면 `retried` = 지금, `src` = 원래 주소 + (`?` 또는 `&`) + `_r={지금}` 으로 다시 연결.

실패 페이지(`render.py:1200-1211`): 503, 제목 「[화면 오류] 구조 관제」, 5 s 자동 새로고침, 「화면을 그릴 수 없습니다」, 코드별 확인할 것(STORAGE_ERROR·INVALID_INPUT·POLICY_MISMATCH 세 가지, 그 밖 「콘솔을 띄운 터미널의 로그를 확인하세요.」, `render.py:62-67`), 코드 + 원문, 「5초마다 다시 시도합니다. 로봇은 이 화면과 상관없이 동작하므로, 위험하면 현장 비상 정지 장치를 쓰세요.」.

로그인 페이지(`render.py:1234-1242`): 「구조 관제 로그인」, flash, 폼(숨은 csrf, 아이디, 비밀번호, 「로그인」).
계정 관리(`render.py:1245-1280`): 머리(「구조 관제」 + 사용자 메뉴), flash, 「계정 추가」 폼(아이디, 표시 이름, 이메일, 역할 기본 CUSTOMER, 비밀번호 8자 이상), 계정 표(아이디 · 수정 폼[표시 이름, 이메일, 역할, 상태, 새 비밀번호(선택), 저장] · 삭제 버튼(브라우저 확인 「삭제할까요?」, 본인 행은 「본인」) · 갱신 시각), 「관제 화면으로」 링크.

#### 7.11 코드 → 한국어 낱말 — `labels.py`

`label` 은 종류와 코드를 받는다(`labels.py:59-67`): 코드가 None 이면 None(화면 「—」), 표에 없으면 코드 그대로(새 코드를 숨기지 않음), 종류가 표에 없으면 프로그래밍 오류(KeyError).

| 종류 | 코드 → 낱말 |
| --- | --- |
| role | SCOUT 정찰, SUPPORT 지원, UNKNOWN 모름 |
| motion | MOVING 이동 중, STOPPING 정지 중, STOPPED 정지, STOP_UNCONFIRMED 정지 미확인, UNKNOWN 모름 |
| readiness | READY 준비됨, NOT_READY 준비 안 됨, UNKNOWN 모름 |
| rescue | NOT_CHECKED 확인 전, RECHECK_NEEDED 재확인 필요, RESCUE_CONFIRMED 구출 완료 |
| support | NONE 없음, ASSIGNED 배정, ARRIVED 도착, FAILED 실패 |
| approach | NOT_ATTEMPTED 접근 전, APPROACH_COMPLETE 접근 완료, UNREACHABLE 도달 불가, FAILED 접근 실패, CANCELED 접근 취소 |
| confirmation | CONFIRMED 확정, UNCONFIRMED 미확정 |
| receipt | RECEIVED 수신, PENDING_EVIDENCE 증거 대기, PERSISTED 저장 완료 |
| event | REPORT 구조 보고, DISASTER_SIGN 진동 감지, EXIT_PASS 출구 통과, TARGET_DISAPPEARED_CANDIDATE 대상 소실 후보 |
| mode | NORMAL 평시, DISASTER 재난, CLEARING 해제 중, MISSION_FINISHED 임무 종료 |
| phase | IDLE 대기, DISASTER 재난 감지, SEARCHING 수색 중, TARGET_FOUND 요구조자 발견, DISPATCH_APPROVED 출동 승인됨, SEGMENT_MOVING 구간 이동 중, AWAITING_CONTINUE 계속 승인 대기, ARRIVED 도착, CANDIDATE 구조 완료 후보, RECHECKING 재확인 중, RESCUE_CONFIRMED 구출 완료, RECHECK_NEEDED 재확인 필요, RETURNING 복귀 중, FINISHED 종료 |
| segment | IDLE 대기, EVALUATING 판단 중, AWAITING_APPROVAL 승인 대기, EXECUTING 이동 중, STOPPING 정지 중, STOPPED 정지, UNKNOWN 모름 |
| reply | NOT_AUTHORIZED 재난 모드가 아님, NOT_FOUND 대상이나 복귀 위치를 모름, PENDING_EVIDENCE 확정·경로·판정이 아직 없음, TASK_BUSY 이미 진행 중, INVALID_INPUT 화면의 값이 지금 상태와 다름, FRAME_REVISION_MISMATCH 지도 기준이 다름, NAV_NOT_READY 로봇 준비 안 됨 — 준비 뒤 다시 누르세요, ID_CONTENT_CONFLICT 같은 ID 에 다른 내용 |

근거 `labels.py:31-56`. 4단계 낱말은 `view.py:15-20`(미확정·미구조·지원 중·구출 완료).

### 8. 순서도

8.1 한 번의 폴링(주 흐름)

```mermaid
flowchart TD
  A["브라우저 1초 인터벌이 poll 실행"] --> B["GET /fragment 요청"]
  B --> C{"세션 계정이 유효한가?"}
  C -->|아니오| D["401 JSON - 브라우저는 폴링을 멈추고 로그인 화면으로"]
  C -->|예| E["current_view 시작 - 입력 원천 규칙 확인"]
  E --> F["load_site_state - 9개 읽기를 읽기 전용으로 차례로"]
  F --> I["build_view - 로봇, 대상, 사건, 임무, 기록, 출구 관측, 명령 관문"]
  I --> G{"읽기와 형식 검사가 모두 성공?"}
  G -->|아니오| H["503 text/plain 한 줄 - 브라우저는 갱신 끊김과 버튼 끔"]
  G -->|예| J{"지도 원천이 있는가?"}
  J -->|예| K["LiveMap.current 또는 고정 지도로 map_overlay - 로봇, 대상, 출구"]
  J -->|아니오| L["render_fragment - HTML 조각"]
  K --> L
  L --> M["브라우저가 live 영역 교체 후 stamp, readGates, pollStatus 정상"]
  M --> N["applyGates - 버튼 켜짐과 확인 칸 유지 판단"]
```

상자 ↔ 코드: A `render.py:1096` · B `render.py:895`, `app.py:298-302` · C `app.py:169-183`, `app.py:198-214` · D `app.py:210-211`, `render.py:896-902` · E `app.py:50-60` · F `sources.py:75-99` · I `view.py:692-790` · G `app.py:229-243` · H `app.py:243`, `render.py:903-914` · J `app.py:68-72` · K `mapimage.py:233-270`, `mapimage.py:288-325` · L `render.py:506-509` · M `render.py:908-911` · N `render.py:954-969`

8.2 명령 한 번

```mermaid
flowchart TD
  A["운영자가 켜진 버튼을 누름 - arm"] --> B["확인 칸 - 대상 고르기와 보낼 값 표시"]
  B --> C{"보내기를 눌렀는가?"}
  C -->|취소 또는 관문 바뀜| D["disarm - 확인 칸 닫음"]
  C -->|예| E["POST /commands 폼 한 번"]
  E --> F{"로그인, CSRF, ADMIN, operations 주소 통과?"}
  F -->|아니오| G["401, 400, 403, 503 JSON sent false"]
  F -->|예| H{"build_request 통과?"}
  H -->|아니오| I["400 JSON sent false"]
  H -->|예| J["post_json - operations 로 한 번 POST, 5초 대기"]
  J --> K{"명령 답 형식의 본문?"}
  K -->|예| L["200 JSON sent true - 접수 또는 거절"]
  K -->|아니오| M["502 JSON sent false - delivery 표시"]
  L --> N["브라우저 기록에 한 줄 남기고 disarm"]
  M --> N
  G --> N
  I --> N
```

상자 ↔ 코드: A `render.py:995-1011` · B `render.py:973-992` · C `render.py:1082-1089` · D `render.py:1013-1020`, `render.py:966-968` · E `render.py:1052-1068` · F `app.py:324-344` · G `app.py:334-344`, `render.py:1071` · H `commands.py:91-112` · I `app.py:345-350` · J `commands.py:115-136`, `commands.py:168-172` · K `commands.py:143-165` · L `commands.py:175`, `app.py:356` · M `commands.py:173-174`, `app.py:356` · N `render.py:1035-1048`, `render.py:1076-1078`

### 9. 오류·예외 처리

| 상황 | 동작 | 코드·문구 | 근거 |
| --- | --- | --- | --- |
| 비밀 환경 변수 없음·16자 미만 | 기동 안 함, 종료코드 2 | `CONFIG_MISSING` | `__main__.py:66-68`, `app.py:130-131` |
| Flask 없음 | 기동 안 함(다른 방식으로 대신 뜨지 않음) | `DEPENDENCY_MISSING` | `app.py:125-129` |
| 계정 DB 없음·다른 파일 | 기동 안 함 | `STORAGE_ERROR`·`POLICY_MISMATCH` | `accounts.py:160-177` |
| `--map` 못 읽음·형식 틀림 | 기동 안 함(지도를 지어내지 않음) | `STORAGE_ERROR`·`INVALID_INPUT` 「map: …」 | `mapimage.py:29-30`, `:149-153` |
| 카메라가 있는데 끊김 시간 없음 | 기동 안 함 | `CONFIG_MISSING` 「cameras need --camera-stale-s …」 | `render.py:1130-1131` |
| 현장 상태 DB 없음·잠김·역할 다름·schema 1.4 아님(1.0~1.3 파일은 새 db_writer 가 한 번 열어 올리기 전) | 그 요청 503(페이지는 안내 + 5 s 새로고침, 조각은 한 줄, API 는 JSON). 파일을 만들지 않음 | `STORAGE_ERROR` 「site-state read failed: …」 | `sources.py:96-99`, `app.py:229-243`, `rescue_operations/site_state.py:581-582` |
| 장부를 줬는데 못 읽음 | 그 요청 503 | `STORAGE_ERROR` 「ledger read failed: …」 | `sources.py:26-31` |
| 입력 모양이 틀림(중복 로봇·대상, 사건 계약 위반, revision 타입, 임무 행 키) | 그 요청 503 | `INVALID_INPUT` | `view.py:122-133`, `:202`, `:207`, `:250`, `:285-297`, `:355-358`, `:735` |
| 배치 밖 로봇 상태 | 버리고 경고 | 「배치 밖 로봇 상태 무시: …」 | `view.py:247-249` |
| `--robot-watch` 파일 없음·못 읽음·모양 다름 | 요청은 성공, 직접 관측 없이 그리고 경고 | 「로봇 직접 관측 파일 없음」·「로봇 직접 관측 파일 읽기 실패: …」 | `sources.py:57-72`, `view.py:668-670` |
| 직접 관측 파일이 `--robot-watch-stale-s` 넘게 안 바뀜 | 모든 직접 관측을 낡음으로, 위치를 쓰지 않음, 경고 | 「로봇 직접 관측 파일이 {n}초 동안 갱신 안 됨 — robot_watch 노드 확인」 | `view.py:671-674` |
| 배치 밖 로봇의 직접 관측 | 버리고 경고 | 「배치 밖 로봇 직접 관측 무시: '{ID}'」 | `view.py:676-677` |
| `--robot-watch-stale-s` 가 `nan`·0·음수 | 막지 않는다. `nan` 이면 크기 비교가 모두 거짓이라 **낡음이 영영 안 뜨고** 직접 관측 위치도 쓰지 않는다. 0·음수면 늘 낡음 [관찰, 코드 읽기, 미실행] | — | `__main__.py:51-52`(argparse float), `view.py:672`, `:684-685` |
| 터널 판정이 있는데 지금 지도(`current_map`)를 모름 | 모두 진하게 그리고 경고 | 「현재 지도 revision 모름 — 터널 판정을 모두 진하게 그림」 | `view.py:645-646` |
| 상태를 한 번도 못 받은 로봇 | 카드는 그리고 알림 | 「상태를 한 번도 받지 못했습니다. …」 | `render.py:346-349` |
| 로봇 상태가 5 s 넘게 안 옴 | 카드 흐림 + 알림(준비 판정 아님) | 「낡음: {나이} 동안 새 상태 없음」 | `view.py:280`, `render.py:352-353` |
| 세션 무효(삭제·재생성·비활성·비밀번호/역할 변경) | 세션 비움, 데이터 401·페이지 302 | `{"error": "login required"}` | `app.py:176-183`, `:210-212` |
| CSRF 틀림 | 400 | Flask 기본 400 또는 JSON `CSRF` | `app.py:191-196`, `:338-339` |
| 로그인 5회/300 s 실패 | 그 (아이디, 주소)를 60 s 거절(맞는 비밀번호도) | 「로그인을 너무 많이 틀렸습니다. {n}초 뒤에 다시 하세요.」 | `app.py:262-266`, `throttle.py:52-69` |
| CUSTOMER 가 명령·계정 관리 | 403 | `NOT_AUTHORIZED` / 「admin only」 | `app.py:340-344`, `:224-225` |
| operations 주소 없음 | 버튼 꺼짐, POST 는 503 | `NOT_CONNECTED` | `commands.py:72-73`, `app.py:343-344` |
| operations 연결 거부·이름 풀이 실패 | 보내지 않음, 502, 다시 안 보냄 | `UNREACHABLE`, delivery NOT_SENT | `commands.py:130-131` |
| operations 시간 초과·응답 모양 틀림·HTTP 오류 | 502, 다시 안 보냄, 「전달됐는지 모름 — command_id … 로 명령 기록을 확인」 | `TIMEOUT`·`BAD_RESPONSE`·`HTTP_ERROR`, delivery UNKNOWN | `commands.py:128-136`, `:159-164`, `render.py:1042-1046` |
| 계정 추가·수정·삭제 검사 실패(마지막 ADMIN, 본인 삭제, 중복, 형식) | 바꾸지 않음, flash | `LAST_ADMIN`·`SELF_DELETE`·`ALREADY_EXISTS`·`NOT_FOUND`·`INVALID_INPUT` | `accounts.py:239-240`, `:267`, `:291`, `:304` |
| 계정 DB 작업 중 sqlite 오류 | `STORAGE_ERROR` 로 바꾸려 하지만 비교가 정확한 타입 일치라 하위 오류(OperationalError 등)는 그대로 올라가 Flask 500 이 된다 [관찰, 코드 읽기] | — | `accounts.py:348-349` |
| 실시간 지도 아직 없음 | 지도 칸에 문구, 기동은 정상 | 「실시간 지도 없음 — live_map.yaml 를 아직 받지 못함」 | `mapimage.py:239-241` |
| 실시간 지도 새 파일 못 읽음 | 마지막 좋은 지도 유지 + 경고, 지도 흐림 | 「지도 읽기 실패: … — 마지막으로 읽은 지도를 보임」 | `mapimage.py:248-259` |
| 실시간 지도 발행 끊김 | 경고, 지도 흐림 | 「지도 낡음: {n}초 동안 지도 발행 없음」 | `mapimage.py:265-269` |
| 좌표 frame 이 `map` 이 아님·지도 밖·위치 없음 | 그리지 않고 경고·목록 | 「지도에 안 그림 …」·「지도 밖 좌표 …」·「위치 미확정」 | `app.py:73-76`, `mapimage.py:310-322` |
| 출구(QR) 관측이 다른 frame·지도 밖 | 표식 없이 경고와 「지도에 그리지 못한 위치」 목록(가장자리로 끌어오지 않음) | 「지도에 안 그림: exit 출구(QR) frame '{frame}' ≠ 지도 frame」 또는 「지도 밖 좌표: exit 출구(QR) ({x}, {y})」, 목록 「출구(QR): 좌표계 …」 또는 「출구(QR): 지도 밖 ({x}, {y}) m」 | `mapimage.py:304-323`, `app.py:73-76`, `render.py:268-269` |
| 출구(QR) 관측 없음(빈 표·픽스처 키 없음) | 표식 없음, 경고 없음 | — | `view.py:653-654` |
| 조정기 `search_start` 와 `--search-start` 가 다름 | 조정기 AUTO·콘솔 버튼: 버튼이 남고 누르면 NOT_AUTHORIZED(`AUTO_SEARCH_ENABLED`) 거절, 그 거절이나 임무 사유를 보고 경고. 콘솔 AUTO·조정기 OPERATOR: 「자동 출발」로 보이지만 수색이 시작되지 않고 화면은 알 수 없음 | 「조정기는 자동 수색(search_start AUTO) — 관제 웹을 --search-start AUTO 로 다시 띄우세요」 | `view.py:763-767`, `README:49` |
| 지도 없는데 `/map.png` | 404 | 「no map: start with --map MAP_YAML or --live-map LIVE_YAML」 | `app.py:318-319` |
| 웹캠 오류·멈춤 | 끊김 표시, stale 마다 재연결 | 「영상 끊김」 | `render.py:1182-1183`, `:1155-1163` |
| 폴링 실패(503, 네트워크) | 끊김 표시, 모든 버튼 끔, 열린 확인 취소, 탭 제목 | 「갱신 끊김: …」 | `render.py:875-888`, `:954-969` |
| 폴링이 1 s 넘게 걸림 | 앞 요청을 기다리지 않고 다음 요청이 나간다. 늦게 온 옛 조각이 새 조각을 덮을 수 있다(막는 장치 없음) [관찰, 코드 읽기] | — | `render.py:1096` |

### 10. 시험

- `tests/test_console_web.py::FlaskBoundaryTests.test_everything_needs_login_and_data_routes_stay_get_only` — 데이터 경로는 GET 뿐이고 익명은 페이지 302·데이터 401.
- `tests/test_console_web.py::FlaskBoundaryTests.test_old_sessions_die_with_the_account_they_came_from` — 삭제 뒤 재생성·비밀번호 변경·비활성화가 이전 쿠키를 서버에서 끊음(AST-50-01).
- `tests/test_console_web.py::FlaskBoundaryTests.test_broken_ledger_is_a_visible_failure` — 입력 실패가 페이지·조각·API 모두 503 이고 파일을 만들지 않음.
- `tests/test_console_commands.py::GateTests.test_dispatch_waits_for_the_report_receipt` — 출동 승인은 `persisted_report_id` 가 있는 대상만 고름.
- `tests/test_console_commands.py::CommandRouteTests.test_a_failed_send_is_shown_and_not_retried` — 전송 실패는 502 로 드러나고 다시 보내지 않음.
- `tests/test_console_live_map.py::FrozenMapTests.test_a_gone_publisher_is_stale` — 지도가 그대로여도 alive 가 멈추면 낡음.
- `tests/test_console_reasons.py::SearchStartModeTests.test_auto_shows_the_state_instead_of_the_button` — `--search-start AUTO` 면 START_SEARCH 가 「TB-6 자동 출발」 상태 행이고 페이지에 `data-command="START_SEARCH"` 버튼이 없음.
- `tests/test_console_reasons.py::SearchStartModeTests.test_a_coordinator_in_auto_against_a_button_console_is_said` — 콘솔이 AUTO 가 아닐 때 기록의 `AUTO_SEARCH_ENABLED` 거절이나 임무 사유가 「--search-start AUTO」 경고를 내고, AUTO 면 내지 않음.
- `tests/test_console_live_map.py::ExitMapTests.test_an_exit_off_the_map_is_listed_not_placed` — 지도 밖 출구 관측은 `exits` 에 없고 「출구(QR): 지도 밖」 목록에 있음.
- 모듈별 나머지: `test_console_accounts.py`(계정·해시·마지막 ADMIN), `test_console_throttle.py`(잠금), `test_console_view.py`(4단계·장부 없는 실행·배치 로봇·최신 출구 관측 읽기 `SiteStateSourceTests.test_the_newest_exit_qr_observation_is_read`), `test_console_history.py`, `test_console_reasons.py`(사유 문구·출동/수색 AUTO·#228/#234 코드), `test_console_live_map.py`(실시간 지도·출구 표식 `ExitMapTests`), `test_console_map.py`(출구 시험 없음), `test_console_labels.py`. Flask 가 없는 환경에서는 Flask 시험이 skip(= NOT_RUN)이다(`tests/test_console_web.py:310`).
- `tests/test_console_robot_watch.py` — 상태 없는 로봇은 직접 관측이 카드·위치를 채움, 프로젝트 상태가 먼저, 낡은 직접 관측은 위치를 안 그림, 파일이 멈추면 모두 낡음, 배치 밖 직접 관측은 경고, 파일 없음·깨짐은 경고(요청 성공), 인자가 없으면 꺼짐.
- `tests/test_console_tunnel_map.py` — 세 상태의 낱말·색, 옛 revision 흐림, `current_map` 모름은 모두 진하게 + 경고, 지도 밖 터널은 목록, 표가 없으면 아무것도 안 그림.
- 이 명세를 쓰면서 시험은 실행하지 않았다(미실행). 시험 실행 기록은 [관제 웹 코드 리뷰 준비](../process/console-code-review.md) 4절.
- E3/E4: 운영 PC 에서 장부 없이 기동 PASS, 로봇 카드 2개 PASS, MJPEG 중계와 `--camera` PASS [측정 2026-10-13, `docs/evidence/field-2026-10-13-operations-host.md:25-27`, `:162-181`]. Firefox 눈 확인·웹캠 가림 「영상 끊김」·진동 → 재난 모드는 [대기](같은 문서 `:28-30`). 콘솔 → operations → 조정기 명령 왕복은 기록 없음 = NOT_RUN(검색: `docs/evidence`·`docs/process` 에서 `operator-commands`, `operator_command`, `/commands` — 운영 PC 기록의 db_writer 기동 로그 한 줄 말고 없음).

## 웹이 기대는 인터페이스 (operations 쪽)

operations 노드의 내부는 다루지 않는다. 웹이 기대는 계약만 적는다.

### I-1. 운영자 명령 API — `POST /api/operator-commands`

- 주소: db_writer 프로세스 안의 HTTP 서버, `api_host` = `127.0.0.1`(운영 PC 안에서만), 포트 = launch `api_port`(`rescue_bringup/operations_launch.py:26`, `:73-74`; `rescue_operations/ros_node.py:248`). 이 API 에는 로그인이 없다. ADMIN 확인은 같은 PC 의 콘솔이 한다(`rescue_bringup/operations_launch.py:25`).
- 요청: POST, `Content-Length` 필수, 본문 0~8192 byte(`rescue_operations/command_api.py:25`, `:183-187`). 경로가 다르면 404. `Content-Type` 은 보지 않는다.
- JSON 은 객체이고 키 집합이 **정확히** 기본 3개 + 종류별 필드여야 한다(`rescue_operations/command_api.py:30-38`, `:51-73`):

| kind | 키 | 콘솔이 보내나 |
| --- | --- | --- |
| START_SEARCH | command_id, kind, operator_id | 예 |
| DISPATCH_SUPPORT | + target_id, target_revision | 예 |
| CONTINUE_SEGMENT | + target_id, target_revision, route_revision, segment_index | 예 |
| REQUEST_RECHECK | + target_id | 예 |
| RETURN_HOME | 기본만 | 예 |
| SET_CAMERA_WATCH | + target_id | 아니오(콘솔 표에 없음, `commands.py:26-32`) |

- 값 검사: `command_id`·`target_id` = 정규 UUID 문자열, `operator_id` = 1~256자·NUL 없음, `target_revision`·`route_revision` = 0~2^64−1 정수(bool 아님), `segment_index` = 0~2^32−1 정수.
- 응답(JSON, `Content-Type: application/json`, `rescue_operations/command_api.py:107-149`, `:179-189`):

| HTTP | 언제 | 본문 | 콘솔 `read_reply` 의 해석 | 화면 |
| --- | --- | --- | --- | --- |
| 200 | 조정기가 답함 | `accepted`(bool), `request_id`(= command_id), `reason`(접수면 조정기 코드 또는 null, 거절인데 코드가 없으면 `UNKNOWN`), `request_state`(ACCEPTED·REJECTED·DUPLICATE), `created_task_id`(빈 값은 null) | 답 | 「접수됨 (실행 완료 아님)」 또는 「거절 {코드} ({낱말})」 |
| 400 | operations 가 요청 자체를 거절(조정기에 보내지 않음) | `accepted: false`, `request_id`(못 읽으면 null), `reason: "INVALID_INPUT"`, `detail` | 답(거절) | 「거절 INVALID_INPUT (화면의 값이 지금 상태와 다름)」 |
| 404 | 다른 경로 | `error: "NOT_FOUND"` | BAD_RESPONSE | 보내지 못함 — 전달됐는지 모름 |
| 411, 413 | 길이 없음·숫자 아님 / 범위 밖 | `error: "INVALID_INPUT"`, `detail` | BAD_RESPONSE | 같음 |
| 503 | 조정기 서비스가 없음(보내지 않음) | `error: "ACCEPT_TIMEOUT"`, `request_id`, `detail` | BAD_RESPONSE(`accepted` 가 없으니) | 「전달됐는지 모름」(실제로는 안 보냈어도 보수적으로) |
| 504 | 보냈으나 `command_timeout_s` 안에 답 없음 | 같음 | BAD_RESPONSE | 「전달됐는지 모름」 |

- 시간: operations 는 `command_timeout_s` 가 0 초과 5.0 s 미만이어야 기동하고(`rescue_operations/ros_node.py:92-93`, `rescue_bringup/operations_launch.py:56-57`), 콘솔은 5.0 s 를 기다린다(`commands.py:22`). 그래서 조정기 무응답은 콘솔 시간 초과 전에 504 로 온다.
- 멱등: operations 는 같은 `command_id` 의 첫 수신 시각을 프로세스 동안 기억해 재전송을 DUPLICATE 로 만든다(`rescue_operations/command_api.py:103-113`). 콘솔은 누를 때마다 새 `command_id` 를 만들고 다시 보내지 않으므로 이 경로를 쓰지 않는다.
- 기록: operations 는 200 답과 400 거절(쓸 만한 command_id·kind·operator_id 가 있을 때, `result` = `REFUSED_BY_OPERATIONS`)을 `operator_commands` 에, 503/504 와 쓸 수 없는 400 을 `error_log`(코드 `ACCEPT_TIMEOUT` 또는 `INVALID_INPUT`)에 남긴다(`rescue_operations/command_api.py:116-164`). 콘솔의 「명령·오류 기록」 칸이 이것을 읽는다.

### I-2. 현장 상태 DB — 웹이 읽는 표와 열

- 파일: launch `db_path`(`launch/operations.launch.py:19`). 쓰는 쪽은 db_writer 하나. 쓰는 쪽 설정: `journal_mode=WAL`, `synchronous=FULL`, `busy_timeout=1000`, 쓰기마다 `BEGIN IMMEDIATE`(`rescue_operations/ledger.py:78-82`, `:104-115`). 그래서 읽는 쪽은 쓰는 쪽을 막지 않고 커밋된 행만 본다.
- 열기 규칙(`rescue_operations/site_state.py:572-592`, 경로 규칙 `rescue_operations/ledger.py:39-48`): 경로가 빈 값·`:memory:`·NUL 포함이면 거절, 절대 경로로 풀어 폴더가 있고 경로가 폴더가 아니어야 함 → `file:{절대 경로}?mode=ro` URI, 대기 1.0 s → `PRAGMA query_only = ON` → `metadata` 의 `role` = `operations-site-state` 이고 `schema` = `1.3` 이 아니면 `POLICY_MISMATCH` → `allowed_robot_ids` 가 JSON 이고 프로파일 로봇 ID 2개(서로 다름)가 아니면 `POLICY_MISMATCH` → 질의 → 결과에 `schema_version: "1.3"` 을 붙여 반환 → 늘 닫음. sqlite 오류는 `STORAGE_ERROR`. 파일을 만들지 않는다. 1.0·1.1·1.2 파일은 db_writer 가 한 번 열어 1.3 으로 올리기 전에는 읽히지 않는다(`rescue_operations/site_state.py:28-31`; 1.2 → 1.3 은 `exit_locations` 표만 더함).
- 그래서 콘솔과 db_writer 는 같은 판의 `rescue_operations` 를 써야 한다. 콘솔만 1.4 코드로 올리고 옛 db_writer 가 1.3 파일을 계속 쓰면, 또는 그 반대면, 모든 폴링이 `STORAGE_ERROR` 503 이다 [관찰, 코드 읽기].

| 표(읽기 함수) | 정렬·한도 | 웹이 쓰는 열·키 | 쓰지 않는 열·키 |
| --- | --- | --- | --- |
| `metadata`(`read_allowed_robot_ids`) | 정렬된 목록 | `allowed_robot_ids` → 로봇 카드 목록 | — |
| `robot_status`(`read_robot_status`) | robot_id 순 | `status_json` 의 robot_id, role(SCOUT·SUPPORT·UNKNOWN), stamp_ns, pose{x,y,yaw,frame_id} 또는 null, motion_state, readiness, battery_percent(0~100) 또는 null, battery_stamp_ns, active_task_id; 열 `received_at_ns` | robot_boot_id, state_sequence, generation, dock_state, segment_index |
| `site_events`(`read_site_events`, limit 1000) | `observed_at_ns`, `event_id` **오래된 순** — 1000건을 넘으면 최신이 빠진다 | `event_json`(SiteEvent 전 필드로 다시 만들어 계약 검사: event_id, camera_id, kind, observed_at_ns, evidence_id, evidence_sha256, confidence, clock_domain_id, stamp_semantics, clock_state, clock_error_bound_ns, detail, target_id, watch_id), 집계 `exit_pass_count`(EXIT_PASS 의 서로 다른 event_id 수) | 열 `received_at_ns`(돌려주지 않음) |
| `target_states`(`read_target_states`) | target_id 순 | `state_json` 의 target_id, position{x,y,frame_id,map_revision} 또는 null, detection_confirmed, rescue_state, rescue_reasons, support_state, support_robot_id, target_revision(선택 키), persisted_report_id(선택 키) | verdict_id, updated_at_ns, position.map_session_id |
| `mission_state`(`read_mission_state`) | 1행 또는 null | mode, phase, trigger_event_id, started_at_ns, updated_at_ns, reason, target_id, target_revision, route_revision, segment_index, segment_count, segment_phase | generation |
| `operator_commands`(`read_operator_commands`, limit 30) | `requested_at_ns`, `command_id` 최신 순 | command_id, kind, operator_id, target_id, requested_at_ns, accepted(bool 로), result, reason | robot_id, segment_index, route_revision |
| `error_log`(`read_error_log`, limit 30) | `at_ns`, `seq` 최신 순 | seq, at_ns, code, detail | robot_id, task_id |
| `exit_locations`(`read_exit_locations`, limit 1, 1.3) | `observed_at_ns` 최신 순, 같으면 `location_id` 오름차순 — 맨 위 1행만 | location_id, x, y, mission_frame(→ 화면 `exit.frame_id`), observed_at_ns, map_revision(마우스 설명에만) | received_at_ns, map_session_id, transform_revision, class_label(늘 `exit_qr`) |
| `overhead_passage_lists` + `overhead_passages`(`read_overhead_passages`, 1.4) | 로봇 ID 순, 로봇마다 항목은 `last_seen_ns` 최신 순·같으면 `id` | 목록의 robot_id, map_frame(→ 행 `frame_id`); 항목의 id, state(PASSABLE·BLOCKED·UNKNOWN), lowest_m(null 가능), x, y, map_session_id, map_revision, last_seen_ns | 목록의 generated_at_ns, received_at_ns, clearance_m, floor_noise_m, lidar_plane_m; 항목의 frames, first_seen_ns, reason |
| `current_map`(같은 읽기, 1.4) | 1행 또는 null | map_session_id, map_revision(→ 터널 행 `current`) | robot_id, generated_at_ns, received_at_ns |
| `map_snapshots` | — | 읽지 않음 | 전부 |

- `exit_locations` 는 db_writer 가 `/rescue/exit_location` 의 `projection_status` VALID 관측만 넣고(아니면 거절, `rescue_operations/ros_ingest.py:147-163`), 같은 `location_id` 를 다시 받으면 처음 행을 지킨다(`rescue_operations/site_state.py:489-504`). 조정기가 등록·재기준한 출구가 아니라 인식의 관측 원본이다. 지도 고정 전·후 관측을 웹은 구분하지 않는다(`README:168`).

- 터널 표: db_writer 가 `/rescue/robots/<robot_id>/overhead_passages`(`std_msgs/String` JSON)를 받아 로봇마다 **목록 통째로** 바꾼다. `generated_at_ns` 가 더 새 목록이면 그 로봇의 항목을 지우고 다시 쓰고, 같거나 옛 목록은 버린다(`rescue_operations/site_state.py:506-527`). `current_map` 은 SLAM 로봇(SCOUT)의 `/rescue/robots/<robot_id>/map_state`(`MapState`)에서 같은 규칙으로 1행(`rescue_operations/site_state.py:529-544`). 판정 자체는 인식 `overhead_passage` 노드 몫이고 웹은 다시 판정하지 않는다.

근거: 표 정의 `rescue_operations/site_state.py:95-138`(`exit_locations` 는 `:122-125`, 터널 표는 `:126-137`), 읽기 함수 `rescue_operations/site_state.py:595-677`, 키 정의 `rescue_operations/site_state.py:55-92`. 웹이 기대는 값 범위: `mission_state.mode` ∈ {NORMAL, DISASTER, CLEARING, MISSION_FINISHED}, `phase` ∈ 14개(7.11 phase 줄), `segment_phase` ∈ 7개(segment 줄), 출동 6칸은 모두 있거나 모두 null(`rescue_operations/site_state.py:38-47`, `:268-293`). 웹은 이 범위를 다시 검사하지 않고 낱말 표에 없으면 코드 그대로 보인다.

### I-3. 실시간 지도 파일 — `live_map.yaml`

쓰는 쪽은 map_snapshot(`launch/operations.launch.py:71-75`, 인자 `map_topic`·`map_dir` 는 둘 다 있거나 둘 다 없음, `rescue_bringup/operations_launch.py:198-210`).

| 항목 | 계약 | 쓰는 쪽 근거 | 웹 쪽 근거 |
| --- | --- | --- | --- |
| 위치 | `{map_dir}/live_map.yaml`, 같은 폴더의 `map_{PGM sha256 앞 12자}.pgm`, `live_map.alive` | `rescue_operations/map_snapshot.py:22-23`, `:79` | `mapimage.py:22`, `:179-181` |
| YAML 키 | 정확히 7개: `image`, `mode: trinary`, `resolution`(양의 유한 실수), `origin: [x, y, 0.0]`, `negate: 0`, `occupied_thresh: 0.65`, `free_thresh: 0.25`. 평평한 한 줄 `키: 값` | `rescue_operations/map_snapshot.py:84-86` | `mapimage.py:24-25`, `:49-77` |
| 회전 | 없음. 쓰는 쪽이 yaw 절댓값 1e-6 초과 지도를 버리고 늘 0.0 을 씀, 웹은 0.0 이 아니면 거절 | `rescue_operations/map_snapshot.py:27`, `:71-73` | `mapimage.py:168-170` |
| PGM | `P5`, 머리 「P5 줄바꿈 폭 높이 줄바꿈 255 줄바꿈」, 위 행부터(격자 0행 = 아래를 뒤집음), 값: 점유 ≥ 65 → 0, 0 ≤ 값 ≤ 25 → 254, 그 밖·미지(음수) → 205 | `rescue_operations/map_snapshot.py:31-42` | `mapimage.py:86-121`(maxval 255 라 비례 변환 없음) |
| 원자성·순서 | PGM(없을 때만) → YAML 순으로, 각각 임시 파일에 쓰고 rename. 지금·직전 PGM 만 남기고 지움. 그래서 YAML 이 없는 PGM 을 가리키지 않는다 | `rescue_operations/map_snapshot.py:49-52`, `:77-90` | `mapimage.py:233-252`(mtime·크기가 바뀌면 다시 읽음) |
| 받은 시각 | YAML mtime(같은 지도를 다시 받아도 YAML 은 다시 써서 mtime 이 바뀜) | `rescue_operations/map_snapshot.py:4-6`, `:84` | `mapimage.py:253` |
| 살아 있음 | 지도 토픽에 발행자가 있는 동안 2.0 s 마다 `live_map.alive` 의 mtime 갱신. 웹의 낡음 기준은 이보다 길어야 함 | `rescue_operations/map_snapshot.py:24`, `:141-148` | `mapimage.py:260-269` |
| frame | YAML 에 frame 이 없다. 웹은 이 지도를 frame `map` 으로 보고, 로봇·대상 좌표의 `frame_id` 가 `map` 일 때만 그린다 | (OccupancyGrid frame 은 로그만, `rescue_operations/map_snapshot.py:134-135`) | `mapimage.py:20`, `:200`, `:317` |
| 시계 | mtime 은 운영 PC 파일 시스템 시각, 웹은 자기 `time.time_ns` 와 비교 — 같은 PC 를 전제 [추정] | — | `mapimage.py:254`, `:266` |

### I-4. 그 밖

- 옛 보고 장부(선택 `LEDGER`): `read_reports(limit=1000)`(`rescue_operations/ledger.py:350-380`), `metadata.role` = `operations-ledger`, `schema` = `1.0`. 웹이 쓰는 것: 최상위 `evidence_policy`·`allowed_robot_ids`, 보고마다 `payload` 의 report_id, target_id, source_robot, reported_at_ns, observed_at_ns, confirmation_result, approach_result, position{x,y,frame_id,map_revision}, clock_domain_id, `receipt.status`, `received_at_ns`. report_node 의 장부(`metadata.role` = `rescue-report-store`, `rescue_operations/report_store.py:29`)는 이 자리에 줄 수 없다(`POLICY_MISMATCH` → 화면 `STORAGE_ERROR`).
- 웹캠 MJPEG: `http://{video_bind_host}:{video_port}/{camera_id}.mjpg`(`rescue_bringup/operations_launch.py:170`, `:213-215`). 브라우저가 직접 붙으므로 `video_bind_host` 는 브라우저 PC 에서 닿는 주소여야 한다(`launch/operations.launch.py:101-103`).

### I-5. 로봇 직접 관측 파일 — `--robot-watch`

쓰는 쪽은 `rescue_robot_watch`(rescue_operations `robot_watch`, `launch/operations.launch.py:79-81`, 인자 `robot_namespaces`·`robot_watch_path` 는 둘 다 있거나 둘 다 없음, `rescue_bringup/operations_launch.py:178-195`). 로봇 기본 토픽 `<ns>/battery_state`·`<ns>/dock_status`·`<ns>/tf`·`<ns>/tf_static` 을 **구독만** 한다(발행·서비스·액션 없음, `rescue_operations/robot_watch.py:103-106`).

| 항목 | 계약 | 쓰는 쪽 근거 | 웹 쪽 근거 |
| --- | --- | --- | --- |
| 형식 | UTF-8 JSON 객체 `{schema: "robot_watch/1", written_at_ns: int, robots: {robot_id: 행}}` | `rescue_operations/robot_watch.py:18`, `:35-40` | `sources.py:69-71`(robots 객체·`written_at_ns` int 만 검사, `schema` 는 보지 않음) |
| 로봇 행 | `namespace`, `last_seen_ns`(네 토픽 중 아무거나 받은 마지막 시각) 또는 null, `battery_percent`(0~100, NaN·범위 밖은 null), `docked`(bool 또는 null), `pose`{x, y, yaw, frame_id = map_frame} 또는 null, `pose_seen_ns` 또는 null | `rescue_operations/robot_watch.py:82-83`, `:24-28`, `:113-119` | `view.py:679-687` |
| 위치 | tf 에 `map_frame -> base_frame`(launch 값 `map` → `base_link`)이 있을 때만. 없으면 null 이고 지어내지 않음 | `rescue_operations/robot_watch.py:110-119`, `rescue_bringup/operations_launch.py:194-195` | `view.py:685` |
| 주기·원자성 | 1.0 s 마다 임시 파일에 쓰고 rename | `rescue_operations/robot_watch.py:19`, `:35-40`, `:127` | `sources.py:58-59`(반쯤 쓴 파일을 보지 않음) |
| 시계 | 모든 ns 는 관측 노드의 ROS 시계(운영 PC). 웹은 자기 `time.time_ns` 와 비교 — 같은 PC 를 전제 [추정] | `rescue_operations/robot_watch.py:87-88`, `:109` | `view.py:671`, `:682-683` |

## 문서와 코드의 차이 (bd4bfa5 에서 찾고 32e8ee1 에서 다시 봄)

「고침(이 PR)」은 이 브랜치에서 문서를 고쳐 차이가 없어진 줄이고, 그 줄의 「문서 내용」은 고치기 전 글이다.

| 문서:줄 | 문서 내용 | 코드 | 비고 |
| --- | --- | --- | --- |
| `docs/design/console-and-db-plan.md:13`, `:20` | 콘솔 버튼은 비활성 placeholder, operations POST 미연결 | 버튼이 켜지고 operations API 로 보냄(`view.py:530-623`, `app.py:324-356`, `rescue_operations/command_api.py`) | 문서가 낡음(updated 2026-10-11) |
| `docs/design/console-and-db-plan.md:66`, `:69` | 웹은 operations `GET /api/snapshot` 으로 읽고, 지도는 `PUT /api/maps/{id}` | 웹이 현장 상태 DB 를 직접 읽기 전용으로 읽고(`sources.py:75-99`), 지도는 map_snapshot 파일(`mapimage.py:210-270`). 두 API 는 없음 | 계획 [제안] 과 구현이 다름 |
| `docs/design/console-and-db-plan.md:82`, `:85` | 웹캠은 화면 위, 버튼에 일시정지·재개·취소·정지 | 웹캠은 오른쪽 옆 칸(`render.py:572-575`), 넷은 늘 꺼짐(`view.py:58-63`) | — |
| `docs/process/ops-db-tables.md:41`, `:129-131` | 현장 상태는 표 3개, 1.1 표 넷은 아직 코드에 없음 | schema 1.3, `metadata` 말고 표 8개(`rescue_operations/site_state.py:28`, `:95-138`) | 문서 updated 2026-10-10 로 낡음 |
| `docs/design/db-schema.md:11-16`, `docs/process/ops-db-tables.md:37-42` | SQLite 역할 4가지 | report_node 의 `rescue-report-store` 파일이 다섯째(`rescue_operations/report_store.py:29`) | 콘솔은 그 파일을 읽지 않음 |
| `docs/design/db-schema.md:20` | `metadata(key TEXT PRIMARY KEY, value TEXT)` | `value TEXT NOT NULL`(`rescue_operations/ledger.py:84`, `accounts.py:140`) | 사소 |
| `README:14` | 현장 상태 「읽기 함수 넷」 | 읽기 함수 8개(배치 로봇·명령·오류·출구 포함, `sources.py:84-95`) | **고침(이 PR)**: 「여덟」(merge 로 들어온 `read_exit_locations`, `sources.py:93` 포함) |
| `README:76-84`, `:86`, `:88` | 입력 표 5행, 「픽스처의 네 칸」, 「네 읽기는 한 트랜잭션이 아니다」 | 읽기 9개(`sources.py:84-95`), 픽스처 키 9개(`sources.py:22-23`) | 그대로 |
| `README:14`(2026-10-13 main `ede7f8b`) | 현장 상태 「읽기 함수 여덟」 | 읽기 함수 9개(터널 판정 `read_overhead_passages`, `sources.py:94` 가 더해짐) | **고침(이 PR)**: 「아홉」 |
| `README:193` | 갱신 실패 문구 「갱신 실패 — 표시 값이 낡았을 수 있음」 | 「갱신 끊김: {이유}, {n} 전 값을 보는 중」(`render.py:885-886`) | **고침(이 PR)** |
| `README:94` | 가로 1400px **또는 세로 820px** 미만이면 한 줄로 쌓임 | 쌓임은 폭 1399px 이하만(`render.py:577-580`), 높이는 고정 여부만 정함(`render.py:576`) | **고침(이 PR)** |
| `src/rescue_console/setup.py:20` | 「Read-only … view …; no ROS, no commands.」 | 명령을 보냄(`app.py:324-356`) | **고침(이 PR)**: 「sends operator commands to operations once」 |
| `docs/process/ops-host-record.md:54` | 콘솔이 기동 때 git SHA 를 로그에 남김 | 콘솔 코드에 그런 줄이 없음(검색은 6.5 끝). 서비스 래퍼가 하는지는 [미확인] | AGENTS 8절 「기동 로그에 git SHA」 |
| `labels.py:8-14` | motion·readiness 는 「no producer yet」 | operations 가 RobotStatus 에서 씀(`rescue_operations/ros_ingest.py:70-87`) | **고침(이 PR)**: 「copied by operations from RobotStatus」 |

# rescue_console — 관제 화면

운영 장부와 상태 스냅샷을 **읽어서** 한 화면에 보여 주고, 운영자 명령을 operations 로 넘긴다(BRD Part I 3.7.5·3.7.6). 2026-10-08 첫 판, 담당 자리: 관제·화면.

- 데이터의 진실은 협업·관제의 운영 장부(`rescue_operations`)다. 이 패키지는 운영 DB 에 쓰지 않고, DB 파일을 만들지 않고, ROS 를 구독하지 않고, 로봇에 직접 명령하지 않는다. 운영자 명령은 operations HTTP API(`POST /api/operator-commands`)로만 넘기고, 받을지는 조정기가 정한다(아래 「운영자 명령」).
- **현장 정지 수단을 웹 연결에만 의존시키지 않는다**(팀 `AGENTS.md` 7절). 이 화면이 꺼지거나 멈춰도 로봇 쪽 정지(통신 단절 watchdog, 현장 비상 정지)는 따로 동작해야 하고, 화면은 그 대체물이 아니다.

## 구성

| 모듈 | 하는 일 | 의존 |
| --- | --- | --- |
| `view.py` | 장부 + 상태 → 화면 모델(dict). 4단계 합성 규칙표, 임무 상태 배지, 명령 버튼의 관문 거울 | 표준 라이브러리, `rescue_contracts` |
| `commands.py` | 운영자 명령 요청 만들기·한 번 보내기·응답 읽기(`POST /api/operator-commands`) | 표준 라이브러리 |
| `sources.py` | `rescue_operations.ledger.read_reports` 로 장부를, `rescue_operations.site_state` 읽기 함수 넷으로 현장 상태 DB 를 읽기 전용(`mode=ro`, `query_only`)으로 읽음. 현장 상태 DB 대신 픽스처 JSON 도 받음(E2) | `rescue_operations` |
| `render.py` | 화면 모델 → HTML 한 장(CSS·JS 포함). Flask 없이 시험 | 표준 라이브러리 |
| `labels.py` | 화면에 나오는 코드(`MOVING`, `NOT_CHECKED` …) → 한국어 낱말. 모르는 코드는 그대로 보임 | 없음 |
| `mapimage.py` | 지도 YAML + PGM → PNG, map frame 좌표 → 지도 픽셀, 지도 위 표식 목록 | 표준 라이브러리 |
| `app.py` | Flask 경계. 데이터 경로는 `GET /`, `GET /fragment`, `GET /api/view`, `GET /map.png`, 명령은 `POST /commands` 하나(로그인·계정 폼은 아래 절) | Flask (선택) |

Flask 는 선택 의존이다(`extras_require={"web": ["flask"]}`). 없으면 `create_app` 이 `DEPENDENCY_MISSING` 오류로 끝나고 다른 방식으로 대신 뜨지 않는다. 집계·HTML·명령 요청·전송 시험은 Flask 없이 돈다.

## 실행 (로봇 없이, E2)

```bash
python3 -m pip install flask                      # 웹으로 볼 때만
export PYTHONPATH=$(ls -d src/rescue_* | tr '\n' ':')
python3 -m rescue_bringup.offline_demo config/deployment.example.json --output <TMP>/run
# 계정 파일은 한 번만 만든다(관리자 1명). 비밀번호는 환경 변수 또는 프롬프트
RESCUE_CONSOLE_ADMIN_PASSWORD='<8자 이상>' python3 -m rescue_console.accounts init <TMP>/accounts.sqlite3 --admin admin
# 세션 서명 비밀은 환경 변수(16자 이상). 코드·저장소에 두지 않는다
export RESCUE_CONSOLE_SECRET="$(python3 -c 'import secrets;print(secrets.token_urlsafe(32))')"
python3 -m rescue_console <TMP>/run/operations.sqlite3 --accounts <TMP>/accounts.sqlite3 \
    --status src/rescue_console/examples/status.example.json \
    --map assets/maps/pre_quake/map.yaml --port 8080       # --map 은 선택
# 브라우저: http://127.0.0.1:8080/  → 로그인 페이지
```

operations 가 현장 상태 DB 를 쓰고 있으면 픽스처 대신 그 파일을 준다. 둘을 같이 주면 시작하지 않는다.

```bash
python3 -m rescue_console [<운영 장부>] --accounts <계정 DB> --site-state <현장 상태 DB> \
    --camera cam_a=http://<카메라 PC>:<포트>/<경로> --map assets/maps/pre_quake/map.yaml \
    --operations http://<operations 호스트>:<포트> --port 8080
```

- `<운영 장부>`(옛 `ReportLedger`)는 `--site-state` 와 함께면 생략할 수 있다(2026-10-13). 지금 그 파일을 쓰는 운영 코드가 없고, #137 `report_node` 는 다른 형식의 별도 장부를 쓴다. 생략하면 「확인할 것」 줄과 각 대상 줄에 「보고 장부 입력 없음」이 보이고, 구조 보고 행·건수는 그리지 않는다(빈 목록으로 보이지 않는다). 로봇 카드는 현장 상태 DB 가 정한 배치 로봇 목록(`read_allowed_robot_ids`, #149)으로 그린다. 상태를 아직 안 보낸 로봇도 카드가 있다. 그 목록이 없는 입력(픽스처)에서만 상태가 들어온 로봇만 그리고, 그 사실을 「확인할 것」 줄에 적는다. 출동 승인 버튼은 장부가 아니라 대상 상태의 `persisted_report_id` 로 켜지므로 그대로 동작한다. 장부 경로를 줬는데 못 열면 지금처럼 오류다. 픽스처(`--status`)로 띄울 때는 장부가 필요하다.
- `--robot-name ROBOT_ID=NAME` 은 로봇마다 줄 수 있다(예: `--robot-name robot_a=TB-6 --robot-name robot_b=TB-5`, 재범 결정 2026-10-13). 기본값이 없고, 주지 않으면 로봇 ID 를 그대로 보인다. 이름은 역할에 묶지 않는다. 출동 승인 버튼과 명령 기록에는 상태가 SUPPORT 인 로봇의 이름이 들어간다(예: 「TB-5 출동 승인」). 그 로봇의 상태가 없으면 「지원 로봇 출동 승인」이다.
- `--camera ID=URL` 은 여러 번 줄 수 있다. URL 은 `http`·`https` 만 받는다.
- `--operations URL` 은 operations HTTP API 주소다(`http`·`https`, 기본값 없음). 명령은 `URL/api/operator-commands` 로 간다. 주지 않으면 명령 버튼이 「operations 주소 없음」 사유로 꺼지고, 기동할 때 터미널에 「operations 주소 없음: 명령 비활성」을 한 줄 남긴다. 보기 전용 실행(E2 픽스처, 오프라인 데모)은 그대로 돈다.

- 장부 경로·계정 파일·상태 경로·포트·바인드 주소는 인자로 받는다. 기본 바인드는 `127.0.0.1` 이고, 다른 PC 에서 볼 주소는 배치 프로파일(`operations_host`, 작전)이 정한 값을 인자로 넘긴다. 코드에 호스트·IP·개인 경로·비밀을 두지 않는다.
- 계정 파일과 세션 비밀은 저장소 밖에 둔다([SDD 10절 외부 파일 목록](../../docs/design/architecture.md)에 추가 예정).
- `examples/status.example.json` 은 예시 값이다(오프라인 리허설 장부의 대상 ID 에 맞춤). 실제 값이 아니다.

## 로그인과 계정 (강사 day5 요구: 로그인 페이지 + SQLite 로그인 데이터)

| 역할 | 할 수 있는 것 |
| --- | --- |
| `ADMIN` | 관제 화면, 운영자 명령, 계정 관리(`/admin/accounts`: CUSTOMER·ADMIN 추가, 표시 이름·이메일·역할·상태·비밀번호 수정, 삭제) |
| `CUSTOMER` | 관제 화면 보기만. 명령 버튼은 꺼져 있고 `POST /commands` 는 403, 계정 관리 페이지도 403 |

- 모든 페이지는 로그인이 필요하다. 미로그인은 `/login` 으로, `/api/*` 는 401.
- 비밀번호는 PBKDF2-HMAC-SHA256(소금 16바이트, 200,000회) 해시로만 저장한다. 평문은 저장·로그하지 않는다. 틀린 아이디·틀린 비밀번호·비활성 계정은 같은 문구로 거절한다.
- 마지막 `ACTIVE` `ADMIN` 은 비활성화·강등·삭제할 수 없고, 로그인한 본인 계정은 삭제할 수 없다.
- 세션 쿠키는 계정 ID 와 인증 버전을 담는다. 계정을 지우고 같은 이름을 다시 만들면 ID 가 달라지고, 비밀번호·역할·상태를 바꾸면 인증 버전이 올라가서, 그 전에 발급된 쿠키는 서버에서 전부 거절된다(API·조각은 401, 페이지는 로그인으로). 폴링 중 401 을 받으면 화면이 폴링을 멈추고 로그인 페이지로 간다.
- 로그아웃은 POST + CSRF 다. 링크·이미지 태그로는 로그아웃되지 않는다.
- 폼은 세션별 CSRF 토큰을 요구한다(없으면 400). 세션 쿠키는 HttpOnly·SameSite=Lax.
- **반복 실패 잠금:** 같은 아이디를 같은 접속지에서 5분 안에 5번 틀리면 60초 동안 그 아이디·접속지의 로그인을 거절한다(맞는 비밀번호도). 화면에는 「n초 뒤에 다시 하세요」가 뜬다. 기록은 콘솔 프로세스 메모리에만 있고(재시작하면 사라짐) 계정 DB 에 쓰지 않는다. 다른 아이디·다른 접속지는 영향이 없다. 값은 `throttle.py` 상단 상수([제안] 2026-10-10). 기록은 스레드 잠금(`threading.Lock`) 안에서만 바뀐다(개발 서버가 요청을 스레드로 처리, OPS-51-01). 한계: 잠금 확인과 비밀번호 확인 사이에 동시에 들어온 요청 묶음은 잠금이 걸리기 전에 모두 평가된다. 그 묶음의 크기는 요청마다 드는 PBKDF2 20만 회 비용이 묶는다.
- **같은 비용:** 없는 아이디·비활성 계정도 비밀번호 해시 확인을 한 번 거친다. 응답 시간으로 어떤 아이디가 있는지 알 수 없다.
- **응답 헤더:** 모든 응답에 `X-Content-Type-Options: nosniff`, `X-Frame-Options: DENY`, `Referrer-Policy: no-referrer`, `Content-Security-Policy: frame-ancestors 'none'; base-uri 'none'; form-action 'self'; object-src 'none'` 을 붙인다(`app.SECURITY_HEADERS`). 페이지·조각·API 는 `Cache-Control: no-store` 이고, `/map.png` 만 자기 캐시 정책을 쓴다. 스크립트까지 막는 CSP 는 인라인 스크립트·스타일에 nonce 를 붙인 뒤에 한다(후속).
- 계정 DB 는 콘솔 소유 파일이다. 운영 장부(보고·site_state)의 단일 쓰기 규칙과 무관하며 콘솔은 그쪽에 여전히 쓰지 않는다.

## 입력

| 입력 | `--site-state` (operations 가 쓰는 현장 상태 DB) | `--status` (E2 픽스처) |
| --- | --- | --- |
| 보고 장부 | `read_reports(db, limit=1000)` | 같음 |
| 로봇 상태 | `rescue_operations.site_state.read_robot_status(site_db)` | 픽스처의 `robot_status` |
| 현장 사건 | `read_site_events(site_db, limit=1000)` | 픽스처의 `site_events` |
| 대상 상태(구조·지원) | `read_target_states(site_db)` | 픽스처의 `target_states` |
| 임무 상태(모드·단계·출동 구간) | `read_mission_state(site_db)` (#111) | 픽스처의 `mission_state` |

픽스처의 네 칸은 위 함수의 반환 형식 그대로다(협업·관제와 2026-10-08 합의). 칸이 없으면 기본값으로 채우지 않고 화면 경고 「<칸> 입력 없음」으로 보인다.

- 네 읽기는 한 트랜잭션이 아니다. 그 사이에 쓰인 행은 한 번의 폴링에서 한 표에만 먼저 보일 수 있다(다음 폴링에서 맞춰짐).
- `read_site_events` 는 오래된 순으로 `limit` 개를 준다. 콘솔은 최대(1000)를 요청하므로 현장 사건이 1000건을 넘으면 최신 사건이 빠진다(후속: 협업·관제에 최신순 또는 커서 요청).
- 출구 누적(`exit_pass_count`)은 화면에 그리지 않는다. 고정 카메라 1대 결정(2026-10-09)으로 `EXIT_PASS` 생산자가 없어 항상 0 이다([DB 스키마](../../docs/design/db-schema.md) 2절, [관제 웹·DB 계획](../../docs/design/console-and-db-plan.md) 3절). `/api/view` 에는 값이 그대로 남는다. 「구출 완료 ≤ 출구 누적」(Part I 3.5.5) 경고도 같이 뺐다 — 구출 판정의 트리거는 카메라의 대상 소실 후보다(#48).

## 화면 배치 (1920×1080 한 화면)

[관제 웹·DB 계획](../../docs/design/console-and-db-plan.md) 5절의 자리 배치를 따른다. 가로 1400px·세로 820px 이상이면 칸 배치가 고정되고 페이지 전체는 스크롤되지 않는다. 1920×1080 모니터의 실제 브라우저 창(탭바를 뺀 약 1853×963)에서 모든 칸이 한 화면에 보인다. 창이 더 낮으면 로봇 카드 칸이 먼저 줄어 그 안에서 스크롤되고, 요구조자 칸은 목록 두 줄이 보이는 높이(240px)를 지킨다. 사건 기록 줄은 내용만큼(최대 230px) 차지한다(#64). 가로 1400px 또는 세로 820px 미만이면 한 줄로 쌓여 페이지가 스크롤된다.

```text
┌ 재난 띠: 모드 · 트리거 · 경과 · 단계 · 구간 · 사유 ──────────┬ 갱신 상태 · 사용자 ┐
├ (경고가 있을 때만) 확인할 것 ────────────────────────────────┼ 웹캠              │
├ 지도: 기준 지도 + 1m 격자 + 로봇·요구조자 ─┬ 로봇 TB-6·TB-5 ───┤                   │
│                                          │ 요구조자 목록       ├ 명령              │
├ 사건 기록 (KPI-04 열 포함) ────────────────┼ 명령·오류 기록 ─────┤ (승인 · 임무 · 정지)│
└───────────────────────────────────────────────────────────────┴──────────────────┘
```

- **오프라인:** 현장 네트워크에 인터넷이 없을 수 있어 외부 CDN·웹 글꼴·외부 CSS·JS 를 쓰지 않는다(시험 `test_page_loads_nothing_from_outside`). 글꼴은 운영 PC 에 깔린 Noto Sans CJK KR 을 먼저 찾고, 없으면 OS 기본 한글 글꼴로 넘어간다. 숫자는 고정폭(`tabular-nums`)이라 1초마다 바뀌어도 흔들리지 않는다.
- **색은 의미에만 쓴다(국제 안전표지 ISO 7010 계열):**

  | 색 | 뜻 |
  | --- | --- |
  | 신호 빨강 | 재난·정지 |
  | 구조 주황 | 미구조 요구조자 |
  | 안내 파랑 | 지원 중·지원 로봇(SUPPORT) |
  | 안전 초록 | 구출 완료 |
  | 회색 점선 | 미확정 |
  | 경고 노랑 | 낡음·끊김·확인할 것 |

  바탕은 차가운 회색이고 프로젝터에서 읽히도록 밝게 둔다.
- **요구조자 표식:** 지진 구조대(INSARAG)가 현장에 칠하는 「V」 모양이다. V 의 꼭짓점이 보고 좌표다. 구출 완료는 V 에 가로줄을 긋는다. 지도·목록·범례가 같은 모양을 쓴다.
- **코드 → 낱말:** 로봇 상태·구조 상태·접근 결과·보고 상태·사건 종류는 `labels.py` 의 한국어로 보이고, 원래 코드는 시험·API(`/api/view`)에 그대로 남는다. 표에 없는 새 코드는 숨기지 않고 코드 그대로 보인다.
- **새 사건:** 페이지를 연 뒤 들어온 사건 행은 3초 동안 노랗게 칠한다(움직임 없음).
- **명령·오류 기록:** `--site-state` 로 띄우면 사건 기록 오른쪽(요구조자 목록 아래 줄)에 운영자 명령과 operations 오류(`read_operator_commands`·`read_error_log`, #125)를 최신순 30행(`view.HISTORY_ROWS`)까지 보인다. 명령은 접수/거절, 조정기가 준 사유(한국어 낱말 + 코드), 대상이고, 오류는 코드와 내용이다. 명령 패널의 답은 그 페이지에서 누른 것만 보이지만, 이 표는 새로 고침 뒤에도, 다른 사람 화면에도 같다. 사건 기록 표와 섞지 않는다(KPI-04 지연 열은 사건에만 뜻이 있다). 사건 기록과 같은 줄(최대 230px)을 쓰고 안에서 스크롤된다. 로봇 카드·요구조자 칸은 줄지 않는다. 상태 fixture(`--status`)에는 이 두 입력이 없어 「기록 입력 없음」이 보인다.
- **갱신 상태:** 오른쪽 위 「실시간 갱신 HH:MM:SS」. 폴링이 실패하면 빨갛게 「갱신 끊김: 이유, n초 전 값을 보는 중」으로 바뀐다. 이유에는 서버가 돌려준 오류 코드와 확인할 것이 들어간다.
- **탭 제목:** 재난 모드면 「[재난] 구조 관제」, 갱신이 끊기면 「[갱신 끊김] 구조 관제」. 다른 탭을 보고 있어도 상태가 보인다.
- **화면 자체를 못 그릴 때:** 장부·현장 상태를 못 읽으면 첫 페이지(`GET /`)는 503 으로 「화면을 그릴 수 없습니다」 안내를 보인다. 안내에는 오류 코드별 확인할 것(`render.FAILURE_HINTS`), 원래 오류 문구, 「로봇은 이 화면과 상관없이 동작하므로 위험하면 현장 비상 정지 장치를 쓰세요」가 들어간다. 이 페이지는 5초마다 스스로 다시 읽어서, 입력이 돌아오면 관제 화면으로 돌아온다. 폴링 조각(`/fragment`)은 같은 내용을 한 줄로, `/api/view` 는 `{error, message, hint}` JSON 으로 돌려준다(모두 503). 빈 화면을 정상처럼 보이지 않는다.

## 화면 위쪽: 모드 배지와 웹캠

- **모드 배지:** `mission_state` 행(operations 가 조정기의 SystemState 를 적은 것, #111)을 보인다. 모드는 「평시」·「재난」(빨강 띠)·「해제 중」·「임무 종료」, 그 옆에 트리거 시각·경과(서버 집계 시각 − 트리거)·단계(`phase`)·구간 「끝낸 구간/전체 + 구간 상태」(출동 중일 때)·사유(`reason`, 있을 때)와 「조정기 임무 상태, n 전 바뀜」. 행이 아직 없으면 현장 사건의 `DISASTER_SIGN` 으로 재난/평시를 **추정**하고 「임무 상태 기록 전 — … 추정」이라고 적는다. 현장 사건도 없으면 「모름」. 단계·구간 낱말은 `labels.py`.
- **웹캠:** `--camera` 로 받은 MJPEG 주소를 `<img src>` 로 넣는다(IF-082). 콘솔은 영상을 중계하지 않는다. 영상은 1초마다 바뀌는 조각(`#live`) **밖**에 있어 폴링이 스트림을 끊지 않는다. 영상이 끊기면 화면 안에 「영상이 들어오지 않습니다 …」와 캡션 「영상 끊김」을 띄운다. 주소가 없으면 `--camera` 를 넣으라는 안내가 자리에 남는다. 카메라 주소는 프로파일 v2.1 `site_cameras`(#39)가 들어오면 거기서 받는다.

## 시각과 나이 표시

- 시각은 `HH:MM:SS.mmm`(서버·브라우저 같은 형식, 24시간). 서버 집계 날짜와 다른 날의 시각은 날짜를 붙인다. 원래 ns 값은 마우스를 올리면(title) 보인다.
- 나이는 `0.4s` · `3분 05초` · `2시간 10분` · `41일` 로 줄여 쓴다. 음수는 상대 시계가 이 PC 보다 앞선 것이다.
- 로봇 행은 operations 수신 뒤 `STATUS_STALE_S = 5` 초가 지나면 「낡음」으로 표시한다([제안] 관제·화면 2026-10-10, 표시 전용 — 준비 판정 아님 C-05). 프로파일 v2.1(#39)의 readiness `max_age_s` 가 들어오면 그 값으로 바꾼다.

## 지도 (`--map`)

- **원본:** map_server 형식 지도(YAML + 이진 PGM `P5`). 지금 쓸 수 있는 것은 재난 전 기준 지도 `assets/maps/pre_quake/map.yaml` 이다(출처·해시는 [assets/manifest/pre_quake_map.json](../../assets/manifest/pre_quake_map.json)). 경로는 인자로만 받고 코드에 두지 않는다.
- **읽기:** 기동할 때 한 번 읽어 PNG 로 바꿔 메모리에 둔다. YAML 은 map_server 키 7개(`image` `mode` `resolution` `origin` `negate` `occupied_thresh` `free_thresh`)의 평평한 `키: 값` 만 받고, 모르는 키·중첩·`origin` 회전(yaw ≠ 0)·크기가 안 맞는 PGM 은 `INVALID_INPUT`, 파일이 없으면 `STORAGE_ERROR` 로 **기동이 실패**한다. 지도를 지어내거나 빈 지도로 대신 뜨지 않는다. PyYAML·Pillow 없이 표준 라이브러리만 쓴다.
- **그리기:** `GET /map.png`(로그인 필요, GET 만, URL 에 PGM 해시 `?v=` 를 붙여 캐시) 위에 SVG 를 겹친다. SVG 좌표는 지도 픽셀이고, `origin` 은 왼쪽 아래 칸의 왼쪽 아래 모서리다(열 = (x − origin_x) / resolution, 행 = 높이 − (y − origin_y) / resolution). 지도 PNG 는 그 SVG 의 `<image>` 라서 표식과 같이 늘고 준다. 로봇은 진행 방향(yaw)을 가리키는 화살촉 + 표시 이름(예: 「TB-6」·「TB-5」, yaw 가 없으면 원), 요구조자는 위 「V」 표식, 1m 격자·축 숫자·축척 막대가 같이 그려진다. SVG 는 조각(`fragment`)에 들어 있어 1초 폴링마다 바뀌고 PNG 는 다시 받지 않는다.
- **frame 규칙:** `frame_id == "map"` 인 좌표만 그린다. 다른 frame 은 투영하지 않고 「지도에 안 그림」 경고와 지도 아래 목록으로 보인다. 지도 밖 좌표는 가장자리로 끌어오지 않고 「지도 밖 좌표」로 보인다. 위치가 없는 로봇·대상은 그리지 않는다(표에 「—」).
- **확인하지 않는 것:** 좌표의 `map_session_id`·`map_revision` 이 이 기준 지도와 같은 지도에서 나왔는지는 모른다(Q-09, 지도 아래 문구로 표시). 재난 뒤 SLAM 지도 실시간 표시, IF-084 `PUT /api/maps/{id}`, `map_snapshots` 표(site_state 1.1) 연결은 후속이다.

## 구조 상태 4단계 (Q-20: 표시층 합성)

계약 어휘(`RESCUE_STATES`, `confirmation_result`)는 바꾸지 않고 이 표로 표시 단계를 만든다. 위에서부터 처음 맞는 줄이 이긴다. 코드는 `view.STAGE_RULES`·`view.compose_stage`, 시험은 `tests/test_console_view.py` `ComposeStageTests`.

| 규칙 | 조건 | 표시 |
| --- | --- | --- |
| R1 | 인식 확정이 아님(`detection_confirmed` 거짓; 대상 상태가 없으면 최신 보고의 `confirmation_result`) | 미확정 |
| R2 | `rescue_state == RESCUE_CONFIRMED` | 구출 완료 |
| R3 | `support_state` 가 `ASSIGNED` 또는 `ARRIVED` | 지원 중 |
| R4 | 지원 상태를 모르고(대상 상태 없음) 최신 보고가 `APPROACH_COMPLETE` | 지원 중 |
| R5 | 그 밖(확정, 구출 아님, 지원 없음 — `NOT_CHECKED`·`RECHECK_NEEDED` 포함) | 미구조 |

- 미확정 대상에 구출·지원 상태가 붙어 오면 R1 로 보이고 「입력 불일치」 경고를 단다.
- 대상마다 최신 보고는 `(reported_at_ns, received_at_ns, report_id)` 가 가장 큰 것이다.

## KPI-04 측정 (표시 지연 3초)

- **정의:** 지연 = 브라우저가 그 이벤트 행을 처음 그린 시각 − 이벤트 시각(`observed_at_ns`). 이벤트 시각은 로봇·카메라 시계, 표시 시각은 브라우저 PC 시계라 **두 시계가 맞을 때만**(C-03, 같은 시계 영역 또는 동기화 오차 안) 의미가 있다. 행마다 시계 영역(`clock_domain_id`)을 같이 보인다.
- **기록 위치:** 이벤트 표에 이벤트 시각·장부 기록 시각(`received_at_ns`)·화면 표시 시각·지연을 같이 둔다. 브라우저 `window.kpi04` 배열에 행을 처음 그린 순서대로 `{ref_id, kind, clock_domain_id, event_ns, received_at_ns, rendered_ms, latency_s, live}` 가 쌓인다. 서버 집계 시각은 `data-built-ns` 에 있다.
- **실시간 사건만 센다:** 페이지를 열 때 이미 있던 행은 지난 기록이라 `live: false` 이고 요약에서 뺀다. 페이지를 연 뒤 새로 나타난 행만 `live: true` 다.
- **요약과 저장:** 오른쪽 위에 「KPI-04: 실시간 사건 n건 중 3초 이내 m건, 최대 x」가 보인다. 모두 3초(`render.KPI04_BUDGET_S`) 이내면 초록, 하나라도 넘으면 빨강이다. 「지연 기록 저장」을 누르면 같은 기록을 CSV(`kpi04-<시각>.csv`)로 내려받는다. 파일은 브라우저 안에서 만들고 서버로는 아무것도 보내지 않는다. 시험 회차가 끝나면 이 파일을 증거로 남긴다.
- **폴링:** 브라우저가 `POLL_INTERVAL_MS = 1000` 마다 `GET fragment` 를 받아 바꾼다. 3초 예산 중 폴링 대기 최대 1초, 나머지가 장부 기록·읽기·전송이다. 갱신이 실패하면 화면 위에 「갱신 실패 — 표시 값이 낡았을 수 있음」을 띄운다.
- 지연 구간 나누기: 이벤트 → 장부 기록(`received_at_ns − event_ns`, 장부 PC 시계) / 장부 기록 → 화면(`rendered_ms − received_at_ns`, 화면과 장부가 같은 PC 일 때). CSV 에 두 시각이 다 있어 회차 뒤에 나눠 볼 수 있다. 현장 사건 행은 `received_at_ns` 가 비어 있다(`read_site_events` 가 돌려주지 않음, 협업·관제 요청 중).

## 운영자 명령

버튼은 [관제 웹·DB 계획](../../docs/design/console-and-db-plan.md) 4절 `POST /api/operator-commands` 의 종류다(`view.COMMANDS`). 결과가 큰 순서로 묶는다.

| 묶음 | 버튼 | 보냄 |
| --- | --- | --- |
| 승인 (크게, 파랑) — 임무 흐름의 사람 승인 지점 순서 | 수색 시작(`START_SEARCH`), 지원 로봇 출동 승인(`DISPATCH_SUPPORT`, 예: 「TB-5 출동 승인」), 다음 구간 계속(`CONTINUE_SEGMENT`) | 보냄 |
| 임무 | 재확인 지시(`REQUEST_RECHECK`), 복귀(`RETURN_HOME`) | 보냄 |
| 임무 | 일시정지·재개·작업 취소 | 꺼 둠 — Q-22 재개 관문 미결 |
| 정지 (따로, 빨강) | 정지(`STOP_ROBOT`) | 꺼 둠 — RequestStop 별도 경로, 현장 정지는 별도 수단 |

- **켜지는 때 (조정기 관문의 거울, `view.command_gates`):** 버튼 활성은 관문이 아니다. 켜진 명령도 조정기가 거절할 수 있고, 그 답을 그대로 보인다. 아래 조건 모두 `mission_state` 행이 있고 모드가 `DISASTER` 일 때만이다. 버튼 아래 작은 글씨가 지금 꺼진(켜진) 이유다.

  | 명령 | 켜짐 | 보내는 값 |
  | --- | --- | --- |
  | 수색 시작 | 단계 `DISASTER`(아직 수색 전) | 없음 |
  | 지원 로봇 출동 승인 | 출동 없음, 확정·미구출 대상 중 대상 revision 과 저장된 보고(`persisted_report_id`)가 둘 다 있는 대상이 있음(운영자가 고름) | 고른 대상의 `target_id`·`target_revision` |
  | 다음 구간 계속 | `segment_phase == AWAITING_APPROVAL` | `mission_state` 의 `target_id`·`target_revision`·`route_revision`·`segment_index` |
  | 재확인 지시 | 구출 확인 전 대상이 있음(운영자가 고름) | 고른 대상의 `target_id` |
  | 복귀 | 단계 `RESCUE_CONFIRMED`·`RECHECK_NEEDED`(판정 뒤) | 없음 |

  보내는 값은 **운영자가 화면에서 본 값**이다. revision·구간을 모르면 `null` 로 보내지 않고 버튼을 끈다(대상 revision 은 대상 상태에 기록되면 켜짐). 종류별 필드는 조정기 `rescue_coordination.mission.COMMAND_FIELDS` 와 같다(시험이 대조).
  출동 승인은 조정기가 그 revision 의 보고 저장 영수증(`ReportReceipt` PERSISTED)을 받은 대상만 고를 수 있다. 조정기는 확인됨과 저장 영수증을 둘 다 요구하는데(`mission.py` `_dispatch_support`), thin 버전 `TargetSnapshot` 은 모든 대상을 확인됨으로 보낸다. 저장 여부는 대상 상태의 선택 키 `persisted_report_id`(#118)로 읽고, 키가 없는 옛 행은 저장 전으로 본다(「보고 저장 대기」). 장부의 영수증 상태로 대신 추정하지 않는다 — 장부에 PERSISTED 로 있어도 조정기가 그 영수증을 받았다는 뜻은 아니다.
- **누르기:** ADMIN 만 누를 수 있다. 버튼을 누르면 아래에 확인 칸이 열려 무엇을 보내는지(대상·revision·「n번째 구간」) 보인다. 「보내기」를 눌러야 보낸다. 확인 칸이 열린 동안 관문 값이 바뀌면(1초 폴링) 확인을 취소하고 「상태가 바뀌어 … 다시 누르세요」를 남긴다. 갱신이 끊기면 모든 버튼을 끈다(최신이 아닌 화면으로 승인하지 않음). 한 번에 한 명령만 보낸다.
- **보내기:** 브라우저 → 콘솔 `POST /commands`(로그인·CSRF·ADMIN 확인, 폼에 없는 키는 거절) → operations `POST /api/operator-commands` 로 **한 번** 보낸다(`commands.py`, 대기 `COMMAND_TIMEOUT_S = 5` 초 [제안]). 콘솔은 `command_id`(UUID4) 를 새로 만들고 `operator_id` 에 로그인 아이디를 넣는다. 콘솔은 어떤 DB 에도 쓰지 않는다 — 명령 기록(`operator_commands`)은 operations 몫이다.
- **답 보이기:** 패널 아래 기록(최근 6건)에 「접수됨 (실행 완료 아님)」 또는 「거절 <사유 코드> (한국어 풀이)」와 `request_id` 를 그대로 보인다. 사유 풀이는 `labels.py` 의 `reply`, 모르는 코드는 코드 그대로.
- **실패:** 「보내지 못함」으로 드러내고 다시 보내지 않는다(팀 `AGENTS.md` 7절). 연결 거부는 「전달 안 됨」, 시간 초과·operations 오류·응답 모양이 틀림은 「전달됐는지 모름 — command_id … 로 명령 기록을 확인한 뒤 다시 누르세요」. 콘솔 터미널 로그에도 명령마다 한 줄씩 남는다.
- 패널 아래에 「위험할 때는 현장 비상 정지 장치를 쓰세요」를 늘 둔다. 버튼은 1초 폴링 조각 밖에 있어 누르려는 순간 다시 그려지지 않고, 켜짐 상태는 조각에 실린 관문 값(`#gates`)으로 바뀐다.

## 조정기 사유 (SystemState.reason, 2026-10-13)

조정기가 임무 상태의 사유(`SystemState.reason`, operations 의 mission 행 `reason`)에 넣은 코드를 쉬운 한국어로 보인다(`view.mission_reason`). 지원 로봇 이름은 `--robot-name` 대응(상태가 SUPPORT 인 로봇)에서 온다. 없으면 「지원 로봇」이다. 글 옆에 코드를 같이 둔다. 모르는 코드는 코드 그대로 보인다.

| 코드 | 표시 | 어디에 |
| --- | --- | --- |
| `SUPPORT_EXIT_IN_<n>S` | 「TB-5 출구 출발까지 n초」(폴링 1초마다 갱신) | 임무 띠, 지원 로봇 카드, 출동 중인 대상 줄 |
| `SUPPORT_EXITING` · `SUPPORT_AT_EXIT` | 「TB-5 출구로 이동 중」·「TB-5 출구 도착」 | 같음 |
| `SUPPORT_EXIT_CANCELED` | 「TB-5 출구 이동 취소(판정 종료·재난 해제)」 | 같음 |
| `SUPPORT_EXIT_FAILED` | 「TB-5 출구 이동 실패 — 다시 보내지 않음」(빨간색) | 같음 |
| `SUPPORT_POSE_UNKNOWN` | 「TB-5 위치 모름」(주의색) | 같음 |
| `TARGET_TIME_BASIS_UNUSABLE` 등 대상 위치 시각 거절(#174) | 「대상 위치의 관측 시각을 쓸 수 없음(…)」 등(주의색) | 임무 띠 |

자동 출동 사유 코드의 전체 목록은 조정기 PR 이 나오면 맞춘다. 그 전까지 위에 없는 코드는 코드 그대로 보인다.

## 대상 소실 후보 (2026-10-13)

고정 웹캠이 감시하던 대상을 놓치면 `SiteEvent` `TARGET_DISAPPEARED_CANDIDATE`(대상·감시 ID 포함)가 현장 상태 DB 에 들어온다. 관제 화면은 이를 두 곳에 보인다.

- 대상 줄: 그 대상의 가장 최근 후보 시각을 「소실 후보 HH:MM:SS」로 강조한다. 툴팁에 카메라·사건 ID 와 「후보일 뿐, 로봇 재확인으로 구출을 판정」을 적는다. 구출 확인은 로봇 재확인 결과(`rescue_state`)로만 바뀐다.
- 사건 기록: 「대상 소실 후보」 행의 대상 칸에 그 대상 ID 를 적는다.
- 후보가 가리키는 대상이 목록에 없으면 「확인할 것」 줄에 「소실 후보의 대상이 목록에 없음: <ID>」를 띄운다.

## 하지 않는 것 / 후속

- 하지 않음: 운영 장부 쓰기, ROS 구독, 로봇에 직접 명령, 명령 재전송, 카메라 원본 스트리밍. 로그인·계정은 위 절(콘솔 소유 DB 만 쓴다).
- 오류: `ConsoleError` 는 화면 안의 지역 오류이고 `RescueError`(계약 오류)가 아니다. `rescue_contracts.DETAIL_CATEGORIES` 에 등록하지 않는다 — 계약 코드 목록은 로봇·판정·보고 경로에만 쓴다(작전 결정, 2026-10-08).
- 후속: 일시정지·재개·취소(Q-22 뒤), 정지 버튼(RequestStop 경로), 재난 뒤 SLAM 지도 갱신 표시(IF-084·`map_snapshots`), 카메라 주소를 프로파일에서 받기(#39), 장부 이어 읽기(`read_reports` 커서).
- 강사 day5 예제는 [docs/process/source-selection.md](../../docs/process/source-selection.md) TS-02(조회 → 표의 최소 구조, DB 는 operations API 뒤)·TS-03(폴링으로 KPI-04 를 만족하면 push 를 쓰지 않음)의 판단만 따랐다. 로그인 페이지·로그인 데이터 DB 라는 **요구**는 따르고(4_login.py·welcome 표의 역할·상태 열), 하드코딩 자격 증명·고정 secret·평문 비교는 쓰지 않았다. 옮긴 줄은 없다.

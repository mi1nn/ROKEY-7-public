# rescue_console — 관제 화면 (읽기 전용)

운영 장부와 상태 스냅샷을 **읽어서** 한 화면에 보여 준다(BRD Part I 3.7.5). 2026-10-08 첫 판, 담당 자리: 관제·화면.

- 데이터의 진실은 협업·관제의 운영 장부(`rescue_operations`)다. 이 패키지는 DB 에 쓰지 않고, DB 파일을 만들지 않고, ROS 를 구독하지 않고, 로봇에 명령을 보내지 않는다.
- **현장 정지 수단을 웹 연결에만 의존시키지 않는다**(팀 `AGENTS.md` 7절). 이 화면이 꺼지거나 멈춰도 로봇 쪽 정지(통신 단절 watchdog, 현장 비상 정지)는 따로 동작해야 하고, 화면은 그 대체물이 아니다.

## 구성

| 모듈 | 하는 일 | 의존 |
| --- | --- | --- |
| `view.py` | 장부 + 상태 → 화면 모델(dict). 4단계 합성 규칙표, 명령 자리 | 표준 라이브러리, `rescue_contracts` |
| `sources.py` | `rescue_operations.ledger.read_reports` 로 장부를, `rescue_operations.site_state` 읽기 함수 셋으로 현장 상태 DB 를 읽기 전용(`mode=ro`, `query_only`)으로 읽음. 현장 상태 DB 대신 픽스처 JSON 도 받음(E2) | `rescue_operations` |
| `render.py` | 화면 모델 → HTML 한 장(CSS·JS 포함). Flask 없이 시험 | 표준 라이브러리 |
| `labels.py` | 화면에 나오는 코드(`MOVING`, `NOT_CHECKED` …) → 한국어 낱말. 모르는 코드는 그대로 보임 | 없음 |
| `mapimage.py` | 지도 YAML + PGM → PNG, map frame 좌표 → 지도 픽셀, 지도 위 표식 목록 | 표준 라이브러리 |
| `app.py` | Flask 경계. 데이터 경로는 `GET /`, `GET /fragment`, `GET /api/view`, `GET /map.png` 만(로그인·계정 폼은 아래 절) | Flask (선택) |

Flask 는 선택 의존이다(`extras_require={"web": ["flask"]}`). 없으면 `create_app` 이 `DEPENDENCY_MISSING` 오류로 끝나고 다른 방식으로 대신 뜨지 않는다. 집계·HTML·명령 무동작 시험은 Flask 없이 돈다.

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
python3 -m rescue_console <운영 장부> --accounts <계정 DB> --site-state <현장 상태 DB> \
    --camera cam_a=http://<카메라 PC>:<포트>/<경로> --map assets/maps/pre_quake/map.yaml --port 8080
```

- `--camera ID=URL` 은 여러 번 줄 수 있다. URL 은 `http`·`https` 만 받는다.

- 장부 경로·계정 파일·상태 경로·포트·바인드 주소는 인자로 받는다. 기본 바인드는 `127.0.0.1` 이고, 다른 PC 에서 볼 주소는 배치 프로파일(`operations_host`, 작전)이 정한 값을 인자로 넘긴다. 코드에 호스트·IP·개인 경로·비밀을 두지 않는다.
- 계정 파일과 세션 비밀은 저장소 밖에 둔다([SDD 10절 외부 파일 목록](../../docs/design/architecture.md)에 추가 예정).
- `examples/status.example.json` 은 예시 값이다(오프라인 리허설 장부의 대상 ID 에 맞춤). 실제 값이 아니다.

## 로그인과 계정 (강사 day5 요구: 로그인 페이지 + SQLite 로그인 데이터)

| 역할 | 할 수 있는 것 |
| --- | --- |
| `ADMIN` | 관제 화면, 계정 관리(`/admin/accounts`: CUSTOMER·ADMIN 추가, 표시 이름·이메일·역할·상태·비밀번호 수정, 삭제), 앞으로 운영자 명령 |
| `CUSTOMER` | 관제 화면 보기만. 계정 관리 페이지는 403 |

- 모든 페이지는 로그인이 필요하다. 미로그인은 `/login` 으로, `/api/*` 는 401.
- 비밀번호는 PBKDF2-HMAC-SHA256(소금 16바이트, 200,000회) 해시로만 저장한다. 평문은 저장·로그하지 않는다. 틀린 아이디·틀린 비밀번호·비활성 계정은 같은 문구로 거절한다.
- 마지막 `ACTIVE` `ADMIN` 은 비활성화·강등·삭제할 수 없고, 로그인한 본인 계정은 삭제할 수 없다.
- 세션 쿠키는 계정 ID 와 인증 버전을 담는다. 계정을 지우고 같은 이름을 다시 만들면 ID 가 달라지고, 비밀번호·역할·상태를 바꾸면 인증 버전이 올라가서, 그 전에 발급된 쿠키는 서버에서 전부 거절된다(API·조각은 401, 페이지는 로그인으로). 폴링 중 401 을 받으면 화면이 폴링을 멈추고 로그인 페이지로 간다.
- 로그아웃은 POST + CSRF 다. 링크·이미지 태그로는 로그아웃되지 않는다.
- 폼은 세션별 CSRF 토큰을 요구한다(없으면 400). 세션 쿠키는 HttpOnly·SameSite=Lax.
- 계정 DB 는 콘솔 소유 파일이다. 운영 장부(보고·site_state)의 단일 쓰기 규칙과 무관하며 콘솔은 그쪽에 여전히 쓰지 않는다.

## 입력

| 입력 | `--site-state` (operations 가 쓰는 현장 상태 DB) | `--status` (E2 픽스처) |
| --- | --- | --- |
| 보고 장부 | `read_reports(db, limit=1000)` | 같음 |
| 로봇 상태 | `rescue_operations.site_state.read_robot_status(site_db)` | 픽스처의 `robot_status` |
| 현장 사건 | `read_site_events(site_db, limit=1000)` | 픽스처의 `site_events` |
| 대상 상태(구조·지원) | `read_target_states(site_db)` | 픽스처의 `target_states` |

픽스처의 세 칸은 위 함수의 반환 형식 그대로다(협업·관제와 2026-10-08 합의). 칸이 없으면 기본값으로 채우지 않고 화면 경고 「<칸> 입력 없음」으로 보인다.

- 세 읽기는 한 트랜잭션이 아니다. 그 사이에 쓰인 행은 한 번의 폴링에서 한 표에만 먼저 보일 수 있다(다음 폴링에서 맞춰짐).
- `read_site_events` 는 오래된 순으로 `limit` 개를 준다. 콘솔은 최대(1000)를 요청하므로 현장 사건이 1000건을 넘으면 최신 사건이 빠진다(후속: 협업·관제에 최신순 또는 커서 요청).
- 출구 누적(`exit_pass_count`)은 화면에 그리지 않는다. 고정 카메라 1대 결정(2026-10-09)으로 `EXIT_PASS` 생산자가 없어 항상 0 이다([DB 스키마](../../docs/design/db-schema.md) 2절, [관제 웹·DB 계획](../../docs/design/console-and-db-plan.md) 3절). `/api/view` 에는 값이 그대로 남는다. 「구출 완료 ≤ 출구 누적」(Part I 3.5.5) 경고도 같이 뺐다 — 구출 판정의 트리거는 카메라의 대상 소실 후보다(#48).

## 화면 배치 (1920×1080 한 화면)

[관제 웹·DB 계획](../../docs/design/console-and-db-plan.md) 5절의 자리 배치를 따른다. 가로 1400px·세로 820px 이상이면 스크롤 없이 한 화면에 들어가고, 그보다 작으면 한 줄로 쌓여 스크롤된다.

```text
┌ 재난 띠: 모드 · 트리거 · 경과 · 진동 감지 수 ───────────────┬ 갱신 상태 · 사용자 ┐
├ (경고가 있을 때만) 확인할 것 ────────────────────────────────┼ 웹캠              │
├ 지도: 기준 지도 + 1m 격자 + 로봇·요구조자 ─┬ 로봇 AMR1·AMR2 ───┤                   │
│                                          │ 요구조자 목록       ├ 명령              │
├ 사건 기록 (KPI-04 열 포함) ────────────────┴───────────────────┤ (승인 · 임무 · 정지)│
└───────────────────────────────────────────────────────────────┴──────────────────┘
```

- **오프라인:** 현장 네트워크에 인터넷이 없을 수 있어 외부 CDN·웹 글꼴·외부 CSS·JS 를 쓰지 않는다(시험 `test_page_loads_nothing_from_outside`). 글꼴은 운영 PC 에 깔린 Noto Sans CJK KR 을 먼저 찾고, 없으면 OS 기본 한글 글꼴로 넘어간다. 숫자는 고정폭(`tabular-nums`)이라 1초마다 바뀌어도 흔들리지 않는다.
- **색은 의미에만 쓴다(국제 안전표지 ISO 7010 계열):**

  | 색 | 뜻 |
  | --- | --- |
  | 신호 빨강 | 재난·정지 |
  | 구조 주황 | 미구조 요구조자 |
  | 안내 파랑 | 지원 중·AMR2 |
  | 안전 초록 | 구출 완료 |
  | 회색 점선 | 미확정 |
  | 경고 노랑 | 낡음·끊김·확인할 것 |

  바탕은 차가운 회색이고 프로젝터에서 읽히도록 밝게 둔다.
- **요구조자 표식:** 지진 구조대(INSARAG)가 현장에 칠하는 「V」 모양이다. V 의 꼭짓점이 보고 좌표다. 구출 완료는 V 에 가로줄을 긋는다. 지도·목록·범례가 같은 모양을 쓴다.
- **코드 → 낱말:** 로봇 상태·구조 상태·접근 결과·보고 상태·사건 종류는 `labels.py` 의 한국어로 보이고, 원래 코드는 시험·API(`/api/view`)에 그대로 남는다. 표에 없는 새 코드는 숨기지 않고 코드 그대로 보인다.
- **새 사건:** 페이지를 연 뒤 들어온 사건 행은 3초 동안 노랗게 칠한다(움직임 없음).
- **갱신 상태:** 오른쪽 위 「실시간 갱신 HH:MM:SS」. 폴링이 실패하면 빨갛게 「갱신 끊김: 이유, n초 전 값을 보는 중」으로 바뀐다.

## 화면 위쪽: 모드 배지와 웹캠

- **모드 배지:** 현장 사건에 `DISASTER_SIGN` 이 하나라도 있으면 「재난」(빨강 띠), 없으면 「평시」, 현장 사건 입력이 없으면 「모름」. 트리거 시각은 가장 이른 `DISASTER_SIGN` 의 `observed_at_ns` 이고, 경과는 서버 집계 시각 − 트리거다. 현장 상태 1.1 의 `mission_state` 표가 생기면 그 값으로 바꾼다 — 지금은 현장 사건에서 **추정**한 것이고 화면에도 그렇게 적는다.
- **웹캠:** `--camera` 로 받은 MJPEG 주소를 `<img src>` 로 넣는다(IF-082). 콘솔은 영상을 중계하지 않는다. 영상은 1초마다 바뀌는 조각(`#live`) **밖**에 있어 폴링이 스트림을 끊지 않는다. 영상이 끊기면 화면 안에 「영상이 들어오지 않습니다 …」와 캡션 「영상 끊김」을 띄운다. 주소가 없으면 `--camera` 를 넣으라는 안내가 자리에 남는다. 카메라 주소는 프로파일 v2.1 `site_cameras`(#39)가 들어오면 거기서 받는다.

## 시각과 나이 표시

- 시각은 `HH:MM:SS.mmm`(서버·브라우저 같은 형식, 24시간). 서버 집계 날짜와 다른 날의 시각은 날짜를 붙인다. 원래 ns 값은 마우스를 올리면(title) 보인다.
- 나이는 `0.4s` · `3분 05초` · `2시간 10분` · `41일` 로 줄여 쓴다. 음수는 상대 시계가 이 PC 보다 앞선 것이다.
- 로봇 행은 operations 수신 뒤 `STATUS_STALE_S = 5` 초가 지나면 「낡음」으로 표시한다([제안] 관제·화면 2026-10-10, 표시 전용 — 준비 판정 아님 C-05). 프로파일 v2.1(#39)의 readiness `max_age_s` 가 들어오면 그 값으로 바꾼다.

## 지도 (`--map`)

- **원본:** map_server 형식 지도(YAML + 이진 PGM `P5`). 지금 쓸 수 있는 것은 재난 전 기준 지도 `assets/maps/pre_quake/map.yaml` 이다(출처·해시는 [assets/manifest/pre_quake_map.json](../../assets/manifest/pre_quake_map.json)). 경로는 인자로만 받고 코드에 두지 않는다.
- **읽기:** 기동할 때 한 번 읽어 PNG 로 바꿔 메모리에 둔다. YAML 은 map_server 키 7개(`image` `mode` `resolution` `origin` `negate` `occupied_thresh` `free_thresh`)의 평평한 `키: 값` 만 받고, 모르는 키·중첩·`origin` 회전(yaw ≠ 0)·크기가 안 맞는 PGM 은 `INVALID_INPUT`, 파일이 없으면 `STORAGE_ERROR` 로 **기동이 실패**한다. 지도를 지어내거나 빈 지도로 대신 뜨지 않는다. PyYAML·Pillow 없이 표준 라이브러리만 쓴다.
- **그리기:** `GET /map.png`(로그인 필요, GET 만, URL 에 PGM 해시 `?v=` 를 붙여 캐시) 위에 SVG 를 겹친다. SVG 좌표는 지도 픽셀이고, `origin` 은 왼쪽 아래 칸의 왼쪽 아래 모서리다(열 = (x − origin_x) / resolution, 행 = 높이 − (y − origin_y) / resolution). 지도 PNG 는 그 SVG 의 `<image>` 라서 표식과 같이 늘고 준다. 로봇은 진행 방향(yaw)을 가리키는 화살촉 + 「AMR1」·「AMR2」(yaw 가 없으면 원), 요구조자는 위 「V」 표식, 1m 격자·축 숫자·축척 막대가 같이 그려진다. SVG 는 조각(`fragment`)에 들어 있어 1초 폴링마다 바뀌고 PNG 는 다시 받지 않는다.
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
- **기록 위치:** 이벤트 표에 이벤트 시각·장부 기록 시각(`received_at_ns`)·화면 표시 시각·지연을 같이 둔다. 브라우저 `window.kpi04` 배열에 `{ref_id, event_ns, rendered_ms}` 가 처음 그린 순서대로 쌓인다(개발자 도구에서 `copy(JSON.stringify(kpi04))`). 서버 집계 시각은 `data-built-ns` 에 있다.
- **폴링:** 브라우저가 `POLL_INTERVAL_MS = 1000` 마다 `GET fragment` 를 받아 바꾼다. 3초 예산 중 폴링 대기 최대 1초, 나머지가 장부 기록·읽기·전송이다. 갱신이 실패하면 화면 위에 「갱신 실패 — 표시 값이 낡았을 수 있음」을 띄운다.
- 지연 구간 나누기: 이벤트 → 장부 기록(`received_at_ns − event_ns`, 장부 PC 시계) / 장부 기록 → 화면(`rendered − received_at_ns`, 화면과 장부가 같은 PC 일 때). 자동 측정 스크립트는 후속이다.

## 운영자 명령 (자리만)

버튼은 [관제 웹·DB 계획](../../docs/design/console-and-db-plan.md) 4절 `POST /api/operator-commands` 의 종류를 그대로 쓴다(`view.COMMANDS`). 결과가 큰 순서로 묶는다.

| 묶음 | 버튼 |
| --- | --- |
| 승인 (크게, 파랑) | AMR2 출동 승인(`DISPATCH_SUPPORT`), 다음 구간 계속(`CONTINUE_SEGMENT`) |
| 임무 | 재확인 지시, 복귀, 일시정지, 재개, 작업 취소, 수동 탐색 시작 |
| 정지 (따로, 빨강) | 정지(`STOP_ROBOT`) |

모두 `disabled` 다. 누를 수 없고, 명령을 받는 경로(POST 라우트)도 없고, `view.request_command` 는 항상 `NOT_CONNECTED` 로 거절한다. 명령은 operations 의 명령 API 와 Q-22 안전 관문이 생긴 뒤 연결한다(버튼 활성은 서버 관문의 거울일 뿐 관문이 아니다). 패널 아래에 「위험할 때는 현장 비상 정지 장치를 쓰세요」를 늘 둔다. 버튼은 1초 폴링 조각 밖에 있어 누르려는 순간 다시 그려지지 않는다.

## 하지 않는 것 / 후속

- 하지 않음: 운영 장부 쓰기, ROS 구독, 명령 전송, 카메라 원본 스트리밍. 로그인·계정은 위 절(콘솔 소유 DB 만 쓴다).
- 오류: `ConsoleError` 는 화면 안의 지역 오류이고 `RescueError`(계약 오류)가 아니다. `rescue_contracts.DETAIL_CATEGORIES` 에 등록하지 않는다 — 계약 코드 목록은 로봇·판정·보고 경로에만 쓴다(작전 결정, 2026-10-08).
- 후속: 현장 상태 1.1(`mission_state`·`error_log`·`operator_commands`) 읽기, 재난 뒤 SLAM 지도 갱신 표시(IF-084·`map_snapshots`), 대상 소실 후보(`TARGET_DISAPPEARED_CANDIDATE`, #48) 표시, 카메라 주소를 프로파일에서 받기(#39), 장부 이어 읽기(`read_reports` 커서), KPI-04 자동 측정, 오류 이력 표(3.7.4), 명령 패널(operations `POST /api/operator-commands` 뒤).
- 강사 day5 예제는 [docs/process/source-selection.md](../../docs/process/source-selection.md) TS-02(조회 → 표의 최소 구조, DB 는 operations API 뒤)·TS-03(폴링으로 KPI-04 를 만족하면 push 를 쓰지 않음)의 판단만 따랐다. 로그인 페이지·로그인 데이터 DB 라는 **요구**는 따르고(4_login.py·welcome 표의 역할·상태 열), 하드코딩 자격 증명·고정 secret·평문 비교는 쓰지 않았다. 옮긴 줄은 없다.

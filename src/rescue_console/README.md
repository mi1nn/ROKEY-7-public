# rescue_console — 관제 화면

운영 장부와 상태 스냅샷을 **읽어서** 한 화면에 보여 주고, 운영자 명령을 operations 로 넘긴다(BRD Part I 3.7.5·3.7.6). 2026-10-08 첫 판, 담당 자리: 관제·화면.

- 데이터의 진실은 협업·관제의 운영 장부(`rescue_operations`)다. 이 패키지는 운영 DB 에 쓰지 않고, DB 파일을 만들지 않고, ROS 를 구독하지 않고, 로봇에 직접 명령하지 않는다. 운영자 명령은 operations HTTP API(`POST /api/operator-commands`)로만 넘기고, 받을지는 조정기가 정한다(아래 「운영자 명령」).
- **현장 정지 수단을 웹 연결에만 의존시키지 않는다**(팀 `AGENTS.md` 7절). 이 화면이 꺼지거나 멈춰도 로봇 쪽 정지(통신 단절 watchdog, 현장 비상 정지)는 따로 동작해야 하고, 화면은 그 대체물이 아니다.

## 현재 상태 (2026-10-14, main 3278a3e)

| 기능 | 상태 | 근거 |
| --- | --- | --- |
| 로그인·계정(관리자 ADMIN, 보기 전용 CUSTOMER) | 구현됨 | `accounts.py`, `app.py` `login`·`accounts_*` |
| 현장 상태 DB 읽기(스키마 1.4) | 구현됨 | `sources.py:load_site_state`, #262 |
| 실시간 SLAM 지도(`--live-map`) | 구현됨 · 실기 일부: 시험용 TB-5 SLAM 지도 표시 [측정 2026-10-13]. TB-6 지도는 NOT_RUN | `mapimage.py:LiveMap`, `rescue_operations` `map_snapshot`, #209 |
| 로봇 직접 관측(`--robot-watch`) | 구현됨 · 실기 일부: TB-5·TB-6 배터리·도킹, TB-5 지도 위치 수신 [측정 2026-10-13]. 지도 주인 로봇만 지도에 그림(`--robot-watch-map-robot`) | `view.py:_apply_watch`, `rescue_operations` `robot_watch`, #251 |
| 터널 표시(통과 가능/불가/판정 불가) | 구현됨 · 실기 NOT_RUN | `view.py:_passages`, `mapimage.py:map_overlay`, #263 |
| 출구(QR) 관측 표시 | 구현됨 · 실기 NOT_RUN | `view.py:_exit`, #238 |
| 웹캠 고르기(관리자) | 구현됨 · 실기 일부: 운영 PC 카메라 2대 전환 [측정 2026-10-13] | `app.py:webcam`, `commands.py:switch_webcam`, #257 |
| 로봇 카메라 칸(TB-6/TB-5 고르기) | 구현됨 · 실기 NOT_RUN(로봇 판정 영상 노드 미기동) | `render.py:_robot_camera`, `app.py:robot_camera_pick`, #261 |
| 자동 출발(`--search-start AUTO`) | 구현됨 · 실기 NOT_RUN | `view.py:command_gates`, #230 |
| 재난 선포(`DECLARE_DISASTER`, 지진 감지 대신 버튼) | 구현됨 · 실기 NOT_RUN | `view.py:_gate`·`_declare_confirm` |
| 자동 출동(`--support-dispatch AUTO`) | 구현됨 · 실기 NOT_RUN | `view.py:command_gates` |
| 운영자 명령 보내기 | 구현됨 · 실기 NOT_RUN | `commands.py:send_command`, `app.py:command` |
| 조정기 사유 한국어 문구 | 구현됨 | `view.py:MISSION_REASONS`·`mission_reason` |
| 교통 관제 사유(`TRAFFIC_*`) 한국어 문구 | 구현됨 | `view.py:MISSION_REASONS`, #272(조정기), #286 |

## 다른 기기에서 접속하기

- 기본 바인드는 `127.0.0.1` 이다. 이 값이면 관제 웹을 띄운 PC 의 브라우저에서만 열린다.
- 다른 기기(노트북 등)에서 보려면 `--host <운영 PC 주소>` 로 띄운다. 주소는 배포 프로파일 값(`operations_host`)을 쓴다. 저장소와 문서에 실제 주소를 적지 않는다.
- 웹캠·로봇 카메라 영상은 브라우저가 중계 주소에서 직접 받는다. 그래서 `--camera`·`--robot-camera` 주소도 그 기기에서 닿는 주소여야 한다(운영 launch `video_bind_host`).
- 접속하는 기기와 운영 PC 는 같은 망(또는 같은 VPN)에 있어야 한다.

## 구성

| 모듈 | 하는 일 | 의존 |
| --- | --- | --- |
| `view.py` | 장부 + 상태 → 화면 모델(dict). 4단계 합성 규칙표, 임무 상태 배지, 명령 버튼의 관문 거울 | 표준 라이브러리, `rescue_contracts` |
| `commands.py` | 운영자 명령 요청 만들기·한 번 보내기·응답 읽기(`POST /api/operator-commands`) | 표준 라이브러리 |
| `sources.py` | `rescue_operations.ledger.read_reports` 로 장부를, `rescue_operations.site_state` 읽기 함수 아홉(배치 로봇·로봇 상태·현장 사건·대상 상태·임무 상태·운영자 명령·오류 기록·최신 출구(QR) 관측·터널 판정)으로 현장 상태 DB 를 읽기 전용(`mode=ro`, `query_only`)으로 읽음. 현장 상태 DB 대신 픽스처 JSON 도 받음(E2) | `rescue_operations` |
| `render.py` | 화면 모델 → HTML 한 장(CSS·JS 포함). Flask 없이 시험 | 표준 라이브러리 |
| `replay.py` | 회차 하나의 쌓이는 표 넷(사건·명령·오류·출구)과 이력 표 둘(로봇·대상) → 리플레이 시간 막대와 시점 상태 | 표준 라이브러리, `rescue_contracts` |
| `replay_page.py` | 리플레이 모델 → HTML 페이지·조각. Flask 없이 시험 | 표준 라이브러리 |
| `reset.py` | 회차 초기화 요청 한 번 보내기·답 읽기, 로봇 미리 보기, 다시 띄우기 체크 목록 모델 | 표준 라이브러리 |
| `reset_page.py` | 초기화 모델 → HTML 페이지·조각. Flask 없이 시험 | 표준 라이브러리 |
| `labels.py` | 화면에 나오는 코드(`MOVING`, `NOT_CHECKED` …) → 한국어 낱말. 모르는 코드는 그대로 보임 | 없음 |
| `mapimage.py` | 지도 YAML + PGM → PNG, map frame 좌표 → 지도 픽셀, 지도 위 표식 목록 | 표준 라이브러리 |
| `app.py` | Flask 경계. 데이터 경로는 `GET /`, `GET /fragment`, `GET /api/view`, `GET /map.png`, `GET /replay`, `GET /replay/fragment`, `GET /reset`, `GET /reset/fragment` 이다. 쓰기 경로는 운영자 명령 `POST /commands`, 웹캠 고르기 `POST /webcam`(관리자), 로봇 카메라 고르기 `POST /robot-camera`(로그인한 모든 계정, 보기 전용 포함), 회차 초기화 `POST /reset/send`(관리자), 초기화 체크 `POST /reset/check`(관리자) 다섯이다(로그인·계정 폼은 아래 절) | Flask (선택) |

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
- **새 회차는 새 현장 상태 DB 로 띄운다(OPS-183).** 재난 모드·사건 기록·임무 상태는 DB 에 남아 있어서, 같은 `--site-state`(운영 launch 의 `db_path`)로 다시 띄우면 지난 회차의 재난(예: 오전 15:13 진동 감지)이 그대로 보인다. 회차마다 새 폴더(예: `<회차 폴더>/site_state.sqlite3`)를 운영 launch `db_path` 와 관제 웹 `--site-state` 에 같이 준다. 지난 DB 는 지우지 않고 증거로 둔다.
- `--robot-name ROBOT_ID=NAME` 은 로봇마다 줄 수 있다(예: `--robot-name robot_a=TB-6 --robot-name robot_b=TB-5`, 재범 결정 2026-10-13). 기본값이 없고, 주지 않으면 로봇 ID 를 그대로 보인다. 이름은 역할에 묶지 않는다. 출동 승인 버튼과 명령 기록에는 상태가 SUPPORT 인 로봇의 이름이 들어간다(예: 「TB-5 출동 승인」). 그 로봇의 상태가 아직 없으면 어느 robot_id 인지 모르므로 역할 코드로 「SUPPORT 출동 승인」이다(「AMR·로봇」 같은 일반 명칭은 쓰지 않음, 재범 규칙 2026-10-13).
- `--support-dispatch AUTO|OPERATOR` 는 조정기의 `support_dispatch` 와 같은 현장 값이다(작전 결정 2026-10-13). AUTO 이면 조정기가 지원 로봇을 스스로 보내고 운영자의 출동 승인은 NOT_AUTHORIZED 로 거절하므로, 출동 승인 버튼 대신 「TB-5 자동 출동」 상태를 보인다. 아직 출동하지 않은 동안의 지원 사유(SUPPORT_*)는 미구조 대상 줄에 나온다. OPERATOR 이거나 인자가 없으면 지금처럼 버튼이다. IDL 을 바꾸지 않으려고 조정기가 SystemState 에 방식을 싣지 않아서 이 인자로 받는다.
- `--search-start AUTO|OPERATOR` 는 조정기의 `search_start` 와 같은 현장 값이다(#228, OPS-140). AUTO 이면 지진 감지나 재난 선포(DISASTER)가 곧 TB-6 수색 시작이고 조정기는 운영자의 수색 시작을 NOT_AUTHORIZED(`AUTO_SEARCH_ENABLED`)로 거절하므로, 수색 시작 버튼 대신 「TB-6 자동 출발」 상태를 보인다. OPERATOR 이거나 인자가 없으면 버튼이다. 값이 어긋나면: 조정기가 AUTO 인데 관제 웹이 버튼이면 버튼을 눌렀을 때의 거절(또는 임무 띠의 `AUTO_SEARCH_ENABLED`)을 보고 경고 줄에 「관제 웹을 --search-start AUTO 로 다시 띄우세요」가 나온다. 반대로 관제 웹이 AUTO 인데 조정기가 OPERATOR 이면 화면은 자동 출발이라 하지만 수색이 시작되지 않는다 — 조정기가 방식을 SystemState 에 싣지 않아 화면이 알 수 없으므로, 두 값은 같은 현장 파일에서 가져온다.
- `--camera ID=URL` 은 여러 번 줄 수 있다. URL 은 `http`·`https` 만 받는다.
- `--camera` 를 주면 `--camera-stale-s S` 도 줘야 한다(기본값 없음, 없으면 시작하지 않음). 페이지가 영상 중계기의 `/health`(마지막 프레임 나이 `age_s`)를 S/2 주기로 묻고, 나이가 S 초 이상이거나 답이 없으면 그 칸에 「영상 끊김」을 띄우고 캐시 무효화 쿼리(`_r=`)를 붙여 S 초마다 한 번 다시 연결한다. 새 프레임이 다시 오면 표시를 지우고 한 번 다시 연다. 정상으로 닫힌 multipart 스트림은 `onerror` 없이 마지막 프레임을 남긴다 [측정, 2026-10-13, Firefox]. 예전에는 프레임마다 오는 `load` 이벤트로 봤는데 Chrome 은 첫 프레임에만 load 를 내서(8초 40장에 1번 [측정, 2026-10-15, Chromium]) 살아 있는 영상을 S 초마다 끊고 다시 연결했다(2026-10-15 고침). `onerror` 표시도 그대로 둔다.
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

[관제 웹·DB 계획](../../docs/design/console-and-db-plan.md) 5절의 자리 배치를 따른다. 가로 1400px·세로 820px 이상이면 칸 배치가 고정되고 페이지 전체는 스크롤되지 않는다. 1920×1080 모니터의 실제 브라우저 창(탭바를 뺀 약 1853×963)에서 모든 칸이 한 화면에 보인다. 창이 더 낮으면 로봇 카드 칸이 먼저 줄어 그 안에서 스크롤되고, 요구조자 칸은 목록 두 줄이 보이는 높이(240px)를 지킨다. 사건 기록 줄은 내용만큼(최대 230px) 차지한다(#64). 가로 1399px 이하이면 한 줄로 쌓여 페이지가 스크롤된다. 가로 1400px 이상이고 세로 820px 미만이면 세 열은 그대로이고 페이지가 세로로 스크롤된다.

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
  | 안내 파랑 | 지원 중·SUPPORT 역할 로봇 |
  | 안전 초록 | 구출 완료 |
  | 회색 점선 | 미확정 |
  | 경고 노랑 | 낡음·끊김·확인할 것 |

  색의 뜻은 그대로다(2026-10-15 사용자 지시 「지진 재난 감지·대응 시스템 Web UI 개선」과 후속 「레이아웃은 기존처럼」, 「현대적이고 모던하게, 색 바꿔도 됨」: 기능·API·데이터·칸 배치는 그대로 두고 화면만). 밝은 바탕 위 그림자 카드(테두리 없음), 인디고 강조색(이름 줄 배지·링크·포커스), 굵고 또렷한 숫자, 작은 색 배지(로봇 움직임·준비, 요구조자 단계), 요구조자 수는 단계 색 타일이다. 주 버튼은 파랑(승인·보내기), 빨강은 재난 배지·정지 버튼·실패에만 쓴다. 색 값은 `render.py` `_STYLE` 맨 위 `:root` 한 곳에 있다. 지도 그림(PGM)은 SVG 필터 `#map-tone` 으로 화면 색만 바꾼다(벽 차콜, 미탐색 연회색, 빈 곳 흰색). 한 화면 고정 배치에서는 카메라 영상 높이를 화면 높이의 16%(로봇 카메라 칸이 없으면 웹캠 27%)로 묶고 명령 칸이 남는 높이를 쓴다. 영상 이름표는 영상 왼쪽 아래 작은 배지다. 칸 제목·표 머리·도움말은 처음 보는 사람도 알 수 있는 말로 바꿨다(예: 트리거 → 재난 감지 시각, 장부 기록 → 기록 시각).
- **화면에서 뺀 문구(2026-10-15 사용자 요청, 데이터·스크립트는 그대로):** 지도 설명 줄(실시간 지도가 낡았을 때의 빨간 경고만 보임), 임무 띠의 모드 출처 설명(띠에 마우스를 올리면 보임), 시각의 밀리초(마우스를 올리면 ns), 사건 표의 화면 표시·표시 지연 열(스크립트는 계속 채우고 「지연 기록 저장」으로 받음), 명령 칸의 「누르면 operations 로…」 안내(보낼 수 없을 때의 빨간 사유는 보임)와 「이 화면의 명령은 보조 수단이고…」 문장. 위쪽 막대의 KPI-04 요약 문구(스크립트는 계속 계산하고 「지연 기록 저장」으로 받음). 임무 띠는 모드 배지 뒤로 값마다 같은 폭의 칸과 얇은 구분선을 둔 지표 줄이다(서버 기준 시각이 마지막 칸). 재난 띠는 그라데이션 없이 연한 단색이다. 작은 회색 라벨 색(`--ink-3`)은 흰 바탕 대비 약 4.5:1 이다. 「위험할 때는 현장 비상 정지 장치를 쓰세요」는 남긴다.
- **보낸 명령 · 오류:** 평소에는 접어 두고 그 자리를 로봇 상태 칸이 쓴다(두 로봇이 한 눈에). 위쪽 막대의 「보낸 명령 · 오류」를 누르면 펴진다. 펼침 상태는 갱신 영역 밖 체크박스라 1초 갱신에도 유지되고 스크립트는 바꾸지 않았다(2026-10-15 사용자 요청).
- **시스템 이름:** 화면 맨 위 한 줄과 로그인·계정 관리 화면에 팀 BRD Part I 제목 「지진 재난 현장 대응형 실내 탐색 로봇」(`render.SYSTEM_NAME`)을 둔다. 탭 제목(「[재난] 구조 관제」 등)과 「구조 관제」 문구는 바꾸지 않는다.
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

## 웹캠 고르기 (관리자, OPS-183)

`--camera` 와 `--operations` 로 띄우면 관리자 화면의 웹캠 칸 아래에 「카메라 [목록] 카메라 바꾸기」가 나온다. 목록은 operations 가 운영 PC 의 `/dev/v4l/by-id/*-video-index0`(기동 인자 `webcam_device` 와 같은 이름, #204)을 읽어 주고, 지금 장치가 선택돼 있다.

- 바꾸면 operations(`POST /api/webcam`)가 webcam_node 의 `device_path` 파라미터를 바꾼다. webcam_node 는 카메라를 닫고 새 장치로 열며, **지진 판정을 처음부터 다시 하고 출구 감시를 푼다**(화면·ROI 가 바뀜). 카메라 상태는 `OFFLINE`/`CAMERA_SWITCHED` 를 한 번 낸다.
- 화면에는 「카메라 전환 — 지진 판정 다시 시작, 출구 감시 풀림」, 거절이면 「카메라 전환 거절: <이유>」가 다음 화면에 보인다. operations 는 둘 다 오류 기록에 남겨 「명령·오류 기록」 표에 나온다. 오류 기록은 공통 코드(C-09)만 받으므로 내용이 `CAMERA_SWITCHED`·`CAMERA_SWITCH_REFUSED` 로 시작하고(누가·어디서·어디로), 코드는 전환 `SENSOR_UNAVAILABLE`(판정 다시 시작 중), 꽂혀 있지 않음 `NOT_FOUND`, webcam_node 거절 `INVALID_INPUT`, 무응답 `DRIVER_UNAVAILABLE` 이다.
- 꽂혀 있지 않은 장치는 operations 가 `UNKNOWN_DEVICE` 로, 사라진 경로는 webcam_node 가 거절한다. 조용히 이전 장치로 두지 않는다.
- 바꾸기 전에 늘 확인 창이 뜨고, 재난 모드에서는 「임무 진행 중(재난 모드)입니다…」로 묻는다(재범 결정 대기: 재난 모드에서 아예 막을지).
- 손님(CUSTOMER)에게는 목록도 버튼도 없고, `POST /webcam` 은 403 이다. operations 가 답하지 않으면 「카메라 목록을 못 받음」이 보인다.

## 로봇 카메라 (OPS-191, 재범 결정)

웹캠 칸 아래에 「로봇 카메라」 칸 하나가 있고 TB-5 터널 판정 영상을 띄운다(기본 TB-5 = 프로파일의 SUPPORT, 재범 결정 2026-10-14). 두 로봇을 주면 TB-6 도 고를 수 있다. 운영 명령이 아니라 보기 설정이라 로그인한 모든 계정(보기 전용 포함)이 바꾸고 확인 창이 없다.

- 영상은 통합-인식이 로봇 PC 에서 만든 판정 그린 영상 `<ns>/rescue/passage_view/compressed`(CompressedImage jpeg, BEST_EFFORT depth 1, 352×352 초당 5장 [제안])이다. 원본 RGB 를 Wi-Fi 로 또 끌지 않는다.
- 운영 launch `robot_views:=robot_b=/robot5`(TB-5 만) 또는 `robot_views:=robot_a=/robot6,robot_b=/robot5`(둘 다, 기본 TB-5)를 주면 `rescue_perception robot_view_relay`(노드 이름 `robot_view_relay`)가 `video_port + 1` 에서 `/robot_view.mjpg` 를 내고, **고른 로봇 하나만 구독**한다(고르지 않은 쪽은 구독을 푼다 — Wi-Fi 에 영상 한 줄). 관제 웹은 `--robot-camera http://<운영 PC>:<video_port+1>/robot_view.mjpg` 로 띄운다(`--camera-stale-s` 필요).
- 고르기는 관제 웹 `POST /robot-camera` → 중계 `POST /select {"robot": id}`. 바꾸면 중계가 이전 로봇의 마지막 화면을 버린다.
- 고른 로봇 영상이 `video_stale_s`(중계)·`--camera-stale-s`(화면) 동안 없으면 「카메라 영상 낡음」이고 마지막 화면을 감춘다. 다른 로봇의 터널 판정은 지도 표시로만 본다(다음 PR, `overhead_passages` 기록 뒤).

## 시각과 나이 표시

- 시각은 `HH:MM:SS.mmm`(서버·브라우저 같은 형식, 24시간). 서버 집계 날짜와 다른 날의 시각은 날짜를 붙인다. 원래 ns 값은 마우스를 올리면(title) 보인다.
- 나이는 `0.4s` · `3분 05초` · `2시간 10분` · `41일` 로 줄여 쓴다. 음수는 상대 시계가 이 PC 보다 앞선 것이다.
- 로봇 행은 operations 수신 뒤 `STATUS_STALE_S = 5` 초가 지나면 「낡음」으로 표시한다([제안] 관제·화면 2026-10-10, 표시 전용 — 준비 판정 아님 C-05). 프로파일 v2.1(#39)의 readiness `max_age_s` 가 들어오면 그 값으로 바꾼다.

## 지도 (`--map`)

- **원본:** map_server 형식 지도(YAML + 이진 PGM `P5`). 지금 쓸 수 있는 것은 재난 전 기준 지도 `assets/maps/pre_quake/map.yaml` 이다(출처·해시는 [assets/manifest/pre_quake_map.json](../../assets/manifest/pre_quake_map.json)). 경로는 인자로만 받고 코드에 두지 않는다.
- **읽기:** 기동할 때 한 번 읽어 PNG 로 바꿔 메모리에 둔다. YAML 은 map_server 키 7개(`image` `mode` `resolution` `origin` `negate` `occupied_thresh` `free_thresh`)의 평평한 `키: 값` 만 받고, 모르는 키·중첩·`origin` 회전(yaw ≠ 0)·크기가 안 맞는 PGM 은 `INVALID_INPUT`, 파일이 없으면 `STORAGE_ERROR` 로 **기동이 실패**한다. 지도를 지어내거나 빈 지도로 대신 뜨지 않는다. PyYAML·Pillow 없이 표준 라이브러리만 쓴다.
- **그리기:** `GET /map.png`(로그인 필요, GET 만, URL 에 PGM 해시 `?v=` 를 붙여 캐시) 위에 SVG 를 겹친다. SVG 좌표는 지도 픽셀이고, `origin` 은 왼쪽 아래 칸의 왼쪽 아래 모서리다(열 = (x − origin_x) / resolution, 행 = 높이 − (y − origin_y) / resolution). 지도 PNG 는 그 SVG 의 `<image>` 라서 표식과 같이 늘고 준다. 로봇은 진행 방향(yaw)을 가리키는 화살촉 + 표시 이름(예: 「TB-6」·「TB-5」, yaw 가 없으면 원), 요구조자는 위 「V」 표식, 1m 격자·축 숫자·축척 막대가 같이 그려진다. SVG 는 조각(`fragment`)에 들어 있어 1초 폴링마다 바뀌고 PNG 는 다시 받지 않는다.
- **frame 규칙:** `frame_id == "map"` 인 좌표만 그린다. 다른 frame 은 투영하지 않고 「지도에 안 그림」 경고와 지도 아래 목록으로 보인다. 지도 밖 좌표는 가장자리로 끌어오지 않고 「지도 밖 좌표」로 보인다. 위치가 없는 로봇·대상은 그리지 않는다(표에 「—」).
- **확인하지 않는 것:** 좌표의 `map_session_id`·`map_revision` 이 이 기준 지도와 같은 지도에서 나왔는지는 모른다(Q-09, 지도 아래 문구로 표시). IF-084 `PUT /api/maps/{id}`, `map_snapshots` 표(site_state 1.1) 연결은 후속이다.

## 로봇 직접 관측 (`--robot-watch`, 2026-10-13)

로봇 PC 의 `robot.launch.py`(`/rescue/robots/...`)가 아직 없어도 로봇 칸에 로봇이 보이게 한다(재범 2026-10-13). 운영 launch 에 `robot_namespaces:=robot_a=/robot6,robot_b=/robot5 robot_watch_path:=<파일>` 을 주면 `rescue_operations` `robot_watch` 노드(이름 `rescue_robot_watch`)가 아래 토픽을 **구독만** 하고(발행·서비스·액션 없음, 재범 허용) 1초마다 그 파일을 쓴다. 관제 웹은 `--robot-watch <파일> --robot-watch-stale-s <초> --robot-watch-map-robot <robot_id>` 로 읽는다. 셋은 같이 준다.

| 토픽 | 타입 | QoS |
|---|---|---|
| `<ns>/battery_state` | `sensor_msgs/BatteryState` | sensor data(BEST_EFFORT) |
| `<ns>/dock_status` | `irobot_create_msgs/DockStatus` | sensor data(BEST_EFFORT) |
| `<ns>/tf` | `tf2_msgs/TFMessage` | RELIABLE, VOLATILE, 100 |
| `<ns>/tf_static` | `tf2_msgs/TFMessage` | RELIABLE, TRANSIENT_LOCAL, 100 |

- 카드에는 「직접 관측 · N초 전 · 배터리 N% · 도킹」이 나온다. 프로젝트 상태가 없는 로봇은 「프로젝트 상태 없음 — 로봇 기본 토픽 직접 관측만 보임」 카드를 받는다.
- 위치는 tf 에 `map -> base_link` 가 있을 때만 쓴다(TB-6 SLAM 중). 없으면 지어내지 않고 「위치 미확정」이다. 프로젝트 상태의 위치가 있으면 그쪽이 먼저다.
- 이 `map` 은 로봇마다 자기 localization(자기 SLAM, 시작 위치 없는 AMCL)의 map 이라 원점이 각자 출발한 도크다. 그래서 큰 지도에는 `--robot-watch-map-robot` 로 준 로봇(실시간 SLAM 지도를 만드는 TB-6)의 직접 관측 위치만 그린다. 다른 로봇은 좌표를 옮기지 않고 frame 만 `<ns> map`(예: `/robot5 map`)으로 바꿔 「지도에 안 그림」 목록에 둔다. 카드에는 「위치는 자기 지도 기준, 이 지도와 다름」이 나온다(현장 2026-10-14, TB-5 가 TB-6 지도 원점에 겹쳐 그려짐). TB-5 가 고정 지도에서 위치를 잡아 프로젝트 상태로 보내는 위치는 그대로 그린다.
- `--robot-watch-stale-s` 보다 오래 받은 것이 없으면 「직접 관측 낡음」이고, 그 위치는 그리지 않는다. 파일이 갱신되지 않으면 경고 줄에 나온다. 낡음은 이 호스트 시계로 재므로 관측 노드와 관제 웹은 같은 PC 에서 돈다.

## 실시간 SLAM 지도 (`--live-map`, 2026-10-13)

TB-6 SLAM 지도를 큰 지도 칸에 띄운다(재범 결정 2026-10-13). 운영 launch 의 `map_topic`·`map_dir` 가 지도를 받아 `<map_dir>/live_map.yaml` 로 쓰고(`rescue_operations` `map_snapshot`), 관제 웹은 `--live-map <map_dir>/live_map.yaml --live-map-stale-s <초>` 로 그 파일을 본다. `--map` 과 같이 줄 수 없다.

- **다시 읽기:** YAML 이 바뀔 때마다(수정 시각·크기) 다시 읽고 PNG URL 의 해시가 바뀐다. 기동 때 파일이 아직 없어도 뜬다.
- **지도 판 표시:** 지도 아래에 받은 시각(YAML 수정 시각)·몇 초 전·크기·해당도·PGM 해시 앞 12자.
- **없음·낡음·실패를 숨기지 않음:** 파일이 없으면 「실시간 지도 없음」 칸, `--live-map-stale-s` 보다 오래 지도 발행이 없으면(새 지도도 없고 `live_map.alive` 도 갱신되지 않음) 「지도 낡음」 경고와 흐린 지도, 새 파일을 못 읽으면 「지도 읽기 실패 — 마지막으로 읽은 지도를 보임」 경고. 기본 낡음 시간은 없다(현장 값).
- **위치 없는 로봇:** 위치가 없는 로봇은 지도에 그리지 않고 「지도에 그리지 못한 위치」 목록에 「TB-5: 위치 미확정」처럼 보인다(지어내지 않음). `--map` 에도 같다.
- **출구(QR):** 현장 상태 DB `exit_locations`(#237, VALID `exit_qr` 관측만)의 최신 관측 하나를 초록 네모와 「출구(QR) 관측 HH:MM:SS」로 찍는다. 관측일 뿐 조정기가 등록한 출구가 아니다 — 등록 여부는 사유(`SUPPORT_EXIT_UNREGISTERED` 등)로 보인다. 고정 전·후 관측은 구분하지 않고 `map_revision` 은 마우스를 올린 설명에만 있다(OPS-150). 지도 밖이면 목록에 「출구(QR): 지도 밖」. `--map` 에도 같다.
- **터널(높이 장애물, OPS-189·196):** 현장 상태 DB `overhead_passages`(#262 `read_overhead_passages`, 통합-인식 #259 판정을 통합-조정이 기록)를 ∏ 모양 표시와 문구로 그린다 — 초록 「통과 가능 ↑0.47 m」, 빨강 「통과 불가 ↑0.40 m」, 회색 「판정 불가」(지도·카메라 공통 문구). 항목의 (map_session_id, map_revision) 이 `current_map`(TB-6 최신 MapState)과 같으면 진하게, 다르면 흐리게(이전 지도). `current_map` 을 모르면 전부 진하게 그리고 「현재 지도 revision 모름」을 경고 줄에 적는다. 지도 밖이면 「통과 불가 ↑0.40 m: 지도 밖」으로 목록에만. 로봇의 판정 노드가 다시 켜지면 빈 목록부터라 이전 항목도 사라진다(#262).
- **고정 뒤:** TB-6 은 고정 지도를 같은 `/robot6/map` 에 한 번 다시 내고 그 뒤로 해시가 바뀌지 않는다(작전 OPS-117/118). `map_snapshot` 이 토픽에 발행자가 있는 동안 `live_map.alive` 를 2초마다 갱신하므로, 바뀌지 않는 고정 지도는 낡음이 아니고 발행자가 사라질 때만 낡음이다. 「받은 시각」은 지도를 실제로 받은 시각 그대로다.

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
- **폴링:** 브라우저가 `POLL_INTERVAL_MS = 1000` 마다 `GET fragment` 를 받아 바꾼다. 3초 예산 중 폴링 대기 최대 1초, 나머지가 장부 기록·읽기·전송이다. 갱신이 실패하면 화면 위에 「갱신 끊김: {이유}, {n} 전 값을 보는 중」을 띄우고 명령 버튼을 모두 끈다(화면이 최신이 아니므로).
- 지연 구간 나누기: 이벤트 → 장부 기록(`received_at_ns − event_ns`, 장부 PC 시계) / 장부 기록 → 화면(`rendered_ms − received_at_ns`, 화면과 장부가 같은 PC 일 때). CSV 에 두 시각이 다 있어 회차 뒤에 나눠 볼 수 있다. 현장 사건 행은 `received_at_ns` 가 비어 있다(`read_site_events` 가 돌려주지 않음, 협업·관제 요청 중).

## 운영자 명령

버튼은 [관제 웹·DB 계획](../../docs/design/console-and-db-plan.md) 4절 `POST /api/operator-commands` 의 종류다(`view.COMMANDS`). 결과가 큰 순서로 묶는다.

| 묶음 | 버튼 | 보냄 |
| --- | --- | --- |
| 승인 (크게, 파랑) — 임무 흐름의 사람 승인 지점 순서 | 재난 선포(`DECLARE_DISASTER`), 수색 시작(`START_SEARCH`), 출동 승인(`DISPATCH_SUPPORT`, 예: 「TB-5 출동 승인」), 다음 구간 계속(`CONTINUE_SEGMENT`) | 보냄 |
| 임무 | 재확인 지시(`REQUEST_RECHECK`), 복귀(`RETURN_HOME`) | 보냄 |
| 임무 | 일시정지·재개·작업 취소 | 꺼 둠 — Q-22 재개 관문 미결 |
| 정지 (따로, 빨강) | 정지(`STOP_ROBOT`) | 꺼 둠 — RequestStop 별도 경로, 현장 정지는 별도 수단 |

- **켜지는 때 (조정기 관문의 거울, `view.command_gates`):** 버튼 활성은 관문이 아니다. 켜진 명령도 조정기가 거절할 수 있고, 그 답을 그대로 보인다. 아래 조건 모두 `mission_state` 행이 있어야 하고, 재난 선포만 모드가 `NORMAL` 일 때, 나머지는 `DISASTER` 일 때만이다. 버튼 아래 작은 글씨가 지금 꺼진(켜진) 이유다.

  | 명령 | 켜짐 | 보내는 값 |
  | --- | --- | --- |
  | 재난 선포 | 모드 `NORMAL`(이미 재난이면 「이미 재난 모드」로 꺼짐) | 없음 |
  | 수색 시작 | 단계 `DISASTER`(아직 수색 전) | 없음 |
  | 출동 승인(`DISPATCH_SUPPORT`) | 출동 없음, 확정·미구출 대상 중 대상 revision 과 저장된 보고(`persisted_report_id`)가 둘 다 있는 대상이 있음(운영자가 고름) | 고른 대상의 `target_id`·`target_revision` |
  | 다음 구간 계속 | `segment_phase == AWAITING_APPROVAL` | `mission_state` 의 `target_id`·`target_revision`·`route_revision`·`segment_index` |
  | 재확인 지시 | 구출 확인 전 대상이 있음(운영자가 고름) | 고른 대상의 `target_id` |
  | 복귀 | 단계 `RESCUE_CONFIRMED`·`RECHECK_NEEDED`(판정 뒤) | 없음 |

  보내는 값은 **운영자가 화면에서 본 값**이다. revision·구간을 모르면 `null` 로 보내지 않고 버튼을 끈다(대상 revision 은 대상 상태에 기록되면 켜짐). 종류별 필드는 조정기 `rescue_coordination.mission.COMMAND_FIELDS` 와 같고, 재난 선포는 `MODE_COMMAND_FIELDS` 와 같다(시험이 대조).
  재난 선포(재범 결정·작전 설계 2026-10-15)는 지진 감지 대신 운영자가 재난 모드로 넣는 버튼이다. 지진 감지와 버튼 가운데 먼저 온 쪽이 재난 모드를 열고, 나중 것은 조정기가 `TASK_BUSY`(이 명령에서는 「이미 재난 모드입니다」로 풀이, 사유 `ALREADY_DISASTER`, 「이미 재난 모드입니다 — 재난 선포 거절」)로 거절한다. 버튼으로 들어간 재난은 사건(`site_events`)이 아니라 명령 기록에만 남아 사건 목록에 지진 감지 행이 없고, 임무 띠에 「운영자 선포 — 지진 감지가 아니라 재난 선포 버튼으로 재난 모드」(`DISASTER_DECLARED_BY_OPERATOR`)가 나온다(#330). `--search-start AUTO` 이면 재난 모드가 되자마자 TB-6 이 탐색을 시작하므로, 확인 칸 첫 줄이 「재난 선포 — 재난 모드로 들어가고 TB-6 이 바로 탐색을 시작합니다」다. OPERATOR 이면 「… TB-6 탐색은 수색 시작 버튼으로 시작합니다」, 인자가 없으면 「재난 선포 — 재난 모드로 들어갑니다」다.
  출동 승인은 조정기가 그 revision 의 보고 저장 영수증(`ReportReceipt` PERSISTED)을 받은 대상만 고를 수 있다. 조정기는 확인됨과 저장 영수증을 둘 다 요구하는데(`mission.py` `_dispatch_support`), thin 버전 `TargetSnapshot` 은 모든 대상을 확인됨으로 보낸다. 저장 여부는 대상 상태의 선택 키 `persisted_report_id`(#118)로 읽고, 키가 없는 옛 행은 저장 전으로 본다(「보고 저장 대기」). 장부의 영수증 상태로 대신 추정하지 않는다 — 장부에 PERSISTED 로 있어도 조정기가 그 영수증을 받았다는 뜻은 아니다.
- **누르기:** ADMIN 만 누를 수 있다. 버튼을 누르면 아래에 확인 칸이 열려 무엇을 보내는지(대상·revision·「n번째 구간」) 보인다. 「보내기」를 눌러야 보낸다. 확인 칸이 열린 동안 관문 값이 바뀌면(1초 폴링) 확인을 취소하고 「상태가 바뀌어 … 다시 누르세요」를 남긴다. 갱신이 끊기면 모든 버튼을 끈다(최신이 아닌 화면으로 승인하지 않음). 한 번에 한 명령만 보낸다.
- **보내기:** 브라우저 → 콘솔 `POST /commands`(로그인·CSRF·ADMIN 확인, 폼에 없는 키는 거절) → operations `POST /api/operator-commands` 로 **한 번** 보낸다(`commands.py`, 대기 `COMMAND_TIMEOUT_S = 5` 초 [제안]). 콘솔은 `command_id`(UUID4) 를 새로 만들고 `operator_id` 에 로그인 아이디를 넣는다. 콘솔은 어떤 DB 에도 쓰지 않는다 — 명령 기록(`operator_commands`)은 operations 몫이다.
- **답 보이기:** 패널 아래 기록(최근 6건)에 「접수됨 (실행 완료 아님)」 또는 「거절 <사유 코드> (한국어 풀이)」와 `request_id` 를 그대로 보인다. 사유 풀이는 `labels.py` 의 `reply`, 모르는 코드는 코드 그대로.
- **실패:** 「보내지 못함」으로 드러내고 다시 보내지 않는다(팀 `AGENTS.md` 7절). 연결 거부는 「전달 안 됨」, 시간 초과·operations 오류·응답 모양이 틀림은 「전달됐는지 모름 — command_id … 로 명령 기록을 확인한 뒤 다시 누르세요」. 콘솔 터미널 로그에도 명령마다 한 줄씩 남는다.
- 패널 아래에 「위험할 때는 현장 비상 정지 장치를 쓰세요」를 늘 둔다(뒷문장 「이 화면의 명령은 보조 수단이고, 받을지는 조정기가 정합니다」는 2026-10-15 사용자 요청으로 뺌). 버튼은 1초 폴링 조각 밖에 있어 누르려는 순간 다시 그려지지 않고, 켜짐 상태는 조각에 실린 관문 값(`#gates`)으로 바뀐다.

## 조정기 사유 (SystemState.reason, 2026-10-13)

조정기가 임무 상태의 사유(`SystemState.reason`, operations 의 mission 행 `reason`)에 넣은 코드를 쉬운 한국어로 보인다(`view.mission_reason`). 로봇 이름은 상태가 SUPPORT·SCOUT 인 로봇의 표시 이름이다(`--robot-name` 대응, 없으면 robot_id). 그 역할의 로봇 상태가 아직 없으면 역할 코드(SUPPORT·SCOUT)를 쓴다(일반 명칭은 쓰지 않음, 재범 규칙). 판정 RESCUE_CONFIRMED 는 「구출 완료」(초록), 모드 MISSION_FINISHED 는 「임무 종료」다. 글 옆에 코드를 같이 둔다. 모르는 코드는 코드 그대로 보인다.

| 코드 | 표시 | 어디에 |
| --- | --- | --- |
| `SUPPORT_EXIT_IN_<n>S` | 「TB-5 출구 출발까지 n초」(폴링 1초마다 갱신) | 임무 띠, TB-5(SUPPORT) 카드, 출동 중인 대상 줄 |
| `SUPPORT_EXITING` · `SUPPORT_AT_EXIT` | 「TB-5 출구로 이동 중」·「TB-5 출구 도착」 | 같음 |
| `SUPPORT_EXIT_CANCELED` | 「TB-5 출구 이동 취소(판정 종료·재난 해제)」 | 같음 |
| `SUPPORT_EXIT_FAILED` | 「TB-5 출구 이동 실패 — 다시 보내지 않음」(빨간색) | 같음 |
| `SUPPORT_POSE_UNKNOWN` | 「TB-5 위치 모름」(주의색) | 같음 |
| `SUPPORT_POSE_BASIS_MISMATCH` · `SUPPORT_AT_TARGET` | 「TB-5 위치의 지도 기준이 다름 — 자동 출동 보류」·「TB-5 가 이미 대상 바로 옆 — 접근 지점을 정할 수 없음」(주의색) | 같음 |
| `SUPPORT_NOT_READY` | 「TB-5 준비 안 됨 — arm 필요」(주의색, #196: TB-5 상태가 없거나 READY·최신이 아님) | 같음 |
| `APPROACH_FAILED` | 「TB-5 대상 접근 실패 — 다시 보내지 않음」(빨간색) | 같음 |
| `APPROACH_FAILED_<사유>`(#216) | 「TB-5 접근 실패: <사유> — 다시 보내지 않음」(빨간색). 사유: 통로 막힘·갈 수 없는 위치·실행 실패·취소됨·시간 초과·주행 준비 안 됨·정지 유지 중·명령 유효 시간 지남·시계 불확실. 모르는 사유는 코드 그대로 | 같음 |
| 자동 접근 판단(`MODE_NOT_DISASTER`·`ALREADY_DISPATCHED`·`UNKNOWN_TARGET`·`TARGET_NOT_CONFIRMED`·`TARGET_REVISION_STALE`·`TARGET_ALREADY_JUDGED`) | 「… — TB-5 출동 안 함/대기/다시 판단」 | 같음 |
| `TARGET_TIME_BASIS_UNUSABLE` 등 대상 위치 시각 거절(#174) | 「대상 위치의 관측 시각을 쓸 수 없음(…)」 등(주의색) | 임무 띠 |
| `RECHECK_NEEDED_<이유>` | 「재확인 필요: <이유>」(주의색). 이유: 후보 연결 안 됨·지도 기준 다름·시각 기준 못 씀·시각 순서 틀림·인형이 아직 보임·자리가 막힘·관측 범위 부족·재확인 관측 없음(「재확인 지시」 버튼 안내). 모르는 이유는 코드 그대로 | 임무 띠, 출동 중인 대상 줄 |
| `SCOUT_RETURNING` · `SCOUT_AT_DOCK_APPROACH_DOCK_BY_HAND` | 「TB-6 도킹 접근 지점으로 이동 중」·「TB-6 도킹 접근 지점 도착 — 도킹은 사람이 합니다」 | 임무 띠, TB-6(SCOUT) 카드 |
| `SCOUT_RETURN_FAILED` | 「TB-6 도킹 접근 지점 이동 실패 — 다시 보내지 않음」(빨간색) | 같음 |
| 지도 고정 관문(#214): `MAP_FREEZE_WAITING_SCOUT_STOP`·`MAP_FREEZE_REQUESTED`·`MAP_NOT_FROZEN`·`MAP_FREEZE_FAILED`·`TARGET_NOT_ON_FROZEN_MAP` | 「TB-6 이 멈추기를 기다림 — 멈추면 지도 고정」·「지도 고정 요청함 — 고정 지도 확인 중」·「지도가 아직 고정되지 않음 — TB-5 출동 대기」(주의색)·「지도 고정 실패 — 다시 요청하지 않음」(빨간색)·「고정 지도 기준 대상 위치가 아직 없음 — TB-5 출동 대기」 | 임무 띠 |
| `SCOUT_NOT_LOCALIZED`·`SUPPORT_NOT_LOCALIZED`·`SUPPORT_APPROACH_NOT_FREE`·`SUPPORT_EXIT_NOT_FREE`·`SCOUT_HOME_NOT_FREE`(#214) | 「TB-6 이 고정 지도에서 위치를 못 잡음 — TB-5 출동 대기」·「TB-5 가 고정 지도에서 위치를 못 잡음 — 출동 대기」·「TB-5 접근 지점이 막힘 — 출동 보류」·「TB-5 출구 지점이 막힘 — 출구 이동 보류(대상 옆 대기)」·「TB-6 도킹 접근 지점이 막힘 — TB-6 복귀 보류」(주의색). 각 사유는 그 사유가 지키는 이동만 보류한다(TB-5 출동·TB-5 출구 이동·TB-6 복귀, #218 검토) | 임무 띠와 그 로봇 카드 |
| `NO_HOME_POSITION`(#214) | 「복귀 위치(출구·도킹 접근) 설정 없음 — 복귀 이동 안 함」(주의색). 운영자 RETURN_HOME 거절 답에도 같은 코드 | 임무 띠 |
| 자동 수색 시작(#228): `SEARCH_GOAL_NOT_SELECTED`·`AUTO_SEARCH_ENABLED`, 출구 QR(OPS-136): `MAP_FREEZE_WAITING_EXIT` | 「TB-6 수색 목표 선택 대기」(주의색)·「자동 수색 — 지진 감지나 재난 선포로 시작, 수색 시작 버튼은 받지 않음」·「출구 QR 찾는 중 — TB-5 출동 대기」 | 임무 띠 |
| QR 출구·재기준·순찰·출발 자세(#234): `TARGET_REBASED_ON_FREEZE`·`EXIT_NOT_ON_FROZEN_MAP`·`SUPPORT_EXIT_UNREGISTERED`·`START_POSE_UNKNOWN`·`START_POSE_CONFLICT`·`PATROL_ROUND_<n>`·`PATROL_EXHAUSTED` | 「대상 위치를 고정 지도로 옮김 — 보고 저장 대기」·「출구(QR) 위치가 고정 지도 기준에 없음 — TB-5 출동·출구 이동 보류」(주의색)·「출구(QR) 미등록 — TB-5 출동·출구 이동 보류」(주의색)·「TB-6 출발 자세 모름 — 순찰·TB-6 복귀 보류」(주의색)·「TB-6 출발 자세가 다른 값으로 다시 옴 — 처음 값 유지, 확인 필요」(빨간색)·「TB-6 순찰 <n>바퀴째」·「TB-6 순찰 끝(정한 바퀴 다 돎) — 더 돌지 않음」(주의색) | 임무 띠(`SUPPORT_EXIT_UNREGISTERED` 는 TB-5 카드에도) |
| 탐색·완성 지도·TB-5 출구 길(#247/#248): `MAP_NOT_FINAL`·`SUPPORT_START_UNKNOWN`·`SUPPORT_EXIT_UNREACHABLE`·`NO_SURVIVOR_FOUND`·`EXPLORATION_COMPLETE`·`EXPLORATION_TIMEOUT`·`EXPLORATION_MAP_UNKNOWN`·`SCOUT_POSE_UNKNOWN` | 「완성 지도(두 번째 고정) 대기 — TB-6 복귀·TB-5 출구 이동 보류」·「출구(QR) 없음·TB-5 시작점 모름 — TB-5 출구 이동 보류」·「출구로 갈 길 없음 — TB-5 출구 이동 보류」·「생존자 없음 — 탐색 완료, TB-5 출동 안 함」·「TB-6 지도 전체 탐색 완료」·「탐색 시간 초과 — 탐색 종료」·「TB-6 탐색 지도 아직 없음 — 탐색 보류」·「TB-6 위치 모름 — 탐색 보류」(보류·초과는 주의색) | 임무 띠(`SUPPORT_*` 는 TB-5 카드, `SCOUT_*` 는 TB-6 카드에도) |
| 두 로봇 교통 관제(#272): `TRAFFIC_SUPPORT_WAITING_SCOUT_STOP`·`TRAFFIC_SCOUT_STOP_TIMEOUT`·`TRAFFIC_SUPPORT_WAITING_SCOUT_YIELD`·`TRAFFIC_SCOUT_YIELD_NO_SPACE`·`TRAFFIC_SCOUT_HELD_FOR_SUPPORT`·`TRAFFIC_ROBOTS_TOO_CLOSE`·`TRAFFIC_SUPPORT_PATH_UNKNOWN`·`TRAFFIC_SCOUT_POSE_UNKNOWN`·`TRAFFIC_SUPPORT_POSE_UNKNOWN`·`TRAFFIC_SEPARATION_UNCHECKED` | 「TB-6 정지 확인 기다림 — TB-5 대기」·「TB-6 정지 확인 안 됨(시간 초과) — TB-5 대기, 현장 확인」(빨간색)·「TB-6 이 TB-5 경로 밖으로 비키는 중 — TB-5 대기」·「TB-6 이 비킬 자리 없음 — 두 로봇 모두 정지」(빨간색)·「TB-5 경로와 겹쳐 TB-6 명령 보류」·「두 로봇이 너무 가까움 — 새 명령 보류」·「고정 지도에서 TB-5 경로를 찾지 못함 — 대기」·「TB-6 위치 확인 안 됨 — 대기」·「TB-5 위치 확인 안 됨 — 대기」(나머지 보류는 주의색)·「TB-5 출발 자리 미측정 — 두 로봇 간격 검사 꺼짐」(표시만) | 임무 띠 |

SUPPORT 쪽 코드는 #187(head 4528226), 재확인·판정·TB-6 복귀 코드는 #194(head 1fc7770)와 대조했다. 위에 없는 코드는 코드 그대로 보인다.

## 대상 소실 후보 (2026-10-13)

고정 웹캠이 감시하던 대상을 놓치면 `SiteEvent` `TARGET_DISAPPEARED_CANDIDATE`(대상·감시 ID 포함)가 현장 상태 DB 에 들어온다. 관제 화면은 이를 두 곳에 보인다.

- 대상 줄: 그 대상의 가장 최근 후보 시각을 「소실 후보 HH:MM:SS」로 강조한다. 툴팁에 카메라·사건 ID 와 「후보일 뿐, 로봇 재확인으로 구출을 판정」을 적는다. 구출 확인은 로봇 재확인 결과(`rescue_state`)로만 바뀐다.
- 사건 기록: 「대상 소실 후보」 행의 대상 칸에 그 대상 ID 를 적는다.
- 후보가 가리키는 대상이 목록에 없으면 「확인할 것」 줄에 「소실 후보의 대상이 목록에 없음: <ID>」를 띄운다.

## 지난 회차 다시 보기 (`/replay`, 2026-10-15)

관제 화면 맨 위 줄의 「지난 회차 다시 보기」를 누르면 열린다. `--site-state` 로 띄웠을 때만 쓸 수 있고, 로그인한 모든 계정이 볼 수 있다.

- 위 칸에서 회차를 고른다. 처음에는 진행 중인 회차가, 없으면 마지막 회차가 골라져 있다. 회차는 현장 상태 DB 스키마 1.5 의 `runs` 표다(#318).
- 재생·멈춤, 배속(1·2·4·8배), 시간 막대가 있다. 막대 아래 점은 사건·명령·오류·출구·인형 단계 바뀜 하나씩이고, 누르면 그 시각으로 간다.
- 왼쪽 칸은 그 시점의 TB-5·TB-6 위치·배터리·상태와 인형 4단계, 출구 관측과 출구 통과 수다. 로봇과 인형 값은 그 시각 이전의 가장 최근 이력 행이고, 로봇 값은 몇 초 전 값인지 같이 보인다. 5초보다 오래되면 「낡음」이다.
- 지도 그림은 아직 회차별로 남지 않아 「지도 이력 없음」으로 보인다.
- 오른쪽 칸은 그 시점까지의 기록이다. 명령은 접수·거절과 사유를 보인다.
- 자세한 동작은 [설계 명세 7.13](../../docs/design/console-spec.md)에 있다.

## 회차 초기화 (`/reset`, 2026-10-15)

관제 화면 맨 위 줄의 「회차 초기화」(ADMIN 에게만 보임)를 누르면 열린다. `--site-state` 로 띄웠을 때 쓸 수 있고, 보내기는 `--operations` 도 있어야 한다. 다른 계정은 보기만 한다.

- 초기화는 TB-6 SLAM 지도와 지도 고정까지 처음으로 되돌린다. 지난 기록은 지우지 않고 회차별로 남아 「지난 회차 다시 보기」에서 볼 수 있다.
- 확인 칸을 체크하고 「초기화 보내기」를 누르면 operations 에 한 번만 보낸다. 거절되면 빨간 글씨로 사유가 나온다. 초기화는 두 로봇이 켜지고 도킹·정지·임무 없음이어야 받는다. 결과를 모르면 노란 글씨가 나오고, 다시 누르기 전에 리플레이에서 회차가 바뀌었는지 본다.
- 접수되면 아래에 PC 별 다시 띄우기 체크 목록이 나온다. 조정기와 두 로봇 실행기가 다시 떴는지는 자동으로 체크된다. SLAM·지도 고정·출발 정렬 같은 칸은 사람이 「확인함」을 누른다. 체크는 그 회차에만 묶이고 모든 화면에 같게 보인다.
- 체크는 관제 웹 계정 DB(스키마 1.1, `reset_checks` 표)에 남는다. 지금 쓰는 1.0 계정 파일은 관제 웹이 처음 열 때 1.1 로 올린다. 계정 행은 그대로다.
- 자세한 동작은 [설계 명세 7.14](../../docs/design/console-spec.md)에 있다.

## 하지 않는 것 / 후속

- 하지 않음: 운영 장부 쓰기, ROS 구독, 로봇에 직접 명령, 명령 재전송, 카메라 원본 스트리밍. 로그인·계정은 위 절(콘솔 소유 DB 만 쓴다).
- 오류: `ConsoleError` 는 화면 안의 지역 오류이고 `RescueError`(계약 오류)가 아니다. `rescue_contracts.DETAIL_CATEGORIES` 에 등록하지 않는다 — 계약 코드 목록은 로봇·판정·보고 경로에만 쓴다(작전 결정, 2026-10-08).
- 후속: 일시정지·재개·취소(Q-22 뒤), 정지 버튼(RequestStop 경로), 재난 뒤 SLAM 지도 갱신 표시(IF-084·`map_snapshots`), 카메라 주소를 프로파일에서 받기(#39), 장부 이어 읽기(`read_reports` 커서).
- 강사 day5 예제는 [docs/process/source-selection.md](../../docs/process/source-selection.md) TS-02(조회 → 표의 최소 구조, DB 는 operations API 뒤)·TS-03(폴링으로 KPI-04 를 만족하면 push 를 쓰지 않음)의 판단만 따랐다. 로그인 페이지·로그인 데이터 DB 라는 **요구**는 따르고(4_login.py·welcome 표의 역할·상태 열), 하드코딩 자격 증명·고정 secret·평문 비교는 쓰지 않았다. 옮긴 줄은 없다.

## 기록 (2026-10-14)

지금 코드와 맞지 않게 된 설명을 옮겨 둔다(OPS-211).

- 2026-10-13 까지 「구성」 표의 `app.py` 줄: 「데이터 경로는 `GET /`, `GET /fragment`, `GET /api/view`, `GET /map.png`, 명령은 `POST /commands` 하나」. 웹캠 고르기(#257)와 로봇 카메라 고르기(#261)가 쓰기 경로를 하나씩 더했다.
- 2026-10-13 의 「새 회차는 새 현장 상태 DB 로 띄운다」 예시는 개인 작업 폴더 경로를 적었다. 지금은 `<회차 폴더>` 자리표시자로 쓴다.


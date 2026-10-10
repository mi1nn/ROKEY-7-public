# rescue_console — 관제 화면 (읽기 전용)

운영 장부와 상태 스냅샷을 **읽어서** 한 화면에 보여 준다(BRD Part I 3.7.5). 2026-10-08 첫 판, 담당 자리: 관제·화면.

- 데이터의 진실은 협업·관제의 운영 장부(`rescue_operations`)다. 이 패키지는 DB 에 쓰지 않고, DB 파일을 만들지 않고, ROS 를 구독하지 않고, 로봇에 명령을 보내지 않는다.
- **현장 정지 수단을 웹 연결에만 의존시키지 않는다**(팀 `AGENTS.md` 7절). 이 화면이 꺼지거나 멈춰도 로봇 쪽 정지(통신 단절 watchdog, 현장 비상 정지)는 따로 동작해야 하고, 화면은 그 대체물이 아니다.

## 구성

| 모듈 | 하는 일 | 의존 |
| --- | --- | --- |
| `view.py` | 장부 + 상태 → 화면 모델(dict). 4단계 합성 규칙표, 명령 자리 | 표준 라이브러리, `rescue_contracts` |
| `sources.py` | `rescue_operations.ledger.read_reports` 로 장부를, `rescue_operations.site_state` 읽기 함수 셋으로 현장 상태 DB 를 읽기 전용(`mode=ro`, `query_only`)으로 읽음. 현장 상태 DB 대신 픽스처 JSON 도 받음(E2) | `rescue_operations` |
| `render.py` | 화면 모델 → HTML 한 장. Flask 없이 시험 | 표준 라이브러리 |
| `app.py` | Flask 경계. `GET /`, `GET /fragment`, `GET /api/view` 만 | Flask (선택) |

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
    --status src/rescue_console/examples/status.example.json --port 8080
# 브라우저: http://127.0.0.1:8080/  → 로그인 페이지
```

operations 가 현장 상태 DB 를 쓰고 있으면 픽스처 대신 그 파일을 준다. 둘을 같이 주면 시작하지 않는다.

```bash
python3 -m rescue_console <운영 장부> --accounts <계정 DB> --site-state <현장 상태 DB> \
    --camera cam_a=http://<카메라 PC>:<포트>/<경로> --port 8080
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

## 화면 위쪽: 모드 배지와 웹캠

- **모드 배지:** 현장 사건에 `DISASTER_SIGN` 이 하나라도 있으면 「재난」(빨강), 없으면 「평시」, 현장 사건 입력이 없으면 「모름」. 트리거 시각은 가장 이른 `DISASTER_SIGN` 의 `observed_at_ns` 다. 현장 상태 1.1 의 `mission_state` 표가 생기면 그 값으로 바꾼다 — 지금은 현장 사건에서 **추정**한 것이고 화면에도 그렇게 적는다.
- **웹캠:** `--camera` 로 받은 MJPEG 주소를 `<img src>` 로 넣는다(IF-082). 콘솔은 영상을 중계하지 않는다. 영상은 1초마다 바뀌는 조각(`#live`) **밖**에 있어 폴링이 스트림을 끊지 않는다. 영상이 끊기면 캡션에 「영상 끊김」을 띄운다. 카메라 주소는 프로파일 v2.1 `site_cameras`(#39)가 들어오면 거기서 받는다.

## 시각과 나이 표시

- 시각은 `HH:MM:SS.mmm`(서버·브라우저 같은 형식, 24시간). 서버 집계 날짜와 다른 날의 시각은 날짜를 붙인다. 원래 ns 값은 마우스를 올리면(title) 보인다.
- 나이는 `0.4s` · `3분 05초` · `2시간 10분` · `41일` 로 줄여 쓴다. 음수는 상대 시계가 이 PC 보다 앞선 것이다.
- 로봇 행은 operations 수신 뒤 `STATUS_STALE_S = 5` 초가 지나면 「낡음」으로 표시한다([제안] 관제·화면 2026-10-10, 표시 전용 — 준비 판정 아님 C-05). 프로파일 v2.1(#39)의 readiness `max_age_s` 가 들어오면 그 값으로 바꾼다.

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

일시정지·재개·작업 취소·복귀(Part I 3.7.6) 버튼은 `disabled` 로만 있다. 누를 수 없고, 명령을 받는 경로(POST 라우트)도 없고, `view.request_command` 는 항상 `NOT_CONNECTED` 로 거절한다. 명령 의미와 안전 관문은 Q-22 결정 뒤 협업·관제가 정한 API 로 연결한다.

## 하지 않는 것 / 후속

- 하지 않음: 운영 장부 쓰기, ROS 구독, 명령 전송, 카메라 원본 스트리밍. 로그인·계정은 위 절(콘솔 소유 DB 만 쓴다).
- 오류: `ConsoleError` 는 화면 안의 지역 오류이고 `RescueError`(계약 오류)가 아니다. `rescue_contracts.DETAIL_CATEGORIES` 에 등록하지 않는다 — 계약 코드 목록은 로봇·판정·보고 경로에만 쓴다(작전 결정, 2026-10-08).
- 후속: 지도와 마커(별도 PR), 현장 상태 1.1(`mission_state`·`error_log`·`operator_commands`) 읽기, 대상 소실 후보(`TARGET_DISAPPEARED_CANDIDATE`, #48) 표시, 카메라 주소를 프로파일에서 받기(#39), 장부 이어 읽기(`read_reports` 커서), KPI-04 자동 측정, 오류 이력 표(3.7.4), 명령 패널(operations `POST /api/operator-commands` 뒤).
- 강사 day5 예제는 [docs/process/source-selection.md](../../docs/process/source-selection.md) TS-02(조회 → 표의 최소 구조, DB 는 operations API 뒤)·TS-03(폴링으로 KPI-04 를 만족하면 push 를 쓰지 않음)의 판단만 따랐다. 로그인 페이지·로그인 데이터 DB 라는 **요구**는 따르고(4_login.py·welcome 표의 역할·상태 열), 하드코딩 자격 증명·고정 secret·평문 비교는 쓰지 않았다. 옮긴 줄은 없다.

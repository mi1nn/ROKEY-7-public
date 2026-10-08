# rescue_console — 관제 화면 (읽기 전용)

운영 장부와 상태 스냅샷을 **읽어서** 한 화면에 보여 준다(BRD Part I 3.7.5). 2026-10-08 첫 판, 담당 자리: 관제·화면.

- 데이터의 진실은 협업·관제의 운영 장부(`rescue_operations`)다. 이 패키지는 DB 에 쓰지 않고, DB 파일을 만들지 않고, ROS 를 구독하지 않고, 로봇에 명령을 보내지 않는다.
- **현장 정지 수단을 웹 연결에만 의존시키지 않는다**(팀 `AGENTS.md` 7절). 이 화면이 꺼지거나 멈춰도 로봇 쪽 정지(통신 단절 watchdog, 현장 비상 정지)는 따로 동작해야 하고, 화면은 그 대체물이 아니다.

## 구성

| 모듈 | 하는 일 | 의존 |
| --- | --- | --- |
| `view.py` | 장부 + 상태 → 화면 모델(dict). 4단계 합성 규칙표, 명령 자리 | 표준 라이브러리, `rescue_contracts` |
| `sources.py` | `rescue_operations.ledger.read_reports` 로 장부를 읽기 전용(`mode=ro`, `query_only`)으로 읽고, 상태 픽스처 JSON 을 읽음 | `rescue_operations` |
| `render.py` | 화면 모델 → HTML 한 장. Flask 없이 시험 | 표준 라이브러리 |
| `app.py` | Flask 경계. `GET /`, `GET /fragment`, `GET /api/view` 만 | Flask (선택) |

Flask 는 선택 의존이다(`extras_require={"web": ["flask"]}`). 없으면 `create_app` 이 `DEPENDENCY_MISSING` 오류로 끝나고 다른 방식으로 대신 뜨지 않는다. 집계·HTML·명령 무동작 시험은 Flask 없이 돈다.

## 실행 (로봇 없이, E2)

```bash
python3 -m pip install flask                      # 웹으로 볼 때만
export PYTHONPATH=$(ls -d src/rescue_* | tr '\n' ':')
python3 -m rescue_bringup.offline_demo config/deployment.example.json --output <TMP>/run
python3 -m rescue_console <TMP>/run/operations.sqlite3 \
    --status src/rescue_console/examples/status.example.json --port 8080
# 브라우저: http://127.0.0.1:8080/
```

- 장부 경로·상태 경로·포트·바인드 주소는 인자로 받는다. 기본 바인드는 `127.0.0.1` 이고, 다른 PC 에서 볼 주소는 배치 프로파일(`operations_host`, 작전)이 정한 값을 인자로 넘긴다. 코드에 호스트·IP·개인 경로를 두지 않는다.
- `examples/status.example.json` 은 예시 값이다(오프라인 리허설 장부의 대상 ID 에 맞춤). 실제 값이 아니다.

## 입력

| 입력 | 지금 | 이후 |
| --- | --- | --- |
| 보고 장부 | `read_reports(db, limit=1000)` 결과 | 그대로 |
| 로봇 상태 | 픽스처의 `robot_status` | `rescue_operations.site_state.read_robot_status(site_db)` (협업·관제 PR #34) |
| 현장 사건·출구 누적 | 픽스처의 `site_events` | `read_site_events(site_db)` |
| 대상 상태(구조·지원) | 픽스처의 `target_states` | `read_target_states(site_db)` |

픽스처의 세 칸은 위 함수의 반환 형식 그대로다(협업·관제와 2026-10-08 합의). 칸이 없으면 기본값으로 채우지 않고 화면 경고 「<칸> 입력 없음」으로 보인다. 출구 누적 수는 `exit_pass_count` 를 그대로 쓰고 화면에서 다시 세지 않는다(구출 판정과 같은 숫자를 쓰기 위해).

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
- 구출 완료 수가 출구 누적보다 크면 경고한다(Part I 3.5.5). 화면은 막지 않고 보여만 준다.

## KPI-04 측정 (표시 지연 3초)

- **정의:** 지연 = 브라우저가 그 이벤트 행을 처음 그린 시각 − 이벤트 시각(`observed_at_ns`). 이벤트 시각은 로봇·카메라 시계, 표시 시각은 브라우저 PC 시계라 **두 시계가 맞을 때만**(C-03, 같은 시계 영역 또는 동기화 오차 안) 의미가 있다. 행마다 시계 영역(`clock_domain_id`)을 같이 보인다.
- **기록 위치:** 이벤트 표에 이벤트 시각·장부 기록 시각(`received_at_ns`)·화면 표시 시각·지연을 같이 둔다. 브라우저 `window.kpi04` 배열에 `{ref_id, event_ns, rendered_ms}` 가 처음 그린 순서대로 쌓인다(개발자 도구에서 `copy(JSON.stringify(kpi04))`). 서버 집계 시각은 `data-built-ns` 에 있다.
- **폴링:** 브라우저가 `POLL_INTERVAL_MS = 1000` 마다 `GET fragment` 를 받아 바꾼다. 3초 예산 중 폴링 대기 최대 1초, 나머지가 장부 기록·읽기·전송이다. 갱신이 실패하면 화면 위에 「갱신 실패 — 표시 값이 낡았을 수 있음」을 띄운다.
- 지연 구간 나누기: 이벤트 → 장부 기록(`received_at_ns − event_ns`, 장부 PC 시계) / 장부 기록 → 화면(`rendered − received_at_ns`, 화면과 장부가 같은 PC 일 때). 자동 측정 스크립트는 후속이다.

## 운영자 명령 (자리만)

일시정지·재개·작업 취소·복귀(Part I 3.7.6) 버튼은 `disabled` 로만 있다. 누를 수 없고, 명령을 받는 경로(POST 라우트)도 없고, `view.request_command` 는 항상 `NOT_CONNECTED` 로 거절한다. 명령 의미와 안전 관문은 Q-22 결정 뒤 협업·관제가 정한 API 로 연결한다.

## 하지 않는 것 / 후속

- 하지 않음: DB 쓰기, ROS 구독, 명령 전송, 카메라 원본 스트리밍, 로그인(Q 결정 전).
- 오류: `ConsoleError` 는 화면 안의 지역 오류이고 `RescueError`(계약 오류)가 아니다. `rescue_contracts.DETAIL_CATEGORIES` 에 등록하지 않는다 — 계약 코드 목록은 로봇·판정·보고 경로에만 쓴다(작전 결정, 2026-10-08).
- 후속: site-state DB 경로 인자와 PR #34 읽기 함수 연결, 지도 스냅샷 표시(로봇·지도 제공 뒤), 카메라 A·B 영상 두 창(인식 MJPEG 합의 뒤), KPI-04 자동 측정, 오류 이력 표(3.7.4, `read_errors`), 명령 연결(Q-22 뒤).
- 강사 day5 예제는 [docs/source-selection.md](../../docs/source-selection.md) TS-02(조회 → 표의 최소 구조, DB 는 operations API 뒤)·TS-03(폴링으로 KPI-04 를 만족하면 push 를 쓰지 않음)의 판단만 따랐다. 예제 코드는 열어 보지 않았고 옮긴 줄이 없다.

# rescue_operations

## 현재 상태 (2026-10-14, main 3278a3e)

| 기능 | 상태 | 근거 |
| --- | --- | --- |
| 현장 상태 DB 쓰기(스키마 1.5) | 구현됨 | `site_state.py:SiteStateLedger`, `ros_node.py`(`db_writer`), #262 |
| 회차와 리플레이 이력(스키마 1.5) | 구현됨 | `site_state.py`(`runs`, `run_id`, `read_*_range`), [DB 스키마 4-1절](../../docs/design/db-schema.md) |
| 초기화 API(`POST /api/runs/reset`) | 구현됨 · 실기 NOT_RUN | `command_api.py:RunReset`, `site_state.py:SiteStateLedger.reset_run` |
| 리플레이 지도 보관(`map_dir/<run_id>/`) | 구현됨 · 실기 NOT_RUN | `map_archive.py`, `db_writer` 파라미터 `map_archive_topic`·`map_archive_dir` |
| 운영자 명령 API(`POST /api/operator-commands`) | 구현됨 · 실기 NOT_RUN | `command_api.py:CommandGate`, #115 |
| 웹캠 고르기 API(`GET`·`POST /api/webcam`) | 구현됨 · 실기 일부: 운영 PC 카메라 2대 전환 [측정 2026-10-13] | `command_api.py:WebcamSwitch`, #257 |
| 구조 보고 저장(`report_node`) | 구현됨 | `report_node.py`, `report_store.py`, #137 |
| SLAM 지도 파일 쓰기(`map_snapshot`) | 구현됨 · 실기 일부: 시험용 TB-5 SLAM 지도 [측정 2026-10-13] | `map_snapshot.py`, #209 |
| 로봇 직접 관측 파일 쓰기(`robot_watch`) | 구현됨 · 실기 일부: TB-5·TB-6 수신 [측정 2026-10-13] | `robot_watch.py`, #251 |
| API 인증·원격 HTTP | 미구현(`127.0.0.1` 에서만 연다) | `operations_launch.py` `API_HOST` |

## 현장 상태 DB (스키마 1.5)

- 쓰는 프로세스는 `db_writer` 하나다. 관제 웹은 `site_state.read_*` 로 읽기만 한다(`mode=ro`).
- 표: `mission_state`, `robot_status`, `site_events`, `target_states`, `error_log`, `operator_commands`, `map_snapshots`, `exit_locations`(1.3, 출구(QR) 관측), `overhead_passage_lists`·`overhead_passages`·`current_map`(1.4, 터널 판정과 TB-6 최신 지도 revision, #262). 표마다 누가 쓰고 누가 읽는지는 [DB 스키마](../../docs/design/db-schema.md)에 있다.
- 기존 파일(1.0~1.4)은 새 `db_writer` 가 열 때 1.5 로 올린다(`UPGRADES_FROM`). 1.4 파일의 쌓인 행은 닫힌 「이전 기록」 회차 하나로 묶인다. 그 전까지 관제 웹의 읽기 함수는 최신 스키마만 받으므로 화면이 503 응답을 낸다.
- 회차는 한 DB 안에서 `runs` 표와 `run_id` 로 나눈다(1.5). 같은 파일로 다시 띄우면 열린 회차를 잇는다. 화면용 읽기 함수는 열린 회차의 사건·명령·오류·지도·출구만 준다. 지난 회차는 지우지 않고 리플레이 읽기 함수(`list_runs`, `read_*_range`)로 읽는다.

## HTTP API (`db_writer` 와 같은 프로세스, `127.0.0.1`)

| 경로 | 하는 일 | 근거 |
| --- | --- | --- |
| `POST /api/operator-commands` | 관제 웹의 운영자 명령을 조정기 서비스로 넘기고 답을 기록한다. 받는 명령은 `START_SEARCH`·`DISPATCH_SUPPORT`·`CONTINUE_SEGMENT`·`REQUEST_RECHECK`·`RETURN_HOME`·`SET_CAMERA_WATCH` 다 | `command_api.py:COMMAND_FIELDS` |
| `GET /api/webcam` | 운영 PC 에 꽂힌 카메라(`/dev/v4l/by-id/*-video-index0`)와 지금 장치를 준다 | `command_api.py:WebcamSwitch.state` |
| `POST /api/webcam` | `{device, operator_id}` 로 `webcam_node` 의 `device_path` 를 바꾼다. 꽂혀 있지 않은 장치는 거절한다(`UNKNOWN_DEVICE`). 결과는 `error_log` 에 남는다 | `command_api.py:WebcamSwitch.switch` |
| `POST /api/runs/reset` | 관제 웹의 초기화 버튼. `{command_id, operator_id[, note]}` 를 받아 두 로봇이 쉬고 있으면 새 회차를 연다. 거절 이유는 `ROBOT_STATUS_STALE`·`ROBOT_MOVING`·`TASK_ACTIVE`·`ROBOT_NOT_DOCKED`. 답은 운영자 명령과 같은 모양에 `run_id` 를 더한다. 로봇 쪽 재기동은 사람이 한다([운영 구현](../../docs/implementation/implementation-operations.md) 「초기화」) | `command_api.py:RunReset` |

웹캠 고르기는 운영 launch 가 `webcam_params_file` 로 웹캠을 띄울 때만 켜진다(`db_writer` 파라미터 `webcam_node`, 빈 값이면 끔).

ROS 없이 실행할 수 있는 SQLite 보고 저장소, 증거 bytes 저장, 송신 outbox와 로컬 JSON 내보내기를 제공한다. C-02·C-05·C-08 및 CT-12의 로컬 구현이다. 사진 필수 여부는 확정하지 않고 저장소 생성자가 `evidence_policy`를 명시하게 한다.

```python
from rescue_operations.ledger import ReportLedger, ReportOutbox, LedgerTransport

allowed_robot_ids = ("robot_a", "robot_b")  # 예시. 실제 값은 검증된 deployment.robots에서 주입
with ReportLedger("operations.sqlite3", evidence_policy="OPTIONAL", allowed_robot_ids=allowed_robot_ids) as ledger:
    with ReportOutbox("sender.sqlite3", allowed_robot_ids=allowed_robot_ids) as outbox:
        outbox.enqueue(payload)  # schema 1.0의 검증된 보고 dict
        receipt = outbox.deliver(payload["report_id"], envelope, LedgerTransport(ledger))
        # RECEIVED/PENDING_EVIDENCE는 보고 완료가 아니다.
```

`payload`와 `envelope`의 고정 필드, 예시, 실패 의미는 [구현 문서](../../docs/implementation/implementation-operations.md)에 있다. `OPTIONAL`은 예시 선택이며 운영 사진 정책의 결정이 아니다. 필요한 증거가 선언됐으면 OPTIONAL에서도 실제 bytes 저장을 기다린다.

```bash
PYTHONPATH=src/rescue_contracts:src/rescue_operations python3 -m rescue_operations operations.sqlite3 --limit 100
# 설치한 패키지:
rescue-report-export operations.sqlite3 --report-id <report-uuid>
```

CLI는 operations 호스트의 기존 DB를 읽어 JSON을 출력한다. 없는 DB를 만들거나 보고를 변경하지 않는다. `PERSISTED`는 보고와 선택한 정책의 증거가 로컬 SQLite transaction으로 저장됐다는 뜻이며, 접근의 물리적 성공이나 사람 구조·생존을 판단하지 않는다.

operations DB의 소유자는 operations 호스트다. 다른 PC는 DB 파일을 마운트하거나 직접 쓰지 않는다. 송신 측 outbox는 별도 로컬 DB이고 API를 통해 같은 보고 ID를 재전송한다. 보고 장부의 API 어댑터는 프로세스 내부 callable이다. 현장 상태 DB 에는 ROS 구독 노드 `db_writer`(로봇 상태·위치·고정 카메라 사건·임무 상태·대상 목록, [구현 문서](../../docs/implementation/implementation-operations.md) 「DB 쓰기 어댑터」)가 있다. 같은 프로세스가 관제 웹의 운영자 명령을 받아 조정기 서비스로 넘기는 HTTP API(`POST /api/operator-commands`, 같은 문서 「운영자 명령 API」)를 `127.0.0.1` 에서 연다. API 자체의 인증·원격 HTTP·실기·4-PC 통신은 아직 구현하거나 검증하지 않았다. 다른 시스템의 Nav2 goal이나 속도 명령을 발행하지 않는다.

## ROS 노드

운영 PC 에서 `rescue_bringup` 의 `operations.launch.py` 가 띄운다(#144). `db_writer`·`report_node` 는 늘 뜨고, `map_snapshot`(인자 `map_topic`·`map_dir`)과 `robot_watch`(인자 `robot_namespaces`·`robot_watch_path`)는 인자를 줄 때만 뜬다. 같은 launch 가 인식 패키지(`rescue_perception`)의 `webcam_node`·`mjpeg_relay`·`robot_view_relay` 도 띄운다(인자를 줄 때). 두 DB 는 서로 다른 파일이어야 한다(파일당 쓰는 프로세스 하나).

- `db_writer`(`ros_node.py`): 현장 상태 DB 를 쓰는 유일한 프로세스다.
  - 받는 것: 로봇 상태·위치, 고정 카메라 사건, 조정기 `SystemState`·`TargetSnapshot`, 출구(QR) 관측 `TargetLocation`, 로봇별 터널 판정(`overhead_passages`, JSON 문자열), TB-6 지도 상태 `MapState`. 대상 행에 `target_revision`·`persisted_report_id` 를 남긴다.
  - 같은 프로세스가 `127.0.0.1` 에서 HTTP API 를 연다(위 「HTTP API」).
- `report_node`(`report_node.py`, #137): 조정기 `RescueReport` 를 보고 저장 DB(`report_store.RescueReportStore`)에 저장하고 `ReportReceipt` 를 낸다.
  - `PERSISTED` 는 그 보고가 이 파일에 commit 됐다는 뜻이다. 그 영수증이 TB-5 출동 승인의 "저장" 조건이다.
  - 같은 `report_id`·같은 내용은 처음 영수증을 그대로 다시 낸다. 내용이 다르면 `REJECTED`(ID_CONTENT_CONFLICT)다.
  - 보고 하나의 한도는 1 MiB 다. 조정기가 저장 안 된 위치만 싣게 된 뒤(#139) 위치 42개 대상의 가장 큰 보고가 4,021 B 였다 [측정 2026-10-13].
  - 이 장부는 위의 `ReportLedger`(schema 1.0 보고)와 다르다. IDL 보고에는 없는 필드를 지어내지 않으려고 따로 둔다.
  - 조정기가 보내는 `task_id=''`·`generation=0`·`evidence_refs=[]` 는 그대로 저장한다. 증거는 `locations[].observation.evidence` 에 있다(작전 결정 2026-10-13, 채우지 않음).
- 종료: Ctrl-C 를 두 번 받아도(launch + 터미널) 정리를 마치고 exit 0 으로 끝난다(#143, `ros_node.teardown`). 고치기 전에는 두 번째 SIGINT 가 정리 중에 오면 traceback 과 exit -2 로 끝났다 [측정 2026-10-13].
- 읽기: 관제 웹은 `site_state.read_*`(읽기 전용, `mode=ro`)로 DB 를 읽는다. 배치 로봇 목록은 `read_allowed_robot_ids`(#149)로 읽는다.
- `map_snapshot`(`map_snapshot.py`, #209): SLAM 지도(`OccupancyGrid`)를 관제 웹 `--live-map` 이 읽는 지도 파일로 쓴다. 발행자가 있는 동안 `live_map.alive` 를 갱신한다.
- 지도 보관(`map_archive.py`, `db_writer` 안): 운영 launch 에 `map_topic`·`map_dir` 을 주면 `db_writer` 도 같은 SLAM 지도와 고정 지도(`/rescue/map/frozen`·`frozen_revision`)를 구독한다. 열린 회차의 `map_dir/<run_id>/` 에 PGM·YAML 을 남기고 `map_snapshots` 에 한 줄씩 쓴다. 실시간 지도는 5초에 한 장 [제안]이고 같은 그림은 다시 남기지 않는다. 고정 지도는 revision 글이 그 지도의 해시를 가리킬 때 늘 남긴다. TB-6 `MapState` 를 받기 전과 회차 사이에는 남기지 않는다.
- `robot_watch`(`robot_watch.py`, #251): `<ns>/battery_state`·`dock_status`·`tf`·`tf_static` 을 구독만 하고(로봇에 아무것도 보내지 않음) 관제 웹 `--robot-watch` 가 읽는 파일을 1초마다 쓴다.

원본 코드를 복사하지 않은 새 구현이다. 패키지 버전 `0.0.0`, `LicenseRef-Unspecified`, 미지정 maintainer는 기존 기초 구조의 미결정 메타데이터를 유지한다. 배포 권한이나 라이선스 허가를 뜻하지 않는다.

## 프로파일 ID와 오류 계약

`source_robot`와 `robot_id`의 형식은 공통 `rescue_contracts.robot_id`로 검사한다. 저장소·outbox 생성자는 검증된 프로파일에서 정확히 두 개의 서로 다른 ID를 `allowed_robot_ids`로 받아야 하며 기본값은 없다. `robot3`/`robot7` 같은 ID도 가능하고, 두 필드 중 하나라도 허용 집합 밖이면 `WRONG_ROBOT`으로 쓰기 전에 거절한다. 허용 집합은 정렬한 불변 tuple로 보존하고 읽기 전용 속성으로 제공한다.

DB metadata에는 허용 ID 집합을 사진 정책과 함께 고정한다. 순서만 다른 같은 집합으로는 재오픈할 수 있고, 집합 변경이나 허용 ID metadata가 없는 이전 DB는 `POLICY_MISMATCH`다. 기존 DB에 현재 설정을 조용히 덮어쓰거나 과거 보고를 바꾸지 않는다. 필요한 데이터 이전은 원본 보존 후 별도 명시적 migration으로 다뤄야 한다. 읽기 전용 export도 저장된 `allowed_robot_ids`를 함께 표시한다.

`OperationsError`는 공통 `RescueError`이며 `.code`는 C-09, `.detail_code`는 `rescue_operations.<원 세부 코드>`다. 예를 들어 저장 실패는 `REPORT_NOT_PERSISTED`/`rescue_operations.STORAGE_ERROR`로 표현한다. `tests/test_review_operations.py`는 사용자 지정 ID, 알 수 없는 원본/수행 로봇, 정책 불변·재오픈 불일치·기존 metadata 부재를 실제 임시 SQLite로 검사한다. 기존 실행 증거·source hash는 수정하지 않는다.

## 기록 (2026-10-14)

지금 코드와 맞지 않게 된 설명을 옮겨 둔다(OPS-211).

- 2026-10-13 절 제목 「ROS 노드 두 개」. 그 뒤 `map_snapshot`(#209)과 `robot_watch`(#251)가 더해졌다.
- 2026-10-13 의 `db_writer` 설명: 「받는 명령은 `START_SEARCH`·`DISPATCH_SUPPORT`·`CONTINUE_SEGMENT`·`REQUEST_RECHECK`·`RETURN_HOME` 이다. `SET_CAMERA_WATCH` 는 PR #148 머지 뒤에 받는다.」 지금은 `SET_CAMERA_WATCH` 도 받는다(`command_api.py:COMMAND_FIELDS`).
- 2026-10-13 의 `db_writer` 「받는 것」에는 출구(QR) 관측, 터널 판정, TB-6 지도 상태가 없었다(#237·#262 에서 더함).
- 로봇 이름 「AMR2」는 「TB-5」로 바꿨다(재범 규칙 2026-10-13).


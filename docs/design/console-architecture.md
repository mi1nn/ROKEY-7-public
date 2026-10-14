---
title: 관제 웹 아키텍처 — 구성과 경계, 데이터 흐름, 핵심 함수와 클래스, 설계 판단
status: draft
updated: 2026-10-14
---

# 관제 웹 아키텍처

> 기준: main `ccadedc`(2026-10-14) + PR #284. 관제 웹 코드는 `4163a25` 와 같고(README 만 다름), 줄 번호는 이 상태 기준이다. `rescue_operations/ros_node.py` 줄 번호는 PR #284 의 주석이 들어간 뒤의 것이다.
> 이 문서는 「관제 웹 데이터 흐름」(`console-dataflow.md`)의 이름을 바꾸고 구성·클래스·설계 판단·알려진 한계를 더한 것이다. 처리 규칙·화면 문구·오류의 자세한 내용은 [관제 웹 설계 명세](console-spec.md)에 있다.
> 코드를 읽어 적었다(E1). 로봇·ROS 를 띄워 토픽을 확인하지 않았다(NOT_RUN).

## 0. 먼저 한 줄

관제 웹(`rescue_console`)은 **ROS 노드가 아니다.** 토픽을 구독하지도 발행하지도 않는다. ROS 쪽 값은 운영 PC 의 노드들이 **파일·SQLite·HTTP** 로 바꿔 두고, 웹은 이 값을 **요청마다 읽기만** 한다. 웹이 밖으로 내보내는 것은 넷이다: 브라우저에 주는 HTTP 응답, operations HTTP API 로 보내는 요청(운영자 명령 POST, 웹캠 목록 GET·바꾸기 POST), 로봇 카메라 중계(`robot_view_relay`)의 `/select` 요청(GET·POST), 자기 계정 DB 쓰기.

- 확인한 방법: `grep -rn -E "import rclpy|from rclpy" src/rescue_console` → 0건. 밖으로 나가는 HTTP 는 모두 `post_json`(`commands.py:116-139`) 안의 `urllib.request.urlopen` 한 곳(`commands.py:123`)을 지나고 재시도가 없다. 쓰는 SQL 은 계정 DB(`accounts.py:150`, `:152`, `:242`, `:282`, `:297`)뿐이다.

## 1. 구성 요소와 경계

| 실행 단위 | 어디서 | 하는 일 | 쓰는 파일 |
| --- | --- | --- | --- |
| 관제 웹 `rescue_console`(Flask, ROS 노드 아님) | 운영 PC | 요청이 올 때마다 DB·파일을 읽어 화면을 만든다. 운영자 명령은 operations API 로 한 번 넘긴다 | 계정 DB 만 |
| `rescue_db_writer`(rescue_operations `db_writer`) | 운영 PC | ① 토픽을 구독해 현장 상태 DB 에 쓴다. ② 같은 프로세스 안의 스레드로 operations HTTP API(`/api/operator-commands`, `/api/webcam`)를 연다. 주소는 `127.0.0.1`(`rescue_bringup/operations_launch.py:26`) | 현장 상태 DB(유일한 기록자) |
| `rescue_map_snapshot`(rescue_operations `map_snapshot`) | 운영 PC | SLAM 지도 토픽 → `live_map.yaml`·PGM·`live_map.alive` | 지도 파일 |
| `rescue_robot_watch`(rescue_operations `robot_watch`) | 운영 PC | 로봇 기본 토픽을 구독만 해 JSON 파일로 쓴다 | 직접 관측 파일 |
| `mjpeg_relay`(rescue_perception) | 운영 PC | 웹캠 판정 영상 → MJPEG HTTP | — |
| `robot_view_relay`(rescue_perception) | 운영 PC | 고른 로봇 하나의 터널 판정 영상 → MJPEG HTTP, 고르기 `/select` | — |
| `mission_coordinator`(rescue_coordination) | ROS 쪽 | 명령을 받아들이거나 거절하고, 임무 상태·대상을 발행한다 | — |
| 브라우저 | 운영자 PC | 1초마다 `/fragment` 를 요청한다. 영상은 중계 노드에서 직접 받는다 | — |

헷갈리기 쉬운 점:

- 조정기는 영상을 내지 않는다. `src/rescue_coordination` 에 이미지 발행 코드는 0건이다. 영상은 인식 노드 → 운영 PC 중계 → 브라우저다(2절 ③).
- operations API 는 웹 안이 아니라 `rescue_db_writer` 프로세스 안에 있다(`rescue_operations/ros_node.py:300-305`, `make_server` → `serve_forever` 스레드).
- 1초마다 요청하는 쪽은 브라우저 JS 다. Flask 는 요청이 올 때 DB 를 연다.
- 지도 파일과 직접 관측 파일은 Flask 만 읽는다. 브라우저는 `/map.png` 를 받는다.
- 현장 상태 DB 의 표 9개는 한 시점의 값이 아니다(8절).

## 2. 흐름 그림

### ① 상태와 조회

```mermaid
flowchart LR
  subgraph ROS["ROS 쪽 노드"]
    R1["rescue_command_adapter<br/>(rescue_robot task_input)"]
    P1["webcam_node"]
    P2["amr_target → exit_locator"]
    P3["overhead_passage"]
    C1["mission_coordinator"]
    R2["TurtleBot4 기본 토픽<br/>battery_state·dock_status·tf"]
    R3["SLAM /robot6/map"]
  end
  subgraph OPS["운영 PC"]
    W["rescue_db_writer"]
    DB[("현장 상태 DB<br/>SQLite schema 1.4")]
    RW["rescue_robot_watch"]
    MS["rescue_map_snapshot"]
  end
  F["관제 웹 Flask<br/>sources → view → render → app<br/>mapimage"]
  B["브라우저"]
  R1 -- "status·pose·map_state" --> W
  P1 -- "/rescue/site_events" --> W
  P2 -- "/rescue/exit_location" --> W
  P3 -- "overhead_passages" --> W
  C1 -- "/rescue/system_state<br/>/rescue/targets" --> W
  W --> DB
  DB -- "요청마다 읽기 전용<br/>표 9개 따로" --> F
  R2 --> RW -- "JSON 파일" --> F
  R3 --> MS -- "live_map 파일" --> F
  B -- "1초마다 GET /fragment" --> F
  F -- "HTML 조각 · /map.png" --> B
```

### ② 명령과 웹캠 바꾸기 — 답은 같은 길로 돌아온다

```mermaid
flowchart LR
  B["브라우저"]
  F["관제 웹 Flask<br/>app.py → commands.py"]
  subgraph DBW["rescue_db_writer 프로세스"]
    API["operations HTTP API<br/>127.0.0.1"]
  end
  C1["mission_coordinator"]
  P1["webcam_node"]
  DB[("현장 상태 DB")]
  B -- "POST /commands (ADMIN)<br/>POST /webcam (ADMIN)" --> F
  F -- "POST /api/operator-commands<br/>GET·POST /api/webcam<br/>한 번만" --> API
  API -- "ROS 서비스<br/>/rescue/operator_command" --> C1
  API -- "set_parameters<br/>device_path" --> P1
  C1 -. "수락 · 거절" .-> API
  API -. "답 그대로" .-> F
  F -. "결과 표시" .-> B
  API -- "operator_commands · error_log 기록" --> DB
```

### ③ 영상 — 웹 서버를 거치지 않는다

```mermaid
flowchart LR
  WN["webcam_node<br/>판정을 그린 영상"]
  OP["overhead_passage (로봇 PC)<br/>ns/rescue/passage_view/compressed"]
  MJ["mjpeg_relay<br/>/cam_id.mjpg"]
  RV["robot_view_relay<br/>고른 로봇 하나만 구독<br/>/robot_view.mjpg"]
  F["관제 웹 Flask"]
  B["브라우저"]
  WN --> MJ -- "MJPEG" --> B
  OP --> RV -- "MJPEG" --> B
  F -- "영상 주소만 HTML 에" --> B
  B -- "POST /robot-camera (누구나)" --> F
  F -- "GET·POST /select" --> RV
```

## 3. 들어오는 것 ① — ROS 토픽 → 현장 상태 DB → 웹

`rescue_db_writer`(`rescue_operations` `db_writer`, 운영 PC)가 토픽을 받아 현장 상태 DB 에 쓴다. 구독 연결은 `rescue_operations/ros_node.py:214-246`, 토픽 이름은 `rescue_bringup/operations_launch.py:17-24`, `:59-79` 이 정한다. 웹은 `sources.py` `load_site_state`(`sources.py:75-99`)에서 읽기 함수 9개로 이 DB 를 **읽기 전용**(`mode=ro`, `query_only`)으로 읽는다.

경로 표기: 웹 밖 파일은 `<패키지>/<파일>:줄` 이고 실제 위치는 `src/<패키지>/<패키지>/<파일>` 이다(예: `rescue_operations/site_state.py` = `src/rescue_operations/rescue_operations/site_state.py`). 웹 파일은 파일 이름만 적고 위치는 `src/rescue_console/rescue_console/<파일>` 이다.

| # | 발행 노드 (패키지 · 실행 파일 · 발행 줄) | 토픽 · 메시지 | db_writer 처리 → DB 표 | 웹 읽기 (`sources.py` 키) | 웹 처리 (`view.py`) | 화면 |
| --- | --- | --- | --- | --- | --- | --- |
| A1 | `rescue_command_adapter` (rescue_robot · `task_input` · `rescue_robot/task_server.py:100`) | `/rescue/robots/<robot_id>/status` · `RobotStatus` | `on_robot_status`(`rescue_operations/ros_ingest.py:209`) → `robot_status`(`record_robot_status`, `rescue_operations/site_state.py:378`) | `read_robot_status`(`rescue_operations/site_state.py:603`) → `robot_status` | `_robots`(`view.py:237`): 역할·주행·준비·배터리·작업, 5 s 넘으면 낡음 | 로봇 카드(`render.py:327`) |
| A2 | `rescue_command_adapter` (rescue_robot · `task_input` · `rescue_robot/live_pose.py:131`) | `/rescue/robots/<robot_id>/pose` · `RobotPose` | `on_robot_pose`(`rescue_operations/ros_ingest.py:197`) → `robot_status` 행의 위치만 고침(`record_robot_pose`, `rescue_operations/site_state.py:401`) | A1 과 같은 읽기 | `_robots` 의 `pose` | 카드 「위치」, 지도 로봇 화살표 |
| A3 | `webcam_node` (rescue_perception · `webcam_node` · `rescue_perception/webcam_node.py:253`) | `/rescue/site_events` · `SiteEvent` | `on_site_event`(`rescue_operations/ros_ingest.py:221`) → `site_events`(`record_site_event`, `rescue_operations/site_state.py:420`) | `read_site_events`(`rescue_operations/site_state.py:611`, 최대 1000, 오래된 순) → `site_events` | `_site_events`(`view.py:285`), 소실 후보 `_latest_candidates`(`view.py:161`), 사건 목록 `_events`(`view.py:300`), 지진 사건 `_mission`(`view.py:466`) | 사건 목록, 대상 단계, 임무 배너 |
| A4 | `mission_coordinator` (rescue_coordination · `mission_coordinator` · `rescue_coordination/ros_node.py:2289`) | `/rescue/system_state` · `SystemState` | `on_system_state`(`rescue_operations/ros_ingest.py:229`) → `mission_state` 1행(`record_mission_state`, `rescue_operations/site_state.py:546`) | `read_mission_state`(`rescue_operations/site_state.py:640`) → `mission_state` | `_mission_row`(`view.py:350`), 사유 문구 `mission_reason`(`view.py:438`), 버튼 활성 판정 `command_gates`(`view.py:598`) | 임무 배너·사유, 명령 버튼 켜짐/꺼짐 |
| A5 | `mission_coordinator` (`rescue_coordination/ros_node.py:2310`) | `/rescue/targets` · `TargetSnapshot` | `on_target_snapshot`(`rescue_operations/ros_ingest.py:263`) → `target_states`(`record_target_state`, `rescue_operations/site_state.py:435`) | `read_target_states`(`rescue_operations/site_state.py:622`) → `target_states` | `_targets`(`view.py:174`), 4단계 `compose_stage`(`view.py:99`) | 대상 목록, 지도 대상 마커, 출동 승인 선택지 |
| A6 | `amr_target` → `/rescue/exit_observations` → `exit_locator` (rescue_perception · `target_locator` · `rescue_perception/target_locator.py:113`) | `/rescue/exit_location` · `TargetLocation` | `on_exit_location`(`rescue_operations/ros_ingest.py:237`, VALID 만) → `exit_locations`(`record_exit_location`, `rescue_operations/site_state.py:489`) | `read_exit_locations`(`rescue_operations/site_state.py:670`, 최신 1행) → `exit_locations` | `_exit`(`view.py:650`) | 지도 출구(QR) 네모 |
| A7 | `overhead_passage` (rescue_perception · `overhead_passage` · `rescue_perception/overhead.py:348`) | `/rescue/robots/<robot_id>/overhead_passages` · `std_msgs/String`(JSON) | `on_overhead_passages`(`rescue_operations/ros_ingest.py:245`) → `overhead_passage_lists`·`overhead_passages`(`record_overhead_passages`, `rescue_operations/site_state.py:506`) | `read_overhead_passages`(`rescue_operations/site_state.py:681`) → `overhead_passages` | `_passages`(`view.py:629`) | 지도 터널 ∏ 표시(초록·빨강·회색) |
| A8 | `rescue_command_adapter` (rescue_robot · `task_input` · `rescue_robot/live_pose.py:132`), SLAM 로봇(SCOUT) 것만 구독 | `/rescue/robots/<SCOUT>/map_state` · `MapState` | `on_map_state`(`rescue_operations/ros_ingest.py:254`) → `current_map` 1행(`record_current_map`, `rescue_operations/site_state.py:529`) | A7 과 같은 읽기의 `current_map` | `_passages` 의 `current`(지금 지도와 같은 revision 인가) | 지금 지도 터널 진하게, 이전 revision 흐리게 |

토픽이 아니라 operations 가 스스로 쓰는 표:

| 표 | 누가 언제 쓰나 | 웹 읽기 | 웹 처리 · 화면 |
| --- | --- | --- | --- |
| `metadata.allowed_robot_ids` | db_writer 가 기동 때 파라미터 `allowed_robot_ids`(배치 프로파일의 로봇)로 씀(`rescue_operations/site_state.py:373-375`) | `read_allowed_robot_ids`(`rescue_operations/site_state.py:595`) → `allowed_robots` | `build_view` 가 로봇 카드 목록으로 씀(`view.py:713-716`). 상태를 아직 안 보낸 로봇도 카드가 있다 |
| `operator_commands` | 명령 API 가 명령마다 접수·거절을 씀(`rescue_operations/command_api.py:147`, `:158`) | `read_operator_commands`(`rescue_operations/site_state.py:648`, 최근 30) | `_history`(`view.py:499`) → 명령 이력 표 |
| `error_log` | 명령 API 의 실패(`rescue_operations/command_api.py:165`), db_writer 가 거절한 메시지(`rescue_operations/ros_ingest.py:291`) | `read_error_log`(`rescue_operations/site_state.py:629`, 최근 30) | `_history` → 이력 표의 오류 줄 |
| `map_snapshots` | — | 웹은 읽지 않는다 | — |

## 4. 들어오는 것 ② — DB 를 거치지 않는 것

| # | 출발 | 중간 노드 · 산출물 | 웹 인자 · 읽는 곳 | 웹 처리 · 화면 |
| --- | --- | --- | --- | --- |
| B1 | SLAM 지도 `OccupancyGrid`(launch 인자 `map_topic`, 예 `/robot6/map`) | `rescue_map_snapshot`(rescue_operations · `map_snapshot`, 구독 `rescue_operations/map_snapshot.py:139`) → `live_map.yaml` + `map_<해시12>.pgm`(임시 파일 + rename), 발행자가 있는 동안 `live_map.alive` 를 2 s 마다 갱신(`rescue_operations/map_snapshot.py:142-148`) | `--live-map` + `--live-map-stale-s` → `LiveMap.current`(`mapimage.py:233`) | PGM → PNG(`load_map`, `mapimage.py:156`), `GET /map.png`(`app.py:363-374`), 낡음 안내 |
| B2 | 로봇 기본 토픽 `<ns>/battery_state`·`<ns>/dock_status`·`<ns>/tf`·`<ns>/tf_static`(TurtleBot4 쪽, 우리 코드 아님) | `rescue_robot_watch`(rescue_operations · `robot_watch`, 구독만 `rescue_operations/robot_watch.py:103-106`) → JSON 파일(`robot_watch/1`, 1 s 마다, 로봇별 `last_seen_ns`·`battery_percent`·`docked`·`pose`·`pose_seen_ns`, `rescue_operations/robot_watch.py:108-125`) | `--robot-watch` + `--robot-watch-stale-s` → `load_robot_watch`(`sources.py:57-72`) | `_apply_watch`(`view.py:660-689`): 카드에 직접 관측 줄(`render.py:316`), 프로젝트 상태가 없는 로봇은 낡지 않은 직접 관측 위치를 지도에 씀 |
| B3 | 웹캠 판정 영상 `CompressedImage`(`rescue_perception/webcam_node.py:258`) | `mjpeg_relay`(rescue_perception · `mjpeg_relay`, 구독 `rescue_perception/mjpeg_relay.py:191`) → HTTP `/<camera_id>.mjpg`(`rescue_bringup/operations_launch.py:172`) | `--camera ID=URL` → 웹은 주소만 HTML 에 적음(`render.py:1170-1187`) | **브라우저가 영상을 직접 받는다**(웹 서버는 영상을 중계하지 않음). 새 프레임이 끊기면 JS 가 다시 연결(`render.py:1141-1167`) |
| B4 | 옛 보고 장부(`ReportLedger`, `LEDGER` 인자) | 지금 이 파일을 쓰는 운영 코드는 없다. `rescue_report_node` 는 다른 형식의 별도 저장소(`rescue_reports` 표)를 쓰고 웹은 이 표를 읽지 않는다([README](../../src/rescue_console/README.md) `<운영 장부>` 설명) | `load_ledger`(`sources.py:26-31`) | 없으면 「보고 장부 입력 없음」 경고, 보고 행·건수를 그리지 않음 |
| B5 | 기준 지도 파일 | 없음(저장소의 map_server YAML·PGM) | `--map` → `load_map`(`mapimage.py:156`), 기동 때 한 번 | B1 과 같은 그림. `--live-map` 과 함께 쓸 수 없음(`app.py:146-147`) |
| B6 | 로봇 PC 의 터널 판정 영상 `<ns>/rescue/passage_view/compressed`(`CompressedImage`, `rescue_perception/overhead.py:352`) | `robot_view_relay`(rescue_perception · `robot_view_relay`, 운영 PC, 고른 로봇 하나만 구독하고 나머지는 구독을 푼다 `rescue_perception/robot_view_relay.py:113-123`) → HTTP `/robot_view.mjpg`(포트 = `video_port` + 1, `rescue_bringup/operations_launch.py:203-224`) | `--robot-camera URL` → 주소만 HTML 에 적음(`render.py:1208-1231`). 고를 수 있는 로봇·지금 로봇은 페이지를 그릴 때마다 같은 host:port 의 `GET /select` 로 물음(`commands.py:196-205`) | 로봇 카메라 칸. 브라우저가 영상을 직접 받고, 끊기면 「카메라 영상 낡음」 |
| B7 | operations 의 웹캠 목록 `GET /api/webcam`(꽂힌 `/dev/v4l/by-id` 카메라와 지금 장치, `rescue_operations/command_api.py:185-190`) | db_writer 프로세스 안의 `WebcamSwitch`(`rescue_operations/command_api.py:168-221`) | `--camera` 와 `--operations` 가 있고 ADMIN 일 때 페이지를 그릴 때마다 한 번(`app.py:308-310`, `commands.py:184-193`, 대기 1 s) | 웹캠 칸 아래 카메라 고르기(못 받으면 「카메라 목록을 못 받음」) |
| B8 | 브라우저 | 없음 | 13개 HTTP 경로(`app.py:259-468`) | 5절 |

## 5. 나가는 것

| 받는 쪽 | 무엇을 | 어디서 | 언제 |
| --- | --- | --- | --- |
| 브라우저 | 로그인 화면·관제 화면 HTML(`GET /login`, `GET /`) | `render_login`(`render.py:1280`), `render_page`(`render.py:1258`) | 접속할 때 |
| 브라우저 | 화면 조각 HTML(`GET /fragment`) | `render_fragment`(`render.py:506`) | 1초마다(JS `setInterval(poll, POLL_MS)`, `render.py:1096`, 값은 `POLL_INTERVAL_MS`) |
| 브라우저 | 화면 모델 JSON(`GET /api/view`) | `app.py:357-361` | 요청할 때(시험·디버그) |
| 브라우저 | 지도 PNG(`GET /map.png`) | `app.py:363-374` | 지도가 바뀔 때(`?v=` 해시) |
| 브라우저 | 명령 결과 JSON(`POST /commands`) | `app.py:377-409` | ADMIN 이 확인 칸에서 누를 때 |
| 브라우저 | 웹캠·로봇 카메라 고르기 뒤 302 → `/`(결과는 다음 화면의 알림) | `POST /webcam`(`app.py:338-349`, ADMIN), `POST /robot-camera`(`app.py:325-336`, 누구나) | 고를 때 |
| operations 명령 API | `POST http://127.0.0.1:<api_port>/api/operator-commands` JSON 한 건: `command_id`(UUID), `kind`, `operator_id`(로그인 계정), 명령별 필드 | `build_request`(`commands.py:92-113`) → `send_command`(`commands.py:171-178`) → `post_json`(`commands.py:116-139`) | 명령 한 번에 **정확히 한 번**, 재시도 없음, 5 s 제한(`commands.py:23`) |
| operations 웹캠 API | `GET /api/webcam`(목록), `POST /api/webcam` `{device, operator_id}`(바꾸기) | `webcam_state`(`commands.py:184-193`), `switch_webcam`(`commands.py:222-235`) | 목록은 ADMIN 페이지를 그릴 때마다(1 s), 바꾸기는 ADMIN 이 확인 창에서 고를 때 한 번(5 s, 재시도 없음) |
| `robot_view_relay` | `GET /select`(고를 수 있는 로봇·지금 로봇), `POST /select` `{robot}` | `robot_camera_state`(`commands.py:196-205`), `select_robot_camera`(`commands.py:208-219`) | 목록은 페이지를 그릴 때마다(1 s), 바꾸기는 누가 고를 때 한 번(1 s, 확인 창 없음 — 보기 설정) |
| 계정 DB(웹 자신의 SQLite) | 계정 추가·수정·삭제, 로그인 시각 | `accounts.py` `AccountStore`(`accounts.py:119`) | 로그인·계정 관리 화면 |
| ROS | **없음** | — | — |

명령의 그 뒤(웹 밖): 명령 API(`rescue_operations/command_api.py:241` `do_POST` → `CommandGate.handle` `rescue_operations/command_api.py:108`)가 `operator_commands` 에 적고 ROS 서비스 `/rescue/operator_command`(`SubmitOperatorCommand`)를 부른다(`rescue_operations/ros_node.py:247-266`). 받는 쪽은 `mission_coordinator`(`rescue_coordination/ros_node.py:2485`)다. 조정기가 받아들이면 다음 `/rescue/system_state`·`/rescue/targets` 가 바뀌고(A4·A5), 웹은 다음 폴링에서 그 결과를 본다. 웹은 명령 결과를 화면 상태로 추측하지 않는다.

웹캠 바꾸기의 그 뒤: `POST /api/webcam`(`rescue_operations/command_api.py:241-250`) → `WebcamSwitch.switch`(`rescue_operations/command_api.py:192-218`)가 꽂힌 카메라인지 보고 `webcam_node` 의 파라미터 서비스(`/webcam_node/set_parameters`, `device_path`)를 부른다(`rescue_operations/ros_node.py:271-298`). 결과는 `error_log` 에 `CAMERA_SWITCHED`·`CAMERA_SWITCH_REFUSED` 로 남아 웹의 명령·오류 기록에 보인다. 바꾸면 지진 판정이 처음부터 다시 돌고 출구 감시가 풀린다(그래서 웹이 확인 창을 띄운다). 로봇 카메라 고르기는 운영 명령이 아니라 보기 설정이라 장부에 남지 않는다.

## 6. 웹 파일마다 받는 것 · 하는 것 · 내보내는 것

경로는 `src/rescue_console/rescue_console/` 아래다.

| 파일 | 받는 것 | 하는 것 | 내보내는 것 | 볼 함수(줄) |
| --- | --- | --- | --- | --- |
| `__main__.py` | 명령행 인자, 환경 변수 `RESCUE_CONSOLE_SECRET` | 인자끼리 맞는지 검사(`--live-map`↔`--live-map-stale-s`, `--robot-watch`↔`--robot-watch-stale-s`, 장부 또는 `--site-state`), `create_app` 호출, 서버 기동 | 실패하면 stderr `CONSOLE FAILED: …` + 종료코드 2(서버를 띄우지 않음) | `main`(`__main__.py:17-98`) |
| `app.py` | HTTP 요청(쿠키 세션, 폼), 기동 설정 | 라우트 13개, 로그인·세션 무효화·CSRF·보안 헤더, 폴링마다 `current_view` 로 화면 모델을 새로 만듦, 명령 중계 전 권한 검사, 페이지를 그릴 때 웹캠 목록·로봇 카메라 상태를 물음 | HTTP 응답(HTML·JSON·PNG), 로그 | `current_view`(`app.py:52-84`), `create_app`(`app.py:121-470`), `GET /`(`app.py:303-323`), `POST /commands`(`app.py:377-409`), `POST /webcam`(`app.py:338-349`), `POST /robot-camera`(`app.py:325-336`) |
| `sources.py` | 현장 상태 DB, 픽스처 JSON(`--status`), 옛 장부, robot_watch 파일 | 읽기 전용으로 읽어 딕셔너리로 모음. 하나라도 실패하면 `STORAGE_ERROR`(그 요청 503) | 입력 딕셔너리(쓰기 없음) | `load_site_state`(`sources.py:75-99`), `load_robot_watch`(`sources.py:57-72`) |
| `view.py` | `sources.py` 의 입력, 기동 설정(로봇 이름, `--support-dispatch`, `--search-start`) | 화면 모델 합성: 대상 4단계, 로봇 낡음(5 s), 직접 관측 붙이기, 임무 사유 문구, 명령 버튼 활성 판정, 터널·출구 행, 빠진 입력은 경고 | 화면 모델 딕셔너리(= `/api/view` JSON) | `build_view`(`view.py:692-790`), `_gate`(`view.py:530-595`), `command_gates`(`view.py:598-623`), `_apply_watch`(`view.py:660-689`), `_passages`(`view.py:629-647`) |
| `mapimage.py` | 지도 YAML·PGM, `live_map.alive` | PGM → PNG, 실시간 지도 다시 읽기·낡음 판정, 월드 좌표 → 픽셀, frame 이 다르거나 지도 밖이면 그리지 않고 목록에 올림 | PNG 바이트, 지도 위 마커 목록 | `load_map`(`mapimage.py:156-207`), `LiveMap.current`(`mapimage.py:233-270`), `map_overlay`(`mapimage.py:291-325`) |
| `render.py` | 화면 모델, 웹캠 목록·로봇 카메라 상태 | HTML 조각·전체 페이지, 1초 폴링 JS, 명령 확인 칸 JS, 영상 끊김 재연결 JS, 웹캠 고르기(확인 창)·로봇 카메라 칸 | HTML 문자열(브라우저로) | `render_fragment`(`render.py:506`), `_robots`(`render.py:327-365`), `_map`(`render.py:211-295`), `_commands`(`render.py:514`), 폴링 JS(`render.py:895`, `:1096`), `_webcam_picker`(`render.py:1190-1205`), `_robot_camera`(`render.py:1208-1231`) |
| `commands.py` | `POST /commands`·`/webcam`·`/robot-camera` 폼, 로그인 계정 | 폼 → 요청 JSON(허용 필드만, 숫자 검사), 한 번 보내고 답 모양 검사. 웹캠 목록·로봇 카메라 상태 묻기 | operations·`robot_view_relay` 로 HTTP 요청 한 번씩, 결과 딕셔너리·문구 | `build_request`(`commands.py:92-113`), `post_json`(`commands.py:116-139`), `read_reply`(`commands.py:146-168`), `send_command`(`commands.py:171-178`), `webcam_state`(`commands.py:184-193`), `switch_webcam`(`commands.py:222-235`), `robot_camera_state`(`commands.py:196-205`), `select_robot_camera`(`commands.py:208-219`) |
| `accounts.py` | 계정 DB 경로, 로그인·계정 관리 폼 | PBKDF2 해시(200000회), 역할 ADMIN·CUSTOMER, 마지막 ADMIN 지우기 막기, `init` 하위 명령 | 계정 DB 쓰기 | `AccountStore`(`accounts.py:119`), `authenticate`(`accounts.py:209`), `main`(`accounts.py:353`) |
| `throttle.py` | 로그인 실패(계정명·주소) | 300 s 안에 5번 실패하면 60 s 잠금(메모리) | 잠금 남은 시간 | `LoginThrottle`(`throttle.py:21-73`) |
| `labels.py` | 상태 코드 | 코드 → 한국어 표시 문구 표 | 표시 문구 | `label`(`labels.py:59-67`) |

## 7. 핵심 클래스와 함수

경로는 `src/rescue_console/rescue_console/` 아래다. 클래스는 7개뿐이다. 화면 판단은 전부 `view.py` 의 순수 함수(I/O·시계 없음)라서 ROS·Flask 없이 import 해서 시험할 수 있다([AGENTS.md](../../AGENTS.md) 8절). Flask 앱도 클래스가 아니라 `create_app` 이 만든 객체이고, 라우트는 그 안의 함수다.

### 7.1 클래스

| 클래스 | 위치 | 무엇 |
| --- | --- | --- |
| `ConsoleError` | `view.py:82-91` | 웹의 오류 하나. 코드(`INVALID_INPUT`·`STORAGE_ERROR`·`CONFIG_MISSING` …)와 설명을 담는다. 요청 중에 나면 `failed`(`app.py:243-256`)가 503 으로 바꾸고, 기동 중에 나면 `CONSOLE FAILED` 로 종료한다 |
| `CommandSendError` | `commands.py:47-54` | 명령을 보내다 실패한 것. `delivery` 는 `NOT_SENT`(확실히 안 감) 또는 `UNKNOWN`(갔을 수도 있음) |
| `MapImage` | `mapimage.py:139-146` | 바뀌지 않는 지도 한 장(frozen dataclass): PNG 바이트와 화면 모델용 `meta` |
| `LiveMap` | `mapimage.py:210-270` | SLAM 지도 파일을 바뀔 때만 다시 읽고 `OK`·`STALE`·`ERROR`·`MISSING` 을 판정 |
| `AccountStore` | `accounts.py:119-304` | 계정 DB(웹이 쓰는 유일한 파일)의 읽기·쓰기 |
| `_Transaction` | `accounts.py:307-350` | 계정 DB 쓰기를 `BEGIN IMMEDIATE` … `COMMIT`/`ROLLBACK` 으로 감싸는 `with` 블록 |
| `LoginThrottle` | `throttle.py:21-73` | (계정명, 접속 IP) 쌍마다 로그인 실패를 세어 잠금 |

### 7.2 핵심 함수

| 함수 | 위치 | 받음 → 내보냄 | 규칙과 이유 | 시험 |
| --- | --- | --- | --- | --- |
| `main` | `__main__.py:17-98` | 명령행 인자, 환경 변수 → 서버 기동 / `CONSOLE FAILED` + 종료코드 2 | 세션 비밀값은 환경 변수로만 받는다(프로세스 목록에 보이지 않게). 낡음 기준 시간은 짝으로만 받고 기본값이 없다. 잘못된 설정은 첫 요청이 아니라 기동 때 멈춘다 | — |
| `create_app` | `app.py:121-470` | 기동 설정 → 라우트 13개가 붙은 Flask 앱 | 기동 때 카메라 끊김 시간·비밀값 16자 이상·입력 조합·계정 DB·기준 지도·operations 주소를 확인한다. 현장 상태 DB·장부·픽스처는 열지 않고 요청마다 읽는다. 세션 쿠키 HttpOnly·SameSite=Lax | `test_console_web.py` |
| `current_view` | `app.py:52-84` | 입력 경로, 시계 → 화면 모델 하나 | 캐시 없이 매번 새로 읽는다. 시계는 `clock()` 으로 한 번 읽어 `built_at_ns` 로 넘긴다. 그래야 `build_view` 가 시계 없이 시험된다 | `test_console_view.py` |
| `load_site_state` | `sources.py:75-99` | 현장 상태 DB 경로 → 9개 키 딕셔너리(픽스처와 같은 모양) | 읽기 함수 9개가 각자 `mode=ro` + `query_only` 로 연결하고 role·schema 를 확인한다(`rescue_operations/site_state.py:572-592`). 하나라도 실패하면 `STORAGE_ERROR` → 503, 반쯤 채운 화면은 내지 않는다 | `test_console_view.py`, `test_console_history.py` |
| `build_view` | `view.py:692-790` | 장부, 상태, `built_at_ns`, 기동 설정 → 화면 모델(= `/api/view`) | I/O·시계가 없는 순수 함수라 같은 입력이면 늘 같은 결과다. 빠진 입력은 빈 목록이 아니라 「○○ 입력 없음」 경고로 남겨 「데이터 없음」과 「정상인데 0건」을 구별한다. 형식이 틀리면 `INVALID_INPUT` 으로 화면 전체를 실패시킨다 | `test_console_view.py` |
| `compose_stage`, `STAGE_RULES` | `view.py:99-119`, `:27-33` | 확정 여부·구조·지원·접근 결과 → (단계, 규칙 ID, 경고) | R1 ~ R5 를 위에서부터 처음 맞는 것이 이긴다. `detection_confirmed` 는 정확히 bool 이어야 한다. 모순된 입력은 R1 에 경고를 붙이고 위 단계로 올리지 않는다. 계약 어휘를 늘리지 않으려고 화면 쪽에서 합성한다(scope.md Q-20) | `test_rule_table_rows_are_each_reachable` |
| `_robots` | `view.py:237-282` | 배치 로봇, 로봇 상태 → 로봇 카드 행 | 상태가 없는 배치 로봇도 행을 남긴다. 낡음은 로봇 시각이 아니라 operations 수신 시각으로 잰다(5 s). 로봇이 재부팅해 시계가 되돌아가도 믿을 수 있다 | `test_console_view.py` |
| `_gate`, `command_gates` | `view.py:530-595`, `:598-623` | 임무 상태, 대상, AUTO 설정 → 명령마다 (켜짐, 사유, 보낼 값, 고를 대상) | 검사 순서는 보류 명령 → 임무 상태 없음 → 재난 모드 아님 → 명령별 조건이고, 처음 걸린 것이 사유로 나온다. 조정기의 수락 조건(`rescue_coordination.mission`)을 미리 비추는 안내이지 권한 확인이 아니다 | `test_console_commands.py`(`:237` 부터) |
| `mission_reason` | `view.py:438-463` | `SystemState.reason` → {code, text, level, scope} | 모르는 코드는 그대로 보인다. 로봇 이름을 모르면 이름을 지어내지 않고 역할 코드(SUPPORT/SCOUT)를 쓴다 | `test_console_reasons.py` |
| `_passages`, `_apply_watch` | `view.py:629-647`, `:660-689` | 터널 판정 / 직접 관측 → 지도 행 / 카드 줄 | 터널: 지금 지도와 (세션, revision)이 같으면 진하게, 지금 지도를 모르면 전부 진하게 + 경고. 직접 관측: 프로젝트 상태가 먼저이고, 상태가 없는 로봇만 낡지 않은 직접 관측 위치를 쓴다 | `test_console_tunnel_map.py`, `test_console_robot_watch.py` |
| `LiveMap.current` | `mapimage.py:233-270` | 지도 파일 → (지도, 상태) | (수정 시각, 크기)가 바뀔 때만 다시 읽는다. `ERROR` 를 `STALE` 보다 먼저 보고, 읽기에 실패해도 마지막 정상 지도는 보이되 `ERROR` 라고 알린다. 낡음은 지도 수정 시각과 `live_map.alive` 중 늦은 쪽으로 판정해 안 바뀌는 고정 지도를 낡음으로 보지 않는다 | `test_console_live_map.py` |
| `map_overlay`, `world_to_pixel` | `mapimage.py:291-325`, `:273-285` | 화면 모델 행 → 지도 마커 | frame 이 `map` 이 아니면 `frame_mismatch`, 지도 밖이면 `off_map`, 위치 없는 로봇은 `no_pose` 로 분류하고 가장자리에 붙여 그리지 않는다. 경계 검사 전 1/100 픽셀로 반올림한다(부동소수점 오차) | `test_console_map.py` |
| `command`(`POST /commands`) | `app.py:379-409` | 폼 → 결과 JSON | CSRF → `command_access` → `build_request` → `send_command`. 화면 JS 가 JSON 을 기다리므로 `admin_required` 대신 `login_required` 를 쓰고 권한 실패도 `{sent:false}` 로 돌려준다(CUSTOMER 403, 연결 없음 503). 거절도 통신은 정상이라 200, 답이 없으면 502 | `test_console_commands.py` |
| `build_request` | `commands.py:92-113` | 폼, 로그인 계정, 서버 UUID → 요청 JSON | 명령마다 정해진 필드만 받는다(`COMMAND_FIELDS`). `operator_id` 는 세션에서, `command_id` 는 서버가 넣는다. 숫자는 ASCII 숫자만. 변조된 요청으로 로봇 ID·남의 운영자 ID·임의 값을 끼워 넣지 못한다 | `test_fields_match_the_coordinator_gate` |
| `post_json` | `commands.py:116-139` | URL, JSON → (status, body) / `CommandSendError` | 밖으로 나가는 HTTP 는 모두 이 함수의 `urlopen`(`:123`)을 지난다. 재시도 없음. 연결 거부·DNS 실패만 `NOT_SENT`, 시간 초과 등은 operations 가 받았을 수 있어 `UNKNOWN`. HTTP 오류 응답도 본문(거절 사유)을 읽는다 | `test_slow_operations_times_out_once_and_delivery_is_unknown` |
| `read_reply`, `send_command` | `commands.py:146-168`, `:171-178` | 응답 → 결과 딕셔너리 | `accepted` 는 정확히 bool, 거절이면 `reason` 필수, 2xx 가 아닌데 수락이면 안 된다. 모양이 틀리면 「거절」이 아니라 「실패」다. 화면은 `NOT_SENT` 를 「전달 안 됨」, `UNKNOWN` 을 「전달됐는지 모름 — command_id … 로 명령 기록을 확인」으로 보인다(`render.py:1038-1044`) | `test_a_failure_is_reported_and_never_retried` |
| `render_page`, `render_fragment` | `render.py:1258-1277`, `:506-509` | 화면 모델 → HTML | 페이지는 고정 틀(상단·카메라·명령 패널)과 `div#live` 로 나뉘고 폴링은 `div#live` 만 바꾼다. 영상 `<img>` 는 조각 밖이라 폴링이 영상을 끊지 않는다. JS `poll()`(`:893`)은 401·리다이렉트면 로그인으로 보내고, 실패하면 기존 화면을 두고 「갱신 끊김」을 띄운다 | `test_console_web.py` |
| `label` | `labels.py:59-67` | (종류, 코드) → 표시 문구 | 모르는 코드는 그대로, `None` 은 `None`. 없는 종류는 `KeyError`(프로그램 실수를 숨기지 않음) | `test_console_labels.py` |
| `user`, `login` | `app.py:182-196`, `:267-293` | 세션 / 로그인 폼 → 계정 / 리다이렉트 | 세션에는 `account_id`·`auth_version` 만 두고 요청마다 계정 DB 와 맞춰 본다. 로그인 순서는 CSRF → 잠금 → 비밀번호, 성공하면 세션을 새로 만든다. 실패 문구는 하나다 | `test_console_web.py` |
| `AccountStore`, `hash_password` | `accounts.py:119-304`, `:63-90` | 계정 폼 → 계정 DB | metadata role 이 `console-accounts` 인 파일만 연다. 쓰기는 `BEGIN IMMEDIATE` 안에서 확인 후 쓰기(마지막 ADMIN 보호). 비밀번호·역할·상태가 바뀌면 `auth_version` +1 로 옛 세션을 끊는다. 없는 계정도 더미 해시로 같은 시간을 쓴다. PBKDF2-SHA256 200000회, 16바이트 솔트, `hmac.compare_digest` | `test_console_accounts.py` |
| `LoginThrottle` | `throttle.py:21-73` | 로그인 실패 → 잠금 남은 시간 | (계정명, IP) 쌍마다 300 s 안 5회 실패면 60 s. `time.monotonic` 이라 시스템 시계 변경에 흔들리지 않는다. 메모리에만 두고 키는 최대 10000개 | `test_console_throttle.py` |

### 7.3 주요 상수

숫자는 코드에 흩뿌리지 않고 모듈 맨 위 상수에 둔다([AGENTS.md](../../AGENTS.md) 8절). 대부분 주석에 누가 언제 정했는지 적혀 있다.

| 상수 | 값 | 위치 | 뜻 |
| --- | --- | --- | --- |
| `POLL_INTERVAL_MS` | 1000 | `render.py:14` | 브라우저 폴링 간격. KPI-04 예산 3 s 중 남는 2 s 를 기록·조회에 쓴다 |
| `KPI04_BUDGET_S` | 3.0 | `render.py:16` | 사건 발생 → 화면 표시 예산 |
| `STATUS_STALE_S` | 5.0 | `view.py:75` | 로봇 상태 낡음 기준(수신 시각 기준) |
| `COMMAND_TIMEOUT_S` | 5.0 | `commands.py:23` | 명령 답을 기다리는 시간 |
| `COMMAND_FIELDS` | 명령 5종 × 필드 | `commands.py:27-33` | 보낼 수 있는 명령과 필드. 조정기와 같은지 시험이 확인한다 |
| `HELD_COMMANDS` | PAUSE·RESUME·CANCEL·STOP_ROBOT | `view.py:58-63` | 늘 꺼 두는 명령과 그 이유 |
| `STAGE_RULES` | R1 ~ R5 | `view.py:27-33` | 대상 4단계 규칙표 |
| `EVENT_ROWS` / `HISTORY_ROWS` | 50 / 30 | `view.py:68`, `:70` | 화면에 보이는 사건 / 명령·오류 기록 행 수 |
| `SITE_EVENT_READ_LIMIT` | 1000 | `sources.py:20` | 사건 읽기 상한(11절) |
| `MAP_FRAME` | `'map'` | `mapimage.py:20` | 이 frame 의 좌표만 지도에 그린다 |
| `LOGIN_MAX_FAILURES` · `LOGIN_WINDOW_S` · `LOGIN_LOCK_S` | 5 · 300 · 60 | `throttle.py:14-16` | 로그인 잠금 |
| `_ITERATIONS` | 200000 | `accounts.py:39` | PBKDF2 반복 횟수 |
| `SECURITY_HEADERS` | 4개 | `app.py:30-35` | nosniff, iframe 금지, referrer 없음, CSP 일부 |

## 8. 한 번의 폴링 (1초마다)

1. 브라우저 JS 가 `fetch('fragment')`(`render.py:895`)를 보낸다. 1초 간격은 `setInterval(poll, POLL_MS)`(`render.py:1096`).
2. `GET /fragment`(`app.py:351-355`): 로그인·세션 확인 뒤 `view()`(`app.py:174-179`) → `current_view`(`app.py:52-84`).
3. `current_view`: 현장 상태 DB 읽기 9개(`load_site_state`, `sources.py:75-99`) 또는 픽스처, 옛 장부(있으면), robot_watch 파일(있으면)을 캐시 없이 새로 읽는다. 읽기 함수 9개는 표마다 따로 연결하므로 한 시점의 값이 아니다. 읽는 도중 기록이 들어오면 한 폴링 안에서 표끼리 잠깐 어긋날 수 있고 다음 폴링에서 맞춰진다(`sources.py:76-83` docstring). 명령은 조정기가 받을 때 자기 상태로 다시 검사하므로, 이 어긋남이 잘못된 명령 실행으로 이어지지 않는다.
4. `build_view`(`view.py:692-790`)가 화면 모델을 만든다. 순서: 대상 행·4단계 → 사건 목록 → 임무 → 로봇 카드 → 직접 관측 붙이기(`view.py:753`) → 사유 문구 → 명령 이력 → 출구·터널 → 버튼 활성 판정(반환 값 안의 `command_gates`).
5. 지도가 있으면 `map_overlay`(`mapimage.py:291-325`)가 로봇·대상·출구·터널을 픽셀로 바꾼다. 못 그린 것은 경고(`app.py:75-78`).
6. `render_fragment`(`render.py:506`)가 HTML 로 바꿔 응답한다. JS 가 `div#live` 를 통째로 바꾼다.
7. 실패: 읽기 실패·모양 실패는 그 요청만 503 이고, 다음 폴링에서 다시 읽는다(빈 화면이나 지어낸 값을 보이지 않는다).

## 9. 한 번의 명령

1. ADMIN 이 버튼을 누르면 확인 칸이 열리고, 확인하면 JS 가 `fetch('commands', {method: 'POST'})`(`render.py:1068`)를 보낸다.
2. `POST /commands`(`app.py:377-409`): CSRF → 역할·연결 확인(`command_access`) → `build_request` 로 요청을 만든다. `operator_id` 는 폼이 아니라 로그인 계정이고, `command_id` 는 누를 때마다 새 UUID 다.
3. `send_command`(`commands.py:171-178`)가 operations 로 **한 번** POST 한다. 실패하면 `delivery`(`NOT_SENT`/`UNKNOWN`)를 붙여 돌려주고 다시 보내지 않는다.
4. operations 명령 API → `/rescue/operator_command` → `mission_coordinator`(5절). 답(`accepted`, `reason_code` …)은 같은 길로 돌아와 그대로 브라우저에 보이고, operations 는 그 답을 `operator_commands` 에 적는다.
5. 바뀐 상태는 명령 응답이 아니라 다음 폴링(8절)의 `mission_state`·`target_states`·`operator_commands` 로 보인다.

웹캠·로봇 카메라 고르기는 JS 가 아니라 보통 폼이다: `POST /webcam`(ADMIN, 브라우저 확인 창) 또는 `POST /robot-camera`(누구나, 고르면 바로 제출) → CSRF 확인 → 한 번 보내기 → 결과 문구를 flash 로 남기고 302 `/` → 다시 그린 페이지에 알림. 영상 자체는 다시 그린 페이지의 `<img>` 가 새로 붙는다.

## 10. 설계 판단

| 판단 | 이유 | 근거 |
| --- | --- | --- |
| 화면 조회는 현장 상태 DB 를 직접 읽기 전용으로 읽고, operations 로는 명령만 보낸다 | 현장 상태 DB 에 쓰는 프로세스는 operations 하나로 정했다. 표시만 하려고 HTTP 프로세스를 하나 더 두지 않는다 | `sources.py:75-99`, `rescue_operations/site_state.py:572-592` |
| 웹은 토픽을 구독·발행하지 않는다 | ROS 입출력은 경계 모듈(operations)에 모은다. 웹은 ROS 없이 시험된다 | `src/rescue_console` 에 rclpy 0건, [AGENTS.md](../../AGENTS.md) 8절 |
| 명령은 한 번만 보내고 다시 보내지 않는다 | 조정기가 이미 받았을 수 있어서 다시 보내면 같은 명령이 두 번 실행될 수 있다. 실패하면 `delivery` 로 전달 여부를 보인다 | `commands.py:1-7`, `post_json` |
| 버튼 켜짐/꺼짐은 안내용 판정이다 | 최종 판단은 조정기가 한다. 켜진 버튼도 거절될 수 있고 그 사유를 그대로 보인다 | `view.py:598-601` |
| 정지 버튼은 늘 꺼져 있다 | 현장 정지 수단을 웹 연결에 맡기지 않는다. 위험하면 현장 비상 정지 장치를 쓴다 | `view.py:58-63`, [AGENTS.md](../../AGENTS.md) 7절 |
| 1초 폴링(WebSocket 아님) | 표준 라이브러리와 Flask 만으로 되고, 끊겨도 다음 폴링에서 복구된다. KPI-04 예산 3 s 안에 든다 | `render.py:14` |
| 영상은 웹 서버를 거치지 않는다 | 브라우저가 중계 노드에서 직접 받는다. 영상 `<img>` 는 폴링 조각 밖이라 폴링이 영상을 끊지 않는다 | `render.py:1170-1187`, `:1258-1277` |
| 모르는 값은 지어내지 않는다 | 낡음·위치 미확정·입력 없음을 경고로 보인다. 지도 밖·다른 frame 좌표는 그리지 않고 목록에 올린다 | `build_view`, `map_overlay` |
| 낡음 기준 시간에 기본값이 없다 | 현장마다 값이 다르다. 인자 짝이 안 맞으면 기동하지 않는다 | `__main__.py:76-79`, `render.py:1126-1134` |
| 웹캠 바꾸기는 ADMIN + 확인 창, 로봇 카메라 고르기는 누구나 + 확인 없음 | 웹캠을 바꾸면 지진 판정이 처음부터 다시 돌고 출구 감시가 풀린다. 로봇 카메라는 보기 설정이라 로봇에 아무것도 보내지 않는다 | `app.py:325-349` |
| 자동 출동·자동 수색 시작은 기동 인자(`--support-dispatch`, `--search-start`)로 받는다 | IDL 을 바꾸지 않으려고 조정기가 `SystemState` 에 방식을 싣지 않는다 | [README](../../src/rescue_console/README.md) 「실행 (로봇 없이, E2)」 |
| 직접 관측(`--robot-watch`)을 둔다 | 로봇 PC 의 프로젝트 노드가 아직 없어도 로봇이 보이게 한다. 프로젝트 상태가 있으면 그쪽이 먼저다 | `view.py:660-689` |

## 11. 알려진 한계

코드는 아직 고치지 않았다. [README](../../src/rescue_console/README.md) 「하지 않는 것 / 후속」과 [설계 명세](console-spec.md) 9절에도 일부가 있다.

| 한계 | 결과 | 위치 | 근거 |
| --- | --- | --- | --- |
| `--robot-camera` 만 주고 `--camera-stale-s` 를 안 줌 | 기동은 되는데 `GET /` 가 매번 503 `CONFIG_MISSING` | `app.py:135`, `render.py:1264` | [측정] |
| `--robot-watch-stale-s` 값 검사가 없음 | `nan` 이면 낡음이 영영 안 뜨고 위치도 안 쓴다. 0·음수면 늘 낡음 | `__main__.py:54-55`, `view.py:672` | 코드 읽기 |
| 계정 DB 의 sqlite 오류 처리 | `exc_type is sqlite3.Error`(정확한 타입 비교)라 하위 오류가 503 이 아니라 500 이 된다 | `accounts.py:348` | 코드 읽기 |
| 1초 폴링이 앞 요청을 기다리지 않음 | 서버가 느리면 늦게 온 옛 화면이 새 화면을 덮을 수 있다(다음 폴링에서 바로잡힘) | `render.py:1096` | 코드 읽기 |
| CSRF 비교의 기본값 `'-'` | 세션에 토큰이 없을 때 폼 `csrf=-` 가 통과한다. 로그인 전 세션만 해당하므로 `/login`·`/logout` 에 영향이 있고, 명령·계정 관리는 로그인 때 토큰이 생겨 해당 없다 | `app.py:208`, `:391` | 코드 읽기 |
| 옛 보고 장부를 쓰는 운영 코드가 없음 | 현장에서 보고 칸이 늘 「보고 장부 입력 없음」 | `sources.py:26-31` | README |
| 현장 사건이 1000건을 넘음 | 오래된 순으로 1000건만 읽어 최신 사건이 빠진다 | `sources.py:16-20` | 코드 읽기 |
| 기동 로그에 git SHA 가 없음 | 팀 규칙(노드는 기동 때 git SHA 를 남김)과 다르다. 웹은 ROS 노드가 아니다 | `__main__.py` | 코드 검색 |

## 12. 이 문서가 확인하지 않은 것

- 토픽 이름은 launch 코드의 상수와 예시 설정에서 읽었다. 현장에서 `ros2 topic list` 로 본 값이 아니다(NOT_RUN).
- QoS 는 적지 않았다. 각 구독 줄(`rescue_operations/ros_node.py:214-246`)과 [계약](contracts.md)이 기준이다.
- 직접 관측(B2)과 터널 판정(A7·A8)의 처리 규칙·문구는 [관제 웹 설계 명세](console-spec.md) 7.4.7·7.4.8·7.9·7.10·I-2·I-5, 웹캠 고르기(B7)와 로봇 카메라 칸(B6)은 7.12·I-6·I-7 에 있다.
- 알려진 한계는 11절에 모았다.
- 흐름 그림 세 장(2절)은 mermaid 11.4.1 로 그려지는 것을 확인했다 [측정, 2026-10-14].
- 웹 시험 `tests/test_console_*.py` 14 파일 235 OK(skip 0) [측정, 2026-10-13, main `4163a25` 합친 뒤, Python 3.13, Flask 3.1.3]. 이 문서의 사슬을 끝에서 끝까지 잇는 시험(ROS 발행 → 장부 → 화면)은 없다(NOT_RUN).

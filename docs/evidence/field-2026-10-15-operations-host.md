# 현장 운영 PC 기록 (2026-10-15 실기)

- 대상 PC: 운영 PC(프로파일 호스트 `operations` + `coordination`). 운영 launch(DB 기록·보고·웹캠), 조정기, 관제 웹, 실시간 지도 중계, 직접 관측.
- 함께 돈 PC: 로봇 담당 PC 한 대(TB-6·TB-5 둘 다, [현장 실행 안내](../process/field-runbook.md) 「한눈에」), 전용 Discovery PC 한 대([Discovery 전용 PC 운영 안내](../process/discovery-host.md)).
- 대상 코드: main `5669356`(14:0x) → `6d43fce`(15:5x 부터 끝까지). 현장 파일 한 줄은 로컬 수정(5절, 뒤에 #334 로 main 반영).
- 운영 PC 는 로봇에 이동·언독 명령을 보내지 않았다. 로봇 쪽 명령은 모두 로봇 담당 PC 에서 사람이 보냈다. 운영 PC 가 부른 서비스는 조정기 자기 서비스 `refresh_state` 뿐이다.
- 실습은 17:54 재난 선포 뒤 중지됐다(7절). 주소는 `<DISCOVERY_IP>` 로 적는다. 시각은 KST.

## 결과 요약

| # | 항목 | 결과 |
| --- | --- | --- |
| 1 | 운영 launch·조정기·관제 웹 기동(`6d43fce`) | PASS [측정] 성공 로그 모두 나옴 |
| 2 | 준비 점검 `preflight_role.py --role coordination` / `--role operations`(웹캠 YAML 포함) | PASS [측정] 둘 다 exit 0. 프로파일을 고친 뒤(2절) |
| 3 | Discovery 서버로 로봇 토픽 조회 | 기본 대기로는 0개, `--spin-time 10` 으로 `/robot6`·`/robot5` 각 43개 [측정] |
| 4 | 망이 바뀐 뒤 이미 떠 있던 노드 | **FAIL** 두 번. 다시 띄운 뒤 PASS [측정] |
| 5 | 늦게 뜬 실행기의 조정기 세션 | 매번 `CLOCK_UNCERTAIN` 거절 → `refresh_state` 로 풀림 [측정] |
| 6 | `task_arm` | **FAIL** `STOP_OR_READINESS_NOT_READY` — 카메라 확인 토픽 무응답 [측정] |
| 7 | 재난 선포 → TB-6 탐색 | **FAIL** 첫 목표 실패 2초 뒤 `NO_SURVIVOR_FOUND` [측정] |
| 8 | 지도 고정 전 TB-6 위치를 관제 지도에 표시 | PASS [측정] 직접 관측(`robot_watch`)으로만 |
| 9 | 지도 고정 전 TB-5 위치 | NOT_RUN — 설계상 1차 고정 전에는 없음 |

## 1. 시간 순서

| 시각 | 일 |
| --- | --- |
| 15:58~15:59 | 운영 PC 와 Discovery PC 가 거의 같은 때 재부팅됨(원인 미확인, 운영 PC 는 `journalctl` 의 새 Boot 줄). 운영 PC 의 서버가 모두 꺼짐 |
| 16:01~16:04 | 운영 PC 서버를 `6d43fce` 로 다시 띄움. 이때 Discovery 서버는 꺼져 있었다 |
| 16:27:45 | Discovery 서버 다시 기동(Discovery PC 담당 보고). 운영 PC 노드는 따로 다시 띄우지 않아도 서버에 다시 붙었다 [측정] |
| 16:33 | 관제 웹 지도에 다른 지도(118×77, 해시 `1c3ee1ec616c`)가 들어옴 |
| 16:41 | 회차를 새로 시작: 새 회차 폴더로 운영 launch·조정기·관제 웹을 다시 띄움 |
| 16:42 | 운영 PC 를 Wi-Fi 에서 유선으로 바꿈 → 4절 |
| 16:50 | 16:33 지도의 발행자가 다른 PC 의 `/robot6/map_server` 임을 찾음 → 3절 |
| 16:54 | 로봇 담당 PC 에서 TB-6 SLAM 기동 |
| 16:58 | 관제 웹 지도가 TB-6 SLAM 지도(55×70, `c558e2de62cf`)로 바뀜 |
| 17:01:13 | `START_POSE bound (-0.116, -0.057, 2.950)`, `MAP_STATE robot_a ... VALID` |
| 17:04~17:08 | 실행기 세션 거절 → `refresh_state` 세 번 → 5절 |
| 17:08 | 현장 파일에 TB-5 출발값을 넣고 조정기를 다시 띄움 → 5절 |
| 17:16~17:25 | `task_arm` 거절, 카메라 확인 토픽 바꿈 → 6절 |
| 17:26~17:30 | 운영 PC 의 Wi-Fi 가 끊겨 유선만 남음 → 노드를 다시 띄움 → 4절 |
| 17:28 | Discovery 서버가 넘쳐 패킷을 버림(초당 약 2,300개, Discovery PC 담당 보고) → 8절 |
| 17:54:07 | 관제 웹 「재난 선포」 → 17:54:12 `NO_SURVIVOR_FOUND` → 7절 |

## 2. 현장 파일을 실기 값으로 바꿈

운영 PC 의 배포 프로파일과 로봇 파일은 저장소의 예시(`config/deployment.v2.example.json`, `config/robots/robot_[ab].v2.json`)를 복사한 것이었다. 그대로는 로봇 담당 PC 의 준비 점검이 FAILED 였다. 바꾸기 전마다 같은 폴더에 `.bak-<시각>` 으로 복사했다. 같은 파일을 로봇 담당 PC 에도 두고 sha256 이 같은지 맞췄다.

| 파일 | 바꾼 것 | 바꾼 뒤 sha256 앞 12자리 |
| --- | --- | --- |
| 프로파일 | `hosts` 네 개의 `network` 만 `{"domain_id": 6, "discovery_servers": [{"server_id": 0, "address": "<DISCOVERY_IP>", "port": 11811}], "super_client": false}` 로 | `b596a79d3327` |
| 프로파일 | `robots[].namespace` `/robot_a` → `/robot6`, `/robot_b` → `/robot5` | `8a8efff9713e` |
| `robots/robot_a.v2.json` | 네임스페이스가 들어간 줄 모두 `/robot_a` → `/robot6` | `b1224b6c4176` |
| `robots/robot_b.v2.json` | 네임스페이스가 들어간 줄 모두 `/robot_b` → `/robot5` | `a025a222c757` |
| `robots/robot_a.v2.json` | `readiness.camera_topics` 를 `/robot6/oakd/rgb/image_raw/compressed` 로(6절) | `7bafe6dad992` |
| `robots/robot_b.v2.json` | 같은 바꿈 `/robot5/...` | `aa03b2c63e6f` |

- 프로파일의 다른 값은 예시 그대로다: `deployment_id=review_example_v2`, `map_basis.map_session_id=22222222-2222-4222-8222-222222222222`, `schema_version=2`, `home_positions` 없음. 첫 회차는 `home_positions` 없이 가기로 했다(TB-6 자동 복귀 없음, 재범 결정 2026-10-15). 실기용 프로파일을 따로 만드는 일은 남아 있다.

## 3. Discovery 서버로 조회할 때

- `ros2 topic list --no-daemon` 은 기본 대기가 짧아 Discovery 서버를 거친 목록이 오기 전에 끝난다. 로봇 담당 PC 와 운영 PC 모두 로봇 토픽이 0개로 보였다. `--spin-time 10` 을 붙이자 `/robot6`·`/robot5` 각 43개가 보였다. `timeout 30` 을 앞에 붙여도 대기는 늘지 않는다 [측정].
- `ROS_SUPER_CLIENT=False` 인 셸에서는 목록이 0개여도 `ros2 topic echo --once <토픽> <타입>` 으로 데이터는 받았다(`/robot5/battery_state` 16.27 V) [측정]. 목록을 보려면 super client 로 조회한다.
- 데몬을 쓰는 `ros2` 명령(`--no-daemon` 없음)은 그 셸 환경으로 데몬을 띄운다. super client 셸에서 띄운 데몬이 운영 PC 에 남아 있어 17:31 에 껐다.
- **다른 PC 의 지도.** 16:33 관제 웹 지도는 우리 SLAM 이 아니었다. 엔드포인트 GID 의 호스트 바이트로 노드를 묶어 보니, `/robot6/map` 발행자는 우리 PC 가 아닌 호스트의 `/robot6/map_server` 였다. 같은 호스트에 `/robot6/amcl`, `/robot5/map_server`, `/robot5/amcl` 이 함께 떠 있었다(저장 지도로 두 로봇 위치를 잡는 `localization.launch.py`) [측정]. 그 PC 에서 끈 뒤 우리 SLAM 이 떴다.

## 4. 망이 바뀐 뒤 노드가 받지 못함

- 16:42 운영 PC 를 Wi-Fi 에서 유선으로 바꿨다(두 주소가 같은 대역). 그 전에 띄운 `map_snapshot` 과 조정기는 `/robot6/map` 구독은 맺었지만 지도를 받지 못했다. 같은 때 새로 띄운 `ros2 topic echo` 는 바로 받았다(118×77) [측정]. 운영 launch, 조정기, `map_snapshot` 을 다시 띄운 뒤 받았다.
- 17:26 운영 PC 의 Wi-Fi 가 끊겨 유선만 남았다. 이미 떠 있던 노드는 실행기 상태를 받지 못했고, `refresh_state` 호출은 30초 동안 답이 없었다. 다시 띄운 뒤 17:30 에 답했다(12초 대기 뒤) [측정].
- 원인은 확인하지 못했다. 이미 만든 참가자가 옛 주소를 계속 알리는 것으로 본다 [추정].

## 5. 늦게 뜬 실행기와 조정기 세션

- 실행기를 조정기보다 늦게 띄우면 실행기 로그에 `coordinator session rejected: CLOCK_UNCERTAIN: coordinator SystemState is future or stale` 또는 `... waiting for measured coordinator clock` 이 나오고, 이어서 `input session blocked: NOT_AUTHORIZED: waiting for fresh configured coordinator SystemState` 가 나왔다(로봇 담당 PC 담당 보고).
- 조정기는 SystemState 를 상태가 바뀔 때만 보낸다(`StatePublication`, `live_task_inputs.py`). 늦게 뜬 실행기는 앞서 나간 오래된 상태만 받는다. 조정기 서비스 `/mission_coordinator/refresh_state` 가 같은 상태를 새 시각으로 다시 보낸다.
- 운영 PC 에서 부른 시각: 15:32, 17:04:14, 17:05:52, 17:08:53, 17:30:00, 17:57:59. 모두 `success=True, message='SYSTEM_STATE_REPUBLISHED; original changed_at_ns preserved'` [측정]. 17:04:14 호출 뒤 운영 DB 의 TB-6 상태에 위치가 채워졌다(17:04:48) [측정].
- 로봇 상태(`robot_status`)는 세션이 거절돼도 DB 에 들어온다. DB 에 행이 있다고 세션이 맺어진 것은 아니다. 실행기 로그로 확인한다.
- **TB-5 출발값.** 재범 실측(TB-6 기준 오른쪽 1.86 m, 같은 방향)으로 현장 파일 `robots.robot_b` 에 `"initial_pose_xy_yaw": [0.0, -1.86, 0.0]` 을 넣었다(재범 승인 17:08). sha256 `b076321e…` → `61be502b…` 로 두 PC 가 같았고, #334 로 main 에 들어갔다. 조정기는 현장 파일을 기동할 때 읽으므로 다시 띄웠다(이 회차에 나간 임무가 없어 실행기는 그대로).

## 6. `task_arm` 거절 — 카메라 확인 토픽

17:16 두 로봇 모두 `task_arm` 답이 `success=False, message='STOP_OR_READINESS_NOT_READY'` 였다. 준비 상태 입력을 운영 PC 에서 재 보았다(TB-6, super client) [측정].

| 입력 | 토픽 | 최대 나이 | 실제 |
| --- | --- | --- | --- |
| driver | `/robot6/odom` | 0.5 s | 20 Hz |
| driver | `/robot6/wheel_status` | 0.5 s | 32 Hz |
| sensors | `/robot6/scan` | 0.5 s | 9 Hz |
| slam | `map -> odom` TF | 1.0 s | 직접 관측 위치 0.5 s 전 |
| sensors | `/robot6/oakd/rgb/preview/image_raw/compressed` | 1.0 s | **12 초 동안 0장**(발행자 1, `camera_info` 도 없음) |

- TB-5 의 같은 미리보기 토픽도 0장이었다. 두 Pi 는 미리보기를 끈 설정이다(재범 2026-10-15 17:24). 로봇 파일의 카메라 토픽을 `<ns>/oakd/rgb/image_raw/compressed` 로 바꾸고(2절) 실행기를 다시 띄웠다.
- 저장소 예시 `config/robots/robot_[ab].v2.json` 과 현장 파일 `passage.view_image_topic`(터널 판정 카메라 화면)도 미리보기 토픽이다. 바꾸지 않았다. 담당(통합-로봇, 통합-인식)이 정한다.

## 7. 재난 선포 → 탐색이 바로 끝남

조정기 로그(운영 PC 시각) [측정]:

```text
17:54:07 COMMAND ... DECLARE_DISASTER -> accepted=True
17:54:07 SEARCH_AUTO_STARTED on DISASTER
17:54:07 FRONTIER e291042b... cells=14 centroid=(-1.80, 1.69) path_m=2.79 goal=(-1.17, 0.94, 2.36)
17:54:09 SEND SEARCH_AREA task=b9c1a355... robot=robot_a generation=0
17:54:09 ADMISSION ... -> True; GOAL ... accepted=True
17:54:10 RESULT task=b9c1a355... outcome=FAILED
17:54:10 FRONTIER_DONE e291042b... outcome=FAILED
17:54:10 FRONTIER none on map update 1485 (1/3)   (1486 2/3, 1487 3/3)
17:54:12 EXPLORATION_COMPLETE reason=EXPLORATION_COMPLETE frontiers_sent=1
17:54:12 SYSTEM_STATE mode=DISASTER phase=SEARCHING reason=NO_SURVIVOR_FOUND
```

- 실패 원인은 TB-6 Nav2 `bt_navigator` 가 inactive 라 목표를 거절한 것이다(`Action server is inactive. Rejecting the goal`, 로봇 담당 PC 담당 보고). 17:30 무렵 Wi-Fi 장애 때 Nav2 기동이 덜 됐다고 본다(같은 보고) [추정].
- 지도가 작아(17:26 에 58×72) 탐색 경계가 그 하나뿐이었다. 그 경계가 실패하자 지도 갱신 세 번 동안 새 경계가 없어 조정기가 탐색을 끝냈다. 이 상태의 조정기는 새 목표를 내지 않는다. 다시 하려면 조정기와 두 실행기를 함께 다시 띄워야 한다(이미 generation 0 이 나갔다).
- 재난 선포 전부터 조정기 상태 사유가 `START_POSE_CONFLICT` 였다. 조정기가 묶은 출발 자세와 다른 값이 `/rescue/map/start_pose/robot_a` 로 들어왔다는 뜻이다(`ros_node.py` `on_start_pose`). 원인은 확인하지 못했다.
- 「경계 하나가 Nav2 사정으로 실패해도 탐색 끝」은 조정기 동작 문제로 통합-조정에 넘긴다. 현장 안내에는 「재난 시작 전 `bt_navigator` active 확인」을 넣었다.

## 8. 그 밖

- **지도 고정 전 위치.** 실행기가 보내는 로봇 위치는 지도 고정과 AMCL 이 맞은 뒤에만 채워진다(`live_pose.py`). TB-6 은 직접 관측(`robot_watch`, 구독만)으로 관제 지도에 그렸다. TB-5 는 1차 고정 전에는 위치가 없다. 재범은 처음부터 TB-5 가 보이길 원해 「출발 자세 + 출발값」 한 점을 「추정」으로 그리는 일을 웹개발 담당에게 맡겼다(작전 2026-10-15 17:18).
- **실시간 지도·직접 관측을 따로 띄움.** 이날은 `map_snapshot`·`robot_watch` 를 운영 launch 밖에서 따로 띄웠다. 운영 launch 의 `map_topic`·`map_dir`·`robot_namespaces`·`robot_watch_path` 를 쓰면 한 번에 뜨고 리플레이에도 지도가 남는다. 현장 안내 3단계에 넣었다. 16:45 에 `map_snapshot` 하나를 ROS 환경 없이 띄워 바로 죽은 일이 있었다(운영 PC 담당 실수, 16:58 에 다시 띄움).
- **Discovery 서버 넘침(17:28).** Discovery PC 담당이 받은 오류의 대부분이 GUID 접두 `01.0f.ba.b9...` 참가자에서 나왔다. 3절과 같은 방법으로 묶었을 때 호스트 바이트 `ba.b9` 에는 TB-5 Pi 노드만 있었다 [추정]. 운영 PC 의 접두는 `01.0f.20.13...` 이다.
- **운영 PC 시계.** timesyncd 가 인터넷 NTP 서버를 쓴다. 한 번은 지연 7.5 s·오프셋 -3.6 s 가 나왔고, 조정기 로그에 `central live clock: no valid synchronized NTP reply` 와 `SYNCED` 가 번갈아 나왔다. 16:3x 에 직접 잰 차이는 -11 ms 였다 [측정]. 로봇망 안에서는 NTP 로 답하는 곳을 찾지 못했다(Discovery PC, 로봇망에서 본 다른 PC 주소 셋, 두 Pi 모두 UDP 123 무응답) [측정].
- **TB-6 라이다 멈춤(두 번).** 15:1x 와 16:50 에 TB-6 의 `/robot6/scan` 이 나오지 않았다. 라이다 노드는 떠 있었다(로봇 담당 PC 담당 보고). 첫 번째는 Pi 를 다시 켰고, 두 번째는 TB-6 Pi 에 ssh 로 들어가 `sudo systemctl restart turtlebot4` 로 살렸다. 이 명령은 PC 에서 치면 `Unit turtlebot4.service not found` 다. 멈춘 원인은 확인하지 못했다.
- **같은 도메인의 다른 팀.** 3절의 `/robot6/map_server` 는 같은 도메인 6, 같은 Discovery 서버를 쓰는 다른 PC 였다. 같은 도메인을 쓰는 한 다시 생길 수 있다. 도메인을 따로 받을지는 정하지 않았다.
- **운영 PC Wi-Fi 절전.** NetworkManager 전역 설정이 `wifi.powersave = 3`(켬)이다. 끄지 않았다(재범 결정 사항). 16:42 부터 유선을 썼다.
- **계정 DB.** `6d43fce` 관제 웹을 처음 띄울 때 계정 DB 가 1.0 → 1.1 로 올라갔다. 먼저 `sqlite3 .backup` 으로 사본을 떴다(사본 schema_version 1.0 확인).

## 남은 일

| 일 | 담당 |
| --- | --- |
| 실기용 배포 프로파일(실제 `map_session_id`, `home_positions`)을 만들고 두 PC 에 같은 sha256 으로 둠 | 작전·재범 |
| 예시 로봇 파일과 `passage.view_image_topic` 의 미리보기 토픽 | 통합-로봇, 통합-인식 |
| 탐색 경계 하나 실패로 탐색이 끝나는 조정기 동작 | 끝남: #338 |
| `START_POSE_CONFLICT` 원인 | 끝남: 설계대로, #339 현장 안내에 반영 |
| 지도 고정 전 TB-5 「추정」 위치 표시 | 끝남: #335 |
| 관제 웹 버전을 올리기 전 계정 DB 백업 안내(1.0 → 1.1 은 되돌릴 수 없음) | 웹개발 |
| 망이 바뀐 뒤 노드가 받지 못하는 원인 | 미정 |
| 같은 도메인 6 을 쓰는 다른 팀과 겹침(도메인 분리 여부) | 작전 |
| Discovery 서버 넘침(GUID `01.0f.ba.b9...`)의 원인 | Discovery 담당 |
| 운영 PC 와 Discovery PC 가 함께 재부팅된 원인 | 작전 |
| 로봇망 안 시간 서버를 둘지 | 작전·재범 |
| 운영 PC Wi-Fi 절전 | 재범 |
| TB-6 라이다가 멈추는 원인 | 통합-로봇 |

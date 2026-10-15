# 가제보 실습 기록 (2026-10-15 저녁, 시뮬레이션 PC 한 대)

## 한눈에

실기 한 회차(런북 3~7단계)를 **시뮬레이션 PC 한 대**에서 Gazebo 로 돌린 첫날 기록이다. 재범 결정(2026-10-15 저녁): 웹 / 웹 없음 / 시뮬레이션 세 트랙 중 시뮬레이션은 운영 PC·Discovery PC·로봇 담당 PC·로봇 Pi 역할을 PC 한 대가 모두 맡는다.

- 결과: 마지막 회차에서 런북 성공 로그(`START_POSE captured`, `SYNCED`, `ARMED`)를 모두 받고 재난 선포 → 자동 탐색 → 탐색 시간 초과 → 지도 고정 R1·R2 → TB-5 출발 정렬(`START_ALIGN_ALIGNED`)까지 갔다. 인형은 발견하지 못했고(`NO_SURVIVOR_FOUND`), TB-5 출동·출구 도착은 NOT_RUN 이다.
- 우리 노드의 코드는 저장소 그대로다. 시뮬레이션 회차의 `src/` 는 main `97f2cef` 와 diff 0줄이다. 시뮬레이션 전용 코드는 `scripts/sim/` 에만 있다.
- 로봇은 지도를 모르고 시작했다. 실제 지도·Gazebo 정답 위치는 어떤 노드에도 주지 않았다. 정답 위치는 원인 확인에만 읽었다(표에 「확인용」).
- 막힌 곳 17가지. 「현장에서도 나올 수 있음」 6가지와 「시뮬레이션만」 11가지로 나눴다(3절).

## 1. 구성

| 실기 역할 | 시뮬레이션 PC 에서 | 코드 |
| --- | --- | --- |
| 로봇 Pi 두 대 | Gazebo arena world(평면도, #364) + TB-6·TB-5, 로봇 쪽 ROS 도메인, `GZ_PARTITION` 따로 | `scripts/sim/sim_robots.sh`, `spawn_extra.launch.py`, `strip_sensors.py` |
| 로봇망 Wi-Fi | 로봇 쪽 도메인 ↔ 현장 도메인 중계: 벽시계 재찍기, 프레임 이름(`odom`·`base_link`·`rplidar_link`), 카메라·깊이 토픽 | `scripts/sim/wall_clock_relay.py`(#362) |
| Discovery 전용 PC | 없음. 한 PC 안이라 LOCALHOST 탐색 + Fast DDS 프로필 | `scripts/sim/fastdds_localhost.xml`(#364) |
| 운영 PC | `operations.launch.py` + 관제 웹 + `coordinator.launch.py` | 저장소 그대로 |
| 로봇 담당 PC | `tb6_slam`, `robot.launch` 두 벌, `map_freeze`, `frozen_localization`(TB-5), `passage_points`·`overhead_passage` 두 벌 | 저장소 그대로. 두 실행기에만 한 PC 리허설용 `scripts/field/wiring-rehearsal/host_shim` |
| 인형 인식(TB-6 카메라 YOLO) | `target_locator`(실물, amr_target 없이) + 인형 관측 대역 | `scripts/sim/fake_target.py`(정답 위치 없이 도면 인형 자리와 로봇 자신의 TF·SLAM 지도로 보임 판정) |

- 한 번에 띄우기: `scripts/sim/run_full.sh`(런북 순서), 끄기 `scripts/sim/stop_full.sh <회차 폴더>`. 두 로봇 생성·실행 스크립트·인형 대역은 다음 PR 에 올린다(이 기록 시점에는 시뮬레이션 작업 브랜치에만 있음). 관제 웹은 Tailscale 주소로 바인드해 재범이 휴대폰·다른 PC 에서 봤다(주소는 실행 인자, 저장소에 없음).
- 시뮬레이션 사본: 현장 파일을 고치지 않고 회차 폴더에 사본을 만들어 썼다. 다른 값은 기동 로그 첫 줄에 「현장 값과 다름」으로 남긴다.

## 2. 마지막 회차 표 (full-1015-223913)

| 시각 | 단계 | 로그 한 줄 | 결과 |
| --- | --- | --- | --- |
| 22:39:13 | 회차 시작 | `run_full: 회차 폴더 …/full-1015-223913` | |
| 22:39:26 | 로봇 쪽 | `sim_robots: robot5 생성 확인` → `떴습니다` | TB-6·TB-5 도크 위 |
| 22:39:29 | 중계 | `RELAY robot5: sim domain … -> field domain …` | |
| 22:39:51 / 22:40:07 | 언도킹(런북 4·5단계, 로봇 감독 몫) | `언도킹 robot6` / `언도킹 robot5` | SUCCEEDED |
| 22:40:11 | 출발 자세 | `언도킹 뒤 arena 자세(출발 자리 ⊕ odom): TB-6 0.88 0.723 1.7662, TB-5 2.7 0.723 1.7632` | 언도킹이 약 191° 돌아 북쪽에서 11° 기욺 |
| 22:40:11 | 사본 값 | `robots.robot_b.initial_pose_xy_yaw 현장 [0.0, -1.86, 0.0], 시뮬레이션 사본 [-0.353, -1.785, -0.003]` | 부호가 현장 값과 같음 |
| 22:40:11 | 사본 값 | `현장 값과 다름: passage.front_m [0.4, 1.0] -> [0.0, 0.05] (SIM_PASSAGE=off)` | 3절 C1 때문에 이 회차만 |
| 22:40:13~14 | 운영·관제 웹 | `rescue_db_writer git=…`, `Running on http://…:8080` | |
| 22:40:35 | TB-6 SLAM | `SLAM_ACTIVE /robot6/slam_toolbox` | |
| 22:40:39 | 로봇 launch 두 벌 | `installed task runtime; input_mode=LIVE_ROS; explicit arm required` | |
| 22:40:45 | 지도 고정 노드 | `START_POSE captured robot_a` | 언도킹 뒤 자세 |
| 22:40:49 | TB-5 위치 추정 | `START_ALIGN_WAITING START_ALIGN_WAIT_MAP` | 고정 전 정상 |
| 22:40:53 | 조정기 | `central live clock: SYNCED` | |
| 22:40:55~22:41:00 | 인형 대역 | `MapState VALID`, `FAKE_TARGET start_pose map=(…) doll map=(…)` | |
| 22:41:08 | TB-6 arm | `ARMED` | |
| 22:55:38 | 재난 선포(재범, 휴대폰 웹) | `DISASTER_DECLARED by operator admin` → `SEARCH_AUTO_STARTED on DISASTER` | |
| 22:55:38 | 첫 탐색 경계 | `FRONTIER e2910… cells=12 centroid=(-1.30, -0.65) goal=(-1.00, -0.74, 3.14)` | 목표가 서쪽 벽 0.1~0.2 m 안쪽(C3). 처음에 「남쪽 벽」으로 적은 것은 지도 원점을 잘못 잡은 계산이었다(2026-10-16 정정, [둘째 회차 자료](sim-2026-10-15-run232037/README.md) 「지도 좌표와 실습장 좌표」) |
| 22:55:40~22:56:40 | TB-6 이동 | `SEND 473f572f GoalPose(x=-1.003, y=-0.738)` → `CommandLease STOPPING` | Nav2 `detected collision ahead!` 64, `Controller patience exceeded`·`spin failed` 14 |
| 22:56:40 | 결과 | `RESULT task=21965219… outcome=FAILED` | 다시 arm 하지 않아 다음 경계 `a116…`(goal (-1.45, 0.57))는 안 나감(C4) |
| 23:05:38 | 탐색 끝 | `EXPLORATION_COMPLETE reason=EXPLORATION_TIMEOUT frontiers_sent=2` | 600 s |
| 23:08:50 | 지도 고정 | `MAP_FREEZE 1 requested` → `MAP_FROZEN 1` → `MAP_FREEZE 2 requested: exploration complete` → `MAP_FROZEN 2` | R1·R2 가 같은 sha `6c251e…`, 62×80 |
| 23:08:50 | 다음 작업 없음 | `task not issued: NO_HOME_POSITION`, `TRAFFIC_HOLD SCOUT TRAFFIC_SCOUT_POSE_UNKNOWN` | C5·C6 |
| 23:08:50~23:09:00 | TB-5 | `MAP_RELAYED /rescue/map/frozen R1 -> /robot5/map`(두 번, 둘째도 R1) → `START_ALIGN_ALIGNED first_guess=[-0.353, 1.820, -2.949] jump_xy 0.24 m` | R1 에서 위치 맞춤 |
| 23:09 이후 | TB-5 | robot_b `MAP_STATE rev=2 quality=UNKNOWN`, `START_ALIGN robot_b ALIGNING: localization not VALID`, `/robot5/amcl_pose` 마지막 stamp 23:09:00 | 관제 웹 「TB-5 위치 미확정」(C2) |
| 23:3x | 관제 웹만 main `6c7b23d`(#366 포함)로 다시 띄움 | 모듈 경로 `install/rescue_console/…` | 로봇·조정기는 그대로 |

- first_guess 의 yaw −2.95 는 정상이다. SLAM 지도 frame 은 생성(도킹) 자리의 odom 이라 언도킹 뒤 TB-6 자세가 지도에서 약 −2.95 rad 다.
- 인형 대역은 한 번도 내지 않았다. TB-6 이 인형 2 m 안에 가지 못했다(`OUT_OF_RANGE`·`OUT_OF_VIEW`·`NOT_IN_FRONT`).

## 3. 막힌 곳

### 3.1 현장에서도 나올 수 있음 (우리 코드·현장 값, 시뮬레이션에서는 고치지 않음)

| # | 무엇 | 근거 | 넘긴 곳 · 상태 |
| --- | --- | --- | --- |
| C1 | 깊이 장애물 정지(PassageStop, 높이 0.05~0.46 m·앞 0.4~1.0 m, stop)가 TB-5 도크·벽 앞에서 TB-6 을 회전까지 세워 작업 120 s 만료 | [재현 기록](sim-2026-10-15-passage-deadlock.md)(#365) | 통합-로봇 #363(분석·실패 시험, 추천 stop → limit). 값 변경은 재범 승인 대기. 시뮬레이션만 `SIM_PASSAGE=off` 선택 인자 |
| C2 | R2 뒤 TB-5 위치 추정이 끊김: relay 줄이 R1 로 두 번 찍힘, `amcl_pose` 멈춤, `localization not VALID` | 2절 23:09 줄 | 통합-로봇(작전 배정) |
| C3 | 탐색 경계 목표가 벽 0.1~0.2 m 안쪽에 잡힘(조정기 `freeze_clearance_m` 0.2 vs Nav2 로봇 반지름 0.189·inflation 0.45) → RPP 충돌 판정·인내 초과·회전 실패 → FAILED | 2절 22:55 줄, 로그 개수 | 통합-조정(작전 배정). 여유 값은 아직 안 바꿈 |
| C4 | FAILED 뒤 다시 arm 하지 않으면 다음 경계가 나가지 않아 탐색이 경계 2개에서 시간 초과로 끝남 | 2절 22:56~23:05 | 통합-조정(작전 배정). 런북 「실패·취소·정지 뒤에는 다시 task_arm」 |
| C5 | `home_positions` 가 프로파일에 없으면 TB-6 복귀 작업이 안 나감(`NO_HOME_POSITION`) | 2절 23:08:50. `make_field_profile.py` 는 `--home` 을 줄 때만 넣는다(런북 1단계: 1차 고정 지도에서 재서 다시 만든다) | 현장에도 측정 전에는 없음. 시뮬레이션은 다음 회차에 언도킹 뒤 odom 으로 넣는다 |
| C6 | R2 뒤 런북 7.5(tb6_slam 끄기 → frozen_localization robot_a)를 안 하면 TB-6 위치가 없어 `TRAFFIC_SCOUT_POSE_UNKNOWN` | 2절 23:08:52 | 런북 단계. 시뮬레이션 실행 스크립트가 다음 회차부터 런북 명령 그대로 실행 |

### 3.2 시뮬레이션만 (scripts/sim 에서 고침·고칠 것)

| # | 무엇 | 처리 |
| --- | --- | --- |
| S1 | TurtleBot4 시뮬레이터 한 대에 ROS 프로세스 약 50개. LOCALHOST 탐색 범위 밖의 나중 참가자(slam_toolbox, ros2 CLI)가 서로 못 봐 slam_toolbox 가 configure 되지 않음 | Fast DDS 프로필(UDP `maxInitialPeersRange` 120), 시뮬레이션 셸에서만 |
| S2 | Create3 모델이 로봇마다 Gazebo Sensors system 을 올려 두 번째 로봇에서 서버 abort(`A scene node with the name sun already exists`, 4회 중 4회) | 두 번째 로봇은 Sensors plugin 을 뺀 description 으로 생성. 고친 뒤 abort 0 |
| S3 | NVIDIA 를 안 쓰면 가상 라이다 640빔이 모두 0.164 m(range_min), 실시간의 22% | PRIME offload 세 변수 |
| S4 | 준비 확인이 토픽 이름만 보면 Gazebo 가 멈춰도 「떴습니다」(bridge 가 토픽을 미리 만듦), gz 로그 색 코드로 grep 실패 | gz 로그의 world·`Entity creation successful` 를 색 코드 지우고 확인 |
| S5 | 터미널 창을 닫으면 정리가 반쯤 끊김(HUP 두 번), `/proc` 를 프로세스마다 읽어 Permission denied 가 쏟아짐 | cleanup 중 신호 무시, `grep -lsz` 한 번 |
| S6 | 우리 노드는 벽시계로 메시지 나이를 봄 | 중계가 벽시계로 다시 찍음 |
| S7 | 메시지마다 따로 찍어 깊이·camera_info exact 짝 0쌍 → 깊이 점 없음 → collision_monitor 정지 | 같은 시뮬레이션 stamp = 같은 벽시계 stamp |
| S8 | 깊이 compressedDepth 가 없어 `TASK_ARM refused NAV_NOT_READY … compressedDepth(never)` | 중계가 32FC1 m → 16UC1 mm PNG 로 변환 |
| S9 | 중계를 바꿔 띄울 때 옛 것이 남아 두 벌이 돌며 RTF 0.23 | 운영 실수. 끄고 확인 뒤 RTF 약 1.0 |
| S10 | arena 벽 두께 1 cm 를 라이다 일부가 관통(TB-5 자리에서 12,800빔 중 680빔 5.3%, 동쪽 벽, 방위 10~16°, 확인용 정답 위치로 계산) | world 몫(지도 자리). 벽을 두껍게 할지 정해야 함 |
| S11 | 회차를 다시 띄울 때마다 관제 웹 로그인이 풀림(계정 DB·세션 키를 회차마다 새로 만듦) | 계정 DB·세션 키를 회차 밖 한 폴더에서 계속 씀 |

- 실행 순서 실수 하나: 처음에는 언도킹을 조정기 뒤에 해서 출발 자세가 도크를 보는 자세로 잡혔고, TB-5 출발값 부호가 현장 값과 반대로 나왔다. 런북대로 언도킹을 SLAM·지도 고정 앞으로 옮긴 뒤 같아졌다(#367 런북도 같은 순서를 적음).

## 4. 시뮬레이션 PC 부하

| 때 | load average(22코어) | 무엇이 컸나 |
| --- | --- | --- |
| 1단계(지도 자리)와 2단계 Gazebo 동시, 화면 포함 | 70~88 | Gazebo 서버 둘(약 140~180%), 지도 자리 Gazebo 화면(약 117%), 중계(약 77~92%) |
| 2단계만, 화면 없음(headless) | 25~30 | Gazebo 서버(약 170%), 중계(약 70%), 실행기 두 벌(약 40%) |

- 작전 결정: 한 번에 한 시뮬레이션만 돌린다.
- tf 지연(현장 쪽 `odom → base_link`, 41 Hz, 10 s): 중앙값 1.5 ms, 95% 153 ms, 최대 329 ms.

## 5. 다음 회차 전에

- 시뮬레이션: `home_positions` 를 언도킹 뒤 odom 으로 넣는다(C5). 런북 7.5 와 FAILED 뒤 다시 arm 을 런북 명령 그대로 실행 스크립트에 넣는다(C4·C6). 바뀐 값은 기동 로그에 남긴다.
- world: 벽 관통(S10) 처리 여부(지도 자리).
- 우리 코드: C1~C4 는 위 표의 자리가 정한다. 값 변경은 재범 승인.

## 재현

```bash
# 저장소 루트, colcon build --packages-up-to rescue_bringup rescue_console 뒤
RUN_FULL_ADMIN_PASSWORD=<처음 한 번> SIM_PASSAGE=off scripts/sim/run_full.sh --world arena \
  --world-dir config/sim/worlds --web-host <관제 웹 바인드 주소> --headless
scripts/sim/stop_full.sh <회차 폴더>
```

## 6. 둘째 회차 (full-1015-232037)

첫 회차 뒤 바꾼 것(시뮬레이션만): 복귀 지점 `home_positions` 를 언도킹 뒤 odom 으로 넣음(C5, 기동 로그 「현장 값과 다름」), 런북 7단계 사람 몫(7.1 실패 뒤 다시 arm·TB-5 arm, 7.5 2차 고정 뒤 TB-6 위치 추정)을 런북 명령 그대로 대신하는 `scripts/sim/sim_operator.sh`(로그 `operator.log`). 빌드는 첫 회차와 같다(`src/` = main `97f2cef`). 관제 웹만 main `6c7b23d`(#366).

| 시각 | 단계 | 로그 한 줄 | 결과 |
| --- | --- | --- | --- |
| 23:20:37~23:22:35 | 기동(런북 순서) | `TB-6 ARMED`, `operator` 시작 | 첫 회차와 같은 성공 로그 |
| 23:21:30 | 사본 값 | `현장 값과 다름: home_positions 현장 없음(측정 전), 시뮬레이션 robot_a=-0.353,0.000,-2.946 robot_b=-0.353,1.82,-2.947` | |
| 23:23:47 | 재난 선포(재범, 휴대폰 웹) | `DISASTER_DECLARED` → `SEARCH_AUTO_STARTED` → `FRONTIER e2910… goal=(-1.00, -0.74)` | 첫 회차와 같은 벽 쪽 목표(C3) |
| 23:24:49 | 결과 | `RESULT task=b8596392… outcome=FAILED` | |
| 23:24:49~23:25:02 | 운영자 대역(7.1) | `OPERATOR 런북 7.1 refresh_state …` → `robot6 task_arm: success=True message='ARMED'` | 다시 arm(C4 를 런북대로 처리) |
| 23:25:02 | 둘째 경계 | `FRONTIER a116… goal=(-1.43, 0.64)` → `SEND SEARCH_AREA fb5ea732…` | |
| 23:25:26 | 결과 | `RESULT task=fb5ea732… outcome=FAILED` | 24 s 만에 실패 |
| 23:25:26~23:25:28 | 탐색 끝 | `FRONTIER none on map update 193 (1/3)` … `(3/3)` → `EXPLORATION_COMPLETE reason=EXPLORATION_COMPLETE frontiers_sent=2` | 메인 구역(인형 자리)을 보지 못하고 끝(C9) |
| 23:25:32 | 지도 고정 | `MAP_FROZEN 1` → `MAP_FREEZE 2 requested: exploration complete` → R2(sha `2f8fa0…`) | 첫 회차처럼 R1·R2 연달아 |
| 23:25:32~23:25:53 | 운영자 대역(7.5) | `tb6_slam 끄기` → `/robot6/map 발행자 수 0` → `frozen_localization robot_a 띄움` | |
| 23:25:57~23:25:59 | TB-6 고정 지도 위치 | `amcl: Setting pose … -1.001 -0.688 -1.132` → `Managed nodes are active` → `MAP_RELAY_BLOCKED another node publishes /robot6/map (SLAM still running?)` | 발행자 0 을 본 뒤 6 s 만에 막힘(C7). TB-6 위치 미확정 |
| 23:25:32 | TB-5 | `MAP_STATE robot_b … rev=1 UNKNOWN` → `rev=2 UNKNOWN` → `START_ALIGN robot_b ALIGNING: localization not VALID`, `live input rejected: Logger severity cannot be changed between calls.` | C2 와 같은 줄. 원인 확정, #370 |
| 23:25:59~23:27:11 | 운영자 대역(TB-5 arm) | `TASK_ARM refused NAV_NOT_READY missing=['nav2']` | TB-5 Nav2 가 23:21:56 `Starting managed nodes bringup...` 뒤 active 가 되지 않음(C8) |

새로 찾은 것:

| # | 무엇 | 나눔 | 넘긴 곳 · 상태 |
| --- | --- | --- | --- |
| C7 | 7.5 뒤 TB-6 `frozen_map_relay` 가 `MAP_RELAY_BLOCKED another node publishes /robot6/map` 로 고정 지도를 넘기지 못함. 운영자 대역은 띄우기 전에 `ros2 topic info /robot6/map` 발행자 0 을 봤다 | 현장에서도 나올 수 있음 [추정] | 작전에 보고. 누가 냈는지 확인 필요 [미확인] |
| C8 | TB-5 Nav2 bringup 이 고정 전(지도 없음)에 시작해 active 가 되지 않고, 고정 뒤에도 그대로라 TB-5 arm 이 `NAV_NOT_READY missing=['nav2']` | 현장에서도 나올 수 있음 [추정] | 작전에 보고 |
| C9 | 경계 두 개가 실패한 뒤 남은 경계가 없어 탐색이 끝남. 터널 1 너머 메인 구역(인형 자리)을 탐색 대상으로 잡지 못했다 | 판단 보류 [미확인]: 시뮬레이션 지도·벽 관통(S10)과 조정기 경계 선택 중 어느 쪽인지 모름 | 작전에 보고 |

- 운영자 대역이 런북대로 다시 arm 하자 첫 회차(경계 2개 뒤 600 s 시간 초과)와 달리 바로 다음 경계로 넘어갔다. C4 는 런북 단계를 하면 풀린다.
- 다음 회차는 최신 main(#369 경계 목표 사유, #370 TB-5 정렬 로그 고침)으로 다시 빌드해서 띄운다. 빌드 SHA 는 그 회차 표에 적는다.

## 7. 셋째 회차 (full-1015-235518)와 정정

회차 표·단계별 결과·나눔은 [셋째 회차 자료](sim-2026-10-16-run235518/README.md)에 있다.

- 빌드는 `fe54428`(`src/` = main `4896338`)이다. world 는 main `config/sim/worlds/arena.sdf` 다.
- 재범이 정한 단계(2026-10-16)로 보면 1 통과, 2·3 실패, 4 실행 안 됨이다.
  - 단계 2: TB-6 은 선포 2.5 s 뒤 움직였지만 22초 뒤 흰 상자 남쪽 벽 앞에서 서서 끝까지 섰다. C1 PassageStop 과 동시 실행 구간의 `invalid source` 가 원인이다.
  - 둘째 경계는 실습장 밖 가짜 경계였다(라이다 벽 관통). 목표가 TB-5 옆이라 조정기가 교통 대기로 멈췄다(C10).
- 새로 찾은 것:
  - C10: TB-5 옆 목표에서 교통 대기, 다음 경계를 고르지 않음. 실물에서도 날 수 있음 [추정].
  - 7.5 뒤 서 있는 TB-6 의 AMCL 자세가 낡아 `TRAFFIC_SCOUT_POSE_UNKNOWN`. 실물에서도 날 수 있음 [추정].
  - TB-5 `START_ALIGN_NOT_CONVERGED`. 시뮬레이션 사본 값 탓 [추정], 런북대로 다시 하자 ALIGNED.
- C7 은 운영자 대역 잘못이었고(6절 표의 「현장에서도 나올 수 있음 [추정]」을 정정), 고친 뒤 3회차에서 재현되지 않았다.
- C8 은 3회차에서 #375 분석과 같은 줄(`Failed to activate global_costmap` → `Aborting bringup`)로 났다.
- 정정 1: 1·2회차는 main world 가 아니라 가제보 실습의 임시 world 파일로 돌았다. 바깥 벽·칸막이는 같고, 터널 2(막힌 벽)와 가운데 기둥이 다르다. 1절 표의 「arena world(평면도, #364)」는 3회차부터 맞다.
- 정정 2: 3.2절 S10 의 「TB-5 자리 5.3%」는 3회차에서 재현되지 않았다. 그 대신 3회차 main world 에서 TB-6 자리 두 곳에서 바깥 벽을 지나는 빔이 있었다(남쪽 벽 2.7%, 동쪽 벽 밖 free 칸). 원인은 가제보_지도가 보고 있다.

## 8. 넷째 회차 (full-1016-012149)

회차 표·단계별 결과는 [넷째 회차 자료](sim-2026-10-16-run012149/README.md)에 있다. 빌드는 `2d7df28`(`src/` = main `f314bbf`, #379·#385·#387·#389·#390·#391·#392)이다. 단계 1 통과, 2·3 실패, 4 실행 안 됨이다. TB-6 은 첫 목표 쪽으로 돌다가 서쪽 벽이 PassageStop 상자 모서리에 들어와 출발 자리에서 한 번도 옮기지 못했다(C1). C8(TB-5 Nav2)은 고쳐져 TB-5 가 처음으로 `ARMED` 가 됐다. C10·`TRAFFIC_SCOUT_POSE_UNKNOWN`·`invalid source` 는 나지 않았다.

## 검증 범위

- 확인한 것: 2·6절 로그 원문(회차 폴더 `logs/`), 명령 사슬·깊이 점·tf 지연 측정, 확인용 Gazebo 정답 위치.
- 확인하지 않은 것: 인형 발견 → TB-5 출동 → 출구 도착(NOT_RUN), 실기, 다른 출발 자리, C7 의 발행자.

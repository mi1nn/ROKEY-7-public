# rescue_bringup

정적 JSON 배포 검사와 단일 프로세스의 합성 두 로봇 통합 리허설을 제공한다. 두 명령 모두 ROS 연결·로봇 이동은 수행하지 않는다.

프로파일 CLI는 이 패키지만 필요하다. `python -m rescue_bringup.offline_demo`는 공통 계약을 포함한 일곱 rescue 패키지 설치가 필요하다. Python metadata의 `offline` extra와 ROS package manifest에 의존성을 기록했다. 실행 명령·산출물·E3/E4 미실행 관문은 [통합 기록](../../docs/implementation/implementation-integration.md)과 [실기 안내](../../docs/process/field-runbook.md)를 따른다.

라이선스와 maintainer는 미결정이며 현재 패키지는 배포 릴리스가 아니다. runtime_enabled는 false를 유지한다.

## 수동 실습용 SLAM·Nav2 실행 구성

`mapping_navigation.launch.py`는 TurtleBot4의 SLAM + Nav2를, `localization_navigation.launch.py`는 저장 지도 기반 localization(AMCL) + Nav2를 실행한다. 두 모드 모두 RViz를 선택적으로 함께 실행한다. TurtleBot4 설치 패키지의 launch와 기본 파라미터를 재사용하며, 로봇 드라이버는 해당 로봇에서 이미 실행 중이어야 한다.

이 구성은 사람이 RViz에서 목표를 지정하는 단일 로봇 실습용이다. 기존 오프라인 CLI나 `runtime_enabled`를 활성화하지 않으며, 프로젝트 임무 명령·정지/권한·watchdog 및 `RobotPose`/`MapState` 연결은 포함하지 않는다. SLAM은 목표 없이 스스로 탐색하지 않는다. 복사해 둔 `rescue_perception` 내부 실습 파일은 이 launch에서 사용하지 않는다.

### 빌드와 준비

ROS 2 Jazzy, `turtlebot4_navigation`, `turtlebot4_viz`가 설치된 작업 PC에서 저장소 루트 기준으로 실행한다. 별도 TurtleBot 작업공간으로 설치했다면 그 작업공간의 `install/setup.bash`도 먼저 source한다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to rescue_bringup
source install/local_setup.bash
```

위 명령은 rescue_bringup과 작업공간 안의 의존 패키지를 함께 빌드한다. 실제 로봇의 domain·namespace·네트워크를 확인하고, 이 로봇을 사용하는 다른 SLAM/localization/Nav2 프로세스를 종료한 뒤 시작한다. 두 모드를 동시에 실행하지 않는다. 로봇 이동은 사람이 현장에서 관찰하고 정지 수단을 준비한 상태에서 수행한다.

실행 예시의 값을 실제 확인한 설정으로 지정한다. 로봇 ID·domain은 배포 결정 Q-01의 확정을 대신하지 않는다.

```bash
export ROS_DOMAIN_ID=<실제_로봇_domain>
```

### SLAM으로 지도 작성하면서 목표 주행

```bash
ros2 launch rescue_bringup mapping_navigation.launch.py \
  namespace:=<실제_로봇_namespace> use_sim_time:=false rviz:=true
```

LiDAR·TF를 확인하고 RViz에서 목표를 지정한다. Nav2는 실행 중인 costmap과 센서 입력으로 경로를 계산한다. 실제 장애물 회피 성능은 별도 실기 확인이 필요하다. 다른 SLAM이나 AMCL localization을 동시에 실행하지 않는다.

별도 터미널에서 동일한 ROS 환경과 domain을 사용하여 저장한다. `<출력_경로_접두어>`는 확장자 없는 경로이며, 상위 디렉터리는 미리 만들어 둔다.

```bash
ros2 run nav2_map_server map_saver_cli \
  -f <출력_경로_접두어> \
  --ros-args -p map_subscribe_transient_local:=true \
  -r __ns:=<실제_로봇_namespace>
```

PGM/YAML은 점유 지도다. 이것만으로 기존 SLAM pose graph를 이어서 매핑하거나 새 지도와 이전 지도의 좌표 원점이 같다고 가정하지 않는다.

### 저장 지도로 위치 추정하면서 목표 주행

SLAM 모드의 로봇 주행을 현장에서 정지·확인한 뒤 해당 launch 터미널에서 `Ctrl+C`로 종료한다. 저장 지도 경로는 작업 PC에서 접근 가능한 YAML의 절대 경로로 명시한다. YAML이 참조하는 이미지도 존재해야 한다. 지도 자산은 Python wheel/share에 복사되지 않는다.

```bash
ros2 launch rescue_bringup localization_navigation.launch.py \
  namespace:=<실제_로봇_namespace> \
  map:=<저장_지도_yaml_절대_경로> \
  use_sim_time:=false rviz:=true
```

RViz에서 `2D Pose Estimate`로 실제 초기 위치와 방향을 지정하고 scan과 지도가 맞는지 확인한다. 이후 `Nav2 Goal`로 주행을 확인한다. localization은 저장 지도를 이용해 현재 위치를 추정하며, Nav2 costmap의 새 장애물이 저장 PGM/YAML을 수정하지는 않는다.

### 실행 인자와 검증 범위

| 인자 | 적용 | 기본값 |
| --- | --- | --- |
| `namespace` | 두 모드 | 필수, 실제 로봇 namespace |
| `map` | localization 모드 | 필수, 저장 지도 YAML 경로 |
| `use_sim_time` | 두 모드 | `false` |
| `rviz` | 두 모드 | `true`; 화면 없는 환경은 `false` |
| `slam_params_file` | SLAM 모드 | TurtleBot4 `config/slam.yaml` |
| `localization_params_file` | localization 모드 | TurtleBot4 `config/localization.yaml` |
| `nav2_params_file` | 두 모드 | TurtleBot4 `config/nav2.yaml` |

`--show-args`는 실행 인자와 launch 읽기만 확인하며 ROS 노드를 시작하지 않는다. 이 검사나 빌드 성공을 실제 map/TF·초기 위치·주행 성공으로 해석하지 않는다. 실제 지도 작성·저장·재위치 추정·장애물 회피와 두 로봇/프로젝트 인터페이스 통합은 E3/E4 NOT_RUN이며 현장 증거를 추가해야 한다.

## 운영 PC launch (`operations.launch.py`, 2026-10-12)

운영 PC(프로파일 `operations` 호스트)에서 db_writer(현장 상태 DB 쓰기 + 운영자 명령 API)와, 원하면 고정 웹캠 노드를 띄운다. 로봇 ID·역할·토픽은 배치 프로파일 v2.1(`mission.roles`)에서 계산하므로, 실기 로봇이 바뀌면 프로파일 파일만 바꾼다. 실기 값(로봇 ID·주소)은 저장소 밖 프로파일에 두고 경로로 넘긴다. 변환은 `rescue_bringup/operations_launch.py`(ROS 없음, `tests/test_bringup_operations_launch.py`)이고, 토픽·서비스 이름은 조정기 상수와 같은지 시험이 대조한다.

```bash
ros2 launch rescue_bringup operations.launch.py profile_path:=<프로파일.json> \
  db_path:=<site_state.sqlite3> api_port:=<포트> command_timeout_s:=3.0 \
  report_db_path:=<reports.sqlite3> envelope_schema_version:=rescue-envelope/1 report_publisher_id:=<id> \
  webcam_params_file:=<webcam_node.yaml> \
  video_port:=<포트> video_bind_host:=<주소> video_stale_s:=2.0
```

`webcam_params_file` 을 빼면 웹캠 없이, `video_*` 를 빼면 판정 영상 중계 없이 뜬다.

- 웹캠 고르기: `webcam_device:=<이름>` 을 주면 웹캠 설정의 `device_path` 대신 그 카메라를 연다(예: USB 카메라를 꽂았을 때). 이름은 `ls /dev/v4l/by-id/` 의 `*-video-index0` 하나이고, 경로를 줘도 된다. launch 로그 `plugged cameras: …` 에 지금 꽂힌 카메라가 나온다. 없는 카메라를 주면 launch 가 꽂힌 목록과 함께 멈춘다. 빼면 설정 파일의 카메라다.
- 실시간 SLAM 지도(2026-10-13, 재범 결정: TB-6): `map_topic:=/robot6/map map_dir:=<폴더>` 를 주면 `map_snapshot` 노드가 지도(OccupancyGrid, RELIABLE·TRANSIENT_LOCAL)를 받을 때마다 `<폴더>/live_map.yaml` 과 PGM 을 쓴다. 둘은 같이 줘야 하고, 토픽은 고정 뒤 바뀔 수 있어 인자로 둔다. launch 가 관제 웹 값 `console --live-map <폴더>/live_map.yaml --live-map-stale-s <s>` 를 로그로 남긴다. 회전된 지도 원점(yaw ≠ 0)은 관제 웹이 그리지 못해 쓰지 않고 경고한다. 토픽에 발행자가 있는 동안 `live_map.alive` 를 갱신해, 고정 뒤 다시 오지 않는 지도도 낡음으로 보이지 않게 한다.

- 판정 영상 중계(`mjpeg_relay`, #153): `video_port` 를 주면 켠다. 웹캠을 이 PC 에서 띄울 때만 가능하고(`webcam_params_file` 없이 주면 launch 가 멈춤), 입력 토픽은 웹캠 설정의 `debug_image_topic`, 주소 경로는 `/<camera_id>.mjpg` 다. `video_bind_host`·`video_stale_s` 는 켤 때 필수다. launch 가 관제 웹에 줄 값을 로그로 남긴다: `console --camera cam_fixed=http://<video_bind_host>:<video_port>/cam_fixed.mjpg`. 관제 웹을 보는 브라우저가 이 주소에 닿아야 하므로, 다른 기기에서 볼 때는 `video_bind_host` 를 그 기기가 닿는 주소로 준다.

- `db_path` 는 프로파일의 로봇 ID 구성(지금은 `robot_a`·`robot_b`)이 같으면 같은 파일을 계속 쓴다. 다른 로봇 ID 로 만든 파일을 열면 db_writer 가 `POLICY_MISMATCH` 로 시작하지 않으므로, ID 구성을 바꿀 때만 새 파일을 쓴다. 더 낮은 schema 파일은 처음 열 때 올라가므로 그 전에 복사본을 남긴다.
- 운영자 명령 API 는 `127.0.0.1` 에서만 받는다(API 자체에 로그인 확인 없음). 관제 웹은 이 launch 밖의 웹 서비스로, `--site-state <db_path> --operations http://127.0.0.1:<api_port>` 로 같은 DB·API 를 가리킨다.
- `command_timeout_s` 는 관제 웹의 5초보다 짧아야 한다.
- 프로파일에 `mission.roles` 가 없으면(v1) 시작하지 않는다.

| 검증 | 결과 |
| --- | --- |
| E2: `tests/test_bringup_operations_launch.py` | 통과 |
| 로컬 ROS(jazzy, `ROS_DOMAIN_ID=77`, LOCALHOST, Discovery Server 해제, 한 PC, 2026-10-12, 스크래치 colcon 설치본): 예시 프로파일로 이 launch + 조정기 노드, 가짜 로봇 상태·DISASTER_SIGN | db_writer 기동(`allowed_robot_ids` = robot_a·robot_b), 조정기 전 API 503, 재난 전 START_SEARCH 거절 `NOT_AUTHORIZED`, 재난 뒤 접수, 명령 2행·임무 DISASTER/SEARCHING·로봇 2행(SCOUT·SUPPORT) 저장, 종료 코드 0. v1 프로파일은 launch 오류로 시작하지 않음 |

NOT_RUN: 웹캠 노드를 이 launch 로 띄우는 것(카메라·모델 필요), 운영 PC·실기 프로파일, 조정기·로봇 PC launch.

## 현장 실행 순서: PC 역할별 (2026-10-13, main 기준)

PC 는 프로파일 호스트 id 로 적는다. 오늘 확정 배치는 이렇다. 최종 구성에는 이 넷과 별도로 전용 Discovery PC 1대(서버만, 프로파일 호스트 아님)가 있다. 시험 중에는 그 PC 없이 권고안 A 로 돈다(0단계).

| 호스트 id | 물리 PC | 띄우는 것 |
| --- | --- | --- |
| `operations` + `coordination` | 같은 PC(웹·DB·웹캠·조정기) | `operations.launch.py`(db_writer·report_node·webcam_node·mjpeg_relay), 관제 웹, `coordinator.launch.py` |
| `compute_a` | TB-6(robot_a, `/robot6`, SCOUT) 담당 PC | 위치 추정, `robot.launch.py robot_id:=robot_a`, 탐지·대상 위치·재확인 서버(통합-인식) |
| `compute_b` | TB-5(robot_b, `/robot5`, SUPPORT) 담당 PC | 위치 추정, `robot.launch.py robot_id:=robot_b` |

값은 세 곳에서만 온다. 첫째는 배포 프로파일 v2.1(저장소 밖)로, 로봇·역할·namespace·readiness·검색 영역·`map_basis`·`home_positions` 를 준다. 둘째는 현장 실행 파일 `config/field/search_area.field.json` 으로, 정책·로봇별 값을 준다. 셋째는 launch 인자로, 현장에서 재는 값이다. 이 launch 들은 arm·임무·이동 명령을 보내지 않는다. 로봇이 움직이는 단계는 사람이 지켜본다.

| 기능 | 상태 |
| --- | --- |
| 운영 launch(db_writer·보고 저장·웹캠·영상 중계), 조정기·로봇 launch, 지도 해시, 준비 점검 | 있음(#144·#152·#156·#158) |
| TB-5 자동 접근(APPROACH_TARGET), TB-6 재관측(REOBSERVE_TARGET)+재확인 관측, 로봇 위치·지도 상태 발행 | 있음(#166·#179·#176). 재확인 서버: 판정·action 서버 코드는 있음(#165), 실행 노드와 센서 입력(프레임·스캔) 연결은 통합-인식 다음 PR 이다. 그 전에는 서버가 없어 재관측 결과가 `has_recheck=false` 다 |
| 복귀(RETURN_HOME) | 있음(#192). 자동 도킹은 없다(도킹은 사람이 감독) |
| TB-5 로봇 launch | 있음(현장 실행 파일 `robots.robot_b`: 접근·복귀만, Pi 시계 150 ms) |
| watchdog 현장 값 | #132 임시값(#94 미확정). 영구 정지와 리셋은 [rescue_robot README](../rescue_robot/README.md) |

### 0단계 — 모든 PC, 그리고 로봇 Pi

로봇 Pi 두 대(재범이 Pi 에서 본다. 근거 [Pi 설정 기록](../../docs/evidence/field-2026-10-13-pi-settings.md)):

| 확인 | 명령 | 맞는 값 | 메모 |
| --- | --- | --- | --- |
| 무선 절전 | `iw dev wlan0 get power_save` | `Power save: off` | 일시 설정이다. 재부팅하면 켜지므로 켤 때마다 `sudo iw dev wlan0 set power_save off` |
| 자동 업데이트 | `systemctl is-enabled apt-daily.timer apt-daily-upgrade.timer unattended-upgrades` | 셋 다 `disabled` | 재부팅해도 유지 |
| Pi 시계 | `chronyc tracking` | — | 실기 당일 다시 잰다. `rescue_contracts/clock_measurement.py` 의 `parse_chrony` 식(차이 + root delay/2 + root dispersion + 갱신 간격 × 100 ppm)으로 계산한다. 현장 실행 파일 `robots.<id>.task.live.sensor_host_clock_error_bound_ns` 의 2026-10-13 측정값은 150 ms 다(TB-6 약 109 ms, TB-5 약 107 ms). 계산값이 150 ms 를 넘으면 값을 바꾼다. -1 이면 로봇 위치가 UNKNOWN 이고 TB-5 자동 출동이 보류된다 |
| 카메라(TB-6) | 언도킹 뒤 `ros2 topic hz /robot6/oakd/rgb/image_raw/compressed` | 약 30 Hz(2026-10-13 측정 28.7 Hz) | 도킹 중에는 카메라가 꺼져 있다. 언도킹한 뒤에 확인한다([OAK-D 기록](../../docs/evidence/field-2026-10-13-oakd-robot6.md)) |

모든 PC(같은 로봇망 SSID 여야 한다. 2026-10-13 운영 PC 가 다른 SSID 라 TB-5 Pi ping 이 100% 실패했다 — [운영 PC 기록](../../docs/evidence/field-2026-10-13-operations-host.md)):

```bash
ping -c 20 <TB-5 Pi 주소>; ping -c 20 <TB-6 Pi 주소>      # 손실 0% 인지(로봇 담당 PC 는 자기 로봇만)
iw dev <무선 장치> get power_save                         # PC 쪽 무선 절전(끌지는 재범이 정함)
# Discovery — 세미콜론 자리 번호 = 서버 ID. 지금 어느 구성인지 먼저 정하고 그 줄을 쓴다(재범 확인 2026-10-13 15:40).
# [시험 중] 권고안 A: 각 PC 가 로봇 Pi 서버에 직접(TB-5 Pi = ID 5, TB-6 Pi = ID 6)
#   compute_a: ';;;;;;<TB-6 Pi>:11811'   compute_b: ';;;;;<TB-5 Pi>:11811'
#   operations·coordination: ';;;;;<TB-5 Pi>:11811;<TB-6 Pi>:11811'
# [최종] 전용 Discovery PC 1대(옛 중앙 서버 PC, 서버 ID 0): 모든 PC 가 '<전용 PC>:11811'(자리 0)
#   로봇이 전용 서버에 보이게 하는 방법은 결정 대기 — 로봇 Pi 에 외부 서버 추가(로봇 설정, 등급 4) 또는 서버끼리 연결.
#   정해지기 전에는 최종 구성으로 바꾸지 않는다(로봇 토픽이 안 보인다).
export ROS_DOMAIN_ID=<프로파일 domain_id>
export ROS_DISCOVERY_SERVER='<위 값, 프로파일 hosts[<host id>].network 와 같게>'
ros2 daemon stop && ros2 daemon start
ROS_SUPER_CLIENT=True ros2 topic list --no-daemon | grep -E '^/robot[56]/' | cut -d/ -f2 | sort | uniq -c
python3 scripts/field/preflight_role.py --role <host id> --profile <profile.json> \
  --field config/field/search_area.field.json [--webcam-params <webcam_node.yaml>] [--map <map.yaml>]
```

현장 프로파일에는 `map_basis`(frame `map`)와 `home_positions`(TB-6 도킹 접근 지점, TB-5 출구; 현장에서 잼)가 있어야 한다. 노드는 super client 가 아니다(`ROS_SUPER_CLIENT=False`). 준비 점검이 모두 PASSED 여야 다음으로 간다.

### 1단계 — operations + coordination PC

```bash
# 운영: 현장 상태 DB·명령 API·구조 보고 저장·웹캠·판정 영상 중계
ros2 launch rescue_bringup operations.launch.py profile_path:=<profile.json> \
  db_path:=<site_state.sqlite3> api_port:=<api 포트> command_timeout_s:=3.0 \
  report_db_path:=<reports.sqlite3> envelope_schema_version:=rescue-envelope/1 report_publisher_id:=<id> \
  webcam_params_file:=<webcam_node.yaml> video_port:=<영상 포트> video_bind_host:=<브라우저가 닿는 주소> video_stale_s:=2.0
# → 로그 "console --camera cam_fixed=http://..." 를 아래 --camera 에 그대로

# 관제 웹(계정 DB·비밀키 준비는 rescue_console README)
python3 -m rescue_console --accounts <계정 DB> --site-state <site_state.sqlite3> \
  --operations http://127.0.0.1:<api 포트> --camera <launch 로그의 값> --camera-stale-s 2.0 \
  --robot-name robot_a=TB-6 --robot-name robot_b=TB-5 --map <map.yaml> --port 8080
#   PR #185 머지 뒤: --support-dispatch AUTO 를 더한다(없으면 관제 웹이 TB-5 출동 승인 버튼을 켠다. 조정기 support_dispatch 와 같게)

# 조정기(live 입력). 경로·관측 목표는 아직 자리표시, 웹캠 감시 ROI 는 현장에서 잰 값(재범 결정 6A)
ros2 launch rescue_bringup coordinator.launch.py profile_path:=<profile.json> \
  field_path:=$PWD/config/field/search_area.field.json \
  route_segment_goals:='[0.0, 0.0, 0.0]' observe_goal:='[0.0, 0.0, 0.0]' route_segment_passage_free:='[false]' \
  camera_watch_roi:='<현장에서 잰 값, 예: [0.1, 0.2, 0.6, 0.9]>'
# ↑ 자리표시다. '[x0, y0, x1, y1]' 글자를 그대로 붙이면 JSON 이 아니라 launch 가 exit 1
```

성공 로그: `rescue_db_writer git=... api=http://127.0.0.1:<port>`, `rescue_report_node ... receipts=/rescue/report_receipts`, `webcam_node git=... model=...`, `mjpeg_relay ...`, 조정기 `live task source enabled` · `central live clock: SYNCED` · `awaiting RViz search goal on /rescue/search_goal`. 조정기는 `central live clock: SYNCED` 전에 `central live clock: waiting for clock measurement` 를 한 번 내는 것이 정상이다(첫 측정 전).

TB-5 가 대상에서 출구로 떠나기 전 기다리는 시간 `support_exit_wait_s`(15 s, 재범 결정 2026-10-13)는 launch 인자가 아니라 현장 실행 파일 coordinator 절의 값이다. 리허설 중에 바꾸면 조정기 launch 를 다시 띄운다.

### 2단계 — compute_a (TB-6), compute_b (TB-5)

두 로봇은 같은 지도 파일 하나를 쓴다(`map_basis` 하나). 언도킹은 기존 감독 절차로 한다.

```bash
# <ns> = /robot6 (compute_a) 또는 /robot5 (compute_b), <id> = robot_a 또는 robot_b
ros2 launch turtlebot4_navigation localization.launch.py namespace:=<ns> map:=<map.yaml> use_sim_time:=false
python3 scripts/field/map_occupancy_sha256.py <ns>/map map        # stdout 64자리 해시, 실패면 exit 2
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_SUPER_CLIENT=False \
ros2 launch rescue_bringup robot.launch.py profile_path:=<profile.json> \
  field_path:=$PWD/config/field/search_area.field.json robot_id:=<id> \
  occupancy_sha256:=$(python3 scripts/field/map_occupancy_sha256.py <ns>/map map)
```

성공 로그: `robot <id> ns=<ns> ... map_sha256=...`, `[<ns 이름>.rescue_command_watchdog]: git_sha=...`, `installed task runtime; input_mode=LIVE_ROS; explicit arm required`, `MAP_STATE <id> ... quality=VALID`(AMCL 이 자리를 잡은 뒤).

compute_a 에서는 TB-6 탐지와 대상 위치도 띄운다(통합-인식). 탐지는 `amr_perception.launch.py` 이고, 인자는 그 파일 머리말과 [rescue_perception README](../../src/rescue_perception/README.md) 에 있다. `sensor_host_clock_*` 는 위 Pi 시계 값과 같다. 대상 위치는 `target_locator`(같은 README 「지도 좌표 TargetLocation」)다. 재확인 서버 `/rescue/robots/robot_a/observe_target` 은 실행 노드가 생긴 뒤(통합-인식 다음 PR) 띄운다. 코드(#165)만으로는 띄울 실행 파일이 없다.

### 3단계 — 출동 준비와 실행 (operations + coordination PC 에서, 사람이 지켜봄)

```bash
# 각 로봇: 정지 상태에서 AMCL 갱신 한 번 → 조정기 상태 재발행 → 실행기 arm(#132 start_search.py 순서)
ros2 service call <ns>/request_nomotion_update std_srvs/srv/Empty {}
ros2 service call /mission_coordinator/refresh_state std_srvs/srv/Trigger {}
ros2 service call <ns>/rescue_command_adapter/task_arm std_srvs/srv/Trigger {}      # success=True
```

1. RViz(2D Goal Pose)로 `/rescue/search_goal` 에 TB-6 수색 목표를 고른다. 조정기 로그 `SEARCH_GOAL_SELECTED`.
2. TB-6 과 TB-5 를 모두 arm 한다. TB-5 는 자동 출동이라 대상이 나오기 전에 arm 돼 있어야 한다. 실행기는 arm(latch ARMED)일 때만 READY 를 내고(`task_status.py`), 조정기는 READY·최신 상태인 SUPPORT 에만 task 를 낸다(통합-조정 확인 2026-10-13). arm 하지 않으면 조정기는 기다리기만 한다. PR #196 머지 뒤에는 SystemState `reason=SUPPORT_NOT_READY` 로 보이고, 관제 웹에는 「TB-5 준비 안 됨 — arm 필요」로 보인다(PR #185 머지 뒤).
3. 웹캠 앞 흔들림 → 관제 화면 모드 DISASTER → 관제 웹 출동 버튼(START_SEARCH).
4. 정지: `ros2 service call <ns>/rescue_command_adapter/task_stop std_srvs/srv/Trigger {}` 로 하고, 실제로 멈췄는지 눈으로 확인한다. watchdog 이 세운 경우(`watchdog STOP_LATCHED`)는 rescue_robot README 의 리셋 절을 본다.
5. 다시 하려면 조정기·로봇 launch 를 다시 띄우고 새 목표를 고른다.

### 참고

- 정렬 유효 구간은 #132 처럼 launch 시각 기준 `now - alignment.backdate_s` ~ `now + alignment.window_s` 로 선언한다(관측값이 아님).
- ROS 파라미터로 빈 배열을 넘길 수 없다. 그래서 `route_segment_passage_free:='[]'` 와 빈 `camera_topics` 는 launch 가 거절하고, 표시 개수는 구간 수와 같아야 한다(#163).
- 문자열 값은 `ParameterValue(str)` 로 넘긴다(`ros_parameters.typed`).
- 시험: `tests/test_bringup_field_launch.py`(#132 현장 스크립트와 같은 파라미터인지).

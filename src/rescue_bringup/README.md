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

## SEARCH_AREA 현장 실행: 운영 · 조정기 · 로봇 launch (2026-10-13)

| 무엇 | 상태 (2026-10-13) |
| --- | --- |
| 운영 `operations.launch.py`: db_writer + 구조 보고 저장(report_node) + 웹캠 | 있음(#134·#144) |
| 운영 launch 의 웹캠 판정 영상 중계(mjpeg_relay, `video_port`) | 있음(#158) |
| 조정기 `coordinator.launch.py`, 로봇 `robot.launch.py`, 지도 해시 `scripts/field/map_occupancy_sha256.py` | 있음(#152) |
| 지도 기준 값 `map_basis`(프로파일, 조정기·로봇·탐지 launch 공용) | 있음(#145). 현장 프로파일에 넣어야 한다(frame `map`) |
| PC 역할별 준비 점검 `scripts/field/preflight_role.py`, 0단계 Discovery 확인 | 있음(#156) |
| robot_b(robot5) 로봇 launch | 안 됨: 현장 실행 파일에 `robots.robot_b` 값 없음(AMR2 시나리오 #131·#133 결정 대기) |
| 구간 이동·재관측·복귀(MOVE_SEGMENT·REOBSERVE_TARGET·RETURN_HOME)의 LIVE_ROS 입력 | 안 됨: LIVE_ROS 는 SEARCH_AREA 만([rescue_robot README](../rescue_robot/README.md)) |
| watchdog 현장 값 | #132 임시값, 확정은 #94. 영구 정지(STOP_LATCHED)와 리셋은 [rescue_robot README](../rescue_robot/README.md) 「2026-10-13 현재」 절 |

세 launch 로 웹캠 → 재난 판정 → 관제 출동(START_SEARCH) → AMR1 SEARCH_AREA(Nav2) 를 띄운다. 값은 세 곳에서만 온다.

| 어디서 | 무엇 |
| --- | --- |
| 배포 프로파일 v2.1(저장소 밖, 현장 값) | 로봇 ID·역할·namespace·ROS 이름·readiness·검색 영역(`entry_yaw`)·지도 기준 `map_basis` |
| 현장 실행 파일 `config/field/search_area.field.json` | #132 현장 스크립트(`scripts/field/search-area/start_*.sh`)의 정책 값. 조정기와 로봇이 같아야 하는 값(lease·authority epoch·coordinator ID·envelope·시계)은 양쪽에 적고 launch 가 같은지 검사한다 |
| launch 인자(기본값 없음) | 아직 정하지 않은 Phase 1 경로·관측 목표(#132 의 `[0, 0, 0]` 자리표시값), 현장에서 잰 지도 해시 |

먼저 모든 PC 에서(0단계) Discovery 연결과 준비 상태를 본다. 각 PC 는 로봇 Pi 온보드 서버에 직접 붙는다([현장 결정 기록](../../docs/design/network-discovery.md#11-현장-적용-기록-2026-10-13), 권고안 A, 2026-10-13). 로봇 담당 PC 는 자기 로봇 서버 하나, 조정·운영 PC 는 두 서버 모두다. 로봇 설정은 바꾸지 않는다.

준비 점검은 JSON 형식 설정도 읽는다(YAML의 부분집합). 다른 YAML은 PyYAML이 필요하다. 파서가 없으면 해당 파일 검사는 `NOT_RUN`, 문법이 틀리면 `FAILED`이며 모두 `PASSED`가 되기 전에는 준비 완료가 아니다.

```bash
# 이 PC 의 값 = 프로파일 hosts[<이 PC 의 host id>].network. 세미콜론 자리 번호가 서버 ID 다
# (예: ID 6 하나면 ';;;;;;<주소>:11811', ID 5·6 이면 ';;;;;<주소5>:11811;<주소6>:11811')
export ROS_DOMAIN_ID=<프로파일 domain_id>
export ROS_DISCOVERY_SERVER='<위 규칙의 값>'
ros2 daemon stop && ros2 daemon start          # 바뀐 서버 목록으로 CLI 데몬을 다시 띄운다
# 확인만 super client 로: 조정·운영 PC 는 두 로봇, 로봇 담당 PC 는 자기 로봇 토픽이 보여야 한다
ROS_SUPER_CLIENT=True ros2 topic list --no-daemon | grep -E '^/robot[56]/' | cut -d/ -f2 | sort | uniq -c
# 노드는 super client 가 아니다(ROS_SUPER_CLIENT=False, 프로파일 super_client=false)
python3 scripts/field/preflight_role.py --role <host id> --profile <profile.json> \
  --field config/field/search_area.field.json [--webcam-params <yaml>] [--map <map.yaml>]   # 모두 PASSED 인지
```

두 로봇의 온보드 서버 ID 는 서로 달라야 한다. 같으면 두 서버에 붙는 PC 가 한 자리에 두 서버를 적을 수 없고, 프로파일도 `server_id must be unique` 로 거절한다.

순서(로봇을 움직이는 단계는 사람이 지켜본다. 이 launch 들은 arm·임무·이동 명령을 보내지 않는다):

```bash
# 운영 PC: 현장 상태 DB·명령 API(+ 구조 보고 저장). 웹캠이 다른 PC 면 webcam_params_file 을 뺀다
ros2 launch rescue_bringup operations.launch.py profile_path:=<profile.json> \
  db_path:=<site_state.sqlite3> api_port:=<port> command_timeout_s:=3.0 \
  report_db_path:=<reports.sqlite3> envelope_schema_version:=rescue-envelope/1 report_publisher_id:=<id> \
  [webcam_params_file:=<webcam_node.yaml> [video_port:=<port> video_bind_host:=<address> video_stale_s:=2.0]]
# 관제 웹: --site-state <db_path> --operations http://127.0.0.1:<port> --camera <launch 로그의 console --camera 값>

# 웹캠 PC(운영 PC 와 다를 때): 웹캠 노드만. 파라미터 예시는 rescue_perception share/config/webcam_node.example.yaml
# 프로파일 site_cameras[].host 는 4대 중 그 PC 의 호스트 id(예: coordination)
ros2 run rescue_perception webcam_node --ros-args --params-file <webcam_node.yaml>

# 로봇: 위치 추정(저장 지도) 한 번, 그 지도 해시 측정
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot6 map:=<map.yaml> use_sim_time:=false
python3 scripts/field/map_occupancy_sha256.py /robot6/map map        # stdout: 64자리 해시, 실패 시 exit 2

# 로봇: Nav2(최종 cmd_vel 은 watchdog 로) + command_watchdog + task_input
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_SUPER_CLIENT=False \
ros2 launch rescue_bringup robot.launch.py profile_path:=<profile.json> \
  field_path:=$PWD/config/field/search_area.field.json robot_id:=robot_a \
  occupancy_sha256:=$(python3 scripts/field/map_occupancy_sha256.py /robot6/map map)

# 조정 PC: mission_coordinator(live 입력, 노드 이름 mission_coordinator)
ros2 launch rescue_bringup coordinator.launch.py profile_path:=<profile.json> \
  field_path:=$PWD/config/field/search_area.field.json \
  route_segment_goals:='[0.0, 0.0, 0.0]' observe_goal:='[0.0, 0.0, 0.0]' route_segment_passage_free:='[false]' \
  [camera_watch_roi:='[x0, y0, x1, y1]']   # 빼면 조정기가 웹캠 감시 지정을 거절(화면 전체 기본값 없음)
```

- 로봇 2(robot_b)는 `robot_id:=robot_b` 로 같은 launch 를 쓴다. 현장 실행 파일에 `robots.robot_b`(`task.publisher_id`·`task.mapping_authority`)가 없으면 launch 가 멈춘다. #132 에 robot_b 값이 없어 지어내지 않았다.
- 정렬 유효 구간은 #132 처럼 launch 시각 기준 `now - alignment.backdate_s` ~ `now + alignment.window_s` 로 선언한다(관측값이 아님). launch 로그에 해시·구간을 남긴다.
- ROS 파라미터로는 빈 배열을 넘길 수 없다(launch_ros 거절, `-p`·`--params-file` 은 미설정으로 남음 [측정 2026-10-13]). 그래서 `route_segment_passage_free:='[]'` 와 빈 `camera_topics` 는 launch 가 거절한다.
- 문자열 값은 `ParameterValue(str)` 로 넘긴다(`ros_parameters.typed`). 일반 문자열은 launch_ros 가 YAML 로 다시 읽어 숫자처럼 보이는 문자열이 숫자가 된다.
- 시험: `tests/test_bringup_field_launch.py`. 만든 파라미터가 #132 `start_coordinator.sh`·`start_robot.sh`·`nav2_watchdog.launch.py` 와 같은지(같은 프로파일 값일 때), 서로 맞아야 하는 값과 자리표시 인자를 검사하는지 본다.

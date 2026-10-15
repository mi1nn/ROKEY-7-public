# rescue_bringup

## 현재 상태 (2026-10-14, main 4163a25)

코드 기준이다. 「실기 NOT_RUN」은 코드와 로봇 없는 시험은 있지만 실제 로봇에서 확인하지 않았다는 뜻이다(아래 「실기 NOT_RUN」 절). 맨 아래 「기록」 절은 당시 설명이다. 이 표와 다르면 이 표가 지금 상태다.

| 기능 | 상태 | 근거 |
| --- | --- | --- |
| 현장 launch: `operations`·`coordinator`·`robot`·`tb6_slam`·`frozen_localization`·`map_freeze`·`passage_points`·`overhead_passage` | 구현됨, 실기 NOT_RUN(전체 회차) | `launch/*.launch.py`, `rescue_bringup/field_launch.py`, #144·#152 |
| 현장 정지 스크립트 | 구현됨 | `scripts/field/stop_robot.sh`, #256 |
| `<ns>/map` 에 이미 발행자가 있으면 SLAM 기동 거절 | 구현됨, 실기 NOT_RUN | `rescue_bringup/map_guard.py:check_no_map_publisher`, #256 |
| 깊이 정지 `PassageStop` 없음(재범 결정 2026-10-16). 터널은 `overhead_passage` BLOCKED → `passage_keepout` 진입 금지 지도 → 두 로봇 Nav2 `KeepoutFilter` | 구현됨, 실기 NOT_RUN | `field_launch.py:nav2_with_passage`, 현장 파일 `robot` 절 `passage.*`, #226 |
| 통로 감속 `PassageSlow`: 깊이 점 0.23 ~ 1.0 m, 앞 0.4 ~ 1.2 m, 속도 ×0.3 | 구현됨, 실기 NOT_RUN | 같은 함수, `passage.slow.*`·`passage.overhead.*`, #260 |
| 깊이 점 `<ns>/passage_points`(depth_image_proc) | 구현됨, 실기 NOT_RUN | `launch/passage_points.launch.py`, `field_launch.py:passage_points_launch_values`, #226 |
| 지도 위 터널 판정 `overhead_passage` | 구현됨, 실기 NOT_RUN | `launch/overhead_passage.launch.py`, `field_launch.py:overhead_passage_launch_values`, #259·#264 |
| 지도 두 번 고정 노드, 고정 지도 위치 추정 | 구현됨, 실기 NOT_RUN | `launch/map_freeze.launch.py`, `launch/frozen_localization.launch.py`, #213·#242·#255 |
| TB-5 출발 정렬 노드(`rescue_start_align`). `frozen_localization.launch.py robot_id:=robot_b` 가 AMCL 옆에 띄운다. 로그 끝에 ` + start_align` 이 붙는다. TB-6 에는 띄우지 않는다 | 구현됨, 실기 NOT_RUN | `launch/frozen_localization.launch.py`, `field_launch.py:START_ALIGN_KEYS`, [rescue_robot README](../rescue_robot/README.md) |
| TB-5 출발값 `robots.robot_b.initial_pose_xy_yaw` | `[0.0, -1.86, 0.0]`(현장 파일 `robots.robot_b`). TB-6 오른쪽 1.86 m 를 줄자로 쟀다(2026-10-15, 둘 다 언독). 앞쪽 거리와 각도는 「옆에 있음」으로 읽은 값이다 | 아래 2단계 「TB-5 출발값 재기」 |
| TB-5 Nav2 는 1차 고정 지도에서 위치가 잡힌 뒤 켬. `robot.launch.py` 가 FROZEN_MAP_AMCL 로봇의 Nav2 를 `autostart:=false` 로 띄우고, `nav2_start_after_map` 이 `map → base_link` 가 생기면 `manage_nodes` STARTUP 을 한 번 보낸다. 실패는 `NAV2_STARTUP_FAILED`·`NAV2_STARTUP_NO_ANSWER` 로 남고 실행기는 NOT_READY 그대로다 | 구현됨, 실기 NOT_RUN | `rescue_bringup/nav2_start.py`, `field_launch.py:robot_launch_values`, #375 |
| Nav2 복구에서 후진 빼기 | 구현됨(#267 머지), 실기 NOT_RUN. 복구는 비용 지도 지우기 → 제자리 회전 → 대기 | `nav2_recovery.py`, #267(재범 승인 2026-10-14) |
| Nav2 경로 따라가기(FollowPath) | 구현됨, 실기 NOT_RUN. DWB 대신 Rotation Shim + Regulated Pure Pursuit, 후진 없음. 값은 [제안] | `nav2_field.py`, 아래 「경로 따라가기」 |
| 수동 실습 launch(`mapping_navigation`·`localization_navigation`) | 구현됨. 통로 정지·실행기·watchdog 없음 | 아래 「수동 실습용」 |
| 정적 프로파일 검사(`check_deployment`)·합성 리허설(`offline_demo`) | 구현됨. ROS·로봇 이동 없음 | `rescue_bringup/cli.py`, `offline_demo.py` |

## 지금 현장 실행 순서 (짧게)

준비물은 현장 파일 `config/field/search_area.field.json`(저장소), 배포 프로파일 v2.1(저장소 밖, `map_basis`·`home_positions` 포함)이다. 명령·로그는 아래 「현장 실행 순서: PC 역할별」에 있다.

1. 모든 PC 에서 Discovery 를 설정하고 `scripts/field/preflight_role.py --role <host id>` 결과가 모두 PASSED 인지 본다(0단계).
   - 조정기(`coordinator.launch.py`)는 로봇 launch(`robot.launch.py`)를 띄운 PC 와 다른 PC 에서 띄운다. 실행기는 같은 호스트의 조정기 시계를 거절한다(`LIVE_ROS expects two distinct PCs`, `rescue_robot/live_clock.py`).
2. 운영 PC(`operations` + `coordination`)에서 `operations.launch.py` → 관제 웹 → `coordinator.launch.py`(1단계).
3. TB-6 담당 PC(`compute_a`)에서 `tb6_slam.launch.py` → `passage_points.launch.py` → `overhead_passage.launch.py` → `robot.launch.py` → `map_freeze.launch.py`(TB-6 이 움직이기 전에)(2단계).
4. TB-5 담당 PC(`compute_b`)에서 `passage_points.launch.py` → `overhead_passage.launch.py` → `frozen_localization.launch.py robot_id:=robot_b` → `robot.launch.py`(2단계).
   - TB-6 담당 PC 와 TB-5 담당 PC 는 같은 PC 한 대여도 된다(재범 결정 2026-10-15, 조건은 [현장 실행 안내](../../docs/process/field-runbook.md) 「한눈에」). 조정기만 다른 PC 에 둔다.
5. 아무 PC 에서나 `ros2 daemon stop && ros2 daemon start` 뒤 `scripts/field/wiring_check.py` 로 배선을 본다. 모든 연결이 PASSED 여야 다음으로 간다(아래 「배선 점검」).
6. 두 로봇 arm → 웹캠 앞 흔들림(DISASTER) → TB-6 탐색이 저절로 시작된다(현장 파일 `search_start` AUTO, `search_mode` FRONTIER).
7. 인형 발견 → 1차 고정(R0+1), TB-5 자동 출동. 탐색 완료 → 2차 고정(R0+2), TB-6 은 `tb6_slam` 을 끄고 `frozen_localization.launch.py robot_id:=robot_a`(3단계).
8. TB-5 가 출구에 도착하면 끝난다(`completion_mode` EXIT_ARRIVAL).

**정지:** `bash scripts/field/stop_robot.sh /robot6`(TB-5 는 `/robot5`). 그래도 움직이면 사람이 로봇을 들어 올린다(3단계 「정지 방법」 상자).

## 패키지

정적 JSON 배포 검사(`check_deployment`)와 단일 프로세스의 합성 두 로봇 통합 리허설(`offline_demo`), 그리고 위 현장 launch 를 담는다. 정적 검사와 합성 리허설은 ROS 연결·로봇 이동을 하지 않는다.

프로파일 CLI는 이 패키지만 필요하다. `python -m rescue_bringup.offline_demo`는 공통 계약을 포함한 일곱 rescue 패키지 설치가 필요하다. Python metadata의 `offline` extra와 ROS package manifest에 의존성을 기록했다. 실행 명령·산출물·E3/E4 미실행 관문은 [통합 기록](../../docs/implementation/implementation-integration.md)과 [실기 안내](../../docs/process/field-runbook.md)를 따른다.

라이선스와 maintainer는 미결정이며 현재 패키지는 배포 릴리스가 아니다. runtime_enabled는 false를 유지한다.

## 수동 실습용 SLAM·Nav2 실행 구성

`mapping_navigation.launch.py`는 TurtleBot4의 SLAM + Nav2를, `localization_navigation.launch.py`는 저장 지도 기반 localization(AMCL) + Nav2를 실행한다. 두 모드 모두 RViz를 선택적으로 함께 실행한다. TurtleBot4 설치 패키지의 launch와 기본 파라미터를 재사용하며, 로봇 드라이버는 해당 로봇에서 이미 실행 중이어야 한다.

이 구성은 사람이 RViz에서 목표를 지정하는 단일 로봇 실습용이다. 기존 오프라인 CLI나 `runtime_enabled`를 활성화하지 않으며, 프로젝트 임무 명령·정지/권한·watchdog 및 `RobotPose`/`MapState` 연결은 포함하지 않는다. SLAM은 목표 없이 스스로 탐색하지 않는다. 복사해 둔 `rescue_perception` 내부 실습 파일은 이 launch에서 사용하지 않는다.

> **⚠ 이 경로(mapping_navigation·localization_navigation·explore_lite)에는 터널 감속과 진입 금지가 없다.** 둘은 `robot.launch.py`(Nav2 `collision_monitor` 감속, 비용 지도 `KeepoutFilter`)와 `passage_points.launch.py`·`overhead_passage.launch.py` 를 모두 띄울 때만 켜진다. 켜졌는지 확인:
> - `robot.launch.py` 로그에 `+ no depth stop + tunnel keepout` 이 있다.
> - `ros2 topic hz <ns>/passage_points` 가 나온다(깊이 점, 없으면 `collision_monitor` 가 1 s 뒤 멈춘다).
>
> **⚠ 정지:** 이 경로에는 실행기·watchdog 이 없다. 멈추려면 `bash scripts/field/stop_robot.sh <ns>`(Nav2 목표 전부 취소 + cmd_vel 0, 아래 3단계 정지 상자). explore_lite 를 Ctrl+C 로 끄면 노드가 끝나면서 목표 취소를 보내지만, 종료 중이라 그 요청이 Nav2 에 닿는지 보장되지 않는다[추정: upstream `Explore::~Explore()` → `stop()` → `async_cancel_all_goals()`]. Nav2 가 다른 터미널에서 계속 떠 있으면 **마지막 목표로 계속 간다.** explore 를 끈 뒤에는 반드시 정지 스크립트를 돌리거나 Nav2 launch 도 끈다.
>
> **SLAM 지도 새로 시작:** `mapping_navigation.launch.py`·`tb6_slam.launch.py` 는 `<ns>/map` 에 이미 발행자(앞의 SLAM)가 있으면 그 노드 이름을 내고 기동하지 않는다(옛 지도를 이어 쓰지 않음, OPS-184). 앞 SLAM 은 정지 스크립트가 보여 주는 PID 로 끈다. slam_toolbox 의 지도 리셋 서비스(Jazzy 판)는 **미확인**이다(이 PC 에 slam_toolbox 가 없어 확인 못 함). 실기에서 `ros2 service list --no-daemon | grep slam_toolbox` 로 본다. 확인 전에는 SLAM 노드를 끄고 다시 띄우는 것만 쓴다.

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
| `compute_a` | TB-6(robot_a, `/robot6`, SCOUT) 담당 PC | 탐색 중 SLAM(`tb6_slam.launch.py`) → 고정 뒤 `frozen_localization.launch.py robot_id:=robot_a`, `robot.launch.py robot_id:=robot_a`, 지도 고정 노드(`map_freeze.launch.py`), 탐지·대상 위치·재확인 서버(통합-인식) |
| `compute_b` | TB-5(robot_b, `/robot5`, SUPPORT) 담당 PC | `frozen_localization.launch.py robot_id:=robot_b`, `robot.launch.py robot_id:=robot_b` |

시험 중에는 `compute_a` 와 `compute_b` 를 한 PC 에서 함께 띄워도 된다(재범 검토 중, OPS-124). 로봇 노드는 모두 로봇 namespace(`/robot6`, `/robot5`) 안에 있어 이름이 겹치지 않는다. 지도 고정 노드는 한 번만 띄우고, 저장 위치는 `output_dir` 인자로 정한다. ROS 명령은 `--no-daemon` 으로 쓴다.

값은 세 곳에서만 온다. 첫째는 배포 프로파일 v2.1(저장소 밖)로, 로봇·역할·namespace·readiness·검색 영역·`map_basis`·`home_positions` 를 준다. 둘째는 현장 실행 파일 `config/field/search_area.field.json` 으로, 정책·로봇별 값을 준다. 셋째는 launch 인자로, 현장에서 재는 값이다. 이 launch 들은 arm·임무·이동 명령을 보내지 않는다. 로봇이 움직이는 단계는 사람이 지켜본다.

지금 기능별 상태는 맨 위 「현재 상태」 표를 본다(2026-10-13 표는 「기록」 절).

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
ping -c 20 <TB-5 Pi 주소>; ping -c 20 <TB-6 Pi 주소>      # 손실 0% 인지(로봇 담당 PC 도 두 Pi 모두)
iw dev <무선 장치> get power_save                         # PC 쪽 무선 절전(끌지는 재범이 정함)
# Discovery — 세미콜론 자리 번호 = 서버 ID. 지금 어느 구성인지 먼저 정하고 그 줄을 쓴다(재범 확인 2026-10-13 15:40).
# [시험 중] 권고안 A: 각 PC 가 로봇 Pi 서버에 직접(TB-5 Pi = ID 5, TB-6 Pi = ID 6)
#   모든 PC(compute_a·compute_b 포함): ';;;;;<TB-5 Pi>:11811;<TB-6 Pi>:11811'
#   고정 지도 흐름에서는 로봇 PC 도 두 서버를 다 적는다: compute_a 의 map_freeze 가 내는 /rescue/map/frozen 을
#   compute_b 의 TB-5 위치 추정·실행기가 받아야 한다. 서버 하나만 적으면 서로 못 찾는다[추정, 실측 전] (OPS-124).
#   (2026-10-13 낮까지는 compute_a: ';;;;;;<TB-6 Pi>:11811', compute_b: ';;;;;<TB-5 Pi>:11811' 였다)
# [최종] 전용 Discovery PC 1대(옛 중앙 서버 PC, 서버 ID 0): 모든 PC 가 '<전용 PC>:11811'(자리 0)
#   로봇이 전용 서버에 보이게 하는 방법은 결정 대기 — 로봇 Pi 에 외부 서버 추가(로봇 설정, 등급 4) 또는 서버끼리 연결.
#   정해지기 전에는 최종 구성으로 바꾸지 않는다(로봇 토픽이 안 보인다).
export ROS_DOMAIN_ID=<프로파일 domain_id>
export ROS_DISCOVERY_SERVER='<위 값, 프로파일 hosts[<host id>].network 와 같게>'
ros2 daemon stop && ros2 daemon start
ROS_SUPER_CLIENT=True ros2 topic list --no-daemon | grep -E '^/robot[56]/|^/rescue/map/' | cut -d/ -f2-3 | sort | uniq -c
python3 scripts/field/preflight_role.py --role <host id> --profile <profile.json> \
  --field config/field/search_area.field.json [--webcam-params <webcam_node.yaml>] [--map <map.yaml>]
```

현장 프로파일에는 `map_basis`(frame `map`, revision R0 = 탐색 중 revision; 1차 고정 지도는 R0+1, 2차는 R0+2)와 `home_positions`(TB-6 도킹 접근 지점, TB-5 출구; TB-6 지도에서 잼)가 있어야 한다. 프로파일은 저장소 밖이라 **`hosts[compute_a]`·`hosts[compute_b]` 의 `network.discovery_servers` 를 두 Pi 서버(자리 5·6)로 직접 고쳐야** 준비 점검 `env.ROS_DISCOVERY_SERVER` 가 PASSED 다. 노드는 super client 가 아니다(`ROS_SUPER_CLIENT=False`). 준비 점검이 모두 PASSED 여야 다음으로 간다.

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
#   --support-dispatch AUTO --search-start AUTO 를 더한다(현장 파일 coordinator 절 support_dispatch·search_start 와 같게.
#   빼면 관제 웹이 출동 승인·수색 시작 버튼을 켠다)

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

기준 지도 없이 시작한다. 두 로봇은 각자 바닥에 표시한 출발 자리에서 출발한다(TB-6 SLAM 원점 = TB-6 출발 자세, TB-5 는 TB-6 기준 `robots.robot_b.initial_pose_xy_yaw`). 2026-10-14 출발 간격은 1.95 m(가로)·0.05 m(세로)였다[측정, 재범 줄자, 둘 다 도킹 상태]. 언도킹은 기존 감독 절차로 한다.

```bash
F=$PWD/config/field/search_area.field.json
# compute_a: TB-6 SLAM(탐색 중만) → 로봇 launch → 지도 고정 노드
ros2 launch rescue_bringup tb6_slam.launch.py profile_path:=<profile.json> field_path:=$F robot_id:=robot_a
ros2 launch rescue_bringup passage_points.launch.py profile_path:=<profile.json> field_path:=$F robot_id:=robot_a
ros2 launch rescue_bringup overhead_passage.launch.py profile_path:=<profile.json> field_path:=$F robot_id:=robot_a
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_SUPER_CLIENT=False \
ros2 launch rescue_bringup robot.launch.py profile_path:=<profile.json> field_path:=$F robot_id:=robot_a
ros2 launch rescue_bringup map_freeze.launch.py profile_path:=<profile.json> field_path:=$F output_dir:=<회차마다 새 절대 경로>

# compute_b: TB-5 는 고정 지도를 기다린다(그 전에는 지도 없음, MapState UNKNOWN)
ros2 launch rescue_bringup passage_points.launch.py profile_path:=<profile.json> field_path:=$F robot_id:=robot_b
ros2 launch rescue_bringup overhead_passage.launch.py profile_path:=<profile.json> field_path:=$F robot_id:=robot_b
ros2 launch rescue_bringup frozen_localization.launch.py profile_path:=<profile.json> field_path:=$F \
  robot_id:=robot_b frozen_pose_wait_s:=10.0
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_SUPER_CLIENT=False \
ros2 launch rescue_bringup robot.launch.py profile_path:=<profile.json> field_path:=$F robot_id:=robot_b
```

**TB-6 SLAM 설정과 active 확인(실습 기록 #339 B3·B4, 2026-10-15):**
- `tb6_slam.launch.py` 는 TurtleBot4 `slam.yaml`(또는 `slam_params_file`)을 그대로 읽고 `minimum_travel_distance`·`minimum_travel_heading` 두 값만 0.1 에서 0.0 으로 바꿔 띄운다(`rescue_bringup/slam_field.py`). 기본값이면 0.1 m·0.1 rad 이상 움직인 스캔만 처리한다. 그래서 출발 자리에 서 있는 TB-6 은 `/robot6/pose` 를 내지 않았고 출발 자세가 잡히지 않았다. 0.0 으로 바꾸자 2 Hz 로 나왔다[측정 2026-10-15]. 로그: `+ {'minimum_travel_distance': 0.0, 'minimum_travel_heading': 0.0}`.
- 서 있어도 지도가 `map_update_interval` 1.0 s 마다 갱신된다. 조정기의 탐색 종료 조건 `exploration_empty_checks`(3)는 지도 갱신 횟수로 센다. 그래서 갈 곳이 없으면 몇 초 만에 탐색이 끝날 수 있다(같은 기록 B10, 조정 쪽 #338).
- `slam_active_wait_s`(기본 15 s [제안]) 뒤 `tb6_slam_active_check` 가 slam_toolbox lifecycle 상태를 읽고 한 줄을 남긴다. active 면 `SLAM_ACTIVE /robot6/slam_toolbox`, 아니면 `SLAM_NOT_ACTIVE /robot6/slam_toolbox state=<상태>` 다(ERROR).
- inactive 이고 `slam_activate_once:=true`(기본)이면 activate 를 한 번만 보낸다. 로그 `SLAM_ACTIVATE_SENT` 를 남기고 상태를 다시 읽는다. 백그라운드로 띄우면 launch 가 「inactive 가 됐다」 이벤트를 놓쳐 inactive 에 머문 일이 있었다[추정]. 그 밖의 상태(unconfigured 등)나 답이 없을 때는 로그만 남긴다. 이때는 `ros2 lifecycle get /robot6/slam_toolbox` 로 보고 손으로 다룬다.

**머리 위 터널(두 로봇): 깊이 정지 없음, 비전 판정으로 진입 금지(재범 결정 2026-10-16 01:4x 「터널 지나갈 때 비전으로 터널 높이 측정해서 지나가는 로직 짰잖아」, 01:5x 「A로 진행」, 등급 4):**
- 예전의 깊이 정지 `PassageStop`(깊이 점 0.05~0.46 m, 앞 0.4~1.0 m → 정지, 회전까지 0)은 뺐다. 벽·도크 앞에서 로봇을 세워 교착이 났다(C1 #363, 4회차).
- 터널은 `overhead_passage` 판정으로 다룬다. `rescue_robot` 의 `passage_keepout`(robot_a 의 `robot.launch.py` 가 하나만 띄움)이 두 로봇의 판정 목록을 모은다. 자리마다 가장 최근(`last_seen_ns`) 항목이 BLOCKED 이면, 반지름 `passage.keepout.radius_m` 0.3 m 원을 진입 금지 지도(`/rescue/passage_keepout/mask`, map 좌표)에 칠한다. UNKNOWN 은 구조물이 상자 가까운 끝(`passage.overhead.near_end_m` 0.05 m 안)에 왔는데도 바닥을 못 본 경우(`NO_FLOOR_NEAR`)만 칠한다. PASSABLE 과 그 밖의 UNKNOWN 은 칠하지 않는다. 값은 모두 [제안]이다.
- 두 로봇의 전역·지역 비용 지도에 Nav2 `KeepoutFilter`(`tunnel_keepout`, `/rescue/passage_keepout/filter_info`)를 더한다(`nav2_field.nav2_with_tunnel_keepout`, `robot.launch.py` 만). 경로가 칠한 자리를 지나지 않는다. 라이다가 덮개 아래를 지나도 지워지지 않는다.
- 사람이 막힘을 푸는 것(통과 승인)은 판정 노드 안에 있고(`KeepoutCore.approve`), ROS 서비스는 다음 PR 이다(새 인터페이스를 SDD JSON 에 먼저 넣어야 함).
- 실행기의 `PASSAGE_BLOCKED` 알림은 `PassageStop` 다각형을 보던 것이라 이제 켜지지 않는다(현장 파일 `task.live.passage.polygon_name` 은 남아 있음).
- **라이다 아래 낮은 물체(재범 결정 2026-10-16 03:1x 「1 피하게 하자」, D, 등급 4):** 도크(0.1 m)나 바닥의 인형처럼 라이다 평면(0.23 m) 아래에 있는 것을 경로가 피한다.
  - `overhead_passage` 가 깊이 점을 본 바닥 기준 높이로 바꿔 `<ns>/rescue/low_obstacle_points` 로 낸다. 바닥 점과 라이다 평면 아래 점만 담는다. 바닥이 안 보이면 내지 않는다.
  - 두 비용 지도에 깊이 전용 `depth_low_layer`(ObstacleLayer)를 inflation 앞에 둔다. 관측원은 두 개다. `depth_low` 는 바닥 위 `passage.min_height_m` 0.05 ~ `passage.overhead.lidar_plane_m` 0.23 m 를 칠하고, `depth_floor` 는 바닥 점까지 raytrace 로 지운다.
  - 라이다 scan 은 이 층의 관측원이 아니라 이 층을 지우지 못한다. 치운 인형 자리는 그 너머 바닥이 보이면 지워진다.
  - 거리 값은 같은 비용 지도의 scan 값(칠하기 2.5 m, 지우기 3.0 m)이다. `observation_persistence`·`expected_update_rate` 는 Nav2 기본값 0.0 이다.
  - **깊이가 끊기면 이 층은 칠한 것을 그대로 남긴다(안전 쪽).** 새 점이 없으면 지우지도 않는다. 카메라가 조용해도 비용 지도가 오래됨으로 멈추지는 않는다.
- 로그: `+ no depth stop + tunnel keepout /rescue/passage_keepout/filter_info`, `PASSAGE_KEEPOUT <n> place(s) kept out`.
- **감속(OPS-189):** 같은 깊이 점 중 라이다 평면 위(`passage.overhead.lidar_plane_m` 0.23 m [측정]~`passage.overhead.top_m` 1.0 m)에 있는 것이 앞 `passage.slow.front_m` [0.4, 1.2] m 안에 `passage.min_points` 개 이상이면 `PassageSlow` 가 속도를 `passage.slow.ratio` 0.3 배로 줄인다([제안]). 라이다가 못 보는 터널 천장 앞에서 높이 판정(약 1.5 s)이 나올 시간을 주려는 것이다. 0.46 m 이하는 `PassageStop` 이 그대로 멈춘다. 로그: `+ passage slow x0.3 0.23..1.0 m`. **벽 앞에서도 감속한다**: collision_monitor 는 벽과 터널을 구분하지 못해, 라이다 위 높이까지 올라온 벽이 앞 1.2 m 안에 있으면 똑같이 0.3 배로 줄인다. 그래서 감속 사유 코드는 두지 않는다(「측정 중」이 아님, 웹 「판정 중」은 터널 노드 UNKNOWN 으로 본다, OPS-193). 실기에서 벽 앞 감속 때문에 탐색이 너무 느리면 현장 파일 `robot` 절의 `passage.slow.ratio`(크게) 또는 `passage.slow.front_m`(먼 끝을 가깝게)을 고친다.
- 깊이 점은 `passage_points.launch.py` 가 만든다. 깊이 영상 이름은 현장 실행 파일 `robots.<id>.passage.*` 에 있다. 이 노드가 없으면 깊이 점이 오지 않으므로 `collision_monitor` 가 1 s 뒤 로봇을 세운다. 깊이 영상은 준비 신호에도 들어가서, 끊기면 NOT_READY 다. 준비 신호도 깊이 점과 같은 transport 로 받는다(compressedDepth 이면 `<depth_topic>/compressedDepth`). 그래서 원본 깊이를 Wi-Fi 로 따로 끌어오지 않는다.
- **지도에 터널 표시(OPS-185/186):** `overhead_passage.launch.py` 가 같은 깊이 점, 같은 상자와 높이(`passage.*`)로 머리 위 구조물을 판정한다. 결과는 `/rescue/robots/<id>/overhead_passages`(JSON, contracts C-04b)로 나간다. 추가 값은 `passage.overhead.*` 에 있다. 판정을 그린 카메라 영상 `<ns>/rescue/passage_view/compressed` 는 `passage.view.*` 와 로봇별 `passage.view_image_topic`·`passage.view_camera_info_topic`(각 Pi 의 RGB 미리보기와 그 camera_info. 전체 RGB 는 인형 탐지 입력이라 구독을 늘리지 않음, OPS-198. 둘 다 지우면 그 로봇 영상 끔)으로 정한다. 라이다 높이 `lidar_plane_m` 0.23 m 는 TB-5 줄자 실측이고 TB-6 은 같다고 본 추정이다. 판정 규칙은 `src/rescue_perception/README.md` 「지도의 머리 위 구조물」 절에 있다.
- 멈춘 동안 실행기 feedback 이 `PASSAGE_BLOCKED` 다. 그 뒤 Nav2 가 포기하면 결과도 `PASSAGE_BLOCKED` 다(#219).
- **`robots.<id>.passage.*` 가 없는 로봇은 `robot.launch.py` 가 띄우지 않는다**(OPS-137 A). 두 로봇 모두 OAK-D 깊이 `/<ns>/oakd/stereo/image_raw` + `camera_info`, 전송 `compressedDepth` 다.
  - TB-5 측정(2026-10-13 20:19 ~ 20:21, OPS-165 ~ 167): raw 5.1 ~ 5.5 Hz(최대 간격 0.449 s), compressedDepth 6.9 ~ 9.2 Hz(최대 간격 0.302 s), camera_info 10.0 Hz. 그래서 compressedDepth. `collision_monitor` 의 source_timeout 1 s 안이다.
  - TB-5 깊이 `camera_info` 의 `header.frame_id` 는 `oakd_rgb_camera_optical_frame`(깊이가 RGB 에 정렬, TB-6 과 같음)이고 `base_link` 에서 TF 가 있다(x -0.060, z 0.244 m, 2026-10-13 확인). 확인 명령: `ros2 run tf2_ros tf2_echo base_link oakd_rgb_camera_optical_frame` (`/tf`·`/tf_static` 을 `<ns>/tf`·`<ns>/tf_static` 으로 remap). TF 가 없으면 깊이 점이 `base_link` 로 바뀌지 않아 통로 정지가 동작하지 않는다.
- 필요한 것: 로봇 PC 의 `depth_image_proc`. TB-5 PC 에는 설치돼 있다(`ros-jazzy-depth-image-proc` 5.0.13, 2026-10-13 확인). 시험 중처럼 두 로봇을 그 PC 한 대에서 돌리면 TB-6 도 설치된 것이고, TB-6 을 다른 PC 에서 돌리면 그 PC 는 확인 전이다.
- **복구 동작에 후진 없음(OPS-207, 재범 승인 2026-10-14 「오케이 그럼 빼자」):** 시나리오 ⑧ 「후진 없음」(원형 로봇이라 제자리 회전 + 전진), TB-5 뒤에는 따라오는 사람이 있다. `robot.launch.py`·`mapping_navigation.launch.py`·`localization_navigation.launch.py` 가 TurtleBot4 `nav2.yaml` 에서 `backup` 플러그인을 빼고, `bt_navigator` 를 이 패키지의 BT(`rescue_bringup/behavior_trees/*_no_backup.xml`, Nav2 Jazzy 기본 BT 에서 `BackUp` 한 줄만 뺀 것)로 바꾼다. 복구는 비용 지도 지우기 → 제자리 회전(1.57 rad) → 5 s 대기 순서다. 플러그인만 빼면 기본 BT 의 `BackUp` 노드가 없는 action server 를 기다리다 BT 를 못 올리므로 둘을 같이 바꾼다[Nav2 jazzy 소스 `bt_action_node.hpp`]. 로그: `+ no BackUp recovery`. `drive_on_heading`·`assisted_teleop` 은 남아 있다(음수 속도면 뒤로 갈 수 있지만, 우리 BT 와 코드는 부르지 않음). 이 경로 밖에서 Nav2 를 띄우는 `scripts/field/search-area/nav2_watchdog.launch.py`, `rescue_perception` 탐색 실습 `exploration/nav2.yaml` 에는 아직 후진이 있다. (이전 기록 2026-10-13: 복구 동작은 turtlebot4 기본 그대로, 작전 답.)
- **경로 따라가기: DWB 대신 Rotation Shim + Regulated Pure Pursuit(재범 요청 2026-10-14 저녁, 실습 중 발견):**
  - 무엇이 있었나. TB-6 을 RViz 로 ㄷ자 막다른 곳 안에서 그 뒷벽 너머 목표로 보내면, ㄷ 안에서 좌우로 고개만 돌리다 갇혔다. 전역 경로(초록)는 ㄷ 입구로 나가서 돌아가는 맞는 경로였다.
  - 왜 그런가[추정]. TurtleBot4 `nav2.yaml` 의 DWB 는 목표 쪽 점수(GoalDist·GoalAlign 각 24)와 경로 쪽 점수(PathAlign·PathDist 각 32)를 같이 본다. 그래서 목표가 벽 너머면 두 쪽이 다툰다. 좌우로 흔들리다 진행 검사(10 s 에 0.5 m)에 걸리고, 복구는 회전과 대기뿐이다(후진 없음).
  - 무엇을 바꿨나. `robot.launch.py`·`mapping_navigation.launch.py`·`localization_navigation.launch.py` 가 후진 제거와 같은 자리(`rescue_bringup/nav2_field.py`)에서 `controller_server.FollowPath` 를 통째로 바꾼다. RPP 는 경로 자체를 따라가고, 경로가 뒤로 시작하면 shim 이 먼저 제자리에서 돈다. TurtleBot4 원본 `nav2.yaml` 과 `config/robots` 기준 파일은 고치지 않았다.
  - 값(모두 [제안], 근거는 바꾸기 전 DWB 값). `desired_linear_vel` 0.2 m/s(DWB 최고 0.26 이하), lookahead 0.3~0.6 m(기본 0.4), `angular_dist_threshold` 0.785 rad, 회전 1.0 rad/s(DWB `max_vel_theta`), 각가속 3.2(DWB `acc_lim_theta`), `use_collision_detection` true, `allow_reversing` false(OPS-207), `use_rotate_to_heading` false(shim 과 같이 쓸 때 끔, Nav2 문서). `controller_server` 공통 값(`enable_stamped_cmd_vel`, 진행 검사, 목표 판정 0.25 m·0.25 rad)은 그대로다.
  - 현장 PC 확인. `ros2 pkg prefix nav2_rotation_shim_controller nav2_regulated_pure_pursuit_controller` 가 두 경로를 내야 한다(Jazzy navigation2 에 들어 있음, 현장 PC 설치 여부 미확인). `preflight_role.py` 가 `command` 역할 PC 에서 두 패키지를 본다. 로봇 Pi 에는 아무것도 설치하지 않는다.
  - 로그: `+ no BackUp recovery + FollowPath Rotation Shim/RPP`.
  - 확정 절차. 로봇 움직임이 바뀌는 설정이다. 재범 님 감독 아래 같은 ㄷ 자리에서 바꾸기 전(DWB, 이 변경 이전 main)과 후(Rotation Shim + RPP)를 같은 목표로 비교하고, 그 회차 기록 뒤에 확정한다(실기 NOT_RUN).

성공 로그:
- `TB-6 SLAM robot_a ns=/robot6`
- `passage points /robot6/oakd/stereo/image_raw (compressedDepth) + ... -> /robot6/passage_points`
- `robot <id> ns=<ns> ... + no depth stop + tunnel keepout ... map_mode=SLAM_THEN_FROZEN`(또는 `FROZEN_MAP_AMCL`) `map_sha256=(decided by the freeze)`
- `[<ns 이름>.rescue_command_watchdog]: git_sha=...`, `installed task runtime; input_mode=LIVE_ROS; explicit arm required`
- `rescue_map_freeze`: `git_sha=... parameters=...`, 그 뒤 `START_POSE captured robot_a map=(x, y, yaw)` 한 번(TB-6 출발 자세)
- TB-6 `MAP_STATE robot_a ... rev=<R0> quality=VALID`(SLAM 위치가 잡힌 뒤). TB-5 는 고정 전 `quality=UNKNOWN` 이 정상이다.

**좌표 기준: TB-6 출발 자세(OPS-141).** SLAM 지도 원점은 Create3 odom 원점이다. 바닥 표시 출발 자리와 다를 수 있다[추정, 실기 확인 항목: SLAM 시작 직후 `/robot6/pose` 가 (0,0,0) 인지 본다. 맞으면 아래 변환이 항등일 뿐 틀리지 않는다].
- 그래서 미리 재는 좌표는 모두 TB-6 출발 자세 기준으로 적는다: 순찰 지점, TB-6 도킹 접근 지점, TB-5 출발값.
- `map_freeze` 의 TB-6 위치도 실행기처럼 TF `map→base_link`(`/robot6/tf`, 0.2 s 마다)이고, 공분산은 마지막 `/robot6/pose` 값이다(30 s 넘으면 세지 않음, OPS-179). 그래서 서 있는 TB-6 도 고정할 수 있다. 공분산이 오래되면 고정은 `MAP_FREEZE_FAILED SLAM pose covariance is too old` 다.
- `map_freeze` 는 SLAM 시작 뒤 처음 받은 유효한 TB-6 위치를 회차마다 한 번 `output_dir/start_pose.json` 에 남기고, `/rescue/map/start_pose/robot_a`(TRANSIENT_LOCAL)로 낸다.
- 그래서 **`map_freeze` 는 TB-6 이 움직이기 전에 띄운다.** 노드를 다시 띄우면 저장한 값을 다시 낸다(새로 잡지 않음). 새 회차는 새 `output_dir` 로 띄운다.
- TB-5 위치 추정 launch 는 이 출발 자세로 `robots.robot_b.initial_pose_xy_yaw` 를 map 좌표로 바꾼다. 순찰·도킹 접근 지점은 조정기가 바꾼다(통합-조정).

`occupancy_sha256` 는 주지 않는다(고정이 해시를 정함). 탐색 중 TB-6 위치는 TF `map→base_link`(slam_toolbox 의 `map→odom` + Create3 의 `odom→base_link`, `/robot6/tf`)를 실행기 poll 주기로 읽어 만든다(OPS-177). slam_toolbox 는 처리한 스캔마다만 `/robot6/pose` 를 내고 일정 거리를 움직여야 스캔을 처리하므로, 서 있는 TB-6 은 새 `/robot6/pose` 가 없다[추정]. 공분산은 마지막 `/robot6/pose` 값을 쓰고, 그것이 30 s(`robots.robot_a.task.saved_map.slam_covariance_max_age_ns`, [제안])보다 오래되면 위치는 UNKNOWN 이다. 실기 확인: `ros2 topic hz /robot6/pose`(서 있을 때 끊기는지), `ros2 run tf2_ros tf2_echo map base_link --ros-args -r /tf:=/robot6/tf -r /tf_static:=/robot6/tf_static`. 안 나오면 TB-6 MapState 가 UNKNOWN 이고 수색 목표를 거절한다(다른 값으로 대체하지 않음).

**고정 지도 단계의 위치(두 로봇):** AMCL 은 서 있는 로봇에 `amcl_pose` 를 내지 않는다. 그래서 고정 지도 단계에서도 실행기가 TF `map → base_link` 를 poll 주기마다 읽고, 공분산은 마지막 `amcl_pose` 값을 쓴다. 그 뒤 로봇이 AMCL 갱신 문턱(0.25 m / 0.2 rad)보다 많이 움직였는데 새 `amcl_pose` 가 없으면 VALID 가 아니다. 문턱은 `robot.launch.py` 가 `localization_params_file`(비우면 TurtleBot4 `localization.yaml`)에서 읽는다. `frozen_localization.launch.py` 에 다른 파일을 줬다면 `robot.launch.py` 에도 같은 파일을 준다. 로그: `frozen pose TF <ns>/tf with AMCL covariance while moved < 0.25 m / 0.2 rad`.

**TB-5 출발값 재기(시험 회차, 한 번):** 고정 지도가 나온 뒤에 한다.
1. TB-5 를 출발 자리에 둔다.
2. 출발값 없이는 위 TB-5 launch 가 거절하므로, 이 회차만 turtlebot4 기본 위치 추정을 쓴다. `ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot5 map:=<고정 지도 .yaml>` 을 띄우고, RViz `2D Pose Estimate` 로 TB-6 지도 위 자리를 찍는다. 고정 노드가 `output_dir` 에 남긴 `.yaml` 을 쓴다.
3. AMCL 이 맞춘 뒤 `python3 scripts/field/capture_initial_pose.py /robot5/amcl_pose map /rescue/map/start_pose/robot_a` 를 돌린다. stdout 한 줄(TB-6 출발 자세 기준 값)을 현장 실행 파일 `robots.robot_b` 에 넣는다(코드에 박지 않음).
4. 출구(TB-5)·TB-6 도킹 접근 지점(프로파일 `home_positions`)도 같은 지도에서 같은 도구로 잰다.

compute_a 에서는 TB-6 탐지와 대상 위치도 띄운다(통합-인식). 탐지는 `amr_perception.launch.py` 이고, 인자는 그 파일 머리말과 [rescue_perception README](../../src/rescue_perception/README.md) 에 있다. `sensor_host_clock_*` 는 위 Pi 시계 값과 같다. 대상 위치는 `target_locator`(같은 README 「지도 좌표 TargetLocation」)다. 재확인 서버 `/rescue/robots/robot_a/observe_target` 은 `amr_target` 이 연다(`recheck_action_name` 등 인자, #200). 지금 현장 파일은 `completion_mode` EXIT_ARRIVAL 이라 조정기가 TB-6 재확인을 쓰지 않는다.

저장 지도로 하는 시험(로봇 없는 시험, #132)은 현장 실행 파일을 조정기 `map_mode` `SAVED_MAP` + 두 로봇 `SAVED_MAP_AMCL` 로 바꾸고, 예전처럼 `turtlebot4_navigation localization.launch.py` → `occupancy_sha256:=$(python3 scripts/field/map_occupancy_sha256.py <ns>/map map)` 로 띄운다.

### 배선 점검 — 모든 launch 뒤, 로봇이 움직이기 전 (2026-10-14)

PC 한 대의 환경은 `preflight_role.py` 가 보고, 연결이 실제로 이어졌는지는 이 도구가 한 번에 본다.

1. 0단계 `preflight_role.py` 가 모두 PASSED 인지 본다.
2. 점검할 PC 에서 `ros2 daemon stop && ros2 daemon start` 를 한다. 데몬이 예전 그래프를 들고 있으면 꺼진 노드가 보일 수 있다.
3. 1·2단계 launch 를 모두 띄운 뒤 아래를 돌린다. Discovery 설정은 0단계와 같게 둔다.

```bash
python3 scripts/field/wiring_check.py --profile <profile.json> \
  --field config/field/search_area.field.json --json <결과.json> > /dev/null
#   한 로봇만: --robot robot_b       2차 고정 뒤 TB-6: --slam-phase FROZEN
```

| 보는 것 | FAILED 가 되는 경우 |
| --- | --- |
| 연결마다 발행자(서비스·액션은 서버) | 0개, 2개 이상, 기대와 다른 노드 |
| 기대한 구독 노드(서비스·액션은 클라이언트) | 하나라도 없음 |
| 타입·QoS(reliability, durability) | 발행자와 구독자가 맞지 않음 |
| 같은 이름 노드 | 둘 이상(예: `/robot6/slam_toolbox` 둘이면 lifecycle 요청에 둘이 답한다, 2026-10-14 [추정]) |

- 기대 연결표는 코드에서 만든다. 프로파일과 현장 파일을 `field_launch` 함수(`robot_launch_values` 등)에 넣고, 조정 노드 상수와 launch 파일의 노드 이름을 쓴다(`wiring_check.py` `expected_links`). `docs/design/sdd-node-connections-v0.1.csv` 는 이름이 지금과 달라(`/amr1` 등) 쓰지 않는다.
- 로봇 Pi 쪽 토픽(odom, scan, 카메라 등)은 노드 이름을 보지 않고, 발행자가 하나인지만 본다.
- 같은 이름 노드 줄은 그 이름을 쓰는 프로그램이 두 곳에서 떠 있다는 뜻이다. 우리 노드면 한쪽 launch 를 끈다. 우리 노드가 아니면(예: 두 PC 에서 띄운 `/rviz2`) 한쪽을 끄거나 `--ros-args -r __node:=<다른 이름>` 으로 이름을 바꾼다.
- 우리가 끌 수 없는 로봇 Pi 기본 노드(`wiring_check.py` `PI_DEFAULT_NODES`, 지금은 `teleop_twist_joy_node` 하나)는 FAILED 대신 PASSED 에 `!` 줄(JSON `notes`)을 붙인다. 이 조이스틱 노드는 TurtleBot4 Pi 기본 launch 가 띄우고 `<ns>/cmd_vel` 에 직접 쓴다. 그래서 enable 버튼을 누르는 동안에는 watchdog 을 거치지 않고 로봇을 움직인다. 회차 중에는 조이스틱을 치워 둔다(2026-10-15 17:10 두 Pi 모두 이 줄이 FAILED 였다). 다른 로봇의 `cmd_vel` 이나 다른 토픽에 쓰면 그대로 FAILED 다.
- `search_mode` 가 FRONTIER 이면 조정 노드도 TB-6 지도를 구독한다. 이 이름은 조정 노드 상수 `SCOUT_MAP_TOPIC`(`/robot6/map`)으로 고정돼 있다. 프로파일의 TB-6 namespace 가 다르면 이 줄이 발행자 없음으로 FAILED 다.
- 그래프를 읽다가 오류가 나면(rclpy 시작 실패, RMW·Discovery 오류) JSON `graph_error` 에 적고 모든 줄을 NOT_RUN 으로 둔다(종료 코드 3).
- 기대를 모르거나 그래프를 못 읽으면 NOT_RUN 이다(PASSED 아님). 종료 코드는 0 = 모두 PASSED, 2 = FAILED 있음, 3 = NOT_RUN 있음. 표는 stderr, JSON 은 stdout 으로 나간다.
- 그래프만 읽는다. 구독·발행·서비스 호출을 하지 않고 로봇을 움직이지 않는다. 이 프로세스만 `ROS_SUPER_CLIENT=True` 로 띄워 Discovery Server 아래 전체를 본다.
- 이 도구가 보지 않는 것은 메시지가 실제로 오는 간격(`link_check.py`), TF 내용, 표에 없는 연결(판정 영상 중계, 웹캠 감시 서비스, 재확인 action 등)이다.
- 시험은 `tests/test_field_wiring_check.py`(ROS 없이)와 `tests/ros/e3_wiring_check_probe.py`(가짜 노드 그래프, 도메인 193, LOCALHOST)다. 실제 PC 4대에서는 아직 돌리지 않았다(NOT_RUN).

### 한 PC 배선 리허설 (로봇 없이, 2026-10-14)

이 저장소의 현장 노드를 실제 namespace(`/robot6`, `/robot5`)로 한 PC 에 모두 띄우고 `wiring_check.py` 로 본다. 현장 PC 에서 처음 터질 배선 오류를 미리 찾으려는 것이다. 첫 실행에서 실행기가 시작하자마자 꺼지는 문제를 찾았다(#298). 기록은 [리허설 기록](../../docs/evidence/rehearsal-2026-10-14-wiring-one-pc.md)에 있다.

1. ROS 작업공간을 만든다. `rescue_*` 패키지에 더해, Nav2 가 없는 PC 는 `nav2_msgs`·`irobot_create_msgs`·`geographic_msgs` 소스(jazzy)를 같이 빌드한다(`colcon build --packages-up-to ...`, sudo 없음).
2. 아래를 돌린다. 인자는 작업공간 install, 결과 폴더, TB-6 지도 단계(SEARCHING 또는 FROZEN), 포트, 도메인 순서다.

```bash
bash scripts/field/wiring-rehearsal/run.sh <install> <결과 폴더> SEARCHING 11888 193
```

- `run.sh` 는 시작할 때 `ROS_DISCOVERY_SERVER` 를 지우고 `127.0.0.1:<포트>` 만 넣는다. 현장 서버 주소가 남으면 실제 로봇 그래프에 붙기 때문이다(Discovery Server 는 도메인으로 갈리지 않음). 도메인이 10 미만이거나 포트가 쓰이고 있으면 시작하지 않는다.
- 띄우는 것은 셋이다. `fast-discovery-server` 하나, `wiring-rehearsal/rehearsal.launch.py`(이 저장소 노드), `wiring-rehearsal/fake_graph.py`(Nav2·SLAM·AMCL·깊이 컨테이너·로봇 Pi·웹캠·인식 노드의 가짜 끝점)다.
- 가짜는 끝점만 만든다. 메시지를 내지 않고 action 목표는 모두 거절한다. 그래서 목표가 Nav2 에 닿아도 아무것도 움직이지 않는다(로봇도 이 그래프에 없음).
- 끝나면 각 프로세스 그룹에 INT → TERM → KILL 을 보내고, Discovery Server 는 그 PID 로 끈다(`pkill -f` 를 쓰지 않음). 남은 것이 있으면 `WARNING: rehearsal processes left` 를 낸다.
- 진짜 노드끼리의 연결은 QoS 까지 본다. 가짜와의 연결은 이름·개수만 의미가 있다(가짜는 진짜 타입을 따라 하고 가장 약한 QoS 를 요청함).
- 실행기는 같은 호스트의 조정기를 거절한다(`LIVE_ROS expects two distinct PCs`, `rescue_robot/live_clock.py`). 그래서 리허설에서만 두 `task_input` 프로세스에 `wiring-rehearsal/host_shim` 을 넣어 시계 측정의 host id 를 따로 준다. chrony 값과 오차는 그대로 둔다. 현장 PC 에서는 쓰지 않는다.
- 배선 점검 뒤에 `REHEARSAL_PROBE` 로 흐름 하나를 더 돌릴 수 있다. 가짜 SLAM·AMCL·Pi 데이터는 `wiring-rehearsal/feed.py` 가 낸다(합성값, 현장 값 아님).
  - `REHEARSAL_PROBE=freeze`(`freeze_probe.py`): 지도 고정 서비스를 직접 불러 1차 고정 → TB-5 relay·MapState R1 을 본다. 단계를 FROZEN 으로 주면 2차 고정 → TB-6 SLAM 끄기 → TB-6 relay·두 MapState R2 까지 본다.
    - TB-6 TF 는 기본으로 2026-10-14 현장 실패 때 잰 값(#291 `measured_tb6_tf`)을 쓴다. `REHEARSAL_TF=flat` 이면 정확한 평면이다.
  - `REHEARSAL_PROBE=mission`(`mission_probe.py`, SEARCHING): 현장 3단계처럼 `refresh_state` → 두 로봇 `task_arm` 을 한다. 이어서 가짜 웹캠 DISASTER_SIGN → TB-6 FRONTIER 목표 → 실행기 admit → 가짜 `bt_navigator` 거절까지 보고, TB-6 이 FAILED 뒤 `STOP_LATCHED` 로 남는지(자동 재준비 없음) 확인한다.

### 3단계 — 출동 준비와 실행 (operations + coordination PC 에서, 사람이 지켜봄)

> **정지 방법 (먼저 읽기)**
> 1. `bash scripts/field/stop_robot.sh /robot6` (TB-5 는 `/robot5`). 실행기 `task_stop` → Nav2 목표 전부 취소(`navigate_to_pose`·`navigate_through_poses`·`follow_waypoints`) → `cmd_vel` 0 을 보내고, 단계마다 `[보냄]`/`[없음]`/`[실패]` 한 줄을 낸다. 끝에 아직 떠 있는 `<ns>` 노드와 이 PC 의 SLAM·Nav2·explore 프로세스 PID 를 보여 준다.
> 2. 스크립트가 없거나 실패하면 수동으로: `ros2 service call <ns>/rescue_command_adapter/task_stop std_srvs/srv/Trigger {}`, `ros2 service call <ns>/navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal {}`, 남은 프로세스는 PID 로 `kill -INT <PID>`(이름으로 넓게 잡아 죽이지 않는다).
> 3. 그래도 움직이면 **사람이 로봇을 들어 올린다.**
>
> `robot.launch.py` 를 Ctrl+C 하면 Nav2 도 같이 꺼지고, watchdog 이 꺼지기 전에 `<ns>/cmd_vel` 에 0 을 3번 낸다(로그 `watchdog stopped: 3 zero commands`).

```bash
# 각 로봇: 정지 상태에서 AMCL 갱신 한 번 → 조정기 상태 재발행 → 실행기 arm(#132 start_search.py 순서)
ros2 service call <ns>/request_nomotion_update std_srvs/srv/Empty {}
ros2 service call /mission_coordinator/refresh_state std_srvs/srv/Trigger {}
ros2 service call <ns>/rescue_command_adapter/task_arm std_srvs/srv/Trigger {}      # success=True
# (TB-6 은 탐색 중 SLAM 이라 AMCL 이 없다: request_nomotion_update 는 TB-5 만, 고정 R0+2 뒤에는 TB-6 도)

# TB-6 탐색 중 위치(읽기 전용, 조정기 frontier 탐색이 받는 값). Discovery 서버 아래에서는 topic list 대신 hz/echo 로 본다.
ros2 topic hz /robot6/pose        # slam_toolbox 위치(공분산 출처). 세워 둔 채로 끊기는지 기록한다(OPS-177 판단 확인용)
ros2 topic echo --once /rescue/robots/robot_a/pose   # mission_pose_status: VALID, frame_ref.map_revision: <R0>
```

TB-6 탐색 중 RobotPose 는 TF `map→base_link`(`/robot6/tf`)를 실행기 poll 주기(0.5 s)로 읽어 `/rescue/robots/robot_a/pose` 로 낸다(`live_sources.attach_slam_tf_pose` → `receive_pose(source='slam')` → `live_pose.LivePosePublisher.on_pose`, OPS-177). 유효 기간은 `task.live.pose_valid_ns` 2 s 다. 공분산은 마지막 `/robot6/pose` 값이고, 그것이 30 s 보다 오래되면 위치는 `UNKNOWN` 이다. 그래서 `/robot6/pose` 가 서 있는 동안 끊겨도(위 hz 줄로 기록한다) 위치는 이어지지만, 30 s 넘게 끊기면 UNKNOWN 이 된다. `VALID` 가 되려면 `/robot6/map` 을 받았고, TF 두 edge(`map→odom`, `odom→base_link`)가 오며, 공분산이 한도(`max_position_variance_m2` 0.25) 안이고, 시계가 SYNCED 여야 한다.

1. 수색 목표는 고르지 않는다. 현장 파일 `search_mode` FRONTIER 이면 조정기가 TB-6 실시간 지도의 가장 가까운 frontier 로 목표를 낸다(#247). 탐색 중 목표는 revision R0 다. RViz 목표 고르기(`/rescue/search_goal`)는 `search_mode` AREAS 일 때만 쓴다. 조정기 기동 로그의 `awaiting RViz search goal` 은 FRONTIER 에서도 나온다.
2. TB-6 과 TB-5 를 모두 arm 한다. TB-5 는 자동 출동이라 대상이 나오기 전에 arm 돼 있어야 한다. 실행기는 arm(latch ARMED)일 때만 READY 를 내고(`task_status.py`), 조정기는 READY·최신 상태인 SUPPORT 에만 task 를 낸다(통합-조정 확인 2026-10-13). arm 하지 않으면 조정기는 기다리기만 한다. PR #196 머지 뒤에는 SystemState `reason=SUPPORT_NOT_READY` 로 보이고, 관제 웹에는 「TB-5 준비 안 됨 — arm 필요」로 보인다(PR #185 머지 뒤).
3. 웹캠 앞 흔들림 → 관제 화면 모드 DISASTER → 탐색이 저절로 시작된다(`search_start` AUTO, 조정기 기록 `SEARCH_AUTO_STARTED on DISASTER`, OPS-136). 이때 관제 웹 START_SEARCH 는 `NOT_AUTHORIZED` 로 거절된다.
4. **1차 고정(인형 발견) — TB-5 출동, TB-6 은 계속 탐색.** 대상이 확정·저장되면 조정기가 `/rescue/map/freeze` 를 부른다(통합-조정 두 번 고정). TB-6 은 멈추지 않아도 된다. compute_a 로그 `MAP_FROZEN ... rev=<R0+1> final=False tb6 keeps SLAM`.
   - TB-5: `MAP_REVISION_ADOPTED robot_b RNone -> R<R0+1>` → `MAP_RELAYED /rescue/map/frozen R<R0+1> -> /robot5/map` → `MAP_STATE robot_b ... rev=<R0+1> quality=VALID` 이면 출동할 수 있다. AMCL 이 자리를 못 잡으면(공분산 초과) `quality=UNKNOWN` 이고 출동하지 않는다.
   - TB-6: 아무것도 바뀌지 않는다. `MAP_STATE robot_a ... rev=<R0>` 그대로이고 `/robot6/map` 은 SLAM 지도다(중계는 TB-6 MapState 가 R0 라 복사하지 않음).
   - 고정이 실패하면(`MAP_FREEZE_FAILED`) 아무것도 바뀌지 않는다. TB-5 도 출동하지 않는다.
5. **2차 고정(탐색 완료) — TB-6 전환, TB-5 재정합.** 탐색이 끝나면(조정기 기록 `EXPLORATION_COMPLETE`) 조정기가 TB-6 정지를 확인한 뒤 다시 `/rescue/map/freeze` 를 부른다. compute_a 로그 `MAP_FROZEN ... rev=<R0+2> final=True tb6 amcl start=(...)`. 세 번째 요청은 거절된다.
   1. TB-6: `MAP_REVISION_ADOPTED robot_a RNone -> R<R0+2>` 를 확인하고 `tb6_slam.launch.py` 터미널에서 Ctrl-C 를 누른다. `ros2 topic info /robot6/map --no-daemon` 의 발행자 수가 0 이 될 때까지 기다린다.
   2. 아래 한 줄을 띄운다. 고정 위치에서 AMCL 이 시작한다(`/rescue/map/frozen_pose/robot_a`, 2차에만 나옴, 10 s 안에 없으면 launch 실패):
      `ros2 launch rescue_bringup frozen_localization.launch.py profile_path:=<profile.json> field_path:=$F robot_id:=robot_a frozen_pose_wait_s:=10.0`
   3. 로그 `MAP_RELAYED /rescue/map/frozen R<R0+2> -> /robot6/map` 를 확인한다. SLAM 이 아직 떠 있으면 `MAP_RELAY_BLOCKED` 가 나오고 지도를 넘기지 않는다.
   4. TB-5: 인형 앞에서 정지·작업 없음이면 바로 `MAP_REVISION_ADOPTED robot_b R<R0+1> -> R<R0+2>` → `MAP_RELAYED ... R<R0+2> -> /robot5/map` → `MAP_REALIGN robot_b ...`(직전 자기 위치로 AMCL 재시작). 움직이는 중이면 `MAP_REVISION_DEFERRED robot_b ... not stopped`(또는 `task active`)가 나오고, 멈추면 자동으로 받는다. 그동안 새 목표는 `map revision pending` 으로 거절된다.
   5. 두 로봇 모두 `MAP_STATE ... rev=<R0+2> quality=VALID` 가 나오는지 확인한다. `snapshot_id` 가 두 로봇에서 같아야 한다.
6. 정지: `ros2 service call <ns>/rescue_command_adapter/task_stop std_srvs/srv/Trigger {}` 로 하고, 실제로 멈췄는지 눈으로 확인한다. watchdog 이 세운 경우(`watchdog STOP_LATCHED`)는 rescue_robot README 의 리셋 절을 본다.
7. 다시 하려면 조정기·로봇 launch 를 다시 띄우고 새 목표를 고른다. 고정은 회차마다 두 번까지다(R0+1 인형 발견, R0+2 탐색 완료, OPS-154). 새 회차는 `tb6_slam` 부터 다시 띄우고, 고정 노드는 **새 `output_dir`** 로 띄운다. 같은 회차에서 고정 노드만 다시 띄울 때는 같은 `output_dir` 로 띄우면 그 회차의 출발 자세와 고정 기록(`frozen-r<revision>-*.json`)을 순서대로 다시 낸다(`record_path` 인자는 없어졌다). 기록이 한 회차로 맞지 않으면(같은 revision 두 개, R0+1 없는 R0+2) `MAP_FREEZE_RESTORE_FAILED` 로 기동하지 않는다.

### 참고

- 정렬 유효 구간은 #132 처럼 launch 시각 기준 `now - alignment.backdate_s` ~ `now + alignment.window_s` 로 선언한다(관측값이 아님).
- ROS 파라미터로 빈 배열을 넘길 수 없다. 그래서 `route_segment_passage_free:='[]'` 와 빈 `camera_topics` 는 launch 가 거절하고, 표시 개수는 구간 수와 같아야 한다(#163).
- 문자열 값은 `ParameterValue(str)` 로 넘긴다(`ros_parameters.typed`).
- 시험: `tests/test_bringup_field_launch.py`(#132 현장 스크립트와 같은 파라미터인지).

## 실기 NOT_RUN (2026-10-14)

코드와 로봇 없는 시험은 있지만 실제 로봇에서 확인하지 않은 것이다. 실기 회차에 하나씩 지운다.

- 현장 회차 전체(위 0 ~ 3단계를 실제 로봇으로)와 두 번 고정 흐름.
- TB-6 `/robot6/tf` 에 `map→odom`·`odom→base_link` 가 오는지, 서 있는 TB-6 에서 `/robot6/pose` 가 끊기는지(OPS-177 [추정]).
- `map_guard` 가 Discovery 서버 아래에서 앞 SLAM 을 찾는지(#256).
- Create3 `cmd_vel` 형식(`TwistStamped`)과 watchdog 종료 0 이 로봇에 닿는지.
- Jazzy collision_monitor 의 `slowdown`·`sources_names` 동작(#260), 깊이 점이 실제로 `PassageSlow` 를 켜는지, 터널 BLOCKED 가 진입 금지 지도로 들어가 경로가 피하는지(`KeepoutFilter`).
- TB-6 라이다 높이 0.23 m(TB-5 줄자 실측을 같은 모델로 본 [추정]).
- **(해결 2026-10-16: 터널 판정 높이는 이제 본 바닥 기준, rescue_perception README 「높이 기준」)** **#254 측정(2026-10-13, TB-6 정지 관측 한 번)이 높이 띠에 주는 영향(OPS-203):** TB-6 앞 바닥 점이 `base_link` 기준 z ≈ −0.22 ~ −0.19 m 였다. 정지 띠(0.05 ~ 0.46 m)·감속 띠(0.23 ~ 1.0 m)·터널 판정(바닥 ≤ 0.05 m, 여유 높이 0.46 m)은 모두 `base_link` 높이로 잰다. 그래서 바닥이 정말 그 높이라면 바닥 위로는 약 0.2 m 씩 올라간다[추정]. 정지 띠는 바닥 위 약 0.25 ~ 0.66 m 다. 바닥 위 약 0.05 ~ 0.25 m 의 낮은 물체는 깊이 정지 띠에서 빠진다(라이다 평면 아래면 라이다도 못 봄). 터널 판정은 바닥 높이가 아니라 `base_link` 높이로 여유를 잰다. 바닥·`base_link`·카메라 TF 관계를 줄자로 확인하기 전에는 값을 바꾸지 않는다.
- Nav2 후진 복구. #267 로 우리 launch 세 곳은 후진이 없다. 그래도 `scripts/field/search-area/nav2_watchdog.launch.py` 와 `rescue_perception` 의 `exploration/nav2.yaml` 에는 후진 복구가 남는다. **이 두 경로는 현장에서 쓰지 않는다**(작전 결정 2026-10-14, OPS-215).
- Nav2 경로 따라가기(Rotation Shim + RPP). ㄷ자 막다른 곳에서 바꾸기 전·후 비교, 좁은 통로·터널 앞 감속(`PassageSlow`)과 함께 도는지, `desired_linear_vel` 0.2 m/s 로 탐색 시간이 얼마나 늘어나는지를 본다. 위 두 경로(`nav2_watchdog.launch.py`, `exploration/nav2.yaml`)는 DWB 그대로다.
- slam_toolbox 지도 리셋 서비스(Jazzy 판)가 있는지(위 「SLAM 지도 새로 시작」).
- **정상 완료 뒤 자동으로 다시 준비(OPS-231, 재범 4등급 승인 2026-10-14).** 이동이 정상으로 끝나고 정지가 확인되면 실행기가 스스로 다시 준비한다(`rescue_robot/executor.py:rearm_after_completion`). 그래서 3단계의 arm 은 처음 한 번이면 된다. 실기에서 두 가지를 본다. TB-6 이 탐색 지점마다 멈추지 않고 이어 가는지, TB-5 가 사람 앞에 선 뒤 15 s(`support_exit_wait_s`)가 지나 출구로 출발하는지. 실패·취소·정지 버튼 뒤에는 지금처럼 `task_arm` 이 필요하다. 다시 준비가 거절되면 `RobotStatus.stop_reason` 에 `REARM_REFUSED_<이유>` 가 보인다. 이슈 #233 3번.

## 기록

아래는 당시 설명을 날짜와 함께 그대로 옮긴 것이다. 지금 상태는 맨 위 「현재 상태」 표와 「지금 현장 실행 순서」를 본다.

### 2026-10-13: 「현장 실행 순서」의 기능 표

| 기능 | 상태 |
| --- | --- |
| 운영 launch(db_writer·보고 저장·웹캠·영상 중계), 조정기·로봇 launch, 지도 해시, 준비 점검 | 있음(#144·#152·#156·#158) |
| TB-5 자동 접근(APPROACH_TARGET), TB-6 재관측(REOBSERVE_TARGET)+재확인 관측, 로봇 위치·지도 상태 발행 | 있음(#166·#179·#176). 재확인 서버: 판정·action 서버 코드는 있음(#165), 실행 노드와 센서 입력(프레임·스캔) 연결은 통합-인식 다음 PR 이다. 그 전에는 서버가 없어 재관측 결과가 `has_recheck=false` 다 |
| 복귀(RETURN_HOME) | 있음(#192). 자동 도킹은 없다(도킹은 사람이 감독) |
| TB-5 로봇 launch | 있음(현장 실행 파일 `robots.robot_b`: 접근·복귀만, Pi 시계 150 ms) |
| 지도: 기준 지도 없이 TB-6 SLAM → 대상 확정 때 고정 → 두 로봇 AMCL(scope Q-29 ⑦) | 있음(#211 실행기, #213 고정 노드, #214 조정기, 이 launch 들). 현장 실행 파일: 조정기 `map_mode` `SLAM_FREEZE`, `robots.robot_a.task.saved_map.mode` `SLAM_THEN_FROZEN`, `robots.robot_b...` `FROZEN_MAP_AMCL`. 짝이 어긋나면 launch 가 거절한다. TB-5 출발값 `robots.robot_b.initial_pose_xy_yaw` 는 `[0.0, -1.86, 0.0]` 이다(줄자, 2026-10-15). 이 값이 없는 현장 파일이면 TB-5 위치 추정 launch 가 거절하고 준비 점검 `field.initial_pose.robot_b` 가 FAILED 다 |
| watchdog 현장 값 | #132 임시값(#94 미확정). 영구 정지와 리셋은 [rescue_robot README](../rescue_robot/README.md) |

### 2026-10-13: 바뀐 문장

- 1단계 관제 웹 주석: `PR #185 머지 뒤: --support-dispatch AUTO 를 더한다(없으면 관제 웹이 TB-5 출동 승인 버튼을 켠다. 조정기 support_dispatch 와 같게)`
- 2단계: 재확인 서버 `/rescue/robots/robot_a/observe_target` 은 실행 노드가 생긴 뒤(통합-인식 다음 PR) 띄운다. 코드(#165)만으로는 띄울 실행 파일이 없다.
- 3단계 1: RViz(2D Goal Pose)로 `/rescue/search_goal` 에 TB-6 수색 목표를 고른다. 조정기 로그 `SEARCH_GOAL_SELECTED`. 탐색 중 목표는 revision R0 다.
- 3단계 3: 웹캠 앞 흔들림 → 관제 화면 모드 DISASTER → 관제 웹 출동 버튼(START_SEARCH).
- 3단계 5: 「조정기가 TB-6 정지를 확인한 뒤 다시 `/rescue/map/freeze` 를 부른다」(탐색 완료 `EXPLORATION_COMPLETE` 조건은 #247·OPS-174 로 더해짐).

### 2026-10-08: 패키지 첫 문장

정적 JSON 배포 검사와 단일 프로세스의 합성 두 로봇 통합 리허설을 제공한다. 두 명령 모두 ROS 연결·로봇 이동은 수행하지 않는다.

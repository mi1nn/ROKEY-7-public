---
title: 현장 실행 안내 — 실기 날 이 문서 하나로 돌린다
status: current-not-run   # 순서와 명령은 main 5d9304a 기준. 처음부터 끝까지 실기로 돌린 적은 아직 없다. 아래 「기록」 절은 2026-10-08 판 전문
updated: 2026-10-15
---

# 현장 실행 안내

## 한눈에 (2026-10-15, main `5d9304a`)

이 문서가 답하는 질문은 하나다. **실기 날 누가, 어느 PC 에서, 무엇을 어떤 순서로 띄우고, 무엇을 보면 다음으로 넘어가나?**

| 단계 | 누가 | 어디서 | 무엇 | 넘어가는 조건 |
| --- | --- | --- | --- | --- |
| 0 | 모두 | — | 「정지 방법」을 읽는다 | 정지 스크립트 위치를 안다 |
| 1 | 각 PC 담당 | 모든 PC | 코드 맞추기, Discovery 설정, 준비 점검 | `preflight_role.py` 모두 PASSED |
| 2 | 로봇 감독 | 로봇 담당 PC | 후진 없는 BT 대조, 로봇 Pi 확인 | diff 가 머리 주석뿐, Pi 값이 맞음 |
| 3 | 운영 PC 담당 | 운영 PC | 운영 launch, 관제 웹, 조정기 | 성공 로그 |
| 4 | TB-6 담당 PC 담당 | TB-6 담당 PC | SLAM, 깊이 점, 터널 판정, 로봇 launch, 지도 고정 | `START_POSE captured robot_a` |
| 5 | TB-5 담당 PC 담당 | TB-5 담당 PC | 깊이 점, 터널 판정, 고정 지도 위치 추정, 로봇 launch | 성공 로그 |
| 6 | 아무 PC 한 대 | 아무 PC | 배선 점검 `wiring_check.py` | 종료 코드 0 |
| 7 | 로봇 감독 | 운영 PC | 두 로봇 arm → 웹캠 흔들림 → 1차 고정 → 2차 고정 → 출구 도착 | 단계별 로그 |
| 8 | 로봇 감독 | — | 끝내기, 도킹, 기록 남기기 | 기록 파일 |

- 로봇은 한 번에 한 사람만 움직인다. 이 문서에서는 그 사람을 「로봇 감독」이라 부른다. 언도킹·도킹·이동 전후에 팀에 알린다([작업 규칙](../../AGENTS.md) 6절).
- PC 는 역할 이름으로 부른다. 운영 PC(`operations` + `coordination`), TB-6 담당 PC(`compute_a`), TB-5 담당 PC(`compute_b`)다. 로봇은 TB-6(`robot_a`, `/robot6`), TB-5(`robot_b`, `/robot5`)다([용어집](../glossary.md)).
- 조정기는 로봇 launch 를 띄운 PC 와 **다른 PC** 에서 띄운다. 같은 PC 면 실행기가 거절한다(`LIVE_ROS expects two distinct PCs`).
- **로봇 담당 PC 한 대로 두 로봇을 맡아도 된다**(재범 결정 2026-10-15). TB-6 담당 PC 와 TB-5 담당 PC 가 같은 PC 이면 4단계와 5단계를 그 PC 의 **다른 터미널**에서 띄운다. 조건은 셋이다.
  1. 조정기(운영 PC)는 그 PC 가 아니어야 한다. 실행기는 조정기 시계의 `/etc/machine-id` 가 자기와 같으면 거절한다(`live_clock.py`, `clock_measurement.py`).
  2. 배포 프로파일의 `compute_a` 와 `compute_b` 는 그대로 둔다(역할 이름이고 PC 이름이 아니다). 두 역할의 `network` 값은 그 PC 의 Discovery 설정과 같아야 한다. 1단계 준비 점검은 역할마다 한 번씩(`--role compute_a`, `--role compute_b`) 돌린다.
  3. 두 로봇의 노드는 모두 네임스페이스(`/robot6`, `/robot5`) 아래에 있어 이름이 겹치지 않는다. 겹침은 6단계 배선 점검이 잡는다(같은 노드 이름 두 개 = FAILED).
  - 한 PC 에 SLAM, Nav2 두 벌, 깊이 점 두 벌이 함께 돈다. CPU·메모리와 Wi-Fi 수신량은 아직 재지 않았다 [미확인]. 회차 중 `top` 으로 한 번 기록한다.
- 값은 현장 파일 [`config/field/search_area.field.json`](../../config/field/search_area.field.json) 과 배포 프로파일 v2.1(저장소 밖, `map_basis`·`home_positions` 포함)에 있다. 이 문서는 값을 다시 적지 않는다.
- 임무 흐름은 [scope 「지금 시나리오」](../requirements/scope.md), 노드 설명은 [로봇](../implementation/implementation-robot.md)·[지도](../implementation/implementation-mapping.md)·[통합](../implementation/implementation-integration.md) 구현 기록에 있다.

## 정지 방법 (먼저 읽기)

1. `bash scripts/field/stop_robot.sh /robot6`(TB-5 는 `/robot5`). 실행기 `task_stop` → Nav2 목표 모두 취소 → `cmd_vel` 0 을 보낸다. 단계마다 `[보냄]`·`[없음]`·`[실패]` 한 줄을 찍고, 끝에 아직 떠 있는 노드와 이 PC 의 SLAM·Nav2 프로세스 PID 를 보여 준다.
2. 스크립트가 없거나 실패하면 손으로 부른다.
   ```bash
   ros2 service call <ns>/rescue_command_adapter/task_stop std_srvs/srv/Trigger {}
   ros2 service call <ns>/navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal {}
   kill -INT <PID>      # 남은 프로세스는 PID 로. 이름으로 넓게 잡아 끄지 않는다(pkill -f 금지)
   ```
3. 그래도 움직이면 **사람이 로봇을 들어 올린다.**

- `robot.launch.py` 를 Ctrl+C 하면 속도 감시가 꺼지기 전에 `cmd_vel` 0 을 세 번 보낸다(로그 `watchdog stopped: 3 zero commands on <ns>/cmd_vel`).
- 속도 감시가 세운 경우 로그에 `watchdog STOP_LATCHED reason=<이유>` 가 있다. 원인을 보고 로봇이 멈춘 것을 눈으로 확인한 뒤에만 `ros2 service call <ns>/rescue_command_watchdog/reset std_srvs/srv/Trigger {}`(`<ns>` 는 `/robot6` 또는 `/robot5`) 로 푼다. 이유별로 볼 것은 [rescue_robot README](../../src/rescue_robot/README.md) 「watchdog 영구 정지」 절에 있다.

## 1단계 — 모든 PC: 코드, Discovery, 준비 점검

각 PC 담당이 새 터미널에서 한다. 로봇 없는 시험 터미널(도메인 10 이상, LOCALHOST)과 섞지 않는다.

**launch 는 하나마다 새 터미널에서 띄운다.** launch 가 터미널을 붙잡기 때문이다. 새 터미널을 열 때마다 저장소 루트에서 아래 「터미널 준비」 네 줄을 먼저 넣는다. `export` 값과 `F` 는 그 터미널에만 적용된다. 빠뜨리면 `field_path:=` 가 빈 값이 되어 launch 가 오류로 멈춘다.

```bash
# 터미널 준비 (새 터미널마다, 저장소 루트에서)
source /opt/ros/jazzy/setup.bash && source <작업공간>/install/setup.bash
export ROS_DOMAIN_ID=<프로파일 domain_id>
export ROS_DISCOVERY_SERVER='<아래 「Discovery 값」>'
F=$PWD/config/field/search_area.field.json
```

처음 한 번은 아래도 한다.

```bash
git rev-parse HEAD; git status --short       # 모든 PC 가 같은 SHA, 작업 폴더 깨끗함
source /opt/ros/jazzy/setup.bash && source <작업공간>/install/setup.bash
ping -c 20 <TB-5 Pi 주소>; ping -c 20 <TB-6 Pi 주소>   # 손실 0%. 모든 PC 가 같은 로봇망 SSID
export ROS_DOMAIN_ID=<프로파일 domain_id>
export ROS_DISCOVERY_SERVER='<아래 「Discovery 값」>'
ros2 daemon stop && ros2 daemon start
python3 scripts/field/preflight_role.py --role <host id> --profile <profile.json> \
  --field config/field/search_area.field.json [--webcam-params <webcam_node.yaml>] [--map <map.yaml>]
```

- **Discovery 값.** 2026-10-15 부터 전용 Discovery 서버 PC 하나를 쓴다(재범 결정 2026-10-15, [Discovery 전용 PC 운영 안내](discovery-host.md)). 모든 PC(로봇 담당 PC, 운영 PC 포함)가 그 서버 하나만 적는다. 서버 ID 는 0 이라 세미콜론 없이 적는다. 로봇 Pi 의 Discovery 설정은 바꾸지 않는다.
  ```text
  <전용 Discovery PC 주소>:11811
  ```
  권고안 A(두 로봇 Pi 서버 `;;;;;<TB-5 Pi 주소>:11811;<TB-6 Pi 주소>:11811`)는 전용 PC 가 꺼졌을 때 로봇 쪽을 직접 조회하는 데만 쓴다([Discovery 전용 PC 운영 안내](discovery-host.md) 5절).
- `--role` 은 프로파일의 host id 다. 운영 PC 는 프로파일에 operations 호스트와 coordination 호스트로 따로 올라 있으므로, 두 host id 로 각각 한 번씩 돌린다. 둘 다 PASSED 여야 한다.
- 프로파일의 `hosts[<host id>].network.discovery_servers` 가 위 값과 같아야 준비 점검의 `env.ROS_DISCOVERY_SERVER` 가 PASSED 다. 전용 서버 하나면 모든 host 가 `[{"server_id": 0, "address": "<전용 Discovery PC 주소>", "port": 11811}]` 다. 프로파일이 아직 권고안 A 값이면 셸이 아니라 프로파일을 고친다(바꾸기 전 백업, 전후 sha256 기록).
- **현장 프로파일은 손으로 고치지 않고 만든다.** 모든 PC 에서 같은 main 으로 `python3 scripts/field/make_field_profile.py --out <새 폴더> --discovery <전용 Discovery PC 주소>:11811 --domain <도메인>` 을 돌리고, 찍힌 sha256 세 줄(프로파일, 두 로봇 파일)이 PC 마다 같은지 본다. 저장소 예시에서 namespace(TB-6 `/robot6`, TB-5 `/robot5`), 모든 host 의 network(전용 서버 하나), `map_basis.frame_id`(`map`), `deployment_id` 만 바꾼다. 이미 있는 폴더에는 쓰지 않는다. 2026-10-15 까지는 예시를 PC 마다 손으로 고친 뒤 sha256 을 맞췄다([실습 기록](../evidence/field-2026-10-15-run1015.md)).
  - `home_positions` 는 1차 고정 지도에서 잰 값만 `--home robot_a=x,y,yaw`(TB-5 는 `robot_b=`)로 넣어 새 폴더로 다시 만든다. 없으면 TB-6 은 복귀하지 않는다(`NO_HOME_POSITION`).
- 노드는 super client 가 아니다(`ROS_SUPER_CLIENT=False`). 배선 점검 도구만 super client 로 띄운다.
- **ROS 로 조회할 때는 `--spin-time 10` 을 붙인다.** Discovery 서버를 거친 목록은 늦게 온다. `ros2 topic list --no-daemon` 만 쓰면 기본 대기가 짧아 로봇 토픽이 0개로 보인다. 앞에 `timeout 30` 을 붙여도 대기는 늘지 않는다(2026-10-15 실기, [운영 PC 기록](../evidence/field-2026-10-15-operations-host.md) 3절).
- **PC 의 망이 바뀌면 그 PC 의 ROS 노드를 모두 다시 띄운다.** Wi-Fi 에서 유선으로 바꾸거나 Wi-Fi 가 끊긴 뒤에는, 이미 떠 있던 노드가 구독은 맺어도 데이터와 서비스 응답을 받지 못했다. 같은 일이 운영 PC 에서 두 번 있었다(2026-10-15, [운영 PC 기록](../evidence/field-2026-10-15-operations-host.md) 4절).
- `preflight_role.py` 가 FAILED 를 내면 고치기 전에는 다음 단계로 가지 않는다. NOT_RUN 을 통과로 적지 않는다.

## 2단계 — BT 대조와 로봇 Pi (로봇 감독)

**후진 없는 BT 대조(#267, 로봇 담당 PC 마다 Nav2 를 띄우기 전).** 우리 BT(행동 트리) 사본은 Nav2 `jazzy` 브랜치에서 후진 한 줄만 뺀 것이다. 설치된 Nav2 의 기본 BT 와 다르면 BT 로드가 실패할 수 있다 [추정].

```bash
BT=$(python3 -c 'from rescue_bringup.nav2_recovery import bt_dir; print(bt_dir())') && ls "$BT"
for f in navigate_to_pose navigate_through_poses; do
  diff <(grep -v BackUp /opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/${f}_w_replanning_and_recovery.xml | grep -v '^\s*$') \
       <(grep -v '^\s*$' "$BT/${f}_no_backup.xml" | sed '/^<!--/,/-->/d')
done
```

`BT` 는 launch 가 bt_navigator 에 넘기는 폴더다(`nav2_recovery.bt_dir()`). 그래서 일반 빌드와 `--symlink-install` 빌드에서 모두 맞는 파일을 본다. `--symlink-install` 빌드에는 `<prefix>/lib/python3*/site-packages/rescue_bringup` 이 없다(2026-10-15 실기). `ls` 가 `_no_backup.xml` 두 개를 보여야 한다.

머리 주석 말고 다른 차이가 없어야 한다. 차이가 있으면 Nav2 를 띄우지 말고 팀에 알린다.

**로봇 Pi 두 대**(근거 [Pi 설정 기록](../evidence/field-2026-10-13-pi-settings.md)).

| 확인 | 명령 | 맞는 값 |
| --- | --- | --- |
| 무선 절전 | `iw dev wlan0 get power_save` | `Power save: off`. 재부팅하면 켜지므로 켤 때마다 `sudo iw dev wlan0 set power_save off`. 2026-10-15 에는 이 줄을 빠뜨려 TB-6 이 `on` 이었고 ping 이 1초 넘게 튀었다([실습 기록](../evidence/field-2026-10-15-run1015.md) B5) |
| Create3 연결 | 로봇 담당 PC 에서 `timeout 10 ros2 topic echo --once <ns>/dock_status` | 10초 안에 온다. Pi 의 turtlebot4 서비스를 다시 시작한 뒤에는 꼭 다시 본다. 안 오면 Create3 통신 LED 가 꺼져 있고 언도킹·도킹이 「Waiting for an action server」에서 멈춘다(2026-10-15 두 로봇 모두, 같은 기록 B6). 이때는 Create3 를 다시 시작한다(재범 승인) |
| 자동 업데이트 | `systemctl is-enabled apt-daily.timer apt-daily-upgrade.timer unattended-upgrades` | 셋 다 `disabled` |
| Pi 시계 | `chronyc tracking` | 계산한 오차가 현장 파일 `robots.<id>.task.live.sensor_host_clock_error_bound_ns` 안. 넘으면 값을 바꾸고 기록한다 |

- 로봇 Pi 에는 아무것도 설치하지 않는다. 로봇 설정을 바꾸려면 재범 승인이 필요하다(등급 4).
- **라이다가 멈추면**(`ros2 topic hz <ns>/scan` 이 비고 SLAM 로그에 `rplidar_link` 메시지가 버려짐) 재범 승인을 받고 그 로봇 Pi 에 ssh 로 들어가 `sudo systemctl restart turtlebot4` 를 한다. PC 에서 치면 `Unit turtlebot4.service not found` 다. 1~2분 뒤 scan 이 오는지 보고, 바로 위 표의 「Create3 연결」 줄로 `dock_status` 가 오는지도 본다. 이 재시작 뒤 Create3 연결이 끊긴 일이 있다(2026-10-15 TB-6). 2026-10-15 TB-6 에서 두 번 있었다([운영 PC 기록](../evidence/field-2026-10-15-operations-host.md) 8절).
- 카메라는 도킹 중 꺼져 있다. 카메라 확인은 언도킹 뒤에 두 로봇 모두 한다. RGB 만 보지 말고 깊이도 본다: `ros2 topic hz <ns>/oakd/rgb/image_raw/compressed` 와 `ros2 topic hz <ns>/oakd/stereo/camera_info`. 깊이 camera_info 가 0 이면 깊이만 멈춘 것이다. 터널 정지와 준비 점검이 막히므로 `ros2 service call <ns>/oakd/stop_camera std_srvs/srv/Trigger {}` 뒤 `start_camera` 로 카메라만 다시 켠다(재범 승인, 2026-10-15 TB-6 에서 이것으로 복구, 같은 기록 B8).

## 3단계 — 운영 PC

```bash
ros2 launch rescue_bringup operations.launch.py profile_path:=<profile.json> \
  db_path:=<site_state.sqlite3> api_port:=<api 포트> command_timeout_s:=3.0 \
  report_db_path:=<reports.sqlite3> envelope_schema_version:=rescue-envelope/1 report_publisher_id:=<id> \
  webcam_params_file:=<webcam_node.yaml> video_port:=<영상 포트> video_bind_host:=<브라우저가 닿는 주소> video_stale_s:=2.0 \
  map_topic:=/robot6/map map_dir:=<실시간 지도 폴더> \
  robot_namespaces:=robot_a=/robot6,robot_b=/robot5 robot_watch_path:=<직접 관측 파일>
# 로그의 "console --camera cam_fixed=http://..." 값을 관제 웹 --camera 에 그대로 쓴다

python3 -m rescue_console --accounts <계정 DB> --site-state <site_state.sqlite3> \
  --operations http://127.0.0.1:<api 포트> --camera <launch 로그의 값> --camera-stale-s 2.0 \
  --robot-name robot_a=TB-6 --robot-name robot_b=TB-5 --port 8080 \
  --live-map <실시간 지도 폴더>/live_map.yaml --live-map-stale-s 15 \
  --robot-watch <직접 관측 파일> --robot-watch-stale-s 5 --robot-watch-map-robot robot_a \
  --support-dispatch AUTO --search-start AUTO

ros2 launch rescue_bringup coordinator.launch.py profile_path:=<profile.json> \
  field_path:=$PWD/config/field/search_area.field.json \
  route_segment_goals:='[0.0, 0.0, 0.0]' observe_goal:='[0.0, 0.0, 0.0]' route_segment_passage_free:='[false]' \
  camera_watch_roi:='<현장에서 잰 값, 예: [0.1, 0.2, 0.6, 0.9]>'
```

- 관제 웹의 `--support-dispatch`·`--search-start` 는 현장 파일 `coordinator` 절과 같게 준다. 빼면 관제 웹이 출동 승인·수색 시작 버튼을 켠다.
- 조정기의 `route_segment_goals`·`observe_goal` 은 자리표시다. `camera_watch_roi` 는 JSON 숫자 배열이어야 한다. `'[x0, y0, x1, y1]'` 글자를 그대로 쓰면 launch 가 exit 1 이다.
- 성공 로그는 `rescue_db_writer git=... api=http://127.0.0.1:<port>`, `rescue_report_node ...`, `webcam_node git=...`, 조정기 `live task source enabled`·`central live clock: SYNCED` 다. `central live clock: waiting for clock measurement` 가 먼저 한 번 나오는 것은 정상이다.
- 계정 DB 와 비밀키 준비는 [rescue_console README](../../src/rescue_console/README.md)에 있다.
- **지도와 지도 고정 전 TB-6 위치는 운영 launch 의 두 묶음 인자로 본다.** `map_topic`·`map_dir` 는 TB-6 SLAM 지도를 받아 관제 웹 `--live-map` 과 리플레이에 넘긴다. `robot_namespaces`·`robot_watch_path` 는 두 로봇의 TF·배터리·도킹을 구독만 하는 직접 관측(`robot_watch`)이다. 실행기가 보내는 로봇 위치는 지도를 고정하고 AMCL 이 맞춘 뒤에만 채워진다(`live_pose.py`). 그래서 지도 고정 전 TB-6 위치는 직접 관측으로만 지도에 그려진다. TB-5 는 1차 고정 전에는 위치가 없고 「위치 미확정」이 정상이다(2026-10-15 실기).

## 4단계 — TB-6 담당 PC

로봇 감독이 TB-6 을 바닥에 표시한 출발 자리에 두고 언도킹한다. **지도 고정 노드는 언도킹이 끝나 TB-6 이 서 있을 때, 주행하기 전에 띄운다.** 처음 받은 TB-6 위치가 이 회차의 출발 자세가 되기 때문이다.

- **도킹한 채로 띄우지 않는다.** 언도킹은 뒤로 물러난 뒤 제자리에서 돈다. 도킹한 채로 띄우면 출발 자세가 도크를 보는 방향으로 잡힌다. TB-5 출발값 `robots.robot_b.initial_pose_xy_yaw` 는 두 로봇이 언도킹한 상태에서 쟀으므로(2026-10-15, [rescue_bringup README](../../src/rescue_bringup/README.md)), 이 순서가 바뀌면 TB-5 의 좌우 부호가 뒤집힌다.

> [!WARNING]
> 원문 미확인 (전달: 가제보 실습 자리, 2026-10-15): 시뮬레이션에서 언도킹은 약 191° 돌았다. 지도 고정을 언도킹 전에 띄우자 시뮬레이션 사본의 TB-5 출발값이 (0, +1.82)로, 언도킹 뒤에 띄우자 (−0.353, −1.785, −0.003)으로 계산됐다(현장 값 (0, −1.86, 0)). 실물 언도킹 회전 각도는 [미확인].

아래 launch 는 하나마다 새 터미널에서 띄운다. 터미널마다 1단계 「터미널 준비」 네 줄을 먼저 넣는다(`F` 포함).

```bash
F=$PWD/config/field/search_area.field.json   # 1단계 「터미널 준비」에 있음. 터미널마다 다시
ros2 launch rescue_bringup tb6_slam.launch.py profile_path:=<profile.json> field_path:=$F robot_id:=robot_a
ros2 launch rescue_bringup passage_points.launch.py profile_path:=<profile.json> field_path:=$F robot_id:=robot_a
ros2 launch rescue_bringup overhead_passage.launch.py profile_path:=<profile.json> field_path:=$F robot_id:=robot_a
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_SUPER_CLIENT=False \
ros2 launch rescue_bringup robot.launch.py profile_path:=<profile.json> field_path:=$F robot_id:=robot_a
ros2 launch rescue_bringup map_freeze.launch.py profile_path:=<profile.json> field_path:=$F output_dir:=<회차마다 새 절대 경로>
ros2 launch rescue_bringup amr_perception.launch.py <인자는 rescue_perception README>
```

성공 로그를 차례로 본다.

- `TB-6 SLAM robot_a ns=/robot6`. 그 뒤 `ros2 lifecycle get /robot6/slam_toolbox` 가 `active [3]` 인지 본다. `inactive [2]` 에 머물면 `ros2 lifecycle set /robot6/slam_toolbox activate` 로 켠다(2026-10-15, 같은 기록 B4)
- `passage points /robot6/oakd/stereo/image_raw (compressedDepth) + ... -> /robot6/passage_points`
- `robot robot_a ns=/robot6 ... + passage stop 0.05..0.46 m ... + no BackUp recovery ... map_mode=SLAM_THEN_FROZEN map_sha256=(decided by the freeze)`
- `installed task runtime; input_mode=LIVE_ROS; explicit arm required`
- `rescue_map_freeze` 의 `git_sha=...`, 그 뒤 `START_POSE captured robot_a map=(x, y, yaw)` 한 번
- `MAP_STATE robot_a ... rev=<R0> quality=VALID`

실행기를 조정기보다 늦게 띄우면 실행기 로그에 `coordinator session rejected: CLOCK_UNCERTAIN: coordinator SystemState is future or stale` 와 `input session blocked: NOT_AUTHORIZED: waiting for fresh configured coordinator SystemState` 가 나온다. 조정기는 상태가 바뀔 때만 SystemState 를 보내서, 늦게 뜬 실행기는 오래된 상태만 받기 때문이다. 운영 PC 에서 `ros2 service call /mission_coordinator/refresh_state std_srvs/srv/Trigger {}` 를 부르면 풀린다(답 `SYSTEM_STATE_REPUBLISHED`). 5단계 TB-5 실행기도 같다. 2026-10-15 실기에서는 실행기를 띄울 때마다 이 호출이 필요했다([운영 PC 기록](../evidence/field-2026-10-15-operations-host.md) 5절).

`<ns>/map` 에 이미 발행자가 있으면 `tb6_slam.launch.py` 가 시작하지 않는다(앞 SLAM 이 남은 것). 다른 팀 PC 의 `localization.launch.py`(`/robot6/map_server`) 가 같은 도메인에 떠 있어도 그렇다. 2026-10-15 에는 그 지도가 관제 웹에 우리 지도처럼 들어왔다. `ros2 topic info -v /robot6/map --no-daemon --spin-time 10` 의 발행 노드 이름이 `slam_toolbox` 인지 본다. `START_POSE captured` 가 나오지 않으면 다음 단계로 가지 않는다. 2026-10-14 현장에서는 이 줄이 나오지 않았다. TB-6 TF 의 작은 기울기를 거절했기 때문이고, #291 이 고쳤다. 지금은 두 TF 를 3D 로 합성하고, 합성한 기울기가 10도([제안])를 넘을 때만 위치를 만들지 않는다. 이때 로그에 `SLAM_TF_POSE ... tilted` 가 남는다.

## 5단계 — TB-5 담당 PC

TB-5 도 바닥에 표시한 출발 자리에서 출발한다. 출발값은 TB-6 출발 자세 기준 `robots.robot_b.initial_pose_xy_yaw` 다. 2026-10-14 두 로봇 출발 간격은 1.95 m(가로)·0.05 m(세로)였다 [측정, 재범 줄자, 둘 다 도킹 상태].

아래 launch 는 하나마다 새 터미널에서 띄운다. 터미널마다 1단계 「터미널 준비」 네 줄을 먼저 넣는다(`F` 포함).

```bash
F=$PWD/config/field/search_area.field.json   # 1단계 「터미널 준비」에 있음. 터미널마다 다시
ros2 launch rescue_bringup passage_points.launch.py profile_path:=<profile.json> field_path:=$F robot_id:=robot_b
ros2 launch rescue_bringup overhead_passage.launch.py profile_path:=<profile.json> field_path:=$F robot_id:=robot_b
ros2 launch rescue_bringup frozen_localization.launch.py profile_path:=<profile.json> field_path:=$F \
  robot_id:=robot_b frozen_pose_wait_s:=10.0
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_SUPER_CLIENT=False \
ros2 launch rescue_bringup robot.launch.py profile_path:=<profile.json> field_path:=$F robot_id:=robot_b
```

- **TB-5 는 1차 고정 전에 언독해 둔다.** 출발 정렬(`rescue_start_align`, #317)은 고정 지도가 오고, TB-5 가 언독해 서 있고, 스캔이 들어올 때 시작한다. 도킹한 채로는 `START_ALIGN_DOCKED` 로 기다린다.
- 1차 고정 전 TB-5 는 지도가 없다. `MAP_STATE robot_b ... quality=UNKNOWN` 이 정상이고, 출발 정렬은 `START_ALIGN_WAITING START_ALIGN_WAIT_MAP` 이다.
- 1차 고정 뒤 출발 정렬 로그 `START_ALIGN_ALIGNED` 를 본다. `START_ALIGN_FAILED <이유>` 이면 TB-5 를 출발 자리에 다시 두고 `ros2 service call /robot5/rescue_start_align/restart std_srvs/srv/Trigger {}` 로 다시 한다. 결과는 `/robot5/rescue_start_align`(JSON)에도 있다. 이유 코드는 [rescue_robot README](../../src/rescue_robot/README.md) 상태 표에 있다.
- 현장 파일에 `robots.robot_b.initial_pose_xy_yaw` 가 없으면 위치 추정 launch 가 거절한다. 이때는 시험 회차에서 한 번 잰다.
  1. 1차 고정 지도가 나온 뒤 TB-5 를 출발 자리에 둔다.
  2. `ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot5 map:=<output_dir 의 고정 지도 .yaml>` 을 띄우고 RViz `2D Pose Estimate` 로 자리를 찍는다.
  3. AMCL 이 맞춘 뒤 `python3 scripts/field/capture_initial_pose.py /robot5/amcl_pose map /rescue/map/start_pose/robot_a` 를 돌린다. 나온 한 줄을 현장 파일 `robots.robot_b` 에 넣는다.
  4. 출구(TB-5)와 TB-6 도킹 접근 지점(프로파일 `home_positions`)도 같은 지도에서 같은 도구로 잰다.

## 6단계 — 배선 점검

모든 launch 를 띄운 뒤, 로봇이 움직이기 전에 아무 PC 한 대에서 한다. Discovery 설정은 1단계와 같게 둔다. 로봇이 하나도 안 보이면 먼저 Discovery 전용 PC 의 서버가 떠 있는지 본다([Discovery 전용 PC 운영 안내](discovery-host.md) 3절). 2026-10-15 에는 그 PC 를 재부팅한 뒤 서버가 30분 꺼져 있었다([실습 기록](../evidence/field-2026-10-15-run1015.md) B1).

```bash
ros2 daemon stop && ros2 daemon start
python3 scripts/field/wiring_check.py --profile <profile.json> \
  --field config/field/search_area.field.json --json <결과.json> > /dev/null
echo $?        # 0 = 모두 PASSED, 2 = FAILED 있음, 3 = NOT_RUN 있음
```

- 표는 화면(stderr)에, 줄마다 결과는 JSON 에 남는다. 종료 코드가 0 이 아니면 다음으로 가지 않는다.
- 같은 이름 노드 줄이 FAILED 면 같은 프로그램이 두 곳에서 떠 있다. 우리 노드면 한쪽 launch 를 끈다. 우리 노드가 아니면(예: 두 PC 의 `/rviz2`) 한쪽을 끄거나 `--ros-args -r __node:=<다른 이름>` 으로 이름을 바꾼다.
- 우리가 끌 수 없는 로봇 Pi 기본 노드(`wiring_check.py` `PI_DEFAULT_NODES`, 지금은 `teleop_twist_joy_node` 하나)는 FAILED 대신 PASSED 에 `!` 줄(JSON `notes`)을 붙인다. 이 조이스틱 노드는 TurtleBot4 Pi 기본 launch 가 띄우고 `<ns>/cmd_vel` 에 직접 쓴다. 그래서 enable 버튼을 누르는 동안에는 watchdog 을 거치지 않고 로봇을 움직인다. 회차 중에는 조이스틱을 치워 둔다(2026-10-15 17:10 두 Pi 모두 이 줄이 FAILED 였다). 다른 로봇의 `cmd_vel` 이나 다른 토픽에 쓰면 그대로 FAILED 다.
- 2차 고정 뒤 TB-6 을 다시 볼 때는 `--slam-phase FROZEN` 을 더한다. 한 로봇만 볼 때는 `--robot robot_b` 다.
- 메시지 간격은 이 도구가 보지 않는다. 로봇이 자꾸 멈추면 `scripts/field/link_check.py` 로 끊김 길이를 본다.

## 7단계 — 출동 준비와 실행 (로봇 감독, 운영 PC)

1. **두 로봇 arm.** 로봇마다 아래 순서다. TB-6 은 탐색 중 AMCL 이 없으므로 첫 줄은 TB-5 만 한다.
   ```bash
   ros2 service call /robot5/request_nomotion_update std_srvs/srv/Empty {}
   ros2 service call /mission_coordinator/refresh_state std_srvs/srv/Trigger {}
   ros2 service call <ns>/rescue_command_adapter/task_arm std_srvs/srv/Trigger {}     # success=True, message='ARMED'
   ```
   TB-5 는 자동 출동이라 인형이 나오기 전에 arm 돼 있어야 한다. arm 은 처음 한 번이면 된다. 이동이 정상으로 끝나면 실행기가 스스로 다시 준비한다(#281). 실패·취소·정지 뒤에는 다시 `task_arm` 을 부른다.
   - 답이 `success=False, message='STOP_OR_READINESS_NOT_READY …'` 이면 뒤에 이유 코드(`STOP_EVIDENCE_STALE`·`NAV_NOT_READY` 등), 준비되지 않은 신호(`missing=[...]`), 늦거나 안 온 입력(`sources=[...]`)이 붙는다. 실행기 로그 `TASK_ARM refused …` 에도 같은 줄이 남는다. 준비 상태 입력 가운데 하나가 안 온다는 뜻이다. 로봇 파일(`robots/<id>.v2.json`) `readiness` 의 odom·wheel_status·scan·카메라 토픽과 `map_frame -> odom_frame` TF 가 각 `max_age_s` 안에 오는지 `ros2 topic hz <토픽>` 으로 본다. 2026-10-15 에는 카메라 토픽이 `<ns>/oakd/rgb/preview/image_raw/compressed` 였는데 두 Pi 모두 미리보기를 내지 않아 거절됐다. `<ns>/oakd/rgb/image_raw/compressed` 로 바꾼 뒤에 넘어갔다(재범 결정 2026-10-15, [운영 PC 기록](../evidence/field-2026-10-15-operations-host.md) 6절). 그 뒤 저장소 로봇 파일은 같은 카메라의 `<ns>/oakd/rgb/camera_info` 로 바뀌었다. 준비 점검은 도착만 보므로 큰 RGB 영상을 한 번 더 받지 않는다(같은 기록 B7). 현장 PC 의 로봇 파일도 이 이름으로 맞춘다.
   - **재난을 시작하기 전에 TB-6 Nav2 가 active 인지 본다**: `ros2 lifecycle get /robot6/bt_navigator` 가 `active [3]`. inactive 이면 첫 탐색 목표가 `Action server is inactive` 로 실패한다. 지도가 작아 탐색 경계가 그 하나뿐이면, 조정기는 지도 갱신 세 번 뒤 `EXPLORATION_COMPLETE` 와 `NO_SURVIVOR_FOUND` 로 끝내고 새 목표를 내지 않는다(2026-10-15 17:54, [운영 PC 기록](../evidence/field-2026-10-15-operations-host.md) 7절). 그러면 조정기와 두 실행기를 다시 띄워야 한다.
2. **TB-6 위치 확인.** `ros2 topic echo --once /rescue/robots/robot_a/pose` 의 `mission_pose_status` 가 `VALID` 인지 본다. 서 있는 TB-6 에서 `ros2 topic hz /robot6/pose` 가 끊기는지 기록한다.
3. **재난 시작.** 웹캠 앞을 흔든다. 관제 화면이 DISASTER 가 되고 조정기 로그에 `SEARCH_AUTO_STARTED on DISASTER` 가 나온다. TB-6 은 가장 가까운 탐색 경계로 간다. 수색 목표는 고르지 않는다.
4. **1차 고정(인형 발견).** 인형이 확정되고 보고가 저장되면 조정기가 고정을 부른다. TB-6 은 계속 탐색한다.
   - TB-6 담당 PC 로그 `MAP_FROZEN ... rev=<R0+1> final=False tb6 keeps SLAM`
   - TB-5 담당 PC 로그 `MAP_REVISION_ADOPTED robot_b RNone -> R<R0+1>` → `MAP_RELAYED /rescue/map/frozen R<R0+1> -> /robot5/map` → `MAP_STATE robot_b ... rev=<R0+1> quality=VALID`. 이 줄이 나와야 TB-5 가 출동한다.
   - `MAP_FREEZE_FAILED` 이면 아무것도 바뀌지 않고 TB-5 도 출동하지 않는다.
5. **2차 고정(탐색 완료).** 조정기 로그 `EXPLORATION_COMPLETE` 뒤 조정기가 TB-6 정지를 확인하고 다시 고정을 부른다. TB-6 담당 PC 로그 `MAP_FROZEN ... rev=<R0+2> final=True tb6 amcl start=(...)`.
   1. `MAP_REVISION_ADOPTED robot_a RNone -> R<R0+2>` 를 보고 `tb6_slam.launch.py` 터미널에서 Ctrl+C 를 누른다. `ros2 topic info /robot6/map --no-daemon` 발행자 수가 0 이 될 때까지 기다린다.
   2. TB-6 담당 PC 의 새 터미널에서 1단계 「터미널 준비」 네 줄(`F` 포함)을 넣고 `ros2 launch rescue_bringup frozen_localization.launch.py profile_path:=<profile.json> field_path:=$F robot_id:=robot_a frozen_pose_wait_s:=10.0` 을 띄운다.
   3. `MAP_RELAYED /rescue/map/frozen R<R0+2> -> /robot6/map` 를 본다. `MAP_RELAY_BLOCKED` 이면 SLAM 이 아직 떠 있다.
   4. TB-5 는 멈춰 있으면 `MAP_REALIGN robot_b ...` 로 다시 맞춘다. 움직이는 중이면 `MAP_REVISION_DEFERRED` 가 나오고 멈춘 뒤 받는다.
   5. 두 로봇 모두 `MAP_STATE ... rev=<R0+2> quality=VALID` 이고 `snapshot_id` 가 같은지 본다.
6. **끝.** TB-5 가 인형에서 가장 가까운 출구에 도착하고 TB-6 이 도킹 접근 지점에 도착하면 `MISSION_FINISHED` 다. 인형이 없으면 `NO_SURVIVOR_FOUND` 이고 TB-5 는 출발하지 않는다.

## 8단계 — 끝내기와 다시 하기

- 끝낼 때는 각 PC 의 launch 를 Ctrl+C 로 끈다. 로봇은 로봇 감독이 도킹하고 끈다(자동 도킹 없음).
- 같은 회차에서 지도 고정 노드만 다시 띄울 때는 같은 `output_dir` 를 쓴다. 단 **TB-6 이 그 사이 움직였으면 새 회차로 하고 조정기도 다시 띄운다**(아래 「초기화하고 새 회차로 다시 하기」 ②·④). 조정기는 프로세스 하나에 출발 자세를 한 번만 받는다. 새 출발 자세가 잡히면 조정기는 처음 받은 값과 달라 `START_POSE_CONFLICT` 로 거절한다(2026-10-15 17:42, [실습 기록](../evidence/field-2026-10-15-run1015.md) B11).

### 초기화하고 새 회차로 다시 하기

초기화 API(`POST /api/runs/reset`, #323)는 운영 쪽 회차만 넘긴다. TB-6 SLAM 지도와 지도 고정까지 처음으로 되돌리고, 지난 기록은 회차별로 남는다(재범 결정 2026-10-15). 로봇 쪽 노드는 사람이 다시 띄운다. 자세한 동작은 [운영 구현 문서](../implementation/implementation-operations.md) 「초기화」 절에 있다.

**조건.** 두 로봇이 켜져 있고, 도킹했고, 멈춰 있고, 임무가 없어야 한다. 로봇 launch 를 먼저 끄면 상태가 끊겨 `ROBOT_STATUS_STALE` 로 거절된다. 그래서 **로봇 launch 를 끄기 전에 초기화를 누른다.**

| 순서 | PC | 할 일 | 확인 |
| --- | --- | --- | --- |
| ① | 운영 PC | 로봇 감독이 두 로봇을 도킹한 뒤 아래 「초기화 명령」을 보낸다. 관제 웹 초기화 버튼은 아직 없다(후속, 미구현) | 답 `accepted=true` 와 새 `run_id` |
| ② | 운영 PC | 조정기 launch 를 끈다. 운영 launch(`db_writer`·`report_node`)와 관제 웹은 그대로 둔다 | — |
| ③ | 로봇 담당 PC | 두 로봇의 launch 를 모두 끈다. SLAM, 지도 고정, 고정 지도 relay·AMCL·출발 정렬(`frozen_localization`), 실행기(`robot.launch.py`), 깊이 점, 터널 판정이다 | 사람이 터미널마다 확인 |
| ④ | 운영 PC | 조정기를 다시 띄운다(3단계와 같은 명령) | 조정기 로그 `live task source enabled` |
| ⑤ | 로봇 담당 PC | 4단계를 다시 한다. 로봇 감독이 TB-6 을 언도킹하고 출발 표시에 둔다(사람 확인). 그다음 SLAM, 실행기, 지도 고정을 띄운다. 지도 고정은 **새 `output_dir`** 로 띄운다 | 출발 표시는 사람 확인. 4단계 성공 로그(`START_POSE captured` 포함) |
| ⑥ | 로봇 담당 PC | 5단계를 다시 한다. TB-5 를 언도킹하고 고정 지도 위치 추정·출발 정렬(`frozen_localization`)과 실행기를 띄운다 | 5단계 성공 로그, `START_ALIGN_ALIGNED` |

**초기화 명령(운영 PC).** 포트는 운영 launch 로그 `rescue_db_writer ... api=http://127.0.0.1:<port>` 의 값이다. `command_id` 는 보낼 때마다 새 UUID 다. 같은 `command_id` 를 다시 보내면 처음 답이 `DUPLICATE` 로 오고 다시 초기화하지 않는다.

```bash
curl -s -X POST http://127.0.0.1:<port>/api/runs/reset -H 'Content-Type: application/json' \
  -d "{\"command_id\": \"$(python3 -c 'import uuid; print(uuid.uuid4())')\", \"operator_id\": \"<운영자 이름>\"}"
```

재기동 확인 표(조정기·실행기가 다시 떴는지)는 운영 DB 에만 있고(`site_state.read_reset_checklist`), 관제 웹 화면은 아직 없다(후속, 미구현). 그래서 지금은 위 로그로 확인한다.

- **도킹한 채로 지도 고정 노드를 띄우지 않는다.** 지도 고정 노드는 처음 받은 TB-6 위치를 이 회차의 출발 자세로 남긴다. 언도킹한 자리에서 잡아야 TB-5 출발값과 맞는다.
- **조정기와 두 실행기는 반드시 함께 다시 띄운다.** 실행기는 이미 받은 것 이하의 임무 번호를 `STALE_GENERATION` 으로 거절한다.
- **relay 와 실행기를 빠뜨리지 않는다.** 앞 회차의 relay 가 남아 있으면 새 회차의 R0+1 이 다른 지도라 `MAP_RELAY_BLOCKED` 로 막힌다.

초기화가 거절되면 아무것도 바뀌지 않는다. 배치 로봇 둘 중 처음 걸린 이유 하나가 답에 온다.

| 이유 코드 | 뜻 | 고치는 법 |
| --- | --- | --- |
| `ROBOT_STATUS_STALE` | 로봇 상태가 없거나 받은 지 5초가 넘었다([제안] 값) | 그 로봇의 `robot.launch.py` 가 떠 있는지 본다. 이미 껐으면 다시 띄우고 상태가 관제 웹에 보인 뒤 누른다 |
| `ROBOT_MOVING` | 로봇이 멈춤(`STOPPED`)이 아니다. 정지 미확인(`STOP_UNCONFIRMED`) 포함 | 로봇이 멈출 때까지 기다린다. 움직이면 「정지 방법」대로 세운다 |
| `TASK_ACTIVE` | 로봇에 임무가 남아 있다 | 「정지 방법」 1번으로 임무를 멈춘다. 관제 웹 로봇 카드에서 임무가 없어진 것을 본 뒤 누른다 |
| `ROBOT_NOT_DOCKED` | 도킹 상태가 아니다. 도킹 상태를 모를 때도 포함 | 로봇 감독이 도킹한다. `<ns>/dock_status` 가 오는지 본다 |
- 회차마다 남길 것. 코드 SHA, 각 PC 의 `preflight_role.py`·`wiring_check.py` JSON, 지도 고정 `output_dir`, 단계별 로그, 결과(PASSED·FAILED·BLOCKED·NOT_RUN)와 원문 증거. 저장소에 올릴 때는 PC 이름·IP·개인 경로를 지운다(`docs/evidence/field-<날짜>-<역할>.*`).
- 로봇을 재부팅하거나 네트워크를 바꾼 뒤에는 1단계 준비 점검과 6단계 배선 점검을 다시 한다.

## 웹 없이 하기

관제 웹 없이 RViz 와 터미널만으로 실기 회차를 돌린다(재범 결정 2026-10-15 저녁). Gazebo 는 쓰지 않는다. 1·2·4·5·6단계는 위와 같다. 3·7·8단계에서 관제 웹이 하던 일만 아래 도구가 대신한다. 도구 설명은 [`scripts/field/noweb/README.md`](../../scripts/field/noweb/README.md)에 있다.

**3단계에서 바뀌는 것.**
- 운영 launch 는 꼭 띄운다. 웹캠 흔들림 판정과 보고 저장이 여기에 있다. 3단계 명령 그대로 **`map_topic:=/robot6/map map_dir:=<실시간 지도 폴더>` 도 준다.** 빼면 `status_watch.py` 가 지도 고정 단계를 알 수 없어 「지도 보관 꺼짐 — 고정 여부 모름」을 찍는다.
- 관제 웹(`python3 -m rescue_console ...`)은 띄우지 않는다.
- 조정기는 3단계 명령 그대로 띄운다.

**관제 웹 대신 쓰는 것.** 포트는 운영 launch 로그 `rescue_db_writer ... api=http://127.0.0.1:<port>` 의 값이다.

| 할 일 | 도구 | 어디서 |
| --- | --- | --- |
| 상태 보기 | `python3 scripts/field/noweb/status_watch.py --db <site_state.sqlite3> --log <회차 폴더>/status.log` | 운영 PC |
| 지도와 두 로봇 보기 | `rviz2 -d scripts/field/noweb/two_robots.rviz --ros-args -r /tf:=/robot6/tf -r /tf_static:=/robot6/tf_static` | RViz 를 띄울 PC |
| 인형과 출구 위치 보기 | `python3 scripts/field/noweb/target_markers.py` 를 띄우면 위 RViz 화면의 「Targets and exits」에 그려진다 | RViz 를 띄울 PC |
| 재난 선포 | 웹캠 앞을 흔든다. 흔들기 어려우면 `python3 scripts/field/noweb/operator_command.py --port <port> --operator <운영자 이름> DECLARE_DISASTER` | 운영 PC |
| 초기화 | 8단계 ①의 `curl` 대신 `python3 scripts/field/noweb/operator_command.py --port <port> --operator <운영자 이름> RESET_RUN` | 운영 PC |
| 정지 | 「정지 방법」의 `bash scripts/field/stop_robot.sh /robot6`(TB-5 는 `/robot5`) | 로봇 담당 PC |

- `operator_command.py` 는 답 JSON 을 그대로 찍는다. 끝 코드는 0 접수, 1 거절, 2 보내지 않음, 3 답 없음이다. 3 이면 명령이 들어갔는지 모르니 `status_watch.py` 줄로 확인한 뒤 다시 보낸다. 다시 보내면 새 명령이다.
- 7단계에서 「관제 화면이 DISASTER 가 되고」는 `status_watch.py` 줄의 `임무 DISASTER/...` 로 본다. 1차·2차 고정은 줄 끝의 `고정 지도 r<R0+1>`·`r<R0+2>` 와 로봇 담당 PC 로그로 본다.
- 8단계 초기화 뒤의 재기동 확인은 `status_watch.py` 줄의 `초기화 확인 조정기=... robot_a=... robot_b=...` 로 본다. SLAM, 지도 고정, relay, AMCL 재기동은 운영 DB 에 남지 않으니 터미널마다 사람이 확인한다.
- RViz 하나는 TF 를 하나만 읽는다. 위 명령은 TB-6 의 TF 를 읽으므로 TB-5 의 모델과 라이다는 그리지 않는다. TB-5 는 AMCL 자세 화살표와 경로로만 보인다. 1차 고정 전에는 TB-5 가 안 보이는 것이 정상이다.
- Nav2 상태는 운영 DB 에 없다. 7단계 1번처럼 `ros2 lifecycle get /robot6/bt_navigator` 로 본다.

**현장 폴더(저장소 밖).** 회차 이름은 `<날짜-noweb-순번>` 이다. 웹으로 하는 회차와 폴더·DB 를 나눈다.

| PC | 폴더 | 들어가는 것 |
| --- | --- | --- |
| 로봇 담당 PC | `~/Dev/rokey7-field/runs/<날짜-noweb-순번>/` | `freeze/`(지도 고정 `output_dir`), `logs/`, `HEAD.txt`(코드 SHA) |
| 운영 PC | `~/rokey7-field/noweb/<날짜-noweb-순번>/` | `site_state.sqlite3`(운영 launch `db_path`), `reports.sqlite3`(`report_db_path`), `logs/`, `status.log` |

- RViz 화면 파일은 저장소에서 문법만 시험했다. 실제 화면은 실기에서 아직 보지 않았다(NOT_RUN).

## 실기에서 아직 확인하지 못한 것 (NOT_RUN)

이 순서를 처음부터 끝까지 실제 로봇으로 돌린 회차는 아직 없다. 실기에서 볼 항목은 [rescue_bringup README](../../src/rescue_bringup/README.md) 「실기 NOT_RUN」 절에 모여 있다. 그중 이 순서에 바로 걸리는 것은 아래다.

- 시뮬레이션에서 찾아 고친 것(C1~C9)의 실기 확인. 목록과 상태는 [시뮬레이션에서 찾은 것을 현장 코드로 옮기기](sim-to-field.md)에 있다. 실기 전에 「실기」 칸이 빈 줄을 본다.
- TB-6 출발 자세와 지도 고정. 2026-10-14 현장에서 실패했고 #291 이 고쳤다. 한 PC 리허설(현장 TF 값)에서는 통과했고, 실기는 아직이다.
- 권고안 A 에서 TB-5 담당 PC 가 `/rescue/map/frozen` 을 받는지 [추정, 실측 전].
- 후진 없는 BT 가 설치된 Nav2 에서 올라오는지(2단계 대조).
- 자동 재준비로 TB-6 이 탐색 지점마다 멈추지 않는지, TB-5 가 사람 앞에서 기다린 뒤 출구로 떠나는지.

## 기록 (2026-10-08 판)

아래는 2026-10-08 판 본문을 그대로 옮긴 것이다. 당시에는 현장 launch 가 없어 관문 목록만 있었다. 5절 「실기 날 아침 점검 목록」은 10-12·10-13 에 더한 것이다. 지금 순서는 위 절을 본다.

### 현장 실행 전 관문

현재 실행 가능한 통합 산출물은 [오프라인 E2 데모](../implementation/implementation-integration.md)다. 현장 ROS launch, 실제 드라이버/SLAM/Nav2 adapters, 모델 추론과 센서 검증, 물리적 정지 확인은 아직 완성/실행하지 않았다. 아래는 현장에서 구현·측정할 순서이며 이번 PR에서 성공한 실기 절차가 아니다. 실행할 수 없는 ROS 명령을 예시로 만들어 넣지 않는다.

실제 AMR 2대 활용은 사용자 확정 요구다. 고사양 PC 2대·중간 사양 PC 2대는 예정 자원이고 네 대를 모두 쓰는 것은 현재 배포 기준안의 검증 조건이다. 발견→접근→보고와 4-PC 전체 사용의 최종 인수 의무는 [scope Q-08](../requirements/scope.md)에 남긴다. 조건이 바뀌면 배치·프로파일·AC를 함께 수정한다.

#### 1. 실행 기록과 장비 확인

현장 담당과 통합 담당이 두 로봇과 네 PC의 실제 식별자·CPU/GPU/RAM·OS·ROS/패키지/드라이버/펌웨어 버전·센서·네트워크를 기록한다. 고사양 PC별 SLAM/Nav2·추론, 중간 PC별 협업/관제 배치는 부하 측정 전 가설이다. 저장소의 example 이름을 실제 장비 확인으로 사용하지 않는다.

매 run에 run ID, 코드 전체 SHA·dirty diff, 모든 호스트의 배포 SHA, 프로파일/로봇별 parameter hash, 모델/데이터/보정 hash, 시계 상태, map session/revision·transform revision, 실제 명령, 원시 로그·영상·지도·관측/임무/보고 ID, 미실행 이유를 연결한다. 날짜만 같은 결과를 같은 실행으로 묶지 않는다. 비공개 원본을 확인하지 못하면 해당 출처 검증은 미확인으로 남긴다.

먼저 아래 **로컬 E2 명령**을 저장소 루트에서 실행할 수 있다. 새 output 경로를 사용한다.

```sh
export PYTHONPATH=src/rescue_bringup:src/rescue_robot:src/rescue_mapping:src/rescue_perception:src/rescue_coordination:src/rescue_operations:src/rescue_contracts
python3 -m rescue_bringup config/deployment.example.json
python3 -m unittest discover -s tests -p 'test_*.py' -v
python3 -m rescue_bringup.offline_demo config/deployment.example.json --output work/offline-run-001
```

이 명령은 호스트에 서비스를 배포하지 않는다. 설정의 `runtime_enabled=false`를 true로 편집하는 것만으로 실행이 활성화되는 구조도 아니다. 실제 launch/adapters를 구현하고 해당 검사를 통과한 뒤 문서에 검증한 실기 명령을 추가해야 한다.

#### 2. E3 연결과 제어 권한

| 관문 | 준비/시험 | 다음 단계에 필요한 증거 |
| --- | --- | --- |
| 그래프·장치 소유 | 단일 그래프 A 또는 로봇별 domain+API B 선택. 호스트별 domain·Discovery Server 목록/고유 ID·ROS_SUPER_CLIENT·participant/구독자 수 기록 | 실제 메시지 수신 시각·요청/응답·재연결, namespace/TF buffer/명령 경로 분리. 목록 조회만으로 PASS 금지 |
| 준비/도킹 | 장치 driver·watchdog, 별도 권한의 UNDOCK/DOCK adapter 구현. power_saver/도킹 중 센서 발행 조건 확인 | 접수 후 무동작·timeout과 실제 dock status 확인. 센서 READY와 이동 허가 분리; 이동 전 fresh is_docked=false |
| 단독 명령 소유 | 주행·도킹 요청은 rescue_robot 실행기 하나가 발행. 탐색기는 후보만 제공 | competing goal/속도 발행자 없음. 수락 지연·늦은 terminal·cancel·lease/authority/boot fencing과 독립적인 최종 명령 억제 경로 |
| 시각/센서 | 원 sec/nanosec·clock domain·stamp 의미·오차/드라이버 지연 상한 측정. RGB/Depth/CameraInfo·정렬·encoding/단위 확인 | 0/미래/역순/UNKNOWN clock 거절, 올바른 원 시각 TF 조회, 수신 지연이 원 관측 수명을 연장하지 않는 기록 |

P4는 로봇별 domain과 서버를 기록했고 단일 그래프 A는 당시 제안이었다([DEC-010](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/decisions.md#L142-L151)). 다른 PC의 `/robot3/map` 발행자 2개는 [실습 L33](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L33), Discovery Server CPU 92%는 [실습 L34](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L34)다. L24–L32의 가짜 controller, 시계 점검, 언도크, 탐색 성공 행은 이 두 사고의 근거가 아니다. 이 값은 이번 네 PC의 재현 결과가 아니므로 새 구성에서 다시 측정한다.

#### 3. 현장 수치·좌표·탐지 관문

[시험 계획의 T-01~14](acceptance-plan.md)를 단위·근거·정책 version/hash·결정 시각과 함께 `LOCKED_FOR_RUN`으로 확정한다. 현재 코드의 synthetic fixture 수치는 근거가 아니다. 합격 기준을 실행 결과에 맞춰 사후 낮추고 같은 회차를 PASS로 바꾸지 않는다. 값이 비어 있거나 stamp 의미/시계 오차가 미확인이면 그 값에 의존하는 자율 접근은 준비되지 않은 상태다.

독립 SLAM과 물리 기준점 측정부터 시작한다. 로봇별 TF·map 소유, A/B의 같은 기준점 오차, 다른 출발 방향, pose drift와 실제 backend의 loop correction 이벤트/posegraph 연결을 확인한다. 임의 revision 증가나 일반 map→odom 변화만으로 보정 감지 성공을 주장하지 않는다. 영향 관측을 재투영할 근거가 없으면 재관측으로 전환하며, A/B 원 관측별 provenance를 보존한다.

현재 mapping의 `auto_approach_eligible`은 false이며 운영 준비 provider가 없다. 실제 backend correction과 readiness/권한/시간/좌표를 연결한 adapter·실패 처리를 구현하고 검증하기 전에는 교차 로봇 접근을 허가하지 않는다. 좌표가 맞는 것과 명령을 보내도 되는 것은 별도 관문이다.

사람/시연 인형·촬영 세션을 분리한 평가 데이터로 모델 class·confidence·track 의미, Depth 결측·배경 혼입, 위치 오차/공분산, 가림·조명을 검증한다. 객체의 점 자체를 goal로 보내지 않고 자유 공간·footprint·불확실성·제동/보호 영역과 실제 경로를 평가한다. 전송 FPS·합성 타일·기존 car/dummy 점수는 이 평가를 대신하지 않는다.

#### 4. 제한된 공간의 E4 실행 순서

CT-06·CT-07의 순수 코어 E2 부분은 [통합 리허설](../implementation/implementation-integration.md)에서 수색 SEND 의도→취소 요청→terminal CANCELED→합성 정지 증거→새 generation→접근 admission→실제 TaskAdmission으로 배정 승격까지 연결했다. 자동 접근 SEND는 여전히 차단된다. 이 결과는 ROS action 취소/terminal 전달, 실제 odometry의 정지 판정, 접근 주행·재관측을 검증하지 않는다. 아래 E3/E4 관문은 별도로 구현하고 실행해야 한다.

현장 실행 담당은 합의한 속도·영역·정지 수단으로 먼저 한 로봇의 최종 명령 차단·fresh stop 관측을 검증하고, 두 로봇의 분리된 수색과 공통 좌표 관문을 이어간다. 이후 [AC-01~07](acceptance-plan.md)에 따라 현재 배포 기준안의 실제 네 PC 부하에서 역할을 교환한다.

1. 두 로봇 센서/TF/SLAM·명령 분리와 host별 부하·수신/처리 지연을 기록한다.
2. A 발견→B 수색 취소→정지 확인→새 generation·후보 경로→B 접근·fresh 재관측·이격/정지→보고 저장을 검증한다. A의 별도 수색 유지/재개 정책과 역할 교환도 확인한다.
3. 장애물 추가/제거를 센서→costmap→경로→SLAM 각 층에서 측정한다. 프로세스 생존만으로 변화 대응을 통과시키지 않는다.
4. E2/E3에서 먼저 검증한 관측 만료·SLAM 보정·센서/TF·lease·연결 단절·늦은 action·좁은 통로 점유 실패를 현장 조건 안에서 확인한다. 정지 요청/응답·terminal fence·실제 정지 증거와 재개 권한을 구분한다.
5. 접근 불가·미확인도 원인과 발견 증거를 보고한다. Q-07에서 정할 사진 정책과 실제 evidence bytes·receipt를 확인하고, 응답 유실/재시작 뒤 같은 report ID 복구를 검증한다.

`CONFIRMATION` 사진 정책은 재확인하지 못한 보고를 PENDING_EVIDENCE에 남긴다는 현재 구현 한계가 있다. 실패 보고의 완료까지 요구하는 운영에서는 정책을 그대로 채택하지 않고 사전에 보고 종류별 요구와 구현을 맞춘다. 사진 정책·운영 scheduler·원격 인증·증거 보존/용량은 E2 데모가 결정하지 않는다.

각 단계는 NOT_RUN/BLOCKED/FAILED/PASSED와 원문 증거를 남긴다. 실제 두 로봇 사용을 한 로봇 재생으로 대체하지 않으며, 네 PC 기준안 시험을 단일 프로세스의 네 host 이름으로 대체하지 않는다. 필수 관문이 미실행이면 해당 실기 완료 주장은 남겨 둔다.

#### 5. 실기 날 아침 점검 목록 (10-12·10-13, 실기·배포 2026-10-08)

로봇에 닿기 전 PC 쪽 준비를 확인하는 순서다. 로봇을 움직이는 일(언도크·이동·도킹)은 이 목록에 없고 로봇·지도 담당 한 사람·한 세션만 한다([팀 작업 규칙](../../AGENTS.md) 6절). 각 줄의 결과는 그날 실행 기록에 남기고, preflight JSON 은 `docs/evidence/field-<날짜>-preflight-<role>.json` 처럼 역할 이름으로 저장한다(호스트 이름·IP·개인 경로는 지운다).

| # | 누가 | 할 일 | 통과 조건 |
| --- | --- | --- | --- |
| 1 | 사용자 | 오늘 쓸 로봇 두 대의 번호·네임스페이스·서버 ID([scope Q-01](../requirements/scope.md))와 로봇을 움직일 사람·세션을 팀에 알린다 | 값이 정해져 있다. 정해지지 않았으면 이후 단계는 `BLOCKED` |
| 2 | 각 PC 담당 | 저장소 `git rev-parse HEAD` 와 `git status --short` 를 적는다 | 모든 PC 가 같은 SHA, 작업 폴더가 깨끗하다 |
| 3 | 각 PC 담당 | 새 터미널에서 ROS 를 source 하고 [Discovery 설정 파일](../design/network-discovery.md)(8절)을 source 한다. 오프라인 시험 터미널과 섞지 않는다 | `.bashrc` 해시가 어제와 같다 |
| 4 | 각 PC 담당 | 로봇을 켜기 전에 `python3 -I scripts/field/preflight.py --host-config config/hosts/<role>.example.json --json <파일>` | `env.*`·`pkg.*`·`tool.fastdds` 가 전부 `PASSED`. 서버·그래프 항목은 로봇이 꺼져 있어 `NOT_RUN` 이어도 된다 |
| 5 | 로봇·지도 담당 | 로봇 전원을 켜고(도킹 상태 그대로) 부팅을 기다린 뒤 각 PC 에서 `ros2 daemon stop; ros2 daemon start` | — |
| 6 | 각 PC 담당 | 4번 명령을 다시 실행한다 | `graph./robot_*` 가 `NOT_RUN` 이 아니고, `cmd_vel`·`map` 발행자가 각각 1 이하다. `server.*.reach` 가 `FAILED` 면 멈춘다 |
| 7 | 관제 PC 담당 | `ROS_SUPER_CLIENT=True ros2 node list --no-daemon` 로 각 로봇 서버에 붙은 노드를 보고, 서버 목록을 적기로 한 PC 말고 다른 PC 의 SLAM·Nav2 가 붙어 있지 않은지 본다 | 예상하지 않은 PC 의 노드가 없다(2026-10-06 P4 `/robot3/map` 중복 발행 사고) |
| 8 | 로봇·지도 담당 | 여기서부터 로봇·지도의 E3/E4 순서(이 문서 2절)로 넘긴다 | 1 ~ 7 이 통과했다는 기록이 있다 |
| 9 | 각 PC 담당 | 끝날 때 현장 터미널을 닫는다. 로봇은 로봇·지도 담당이 도킹하고 끈다 | 다음 날 3번에서 `.bashrc` 해시가 같다 |

로봇을 재부팅하거나 네트워크를 바꾼 뒤에는 5 ~ 7 을 다시 한다. preflight 가 `FAILED` 를 내면 원인을 고치기 전에는 로봇·지도에 넘기지 않는다. `NOT_RUN` 을 통과로 적지 않는다.

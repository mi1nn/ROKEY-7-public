# 현장 운영 PC 준비 기록 (2026-10-13)

- 대상 PC: 프로파일 호스트 `operations` + `coordination`(같은 물리 PC). 관제 웹·현장 상태 DB·보고 저장·웹캠.
- 대상 코드: main `d2648ea`(1차 기동) → `ceac0f3`(#156·#158·#161 머지 뒤 재기동). 둘 다 `colcon build` 9 패키지 성공.
- 로봇은 움직이지 않았다. 로봇에 명령을 보내지 않았고, PC·Discovery·와이파이 설정을 바꾸지 않았다. 재부팅하지 않았다.
- ROS 시험은 모두 격리 터미널: `unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT; export ROS_DOMAIN_ID=88 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. 기동한 프로세스 4개와 관제 웹의 환경을 `/proc/<pid>/environ` 으로 확인했다.
- 경로는 `<HOME>`, 저장소 밖 현장 폴더는 `<FIELD>`, 시험 실행 폴더는 `<RUN>`, 주소는 `<PC_IP>`·`<TB5_PI_IP>`·`<DISCOVERY_IP>`·`<GW>` 로 바꿨다. 시각은 KST.

## 결과 요약

| # | 항목 | 결과 | 시각 |
| --- | --- | --- | --- |
| 1 | Ubuntu 24.04.5 · ROS 2 jazzy · Python 3.12.3 | PASS [측정] | 12:38 |
| 1 | 저장소 HEAD: main 이 origin 보다 207 커밋 뒤 → `git pull --ff-only` 로 최신 | PASS [측정] | 12:39 |
| 1 | `colcon build` 9 패키지 | PASS [측정] | 12:40, 12:53 |
| 1 | 웹캠 by-id 경로 | PASS [측정] (예시 YAML 의 장치와 다른 HP 웹캠) | 12:39 |
| 1 | `cv2` 5.0.0 · `ultralytics` 8.4.170 · `flask` 3.1.3 import | PASS [측정] | 12:39 |
| 1 | 웹캠 모델 sha256 = 예시 YAML `model_sha256` | PASS [측정] | 12:41 |
| 2 | timesyncd active, chrony inactive, NTPSynchronized=yes | PASS [측정] | 12:39 |
| 3 | 와이파이 절전 | **on** — 끄지 않음(결정은 재범) [측정] | 12:40, 12:52 |
| 3 | TB-5 Pi ping 50회 | **FAIL** 100% 손실 — 이 PC 가 로봇망과 다른 SSID [측정] | 12:52 |
| 3 | TB-6 Pi ping | NOT_RUN: 주소 대기 | — |
| 4 | 현장 웹캠 설정 `<FIELD>/webcam_node.yaml` | PASS. `watch_exit_roi`·감시 ROI 는 예시값 [미측정] | 12:41 |
| 5 | `operations.launch.py`: db_writer·report_node·webcam_node 기동, `clock SYNCED` | PASS [측정] (기동 뒤 약 5 s) | 12:42, 12:54 |
| 5 | 관제 웹이 장부 없이 뜸(#147) | PASS [측정] | 12:42 |
| 5 | 로봇 카드 2개 | `d2648ea`: FAIL(0개, #161 미머지) → `ceac0f3`: PASS(robot_a·robot_b) [측정] | 12:42, 12:54 |
| 5 | 영상 중계(#158) `mjpeg_relay` 기동·MJPEG 수신·관제 웹 `--camera` | PASS [측정] | 12:54 |
| 5 | Firefox 화면 눈 확인 | [대기: 재범] | — |
| 5 | 웹캠 가림 → 「영상 끊김」·복구 | [대기: 재범] | — |
| 5 | 웹캠 흔들기 → SiteEvent DISASTER_SIGN → 관제 화면 모드 | [대기: 재범] | — |
| 6 | `preflight_role.py --role operations` | 환경 2개만 FAILED(예시 프로파일 기준), 나머지 PASSED [측정] | 12:55 |
| 6 | `preflight_role.py --role coordination` | 환경 2개만 FAILED(예시 프로파일 기준), 나머지 PASSED [측정] | 12:55 |

## 1. 현황

```text
$ cat /etc/os-release | head -3; echo ROS=$ROS_DISTRO
PRETTY_NAME="Ubuntu 24.04.5 LTS"
NAME="Ubuntu"
VERSION_ID="24.04"
ROS=jazzy

$ git status -sb   # 12:39, pull 전
## main...origin/main [behind 207]
$ git log -1 origin/main
d2648ea 2026-10-13 12:38:10 +0900 Merge pull request #162 from jaebeom/fix/mjpeg-stuck-client

$ ls -l /dev/v4l/by-id/
usb-DMWCV0ABIG6GLU_HP_Wide_Vision_HD_Camera_01.00.00-video-index0 -> ../../video0
usb-DMWCV0ABIG6GLU_HP_Wide_Vision_HD_Camera_01.00.00-video-index1 -> ../../video1

$ python3 -c "import cv2, ultralytics, flask; ..."
cv2 5.0.0
ultralytics 8.4.170
flask 3.1.3

$ sha256sum assets/models/webcam-yolo26n-best.pt
8b61f691f17a538410c80d6d8ef7aad7ad80b77365f76a9963e6164e797f18d5  assets/models/webcam-yolo26n-best.pt
```

## 2. 시계

```text
$ systemctl is-active systemd-timesyncd chrony
active
inactive
$ timedatectl show --property=NTPSynchronized --value
yes
```

## 3. 와이파이

```text
$ ip -4 -br addr
lo               UNKNOWN        127.0.0.1/8
wlp5s0           UP             <PC_IP>/24
tailscale0       UNKNOWN        <TAILSCALE_IP>/32

$ iw dev wlp5s0 get power_save
/bin/bash: line 1: iw: command not found        # iw 미설치(설치하지 않음). 대신 iwconfig 로 읽음
$ iwconfig wlp5s0 | grep -i "power\|ESSID"
wlp5s0    IEEE 802.11  ESSID:"<CAMPUS_SSID>"
          Power Management:on
$ nmcli -f 802-11-wireless.powersave connection show <연결>
802-11-wireless.powersave:              0 (default)
$ cat /etc/NetworkManager/conf.d/*wifi*powersave*
[connection]
wifi.powersave = 3                                # 3 = 켬

$ ping -c 50 -i 0.2 <TB5_PI_IP> | tail -3          # 12:52:25
--- <TB5_PI_IP> ping statistics ---
50 packets transmitted, 0 received, 100% packet loss, time 10116ms

$ ip route get <TB5_PI_IP>
<TB5_PI_IP> via <GW> dev wlp5s0 src <PC_IP>        # 로봇망 대역으로 가는 경로 없음, 기본 게이트웨이로 나감
$ ping -c 5 -i 0.2 -W 1 <DISCOVERY_IP> | tail -2   # 옛 Discovery 서버(로봇망)
5 packets transmitted, 0 received, 100% packet loss, time 822ms
$ ping -c 5 -i 0.2 -W 1 <GW> | tail -2
5 packets transmitted, 5 received, 0% packet loss, time 803ms
```

해석: 이 PC 는 캠퍼스 SSID(`<CAMPUS_SSID>`, <PC_IP> 대역)에 있고 로봇·Discovery 서버는 다른 대역(로봇망 `<ROBOT_SSID>`)이다. 그래서 `~/.bashrc` 의 Discovery 설정(도메인 6, 서버 1개)으로도 지금은 로봇 그래프에 붙지 못한다. 로봇망 `<ROBOT_SSID>` 로 옮길지는 재범 결정이다. 옮기면 ping·절전을 다시 잰다.

## 4. 현장 웹캠 설정

설치본 `share/rescue_perception/config/webcam_node.example.yaml` 을 복사하고 아래만 바꿨다. 시계 키 5줄(#146)은 예시값이 지시값과 같아서 그대로다(`clock_domain_id: rokey7-utc-field`, `clock_backend: timesyncd`, `clock_max_age_ns: 3600000000000`, `clock_drift_ppm: 100.0`, `clock_poll_period_s: 5.0`).

```text
$ diff <install>/webcam_node.example.yaml <FIELD>/webcam_node.yaml
> # 현장 사본 2026-10-13: 설치본 webcam_node.example.yaml 복사, device_path·model_path(절대)만 바꿈
<     device_path: /dev/v4l/by-id/usb-Web_Camera_Web_Camera_202400126002-video-index0
>     device_path: /dev/v4l/by-id/usb-DMWCV0ABIG6GLU_HP_Wide_Vision_HD_Camera_01.00.00-video-index0
<     model_path: assets/models/webcam-yolo26n-best.pt
>     model_path: <HOME>/ROKEY-7/assets/models/webcam-yolo26n-best.pt
<     watch_exit_roi: [0.0, 0.0, 0.2, 1.0]
>     watch_exit_roi: [0.0, 0.0, 0.2, 1.0]   # [미측정] 카메라 설치 뒤 재범과 정함 (2026-10-13)
```

시험용 프로파일 `<FIELD>/profile.json` 은 `config/deployment.v2.example.json` 에 #152 0단계의 `map_basis`(`frame_id: map`)만 넣은 것이다. `parameters_file` 은 프로파일 폴더 안 상대 경로여야 해서(심볼릭 링크도 거절) `config/robots/` 를 `<FIELD>/robots/` 로 복사했다. **이 프로파일은 예시라 namespace 가 `/robot_a`·`/robot_b`, 도메인 7, 문서용 주소다. 현장 프로파일(`/robot6`·`/robot5`, 실제 Discovery)은 아직 없다.**

```text
$ PYTHONPATH=src/rescue_bringup python3 -m rescue_bringup <FIELD>/profile.json
STATIC CONFIG VALID: review_example_v2; 2 robots; 4 hosts (2 high, 2 mid)
robot_a: namespace=/robot_a, command_host=compute_a, role=SCOUT
robot_b: namespace=/robot_b, command_host=compute_b, role=SUPPORT
mission: schema v2; 2 search areas; hold positions for robot_b
v2.1 blocks: 1 site camera(s): cam_fixed[DISASTER_SIGN+TARGET_WATCH]; 4 hosts with network; readiness for robot_a, robot_b
Runtime is disabled. No hardware, ROS graph, timing or safety validation was performed.
```

## 5. 격리 시험 기동

명령(`ceac0f3`, 2차):

```text
ros2 launch rescue_bringup operations.launch.py profile_path:=<FIELD>/profile.json \
  db_path:=<RUN>/site_state.sqlite3 api_port:=18080 command_timeout_s:=3.0 \
  report_db_path:=<RUN>/reports.sqlite3 envelope_schema_version:=rescue-envelope/1 report_publisher_id:=ops.report \
  webcam_params_file:=<FIELD>/webcam_node.yaml \
  video_port:=18082 video_bind_host:=127.0.0.1 video_stale_s:=2.0
```

로그(줄 끝 생략):

```text
[INFO] [launch.user]: console --camera cam_fixed=http://127.0.0.1:18082/cam_fixed.mjpg
[INFO] [db_writer-1]: process started with pid [...]
[INFO] [report_node-2]: process started with pid [...]
[INFO] [webcam_node-3]: process started with pid [...]
[INFO] [mjpeg_relay-4]: process started with pid [...]
[rescue_report_node]: rescue_report_node git=ceac0f3ba9df0b16497f6f7f371b40f26063845b boot=... db=<RUN>/reports.sqlite3 reports=/rescue/reports receipts=/rescue/report_receipts
[rescue_db_writer]: rescue_db_writer git=ceac0f3ba9df0b16497f6f7f371b40f26063845b db=<RUN>/site_state.sqlite3 schema=1.2 robots=[('robot_a', ...), ('robot_b', ...)] ... api=http://127.0.0.1:18080 service=/rescue/operator_command
[mjpeg_relay]: mjpeg_relay git=ceac0f3ba9df0b16497f6f7f371b40f26063845b+dirty parameters={"bind_host": "127.0.0.1", "image_topic": "/rescue/cameras/cam_fixed/debug_image/compressed", "path": "/cam_fixed.mjpg", ...
[webcam_node]: webcam_node git=ceac0f3ba9df0b16497f6f7f371b40f26063845b ... model=<HOME>/ROKEY-7/assets/models/webcam-yolo26n-best.pt sha256=8b61f691...18d5 camera=cam_fixed device=/dev/v4l/by-id/usb-DMWCV0ABIG6GLU_HP_Wide_Vision_HD_Camera_01.00.00-video-index0->/dev/video0 640x480 period=0.2s ... clock=timesyncd domain=rokey7-utc-field max_age_ns=3600000000000 drift_ppm=100.0 poll=5.0s
[WARN] [1791863679.344399926] [webcam_node]: clock UNKNOWN backend=timesyncd: waiting for clock measurement
[INFO] [1791863684.375329019] [webcam_node]: clock SYNCED backend=timesyncd error_bound_ns=306117295 measured_at_ns=1791862658237259000
```

- `mjpeg_relay` 의 `+dirty` 는 작업 폴더의 추적 안 하는 `.vscode/` 하나 때문으로 보인다(`git status --short` 에 `?? .vscode/` 만 있음) [추정: dirty 판정 방식은 확인하지 않음].
- 1차(`d2648ea`, 중계 없음)도 같은 3개 노드가 떴고 `clock SYNCED` 까지 약 5 s 였다.

관제 웹(격리 환경, 장부 인자 없음):

```text
rescue-console --accounts <RUN>/accounts.sqlite3 --site-state <RUN>/site_state.sqlite3 \
  --operations http://127.0.0.1:18080 --camera cam_fixed=http://127.0.0.1:18082/cam_fixed.mjpg --port 18081
 * Running on http://127.0.0.1:18081
```

로그인 뒤 첫 화면 HTML 에서 읽은 것 [측정, curl]:

```text
경고: 보고 장부 입력 없음 — 구조 보고 행·건수를 표시하지 않음 ...        (#147)
모드: 평시 (data-mode="NORMAL") · 임무 상태 기록 전 — 진동 감지 없음
로봇: robot_a 상태를 한 번도 받지 못했습니다. ... / robot_b 상태를 한 번도 받지 못했습니다. ...   (ceac0f3, #161)
      d2648ea 에서는 로봇 칸이 비어 있었다(0개).
웹캠: <img src="http://127.0.0.1:18082/cam_fixed.mjpg" ... onerror → '영상 끊김'>

$ timeout 3 curl -s http://127.0.0.1:18082/cam_fixed.mjpg -o sample.bin
856064 bytes, 첫 부분: --frame / Content-Type: image/jpeg / Content-Length: 53235 / JFIF
```

남은 눈 확인(재범): Firefox 화면, 웹캠을 손으로 가렸다 뗄 때 「영상 끊김」·복구, 웹캠 흔들기 → `site_events` 의 DISASTER_SIGN → 관제 화면 모드. 조정기는 이 시험에서 띄우지 않았다.

## 6. 역할별 준비 점검 (#156)

```text
$ python3 scripts/field/preflight_role.py --role operations --profile <FIELD>/profile.json \
    --field config/field/search_area.field.json --webcam-params <FIELD>/webcam_node.yaml --json <RUN>/preflight_operations.json
exit 2: FAILED 있음
PASSED  ros.distro — jazzy
PASSED  ros.workspace — rescue_bringup 설치 확인
PASSED  profile.load — <FIELD>/profile.json deployment_id=review_example_v2
PASSED  profile.map_basis — MapBasis(frame_id='map', map_session_id='22222222-2222-4222-8222-222222222222', map_revision=0, transform_revision=1)
PASSED  profile.role — operations, camera
FAILED  env.ROS_DOMAIN_ID — 지금 '6', 프로파일 7
FAILED  env.ROS_DISCOVERY_SERVER — 지금 '<DISCOVERY_IP>:11811;' (자리=server id), 프로파일 [(0, '192.0.2.13', 11811), (1, '192.0.2.16', 11811)]; ROS_AUTOMATIC_DISCOVERY_RANGE='SUBNET'. 서버 응답은 preflight.py 로 본다
PASSED  module.rclpy — import 가능
PASSED  module.rescue_interfaces — import 가능
PASSED  module.rescue_operations.ros_node — import 가능
PASSED  module.rescue_operations.report_node — import 가능
PASSED  module.rescue_console.app — import 가능
PASSED  module.flask — import 가능
PASSED  module.rescue_perception.webcam_node — import 가능
PASSED  module.cv2 — import 가능
PASSED  module.ultralytics — import 가능
PASSED  webcam.params — 설치된 webcam_node 키 51개 모두 있음; 시계 키 ['clock_backend', 'clock_domain_id', 'clock_drift_ppm', 'clock_max_age_ns', 'clock_poll_period_s']
PASSED  webcam.model — ... sha256=8b61f691f17a538410c80d6d8ef7aad7ad80b77365f76a9963e6164e797f18d5, 설정=8b61f691f17a538410c80d6d8ef7aad7ad80b77365f76a9963e6164e797f18d5
PASSED  clock.timesyncd — webcam yaml: state=SYNCED error_bound_ns=309428162

$ python3 scripts/field/preflight_role.py --role coordination ...   (같은 인자)
exit 2: FAILED 있음
PASSED  ros.distro / ros.workspace / profile.load / profile.map_basis
PASSED  profile.role — coordination
FAILED  env.ROS_DOMAIN_ID — 지금 '6', 프로파일 7
FAILED  env.ROS_DISCOVERY_SERVER — (위와 같음)
PASSED  module.rclpy / module.rescue_interfaces / module.rescue_coordination.ros_node — import 가능
PASSED  field.load — config/field/search_area.field.json
PASSED  field.coordinator — 프로파일·현장 파일로 조정기 파라미터를 만들 수 있음 (경로·관측 목표 인자는 따로)
PASSED  clock.timesyncd — field coordinator: state=SYNCED error_bound_ns=309437401
```

FAILED 2개는 시험 프로파일이 예시(도메인 7, 문서용 주소 `192.0.2.x`)라서 난다. 현장 프로파일이 생기면 다시 돌린다. 결과 JSON 은 현장 폴더에 남겼다(저장소에 넣지 않음: 개인 경로 포함).

## 막힌 것과 필요한 결정

1. **와이파이 망(재범):** 이 PC 는 로봇망과 다른 SSID 라 로봇 Pi·Discovery 서버에 닿지 않는다. 로봇망으로 옮길지. 옮기면 주소가 바뀌므로 ping·절전·preflight 를 다시 잰다.
2. **와이파이 절전(재범):** on. 끄는 명령은 작전이 재범에게 따로 전달했다.
3. **현장 프로파일(재범·통합):** 실제 namespace(`/robot6`·`/robot5`)·도메인·Discovery 주소·`site_cameras[].source` 를 담은 프로파일이 아직 없다. 지금 시험은 예시 프로파일이다.
4. **눈 확인(재범):** 웹캠 흔들기·가림, Firefox 화면.
5. **TB-6 Pi 주소:** 받으면 ping.
6. `iw` 미설치. `iwconfig` 로 대신 읽었으므로 설치는 필요 없다.

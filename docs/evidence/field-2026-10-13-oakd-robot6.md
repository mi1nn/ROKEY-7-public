# TB-6 OAK-D 카메라 설정 확인 기록 (/robot6/oakd, 2026-10-13)

- 요청: 작전(삼성)-R7/1007 → 실기-기록 (사용자 배정). `amr_target` 실행 인자(`rgb/depth_stamp_semantics`, `rgb_depth_alignment_verified`, `rgb_rectified`, `calibration_id`)를 채울 근거.
- 기준: main `d2648ea`. 측정 12:38–12:44 KST. TB-5 담당 PC 호스트 셸, ROS Jazzy, `rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=6`.
- Discovery: 이 측정 프로세스의 환경변수만 TB-5·TB-6 Pi 서버(ID 5·6) 둘로 바꿨다. `/etc` 파일, 로봇 설정, 카메라 노드는 바꾸지 않았다. 로봇은 움직이지 않았다.
- 원문 출력: [field-2026-10-13-oakd-robot6.log](field-2026-10-13-oakd-robot6.log). 주소는 `<TB5_PI>`·`<TB6_PI>` 로 가렸다.
- 표기: TB-6 = `/robot6` (robot_a, SCOUT), TB-5 = `/robot5` (robot_b, SUPPORT).

## 요약

| 항목 | 결과 | 근거 |
| --- | --- | --- |
| Depth 를 내는 실행 방법 | 지금 떠 있는 카메라 노드 `/robot6/oakd` 가 `camera.i_pipeline_type: RGBD` 로 Depth 를 낸다. Depth 토픽은 `/robot6/oakd/stereo/image_raw`. 띄운 명령·설정 파일 경로는 [미확인] | [측정] param dump |
| RGB·Depth 정렬 | 정렬됨. `stereo.i_align_depth: true`, `stereo.i_board_socket_id: 0`(RGB 소켓). 두 CameraInfo 의 frame_id·해상도·K·P 가 같다 | [측정] |
| 왜곡 보정(rectified) | **RGB 는 보정 안 됨.** `/robot6/oakd/rgb/camera_info` D 가 0 이 아니다(`rational_polynomial`, 8개). rect 토픽 없음 | [측정] |
| 시각 기준 | 장치 촬영 시각을 호스트 ROS 시각으로 옮긴 값. `i_get_base_device_timestamp: false` → 드라이버가 `getTimestamp()`(호스트와 맞춘 장치 시각)을 쓴다 | [측정] 파라미터 + [문서] depthai-ros 소스. Pi 에 설치된 드라이버 버전은 [미확인] |
| RGB·Depth stamp 차이 | 같은 stamp 는 0/348. 가장 가까운 RGB 와의 차이 중앙값 2.891 ms, 최소 0.077 ms, 최대 138.667 ms | [측정] 40 s 구독 |
| calibration 식별자 | 카메라 쪽 식별자는 없다(`i_calibration_file: ''`, `i_external_calibration_path: ''` → 장치 내장 보정). 깊이 축척은 저장소 `robot6_depth.json` 의 `calibration_id: robot6_depth` | [측정] + [코드] |
| Pi discovery 설정 | [미확인]. ssh 키 인증이 안 돼 읽지 못했다. 비밀번호는 쓰지 않았다 | [측정] |

## 1. 카메라 노드와 실행 설정 [측정 12:38–12:39]

- 노드: `/robot6/oakd`, `/robot6/oakd_container`. depthai-ros 드라이버의 파라미터 이름(`camera.i_*`, `stereo.i_*`)이다.
- 저장소에는 OAK-D 실행 설정 파일이 없다. #129 는 `robot6_depth.json`(깊이 축척), #126 은 클릭 확인 도구만 더했다. [depth-click-calibration.md](../implementation/depth-click-calibration.md) 는 `/robot6/oakd/stereo/image_raw` 를 썼다.
> [!WARNING]
> 원문 미확인 (전달: 작전 자리, 2026-10-13): TurtleBot4 기본 `turtlebot4_bringup/config/oakd_pro.yaml`(jazzy)은 `i_pipeline_type: RGB` 다. 이 측정 PC 에는 `turtlebot4_bringup` 이 설치돼 있지 않아 직접 보지 못했다.

- 지금 TB-6 는 `RGBD` 로 떠 있다 [측정]. 어떤 launch·yaml 로 띄웠는지는 Pi 의 프로세스를 읽어야 안다 [미확인].

`ros2 param dump /robot6/oakd` 중 관련 값:

```text
camera.i_pipeline_type: RGBD
camera.i_enable_sync: true
camera.i_external_calibration_path: ''
camera.i_tf_camera_name: oakd
rgb.i_resolution: 1080P   rgb.i_set_isp_scale: true (2/3)   rgb.i_width: 704  rgb.i_height: 704
rgb.i_fps: 30.0   rgb.i_low_bandwidth: true   rgb.i_synced: false
rgb.i_get_base_device_timestamp: false   rgb.i_update_ros_base_time_on_ros_msg: false
rgb.i_calibration_file: ''
left/right.i_resolution: 720P   left/right.i_fps: 10.0
stereo.i_align_depth: true   stereo.i_board_socket_id: 0   stereo.i_socket_name: rgb
stereo.i_width: 704   stereo.i_height: 704   stereo.i_subpixel: true   stereo.i_lr_check: true
stereo.i_depth_preset: HIGH_ACCURACY   stereo.i_enable_distortion_correction: false
stereo.i_get_base_device_timestamp: false   stereo.i_synced: false
```

## 2. 토픽과 메시지 [측정 12:43:36, 40 s 구독, best_effort]

| 토픽 | frame_id | 크기 | 받은 수 / 주기(stamp 기준) | 형식 |
| --- | --- | --- | --- | --- |
| `/robot6/oakd/rgb/camera_info` | `oakd_rgb_camera_optical_frame` | 704×704 | 1134 / 29.66 Hz | `rational_polynomial` |
| `/robot6/oakd/stereo/camera_info` | `oakd_rgb_camera_optical_frame` | 704×704 | 348 / 9.25 Hz | `rational_polynomial` |
| `/robot6/oakd/rgb/image_raw/compressed` | `oakd_rgb_camera_optical_frame` | — | 1095 / 28.67 Hz | `bgr8; jpeg compressed bgr8` |
| `/robot6/oakd/stereo/image_raw/compressedDepth` | `oakd_rgb_camera_optical_frame` | — | 179 / 4.76 Hz | `16UC1; compressedDepth png` |

- Raw `/robot6/oakd/rgb/image_raw`·`/robot6/oakd/stereo/image_raw` 는 Wi-Fi 부하 때문에 구독하지 않았다. 같은 헤더를 쓰는 CameraInfo·압축 토픽으로 frame_id 와 stamp 를 읽었다. Raw Depth 인코딩은 compressedDepth format 으로 `16UC1` 이다.
- 12:38 토픽 목록에는 `rgb/image_raw`·`camera_info` 가 안 보였다. 구독하자 메시지는 들어왔다. 드라이버가 `i_enable_lazy_publisher: true` 라서 구독자가 있을 때만 내는 것으로 보인다 [추정].
- 받은 시각 − stamp(이 PC 시계 기준): RGB 정보 −2.7 ms, RGB 압축 10.1 ms, Depth 정보 776.9 ms, Depth 압축 776.2 ms. 두 시계 차이는 재지 않았으므로 참고값이다. Depth 가 약 0.78 s 늦게 도착한다.

## 3. 정렬

- 두 CameraInfo 의 frame_id(`oakd_rgb_camera_optical_frame`), 크기(704×704), K·P 가 같다. Depth 는 RGB 카메라 기준으로 정렬돼 나온다 (`stereo.i_align_depth: true`, `stereo.i_board_socket_id: 0`).
- `rgb_optical_frame` = `depth_optical_frame` = `oakd_rgb_camera_optical_frame` 로 넣으면 된다.
- 정렬이 맞는지 영상으로 겹쳐 본 시험(가장자리 일치 등)은 하지 않았다 [미확인].

## 4. 왜곡 보정

- RGB CameraInfo D = `[8.7523, -106.5924, -0.002685, -0.001596, 506.5583, 8.4794, -104.1678, 495.6710]`. 0 이 아니다. R 은 단위행렬, P 의 초점·중심은 K 와 같다.
- Depth CameraInfo D 는 8개 모두 0.0 이다. K 는 RGB 와 같다.
- rect 토픽(`image_rect` 등)은 없다. 따라서 `/robot6/oakd/rgb/image_raw` 는 **보정되지 않은 영상**이다. `rgb_rectified=True` 로 넣을 근거가 없다.
- Depth 를 보정 안 된 RGB 에 맞췄는지, 보정된 RGB 기준인지는 드라이버 문서로 확인하지 않았다 [미확인].

## 5. 시각 기준

- 파라미터: `rgb`·`stereo` 모두 `i_get_base_device_timestamp: false`, `i_update_ros_base_time_on_ros_msg: false`. 노출 보정 `i_add_exposure_offset: false`.
- depthai-ros `jazzy` 브랜치 `depthai_bridge/src/ImageConverter.cpp` [문서]: `header.stamp = getFrameTime(rosBaseTime, steadyBaseTime, tstamp)`. `getBaseDeviceTimestamp` 가 false 면 `tstamp = inData->getTimestamp()` 다. 즉 장치가 찍은 프레임 시각(호스트 steady clock 에 맞춘 값)을 노드 시작 때 잡은 ROS 기준 시각으로 옮긴다. 드라이버가 받은 시각이 아니라 **촬영 시각**이다. Pi 에 설치된 depthai-ros 버전이 이 브랜치와 같은지는 [미확인].
- stamp 값은 벽시계 기준이다(마지막 stamp 1791863016 s = 2026-10-13 12:43 KST).
- RGB 30 Hz 와 Depth 약 9 Hz 는 서로 다른 센서라 stamp 가 정확히 같지 않다 (`rgb.i_synced: false`). `amr_target` 이 `sync_slop_ns=20000000`(20 ms) 근사 동기화를 쓰면 중앙값 2.9 ms 는 들어간다. 최대 138.7 ms 는 RGB 프레임이 빠진 구간으로 보인다 [추정].

## 6. Pi discovery 설정

- TB-5·TB-6 Pi 에 `ssh -o BatchMode=yes ubuntu@<Pi>` 로 `grep ROS_DISCOVERY_SERVER /etc/turtlebot4/setup.bash` 와 `ps -eo pid,args | grep depthai` 를 시도했다. 둘 다 `Permission denied (publickey,password)`. 비밀번호는 쓰지 않았다. [미확인]
- 간접 근거 [측정 11:02–11:03, field-2026-10-13-situation.log]: 두 Pi 모두 11811 포트에 Discovery Server 가 응답하고, 서버 ID 5·6 슬롯으로 붙으면 그래프가 보인다.

## 7. 도킹 상태 재측정 [측정 12:42]

- 11:03 기록은 `/robot5` 도킹(배터리 1.0), `/robot6` 언도킹(0.82)이었다. 사용자는 그 뒤 "TB-5 언독, TB-6 독"이었다가 "방금 TB-6 도 언독"했다고 전했다.
- 12:42 측정: `/robot5` `is_docked: False`, 0.89 / `/robot6` `is_docked: False`, 0.84. 사용자 전달과 맞는다.
- 12:38 기준 TB-6 에는 `slam_toolbox`, Nav2 서버들, `teleop_twist_joy_node`, `rviz2` 노드가 떠 있었다. 다른 사람이 쓰고 있는 것으로 보인다 [추정].

## amr_target 에 넣을 값 (후보)

| 인자 | 후보 | 상태 |
| --- | --- | --- |
| `image_topic` | `/robot6/oakd/rgb/image_raw` | [측정] |
| `depth_topic` | `/robot6/oakd/stereo/image_raw` | [측정] |
| `camera_info_topic` | `/robot6/oakd/rgb/camera_info` | [측정] |
| `rgb_optical_frame`·`depth_optical_frame` | `oakd_rgb_camera_optical_frame` | [측정] |
| `rgb_stamp_semantics`·`depth_stamp_semantics` | `CAPTURE` | [측정] 파라미터 + [문서] 드라이버 소스 |
| `rgb_depth_alignment_verified` | `True` 후보. frame_id·크기·K 일치와 `i_align_depth: true` 까지 확인. 겹쳐 보기 시험은 안 함 | 결정 필요 |
| `rgb_rectified` | `False`. RGB D 가 0 이 아니다 | [측정] |
| `calibration_id` | 카메라 쪽 없음(장치 내장 보정). 깊이 축척은 `robot6_depth` | 결정 필요 |

`core.py:307` 은 `rectified` 가 False 면 위치를 붙이지 않는다(`ALIGNMENT_UNVERIFIED`). 따라서 지금 TB-6 카메라 설정으로는 RGB 보정 문제가 남는다. 방법은 세 가지다: (1) 호스트에서 `image_proc` 등으로 보정한 RGB 를 쓰기, (2) 카메라 설정을 바꿔 보정된 RGB 를 내기, (3) 보정 안 된 영상을 받아들이도록 코드·정책 바꾸기. (2) 는 로봇 설정 변경(등급 4)이고, (3) 은 코드 결정이다. 이 기록에서는 고르지 않는다.

## 8. Wi-Fi [측정 12:47–12:48]

TB-5 담당 PC 에서 `ping -c 50 -i 0.2` 로 두 Pi 를 쟀다. 원문은 로그 G절에 있다.

| 대상 | 손실 | 최소 / 평균 / 최대 / 편차 (ms) |
| --- | --- | --- |
| TB-5 Pi | 0/50 | 2.069 / 11.714 / 52.309 / 9.545 |
| TB-6 Pi | 0/50 | 3.226 / 35.053 / 404.210 / 68.987 |

- 이 PC 의 power_save: **켜짐으로 보인다.** `iw` 가 설치돼 있지 않아 드라이버 상태를 직접 읽지 못했다. 대신 NetworkManager 설정을 읽었다.
  - 활성 Wi-Fi 연결의 `802-11-wireless.powersave` 는 `default` 다(전역 기본값을 따른다는 뜻).
  - 전역 기본값은 `wifi.powersave = 3`(켬)이다. 따라서 실제로 켜져 있을 것으로 본다 [추정].
- 이 PC 링크: 채널 40(5 GHz), 270 Mbit/s, 신호 84(nmcli), level −54 dBm, missed beacon 149.
- Pi 쪽 power_save 는 [미확인]. ssh 키 인증이 거부됐다(6절).
- TB-6 는 평균·최대 지연이 TB-5 보다 크다. 측정 때 TB-6 에서 SLAM·Nav2·rviz2 가 돌고 있었으므로, 그 부하나 무선 상태 탓일 수 있다 [추정]. 원인을 가르는 측정은 하지 않았다.

# rescue_perception

기록된 탐지 결과와 RGB/Depth 메타데이터·Depth 배열을 검증하는 순수 Python 코어다. 표준 라이브러리와 저장소 내부 `rescue_contracts`에 의존한다. `adapter`가 ROS 메시지 모양의 입력(비압축 Image·CameraInfo·Detection2D)을 이 코어로 잇는다. 모델 추론, ROS 노드, 카메라 구독, 압축 영상 해제, TF, 주행 명령은 구현하지 않았다.

- `locate_detection`: 같은 시간 기준·시각 의미·해상도·보정/정렬 근거를 확인하고, bbox ROI의 유효 깊이 중앙값을 rectified RGB optical frame의 XYZ(m)로 역투영한다. `16UC1/mm`, `32FC1/m`만 지원한다. 깊이 부족/불연속/정합 불명은 이유와 함께 위치 없는 결과다.
- `make_observation`: source/boot/원 stamp·모델 ID/SHA/class·탐지·위치 결과·증거 참조·최초 만료를 불변 객체로 만든다. 탐지 성공과 위치 성공을 분리한다. 모델 가중치와 원본 증거 파일의 해시는 caller가 계산·검증해야 한다.
- `person_pipeline.PersonPipeline`: 로봇 카메라 한 스트림에서 탐지기(`Detector`, 모델은 Q-18 전까지 미정) 결과를 RGB-D 위치와 `Observation`으로 바꾸고, `ConfirmationPolicy`(Q-21)로 CONFIRMED/UNCONFIRMED와 이유를 붙인다. 같은 track은 처음 한 번과 CONFIRMED로 오를 때만 새 후보로 표시한다. 녹화된 탐지는 `RecordedDetections`로 재생한다.
- `Detection.tracking_mode`: `DETECTION_ONLY`/`TRACKING`/`TRACKING_FALLBACK`을 생략할 수 없다. TRACKING만 track_id를 가지며 mode도 원 관측 payload hash에 포함한다.
- `validate_freshness`, `ObservationRegistry.ingest`: clock 오차·센서 stamp 지연 상한을 포함해 최신성을 검사한다. 중복·같은 frame의 추가 사람·역순 관측은 공백 타이머를 갱신하지 않는다. 같은 observation ID의 payload 변경은 거절한다.
- `disaster_sign.DisasterSignDetector`: 고정 웹캠 영상에서 화면 전체가 같은 방향으로 움직였는지 블록 매칭(ZNCC)으로 재고, 시간 창 안 방향 전환 횟수가 정책 값을 넘으면 `SiteEvent(DISASTER_SIGN)`를 하나 낸다. 가림·매칭 실패는 UNKNOWN. 모드 전환은 하지 않는다.
- `target_watch.TargetWatchDetector`: 고정 웹캠 감시 영역(ROI) 안의 인형 유무로 `SiteEvent(TARGET_DISAPPEARED_CANDIDATE)`를 watch 하나에 하나 낸다. 가림·영상 불명·탐지기 결과 없음은 UNKNOWN 으로 후보를 만들지 않는다(운영자 판단, Q-26·C-11).
- `webcam_node`(ROS 경계, `ros2 run rescue_perception webcam_node`): 고정 웹캠 1대에서 `CameraStatus`·`SiteEvent`·`TargetPresence` 를 내고 `SetCameraWatch` 를 받는다. 판정 경계는 `webcam_service`. 파라미터 예시 `rescue_perception/config/webcam_node.example.yaml`(기본값 없음).
- `ObservationRegistry.history()`: ACCEPTED와 REORDERED의 불변 수신 이력을 반환한다. 역순 발견을 지우지 않으며, 이력 자체가 신규 사건·자동 접근 허가는 아니다.
- `adapter.observe`: RGB·Depth·CameraInfo·탐지 한 묶음 → `LocalizationResult` (+ `Observation`). 헤더 stamp는 `ros_time_to_ns`로만 옮기고, 탐지 헤더가 RGB와 1 ns라도 다르면 거절한다. 소스·stamp 의미·정렬 근거는 `SourceBinding`/`AlignmentEvidence`로 밖에서 넣는다. 로봇이 아닌 소스(`source_robot=None`)는 위치까지만 만들고 관측은 만들지 않는다.
- `ConservativeEventRegistry.register`: 유효 수신의 admission으로 잠정 사건을 등록한다. 다른 로봇의 같은 track ID, 같은 class/높은 confidence를 같은 사람으로 병합하지 않는다. 모호한 연결은 별도 사건의 후보 관계로만 남긴다.

정합·최신성·깊이 품질의 모든 임계값은 caller가 명시한다. 시험의 숫자는 하드웨어 기본값이 아니다. 위치 결과는 공분산·지도 변환·SLAM revision·대상 동일성·안전 접근이 검증됐다는 뜻이 아니다. 시간 불명 관측은 기록할 수 있지만 자동 임무의 유효 입력으로 admission하지 않는다.

관측·만료·발행·현재 wall 시각은 양수 int64 ns다. duration·시계 오차·Depth 동기 허용치는 0 이상 int64 ns이며, 로컬 `received_steady`만 float 초다. 원 ROS 헤더는 `rescue_contracts.ros_time_to_ns(sec, nanosec)`로 옮기고 float epoch 초로 경유하지 않는다. `PerceptionError`는 공통 `RescueError`를 상속하며 canonical `.code`와 `rescue_perception.<원인>`인 `.detail_code`를 함께 제공한다. 위치 실패는 `.reason` 상태를 보존하면서 `.error_code`/`.detail_code`도 제공한다.

DepthFrame의 전체 픽셀 검사와 ROI list는 E2 기록 입력용이다. 실시간 승급 전에는 ROI 원점·intrinsics·encoding 검증을 보존하는 crop/array 어댑터와 두 영상 동시 입력의 지연·메모리 측정이 필요하다.

실제 실행은 로봇 제어용 컴퓨터의 저장소 루트에서 준비한다:

```sh
PYTHONPATH=src/rescue_contracts:src/rescue_perception python3 -m unittest discover -s tests -p 'test_perception*.py' -v
```

API·제한·원본 근거는 [구현 문서](../../docs/implementation/implementation-perception.md)를 따른다. 이 상대 링크는 저장소 checkout 기준이며 설치된 share 디렉터리는 저장소 문서를 포함하지 않는다.

원본 코드를 복사하지 않고 선별한 원리를 새 계약에 맞춰 구현했다. 버전 `0.0.0`, `LicenseRef-Unspecified`, 임시 maintainer는 기존 초기 구조의 미결정 메타데이터다. 이는 사용 허가나 실제 담당자를 뜻하지 않으며 배포 전에 확정해야 한다.

[OAK-D 학습 계획](../../docs/implementation/oakd-survivor-training-plan.md): 사용자 결정(2026-10-11)에 따라 이번 학습은 `oakd/image_raw` 영상의 1초 간격 프레임을 전처리해 단일 클래스 `survivor`(ID 0)를 사용한다. 후속 연동의 모델 class metadata·manifest·`ConfirmationPolicy.person_labels`·Detector 출력 label을 맞춘다. 기존 API 이름과 과거 person 합성 시험은 유지한다. 학습·실제 카메라 시험은 아직 NOT_RUN이다.


## AMR 단위 검증 노드 (2026-10-12)

[사용자 요구] 기존 인식 코어에 AMR 알고리즘을 연결하고 amr_camera·amr_target을 각각 시험·보정한 뒤 기존 코드의 후속 수정·통합을 진행한다. 이번 PR은 Draft이며 실제 로봇 인수 완료를 뜻하지 않는다.

| 파일 | 목적 |
| --- | --- |
| [amr_camera.py](rescue_perception/amr_camera.py) | RGB raw/compressed → YOLO 1회 → 원 헤더를 보존한 annotated Image. ROS·영상·모델 의존성은 파일 상단에서 import한다 |
| [amr_target.py](rescue_perception/amr_target.py) | RGB·Depth·CameraInfo → TargetObservation(카메라 기준), 유효한 camera→base TF가 있으면 PointStamped(로봇 기준). ROS 의존성은 상단 import |
| [camera_utils.py](rescue_perception/camera_utils.py) | 공통 SETTINGS, 모델 SHA/class·RGB 형식 검사, 복호화·추론·annotated Image 생성 |
| [target_utils.py](rescue_perception/target_utils.py) | RGB-D SETTINGS·target_config, YOLO 관측 메시지, Depth 복호화·robot6 보정, 카메라→base 상대 좌표 |
| [map_projection.py](rescue_perception/map_projection.py) | 지도 상태·정렬·관측 시각 TF 검사, 지도/mission 좌표·공분산 변환. 코드와 합성 시험만 보존하며 실행 노드에 연결하지 않음 |
| [target_location.py](rescue_perception/target_location.py) | 확정 TargetObservation + 촬영 시각 TF map←camera → TargetLocation(VALID) 좌표·공분산·frame_ref. ROS 없음 |
| [target_locator.py](rescue_perception/target_locator.py) | 위 변환의 ROS 노드: `/rescue/observations` 구독 → 촬영 시각 TF 조회 → `/rescue/target_locations` 발행 |
| [exit_qr.py](rescue_perception/exit_qr.py) | 출구 QR 판정(내용 정확 일치·N 프레임 연속·위치 허용 범위·깊이)과 출구 TargetObservation 채우기. ROS 없음, amr_target 이 프레임마다 부름 |
| [headroom.py](rescue_perception/headroom.py) | TB-5 머리 위 여유 실기 반복 판정 기록 도구 `headroom_record`(OPS-115). 판정·발행·정지 명령 없음 |
| [robot6_depth.json](rescue_perception/config/robot6_depth.json) | robot6 실행용 보정계수·offset·로봇/보정 ID·입력 단위/상태. 설치본에도 포함 |

[사용자 요구, 2026-10-12] AMR 환경별 YAML·projection template와 필수 ROS 파라미터 선언/읽기 코드를 제거하고 기존 utils의 SETTINGS에 필수 설정을 모았다. 노드는 설정을 가져와 그대로 실행한다. 기존 main의 웹캠·bag 코드와 웹캠 설정은 유지한다.

[사용자 요구, 2026-10-12] 실제 실행 대상은 README 시나리오의 로봇 제어용 컴퓨터다. 기준 환경은 MSI RTX 4070 노트북·Ubuntu 24.04·ROS 2 Jazzy다. 이 저장소에서는 베이스와 테스트 코드를 작성하며 작성 환경에서 ROS 노드를 실행하는 것을 전제하지 않는다. ROS가 없는 환경에서 노드 import를 허용하기 위한 함수 내부 import와 의존성 오류 호환 분기를 제거했다. 순수 설정·보정·검사 시험은 utils를 직접 import한다.

[설정/사용자 요구, 2026-10-12] 첫 CUDA GPU `device: '0'`, 탐지 신뢰도 `model_conf: 0.5`, 기존 대상 확정 정책 `confirm_confidence_min: 0.8`, `physical_robot_id: robot6`, `depth_calibration_path: package:robot6_depth.json`. 그 외 토픽·frame 이름과 최신성·깊이 임계값은 이전 예시의 [제안]이며 실기 검증 전이다. clock/stamp/alignment는 UNKNOWN·미확인을 유지한다. 코드 통합은 이 값을 측정 사실로 바꾸지 않는다.

[사용자 요구, 2026-10-12] robot6_depth.json은 실행에 쓰는 6개 필드만 유지하고 출처·날짜·측정 기록·설명은 제거했다. 로더는 로봇 일치·m 단위 raw 입력·유효한 scale/offset을 검사한다. RGB는 유지하고 Depth를 m로 변환한 뒤 원 0·음수·NaN·Inf를 제외한다. raw_m * scale + offset_m을 한 번 적용하고 범위·불연속 검사·중앙값으로 optical XYZ를 계산한다. raw/PNG compressedDepth, 16UC1/mm·32FC1/m 모두 같은 보정을 사용한다. 다른 physical_robot_id는 보정 로드 시 거절하며 이미 보정된 입력은 사용하지 않는다.

### 빌드와 실행

ROS·OpenCV·cv_bridge·Ultralytics와 CUDA 사용 가능한 추론 환경이 필요하다. 모델은 [선택 manifest](../../assets/manifest/yolo26n-survivor-v2.json)의 best.pt다. 실제 실행은 로봇 제어용 컴퓨터의 저장소 루트에서 준비한다:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rescue_contracts rescue_mapping rescue_interfaces rescue_perception \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 run rescue_perception amr_camera --ros-args -p image_topic:=<RGB 토픽>
# camera 노드를 종료한 뒤 target 노드를 따로 시험한다. launch 가 토픽·frame·TF remap 을 넘긴다.
ros2 launch rescue_bringup amr_perception.launch.py image_topic:=<RGB> depth_topic:=<Depth> \
  camera_info_topic:=<CameraInfo> rgb_optical_frame:=<RGB frame_id> depth_optical_frame:=<Depth frame_id> \
  tf_topic:=/robot6/tf tf_static_topic:=/robot6/tf_static \
  rgb_stamp_semantics:=<…> depth_stamp_semantics:=<…> rgb_depth_alignment_verified:=<true|false> \
  rgb_rectified:=<true|false> calibration_id:=<…> alignment_evidence_id:=<…> \
  profile_path:=<배포 프로파일.json> tf_wait_s:=1.0
```

TB-6 현장 값 예(2026-10-13). 측정한 값만 넣었고, 나머지는 현장에서 확인해 넣는다:

```text
sensor_host_clock_error_bound_ns:=150000000 sensor_host_clock_domain_id:=rokey7-utc-field
  # [측정, 재범 화면 14:31~14:34, TB-6 Pi chronyc tracking] System time 0.257 ms, Root delay 7.08 ms,
  # Root dispersion 2.18 ms, Update interval 1033.5 s -> parse_chrony 식(drift 100 ppm x 갱신 간격) 약 109 ms, 여유 150 ms.
  # TB-6·TB-5 Pi 는 chrony 를 쓴다(timesyncd 아님). clock_backend 는 amr_target 이 도는 PC 의 것이다.
clock_domain_id:=rokey7-utc-field   # #132·#146 과 같은 현장 시계 영역
```

같은 launch 가 target_locator 도 띄운다. 시계 영역은 amr_target 과 같은 `clock_domain_id` 를 쓴다. 정렬 권한 `alignment_authority_id`·`alignment_valid_from_ns`·`alignment_valid_until_ns`(OPS-103, 기본값 없음)는 실행 인자다. 권한 출처는 프로파일 `map_basis` 와 TB-6 의 saved-map 선언(#132)이다. 창 밖이면 target_locator 가 VALID 를 내지 않는다. 지도 기준(map session·revision·transform revision·mission frame)과 SCOUT 로봇 ID·지도 frame 은 배포 프로파일의 `map_basis` 블록과 로봇 readiness 에서 읽는다. 조정기도 같은 블록을 읽는다(작전 결정 2026-10-13). 블록이 없으면 launch 가 멈춘다. 프로파일에 map → mission 변환이 없으므로, 로봇 지도 frame 과 `map_basis.frame_id` 가 같을 때만 identity 로 두고 다르면 멈춘다.

[작전 B1, 2026-10-13] 카메라 토픽(`image_topic`·`depth_topic`·`camera_info_topic`)과 광학 frame(`rgb_optical_frame`·`depth_optical_frame`)은 코드 기본값이 없는 STRING 파라미터다. 전에는 코드 기본값 `camera/...`, readiness 설정 `oakd/rgb/preview/...`, #132 현장 스크립트 `/robot6/oakd/rgb/...` 세 가지가 서로 달랐다. 값이 없거나 비어 있으면 노드가 시작하지 않는다. 실제 robot6 토픽·frame 이름은 [미확인]이며 로봇에서 `ros2 topic list`와 메시지 header 로 확인해 넣는다. 위치 계산 근거도 같은 방식이다(2026-10-13). `rgb_stamp_semantics`·`depth_stamp_semantics`(CAPTURE·DRIVER_RECEIVE·UNKNOWN), `rgb_depth_alignment_verified`·`rgb_rectified`(bool), `calibration_id`·`alignment_evidence_id` 는 기본값이 없다. 전에는 코드 값 UNKNOWN·False 라서 `core.locate_detection` 이 모든 탐지의 위치를 거절했다(CLOCK_UNCERTAIN·ALIGNMENT_UNVERIFIED). 그래서 확정 관측이 한 번도 나가지 않았다. 두 stamp 의미가 알려져 있고(UNKNOWN 아님) 서로 같으며, 정렬이 검증됐고 RGB 가 rectified 일 때만 위치가 나온다. 값은 로봇 카메라 설정에서 확인해 넣는다 [미확인]. RGB 가 rectified 가 아니어도 CameraInfo 의 왜곡 모델이 plumb_bob(5)·rational_polynomial(8)이면 위치를 낸다. 이때 탐지 상자의 모서리·중심을 K·D 로 왜곡 보정해(cv2.undistortPoints, P 로 재투영) 정렬된 Depth 에서 읽는다. 이미지 전체는 보정하지 않는다. 근거(`<model>:<sha256(D) 앞 12자리>`)는 `LocalizationResult.undistortion_id` 와 SENT 로그에 남는다. 다른 모델이면 지금처럼 ALIGNMENT_UNVERIFIED 다. 입력 형식 `image_type`(raw·compressed)·`depth_transport`(raw·compressedDepth)도 현장 값이다(TB-6 Depth 는 raw 16UC1 로 측정, 옛 코드 값 compressedDepth). 짝 맞추기 `sync_queue_size`·`sync_slop_ns`·`camera_info_max_age_ns` 도 기본값 없는 현장 값이다. TB-6 은 Depth 가 약 0.78 s 늦게 와서, 옛 코드 값 큐 3 으로는 짝이 하나도 만들어지지 않았다(message_filters 재현, 2026-10-13). 나머지 설정은 SETTINGS 그대로이고 YAML은 필요 없다. 모델 경로는 실행 작업 디렉터리 기준이다. GUI는 SETTINGS의 show=True에서만 켠다. GPU 추론 실패를 CPU 추론으로 숨기지 않는다.

RGB·Depth는 원 stamp로 동기화한다. CameraInfo frame/해상도·최신성, 모델 SHA/class, Depth 유효값·정렬·시각 검사는 유지한다. [사용자 요구, 2026-10-12] 지도 변환은 map_projection.py로 분리해 코드로만 유지하고, 기존 rescue_mapping과의 연결은 후속으로 진행한다. amr_target은 원 카메라 기준 TargetObservation과 annotated Image를 발행하고, 유효한 카메라→base 변환이 있으면 로봇 기준 PointStamped를 추가 발행한다. amr_target 자신은 map/odom TF 를 조회하지 않고 TargetLocation 도 내지 않는다. 지도 좌표는 별도 노드 target_locator 가 만든다(아래 「지도 좌표 TargetLocation」). 별도 지도 변환 모듈에도 ROS 노드·실행 등록은 없다. 후속 연결에서는 기존 mapping의 fresh MapState·clock·alignment·관측 시각 TF·공분산 근거를 제공하고 검사해야 한다. 현재 합성 함수 시험은 실제 mapping 연동을 증명하지 않는다. 주행 명령은 보내지 않는다.

### TB-6 현장 실행 한 줄과 원인 가르기 (2026-10-13)

TB-6 담당 PC 에서 저장소 루트(모델 경로 `assets/models/best.pt` 가 실행 위치 기준)에서, 현장 ROS 환경을 source 한 뒤 실행한다. 값 옆 표시: [측정] 오늘 TB-6 에서 잰 값, [설정] 다른 PR 의 현장 설정, [추정] 아직 로봇에서 이름을 확인하지 않은 값.

```bash
from_ns=$(python3 -c 'import time; print(time.time_ns() - 3_000_000_000)')         # 정렬 창: 지금 -3 s ~ +1 h (#132 와 같은 방식)
until_ns=$(python3 -c 'import time; print(time.time_ns() + 3_600_000_000_000)')
ros2 launch rescue_bringup amr_perception.launch.py \
  image_topic:=/robot6/oakd/rgb/image_raw/compressed image_type:=compressed \
  depth_topic:=/robot6/oakd/stereo/image_raw depth_transport:=raw \
  camera_info_topic:=/robot6/oakd/rgb/camera_info \
  rgb_optical_frame:=oakd_rgb_camera_optical_frame depth_optical_frame:=oakd_rgb_camera_optical_frame \
  rgb_stamp_semantics:=CAPTURE depth_stamp_semantics:=CAPTURE \
  rgb_depth_alignment_verified:=true rgb_rectified:=false \
  calibration_id:=tb6-oakd-device-calibration alignment_evidence_id:=tb6-oakd-align-measured-20261013 \
  sync_queue_size:=40 sync_slop_ns:=20000000 camera_info_max_age_ns:=1500000000 \
  tf_topic:=/robot6/tf tf_static_topic:=/robot6/tf_static \
  clock_domain_id:=rokey7-utc-field clock_backend:=timesyncd clock_max_age_ns:=3600000000000 \
  clock_drift_ppm:=100.0 clock_poll_period_s:=5.0 \
  sensor_host_clock_error_bound_ns:=150000000 sensor_host_clock_domain_id:=rokey7-utc-field \
  profile_path:=<현장 배포 프로파일.json> tf_wait_s:=1.0 \
  alignment_authority_id:=tb6-saved-map-identity alignment_valid_from_ns:=$from_ns alignment_valid_until_ns:=$until_ns \
  recheck_action_name:=/rescue/robots/robot_a/observe_target recheck_map_frame:=map recheck_scan_topic:=/robot6/scan \
  recheck_max_window_ns:=10000000000 recheck_validity_ns:=5000000000 \
  recheck_presence_radius_m:=0.5 recheck_lidar_radius_m:=0.3 recheck_min_frames:=3 recheck_min_scans:=3 \
  recheck_min_lidar_rays:=3 recheck_min_lidar_hits:=2 recheck_image_margin_px:=10 \
  recheck_min_depth_m:=0.2 recheck_max_depth_m:=5.0 recheck_occlusion_tolerance_m:=0.3 recheck_depth_window_px:=2
```

| 인자 | 근거 |
| --- | --- |
| `image_topic` | [설정] #132 `camera_topics` 의 `/robot6/oakd/rgb/image_raw/compressed`. 실제 RGB 토픽 이름과 종류는 [추정] — `ros2 topic list \| grep oakd` 로 확인 |
| `depth_topic`·`depth_transport:=raw` | [측정] `/robot6/oakd/stereo/image_raw`, 16UC1 raw |
| `camera_info_topic` | [추정] depthai-ros 이름 규칙. RGB CameraInfo 값 자체는 [측정] |
| 광학 frame·`CAPTURE`·정렬 `true`·`rectified false` | [측정] RGB·Depth CameraInfo frame_id 같음, 704×704, K·P 같음, `i_align_depth=true`. RGB D 는 rational_polynomial(점 보정 경로). stamp 는 depthai-ros 의 장치 촬영 시각 [문서] + 두 stamp 차이 중앙값 2.9 ms [측정]. 겹쳐 보기 시험은 아직 안 함 |
| `sync_*`·`camera_info_max_age_ns` | [측정] Depth 가 약 0.78 s 늦게 옴(RGB 29.7 Hz, Depth 9.3 Hz). 큐 3 으로는 짝 0개(message_filters 재현) |
| `clock_*` | [설정] #132·#146 현장 시계 영역과 backend(TB-6 담당 PC). PC 가 chrony 면 `clock_backend:=chrony` |
| `sensor_host_*` | [측정] TB-6 Pi chronyc tracking → 약 109 ms, 여유 150 ms |
| `profile_path` | 현장 프로파일. `map_basis.frame_id` 가 `map` 이어야 한다(아니면 launch 가 멈춤) |
| 정렬 창·권한 | OPS-103 결정 2. 프로파일 `map_basis` + TB-6 saved-map 선언(#132) |
| `recheck_action_name`·`recheck_scan_topic` | ObserveTarget 은 논리 ID 경로 `/rescue/robots/robot_a/observe_target`(로봇 실행기 #179 가 부름). scan 은 [설정] #132 `scan_topic` |
| `recheck_*` 정책 값 | [제안] 판정 반경 0.5 m·LiDAR 원 0.3 m·최소 프레임·스캔 3·빔 3·원 안 반사 2·깊이 0.2~5 m·가림 0.3 m. 창 상한 10 s 는 #199 의 실행기 재확인 창 3 s·마감 10 s 를 덮는다. 현장에서 바꿀 수 있다 |

**「TB-6 이 인형을 보는데 위치가 안 나옴」 원인 가르기** — amr_target·target_locator 로그를 위에서부터 본다. 처음으로 맞지 않는 단계가 원인이다.

| # | 볼 로그 | 정상 | 아니면 |
| --- | --- | --- | --- |
| 1 | `AMR target startup/runtime failed: missing parameters: [...]` / `amr_target git=... parameters=...` | 기동 줄에 위 값이 그대로 있음 | 빠진 인자·틀린 값(exit 2) |
| 2 | `clock SYNCED backend=... error_bound_ns=...` / `sensor host (camera driver) clock ...` | 기동 뒤 `clock_poll_period_s` 안에 SYNCED | `clock UNKNOWN ... reason=<이유>`: PC 시계 동기 안 됨(`timedatectl`·`chronyc tracking`). `... clock domain ... differs`: 두 영역 값이 다름 |
| 3 | `No successful synchronized RGB-D frame; check both topics, QoS, CameraInfo and frame errors` (5 s 마다) | 이 경고가 없음 | 짝이 안 생김. `ros2 topic hz` 로 RGB·Depth 가 오는지, 큐(`sync_queue_size`)가 Depth 지연을 덮는지 |
| 4 | `AMR target frame failed (no fabricated coordinates): ValueError: fresh matching CameraInfo required` | 없음 | CameraInfo 가 안 오거나 `camera_info_max_age_ns` 보다 오래됨 |
| 5 | `... RGB/Depth optical frames differ from configured alignment evidence` | 없음 | 메시지 header frame_id 와 인자 frame 이 다름 |
| 6 | `target not sent: UNCONFIRMED(<이유>)` | `target state=SENT observation_id=... undistortion=<model>:<hash>` | 이유별: `LOW_CONFIDENCE` 신뢰도 부족(인형이 멀거나 가림) · `CLOCK_UNCERTAIN` stamp 의미 UNKNOWN 이거나 서로 다름 · `UNSYNCHRONIZED` RGB-Depth stamp 차이가 `sync_slop_ns` 초과 · `ALIGNMENT_UNVERIFIED` 정렬 false 이거나 RGB D 모델을 보정 못 함 · `CALIBRATION_MISMATCH` frame·보정 ID 불일치 · `DIMENSION_MISMATCH` RGB·Depth 해상도 다름 · `DEPTH_INSUFFICIENT`·`DEPTH_DISCONTINUITY` 상자 안 Depth 가 비었거나 들쭉날쭉(너무 가깝거나 멂) |
| 7 | `target not sent: BBOX_AT_IMAGE_EDGE` / `INVALID_POSITION` | — | 인형이 화면 가장자리에 걸림 / 위치 계산 실패 |
| 8 | target_locator `refused, not published: <이유>` | `TargetLocation <id> VALID observation=... mission=(x, y, z)` | `CLOCK_NOT_SYNCED`·`NO_CLOCK_ERROR_BOUND`·`STAMP_SEMANTICS_UNKNOWN`·`CLOCK_DOMAIN_MISMATCH` 시계(2단계) · `OBSERVATION_EXPIRED`·`FUTURE_STAMP` 관측 나이·PC 시계 차이 · `ALIGNMENT_NOT_VALID_AT_CAPTURE`·`ALIGNMENT_EXPIRED` 정렬 창 · `COVARIANCE_INVALID` 공분산 |
| 9 | target_locator `no TF map<-oakd_rgb_camera_optical_frame at capture ...` | 없음 | 촬영 시각의 TF 가 없음: `/robot6/tf` 에 map→odom(AMCL)이 오는지, TF remap 이 맞는지 |
| 10 | 관제 화면 사유 `대상 위치의 관측 시각을 쓸 수 없음` 등(조정기 #174) | 대상이 생김 | 조정기 쪽 시계 검사. 조정기 `target_clock_domain_id` 가 같은 값인지 |

### 관측 시계 (OPS-103)

[작전 결정 2026-10-13] amr_target 관측의 시계는 코드 값(`clock_state='UNKNOWN'`·`clock_error_bound_ns=-1`)이 아니라 이 호스트의 측정값이다. 웹캠(#146)과 같은 방식이다(`rescue_contracts.clock_measurement.ClockSampler` + `webcam_clock`). `clock_domain_id`·`clock_backend`(timesyncd·chrony·NONE)·`clock_max_age_ns`·`clock_drift_ppm`·`clock_poll_period_s` 는 기본값 없는 파라미터이고, launch 인자로도 받는다. 프레임마다 그 촬영 시각(RGB stamp)에서 마지막 측정이 SYNCED 이고 유효하면 SYNCED·오차 상한(측정값 + 표본 뒤 drift)이고, 아니면 UNKNOWN 이다. 기동 직후 첫 측정 전과 `clock_backend: NONE` 은 UNKNOWN 이다. OAK-D stamp 는 카메라 드라이버가 도는 호스트(TB-6 Pi)의 시계라서, 그 호스트의 오차 상한과 시계 영역을 `sensor_host_clock_error_bound_ns`·`sensor_host_clock_domain_id`(기본값 없음, 로봇 쪽과 같은 이름)로 받는다. 오차 상한은 이 호스트 측정값 + 센서 호스트 값이다. 두 시계 영역이 다르면 SYNCED 가 아니다. 드라이버가 이 호스트에서 돌 때만 0 과 같은 영역을 쓰고, 그 근거가 기동 로그에 남는다. 하류(target_locator, 조정기)는 UNKNOWN 관측을 쓰지 않는다(C-03).

### 상대 좌표 출력

[사용자 요구, 2026-10-13] “인형이 detect되서 정확히 감지된 1개의 프레임만
depth를 뽑는게 어떨까? 나머지때는 depth를 사용하지 않고 있는거야 … 그렇게 코드를 수정해봐”.
RGB 추론·화면은 Depth·CameraInfo 없이도 계속 실행한다. 좌표 처리 조건은 confidence
`confirm_confidence_min`(지금 0.8) 이상과 화면 내부 bbox이며, 여러 탐지가 있으면 조건을 만족한 최고 confidence 하나를
선택한다. RGB의 원 stamp에서 `sync_slop_ns` 이내의 Depth 하나만 해제·검사한다.
탐지 전 Depth는 압축 상태(raw 입력은 원 메시지 상태)로 `sync_queue_size`개만 보관한다.
대기 RGB는 하나이고 `depth_wait_s`가 지나면 해제하며 다음 탐지에서 재시도한다.
`depth_wait_s=0.5`는 제한된 PC 대기 시간의 [제안]이며 실기 확정값이 아니다.
시각·정렬 설정이 이미 계산을 차단하는 경우 Depth 해제를 건너뛰고 이유를 표시한다.
깊이 부족·처리 실패·publish 실패는 다음 탐지 프레임에서 재시도한다. 전송 성공 뒤에는
Depth 구독과 계산을 중단하고 RGB 표시만 계속한다. TF 실패 시 카메라 관측을 한 번
남기는 기존 동작은 유지한다.

탐지 전에도 원 stamp 매칭용 Depth 수신은 필요하므로 전송량 감소를 보장하지 않는다.
이 변경은 로봇의 해상도·발행 FPS·Depth 드라이버 설정을 바꾸지 않는다. 다른 Depth
구독자가 있으면 이 노드의 구독 종료 뒤에도 그 구독자에게 영상이 발행될 수 있다.

[사용자 요구, 2026-10-13] 로봇은 RGB·Depth 토픽 전송을 담당하며, 추론·Depth 해제·
거리 추정·좌표 변환은 모두 로봇 제어용 PC에서 수행한다. 이 선택 처리 노드도 PC에서
실행하며 로봇에 계산 프로세스를 배포하거나 카메라 설정을 변경하지 않는다.

탐지 confidence와 좌표 유효성은 별도로 표시한다. `amr_target_selective`는 시작할 때
`CLOCK_UNCERTAIN`·`ALIGNMENT_UNVERIFIED` 설정 관문을 경고하고, 후보의 실제
실패 이유와 confidence를 화면·로그에 함께 표시한다. 같은 이유의 로그는
`no_frame_warn_s` 주기로 제한한다. `UNCONFIRMED`만으로 탐지 실패라고 판단하지 않는다.
RGB 원본의 왜곡 계수가 0이 아니면 `rgb_rectified=True`로 선언하는 것만으로
보정되지 않는다. 실제 rectified RGB와 그 픽셀에 정합된 Depth 또는 검증된
소프트웨어 보정이 필요하다. 드라이버의 depth alignment 설정·동일 frame 이름만으로
정렬 검증을 완료했다고 간주하지 않는다.

[사용자 요구, 2026-10-12] 현재 좌표 계산 범위는 카메라 optical 기준과 로봇 base_link 기준까지다. 지도 좌표는 이후 mapping 연결에서 처리한다. 원 TargetObservation의 sensor_frame·sensor_position·관측 시각·ID·신뢰도·공분산은 카메라 기준으로 보존한다.

로봇 기준 점은 `perception/target_base_point`에 geometry_msgs/PointStamped로 발행한다. header.frame_id는 SETTINGS의 base_frame(현재 base_link), stamp는 원 RGB 관측 시각, point는 m 단위 XYZ다. camera→base 변환은 실제 TF를 관측 시각으로 조회하며 위치 이동과 회전을 모두 적용한다. 최신 TF 조회나 임의 장착값으로 대체하지 않는다. 고정 장착 static TF의 반환 stamp=0은 관측 시각으로 조회한 결과일 때 허용한다.

카메라 XYZ가 무효이면 base 점을 발행하지 않는다. TF 조회·frame·시각·변환 검사에 실패하면 이유와 observation_id를 로그에 남기고 원 카메라 관측만 유지한다. base PointStamped는 ID·신뢰도·공분산을 담지 않는 보조 좌표 출력이며 여러 탐지를 영속 연결하거나 이동을 허가하는 메시지가 아니다. 현재 UNKNOWN clock·미검증 정렬 설정에서는 유효 XYZ를 발행한다고 가정할 수 없다. 실제 입력의 근거를 확인한 뒤 사용해야 한다.

### 지도 좌표 TargetLocation (target_locator)

[작전 B2, 2026-10-13] `target_locator` 는 amr_target 이 낸 확정 TargetObservation 을 받아 조정기가 받는 TargetLocation 을 만든다.

| 항목 | 규칙 |
| --- | --- |
| 입력 | `observations_topic`(RELIABLE·TRANSIENT_LOCAL)의 TargetObservation. `source_robot` 이 `robot_id` 와 같고 `localization_status=VALID`·유한한 `sensor_position` 일 때만 쓴다 |
| TF | `map_frame ← sensor_frame` 을 관측의 `observed_at_ns`(RGB stamp)로 조회한다. 최신 TF로 바꾸지 않는다. `tf_wait_s` 안에 그 시각 TF가 없으면 내지 않고 ERROR 로그를 남긴다. 로봇 TF는 namespace 아래라 launch 에서 `/tf`·`/tf_static` 을 remap 한다 |
| 좌표 | 카메라 점에 TF를 적용한 뒤, 선언된 정렬 `alignment_xy_yaw`(mission ← map)를 적용한다. 공분산은 관측 공분산을 같은 회전으로 돌리고 선언된 정렬 오차(위치 + 회전 lever arm)를 더한다. AMCL 위치 불확실성은 들어가지 않는다 [미확인] |
| frame_ref | `map_session_id`·`map_revision` 은 TB-6 MapState(`map_state_topic`, 기본 `/rescue/robots/<SCOUT>/map_state`)에서 온다(OPS-115: TB-6 만 SLAM, 대상 확정 때 지도 고정). 가장 최근 VALID MapState 를 쓰되, 관측 촬영 시각 − 시계 오차가 그 revision 이 처음 VALID 로 나온 시각보다 뒤일 때만 쓴다. 아니면 `CAPTURED_BEFORE_MAP_REVISION` 으로 내지 않는다. VALID MapState 가 없거나 위치 추정을 잃었으면 `NO_VALID_MAP_STATE`, 지도 frame 이 다르면 `MAP_FRAME_MISMATCH` 다. 고정 직전 관측이 새 revision 으로 붙지 않는다. `transform_revision`·`mission_frame` 은 프로파일 `map_basis` 값이다. 조정기가 고정 뒤 이전 기준 관측을 거절한다(OPS-114) |
| 출력 | `target_locations_topic` 에 `projection_status=VALID` 하나. 같은 관측·같은 정렬은 같은 `location_id` 다(아래). 거절된 입력은 VALID 아닌 상태로도 내지 않는다(조정기가 VALID 만 받는다) |
| 시간 기준 (C-03, OPS-103) | 관측이 `clock_state=SYNCED`·오차 상한 있음·`stamp_semantics` 가 UNKNOWN 아님·`clock_domain_id` 가 설정값과 같음이어야 한다. 이 호스트 시각으로 만료(`now > valid_until_ns`)거나 미래 stamp(`observed_at_ns > now + 오차`)면 내지 않는다. 이유는 로그(`CLOCK_NOT_SYNCED` 등)에 남는다 |
| 정렬 권한 (C-04, OPS-103) | `alignment_authority_id`·`alignment_valid_from_ns`·`alignment_valid_until_ns` 를 실행 인자로 받는다(권한 출처: 프로파일 `map_basis` + 로봇의 saved-map 선언, #132·#145). 촬영 시각이 창 안이고 지금이 창 끝 전일 때만 VALID 다. 설정값만으로 VALID 를 채우지 않는다 |
| 공분산 (C-02) | 입력·출력 공분산이 유한·대칭·대각 비음수·양의 준정부호(주소행렬식 모두 ≥ 0)가 아니면 `COVARIANCE_INVALID` 로 내지 않는다 |
| location_id | (observation_id, session, revision, transform revision, 정렬 권한, 정렬값)의 uuid5. 정렬을 바꾸면 새 ID 라서 조정기의 `ID_CONTENT_CONFLICT` 가 나지 않는다 |
| 파라미터 | 모두 기본값 없는 타입 명시 파라미터. 없거나 틀리면 exit 2. 기동 로그에 git SHA·적용 값 |

시험: `tests/test_perception_target_location.py`(합성 TF·관측, E2), `tests/ros/e3_target_locator_probe.py`(LOCALHOST ROS, 가짜 관측·TF → target_locator → 조정기 대상, E3). 실제 로봇 TF·카메라는 NOT_RUN이다.
### 웹캠 판정 영상 중계 (mjpeg_relay)

[작전 배정, 2026-10-13] `mjpeg_relay` 는 webcam_node 의 판정 화면 영상(`debug_image_topic`, CompressedImage jpeg)을 HTTP multipart MJPEG 로 낸다. 관제 웹은 `--camera cam_fixed=http://<웹캠 PC>:<port><path>` 로 보여 준다. 표준 라이브러리 `http.server` 만 쓰고 새 설치는 없다.

```bash
ros2 run rescue_perception mjpeg_relay --ros-args -p image_topic:=/rescue/cameras/cam_fixed/debug_image/compressed \
  -p bind_host:=<관제 PC 에서 닿는 주소> -p port:=<포트> -p path:=/cam_fixed -p stale_s:=1.0
```

| 항목 | 규칙 |
| --- | --- |
| 프레임 | 받은 프레임을 한 번씩만 보낸다. jpeg 가 아닌 형식은 보내지 않고 형식마다 한 번 경고한다 |
| 끊김 | `stale_s` 안에 새 프레임이 없으면 열린 스트림을 닫는다. 마지막 프레임을 반복하지 않는다. 새 프레임이 없을 때 온 요청은 503 이다 |
| 경로 | `path` 하나만 연다. 다른 경로는 404 |
| 느린 클라이언트 | 소켓 읽기·쓰기 타임아웃은 `stale_s` 다. 읽지 않는 클라이언트(멈춘 탭, FIN 없이 끊긴 네트워크)는 그 시간 뒤 끊어서 서버 스레드가 묶이지 않게 한다 |
| 파라미터 | 모두 기본값 없는 타입 명시. 없거나 틀리면 exit 2. 기동 로그에 git SHA·적용 값·URL |

시험: `tests/test_perception_mjpeg_relay.py`(localhost HTTP, E2). 판정 화면 영상은 판정에 쓰지 않는 확인용이다.

### TB-6 재확인 관측 (ObserveTarget)

[재범 결정 2026-10-13] 구출 완료는 「웹캠 출구 소실 후보 + TB-6 현장 재확인」이다. `recheck.py`(판정, ROS 없음)와 `recheck_server.py`(`ObserveTarget` action 서버)가 재확인 관측을 맡는다. 로봇 실행기가 REOBSERVE_TARGET 지점에 도착·정지한 뒤 goal 을 보내고(SDD IF-023), 결과의 `RecheckObservation` 을 TaskResult `recheck` 에 실어 조정기로 보낸다(그 연결은 로봇·조정기 쪽).

| 항목 | 규칙 |
| --- | --- |
| 관측 창 | goal 을 받은 시각부터 `requested_window_ns` 동안이다. 그 뒤에 이 호스트가 받은 프레임·스캔만 센다(C-11 8·11행). 취소·마감 초과는 outcome UNKNOWN, 세 상태 모두 UNKNOWN 이고 부재로 만들지 않는다 |
| 카메라 | 확정 탐지가 지점 반경 `presence_radius_m` 안에 있으면 PRESENT. 지점이 화면 안·깊이 범위 안이고 가려지지 않은 프레임(`frame_coverage` VALID)이 `min_frames` 이상이면서 탐지가 없을 때만 ABSENT. 그 지점에 미확정 탐지가 있었거나 프레임이 모자라면 UNKNOWN |
| LiDAR | 지점 둘레 원(`lidar_radius_m`)을 지나는 빔마다 광선-원 교점(들어감·나감 거리)으로 본다(#165 검토 CDX-165-02). 들어감~나감 사이에서 돌아오면 원 안 반사, 들어감 앞에서 돌아오면 가림, 나감 뒤에서 돌아오거나 반사가 없으면(+inf, 또는 range_max 보다 큰 유한값, REP-117) 통과다. 반사가 없을 때는 나감 거리가 range_max 안일 때만 통과로 센다. -inf(range_min 보다 가까이 무언가 있음, REP-117)·NaN·range_min 미만은 판단 불가다(CDX-165-01). 원 안 반사가 `min_lidar_hits` 이상이면 BLOCKED. 가린 빔이나 판단 불가 빔이 하나라도 있으면 UNKNOWN. 모든 빔이 통과하고 `min_lidar_rays` 이상이면 CLEAR. 창 안에 BLOCKED 가 하나라도 있으면 BLOCKED, CLEAR 스캔이 `min_scans` 이상이면 CLEAR |
| 범위 | 카메라 VALID 프레임이 충분하고 LiDAR 가 UNKNOWN 이 아니면 VALID, 지점이 계속 화면·깊이 밖이면 INVALID, 그 밖은 UNKNOWN |
| 시각 | 표본 시각은 센서 캡처 stamp(RGB·scan header, 카메라·LiDAR 드라이버 호스트 = TB-6 Pi 시계)다(OPS-108). goal 수락은 이 호스트 시계라서, 수락 시각 + (이 호스트 측정 오차 + `sensor_host_clock_error_bound_ns`) 이후에 찍힌 표본만 센다. 창 길이는 그 시각부터 잰다. 관측 시각은 CAPTURE 이고, 오차 상한은 두 오차의 합이다. 이 호스트가 측정되지 않았거나 센서 호스트의 시계 영역이 다르면 UNKNOWN 이다 |
| goal 검사 | robot_id·`frame_ref.native_frame`·유한한 지점·창 길이(0 초과, 상한 이하)·마감이 맞지 않으면 FAILED 와 이유 코드. 창은 goal 을 받을 때(수락 콜백) 예약하고 goal 이 어떻게 끝나든 해제한다. 예약된 창이 있으면 새 goal 은 거절(#165 검토 CDX-165-03) |
| 마감 (C-03) | goal 마감은 실행기 시계 값이다. 이 호스트의 측정 오차 상한을 더해 비교한다: `now + 오차 ≥ 마감` 이면 마감이 지난 것(`DEADLINE_PASSED`, 창 중에는 UNKNOWN), `now + 창 + 오차 > 마감` 이면 마감 전에 못 끝나므로 받지 않는다(`DEADLINE_TOO_CLOSE`). 측정이 없으면 오차를 0 으로 주장하지 않고 그대로 비교하며, 결과 관측의 시계는 UNKNOWN 이다. 실행기와 관측 서버가 다른 PC 라면 실행기 쪽 오차는 여기 들어 있지 않다 |

amr_target 이 서버를 연다. 창이 열려 있는 동안 처리한 프레임마다 지점이 보이는지(`frame_coverage`), 확정 탐지의 지도 좌표, 지점 위 미확정 탐지 여부를 넣는다. RGB 가 rectified 가 아니면 미확정 탐지 상자도 탐지 위치와 같은 규칙(모서리·변 중점 8점 왜곡 보정)으로 보정된 격자에 옮긴 뒤 지점과 비교한다(#200 검토 REV-200-01). 지도 좌표는 촬영 시각 TF map ← 카메라, 깊이는 보정된 정렬 Depth 다. TB-6 LaserScan(`recheck_scan_topic`)은 `scan_state` 로 넣는다(`recheck_feed.py`). 설정은 `recheck_*` 파라미터 16개(기본값 없음, `RECHECK_PARAMS`)이고 launch 인자로도 받는다. 창을 기다리는 goal 이 프레임 처리를 막지 않게 amr_target 은 MultiThreadedExecutor 로 돈다. 시험: `tests/test_perception_recheck.py`·`tests/test_perception_recheck_feed.py`(E2), `tests/ros/e3_recheck_probe.py`(LOCALHOST action, 합성 입력), `tests/ros/e3_amr_target_probe.py`(실제 amr_target + 빨간 사각형만 찾는 가짜 탐지기 + 합성 RGB·Depth·TF·스캔). 실제 카메라·LiDAR·로봇은 NOT_RUN 이다.

### 단일 인형·회차별 1회 전송

[사용자 요구, 2026-10-12] 한 회차에는 인형 하나만 사용한다. 화면 표시 기준은 model_conf 설정을 따르지만, 관측/좌표 전송은 첫 CONFIRMED(현재 `confirm_confidence_min` 0.8 이상 및 유효 위치)이며 bbox가 화면 내부에 완전히 들어온 경우만 수행한다. x0>0, y0>0, x1<width, y1<height를 요구하며 어느 경계에든 닿으면 전송을 보류한다. 이 검사는 화면 가장자리에서 잘린 박스를 거절하며 중앙 가림이나 인형 전체 노출을 증명하지 않는다.

상태는 SEARCHING→SENT다. 최초 카메라 관측의 로컬 publish 성공 뒤 SENT로 고정하며 같은 회차의 카메라 관측·base 점을 다시 발행하지 않는다. TF 실패 시에는 카메라 관측만 한 번 남기고 base 점 실패를 로그로 드러낸다. publish 자체가 예외로 실패하면 SEARCHING을 유지한다. 영상 추론·annotated Image는 계속 처리하고 AMR1의 auto SLAM이나 주행을 변경하지 않는다. 인형이 사라졌다 다시 보여도 재무장하지 않는다. 현재 회차 경계는 amr_target의 기동이며 다시 기동하면 새 회차로 시작한다.

두 좌표/관측 publisher는 RELIABLE·TRANSIENT_LOCAL·depth=1로 마지막 출력을 보존한다. 이후 구독자가 캐시를 받으려면 맞는 QoS가 필요하다. 한 번 발행은 AMR2 수신·저장·주행 수락을 뜻하지 않으며, 재전달 시 소비자는 observation_id로 중복을 처리해야 한다. 캐시 전달은 원 관측 시각·유효기간을 연장하지 않는다.

위치는 bbox 중심 픽셀과 bbox 내부 유효 Depth 중앙값으로 구한 인형 위치의 기준점이다. 인형 형상의 정확한 중심·무게중심이나 AMR2의 주행 목표가 아니다. 실제 접근점·이격 거리·장애물/도달 가능성은 기존 mapping·협업·로봇 제어의 후속 연동에서 결정한다. AMR1 기준 좌표를 AMR2 기준 목표로 그대로 사용하지 않는다.

후속 AMR 단위별 연동 시험에는 후보 탐지→기존 제어 측 정지 요청→실제 정지 확인→새 RGB-D 관측→좌표 전달/수신 확인→주행 재개를 포함한다. 인식 노드에서 직접 cmd_vel·Nav2 cancel/goal을 발행하지 않는다. 정지·수신·AMR2 접근 연결은 현재 구현하지 않았으며 자동 SLAM의 지도 갱신과 주행 정지를 구분한다.

### TB-5 머리 위 여유 반복 기록 (headroom_record)

[작전 OPS-113/115, 2026-10-13] TB-5 를 실제로 세우는 것은 Nav2 collision_monitor 의 깊이 점구름 소스다(통합-로봇 담당). 이 도구는 그 소스가 볼 것을 같은 방식으로 세어 기록만 한다. 깊이 영상마다 base_link 앞 상자(`box_x_min_m`~`box_x_max_m`, 좌우 `box_half_width_m`) 안에서 바닥 잡음(`floor_noise_m`, 제안 0.05 m) 위·여유 높이(`clearance_m`) 아래 점 수가 `min_points` 이상이면 STOP 이고, 아니면 PASS 다. 46 cm 여유 높이는 저장소 소유자 결정(2026-10-13)이다. 깊이·CameraInfo 가 없거나 오래됐거나 base_frame TF 가 없으면 UNKNOWN 이고, PASS 로 세지 않는다. 결과 JSON 에는 git SHA·여유 높이 출처·적용 파라미터·회차별 판정(점 수·가장 낮은 높이·이유)·요약(판정 수, 가장 낮은 높이 최소/최대)이 들어간다. 파라미터는 모두 기본값이 없다. 쓰는 법은 [headroom.py](rescue_perception/headroom.py) 맨 위에 있다. 가로판 앞 한 자리에서 10회 돌려 분포를 보고한다.

시험: `tests/test_perception_headroom.py`(판정·점 변환·요약), `tests/ros/e3_headroom_probe.py`(LOCALHOST ROS, 합성 깊이·정적 TF → 판정 기록, E3). 실제 TB-5 깊이·가로판·보이는 거리는 NOT_RUN 이다.

### 출구 QR (OPS-136/138)

[저장소 소유자 결정 2026-10-13 저녁, 작전 OPS-138 설계 승인] TB-6 가 순찰 중 출구 QR 을 보면 그 위치가 출구다. amr_target 이 대상과 같은 RGB-D 프레임에서 OpenCV `QRCodeDetector` 로 읽는다. 같은 시계·CameraInfo·왜곡 보정(#164)·깊이 보정을 쓴다. 내용이 `exit_qr_payload` 와 정확히 같은 프레임이 `exit_qr_required_frames` 번 연속이어야 출구로 인정한다. 그 프레임들의 위치가 모두 있어야 하고, 첫 위치에서 `exit_qr_position_tolerance_m` 안이어야 한다. QR 이 없거나, 내용이 다르거나, 깊이가 모자란(`exit_qr_min_valid_depth_ratio`) 프레임이 하나라도 끼면 처음부터 다시 센다. 위치는 QR 네 꼭짓점을 왜곡 보정하고, 코드 안쪽 절반의 깊이 중앙값으로 중심을 역투영한 점이다. 시각 근거(stamp 의미)나 RGB-깊이 정렬 근거가 없으면 위치를 내지 않는다. 대상과 같은 규칙이다.

인정되면 `/rescue/exit_observations` 에 TargetObservation 하나를 낸다(새 IDL 없음). `class_label` 은 `exit_qr` 이다. `confidence` 1.0 은 확률이 아니라 내용 일치를 뜻한다. `model_id` 는 `opencv.QRCodeDetector/<cv2 버전>`, `model_sha256` 은 설치된 cv2 바이너리의 sha256 이고, 둘 다 기동 로그에 남는다. `tracking_mode` 는 `DETECTION_ONLY` 다. TB-6 지도(session, revision)마다 한 번만 내고, revision 이 바뀌면 같은 QR 도 다시 낸다. launch 의 두 번째 target_locator(`exit_locator`)가 이것을 `/rescue/exit_location` TargetLocation 으로 바꾼다. 지도 기준은 대상과 같다(MapRevisionTracker). 조정기는 `class_label == 'exit_qr'` 만 출구로 쓴다. 지도 고정 때 이전 revision 의 위치를 재기준하는 일과 출구 앞 접근점 계산은 조정기 몫이다.

필수 파라미터(기본값 없음): `exit_qr_payload`, `exit_qr_required_frames`([제안] 5), `exit_qr_position_tolerance_m`([제안] 0.2), `exit_qr_min_valid_depth_ratio`([제안] 0.5), `exit_map_state_topic`(launch 가 프로파일 SCOUT 의 MapState 로 채움). 실기 확인 항목: QR 크기(한 변 20 cm 이상·무광 [제안]), 높이(카메라 높이 근처), TB-6 에서 읽히는 거리·각도, QR 면의 깊이.

시험: `tests/test_perception_exit_qr.py`(판정·위치·메시지·송신), `tests/ros/e3_amr_target_probe.py` 5–7번(합성 RGB-D 의 QR → 출구 관측). 실제 TB-6 영상의 QR, exit_locator → `/rescue/exit_location` 연결, 검출 처리 시간은 NOT_RUN 이다.

### 검증과 범위

[사용자 요구, 2026-10-12] 이번 AMR 합성 입력 시험 3개는 작성 환경의 로컬 검사 전용으로 보관하고 PR에는 포함하지 않는다. 기존 main의 단위시험·CI는 유지한다. 로컬 검사 결과는 코드 규칙의 검증이며 실제 탐지 성능이나 ROS·GPU·로봇 실행을 증명하지 않는다. 실제 영상·Depth·TF를 이용한 검증은 제어용 컴퓨터에서 AMR 단위별 연동 시험으로 진행한다.

실제 robot6 RGB-D·CUDA 추론·GUI·다거리 보정·시간/정렬·공분산·TF/SLAM·조정기/DB·2 AMR/4 PC 인수는 NOT_RUN이다. 단일 거리 보정은 공간 정렬·시간 동기를 증명하지 않는다. 지도 변환 모듈은 아직 실제 mapping 경로에 연결되지 않았다.


### robot5 로컬 선택 Depth 시험 (2026-10-13)

[사용자 요구] “보정은 6과 5과 같은 상황이라 6의 보정을 그대로 가지고 오고 최신 메인 기준으로 가져와줘”.
robot5_depth.json은 robot6_depth.json의 scale/offset을 그대로 복사하고 physical_robot_id는
robot5로 명시했다. 재사용 근거를 파일에 기록했으며 robot5에서 새로 측정한 계수라고
주장하지 않는다. 거리 scale 재사용은 공간 정합 verified나 PC 간 clock SYNCED의 근거가 아니다.

최신 main의 amr_target과 clock/recheck 경로는 유지한다. 로컬 테스트용
[amr_target_selective.py](rescue_perception/amr_target_selective.py)는 최신 adapter/core의
점 왜곡 보정을 사용하며 RGB만 계속 표시하고 조건을 만족한 한 탐지에만 Depth를 처리한다.
기본 amr_target과 이 선택 시험 모듈을 동시에 실행하지 않는다. 노드 이름은 `amr_target_selective`이고, 관측은 `/rescue/selective_test/observations`, 로봇 기준 점은 `perception/selective_test/target_base_point`에 낸다(실제 토픽이 필요하면 ROS remap, #205 검토 REV-205-02). 재확인 서버(#200)와 시계 경로가 없어 TB-6 구출 완료 판정을 대신할 수 없다. 선택 시험의 관측 clock은
UNKNOWN을 유지하며 별도 로봇 제어 입력을 만들지 않는다. 실제 실행 인자는 최신 FIELD_PARAMS와
clock_domain_id를 명시한다. rgb_rectified=False라도 지원되는 K/D/P 점 보정 모델이면 계산할
수 있으며, 공간 정합 미검증은 계속 차단한다. 위 선택 처리 설명은 이 로컬 시험 모듈에 적용한다.
보정 파일은 [robot5_depth.json](rescue_perception/config/robot5_depth.json)이다.

[사용자 검증·설정 승인, 2026-10-13] 현장 시험 후 사용자는 “depth값은 매우 잘맞아”,
“검증 됬으니 false 날려줘”라고 확인했다. robot5 로컬 실행 설정에
rgb_depth_alignment_verified=true와 사용자 확인 근거 ID를 적용했다. 이는 사용자
현장 검증 기록이며 이 PR에서 별도로 수집한 공간 정합 평가 데이터는 아니다.
원본 RGB의 rgb_rectified=false는 유지하고 최신 main의 K/D/P 점 보정을 사용한다.
이 값은 보정을 끄지 않으며, 지원 모델에서 보정된 bbox/중심으로 Depth를 읽는다.
다른 로봇·토픽·해상도·보정값에 사용자 확인을 자동 승계하지 않는다.

설치 후 `ros2 run rescue_perception amr_target_selective`로 실행할 수 있다.
최신 FIELD_PARAMS와 clock_domain_id를 명시해야 한다. robot5 실행에서는
physical_robot_id와 depth_calibration_path를 사용자 로컬 실행 설정으로 각각 robot5와
package:robot5_depth.json에 맞춘다. 일반 main 노드의 clock/recheck 경로와 이 테스트
노드는 동시에 실행하지 않는다. 정합 설정 변경 뒤 실제 노드의 좌표 전송은 아직
재시험하지 않았으며, clock_state UNKNOWN은 운영 최신성 승인으로 사용할 수 없다.

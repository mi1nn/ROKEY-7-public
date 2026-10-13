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
  tf_topic:=/robot6/tf tf_static_topic:=/robot6/tf_static
```

[작전 B1, 2026-10-13] 카메라 토픽(`image_topic`·`depth_topic`·`camera_info_topic`)과 광학 frame(`rgb_optical_frame`·`depth_optical_frame`)은 코드 기본값이 없는 STRING 파라미터다. 전에는 코드 기본값 `camera/...`, readiness 설정 `oakd/rgb/preview/...`, #132 현장 스크립트 `/robot6/oakd/rgb/...` 세 가지가 서로 달랐다. 값이 없거나 비어 있으면 노드가 시작하지 않는다. 실제 robot6 토픽·frame 이름은 [미확인]이며 로봇에서 `ros2 topic list`와 메시지 header 로 확인해 넣는다. 나머지 설정은 SETTINGS 그대로이고 YAML은 필요 없다. 모델 경로는 실행 작업 디렉터리 기준이다. GUI는 SETTINGS의 show=True에서만 켠다. GPU 추론 실패를 CPU 추론으로 숨기지 않는다.

RGB·Depth는 원 stamp로 동기화한다. CameraInfo frame/해상도·최신성, 모델 SHA/class, Depth 유효값·정렬·시각 검사는 유지한다. [사용자 요구, 2026-10-12] 지도 변환은 map_projection.py로 분리해 코드로만 유지하고, 기존 rescue_mapping과의 연결은 후속으로 진행한다. amr_target은 원 카메라 기준 TargetObservation과 annotated Image를 발행하고, 유효한 카메라→base 변환이 있으면 로봇 기준 PointStamped를 추가 발행한다. MapState 구독·map/odom TF 조회·TargetLocation publisher는 없다. 별도 지도 변환 모듈에도 ROS 노드·실행 등록은 없다. 후속 연결에서는 기존 mapping의 fresh MapState·clock·alignment·관측 시각 TF·공분산 근거를 제공하고 검사해야 한다. 현재 합성 함수 시험은 실제 mapping 연동을 증명하지 않는다. 주행 명령은 보내지 않는다.

### 관측 시계 (OPS-103)

[작전 결정 2026-10-13] amr_target 관측의 시계는 코드 값(`clock_state='UNKNOWN'`·`clock_error_bound_ns=-1`)이 아니라 이 호스트의 측정값이다. 웹캠(#146)과 같은 방식이다(`rescue_contracts.clock_measurement.ClockSampler` + `webcam_clock`). `clock_domain_id`·`clock_backend`(timesyncd·chrony·NONE)·`clock_max_age_ns`·`clock_drift_ppm`·`clock_poll_period_s` 는 기본값 없는 파라미터이고, launch 인자로도 받는다. 프레임마다 그 촬영 시각(RGB stamp)에서 마지막 측정이 SYNCED 이고 유효하면 SYNCED·오차 상한(측정값 + 표본 뒤 drift)이고, 아니면 UNKNOWN 이다. 기동 직후 첫 측정 전과 `clock_backend: NONE` 은 UNKNOWN 이다. OAK-D stamp 는 카메라 드라이버가 도는 호스트(TB-6 Pi)의 시계라서, 그 호스트의 오차 상한과 시계 영역을 `sensor_host_clock_error_bound_ns`·`sensor_host_clock_domain_id`(기본값 없음, 로봇 쪽과 같은 이름)로 받는다. 오차 상한은 이 호스트 측정값 + 센서 호스트 값이다. 두 시계 영역이 다르면 SYNCED 가 아니다. 드라이버가 이 호스트에서 돌 때만 0 과 같은 영역을 쓰고, 그 근거가 기동 로그에 남는다. 하류(target_locator, 조정기)는 UNKNOWN 관측을 쓰지 않는다(C-03).

### 상대 좌표 출력

[사용자 요구, 2026-10-12] 현재 좌표 계산 범위는 카메라 optical 기준과 로봇 base_link 기준까지다. 지도 좌표는 이후 mapping 연결에서 처리한다. 원 TargetObservation의 sensor_frame·sensor_position·관측 시각·ID·신뢰도·공분산은 카메라 기준으로 보존한다.

로봇 기준 점은 `perception/target_base_point`에 geometry_msgs/PointStamped로 발행한다. header.frame_id는 SETTINGS의 base_frame(현재 base_link), stamp는 원 RGB 관측 시각, point는 m 단위 XYZ다. camera→base 변환은 실제 TF를 관측 시각으로 조회하며 위치 이동과 회전을 모두 적용한다. 최신 TF 조회나 임의 장착값으로 대체하지 않는다. 고정 장착 static TF의 반환 stamp=0은 관측 시각으로 조회한 결과일 때 허용한다.

카메라 XYZ가 무효이면 base 점을 발행하지 않는다. TF 조회·frame·시각·변환 검사에 실패하면 이유와 observation_id를 로그에 남기고 원 카메라 관측만 유지한다. base PointStamped는 ID·신뢰도·공분산을 담지 않는 보조 좌표 출력이며 여러 탐지를 영속 연결하거나 이동을 허가하는 메시지가 아니다. 현재 UNKNOWN clock·미검증 정렬 설정에서는 유효 XYZ를 발행한다고 가정할 수 없다. 실제 입력의 근거를 확인한 뒤 사용해야 한다.

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

### 단일 인형·회차별 1회 전송

[사용자 요구, 2026-10-12] 한 회차에는 인형 하나만 사용한다. 화면 표시는 탐지 confidence 0.5부터 유지하지만, 관측/좌표 전송은 첫 CONFIRMED(현재 confidence 0.8 이상 및 유효 위치)이며 bbox가 화면 내부에 완전히 들어온 경우만 수행한다. x0>0, y0>0, x1<width, y1<height를 요구하며 어느 경계에든 닿으면 전송을 보류한다. 이 검사는 화면 가장자리에서 잘린 박스를 거절하며 중앙 가림이나 인형 전체 노출을 증명하지 않는다.

상태는 SEARCHING→SENT다. 최초 카메라 관측의 로컬 publish 성공 뒤 SENT로 고정하며 같은 회차의 카메라 관측·base 점을 다시 발행하지 않는다. TF 실패 시에는 카메라 관측만 한 번 남기고 base 점 실패를 로그로 드러낸다. publish 자체가 예외로 실패하면 SEARCHING을 유지한다. 영상 추론·annotated Image는 계속 처리하고 AMR1의 auto SLAM이나 주행을 변경하지 않는다. 인형이 사라졌다 다시 보여도 재무장하지 않는다. 현재 회차 경계는 amr_target의 기동이며 다시 기동하면 새 회차로 시작한다.

두 좌표/관측 publisher는 RELIABLE·TRANSIENT_LOCAL·depth=1로 마지막 출력을 보존한다. 이후 구독자가 캐시를 받으려면 맞는 QoS가 필요하다. 한 번 발행은 AMR2 수신·저장·주행 수락을 뜻하지 않으며, 재전달 시 소비자는 observation_id로 중복을 처리해야 한다. 캐시 전달은 원 관측 시각·유효기간을 연장하지 않는다.

위치는 bbox 중심 픽셀과 bbox 내부 유효 Depth 중앙값으로 구한 인형 위치의 기준점이다. 인형 형상의 정확한 중심·무게중심이나 AMR2의 주행 목표가 아니다. 실제 접근점·이격 거리·장애물/도달 가능성은 기존 mapping·협업·로봇 제어의 후속 연동에서 결정한다. AMR1 기준 좌표를 AMR2 기준 목표로 그대로 사용하지 않는다.

후속 AMR 단위별 연동 시험에는 후보 탐지→기존 제어 측 정지 요청→실제 정지 확인→새 RGB-D 관측→좌표 전달/수신 확인→주행 재개를 포함한다. 인식 노드에서 직접 cmd_vel·Nav2 cancel/goal을 발행하지 않는다. 정지·수신·AMR2 접근 연결은 현재 구현하지 않았으며 자동 SLAM의 지도 갱신과 주행 정지를 구분한다.

### 검증과 범위

[사용자 요구, 2026-10-12] 이번 AMR 합성 입력 시험 3개는 작성 환경의 로컬 검사 전용으로 보관하고 PR에는 포함하지 않는다. 기존 main의 단위시험·CI는 유지한다. 로컬 검사 결과는 코드 규칙의 검증이며 실제 탐지 성능이나 ROS·GPU·로봇 실행을 증명하지 않는다. 실제 영상·Depth·TF를 이용한 검증은 제어용 컴퓨터에서 AMR 단위별 연동 시험으로 진행한다.

실제 robot6 RGB-D·CUDA 추론·GUI·다거리 보정·시간/정렬·공분산·TF/SLAM·조정기/DB·2 AMR/4 PC 인수는 NOT_RUN이다. 단일 거리 보정은 공간 정렬·시간 동기를 증명하지 않는다. 지도 변환 모듈은 아직 실제 mapping 경로에 연결되지 않았다.

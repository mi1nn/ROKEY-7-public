# rescue_perception

## 현재 상태 (2026-10-14, main 4163a25)

main 4163a25 의 코드를 읽고 적었다(작전 OPS-210). 「구현됨」은 코드와 합성·격리 시험이 있다는 뜻이다. 실제 로봇·카메라에서 확인했다는 뜻이 아니다. 실기 결과는 따로 적었고, 실기 기록이 없는 기능은 「실기 NOT_RUN」이다. 실기 기록은 이 README 와 [구현 문서](../../docs/implementation/implementation-perception.md)에서 찾았다.

| 기능 | 상태 | 근거 |
| --- | --- | --- |
| 웹캠 지진 판정 (`DISASTER_SIGN`) | 구현됨. 실기는 한 번 확인했다: 실물 USB 웹캠을 한 번 흔들어 사건 1건이 나왔다 [측정, 2026-10-12, 구현 문서]. 오탐률과 장시간 동작은 NOT_RUN | `disaster_sign.py:DisasterSignDetector`, `webcam_node` |
| 지진 판정 → TB-6 자동 출발 | 구현됨(조정기 쪽, 현장 파일 `search_start` AUTO). 실기 NOT_RUN: 위 두 문서와 조정 구현 문서에 실기 기록이 없다 | #228, `rescue_coordination/ros_node.py` `tick`(`search_start == 'AUTO'`) |
| 웹캠 인형 소실 후보 | 구현됨. 실물 웹캠으로 인형 탐지 신뢰도 0.96·소실 후보 정상 [측정, 2026-10-12, 구현 문서]. 현장 PC 와 조건별 측정은 NOT_RUN | `target_watch.py:TargetWatchDetector` |
| 로봇 인형 탐지와 RGB-D 위치 (amr_camera·amr_target) | 구현됨. robot5 에서 깊이 약 1.350 m·base XYZ 를 사용자가 확인했다(아래 「AMR 통합 전 최종 인식 파일」). 고친 노드 전체를 로봇에서 다시 돌린 시험(E4)은 NOT_RUN | `amr_target.py:create_node`, `person_pipeline.PersonPipeline` |
| 지도 좌표 TargetLocation (target_locator) | 구현됨. 실기 NOT_RUN | `target_locator.py:main`, `target_location.py` |
| TB-6 재확인 관측 (ObserveTarget) | 구현됨, 지금 시나리오에서는 쓰지 않음(scope Q-10, completion_mode EXIT_ARRIVAL). 실기 NOT_RUN | `recheck_server.py:RecheckServer`, `recheck.py` |
| 출구 QR (여러 출구, 정확히 같은 내용만) | 구현됨. 실기 NOT_RUN | #245, `exit_qr.py:detect_qrs`·`ExitQrConfirmer` |
| 머리 위 구조물(터널) 판정 overhead_passage | 구현됨. 실기 NOT_RUN. 높이 기준을 먼저 확인해야 한다(아래 절) | #259, `overhead.py:judge`·`PassageRegistry` |
| 터널 판정 영상 passage_view | 구현됨. 실기 NOT_RUN(미리보기 해상도·속도, 각 PC 글꼴) | #264, `overhead.py:render_view` |
| TB-5 머리 위 여유 기록 headroom_record | 구현됨. 실기 NOT_RUN. 상자 안 바닥 위 점이 `min_points` 개 미만이면 UNKNOWN(`FEW_POINTS_ABOVE_FLOOR`)이다(2026-10-14, 재범 결정) | `headroom.py:judge` |
| 관측이 모자랄 때 UNKNOWN (headroom) | 구현됨(점 수 기준만). 가림·시야 밖 구분과 46 cm 근처 오차 보류는 실측 뒤에 정한다 | `headroom.py:judge` |
| 판정 영상 중계 (mjpeg_relay, robot_view_relay) | 구현됨. 실기 NOT_RUN | `mjpeg_relay.py`, #261 `robot_view_relay.py` |

**headroom `judge` 의 관측 부족 UNKNOWN** (2026-10-14 고침, 재범 결정): 예전에는 `points` 가 `None` 일 때만 UNKNOWN 이었다. 빈 목록, 상자 밖 점만 있는 목록, 바닥 잡음(`floor_noise_m`) 아래 점만 있는 목록도 `PASS`, `lowest_m None` 이 나왔다(robot6 2026-10-13 기록 3회가 이 경우). 이제는 상자 안 바닥 위 점이 `min_points` 개 이상일 때만 PASS 다. 그보다 적으면 가로대를 보지 못한 것이라 UNKNOWN 으로 기록하고, 이유에 `FEW_POINTS_ABOVE_FLOOR seen=<개수>` 를 남긴다. 새 숫자는 만들지 않고 낮은 점을 세는 `min_points` 를 그대로 쓴다. 결과에 `points_seen`(상자 안 바닥 위 점 수)을 더했다. 그래서 가로대가 없는 빈 통로에서 재면 PASS 가 아니라 UNKNOWN 이 나온다. 이 도구는 가로대 앞에서 쓰는 측정 도구라 그렇게 둔다.

## 패키지 개요

기록된 탐지 결과와 RGB/Depth 메타데이터·Depth 배열을 검증하는 순수 Python 코어가 바탕이다. 표준 라이브러리와 저장소 내부 `rescue_contracts`에 의존한다. `adapter`가 ROS 메시지 모양의 입력(비압축 Image·CameraInfo·Detection2D)을 이 코어로 잇는다. 그 위의 ROS 노드(`setup.py` 진입점: webcam_node, amr_camera, amr_target, target_locator, mjpeg_relay, robot_view_relay, headroom_record, overhead_passage)는 아래 절에 있다. 주행 명령은 어느 노드도 내지 않는다.

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

[OAK-D 학습 계획](../../docs/implementation/oakd-survivor-training-plan.md): 사용자 결정(2026-10-11)에 따라 이번 학습은 `oakd/image_raw` 영상의 1초 간격 프레임을 전처리해 단일 클래스 `survivor`(ID 0)를 사용한다. 후속 연동의 모델 class metadata·manifest·`ConfirmationPolicy.person_labels`·Detector 출력 label을 맞춘다. 기존 API 이름과 과거 person 합성 시험은 유지한다. 학습은 끝났고 2026-10-11 에 100 epoch best.pt 를 골랐다([선택 manifest](../../assets/manifest/yolo26n-survivor-v2.json)). 실기 결과는 맨 위 「현재 상태」 표에 있다.


## AMR 단위 검증 노드 (2026-10-12)

[사용자 요구] 기존 인식 코어에 AMR 알고리즘을 연결하고 amr_camera·amr_target을 각각 시험·보정한 뒤 기존 코드의 후속 수정·통합을 진행한다.

| 파일 | 목적 |
| --- | --- |
| [amr_camera.py](rescue_perception/amr_camera.py) | RGB raw/compressed → YOLO 1회 → 원 헤더를 보존한 annotated Image. ROS·영상·모델 의존성은 파일 상단에서 import한다 |
| [amr_target.py](rescue_perception/amr_target.py) | RGB·Depth·CameraInfo → TargetObservation(카메라 기준), 유효한 camera→base TF가 있으면 PointStamped(로봇 기준). ROS 의존성은 상단 import |
| [camera_utils.py](rescue_perception/camera_utils.py) | 공통 SETTINGS, 모델 SHA/class·RGB 형식 검사, 복호화·추론·annotated Image 생성 |
| [target_utils.py](rescue_perception/target_utils.py) | RGB-D SETTINGS·target_config, YOLO 관측 메시지, Depth 복호화·robot6 보정, 카메라→base 상대 좌표 |
| [map_projection.py](rescue_perception/map_projection.py) | 지도 상태·정렬·관측 시각 TF 검사, 지도/mission 좌표·공분산 변환. 코드와 합성 시험만 보존하며 실행 노드에 연결하지 않음 |
| [target_location.py](rescue_perception/target_location.py) | 확정 TargetObservation + 촬영 시각 TF map←camera → TargetLocation(VALID) 좌표·공분산·frame_ref. ROS 없음 |
| [target_locator.py](rescue_perception/target_locator.py) | 위 변환의 ROS 노드: `/rescue/observations` 구독 → 촬영 시각 TF 조회 → `/rescue/target_locations` 발행 |
| [exit_qr.py](rescue_perception/exit_qr.py) | 출구 QR 판정(허용 목록과 정확 일치·출구별 N 프레임 연속·위치 허용 범위·깊이)과 출구 TargetObservation 채우기. ROS 없음, amr_target 이 프레임마다 부름 |
| [headroom.py](rescue_perception/headroom.py) | TB-5 머리 위 여유 실기 반복 판정 기록 도구 `headroom_record`(OPS-115). 판정·발행·정지 명령 없음 |
| [robot_depth.json](rescue_perception/config/robot_depth.json) | robot5·robot6 공통 보정계수·offset·허용 로봇 ID·입력 단위/상태. 설치본에도 포함 |

실제 실행 대상은 로봇 제어용 컴퓨터다(Ubuntu 24.04·ROS 2 Jazzy, 아래 「사용자 요구, 2026-10-13」 문단).

[설정/사용자 요구, 2026-10-13] 첫 CUDA GPU `device: '0'`, 탐지 신뢰도 `model_conf: 0.5`, 기존 대상 확정 정책 `confirm_confidence_min: 0.9`, `physical_robot_id: robot6`, `depth_calibration_path: package:robot_depth.json`. 그 외 토픽·frame 이름과 최신성·깊이 임계값은 이전 예시의 [제안]이며 실기 검증 전이다. 코드 통합은 이 값을 측정 사실로 바꾸지 않는다.

[사용자 요구, 2026-10-13] “robot 5랑 6은 depth보정도 같으니 로봇용 보정으로 통합해줘”. 공통 robot_depth.json의 physical_robot_ids는 robot5·robot6만 허용한다. 실제 physical_robot_id는 실행 로봇에 맞춰 유지한다. m 단위 raw 입력·유효한 scale/offset을 검사하고 raw_m * scale + offset_m을 한 번만 적용한다. raw/PNG compressedDepth, 16UC1/mm·32FC1/m에 같은 보정을 쓰며 이미 보정된 입력은 사용하지 않는다. 거리 scale 공유는 공간 정합이나 시계 검증을 대신하지 않는다.

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

RGB·Depth는 원 stamp로 동기화한다. CameraInfo frame/해상도·최신성, 모델 SHA/class, Depth 유효값·정렬·시각 검사는 유지한다. [사용자 요구, 2026-10-12] 지도 변환은 map_projection.py로 분리해 코드로만 유지하고, 기존 rescue_mapping과의 연결은 후속으로 진행한다. amr_target은 원 카메라 기준 TargetObservation과 annotated Image를 발행하고, 유효한 촬영 시각 카메라→base 변환을 확인한 뒤 로봇 기준 PointStamped와 원 카메라 관측을 발행한다. amr_target 자신은 map/odom TF 를 조회하지 않고 TargetLocation 도 내지 않는다. 지도 좌표는 별도 노드 target_locator 가 만든다(아래 「지도 좌표 TargetLocation」). 별도 지도 변환 모듈에도 ROS 노드·실행 등록은 없다. 후속 연결에서는 기존 mapping의 fresh MapState·clock·alignment·관측 시각 TF·공분산 근거를 제공하고 검사해야 한다. 현재 합성 함수 시험은 실제 mapping 연동을 증명하지 않는다. 주행 명령은 보내지 않는다.

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
`confirm_confidence_min`(지금 0.9) 이상과 화면 내부 bbox이며, 여러 탐지가 있으면 조건을 만족한 최고 confidence 하나를
선택한다. RGB의 원 stamp에서 `sync_slop_ns` 이내의 Depth 하나만 해제·검사한다.
탐지 전 Depth는 압축 상태(raw 입력은 원 메시지 상태)로 `sync_queue_size`개만 보관한다.
대기 RGB는 하나이고 `depth_wait_s`가 지나면 해제하며 다음 탐지에서 재시도한다.
`depth_wait_s=1.5`는 최신 main 문서의 TB-6 Depth 지연 약 0.78 s를 덮기 위한 [설정, 2026-10-13] 대기 상한이다. 타임아웃이면 다음 RGB 탐지에서 재시도한다.
시각·정렬 설정이 계산을 차단하면 Depth 해제를 건너뛴다. 지원되는 CameraInfo K/D/P로
RGB 중심과 bbox를 정렬 Depth 격자로 먼저 보정한 뒤 중심 11×11 ROI(`depth_roi_radius_px=5`)
내 유효 depth만 검사한다. ROI는 보정된 객체 bbox와 영상 경계로 제한한다.
범위·유효 비율·깊이 불연속 검사, 보정 1회 적용, 촬영 시각 TF 검사를 유지한다.

첫 객체 전송 뒤 객체용 Depth 계산은 중단한다. 최신 main의 ObserveTarget 재관측 창이
열렸거나 출구 QR 후보가 있을 때는 그 기능에 필요한 Depth를 처리하므로 구독은 유지한다.
재관측은 객체가 없어도 depth/coverage 증거가 필요하다. 대기 요청 하나·Depth 캐시는
제한되며 압축 해제 없이 받은 메시지만 보관한다. 수신 전송량 자체를 줄이는 변경은 아니다.
로봇의 해상도·발행 FPS·카메라 설정은 변경하지 않는다.

[사용자 요구, 2026-10-13] 로봇은 RGB·Depth 토픽 전송을 담당하며, 추론·Depth 해제·
거리 추정·좌표 변환은 모두 로봇 제어용 PC에서 수행한다. 이 선택 처리 노드도 PC에서
실행하며 로봇에 계산 프로세스를 배포하거나 카메라 설정을 변경하지 않는다.

탐지 confidence와 좌표 유효성은 별도로 표시한다. 기반 `amr_target` 한 노드에서
RGB 추론·annotated 화면을 독립 처리하고 Depth 수신/해제 수·stamp 차이·요청 만료 수를
로그로 남긴다. corrected depth·valid_pixels·base_xyz·촬영 stamp도 별도 줄로 출력한다.
`rgb_rectified=false`이면 지원되는 K/D/P 점 보정을 사용한다. compressed는 전송 형식으로
왜곡 보정 여부를 뜻하지 않는다. 정렬 검증은 사용자 현장 근거를 명시해야 한다.

[사용자 요구, 2026-10-12] 현재 좌표 계산 범위는 카메라 optical 기준과 로봇 base_link 기준까지다. 지도 좌표는 이후 mapping 연결에서 처리한다. 원 TargetObservation의 sensor_frame·sensor_position·관측 시각·ID·신뢰도·공분산은 카메라 기준으로 보존한다.

로봇 기준 점은 `perception/target_base_point`에 geometry_msgs/PointStamped로 발행한다. header.frame_id는 SETTINGS의 base_frame(현재 base_link), stamp는 원 RGB 관측 시각, point는 m 단위 XYZ다. camera→base 변환은 실제 TF를 관측 시각으로 조회하며 위치 이동과 회전을 모두 적용한다. 최신 TF 조회나 임의 장착값으로 대체하지 않는다. 고정 장착 static TF의 반환 stamp=0은 관측 시각으로 조회한 결과일 때 허용한다.

카메라 XYZ가 무효이면 base 점을 발행하지 않는다. TF 조회·frame·시각·변환 검사에 실패하면 오류를 기록하고 관측/좌표 전송과 SENT 전환을 보류한다. 다음 탐지 프레임에서 재시도한다. base PointStamped는 ID·신뢰도·공분산을 담지 않는 보조 좌표 출력이며 여러 탐지를 영속 연결하거나 이동을 허가하는 메시지가 아니다.

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

[2026-10-15] 두 중계기(mjpeg_relay·robot_view_relay)는 `GET /health` 로 `{age_s, stale_s}`(마지막 프레임 나이, 첫 프레임 전에는 `age_s: null`)를 낸다. 관제 웹 페이지가 다른 origin 에서 묻기 때문에 `Access-Control-Allow-Origin: *`, `Cache-Control: no-store` 를 붙인다. 관제 웹은 이 값으로 영상 끊김을 판정한다(Chrome 은 multipart img 에 load 이벤트를 한 번만 낸다).

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

[사용자 요구, 2026-10-12] 한 회차에는 인형 하나만 사용한다. 화면 표시 기준은 model_conf 설정을 따르지만, 관측/좌표 전송은 첫 CONFIRMED(현재 `confirm_confidence_min` 0.9 이상 및 유효 위치)이며 bbox가 화면 내부에 완전히 들어온 경우만 수행한다. x0>0, y0>0, x1<width, y1<height를 요구하며 어느 경계에든 닿으면 전송을 보류한다. 이 검사는 화면 가장자리에서 잘린 박스를 거절하며 중앙 가림이나 인형 전체 노출을 증명하지 않는다.

상태는 SEARCHING→SENT다. 촬영 시각의 base TF 확인과 base 점·카메라 관측의 로컬 publish 성공 뒤 SENT로 고정하며 같은 회차의 카메라 관측·base 점을 다시 발행하지 않는다. TF 실패 시 SEARCHING을 유지하며 다음 탐지에서 재시도한다. publish 자체가 예외로 실패하면 SEARCHING을 유지한다. 영상 추론·annotated Image는 계속 처리하고 AMR1의 auto SLAM이나 주행을 변경하지 않는다. 인형이 사라졌다 다시 보여도 재무장하지 않는다. 현재 회차 경계는 amr_target의 기동이며 다시 기동하면 새 회차로 시작한다.

두 좌표/관측 publisher는 RELIABLE·TRANSIENT_LOCAL·depth=1로 마지막 출력을 보존한다. 이후 구독자가 캐시를 받으려면 맞는 QoS가 필요하다. 한 번 발행은 AMR2 수신·저장·주행 수락을 뜻하지 않으며, 재전달 시 소비자는 observation_id로 중복을 처리해야 한다. 캐시 전달은 원 관측 시각·유효기간을 연장하지 않는다.

위치는 bbox 중심 픽셀과 bbox 중심 ROI 내부 유효 Depth 중앙값으로 구한 인형 위치의 기준점이다. 인형 형상의 정확한 중심·무게중심이나 AMR2의 주행 목표가 아니다. 실제 접근점·이격 거리·장애물/도달 가능성은 기존 mapping·협업·로봇 제어의 후속 연동에서 결정한다. AMR1 기준 좌표를 AMR2 기준 목표로 그대로 사용하지 않는다.

후속 AMR 단위별 연동 시험에는 후보 탐지→기존 제어 측 정지 요청→실제 정지 확인→새 RGB-D 관측→좌표 전달/수신 확인→주행 재개를 포함한다. 인식 노드에서 직접 cmd_vel·Nav2 cancel/goal을 발행하지 않는다. 정지·수신·AMR2 접근 연결은 현재 구현하지 않았으며 자동 SLAM의 지도 갱신과 주행 정지를 구분한다.

### TB-5 머리 위 여유 반복 기록 (headroom_record)

[작전 OPS-113/115, 2026-10-13] 저장소의 [robot.launch.py](../rescue_bringup/launch/robot.launch.py)는 Nav2 collision_monitor 에 `PassageStop`과 깊이 점구름 소스를 추가한다. 별도 [passage_points.launch.py](../rescue_bringup/launch/passage_points.launch.py)는 깊이 영상과 CameraInfo를 점구름으로 바꾼다. `headroom_record`는 이 정지 경로가 아니라 깊이 영상을 판정·기록하는 도구다. 깊이 영상마다 base_link 앞 상자(`box_x_min_m`~`box_x_max_m`, 좌우 `box_half_width_m`) 안에서 바닥 잡음(`floor_noise_m`, 제안 0.05 m) 위·여유 높이(`clearance_m`) 아래 점 수가 `min_points` 이상이면 STOP 이고, 아니면 PASS 다. 46 cm 여유 높이는 저장소 소유자 결정(2026-10-13)이다. 깊이·CameraInfo 가 없거나 오래됐거나 base_frame TF 가 없으면 UNKNOWN 이고, PASS 로 세지 않는다. 결과 JSON 에는 git SHA·여유 높이 출처·적용 파라미터·회차별 판정(점 수·가장 낮은 높이·이유)·요약(판정 수, 가장 낮은 높이 최소/최대)이 들어간다. 파라미터는 모두 기본값이 없다. 쓰는 법은 [headroom.py](rescue_perception/headroom.py) 맨 위에 있다. 가로판 앞 한 자리에서 10회 돌려 분포를 보고한다.

시험: `tests/test_perception_headroom.py`(판정·점 변환·요약), `tests/ros/e3_headroom_probe.py`(LOCALHOST ROS, 합성 깊이·정적 TF → 판정 기록, E3). 실제 TB-5 깊이·가로판·보이는 거리는 NOT_RUN 이다.

#### robot6 적용 전 확인 (2026-10-13)

[사용자 요구] 통과 높이 **0.46 m**는 정책 기준이다. 깊이 영상의 거리값을 높이로 쓰지 않는다. [headroom.py](rescue_perception/headroom.py)의 `depth_points`는 카메라 정보를 이용해 점을 만들고 촬영 시각 TF로 `base_link`에 변환한다. 그 `z`를 바닥 위 실제 높이로 해석하려면 바닥과 `base_link` 원점의 관계, 카메라 TF, 로봇과 적재물 최고 높이를 확인해야 한다.

[원본 코드] `judge`는 검사 상자 안에서 `z > floor_noise_m`인 점만 사용한다. `z < clearance_m`인 점이 `min_points` 이상이면 STOP이다. 그렇지 않을 때 상자 안 바닥 위 점이 `min_points` 이상이면 PASS, 그보다 적으면 UNKNOWN(`FEW_POINTS_ABOVE_FLOOR`)이다(2026-10-14 고침, #280). 따라서 **정확히 0.46 m인 점은 낮은 점에 포함되지 않는다**. 2026-10-13 측정 당시 코드는 점이 하나도 없어도 PASS를 돌려줬다. 아래 robot6 기록 3회의 PASS는 그 코드의 결과다. 점 수가 모자랄 때의 UNKNOWN은 이제 `judge`에 있다. 가림·시야 밖을 따로 구분하는 판정은 아직 없다. 경계 비교 연산은 바꾸지 않았다.

[설정] `tests/test_perception_headroom.py`의 0.46 m 이외 값(`floor_noise_m=0.05`, 전방 `0.3~1.2 m`, 좌우 반폭 `0.2 m`, `min_points=3`)과 예시 명령의 픽셀 `stride=4`는 시험값이다. 시야·로봇 및 적재물 치수·깊이 잡음으로 검증하기 전에는 robot6 운영 기본값으로 승계하지 않는다. 저장소의 [robot.launch.py](../rescue_bringup/launch/robot.launch.py)는 [nav2_with_passage](../rescue_bringup/rescue_bringup/field_launch.py)의 조립 함수를 거쳐 `PassageStop`과 `passage_depth` 입력을 Nav2 `collision_monitor`에 추가한다. [passage_points.launch.py](../rescue_bringup/launch/passage_points.launch.py)는 Depth와 CameraInfo로 점구름 입력을 별도 실행한다. 예시 현장 파일 [search_area.field.json](../../config/field/search_area.field.json)은 `min_height=0.05 m`, `max_height=0.46 m`, 전방 `[0.4, 1.0] m`, 좌우 반폭 `0.2 m`, `min_points=5`를 설정한다. 이는 저장소 구현·예시 설정의 확인이지, 모든 Nav2 실행에 적용됐거나 실기 정지가 검증됐다는 뜻은 아니다.

[측정, 2026-10-13, robot6 실행 상태] 당시 실행된 `collision_monitor`의 `observation_sources`는 `['scan']`이었고 `/robot6/passage_points` 토픽은 없었다. 실행 프로세스는 기존 `turtlebot4_navigation nav2.launch.py` 경로였다. 그러므로 저장소의 새 `robot.launch.py` 통로 정지 구성은 측정 당시 robot6 프로세스에 적용되지 않았다. 저장소 경로와 실기 실행 상태를 구분한다.

[측정, 2026-10-13, 정지 상태 robot6] 언도킹 뒤 깊이 영상 `16UC1`과 CameraInfo를 수신했다. 시험값으로 `headroom_record`를 3회 실행하면 모두 PASS였지만 `lowest_m`은 모두 `None`이었다. 별도 한 프레임의 점 분포에서는 전방 `0.3~1.2 m`, 좌우 반폭 `0.2 m` 안의 2,287점이 `base_link` 기준 대략 `-0.22~-0.19 m`에 있었다. `tf2_echo base_link oakd_rgb_camera_optical_frame`은 카메라 원점을 `base_link` 기준 `[-0.060, 0, 0.244] m`, 회전을 RPY `[-90, 0, -90]`도로 보였다. TF가 조회된 사실은 물리 장착과 바닥 기준에서 맞다는 검증이 아니다. 저장소 통로 정지 예시의 높이 띠도 `base_link` 기준 `0.05~0.46 m`이므로 바닥과 `base_link` 원점 관계가 확인될 때까지 실제 46 cm 여유와 같다고 간주할 수 없다. 가로판·적재물 치수, 줄자 기준 높이와 TF 정확도는 미측정이다. 주행 중 높이 정지 시험은 NOT_RUN이다.

[제안] 운영 판정에서는 낮은 점이 충분하면 STOP, 깊이·CameraInfo·TF 누락 또는 오래됨과 유효 관측 부족·가림·시야 밖은 UNKNOWN으로 보류한다. PASS에 필요한 관측량과 46 cm 근처의 오차 보류 범위는 임의의 점 수로 확정하지 않는다. 먼저 가로판 아래와 로봇+적재물 최고 높이를 줄자로 재고, 46 cm 미만·근처·초과 높이를 가깝고 먼 거리에서 정지 상태로 반복 측정한다. 판정·유효 점 수·낮은 점 수·최저 높이·시각·TF·적용값을 기록하고 `headroom_record`와 실제 `collision_monitor`의 판정을 대조한다. 저장소 실행 경로의 robot6 적용과 실제 정지 검증은 NOT_RUN이다.

### 지도의 머리 위 구조물 (overhead_passage, OPS-185/186)

46 cm 통로 정지(Nav2 collision_monitor)는 로봇을 세우기만 하고, 어디에 얼마나 높은 구조물이 있는지는 남기지 않는다. `overhead_passage` 는 같은 깊이 점(`<ns>/passage_points`), 같은 상자(`passage.front_m`, `passage.half_width_m`), 같은 높이(`passage.min_height_m`, `passage.max_height_m`)로 구조물을 판정해 지도 좌표로 낸다. 로봇마다 하나씩 띄운다(`overhead_passage.launch.py`).

- **가장 낮은 높이:** 상자 안에서 바닥 여유 위 ~ `top_m` 사이 점 중 `passage.min_points` 번째로 낮은 점의 높이다. collision_monitor 가 멈추는 데 필요한 점 수와 같으므로, 잡음 점 몇 개로는 높이가 내려가지 않는다.
- **떠 있는 구조물만:** 그 높이가 라이다 평면(`passage.overhead.lidar_plane_m`) 위일 때만 구조물로 본다. 그 아래면 라이다가 보는 보통 장애물(벽, 상자)이라 내지 않는다(OPS-186 a).
- **판정:** `PASSABLE`(≥ 0.46 m, 재범 결정) / `BLOCKED`(그 아래) / `UNKNOWN`. 구조물과 같은 x 구간에서 바닥 점이 `floor_min_points` 개 미만이면, 아래가 비었는지 모르므로 높이를 내지 않고 `UNKNOWN`(`NO_FLOOR_SEEN`)으로 둔다(OPS-186 b).
- **확정:** 처리한 프레임 `confirm_frames` 번이 같은 판정, 같은 자리(`position_tolerance_m`)여야 낸다. 다른 판정, 구조물 없음, 오래된 점구름, 지도 revision 없음, TF 없음 프레임이 하나라도 끼면 처음부터 센다.
- **위치와 revision:** 위치는 구조물 바닥 중심을 점구름 시각의 TF 로 map 에 옮긴 점이다. 지도 revision 은 SLAM 로봇(TB-6)의 MapState 에서 가져온다(MapRevisionTracker, 시계 오차는 그 로봇 Pi 값). TB-5 도 TB-6 지도 위에 있다.
- **목록:** 같은 revision 에서 `merge_radius_m` 안이면 같은 항목을 갱신한다. `UNKNOWN` 은 이미 잰 높이를 덮지 않는다. 다른 revision 항목은 따로 남는다(웹이 현재 revision 을 진하게, 이전 것을 흐리게 그림, OPS-186 c).
- **발행:** 목록이 바뀔 때마다 전체를 `/rescue/robots/<id>/overhead_passages`(std_msgs/String JSON `rescue-overhead-passages/1`, latched)로 낸다. 기동 때 빈 목록도 한 번 낸다. 해석은 `rescue_contracts.overhead.parse_overhead_passages` 하나로만 한다(contracts C-04b).
- **판정 영상(OPS-189):** 웹 카메라 칸용으로 `<ns>/rescue/passage_view/compressed`(CompressedImage jpeg, BEST_EFFORT)를 낸다.
  - 바탕은 각 Pi 가 이미 내는 작은 RGB 미리보기(`<ns>/oakd/rgb/preview/image_raw/compressed`)다. 처리한 점구름마다 시각이 가장 가까운 미리보기 프레임(`view_slop_ns` 안)을 쓴다. 투영은 미리보기 camera_info 로 한다. 깊이가 RGB 카메라에 정렬돼 있어 같은 광학 frame 이고 크기만 다르다. 영상 크기가 그 camera_info 와 다르거나 frame 이 다르면 거절한다.
  - 구조물 아래 빈 공간(구조물의 x 구간, 통로 폭, 바닥 ~ 가장 낮은 점)을 투영해 반투명으로 칠한다.
  - 색과 문구는 지도와 같다(OPS-196): 초록 「통과 가능 ↑h」 / 빨강 「통과 불가 ↑h」 / 회색 「판정 불가」(바닥을 못 봄, 판정하지 못한 프레임). 카메라에만 있는 회색 「판정 중」은 그 자리가 아직 연속 횟수를 채우는 중이라는 뜻이다. 구조물이 없으면 칠하지 않는다.
  - 구독: 전체 RGB(인형 탐지 입력)는 구독하지 않는다(OPS-198). Fast DDS 유니캐스트에서는 같은 PC 의 두 번째 구독자에게도 Pi 가 Wi-Fi 로 한 번 더 보내기 때문이다 [추정]. 미리보기는 작아서 부담이 적지만, 해상도와 속도는 아직 재지 않았다. 영상을 끄려면 현장 파일에서 그 로봇의 `passage.view_image_topic`, `passage.view_camera_info_topic` 을 둘 다 지운다.
  - 맞는 RGB 프레임이 없으면 영상을 내지 않는다(다른 프레임으로 대신하지 않음). 영상 실패는 판정에 영향을 주지 않는다.
  - 한글 문구는 PIL 과 `view_font_path`(Noto Sans CJK, 각 PC 에서 `fc-list :lang=ko` 로 확인)로 쓴다. `view_image_topic` 이 빈 값이면 영상을 끄고 기동 로그에 남긴다.
  - 합성 시험에서 판 등장 → 확정 목록·초록 영상까지 0.68 s 였다(`process_period_s` 0.2 × `confirm_frames` 3). 감속 0.3 배(0.078 m/s)면 그동안 약 5 cm 움직이므로 [계산] 초록 표시가 터널 앞에서 먼저 나온다. 704→352 디코드와 그리기는 약 7 ms 다.
- **처리량:** `process_period_s` 마다 가장 최근 점구름 한 장만 보고, `stride` 행·열마다 한 점을 쓴다. 합성 704×704 점구름에서 판정 시간은 stride 1 이 41 ms, stride 2 가 9 ms, stride 4 가 2 ms 였다(개발 PC, numpy). 노드 로그는 20장마다 평균 처리 시간을 남긴다.

필수 파라미터(기본값 없음)는 `rescue_perception.overhead.OVERHEAD_PARAMS` 이고, launch 가 현장 실행 파일 `passage.*` 와 `passage.overhead.*` 에서 채운다. 시험: `tests/test_perception_overhead.py`, `tests/test_contracts_overhead.py`, `tests/test_bringup_passage.py`, `tests/ros/e3_overhead_probe.py`(격리 도메인, 실제 노드). 실제 OAK-D 점구름, 현장 PC 의 수신·처리 시간, TB-6 라이다 높이는 NOT_RUN 이다.

실기 NOT_RUN: 높이 기준을 먼저 확인해야 한다(작전 OPS-203). 판정 높이는 `base_frame`(base_link) 점의 z 이고, `lidar_plane_m` 0.23 m 는 TB-5 에서 바닥부터 줄자로 잰 값이다 [측정, 2026-10-13]. TB-6 은 같은 모델이라 같은 값으로 본 것이다 [추정]. 열린 PR #254 의 측정대로 바닥 점이 base_link 기준 약 −0.2 m 라면, 높이가 실제보다 약 0.2 m 낮게 나오고 바닥 근처 물체가 바닥으로 세어진다 [계산, 측정 아님].

### 출구 QR (OPS-136/138, 여러 출구 OPS-154/157)

[저장소 소유자 결정 2026-10-13 저녁, 작전 OPS-138 설계 승인. 여러 출구는 OPS-154, 정확 일치는 재범 확정 OPS-155] TB-6 가 지도를 탐색하다 출구 QR 을 보면 그 위치가 그 출구다. 출구는 여러 개일 수 있다(예 EXIT-1, EXIT-2). amr_target 이 대상과 같은 RGB-D 프레임에서 OpenCV `QRCodeDetector.detectAndDecodeMulti` 로 한 프레임의 QR 을 모두 읽는다. 같은 시계·CameraInfo·왜곡 보정(#164)·깊이 보정을 쓴다.

내용이 `exit_qr_payloads` 목록의 한 값과 정확히 같아야 한다. 대소문자도 같아야 하고 앞뒤 공백도 그대로 비교한다. 출구마다 따로 센다. 검사한 프레임이 그 출구를 `exit_qr_required_frames` 번 연속 읽어야 인정한다. 그 프레임들의 위치가 모두 있어야 하고, 첫 위치에서 `exit_qr_position_tolerance_m` 안이어야 한다. 검사한 프레임에 그 출구가 없거나, 깊이가 모자라거나(`exit_qr_min_valid_depth_ratio`), 한 프레임에서 두 번 읽히면 그 출구만 처음부터 다시 센다. 다른 출구의 연속 횟수는 그대로다. 목록에 없는 내용은 출구가 아니고 로그(5초 간격)만 남는다. 위치는 QR 네 꼭짓점을 왜곡 보정하고, 코드 안쪽 절반의 깊이 중앙값으로 중심을 역투영한 점이다. 시각 근거(stamp 의미)나 RGB-깊이 정렬 근거가 없으면 위치를 내지 않는다. 대상과 같은 규칙이다.

인정되면 `/rescue/exit_observations` 에 TargetObservation 하나를 낸다(새 IDL 없음). `class_label` 은 `exit_qr` 이다. `confidence` 1.0 은 확률이 아니라 내용 일치를 뜻한다. `model_id` 는 `opencv.QRCodeDetector/<cv2 버전>`, `model_sha256` 은 설치된 cv2 바이너리의 sha256 이고, 둘 다 기동 로그에 남는다. 어느 출구인지는 `track_id` 에 QR 내용 그대로 싣는다(`has_track_id` true, `tracking_mode` `TRACKING`, OPS-157). 같은 출구는 프레임이나 revision 이 바뀌어도 같은 이름이다. `evidence.evidence_id` 는 계속 프레임 근거다. 출구마다, 그리고 TB-6 지도(session, revision)마다 한 번만 낸다. revision 이 바뀌면 같은 QR 도 다시 낸다. launch 의 두 번째 target_locator(`exit_locator`)가 이것을 `/rescue/exit_location` TargetLocation 으로 바꾼다. 지도 기준은 대상과 같다(MapRevisionTracker). 조정기는 `class_label == 'exit_qr'` 만 출구로 쓰고, `observation.track_id` 로 출구를 구분한다. 지도 고정 때 이전 revision 의 위치를 재기준하는 일과 출구 앞 접근점 계산은 조정기 몫이다.

필수 파라미터(기본값 없음): `exit_qr_payloads`(STRING_ARRAY. launch 인자는 쉼표로 나눈 `exit_qr_payloads:=EXIT-1,EXIT-2` 이다. 빈 항목이나 중복이 있으면 기동하지 않는다. QR 내용에 쉼표를 쓰지 않는다. 예전 `exit_qr_payload` 는 없어졌다), `exit_qr_required_frames`([제안] 5), `exit_qr_position_tolerance_m`([제안] 0.2), `exit_qr_min_valid_depth_ratio`([제안] 0.5), `exit_qr_every_n_frames`([제안] 6: 30 Hz 에서 5 Hz 검사. 704×704 프레임 한 장에 QR 0개 15 ms, 1개 35–37 ms, 2개 45–49 ms 걸린다(detectAndDecodeMulti, OPS-141/154 실측). 이전의 한 개 검출은 QR 이 2개 보이면 하나도 읽지 못했다. 연속 횟수는 검사한 프레임 기준이고 건너뛴 프레임은 연속을 끊지 않는다), `exit_map_state_topic`(launch 가 프로파일 SCOUT 의 MapState 로 채움). 실기 확인 항목: QR 크기(한 변 20 cm 이상·무광 [제안]), 높이(카메라 높이 근처), TB-6 에서 읽히는 거리·각도, QR 면의 깊이.

시험: `tests/test_perception_exit_qr.py`(목록·출구별 판정·위치·메시지·송신), `tests/ros/e3_amr_target_probe.py` 5–8번(합성 RGB-D 의 QR → 출구 관측, 두 번째 출구는 자기 track_id). 실제 TB-6 영상의 QR, exit_locator → `/rescue/exit_location` 연결, 검출 처리 시간은 NOT_RUN 이다.

### 검증과 범위

합성 입력 시험(`tests/test_perception_center_depth.py`, `tests/test_perception_base_point.py`, `tests/ros/e3_amr_target_probe.py`)은 저장소에 있다. 로컬 검사 결과는 코드 규칙의 검증이며 실제 탐지 성능이나 ROS·GPU·로봇 실행을 증명하지 않는다. 실제 영상·Depth·TF를 이용한 검증은 제어용 컴퓨터에서 AMR 단위별 연동 시험으로 진행한다.

실제 robot6 RGB-D·CUDA 추론·GUI·다거리 보정·시간/정렬·공분산·TF/SLAM·조정기/DB·2 AMR/4 PC 인수는 NOT_RUN이다. 단일 거리 보정은 공간 정렬·시간 동기를 증명하지 않는다. 지도 변환 모듈은 아직 실제 mapping 경로에 연결되지 않았다.


### AMR 통합 전 최종 인식 파일 (2026-10-13)

[사용자 요구] “잘된 경우에서 selective 원래 코드들과 필요한 부분만 합쳐서 다시 넣어줘”,
“최종 통합해서 amr 통합전 최종 파일이라고 올리는데 그전에 baselink까지 뽑는 과정까지 잘 구현됬는지 확인해줘”.
실행 파일은 [amr_target.py](rescue_perception/amr_target.py)로 통합했다.
별도 amr_target_selective 실행 파일/entry point는 제거하고 bounded cache 정책만
[selective_depth.py](rescue_perception/selective_depth.py)에서 재사용한다.
최신 main의 clock_binding, ObserveTarget, 출구 QR와 관측 wire 형식을 유지한다.

[사용자 실기 검증] robot5에서 depth 약 1.350 m와 base XYZ가 잘 나온 진단 경로의
중심 ROI·K/D/P 보정 순서를 반영한다. “depth값은 매우 잘맞아”, “검증 됬으니 false 날려줘”는
당시 robot5 구성의 정합 확인 기록이며 다른 로봇·토픽·해상도에 자동 승계하지 않는다.
수정된 기반 노드 전체의 실제 로봇 E4 재시험은 NOT_RUN이다.

robot5/robot6 모두 `depth_calibration_path=package:robot_depth.json`을 사용하고
`physical_robot_id`는 실제 로봇으로 설정한다. SETTINGS도 타입을 유지한 ROS 파라미터로 읽으므로 physical_robot_id·base_frame·base_point_topic 등의 실행 인자가 적용된다.
기존 FIELD_PARAMS와 최신 CLOCK_PARAMS,
RECHECK_PARAMS, EXIT_PARAMS가 모두 필요하다. UNKNOWN clock을 운영 좌표 승인으로 쓰지 않는다.

합성 입력 검사는 `tests/test_perception_center_depth.py`(ROI·왜곡 보정·깊이 정책),
`tests/test_perception_base_point.py`(축 변환·capture TF·실패 재시도),
`tests/ros/e3_amr_target_probe.py`(실제 노드→base_link·재관측·QR)를 사용한다.
map 통합 시에는 촬영 시각의 map←base_link TF와 지도 session/revision·유효기간·공분산
검사를 연결해야 한다. 현재 TargetObservation은 원 camera 좌표와 ID·confidence·공분산을
보존하므로 기존 target_locator의 map←camera 경로도 그대로 사용할 수 있다.
base PointStamped만으로는 관측 ID·confidence·공분산을 전달할 수 없다.

## 기록 (2026-10-14 에 옮김, OPS-210)

아래는 위 본문에 있던 옛 설명이다. 지금 코드(main 4163a25)와 맞지 않아 이곳으로 옮겼다. 괄호 안은 그 설명을 쓴 날과 옮기기 전 자리다. 화살표 뒤가 지금 상태다.

- (2026-10-08, 맨 위 소개 문단) 원문: 모델 추론, ROS 노드, 카메라 구독, 압축 영상 해제, TF, 주행 명령은 구현하지 않았다.
  → 지금: 주행 명령만 그대로 없다. 나머지는 「패키지 개요」의 ROS 노드들이 한다(amr_camera 의 YOLO 추론, amr_target 의 카메라 구독·압축 해제·촬영 시각 TF 등).
- (2026-10-11, OAK-D 학습 계획 문단) 원문: 학습·실제 카메라 시험은 아직 NOT_RUN이다.
  → 지금: [선택 manifest](../../assets/manifest/yolo26n-survivor-v2.json)(2026-10-11)로 학습과 모델 선택이 끝났다.
- (2026-10-12, AMR 단위 검증 노드 절) 원문: 이번 PR은 Draft이며 실제 로봇 인수 완료를 뜻하지 않는다.
  → 지금: 그 PR 은 이미 머지됐다. 실제 로봇 인수는 여전히 끝나지 않았다(「현재 상태」 표).
- (2026-10-12) 원문: [사용자 요구, 2026-10-12] AMR 환경별 YAML·projection template와 필수 ROS 파라미터 선언/읽기 코드를 제거하고 기존 utils의 SETTINGS에 필수 설정을 모았다. 노드는 설정을 가져와 그대로 실행한다. 기존 main의 웹캠·bag 코드와 웹캠 설정은 유지한다.
  → 지금: 지금 amr_target 은 토픽·frame·시계·정렬·짝 맞추기 값을 기본값 없는 ROS 파라미터로 받는다(위 「작전 B1」 문단, `FIELD_PARAMS`·`CLOCK_PARAMS`·`RECHECK_PARAMS`·`EXIT_PARAMS`). SETTINGS 는 나머지 값에만 쓴다.
- (2026-10-12) 원문: [사용자 요구, 2026-10-12] 실제 실행 대상은 README 시나리오의 로봇 제어용 컴퓨터다. 기준 환경은 MSI RTX 4070 노트북·Ubuntu 24.04·ROS 2 Jazzy다. 이 저장소에서는 베이스와 테스트 코드를 작성하며 작성 환경에서 ROS 노드를 실행하는 것을 전제하지 않는다. ROS가 없는 환경에서 노드 import를 허용하기 위한 함수 내부 import와 의존성 오류 호환 분기를 제거했다. 순수 설정·보정·검사 시험은 utils를 직접 import한다.
  → 지금: 지금은 작성 환경에서도 격리 도메인 ROS 시험(`tests/ros/e3_*`)으로 실제 노드를 띄운다. 일부 모듈은 함수 안에서 import 한다(예: `exit_qr.py:detect_qrs` 의 cv2, `headroom.py:depth_points` 의 numpy).
- (2026-10-13, 설정 문단) 원문: clock/stamp/alignment는 UNKNOWN·미확인을 유지한다.
  → 지금: 시계는 이 호스트의 측정값을 쓴다(위 「관측 시계 (OPS-103)」). stamp 의미와 정렬 근거는 기본값 없는 실행 인자다(「작전 B1」 문단).
- (2026-10-12, 상대 좌표 출력 절) 원문: 현재 UNKNOWN clock·미검증 정렬 설정에서는 유효 XYZ를 발행한다고 가정할 수 없다. 실제 입력의 근거를 확인한 뒤 사용해야 한다.
  → 지금: 시계·정렬은 위와 같이 바뀌었다. 실제 입력의 근거를 확인한 뒤 써야 한다는 점은 그대로다(「작전 B1」 문단).
- (2026-10-12, 검증과 범위 절) 원문: [사용자 요구, 2026-10-12] 이번 AMR 합성 입력 시험 3개는 작성 환경의 로컬 검사 전용으로 보관하고 PR에는 포함하지 않는다. 기존 main의 단위시험·CI는 유지한다.
  → 지금: 세 시험은 지금 저장소에 있다(위 「AMR 통합 전 최종 인식 파일」).

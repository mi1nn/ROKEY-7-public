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
| [amr_target.py](rescue_perception/amr_target.py) | RGB·Depth·CameraInfo → 기존 PersonPipeline·adapter·core → TargetObservation. ROS·영상·모델 의존성은 파일 상단에서 import한다 |
| [camera_utils.py](rescue_perception/camera_utils.py) | 공통 SETTINGS, 모델 SHA/class·RGB 형식 검사, 복호화·추론·annotated Image 생성 |
| [target_utils.py](rescue_perception/target_utils.py) | RGB-D SETTINGS·target_config, YOLO 관측 메시지, Depth 복호화·robot6 보정 |
| [map_projection.py](rescue_perception/map_projection.py) | 지도 상태·정렬·관측 시각 TF 검사, 지도/mission 좌표·공분산 변환. 코드와 합성 시험만 보존하며 실행 노드에 연결하지 않음 |
| [robot6_depth.json](rescue_perception/config/robot6_depth.json) | robot6 보정계수·원 측정값·날짜·PR #126 출처·적용 한계. 설치본에도 포함 |

[사용자 요구, 2026-10-12] AMR 환경별 YAML·projection template와 필수 ROS 파라미터 선언/읽기 코드를 제거하고 기존 utils의 SETTINGS에 필수 설정을 모았다. 노드는 설정을 가져와 그대로 실행한다. 기존 main의 웹캠·bag 코드와 웹캠 설정은 유지한다.

[사용자 요구, 2026-10-12] 실제 실행 대상은 README 시나리오의 로봇 제어용 컴퓨터다. 기준 환경은 MSI RTX 4070 노트북·Ubuntu 24.04·ROS 2 Jazzy다. 이 저장소에서는 베이스와 테스트 코드를 작성하며 작성 환경에서 ROS 노드를 실행하는 것을 전제하지 않는다. ROS가 없는 환경에서 노드 import를 허용하기 위한 함수 내부 import와 의존성 오류 호환 분기를 제거했다. 순수 설정·보정·검사 시험은 utils를 직접 import한다.

[설정/사용자 요구, 2026-10-12] 첫 CUDA GPU `device: '0'`, 탐지 신뢰도 `model_conf: 0.5`, 기존 대상 확정 정책 `confirm_confidence_min: 0.8`, `physical_robot_id: robot6`, `depth_calibration_path: package:robot6_depth.json`. 그 외 토픽·frame 이름과 최신성·깊이 임계값은 이전 예시의 [제안]이며 실기 검증 전이다. clock/stamp/alignment는 UNKNOWN·미확인을 유지한다. 코드 통합은 이 값을 측정 사실로 바꾸지 않는다.

[사용자 요구, 2026-10-12] robot6 보정 데이터와 출처는 JSON 한 곳에 둔다. [계산, 2026-10-12] PR #126의 단일 82cm 기록에서 scale = 0.82 / mean(0.854, 0.853, 0.857), offset=0. RGB는 유지하고 Depth를 m로 변환한 뒤 원 0·음수·NaN·Inf를 제외한다. raw_m * scale + offset_m을 한 번 적용하고 범위·불연속 검사·중앙값으로 optical XYZ를 계산한다. raw/PNG compressedDepth, 16UC1/mm·32FC1/m 모두 같은 보정을 사용한다. 다른 physical_robot_id는 보정 로드 시 거절하며 이미 보정된 입력은 사용하지 않는다.

### 빌드와 실행

ROS·OpenCV·cv_bridge·Ultralytics와 CUDA 사용 가능한 추론 환경이 필요하다. 모델은 [선택 manifest](../../assets/manifest/yolo26n-survivor-v2.json)의 best.pt다. 실제 실행은 로봇 제어용 컴퓨터의 저장소 루트에서 준비한다:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rescue_contracts rescue_mapping rescue_interfaces rescue_perception \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 run rescue_perception amr_camera
# camera 노드를 종료한 뒤 target 노드를 따로 시험한다.
ros2 run rescue_perception amr_target
```

YAML이나 `--params-file`은 필요 없다. ROS 표준 namespace/topic remap은 사용할 수 있다. 모델 경로는 실행 작업 디렉터리 기준이다. GUI는 SETTINGS의 show=True에서만 켠다. GPU 추론 실패를 CPU 추론으로 숨기지 않는다.

RGB·Depth는 원 stamp로 동기화한다. CameraInfo frame/해상도·최신성, 모델 SHA/class, Depth 유효값·정렬·시각 검사는 유지한다. [사용자 요구, 2026-10-12] 지도 변환은 map_projection.py로 분리해 코드로만 유지하고, 기존 rescue_mapping과의 연결은 후속으로 진행한다. amr_target은 TargetObservation과 annotated Image만 발행하며 MapState 구독·TF 조회·TargetLocation publisher가 없다. 별도 지도 변환 모듈에도 ROS 노드·실행 등록은 없다. 후속 연결에서는 기존 mapping의 fresh MapState·clock·alignment·관측 시각 TF·공분산 근거를 제공하고 검사해야 한다. 현재 합성 함수 시험은 실제 mapping 연동을 증명하지 않는다. 주행 명령은 보내지 않는다.

### 시험과 범위

- tests/test_perception_amr_camera.py: 코드 설정·GPU/신뢰도 전달·모델 SHA/class·RGB 형식·원 헤더 보존·추론 실패.
- tests/test_perception_amr_target.py: YOLO 관측·optical 좌표, 분리된 지도 변환 함수의 합성 mission 좌표·clock/map revision/TF·공분산 회전.
- tests/test_perception_depth_calibration.py: 코드 기본값과 robot6 프로파일 연결·mm/m XYZ·무효값·로봇 불일치·설치 리소스.

```bash
PYTHONPATH=src/rescue_contracts:src/rescue_mapping:src/rescue_perception \
  python3 -m unittest discover -s tests -p 'test_perception*.py' -v
```

실제 robot6 RGB-D·CUDA 추론·GUI·다거리 보정·시간/정렬·공분산·TF/SLAM·조정기/DB·2 AMR/4 PC 인수는 NOT_RUN이다. 단일 거리 보정은 공간 정렬·시간 동기를 증명하지 않는다. 지도 변환 모듈은 아직 실제 mapping 경로에 연결되지 않았다.

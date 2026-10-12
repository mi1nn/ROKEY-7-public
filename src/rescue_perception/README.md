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

저장소 루트에서:

```sh
PYTHONPATH=src/rescue_contracts:src/rescue_perception python3 -m unittest discover -s tests -p 'test_perception*.py' -v
```

API·제한·원본 근거는 [구현 문서](../../docs/implementation/implementation-perception.md)를 따른다. 이 상대 링크는 저장소 checkout 기준이며 설치된 share 디렉터리는 저장소 문서를 포함하지 않는다.

원본 코드를 복사하지 않고 선별한 원리를 새 계약에 맞춰 구현했다. 버전 `0.0.0`, `LicenseRef-Unspecified`, 임시 maintainer는 기존 초기 구조의 미결정 메타데이터다. 이는 사용 허가나 실제 담당자를 뜻하지 않으며 배포 전에 확정해야 한다.

[OAK-D 학습 계획](../../docs/implementation/oakd-survivor-training-plan.md): 사용자 결정(2026-10-11)에 따라 이번 학습은 `oakd/image_raw` 영상의 1초 간격 프레임을 전처리해 단일 클래스 `survivor`(ID 0)를 사용한다. 후속 연동의 모델 class metadata·manifest·`ConfirmationPolicy.person_labels`·Detector 출력 label을 맞춘다. 기존 API 이름과 과거 person 합성 시험은 유지한다. 학습·실제 카메라 시험은 아직 NOT_RUN이다.


## AMR 단위 검증 노드 (2026-10-12)

[사용자 요구] 기존 인식 코어에 AMR 알고리즘을 연결하고 amr_camera·amr_target을 각각 시험·보정한 뒤 기존 코드의 후속 수정·통합을 진행한다. 이번 PR은 Draft이며 실제 로봇 인수 완료를 뜻하지 않는다. 기존 웹캠·bag 코드는 유지한다.

- `amr_camera_node`: RGB raw/compressed → YOLO 1회 → 원 헤더를 보존한 annotated Image. GUI는 show=true에서만 사용한다.
- `amr_target_node`: 동기화한 RGB·Depth + CameraInfo → 기존 PersonPipeline·adapter·core → TargetObservation. 충분한 신뢰도·위치·fresh clock/map/alignment·관측 시각 TF·공분산 근거를 만족할 때만 TargetLocation.
- `amr_camera_support`: 설정·모델 SHA·단일 survivor 클래스 검사와 RGB 복호화/추론을 두 노드가 공유한다.
- `amr_target_support`: YOLO 결과 변환·관측/위치 메시지 작성·map revision/TF/공분산 검사·compressedDepth PNG 복호화·robot6 보정 로드를 한 모듈에 모았다. ROS는 노드 경계에서, OpenCV/NumPy 복호화 의존성은 함수 안에서 import한다.

### 설정 파일

| 파일 | 담는 값 |
| --- | --- |
| [amr_camera_node.example.yaml](config/amr_camera_node.example.yaml) | RGB 토픽·raw/compressed·출력 토픽, 모델 경로/SHA/class/imgsz/conf/device, GUI·단절 경고 |
| [amr_target_node.example.yaml](config/amr_target_node.example.yaml) | RGB·Depth·CameraInfo·MapState/출력 토픽, 로봇/카메라/frame ID, 시각 의미·동기 허용치, 깊이·확정·최신성 정책, 좌표 발행·공분산·지도 정렬 조건 |
| [amr_target_node.robot6-depth.yaml](config/amr_target_node.robot6-depth.yaml) | physical_robot_id=robot6와 package:robot6_depth.json을 선택하는 override. 보정계수 중복 없음 |
| [amr_projection.template.json](config/amr_projection.template.json) | mapping 담당이 채울 map/mission frame·session/revision·authority·변환·유효기간·오차·공분산 근거. 기본 INVALID라 그대로 사용 불가 |
| [robot6_depth.json](rescue_perception/config/robot6_depth.json) | robot6 보정계수·offset·원 측정값·날짜·PR #126 출처·사용자 승인·한계. 설치본에도 포함 |

[사용자 요구] 2026-10-12 “이것은 robot6_depth로 수정해서 한개의 파일로 합쳐주고”. 보정 데이터는 robot6_depth.json 한 곳에 둔다. 별도 assets manifest 참조는 제거했다. 파일은 저장소 안의 패키지 자산이며 출처·측정·적용 범위는 JSON 안에 보존한다.

[계산, 2026-10-12] scale = 0.82 / mean(0.854, 0.853, 0.857), offset=0. RGB는 그대로 두고 Depth를 m로 변환한 뒤 원 0·음수·NaN·Inf를 제외한다. corrected_m = raw_m * scale + offset_m을 한 번 적용하고 보정된 범위·불연속 검사·중앙값으로 optical XYZ를 계산한다. 16UC1/mm·32FC1/m, raw·PNG compressedDepth 경로가 같은 계산을 사용한다. 원 Depth 토픽을 재발행하지 않는다.

보정 프로파일 physical_robot_id가 실행 설정과 다르면 기동을 거절한다. 다른 로봇의 보정 기본은 identity다. 이미 보정된 입력에 다시 적용하면 이중 보정이므로 raw 입력을 사용한다. 이 계수는 단일 82cm 측정에서 계산한 것이며 다거리 정확도·RGB-Depth 정렬·시간 동기를 증명하지 않는다.

[사용자 요구, 2026-10-12] “실제 환경에 맞게 바꿔줘  일단 cpu 말고 gpu 쓰고 있고 confi도 탐지는 0.5이상으로 해주고”. 두 AMR 실행 YAML에 `device: '0'`(첫 번째 CUDA GPU)와 `model_conf: 0.5`를 적용했다. ROS 문자열 타입을 유지한다. CUDA 사용 가능 여부와 실제 GPU 추론은 실기에서 확인해야 한다. 탐지와 별개인 `confirm_confidence_min: 0.8`은 기존 확정 정책이다. 토픽·frame·정렬·시각 값은 확인 근거가 없으므로 실제 환경 검증이 필요하다.

### 빌드와 실행

ROS·OpenCV·cv_bridge·Ultralytics는 실행 환경에서 준비해야 한다. 버전 목록 파일은 사용자 요청으로 제거했으며 자동 설치·로봇 패키지 변경은 하지 않는다. 저장소 루트에서:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rescue_contracts rescue_mapping rescue_interfaces rescue_perception \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 run rescue_perception amr_camera_node --ros-args \
  --params-file src/rescue_perception/config/amr_camera_node.example.yaml
# camera 노드를 종료한 뒤 좌표 노드를 따로 시험한다.
ros2 run rescue_perception amr_target_node --ros-args \
  --params-file src/rescue_perception/config/amr_target_node.example.yaml \
  --params-file src/rescue_perception/config/amr_target_node.robot6-depth.yaml
```

모델은 [선택 manifest](../../assets/manifest/yolo26n-survivor-v2.json)의 best.pt를 사용한다. 예시 토픽·frame·시간·정렬·공분산 값은 실제 입력과 근거로 별도 설정해야 한다. robot6 override는 거리 보정만 선택하고 locations_enabled:false·UNKNOWN·정렬 미확인 등의 관문을 해제하지 않는다. 유효 지도 좌표 발행에는 mapping 권한자의 fresh MapState·FrameAlignment snapshot·관측 시각 TF가 필요하다.

RGB·Depth는 stamp로 동기화한다. sync_slop_ns=0은 exact synchronizer다. CameraInfo는 frame/해상도와 최신 stamp를 요구한다. Depth는 ConfigHeader+PNG만 지원하며 RVL·RGB JPEG·컬러화 depth는 거절한다. 32FC1은 전송 양자화로 원 float와 완전히 같지 않다. 최신 TF나 static-zero 결과로 원 관측 시각을 대체하지 않는다. 실패는 UNKNOWN 관측 또는 위치 발행 차단·이유 로그로 드러난다.

### 시험과 범위

- tests/test_perception_amr_camera.py: 필수 설정·모델 SHA/class·RGB 형식·원 헤더 보존·빈 탐지/추론 실패.
- tests/test_perception_amr_target.py: YOLO 결과·optical/mission 좌표·메시지 provenance·clock/map revision/TF·공분산 회전.
- tests/test_perception_depth_calibration.py: mm/m 보정 XYZ·무효값 보존·로봇 불일치·단일 패키지 프로파일·설치 설정 연결.

```bash
PYTHONPATH=src/rescue_contracts:src/rescue_mapping:src/rescue_perception \
  python3 -m unittest discover -s tests -p 'test_perception*.py' -v
```

[사용자 요구, 2026-10-12] 작은 helper를 amr_target_support로 합치고 두 AMR smoke 도구·이미지 파일 infer 도구·버전 목록은 제거한다. 단위시험과 실제 실행 노드는 유지한다. 제거한 도구의 실행 명령은 현재 인터페이스가 아니다.

현재 head의 실제 robot6 RGB-D·GUI·다거리 보정·시간/정렬·센서 공분산·TF/SLAM·조정기/DB·중요 사건 receipt·2 AMR/4 PC 인수는 NOT_RUN이다. covariance 근거는 보정 후 좌표의 분산을 설명해야 한다. TargetLocation 발행은 저장 보장·자동 접근 승인이 아니다. 기존 full-frame Python Depth 검사의 실기 처리량은 측정이 필요하다.

# rescue_perception

현재 AMR 추가 구현은 `amr_camera_node`와 `amr_target_node`다. 기존 PersonPipeline·adapter·core를 재사용하며, 기존 웹캠 노드와 bag 재생 경로도 유지한다. 아래 기존 코어 설명의 제한과 실제 AMR 실기 NOT_RUN은 구분한다.

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


## YOLO 영상 미리보기 기반 작업

[사용자 요구, 2026-10-12] “그럼 이제 내가 기반 코드를 만들어보자”. 적용 범위는 인식 레인의 RGB 영상 구독 → YOLO 추론 → 박스·클래스·신뢰도 표시다. 토픽·모델 경로는 실행 시 지정한다. 영상 미리보기는 위치·관측 사건·주행 승인과 연결하지 않는다. 실행 구현과 검증 결과는 이 절에 기록한다.

[사용자 요구, 2026-10-12] “일단 compressed 기반으로 코드를 고쳐줘”. 예시 입력과 기본 smoke 시험을 RGB compressed로 바꾼다. raw는 명시적인 비교용으로 유지한다. Depth compressedDepth는 이 RGB 경로에서 거절하며 좌표 노드의 `compressed_depth.py`가 전용 복호화를 담당한다.


### 실행 구조

`amr_camera_support.py`는 ROS 없이 설정·SHA-256·단일 survivor 클래스와 프레임 처리를 검증한다.
`amr_camera_node.py`는 ROS·cv_bridge·OpenCV·Ultralytics 경계다. PR #113의 필수 파라미터,
모델 해시 확인·기동 설정 로그·실패 로그 구조를 참고해 새로 구현했다(원본 코드 복사 없음).
각 파라미터에 타입과 read-only descriptor를 지정했다. 누락·잘못된 타입·잘못된 해시·class 불일치는 기동 실패(exit 2)다.
입력은 BEST_EFFORT/VOLATILE/depth 1, 출력 Image는 RELIABLE/VOLATILE/depth 1이다.
단일 executor로 처리하며 처리 중 큐는 한 프레임만 둔다. 실제 종단 지연·FPS는 아직 미측정이다.
원 `header.stamp`의 정수 sec/nanosec와 `frame_id`를 결과 영상에 보존한다. 원 stamp가 0이면 그대로
보존할 뿐 유효 관측으로 승인하지 않는다. 영상 실패·추론 실패는 error 로그와 출력 없음으로 드러내고,
입력 단절 또는 연속 처리 실패는 timeout warning으로 드러낸다. 성공한 빈 탐지는 박스 없는 영상을 발행한다. RGB 압축은 JPEG/PNG 형식을 확인하고 compressedDepth·16UC1·32FC1·mono16 입력은 YOLO에 넘기기 전에 거절한다. 압축 복호화는 수신 PC에서 수행하며, 로봇에서 raw를 추가 구독하지 않는다. 결과 Image는 PC에서 확인한다. 원격 viewer를 구독시키면 추가 raw 전송량이 발생하므로 Wi-Fi 반대편으로 보내지 않는다.

### 환경과 카메라 실행

ROS 환경을 source하고 ROS Python과 호환되는 Python 환경에서 실행한다. PC/로봇의 시스템 패키지·
Discovery 설정·서비스는 이 작업에서 바꾸지 않는다. 로컬 `.venv`에 Ultralytics·CPU torch를 준비해 실제 best.pt 파일 추론과 recorded-image compressed ROS 미리보기 경로를 확인했다. 실제 AMR 카메라 연동은 NOT_RUN이다.
필요하면 별도 가상환경을 준비한다(시스템 Python에 설치하지 않음). 모델 학습 환경은
Ultralytics 8.4.170이며 그 버전과 모델을 함께 확인한다.

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m venv --system-site-packages .venv
. .venv/bin/activate
PIP_CACHE_DIR=/tmp/amr-yolo-pip-cache python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
PIP_CACHE_DIR=/tmp/amr-yolo-pip-cache python -m pip install -r tools/requirements-amr-yolo.txt
mkdir -p .artifacts/ultralytics-config
export YOLO_CONFIG_DIR="$PWD/.artifacts/ultralytics-config"
python -c 'import rclpy, cv_bridge, cv2, ultralytics; print(ultralytics.__version__)'
```

이미 카메라를 볼 수 있는 ROS 환경에서 토픽·메시지 타입·QoS를 먼저 확인한다. raw는
`sensor_msgs/msg/Image`, compressed는 `sensor_msgs/msg/CompressedImage`다. 로봇 번호나 실제 토픽을 예시에서 추정하지 않는다.

```bash
ros2 topic list -t
ros2 topic type <actual_rgb_topic>
ros2 topic info -v <actual_rgb_topic>
ros2 topic hz <actual_rgb_topic>
```

ROS 환경을 source한 뒤 PYTHONPATH에 기존 ROS 경로를 덧붙여 보존한다. 시스템 ROS 시험은 `/usr/bin/python3`로 실행했고, 가상환경은 위처럼 시스템 Python으로 생성한다. 이 checkout 루트에서 다음처럼 실행한다. `<actual_rgb_compressed_topic>`은 확인한 실제 RGB 압축 토픽으로 바꾼다.
예시 설정의 `image_type`은 `compressed`다. raw 비교 시 토픽과 `image_type:=raw`를 함께 바꾼다. `show:=true`는 GUI가 있는 PC에서만 사용한다.

```bash
PYTHONPATH=src/rescue_contracts:src/rescue_perception:$PYTHONPATH python -m rescue_perception.amr_camera_node --ros-args \
  --params-file src/rescue_perception/config/amr_camera_node.example.yaml \
  -p image_topic:=<actual_rgb_compressed_topic> -p show:=true
```

`AMR YOLO` 창의 survivor 박스·신뢰도, 시작 로그의 실제 토픽·모델 해시, 처리 FPS·오류를 확인한다.
GUI 없이 `show:=false`로 실행한 경우 `rqt_image_view`에서 `/perception/annotated`를 선택한다.
node namespace를 넣으면 상대 출력 토픽도 그 namespace 아래로 이동한다.

```bash
ros2 topic hz /perception/annotated
ros2 run rqt_image_view rqt_image_view
```

`colcon build --packages-select rescue_contracts rescue_perception` 후 `install/setup.bash`를 source하면
`ros2 run rescue_perception amr_camera_node --ros-args --params-file <parameter_yaml>`로도 실행할 수 있다.
해시는 [선택 모델 manifest](../../assets/manifest/yolo26n-survivor-v2.json)의 uploaded_model 값과 일치해야 한다.

### 테스트 코드와 다음 단계

`tests/test_perception_amr_camera.py`: E2 합성 시험. 필수 설정·모델 해시·class 불일치, raw/compressed
분기·추론 인자·정확한 원 헤더 보존·정상 빈 탐지와 추론/복호화 실패를 구분한다.
`tools/amr_camera_smoke.py`: ROS·cv_bridge·영상 전송은 실제 실행하고 모델만 명시적으로 가짜를
주입하는 합성 시험이다. 실제 YOLO 탐지 성능·카메라·GUI 성공으로 보고하지 않는다.
이 checkout 루트에서 로봇과 분리된 로컬 시험 환경으로 실행한다. 예시 도메인은 시험용이며 장비 도메인이 아니다.

```bash
PYTHONPATH=src/rescue_contracts:src/rescue_perception python3 -m unittest discover -s tests -p 'test_perception_amr_camera.py' -v
ROS_DOMAIN_ID=137 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DISCOVERY_SERVER= \
  PYTHONPATH=src/rescue_contracts:src/rescue_perception:$PYTHONPATH /usr/bin/python3 tools/amr_camera_smoke.py --image-type raw
ROS_DOMAIN_ID=137 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DISCOVERY_SERVER= \
  PYTHONPATH=src/rescue_contracts:src/rescue_perception:$PYTHONPATH /usr/bin/python3 tools/amr_camera_smoke.py
```

[사용자 요구, 2026-10-12] “그리고 이게 잘되면 yolo와 rgb depth까지 띄우는 test 코드를 만들어보자”.
실제 AMR RGB에서 첫 단계 성공을 확인한 뒤 YOLO·RGB·Depth 동시 표시 시험을 추가한다.
Depth 토픽·encoding·단위·RGB 정렬과 시각 차이를 확인한 뒤 구성하며 이 단계에서 거리를 지어내지 않는다.
현재 Depth 표시·RGB-D/CameraInfo 정합·TF/XYZ·PersonPipeline·관제·주행 연동은 NOT_RUN/후속이다.

검증 기록 [측정, 2026-10-12]: 전체 단위시험 842개 OK(exit 0), 문서 검사 PASS(exit 0), 두 패키지 colcon build PASS(exit 0). 실제 YOLO 모델·AMR 카메라·GUI·Depth·장시간 부하 시험은 NOT_RUN. ROS raw·compressed 합성 전송 각각 PASS(exit 0): 원 헤더 보존·첫 추론 실패 프레임 출력 없음·후속 영상 3개 수신. 모델만 fake로 주입했으며 실제 ROS·cv_bridge를 실행했다. 실제 best.pt 기동은 ultralytics 미설치로 정상적으로 차단(exit 2)됨을 확인했다.

compressed 전환 검증 [측정, 2026-10-12]: 전체 단위시험 843개 OK(exit 0), compressed 기본 ROS 합성 시험 PASS(exit 0), 두 패키지 재빌드 PASS(exit 0), 소스·설치본 compressed YAML 값 일치 PASS, 문서·공백 검사 PASS. 앞의 842개는 전환 전 기록이다. 실제 모델·AMR 카메라·GUI·Depth 연동은 여전히 NOT_RUN.


## AMR RGB-D 탐지·좌표 발행 노드

[사용자 요구, 2026-10-12] “박세준님: AMR 탐지 노드 (best.pt로 탐지해서 좌표를 TargetLocation으로 발행) 오늘 최종목표는 이거야”, “git 전체 구도를 보고 노드용 코드는 예시가 있는지 찾아보고 있으면 그틀에 맞게 하고 아니면 만들어줘”. 적용 범위는 인식 레인에서 로컬 구현·시험이며 push/PR는 하지 않는다.

### 확인한 틀과 구현 경계

main `c79097d`의 `rescue_robot/ros_node.py`, `rescue_operations/ros_node.py`, `rescue_coordination/ros_node.py`는 ROS 입출력과 순수 로직을 분리한다. 열린 [PR #113](https://github.com/jaebeom/ROKEY-7/pull/113)의 웹캠 노드는 필수 파라미터·모델 해시·기동/실패 로그 틀이다. `tests/ros/e3_coordinator_probe.py`에는 합성 TargetLocation 발행 예시가 있으나 카메라/Depth에서 얻은 좌표를 발행하는 구현은 아니다. `src/`, `tools/`, `tests/ros/`와 GitHub main 트리·열린 PR 목록을 TargetLocation/create_publisher/lookup_transform으로 확인했다. main의 mapping은 순수 Python 코어이며 카메라→TargetLocation ROS 발행 노드는 없었다.

새 `amr_target_node`는 그 틀을 따라 기존 `PersonPipeline`, `SourceBinding`, `AlignmentEvidence`, `DepthPolicy`, `TimePolicy`를 재사용한다. `amr_target_support`는 YOLO 결과 변환·원 관측 메시지·좌표 검증/변환을 담당한다. `compressed_depth`는 Depth PNG 전용 복호화 경계다. 기존 `.msg`, mapping 파일, 로봇/PC 설정은 수정하지 않았다.

흐름은 RGB 압축 복호화 → YOLO 1회 → 같은 RGB stamp에 탐지 연결 → RGB/Depth 동기·CameraInfo·정렬 검증 → bbox 유효 깊이 중앙값·카메라 XYZ → TargetObservation이다. 높은 confidence와 유효 위치를 모두 만족한 후보만 지도 변환 단계로 넘긴다. 여러 bbox는 각각 원 관측 ID를 가진다. tracking을 실행하지 않으며 대상 병합은 coordinator 책임이다.

기존 계약에서 TargetLocation 생산 책임은 mapping이다. 이번 노드의 `locations_enabled`는 현장 시험을 위해 mapping 입력·변환 경계를 같은 프로세스에 조합한 선택적 bridge다. `rescue_mapping.RigidTransform`과 기존 `FrameAlignment`를 사용하고 mapping 권한자의 메타데이터를 읽기만 한다. 정식 배포에서 mapping 노드로 이 경계를 옮기는 것은 기반/통합 담당 후속이다. 카메라 optical XYZ를 mission_position에 그대로 넣지 않는다.

### 좌표를 발행하는 조건

- PersonPipeline의 CONFIRMED(설정 confidence 하한 + LOCATED).
- 명시된 시간 기준·SYNCED·측정된 clock 오차·원 observed_at/valid_until의 최신성.
- mapping 권한자가 제공한 정렬 snapshot의 authority·robot·clock domain·유효기간·오차 상한.
- snapshot과 같은 robot/frame/session/revision의 fresh MapState. `correction_detection_status=KNOWN`, `localization_quality=VALID`가 필요하다. snapshot만 넣고 실제 지도 상태를 지어내지 않는다.
- 원 RGB 관측 시각으로 TF lookup. 결과 frame·stamp가 요청과 맞아야 한다. latest TF와 static-zero 결과 대체는 지원하지 않는다. tf2 내부 보간 표본의 EXACT 증거를 만들어내지는 않는다.
- covariance 검증 근거와 설정 분산 일치. 센서 공분산은 좌표 회전에 맞춰 변환하고 정렬 위치/회전 오차의 보수적 상한을 추가한다. 이 계산은 실측 센서 공분산을 만들어주는 기능이 아니다.

검증 실패 시 TargetLocation을 발행하지 않고 이유를 로그에 남긴다. 깊이 부족 후보는 원 관측을 UNKNOWN과 NaN 위치로 보존하여 유효 원점(0,0,0)처럼 보이지 않게 한다. evidence SHA는 복호화 RGB 픽셀 바이트의 해시이며 실제 사진 저장/업로드 성공을 뜻하지 않는다.

### 설정과 실행

`config/amr_target_node.example.yaml`의 토픽/frame/보정/정렬/시간 값은 교체가 필요한 예시다. 임계값·공분산은 제안이며 실기 합격 기준이 아니다. `UNKNOWN`, 정렬 미확인, `locations_enabled:false` 상태에서 유효 좌표를 발행하지 않는다. `config/amr_projection.template.json`은 INVALID 템플릿이며 그대로 기동용 projection snapshot으로 쓰면 거절한다.

mapping 담당에게 현재 FrameAlignment(로봇·지도 세션·revision·transform revision·authority·유효기간·오차·변환), 검증된 센서 분산과 근거를 받아 별도 JSON을 준비한다. 그 경로를 `projection_metadata_path`에 지정하고 실제 MapState 토픽도 연결한 뒤 `locations_enabled:true`로 실행한다. 단순히 verified/SYNCED를 true로 바꾸는 것은 측정이 아니다. 지도 revision/세션이 바뀌면 기존 snapshot으로 발행이 차단되며 새 snapshot을 받아 노드를 재기동한다. 낮은 revision·retired session 재전송은 현재 지도 상태를 되살리지 못하도록 거절한다.

ROS와 Ultralytics가 준비된 환경에서 이 checkout 루트 기준:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rescue_contracts rescue_mapping rescue_interfaces rescue_perception \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 run rescue_perception amr_target_node --ros-args \
  --params-file src/rescue_perception/config/amr_target_node.example.yaml
```

가상환경의 YOLO를 쓸 때는 ROS와 호환되는 시스템 Python 기반 환경이어야 한다. 소스 실행은 같은 환경에서 아래처럼 한다.

```bash
PYTHONPATH=src/rescue_contracts:src/rescue_mapping:src/rescue_perception:$PYTHONPATH \
  python -m rescue_perception.amr_target_node --ros-args \
  --params-file src/rescue_perception/config/amr_target_node.example.yaml
ros2 topic echo /rescue/target_locations --once
```

RGB와 Depth는 입력 stamp로 동기화하며 queue 크기·skew는 파라미터다. skew=0은 exact TimeSynchronizer를 쓴다. CameraInfo도 같은 RGB frame/크기와 fresh stamp를 요구한다. 압축 Depth는 ROS compressed_depth_image_transport의 ConfigHeader+PNG 형식을 지원한다(16UC1/32FC1, byte order 명시). RVL·일반 JPEG·컬러화 depth는 거절한다. 32FC1 복원은 전송 시 양자화되어 원본 float와 완전히 같지 않다. [공식 codec](https://github.com/ros-perception/image_transport_plugins/blob/rolling/compressed_depth_image_transport/src/codec.cpp), [공식 header](https://github.com/ros-perception/image_transport_plugins/blob/rolling/compressed_depth_image_transport/include/compressed_depth_image_transport/compression_common.hpp).

### 시험과 남은 검증

`tests/test_perception_amr_target.py`는 YOLO 결과 어댑터·기존 PersonPipeline·optical XYZ·mission 변환·메시지 provenance, 잘못된 Depth/clock/map/revision/TF와 covariance 회전을 합성 입력으로 시험한다.
`tools/amr_target_smoke.py`는 실제 ROS 메시지·message_filters·cv_bridge·RGB JPEG·Depth PNG·TargetLocation 직렬화/수신을 실행한다. 모델은 명시적인 synthetic fixture로 주입한다. 기본 TF 시험은 합성 TransformStamped를 실제 로컬 broadcaster/listener/buffer에 넣는다. `--tf-mode injected`는 주입 lookup 비교용이며 실제 로봇 TF는 두 모드 모두 미검증이다. 좌표 (1.5,1.5,2.0)는 계산 fixture이며 실측 위치가 아니다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=171 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DISCOVERY_SERVER= \
  PYTHONPATH=src/rescue_contracts:src/rescue_mapping:src/rescue_perception:$PYTHONPATH \
  /usr/bin/python3 tools/amr_target_smoke.py
ROS_DOMAIN_ID=172 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DISCOVERY_SERVER= \
  PYTHONPATH=src/rescue_contracts:src/rescue_mapping:src/rescue_perception:$PYTHONPATH \
  /usr/bin/python3 tools/amr_target_smoke.py --depth-encoding 32FC1
```

NOT_RUN: 실제 best.pt 추론, AMR RGB/Depth/CameraInfo 정합, 실제 TF·SLAM 보정 감지·시간 동기·공분산 측정, GUI, 종단 FPS/지연·장시간 부하, coordinator/DB·중요 사건 outbox/receipt 연결. TargetLocation은 현재 ROS 발행까지이며 저장 보장·자동 접근 승인은 아니다. 기존 E2 DepthFrame은 bbox마다 full-frame Python 검사를 수행하므로 실기 처리량은 측정해야 한다. 헤더 보존과 합성 PASS를 실기 완료로 확대하지 않는다.

최종 검증 [측정, 2026-10-12]: 전체 단위시험 851개 OK(exit 0). AMR 미리보기/좌표 전용 단위시험은 그중 16개다. ROS 합성 compressedDepth 16UC1·32FC1, exact 동기화·지도 revision 변경/재전송 차단 PASS(exit 0). 32FC1 경로는 실제 로컬 TF broadcaster/listener/buffer로 TargetLocation 수신까지 PASS(exit 0)이며 좌표·지도·모델은 합성 fixture다. 기존 contracts/mapping/interfaces와 새 perception의 colcon 빌드 PASS. 로컬 미커밋 코드임을 기동 로그의 git SHA 뒤 +dirty로 표시한다. 실제 AMR/모델·실측 시간/공분산·관제 저장·자동 이동은 NOT_RUN.


## 실제 Git best.pt 추론 확인

[측정, 2026-10-12] 앞의 NOT_RUN 기록은 환경 설치 전 기록이다. 이제 작업 폴더의 `.venv`에 CPU torch 2.14.1+cpu, torchvision 0.29.1+cpu, Ultralytics 8.4.170, NumPy 1.26.4, OpenCV 4.10.0을 준비했다. 시스템 Python/기존 환경·로봇·GPU 드라이버 설정은 바꾸지 않았다. NVIDIA 드라이버를 사용할 수 없어 이번 시험은 CPU로 명시했다. NumPy 1.x 기반 ROS cv_bridge의 실제 변환도 PASS다.

`tools/amr_yolo_infer.py`는 Git manifest의 uploaded_model 경로·전체 SHA·class·imgsz로 실제 모델을 로드한다. 로컬 이미지만 입력받고 결과 이미지와 inference.json을 새 출력 폴더에 저장한다. 기존 결과 폴더는 덮어쓰지 않는다. CUDA를 명시했는데 가용하지 않으면 CPU로 바꾸지 않고 실패한다. Ultralytics 설정은 작업 폴더 `.artifacts` 아래로 둔다.

```bash
.venv/bin/python tools/amr_yolo_infer.py \
  --image <local_source_image> \
  --output-dir .artifacts/new-yolo-test --device cpu --confidence 0.25
```

Git의 `robot5-100epoch-capture.png`는 이미 박스가 그려진 기존 결과 이미지다. 이를 원본 그대로 실행 확인용으로 재입력했으며 새로운 카메라 영상·독립 test·정확도 측정으로 보고하지 않는다. 실제 모델 SHA는 최종 manifest와 일치했다. 이 입력에서 survivor 1개, confidence 0.9326171875가 반환됐다. 이 점수를 원본 결과의 0.9814와 성능 비교하지 않는다.

`tools/amr_camera_smoke.py --real-model --image <local_source_image>`는 가짜 모델 대신 실제 Git best.pt를 노드에 로드한다. 로컬 그림을 ROS CompressedImage로 발행해 실제 복호화·YOLO·annotated Image 발행/수신·원 헤더 보존을 확인한다. fake 모델 시험의 의도적 첫 추론 실패 검사는 fake 모드에만 적용한다. 실제 모델 모드는 소스 카메라·Depth를 대신하지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=176 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DISCOVERY_SERVER= \
  PYTHONPATH=src/rescue_contracts:src/rescue_mapping:src/rescue_perception:$PYTHONPATH \
  .venv/bin/python tools/amr_camera_smoke.py --real-model \
  --image docs/implementation/yolo26n-results/robot5-100epoch-capture.png
```

[측정] 실제 모델 파일 추론 PASS(exit 0), 실제 모델 compressed ROS 미리보기 PASS(exit 0, annotated 프레임 3개 수신, 원 헤더 보존), amr_target_node의 실제 best.pt 기동/정리 PASS(exit 0). 실제 모델로 live AMR RGB-D→TargetLocation을 발행한 시험은 NOT_RUN이다. TargetLocation 전체 체인의 이전 PASS는 합성 모델·깊이·지도 입력 시험이며 별도로 유지한다.

작업 산출물은 `.artifacts/bestpt-real-inference-20261012/`의 inference.json·annotated PNG이며 Git 추적 대상이 아니다. 실제 노드의 모델 경로는 `assets/models/best.pt`, 해시는 manifest 값, imgsz=640으로 그대로 연결돼 있다. 실제 센서 토픽·정렬·시간·mapping 메타데이터가 준비되면 같은 모델을 AMR 노드에 사용한다.


## Robot6 Depth 보정 연결 (2026-10-12)

[사용자 요구] 2026-10-12 원문: “126의 보정이 robot6에서의 depth 보정값이거든 이거와 연결해서 RBGD를 보정해서 코드를 수정해줘”. 적용 범위는 인식 노드의 Depth 거리·XYZ 계산이며 로봇 센서 설정은 변경하지 않는다.

[계산] [PR #126](https://github.com/jaebeom/ROKEY-7/pull/126)의 `0.82 / mean(0.854, 0.853, 0.857)`를 [robot6 보정 manifest](../../assets/manifest/robot6-depth-pr126.json)에 보존한다. 실제 계수·측정·승인 기록은 패키지 리소스 `rescue_perception/config/robot6-depth-pr126.json` 한 곳에 두고, manifest는 리소스와 해시를 참조한다. 노드는 `package:robot6-depth-pr126.json` 또는 manifest 경로로 읽을 수 있다. `physical_robot_id`가 manifest와 다르면 기동 실패하며 다른 로봇의 기본 보정은 scale=1, offset=0이다.

RGB는 그대로 두고 Depth를 mm→m로 변환한 뒤 원 0·음수·NaN·Inf를 제외한다. `corrected_m = raw_m * scale + offset_m`를 한 번 적용하고, 보정된 거리 범위·불연속 검사와 중앙값으로 optical XYZ를 계산한다. 16UC1·32FC1, raw·PNG compressedDepth 입력이 같은 계산을 사용한다. 원 토픽·픽셀·헤더는 재발행/수정하지 않는다. 관측 calibration_id와 기동 로그에 manifest SHA·scale·offset을 남긴다.

작업본 루트에서 기본/실제 센서 설정 뒤 [robot6 overlay](config/amr_target_node.robot6-depth.yaml)를 추가한다:

```bash
ros2 run rescue_perception amr_target_node --ros-args \
  --params-file src/rescue_perception/config/amr_target_node.example.yaml \
  --params-file src/rescue_perception/config/amr_target_node.robot6-depth.yaml
```

기본 토픽/frame/시간/정렬 값은 실제 장비 증거로 별도 설정해야 한다. overlay는 거리 보정만 선택하며 `locations_enabled:false` 등의 기존 관문을 해제하지 않는다. 보정된 Depth를 이미 발행하는 입력에는 이 overlay를 쓰지 않는다(이중 보정). RGB-Depth 공간 정렬·CameraInfo·시간·TF와 공분산 증거는 별도로 필요하다. covariance metadata는 보정 후 좌표의 분산을 설명해야 한다. 단일 82cm 보정을 전 거리 정확도나 자동 접근 검증으로 확대하지 않는다.

시험: `tests/test_perception_depth_calibration.py`의 E2 합성 시험으로 mm/m 입력의 보정 XYZ·TargetLocation 전달·원 무효값 보존·물리 로봇 불일치 거절·identity 기본값·노드 설정 연결을 확인한다. 실제 robot6 수정본 재실행·다거리·방향 시험은 NOT_RUN.

[측정, 2026-10-12] 보정 회귀시험을 구현 전에 별도 커밋했다. 수정 전 전체 859개 시험은 errors=20, exit 1(신규 8개 시험에서 보정 API/파일 부재), 수정 후 전체 859개는 skip 없이 OK, exit 0이다. 문서 검사 PASS, 원본 객체·줄 상한은 --sources 미제공으로 SKIP. 로컬 미커밋 AMR 기반을 별도 snapshot 커밋으로 보존한 작업 브랜치이며 원격 push/PR/머지는 하지 않았다.


## 기존 저장소 파일에 알고리즘 접목 (2026-10-12)

[사용자 요구] 원문: “내가 했던 알고리즘을 기존 main 인 Dev 폴더의 기존 파일에 접목시켜줘”. 적용 범위는 로컬 저장소의 인식 패키지·보정 manifest·관련 시험·도구다. 기존 `adapter.observe()`와 `PersonPipeline.process()`를 재사용하고, 기존 `core.locate_detection()`의 단위 변환 뒤에 robot6 Depth 보정을 넣는다. `setup.py`와 `package.xml`은 동일 패키지의 AMR 노드 실행·의존성을 등록한다. 별도 저장소나 패키지를 실행할 필요 없이 이 저장소 루트에서 빌드·설정·실행한다.

실행 순서는 `amr_target_node` → YOLO → 기존 PersonPipeline → 기존 adapter → 보정된 optical XYZ → 선택적 지도 변환 → TargetObservation/TargetLocation이다. 영상 미리보기는 별도 `amr_camera_node`로도 실행할 수 있다. 주행 명령·로봇 센서 설정은 바꾸지 않는다.

[측정] 현재 로컬 main 작업 파일에 접목 후 전체 단위시험 859개 OK(exit 0). 원격 main 동기화·commit·push·PR·머지는 이 요청의 수행 결과가 아니다. 최신 원격 웹캠 노드와 패키지 등록을 합치는 것은 향후 PR 전 별도 통합 작업이다. 실제 robot6 재실행·다거리 보정·센서 정렬·지도·관제 저장은 NOT_RUN.

[측정, 2026-10-12] 설치본 첫 검사에서 신규 보정 시험 2건이 소스 경로 의존으로 실패했다. 보정계수 원본을 패키지 JSON 리소스에 포함하고 manifest를 해시 참조로 연결했으며, 설정 시험은 설치된 share 설정을 읽도록 수정했다. 최종 소스 시험 859개는 skip 없이 OK(exit 0). 설치 wheel 8개 빌드·의존성·격리 import·설치본 시험은 OK(exit 0, 859개 중 기존 27개 SKIP/NOT_RUN). 신규 AMR·보정 시험은 설치본에서도 skip 없이 실행했다. 문서 검사·diff 공백 검사 PASS. 원본 객체·줄 상한은 --sources 미제공으로 SKIP.


## Draft PR의 후속 수정 순서 (2026-10-12)

[사용자 요구] “일단 올리고 단위 테스트 별 보정후 기존 코드 수정 들어갈 예정이라고 추가해줘”. 먼저 amr_camera·amr_target을 각각 단위별로 시험하고 입력·Depth·시간·좌표를 보정한 뒤, 그 결과에 따라 기존 코드의 후속 수정·통합을 진행할 예정이다. 이번 PR은 독립 노드 검증 기반이며 실제 robot6·조정기/DB·2 AMR 인수 완료를 뜻하지 않는다.

[사용자 요구] “당연히 지금 최신 git main에서 지우면서 올라가는 것이 없도록 확인해주고”. 최신 main 49f43ab의 정확한 tree를 출발점으로 삼고, 웹캠 실행 등록·의존성·패키지 설정과 bag 코드를 유지한다. 파일 삭제 없이 인식 레인 변경만 올린다.

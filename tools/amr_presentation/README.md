# Survivor 발표 화면 및 중심 ROI 비교

[사용자 요구, 2026-10-15] survivor 탐지와 카메라 optical XYZ를 발표용 이미지·영상으로 촬영하고,
bbox 중심 ROI 크기를 바꿔 비교 데이터를 수집한다. 실행·테스트·저장 위치를 이 README에 기록한다.

기존 ROS 노드와 별도로 실행하는 관제 PC용 도구다. 기존 `src/`, launch, 모델, 로봇 설정은 수정하지 않는다.
ROS 영상·CameraInfo를 구독하며 좌표 메시지나 주행 명령을 발행하지 않는다.
화면의 `LOCATED`는 카메라 기준 좌표 계산 성공이며 TF 변환·지도 좌표·임무 수신 성공을 뜻하지 않는다.

## 파일과 알고리즘 기준

| 파일 | 역할 |
| --- | --- |
| `yolo_depth_viewer.py` | RGB·YOLO·depth/ROI·카메라 XYZ 화면, 이미지·영상 저장 |
| `roi_compare_viewer.py` | 비교 모드 실행, `C` 키로 표본 수집 |
| `roi_comparison.py` | 같은 RGB-depth 쌍에서 ROI 크기별 계산, 비동기 CSV 저장 |
| `roi_report.py` | 수집 종료 후 통계와 PNG/PDF 그래프 생성 |
| `test_offline.py` | 로봇·YOLO 없이 합성 입력으로 회귀 검사 |
| `vendor/`, `source_manifest.json` | 기존 알고리즘의 고정 스냅샷과 원본 blob SHA |

[원본 코드] `vendor/`의 5개 모듈은 ROKEY-7 커밋
`5669356d5f8b2ff04e895a90d40bd8015f25beba`에서 가져온 파일이다.
원본 파일은 변경하지 않았고, 도구 안의 스냅샷이므로 이후 main 변경을 자동으로 따라가지 않는다.
`SelectiveDepth`와 `locate_center_depth`를 직접 호출한다.

[설정] survivor 탐지 점수 0.5 이상을 표시하고, 0.9 이상이며 이미지 경계에 닿지 않은 후보 중
최고 점수 bbox를 depth 요청에 사용한다. 기본 ROI는 bbox 중심 반경 5픽셀, 최대 11×11이다.
필요하면 CameraInfo로 bbox 중심점을 왜곡 보정한 뒤 depth 격자에서 검사한다.
거리 범위 0.2~5m, 유효 픽셀 10개 이상, 유효 비율 50% 이상, 깊이 최대-최소 0.5m 이하를 검사한다.
보정 계수는 기본 scale=1, offset=0이며 현장 보정이 검증된 경우에만 옵션으로 지정한다.

## 설치와 실행 위치

GUI가 있는 **관제 PC**에서 저장소 루트로 이동한 뒤 실행한다. 로봇 Pi의 SSH 셸에서 GUI를 실행하지 않는다.
아래 명령은 Ubuntu / ROS 2 Jazzy 환경의 예시다.

```bash
source /opt/ros/jazzy/setup.bash
python3 -c 'import rclpy, cv_bridge, cv2, numpy, torch, ultralytics; print("CUDA:", torch.cuda.is_available())'
```

같은 Python에서 위 모듈을 사용할 수 있으면 설치를 생략한다. 별도 환경이 필요하면:

```bash
python3 -m venv --system-site-packages /tmp/amr-presentation-venv
source /tmp/amr-presentation-venv/bin/activate
python -m pip install 'numpy<2' ultralytics matplotlib
```

ROS와 호환되는 시스템 Python을 사용한다. OpenCV는 GUI가 가능한 빌드여야 한다.
`best.pt`는 프로젝트의 survivor 단일 클래스 탐지 모델을 사용하며 자동 다운로드하지 않는다.
ROS_DOMAIN_ID, RMW_IMPLEMENTATION, ROS_DISCOVERY_SERVER 등은 실제 카메라 토픽을 받던 환경을 유지한다.
이 도구는 카메라·네트워크·로봇 설정을 변경하지 않는다.

## 토픽 확인

먼저 관제 PC에서 확인한다.

```bash
ros2 topic list -t
ros2 topic info -v /robot6/oakd/rgb/preview/image_raw
ros2 topic info -v /robot6/oakd/stereo/image_raw
```

토픽명은 예시다. RGB는 `Image`, 또는 `--compressed`를 지정한 경우 `CompressedImage`다.
Depth는 **raw `Image`의 16UC1(mm) 또는 32FC1(m)**만 받는다. `compressedDepth`는 이 도구의 입력이 아니다.
CameraInfo는 사용 중인 RGB와 정렬된 depth 격자에 대응해야 한다.
해상도·화각이 다르면 단순 resize로 정렬이 확인된 것으로 처리하지 않는다.

## 실행

예시 토픽을 실제 값으로 바꾼다. GPU가 확인된 환경에서는 `--device cpu`를 `--device 0`으로 바꿀 수 있다.

```bash
python tools/amr_presentation/yolo_depth_viewer.py \
  --rgb-topic /robot6/oakd/rgb/preview/image_raw \
  --depth-topic /robot6/oakd/stereo/image_raw \
  --camera-info-topic /robot6/oakd/rgb/preview/camera_info \
  --model assets/models/best.pt --device cpu \
  --output tools/amr_presentation/captures
```

RGB 압축 토픽이면 실제 토픽 전체 이름과 `--compressed`를 함께 지정한다.
geometry를 생략하면 시간 의미·정렬은 미확인 상태이며 좌표 계산을 보류한다.

### 실제 geometry 설정

현장에서 확인한 값만 JSON에 기록하고 `--geometry /실제/geometry.json`으로 전달한다.
아래 템플릿은 의도적으로 미확인 상태다. 좌표를 표시하기 위해 임의로 true나 CAPTURE로 바꾸지 않는다.

```json
{
  "rgb_stamp_semantics": "UNKNOWN",
  "depth_stamp_semantics": "UNKNOWN",
  "clock_domain_id": "UNCONFIGURED",
  "calibration_id": "UNCONFIGURED",
  "alignment_evidence_id": "UNCONFIGURED",
  "rgb_optical_frame": "UNCONFIGURED",
  "depth_optical_frame": "UNCONFIGURED",
  "rgb_depth_alignment_verified": false,
  "rgb_rectified": false,
  "camera_info_max_age_ns": 1000000000
}
```

`rgb_rectified=false`여도 지원되는 CameraInfo K/D/P가 있으면 원본 함수가 점을 왜곡 보정한다.
공간 정렬과 시간 동기화는 별개다. 보정값이 확인됐다면 `--depth-scale 값 --depth-offset 값`을 지정한다.
기본 stamp 허용 차이 20ms와 늦은 depth 대기 시간 1.5초는 서로 다른 조건이다.
단순히 가장 최근 도착한 RGB와 depth를 묶지 않는다.

## 화면과 키

왼쪽은 최신 RGB, 가운데는 실제 분석에 사용한 RGB, 오른쪽은 대응 depth와 검사 ROI다.
분석 결과는 늦게 도착한 depth의 RGB일 수 있어 왼쪽과 시점이 다를 수 있다.
매칭 결과가 없으면 오른쪽에 `LIVE DEPTH / not a selected pair`가 표시된다.
오래된 결과는 `STALE`로 표시한다.

| 키 | 동작 |
| --- | --- |
| `S` | 현재 합성 화면과 분석 RGB·YOLO·depth·메타데이터 저장 |
| `R` | 영상 녹화 시작/정지 |
| `[` / `]` | 표시할 중심 ROI 크기 변경 |
| `C` | 비교 모드에서 CSV 수집 시작/정지 |
| `Q` / Esc | 종료, 저장 작업 및 녹화 마무리 |
| depth 클릭 | 원시 optical Z 확인. 보정 XYZ나 유클리드 직선거리가 아님 |

카메라 optical 좌표축은 오른쪽 +X, 아래쪽 +Y, 앞쪽 +Z다.
이 도구는 base_link TF, map 좌표, QR 출구, 대상 재확인, SENT 상태를 실행하지 않는다.

## ROI 비교와 데이터 수집

기본 5×5·11×11·21×21을 **같은 RGB-depth 쌍과 같은 bbox**에 적용한다.
`[` / `]`는 화면 강조만 바꾸며 비교 모드에서는 지정한 모든 크기를 계속 기록한다.
YOLO를 크기마다 다시 실행하지 않는다.
서로 다른 홀수 5~101 중 최대 5개를 지정할 수 있다.
최소 유효 픽셀 10개 정책을 유지하기 위해 1×1/3×3은 비교 크기로 허용하지 않는다.

```bash
python tools/amr_presentation/roi_compare_viewer.py \
  --rgb-topic /robot6/oakd/rgb/preview/image_raw \
  --depth-topic /robot6/oakd/stereo/image_raw \
  --camera-info-topic /robot6/oakd/rgb/preview/camera_info \
  --model assets/models/best.pt --device cpu \
  --geometry /실제/geometry.json \
  --roi-sizes 5 11 21 --roi-size 11 \
  --trial-label front_static --compare-seconds 30 \
  --output tools/amr_presentation/captures
```

기준 거리 오차를 계산하려면 **실측한 대상 표면의 optical Z**를 `--reference-z 값`으로 추가한다.
기준값이 없으면 생략한다. 이때 오차는 N/A다. 비스듬히 잰 직선거리와 optical Z는 다르다.
거리·조명·가림 조건이 바뀌면 새 label/reference로 별도 실행한다.

### 테스트 순서와 중단 기준

1. 로봇·카메라·대상을 고정하고 한 대상만 놓는다. 초기 추론 준비가 끝날 때까지 기다린다.
2. 실제 영상과 frame_id, CameraInfo, geometry를 확인한다. 미확인 기하나 잘못된 토픽이면 수집을 중단한다.
3. `LOCATED`와 XYZ를 확인한다. 지속적인 STALE, RGB/depth 수신 중단, 저장 실패 경고가 있으면 원인을 먼저 확인한다.
4. `C`로 수집한다. 기본 30초 후 자동 종료하며 다시 `C`를 누르면 조기 종료한다.
5. 이미지/영상이 필요하면 `S`/`R`을 사용한다. 녹화 부하를 비교하려면 녹화 ON/OFF 회차를 구분한다.
6. `Q`로 정상 종료한 뒤 보고서를 만든다. 동일 조건을 여러 회차 반복하고 실패 표본도 유지한다.

[미확인] 이 절차는 현장 시험 방법이며 현장 합격을 의미하지 않는다.
정확도·지연의 최종 허용치는 이 도구에서 임의로 정하지 않는다.

## 저장 위치와 파일 의미

상대 `--output`은 **실행한 터미널의 현재 디렉터리 기준**이다. 시작 시 해석된 절대 출력 경로를 출력한다.
생략하면 현재 디렉터리 아래 `captures/`다. 위 예시는 저장소의 `tools/amr_presentation/captures/`에 저장한다.
현재 PC에 저장되며 로봇이나 클라우드로 자동 전송하지 않는다.

```text
captures/
  session_날짜_시각.json            모델 SHA·원본 SHA·실행 옵션·geometry
  image_날짜_시각/
    presentation.png              합성 발표 화면
    rgb.png                       분석에 사용한 RGB
    yolo.png                      분석 RGB의 표시 영상
    depth_m.npy                   매칭된 원시 depth를 m로 변환, 무효값 NaN
    metadata.json                 원본 stamp·ROI·좌표·표시 age·상태
  video_날짜_시각/
    presentation.avi              MJPG 영상
    frames.jsonl                  녹화 프레임별 시간·원본 stamp·상태
  roi_날짜_시각/
    trial.json                    회차 조건·모델·geometry·기준 Z
    samples.csv                   pair_id별 각 ROI의 성공/거절 결과
    collection_end.json           기록 표본 수·큐 누락 수·시작/종료
    summary.csv / summary.json    보고서 명령 실행 후 생성
    roi_comparison.png / .pdf     보고서 명령 실행 후 생성
```

분석 결과나 매칭 depth가 없는 시점에는 관련 이미지/NPY 파일을 생성하지 않는다.
`depth_m.npy`는 미보정 값이며 XYZ는 설정한 scale/offset을 적용한 계산 결과다.
CSV는 전체 RGB-D 영상을 저장하지 않는다. 전체 표본의 영상 재처리가 필요하면 별도 rosbag을 기록한다.

최신 저장 회차 찾기:

```bash
ls -td tools/amr_presentation/captures/image_* \
       tools/amr_presentation/captures/video_* \
       tools/amr_presentation/captures/roi_*
```

촬영 후 MP4가 필요할 때만 변환한다.

```bash
ffmpeg -i /실제/video_회차/presentation.avi -c:v libx264 -pix_fmt yuv420p /실제/presentation.mp4
```

## 보고서 생성과 지표 해석

```bash
python tools/amr_presentation/roi_report.py tools/amr_presentation/captures/roi_실제회차
# matplotlib 없이 통계 파일만 생성:
python tools/amr_presentation/roi_report.py tools/amr_presentation/captures/roi_실제회차 --no-plots
```

- 통과율 분모는 **매칭된 eligible 쌍**이다. 전체 카메라 프레임 대비 탐지 성공률이 아니다.
- 각 크기의 MAE/RMSE와 표준편차는 통과 표본만 사용한다. 실패를 0 오차로 넣지 않는다.
- `common_mae_m`는 모든 크기가 함께 통과한 동일 쌍의 오차다. 통과율과 함께 해석한다.
- Z 표준편차는 카메라와 대상이 정지했을 때만 흔들림 지표로 해석한다.
- 작은 bbox에서는 ROI가 잘린다. `roi_pixels`와 `clipped`도 확인한다.
- 기준값 없음/통과 0개는 오차 N/A, 통과 1개는 표준편차 N/A다.
- `collection_end.json`이 없거나 `writer_count_matches=false`면 중단·저장 누락을 먼저 확인한다.
- CSV 큐가 포화되면 모든 크기의 쌍을 통째로 버리고 `dropped_pairs`를 기록한다.

## 처리 부하와 한계

ROS 수신·영상 변환·YOLO 추론·GUI·저장을 분리한다. RGB는 최신 슬롯을 처리하며 오래된 프레임을 무한히 쌓지 않는다.
창을 캡처해 다시 띄우지 않고 메모리의 합성 이미지를 화면·녹화로 전달한다.
모든 스레드는 CPU/GPU/메모리 대역폭을 공유하므로 실제 성능 개선은 현장 측정이 필요하다.

화면의 RX는 수신 빈도, INFER는 추론 완료 빈도, UI는 화면 갱신 빈도다.
receive age는 PC 수신 이후 표시까지의 경과 시간이며 센서부터의 전체 지연이 아니다.
녹화는 고정 시간축에 최신 화면을 반복할 수 있다. 녹화 프레임 수가 추론 횟수를 의미하지 않는다.
CSV는 매칭된 표본만 기록하므로 전체 구간의 성능 계측 자료로 간주하지 않는다.

## 로봇 없는 회귀 검사

NumPy와 OpenCV가 있는 환경에서 저장소 루트에서 실행한다. ROS·torch·모델은 필요 없다.

```bash
python tools/amr_presentation/test_offline.py
```

합성 depth의 단위, ROI 크기·배경 혼입·무효값, 정렬 거절, 지연 매칭,
CSV 중복 방지, 알려진 오차 통계, 빈 회차를 확인한다. 임시 출력은 시스템 임시 폴더에 생성 후 삭제한다.

[검증 범위] 로봇 없는 합성 입력은 E2다. 실제 ROS 카메라, YOLO 실행, GUI 키·마우스,
실제 좌표 정확도 및 촬영 부하는 E3/E4 **NOT_RUN**이다. 모델 파일은 이 도구에 포함하지 않는다.

---
title: Depth 클릭 확인 도구와 82cm 보정 기록
status: partial-measurement
updated: 2026-10-13
---

# Depth 클릭 확인 도구와 82cm 보정 기록

> **기록 (2026-10-12 ~ 2026-10-13).** 깊이 보정 측정 기록이다. 지금 쓰는 깊이 보정은 TB-5·TB-6 공통 파일 [robot_depth.json](../../src/rescue_perception/rescue_perception/config/robot_depth.json)이다. 쓰는 법은 [인식 README](../../src/rescue_perception/README.md)를 본다.

[사용자 요구, 2026-10-12] “오케이 그러면 그렇게 해서 PR 올려줘.”에 따라 사용자가 실행한 depth 확인 코드와 보정 계산·측정 로그를 별도 PR에 보존한다. [도구](../../tools/depth_click_check.py)는 원본 ROS depth 토픽을 읽고 화면 클릭 지점의 Raw와 보정 거리를 출력한다. 주행 명령, 센서 설정 변경, 보정 depth 재발행은 하지 않는다.

## 보정 근거

[사용자 측정 보고, 2026-10-12] 실제 거리는 **0.82m**다. 계수를 구하는 데 사용한 세 Raw 값은 **0.854, 0.853, 0.857m**다. 이 보고값으로 계산한 단일 거리 비례 계수는 다음과 같다.

```text
Raw 평균 = (0.854 + 0.853 + 0.857) / 3
          = 0.8546666667 m
DEPTH_SCALE = 실제 거리 / Raw 평균
            = 0.82 / 0.8546666667
            = 0.9594383775...
표시 보정값 = Raw × DEPTH_SCALE + DEPTH_OFFSET_M
DEPTH_OFFSET_M = 0.0
```

이 값은 perception 코어가 읽는 [`robot_depth.json`](../../src/rescue_perception/rescue_perception/config/robot_depth.json)의 `scale`(0.9594383775351013, `offset_m` 0.0)과 같다. 도구는 계수를 상수로 따로 두지 않고 이 파일을 코어와 같은 함수(`target_utils.load_depth_calibration`)로 읽는다. 시작 로그의 소수점 9자리 표시는 `0.959438378`이다. 이전 계수는 `1.38 / 1.47 ≈ 0.938775510`이었다. 이번 세 값을 기존 Corrected 값으로 다시 나누지 않고, **원본 Raw 평균**을 사용했다.

## 사용자가 제공한 원문 결과

보정 전, 실제 거리 82cm에서 전달받은 로그:

```text
[INFO] [1791786845.874172018] [depth_checker]: Clicked at (u=354, v=353) → Raw = 0.854 m, Corrected = 0.802 m (valid 25/25 px)
[INFO] [1791786849.091503528] [depth_checker]: Clicked at (u=351, v=358) → Raw = 0.853 m, Corrected = 0.801 m (valid 25/25 px)
[INFO] [1791786852.992141342] [depth_checker]: Clicked at (u=351, v=358) → Raw = 0.857 m, Corrected = 0.805 m (valid 25/25 px)
```

계수 변경 및 프로그램 재실행 후 전달받은 로그:

```text
[INFO] [1791787202.717752566] [depth_checker]: Clicked at (u=334, v=348) → Raw = 0.857 m, Corrected = 0.822 m (valid 25/25 px)
[INFO] [1791787203.835980734] [depth_checker]: Clicked at (u=522, v=437) → Raw = 0.862 m, Corrected = 0.827 m (valid 25/25 px)
[INFO] [1791787205.099016616] [depth_checker]: Clicked at (u=506, v=308) → Raw = 0.863 m, Corrected = 0.828 m (valid 25/25 px)
```

| 측정 | 변경 후 Raw | 로그의 Corrected | 실제 82cm 대비 오차 |
| --- | --- | --- | --- |
| 1 | 85.7cm | 82.2cm | +0.2cm |
| 2 | 86.2cm | 82.7cm | +0.7cm |
| 3 | 86.3cm | 82.8cm | +0.8cm |
| 평균 | 86.07cm | 82.57cm | +0.57cm |

[계산, 2026-10-12] 표는 소수점 3자리 m로 출력된 로그를 기준으로 계산했다. 표의 평균은 반올림된 표시값이며 원 센서의 절대 정밀도를 주장하지 않는다. 클릭 좌표가 달라 각 지점이 동일한 82cm 평면인지 확인이 필요하다. 25/25 유효 픽셀은 패치 안 양의 유효값 수이지 정확도 보증이 아니다.

이는 사용자가 제공한 단독 실기 관찰이며 작성자가 독립적으로 재측정한 결과가 아니다. 실제 거리의 측정 기준점·방법, 목표 평면, 당시 센서 설정·설치본 SHA는 미확인이다. 다거리/다각도 정확도, 보정 모델의 일반화 및 전체 프로젝트 E4 인수는 `NOT_RUN`이다.

## 코드와 실행 방법

사용자가 실행한 standalone 코드를 저장소 도구로 옮겼다. 클릭 주변 최대 **5×5** 패치의 양의 유효값 중앙값을 사용하고 `16UC1` mm 입력을 m로 변환한다. 마지막 수신 이후 **2초** 넘은 프레임은 클릭 측정을 거절한다. 이는 로컬 수신 단절 검사이며 원 header의 캡처 나이나 시계 동기화 판정이 아니다. 화면 색상 범위 3m는 시각화 설정이다.

실제 로봇 토픽은 필수 CLI 인자로 받는다. 보정은 기본으로 `--calibration package:robot_depth.json`, `--physical-robot-id robot6` 이고, 다른 보정 파일은 이 두 인자로 고른다(파일의 `physical_robot_ids`에 실행 로봇이 없으면 시작하지 않는다). `--depth-scale`·`--depth-offset-m` 은 실험용 덮어쓰기이며 시작 로그에 `override` 와 파일 값이 같이 남는다. 시작 로그에는 도구 소스 SHA256, 입력 토픽, 보정 파일 ID·SHA256, 실제 적용 계수와 offset을 남긴다. 클릭 계산(중앙값·거절·계수 적용)과 기본 보정 파일은 `tests/test_depth_click_check.py` 가 ROS 없이 시험한다.

ROS 2 Jazzy, rclpy, sensor_msgs, cv_bridge, NumPy, OpenCV, GUI 화면과 빌드한 `rescue_perception`(보정 파일)이 필요하다. 워크스페이스를 source 한 뒤 저장소 루트 기준으로 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=6
python3 tools/depth_click_check.py \
  --depth-topic /robot6/oakd/stereo/image_raw \
  --camera-info-topic /robot6/oakd/stereo/camera_info
```

창에서 대상 표면을 클릭하면 Raw/Corrected와 유효 픽셀 수를 출력한다. `q`, 창 닫기 또는 Ctrl+C로 종료한다. 기본 계수는 위 82cm 실측 보고에서 계산한 값(보정 파일)이다. 원본 값만 비교하려면 다음처럼 덮어쓴다.

```bash
python3 tools/depth_click_check.py \
  --depth-topic /robot6/oakd/stereo/image_raw \
  --camera-info-topic /robot6/oakd/stereo/camera_info \
  --depth-scale 1.0 --depth-offset-m 0.0
```

ROS 센서 원본 토픽은 바꾸지 않는다. perception 코어는 같은 보정 파일을 `depth_calibration_path`(기본 `package:robot_depth.json`)로 읽어 자기 좌표 계산에 쓴다. 이 도구가 코어에 값을 넘기지는 않는다. CLI와 시작 기록을 추가한 저장소 버전의 실제 센서/GUI 재실행은 `NOT_RUN`이다. 로컬에서 시험한 이전 standalone 코드의 관찰을 이 변경본의 실행 통과로 바꾸지 않는다.

## 후속 검증

같은 표면·클릭 위치에서 반복하고 다른 거리·방향에서도 실제 거리와 Raw/Corrected를 비교한다. 계수를 바꾸면 입력값·측정 기준·새 계수·실행 코드 SHA·파라미터·원문 로그를 같은 PR에 기록한다. 한 거리의 평균이 맞았다는 이유만으로 전체 거리 범위에 대한 보정 완료나 자동 접근용 정확도를 선언하지 않는다.

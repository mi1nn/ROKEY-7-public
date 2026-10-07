---
title: 기록 입력 기반 인식 코어 구현
status: draft-implementation-e2
updated: 2026-10-08
---

# 기록 입력 기반 인식 코어

계획 PR 7은 [C-02·03·05](contracts.md), [D-06](algorithm-decisions.md), [CT-01·02·03·10](acceptance-plan.md)의 일부를 실행 가능한 순수 Python 로직으로 구현한다. RGB 픽셀에서 사람을 찾는 모델은 아직 없다. 입력은 이미 얻은 탐지 bbox/class와 RGB/Depth 메타데이터, 복호화된 깊이 배열이다. 이 PR에서 실제 YOLO, 가중치, ROS, CameraInfo 구독, TF, 카메라, 두 로봇을 실행하지 않았다.

## 입력과 광학 위치

`FrameMeta`는 frame·원 observed_at·clock domain·stamp 의미·해상도·보정 ID다. `DepthFrame`은 `16UC1`/mm 또는 `32FC1`/m와 직사각형 Python sequence를 받는다. 16비트 자료는 범위 안 정수만 허용한다. 32비트 깊이의 NaN/Inf/0/범위 밖 값은 센서 결측으로 제외하며, scalar 설정/시간·bbox의 NaN/Inf/bool은 오류다. 압축 이미지나 bytes를 디코딩하는 adapter는 후속이다.

`AlignmentEvidence.verified`는 설치 adapter가 **Depth가 RGB rectified pixel grid에 등록됐고 값이 해당 optical z**라는 실험 근거를 제공할 때만 true로 넣는 계약이다. 이 코어가 물리 정렬을 인증하는 필드가 아니다. 근거 ID·calibration ID·양쪽 frame·해상도·intrinsics·rectified 상태를 검사하며 해상도 비율만으로 정합시키지 않는다. 서로 다른 stamp 의미나 clock domain, `UNKNOWN` 시각 의미를 가진 pair로 위치를 만들지 않는다. pair의 최대 stamp 차이는 `DepthPolicy.max_stamp_skew_s`다. 한쌍의 기록을 검증하며 실시간 동기화 queue는 제공하지 않는다.

bbox는 `[x0,y0,x1,y1)`의 영상 경계 좌표다. 정수로 감싼 bbox 전체가 ROI이며, bbox 바깥/빈 bbox는 조용히 잘라 쓰지 않고 거절한다. 유효 깊이 수·비율·최대−최소 spread를 명시한 정책과 비교한다. 통과하면 중앙값 z와 bbox 중심의 pixel centre로 `(x right, y down, z forward)` m 좌표를 계산한다. 배경 혼입 때문에 spread가 크면 `DEPTH_DISCONTINUITY`, 값이 부족하면 `DEPTH_INSUFFICIENT`다. 중앙값과 이 품질 관문은 표적 표면을 올바르게 선택했다는 증거가 아니며 segmentation/ROI 개선은 데이터로 평가해야 한다.

`LocalizationResult`는 위치 또는 실패 이유·유효 표본 수·정렬 근거를 갖는다. sensor covariance를 추정하거나 0으로 꾸미지 않는다. 공분산, 외부 파라미터, 관측 시각 TF, 공통 frame/session/revision은 mapping adapter와 추가 검증의 책임이다. `LOCATED`는 optical point 계산에 성공했다는 뜻이며 접근 허가는 아니다.

## 불변 관측과 최신성

`ModelManifest`는 모델 ID·SHA-256 형식·중복 없는 class 순서를 검증한다. Detection의 class ID와 label이 manifest에 동시에 맞아야 한다. 가중치를 로드/검사하지 않으므로 제공된 SHA가 실제 파일과 일치하는지는 adapter의 책임이다. track ID는 `None`이어도 관측을 보존하고, 숫자 문자열 `"0"`도 유효하다. track 없이 같은 class만으로 동일 인물이라고 확정하지 않는다.

`Observation`은 source robot/camera/boot, 원 `FrameMeta`, 모델·탐지·위치, 최초 `valid_until`, clock 상태/오차, 증거 ID/SHA를 보존한다. 문자열 UUID는 소문자 표준 형식의 non-nil ID를 요구한다. 시각 0·불리언·무한대, 만료≤관측, 잘못된 SHA·class·bbox를 거절한다. 중첩된 배열은 tuple로 저장한다. observed_at은 frame에서 읽으며 수신/발행 시각으로 바꾸는 setter가 없다.

`Publication`은 publisher/boot/uint64 sequence/published_at/schema_version을 관측 바깥에 둔다. 이 전송 envelope를 바꿔도 관측의 의미 payload hash는 변하지 않는다. 현재 schema는 `1.0`만 지원한다. 발행 sequence의 실제 증가·수신 queue/QoS를 담당하는 transport adapter는 아직 없다.

`validate_freshness`는 caller가 준 현재 wall time·receiver clock domain/오차와 `TimePolicy`를 검사한다. UNKNOWN stamp/clock·미확인 오차·다른 domain·과도한 clock 오차·미래 stamp·나이 초과·최초 유효기간 만료는 유효 임무 입력으로 거절한다. 보수적 나이는 `now−observed_at+송수신 clock 오차+센서 stamp 지연 상한`이다. 원 유효기간도 오차/지연을 더한 현재 시각으로 검사한다. CAPTURE/DRIVER_RECEIVE 각각의 실제 의미·지연 상한은 설치 adapter에서 측정해야 한다.

`ObservationRegistry.ingest`는 ID 충돌을 먼저 검사하고 현재 최신성을 평가한다. 같은 ID·같은 payload는 DUPLICATE이며 기존 객체를 반환한다. 새 ID이지만 원 stamp가 역순이면 기록은 보존하고 REORDERED를 반환한다. 같은 frame stamp의 서로 다른 사람은 각각 ACCEPTED하되 공백 타이머는 갱신하지 않는다. source robot/camera/boot별로 **원 stamp가 전진하는 새 유효 관측만** last_new_receive를 갱신한다. 만료된 재발행은 거절되고, 이전 boot의 수신 기록은 새 boot로 이어지지 않는다. 수신 monotonic 역행·wall time 역행을 거절한다.

이 registry는 단일 프로세스의 E2 기록이다. 영속 dedup, 재시작 복구, 시계의 순방향 급변 감지, 소스 재등록/활성 boot fencing, 수신 메모리 상한·감사 저장·transport 재연결은 미구현이다. 소비자는 admission 당시의 유효성을 영구 허가로 쓰지 않고 실제 사용 시점에 freshness·좌표·권한을 다시 검사해야 한다. 인터페이스 dataclass는 신뢰 경계나 서명이 아니며 외부 입력 adapter의 검증을 대체하지 않는다.

## 보수적 사건 후보

`ConservativeEventRegistry.register(admission, event_id=..., candidate_event_ids=...)`는 수신한 관측별로 별도 CANDIDATE를 만든다. 같은 관측을 재등록하면 같은 사건을 반환하며 event ID/연결 목록 또는 원 payload가 달라지면 충돌이다. REORDERED 관측은 새 사건 입력으로 수락하지 않는다. 같은 frame의 다른 사람, 다른 로봇의 같은 track ID, track 없는 관측은 각각 별도 사건이다.

`candidate_event_ids`는 이미 존재하는 사건과의 **미확인 연결 후보**다. 사건을 합치거나 CONFIRMED로 올리지 않는다. 공통 좌표·시간·불확실성·다중 관측/재확인 근거로 동일성을 판정하는 후속 coordination 정책이 필요하다. 현재는 같은 실제 사람의 연속 프레임도 별도 후보가 될 수 있으며, 이 보수적 과분리를 사람 수/구조 성공 통계로 집계하면 안 된다.

## 공개 API와 E2 실행

공개 항목은 `rescue_perception.__init__`에서 가져올 수 있다. 호출 예시는 `tests/test_perception.py`의 setUp/observation/ingest helper에 있다. 모든 정책 수치는 그 시험의 합성 입력 전용이다.

```python
from rescue_perception import ObservationRegistry, ConservativeEventRegistry

registry = ObservationRegistry()
admission = registry.ingest(
    observation,
    now=wall_time,
    received_steady=local_monotonic,
    receiver_clock_domain=clock_domain,
    receiver_clock_error_bound=clock_error,
    policy=time_policy,
)
event = ConservativeEventRegistry().register(admission, event_id=new_event_uuid)
# observation.localization.point_optical -> mapping adapter; Nav2 goal이 아님.
```

저장소 루트에서 `PYTHONPATH=src/rescue_perception python3 -m unittest discover -s tests -p test_perception.py -v`로 실행한다. 21개 unittest는 등록 깊이의 수학·단위, 잘못된 정합/해상도/시각, 깊이 결측·불연속, 모델 class, UUID·NaN/Inf/bool, 관측 불변·중복/충돌/역순/만료, 복수 대상과 로봇별 사건 분리를 다룬다. 이는 CT의 E2 일부이며 ROS/센서/모델 성능·실제 표적 위치/접근 검증은 NOT_RUN이다. TF 누락/SLAM revision은 이 코어의 PASS로 대신하지 않는다.

## 원리 참고와 남는 관문

새 구현은 원본 함수를 복사하지 않았다. 다음 고정 원본을 읽고 실패 조건과 필요한 수학 원리를 새 데이터 계약으로 작성했다.

- [P4 depth 디코딩·유효 중앙값·역투영](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/rokey_vision/rokey_vision/common.py#L172-L251): encoding/단위와 유효 표본, pinhole 원리를 참고. 원본 numpy/OpenCV·반경/거리 기본값·압축 decoder는 이식하지 않음.
- [P4 RGB/Depth/탐지 시간 동기](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/rokey_vision/rokey_vision/depth_distance.py#L113-L132): 헤더 pair의 정합 필요를 참고. ROS synchronizer 구현과 설정을 복사하지 않음.
- [P4 관측 시각 TF](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/target_locator.py#L74-L107): 원 stamp 보존을 참고. 최신 TF fallback이나 지도 변환을 이 PR에서 구현하지 않음.
- [AMR hold 재발행](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/webcam_detector.py#L109-L159): 과거 좌표와 새 발행 stamp를 혼동하는 경로를 방지하는 시험 근거. 최고 confidence 단일 대상/고정 평면 보정값을 채택하지 않음.

프로젝트 라이선스·maintainer의 기존 미결정 표시는 유지한다. 이 PR은 원본 소스·가중치·데이터·카메라 보정 파일을 복제하지 않는다. 새 대상의 독립 평가 데이터, 실제 모델/가중치 검증, CameraInfo/rectification/Depth 정렬·시각 의미, covariance, 실시간 adapter와 E3/E4의 2 AMR·배포 시험은 후속 관문이다.

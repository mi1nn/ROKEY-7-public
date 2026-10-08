---
title: 기록 입력 기반 인식 코어 구현
status: draft-implementation-e2
updated: 2026-10-08
---

# 기록 입력 기반 인식 코어

계획 PR 7은 [C-02·03·05](contracts.md), [D-06](algorithm-decisions.md), [CT-01·02·03·10](acceptance-plan.md)의 일부를 실행 가능한 순수 Python 로직으로 구현한다. RGB 픽셀에서 사람을 찾는 모델은 아직 없다. 입력은 이미 얻은 탐지 bbox/class와 RGB/Depth 메타데이터, 복호화된 깊이 배열이다. 이 PR에서 실제 YOLO, 가중치, ROS, CameraInfo 구독, TF, 카메라, 두 로봇을 실행하지 않았다.

## 입력과 광학 위치

`FrameMeta`는 frame·원 `observed_at_ns`·clock domain·stamp 의미·해상도·보정 ID다. `DepthFrame`은 `16UC1`/mm 또는 `32FC1`/m와 직사각형 Python sequence를 받는다. 16비트 자료는 범위 안 정수만 허용한다. 32비트 깊이의 NaN/Inf/0/범위 밖 값은 센서 결측으로 제외하며, scalar 설정/시간·bbox의 NaN/Inf/bool은 오류다. 압축 이미지나 bytes를 디코딩하는 adapter는 후속이다.

`AlignmentEvidence.verified`는 설치 adapter가 **Depth가 RGB rectified pixel grid에 등록됐고 값이 해당 optical z**라는 실험 근거를 제공할 때만 true로 넣는 계약이다. 이 코어가 물리 정렬을 인증하는 필드가 아니다. 근거 ID·calibration ID·양쪽 frame·해상도·intrinsics·rectified 상태를 검사하며 해상도 비율만으로 정합시키지 않는다. 서로 다른 stamp 의미나 clock domain, `UNKNOWN` 시각 의미를 가진 pair로 위치를 만들지 않는다. pair의 최대 stamp 차이는 정수 ns의 `DepthPolicy.max_stamp_skew_ns`다. 한쌍의 기록을 검증하며 실시간 동기화 queue는 제공하지 않는다.

bbox는 `[x0,y0,x1,y1)`의 영상 경계 좌표다. 정수로 감싼 bbox 전체가 ROI이며, bbox 바깥/빈 bbox는 조용히 잘라 쓰지 않고 거절한다. 유효 깊이 수·비율·최대−최소 spread를 명시한 정책과 비교한다. 통과하면 중앙값 z와 bbox 중심의 pixel centre로 `(x right, y down, z forward)` m 좌표를 계산한다. 배경 혼입 때문에 spread가 크면 `DEPTH_DISCONTINUITY`, 값이 부족하면 `DEPTH_INSUFFICIENT`다. 중앙값과 이 품질 관문은 표적 표면을 올바르게 선택했다는 증거가 아니며 segmentation/ROI 개선은 데이터로 평가해야 한다.

**E2 성능 경계:** `DepthFrame.__post_init__`는 전체 영상의 각 픽셀을 Python으로 검증하고 복사한다. ROI도 Python list를 만든다. 1280×720이면 전체 검사만 프레임당 921,600개 표본이며 실시간 성능을 주장하지 않는다. 후속 실시간 어댑터는 bounded ROI crop 또는 검증된 array 입력을 제공해야 한다. 이때 encoding/단위·shape·NaN/범위 처리·immutable 입력 소유권을 보존하고, crop 원점에서 원 영상 pixel centre와 intrinsics로 돌아가는 대응을 명시한다. 이식 전후의 위치/거절 결과가 같고, 두 영상 동시 처리에서 end-to-end 지연·누락·메모리가 사전 합의한 예산 안임을 측정한 뒤 승급한다. 이 PR에 numpy/ROI 어댑터나 실측 예산을 추가한 것은 아니다.

`LocalizationResult`는 위치 또는 실패 이유·유효 표본 수·정렬 근거를 갖는다. sensor covariance를 추정하거나 0으로 꾸미지 않는다. 공분산, 외부 파라미터, 관측 시각 TF, 공통 frame/session/revision은 mapping adapter와 추가 검증의 책임이다. `LOCATED`는 optical point 계산에 성공했다는 뜻이며 접근 허가는 아니다.

## 불변 관측과 최신성

`ModelManifest`는 모델 ID·SHA-256 형식·중복 없는 class 순서를 검증한다. Detection의 class ID와 label이 manifest에 동시에 맞아야 한다. 가중치를 로드/검사하지 않으므로 제공된 SHA가 실제 파일과 일치하는지는 adapter의 책임이다. `Detection.tracking_mode`는 기본값 없이 필수이며 `DETECTION_ONLY`(처음부터 탐지만), `TRACKING`(유효 track ID가 있는 추적), `TRACKING_FALLBACK`(추적 실패 뒤 탐지로 저하)을 구분한다. TRACKING에는 비어 있지 않은 track_id가 필수이고 숫자 문자열 `"0"`도 유효하다. 나머지 두 모드의 track_id는 None이어야 한다. 모드도 원 관측의 의미 payload hash에 포함되므로 같은 관측 ID에서 상태를 조용히 바꿀 수 없다. track 없이 같은 class만으로 동일 인물이라고 확정하지 않는다.

`Observation`은 source robot/camera/boot, 원 `FrameMeta`, 모델·탐지·위치, 최초 `valid_until_ns`, clock 상태/`clock_error_bound_ns`, 증거 ID/SHA를 보존한다. 로봇 ID는 `rescue_contracts.robot_id`의 배포 프로파일 형식에 맞아야 한다. 문자열 UUID는 소문자 표준 형식의 non-nil ID를 요구한다. 시각 0·float·bool·int64 범위 초과, 만료≤관측, 잘못된 SHA·class·bbox를 거절한다. 중첩된 배열은 tuple로 저장한다. `observed_at_ns`는 frame에서 읽으며 수신/발행 시각으로 바꾸는 setter가 없다.

`Publication`은 publisher/boot/uint64 sequence/`published_at_ns`/schema_version을 관측 바깥에 둔다. 이 전송 envelope를 바꿔도 관측의 의미 payload hash는 변하지 않는다. 현재 schema는 `1.0`만 지원한다. 발행 sequence의 실제 증가·수신 queue/QoS를 담당하는 transport adapter는 아직 없다.

`validate_freshness`는 `now_ns`·receiver clock domain/`receiver_clock_error_bound_ns`와 `TimePolicy(max_age_ns, max_future_ns, max_clock_error_ns, stamp_delay_bound_ns)`를 검사한다. UNKNOWN stamp/clock·미확인 오차·다른 domain·과도한 clock 오차·미래 stamp·나이 초과·최초 유효기간 만료는 유효 임무 입력으로 거절한다. 보수적 나이는 `now_ns−observed_at_ns+송수신 clock 오차 ns+센서 stamp 지연 상한 ns`이며 반환값도 int ns다. 원 유효기간도 오차/지연을 더한 현재 시각으로 검사한다. CAPTURE/DRIVER_RECEIVE 각각의 실제 의미·지연 상한은 설치 adapter에서 측정해야 한다.

**패키지 경계 시간 계약:** 모든 관측·TF query·만료·발행·현재 wall 시각은 `timestamp_ns`가 검사하는 양수 int64 ns, 정책 duration과 시계 오차는 `duration_ns`가 검사하는 0 이상 int64 ns다. 로컬 `received_steady`만 프로세스 내부 monotonic float 초이며 다른 PC 값과 비교하지 않는다. ROS 헤더는 공통 `ros_time_to_ns(sec, nanosec)`로 정확히 합치고 `ns_to_ros_time(ns)`로 분리한다. 실측 stamp를 float epoch 초로 바꿨다가 복원하지 않는다. 실제 epoch `1700000000123456789`의 1 ns 보존/Depth pair 경계와 TF 조회 입력 exact equality를 시험했다. 공통 `seconds_to_ns`는 decimal 정책/fixture 입력에만 쓰며 이미 소실된 ROS stamp 정밀도를 복구하는 API가 아니다. 구 API의 float 초 필드·인자는 호환 경로 없이 폐기하므로 소비자도 함께 갱신한다.

**오류 계약:** 순수 Python 공통 패키지 `rescue_contracts`는 ROSIDL 메시지 구현이 아니다. `PerceptionError`는 `RescueError`를 상속하고 소비자가 분기할 canonical `.code`, `rescue_perception.<원인>`의 `.detail_code`, `.component`, 상세 `.detail`을 보존한다. 대응표는 공통 `canonical_code` 하나가 소유한다. 예를 들어 FUTURE_OBSERVATION은 CLOCK_UNCERTAIN, UNSUPPORTED_DEPTH/MODEL_CLASS_MISMATCH는 INVALID_INPUT으로 분류된다. `LocalizationResult.reason`은 위치 상태를 설명하는 기존 어휘이고 예외 코드가 아니다. 실패 결과에도 `.error_code`/`.detail_code`를 제공하며 LOCATED에는 둘 다 None이다.

`ObservationRegistry.ingest`는 수신 시계 관문을 통과한 뒤 ID 충돌을 검사하고 현재 최신성을 평가한다. 같은 ID·같은 payload는 DUPLICATE이며 기존 객체를 반환한다. 새 ID이지만 원 stamp가 역순이면 기록은 보존하고 REORDERED를 반환한다. 같은 frame stamp의 서로 다른 사람은 각각 ACCEPTED하되 공백 타이머는 갱신하지 않는다. source robot/camera/boot별로 **원 stamp가 전진하는 새 유효 관측만** last_new_receive를 갱신한다. 만료된 재발행은 거절되고, 이전 boot의 수신 기록은 새 boot로 이어지지 않는다.

**역순 발견의 설계 결정:** 역순을 재발행과 같다고 보지 않는다. 새로운 observation_id의 REORDERED 관측도 최초 원본과 Admission 상태를 `history()`의 불변 tuple에 남긴다. 중복은 기존 이력을 바꾸지 않고 타이머를 갱신하지 않는다. 현재 ConservativeEventRegistry는 REORDERED에서 신규 자동 사건을 만들지 않는다. 이는 발견 삭제가 아니라 기록과 자동 허가의 분리다. 다중 카메라 지연으로 뒤늦게 발견된 다른 사람을 보고/재확인 후보로 승격하는 후속 사건 정책은 관측 나이·공통 좌표·동일성 불확실성·원본 provenance를 검사해야 하며, 과거 frame의 도착만으로 최신 관측·새 접근 권한을 부여하지 않아야 한다. 별도 정책을 검증하기 전까지 현재 제한을 유지한다.

유효한 receiver wall/monotonic 시각은 관측이 오래됐거나 ID 충돌로 거절되어도 추적한다. 어느 수신 시계든 역행하면 해당 registry의 clock fault를 latch하여 이후 정상 범위의 시각·새 관측·새 source boot에도 계속 `CLOCK_UNCERTAIN`을 반환한다. 기존 수신 기록을 성공으로 갱신하지 않는다. caller가 시간 기준을 복구·재검증한 뒤 새 receiver session(`ObservationRegistry`)을 만들어야 한다. 새 객체 생성 자체가 시계 동기화의 증거는 아니며, source 프로세스 boot 변경도 receiver 시계 fault를 해제하지 않는다.

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
    now_ns=wall_time_ns,
    received_steady=local_monotonic,
    receiver_clock_domain=clock_domain,
    receiver_clock_error_bound_ns=clock_error_ns,
    policy=time_policy,
)
event = ConservativeEventRegistry().register(admission, event_id=new_event_uuid)
# observation.localization.point_optical -> mapping adapter; Nav2 goal이 아님.
```

저장소 루트에서 `PYTHONPATH=src/rescue_contracts:src/rescue_perception python3 -m unittest discover -s tests -p test_perception.py -v`로 실행한다. 34개 unittest는 등록 깊이의 수학·단위, 잘못된 정합/해상도/시각, 깊이 결측·불연속, 모델 class, UUID·NaN/Inf/bool, 관측 불변·중복/충돌/역순/만료, receiver 시계 역행 latch·거절 입력 뒤 역행 감지, 복수 대상과 로봇별 사건 분리, 실제 epoch ns 경계·공통 오류·tracking_mode·역순 이력을 다룬다. 이는 CT의 E2 일부이며 ROS/센서/모델 성능·실제 표적 위치/접근 검증은 NOT_RUN이다. TF 누락/SLAM revision은 이 코어의 PASS로 대신하지 않는다.

## 원리 참고와 남는 관문

새 구현은 원본 함수를 복사하지 않았다. 다음 고정 원본을 읽고 실패 조건과 필요한 수학 원리를 새 데이터 계약으로 작성했다.

- [P4 depth 디코딩·유효 중앙값·역투영](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/rokey_vision/rokey_vision/common.py#L172-L251): encoding/단위와 유효 표본, pinhole 원리를 참고. 원본 numpy/OpenCV·반경/거리 기본값·압축 decoder는 이식하지 않음.
- [P4 RGB/Depth/탐지 시간 동기](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/rokey_vision/rokey_vision/depth_distance.py#L113-L132): 헤더 pair의 정합 필요를 참고. ROS synchronizer 구현과 설정을 복사하지 않음.
- [P4 관측 시각 TF](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/target_locator.py#L74-L107): 원 stamp 보존을 참고. 최신 TF fallback이나 지도 변환을 이 PR에서 구현하지 않음.
- [AMR hold 재발행](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/webcam_detector.py#L109-L159): 과거 좌표와 새 발행 stamp를 혼동하는 경로를 방지하는 시험 근거. 최고 confidence 단일 대상/고정 평면 보정값을 채택하지 않음.

프로젝트 라이선스·maintainer의 기존 미결정 표시는 유지한다. 이 PR은 원본 소스·가중치·데이터·카메라 보정 파일을 복제하지 않는다. 새 대상의 독립 평가 데이터, 실제 모델/가중치 검증, CameraInfo/rectification/Depth 정렬·시각 의미, covariance, 실시간 adapter와 E3/E4의 2 AMR·배포 시험은 후속 관문이다.

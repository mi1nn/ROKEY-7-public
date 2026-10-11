---
title: 기록 입력 기반 인식 코어 구현
status: draft-implementation-e2
updated: 2026-10-10
---

# 기록 입력 기반 인식 코어

계획 PR 7은 [C-02·03·05](../design/contracts.md), [D-06](../design/algorithm-decisions.md), [CT-01·02·03·10](../process/acceptance-plan.md)의 일부를 실행 가능한 순수 Python 로직으로 구현한다. RGB 픽셀에서 사람을 찾는 모델은 아직 없다. 입력은 이미 얻은 탐지 bbox/class와 RGB/Depth 메타데이터, 복호화된 깊이 배열이다. 이 PR에서 실제 YOLO, 가중치, ROS, CameraInfo 구독, TF, 카메라, 두 로봇을 실행하지 않았다.

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

## 고정 웹캠 지진 징후 (E2, 실측 전)

`rescue_perception.disaster_sign`은 고정 웹캠 영상만으로(모델 없이) `rescue_contracts.SiteEvent(kind="DISASTER_SIGN")`를 만든다. 사건은 증거이고 재난 모드 전환은 협업·관제의 정책이 한다([계약](../design/contracts.md) C-01, [S-00](../design/state-machines.md)). 판정 방식은 PR #33 설계 v3 합의(2026-10-09)다. 화면 **전체**가 앞뒤로 여러 번 흔들리면 지진 징후로 본다.

| 단계 | 규칙 |
| --- | --- |
| 이동량 | 회색조 격자를 `blocks_x`×`blocks_y` 블록으로 나누고, 블록마다 직전 프레임에서 ±`search_cells` 칸 안을 ZNCC 로 찾는다. 평균을 빼고 정규화하므로 조명이 전체로 밝아지거나 어두워져도 이동으로 보지 않는다 |
| 영상 정상 | 무늬 있는 블록(표준편차 ≥ `texture_min`)이나 매칭된 블록(ZNCC ≥ `match_min`)이 `valid_ratio_min` 미만이면 UNKNOWN(가림·어두움·매칭 실패). UNKNOWN 은 흔들림 횟수를 더하지도 지우지도 않는다 |
| 화면 전체 | 원래 장면이 보이는 블록(아래 장면 확인)의 이동 중앙값을 화면 이동으로 보고, 네 사분면 모두에 그와 `agree_tol` 안으로 맞는 블록이 있어야 한다. 아니면 부분 움직임(사람 통과)이라 조건 거짓. 손·사람이 덮은 블록은 투표에서 빠진다 |
| 장면 확인 | 카메라가 흔들리면 장면은 같고 위치만 밀리지만, 렌즈 앞 손처럼 가까운 물체는 장면 자체를 가린다. 흔들림 없이 `window_ns` 이상 멈춰 있던 화면을 기준 장면으로 두고, 블록마다 지금 화면이 기준 장면을 ±`search_cells` 칸 옮긴 것과 ZNCC ≥ `scene_match_min` 이면 "보임". 보이는 블록이 `visible_min` 미만이면(화면 대부분이 가려짐) UNKNOWN(`SCENE_CHANGED`), 보이는 블록이 `valid_ratio_min` 미만이면서 한 사분면에 보이는 블록이 하나도 없으면 사분면 확인을 할 수 없으므로 UNKNOWN(`QUADRANT_COVERED`), 그보다 덜 가려지면(조금 떨어져 지나가는 사람, 위쪽으로 지나가며 한 구역만 가림) 판정을 계속한다. 사람이 있으면 그림자·자동 노출로 가려지지 않은 바닥 칸도 기준 장면과 달라질 수 있어, 한 구역 가림만으로 보류하면 지나가는 사람에도 보류가 잦았다(17:51 실측 지나가기 97/335). 16:06 녹화 재생 [측정]: 지나가기 보류 11/377, 가리기 50/437, 흔들기·가만히 보류 0. 카메라 바로 앞에 서서 화면 절반 가까이를 막으면 여전히 보류된다(담당자 결정 2026-10-10: 가림을 잡되 지나가는 사람은 보류하지 않음). 손에도 무늬가 있어 무늬 기준(`texture_min`)만으로는 가림을 못 잡는다(2026-10-10 녹화에서 가려진 블록의 표준편차 중앙값 13.2 [측정]). 바뀐 화면이 `rebase_ns` 이상 안정적이면 카메라가 옮겨진 것으로 보고 새 기준 장면으로 삼는다 |
| 횟수 | 이동 크기 ≥ `shift_min` 이면 방향을 기록하고, 직전 방향과 반대(내적 < 0)면 전환 1회. `window_ns` 안의 전환이 `shakes_min` 이상이면 조건 참 |
| 사건 | #33 규칙 그대로. 조건이 `hold_ns`(≤ `window_ns`) 이어지면 사건 1개, `release_ns` 동안 거짓이어야 다시 낸다. 간격 > `max_gap_ns` 면 횟수·구간·직전 프레임을 버린다. 중복·역순·디코딩 거절 프레임은 상태를 바꾸지 않는다 |
| 시각·ID·confidence | #33 그대로. `observed_at_ns`는 hold 를 채운 프레임, 시작은 `detail`의 `onset_ns`. uuid5 ID, 시간 기준은 `SourceBinding`에서. `confidence=1.0`은 "규칙 충족(이진)" |

정책 값은 모두 Q-11 로 정하고 **기본값이 없다**. 진도 7 은 시나리오 근거일 뿐 웹캠으로 재지 않는다. 시험은 `tests/test_perception_disaster_sign.py`(30개, 합성 48×48 영상)다. 조명 오탐, 해상도 변경 `IndexError`, 렌즈 앞 손, 잠깐 멈춘 손, 무늬 있는 손의 가림, 화면 일부를 가리며 지나가는 사람, 화면 대부분을 덮고 움직이는 손, 한쪽을 덮은 손, 위쪽으로 지나가는 사람의 회귀 시험은 고치기 전 코드에서 실패함을 커밋으로 남겼다.

**실측 도구:** `tools/webcam_quake_probe.py`가 OpenCV 로 웹캠 프레임을 읽어 위 판정에 넣고 대시보드로 보여 준다. 왼쪽은 카메라 화면(블록 격자·사분면 경계·이동 화살표·상태 색 테두리·사건 배너), 오른쪽은 상태 카드(정상·흔들림 감지·판단 보류·카메라 끊김과 이유), 상황 칩, 방향 전환 칸 막대, 무늬·직전 화면 일치·원래 장면 보임 게이지(눈금 = 기준), 최근 5초 이동 그래프, 사건 기록이다. 숫자 키로 상황(가만히·흔들기·조명·지나가기·가리기)을 표시한다. 결과는 측정마다 `.artifacts/webcam_quake/<날짜-시각>/`(`.gitignore` 대상)에 `probe.csv`(프레임별 기록), `meta.json`(인자·정책·git SHA·상황별 요약), `--record` 이면 `view.mp4`(대시보드 화면)로 남는다. `--video`로 영상 파일을 같은 판정에 넣을 수 있다. 카메라가 끊기면 종료하지 않고 다시 연결한다. 인자 기본값은 측정 시작값이지 Q-11 값이 아니다(장면 일치 0.6·보임 30% 는 2026-10-10 녹화 분석에서 고른 시작값).

**한계:** 횟수만 보고 세기는 보지 않는다(시연용 판정). 순수 Python 이라 해상도·`sample_stride`·`search_cells`에 따라 처리 시간이 커진다. 웹캠 자동 초점은 화면 배율을 바꿔 전체 이동처럼 보일 수 있으므로 고정한다. 실제 웹캠 측정·오탐률·처리 속도는 NOT_RUN 이다.

## 고정 웹캠 인형 탐지 모델 비교 (E2, 2026-10-11, 결정 전)

출구 인형 카운트([PR #78](https://github.com/jaebeom/ROKEY-7/pull/78) Q-26 [제안])에 쓸 탐지 모델 후보를 같은 데이터·같은 설정으로 비교했다. 클래스는 `survivor` 하나다([PR #84](https://github.com/jaebeom/ROKEY-7/pull/84)). 모델 가중치·영상·데이터셋은 저장소에 넣지 않는다(`.gitignore` 대상 `.artifacts/`, 선정 뒤 `assets/manifest/` 에 출처·해시 기록).

**도구**
- `tools/webcam_record.py`: 학습·평가 영상 촬영. `gui` 는 미리보기·시작/일시정지·장면 표시·경과 시간을 보여 주고, 요청 해상도를 카메라가 주지 않으면 녹화하지 않는다. 카메라가 요청보다 낮은 fps 를 주면 실제 fps 로 다시 저장해 재생 속도를 맞춘다(2026-10-11 실측: 요청 30fps 에 15~16fps). `split`·`conditions` 는 학습/평가 분리와 평가 조건 표를 만든다.
- `tools/colab/survivor_model_compare.ipynb`: Colab 한 번 실행으로 여러 모델 학습·평가·속도 측정·오탐/미탐·bootstrap 신뢰구간·Notion 요약까지. 끝난 모델은 건너뛰어 이어서 실행할 수 있고, 한 모델이 실패해도 나머지를 계속하며 실패를 기록한다.

**데이터** [측정, 2026-10-11]
- 첫 데이터셋(v1)은 영상 하나의 프레임을 무작위로 나눠 valid·test 60장 중 58장이 학습 프레임의 바로 이웃이었다. 인형 부분 유사도 ≥ 0.95 가 v1 valid 39/43 이라 점수(mAP50-95 0.98)가 일반화 성능이 아니었다.
- v2(데이터셋 이름 `survivor-webcam-v2`, 파일 `survivor-v2-merged.zip`)는 train 을 v1 학습 그대로(720장, 원본 240 × 증강 3), valid·test 를 **따로 찍은 영상**에서만 만들었다(valid 236장·인형 222, test 246장·인형 236). 같은 지표로 VAL 0/222, TEST 0/236. valid·test 는 같은 시간대·같은 사람이 연달아 찍어 조건이 비슷하다.

**결과** (Colab T4, 100 epoch·patience 30. test 는 학습·epoch 선택에 쓰지 않은 영상이라 판단 기준. 괄호는 bootstrap 95% 구간)

| 모델 | 파라미터(M) | test mAP50-95 | val mAP50-95 | test 오탐 / 미탐 | CPU ms/장(Colab) |
| --- | --- | --- | --- | --- | --- |
| **yolo26n** | 2.38 | **0.920** (0.900~0.937) | 0.958 | 8 / 7 | **107** |
| yolo26s | 9.47 | 0.901 (0.877~0.920) | 0.958 | 18 / 6 | 312 |
| yolov8s | 11.13 | 0.897 (0.875~0.923) | 0.960 | 5 / 4 | 299 |
| yolo11s | 9.41 | 0.897 (0.868~0.918) | 0.948 | 6 / 6 | 281 |
| rtdetr-l | 31.99 | 0.894 (0.867~0.918) | 0.966 | 4 / 9 | 1,245 |
| yolov8n | 3.01 | 0.882 (0.865~0.908) | 0.962 | 2 / 6 | 116 |
| yolo11n | 2.58 | 0.881 (0.862~0.903) | 0.952 | 6 / 3 | 155 |

- yolo26n 은 test 에서 나머지 6개 모델 모두보다 높고 차이의 95% 구간이 모두 0 보다 크다(+0.020~+0.037). 나머지 6개끼리는 test 에서 15쌍 모두 구분 불가.
- 그래서 **yolo26n 을 출구 인형 카운트 모델로 제안한다** [제안]: 정확도·크기·CPU 속도 모두 1위. 확정 조건은 PC3 실측 fps.
- mAP50 은 모든 모델이 0.965~0.995 로 포화돼 차이가 mAP50-95(상자 위치 정확도)에서 난다.
- yolo26n 은 첫 실행에서 노트북 오류로 학습되지 않아(GPU AMP 점검이 같은 이름의 가중치를 설정 폴더에 받아 두어 해시 단계 실패) 실제 읽은 경로로 해시하게 고친 뒤 같은 설정으로 다시 돌렸다.
- 한계: valid·test 를 같은 시간대·같은 사람이 찍었다. 배경 이미지가 valid 14·test 10장뿐이라 "배경 오탐 0" 의 근거가 약하다. 조건별 분석 없음.
- NOT_RUN: PC3 실측 fps, 운영 신뢰도 기준, 실시간 노드, 실기(E3·E4).
- 전체 보고서(데이터·증강·하이퍼파라미터·모델 간 비교·가중치 해시)는 팀 Notion 「survivor 탐지 모델 선정 보고서 — survivor-webcam-v2」.

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

## 입력 어댑터 (기록 입력, E2)

`rescue_perception.adapter`는 `sensor_msgs/Image`(RGB·Depth)·`CameraInfo`·`vision_msgs/Detection2D`와 **필드 이름이 같은 객체**를 받아 위 코어로 넘긴다. ROS를 import하지 않으므로 rclpy 메시지, bag에서 역직렬화한 메시지, 시험용 객체를 똑같이 받는다.

| 단계 | 함수 | 하는 일 · 거절 |
| --- | --- | --- |
| 시각 | `stamp_ns(header)` | `ros_time_to_ns(sec, nanosec)` 하나로만 정수 ns를 만든다. float `sec`는 `INVALID_INPUT` |
| RGB | `rgb_frame(image, binding)` | `FrameMeta` 와 픽셀 바이트의 실제 SHA-256(관측 증거). `data` 길이 ≠ `step × height` 면 거절 |
| Depth | `depth_frame(image, binding)` | `16UC1`(mm)·`32FC1`(m)만. `is_bigendian`·행 패딩(`step`)을 지켜 표준 라이브러리 `struct`로 푼다. 다른 encoding(`mono16`, 압축)은 변환하지 않고 `UNSUPPORTED_DEPTH` |
| 보정 | `camera_intrinsics(info, binding, rectified=)` | `rectified`는 caller가 넣는 근거다(구독한 영상이 rectified 스트림인지). `d`에서 추정하지 않는다. rectified면 ROS 관례대로 `p`, 아니면 `k`를 쓰고 코어가 `ALIGNMENT_UNVERIFIED`로 위치를 거절한다. `p` 이동 성분 ≠ 0(보조 스테레오 카메라)·binning·부분 ROI는 거절 |
| 탐지 | `detection_from_vision_msgs(det, model, tracking_mode=)` | 가설이 정확히 하나여야 한다(최고 점수를 몰래 고르지 않음). `tracking_mode`는 caller가 반드시 넣는다. `TRACKING`이 아닌데 `det.id`가 있으면 거절한다 |
| 묶음 | `observe(rgb, depth, info, detection, detection_header, …)` | 탐지 헤더의 stamp·frame_id가 RGB와 1 ns라도 다르면 거절(다른 프레임의 상자를 붙이지 않음, LL-06). `valid_until_ns = observed_at_ns + ObservationPolicy.validity_ns`이고 수신 시각과 무관하다 |

**소스 정보는 밖에서 넣는다.** `SourceBinding`은 `source_robot`·`source_camera`·`source_boot_id`·clock domain·**스트림별** stamp 의미·`calibration_id`·clock 상태·오차 상한을 가진다. 어댑터는 토픽 이름이나 메시지 내용으로 이것을 추정하지 않는다. `source_robot=None`(로봇이 아닌 소스, 예: 고정 카메라)이면 `LocalizationResult`까지만 만들고 `observation=None`, `observation_blocked="SOURCE_IDENTITY_UNDEFINED"`를 돌려준다. 지금 `Observation` 계약에는 로봇 식별만 있으므로, 고정 카메라의 식별은 공통 계약이 정할 때까지 임시 값으로 채우지 않는다. 그때도 `FrameMeta`·`LocalizationResult`를 그대로 입력으로 쓸 수 있게 두 단계를 나눴다.

시험은 `tests/test_perception_adapter.py`(14개)다. 2026-10-08 기준 녹화 bag이 없어 **모든 입력이 합성**이다. 실제 epoch `1700000000123456789` ns가 헤더 → `FrameMeta` → `Observation`까지 리터럴로 같은지, 정렬·시각·깊이 실패 7종이 위치 없이 관측으로 남는지, 중복·역순 입력이 `last_new_receive`와 `valid_until_ns`를 바꾸지 않는지를 확인한다. **실시간 성능은 미검증(NOT_RUN)이다:** Depth 바이트 전수 디코딩과 `DepthFrame` 전수 검사가 모두 Python 루프다.

## 실제 카메라 연결 조건

아래는 원본 P4 로봇 3번의 당시 기록이다. 파이널 로봇(`Q-01`)에서 다시 재야 하며, 재기 전에는 `SourceBinding`의 stamp 의미를 `UNKNOWN`으로 둔다.

| 항목 | 이 어댑터가 요구하는 것 | 원본 기록 |
| --- | --- | --- |
| Depth 형식 | 비압축 `sensor_msgs/Image` `16UC1` 또는 `32FC1` | 원본은 대역폭 때문에 `compressedDepth`(16비트 PNG + 12바이트 헤더)를 받았다 [[원본 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/reference/2026-10-02-day2-ai-vision.md#L36-L36)]. 압축 해제 어댑터는 이 PR에 없다 [미확인: 파이널에서 쓸 전송 형식] |
| 정렬 | Depth가 RGB rectified 격자에 등록되고, 해상도·`frame_id`·`calibration_id`가 같다 | 후보 D 설정에서 두 `camera_info` 크기 같음, `K` 차이 0, Depth `frame_id` = RGB 광학 프레임 [측정, 2026-10-06; [원본 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/decisions.md#L103-L104)]. 다음 날 stereo 640×360으로 바뀌었다. `camera_info` 일치는 설정 일관성일 뿐 정렬 증거가 아니다 [[원본 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/reference/2026-10-03-pr-review.md#L58-L58)]. 겹쳐 보기와 줄자 비교가 있어야 `AlignmentEvidence.verified=True`로 넣는다 [미확인] |
| 해상도·주기 | 제한 없음. RGB·Depth·CameraInfo 크기가 같아야 위치를 낸다 | RGB 1280×720 15 fps, 수신 지연(수신 − 헤더) 82 ms, Depth `compressedDepth` 6.3 Hz [측정, 2026-10-06; [원본 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/decisions.md#L90-L90)]. 이 82 ms는 시계 오차를 포함한 차이라 stamp 의미를 정하는 근거가 아니다 |
| stamp 의미 | 스트림마다 `CAPTURE`/`DRIVER_RECEIVE`를 실측해 넣는다 | [미확인] — 원본에서 재지 않았다 |
| RGB–Depth stamp 차 | `DepthPolicy.max_stamp_skew_ns` 이하 | [미확인] — 값은 실측 뒤 정책으로 정한다 |
| 탐지 입력 | 탐지 헤더 = 그 RGB 프레임의 헤더, 가설 1개, 추적 모드 명시 | 모델 추론 연결은 후속 PR이다 |
| QoS | 어댑터는 QoS를 모른다. 구독 노드(후속)가 정한다 | 원본 RGB preview 250×250은 RELIABLE이었다 [측정, 2026-10-01; [원본 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/reference/2026-10-01-lab-environment-baseline.md#L50-L50)]. Depth·CameraInfo QoS는 [미확인] |
| 센서 가용 | — | 원본 로봇은 도킹 중 카메라를 끄는 설정이었다 [측정, 2026-10-01; [원본 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/reference/2026-10-01-lab-environment-baseline.md#L51-L51)] |

후속: 로봇·지도가 로봇 배정 뒤 녹화한 bag으로 같은 시험을 다시 돌린다(bag과 해시는 `assets/manifest/`). 압축 영상 해제, ROI/array 실시간 어댑터, ROS 구독 노드는 그 뒤다.

## 원리 참고와 남는 관문

새 구현은 원본 함수를 복사하지 않았다. 다음 고정 원본을 읽고 실패 조건과 필요한 수학 원리를 새 데이터 계약으로 작성했다.

- [P4 depth 디코딩·유효 중앙값·역투영](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/rokey_vision/rokey_vision/common.py#L172-L251): encoding/단위와 유효 표본, pinhole 원리를 참고. 원본 numpy/OpenCV·반경/거리 기본값·압축 decoder는 이식하지 않음.
- [P4 RGB/Depth/탐지 시간 동기](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/homework/day2/rokey_vision/rokey_vision/depth_distance.py#L113-L132): 헤더 pair의 정합 필요를 참고. ROS synchronizer 구현과 설정을 복사하지 않음.
- [P4 관측 시각 TF](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/target_locator.py#L74-L107): 원 stamp 보존을 참고. 최신 TF fallback이나 지도 변환을 이 PR에서 구현하지 않음.
- [AMR hold 재발행](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/webcam_detector.py#L109-L159): 과거 좌표와 새 발행 stamp를 혼동하는 경로를 방지하는 시험 근거. 최고 confidence 단일 대상/고정 평면 보정값을 채택하지 않음.

프로젝트 라이선스·maintainer의 기존 미결정 표시는 유지한다. 이 PR은 원본 소스·가중치·데이터·카메라 보정 파일을 복제하지 않는다. 새 대상의 독립 평가 데이터, 실제 모델/가중치 검증, CameraInfo/rectification/Depth 정렬·시각 의미, covariance, 실시간 adapter와 E3/E4의 2 AMR·배포 시험은 후속 관문이다.

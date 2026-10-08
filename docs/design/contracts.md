---
title: 구조 수색 시스템의 인터페이스 계약 초안
status: draft-proposal
updated: 2026-10-08
---

# 인터페이스 계약 초안

이 문서는 계획 PR 3의 계약과 후속 Python 코어 구현의 공통 의미를 관리한다. C-02/C-03/C-09의 실행 가능한 Python 계약은 `rescue_contracts`에 두며 ROS IDL `rescue_interfaces`는 아직 미구현이다. PR 2의 책임·배포 구조를 전제로 검토하되, 실제 `.msg/.srv/.action`, 패키지 코드, ROS 설정을 이식하지 않는다. 장비·OS·ROS 배포판·센서 시각 의미·네트워크는 아직 확정되지 않았다. 아래 타입 이름과 토픽·QoS는 **계약 제안**이며 구현·실기 검증 완료를 뜻하지 않는다. 수치의 확정 절차는 [시험 계획](../process/acceptance-plan.md), 사건·임무 전이는 [상태 전이](state-machines.md)에 둔다.

두 AMR 활용은 사용자 확정 요구다. 고사양 PC 2대·중간 사양 PC 2대는 예정 자원이며, 네 대를 모두 사용하는 배포와 발견→안전 접근→보고는 현재 설계 기준안이다. 이 문서의 계약을 최종 인수 범위가 모두 확정된 것으로 해석하지 않는다.

## C-01. 책임과 명령 권한

| 책임 | 유일한 소유자 제안 | 허용/금지 경계 |
| --- | --- | --- |
| 센서 드라이버·최종 명령 감시 | 각 로봇의 드라이버/로컬 watchdog | 최종 속도 제한·명령 만료·정지 latch를 로봇에서 적용. 정확한 설치 방법은 장비 확인 필요 |
| 로봇별 Nav2/SLAM 실행·임무 실행 | `rescue_robot` + `rescue_bringup`의 로봇별 인스턴스 | Nav2 goal/cancel·도킹 장치의 준비/운영 action은 로봇 실행기만 발행. 협업 임무와 준비용 UNDOCK/DOCK 권한을 분리(C-06) |
| 지도 세션·공통 좌표 변환 | `rescue_mapping` | frame/session/revision의 권한자. perception/coordinator가 변환을 임의 보정하지 않음 |
| 관측 생성 | `rescue_perception` | 탐지·거리·위치·관측 품질 발행. 임무·Nav2 goal·속도 발행 금지 |
| 수색 분담·대상 사건·임무 배정 | `rescue_coordination` | 로봇 실행기에 임무 요청. 직접 `cmd_vel`·Nav2 action·lifecycle 변경 금지 |
| 운영자 요청·보고 수신·영속 기록 | `rescue_operations` | API 요청을 coordination/robot 계약으로 전달. 직접 Nav2/속도 발행 금지 |
| ROS 메시지와 버전 | `rescue_interfaces` 담당 1명 | 관련 소비자·시험의 변경 영향과 함께 수정. 각 구현 세션이 독자 계약을 추가하지 않음 |
| 고정 카메라 A·B 의 현장 사건(`SiteEvent`) | `rescue_perception` 의 고정 카메라 어댑터가 생성, `rescue_coordination` 이 소비 | 재난 징후(`DISASTER_SIGN`, 판정 조건은 정책 값 — 지진 시연은 흔들림·가려짐)·출구 통과(`EXIT_PASS`)는 **증거**이지 명령이 아니다. 재난 모드 전환·출동 지시는 coordination 의 정책이 하고, 카메라 어댑터는 주행·모드 변경을 발행하지 않음(BRD v0.1 UC-01·06, 2026-10-08) |

고사양 PC 두 대에 로봇별 SLAM/Nav2/실행·추론을 배치하고, 중간 사양 PC 두 대에 coordination과 operations를 나누는 것은 부하 측정 전 가설이다. 순수 판단 로직은 ROS 입출력 어댑터와 분리한다. 파일 경계는 PR 2와 함께 확정한다.

Nav2·탐색 회전·수동 명령은 모두 같은 로봇별 명령 중재와 최종 watchdog 경로를 통과해야 한다. `explore_lite`는 통합 시 후보 생성 방식(B)을 평가하며 실행기가 후보를 검증해 Nav2 goal을 발행한다. `explore_lite`가 직접 goal을 보내는 방식(A)은 coordinator/controller가 같은 로봇을 제어하지 않는 격리 실험으로 한정한다. perception이 별도 회전 명령을 보내는 경로는 두지 않는다. 원격 연결이 끊겨도 로봇에서 명령 만료를 처리할 수 있다는 근거를 얻기 전에는 원격 자율주행 준비 완료로 판정하지 않는다.

## C-02. ID·버전·공통 envelope

- 논리 로봇 ID는 초안에서 `robot_a`, `robot_b`로 쓴다. 실제 robot3/robot6 등 장비 이름과의 매핑은 배포 설정이다.
- 모든 사용자 정의 메시지는 전송 envelope에 `schema_version`, `publisher_id`, `publisher_boot_id`, `sequence`를 가진다. publisher_boot_id는 발행 프로세스 기동별 UUID, sequence는 같은 발행 boot 내 단조 증가 `uint64`다. 원 관측 payload의 `source_robot/camera`, `source_boot_id`, 원 관측/발생 시각은 별도로 보존한다. 중계자의 identity를 원 센서/관측 생성자의 identity로 덮어쓰지 않는다.
- ID(`observation_id`, `target_id`, `task_id`, `command_id`, `report_id`)는 문자열 UUID를 제안한다. `generation`·revision·sequence는 `uint64`; 단위는 거리 m, 각도 rad, 시간 s/ns로 명시한다. NaN/Inf, 음수 거리·분산, 누락 필수 ID는 거절한다.
- 메시지 버전의 major 불일치는 자동 변환하지 않고 `INCOMPATIBLE_SCHEMA`로 거절한다. 허용하는 minor 확장과 기본값은 구현 전에 명시한다.
- 불변 관측·명령·보고의 재전송은 같은 ID와 같은 **의미 payload**다. 중복 검사에 사용하는 정규화 payload 해시에서 published_at, publisher identity/boot, 전송 sequence, 수신 시각·재전송 횟수 같은 전송 envelope는 제외한다. observed_at, 원 source identity/boot, 대상/임무·좌표 참조, 최초 deadline, 원 증거 해시는 포함한다. 같은 ID의 의미 payload가 다르면 `ID_CONTENT_CONFLICT`로 거절한다. 재시도 횟수가 늘어도 최초 deadline을 늘리지 않는다.
- 변경 가능한 상태는 ID만으로 중복 판정하지 않는다. TargetEstimate는 `(target_id, estimate_revision)`, 로봇 상태는 `(robot_id, robot_boot_id, state_sequence)`, 좌표 snapshot은 authority/session/revision으로 버전을 구분한다. 더 높은 정상 revision은 업데이트이며 같은 키의 다른 의미 payload만 충돌이다. ReportReceipt는 자체 receipt_id와 참조 report_id를 가지므로 RECEIVED→PERSISTED 전이를 보고 payload 변경과 혼동하지 않는다.
- boot 변경 시 이전 boot의 sequence를 새 sequence와 비교하지 않는다. 이전 boot의 임무·lease·frame 세션을 명시적으로 폐기하고 준비 절차를 다시 밟는다. retained 상태 한 개를 받았다는 이유로 새 boot가 READY가 되지 않는다.

로봇의 **역할**(`SCOUT` 정찰 / `SUPPORT` 구조 지원, BRD v0.1 TB1/TB2)은 배포 프로파일 v2 `mission.roles` 의 데이터이며 정확히 하나씩이다. 코드는 역할을 상수로 두지 않고 프로파일에서 받는다(역할을 바꿔 넣으면 두 로봇이 뒤집혀 동작해야 한다). 고정 카메라는 `cam_<id>` 형식의 `camera_id` 로 식별하며 `robot_id` 집합에 들어가지 않는다. `robot_id` 값은 예시 `robot_a`/`robot_b`로 고정하지 않는다. DTO는 `[A-Za-z][A-Za-z0-9_]{0,63}` 형식을 검사하고 실행기·배정기·보고 저장소 경계는 검증된 배포 프로파일에서 주입한 정확히 2개의 distinct ID를 허용한다. 기본 allowlist는 없으며 수신 데이터가 설정 집합을 변경하지 못한다.

## C-03. 시각·최신성 계약

Python 패키지 사이의 절대 시각은 양의 **int64 나노초** (`*_at_ns`, `valid_until_ns`, `deadline_ns`, `now_wall_ns` 등)로 고정한다. bool/float/0/음수/범위 초과는 거절한다. ROS header는 `ros_time_to_ns(int sec, int nanosec)`로 합성하고 `ns_to_ros_time`의 정수 divmod로 복원한다. 원 stamp를 float 초로 변환했다가 되돌리지 않는다. 실제 epoch `1700000000123456789` ns를 왕복해 하위 ns까지 보존하는 회귀시험을 유지한다. 문서의 suffix 없는 논리 필드도 이 표현을 따른다.

수신 프로세스 **내부** monotonic deadline/lease/stop-window는 유한 float 초를 사용할 수 있다. 이 값은 직렬화하거나 다른 PC의 monotonic 시각과 비교하지 않는다. 절대 시각에 적용하는 기간/오차 허용치는 정수 ns다. `seconds_to_ns`는 십진 정책값·합성 fixture 변환 전용이며 이미 float로 손상된 sensor stamp 복구 수단이 아니다.


| 값 | 의미와 사용 |
| --- | --- |
| `observed_at` | 원 센서 관측의 ROS system-time stamp. 재발행·변환·DB 저장 시 바꾸지 않음 |
| `stamp_semantics` | `CAPTURE`, `DRIVER_RECEIVE`, `UNKNOWN`. 실제 드라이버가 어느 시점을 찍는지 확인. 수신 stamp를 캡처 stamp로 부르지 않음 |
| `published_at` / `issued_at` | 메시지 발행/명령 최초 발급 시각. 원 관측 시각과 별개 |
| `valid_until` / `deadline` | 원 관측 또는 임무 정책에서 정한 절대 유효 시한. 중계가 연장하지 않음 |
| `received_steady` (로컬 초) | 각 수신 프로세스 내부의 monotonic 시각. 통신 watchdog·처리 지연에 사용하며 다른 PC의 monotonic 값과 비교하지 않음 |
| `clock_domain_id`, `clock_state`, `clock_error_bound` | 동일 시간 기준인지, 동기화 상태·최대 오차 추정이 사용 가능한지 표시. 단순한 header-수신 시각 차이만으로 동기화를 주장하지 않음 |

실기에서는 system time을 공통 기준으로 사용하고 `use_sim_time=false`를 제안한다. bag/시뮬레이션은 별도 profile·`clock_domain_id`를 사용하며 실제 로봇 명령으로 연결하지 않는다. replay 중 시간 되감김은 새 세션으로 처리하고 이전 lease/목표를 재사용하지 않는다.

원 관측의 최대 나이는 `수신측 현재 wall time - observed_at + 송수신 clock_error_bound 합 + 검증된 센서 stamp 지연 상한`으로 보수적으로 평가한다. 오차 상한을 알 수 없거나 `UNKNOWN` stamp이면 자동 접근의 유효 관측으로 쓰지 않는다. 표시·오프라인 기록에는 `시간 불확실`로 남길 수 있다. 유효성은 **원 관측 나이, 로컬 수신 단절 시간, 필요한 TF/센서 상태**를 모두 통과해야 한다. 어느 하나의 PASS가 다른 조건을 대신하지 않는다.

**짧은 탐지 공백과 만료는 다르다.** 대상에 의존하는 진행 중 임무는 마지막 유효 관측·기존 goal의 좌표/시간/lease·필수 센서가 계속 유효하고, 원 observed_at가 전진하는 마지막 새 유효 관측의 로컬 monotonic 수신 이후 공백이 T-01의 `SHORT_LOSS_MAX` 안일 때만 기존 goal을 유지할 수 있다. 허용 시간은 원 관측의 남은 유효시간과 SHORT_LOSS_MAX 중 먼저 끝나는 시점까지다. 새 탐지가 없다는 이유로 새 goal을 발급하거나 observed_at/valid_until을 갱신하지 않는다. hold/중복 재발행은 공백 타이머를 초기화하지 않으며, 이동 대상의 미래 위치를 임의로 외삽하지 않는다. 관측 만료·공백 상한 초과·좌표/시간/lease/필수 센서 유효성 상실 중 하나라도 발생하면 즉시 취소→정지 관측으로 전환한다. 이 조건은 watchdog을 늦추거나 만료된 관측으로 접근을 시작할 권한을 주지 않는다.

`SEARCH_AREA`는 수색 자체에 불필요한 특정 대상의 관측 만료만으로 중지하지 않는다. 해당 수색의 필수 scan/pose/인지 준비나 명령 권한이 상실되면 별도 readiness/정지 관문을 적용한다. 대상 의존 `APPROACH_TARGET`과 `REOBSERVE_TARGET`에는 위 공백/만료 계약을 적용하며, 오래된 위치 주변을 새로 수색하는 재관측은 S-03의 별도 구역 정책과 새 generation이 필요하다.

0·허용 오차보다 미래인 stamp·너무 오래된 stamp·다른 clock domain은 자동 임무 입력에서 거절한다. 시계 step/restart를 감지하면 활성 접근을 보류/정지하고 시간 기준을 다시 확인한다. 시간 임계값은 T-01~T-06을 실기 전에 고정한다.

## C-04. 좌표계·지도 revision 계약

| 좌표/식별자 | 의미 |
| --- | --- |
| `mission` | 협업에 사용할 공통 기준 frame 제안. 물리 기준점과 정렬 방법은 PR 2의 실험으로 결정 |
| `robot_a/map`, `robot_a/odom`, `robot_a/base_link` | 로봇 A/B를 구분하기 위한 논리 이름 예시. 장비의 실제 frame 문자열과 대응은 프로파일로 관리하며 접두사 없는 map/odom/base_link일 수도 있음. 토픽 namespace만으로 TF frame ID가 자동 분리되지 않음 |
| `map_session_id` | SLAM 시작·리셋·기준 지도 교체 등 새 좌표 세션의 ID |
| `map_revision` | loop closure 등 저장 관측의 지도 좌표 해석을 바꾸는 보정 세대. 매 센서/TF 업데이트 번호와 구분 |
| `map_update_sequence` | 점유 지도 내용 업데이트 순서. 이것만으로 공통 좌표 정렬 품질을 판정하지 않음 |
| `transform_revision` | mission↔로봇 map 정렬의 세대. authority와 유효 조건을 함께 기록 |

접두사 없는 native frame을 사용할 때는 로봇별 ROS domain 또는 context·토픽 remap·전용 TF buffer로 수신을 격리하고 `source_robot + native_frame`으로 해석한다. 단일 그래프에서 두 TF tree를 합치는 경우에는 충돌 없는 실제 frame ID/변환 어댑터를 검증해야 한다. 공유 TF buffer에 같은 `map/odom/base_link`를 그대로 합치지 않는다.

각 로봇의 SLAM은 자신의 `map→odom`, odometry는 `odom→base_link`, 고정 센서 TF는 각 장비의 정해진 발행자가 소유한다. `rescue_mapping`은 필요한 `mission→robot_i/map` 정렬의 단일 authority를 관리한다. TF 다중 부모·동일 edge의 중복 발행자를 허용하지 않는다. 구체적인 publisher 배치는 PR 2에서 제안하고 실제 그래프 검증 후 확정한다.

revision 생성은 실제 선택한 SLAM backend의 보정 이벤트·pose graph/세션 연계와 연결되어야 한다. 일반적인 map→odom TF 변화만 보고 loop closure가 발생했다고 추정하지 않는다. 현재 설치 버전에서 보정 발생을 감지하고 영향 관측을 판별할 수 있는지 확인하지 못하면 alignment의 자동 협업 접근 자격은 INVALID로 둔다. 독립된 기준으로 좌표를 재검증/재관측하여 유효성을 확보하기 전에는 A의 과거 좌표로 B를 자동 접근시키지 않는다. 실험실에서 조작한 revision 번호만 통과한 시험은 backend 연동 시험을 대신하지 않는다.

`FrameAlignment`는 `source_frame`, `target_frame`, `map_session_id`, `map_revision`, `transform_revision`, 변환, 위치/회전 불확실성, `valid_from`, `valid_until`, `status={VALID,DEGRADED,INVALID}`를 가진다. TF 메시지만으로 revision/불확실성을 표현할 수 없으므로 해당 메타데이터를 함께 검증한다.

관측의 원 sensor frame·원 stamp·sensor 좌표와 그때 사용한 보정/변환 참조는 불변 보존한다. mission 좌표는 파생 결과이며 원본을 덮어쓰지 않는다. 원 관측 시각의 올바른 TF가 없을 때 최신 TF로 조용히 대체하지 않는다.

좌표 해석에 영향을 주는 revision 변경 시: 새 접근 배정 중단 → 영향받은 goal 취소/정지 처리 → 원 관측을 새 기준으로 재계산할 수 있는지 판정 → 가능하면 새 파생 위치, 불가능하면 `NEEDS_REOBSERVATION` → 새 generation으로 재배정한다. loop closure가 항상 단일 강체 변환으로 수정 가능하다고 가정하지 않는다. 변환 갱신과 새 목표 수락 사이의 경합은 executor가 수락 시점과 실제 goal 발행 직전에 revision을 다시 확인해 거절한다.

`Projection.status`의 공통 어휘는 `VALID`, `STALE_DERIVATION`, `NEEDS_REOBSERVATION`, `EXPIRED`다. 유효한 map→mission 정렬만 변경되어 같은 원 관측·sensor-time TF로 재계산할 수 있는 파생 위치는 `STALE_DERIVATION`이다. map session/map revision 변경 또는 정렬 무효화로 원 관측의 새 위치를 증명할 수 없으면 `NEEDS_REOBSERVATION`이다. 상태 확인에서 VERSION_MISMATCH를 감지한 경우에도 같은 기준을 적용한다. STALE_DERIVATION 뒤 지도 보정·세션 교체·무효화가 발생하면 NEEDS_REOBSERVATION으로 승격한다. 원 관측/파생 위치 유효기한이 끝나면 EXPIRED이며 중계가 기한을 연장하지 않는다. VALID 이외의 상태는 기존 goal의 유효 근거가 아니며, 새 파생 위치 또는 새 관측 확인 후 별도 generation으로 재배정한다. VALID만으로 실제 backend 검증/자동 접근 자격을 승인하지 않는다.


## C-05. 관측·사건·보고의 논리 스키마

실제 ROS 정의 파일 이름·배열 상한·직렬화 형식은 아래 의미를 보존하며 구현 PR에서 고정한다.

| 객체 | 필수 내용 | 의미/거절 조건 |
| --- | --- | --- |
| `TargetObservation` | 공통 envelope, observation_id, source_robot/camera, observed_at·stamp_semantics·clock 정보, class·confidence·tracking_mode·track_id, model_id/hash, sensor frame·bbox, 위치 유효성·sensor 위치/공분산, depth 유효성, map/transform 참조(변환한 경우), evidence_ref | 탐지와 위치 성공을 분리. 위치가 없으면 발견 기록은 가능하지만 자동 접근점 입력으로 사용 불가. class confidence를 위치 정확도나 부상 확률로 해석하지 않음 |
| `TargetEstimate` | target_id, 관련 observation_id 목록, estimate_revision, 원 최근 관측 시각/유효시한, mission 위치·공분산, **관측별** source robot·map_session_id/map_revision·transform_revision·보정 참조 목록, 확인 단계·접근 상태·보고 상태 | A/B 관측을 합쳐도 원본별 provenance를 공통 revision 하나로 덮어쓰지 않음. 두 카메라의 track ID만 같다고 병합하지 않으며 동일성 불확실하면 별개 사건/후보 관계로 보존 |
| `RobotStatus` | robot_id, boot·sequence, 로봇별 pose/frame/stamp/품질, 활성 task/generation, readiness, motion_state, stop 사유/증거, 센서·TF·시간·Nav2·명령 lease 상태, 도킹 상태/관측 시각·준비 action 상태(해당 플랫폼), 배터리 상태 | 상대 pose를 같은 mission frame으로 변환하지 않고 거리 비교 금지. 알 수 없는 값은 UNKNOWN. retained snapshot은 운영 표시용이며 live readiness 증거가 아님 |
| `RescueReport` | report_id, target_id, 관측·확인·접근 결과, 수행 로봇/task/generation, 위치/frame/revision, observed_at·confirmed_at·reported_at, 접근 불가 등 사유, evidence_ref, 스키마·모델·배포 버전 | 사람 이송/부상/생존을 확정하지 않음. 오래된 사진을 새 재확인 사진으로 대체하지 않음 |
| `ReportReceipt` | receipt_id, 참조 report_id, 수신 상태, 영속 저장 시각·저장 revision 또는 실패 코드 | `RECEIVED`와 `PERSISTED`를 구분. 보고 완료는 현재 정책에 필요한 payload와 증거가 영속 저장되고 해당 receipt가 돌아온 상태 |
| `SiteEvent` | event_id, camera_id(`cam_<id>`), kind(`DISASTER_SIGN`/`EXIT_PASS`), observed_at_ns + C-03 시간 기준(clock_domain_id·stamp_semantics·clock_state·clock_error_bound_ns), evidence_id/sha256(생산자가 발급, operations 는 bytes 를 그 ID 로 저장·해시 검증), confidence(규칙 기반 어댑터는 1.0 = "규칙 충족", 이진; 소비자는 confidence 로 2차 판정하지 않고 측정값은 detail 에서 감사), detail | 고정 카메라의 증거 사건. 로봇 관측(`TargetObservation`)과 별개이며 `source_robot` 에 카메라를 넣지 않는다. `EXIT_PASS` 의 사람 식별·중복 집계 금지는 Q-13 결정 뒤 필드 추가. `time_basis_declared` 는 세 시간 메타데이터가 **선언됐다**는 사전 조건일 뿐이다 — True 만으로 다른 관측과의 순서·최신성·구출 완료를 승인할 수 없고, 소비자가 양측 `clock_domain_id` 일치, 송수신 오차 합의 정책 상한, 미래 시각·나이 한도를 따로 검사한다(C-03; `rescue_perception.validate_freshness` 와 같은 규칙). False 면 기록만 되고 판정 입력이 아니다. 실행 가능한 Python 정의는 `rescue_contracts.SiteEvent` |

`tracking_mode={DETECTION_ONLY,TRACKING,TRACKING_FALLBACK}`는 필수이며 관측 payload hash에 포함한다. TRACKING은 비어 있지 않은 track ID가 필요하고 다른 두 모드는 track_id=None이다. tracking fallback을 정상 tracking이나 단순 ID 누락과 구별한다.

`evidence_ref`는 소유 API의 불변 ID·해시·매체 유형·관측 ID·획득 시각이다. 다른 PC의 임의 로컬 파일 경로를 공유 가능한 URL처럼 사용하지 않는다. 사진 원본의 전송/저장은 operations API에서 처리하며 DDS에 대형 사진을 반복 포함하지 않는 안을 제안한다. 사진을 필수로 할지와 보관 범위는 [scope.md의 Q-07](../requirements/scope.md)에 두며 결정 시점은 계획 PR 7·9다. 이 계약은 `PENDING_EVIDENCE` 상태를 정의하고 사진 필수 여부를 결정하지 않는다. 필수 증거 업로드가 실패하면 보고를 `PENDING_EVIDENCE`로 남기며 완료로 표시하지 않는다.

## C-06. 임무 action·lease·정지

`ExecuteTask` action endpoint 제안: `/rescue/robots/{robot_id}/execute_task`. 이 주소는 C-08의 그래프 A 가설에서 직접 연결하며, 그래프 B에서는 도메인별 어댑터/API가 동일 의미를 전달한다.

| 구분 | 내용 |
| --- | --- |
| Goal | command_id, task_id, generation, robot_id, coordinator_id/authority_epoch, kind, issued_at/deadline, command lease 참조, target_id/estimate_revision 또는 search_area_ref, 필요한 frame/session/revision, 정책/설정 버전 |
| kind | `SEARCH_AREA`, `REOBSERVE_TARGET`, `APPROACH_TARGET`, `HOLD`. 협업의 자동 귀환·도킹·물품 전달은 기본 범위에 추가하지 않음. 준비용 UNDOCK/DOCK는 아래 별도 경로 |
| 수락 조건 | ID/버전/시각/권한·generation 일치, 필요한 센서·시간·TF·Nav2 준비, stop latch 없음, lease 유효, 필수 설정 확정, 현재 지도 revision 호환. 도킹 장치가 있는 플랫폼의 수색/접근/재관측 이동은 fresh dock_status의 is_docked=false 필요 |
| Feedback | task/generation, 실행 상태, 현재 목표/경로 상태, 마지막 관측 나이, 장애/보류 사유, 실제 pose의 시각/품질. action feedback 수신 자체를 성공으로 보지 않음 |
| Result | 수락한 action의 outcome은 `COMPLETED`, `CANCELED`, `EXPIRED`, `UNREACHABLE`, `FAILED` 중 하나. terminal_reason, stop_confirmed, motion_state, 사용한 관측/좌표 revision, stop 증거, 보고 참조를 포함. 결과가 없으면 UNKNOWN/PENDING 유지 |

도킹 장치가 있는 플랫폼의 준비/운영 `UNDOCK`·`DOCK`는 협업 `ExecuteTask.kind`와 분리한 기동 절차 또는 운영자 요청으로만 실행한다. `rescue_bringup`/operations는 실행기에 요청하고 실제 장치 action 발행은 `rescue_robot` 한 곳이 소유한다. 이 경로도 command ID·권한·deadline·배타적 명령 소유·로컬 정지 경로를 요구한다. 센서가 도킹 중 꺼지는 장비의 준비 교착을 피하도록 UNDOCK는 완전한 Nav2/카메라 READY를 선행 조건으로 삼지 않고, fresh 도킹/driver 상태·실제 정지·장치 action 가용성·유효한 준비 제어권·fault/stop 해제 조건을 따로 검사한다. 이 준비 제어권은 협업 ARMED를 뜻하지 않으며 Nav2와 동시에 움직일 수 없다.

도킹 action의 접수/성공 응답만으로 물리 상태를 확정하지 않는다. UNDOCK 뒤 fresh `is_docked=false`·정지와 필요한 센서/TF/위치 추정 준비를 확인한 뒤 협업 이동을 허용한다. DOCK 뒤에도 fresh 도킹 상태·정지를 확인한다. 접수 뒤 무동작·상태 불명·timeout은 실패/NOT_READY와 S-04 정지 관문으로 처리한다. `power_saver: False`로 도킹 중 센서가 나오더라도 수색/접근/재관측 이동의 `is_docked=false` guard는 생략하지 않는다. 도킹 장치가 없는 플랫폼의 적용 제외는 장비 프로파일과 근거로 명시한다.

Nav2 terminal status `SUCCEEDED`/`CANCELED`/`ABORTED`는 별도 `action_status`로 보존한다. 외부 취소/실패와 실행기가 먼저 요청한 정지 원인을 섞지 않으며 `original_cause`와 canonical `terminal_reason`/qualified `detail_code`를 남긴다. action 성공 뒤에도 정지 증거가 필요하다.

action 수락 전 거절은 goal 응답의 `accepted=false`와 별도의 `TaskAdmission`/상태 기록에 command_id/task_id/robot_id/robot_boot_id/authority_epoch/generation 및 canonical 거절 이유를 남긴다. coordinator의 발급 세대와 실행기의 최고 수락 세대를 구분하며 예약 생성만으로 수락 완료를 기록하지 않는다. 도메인 로그의 `REJECTED`는 action Result enum이 아니다. 정지 확인이 실패한 수락 임무는 `outcome=FAILED`, `terminal_reason=STOP_UNCONFIRMED`, `stop_confirmed=false`, `motion_state=STOP_UNCONFIRMED`로 보고하고 최초 취소/만료/실패 원인도 별도 기록한다. 결과 전송 후에도 정지 latch와 감시를 유지하며 새 이동을 허용하지 않는다.

실행기는 로봇별 활성 이동 task를 하나만 소유한다. 새 임무는 기존 임무 종료·정지 관문을 거친 뒤 시작한다. 동일 command_id의 재전송은 이전 수락/결과를 돌려주며 새 goal을 만들지 않는다. 낮은 generation·이전 boot/authority의 메시지·완료한 goal의 늦은 feedback/result는 현재 상태를 덮어쓰지 않는다.

generation은 coordinator가 로봇별 발급하며 발급 장부와 수락 장부를 분리한다. 예약은 PENDING이고, command_id/task_id/robot_id/robot_boot_id/authority_epoch/generation이 정확히 일치하는 실행기 TaskAdmission의 accepted=true가 유일한 수락 근거다. 실행기는 최고 수락 세대를 검증하고 coordinator는 해당 응답을 받은 뒤에만 ACTIVE와 accepted generation을 갱신한다. 거절/해제된 명령의 지연·충돌·재전송으로 재활성화하지 않는다. 재시작 뒤 발급 이력·실행기 이력·authority를 조정하기 전에는 자율 임무를 재개하지 않는다. 예약의 `candidate`(planner 경로 증거)는 `APPROACH_TARGET`·`REOBSERVE_TARGET` 에만 필수이고 `SEARCH_AREA`·`HOLD` 는 None 이다. `HOLD` 는 제자리 정지 유지 임무이며 프로파일 `hold_positions` 로 이동시키는 명령이 아니다(대기 위치로의 이동은 별도 kind 가 필요하며 미정, Q-14).

coordinator는 단일 인스턴스를 전제로 하고 자동 leader election/failover는 이번 기본 범위에 넣지 않는다. 재기동하면 새 `authority_epoch`가 되며 이전 lease/배정을 복구된 것으로 자동 인정하지 않는다. 로봇 실행기는 정지·기존 권한 폐기 확인 후 명시적인 재등록으로 새 authority를 허용한다. UUID 문자열 크기 비교로 권한의 신구를 판단하지 않는다.

command lease는 로봇별 현재 authority/generation에 묶고 유효기간·최대 갱신 간격·로컬 monotonic 만료를 가진다. 중복 또는 늦은 갱신으로 만료 lease를 부활시키지 않는다. coordinator lease, executor heartbeat, 최종 속도 입력이 각각 만료될 때의 중단 책임을 구분한다. 새 연결이나 heartbeat만으로 정지 latch가 해제되지 않는다.

`RequestStop` service 제안: `/rescue/robots/{robot_id}/request_stop`; 입력은 stop_command_id, requester/authority, reason, scope, issued_at이다. 응답 `ACCEPTED`는 정지 요청 접수만 뜻한다. 로봇 실행기는 활성 goal 취소와 최종 명령 억제를 요청하고 별도 `RobotStatus`로 `STOPPED` 또는 `STOP_UNCONFIRMED`를 보고한다. 정지 요청은 새 임무·추론 완료·지도 계산보다 우선 처리되어야 한다.

`STOPPED`는 fresh odometry의 속도가 사전 기준 이하로 일정 관측 구간 유지되고 최종 명령 경로가 억제된 것을 확인한 상태를 제안한다. 해당 값·시간·관측 방식은 실기 전 고정하고 실제 주행 시험으로 검증한다. cancel 응답이나 0 속도 한 번 발행은 이 증거를 대신하지 않는다. odometry/driver 상태도 끊기면 `STOP_UNCONFIRMED`다. 이 상태를 물리적 안전 인증으로 표현하지 않는다.

정지 latch의 해제는 유효한 권한의 명시적 `Resume/Reset` 요청, 원인 해결·readiness·새 generation을 모두 요구한다. 비상 정지/장비 fault의 해제 조건은 장비 매뉴얼·실물 확인 후 따로 고정한다. 웹 요청이 실패했다고 정지가 성공했다고 표시하지 않으며, 로봇 현장 정지 수단을 웹 연결에만 의존시키지 않는다.

## C-07. 안전 접근·통로 자원

대상의 위치와 Nav2 접근 목표를 별도 값으로 저장한다. 목표는 대상 보호 영역·위치 불확실성·로봇 footprint·제동/제어 오차를 고려해 자유 공간의 후보에서 선택하고 경로 가능성을 확인한다. 대상 좌표 자체나 로봇-대상 직선 위 한 점만으로 접근 성공을 정하지 않는다. Nav2 `SUCCEEDED` 이후에도 재확인·거리/정지·좌표 유효성 관문을 통과해야 `APPROACH_COMPLETE`다.

`D_MIN`, `D_MAX`, 보호 영역 정의, 위치 오차 상한, 재확인 최대 나이, 후보 검색 범위·재계획/retry budget은 시험 전에 고정한다. 사람/인형 형상·가림 등으로 이 조건을 평가할 수 없으면 `LOCATION_UNCERTAIN`/`NEEDS_REOBSERVATION`으로 정지·보고한다. 가까워지기 위한 후진/회전도 공통 명령·장애물 경로를 통과한다.

HELD 또는 holder를 보존한 OCCUPANCY_UNKNOWN에서 `confirm_vacant(holder=...)`는 저장된 robot/task/generation/authority와 정확히 일치해야 한다. 제3자나 이전 세대는 점유를 해제할 수 없다. holder가 없는 초기 UNKNOWN에서만 holder=None과 fresh verified vacancy를 함께 받을 수 있다.

좁은 통로를 공유 자원으로 채택하면 `resource_id`, holder robot/task/generation, revision, lease, 진입·이탈 확인을 계약화한다. lease 만료는 점유 로봇이 물리적으로 빠져나갔다는 증거가 아니다. holder 위치·이탈이 불확실하면 `OCCUPANCY_UNKNOWN`으로 새 진입을 보류한다. 다른 로봇의 stale pose만으로 통로가 비었다고 판단하지 않는다. 구체적인 자원 모델/경로 교착 정책은 PR 2 선택과 PR 8 구현에서 검증한다.

## C-08. 토픽·QoS·API 제안

**그래프 A 가설:** 아래 전역 토픽/action을 직접 연결하는 안은 두 로봇·네 PC가 단일 ROS 그래프를 공유한다는 전제다. PR 2의 A0/A1 실험에서 domain·Discovery Server 목록/고유 ID·참여자 수·호스트별 수신·명령 분리가 확인되기 전에는 배포 확정이 아니다. 원본 P4의 DEC-010도 A는 제안이고 B는 도메인 분리 대안이었다. 원본 프로젝트의 B 기본값 결정을 새 프로젝트의 사용자 확정 결정으로 승계하지 않는다.

**그래프 B 대체 경로:** 로봇별 domain 안에 독립 수신/실행 프로세스를 두고, coordination/operations는 도메인별 어댑터와 API로 관측·상태·임무 요청·정지·보고를 중계한다. Nav2/도킹 action은 해당 로봇 domain의 실행기가 호출한다. 범용 domain bridge가 action까지 투명하게 중계한다고 가정하지 않는다. API여도 원 source/관측 시각·ID·generation·좌표/session/revision·권한·deadline·정지 증거·영속 receipt 의미를 보존하고, 중계 단절도 lease/watchdog 대상이다. 한 프로세스의 다중 ROS context는 기본 구현으로 확정하지 않는다. 두 안 중 선택과 실제 endpoint/라우팅은 프로파일로 고정한다.

아래 depth 값은 큐 설정 제안이며 지연·누락·재시작 시험으로 고정한다. DDS reliability/durability를 DB 저장·명령 완료 보장으로 해석하지 않는다.

| 데이터 | 토픽/endpoint 제안 | QoS/전달 방식 제안 |
| --- | --- | --- |
| 로봇 native RGB/Depth/scan/odom | `/{robot_id}/...`의 상대 sensor topic | 실제 드라이버 QoS와 호환. 영상/센서는 BEST_EFFORT, KEEP_LAST 1 우선 검토; 처리 지연 시 최신 입력 유지. 정합·누락은 별도 계측 |
| 관측 | `/rescue/observations` | BEST_EFFORT, KEEP_LAST 10, VOLATILE; 관측 중복 제거·신선도 검증. 중요한 사건 확인/보고는 아래 reliable 경로 사용 |
| live heartbeat/readiness | `/rescue/robots/{robot_id}/heartbeat` | RELIABLE, KEEP_LAST 1, VOLATILE; 어플리케이션 watchdog. DDS deadline 이벤트만으로 물리 정지를 주장하지 않음 |
| 상태 snapshot | `/rescue/robots/{robot_id}/status`, `/rescue/targets` | RELIABLE, KEEP_LAST 1, TRANSIENT_LOCAL; envelope/stamp로 오래된 snapshot 식별. targets는 전체 snapshot인지 delta인지 혼합하지 않고 전체 snapshot을 제안 |
| 좌표 정렬 상태 | `/rescue/frame_alignments` | RELIABLE, KEEP_LAST 1, TRANSIENT_LOCAL; 현재 전체 alignment snapshot과 revision/validity. TF는 표준 전송을 사용하며 frame ID와 authority를 검사 |
| 임무·정지 | ExecuteTask action, RequestStop service | RELIABLE, VOLATILE 제안. 재접속 후 오래된 명령을 retained 값으로 재실행하지 않음. timeout/retry/idempotency는 응용 계약 |
| 보고 | `/rescue/reports`, `/rescue/report_receipts` 또는 동등 API | RELIABLE, VOLATILE + 송신측 보류 저장/동일 ID 재전송 + 수신측 중복 제거/영속 receipt. endpoint 최종 선택은 PR 2의 operations API와 일치시킴 |
| 지도/영상 관제 | operations API의 snapshot/stream | frame/session/revision·생성/관측 시각·유효 상태 포함. 지도·메타데이터를 같은 snapshot ID로 전달. PC간 DB 파일 공유 대신 API 책임 분리 |

Discovery 진단에는 `ROS_SUPER_CLIENT` 설정 여부·서버 목록/ID·프로세스 및 DDS participant 수·서버 CPU를 기록한다. 목록 조회용 설정과 일반 runtime의 필수 조건을 혼동하지 않으며, 토픽 목록만 보이는 것을 통신 PASS로 쓰지 않는다. 실제 메시지 수신 시각·양방향 요청/응답과 장애 복구를 시험한다.

관측은 누락될 수 있는 센서 흐름이다. 첫 발견·확인된 사건·보고의 보존은 별도 reliable/영속 경로에서 처리한다. 전송 보장과 탐지 성공률을 혼동하지 않는다. HTTP를 선택하는 경우에도 동일 command/report ID와 상태 단계를 유지하며 UI는 요청 중·수락·실행·저장·실패를 구분한다.

## C-09. 오류 코드와 복구 원칙

공통 Python 오류의 단일 사전은 [rescue_contracts](../../src/rescue_contracts/rescue_contracts/__init__.py)의 `CANONICAL_CODES`와 `DETAIL_CATEGORIES`다. 모든 도메인 예외는 `RescueError`를 상속하고 `.code`에는 공통 코드, `.component`에는 발생 패키지, `.detail_code`에는 `rescue_mapping.TF_TIME_MISMATCH` 같은 qualified 세부 원인, `.detail`에는 설명을 둔다. 소비자는 세부 문자열 대신 `.code`로 복구 분기한다. 예외를 발생시키지 않는 실패 결과도 같은 canonical code/detail_code 의미를 노출한다. 성공/상태 enum (`ACCEPTED`, `NAVIGATION_SUCCEEDED`, `HOLD`, `VALID`, `NEEDS_REOBSERVATION` 등)을 오류 코드와 혼합하지 않는다.

공통 코드: `INVALID_INPUT`, `INCOMPATIBLE_SCHEMA`, `WRONG_ROBOT`, `STALE_OBSERVATION`, `CLOCK_UNCERTAIN`, `STALE_TRANSFORM`, `FRAME_REVISION_MISMATCH`, `LOCALIZATION_UNCERTAIN`, `SENSOR_UNAVAILABLE`, `NAV_NOT_READY`, `NOT_AUTHORIZED`, `STALE_GENERATION`, `LEASE_EXPIRED`, `STOP_LATCHED`, `UNREACHABLE`, `RETRY_EXHAUSTED`, `REPORT_NOT_PERSISTED`, `STOP_UNCONFIRMED`, `ID_CONTENT_CONFLICT`, `TASK_BUSY`, `DEADLINE_EXPIRED`, `ACCEPT_TIMEOUT`, `GUARD_INVALID`, `STOP_EVIDENCE_STALE`, `SESSION_FAULT`, `GOAL_REJECTED`, `ACTION_CANCELED`, `ACTION_ABORTED`, `ROBOT_DOCKED`, `DRIVER_UNAVAILABLE`, `COMMAND_WATCHDOG_UNAVAILABLE`, `POLICY_MISMATCH`, `NOT_FOUND`, `PENDING_EVIDENCE`.

`NEEDS_REOBSERVATION`가 예외의 세부 원인일 때 `.code=FRAME_REVISION_MISMATCH`이며 상태 표기의 이름은 그대로 유지한다. 알려지지 않은 진단 코드를 사전에 등록하지 않고 성공으로 처리하지 않는다. E2 코어는 객체·시각·task 정보가 담긴 호출 입력/반환값을 제공하며, 전체 런타임 로그 envelope와 재시도 정책은 ROS/API 어댑터에서 결합해야 한다.

모든 오류에는 원인 컴포넌트·객체 ID·발생 시각·활성 task/generation·재시도 가능 여부를 기록한다. 원인을 모르면 UNKNOWN을 유지하고 성공으로 추정하지 않는다. sensor/time/frame 실패와 임무 정책 실패를 같은 무한 재시도로 숨기지 않는다. 재시도 budget·전체 deadline·정지 사유는 상태 전이와 함께 시험한다.

## 고정 원본에서 가져온 계약 근거

- 관측 재발행·수신 시각 혼동: [AMR webcam publish](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/webcam_detector.py#L109-L159), [follower 수신](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L290-L304).
- 대상 자체 goal·만료 후 기존 goal 유지: [go_near](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L388-L446), [approach](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L517-L528).
- AMCL 준비·직접 lifecycle 복구: [기동](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L109-L123), [복구](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L263-L288).
- 원 관측 시각 TF의 재사용 후보: [P4 target_locator](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/target_locator.py#L74-L107). stamp의 유효성/지도 보정 처리까지 검증 완료됐다는 뜻은 아니다.
- 도메인 A/B는 [P4 DEC-010](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/decisions.md#L142-L151)의 당시 후보/기록이며 새 배포 선택이 아니다. native frame의 접두사 없는 예시는 [adapters.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/adapters.py#L20), [follow_nav.launch.py](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/rokey_mini/launch/follow_nav.launch.py#L49-L60)에서 확인된다.
- 도킹·센서의 조건부 관계는 [로봇 3번 baseline](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/reference/2026-10-01-lab-environment-baseline.md#L40-L63), 운영자 전용 준비 action은 [Task.msg](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_interfaces/msg/Task.msg#L15-L17)에 있다. 접수 뒤 무동작은 [실습 기록](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L26-L27)의 관찰이며 [원인은 미확인으로 기록됐다](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L66).
- 늦은 action 수락·취소 관리 후보: [P4 goal_timer](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/goal_timer.py#L83-L173).

원본 링크는 근거일 뿐 이 초안 계약의 기존 구현이 아니다. 원본 접근이 안 되면 확인하지 못한 근거를 명시하고, 출처 검토가 끝났다고 처리하지 않는다.

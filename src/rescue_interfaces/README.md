# rescue_interfaces

SDD 검토안 v0.1 중 1차로 채택한 ROS 메시지 19·서비스 4·액션 2의 정의만 둔다. 판단 로직은 없다. 채택 범위와 IDL 규칙은 [contracts C-10](../../docs/design/contracts.md)에 있다.

- 필드는 [`sdd-node-contracts-v0.1.json`](../../docs/design/sdd-node-contracts-v0.1.json)이 원본이다. 필드를 바꿀 때는 JSON 을 먼저 고친다. [`tests/test_interfaces_schema.py`](../../tests/test_interfaces_schema.py)가 이 폴더의 IDL 과 JSON·`CMakeLists.txt` 목록이 같은지 ROS 없이 검사한다.
- ROS 메시지와 Python 값(`rescue_contracts`·각 패키지 dataclass)의 변환은 각 패키지의 ROS 경계 모듈이 한다.
- 빌드: `colcon build --packages-select rescue_interfaces` (ROS 2 Jazzy). CI 에는 ROS 가 없어 이 빌드는 돌지 않는다.

## 통신별 입력과 출력

[SDD 명세](../../docs/design/sdd-node-interface-spec-v0.1.md)「토픽·서비스·액션 연결 정의」를 흐름별로 줄인 표다. 연결의 원본(endpoint·발행자·구독자·QoS)은 그 명세이고, 여기와 다르면 명세가 이긴다. `{r}` 은 로봇(`amr1`/`amr2`, 실제 이름은 배포 프로파일). `1차` 는 이 패키지에 정의가 있음, `2차` 는 아직 없음([C-10](../../docs/design/contracts.md)). 정의가 있다는 것이지 그 통신을 하는 노드가 구현됐다는 뜻은 아니다(2026-10-10 기준 노드 연결 0개).

### PC4 → 로봇 PC: 명령과 권한 (`coordinator` → `{r}.executor`)

| 통신 | 종류 | 입력 | 출력 | IDL |
| --- | --- | --- | --- | --- |
| `/rescue/robots/{r}/execute_task` | Action `ExecuteTask` | Goal: command_id, task_id, kind(`SEARCH_AREA`·`APPROACH_TARGET`·`REOBSERVE_TARGET`·`HOLD`·`MOVE_SEGMENT`·`RETURN_HOME`), generation, authority_epoch, lease_id, 마감 시각, target·search_area·home_ref, frame_ref, route, approval | Feedback: 단계(VALIDATING·PLANNING·NAVIGATING·VERIFYING·STOPPING), 구간 번호, 현재 pose, 남은 거리 / Result: `TaskResult`(outcome, completion_scope, stop_confirmed, final_pose) | 1차 |
| `/rescue/robots/{r}/command_lease` | Topic `CommandLease` | lease_id, generation, 갱신 번호, 만료 시각 | 없음. 갱신이 끊기면 실행기가 정지 관문으로 감 | 1차 |
| `/rescue/robots/{r}/register_authority` | Service `RegisterAuthority` | 새 authority 등록(정지 상태에서만) | 수락 여부, reason_code. 등록만으로 움직이지 않음 | 1차 |
| `/rescue/robots/{r}/request_stop` | Service `RequestStop` (operations 도 호출) | 범위(ACTIVE_TASK/ALL_MOTION), 이유 | 접수 상태(ACCEPTED/REJECTED/DUPLICATE). 실제 정지는 `status` 로 확인 | 1차 |
| `/rescue/robots/{r}/reset_stop` | Service `ResetStop` | 원인 해결 뒤 정지 잠금 해제 요청 | 수락 여부, reason_code | 1차 |
| `/rescue/robots/{r}/evaluate_route` | Action `EvaluateRoute` | 대상, 구간 | 주행 없이 계산한 승인용 `RouteCandidate` | 2차 |

### 로봇 PC → PC4: 상태 보고 (→ `coordinator`·`operations`)

| 통신 | 종류 | 보내는 쪽 | 내용 | IDL |
| --- | --- | --- | --- | --- |
| `/rescue/robots/{r}/status` | Topic `RobotStatus` | `{r}.executor` | 준비 상태, motion(STOPPED·MOVING·STOPPING·STOP_UNCONFIRMED), 현재 task, 구간 번호, 도킹, 배터리 | 1차 |
| `/rescue/robots/{r}/heartbeat` | Topic `Heartbeat` | `{r}.executor` | 생존 신호(boot_id, generation, command_fenced) | 1차 |
| `/rescue/robots/{r}/task_admission` | Topic `TaskAdmission` | `{r}.executor` | task 수락·거절과 이유. action Result 와 다른 신호 | 1차 |
| `/rescue/robots/{r}/pose` | Topic `RobotPose` | `{r}.mapping` | 로봇 지도 pose + 공통 좌표 pose + 시각 | 1차 |
| `/rescue/robots/{r}/map_state` | Topic `MapState` | `{r}.mapping` | 지도 session·revision·위치 추정 품질 | 1차 |

### 로봇 PC 안: 인식 → 지도 → PC4

| 통신 | 종류 | 보내는 쪽 → 받는 쪽 | 입력 → 출력 | IDL |
| --- | --- | --- | --- | --- |
| `/rescue/observations` | Topic `TargetObservation` | `{r}.perception` → `{r}.mapping` | 인형 탐지: bbox, 신뢰도, 모델 해시, 센서 기준 위치, 깊이 상태 | 1차 |
| `/rescue/target_locations` | Topic `TargetLocation` | `{r}.mapping` → `coordinator` | 공통 좌표 위치 + 지도 버전. 저장 확인까지 재전송 | 1차 |
| `/rescue/robots/{r}/observe_target` | Action `ObserveTarget` | `{r}.executor` → `{r}.perception` | Goal: 대상, 볼 지점, 관측 시간 창 / Result: `RecheckObservation`(카메라 PRESENT·ABSENT·UNKNOWN, LiDAR, 시야 유효성) | 1차 |
| `/rescue/robots/{r}/evaluate_passage` | Action `EvaluatePassage` | `{r}.executor` → `{r}.passage` | 구간, 로봇 크기 → PASSABLE·BLOCKED·UNKNOWN (Q-25) | 2차 |

### 웹캠(PC3) → PC4

| 통신 | 종류 | 내용 | IDL |
| --- | --- | --- | --- |
| `/rescue/site_events` | Topic `SiteEvent` | 지진 징후(`DISASTER_SIGN`) 또는 인형 소실 후보(`TARGET_DISAPPEARED_CANDIDATE`) + 근거 | 1차 |
| `/rescue/cameras/cam_fixed/presence` | Topic `TargetPresence` | 감시 영역별 PRESENT·ABSENT_CANDIDATE·UNKNOWN | 2차 |
| `/rescue/cameras/cam_fixed/status` | Topic `CameraStatus` | 영상 끊김·품질 | 2차 |
| `/rescue/cameras/cam_fixed/set_watch` | Service `SetCameraWatch` | `coordinator` 가 감시 영역 지정 → 수락 여부 | 2차 |

### PC4 안: `operations` ↔ `coordinator`

| 통신 | 종류 | 입력 | 출력 | IDL |
| --- | --- | --- | --- | --- |
| `/rescue/operator_command` | Service `SubmitOperatorCommand` (`operations` → `coordinator`) | operation(`DISPATCH_SUPPORT`·`CONTINUE_SEGMENT`·`REQUEST_RECHECK`·`RETURN_HOME`·`FINISH_MISSION`·`RESET_STOP`·`SET_CAMERA_WATCH`), 승인자, 로봇, 대상, 예상 task·generation, 경로·구간 번호 | 접수 여부, created_task_id, approval_id. 접수일 뿐 이동 완료가 아님 | 1차 |
| `/rescue/report_receipts` | Topic `ReportReceipt` (`operations` → 보고를 보낸 모든 노드) | DB 저장 뒤 PERSISTED | 받은 쪽이 재전송을 멈춤 | 1차 |
| `/rescue/reports` | Topic `RescueReport` (`coordinator` → `operations`) | 구조 보고 묶음 | | 2차 |
| `/rescue/targets`, `/rescue/system_state` | Topic `TargetSnapshot`·`SystemState` | 대상 전체 상태, 시스템 모드 | | 2차 |
| `/rescue/frame_alignments` | Topic `FrameAlignmentSnapshot` | 두 로봇 지도 정렬(Q-03·Q-09) | | 2차 |

관제 웹 ↔ `operations` 는 ROS 가 아니라 HTTP·SSE 다(명세 「HTTP·웹 연결 정의」).

### 예: AMR2 구간 하나

```text
관제 웹 "계속" ──HTTP──▶ operations
operations ──operator_command(CONTINUE_SEGMENT)──▶ coordinator          → 접수 응답
coordinator ──execute_task(MOVE_SEGMENT, 새 generation)──▶ amr2.executor
amr2.executor ──task_admission──▶ coordinator·operations                (수락/거절)
amr2.executor ──NavigateToPose──▶ Nav2 → 로봇 이동
amr2.executor ──Feedback(구간 i, 남은 거리)──▶ coordinator
amr2.executor ──status(STOPPED)──▶ … ──Result(COMPLETED, scope=SEGMENT)──▶ coordinator
coordinator: AWAITING_APPROVAL → 다음 "계속" 대기
```

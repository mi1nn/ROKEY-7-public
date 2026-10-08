# PR 5 — 로봇 명령·정지 판정 코어

계획 PR 5의 **선행 Python 코어 구현**이다. 계획의 두 로봇 SLAM·Nav2 실제 연결/기동과 CT-08 실기 통과는 완료되지 않았다. PR 6~10을 작성할 수 있는 검증 가능한 임무 경계를 마련한다. 이 범위 변경은 실기 인수 조건을 삭제하지 않는다.

## 동작

- UUID 명령·임무·로봇 세션·authority와 uint64 generation을 검증한다. 동일 명령/내용은 같은 수락 결과, 동일 ID/다른 내용은 충돌이다. 거절도 해당 ID의 불변 결과로 유지한다. 재시도 조건이 달라지면 새 명령 ID가 필요하다.
- `arm`은 fresh 정지 증거와 준비 상태가 있어야 한다. 다른 authority는 기존 authority를 명시한 stopped 등록 절차를 거친다. UUID 크기로 신구를 판정하지 않는다.
- 수락과 발송을 분리한다. `dispatch`에서 준비 상태·외부 guards·기한·임대를 다시 검사한다. 도킹 상태나 watchdog 미준비 상태에서는 이동 의도를 만들지 않는다.
- 수락 지연·deadline/lease 만료·guard 손실은 STOPPING으로 전이한다. 만료 후 임대 갱신은 부활시키지 않는다. 늦은 수락은 그 명령 UUID의 취소 의도를 만든다.
- 정지는 **액션 종료 fence + 연속된 fresh odometry 표본의 선속도/각속도 상한 충족**으로 판정한다. 취소 요청/응답·제로 속도 명령만으로 정지 성공 처리하지 않는다. 시한 초과는 `FAILED / STOP_UNCONFIRMED`, 원래 원인, `stop_confirmed=false`를 남기고 fault latch를 유지한다.
- Nav2 성공 뒤 정지한 `COMPLETED`는 경로 실행 경계의 결과다. 대상 재관측·신원·거리 조건을 확인하는 `APPROACH_COMPLETE`는 조정 계층에서 별도로 판정해야 한다.

## 시험과 실행

```bash
python3 -I scripts/run_tests.py
python3 -I scripts/check_docs.py
```

공통 runner는 자신의 저장소 경로에서 시험을 탐색하고 하위 프로세스에도 동일 source 경로를 전달한다. 수집 0개를 오류로 처리한다. 첫 runner 실행에서 기존 CLI 하위 프로세스 3개가 import 경로를 받지 못한 실패를 발견했고, runner의 자식 환경 경로를 수정했다. 배포 검사 자체의 기대값은 바꾸지 않았다.

E2 시험은 중복/충돌·다른 로봇/boot/authority·미준비·발송 전 guard 손실·늦은 수락·임대 중복/만료·연속 정지·정지 시한 실패·이전 결과·잘못된 입력·단조 시계 역행을 확인한다. ROS·실제 액션 통신·장비측 정지는 E3/E4 미실행이다.

## 출처와 설계 변경

P4 [goal_timer](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/goal_timer.py#L83-L173)의 수락/결과/취소 시간 경계와 늦은 수락 처리 원리를 참고했다. 원본 코드 복사 없이 C06 계약에 맞춰 새로 작성했다. P4 [FSM](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/fsm.py#L770-L815)의 AMCL 초기화·도크 준비 정책은 이식하지 않았다.

AMR [대상 만료 처리](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L517-L528)는 반환만으로 기존 이동이 멈추는지 점검할 반례다. 이번 코어는 명시적인 취소와 정지 증거를 분리한다.

## 연결 전에 필요한 것

1. 실제 로봇/ROS 배포판/네임스페이스/도메인/발행자 확인, SLAM·Nav2·센서 ready 수집. 현재 `Readiness`는 어댑터가 제공해야 하는 입력일 뿐 실제 검사기가 아니다.
2. `SEND` 의도를 목표 좌표·정확한 map session/revision에 결합하는 ROS action 어댑터. 목표 wire schema, wall clock bound 검사, 원격 인증/권한과 replay 방어, 프로세스 재시작 상태 복구는 미구현이다.
3. goal terminal callback이 더 이상 속도 의도를 낼 수 없다는 실행 fence와 로봇측 정지 감시. fault latch 복구는 현재 세션 재생성 전에 운영자/어댑터가 원인 해소·정지를 증명해야 한다. 원격 reset API는 없다.
4. 언도킹은 별도 준비 액션이다. 도킹 중 카메라 절전과 Nav2 미준비를 고려한 실제 운영 절차는 미구현이다.
5. `seen`/결과 이력은 프로세스 메모리이며 크기 제한·영속 세션/세대 저장은 후속 실기 어댑터의 필수 과제다. 장시간 공개 서비스로 사용하지 않는다.

CT-06/07/08의 일부 코어 불변식만 E2 증거를 얻는다. 상위 계약 전체와 계획 PR 5의 완료는 선언하지 않는다.

## CLA-18/19/20 최종 수정

- `rescue_contracts`의 canonical error + qualified diagnostic, 정확한 int64 ns wall time과 TaskAdmission을 사용한다. 로컬 monotonic timer는 float 초이며 overflow/정밀도 소실 거절과 clock fault latch를 보존했다.
- `Executor(..., allowed_robot_ids=...)`는 프로파일에서 정확히 2개 ID를 명시적으로 받는다. Task DTO는 ID 형식만 검사하고 실행기가 대상·boot·authority·generation·policy를 확인한다.
- bounded odometry만 있고 action fence가 없는 실행 중 상태는 `MOVING`이다. 실제 정지를 요청한 STOPPING에서만 `STOP_REQUESTED`를 사용한다.
- `goal_terminal(..., terminal_status=...)`는 SUCCEEDED/CANCELED/ABORTED를 구별하고 Result.action_status와 original_cause를 보존한다. 중복 terminal은 정지 창을 초기화하지 않고 상충하는 terminal은 거절한다.
- CI의 `python -I scripts/run_tests.py`에 무효였던 PYTHONPATH 설정을 제거했다. 프로파일 CLI 단계의 환경은 유지한다.

`tests/test_executor_review.py`는 새 API와 위 문제의 회귀시험이다. 실물 driver/Nav2/odometry 및 준비용 undock는 연결하지 않았고, 모든 motion/정지 증거는 합성 E2 입력이다.

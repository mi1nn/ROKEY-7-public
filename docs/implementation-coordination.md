# PR 8 — 임무 배정·접근 검증·통로 점유 코어

계획 PR 8의 Python 정책 코어다. 실제 frontier 탐색, Nav2 path service, ROS 임무 전송은 미구현이다. `PathCandidate`는 외부 planner가 제공해야 하는 증거이며, 후보를 선택했다는 사실로 이동을 허가하지 않는다. PR 6의 실제 SLAM 보정 연동이 없어 `auto_approach_eligible`은 여전히 false다.

## 구현한 경계

| API | 입력·판정 | 결과 |
| --- | --- | --- |
| `select_candidate` | 명시적인 가용 로봇, 경로 길이, footprint/경로 free 여부, clearance, 불확실성, 만료 | 검증 입력을 만족하는 최단 경로 후보 또는 None. 대상 점 자체를 목표로 삼지 않음 |
| `Allocator.reserve/release` | task/target/robot/generation/authority, 같은 사건·로봇 중복 확인 | 로봇·사건마다 하나의 논리 배정. 정지 확인 없는 release 거절 |
| `ApproachTracker.observe/guard` | 원 관측 시각·기한, 새 UUID/증가 시각, 사건/추정 revision·신원·최신성 검증 결과 | 새 관측만 local gap을 갱신. 원 기한·짧은 유실 허용 시간·guard 중 하나라도 깨지면 CANCEL. 취소된 tracker는 새 관측으로 부활하지 않음 |
| `ApproachTracker.complete` | 경로 성공, 종료 시각 이후 재관측, fresh 신원 확인, 거리·불확실성, 정지 확인 | 모든 조건을 만족할 때만 접근 확인 true |
| `Corridor` | holder의 task/robot/generation, revision, local lease/갱신 sequence, fresh pose·실제 비움 확인 | 임대 만료·pose 손실은 OCCUPANCY_UNKNOWN. 자동 FREE 전환 없음 |

`CONTINUE_EXISTING`은 현재 목표의 짧은 유실 허용 여부이며 새 목표 발행 허가가 아니다. `ApproachTracker`는 해당 대상의 접근에만 연결한다. A의 수색을 B의 대상 만료로 중단하는 전역 guard로 사용하지 않는다.

## 계약·외부 입력 책임

C06/07, CT-06/09/10/11의 일부 E2 불변식을 구현한다. 호출자는 perception의 보수적인 시간 검증, mapping의 원본별 좌표/목표 snapshot 검증, robot executor의 stop result를 실제로 연결해야 한다. bool 입력 자체는 fresh 센서/인증·정지·경로 증명이 아니며, 이 PR은 해당 증거를 만드는 ROS 어댑터를 제공하지 않는다.

수치는 `ApproachPolicy`에 모두 명시한다. 기본 거리·속도·유실 시간을 장비값으로 확정하지 않는다. 메모리 배정/통로 상태는 재시작 복구가 없다. 통로 객체는 처음부터 UNKNOWN이다. 분산 coordinator election/lease 인증·영속 배정·분할 통로 deadlock 해결·탐색 frontier 생성·자동 역할 교대는 후속 과제다.

## 출처와 변경 이유

- P4 [배정기](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/src/rokey_amr/rokey_amr/dispatcher.py#L253-L276)는 배정 책임을 한 곳에 둔 원리만 참고한다. 도킹한 로봇만 선택하거나 도크와 구역의 직선거리로 비용을 계산하는 정책은 사용하지 않는다.
- AMR [목표 갱신·도착](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L388-L446), [만료](https://github.com/mi1nn/amr-mini-project/blob/ee355d0b5896ea58bccac73ce1a7857f62e96024/src/rokey_pjt/rokey_pjt/rc_car_follower.py#L517-L528)의 갱신 억제/유실 경험을 참고했다. 목표 자체로 접근하는 정책을 거리 구간·불확실성 조건으로 대체하고 도착을 fresh 재관측·정지와 분리했다.

원본 소스 복사 없이 새로 작성했다. 현재 입력은 외부 planner 가정이므로 실제 길찾기 성공이나 사람 신원 판별 성능을 증명하지 않는다.

## 검증

`python3 -I scripts/run_tests.py`에서 후보 보호 거리·경로/footprint/불확실성 거절, 비가용 관측 로봇 유지, 중복 배정/ID 충돌/정지 전 해제 거절, duplicate 관측의 타이머 불변, 원 기한·guard 손실·취소 후 재활성화 차단, 접근 확인, 통로 UNKNOWN과 중복 lease를 확인한다. 실기 2대 수색·접근, ROS path/action 통신, 통로 실제 충돌 회피는 E3/E4 미실행이다.

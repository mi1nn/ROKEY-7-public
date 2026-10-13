# rescue_mapping

## 현재 상태 (2026-10-14, main 4163a25)

이 패키지는 좌표 변환과 지도 버전 검사의 순수 Python 라이브러리다. 현장의 지도 흐름(TB-6 SLAM → 두 번 고정 → AMCL)은 **이 패키지에 없다.** 아래 표의 다른 패키지에 있다. 코드 기준이며, 이 패키지 코드는 2026-10-08 뒤로 바뀌지 않았다(`git log -- src/rescue_mapping`).

| 기능 | 상태 | 근거 |
| --- | --- | --- |
| 3D 강체 변환·지도→임무 정렬(`RigidTransform`·`FrameAlignment`) | 구현됨. 현장 코드가 씀 | `geometry.py`·`models.py`. 사용처: `rescue_robot/task_site_runtime.py`, `rescue_perception` `map_projection.py`·`target_location.py`·`recheck.py` 등 |
| 관측 보존·목표 snapshot 검사(`MappingRegistry`) | 구현됨. 현장 실행 경로에서는 안 씀(합성 리허설 `rescue_bringup/offline_demo.py` 만) | `registry.py`. `rescue_robot`·`rescue_coordination` 에서 import 없음 |
| `auto_approach_eligible` | 이 패키지 출력은 항상 `False`. TB-5 자동 접근은 이것이 아니라 로봇별 정책(`task.live.auto_target.*`)으로 받는다 | `registry.py`, `rescue_robot/task_site_runtime.py:target_gate` |
| 조정기 지도 방식 `SLAM_FREEZE`(기준 지도 없음) | 구현됨, 실기 NOT_RUN. 이 패키지 밖 | `rescue_coordination/ros_node.py`(`MAP_MODES`), 현장 파일 coordinator `map_mode` |
| 두 번 고정(R0+1 인형 발견, R0+2 탐색 완료)·고정 기록 | 구현됨, 실기 NOT_RUN. 이 패키지 밖 | `rescue_robot/map_freeze.py`, #242·#255 |
| 짝 토픽 `/rescue/map/frozen_revision`(고정 지도가 몇 번째 revision 인지) | 구현됨. 이 패키지 밖 | `rescue_contracts/occupancy.py:frozen_revision_text`·`parse_frozen_revision`, `docs/design/contracts.md` C-04a, #242 |
| TB-6 출발 자세 `start_pose`(미리 재는 좌표의 기준) | 구현됨, 실기 NOT_RUN. 이 패키지 밖 | `rescue_robot/map_freeze.py`(`/rescue/map/start_pose/robot_a`, `output_dir/start_pose.json`), OPS-141 |
| 로봇 쪽 고정 지도 받기·재정합 | 구현됨, 실기 NOT_RUN. 이 패키지 밖 | `rescue_robot/live_sources.py:SavedMapSources`, `frozen_map_relay.py`, [rescue_robot README](../rescue_robot/README.md) 「지도 수명 주기」 |

실행 방법은 [rescue_bringup README](../rescue_bringup/README.md) 「지금 현장 실행 순서」를 본다.

## 라이브러리 (2026-10-08 작성, 지금도 코드와 같음)

Python 표준 라이브러리로 3D 좌표 변환, 원 관측 보존, 로봇별 지도 세션·보정·정렬 버전과 목표 snapshot의 유효성을 검사한다. 네트워크·ROS·TF·SLAM을 실행하거나 로봇에 명령하지 않는다.

**`VALID` / `coordinates_valid=True`는 입력된 기하·시각·버전의 일관성만 뜻한다. `auto_approach_eligible`은 모든 출력에서 항상 `False`다.** 실제 backend 보정 감지, 센서 시각/TF 검증, 실행기 정지·재개 연동이 없으며 이를 활성화하는 boolean 옵션도 제공하지 않는다.

공개 API는 `from rescue_mapping import ...`으로 가져온다.

| API | 역할 |
| --- | --- |
| `RigidTransform(translation_xyz, rotation_xyzw)` | source→target 3D 강체 변환. quaternion 순서 x/y/z/w, 비영·유한 quaternion 정규화. `apply`, `inverse`, `planar`, `identity` |
| `SourceObservation` | ID, source robot/camera/boot, 원 sensor frame·3D 점·관측 시각·원 만료, clock domain, 관측 시점에 결부된 map session/revision. frozen 값 |
| `TimedTransform` | 같은 로봇의 sensor→native map 변환. 원 관측 시각과 같은 query stamp 및 exact/검증된 보간 표본 구간 |
| `FrameAlignment` | 로봇 map→mission 변환, authority·시각·오차·session/map/transform revision |
| `MappingRegistry` | 원 관측·현재 파생 위치·기록·목표 snapshot을 메모리에 보존하고 검증 |

`MappingRegistry` 생성에는 `clock_domain_id`, `alignment_authority`, `max_position_error_m`, `max_rotation_error_rad`가 필요하다. 오차 상한은 호출자가 시험 기준에서 제공하며 코드 기본값은 없다. `mission_frame`은 기본 `mission`이다. 시각은 같은 명시적 clock domain의 양의 정수 ns다. 다른 PC의 monotonic 시각을 이 입력으로 섞으면 안 된다.

관측·sensor boot·map session ID는 canonical nonzero UUID 문자열이고 revision·map update sequence는 uint64 범위다. 잘못된 입력은 `MappingError`로 거절한다.

기본 순서는 `start_session` → `set_alignment` → `project` → `capture_goal_snapshot` → `validate_goal_snapshot`이다. `project(observation, sensor_to_map, now_ns=...)`는 TF가 없어도 유효한 원 관측을 먼저 보존하고 파생 위치 생성을 거절한다. 같은 관측 ID의 의미가 달라지면 `MappingError.code == 'ID_CONTENT_CONFLICT'`다. 새 map revision을 기존 관측에 덮어써 재관측을 가장하지 않는다.

`advance_map_revision`과 `start_session`은 해당 로봇 관측의 파생 위치 및 의존하는 source/destination 목표를 무효화한다. `set_alignment`의 유효한 정렬만 변경되면 파생 위치는 `STALE_DERIVATION`이며 같은 원 관측·sensor-time 변환으로 다시 계산할 수 있다. 지도 세션·보정 변경이나 정렬 무효화는 `NEEDS_REOBSERVATION`이다. 두 상태 모두 기존 목표를 무효화한다. map session/revision이 바뀌면 이 구현에는 posegraph 연계 재투영이 없어 새 관측이 필요하다. `record_map_update`는 점유 지도 갱신 순서만 바꾸며 보정 revision을 만들어내지 않는다.

`Invalidation.observation_ids`와 `goal_snapshot_ids`는 영향 목록이다. 이것을 반환했다는 사실은 실제 goal 취소·정지가 아니다. 미래 실행기가 이를 소비하고 수락/발행 직전 snapshot을 다시 검사해야 한다. 이 프로세스의 lock은 다른 PC의 검증·발행을 원자적으로 만들지 못한다.

저장소 루트의 순수 로직 시험:

```bash
PYTHONPATH=src/rescue_contracts:src/rescue_mapping python3 -m unittest discover -s tests -p test_mapping.py -v
```

테스트의 시간·오차·좌표는 합성 fixture이고 운용 기준이 아니다. Python wheel 설치는 가능하지만 ROS adapter·실제 backend 연동·`colcon`/실기 통과는 이 시험의 범위가 아니다. 저장소의 `docs/implementation/implementation-mapping.md`에 계약과 미구현 범위를 기록한다.

원본 코드는 복사하지 않았다. 프로젝트 라이선스·maintainer는 미결정이며 기존 `LicenseRef-Unspecified`·`maintainer@example.invalid`를 유지한다. 배포용 릴리스 metadata가 아니다.

`MappingError`는 공통 `RescueError`를 상속한다. `.code`는 C-09 공통 분류이고 `.detail_code`는 `rescue_mapping.OBSERVATION_EXPIRED` 같은 원 세부 코드다. 대응 사전은 `rescue_contracts` 한 곳에서 관리한다.

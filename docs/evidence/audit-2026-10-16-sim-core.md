---
title: TB-6 탐색·TB-5 접촉 수거 핵심 로직 감사
status: audit-open
updated: 2026-10-16
---

# TB-6 탐색·TB-5 접촉 수거 핵심 로직 감사

**접촉 수거와 수거 확인을 완료 판정에 연결하는 단계가 없고, TB-6의 중간 목표 성공이 탐색 조기 종료로 이어진다.** 단일 PC 실행 제약은 최신 시뮬레이션 실행기의 별도 처리로 보완되어 있어, 현재 전체 실행이 불가능하다는 지적으로 남기지 않는다.

이 문서는 감사 결과와 인계 조건을 기록한다. 기능·설정·인터페이스를 수정한 PR이 아니며, 아래 OPEN 항목은 해결됐다는 뜻이 아니다.

## 기준과 요청 범위

- 감사일: 2026-10-16, 한국 시간.
- 최초 채팅 감사: `d63da7515f59997c2607a2a0ef29253a1126f5f3`.
- 문서 작성 시 다시 확인한 main: `738f2d54cd67c5c0b7247d7fa34a2737be0fd323`. 아래 코드 링크는 이 SHA에 고정한다.
- [사용자 요구] 시뮬레이션에서 한 PC에 Gazebo·RViz·웹 서버를 함께 실행하고, 첨부 도면을 사용한다. 역할 원문: “tb-5는 인형에 닿으면 자동으로 수거하고 tb-6는 먼저 탐색을 하지만 인형을 찾아도 그냥 tb-5가 찾기 위해 위치만 던지지 수거하진 않을꺼야.”
- [사용자 요구] 작업 범위 정정 원문: “코어한 로직만 확인하고 부족한 부분이 없는지 audit만 하면 되잖아”. 이후 감사의 문서화와 PR 작성을 요청했다. 구현·배포·로봇 조작은 이번 작업 범위가 아니다.
- 기존 [현장 시나리오](../requirements/scope.md)는 사람이 인형을 옮기고 TB-5가 출구를 안내하는 흐름이다. AUD-01·02는 **이번 시뮬레이션 접촉 수거 요구와의 차이**이며, 기존 현장 정책을 잘못 구현했다는 판정이나 실기 접촉 수거 승인으로 확대하지 않는다.
- 첨부 SVG와 최초 감사의 `config/sim/worlds/arena-map.svg`는 SHA-256 `f407b5355534c237ec24f1f2629c2dfe9161ac19c45addde4c10d9c82219c59b`로 동일했다. 지도 형상 전체·센서 현실성·물리 접촉은 이번 핵심 로직 감사로 검증하지 않았다.

## 지적과 현재 상태

P1은 요청 시나리오의 성공 판정을 막는 중요 문제다. 이 표의 문제는 **감사 문서 PR 자체의 머지 차단 지적이 아니라 후속 구현·인수의 미해결 조건**이다.

| ID | 우선순위·상태 | 확인한 문제 | 후속 담당 제안 |
| --- | --- | --- | --- |
| AUD-01 | P1 · OPEN | TB-5가 인형 앞에 정지한 뒤 접촉·수거하는 단계가 없음 | 로봇·지도 + 시뮬레이션 담당, 작전 조율 |
| AUD-02 | P1 · OPEN | 수거 확인 없이 출구 도착을 구조 완료로 기록 | 협업·관제 + 통합/계약 |
| AUD-03 | P1 · OPEN | 짧은 중간 목표 성공으로 원래 미탐색 경계 전체를 제외 | 협업·관제 |
| AUD-04 | 실행 제약 · MITIGATED | 현장 LIVE_ROS는 동일 호스트를 거부하지만 전체 시뮬 실행기는 host_shim 사용 | 실기·배포/시뮬레이션 담당: 격리·실행 증거 확인 |

### AUD-01 — 접근 완료 다음의 접촉·수거 단계가 없다

[설정·원본 코드, 2026-10-16] [현장 프로파일 L29–35](https://github.com/jaebeom/ROKEY-7/blob/738f2d54cd67c5c0b7247d7fa34a2737be0fd323/config/field/search_area.field.json#L29-L35)의 `approach_standoff_m`은 0.8 m다. [자동 접근 L2047–2100](https://github.com/jaebeom/ROKEY-7/blob/738f2d54cd67c5c0b7247d7fa34a2737be0fd323/src/rescue_coordination/rescue_coordination/ros_node.py#L2047-L2100)는 이 거리만큼 떨어진 점을 TB-5 목표로 보낸다. 이미 이 거리 안이면 [approach_point](https://github.com/jaebeom/ROKEY-7/blob/738f2d54cd67c5c0b7247d7fa34a2737be0fd323/src/rescue_coordination/rescue_coordination/ros_node.py#L296-L310)는 더 다가가지 않고 현재 자리에서 대상 쪽을 본다.

현재 [인형 모델](https://github.com/jaebeom/ROKEY-7/blob/738f2d54cd67c5c0b7247d7fa34a2737be0fd323/config/sim/worlds/arena.sdf#L156-L163)은 충돌체와 시각 모델이 있는 고정 원기둥이다. `src/rescue_*`, `scripts/sim`, `config/sim`에서 `pickup`, `collect`, `contact`, `수거`, `doll`을 검색하고 접근 결과·모델 생성 경로를 읽었으나, TB-5 접촉 확인 → 인형 제거/적재 → 수거 성공 기록의 연결은 확인하지 못했다. world의 Contact 시스템 플러그인 존재만으로 수거 처리가 구현되지는 않는다. `fake_target.py`는 인형 **관측 대역**이다.

**영향:** 탐지·위치 전달·Nav2 접근이 성공해도 인형까지 자동으로 접촉하는 단계가 없고, 접촉하더라도 TB-5에 한정한 수거 결과를 만들지 않는다.

**풀리는 조건 [제안]:** 접근 후 최종 접촉 단계를 기존 로봇 명령 소유권 안에 연결하고, 동일 회차·대상에 대한 실제 접촉과 수거 처리를 구분해 기록한다. TB-6/벽 접촉은 무수거, TB-5 접촉은 한 번만 수거, 중복 이벤트·제거 실패는 성공 오인 없이 처리하는 시험이 필요하다. `approach_standoff_m`이나 충돌 방지 값을 낮추는 것만으로 해결했다고 판단하지 않는다.

### AUD-02 — 수거하지 않아도 출구 도착으로 구조 완료가 된다

[설정·원본 코드, 2026-10-16] 현재 [프로파일 L52](https://github.com/jaebeom/ROKEY-7/blob/738f2d54cd67c5c0b7247d7fa34a2737be0fd323/config/field/search_area.field.json#L52)의 `completion_mode=EXIT_ARRIVAL` 경로에 대한 지적이다. `RECHECK` 모드까지 같은 판정이라고 일반화하지 않는다. [approach_finished L412–418](https://github.com/jaebeom/ROKEY-7/blob/738f2d54cd67c5c0b7247d7fa34a2737be0fd323/src/rescue_coordination/rescue_coordination/mission.py#L412-L418)는 이동 성공과 정지 확인으로 `ARRIVED`를 만든다. [출구 이동 L1715–1748](https://github.com/jaebeom/ROKEY-7/blob/738f2d54cd67c5c0b7247d7fa34a2737be0fd323/src/rescue_coordination/rescue_coordination/ros_node.py#L1715-L1748)은 그 뒤 설정된 대기 시간과 지도·준비·교통 조건을 검사하며 수거 상태는 검사하지 않는다. 출구 이동 성공 시 [호출부 L1278–1283](https://github.com/jaebeom/ROKEY-7/blob/738f2d54cd67c5c0b7247d7fa34a2737be0fd323/src/rescue_coordination/rescue_coordination/ros_node.py#L1278-L1283)가 [출구 도착 판정 L1844–1848](https://github.com/jaebeom/ROKEY-7/blob/738f2d54cd67c5c0b7247d7fa34a2737be0fd323/src/rescue_coordination/rescue_coordination/ros_node.py#L1844-L1848)을 불러 곧바로 `RESCUE_CONFIRMED`를 기록한다.

**재현 조건과 영향:** 인형을 그대로 둔 채 접근 작업과 출구 이동 작업만 성공시키면 수거 증거 없이 구조 완료가 된다. [TaskResult](https://github.com/jaebeom/ROKEY-7/blob/738f2d54cd67c5c0b7247d7fa34a2737be0fd323/src/rescue_interfaces/msg/TaskResult.msg#L14-L30)의 `TARGET_APPROACH` 완료는 접촉 수거 완료를 뜻하지 않는다.

**풀리는 조건 [제안]:** 이번 시뮬레이션의 출구 출발·완료 판정에 해당 회차와 대상의 수거 증거를 연결한다. 수거되지 않았거나 실패·다른 대상·이전 회차 이벤트이면 출구 출발/수거 성공 판정이 나지 않아야 한다. 기존 현장 `EXIT_ARRIVAL` 정책과 이 시뮬레이션 조건의 적용 범위는 별도 구현 PR에서 명시한다.

### AUD-03 — 중간 목표 성공을 경계 탐색 완료로 처리한다

[원본 코드, 2026-10-16] [경로 절단 L805–815](https://github.com/jaebeom/ROKEY-7/blob/738f2d54cd67c5c0b7247d7fa34a2737be0fd323/src/rescue_coordination/rescue_coordination/ros_node.py#L805-L815)은 먼 frontier(알려진 영역과 미지 영역의 경계)까지의 경로를 짧은 이동 목표로 자른다. 그런데 [성공 처리 L842–860](https://github.com/jaebeom/ROKEY-7/blob/738f2d54cd67c5c0b7247d7fa34a2737be0fd323/src/rescue_coordination/rescue_coordination/ros_node.py#L842-L860)는 원래 경계의 goal과 centroid를 모두 재선택 제외 목록에 넣는다. 다음 지도 갱신에서 경계가 남아 있어도 후보가 사라져 탐색 완료로 이어진다.

[측정, E2, 2026-10-16] 아래 순수 Python 재현은 최신 기준 SHA에서도 **원래 목표까지 1.6 m의 도달 가능한 경로와 frontier 1개가 남은 상태에서 `EXPLORATION_COMPLETE`**가 되는 것을 확인했다. `SAVED_MAP` 테스트 fixture로 경계 선택 코어를 분리한 재현이며, 실제 SLAM 센서나 Gazebo 실행 결과가 아니다.

**영향:** 인형이 아직 탐색하지 않은 안쪽에 있으면 발견 전에 끝날 수 있다. 실제 `SLAM_FREEZE` 흐름의 [생존자 없음 판정 L1840–1855](https://github.com/jaebeom/ROKEY-7/blob/738f2d54cd67c5c0b7247d7fa34a2737be0fd323/src/rescue_coordination/rescue_coordination/ros_node.py#L1840-L1855)도 `exploration_complete`에 의존한다. 이 뒤의 지도 고정·귀환까지 이번 재현으로 실행 검증한 것은 아니다.

**최신 main 반영사항:** [PR #390](https://github.com/jaebeom/ROKEY-7/pull/390)은 실패한 경계를 두 번까지 시도하고, 실패로 포기한 경계만 남으면 `EXPLORATION_FRONTIERS_FAILED`로 표시한다. 최초 감사의 “실패한 목표도 즉시 제외”는 최신 상태 설명에서 제외한다. **중간 목표가 성공한 경우의 조기 완료는 남아 있다.**

**풀리는 조건 [제안]:** 중간 이동 성공과 원래 경계 소진을 분리한다. 짧은 이동 뒤 경계가 남아 있으면 다음 접근 목표를 선택해야 하며, 성공·일시 실패·재시도 소진·실제 탐색 완료를 구분하는 시험이 필요하다. 기존 `test_coordination_frontier.py`의 성공 후 경계 제외를 기대하는 시험도 함께 검토한다.

### AUD-04 — 동일 호스트 제약은 시뮬레이션 실행기에서 보완됨

[원본 코드, 2026-10-16] [live_clock.py L118–132](https://github.com/jaebeom/ROKEY-7/blob/738f2d54cd67c5c0b7247d7fa34a2737be0fd323/src/rescue_robot/rescue_robot/live_clock.py#L118-L132)는 조정기와 로봇 실행기의 `host_id`가 같으면 `CLOCK_UNCERTAIN` / `LIVE_ROS expects two distinct PCs`로 거부한다. **현장 launch를 그대로 한 호스트에 모으면 이 제약이 적용된다.**

다만 최신 main에는 [run_full.sh L218–220](https://github.com/jaebeom/ROKEY-7/blob/738f2d54cd67c5c0b7247d7fa34a2737be0fd323/scripts/sim/run_full.sh#L218-L220)가 있다. 두 실행기에 `REHEARSAL_HOST_ID`와 [host_shim](https://github.com/jaebeom/ROKEY-7/blob/738f2d54cd67c5c0b7247d7fa34a2737be0fd323/scripts/field/wiring-rehearsal/host_shim/sitecustomize.py#L1-L22)을 적용하여 시계 측정의 호스트 식별자를 바꾼다. [시뮬레이션→현장 구분표](../process/sim-to-field.md)에도 기록돼 있다.

**정정:** 최초 채팅의 단일 PC 제약을 현재 전체 시뮬레이션의 전면 실행 차단으로 해석하면 부정확하다. 상태는 `MITIGATED`(별도 처리 존재), 실제 기동 확인은 `NOT_RUN`이다. 다음 실행 담당은 shim 적용 프로세스·시계 로그와 현장 실행으로 섞이지 않는 범위를 확인한다. 현장 시계 검사를 무조건 삭제하는 변경을 요청하지 않는다.

## 재현과 검증

### AUD-03 순수 Python 재현

위 기준 SHA의 저장소 루트에서 실행한다. 파일을 수정하지 않는다. 아래의 1 m 이동 한도와 지도 갱신 2회는 **기존 테스트 fixture 설정**으로, 현장 프로파일 값을 바꾼 것이 아니다.

```bash
python3 -B - <<'PY'
from pathlib import Path
import math
import sys
for path in sorted(Path('src').glob('rescue_*')):
    sys.path.insert(0, str(path.resolve()))
sys.path.insert(0, str(Path('tests').resolve()))
from test_coordination_frontier import Base, grid, scout_pose, NOW, DELAY
from rescue_contracts.grid_search import find_frontiers, grid_path

case = Base()
case.setUp()
case.core.on_scout_map(grid())
case.core.on_pose(scout_pose())
case.searches()
original = dict(case.core.frontier)
(task,) = case.searches(NOW + DELAY)
mid = original['site_xy']
case.core.on_pose(scout_pose(mid, at=NOW + DELAY))
case.finish(task, 'COMPLETED')
for k in range(2):
    now = NOW + DELAY + k + 1
    case.core.on_scout_map(grid())
    case.core.on_pose(scout_pose(mid, at=now))
    assert case.searches(now) == []
fields = grid().fields
frontiers = find_frontiers(**fields, radius=0.2, min_cells=3, max_goal_distance_m=1.0)
path = grid_path(**fields, radius=0.2, start=mid, goal=original['goal'])
print('original_goal=', original['goal'], 'intermediate_goal=', mid)
print('remaining_distance_m=', round(math.dist(mid, original['goal']), 3))
print('remaining_frontiers=', len(frontiers), 'remaining_path_m=', round(path.length_m, 3))
print('exploration_complete=', case.core.exploration_complete,
      'reason=', case.core.exploration_reason)
assert len(frontiers) == 1 and path.length_m > 1.5
assert case.core.exploration_complete and case.core.exploration_reason == 'EXPLORATION_COMPLETE'
PY
```

실측 출력(E2, 위 기준 SHA):

```text
original_goal= (3.7, 1.9000000000000001) intermediate_goal= (2.1, 1.9000000000000001)
remaining_distance_m= 1.6
remaining_frontiers= 1 remaining_path_m= 1.6
exploration_complete= True reason= EXPLORATION_COMPLETE
```

종료 코드 0은 **버그가 재현됐음**을 뜻한다. 탐색 요구를 통과했다는 뜻이 아니다.

### 실행 검사와 한계

- 최초 감사 `d63da75`: 접근·출구·임무 종료·경계 탐색·자동 접근 기존 시험 64개, `OK`, 종료 코드 0. 현재 동작에 맞춘 시험이므로 접촉 수거 요구 충족의 증거가 아니다.
- 최신 기준 SHA: 위 재현 종료 코드 0(버그 존재 확인). `python3 -I scripts/run_tests.py` 종료 코드 0, `Ran 2363 tests in 66.935s`, `OK (skipped=63, expected failures=2)`. 건너뛴 63개와 예상 실패 2개는 통과 증거로 세지 않는다.
- `python3 -I scripts/check_docs.py`, `git diff --check`: 종료 코드 0. 문서 검사는 링크·경로를 검사하며 문장 의미나 실행 동작을 보증하지 않는다. 원본 별도 저장소 객체 검사는 `--sources` 미제공으로 SKIP이다.
- E3 ROS/Gazebo 통합, 실제 접촉·모델 제거, 단일 PC 실제 기동, E4 실기: `NOT_RUN`.
- [인형 관측 대역](https://github.com/jaebeom/ROKEY-7/blob/738f2d54cd67c5c0b7247d7fa34a2737be0fd323/scripts/sim/fake_target.py#L1-L32)은 사전 도면 좌표와 TF·시야·지도 가림 조건으로 관측을 만든다. 이 경로의 성공을 실제 카메라/YOLO 인형 검출 성능으로 보지 않는다.
- TB-6 탐색·위치 전달 / TB-5 접근의 역할 분리는 기존 설정과 조정기 발행 경로에서 확인했다. TB-6의 1차 지도 고정 후 FRONTIER 탐색 재개도 존재한다. AUD-01은 TB-6가 수거한다는 버그가 아니라 수거 기능 자체의 누락이다.

전체 시험 결과 원문 끝부분:

```text
Ran 2363 tests in 66.935s

OK (skipped=63, expected failures=2)
```

## 후속 에이전트 인계

| ID | 수정 검토 위치 | 통과 조건 | 처리 상태 |
| --- | --- | --- | --- |
| AUD-01 | `rescue_robot` 접근 실행 경계, `scripts/sim/` 접촉·수거 대역 | TB-5 접촉 수거 1회, TB-6 접촉 무수거, 처리 실패·중복 구분 | OPEN · 담당 제안만 기록 |
| AUD-02 | `ros_node.py`의 `_issue_support_exit`, `_confirm_by_exit_arrival`; 수거 결과 인터페이스 | 미수거·다른 대상·이전 회차 결과로 완료되지 않음 | OPEN · AUD-01 결과 의미와 함께 검토 |
| AUD-03 | `ros_node.py`의 `_frontier_tick`, `_frontier_done`; `test_coordination_frontier.py` | 중간 목표 성공 후 남은 경계 계속 탐색, 실제 소진과 실패 종료 구분 | OPEN · 기존 #390 실패 재시도 유지 |
| AUD-04 | `run_full.sh`, `host_shim`, `live_clock.py` | 실제 단일 PC 기동 로그, 현장 경로 격리 확인 | MITIGATED · E3 NOT_RUN |

후속 PR은 이 문서의 ID와 검토 SHA를 인용하고, 수정 전 재현·수정 후 결과·미실행 범위를 연결한다. 실제 소유권 배정과 공통 인터페이스 변경은 [검토 절차](../process/review-workflow.md)와 [A-07](../process/acceptance-plan.md)을 따른다. 이 문서를 게시한 것만으로 담당 배정·구현 착수·해결 완료를 뜻하지 않는다.

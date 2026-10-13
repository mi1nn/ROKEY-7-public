# 탐색 알고리즘 3종 주행 관측 — 2026-10-13

> **기록 (2026-10-13).** 탐색 알고리즘 주행 관측 기록이다. 지금 현장 탐색은 explore_lite 가 아니다. 조정기가 `search_mode` FRONTIER 로 TB-6 의 다음 목표를 직접 고른다(`rescue_coordination/ros_node.py`, `rescue_contracts.grid_search`, 현장 파일 [search_area.field.json](../../config/field/search_area.field.json)). 현장 실행은 [rescue_bringup README](../../src/rescue_bringup/README.md)를 본다. 아래의 robot6 은 TB-6 이다.

현재 상태는 단독 주행 관측 기록이다. 전체 지도 완성·독립 정확도·세 알고리즘의 최종 성능 우열은 검증하지 않았다. [선정안](exploration-algorithm-selection-2026-10-13.md)과 함께 검토한다.

## 범위와 공통 조건

[측정/설정, 2026-10-13] ROS 2 Jazzy, robot6, 도메인 6, SLAM+Nav2. 중앙 서버 연결 장애 뒤 로봇 내부 Discovery Server에 PC를 직접 연결해 정상 scan·odom·TF 및 언도킹 상태를 확인했다. 로봇의 Discovery 설정은 바꾸지 않았다. SSH 인증 실패로 로봇 파일·서비스 설정을 직접 점검하지는 못했다.

세 회차는 새 SLAM 지도로 시작하고 RViz·지도·odom·로그를 기록했다. 도킹·언도킹·위치 복귀는 사용자가 수행하며 탐색 시간에 포함하지 않는다. [사용자 보고, 2026-10-13] 이전 정체 구역을 물리적으로 차단하고 같은 시작 장소로 옮겼다. 물리 배치·시작 위치·자세의 정밀 일치는 독립 측정하지 않았다.

- 두 costmap inflation_radius: 0.25m; cost_scaling_factor: 4.0.
- DWB max_vel_x/max_speed_xy: 0.26m/s; max_vel_theta: 1.0rad/s. smoother 상·하한도 일치.
- Nav2 설정 원문 SHA256: `ce27caeaecf8b9471300b3b283b982265bb46446fe0360507f4cc541050426db`. 세 회차의 byte 동일성 확인.
- 시간 제한: 180초. 최종 탐색 완료 기준으로 쓰지 않는다.
- 알고리즘 기본 설정은 개별 첨부본 참조. 2번 occ_threshold=75는 1·3번과 공유하는 파라미터가 아니다.

## 결과

[측정, 2026-10-13] 원문 숫자는 [CSV](../evidence/exploration-2026-10-13/comparison.csv)에 있다.

| 번호 | 알고리즘 | 종료까지 시간(s) | odom 거리(m) | 최종 지도 확인 영역(m²) | 종료 관측 |
| --- | --- | ---: | ---: | ---: | --- |
| 1 | `explore_lite` | 29.93 | 6.30 | 19.62 | 자체 완료 신호 |
| 2 | `frontier_exploration_ros2` | 36.60 | 7.27 | 19.87 | 자체 완료 신호 |
| 3 | `frontier_toolbox` | 136.15 | 7.30 | 19.81 | 동일 목표 반복·정체 중단 |

세 회차 모두 중단 후 odom으로 실제 정지를 확인했다. 3번의 최초 중단 관측은 이전 움직임 표본이 남아 미확인이었고, 새 표본을 다시 모아 정지를 확인했다. 원문 실패 관측을 덮어쓰지 않았다.

## 수치의 의미

시간은 기록기가 탐색 프로세스를 시작한 시점부터 종료 관측/중단 판단까지이며 초기 연결·활성화 시간이 포함된다. 중단 후 정지 확인 시간은 별도다. 3번은 native 전체 완료 신호가 없고, 136.15초는 정체 중단까지의 시간이다. 완료 시간과 같은 값으로 순위를 매기지 않는다.

이동 거리는 odom 위치 표본 사이 거리의 합이다. 정지 중 odom 잡음이 누적될 수 있어 독립 실측 경로 길이가 아니다. 지도 확인 영역은 SLAM 지도에서 `data>=0`인 칸 수 × resolution²로, 빈 공간과 장애물 칸을 포함한다. 실내 전체 구역 대비 탐색률이나 정확도가 아니다. 서로 다른 사각 지도 범위의 unknown 비율도 직접 비교하지 않는다.

## 개별 종료 관측

### 1번 explore_lite

`exploration_complete`와 `No frontiers found, stopping.`을 관측했다. `return_to_init=false`로 자동 복귀를 막았다. pause와 실제 정지 확인 뒤 종료 요청을 처리했고, SIGINT 시간 초과 후 소유·namespace를 확인한 정확한 탐색 PID를 SIGTERM으로 정리했다. 전체 공간을 모두 탐색했다고 판정하지 않는다.

### 2번 frontier_exploration_ros2

`No more frontiers found`와 완료 이벤트를 관측했다. 직전 비용 99인 목표를 건너뛴 로그도 있으므로 접근 가능한 전체 공간 완료 여부는 미검증이다. `goal_skip_on_blocked_goal=true`, `occ_threshold=75`, 자동 복귀=false로 시험했다.

### 3번 frontier_toolbox

upstream 기본 naive distance+DBSCAN 변형과 전용 Nav2 client를 사용했다. 목표 생성·전달·취소는 먼저 격리 도메인에서 가짜 액션 서버로 점검했다. 필드에서는 남은 경계 21개·군집 1개 상태에서 같은 목표를 약 1초마다 취소·재전송했고 지도 확인 영역이 정체됐다. `republish_same_goal=true`를 시험 중 변경하지 않았다. native 전체 완료는 없었고 감독자가 중단했다. 실제 정지와 전용 프로세스 종료를 확인했다.

`USER_REQUEST_STOPPED`는 이 기록기의 일반 중단 파일 표기다. 이번 파일은 감독자가 정체를 보고 만든 것으로 사용자 신규 중단 지시를 뜻하지 않는다. 정리한 결과는 `BLOCKED / STALLED_REPEATED_IDENTICAL_GOAL`이다.

## 근거 자료와 한계

[자료 manifest](../../assets/manifest/exploration-2026-10-13.json)에 원문·첨부본 SHA256과 취득 방법을 둔다. 개인 경로·호스트·네트워크 주소는 치환했으며 치환된 명령은 실행용이 아니다. 각 회차 폴더에 result/summary, 시간별 CSV, 최종 지도, 적용 설정, 탐색 로그, 정지 확인을 보존한다.

이 기록은 2026-10-13의 최종 공통 설정 회차 3개로 한정한다. 이전 회차의 0.08m 벽 접촉 사용자 보고, 차단 기준 65/75 정체, DDS 장애를 이 표의 성공 근거로 섞지 않는다. 전체 로봇 인수·독립 정확도·반복 시험은 NOT_RUN이다.

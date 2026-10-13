---
title: AMR1 무지도 자율 탐색 알고리즘 후보와 비교 계획
status: proposal-e1
updated: 2026-10-12
---

# AMR1 무지도 자율 탐색 알고리즘 후보와 비교 계획

> **기록 (2026-10-12).** 탐색 알고리즘 후보 조사 기록이다. 지금 현장 탐색은 explore_lite 가 아니다. 조정기가 `search_mode` FRONTIER 로 TB-6 의 다음 목표를 직접 고른다(`rescue_coordination/ros_node.py`, `rescue_contracts.grid_search`, 현장 파일 [search_area.field.json](../../config/field/search_area.field.json)). 현장 실행은 [rescue_bringup README](../../src/rescue_bringup/README.md)를 본다. 아래의 AMR1 은 지금의 TB-6 이다.

이 문서는 **사전 지도가 없을 때** AMR1이 SLAM으로 지도를 만들면서 다음 탐색 목표를 스스로 고르는 방법을 비교하기 위한 E1 문헌 검토다. [범위 RQ-03·첫 통합 시나리오](../requirements/scope.md)의 지속 지도 갱신과 [D-01·D-05](../design/algorithm-decisions.md)의 SLAM/Nav2·탐색 제안을 구체화한다. 알고리즘 채택 결정, ROS 통합 통과 또는 실기 성능 보고가 아니다. 후보 5개의 프로젝트 적용 시험 상태는 모두 `NOT_RUN`이다.

## 공통 구성과 비교 범위

**[제안]** 첫 비교에서는 기존 단독 주행 기반인 `slam_toolbox`와 Nav2의 설정을 고정하고, **다음 탐색 목표를 고르는 방법**을 바꾼다. SLAM은 지도·위치를 제공하고, Nav2는 선택한 목표로 이동한다. [Nav2의 SLAM 주행 안내](https://docs.nav2.org/jazzy/tutorials/general_tutorials/navigation2_with_slam/navigation2_with_slam/)와 [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)가 이 역할 구분의 공식 자료다. 사용자가 보고한 AMR1의 수동 목표 `SLAM + Nav2` 및 저장 지도 `localization + Nav2` 주행 성공은 이 비교의 출발 경험이다. 이 문서에서 해당 실기 로그를 독립 검증하거나 새 후보를 실행하지 않았다.

여기서 **frontier**는 지도상 알려진 자유 공간과 미지 공간의 경계다. frontier를 목표로 삼는 기본 원리는 [Yamauchi, 1997](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/yamauchi_frontiers.pdf)에 근거한다. 탐색 후보가 frontier를 찾았다는 사실만으로 주행 가능한 목표나 카메라 관측 완료를 뜻하지 않는다.

**[제안] 비교 범위:** 우선 단일 AMR1·무지도·동일 환경/시작 조건에서 자동 지도 탐색을 비교한다. 두 로봇 구역 배정, 공통 지도 정렬, 관제 `SEARCH_AREA` 통합은 [D-02·D-05](../design/algorithm-decisions.md) 및 [F-01](../process/acceptance-plan.md)의 후속 검증이다. 고정 waypoint 순찰이나 기지도가 필요한 구역 전면 커버리지는 이번 무지도 초기 비교 대상에서 제외한다.

## 후보와 선택 근거

아래 다섯 개는 서로 다른 계층의 구현 후보이다. `frontier_toolbox`는 단일 점수식이 아니라 여러 frontier 평가 함수를 제공한다. 외부 저장소의 자체 성능 주장은 프로젝트의 측정값이 아니다.

| 후보 / 우선순위 | 다음 목표를 고르는 방법 | 선택 근거 | 적용 전 확인할 점 |
| --- | --- | --- | --- |
| [explore_lite](https://github.com/robo-friends/m-explore-ros2) / 1차 기준 | 지도에서 frontier를 군집화하고 거리·미지 영역 이득을 평가 | [강의 자료 저장소](https://github.com/kimandreas/to_students/tree/main)의 AutoSLAM과 기존 [D-05](../design/algorithm-decisions.md)의 재사용 우선 후보. 복잡한 방식의 이득을 비교할 기준 | upstream 전체 노드는 Nav2 goal을 직접 보내고 기본값으로 자동 시작한다. 협업 통합에는 후보 생성부와 주행 발행부를 분리해야 함 |
| [frontier_exploration_ros2](https://github.com/mertgulerx/frontier_exploration_ros2) / 1차 비교 | WFD 방식 frontier 검출, 지도 전처리, MRTSP 기반 순서 결정(greedy 또는 제한 범위 DP) | 거리만 보는 선택보다 방문 순서·재방문이 개선되는지 검증. 작성자는 Jazzy/Humble·SLAM Toolbox·Nav2 시험 환경과 재사용 C++ 코어를 명시 | 작성자 환경은 TurtleBot3 Waffle Pi. 센서 범위·footprint·QoS·프레임·goal 발행권한을 AMR1에서 검증. 전처리와 목표 선정의 효과를 구별 |
| [frontier_toolbox](https://github.com/Schelbert197/frontier_toolbox) / 조건부 비교 | frontier 검출·DBSCAN 군집과 거리, entropy, 미지 셀 변화량 등의 선택 함수를 조합 | 같은 frontier에서 목표 점수 기준별 차이를 분석할 수 있음 | [사용 문서](https://github.com/Schelbert197/frontier_toolbox/blob/main/use.md)는 Humble·Gazebo Classic 예시이며 안내하는 `humble-devel` 브랜치는 2026-10-12 원격 브랜치 조회에서 확인되지 않았다. [예제 노드](https://github.com/Schelbert197/frontier_toolbox/blob/main/frontier_exp_cpp/src/frontier_lc.cpp)는 `/goal_pose` 또는 `jackal_nav_to_pose`로 주행 요청한다. 실제 존재하는 브랜치·Jazzy 빌드·후보 함수 분리를 확인한 뒤 비교한다 |
| [RRT_exploration](https://github.com/Incomprehensible/RRT_exploration) / 추가 비교 | 무작위 트리 확장으로 탐색 경계 후보를 찾음 | 격자 기반 frontier 검출과 다른 후보 생성 방식을 비교 | 작성자가 실험용 구현, 탐색 후반 감속·미해결 버그, Humble·Gazebo Classic 시험을 명시. Jazzy 이전 및 안정성 검증 필요 |
| [ARiADNE-ROS-Planner](https://github.com/marmotlab/ARiADNE-ROS-Planner) / 연구 후보 | 부분 지도를 informative graph로 구성하고 강화학습된 Attention 정책이 다음 waypoint를 선택 | 손으로 설계한 점수식과 학습된 장기 목표 선택을 비교할 수 있음 | 공개 ROS2 지원은 Humble 기준. 지도·센서·좌표계·Nav2 goal 연결과 학습 환경 차이를 검증. 제공 checkpoint가 AMR1에서 유효한지 미확인 |

`frontier_exploration_ros2`의 논문은 아래 WFD·지도 전처리/MRTSP **기반 연구**이며, 해당 패키지 전체에 대한 독립 실험 논문은 저장소가 준비 중이라고 표시한다. `frontier_toolbox`의 이 문서상 근거는 공개 구현·사용 안내이고 특정 논문의 공식 재현판이라는 주장은 하지 않는다. RRT 구현도 원 논문에서 영감을 받은 별도 ROS2 코드다. ARiADNE의 학습 모델은 사람 탐지 모델이나 SLAM backend가 아니라 다음 탐색 waypoint를 고르는 정책이다.

### GitHub와 논문 출처

| 후보 | 구현 출처 | 알고리즘 근거의 범위 |
| --- | --- | --- |
| explore_lite | [robo-friends/m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) | [Yamauchi, *A Frontier-Based Approach for Autonomous Exploration*, CIRA 1997](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/yamauchi_frontiers.pdf): frontier 원리. ROS2 구현 자체의 검증 논문은 아님 |
| frontier_exploration_ros2 | [mertgulerx/frontier_exploration_ros2](https://github.com/mertgulerx/frontier_exploration_ros2) | [*Frontier Based Exploration for Autonomous Robot*, 2018](https://arxiv.org/abs/1806.03581): WFD 탐색. [*Enhancing autonomous exploration for robotics via real time map optimization and improved frontier costs*, Scientific Reports 2025](https://www.nature.com/articles/s41598-025-97231-9): 전처리·MRTSP 비용의 참고 연구. 구현의 DP 확장까지 논문 검증이라고 보지 않음 |
| frontier_toolbox | [Schelbert197/frontier_toolbox](https://github.com/Schelbert197/frontier_toolbox), [사용 문서](https://github.com/Schelbert197/frontier_toolbox/blob/main/use.md) | 공개 함수와 설정을 근거로 사용. 특정 논문과 구현의 일대일 대응은 미확인 |
| RRT_exploration | [Incomprehensible/RRT_exploration](https://github.com/Incomprehensible/RRT_exploration) | [Umari·Mukhopadhyay, *Autonomous robotic exploration based on multiple rapidly-exploring randomized trees*, IROS 2017](https://ieeexplore.ieee.org/document/8202319/): 기반 아이디어. ROS2 패키지 자체의 실기 검증과 구분 |
| ARiADNE | [marmotlab/ARiADNE-ROS-Planner](https://github.com/marmotlab/ARiADNE-ROS-Planner) | [Cao 등, *ARiADNE*, ICRA 2023](https://arxiv.org/abs/2301.11575): Attention 강화학습 탐색 정책. [후속 대규모 탐색 연구, 2024](https://arxiv.org/abs/2403.10833)는 확장 맥락이며 현재 ROS2/Jazzy 통합 통과 근거가 아님 |

## 비교 순서와 판정 방법

1. **E1 호환성 점검:** 각 후보의 버전·라이선스·Jazzy 빌드 가능성, 입력 지도/QoS, TF·namespace, Nav2 목표 출력 방식, 시작·중단·종료 인터페이스를 기록한다. 원본 브랜치와 실제 시험 커밋을 고정한다.
2. **E3 격리 시뮬레이션:** 동일한 SLAM/Nav2 설정, 환경·초기 자세·센서 조건, 반복 횟수를 고정한다. 각 후보의 미탐색 종료, 실패 목표, 주행 정체, 재시작을 기록한다. 직접 goal을 보내는 upstream 노드 시험에는 다른 goal 발행기를 동시에 켜지 않는다.
3. **E4 실제 AMR1:** E3에서 동작하고 정지 제어가 확인된 후보만 현장 감독 아래 평가한다. 로봇 번호·ROS domain·namespace와 단일 주행 발행자를 확인한 뒤 진행한다.

**[제안] 사전에 고정할 비교값:** 접근 가능한 영역의 탐색 비율, 총 소요 시간, 이동 거리, 반복 방문, 실패/정체·수동 개입 횟수, 목표 재계획 지연, CPU/RAM, 중단 명령 뒤 goal 취소와 실제 정지, 종료 사유. 임계값·시험 횟수·공간의 기준 지도 또는 독립 측정 방법은 시험 전에 [acceptance-plan](../process/acceptance-plan.md)에 정한다. 지도 점유 상태와 카메라의 구조 대상 관측 범위는 별도로 평가한다. frontier 소진만으로 모든 대상 탐색 완료를 선언하지 않는다.

## 프로젝트 통합 관문

[D-05](../design/algorithm-decisions.md)는 협업 통합에서 frontier 생성기를 후보 공급자로 재사용하고, **`rescue_robot`만 Nav2 goal·cancel을 발행**하도록 제안한다. 다른 후보에도 같은 제어권 원칙을 적용한다. 후보의 좌표·시각·지도 revision과 도달 가능성을 검증하고, 선택된 목표를 `rescue_robot`에 넘기는 adapter가 필요하다. upstream의 직접 주행 데모 성공은 이 adapter가 구현되었다는 증거가 아니다.

**[미결정]** 최종 채택 알고리즘, 실제 센서·로봇 footprint에 맞는 설정, 중단/재개·완료 계약, 관제 `SEARCH_AREA`와 탐색 영역 제한, 두 로봇 영역 분할·지도 정렬은 실험과 담당자 검토 뒤 확정한다. 현재는 `explore_lite`를 기준으로 `frontier_exploration_ros2`를 먼저 비교하고, `frontier_toolbox`는 브랜치·Jazzy 호환성이 확인될 때 비교하는 **시험 순서 제안**이다. RRT·ARiADNE는 결과나 연구 필요에 따라 확장한다.

## 처음 읽는 사람을 위한 1~3번 학습 메모

이 절은 논문에서 설명하는 **원리**와 GitHub 코드에서 확인한 **구현 동작**을 구분해 적는다. 링크는 메모를 만든 위치로 바로 연결한다. GitHub `main`은 바뀔 수 있으므로 실제 시험 PR에서는 사용한 커밋으로 링크를 고정해야 한다. 다음의 A/B 숫자는 이해용 예시이고 측정 결과가 아니다.

> 현재 위치에서 frontier A까지 2 m, A에서 새로 볼 것으로 예상한 경계가 4칸이다. B는 5 m 떨어져 있지만 20칸을 볼 수 있을 것으로 예상한다. **어떤 후보를 고를지는 거리와 기대 정보에 매긴 점수에 따라 달라진다.** 실제로 4칸·20칸이 드러난다는 보장은 없다.

### 먼저 알아둘 말

| 말 | 쉬운 뜻 | 메모 근거 |
| --- | --- | --- |
| occupancy grid | 공간을 작은 칸으로 나눈 지도. 각 칸의 상태를 자유·장애물·미지로 구분 | [WFD 논문 1쪽, Introduction의 용어 정의·Fig. 1](https://arxiv.org/pdf/1806.03581#page=1) |
| frontier | 이동 가능한 것으로 알려진 영역과 아직 보지 못한 영역의 경계 | [Yamauchi 논문 1~2쪽, Introduction·Frontier-Based Exploration](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/yamauchi_frontiers.pdf#page=2) |
| 정보 이득 | 목표에 갔을 때 새로 알게 될 지도의 양에 대한 **예측값**. 실제 새 관측량과 구분 | [2025 논문 4쪽, Frontier detection·식 (6)](https://www.nature.com/articles/s41598-025-97231-9.pdf#page=4) |
| BFS / WFD | BFS는 가까운 지도 칸부터 차례로 훑는 탐색. WFD는 알려진 영역을 훑는 바깥 BFS와 붙어 있는 frontier를 묶는 안쪽 BFS를 사용 | [WFD 논문 3쪽, III. Methodology·Fig. 4](https://arxiv.org/pdf/1806.03581#page=3) |

### 1. `explore_lite`: 기본 frontier 탐색

**한 문장 메모:** 지도에서 미지 영역의 경계를 찾고, 가까우면서 경계가 큰 후보를 골라 이동한다. 목표에 도착하거나 지도가 바뀌면 다시 고른다.

| 내가 확인할 것 | 공부 메모 | 메모를 만든 위치 |
| --- | --- | --- |
| 입력 | 현재 로봇 위치와 탐색용 costmap이 필요하다. 지도 안에서 시작할 자유 셀을 찾고, 주변 자유 공간을 따라 frontier를 찾는다. | [GitHub `frontier_search.cpp`, `searchFrom`](https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/frontier_search.cpp#L23-L70), [GitHub `explore.cpp`, `makePlan`](https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/explore.cpp#L212-L218) |
| 선택 | `frontierCost = 거리 가중치 × 거리 − 이득 가중치 × frontier 크기`다. 점수가 낮은 후보를 선호한다. 여기서 크기는 실제 관측 면적이 아니라 frontier의 길이/셀 수에 가까운 대리값이다. | [GitHub `frontier_search.cpp`, `frontierCost`](https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/frontier_search.cpp#L176-L181), [frontier 논문 2쪽, 원리](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/yamauchi_frontiers.pdf#page=2) |
| 출력 | 예제 전체 노드는 고른 frontier의 중심을 `NavigateToPose` 목표로 Nav2에 직접 보낸다. 우리 통합에서 후보 선택 기능만 재사용하려면 이 주행 발행 경로를 분리해야 한다. | [GitHub `explore.cpp`, 목표 작성·`async_send_goal`](https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/explore.cpp#L278-L305), [D-05](../design/algorithm-decisions.md) |
| 실패·종료 | 목표 진전이 없으면 blacklist에 넣고 다른 후보를 찾는다. frontier가 없거나 전부 blacklist에 들어가면 탐색 완료 상태를 발행하고 멈춘다. 이 완료는 모든 공간·대상을 확인했다는 증거가 아니다. | [GitHub `explore.cpp`, `makePlan`의 빈 목록·blacklist·timeout](https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/explore.cpp#L212-L266), [결과 처리](https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/explore.cpp#L346-L383) |

**읽는 순서:** [Yamauchi 논문 1~2쪽](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/yamauchi_frontiers.pdf#page=1)의 frontier 그림 → [패키지 README의 Autonomous exploration·RUNNING](https://github.com/robo-friends/m-explore-ros2#autonomous-exploration) → 위 표의 `searchFrom`, `frontierCost`, `makePlan`만 읽는다. `map_merge`는 AMR1 단독 탐색 공부에서 건너뛰어도 된다.

### 2. `frontier_exploration_ros2`: WFD와 방문 순서

**한 문장 메모:** frontier를 WFD로 찾고, 지도 전처리로 불필요한 후보를 줄인 뒤, 거리와 예상 탐색 이득을 이용해 여러 후보의 방문 순서를 고른다. 실행할 때는 첫 목표로 이동하고 새 지도를 받아 다시 계획한다.

| 내가 확인할 것 | 공부 메모 | 메모를 만든 위치 |
| --- | --- | --- |
| 입력 | occupancy map, 전역·지역 costmap, `map`에서 로봇 기준 frame으로 가는 TF, Nav2 서버를 기대한다. WFD는 알려진 영역의 연결을 먼저 훑고, frontier 셀을 발견하면 두 번째 BFS로 한 덩어리로 묶는다. | [GitHub README, Architecture·Runtime](https://github.com/mertgulerx/frontier_exploration_ros2#architecture), [Algorithm and Mathematics의 WFD-Style Two-Level Search](https://github.com/mertgulerx/frontier_exploration_ros2#algorithm-and-mathematics), [WFD 논문 3쪽, III. Methodology·Fig. 4](https://arxiv.org/pdf/1806.03581#page=3) |
| 선택 | 먼저 bilateral filter와 dilation으로 탐색 **판단용 지도**를 다듬는다. 이후 frontier 크기를 정보 이득의 대리값으로, 이동 비용과 센서 범위를 비용 요소로 삼아 MRTSP 방식의 방문 순서를 정한다. 저장소는 greedy와 제한 범위 DP를 제공한다. 전처리된 판단 지도는 원본 SLAM 지도를 안전하게 수정했다는 뜻이 아니다. | [2025 논문 2~4쪽, Map optimization·Frontier detection](https://www.nature.com/articles/s41598-025-97231-9.pdf#page=2), [같은 논문 5쪽, Exploring sequence optimization·식 (7)](https://www.nature.com/articles/s41598-025-97231-9.pdf#page=5), [GitHub README, Decision-Map Optimization·MRTSP Cost Matrix](https://github.com/mertgulerx/frontier_exploration_ros2#algorithm-and-mathematics) |
| 출력 | 계산한 순서의 첫 frontier 주변에서 주행 가능한 자유 셀을 골라 `NavigateToPose` 목표 하나를 보낸다. 다음 지도 갱신 뒤 재계획한다. 공개 C++ core를 분리해 후보만 연결할 가능성이 있으나 우리 `rescue_robot` 연동은 미구현이다. | [GitHub README, Frontier Candidate Geometry](https://github.com/mertgulerx/frontier_exploration_ros2#frontier-candidate-geometry), [Nav2 Integration](https://github.com/mertgulerx/frontier_exploration_ros2#nav2-integration), [Reusing the C++ Core Library](https://github.com/mertgulerx/frontier_exploration_ros2#reusing-the-c-core-library) |
| 실패·종료 | 문서에는 막힌 목표 건너뛰기, 반복 실패 frontier의 일시 억제, stop 제어, frontier 소진 이벤트가 있다. 각 기능의 실제 동작과 취소 후 실제 정지는 우리 환경에서 별도 검증해야 한다. | [GitHub README, Preemption and Blocked-Goal Design](https://github.com/mertgulerx/frontier_exploration_ros2#preemption-and-blocked-goal-design), [Runtime Control](https://github.com/mertgulerx/frontier_exploration_ros2#runtime-control), [Completion Event Consumption](https://github.com/mertgulerx/frontier_exploration_ros2#completion-event-consumption) |

**읽는 순서:** [WFD 논문 1·3쪽](https://arxiv.org/pdf/1806.03581#page=3)에서 frontier와 두 BFS → [2025 논문 2~5쪽](https://www.nature.com/articles/s41598-025-97231-9.pdf#page=2)에서 전처리와 방문 순서 그림·식 → [GitHub Algorithm and Mathematics](https://github.com/mertgulerx/frontier_exploration_ros2#algorithm-and-mathematics)의 같은 항목을 대조한다. 처음에는 수식 계수를 외우지 말고 **후보를 찾는 단계와 방문 순서를 정하는 단계가 다르다**는 점을 잡는다. 저장소의 제한 범위 DP는 이 패키지의 확장으로 보고 논문의 실험 결과와 동일시하지 않는다.

### 3. `frontier_toolbox`: 같은 후보에 다른 점수 붙이기

**한 문장 메모:** frontier 후보를 찾은 뒤 거리·예상 정보량·군집 크기 중 어떤 규칙을 쓰느냐에 따라 선택 목표가 달라지는지 공부하기 좋은 도구 모음이다.

| 내가 확인할 것 | 공부 메모 | 메모를 만든 위치 |
| --- | --- | --- |
| 입력 | `OccupancyGrid`와 로봇 위치를 받아 frontier 후보를 찾는다. `findFrontiers`는 미지 셀이 자유 셀에 붙어 있는지를 확인하며, DBSCAN을 켜면 가까운 후보를 군집으로 묶는다. | [GitHub 사용 문서, Using the Available Libraries](https://github.com/Schelbert197/frontier_toolbox/blob/main/use.md#using-the-available-libraries), [GitHub `frontier_helper.cpp`, `findFrontiers`](https://github.com/Schelbert197/frontier_toolbox/blob/main/frontier_exp_cpp/src/frontier_helper.cpp), [GitHub `frontier_lc.cpp`, `/map` 구독](https://github.com/Schelbert197/frontier_toolbox/blob/main/frontier_exp_cpp/src/frontier_lc.cpp) |
| 선택 | `selectByDistance`는 가까운 후보, `scoreByEntropy`는 지도 불확실성 감소 예상, `scoreByFlipCount`는 관측으로 알려질 미지 셀 수 예상, DBSCAN은 군집 단위 선택에 쓰인다. 이는 **선택 규칙의 메뉴**이며 네 가지를 한꺼번에 쓰는 고정 알고리즘은 아니다. | [GitHub 사용 문서, 함수 목록](https://github.com/Schelbert197/frontier_toolbox/blob/main/use.md#using-the-available-libraries), [GitHub `frontier_helper.cpp`, `selectByDistance`·`scoreByEntropy`·`scoreByFlipCount`](https://github.com/Schelbert197/frontier_toolbox/blob/main/frontier_exp_cpp/src/frontier_helper.cpp), [GitHub `frontier_lc.cpp`, 설정 `use_naive`·`use_clustering`·`use_entropy_calc`](https://github.com/Schelbert197/frontier_toolbox/blob/main/frontier_exp_cpp/src/frontier_lc.cpp) |
| 출력 | 예제 노드는 설정에 따라 `/goal_pose`를 발행하거나 `jackal_nav_to_pose` 서비스에 좌표를 보낸다. 코어 함수는 선택된 frontier 셀을 반환한다. 서비스 이름·frame이 우리 AMR1과 맞는지 검증하고, 협업 통합에서는 후보만 `rescue_robot`에 넘겨야 한다. | [GitHub `frontier_lc.cpp`, `publishGoalFrontier`](https://github.com/Schelbert197/frontier_toolbox/blob/main/frontier_exp_cpp/src/frontier_lc.cpp), [같은 파일 `publishToNav2ActionClient`·`publishToNav2PlannerServer`](https://github.com/Schelbert197/frontier_toolbox/blob/main/frontier_exp_cpp/src/frontier_lc.cpp), [D-05](../design/algorithm-decisions.md) |
| 실패·종료 | frontier나 유효 군집이 없으면 이 예제 노드는 새 목표 발행을 건너뛴다. 금지 영역·정체 시 재선택 코드도 보인다. 이것만으로 팀 계약의 탐색 완료 보고·goal 취소·실제 정지가 구현됐다고 판정하지 않는다. | [GitHub `frontier_lc.cpp`, `publishGoalFrontier`·`replanIfStuck`](https://github.com/Schelbert197/frontier_toolbox/blob/main/frontier_exp_cpp/src/frontier_lc.cpp), [GitHub 사용 문서, lifecycle 기동](https://github.com/Schelbert197/frontier_toolbox/blob/main/use.md#running-a-full-simulation) |

**읽는 순서:** [사용 문서의 함수 목록](https://github.com/Schelbert197/frontier_toolbox/blob/main/use.md#using-the-available-libraries) → [`frontier_helper.cpp`](https://github.com/Schelbert197/frontier_toolbox/blob/main/frontier_exp_cpp/src/frontier_helper.cpp)에서 `findFrontiers`, `selectByDistance`, `scoreByEntropy`, `scoreByFlipCount` 검색 → [`frontier_lc.cpp`](https://github.com/Schelbert197/frontier_toolbox/blob/main/frontier_exp_cpp/src/frontier_lc.cpp)에서 `selectGoal`, `publishGoalFrontier` 검색. 이 후보는 특정 논문의 공식 구현이라고 확인되지 않았으므로 기본 frontier 논문은 원리 학습에만 쓴다.

### 세 후보를 비교할 때 적을 한 문장

| 비교 질문 | 1번 | 2번 | 3번 |
| --- | --- | --- | --- |
| frontier를 찾은 뒤 무엇을 바꾸는가? | 거리와 frontier 크기의 단순 점수 | 지도 전처리와 여러 후보의 방문 순서 | 거리·entropy·미지 셀 수·군집 크기 등 점수 함수 |
| 원본 예제가 주행까지 하는가? | `NavigateToPose` 직접 전송 | `NavigateToPose` 직접 전송 | `/goal_pose` 또는 `jackal_nav_to_pose` 사용 |
| 공부 뒤 확인할 핵심 | 단순 점수로 충분한가? | 긴 이동·재방문이 줄어드는가? | 어느 점수 함수가 우리 환경에 맞는가? |

이 표의 마지막 행은 **시험 질문**이다. 어느 방식이 실제 AMR1에서 더 빠르다는 측정 결과가 아니다.

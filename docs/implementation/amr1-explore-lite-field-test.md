---
title: AMR1 explore_lite 단독 시험 절차와 결과 기록
status: test-plan-not-run
updated: 2026-10-12
---

# AMR1 explore_lite 단독 시험 절차와 결과 기록

[사용자 요구, 2026-10-12] “지금 테스트 하는거에 대해서 git에 먼저 올리고 테스트 해보고 그거에 대한 피드백을 올린 pr밑에다 달면 어때?”에 따라 실행 절차를 먼저 저장하고, 실행한 커밋의 결과를 이 문서를 추가한 PR 댓글에 남긴다. [AMR1 자율 탐색 후보](amr1-exploration-candidates.md)([PR #116](https://github.com/jaebeom/ROKEY-7/pull/116))의 첫 기준 후보 `explore_lite`를 대상으로 한다. 최종 알고리즘 채택이나 프로젝트 주행 adapter 구현을 뜻하지 않는다.

[사용자 설정, 2026-10-12] 이번 시험 대상은 `robot6`, `ROS_DOMAIN_ID=6`이며 SLAM/Nav2는 아직 시작하지 않았다고 보고했다. 이는 이번 회차의 설정이며 논리 AMR1의 영구 배치나 팀 공통 프로파일을 확정하지 않는다.

## 현재 확인한 범위

| 항목 | 상태 / 근거 |
| --- | --- |
| PC의 ROS 2 Jazzy, TurtleBot4 navigation/viz, explore_lite | 설치 경로와 `ros2 pkg executables explore_lite`를 확인함. 실행 파일 `explore` 존재 |
| TurtleBot4 launch 인자 | `slam.launch.py`, `nav2.launch.py`, `view_navigation.launch.py`의 `--show-args` 성공. 노드를 시작하지 않음 |
| 탐색 소스 | 로컬 `m-explore-ros2` HEAD `326cf8a0b487c34246bb8f3326afbcd69576dc60` 확인. 시험 전에 dirty 여부와 설치 바이너리 해시를 별도 기록해야 함 |
| 자동 시작과 중단 | 로컬 소스 정적 확인: 생성자에서 `makePlan()`을 즉시 호출. `explore/resume=false`는 timer 중단과 Nav2 전체 goal 취소를 요청함. 실제 정지는 미검증 |
| E3 격리 ROS/시뮬레이션 | `NOT_RUN` — 이번 세션에서 증거 미확인 |
| E4 단독 실기 | `NOT_RUN` — ROS 그래프 연결, 주행, 정지와 성능은 아직 시험하지 않음 |

현재 소스 확인 위치는 `explore/src/explore.cpp`의 `Explore::Explore`, `Explore::stop`, `Explore::resumeCallback`, `explore/config/params.yaml`이다. 설치본과 소스가 같은 빌드인지 HEAD만으로 증명하지 않는다. 설치 기본 설정은 `return_to_init=true`이므로 아래 시험 명령은 이를 `false`로 명시한다.

## 실행 전 관문

- [시험·증거 기준](../process/acceptance-plan.md)과 [현장 실행 관문](../process/field-runbook.md)을 따른다. [AMR1 자율 탐색 후보](amr1-exploration-candidates.md)(PR #116)의 E3→E4 순서를 유지한다. E3의 목표 발행·취소 동작 증거가 없으면 먼저 격리 시험하고, 그 상태를 이 PR에 남긴다. 이 문서 게시만으로 실기 준비가 완료되지 않는다.
- 현장에서 한 사람이 robot6를 관찰하고 정지 수단을 사용할 수 있어야 한다. 팀에 사용 시작·종료를 알리고, 다른 PC의 SLAM·탐색·주행 발행기가 같은 로봇에 연결되지 않았는지 확인한다.
- 단독 시험에서는 upstream 탐색 노드가 Nav2 goal을 직접 발행한다. `rescue_robot`의 임무 실행기, 다른 탐색 노드 및 RViz 목표 발행과 함께 사용하지 않는다. 협업 통합에서는 [D-05](../design/algorithm-decisions.md)의 `rescue_robot` 전용 goal/cancel 경계를 유지한다.
- 로봇이 도킹 상태라면 기존 운영 절차로 언도크하고 실제 상태를 확인한다. 이 문서는 언도크·도킹 명령이나 로봇 설정 변경을 수행하지 않는다.
- 시험 공간, 종료 조건, 관찰자, 속도 설정, 반복 수와 사용할 판정 기준을 실행 전에 기록한다. 첫 관찰 회차의 동작 확인을 탐색 성능 합격으로 확대하지 않는다.

## 터미널 공통 환경

각 터미널에서 먼저 실행한다. `TB4_WS`는 이미 설치된 TurtleBot4 워크스페이스의 실제 경로를 사용자가 지정한다. 로봇 Pi에 패키지를 설치하거나 Discovery 설정을 바꾸지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
read -r -p 'TurtleBot4 workspace path: ' TB4_WS
source "$TB4_WS/install/setup.bash"
export ROS_DOMAIN_ID=6
export AMR_NS=robot6
```

다른 회차는 도메인과 namespace를 모두 바꿔 확인한다. DDS/Discovery 환경은 기존에 실습에 성공한 환경을 사용하고 실제 값을 회차 기록에 남긴다. 다른 도메인의 ROS CLI daemon 결과가 섞이면 daemon을 종료하고 다시 조회한다.

## 1. 읽기 전용 연결 확인

```bash
ros2 node list
ros2 topic info "/$AMR_NS/scan" --verbose
timeout 10s ros2 topic echo "/$AMR_NS/scan" --once --field header
```

실제 scan 수신과 frame을 확인한다. 토픽 목록에 이름이 보이는 것만으로 수신 성공을 판정하지 않는다. timeout/수신 실패 시 자동 탐색을 시작하지 않고 로그와 종료 코드를 기록한다.

## 2. SLAM, Nav2, RViz 기동

각 명령은 별도 터미널에서 실행한다. 기존 프로세스가 떠 있으면 중복 실행하지 않는다. 저장 지도 localization/AMCL은 이 SLAM 시험과 동시에 시작하지 않는다.

```bash
# 터미널 1
ros2 launch turtlebot4_navigation slam.launch.py \
  namespace:="$AMR_NS" use_sim_time:=false
```

```bash
# 터미널 2
ros2 launch turtlebot4_navigation nav2.launch.py \
  namespace:="$AMR_NS" use_sim_time:=false
```

```bash
# 터미널 3
ros2 launch turtlebot4_viz view_navigation.launch.py \
  namespace:="$AMR_NS" use_sim_time:=false
```

RViz에서 지도, scan, 로봇 위치가 연결되는지 확인하고, 다른 터미널에서 아래 결과를 기록한다. `map`과 `base_link`는 설치된 기본 frame이며 실제 입력과 다르면 탐색을 실행하지 말고 설정을 검토한다. 토픽 namespace와 TF frame 이름은 별개다.

```bash
ros2 topic info "/$AMR_NS/map" --verbose
timeout 10s ros2 topic echo "/$AMR_NS/map" --once --field header \
  --qos-durability transient_local --qos-reliability reliable
timeout 10s ros2 run tf2_ros tf2_echo map base_link --ros-args \
  -r /tf:="/$AMR_NS/tf" -r /tf_static:="/$AMR_NS/tf_static"
ros2 lifecycle get "/$AMR_NS/bt_navigator"
ros2 action info "/$AMR_NS/navigate_to_pose"
```

TF 출력 확인은 마지막 timeout 종료 코드만으로 판정하지 않고, 연속으로 출력된 변환 값을 저장한다. `bt_navigator` 가 active 이고 Nav2 action server 를 쓸 수 있는지도 확인한다. 이것이 실기의 정지를 보장하지는 않는다.

## 3. 중단용 터미널을 준비한 뒤 자동 탐색

중단용 터미널에 아래 명령을 준비한다. 탐색 시작 전에 보낸 false는 시작 대기를 보장하지 않는다. **탐색 노드 시작 후의 중단 명령**으로 사용한다.

```bash
ros2 topic pub --once "/$AMR_NS/explore/resume" \
  std_msgs/msg/Bool '{data: false}'
```

현장 관찰자가 준비됐을 때만 별도 터미널에서 아래 명령을 실행한다. **시작 직후 목표를 보내 로봇이 움직일 수 있다.**

```bash
EXPLORE_SHARE="$(ros2 pkg prefix --share explore_lite)"
ros2 run explore_lite explore --ros-args \
  --params-file "$EXPLORE_SHARE/config/params.yaml" \
  -r __ns:="/$AMR_NS" \
  -r /tf:="/$AMR_NS/tf" -r /tf_static:="/$AMR_NS/tf_static" \
  -p use_sim_time:=false -p return_to_init:=false
```

지도가 커지면서 다음 목표를 자동 선택하는지 관찰한다. 막힌 목표·정체·반복 방문·오류와 수동 개입을 기록한다. Frontier 소진은 모든 공간 또는 구조 대상 확인 완료의 증거가 아니다.

중단용 명령을 실행한 뒤 Nav2 취소 결과와 실제 바퀴 정지를 확인한다. false는 해당 Nav2 서버의 모든 goal 취소를 요청하므로 다른 주행 발행자가 있어서는 안 된다. 취소 요청이나 `Exploration stopped` 로그만으로 물리 정지를 판정하지 않는다. 정지 미확인 시 `STOP_UNCONFIRMED`로 기록하고 현장 물리 정지 수단을 사용한다.

실제 정지 확인 후 탐색 터미널을 Ctrl+C로 종료한다. SLAM/Nav2/RViz는 각각 자신의 터미널에서 종료한다. 재개는 관찰자가 원인·현장 상태를 확인한 뒤 별도 회차로 기록한다.

## 4. 실행본과 증거 기록

시험 전에 아래를 기록한다. `REPO`와 `EXPLORE_SRC`는 사용자가 선택한 실제 체크아웃 경로이다. 실제 적용된 SLAM/Nav2/explore 파라미터도 함께 저장한다.

```bash
git -C "$REPO" rev-parse HEAD
git -C "$REPO" status --short
git -C "$EXPLORE_SRC" rev-parse HEAD
git -C "$EXPLORE_SRC" status --short
EXPLORE_PREFIX="$(ros2 pkg prefix explore_lite)"
sha256sum "$EXPLORE_PREFIX/lib/explore_lite/explore"
ros2 param dump "/$AMR_NS/explore_node"
```

첫 기록에는 날짜·시각(KST), run ID, 시험한 문서 커밋, upstream 소스 SHA/dirty, 실행 바이너리 해시, 실제 설정과 명령, 시작·중단 시각, 지도/영상/로그 위치·해시를 남긴다. bag·영상·생성 지도·설치 폴더는 이 절차 PR에 일괄 추가하지 않는다. 공유 로그의 개인 경로·주소·비밀값은 제거한다.

## PR 결과 댓글 양식

실행 뒤 이 문서를 추가한 PR 아래에 원문 결과를 남긴다. 실행하지 않은 항목은 `NOT_RUN`, 환경 때문에 못 하면 `BLOCKED`, 실행했으나 기대 동작을 충족하지 못하면 `FAILED`로 구분한다.

```text
**송신:** 로봇·지도(Codex) → **수신:** 작전·총괄(Claude), 검증·수정·머지(Cursor)
**커밋:** <시험한 문서 SHA> · **종류:** 기록

- run_id / KST 시각 / 관찰자:
- robot namespace / ROS_DOMAIN_ID: robot6 / 6
- E3 선행 시험 증거:
- 문서 SHA / upstream SHA·dirty / 설치 바이너리 해시:
- 실제 SLAM/Nav2/explore 파라미터와 실행 명령:
| 확인 항목 | 상태 | 실제 관찰 / 로그 |
| 지도·scan·TF / Nav2 준비 | NOT_RUN | |
| 자동 목표 선정과 이동 | NOT_RUN | |
| 지도 확장과 목표 재선정 | NOT_RUN | |
| 막힌 목표·정체 처리 | NOT_RUN | |
| 중단 요청과 goal 취소 | NOT_RUN | |
| 실제 바퀴 정지 | NOT_RUN | |
| 탐색 종료 사유 | NOT_RUN | |
- 실패·수동 개입 / 원문 오류:
- 로그·영상·지도 참조와 SHA256:
- 피드백 / 다음 회차에서 하나만 바꿀 항목:
- 미측정: 탐색 비율·시간·거리 등 성능값, 두 AMR·네 PC 통합
```

이 PR의 문서 검사 통과와 향후 실기 결과는 별도 증거다. 프로젝트 통합 adapter 및 최종 알고리즘 채택은 후속 검토 대상이다.

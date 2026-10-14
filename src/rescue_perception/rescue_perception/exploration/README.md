# explore_lite 단독 실기 인계

> **현재 관계 (2026-10-14, main 4163a25, OPS-210):** 이 문서는 explore_lite 를 혼자 띄워 보는 단독 실험용이다. 현장 실행은 [rescue_bringup README](../../../rescue_bringup/README.md)를 따른다. 현장에서는 조정기가 `search_mode` FRONTIER 로 TB-6 탐색 목표를 직접 고른다(`rescue_coordination/ros_node.py` 가 `rescue_contracts.grid_search.find_frontiers` 로 고르고, FRONTIER 작업을 로봇 실행기로 보냄). 현장 파일 `config/field/search_area.field.json` 의 `search_mode` 가 FRONTIER 이고, `field_launch.py` 는 그 값과 `executable_bases` 의 FRONTIER 를 검사한다. rescue_bringup 의 launch 는 explore_lite 를 띄우지 않는다(`src/rescue_bringup` 에서 `explore_lite` 를 찾아 없음을 확인). 아래 구성을 현장 흐름과 같이 띄우면 Nav2 목표를 두 곳이 보내게 된다.

[사용자 요구, 2026-10-13] “1번에 대해서 slam, nav, 알고리즘까지 깃에 올리고 그거를 팀원이 다운받아서 사용할 수 있도록 하면 좋을거 같아.” 설정과 실행 파일 위치는 사용자가 지정한 이 폴더다. 단독 실기 인계용이며 `rescue_perception`의 Python 노드나 설치 진입점을 바꾸지 않는다.

이 구성은 SLAM으로 지도를 만들고, Nav2가 경로와 속도를 계산하며, explore_lite가 탐색 목표를 고른다. upstream 탐색기는 Nav2 액션을 직접 발행한다. 팀 협업 실행기와 동시에 사용하지 않는다. 협업 배포는 [D-05](../../../../docs/design/algorithm-decisions.md)의 frontier 후보 → rescue_robot 연결이 별도로 필요하다.

## 내려받기와 빌드

PC의 Ubuntu 24.04 / ROS 2 Jazzy 환경에서 실행한다. 로봇 Pi에 설치하는 명령이 아니다.

```bash
git clone --branch feat/explore-lite-team-handoff https://github.com/jaebeom/ROKEY-7.git
cd ROKEY-7
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths \
  src/vendor/m-explore-ros2 \
  src/vendor/turtlebot4/turtlebot4_navigation \
  src/vendor/turtlebot4_desktop/turtlebot4_viz \
  --ignore-src --rosdistro jazzy -r -y
colcon build --base-paths src/vendor --symlink-install --packages-select \
  explore_lite_msgs explore_lite turtlebot4_navigation turtlebot4_viz \
  --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
```

`rosdep`는 미리 설치·초기화되어 있어야 한다. `turtlebot4_description`은 ROS 시스템 패키지로 설치된다. 지도 작성에 쓰지 않는 예제 지도와 로봇 메시 원본은 복사하지 않았다. Slam Toolbox/Nav2는 ROS 의존성이고, 이 저장소에는 TurtleBot4 실행 구성 및 탐색 C++ 소스가 포함된다.

기존 작업 폴더에서는 다른 사람의 브랜치를 전환하지 말고 별도 clone 또는 worktree를 쓴다. 새 터미널마다 아래 설정을 반복한다. `ROBOT_NS`, `ROS_DOMAIN_ID`, DDS 환경은 팀이 배정한 로봇과 현재 네트워크에 맞춰 지정한다. [현재 네트워크 안내](../../../../docs/design/network-discovery.md)를 참고하고 다른 워크스페이스의 setup을 섞지 않는다.

```bash
# 저장소 루트에서, 현장 배정값으로 먼저 export한 뒤 실행한다.
: "${ROBOT_NS:?배정한 로봇 이름을 설정하세요}"
: "${ROS_DOMAIN_ID:?현장 도메인을 설정하세요}"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_SUPER_CLIENT=True
# ROS_DISCOVERY_SERVER는 현장에서 확인한 서버 ID/주소 목록을 그대로 사용한다.
export EXPLORE_CONFIG_DIR="$PWD/src/rescue_perception/rescue_perception/exploration"
ros2 pkg prefix explore_lite
ros2 pkg prefix turtlebot4_navigation
ros2 pkg prefix turtlebot4_viz
```

세 패키지 경로가 이 checkout의 install 아래인지 확인한다. 이 폴더의 launch는 파일 경로로 실행하므로 `rescue_perception`을 빌드할 필요가 없다.

## 실행 순서

1. 담당자가 로봇 번호·현장 관찰·단독 제어권을 확인하고 언도킹한다. 이전 explore/SLAM/Nav2/RViz는 각 실행 터미널의 Ctrl+C 또는 확인한 PID만 종료한다. 지도 발행자는 한 개여야 한다.
2. 터미널 1에서 새 SLAM + Nav2 + RViz를 실행한다. 저장 지도를 읽지 않는 새 SLAM이다.

```bash
ROS_SUPER_CLIENT=False ros2 launch "$EXPLORE_CONFIG_DIR/mapping.launch.py" \
  namespace:="$ROBOT_NS"
```

3. 터미널 2에서 실제 수신과 상태를 확인한다. Publisher count만으로 통과시키지 않는다. 센서/TF 시각은 현재 시각이어야 하고 dock 상태는 false여야 한다. 도킹 중 센서가 꺼진 장비는 먼저 담당자가 언도킹한다.

```bash
ros2 topic echo --once "/$ROBOT_NS/dock_status"
ros2 topic echo --once "/$ROBOT_NS/scan" --field header
ros2 topic echo --once "/$ROBOT_NS/map" --field header
ros2 topic echo --once "/$ROBOT_NS/odom"
ros2 run tf2_ros tf2_echo map base_link --ros-args \
  -r /tf:="/$ROBOT_NS/tf" -r /tf_static:="/$ROBOT_NS/tf_static"
# TF 확인 뒤 Ctrl+C
for node in slam_toolbox controller_server planner_server bt_navigator behavior_server collision_monitor; do
  ros2 lifecycle get "/$ROBOT_NS/$node"
done
```

모두 active가 되어야 한다. SLAM만 inactive이면 그 노드에 한해 `ros2 lifecycle set "/$ROBOT_NS/slam_toolbox" activate` 후 재확인한다. Nav2가 준비되지 않았거나 입력 수신이 없으면 탐색을 실행하지 않는다. odom 선속도·각속도와 현장 관찰로 실제 정지를 확인한다.

4. 기록할 경우 별도 터미널에서 먼저 실행한다. 같은 출력 폴더는 재사용하지 않는다.

```bash
ros2 bag record -o "log/explore_$(date +%Y%m%d_%H%M%S)" \
  "/$ROBOT_NS/map" "/$ROBOT_NS/scan" "/$ROBOT_NS/odom" \
  "/$ROBOT_NS/tf" "/$ROBOT_NS/tf_static" "/$ROBOT_NS/explore/status"
```

5. 준비 확인과 현장 관찰 후 터미널 3에서 탐색을 시작한다. 이 명령 직후 로봇이 움직일 수 있다.

```bash
ros2 launch "$EXPLORE_CONFIG_DIR/explore.launch.py" namespace:="$ROBOT_NS"
```

## 중단과 지도 저장

```bash
ros2 topic pub --once "/$ROBOT_NS/explore/resume" std_msgs/msg/Bool '{data: false}'
ros2 topic echo "/$ROBOT_NS/odom" --field twist.twist
```

중단 요청 전송은 실제 정지 증명이 아니다. 새 odom에서 선속도·각속도가 0에 가깝고 위치가 변하지 않는지 현장과 함께 확인한다. 수신이 끊기거나 멈추지 않으면 현장 정지 수단을 사용한다. 정지 확인 후 탐색 터미널을 Ctrl+C로 종료한다. upstream 탐색기는 실기 감독기의 자동 timeout·입력 끊김 감시 기능을 포함하지 않으므로 담당자가 계속 관찰하고 필요한 때 중단한다.

SLAM을 끄기 전에 지도를 저장한다.

```bash
mkdir -p log/maps
ros2 run nav2_map_server map_saver_cli -t "/$ROBOT_NS/map" \
  -f "log/maps/explore_$(date +%Y%m%d_%H%M%S)" --fmt pgm \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true
```

기록 터미널을 Ctrl+C로 종료해 bag을 마무리하고, 마지막에 mapping 터미널을 종료한다. 도킹·언도킹은 담당자가 수행한다. `return_to_init=false`이므로 자동 복귀나 자동 도킹은 하지 않는다.

## 설정과 변경 범위

[설정, 2026-10-13] `nav2.yaml`은 run-014에서 적용한 파일과 바이트가 같다. local/global inflation_radius는 둘 다 0.25m, cost_scaling_factor는 4.0, 최대 전진 속도는 0.26m/s, 최대 회전 속도는 1.0rad/s다. `explore.yaml`도 해당 회차 파일과 같다. 상위 폴더의 기존 `nav2.yaml`과 별도 파일이므로 이 폴더의 파일을 수정한다. `slam.yaml`은 같은 TurtleBot4 SLAM 설정이다.

변경 전 복사와 SHA256을 남기고 한 번에 한 항목만 바꾼다. 바꾼 값은 재시작 뒤 `ros2 param get`으로 확인한다. 되돌릴 때는 백업 YAML로 복구한 뒤 정지 상태에서 다시 실행한다. 다른 로봇에 동일 값이 적합하다는 보증은 아니다.

외부 코드 출처·파일 해시는 [배포 manifest](../../../vendor/explore-lite-handoff-manifest.json)에 있다. upstream 참조 SHA와 로컬 복사본의 동일성은 미검증이며 배포한 파일 해시가 재현 기준이다. 원본 라이선스·저작권 고지를 보존했다.

[측정, 2026-10-13] 기존 explore_lite 단독 회차는 약 29.93초 뒤 알고리즘 종료를 보고했고 실제 정지를 확인했다. 이는 전체 공간 탐색 완료 증명이 아니다. 이 새 launch 묶음의 팀원 PC 실기, 자동 도킹, 두 로봇 협업은 NOT_RUN이다. 기존 기록은 [실기 기록 PR](https://github.com/jaebeom/ROKEY-7/pull/191)을 참고한다.

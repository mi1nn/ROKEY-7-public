# TB-5 출발 정렬 한 PC 리허설 (2026-10-15)

## 한눈에

TB-5 출발 정렬 노드(`rescue_start_align`)가 실제 AMCL(Adaptive Monte Carlo Localization, 고정 지도 위 위치 추정)과 함께 잰 출발값의 오차를 바로잡는지, 너무 많이 옮기면 막는지를 로봇 없이 확인했다. 종류는 E3(실제 노드, 가짜 센서)이다. 실기는 NOT_RUN 이다.

| 회차 | 실제 위치 (x, y, yaw) | 잰 값(AMCL 시작) | 결과 | 맞춘 위치 | 실제와 차이 | 갱신 횟수 |
| --- | --- | --- | --- | --- | --- | --- |
| 2 | (0.3, 0.0, 0.0) | (0.5, 0.0, 0.1745) 0.2 m·10° 틀림 | `ALIGNED` | (0.283, −0.010, 0.015) | 2.0 cm, 0.9° | 7 |
| 3 | (0.3, 0.0, 0.0) | (1.3, 0.0, 0.0) 1 m 틀림 | `FAILED START_ALIGN_JUMP_TOO_LARGE` | (0.301, −0.056, 0.013) | 이동 1.00 m 가 상한 0.5 m 초과 | 10 |
| 4 | (−0.5, 0.6, 0.5) | (−0.65, 0.7, 0.36) 0.18 m·8° 틀림 | `ALIGNED` | (−0.502, 0.598, 0.495) | 0.3 cm, 0.3° | 7 |
| 5 | (1.8, −1.0, −1.2) | (1.95, −1.12, −1.05) 0.19 m·9° 틀림 | `ALIGNED` | (1.766, −1.001, −1.196) | 3.4 cm, 0.2° | 8 |
| 6 | 2회차와 같음(저장소 `run.sh` 로 다시) | | `ALIGNED` | 2회차와 같은 값 | | 7 |

- 단위는 m, rad 이다. 스캔 일치 비율은 `ALIGNED` 회차 모두 1.0(360점 중 360점)이고 3회차는 0.95 이다.
- 3회차에서 AMCL 은 실제 위치를 찾았지만 첫 추정에서 1 m 옮긴 결과라서 막았다. 잰 값이 이만큼 틀리면 측정이나 놓은 자리를 다시 확인해야 한다는 뜻이다.
- 2회차와 6회차의 값이 같다. 같은 입력이면 이 AMCL 빌드는 같은 결과를 냈다.

## 구성

| 부분 | 무엇 | 출처 |
| --- | --- | --- |
| AMCL | `nav2_amcl` (Nav2 jazzy 브랜치 `645abd9` 소스 빌드, 이 PC 에 Nav2 가 없어서) | TurtleBot4 `localization.yaml` 값 + `field_launch.amcl_parameters` 로 잰 값을 시작점으로 |
| 출발 정렬 | `rescue_robot` `start_align` | `field_launch.frozen_localization_values` 가 낸 값 그대로(현장 파일 `robots.robot_b.start_align.*`) |
| 묶음 | `/tf`→`/robot5/tf`, `/tf_static`→`/robot5/tf_static` remap 안에 AMCL·정렬·lifecycle_manager | `frozen_localization.launch.py` 와 같은 묶음 |
| 가짜 세계 | 6 × 4 m 방(상자, 벽 조각, 선반이 있어 대칭이 아님), 0.05 m 칸. LiDAR(`rplidar_link`)는 `base_link` 에서 (−0.04, 0, 0.2), 90° 돌려 붙임. 360개 빔 5 Hz. `odom→base_link` 고정, 속도 0, 언독 | `scripts/field/start-align-rehearsal/fake_world.py` |
| 네트워크 | Discovery Server 없음, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, 도메인 47 | `run.sh` |

다시 돌리는 방법은 아래와 같다.

```bash
bash scripts/field/start-align-rehearsal/run.sh /opt/ros/jazzy/setup.bash <작업공간>/install/setup.bash \
  <nav2_amcl 작업공간>/install/setup.bash -- <결과 폴더> "0.3 0.0 0.0" "0.5 0.0 0.1745"
```

## 이 리허설에서 찾은 것

- **퍼짐 0 시작 확인.** 잰 값으로 시작한 AMCL 의 첫 `amcl_pose` 공분산은 x·y 0.0, yaw −5.7e−14 였다. Nav2 소스(`amcl_node.cpp:283-295`)대로 파티클이 한 점에 모여 있다. 이 상태에서는 서 있는 동안 잰 값의 오차가 그대로 남는다.
- **0 아래 반올림 값.** yaw 분산 −5.7e−14 를 첫 구현이 「공분산이 아님」으로 버려 정렬이 시작되지 않았다(1회차, 시간 초과). 1e−9 아래의 음수는 0 으로 보게 고쳤다. 지금 `live_sources.receive_pose` 의 유효 판정에도 `0<=covariance[35]` 조건이 있다. 이 PR 에서는 고치지 않았다. TB-5 위치 유효 조건을 다루는 다음 PR 에서 고친다.
- AMCL 은 `set_initial_pose` 서비스로 받은 시작 자세의 odom 보정 단계에서 `Failed to transform initial pose in time` 경고를 한 번 냈다(시각 0 을 보냄). 로봇이 서 있으므로 결과에는 영향이 없다.

## 하지 않은 것

- 실기(E4) NOT_RUN. 실제 LiDAR 잡음, 사람·다른 로봇이 스캔에 들어오는 경우, 실제 고정 지도 품질은 보지 않았다.
- `START_ALIGN_LOW_MATCH` 는 실제 AMCL 로 만들지 못했고 단위시험(E2)으로만 확인했다.
- 고정 지도 중계(`rescue_frozen_map_relay`)는 띄우지 않았다. 지도는 가짜 세계가 `/robot5/map` 에 바로 냈다.
- 값(퍼짐, 문턱, 상한)은 모두 [제안]이다.

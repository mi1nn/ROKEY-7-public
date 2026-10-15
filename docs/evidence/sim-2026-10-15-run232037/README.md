# 가제보 실습 둘째 회차 자료 (full-1015-232037, 2026-10-15)

[가제보 실습 기록](../sim-2026-10-15-gazebo-practice.md) 6절 둘째 회차의 원문이다. 통합-조정(C9)과 통합-로봇(C7·C8)이 원인을 보도록 올린다. 시뮬레이션 PC 의 회차 폴더에서 그대로 옮기고 색 코드를 지웠다. 저장소 경로는 `<저장소>`, 회차 폴더는 `<회차 폴더>`, ROS 로그 폴더는 `<ROS 로그>` 로 바꿨다. 빌드는 `src/` = main `97f2cef` 다. world 는 main 의 `config/sim/worlds/arena.sdf` 가 아니라 가제보 실습의 임시 world 파일이었다(2026-10-16 확인, `gz.log` 의 `Loading SDF world file`). 바깥 벽·칸막이는 같다. 다른 것은 두 가지다. 터널 2 가 덮개 0.30 m 가 아니라 막힌 벽이고, 가운데 기둥(1.80, 3.585)이 있다. 1회차(223913)도 같은 파일이다.

## 파일

| 파일 | 무엇 | 줄인 것 |
| --- | --- | --- |
| `maps/live-r0-27be82b2a9e5.pgm`·`.yaml` | TB-6 SLAM 지도, `EXPLORATION_COMPLETE`(23:25:28) 순간 저장본(운영 launch `map_snapshot`) | 없음 |
| `maps/frozen-r1-…`·`frozen-r2-…` | 23:25:32 1·2차 고정 지도(같은 내용) | 없음 |
| `logs/coordinator.log` | 조정기 로그 전부(회차 시작~끝) | 없음 |
| `logs/robot_robot_b.log` | TB-5 `robot.launch`(Nav2·실행기) 로그 전부 | 없음 |
| `logs/frozen_loc_b.log` | TB-5 `frozen_localization`(고정 지도 relay·AMCL·출발 정렬) | 1초마다 나오는 `Requesting no-motion update`·`Waiting for map` 줄 |
| `logs/frozen_loc_a.log` | 7.5 뒤 TB-6 `frozen_localization` | `Requesting no-motion update`·`Message Filter dropping` 줄 |
| `logs/tb6_slam.log` | TB-6 SLAM | `Message Filter dropping` 줄 |
| `logs/operator.log` | 시뮬레이션 운영자 대역(런북 7.1·7.5 대신) | 없음 |
| `logs/robot_robot_a.log` | TB-6 `robot.launch` 로그 전부(실행기, Nav2 `bt_navigator`·`controller_server`·`planner_server`·`behavior_server`, `collision_monitor`) | 없음 (2026-10-16 더함) |
| `logs/passage_robot_a.log` | TB-6 `passage_points`(깊이 → 통로 점) | 없음 (2026-10-16 더함) |
| `logs/overhead_robot_a.log` | TB-6 `overhead_passage`(깊이 → 머리 위 높이) | 없음 (2026-10-16 더함) |

2026-10-16 에 더한 세 파일도 같은 방법으로 옮겼다. ROS 로그 폴더 이름의 PC 이름은 `sim-pc` 로 바꿨다.

## 읽을 때

- **C9(탐색이 경계 2개 뒤 끝남):** `coordinator.log` 의 `FRONTIER none on map update 193~195` → `EXPLORATION_COMPLETE … frontiers_sent=2`(23:25:26~28)와 `maps/live-r0-27be82b2a9e5`. 이 빌드에는 #373(후보별로 빠진 이유) 로그가 없다. 작전 판단(2026-10-15): 첫 경계가 벽 쪽이던 것은 시뮬레이션 벽 관통(S10)으로 SLAM 벽 줄에 틈이 생긴 탓일 수 있다 — 「시뮬레이션 world(S10) 후보」. 이 판단의 근거였던 「첫 경계 중심이 남쪽 벽 바깥 약 0.4 m」는 틀렸다. 아래 「지도 좌표와 실습장 좌표」를 본다.

## 지도 좌표와 실습장 좌표 (2026-10-16 정정)

가제보 실습이 처음 낸 「첫 경계 중심이 남쪽 벽 바깥 약 0.4 m」는 계산 실수였다. 지도 원점을 언도킹 뒤 자세 (0.88, 0.72, 101°)로 잡았다. 실제 원점은 odom 원점, 즉 Gazebo 생성 자세 (0.88, 0.37, −90°)다. 정정 코멘트는 #377 에 있다.

- **근거 1:** `map_freeze` 로그가 세 회차 모두 `START_POSE captured robot_a map=(-0.353, -0.000, -2.94x)` 다. 언도킹 뒤 자세가 원점이었다면 (0, 0, 0) 근처여야 한다.
- **근거 2:** `maps/live-r0-27be82b2a9e5` 의 점유 칸 484개를 실습장 벽·상자에 맞춰 봤다. 생성 자세를 원점으로 두면 58%가 벽 0.1 m 안에 든다(중앙값 0.09 m). 언도킹 뒤 자세를 원점으로 두면 4%다(0.87 m). 생성 자세 둘레에서 6~8° 더 돌리면 83%까지 오른다(0.035 m).
- **근거 3:** 3회차(full-1015-235518) 확인용 정답 위치로 언도킹 뒤 방향은 89.5°, odom 은 101° 다. 언도킹 회전에서 odom 이 약 11° 더 돈 만큼 지도가 돌아 있다.
- **변환:** 실습장 좌표(x 동쪽, y 북쪽, 원점 로봇 구역 남서 모서리)는 대략 (0.88 + y_map, 0.37 − x_map)이다. 지도 그림에서 위쪽(+y_map)은 실습장 동쪽이고, 왼쪽(−x_map)은 북쪽이다. 아래 표의 범위는 세 변환의 차이다. 세 변환은 생성 자세, 벽 맞춤 −8°, 3회차 정답 위치 기준이다.

| 지도 좌표 | 실습장 좌표 | 어디 |
| --- | --- | --- |
| 첫 경계 e291 중심 (−1.30, −0.65) | (0.23~0.44, 1.67~1.80) | 로봇 구역 북서쪽. 서쪽 벽(x=0)에서 0.2~0.4 m, 칸막이(y=2.01) 남쪽 |
| 첫 목표 (−1.00, −0.74) | (0.14~0.28, 1.37~1.51) | 서쪽 벽 앞. 지도에서 가장 가까운 점유 칸까지 0.20 m(Nav2 로봇 반지름 0.189) |
| 둘째 경계 a116 중심 (−2.17, 1.84) | (2.72~3.05, 2.12~2.54) | 로봇 구역 **북동쪽** 터널 1(덮개 x 2.75~3.43, y 2.23) 자리. 지도 그림에서는 왼쪽 위(북서쪽처럼 보임) |
| 둘째 목표 (−1.43, 0.64) | (1.52~1.72, 1.65~1.80) | 로봇 구역 안, 흰 상자 남쪽 |

- 1회차에서 첫 목표로 간 TB-6 의 확인용 정답 위치는 (0.21, 1.47)이었다(23:03~23:06 측정). 서쪽 벽까지 0.21 m, 로봇 반지름을 빼면 약 0.02 m 다.
- 첫 경계가 서쪽 벽 앞에 왜 생겼는지는 모른다. 벽 관통(S10) 때문인지 이 자료로 가릴 수 없다.

## 둘째 경계 a116 의 24초 (23:25:02~23:25:26)

`robot_robot_a.log` 에서 읽었다. TB-6 위치는 Nav2 가 찍은 지도 좌표다. 이 회차는 확인용 정답 위치를 남기지 않았다.

| 시각 | 로그 | 뜻 |
| --- | --- | --- |
| 23:23:49.81 | `Begin navigating from current location (-0.35, -0.00) to (-1.00, -0.74)` | 첫 경계 목표로 출발 |
| 23:23:50~23:24:20 | `Robot to slowdown for 30.000000 percents due to PassageSlow polygon` 4회, `Failed to make progress` 3회(23:23:59.84, 23:24:09.98, 23:24:20.22) | 느리게 가다 진행 없음 판정 |
| 23:24:28.63~23:24:48.65 | `RegulatedPurePursuitController detected collision ahead!` 76회, `Controller patience exceeded` 11회, `Running spin` 3회 → `Goal failed` | 멈춤. 23:25:02 출발 위치로 보면 목표에서 5 cm 떨어진 자리 |
| 23:25:02.31 | `Begin navigating from current location (-1.00, -0.69) to (-1.43, 0.64)` | 둘째 목표로 출발. **첫 목표 자리에서 떠나지 못한 상태** |
| 23:25:02.35 | `detected collision ahead!` | 출발 20 ms 만에 충돌 앞 판정 |
| 23:25:02~23:25:25 | `detected collision ahead!` 97회, `Controller patience exceeded` 14회, `Running spin` 3회 모두 `Collision Ahead - Exiting Spin`, `Running wait` 3회, `Passing new path to controller` 1회(23:25:17.12) | 경로를 따라 나가지 못함 |
| 23:25:25.28 | `bt_navigator: Goal failed` | |
| 23:25:25.51 | `CommandLease STOPPING` | 실행기 정지 |
| 23:25:26.25 | 조정기 `RESULT task=fb5ea732… outcome=FAILED` | |

- **어디까지 갔나:** 둘째 목표 쪽으로 나가지 못했다. 터널 1 앞까지 가지 않았다. 24초 내내 첫 목표 자리(서쪽 벽 앞)에 있었다. 확인용 정답 위치가 없어서 이것은 Nav2 의 지도 좌표 기준이다.
- **실물에서도 날 것인가:** 실물에서도 날 수 있다 [추정].
  - 근거: TB-6 이 첫 목표 자리(지도에서 점유 칸까지 0.20 m, 로봇 반지름 0.189 m)에서 RPP 충돌 판정과 회전 충돌 판정에 막혔다. 이 판정은 Nav2 동작이고 시뮬레이션과 상관이 없다.
  - 근거: 1회차에 같은 목표로 간 정답 위치는 벽까지 약 0.02 m 였다.
  - 모름: 첫 경계가 서쪽 벽 앞에 생긴 원인. 벽 관통(S10)인지, 중계 지연(tf 95% 153 ms)이 판정에 끼었는지는 이 자료로 가릴 수 없다.
- 3회차부터는 확인용 정답 위치를 0.5초마다 남긴다(`gt_track.txt`, 어떤 노드에도 주지 않음).
- **C7(TB-6 relay 막힘):** 시뮬레이션 운영자 대역의 잘못으로 본다. `operator.log` 23:25:32 에 tb6_slam launch PID 에 SIGINT 를 보냈지만, 백그라운드로 띄운 launch 는 SIGINT 를 무시로 물려받아 SLAM 이 꺼지지 않았다. `tb6_slam.log` 의 slam_toolbox 종료 줄은 회차를 끈 23:37:23 하나뿐이다. 「발행자 0」(23:25:53)은 `--no-daemon --spin-time 3` 이라 발행자를 못 본 것으로 본다. 그래서 `frozen_loc_a.log` 23:25:59 `MAP_RELAY_BLOCKED` 는 맞는 판정이었고, 그 뒤 `MAP_RELAYED` 는 없다. 운영자 대역은 SIGTERM + slam_toolbox PID 종료 확인으로 고쳤다.
- **C8(TB-5 Nav2 가 active 가 안 됨):** `robot_robot_b.log` 의 `lifecycle_manager_navigation` 줄은 23:21:56 `Starting managed nodes bringup...` → `Configuring controller_server` → `Configuring smoother_server` 다음이 회차 끝(23:37:23 `rcl preshutdown`)이다. `Aborting bringup`·`Timed out waiting for transform`·`Failed to activate` 줄은 없다. 원인 분석은 #375.
- **C2(TB-5 정렬 판정 버려짐):** `robot_robot_b.log` 의 `live input rejected: Logger severity cannot be changed between calls.` #370 이 고쳤다(이 회차 빌드에는 없음).

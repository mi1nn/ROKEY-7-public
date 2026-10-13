# rescue_contracts

Python 코어 사이의 공통 오류·논리 로봇 ID·정수 나노초·임무 수락 receipt를 소유한다. 실제 중복된 계약 책임을 추출한 패키지이며 ROS `.msg/.srv/.action`을 생성하지 않는다. 예정된 `rescue_interfaces`와 구별한다.

`RescueError`의 `code`는 공통 C-09 분기 코드, `detail_code`는 원인 패키지를 포함한 세부 진단, `detail`은 설명이다. 진단→공통 코드 대응은 이 패키지의 `DETAIL_CATEGORIES` 한 곳에 둔다.

관측·TF·명령 발급·만료의 절대 시각은 int64 ns다. ROS 헤더는 `ros_time_to_ns(sec, nanosec)`와 `ns_to_ros_time`으로 정수 왕복한다. `seconds_to_ns`는 정확한 십진 fixture/정책 수치용이며 이미 정밀도를 잃은 epoch float를 복원하는 API가 아니다. 프로세스 내부 monotonic 시간과 timeout 정책은 로컬 float 초로 유지할 수 있다.

`robot_ids`는 프로파일에서 읽은 정확히 두 개의 ID를 요구한다. 단일 `robot_id`는 형식만 검사한다. 소속·명령 권한은 실행기/수신기 생성자에 주입한 allowlist와 authority로 검증한다. `TaskAdmission`은 해당 command/task/robot/boot/authority/generation의 수락 결과다. 순수 값 객체는 원격 인증·직렬화 검증을 대신하지 않는다.

## 격자 탐색 (`grid_search`, OPS-154/157)

TB-6 전체 탐색(frontier)과 TB-5 출구 선택(인형에서 출구까지 격자 경로 길이)에 조정기가 쓰는 순수 함수다. ROS 를 쓰지 않는다. 격자 인자(frame, width, height, resolution, origin=(x, y, yaw), data)는 `occupancy.footprint_clear` 와 같다. 좌표는 격자 frame 이다. 같은 격자는 같은 결과를 준다. 잘못된 입력은 ValueError 다.

- `clear_mask(..., radius, blocked=())`: 칸 중심에서 footprint_clear 가 통과하면 1 인 bytes 다(행 우선). `blocked=((x, y, r), …)` 원에 닿는 칸은 점유로 보고 같은 규칙을 적용한다(예: 오래 서 있는 다른 로봇). 아래 함수들은 `mask=` 로 이 결과를 다시 쓸 수 있다. mask 와 blocked 를 함께 주면 거절한다.
- `find_frontiers(..., radius, min_cells, max_goal_distance_m)`: 값 0 이면서 상하좌우에 -1 이 있는 칸을 8방향으로 묶은 덩어리다. `min_cells` 보다 작은 덩어리는 뺀다. 큰 것부터 `Frontier(cells, centroid, goal)` 로 준다. goal 은 centroid 에 가장 가까운 clear 칸이다(같으면 아래 행, 왼쪽 열). `max_goal_distance_m` 안에 없으면 None 이다. 도달 여부는 경로 함수로 따로 확인한다.
- `grid_path(..., radius, start, goal)`: clear 칸만 지나는 8방향 최단 경로다. 대각선은 양옆 칸이 둘 다 clear 일 때만 간다. `GridPath(length_m, points, start_moved_m)` 를 주고, 없으면 None 이다.
  - start 칸이 clear 가 아니면 radius 안의 가장 가까운 clear 칸으로 옮겨 시작한다. 로봇은 실제로 그 자리에 있으므로, 미탐색 칸 옆이라고 경로가 없다고 하지 않는다. 옮긴 거리는 길이에 더하고 `start_moved_m` 으로 남긴다.
  - radius 안에 clear 칸이 없으면 None 이다. goal 은 옮기지 않는다.
  - 길이는 칸 중심 사이 거리의 합이다.
- `path_lengths(..., radius, start, goals)`: 한 번의 탐색으로 목표마다 grid_path 와 같은 길이를 준다. 갈 수 없는 목표는 None 이다.
- `nearest_clear_outside(..., radius, from_xy, path_points, distance_m)`: 경로의 모든 점에서 `distance_m` 보다 먼 clear 칸 중 from_xy 에서 격자 경로가 가장 짧은 칸의 중심이다(TB-6 비켜서기 자리). 없으면 None 이다.

117×80 합성 격자(해상도 0.05 m, 반지름 0.2 m), 이 저장소 개발 PC 의 Python 3.12 실측:
- clear_mask: 약 52 ms
- mask 를 다시 쓸 때 find_frontiers: 12 ms
- 목표 13개 path_lengths: 13 ms
- 6.3 m grid_path: 8 ms
- nearest_clear_outside: 4 ms

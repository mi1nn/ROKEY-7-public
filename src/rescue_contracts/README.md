# rescue_contracts

Python 코어 사이의 공통 오류·논리 로봇 ID·정수 나노초·임무 수락 receipt를 소유한다. 실제 중복된 계약 책임을 추출한 패키지이며 ROS `.msg/.srv/.action`을 생성하지 않는다. 예정된 `rescue_interfaces`와 구별한다.

`RescueError`의 `code`는 공통 C-09 분기 코드, `detail_code`는 원인 패키지를 포함한 세부 진단, `detail`은 설명이다. 진단→공통 코드 대응은 이 패키지의 `DETAIL_CATEGORIES` 한 곳에 둔다.

관측·TF·명령 발급·만료의 절대 시각은 int64 ns다. ROS 헤더는 `ros_time_to_ns(sec, nanosec)`와 `ns_to_ros_time`으로 정수 왕복한다. `seconds_to_ns`는 정확한 십진 fixture/정책 수치용이며 이미 정밀도를 잃은 epoch float를 복원하는 API가 아니다. 프로세스 내부 monotonic 시간과 timeout 정책은 로컬 float 초로 유지할 수 있다.

`robot_ids`는 프로파일에서 읽은 정확히 두 개의 ID를 요구한다. 단일 `robot_id`는 형식만 검사한다. 소속·명령 권한은 실행기/수신기 생성자에 주입한 allowlist와 authority로 검증한다. `TaskAdmission`은 해당 command/task/robot/boot/authority/generation의 수락 결과다. 순수 값 객체는 원격 인증·직렬화 검증을 대신하지 않는다.

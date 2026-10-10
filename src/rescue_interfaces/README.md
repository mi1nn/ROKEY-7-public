# rescue_interfaces

SDD 검토안 v0.1 중 1차로 채택한 ROS 메시지 19·서비스 4·액션 2의 정의만 둔다. 판단 로직은 없다. 채택 범위와 IDL 규칙은 [contracts C-10](../../docs/design/contracts.md)에 있다.

- 필드는 [`sdd-node-contracts-v0.1.json`](../../docs/design/sdd-node-contracts-v0.1.json)이 원본이다. 필드를 바꿀 때는 JSON 을 먼저 고친다. [`tests/test_interfaces_schema.py`](../../tests/test_interfaces_schema.py)가 이 폴더의 IDL 과 JSON·`CMakeLists.txt` 목록이 같은지 ROS 없이 검사한다.
- ROS 메시지와 Python 값(`rescue_contracts`·각 패키지 dataclass)의 변환은 각 패키지의 ROS 경계 모듈이 한다.
- 빌드: `colcon build --packages-select rescue_interfaces` (ROS 2 Jazzy). CI 에는 ROS 가 없어 이 빌드는 돌지 않는다.

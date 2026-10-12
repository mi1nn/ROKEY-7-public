# rescue_coordination

경로 증거 기반 후보 선택, 중복 임무 배정 방지, 대상 최신성/접근 확인, 통로 점유 상태의 Python 코어다. 현재 코드는 로봇을 움직이거나 실제 경로를 계산하지 않는다. 임무 조정기 ROS 노드(얇은 버전, `mission_coordinator`)는 `ExecuteTask` goal 을 보낼 뿐이고, 토픽·파라미터·`reason_code` 대응표는 [구현 기록](../../docs/implementation/implementation-coordination.md)「임무 조정기 ROS 노드」에 있다.

API와 시험용 fixture는 [`tests/test_coordination.py`](../../tests/test_coordination.py), 출처·계약·미구현은 [구현 기록](../../docs/implementation/implementation-coordination.md)을 참고한다. 외부 증거를 생성하는 어댑터와 지도 자동 접근 허가가 연결되기 전에는 물리 로봇 실행 계층으로 사용하지 않는다.

프로파일의 두 ID를 `Allocator(..., allowed_robot_ids=...)`에 주입한다. DTO는 공통 ID 문법만 검사한다. `issue_generation`은 발급만 하고, `reserve`는 PENDING 예약만 만든다. 실행기에서 받은 공통 `TaskAdmission`을 `record_admission`에 넣어 정확한 명령·로봇·boot·authority·세대가 일치한 수락을 확인해야 ACTIVE가 된다. 발급 카운터와 실행기가 수락한 카운터를 별도로 기록한다.

외부 관측·기한·확인 시각은 정수 ns이며 `*_ns` 인자만 받는다. 로컬 수신 간격과 통로 lease는 monotonic 초다. `rescue_contracts`의 공통 오류/ID/시각/receipt 경계에 의존한다. 수색 구역 분할·frontier 어댑터·자동 역할 교대는 F-01 후속 범위이며 구현 완료로 집계하지 않는다.

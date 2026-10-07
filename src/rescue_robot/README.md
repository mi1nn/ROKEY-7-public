# rescue_robot

명령 수락·단일 활성 임무·임대·취소·정지 확인의 순수 Python 코어다. `executor.py`는 `SEND`/`CANCEL` 의도를 반환한다. ROS 발행, Nav2 연결, SLAM 실행, 언도킹은 하지 않는다.

공개 API는 `Policy`, `Readiness`, `Task`, `Executor`다. 인수와 시간 진행 예시는 [`tests/test_executor.py`](../../tests/test_executor.py)를 참고한다. 모든 운용 임계값은 호출자가 명시해야 한다. 시험값은 실기 기본값이 아니다.

자세한 구현 경계·출처·남은 검증은 [구현 기록](../../docs/implementation-robot.md)에 있다. 이 코어를 단독으로 로봇에 연결하지 않는다. 인증된 단일 어댑터, 시각/센서 증거, 장비측 watchdog이 필요한 상태다.

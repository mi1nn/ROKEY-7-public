# rescue_coordination

경로 증거 기반 후보 선택, 중복 임무 배정 방지, 대상 최신성/접근 확인, 통로 점유 상태의 Python 코어다. 현재 코드는 로봇을 움직이거나 실제 경로를 계산하지 않는다.

API와 시험용 fixture는 [`tests/test_coordination.py`](../../tests/test_coordination.py), 출처·계약·미구현은 [구현 기록](../../docs/implementation-coordination.md)을 참고한다. 외부 증거를 생성하는 어댑터와 지도 자동 접근 허가가 연결되기 전에는 물리 로봇 실행 계층으로 사용하지 않는다.

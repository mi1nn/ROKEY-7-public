"""로그인 반복 실패 제어: 동일 (계정명, 접속 IP) 쌍에서 비밀번호 오류 누적 시 일정 시간 요청을 차단한다.

관제 웹 프로세스별 인메모리로 관리되며 프로세스 재시작 시 초기화된다. 무차별 대입 공격을 효과적으로 지연시킨다.
계정 자체 잠금이 아니다: 계정 DB를 변경하지 않으며, 동일 계정이라도 다른 접속 주소에서는 정상 로그인이
가능하다. 개발 서버의 멀티스레드 요청 처리에 대응하여 모든 메서드는 단일 스레드 락(Lock)으로 동기화된다
(OPS-51-01).
"""

import math
import threading
import time

# [제안] 관제·화면, 2026-10-10: 5분 내 5회 비밀번호 오류 시 해당 접속 주소의 계정 시도를 60초간 차단한다.
LOGIN_MAX_FAILURES = 5
LOGIN_WINDOW_S = 300
LOGIN_LOCK_S = 60
# 추적할 최대 키 쌍 개수. 임의 계정명의 대량 시도로 인한 무제한 메모리 증가를 방지한다.
LOGIN_MAX_KEYS = 10_000


class LoginThrottle:
    """(계정명, 접속 IP) 쌍별로 비밀번호 오류 횟수를 추적하고, 임계치 초과 시 해당 요청을 일시 차단한다.

    내부 상태: `_entries`(키별 최근 실패 시각 목록 및 차단 해제 시각). 공유 스레드 락으로 보호된다.
    `clock`은 기본값으로 time.monotonic을 사용하여 시스템 시계 변경 시에도 차단 시간에 영향을 주지 않는다.
    테스트 시에는 모의(mock) 시계를 주입할 수 있다. 로그인 라우트는 비밀번호 검증 전 locked_for()를 호출하고,
    검증 결과에 따라 failed() 또는 succeeded()를 호출한다.
    """

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._entries = {}      # (계정명, 접속 IP) -> {'failures': [t, ...], 'locked_until': t}
        self._lock = threading.Lock()

    def __len__(self):
        with self._lock:
            return len(self._entries)

    @staticmethod
    def _key(name, client):
        # 공백 제거 및 소문자 정규화를 통해 대소문자나 공백 차이 계정을 동일 키로 묶어 집계한다.
        return (str(name).strip().lower(), str(client or ''))

    def locked_for(self, name, client):
        """해당 키 쌍의 차단 잔여 시간(올림 정수 초)을 반환한다. 시도 가능한 상태면 0."""
        with self._lock:
            entry = self._entries.get(self._key(name, client))
            if entry is None:
                return 0
            return max(0, math.ceil(entry['locked_until'] - self._clock()))

    def failed(self, name, client):
        """해당 키 쌍의 인증 실패 1회를 기록한다. 윈도우 시간 내 실패 횟수가 LOGIN_MAX_FAILURES에 도달하면 차단한다.

        LOGIN_WINDOW_S보다 오래된 실패 기록은 먼저 만료 제거한다. 차단 시에는 locked_until = now + LOGIN_LOCK_S로
        설정하고 목록을 비워 차단 해제 후 0부터 재집계한다. 엔트리가 LOGIN_MAX_KEYS를 초과하면
        가장 오래된 항목부터 순서대로 제거한다.
        """
        key = self._key(name, client)
        with self._lock:
            now = self._clock()
            entry = self._entries.pop(key, None) or {'failures': [], 'locked_until': 0.0}
            entry['failures'] = [t for t in entry['failures'] if now - t < LOGIN_WINDOW_S] + [now]
            if len(entry['failures']) >= LOGIN_MAX_FAILURES:
                entry['locked_until'] = now + LOGIN_LOCK_S
                entry['failures'] = []
            self._entries[key] = entry          # dict 끝에 재삽입하여 최근 활동 순서로 갱신한다
            while len(self._entries) > LOGIN_MAX_KEYS:
                del self._entries[next(iter(self._entries))]

    def succeeded(self, name, client):
        with self._lock:
            self._entries.pop(self._key(name, client), None)

"""로그인 반복 실패 잠금: (아이디, 접속 주소) 쌍 하나로 비밀번호를 거듭 틀리면 한동안 거절한다.

관제 웹 프로세스마다 메모리에만 두며, 다시 시작하면 지워진다. 한 곳에서 하는 비밀번호 추측을 늦춘다.
계정 잠금이 아니다: 계정 DB 에는 아무것도 쓰지 않고, 같은 아이디도 다른 접속 주소에서는
그대로 쓸 수 있다. 개발 서버는 요청을 스레드로 처리하므로 모든 메서드가 같은 스레드 잠금 하나를 잡는다
(OPS-51-01).
"""

import math
import threading
import time

# [제안] 관제·화면, 2026-10-10: 5분 안에 비밀번호를 5번 틀리면 그 접속 주소에서 그 아이디를 60 s 동안 잠근다.
LOGIN_MAX_FAILURES = 5
LOGIN_WINDOW_S = 300
LOGIN_LOCK_S = 60
# 기억하는 쌍의 최대 수. 지어낸 아이디를 쏟아부어도 메모리가 끝없이 늘지 않게 한다.
LOGIN_MAX_KEYS = 10_000


class LoginThrottle:
    """(아이디, 접속 주소) 쌍마다 틀린 비밀번호를 세고, 너무 많으면 그 쌍을 한동안 거절한다.

    상태: `_entries`(쌍마다 최근 실패 시각과 잠금 끝 시각). 요청 스레드가 함께 쓰는 스레드 잠금 하나로 지킨다.
    `clock` 은 기본이 time.monotonic 이라 벽시계를 바꿔도 잠금이 짧아지거나 길어지지 않는다. 시험은
    가짜 시계를 넘긴다. 로그인 라우트는 비밀번호를 확인하기 전에 locked_for() 를 부르고, 결과에 따라
    failed() 나 succeeded() 를 부른다.
    """

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._entries = {}      # (아이디, 접속 주소) -> {'failures': [t, ...], 'locked_until': t}
        self._lock = threading.Lock()

    def __len__(self):
        with self._lock:
            return len(self._entries)

    @staticmethod
    def _key(name, client):
        # 앞뒤 공백을 자르고 소문자로 바꿔, 대소문자나 공백만 다른 같은 아이디가 실패 수를 함께 센다.
        return (str(name).strip().lower(), str(client or ''))

    def locked_for(self, name, client):
        """이 쌍이 더 기다려야 하는 초(올림한 정수). 시도해도 되면 0."""
        with self._lock:
            entry = self._entries.get(self._key(name, client))
            if entry is None:
                return 0
            return max(0, math.ceil(entry['locked_until'] - self._clock()))

    def failed(self, name, client):
        """이 쌍의 틀린 비밀번호 한 번을 기록한다. 시간 창 안에서 LOGIN_MAX_FAILURES 에 이르면 잠금을 건다.

        LOGIN_WINDOW_S 보다 오래된 실패는 먼저 버린다. 잠글 때는 locked_until = now + LOGIN_LOCK_S 로 두고
        목록을 비우므로, 잠금이 끝나면 0 부터 다시 센다. 쌍이 LOGIN_MAX_KEYS 개를 넘으면
        마지막 실패가 가장 오래된 쌍부터 잊는다.
        """
        key = self._key(name, client)
        with self._lock:
            now = self._clock()
            entry = self._entries.pop(key, None) or {'failures': [], 'locked_until': 0.0}
            entry['failures'] = [t for t in entry['failures'] if now - t < LOGIN_WINDOW_S] + [now]
            if len(entry['failures']) >= LOGIN_MAX_FAILURES:
                entry['locked_until'] = now + LOGIN_LOCK_S
                entry['failures'] = []
            self._entries[key] = entry          # 맨 뒤에 다시 넣는다: dict 순서는 활동이 오래된 것부터다
            while len(self._entries) > LOGIN_MAX_KEYS:
                del self._entries[next(iter(self._entries))]

    def succeeded(self, name, client):
        with self._lock:
            self._entries.pop(self._key(name, client), None)

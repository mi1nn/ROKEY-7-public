"""Login throttle: after repeated wrong passwords for one (name, client) pair, refuse for a while.

In memory, per console process; a restart clears it. It slows password guessing from one place.
It is not an account lock: nothing is written to the accounts DB and the same name still works
from another client. The dev server serves requests on threads, so every method holds one lock
(OPS-51-01).
"""

import math
import threading
import time

# [제안] 관제·화면, 2026-10-10: 5 wrong passwords within 5 minutes lock that name from that client for 60 s.
LOGIN_MAX_FAILURES = 5
LOGIN_WINDOW_S = 300
LOGIN_LOCK_S = 60
# Remembered pairs at most, so a flood of made-up names cannot grow memory without bound.
LOGIN_MAX_KEYS = 10_000


class LoginThrottle:
    """Counts wrong passwords per (name, client) pair and refuses that pair for a while after too many.

    State: `_entries` (recent failure times and lock end per pair) behind one lock shared by request threads.
    `clock` is time.monotonic by default, so a wall-clock change cannot shorten or extend a lock; tests pass
    a fake clock. The login route calls locked_for() before checking the password, then failed() or
    succeeded() with the result.
    """

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._entries = {}      # (name, client) -> {'failures': [t, ...], 'locked_until': t}
        self._lock = threading.Lock()

    def __len__(self):
        with self._lock:
            return len(self._entries)

    @staticmethod
    def _key(name, client):
        # Trimmed and lower-cased, so case or space variants of one name share a single counter.
        return (str(name).strip().lower(), str(client or ''))

    def locked_for(self, name, client):
        """Whole seconds this pair must still wait; 0 when it may try."""
        with self._lock:
            entry = self._entries.get(self._key(name, client))
            if entry is None:
                return 0
            return max(0, math.ceil(entry['locked_until'] - self._clock()))

    def failed(self, name, client):
        """Record one wrong password for this pair; reaching LOGIN_MAX_FAILURES in the window starts the lock.

        Failures older than LOGIN_WINDOW_S are dropped first. On the lock, locked_until = now + LOGIN_LOCK_S
        and the list is cleared, so the count starts again from zero once the lock ends. Above LOGIN_MAX_KEYS
        pairs, the pairs whose last failure is oldest are forgotten first.
        """
        key = self._key(name, client)
        with self._lock:
            now = self._clock()
            entry = self._entries.pop(key, None) or {'failures': [], 'locked_until': 0.0}
            entry['failures'] = [t for t in entry['failures'] if now - t < LOGIN_WINDOW_S] + [now]
            if len(entry['failures']) >= LOGIN_MAX_FAILURES:
                entry['locked_until'] = now + LOGIN_LOCK_S
                entry['failures'] = []
            self._entries[key] = entry          # re-inserted last: dict order is oldest activity first
            while len(self._entries) > LOGIN_MAX_KEYS:
                del self._entries[next(iter(self._entries))]

    def succeeded(self, name, client):
        with self._lock:
            self._entries.pop(self._key(name, client), None)

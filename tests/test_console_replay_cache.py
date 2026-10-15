"""Closed-run timeline cache under concurrent requests (#322 review).

Flask serves requests on threads. When several requests each store a new closed run into a full cache, two
of them could pick the same oldest run and the second delete failed (KeyError), so that request answered 503.
The test widens the gap between picking and deleting; with the cache's lock nothing fails, and the same
test without the lock shows the failure, so the test does see the race.
"""
import contextlib
import threading
import time
import unittest
from unittest import mock

from rescue_console.replay import ClosedRunCache

SIZE, WRITERS = 4, 8


def fill_in_parallel(cache):
    """SIZE runs in the cache, then WRITERS threads each add a new one at once. Returns the errors raised."""
    for n in range(SIZE):
        cache.put(f'old-{n}', n)
    gate, errors = threading.Barrier(WRITERS), []

    def put(n):
        gate.wait(timeout=10)
        try:
            cache.put(f'new-{n}', n)
        except Exception as exc:   # noqa: BLE001 - the test reports whatever the race raises
            errors.append(exc)

    with mock.patch.object(ClosedRunCache, '_between', lambda self: time.sleep(0.02)):
        threads = [threading.Thread(target=put, args=(n,)) for n in range(WRITERS)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
    return errors


class ClosedRunCacheTests(unittest.TestCase):
    def test_parallel_puts_with_the_lock_never_fail_and_keep_the_size(self):
        cache = ClosedRunCache(SIZE)
        self.assertEqual([], fill_in_parallel(cache))
        self.assertEqual(SIZE, len(cache._runs))

    def test_the_same_puts_without_a_lock_fail(self):
        errors = fill_in_parallel(ClosedRunCache(SIZE, lock=contextlib.nullcontext()))
        self.assertTrue(errors)
        self.assertTrue(all(isinstance(exc, KeyError) for exc in errors))

    def test_oldest_goes_first_and_a_kept_run_is_returned(self):
        cache = ClosedRunCache(2)
        cache.put('a', 1)
        cache.put('b', 2)
        cache.put('c', 3)
        self.assertEqual((None, 2, 3), (cache.get('a'), cache.get('b'), cache.get('c')))
        cache.put('b', 20)   # replacing a kept run evicts nothing
        self.assertEqual((20, 3), (cache.get('b'), cache.get('c')))


if __name__ == '__main__':
    unittest.main()

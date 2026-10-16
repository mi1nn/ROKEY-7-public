"""The coordinator's task inputs are taken as soon as they arrive (C32; no ROS).

The executor asks the coordinator's ~/task_inputs (GetParameters 'source') every poll_period_s (0.5 s). The answer
was read only at the next poll, so a destination the coordinator had just picked reached the robot up to about
1.0 s later plus the network (통합-조정 read the code; the coordinator waits frontier_issue_delay_s 1.0 s for it).
Now the answer is handled in the request's done callback, under the same adapter lock as before (receive), and the
poll keeps only sending, the timeout and the clock sampler.
"""
import json
import threading
from types import SimpleNamespace as NS
import unittest

from rescue_robot.live_clock import LiveClock


class Future:
    """rclpy Future without an executor: callbacks run when the result is set."""

    def __init__(self):
        self._done, self._result, self.callbacks, self.cancelled = False, None, [], False

    def add_done_callback(self, callback):
        self.callbacks.append(callback)
        if self._done:
            callback(self)

    def set_result(self, result):
        self._done, self._result = True, result
        for callback in self.callbacks:
            callback(self)

    def done(self):
        return self._done

    def result(self):
        return self._result

    def cancel(self):
        self.cancelled = True


class Client:
    def __init__(self):
        self.sent = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.sent.append(Future())
        return self.sent[-1]


def answer(record):
    return NS(values=[NS(type=4, string_value=json.dumps(record))])


class TaskInputsImmediate(unittest.TestCase):
    def setUp(self):
        self.stopped, self.errors, self.received = [], [], []
        logger = NS(error=self.errors.append, warning=self.errors.append)
        runtime = NS(adapter=NS(lock=threading.RLock()), node=NS(get_logger=lambda: logger),
                     inputs=NS(lease_policy=NS(max_age_ns=10**12)),
                     _stop_input_session=lambda: self.stopped.append(True))
        self.clock = object.__new__(LiveClock)
        self.clock.runtime, self.clock.settings = runtime, {}
        self.clock.peer, self.clock.pending, self.clock.pending_since = None, None, None
        self.clock.sampler, self.clock.client = NS(poll=lambda: None), Client()
        self.clock.request_type = lambda names: NS(names=names)
        self.clock.receive = self.received.append

    def test_an_answer_is_taken_before_the_next_poll(self):
        self.clock.poll()                                   # asks
        self.clock.client.sent[0].set_result(answer({'destinations': {'a': 1}}))
        self.assertEqual([{'destinations': {'a': 1}}], self.received)   # before: only at the next poll
        self.assertIsNone(self.clock.pending)
        self.clock.poll()                                   # the next poll asks again, takes nothing twice
        self.assertEqual(2, len(self.clock.client.sent))
        self.assertEqual(1, len(self.received))

    def test_a_bad_answer_stops_the_input_session_at_once(self):
        self.clock.poll()
        self.clock.client.sent[0].set_result(NS(values=[]))
        self.assertEqual(([True], 1), (self.stopped, len(self.errors)))
        self.assertTrue(self.errors[0].startswith('live central input rejected'))
        self.assertIsNone(self.clock.peer)

    def test_an_answer_to_a_request_given_up_on_is_not_taken(self):
        self.clock.poll()
        old = self.clock.client.sent[0]
        self.clock.pending = None                           # timed out and dropped by poll
        old.set_result(answer({'destinations': {}}))
        self.assertEqual([], self.received)


if __name__ == '__main__':
    unittest.main()

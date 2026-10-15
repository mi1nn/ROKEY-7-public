"""scripts/sim/sim_operator.sh runbook 7.5: stopping TB-6's SLAM must stop slam_toolbox, not only the launch.

11th run (2026-10-16 05:07): TERM to the tb6_slam launch PID ended the launch but sync_slam_toolbox_node lived on
(re-parented to the subreaper) and kept publishing /robot6/map, so frozen_localization was not started. A terminal
Ctrl+C reaches the whole foreground process group; the launch was started with `exec setsid`, so its PID is its
process group, and signalling that group reaches every node it started.
"""
from pathlib import Path
import os
import signal
import subprocess
import time
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'sim' / 'sim_operator.sh'


@unittest.skipUnless(SCRIPT.exists(), 'installed-wheel checkout has no scripts/')
class SlamStop(unittest.TestCase):
    def test_the_launch_process_group_is_signalled(self):
        body = SCRIPT.read_text()
        stop = body[body.index('slam_to_frozen() {'):body.index('\n}\n', body.index('slam_to_frozen() {'))]
        self.assertIn('kill -TERM -- "-$pid"', stop)
        self.assertNotIn('kill -TERM "$pid"', stop)

    def test_a_group_signal_reaches_a_child_that_outlives_its_parent(self):
        # the shape of the 11th run: a launch-like parent in its own session with a long-lived child
        parent = subprocess.Popen(['setsid', 'bash', '-c', 'sleep 60 & echo $! ; wait'], stdout=subprocess.PIPE,
                                  text=True)
        child = int(parent.stdout.readline())
        time.sleep(0.2)
        try:
            os.kill(-parent.pid, signal.SIGTERM)   # what sim_operator.sh now does: kill -TERM -- "-$pid"
            parent.wait(timeout=5)
            for _ in range(50):
                try:
                    os.kill(child, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.1)
            else:
                self.fail('the child outlived the group signal')
        finally:
            for pid in (child, parent.pid):
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass


if __name__ == '__main__':
    unittest.main()

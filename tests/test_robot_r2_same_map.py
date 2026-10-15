"""R1 and R2 frozen almost together with the same grid (simulation 2026-10-15 23:08:50, mu-03), no ROS.

Exploration timed out, so the coordinator asked both freezes in the same second: MAP_FROZEN 1 and
MAP_FROZEN 2 carried the same sha256. TB-5's relay logged 'MAP_RELAYED ... R1 -> /robot5/map' twice,
the start alignment reached ALIGNED at 23:09:00, and after that robot_b showed MAP_STATE rev=2
quality=UNKNOWN with no /robot5/amcl_pose after 23:09:00.

What these tests pin down:
- The relay keeps one message per sha256 and replaces it when the same grid comes again (the R2
  freeze). The new object is not the one it sent, so it sends the same grid again, logged with the
  revision its executor names at that moment (still R1). AMCL and start_align get the same map twice.
- Nav2 Jazzy AMCL keeps its particles on a new map message (handleMapMessage frees the map and the
  laser models only) and publishes amcl_pose only after a filter update: the robot moved, a no-motion
  update was asked, or an initial pose was set. TB-5 standing still after ALIGNED publishes nothing.
- The executor drops TB-5's pose when it takes R2 and counts only AMCL poses stamped after that. For a
  grid it already localizes on, nothing about the pose changed, but VALID now waits for AMCL to answer
  the re-initialization.

The cause of the stuck UNKNOWN (robot_b logs, 가제보 실습 2026-10-15): the executor logged
'START_ALIGN robot_b ALIGNING: localization not VALID' at 23:08:50 with log.warning, then at ALIGNED
(23:09:00) the same line with log.info. rclpy (RcutilsLogger.log) refuses another severity at a call
site it has logged from: 'live input rejected: Logger severity cannot be changed between calls.'
(0.8 ms after START_ALIGN_ALIGNED). The input wrapper then drops the alignment (aligned.pop), so
robot_b never becomes VALID and no MAP_STATE follows. A disabled severity (debug) is skipped before
the check, so only enabled severities collide.

The tests marked expectedFailure describe the wanted behaviour and fail on main. The fix removes the
marks.
"""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

from rescue_robot.frozen_map_relay import RelayCore
from rescue_robot.live_sources import Grid, SavedMapSources

from test_robot_live_map_lifecycle import Runtime, pose_message, settings
from test_robot_live_map_twice import G1, R1, R2, grid_message, revision_message


def print_nothing(_text):
    pass


def copy_of(message):
    """The same grid as another message object (map_freeze publishes each freeze anew)."""
    return grid_message({i: v for i, v in enumerate(message.data) if v})


class RclpyLogger:
    """rclpy's rule: one severity per call site (file, line) once it has logged; debug is off (skipped)."""

    def __init__(self):
        self.sites, self.lines = {}, []
        for severity in ('info', 'warning', 'error', 'fatal'):
            setattr(self, severity, lambda text, severity=severity: self._log(severity, text))
        self.debug = lambda text: None

    def _log(self, severity, text):
        frame = sys._getframe(2)
        site = (frame.f_code.co_filename, frame.f_lineno)
        if self.sites.setdefault(site, severity) != severity:
            raise ValueError('Logger severity cannot be changed between calls.')
        self.lines.append(text)


class Relay(unittest.TestCase):
    def setUp(self):
        self.core = RelayCore()
        self.sha = Grid(G1).sha256
        self.assertEqual(self.sha, Grid(copy_of(G1)).sha256)

    def freeze(self, message, revision):
        self.core.on_grid(Grid(message).sha256, message)
        self.core.on_revision(revision_message(message, revision).data)

    def test_the_same_grid_frozen_twice_maps_both_revisions_to_it(self):
        self.freeze(G1, R1)
        self.freeze(copy_of(G1), R2)
        self.core.on_map_state(R2)
        self.assertEqual(self.sha, Grid(self.core.wanted()).sha256)
        self.assertEqual('', self.core.blocked)

    @unittest.expectedFailure
    def test_the_same_grid_again_is_not_a_new_map_to_send(self):
        """main() sends again unless wanted() is the very object it sent: R2's copy replaces it."""
        self.freeze(G1, R1)
        self.core.on_map_state(R1)
        sent = self.core.wanted()
        self.freeze(copy_of(G1), R2)                 # the R2 freeze, same sha256, a new message
        self.assertIs(sent, self.core.wanted())     # nothing new for <ns>/map while on R1
        self.core.on_map_state(R2)
        self.assertIs(sent, self.core.wanted())     # nor after the executor takes R2


class Executor(unittest.TestCase):
    def setUp(self):
        self.r = Runtime()
        self.logger = RclpyLogger()
        self.r.node = NS(get_logger=lambda: self.logger)
        self.s = SavedMapSources(self.r, settings('FROZEN_MAP_AMCL', 'robot_b'))
        self.reinitialized = []
        self.s.reinitialize = lambda robot, message: self.reinitialized.append(message)

    def freeze(self, message, revision):
        self.s.receive_frozen(message)
        self.s.receive_frozen_revision(revision_message(message, revision))

    def align(self, status):
        self.s.receive_start_align('robot_b', NS(data='{"status": "%s", "reason": ""}' % status))

    def pose(self):
        self.r.now += 1_000_000
        self.s.receive_pose('robot_b', pose_message(self.r.now - 500_000))

    def state(self):
        record = self.s.records()['robot_b']
        return record['localization_status'], record['map_revision']

    @unittest.expectedFailure
    def test_aligned_after_aligning_is_taken_with_the_rclpy_logger(self):
        """The simulation: ALIGNING (warning) then ALIGNED (info) from the same log line."""
        self.align('WAITING')
        self.freeze(G1, R1)
        self.s.receive_map('robot_b', G1)
        self.freeze(copy_of(G1), R2)
        self.align('ALIGNING')
        self.pose()
        self.align('ALIGNED')                        # ValueError on main: the verdict is dropped
        self.assertEqual(('VALID', R2), self.state())

    def test_both_freezes_before_the_alignment_end_valid_on_r2(self):
        """The simulation order as logged: R1 and R2 together, then the alignment poses, then ALIGNED.
        With a logger that accepts any severity the executor logic ends VALID: the map logic is not the cause."""
        self.r.node = NS(get_logger=lambda: NS(info=print_nothing, warning=print_nothing, error=print_nothing))
        self.align('WAITING')
        self.freeze(G1, R1)
        self.s.receive_map('robot_b', G1)
        self.freeze(copy_of(G1), R2)
        self.s.receive_map('robot_b', copy_of(G1))   # the relay's second copy
        self.align('ALIGNING')
        self.pose()
        self.pose()
        self.align('ALIGNED')
        self.assertEqual(('VALID', R2), self.state())
        self.assertEqual([], self.reinitialized)

    @unittest.expectedFailure
    def test_r2_with_the_grid_tb5_is_already_on_keeps_its_pose(self):
        """TB-5 aligned on R1 and stands still (AMCL silent). R2 is the same grid: nothing to re-align."""
        self.r.node = NS(get_logger=lambda: NS(info=print_nothing, warning=print_nothing, error=print_nothing))
        self.align('WAITING')
        self.freeze(G1, R1)
        self.s.receive_map('robot_b', G1)
        self.align('ALIGNING')
        self.pose()
        self.align('ALIGNED')
        self.assertEqual(('VALID', R1), self.state())
        self.freeze(copy_of(G1), R2)
        self.assertEqual(('VALID', R2), self.state())

    def test_r2_with_the_same_grid_waits_for_amcl_now(self):
        """What main does today in the case above: UNKNOWN until AMCL answers the re-initialization."""
        self.r.node = NS(get_logger=lambda: NS(info=print_nothing, warning=print_nothing, error=print_nothing))
        self.align('WAITING')
        self.freeze(G1, R1)
        self.s.receive_map('robot_b', G1)
        self.align('ALIGNING')
        self.pose()
        self.align('ALIGNED')
        self.freeze(copy_of(G1), R2)
        self.assertEqual(('UNKNOWN', R2), self.state())
        self.assertEqual(1, len(self.reinitialized))
        self.pose()
        self.assertEqual(('VALID', R2), self.state())


SRC = Path(__file__).resolve().parents[1] / 'src'
LOGGERS = ('info', 'warning', 'warn', 'error', 'fatal', 'debug')


def severity_chosen_at_run_time(tree):
    """Calls like (log.info if ok else log.error)(...) or getattr(log, level)(...): one site, several severities."""
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        f = node.func
        if isinstance(f, ast.IfExp) and all(isinstance(b, ast.Attribute) and b.attr in LOGGERS
                                            for b in (f.body, f.orelse)):
            yield node.lineno
        elif (isinstance(f, ast.Call) and isinstance(f.func, ast.Name) and f.func.id == 'getattr'
              and len(f.args) == 2 and not isinstance(f.args[1], ast.Constant)
              and 'log' in ast.unparse(f.args[0]).lower()):
            yield node.lineno


@unittest.skipUnless(SRC.exists(), 'installed-wheel checkout has no src/')
class OneSeverityPerLogLine(unittest.TestCase):
    """Every ROS node in this repository: a log line has one severity (rclpy raises otherwise).

    rescue_console logs with Flask (Python logging), which has no such rule; it is not checked."""

    @unittest.expectedFailure
    def test_no_ros_log_line_chooses_its_severity_at_run_time(self):
        found = []
        for path in sorted(SRC.glob('rescue_*/rescue_*/**/*.py')):
            if path.parts[-3] == 'rescue_console':
                continue
            for line in severity_chosen_at_run_time(ast.parse(path.read_text())):
                found.append(f'{path.relative_to(SRC)}:{line}')
        self.assertEqual([], found)

    def test_the_check_finds_both_forms(self):
        tree = ast.parse('(log.info if ok else log.warning)("x")\ngetattr(node.get_logger(), level)("y")\n'
                         'log.info("z")\ngetattr(log, "info")("w")\n')
        self.assertEqual([1, 2], sorted(severity_chosen_at_run_time(tree)))


if __name__ == '__main__':
    unittest.main()

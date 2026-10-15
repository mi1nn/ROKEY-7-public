"""scripts/field/noweb/status_watch.py: the console's cards as one line a second (웹 없는 트랙, 2026-10-15).

It reads the operations DB read-only through the site_state readers (no ROS, never creates the file). An
old value says how old it is and is marked stale; a value the DB does not have is printed as missing, never
filled in. The same line goes to the screen and, with --log, to a file.
"""
from contextlib import redirect_stdout
import importlib.util
import io
from pathlib import Path
import tempfile
import time
import unittest

from rescue_operations import site_state
from test_operations_site_state import dispatch, mission, robot_status, target_state, uid

REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / 'scripts' / 'field' / 'noweb' / 'status_watch.py'
if not SCRIPT.is_file():
    # as tests/test_field_wiring_check.py: a repo script, not part of the installed-wheel checkout
    raise unittest.SkipTest('scripts/field/noweb/status_watch.py absent (installed-wheel checkout)')
SPEC = importlib.util.spec_from_file_location('noweb_status_watch', SCRIPT)
sw = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(sw)

ROBOTS = ('robot_a', 'robot_b')
SECOND = 1_000_000_000


class Case(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.db = Path(self.temp.name) / 'site_state.sqlite3'
        self.ledger = site_state.SiteStateLedger(self.db, allowed_robot_ids=ROBOTS)
        self.addCleanup(self.ledger.close)

    def line(self, later_s=0):
        return sw.snapshot(self.db, now_ns=time.time_ns() + later_s * SECOND)


class LineTests(Case):
    def test_a_fresh_db_says_what_is_not_there_yet(self):
        line = self.line()
        for part in ('임무 기록 없음', 'robot_a 상태 없음', 'robot_b 상태 없음', '지도 상태 없음',
                     '지도 보관 꺼짐 — 고정 여부 모름', '대상 없음'):
            self.assertIn(part, line)
        self.assertEqual(1, len(line.splitlines()))

    def test_mission_robots_maps_and_targets_are_on_the_line(self):
        now = time.time_ns()
        self.ledger.record_mission_state(mission(phase='SEGMENT_MOVING', updated_at_ns=now, reason='START_POSE_CONFLICT')
                                         | dispatch(segment_index=1, segment_count=3, segment_phase='EXECUTING'))
        self.ledger.record_robot_status(robot_status(dock_state='UNDOCKED'))
        self.ledger.record_robot_status(robot_status(robot_id='robot_b', role='SUPPORT', robot_boot_id=uid(3),
                                                     motion_state='STOPPED', readiness='NOT_READY', pose=None,
                                                     active_task_id=None, generation=None, dock_state='DOCKED'))
        self.ledger.record_current_map(dict(robot_id='robot_a', map_session_id=uid(5), map_revision=7,
                                            generated_at_ns=now, received_at_ns=now))
        run = self.ledger.run_id
        for revision in (1, 2):
            self.ledger.record_map_snapshot(dict(snapshot_id=uid(800 + revision), robot_id='robot_a', revision=revision,
                                                 path=f'maps/{run}/frozen-r{revision}-{"c" * 12}.pgm', sha256='c' * 64,
                                                 at_ns=now + revision))
        self.ledger.record_map_snapshot(dict(snapshot_id=uid(810), robot_id='robot_a', revision=7,
                                             path=f'maps/{run}/live-r7-{"d" * 12}.pgm', sha256='d' * 64, at_ns=now))
        self.ledger.record_target_state(target_state(rescue_state='RECHECK_NEEDED', support_state='ASSIGNED'))
        line = self.line()
        for part in ('임무 DISASTER/SEGMENT_MOVING', '이유=START_POSE_CONFLICT', '구간 1/3 EXECUTING',
                     'robot_a(SCOUT) 준비=READY 이동=MOVING 도킹=UNDOCKED 자세=있음 작업=있음',
                     'robot_b(SUPPORT) 준비=NOT_READY 이동=STOPPED 도킹=DOCKED 자세=없음 작업=없음',
                     '지도 r7', '고정 지도 r1,r2', f'대상 1개 {uid(500)[:8]} RECHECK_NEEDED/ASSIGNED'):
            self.assertIn(part, line)
        self.assertNotIn('낡음', line)

    def test_no_frozen_map_is_said_only_when_the_archive_keeps_maps(self):
        """Without any kept map in the run the archive is off (or has kept nothing yet): frozen or not is not
        known, so the line says that and never 「고정 지도 없음」 (작전 OPS-289)."""
        self.assertNotIn('고정 지도 없음', self.line())
        self.ledger.record_map_snapshot(dict(snapshot_id=uid(820), robot_id='robot_a', revision=3,
                                             path=f'maps/{self.ledger.run_id}/live-r3-{"e" * 12}.pgm', sha256='e' * 64,
                                             at_ns=time.time_ns()))
        line = self.line()
        self.assertIn('고정 지도 없음', line)
        self.assertNotIn('지도 보관 꺼짐', line)

    def test_old_values_say_their_age_and_are_marked_stale(self):
        self.ledger.record_robot_status(robot_status())
        self.ledger.record_mission_state(mission(updated_at_ns=time.time_ns()))
        line = self.line(later_s=12)
        self.assertRegex(line, r'robot_a\(SCOUT\)[^|]*\(1[23]초 전, 낡음\)')
        self.assertRegex(line, r'임무 [^|]*\(1[23]초 전, 낡음\)')

    def test_the_reset_checklist_shows_after_a_reset(self):
        self.assertNotIn('초기화', self.line())
        self.ledger.record_robot_status(robot_status(robot_boot_id=uid(1)))
        self.ledger.close_run(at_ns=time.time_ns())
        self.ledger.open_run(at_ns=time.time_ns())
        self.assertIn('초기화 확인 조정기=아직 robot_a=아직 robot_b=아직', self.line())
        self.ledger.record_robot_status(robot_status(robot_boot_id=uid(11), stamp_ns=time.time_ns()))  # new boot
        self.assertIn('robot_a=다시 켜짐', self.line())

    def test_between_runs_says_so(self):
        self.ledger.close_run(at_ns=time.time_ns())
        self.assertIn('회차 없음', self.line())


class FileTests(unittest.TestCase):
    def test_a_missing_db_is_a_line_not_a_crash_and_is_not_created(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'none.sqlite3'
            line = sw.snapshot(path, now_ns=time.time_ns())
            self.assertIn('DB 읽기 실패', line)
            self.assertFalse(path.exists())

    def test_once_prints_and_appends_the_same_line_to_the_log(self):
        with tempfile.TemporaryDirectory() as temp:
            db = Path(temp) / 'site_state.sqlite3'
            site_state.SiteStateLedger(db, allowed_robot_ids=ROBOTS).close()
            log = Path(temp) / 'logs' / 'status.log'
            for _ in range(2):
                out = io.StringIO()
                with redirect_stdout(out):
                    self.assertEqual(0, sw.main(['--db', str(db), '--log', str(log), '--once']))
            lines = log.read_text().splitlines()
            self.assertEqual(2, len(lines))
            self.assertEqual(out.getvalue().strip(), lines[-1])


if __name__ == '__main__':
    unittest.main()

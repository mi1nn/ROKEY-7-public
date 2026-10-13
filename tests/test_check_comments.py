"""scripts/check_comments.py: 코드 리뷰가 묻는 주석 수준(프로그램·노드·함수)을 빠짐없이 세는지."""
import importlib.util
from pathlib import Path
import unittest

CHECK_COMMENTS = Path(__file__).resolve().parents[1] / 'scripts' / 'check_comments.py'

SOURCE = '''"""program docstring"""
import rclpy  # inline comment


class Watcher:
    """node docstring"""

    def step(self, frame):
        """function docstring"""
        a = 1
        b = 2
        c = 0  # inline comment
        return a + b + c + frame


class Bare:
    def run(self):
        a = 1
        b = 2
        c = 3
        d = 4
        return a + b + c + d


def tiny():
    return 1
'''


def load():
    spec = importlib.util.spec_from_file_location('check_comments', CHECK_COMMENTS)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@unittest.skipUnless(CHECK_COMMENTS.exists(), 'installed-wheel checkout has no scripts/')
class CommentAuditTests(unittest.TestCase):
    def setUp(self):
        self.audit = load().audit

    def test_reports_each_missing_level_with_line(self):
        r = self.audit(SOURCE, min_lines=5)
        self.assertTrue(r['program'])
        self.assertEqual(r['missing_classes'], ['Bare:16'])
        self.assertEqual(r['missing_functions'], ['run:17'])
        self.assertEqual(r['functions'], 2)  # tiny() is shorter than 5 lines and not counted
        self.assertEqual(r['comments'], 2)

    def test_missing_program_docstring_is_reported(self):
        self.assertFalse(self.audit('import os\n')['program'])

    def test_min_lines_one_counts_every_function(self):
        r = self.audit(SOURCE, min_lines=1)
        self.assertEqual(r['missing_functions'], ['run:17', 'tiny:25'])


if __name__ == '__main__':
    unittest.main()

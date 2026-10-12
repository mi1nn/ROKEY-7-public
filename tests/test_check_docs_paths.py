"""scripts/check_docs.py 의 개인 절대 경로 검사(AGENTS.md: 개인 절대 경로를 저장소에 쓰지 않음).

시험 문자열은 이 파일 자체가 검사에 걸리지 않도록 조각을 이어 만든다.
"""
import importlib.util
from pathlib import Path
import unittest

CHECK_DOCS = Path(__file__).resolve().parents[1] / 'scripts' / 'check_docs.py'


def load():
    spec = importlib.util.spec_from_file_location('check_docs', CHECK_DOCS)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@unittest.skipUnless(CHECK_DOCS.exists(), 'installed-wheel checkout has no scripts/')
class PersonalPathTests(unittest.TestCase):
    def setUp(self):
        self.personal_paths = load().personal_paths

    def test_flags_personal_paths_with_line_numbers(self):
        text = '\n'.join(['ok line',
                          'cd ' + '/Us' + 'ers/someone/work/repo',
                          'python3 ' + '/ho' + 'me/rokey/.cache/p4/x.py',
                          'tmp ' + '/private/var/fol' + 'ders/n3/abc/T/x',
                          'C:' + '/Us' + 'ers/Someone/x'])
        self.assertEqual(self.personal_paths('a.log', text), ['a.log:2', 'a.log:3', 'a.log:4', 'a.log:5'])

    def test_allows_tokens_robot_user_and_system_paths(self):
        text = '\n'.join(['<REPO>/src/rescue_robot', '<TMP>/x', '/home/<사용자>/', '/home/' + 'ubuntu/turtlebot4_ws',
                          '/opt/ros/jazzy/setup.bash', '/usr/lib/python3.12', '/var/log/syslog',
                          'https://github.com/jaebeom/ROKEY-7/blob/main/README.md'])
        self.assertEqual(self.personal_paths('b.md', text), [])

    def test_ubuntu_prefix_alone_is_not_an_exemption(self):
        self.assertEqual(self.personal_paths('c.md', '/ho' + 'me/ubuntux/x'), ['c.md:1'])


if __name__ == '__main__':
    unittest.main()

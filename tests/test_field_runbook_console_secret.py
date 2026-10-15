"""Field runbook step 3: the console's session secret comes from one fixed file on the operations PC.

Login now lasts 7 days (#446), but only while the secret that signs the cookie stays the same. A secret
made anew at every start logs everybody out at every restart of the web (작전 2026-10-16). The file is made
once, outside the checkout, and every start reads it.
"""
from pathlib import Path
import re
import unittest

REPO = Path(__file__).resolve().parents[1]
RUNBOOK = REPO / 'docs' / 'process' / 'field-runbook.md'
SECRET_FILE = '~/rokey7-field/console/secret'


def step3():
    text = RUNBOOK.read_text()
    return text[text.index('## 3단계 — 운영 PC'):text.index('## 4단계')]


@unittest.skipUnless(RUNBOOK.exists(), 'installed-wheel checkout has no docs/')
class ConsoleSecret(unittest.TestCase):
    def test_the_web_reads_the_secret_from_the_fixed_file(self):
        self.assertIn(f'RESCUE_CONSOLE_SECRET="$(cat {SECRET_FILE})" python3 -m rescue_console', step3())

    def test_the_file_is_made_only_when_it_is_not_there(self):
        made = [line for line in step3().splitlines() if 'token_urlsafe' in line]
        self.assertEqual(1, len(made), made)
        self.assertTrue(made[0].startswith(f'[ -f {SECRET_FILE} ] ||'), made[0])
        self.assertIn('umask 077', made[0])

    def test_the_file_is_outside_the_checkout(self):
        self.assertTrue(SECRET_FILE.startswith('~/') and not re.search(r'R7|ROKEY-7|rokey7/', SECRET_FILE))
        self.assertIn('저장소에 넣지 않는다', step3())


if __name__ == '__main__':
    unittest.main()

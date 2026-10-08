#!/usr/bin/env python3
"""Run stdlib tests from any cwd, using this checkout; reject empty suites."""
from pathlib import Path
import sys
import os
import unittest

ROOT = Path(__file__).resolve().parents[1]
for path in sorted((ROOT / 'src').glob('rescue_*')):
    sys.path.insert(0, str(path))
# Subprocess CLI tests must import the same checkout.
os.environ['PYTHONPATH'] = os.pathsep.join(str(p) for p in sorted((ROOT / 'src').glob('rescue_*')))
suite = unittest.defaultTestLoader.discover(str(ROOT / 'tests'))
count = suite.countTestCases()
if count == 0:
    raise SystemExit('ERROR: no tests collected')
print(f'Collected {count} tests', flush=True)
raise SystemExit(0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 1)

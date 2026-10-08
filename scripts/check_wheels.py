#!/usr/bin/env python3
"""Offline-build every reviewed package; test installed wheels in a fresh venv.

Build interpreter must already have setuptools and wheel. No network/package
resolver is used. Temporary build copies prevent dirtying the source checkout.
"""
from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
EXPECTED_PACKAGES = ('rescue_bringup', 'rescue_console', 'rescue_contracts', 'rescue_coordination',
                     'rescue_mapping', 'rescue_operations', 'rescue_perception', 'rescue_robot')
PACKAGES = sorted((ROOT / 'src').glob('rescue_*'))


def main():
    if tuple(p.name for p in PACKAGES) != EXPECTED_PACKAGES:
        raise RuntimeError(f'source package set does not match the {len(EXPECTED_PACKAGES)} reviewed packages')
    env = dict(os.environ)
    env.pop('PYTHONPATH', None)
    env.pop('PYTHONHOME', None)
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    with tempfile.TemporaryDirectory(prefix='rescue-wheel-check-') as folder:
        scratch = Path(folder)
        source, wheels, venv = scratch/'source', scratch/'wheels', scratch/'venv'
        source.mkdir()
        for package in PACKAGES:
            shutil.copytree(package, source/package.name,
                            ignore=shutil.ignore_patterns('__pycache__', '*.egg-info', 'build', '*.pyc'))
        def run(args, **kwargs):
            print('+', ' '.join(str(arg).replace(str(scratch), '<TMP>') for arg in args), flush=True)
            subprocess.run([str(arg) for arg in args], env=env, check=True, **kwargs)
        run([sys.executable, '-m', 'pip', 'wheel', '--no-index', '--no-deps', '--no-build-isolation',
             '--no-cache-dir', '--wheel-dir', wheels, *sorted(source.iterdir())])
        run([sys.executable, '-m', 'venv', venv])
        python = venv/('Scripts/python.exe' if os.name == 'nt' else 'bin/python')
        built = sorted(wheels.glob('*.whl'))
        if tuple(p.name.split('-')[0] for p in built) != EXPECTED_PACKAGES:
            raise RuntimeError('expected exactly one wheel per reviewed package, including rescue_contracts')
        run([python, '-m', 'pip', 'install', '--no-index', '--no-deps', *built])
        run([python, '-m', 'pip', 'check'])
        code = '''import importlib,sys
for name in ('rescue_robot','rescue_mapping','rescue_perception','rescue_coordination','rescue_operations','rescue_bringup','rescue_contracts','rescue_console'):
 module=importlib.import_module(name)
 assert module.__file__.startswith(sys.prefix), (name,module.__file__,sys.prefix)
 print('INSTALLED IMPORT',name)
'''
        run([python, '-I', '-c', code])
        # Copy tests/config only: no source tree exists in the test checkout.
        check = scratch/'check'
        for name in ('tests', 'config'):
            shutil.copytree(ROOT/name, check/name, ignore=shutil.ignore_patterns('__pycache__'))
        test_code = '''import sys,unittest
from pathlib import Path
assert not (Path.cwd()/'src').exists(), 'installed test checkout must not contain source'
print('SOURCELESS_CHECKOUT', flush=True)
suite=unittest.defaultTestLoader.discover(sys.argv[1])
assert suite.countTestCases()>0, 'empty installed test suite'
print('Installed suite:',suite.countTestCases(),flush=True)
raise SystemExit(0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 1)
'''
        run([python, '-I', '-B', '-c', test_code, check/'tests'], cwd=check)
        hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in built}
        print('WHEEL_SHA256', json.dumps(hashes, sort_keys=True), flush=True)
    print(f'PASS {len(EXPECTED_PACKAGES)} installed wheels, dependency check, isolated imports and nonempty tests')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Check repository Markdown locally; optionally verify pinned source objects.

No network, imports or execution of source-repository code. --sources requires
both clean clones at the fixed source commits. Without it source checks SKIP.
"""
import argparse
import collections
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
import sys
from urllib.parse import unquote, urlparse


def require(condition, message):
    if not condition:
        raise ValueError(message)


def git(repo, *args):
    return subprocess.check_output(['git', '-C', str(repo), *args], stderr=subprocess.PIPE)


def inventory(repo, record, name):
    paths = git(repo, 'ls-tree', '-r', '--name-only', '-z', record['commit']).decode().split('\0')[:-1]
    measured = {
        'tracked_files': len(paths),
        'root_groups': dict(sorted(collections.Counter(p.split('/')[0] if '/' in p else '(root files)' for p in paths).items())),
        'packages': sorted(p for p in paths if PurePosixPath(p).name == 'package.xml'),
        'test_files': sum(bool(re.fullmatch(r'test_.*\.py', PurePosixPath(p).name)) for p in paths),
        'python_files': sum(p.endswith('.py') for p in paths),
        'license_paths': sorted(p for p in paths if re.match(r'^(LICENSE|COPYING|NOTICE)(\.|$)', PurePosixPath(p).name, re.I)),
        'generated_roots': sorted({p.split('/')[0] for p in paths if p.split('/')[0] in {'build', 'install', 'log'}}),
    }
    for key, value in measured.items():
        require(record[key] == value, f'{name}: inventory mismatch: {key}')
    print(f"PASS inventory {name} {record['commit'][:7]}: files={len(paths)}, packages={len(measured['packages'])}, test_files={measured['test_files']}")


def check(root, source_paths):
    audit = json.loads((root / 'docs/evidence/source-audit.json').read_text())
    records = {'jaebeom/ROKEY_P4': audit['sources']['ROKEY_P4'],
               'mi1nn/amr-mini-project': audit['sources']['amr-mini-project']}
    sources = {}
    if source_paths:
        sources = dict(zip(records, (Path(p).resolve() for p in source_paths)))
        for name, repo in sources.items():
            require(git(repo, 'rev-parse', 'HEAD').decode().strip() == records[name]['commit'], f'{name}: wrong checkout SHA')
            require(not git(repo, 'status', '--porcelain'), f'{name}: dirty source checkout')
            inventory(repo, records[name], name)
        inventory(root, audit['destination_baseline'], 'jaebeom/ROKEY-7 baseline')
    docs = [root / 'README.md', *sorted((root / 'docs').rglob('*.md'))]
    local = pinned = ranges = 0
    for doc in docs:
        for url in re.findall(r'\[[^\]\n]+\]\(([^\s)]+)\)', doc.read_text()):
            parsed = urlparse(url)
            where = f'{doc.relative_to(root)}: {url}'
            if not parsed.scheme and not parsed.netloc:
                target = (doc.parent / unquote(parsed.path)).resolve()
                require(target.is_relative_to(root) and target.exists(), f'missing or external local path: {where}')
                local += 1
                continue
            parts = parsed.path.strip('/').split('/')
            name = '/'.join(parts[:2])
            if parsed.netloc != 'github.com' or name not in records or len(parts) < 3 or parts[2] not in {'blob', 'tree'}:
                continue
            require(len(parts) >= 4 and parts[3] == records[name]['commit'], f'source link is not pinned to audit SHA: {where}')
            kind, sha = parts[2:4]
            path = unquote('/'.join(parts[4:]))
            obj = f'{sha}:{path}' if path else f'{sha}^{{tree}}'
            lines = None
            if parsed.fragment:
                lines = re.fullmatch(r'L(\d+)(?:-L(\d+))?', parsed.fragment)
                require(lines is not None and kind == 'blob', f'invalid source line anchor: {where}')
                require(1 <= int(lines[1]) <= int(lines[2] or lines[1]), f'reversed source line range: {where}')
                ranges += 1
            if sources:
                require(git(sources[name], 'cat-file', '-t', obj).decode().strip() == kind, f'wrong source object: {where}')
                if lines:
                    require(int(lines[2] or lines[1]) <= len(git(sources[name], 'show', obj).splitlines()), f'source line range exceeds file: {where}')
            pinned += 1
    print(f'PASS markdown: files={len(docs)}, local_links={local}, pinned_source_links={pinned}, line_ranges={ranges}')
    if not sources:
        print('SKIP source objects, line bounds and inventory: supply --sources P4_DIR AMR_DIR')
    print('NOT CHECKED: prose meaning, local heading anchors, external web availability')
    print('NOT RUN: source tests, colcon, ROS, model inference, bag replay, robot/4-PC acceptance')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--sources', nargs=2, metavar=('P4_DIR', 'AMR_DIR'))
    args = parser.parse_args()
    try:
        check(args.root.resolve(), args.sources)
    except (ValueError, KeyError, OSError, subprocess.CalledProcessError) as exc:
        print(f'FAIL: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())

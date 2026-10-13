#!/usr/bin/env python3
"""Report comment levels the course code review asks for: program, node(class), function, inline.

Code review slide (2026-10-13, tutor): "Must have #Comment's at Program, Node, Function,
and in line (where appropriate) level". Program = module docstring, node = class docstring,
function = function docstring. Inline comments cannot be judged mechanically, so only their
count is printed. Functions shorter than --min-lines body lines are not counted.

Report only by default (exit 0). With --strict, exit 1 when any counted docstring is missing,
so a lane can confirm its own files after fixing them.
"""
import argparse
import ast
import io
from pathlib import Path
import sys
import tokenize

ROOT = Path(__file__).resolve().parents[1]
MIN_LINES = 5  # 2026-10-13 code review prep: 4-line helpers rarely need a docstring to be read


def audit(source, min_lines=MIN_LINES):
    """Return program/class/function results and inline comment count for one source text."""
    tree = ast.parse(source)
    nodes = sorted(ast.walk(tree), key=lambda n: getattr(n, 'lineno', 0))  # report in file order
    classes = [n for n in nodes if isinstance(n, ast.ClassDef)]
    funcs = [n for n in nodes if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
             and n.end_lineno - n.body[0].lineno + 1 >= min_lines]
    comments = sum(t.type == tokenize.COMMENT for t in tokenize.generate_tokens(io.StringIO(source).readline))
    code = sum(1 for line in source.splitlines() if line.strip() and not line.strip().startswith('#'))
    return {
        'program': ast.get_docstring(tree) is not None,
        'classes': len(classes),
        'missing_classes': [f'{n.name}:{n.lineno}' for n in classes if ast.get_docstring(n) is None],
        'functions': len(funcs),
        'missing_functions': [f'{n.name}:{n.lineno}' for n in funcs if ast.get_docstring(n) is None],
        'comments': comments,
        'code': code,
    }


def files(paths):
    """Python files under the given paths, skipping vendored code, tests and setup.py."""
    for path in paths:
        found = [path] if path.is_file() else sorted(path.rglob('*.py'))
        for f in found:
            if 'vendor' not in f.parts and 'test' not in f.parts and f.name != 'setup.py':
                yield f


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('paths', nargs='*', type=Path, help='files or folders (default: src/rescue_*)')
    parser.add_argument('--min-lines', type=int, default=MIN_LINES)
    parser.add_argument('--strict', action='store_true')
    args = parser.parse_args()
    missing = 0
    for f in files(args.paths or sorted((ROOT / 'src').glob('rescue_*'))):
        r = audit(f.read_text(encoding='utf-8'), args.min_lines)
        name = f.resolve().relative_to(ROOT) if f.resolve().is_relative_to(ROOT) else f
        gaps = (not r['program']) + len(r['missing_classes']) + len(r['missing_functions'])
        missing += gaps
        print(f"{'GAP ' if gaps else 'OK  '}{name}  program={'yes' if r['program'] else 'NO'}"
              f"  class={r['classes'] - len(r['missing_classes'])}/{r['classes']}"
              f"  function={r['functions'] - len(r['missing_functions'])}/{r['functions']}"
              f"  inline_comments={r['comments']}/{r['code']}")
        for item in r['missing_classes'] + r['missing_functions']:
            print(f'      no docstring: {name}:{item.split(":")[1]} {item.split(":")[0]}')
    print(f'TOTAL missing docstrings: {missing} (function body >= {args.min_lines} lines; inline comments not judged)')
    return 1 if args.strict and missing else 0


if __name__ == '__main__':
    sys.exit(main())

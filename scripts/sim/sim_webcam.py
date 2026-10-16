#!/usr/bin/env python3
"""The fixed webcam for run_full.sh, attached as runbook step 3 attaches it (재범 2026-10-16 10:07 「mu-03에 웹캠 붙였는데
안뜸. 이거 뜨게 해줘」).

    sim_webcam.py yaml <example.yaml> <repo> > <run>/webcam_node.yaml   # the webcam YAML, model path made absolute
    sim_webcam.py camera <operations.log>                               # the console --camera value the launch logged

The YAML's model_path is relative to the repository root (webcam_node.example.yaml says so) and the launch runs
elsewhere, so the copy gets the absolute path; nothing else changes (device_path is overridden by webcam_device:=).
Anything unexpected is refused, nothing is guessed.
"""
import re
import sys

MODEL_PATH = re.compile(r'^(\s*model_path:\s*)(assets/\S+)', re.MULTILINE)
CAMERA = re.compile(r'console --camera (\S+)')
COLOUR = re.compile(r'\x1b\[[0-9;]*m')


def webcam_yaml(text, repo):
    """The webcam YAML text with its relative model_path made absolute under repo; ValueError without one."""
    out, n = MODEL_PATH.subn(lambda m: m.group(1) + repo.rstrip('/') + '/' + m.group(2), text)
    if n != 1:
        raise ValueError(f'expected one relative model_path under assets/, found {n}')
    return out


def console_camera(log_text):
    """The value after 'console --camera ' in the operations launch log (colour codes removed), or None."""
    m = CAMERA.search(COLOUR.sub('', log_text))
    return m.group(1) if m else None


def main(argv):
    if len(argv) == 3 and argv[0] == 'yaml':
        with open(argv[1], encoding='utf-8') as handle:
            sys.stdout.write(webcam_yaml(handle.read(), argv[2]))
        return 0
    if len(argv) == 2 and argv[0] == 'camera':
        with open(argv[1], encoding='utf-8', errors='replace') as handle:
            value = console_camera(handle.read())
        if value is None:
            print('sim_webcam: no "console --camera" line', file=sys.stderr)
            return 1
        print(value)
        return 0
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))

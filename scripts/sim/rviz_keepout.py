#!/usr/bin/env python3
"""A copy of scripts/field/noweb/two_robots.rviz with the tunnel keep-out mask as one more Map display (2026-10-16).

    rviz_keepout.py <two_robots.rviz> <out.rviz>

run_full.sh shows the simulation on the simulation PC's screen with the field's two-robot RViz (tf_merge.py + this
file, #414). The keep-out mask (/rescue/passage_keepout/mask, rescue_robot.passage_keepout, latched) is added
after the frozen map, so the field file stays as its owner keeps it. Standard library only; refuses a file it does
not recognise rather than guessing where to put the display.
"""
import sys

ANCHOR = '    - Class: rviz_default_plugins/Map\n      Name: Frozen map\n'
MASK_TOPIC = '/rescue/passage_keepout/mask'
DISPLAY = f'''    - Class: rviz_default_plugins/Map
      Name: Tunnel keep-out mask
      Enabled: true
      Value: true
      Alpha: 0.6
      Color Scheme: costmap
      Draw Behind: false
      Topic:
        Value: {MASK_TOPIC}
        Depth: 1
        History Policy: Keep Last
        Reliability Policy: Reliable
        Durability Policy: Transient Local
'''


def with_keepout(text):
    """The RViz text with the mask display right after the frozen map display, or ValueError."""
    if MASK_TOPIC in text:
        raise ValueError('the keep-out mask is already there')
    start = text.find(ANCHOR)
    if start < 0 or text.count(ANCHOR) != 1:
        raise ValueError('no single "Frozen map" Map display to put the mask after')
    end = text.find('\n    - Class:', start + len(ANCHOR))   # the next display at the same indent
    if end < 0:
        raise ValueError('the frozen map display is not followed by another display')
    return text[:end + 1] + DISPLAY + text[end + 1:]


def main(argv):
    if len(argv) != 2:
        print(__doc__.split('\n\n')[1].strip(), file=sys.stderr)
        return 2
    try:
        out = with_keepout(open(argv[0], encoding='utf-8').read())
    except (OSError, ValueError) as exc:
        print(f'rviz_keepout: {exc}', file=sys.stderr)
        return 2
    with open(argv[1], 'w', encoding='utf-8') as handle:
        handle.write(out)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))

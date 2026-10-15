#!/usr/bin/env python3
"""Launch the AMR viewer with same-frame ROI comparison and C-key collection."""
import sys
from yolo_depth_viewer import parse, run

if __name__ == '__main__':
    if '--compare' not in sys.argv:
        sys.argv.append('--compare')
    run(parse())

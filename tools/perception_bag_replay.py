#!/usr/bin/env python3
"""Source-checkout entry point for the installed E2 offline replay CLI."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
for package in ("rescue_contracts", "rescue_perception"):
    sys.path.insert(0, str(ROOT / "src" / package))

from rescue_perception.bag_replay_cli import main  # noqa: E402


if __name__ == "__main__":
    sys.exit(main())

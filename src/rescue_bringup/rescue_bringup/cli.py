"""Local profile inspection. This command cannot start ROS or move a robot."""

import argparse
from collections import Counter
from pathlib import Path
import sys

from .profile import ProfileError, load_deployment


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", type=Path, help="deployment JSON to validate")
    args = parser.parse_args(argv)
    try:
        profile = load_deployment(args.profile)
    except ProfileError as exc:
        print(f"INVALID: {exc}", file=sys.stderr)
        return 2
    tiers = Counter(host.tier for host in profile.hosts)
    tier_summary = ", ".join(f"{count} {tier}" for tier, count in sorted(tiers.items()))
    print(f"STATIC CONFIG VALID: {profile.id}; {len(profile.robots)} robots; "
          f"{len(profile.hosts)} hosts ({tier_summary})")
    for robot in profile.robots:
        print(f"{robot.id}: namespace={robot.namespace}, command_host={robot.command_host}")
    print("Runtime is disabled. No hardware, ROS graph, timing or safety validation was performed.")
    return 0

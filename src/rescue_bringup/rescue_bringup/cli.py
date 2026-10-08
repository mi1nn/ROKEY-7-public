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
        role = f", role={profile.mission.role_of(robot.id)}" if profile.mission else ""
        print(f"{robot.id}: namespace={robot.namespace}, command_host={robot.command_host}{role}")
    if profile.mission:
        print(f"mission: schema v{profile.schema_version}; {len(profile.mission.search_areas)} search areas; "
              f"hold positions for {', '.join(r for r, _ in profile.mission.hold_positions)}")
    if profile.schema_version >= 2:
        with_network = sum(1 for host in profile.hosts if host.network is not None)
        print(f"v2.1 blocks: {len(profile.site_cameras)} site cameras; {with_network} hosts with network; "
              f"readiness for {', '.join(r.id for r in profile.robots if r.parameters.readiness) or 'none'}")
    print("Runtime is disabled. No hardware, ROS graph, timing or safety validation was performed.")
    return 0

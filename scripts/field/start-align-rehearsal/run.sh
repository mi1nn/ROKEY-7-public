#!/usr/bin/env bash
# TB-5 start alignment on one PC, no robot (E3): a fake TB-5 world (frozen map, LaserScan from the true pose,
# TF, odom, undocked) + the real nav2_amcl started at a wrong "measured" start + rescue_robot start_align,
# grouped as frozen_localization.launch.py groups them. Prints every /robot5/rescue_start_align state.
#
#   bash scripts/field/start-align-rehearsal/run.sh <setup.bash ...> -- <out dir> "<truth x y yaw>" \
#       "<measured x y yaw>" [domain]
#
# <setup.bash ...>: ROS 2 Jazzy, then workspaces with this repository's packages and nav2_amcl,
# nav2_lifecycle_manager (built from source where Nav2 is not installed).
# Safety: no Discovery Server, LOCALHOST only, domain >= 10; nothing here commands a robot.
set -m
SETUP=()
while [ "$#" -gt 0 ] && [ "$1" != "--" ]; do SETUP+=("$1"); shift; done
[ "$1" = "--" ] || { echo "usage: run.sh <setup.bash ...> -- <out> \"<truth>\" \"<measured>\" [domain]"; exit 2; }
shift
OUT=$1; TRUTH=$2; MEASURED=$3; DOMAIN=${4:-47}
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../.." && pwd)
[ "$DOMAIN" -ge 10 ] || { echo "domain must be >= 10"; exit 2; }
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT ROS_STATIC_PEERS
export ROS_DOMAIN_ID=$DOMAIN ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
for f in "${SETUP[@]}"; do source "$f"; done
mkdir -p "$OUT"
python3 "$HERE/fake_world.py" $TRUTH > "$OUT/world.log" 2>&1 & W=$!
E3_REPO=$REPO E3_MEASURED="$MEASURED" ros2 launch "$HERE/align.launch.py" > "$OUT/launch.log" 2>&1 & L=$!
python3 "$HERE/watch.py" 60 > "$OUT/watch.log" 2>&1; RC=$?
for signal in INT TERM KILL; do
  for pid in $L $W; do kill "-$signal" -- "-$pid" 2>/dev/null; done
  sleep 2
done
cat "$OUT/watch.log"; echo "watch exit $RC (logs in $OUT)"
exit $RC

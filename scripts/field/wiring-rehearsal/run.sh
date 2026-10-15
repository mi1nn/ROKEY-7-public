#!/usr/bin/env bash
# One-PC wiring rehearsal (2026-10-14): a local Discovery Server, this repository's field nodes with the real
# namespaces (/robot6, /robot5), fakes for the rest (fake_graph.py), then scripts/field/wiring_check.py.
#
#   bash scripts/field/wiring-rehearsal/run.sh <colcon install dir> <out dir> [SEARCHING|FROZEN] [port] [domain]
#
# Safety: ROS_DISCOVERY_SERVER is cleared and set to 127.0.0.1:<port> only. A field server address left in it
# would join the real robot graph (a Discovery Server does not separate by domain, network-discovery.md 3절).
# ROS_DOMAIN_ID >= 10. No robot is on this graph and the fake Nav2 refuses every goal; only REHEARSAL_PROBE=mission
# arms the two executors (as the field step 3). Everything started here is stopped by its own process group or
# PID (no pkill -f).
# REHEARSAL_PROBE=freeze|mission|noweb (freeze_probe.py, mission_probe.py, noweb_probe.py), REHEARSAL_TF=measured|flat (TB-6 TF, feed.py).
set -euo pipefail
# job control: each background job is its own process group and keeps SIGINT (a non-interactive shell's
# background jobs ignore SIGINT otherwise, and the launch children then outlive the launch)
set -m
INSTALL=${1:?colcon install dir with rescue_* (and nav2_msgs, irobot_create_msgs)}
OUT=${2:?out dir}
PHASE=${3:-SEARCHING}
PORT=${4:-11888}
DOMAIN=${5:-193}
REPO=$(cd "$(dirname "$0")/../../.." && pwd)
HERE="$REPO/scripts/field/wiring-rehearsal"
mkdir -p "$OUT"; OUT=$(cd "$OUT" && pwd)

for name in $(env | grep -oE '^(ROS_DISCOVERY_SERVER|ROS_SUPER_CLIENT|ROS_STATIC_PEERS|FASTRTPS_DEFAULT_PROFILES_FILE|FASTDDS_DEFAULT_PROFILES_FILE)=' | tr -d =); do
  unset "$name"
done
set +u; source /opt/ros/jazzy/setup.bash; source "$INSTALL/setup.bash"; set -u
export ROS_DISCOVERY_SERVER="127.0.0.1:$PORT" ROS_SUPER_CLIENT=False ROS_DOMAIN_ID="$DOMAIN"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4
[ "$ROS_DISCOVERY_SERVER" = "127.0.0.1:$PORT" ] || { echo "ROS_DISCOVERY_SERVER is not local only"; exit 2; }
[ "$DOMAIN" -ge 10 ] || { echo "ROS_DOMAIN_ID must be >= 10"; exit 2; }
if ss -lun | grep -q ":$PORT "; then echo "port $PORT already in use"; exit 2; fi

# profile: the example profile with the field namespaces (tests/test_bringup_field_launch.py field_profile);
# field: the repository field file + a rehearsal-only TB-5 start [0, 0, 0] (frozen_localization needs one; not a measurement)
python3 - "$REPO" "$OUT" <<'PY'
import json, sys
from pathlib import Path
repo, out = Path(sys.argv[1]), Path(sys.argv[2])
sys.path.insert(0, str(repo / 'tests'))
import test_bringup_field_launch as fixtures
work = out / 'inputs'; work.mkdir(exist_ok=True)
if not (work / 'profile.json').exists():
    fixtures.field_profile(work)
field = json.loads(fixtures.FIELD.read_text())
field['robots']['robot_b'].setdefault('initial_pose_xy_yaw', [0.0, 0.0, 0.0])
(work / 'field.json').write_text(json.dumps(field))
PY
PROFILE="$OUT/inputs/profile.json"; FIELD="$OUT/inputs/field.json"

PIDS=()   # each is also its process group id (set -m)
alive() { for pid in "$@"; do kill -0 -- "-$pid" 2>/dev/null && return 0; done; return 1; }
cleanup() {
  for signal in INT TERM KILL; do
    alive "${PIDS[@]}" || break
    for pid in "${PIDS[@]}"; do kill "-$signal" -- "-$pid" 2>/dev/null || true; done
    for _ in $(seq 30); do alive "${PIDS[@]}" || break; sleep 0.5; done
  done
  [ -n "${DS_PID:-}" ] && kill -TERM "$DS_PID" 2>/dev/null || true
  wait 2>/dev/null || true
  alive "${PIDS[@]}" && echo "WARNING: rehearsal processes left; process groups ${PIDS[*]}" || echo "stopped: process groups ${PIDS[*]}, discovery server $DS_PID"
}
trap cleanup EXIT

fast-discovery-server -i 0 -l 127.0.0.1 -p "$PORT" > "$OUT/discovery_server.log" 2>&1 &
DS_PID=$!; echo "discovery server pid $DS_PID 127.0.0.1:$PORT"
sleep 1
ros2 launch "$HERE/rehearsal.launch.py" profile_path:="$PROFILE" field_path:="$FIELD" out_dir:="$OUT" \
  api_port:=18765 slam_phase:="$PHASE" > "$OUT/launch.log" 2>&1 &
PIDS+=($!)
python3 "$HERE/fake_graph.py" --profile "$PROFILE" --field "$FIELD" --slam-phase "$PHASE" > "$OUT/fake_graph.log" 2>&1 &
PIDS+=($!)
for _ in $(seq 120); do grep -q 'fake nodes up' "$OUT/fake_graph.log" 2>/dev/null && break; sleep 0.5; done
sleep 3
set +e
python3 "$REPO/scripts/field/wiring_check.py" --profile "$PROFILE" --field "$FIELD" --slam-phase "$PHASE" \
  --spin-time 5 --json "$OUT/wiring_$PHASE.json" > /dev/null 2> "$OUT/wiring_$PHASE.txt"
CODE=$?
cat "$OUT/wiring_$PHASE.txt"
echo "wiring_check exit $CODE (logs in $OUT)"
if [ "${REHEARSAL_PROBE:-}" = freeze ]; then   # frozen map path with synthetic SLAM/AMCL data (freeze_probe.py)
  FINAL=""; [ "$PHASE" = FROZEN ] && FINAL=--final
  python3 "$HERE/freeze_probe.py" $FINAL --tf "${REHEARSAL_TF:-measured}" > "$OUT/freeze_probe.txt" 2>&1
  PROBE=$?
  cat "$OUT/freeze_probe.txt"
  echo "freeze_probe exit $PROBE"
  [ "$CODE" -eq 0 ] && CODE=$PROBE
elif [ "${REHEARSAL_PROBE:-}" = mission ]; then   # DISASTER -> TB-6 goal -> refused by the fake Nav2 (mission_probe.py)
  python3 "$HERE/mission_probe.py" > "$OUT/mission_probe.txt" 2>&1
  PROBE=$?
  cat "$OUT/mission_probe.txt"
  echo "mission_probe exit $PROBE"
  [ "$CODE" -eq 0 ] && CODE=$PROBE
elif [ "${REHEARSAL_PROBE:-}" = noweb ]; then   # the run without the web console (noweb_probe.py)
  python3 "$HERE/noweb_probe.py" "$OUT" 18765 > "$OUT/noweb_probe.txt" 2>&1
  PROBE=$?
  cat "$OUT/noweb_probe.txt"
  echo "noweb_probe exit $PROBE"
  [ "$CODE" -eq 0 ] && CODE=$PROBE
fi
set -e
exit $CODE

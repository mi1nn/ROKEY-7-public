#!/usr/bin/env bash
# Operator action: arms robot_a and starts physical SEARCH_AREA with a synthetic trigger.
set -e -o pipefail
task_workspace_root="${1:?Usage: bash start_search.sh CENTRAL_WORKSPACE}"
cd "$task_workspace_root"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
task_bundle_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
shift
exec python3 "$task_bundle_dir/start_search.py" "$@"

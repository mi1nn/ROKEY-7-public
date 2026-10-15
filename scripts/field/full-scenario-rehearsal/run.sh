#!/usr/bin/env bash
# 로봇 없는 한 PC 전체 시나리오 리허설 (방식 B, 2026-10-15): 실제 통합 실행기를 --rehearsal 로 돌린다.
#
#   bash scripts/field/full-scenario-rehearsal/run.sh <colcon install> <결과 폴더> [domain] [실행기 추가 인자...]
#
# 로봇 Pi·센서·OAK-D 감지기만 sim_world.py(2D 모의, world.json)로 바꾸고 SLAM·AMCL·start_align·Nav2 두 벌·실행기·
# watchdog·조정기·운영 노드·도킹 노드·알림음은 실제로 띄운다. 모의 입력의 결과이지 실기 검증이 아니다.
# 안전: 현장 Discovery 설정(ROS_DISCOVERY_SERVER 등)을 지우고 이 PC 안에서만 찾는다. 도메인 >= 10(로봇 도메인과 분리).
#   REHEARSAL_DISCOVERY=server (기본): 127.0.0.1:<REHEARSAL_PORT, 기본 11889> 의 로컬 fast-discovery-server(현장과 같은
#     Discovery Server 방식, wiring-rehearsal/run.sh 와 같음). 실행기만 super client, 노드는 일반 client.
#   REHEARSAL_DISCOVERY=localhost: ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST 단순 discovery. 2026-10-15 리허설에서 프로세스
#     약 30개·서비스 600개 규모에서 일부 노드 쌍이 서로를 끝내 찾지 못했다(조정기 시계 서비스 ↔ 실행기, 회차마다 다른
#     로봇). 비교용으로만 남긴다.
# 프로파일은 저장소 예시 + 현장 namespace(tests/test_bringup_field_launch.py field_profile).
# TB-5 seed 는 world.json 의 정답(TB-6 출발 기준 [0, 1.0, 0])에 일부러 (+0.08 m, -0.06 m, +0.05 rad) 오차를 넣은
# 합성값이다(start_align 이 바로잡는지 보려는 것, 현장값 아님).
set -eo pipefail
INSTALL=${1:?colcon install dir}
OUT=${2:?result dir}
DOMAIN=${3:-195}
shift 3 2>/dev/null || shift $#
REPO=$(cd "$(dirname "$0")/../../.." && pwd)
HERE="$REPO/scripts/field/full-scenario-rehearsal"
for name in ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT ROS_STATIC_PEERS FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE; do
  unset "$name"
done
source /opt/ros/jazzy/setup.bash
source "$INSTALL/setup.bash"
export ROS_DOMAIN_ID="$DOMAIN" FASTDDS_BUILTIN_TRANSPORTS=UDPv4
[ "$DOMAIN" -ge 10 ] || { echo "ROS_DOMAIN_ID must be >= 10"; exit 2; }
mkdir -p "$OUT/inputs"; OUT=$(cd "$OUT" && pwd)
[ -e "$OUT/run" ] && { echo "$OUT/run exists: use a new result dir"; exit 2; }
MODE=${REHEARSAL_DISCOVERY:-server}
DS_PID=""
if [ "$MODE" = server ]; then
  PORT=${REHEARSAL_PORT:-11889}
  if ss -lun | grep -q ":$PORT "; then echo "port $PORT already in use"; exit 2; fi
  fast-discovery-server -i 0 -l 127.0.0.1 -p "$PORT" > "$OUT/discovery_server.log" 2>&1 &
  DS_PID=$!
  trap '[ -n "$DS_PID" ] && kill -TERM "$DS_PID" 2>/dev/null; wait "$DS_PID" 2>/dev/null; echo "discovery server $DS_PID stopped"' EXIT
  export ROS_DISCOVERY_SERVER="127.0.0.1:$PORT"
  sleep 1
  echo "local discovery server pid $DS_PID 127.0.0.1:$PORT"
elif [ "$MODE" = localhost ]; then
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
else
  echo "REHEARSAL_DISCOVERY must be server or localhost"; exit 2
fi
/usr/bin/python3 - "$REPO" "$OUT/inputs" <<'PY'
import sys
from pathlib import Path
repo, work = Path(sys.argv[1]), Path(sys.argv[2])
sys.path.insert(0, str(repo / 'tests'))
import test_bringup_field_launch as fixtures
if not (work / 'profile.json').exists():
    fixtures.field_profile(work)
PY
cd "$REPO"
/usr/bin/python3 -m rescue_bringup.full_scenario --rehearsal --profile "$OUT/inputs/profile.json" \
  --field "$REPO/config/field/live_map.overlay.json" --run-dir "$OUT/run" --tb5-start 0.08 0.94 0.05 \
  --sim-world "$HERE/sim_world.py" --world "$HERE/world.json" --no-rviz "$@"

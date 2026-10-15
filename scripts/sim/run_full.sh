#!/bin/bash
# 시뮬레이션 2단계: 실기 한 회차를 PC 한 대에서 (가제보_실습, 2026-10-15, 재범 지시 「운영 PC·Discovery PC·로봇 담당 PC·로봇 역할을 한 PC 로」).
#
#   로봇 Pi 두 대  -> sim_robots.sh (Gazebo, TB-6·TB-5, 도메인 SIM_DOMAIN)
#   Wi-Fi          -> wall_clock_relay.py (시각·프레임·카메라 이름을 실기와 같게, SIM_DOMAIN <-> FIELD_DOMAIN)
#   Discovery PC   -> 없음. 한 PC 안이라 LOCALHOST 탐색 + scripts/sim/fastdds_localhost.xml
#   운영 PC        -> operations.launch.py + 관제 웹 + coordinator.launch.py
#   로봇 담당 PC   -> tb6_slam, robot.launch(두 벌), map_freeze, frozen_localization(TB-5), passage_points, overhead_passage
# 순서는 docs/process/field-runbook.md 3~7단계 그대로다. 실행기는 조정기와 같은 PC 이면 거절하므로 두 실행기에만
# scripts/field/wiring-rehearsal/host_shim 을 PYTHONPATH 앞에 둔다(10-14 한 PC 리허설과 같음, robot.launch 전체에
# 걸리지만 그 shim 은 REHEARSAL_HOST_ID 가 있을 때 clock_measurement 의 host_id 만 바꾼다).
# 로봇은 지도를 모른다: 실제 지도·Gazebo 정답 위치는 어느 노드에도 주지 않는다.
#
# 사용: run_full.sh --world <이름> --world-dir <폴더> --web-host <관제 웹 바인드 주소> [--web-port 8080] [--headless]
#        관리자 비밀번호는 처음 한 번 환경 변수 RUN_FULL_ADMIN_PASSWORD(8자 이상)로만 받는다(저장소·로그에 남기지 않음).
#        계정 DB·세션 비밀키는 RUN_FULL_WEB_STATE(기본 ~/sim-runs/web-state)에 두고 회차 사이에 그대로 쓴다.
#       끄기: stop_full.sh <회차 폴더>
#       SIM_PASSAGE=off: 깊이 장애물 정지(PassageStop)를 시뮬레이션 사본에서만 끈다. 정지 구역 앞 범위를 로봇 몸통 안
#       [0.0, 0.05] m 로 줄여 카메라가 볼 수 없게 한다(높이 띠는 현장 검사 규칙 min < 라이다 0.23 m 때문에 못 올림).
#       같은 값을 쓰는 터널 판정(overhead_passage)도 사실상 꺼진다. 기본은 현장 값. 2026-10-15 그 값이 TB-5 도크 앞에서 TB-6 을
#       회전까지 세워 작업이 만료됐다(설계 결함으로 작전이 통합-로봇에 넘김, docs/evidence/sim-2026-10-15-passage-deadlock.md).
# 재난 선포는 하지 않는다. 운영자가 관제 웹에서 누른다(실기와 같음). 인형 인식은 아직 없다(가짜 표적 주입기 후속).
set -u
SIM_DOMAIN=79
FIELD_DOMAIN=78
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
RUN=""
die() {   # 실패하면 이 회차에서 이미 띄운 것을 끈다(반쯤 뜬 상태로 두지 않는다)
    echo "run_full: $*" >&2
    [ -n "$RUN" ] && [ -f "$RUN/token" ] && "$HERE/stop_full.sh" "$RUN" >&2
    exit 2
}
say() { echo "run_full: $(date +%H:%M:%S) $*"; }

world=""; world_dir=""; web_host=""; web_port=8080; headless=""
while [ $# -gt 0 ]; do
    case "$1" in
        --world) world="${2:-}"; shift 2 || die "--world 값이 없습니다" ;;
        --world-dir) world_dir="${2:-}"; shift 2 || die "--world-dir 값이 없습니다" ;;
        --web-host) web_host="${2:-}"; shift 2 || die "--web-host 값이 없습니다" ;;
        --web-port) web_port="${2:-}"; shift 2 || die "--web-port 값이 없습니다" ;;
        --headless) headless=--headless; shift ;;
        *) die "알 수 없는 인자 '$1'" ;;
    esac
done
[ -n "$world" ] && [ -n "$world_dir" ] && [ -n "$web_host" ] || die "--world, --world-dir, --web-host 가 필요합니다"
WEB_STATE="${RUN_FULL_WEB_STATE:-$HOME/sim-runs/web-state}"   # 관제 웹 계정 DB·세션 비밀키(회차 사이 유지)
RUN_FULL_ADMIN_PASSWORD="${RUN_FULL_ADMIN_PASSWORD:-}"
[ -f "$REPO/install/setup.bash" ] || die "$REPO/install 이 없습니다(colcon build --packages-up-to rescue_bringup rescue_console)"
[ -x "${RUN_FULL_WEB_PYTHON:-$HOME/venvs/sim_web/bin/python}" ] || die "Flask 가 있는 python 이 없습니다(RUN_FULL_WEB_PYTHON)"
WEB_PY="${RUN_FULL_WEB_PYTHON:-$HOME/venvs/sim_web/bin/python}"

RUN="${RUN_FULL_DIR:-$HOME/sim-runs/full-$(date +%Y%m%d-%H%M%S)}"
[ -e "$RUN" ] && die "$RUN 이 이미 있습니다(회차마다 새 폴더)"
mkdir -p "$RUN/logs" "$RUN/maps" "$RUN/freeze" "$RUN/pids"
TOKEN="run_full.$$.$(date +%s)"
echo "$TOKEN" > "$RUN/token"
FIELD_SRC="$REPO/config/field/search_area.field.json"
F="$RUN/field.sim.json"   # 현장 파일 사본. 현장 파일은 고치지 않는다
# 출발 자리와 인형 자리는 world 의 spawn.json 에서 읽는다(sim1 = arena: 평면도 [추정], sim2: 미로 [설정]). 없으면 멈춘다.
read -r -a SPAWN < <(python3 - "$world_dir/$world.spawn.json" <<'PY'
import json, sys
try:
    d = json.load(open(sys.argv[1]))
    print(*(d['robots'][r][k] for r in ('robot6', 'robot5') for k in ('x', 'y', 'yaw')), *d['doll'])
except (OSError, KeyError, ValueError, TypeError) as exc:
    sys.exit(f'{sys.argv[1]}: {exc!r}')
PY
)
[ "${#SPAWN[@]}" -eq 8 ] || die "$world_dir/$world.spawn.json 에 robot6·robot5 출발 자리와 doll 자리가 있어야 합니다"
TB6_START=("${SPAWN[@]:0:3}")   # world 좌표, 도크를 보고 선다
TB5_START=("${SPAWN[@]:3:3}")
DOLL=("${SPAWN[@]:6:2}")
P="$RUN/profile/profile.json"
SHIM="$REPO/scripts/field/wiring-rehearsal/host_shim"
DDS="$REPO/scripts/sim/fastdds_localhost.xml"
BASE_ENV="set +u; source /opt/ros/jazzy/setup.bash; source $REPO/install/setup.bash; set -u
export RUN_FULL_TOKEN=$TOKEN ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTRTPS_DEFAULT_PROFILES_FILE=$DDS ROS_SUPER_CLIENT=False"
FIELD_ENV="$BASE_ENV; export ROS_DOMAIN_ID=$FIELD_DOMAIN"

# 하나를 터미널 없이 띄운다. 로그는 $RUN/logs/<이름>.log, 이 회차 표시(RUN_FULL_TOKEN)로 나중에 PID 로 끈다.
start() {
    local name="$1"; shift
    env -i HOME="$HOME" PATH=/usr/bin:/bin bash -c "$FIELD_ENV; $*" > "$RUN/logs/$name.log" 2>&1 < /dev/null &
    echo $! > "$RUN/pids/$name"   # exec setsid 라 이 PID 가 launch 자신이다(런북 7.5 에서 tb6_slam 을 끈다)
    say "띄움 $name (로그 $RUN/logs/$name.log)"
}
# 로그에 줄이 나올 때까지 기다린다(색 코드 지우고). 안 나오면 멈춘다.
wait_for() {
    local name="$1" pattern="$2" seconds="$3" i
    for i in $(seq 1 "$seconds"); do
        sed 's/\x1b\[[0-9;]*m//g' "$RUN/logs/$name.log" 2>/dev/null | grep -qE "$pattern" && { say "확인 $name: $pattern"; return 0; }
        sleep 1
    done
    die "$name 에서 '$pattern' 이 ${seconds}초 안에 나오지 않았습니다. $RUN/logs/$name.log"
}
call() {   # ros2 CLI 한 번(현장 도메인)
    env -i HOME="$HOME" PATH=/usr/bin:/bin bash -c "$FIELD_ENV; $*"
}

say "회차 폴더 $RUN"
say "빌드 $(git -C "$REPO" rev-parse --short HEAD)$(git -C "$REPO" diff --quiet HEAD -- src || echo '+src 수정')(src 는 main 과 diff $(git -C "$REPO" diff origin/main HEAD -- src 2>/dev/null | wc -l)줄), 관제 웹 빌드 $(git -C "${RUN_FULL_WEB_REPO:-$REPO}" rev-parse --short HEAD)"
# 관제 웹 계정 DB 와 세션 비밀키는 회차마다 새로 만들지 않고 한 폴더에 두고 계속 쓴다. 회차를 다시 띄울 때마다 휴대폰
# 로그인이 풀렸다(재범 2026-10-15). 처음 한 번만 RUN_FULL_ADMIN_PASSWORD 로 만든다.
mkdir -p "$WEB_STATE"; chmod 700 "$WEB_STATE"
if [ ! -f "$WEB_STATE/accounts.sqlite3" ]; then
    [ "${#RUN_FULL_ADMIN_PASSWORD}" -ge 8 ] || die "처음에는 RUN_FULL_ADMIN_PASSWORD(8자 이상)가 필요합니다"
    RESCUE_CONSOLE_ADMIN_PASSWORD="$RUN_FULL_ADMIN_PASSWORD" PYTHONPATH="$REPO/src/rescue_console:$REPO/src/rescue_contracts" \
        python3 -m rescue_console.accounts init "$WEB_STATE/accounts.sqlite3" --admin admin > "$RUN/logs/accounts.log" 2>&1 \
        || die "계정 DB 를 못 만들었습니다"
fi
[ -f "$WEB_STATE/secret" ] || { python3 -c 'import secrets; print(secrets.token_urlsafe(32))' > "$WEB_STATE/secret"; chmod 600 "$WEB_STATE/secret"; }

# 1. 로봇 쪽(Gazebo)
env -i HOME="$HOME" PATH=/usr/bin:/bin RUN_FULL_TOKEN="$TOKEN" RUN_SIM_LOG_DIR="$RUN/logs/robots" \
    DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
    setsid "$HERE/sim_robots.sh" --world "$world" --world-dir "$world_dir" --x "${TB6_START[0]}" --y "${TB6_START[1]}" \
    --yaw "${TB6_START[2]}" --also robot5 "${TB5_START[@]}" $headless > "$RUN/logs/robots.log" 2>&1 < /dev/null &
say "띄움 robots (로그 $RUN/logs/robots.log)"
wait_for robots "떴습니다" 240

# 2. 중계
start relay "exec setsid python3 $HERE/wall_clock_relay.py --sim-domain $SIM_DOMAIN --field-domain $FIELD_DOMAIN --namespace robot6 --namespace robot5"
wait_for relay "RELAY robot5" 30

# 3. 두 로봇 언도킹(로봇 감독 몫, 시뮬레이션 도메인). 런북 4단계는 TB-6 을 언도킹해 출발 표시에 둔 뒤 SLAM·실행기·지도 고정,
#    5단계는 TB-5 도 1차 고정 전에 언도킹. 출발 자세(START_POSE)는 언도킹 뒤 자세다(작전 2026-10-15).
for ns in robot6 robot5; do
    # 런북 2단계 「Create3 연결」: dock_status 가 오고 도킹 상태인지 먼저 본다. 로봇이 뜬 직후 바로 보내면 거절됐다(2026-10-15).
    for i in $(seq 1 12); do
        call "timeout 10 ros2 topic echo --once /$ns/dock_status irobot_create_msgs/msg/DockStatus" > "$RUN/logs/dock_status_$ns.log" 2>&1
        grep -q "is_docked: true" "$RUN/logs/dock_status_$ns.log" && break
        sleep 5
    done
    grep -q "is_docked: true" "$RUN/logs/dock_status_$ns.log" || die "$ns dock_status 가 도킹 상태로 오지 않습니다(로그 $RUN/logs/dock_status_$ns.log)"
    say "$ns dock_status is_docked: true"
    env -i HOME="$HOME" PATH=/usr/bin:/bin bash -c "$BASE_ENV; export ROS_DOMAIN_ID=$SIM_DOMAIN; \
timeout 120 ros2 action send_goal /$ns/undock irobot_create_msgs/action/Undock '{}'" > "$RUN/logs/undock_$ns.log" 2>&1
    grep -q "SUCCEEDED" "$RUN/logs/undock_$ns.log" || die "언도킹 $ns 실패(로그 $RUN/logs/undock_$ns.log)"
    say "언도킹 $ns"
done

# 언도킹 뒤 두 로봇의 자세. 현장에서는 언도킹한 두 로봇을 재서 TB-6 몸 기준 TB-5 상대 위치를 넣는다(줄자 1.86 m).
# 시뮬레이션에서는 그 「재는 일」을 확인용 정답 위치(Gazebo, /<ns>/sim_ground_truth_pose)로 한다. 이 값은 현장 파일 사본을
# 만드는 데만 쓰고, 어떤 노드에도 주지 않는다.
# 출발 자리 ⊕ odom 으로 계산하던 것을 바꿨다: 언도킹 회전에서 시뮬레이션 odom 이 약 11.5° 더 돌아(TB-6 odom 101° 대 정답 89.5°)
# TB-6 방향 기준 TB-5 상대 위치가 0.4 m 어긋났고, TB-5 출발 정렬이 START_ALIGN_NOT_CONVERGED 로 끝났다(3회차, #386).
# 함께 읽는 TB-6 odom 은 복귀 지점(SIM_HOME=odom)을 TB-6 지도 frame(= TB-6 odom frame)으로 옮기는 데만 쓴다.
# 시뮬레이션 도메인에서 읽는다: 정답 위치는 중계가 현장 도메인으로 넘기지 않는다(넘기면 안 된다).
read -r -a POSES < <(env -i HOME="$HOME" PATH=/usr/bin:/bin bash -c "$BASE_ENV; export ROS_DOMAIN_ID=$SIM_DOMAIN; python3 -" <<'PY'
import math, sys, time, rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); node = rclpy.create_node('run_full_poses'); got = {}
def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
def keep(key):
    def f(m):
        p = m.pose.pose
        got[key] = (p.position.x, p.position.y, yaw(p.orientation))
    return f
subs = [node.create_subscription(Odometry, topic, keep(key), qos_profile_sensor_data)
        for key, topic in (('gt6', '/robot6/sim_ground_truth_pose'), ('gt5', '/robot5/sim_ground_truth_pose'),
                           ('odom6', '/robot6/odom'))]
end = time.time() + 20
while time.time() < end and len(got) < 3:
    rclpy.spin_once(node, timeout_sec=0.2)
if len(got) < 3:
    sys.exit('missing: ' + ' '.join(k for k in ('gt6', 'gt5', 'odom6') if k not in got))
print(*got['gt6'], *got['gt5'], *got['odom6'])
PY
)
[ "${#POSES[@]}" -eq 9 ] || die "언도킹 뒤 정답 위치·odom 을 못 읽었습니다"
TB6_UNDOCKED=("${POSES[@]:0:3}"); TB5_UNDOCKED=("${POSES[@]:3:3}"); TB6_ODOM=("${POSES[@]:6:3}")
say "언도킹 뒤 arena 자세(확인용 정답 위치): TB-6 ${TB6_UNDOCKED[*]}, TB-5 ${TB5_UNDOCKED[*]}; TB-6 odom ${TB6_ODOM[*]}"
# 시뮬레이션 사본: TB-5 출발값 = 언도킹 뒤 TB-6 몸 기준으로 본 TB-5 의 실제 상대 위치(현장에서 재는 값과 같은 뜻).
read -r -a TB5_REL < <(python3 "$HERE/start_pose.py" relative "${TB6_UNDOCKED[@]}" "${TB5_UNDOCKED[@]}")
[ "${#TB5_REL[@]}" -eq 3 ] || die "TB-5 출발값을 계산하지 못했습니다"
python3 - "$FIELD_SRC" "$F" "${TB5_REL[@]}" "${SIM_PASSAGE:-on}" <<'PY' | while read -r line; do say "$line"; done
import json, sys
src, out, passage = sys.argv[1], sys.argv[2], sys.argv[6]
rel = [float(v) for v in sys.argv[3:6]]
field = json.load(open(src))
old = field['robots']['robot_b']['initial_pose_xy_yaw']
field['robots']['robot_b']['initial_pose_xy_yaw'] = rel
print(f'robots.robot_b.initial_pose_xy_yaw 현장 {old}, 시뮬레이션 사본 {rel} (언도킹 뒤 TB-6 몸 기준 TB-5, 확인용 정답 위치로 잼)')
if passage == 'off':
    old = field['robot']['passage.front_m']
    field['robot']['passage.front_m'] = [0.0, 0.05]
    print(f'현장 값과 다름: passage.front_m {old} -> [0.0, 0.05] (SIM_PASSAGE=off: PassageStop 과 터널 판정 사실상 끔)')
elif passage != 'on':
    sys.exit(f'SIM_PASSAGE must be on or off, got {passage!r}')
json.dump(field, open(out, 'w'), ensure_ascii=False, indent=2, sort_keys=True)
PY
[ -s "$F" ] || die "시뮬레이션 현장 파일 사본을 못 만들었습니다"
# 복귀 지점(home_positions): 현장은 1차 고정 지도에서 재서 --home 으로 넣는다(런북 1·5단계). 시뮬레이션은 같은 자리(언도킹 뒤
# 출발 자세)를 TB-6 지도 frame 으로 넣는다. TB-6 지도 frame = TB-6 odom frame 이라 TB-6 은 odom 값 그대로다. TB-5 는 arena 자세를
# 그 frame 으로 옮긴다: arena -> TB-6 odom frame 변환은 같은 순간의 TB-6 정답 자세와 odom 자세로 정한다(odom 이 돈 만큼 포함).
read -r -a TB5_IN_MAP < <(python3 "$HERE/start_pose.py" in-frame "${TB6_UNDOCKED[@]}" "${TB6_ODOM[@]}" "${TB5_UNDOCKED[@]}")
[ "${#TB5_IN_MAP[@]}" -eq 3 ] || die "TB-5 복귀 지점을 계산하지 못했습니다"
HOME_A="${TB6_ODOM[0]},${TB6_ODOM[1]},${TB6_ODOM[2]}"
HOME_B="${TB5_IN_MAP[0]},${TB5_IN_MAP[1]},${TB5_IN_MAP[2]}"
HOME_ARGS=()
case "${SIM_HOME:-off}" in   # 기본은 현장과 같게(측정 전에는 없음). sim-to-field 원칙(#374): 현장 값과 다른 것은 기본 꺼짐
    odom) HOME_ARGS=(--home "robot_a=$HOME_A" --home "robot_b=$HOME_B")
          say "현장 값과 다름: home_positions 현장 없음(측정 전), 시뮬레이션 robot_a=$HOME_A robot_b=$HOME_B (SIM_HOME=odom, 언도킹 뒤 자세)" ;;
    off) say "home_positions 없음(현장과 같음, 런북 1·5단계에서 재서 넣는 값). SIM_HOME=odom 이면 언도킹 뒤 자세를 넣는다" ;;
    *) die "SIM_HOME must be off or odom" ;;
esac
python3 "$REPO/scripts/field/make_field_profile.py" --out "$RUN/profile" --discovery 127.0.0.1:11811 \
    --domain "$FIELD_DOMAIN" "${HOME_ARGS[@]}" > "$RUN/logs/profile.log" 2>&1 \
    || die "프로파일을 못 만들었습니다. $RUN/logs/profile.log"

# 4. 운영 + 관제 웹 (런북 3단계)
start operations "exec setsid ros2 launch rescue_bringup operations.launch.py profile_path:=$P db_path:=$RUN/site_state.sqlite3 \
api_port:=18180 command_timeout_s:=3.0 report_db_path:=$RUN/reports.sqlite3 envelope_schema_version:=rescue-envelope/1 \
report_publisher_id:=ops.report map_topic:=/robot6/map map_dir:=$RUN/maps robot_namespaces:=robot_a=/robot6,robot_b=/robot5 \
robot_watch_path:=$RUN/robot_watch.json"
wait_for operations "rescue_db_writer git=" 60
# 관제 웹은 RUN_FULL_WEB_REPO(기본 이 저장소)의 빌드로 띄운다. 웹만 더 새 main 으로 볼 때 쓴다(2026-10-15 #366).
WEB_REPO="${RUN_FULL_WEB_REPO:-$REPO}"
[ -f "$WEB_REPO/install/setup.bash" ] || die "$WEB_REPO/install 이 없습니다"
start console "set +u; source $WEB_REPO/install/setup.bash; set -u; export RESCUE_CONSOLE_SECRET=\$(cat $WEB_STATE/secret); exec setsid $WEB_PY -m rescue_console --accounts $WEB_STATE/accounts.sqlite3 \
--site-state $RUN/site_state.sqlite3 --operations http://127.0.0.1:18180 --robot-name robot_a=TB-6 --robot-name robot_b=TB-5 \
--host $web_host --port $web_port --live-map $RUN/maps/live_map.yaml --live-map-stale-s 15 --robot-watch $RUN/robot_watch.json \
--robot-watch-stale-s 5 --robot-watch-map-robot robot_a --support-dispatch AUTO --search-start AUTO --field-file $F"
wait_for console "Running on" 30


# 5. TB-6 (런북 4단계): SLAM -> 로봇 launch -> 지도 고정(TB-6 이 언도킹 뒤 움직이기 전)
start tb6_slam "exec setsid ros2 launch rescue_bringup tb6_slam.launch.py profile_path:=$P field_path:=$F robot_id:=robot_a"
wait_for tb6_slam "SLAM_ACTIVE" 60
for r in robot_a robot_b; do
    start "robot_$r" "export PYTHONPATH=$SHIM:\$PYTHONPATH REHEARSAL_HOST_ID=sim-$r; exec setsid ros2 launch rescue_bringup robot.launch.py \
profile_path:=$P field_path:=$F robot_id:=$r"
done
wait_for robot_robot_a "installed task runtime" 90
wait_for robot_robot_b "installed task runtime" 90
start map_freeze "exec setsid ros2 launch rescue_bringup map_freeze.launch.py profile_path:=$P field_path:=$F output_dir:=$RUN/freeze"
wait_for map_freeze "START_POSE captured robot_a" 60

# 6. TB-5 (런북 5단계) + 두 로봇 깊이 점·터널 판정
start frozen_loc_b "exec setsid ros2 launch rescue_bringup frozen_localization.launch.py profile_path:=$P field_path:=$F robot_id:=robot_b frozen_pose_wait_s:=10.0"
for r in robot_a robot_b; do
    start "passage_$r" "exec setsid ros2 launch rescue_bringup passage_points.launch.py profile_path:=$P field_path:=$F robot_id:=$r"
    start "overhead_$r" "exec setsid ros2 launch rescue_bringup overhead_passage.launch.py profile_path:=$P field_path:=$F robot_id:=$r"
done
wait_for frozen_loc_b "START_ALIGN_WAITING" 60

# 6b. 두 로봇을 한 RViz 창에(재범 「제일 중요」, 2026-10-16): 현장과 같은 tf_merge.py + two_robots.rviz(#414)를 현장 도메인에서
#     이 PC 화면에 띄운다. 터널 진입 금지 마스크(/rescue/passage_keepout/mask)를 Map 표시로 더한 사본을 회차 폴더에 쓴다
#     (rviz_keepout.py; 현장 파일은 그대로). SIM_RVIZ=off 면 띄우지 않는다. 창을 닫아도 회차는 계속된다.
if [ "${SIM_RVIZ:-on}" = on ]; then
    mkdir -p "$RUN/viz"
    python3 "$HERE/rviz_keepout.py" "$REPO/scripts/field/noweb/two_robots.rviz" "$RUN/viz/two_robots_keepout.rviz" \
        || die "RViz 설정 사본을 못 만들었습니다"
    start tf_merge "exec setsid python3 $REPO/scripts/field/noweb/tf_merge.py --robots robot6 robot5"
    env -i HOME="$HOME" PATH=/usr/bin:/bin DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" \
        WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
        bash -c "$FIELD_ENV; exec setsid rviz2 -d $RUN/viz/two_robots_keepout.rviz" > "$RUN/logs/rviz.log" 2>&1 < /dev/null &
    echo $! > "$RUN/pids/rviz"
    say "띄움 rviz 두 로봇 한 창(tf_merge + two_robots.rviz + 진입 금지 마스크, 로그 $RUN/logs/rviz.log)"
elif [ "${SIM_RVIZ}" != off ]; then
    die "SIM_RVIZ must be on or off"
fi

# 7. 조정기 (런북 3단계 마지막 줄; 값은 런북 자리표시, camera_watch_roi 는 웹캠이 없어 예시값)
start coordinator "exec setsid ros2 launch rescue_bringup coordinator.launch.py profile_path:=$P field_path:=$F \
route_segment_goals:='[0.0, 0.0, 0.0]' observe_goal:='[0.0, 0.0, 0.0]' route_segment_passage_free:='[false]' \
camera_watch_roi:='[0.1, 0.2, 0.6, 0.9]'"
wait_for coordinator "central live clock: SYNCED" 60

# 8. 인형 인식 대역: target_locator(실물, amr_target 없이) + scripts/sim/fake_target.py(정답 위치 없이 도면 인형 자리와
#    로봇 자신의 TF·SLAM 지도로 보임 판정). 인형 자리는 평면도 값 [추정].
call "python3 - $P" > "$RUN/target_locator.params.yaml" <<'PY' || die "target_locator 파라미터를 못 만들었습니다"
import json, sys, time
from rescue_bringup.profile import load_deployment, require_map_basis
from rescue_perception.target_locator import profile_parameters
d = load_deployment(sys.argv[1]); now = time.time_ns()
v = profile_parameters(d, require_map_basis(d), tf_wait_s=1.0, clock_domain_id='rokey7-utc-field',
    alignment_authority_id='tb6-saved-map-identity', alignment_valid_from_ns=now - 3_000_000_000,
    alignment_valid_until_ns=now + 3_600_000_000_000)
print(json.dumps({'target_locator': {'ros__parameters': v}}, indent=1))
PY
start target_locator "exec setsid ros2 run rescue_perception target_locator --ros-args --params-file $RUN/target_locator.params.yaml \
-r /tf:=/robot6/tf -r /tf_static:=/robot6/tf_static"
wait_for target_locator "MapState VALID" 60
start fake_target "exec setsid python3 $HERE/fake_target.py --profile $P --field $F --doll ${DOLL[*]} \
--scout-start ${TB6_UNDOCKED[*]} --camera-info-topic /robot6/oakd/rgb/camera_info --rgb-optical-frame oakd_rgb_camera_optical_frame"
wait_for fake_target "FAKE_TARGET start_pose" 60

# 9. 늦게 뜬 실행기에 상태 다시 보내기 -> TB-6 arm
call "timeout 30 ros2 service call /mission_coordinator/refresh_state std_srvs/srv/Trigger {}" > "$RUN/logs/refresh.log" 2>&1
for i in 1 2 3; do
    call "timeout 40 ros2 service call /robot6/rescue_command_adapter/task_arm std_srvs/srv/Trigger {}" > "$RUN/logs/arm_robot6.log" 2>&1
    grep -q "ARMED" "$RUN/logs/arm_robot6.log" && break
    sleep 10
done
grep -q "ARMED" "$RUN/logs/arm_robot6.log" && say "TB-6 ARMED" || say "TB-6 arm 실패: $(grep -o "message='[^']*'" "$RUN/logs/arm_robot6.log")"
# 10. 로봇 감독·각 PC 담당이 회차 중에 하는 런북 7단계 조작을 런북 명령 그대로 대신한다(sim_operator.sh, 로그 operator.log).
start operator "exec setsid bash $HERE/sim_operator.sh $RUN $F $P"
say "준비 끝. 관제 웹 http://$web_host:$web_port 에서 재난 선포. 끄기: $HERE/stop_full.sh $RUN"

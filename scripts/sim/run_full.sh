#!/bin/bash
# 시뮬레이션 2단계: 실기 한 회차를 PC 한 대에서 (가제보_실습, 2026-10-15, 재범 지시 「hv-06·hv-03·mu-03·로봇 역할을 한 PC 로」).
#
#   로봇 Pi 두 대  -> sim_robots.sh (Gazebo, TB-6·TB-5, 도메인 SIM_DOMAIN)
#   Wi-Fi          -> wall_clock_relay.py (시각·프레임·카메라 이름을 실기와 같게, SIM_DOMAIN <-> FIELD_DOMAIN)
#   Discovery PC   -> 없음. 한 PC 안이라 LOCALHOST 탐색 + config/sim/fastdds_localhost.xml
#   운영 PC        -> operations.launch.py + 관제 웹 + coordinator.launch.py
#   로봇 담당 PC   -> tb6_slam, robot.launch(두 벌), map_freeze, frozen_localization(TB-5), passage_points, overhead_passage
# 순서는 docs/process/field-runbook.md 3~7단계 그대로다. 실행기는 조정기와 같은 PC 이면 거절하므로 두 실행기에만
# scripts/field/wiring-rehearsal/host_shim 을 PYTHONPATH 앞에 둔다(10-14 한 PC 리허설과 같음, robot.launch 전체에
# 걸리지만 그 shim 은 REHEARSAL_HOST_ID 가 있을 때 clock_measurement 의 host_id 만 바꾼다).
# 로봇은 지도를 모른다: 실제 지도·Gazebo 정답 위치는 어느 노드에도 주지 않는다.
#
# 사용: run_full.sh --world <이름> --world-dir <폴더> --web-host <관제 웹 바인드 주소> [--web-port 8080] [--headless]
#        관리자 비밀번호는 환경 변수 RUN_FULL_ADMIN_PASSWORD(8자 이상)로만 받는다(저장소·로그에 남기지 않음).
#       끄기: stop_full.sh <회차 폴더>
# 재난 선포는 하지 않는다. 운영자가 관제 웹에서 누른다(실기와 같음). 인형 인식은 아직 없다(가짜 표적 주입기 후속).
set -u
SIM_DOMAIN=79
FIELD_DOMAIN=78
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
die() { echo "run_full: $*" >&2; exit 2; }
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
[ "${#RUN_FULL_ADMIN_PASSWORD}" -ge 8 ] || die "RUN_FULL_ADMIN_PASSWORD(8자 이상)가 필요합니다"
[ -f "$REPO/install/setup.bash" ] || die "$REPO/install 이 없습니다(colcon build --packages-up-to rescue_bringup rescue_console)"
[ -x "${RUN_FULL_WEB_PYTHON:-$HOME/venvs/sim_web/bin/python}" ] || die "Flask 가 있는 python 이 없습니다(RUN_FULL_WEB_PYTHON)"
WEB_PY="${RUN_FULL_WEB_PYTHON:-$HOME/venvs/sim_web/bin/python}"

RUN="${RUN_FULL_DIR:-$HOME/sim-runs/full-$(date +%Y%m%d-%H%M%S)}"
[ -e "$RUN" ] && die "$RUN 이 이미 있습니다(회차마다 새 폴더)"
mkdir -p "$RUN/logs" "$RUN/maps" "$RUN/freeze"
TOKEN="run_full.$$.$(date +%s)"
echo "$TOKEN" > "$RUN/token"
F="$REPO/config/field/search_area.field.json"
P="$RUN/profile/profile.json"
SHIM="$REPO/scripts/field/wiring-rehearsal/host_shim"
DDS="$REPO/config/sim/fastdds_localhost.xml"
BASE_ENV="set +u; source /opt/ros/jazzy/setup.bash; source $REPO/install/setup.bash; set -u
export RUN_FULL_TOKEN=$TOKEN ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTRTPS_DEFAULT_PROFILES_FILE=$DDS ROS_SUPER_CLIENT=False"
FIELD_ENV="$BASE_ENV; export ROS_DOMAIN_ID=$FIELD_DOMAIN"

# 하나를 터미널 없이 띄운다. 로그는 $RUN/logs/<이름>.log, 이 회차 표시(RUN_FULL_TOKEN)로 나중에 PID 로 끈다.
start() {
    local name="$1"; shift
    env -i HOME="$HOME" PATH=/usr/bin:/bin bash -c "$FIELD_ENV; $*" > "$RUN/logs/$name.log" 2>&1 < /dev/null &
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
python3 "$REPO/scripts/field/make_field_profile.py" --out "$RUN/profile" --discovery 127.0.0.1:11811 \
    --domain "$FIELD_DOMAIN" > "$RUN/logs/profile.log" 2>&1 || die "프로파일을 못 만들었습니다. $RUN/logs/profile.log"
RESCUE_CONSOLE_ADMIN_PASSWORD="$RUN_FULL_ADMIN_PASSWORD" PYTHONPATH="$REPO/src/rescue_console:$REPO/src/rescue_contracts" \
    python3 -m rescue_console.accounts init "$RUN/accounts.sqlite3" --admin admin > "$RUN/logs/accounts.log" 2>&1 \
    || die "계정 DB 를 못 만들었습니다"
python3 -c 'import secrets; print(secrets.token_urlsafe(32))' > "$RUN/secret"; chmod 600 "$RUN/secret"

# 1. 로봇 쪽(Gazebo)
env -i HOME="$HOME" PATH=/usr/bin:/bin RUN_FULL_TOKEN="$TOKEN" RUN_SIM_LOG_DIR="$RUN/logs/robots" \
    DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
    setsid "$HERE/sim_robots.sh" --world "$world" --world-dir "$world_dir" --x 0.88 --y 0.37 --yaw -1.5708 \
    --also robot5 2.70 0.37 -1.5708 $headless > "$RUN/logs/robots.log" 2>&1 < /dev/null &
say "띄움 robots (로그 $RUN/logs/robots.log)"
wait_for robots "떴습니다" 240

# 2. 중계
start relay "exec setsid python3 $HERE/wall_clock_relay.py --sim-domain $SIM_DOMAIN --field-domain $FIELD_DOMAIN --namespace robot6 --namespace robot5"
wait_for relay "RELAY robot5" 30

# 3. 운영 + 관제 웹 (런북 3단계)
start operations "exec setsid ros2 launch rescue_bringup operations.launch.py profile_path:=$P db_path:=$RUN/site_state.sqlite3 \
api_port:=18180 command_timeout_s:=3.0 report_db_path:=$RUN/reports.sqlite3 envelope_schema_version:=rescue-envelope/1 \
report_publisher_id:=ops.report map_topic:=/robot6/map map_dir:=$RUN/maps robot_namespaces:=robot_a=/robot6,robot_b=/robot5 \
robot_watch_path:=$RUN/robot_watch.json"
wait_for operations "rescue_db_writer git=" 60
start console "export RESCUE_CONSOLE_SECRET=\$(cat $RUN/secret); exec setsid $WEB_PY -m rescue_console --accounts $RUN/accounts.sqlite3 \
--site-state $RUN/site_state.sqlite3 --operations http://127.0.0.1:18180 --robot-name robot_a=TB-6 --robot-name robot_b=TB-5 \
--host $web_host --port $web_port --live-map $RUN/maps/live_map.yaml --live-map-stale-s 15 --robot-watch $RUN/robot_watch.json \
--robot-watch-stale-s 5 --robot-watch-map-robot robot_a --support-dispatch AUTO --search-start AUTO"
wait_for console "Running on" 30

# 4. TB-6 (런북 4단계): SLAM -> 로봇 launch -> 지도 고정(TB-6 이 움직이기 전)
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

# 5. TB-5 (런북 5단계) + 두 로봇 깊이 점·터널 판정
start frozen_loc_b "exec setsid ros2 launch rescue_bringup frozen_localization.launch.py profile_path:=$P field_path:=$F robot_id:=robot_b frozen_pose_wait_s:=10.0"
for r in robot_a robot_b; do
    start "passage_$r" "exec setsid ros2 launch rescue_bringup passage_points.launch.py profile_path:=$P field_path:=$F robot_id:=$r"
    start "overhead_$r" "exec setsid ros2 launch rescue_bringup overhead_passage.launch.py profile_path:=$P field_path:=$F robot_id:=$r"
done
wait_for frozen_loc_b "START_ALIGN_WAITING" 60

# 6. 조정기 (런북 3단계 마지막 줄; 값은 런북 자리표시, camera_watch_roi 는 웹캠이 없어 예시값)
start coordinator "exec setsid ros2 launch rescue_bringup coordinator.launch.py profile_path:=$P field_path:=$F \
route_segment_goals:='[0.0, 0.0, 0.0]' observe_goal:='[0.0, 0.0, 0.0]' route_segment_passage_free:='[false]' \
camera_watch_roi:='[0.1, 0.2, 0.6, 0.9]'"
wait_for coordinator "central live clock: SYNCED" 60

# 7. 늦게 뜬 실행기에 상태 다시 보내기 -> 두 로봇 언도킹(로봇 감독 몫, 시뮬레이션 도메인) -> TB-6 arm
call "timeout 30 ros2 service call /mission_coordinator/refresh_state std_srvs/srv/Trigger {}" > "$RUN/logs/refresh.log" 2>&1
for ns in robot6 robot5; do
    env -i HOME="$HOME" PATH=/usr/bin:/bin bash -c "$BASE_ENV; export ROS_DOMAIN_ID=$SIM_DOMAIN; \
timeout 90 ros2 action send_goal /$ns/undock irobot_create_msgs/action/Undock '{}'" > "$RUN/logs/undock_$ns.log" 2>&1
    grep -q "SUCCEEDED" "$RUN/logs/undock_$ns.log" && say "언도킹 $ns" || say "언도킹 $ns 실패(로그 $RUN/logs/undock_$ns.log)"
done
for i in 1 2 3; do
    call "timeout 40 ros2 service call /robot6/rescue_command_adapter/task_arm std_srvs/srv/Trigger {}" > "$RUN/logs/arm_robot6.log" 2>&1
    grep -q "ARMED" "$RUN/logs/arm_robot6.log" && break
    sleep 10
done
grep -q "ARMED" "$RUN/logs/arm_robot6.log" && say "TB-6 ARMED" || say "TB-6 arm 실패: $(grep -o "message='[^']*'" "$RUN/logs/arm_robot6.log")"
say "준비 끝. 관제 웹 http://$web_host:$web_port 에서 재난 선포. 끄기: $HERE/stop_full.sh $RUN"

#!/bin/bash
# 시뮬레이션 2단계 「로봇 쪽」: Gazebo world 하나에 TB-6·TB-5 를 세운다. 실기의 로봇 Pi 두 대 역할이다(2026-10-15).
# 우리 노드(운영·SLAM·로봇 launch·조정기)는 이 도메인에 붙지 않는다. wall_clock_relay.py 가 이 도메인(시뮬레이션 시각)
# 과 현장 도메인(벽시계) 사이를 잇는다. SLAM·RViz 는 띄우지 않는다(1단계 run_sim.sh 와 다른 점).
#
# 사용: scripts/sim/sim_robots.sh --world <이름> --world-dir <폴더> --x <m> --y <m> --yaw <rad> [--namespace robot6]
#        [--also <namespace> <x> <y> <yaw>]... [--headless] [--check-only]
# --headless: Gazebo 서버만(화면 없이) 띄운다. 화면은 나중에 같은 GZ_PARTITION 으로 `gz sim -g` 를 붙인다.
# 지키는 것: 깨끗한 셸(env -i), 도메인 SIM_DOMAIN_ID·LOCALHOST·DS 없음·GZ_PARTITION, 실기 회차가 돌면 안 띄움, 남은 Gazebo
# 가 있으면 안 띄움, 이 실행이 띄운 프로세스만 PID 로 끔. 실제 지도를 받는 인자 없음. 출발 자리는 꼭 받음.
set -u

SIM_DOMAIN_ID=79   # 로봇 쪽(시뮬레이션 시각). 현장 쪽은 78, 1단계 지도 자리는 77 (작전 2026-10-15)
SIM_GZ_PARTITION=sim_full   # 1단계 Gazebo 와 섞이지 않게 (작전 2026-10-15)
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"

die() { echo "sim_robots: $*" >&2; exit 2; }

# 렌더링은 NVIDIA 로 한다(PRIME offload). 빼면 EGL 이 드라이버를 못 찾아 소프트웨어로 그리고, 가상 라이다가 모든 방향
# 0.164 m(range_min)를 내며 실시간의 22% 로 느려졌다 [측정 2026-10-15 mu-03, NVIDIA 노트북 GPU + Intel 내장].
# 깨끗한 셸로 다시 실행한다. 부른 셸의 ROS 환경(~/turtlebot4_ws 등)은 하나도 넘기지 않는다.
if [ -z "${RUN_SIM_CLEAN:-}" ]; then
    exec env -i HOME="$HOME" PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
        LANG="${LANG:-C.UTF-8}" DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" \
        WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
        RUN_SIM_CLEAN=1 RUN_SIM_LOG_DIR="${RUN_SIM_LOG_DIR:-}" RUN_FULL_TOKEN="${RUN_FULL_TOKEN:-}" \
        __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
        __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json bash "$0" "$@"
fi

world=""; world_dir=""; x=""; y=""; yaw=""; namespace="robot6"; check_only=false; headless=false; also=()
while [ $# -gt 0 ]; do
    case "$1" in
        --world) world="${2:-}"; shift 2 || die "--world 값이 없습니다" ;;
        --world-dir) world_dir="${2:-}"; shift 2 || die "--world-dir 값이 없습니다" ;;
        --x) x="${2:-}"; shift 2 || die "--x 값이 없습니다" ;;
        --y) y="${2:-}"; shift 2 || die "--y 값이 없습니다" ;;
        --yaw) yaw="${2:-}"; shift 2 || die "--yaw 값이 없습니다" ;;
        --namespace) namespace="${2:-}"; shift 2 || die "--namespace 값이 없습니다" ;;
        --also) [ $# -ge 5 ] || die "--also 에는 <namespace> <x> <y> <yaw> 네 값이 필요합니다"
                also+=("$2 $3 $4 $5"); shift 5 ;;
        --check-only) check_only=true; shift ;;
        --headless) headless=true; shift ;;
        *) die "알 수 없는 인자 '$1' (실제 지도를 받는 인자는 없습니다)" ;;
    esac
done

num='^-?[0-9]+(\.[0-9]+)?$'
[ -n "$world" ] || die "--world 가 필요합니다"
[[ "$world" =~ ^[a-z0-9_]+$ ]] || die "world 이름은 소문자·숫자·밑줄만 씁니다: $world"
for pair in "x:$x" "y:$y" "yaw:$yaw"; do
    [[ "${pair#*:}" =~ $num ]] || die "--${pair%%:*} 는 숫자여야 하고 꼭 줘야 합니다(출발 자리는 지어내지 않습니다)"
done
[[ "$namespace" =~ ^[a-z][a-z0-9_]*$ ]] || die "--namespace 에는 슬래시를 붙이지 않습니다(bridge 가 실패합니다): $namespace"
seen=" $namespace "
for spec in "${also[@]}"; do
    read -r ans ax ay ayaw <<< "$spec"
    [[ "$ans" =~ ^[a-z][a-z0-9_]*$ ]] || die "--also namespace 에는 슬래시를 붙이지 않습니다: $ans"
    case "$seen" in *" $ans "*) die "namespace 가 겹칩니다: $ans" ;; esac
    seen="$seen$ans "
    for v in "$ax" "$ay" "$ayaw"; do [[ "$v" =~ $num ]] || die "--also $ans 의 x y yaw 는 숫자여야 합니다"; done
done

[ -n "$world_dir" ] || die "--world-dir 가 필요합니다"
world_file="$world_dir/$world.sdf"
[ -f "$world_file" ] || die "world 파일이 없습니다: $world_file"
grep -Eq "<world name=['\"]$world['\"]" "$world_file" || die "$world_file 의 <world name> 이 '$world' 와 다릅니다(bridge 토픽 이름이 어긋납니다)"

# 환경을 정해 둔 값으로 못 박는다.
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT ROS_STATIC_PEERS ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID="$SIM_DOMAIN_ID" ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION="$SIM_GZ_PARTITION"
# 시뮬레이터 프로세스가 많아 LOCALHOST 탐색 범위를 넓힌다(fastdds_localhost.xml 주석). 다른 터미널에서 조회할 때도 같은 값을 쓴다.
export FASTRTPS_DEFAULT_PROFILES_FILE="$REPO/scripts/sim/fastdds_localhost.xml"
[ -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ] || die "$FASTRTPS_DEFAULT_PROFILES_FILE 가 없습니다"

# /proc 읽기. 다른 사용자(root) 프로세스의 environ 은 못 읽으니 오류를 버린다. grep 한 번으로 모두 훑는다
# (프로세스마다 tr 을 부르면 느리고, 리다이렉션 오류가 화면에 쏟아졌다 2026-10-15).
pids_with_env() { grep -lsz -x "$@" /proc/[0-9]*/environ 2>/dev/null | cut -d/ -f3; }
proc_cmd() { { tr '\0' ' ' < "/proc/$1/cmdline"; } 2>/dev/null; }

# 이 스크립트를 부른 쪽(조상 프로세스)은 검사에서 뺀다. 명령줄에 ros2 가 들어 있어도 실기가 아니다.
ancestors=" "
pid=$$
while [ "${pid:-0}" -gt 1 ]; do
    ancestors="$ancestors$pid "
    pid="$(awk '{print $4}' "/proc/$pid/stat" 2>/dev/null)"
done

for pid in $(pids_with_env -E 'ROS_DISCOVERY_SERVER=.+'); do
    case "$ancestors" in *" $pid "*) continue ;; esac
    cmd="$(proc_cmd "$pid")"
    case "${cmd%% *}" in   # 셸은 빼고 실제 프로그램만(명령줄 글자에 rescue_ 가 들어간 셸을 실기로 잘못 본 적이 있다 2026-10-15)
        */bash|bash|*/sh|sh|*/zsh|zsh|*/env|env|*/timeout|timeout) continue ;;
    esac
    case "$cmd" in
        *bin/ros2\ *|*rescue_*) die "디스커버리 서버를 쓰는 ROS 프로세스가 돌고 있습니다(실기 회차로 봅니다, PID $pid: ${cmd:0:80}). 실기 중에는 Gazebo 를 띄우지 않습니다" ;;
    esac
done

# 같은 GZ_PARTITION 의 Gazebo 만 막는다. 다른 자리(1단계 지도 자리 sim_map)의 Gazebo 는 따로 돈다.
for pid in $(pids_with_env -F "GZ_PARTITION=$SIM_GZ_PARTITION"); do
    case " $(proc_cmd "$pid")" in
        " gz sim "*|*" gz sim "*) ;;
        *) continue ;;
    esac
    case "$ancestors" in *" $pid "*) continue ;; esac
    die "이전 Gazebo(GZ_PARTITION=$SIM_GZ_PARTITION)가 남아 있습니다(PID $pid). PID 로 끈 뒤 다시 하세요"
done

if $check_only; then
    echo "CHECK_OK world=$world namespace=$namespace x=$x y=$y yaw=$yaw domain=$ROS_DOMAIN_ID range=$ROS_AUTOMATIC_DISCOVERY_RANGE"
    for spec in "${also[@]}"; do echo "ALSO $spec"; done
    echo "ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER:-<unset>}"
    exit 0
fi

[ -f /opt/ros/jazzy/setup.bash ] || die "/opt/ros/jazzy 가 없습니다"
# ROS setup 파일은 정의 안 된 변수를 읽으므로 source 하는 동안만 set -u 를 끈다.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash || die "ROS source 에 실패했습니다"
set -u
ros2 pkg prefix turtlebot4_gz_bringup >/dev/null 2>&1 || die "turtlebot4_gz_bringup 이 없습니다(ros-jazzy-turtlebot4-simulator)"

# 이 실행이 띄운 프로세스를 표시한다. 끌 때 이 표시가 있는 PID 만 끈다.
export RUN_SIM_TOKEN="sim_robots.$$.$(date +%s)"
log_dir="${RUN_SIM_LOG_DIR:-$HOME/sim-logs/$(date +%Y%m%d-%H%M%S)}"
mkdir -p "$log_dir" || die "로그 폴더를 못 만듭니다: $log_dir"

pids=()
tagged_pids() { pids_with_env -F "RUN_SIM_TOKEN=$RUN_SIM_TOKEN" | grep -vx "$$"; }
cleanup() {
    # 창을 닫으면 HUP 이 두 번 온다. 정리 중에는 어떤 신호로도 끊기지 않게 한다.
    trap '' EXIT INT TERM HUP
    local p left i
    echo "sim_robots: 끝내는 중"
    for p in "${pids[@]}"; do kill -INT "$p" 2>/dev/null; done
    for i in $(seq 1 20); do
        left="$(tagged_pids | tr '\n' ' ')"
        [ -z "${left// /}" ] && break
        sleep 0.5
    done
    for p in $(tagged_pids); do kill -TERM "$p" 2>/dev/null; done
    sleep 2
    for p in $(tagged_pids); do kill -KILL "$p" 2>/dev/null; done
    echo "sim_robots: 끝. 로그 $log_dir"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
trap 'exit 129' HUP   # 터미널 창을 닫아도 정리한다

echo "sim_robots: world=$world namespace=$namespace x=$x y=$y yaw=$yaw 도메인=$ROS_DOMAIN_ID 로그=$log_dir"

# gz 는 world 파일을 현재 폴더에서 '<이름>.sdf' 로 찾는다(sim.launch.py 가 world 이름에 .sdf 를 붙인다).
if $headless; then
    # sim.launch.py 가 하던 것을 직접 한다: 모델 자원 경로, gz 서버(-s), /clock bridge. 로봇은 spawn launch 로.
    share=/opt/ros/jazzy/share
    export GZ_SIM_RESOURCE_PATH="$share/turtlebot4_gz_bringup/worlds:$share/irobot_create_gz_bringup/worlds:$share"
    ( cd "$world_dir" && exec setsid gz sim -s -r -v 4 "$world.sdf" ) > "$log_dir/gz.log" 2>&1 &
    pids+=($!)
    setsid ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
        --ros-args -r __node:=clock_bridge > "$log_dir/clock.log" 2>&1 &
    pids+=($!)
    setsid ros2 launch turtlebot4_gz_bringup turtlebot4_spawn.launch.py "world:=$world" "namespace:=$namespace" \
        "x:=$x" "y:=$y" "yaw:=$yaw" rviz:=false > "$log_dir/spawn_$namespace.log" 2>&1 &
    pids+=($!)
    first_log="$log_dir/spawn_$namespace.log"
else
    ( cd "$world_dir" && exec setsid ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py \
        "world:=$world" "namespace:=$namespace" "x:=$x" "y:=$y" "yaw:=$yaw" rviz:=false ) \
        > "$log_dir/gz.log" 2>&1 &
    pids+=($!)
    first_log="$log_dir/gz.log"
fi

# Gazebo 가 world 를 실제로 띄우고 첫 로봇을 만들 때까지 Gazebo 로그로 기다린다.
# 토픽 이름만 보면 안 된다: bridge 가 gz 없이도 토픽을 만들어 두어, gz 가 멈춘 채 「떴습니다」가 된 적이 있다.
# ros2 CLI 로 값을 받아 보는 것도 안 된다: 참가자가 많아 CLI 가 몇 초씩 걸려 확인이 계속 실패했다(2026-10-15).
# Gazebo 로그에는 색 코드가 섞여 있어 지우고 찾는다.
plain() { sed 's/\x1b\[[0-9;]*m//g' "$1" 2>/dev/null; }
created() { plain "$1" | grep -c "Entity creation successful"; }
ready=false
for i in $(seq 1 45); do
    kill -0 "${pids[0]}" 2>/dev/null || die "Gazebo launch 가 먼저 끝났습니다. $log_dir/gz.log 를 보세요"
    if plain "$log_dir/gz.log" | grep -q "Serving world controls on \[/world/$world/control\]" &&
        [ "$(created "$first_log")" -ge 1 ]; then
        ready=true; break
    fi
    sleep 2
done
$ready || die "Gazebo 가 world '$world' 를 띄우지 못했거나 $namespace 를 만들지 못했습니다. $log_dir/gz.log 를 보세요"
echo "sim_robots: Gazebo world '$world' 에 $namespace 생성 확인"

# 더 세울 로봇. spawn_extra.launch.py 는 turtlebot4_spawn 과 같고(로봇 앞 0.157 m 에 도크), 센서 시스템만 뺀
# 모델로 만든다. 로봇마다 센서 시스템이 있으면 두 번째 로봇에서 Gazebo 서버가 죽는다(strip_sensors.py 주석).
for spec in "${also[@]}"; do
    read -r ans ax ay ayaw <<< "$spec"
    echo "sim_robots: 로봇 추가 $ans x=$ax y=$ay yaw=$ayaw"
    setsid ros2 launch "$HERE/spawn_extra.launch.py" "world:=$world" "namespace:=$ans" \
        "x:=$ax" "y:=$ay" "yaw:=$ayaw" > "$log_dir/spawn_$ans.log" 2>&1 &
    pids+=($!)
    for i in $(seq 1 30); do
        [ "$(created "$log_dir/spawn_$ans.log")" -ge 1 ] && break
        sleep 2
    done
    [ "$(created "$log_dir/spawn_$ans.log")" -ge 1 ] || die "$ans 를 Gazebo 에 만들지 못했습니다. $log_dir/spawn_$ans.log 를 보세요"
    echo "sim_robots: $ans 생성 확인"
done

echo "sim_robots: 떴습니다. 끝내려면 Ctrl-C"
echo "         source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE"
# Gazebo 창을 닫으면 gz 서버도 같이 끝난다. 그때 남은 로봇 노드·SLAM·RViz 도 함께 끝낸다.
while kill -0 "${pids[0]}" 2>/dev/null; do
    if plain "$log_dir/gz.log" | grep -q "\[gazebo-1\]: process has finished\|\[gazebo-1\]: process has died"; then
        echo "sim_robots: Gazebo 가 끝났습니다(창을 닫았거나 멈춤). 나머지도 끕니다"
        break
    fi
    sleep 2
done

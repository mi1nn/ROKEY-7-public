#!/bin/bash
# 시뮬레이션 트랙 S2: Gazebo 실습장 world 에 가상 로봇 한 대를 세우고 SLAM 과 RViz 를 띄운다 (2026-10-15 작전 결정).
#
# 사용: scripts/sim/run_sim.sh [--world practice] [--namespace robot6] [--x <m> --y <m> --yaw <rad>] [--no-rviz] [--check-only]
# 출발 자리를 안 주면 <world>.spawn.json 의 robots.<namespace> 를 읽는다. x y yaw 는 셋을 함께 주거나 함께 뺀다.
#
# 환경변수(자리마다 다르게 쓴다. 전체 흐름을 돌리는 실습 자리가 이 스크립트를 그대로 부른다):
#   RUN_SIM_DOMAIN_ID     ROS 도메인. 기본 77(지도 자리). 70~99 만 받는다(실기는 6)
#   RUN_SIM_GZ_PARTITION  gz transport 파티션. 기본 sim_map. 자리마다 달라야 같은 PC 의 두 Gazebo 가 섞이지 않는다
#   RUN_SIM_NAMESPACE     로봇 이름(슬래시 없이). 기본 robot6. --namespace 가 이긴다
#   RUN_SIM_GUI           0 이면 Gazebo 창 없이 서버만(gz sim -s) 띄운다. 기본 1. RViz 는 --no-rviz 로 끈다
#   RUN_SIM_WORLD_DIR     world 폴더(기본 config/sim/worlds). 시험용 임시 world 에 쓴다
#   RUN_SIM_LOG_DIR       로그 폴더(기본 ~/sim-logs/<시각>)
#
# 지키는 것(스크립트가 직접 확인한다):
#   - 실제 로봇과 섞이지 않게 깨끗한 셸(env -i)로 다시 실행한다. 도메인 70~99, LOCALHOST 만, 디스커버리 서버 없음, /opt/ros/jazzy 만 source.
#   - 실기 회차가 도는 것으로 보이면 띄우지 않는다(디스커버리 서버를 쓰는 ROS 프로세스가 있으면 실기로 본다).
#   - 같은 파티션에 남은 Gazebo 가 있으면 띄우지 않는다(다른 파티션의 Gazebo 는 상관하지 않는다).
#     끝낼 때 이 실행이 띄운 프로세스만 PID 로 끈다(pkill -f 안 씀).
#   - 로봇은 지도를 모른다: 실제 지도를 받는 인자가 없고, map_server 도 띄우지 않는다. SLAM 이 만든 /<namespace>/map 만 있다.
#   - 출발 자리는 값을 지어내지 않는다: 인자나 spawn.json 에 없으면 멈춘다.
# --check-only 는 안전 확인과 인자 검사만 하고 끝낸다(ROS 가 없어도 돈다).
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"

die() { echo "run_sim: $*" >&2; exit 2; }

# 깨끗한 셸로 다시 실행한다. 부른 셸의 ROS 환경(~/turtlebot4_ws 등)은 하나도 넘기지 않는다. 넘기는 것은 RUN_SIM_* 뿐이다.
if [ -z "${RUN_SIM_CLEAN:-}" ]; then
    # 라이다는 GPU 로 그려진다. 하이브리드 노트북(Intel + NVIDIA)에서 NVIDIA 를 안 고르면 소프트웨어 그리기로 떨어져
    # 라이다 640 개 값이 모두 range_min(0.164 m)이 되고 SLAM 은 아무것도 못 본다 [측정 2026-10-15 시뮬레이션 PC].
    # NVIDIA 드라이버가 있고 사용자가 그리기 변수를 따로 안 줬으면 NVIDIA 오프로드로 둔다. 끄려면 LIBGL_ALWAYS_SOFTWARE=1.
    if [ -e /dev/nvidiactl ] && [ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ] && [ -z "${LIBGL_ALWAYS_SOFTWARE:-}" ]; then
        export __NV_PRIME_RENDER_OFFLOAD="${__NV_PRIME_RENDER_OFFLOAD:-1}"
        export __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-nvidia}"
        export __EGL_VENDOR_LIBRARY_FILENAMES="${__EGL_VENDOR_LIBRARY_FILENAMES:-/usr/share/glvnd/egl_vendor.d/10_nvidia.json}"
    fi
    # 그리기 장치 선택 변수는 ROS 환경이 아니므로 값이 있을 때만 깨끗한 셸로 넘긴다.
    gl_env=()
    for v in __NV_PRIME_RENDER_OFFLOAD __GLX_VENDOR_LIBRARY_NAME __EGL_VENDOR_LIBRARY_FILENAMES __VK_LAYER_NV_optimus \
             LIBGL_ALWAYS_SOFTWARE MESA_LOADER_DRIVER_OVERRIDE; do
        [ -n "${!v:-}" ] && gl_env+=("$v=${!v}")
    done
    exec env -i "${gl_env[@]}" HOME="$HOME" PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
        LANG="${LANG:-C.UTF-8}" DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" \
        WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
        RUN_SIM_CLEAN=1 RUN_SIM_WORLD_DIR="${RUN_SIM_WORLD_DIR:-}" RUN_SIM_LOG_DIR="${RUN_SIM_LOG_DIR:-}" \
        RUN_SIM_DOMAIN_ID="${RUN_SIM_DOMAIN_ID:-}" RUN_SIM_GZ_PARTITION="${RUN_SIM_GZ_PARTITION:-}" \
        RUN_SIM_NAMESPACE="${RUN_SIM_NAMESPACE:-}" RUN_SIM_GUI="${RUN_SIM_GUI:-}" bash "$0" "$@"
fi

sim_domain="${RUN_SIM_DOMAIN_ID:-77}"          # 실기 도메인과 겹치지 않는 값 (AGENTS.md 6절, 작전 2026-10-15)
gz_partition="${RUN_SIM_GZ_PARTITION:-sim_map}"
namespace="${RUN_SIM_NAMESPACE:-robot6}"
gui="${RUN_SIM_GUI:-1}"
world="practice"; x=""; y=""; yaw=""; rviz=true; check_only=false
while [ $# -gt 0 ]; do
    case "$1" in
        --world) world="${2:-}"; shift 2 || die "--world 값이 없습니다" ;;
        --x) x="${2:-}"; shift 2 || die "--x 값이 없습니다" ;;
        --y) y="${2:-}"; shift 2 || die "--y 값이 없습니다" ;;
        --yaw) yaw="${2:-}"; shift 2 || die "--yaw 값이 없습니다" ;;
        --namespace) namespace="${2:-}"; shift 2 || die "--namespace 값이 없습니다" ;;
        --no-rviz) rviz=false; shift ;;
        --check-only) check_only=true; shift ;;
        *) die "알 수 없는 인자 '$1' (실제 지도를 받는 인자는 없습니다)" ;;
    esac
done

num='^-?[0-9]+(\.[0-9]+)?$'
[[ "$sim_domain" =~ ^[0-9]+$ ]] && [ "$sim_domain" -ge 70 ] && [ "$sim_domain" -le 99 ] ||
    die "RUN_SIM_DOMAIN_ID 는 70~99 여야 합니다(실기 도메인과 섞이지 않게): '$sim_domain'"
[[ "$gui" =~ ^[01]$ ]] || die "RUN_SIM_GUI 는 0 이나 1 입니다: '$gui'"
[[ "$gz_partition" =~ ^[a-z][a-z0-9_]*$ ]] || die "RUN_SIM_GZ_PARTITION 은 소문자·숫자·밑줄만 씁니다: '$gz_partition'"
[[ "$world" =~ ^[a-z0-9_]+$ ]] || die "world 이름은 소문자·숫자·밑줄만 씁니다: $world"
[[ "$namespace" =~ ^[a-z][a-z0-9_]*$ ]] || die "namespace 에는 슬래시를 붙이지 않습니다(bridge 가 실패합니다): '$namespace'"

world_dir="${RUN_SIM_WORLD_DIR:-$REPO/config/sim/worlds}"
world_file="$world_dir/$world.sdf"
[ -f "$world_file" ] || die "world 파일이 없습니다: $world_file"
grep -Eq "<world name=['\"]$world['\"]" "$world_file" || die "$world_file 의 <world name> 이 '$world' 와 다릅니다(bridge 토픽 이름이 어긋납니다)"

# 출발 자리: 셋을 함께 주거나, 함께 빼서 spawn.json 에서 읽는다.
given=0; for v in "$x" "$y" "$yaw"; do [ -n "$v" ] && given=$((given + 1)); done
if [ "$given" -eq 0 ]; then
    spawn_file="$world_dir/$world.spawn.json"
    [ -f "$spawn_file" ] || die "--x --y --yaw 를 안 줬고 $spawn_file 도 없습니다(출발 자리는 지어내지 않습니다)"
    spawn="$(python3 -I - "$spawn_file" "$namespace" 2>&1 <<'PY'
import json, sys
robots = json.load(open(sys.argv[1])).get('robots', {})
r = robots.get(sys.argv[2])
if r is None:
    sys.exit('spawn.json 에 %s 가 없습니다(있는 이름: %s)' % (sys.argv[2], ', '.join(sorted(robots)) or '없음'))
print(r['x'], r['y'], r['yaw'])
PY
    )" || die "$spawn"
    read -r x y yaw <<< "$spawn"
elif [ "$given" -ne 3 ]; then
    die "--x --y --yaw 는 셋을 함께 주거나 함께 빼야 합니다"
fi
for pair in "x:$x" "y:$y" "yaw:$yaw"; do
    [[ "${pair#*:}" =~ $num ]] || die "${pair%%:*} 는 숫자여야 합니다: '${pair#*:}'"
done

# 환경을 정해 둔 값으로 못 박는다.
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT ROS_STATIC_PEERS ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID="$sim_domain" ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION="$gz_partition"
# 시뮬레이터 프로세스가 많아 LOCALHOST 탐색 범위를 넓힌다(fastdds_localhost.xml 주석). 다른 터미널에서 조회할 때도 같은 값을 쓴다.
export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/fastdds_localhost.xml"
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
    case "$cmd" in
        *bin/ros2\ *|*rescue_*) die "디스커버리 서버를 쓰는 ROS 프로세스가 돌고 있습니다(실기 회차로 봅니다, PID $pid: ${cmd:0:80}). 실기 중에는 Gazebo 를 띄우지 않습니다" ;;
    esac
done

# 같은 파티션의 Gazebo 만 막는다. 다른 파티션(다른 자리의 시뮬레이션)과는 transport 가 섞이지 않는다.
for pid in $(pids_with_env -x "GZ_PARTITION=$gz_partition"); do
    case " $(proc_cmd "$pid")" in
        " gz sim "*|*" gz sim "*) ;;
        *) continue ;;
    esac
    case "$ancestors" in *" $pid "*) continue ;; esac
    die "파티션 $gz_partition 에 이전 Gazebo 가 남아 있습니다(PID $pid). PID 로 끈 뒤 다시 하세요"
done

if $check_only; then
    echo "CHECK_OK world=$world namespace=$namespace x=$x y=$y yaw=$yaw domain=$ROS_DOMAIN_ID range=$ROS_AUTOMATIC_DISCOVERY_RANGE partition=$GZ_PARTITION gui=$gui"
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
export RUN_SIM_TOKEN="run_sim.$$.$(date +%s)"
log_dir="${RUN_SIM_LOG_DIR:-$HOME/sim-logs/$(date +%Y%m%d-%H%M%S)}"
mkdir -p "$log_dir" || die "로그 폴더를 못 만듭니다: $log_dir"

pids=()
tagged_pids() { pids_with_env -F "RUN_SIM_TOKEN=$RUN_SIM_TOKEN" | grep -vx "$$"; }
cleanup() {
    # 창을 닫으면 HUP 이 두 번 온다. 정리 중에는 어떤 신호로도 끊기지 않게 한다.
    trap '' EXIT INT TERM HUP
    local p left i
    echo "run_sim: 끝내는 중"
    for p in "${pids[@]}"; do kill -INT "$p" 2>/dev/null; done
    for i in $(seq 1 20); do
        left="$(tagged_pids | tr '\n' ' ')"
        [ -z "${left// /}" ] && break
        sleep 0.5
    done
    for p in $(tagged_pids); do kill -TERM "$p" 2>/dev/null; done
    sleep 2
    for p in $(tagged_pids); do kill -KILL "$p" 2>/dev/null; done
    echo "run_sim: 끝. 로그 $log_dir"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
trap 'exit 129' HUP   # 터미널 창을 닫아도 정리한다

echo "run_sim: world=$world namespace=$namespace x=$x y=$y yaw=$yaw 도메인=$ROS_DOMAIN_ID 파티션=$GZ_PARTITION 로그=$log_dir"

# gz 는 world 파일을 현재 폴더에서 '<이름>.sdf' 로 찾는다(sim.launch.py 가 world 이름에 .sdf 를 붙인다).
if [ "$gui" = 1 ]; then
    launch_cmd=(ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py)
else
    launch_cmd=(ros2 launch "$HERE/headless_gz.launch.py")
fi
( cd "$world_dir" && exec setsid "${launch_cmd[@]}" \
    "world:=$world" "namespace:=$namespace" "x:=$x" "y:=$y" "yaw:=$yaw" $([ "$gui" = 1 ] && echo rviz:=false) ) \
    > "$log_dir/gz.log" 2>&1 &
pids+=($!)

# Gazebo 가 world 를 실제로 띄우고 로봇을 만들 때까지 Gazebo 로그로 기다린다.
# 토픽 이름만 보면 안 된다: bridge 가 gz 없이도 토픽을 만들어 두어, gz 가 멈춘 채 「떴습니다」가 된 적이 있다.
# ros2 CLI 로 값을 받아 보는 것도 안 된다: 참가자가 많아 CLI 가 몇 초씩 걸려 확인이 계속 실패했다(2026-10-15).
# Gazebo 로그에는 색 코드가 섞여 있어 지우고 찾는다.
plain() { sed 's/\x1b\[[0-9;]*m//g' "$1" 2>/dev/null; }
created() { plain "$1" | grep -c "Entity creation successful"; }
ready=false
for i in $(seq 1 45); do
    kill -0 "${pids[0]}" 2>/dev/null || die "Gazebo launch 가 먼저 끝났습니다. $log_dir/gz.log 를 보세요"
    if plain "$log_dir/gz.log" | grep -q "Serving world controls on \[/world/$world/control\]" &&
        [ "$(created "$log_dir/gz.log")" -ge 1 ]; then
        ready=true; break
    fi
    sleep 2
done
$ready || die "Gazebo 가 world '$world' 를 띄우지 못했거나 $namespace 를 만들지 못했습니다. $log_dir/gz.log 를 보세요"
echo "run_sim: Gazebo world '$world' 에 $namespace 생성 확인"

setsid ros2 launch turtlebot4_navigation slam.launch.py use_sim_time:=true "namespace:=$namespace" \
    > "$log_dir/slam.log" 2>&1 &
pids+=($!)

if $rviz; then
    # 위에서 본 2D, 밝은 배경. 맵 프레임은 SLAM 이 만든 map, 토픽은 /<namespace>/ 아래, tf 는 /<namespace>/tf 로 바꿔 읽는다.
    # 토픽 이름이 /robot6/ 로 박혀 있다: 다른 이름의 로봇은 RViz 안에서 토픽을 바꾼다.
    setsid rviz2 -d "$HERE/sim_map.rviz" --ros-args -p use_sim_time:=true \
        -r "/tf:=/$namespace/tf" -r "/tf_static:=/$namespace/tf_static" > "$log_dir/rviz.log" 2>&1 &
    pids+=($!)
fi

echo "run_sim: 떴습니다. 끝내려면 Ctrl-C"
echo "run_sim: 다른 터미널에서 조회: env -i HOME=\$HOME PATH=/usr/bin:/bin bash --norc, 그 안에서"
echo "         source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE"
# Gazebo 창을 닫으면 gz 서버도 같이 끝난다. 그때 남은 로봇 노드·SLAM·RViz 도 함께 끝낸다.
while kill -0 "${pids[0]}" 2>/dev/null; do
    if plain "$log_dir/gz.log" | grep -q "\[gazebo-1\]: process has finished\|\[gazebo-1\]: process has died"; then
        echo "run_sim: Gazebo 가 끝났습니다(창을 닫았거나 멈춤). 나머지도 끕니다"
        break
    fi
    sleep 2
done

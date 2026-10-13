#!/usr/bin/env bash
# Stop one robot now (OPS-183/184). Run on any PC that sees the robot, with ROS sourced:
#   bash scripts/field/stop_robot.sh /robot6
# 1. executor task_stop  2. cancel every Nav2 goal (empty CancelGoal = all goals)
# 3. zero <ns>/cmd_vel a few times  4. list what is still running for <ns> (nodes, processes here).
# Sends no motion. If the robot still moves: lift it (people first), then kill what step 4 lists.
set -u
ns="${1:-}"
case "$ns" in
  /*) ;;
  *) echo "usage: stop_robot.sh </robot6|/robot5>  (absolute namespace)" >&2; exit 2 ;;
esac
ns="${ns%/}"
wait_s="${STOP_WAIT_S:-5}"

services="$(timeout "$wait_s" ros2 service list --no-daemon 2>/dev/null || true)"
has() { printf '%s\n' "$services" | grep -qxF "$1"; }

call() {   # call <service> <type> <label>
  if ! has "$1"; then
    echo "[없음] $3: $1 서비스가 보이지 않음"
    return
  fi
  if out="$(timeout "$wait_s" ros2 service call "$1" "$2" '{}' 2>&1)"; then
    echo "[보냄] $3: $1 -> $(printf '%s' "$out" | tr '\n' ' ' | sed 's/  */ /g' | cut -c1-160)"
  else
    echo "[실패] $3: $1 응답 없음(${wait_s} s)"
  fi
}

call "$ns/rescue_command_adapter/task_stop" std_srvs/srv/Trigger "실행기 task_stop"
for action in navigate_to_pose navigate_through_poses follow_waypoints; do
  call "$ns/$action/_action/cancel_goal" action_msgs/srv/CancelGoal "Nav2 $action 목표 전부 취소"
done

if timeout "$wait_s" ros2 topic pub --times 5 --rate 10 "$ns/cmd_vel" geometry_msgs/msg/TwistStamped '{}' >/dev/null 2>&1; then
  echo "[보냄] cmd_vel 0: $ns/cmd_vel TwistStamped 0 x5"
else
  echo "[실패] cmd_vel 0: $ns/cmd_vel 에 보내지 못함(구독자 없음 또는 ${wait_s} s 초과)"
fi

echo "--- 아직 떠 있는 $ns 노드(ros2 node list --no-daemon)"
timeout "$wait_s" ros2 node list --no-daemon 2>/dev/null | grep -E "^$ns/" || echo "(없음 또는 보이지 않음)"
echo "--- 이 PC 의 SLAM·Nav2·explore·rescue 프로세스(PID 경과초 명령)"
left="$(ps -eo pid=,etimes=,args= | grep -E 'slam_toolbox|nav2_|controller_server|planner_server|bt_navigator|behavior_server|collision_monitor|amcl|explore|rescue_robot|ros2 launch' \
  | grep -v -E 'grep|stop_robot\.sh' | cut -c1-200)"
printf '%s\n' "${left:-(없음)}"
echo "끄기: 위 PID 하나씩 kill -INT <PID>, 5 s 뒤에도 남으면 kill -TERM <PID>. 이름으로 넓게 잡아 죽이지 않는다."

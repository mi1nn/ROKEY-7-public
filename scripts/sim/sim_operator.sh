#!/bin/bash
# 시뮬레이션 회차의 「사람 몫」 대역 (가제보 실습, 2026-10-15, 작전 지시): 로봇 감독·각 PC 담당이 회차 중에 하는 런북 7단계
# 조작을 런북 명령 그대로 실행한다. 판단을 더하지 않는다. 한 일은 모두 operator.log 에 「OPERATOR 런북 <절>」로 남긴다.
#   - 7.1 「실패·취소·정지 뒤에는 다시 task_arm」: 조정기 RESULT 가 FAILED·EXPIRED·CANCELED 이면 그 작업의 로봇을
#     refresh_state 뒤 다시 arm 한다.
#   - 7.1 TB-5 arm: TB-5 출발 정렬 START_ALIGN_ALIGNED 뒤 request_nomotion_update → refresh_state → task_arm.
#   - 7.5 2차 고정 뒤: MAP_REVISION_ADOPTED robot_a … -> R2 를 보면 tb6_slam 을 끄고(/robot6/map 발행자 0 확인)
#     frozen_localization robot_a 를 띄운다.
# run_full.sh 가 현장 도메인 셸에서 띄운다(RUN_FULL_TOKEN 이 있어 stop_full.sh 가 함께 끈다).
# 사용: sim_operator.sh <회차 폴더> <현장 파일 사본> <프로파일>
set -u
RUN="${1:?회차 폴더}"; F="${2:?현장 파일}"; P="${3:?프로파일}"
LOG="$RUN/logs"
say() { echo "OPERATOR $(date +%H:%M:%S) $*"; }
srv() { timeout 40 ros2 service call "$1" "$2" "${3:-{\}}" 2>&1 | grep -oE "success=(True|False)|message='[^']{0,200}" | tr '\n' ' '; }
arm() {   # 7.1: 한 로봇 다시 arm (TB-5 는 AMCL 갱신 먼저)
    local ns="$1" i out
    [ "$ns" = robot5 ] && say "런북 7.1 /robot5/request_nomotion_update: $(srv /robot5/request_nomotion_update std_srvs/srv/Empty)"
    for i in 1 2 3; do
        say "런북 7.1 refresh_state: $(srv /mission_coordinator/refresh_state std_srvs/srv/Trigger)"
        out="$(srv /$ns/rescue_command_adapter/task_arm std_srvs/srv/Trigger)"
        say "런북 7.1 $ns task_arm: $out"
        case "$out" in *ARMED*) return 0 ;; esac
        sleep 10
    done
}
diag() {   # 원인 확인용 자료(작전·통합 요청 2026-10-15): 그 순간의 /robot6/map 발행자·노드·프로세스
    local tag="$1" out="$LOG/diag_$1.txt"
    { echo "== $(date +%H:%M:%S.%N) $tag"
      echo "-- ros2 topic info -v /robot6/map --no-daemon --spin-time 10"; ros2 topic info -v /robot6/map --no-daemon --spin-time 10 2>&1
      echo "-- ros2 node list (map|slam)"; ros2 node list --no-daemon --spin-time 5 2>&1 | grep -i -E "map|slam"
      echo "-- ps (slam_toolbox|map_server)"; ps -eo pid,etime,args | grep -E "slam_toolbox|map_server" | grep -v grep | cut -c1-200
    } > "$out" 2>&1
    say "자료 남김 $out"
}
lifecycle_diag() {   # C8: TB-5 Nav2 서버 상태
    local out="$LOG/diag_c8_tb5_nav2.txt" n
    { echo "== $(date +%H:%M:%S.%N) $1"
      for n in controller_server planner_server bt_navigator behavior_server lifecycle_manager_navigation; do
          echo "-- /robot5/$n: $(timeout 15 ros2 lifecycle get /robot5/$n 2>&1 | head -1)"
      done; } >> "$out" 2>&1
    say "자료 남김 $out"
}
slam_to_frozen() {   # 7.5: tb6_slam 끄기 -> /robot6/map 발행자 0 -> frozen_localization robot_a
    local pid i count
    pid="$(cat "$RUN/pids/tb6_slam" 2>/dev/null)"
    diag c7_before_slam_stop
    # 런북은 터미널 Ctrl+C(SIGINT)다. 여기서는 SIGTERM: 백그라운드로 띄운 launch 는 SIGINT 를 무시로 물려받아 INT 로는
    # SLAM 이 꺼지지 않았다(2026-10-15 회차 232037, slam_toolbox 가 회차 끝까지 돌아 relay 가 MAP_RELAY_BLOCKED).
    # 터미널 Ctrl+C 는 전경 프로세스 그룹 전체에 간다. launch 는 `exec setsid` 로 떠서 그 PID 가 곧 프로세스 그룹이라,
    # 그룹 전체에 보낸다: launch PID 에만 보내면 launch 만 끝나고 sync_slam_toolbox_node 가 살아남아 /robot6/map 을 계속
    # 냈다(2026-10-16 11회차 05:07, 발행자 1). slam_toolbox 프로세스가 실제로 끝났는지 PID 로 확인한다.
    local slam_pid
    slam_pid="$(sed 's/\x1b\[[0-9;]*m//g' "$LOG/tb6_slam.log" | sed -n 's/.*sync_slam_toolbox_node-1\]: process started with pid \[\([0-9]*\)\].*/\1/p' | head -1)"
    say "런북 7.5 2차 고정 뒤 tb6_slam 끄기(launch 프로세스 그룹 ${pid:-없음}에 TERM, slam_toolbox PID ${slam_pid:-모름})"
    [ -n "$pid" ] && kill -TERM -- "-$pid" 2>/dev/null
    for i in $(seq 1 30); do
        [ -n "$slam_pid" ] && kill -0 "$slam_pid" 2>/dev/null && { sleep 1; continue; }
        count="$(ros2 topic info /robot6/map --no-daemon --spin-time 10 2>/dev/null | sed -n 's/^Publisher count: //p')"
        [ "${count:-1}" = 0 ] && break
        sleep 2
    done
    [ -n "$slam_pid" ] && kill -0 "$slam_pid" 2>/dev/null && say "런북 7.5 slam_toolbox(PID $slam_pid)가 아직 살아 있음"
    say "런북 7.5 /robot6/map 발행자 수 ${count:-모름}(--no-daemon)"
    diag c7_before_frozen_loc
    [ "${count:-1}" = 0 ] || { say "런북 7.5 SLAM 이 아직 지도를 냄 — frozen_localization 을 띄우지 않음"; return 1; }
    setsid ros2 launch rescue_bringup frozen_localization.launch.py "profile_path:=$P" "field_path:=$F" \
        robot_id:=robot_a frozen_pose_wait_s:=10.0 > "$LOG/frozen_loc_a.log" 2>&1 < /dev/null &
    echo $! > "$RUN/pids/frozen_loc_a"
    say "런북 7.5 frozen_localization robot_a 띄움(로그 $LOG/frozen_loc_a.log)"
}

say "시작: 런북 7.1(실패 뒤 다시 arm, TB-5 arm)·7.5(2차 고정 뒤 TB-6 위치 추정)를 대신한다"
declare -A robot_of
done75=false
tail -n 0 -F "$LOG/coordinator.log" "$LOG/robot_robot_a.log" "$LOG/frozen_loc_b.log" "$LOG/frozen_loc_a.log" 2>/dev/null \
    | sed -u 's/\x1b\[[0-9;]*m//g' | while read -r line; do
    case "$line" in
        *"SEND "*" task="*" robot="*)
            task="${line##*task=}"; task="${task%% *}"; robot="${line##*robot=}"; robot="${robot%% *}"
            robot_of[$task]="$robot" ;;
        *"RESULT task="*" outcome="*)
            task="${line##*task=}"; task="${task%% *}"; outcome="${line##*outcome=}"; outcome="${outcome%% *}"
            case "$outcome" in
                FAILED|EXPIRED|CANCELED|CANCELLED)
                    case "${robot_of[$task]:-}" in
                        robot_a) say "결과 $outcome task=$task robot_a"; arm robot6 ;;
                        robot_b) say "결과 $outcome task=$task robot_b"; arm robot5 ;;
                        *) say "결과 $outcome task=$task 로봇 모름 — arm 하지 않음" ;;
                    esac ;;
            esac ;;
        *"START_ALIGN_ALIGNED"*)
            say "TB-5 출발 정렬 끝"; arm robot5 ;;
        *"MAP_RELAY_BLOCKED"*"/robot6/map"*)
            if [ ! -f "$LOG/diag_c7_after_blocked.txt" ]; then say "TB-6 relay 막힘"; diag c7_after_blocked; fi ;;
        *"MAP_FROZEN 1 "*)
            lifecycle_diag "1차 고정 뒤" ;;
        *"EXPLORATION_COMPLETE"*)   # C9: 탐색 끝 순간의 TB-6 SLAM 지도
            mkdir -p "$RUN/diag_c9" && cp -p "$RUN/maps/live_map.yaml" "$RUN/diag_c9/" 2>/dev/null \
                && cp -p "$RUN/maps/$(sed -n 's/^image: //p' "$RUN/maps/live_map.yaml")" "$RUN/diag_c9/" 2>/dev/null
            say "탐색 끝: SLAM 지도 저장 $RUN/diag_c9" ;;
        *"MAP_REVISION_ADOPTED robot_a"*"-> R"*)
            rev="${line##*-> R}"; rev="${rev%% *}"
            if [ "${rev:-0}" -ge 2 ] && ! $done75; then done75=true; slam_to_frozen; fi ;;
    esac
done

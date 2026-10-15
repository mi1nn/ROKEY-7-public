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
slam_to_frozen() {   # 7.5: tb6_slam 끄기 -> /robot6/map 발행자 0 -> frozen_localization robot_a
    local pid i count
    pid="$(cat "$RUN/pids/tb6_slam" 2>/dev/null)"
    say "런북 7.5 2차 고정 뒤 tb6_slam 끄기(PID ${pid:-없음})"
    [ -n "$pid" ] && kill -INT "$pid" 2>/dev/null
    for i in $(seq 1 30); do
        count="$(ros2 topic info /robot6/map --no-daemon --spin-time 3 2>/dev/null | sed -n 's/^Publisher count: //p')"
        [ "${count:-1}" = 0 ] && break
        sleep 2
    done
    say "런북 7.5 /robot6/map 발행자 수 ${count:-모름}"
    [ "${count:-1}" = 0 ] || { say "런북 7.5 SLAM 이 아직 지도를 냄 — frozen_localization 을 띄우지 않음"; return 1; }
    setsid ros2 launch rescue_bringup frozen_localization.launch.py "profile_path:=$P" "field_path:=$F" \
        robot_id:=robot_a frozen_pose_wait_s:=10.0 > "$LOG/frozen_loc_a.log" 2>&1 < /dev/null &
    echo $! > "$RUN/pids/frozen_loc_a"
    say "런북 7.5 frozen_localization robot_a 띄움(로그 $LOG/frozen_loc_a.log)"
}

say "시작: 런북 7.1(실패 뒤 다시 arm, TB-5 arm)·7.5(2차 고정 뒤 TB-6 위치 추정)를 대신한다"
declare -A robot_of
done75=false
tail -n 0 -F "$LOG/coordinator.log" "$LOG/robot_robot_a.log" "$LOG/frozen_loc_b.log" 2>/dev/null \
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
        *"MAP_REVISION_ADOPTED robot_a"*"-> R"*)
            rev="${line##*-> R}"; rev="${rev%% *}"
            if [ "${rev:-0}" -ge 2 ] && ! $done75; then done75=true; slam_to_frozen; fi ;;
    esac
done

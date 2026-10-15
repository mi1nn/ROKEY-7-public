#!/bin/bash
# run_full.sh 회차 하나를 끈다: 그 회차 표시(RUN_FULL_TOKEN)가 있는 프로세스만 PID 로(INT -> TERM -> KILL). pkill -f 안 씀.
# 사용: stop_full.sh <회차 폴더>
set -u
RUN="${1:?회차 폴더}"
TOKEN="$(cat "$RUN/token" 2>/dev/null)" || { echo "stop_full: $RUN/token 이 없습니다" >&2; exit 2; }
tagged() { grep -lsz -x -F "RUN_FULL_TOKEN=$TOKEN" /proc/[0-9]*/environ 2>/dev/null | cut -d/ -f3 | grep -vx "$$"; }
pids="$(tagged | tr '\n' ' ')"
[ -n "${pids// /}" ] || { echo "stop_full: 떠 있는 것이 없습니다"; exit 0; }
kill -INT $pids 2>/dev/null
for i in $(seq 1 30); do [ -z "$(tagged)" ] && break; sleep 0.5; done
left="$(tagged | tr '\n' ' ')"; [ -n "${left// /}" ] && kill -TERM $left 2>/dev/null && sleep 3
left="$(tagged | tr '\n' ' ')"; [ -n "${left// /}" ] && kill -KILL $left 2>/dev/null
echo "stop_full: 껐습니다 ($RUN)"

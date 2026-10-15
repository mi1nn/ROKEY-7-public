#!/usr/bin/env bash
# 한 PC 전체 시나리오 (방식 B, 실시간 지도 공유) — 현장에서 실행하는 명령은 이것 하나다 (2026-10-15).
#
#   bash scripts/field/run_full_scenario.sh --profile <profile.json> --perception-args <amr_perception.json> \
#        [--tb5-start X Y YAW] [--no-rviz] [--no-alert] [--no-overhead] [--web-accounts <db>] [--run-dir <새 폴더>]
#
# "웹캠이 지진을 감지했다"는 가정 이후 전부를 자동으로 한다: 단계별 기동 + 실제 준비 확인, 두 로봇 arm,
# 합성 DISASTER_SIGN 1회, 진행 감시, 끝(TB-5 출구 대기 + TB-6 시작점 도킹) 또는 실패/Ctrl+C 때 두 로봇 안전 정지와
# 프로세스 정리. 절차서: docs/process/full-scenario-single-pc.md. 실제 로봇은 사람이 현장을 지켜볼 때만 움직인다.
#
# 환경: ROS 2 Jazzy + 이 저장소의 colcon 설치본(ROKEY7_INSTALL, 기본 <repo>/install). ROS_DOMAIN_ID 는 미리
# export 한다(프로파일 hosts 의 domain_id 와 다르면 시작하지 않음). Discovery Server 를 쓰면 ROS_DISCOVERY_SERVER 도
# 미리 export 한다(0단계, field README). 실행되는 노드는 ROS_SUPER_CLIENT=False, 실행기 자신만 그래프 확인용 super client.
set -eo pipefail
REPO=$(cd "$(dirname "$0")/../.." && pwd)
INSTALL=${ROKEY7_INSTALL:-$REPO/install}
if [ ! -f "$INSTALL/setup.bash" ]; then
  echo "[preflight] FAILED 작업공간 설치본이 없다: $INSTALL"
  echo "  cd $REPO && source /opt/ros/jazzy/setup.bash && colcon build --packages-up-to rescue_bringup"
  echo "  (다른 위치면 ROKEY7_INSTALL=<install 폴더>)"
  exit 2
fi
source /opt/ros/jazzy/setup.bash
source "$INSTALL/setup.bash"
export ROS_SUPER_CLIENT=False FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}
echo "[preflight] ROS_DISTRO=${ROS_DISTRO:-?} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-(unset)}" \
     "ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER:-(unset)} RMW=${RMW_IMPLEMENTATION:-(default)} install=$INSTALL"
cd "$REPO"
exec /usr/bin/python3 -m rescue_bringup.full_scenario "$@"

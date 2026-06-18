#!/usr/bin/env bash
# Shared setup for the twin-only proof scripts (scripts/proof/*.sh).
# Source it first in each script:  source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
#
# It (idempotently) sources ROS 2 + the workspace when `ros2` isn't already on
# PATH, defaults ROS_DOMAIN_ID / RMW to the lab values, and provides
# say/ok/warn/fail helpers. Run these INSIDE the running stack's environment:
#   * emulation:  docker exec algae-twin-emu bash /opt/proof/00_preflight.sh
#                 (the runner mounts emulation/ ; copy proof/ in or exec by path)
#   * lab PC:     source /opt/ros/jazzy/setup.bash
#                 source ~/turtlebot3_ws/install/setup.bash
#                 ROS_DOMAIN_ID=36 bash 00_preflight.sh
PROOF_TIMEOUT="${PROOF_TIMEOUT:-6}"

if [ -t 1 ]; then
  C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_ERR=$'\033[31m'; C_H=$'\033[1;36m'; C_0=$'\033[0m'
else
  C_OK=; C_WARN=; C_ERR=; C_H=; C_0=
fi
say()  { printf '\n%s== %s ==%s\n' "$C_H"   "$*" "$C_0"; }
ok()   { printf '   %sOK%s   %s\n'  "$C_OK"  "$C_0" "$*"; }
warn() { printf '   %sWARN%s %s\n'  "$C_WARN" "$C_0" "$*"; }
fail() { printf '   %sFAIL%s %s\n'  "$C_ERR" "$C_0" "$*"; }

# source ROS + a workspace only if ros2 isn't already available (keeps an already
# sourced lab/container shell untouched; never toggles set -u around sourcing).
if ! command -v ros2 >/dev/null 2>&1; then
  [ -f /opt/ros/jazzy/setup.bash ] && . /opt/ros/jazzy/setup.bash
  for _ws in /opt/twin_ws/install/setup.bash "$HOME/turtlebot3_ws/install/setup.bash"; do
    [ -f "$_ws" ] && { . "$_ws"; break; }
  done
fi
if ! command -v ros2 >/dev/null 2>&1; then
  fail "ros2 not found — source /opt/ros/jazzy/setup.bash + your workspace first"
  exit 2
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-36}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

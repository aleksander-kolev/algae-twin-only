#!/usr/bin/env bash
# PROOF 10 — navigation goal end-to-end.  *** MOVES THE ROBOT ***
# Sends a Nav2 goal and reports SUCCEEDED/ABORTED; meanwhile /plan holds a path.
#   usage: bash 10_nav_goal.sh [X] [Y]      (map frame, default 0.6 0.0)
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; source "$HERE/_common.sh"
X="${1:-0.6}"; Y="${2:-0.0}"
say "PROOF 10 — Nav2 goal to ($X, $Y)   *** MOVES THE ROBOT ***"
warn "the real robot WILL drive to ($X,$Y). Ctrl-C within 3 s to abort."; sleep 3

( sleep 3; pc="$(timeout 4 ros2 topic echo /plan --once 2>/dev/null | grep -c 'position:')"
  echo "   /plan poses computed: ${pc:-0}" ) &

ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {w: 1.0}}}}"
rc=$?
wait 2>/dev/null
[ $rc -eq 0 ] && ok "goal returned SUCCEEDED — controller reached it" \
             || fail "goal did not succeed (status above) — check loops (proof 07) + localization (proof 03)"
exit $rc

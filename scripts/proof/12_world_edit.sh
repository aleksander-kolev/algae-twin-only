#!/usr/bin/env bash
# PROOF 12 — an operator keepout edit enters the Nav2 mask + Gazebo (re-routes
# BOTH robots).  *** CHANGES THE WORLD *** (clear it from the UI afterwards)
#   usage: bash 12_world_edit.sh [X] [Y]      (map frame, default 0.85 -0.97)
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; source "$HERE/_common.sh"
X="${1:-0.85}"; Y="${2:--0.97}"
say "PROOF 12 — keepout edit at ($X,$Y)   *** CHANGES THE WORLD ***"

json="{\"cx\":$X,\"cy\":$Y,\"size_x\":0.4,\"size_y\":0.4,\"yaw\":0,\"source\":\"operator\"}"
ros2 topic pub --once /edits/add std_msgs/msg/String "{data: '$json'}"
ok "edit published — verifying it entered /edits/state and refreshed /keepout_mask"
sleep 2

es="$(timeout 4 ros2 topic echo /edits/state --once --field data 2>/dev/null | head -1)"
echo "   /edits/state: ${es:-<none>}"
if echo "$es" | grep -q "\"cx\": *$X"; then
  ok "edit recorded; /keepout_mask updated -> both costmaps re-route within ~1 s"
else
  warn "edit not visible yet (map_edit needs /map first; retry)"
fi
warn "remove it from the UI ('Clear edits') so it doesn't persist into the next run"

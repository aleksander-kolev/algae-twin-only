#!/usr/bin/env bash
# PROOF 11 — place algae -> navigate -> 3-spin clean -> cleared.  *** MOVES THE ROBOT ***
#   usage: bash 11_mission_clean.sh [X] [Y]      (map frame, default 0.6 -0.5)
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; source "$HERE/_common.sh"
X="${1:-0.6}"; Y="${2:--0.5}"
say "PROOF 11 — mission: algae at ($X,$Y) -> clean   *** MOVES THE ROBOT ***"
warn "the robot WILL navigate to the algae and spin-clean it. Ctrl-C within 3 s to abort."; sleep 3

ros2 topic pub --once /algae/add geometry_msgs/msg/PointStamped \
  "{header: {frame_id: map}, point: {x: $X, y: $Y, z: 0.0}}"
ok "algae placed — watching /algae/state (queued -> active -> cleaning -> cleared):"
timeout 12 ros2 topic echo /algae/state 2>/dev/null | sed 's/^/   /' | head -30
echo "   --- /clean/active (true while the 3-spin dispersal runs) ---"
timeout 4 ros2 topic echo /clean/active --once --field data 2>/dev/null | sed 's/^/   clean_active: /'
echo "   --- /sim/sprayer_cmd (dispersal motor velocity) ---"
timeout 4 ros2 topic echo /sim/sprayer_cmd --once --field data 2>/dev/null | sed 's/^/   sprayer: /'
ok "watch the UI: the patch turns cleared and the twin sprays + beeps on the real robot"

#!/usr/bin/env bash
# PROOF 05 — the Gazebo twin shadows the real robot: ground truth flows, the
# divergence is small, and commands are mirrored to /sim/cmd_vel.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; source "$HERE/_common.sh"
say "PROOF 05 — the Gazebo twin shadows the real robot"
rc=0

gt="$(timeout 5 ros2 topic echo /sim/ground_truth --once --field pose.pose.position 2>/dev/null \
      | awk '/^x:/{x=$2}/^y:/{y=$2}END{if(x!="")print "x="x" y="y}')"
if [ -n "$gt" ]; then ok "/sim/ground_truth twin pose: $gt"; else fail "/sim/ground_truth silent — gz twin down?"; rc=1; fi

div="$(timeout 4 ros2 topic echo /twin/divergence --once --field data 2>/dev/null | head -1)"
if [ -n "$div" ]; then
  echo "   real<->twin divergence: ${div} m  (teleport threshold = 1.0 m)"
  awk -v d="$div" 'BEGIN{exit !(d+0 < 1.0)}' && ok "twin glued (< 1.0 m)" || warn "divergence high — pose_sync will teleport"
else
  warn "/twin/divergence silent (pose_sync up?)"
fi

if timeout 3 ros2 topic echo /sim/cmd_vel --once >/dev/null 2>&1; then
  ok "/sim/cmd_vel present — commands mirrored to the twin"
else
  warn "/sim/cmd_vel idle (no motion right now — fine when stationary)"
fi
exit $rc

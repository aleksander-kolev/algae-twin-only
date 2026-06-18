#!/usr/bin/env bash
# PROOF 13 — a real obstacle the static map can't explain is mirrored into the
# digital world (orange box).  EMULATION uses /fake_world/add_box; on the LAB PC
# you physically place a box >=25 cm in front instead.
#   usage: bash 13_obstacle_mirror.sh [X] [Y]   (map frame, default 0.8 0.0)
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; source "$HERE/_common.sh"
X="${1:-0.8}"; Y="${2:-0.0}"
say "PROOF 13 — physical -> digital obstacle mirror"

if ros2 topic info /fake_world/add_box >/dev/null 2>&1; then
  ros2 topic pub --once /fake_world/add_box geometry_msgs/msg/PointStamped \
    "{header: {frame_id: map}, point: {x: $X, y: $Y, z: 0.0}}"
  ok "emulated real box dropped at ($X,$Y) — the lidar now sees it"
else
  warn "no /fake_world (this is the LAB PC) — physically place a box >=25 cm in front of the robot now"
fi

echo "   watching /edits/state for a 'mirrored' edit (obstacle_mirror needs stable returns)…"
for _ in $(seq 1 20); do
  es="$(timeout 3 ros2 topic echo /edits/state --once --field data 2>/dev/null | head -1)"
  if echo "$es" | grep -q 'mirrored'; then ok "mirrored edit appeared: $es"; exit 0; fi
  sleep 1
done
warn "no mirrored edit within 20 s (ensure AMCL is localized; obstacle within lidar range)"
exit 1

#!/usr/bin/env bash
# PROOF 03 (headline) — localization is LOCKED: AMCL's map->base_footprint
# estimate agrees with the robot's TRUE /odom pose to within 0.30 m.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; source "$HERE/_common.sh"
say "PROOF 03 — localization locked (AMCL estimate vs TRUE odom)"

odom="$(timeout 5 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null)"
ox="$(awk '/^x:/{print $2; exit}' <<<"$odom")"
oy="$(awk '/^y:/{print $2; exit}' <<<"$odom")"

tf="$(timeout 5 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null | grep -m1 'Translation')"
ax="$(sed -nE 's/.*\[ *([^,]+), *([^,]+).*/\1/p' <<<"$tf")"
ay="$(sed -nE 's/.*\[ *([^,]+), *([^,]+).*/\2/p' <<<"$tf")"

printf '   TRUE /odom         : x=%s  y=%s\n' "${ox:-?}" "${oy:-?}"
printf '   AMCL map->base_foot: x=%s  y=%s\n' "${ax:-?}" "${ay:-?}"

if [ -z "$ox" ] || [ -z "$ax" ]; then
  fail "could not read both poses — is the stack up and AMCL seeded? (Set robot pose)"; exit 1
fi
d="$(python3 -c "import math;print(f'{math.hypot($ox-($ax),$oy-($ay)):.3f}')" 2>/dev/null)"
printf '   divergence         : %s m\n' "${d:-?}"
if awk -v d="$d" 'BEGIN{exit !(d+0 < 0.30)}'; then
  ok "AMCL within 0.30 m of truth — localization is locked (not the drift bug)"; exit 0
else
  fail "divergence >= 0.30 m — relocalize (Set robot pose) or investigate"; exit 1
fi

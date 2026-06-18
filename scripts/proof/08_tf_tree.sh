#!/usr/bin/env bash
# PROOF 08 — the TF chain map->odom->base_footprint->base_scan is intact and the
# lidar offset matches the burger URDF (base_footprint->base_scan x ~= -0.032 m).
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; source "$HERE/_common.sh"
say "PROOF 08 — TF chain (map -> odom -> base_footprint -> base_scan)"
rc=0

if timeout 5 ros2 run tf2_ros tf2_echo map base_footprint >/dev/null 2>&1; then
  ok "map->base_footprint resolves (AMCL localized)"
else
  fail "map->base_footprint missing — AMCL not seeded (Set robot pose)"; rc=1
fi

tx="$(timeout 5 ros2 run tf2_ros tf2_echo base_footprint base_scan 2>/dev/null \
      | sed -nE 's/.*Translation: \[ *([^,]+),.*/\1/p' | head -1)"
echo "   base_footprint->base_scan  x = ${tx:-?} m  (expect ~ -0.032)"
if [ -n "$tx" ] && awk -v x="$tx" 'BEGIN{exit !(x < -0.02 && x > -0.05)}'; then
  ok "lidar offset matches the burger URDF"
else
  warn "lidar offset unexpected (check the URDF / robot_state_publisher)"
fi
echo "   tip: 'ros2 run tf2_tools view_frames' writes a full frames.pdf (sim/* are separate)"
exit $rc

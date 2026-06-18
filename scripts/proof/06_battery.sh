#!/usr/bin/env bash
# PROOF 06 — battery monitoring + the mission voltage gate.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; source "$HERE/_common.sh"
say "PROOF 06 — battery monitoring + gate"

rv="$(timeout 4 ros2 topic echo /battery_state --once --field voltage 2>/dev/null | head -1)"
sv="$(timeout 4 ros2 topic echo /sim/battery_state --once --field voltage 2>/dev/null | head -1)"
echo "   real pack : ${rv:-?} V"
echo "   twin pack : ${sv:-?} V  (mirrors the real pack in twin mode)"
echo "   mission gate: battery_min_voltage = 11.3 V (above the OpenCR ~11.0 V motor cutoff)"

if [ -n "$rv" ] && awk -v v="$rv" 'BEGIN{exit !(v+0 >= 11.3)}'; then
  ok "real pack above the mission gate — missions allowed"
else
  warn "real pack at/below the gate (or silent) — missions may be refused"
fi

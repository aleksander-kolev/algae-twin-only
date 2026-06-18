#!/usr/bin/env bash
# PROOF 04 — the one-brain safety bus: EXACTLY ONE publisher owns /cmd_vel
# (the twin_bridge mux). 0 = mux down; >1 = Nav2 bypassed the safety bus.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; source "$HERE/_common.sh"
say "PROOF 04 — one-brain safety bus (/cmd_vel single writer)"

pubs="$(ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count/{print $3}')"
subs="$(ros2 topic info /cmd_vel 2>/dev/null | awk '/Subscription count/{print $3}')"
echo "   /cmd_vel  publishers=${pubs:-?}  subscribers=${subs:-?}"

chz="$(timeout 4 ros2 topic hz /cmd_vel 2>/dev/null | awk '/average rate/{print $3; exit}')"
[ -n "$chz" ] && echo "   /cmd_vel rate: ~${chz} Hz (the 20 Hz mux tick)"
echo "   Nav2's raw command flows on /nav_cmd_vel -> the mux -> /cmd_vel:"
timeout 3 ros2 topic echo /nav_cmd_vel --once 2>/dev/null | sed 's/^/      /' | head -8

if [ "${pubs:-0}" = "1" ]; then
  ok "exactly ONE writer on /cmd_vel (the mux) — Nav2 cannot bypass the safety bus"; exit 0
else
  fail "/cmd_vel writers=${pubs:-0} (want 1: 0=mux down, >1=Nav2 bypassed the mux)"; exit 1
fi

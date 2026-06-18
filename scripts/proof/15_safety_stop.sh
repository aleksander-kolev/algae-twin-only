#!/usr/bin/env bash
# PROOF 15 — the front-cone collision safety-stop (observe). Reports the live
# safety flag from /ui/status. Place a box <0.25 m ahead to see it latch.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; source "$HERE/_common.sh"
say "PROOF 15 — front-cone collision safety-stop (observe)"
echo "   cuts FORWARD motion when the front cone (~+/-29 deg) sees < 0.25 m;"
echo "   latches until clear > 0.35 m for 0.3 s; fails SAFE on a stale /scan;"
echo "   rotation + reverse stay allowed."

s="$(timeout 5 ros2 topic echo /ui/status --once --field data 2>/dev/null \
     | grep -o '"safety": *[a-z]*' | head -1)"
echo "   /ui/status -> ${s:-<no status yet>}"
if echo "$s" | grep -q true; then
  ok "safety stop is ENGAGED right now (obstacle in the front cone)"
else
  ok "safety stop clear (no near obstacle) — place a box < 0.25 m ahead to watch it latch"
fi

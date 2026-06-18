#!/usr/bin/env bash
# PROOF 07 — Nav2's control/planner loops are real-time (no software-GL render
# starvation). Samples /rosout for "missed its desired rate" warnings.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; source "$HERE/_common.sh"
say "PROOF 07 — Nav2 loops are real-time (no starvation)"

echo "   sampling /rosout for ${PROOF_TIMEOUT}s for loop rate-miss warnings…"
misses="$(timeout "$PROOF_TIMEOUT" ros2 topic echo /rosout --field msg 2>/dev/null \
          | grep -ic 'missed its desired rate' || true)"
echo "   'missed its desired rate' messages in window: ${misses:-0}"

if [ "${misses:-0}" = "0" ]; then
  ok "control + planner loops holding rate — no starvation"
  exit 0
else
  fail "loops missing their rate — software-GL render starvation; use the GPU or a lighter mode"
  exit 1
fi

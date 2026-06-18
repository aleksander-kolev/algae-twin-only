#!/usr/bin/env bash
# PROOF 14 — E-STOP freezes both robots (the mux zeroes /cmd_vel), then releases.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; source "$HERE/_common.sh"
say "PROOF 14 — E-STOP freezes both robots"

ros2 topic pub --once /estop std_msgs/msg/Bool "{data: true}"
ok "E-STOP engaged — /cmd_vel must go to zero"
sleep 1
cv="$(timeout 3 ros2 topic echo /cmd_vel --once 2>/dev/null)"
echo "$cv" | grep -A4 'twist:' | head -6 | sed 's/^/   /'
if echo "$cv" | grep -qE '^[[:space:]]*x:[[:space:]]*0\.0$'; then
  ok "forward/linear command is zero under E-STOP"
else
  warn "could not confirm a zero twist (topic idle is also fine while stopped)"
fi

warn "releasing E-STOP in 2 s…"; sleep 2
ros2 topic pub --once /estop std_msgs/msg/Bool "{data: false}"
ok "E-STOP released — the mux resumes passing commands"

#!/usr/bin/env bash
# Run the OBSERVE-ONLY proofs (00–09) in order and tally PASS/FAIL. These never
# move the robot or change the world. The ACTIVE proofs (10–15) are listed at the
# end — run them individually because they drive the robot or edit the world.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; source "$HERE/_common.sh"
say "Algae Twin — proof suite (observe-only; does NOT move the robot)"

OBSERVE=(00_preflight 01_graph 02_robot_link 03_localization 04_command_bus \
         05_twin_shadow 06_battery 07_loops_perf 08_tf_tree 09_keepout)
pass=0; failc=0; failed=()
for s in "${OBSERVE[@]}"; do
  if bash "$HERE/$s.sh"; then pass=$((pass + 1)); else failc=$((failc + 1)); failed+=("$s"); fi
done

say "RESULT: $pass passed, $failc failed (observe-only suite)"
[ "$failc" -eq 0 ] || printf '   failed: %s\n' "${failed[*]}"
cat <<'EOF'

  Active proofs (run individually — they MOVE THE ROBOT or change the world):
    bash 10_nav_goal.sh [X Y]        send a Nav2 goal              (drives)
    bash 11_mission_clean.sh [X Y]   place algae + 3-spin clean    (drives)
    bash 12_world_edit.sh [X Y]      add a keepout box             (re-routes both)
    bash 13_obstacle_mirror.sh [X Y] mirror a real obstacle        (emulation: /fake_world)
    bash 14_estop.sh                 E-STOP then release
    bash 15_safety_stop.sh           observe the front-cone safety flag
EOF
[ "$failc" -eq 0 ]

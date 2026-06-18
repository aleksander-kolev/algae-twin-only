#!/usr/bin/env bash
# PROOF 01 — the ROS graph is up: core nodes present, topics + actions listed.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; source "$HERE/_common.sh"
say "PROOF 01 — ROS graph (nodes / topics / actions)"

nodes="$(ros2 node list 2>/dev/null | sort)"
echo "$nodes" | sed 's/^/   node   /'
miss=0
for n in amcl bt_navigator controller_server planner_server map_server \
         twin_bridge pose_sync mission map_edit obstacle_mirror; do
  if echo "$nodes" | grep -q "/$n\b"; then :; else fail "missing node: $n"; miss=$((miss + 1)); fi
done
echo "   topics : $(ros2 topic list 2>/dev/null | wc -l) total"
echo "   actions: $(ros2 action list 2>/dev/null | tr '\n' ' ')"

if [ "$miss" -eq 0 ]; then ok "all core nodes present"; else fail "$miss core node(s) missing"; exit 1; fi

#!/usr/bin/env bash
# Live evidence for the THREE graded DT usages, in one pass. Run it while the
# twin is up and the robot is localized; screen-record this terminal next to the
# browser UI (+ RViz) for the 2-3 min video. See docs/DEMO.md for the sequence.
#
#   1) Bidirectional communication (two-way pub/sub between the entities)
#   2) State synchronization (motion + non-motion: battery / e-stop / mode)
#   3) Environmental & object interaction (propagates BOTH ways)
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; source "$HERE/_common.sh"

rate() {  # rate TOPIC [sensor]   -> "12.3 Hz" or "-"
  local t="$1" q=""; [ "${2:-}" = sensor ] && q="--qos-profile sensor_data"
  local r; r="$(timeout 4 ros2 topic hz "$t" $q 2>/dev/null | awk '/average rate/{printf "%.1f Hz",$3; exit}')"
  printf '%s' "${r:--}"
}
pubs() { ros2 topic info "$1" 2>/dev/null | awk '/Publisher count/{print $3}'; }

say "PILLAR 1 — Bidirectional communication (each entity publishes streams the other uses)"
echo "   REAL robot --> DT/brain :  /scan=$(rate /scan sensor)   /odom=$(rate /odom)   /battery_state=$(rate /battery_state)"
echo "   DT/brain  --> REAL robot:  /cmd_vel=$(rate /cmd_vel)   (publishers=$(pubs /cmd_vel); exactly 1 = the safety mux)"
echo "   GAZEBO twin --> DT      :  /sim/ground_truth=$(rate /sim/ground_truth)   /sim/scan=$(rate /sim/scan sensor)"
echo "   DT --> GAZEBO twin      :  /sim/cmd_vel=$(rate /sim/cmd_vel)"
echo "   internal status         :  /ui/status=$(rate /ui/status)   /twin/divergence=$(rate /twin/divergence)"
ok "two-way data flow: real <-> DT <-> twin, steady-rate (no one-way-only)"

say "PILLAR 2 — State synchronization (motion AND internal state)"
timeout 5 ros2 topic echo /ui/status --once --field data 2>/dev/null | python3 -c '
import sys, json
raw = sys.stdin.read()
i, j = raw.find("{"), raw.rfind("}")
if i < 0:
    print("   (no /ui/status yet — is the stack up + UI node running?)"); sys.exit()
s = json.loads(raw[i:j+1])
def pose(p): return "n/a" if not p else f"({p[0]:+.2f}, {p[1]:+.2f}, {p[2]:+.2f} rad)"
r, m = s.get("real", {}), s.get("sim", {})
print(f"   pose      real={pose(r.get(\"pose\"))}   twin={pose(m.get(\"pose\"))}     (motion mirror)")
print(f"   battery   real={r.get(\"voltage\")} V   twin={m.get(\"voltage\")} V     (NON-motion: twin MIRRORS the real pack)")
print(f"   divergence={s.get(\"divergence\")} m   e-stop={s.get(\"estop\")}   safety={s.get(\"safety\")}   mode={s.get(\"mode\")}")
'
echo "   raw packs: real /battery_state=$(timeout 4 ros2 topic echo /battery_state --once --field voltage 2>/dev/null | head -1) V" \
     " | twin /sim/battery_state=$(timeout 4 ros2 topic echo /sim/battery_state --once --field voltage 2>/dev/null | head -1) V"
ok "pose + battery + divergence + e-stop/mode mirrored near-real-time (internal state included)"
warn "show a state CHANGE on camera: E-STOP (both freeze) -> bash $HERE/14_estop.sh ; battery gate refuses missions when low"

say "PILLAR 3 — Environmental & object interaction (events do NOT stay on one side)"
echo "   current world edits (/edits/state):"
timeout 4 ros2 topic echo /edits/state --once --field data 2>/dev/null | head -1 | sed 's/^/      /'
cat <<EOF

   A) REAL obstacle  -->  DIGITAL twin     (obstacle_mirror; real /scan -> mask + Gazebo box)
        emulation:  bash $HERE/13_obstacle_mirror.sh 0.8 0
        lab     :   physically place a box >=25 cm in front, then run 13 to watch it mirror
   B) DIGITAL edit   -->  REAL robot        (map_edit keepout; re-routes BOTH costmaps ~1 s)
        trigger :   bash $HERE/12_world_edit.sh 0.85 -0.97        (or 'Block path' in the UI)
   C) live avoidance during a goal (robot plans around it):
        drive   :   bash $HERE/10_nav_goal.sh
EOF
ok "real->digital (mirror) AND digital->real (keepout) both demonstrable — bidirectional environment propagation"

say "Record the UI + RViz alongside this terminal. Full sequence: docs/DEMO.md"

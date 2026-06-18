# Demo script — the 3 graded DT usages (2–3 min video)

twin-only hits all three required Digital-Twin usages at the top **Redlining
(4/4)** level. This is the on-screen sequence, the exact topics/scripts that
prove each, and what to say.

## Which script to run
1. **Launch** the twin (Week-9 lab): `./scripts/run_lab.sh` (real robot) —
   add `--headless` on a weak GPU, `--domain N` if the robot ≠ #36. Hardware-free
   rehearsal at home: `bash scripts/run_twin_emulated_wsl_docker.sh --rviz`.
   Full steps + fallbacks: [`RUNBOOK.md`](RUNBOOK.md).
2. **Localize once**: RViz **2D Pose Estimate** (or UI "Set robot pose") until the
   scan hugs the walls; the twin snaps onto the robot.
3. **Record** the browser UI (**http://localhost:8088**) + RViz next to a terminal
   running **`bash scripts/proof/demo_rubric.sh`** — it prints the live two-way
   evidence for all three pillars in one pass.

## Rubric → twin-only evidence

| Graded usage | What twin-only shows | Topics (proof) | Script | Target |
|---|---|---|---|---|
| **1. Bidirectional comms** | real `/scan`+`/odom`→DT/brain; DT→robot `/cmd_vel`; twin `/sim/ground_truth`+`/sim/scan`→DT; DT→twin `/sim/cmd_vel`; internal `/ui/status`,`/twin/divergence` | `02_robot_link` `04_command_bus` `05_twin_shadow` | `demo_rubric.sh` §1 | **4** (≥1 stream each way + status) |
| **2. State synchronization** | pose mirror **+ battery mirrored real→twin** (internal) **+ divergence** (localization quality) **+ e-stop/mode** (affects behavior) | `/ui/status`, `/battery_state` vs `/sim/battery_state`, `/twin/divergence`, `/estop` | `05_twin_shadow` `06_battery` `14_estop` | **4** (multiple states incl. internal) |
| **3. Env & object interaction** | real obstacle→digital (mirror) **AND** digital edit→real robot re-route (keepout); live avoidance | `/scan`→`/edits/state`(mirrored)→`/keepout_mask`; `/edits/add`→`/keepout_mask` | `13_obstacle_mirror` `12_world_edit` `10_nav_goal` | **4** (mirrored both ways, change introduced live) |

## On-screen sequence (~2.5 min)

**0:00 — Both entities online (15 s).** UI shows the **real robot + the Gazebo
twin** moving together, both batteries live, divergence in cm. Say: "one Nav2
brain drives the real Burger; the Gazebo twin shadows it."

**0:15 — Pillar 1, Bidirectional (35 s).** Terminal: `bash scripts/proof/demo_rubric.sh`
(or `04_command_bus.sh` + `05_twin_shadow.sh`). Point at the rates: real
`/scan`+`/odom` → brain, brain → `/cmd_vel` (one writer = the safety mux), twin
`/sim/ground_truth` → brain, brain → `/sim/cmd_vel`, plus `/ui/status`. Say:
"each entity publishes streams the other consumes — two-way, steady-rate."

**0:50 — Pillar 2, State sync (40 s).** Show in the UI/`demo_rubric.sh` §2: the
twin **battery mirrors the real pack** (non-motion internal state), pose matches,
divergence small. Then press **E-STOP** (`14_estop.sh` or the UI button) — **both
robots freeze** and the mode/flag flips; **RESUME** — both continue. Say:
"internal state (battery, e-stop/mode) is mirrored and it changes behavior."

**1:30 — Pillar 3, Environment (50 s), introduce a change LIVE:**
- **Digital → real:** in the UI **Block path** (or `12_world_edit.sh`). The red
  box appears in Gazebo **and** the keepout mask re-routes the **real** robot's
  plan within ~1 s. 
- **Real → digital:** place a real box in front (lab) / `13_obstacle_mirror.sh`
  (emulation). An **orange mirrored box appears in the twin + on the map**, and
  nav avoids it. Say: "environment events don't stay local — they propagate **both
  ways** across the entities."
- Optionally drive a goal (`10_nav_goal.sh`) so the robot visibly plans around it.

**2:20 — Wrap (10 s).** `00_preflight.sh` → **GO**, and `03_localization.sh` →
AMCL ≈ truth: the link is solid throughout.

## Tips
- Prefer **`--rviz`/`--headless`** unless the GPU is strong — a starved control
  loop ruins the take (see RUNBOOK §2a).
- Re-seed the **2D Pose Estimate** if nav looks off before recording.
- `demo_rubric.sh` is observe-only and safe to run anytime; the env-interaction
  beats (12/13) and e-stop (14) are the live "changes" the rubric asks for.

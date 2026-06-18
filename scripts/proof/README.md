# Proof scripts — verify every link with `ros2`

One script per claim. Each prints what it proves, runs the `ros2`
commands, shows the evidence, and (where it can) exits **0 = PASS / non-zero =
FAIL**. These are the runnable form of [`docs/REFERENCE.md`](../../docs/REFERENCE.md)
§9 *Proof cookbook*.

## Run them

Run **inside the running stack's ROS environment** (`_common.sh` auto-sources ROS
+ the workspace and defaults `ROS_DOMAIN_ID=36` if you haven't):

**Emulation (Docker)** — the proof folder ships in the package; exec it:
```bash
# whole observe-only suite:
docker exec algae-twin-emu bash /opt/twin_ws/src/algae_twin/../../../proof/prove_all.sh 2>/dev/null \
  || docker cp scripts/proof algae-twin-emu:/tmp/proof && docker exec algae-twin-emu bash /tmp/proof/prove_all.sh
# or open a shell and run them from a bind-mounted/copied path:
docker exec -it algae-twin-emu bash         # then: bash /tmp/proof/00_preflight.sh
```
Simplest: `docker cp scripts/proof algae-twin-emu:/tmp/proof` once, then
`docker exec algae-twin-emu bash /tmp/proof/prove_all.sh`.

**Lab PC (native)** — from the repo, in a sourced shell:
```bash
source /opt/ros/jazzy/setup.bash && source ~/turtlebot3_ws/install/setup.bash
ROS_DOMAIN_ID=36 bash scripts/proof/prove_all.sh
```

## What's here

| # | script | proves | safe? |
|---|---|---|---|
| 00 | `00_preflight.sh` | every link GO/NO-GO (exit code = verdict) | observe |
| 01 | `01_graph.sh` | core nodes up; topics/actions listed | observe |
| 02 | `02_robot_link.sh` | `/odom` `/scan` `/battery_state` live | observe |
| 03 | `03_localization.sh` | **AMCL ≈ true odom (< 0.30 m)** — the headline | observe |
| 04 | `04_command_bus.sh` | exactly ONE writer on `/cmd_vel` (the mux) | observe |
| 05 | `05_twin_shadow.sh` | twin shadows real (ground truth, divergence, mirror) | observe |
| 06 | `06_battery.sh` | battery + the 11.3 V mission gate | observe |
| 07 | `07_loops_perf.sh` | Nav2 loops real-time (no starvation) | observe |
| 08 | `08_tf_tree.sh` | TF chain intact; lidar offset −0.032 m | observe |
| 09 | `09_keepout.sh` | keepout/world-edit pipeline wired | observe |
| 10 | `10_nav_goal.sh [X Y]` | a Nav2 goal reaches the goal | **drives** |
| 11 | `11_mission_clean.sh [X Y]` | place → navigate → 3-spin clean → cleared | **drives** |
| 12 | `12_world_edit.sh [X Y]` | a keepout edit re-routes both robots | **edits world** |
| 13 | `13_obstacle_mirror.sh [X Y]` | a real obstacle is mirrored into the twin | **edits world** |
| 14 | `14_estop.sh` | E-STOP zeroes `/cmd_vel`, then releases | changes state |
| 15 | `15_safety_stop.sh` | front-cone safety-stop flag | observe |
| — | `prove_all.sh` | runs 00–09 and tallies PASS/FAIL | observe |

`prove_all.sh` runs only the **observe-only** proofs. The **drives / edits world**
ones move the real robot or change the map — run them one at a time, on purpose.

Override per run: `ROS_DOMAIN_ID=36 PROOF_TIMEOUT=8 bash 03_localization.sh`.

# Algae Twin — TurtleBot3 Burger digital twin (real robot + Gazebo twin)

A digital twin for a **physical TurtleBot3 Burger** on **ROS 2 Jazzy** + **Gazebo
Harmonic (gz-sim)**. **One Nav2 brain** localises and drives the real robot; the
**Gazebo twin** shadows it in lockstep; world changes flow **both ways**; and the
operator works from a **browser dashboard** (served from the Python stdlib — no
GUI library to install).

Place "algae" on the map → the pair navigates there and runs a 3‑spin chemical
dispersal clean → it clears. Block a path on the map → both robots re‑route and a
box appears in Gazebo. Drop a real obstacle in front of the robot → it is mirrored
into the digital world and the robot avoids it.

This build does **one thing**: run the twin against the real robot. There is no
standalone `sim` mode, no `mode:=` argument, no `--demo` preview and no
re‑mapping/cartographer workflow — everything not needed for that was removed.

| Requirement | Where it lives |
|---|---|
| Bidirectional communication | one ROS 2 graph; `twin_bridge` mux duplicates every command to both robots, state flows back from both |
| Same path / same moves | a single Nav2 stack → one plan; `pose_sync` shadow‑corrects the twin and teleports it on large divergence |
| Digital map edit re‑routes the real robot | `map_edit` → Nav2 **keepout filter mask** + a spawned Gazebo box → ~1 Hz BT replanning |
| Physical change updates the digital world | `obstacle_mirror` turns unexplained lidar returns into mirrored edits (Gazebo box + keepout + map) |
| Battery monitoring | real `/battery_state` + `sim_battery` twin model (mirrors the real pack); UI gauges + mission gate |
| Algae: place → navigate → 3 spins → disperse | `mission` node: Nav2 `NavigateToPose`, then `Spin(3×2π)`; twin's dispersal motor = sprayer joint + chemical particle emitter; `/clean/active` is the real‑hardware hook and the robot beeps via `/sound` |
| Operator UI (simple Python) | `operator_ui` — a browser dashboard (stdlib HTTP, no GUI lib) |

## Architecture at a glance

```
                       ┌─────────────────────────── ONE NAV2 BRAIN ───────────────────────────┐
                       │  map_server → amcl → planner → controller → bt_navigator → behaviors  │
                       │  localises against map.pgm · publishes TF map→odom · /plan            │
                       └───────▲───────────────────────────────────────────────┬──────────────┘
            /scan,/odom,TF     │ (REAL robot, wall clock)                        │ /nav_cmd_vel
   ┌───────────────────────────┴─────────┐                           ┌──────────▼────────────┐
   │  REAL TURTLEBOT3 BURGER             │   /cmd_vel (TwistStamped)  │  twin_bridge  (MUX)   │
   │  turtlebot3_bringup on the robot Pi │◄──────────────────────────┤  nav│clean│estop pick │
   │  → /odom /scan /battery /joints TF  │                           │  + front-cone safety  │
   └──────────────────────────────────────┘                          │  → /cmd_vel     (real)│
   ┌──────────────────────────────────────┐  /sim/cmd_vel            │  → /sim/cmd_vel (twin)│
   │  GAZEBO TWIN  (burger_twin)          │◄─────────────────────────┤  → /ui/status  (JSON) │
   │  DiffDrive · gpu_lidar · sprayer     │       + /twin/correction  └──────────┬────────────┘
   │  → /sim/ground_truth /sim/scan …     ├──► pose_sync · /twin/divergence · teleport
   │    frames prefixed  sim/*            │
   └──────────────────────────────────────┘
   operator_ui (browser :8088) ◄──► mission · map_edit (/keepout_mask) · obstacle_mirror · sim_battery
                                     preflight  (one-shot GO/NO-GO of every link)
```

* **One brain, two bodies.** Nav2 localises and drives the **real** robot; every
  command is muxed by `twin_bridge` to **both** `/cmd_vel` (real) and
  `/sim/cmd_vel` (twin). `pose_sync` keeps the twin glued (gentle correction;
  teleport on >1 m sustained divergence).
* **Wall clock** (`use_sim_time:=false`) — the real robot is the time authority.
  The twin's `sim/*` frames never enter the real localization TF chain.
* **Bidirectional world.** Operator keepout edits and mirrored real obstacles go
  into the Nav2 keepout mask **and** the Gazebo world at once (`map_edit`).

## Prerequisites

ROS 2 **Jazzy** + Gazebo Harmonic + the **turtlebot3 stack** + Nav2 + `ros_gz`.

### Lab PC — install nothing (no sudo)
The lab laptop is **fully native (no Docker, no sudo)** and already has ROS 2
Jazzy at `/opt/ros/jazzy` with the **turtlebot3 stack built from source in
`~/turtlebot3_ws/src`**. You build only `algae_twin` on top of that workspace —
which `scripts/run_lab.sh` does for you (copies the package into
`~/turtlebot3_ws/src`, `colcon build --packages-select algae_twin`, sources,
launches). If a system package is genuinely missing there is **no sudo on the lab
laptop — flag it to a TA**; `run_lab.sh` prints exactly which one.

### Home / a fresh machine (you have sudo)
Easiest is the **Docker** image below — it ships the whole stack. To build
natively from scratch instead:

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server \
                 ros-jazzy-xacro python3-colcon-common-extensions
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cp -r /path/to/this/repo/algae_twin .             # this package
cd ~/turtlebot3_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select turtlebot3_msgs turtlebot3_description \
    turtlebot3_gazebo turtlebot3_teleop algae_twin && source install/setup.bash
```

> This repo carries a top‑level `COLCON_IGNORE` so that dropping it **inside** an
> existing colcon workspace won't double‑build the package (two packages named
> `algae_twin` would clash). The package is always built by copying the inner
> `algae_twin/` folder into a workspace, which `run_lab.sh` and the steps above do.

## Run

The robot's standard bringup must be running first, on the **same**
`ROS_DOMAIN_ID` (the lab uses **36**, the robot's sticker number) and the same RMW:

```bash
ssh turtlebot@192.168.8.36            # robot IP is on the sticker
export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 ROS_DOMAIN_ID=36 \
       RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch turtlebot3_bringup robot.launch.py     # leave this terminal running
```

### A. Lab PC — one command (recommended)

```bash
./scripts/run_lab.sh                 # deps → build → verify /scan → launch + preflight
./scripts/run_lab.sh --headless      # no 3D Gazebo window (weak/no GPU — safest)
./scripts/run_lab.sh --no-rviz       # skip RViz (RViz opens by default for 2D Pose Estimate)
./scripts/run_lab.sh --check-only    # just verify the machine + robot link, don't launch
```

Defaults: `ROS_DOMAIN_ID=36` (`--domain N`), workspace `~/turtlebot3_ws`
(`--ws PATH`). It starts with an **empty world** (you place algae/obstacles during
the demo; `--keep-edits` restores the previous session's boxes). The console is
logged to `~/algae_twin_logs/<stamp>/console.log` — copy it off before leaving
(lab laptops get wiped).

### B. Docker — any PC on the robot's network

```bash
docker build -t algae-twin-only .
docker run -it --rm --network host -e ROS_DOMAIN_ID=36 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp algae-twin-only \
  bash -lc 'ros2 launch algae_twin twin.launch.py'
```

Host networking lets DDS discovery reach the robot. The image bundles ROS Jazzy,
the turtlebot3 stack and `algae_twin`; `xvfb` gives the twin's `gpu_lidar` a
software‑GL context so `/sim/scan` is produced with no display.

### C. Manual

```bash
export ROS_DOMAIN_ID=36                       # must match the robot
ros2 launch algae_twin twin.launch.py         # args: headless, ui, rviz, spawn_x/y/yaw, map, world
```

Then open the operator UI at **http://localhost:8088**.

### Run modes & the GPU (read before using the 3D window)

Gazebo renders the twin's `gpu_lidar` every cycle. On **software GL** (`llvmpipe`,
i.e. a machine with no GPU driver) that rendering runs in CPU bursts that **starve
Nav2's control loop** — it drops below 10 Hz and goals abort with *"Failed to make
progress"*. **This — not AMCL — is the usual cause of "the 2nd/3rd goal fails /
the robot stalls".** Localization itself stays locked to a few cm throughout.

| Mode | gz 3D window | Use when |
|---|---|---|
| default | headless server | lightest, most reliable nav; drive from the browser UI |
| `--rviz` | headless server + RViz | full view, still reliable |
| (3D window) | yes | prettiest; **needs a real GPU** |

Confirm GL on the lab PC: `glxinfo -B \| grep -i renderer` — if it says `llvmpipe`,
use `--headless`. Because the lab PC drives a **real** robot, prefer `--headless`
unless you've confirmed the GPU holds the loop.

## Operate

* **Set robot pose** (once per run): AMCL starts at the map origin. In **RViz**
  click *2D Pose Estimate* on the robot's real spot and drag toward its heading
  (the red lidar points should snap onto the walls), or use **Set robot pose** in
  the browser UI. The twin then snaps onto the real robot a moment later.
* **Place algae** — click the map: a green patch appears (also in Gazebo), the
  robot navigates to it, spins 3× while the dispersal motor sprays (green mist +
  spinning nozzle in Gazebo, a beep on the real robot), then the patch clears.
* **Block path** — drag a box: it becomes a keepout zone for the shared planner
  **and** a physical box in Gazebo. An active mission replans within ~1–2 s.
* **Nav goal / Set robot pose** — drag for position + heading.
* Right panel: real/twin **online dots + battery gauges**, twin **divergence**,
  mission queue, world edits, **E‑STOP / RESUME** (freezes BOTH robots),
  return‑home, recharge‑twin, **Sync twin → real**, clear‑mirrored.

What twin mode does:

* Nav2 localises and drives the **real** robot (`/scan`, `base_footprint`); every
  velocity command is duplicated to the twin (`/sim/cmd_vel`).
* `pose_sync` measures real↔twin divergence (shown in the UI), nudges the twin to
  stay glued, and **teleports** it if divergence exceeds 1 m for 2 s (e.g. after
  you relocalise AMCL).
* `sim_battery` mirrors the real pack — the twin reports the truth.
* `obstacle_mirror` adds anything the real lidar sees that the map can't explain
  into the digital world (orange boxes; "Clear mirrored" removes them).
* The real robot beeps (OpenCR `/sound` service) when dispersal starts/ends —
  attach a real dispersal motor to the `/clean/active` Bool topic.
* A front‑cone **collision safety‑stop** cuts forward motion when the real lidar
  sees an obstacle within ~25 cm (latched until it clears past ~35 cm; rotation
  and reverse stay allowed; **fails safe** — stops — on a stale `/scan`). Tune via
  `safety_*` in `config/twin.yaml`; disable with `safety_enable:=false`.

## Verify the link (go / no‑go)

```bash
ros2 run algae_twin preflight        # ~6 s; "GO — all links up" or a per-link hint
```

It checks: the status bridge, Gazebo ground truth, the twin lidar, the keepout
pipeline, the Nav2 action server, localization TF, robot odom/lidar/battery, and
that **exactly one** publisher owns `/cmd_vel` (the safety mux). `run_lab.sh`
runs it automatically ~40 s after launch.

The two facts that matter most, by hand:

```bash
# 1) localization locked: AMCL estimate ≈ the robot's TRUE odom (to a few cm)
ros2 topic echo /odom --once --field pose.pose.position    # true pose
ros2 run tf2_ros tf2_echo map base_footprint               # AMCL estimate

# 2) the one-brain safety bus: exactly ONE publisher on /cmd_vel (the mux)
ros2 topic info /cmd_vel | grep 'Publisher count'          # → Publisher count: 1
ros2 topic hz /cmd_vel                                      # → ~20 Hz
```

If the launch console logs `Control loop missed its desired rate`, the render is
starving Nav2 — use `--headless` or a real GPU (see *Run modes & the GPU*).

## Five‑minute demo

1. Launch → `ros2 run algae_twin preflight` → **GO**. The UI shows both robots
   online, batteries live, divergence in cm.
2. If the robot didn't start at the map origin: **Set robot pose** once — watch
   the twin teleport onto it.
3. **Place algae** across the arena: both robots drive the same path (trails
   overlap), then the 3‑spin clean runs — green mist + spinning dispersal motor in
   Gazebo, a beep on the real robot, the battery sagging in the UI.
4. Mid‑mission, **Block path** across the route: the plan bends within ~2 s for
   BOTH robots, and the red box stands in Gazebo.
5. Drop a **real** box (≥ 25 cm tall — the burger's lidar scans at ~17 cm) in
   front of the robot: the costmap avoids it and an orange mirrored box appears in
   Gazebo + on the map.
6. **E‑STOP** during a clean: both robots freeze, dispersal stops, the mission
   re‑queues; **RESUME** finishes it.

## Interface contract (all standard messages)

| Topic | Type | Purpose |
|---|---|---|
| `/algae/add`, `/algae/remove` | PointStamped, String | drop / delete algae |
| `/algae/state` | String (JSON, latched) | mission queue + progress |
| `/edits/add` `/edits/remove` `/edits/clear` | String (JSON) | world edits |
| `/edits/state`, `/keepout_mask` | String / OccupancyGrid (latched) | edits + Nav2 filter mask |
| `/estop` | Bool (latched) | freeze / resume both robots |
| `/mission/goto`, `/mission/home` | PoseStamped, Empty | manual goals |
| `/twin/resync` | Empty | snap the twin onto the real robot now |
| `/clean/active` | Bool (latched) | dispersal motor state (real‑HW hook) |
| `/sim/sprayer_cmd` | Float64 | dispersal motor speed (bridged to gz) |
| `/ui/status` | String (JSON, 5 Hz) | poses, batteries, divergence, e‑stop, safety |
| `/twin/divergence`, `/twin/correction` | Float32, Twist | shadow controller |
| `/cmd_vel`, `/sim/cmd_vel` | TwistStamped / Twist | mux → real / twin |
| `/battery_state`, `/sim/battery_state` | BatteryState | monitoring |

## Tests

ROS‑free where possible; run standalone or under `colcon test`:

```bash
# from algae_twin/ :
python test/test_operator_ui_smoke.py     # browser UI: endpoints, SSE, commands, validation (no ROS)
python test/test_util.py                  # angle/quaternion/battery/map-transform math (ROS sourced)
python test/test_imports.py               # every node imports; the sim strip is gone (ROS sourced)
```

## Troubleshooting

| Symptom | Cause → fix |
|---|---|
| Goals abort *"Failed to make progress"*; console shows `Control loop missed its desired rate` | software‑GL render starving the control loop (NOT AMCL) → `--headless` or a real GPU |
| Robot ignores `/cmd_vel` | TB3 jazzy bringup expects `TwistStamped` — the mux **auto‑detects** + switches; or set `real_cmd_stamped` on `twin_bridge` (`real_cmd_autodetect:=false` to disable detection) |
| Robot & PC don't see each other | same `ROS_DOMAIN_ID` (lab = **36**) and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` on both; clear a leftover `ROS_LOCALHOST_ONLY=1` |
| `preflight` NO‑GO `localization` | AMCL has no pose → **Set robot pose** in the UI (or RViz 2D Pose Estimate) |
| `preflight` NO‑GO `robot odom`/`lidar`/`battery` | robot bringup down → start `turtlebot3_bringup`; check Wi‑Fi / domain / RMW / `LDS_MODEL` |
| `cmd writer` ≠ 1 | only `twin_bridge` may publish `/cmd_vel` — >1 means Nav2 bypassed the mux, 0 means the mux is down |
| No `/sim/scan` | the `gpu_lidar` needs a render backend → provide a GPU, or `LIBGL_ALWAYS_SOFTWARE=1` (slow) |
| Keepout edits don't re‑route | the mask only affects *planning*; the robot finishes the current ~1 s BT cycle first. Check `/keepout_mask` and that both costmaps log "Received filter mask" |
| Missions refused, "battery low" | voltage gate (`battery_min_voltage` 11.3 V, above the OpenCR ~11.0 V motor cutoff) → recharge; or `/battery_state` is stale |
| Robot stops at an obstacle, "won't go" | the 25 cm front‑cone safety **latch** — clear past 0.35 m (rotation / reverse still work) |
| Sim time jumps back after Ctrl+C | a leftover Gazebo server → `pkill -f 'gz sim'` |

## Package layout

```
algae_twin/                ROS 2 package (ament_python)
  algae_twin/
    twin_bridge.py         command mux (nav/clean/estop) + front-cone safety + /ui/status
    pose_sync.py           twin shadow controller: correction + divergence + teleport
    mission.py             algae lifecycle: place → navigate → 3-spin clean → clear; battery gate
    map_edit.py            world edits → /keepout_mask (Nav2) + Gazebo boxes; persisted
    obstacle_mirror.py     real lidar → unmapped obstacle → mirrored edit (physical→digital)
    sim_battery.py         twin battery (mirrors the real pack, or simulates discharge)
    operator_ui.py         browser dashboard (stdlib http.server, no GUI lib) :8088
    preflight.py           ~6 s GO/NO-GO check of every link
    grid.py                occupancy-grid math (StaticMap / EditGrid / GridGeometry)
    gz_io.py               spawn/remove/teleport/emitter via ros_gz services (gz CLI fallback)
    util.py                angles, quaternions, QoS profiles, JSON topics, battery
    ui_web.py              the dashboard HTML/JS (served by operator_ui)
  launch/    twin.launch.py (entry) · sim_gz.launch.py · nav2.launch.py
  config/    twin.yaml · nav2_params.yaml · bridge.yaml
  maps/      map.pgm + map.yaml            (86×110 @ 0.05 m, origin −2.051, −4.194)
  models/burger_twin/model.sdf            (DiffDrive · gpu_lidar · sprayer + particles)
  worlds/algae_world.sdf · rviz/twin.rviz · test/ · setup.py · package.xml
scripts/run_lab.sh         one-command lab-PC runner (native, no sudo)
Dockerfile                 reproducible ROS 2 Jazzy image (real robot via host networking)
```

License: Apache‑2.0.

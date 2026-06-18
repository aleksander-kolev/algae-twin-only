# Algae Twin — TWIN MODE ONLY (real robot + Gazebo twin)

A self-contained, stripped-down build of the Algae Twin that does **one thing**:
run the digital twin against the **physical TurtleBot3 Burger**. **One Nav2 brain**
localises and drives the real robot; the **Gazebo twin** shadows it in lockstep;
world changes flow both ways; the operator works from a browser dashboard — place
"algae", the pair navigates there, runs a 3‑spin chemical‑dispersal clean and
clears it; block a path and both re‑route; drop a real obstacle and it is mirrored
into the digital world.

Everything **not** needed for twin mode is removed — no standalone `sim` mode, no
`mode:=` argument, no UI `--demo` preview, no cartographer workflow. The same
`twin.launch.py` runs in two places, byte‑for‑byte identical:

| | Robot source | Started by |
|---|---|---|
| **Lab PC** (real robot) | physical TB3 + `turtlebot3_bringup` | `scripts/run_lab.sh` |
| **Any laptop** (no hardware) | `emulation/fake_robot.py` (wall‑clock software robot) | `scripts/run_twin_emulated_wsl_docker.sh` |

So a **GO** in emulation means a **GO** in the lab once the robot's bringup is up
on the same `ROS_DOMAIN_ID`.

> **The complete node/topic/service/frame catalog, every config parameter, and a
> step‑by‑step "prove every link with `ros2`" cookbook are in
> [`docs/REFERENCE.md`](docs/REFERENCE.md).** This README is the tour; that file
> is the manual.
>
> **Going to the lab?** → [`docs/RUNBOOK.md`](docs/RUNBOOK.md) — run steps,
> fallbacks, and a fix‑it table.

---

## Architecture at a glance

```
                       ┌─────────────────────────── ONE NAV2 BRAIN ───────────────────────────┐
                       │  map_server → amcl → planner → controller → bt_navigator → behaviors  │
                       │  localises against map.pgm · publishes TF map→odom · /plan            │
                       └───────▲───────────────────────────────────────────────┬──────────────┘
            /scan,/odom,TF     │ (REAL robot, wall clock)                        │ /nav_cmd_vel
   ┌───────────────────────────┴────────┐                            ┌──────────▼────────────┐
   │  REAL ROBOT                         │                            │  twin_bridge  (MUX)   │
   │  lab : turtlebot3_bringup           │   /cmd_vel (TwistStamped)  │  nav│clean│estop pick │
   │  test: emulation/fake_robot.py      │◄───────────────────────────┤  + front‑cone safety  │
   │  → /odom /scan /battery /joints TF  │                            │  → /cmd_vel     (real)│
   └─────────────────────────────────────┘                           │  → /sim/cmd_vel (twin)│
   ┌─────────────────────────────────────┐   /sim/cmd_vel            │  → /ui/status  (JSON) │
   │  GAZEBO TWIN  (burger_twin)         │◄──────────────────────────┴──────────┬────────────┘
   │  DiffDrive · gpu_lidar · sprayer    │        + /twin/correction              │
   │  → /sim/ground_truth /sim/scan …    ├──► pose_sync ─ correction · /twin/divergence · teleport
   │    frames prefixed  sim/*           │
   └─────────────────────────────────────┘
   operator_ui (browser :8088) ◄──► mission · map_edit (/keepout_mask) · obstacle_mirror · sim_battery
                                     preflight  (GO/NO‑GO of every link)
```

* **One brain, two bodies.** Nav2 localises/drives the **real** robot; every
  command is muxed by `twin_bridge` to **both** `/cmd_vel` (real) and
  `/sim/cmd_vel` (twin). `pose_sync` keeps the twin glued (gentle correction;
  teleport on >1 m divergence).
* **Wall clock** (`use_sim_time:=false`) — the real robot is the time authority.
  The twin's `sim/*` frames never enter the real localization TF chain.
* **Bidirectional world.** Operator/keepout edits and mirrored real obstacles go
  into the Nav2 keepout mask **and** the Gazebo world at once (`map_edit`).

---

## Repository layout (code map)

```
twin-only/
├─ algae_twin/                      the ROS 2 package (ament_python)
│  ├─ algae_twin/
│  │  ├─ twin_bridge.py     command mux (nav/clean/estop) + front‑cone safety + /ui/status
│  │  ├─ pose_sync.py       twin shadow controller: correction + divergence + teleport
│  │  ├─ mission.py         algae lifecycle: place → navigate → 3‑spin clean → clear; battery gate
│  │  ├─ map_edit.py        world edits → /keepout_mask (Nav2) + Gazebo boxes; persisted
│  │  ├─ obstacle_mirror.py real lidar → unmapped obstacle → mirrored edit (physical→digital)
│  │  ├─ sim_battery.py     twin battery (mirrors the real pack, or simulates discharge)
│  │  ├─ operator_ui.py     browser dashboard (stdlib http.server, no GUI lib) :8088
│  │  ├─ preflight.py       6‑second GO/NO‑GO check of every link
│  │  ├─ grid.py            StaticMap / EditGrid / GridGeometry (occupancy math)
│  │  ├─ gz_io.py           spawn/remove/teleport/emitter via ros_gz services (gz CLI fallback)
│  │  ├─ util.py            angles, quaternions, QoS profiles, JSON topics, battery
│  │  └─ ui_web.py          the dashboard HTML/JS (served by operator_ui)
│  ├─ launch/  twin.launch.py · nav2.launch.py · sim_gz.launch.py
│  ├─ config/  nav2_params.yaml · twin.yaml · bridge.yaml
│  ├─ maps/    map.pgm + map.yaml          (86×110 @ 0.05 m, origin −2.051,−4.194)
│  ├─ models/burger_twin/model.sdf         (DiffDrive · gpu_lidar · sprayer + particles)
│  ├─ worlds/algae_world.sdf · rviz/twin.rviz · test/ · setup.py · package.xml
├─ emulation/                       hardware‑free test harness (NOT shipped in the package)
│  ├─ fake_robot.py · twin_emulated.launch.py · add_fake_obstacle.sh
├─ scripts/  run_lab.sh (native lab PC) · run_twin_emulated_wsl_docker.sh (WSL2+Docker)
├─ Dockerfile · README.md · docs/REFERENCE.md
```

---

## Prerequisites

ROS 2 **Jazzy** + Gazebo Harmonic + the **turtlebot3 stack** + Nav2 + `ros_gz`.
How you get them depends on the machine:

### Lab PC — install nothing (no sudo)
The lab laptop is **fully native (no Docker, no sudo)** and already has ROS 2
Jazzy at `/opt/ros/jazzy` with the **turtlebot3 stack built from source in
`~/turtlebot3_ws/src`**. You install **nothing** — you only build `algae_twin` on
top of that workspace, which `scripts/run_lab.sh` does for you (copies the package
into `~/turtlebot3_ws/src`, `colcon build --packages-select algae_twin`, sources,
launches). If a system package is genuinely missing there is **no sudo on the lab
laptop — flag it to a TA**; `run_lab.sh` prints exactly which one. The workspace
is already provisioned on the lab machine; this build assumes it is present.

### Home / a fresh machine (you have sudo)
Easiest is the **Docker emulation** below — it ships the whole stack. To build
natively from scratch instead:
```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server \
                 ros-jazzy-xacro python3-colcon-common-extensions
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cp -r /path/to/twin-only/algae_twin .             # this package
cd ~/turtlebot3_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select turtlebot3_msgs turtlebot3_description \
    turtlebot3_gazebo turtlebot3_teleop algae_twin && source install/setup.bash
```

> `twin-only/` re‑uses the package name `algae_twin` and carries a `COLCON_IGNORE`
> so a `colcon build` from the **parent** repo root skips it (two packages named
> `algae_twin` would clash). On the lab PC the package is built **in
> `~/turtlebot3_ws`** by `run_lab.sh`, not from this folder in place.

---

## Run

### A. Lab PC — one command (real robot)

```bash
ros2 launch turtlebot3_bringup robot.launch.py     # on the robot's Pi first

./scripts/run_lab.sh                 # deps → build → verify /scan → launch + preflight
./scripts/run_lab.sh --headless      # no 3D Gazebo window (weak/no GPU — safest for the real robot)
./scripts/run_lab.sh --rviz          # + RViz
./scripts/run_lab.sh --check-only    # just verify the machine + robot link
```

Defaults: `ROS_DOMAIN_ID=36` (`--domain N`), workspace `~/turtlebot3_ws`
(`--ws PATH`); console logged to `~/algae_twin_logs/<stamp>/`. On the robot
(standard, unchanged TB3 jazzy bringup):

```bash
ssh turtlebot@192.168.8.36
export TURTLEBOT3_MODEL=burger ROS_DOMAIN_ID=36 LDS_MODEL=LDS-02 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch turtlebot3_bringup robot.launch.py
```

Then **Set robot pose** once in the UI (or RViz *2D Pose Estimate*) so AMCL
converges. Drive from <http://localhost:8088>.

### B. Emulation — laptop, no hardware (WSL2 + Docker)

```bash
bash scripts/run_twin_emulated_wsl_docker.sh         # headless gz + browser UI (lightest, reliable)
bash scripts/run_twin_emulated_wsl_docker.sh --rviz  # + RViz (no 3D gz window)
bash scripts/run_twin_emulated_wsl_docker.sh --gui   # + 3D Gazebo window (GPU auto‑detected)
```

Image `algae-twin-only`, container `algae-twin-emu`, `ROS_DOMAIN_ID=36`, UI at
<http://localhost:8088>. In‑container ROS:

```bash
docker exec algae-twin-emu bash -c 'source /opt/ros/jazzy/setup.bash && \
  source /opt/twin_ws/install/setup.bash && export ROS_DOMAIN_ID=36 && <cmd>'
```

Rebuild after package edits (Git Bash): `MSYS_NO_PATHCONV=1 docker build -t algae-twin-only twin-only/`
(the runner live‑mounts `emulation/`, so emulator‑only edits need no rebuild).

### C. Manual / Docker (any PC on the robot's network + matching domain)

```bash
export ROS_DOMAIN_ID=36                      # must match the robot
ros2 launch algae_twin twin.launch.py        # args: headless, ui, rviz, spawn_x/y/yaw, map, world

docker build -t algae-twin-only .
docker run -it --rm --network host -e ROS_DOMAIN_ID=36 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp algae-twin-only \
  bash -lc 'ros2 launch algae_twin twin.launch.py'
```

---

## Run modes & the GPU (read this before `--gui`)

Gazebo renders the twin's `gpu_lidar` every cycle. On **software GL** (`llvmpipe`
— forced inside the Docker runner, or any machine with no GPU driver) that
rendering runs in **CPU bursts that starve Nav2's control loop** (drops below
10 Hz → robot stops making progress → goals abort with *"Failed to make
progress"*). **This — not AMCL — is the cause of the classic "2nd/3rd goal fails
/ robot stalls".** (Localization itself stays locked to a few cm throughout.)

| Mode | gz 3D window | RViz | Use when |
|---|---|---|---|
| default | headless | no | lightest, most reliable nav; drive from the browser UI |
| `--rviz` | headless | yes | full view, still reliable |
| `--gui` | **yes** | yes | prettiest; **needs a real GPU** |

* **Emulation runner auto‑detects an NVIDIA GPU** and renders on it (WSL2: Mesa
  `d3d12` over `/dev/dxg`), removing the starvation; falls back to software GL.
  Force with `--gpu` / `--no-gpu`.
* **Lab PC (native)** — none of the WSL/Docker GL plumbing applies. Just verify
  GL is hardware‑accelerated:

  ```bash
  sudo apt install -y mesa-utils && glxinfo -B | grep -i renderer   # must NOT say "llvmpipe"
  ```

  GPU shown → any mode is fine. `llvmpipe` → run `run_lab.sh --headless`
  (and/or `sudo ubuntu-drivers autoinstall`). Because the lab PC drives a **real**
  robot, prefer `--headless` unless you have confirmed the GPU holds the loop.
  Full diagnosis + the d3d12 recipe: [`docs/REFERENCE.md`](docs/REFERENCE.md) →
  *Performance & GPU*.

---

## What twin mode does

* Nav2 localises and drives the **real** robot (`/scan`, `base_footprint`); every
  velocity command is duplicated to the twin (`/sim/cmd_vel`).
* `pose_sync` measures real↔twin divergence (shown in the UI), nudges the twin to
  stay glued, and **teleports** it if divergence exceeds 1 m for 2 s (e.g. after
  you relocalise AMCL).
* `sim_battery` **mirrors the real pack** — the twin reports the truth.
* `obstacle_mirror` adds anything the real lidar sees that the map can't explain
  into the digital world (orange boxes; "Clear mirrored" removes them).
* `map_edit` turns operator boxes into a Nav2 keepout mask **and** a Gazebo box,
  so a digital edit re‑routes the real robot.
* `mission` runs algae cleaning: navigate → 3 spins while the twin's dispersal
  motor sprays → patch cleared; the real robot beeps via `/sound`; `/clean/active`
  is the hook for real dispersal hardware.
* A front‑cone **collision safety‑stop** cuts forward motion when the real lidar
  sees an obstacle within ~25 cm (fails safe on a stale `/scan`; rotation/reverse
  stay allowed). Tune via `safety_*` in `config/twin.yaml`.

---

## Five‑minute demo

1. Launch → `ros2 run algae_twin preflight` → **GO**. UI shows both robots
   online, batteries live, divergence in cm.
2. If the robot didn't start at the map origin: **Set robot pose** once — the twin
   teleports onto it.
3. **Place algae**: both robots drive the same path; the 3‑spin clean runs (green
   mist + spinning dispersal motor in Gazebo, beep on the real robot).
4. Mid‑mission, **Block path**: the plan bends within ~2 s for BOTH robots; the
   red box stands in Gazebo.
5. Drop a real box (≥ 25 cm tall) in front of the robot (emulation:
   `bash emulation/add_fake_obstacle.sh 0.8 0`): the costmap avoids it and an
   orange mirrored box appears in Gazebo + on the map.
6. **E‑STOP** during a clean: both robots freeze; **RESUME** finishes it.

---

## Prove it works (60‑second smoke test)

Run `preflight`, then watch the two facts that matter most — **localization is
locked** (AMCL ≈ true odom) and **exactly one writer owns `/cmd_vel`** (the safety
mux). Prefix each with the in‑container `docker exec … bash -c '… && <cmd>'` for
emulation, or run in a sourced terminal on the lab PC.

```bash
# 0) one‑line GO/NO‑GO of every link
ros2 run algae_twin preflight                          # → "GO — all links up"

# 1) localization locked: AMCL estimate vs the robot's TRUE odom agree to a few cm
ros2 topic echo /odom --once --field pose.pose.position   # true pose
ros2 run tf2_ros tf2_echo map base_footprint              # AMCL estimate (Ctrl‑C)

# 2) the one‑brain safety bus: exactly ONE publisher on /cmd_vel (the mux)
ros2 topic info /cmd_vel | grep 'Publisher count'      # → Publisher count: 1
ros2 topic hz /cmd_vel                                  # → ~20 Hz

# 3) the loops are real‑time (the thing software GL breaks)
docker logs algae-twin-emu 2>&1 | grep -c "missed its desired rate"   # → 0 when healthy
```

Every check above — plus navigation goals, missions, keepout edits, obstacle
mirror, twin shadow, battery gate, E‑STOP and safety‑stop — is a **ready‑to‑run
script** in [`scripts/proof/`](scripts/proof/): `bash scripts/proof/prove_all.sh`
runs the observe‑only suite (00–09) and tallies PASS/FAIL; the active ones
(`10_nav_goal.sh` … `15_safety_stop.sh`) drive the robot and are run on purpose.
Each command is also documented with its expected output in
[`docs/REFERENCE.md`](docs/REFERENCE.md) → *Proof cookbook*.

---

## Interface contract (summary — full catalog in `docs/REFERENCE.md`)

| Topic | Type | Purpose |
|---|---|---|
| `/algae/add`, `/algae/remove` | PointStamped, String | drop / delete algae |
| `/algae/state` | String (JSON, latched) | mission queue + progress |
| `/edits/add` `/edits/remove` `/edits/clear` | String (JSON) | world edits |
| `/edits/state`, `/keepout_mask` | String / OccupancyGrid (latched) | edits + Nav2 mask |
| `/estop` | Bool (latched) | freeze/resume both robots |
| `/mission/goto`, `/mission/home` | PoseStamped, Empty | manual goals |
| `/clean/active` | Bool (latched) | dispersal motor state (real‑HW hook) |
| `/ui/status` | String (JSON, 5 Hz) | poses, batteries, divergence, e‑stop, safety |
| `/twin/divergence`, `/twin/correction` | Float32, Twist | shadow controller |
| `/cmd_vel`, `/sim/cmd_vel` | TwistStamped / Twist | mux → real / twin |
| `/battery_state`, `/sim/battery_state` | BatteryState | monitoring |

---

## Tests

ROS‑free and hardware‑free; run standalone or under `colcon test`:

```bash
# from twin-only/algae_twin:
python test/test_operator_ui_smoke.py     # browser UI: endpoints, SSE, commands, validation (no ROS)
python test/test_util.py                  # angle/quaternion/battery/map‑transform math (ROS sourced)
python test/test_imports.py               # every node imports; sim strip is gone (ROS sourced)
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Goals abort *"Failed to make progress"*; logs show `Control loop missed its desired rate` | software‑GL render starving the control loop (NOT AMCL) | use the GPU (`--gpu`) or a lighter mode (default / `--rviz` / `--headless`) |
| Robot ignores `/cmd_vel` | TB3 jazzy bringup expects `TwistStamped` | the mux auto‑detects + switches; or set `real_cmd_stamped` on `twin_bridge` |
| Robot & PC don't see each other | domain / RMW mismatch | same `ROS_DOMAIN_ID` (lab = **36**) and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` on both |
| `preflight` NO‑GO `localization` | AMCL has no pose | **Set robot pose** in the UI (or RViz 2D Pose Estimate) |
| `preflight` NO‑GO `robot odom`/`lidar`/`battery` | robot bringup down | start `turtlebot3_bringup`; check WiFi/domain/RMW/`LDS_MODEL` |
| `cmd writer` ≠ 1 | Nav2 bypassed the mux (>1) or mux down (0) | only `twin_bridge` may publish `/cmd_vel` |
| No `/sim/scan` | gpu_lidar has no render backend | provide a GPU, or `LIBGL_ALWAYS_SOFTWARE=1` (slow) |
| Missions refused, "battery low" | voltage gate (`battery_min_voltage` 11.3 V, above OpenCR ~11.0 V cutoff) | recharge; or `/battery_state` is stale |
| Sim time jumps back after Ctrl+C | leftover Gazebo server | `pkill -f 'gz sim'` |

`preflight` prints a one‑line hint for every failed link. Deep dives (AMCL drift
analysis, the GPU/d3d12 recipe, lab‑PC deployment) live in
[`docs/REFERENCE.md`](docs/REFERENCE.md).

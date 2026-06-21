# Algae Twin — how to run

Digital twin for a **physical TurtleBot3 Burger** on **ROS 2 Jazzy** + **Gazebo
Harmonic**: one Nav2 brain drives the real robot and its Gazebo twin in lockstep,
operated from a browser dashboard. This file is the run guide. (How it works is
documented in each node's module docstring under `algae_twin/algae_twin/`.)

You need a **real** TurtleBot3 Burger reachable on the network — there is no
hardware‑free mode in this build.

---

## 1. Prerequisites

ROS 2 **Jazzy** + Gazebo Harmonic + the **turtlebot3 stack** + Nav2 + `ros_gz`.

**Lab PC — install nothing (no sudo).** It already has ROS 2 Jazzy at
`/opt/ros/jazzy` and the turtlebot3 stack built in `~/turtlebot3_ws`.
`scripts/run_lab.sh` builds only `algae_twin` on top of it. If a system package is
missing there is no sudo on the lab laptop — `run_lab.sh` names it; flag it to a TA.

**Home / fresh machine (you have sudo).** Easiest is the Docker image (§2B). To
build natively instead:

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

> This repo carries a top‑level `COLCON_IGNORE` so dropping it inside an existing
> colcon workspace won't double‑build the package. Always build by copying the
> inner `algae_twin/` folder into a workspace (which `run_lab.sh` and the steps
> above do).

---

## 2. Run

**First, start the robot's bringup** on the robot Pi, on the **same**
`ROS_DOMAIN_ID` (lab = `36`, the sticker number) and RMW — leave it running:

```bash
ssh turtlebot@192.168.8.36            # robot IP is on the sticker
export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 ROS_DOMAIN_ID=36 \
       RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch turtlebot3_bringup robot.launch.py
```

### A. Lab PC — one command (recommended)

```bash
./scripts/run_lab.sh                 # deps → build → verify /scan → launch + preflight
./scripts/run_lab.sh --headless      # no 3D Gazebo window (weak/no GPU — safest)
./scripts/run_lab.sh --no-rviz       # skip RViz (opens by default for 2D Pose Estimate)
./scripts/run_lab.sh --check-only    # verify the machine + robot link, don't launch
```

Other flags: `--domain N`, `--ws PATH`, `--keep-edits` (restore last session's
boxes), `--no-ui`, `--no-preflight`. Console is logged to
`~/algae_twin_logs/<stamp>/console.log` — copy it off before leaving (lab laptops
get wiped).

### B. Docker — any PC on the robot's network

```bash
docker build -t algae-twin-only .
docker run -it --rm --network host -e ROS_DOMAIN_ID=36 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp algae-twin-only \
  bash -lc 'ros2 launch algae_twin twin.launch.py'
```

### C. Manual (already in a sourced workspace)

```bash
export ROS_DOMAIN_ID=36                       # must match the robot
ros2 launch algae_twin twin.launch.py         # args: headless, ui, rviz, spawn_x/y/yaw, map, world
```

### Headless / GPU

The twin's `gpu_lidar` renders every cycle. On software GL (no GPU driver,
`llvmpipe`) that starves Nav2's control loop and goals abort with *"Failed to make
progress"*. Check with `glxinfo -B | grep -i renderer`; if it says `llvmpipe`, run
`--headless`. Because the lab PC drives a real robot, prefer `--headless` unless
the GPU is confirmed.

---

## 3. After launch

1. Open the operator UI: **http://localhost:8088**.
2. **Set robot pose once.** AMCL starts at the map origin — in RViz click
   *2D Pose Estimate* on the robot's real spot and drag toward its heading (the red
   lidar points snap onto the walls), or use **Set robot pose** in the UI. The twin
   then snaps onto the real robot.
3. **Check every link:** `ros2 run algae_twin preflight` → `GO — all links up`
   (or a per‑link hint). `run_lab.sh` runs this automatically ~40 s after launch.

Drive it from the dashboard: **place algae** (click the map — navigate + 3‑spin
clean), **block path** (drag a box — keepout that re‑routes both robots), **Nav
goal / Set robot pose** (drag for position + heading), **E‑STOP / RESUME** (freeze
both robots). A real box (≥ 25 cm tall) dropped in front of the robot is mirrored
into the twin + map. Optional keyboard teleop:

```bash
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/nav_cmd_vel_stamped
```

---

## 4. Troubleshooting

| Symptom | Cause → fix |
|---|---|
| Goals abort *"Failed to make progress"*; console shows `Control loop missed its desired rate` | software‑GL render starving the control loop (NOT AMCL) → `--headless` or a real GPU |
| Robot ignores `/cmd_vel` | TB3 jazzy bringup expects `TwistStamped` — the mux auto‑detects + switches; or set `real_cmd_stamped` on `twin_bridge` |
| Robot & PC don't see each other | same `ROS_DOMAIN_ID` (lab = 36) and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` on both; clear a leftover `ROS_LOCALHOST_ONLY=1` |
| `preflight` NO‑GO `localization` | AMCL has no pose → **Set robot pose** (UI or RViz 2D Pose Estimate) |
| `preflight` NO‑GO `robot odom`/`lidar`/`battery` | robot bringup down → start `turtlebot3_bringup`; check Wi‑Fi / domain / RMW / `LDS_MODEL` |
| `cmd writer` ≠ 1 | only `twin_bridge` may publish `/cmd_vel` — >1 means Nav2 bypassed the mux, 0 means the mux is down |
| No `/sim/scan` | the `gpu_lidar` needs a render backend → provide a GPU, or `LIBGL_ALWAYS_SOFTWARE=1` (slow) |
| Keepout edits don't re‑route | the mask only affects planning; the robot finishes the current ~1 s BT cycle first. Check `/keepout_mask` |
| Missions refused, "battery low" | voltage gate (11.3 V, above the OpenCR ~11.0 V cutoff) → recharge; or `/battery_state` is stale |
| Robot stops at an obstacle, "won't go" | the 25 cm front‑cone safety latch — clear past 0.35 m (rotation / reverse still work) |
| Sim time jumps back after Ctrl+C | leftover Gazebo server → `pkill -f 'gz sim'` |

---

## 5. Tests

```bash
# from algae_twin/ :
python test/test_operator_ui_smoke.py     # browser UI: endpoints, SSE, commands (no ROS)
python test/test_util.py                  # angle/quaternion/battery/map math (ROS sourced)
python test/test_imports.py               # every node imports (ROS sourced)
```

License: Apache‑2.0.

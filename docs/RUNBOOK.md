# Lab runbook — run the twin (real Burger ⇄ Gazebo twin)

The happy path, the manual fallback, a hardware-free rehearsal, and a
fix-it table. Lab PC = **native, no Docker, no sudo**; the turtlebot3 stack is
already built in `~/turtlebot3_ws`. `ROS_DOMAIN_ID` **= the robot's sticker
number** (default **36**) on BOTH sides; both on Wi-Fi **AP2IRR10**.

---

## TL;DR
```bash
# 1) ROBOT Pi (ssh) — start it first, leave running:
ssh turtlebot@<robot-ip>
export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 ROS_DOMAIN_ID=<robot#>
ros2 launch turtlebot3_bringup robot.launch.py

# 2) LAB PC — clone + one command:
git clone https://github.com/aleksander-kolev/algae-twin-only.git
cd algae-twin-only
./scripts/run_lab.sh                       # add --headless on a weak GPU; --domain N if not #36

# 3) Set robot pose once (UI or RViz 2D Pose Estimate) → drive from http://localhost:8088
```

---

## 1. Run (happy path)

### 1a. Robot Pi — bringup (ssh, FIRST)
```bash
ssh turtlebot@<robot-ip>                          # IP is on the sticker
export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 ROS_DOMAIN_ID=<robot#>
ros2 launch turtlebot3_bringup robot.launch.py    # leave this terminal running
```

### 1b. Lab PC — clone + run
```bash
git clone https://github.com/aleksander-kolev/algae-twin-only.git
cd algae-twin-only
./scripts/run_lab.sh
```
`run_lab.sh` does the rest: sources ROS + `~/turtlebot3_ws`, checks deps, copies
`algae_twin` into `~/turtlebot3_ws/src`, `colcon build`s it, **verifies the
robot's `/scan`**, launches, runs `preflight` ~40 s in. It **starts with an empty
world** — only the robot + twin come up; **you** place algae/obstacles during the
demo. Useful flags: `--headless` (weak GPU), `--rviz`, `--domain N` (robot ≠ 36),
`--ws PATH`, `--keep-edits` (restore last session's keepout/obstacle boxes),
`--check-only` (verify the link, don't launch).

### 1c. Set robot pose (once, every run)
AMCL starts at the map origin. In **RViz** (`--rviz`) click **2D Pose Estimate**,
click the robot's real spot and drag toward its heading. The red lidar points
should snap onto the map walls. (Or use **Set robot pose** in the browser UI.)
The twin then snaps onto the real robot within a few seconds.

### 1d. Drive + prove
- Operator UI: **http://localhost:8088** — place algae, block a path, E-STOP/resume.
- Teleop (extra terminal, after sourcing the same env):
  ```bash
  ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/nav_cmd_vel_stamped
  ```
- Verify every link (copy the proof folder in once, then run):
  ```bash
  cp -r scripts/proof ~/turtlebot3_ws/proof
  ROS_DOMAIN_ID=<robot#> bash ~/turtlebot3_ws/proof/prove_all.sh   # 00–09 PASS/FAIL
  ```
  Headline check: `03_localization.sh` (AMCL ≈ true odom) and `00_preflight.sh` (GO).

### 1e. Shutdown
```bash
ssh turtlebot@<robot-ip> 'sudo shutdown now'      # BEFORE the power switch
```

---

## 2. Fallbacks

### 2a. Weak / no GPU (software GL) → headless
The 3D Gazebo window on a GPU-less machine starves Nav2's control loop (goals
abort with *"Failed to make progress"*). Drop the window — the robot still drives
and the UI/RViz show everything:
```bash
./scripts/run_lab.sh --headless
```
Confirm GL first if unsure: `glxinfo -B | grep -i renderer` — `llvmpipe` = software
(use `--headless`); a GPU name = any mode is fine.

### 2b. `run_lab.sh` fails / you want manual control
Run the same steps by hand (native, no sudo):
```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
cp -r algae_twin ~/turtlebot3_ws/src/algae_twin           # from the cloned repo
cd ~/turtlebot3_ws && colcon build --packages-select algae_twin && source install/setup.bash
export TURTLEBOT3_MODEL=burger ROS_DOMAIN_ID=<robot#> \
       RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
ros2 topic hz /scan                                       # robot link OK? (~5 Hz)
ros2 launch algae_twin twin.launch.py headless:=true rviz:=true ui:=true
```
If `colcon build` complains about a **duplicate `algae_twin`**, remove the stray
copy: `rm -rf ~/turtlebot3_ws/src/algae_twin` (keep one), rebuild.

### 2c. No robot today — rehearse hardware-free
twin-only needs the real robot; for a hardware-free dry run use the **emulator**
(a wall-clock software robot stands in — same `twin.launch.py`):
```bash
# At home (WSL2 + Docker) — the tested rehearsal:
bash scripts/run_twin_emulated_wsl_docker.sh --rviz     # auto-detects GPU; UI at :8088

# Natively on the lab PC (no Docker), after building algae_twin (2b):
ros2 launch ./emulation/twin_emulated.launch.py headless:=true rviz:=true ui:=true
```
A `preflight` **GO** in emulation ⇒ **GO** in the lab (same package, same launch).

### 2d. Fix-it table
| Symptom | Cause → fix |
|---|---|
| `no /scan from the robot` (script dies) | Pi bringup not up / wrong `ROS_DOMAIN_ID` (must = robot #) / not on AP2IRR10 / RMW mismatch / leftover `ROS_LOCALHOST_ONLY=1`. `run_lab.sh` prints this checklist. |
| Robot drives wrong / skips goals | AMCL not seeded → redo **2D Pose Estimate** until the scan hugs the walls. |
| Goals abort *"Failed to make progress"* + `Control loop missed its desired rate` | software-GL render starvation → `--headless` (or a real GPU). NOT a localization bug. |
| `MISSING: nav2_* / ros_gz_* / rviz2` | system packages, **no sudo on the lab PC → flag a TA** (`run_lab.sh` names the exact one). |
| `MISSING: turtlebot3_*` | `~/turtlebot3_ws` isn't the provisioned workspace → `--ws /correct/path`. |
| nav weaving / grazing walls | `ros2 topic info /cmd_vel` must show **1 publisher** (`twin_bridge`); then re-seed the pose; tune inflation live: `ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.25`. |
| Missions refused, "battery low" | voltage gate (11.3 V) → recharge; or `/battery_state` stale. |
| Sim time jumps back after Ctrl+C | leftover Gazebo: `pkill -f 'gz sim'`. |
| Robot stops at an obstacle "won't go" | the 25 cm front-cone safety **latch** — clear past 0.35 m (rotation/reverse still work). |

> No `nav2` "safe-params" refusal here: twin-only ships its **own** complete
> `config/nav2_params.yaml` and gates `/cmd_vel` via the `twin_bridge` mux — it
> never rewrites the lab's stock nav2 params, so there is no preflight that can
> refuse to launch over them.

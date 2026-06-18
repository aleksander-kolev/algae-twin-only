# Hardware-free test harness — the real robot, emulated exactly

Run the twin-only build on a laptop with **no robot** and confirm it will work
in the lab. The harness emulates the physical TurtleBot3 **exactly**, so the
twin-only package runs completely unchanged — the same `twin.launch.py` the lab
PC runs.

## Why software (not a second Gazebo)

Twin mode runs on the **wall clock** — the real robot is the time authority
(`use_sim_time:=false`). The real TB3 stamps `/scan`, `/odom` and its TF with
its system clock. A Gazebo stand-in stamps in **sim time**, which mismatches
AMCL's wall-clock `map→odom` and breaks the TF chain. So the faithful emulator
is a wall-clock software robot — exactly what `turtlebot3_bringup` is from the
twin's point of view.

`fake_robot.py` publishes the robot's real interface:

* `/odom` + TF `odom→base_footprint` — diff-drive integration of `/cmd_vel`
* `/scan` — LDS-02 (360 beams, 0.16–8 m), a **ray-cast of the same map** Nav2
  localises against, so AMCL gets a genuine scan-to-map fix (just like the lab)
* `/battery_state` — OpenCR pack, slow discharge
* `/joint_states`, `/sound` (best-effort beep)
* subscribes `/cmd_vel` as **TwistStamped** (the TB3 jazzy bringup default)

`twin_emulated.launch.py` runs the package's **unchanged** `twin.launch.py` plus
`fake_robot` and its `robot_state_publisher`. This folder is **test scaffolding,
not part of the shipped package.**

## Run (WSL2 + Docker)

```bash
bash scripts/run_twin_emulated_wsl_docker.sh        # build if needed, then run
```

By default Gazebo runs **headless** and you drive everything from the **browser
operator UI** at <http://localhost:8088> — it shows the map, both robots, the
live scan, the plan, algae, world edits, batteries and divergence. This keeps
Nav2's control loop real-time under software GL: the 3D Gazebo window + RViz are
heavy renderers that otherwise saturate the CPU (~600%+), starve the loop below
10 Hz, stale the `map→odom` TF, and make the controller abort goals. Headless +
the browser UI runs at ~200% CPU with the loops real-time.

A `preflight` GO/NO-GO prints ~50 s in — it should be **GO**. Then drive the full
demo from the UI: place algae (it navigates → 3-spin clean → clears), block a
path, `bash emulation/add_fake_obstacle.sh 0.8 0` (physical→digital mirror),
E-STOP/resume.

Want the 3D Gazebo window + RViz too (needs a real GPU)?

```bash
bash scripts/run_twin_emulated_wsl_docker.sh --gui
```

> The matching robustness fix — `transform_tolerance` raised to 1.0 s in
> `config/nav2_params.yaml` so Nav2 tolerates slow-loop TF staleness — is in the
> shipped package, so it helps the lab PC (also software GL) too.

## What maps to the lab

| here (laptop, no hardware)                | lab PC (real robot)                              |
|-------------------------------------------|--------------------------------------------------|
| `fake_robot.py` (wall-clock emulator)     | the physical TurtleBot3 + `turtlebot3_bringup`   |
| `twin_emulated.launch.py`                 | `ros2 launch algae_twin twin.launch.py`          |
| `run_twin_emulated_wsl_docker.sh`         | `scripts/run_lab.sh`                             |
| everything in `algae_twin/`               | **identical** (byte-for-byte)                    |

Because the package and `twin.launch.py` are identical in both columns, a GO
here means a GO in the lab once the real robot's bringup is up on the matching
`ROS_DOMAIN_ID`.

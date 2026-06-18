# Algae Twin (twin-only) — Reference & Proof Manual

Everything the [README](../README.md) summarises, in full: the process graph,
every node with its topics/services/actions/parameters, the complete topic
catalog, the TF tree, the config files, the data flows — and a **proof cookbook**
that verifies every link with `ros2` from the command line.

## Contents
1. [Conventions](#1-conventions)
2. [Process / launch graph](#2-process--launch-graph)
3. [Node reference](#3-node-reference)
4. [Topic catalog](#4-topic-catalog)
5. [Actions & services](#5-actions--services)
6. [TF frames & tree](#6-tf-frames--tree)
7. [Configuration parameters](#7-configuration-parameters)
8. [Data flows](#8-data-flows)
9. [Proof cookbook](#9-proof-cookbook)
10. [Build & run internals](#10-build--run-internals)
11. [Performance & GPU](#11-performance--gpu)
12. [Lab-PC deployment & safety](#12-lab-pc-deployment--safety)
13. [Tests](#13-tests)

---

## 1. Conventions

All `ros2` commands below assume a **sourced shell on the robot's
`ROS_DOMAIN_ID`**.

**Emulation (Docker)** — either open one shell and stay in it:

```bash
docker exec -it algae-twin-emu bash
# then, inside the container, once:
source /opt/ros/jazzy/setup.bash && source /opt/twin_ws/install/setup.bash && export ROS_DOMAIN_ID=36
```

…or wrap a one-off command:

```bash
docker exec algae-twin-emu bash -lc 'source /opt/ros/jazzy/setup.bash && \
  source /opt/twin_ws/install/setup.bash && export ROS_DOMAIN_ID=36 && <ros2 command>'
```

**Lab PC (native)** — `source /opt/ros/jazzy/setup.bash`, source your workspace
overlay (`~/turtlebot3_ws/install/setup.bash`), `export ROS_DOMAIN_ID=36`, then
run the `ros2` commands directly.

> `/scan`, `/sim/scan`, `/clock` are **BEST_EFFORT** (sensor QoS). To echo them
> add `--qos-profile sensor_data`. Latched topics (`/map`, `/keepout_mask`,
> `/algae/state`, `/edits/state`, `/estop`, `/clean/active`) are
> `TRANSIENT_LOCAL` — late subscribers still get the last value.

---

## 2. Process / launch graph

```
run_lab.sh (lab)  |  run_twin_emulated_wsl_docker.sh (emulation)
        │                         │
        │                         └─ emulation/twin_emulated.launch.py
        │                              ├─ fake_robot.py            (= the real robot)
        │                              ├─ real_robot_state_publisher (URDF, un-prefixed frames)
        │                              └─ includes ▼ (UNCHANGED)
        └───────────────────────► algae_twin/launch/twin.launch.py
                                       ├─ sim_gz.launch.py
                                       │    ├─ gz_sim            (Gazebo Harmonic server [+ GUI])
                                       │    ├─ create burger_twin
                                       │    ├─ parameter_bridge  (config/bridge.yaml + entity services)
                                       │    └─ sim_robot_state_publisher (frame_prefix sim/)
                                       ├─ nav2.launch.py         (renders nav2_params.yaml @tokens)
                                       │    ├─ map_server  amcl  planner_server  controller_server
                                       │    ├─ behavior_server  bt_navigator  costmap_filter_info_server
                                       │    └─ lifecycle_manager_navigation (autostart)
                                       ├─ twin_bridge   map_edit   mission   sim_battery
                                       ├─ pose_sync     obstacle_mirror
                                       ├─ operator_ui   (if ui:=true)
                                       └─ rviz2         (if rviz:=true)
```

`nav2.launch.py` renders `config/nav2_params.yaml` to a temp file, substituting
`@MAP_YAML@`, `@INIT_X@`, `@INIT_Y@`, `@INIT_YAW@`. Nav2 velocity output is
remapped `cmd_vel → /nav_cmd_vel` so **only `twin_bridge` ever writes
`/cmd_vel`**.

---

## 3. Node reference

### Twin application nodes (`algae_twin`)

#### `twin_bridge` — command mux + safety + status
The single writer of `/cmd_vel`. Picks the highest-priority fresh source
(clean > nav), clamps to the burger envelope, applies the front-cone safety stop,
and publishes to **both** robots.

| | |
|---|---|
| **Subscribes** | `/nav_cmd_vel` (Twist), `/nav_cmd_vel_stamped` (TwistStamped), `/clean_cmd_vel` (Twist), `/twin/correction` (Twist), `/estop` (Bool✦), `/clean/active` (Bool✦), `/twin/divergence` (Float32), `/odom` (Odometry), `/sim/ground_truth` (Odometry), `/battery_state` (BatteryState), `/sim/battery_state` (BatteryState), `/scan` (LaserScan, sensor) |
| **Publishes** | `/cmd_vel` (TwistStamped *or* Twist — auto-detected), `/sim/cmd_vel` (Twist), `/ui/status` (String JSON, 5 Hz) |
| **TF** | listens `map→base_footprint` (status pose) |
| **Key params** | `rate 20`, `real_cmd_stamped True`, `real_cmd_autodetect True`, `cmd_timeout 0.6`, `max_lin 0.22`, `max_ang 2.84`, `safety_enable True`, `safety_stop_distance 0.25`, `safety_release_distance 0.35`, `safety_release_hold 0.3`, `safety_front_angle 0.5`, `safety_scan_timeout 0.5` |

(✦ = latched/TRANSIENT_LOCAL)

Priority & freshness: a source counts only if seen within `cmd_timeout` (0.6 s).
On E-STOP, the mux publishes zero. Safety stop cuts **forward** velocity only
(rotation/reverse allowed) and **fails safe** (blocks) on a stale/absent `/scan`.

#### `pose_sync` — twin shadow controller
Keeps the Gazebo twin glued to the real robot.

| | |
|---|---|
| **Subscribes** | `/sim/ground_truth` (Odometry), `/estop` (Bool✦), `/clean/active` (Bool✦) |
| **Publishes** | `/twin/correction` (Twist), `/twin/divergence` (Float32) |
| **TF** | listens `map→base_footprint` (real pose) |
| **Gazebo** | `SetEntityPose` (teleport `burger_twin`) |
| **Key params** | `rate 10`, `kp_lin 0.6`, `kp_ang 1.2`, `max_corr_lin 0.06`, `max_corr_ang 0.5`, `teleport_divergence 1.0`, `teleport_after 2.0`, `teleport_cooldown 5.0`, `model burger_twin` |

Small drift → bounded corrective twist into the mux. Divergence > 1 m sustained
2 s → teleport (suppressed during cleaning and E-STOP).

#### `mission` — algae lifecycle + nav
| | |
|---|---|
| **Subscribes** | `/algae/add` (PointStamped), `/algae/remove` (String), `/mission/goto` (PoseStamped), `/mission/home` (Empty), `/estop` (Bool✦), `/map` (OccupancyGrid✦), `/battery_state`, `/sim/battery_state` (BatteryState) |
| **Publishes** | `/algae/state` (String JSON✦), `/clean/active` (Bool✦), `/sim/sprayer_cmd` (Float64), `/clean_cmd_vel` (Twist) |
| **Action clients** | `navigate_to_pose` (NavigateToPose), `spin` (Spin) |
| **Service client** | `/sound` (turtlebot3_msgs/srv/Sound) |
| **Gazebo** | spawn/remove green algae cylinders |
| **Key params** | `spin_turns 3`, `spin_time_allowance 90`, `battery_min_voltage 11.3`, `algae_radius 0.15`, `home_pose [0,0,0]`, `sprayer_speed 25`, `cleared_linger_sec 20`, `nav_timeout 45`, `placement_clearance 0.25` |

State machine: `idle → nav → clean (Spin × 3 turns) → cleared`. Watchdogs: nav
server unresponsive (15 s) and post-accept stall (`nav_timeout` 45 s) both cancel
+ fail cleanly. Placement is rejected unless the cell is free map space with
`placement_clearance` margin. Battery gate uses **voltage** across both packs.

#### `map_edit` — world edits → keepout mask + Gazebo
| | |
|---|---|
| **Subscribes** | `/map` (OccupancyGrid✦), `/edits/add` (String JSON), `/edits/remove` (String JSON), `/edits/clear` (String JSON) |
| **Publishes** | `/keepout_mask` (OccupancyGrid✦), `/edits/state` (String JSON✦) |
| **Gazebo** | spawn/remove box models (operator=red, mirrored=orange) |
| **Key params** | `world default`, `box_height 0.25`, `persist True`, `persist_file ~/.algae_twin/edits.json` |

Every edit is rasterised into `/keepout_mask` (consumed by both costmaps'
keepout filter) **and** spawned as a Gazebo box. Edits persist across restarts.

#### `obstacle_mirror` — physical → digital
| | |
|---|---|
| **Subscribes** | `/map` (OccupancyGrid✦), `/edits/state` (String✦), `/scan` (LaserScan, sensor), `/twin/divergence` (Float32) |
| **Publishes** | `/edits/add` (String JSON, `source:"mirrored"`), `/edits/remove` (String JSON) |
| **TF** | listens `map→base_scan` |
| **Key params** | `bin_size 0.10`, `min_hits 8`, `decay 0.85`, `map_clearance 0.12`, `box_size 0.20`, `max_range 8.0`, `max_divergence 0.5`, `demote_after 8.0`, `demote_range 3.0` |

Tallies lidar returns the static map can't explain; a bin crossing `min_hits`
becomes a mirrored edit. Skips mirroring while divergence > 0.5 m or stale
(don't trust a mislocalised pose). Retires (`demote`) a mirror once the robot is
close enough to see its cell yet the lidar no longer returns there.

#### `sim_battery` — twin battery
| | |
|---|---|
| **Subscribes** | `/sim/ground_truth` (Odometry), `/clean/active` (Bool✦), `/battery_state` (BatteryState), `/sim/battery/set` (Float32) |
| **Publishes** | `/sim/battery_state` (BatteryState, 1 Hz) |
| **Key params** | `mirror_real True` (set by launch), `initial_percent 100`, `idle_drain 0.01`, `drain_per_meter 0.10`, `drain_per_rad 0.02`, `spray_drain 0.08` |

In twin mode mirrors the real pack; falls back to a discharge model if the real
battery is silent. `/sim/battery/set` resets the simulated charge (UI "Recharge").

#### `operator_ui` — browser dashboard
Embeds a stdlib `http.server` (no GUI library). SSE state push at 10 Hz with a
short-poll fallback; commands are `POST /cmd/<name>`.

| | |
|---|---|
| **Subscribes** | `/map`, `/ui/status`, `/algae/state`, `/edits/state`, `/plan` (Path), `/scan`, `/sim/scan` |
| **Publishes** | `/algae/add`, `/algae/remove`, `/edits/add`, `/edits/remove`, `/edits/clear`, `/estop`, `/mission/goto`, `/mission/home`, `/sim/battery/set`, `/initialpose` (PoseWithCovarianceStamped) |
| **HTTP** | `:8088` — `GET /`, `/map.json`, `/state.json`, `/events` (SSE), `POST /cmd/<name>` |
| **Args** | `--port 8088`, `--host` (`ALGAE_UI_HOST`; `0.0.0.0` in Docker), `--no-browser` |

#### `preflight` — GO/NO-GO
Listens ~6 s and reports every link with a hint; exits non-zero on failure. See
[§9](#9-proof-cookbook). Not part of the running stack — run it on demand.

### Nav2 nodes (stock, configured by `nav2_params.yaml`)
`map_server` (`/map`✦) · `amcl` (TF `map→odom`, `/amcl_pose`, `/particle_cloud`)
· `planner_server` (`/plan`, NavFn) · `controller_server` (DWB, `/nav_cmd_vel`,
`local_costmap`) · `behavior_server` (`spin`/`backup`/`drive_on_heading`/`wait`)
· `bt_navigator` (`navigate_to_pose`) · `costmap_filter_info_server`
(`/costmap_filter_info`✦) · `lifecycle_manager_navigation` (autostart).

### Gazebo & bridge
`gz_sim` server (+GUI if not headless) · `create` (spawns `burger_twin`) ·
`parameter_bridge` (topics from `bridge.yaml` + the three entity services) ·
`sim_robot_state_publisher` (`frame_prefix sim/`).

### Emulation only (not shipped)
`fake_robot` (the real robot's interface, wall clock) · `real_robot_state_publisher`
(burger URDF, un-prefixed `base_footprint→base_scan…`).

---

## 4. Topic catalog

**Real robot (emulation: `fake_robot`; lab: `turtlebot3_bringup`)**

| Topic | Type | QoS | Dir |
|---|---|---|---|
| `/odom` | nav_msgs/Odometry | default | robot → Nav2, bridge, mission |
| `/scan` | sensor_msgs/LaserScan | sensor | robot → amcl, costmaps, safety, mirror |
| `/battery_state` | sensor_msgs/BatteryState | default | robot → mission, sim_battery, bridge |
| `/joint_states` | sensor_msgs/JointState | default | robot → real_robot_state_publisher |
| `/cmd_vel` | geometry_msgs/TwistStamped¹ | default | `twin_bridge` → robot |
| `/sound` | turtlebot3_msgs/srv/Sound | service | mission → robot (beep) |
| `/fake_world/add_box` | geometry_msgs/PointStamped | default | *emulation only* → fake_robot |
| `/fake_world/clear` | std_msgs/Empty | default | *emulation only* → fake_robot |

¹ auto-detected; some bringups use plain `Twist`.

**Navigation**

| Topic | Type | QoS |
|---|---|---|
| `/map` | nav_msgs/OccupancyGrid | latched |
| `/nav_cmd_vel` | geometry_msgs/Twist | default |
| `/plan` | nav_msgs/Path | default |
| `/keepout_mask` | nav_msgs/OccupancyGrid | latched |
| `/costmap_filter_info` | nav2_msgs/CostmapFilterInfo | latched |
| `/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | default |
| `/particle_cloud` | nav2_msgs/ParticleCloud | sensor |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | default |
| `/local_costmap/costmap`, `/global_costmap/costmap` | nav_msgs/OccupancyGrid | latched |

**Gazebo twin (via `parameter_bridge`)**

| Topic | Type | QoS | Dir |
|---|---|---|---|
| `/clock` | rosgraph_msgs/Clock | clock | GZ→ROS (unused by the wall-clock real chain) |
| `/sim/cmd_vel` | geometry_msgs/Twist | default | ROS→GZ (DiffDrive) |
| `/sim/odom` | nav_msgs/Odometry | default | GZ→ROS |
| `/sim/ground_truth` | nav_msgs/Odometry | default | GZ→ROS (`map→sim/base_footprint`) |
| `/sim/scan` | sensor_msgs/LaserScan | sensor | GZ→ROS |
| `/sim/joint_states` | sensor_msgs/JointState | default | GZ→ROS |
| `/sim/sprayer_cmd` | std_msgs/Float64 | default | ROS→GZ (dispersal motor) |
| `/sim/sprayer_particles` | gz ParticleEmitter | gz topic | gz CLI (no bridge pair) |
| `/tf` | tf2_msgs/TFMessage | default | from gz `/sim/tf` — `sim/odom→sim/base_footprint` |

**Twin application**

| Topic | Type | QoS |
|---|---|---|
| `/ui/status` | std_msgs/String (JSON) | default, 5 Hz |
| `/twin/correction` | geometry_msgs/Twist | default |
| `/twin/divergence` | std_msgs/Float32 | default |
| `/sim/battery_state` | sensor_msgs/BatteryState | default |
| `/sim/battery/set` | std_msgs/Float32 | default |
| `/algae/add` · `/algae/remove` | geometry_msgs/PointStamped · std_msgs/String | default |
| `/algae/state` | std_msgs/String (JSON) | latched |
| `/edits/add` · `/edits/remove` · `/edits/clear` | std_msgs/String (JSON) | default |
| `/edits/state` | std_msgs/String (JSON) | latched |
| `/keepout_mask` | nav_msgs/OccupancyGrid | latched |
| `/estop` | std_msgs/Bool | latched |
| `/clean/active` | std_msgs/Bool | latched |
| `/clean_cmd_vel` | geometry_msgs/Twist | default |
| `/mission/goto` · `/mission/home` | geometry_msgs/PoseStamped · std_msgs/Empty | default |

---

## 5. Actions & services

**Actions** (Nav2): `navigate_to_pose` (NavigateToPose, bt_navigator) ·
`navigate_through_poses` · `spin` (Spin, behavior_server) · `backup` ·
`drive_on_heading` · `wait` · `follow_path` (controller) · `compute_path_to_pose`
(planner).

**Services**: `/sound` (turtlebot3_msgs/srv/Sound, real OpenCR) ·
`/world/default/create` (ros_gz_interfaces/SpawnEntity) · `/world/default/remove`
(DeleteEntity) · `/world/default/set_pose` (SetEntityPose) · plus every node's
standard lifecycle/parameter services.

---

## 6. TF frames & tree

The **real** localization chain (what AMCL/Nav2 use) and the **twin** chain are
fully separate — the twin's frames are `sim/`-prefixed and never enter the real
chain:

```
map ── amcl ──► odom ── robot ──► base_footprint ── URDF ──► base_link ─► base_scan
                                                                       ├─► wheel_left/right_link
                                                                       ├─► caster_back_link
                                                                       └─► imu_link
sim/odom ── gz DiffDrive (/sim/tf→/tf) ──► sim/base_footprint ── sim RSP ─► sim/base_link ─► sim/base_scan …
```

* `map→odom`: **amcl** (`tf_broadcast: true`).
* `odom→base_footprint`: the robot (fake_robot or real bringup).
* `base_footprint→base_scan` etc.: `real_robot_state_publisher` from the burger
  URDF — **`base_scan` at x = −0.032 m**, matching the emulator's `LIDAR_X`.
* `map→sim/base_footprint` exists **only** as the `/sim/ground_truth` *odometry
  message* (the gz OdometryPublisher's TF goes to an unused topic), so it does
  not link the two trees.

---

## 7. Configuration parameters

### `config/nav2_params.yaml` (highlights)
Wall clock everywhere (`use_sim_time: false`); frames `base_footprint`/`odom`/`map`;
`scan_topic /scan`, `odom_topic /odom`.

* **amcl** — `DifferentialMotionModel`, `alpha1..5 0.2`, `likelihood_field`,
  `laser_max_range 8.0`, `laser_min_range 0.16`, `max_beams 60`,
  `max_particles 2000`/`min 500`, `update_min_d 0.25`, `update_min_a 0.2`,
  `transform_tolerance 1.0`, `set_initial_pose true` (`@INIT_*@`).
* **controller** (DWB) — `controller_frequency 10`, `max_vel_x 0.22`,
  `max_vel_theta 1.0`, `xy_goal_tolerance 0.15`, progress checker
  `required_movement_radius 0.1` / `movement_time_allowance 15`.
* **costmaps** — `robot_radius 0.105`, `inflation_radius 0.25`, keepout filter on
  both, `transform_tolerance 1.0`, local rolling 3×3 @ 0.05.
* **planner** — NavfnPlanner, `tolerance 0.3`, `allow_unknown true`.

### `config/twin.yaml` (per-node overrides)
`twin_bridge` safety cone (`safety_stop_distance 0.25` …); `pose_sync`
(`teleport_divergence 1.0`, `teleport_after 2.0`); `mission` (`spin_turns 3`,
`battery_min_voltage 11.3`, `nav_timeout 45`, `placement_clearance 0.25`);
`obstacle_mirror` (`min_hits 8`, `max_divergence 0.5`, `demote_after 8.0`);
`sim_battery` discharge rates. `map_to_world [0,0,0]` everywhere (world ≡ map).

### `config/bridge.yaml`
The `ros_gz_bridge` topic map (see [§4](#4-topic-catalog)). **`/sim/tf` →
`/tf`** and `/sim/scan` (SENSOR_DATA) — never add a RELIABLE subscriber to a
best-effort bridge topic (incompatible QoS = silent no data).

### `maps/map.yaml`
`map.pgm`, 86×110, `resolution 0.05`, `origin [-2.051, -4.194, 0]`, trinary,
`occupied_thresh 0.65`, `free_thresh 0.196`.

---

## 8. Data flows

**Command path (one brain, two bodies).**
`Nav2 controller → /nav_cmd_vel` → `twin_bridge` mux (priority clean>nav, clamp,
safety) → `/cmd_vel` (real) **and** `/sim/cmd_vel` (twin, + `/twin/correction`
from `pose_sync`). E-STOP or stale source ⇒ zero.

**Localization.** robot `/scan` + `odom→base_footprint` → `amcl` matches the
likelihood field of `/map` → publishes `map→odom`. Nav2 plans/controls in `map`.

**World edit.** UI/`obstacle_mirror` → `/edits/add` → `map_edit` → `/keepout_mask`
(both costmaps re-route in ~1 s) **and** a Gazebo box (the twin's lidar sees it).

**Obstacle mirror.** real `/scan` returns unexplained by `/map` accumulate per
bin; ≥ `min_hits` → `/edits/add` (`mirrored`) → appears in Gazebo + mask + UI;
retired when no longer seen.

**Mission.** `/algae/add` → spawn patch → `navigate_to_pose` → on success
`/clean/active true` + `Spin ×3` + `/sim/sprayer_cmd` + emitter + beep → patch
`cleared` → removed.

---

## 9. Proof cookbook

Each block: **what you prove**, the **command(s)**, and the **healthy result**.
(Values shown are representative of a real session.)

### 9.0 One-shot: every link
```bash
ros2 run algae_twin preflight
```
```
  OK    status        mode=twin
  OK    gazebo        ground truth
  OK    twin lidar    /sim/scan
  OK    mask          keepout pipeline
  OK    nav2          navigate_to_pose server
  OK    localization  map->base_footprint
  OK    robot odom    /odom
  OK    robot lidar   /scan
  OK    battery       12.2 V
  OK    cmd link      TwistStamped
  OK    cmd writer    1 publisher(s)
  GO — all links up
```

### 9.1 The graph is up
```bash
ros2 node list          # → /amcl /bt_navigator /controller_server /planner_server
                        #   /behavior_server /map_server /twin_bridge /pose_sync
                        #   /mission /map_edit /obstacle_mirror /sim_battery /operator_ui …
ros2 topic list | wc -l # → dozens; spot-check with `ros2 topic list -t`
ros2 action list        # → /navigate_to_pose /spin /backup /follow_path …
```

### 9.2 Real-robot interface is live
```bash
ros2 topic hz /odom                              # → ~30 Hz
ros2 topic hz /scan --qos-profile sensor_data    # → ~5 Hz
ros2 topic echo /scan --qos-profile sensor_data --once | grep -E 'frame_id|range_min|range_max'
#   → frame_id: base_scan   range_min: 0.16   range_max: 8.0
ros2 topic echo /battery_state --once --field voltage     # → ~12.x  (>11.3 gate)
```

### 9.3 Localization is locked (AMCL ≈ truth) — the headline proof
```bash
ros2 topic echo /odom --once --field pose.pose.position   # TRUE pose, e.g. x:0.489 y:-0.765
ros2 run tf2_ros tf2_echo map base_footprint              # AMCL estimate (Ctrl-C)
#   → Translation: [0.446, -0.826, 0.000]   ⇒ agrees within a few cm
ros2 topic echo /amcl_pose --once --field pose.pose.position
```
**Long-nav regression** (the original "drift" report): drive a long, around-an-
obstacle goal and watch both over the whole path — they must stay within a few cm,
and divergence stays well under the 1 m teleport threshold:
```bash
ros2 topic echo /twin/divergence            # → small (cm); never a sustained >1.0
```

### 9.4 One-brain safety bus
```bash
ros2 topic info /cmd_vel | grep -E 'Publisher count|Subscription count'
#   → Publisher count: 1   (ONLY twin_bridge)   Subscription count: 1 (the robot)
ros2 topic hz /cmd_vel                       # → ~20 Hz
ros2 topic echo /nav_cmd_vel --once          # Nav2's raw command (pre-mux)
ros2 topic echo /cmd_vel --once              # what the robot receives (TwistStamped)
```

### 9.5 Navigation goal end-to-end
```bash
# OBSERVE while the operator (or you) sends a goal:
ros2 topic echo /plan --once | grep -c position     # → >0 poses = a path exists
docker logs -f algae-twin-emu | grep -E 'Received a goal|Reached the goal|Failed'
#   healthy → "Received a goal…" then "Reached the goal!" then bt "Goal succeeded"

# DRIVE a goal yourself (⚠ moves the REAL robot on the lab PC):
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.8, y: 0.0}, \
   orientation: {w: 1.0}}}}"
```

### 9.6 Mission / algae clean
```bash
ros2 topic echo /algae/state                 # latched JSON: queue + per-patch progress
ros2 topic echo /clean/active                # → data: true during the 3-spin clean
ros2 topic echo /sim/sprayer_cmd --once      # → data: 25.0 while dispersing
# trigger one without the UI:
ros2 topic pub --once /algae/add geometry_msgs/PointStamped \
  "{header: {frame_id: map}, point: {x: 0.6, y: -0.5}}"
docker logs algae-twin-emu | grep -E 'algae .* placed|cleared|spin completed'
```

### 9.7 World edit → keepout (re-routes both robots)
```bash
ros2 topic echo /keepout_mask --once | grep -E 'width|height'   # mask exists
ros2 topic echo /edits/state                                    # latched JSON list of boxes
ros2 topic pub --once /edits/add std_msgs/String \
  '{data: "{\"cx\":0.85,\"cy\":-0.97,\"size_x\":0.4,\"size_y\":0.4,\"yaw\":0,\"source\":\"operator\"}"}'
docker logs algae-twin-emu | grep -E 'edit .* added|New filter mask arrived'
```

### 9.8 Physical → digital obstacle mirror (emulation)
```bash
bash emulation/add_fake_obstacle.sh 0.8 0    # drop a real box the map can't explain
docker logs -f algae-twin-emu | grep -E 'unmapped obstacle|mirroring|retiring'
#   → "unmapped obstacle at (…) — mirroring into the digital world"
ros2 topic echo /edits/state --once | grep mirrored     # the box, source:"mirrored"
```

### 9.9 Twin shadow
```bash
ros2 topic echo /sim/ground_truth --once --field pose.pose.position   # twin true pose
ros2 topic echo /twin/correction --once      # bounded corrective twist into the mux
ros2 topic echo /twin/divergence             # real↔twin gap (m)
ros2 topic echo /ui/status --once            # JSON: real/sim poses, divergence, safety, estop
```

### 9.10 Battery gate
```bash
ros2 topic echo /battery_state --once --field voltage       # real pack
ros2 topic echo /sim/battery_state --once --field voltage   # twin mirrors it
# below battery_min_voltage (11.3 V) the mission refuses new goals with a UI note
```

### 9.11 E-STOP
```bash
ros2 topic pub --once /estop std_msgs/Bool '{data: true}'
ros2 topic echo /cmd_vel --once              # → all-zero twist; robots frozen
docker logs algae-twin-emu | grep -i 'E-STOP'
ros2 topic pub --once /estop std_msgs/Bool '{data: false}'   # resume
```

### 9.12 Front-cone safety stop
With an obstacle <0.25 m ahead, forward is cut but rotation is allowed:
```bash
ros2 topic echo /ui/status --once --field data | grep -o '"safety": [a-z]*'   # → "safety": true
# while blocked, the mux zeroes forward but passes rotation:
ros2 topic echo /cmd_vel --once              # linear.x == 0.0, angular.z may be non-zero
```

### 9.13 Loops are real-time (performance / GPU proof)
```bash
docker logs algae-twin-emu 2>&1 | grep -c 'missed its desired rate'    # → 0 healthy
docker stats --no-stream algae-twin-emu                                 # CPU well under saturation
# GPU rendering active (emulation, NVIDIA):
nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader  # util >0, VRAM used
```

### 9.14 TF tree snapshot
```bash
ros2 run tf2_tools view_frames        # writes frames.pdf (map→odom→base_footprint→base_scan; sim/* separate)
ros2 run tf2_ros tf2_echo odom base_footprint   # robot motion
ros2 run tf2_ros tf2_echo base_footprint base_scan   # → x:-0.032 (lidar offset)
```

---

## 10. Build & run internals

**Emulation runner** (`scripts/run_twin_emulated_wsl_docker.sh`): builds/uses
image `algae-twin-only`, removes any old `algae-twin-emu`, runs detached with
`ROS_DOMAIN_ID=36`, port `8088`, live-mounts `emulation/` read-only, waits ~50 s,
runs `preflight`. Flags: `--build`, `--gui`, `--rviz`, `--gpu`, `--no-gpu`,
`--check`, `--no-preflight`. The container launches
`emulation/twin_emulated.launch.py headless:=… rviz:=… ui:=true`.

**Native build**: copy `algae_twin/` into a TB3 workspace `src/`, `colcon build`,
source the overlay (see [README](../README.md) → Prerequisites). `run_lab.sh`
automates deps → build → `/scan` check → launch + preflight.

**Docker (lab, real robot)**: `--network host` so DDS reaches the robot;
`ROS_DOMAIN_ID` + `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` must match the robot.

---

## 11. Performance & GPU

**Root cause of "2nd/3rd goal fails / robot stalls / teleports": control-loop
starvation from software-GL rendering — not AMCL.** Gazebo renders the twin's
`gpu_lidar` each cycle; on `llvmpipe` (software) that runs in CPU bursts that
stall Nav2's executor. The controller/planner loops drop below 10 Hz, DWB stops
commanding effective velocity, the progress checker aborts after 15 s, and the
mission cancels at `nav_timeout`. Throughout, **AMCL stays locked to a few cm** —
verified live and in offline simulation (a faithful AMCL particle filter does not
drift on this map with either perfect or noisy odom).

Symptoms to confirm it's starvation (not localization):
```
controller_server: Failed to make progress
controller_server: Control loop missed its desired rate of 10 Hz. Current loop rate is 6.x Hz
planner_server:    Planner loop missed its desired rate … 3.x Hz
```
…while `tf2_echo map base_footprint` ≈ `/odom` (no drift).

**Fix — render on the GPU.** On WSL2 there is no native NVIDIA OpenGL; GL is
**Mesa's `d3d12` driver** over the WSL D3D12 libs, reaching the GPU via
`/dev/dxg`. The emulation runner sets this automatically when a GPU is present:

```
--gpus all
-e NVIDIA_VISIBLE_DEVICES=all  -e NVIDIA_DRIVER_CAPABILITIES=all
-e GALLIUM_DRIVER=d3d12         -e MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
-v /usr/lib/wsl:/usr/lib/wsl:ro
# inside the container, before launch:  export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH
# and do NOT set LIBGL_ALWAYS_SOFTWARE=1
```
Verify the renderer flipped off software (needs `mesa-utils`):
```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
LD_LIBRARY_PATH=/usr/lib/wsl/lib eglinfo -B | grep -i renderer
#   → OpenGL … renderer: D3D12 (NVIDIA …)      (NOT "llvmpipe")
```
> Knob that works: **`GALLIUM_DRIVER=d3d12`** (not `MESA_LOADER_DRIVER_OVERRIDE`).
> Even on the GPU the `--gui` 3D window's gz client stays CPU-heavy (~200%); for
> the most reliable nav use the default/`--rviz` modes.

---

## 12. Lab-PC deployment & safety

The lab PC runs **natively** — none of the WSL/Docker/`d3d12` plumbing applies,
and `LIBGL_ALWAYS_SOFTWARE` is **not** set. Gazebo uses the machine's real GL
driver. So:

1. **Check GL once:** `glxinfo -B | grep -i renderer` — must show the GPU, not
   `llvmpipe`. If `llvmpipe`: `sudo ubuntu-drivers autoinstall` (NVIDIA) or run
   headless.
2. **Prefer `run_lab.sh --headless`** for real-robot runs unless you have
   confirmed the GPU holds the loop — the cosmetic 3D Gazebo window must never
   starve the safety-critical control loop driving a real robot. The UI + RViz
   still show the robot, scan, costmaps and plan.
3. Match `ROS_DOMAIN_ID` (36) and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` with the
   robot; **Set robot pose** once so AMCL converges; confirm with
   [§9.0](#90-one-shot-every-link) → GO and [§9.13](#913-loops-are-real-time-performance--gpu-proof)
   → 0 rate-misses while driving.

---

## 13. Tests

ROS-free / hardware-free (`twin-only/algae_twin/test/`):

```bash
python test/test_operator_ui_smoke.py   # UI endpoints, SSE, command dispatch, input validation (no ROS)
python test/test_util.py                # angles, quaternions, battery gate, map↔world transform (ROS sourced)
python test/test_imports.py             # every node imports; the sim-only path is gone (ROS sourced)
colcon test --packages-select algae_twin
```

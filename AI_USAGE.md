# AI Usage Statement — Algae Twin

- **Project:** Algae Twin (TurtleBot3 Burger digital twin)
- **Team:** Team 36
- **Date:** 2026-06-22

## Roles

Team 36 designed the project and made all of the architecture and design
decisions: the one shared Nav2 "brain" that drives the real robot and its Gazebo
twin together, the two-way world sync (map keepout edits and mirrored real
obstacles), the single command bus with the safety stop, the battery and mission
rules, and the browser operator UI.

Claude Code (Anthropic) was used as an AI coding assistant, only for **debugging,
documentation, and low-level code**, and always under the team's direction. It did
not make any design or architecture decisions on its own.

## Course files

No AI has had access to the course files.

## Responsibility

Team 36 has reviewed and understood all of the code and documentation, verified
the system by running it on the real robot and the Gazebo twin, and takes full
responsibility for the work.

## Prompts

The list below is a paraphrased summary of what we asked the AI to do — short
descriptions, not word-for-word prompts.

**Setup & the Gazebo twin**

1. Create a single ROS 2 Jazzy (ament_python) package `algae_twin` for a
   TurtleBot3 Burger digital twin. Bring up a Gazebo Harmonic (gz-sim) world and
   spawn a `burger_twin` model with `ros_gz_sim create`. Bridge it with
   `ros_gz_bridge` (parameter_bridge) so the twin publishes `/sim/scan`
   (gpu_lidar), `/sim/odom` and `/sim/ground_truth` (DiffDrive plugin),
   `/sim/joint_states` and `/clock`, and subscribes to `/sim/cmd_vel`. Prefix the
   twin's TF frames with `sim/` so they never collide with the real robot's frames.
2. I have lab map files (`map.pgm` + `map.yaml`). Wire them so Nav2's `map_server`
   and the Gazebo world share one coordinate frame — the gz world origin equals the
   `map` frame — so a point on the map is the same point in Gazebo (identity
   map→world transform).

**One Nav2 brain & the two-way command path**

3. Run one Nav2 stack (lifecycle nodes: `map_server`, `amcl`, `planner_server`,
   `controller_server`, `behavior_server`, `bt_navigator`, `lifecycle_manager`)
   that localizes and drives the REAL robot from its `/scan` and `/odom`. Remap
   Nav2's `cmd_vel` output to `/nav_cmd_vel` so Nav2 never talks to a robot directly.
4. Write a single `twin_bridge` node that is the ONLY publisher of `/cmd_vel`. It
   subscribes to `/nav_cmd_vel` and mirrors every command BOTH ways: to the real
   robot on `/cmd_vel` (`geometry_msgs/TwistStamped`) and to the Gazebo twin on
   `/sim/cmd_vel` (`Twist`), so the two bodies move from one command. Auto-detect
   whether the robot expects `Twist` or `TwistStamped`, and clamp linear/angular
   speed to the Burger's limits.

**Keeping the twin in sync with the real robot**

5. Write a `pose_sync` node for two-way twin↔real sync: compare the real pose (the
   `map`→`base_footprint` TF from AMCL) with the twin pose (`/sim/ground_truth`),
   publish the error on `/twin/divergence`, and feed a small corrective `Twist` on
   `/twin/correction` back into the mux so the twin stays glued. If divergence stays
   over a threshold, teleport the twin in Gazebo via the `set_pose` service.
6. Add a `/twin/resync` topic (and a UI button) that snaps the twin straight onto
   the real robot's current pose on demand.
7. Write a `sim_battery` node that mirrors the real `/battery_state` to
   `/sim/battery_state` so the twin reports the real pack; if the real battery goes
   silent, fall back to a discharge model from the twin's `/sim/odom` motion and the
   sprayer load.

**Two-way world: real→digital and digital→real**

8. Write an `obstacle_mirror` node (real→digital): accumulate the real `/scan` into
   a grid, find returns the static `/map` can't explain, and once stable publish
   them as "mirrored" world edits so a real obstacle shows up in Gazebo and on the
   map. Drop a mirrored obstacle after a few seconds of the lidar no longer seeing
   it. Gate it on good localization (enough scan matching the map) so a bad AMCL
   pose can't spawn phantom boxes.
9. Write a `map_edit` node (digital→real): from the operator UI, rasterize blocked
   areas into a Nav2 keepout `nav_msgs/OccupancyGrid` on `/keepout_mask` (consumed
   by `costmap_filter_info_server` + `KeepoutFilter` on both global and local
   costmaps) so the SHARED planner re-routes the real robot, and spawn the matching
   box in Gazebo at the same time. Persist edits across restarts.

**Mission, navigation & Gazebo actuation**

10. Write a `mission` node for the algae-cleaning cycle: on `/algae/add`, check the
    point is free on the `/map`, spawn a green patch in Gazebo, send a
    `nav2_msgs/action/NavigateToPose` goal (one brain → both robots drive there),
    and on arrival run a `nav2_msgs/action/Spin` for 3 full turns. Handle goal
    rejection, result timeouts and cancellation with watchdogs.
11. Add the dispersal motor to the `burger_twin` SDF: a revolute `sprayer_joint`
    driven by a `JointController` plugin on `/sim/sprayer_cmd`, plus a
    `particle_emitter` toggled on `/sim/sprayer_particles`. During the spins, latch
    `/clean/active` true (the hook for real dispersal hardware) and beep the real
    robot through the `/sound` service.
12. Gate missions on battery voltage — refuse to start a new clean below a safe
    threshold.

**Safety, e-stop, UI & checks**

13. Add a front-cone collision safety-stop inside `twin_bridge`: cut forward
    `/cmd_vel` when the real `/scan` sees an obstacle within ~25 cm ahead, latch
    until it clears, and fail safe (stop) if `/scan` goes stale — applied to both
    robots.
14. Add a latched `/estop` topic that freezes BOTH robots (zero `/cmd_vel` and
    `/sim/cmd_vel`) and pauses the mission; resume continues it.
15. Publish one `/ui/status` JSON topic (both poses, battery voltages, divergence,
    e-stop/safety flags). Write an `operator_ui` node: a browser dashboard served
    from the Python stdlib (no GUI library) that subscribes to `/map`, `/ui/status`,
    `/algae/state`, `/edits/state`, `/plan`, `/scan`, streams to the page over SSE,
    and turns clicks into publishes (`/algae/add`, `/edits/*`, `/mission/*`,
    `/estop`, `/initialpose`, `/twin/resync`).
16. Write a `preflight` node — a ~6-second go/no-go that checks every link:
    `/ui/status`, Gazebo `/sim/ground_truth` + `/sim/scan`, `/keepout_mask`, the
    `navigate_to_pose` action server, the `map`→`base_footprint` TF, the robot's
    `/odom` `/scan` `/battery_state`, and that exactly ONE node publishes `/cmd_vel`.

**Documentation**

17. Write a README with simple build/run instructions for the real robot + twin.

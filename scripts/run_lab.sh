#!/usr/bin/env bash
# Algae Twin — one-command runner for the LAB PC (native Jazzy, no Docker,
# no sudo). The TurtleBot3 stack is expected built from source in
# ~/turtlebot3_ws (the lab convention; override with --ws PATH).
#
#   ./scripts/run_lab.sh                 # real robot + digital twin
#   ./scripts/run_lab.sh --check-only    # preflight the machine, don't launch
#
# What it does: source ROS + the TB3 workspace -> verify every dependency
# (clear remedy per missing item) -> copy algae_twin into the workspace ->
# build -> verify the robot link (/scan) in twin mode -> launch, with an
# automatic `algae_twin preflight` report ~40 s in. Console is logged to
# ~/algae_twin_logs/<timestamp>/console.log.
#
# Robot side (separate terminal, BEFORE this script — it will remind you):
#   ssh turtlebot@192.168.8.36     # robot IP is on a sticker; ROS_DOMAIN_ID = robot number
#   export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 ROS_DOMAIN_ID=36 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#   ros2 launch turtlebot3_bringup robot.launch.py
set -eo pipefail

MODE=twin
DOMAIN="${ROS_DOMAIN_ID:-36}"        # lab robots use the sticker number (36)
WS="${TB3_WS:-$HOME/turtlebot3_ws}"
ROBOT_HOST="${ROBOT_HOST:-turtlebot@192.168.8.36}"
RVIZ=true              # lab needs RViz for the 2D Pose Estimate (--no-rviz to skip)
HEADLESS=false
UI=true
PREFLIGHT=true
CHECK_ONLY=false
KEEP_EDITS=false      # default: start with an EMPTY world (you place algae/obstacles)

usage() { grep '^#' "$0" | sed 's/^# \{0,1\}//' | head -18; cat <<EOF

flags:
  --domain N       ROS_DOMAIN_ID (default: $DOMAIN — the robot's sticker number)
  --ws PATH        TurtleBot3 workspace (default: $WS)
  --no-rviz        don't open RViz (default: RViz opens for the 2D Pose Estimate)
  --headless       no Gazebo 3D window (weak GPU; UI/RViz still show all)
  --no-ui          skip the operator UI
  --keep-edits     restore keepout/obstacle edits from a previous session
                   (default: start with an EMPTY world — you place objects)
  --no-preflight   skip the automatic link check after launch
  --check-only     verify machine + robot link and exit
EOF
exit 0; }

while [ $# -gt 0 ]; do
  case "$1" in
    --domain) DOMAIN="$2"; shift ;;
    --ws) WS="$2"; shift ;;
    --rviz) RVIZ=true ;;
    --no-rviz) RVIZ=false ;;
    --headless) HEADLESS=true ;;
    --no-ui) UI=false ;;
    --keep-edits) KEEP_EDITS=true ;;
    --no-preflight) PREFLIGHT=false ;;
    --check-only) CHECK_ONLY=true ;;
    -h|--help) usage ;;
    *) echo "unknown flag: $1 (try --help)"; exit 2 ;;
  esac
  shift
done

say()  { printf '\n\033[1;36m== %s\033[0m\n' "$*"; }
ok()   { printf '   \033[32mOK\033[0m  %s\n' "$*"; }
warn() { printf '   \033[33mWARN\033[0m %s\n' "$*"; }
die()  { printf '\n\033[31mERROR: %s\033[0m\n' "$*" >&2; exit 1; }

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG_SRC="$REPO_DIR/algae_twin"
[ -f "$PKG_SRC/package.xml" ] || die "algae_twin package not found at $PKG_SRC (run from the cloned repo)"

say "Environment"
[ -f /opt/ros/jazzy/setup.bash ] || die "/opt/ros/jazzy not found — wrong machine?"
# ROS setup files use unbound vars; keep 'set -u' off around sourcing
source /opt/ros/jazzy/setup.bash
ok "ROS 2 Jazzy"
[ -f "$WS/install/setup.bash" ] || die \
"TurtleBot3 workspace not found at $WS — the lab keeps the TB3 stack built in ~/turtlebot3_ws.
 Different location? re-run with: --ws /path/to/turtlebot3_ws"
source "$WS/install/setup.bash"
ok "TB3 workspace: $WS"

say "Dependencies"
MISSING_SYS=(); MISSING_TB3=()
# needed on the LAPTOP (turtlebot3_bringup runs on the robot Pi, not here)
for p in turtlebot3_description turtlebot3_gazebo turtlebot3_msgs; do
  ros2 pkg prefix "$p" >/dev/null 2>&1 && ok "$p" || { MISSING_TB3+=("$p"); warn "MISSING: $p"; }
done
ros2 pkg prefix turtlebot3_teleop >/dev/null 2>&1 \
  && ok "turtlebot3_teleop" || warn "turtlebot3_teleop absent (optional: keyboard teleop)"
for p in nav2_map_server nav2_amcl nav2_controller nav2_planner nav2_behaviors \
         nav2_bt_navigator nav2_lifecycle_manager ros_gz_sim ros_gz_bridge; do
  ros2 pkg prefix "$p" >/dev/null 2>&1 && ok "$p" || { MISSING_SYS+=("$p"); warn "MISSING: $p"; }
done
[ ${#MISSING_TB3[@]} -eq 0 ] || die \
"TB3 packages missing from $WS — that workspace must contain the built turtlebot3 stack (check --ws PATH)."
[ ${#MISSING_SYS[@]} -eq 0 ] || die \
"system packages missing (${MISSING_SYS[*]}) — these are apt packages (ros-jazzy-nav2-bringup, ros-jazzy-ros-gz);
 NO SUDO on the lab laptop: flag it to a TA."
ok "operator UI is browser-based (stdlib HTTP — no python3-tk / PyQt5 needed)"

say "Install algae_twin into the workspace"
DUPS=$(find "$WS/src" -name package.xml -path '*algae_twin*' \
       ! -path "$WS/src/algae_twin/*" 2>/dev/null || true)
[ -z "$DUPS" ] || die "another algae_twin copy exists in the workspace (colcon would abort):
$DUPS
 remove it, then re-run."
rm -rf "$WS/src/algae_twin"
cp -r "$PKG_SRC" "$WS/src/algae_twin"
ok "copied $PKG_SRC -> $WS/src/algae_twin"

say "Build"
( cd "$WS" && colcon build --packages-select algae_twin ) \
  || die "build failed — scroll up for the first error"
source "$WS/install/setup.bash"
ok "algae_twin built"

export ROS_DOMAIN_ID="$DOMAIN" TURTLEBOT3_MODEL=burger
# RMW must match the robot (TB3 jazzy uses fastrtps); a mismatch silently breaks
# discovery — the #1 cause of "no /scan" even with the right domain id.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# a leftover ROS_LOCALHOST_ONLY=1 in the operator's shell confines DDS to loopback and
# hides the robot's /scan with a misleading diagnosis — neutralize it explicitly.
export ROS_LOCALHOST_ONLY=0
say "ROS_DOMAIN_ID=$DOMAIN  RMW=$RMW_IMPLEMENTATION  mode=$MODE"

if [ "$MODE" = twin ]; then
  say "Robot link check (/scan)"
  if timeout 8 ros2 topic echo /scan --once >/dev/null 2>&1; then
    ok "robot lidar visible"
  else
    [ "$CHECK_ONLY" = true ] && warn "no /scan from the robot" || die \
"no /scan from the robot. Checklist:
   1. robot bringup running?   ssh $ROBOT_HOST
      export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 ROS_DOMAIN_ID=$DOMAIN
      export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      ros2 launch turtlebot3_bringup robot.launch.py
   2. laptop + robot on the lab Wi-Fi (AP2IRR10)?
   3. same ROS_DOMAIN_ID on both sides? (sticker number; this run: $DOMAIN — override: --domain N)
   4. same RMW_IMPLEMENTATION on both sides? (this run: $RMW_IMPLEMENTATION)
   5. no robot today? this build runs against the real robot only — there is no
      hardware-free fallback in this repo."
  fi
fi

if [ "$CHECK_ONLY" = true ]; then
  say "Check-only: machine is ready. Launch with: $0"
  exit 0
fi

# Start with an EMPTY world: map_edit restores keepout/obstacle edits persisted
# from a previous session (~/.algae_twin/edits.json) on launch. Clear them so the
# demo begins clean — YOU place algae + obstacles. (--keep-edits to restore.)
if [ "$KEEP_EDITS" != true ]; then
  rm -f "$HOME/.algae_twin/edits.json" "$HOME/.algae_twin/edits.json.tmp" 2>/dev/null || true
  say "world starts EMPTY — persisted edits cleared (place objects yourself; --keep-edits to restore)"
else
  say "--keep-edits: restoring keepout/obstacle edits from the last session"
fi

LOG_DIR="$HOME/algae_twin_logs/$(date -u +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

cat <<EOF

──────────────────────────────────────────────────────────────
 Launching Algae Twin ($MODE). Console -> $LOG_DIR/console.log
   * twin mode: robot bringup must be running (see above).
   * if the robot did not start at the map origin: UI tool
     "Set robot pose" once (or RViz 2D Pose Estimate with --rviz)
     — the twin teleports onto the real robot a moment later.
   * operator UI: a browser opens automatically — else open the
     http://localhost:8088 URL the operator_ui node prints.
   * teleop (extra terminal, after sourcing the same setup):
       ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/nav_cmd_vel_stamped
   * robot shutdown when done:  ssh $ROBOT_HOST 'sudo shutdown now'
──────────────────────────────────────────────────────────────
EOF

if [ "$PREFLIGHT" = true ]; then
  ( sleep 40
    echo; echo '== automatic preflight (40 s after launch) =='
    ros2 run algae_twin preflight || true ) &
  PREFLIGHT_PID=$!
  trap 'kill "$PREFLIGHT_PID" 2>/dev/null || true' EXIT INT TERM
fi

ros2 launch algae_twin twin.launch.py \
    "rviz:=$RVIZ" "headless:=$HEADLESS" "ui:=$UI" \
  2>&1 | tee "$LOG_DIR/console.log"

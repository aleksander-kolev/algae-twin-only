#!/usr/bin/env bash
# Algae Twin (TWIN-ONLY) — run on a laptop with NO robot, emulating the real
# TurtleBot3 EXACTLY. A wall-clock software robot (emulation/fake_robot.py)
# stands in for `turtlebot3_bringup` on the robot's Pi, so this runs the SAME
# twin.launch.py the lab PC runs — only the robot source differs (here: the
# emulator; in the lab: the real bringup). WSL2 + Docker, GUI via WSLg.
#
#   bash scripts/run_twin_emulated_wsl_docker.sh           # build if needed, run
#   bash scripts/run_twin_emulated_wsl_docker.sh --build   # force a rebuild
#   bash scripts/run_twin_emulated_wsl_docker.sh --gui      # add 3D Gazebo + RViz (GPU recommended)
#   bash scripts/run_twin_emulated_wsl_docker.sh --no-gpu   # force software GL (llvmpipe)
#   bash scripts/run_twin_emulated_wsl_docker.sh --gpu      # force GPU render (auto-detected by default)
#   bash scripts/run_twin_emulated_wsl_docker.sh --check    # preflight a running stack
#
# By default Gazebo runs headless and you drive everything from the browser
# operator UI at http://localhost:8088 (map, both robots, scan, plan, algae,
# batteries, divergence). --gui adds the heavy 3D Gazebo + RViz windows via
# WSLg for machines with a real GPU. Stop with: docker stop algae-twin-emu
set -euo pipefail

IMG=algae-twin-only
NAME=algae-twin-emu
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOMAIN=36
SETUP='source /opt/ros/jazzy/setup.bash && source /opt/twin_ws/install/setup.bash'

# Default: headless Gazebo + the browser UI — light enough that Nav2 holds its
# loop rate under software GL (the 3D Gazebo window + RViz are the CPU hogs and
# starve the control loop). The browser UI shows everything (map, both robots,
# scan, plan, algae, batteries, divergence). --gui adds the heavy 3D + RViz
# windows for machines with a real GPU.
GUI=false
RVIZ_ONLY=false
BUILD=false
CHECK_ONLY=false
PREFLIGHT=true
GPU=auto          # auto: use the NVIDIA GPU if Docker can reach it, else software GL
while [ $# -gt 0 ]; do
  case "$1" in
    --build) BUILD=true ;;
    --gui) GUI=true ;;
    --rviz) RVIZ_ONLY=true ;;
    --gpu) GPU=true ;;
    --no-gpu) GPU=false ;;
    --no-preflight) PREFLIGHT=false ;;
    --check) CHECK_ONLY=true ;;
    -h|--help) grep '^#' "$0" | sed 's/^# \{0,1\}//' | head -26; exit 0 ;;
    *) echo "unknown flag: $1 (try --help)"; exit 2 ;;
  esac; shift
done
# View modes (Gazebo always RUNS; this is just what windows open):
#   default : headless gz + browser UI            (lightest, reliable nav)
#   --rviz  : headless gz + RViz + browser UI     (full view, reliable nav: RViz
#             shows both robots/scan/costmaps/plan without the heavy gz 3D window)
#   --gui   : 3D Gazebo window + RViz + UI         (prettiest, but the 3D render
#             starves nav on software GL -> twin lags + teleports)
if [ "$GUI" = true ]; then HL=false; RV=true
elif [ "$RVIZ_ONLY" = true ]; then HL=true; RV=true
else HL=true; RV=false; fi

run_preflight() {
  echo; echo "== preflight (twin mode go/no-go — emulated robot present) =="
  docker exec "$NAME" bash -c \
    "$SETUP && ROS_DOMAIN_ID=$DOMAIN ros2 run algae_twin preflight" || true
}

if [ "$CHECK_ONLY" = true ]; then
  docker ps --format '{{.Names}}' | grep -qx "$NAME" \
    || { echo "container '$NAME' is not running — start it first."; exit 1; }
  run_preflight
  exit 0
fi

[ "$BUILD" = true ] && docker build -t "$IMG" "$REPO"
docker image inspect "$IMG" >/dev/null 2>&1 || docker build -t "$IMG" "$REPO"
docker rm -f "$NAME" >/dev/null 2>&1 || true

# WSLg X server (native docker falls back to /tmp/.X11-unix)
WSLG_X=/run/desktop/mnt/host/wslg/.X11-unix
[ -d "$WSLG_X" ] || WSLG_X=/tmp/.X11-unix

# GPU vs software GL. Gazebo renders the twin's lidar every cycle; on software GL
# (llvmpipe) that burns a CPU thread in bursts and starves Nav2's control loop
# (the robot stops making progress). On WSL2 there is NO native NVIDIA OpenGL —
# GL is Mesa's d3d12 driver layered on the WSL D3D12 libs (/usr/lib/wsl/lib),
# selected with GALLIUM_DRIVER=d3d12 + the NVIDIA adapter, reaching the GPU via
# /dev/dxg (needs --gpus all / NVIDIA Container Toolkit). Auto-detected; override
# with --gpu / --no-gpu. (This whole block is WSL+Docker-specific — the lab PC
# runs Gazebo natively and uses its GPU's normal GL driver; see docs.)
GPU_ARGS=(); GL_ARGS=(-e LIBGL_ALWAYS_SOFTWARE=1); GL_PREP=":"
if [ "$GPU" != false ] \
   && { [ "$GPU" = true ] || docker run --rm --gpus all "$IMG" true >/dev/null 2>&1; }; then
  echo "GPU: NVIDIA reachable — hardware GL (Mesa d3d12 over WSL /dev/dxg)"
  GPU_ARGS=(--gpus all
            -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all
            -e GALLIUM_DRIVER=d3d12 -e MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
            -v /usr/lib/wsl:/usr/lib/wsl:ro)
  GL_ARGS=()
  GL_PREP="export LD_LIBRARY_PATH=/usr/lib/wsl/lib:\${LD_LIBRARY_PATH:-}"
else
  echo "GPU: software GL (llvmpipe) — control loop may stall under load; --gpu to force"
fi

docker run -d --rm --name "$NAME" -p 8088:8088 \
  "${GPU_ARGS[@]}" \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_QPA_PLATFORM=xcb \
  "${GL_ARGS[@]}" \
  -e XDG_RUNTIME_DIR=/tmp/xdg \
  -e ALGAE_UI_HOST=0.0.0.0 \
  -e PYTHONUNBUFFERED=1 \
  -e TURTLEBOT3_MODEL=burger \
  -e ROS_DOMAIN_ID="$DOMAIN" \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v "$WSLG_X:/tmp/.X11-unix" \
  -v "$REPO/emulation:/opt/emulation:ro" \
  "$IMG" \
  bash -c "mkdir -p /tmp/xdg && chmod 700 /tmp/xdg
    $SETUP
    $GL_PREP
    exec ros2 launch /opt/emulation/twin_emulated.launch.py \
      headless:=$HL rviz:=$RV ui:=true"

cat <<EOF

  Algae Twin (twin-only) — emulated-robot run. Software GL: first start ~40-90 s.
    * Open the operator UI:  http://localhost:8088
      It shows EVERYTHING: the map, BOTH robots (the emulated real one + the
      twin), live scan, the plan, algae, world edits, batteries and divergence.
    * Drive the demo there: place algae, block a path, E-STOP / resume.
    * (Gazebo runs headless for reliable nav under software GL. Want the 3D
      window + RViz too? re-run with --gui — needs a real GPU.)
    * logs:  docker logs -f $NAME      stop:  docker stop $NAME
    * re-check anytime:  bash scripts/run_twin_emulated_wsl_docker.sh --check
EOF

if [ "$PREFLIGHT" = true ]; then
  echo; echo "  waiting ~50 s for the stack + emulated robot to come up, then preflight…"
  sleep 50
  run_preflight
fi

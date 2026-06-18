#!/usr/bin/env bash
# Emulate dropping a REAL obstacle in front of the robot (physical -> digital
# test): the fake robot's lidar then sees it, and obstacle_mirror mirrors it
# into the twin (orange box) + the keepout mask, so both robots re-route — the
# same path the real robot + a real box exercise in the lab.
#
#   bash emulation/add_fake_obstacle.sh [MAP_X] [MAP_Y]   # default 0.8 0.0
#   bash emulation/add_fake_obstacle.sh --clear           # remove them
set -euo pipefail
NAME=algae-twin-emu
DOMAIN=36
SETUP='source /opt/ros/jazzy/setup.bash && source /opt/twin_ws/install/setup.bash'

docker ps --format '{{.Names}}' | grep -qx "$NAME" \
  || { echo "container '$NAME' is not running — start it first."; exit 1; }

if [ "${1:-}" = "--clear" ]; then
  docker exec "$NAME" bash -c "$SETUP && ROS_DOMAIN_ID=$DOMAIN \
    ros2 topic pub --once /fake_world/clear std_msgs/msg/Empty '{}'" >/dev/null
  echo "cleared emulated real obstacles (obstacle_mirror retires the mirror once the robot reconfirms it is gone)"
  exit 0
fi

X="${1:-0.8}"; Y="${2:-0.0}"
docker exec "$NAME" bash -c "$SETUP && ROS_DOMAIN_ID=$DOMAIN \
  ros2 topic pub --once /fake_world/add_box geometry_msgs/msg/PointStamped \
  '{header: {frame_id: map}, point: {x: $X, y: $Y}}'" >/dev/null
echo "dropped a real obstacle at map ($X, $Y) — within ~2 s watch an ORANGE box"
echo "appear in the twin (Gazebo) + a keepout on the map; both robots re-route."

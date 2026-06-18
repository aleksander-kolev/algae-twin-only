#!/usr/bin/env bash
# PROOF 02 — the real-robot interface is live: /odom, /scan, /battery_state.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; source "$HERE/_common.sh"
say "PROOF 02 — real-robot interface (/odom /scan /battery_state)"
rc=0

ohz="$(timeout 4 ros2 topic hz /odom 2>/dev/null | awk '/average rate/{print $3; exit}')"
if [ -n "$ohz" ]; then ok "/odom publishing (~${ohz} Hz)"; else fail "/odom silent — robot/emulator down?"; rc=1; fi

# /scan is BEST_EFFORT (sensor QoS) — must request the matching profile to receive it
sframe="$(timeout 5 ros2 topic echo /scan --once --qos-profile sensor_data \
          --field header.frame_id 2>/dev/null | head -1)"
if [ -n "$sframe" ]; then ok "/scan publishing (frame: $sframe)"; else fail "/scan silent (sensor QoS / robot lidar down?)"; rc=1; fi

volt="$(timeout 4 ros2 topic echo /battery_state --once --field voltage 2>/dev/null | head -1)"
if [ -n "$volt" ]; then ok "/battery_state voltage = ${volt} V"; else warn "/battery_state silent (OK in some emulator configs)"; fi

exit $rc

#!/usr/bin/env bash
# PROOF 09 — the world-edit / keepout pipeline is wired (observe only):
# /keepout_mask + /costmap_filter_info published, /edits/state available.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; source "$HERE/_common.sh"
say "PROOF 09 — world-edit / keepout pipeline (observe)"
rc=0

if timeout 5 ros2 topic echo /keepout_mask --once --field info.width >/dev/null 2>&1; then
  ok "/keepout_mask published (the Nav2 keepout-filter source for both costmaps)"
else
  fail "/keepout_mask missing (map_edit waits for /map — is map_server up?)"; rc=1
fi

if timeout 4 ros2 topic echo /costmap_filter_info --once >/dev/null 2>&1; then
  ok "/costmap_filter_info present (filter wired into the costmaps)"
else
  warn "/costmap_filter_info silent"
fi

edits="$(timeout 4 ros2 topic echo /edits/state --once --field data 2>/dev/null | head -1)"
echo "   /edits/state: ${edits:-<none yet>}"
exit $rc

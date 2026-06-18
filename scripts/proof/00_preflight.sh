#!/usr/bin/env bash
# PROOF 00 — every link GO/NO-GO in ~6 s. Exit code = verdict (0 GO, !0 NO-GO).
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; source "$HERE/_common.sh"
say "PROOF 00 — every link GO/NO-GO (preflight)"
ros2 run algae_twin preflight

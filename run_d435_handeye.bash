#!/usr/bin/env bash
set -e
_d435_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${_d435_root}/activate_handover.bash"
cd -- "${_d435_root}"
if [[ "${1:-}" == "--solve-only" ]]; then
    shift
    exec python -m tools.calibration.calibrate_handeye_d435 "$@"
fi
exec python -m tools.calibration.capture_handeye_d435 --solve --samples 20 "$@"

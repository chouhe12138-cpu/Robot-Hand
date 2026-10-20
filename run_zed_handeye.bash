#!/usr/bin/env bash
set -e
_zed_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${_zed_root}/activate_handover.bash"
cd -- "${_zed_root}"
exec python -m tools.calibration.capture_handeye_zed --solve --samples 20 --tcp-offset 0 0 0.135 0 0 0 "$@"

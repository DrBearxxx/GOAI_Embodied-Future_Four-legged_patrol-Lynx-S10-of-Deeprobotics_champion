#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
python3 scripts/verify_goai_overlay.py
export S10_INDOOR_LOCALIZER=scripts/localize_mp.py
exec bash run_localizer.sh "$@"

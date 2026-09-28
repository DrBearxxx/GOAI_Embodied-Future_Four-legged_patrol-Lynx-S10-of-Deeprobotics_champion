#!/usr/bin/env bash
# No implicit --execute, ARM, stand, sensors, or acceptance overrides.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
exec bash ./run_route.sh --trial-50cm "$@"

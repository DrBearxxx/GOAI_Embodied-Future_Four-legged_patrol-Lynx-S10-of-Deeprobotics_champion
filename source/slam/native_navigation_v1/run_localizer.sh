#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
python3 -c 'from native_nav.bootstrap import verify; print(verify())'
exec bash /home/wym/s10_native_navigation_v1/run_robust_localizer.sh "$@"

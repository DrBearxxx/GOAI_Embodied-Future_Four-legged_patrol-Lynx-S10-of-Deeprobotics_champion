#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
python3 scripts/verify_goai_overlay.py
source /home/wym/s10_goai_ws/env.sh
export PYTHONPATH="/home/wym/s10_slam/venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
exec /usr/bin/python3 scripts/goai_route.py "$@"

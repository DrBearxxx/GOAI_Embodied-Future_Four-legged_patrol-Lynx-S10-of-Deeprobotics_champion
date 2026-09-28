#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /home/wym/s10_slam/env.sh
export ROS_DOMAIN_ID=88
export PYTHONPATH="/home/wym/s10_slam/venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
exec python3 scripts/supervise_localizer.py "$@"

#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
source /home/wym/s10_indoor_navigation_v1/sdk_ws/install/setup.bash
export PYTHONPATH="/home/wym/s10_slam/venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
exec /usr/bin/python3 scripts/route_ros.py "$@"

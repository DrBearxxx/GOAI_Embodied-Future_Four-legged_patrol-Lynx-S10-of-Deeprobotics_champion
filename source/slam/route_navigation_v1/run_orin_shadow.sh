#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /home/wym/s10_slam/env.sh
export ROS_DOMAIN_ID="${S10_ROUTE_DOMAIN:-88}"
# Use existing system ROS and the existing small_gicp virtualenv dependency.
export PYTHONPATH="/home/wym/s10_slam/venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
exec /usr/bin/python3 scripts/ros_shadow.py "$@"

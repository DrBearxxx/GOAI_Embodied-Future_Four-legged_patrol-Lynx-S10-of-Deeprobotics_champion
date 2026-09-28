#!/usr/bin/env bash
set -eo pipefail
cd /home/wym/s10_indoor_navigation_v1
source /home/wym/s10_slam/env.sh
export S10_INDOOR_PROFILE=1 S10_INDOOR_CHECK_SECONDS=20
/home/wym/s10_slam/venv/bin/python scripts/live_check.py

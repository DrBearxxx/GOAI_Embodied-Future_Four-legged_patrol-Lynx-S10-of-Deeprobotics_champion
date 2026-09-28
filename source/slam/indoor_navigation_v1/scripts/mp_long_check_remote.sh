#!/usr/bin/env bash
set -eo pipefail
cd /home/wym/s10_indoor_navigation_v1
source /home/wym/s10_slam/env.sh
export S10_INDOOR_LOCALIZER=scripts/localize_mp.py S10_INDOOR_CHECK_SECONDS=60
/home/wym/s10_slam/venv/bin/python scripts/live_check.py

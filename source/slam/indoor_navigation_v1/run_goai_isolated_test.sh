#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /home/wym/s10_goai_ws/env.sh
export PYTHONPATH="/home/wym/s10_slam/venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/wym/s10_goai_ws/src/s10_goai_deploy/config/loopback.xml
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
exec /usr/bin/python3 scripts/test_goai_isolated.py

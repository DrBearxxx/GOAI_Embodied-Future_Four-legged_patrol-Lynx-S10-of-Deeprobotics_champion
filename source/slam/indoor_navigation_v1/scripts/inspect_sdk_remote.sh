#!/usr/bin/env bash
set -eo pipefail
systemctl cat rl_deploy.service
ls -la /opt/apps
source /home/wym/s10_blind_ws/install/s10_blind_deploy/lib/s10_blind_deploy/env.sh
/usr/bin/python3 /home/wym/indoor_nav_inspect_20260913.py

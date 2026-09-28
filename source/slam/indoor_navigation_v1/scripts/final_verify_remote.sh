#!/usr/bin/env bash
set -eo pipefail
cd /home/wym/s10_indoor_navigation_v1
tar -xzf /home/wym/indoor_navigation_v1_payload.tar.gz
/usr/bin/python3 scripts/verify_release.py
/usr/bin/python3 -m unittest discover -s tests -v
bash run_isolated_test.sh
source /home/wym/s10_slam/env.sh
/home/wym/s10_slam/venv/bin/python /home/wym/s10_slam/python/control.py status
systemctl is-active rl_deploy.service

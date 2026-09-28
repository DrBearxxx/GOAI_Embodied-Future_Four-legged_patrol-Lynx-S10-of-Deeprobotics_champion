#!/usr/bin/env bash
set -eo pipefail
test "$(readlink -f /home/wym/s10_route_navigation_v1)" = /home/wym/s10_route_navigation_v1
test -f /home/wym/s10_route_navigation_v1/assets/manifest.json
tar -xzf /home/wym/orin_shadow_code_v5.tar.gz -C /home/wym/s10_route_navigation_v1
cd /home/wym/s10_route_navigation_v1
source /home/wym/s10_slam/env.sh
export PYTHONPATH="/home/wym/s10_slam/venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=189
/usr/bin/python3 -m unittest discover -s tests -v
/usr/bin/python3 scripts/ros_isolated_test.py

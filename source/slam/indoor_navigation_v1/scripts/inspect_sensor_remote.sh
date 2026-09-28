#!/usr/bin/env bash
set -eo pipefail
ls /home/wym/s10_slam/bin
source /home/wym/s10_slam/env.sh
/home/wym/s10_slam/venv/bin/python /home/wym/s10_slam/python/control.py status
sha256sum /home/wym/s10_slam/python/control.py
cat /home/wym/s10_slam/env.sh

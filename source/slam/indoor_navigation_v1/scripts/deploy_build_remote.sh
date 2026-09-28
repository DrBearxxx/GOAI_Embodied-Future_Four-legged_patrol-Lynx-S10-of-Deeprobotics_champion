#!/usr/bin/env bash
set -eo pipefail
cd /home/wym/s10_indoor_navigation_v1
tar -xzf /home/wym/indoor_navigation_v1_payload.tar.gz
bash build_sdk.sh

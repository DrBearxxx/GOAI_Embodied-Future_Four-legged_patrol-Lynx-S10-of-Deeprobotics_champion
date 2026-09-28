#!/usr/bin/env bash
set -eo pipefail
test "$(readlink -f /home/wym/s10_route_navigation_v1)" = /home/wym/s10_route_navigation_v1
test -f /home/wym/s10_route_navigation_v1/assets/manifest.json
tar -xzf /home/wym/orin_route_shadow_final_v1.tar.gz -C /home/wym/s10_route_navigation_v1
/usr/bin/python3 /home/wym/s10_route_navigation_v1/scripts/verify_delivery.py

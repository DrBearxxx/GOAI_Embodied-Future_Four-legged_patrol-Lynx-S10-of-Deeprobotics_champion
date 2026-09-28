#!/usr/bin/env bash
# Transport/subscription smoke test only. OFFLINE means no robot UDP heartbeat.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
python3 -m native_nav.gateway --port 18892 &
gateway_pid=$!
cleanup() { kill -TERM "$gateway_pid" 2>/dev/null || true; wait "$gateway_pid" 2>/dev/null || true; }
trap cleanup EXIT
sleep 0.5
python3 scripts/status_client.py --port 18892 --seconds 1
bash run_route.sh --port 18892 --seconds 4

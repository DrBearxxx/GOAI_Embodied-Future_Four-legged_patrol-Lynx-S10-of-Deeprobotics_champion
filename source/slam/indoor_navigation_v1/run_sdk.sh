#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
# Preserve the verified robot Ethernet discovery profile, then overlay ONLY
# this process's high-level message package. Existing controllers are unchanged.
source /home/wym/s10_blind_ws/install/s10_blind_deploy/lib/s10_blind_deploy/env.sh
source sdk_ws/install/local_setup.bash
exec /usr/bin/python3 scripts/sdk_gate.py "$@"

#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
# Existing five-mode commissioning contract is retained for hardware activation.
if [[ "${S10_G12_COMMISSIONING:-}" != YES ]]; then
  echo 'Set S10_G12_COMMISSIONING=YES only for on-site supervised hardware commissioning.' >&2
  exit 2
fi
if [[ "${1:-}" != --worker ]] && tmux has-session -t goai-nav-gateway 2>/dev/null; then
  echo 'Stop the monitor gateway explicitly before hardware activation. Navigation must be paused.' >&2
  exit 2
fi
if [[ "${1:-}" != --worker ]]; then
  mkdir -p logs private
  tmux new-session -d -s goai-nav-gateway "env S10_G12_COMMISSIONING=YES bash '$PWD/start_control.sh' --worker >> '$PWD/logs/gateway.log' 2>&1"
  echo 'Hardware gateway starting idle. Check control_status.py before operating.'
  exit 0
fi
source /home/wym/s10_waypoint_blind_ws/install/s10_waypoint_deploy/lib/s10_waypoint_deploy/env.sh
source /home/wym/s10_indoor_navigation_v1/sdk_ws/install/setup.bash
source /home/wym/s10_g12_damping_ws/install/setup.bash
export FASTDDS_DEFAULT_PROFILES_FILE="$PWD/backend/fastdds_robot.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_DEFAULT_PROFILES_FILE"
set -eo pipefail
exec python3 backend/server.py --bind 127.0.0.1 --port 18895 --key-file private/pairing.key --execute --ack-physical-safety --plaintext-config-verified

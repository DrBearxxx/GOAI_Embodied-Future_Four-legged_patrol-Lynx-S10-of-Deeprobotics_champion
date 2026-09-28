#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
case "${1:-}" in
  body)
    exec ssh -4 -tt -o StrictHostKeyChecking=yes -o UserKnownHostsFile="$PWD/body_known_hosts" \
      -o ExitOnForwardFailure=yes -o ServerAliveInterval=2 -o ServerAliveCountMax=2 \
      -L 127.0.0.1:18891:127.0.0.1:18891 user@10.21.33.103 \
      'cd /home/user/s10_native_navigation_v1 && python3 scripts/verify_delivery.py && exec bash run_gateway.sh --first-trial'
    ;;
  localizer)
    source /home/wym/s10_slam/env.sh
    s10slam sensors-start
    s10slam camera-start
    exec bash ./run_localizer.sh
    ;;
  route)
    export S10_FIRST_TRIAL_SESSION=s10-first-trial
    # Do not start the presence-checked operator loop before attach completes.
    for ((i=0; i<60; i++)); do
      attached="$(tmux display-message -p -t "$S10_FIRST_TRIAL_SESSION" '#{session_attached}')"
      if [[ "$attached" =~ ^[1-9][0-9]*$ ]]; then
        exec bash ./run_trial_50cm.sh --first-trial --allow-start-offset --execute
      fi
      sleep 0.5
    done
    echo 'No operator attached; route was not started.'
    exit 1
    ;;
  *) echo 'Internal pane: body | localizer | route'; exit 2 ;;
esac

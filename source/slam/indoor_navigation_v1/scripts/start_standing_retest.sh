#!/bin/bash
set -euo pipefail
tmux has-session -t wym-goai-standtest
if tmux list-windows -t wym-goai-standtest -F '#{window_name}' | grep -qx retest; then
  echo 'Retest window already exists; inspect it rather than starting twice'; exit 2
fi
tmux new-window -d -t wym-goai-standtest -n retest "bash -c 'source /home/wym/s10_goai_ws/env.sh && exec /usr/bin/python3 /home/wym/s10_indoor_navigation_v1/scripts/standing_retest.py'"
tmux set-option -w -t wym-goai-standtest:retest remain-on-exit on
tmux capture-pane -p -t wym-goai-standtest:retest -S -8

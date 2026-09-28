#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
trial_root="$PWD"
if tmux has-session -t wym-indoor-nav 2>/dev/null; then
  echo 'Existing monitor session; attaching without starting duplicate processes.'
else
  tmux new-session -d -s wym-indoor-nav -n monitor -c "$trial_root" 'bash run_localizer.sh'
  tmux split-window -h -t wym-indoor-nav:monitor -c "$trial_root" 'bash run_sdk.sh --seconds 0'
  tmux select-pane -t wym-indoor-nav:monitor.0
fi
echo 'MONITOR ONLY: no SDK command publisher. Sensors must be started separately.'
exec tmux attach-session -t wym-indoor-nav

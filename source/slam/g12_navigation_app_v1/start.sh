#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
mkdir -p private logs
backend_odom="$(python3 -c 'import json,pathlib; p=pathlib.Path("odometry.json"); print(json.loads(p.read_text()).get("backend","gicp") if p.exists() else "gicp")')"
if [[ "$backend_odom" == lightning ]]; then
  /home/wym/s10_slam/bin/s10slam sensors-start
  python3 lightning/service.py start
fi
if [[ ! -s private/pairing.key ]]; then
  python3 backend/server.py --key-file private/pairing.key --initialize-key
fi
if ! tmux has-session -t '=goai-nav-gateway' 2>/dev/null; then
  backend=monitor
  if [[ -f private/control_backend ]]; then read -r backend < private/control_backend; fi
  case "$backend" in
    hardware) S10_G12_COMMISSIONING=YES bash ./start_control.sh ;;
    monitor) tmux new-session -d -s goai-nav-gateway "python3 '$PWD/backend/server.py' --bind 127.0.0.1 --port 18895 --key-file '$PWD/private/pairing.key' >> '$PWD/logs/gateway.log' 2>&1" ;;
    *) echo 'Invalid saved control backend; no gateway started.' >&2; exit 2 ;;
  esac
fi
if ! tmux has-session -t '=goai-navigation' 2>/dev/null; then
  /home/wym/s10_slam/bin/s10slam sensors-start
  tmux new-session -d -s goai-navigation "exec bash '$PWD/run.sh' --port 18894 --gateway-url http://127.0.0.1:18895 >> '$PWD/logs/runtime.log' 2>&1"
fi
echo "GOAI navigation: port 18894; odometry=$backend_odom. 在 G12 操作重定位、导航或人工接管。"

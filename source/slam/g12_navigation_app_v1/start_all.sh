#!/usr/bin/env bash
# Start the deployed navigation stack; repeat invocations reuse running services.
set -eo pipefail
APP=/home/wym/s10_navigation_app_v1
SLAM=/home/wym/s10_slam
cd "$APP"
mkdir -p logs private
exec 9>logs/start-all.lock
flock -n 9 || { echo '另一条一键启动命令正在运行。' >&2; exit 1; }
trap 'echo "启动未完成；请查看 /home/wym/s10_navigation_app_v1/logs/start-all.log" >&2' ERR
exec > >(tee -a logs/start-all.log) 2>&1
printf '\n[%s] 启动 GOAI 导航服务\n' "$(date -Is)"

# Restore only the verified G12 host route, which can be lost after power-off.
g12_ip="${GOAI_G12_IP:-10.21.41.12}"
python3 - "$g12_ip" <<'PY'
import ipaddress,sys
ipaddress.IPv4Address(sys.argv[1])
PY
route="$(ip -4 route get "$g12_ip" 2>/dev/null || true)"
if [[ "$route" != *'via 10.21.33.103 dev end0'* ]]; then
  echo "恢复 G12 回程路由：$g12_ip，经 10.21.33.103 / end0"
  sudo ip route replace "$g12_ip/32" via 10.21.33.103 dev end0
fi

cold_navigation=0
if ! tmux has-session -t '=goai-navigation' 2>/dev/null; then cold_navigation=1; fi

echo '[1/3] 前后雷达和 IMU 接收'
"$SLAM/bin/s10slam" sensors-start
echo '[2/3] 运控网关、里程计和导航服务'
bash "$APP/start.sh"
echo '[3/3] 检查服务响应'
if [[ "$cold_navigation" == 1 ]]; then
  python3 "$APP/startup_status.py" --select-outdoor
else
  python3 "$APP/startup_status.py"
fi
echo 'G12 App 地址：http://10.21.33.102:18894'
echo '服务已就绪。重定位、起立、导航和人工控制在 G12 App 中操作。'

#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source /home/wym/s10_slam/env.sh
LIGHTNING_RUNTIME="${GOAI_LIGHTNING_RUNTIME:-/home/wym/s10_lightning_runtime_v1}"
source "$LIGHTNING_RUNTIME/install/setup.bash"
export PYTHONPATH="/home/wym/s10_slam/venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
export OMP_NUM_THREADS=4 OMP_WAIT_POLICY=PASSIVE OPENBLAS_NUM_THREADS=1
exec python3 lightning/service.py run --runtime "$LIGHTNING_RUNTIME"

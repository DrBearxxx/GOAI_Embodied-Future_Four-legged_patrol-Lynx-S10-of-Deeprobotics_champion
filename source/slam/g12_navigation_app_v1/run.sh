#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /home/wym/s10_slam/env.sh
export PYTHONPATH="/home/wym/s10_slam/venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
exec python3 runtime.py "$@"

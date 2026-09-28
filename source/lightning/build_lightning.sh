#!/usr/bin/env bash
set -eo pipefail
PACKAGE_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
RUNTIME_ROOT=${GOAI_LIGHTNING_RUNTIME:-/home/wym/s10_lightning_runtime_v1}
python3 "$PACKAGE_ROOT/stage_runtime.py" "$RUNTIME_ROOT"
cd "$RUNTIME_ROOT"
source /opt/ros/jazzy/setup.bash
export OMP_NUM_THREADS=4 OMP_WAIT_POLICY=PASSIVE OPENBLAS_NUM_THREADS=1
export CMAKE_BUILD_PARALLEL_LEVEL=4 MAKEFLAGS=-j4
colcon build --base-paths packages --executor sequential --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF \
  --event-handlers console_direct+
test -x install/lightning_lio_validation/lib/lightning_lio_validation/lightning_lio
python3 - <<'PY'
import hashlib,json,platform
from pathlib import Path
binary=Path('install/lightning_lio_validation/lib/lightning_lio_validation/lightning_lio')
Path('build-provenance.json').write_text(json.dumps(dict(machine=platform.machine(),
    binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
    source_manifest_sha256=hashlib.sha256(Path('goai-runtime-source.json').read_bytes()).hexdigest()),indent=2))
PY
echo 'Lightning 已在独立目录编译完成；导航与运控服务尚未更改。'

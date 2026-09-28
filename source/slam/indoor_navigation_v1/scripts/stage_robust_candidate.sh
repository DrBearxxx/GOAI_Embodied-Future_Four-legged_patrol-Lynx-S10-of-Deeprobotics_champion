#!/usr/bin/env bash
set -euo pipefail
candidate=/home/wym/s10_indoor_robust_candidate_20260914
production=/home/wym/s10_indoor_navigation_v1
test ! -e "$candidate"
test -f "$production/goai_overlay_manifest.json"
sha256sum /home/wym/goai_robust_v2_20260914.tar.gz
mkdir -- "$candidate"
cp -a -- "$production/assets" "$production/indoor" "$production/scripts" "$production/tests" "$candidate/"
cp -a -- "$production/config.json" "$production/dependency_lock.json" "$production/run_localizer.sh" "$candidate/"
tar -xzf /home/wym/goai_robust_v2_20260914.tar.gz -C "$candidate"
cd -- "$candidate"
python3 scripts/verify_goai_overlay.py
PYTHONPATH=/home/wym/s10_slam/venv/lib/python3.12/site-packages OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -q
bash run_goai_isolated_test.sh

#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source ../slam_local/env.sh
exec /usr/bin/python3 "$@"

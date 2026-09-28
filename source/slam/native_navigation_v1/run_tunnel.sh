#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
exec ssh -4 -N -o StrictHostKeyChecking=yes -o UserKnownHostsFile="$PWD/body_known_hosts" \
  -o ExitOnForwardFailure=yes -o ServerAliveInterval=2 -o ServerAliveCountMax=2 \
  -L 127.0.0.1:18891:127.0.0.1:18891 user@10.21.33.103

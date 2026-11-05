#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export DEBIAN_FRONTEND=noninteractive
"${SCRIPT_DIR}/setup_xgc2_apt_source.sh"
apt-get install -y --no-install-recommends libxgc2-state-machine-dev
installed="$(dpkg-query -W -f='${Version}' libxgc2-state-machine-dev)"
echo "Build dependency: libxgc2-state-machine-dev ${installed} (minimum 0.2.0-4~)"
dpkg --compare-versions "${installed}" ge '0.2.0-4~' || {
  echo "ROS1 utils requires published SDK >= 0.2.0-4~; installed ${installed}" >&2
  exit 1
}

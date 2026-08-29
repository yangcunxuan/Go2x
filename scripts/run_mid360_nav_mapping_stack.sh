#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p runtime/logs patrol_data/maps
docker-compose run --rm ros2 ./scripts/inside_mid360_mapping.sh

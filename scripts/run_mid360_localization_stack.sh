#!/usr/bin/env bash
# Host entry point for the localization-mode perception stack. The web's
# navigation start flow calls this before launching the planner.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p runtime/logs
docker-compose run --rm ros2 ./scripts/inside_mid360_localization.sh \
  > runtime/logs/mid360_localization.log 2>&1

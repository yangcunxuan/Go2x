#!/usr/bin/env bash
# Host entry point for the CMU-style planner stack. Mirrors
# run_navigation_stack.sh: the sensor domain is used because all inputs come
# from FAST-LIO (MID360), not from the GO2 domain.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p runtime/logs

docker-compose run --rm \
  -e PLANNER_MODE="${PLANNER_MODE:-full}" \
  ros2 ./scripts/inside_planner.sh \
  > runtime/logs/planner_stack.log 2>&1

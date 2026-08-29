#!/usr/bin/env bash
# Host wrapper for the navigation service (dual-mode era):
#   1. ensure localization-mode perception (FAST-LIO odometry + localizer)
#   2. wait for odometry to come online
#   3. run the CMU planner stack in the foreground
# Killing this wrapper tears down both children (trap below).
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p runtime/logs
PIDS=(); cleanup(){ kill "${PIDS[@]}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

docker-compose run --rm ros2 ./scripts/inside_mid360_localization.sh \
  >> runtime/logs/mid360_localization.log 2>&1 & PIDS+=($!)
LOCALIZATION_PID=${PIDS[0]}

python3 - "$LOCALIZATION_PID" <<'PYEOF'
import json, os, sys, time
pid = int(sys.argv[1])
deadline = time.time() + 90
while time.time() < deadline:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        print('MID360定位栈在里程计就绪前退出', file=sys.stderr)
        sys.exit(5)
    try:
        d = json.load(open('runtime/robot_state.json'))
        if time.time() - float(d['updated_at']) < 2 and d.get('odom_online'):
            print('odometry online')
            sys.exit(0)
    except (OSError, ValueError, TypeError):
        pass
    time.sleep(1)
print('等待里程计超时', file=sys.stderr)
sys.exit(4)
PYEOF

bash scripts/run_planner_stack.sh & PIDS+=($!)

# Localization and planning are one service: either child exiting tears down
# the other, so the web can never report navigation as running with a dead
# localizer or motion bridge.
set +e
wait -n "${PIDS[@]}"; code=$?
set -e
exit "$code"

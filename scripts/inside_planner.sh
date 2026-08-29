#!/usr/bin/env bash
# CMU-style planner stack (terrain analysis / FAR planner / local planner)
# running inside the go2-mid360 container. Replaces the Nav2 pipeline for
# indoor+outdoor patrol navigation.
#
# PLANNER_MODE=sensing (default): terrain analysis only, first verification.
# PLANNER_MODE=full: adds far_planner + local_planner, plus the two bridges
# that make the stack usable end to end:
#   - planner_motion_bridge: /cmd_vel -> GO2 sport commands (pulse cadence)
#   - goal_relay:            web goal.json (map_level) -> /goal_point (camera_init)
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
source /project/ros2_ws/install/setup.bash
set -u

export AMENT_PREFIX_PATH="/project/ros2_ws/install/patrol_bridge:${AMENT_PREFIX_PATH:-}"
MODE="${PLANNER_MODE:-sensing}"

TOPICS=$(ros2 topic list)
if ! grep -Fxq /Odometry <<<"$TOPICS" || ! grep -Fxq /cloud_registered_body <<<"$TOPICS"; then
  echo 'FAST-LIO 尚未运行：planner stack 必须保持 MID360 三维建图/定位服务在线。' >&2
  exit 3
fi

echo "启动 planner stack（mode=${MODE}）"
ros2 launch /project/config/planner_stack.launch.py mode:=${MODE} &
LAUNCH_PID=$!

# far_planner silently drops every /goal_point until its /start_far_planner
# Trigger service is called (is_system_started_). Call it automatically so
# a container restart never leaves planning disabled.
if [ "${MODE}" = "full" ]; then
  python3 /project/scripts/start_far_service.py || echo '警告：far_planner 启动服务调用失败，目标点将被忽略' >&2
  python3 /project/scripts/planner_motion_bridge.py >> /project/runtime/logs/motion_bridge.log 2>&1 &
  python3 /project/scripts/goal_relay.py >> /project/runtime/logs/goal_relay.log 2>&1 &
fi

wait $LAUNCH_PID

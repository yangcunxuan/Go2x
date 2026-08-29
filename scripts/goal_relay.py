#!/usr/bin/env python3
"""Consume runtime/goal.json (map_level) and publish /goal_point (camera_init).

The web server writes navigation goals in the map_level frame; far_planner
consumes /goal_point in the FAST-LIO camera_init frame. This relay bridges
the two with the same fixed level transform patrol_bridge applies to clouds:
roll/pitch are the MID360 mount angles (constant), x/y/z/yaw come from
runtime/localization_alignment.json.

Runs inside the planner container (started by inside_planner.sh in full
mode). Publishes continuously at 2 Hz so a far_planner restart inside the
motion enable window still receives the goal.
"""
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

RUN = Path('/project/runtime')
LEVEL_ROLL = -0.030788
LEVEL_PITCH = 0.621767


def level_transform():
    alignment = {}
    try:
        alignment = json.loads((RUN / 'localization_alignment.json').read_text(encoding='utf-8'))
    except (OSError, ValueError):
        pass
    yaw = float(alignment.get('yaw', 0))
    cr, sr = math.cos(LEVEL_ROLL), math.sin(LEVEL_ROLL)
    cp, sp = math.cos(LEVEL_PITCH), math.sin(LEVEL_PITCH)
    base = np.array([[cp, 0.0, sp],
                     [sr * sp, cr, -sr * cp],
                     [-cr * sp, sr, cr * cp]], dtype=np.float32)
    cy, sy = math.cos(yaw), math.sin(yaw)
    yrot = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)
    rot = yrot @ base
    trans = np.array([float(alignment.get('x', 0)), float(alignment.get('y', 0)),
                      float(alignment.get('z', 0))], dtype=np.float32)
    return rot, trans


def main():
    rot, trans = level_transform()
    rclpy.init()
    node = Node('goal_relay')
    pub = node.create_publisher(PointStamped, '/goal_point', 5)
    last_id = None
    goal = None
    last_publish = 0.0
    stale_since = None
    try:
        last_id = json.loads((RUN / 'goal.json').read_text(encoding='utf-8')).get('id')
    except (OSError, ValueError):
        pass
    node.get_logger().info('goal relay ready (map_level -> camera_init)')
    while True:
        try:
            fresh = json.loads((RUN / 'goal.json').read_text(encoding='utf-8'))
        except (OSError, ValueError):
            fresh = None
        if fresh and fresh.get('id') and fresh['id'] != last_id:
            last_id = fresh['id']
            p = np.array([float(fresh['x']), float(fresh['y']),
                          float(fresh.get('z', 0))], dtype=np.float32)
            c = (p - trans) @ rot
            goal = {'id': fresh['id'], 'x': float(c[0]), 'y': float(c[1]),
                    'z': float(c[2]), 'expires_at': float(fresh.get('expires_at', 0))}
            node.get_logger().info(
                f"goal {fresh['id'][:8]} map_level=({p[0]:.2f},{p[1]:.2f},{p[2]:.2f}) "
                f"-> camera_init=({c[0]:.2f},{c[1]:.2f},{c[2]:.2f})")
        # Goal lifecycle: stop republishing once the goal expires or the file
        # disappears, so a stale goal can never keep planning after a failure
        # or a web restart.
        if goal and not fresh:
            if stale_since is None:
                stale_since = time.monotonic()
            elif time.monotonic() - stale_since > 5.0:
                node.get_logger().info('goal.json removed; goal released')
                goal = None
                stale_since = None
        elif goal and goal.get('expires_at') and time.time() > goal['expires_at']:
            node.get_logger().info('goal expired; released')
            goal = None
            stale_since = None
        else:
            stale_since = None
        if goal and time.monotonic() - last_publish >= 0.5:
            last_publish = time.monotonic()
            m = PointStamped()
            m.header.frame_id = 'camera_init'
            m.header.stamp = node.get_clock().now().to_msg()
            m.point.x, m.point.y, m.point.z = goal['x'], goal['y'], goal['z']
            pub.publish(m)
        time.sleep(0.1)


if __name__ == '__main__':
    main()

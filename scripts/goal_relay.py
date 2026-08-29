#!/usr/bin/env python3
"""Consume runtime/goal.json (map_level) and publish /goal_point (camera_init).

The web server writes navigation goals in the map_level frame; far_planner
consumes /goal_point in the FAST-LIO camera_init frame.

Transform source, in priority order:
  1. Live TF lookup map_level <- camera_init, published by the global
     localization manager (Plan A). Queried on EVERY publish so ongoing
     relocalization corrections keep the goal aligned.
  2. Fallback: the static transform from localization_alignment.json
     (fixed mount roll/pitch + session offset), used only while no TF is
     available.

Goal lifecycle: publishing stops when the goal expires (expires_at) or when
goal.json disappears, so a stale goal can never keep planning.
"""
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.time import Time

RUN = Path('/project/runtime')
LEVEL_ROLL = -0.030788
LEVEL_PITCH = 0.621767


def static_transform():
    """Fallback: fixed mount angles + session offset from the alignment file."""
    try:
        alignment = json.loads((RUN / 'localization_alignment.json').read_text(
            encoding='utf-8'))
    except (OSError, ValueError):
        alignment = {}
    yaw = float(alignment.get('yaw', 0))
    cr, sr = math.cos(LEVEL_ROLL), math.sin(LEVEL_ROLL)
    cp, sp = math.cos(LEVEL_PITCH), math.sin(LEVEL_PITCH)
    base = np.array([[cp, 0.0, sp],
                     [sr * sp, cr, -sr * cp],
                     [-cr * sp, sr, cr * cp]], dtype=np.float64)
    cy, sy = math.cos(yaw), math.sin(yaw)
    yrot = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    rot = yrot @ base
    trans = np.array([float(alignment.get('x', 0)), float(alignment.get('y', 0)),
                      float(alignment.get('z', 0))])
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = rot, trans
    return matrix


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def main():
    rclpy.init()
    node = Node('goal_relay')
    pub = node.create_publisher(PointStamped, '/goal_point', 5)
    tf_buffer = None
    try:
        from tf2_ros import Buffer, TransformListener
        tf_buffer = Buffer()
        TransformListener(tf_buffer, node)
    except Exception as exc:  # tf2 missing should never happen, be safe
        node.get_logger().warning(f'TF 不可用，退回静态变换: {exc}')
    fallback = static_transform()
    last_id = None
    goal = None        # raw map_level coordinates, converted on every publish
    last_publish = 0.0
    stale_since = None
    transform_logged = [False]
    try:
        last_id = json.loads((RUN / 'goal.json').read_text(encoding='utf-8')).get('id')
    except (OSError, ValueError):
        pass
    node.get_logger().info('goal relay ready (map_level -> camera_init, TF 优先)')

    def current_matrix():
        """Latest map_level<-camera_init as 4x4; live TF first, static fallback."""
        if tf_buffer is not None:
            try:
                m = tf_buffer.lookup_transform('map_level', 'camera_init', Time())
                tr = m.transform
                matrix = np.eye(4)
                matrix[:3, :3] = quat_to_matrix(tr.rotation)
                matrix[:3, 3] = [tr.translation.x, tr.translation.y,
                                 tr.translation.z]
                return matrix, True
            except Exception:
                pass
        return fallback, False

    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            fresh = json.loads((RUN / 'goal.json').read_text(encoding='utf-8'))
        except (OSError, ValueError):
            fresh = None
        if fresh and fresh.get('id') and fresh['id'] != last_id:
            last_id = fresh['id']
            goal = {'id': fresh['id'], 'x': float(fresh['x']),
                    'y': float(fresh['y']), 'z': float(fresh.get('z', 0)),
                    'expires_at': float(fresh.get('expires_at', 0))}
            node.get_logger().info(
                f"goal {fresh['id'][:8]} map_level=({goal['x']:.2f},"
                f"{goal['y']:.2f},{goal['z']:.2f})")
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
        now = time.monotonic()
        if goal and now - last_publish >= 0.5:
            last_publish = now
            matrix, live = current_matrix()
            p = np.array([goal['x'], goal['y'], goal['z'], 1.0])
            c = np.linalg.inv(matrix) @ p
            m = PointStamped()
            m.header.frame_id = 'camera_init'
            m.header.stamp = node.get_clock().now().to_msg()
            m.point.x, m.point.y, m.point.z = float(c[0]), float(c[1]), float(c[2])
            pub.publish(m)
            if not transform_logged[0]:
                transform_logged[0] = True
                node.get_logger().info(
                    f"camera_init=({c[0]:.2f},{c[1]:.2f},{c[2]:.2f}) "
                    f"transform={'TF动态' if live else '静态兜底'}")
        time.sleep(0.05)


if __name__ == '__main__':
    main()

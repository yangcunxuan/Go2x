#!/usr/bin/env python3
"""Record the mapping trajectory for the global-localization database.

Subscribes /Odometry (camera_init) during a mapping session and writes a
keyframe pose every ~1 m of travel or ~10 deg of rotation to
runtime/trajectory_<session_id>.json. Only poses are recorded live — the
keyframe clouds themselves are cropped from the saved PCD afterwards by
build_map_db.py, so mapping sessions stay cheap on small machines.
"""
import json
import math
import os
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

RUNTIME = Path(os.environ.get('PATROL_RUNTIME', '/project/runtime'))
DIST_STEP = 1.0
YAW_STEP = math.radians(10.0)


def quat_yaw(q):
    return math.atan2(2.0 * (q.z * q.w + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class KeyframeSaver(Node):
    def __init__(self):
        super().__init__('keyframe_saver')
        self.session_id = os.environ.get('MAPPING_SESSION_ID', 'unknown')
        self.out = RUNTIME / f'trajectory_{self.session_id}.json'
        self.keyframes = []
        self.last = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, qos)
        self.create_timer(2.0, self.flush)
        self.get_logger().info(f'keyframe saver ready, session={self.session_id}')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = quat_yaw(msg.pose.pose.orientation)
        now = time.time()
        if self.last is not None:
            moved = math.hypot(p.x - self.last['x'], p.y - self.last['y'])
            turned = abs(math.atan2(math.sin(yaw - self.last['yaw']),
                                    math.cos(yaw - self.last['yaw'])))
            if moved < DIST_STEP and turned < YAW_STEP:
                return
        self.last = {'x': float(p.x), 'y': float(p.y), 'z': float(p.z), 'yaw': float(yaw)}
        self.keyframes.append({**self.last, 't': now})

    def flush(self):
        if not self.keyframes:
            return
        tmp = Path(str(self.out) + '.tmp')
        tmp.write_text(json.dumps({'session_id': self.session_id,
                                   'keyframes': self.keyframes}), encoding='utf-8')
        os.replace(tmp, self.out)


def main():
    rclpy.init()
    node = KeyframeSaver()
    try:
        rclpy.spin(node)
    finally:
        node.flush()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

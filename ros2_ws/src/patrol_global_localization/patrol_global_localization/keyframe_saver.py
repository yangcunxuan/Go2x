#!/usr/bin/env python3
"""Record the mapping trajectory for the global-localization database.

Subscribes /Odometry (camera_init) during a mapping session and writes a
keyframe every ~1 m of travel or ~10 deg of rotation. Keyframe poses are
stored as FULL SE(3) in the map_level frame, obtained by chaining the
mapping-mode static TF:

    T(mapLevel <- sensor) = T(mapLevel <- cameraInit) x T(cameraInit <- sensor)

Output: runtime/trajectory_<session_id>.json
    {'session_id', 'frame': 'map_level',
     'keyframes': [{x, y, z, qx, qy, qz, qw, t}, ...]}

Only poses are recorded live — keyframe clouds are cropped from the saved
PCD/NPY afterwards by build_map_db, so mapping sessions stay cheap on
small machines.
"""
import json
import math
import os
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

RUNTIME = Path(os.environ.get('PATROL_RUNTIME', '/project/runtime'))
DIST_STEP = 1.0
YAW_STEP = math.radians(10.0)


def quat_from_matrix(r):
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
                (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(r)))
    if i == 0:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        return (0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s,
                (r[2, 1] - r[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        return ((r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s,
                (r[0, 2] - r[2, 0]) / s)
    s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
    return ((r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s,
            (r[1, 0] - r[0, 1]) / s)


def quat_matrix_from_msg(q):
    n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) or 1.0
    qx, qy, qz, qw = q.x / n, q.y / n, q.z / n, q.w / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


class KeyframeSaver(Node):
    def __init__(self):
        super().__init__('keyframe_saver')
        self.session_id = os.environ.get('MAPPING_SESSION_ID', 'unknown')
        self.out = RUNTIME / f'trajectory_{self.session_id}.json'
        self.keyframes = []
        self.last_trigger = None
        # Mapping mode publishes the static map_level <- camera_init TF;
        # keep the listener reference alive or TF callbacks are garbage-collected.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, qos)
        self.create_timer(2.0, self.flush)
        self.get_logger().info(f'keyframe saver ready, session={self.session_id}')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = math.atan2(2.0 * (msg.pose.pose.orientation.z * msg.pose.pose.orientation.w
                                + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
                         1.0 - 2.0 * (msg.pose.pose.orientation.y ** 2
                                      + msg.pose.pose.orientation.z ** 2))
        if self.last_trigger is not None:
            moved = math.hypot(p.x - self.last_trigger['x'], p.y - self.last_trigger['y'])
            turned = abs(math.atan2(math.sin(yaw - self.last_trigger['yaw']),
                                    math.cos(yaw - self.last_trigger['yaw'])))
            if moved < DIST_STEP and turned < YAW_STEP:
                return
        # Capture the full SE(3) keyframe: chain the static map TF.
        try:
            tf = self.tf_buffer.lookup_transform('map_level', 'camera_init', Time())
        except Exception:
            self.get_logger().warning('map_level<-camera_init TF 尚不可用，丢弃关键帧',
                                      throttle_duration_sec=10.0)
            return
        tr = tf.transform
        t_map_cam = np.eye(4)
        t_map_cam[:3, :3] = quat_matrix_from_msg(tr.rotation)
        t_map_cam[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
        t_cam_sensor = np.eye(4)
        t_cam_sensor[:3, :3] = quat_matrix_from_msg(msg.pose.pose.orientation)
        t_cam_sensor[:3, 3] = [p.x, p.y, p.z]
        t_map_sensor = t_map_cam @ t_cam_sensor
        r = t_map_sensor[:3, :3]
        qx, qy, qz, qw = quat_from_matrix(r)
        self.last_trigger = {'x': float(p.x), 'y': float(p.y), 'yaw': float(yaw)}
        self.keyframes.append({
            'x': float(t_map_sensor[0, 3]), 'y': float(t_map_sensor[1, 3]),
            'z': float(t_map_sensor[2, 3]),
            'qx': float(qx), 'qy': float(qy), 'qz': float(qz), 'qw': float(qw),
            't': time.time(),
        })

    def flush(self):
        if not self.keyframes:
            return
        payload = {'session_id': self.session_id, 'frame': 'map_level',
                   'keyframes': self.keyframes}
        tmp = Path(str(self.out) + '.tmp')
        tmp.write_text(json.dumps(payload), encoding='utf-8')
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

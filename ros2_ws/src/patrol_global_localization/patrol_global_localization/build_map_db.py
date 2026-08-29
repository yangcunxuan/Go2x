#!/usr/bin/env python3
"""Build the Scan Context database for a saved map package.

Usage (inside the container):
    build_map_db <map.npy> <trajectory.json>

Map package layout (all coordinates in map_level):
    patrol_data/maps/<map_id>/
      map.npy          (N,3) float32, loaded with mmap
      trajectory.json  {'keyframes': [{x,y,z,qx,qy,qz,qw}, ...]}
      db.npz           poses(N,8: x y z qx qy qz qw + padding), descriptors(K,D)

Every descriptor is computed in the SENSOR frame of its keyframe:
    points_local = inverse(T_mapLevel_sensor) @ points_mapLevel
so the same place yields the same descriptor regardless of where in the
global map it sits (P0 audit #6).
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from patrol_global_localization.scan_context import (  # noqa: E402
    NUM_RINGS, NUM_SECTORS, make_descriptor_height)

VOXEL = 0.15
CROP_RADIUS = 20.0


def quat_matrix(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def pose_matrix(kf):
    x, y, z = float(kf['x']), float(kf['y']), float(kf['z'])
    if 'qw' in kf:
        r = quat_matrix(float(kf['qx']), float(kf['qy']),
                        float(kf['qz']), float(kf['qw']))
    else:
        yaw = float(kf.get('yaw', 0))
        c, s = math.cos(yaw), math.sin(yaw)
        r = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = r, [x, y, z]
    return t


def main():
    if len(sys.argv) < 3:
        print('usage: build_map_db <map.npy> <trajectory.json>', file=sys.stderr)
        sys.exit(2)
    npy_path, traj_path = Path(sys.argv[1]), Path(sys.argv[2])
    points = np.load(npy_path, mmap_mode='r')
    trajectory = json.loads(traj_path.read_text(encoding='utf-8'))['keyframes']
    if len(trajectory) < 3:
        print(f'关键帧不足: {len(trajectory)}', file=sys.stderr)
        sys.exit(1)
    poses, descriptors = [], []
    for kf in trajectory:
        t_map_sensor = pose_matrix(kf)
        t_sensor_map = np.linalg.inv(t_map_sensor)
        d = np.linalg.norm(points[:, :2] - np.array([kf['x'], kf['y']]), axis=1)
        local_global = points[d <= CROP_RADIUS]
        if len(local_global) < 100:
            continue
        # to the keyframe's sensor frame, then to the descriptor
        # np.dot (not @): the matmul operator triggers spurious
        # divide-by-zero warnings on macOS Accelerate numpy 2.0.2
        local = (np.dot(np.ascontiguousarray(local_global[:, :3], dtype=np.float64),
                        t_sensor_map[:3, :3].T) + t_sensor_map[:3, 3])
        sc = make_descriptor_height(local, sensor_z=0.0)
        q = rotation_to_quat(t_map_sensor[:3, :3])
        poses.append([kf['x'], kf['y'], kf['z'], q[0], q[1], q[2], q[3], 0.0])
        descriptors.append(sc.reshape(-1))
    if len(poses) < 3:
        print(f'有效关键帧不足: {len(poses)}', file=sys.stderr)
        sys.exit(1)
    out = npy_path.parent / 'db.npz'
    np.savez_compressed(out, poses=np.array(poses, dtype=np.float32),
                        descriptors=np.array(descriptors, dtype=np.uint8),
                        sc_shape=np.array([NUM_RINGS, NUM_SECTORS]))
    print(f'db built: {out} keyframes={len(poses)}')


def rotation_to_quat(r):
    """Rotation matrix -> quaternion (x, y, z, w)."""
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


if __name__ == '__main__':
    main()

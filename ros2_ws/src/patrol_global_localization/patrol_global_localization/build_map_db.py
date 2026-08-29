#!/usr/bin/env python3
"""Build the Scan Context database for a saved map.

Usage: build_map_db <map_name> <pcd_path> <trajectory_json>

Crops a local submap around each trajectory keyframe from the saved PCD,
computes the Scan Context descriptor, and writes maps/<map_name>/db.npz with
keyframe poses (map frame) + descriptors. The localization manager later
loads the full PCD once and crops candidate submaps on demand for GICP.
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from patrol_global_localization.scan_context import make_descriptor_height  # noqa: E402

VOXEL = 0.15
CROP_RADIUS = 12.0


def load_pcd(path):
    """Minimal PCD reader: ascii and binary (uncompressed) x/y/z float32."""
    data = Path(path).read_bytes()
    end = data.index(b'data ') + len(b'data ')
    eol = data.index(b'\n', end)
    header = data[:eol].decode('ascii', errors='replace').splitlines()
    fields, count, size = [], 0, 0
    for line in header:
        parts = line.split()
        if not parts:
            continue
        if parts[0] == 'FIELDS':
            fields = parts[1:]
        elif parts[0] == 'POINTS':
            count = int(parts[1])
        elif parts[0] == 'SIZE' and 'float32' in ' '.join(header):
            size = sum(4 for f in fields) * count
    body = data[eol + 1:]
    if 'binary' in header[-2] + header[-1] or b'\n' not in body[:20] and False:
        pass
    # locate the body start using the header line count (robust enough for
    # PCDs written by our own bridge and pcl converters)
    text_end = data.index(b'DATA ')
    data_kind = data[text_end:text_end + 20].split()[1].decode()
    if data_kind == 'ascii':
        rows = np.loadtxt(body.splitlines()[:count], usecols=(0, 1, 2))
        return rows.reshape(-1, 3)
    offsets = {'x': 0}
    float_count = sum(1 for f in fields if f in ('x', 'y', 'z', 'intensity'))
    arr = np.frombuffer(body[:count * 4 * float_count], dtype=np.float32)
    return arr.reshape(-1, float_count)[:, :3].copy()


def voxel_downsample(points, voxel=VOXEL):
    if not len(points):
        return points
    keys = np.floor(points[:, :3] / voxel).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(index)]


def main():
    map_name, pcd_path, traj_path = sys.argv[1], Path(sys.argv[2]), Path(sys.argv[3])
    points = voxel_downsample(load_pcd(pcd_path))
    trajectory = json.loads(traj_path.read_text(encoding='utf-8'))['keyframes']
    poses, descriptors = [], []
    for kf in trajectory:
        d = np.hypot(points[:, 0] - kf['x'], points[:, 1] - kf['y'])
        local = points[d <= CROP_RADIUS]
        sc = make_descriptor_height(local, sensor_z=kf['z'])
        poses.append([kf['x'], kf['y'], kf['z'], kf['yaw']])
        descriptors.append(sc.reshape(-1))
    out = pcd_path.parent / map_name / 'db.npz'
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, poses=np.array(poses, dtype=np.float32),
                        descriptors=np.array(descriptors, dtype=np.uint8),
                        sc_shape=np.array([80, 20]))
    print(f'db built: {out} keyframes={len(poses)} points={len(points)}')


if __name__ == '__main__':
    main()

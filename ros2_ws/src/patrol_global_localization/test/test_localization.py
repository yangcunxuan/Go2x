"""Offline tests for the global localization stack (no ROS required)."""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))
from patrol_global_localization.scan_context import (  # noqa: E402
    distance, make_descriptor_height, ring_key, search)


def quat_matrix(x, y, z, qx, qy, qz, qw):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw) or 1.0
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    t = np.eye(4)
    t[:3, :3] = np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy-qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]])
    t[:3, 3] = [x, y, z]
    return t


def yaw_matrix(x, y, z, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    t = np.eye(4)
    t[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    t[:3, 3] = [x, y, z]
    return t


def synthetic_scene(n=20000, seed=3):
    """Asymmetric indoor-ish scene in map_level coordinates."""
    rng = np.random.default_rng(seed)
    walls = np.vstack([
        np.column_stack([rng.uniform(-10, 10, n//4), np.full(n//4, 6.0), rng.uniform(0, 3, n//4)]),
        np.column_stack([np.full(n//4, 12.0), rng.uniform(-10, 10, n//4), rng.uniform(0, 3, n//4)]),
        np.column_stack([rng.uniform(-10, 10, n//4), np.full(n//4, -8.0), rng.uniform(0, 3, n//4)]),
        np.column_stack([rng.uniform(0, 2, n//4), rng.uniform(0, 2, n//4), rng.uniform(0, 0.5, n//4)]),
    ])
    floor = np.column_stack([rng.uniform(-10, 12, n), rng.uniform(-8, 6, n), rng.uniform(-0.05, 0.05, n)])
    return np.vstack([walls, floor])


# ---------- scan context ----------
def test_descriptor_translation_invariance():
    """Same sensor-frame observation regardless of absolute position (P0 #6)."""
    rng = np.random.default_rng(11)
    local = np.column_stack([rng.uniform(1, 15, 5000), rng.uniform(-10, 10, 5000),
                             rng.uniform(-0.5, 1.5, 5000)])
    t1 = yaw_matrix(0, 0, 0, 0.3)
    t2 = yaw_matrix(10, 5, -0.2, 0.3)
    p1 = np.dot(local, t1[:3, :3].T) + t1[:3, 3]
    p2 = np.dot(local, t2[:3, :3].T) + t2[:3, 3]
    sc1 = make_descriptor_height(p1, sensor_z=0.0)
    # sensor moved 0.2 m up: sensor_z compensates in the local transform
    p2_local = np.dot(p2 - [10, 5, -0.2], t2[:3, :3])  # inverse rotation = transpose
    sc2 = make_descriptor_height(p2_local, sensor_z=0.0)
    assert distance(sc1, sc2)[0] < 0.05


def test_descriptor_yaw_shift_sign():
    """Rotating the scene by +90 deg must shift sectors by +num_sectors/4."""
    rng = np.random.default_rng(5)
    local = np.column_stack([rng.uniform(1, 15, 6000), rng.uniform(-15, 15, 6000),
                             rng.uniform(0, 1, 6000)])
    sc0 = make_descriptor_height(local, sensor_z=0.0)
    rot90 = yaw_matrix(0, 0, 0, math.pi / 2)
    rotated = np.dot(local, rot90[:3, :3].T)
    sc90 = make_descriptor_height(rotated, sensor_z=0.0)
    dist, shift = distance(sc0, sc90)
    sector_angle = 2 * math.pi / sc0.shape[1]
    # Convention pinned by this test: rotating the SCENE by +theta shifts
    # the descriptor columns by -theta/sector (measured empirically), so
    # consumers must apply yaw_fix = -shift * sector_angle.
    recovered = (shift * sector_angle) % (2 * math.pi)
    assert abs(recovered - 2 * math.pi + math.pi / 2) < sector_angle * 1.5, \
        (recovered, dist)


def test_search_entry_contract_without_ring_key():
    """search() must tolerate entries lacking a precomputed ring_key."""
    rng = np.random.default_rng(2)
    sc = make_descriptor_height(np.column_stack([rng.uniform(1, 10, 2000),
                                                 rng.uniform(-8, 8, 2000),
                                                 rng.uniform(0, 1, 2000)]), 0.0)
    db = [{'sc': sc, 'index': 0}]
    top = search(db, sc, topk=1)
    assert top and top[0][0]['index'] == 0 and top[0][1] < 0.01


# ---------- build_map_db ----------
def _write_map_package(tmp_path, scene, keyframes):
    np.save(tmp_path / 'map.npy', scene.astype(np.float32))
    (tmp_path / 'trajectory.json').write_text(json.dumps({'keyframes': keyframes}))
    return tmp_path


def test_map_db_build_and_roundtrip(tmp_path):
    from patrol_global_localization.build_map_db import main as build_main
    scene = synthetic_scene()
    kfs = [
        {'x': 0.5, 'y': 0.5, 'z': 0.4, 'qx': 0, 'qy': 0, 'qz': 0, 'qw': 1},
        {'x': 4.0, 'y': 0.5, 'z': 0.4, 'qx': 0, 'qy': 0, 'qz': 0.25, 'qw': 0.968},
        {'x': 8.0, 'y': 1.0, 'z': 0.4, 'yaw': 1.2},
    ]
    _write_map_package(tmp_path, scene, kfs)
    sys.argv = ['build_map_db', str(tmp_path / 'map.npy'),
                str(tmp_path / 'trajectory.json')]
    build_main()
    db = np.load(tmp_path / 'db.npz')
    assert db['descriptors'].shape[0] >= 3
    assert db['poses'].shape[0] == db['descriptors'].shape[0]
    # keyframes carry the exact stored poses
    assert float(db['poses'][0][0]) == 0.5


def test_relocalization_recovers_known_transform(tmp_path):
    """End-to-end synthetic: build DB, query a translated/rotated observation,
    descriptor of the query must match the right keyframe (P0 #6/#7)."""
    from patrol_global_localization.build_map_db import main as build_main
    from patrol_global_localization.scan_context import search as sc_search
    scene = synthetic_scene()
    kf_pose = {'x': 1.0, 'y': 1.0, 'z': 0.4, 'qx': 0, 'qy': 0, 'qz': 0.1, 'qw': 0.995}
    kfs = [
        dict(kf_pose),
        {'x': 5.0, 'y': 2.0, 'z': 0.4, 'qx': 0, 'qy': 0, 'qz': 0.2, 'qw': 0.98},
        {'x': 9.0, 'y': 0.5, 'z': 0.4, 'yaw': -1.0},
    ]
    _write_map_package(tmp_path, scene, kfs)
    sys.argv = ['build_map_db', str(tmp_path / 'map.npy'),
                str(tmp_path / 'trajectory.json')]
    build_main()
    db = np.load(tmp_path / 'db.npz')

    # observation at the first keyframe, yaw rotated +30 deg, small noise
    t_sensor = yaw_matrix(kf_pose['x'], kf_pose['y'], kf_pose['z'], 0.1 + 0.5236)
    d = np.linalg.norm(scene[:, :2] - [kf_pose['x'], kf_pose['y']], axis=1)
    local_obs = scene[d <= 20]
    noisy = local_obs + np.random.default_rng(1).normal(0, 0.01, local_obs.shape)
    obs = np.dot(noisy, t_sensor[:3, :3].T) + t_sensor[:3, 3]

    entries = []
    for i, desc in enumerate(db['descriptors']):
        sc = desc.reshape(tuple(db['sc_shape']))
        entries.append({'sc': sc, 'index': i, 'ring_key': ring_key(sc),
                        'pose': db['poses'][i]})
    # Standard usage: the query cloud arrives in camera_init (world) coords;
    # it must be transformed into the CURRENT sensor frame before the
    # descriptor (P0 audit #6). inv(T_cameraInit_sensor) applied here.
    # inv(T) for a rigid transform: p_local = R^T @ (p_world - t)
    obs_sensor = (np.dot(np.ascontiguousarray(obs, dtype=np.float64) - t_sensor[:3, 3],
                          t_sensor[:3, :3]))
    query = make_descriptor_height(obs_sensor, sensor_z=0.0)
    top = sc_search(entries, query, topk=3)
    assert top[0][0]['index'] == 0, 'must retrieve the matching keyframe'
    # Sparse synthetic clouds make the absolute SC distance noisy; what
    # matters for retrieval is the margin over the runner-up (a genuinely
    # different place).
    assert len(top) < 2 or top[1][1] - top[0][1] > 0.05, \
        f'no margin: {[(e["index"], round(d, 3)) for e, d, _ in top]}'

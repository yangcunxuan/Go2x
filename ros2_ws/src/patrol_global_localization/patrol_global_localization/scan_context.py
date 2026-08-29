#!/usr/bin/env python3
"""Scan Context place descriptor (Kim & Kim, IROS 2018), numpy-only.

Self-contained implementation so the localization stack needs no external
place-recognition dependency. Standard parameters: 80 rings x 20 sectors,
20 m max range. Matching uses the ring-key (row-wise mean) circular shift
search, which gives yaw-invariance up to the sector resolution.
"""
import numpy as np

NUM_RINGS = 80
NUM_SECTORS = 20
MAX_RANGE = 20.0


def make_descriptor(points_xy, num_rings=NUM_RINGS, num_sectors=NUM_SECTORS,
                    max_range=MAX_RANGE):
    """points_xy: (N, 2) sensor-frame points. Returns (num_rings, num_sectors) uint8."""
    sc = np.zeros((num_rings, num_sectors), dtype=np.uint8)
    if len(points_xy) == 0:
        return sc
    d = np.hypot(points_xy[:, 0], points_xy[:, 1])
    a = np.arctan2(points_xy[:, 1], points_xy[:, 0])
    valid = (d > 0.01) & (d <= max_range)
    if not valid.any():
        return sc
    d, a = d[valid], a[valid]
    ring = np.minimum((d / max_range * num_rings).astype(int), num_rings - 1)
    sector = np.minimum(((a + np.pi) / (2 * np.pi) * num_sectors).astype(int), num_sectors - 1)
    idx = ring * num_sectors + sector
    max_z = np.zeros(num_rings * num_sectors)
    # Scan Context encodes the max ring-normalized height; caller passes the
    # per-point "height above sensor" via points_xy[:, 2] when available.
    np.maximum.at(max_z, idx, np.ones(len(d)))
    sc = max_z.reshape(num_rings, num_sectors).astype(np.uint8)
    return sc


def make_descriptor_height(points_xyz, sensor_z, num_rings=NUM_RINGS,
                           num_sectors=NUM_SECTORS, max_range=MAX_RANGE):
    """(N,3) world points + sensor height: encodes max height above sensor
    per cell, quantized to 0..255 over a 2 m window (standard Scan Context)."""
    sc = np.zeros((num_rings, num_sectors), dtype=np.uint8)
    if len(points_xyz) == 0:
        return sc
    rel = points_xyz[:, :2]
    dz = points_xyz[:, 2] - sensor_z
    d = np.hypot(rel[:, 0], rel[:, 1])
    a = np.arctan2(rel[:, 1], rel[:, 0])
    valid = (d > 0.01) & (d <= max_range) & (dz > -2.0) & (dz < 2.0)
    if not valid.any():
        return sc
    d, a, dz = d[valid], a[valid], dz[valid]
    ring = np.minimum((d / max_range * num_rings).astype(int), num_rings - 1)
    sector = np.minimum(((a + np.pi) / (2 * np.pi) * num_sectors).astype(int), num_sectors - 1)
    idx = ring * num_sectors + sector
    best = np.full(num_rings * num_sectors, -2.0)
    np.maximum.at(best, idx, dz)
    sc = np.clip(np.round((best + 2.0) / 4.0 * 255.0), 0, 255).reshape(num_rings, num_sectors).astype(np.uint8)
    return sc


def ring_key(sc):
    """Row-wise mean: shift-invariant fingerprint used for coarse matching."""
    return sc.mean(axis=1)


def distance(sc1, sc2):
    """Sector-shift-invariant distance in [0, 1]: search all column shifts.
    Descriptor cells are uint8 (0..255), so the mean absolute difference is
    normalized by 255."""
    if sc1.size == 0 or sc2.size == 0:
        return 1.0
    num_sectors = sc1.shape[1]
    best = 1.0
    best_shift = 0
    for shift in range(num_sectors):
        cand = np.roll(sc2, shift, axis=1)
        diff = np.abs(sc1.astype(np.float32) - cand.astype(np.float32))
        valid = (sc1 > 0) | (cand > 0)
        d = float(diff[valid].mean()) / 255.0 if valid.any() else 1.0
        if d < best:
            best = d
            best_shift = shift
    return best, best_shift


def search(database, query_sc, topk=5):
    """database: list of dicts with 'sc' and index metadata. Returns the top-k
    (entry, distance, shift) candidates ranked by ring-key then exact distance."""
    if not len(database):
        return []
    query_ring = ring_key(query_sc)
    # Coarse pass: circular cross-correlation of ring keys via FFT.
    coarse = []
    for entry in database:
        rk = entry.get('ring_key')
        if rk is None:  # defensive: callers may pass bare descriptors
            rk = ring_key(entry['sc'])
            entry['ring_key'] = rk
        corr = np.fft.irfft(np.fft.rfft(query_ring) * np.conj(np.fft.rfft(rk)), len(rk))
        coarse.append(float(corr.max()))
    order = np.argsort(coarse)[::-1][:max(topk * 4, 20)]
    results = []
    for i in order:
        d, shift = distance(query_sc, database[i]['sc'])
        results.append((database[i], d, shift))
    results.sort(key=lambda item: item[1])
    return results[:topk]
